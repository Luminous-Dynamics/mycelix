use scientific_policy_metamorphic::role_index::generate_indexed_role_corpus;
use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

fn run() -> Result<(), String> {
    let mut args = env::args_os();
    let program = args.next().unwrap_or_default();
    let Some(source_path) = args.next() else {
        return Err(format!(
            "usage: {} <source-engine-inputs.jsonl> <expected-source-sha256> <output-directory>",
            program.to_string_lossy()
        ));
    };
    let Some(expected_sha256) = args.next() else {
        return Err("expected source SHA-256 is required".to_owned());
    };
    let Some(output_directory) = args.next() else {
        return Err("expected output directory is required".to_owned());
    };
    if args.next().is_some() {
        return Err("expected exactly three arguments".to_owned());
    }
    let expected_sha256 = expected_sha256
        .to_str()
        .ok_or_else(|| "expected source SHA-256 must be UTF-8".to_owned())?;
    let manifest = generate_indexed_role_corpus(
        PathBuf::from(source_path).as_path(),
        expected_sha256,
        PathBuf::from(output_directory).as_path(),
    )
    .map_err(|error| error.to_string())?;
    println!("projection_profile={}", manifest.projection_profile);
    println!("case_count={}", manifest.indexed_case_count);
    println!("source_sha256={}", manifest.source_engine_inputs_sha256);
    println!("indexed_sha256={}", manifest.indexed_engine_inputs_sha256);
    println!("action_count={}", manifest.action_count);
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("indexed role projection failed: {error}");
            ExitCode::from(2)
        }
    }
}

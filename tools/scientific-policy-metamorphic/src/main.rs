use scientific_policy_metamorphic::generate_variants;
use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

fn run() -> Result<ExitCode, String> {
    let mut args = env::args_os();
    let program = args.next().unwrap_or_default();
    let Some(manifest_path) = args.next() else {
        eprintln!(
            "usage: {} <scientific-policy-generator-manifest.json> <output-directory>",
            program.to_string_lossy()
        );
        return Ok(ExitCode::from(64));
    };
    let Some(output_directory) = args.next() else {
        eprintln!("expected an output directory");
        return Ok(ExitCode::from(64));
    };
    if args.next().is_some() {
        return Err("expected exactly two arguments".to_owned());
    }

    let summaries = generate_variants(
        PathBuf::from(manifest_path).as_path(),
        PathBuf::from(output_directory).as_path(),
    )
    .map_err(|error| error.to_string())?;

    for summary in &summaries {
        println!(
            "transform={} cases={} source_sha256={} transformed_sha256={} output={}",
            summary.transform_id.as_str(),
            summary.case_count,
            summary.source_sha256,
            summary.transformed_sha256,
            summary.output_directory
        );
    }
    println!("transform_count={}", summaries.len());
    Ok(ExitCode::SUCCESS)
}

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(error) => {
            eprintln!("metamorphic corpus generation failed: {error}");
            ExitCode::from(2)
        }
    }
}

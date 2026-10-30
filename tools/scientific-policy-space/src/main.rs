use scientific_policy_space::{build_manifest, enumerate_decision_space, project_engine_inputs};
use sha2::{Digest, Sha256};
use std::env;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::process::ExitCode;

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn run() -> Result<ExitCode, String> {
    let mut args = env::args_os();
    let program = args.next().unwrap_or_default();
    let Some(output_directory) = args.next() else {
        eprintln!("usage: {} <output-directory>", program.to_string_lossy());
        return Ok(ExitCode::from(64));
    };
    if args.next().is_some() {
        eprintln!("expected exactly one output directory");
        return Ok(ExitCode::from(64));
    }

    let cases = enumerate_decision_space();
    let manifest = build_manifest(&cases);
    if manifest.case_count != 62_464 {
        return Err(format!("unexpected generated case count: {}", manifest.case_count));
    }

    let output_directory = PathBuf::from(output_directory);
    fs::create_dir_all(&output_directory)
        .map_err(|error| format!("create output directory: {error}"))?;
    let cases_path = output_directory.join("scientific-policy-decision-space-v0.1.jsonl");
    let file = File::create(&cases_path).map_err(|error| format!("create corpus: {error}"))?;
    let mut writer = BufWriter::new(file);
    let mut hasher = Sha256::new();

    for case in &cases {
        let mut row = serde_json::to_vec(case)
            .map_err(|error| format!("serialize generated case: {error}"))?;
        row.push(b'\n');
        writer
            .write_all(&row)
            .map_err(|error| format!("write generated corpus: {error}"))?;
        hasher.update(&row);
    }
    writer.flush().map_err(|error| format!("flush generated corpus: {error}"))?;
    let corpus_sha256 = hex::encode(hasher.finalize());

    let engine_inputs_path = output_directory.join("scientific-policy-engine-inputs-v0.1.jsonl");
    let engine_file = File::create(&engine_inputs_path)
        .map_err(|error| format!("create engine input corpus: {error}"))?;
    let mut engine_writer = BufWriter::new(engine_file);
    let mut engine_hasher = Sha256::new();
    for case in &cases {
        let mut row = serde_json::to_vec(&project_engine_inputs(case))
            .map_err(|error| format!("serialize projected engine inputs: {error}"))?;
        row.push(b'\\n');
        engine_writer
            .write_all(&row)
            .map_err(|error| format!("write projected engine inputs: {error}"))?;
        engine_hasher.update(&row);
    }
    engine_writer.flush()
        .map_err(|error| format!("flush projected engine inputs: {error}"))?;
    let engine_inputs_sha256 = hex::encode(engine_hasher.finalize());

    #[derive(serde::Serialize)]
    struct OutputManifest {
        #[serde(flatten)]
        corpus: scientific_policy_space::CorpusManifest,
        corpus_file: String,
        corpus_sha256: String,
        engine_inputs_file: String,
        engine_inputs_sha256: String,
        engine_input_count: usize,
        engine_input_format: &'static str,
        encoding: &'static str,
    }

    let output_manifest = OutputManifest {
        corpus: manifest,
        corpus_file: "scientific-policy-decision-space-v0.1.jsonl".to_owned(),
        corpus_sha256: corpus_sha256.clone(),
        engine_inputs_file: "scientific-policy-engine-inputs-v0.1.jsonl".to_owned(),
        engine_inputs_sha256: engine_inputs_sha256.clone(),
        engine_input_count: cases.len(),
        engine_input_format: "case_id+cedar_native_request+rego_input",
        encoding: "utf-8-jsonl-one-record-per-line",
    };
    let manifest_bytes = serde_json::to_vec_pretty(&output_manifest)
        .map_err(|error| format!("serialize corpus manifest: {error}"))?;
    fs::write(output_directory.join("manifest.json"), &manifest_bytes)
        .map_err(|error| format!("write corpus manifest: {error}"))?;

    println!("case_count=62464");
    println!("corpus_sha256={corpus_sha256}");
    println!("engine_inputs_sha256={engine_inputs_sha256}");
    println!("engine_input_count={}", cases.len());
    println!("manifest_sha256={}", sha256(&manifest_bytes));
    println!("expected_decisions=pending_authentic_rust_oracle");
    Ok(ExitCode::SUCCESS)
}

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::from(66)
        }
    }
}

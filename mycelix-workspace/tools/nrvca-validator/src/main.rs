use nrvca_validator::{run_adversarial_corpus, validate_report, validate_schema_contract};
use serde_json::Value;
use std::{env, fs, process::ExitCode};

fn read_json(path: &str) -> Result<Value, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("cannot read {path}: {e}"))?;
    serde_json::from_str(&text).map_err(|e| format!("invalid JSON in {path}: {e}"))
}

fn run() -> Result<(), String> {
    let mut fixture = "fixtures/nrvca-synthetic-golden-v0.1.json".to_owned();
    let mut corpus = "fixtures/nrvca-adversarial-corpus-v0.1.json".to_owned();
    let mut schema = "schemas/nrvca-result-v0.1.schema.json".to_owned();
    let mut mode = "all".to_owned();

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        let target = match arg.as_str() {
            "--fixture" => Some(&mut fixture),
            "--corpus" => Some(&mut corpus),
            "--schema" => Some(&mut schema),
            "--mode" => Some(&mut mode),
            "--help" | "-h" => {
                println!("Usage: cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- [--fixture PATH] [--corpus PATH] [--schema PATH] [--mode all|validate|corpus]");
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}")),
        };
        let Some(slot) = target else { continue };
        *slot = args.next().ok_or_else(|| format!("missing value after {arg}"))?;
    }

    if !["all", "validate", "corpus"].contains(&mode.as_str()) {
        return Err(format!("unsupported mode: {mode}"));
    }

    if mode == "all" || mode == "validate" {
        let schema_json = read_json(&schema)?;
        let schema_errors = validate_schema_contract(&schema_json);
        if !schema_errors.is_empty() {
            for d in &schema_errors { eprintln!("{} {}: {}", d.code, d.path, d.message); }
            return Err(format!("schema contract check failed with {} diagnostic(s)", schema_errors.len()));
        }

        let report = read_json(&fixture)?;
        let diagnostics = validate_report(&report);
        if diagnostics.is_empty() {
            println!("REPORT PASS: {}", fixture);
        } else {
            for d in &diagnostics { eprintln!("{} {}: {}", d.code, d.path, d.message); }
            return Err(format!("report validation failed with {} diagnostic(s)", diagnostics.len()));
        }
    }

    if mode == "all" || mode == "corpus" {
        let report = read_json(&fixture)?;
        let adversarial = read_json(&corpus)?;
        let results = run_adversarial_corpus(&report, &adversarial)?;
        for result in &results {
            println!(
                "{} {} expected={} codes={}",
                if result.passed { "PASS" } else { "FAIL" },
                result.case_id,
                result.expected_disposition,
                result.diagnostic_codes.join(",")
            );
        }
        let failures = results.iter().filter(|r| !r.passed).count();
        println!("CORPUS: {}/{} mutations rejected in expected diagnostic class", results.len() - failures, results.len());
        if failures > 0 { return Err(format!("{failures} adversarial case(s) not rejected as expected")); }
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("NRVCA validator error: {error}");
            ExitCode::FAILURE
        }
    }
}

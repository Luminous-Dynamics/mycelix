use nrvca_validator::{run_adversarial_corpus, validate_benchmark_preregistration, validate_benchmark_schema_contract, validate_report, validate_schema_contract};
use serde_json::Value;
use std::{env, fs, process::ExitCode};

fn read_json(path: &str) -> Result<Value, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("cannot read {path}: {e}"))?;
    serde_json::from_str(&text).map_err(|e| format!("invalid JSON in {path}: {e}"))
}

fn run() -> Result<(), String> {
    let all_args: Vec<String> = env::args().skip(1).collect();
    if all_args.first().map(String::as_str) == Some("verify-store") {
        if all_args.len() == 2 && matches!(all_args[1].as_str(), "--help" | "-h") {
            println!("{}", nrvca_validator::source_intake::VERIFY_STORE_USAGE);
            return Ok(());
        }
        let report = nrvca_validator::source_intake::verify_store_from_cli(&all_args[1..])?;
        println!(
            "{}",
            serde_json::to_string_pretty(&report)
                .map_err(|error| format!("cannot format store audit report: {error}"))?
        );
        match report["status"].as_str() {
            Some("failed") => return Err(format!(
                "content store integrity audit failed with {} diagnostic(s)",
                report["diagnostics"].as_array().map_or(0, Vec::len)
            )),
            Some("empty") => return Err("content store is empty; no evidence objects were verified".to_owned()),
            _ => {}
        }
        return Ok(());
    }
    if all_args.first().map(String::as_str) == Some("capture-source") {
        if all_args.len() == 2 && matches!(all_args[1].as_str(), "--help" | "-h") {
            println!("{}", nrvca_validator::source_intake::CAPTURE_SOURCE_USAGE);
            return Ok(());
        }
        let output = nrvca_validator::source_intake::capture_from_cli(&all_args[1..])?;
        if let Some(path) = &output.output_path {
            nrvca_validator::source_intake::write_receipt_new(path, &output.receipt)?;
            println!(
                "SOURCE CAPTURED: {} sha256={}",
                path.display(),
                output.receipt["sha256"].as_str().unwrap_or("missing")
            );
        } else {
            println!(
                "{}",
                serde_json::to_string_pretty(&output.receipt)
                    .map_err(|error| format!("cannot format capture receipt: {error}"))?
            );
        }
        return Ok(());
    }
    if all_args.first().map(String::as_str) == Some("hash-source") {
        if all_args.len() == 2 && matches!(all_args[1].as_str(), "--help" | "-h") {
            println!("{}", nrvca_validator::source_intake::HASH_SOURCE_USAGE);
            return Ok(());
        }
        let output = nrvca_validator::source_intake::receipt_from_cli(&all_args[1..])?;
        if let Some(path) = &output.output_path {
            nrvca_validator::source_intake::write_receipt_new(path, &output.receipt)?;
            println!(
                "SOURCE RECEIPT WRITTEN: {} sha256={}",
                path.display(),
                output.receipt["sha256"].as_str().unwrap_or("missing")
            );
        } else {
            println!(
                "{}",
                serde_json::to_string_pretty(&output.receipt)
                    .map_err(|error| format!("cannot format source receipt: {error}"))?
            );
        }
        return Ok(());
    }

    let mut fixture = "fixtures/nrvca-synthetic-golden-v0.1.json".to_owned();
    let mut corpus = "fixtures/nrvca-adversarial-corpus-v0.1.json".to_owned();
    let mut schema = "schemas/nrvca-result-v0.1.schema.json".to_owned();
    let mut benchmark = "fixtures/nrvca-sa-bridge-preregistration-v0.1.json".to_owned();
    let mut benchmark_schema = "schemas/nrvca-benchmark-preregistration-v0.1.schema.json".to_owned();
    let mut mode = "all".to_owned();

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        let target = match arg.as_str() {
            "--fixture" => Some(&mut fixture),
            "--corpus" => Some(&mut corpus),
            "--schema" => Some(&mut schema),
            "--benchmark" => Some(&mut benchmark),
            "--benchmark-schema" => Some(&mut benchmark_schema),
            "--mode" => Some(&mut mode),
            "--help" | "-h" => {
                println!("Usage: cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- [--fixture PATH] [--corpus PATH] [--schema PATH] [--benchmark PATH] [--benchmark-schema PATH] [--mode all|validate|corpus]");
                println!("       cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- hash-source --file PATH --source-id ID --publisher NAME --source-uri URI --publication-id ID --vintage VINTAGE --captured-at RFC3339 --rights-status STATUS --rights-evidence-uri URI --evaluation-allowed true|false --redistribution-allowed true|false --locator-id ID --locator TEXT --extraction-method METHOD --tool-version VERSION [--expected-sha256 HEX] [--output PATH]");
                println!("       cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- capture-source --file PATH --store-dir DIR --source-id ID --publisher NAME --source-uri URI --publication-id ID --vintage VINTAGE --captured-at RFC3339 --rights-status STATUS --rights-evidence-uri URI --evaluation-allowed true|false --redistribution-allowed true|false --locator-id ID --locator TEXT --extraction-method METHOD --tool-version VERSION [--expected-sha256 HEX] [--output PATH]");
                println!("       cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- verify-store --store-dir DIR");
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

        let benchmark_schema_json = read_json(&benchmark_schema)?;
        let benchmark_schema_errors = validate_benchmark_schema_contract(&benchmark_schema_json);
        if !benchmark_schema_errors.is_empty() {
            for d in &benchmark_schema_errors { eprintln!("{} {}: {}", d.code, d.path, d.message); }
            return Err(format!("benchmark schema contract check failed with {} diagnostic(s)", benchmark_schema_errors.len()));
        }

        let report = read_json(&fixture)?;
        let diagnostics = validate_report(&report);
        if diagnostics.is_empty() {
            println!("REPORT PASS: {}", fixture);
        } else {
            for d in &diagnostics { eprintln!("{} {}: {}", d.code, d.path, d.message); }
            return Err(format!("report validation failed with {} diagnostic(s)", diagnostics.len()));
        }

        let benchmark_record = read_json(&benchmark)?;
        let benchmark_diagnostics = validate_benchmark_preregistration(&benchmark_record);
        if !benchmark_diagnostics.is_empty() {
            for d in &benchmark_diagnostics { eprintln!("{} {}: {}", d.code, d.path, d.message); }
            return Err(format!("benchmark preregistration invalid with {} diagnostic(s)", benchmark_diagnostics.len()));
        }
        println!("BENCHMARK PREREGISTRATION PASS: {} (execution_state={})", benchmark, benchmark_record["execution_state"].as_str().unwrap_or("unknown"));
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

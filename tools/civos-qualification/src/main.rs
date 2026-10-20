use civos_qualification::{evaluate_receipt, validate_contract};
use serde_json::json;
use std::{env, fs, process};

fn value_after(args: &[String], flag: &str) -> Result<String, String> {
    let index = args.iter().position(|arg| arg == flag)
        .ok_or_else(|| format!("missing required option {flag}"))?;
    args.get(index + 1)
        .filter(|value| !value.starts_with("--"))
        .cloned()
        .ok_or_else(|| format!("option {flag} requires a value"))
}

fn read(path: &str, label: &str) -> Result<String, String> {
    fs::read_to_string(path).map_err(|error| format!("cannot read {label} at {path}: {error}"))
}

fn run(args: &[String]) -> Result<i32, String> {
    let Some(command) = args.first().map(String::as_str) else {
        return Err(usage().into());
    };
    match command {
        "validate-contract" => {
            let corpus_path = value_after(args, "--corpus")?;
            let schema_path = value_after(args, "--schema")?;
            let corpus = read(&corpus_path, "scenario corpus")?;
            let schema = read(&schema_path, "scenario schema")?;
            let summary = validate_contract(&corpus, &schema)?;
            println!("{}", serde_json::to_string_pretty(&json!({
                "status":"CONTRACT_VALID",
                "contract_id":summary.contract_id,
                "version":summary.version,
                "scenario_count":summary.scenario_count,
                "hard_gate_count":summary.hard_gate_count,
                "behavioral_count":summary.behavioral_count,
                "performance_baseline_count":summary.performance_baseline_count,
                "corpus_sha256":summary.corpus_sha256,
                "schema_sha256":summary.schema_sha256
            })).map_err(|error| error.to_string())?);
            Ok(0)
        }
        "evaluate-receipt" => {
            let corpus_path = value_after(args, "--corpus")?;
            let schema_path = value_after(args, "--schema")?;
            let receipt_schema_path = value_after(args, "--receipt-schema")?;
            let receipt_path = value_after(args, "--receipt")?;
            let target = value_after(args, "--target")?;
            let corpus = read(&corpus_path, "scenario corpus")?;
            let schema = read(&schema_path, "scenario schema")?;
            let receipt_schema = read(&receipt_schema_path, "receipt schema")?;
            let receipt = read(&receipt_path, "qualification receipt")?;
            let report = evaluate_receipt(
                &corpus, &schema, &receipt_schema, &receipt, &target
            )?;
            let status = report.get("status").and_then(|value| value.as_str()).unwrap_or("INCOMPLETE");
            println!("{}", serde_json::to_string_pretty(&report).map_err(|error| error.to_string())?);
            Ok(match status {
                "QUALIFIED" => 0,
                "FAILED" => 1,
                _ => 2,
            })
        }
        "help" | "--help" | "-h" => {
            println!("{}", usage());
            Ok(0)
        }
        other => Err(format!("unknown command {other}\n{}", usage())),
    }
}

fn usage() -> &'static str {
    "CivOS fail-closed qualification tool\n\n\
civos-qualify validate-contract --corpus PATH --schema PATH\n\
civos-qualify evaluate-receipt --corpus PATH --schema PATH --receipt-schema PATH --receipt PATH --target TARGET\n\n\
Exit codes: 0 qualified/valid, 1 hard-gate failure, 2 incomplete or invalid input."
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    match run(&args) {
        Ok(code) => process::exit(code),
        Err(error) => {
            eprintln!("civos-qualify: {error}");
            process::exit(2);
        }
    }
}

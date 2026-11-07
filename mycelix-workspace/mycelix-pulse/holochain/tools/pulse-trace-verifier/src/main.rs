use pulse_trace_verifier::{parse_and_validate_corpus, verify_trace};
use serde_json::Value;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

fn main() -> ExitCode {
    match run() {
        Ok(report) => {
            println!("{}", serde_json::to_string_pretty(&report).expect("receipt is serializable"));
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<Value, String> {
    let mut args = env::args().skip(1);
    let command = args.next().ok_or_else(usage)?;
    let corpus_path = PathBuf::from(args.next().ok_or_else(usage)?);
    let schema_path = PathBuf::from(args.next().ok_or_else(usage)?);
    let corpus = fs::read(&corpus_path).map_err(|error| format!("cannot read {}: {error}", corpus_path.display()))?;
    let schema = fs::read(&schema_path).map_err(|error| format!("cannot read {}: {error}", schema_path.display()))?;

    match command.as_str() {
        "validate-corpus" => {
            let (_, receipt) = parse_and_validate_corpus(&corpus, &schema)
                .map_err(format_violations)?;
            Ok(receipt)
        }
        "verify-trace" => {
            let trace_schema_path = PathBuf::from(args.next().ok_or_else(usage)?);
            let trace_schema = fs::read(&trace_schema_path)
                .map_err(|error| format!("cannot read {}: {error}", trace_schema_path.display()))?;
            let trace_path = PathBuf::from(args.next().ok_or_else(usage)?);
            let trace = fs::read(&trace_path)
                .map_err(|error| format!("cannot read {}: {error}", trace_path.display()))?;
            let receipt = verify_trace(&corpus, &schema, &trace_schema, &trace)
                .map_err(format_violations)?;
            Ok(receipt.report)
        }
        _ => Err(usage()),
    }
}

fn format_violations(violations: Vec<String>) -> String {
    let mut output = format!("verification failed with {} violation(s):", violations.len());
    for violation in violations {
        output.push_str("\n - ");
        output.push_str(&violation);
    }
    output
}

fn usage() -> String {
    "usage:\n  pulse-trace-verifier validate-corpus <corpus.json> <schema.json>\n  pulse-trace-verifier verify-trace <corpus.json> <corpus-schema.json> <trace-schema.json> <trace.json>".to_string()
}

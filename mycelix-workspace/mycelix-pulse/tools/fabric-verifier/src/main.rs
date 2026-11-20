use pulse_fabric_verifier::{validate_against_schema, verify_trace};
use serde_json::json;
use std::env;
use std::fs;
use std::process;

fn usage() -> ! {
    eprintln!(
        "usage: pulse-fabric-verifier verify TRACE.json --schema TRACE.schema.json [--expected-source-sha <40-hex-sha>]"
    );
    process::exit(64);
}

fn read_json(path: &str, failure_status: &str) -> serde_json::Value {
    let source = match fs::read_to_string(path) {
        Ok(source) => source,
        Err(error) => {
            println!(
                "{}",
                json!({
                    "status": failure_status,
                    "path": path,
                    "error": error.to_string()
                })
            );
            process::exit(66);
        }
    };

    match serde_json::from_str(&source) {
        Ok(value) => value,
        Err(error) => {
            println!(
                "{}",
                json!({
                    "status": "INVALID_JSON",
                    "path": path,
                    "error": error.to_string()
                })
            );
            process::exit(2);
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 4 || args[0] != "verify" || args[2] != "--schema" {
        usage();
    }

    let trace_path = &args[1];
    let schema_path = &args[3];
    let mut expected_source_sha: Option<String> = None;
    let mut index = 4;

    while index < args.len() {
        match args[index].as_str() {
            "--expected-source-sha" => {
                index += 1;
                if index >= args.len() || expected_source_sha.is_some() {
                    usage();
                }
                expected_source_sha = Some(args[index].clone());
            }
            _ => usage(),
        }
        index += 1;
    }

    let schema = read_json(schema_path, "SCHEMA_INPUT_ERROR");
    let trace = read_json(trace_path, "TRACE_INPUT_ERROR");

    if let Err(errors) = validate_against_schema(&trace, &schema) {
        println!(
            "{}",
            json!({
                "status": "SCHEMA_VALIDATION_FAILED",
                "schema_path": schema_path,
                "finding_count": errors.len(),
                "findings": errors
            })
        );
        process::exit(2);
    }

    match verify_trace(&trace, expected_source_sha.as_deref()) {
        Ok(summary) => {
            println!(
                "{}",
                json!({
                    "status": summary.status,
                    "run_id": summary.run_id,
                    "source_sha": summary.source_sha,
                    "scenario_id": summary.scenario_id,
                    "terminal_disposition": summary.terminal_disposition,
                    "event_count": summary.event_count,
                    "operation_count": summary.operation_count,
                    "causal_edge_count": summary.causal_edge_count,
                    "schema_validation": "DRAFT_2020_12_PASS",
                    "claim_ceiling": "structural and causal consistency only"
                })
            );
        }
        Err(findings) => {
            let output_findings: Vec<serde_json::Value> = findings
                .iter()
                .map(|item| json!({"path": item.path, "message": item.message}))
                .collect();
            println!(
                "{}",
                json!({
                    "status": "INVALID_EVIDENCE_TRACE",
                    "finding_count": output_findings.len(),
                    "findings": output_findings,
                    "schema_validation": "DRAFT_2020_12_PASS"
                })
            );
            process::exit(2);
        }
    }
}

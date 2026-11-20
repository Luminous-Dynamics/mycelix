use pulse_fabric_verifier::verify_trace;
use serde_json::json;
use std::env;
use std::fs;
use std::process;

fn usage() -> ! {
    eprintln!(
        "usage: pulse-fabric-verifier verify TRACE.json [--expected-source-sha <40-hex-sha>]"
    );
    process::exit(64);
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 2 || args[0] != "verify" {
        usage();
    }

    let trace_path = &args[1];
    let mut expected_source_sha: Option<String> = None;
    let mut index = 2;

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

    let source = match fs::read_to_string(trace_path) {
        Ok(source) => source,
        Err(error) => {
            println!(
                "{}",
                json!({
                    "status": "INPUT_ERROR",
                    "path": trace_path,
                    "error": error.to_string()
                })
            );
            process::exit(66);
        }
    };

    let trace: serde_json::Value = match serde_json::from_str(&source) {
        Ok(trace) => trace,
        Err(error) => {
            println!(
                "{}",
                json!({
                    "status": "INVALID_JSON",
                    "path": trace_path,
                    "error": error.to_string()
                })
            );
            process::exit(2);
        }
    };

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
                    "findings": output_findings
                })
            );
            process::exit(2);
        }
    }
}

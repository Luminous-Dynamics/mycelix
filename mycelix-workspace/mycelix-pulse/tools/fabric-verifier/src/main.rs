use pulse_fabric_verifier::fanout::verify_fanout;
use pulse_fabric_verifier::{validate_against_schema, verify_trace};
use serde_json::json;
use std::env;
use std::fs;
use std::process;

fn usage() -> ! {
    eprintln!(
        "usage:\n  pulse-fabric-verifier verify TRACE.json --schema TRACE.schema.json [--expected-source-sha <40-hex-sha>]\n  pulse-fabric-verifier verify-fanout MANIFEST.json --manifest-schema MANIFEST.schema.json --trace-schema TRACE.schema.json --manifest-sha256 <64-hex-sha> --child TRACE.json [--child TRACE.json ...] [--expected-source-sha <40-hex-sha>]"
    );
    process::exit(64);
}

fn read_bytes(path: &str, failure_status: &str) -> Vec<u8> {
    match fs::read(path) {
        Ok(bytes) => bytes,
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
    }
}

fn read_json(path: &str, failure_status: &str) -> serde_json::Value {
    let bytes = read_bytes(path, failure_status);
    match serde_json::from_slice(&bytes) {
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

fn read_pinned_schema(path: &str, embedded_schema: &str, failure_status: &str) -> serde_json::Value {
    let schema = read_json(path, failure_status);
    let expected: serde_json::Value = serde_json::from_str(embedded_schema)
        .expect("compiled-in JSON schema must parse");

    if schema != expected {
        println!(
            "{}",
            json!({
                "status": "SCHEMA_PIN_MISMATCH",
                "path": path,
                "error": "supplied schema does not equal the schema compiled into this verifier"
            })
        );
        process::exit(2);
    }

    schema
}

fn run_verify(args: &[String]) {
    if args.len() < 4 || args[2] != "--schema" {
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

    let schema = read_pinned_schema(
        schema_path,
        include_str!("../../../docs/pulse-fabric-trace.schema-v0.1.json"),
        "SCHEMA_INPUT_ERROR",
    );
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

fn run_verify_fanout(args: &[String]) {
    if args.len() < 2 {
        usage();
    }

    let manifest_path = &args[1];
    let mut manifest_schema_path: Option<String> = None;
    let mut trace_schema_path: Option<String> = None;
    let mut expected_manifest_sha256: Option<String> = None;
    let mut expected_source_sha: Option<String> = None;
    let mut child_paths = Vec::new();
    let mut index = 2;

    while index < args.len() {
        match args[index].as_str() {
            "--manifest-schema" => {
                index += 1;
                if index >= args.len() || manifest_schema_path.is_some() {
                    usage();
                }
                manifest_schema_path = Some(args[index].clone());
            }
            "--trace-schema" => {
                index += 1;
                if index >= args.len() || trace_schema_path.is_some() {
                    usage();
                }
                trace_schema_path = Some(args[index].clone());
            }
            "--manifest-sha256" => {
                index += 1;
                if index >= args.len() || expected_manifest_sha256.is_some() {
                    usage();
                }
                expected_manifest_sha256 = Some(args[index].clone());
            }
            "--expected-source-sha" => {
                index += 1;
                if index >= args.len() || expected_source_sha.is_some() {
                    usage();
                }
                expected_source_sha = Some(args[index].clone());
            }
            "--child" => {
                index += 1;
                if index >= args.len() {
                    usage();
                }
                child_paths.push(args[index].clone());
            }
            _ => usage(),
        }
        index += 1;
    }

    let (Some(manifest_schema_path), Some(trace_schema_path), Some(expected_manifest_sha256)) = (
        manifest_schema_path,
        trace_schema_path,
        expected_manifest_sha256,
    ) else {
        usage();
    };

    if child_paths.is_empty() {
        usage();
    }

    let manifest_bytes = read_bytes(manifest_path, "MANIFEST_INPUT_ERROR");
    let manifest_schema = read_pinned_schema(
        &manifest_schema_path,
        include_str!("../../../docs/pulse-group-fanout-manifest.schema-v0.1.json"),
        "MANIFEST_SCHEMA_INPUT_ERROR",
    );
    let trace_schema = read_pinned_schema(
        &trace_schema_path,
        include_str!("../../../docs/pulse-fabric-trace.schema-v0.1.json"),
        "TRACE_SCHEMA_INPUT_ERROR",
    );
    let child_files: Vec<(String, Vec<u8>)> = child_paths
        .iter()
        .map(|path| (path.clone(), read_bytes(path, "CHILD_TRACE_INPUT_ERROR")))
        .collect();

    match verify_fanout(
        &manifest_bytes,
        &expected_manifest_sha256,
        &manifest_schema,
        &trace_schema,
        &child_files,
        expected_source_sha.as_deref(),
    ) {
        Ok(summary) => {
            println!(
                "{}",
                json!({
                    "status": summary.status,
                    "fanout_id": summary.fanout_id,
                    "recipient_unit": summary.recipient_unit,
                    "manifest_sha256": summary.manifest_sha256,
                    "aggregate_evidence_state": summary.disposition,
                    "required_recipients": summary.required_recipients,
                    "verified_recipients": summary.verified_recipients,
                    "partial_recipients": summary.partial_recipients,
                    "unknown_recipients": summary.unknown_recipients,
                    "incomplete_recipients": summary.incomplete_recipients,
                    "expired_recipients": summary.expired_recipients,
                    "failed_recipients": summary.failed_recipients,
                    "verification_scope": "STRUCTURAL_AND_CAUSAL_CONSISTENCY_ONLY",
                    "claim_ceiling": "content-addressed structural consistency only; manifest authority, membership truth, receipt signatures and deadline truth are not verified"
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
                    "status": "INVALID_FANOUT_EVIDENCE",
                    "finding_count": output_findings.len(),
                    "findings": output_findings,
                    "claim_ceiling": "no group delivery claim is established"
                })
            );
            process::exit(2);
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("verify") => run_verify(&args),
        Some("verify-fanout") => run_verify_fanout(&args),
        _ => usage(),
    }
}

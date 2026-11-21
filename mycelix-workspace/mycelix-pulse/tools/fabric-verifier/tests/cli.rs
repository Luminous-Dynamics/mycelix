use serde_json::Value;
use std::path::PathBuf;
use std::process::{Command, Output};
use std::time::{SystemTime, UNIX_EPOCH};

fn run_cli(fixture: &str, expected_source_sha: Option<&str>) -> Output {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let trace = manifest_dir.join("tests").join("fixtures").join(fixture);
    let schema = manifest_dir.join("../../docs/pulse-fabric-trace.schema-v0.1.json");

    let mut command = Command::new(env!("CARGO_BIN_EXE_pulse-fabric-verifier"));
    command.arg("verify").arg(trace).arg("--schema").arg(schema);

    if let Some(source_sha) = expected_source_sha {
        command.arg("--expected-source-sha").arg(source_sha);
    }

    command.output().expect("verifier CLI must start")
}

fn stdout_json(output: &Output) -> Value {
    serde_json::from_slice(&output.stdout).unwrap_or_else(|error| {
        panic!(
            "verifier stdout must be JSON; error={error}; stdout={}; stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        )
    })
}

#[test]
fn cli_accepts_complete_receipt_backed_success_fixture() {
    let source_sha = "1111111111111111111111111111111111111111";
    let output = run_cli("valid-success.json", Some(source_sha));
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stdout)
    );
    let result = stdout_json(&output);
    assert_eq!(result["status"], "EVIDENCE_CONSISTENT");
    assert_eq!(result["terminal_disposition"], "SUCCESS_VERIFIED");
    assert_eq!(result["schema_validation"], "DRAFT_2020_12_PASS");
}

#[test]
fn cli_accepts_unknown_ack_only_fixture_without_promoting_delivery() {
    let output = run_cli("unknown-transport-ack.json", None);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stdout)
    );
    let result = stdout_json(&output);
    assert_eq!(result["status"], "EVIDENCE_CONSISTENT");
    assert_eq!(result["terminal_disposition"], "UNKNOWN");
}

#[test]
fn cli_rejects_transport_ack_promoted_to_verified_delivery() {
    let output = run_cli("false-success-from-ack.json", None);
    assert!(!output.status.success());
    let result = stdout_json(&output);
    assert_eq!(result["status"], "INVALID_EVIDENCE_TRACE");
    assert_eq!(result["schema_validation"], "DRAFT_2020_12_PASS");
}

#[test]
fn cli_rejects_external_source_sha_pin_mismatch() {
    let output = run_cli(
        "valid-success.json",
        Some("9999999999999999999999999999999999999999"),
    );
    assert!(!output.status.success());
    let result = stdout_json(&output);
    assert_eq!(result["status"], "INVALID_EVIDENCE_TRACE");
}

#[test]
fn cli_rejects_a_schema_that_differs_from_the_compiled_schema() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let trace = manifest_dir.join("tests").join("fixtures").join("valid-success.json");
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock must be after UNIX epoch")
        .as_nanos();
    let schema = std::env::temp_dir().join(format!(
        "pulse-invalid-trace-schema-{}-{nonce}.json",
        std::process::id()
    ));
    fs::write(&schema, "{}").expect("altered schema must be written");

    let output = Command::new(env!("CARGO_BIN_EXE_pulse-fabric-verifier"))
        .arg("verify")
        .arg(trace)
        .arg("--schema")
        .arg(&schema)
        .output()
        .expect("verifier CLI must start");

    assert!(!output.status.success());
    let result = stdout_json(&output);
    assert_eq!(result["status"], "SCHEMA_PIN_MISMATCH");

    fs::remove_file(schema).expect("temporary schema must be removed");
}

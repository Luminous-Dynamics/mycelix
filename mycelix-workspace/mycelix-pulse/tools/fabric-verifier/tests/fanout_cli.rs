use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::time::{SystemTime, UNIX_EPOCH};

const RECIPIENT_EVENT_TYPES: &[&str] = &[
    "recipient_admitted",
    "recipient_inbox_committed",
    "recipient_decryption_verified",
    "receipt_created",
    "receipt_verified",
];

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn recipient_set_commitment(recipient_commitments: &[String]) -> String {
    let mut sorted = recipient_commitments.to_vec();
    sorted.sort();
    let mut digest = Sha256::new();
    digest.update(b"PULSE-GROUP-FANOUT-RECIPIENT-SET-V0\0");
    for commitment in sorted {
        let bytes = commitment.as_bytes();
        digest.update((bytes.len() as u64).to_be_bytes());
        digest.update(bytes);
    }
    let output = digest.finalize();
    output.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn fixture_dir() -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock must be after UNIX epoch")
        .as_nanos();
    let path = std::env::temp_dir().join(format!(
        "pulse-fanout-cli-{}-{nonce}",
        std::process::id()
    ));
    fs::create_dir_all(&path).expect("temporary fixture directory must be created");
    path
}

fn write_fanout_fixture(directory: &Path) -> (String, PathBuf, PathBuf, PathBuf, PathBuf) {
    let crate_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let fixture_path = crate_root.join("tests/fixtures/valid-success.json");
    let mut child: Value = serde_json::from_slice(
        &fs::read(fixture_path).expect("valid success fixture must be readable"),
    )
    .expect("valid success fixture must parse");

    let logical_operation_id = "logical-fanout-cli-1";
    let recipient_commitment = "b".repeat(64);
    let recipient_set = recipient_set_commitment(std::slice::from_ref(&recipient_commitment));

    for event in child["events"].as_array_mut().expect("events array") {
        event["logical_operation_id"] = json!(logical_operation_id);
        let event_type = event["event_type"].as_str().unwrap_or("");
        if RECIPIENT_EVENT_TYPES.contains(&event_type) {
            event["recipient_commitment"] = json!(recipient_commitment);
        }
    }

    child["group_fanout"] = json!({
        "fanout_id": "fanout-cli-1",
        "group_id_commitment": "1".repeat(64),
        "membership_epoch": 11,
        "membership_snapshot_digest": "2".repeat(64),
        "required_recipient_set_commitment": recipient_set,
        "recipient_commitment": recipient_commitment,
        "content_commitment": "a".repeat(64),
        "aggregation_policy": "ALL_REQUIRED"
    });

    let child_bytes = serde_json::to_vec_pretty(&child).expect("child trace must serialize");
    let child_sha256 = sha256_hex(&child_bytes);
    let child_path = directory.join("child.json");
    fs::write(&child_path, &child_bytes).expect("child trace must be written");

    let manifest = json!({
        "schema_version": "0.1.0",
        "profile_id": "PULSE-GROUP-FANOUT-QUAL-001",
        "fanout_id": "fanout-cli-1",
        "group_id_commitment": "1".repeat(64),
        "membership_epoch": 11,
        "membership_snapshot_digest": "2".repeat(64),
        "required_recipient_set_commitment": recipient_set,
        "content_commitment": "a".repeat(64),
        "aggregation_policy": "ALL_REQUIRED",
        "children": [{
            "recipient_commitment": recipient_commitment,
            "logical_operation_id": logical_operation_id,
            "trace_sha256": child_sha256
        }],
        "claim_ceiling": [
            "synthetic structural fan-out consistency only",
            "manifest authority and membership truth are not verified"
        ]
    });
    let manifest_bytes =
        serde_json::to_vec_pretty(&manifest).expect("manifest must serialize");
    let manifest_sha256 = sha256_hex(&manifest_bytes);
    let manifest_path = directory.join("manifest.json");
    fs::write(&manifest_path, &manifest_bytes).expect("manifest must be written");

    let manifest_schema_path = directory.join("manifest.schema.json");
    fs::write(
        &manifest_schema_path,
        include_str!("../../../docs/pulse-group-fanout-manifest.schema-v0.1.json"),
    )
    .expect("manifest schema must be written");

    let trace_schema_path = directory.join("trace.schema.json");
    fs::write(
        &trace_schema_path,
        include_str!("../../../docs/pulse-fabric-trace.schema-v0.1.json"),
    )
    .expect("trace schema must be written");

    (
        manifest_sha256,
        manifest_path,
        manifest_schema_path,
        trace_schema_path,
        child_path,
    )
}

fn run_cli(directory: &Path) -> (Output, String) {
    let (manifest_sha, manifest, manifest_schema, trace_schema, child) =
        write_fanout_fixture(directory);

    let output = Command::new(env!("CARGO_BIN_EXE_pulse-fabric-verifier"))
        .arg("verify-fanout")
        .arg(manifest)
        .arg("--manifest-schema")
        .arg(manifest_schema)
        .arg("--trace-schema")
        .arg(trace_schema)
        .arg("--manifest-sha256")
        .arg(&manifest_sha)
        .arg("--child")
        .arg(child)
        .output()
        .expect("fanout verifier CLI must start");

    (output, manifest_sha)
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
fn cli_reports_all_required_receipt_claims_as_consistent() {
    let directory = fixture_dir();
    let (output, _) = run_cli(&directory);

    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stdout)
    );
    let result = stdout_json(&output);
    assert_eq!(result["status"], "FANOUT_EVIDENCE_CONSISTENT");
    assert_eq!(
        result["aggregate_evidence_state"],
        "ALL_REQUIRED_RECEIPT_CLAIMS_CONSISTENT"
    );
    assert_eq!(result["required_recipients"], 1);
    assert_eq!(result["verified_recipients"], 1);
    assert_eq!(
        result["verification_scope"],
        "STRUCTURAL_AND_CAUSAL_CONSISTENCY_ONLY"
    );

    fs::remove_dir_all(directory).expect("temporary fixtures must be removed");
}

#[test]
fn cli_rejects_a_child_trace_that_is_not_pinned_by_the_manifest() {
    let directory = fixture_dir();
    let (_, manifest, manifest_schema, trace_schema, child) =
        write_fanout_fixture(&directory);
    let output = Command::new(env!("CARGO_BIN_EXE_pulse-fabric-verifier"))
        .arg("verify-fanout")
        .arg(manifest)
        .arg("--manifest-schema")
        .arg(manifest_schema)
        .arg("--trace-schema")
        .arg(trace_schema)
        .arg("--manifest-sha256")
        .arg("0".repeat(64))
        .arg("--child")
        .arg(child)
        .output()
        .expect("fanout verifier CLI must start");

    assert!(!output.status.success());
    let result = stdout_json(&output);
    assert_eq!(result["status"], "INVALID_FANOUT_EVIDENCE");

    fs::remove_dir_all(directory).expect("temporary fixtures must be removed");
}

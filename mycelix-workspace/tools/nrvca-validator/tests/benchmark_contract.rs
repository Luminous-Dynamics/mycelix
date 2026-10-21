use nrvca_validator::validate_benchmark_preregistration;
use serde_json::Value;

const BENCHMARK: &str =
    include_str!("../../../../fixtures/nrvca-sa-bridge-preregistration-v0.1.json");

fn load_benchmark() -> Value {
    serde_json::from_str(BENCHMARK).expect("benchmark preregistration parses as JSON")
}

#[test]
fn benchmark_manifest_must_match_declared_source_ids() {
    let mut record = load_benchmark();
    record["source_ids"] = serde_json::json!(["SA-GDP-P0441-Q2-2026"]);
    let diagnostics = validate_benchmark_preregistration(&record);
    assert!(diagnostics.iter().any(|d| d.code == "SEM-BENCH-MANIFEST-UNDECLARED"));
}

#[test]
fn benchmark_requires_candidate_and_baseline() {
    let mut record = load_benchmark();
    for arm in record["arms"].as_array_mut().expect("arms array") {
        arm["role"] = serde_json::json!("incumbent");
    }
    let diagnostics = validate_benchmark_preregistration(&record);
    assert!(diagnostics.iter().any(|d| d.code == "SEM-BENCH-NO-CANDIDATE"));
}

#[test]
fn benchmark_cannot_execute_when_source_rights_are_unreviewed() {
    let mut record = load_benchmark();
    record["status"] = serde_json::json!("preregistered");
    record["execution_state"] = serde_json::json!("ready_for_execution");
    for source in record["source_manifest"].as_array_mut().expect("source manifest") {
        source["content_sha256"] =
            serde_json::json!("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        source["ingest_state"] = serde_json::json!("ingested");
    }
    let diagnostics = validate_benchmark_preregistration(&record);
    assert!(diagnostics.iter().any(|d| d.code == "SEM-BENCH-EXECUTION-RIGHTS"));
}

#[test]
fn benchmark_cannot_hide_failed_runs() {
    let mut record = load_benchmark();
    record["failure_policy"]["retain_failed_runs"] = serde_json::json!(false);
    let diagnostics = validate_benchmark_preregistration(&record);
    assert!(diagnostics.iter().any(|d| d.code == "SEM-BENCH-FAILURE-POLICY"));
}

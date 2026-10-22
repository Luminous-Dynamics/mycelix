use nrvca_validator::external_validity::validate_benchmark_bundle;
use serde_json::{json, Value};

const PROTOCOL: &str =
    include_str!("../../../../fixtures/nrvca-external-validity-bundle-v0.1.json");

fn bundle() -> Value {
    serde_json::from_str(PROTOCOL).expect("pre-registration fixture parses")
}

fn source(id: &str, digest: &str) -> Value {
    json!({
        "source_id": id,
        "publisher": "Synthetic Test Publisher",
        "source_uri": format!("https://example.invalid/{id}.pdf"),
        "publication_id": format!("TEST-{id}"),
        "vintage": "2025",
        "captured_at": "2026-10-22T10:00:00Z",
        "bytes_acquired": true,
        "sha256": digest,
        "extraction": {
            "method": "synthetic test metadata",
            "tool_version": "test-only-v0"
        },
        "locators": [{
            "locator_id": format!("loc-{id}"),
            "locator": "Synthetic test page 1, table 1, row 1"
        }],
        "rights": {
            "status": "open_reuse",
            "evaluation_allowed": true,
            "redistribution_allowed": true,
            "evidence_uri": "https://example.invalid/reuse-policy"
        }
    })
}

fn pair(
    pair_id: &str,
    split: &str,
    left_source: &str,
    right_source: &str,
    left_provider: &str,
    right_provider: &str,
    left_contract: &str,
    right_contract: &str,
    label: &str,
    hard_mismatch: bool,
) -> Value {
    json!({
        "pair_id": pair_id,
        "split": split,
        "left": {
            "source_id": left_source,
            "contract_id": left_contract,
            "contract_locator_id": format!("loc-{left_source}"),
            "provider_group_id": left_provider,
            "vintage": "2024"
        },
        "right": {
            "source_id": right_source,
            "contract_id": right_contract,
            "contract_locator_id": format!("loc-{right_source}"),
            "provider_group_id": right_provider,
            "vintage": "2025"
        },
        "adjudicated_label": label,
        "evidence_sufficiency": "sufficient",
        "known_hard_mismatch": hard_mismatch,
        "adjudication_rationale": "Synthetic test rationale; not empirical evidence.",
        "raters_blinded_to_comparator": true,
        "raters": [
            {"rater_id": "rater-a", "decision": label, "blinded_to_comparator": true},
            {"rater_id": "rater-b", "decision": label, "blinded_to_comparator": true}
        ]
    })
}

fn populated_bundle() -> Value {
    let mut root = bundle();
    root["sources"] = json!([
        source("S1", &"a".repeat(64)),
        source("S2", &"b".repeat(64))
    ]);
    root["pairs"] = json!([
        pair("P1", "development", "S1", "S2", "provider-A", "provider-B", "C1", "C2", "not_comparable", true)
    ]);
    root
}

fn has_code(diagnostics: &[nrvca_validator::external_validity::EvidenceDiagnostic], code: &str) -> bool {
    diagnostics.iter().any(|d| d.code == format!("EXTVAL-{code}"))
}

#[test]
fn preregistration_is_valid_but_remains_blocked_with_zero_real_observations() {
    let root = bundle();
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(diagnostics.is_empty(), "pre-registration diagnostics: {diagnostics:?}");
    assert_eq!(root["sources"].as_array().unwrap().len(), 0);
    assert_eq!(root["pairs"].as_array().unwrap().len(), 0);
    assert_eq!(root["external_evaluation_enabled"], false);
    assert_eq!(root["execution_state"], "blocked_no_adjudicated_records");
}

#[test]
fn pinned_rights_cleared_synthetic_contract_records_satisfy_the_manifest_contract() {
    let root = populated_bundle();
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(diagnostics.is_empty(), "bundle diagnostics: {diagnostics:?}");
}

#[test]
fn missing_source_digest_and_unlicensed_sources_are_rejected() {
    let mut root = populated_bundle();
    root["sources"][0]["sha256"] = json!("not-a-sha");
    root["sources"][1]["rights"]["status"] = json!("unknown");
    root["sources"][1]["rights"]["evaluation_allowed"] = json!(false);
    root["sources"][1]["rights"]["redistribution_allowed"] = json!(false);
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(has_code(&diagnostics, "SOURCE-DIGEST"));
    assert!(has_code(&diagnostics, "RIGHTS-NOT-EVALUABLE"));
    assert!(has_code(&diagnostics, "PAIR-SOURCE-INELIGIBLE"));
}

#[test]
fn rater_disagreement_requires_a_third_independent_adjudicator() {
    let mut root = populated_bundle();
    root["pairs"][0]["raters"][0]["decision"] = json!("comparable");
    root["pairs"][0]["adjudicator"] = json!({
        "adjudicator_id": "rater-c",
        "decision": "not_comparable",
        "rationale": "Hard denominator mismatch confirmed from pinned definition.",
        "independent": true
    });
    assert!(validate_benchmark_bundle(&root).is_empty());

    root["pairs"][0]["adjudicator"]["adjudicator_id"] = json!("rater-a");
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(has_code(&diagnostics, "ADJUDICATION-REQUIRED"));
}

#[test]
fn model_visible_or_duplicate_raters_and_insufficient_gold_labels_are_rejected() {
    let mut root = populated_bundle();
    root["pairs"][0]["raters"][1]["rater_id"] = json!("rater-a");
    root["pairs"][0]["raters"][0]["blinded_to_comparator"] = json!(false);
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(has_code(&diagnostics, "RATER-INDEPENDENCE"));
    assert!(has_code(&diagnostics, "RATER-BLINDING"));

    let mut root = populated_bundle();
    root["pairs"][0]["evidence_sufficiency"] = json!("insufficient");
    root["pairs"][0]["adjudicated_label"] = json!("indeterminate");
    root["pairs"][0]["known_hard_mismatch"] = json!(false);
    root["pairs"][0]["raters"][0]["decision"] = json!("indeterminate");
    root["pairs"][0]["raters"][1]["decision"] = json!("indeterminate");
    assert!(validate_benchmark_bundle(&root).is_empty());
}

#[test]
fn primary_split_leakage_by_provider_is_detected() {
    let mut root = populated_bundle();
    root["sources"].as_array_mut().unwrap().push(source("S3", &"c".repeat(64)));
    root["sources"].as_array_mut().unwrap().push(source("S4", &"d".repeat(64)));
    root["pairs"].as_array_mut().unwrap().push(pair(
        "P2", "validation", "S3", "S4", "provider-A", "provider-D", "C3", "C4",
        "not_comparable", true
    ));
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(has_code(&diagnostics, "PROVIDER-LEAKAGE"));
}

#[test]
fn temporal_vintage_test_is_separate_from_primary_provider_split_map() {
    let mut root = populated_bundle();
    let mut temporal = root["pairs"][0].clone();
    temporal["pair_id"] = json!("P-temporal");
    temporal["split"] = json!("temporal_test");
    temporal["left"]["vintage"] = json!("2023");
    temporal["right"]["vintage"] = json!("2026");
    temporal["left"]["contract_id"] = json!("C1-2023");
    temporal["right"]["contract_id"] = json!("C2-2026");
    root["pairs"].as_array_mut().unwrap().push(temporal);
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(!has_code(&diagnostics, "PROVIDER-LEAKAGE"));
    assert!(!has_code(&diagnostics, "CONTRACT-LEAKAGE"));
    assert!(!has_code(&diagnostics, "PAIR-CROSS-SPLIT"));
}

#[test]
fn enabling_external_evaluation_before_minimum_sample_gates_fails_closed() {
    let mut root = populated_bundle();
    root["external_evaluation_enabled"] = json!(true);
    root["artifact_status"] = json!("external_evaluation_dataset");
    root["execution_state"] = json!("evaluation_ready");
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(has_code(&diagnostics, "EXECUTION-HARD-MISMATCH-SAMPLE"));
    assert!(has_code(&diagnostics, "EXECUTION-QUALITY-GATES"));
}

#[test]
fn weakened_preregistration_cannot_remove_provider_holdout_or_sample_targets() {
    let mut root = bundle();
    root["protocol"]["split_policy"]["random_pair_split_allowed"] = json!(true);
    root["protocol"]["acceptance_targets"]["minimum_hard_mismatch_pairs"] = json!(20);
    root["protocol"]["reporting"]["confidence_intervals"] = json!(false);
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(has_code(&diagnostics, "SPLIT-POLICY"));
    assert!(has_code(&diagnostics, "ACCEPTANCE-TARGETS"));
    assert!(has_code(&diagnostics, "REPORTING-POLICY"));
}

#[test]
fn identical_underlying_contract_pair_cannot_be_reused_across_splits() {
    let mut root = populated_bundle();
    let mut duplicate = root["pairs"][0].clone();
    duplicate["pair_id"] = json!("P1-duplicate");
    duplicate["split"] = json!("provider_held_out_test");
    root["pairs"].as_array_mut().unwrap().push(duplicate);
    let diagnostics = validate_benchmark_bundle(&root);
    assert!(has_code(&diagnostics, "PAIR-CROSS-SPLIT"));
}

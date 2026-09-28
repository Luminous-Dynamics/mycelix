use cos_conformance::integral_oad_cos_formal::{ClosureState, OBLIGATIONS};
use serde_json::Value;

const JSON_MANIFEST: &str = include_str!("../proofs/if01_refinement_manifest.json");
const SMT_ARTIFACT: &str = include_str!("../proofs/integral_if01_authority_outcome.smt2");

#[test]
fn rust_and_json_obligation_ledgers_have_exact_same_ids() {
    let json: Value = serde_json::from_str(JSON_MANIFEST).expect("valid JSON manifest");
    let items = json["obligations"].as_array().expect("obligations array");
    assert_eq!(items.len(), OBLIGATIONS.len());
    for (rust, item) in OBLIGATIONS.iter().zip(items) {
        assert_eq!(item["id"].as_str(), Some(rust.id));
        assert_eq!(item["status"].as_str(), Some("BoundedExecutableWitness"));
        assert_ne!(rust.state, ClosureState::FormallyClosed);
        assert!(!rust.proposition.is_empty());
        assert!(!rust.claim_ceiling.is_empty());
    }
}

#[test]
fn json_production_and_test_symbols_are_nonempty_and_artifacts_are_present() {
    let json: Value = serde_json::from_str(JSON_MANIFEST).expect("valid JSON manifest");
    for item in json["obligations"].as_array().expect("obligations array") {
        assert!(!item["production_symbol"].as_str().unwrap_or("").is_empty());
        assert!(!item["test_symbol"].as_str().unwrap_or("").is_empty());
        assert!(!item["scope"].as_str().unwrap_or("").is_empty());
        assert!(!item["claim_ceiling"].as_str().unwrap_or("").is_empty());
        if let Some(smt) = item["smt"].as_str() {
            assert!(smt.starts_with("proofs/integral_if01_authority_outcome.smt2::"));
            assert!(SMT_ARTIFACT.contains(item["id"].as_str().unwrap()));
        }
    }
}

#[test]
fn formal_closure_requires_more_than_the_current_bounded_state() {
    let json: Value = serde_json::from_str(JSON_MANIFEST).expect("valid JSON manifest");
    assert_eq!(json["status"].as_str(), Some("BoundedExecutableWitness"));
    assert_eq!(json["closure_rule"].as_str().unwrap().contains("production refinement"), true);
    assert_eq!(json["consistency_gate"]["required_obligation_count"].as_u64(), Some(12));
    assert_eq!(json["consistency_gate"]["forbid_formally_closed_without_runtime_receipt"].as_bool(), Some(true));
}

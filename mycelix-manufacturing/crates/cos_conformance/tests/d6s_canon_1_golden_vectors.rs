//! D6S-CANON-1 frozen golden-vector verifier for COS reference semantics.
//! This verifies the declared reference model, not physical or Integral behavior.

use cos_conformance::conformance_report_json;
use serde_json::Value;
use std::collections::BTreeSet;

const GOLDEN: &str = include_str!("../../../../docs/integral/d6s-canon-1-golden-vectors.json");

fn fixture() -> Value {
    serde_json::from_str(GOLDEN).expect("checked-in D6S-CANON-1 JSON must parse")
}

#[test]
fn golden_vector_corpus_is_complete_and_unambiguous() {
    let fixture = fixture();
    assert_eq!(fixture["schema_version"], "d6s-canon-1-golden-vectors/1");
    assert_eq!(fixture["status"], "ReferenceModelOnly");
    assert_eq!(fixture["corpus_id"], cos_conformance::CORPUS_ID);
    let vectors = fixture["vectors"].as_array().expect("vectors");
    assert_eq!(vectors.len(), fixture["case_count"].as_u64().expect("case count") as usize);
    assert_eq!(vectors.len(), 16);

    let ids: BTreeSet<&str> = vectors
        .iter()
        .map(|vector| vector["test_id"].as_str().expect("test_id"))
        .collect();
    assert_eq!(ids.len(), vectors.len(), "canonical test IDs must be unique");
    assert_eq!(
        ids,
        cos_conformance::CASES.iter().map(|case| case.test_id).collect(),
        "the frozen vector set must cover the live reference cases exactly"
    );
    let obligations: BTreeSet<&str> = vectors
        .iter()
        .flat_map(|vector| vector["formal_obligations"].as_array().expect("obligations"))
        .map(|obligation| obligation.as_str().expect("obligation ID"))
        .collect();
    let declared: BTreeSet<&str> = cos_conformance::FORMAL_OBLIGATIONS.iter().copied().collect();
    assert_eq!(obligations, declared, "every formal obligation must be covered by at least one vector");
}

#[test]
fn reference_report_matches_every_frozen_negative_and_positive_disposition() {
    let fixture = fixture();
    let vectors = fixture["vectors"].as_array().expect("vectors");
    let report: Value = serde_json::from_str(&conformance_report_json()).expect("reference report JSON");
    assert_eq!(report["corpus_id"], fixture["corpus_id"]);
    assert_eq!(
        report["case_count"],
        fixture["expected_explicit_case_pair_count"],
        "each canonical vector must have a negative and a positive counterpart"
    );
    assert_eq!(
        report["untested_formal_obligations"],
        fixture["expected_untested_formal_obligations"]
    );
    let records = report["records"].as_array().expect("records");
    assert_eq!(
        records.len(),
        fixture["expected_report_record_count"].as_u64().expect("record count") as usize
    );

    for vector in vectors {
        let id = vector["test_id"].as_str().expect("test ID");
        let record = records
            .iter()
            .find(|record| record["test_id"].as_str() == Some(id))
            .unwrap_or_else(|| panic!("report missing canonical vector {id}"));
        assert_eq!(record["expected_negative"], vector["expected_negative"], "{id} negative oracle");
        assert_eq!(record["actual_negative"], vector["expected_negative"], "{id} negative result");
        assert_eq!(record["expected_positive"], vector["expected_positive"], "{id} positive oracle");
        assert_eq!(record["actual_positive"], vector["expected_positive"], "{id} positive result");
        assert_eq!(record["formal_obligations"], vector["formal_obligations"], "{id} formal map");
        assert_eq!(record["claim_ceiling"], vector["claim_ceiling"], "{id} claim ceiling");
        assert_eq!(record["refinement_status"], "ReferenceModelOnly", "{id} qualification ceiling");
    }
}

#[test]
fn no_vector_can_claim_physical_or_integral_qualification() {
    let fixture = fixture();
    let ceiling = fixture["claim_ceiling"].as_str().expect("claim ceiling");
    assert!(ceiling.contains("no physical"));
    assert!(ceiling.contains("no physical, safety, economic, ecological, or Integral-validation claim"));
    for vector in fixture["vectors"].as_array().expect("vectors") {
        let ceiling = vector["claim_ceiling"].as_str().expect("vector claim ceiling");
        assert!(!ceiling.to_ascii_lowercase().contains("certified"));
        assert!(!ceiling.to_ascii_lowercase().contains("physically validated"));
    }
}

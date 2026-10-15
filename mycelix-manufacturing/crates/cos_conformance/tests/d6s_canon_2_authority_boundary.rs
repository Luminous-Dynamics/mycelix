use cos_conformance::{
    authorize_recommendation, create_source_observation, evaluate_negative,
    project_to_frs, project_to_itc, qualify_output, record_effect, recognize_foreign_evidence,
    stale_evidence_is_rejected, bind_physical_work, Bindings, Decision, Evidence, Origin,
};
use serde_json::Value;

const FIXTURE: &str =
    include_str!("../../../../docs/integral/d6s-canon-2-authority-boundary-fixture.json");

fn case(case_id: &str) -> Value {
    let fixture: Value = serde_json::from_str(FIXTURE).expect("fixture must be valid JSON");
    assert_eq!(fixture["fixture_id"], "D6S-CANON-2");
    assert_eq!(fixture["status"], "ReferenceModelOnly");
    fixture["cases"].as_array().expect("cases").iter()
        .find(|item| item["case_id"] == case_id)
        .cloned()
        .unwrap_or_else(|| panic!("missing fixture case {case_id}"))
}

#[test]
fn production_observation_needs_explicit_itc_projection() {
    let c = case("production-observation-is-not-itc-mint");
    let e = Evidence::current_local("d6s-canon-2-local", 100);
    let mut b = Bindings::default();
    create_source_observation(&mut b);
    let before = evaluate_negative("COS-N-011", &b, Some(&e), 100);
    assert_eq!(format!("{before:?}"), c["expected_without_projection"].as_str().unwrap());
    assert!(!b.itc_projection);
    project_to_itc(&mut b);
    let after = evaluate_negative("COS-N-011", &b, Some(&e), 100);
    assert_eq!(format!("{after:?}"), c["expected_after_explicit_projection"].as_str().unwrap());
}

#[test]
fn frs_projection_needs_explicit_source_observation() {
    let c = case("frs-projection-is-not-source-observation");
    let e = Evidence::current_local("d6s-canon-2-frs", 100);
    let mut b = Bindings::default();
    project_to_frs(&mut b);
    let before = evaluate_negative("COS-N-010", &b, Some(&e), 100);
    assert_eq!(format!("{before:?}"), c["expected_without_source_observation"].as_str().unwrap());
    assert!(!b.source_observation_created);
    create_source_observation(&mut b);
    let after = evaluate_negative("COS-N-010", &b, Some(&e), 100);
    assert_eq!(format!("{after:?}"), c["expected_after_explicit_source_observation"].as_str().unwrap());
}

#[test]
fn recommendation_requires_both_authority_and_effect_receipt() {
    let c = case("recommendation-does-not-create-effect");
    let e = Evidence::current_local("d6s-canon-2-recommendation", 100);
    let mut b = Bindings::default();
    let before = evaluate_negative("COS-N-014", &b, Some(&e), 100);
    assert_eq!(format!("{before:?}"), c["expected_without_authority"].as_str().unwrap());
    authorize_recommendation(&mut b);
    let authorized_without_effect = evaluate_negative("COS-N-014", &b, Some(&e), 100);
    assert_eq!(format!("{authorized_without_effect:?}"), c["expected_with_authority_but_without_effect"].as_str().unwrap());
    assert!(!b.effect_recorded);
    record_effect(&mut b);
    let after = evaluate_negative("COS-N-014", &b, Some(&e), 100);
    assert_eq!(format!("{after:?}"), c["expected_after_authorized_effect_receipt"].as_str().unwrap());
}

#[test]
fn foreign_evidence_retains_origin_after_recognition() {
    let c = case("foreign-recognition-preserves-origin");
    let foreign = Evidence::current_foreign("d6s-canon-2-foreign", "node-b", 100);
    assert_eq!(format!("{:?}", foreign.origin), "Foreign(\"node-b\")");
    let mut b = Bindings::default();
    recognize_foreign_evidence(&mut b);
    let decision = evaluate_negative("COS-N-013", &b, Some(&foreign), 100);
    assert_eq!(format!("{decision:?}"), c["expected_decision"].as_str().unwrap());
    assert_eq!(foreign.origin, Origin::Foreign("node-b".to_owned()));
    assert_eq!(c["expected_origin_before"], "foreign");
    assert_eq!(c["expected_origin_after"], "foreign");
}

#[test]
fn output_qualification_does_not_bind_physical_work() {
    let c = case("qualified-output-is-not-physical-work");
    let e = Evidence::current_local("d6s-canon-2-output", 100);
    let mut b = Bindings::default();
    qualify_output(&mut b);
    let before = evaluate_negative("COS-N-012", &b, Some(&e), 100);
    assert_eq!(format!("{before:?}"), c["expected_without_physical_work_binding"].as_str().unwrap());
    assert!(!b.physical_work_binding);
    bind_physical_work(&mut b);
    let after = evaluate_negative("COS-N-012", &b, Some(&e), 100);
    assert_eq!(format!("{after:?}"), c["expected_after_physical_work_binding"].as_str().unwrap());
}

#[test]
fn stale_evidence_is_not_current_evidence() {
    let c = case("stale-evidence-does-not-satisfy-current-requirement");
    let stale = Evidence::stale("d6s-canon-2-stale", 100);
    let current = Evidence::current_local("d6s-canon-2-current", 100);
    let stale_decision = stale_evidence_is_rejected(&stale, 100);
    assert_eq!(format!("{stale_decision:?}"), c["expected_decision"].as_str().unwrap());
    assert_eq!(stale_evidence_is_rejected(&current, 100), Decision::Accepted);
}

#[test]
fn unknown_transition_stays_unknown() {
    let c = case("unknown-transition-remains-unknown");
    let e = Evidence::current_local("d6s-canon-2-unknown", 100);
    let decision = evaluate_negative("COS-N-999", &Bindings::default(), Some(&e), 100);
    assert_eq!(format!("{decision:?}"), c["expected_decision"].as_str().unwrap());
    assert_eq!(decision, Decision::Unknown);
}

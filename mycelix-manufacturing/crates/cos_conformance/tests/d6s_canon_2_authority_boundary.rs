use cos_conformance::{
    authorize_recommendation, create_source_observation, evaluate_negative, project_to_frs,
    project_to_itc, qualify_output, record_effect, recognize_foreign_evidence,
    stale_evidence_is_rejected, bind_physical_work, Bindings, Decision, Evidence, Origin,
};
use serde_json::Value;

const FIXTURE: &str =
    include_str!("../../../../docs/integral/d6s-canon-2-authority-boundary-fixture.json");

fn fixture() -> Value {
    let fixture: Value = serde_json::from_str(FIXTURE).expect("fixture must be valid JSON");
    assert_eq!(fixture["fixture_id"], "D6S-CANON-2");
    assert_eq!(fixture["status"], "ReferenceModelOnly");
    fixture
}

fn case(case_id: &str) -> Value {
    fixture()["cases"].as_array().expect("cases").iter()
        .find(|item| item["case_id"] == case_id)
        .cloned()
        .unwrap_or_else(|| panic!("missing fixture case {case_id}"))
}

fn apply_bindings(bindings: &mut Bindings, names: &Value) {
    for name in names.as_array().expect("binding list").iter() {
        match name.as_str().expect("binding name string") {
            "source_observation_created" => create_source_observation(bindings),
            "itc_projection" => project_to_itc(bindings),
            "frs_projection" => project_to_frs(bindings),
            "recommendation_authorized" => authorize_recommendation(bindings),
            "effect_recorded" => record_effect(bindings),
            "foreign_recognition" => recognize_foreign_evidence(bindings),
            "output_qualification" => qualify_output(bindings),
            "physical_work_binding" => bind_physical_work(bindings),
            other => panic!("unknown D6S-CANON-2 binding: {other}"),
        }
    }
}

fn evidence(profile: &str) -> Evidence {
    match profile {
        "current_local" => Evidence::current_local("d6s-canon-2-current", 100),
        "stale_local" => Evidence::stale("d6s-canon-2-stale", 100),
        "current_foreign_node_b" => {
            Evidence::current_foreign("d6s-canon-2-foreign", "node-b", 100)
        }
        other => panic!("unknown D6S-CANON-2 evidence profile: {other}"),
    }
}

fn expect_decision(actual: Decision, expected: &Value, label: &str) {
    assert_eq!(
        format!("{actual:?}"),
        expected.as_str().expect("expected decision string"),
        "{label}"
    );
}

#[test]
fn production_observation_needs_explicit_itc_projection() {
    let c = case("production-observation-is-not-itc-mint");
    let e = evidence(c["evidence_profile"].as_str().unwrap());
    let mut b = Bindings::default();
    apply_bindings(&mut b, &c["initial_bindings"]);
    let before = evaluate_negative("COS-N-011", &b, Some(&e), 100);
    expect_decision(before, &c["expected_without_projection"], "projection must be explicit");
    assert!(!b.itc_projection);
    apply_bindings(&mut b, &c["additional_bindings"]);
    let after = evaluate_negative("COS-N-011", &b, Some(&e), 100);
    expect_decision(after, &c["expected_after_explicit_projection"], "explicit ITC projection");
}

#[test]
fn frs_projection_needs_explicit_source_observation() {
    let c = case("frs-projection-is-not-source-observation");
    let e = evidence(c["evidence_profile"].as_str().unwrap());
    let mut b = Bindings::default();
    apply_bindings(&mut b, &c["initial_bindings"]);
    let before = evaluate_negative("COS-N-010", &b, Some(&e), 100);
    expect_decision(before, &c["expected_without_source_observation"], "FRS projection is not a source fact");
    assert!(!b.source_observation_created);
    apply_bindings(&mut b, &c["additional_bindings"]);
    let after = evaluate_negative("COS-N-010", &b, Some(&e), 100);
    expect_decision(after, &c["expected_after_explicit_source_observation"], "explicit source observation");
}

#[test]
fn recommendation_requires_both_authority_and_effect_receipt() {
    let c = case("recommendation-does-not-create-effect");
    let e = evidence(c["evidence_profile"].as_str().unwrap());
    let mut b = Bindings::default();
    apply_bindings(&mut b, &c["initial_bindings"]);
    let before = evaluate_negative("COS-N-014", &b, Some(&e), 100);
    expect_decision(before, &c["expected_without_authority"], "recommendation alone");
    apply_bindings(&mut b, &c["authority_bindings"]);
    let authorized_without_effect = evaluate_negative("COS-N-014", &b, Some(&e), 100);
    expect_decision(authorized_without_effect, &c["expected_with_authority_but_without_effect"], "authority is not an effect receipt");
    assert!(!b.effect_recorded);
    apply_bindings(&mut b, &c["effect_bindings"]);
    let after = evaluate_negative("COS-N-014", &b, Some(&e), 100);
    expect_decision(after, &c["expected_after_authorized_effect_receipt"], "authorized effect receipt");
}

#[test]
fn foreign_evidence_retains_origin_after_recognition() {
    let c = case("foreign-recognition-preserves-origin");
    let foreign = evidence(c["evidence_profile"].as_str().unwrap());
    assert_eq!(foreign.origin, Origin::Foreign("node-b".to_owned()));
    let mut b = Bindings::default();
    apply_bindings(&mut b, &c["initial_bindings"]);
    let before = evaluate_negative("COS-N-013", &b, Some(&foreign), 100);
    expect_decision(before, &c["expected_without_recognition"], "foreign evidence requires explicit recognition");
    apply_bindings(&mut b, &c["additional_bindings"]);
    let after = evaluate_negative("COS-N-013", &b, Some(&foreign), 100);
    expect_decision(after, &c["expected_decision"], "recognized foreign evidence");
    assert_eq!(foreign.origin, Origin::Foreign("node-b".to_owned()));
    assert_eq!(c["expected_origin_before"], "foreign");
    assert_eq!(c["expected_origin_after"], "foreign");
}

#[test]
fn output_qualification_does_not_bind_physical_work() {
    let c = case("qualified-output-is-not-physical-work");
    let e = evidence(c["evidence_profile"].as_str().unwrap());
    let mut b = Bindings::default();
    apply_bindings(&mut b, &c["initial_bindings"]);
    assert!(b.itc_projection, "the negative test must start with an actual ITC projection");
    assert!(b.output_qualification, "the negative test must start with output qualification");
    let before = evaluate_negative("COS-N-012", &b, Some(&e), 100);
    expect_decision(before, &c["expected_without_physical_work_binding"], "ITC plus qualified output is not physical-work evidence");
    assert!(!b.physical_work_binding);
    apply_bindings(&mut b, &c["additional_bindings"]);
    let after = evaluate_negative("COS-N-012", &b, Some(&e), 100);
    expect_decision(after, &c["expected_after_physical_work_binding"], "physical work requires explicit binding");
}

#[test]
fn stale_evidence_is_not_current_evidence() {
    let c = case("stale-evidence-does-not-satisfy-current-requirement");
    let stale = evidence(c["evidence_profile"].as_str().unwrap());
    let current = evidence(c["positive_evidence_profile"].as_str().unwrap());
    let stale_decision = stale_evidence_is_rejected(&stale, 100);
    expect_decision(stale_decision, &c["expected_decision"], "stale evidence must remain stale");
    expect_decision(
        stale_evidence_is_rejected(&current, 100),
        &c["expected_current_decision"],
        "current evidence satisfies the reference freshness check",
    );
}

#[test]
fn unknown_transition_stays_unknown() {
    let c = case("unknown-transition-remains-unknown");
    let e = evidence(c["evidence_profile"].as_str().unwrap());
    let mut b = Bindings::default();
    apply_bindings(&mut b, &c["initial_bindings"]);
    let decision = evaluate_negative("COS-N-999", &b, Some(&e), 100);
    expect_decision(decision, &c["expected_decision"], "unknown transition must remain indeterminate");
}

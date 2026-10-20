//! D6S-CANON-2 authority/provenance negative controls.
//! These are model-level mutations; they do not test plant actuation.

use cos_conformance::source_refinement::{mrp_to_observed_work, mrp_to_scheduled_operation};
use cos_conformance::{
    authorize_recommendation, bind_general_capability, bind_physical_work,
    bind_plan_to_consumption, bind_requirement_to_availability, conformance_report_json,
    create_source_observation, evaluate_negative, preserve_failure_history, project_to_frs,
    project_to_itc, qualify_output, recognize_foreign_evidence, record_effect, Bindings,
    Decision, Evidence, Origin,
};
use hdi::prelude::Timestamp;
use manufacturing_common::{MrpResult, ScheduledOperation};
use serde_json::Value;

fn mrp_fixture() -> MrpResult {
    let at = Timestamp::from_micros(1_700_000_000_000_000);
    MrpResult {
        planned_orders: vec![],
        scheduled_operations: vec![ScheduledOperation {
            work_order_id: "D6S-WO-001".into(),
            operation_name: "Mill".into(),
            machine_id: "D6S-CNC-001".into(),
            start_time: at,
            end_time: at,
            setup_minutes: 5,
            run_minutes: 20,
        }],
        capacity_warnings: vec![],
        material_shortages: vec![],
        feasible: true,
    }
}

const FIXTURE: &str =
    include_str!("../../../../docs/integral/d6s-canon-2-authority-boundary-fixture.json");

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("checked-in authority-boundary JSON must parse")
}

fn case(id: &str) -> Value {
    fixture()["mutations"]
        .as_array()
        .expect("mutations")
        .iter()
        .find(|entry| entry["id"].as_str() == Some(id))
        .unwrap_or_else(|| panic!("missing authority boundary fixture: {id}"))
        .clone()
}

fn decision_name(decision: Decision) -> &'static str {
    match decision {
        Decision::Accepted => "Accepted",
        Decision::Rejected => "Rejected",
        Decision::Unknown => "Unknown",
        Decision::Conflicting => "Conflicting",
        Decision::Stale => "Stale",
        Decision::Superseded => "Superseded",
        Decision::Unauthorized => "Unauthorized",
        Decision::Unbound => "Unbound",
    }
}

#[test]
fn authority_fixture_is_versioned_complete_and_only_reference_model() {
    let f = fixture();
    assert_eq!(f["schema_version"], "d6s-canon-2-authority-boundary-fixture/1");
    assert_eq!(f["status"], "ReferenceModelOnly");
    assert_eq!(f["corpus_id"], cos_conformance::CORPUS_ID);
    assert_eq!(f["mutations"].as_array().expect("mutations").len(), 10);
    let report: Value = serde_json::from_str(&conformance_report_json()).expect("report");
    assert_eq!(report["claim_ceiling"], "Semantic conformance of this reference harness only; no physical, safety, economic, ecological, or Integral-validation claim.");
    assert!(f["claim_ceiling"].as_str().expect("claim ceiling").contains("do not prove runtime authorization"));
}

#[test]
fn planned_material_does_not_become_consumption_without_binding() {
    let f = case("planned_input_is_not_consumption");
    let mut bindings = Bindings::default();
    let evidence = Evidence::current_local("d6s-material", 100);
    let negative = evaluate_negative("COS-N-002", &bindings, Some(&evidence), 100);
    assert_eq!(decision_name(negative), f["expected_without_binding"]);
    bind_plan_to_consumption(&mut bindings);
    let positive = evaluate_negative("COS-N-002", &bindings, Some(&evidence), 100);
    assert_eq!(decision_name(positive), f["expected_with_binding"]);
}

#[test]
fn scheduling_never_implicitly_creates_observed_work() {
    let f = case("scheduled_operation_is_not_observed_work");
    assert_eq!(f["expected_without_binding"], "Rejected");
    assert_eq!(f["expected_with_binding"], "NotAutomaticallyPromoted");
    // A schedule is valid planning evidence but the adapter must never
    // promote it to observed work merely because an operation was scheduled.
    let planned = mrp_fixture();
    assert_eq!(mrp_to_scheduled_operation(&planned).decision, Decision::Accepted);
    assert_eq!(mrp_to_observed_work(&planned).decision, Decision::Rejected);
}

#[test]
fn frs_projection_is_not_a_source_observation() {
    let f = case("finding_is_not_source_observation");
    let mut bindings = Bindings::default();
    project_to_frs(&mut bindings);
    assert!(!bindings.source_observation_created);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-010", &bindings, None, 100)),
        f["expected_without_binding"]
    );
    create_source_observation(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-010", &bindings, None, 100)),
        f["expected_with_binding"]
    );
}

#[test]
fn recommendation_needs_both_authorization_and_effect_receipt() {
    let f = case("recommendation_needs_authorization_and_effect");
    let mut bindings = Bindings::default();
    assert_eq!(
        decision_name(evaluate_negative("COS-N-014", &bindings, None, 100)),
        f["expected_without_binding"]
    );
    authorize_recommendation(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-014", &bindings, None, 100)),
        f["expected_authorization_only"]
    );
    record_effect(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-014", &bindings, None, 100)),
        f["expected_with_authorization_and_effect"]
    );
}

#[test]
fn unknown_delivery_outcome_stays_unknown() {
    let f = case("unknown_delivery_is_not_failure");
    let bindings = Bindings::default();
    let evidence = Evidence::current_local("d6s-delivery", 100);
    assert_eq!(
        decision_name(evaluate_negative(
            "UNSPECIFIED-DELIVERY-OUTCOME",
            &bindings,
            Some(&evidence),
            100
        )),
        f["expected_without_binding"]
    );
}

#[test]
fn foreign_evidence_keeps_foreign_origin_when_recognized() {
    let f = case("foreign_origin_survives_recognition");
    let evidence = Evidence::current_foreign("d6s-foreign", "node-b", 100);
    let mut bindings = Bindings::default();
    assert_eq!(
        decision_name(evaluate_negative("COS-N-013", &bindings, Some(&evidence), 100)),
        f["expected_without_binding"]
    );
    recognize_foreign_evidence(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-013", &bindings, Some(&evidence), 100)),
        f["expected_with_binding"]
    );
    assert_eq!(f["expected_origin"], "Foreign(node-b)");
    assert_eq!(evidence.origin, Origin::Foreign("node-b".to_owned()));
}

#[test]
fn qualification_does_not_mint_physical_work() {
    let f = case("qualification_does_not_prove_physical_work");
    let mut bindings = Bindings::default();
    qualify_output(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-012", &bindings, None, 100)),
        f["expected_without_binding"]
    );
    assert!(!bindings.physical_work_binding);
    bind_physical_work(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-012", &bindings, None, 100)),
        f["expected_with_binding"]
    );
}

#[test]
fn later_success_does_not_erase_the_failure_flag() {
    let f = case("later_success_preserves_failure_history");
    let mut bindings = Bindings::default();
    preserve_failure_history(&mut bindings);
    qualify_output(&mut bindings);
    assert!(bindings.failure_history_preserved);
    assert!(bindings.output_qualification);
    assert_eq!(f["expected_after_later_success"], "Preserved");
}

#[test]
fn availability_requires_an_explicit_temporal_binding() {
    let f = case("availability_requires_temporal_binding");
    let mut bindings = Bindings::default();
    let evidence = Evidence::current_local("d6s-availability", 100);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-004", &bindings, Some(&evidence), 100)),
        f["expected_without_binding"]
    );
    bind_requirement_to_availability(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-004", &bindings, Some(&evidence), 100)),
        f["expected_with_binding"]
    );
}

#[test]
fn single_success_does_not_establish_general_capability() {
    let f = case("single_success_is_not_general_capability");
    let mut bindings = Bindings::default();
    let evidence = Evidence::current_local("d6s-single-run", 100);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-007", &bindings, Some(&evidence), 100)),
        f["expected_without_binding"]
    );
    bind_general_capability(&mut bindings);
    assert_eq!(
        decision_name(evaluate_negative("COS-N-007", &bindings, Some(&evidence), 100)),
        f["expected_with_binding"]
    );
}

#[test]
fn source_observation_does_not_implicitly_create_itc_projection() {
    let mut bindings = Bindings::default();
    create_source_observation(&mut bindings);
    assert!(!bindings.itc_projection);
    let evidence = Evidence::current_local("d6s-itc", 100);
    assert_eq!(
        evaluate_negative("COS-N-011", &bindings, Some(&evidence), 100),
        Decision::Rejected
    );
    project_to_itc(&mut bindings);
    assert_eq!(
        evaluate_negative("COS-N-011", &bindings, Some(&evidence), 100),
        Decision::Accepted
    );
}

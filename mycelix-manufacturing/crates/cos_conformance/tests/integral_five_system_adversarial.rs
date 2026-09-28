//! Oracle-hidden five-system Integral interoperability mutations.
//!
//! These tests intentionally use only the neutral COS conformance vocabulary.
//! They validate semantic boundaries between OAD/COS/ITC/FRS/CDS without
//! pretending to implement Integral's runtime or normative governance model.

use cos_conformance::{
    authorize_recommendation, bind_physical_work, create_source_observation,
    preserve_failure_history, project_to_frs, project_to_itc, qualify_output,
    recognize_foreign_evidence, record_effect, Bindings, Decision, Evidence,
};

fn fresh_local() -> Evidence {
    Evidence::current_local("evidence-local-001", 100)
}

fn fresh_foreign() -> Evidence {
    Evidence::current_foreign("decision-foreign-001", "node-b", 100)
}

#[test]
fn stale_oad_design_cannot_become_current_production_basis() {
    let stale = Evidence::stale("oad-design-v1", 100);
    assert_eq!(
        cos_conformance::stale_evidence_is_rejected(&stale, 100),
        Decision::Stale
    );
}

#[test]
fn planned_labor_cannot_become_observed_labor_without_binding() {
    let bindings = Bindings::default();
    assert_eq!(
        cos_conformance::evaluate_negative(
            "COS-N-001",
            &bindings,
            Some(&fresh_local()),
            100,
        ),
        Decision::Rejected
    );
}

#[test]
fn planned_material_cannot_become_consumed_material_without_binding() {
    let bindings = Bindings::default();
    assert_eq!(
        cos_conformance::evaluate_negative(
            "COS-N-002",
            &bindings,
            Some(&fresh_local()),
            100,
        ),
        Decision::Rejected
    );
}

#[test]
fn duplicate_labor_projection_requires_explicit_itc_projection() {
    let mut bindings = Bindings::default();
    create_source_observation(&mut bindings);
    assert!(!bindings.itc_projection);
    project_to_itc(&mut bindings);
    assert!(bindings.itc_projection);
}

#[test]
fn frs_finding_cannot_become_source_observation_implicitly() {
    let mut bindings = Bindings::default();
    project_to_frs(&mut bindings);
    assert!(!bindings.source_observation_created);
    create_source_observation(&mut bindings);
    assert!(bindings.source_observation_created);
}

#[test]
fn frs_recommendation_cannot_execute_without_authorization_and_effect_receipt() {
    let mut bindings = Bindings::default();
    assert_eq!(
        cos_conformance::evaluate_negative(
            "COS-N-014",
            &bindings,
            Some(&fresh_local()),
            100,
        ),
        Decision::Unauthorized
    );

    authorize_recommendation(&mut bindings);
    assert_eq!(
        cos_conformance::evaluate_negative(
            "COS-N-014",
            &bindings,
            Some(&fresh_local()),
            100,
        ),
        Decision::Unauthorized
    );

    record_effect(&mut bindings);
    assert_eq!(
        cos_conformance::evaluate_negative(
            "COS-N-014",
            &bindings,
            Some(&fresh_local()),
            100,
        ),
        Decision::Accepted
    );
}

#[test]
fn timeout_or_unknown_delivery_is_not_a_failure_receipt() {
    // The COS model deliberately has no transport-level "timeout => failed"
    // coercion. Unknown remains outside the Accepted state until an explicit
    // semantic transition supplies the missing evidence.
    let unknown = Bindings::default();
    assert_eq!(
        cos_conformance::evaluate_negative(
            "UNSPECIFIED-DELIVERY-OUTCOME",
            &unknown,
            Some(&fresh_local()),
            100,
        ),
        Decision::Unknown
    );
}

#[test]
fn foreign_decision_never_becomes_local_origin_by_recognition() {
    let foreign = fresh_foreign();
    let mut bindings = Bindings::default();
    recognize_foreign_evidence(&mut bindings);
    assert!(bindings.foreign_recognition);
    assert_eq!(
        foreign.origin,
        cos_conformance::Origin::Foreign("node-b".to_owned())
    );
}

#[test]
fn derived_summary_cannot_overwrite_source_fact() {
    let mut bindings = Bindings::default();
    project_to_frs(&mut bindings);
    assert!(bindings.frs_projection);
    assert!(!bindings.source_observation_created);
}

#[test]
fn historical_failure_survives_later_success() {
    let mut bindings = Bindings::default();
    preserve_failure_history(&mut bindings);
    qualify_output(&mut bindings);
    assert!(bindings.failure_history_preserved);
    assert!(bindings.output_qualification);
}

#[test]
fn certified_or_qualified_output_does_not_mint_physical_work() {
    let mut bindings = Bindings::default();
    qualify_output(&mut bindings);
    assert!(!bindings.physical_work_binding);
    bind_physical_work(&mut bindings);
    assert!(bindings.physical_work_binding);
}

#[test]
fn five_system_chain_keeps_each_boundary_explicit() {
    // OAD evidence -> COS observation -> ITC/FRS projections -> CDS
    // authorization -> effect receipt. Each edge is explicit.
    let mut bindings = Bindings::default();

    create_source_observation(&mut bindings);
    assert!(bindings.source_observation_created);

    project_to_itc(&mut bindings);
    project_to_frs(&mut bindings);
    assert!(bindings.itc_projection);
    assert!(bindings.frs_projection);

    authorize_recommendation(&mut bindings);
    assert!(bindings.recommendation_authorized);
    assert!(!bindings.effect_recorded);

    record_effect(&mut bindings);
    assert!(bindings.effect_recorded);

    // Qualification and physical execution remain separate even after the
    // five-system semantic chain closes.
    qualify_output(&mut bindings);
    assert!(bindings.output_qualification);
    assert!(bindings.physical_work_binding == false);
}

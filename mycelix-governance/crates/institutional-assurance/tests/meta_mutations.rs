use std::collections::BTreeSet;

use institutional_assurance::capture::{
    CaptureDisposition, CapturePolicy, CaptureScenario, ControlDomain, DomainControl,
    evaluate_capture,
};

fn actors(values: &[&str]) -> BTreeSet<String> {
    values.iter().map(|value| (*value).to_owned()).collect()
}

fn captured_fixture() -> CaptureScenario {
    CaptureScenario {
        scenario_id: "ipb-meta-s3-captured-baseline".to_owned(),
        coalition_id: "declared-coalition".to_owned(),
        coalition_members: actors(&["cartel-a", "cartel-b"]),
        policy: CapturePolicy {
            policy_id: "ipb-meta-s3-policy".to_owned(),
            version: 1,
            execution_domains: BTreeSet::from([ControlDomain::Agenda]),
            oversight_domains: BTreeSet::from([ControlDomain::Audit]),
            minimum_compromised_oversight_domains: 1,
            minimum_independent_oversight_controllers_per_domain: 1,
        },
        domains: vec![
            DomainControl {
                domain: ControlDomain::Agenda,
                authorized_controllers: actors(&["cartel-a", "cartel-b"]),
                available_controllers: actors(&["cartel-a", "cartel-b"]),
                required_signers: 2,
            },
            DomainControl {
                domain: ControlDomain::Audit,
                authorized_controllers: actors(&["cartel-a", "cartel-b"]),
                available_controllers: actors(&["cartel-a", "cartel-b"]),
                required_signers: 1,
            },
        ],
    }
}

fn safe_fixture_with_independent_oversight() -> CaptureScenario {
    let mut scenario = captured_fixture();
    let audit = scenario
        .domains
        .iter_mut()
        .find(|domain| domain.domain == ControlDomain::Audit)
        .expect("audit domain exists");
    audit
        .authorized_controllers
        .insert("independent-reviewer".to_owned());
    audit
        .available_controllers
        .insert("independent-reviewer".to_owned());
    scenario
}

#[test]
fn domain_vector_permutation_preserves_semantics_but_not_input_provenance() {
    let original_scenario = captured_fixture();
    let original = evaluate_capture(&original_scenario).expect("baseline is valid");

    let mut permuted_scenario = original_scenario.clone();
    permuted_scenario.domains.reverse();
    let permuted = evaluate_capture(&permuted_scenario).expect("permuted input is valid");

    assert_eq!(original.disposition, permuted.disposition);
    assert_eq!(
        original.coalition_controlled_execution_domains,
        permuted.coalition_controlled_execution_domains
    );
    assert_eq!(
        original.compromised_oversight_domains,
        permuted.compromised_oversight_domains
    );
    assert_eq!(original.domain_assessments, permuted.domain_assessments);

    // The transformation preserves the evaluated meaning, not the original
    // evidence bytes. Provenance digests must therefore remain distinguishable.
    assert_ne!(original.input_sha256, permuted.input_sha256);
    assert_ne!(original.receipt_sha256, permuted.receipt_sha256);
}

#[test]
fn removing_the_only_independent_auditor_changes_safe_control_into_capture() {
    let safe_scenario = safe_fixture_with_independent_oversight();
    let safe = evaluate_capture(&safe_scenario).expect("safe control is valid");
    assert_eq!(safe.disposition, CaptureDisposition::NoCaptureObserved);

    let mut mutated_scenario = safe_scenario;
    let audit = mutated_scenario
        .domains
        .iter_mut()
        .find(|domain| domain.domain == ControlDomain::Audit)
        .expect("audit domain exists");
    audit
        .available_controllers
        .remove("independent-reviewer");

    let mutated =
        evaluate_capture(&mutated_scenario).expect("availability mutation remains well-formed");
    assert_eq!(mutated.disposition, CaptureDisposition::CaptureDetected);
    assert_eq!(
        mutated.compromised_oversight_domains,
        vec![ControlDomain::Audit]
    );
}

#[test]
fn loss_of_execution_quorum_is_not_misreported_as_no_capture() {
    let mut scenario = captured_fixture();
    let agenda = scenario
        .domains
        .iter_mut()
        .find(|domain| domain.domain == ControlDomain::Agenda)
        .expect("agenda domain exists");
    agenda.available_controllers.remove("cartel-b");

    let receipt = evaluate_capture(&scenario).expect("unavailable quorum is a valid scenario");
    assert_eq!(
        receipt.disposition,
        CaptureDisposition::ExecutionUnavailable
    );
}

#[test]
fn unauthorized_active_controller_is_rejected_instead_of_counted() {
    let mut scenario = captured_fixture();
    let agenda = scenario
        .domains
        .iter_mut()
        .find(|domain| domain.domain == ControlDomain::Agenda)
        .expect("agenda domain exists");
    agenda
        .available_controllers
        .insert("unauthorized-controller".to_owned());

    assert_eq!(
        evaluate_capture(&scenario),
        Err(institutional_assurance::capture::CaptureInputError::AvailableControllerNotAuthorized)
    );
}

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
    audit.available_controllers.remove("independent-reviewer");

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

fn seven_domain_fixture() -> CaptureScenario {
    let coalition = actors(&["cartel-a", "cartel-b"]);
    let controllers = actors(&["cartel-a", "cartel-b", "independent-c", "independent-d"]);
    let mut domains = Vec::new();
    for domain in [
        ControlDomain::Agenda,
        ControlDomain::Appointments,
        ControlDomain::Authorization,
        ControlDomain::TreasuryCustody,
        ControlDomain::Audit,
        ControlDomain::Appeal,
    ] {
        domains.push(DomainControl {
            domain,
            authorized_controllers: controllers.clone(),
            available_controllers: controllers.clone(),
            required_signers: 2,
        });
    }
    domains.push(DomainControl {
        domain: ControlDomain::ConstitutionalAmendment,
        authorized_controllers: actors(&["independent-c", "independent-d", "independent-e"]),
        available_controllers: actors(&["independent-c", "independent-d", "independent-e"]),
        required_signers: 3,
    });

    CaptureScenario {
        scenario_id: "ipb-meta-seven-domain-baseline".to_owned(),
        coalition_id: "declared-coalition".to_owned(),
        coalition_members: coalition,
        policy: CapturePolicy {
            policy_id: "ipb-meta-seven-domain-policy".to_owned(),
            version: 1,
            execution_domains: BTreeSet::from([
                ControlDomain::Agenda,
                ControlDomain::Appointments,
                ControlDomain::Authorization,
                ControlDomain::TreasuryCustody,
            ]),
            oversight_domains: BTreeSet::from([ControlDomain::Audit, ControlDomain::Appeal]),
            minimum_compromised_oversight_domains: 2,
            minimum_independent_oversight_controllers_per_domain: 1,
        },
        domains,
    }
}

fn next_permutation<T: Ord>(items: &mut [T]) -> bool {
    if items.len() < 2 {
        return false;
    }
    let Some(pivot) = (0..items.len() - 1)
        .rev()
        .find(|&index| items[index] < items[index + 1])
    else {
        return false;
    };
    let successor = (pivot + 1..items.len())
        .rev()
        .find(|&index| items[pivot] < items[index])
        .expect("a successor exists after a valid pivot");
    items.swap(pivot, successor);
    items[pivot + 1..].reverse();
    true
}

#[test]
fn all_5040_domain_vector_permutations_preserve_semantics_and_distinct_provenance() {
    let baseline = seven_domain_fixture();
    let expected = evaluate_capture(&baseline).expect("baseline is valid");
    assert_eq!(expected.disposition, CaptureDisposition::NoCaptureObserved);

    // Every one of the 7! possible serializations is evaluated. This tests
    // ordering invariance exhaustively for this fixed seven-domain topology.
    let mut indices: Vec<usize> = (0..baseline.domains.len()).collect();
    let mut input_digests = BTreeSet::new();
    let mut permutation_count = 0usize;

    loop {
        let mut candidate = baseline.clone();
        candidate.domains = indices
            .iter()
            .map(|&index| baseline.domains[index].clone())
            .collect();
        let receipt = evaluate_capture(&candidate).expect("permuted scenario remains valid");
        assert_eq!(receipt.disposition, expected.disposition);
        assert_eq!(
            receipt.coalition_controlled_execution_domains,
            expected.coalition_controlled_execution_domains
        );
        assert_eq!(
            receipt.compromised_oversight_domains,
            expected.compromised_oversight_domains
        );
        assert_eq!(receipt.domain_assessments, expected.domain_assessments);
        input_digests.insert(receipt.input_sha256);
        permutation_count += 1;

        if !next_permutation(&mut indices) {
            break;
        }
    }

    assert_eq!(permutation_count, 5_040);
    assert_eq!(input_digests.len(), 5_040);
}

#[test]
fn oversight_loss_threshold_changes_only_when_the_declared_number_of_domains_is_compromised() {
    let mut scenario = seven_domain_fixture();
    let baseline = evaluate_capture(&scenario).expect("baseline is valid");
    assert_eq!(baseline.disposition, CaptureDisposition::NoCaptureObserved);

    let audit = scenario
        .domains
        .iter_mut()
        .find(|domain| domain.domain == ControlDomain::Audit)
        .expect("audit domain exists");
    audit.available_controllers.remove("independent-d");

    let one_domain_lost = evaluate_capture(&scenario).expect("single-domain loss is valid");
    assert_eq!(
        one_domain_lost.compromised_oversight_domains,
        vec![ControlDomain::Audit]
    );
    assert_eq!(
        one_domain_lost.disposition,
        CaptureDisposition::NoCaptureObserved
    );

    let appeal = scenario
        .domains
        .iter_mut()
        .find(|domain| domain.domain == ControlDomain::Appeal)
        .expect("appeal domain exists");
    appeal.available_controllers.remove("independent-d");

    let two_domains_lost = evaluate_capture(&scenario).expect("two-domain loss is valid");
    assert_eq!(
        two_domains_lost.compromised_oversight_domains,
        vec![ControlDomain::Audit, ControlDomain::Appeal]
    );
    assert_eq!(
        two_domains_lost.disposition,
        CaptureDisposition::CaptureDetected
    );
}

#[test]
fn loss_of_one_critical_execution_quorum_breaks_the_full_capture_predicate() {
    let mut scenario = seven_domain_fixture();
    let baseline = evaluate_capture(&scenario).expect("baseline is valid");
    assert_eq!(baseline.disposition, CaptureDisposition::NoCaptureObserved);

    // Make oversight unavailable to independent controllers in both domains.
    for domain in [ControlDomain::Audit, ControlDomain::Appeal] {
        let control = scenario
            .domains
            .iter_mut()
            .find(|control| control.domain == domain)
            .expect("oversight domain exists");
        control.available_controllers.remove("independent-c");
        control.available_controllers.remove("independent-d");
    }
    let oversight_lost = evaluate_capture(&scenario).expect("oversight-loss scenario is valid");
    assert_eq!(
        oversight_lost.disposition,
        CaptureDisposition::CaptureDetected
    );

    // The declared coalition can no longer meet the quorum in one critical
    // execution domain; loss of execution capability takes precedence.
    let treasury = scenario
        .domains
        .iter_mut()
        .find(|control| control.domain == ControlDomain::TreasuryCustody)
        .expect("treasury domain exists");
    treasury.available_controllers.remove("cartel-b");
    let execution_lost = evaluate_capture(&scenario).expect("execution-loss scenario is valid");
    assert_eq!(
        execution_lost.disposition,
        CaptureDisposition::NoCaptureObserved
    );
}

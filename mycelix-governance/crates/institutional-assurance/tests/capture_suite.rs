use std::collections::BTreeSet;
use std::process::Command;

use institutional_assurance::capture::{
    CaptureDisposition, CapturePolicy, CaptureScenario, CaptureSuiteReport, ControlDomain,
    DomainControl, adversarial_capture_suite, evaluate_capture,
};

#[test]
fn all_s3_known_answer_cases_match_their_expected_disposition() {
    let report = adversarial_capture_suite();
    assert_eq!(report.total_cases, 9);
    assert_eq!(report.passed_cases, 9);
    assert_eq!(report.failed_cases, 0);

    let captured = report
        .cases
        .iter()
        .filter(|case| case.expected_disposition == CaptureDisposition::CaptureDetected)
        .count();
    let controls = report
        .cases
        .iter()
        .filter(|case| case.expected_disposition == CaptureDisposition::NoCaptureObserved)
        .count();
    let unavailable = report
        .cases
        .iter()
        .filter(|case| case.expected_disposition == CaptureDisposition::ExecutionUnavailable)
        .count();

    assert_eq!(captured, 3);
    assert_eq!(controls, 5);
    assert_eq!(unavailable, 1);
}

#[test]
fn ipb_s3_cli_emits_a_complete_machine_readable_report() {
    let output = Command::new(env!("CARGO_BIN_EXE_ipb-s3-run"))
        .output()
        .expect("S3 runner binary starts");

    assert!(
        output.status.success(),
        "S3 runner must exit successfully; stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let report: CaptureSuiteReport =
        serde_json::from_slice(&output.stdout).expect("S3 runner emits valid JSON");
    assert_eq!(report.total_cases, 9);
    assert_eq!(report.passed_cases, 9);
    assert_eq!(report.failed_cases, 0);
}

#[test]
fn receipts_expose_the_declared_controller_topology() {
    let report = adversarial_capture_suite();
    let case = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s3-attack-cross-domain-coalition")
        .expect("cross-domain coalition fixture exists");
    let receipt = &case.receipt;

    assert_eq!(receipt.policy_id, "ipb-s3-fixture-policy");
    assert_eq!(receipt.policy_version, 1);
    let expected_ids = vec!["cartel-a".to_owned(), "cartel-b".to_owned()];
    assert_eq!(receipt.coalition_members, expected_ids);

    let agenda = receipt
        .domain_assessments
        .iter()
        .find(|assessment| {
            assessment.domain == institutional_assurance::capture::ControlDomain::Agenda
        })
        .expect("agenda assessment exists");
    assert_eq!(agenda.authorized_controllers, expected_ids);
    assert_eq!(agenda.available_controllers, expected_ids);
    assert_eq!(agenda.coalition_available_controllers, expected_ids);
    assert!(agenda.independent_available_controllers.is_empty());
    assert_eq!(
        agenda.available_controller_count,
        agenda.available_controllers.len()
    );
}

#[test]
fn every_receipt_binds_policy_input_and_receipt_material() {
    let report = adversarial_capture_suite();
    for case in report.cases {
        assert!(case.receipt.policy_sha256.starts_with("sha256:"));
        assert!(case.receipt.input_sha256.starts_with("sha256:"));
        assert!(case.receipt.receipt_sha256.starts_with("sha256:"));
        assert_eq!(case.receipt.policy_sha256.len(), 71);
        assert_eq!(case.receipt.input_sha256.len(), 71);
        assert_eq!(case.receipt.receipt_sha256.len(), 71);
    }
}

#[test]
fn repeated_runs_produce_identical_s3_receipts() {
    let first = adversarial_capture_suite();
    let second = adversarial_capture_suite();
    assert_eq!(
        serde_json::to_vec(&first).expect("first S3 report serializes"),
        serde_json::to_vec(&second).expect("second S3 report serializes")
    );
}

#[test]
fn captured_execution_with_independent_oversight_is_not_classified_as_capture() {
    let report = adversarial_capture_suite();
    let case = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s3-control-independent-audit-and-appeal")
        .expect("independent-oversight control exists");
    assert_eq!(
        case.receipt.disposition,
        CaptureDisposition::NoCaptureObserved
    );
    assert!(case.receipt.compromised_oversight_domains.is_empty());
}

#[test]
fn coalition_access_does_not_compromise_oversight_when_an_independent_quorum_also_exists() {
    let report = adversarial_capture_suite();
    let case = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s3-control-dual-independent-oversight-quorums")
        .expect("dual independent oversight control exists");
    assert_eq!(
        case.receipt.disposition,
        CaptureDisposition::NoCaptureObserved
    );
    for domain in [
        institutional_assurance::capture::ControlDomain::Audit,
        institutional_assurance::capture::ControlDomain::Appeal,
    ] {
        let assessment = case
            .receipt
            .domain_assessments
            .iter()
            .find(|assessment| assessment.domain == domain)
            .expect("oversight domain assessment exists");
        assert!(assessment.coalition_can_exercise);
        assert!(assessment.independent_oversight_ready);
        assert!(!assessment.oversight_compromised);
    }
}

#[test]
fn oversight_capture_without_execution_control_is_not_full_capture() {
    let report = adversarial_capture_suite();
    let case = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s3-control-oversight-captured-without-execution")
        .expect("oversight-only control exists");
    assert_eq!(
        case.receipt.disposition,
        CaptureDisposition::NoCaptureObserved
    );
    assert_eq!(case.receipt.coalition_controlled_execution_domains.len(), 0);
}

#[test]
fn unavailable_execution_is_not_misreported_as_capture_resistance() {
    let report = adversarial_capture_suite();
    let case = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s3-stress-independent-execution-quorum-unavailable")
        .expect("unavailable-execution control exists");
    assert_eq!(
        case.receipt.disposition,
        CaptureDisposition::ExecutionUnavailable
    );
}


fn actor_set(actors: &[&str]) -> BTreeSet<String> {
    actors.iter().map(|actor| (*actor).to_owned()).collect()
}

fn metamorphic_scenario(
    coalition: &[&str],
    execution_available: &[&str],
    oversight_available: &[&str],
    required_signers: usize,
    minimum_independent: usize,
) -> CaptureScenario {
    let execution_domains = BTreeSet::from([
        ControlDomain::Agenda,
        ControlDomain::Appointments,
        ControlDomain::Authorization,
        ControlDomain::TreasuryCustody,
    ]);
    let oversight_domains =
        BTreeSet::from([ControlDomain::Audit, ControlDomain::Appeal]);
    let all_authorized = ["a", "b", "c", "d", "e", "f"];
    let mut domains = Vec::new();

    for domain in [
        ControlDomain::Agenda,
        ControlDomain::Appointments,
        ControlDomain::Authorization,
        ControlDomain::TreasuryCustody,
    ] {
        domains.push(DomainControl {
            domain,
            authorized_controllers: actor_set(&all_authorized),
            available_controllers: actor_set(execution_available),
            required_signers,
        });
    }
    for domain in [ControlDomain::Audit, ControlDomain::Appeal] {
        domains.push(DomainControl {
            domain,
            authorized_controllers: actor_set(&all_authorized),
            available_controllers: actor_set(oversight_available),
            required_signers,
        });
    }

    CaptureScenario {
        scenario_id: "metamorphic-s3-scenario".to_owned(),
        coalition_id: "metamorphic-coalition".to_owned(),
        coalition_members: actor_set(coalition),
        policy: CapturePolicy {
            policy_id: "metamorphic-policy".to_owned(),
            version: 1,
            execution_domains,
            oversight_domains,
            minimum_compromised_oversight_domains: 2,
            minimum_independent_oversight_controllers_per_domain: minimum_independent,
        },
        domains,
    }
}

fn semantic_signature(receipt: &institutional_assurance::capture::CaptureReceipt) -> Vec<(
    ControlDomain,
    usize,
    usize,
    bool,
    bool,
    bool,
)> {
    receipt
        .domain_assessments
        .iter()
        .map(|assessment| {
            (
                assessment.domain,
                assessment.coalition_available_count,
                assessment.independent_available_count,
                assessment.coalition_can_exercise,
                assessment.independent_oversight_ready,
                assessment.oversight_compromised,
            )
        })
        .collect()
}

#[test]
fn bijective_actor_relabeling_preserves_capture_semantics() {
    let original = metamorphic_scenario(
        &["a", "b", "c", "d"],
        &["a", "b", "c", "d", "e"],
        &["a", "b", "c", "d", "e"],
        2,
        2,
    );
    let original_receipt = evaluate_capture(&original).expect("original scenario validates");

    let mapping = [
        ("a", "actor-01"),
        ("b", "actor-02"),
        ("c", "actor-03"),
        ("d", "actor-04"),
        ("e", "actor-05"),
        ("f", "actor-06"),
    ];
    let rename = |actor: &str| {
        mapping
            .iter()
            .find(|(source, _)| *source == actor)
            .map_or_else(|| actor.to_owned(), |(_, target)| (*target).to_owned())
    };
    let mut renamed = original.clone();
    renamed.coalition_members = original
        .coalition_members
        .iter()
        .map(|actor| rename(actor))
        .collect();
    for domain in &mut renamed.domains {
        domain.authorized_controllers = domain
            .authorized_controllers
            .iter()
            .map(|actor| rename(actor))
            .collect();
        domain.available_controllers = domain
            .available_controllers
            .iter()
            .map(|actor| rename(actor))
            .collect();
    }

    let renamed_receipt = evaluate_capture(&renamed).expect("renamed scenario validates");
    assert_eq!(original_receipt.disposition, renamed_receipt.disposition);
    assert_eq!(
        semantic_signature(&original_receipt),
        semantic_signature(&renamed_receipt)
    );
}

#[test]
fn domain_input_order_does_not_change_s3_assessments() {
    let original = metamorphic_scenario(
        &["a", "b", "c", "d"],
        &["a", "b", "c", "d", "e"],
        &["a", "b", "c", "d", "e"],
        2,
        2,
    );
    let mut reversed = original.clone();
    reversed.domains.reverse();

    let original_receipt = evaluate_capture(&original).expect("original validates");
    let reversed_receipt = evaluate_capture(&reversed).expect("reversed validates");
    assert_eq!(original_receipt.disposition, reversed_receipt.disposition);
    assert_eq!(original_receipt.domain_assessments, reversed_receipt.domain_assessments);
    assert_eq!(
        original_receipt.coalition_controlled_execution_domains,
        reversed_receipt.coalition_controlled_execution_domains
    );
    assert_eq!(
        original_receipt.compromised_oversight_domains,
        reversed_receipt.compromised_oversight_domains
    );
}

#[test]
fn adding_coalition_members_cannot_erase_a_detected_capture() {
    let captured = metamorphic_scenario(
        &["a", "b", "c", "d"],
        &["a", "b", "c", "d", "e"],
        &["a", "b", "c", "d", "e"],
        2,
        2,
    );
    let mut expanded = captured.clone();
    expanded.coalition_members.insert("e".to_owned());

    let captured_receipt = evaluate_capture(&captured).expect("captured scenario validates");
    let expanded_receipt = evaluate_capture(&expanded).expect("expanded scenario validates");
    assert_eq!(captured_receipt.disposition, CaptureDisposition::CaptureDetected);
    assert_eq!(expanded_receipt.disposition, CaptureDisposition::CaptureDetected);
    assert_eq!(
        captured_receipt.coalition_controlled_execution_domains,
        expanded_receipt.coalition_controlled_execution_domains
    );
}

#[test]
fn restoring_independent_oversight_quorum_removes_capture_finding() {
    let captured = metamorphic_scenario(
        &["a", "b"],
        &["a", "b"],
        &["a", "b"],
        2,
        2,
    );
    let mut restored = captured.clone();
    for control in &mut restored.domains {
        if restored.policy.oversight_domains.contains(&control.domain) {
            control.available_controllers.extend(actor_set(&["c", "d"]));
        }
    }

    let captured_receipt = evaluate_capture(&captured).expect("captured scenario validates");
    let restored_receipt = evaluate_capture(&restored).expect("restored scenario validates");
    assert_eq!(captured_receipt.disposition, CaptureDisposition::CaptureDetected);
    assert_eq!(restored_receipt.disposition, CaptureDisposition::NoCaptureObserved);
    assert_eq!(restored_receipt.compromised_oversight_domains.len(), 0);
    for assessment in restored_receipt.domain_assessments.iter().filter(|assessment| {
        restored.policy.oversight_domains.contains(&assessment.domain)
    }) {
        assert!(assessment.independent_oversight_ready);
        assert!(!assessment.oversight_compromised);
    }
}

#[test]
fn increasing_execution_quorum_does_not_create_capture_or_hide_unavailability() {
    let mut scenario = metamorphic_scenario(
        &["a", "b", "c"],
        &["a", "b", "c", "d", "e"],
        &["a", "b", "c", "d", "e"],
        2,
        2,
    );
    for control in &mut scenario.domains {
        control.required_signers = 4;
    }
    let receipt = evaluate_capture(&scenario).expect("higher threshold scenario validates");
    assert_ne!(receipt.disposition, CaptureDisposition::CaptureDetected);
    assert_eq!(
        receipt.disposition,
        CaptureDisposition::NoCaptureObserved
    );
    assert!(receipt.coalition_controlled_execution_domains.is_empty());
}

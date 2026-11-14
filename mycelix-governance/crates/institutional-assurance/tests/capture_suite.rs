use std::process::Command;

use institutional_assurance::capture::{
    CaptureDisposition, CaptureSuiteReport, adversarial_capture_suite,
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

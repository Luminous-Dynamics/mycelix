use institutional_assurance::capture::{CaptureDisposition, adversarial_capture_suite};

#[test]
fn all_s3_known_answer_cases_match_their_expected_disposition() {
    let report = adversarial_capture_suite();
    assert_eq!(report.total_cases, 8);
    assert_eq!(report.passed_cases, 8);
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
    assert_eq!(controls, 4);
    assert_eq!(unavailable, 1);
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

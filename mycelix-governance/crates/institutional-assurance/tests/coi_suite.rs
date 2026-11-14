use institutional_assurance::{Decision, ReasonCode, SUITE_ID, conflict_of_interest_suite};

#[test]
fn all_s2_fixture_expectations_match() {
    let report = conflict_of_interest_suite();
    assert_eq!(report.suite_id, SUITE_ID);
    assert_eq!(report.total_cases, 20);
    assert_eq!(report.passed_cases, 20);
    assert_eq!(report.failed_cases, 0);
}

#[test]
fn conflicted_and_unauthorized_attempts_are_denied_for_distinct_reasons() {
    let report = conflict_of_interest_suite();

    let beneficiary = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s2-negative-conflicted-beneficiary")
        .expect("beneficiary negative control exists");
    assert_eq!(beneficiary.receipt.decision, Decision::Deny);
    assert_eq!(beneficiary.receipt.reason, ReasonCode::ConflictOfInterest);

    let missing_authority = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s2-negative-missing-authority")
        .expect("missing-authority negative control exists");
    assert_eq!(missing_authority.receipt.decision, Decision::Deny);
    assert_eq!(
        missing_authority.receipt.reason,
        ReasonCode::MissingAuthority
    );
}

#[test]
fn receipts_include_policy_input_and_output_sha256_digests() {
    let report = conflict_of_interest_suite();
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
fn repeated_runs_produce_identical_receipts() {
    let first = conflict_of_interest_suite();
    let second = conflict_of_interest_suite();
    assert_eq!(
        serde_json::to_vec(&first).expect("first run serializes"),
        serde_json::to_vec(&second).expect("second run serializes")
    );
}

#[test]
fn independent_authorized_approver_is_a_positive_control() {
    let report = conflict_of_interest_suite();
    let positive = report
        .cases
        .iter()
        .find(|case| case.scenario_id == "s2-positive-independent-authorized-approver")
        .expect("positive control exists");

    assert!(positive.passed);
    assert_eq!(positive.receipt.decision, Decision::Permit);
    assert_eq!(positive.receipt.reason, ReasonCode::Authorized);
}

#[test]
fn proposal_and_authority_boundary_controls_are_denied() {
    let report = conflict_of_interest_suite();
    let expected = [
        (
            "s2-negative-proposal-substitution",
            ReasonCode::ProposalMismatch,
        ),
        (
            "s2-negative-authority-holder-mismatch",
            ReasonCode::AuthorityHolderMismatch,
        ),
        (
            "s2-negative-authority-not-yet-valid",
            ReasonCode::AuthorityNotYetValid,
        ),
        (
            "s2-negative-invalid-authority-window",
            ReasonCode::InvalidAuthorityWindow,
        ),
        ("s2-negative-zero-amount", ReasonCode::InvalidAmount),
    ];

    for (scenario_id, reason) in expected {
        let case = report
            .cases
            .iter()
            .find(|case| case.scenario_id == scenario_id)
            .unwrap_or_else(|| panic!("missing negative control: {scenario_id}"));
        assert_eq!(case.receipt.decision, Decision::Deny);
        assert_eq!(case.receipt.reason, reason);
    }
}

#[test]
fn all_positive_controls_are_permitted() {
    let report = conflict_of_interest_suite();
    let positive: Vec<_> = report
        .cases
        .iter()
        .filter(|case| case.scenario_id.contains("-positive-"))
        .collect();

    assert_eq!(positive.len(), 6);
    for case in positive {
        assert!(case.passed, "positive control failed: {}", case.scenario_id);
        assert_eq!(case.receipt.decision, Decision::Permit);
        assert_eq!(case.receipt.reason, ReasonCode::Authorized);
    }
}

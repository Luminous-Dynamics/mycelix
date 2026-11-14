use fsre_service_universe_audit::audit;

const POSITIVE: &str = r#"
#audit_id=water-works-synthetic
#scope_id=municipal-wastewater
#unit_type=treatment-works
#study_start_seconds=0
#study_end_seconds=100
#reference_frame_id=dws-register
#reference_owner=source-owner-a
#reference_ref=register-v1
#reference_snapshot_id=register-snapshot-2026-01
#reference_valid_from_seconds=0
#reference_valid_until_seconds=100
#reference_coverage_basis=DECLARED_ENUMERATED_REGISTER
#observed_frame_id=independent-audit
#observed_owner=source-owner-b
#observed_ref=audit-register-v1
#observed_snapshot_id=audit-snapshot-2026-01
#observed_valid_from_seconds=0
#observed_valid_until_seconds=100
#crosswalk_owner=review-team
#crosswalk_ref=reviewed-crosswalk-v1
#crosswalk_snapshot_id=crosswalk-snapshot-2026-01
#crosswalk_valid_from_seconds=0
#crosswalk_valid_until_seconds=100
#crosswalk_independence_assessment=REVIEWED_INDEPENDENT
record_type|record_id|scope_status|reference_record_id|decision|evidence_owner|evidence_ref|valid_from_seconds|valid_until_seconds
REFERENCE|ref-a|IN_SCOPE|-|-|-|-|0|100
REFERENCE|ref-b|IN_SCOPE|-|-|-|-|0|100
REFERENCE|ref-c|IN_SCOPE|-|-|-|-|0|100
OBSERVED|obs-a|IN_SCOPE|-|-|-|-|0|100
OBSERVED|obs-b|IN_SCOPE|-|-|-|-|0|100
OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100
LINK|obs-a|IN_SCOPE|ref-a|MATCH|review-team|pair-a|0|100
LINK|obs-b|IN_SCOPE|ref-b|MATCH|review-team|pair-b|0|100
LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100
"#;

#[test]
fn exact_one_to_one_reconciliation_is_scoped_to_declared_register() {
    let report = audit(POSITIVE).expect("valid fixture");
    assert_eq!(report.disposition, "RECONCILED_TO_DECLARED_REFERENCE_FRAME");
    assert_eq!(report.reference_units, 3);
    assert_eq!(report.observed_units, 3);
    assert_eq!(report.matched_reference_units, 3);
    assert_eq!(report.omitted_reference_units, 0);
    assert_eq!(report.duplicate_reference_units, 0);
    assert!(report.render().contains("reference_population_completeness=NOT_ESTABLISHED"));
    assert!(report.render().contains("population_coverage_estimate=NOT_COMPUTED"));
}

#[test]
fn missing_reference_match_is_an_omission() {
    let input = POSITIVE
        .replace("OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100\\n", "")
        .replace(
            "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100\\n",
            "",
        );
    let report = audit(&input).expect("valid but incomplete fixture");
    assert_eq!(report.omitted_reference_units, 1);
    assert_eq!(report.disposition, "INCOMPLETE_OR_CONFLICTED");
}
#[test]
fn many_observed_records_matching_one_reference_is_duplicate_coverage() {
    let input = POSITIVE
        .replace(
            "OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100\n",
            "OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100\nOBSERVED|obs-d|IN_SCOPE|-|-|-|-|0|100\n",
        )
        .replace(
            "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100\n",
            "LINK|obs-c|IN_SCOPE|ref-b|MATCH|review-team|pair-c|0|100\nLINK|obs-d|IN_SCOPE|ref-c|MATCH|review-team|pair-d|0|100\n",
        );
    let report = audit(&input).expect("valid duplicate fixture");
    assert_eq!(report.duplicate_reference_units, 1);
    assert_eq!(report.disposition, "INCOMPLETE_OR_CONFLICTED");
}

#[test]
fn unresolved_candidate_blocks_reconciliation() {
    let input = POSITIVE.replace(
        "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100",
        "LINK|obs-c|IN_SCOPE|ref-c|UNRESOLVED|review-team|pair-c|0|100",
    );
    let report = audit(&input).expect("valid unresolved fixture");
    assert_eq!(report.unresolved_reference_units, 1);
    assert_eq!(report.unresolved_observed, 1);
    assert_eq!(report.disposition, "INCOMPLETE_OR_CONFLICTED");
}

#[test]
fn unmatched_in_scope_observed_record_is_not_silently_discarded() {
    let input = POSITIVE
        .replace(
            "OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100\n",
            "OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100\nOBSERVED|obs-d|IN_SCOPE|-|-|-|-|0|100\n",
        )
        .replace(
            "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100",
            "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100\nLINK|obs-d|IN_SCOPE|-|NO_MATCH_IN_SCOPE|review-team|pair-d|0|100",
        );
    let report = audit(&input).expect("valid extra-record fixture");
    assert_eq!(report.unrepresented_in_scope_observed, 1);
    assert_eq!(report.disposition, "INCOMPLETE_OR_CONFLICTED");
}

#[test]
fn explicit_out_of_scope_record_is_reported_but_not_counted_as_missing() {
    let input = POSITIVE
        .replace(
            "OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100\n",
            "OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100\nOBSERVED|obs-d|OUT_OF_SCOPE|-|-|-|-|0|100\n",
        )
        .replace(
            "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100",
            "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100\nLINK|obs-d|OUT_OF_SCOPE|-|OUT_OF_SCOPE|review-team|pair-d|0|100",
        );
    let report = audit(&input).expect("valid out-of-scope fixture");
    assert_eq!(report.out_of_scope_observed, 1);
    assert_eq!(report.disposition, "RECONCILED_TO_DECLARED_REFERENCE_FRAME");
}

#[test]
fn unknown_scope_is_not_assumed_out_of_scope() {
    let input = POSITIVE
        .replace("OBSERVED|obs-c|IN_SCOPE|-|-|-|-|0|100", "OBSERVED|obs-c|UNKNOWN|-|-|-|-|0|100")
        .replace(
            "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100",
            "LINK|obs-c|UNKNOWN|-|UNRESOLVED|review-team|pair-c|0|100",
        );
    let report = audit(&input).expect("valid unknown-scope fixture");
    assert_eq!(report.unknown_scope_observed, 1);
    assert_eq!(report.disposition, "INCOMPLETE_OR_CONFLICTED");
}

#[test]
fn sample_reference_frame_does_not_produce_a_census_style_claim() {
    let input = POSITIVE.replace(
        "reference_coverage_basis=DECLARED_ENUMERATED_REGISTER",
        "reference_coverage_basis=SAMPLE",
    );
    let report = audit(&input).expect("valid sample fixture");
    assert_eq!(report.disposition, "NOT_ESTIMABLE_REFERENCE_BASIS_UNSUPPORTED");
}

#[test]
fn expired_crosswalk_evidence_is_rejected() {
    let input = POSITIVE.replace(
        "LINK|obs-a|IN_SCOPE|ref-a|MATCH|review-team|pair-a|0|100",
        "LINK|obs-a|IN_SCOPE|ref-a|MATCH|review-team|pair-a|0|99",
    );
    assert!(audit(&input).is_err());
}

#[test]
fn every_observed_record_requires_a_disposition() {
    let input = POSITIVE.replace(
        "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100\n",
        "",
    );
    assert!(audit(&input).is_err());
}

#[test]
fn unknown_metadata_is_rejected() {
    let input = POSITIVE.replace("#audit_id=", "#unknown_key=x\n#audit_id=");
    assert!(audit(&input).is_err());
}

#[test]
fn duplicate_reference_ids_are_rejected() {
    let input = POSITIVE.replace(
        "REFERENCE|ref-c|IN_SCOPE|-|-|-|-|0|100",
        "REFERENCE|ref-b|IN_SCOPE|-|-|-|-|0|100",
    );
    assert!(audit(&input).is_err());
}

#[test]
fn link_to_unknown_reference_is_rejected() {
    let input = POSITIVE.replace(
        "LINK|obs-c|IN_SCOPE|ref-c|MATCH|review-team|pair-c|0|100",
        "LINK|obs-c|IN_SCOPE|missing-ref|MATCH|review-team|pair-c|0|100",
    );
    assert!(audit(&input).is_err());
}

#[test]
fn source_frame_validity_must_cover_the_entire_study_horizon() {
    let input = POSITIVE.replace(
        "reference_valid_until_seconds=100",
        "reference_valid_until_seconds=99",
    );
    assert!(audit(&input).is_err());
}

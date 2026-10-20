// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use praxis_core::decision_submission::{
    AbstentionReason, DECISION_SUBMISSION_BENCHMARK_ID, DECISION_SUBMISSION_SCHEMA_VERSION,
    DecisionContext, DecisionOutcome, DecisionRationale, DecisionSubmission, EvidenceRequestReason,
    MissingEvidenceKind, validate_decision_submission,
};
use serde_json::Value;

const DECISION_SUBMISSION_SCHEMA: &str =
    include_str!("../schemas/decision-submission-v1.schema.json");

fn fixture_context() -> (String, Vec<String>, Vec<String>, Vec<String>) {
    (
        "case-1".into(),
        vec!["h1".into(), "h2".into()],
        vec!["e1".into(), "e2".into(), "e3".into()],
        vec!["req-1".into(), "req-2".into()],
    )
}

fn submission(outcome: DecisionOutcome) -> DecisionSubmission {
    DecisionSubmission {
        schema_version: DECISION_SUBMISSION_SCHEMA_VERSION,
        benchmark_id: DECISION_SUBMISSION_BENCHMARK_ID.into(),
        scenario_id: "case-1".into(),
        outcome,
    }
}

fn claim() -> DecisionOutcome {
    DecisionOutcome::Claim {
        hypothesis_id: "h1".into(),
        cited_evidence_ids: vec!["e1".into()],
        confidence: Some(0.75),
        rationale: DecisionRationale {
            supporting_evidence_ids: vec!["e1".into()],
            contradictory_evidence_ids: vec![],
            missing_evidence: vec![],
        },
    }
}

fn validate(
    value: &DecisionSubmission,
) -> Result<(), Vec<praxis_core::decision_submission::DecisionValidationIssue>> {
    validate_with_confidence_permission(value, true)
}

fn validate_with_confidence_permission(
    value: &DecisionSubmission,
    confidence_scoring_preregistered: bool,
) -> Result<(), Vec<praxis_core::decision_submission::DecisionValidationIssue>> {
    let (scenario_id, hypothesis_ids, available_evidence_ids, authorized_request_ids) =
        fixture_context();
    let context = DecisionContext {
        scenario_id: &scenario_id,
        hypothesis_ids: &hypothesis_ids,
        available_evidence_ids: &available_evidence_ids,
        authorized_request_ids: &authorized_request_ids,
        confidence_scoring_preregistered,
    };
    validate_decision_submission(value, &context)
}

fn issue_codes(
    result: Result<(), Vec<praxis_core::decision_submission::DecisionValidationIssue>>,
) -> Vec<String> {
    result
        .expect_err("submission must fail validation")
        .into_iter()
        .map(|issue| issue.code)
        .collect()
}

#[test]
fn supported_claim_with_known_hypothesis_and_observed_citation_is_valid() {
    validate(&submission(claim())).expect("well-formed claim is valid");
}

#[test]
fn claim_requires_support_and_cannot_substitute_a_request_id_for_evidence() {
    let no_support = DecisionOutcome::Claim {
        hypothesis_id: "h1".into(),
        cited_evidence_ids: vec!["e1".into()],
        confidence: None,
        rationale: DecisionRationale::default(),
    };
    assert!(
        issue_codes(validate(&submission(no_support)))
            .contains(&"missing_supporting_evidence".into())
    );

    let request_as_evidence = DecisionOutcome::Claim {
        hypothesis_id: "h1".into(),
        cited_evidence_ids: vec!["req-1".into()],
        confidence: None,
        rationale: DecisionRationale {
            supporting_evidence_ids: vec!["req-1".into()],
            contradictory_evidence_ids: vec![],
            missing_evidence: vec![],
        },
    };
    assert!(
        issue_codes(validate(&submission(request_as_evidence)))
            .contains(&"unknown_evidence_id".into())
    );
}

#[test]
fn claims_reject_unknown_hypotheses_and_unavailable_evidence_ids() {
    let mut value = submission(claim());
    if let DecisionOutcome::Claim { hypothesis_id, .. } = &mut value.outcome {
        *hypothesis_id = "evaluator-only-answer".into();
    }
    assert!(issue_codes(validate(&value)).contains(&"unknown_hypothesis_id".into()));

    let unsupported_reference = DecisionOutcome::Claim {
        hypothesis_id: "h1".into(),
        cited_evidence_ids: vec!["not-observed".into()],
        confidence: None,
        rationale: DecisionRationale {
            supporting_evidence_ids: vec!["not-observed".into()],
            contradictory_evidence_ids: vec![],
            missing_evidence: vec![],
        },
    };
    assert!(
        issue_codes(validate(&submission(unsupported_reference)))
            .contains(&"unknown_evidence_id".into())
    );
}

#[test]
fn claim_rejects_duplicate_citations_and_uncited_contradiction() {
    let duplicate = DecisionOutcome::Claim {
        hypothesis_id: "h1".into(),
        cited_evidence_ids: vec!["e1".into(), "e1".into()],
        confidence: None,
        rationale: DecisionRationale {
            supporting_evidence_ids: vec!["e1".into()],
            contradictory_evidence_ids: vec![],
            missing_evidence: vec![],
        },
    };
    assert!(
        issue_codes(validate(&submission(duplicate))).contains(&"duplicate_evidence_id".into())
    );

    let uncited_contradiction = DecisionOutcome::Claim {
        hypothesis_id: "h1".into(),
        cited_evidence_ids: vec!["e1".into()],
        confidence: None,
        rationale: DecisionRationale {
            supporting_evidence_ids: vec!["e1".into()],
            contradictory_evidence_ids: vec!["e2".into()],
            missing_evidence: vec![],
        },
    };
    assert!(
        issue_codes(validate(&submission(uncited_contradiction)))
            .contains(&"contradictory_evidence_not_cited".into())
    );
}

#[test]
fn confidence_must_be_finite_and_within_zero_to_one() {
    for confidence in [f64::NAN, -0.01, 1.01] {
        let value = DecisionOutcome::Claim {
            hypothesis_id: "h1".into(),
            cited_evidence_ids: vec!["e1".into()],
            confidence: Some(confidence),
            rationale: DecisionRationale {
                supporting_evidence_ids: vec!["e1".into()],
                contradictory_evidence_ids: vec![],
                missing_evidence: vec![],
            },
        };
        assert!(
            issue_codes(validate(&submission(value))).contains(&"confidence_out_of_range".into())
        );
    }
}

#[test]
fn confidence_is_rejected_without_explicit_preregistered_permission() {
    let value = submission(claim());
    assert!(
        issue_codes(validate_with_confidence_permission(&value, false))
            .contains(&"confidence_not_preregistered".into())
    );
}

#[test]
fn structured_abstention_is_valid_but_unjustified_blanket_abstention_is_not() {
    let justified = DecisionOutcome::Abstain {
        reason: AbstentionReason::ConflictingEvidence,
        cited_evidence_ids: vec!["e2".into()],
        missing_evidence: vec![MissingEvidenceKind::ConflictResolution],
    };
    validate(&submission(justified)).expect("structured abstention should be valid");

    let blanket = DecisionOutcome::Abstain {
        reason: AbstentionReason::InsufficientEvidence,
        cited_evidence_ids: vec![],
        missing_evidence: vec![],
    };
    assert!(
        issue_codes(validate(&submission(blanket))).contains(&"missing_decision_rationale".into())
    );
}

#[test]
fn abstention_cannot_cite_unavailable_or_duplicate_evidence() {
    let value = DecisionOutcome::Abstain {
        reason: AbstentionReason::StaleEvidence,
        cited_evidence_ids: vec!["e3".into(), "e3".into(), "phantom".into()],
        missing_evidence: vec![MissingEvidenceKind::FreshnessVerification],
    };
    let codes = issue_codes(validate(&submission(value)));
    assert!(codes.contains(&"duplicate_evidence_id".into()));
    assert!(codes.contains(&"unknown_evidence_id".into()));
}

#[test]
fn request_more_evidence_accepts_only_authority_and_budget_filtered_ids() {
    let valid = DecisionOutcome::RequestMoreEvidence {
        request_ids: vec!["req-1".into()],
        reason: EvidenceRequestReason::ResolveAmbiguity,
    };
    validate(&submission(valid)).expect("authorized catalog request should be valid");

    let unauthorized = DecisionOutcome::RequestMoreEvidence {
        request_ids: vec!["not-authorized".into()],
        reason: EvidenceRequestReason::ConfirmEffectReceipt,
    };
    assert!(
        issue_codes(validate(&submission(unauthorized)))
            .contains(&"unknown_or_unauthorized_request_id".into())
    );
}

#[test]
fn request_more_evidence_rejects_empty_and_duplicate_request_lists() {
    let empty = DecisionOutcome::RequestMoreEvidence {
        request_ids: vec![],
        reason: EvidenceRequestReason::FillCoverageGap,
    };
    assert!(issue_codes(validate(&submission(empty))).contains(&"missing_request_id".into()));

    let duplicate = DecisionOutcome::RequestMoreEvidence {
        request_ids: vec!["req-1".into(), "req-1".into()],
        reason: EvidenceRequestReason::VerifyFreshness,
    };
    assert!(issue_codes(validate(&submission(duplicate))).contains(&"duplicate_request_id".into()));
}

#[test]
fn submission_must_match_schema_benchmark_and_scenario_context() {
    let mut value = submission(claim());
    value.schema_version += 1;
    value.benchmark_id = "wrong-benchmark".into();
    value.scenario_id = "case-2".into();
    let codes = issue_codes(validate(&value));
    assert!(codes.contains(&"unsupported_schema_version".into()));
    assert!(codes.contains(&"unexpected_benchmark_id".into()));
    assert!(codes.contains(&"scenario_mismatch".into()));
}

#[test]
fn decision_submission_json_schema_accepts_all_three_typed_outcomes() {
    let schema: Value =
        serde_json::from_str(DECISION_SUBMISSION_SCHEMA).expect("decision schema parses");
    assert!(
        jsonschema::draft202012::meta::is_valid(&schema),
        "decision-submission schema must conform to Draft 2020-12"
    );
    let validator =
        jsonschema::draft202012::new(&schema).expect("decision-submission schema compiles");

    let supported_claim = serde_json::to_value(submission(claim())).expect("claim serializes");
    assert!(validator.is_valid(&supported_claim));

    let abstention = submission(DecisionOutcome::Abstain {
        reason: AbstentionReason::ConflictingEvidence,
        cited_evidence_ids: vec!["e2".into()],
        missing_evidence: vec![MissingEvidenceKind::ConflictResolution],
    });
    assert!(validator.is_valid(&serde_json::to_value(abstention).expect("abstention serializes")));

    let follow_up = submission(DecisionOutcome::RequestMoreEvidence {
        request_ids: vec!["req-1".into()],
        reason: EvidenceRequestReason::ResolveAmbiguity,
    });
    assert!(validator.is_valid(&serde_json::to_value(follow_up).expect("request serializes")));
}

#[test]
fn decision_submission_json_schema_rejects_leaks_and_unjustified_outputs() {
    let schema: Value =
        serde_json::from_str(DECISION_SUBMISSION_SCHEMA).expect("decision schema parses");
    let validator =
        jsonschema::draft202012::new(&schema).expect("decision-submission schema compiles");
    let valid = serde_json::to_value(submission(claim())).expect("claim serializes");

    let mut unknown_field = valid.clone();
    unknown_field["outcome"]["evaluator_answer"] = Value::String("hidden".into());
    assert!(!validator.is_valid(&unknown_field));

    let mut unsupported_confidence = valid.clone();
    unsupported_confidence["outcome"]["confidence"] = Value::from(1.01);
    assert!(!validator.is_valid(&unsupported_confidence));

    let mut empty_support = valid.clone();
    empty_support["outcome"]["rationale"]["supporting_evidence_ids"] = Value::Array(Vec::new());
    assert!(!validator.is_valid(&empty_support));

    let mut invalid_kind = valid;
    invalid_kind["outcome"]["kind"] = Value::String("guaranteed_success".into());
    assert!(!validator.is_valid(&invalid_kind));

    let blanket_abstention = submission(DecisionOutcome::Abstain {
        reason: AbstentionReason::InsufficientEvidence,
        cited_evidence_ids: Vec::new(),
        missing_evidence: Vec::new(),
    });
    assert!(!validator.is_valid(
        &serde_json::to_value(blanket_abstention).expect("blanket abstention serializes")
    ));
}

#[test]
fn serde_round_trip_uses_explicit_tag_and_rejects_unknown_evaluator_fields() {
    let value = submission(claim());
    let encoded = serde_json::to_value(&value).expect("submission serializes");
    assert_eq!(encoded["outcome"]["kind"], "claim");
    let decoded: DecisionSubmission =
        serde_json::from_value(encoded.clone()).expect("submission deserializes");
    assert_eq!(decoded, value);

    let mut leaked = encoded;
    leaked["outcome"]["evaluator_answer"] = serde_json::Value::String("secret".into());
    assert!(
        serde_json::from_value::<DecisionSubmission>(leaked).is_err(),
        "unknown fields inside the tagged outcome must fail closed"
    );
}

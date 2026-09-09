// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Green regression successor to the expected-red #361 witness.
//!
//! The old public `CurrentAppealStateV1` path no longer exists. Positive remedy
//! qualification requires a sealed `QualifiedJusticeFinalityV1`, and a token
//! for one exact Decision/time cut must not be replayable into another cut.

use justice_finality_qualification::{
    COMPLETE_APPEAL_COVERAGE_PROFILE, COMPLETE_APPEAL_COVERAGE_VERSION,
    CompleteAppealCoverageEvidenceV1, JusticeFinalityQualificationBasisV1,
    QualifiedJusticeFinalityV1, qualify_justice_finality_v1,
};
use justice_resolution_verifier::{
    ArbitrationSnapshotV1, DecisionSnapshotV1, DecisionVoteChoiceV1, DecisionVoteSnapshotV1,
    FullAwardPolicyV1, FullDecisionOutcomeV1, JusticeVerificationError,
    MonetaryRemedyQualificationBasisV1, MonetaryRemedySnapshotV1, PanelMemberSnapshotV1,
    RuntimeMonetaryRemedyKindV1, TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE,
    TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION, TwoPartyCaseSnapshotV1,
    qualify_monetary_remedy_v1,
};

fn sealed_finality() -> QualifiedJusticeFinalityV1 {
    qualify_justice_finality_v1(JusticeFinalityQualificationBasisV1::NoAppealCoverage {
        decision_ref: "decision:1".into(),
        decision_rendered_at_unix_ms: 100,
        appeal_deadline_unix_ms: 200,
        qualification_time_unix_ms: 201,
        coverage: CompleteAppealCoverageEvidenceV1 {
            coverage_ref: "appeal-coverage:decision-1:through-201".into(),
            decision_ref: "decision:1".into(),
            authority_evidence_ref: "justice-finality-authority:test".into(),
            semantic_profile: COMPLETE_APPEAL_COVERAGE_PROFILE.into(),
            semantic_version: COMPLETE_APPEAL_COVERAGE_VERSION,
            covered_from_unix_ms: 100,
            covered_through_unix_ms: 201,
            observed_appeal_refs: vec![],
        },
    })
    .unwrap()
}

fn valid_basis() -> MonetaryRemedyQualificationBasisV1 {
    MonetaryRemedyQualificationBasisV1 {
        case: TwoPartyCaseSnapshotV1 {
            case_ref: "case:1".into(),
            complainant_ref: "party:customer".into(),
            respondent_ref: "party:merchant".into(),
            subject_ref: "order:1".into(),
        },
        arbitration: ArbitrationSnapshotV1 {
            arbitration_ref: "arbitration:1".into(),
            case_ref: "case:1".into(),
            panel: vec![PanelMemberSnapshotV1 {
                party_ref: "arb:1".into(),
                accepted: true,
                recused: false,
            }],
        },
        decision: DecisionSnapshotV1 {
            decision_ref: "decision:1".into(),
            case_ref: "case:1".into(),
            arbitration_ref: "arbitration:1".into(),
            outcome: FullDecisionOutcomeV1::ForComplainant,
            votes: vec![DecisionVoteSnapshotV1 {
                voter_ref: "arb:1".into(),
                choice: DecisionVoteChoiceV1::ForComplainant,
            }],
            rendered_at_unix_ms: 100,
            appeal_deadline_unix_ms: 200,
            declared_finalized: false,
        },
        remedy: MonetaryRemedySnapshotV1 {
            remedy_index: 0,
            kind: RuntimeMonetaryRemedyKindV1::Restitution,
            responsible_party_ref: "party:merchant".into(),
            amount: Some(5_000),
            unit: Some("USD-cent".into()),
        },
        qualified_finality: sealed_finality(),
        policy: FullAwardPolicyV1 {
            policy_ref: "justice-policy:full-award:v1".into(),
            semantic_profile: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE.into(),
            semantic_version: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION,
            minimum_quorum_votes: 1,
            minimum_support_votes: 1,
        },
        qualification_time_unix_ms: 201,
    }
}

#[test]
fn sealed_finality_cannot_be_replayed_into_another_decision() {
    let mut basis = valid_basis();
    basis.decision.decision_ref = "decision:other".into();
    assert_eq!(
        qualify_monetary_remedy_v1(basis),
        Err(JusticeVerificationError::FinalityDecisionMismatch)
    );
}

#[test]
fn sealed_finality_cannot_be_replayed_after_deadline_drift() {
    let mut basis = valid_basis();
    basis.decision.appeal_deadline_unix_ms = 250;
    assert_eq!(
        qualify_monetary_remedy_v1(basis),
        Err(JusticeVerificationError::FinalityAppealDeadlineMismatch)
    );
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Expected-red diagnostic for issue #358.
//!
//! The monetary-remedy verifier must eventually require a non-forgeable
//! `QualifiedJusticeFinalityV1`. Until then, a caller can construct
//! `CurrentAppealStateV1::None` with an arbitrary evidence string and satisfy
//! the verifier's structural finality checks without going through Justice's
//! positive finality qualifier.

use justice_resolution_verifier::{
    ArbitrationSnapshotV1, CurrentAppealStateV1, DecisionSnapshotV1, DecisionVoteChoiceV1,
    DecisionVoteSnapshotV1, FullAwardPolicyV1, FullDecisionOutcomeV1,
    MonetaryRemedyQualificationBasisV1, MonetaryRemedySnapshotV1, PanelMemberSnapshotV1,
    RuntimeMonetaryRemedyKindV1, TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE,
    TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION, TwoPartyCaseSnapshotV1,
    qualify_monetary_remedy_v1,
};

fn otherwise_valid_basis_with_fabricated_finality() -> MonetaryRemedyQualificationBasisV1 {
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
        appeal_state: CurrentAppealStateV1::None {
            no_live_appeal_evidence_ref: "caller-fabricated:not-authenticated".into(),
        },
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
fn caller_fabricated_no_appeal_state_must_not_mint_positive_finality() {
    let result = qualify_monetary_remedy_v1(otherwise_valid_basis_with_fabricated_finality());
    assert!(
        result.is_err(),
        "expected denial: caller-declared CurrentAppealStateV1 must not bypass sealed Justice finality qualification"
    );
}

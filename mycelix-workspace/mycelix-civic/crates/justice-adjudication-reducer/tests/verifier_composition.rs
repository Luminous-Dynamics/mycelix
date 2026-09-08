// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use justice_adjudication_reducer::{
    AdjudicationReductionBasisV1, AuthenticatedParticipationDispositionV1,
    AuthenticatedParticipationEvidenceV1, AuthenticatedVoteEvidenceV1, SelectedPanelMemberV1,
    reduce_adjudication_evidence_v1,
};
use justice_resolution_verifier::{
    ArbitrationSnapshotV1, CurrentAppealStateV1, DecisionSnapshotV1, DecisionVoteChoiceV1,
    FullAwardPolicyV1, FullDecisionOutcomeV1, MonetaryRemedyQualificationBasisV1,
    MonetaryRemedySnapshotV1, RuntimeMonetaryRemedyKindV1,
    TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE, TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION,
    TwoPartyCaseSnapshotV1, qualify_monetary_remedy_v1,
};

fn participation(actor: &str, attestation: &str) -> AuthenticatedParticipationEvidenceV1 {
    AuthenticatedParticipationEvidenceV1 {
        attestation_ref: attestation.into(),
        arbitration_ref: "arb-action:1".into(),
        arbitrator_ref: actor.into(),
        disposition: AuthenticatedParticipationDispositionV1::Accepted,
    }
}

fn vote(actor: &str, attestation: &str, acceptance: &str) -> AuthenticatedVoteEvidenceV1 {
    AuthenticatedVoteEvidenceV1 {
        attestation_ref: attestation.into(),
        decision_ref: "decision-action:1".into(),
        arbitration_ref: "arb-action:1".into(),
        acceptance_attestation_ref: acceptance.into(),
        arbitrator_ref: actor.into(),
        choice: DecisionVoteChoiceV1::ForComplainant,
    }
}

#[test]
fn reduced_authenticated_evidence_flows_directly_into_pure_remedy_verifier() {
    let reduced = reduce_adjudication_evidence_v1(AdjudicationReductionBasisV1 {
        arbitration_ref: "arb-action:1".into(),
        decision_ref: "decision-action:1".into(),
        selected_panel: vec![
            SelectedPanelMemberV1 {
                party_ref: "did:arb:a".into(),
            },
            SelectedPanelMemberV1 {
                party_ref: "did:arb:b".into(),
            },
            SelectedPanelMemberV1 {
                party_ref: "did:arb:c".into(),
            },
        ],
        participation: vec![
            participation("did:arb:a", "participation:a"),
            participation("did:arb:b", "participation:b"),
            participation("did:arb:c", "participation:c"),
        ],
        votes: vec![
            vote("did:arb:a", "vote:a", "participation:a"),
            vote("did:arb:b", "vote:b", "participation:b"),
            AuthenticatedVoteEvidenceV1 {
                choice: DecisionVoteChoiceV1::Abstain,
                ..vote("did:arb:c", "vote:c", "participation:c")
            },
        ],
    })
    .unwrap();

    let verified = qualify_monetary_remedy_v1(MonetaryRemedyQualificationBasisV1 {
        case: TwoPartyCaseSnapshotV1 {
            case_ref: "case-action:1".into(),
            complainant_ref: "did:party:complainant".into(),
            respondent_ref: "did:party:respondent".into(),
            subject_ref: "order:1".into(),
        },
        arbitration: ArbitrationSnapshotV1 {
            arbitration_ref: "arb-action:1".into(),
            case_ref: "case-action:1".into(),
            panel: reduced.panel().to_vec(),
        },
        decision: DecisionSnapshotV1 {
            decision_ref: "decision-action:1".into(),
            case_ref: "case-action:1".into(),
            arbitration_ref: "arb-action:1".into(),
            outcome: FullDecisionOutcomeV1::ForComplainant,
            votes: reduced.votes().to_vec(),
            rendered_at_unix_ms: 100,
            appeal_deadline_unix_ms: 200,
            declared_finalized: false,
        },
        remedy: MonetaryRemedySnapshotV1 {
            remedy_index: 0,
            kind: RuntimeMonetaryRemedyKindV1::Restitution,
            responsible_party_ref: "did:party:respondent".into(),
            amount: Some(5_000),
            unit: Some("USD-cent".into()),
        },
        appeal_state: CurrentAppealStateV1::None {
            no_live_appeal_evidence_ref: "synthetic:test-only:no-live-appeal".into(),
        },
        policy: FullAwardPolicyV1 {
            policy_ref: "justice-policy:1".into(),
            semantic_profile: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE.into(),
            semantic_version: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION,
            minimum_quorum_votes: 3,
            minimum_support_votes: 2,
        },
        qualification_time_unix_ms: 300,
    })
    .unwrap();

    assert_eq!(verified.receipt().arbitration_ref(), "arb-action:1");
    assert_eq!(verified.receipt().decision_ref(), "decision-action:1");
    assert_eq!(verified.receipt().support_votes(), 2);
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Pure conflict-aware reduction of already-authenticated Justice adjudication evidence.
//!
//! This crate does not fetch or authenticate Holochain records. The owning Justice
//! runtime must first establish the provenance represented by the input evidence.
//! The reducer's job is narrower: preserve exact arbitration/decision identity,
//! fail closed on duplicate or conflicting actor evidence, require an exact
//! Accepted participation basis for every vote, and emit only the panel/vote
//! snapshots consumed by `justice-resolution-verifier`.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use justice_resolution_verifier::{
    DecisionVoteChoiceV1, DecisionVoteSnapshotV1, PanelMemberSnapshotV1,
};

pub const AUTHENTICATED_ADJUDICATION_REDUCTION_PROFILE: &str =
    "justice.authenticated-adjudication-reduction";
pub const AUTHENTICATED_ADJUDICATION_REDUCTION_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SelectedPanelMemberV1 {
    pub party_ref: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuthenticatedParticipationDispositionV1 {
    Accepted,
    Recused,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedParticipationEvidenceV1 {
    pub attestation_ref: String,
    pub arbitration_ref: String,
    pub arbitrator_ref: String,
    pub disposition: AuthenticatedParticipationDispositionV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedVoteEvidenceV1 {
    pub attestation_ref: String,
    pub decision_ref: String,
    pub arbitration_ref: String,
    pub acceptance_attestation_ref: String,
    pub arbitrator_ref: String,
    pub choice: DecisionVoteChoiceV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdjudicationReductionBasisV1 {
    pub arbitration_ref: String,
    pub decision_ref: String,
    pub selected_panel: Vec<SelectedPanelMemberV1>,
    pub participation: Vec<AuthenticatedParticipationEvidenceV1>,
    pub votes: Vec<AuthenticatedVoteEvidenceV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReducedParticipationBindingV1 {
    attestation_ref: String,
    disposition: AuthenticatedParticipationDispositionV1,
}

impl ReducedParticipationBindingV1 {
    #[must_use]
    pub fn attestation_ref(&self) -> &str {
        &self.attestation_ref
    }

    #[must_use]
    pub const fn disposition(&self) -> AuthenticatedParticipationDispositionV1 {
        self.disposition
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReducedVoteBindingV1 {
    attestation_ref: String,
    acceptance_attestation_ref: String,
    choice: DecisionVoteChoiceV1,
}

impl ReducedVoteBindingV1 {
    #[must_use]
    pub fn attestation_ref(&self) -> &str {
        &self.attestation_ref
    }

    #[must_use]
    pub fn acceptance_attestation_ref(&self) -> &str {
        &self.acceptance_attestation_ref
    }

    #[must_use]
    pub const fn choice(&self) -> DecisionVoteChoiceV1 {
        self.choice
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdjudicationReductionReceiptV1 {
    arbitration_ref: String,
    decision_ref: String,
    semantic_profile: &'static str,
    semantic_version: u32,
    selected_panel: BTreeSet<String>,
    participation: BTreeMap<String, ReducedParticipationBindingV1>,
    votes: BTreeMap<String, ReducedVoteBindingV1>,
}

impl AdjudicationReductionReceiptV1 {
    #[must_use]
    pub fn arbitration_ref(&self) -> &str {
        &self.arbitration_ref
    }

    #[must_use]
    pub fn decision_ref(&self) -> &str {
        &self.decision_ref
    }

    #[must_use]
    pub const fn semantic_profile(&self) -> &'static str {
        self.semantic_profile
    }

    #[must_use]
    pub const fn semantic_version(&self) -> u32 {
        self.semantic_version
    }

    #[must_use]
    pub const fn selected_panel(&self) -> &BTreeSet<String> {
        &self.selected_panel
    }

    #[must_use]
    pub const fn participation(&self) -> &BTreeMap<String, ReducedParticipationBindingV1> {
        &self.participation
    }

    #[must_use]
    pub const fn votes(&self) -> &BTreeMap<String, ReducedVoteBindingV1> {
        &self.votes
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReducedAdjudicationEvidenceV1 {
    panel: Vec<PanelMemberSnapshotV1>,
    votes: Vec<DecisionVoteSnapshotV1>,
    receipt: AdjudicationReductionReceiptV1,
}

impl ReducedAdjudicationEvidenceV1 {
    #[must_use]
    pub fn panel(&self) -> &[PanelMemberSnapshotV1] {
        &self.panel
    }

    #[must_use]
    pub fn votes(&self) -> &[DecisionVoteSnapshotV1] {
        &self.votes
    }

    #[must_use]
    pub const fn receipt(&self) -> &AdjudicationReductionReceiptV1 {
        &self.receipt
    }

    #[must_use]
    pub fn into_parts(
        self,
    ) -> (
        Vec<PanelMemberSnapshotV1>,
        Vec<DecisionVoteSnapshotV1>,
        AdjudicationReductionReceiptV1,
    ) {
        (self.panel, self.votes, self.receipt)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdjudicationReductionError {
    EmptyArbitrationRef,
    EmptyDecisionRef,
    EmptySelectedPanel,
    EmptySelectedPanelMember,
    DuplicateSelectedPanelMember,
    EmptyParticipationRef,
    EmptyArbitratorRef,
    ParticipationArbitrationMismatch,
    ParticipationFromUnselectedActor,
    DuplicateParticipationEvidenceRef,
    MultipleParticipationEvidence,
    EmptyVoteRef,
    EmptyAcceptanceRef,
    VoteDecisionMismatch,
    VoteArbitrationMismatch,
    VoteFromUnselectedActor,
    DuplicateVoteEvidenceRef,
    MultipleVoteEvidence,
    MissingAcceptedParticipation,
    AcceptanceReferenceMismatch,
}

impl fmt::Display for AdjudicationReductionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "justice adjudication reduction denied: {self:?}")
    }
}

impl std::error::Error for AdjudicationReductionError {}

pub fn reduce_adjudication_evidence_v1(
    basis: AdjudicationReductionBasisV1,
) -> Result<ReducedAdjudicationEvidenceV1, AdjudicationReductionError> {
    if basis.arbitration_ref.trim().is_empty() {
        return Err(AdjudicationReductionError::EmptyArbitrationRef);
    }
    if basis.decision_ref.trim().is_empty() {
        return Err(AdjudicationReductionError::EmptyDecisionRef);
    }
    if basis.selected_panel.is_empty() {
        return Err(AdjudicationReductionError::EmptySelectedPanel);
    }

    let mut selected_panel = BTreeSet::new();
    for member in &basis.selected_panel {
        if member.party_ref.trim().is_empty() {
            return Err(AdjudicationReductionError::EmptySelectedPanelMember);
        }
        if !selected_panel.insert(member.party_ref.clone()) {
            return Err(AdjudicationReductionError::DuplicateSelectedPanelMember);
        }
    }

    let mut participation_refs = BTreeSet::new();
    let mut participation_by_actor: BTreeMap<
        String,
        &AuthenticatedParticipationEvidenceV1,
    > = BTreeMap::new();
    for evidence in &basis.participation {
        if evidence.attestation_ref.trim().is_empty() {
            return Err(AdjudicationReductionError::EmptyParticipationRef);
        }
        if evidence.arbitrator_ref.trim().is_empty() {
            return Err(AdjudicationReductionError::EmptyArbitratorRef);
        }
        if evidence.arbitration_ref != basis.arbitration_ref {
            return Err(AdjudicationReductionError::ParticipationArbitrationMismatch);
        }
        if !selected_panel.contains(&evidence.arbitrator_ref) {
            return Err(AdjudicationReductionError::ParticipationFromUnselectedActor);
        }
        if !participation_refs.insert(evidence.attestation_ref.clone()) {
            return Err(AdjudicationReductionError::DuplicateParticipationEvidenceRef);
        }
        if participation_by_actor
            .insert(evidence.arbitrator_ref.clone(), evidence)
            .is_some()
        {
            return Err(AdjudicationReductionError::MultipleParticipationEvidence);
        }
    }

    let panel = selected_panel
        .iter()
        .map(|party_ref| match participation_by_actor.get(party_ref) {
            Some(evidence) => match evidence.disposition {
                AuthenticatedParticipationDispositionV1::Accepted => PanelMemberSnapshotV1 {
                    party_ref: party_ref.clone(),
                    accepted: true,
                    recused: false,
                },
                AuthenticatedParticipationDispositionV1::Recused => PanelMemberSnapshotV1 {
                    party_ref: party_ref.clone(),
                    accepted: false,
                    recused: true,
                },
            },
            None => PanelMemberSnapshotV1 {
                party_ref: party_ref.clone(),
                accepted: false,
                recused: false,
            },
        })
        .collect::<Vec<_>>();

    let mut vote_refs = BTreeSet::new();
    let mut votes_by_actor: BTreeMap<String, &AuthenticatedVoteEvidenceV1> = BTreeMap::new();
    for evidence in &basis.votes {
        if evidence.attestation_ref.trim().is_empty() {
            return Err(AdjudicationReductionError::EmptyVoteRef);
        }
        if evidence.acceptance_attestation_ref.trim().is_empty() {
            return Err(AdjudicationReductionError::EmptyAcceptanceRef);
        }
        if evidence.arbitrator_ref.trim().is_empty() {
            return Err(AdjudicationReductionError::EmptyArbitratorRef);
        }
        if evidence.decision_ref != basis.decision_ref {
            return Err(AdjudicationReductionError::VoteDecisionMismatch);
        }
        if evidence.arbitration_ref != basis.arbitration_ref {
            return Err(AdjudicationReductionError::VoteArbitrationMismatch);
        }
        if !selected_panel.contains(&evidence.arbitrator_ref) {
            return Err(AdjudicationReductionError::VoteFromUnselectedActor);
        }
        if !vote_refs.insert(evidence.attestation_ref.clone()) {
            return Err(AdjudicationReductionError::DuplicateVoteEvidenceRef);
        }
        if votes_by_actor
            .insert(evidence.arbitrator_ref.clone(), evidence)
            .is_some()
        {
            return Err(AdjudicationReductionError::MultipleVoteEvidence);
        }

        let participation = participation_by_actor
            .get(&evidence.arbitrator_ref)
            .ok_or(AdjudicationReductionError::MissingAcceptedParticipation)?;
        if participation.disposition != AuthenticatedParticipationDispositionV1::Accepted {
            return Err(AdjudicationReductionError::MissingAcceptedParticipation);
        }
        if evidence.acceptance_attestation_ref != participation.attestation_ref {
            return Err(AdjudicationReductionError::AcceptanceReferenceMismatch);
        }
    }

    let votes = votes_by_actor
        .iter()
        .map(|(voter_ref, evidence)| DecisionVoteSnapshotV1 {
            voter_ref: voter_ref.clone(),
            choice: evidence.choice,
        })
        .collect::<Vec<_>>();

    let reduced_participation = participation_by_actor
        .into_iter()
        .map(|(actor, evidence)| {
            (
                actor,
                ReducedParticipationBindingV1 {
                    attestation_ref: evidence.attestation_ref.clone(),
                    disposition: evidence.disposition,
                },
            )
        })
        .collect();
    let reduced_votes = votes_by_actor
        .into_iter()
        .map(|(actor, evidence)| {
            (
                actor,
                ReducedVoteBindingV1 {
                    attestation_ref: evidence.attestation_ref.clone(),
                    acceptance_attestation_ref: evidence.acceptance_attestation_ref.clone(),
                    choice: evidence.choice,
                },
            )
        })
        .collect();

    let receipt = AdjudicationReductionReceiptV1 {
        arbitration_ref: basis.arbitration_ref,
        decision_ref: basis.decision_ref,
        semantic_profile: AUTHENTICATED_ADJUDICATION_REDUCTION_PROFILE,
        semantic_version: AUTHENTICATED_ADJUDICATION_REDUCTION_VERSION,
        selected_panel,
        participation: reduced_participation,
        votes: reduced_votes,
    };

    Ok(ReducedAdjudicationEvidenceV1 {
        panel,
        votes,
        receipt,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn panel_member(id: &str) -> SelectedPanelMemberV1 {
        SelectedPanelMemberV1 {
            party_ref: id.into(),
        }
    }

    fn participation(
        actor: &str,
        attestation: &str,
        disposition: AuthenticatedParticipationDispositionV1,
    ) -> AuthenticatedParticipationEvidenceV1 {
        AuthenticatedParticipationEvidenceV1 {
            attestation_ref: attestation.into(),
            arbitration_ref: "arb-action:1".into(),
            arbitrator_ref: actor.into(),
            disposition,
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

    fn happy_basis() -> AdjudicationReductionBasisV1 {
        AdjudicationReductionBasisV1 {
            arbitration_ref: "arb-action:1".into(),
            decision_ref: "decision-action:1".into(),
            selected_panel: vec![
                panel_member("did:a"),
                panel_member("did:b"),
                panel_member("did:c"),
            ],
            participation: vec![
                participation(
                    "did:a",
                    "participation:a",
                    AuthenticatedParticipationDispositionV1::Accepted,
                ),
                participation(
                    "did:b",
                    "participation:b",
                    AuthenticatedParticipationDispositionV1::Accepted,
                ),
                participation(
                    "did:c",
                    "participation:c",
                    AuthenticatedParticipationDispositionV1::Recused,
                ),
            ],
            votes: vec![
                vote("did:a", "vote:a", "participation:a"),
                vote("did:b", "vote:b", "participation:b"),
            ],
        }
    }

    #[test]
    fn happy_path_reduces_to_verifier_snapshots_and_self_explaining_receipt() {
        let reduced = reduce_adjudication_evidence_v1(happy_basis()).unwrap();
        assert_eq!(reduced.panel().len(), 3);
        assert_eq!(reduced.votes().len(), 2);
        assert!(reduced.panel()[0].accepted);
        assert!(!reduced.panel()[0].recused);
        assert!(!reduced.panel()[2].accepted);
        assert!(reduced.panel()[2].recused);
        assert_eq!(
            reduced.receipt().semantic_profile(),
            AUTHENTICATED_ADJUDICATION_REDUCTION_PROFILE
        );
        assert_eq!(
            reduced.receipt().participation()["did:a"].disposition(),
            AuthenticatedParticipationDispositionV1::Accepted
        );
        assert_eq!(
            reduced.receipt().votes()["did:a"].acceptance_attestation_ref(),
            "participation:a"
        );
    }

    #[test]
    fn input_order_does_not_change_output() {
        let a = reduce_adjudication_evidence_v1(happy_basis()).unwrap();
        let mut reordered = happy_basis();
        reordered.selected_panel.reverse();
        reordered.participation.reverse();
        reordered.votes.reverse();
        let b = reduce_adjudication_evidence_v1(reordered).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn empty_selected_panel_is_denied() {
        let mut basis = happy_basis();
        basis.selected_panel.clear();
        basis.participation.clear();
        basis.votes.clear();
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::EmptySelectedPanel)
        );
    }

    #[test]
    fn selected_actor_without_participation_or_vote_remains_inactive() {
        let basis = AdjudicationReductionBasisV1 {
            arbitration_ref: "arb-action:1".into(),
            decision_ref: "decision-action:1".into(),
            selected_panel: vec![panel_member("did:a")],
            participation: vec![],
            votes: vec![],
        };
        let reduced = reduce_adjudication_evidence_v1(basis).unwrap();
        assert!(!reduced.panel()[0].accepted);
        assert!(!reduced.panel()[0].recused);
        assert!(reduced.votes().is_empty());
    }

    #[test]
    fn duplicate_selected_panel_member_is_denied() {
        let mut basis = happy_basis();
        basis.selected_panel.push(panel_member("did:a"));
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::DuplicateSelectedPanelMember)
        );
    }

    #[test]
    fn duplicate_participation_even_same_disposition_is_denied() {
        let mut basis = happy_basis();
        basis.participation.push(participation(
            "did:a",
            "participation:a:2",
            AuthenticatedParticipationDispositionV1::Accepted,
        ));
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::MultipleParticipationEvidence)
        );
    }

    #[test]
    fn accepted_and_recused_participation_is_denied() {
        let mut basis = happy_basis();
        basis.participation.push(participation(
            "did:a",
            "participation:a:recused",
            AuthenticatedParticipationDispositionV1::Recused,
        ));
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::MultipleParticipationEvidence)
        );
    }

    #[test]
    fn participation_ref_cannot_be_reused_across_actors() {
        let mut basis = happy_basis();
        basis.participation[1].attestation_ref = "participation:a".into();
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::DuplicateParticipationEvidenceRef)
        );
    }

    #[test]
    fn duplicate_vote_for_same_actor_is_denied() {
        let mut basis = happy_basis();
        basis
            .votes
            .push(vote("did:a", "vote:a:2", "participation:a"));
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::MultipleVoteEvidence)
        );
    }

    #[test]
    fn vote_ref_cannot_be_reused_across_actors() {
        let mut basis = happy_basis();
        basis.votes[1].attestation_ref = "vote:a".into();
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::DuplicateVoteEvidenceRef)
        );
    }

    #[test]
    fn vote_from_unselected_actor_is_denied() {
        let mut basis = happy_basis();
        basis
            .votes
            .push(vote("did:z", "vote:z", "participation:z"));
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::VoteFromUnselectedActor)
        );
    }

    #[test]
    fn vote_without_participation_is_denied() {
        let mut basis = happy_basis();
        basis
            .participation
            .retain(|participation| participation.arbitrator_ref != "did:a");
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::MissingAcceptedParticipation)
        );
    }

    #[test]
    fn recused_actor_cannot_vote() {
        let mut basis = happy_basis();
        basis.participation[0].disposition = AuthenticatedParticipationDispositionV1::Recused;
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::MissingAcceptedParticipation)
        );
    }

    #[test]
    fn empty_acceptance_reference_is_denied() {
        let mut basis = happy_basis();
        basis.votes[0].acceptance_attestation_ref.clear();
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::EmptyAcceptanceRef)
        );
    }

    #[test]
    fn vote_must_reference_exact_accepted_participation() {
        let mut basis = happy_basis();
        basis.votes[0].acceptance_attestation_ref = "participation:other".into();
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::AcceptanceReferenceMismatch)
        );
    }

    #[test]
    fn participation_from_wrong_arbitration_is_denied() {
        let mut basis = happy_basis();
        basis.participation[0].arbitration_ref = "arb-action:other".into();
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::ParticipationArbitrationMismatch)
        );
    }

    #[test]
    fn vote_for_wrong_decision_is_denied() {
        let mut basis = happy_basis();
        basis.votes[0].decision_ref = "decision-action:other".into();
        assert_eq!(
            reduce_adjudication_evidence_v1(basis),
            Err(AdjudicationReductionError::VoteDecisionMismatch)
        );
    }
}

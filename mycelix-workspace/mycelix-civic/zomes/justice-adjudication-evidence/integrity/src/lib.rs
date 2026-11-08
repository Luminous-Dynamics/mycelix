// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Append-only, individually authored evidence for Justice adjudication.
//!
//! The legacy `Arbitration.accepted/recused` and `Decision.votes` aggregates are
//! useful UI/runtime state but are not strong enough for downstream authority.
//! This zome records the corresponding claims as independent DHT actions whose
//! author is the actor being represented.

use hdi::prelude::*;
use justice_arbitration_integrity::{Arbitration, Decision};

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum ParticipationDispositionV1 {
    Accepted,
    Recused { reason: Option<String> },
}

#[hdk_entry_helper]
#[derive(Clone)]
pub struct ArbitratorParticipationAttestationV1 {
    pub arbitration_action_hash: ActionHash,
    pub arbitration_id: String,
    pub arbitrator: String,
    pub disposition: ParticipationDispositionV1,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum AuthenticatedDecisionVoteChoiceV1 {
    ForComplainant,
    ForRespondent,
    Abstain,
}

#[hdk_entry_helper]
#[derive(Clone)]
pub struct DecisionVoteAttestationV1 {
    pub decision_action_hash: ActionHash,
    pub decision_id: String,
    pub arbitration_action_hash: ActionHash,
    pub arbitration_id: String,
    /// Exact individually authored `Accepted` participation attestation that
    /// establishes this actor's participation basis for the exact Arbitration.
    pub acceptance_attestation_action_hash: ActionHash,
    pub arbitrator: String,
    pub vote: AuthenticatedDecisionVoteChoiceV1,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    ArbitratorParticipationAttestationV1(ArbitratorParticipationAttestationV1),
    #[entry_type(visibility = "public")]
    DecisionVoteAttestationV1(DecisionVoteAttestationV1),
}

#[hdk_link_types]
pub enum LinkTypes {
    ArbitrationToParticipationAttestations,
    DecisionToVoteAttestations,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EvidenceValidationError {
    EmptyArbitrationId,
    EmptyDecisionId,
    EmptyArbitrator,
    ArbitratorNotDid,
    RecusalReasonTooLong,
    AuthorMismatch,
    ArbitrationIdMismatch,
    ArbitratorNotSelected,
    DecisionIdMismatch,
    DecisionArbitrationMismatch,
    DecisionCaseMismatch,
    AcceptanceArbitrationActionMismatch,
    AcceptanceArbitrationIdMismatch,
    AcceptanceArbitratorMismatch,
    AcceptanceNotAccepted,
}

impl core::fmt::Display for EvidenceValidationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "justice adjudication evidence invalid: {self:?}")
    }
}

fn expected_author_did(author: &AgentPubKey) -> String {
    format!("did:mycelix:{author}")
}

fn validate_participation_against(
    attestation: &ArbitratorParticipationAttestationV1,
    arbitration: &Arbitration,
    author_did: &str,
) -> Result<(), EvidenceValidationError> {
    if attestation.arbitration_id.trim().is_empty() {
        return Err(EvidenceValidationError::EmptyArbitrationId);
    }
    if attestation.arbitrator.trim().is_empty() {
        return Err(EvidenceValidationError::EmptyArbitrator);
    }
    if !attestation.arbitrator.starts_with("did:") {
        return Err(EvidenceValidationError::ArbitratorNotDid);
    }
    if let ParticipationDispositionV1::Recused { reason: Some(reason) } = &attestation.disposition {
        if reason.len() > 4096 {
            return Err(EvidenceValidationError::RecusalReasonTooLong);
        }
    }
    if attestation.arbitrator != author_did {
        return Err(EvidenceValidationError::AuthorMismatch);
    }
    if attestation.arbitration_id != arbitration.id {
        return Err(EvidenceValidationError::ArbitrationIdMismatch);
    }
    if !arbitration
        .arbitrators
        .iter()
        .any(|candidate| candidate.did == attestation.arbitrator)
    {
        return Err(EvidenceValidationError::ArbitratorNotSelected);
    }
    Ok(())
}

fn validate_vote_against(
    attestation: &DecisionVoteAttestationV1,
    arbitration: &Arbitration,
    decision: &Decision,
    acceptance: &ArbitratorParticipationAttestationV1,
    author_did: &str,
) -> Result<(), EvidenceValidationError> {
    if attestation.decision_id.trim().is_empty() {
        return Err(EvidenceValidationError::EmptyDecisionId);
    }
    if attestation.arbitration_id.trim().is_empty() {
        return Err(EvidenceValidationError::EmptyArbitrationId);
    }
    if attestation.arbitrator.trim().is_empty() {
        return Err(EvidenceValidationError::EmptyArbitrator);
    }
    if !attestation.arbitrator.starts_with("did:") {
        return Err(EvidenceValidationError::ArbitratorNotDid);
    }
    if attestation.arbitrator != author_did {
        return Err(EvidenceValidationError::AuthorMismatch);
    }
    if attestation.decision_id != decision.id {
        return Err(EvidenceValidationError::DecisionIdMismatch);
    }
    if attestation.arbitration_id != arbitration.id {
        return Err(EvidenceValidationError::ArbitrationIdMismatch);
    }
    if decision.arbitration_id != arbitration.id {
        return Err(EvidenceValidationError::DecisionArbitrationMismatch);
    }
    if decision.case_id != arbitration.case_id {
        return Err(EvidenceValidationError::DecisionCaseMismatch);
    }
    if !arbitration
        .arbitrators
        .iter()
        .any(|candidate| candidate.did == attestation.arbitrator)
    {
        return Err(EvidenceValidationError::ArbitratorNotSelected);
    }

    if acceptance.arbitration_action_hash != attestation.arbitration_action_hash {
        return Err(EvidenceValidationError::AcceptanceArbitrationActionMismatch);
    }
    if acceptance.arbitration_id != attestation.arbitration_id {
        return Err(EvidenceValidationError::AcceptanceArbitrationIdMismatch);
    }
    if acceptance.arbitrator != attestation.arbitrator {
        return Err(EvidenceValidationError::AcceptanceArbitratorMismatch);
    }
    if acceptance.disposition != ParticipationDispositionV1::Accepted {
        return Err(EvidenceValidationError::AcceptanceNotAccepted);
    }

    Ok(())
}

fn invalid(error: EvidenceValidationError) -> ValidateCallbackResult {
    ValidateCallbackResult::Invalid(error.to_string())
}

fn decode_arbitration(record: &Record) -> ExternResult<Option<Arbitration>> {
    record.entry().to_app_option::<Arbitration>().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "could not decode referenced Arbitration: {error:?}"
        )))
    })
}

fn decode_decision(record: &Record) -> ExternResult<Option<Decision>> {
    record.entry().to_app_option::<Decision>().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "could not decode referenced Decision: {error:?}"
        )))
    })
}

fn decode_participation_attestation(
    record: &Record,
) -> ExternResult<Option<ArbitratorParticipationAttestationV1>> {
    record
        .entry()
        .to_app_option::<ArbitratorParticipationAttestationV1>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "could not decode referenced participation attestation: {error:?}"
            )))
        })
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => match app_entry {
            EntryTypes::ArbitratorParticipationAttestationV1(attestation) => {
                let arbitration_record =
                    must_get_valid_record(attestation.arbitration_action_hash.clone())?;
                let Some(arbitration) = decode_arbitration(&arbitration_record)? else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "participation attestation must reference an Arbitration record".into(),
                    ));
                };
                match validate_participation_against(
                    &attestation,
                    &arbitration,
                    &expected_author_did(&action.author),
                ) {
                    Ok(()) => Ok(ValidateCallbackResult::Valid),
                    Err(error) => Ok(invalid(error)),
                }
            }
            EntryTypes::DecisionVoteAttestationV1(attestation) => {
                let arbitration_record =
                    must_get_valid_record(attestation.arbitration_action_hash.clone())?;
                let Some(arbitration) = decode_arbitration(&arbitration_record)? else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "vote attestation must reference an Arbitration record".into(),
                    ));
                };
                let decision_record =
                    must_get_valid_record(attestation.decision_action_hash.clone())?;
                let Some(decision) = decode_decision(&decision_record)? else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "vote attestation must reference a Decision record".into(),
                    ));
                };
                let acceptance_record = must_get_valid_record(
                    attestation.acceptance_attestation_action_hash.clone(),
                )?;
                let Some(acceptance) = decode_participation_attestation(&acceptance_record)? else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "vote attestation must reference an Accepted participation attestation"
                            .into(),
                    ));
                };
                match validate_vote_against(
                    &attestation,
                    &arbitration,
                    &decision,
                    &acceptance,
                    &expected_author_did(&action.author),
                ) {
                    Ok(()) => Ok(ValidateCallbackResult::Valid),
                    Err(error) => Ok(invalid(error)),
                }
            }
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) | FlatOp::RegisterUpdate(_) => Ok(
            ValidateCallbackResult::Invalid(
                "Justice adjudication attestations are append-only; updates are forbidden".into(),
            ),
        ),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Justice adjudication attestations are append-only; deletes are forbidden".into(),
        )),
        FlatOp::RegisterCreateLink { tag, .. } | FlatOp::RegisterDeleteLink { tag, .. } => {
            if tag.0.len() > 64 {
                Ok(ValidateCallbackResult::Invalid(
                    "Justice adjudication evidence link tag too long (max 64 bytes)".into(),
                ))
            } else {
                Ok(ValidateCallbackResult::Valid)
            }
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use justice_arbitration_integrity::{
        Arbitrator, ArbitratorRole, ArbitratorSelection, ArbitrationStatus, ArbitratorVote,
        DecisionOutcome, DecisionType, Remedy, VoteChoice,
    };

    fn ts() -> Timestamp {
        Timestamp::from_micros(1_000_000)
    }

    fn arbitration() -> Arbitration {
        Arbitration {
            id: "arbitration:1".into(),
            case_id: "case:1".into(),
            arbitrators: vec![Arbitrator {
                did: "did:example:arb1".into(),
                role: ArbitratorRole::Primary,
                selected_at: ts(),
                accepted: false,
                recused: false,
                recusal_reason: None,
            }],
            selection_method: ArbitratorSelection::PartyAgreed,
            status: ArbitrationStatus::PanelFormation,
            deliberation_deadline: None,
            created_at: ts(),
        }
    }

    fn decision() -> Decision {
        Decision {
            id: "decision:1".into(),
            case_id: "case:1".into(),
            arbitration_id: "arbitration:1".into(),
            decision_type: DecisionType::MeritsDecision,
            outcome: DecisionOutcome::ForComplainant,
            reasoning: "Exact decision".into(),
            remedies: Vec::<Remedy>::new(),
            votes: vec![ArbitratorVote {
                arbitrator: "did:example:arb1".into(),
                vote: VoteChoice::ForComplainant,
                timestamp: ts(),
            }],
            dissents: vec![],
            rendered_at: ts(),
            appeal_deadline: Timestamp::from_micros(2_000_000),
            finalized: false,
        }
    }

    fn fake_hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn acceptance() -> ArbitratorParticipationAttestationV1 {
        ArbitratorParticipationAttestationV1 {
            arbitration_action_hash: fake_hash(1),
            arbitration_id: "arbitration:1".into(),
            arbitrator: "did:example:arb1".into(),
            disposition: ParticipationDispositionV1::Accepted,
        }
    }

    fn vote() -> DecisionVoteAttestationV1 {
        DecisionVoteAttestationV1 {
            decision_action_hash: fake_hash(2),
            decision_id: "decision:1".into(),
            arbitration_action_hash: fake_hash(1),
            arbitration_id: "arbitration:1".into(),
            acceptance_attestation_action_hash: fake_hash(3),
            arbitrator: "did:example:arb1".into(),
            vote: AuthenticatedDecisionVoteChoiceV1::ForComplainant,
        }
    }

    #[test]
    fn exact_selected_arbitrator_can_attest_acceptance() {
        assert_eq!(
            validate_participation_against(&acceptance(), &arbitration(), "did:example:arb1"),
            Ok(())
        );
    }

    #[test]
    fn participation_cannot_impersonate_another_arbitrator() {
        assert_eq!(
            validate_participation_against(&acceptance(), &arbitration(), "did:example:other"),
            Err(EvidenceValidationError::AuthorMismatch)
        );
    }

    #[test]
    fn non_selected_actor_cannot_create_participation_evidence() {
        let mut attestation = acceptance();
        attestation.arbitrator = "did:example:outsider".into();
        assert_eq!(
            validate_participation_against(&attestation, &arbitration(), "did:example:outsider"),
            Err(EvidenceValidationError::ArbitratorNotSelected)
        );
    }

    #[test]
    fn exact_accepted_arbitrator_can_attest_vote() {
        assert_eq!(
            validate_vote_against(
                &vote(),
                &arbitration(),
                &decision(),
                &acceptance(),
                "did:example:arb1"
            ),
            Ok(())
        );
    }

    #[test]
    fn vote_cannot_impersonate_another_arbitrator() {
        assert_eq!(
            validate_vote_against(
                &vote(),
                &arbitration(),
                &decision(),
                &acceptance(),
                "did:example:other"
            ),
            Err(EvidenceValidationError::AuthorMismatch)
        );
    }

    #[test]
    fn vote_must_bind_exact_decision_identity() {
        let mut attestation = vote();
        attestation.decision_id = "decision:other".into();
        assert_eq!(
            validate_vote_against(
                &attestation,
                &arbitration(),
                &decision(),
                &acceptance(),
                "did:example:arb1"
            ),
            Err(EvidenceValidationError::DecisionIdMismatch)
        );
    }

    #[test]
    fn vote_cannot_use_another_arbitrators_acceptance() {
        let mut other = acceptance();
        other.arbitrator = "did:example:other".into();
        assert_eq!(
            validate_vote_against(
                &vote(),
                &arbitration(),
                &decision(),
                &other,
                "did:example:arb1"
            ),
            Err(EvidenceValidationError::AcceptanceArbitratorMismatch)
        );
    }

    #[test]
    fn vote_cannot_use_acceptance_from_another_arbitration_action() {
        let mut other = acceptance();
        other.arbitration_action_hash = fake_hash(9);
        assert_eq!(
            validate_vote_against(
                &vote(),
                &arbitration(),
                &decision(),
                &other,
                "did:example:arb1"
            ),
            Err(EvidenceValidationError::AcceptanceArbitrationActionMismatch)
        );
    }

    #[test]
    fn vote_cannot_use_recusal_as_acceptance() {
        let mut recusal = acceptance();
        recusal.disposition = ParticipationDispositionV1::Recused {
            reason: Some("conflict".into()),
        };
        assert_eq!(
            validate_vote_against(
                &vote(),
                &arbitration(),
                &decision(),
                &recusal,
                "did:example:arb1"
            ),
            Err(EvidenceValidationError::AcceptanceNotAccepted)
        );
    }

    #[test]
    fn recusal_reason_is_bounded() {
        let mut attestation = acceptance();
        attestation.disposition = ParticipationDispositionV1::Recused {
            reason: Some("x".repeat(4097)),
        };
        assert_eq!(
            validate_participation_against(&attestation, &arbitration(), "did:example:arb1"),
            Err(EvidenceValidationError::RecusalReasonTooLong)
        );
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Coordinator for individually authored Justice adjudication evidence.
//!
//! These calls derive the represented arbitrator from the committing agent.
//! Callers provide only exact referenced record hashes plus their own choice.
//! Integrity validation re-fetches the exact referenced records and enforces
//! membership/identity correspondence independently of this coordinator.

use hdk::prelude::*;
use justice_adjudication_evidence_integrity::{
    ArbitratorParticipationAttestationV1, AuthenticatedDecisionVoteChoiceV1,
    DecisionVoteAttestationV1, EntryTypes, LinkTypes, ParticipationDispositionV1,
};
use justice_arbitration_integrity::{Arbitration, Decision};

fn my_did() -> ExternResult<String> {
    let agent_info = agent_info()?;
    Ok(format!("did:mycelix:{}", agent_info.agent_initial_pubkey))
}

fn exact_arbitration(action_hash: &ActionHash) -> ExternResult<Arbitration> {
    let record = get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "exact Arbitration action not found".into()
        ))
    })?;
    record
        .entry()
        .to_app_option::<Arbitration>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "could not decode exact Arbitration: {error:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "referenced action is not an Arbitration".into()
            ))
        })
}

fn exact_decision(action_hash: &ActionHash) -> ExternResult<Decision> {
    let record = get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "exact Decision action not found".into()
        ))
    })?;
    record
        .entry()
        .to_app_option::<Decision>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "could not decode exact Decision: {error:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "referenced action is not a Decision".into()
            ))
        })
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordParticipationInputV1 {
    pub arbitration_action_hash: ActionHash,
    pub disposition: ParticipationDispositionV1,
}

/// Record this exact selected arbitrator's acceptance or recusal as a new,
/// immutable evidence action.
#[hdk_extern]
pub fn record_participation_v1(input: RecordParticipationInputV1) -> ExternResult<Record> {
    let arbitration = exact_arbitration(&input.arbitration_action_hash)?;
    let attestation = ArbitratorParticipationAttestationV1 {
        arbitration_action_hash: input.arbitration_action_hash.clone(),
        arbitration_id: arbitration.id,
        arbitrator: my_did()?,
        disposition: input.disposition,
    };

    let action_hash = create_entry(&EntryTypes::ArbitratorParticipationAttestationV1(
        attestation,
    ))?;
    create_link(
        input.arbitration_action_hash,
        action_hash.clone(),
        LinkTypes::ArbitrationToParticipationAttestations,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "could not read created participation attestation".into()
        ))
    })
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordDecisionVoteInputV1 {
    pub decision_action_hash: ActionHash,
    pub arbitration_action_hash: ActionHash,
    /// Exact `Accepted` participation attestation authored by this arbitrator
    /// for the exact Arbitration. Integrity validates the full provenance edge.
    pub acceptance_attestation_action_hash: ActionHash,
    pub vote: AuthenticatedDecisionVoteChoiceV1,
}

/// Attest this exact accepted arbitrator's vote to one exact Decision action.
#[hdk_extern]
pub fn attest_decision_vote_v1(input: RecordDecisionVoteInputV1) -> ExternResult<Record> {
    let decision = exact_decision(&input.decision_action_hash)?;
    let arbitration = exact_arbitration(&input.arbitration_action_hash)?;

    if decision.arbitration_id != arbitration.id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Decision and Arbitration identities do not correspond".into()
        )));
    }

    let attestation = DecisionVoteAttestationV1 {
        decision_action_hash: input.decision_action_hash.clone(),
        decision_id: decision.id,
        arbitration_action_hash: input.arbitration_action_hash,
        arbitration_id: arbitration.id,
        acceptance_attestation_action_hash: input.acceptance_attestation_action_hash,
        arbitrator: my_did()?,
        vote: input.vote,
    };

    let action_hash = create_entry(&EntryTypes::DecisionVoteAttestationV1(attestation))?;
    create_link(
        input.decision_action_hash,
        action_hash.clone(),
        LinkTypes::DecisionToVoteAttestations,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "could not read created decision vote attestation".into()
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    #[test]
    fn participation_input_roundtrips() {
        let input = RecordParticipationInputV1 {
            arbitration_action_hash: fake_hash(1),
            disposition: ParticipationDispositionV1::Recused {
                reason: Some("conflict".into()),
            },
        };
        let json = serde_json::to_string(&input).unwrap();
        let decoded: RecordParticipationInputV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.arbitration_action_hash, input.arbitration_action_hash);
        assert_eq!(decoded.disposition, input.disposition);
    }

    #[test]
    fn vote_input_roundtrips() {
        let input = RecordDecisionVoteInputV1 {
            decision_action_hash: fake_hash(2),
            arbitration_action_hash: fake_hash(1),
            acceptance_attestation_action_hash: fake_hash(3),
            vote: AuthenticatedDecisionVoteChoiceV1::ForComplainant,
        };
        let json = serde_json::to_string(&input).unwrap();
        let decoded: RecordDecisionVoteInputV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.decision_action_hash, input.decision_action_hash);
        assert_eq!(decoded.arbitration_action_hash, input.arbitration_action_hash);
        assert_eq!(
            decoded.acceptance_attestation_action_hash,
            input.acceptance_attestation_action_hash
        );
        assert_eq!(decoded.vote, input.vote);
    }
}

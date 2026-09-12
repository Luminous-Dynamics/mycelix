#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-019 pure witness allocation journal theorem for SAP V2 transfers.
//!
//! A witness allocates one immutable economic identity for an entire canonical
//! input-note set. Physical spend/allocation actions are provenance, not economic
//! identity. Same-transfer retries are idempotent; a distinct transfer touching
//! any allocated input refuses the entire candidate batch.

use finance_sap_value_notes::{
    SapTransferV2, MAX_ACTION_REFERENCE_LEN, MAX_DID_LEN, MAX_ID_LEN, MAX_TRANSFER_INPUTS,
};
use finance_sap_witness_policy::SapTransferWitnessPolicyV1;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const SAP_WITNESS_ALLOCATION_V1_SCHEMA_VERSION: u16 = 1;
pub const MAX_WITNESS_HISTORY_OBSERVATIONS: usize = 4096;
const ALLOCATION_ID_DOMAIN: &[u8] = b"mycelix:sap-witness-allocation-v1\0";
const POLICY_FINGERPRINT_PREFIX: &str = "wpolicy:v1:";
const MAX_POLICY_FINGERPRINT_LEN: usize = 128;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapWitnessAllocationV1 {
    pub schema_version: u16,
    /// Economic identity. Intentionally excludes physical action references.
    pub allocation_id: String,
    pub witness_did: String,
    pub policy_fingerprint: String,
    pub transfer_id: String,
    /// Exact physical parent transfer-spend action this allocation observed.
    pub spend_action_reference: String,
    pub input_note_ids: Vec<String>,
}

impl SapWitnessAllocationV1 {
    pub fn derive(
        witness_did: String,
        policy: &SapTransferWitnessPolicyV1,
        transfer: &SapTransferV2,
        spend_action_reference: String,
    ) -> Result<Self, SapWitnessAllocationError> {
        policy
            .validate()
            .map_err(|_| SapWitnessAllocationError::InvalidPolicy)?;
        transfer
            .validate_shape()
            .map_err(|_| SapWitnessAllocationError::InvalidTransfer)?;
        validate_did(&witness_did)?;
        validate_action_reference(&spend_action_reference)?;
        require_optional_witness(policy, &witness_did)?;

        let mut allocation = Self {
            schema_version: SAP_WITNESS_ALLOCATION_V1_SCHEMA_VERSION,
            allocation_id: String::new(),
            witness_did,
            policy_fingerprint: policy.policy_fingerprint.clone(),
            transfer_id: transfer.transfer_id.clone(),
            spend_action_reference,
            input_note_ids: transfer.input_note_ids.clone(),
        };
        allocation.allocation_id = allocation.expected_allocation_id()?;
        allocation.validate_for(policy, transfer)?;
        Ok(allocation)
    }

    pub fn validate_shape(&self) -> Result<(), SapWitnessAllocationError> {
        if self.schema_version != SAP_WITNESS_ALLOCATION_V1_SCHEMA_VERSION {
            return Err(SapWitnessAllocationError::UnsupportedSchemaVersion);
        }
        validate_id(&self.allocation_id)?;
        validate_did(&self.witness_did)?;
        validate_policy_fingerprint(&self.policy_fingerprint)?;
        validate_id(&self.transfer_id)?;
        validate_action_reference(&self.spend_action_reference)?;
        validate_sorted_unique_input_ids(&self.input_note_ids)?;
        if self.allocation_id != self.expected_allocation_id()? {
            return Err(SapWitnessAllocationError::AllocationIdMismatch);
        }
        Ok(())
    }

    pub fn validate_for(
        &self,
        policy: &SapTransferWitnessPolicyV1,
        transfer: &SapTransferV2,
    ) -> Result<(), SapWitnessAllocationError> {
        self.validate_shape()?;
        policy
            .validate()
            .map_err(|_| SapWitnessAllocationError::InvalidPolicy)?;
        transfer
            .validate_shape()
            .map_err(|_| SapWitnessAllocationError::InvalidTransfer)?;
        require_optional_witness(policy, &self.witness_did)?;
        if self.policy_fingerprint != policy.policy_fingerprint {
            return Err(SapWitnessAllocationError::PolicyFingerprintMismatch);
        }
        if self.transfer_id != transfer.transfer_id {
            return Err(SapWitnessAllocationError::TransferIdMismatch);
        }
        if self.input_note_ids != transfer.input_note_ids {
            return Err(SapWitnessAllocationError::InputSetMismatch);
        }
        Ok(())
    }

    pub fn expected_allocation_id(&self) -> Result<String, SapWitnessAllocationError> {
        if self.schema_version != SAP_WITNESS_ALLOCATION_V1_SCHEMA_VERSION {
            return Err(SapWitnessAllocationError::UnsupportedSchemaVersion);
        }
        validate_did(&self.witness_did)?;
        validate_policy_fingerprint(&self.policy_fingerprint)?;
        validate_id(&self.transfer_id)?;
        validate_sorted_unique_input_ids(&self.input_note_ids)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(ALLOCATION_ID_DOMAIN);
        hash_u16(&mut hasher, self.schema_version);
        hash_string(&mut hasher, &self.witness_did);
        hash_string(&mut hasher, &self.policy_fingerprint);
        hash_string(&mut hasher, &self.transfer_id);
        hash_u32(&mut hasher, self.input_note_ids.len() as u32);
        for note_id in &self.input_note_ids {
            hash_string(&mut hasher, note_id);
        }
        Ok(format!("walloc:v1:{}", hasher.finalize().to_hex()))
    }

    fn same_economic_identity(&self, other: &Self) -> bool {
        self.schema_version == other.schema_version
            && self.allocation_id == other.allocation_id
            && self.witness_did == other.witness_did
            && self.policy_fingerprint == other.policy_fingerprint
            && self.transfer_id == other.transfer_id
            && self.input_note_ids == other.input_note_ids
    }
}

/// Exact allocation action after a future integration layer authenticates the
/// physical Holochain action and its real author. Not a wire/deserializable proof.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedWitnessAllocationObservationV1 {
    allocation_action_reference: String,
    action_author_did: String,
    allocation: SapWitnessAllocationV1,
}

impl AuthenticatedWitnessAllocationObservationV1 {
    pub fn from_authenticated_action(
        allocation_action_reference: String,
        action_author_did: String,
        allocation: SapWitnessAllocationV1,
    ) -> Result<Self, SapWitnessAllocationError> {
        let observation = Self {
            allocation_action_reference,
            action_author_did,
            allocation,
        };
        observation.validate_shape()?;
        Ok(observation)
    }

    pub fn allocation_action_reference(&self) -> &str {
        &self.allocation_action_reference
    }

    pub fn allocation(&self) -> &SapWitnessAllocationV1 {
        &self.allocation
    }

    pub fn validate_shape(&self) -> Result<(), SapWitnessAllocationError> {
        validate_action_reference(&self.allocation_action_reference)?;
        validate_did(&self.action_author_did)?;
        self.allocation.validate_shape()?;
        if self.action_author_did != self.allocation.witness_did {
            return Err(SapWitnessAllocationError::AllocationAuthorMismatch);
        }
        Ok(())
    }

    fn validate_for_policy(
        &self,
        policy: &SapTransferWitnessPolicyV1,
        expected_witness_did: &str,
    ) -> Result<(), SapWitnessAllocationError> {
        self.validate_shape()?;
        policy
            .validate()
            .map_err(|_| SapWitnessAllocationError::InvalidPolicy)?;
        validate_did(expected_witness_did)?;
        require_optional_witness(policy, expected_witness_did)?;
        if self.allocation.witness_did != expected_witness_did {
            return Err(SapWitnessAllocationError::ForeignWitnessObservation);
        }
        // V1 deliberately fails closed across policy rotation. Historical policy
        // resolution must be explicit before old allocations can be interpreted.
        if self.allocation.policy_fingerprint != policy.policy_fingerprint {
            return Err(SapWitnessAllocationError::HistoricalPolicyMismatch);
        }
        Ok(())
    }
}

/// Integration-attested complete local allocation history for one witness/policy.
///
/// The pure crate cannot query Holochain and therefore cannot independently prove
/// completeness. This non-deserializable capability makes that trust boundary
/// explicit: a future witness service may construct it only after completing its
/// authoritative local source-chain/journal query and authenticating every action.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedCompleteWitnessAllocationHistoryV1 {
    witness_did: String,
    policy_fingerprint: String,
    observations: Vec<AuthenticatedWitnessAllocationObservationV1>,
}

impl AuthenticatedCompleteWitnessAllocationHistoryV1 {
    pub fn from_authenticated_complete_local_history(
        policy: &SapTransferWitnessPolicyV1,
        witness_did: String,
        observations: Vec<AuthenticatedWitnessAllocationObservationV1>,
    ) -> Result<Self, SapWitnessAllocationError> {
        policy
            .validate()
            .map_err(|_| SapWitnessAllocationError::InvalidPolicy)?;
        validate_did(&witness_did)?;
        require_optional_witness(policy, &witness_did)?;
        canonicalize_history(policy, &witness_did, &observations)?;
        Ok(Self {
            witness_did,
            policy_fingerprint: policy.policy_fingerprint.clone(),
            observations,
        })
    }

    pub fn witness_did(&self) -> &str {
        &self.witness_did
    }

    pub fn observations(&self) -> &[AuthenticatedWitnessAllocationObservationV1] {
        &self.observations
    }

    fn validate_for(
        &self,
        policy: &SapTransferWitnessPolicyV1,
        expected_witness_did: &str,
    ) -> Result<(), SapWitnessAllocationError> {
        policy
            .validate()
            .map_err(|_| SapWitnessAllocationError::InvalidPolicy)?;
        validate_did(expected_witness_did)?;
        if self.witness_did != expected_witness_did {
            return Err(SapWitnessAllocationError::CompleteHistoryWitnessMismatch);
        }
        if self.policy_fingerprint != policy.policy_fingerprint {
            return Err(SapWitnessAllocationError::CompleteHistoryPolicyMismatch);
        }
        canonicalize_history(policy, expected_witness_did, &self.observations)?;
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapWitnessInputAllocationStateV1 {
    Unallocated {
        note_id: String,
    },
    AllocatedTo {
        note_id: String,
        transfer_id: String,
        allocation_action_references: Vec<String>,
        spend_action_references: Vec<String>,
    },
    Conflict {
        note_id: String,
        transfer_ids: Vec<String>,
        allocation_action_references: Vec<String>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapWitnessAllocationDecisionV1 {
    ApproveNew {
        allocation: SapWitnessAllocationV1,
    },
    IdempotentExisting {
        transfer_id: String,
        allocation_id: String,
        allocation_action_references: Vec<String>,
        spend_action_references: Vec<String>,
    },
    RefuseConflict {
        conflicting_note_ids: Vec<String>,
        conflicting_transfer_ids: Vec<String>,
        allocation_action_references: Vec<String>,
    },
}

/// Decide one whole-input-set allocation using an integration-attested complete
/// local witness history. This gives logical all-or-nothing semantics, not a claim
/// of Holochain transaction isolation or global consensus.
pub fn decide_witness_allocation(
    policy: &SapTransferWitnessPolicyV1,
    witness_did: &str,
    transfer: &SapTransferV2,
    spend_action_reference: &str,
    history: &AuthenticatedCompleteWitnessAllocationHistoryV1,
) -> Result<SapWitnessAllocationDecisionV1, SapWitnessAllocationError> {
    policy
        .validate()
        .map_err(|_| SapWitnessAllocationError::InvalidPolicy)?;
    transfer
        .validate_shape()
        .map_err(|_| SapWitnessAllocationError::InvalidTransfer)?;
    validate_did(witness_did)?;
    validate_action_reference(spend_action_reference)?;
    require_optional_witness(policy, witness_did)?;
    history.validate_for(policy, witness_did)?;

    let candidate = SapWitnessAllocationV1::derive(
        witness_did.to_string(),
        policy,
        transfer,
        spend_action_reference.to_string(),
    )?;
    let physical = canonicalize_history(policy, witness_did, history.observations())?;

    for observation in physical.values() {
        let allocation = observation.allocation();
        if allocation.transfer_id == candidate.transfer_id
            && allocation.input_note_ids != candidate.input_note_ids
        {
            return Err(SapWitnessAllocationError::ConflictingTransferFacts);
        }
    }

    let mut conflicts = BTreeSet::new();
    let mut conflicting_transfers = BTreeSet::new();
    let mut conflicting_actions = BTreeSet::new();
    let mut existing_actions = BTreeSet::new();
    let mut existing_spends = BTreeSet::new();
    let mut candidate_allocated_notes = BTreeSet::new();

    for note_id in &candidate.input_note_ids {
        match classify_input_from_history(note_id, physical.values().copied())? {
            SapWitnessInputAllocationStateV1::Unallocated { .. } => {}
            SapWitnessInputAllocationStateV1::AllocatedTo {
                transfer_id,
                allocation_action_references,
                spend_action_references,
                ..
            } if transfer_id == candidate.transfer_id => {
                candidate_allocated_notes.insert(note_id.clone());
                existing_actions.extend(allocation_action_references);
                existing_spends.extend(spend_action_references);
            }
            SapWitnessInputAllocationStateV1::AllocatedTo {
                transfer_id,
                allocation_action_references,
                ..
            } => {
                conflicts.insert(note_id.clone());
                conflicting_transfers.insert(transfer_id);
                conflicting_actions.extend(allocation_action_references);
            }
            SapWitnessInputAllocationStateV1::Conflict {
                transfer_ids,
                allocation_action_references,
                ..
            } => {
                conflicts.insert(note_id.clone());
                conflicting_transfers.extend(transfer_ids);
                conflicting_actions.extend(allocation_action_references);
            }
        }
    }

    if !conflicts.is_empty() {
        return Ok(SapWitnessAllocationDecisionV1::RefuseConflict {
            conflicting_note_ids: conflicts.into_iter().collect(),
            conflicting_transfer_ids: conflicting_transfers.into_iter().collect(),
            allocation_action_references: conflicting_actions.into_iter().collect(),
        });
    }

    if !candidate_allocated_notes.is_empty() {
        if candidate_allocated_notes.len() != candidate.input_note_ids.len() {
            return Err(SapWitnessAllocationError::PartialSameTransferHistory);
        }
        return Ok(SapWitnessAllocationDecisionV1::IdempotentExisting {
            transfer_id: candidate.transfer_id,
            allocation_id: candidate.allocation_id,
            allocation_action_references: existing_actions.into_iter().collect(),
            spend_action_references: existing_spends.into_iter().collect(),
        });
    }

    Ok(SapWitnessAllocationDecisionV1::ApproveNew {
        allocation: candidate,
    })
}

pub fn classify_witness_input_allocation(
    policy: &SapTransferWitnessPolicyV1,
    witness_did: &str,
    note_id: &str,
    history: &AuthenticatedCompleteWitnessAllocationHistoryV1,
) -> Result<SapWitnessInputAllocationStateV1, SapWitnessAllocationError> {
    validate_id(note_id)?;
    history.validate_for(policy, witness_did)?;
    let physical = canonicalize_history(policy, witness_did, history.observations())?;
    classify_input_from_history(note_id, physical.values().copied())
}

fn canonicalize_history<'a>(
    policy: &SapTransferWitnessPolicyV1,
    witness_did: &str,
    history: &'a [AuthenticatedWitnessAllocationObservationV1],
) -> Result<
    BTreeMap<String, &'a AuthenticatedWitnessAllocationObservationV1>,
    SapWitnessAllocationError,
> {
    if history.len() > MAX_WITNESS_HISTORY_OBSERVATIONS {
        return Err(SapWitnessAllocationError::TooManyHistoryObservations);
    }
    let mut physical = BTreeMap::new();
    let mut economic: BTreeMap<&str, &SapWitnessAllocationV1> = BTreeMap::new();

    for observation in history {
        observation.validate_for_policy(policy, witness_did)?;
        match physical.get(observation.allocation_action_reference()) {
            Some(existing) if *existing == observation => {}
            Some(_) => return Err(SapWitnessAllocationError::InconsistentDuplicateAction),
            None => {
                physical.insert(
                    observation.allocation_action_reference().to_string(),
                    observation,
                );
            }
        }

        let allocation = observation.allocation();
        match economic.get(allocation.allocation_id.as_str()) {
            Some(existing) if existing.same_economic_identity(allocation) => {}
            Some(_) => return Err(SapWitnessAllocationError::InconsistentAllocationIdentity),
            None => {
                economic.insert(allocation.allocation_id.as_str(), allocation);
            }
        }
    }
    Ok(physical)
}

fn classify_input_from_history<'a>(
    note_id: &str,
    history: impl Iterator<Item = &'a AuthenticatedWitnessAllocationObservationV1>,
) -> Result<SapWitnessInputAllocationStateV1, SapWitnessAllocationError> {
    validate_id(note_id)?;
    let mut by_transfer: BTreeMap<String, (BTreeSet<String>, BTreeSet<String>)> = BTreeMap::new();
    for observation in history {
        let allocation = observation.allocation();
        if !allocation.input_note_ids.iter().any(|id| id == note_id) {
            continue;
        }
        let aggregate = by_transfer
            .entry(allocation.transfer_id.clone())
            .or_insert_with(|| (BTreeSet::new(), BTreeSet::new()));
        aggregate
            .0
            .insert(observation.allocation_action_reference().to_string());
        aggregate
            .1
            .insert(allocation.spend_action_reference.clone());
    }

    if by_transfer.is_empty() {
        return Ok(SapWitnessInputAllocationStateV1::Unallocated {
            note_id: note_id.to_string(),
        });
    }
    if by_transfer.len() == 1 {
        if let Some((transfer_id, (actions, spends))) = by_transfer.into_iter().next() {
            return Ok(SapWitnessInputAllocationStateV1::AllocatedTo {
                note_id: note_id.to_string(),
                transfer_id,
                allocation_action_references: actions.into_iter().collect(),
                spend_action_references: spends.into_iter().collect(),
            });
        }
        return Err(SapWitnessAllocationError::InternalInvariant);
    }

    let transfer_ids = by_transfer.keys().cloned().collect();
    let allocation_action_references = by_transfer
        .values()
        .flat_map(|(actions, _)| actions.iter().cloned())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    Ok(SapWitnessInputAllocationStateV1::Conflict {
        note_id: note_id.to_string(),
        transfer_ids,
        allocation_action_references,
    })
}

fn require_optional_witness(
    policy: &SapTransferWitnessPolicyV1,
    witness_did: &str,
) -> Result<(), SapWitnessAllocationError> {
    if witness_did == policy.enzyme_did {
        return Err(SapWitnessAllocationError::EnzymeCannotBeOptionalWitness);
    }
    if policy
        .optional_witness_dids
        .binary_search_by(|candidate| candidate.as_str().cmp(witness_did))
        .is_ok()
    {
        Ok(())
    } else {
        Err(SapWitnessAllocationError::IneligibleWitness)
    }
}

fn validate_sorted_unique_input_ids(values: &[String]) -> Result<(), SapWitnessAllocationError> {
    if values.is_empty() {
        return Err(SapWitnessAllocationError::EmptyInputSet);
    }
    if values.len() > MAX_TRANSFER_INPUTS {
        return Err(SapWitnessAllocationError::TooManyInputs);
    }
    let mut previous: Option<&str> = None;
    for value in values {
        validate_id(value)?;
        if previous.is_some_and(|prev| prev >= value.as_str()) {
            return Err(SapWitnessAllocationError::NonCanonicalInputSet);
        }
        previous = Some(value);
    }
    Ok(())
}

fn validate_did(value: &str) -> Result<(), SapWitnessAllocationError> {
    if !value.starts_with("did:") || value.len() > MAX_DID_LEN {
        Err(SapWitnessAllocationError::InvalidDid)
    } else {
        Ok(())
    }
}
fn validate_id(value: &str) -> Result<(), SapWitnessAllocationError> {
    if value.is_empty() || value.len() > MAX_ID_LEN {
        Err(SapWitnessAllocationError::InvalidId)
    } else {
        Ok(())
    }
}
fn validate_policy_fingerprint(value: &str) -> Result<(), SapWitnessAllocationError> {
    if !value.starts_with(POLICY_FINGERPRINT_PREFIX)
        || value.len() <= POLICY_FINGERPRINT_PREFIX.len()
        || value.len() > MAX_POLICY_FINGERPRINT_LEN
    {
        Err(SapWitnessAllocationError::InvalidPolicyFingerprint)
    } else {
        Ok(())
    }
}
fn validate_action_reference(value: &str) -> Result<(), SapWitnessAllocationError> {
    if value.is_empty() || value.len() > MAX_ACTION_REFERENCE_LEN {
        Err(SapWitnessAllocationError::InvalidActionReference)
    } else {
        Ok(())
    }
}
fn hash_u16(hasher: &mut blake3::Hasher, value: u16) {
    hasher.update(&value.to_be_bytes());
}
fn hash_u32(hasher: &mut blake3::Hasher, value: u32) {
    hasher.update(&value.to_be_bytes());
}
fn hash_string(hasher: &mut blake3::Hasher, value: &str) {
    hash_u32(hasher, value.len() as u32);
    hasher.update(value.as_bytes());
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapWitnessAllocationError {
    UnsupportedSchemaVersion,
    InvalidPolicy,
    InvalidTransfer,
    InvalidDid,
    InvalidId,
    InvalidActionReference,
    InvalidPolicyFingerprint,
    EmptyInputSet,
    TooManyInputs,
    NonCanonicalInputSet,
    AllocationIdMismatch,
    EnzymeCannotBeOptionalWitness,
    IneligibleWitness,
    AllocationAuthorMismatch,
    ForeignWitnessObservation,
    HistoricalPolicyMismatch,
    PolicyFingerprintMismatch,
    TransferIdMismatch,
    InputSetMismatch,
    ConflictingTransferFacts,
    InconsistentDuplicateAction,
    InconsistentAllocationIdentity,
    PartialSameTransferHistory,
    TooManyHistoryObservations,
    CompleteHistoryWitnessMismatch,
    CompleteHistoryPolicyMismatch,
    InternalInvariant,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_sap_account_v2::ValidatedCollateralClaimV2;
    use finance_sap_value_notes::SapValueNoteV2;

    fn policy() -> SapTransferWitnessPolicyV1 {
        SapTransferWitnessPolicyV1::new(
            "sap-main".into(),
            1,
            "did:mycelix:enzyme".into(),
            vec![
                "did:mycelix:w1".into(),
                "did:mycelix:w2".into(),
                "did:mycelix:w3".into(),
            ],
            2,
        )
        .unwrap()
    }

    fn note(mint: &str, amount: u64) -> SapValueNoteV2 {
        SapValueNoteV2::from_collateral_claim(&ValidatedCollateralClaimV2 {
            claim_action_reference: format!("uhCkk-claim-{mint}"),
            action_author_did: "did:mycelix:alice".into(),
            member_did: "did:mycelix:alice".into(),
            issuance_receipt_action_reference: format!("uhCkk-receipt-{mint}"),
            mint_id: format!("mint:{mint}"),
            deposit_id: format!("deposit:{mint}"),
            amount,
        })
        .unwrap()
    }

    fn transfer(recipient: &str, inputs: &[SapValueNoteV2], amount: u64) -> SapTransferV2 {
        SapTransferV2::derive("did:mycelix:alice".into(), recipient.into(), inputs, amount).unwrap()
    }

    fn observation(
        action: &str,
        allocation: SapWitnessAllocationV1,
    ) -> AuthenticatedWitnessAllocationObservationV1 {
        AuthenticatedWitnessAllocationObservationV1::from_authenticated_action(
            action.into(),
            allocation.witness_did.clone(),
            allocation,
        )
        .unwrap()
    }

    fn complete(
        policy: &SapTransferWitnessPolicyV1,
        witness: &str,
        observations: Vec<AuthenticatedWitnessAllocationObservationV1>,
    ) -> AuthenticatedCompleteWitnessAllocationHistoryV1 {
        AuthenticatedCompleteWitnessAllocationHistoryV1::from_authenticated_complete_local_history(
            policy,
            witness.into(),
            observations,
        )
        .unwrap()
    }

    #[test]
    fn empty_complete_history_approves_whole_candidate() {
        let p = policy();
        let t = transfer("did:mycelix:bob", &[note("a", 40), note("b", 60)], 75);
        let history = complete(&p, "did:mycelix:w1", vec![]);
        assert!(matches!(
            decide_witness_allocation(&p, "did:mycelix:w1", &t, "uhCkk-spend", &history).unwrap(),
            SapWitnessAllocationDecisionV1::ApproveNew { .. }
        ));
    }

    #[test]
    fn physical_duplicate_spends_share_one_economic_allocation_id() {
        let p = policy();
        let t = transfer("did:mycelix:bob", &[note("a", 40)], 20);
        let a =
            SapWitnessAllocationV1::derive("did:mycelix:w1".into(), &p, &t, "uhCkk-spend-a".into())
                .unwrap();
        let b =
            SapWitnessAllocationV1::derive("did:mycelix:w1".into(), &p, &t, "uhCkk-spend-b".into())
                .unwrap();
        assert_eq!(a.allocation_id, b.allocation_id);
        assert_ne!(a.spend_action_reference, b.spend_action_reference);
    }

    #[test]
    fn same_economic_transfer_is_idempotent() {
        let p = policy();
        let t = transfer("did:mycelix:bob", &[note("a", 40)], 20);
        let existing = SapWitnessAllocationV1::derive(
            "did:mycelix:w1".into(),
            &p,
            &t,
            "uhCkk-spend-original".into(),
        )
        .unwrap();
        let expected_id = existing.allocation_id.clone();
        let history = complete(
            &p,
            "did:mycelix:w1",
            vec![observation("uhCkk-allocation", existing)],
        );
        match decide_witness_allocation(&p, "did:mycelix:w1", &t, "uhCkk-spend-retry", &history)
            .unwrap()
        {
            SapWitnessAllocationDecisionV1::IdempotentExisting { allocation_id, .. } => {
                assert_eq!(allocation_id, expected_id)
            }
            other => panic!("expected idempotent existing allocation, got {other:?}"),
        }
    }

    #[test]
    fn one_conflicting_input_refuses_entire_candidate() {
        let p = policy();
        let a = note("a", 40);
        let prior = transfer("did:mycelix:carol", &[a.clone()], 20);
        let candidate = transfer("did:mycelix:bob", &[a.clone(), note("b", 60)], 75);
        let prior_allocation = SapWitnessAllocationV1::derive(
            "did:mycelix:w1".into(),
            &p,
            &prior,
            "uhCkk-spend-prior".into(),
        )
        .unwrap();
        let history = complete(
            &p,
            "did:mycelix:w1",
            vec![observation("uhCkk-allocation-prior", prior_allocation)],
        );
        match decide_witness_allocation(
            &p,
            "did:mycelix:w1",
            &candidate,
            "uhCkk-spend-candidate",
            &history,
        )
        .unwrap()
        {
            SapWitnessAllocationDecisionV1::RefuseConflict {
                conflicting_note_ids,
                conflicting_transfer_ids,
                ..
            } => {
                assert_eq!(conflicting_note_ids, vec![a.note_id]);
                assert_eq!(conflicting_transfer_ids, vec![prior.transfer_id]);
            }
            other => panic!("expected whole-batch refusal, got {other:?}"),
        }
    }

    #[test]
    fn policy_rotation_history_fails_closed() {
        let old = policy();
        let t = transfer("did:mycelix:bob", &[note("a", 40)], 20);
        let allocation =
            SapWitnessAllocationV1::derive("did:mycelix:w1".into(), &old, &t, "uhCkk-spend".into())
                .unwrap();
        let raw = vec![observation("uhCkk-allocation", allocation)];
        let new = SapTransferWitnessPolicyV1::new(
            "sap-main".into(),
            2,
            "did:mycelix:enzyme".into(),
            vec![
                "did:mycelix:w1".into(),
                "did:mycelix:w2".into(),
                "did:mycelix:w3".into(),
            ],
            2,
        )
        .unwrap();
        assert_eq!(
            AuthenticatedCompleteWitnessAllocationHistoryV1::from_authenticated_complete_local_history(
                &new,
                "did:mycelix:w1".into(),
                raw,
            ),
            Err(SapWitnessAllocationError::HistoricalPolicyMismatch)
        );
    }

    #[test]
    fn authenticated_observation_binds_real_author_to_witness() {
        let p = policy();
        let t = transfer("did:mycelix:bob", &[note("a", 40)], 20);
        let allocation =
            SapWitnessAllocationV1::derive("did:mycelix:w1".into(), &p, &t, "uhCkk-spend".into())
                .unwrap();
        assert_eq!(
            AuthenticatedWitnessAllocationObservationV1::from_authenticated_action(
                "uhCkk-allocation".into(),
                "did:mycelix:mallory".into(),
                allocation,
            ),
            Err(SapWitnessAllocationError::AllocationAuthorMismatch)
        );
    }

    #[test]
    fn enzyme_cannot_be_counted_as_optional_witness() {
        let p = policy();
        let t = transfer("did:mycelix:bob", &[note("a", 40)], 20);
        assert_eq!(
            SapWitnessAllocationV1::derive(
                "did:mycelix:enzyme".into(),
                &p,
                &t,
                "uhCkk-spend".into(),
            ),
            Err(SapWitnessAllocationError::EnzymeCannotBeOptionalWitness)
        );
    }

    #[test]
    fn persistent_record_round_trip_requires_revalidation() {
        let p = policy();
        let t = transfer("did:mycelix:bob", &[note("a", 40)], 20);
        let allocation =
            SapWitnessAllocationV1::derive("did:mycelix:w1".into(), &p, &t, "uhCkk-spend".into())
                .unwrap();
        let json = serde_json::to_string(&allocation).unwrap();
        let decoded: SapWitnessAllocationV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.validate_for(&p, &t), Ok(()));
    }
}

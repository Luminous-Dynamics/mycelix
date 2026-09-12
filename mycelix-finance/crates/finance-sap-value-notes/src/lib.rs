#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-015 pure SAP V2 value-note transfer theorem.
//!
//! Economic identity, physical Holochain action identity, and assurance evidence
//! are deliberately separate. Duplicate physical observations of one canonical
//! transfer do not create extra value. Distinct canonical transfers consuming one
//! note are an explicit conflict. Exact origin/action/witness authentication remains
//! the responsibility of the eventual Holochain integration boundary.

use finance_sap_account_v2::ValidatedCollateralClaimV2;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const SAP_VALUE_NOTE_V2_SCHEMA_VERSION: u16 = 1;
pub const SAP_TRANSFER_V2_SCHEMA_VERSION: u16 = 1;
pub const SAP_TRANSFER_CLAIM_V2_SCHEMA_VERSION: u16 = 1;
pub const MAX_DID_LEN: usize = 256;
pub const MAX_ID_LEN: usize = 256;
pub const MAX_ACTION_REFERENCE_LEN: usize = 256;
pub const MAX_TRANSFER_INPUTS: usize = 256;

const COLLATERAL_NOTE_DOMAIN: &[u8] = b"mycelix:sap-note-v2:collateral\0";
const TRANSFER_ID_DOMAIN: &[u8] = b"mycelix:sap-transfer-v2\0";
const TRANSFER_OUTPUT_DOMAIN: &[u8] = b"mycelix:sap-note-v2:transfer-output\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SapTransferOutputRoleV2 {
    Recipient,
    Change,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapValueNoteOriginV2 {
    CollateralMint {
        mint_id: String,
        deposit_id: String,
    },
    TransferOutput {
        transfer_id: String,
        role: SapTransferOutputRoleV2,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapValueNoteV2 {
    pub schema_version: u16,
    pub note_id: String,
    pub owner_did: String,
    pub amount: u64,
    pub origin: SapValueNoteOriginV2,
}

impl SapValueNoteV2 {
    /// Derive the initial SAP note from one already-authenticated FIN-SAFE-014
    /// collateral claim. The note is keyed by canonical `mint_id`, not by a
    /// physical claim/receipt action hash.
    pub fn from_collateral_claim(
        claim: &ValidatedCollateralClaimV2,
    ) -> Result<Self, SapValueNoteError> {
        claim
            .validate_shape()
            .map_err(|_| SapValueNoteError::InvalidCollateralClaim)?;
        let note = Self {
            schema_version: SAP_VALUE_NOTE_V2_SCHEMA_VERSION,
            note_id: collateral_note_id(&claim.mint_id),
            owner_did: claim.member_did.clone(),
            amount: claim.amount,
            origin: SapValueNoteOriginV2::CollateralMint {
                mint_id: claim.mint_id.clone(),
                deposit_id: claim.deposit_id.clone(),
            },
        };
        note.validate_shape()?;
        Ok(note)
    }

    fn transfer_output(
        transfer_id: &str,
        role: SapTransferOutputRoleV2,
        owner_did: String,
        amount: u64,
    ) -> Result<Self, SapValueNoteError> {
        let note = Self {
            schema_version: SAP_VALUE_NOTE_V2_SCHEMA_VERSION,
            note_id: transfer_output_note_id(transfer_id, role),
            owner_did,
            amount,
            origin: SapValueNoteOriginV2::TransferOutput {
                transfer_id: transfer_id.to_string(),
                role,
            },
        };
        note.validate_shape()?;
        Ok(note)
    }

    pub fn validate_shape(&self) -> Result<(), SapValueNoteError> {
        if self.schema_version != SAP_VALUE_NOTE_V2_SCHEMA_VERSION {
            return Err(SapValueNoteError::UnsupportedNoteSchema);
        }
        validate_id(&self.note_id)?;
        validate_did(&self.owner_did)?;
        if self.amount == 0 {
            return Err(SapValueNoteError::ZeroAmount);
        }
        let expected = match &self.origin {
            SapValueNoteOriginV2::CollateralMint {
                mint_id,
                deposit_id,
            } => {
                validate_id(mint_id)?;
                validate_id(deposit_id)?;
                collateral_note_id(mint_id)
            }
            SapValueNoteOriginV2::TransferOutput { transfer_id, role } => {
                validate_id(transfer_id)?;
                transfer_output_note_id(transfer_id, *role)
            }
        };
        if self.note_id != expected {
            return Err(SapValueNoteError::NoteIdMismatch);
        }
        Ok(())
    }
}

/// Assurance is deliberately not part of transfer economic identity. A later
/// witness/notary proof upgrades confidence in the same spend rather than creating
/// another spend. Holochain integration must authenticate witnessed evidence.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SapSpendAssuranceV2 {
    ForkDetectionOnly,
    Witnessed {
        policy_id: String,
        policy_version: u16,
        evidence_action_reference: String,
    },
}

impl SapSpendAssuranceV2 {
    pub fn validate_shape(&self) -> Result<(), SapValueNoteError> {
        match self {
            Self::ForkDetectionOnly => Ok(()),
            Self::Witnessed {
                policy_id,
                policy_version,
                evidence_action_reference,
            } => {
                validate_id(policy_id)?;
                if *policy_version == 0 {
                    return Err(SapValueNoteError::InvalidAssurancePolicyVersion);
                }
                validate_action_reference(evidence_action_reference)
            }
        }
    }
}

/// Canonical storage-independent transfer plan.
///
/// Input order is normalized by note ID. The transfer consumes the entire value of
/// every input note and deterministically creates one recipient output plus optional
/// sender change. No other economic effect is mixed into the plan.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferV2 {
    pub schema_version: u16,
    pub transfer_id: String,
    pub sender_did: String,
    pub recipient_did: String,
    pub input_note_ids: Vec<String>,
    pub input_total: u64,
    pub transfer_amount: u64,
    pub change_amount: u64,
    pub recipient_note: SapValueNoteV2,
    pub change_note: Option<SapValueNoteV2>,
}

impl SapTransferV2 {
    pub fn derive(
        sender_did: String,
        recipient_did: String,
        inputs: &[SapValueNoteV2],
        transfer_amount: u64,
    ) -> Result<Self, SapValueNoteError> {
        validate_did(&sender_did)?;
        validate_did(&recipient_did)?;
        if sender_did == recipient_did {
            return Err(SapValueNoteError::SelfTransfer);
        }
        if transfer_amount == 0 {
            return Err(SapValueNoteError::ZeroAmount);
        }
        if inputs.is_empty() {
            return Err(SapValueNoteError::NoInputs);
        }
        if inputs.len() > MAX_TRANSFER_INPUTS {
            return Err(SapValueNoteError::TooManyInputs);
        }

        let mut input_by_id: BTreeMap<String, &SapValueNoteV2> = BTreeMap::new();
        let mut input_total = 0u64;
        for input in inputs {
            input.validate_shape()?;
            if input.owner_did != sender_did {
                return Err(SapValueNoteError::InputOwnerMismatch);
            }
            if input_by_id.insert(input.note_id.clone(), input).is_some() {
                return Err(SapValueNoteError::DuplicateInputNote);
            }
            input_total = input_total
                .checked_add(input.amount)
                .ok_or(SapValueNoteError::AmountOverflow)?;
        }
        if transfer_amount > input_total {
            return Err(SapValueNoteError::InsufficientInputValue);
        }

        let input_note_ids: Vec<String> = input_by_id.keys().cloned().collect();
        let change_amount = input_total - transfer_amount;
        let transfer_id = canonical_transfer_id(
            &sender_did,
            &recipient_did,
            &input_note_ids,
            transfer_amount,
        )?;
        let recipient_note = SapValueNoteV2::transfer_output(
            &transfer_id,
            SapTransferOutputRoleV2::Recipient,
            recipient_did.clone(),
            transfer_amount,
        )?;
        let change_note = if change_amount == 0 {
            None
        } else {
            Some(SapValueNoteV2::transfer_output(
                &transfer_id,
                SapTransferOutputRoleV2::Change,
                sender_did.clone(),
                change_amount,
            )?)
        };

        let plan = Self {
            schema_version: SAP_TRANSFER_V2_SCHEMA_VERSION,
            transfer_id,
            sender_did,
            recipient_did,
            input_note_ids,
            input_total,
            transfer_amount,
            change_amount,
            recipient_note,
            change_note,
        };
        plan.validate_against_inputs(inputs)?;
        Ok(plan)
    }

    pub fn validate_shape(&self) -> Result<(), SapValueNoteError> {
        if self.schema_version != SAP_TRANSFER_V2_SCHEMA_VERSION {
            return Err(SapValueNoteError::UnsupportedTransferSchema);
        }
        validate_id(&self.transfer_id)?;
        validate_did(&self.sender_did)?;
        validate_did(&self.recipient_did)?;
        if self.sender_did == self.recipient_did {
            return Err(SapValueNoteError::SelfTransfer);
        }
        if self.input_note_ids.is_empty() {
            return Err(SapValueNoteError::NoInputs);
        }
        if self.input_note_ids.len() > MAX_TRANSFER_INPUTS {
            return Err(SapValueNoteError::TooManyInputs);
        }
        if self.transfer_amount == 0 {
            return Err(SapValueNoteError::ZeroAmount);
        }

        let mut previous: Option<&str> = None;
        for note_id in &self.input_note_ids {
            validate_id(note_id)?;
            if let Some(prev) = previous {
                if prev >= note_id.as_str() {
                    return Err(SapValueNoteError::NonCanonicalInputOrder);
                }
            }
            previous = Some(note_id);
        }

        let expected_transfer_id = canonical_transfer_id(
            &self.sender_did,
            &self.recipient_did,
            &self.input_note_ids,
            self.transfer_amount,
        )?;
        if self.transfer_id != expected_transfer_id {
            return Err(SapValueNoteError::TransferIdMismatch);
        }
        if self.transfer_amount > self.input_total {
            return Err(SapValueNoteError::InsufficientInputValue);
        }
        if self.change_amount != self.input_total - self.transfer_amount {
            return Err(SapValueNoteError::ChangeAmountMismatch);
        }

        self.recipient_note.validate_shape()?;
        if self.recipient_note.owner_did != self.recipient_did
            || self.recipient_note.amount != self.transfer_amount
            || self.recipient_note.origin
                != (SapValueNoteOriginV2::TransferOutput {
                    transfer_id: self.transfer_id.clone(),
                    role: SapTransferOutputRoleV2::Recipient,
                })
        {
            return Err(SapValueNoteError::RecipientOutputMismatch);
        }

        match (&self.change_note, self.change_amount) {
            (None, 0) => {}
            (Some(change), amount) if amount > 0 => {
                change.validate_shape()?;
                if change.owner_did != self.sender_did
                    || change.amount != amount
                    || change.origin
                        != (SapValueNoteOriginV2::TransferOutput {
                            transfer_id: self.transfer_id.clone(),
                            role: SapTransferOutputRoleV2::Change,
                        })
                {
                    return Err(SapValueNoteError::ChangeOutputMismatch);
                }
            }
            _ => return Err(SapValueNoteError::ChangeOutputMismatch),
        }
        Ok(())
    }

    /// Re-prove the plan against exact authenticated input-note payloads.
    /// Holochain integration should call this after loading every note origin.
    pub fn validate_against_inputs(
        &self,
        inputs: &[SapValueNoteV2],
    ) -> Result<(), SapValueNoteError> {
        self.validate_shape()?;
        if inputs.len() != self.input_note_ids.len() {
            return Err(SapValueNoteError::InputSetMismatch);
        }
        let mut by_id: BTreeMap<&str, &SapValueNoteV2> = BTreeMap::new();
        let mut total = 0u64;
        for input in inputs {
            input.validate_shape()?;
            if input.owner_did != self.sender_did {
                return Err(SapValueNoteError::InputOwnerMismatch);
            }
            if by_id.insert(input.note_id.as_str(), input).is_some() {
                return Err(SapValueNoteError::DuplicateInputNote);
            }
            total = total
                .checked_add(input.amount)
                .ok_or(SapValueNoteError::AmountOverflow)?;
        }
        let actual_ids: Vec<&str> = by_id.keys().copied().collect();
        let expected_ids: Vec<&str> = self.input_note_ids.iter().map(String::as_str).collect();
        if actual_ids != expected_ids || total != self.input_total {
            return Err(SapValueNoteError::InputSetMismatch);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferSpendObservationV2 {
    pub action_reference: String,
    pub action_author_did: String,
    pub transfer: SapTransferV2,
    pub assurance: SapSpendAssuranceV2,
}

impl SapTransferSpendObservationV2 {
    pub fn validate_shape(&self) -> Result<(), SapValueNoteError> {
        validate_action_reference(&self.action_reference)?;
        validate_did(&self.action_author_did)?;
        self.transfer.validate_shape()?;
        self.assurance.validate_shape()?;
        if self.action_author_did != self.transfer.sender_did {
            return Err(SapValueNoteError::SpendAuthorMismatch);
        }
        Ok(())
    }

    pub fn validate_against_inputs(
        &self,
        inputs: &[SapValueNoteV2],
    ) -> Result<(), SapValueNoteError> {
        self.validate_shape()?;
        self.transfer.validate_against_inputs(inputs)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapNoteConsumptionStateV2 {
    Unspent {
        note_id: String,
    },
    Spent {
        note_id: String,
        transfer_id: String,
        physical_action_references: Vec<String>,
        assurance_observations: Vec<SapSpendAssuranceV2>,
    },
    Conflict {
        note_id: String,
        transfer_ids: Vec<String>,
        physical_action_references: Vec<String>,
    },
}

impl SapNoteConsumptionStateV2 {
    pub fn note_id(&self) -> &str {
        match self {
            Self::Unspent { note_id }
            | Self::Spent { note_id, .. }
            | Self::Conflict { note_id, .. } => note_id,
        }
    }

    pub fn is_conflicted(&self) -> bool {
        matches!(self, Self::Conflict { .. })
    }

    pub fn validate_shape(&self) -> Result<(), SapValueNoteError> {
        validate_id(self.note_id())?;
        match self {
            Self::Unspent { .. } => Ok(()),
            Self::Spent {
                transfer_id,
                physical_action_references,
                assurance_observations,
                ..
            } => {
                validate_id(transfer_id)?;
                if physical_action_references.is_empty() {
                    return Err(SapValueNoteError::MalformedConsumptionState);
                }
                validate_sorted_unique_action_refs(physical_action_references)?;
                if assurance_observations.is_empty() {
                    return Err(SapValueNoteError::MalformedConsumptionState);
                }
                for assurance in assurance_observations {
                    assurance.validate_shape()?;
                }
                Ok(())
            }
            Self::Conflict {
                transfer_ids,
                physical_action_references,
                ..
            } => {
                if transfer_ids.len() < 2 || physical_action_references.len() < 2 {
                    return Err(SapValueNoteError::MalformedConsumptionState);
                }
                validate_sorted_unique_ids(transfer_ids)?;
                validate_sorted_unique_action_refs(physical_action_references)
            }
        }
    }
}

#[derive(Default)]
struct TransferObservationAggregate {
    transfer: Option<SapTransferV2>,
    action_references: BTreeSet<String>,
    assurances: BTreeSet<SapSpendAssuranceV2>,
}

/// Classify all already-authenticated spend observations for one exact note.
///
/// Repeated observation of one physical action is de-duplicated. Distinct physical
/// actions with one identical canonical transfer are one economic spend. If the
/// same transfer ID carries different transfer facts, classification fails closed.
/// Distinct transfer IDs consuming the note are a double-spend conflict.
pub fn classify_note_consumption(
    note_id: &str,
    observations: &[SapTransferSpendObservationV2],
) -> Result<SapNoteConsumptionStateV2, SapValueNoteError> {
    validate_id(note_id)?;
    let mut physical: BTreeMap<String, SapTransferSpendObservationV2> = BTreeMap::new();
    for observation in observations {
        observation.validate_shape()?;
        if !observation
            .transfer
            .input_note_ids
            .iter()
            .any(|input| input == note_id)
        {
            return Err(SapValueNoteError::ObservationDoesNotConsumeNote);
        }
        match physical.get(&observation.action_reference) {
            Some(existing) if existing == observation => {}
            Some(_) => return Err(SapValueNoteError::InconsistentDuplicateSpendAction),
            None => {
                physical.insert(observation.action_reference.clone(), observation.clone());
            }
        }
    }

    if physical.is_empty() {
        return Ok(SapNoteConsumptionStateV2::Unspent {
            note_id: note_id.to_string(),
        });
    }

    let mut by_transfer: BTreeMap<String, TransferObservationAggregate> = BTreeMap::new();
    for observation in physical.values() {
        let aggregate = by_transfer
            .entry(observation.transfer.transfer_id.clone())
            .or_default();
        match &aggregate.transfer {
            Some(existing) if existing != &observation.transfer => {
                return Err(SapValueNoteError::ConflictingTransferFacts);
            }
            Some(_) => {}
            None => aggregate.transfer = Some(observation.transfer.clone()),
        }
        aggregate
            .action_references
            .insert(observation.action_reference.clone());
        aggregate.assurances.insert(observation.assurance.clone());
    }

    if by_transfer.len() == 1 {
        let mut iter = by_transfer.into_iter();
        if let Some((transfer_id, aggregate)) = iter.next() {
            let state = SapNoteConsumptionStateV2::Spent {
                note_id: note_id.to_string(),
                transfer_id,
                physical_action_references: aggregate.action_references.into_iter().collect(),
                assurance_observations: aggregate.assurances.into_iter().collect(),
            };
            state.validate_shape()?;
            return Ok(state);
        }
        return Err(SapValueNoteError::InternalClassificationInvariant);
    }

    let state = SapNoteConsumptionStateV2::Conflict {
        note_id: note_id.to_string(),
        transfer_ids: by_transfer.keys().cloned().collect(),
        physical_action_references: physical.keys().cloned().collect(),
    };
    state.validate_shape()?;
    Ok(state)
}

/// This says only what the supplied observation set establishes. It is not a
/// network-finality or proactive double-spend-prevention claim.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapTransferOutputAvailabilityV2 {
    ConflictFreeObserved,
    FrozenConflict { input_note_ids: Vec<String> },
    Indeterminate { input_note_ids: Vec<String> },
}

pub fn classify_transfer_output_availability(
    transfer: &SapTransferV2,
    input_states: &BTreeMap<String, SapNoteConsumptionStateV2>,
) -> Result<SapTransferOutputAvailabilityV2, SapValueNoteError> {
    transfer.validate_shape()?;
    let mut conflicts = Vec::new();
    let mut indeterminate = Vec::new();

    for note_id in &transfer.input_note_ids {
        let Some(state) = input_states.get(note_id) else {
            indeterminate.push(note_id.clone());
            continue;
        };
        state.validate_shape()?;
        if state.note_id() != note_id {
            return Err(SapValueNoteError::ConsumptionStateNoteMismatch);
        }
        match state {
            SapNoteConsumptionStateV2::Unspent { .. } => indeterminate.push(note_id.clone()),
            SapNoteConsumptionStateV2::Spent { transfer_id, .. }
                if transfer_id == &transfer.transfer_id => {}
            SapNoteConsumptionStateV2::Spent { .. }
            | SapNoteConsumptionStateV2::Conflict { .. } => conflicts.push(note_id.clone()),
        }
    }

    if !conflicts.is_empty() {
        return Ok(SapTransferOutputAvailabilityV2::FrozenConflict {
            input_note_ids: conflicts,
        });
    }
    if !indeterminate.is_empty() {
        return Ok(SapTransferOutputAvailabilityV2::Indeterminate {
            input_note_ids: indeterminate,
        });
    }
    Ok(SapTransferOutputAvailabilityV2::ConflictFreeObserved)
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferClaimV2 {
    pub schema_version: u16,
    pub member_did: String,
    pub output_note_id: String,
}

impl SapTransferClaimV2 {
    pub fn new(member_did: String, output_note_id: String) -> Result<Self, SapValueNoteError> {
        let claim = Self {
            schema_version: SAP_TRANSFER_CLAIM_V2_SCHEMA_VERSION,
            member_did,
            output_note_id,
        };
        claim.validate_shape()?;
        Ok(claim)
    }

    pub fn validate_shape(&self) -> Result<(), SapValueNoteError> {
        if self.schema_version != SAP_TRANSFER_CLAIM_V2_SCHEMA_VERSION {
            return Err(SapValueNoteError::UnsupportedTransferClaimSchema);
        }
        validate_did(&self.member_did)?;
        validate_id(&self.output_note_id)
    }
}

/// A claim object whose semantics are revalidated by every projection call.
/// Fields are private so ordinary callers cannot accidentally construct one by
/// bypassing `from_valid_output`; serde round-trips remain safe because consumers
/// still re-run `validate_shape`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidatedTransferClaimV2 {
    claim_action_reference: String,
    action_author_did: String,
    claim: SapTransferClaimV2,
    output_note: SapValueNoteV2,
}

impl ValidatedTransferClaimV2 {
    pub fn from_valid_output(
        claim_action_reference: String,
        action_author_did: String,
        claim: SapTransferClaimV2,
        output_note: SapValueNoteV2,
    ) -> Result<Self, SapValueNoteError> {
        let validated = Self {
            claim_action_reference,
            action_author_did,
            claim,
            output_note,
        };
        validated.validate_shape()?;
        Ok(validated)
    }

    pub fn claim_action_reference(&self) -> &str {
        &self.claim_action_reference
    }

    pub fn action_author_did(&self) -> &str {
        &self.action_author_did
    }

    pub fn claim(&self) -> &SapTransferClaimV2 {
        &self.claim
    }

    pub fn output_note(&self) -> &SapValueNoteV2 {
        &self.output_note
    }

    pub fn validate_shape(&self) -> Result<(), SapValueNoteError> {
        validate_action_reference(&self.claim_action_reference)?;
        validate_did(&self.action_author_did)?;
        self.claim.validate_shape()?;
        self.output_note.validate_shape()?;
        if self.action_author_did != self.claim.member_did {
            return Err(SapValueNoteError::ClaimAuthorMismatch);
        }
        if self.output_note.owner_did != self.claim.member_did
            || self.output_note.note_id != self.claim.output_note_id
        {
            return Err(SapValueNoteError::ClaimOutputMismatch);
        }
        match &self.output_note.origin {
            SapValueNoteOriginV2::TransferOutput {
                role: SapTransferOutputRoleV2::Recipient,
                ..
            } => Ok(()),
            _ => Err(SapValueNoteError::ClaimOutputMismatch),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferClaimProjectionV2 {
    pub member_did: String,
    pub physical_claim_action_count: u64,
    pub unique_output_note_count: u64,
    pub duplicate_claim_action_count: u64,
    pub claimed_value: u64,
}

/// Idempotent recipient-claim projection keyed by deterministic output note ID.
/// The caller separately decides whether the source transfer output has sufficient
/// conflict/finality assurance to be spendable.
pub fn project_transfer_claims(
    member_did: &str,
    observations: &[ValidatedTransferClaimV2],
) -> Result<SapTransferClaimProjectionV2, SapValueNoteError> {
    validate_did(member_did)?;
    let mut physical: BTreeMap<String, ValidatedTransferClaimV2> = BTreeMap::new();
    for observation in observations {
        observation.validate_shape()?;
        if observation.action_author_did != member_did
            || observation.claim.member_did != member_did
            || observation.output_note.owner_did != member_did
        {
            return Err(SapValueNoteError::ForeignClaimObservation);
        }
        match physical.get(&observation.claim_action_reference) {
            Some(existing) if existing == observation => {}
            Some(_) => return Err(SapValueNoteError::InconsistentDuplicateClaimAction),
            None => {
                physical.insert(
                    observation.claim_action_reference.clone(),
                    observation.clone(),
                );
            }
        }
    }

    let mut by_note: BTreeMap<String, SapValueNoteV2> = BTreeMap::new();
    for observation in physical.values() {
        match by_note.get(&observation.output_note.note_id) {
            Some(existing) if existing == &observation.output_note => {}
            Some(_) => return Err(SapValueNoteError::ConflictingOutputNoteIdentity),
            None => {
                by_note.insert(
                    observation.output_note.note_id.clone(),
                    observation.output_note.clone(),
                );
            }
        }
    }

    let mut claimed_value = 0u64;
    for note in by_note.values() {
        claimed_value = claimed_value
            .checked_add(note.amount)
            .ok_or(SapValueNoteError::AmountOverflow)?;
    }
    let physical_claim_action_count =
        u64::try_from(physical.len()).map_err(|_| SapValueNoteError::CountOverflow)?;
    let unique_output_note_count =
        u64::try_from(by_note.len()).map_err(|_| SapValueNoteError::CountOverflow)?;
    let duplicate_claim_action_count = physical_claim_action_count
        .checked_sub(unique_output_note_count)
        .ok_or(SapValueNoteError::CountUnderflow)?;

    Ok(SapTransferClaimProjectionV2 {
        member_did: member_did.to_string(),
        physical_claim_action_count,
        unique_output_note_count,
        duplicate_claim_action_count,
        claimed_value,
    })
}

fn canonical_transfer_id(
    sender_did: &str,
    recipient_did: &str,
    input_note_ids: &[String],
    transfer_amount: u64,
) -> Result<String, SapValueNoteError> {
    validate_did(sender_did)?;
    validate_did(recipient_did)?;
    if input_note_ids.is_empty() {
        return Err(SapValueNoteError::NoInputs);
    }
    if input_note_ids.len() > MAX_TRANSFER_INPUTS {
        return Err(SapValueNoteError::TooManyInputs);
    }
    let mut hasher = blake3::Hasher::new();
    hasher.update(TRANSFER_ID_DOMAIN);
    hasher.update(&SAP_TRANSFER_V2_SCHEMA_VERSION.to_be_bytes());
    hash_field(&mut hasher, sender_did.as_bytes());
    hash_field(&mut hasher, recipient_did.as_bytes());
    hasher.update(&transfer_amount.to_be_bytes());
    hasher.update(&(input_note_ids.len() as u64).to_be_bytes());
    for note_id in input_note_ids {
        validate_id(note_id)?;
        hash_field(&mut hasher, note_id.as_bytes());
    }
    Ok(format!("sap-transfer:v2:{}", hasher.finalize().to_hex()))
}

fn collateral_note_id(mint_id: &str) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(COLLATERAL_NOTE_DOMAIN);
    hash_field(&mut hasher, mint_id.as_bytes());
    format!("sap-note:v2:{}", hasher.finalize().to_hex())
}

fn transfer_output_note_id(transfer_id: &str, role: SapTransferOutputRoleV2) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(TRANSFER_OUTPUT_DOMAIN);
    hash_field(&mut hasher, transfer_id.as_bytes());
    hasher.update(&[match role {
        SapTransferOutputRoleV2::Recipient => 0,
        SapTransferOutputRoleV2::Change => 1,
    }]);
    format!("sap-note:v2:{}", hasher.finalize().to_hex())
}

fn hash_field(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn validate_sorted_unique_ids(values: &[String]) -> Result<(), SapValueNoteError> {
    let mut previous: Option<&str> = None;
    for value in values {
        validate_id(value)?;
        if let Some(prev) = previous {
            if prev >= value.as_str() {
                return Err(SapValueNoteError::MalformedConsumptionState);
            }
        }
        previous = Some(value);
    }
    Ok(())
}

fn validate_sorted_unique_action_refs(values: &[String]) -> Result<(), SapValueNoteError> {
    let mut previous: Option<&str> = None;
    for value in values {
        validate_action_reference(value)?;
        if let Some(prev) = previous {
            if prev >= value.as_str() {
                return Err(SapValueNoteError::MalformedConsumptionState);
            }
        }
        previous = Some(value);
    }
    Ok(())
}

fn validate_did(value: &str) -> Result<(), SapValueNoteError> {
    if !value.starts_with("did:") || value.len() > MAX_DID_LEN {
        return Err(SapValueNoteError::InvalidDid);
    }
    Ok(())
}

fn validate_id(value: &str) -> Result<(), SapValueNoteError> {
    if value.is_empty() || value.len() > MAX_ID_LEN {
        return Err(SapValueNoteError::InvalidId);
    }
    Ok(())
}

fn validate_action_reference(value: &str) -> Result<(), SapValueNoteError> {
    if value.is_empty() || value.len() > MAX_ACTION_REFERENCE_LEN {
        return Err(SapValueNoteError::InvalidActionReference);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapValueNoteError {
    UnsupportedNoteSchema,
    UnsupportedTransferSchema,
    UnsupportedTransferClaimSchema,
    InvalidCollateralClaim,
    InvalidDid,
    InvalidId,
    InvalidActionReference,
    InvalidAssurancePolicyVersion,
    ZeroAmount,
    NoteIdMismatch,
    SelfTransfer,
    NoInputs,
    TooManyInputs,
    DuplicateInputNote,
    InputOwnerMismatch,
    AmountOverflow,
    InsufficientInputValue,
    NonCanonicalInputOrder,
    TransferIdMismatch,
    ChangeAmountMismatch,
    RecipientOutputMismatch,
    ChangeOutputMismatch,
    InputSetMismatch,
    SpendAuthorMismatch,
    ObservationDoesNotConsumeNote,
    InconsistentDuplicateSpendAction,
    ConflictingTransferFacts,
    MalformedConsumptionState,
    ConsumptionStateNoteMismatch,
    InternalClassificationInvariant,
    ClaimAuthorMismatch,
    ClaimOutputMismatch,
    ForeignClaimObservation,
    InconsistentDuplicateClaimAction,
    ConflictingOutputNoteIdentity,
    CountOverflow,
    CountUnderflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collateral_claim(
        action: &str,
        receipt: &str,
        mint: &str,
        deposit: &str,
        amount: u64,
    ) -> ValidatedCollateralClaimV2 {
        ValidatedCollateralClaimV2 {
            claim_action_reference: action.into(),
            action_author_did: "did:mycelix:alice".into(),
            member_did: "did:mycelix:alice".into(),
            issuance_receipt_action_reference: receipt.into(),
            mint_id: mint.into(),
            deposit_id: deposit.into(),
            amount,
        }
    }

    fn note(mint: &str, amount: u64) -> SapValueNoteV2 {
        SapValueNoteV2::from_collateral_claim(&collateral_claim(
            &format!("uhCkk-claim-{mint}"),
            &format!("uhCkk-receipt-{mint}"),
            mint,
            &format!("deposit:{mint}"),
            amount,
        ))
        .unwrap()
    }

    fn observation(action: &str, transfer: SapTransferV2) -> SapTransferSpendObservationV2 {
        SapTransferSpendObservationV2 {
            action_reference: action.into(),
            action_author_did: transfer.sender_did.clone(),
            transfer,
            assurance: SapSpendAssuranceV2::ForkDetectionOnly,
        }
    }

    #[test]
    fn duplicate_collateral_claims_map_to_same_note_identity() {
        let a = SapValueNoteV2::from_collateral_claim(&collateral_claim(
            "uhCkk-claim-a",
            "uhCkk-receipt-a",
            "mint:one",
            "deposit:one",
            50,
        ))
        .unwrap();
        let b = SapValueNoteV2::from_collateral_claim(&collateral_claim(
            "uhCkk-claim-b",
            "uhCkk-receipt-b",
            "mint:one",
            "deposit:one",
            50,
        ))
        .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn input_order_does_not_change_transfer_identity() {
        let a = note("a", 40);
        let b = note("b", 60);
        let first = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[a.clone(), b.clone()],
            75,
        )
        .unwrap();
        let second = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[b, a],
            75,
        )
        .unwrap();
        assert_eq!(first, second);
        assert_eq!(first.input_total, 100);
        assert_eq!(first.change_amount, 25);
    }

    #[test]
    fn input_sum_overflow_fails_closed() {
        let a = note("a", u64::MAX);
        let b = note("b", 1);
        assert_eq!(
            SapTransferV2::derive(
                "did:mycelix:alice".into(),
                "did:mycelix:bob".into(),
                &[a, b],
                1,
            ),
            Err(SapValueNoteError::AmountOverflow)
        );
    }

    #[test]
    fn duplicate_input_note_is_rejected() {
        let a = note("a", 40);
        assert_eq!(
            SapTransferV2::derive(
                "did:mycelix:alice".into(),
                "did:mycelix:bob".into(),
                &[a.clone(), a],
                20,
            ),
            Err(SapValueNoteError::DuplicateInputNote)
        );
    }

    #[test]
    fn foreign_owned_input_is_rejected() {
        let mut a = note("a", 40);
        a.owner_did = "did:mycelix:mallory".into();
        assert_eq!(
            SapTransferV2::derive(
                "did:mycelix:alice".into(),
                "did:mycelix:bob".into(),
                &[a],
                20,
            ),
            Err(SapValueNoteError::InputOwnerMismatch)
        );
    }

    #[test]
    fn transfer_cannot_create_value() {
        let a = note("a", 40);
        assert_eq!(
            SapTransferV2::derive(
                "did:mycelix:alice".into(),
                "did:mycelix:bob".into(),
                &[a],
                41,
            ),
            Err(SapValueNoteError::InsufficientInputValue)
        );
    }

    #[test]
    fn exact_spend_duplicates_are_one_economic_consumption() {
        let input = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input.clone()],
            20,
        )
        .unwrap();
        let state = classify_note_consumption(
            &input.note_id,
            &[
                observation("uhCkk-spend-a", transfer.clone()),
                observation("uhCkk-spend-b", transfer.clone()),
            ],
        )
        .unwrap();
        match state {
            SapNoteConsumptionStateV2::Spent {
                transfer_id,
                physical_action_references,
                ..
            } => {
                assert_eq!(transfer_id, transfer.transfer_id);
                assert_eq!(physical_action_references.len(), 2);
            }
            other => panic!("expected one economic spend, got {other:?}"),
        }
    }

    #[test]
    fn same_transfer_id_with_conflicting_facts_fails_closed() {
        let input = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input.clone()],
            20,
        )
        .unwrap();
        let mut conflicting = transfer.clone();
        conflicting.input_total = 41;
        conflicting.change_amount = 21;
        let mut change = conflicting.change_note.clone().unwrap();
        change.amount = 21;
        conflicting.change_note = Some(change);
        assert_eq!(conflicting.validate_shape(), Ok(()));
        assert_eq!(
            classify_note_consumption(
                &input.note_id,
                &[
                    observation("uhCkk-spend-a", transfer),
                    observation("uhCkk-spend-b", conflicting),
                ],
            ),
            Err(SapValueNoteError::ConflictingTransferFacts)
        );
    }

    #[test]
    fn distinct_spends_of_one_note_are_conflict() {
        let input = note("a", 40);
        let to_bob = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input.clone()],
            20,
        )
        .unwrap();
        let to_carol = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:carol".into(),
            &[input.clone()],
            20,
        )
        .unwrap();
        let state = classify_note_consumption(
            &input.note_id,
            &[
                observation("uhCkk-spend-a", to_bob),
                observation("uhCkk-spend-b", to_carol),
            ],
        )
        .unwrap();
        assert!(state.is_conflicted());
    }

    #[test]
    fn conflict_free_outputs_require_every_input_to_select_same_transfer() {
        let a = note("a", 40);
        let b = note("b", 60);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[a.clone(), b.clone()],
            75,
        )
        .unwrap();
        let observations = [observation("uhCkk-spend", transfer.clone())];
        let mut states = BTreeMap::new();
        states.insert(
            a.note_id.clone(),
            classify_note_consumption(&a.note_id, &observations).unwrap(),
        );
        states.insert(
            b.note_id.clone(),
            classify_note_consumption(&b.note_id, &observations).unwrap(),
        );
        assert_eq!(
            classify_transfer_output_availability(&transfer, &states),
            Ok(SapTransferOutputAvailabilityV2::ConflictFreeObserved)
        );
    }

    #[test]
    fn mismatched_consumption_state_note_identity_is_rejected() {
        let input = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input.clone()],
            20,
        )
        .unwrap();
        let mut states = BTreeMap::new();
        states.insert(
            input.note_id.clone(),
            SapNoteConsumptionStateV2::Spent {
                note_id: "sap-note:v2:not-the-map-key".into(),
                transfer_id: transfer.transfer_id.clone(),
                physical_action_references: vec!["uhCkk-spend".into()],
                assurance_observations: vec![SapSpendAssuranceV2::ForkDetectionOnly],
            },
        );
        assert_eq!(
            classify_transfer_output_availability(&transfer, &states),
            Err(SapValueNoteError::ConsumptionStateNoteMismatch)
        );
    }

    #[test]
    fn any_input_conflict_freezes_transfer_outputs() {
        let a = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[a.clone()],
            20,
        )
        .unwrap();
        let conflicting = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:carol".into(),
            &[a.clone()],
            20,
        )
        .unwrap();
        let observations = [
            observation("uhCkk-spend-a", transfer.clone()),
            observation("uhCkk-spend-b", conflicting),
        ];
        let mut states = BTreeMap::new();
        states.insert(
            a.note_id.clone(),
            classify_note_consumption(&a.note_id, &observations).unwrap(),
        );
        assert!(matches!(
            classify_transfer_output_availability(&transfer, &states).unwrap(),
            SapTransferOutputAvailabilityV2::FrozenConflict { .. }
        ));
    }

    #[test]
    fn witnessed_assurance_does_not_change_transfer_identity() {
        let input = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input.clone()],
            20,
        )
        .unwrap();
        let fork = observation("uhCkk-spend-a", transfer.clone());
        let witnessed = SapTransferSpendObservationV2 {
            action_reference: "uhCkk-spend-b".into(),
            action_author_did: "did:mycelix:alice".into(),
            transfer: transfer.clone(),
            assurance: SapSpendAssuranceV2::Witnessed {
                policy_id: "policy:high-value".into(),
                policy_version: 1,
                evidence_action_reference: "uhCkk-witness".into(),
            },
        };
        let state = classify_note_consumption(&input.note_id, &[fork, witnessed]).unwrap();
        match state {
            SapNoteConsumptionStateV2::Spent {
                transfer_id,
                assurance_observations,
                ..
            } => {
                assert_eq!(transfer_id, transfer.transfer_id);
                assert_eq!(assurance_observations.len(), 2);
            }
            other => panic!("expected spent, got {other:?}"),
        }
    }

    #[test]
    fn recipient_claim_is_owner_authored_and_idempotent_by_output_note() {
        let input = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input],
            20,
        )
        .unwrap();
        let output = transfer.recipient_note.clone();
        let claim =
            SapTransferClaimV2::new("did:mycelix:bob".into(), output.note_id.clone()).unwrap();
        let a = ValidatedTransferClaimV2::from_valid_output(
            "uhCkk-claim-a".into(),
            "did:mycelix:bob".into(),
            claim.clone(),
            output.clone(),
        )
        .unwrap();
        let b = ValidatedTransferClaimV2::from_valid_output(
            "uhCkk-claim-b".into(),
            "did:mycelix:bob".into(),
            claim,
            output,
        )
        .unwrap();
        let projection = project_transfer_claims("did:mycelix:bob", &[a, b]).unwrap();
        assert_eq!(projection.physical_claim_action_count, 2);
        assert_eq!(projection.unique_output_note_count, 1);
        assert_eq!(projection.duplicate_claim_action_count, 1);
        assert_eq!(projection.claimed_value, 20);
    }

    #[test]
    fn projection_revalidates_recipient_output_role() {
        let input = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input],
            20,
        )
        .unwrap();
        let change = transfer.change_note.unwrap();
        let forged = ValidatedTransferClaimV2 {
            claim_action_reference: "uhCkk-claim".into(),
            action_author_did: "did:mycelix:alice".into(),
            claim: SapTransferClaimV2::new("did:mycelix:alice".into(), change.note_id.clone())
                .unwrap(),
            output_note: change,
        };
        assert_eq!(
            project_transfer_claims("did:mycelix:alice", &[forged]),
            Err(SapValueNoteError::ClaimOutputMismatch)
        );
    }

    #[test]
    fn transfer_round_trips() {
        let input = note("a", 40);
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input],
            20,
        )
        .unwrap();
        let bytes = serde_json::to_vec(&transfer).unwrap();
        let decoded: SapTransferV2 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, transfer);
    }
}

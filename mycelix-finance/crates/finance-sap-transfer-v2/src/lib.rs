#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-016 pure persistence boundary for the first SAP V2 transfer adapter.
//!
//! The first authoritative Holochain lane spends only exact owner-authored
//! collateral-claim actions from FIN-SAFE-014. Persisted spend records carry a
//! canonical transfer plus exact claim references; note amount/owner/origin facts
//! must be reconstructed from those claims before validation succeeds.

use finance_sap_account_v2::ValidatedCollateralClaimV2;
use finance_sap_value_notes::{
    MAX_ACTION_REFERENCE_LEN, MAX_TRANSFER_INPUTS, SapSpendAssuranceV2,
    SapTransferSpendObservationV2, SapTransferV2, SapValueNoteV2,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const SAP_TRANSFER_SPEND_RECORD_V2_SCHEMA_VERSION: u16 = 1;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferV2Config {
    pub enabled: bool,
}

impl Default for SapTransferV2Config {
    fn default() -> Self {
        Self { enabled: false }
    }
}

impl SapTransferV2Config {
    pub fn validate(&self) -> Result<(), SapTransferAdapterError> {
        Ok(())
    }

    pub fn require_enabled(&self) -> Result<(), SapTransferAdapterError> {
        self.validate()?;
        if self.enabled {
            Ok(())
        } else {
            Err(SapTransferAdapterError::ProtocolDisabled)
        }
    }
}

/// Canonical binding between one economic input note and the exact owner-authored
/// FIN-SAFE-014 collateral-claim Create action from which that note is derived.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferInputClaimRefV2 {
    pub note_id: String,
    pub claim_action_reference: String,
}

impl SapTransferInputClaimRefV2 {
    pub fn validate_shape(&self) -> Result<(), SapTransferAdapterError> {
        validate_nonempty_bounded(&self.note_id, finance_sap_value_notes::MAX_ID_LEN)
            .map_err(|_| SapTransferAdapterError::InvalidNoteId)?;
        validate_action_reference(&self.claim_action_reference)
    }
}

/// Persisted transfer-spend theorem for the first adapter tranche.
///
/// No authoritative note payloads are persisted here. `input_claims` must be
/// strictly ordered by `note_id`, and every note is re-derived from the exact
/// referenced FIN-SAFE-014 claim before the transfer is accepted.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferSpendRecordV2 {
    pub schema_version: u16,
    pub transfer: SapTransferV2,
    pub input_claims: Vec<SapTransferInputClaimRefV2>,
}

impl SapTransferSpendRecordV2 {
    pub fn from_validated_claims(
        transfer: SapTransferV2,
        claims: &[ValidatedCollateralClaimV2],
    ) -> Result<Self, SapTransferAdapterError> {
        let mut refs = Vec::with_capacity(claims.len());
        for claim in claims {
            claim
                .validate_shape()
                .map_err(|_| SapTransferAdapterError::InvalidCollateralClaim)?;
            if claim.member_did != transfer.sender_did {
                return Err(SapTransferAdapterError::ForeignClaimOwner);
            }
            let note = SapValueNoteV2::from_collateral_claim(claim)
                .map_err(|_| SapTransferAdapterError::InvalidCollateralClaim)?;
            refs.push(SapTransferInputClaimRefV2 {
                note_id: note.note_id,
                claim_action_reference: claim.claim_action_reference.clone(),
            });
        }
        refs.sort_by(|a, b| a.note_id.cmp(&b.note_id));

        let record = Self {
            schema_version: SAP_TRANSFER_SPEND_RECORD_V2_SCHEMA_VERSION,
            transfer,
            input_claims: refs,
        };
        record.validate_against_claims(claims)?;
        Ok(record)
    }

    pub fn validate_shape(&self) -> Result<(), SapTransferAdapterError> {
        if self.schema_version != SAP_TRANSFER_SPEND_RECORD_V2_SCHEMA_VERSION {
            return Err(SapTransferAdapterError::UnsupportedSchemaVersion);
        }
        self.transfer
            .validate_shape()
            .map_err(|_| SapTransferAdapterError::InvalidTransfer)?;
        if self.input_claims.is_empty() {
            return Err(SapTransferAdapterError::NoInputs);
        }
        if self.input_claims.len() > MAX_TRANSFER_INPUTS {
            return Err(SapTransferAdapterError::TooManyInputs);
        }
        if self.input_claims.len() != self.transfer.input_note_ids.len() {
            return Err(SapTransferAdapterError::InputSetMismatch);
        }

        let mut previous_note: Option<&str> = None;
        let mut claim_refs = BTreeSet::new();
        for (index, input) in self.input_claims.iter().enumerate() {
            input.validate_shape()?;
            if let Some(previous) = previous_note {
                if previous >= input.note_id.as_str() {
                    return Err(SapTransferAdapterError::NonCanonicalInputOrder);
                }
            }
            previous_note = Some(&input.note_id);
            if self.transfer.input_note_ids[index] != input.note_id {
                return Err(SapTransferAdapterError::InputSetMismatch);
            }
            if !claim_refs.insert(input.claim_action_reference.as_str()) {
                return Err(SapTransferAdapterError::DuplicateClaimReference);
            }
        }
        Ok(())
    }

    /// Re-prove the persisted record from independently authenticated FIN-SAFE-014
    /// claims. The caller must exact-load the claim actions and their receipts.
    pub fn validate_against_claims(
        &self,
        claims: &[ValidatedCollateralClaimV2],
    ) -> Result<Vec<SapValueNoteV2>, SapTransferAdapterError> {
        self.validate_shape()?;
        if claims.len() != self.input_claims.len() {
            return Err(SapTransferAdapterError::InputSetMismatch);
        }

        let mut claims_by_action: BTreeMap<&str, &ValidatedCollateralClaimV2> = BTreeMap::new();
        for claim in claims {
            claim
                .validate_shape()
                .map_err(|_| SapTransferAdapterError::InvalidCollateralClaim)?;
            if claim.member_did != self.transfer.sender_did {
                return Err(SapTransferAdapterError::ForeignClaimOwner);
            }
            if claims_by_action
                .insert(claim.claim_action_reference.as_str(), claim)
                .is_some()
            {
                return Err(SapTransferAdapterError::DuplicateClaimReference);
            }
        }

        let mut notes = Vec::with_capacity(self.input_claims.len());
        for input in &self.input_claims {
            let claim = claims_by_action
                .get(input.claim_action_reference.as_str())
                .ok_or(SapTransferAdapterError::MissingClaim)?;
            let note = SapValueNoteV2::from_collateral_claim(claim)
                .map_err(|_| SapTransferAdapterError::InvalidCollateralClaim)?;
            if note.note_id != input.note_id {
                return Err(SapTransferAdapterError::ClaimNoteMismatch);
            }
            notes.push(note);
        }

        self.transfer
            .validate_against_inputs(&notes)
            .map_err(|_| SapTransferAdapterError::InvalidTransfer)?;
        Ok(notes)
    }

    /// Convert one exact, already-authenticated owner-authored spend action into a
    /// FIN-SAFE-015 observation. V1 persisted spends are fork-detection mode only.
    pub fn to_fork_detection_observation(
        &self,
        action_reference: String,
        action_author_did: String,
        claims: &[ValidatedCollateralClaimV2],
    ) -> Result<SapTransferSpendObservationV2, SapTransferAdapterError> {
        validate_action_reference(&action_reference)?;
        let notes = self.validate_against_claims(claims)?;
        if action_author_did != self.transfer.sender_did {
            return Err(SapTransferAdapterError::SpendAuthorMismatch);
        }
        let observation = SapTransferSpendObservationV2 {
            action_reference,
            action_author_did,
            transfer: self.transfer.clone(),
            assurance: SapSpendAssuranceV2::ForkDetectionOnly,
        };
        observation
            .validate_against_inputs(&notes)
            .map_err(|_| SapTransferAdapterError::InvalidTransfer)?;
        Ok(observation)
    }
}

fn validate_action_reference(value: &str) -> Result<(), SapTransferAdapterError> {
    validate_nonempty_bounded(value, MAX_ACTION_REFERENCE_LEN)
        .map_err(|_| SapTransferAdapterError::InvalidActionReference)
}

fn validate_nonempty_bounded(value: &str, max: usize) -> Result<(), ()> {
    if value.is_empty() || value.len() > max {
        Err(())
    } else {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapTransferAdapterError {
    ProtocolDisabled,
    UnsupportedSchemaVersion,
    InvalidNoteId,
    InvalidActionReference,
    NoInputs,
    TooManyInputs,
    NonCanonicalInputOrder,
    DuplicateClaimReference,
    InputSetMismatch,
    MissingClaim,
    ForeignClaimOwner,
    InvalidCollateralClaim,
    ClaimNoteMismatch,
    InvalidTransfer,
    SpendAuthorMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_sap_account_v2::ValidatedCollateralClaimV2;
    use finance_sap_value_notes::SapTransferV2;

    fn claim(action: &str, mint: &str, amount: u64) -> ValidatedCollateralClaimV2 {
        ValidatedCollateralClaimV2 {
            claim_action_reference: action.into(),
            action_author_did: "did:mycelix:alice".into(),
            member_did: "did:mycelix:alice".into(),
            issuance_receipt_action_reference: format!("uhCkk-receipt-{mint}"),
            mint_id: mint.into(),
            deposit_id: format!("deposit-{mint}"),
            amount,
        }
    }

    fn fixture() -> (Vec<ValidatedCollateralClaimV2>, SapTransferV2) {
        let claims = vec![
            claim("uhCkk-claim-a", "mint-a", 70),
            claim("uhCkk-claim-b", "mint-b", 50),
        ];
        let notes: Vec<_> = claims
            .iter()
            .map(|claim| SapValueNoteV2::from_collateral_claim(claim).unwrap())
            .collect();
        let transfer = SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &notes,
            100,
        )
        .unwrap();
        (claims, transfer)
    }

    #[test]
    fn disabled_by_default() {
        assert_eq!(
            SapTransferV2Config::default().require_enabled(),
            Err(SapTransferAdapterError::ProtocolDisabled)
        );
    }

    #[test]
    fn record_derives_only_claim_references_not_authoritative_note_payloads() {
        let (claims, transfer) = fixture();
        let record =
            SapTransferSpendRecordV2::from_validated_claims(transfer.clone(), &claims).unwrap();
        assert_eq!(record.transfer, transfer);
        assert_eq!(record.input_claims.len(), 2);
        assert!(record.validate_against_claims(&claims).is_ok());
    }

    #[test]
    fn claim_order_does_not_change_canonical_record() {
        let (claims, transfer) = fixture();
        let mut reversed = claims.clone();
        reversed.reverse();
        let a = SapTransferSpendRecordV2::from_validated_claims(transfer.clone(), &claims).unwrap();
        let b = SapTransferSpendRecordV2::from_validated_claims(transfer, &reversed).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn foreign_claim_owner_is_rejected() {
        let (mut claims, transfer) = fixture();
        claims[0].member_did = "did:mycelix:mallory".into();
        claims[0].action_author_did = "did:mycelix:mallory".into();
        assert_eq!(
            SapTransferSpendRecordV2::from_validated_claims(transfer, &claims),
            Err(SapTransferAdapterError::ForeignClaimOwner)
        );
    }

    #[test]
    fn forged_note_to_claim_mapping_is_rejected() {
        let (claims, transfer) = fixture();
        let mut record =
            SapTransferSpendRecordV2::from_validated_claims(transfer, &claims).unwrap();
        let first_claim = record.input_claims[0].claim_action_reference.clone();
        record.input_claims[0].claim_action_reference =
            record.input_claims[1].claim_action_reference.clone();
        record.input_claims[1].claim_action_reference = first_claim;
        assert_eq!(record.validate_shape(), Ok(()));
        assert_eq!(
            record.validate_against_claims(&claims),
            Err(SapTransferAdapterError::ClaimNoteMismatch)
        );
    }

    #[test]
    fn duplicate_claim_reference_is_rejected() {
        let (claims, transfer) = fixture();
        let mut record =
            SapTransferSpendRecordV2::from_validated_claims(transfer, &claims).unwrap();
        record.input_claims[1].claim_action_reference =
            record.input_claims[0].claim_action_reference.clone();
        assert_eq!(
            record.validate_shape(),
            Err(SapTransferAdapterError::DuplicateClaimReference)
        );
    }

    #[test]
    fn fork_detection_observation_is_author_bound() {
        let (claims, transfer) = fixture();
        let record = SapTransferSpendRecordV2::from_validated_claims(transfer, &claims).unwrap();
        assert_eq!(
            record.to_fork_detection_observation(
                "uhCkk-spend".into(),
                "did:mycelix:mallory".into(),
                &claims,
            ),
            Err(SapTransferAdapterError::SpendAuthorMismatch)
        );
        let observation = record
            .to_fork_detection_observation(
                "uhCkk-spend".into(),
                "did:mycelix:alice".into(),
                &claims,
            )
            .unwrap();
        assert_eq!(
            observation.assurance,
            SapSpendAssuranceV2::ForkDetectionOnly
        );
    }

    #[test]
    fn serde_round_trip_revalidates() {
        let (claims, transfer) = fixture();
        let record = SapTransferSpendRecordV2::from_validated_claims(transfer, &claims).unwrap();
        let json = serde_json::to_string(&record).unwrap();
        let decoded: SapTransferSpendRecordV2 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, record);
        assert!(decoded.validate_against_claims(&claims).is_ok());
    }
}

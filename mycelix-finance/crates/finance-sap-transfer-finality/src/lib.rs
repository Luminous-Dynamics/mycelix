#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-017 pure transfer-output spendability gate.
//!
//! Exact transfer existence and recipient acknowledgement do not imply that the
//! output is safe to re-spend. Fork-detection-only outputs remain provisional.
//! A later authoritative adapter may promote an output to spendable only after
//! independently authenticating witness/notary evidence under the expected policy.

use finance_sap_value_notes::{
    MAX_ACTION_REFERENCE_LEN, MAX_ID_LEN, SapTransferOutputAvailabilityV2, SapTransferOutputRoleV2,
    SapTransferV2, SapValueNoteOriginV2, SapValueNoteV2, ValidatedTransferClaimV2,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const SAP_TRANSFER_FINALITY_V1_SCHEMA_VERSION: u16 = 1;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferWitnessPolicyRefV1 {
    pub policy_id: String,
    pub policy_version: u16,
}

impl SapTransferWitnessPolicyRefV1 {
    pub fn validate(&self) -> Result<(), SapTransferFinalityError> {
        validate_id(&self.policy_id).map_err(|_| SapTransferFinalityError::InvalidPolicyId)?;
        if self.policy_version == 0 {
            return Err(SapTransferFinalityError::InvalidPolicyVersion);
        }
        Ok(())
    }
}

/// A compact projection of witness/notary evidence that has already been
/// authenticated by a higher-trust integration layer.
///
/// Calling `from_authenticated_evidence` is an explicit trust-boundary operation;
/// this pure crate cannot inspect Holochain countersigning sessions or signatures.
/// Consumers MUST re-run `validate_for` after deserialization and MUST NOT create
/// this object from untrusted caller strings.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedWitnessSpendAssuranceV1 {
    schema_version: u16,
    transfer_id: String,
    spend_action_reference: String,
    evidence_action_reference: String,
    policy_id: String,
    policy_version: u16,
}

impl VerifiedWitnessSpendAssuranceV1 {
    pub fn from_authenticated_evidence(
        transfer_id: String,
        spend_action_reference: String,
        evidence_action_reference: String,
        policy: &SapTransferWitnessPolicyRefV1,
    ) -> Result<Self, SapTransferFinalityError> {
        policy.validate()?;
        let assurance = Self {
            schema_version: SAP_TRANSFER_FINALITY_V1_SCHEMA_VERSION,
            transfer_id,
            spend_action_reference,
            evidence_action_reference,
            policy_id: policy.policy_id.clone(),
            policy_version: policy.policy_version,
        };
        assurance.validate_shape()?;
        Ok(assurance)
    }

    pub fn transfer_id(&self) -> &str {
        &self.transfer_id
    }

    pub fn spend_action_reference(&self) -> &str {
        &self.spend_action_reference
    }

    pub fn evidence_action_reference(&self) -> &str {
        &self.evidence_action_reference
    }

    pub fn policy_id(&self) -> &str {
        &self.policy_id
    }

    pub fn policy_version(&self) -> u16 {
        self.policy_version
    }

    pub fn validate_shape(&self) -> Result<(), SapTransferFinalityError> {
        if self.schema_version != SAP_TRANSFER_FINALITY_V1_SCHEMA_VERSION {
            return Err(SapTransferFinalityError::UnsupportedSchemaVersion);
        }
        validate_id(&self.transfer_id).map_err(|_| SapTransferFinalityError::InvalidTransferId)?;
        validate_action_reference(&self.spend_action_reference)?;
        validate_action_reference(&self.evidence_action_reference)?;
        SapTransferWitnessPolicyRefV1 {
            policy_id: self.policy_id.clone(),
            policy_version: self.policy_version,
        }
        .validate()
    }

    pub fn validate_for(
        &self,
        expected_policy: &SapTransferWitnessPolicyRefV1,
        transfer_id: &str,
        spend_action_reference: &str,
    ) -> Result<(), SapTransferFinalityError> {
        self.validate_shape()?;
        expected_policy.validate()?;
        if self.policy_id != expected_policy.policy_id
            || self.policy_version != expected_policy.policy_version
        {
            return Err(SapTransferFinalityError::WitnessPolicyMismatch);
        }
        if self.transfer_id != transfer_id {
            return Err(SapTransferFinalityError::WitnessTransferMismatch);
        }
        if self.spend_action_reference != spend_action_reference {
            return Err(SapTransferFinalityError::WitnessSpendActionMismatch);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapTransferOutputAssuranceStateV1 {
    ProvisionalForkDetection,
    FrozenConflict {
        input_note_ids: Vec<String>,
    },
    Indeterminate {
        input_note_ids: Vec<String>,
    },
    SpendableWitnessed {
        policy_id: String,
        policy_version: u16,
        evidence_action_reference: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferOutputStatusV1 {
    pub transfer_id: String,
    pub spend_action_reference: String,
    pub state: SapTransferOutputAssuranceStateV1,
}

/// Classify one transfer's output assurance state.
///
/// `ConflictFreeObserved` from FIN-SAFE-015 is deliberately mapped only to
/// `ProvisionalForkDetection`. It never becomes spendable without independently
/// authenticated witness/notary assurance.
pub fn classify_transfer_output_status(
    transfer: &SapTransferV2,
    spend_action_reference: &str,
    availability: &SapTransferOutputAvailabilityV2,
    expected_policy: &SapTransferWitnessPolicyRefV1,
    witnessed: Option<&VerifiedWitnessSpendAssuranceV1>,
) -> Result<SapTransferOutputStatusV1, SapTransferFinalityError> {
    transfer
        .validate_shape()
        .map_err(|_| SapTransferFinalityError::InvalidTransfer)?;
    validate_action_reference(spend_action_reference)?;
    expected_policy.validate()?;
    validate_availability(availability, transfer)?;

    let state = if let Some(witnessed) = witnessed {
        witnessed.validate_for(
            expected_policy,
            &transfer.transfer_id,
            spend_action_reference,
        )?;
        SapTransferOutputAssuranceStateV1::SpendableWitnessed {
            policy_id: witnessed.policy_id().to_string(),
            policy_version: witnessed.policy_version(),
            evidence_action_reference: witnessed.evidence_action_reference().to_string(),
        }
    } else {
        match availability {
            SapTransferOutputAvailabilityV2::ConflictFreeObserved => {
                SapTransferOutputAssuranceStateV1::ProvisionalForkDetection
            }
            SapTransferOutputAvailabilityV2::FrozenConflict { input_note_ids } => {
                SapTransferOutputAssuranceStateV1::FrozenConflict {
                    input_note_ids: input_note_ids.clone(),
                }
            }
            SapTransferOutputAvailabilityV2::Indeterminate { input_note_ids } => {
                SapTransferOutputAssuranceStateV1::Indeterminate {
                    input_note_ids: input_note_ids.clone(),
                }
            }
        }
    };

    Ok(SapTransferOutputStatusV1 {
        transfer_id: transfer.transfer_id.clone(),
        spend_action_reference: spend_action_reference.to_string(),
        state,
    })
}

/// Stronger checked wrapper required before a recipient transfer output may be
/// accepted as a spend input by a future transfer adapter.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SpendableTransferClaimV2 {
    claim: ValidatedTransferClaimV2,
    parent_spend_action_reference: String,
    assurance: VerifiedWitnessSpendAssuranceV1,
}

impl SpendableTransferClaimV2 {
    pub fn from_verified_witness(
        claim: ValidatedTransferClaimV2,
        parent_spend_action_reference: String,
        assurance: VerifiedWitnessSpendAssuranceV1,
        expected_policy: &SapTransferWitnessPolicyRefV1,
    ) -> Result<Self, SapTransferFinalityError> {
        let checked = Self {
            claim,
            parent_spend_action_reference,
            assurance,
        };
        checked.validate_for_policy(expected_policy)?;
        Ok(checked)
    }

    pub fn claim(&self) -> &ValidatedTransferClaimV2 {
        &self.claim
    }

    pub fn spendable_note(&self) -> &SapValueNoteV2 {
        self.claim.output_note()
    }

    pub fn parent_spend_action_reference(&self) -> &str {
        &self.parent_spend_action_reference
    }

    pub fn assurance(&self) -> &VerifiedWitnessSpendAssuranceV1 {
        &self.assurance
    }

    pub fn validate_for_policy(
        &self,
        expected_policy: &SapTransferWitnessPolicyRefV1,
    ) -> Result<(), SapTransferFinalityError> {
        self.claim
            .validate_shape()
            .map_err(|_| SapTransferFinalityError::InvalidTransferClaim)?;
        validate_action_reference(&self.parent_spend_action_reference)?;
        let transfer_id = recipient_output_transfer_id(self.claim.output_note())?;
        self.assurance.validate_for(
            expected_policy,
            transfer_id,
            &self.parent_spend_action_reference,
        )
    }
}

fn recipient_output_transfer_id(note: &SapValueNoteV2) -> Result<&str, SapTransferFinalityError> {
    note.validate_shape()
        .map_err(|_| SapTransferFinalityError::InvalidTransferClaim)?;
    match &note.origin {
        SapValueNoteOriginV2::TransferOutput {
            transfer_id,
            role: SapTransferOutputRoleV2::Recipient,
        } => Ok(transfer_id),
        _ => Err(SapTransferFinalityError::ClaimIsNotRecipientOutput),
    }
}

fn validate_availability(
    availability: &SapTransferOutputAvailabilityV2,
    transfer: &SapTransferV2,
) -> Result<(), SapTransferFinalityError> {
    match availability {
        SapTransferOutputAvailabilityV2::ConflictFreeObserved => Ok(()),
        SapTransferOutputAvailabilityV2::FrozenConflict { input_note_ids }
        | SapTransferOutputAvailabilityV2::Indeterminate { input_note_ids } => {
            if input_note_ids.is_empty() {
                return Err(SapTransferFinalityError::MalformedAvailability);
            }
            let allowed: BTreeSet<&str> =
                transfer.input_note_ids.iter().map(String::as_str).collect();
            let mut seen = BTreeSet::new();
            for note_id in input_note_ids {
                validate_id(note_id)
                    .map_err(|_| SapTransferFinalityError::MalformedAvailability)?;
                if !allowed.contains(note_id.as_str()) || !seen.insert(note_id.as_str()) {
                    return Err(SapTransferFinalityError::MalformedAvailability);
                }
            }
            Ok(())
        }
    }
}

fn validate_id(value: &str) -> Result<(), ()> {
    if value.is_empty() || value.len() > MAX_ID_LEN {
        Err(())
    } else {
        Ok(())
    }
}

fn validate_action_reference(value: &str) -> Result<(), SapTransferFinalityError> {
    if value.is_empty() || value.len() > MAX_ACTION_REFERENCE_LEN {
        Err(SapTransferFinalityError::InvalidActionReference)
    } else {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapTransferFinalityError {
    UnsupportedSchemaVersion,
    InvalidPolicyId,
    InvalidPolicyVersion,
    InvalidTransferId,
    InvalidActionReference,
    InvalidTransfer,
    InvalidTransferClaim,
    ClaimIsNotRecipientOutput,
    MalformedAvailability,
    WitnessPolicyMismatch,
    WitnessTransferMismatch,
    WitnessSpendActionMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_sap_account_v2::ValidatedCollateralClaimV2;
    use finance_sap_value_notes::{SapTransferClaimV2, SapValueNoteV2};

    fn policy() -> SapTransferWitnessPolicyRefV1 {
        SapTransferWitnessPolicyRefV1 {
            policy_id: "sap-witness-main".into(),
            policy_version: 1,
        }
    }

    fn transfer() -> SapTransferV2 {
        let claim = ValidatedCollateralClaimV2 {
            claim_action_reference: "uhCkk-collateral-claim".into(),
            action_author_did: "did:mycelix:alice".into(),
            member_did: "did:mycelix:alice".into(),
            issuance_receipt_action_reference: "uhCkk-receipt".into(),
            mint_id: "mint-a".into(),
            deposit_id: "deposit-a".into(),
            amount: 100,
        };
        let input = SapValueNoteV2::from_collateral_claim(&claim).unwrap();
        SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input],
            60,
        )
        .unwrap()
    }

    fn recipient_claim(transfer: &SapTransferV2) -> ValidatedTransferClaimV2 {
        let claim = SapTransferClaimV2::new(
            "did:mycelix:bob".into(),
            transfer.recipient_note.note_id.clone(),
        )
        .unwrap();
        ValidatedTransferClaimV2::from_valid_output(
            "uhCkk-transfer-claim".into(),
            "did:mycelix:bob".into(),
            claim,
            transfer.recipient_note.clone(),
        )
        .unwrap()
    }

    fn witness(transfer: &SapTransferV2) -> VerifiedWitnessSpendAssuranceV1 {
        VerifiedWitnessSpendAssuranceV1::from_authenticated_evidence(
            transfer.transfer_id.clone(),
            "uhCkk-spend".into(),
            "uhCkk-witness".into(),
            &policy(),
        )
        .unwrap()
    }

    #[test]
    fn conflict_free_observation_without_witness_is_only_provisional() {
        let transfer = transfer();
        let status = classify_transfer_output_status(
            &transfer,
            "uhCkk-spend",
            &SapTransferOutputAvailabilityV2::ConflictFreeObserved,
            &policy(),
            None,
        )
        .unwrap();
        assert_eq!(
            status.state,
            SapTransferOutputAssuranceStateV1::ProvisionalForkDetection
        );
    }

    #[test]
    fn conflict_is_frozen_without_stronger_assurance() {
        let transfer = transfer();
        let status = classify_transfer_output_status(
            &transfer,
            "uhCkk-spend",
            &SapTransferOutputAvailabilityV2::FrozenConflict {
                input_note_ids: transfer.input_note_ids.clone(),
            },
            &policy(),
            None,
        )
        .unwrap();
        assert!(matches!(
            status.state,
            SapTransferOutputAssuranceStateV1::FrozenConflict { .. }
        ));
    }

    #[test]
    fn matching_authenticated_witness_can_promote_same_economic_transfer() {
        let transfer = transfer();
        let witness = witness(&transfer);
        let status = classify_transfer_output_status(
            &transfer,
            "uhCkk-spend",
            &SapTransferOutputAvailabilityV2::ConflictFreeObserved,
            &policy(),
            Some(&witness),
        )
        .unwrap();
        assert!(matches!(
            status.state,
            SapTransferOutputAssuranceStateV1::SpendableWitnessed { .. }
        ));
        assert_eq!(status.transfer_id, transfer.transfer_id);
    }

    #[test]
    fn wrong_policy_or_spend_reference_fails_closed() {
        let transfer = transfer();
        let witness = witness(&transfer);
        let wrong_policy = SapTransferWitnessPolicyRefV1 {
            policy_id: "other".into(),
            policy_version: 1,
        };
        assert_eq!(
            witness.validate_for(&wrong_policy, &transfer.transfer_id, "uhCkk-spend"),
            Err(SapTransferFinalityError::WitnessPolicyMismatch)
        );
        assert_eq!(
            witness.validate_for(&policy(), &transfer.transfer_id, "uhCkk-other-spend"),
            Err(SapTransferFinalityError::WitnessSpendActionMismatch)
        );
    }

    #[test]
    fn bare_recipient_claim_requires_verified_witness_before_spendability() {
        let transfer = transfer();
        let claim = recipient_claim(&transfer);
        let spendable = SpendableTransferClaimV2::from_verified_witness(
            claim,
            "uhCkk-spend".into(),
            witness(&transfer),
            &policy(),
        )
        .unwrap();
        assert_eq!(spendable.spendable_note(), &transfer.recipient_note);
    }

    #[test]
    fn witness_for_different_transfer_cannot_make_claim_spendable() {
        let transfer = transfer();
        let claim = recipient_claim(&transfer);
        let mut other = transfer.clone();
        other.transfer_id = "other-transfer".into();
        let bad = VerifiedWitnessSpendAssuranceV1::from_authenticated_evidence(
            other.transfer_id,
            "uhCkk-spend".into(),
            "uhCkk-witness".into(),
            &policy(),
        )
        .unwrap();
        assert_eq!(
            SpendableTransferClaimV2::from_verified_witness(
                claim,
                "uhCkk-spend".into(),
                bad,
                &policy(),
            ),
            Err(SapTransferFinalityError::WitnessTransferMismatch)
        );
    }

    #[test]
    fn malformed_availability_cannot_be_smuggled_in() {
        let transfer = transfer();
        assert_eq!(
            classify_transfer_output_status(
                &transfer,
                "uhCkk-spend",
                &SapTransferOutputAvailabilityV2::FrozenConflict {
                    input_note_ids: vec!["unrelated-note".into()],
                },
                &policy(),
                None,
            ),
            Err(SapTransferFinalityError::MalformedAvailability)
        );
    }

    #[test]
    fn serde_round_trip_requires_policy_revalidation() {
        let transfer = transfer();
        let spendable = SpendableTransferClaimV2::from_verified_witness(
            recipient_claim(&transfer),
            "uhCkk-spend".into(),
            witness(&transfer),
            &policy(),
        )
        .unwrap();
        let json = serde_json::to_string(&spendable).unwrap();
        let decoded: SpendableTransferClaimV2 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, spendable);
        assert!(decoded.validate_for_policy(&policy()).is_ok());
    }
}

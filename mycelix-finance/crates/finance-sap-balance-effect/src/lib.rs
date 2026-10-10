#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-012 pure SAP balance-effect and fork-conflict theorems.
//!
//! This crate is deliberately storage-independent. It proves what one collateral
//! issuance is allowed to do to one exact predecessor balance, how sequential
//! receipt replay is rejected from exact predecessor ancestry, and how a reader
//! must classify multiple observed successors. It does **not** claim that an
//! action reference is globally unique consumption under Holochain forks.

use finance_collateral_issuance_persistence::{
    CollateralSapIssuanceReceiptRecordV2,
    COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION,
};
use mycelix_finance_types::AmberExemption;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const SAP_BALANCE_EFFECT_V2_SCHEMA_VERSION: u16 = 1;
pub const MAX_ACTION_REFERENCE_LEN: usize = 256;
pub const MAX_DID_LEN: usize = 256;
pub const MAX_ID_LEN: usize = 256;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SapBalanceStateV2 {
    pub member_did: String,
    pub balance: u64,
    pub last_demurrage_at_micros: i64,
    pub exemption: Option<AmberExemption>,
    pub justified_by: Option<String>,
}

impl SapBalanceStateV2 {
    pub fn validate_shape(&self) -> Result<(), SapBalanceEffectError> {
        if !valid_did(&self.member_did) {
            return Err(SapBalanceEffectError::InvalidMemberDid);
        }
        if let Some(reference) = &self.justified_by {
            validate_action_reference(reference)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralIssuanceEffectV2 {
    pub schema_version: u16,
    pub predecessor_action_reference: String,
    pub issuance_receipt_action_reference: String,
    pub member_did: String,
    pub deposit_id: String,
    pub amount: u64,
    pub predecessor_balance: u64,
    pub resulting_balance: u64,
}

impl CollateralIssuanceEffectV2 {
    pub fn derive(
        predecessor_action_reference: String,
        predecessor: &SapBalanceStateV2,
        issuance_receipt_action_reference: String,
        receipt: &CollateralSapIssuanceReceiptRecordV2,
        ancestor_justification_references: &[String],
    ) -> Result<Self, SapBalanceEffectError> {
        predecessor.validate_shape()?;
        validate_action_reference(&predecessor_action_reference)?;
        validate_action_reference(&issuance_receipt_action_reference)?;
        validate_receipt_shape(receipt)?;
        validate_receipt_not_in_ancestry(
            predecessor,
            &issuance_receipt_action_reference,
            ancestor_justification_references,
        )?;

        if receipt.recipient_did != predecessor.member_did {
            return Err(SapBalanceEffectError::ReceiptRecipientMismatch);
        }

        let resulting_balance = predecessor
            .balance
            .checked_add(receipt.amount)
            .ok_or(SapBalanceEffectError::BalanceOverflow)?;

        let effect = Self {
            schema_version: SAP_BALANCE_EFFECT_V2_SCHEMA_VERSION,
            predecessor_action_reference,
            issuance_receipt_action_reference,
            member_did: predecessor.member_did.clone(),
            deposit_id: receipt.deposit_id.clone(),
            amount: receipt.amount,
            predecessor_balance: predecessor.balance,
            resulting_balance,
        };
        effect.validate_against(predecessor, receipt)?;
        Ok(effect)
    }

    pub fn validate_against(
        &self,
        predecessor: &SapBalanceStateV2,
        receipt: &CollateralSapIssuanceReceiptRecordV2,
    ) -> Result<(), SapBalanceEffectError> {
        if self.schema_version != SAP_BALANCE_EFFECT_V2_SCHEMA_VERSION {
            return Err(SapBalanceEffectError::UnsupportedEffectSchema);
        }
        predecessor.validate_shape()?;
        validate_receipt_shape(receipt)?;
        validate_action_reference(&self.predecessor_action_reference)?;
        validate_action_reference(&self.issuance_receipt_action_reference)?;
        if !valid_did(&self.member_did) {
            return Err(SapBalanceEffectError::InvalidMemberDid);
        }
        if !valid_id(&self.deposit_id) {
            return Err(SapBalanceEffectError::InvalidDepositId);
        }
        if self.amount == 0 {
            return Err(SapBalanceEffectError::ZeroAmount);
        }
        if self.member_did != predecessor.member_did
            || self.member_did != receipt.recipient_did
        {
            return Err(SapBalanceEffectError::ReceiptRecipientMismatch);
        }
        if self.deposit_id != receipt.deposit_id || self.amount != receipt.amount {
            return Err(SapBalanceEffectError::ReceiptFactsMismatch);
        }
        if self.predecessor_balance != predecessor.balance {
            return Err(SapBalanceEffectError::PredecessorBalanceMismatch);
        }
        let expected = predecessor
            .balance
            .checked_add(receipt.amount)
            .ok_or(SapBalanceEffectError::BalanceOverflow)?;
        if self.resulting_balance != expected {
            return Err(SapBalanceEffectError::ResultingBalanceMismatch);
        }
        Ok(())
    }

    pub fn expected_successor(
        &self,
        predecessor: &SapBalanceStateV2,
        receipt: &CollateralSapIssuanceReceiptRecordV2,
    ) -> Result<SapBalanceStateV2, SapBalanceEffectError> {
        self.validate_against(predecessor, receipt)?;
        Ok(SapBalanceStateV2 {
            member_did: predecessor.member_did.clone(),
            balance: self.resulting_balance,
            last_demurrage_at_micros: predecessor.last_demurrage_at_micros,
            exemption: predecessor.exemption.clone(),
            justified_by: Some(self.issuance_receipt_action_reference.clone()),
        })
    }
}

pub fn validate_collateral_issuance_transition(
    predecessor_action_reference: &str,
    predecessor: &SapBalanceStateV2,
    issuance_receipt_action_reference: &str,
    receipt: &CollateralSapIssuanceReceiptRecordV2,
    ancestor_justification_references: &[String],
    successor: &SapBalanceStateV2,
) -> Result<CollateralIssuanceEffectV2, SapBalanceEffectError> {
    successor.validate_shape()?;
    let effect = CollateralIssuanceEffectV2::derive(
        predecessor_action_reference.to_string(),
        predecessor,
        issuance_receipt_action_reference.to_string(),
        receipt,
        ancestor_justification_references,
    )?;

    if successor.member_did != predecessor.member_did {
        return Err(SapBalanceEffectError::MemberChanged);
    }
    if successor.balance != effect.resulting_balance {
        return Err(SapBalanceEffectError::IssuanceDeltaMismatch);
    }
    if successor.last_demurrage_at_micros != predecessor.last_demurrage_at_micros {
        return Err(SapBalanceEffectError::DemurrageTimestampChanged);
    }
    if successor.exemption != predecessor.exemption {
        return Err(SapBalanceEffectError::ExemptionChanged);
    }
    if successor.justified_by.as_deref() != Some(issuance_receipt_action_reference) {
        return Err(SapBalanceEffectError::JustificationMismatch);
    }
    Ok(effect)
}

pub fn validate_genesis_balance(
    state: &SapBalanceStateV2,
) -> Result<(), SapBalanceEffectError> {
    state.validate_shape()?;
    if state.balance != 0 {
        return Err(SapBalanceEffectError::NonZeroGenesisBalance);
    }
    if state.justified_by.is_some() {
        return Err(SapBalanceEffectError::GenesisHasJustification);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BalanceSuccessorObservationV2 {
    pub predecessor_action_reference: String,
    pub successor_action_reference: String,
    pub successor: SapBalanceStateV2,
}

impl BalanceSuccessorObservationV2 {
    pub fn validate_shape(&self) -> Result<(), SapBalanceEffectError> {
        validate_action_reference(&self.predecessor_action_reference)?;
        validate_action_reference(&self.successor_action_reference)?;
        if self.predecessor_action_reference == self.successor_action_reference {
            return Err(SapBalanceEffectError::SelfReferentialSuccessor);
        }
        self.successor.validate_shape()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BalanceLineageStatusV2 {
    Current {
        predecessor_action_reference: String,
        state: SapBalanceStateV2,
    },
    Advanced {
        predecessor_action_reference: String,
        successor_action_reference: String,
        state: SapBalanceStateV2,
    },
    Conflict {
        predecessor_action_reference: String,
        successor_action_references: Vec<String>,
    },
}

impl BalanceLineageStatusV2 {
    pub fn is_spendable(&self) -> bool {
        !matches!(self, Self::Conflict { .. })
    }
}

pub fn classify_successors(
    predecessor_action_reference: &str,
    predecessor: &SapBalanceStateV2,
    observations: &[BalanceSuccessorObservationV2],
) -> Result<BalanceLineageStatusV2, SapBalanceEffectError> {
    validate_action_reference(predecessor_action_reference)?;
    predecessor.validate_shape()?;

    let mut unique: BTreeMap<String, SapBalanceStateV2> = BTreeMap::new();
    for observation in observations {
        observation.validate_shape()?;
        if observation.predecessor_action_reference != predecessor_action_reference {
            return Err(SapBalanceEffectError::PredecessorReferenceMismatch);
        }
        match unique.get(&observation.successor_action_reference) {
            Some(existing) if existing == &observation.successor => {}
            Some(_) => return Err(SapBalanceEffectError::InconsistentDuplicateAction),
            None => {
                unique.insert(
                    observation.successor_action_reference.clone(),
                    observation.successor.clone(),
                );
            }
        }
    }

    if unique.is_empty() {
        return Ok(BalanceLineageStatusV2::Current {
            predecessor_action_reference: predecessor_action_reference.to_string(),
            state: predecessor.clone(),
        });
    }
    if unique.len() == 1 {
        if let Some((successor_action_reference, state)) = unique.into_iter().next() {
            return Ok(BalanceLineageStatusV2::Advanced {
                predecessor_action_reference: predecessor_action_reference.to_string(),
                successor_action_reference,
                state,
            });
        }
        return Err(SapBalanceEffectError::SuccessorClassificationInvariant);
    }

    Ok(BalanceLineageStatusV2::Conflict {
        predecessor_action_reference: predecessor_action_reference.to_string(),
        successor_action_references: unique.into_keys().collect(),
    })
}

fn validate_receipt_not_in_ancestry(
    predecessor: &SapBalanceStateV2,
    issuance_receipt_action_reference: &str,
    ancestor_justification_references: &[String],
) -> Result<(), SapBalanceEffectError> {
    validate_action_reference(issuance_receipt_action_reference)?;
    if predecessor.justified_by.as_deref() == Some(issuance_receipt_action_reference) {
        return Err(SapBalanceEffectError::ReceiptReplayInAncestry);
    }
    for reference in ancestor_justification_references {
        validate_action_reference(reference)?;
        if reference == issuance_receipt_action_reference {
            return Err(SapBalanceEffectError::ReceiptReplayInAncestry);
        }
    }
    Ok(())
}

fn validate_receipt_shape(
    receipt: &CollateralSapIssuanceReceiptRecordV2,
) -> Result<(), SapBalanceEffectError> {
    if receipt.schema_version != COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION {
        return Err(SapBalanceEffectError::UnsupportedReceiptSchema);
    }
    for reference in [
        &receipt.authorization_action_reference,
        &receipt.mint_action_reference,
        &receipt.request_action_reference,
        &receipt.price_attestation_action_reference,
        &receipt.custody_attestation_action_reference,
    ] {
        validate_action_reference(reference)?;
    }
    if !valid_id(&receipt.trust_root_id)
        || receipt.trust_root_version == 0
        || !valid_id(&receipt.deposit_id)
        || !valid_id(&receipt.mint_id)
    {
        return Err(SapBalanceEffectError::InvalidReceiptIdentity);
    }
    if !valid_did(&receipt.recipient_did) {
        return Err(SapBalanceEffectError::InvalidReceiptRecipientDid);
    }
    if receipt.amount == 0 {
        return Err(SapBalanceEffectError::ZeroAmount);
    }
    Ok(())
}

fn validate_action_reference(reference: &str) -> Result<(), SapBalanceEffectError> {
    if reference.is_empty() || reference.len() > MAX_ACTION_REFERENCE_LEN {
        return Err(SapBalanceEffectError::InvalidActionReference);
    }
    Ok(())
}

fn valid_did(value: &str) -> bool {
    value.starts_with("did:") && value.len() <= MAX_DID_LEN
}

fn valid_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= MAX_ID_LEN
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapBalanceEffectError {
    InvalidMemberDid,
    InvalidActionReference,
    InvalidDepositId,
    InvalidReceiptIdentity,
    InvalidReceiptRecipientDid,
    UnsupportedEffectSchema,
    UnsupportedReceiptSchema,
    ZeroAmount,
    NonZeroGenesisBalance,
    GenesisHasJustification,
    ReceiptRecipientMismatch,
    ReceiptFactsMismatch,
    ReceiptReplayInAncestry,
    PredecessorBalanceMismatch,
    ResultingBalanceMismatch,
    BalanceOverflow,
    MemberChanged,
    IssuanceDeltaMismatch,
    DemurrageTimestampChanged,
    ExemptionChanged,
    JustificationMismatch,
    PredecessorReferenceMismatch,
    SelfReferentialSuccessor,
    InconsistentDuplicateAction,
    SuccessorClassificationInvariant,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_issuance_persistence::COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION;
    use mycelix_finance_types::{AmberClass, AmberExemption};

    fn receipt(amount: u64) -> CollateralSapIssuanceReceiptRecordV2 {
        CollateralSapIssuanceReceiptRecordV2 {
            schema_version: COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION,
            authorization_action_reference: "uhCkk-auth".into(),
            mint_action_reference: "uhCkk-mint".into(),
            trust_root_id: "finance-root".into(),
            trust_root_version: 1,
            request_action_reference: "uhCkk-request".into(),
            price_attestation_action_reference: "uhCkk-price".into(),
            custody_attestation_action_reference: "uhCkk-custody".into(),
            deposit_id: "deposit:v2:test".into(),
            mint_id: "deposit:v2:test".into(),
            recipient_did: "did:mycelix:alice".into(),
            amount,
        }
    }

    fn balance(value: u64) -> SapBalanceStateV2 {
        SapBalanceStateV2 {
            member_did: "did:mycelix:alice".into(),
            balance: value,
            last_demurrage_at_micros: 10,
            exemption: None,
            justified_by: None,
        }
    }

    #[test]
    fn genesis_is_exactly_zero_and_unjustified() {
        assert_eq!(validate_genesis_balance(&balance(0)), Ok(()));
        assert_eq!(
            validate_genesis_balance(&balance(1)),
            Err(SapBalanceEffectError::NonZeroGenesisBalance)
        );
        let mut justified = balance(0);
        justified.justified_by = Some("uhCkk-old".into());
        assert_eq!(
            validate_genesis_balance(&justified),
            Err(SapBalanceEffectError::GenesisHasJustification)
        );
    }

    #[test]
    fn exact_receipt_produces_one_exact_successor() {
        let predecessor = balance(100);
        let r = receipt(20);
        let effect = CollateralIssuanceEffectV2::derive(
            "uhCkk-prev".into(),
            &predecessor,
            "uhCkk-receipt".into(),
            &r,
            &[],
        )
        .expect("effect");
        let successor = effect
            .expected_successor(&predecessor, &r)
            .expect("successor");
        assert_eq!(successor.balance, 120);
        assert_eq!(successor.last_demurrage_at_micros, 10);
        assert_eq!(successor.justified_by.as_deref(), Some("uhCkk-receipt"));
        assert_eq!(
            validate_collateral_issuance_transition(
                "uhCkk-prev",
                &predecessor,
                "uhCkk-receipt",
                &r,
                &[],
                &successor,
            ),
            Ok(effect)
        );
    }

    #[test]
    fn receipt_for_another_member_fails_closed() {
        let predecessor = balance(100);
        let mut r = receipt(20);
        r.recipient_did = "did:mycelix:bob".into();
        assert_eq!(
            CollateralIssuanceEffectV2::derive(
                "uhCkk-prev".into(),
                &predecessor,
                "uhCkk-receipt".into(),
                &r,
                &[],
            ),
            Err(SapBalanceEffectError::ReceiptRecipientMismatch)
        );
    }

    #[test]
    fn checked_addition_rejects_overflow() {
        let predecessor = balance(u64::MAX);
        assert_eq!(
            CollateralIssuanceEffectV2::derive(
                "uhCkk-prev".into(),
                &predecessor,
                "uhCkk-receipt".into(),
                &receipt(1),
                &[],
            ),
            Err(SapBalanceEffectError::BalanceOverflow)
        );
    }

    #[test]
    fn same_receipt_cannot_be_reapplied_to_later_predecessor() {
        let mut later = balance(120);
        later.justified_by = Some("uhCkk-receipt".into());
        assert_eq!(
            CollateralIssuanceEffectV2::derive(
                "uhCkk-later".into(),
                &later,
                "uhCkk-receipt".into(),
                &receipt(20),
                &[],
            ),
            Err(SapBalanceEffectError::ReceiptReplayInAncestry)
        );

        let later_other_effect = SapBalanceStateV2 {
            justified_by: Some("uhCkk-transfer".into()),
            ..balance(120)
        };
        assert_eq!(
            CollateralIssuanceEffectV2::derive(
                "uhCkk-later".into(),
                &later_other_effect,
                "uhCkk-receipt".into(),
                &receipt(20),
                &["uhCkk-receipt".into()],
            ),
            Err(SapBalanceEffectError::ReceiptReplayInAncestry)
        );
    }

    #[test]
    fn issuance_cannot_hide_demurrage() {
        let predecessor = balance(100);
        let r = receipt(20);
        let mut successor = CollateralIssuanceEffectV2::derive(
            "uhCkk-prev".into(),
            &predecessor,
            "uhCkk-receipt".into(),
            &r,
            &[],
        )
        .unwrap()
        .expected_successor(&predecessor, &r)
        .unwrap();
        successor.last_demurrage_at_micros += 1;
        assert_eq!(
            validate_collateral_issuance_transition(
                "uhCkk-prev",
                &predecessor,
                "uhCkk-receipt",
                &r,
                &[],
                &successor,
            ),
            Err(SapBalanceEffectError::DemurrageTimestampChanged)
        );
    }

    #[test]
    fn issuance_cannot_change_amber_exemption() {
        let mut predecessor = balance(100);
        predecessor.exemption = Some(AmberExemption {
            class: AmberClass::Custodial,
            issuer: "did:mycelix:steward".into(),
            cap_micro_sap: 50,
            expires_at_secs: 1000,
        });
        let r = receipt(20);
        let mut successor = CollateralIssuanceEffectV2::derive(
            "uhCkk-prev".into(),
            &predecessor,
            "uhCkk-receipt".into(),
            &r,
            &[],
        )
        .unwrap()
        .expected_successor(&predecessor, &r)
        .unwrap();
        successor.exemption = None;
        assert_eq!(
            validate_collateral_issuance_transition(
                "uhCkk-prev",
                &predecessor,
                "uhCkk-receipt",
                &r,
                &[],
                &successor,
            ),
            Err(SapBalanceEffectError::ExemptionChanged)
        );
    }

    #[test]
    fn issuance_requires_exact_receipt_justification() {
        let predecessor = balance(100);
        let r = receipt(20);
        let effect = CollateralIssuanceEffectV2::derive(
            "uhCkk-prev".into(),
            &predecessor,
            "uhCkk-receipt".into(),
            &r,
            &[],
        )
        .unwrap();
        let mut successor = effect.expected_successor(&predecessor, &r).unwrap();
        successor.justified_by = Some("uhCkk-other".into());
        assert_eq!(
            validate_collateral_issuance_transition(
                "uhCkk-prev",
                &predecessor,
                "uhCkk-receipt",
                &r,
                &[],
                &successor,
            ),
            Err(SapBalanceEffectError::JustificationMismatch)
        );
    }

    fn observation(
        action: &str,
        predecessor: &str,
        state: SapBalanceStateV2,
    ) -> BalanceSuccessorObservationV2 {
        BalanceSuccessorObservationV2 {
            predecessor_action_reference: predecessor.into(),
            successor_action_reference: action.into(),
            successor: state,
        }
    }

    #[test]
    fn zero_successors_is_current_and_one_is_advanced() {
        let predecessor = balance(100);
        assert!(matches!(
            classify_successors("uhCkk-prev", &predecessor, &[]).unwrap(),
            BalanceLineageStatusV2::Current { .. }
        ));

        let one = vec![observation("uhCkk-next", "uhCkk-prev", balance(120))];
        assert!(matches!(
            classify_successors("uhCkk-prev", &predecessor, &one).unwrap(),
            BalanceLineageStatusV2::Advanced { .. }
        ));
    }

    #[test]
    fn same_action_observed_twice_is_deduped_only_if_payload_matches() {
        let predecessor = balance(100);
        let next = balance(120);
        let repeated = vec![
            observation("uhCkk-next", "uhCkk-prev", next.clone()),
            observation("uhCkk-next", "uhCkk-prev", next),
        ];
        assert!(matches!(
            classify_successors("uhCkk-prev", &predecessor, &repeated).unwrap(),
            BalanceLineageStatusV2::Advanced { .. }
        ));

        let inconsistent = vec![
            observation("uhCkk-next", "uhCkk-prev", balance(120)),
            observation("uhCkk-next", "uhCkk-prev", balance(121)),
        ];
        assert_eq!(
            classify_successors("uhCkk-prev", &predecessor, &inconsistent),
            Err(SapBalanceEffectError::InconsistentDuplicateAction)
        );
    }

    #[test]
    fn distinct_identical_successors_are_conflict_not_deduped() {
        let predecessor = balance(100);
        let same_payload = balance(120);
        let observations = vec![
            observation("uhCkk-next-a", "uhCkk-prev", same_payload.clone()),
            observation("uhCkk-next-b", "uhCkk-prev", same_payload),
        ];
        let status = classify_successors("uhCkk-prev", &predecessor, &observations).unwrap();
        assert!(!status.is_spendable());
        match &status {
            BalanceLineageStatusV2::Conflict {
                successor_action_references,
                ..
            } => assert_eq!(successor_action_references.len(), 2),
            other => panic!("expected conflict, got {other:?}"),
        }
    }

    #[test]
    fn successor_for_another_predecessor_is_not_silently_mixed() {
        let predecessor = balance(100);
        let observations = vec![observation(
            "uhCkk-next",
            "uhCkk-other-prev",
            balance(120),
        )];
        assert_eq!(
            classify_successors("uhCkk-prev", &predecessor, &observations),
            Err(SapBalanceEffectError::PredecessorReferenceMismatch)
        );
    }

    #[test]
    fn serialization_retains_effect_identity() {
        let predecessor = balance(100);
        let effect = CollateralIssuanceEffectV2::derive(
            "uhCkk-prev".into(),
            &predecessor,
            "uhCkk-receipt".into(),
            &receipt(20),
            &[],
        )
        .unwrap();
        let bytes = serde_json::to_vec(&effect).unwrap();
        let decoded: CollateralIssuanceEffectV2 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, effect);
    }
}

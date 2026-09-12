#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Author-bound public API for FIN-SAFE-012.
//!
//! The lower-level transition math lives in `lib.rs`, but authoritative Holochain
//! integration should use this surface. Every checked balance state transition is
//! bound to the balance owner's DID, preventing arbitrary third parties from
//! creating otherwise-valid sibling successors against another member's account.

#[path = "lib.rs"]
mod kernel;

pub use kernel::{
    BalanceLineageStatusV2, CollateralIssuanceEffectV2, SapBalanceEffectError,
    SapBalanceStateV2, MAX_ACTION_REFERENCE_LEN, MAX_DID_LEN, MAX_ID_LEN,
    SAP_BALANCE_EFFECT_V2_SCHEMA_VERSION,
};

use finance_collateral_issuance_persistence::CollateralSapIssuanceReceiptRecordV2;
use serde::{Deserialize, Serialize};

/// One already source-specifically validated successor plus its actual action
/// author. The author must be the balance owner before it may participate in
/// lineage/fork classification.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AuthorBoundBalanceSuccessorObservationV2 {
    pub action_author_did: String,
    pub predecessor_action_reference: String,
    pub successor_action_reference: String,
    pub successor: SapBalanceStateV2,
}

impl AuthorBoundBalanceSuccessorObservationV2 {
    pub fn validate_author_binding(&self) -> Result<(), AuthorBoundSapBalanceEffectError> {
        require_owner_author(&self.action_author_did, &self.successor.member_did)?;
        let inner = kernel::BalanceSuccessorObservationV2 {
            predecessor_action_reference: self.predecessor_action_reference.clone(),
            successor_action_reference: self.successor_action_reference.clone(),
            successor: self.successor.clone(),
        };
        inner
            .validate_shape()
            .map_err(AuthorBoundSapBalanceEffectError::Kernel)
    }
}

/// Checked-path balance genesis is owner-authored, zero, and unjustified.
pub fn validate_author_bound_genesis(
    action_author_did: &str,
    state: &SapBalanceStateV2,
) -> Result<(), AuthorBoundSapBalanceEffectError> {
    require_owner_author(action_author_did, &state.member_did)?;
    kernel::validate_genesis_balance(state).map_err(AuthorBoundSapBalanceEffectError::Kernel)
}

/// Validate one owner-authored collateral issuance successor.
///
/// Both predecessor and successor action authors are bound to the account owner.
/// This does not replace exact Holochain action loading: the integration layer must
/// derive these DID strings from the real actions and must validate the exact
/// FIN-SAFE-010 receipt action before calling this function.
pub fn validate_author_bound_collateral_issuance_transition(
    predecessor_author_did: &str,
    predecessor_action_reference: &str,
    predecessor: &SapBalanceStateV2,
    successor_author_did: &str,
    issuance_receipt_action_reference: &str,
    receipt: &CollateralSapIssuanceReceiptRecordV2,
    ancestor_justification_references: &[String],
    successor: &SapBalanceStateV2,
) -> Result<CollateralIssuanceEffectV2, AuthorBoundSapBalanceEffectError> {
    require_owner_author(predecessor_author_did, &predecessor.member_did)?;
    require_owner_author(successor_author_did, &successor.member_did)?;
    if predecessor.member_did != successor.member_did {
        return Err(AuthorBoundSapBalanceEffectError::OwnerChanged);
    }
    kernel::validate_collateral_issuance_transition(
        predecessor_action_reference,
        predecessor,
        issuance_receipt_action_reference,
        receipt,
        ancestor_justification_references,
        successor,
    )
    .map_err(AuthorBoundSapBalanceEffectError::Kernel)
}

/// Classify already-validated owner-authored successors of one exact predecessor.
///
/// A third party cannot manufacture a sibling observation that participates in
/// the checked lineage. Distinct owner-authored successor action hashes still
/// produce `Conflict`; same-key/source-chain fork ambiguity remains explicit.
pub fn classify_author_bound_successors(
    predecessor_author_did: &str,
    predecessor_action_reference: &str,
    predecessor: &SapBalanceStateV2,
    observations: &[AuthorBoundBalanceSuccessorObservationV2],
) -> Result<BalanceLineageStatusV2, AuthorBoundSapBalanceEffectError> {
    require_owner_author(predecessor_author_did, &predecessor.member_did)?;
    let mut inner = Vec::with_capacity(observations.len());
    for observation in observations {
        observation.validate_author_binding()?;
        if observation.successor.member_did != predecessor.member_did {
            return Err(AuthorBoundSapBalanceEffectError::OwnerChanged);
        }
        inner.push(kernel::BalanceSuccessorObservationV2 {
            predecessor_action_reference: observation.predecessor_action_reference.clone(),
            successor_action_reference: observation.successor_action_reference.clone(),
            successor: observation.successor.clone(),
        });
    }
    kernel::classify_successors(predecessor_action_reference, predecessor, &inner)
        .map_err(AuthorBoundSapBalanceEffectError::Kernel)
}

fn require_owner_author(
    action_author_did: &str,
    member_did: &str,
) -> Result<(), AuthorBoundSapBalanceEffectError> {
    if !valid_did(action_author_did) {
        return Err(AuthorBoundSapBalanceEffectError::InvalidActionAuthorDid);
    }
    if !valid_did(member_did) {
        return Err(AuthorBoundSapBalanceEffectError::InvalidMemberDid);
    }
    if action_author_did != member_did {
        return Err(AuthorBoundSapBalanceEffectError::ActionAuthorMismatch);
    }
    Ok(())
}

fn valid_did(value: &str) -> bool {
    value.starts_with("did:") && value.len() <= MAX_DID_LEN
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorBoundSapBalanceEffectError {
    InvalidActionAuthorDid,
    InvalidMemberDid,
    ActionAuthorMismatch,
    OwnerChanged,
    Kernel(SapBalanceEffectError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_issuance_persistence::{
        CollateralSapIssuanceReceiptRecordV2,
        COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION,
    };

    fn state(value: u64) -> SapBalanceStateV2 {
        SapBalanceStateV2 {
            member_did: "did:mycelix:alice".into(),
            balance: value,
            last_demurrage_at_micros: 10,
            exemption: None,
            justified_by: None,
        }
    }

    fn receipt() -> CollateralSapIssuanceReceiptRecordV2 {
        CollateralSapIssuanceReceiptRecordV2 {
            schema_version: COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION,
            authorization_action_reference: "uhCkk-auth".into(),
            mint_action_reference: "uhCkk-mint".into(),
            trust_root_id: "root".into(),
            trust_root_version: 1,
            request_action_reference: "uhCkk-request".into(),
            price_attestation_action_reference: "uhCkk-price".into(),
            custody_attestation_action_reference: "uhCkk-custody".into(),
            deposit_id: "deposit:v2:test".into(),
            mint_id: "deposit:v2:test".into(),
            recipient_did: "did:mycelix:alice".into(),
            amount: 20,
        }
    }

    #[test]
    fn third_party_cannot_author_genesis() {
        assert_eq!(
            validate_author_bound_genesis("did:mycelix:bob", &state(0)),
            Err(AuthorBoundSapBalanceEffectError::ActionAuthorMismatch)
        );
        assert_eq!(
            validate_author_bound_genesis("did:mycelix:alice", &state(0)),
            Ok(())
        );
    }

    #[test]
    fn third_party_cannot_author_valid_looking_credit_successor() {
        let predecessor = state(100);
        let effect = kernel::CollateralIssuanceEffectV2::derive(
            "uhCkk-prev".into(),
            &predecessor,
            "uhCkk-receipt".into(),
            &receipt(),
            &[],
        )
        .unwrap();
        let successor = effect.expected_successor(&predecessor, &receipt()).unwrap();

        assert_eq!(
            validate_author_bound_collateral_issuance_transition(
                "did:mycelix:alice",
                "uhCkk-prev",
                &predecessor,
                "did:mycelix:mallory",
                "uhCkk-receipt",
                &receipt(),
                &[],
                &successor,
            ),
            Err(AuthorBoundSapBalanceEffectError::ActionAuthorMismatch)
        );
    }

    #[test]
    fn owner_authored_exact_transition_is_valid() {
        let predecessor = state(100);
        let effect = kernel::CollateralIssuanceEffectV2::derive(
            "uhCkk-prev".into(),
            &predecessor,
            "uhCkk-receipt".into(),
            &receipt(),
            &[],
        )
        .unwrap();
        let successor = effect.expected_successor(&predecessor, &receipt()).unwrap();
        assert_eq!(
            validate_author_bound_collateral_issuance_transition(
                "did:mycelix:alice",
                "uhCkk-prev",
                &predecessor,
                "did:mycelix:alice",
                "uhCkk-receipt",
                &receipt(),
                &[],
                &successor,
            ),
            Ok(effect)
        );
    }

    #[test]
    fn third_party_sibling_is_rejected_before_conflict_classification() {
        let predecessor = state(100);
        let observation = AuthorBoundBalanceSuccessorObservationV2 {
            action_author_did: "did:mycelix:mallory".into(),
            predecessor_action_reference: "uhCkk-prev".into(),
            successor_action_reference: "uhCkk-forged".into(),
            successor: state(120),
        };
        assert_eq!(
            classify_author_bound_successors(
                "did:mycelix:alice",
                "uhCkk-prev",
                &predecessor,
                &[observation],
            ),
            Err(AuthorBoundSapBalanceEffectError::ActionAuthorMismatch)
        );
    }

    #[test]
    fn distinct_owner_authored_siblings_remain_non_spendable_conflict() {
        let predecessor = state(100);
        let observations = vec![
            AuthorBoundBalanceSuccessorObservationV2 {
                action_author_did: "did:mycelix:alice".into(),
                predecessor_action_reference: "uhCkk-prev".into(),
                successor_action_reference: "uhCkk-a".into(),
                successor: state(120),
            },
            AuthorBoundBalanceSuccessorObservationV2 {
                action_author_did: "did:mycelix:alice".into(),
                predecessor_action_reference: "uhCkk-prev".into(),
                successor_action_reference: "uhCkk-b".into(),
                successor: state(120),
            },
        ];
        let status = classify_author_bound_successors(
            "did:mycelix:alice",
            "uhCkk-prev",
            &predecessor,
            &observations,
        )
        .unwrap();
        assert!(!status.is_spendable());
        assert!(matches!(status, BalanceLineageStatusV2::Conflict { .. }));
    }
}

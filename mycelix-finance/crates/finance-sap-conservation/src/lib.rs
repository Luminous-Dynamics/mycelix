#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Pure SAP conservation and issuance-provenance theorems.
//!
//! This crate does not replace the Holochain `SapMintRecord` or `SapBalance`
//! entries. It supplies a small deterministic validation kernel that the
//! payments integrity/coordinator integration can map those entries into.
//!
//! A crucial rule is intentionally strict: demurrage and issuance are separate
//! balance transitions. A positive issuance transition must increase the raw
//! balance by exactly the immutable mint amount. Mixing demurrage and minting in
//! one update destroys the simple predecessor-delta theorem and is therefore not
//! accepted by this checked path.

use finance_collateral_settlement::{
    CollateralSettlementAuthorityPolicy, CollateralSettlementIntent,
    PolicyBoundSettlementError,
};
use mycelix_finance_types::SapMintSource;
use serde::{Deserialize, Serialize};

pub const MAX_MINT_ID_LEN: usize = 256;
pub const MAX_ACTION_REFERENCE_LEN: usize = 256;

/// Minimal, storage-independent view of the existing immutable Holochain
/// `SapMintRecord` fields needed by the conservation theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapMintRecordView {
    pub mint_id: String,
    pub recipient_did: String,
    pub amount: u64,
    pub source: SapMintSource,
}

impl SapMintRecordView {
    pub fn validate_shape(&self) -> Result<(), SapConservationError> {
        if self.mint_id.is_empty() || self.mint_id.len() > MAX_MINT_ID_LEN {
            return Err(SapConservationError::InvalidMintId);
        }
        if !self.recipient_did.starts_with("did:") || self.recipient_did.len() > 256 {
            return Err(SapConservationError::InvalidRecipientDid);
        }
        if self.amount == 0 {
            return Err(SapConservationError::ZeroMintAmount);
        }
        match &self.source {
            SapMintSource::CollateralBridge { deposit_id } if deposit_id.is_empty() => {
                Err(SapConservationError::InvalidCollateralDepositId)
            }
            SapMintSource::CollateralBridge { deposit_id }
                if deposit_id.len() > MAX_MINT_ID_LEN =>
            {
                Err(SapConservationError::InvalidCollateralDepositId)
            }
            _ => Ok(()),
        }
    }
}

/// Minimal view of a SAP balance state used to prove one transition.
///
/// `justified_by` is represented as a stable action-reference string here so the
/// pure crate remains HDI-free. Holochain integration must map the real
/// `ActionHash` losslessly to/from this field and load that exact action.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapBalanceStateView {
    pub member_did: String,
    pub balance: u64,
    pub justified_by: Option<String>,
}

impl SapBalanceStateView {
    pub fn validate_shape(&self) -> Result<(), SapConservationError> {
        if !self.member_did.starts_with("did:") || self.member_did.len() > 256 {
            return Err(SapConservationError::InvalidBalanceMemberDid);
        }
        if let Some(reference) = &self.justified_by {
            if reference.is_empty() || reference.len() > MAX_ACTION_REFERENCE_LEN {
                return Err(SapConservationError::InvalidJustificationReference);
            }
        }
        Ok(())
    }
}

/// Exact collateral-mint plan derived from an already authorized FIN-SAFE-006
/// settlement intent.
///
/// The canonical mint ID is the deposit ID itself. This deliberately adds no
/// prefix, so a valid maximum-length deposit ID cannot become an overlength mint
/// ID. Namespace/collision resistance therefore belongs to deposit-ID generation
/// and remains an integration theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizedCollateralSapMintPlan {
    pub mint_id: String,
    pub recipient_did: String,
    pub amount: u64,
    pub deposit_id: String,
}

impl AuthorizedCollateralSapMintPlan {
    pub fn from_settlement_intent(
        intent: &CollateralSettlementIntent,
        expected_policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<Self, SapConservationError> {
        intent
            .validate_against_policy(expected_policy)
            .map_err(SapConservationError::SettlementPolicy)?;

        let deposit_id = intent.basis.terms.deposit_id.clone();
        if deposit_id.is_empty() || deposit_id.len() > MAX_MINT_ID_LEN {
            return Err(SapConservationError::InvalidCollateralDepositId);
        }

        Ok(Self {
            mint_id: canonical_collateral_mint_id(&deposit_id)?.to_string(),
            recipient_did: intent.basis.terms.depositor_did.clone(),
            amount: intent.basis.sap_amount,
            deposit_id,
        })
    }

    /// Validate that an existing immutable mint record is exactly the one
    /// authorized by this settlement plan.
    pub fn validate_mint_record(
        &self,
        mint: &SapMintRecordView,
    ) -> Result<(), SapConservationError> {
        mint.validate_shape()?;
        if mint.mint_id != self.mint_id {
            return Err(SapConservationError::MintIdMismatch);
        }
        if mint.recipient_did != self.recipient_did {
            return Err(SapConservationError::MintRecipientMismatch);
        }
        if mint.amount != self.amount {
            return Err(SapConservationError::MintAmountMismatch);
        }
        match &mint.source {
            SapMintSource::CollateralBridge { deposit_id }
                if deposit_id == &self.deposit_id => Ok(()),
            _ => Err(SapConservationError::MintSourceMismatch),
        }
    }

    pub fn expected_record_view(&self) -> SapMintRecordView {
        SapMintRecordView {
            mint_id: self.mint_id.clone(),
            recipient_did: self.recipient_did.clone(),
            amount: self.amount,
            source: SapMintSource::CollateralBridge {
                deposit_id: self.deposit_id.clone(),
            },
        }
    }
}

/// Canonical collateral mint identity for FIN-SAFE-007 v1.
///
/// The exact deposit ID is reused rather than prefixed/truncated. A future change
/// to this identity rule requires an explicit protocol/version migration.
pub fn canonical_collateral_mint_id(
    deposit_id: &str,
) -> Result<&str, SapConservationError> {
    if deposit_id.is_empty() || deposit_id.len() > MAX_MINT_ID_LEN {
        return Err(SapConservationError::InvalidCollateralDepositId);
    }
    Ok(deposit_id)
}

/// Genesis balance theorem: an ordinary balance is created at zero with no
/// provenance pointer. Issuance to a previously unseen member must initialize
/// zero first and then perform a separately justified issuance update.
pub fn validate_genesis_balance(
    balance: &SapBalanceStateView,
) -> Result<(), SapConservationError> {
    balance.validate_shape()?;
    if balance.balance != 0 {
        return Err(SapConservationError::NonZeroGenesisBalance);
    }
    if balance.justified_by.is_some() {
        return Err(SapConservationError::GenesisMustNotHaveJustification);
    }
    Ok(())
}

/// Validate one positive issuance transition against one exact immutable mint.
///
/// The action reference is supplied separately so Holochain integration can load
/// the exact action, decode it to `SapMintRecordView`, validate it against the
/// source-specific authorization plan, and then prove the balance delta.
///
/// Demurrage MUST NOT be folded into this transition. The expected next raw
/// balance is exactly `previous.balance + mint.amount`, using checked addition.
pub fn validate_positive_issuance_transition(
    previous: &SapBalanceStateView,
    updated: &SapBalanceStateView,
    mint_action_reference: &str,
    mint: &SapMintRecordView,
) -> Result<(), SapConservationError> {
    previous.validate_shape()?;
    updated.validate_shape()?;
    mint.validate_shape()?;

    if previous.member_did != updated.member_did {
        return Err(SapConservationError::BalanceMemberChanged);
    }
    if mint.recipient_did != updated.member_did {
        return Err(SapConservationError::MintRecipientMismatch);
    }
    if mint_action_reference.is_empty()
        || mint_action_reference.len() > MAX_ACTION_REFERENCE_LEN
    {
        return Err(SapConservationError::InvalidJustificationReference);
    }
    if updated.justified_by.as_deref() != Some(mint_action_reference) {
        return Err(SapConservationError::JustificationMismatch);
    }

    let expected_balance = previous
        .balance
        .checked_add(mint.amount)
        .ok_or(SapConservationError::BalanceOverflow)?;
    if updated.balance != expected_balance {
        return Err(SapConservationError::IssuanceDeltaMismatch);
    }

    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapConservationError {
    InvalidMintId,
    InvalidRecipientDid,
    ZeroMintAmount,
    InvalidCollateralDepositId,
    InvalidBalanceMemberDid,
    InvalidJustificationReference,
    SettlementPolicy(PolicyBoundSettlementError),
    MintIdMismatch,
    MintRecipientMismatch,
    MintAmountMismatch,
    MintSourceMismatch,
    NonZeroGenesisBalance,
    GenesisMustNotHaveJustification,
    BalanceMemberChanged,
    JustificationMismatch,
    BalanceOverflow,
    IssuanceDeltaMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_settlement::{
        derive_settlement_intent, CollateralDepositTerms,
        CollateralSettlementAuthorityPolicy, CustodyAttestationCapability,
        CustodyEvidenceEnvelope, CustodyEvidenceOutcome,
        PriceAttestationCapability, PriceEvidenceEnvelope, PriceEvidenceOutcome,
        PriceRatio, SettlementFreshnessPolicy,
        COLLATERAL_SETTLEMENT_PROTOCOL_VERSION, CUSTODY_ATTESTATION_PROTOCOL_VERSION,
        PRICE_ATTESTATION_PROTOCOL_VERSION, SAP_ASSET_ID,
    };

    fn policy(provider: &str) -> CollateralSettlementAuthorityPolicy {
        CollateralSettlementAuthorityPolicy {
            policy_id: "sap-conservation-policy".into(),
            policy_version: 1,
            settlement_protocol_version: COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
            quote_asset_id: SAP_ASSET_ID.into(),
            price_capability: PriceAttestationCapability {
                provider_id: provider.into(),
                method: "price".into(),
                protocol_version: PRICE_ATTESTATION_PROTOCOL_VERSION,
            },
            custody_capability: CustodyAttestationCapability {
                custodian_id: "did:mycelix:custodian".into(),
                method: "custody".into(),
                protocol_version: CUSTODY_ATTESTATION_PROTOCOL_VERSION,
            },
            price_freshness: SettlementFreshnessPolicy {
                policy_id: "price-fresh".into(),
                policy_version: 1,
                max_age_micros: 100,
            },
            custody_freshness: SettlementFreshnessPolicy {
                policy_id: "custody-fresh".into(),
                policy_version: 1,
                max_age_micros: 100,
            },
        }
    }

    fn settlement_intent(
        deposit_id: &str,
        provider: &str,
    ) -> CollateralSettlementIntent {
        let p = policy(provider);
        let terms = CollateralDepositTerms {
            deposit_id: deposit_id.into(),
            depositor_did: "did:mycelix:alice".into(),
            collateral_asset_id: "ETH".into(),
            collateral_amount: 10,
            quote_asset_id: SAP_ASSET_ID.into(),
        };
        let price = PriceEvidenceEnvelope {
            base_asset_id: "ETH".into(),
            quote_asset_id: SAP_ASSET_ID.into(),
            capability: p.price_capability.clone(),
            outcome: PriceEvidenceOutcome::Observed {
                rate: PriceRatio {
                    numerator: 2,
                    denominator: 1,
                },
                observation_reference: "price:obs:1".into(),
                observed_at_micros: 10,
            },
        };
        let custody = CustodyEvidenceEnvelope {
            deposit_id: deposit_id.into(),
            depositor_did: "did:mycelix:alice".into(),
            collateral_asset_id: "ETH".into(),
            collateral_amount: 10,
            capability: p.custody_capability.clone(),
            outcome: CustodyEvidenceOutcome::Confirmed {
                external_reference: "custody:proof:1".into(),
                confirmed_at_micros: 10,
            },
        };
        derive_settlement_intent(terms, &p, &price, &custody, 11)
            .expect("valid settlement intent")
    }

    #[test]
    fn collateral_plan_is_derived_only_under_expected_policy() {
        let intent = settlement_intent("deposit:1", "approved-provider");
        let approved = policy("approved-provider");
        let plan = AuthorizedCollateralSapMintPlan::from_settlement_intent(
            &intent,
            &approved,
        )
        .expect("approved policy");

        assert_eq!(plan.mint_id, "deposit:1");
        assert_eq!(plan.deposit_id, "deposit:1");
        assert_eq!(plan.recipient_did, "did:mycelix:alice");
        assert_eq!(plan.amount, 20);
    }

    #[test]
    fn structurally_valid_alternate_provider_cannot_authorize_mint_plan() {
        let alternate_intent = settlement_intent("deposit:1", "alternate-provider");
        let approved = policy("approved-provider");
        assert!(matches!(
            AuthorizedCollateralSapMintPlan::from_settlement_intent(
                &alternate_intent,
                &approved,
            ),
            Err(SapConservationError::SettlementPolicy(
                PolicyBoundSettlementError::AuthorityPolicyMismatch
            ))
        ));
    }

    #[test]
    fn expected_mint_record_binds_exact_deposit_recipient_and_amount() {
        let intent = settlement_intent("deposit:1", "approved-provider");
        let plan = AuthorizedCollateralSapMintPlan::from_settlement_intent(
            &intent,
            &policy("approved-provider"),
        )
        .expect("plan");
        let record = plan.expected_record_view();
        assert_eq!(plan.validate_mint_record(&record), Ok(()));

        let mut wrong = record.clone();
        wrong.amount += 1;
        assert_eq!(
            plan.validate_mint_record(&wrong),
            Err(SapConservationError::MintAmountMismatch)
        );

        let mut wrong_source = record;
        wrong_source.source = SapMintSource::GovernanceProposal {
            proposal_id: "proposal:1".into(),
        };
        assert_eq!(
            plan.validate_mint_record(&wrong_source),
            Err(SapConservationError::MintSourceMismatch)
        );
    }

    #[test]
    fn canonical_mint_id_does_not_expand_max_length_deposit_id() {
        let deposit_id = "d".repeat(MAX_MINT_ID_LEN);
        assert_eq!(
            canonical_collateral_mint_id(&deposit_id),
            Ok(deposit_id.as_str())
        );
        assert_eq!(
            canonical_collateral_mint_id(&format!("{}x", deposit_id)),
            Err(SapConservationError::InvalidCollateralDepositId)
        );
    }

    #[test]
    fn ordinary_genesis_balance_must_be_zero_and_unjustified() {
        let valid = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 0,
            justified_by: None,
        };
        assert_eq!(validate_genesis_balance(&valid), Ok(()));

        let mut nonzero = valid.clone();
        nonzero.balance = 1;
        assert_eq!(
            validate_genesis_balance(&nonzero),
            Err(SapConservationError::NonZeroGenesisBalance)
        );
    }

    #[test]
    fn positive_delta_must_equal_exact_mint_amount_and_justification() {
        let mint = SapMintRecordView {
            mint_id: "deposit:1".into(),
            recipient_did: "did:mycelix:alice".into(),
            amount: 20,
            source: SapMintSource::CollateralBridge {
                deposit_id: "deposit:1".into(),
            },
        };
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 120,
            justified_by: Some("action:mint:1".into()),
        };
        assert_eq!(
            validate_positive_issuance_transition(
                &previous,
                &updated,
                "action:mint:1",
                &mint,
            ),
            Ok(())
        );
    }

    #[test]
    fn wrong_or_missing_justification_is_rejected() {
        let mint = SapMintRecordView {
            mint_id: "deposit:1".into(),
            recipient_did: "did:mycelix:alice".into(),
            amount: 20,
            source: SapMintSource::CollateralBridge {
                deposit_id: "deposit:1".into(),
            },
        };
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 120,
            justified_by: None,
        };
        assert_eq!(
            validate_positive_issuance_transition(
                &previous,
                &updated,
                "action:mint:1",
                &mint,
            ),
            Err(SapConservationError::JustificationMismatch)
        );
    }

    #[test]
    fn demurrage_and_issuance_cannot_be_mixed_into_one_opaque_delta() {
        let mint = SapMintRecordView {
            mint_id: "deposit:1".into(),
            recipient_did: "did:mycelix:alice".into(),
            amount: 20,
            source: SapMintSource::CollateralBridge {
                deposit_id: "deposit:1".into(),
            },
        };
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        // This could be "10 demurrage then +20 mint", but that combined operation
        // is intentionally not inferable/accepted by the issuance theorem.
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 110,
            justified_by: Some("action:mint:1".into()),
        };
        assert_eq!(
            validate_positive_issuance_transition(
                &previous,
                &updated,
                "action:mint:1",
                &mint,
            ),
            Err(SapConservationError::IssuanceDeltaMismatch)
        );
    }

    #[test]
    fn balance_overflow_fails_closed() {
        let mint = SapMintRecordView {
            mint_id: "deposit:1".into(),
            recipient_did: "did:mycelix:alice".into(),
            amount: 1,
            source: SapMintSource::CollateralBridge {
                deposit_id: "deposit:1".into(),
            },
        };
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: u64::MAX,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: u64::MAX,
            justified_by: Some("action:mint:1".into()),
        };
        assert_eq!(
            validate_positive_issuance_transition(
                &previous,
                &updated,
                "action:mint:1",
                &mint,
            ),
            Err(SapConservationError::BalanceOverflow)
        );
    }

    #[test]
    fn balance_member_cannot_change_during_issuance() {
        let mint = SapMintRecordView {
            mint_id: "deposit:1".into(),
            recipient_did: "did:mycelix:bob".into(),
            amount: 20,
            source: SapMintSource::CollateralBridge {
                deposit_id: "deposit:1".into(),
            },
        };
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:bob".into(),
            balance: 120,
            justified_by: Some("action:mint:1".into()),
        };
        assert_eq!(
            validate_positive_issuance_transition(
                &previous,
                &updated,
                "action:mint:1",
                &mint,
            ),
            Err(SapConservationError::BalanceMemberChanged)
        );
    }

    #[test]
    fn serialization_round_trip_preserves_mint_plan() {
        let intent = settlement_intent("deposit:1", "approved-provider");
        let plan = AuthorizedCollateralSapMintPlan::from_settlement_intent(
            &intent,
            &policy("approved-provider"),
        )
        .expect("plan");
        let encoded = serde_json::to_string(&plan).expect("serialize");
        let decoded: AuthorizedCollateralSapMintPlan =
            serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(decoded, plan);
    }
}

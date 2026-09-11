#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-007 request-bound SAP issuance and conservation theorems.
//!
//! The checked V2 path is deliberately staged:
//!
//! `bound settlement -> mint authorization -> mint record -> issuance receipt -> balance update`
//!
//! Each stage has zero balance effect until the final predecessor-bound balance
//! transition. Demurrage and issuance are separate transitions.

use finance_collateral_request_binding::{
    BoundCollateralSettlementIntent, RequestBindingError,
};
use finance_collateral_settlement::{
    CollateralSettlementAuthorityPolicy, PolicyBoundSettlementError,
};
use mycelix_finance_types::SapMintSource;
use serde::{Deserialize, Serialize};

pub const COLLATERAL_SAP_MINT_AUTHORIZATION_V2_SCHEMA_VERSION: u16 = 2;
pub const COLLATERAL_SAP_ISSUANCE_RECEIPT_V2_SCHEMA_VERSION: u16 = 2;
pub const MAX_MINT_ID_LEN: usize = 256;
pub const MAX_ACTION_REFERENCE_LEN: usize = 256;

/// Storage-independent view of the existing immutable Holochain `SapMintRecord`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SapMintRecordView {
    pub mint_id: String,
    pub recipient_did: String,
    pub amount: u64,
    pub source: SapMintSource,
}

impl SapMintRecordView {
    pub fn validate_shape(&self) -> Result<(), SapConservationError> {
        if !valid_reference(&self.mint_id, MAX_MINT_ID_LEN) {
            return Err(SapConservationError::InvalidMintId);
        }
        if !valid_did(&self.recipient_did) {
            return Err(SapConservationError::InvalidRecipientDid);
        }
        if self.amount == 0 {
            return Err(SapConservationError::ZeroMintAmount);
        }
        if let SapMintSource::CollateralBridge { deposit_id } = &self.source {
            if !valid_reference(deposit_id, MAX_MINT_ID_LEN) {
                return Err(SapConservationError::InvalidCollateralDepositId);
            }
        }
        Ok(())
    }
}

/// Immutable authorization for one collateral-backed SAP mint.
///
/// Unlike the legacy `CollateralBridge { deposit_id }` provenance alone, this
/// object retains the exact FIN-SAFE-009 request action reference by embedding
/// the bound settlement intent from which the mint was derived.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSapMintAuthorizationV2 {
    pub schema_version: u16,
    pub bound_settlement: BoundCollateralSettlementIntent,
    pub mint_id: String,
    pub recipient_did: String,
    pub amount: u64,
}

impl CollateralSapMintAuthorizationV2 {
    pub fn from_bound_settlement(
        bound_settlement: BoundCollateralSettlementIntent,
        expected_policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<Self, SapConservationError> {
        bound_settlement
            .validate()
            .map_err(SapConservationError::RequestBinding)?;
        bound_settlement
            .intent
            .validate_against_policy(expected_policy)
            .map_err(SapConservationError::SettlementPolicy)?;

        let deposit_id = bound_settlement.intent.basis.terms.deposit_id.clone();
        let authorization = Self {
            schema_version: COLLATERAL_SAP_MINT_AUTHORIZATION_V2_SCHEMA_VERSION,
            mint_id: canonical_collateral_mint_id(&deposit_id)?.to_string(),
            recipient_did: bound_settlement.intent.basis.terms.depositor_did.clone(),
            amount: bound_settlement.intent.basis.sap_amount,
            bound_settlement,
        };
        authorization.validate_against_policy(expected_policy)?;
        Ok(authorization)
    }

    pub fn validate_against_policy(
        &self,
        expected_policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<(), SapConservationError> {
        if self.schema_version != COLLATERAL_SAP_MINT_AUTHORIZATION_V2_SCHEMA_VERSION {
            return Err(SapConservationError::UnsupportedAuthorizationSchema);
        }
        self.bound_settlement
            .validate()
            .map_err(SapConservationError::RequestBinding)?;
        self.bound_settlement
            .intent
            .validate_against_policy(expected_policy)
            .map_err(SapConservationError::SettlementPolicy)?;

        let expected_mint_id = canonical_collateral_mint_id(self.deposit_id())?;
        if self.mint_id != expected_mint_id {
            return Err(SapConservationError::MintIdMismatch);
        }
        if self.recipient_did != self.bound_settlement.intent.basis.terms.depositor_did {
            return Err(SapConservationError::MintRecipientMismatch);
        }
        if self.amount != self.bound_settlement.intent.basis.sap_amount {
            return Err(SapConservationError::MintAmountMismatch);
        }
        Ok(())
    }

    pub fn request_action_reference(&self) -> &str {
        &self.bound_settlement.request_action_reference
    }

    pub fn deposit_id(&self) -> &str {
        self.bound_settlement.deposit_id()
    }

    pub fn expected_mint_record_view(&self) -> SapMintRecordView {
        SapMintRecordView {
            mint_id: self.mint_id.clone(),
            recipient_did: self.recipient_did.clone(),
            amount: self.amount,
            source: SapMintSource::CollateralBridge {
                deposit_id: self.deposit_id().to_string(),
            },
        }
    }

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
            SapMintSource::CollateralBridge { deposit_id } if deposit_id == self.deposit_id() => {
                Ok(())
            }
            _ => Err(SapConservationError::MintSourceMismatch),
        }
    }
}

/// Immutable proof object created only after the exact `SapMintRecord` action is
/// known. This is the object a checked positive balance delta should reference.
///
/// Holochain integration must load `mint_record_action_reference` with
/// `must_get_valid_record`, decode the exact immutable mint record, and ensure it
/// equals `mint_record` before persisting this receipt.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollateralSapIssuanceReceiptV2 {
    pub schema_version: u16,
    pub authorization: CollateralSapMintAuthorizationV2,
    pub mint_record_action_reference: String,
    pub mint_record: SapMintRecordView,
}

impl CollateralSapIssuanceReceiptV2 {
    pub fn new(
        authorization: CollateralSapMintAuthorizationV2,
        expected_policy: &CollateralSettlementAuthorityPolicy,
        mint_record_action_reference: String,
        mint_record: SapMintRecordView,
    ) -> Result<Self, SapConservationError> {
        let receipt = Self {
            schema_version: COLLATERAL_SAP_ISSUANCE_RECEIPT_V2_SCHEMA_VERSION,
            authorization,
            mint_record_action_reference,
            mint_record,
        };
        receipt.validate_against_policy(expected_policy)?;
        Ok(receipt)
    }

    pub fn validate_against_policy(
        &self,
        expected_policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<(), SapConservationError> {
        if self.schema_version != COLLATERAL_SAP_ISSUANCE_RECEIPT_V2_SCHEMA_VERSION {
            return Err(SapConservationError::UnsupportedIssuanceReceiptSchema);
        }
        if !valid_reference(
            &self.mint_record_action_reference,
            MAX_ACTION_REFERENCE_LEN,
        ) {
            return Err(SapConservationError::InvalidMintActionReference);
        }
        self.authorization.validate_against_policy(expected_policy)?;
        self.authorization.validate_mint_record(&self.mint_record)?;
        Ok(())
    }

    pub fn request_action_reference(&self) -> &str {
        self.authorization.request_action_reference()
    }

    pub fn deposit_id(&self) -> &str {
        self.authorization.deposit_id()
    }
}

/// Minimal SAP balance state used to prove one exact predecessor transition.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapBalanceStateView {
    pub member_did: String,
    pub balance: u64,
    /// On the checked V2 issuance path this points to the exact immutable
    /// `CollateralSapIssuanceReceiptV2` action, not merely the mint record.
    pub justified_by: Option<String>,
}

impl SapBalanceStateView {
    pub fn validate_shape(&self) -> Result<(), SapConservationError> {
        if !valid_did(&self.member_did) {
            return Err(SapConservationError::InvalidBalanceMemberDid);
        }
        if let Some(reference) = &self.justified_by {
            if !valid_reference(reference, MAX_ACTION_REFERENCE_LEN) {
                return Err(SapConservationError::InvalidJustificationReference);
            }
        }
        Ok(())
    }
}

/// Ordinary balance genesis is zero and carries no provenance pointer. A first
/// issuance is a separate predecessor-bound update.
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

/// Prove one positive collateral-backed issuance delta.
///
/// Demurrage MUST NOT be folded into this transition. The next raw balance is
/// exactly `previous.balance + authorization.amount` using checked addition.
/// `justified_by` must name the exact persisted issuance-receipt action.
pub fn validate_positive_collateral_issuance_transition(
    previous: &SapBalanceStateView,
    updated: &SapBalanceStateView,
    issuance_receipt_action_reference: &str,
    receipt: &CollateralSapIssuanceReceiptV2,
    expected_policy: &CollateralSettlementAuthorityPolicy,
) -> Result<(), SapConservationError> {
    previous.validate_shape()?;
    updated.validate_shape()?;
    receipt.validate_against_policy(expected_policy)?;

    if previous.member_did != updated.member_did {
        return Err(SapConservationError::BalanceMemberChanged);
    }
    if receipt.authorization.recipient_did != updated.member_did {
        return Err(SapConservationError::MintRecipientMismatch);
    }
    if !valid_reference(
        issuance_receipt_action_reference,
        MAX_ACTION_REFERENCE_LEN,
    ) {
        return Err(SapConservationError::InvalidIssuanceReceiptActionReference);
    }
    if updated.justified_by.as_deref() != Some(issuance_receipt_action_reference) {
        return Err(SapConservationError::JustificationMismatch);
    }

    let expected_balance = previous
        .balance
        .checked_add(receipt.authorization.amount)
        .ok_or(SapConservationError::BalanceOverflow)?;
    if updated.balance != expected_balance {
        return Err(SapConservationError::IssuanceDeltaMismatch);
    }
    Ok(())
}

/// Canonical logical mint ID remains the bounded V2 deposit ID. The separate
/// issuance receipt carries the exact request and mint action references.
pub fn canonical_collateral_mint_id(
    deposit_id: &str,
) -> Result<&str, SapConservationError> {
    if !valid_reference(deposit_id, MAX_MINT_ID_LEN) {
        return Err(SapConservationError::InvalidCollateralDepositId);
    }
    Ok(deposit_id)
}

fn valid_reference(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_did(value: &str) -> bool {
    value.starts_with("did:") && value.len() <= 256
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapConservationError {
    InvalidMintId,
    InvalidRecipientDid,
    ZeroMintAmount,
    InvalidCollateralDepositId,
    InvalidBalanceMemberDid,
    InvalidJustificationReference,
    InvalidMintActionReference,
    InvalidIssuanceReceiptActionReference,
    UnsupportedAuthorizationSchema,
    UnsupportedIssuanceReceiptSchema,
    RequestBinding(RequestBindingError),
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
    use finance_collateral_deposit::{
        CollateralDepositRequestV2, COLLATERAL_DEPOSIT_NONCE_BYTES,
    };
    use finance_collateral_request_binding::{
        derive_bound_settlement_intent, BoundCollateralDepositRequest,
    };
    use finance_collateral_settlement::{
        CollateralSettlementAuthorityPolicy, CustodyAttestationCapability,
        CustodyEvidenceEnvelope, CustodyEvidenceOutcome, PriceAttestationCapability,
        PriceEvidenceEnvelope, PriceEvidenceOutcome, PriceRatio, SettlementFreshnessPolicy,
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

    fn bound_settlement(
        request_action_reference: &str,
        provider: &str,
        nonce_byte: u8,
    ) -> BoundCollateralSettlementIntent {
        let p = policy(provider);
        let request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            10,
            [nonce_byte; COLLATERAL_DEPOSIT_NONCE_BYTES],
            9,
        )
        .expect("request");
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
            deposit_id: request.deposit_id.clone(),
            depositor_did: request.depositor_did.clone(),
            collateral_asset_id: request.collateral_asset_id.clone(),
            collateral_amount: request.collateral_amount,
            capability: p.custody_capability.clone(),
            outcome: CustodyEvidenceOutcome::Confirmed {
                external_reference: "custody:proof:1".into(),
                confirmed_at_micros: 10,
            },
        };
        derive_bound_settlement_intent(
            BoundCollateralDepositRequest {
                request_action_reference: request_action_reference.into(),
                request,
            },
            &p,
            &price,
            &custody,
            11,
        )
        .expect("bound settlement")
    }

    fn authorization() -> CollateralSapMintAuthorizationV2 {
        CollateralSapMintAuthorizationV2::from_bound_settlement(
            bound_settlement("uhCkk-request-exact", "approved-provider", 1),
            &policy("approved-provider"),
        )
        .expect("authorization")
    }

    fn receipt() -> CollateralSapIssuanceReceiptV2 {
        let auth = authorization();
        CollateralSapIssuanceReceiptV2::new(
            auth.clone(),
            &policy("approved-provider"),
            "uhCkk-mint-action".into(),
            auth.expected_mint_record_view(),
        )
        .expect("issuance receipt")
    }

    #[test]
    fn authorization_retains_exact_request_action_reference() {
        let auth = authorization();
        assert_eq!(auth.request_action_reference(), "uhCkk-request-exact");
        assert_eq!(auth.amount, 20);
        assert_eq!(auth.validate_against_policy(&policy("approved-provider")), Ok(()));
    }

    #[test]
    fn alternate_provider_policy_cannot_authorize_same_bound_settlement() {
        let bound = bound_settlement("uhCkk-request-exact", "alternate-provider", 2);
        let result = CollateralSapMintAuthorizationV2::from_bound_settlement(
            bound,
            &policy("approved-provider"),
        );
        assert!(matches!(
            result,
            Err(SapConservationError::SettlementPolicy(
                PolicyBoundSettlementError::AuthorityPolicyMismatch
            ))
        ));
    }

    #[test]
    fn tampered_authorization_amount_is_rejected() {
        let mut auth = authorization();
        auth.amount += 1;
        assert_eq!(
            auth.validate_against_policy(&policy("approved-provider")),
            Err(SapConservationError::MintAmountMismatch)
        );
    }

    #[test]
    fn mint_record_must_match_authorization_exactly() {
        let auth = authorization();
        let valid = auth.expected_mint_record_view();
        assert_eq!(auth.validate_mint_record(&valid), Ok(()));

        let mut wrong = valid;
        wrong.source = SapMintSource::GovernanceProposal {
            proposal_id: "proposal:1".into(),
        };
        assert_eq!(
            auth.validate_mint_record(&wrong),
            Err(SapConservationError::MintSourceMismatch)
        );
    }

    #[test]
    fn issuance_receipt_binds_request_and_mint_action_references() {
        let receipt = receipt();
        assert_eq!(receipt.request_action_reference(), "uhCkk-request-exact");
        assert_eq!(receipt.mint_record_action_reference, "uhCkk-mint-action");
        assert_eq!(
            receipt.validate_against_policy(&policy("approved-provider")),
            Ok(())
        );
    }

    #[test]
    fn ordinary_genesis_balance_is_zero_and_unjustified() {
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
    fn positive_delta_requires_exact_issuance_receipt_justification() {
        let receipt = receipt();
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 120,
            justified_by: Some("uhCkk-issuance-receipt".into()),
        };
        assert_eq!(
            validate_positive_collateral_issuance_transition(
                &previous,
                &updated,
                "uhCkk-issuance-receipt",
                &receipt,
                &policy("approved-provider"),
            ),
            Ok(())
        );
    }

    #[test]
    fn mixed_demurrage_and_mint_delta_is_rejected() {
        let receipt = receipt();
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 110,
            justified_by: Some("uhCkk-issuance-receipt".into()),
        };
        assert_eq!(
            validate_positive_collateral_issuance_transition(
                &previous,
                &updated,
                "uhCkk-issuance-receipt",
                &receipt,
                &policy("approved-provider"),
            ),
            Err(SapConservationError::IssuanceDeltaMismatch)
        );
    }

    #[test]
    fn wrong_justification_reference_is_rejected() {
        let receipt = receipt();
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 120,
            justified_by: Some("uhCkk-other".into()),
        };
        assert_eq!(
            validate_positive_collateral_issuance_transition(
                &previous,
                &updated,
                "uhCkk-issuance-receipt",
                &receipt,
                &policy("approved-provider"),
            ),
            Err(SapConservationError::JustificationMismatch)
        );
    }

    #[test]
    fn checked_balance_overflow_fails_closed() {
        let receipt = receipt();
        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: u64::MAX - 10,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: u64::MAX,
            justified_by: Some("uhCkk-issuance-receipt".into()),
        };
        assert_eq!(
            validate_positive_collateral_issuance_transition(
                &previous,
                &updated,
                "uhCkk-issuance-receipt",
                &receipt,
                &policy("approved-provider"),
            ),
            Err(SapConservationError::BalanceOverflow)
        );
    }

    #[test]
    fn serialization_round_trip_preserves_full_provenance_chain() {
        let receipt = receipt();
        let bytes = serde_json::to_vec(&receipt).expect("serialize");
        let decoded: CollateralSapIssuanceReceiptV2 =
            serde_json::from_slice(&bytes).expect("deserialize");
        assert_eq!(decoded, receipt);
        assert_eq!(decoded.request_action_reference(), "uhCkk-request-exact");
        assert_eq!(decoded.mint_record_action_reference, "uhCkk-mint-action");
    }
}

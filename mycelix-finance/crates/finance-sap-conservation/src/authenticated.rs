#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Authenticated FIN-SAFE-007/011 issuance layer.
//!
//! The lower-level conservation kernel remains available unchanged. This wrapper
//! requires the FIN-SAFE-011 authenticated settlement lineage before collateral
//! mint authorization can become eligible for persistence.

#[path = "lib.rs"]
mod base;

pub use base::*;

use finance_collateral_auth::CollateralSettlementTrustRootV1;
use finance_collateral_request_binding::{
    AuthenticatedBoundCollateralSettlementIntentV1, AuthenticatedRequestBindingError,
};
use serde::{Deserialize, Serialize};

/// Mint authorization that retains the exact request + price + custody action
/// lineage used to derive the lower-level FIN-SAFE-007 authorization.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticatedCollateralSapMintAuthorizationV2 {
    pub authenticated_settlement: AuthenticatedBoundCollateralSettlementIntentV1,
    pub authorization: CollateralSapMintAuthorizationV2,
}

impl AuthenticatedCollateralSapMintAuthorizationV2 {
    pub fn from_authenticated_settlement(
        authenticated_settlement: AuthenticatedBoundCollateralSettlementIntentV1,
        root: &CollateralSettlementTrustRootV1,
    ) -> Result<Self, AuthenticatedSapConservationError> {
        authenticated_settlement
            .validate_against_root(root)
            .map_err(AuthenticatedSapConservationError::AuthenticatedSettlement)?;

        let authorization = CollateralSapMintAuthorizationV2::from_bound_settlement(
            authenticated_settlement.bound_settlement.clone(),
            &root.settlement_policy,
        )
        .map_err(AuthenticatedSapConservationError::Conservation)?;

        let value = Self {
            authenticated_settlement,
            authorization,
        };
        value.validate_against_root(root)?;
        Ok(value)
    }

    pub fn validate_against_root(
        &self,
        root: &CollateralSettlementTrustRootV1,
    ) -> Result<(), AuthenticatedSapConservationError> {
        self.authenticated_settlement
            .validate_against_root(root)
            .map_err(AuthenticatedSapConservationError::AuthenticatedSettlement)?;
        self.authorization
            .validate_against_policy(&root.settlement_policy)
            .map_err(AuthenticatedSapConservationError::Conservation)?;
        if self.authorization.bound_settlement != self.authenticated_settlement.bound_settlement {
            return Err(AuthenticatedSapConservationError::AuthorizationSettlementMismatch);
        }
        Ok(())
    }

    pub fn request_action_reference(&self) -> &str {
        self.authenticated_settlement.request_action_reference()
    }

    pub fn price_attestation_action_reference(&self) -> &str {
        &self
            .authenticated_settlement
            .price_attestation_action_reference
    }

    pub fn custody_attestation_action_reference(&self) -> &str {
        &self
            .authenticated_settlement
            .custody_attestation_action_reference
    }
}

/// Final authenticated issuance receipt. The lower-level receipt retains the
/// exact authorization + mint actions, while `authenticated_authorization`
/// retains the exact request + price + custody actions and DNA-root identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticatedCollateralSapIssuanceReceiptV2 {
    pub authenticated_authorization: AuthenticatedCollateralSapMintAuthorizationV2,
    pub receipt: CollateralSapIssuanceReceiptV2,
}

impl AuthenticatedCollateralSapIssuanceReceiptV2 {
    pub fn new(
        authorization_action_reference: String,
        authenticated_authorization: AuthenticatedCollateralSapMintAuthorizationV2,
        root: &CollateralSettlementTrustRootV1,
        mint_record_action_reference: String,
        mint_record: CollateralSapMintRecordV2,
    ) -> Result<Self, AuthenticatedSapConservationError> {
        authenticated_authorization.validate_against_root(root)?;
        let receipt = CollateralSapIssuanceReceiptV2::new(
            authorization_action_reference,
            authenticated_authorization.authorization.clone(),
            &root.settlement_policy,
            mint_record_action_reference,
            mint_record,
        )
        .map_err(AuthenticatedSapConservationError::Conservation)?;

        let value = Self {
            authenticated_authorization,
            receipt,
        };
        value.validate_against_root(root)?;
        Ok(value)
    }

    pub fn validate_against_root(
        &self,
        root: &CollateralSettlementTrustRootV1,
    ) -> Result<(), AuthenticatedSapConservationError> {
        self.authenticated_authorization.validate_against_root(root)?;
        self.receipt
            .validate_against_policy(&root.settlement_policy)
            .map_err(AuthenticatedSapConservationError::Conservation)?;
        if self.receipt.authorization != self.authenticated_authorization.authorization {
            return Err(AuthenticatedSapConservationError::ReceiptAuthorizationMismatch);
        }
        Ok(())
    }

    pub fn request_action_reference(&self) -> &str {
        self.authenticated_authorization.request_action_reference()
    }

    pub fn price_attestation_action_reference(&self) -> &str {
        self.authenticated_authorization
            .price_attestation_action_reference()
    }

    pub fn custody_attestation_action_reference(&self) -> &str {
        self.authenticated_authorization
            .custody_attestation_action_reference()
    }

    pub fn amount(&self) -> u64 {
        self.receipt.amount()
    }
}

/// Checked positive balance delta under the full FIN-SAFE-011 trust root.
pub fn validate_authenticated_positive_collateral_issuance_transition(
    previous: &SapBalanceStateView,
    updated: &SapBalanceStateView,
    issuance_receipt_action_reference: &str,
    receipt: &AuthenticatedCollateralSapIssuanceReceiptV2,
    root: &CollateralSettlementTrustRootV1,
) -> Result<(), AuthenticatedSapConservationError> {
    receipt.validate_against_root(root)?;
    validate_positive_collateral_issuance_transition(
        previous,
        updated,
        issuance_receipt_action_reference,
        &receipt.receipt,
        &root.settlement_policy,
    )
    .map_err(AuthenticatedSapConservationError::Conservation)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthenticatedSapConservationError {
    AuthenticatedSettlement(AuthenticatedRequestBindingError),
    Conservation(SapConservationError),
    AuthorizationSettlementMismatch,
    ReceiptAuthorizationMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_auth::{
        CollateralSettlementTrustRootV1, CustodyAttestationV1, PriceAttestationV1,
        COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION, CUSTODY_ATTESTATION_V1_SCHEMA_VERSION,
        PRICE_ATTESTATION_V1_SCHEMA_VERSION,
    };
    use finance_collateral_deposit::{
        CollateralDepositRequestV2, COLLATERAL_DEPOSIT_NONCE_BYTES,
    };
    use finance_collateral_request_binding::{
        derive_authenticated_bound_settlement_intent_from_valid_actions,
        BoundCollateralDepositRequest,
    };
    use finance_collateral_settlement::{
        CollateralSettlementAuthorityPolicy, CustodyAttestationCapability,
        PriceAttestationCapability, PriceRatio, SettlementFreshnessPolicy,
        COLLATERAL_SETTLEMENT_PROTOCOL_VERSION, CUSTODY_ATTESTATION_PROTOCOL_VERSION,
        PRICE_ATTESTATION_PROTOCOL_VERSION, SAP_ASSET_ID,
    };

    fn root() -> CollateralSettlementTrustRootV1 {
        CollateralSettlementTrustRootV1 {
            root_id: "root-v1".into(),
            root_version: 1,
            authentication_protocol_version: COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION,
            settlement_policy: CollateralSettlementAuthorityPolicy {
                policy_id: "policy-v1".into(),
                policy_version: 1,
                settlement_protocol_version: COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
                quote_asset_id: SAP_ASSET_ID.into(),
                price_capability: PriceAttestationCapability {
                    provider_id: "did:mycelix:price".into(),
                    method: "holochain-price-attestation-v1".into(),
                    protocol_version: PRICE_ATTESTATION_PROTOCOL_VERSION,
                },
                custody_capability: CustodyAttestationCapability {
                    custodian_id: "did:mycelix:custody".into(),
                    method: "holochain-custody-attestation-v1".into(),
                    protocol_version: CUSTODY_ATTESTATION_PROTOCOL_VERSION,
                },
                price_freshness: SettlementFreshnessPolicy {
                    policy_id: "price".into(),
                    policy_version: 1,
                    max_age_micros: 100,
                },
                custody_freshness: SettlementFreshnessPolicy {
                    policy_id: "custody".into(),
                    policy_version: 1,
                    max_age_micros: 100,
                },
            },
            price_authority_did: "did:mycelix:price".into(),
            custody_authority_did: "did:mycelix:custody".into(),
        }
    }

    fn authenticated_settlement(
        root: &CollateralSettlementTrustRootV1,
    ) -> AuthenticatedBoundCollateralSettlementIntentV1 {
        let request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            10,
            [7; COLLATERAL_DEPOSIT_NONCE_BYTES],
            1,
        )
        .expect("request");
        let price = PriceAttestationV1 {
            schema_version: PRICE_ATTESTATION_V1_SCHEMA_VERSION,
            base_asset_id: "ETH".into(),
            quote_asset_id: SAP_ASSET_ID.into(),
            capability: root.settlement_policy.price_capability.clone(),
            rate: PriceRatio {
                numerator: 2,
                denominator: 1,
            },
            observation_reference: "price:1".into(),
            observed_at_micros: 10,
        };
        let custody = CustodyAttestationV1 {
            schema_version: CUSTODY_ATTESTATION_V1_SCHEMA_VERSION,
            deposit_id: request.deposit_id.clone(),
            depositor_did: request.depositor_did.clone(),
            collateral_asset_id: request.collateral_asset_id.clone(),
            collateral_amount: request.collateral_amount,
            capability: root.settlement_policy.custody_capability.clone(),
            external_reference: "custody:1".into(),
            confirmed_at_micros: 10,
        };
        derive_authenticated_bound_settlement_intent_from_valid_actions(
            BoundCollateralDepositRequest {
                request_action_reference: "uhCkk-request".into(),
                request,
            },
            root,
            "uhCkk-price".into(),
            root.price_authority_did.clone(),
            11,
            price,
            "uhCkk-custody".into(),
            root.custody_authority_did.clone(),
            11,
            custody,
            12,
        )
        .expect("authenticated settlement")
    }

    #[test]
    fn mint_authorization_retains_all_exact_evidence_actions() {
        let root = root();
        let authorization = AuthenticatedCollateralSapMintAuthorizationV2::from_authenticated_settlement(
            authenticated_settlement(&root),
            &root,
        )
        .expect("authorization");
        assert_eq!(authorization.request_action_reference(), "uhCkk-request");
        assert_eq!(authorization.price_attestation_action_reference(), "uhCkk-price");
        assert_eq!(authorization.custody_attestation_action_reference(), "uhCkk-custody");
        assert_eq!(authorization.validate_against_root(&root), Ok(()));
    }

    #[test]
    fn authenticated_receipt_preserves_lineage_through_balance_delta() {
        let root = root();
        let authorization = AuthenticatedCollateralSapMintAuthorizationV2::from_authenticated_settlement(
            authenticated_settlement(&root),
            &root,
        )
        .expect("authorization");
        let mint = CollateralSapMintRecordV2::from_authorization(
            "uhCkk-auth".into(),
            &authorization.authorization,
            &root.settlement_policy,
        )
        .expect("mint");
        let receipt = AuthenticatedCollateralSapIssuanceReceiptV2::new(
            "uhCkk-auth".into(),
            authorization,
            &root,
            "uhCkk-mint".into(),
            mint,
        )
        .expect("receipt");

        let previous = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 100,
            justified_by: None,
        };
        let updated = SapBalanceStateView {
            member_did: "did:mycelix:alice".into(),
            balance: 120,
            justified_by: Some("uhCkk-receipt".into()),
        };
        assert_eq!(
            validate_authenticated_positive_collateral_issuance_transition(
                &previous,
                &updated,
                "uhCkk-receipt",
                &receipt,
                &root,
            ),
            Ok(())
        );
    }

    #[test]
    fn alternate_root_cannot_validate_existing_authorization() {
        let root = root();
        let authorization = AuthenticatedCollateralSapMintAuthorizationV2::from_authenticated_settlement(
            authenticated_settlement(&root),
            &root,
        )
        .expect("authorization");
        let mut alternate = root.clone();
        alternate.root_version = 2;
        assert_eq!(
            authorization.validate_against_root(&alternate),
            Err(AuthenticatedSapConservationError::AuthenticatedSettlement(
                AuthenticatedRequestBindingError::TrustRootMismatch
            ))
        );
    }
}

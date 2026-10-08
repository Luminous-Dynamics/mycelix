#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Authenticated extension of FIN-SAFE-009 request binding.
//!
//! The lower-level request-binding API remains available unchanged. This layer
//! additionally retains the exact price/custody attestation action references and
//! action metadata used to derive settlement. Holochain integration must load the
//! referenced actions with `must_get_valid_record` and supply their actual author
//! and timestamp; serialized values are not authority on their own.

#[path = "lib.rs"]
mod base;

pub use base::*;

use finance_collateral_auth::{
    CollateralEvidenceAuthError, CollateralSettlementTrustRootV1, CustodyAttestationV1,
    PriceAttestationV1,
};
use serde::{Deserialize, Serialize};

pub const MAX_EVIDENCE_ACTION_REFERENCE_LEN: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticatedBoundCollateralSettlementIntentV1 {
    pub bound_settlement: BoundCollateralSettlementIntent,
    pub trust_root_id: String,
    pub trust_root_version: u16,
    pub price_attestation_action_reference: String,
    pub price_action_author_did: String,
    pub price_action_timestamp_micros: i64,
    pub price_attestation: PriceAttestationV1,
    pub custody_attestation_action_reference: String,
    pub custody_action_author_did: String,
    pub custody_action_timestamp_micros: i64,
    pub custody_attestation: CustodyAttestationV1,
}

impl AuthenticatedBoundCollateralSettlementIntentV1 {
    pub fn validate_against_root(
        &self,
        root: &CollateralSettlementTrustRootV1,
    ) -> Result<(), AuthenticatedRequestBindingError> {
        root.validate()
            .map_err(AuthenticatedRequestBindingError::Authentication)?;
        if self.trust_root_id != root.root_id || self.trust_root_version != root.root_version {
            return Err(AuthenticatedRequestBindingError::TrustRootMismatch);
        }
        validate_evidence_reference(&self.price_attestation_action_reference)
            .map_err(|_| AuthenticatedRequestBindingError::InvalidPriceActionReference)?;
        validate_evidence_reference(&self.custody_attestation_action_reference)
            .map_err(|_| AuthenticatedRequestBindingError::InvalidCustodyActionReference)?;

        self.bound_settlement
            .validate()
            .map_err(AuthenticatedRequestBindingError::RequestBinding)?;

        let price_evidence = self
            .price_attestation
            .to_evidence_from_valid_action(
                root,
                &self.price_action_author_did,
                self.price_action_timestamp_micros,
            )
            .map_err(AuthenticatedRequestBindingError::Authentication)?;
        let custody_evidence = self
            .custody_attestation
            .to_evidence_from_valid_action(
                root,
                &self.custody_action_author_did,
                self.custody_action_timestamp_micros,
            )
            .map_err(AuthenticatedRequestBindingError::Authentication)?;

        let expected = derive_bound_settlement_intent(
            BoundCollateralDepositRequest {
                request_action_reference: self
                    .bound_settlement
                    .request_action_reference
                    .clone(),
                request: self.bound_settlement.request.clone(),
            },
            &root.settlement_policy,
            &price_evidence,
            &custody_evidence,
            self.bound_settlement.intent.authorized_at_micros,
        )
        .map_err(AuthenticatedRequestBindingError::RequestBinding)?;

        if expected != self.bound_settlement {
            return Err(AuthenticatedRequestBindingError::AuthenticatedSettlementMismatch);
        }
        Ok(())
    }

    pub fn request_action_reference(&self) -> &str {
        &self.bound_settlement.request_action_reference
    }

    pub fn deposit_id(&self) -> &str {
        self.bound_settlement.deposit_id()
    }
}

#[allow(clippy::too_many_arguments)]
pub fn derive_authenticated_bound_settlement_intent_from_valid_actions(
    bound_request: BoundCollateralDepositRequest,
    root: &CollateralSettlementTrustRootV1,
    price_attestation_action_reference: String,
    price_action_author_did: String,
    price_action_timestamp_micros: i64,
    price_attestation: PriceAttestationV1,
    custody_attestation_action_reference: String,
    custody_action_author_did: String,
    custody_action_timestamp_micros: i64,
    custody_attestation: CustodyAttestationV1,
    evaluated_at_micros: i64,
) -> Result<AuthenticatedBoundCollateralSettlementIntentV1, AuthenticatedRequestBindingError> {
    root.validate()
        .map_err(AuthenticatedRequestBindingError::Authentication)?;
    validate_evidence_reference(&price_attestation_action_reference)
        .map_err(|_| AuthenticatedRequestBindingError::InvalidPriceActionReference)?;
    validate_evidence_reference(&custody_attestation_action_reference)
        .map_err(|_| AuthenticatedRequestBindingError::InvalidCustodyActionReference)?;

    let price_evidence = price_attestation
        .to_evidence_from_valid_action(
            root,
            &price_action_author_did,
            price_action_timestamp_micros,
        )
        .map_err(AuthenticatedRequestBindingError::Authentication)?;
    let custody_evidence = custody_attestation
        .to_evidence_from_valid_action(
            root,
            &custody_action_author_did,
            custody_action_timestamp_micros,
        )
        .map_err(AuthenticatedRequestBindingError::Authentication)?;

    let bound_settlement = derive_bound_settlement_intent(
        bound_request,
        &root.settlement_policy,
        &price_evidence,
        &custody_evidence,
        evaluated_at_micros,
    )
    .map_err(AuthenticatedRequestBindingError::RequestBinding)?;

    let authenticated = AuthenticatedBoundCollateralSettlementIntentV1 {
        bound_settlement,
        trust_root_id: root.root_id.clone(),
        trust_root_version: root.root_version,
        price_attestation_action_reference,
        price_action_author_did,
        price_action_timestamp_micros,
        price_attestation,
        custody_attestation_action_reference,
        custody_action_author_did,
        custody_action_timestamp_micros,
        custody_attestation,
    };
    authenticated.validate_against_root(root)?;
    Ok(authenticated)
}

fn validate_evidence_reference(reference: &str) -> Result<(), ()> {
    if reference.is_empty() || reference.len() > MAX_EVIDENCE_ACTION_REFERENCE_LEN {
        Err(())
    } else {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthenticatedRequestBindingError {
    InvalidPriceActionReference,
    InvalidCustodyActionReference,
    TrustRootMismatch,
    Authentication(CollateralEvidenceAuthError),
    RequestBinding(RequestBindingError),
    AuthenticatedSettlementMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_auth::{
        COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION, CUSTODY_ATTESTATION_V1_SCHEMA_VERSION,
        PRICE_ATTESTATION_V1_SCHEMA_VERSION,
    };
    use finance_collateral_deposit::{
        CollateralDepositRequestV2, COLLATERAL_DEPOSIT_NONCE_BYTES,
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

    fn request() -> CollateralDepositRequestV2 {
        CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            10,
            [7; COLLATERAL_DEPOSIT_NONCE_BYTES],
            1,
        )
        .expect("request")
    }

    fn price(root: &CollateralSettlementTrustRootV1) -> PriceAttestationV1 {
        PriceAttestationV1 {
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
        }
    }

    fn custody(
        root: &CollateralSettlementTrustRootV1,
        request: &CollateralDepositRequestV2,
    ) -> CustodyAttestationV1 {
        CustodyAttestationV1 {
            schema_version: CUSTODY_ATTESTATION_V1_SCHEMA_VERSION,
            deposit_id: request.deposit_id.clone(),
            depositor_did: request.depositor_did.clone(),
            collateral_asset_id: request.collateral_asset_id.clone(),
            collateral_amount: request.collateral_amount,
            capability: root.settlement_policy.custody_capability.clone(),
            external_reference: "custody:1".into(),
            confirmed_at_micros: 10,
        }
    }

    #[test]
    fn exact_evidence_actions_are_retained_end_to_end() {
        let root = root();
        let request = request();
        let result = derive_authenticated_bound_settlement_intent_from_valid_actions(
            BoundCollateralDepositRequest {
                request_action_reference: "uhCkk-request".into(),
                request: request.clone(),
            },
            &root,
            "uhCkk-price".into(),
            root.price_authority_did.clone(),
            11,
            price(&root),
            "uhCkk-custody".into(),
            root.custody_authority_did.clone(),
            11,
            custody(&root, &request),
            12,
        )
        .expect("authenticated settlement");

        assert_eq!(result.request_action_reference(), "uhCkk-request");
        assert_eq!(result.price_attestation_action_reference, "uhCkk-price");
        assert_eq!(result.custody_attestation_action_reference, "uhCkk-custody");
        assert_eq!(result.validate_against_root(&root), Ok(()));
    }

    #[test]
    fn evidence_action_reference_tampering_is_rejected() {
        let root = root();
        let request = request();
        let mut result = derive_authenticated_bound_settlement_intent_from_valid_actions(
            BoundCollateralDepositRequest {
                request_action_reference: "uhCkk-request".into(),
                request: request.clone(),
            },
            &root,
            "uhCkk-price".into(),
            root.price_authority_did.clone(),
            11,
            price(&root),
            "uhCkk-custody".into(),
            root.custody_authority_did.clone(),
            11,
            custody(&root, &request),
            12,
        )
        .expect("authenticated settlement");
        result.price_attestation_action_reference.clear();
        assert_eq!(
            result.validate_against_root(&root),
            Err(AuthenticatedRequestBindingError::InvalidPriceActionReference)
        );
    }

    #[test]
    fn trust_root_version_tampering_is_rejected() {
        let root = root();
        let request = request();
        let mut result = derive_authenticated_bound_settlement_intent_from_valid_actions(
            BoundCollateralDepositRequest {
                request_action_reference: "uhCkk-request".into(),
                request: request.clone(),
            },
            &root,
            "uhCkk-price".into(),
            root.price_authority_did.clone(),
            11,
            price(&root),
            "uhCkk-custody".into(),
            root.custody_authority_did.clone(),
            11,
            custody(&root, &request),
            12,
        )
        .expect("authenticated settlement");
        result.trust_root_version += 1;
        assert_eq!(
            result.validate_against_root(&root),
            Err(AuthenticatedRequestBindingError::TrustRootMismatch)
        );
    }
}

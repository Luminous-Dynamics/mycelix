#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-010 compact persistence records for collateral-backed SAP issuance.
//!
//! These records intentionally persist references + derived facts, not a copy of
//! the entire request/evidence/settlement graph. Holochain integrity must reload
//! exact referenced actions and reconstruct the authenticated theorem before
//! accepting a record as authoritative.

use finance_collateral_auth::CollateralSettlementTrustRootV1;
use finance_sap_conservation::{
    AuthenticatedCollateralSapMintAuthorizationV2, AuthenticatedSapConservationError,
};
use serde::{Deserialize, Serialize};

pub const COLLATERAL_MINT_AUTH_RECORD_V2_SCHEMA_VERSION: u16 = 1;
pub const COLLATERAL_MINT_RECORD_V2_SCHEMA_VERSION: u16 = 1;
pub const COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION: u16 = 1;
pub const MAX_ACTION_REFERENCE_LEN: usize = 256;
pub const MAX_ID_LEN: usize = 256;
pub const MAX_DID_LEN: usize = 256;

/// Compact persisted authorization facts.
///
/// Authority does not come from this structure being internally consistent. The
/// integrity adapter must reconstruct `AuthenticatedCollateralSapMintAuthorizationV2`
/// from the exact referenced request/price/custody actions and DNA trust root,
/// then call `validate_against_authenticated`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSapMintAuthorizationRecordV2 {
    pub schema_version: u16,
    pub trust_root_id: String,
    pub trust_root_version: u16,
    pub request_action_reference: String,
    pub price_attestation_action_reference: String,
    pub custody_attestation_action_reference: String,
    pub deposit_id: String,
    pub mint_id: String,
    pub recipient_did: String,
    pub amount: u64,
    pub authorized_at_micros: i64,
}

impl CollateralSapMintAuthorizationRecordV2 {
    pub fn from_authenticated(
        authorization: &AuthenticatedCollateralSapMintAuthorizationV2,
        root: &CollateralSettlementTrustRootV1,
    ) -> Result<Self, IssuancePersistenceError> {
        authorization
            .validate_against_root(root)
            .map_err(IssuancePersistenceError::AuthenticatedConservation)?;
        let record = Self {
            schema_version: COLLATERAL_MINT_AUTH_RECORD_V2_SCHEMA_VERSION,
            trust_root_id: root.root_id.clone(),
            trust_root_version: root.root_version,
            request_action_reference: authorization.request_action_reference().to_string(),
            price_attestation_action_reference: authorization
                .price_attestation_action_reference()
                .to_string(),
            custody_attestation_action_reference: authorization
                .custody_attestation_action_reference()
                .to_string(),
            deposit_id: authorization.authorization.deposit_id().to_string(),
            mint_id: authorization.authorization.mint_id.clone(),
            recipient_did: authorization.authorization.recipient_did.clone(),
            amount: authorization.authorization.amount,
            authorized_at_micros: authorization
                .authenticated_settlement
                .bound_settlement
                .intent
                .authorized_at_micros,
        };
        record.validate_against_authenticated(authorization, root)?;
        Ok(record)
    }

    pub fn validate_shape(&self) -> Result<(), IssuancePersistenceError> {
        if self.schema_version != COLLATERAL_MINT_AUTH_RECORD_V2_SCHEMA_VERSION {
            return Err(IssuancePersistenceError::UnsupportedAuthorizationRecordSchema);
        }
        for reference in [
            &self.request_action_reference,
            &self.price_attestation_action_reference,
            &self.custody_attestation_action_reference,
        ] {
            if !valid_reference(reference) {
                return Err(IssuancePersistenceError::InvalidActionReference);
            }
        }
        if !valid_id(&self.trust_root_id) || self.trust_root_version == 0 {
            return Err(IssuancePersistenceError::InvalidTrustRootReference);
        }
        if !valid_id(&self.deposit_id) || !valid_id(&self.mint_id) {
            return Err(IssuancePersistenceError::InvalidIssuanceId);
        }
        if !valid_did(&self.recipient_did) {
            return Err(IssuancePersistenceError::InvalidRecipientDid);
        }
        if self.amount == 0 {
            return Err(IssuancePersistenceError::ZeroAmount);
        }
        Ok(())
    }

    pub fn validate_against_authenticated(
        &self,
        authorization: &AuthenticatedCollateralSapMintAuthorizationV2,
        root: &CollateralSettlementTrustRootV1,
    ) -> Result<(), IssuancePersistenceError> {
        self.validate_shape()?;
        authorization
            .validate_against_root(root)
            .map_err(IssuancePersistenceError::AuthenticatedConservation)?;
        if self.trust_root_id != root.root_id || self.trust_root_version != root.root_version {
            return Err(IssuancePersistenceError::TrustRootMismatch);
        }
        if self.request_action_reference != authorization.request_action_reference()
            || self.price_attestation_action_reference
                != authorization.price_attestation_action_reference()
            || self.custody_attestation_action_reference
                != authorization.custody_attestation_action_reference()
        {
            return Err(IssuancePersistenceError::EvidenceReferenceMismatch);
        }
        if self.deposit_id != authorization.authorization.deposit_id()
            || self.mint_id != authorization.authorization.mint_id
            || self.recipient_did != authorization.authorization.recipient_did
            || self.amount != authorization.authorization.amount
            || self.authorized_at_micros
                != authorization
                    .authenticated_settlement
                    .bound_settlement
                    .intent
                    .authorized_at_micros
        {
            return Err(IssuancePersistenceError::AuthorizationFactsMismatch);
        }
        Ok(())
    }
}

/// Compact V2 collateral mint. Creation itself has zero balance effect.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSapMintRecordV2Compact {
    pub schema_version: u16,
    pub authorization_action_reference: String,
    pub request_action_reference: String,
    pub deposit_id: String,
    pub mint_id: String,
    pub recipient_did: String,
    pub amount: u64,
}

impl CollateralSapMintRecordV2Compact {
    pub fn from_authorization_record(
        authorization_action_reference: String,
        authorization: &CollateralSapMintAuthorizationRecordV2,
    ) -> Result<Self, IssuancePersistenceError> {
        authorization.validate_shape()?;
        if !valid_reference(&authorization_action_reference) {
            return Err(IssuancePersistenceError::InvalidActionReference);
        }
        let record = Self {
            schema_version: COLLATERAL_MINT_RECORD_V2_SCHEMA_VERSION,
            authorization_action_reference,
            request_action_reference: authorization.request_action_reference.clone(),
            deposit_id: authorization.deposit_id.clone(),
            mint_id: authorization.mint_id.clone(),
            recipient_did: authorization.recipient_did.clone(),
            amount: authorization.amount,
        };
        record.validate_against_authorization(authorization)?;
        Ok(record)
    }

    pub fn validate_against_authorization(
        &self,
        authorization: &CollateralSapMintAuthorizationRecordV2,
    ) -> Result<(), IssuancePersistenceError> {
        if self.schema_version != COLLATERAL_MINT_RECORD_V2_SCHEMA_VERSION {
            return Err(IssuancePersistenceError::UnsupportedMintRecordSchema);
        }
        if !valid_reference(&self.authorization_action_reference) {
            return Err(IssuancePersistenceError::InvalidActionReference);
        }
        authorization.validate_shape()?;
        if self.request_action_reference != authorization.request_action_reference
            || self.deposit_id != authorization.deposit_id
            || self.mint_id != authorization.mint_id
            || self.recipient_did != authorization.recipient_did
            || self.amount != authorization.amount
        {
            return Err(IssuancePersistenceError::MintFactsMismatch);
        }
        Ok(())
    }
}

/// Compact immutable receipt joining one exact authorization action and one exact
/// V2 mint action. Receipt creation still has zero balance effect.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSapIssuanceReceiptRecordV2 {
    pub schema_version: u16,
    pub authorization_action_reference: String,
    pub mint_action_reference: String,
    pub trust_root_id: String,
    pub trust_root_version: u16,
    pub request_action_reference: String,
    pub price_attestation_action_reference: String,
    pub custody_attestation_action_reference: String,
    pub deposit_id: String,
    pub mint_id: String,
    pub recipient_did: String,
    pub amount: u64,
}

impl CollateralSapIssuanceReceiptRecordV2 {
    pub fn from_records(
        authorization_action_reference: String,
        authorization: &CollateralSapMintAuthorizationRecordV2,
        mint_action_reference: String,
        mint: &CollateralSapMintRecordV2Compact,
    ) -> Result<Self, IssuancePersistenceError> {
        authorization.validate_shape()?;
        mint.validate_against_authorization(authorization)?;
        if !valid_reference(&authorization_action_reference)
            || !valid_reference(&mint_action_reference)
        {
            return Err(IssuancePersistenceError::InvalidActionReference);
        }
        if mint.authorization_action_reference != authorization_action_reference {
            return Err(IssuancePersistenceError::AuthorizationActionMismatch);
        }
        let receipt = Self {
            schema_version: COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION,
            authorization_action_reference,
            mint_action_reference,
            trust_root_id: authorization.trust_root_id.clone(),
            trust_root_version: authorization.trust_root_version,
            request_action_reference: authorization.request_action_reference.clone(),
            price_attestation_action_reference: authorization
                .price_attestation_action_reference
                .clone(),
            custody_attestation_action_reference: authorization
                .custody_attestation_action_reference
                .clone(),
            deposit_id: authorization.deposit_id.clone(),
            mint_id: authorization.mint_id.clone(),
            recipient_did: authorization.recipient_did.clone(),
            amount: authorization.amount,
        };
        receipt.validate_against_records(authorization, mint)?;
        Ok(receipt)
    }

    pub fn validate_against_records(
        &self,
        authorization: &CollateralSapMintAuthorizationRecordV2,
        mint: &CollateralSapMintRecordV2Compact,
    ) -> Result<(), IssuancePersistenceError> {
        if self.schema_version != COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION {
            return Err(IssuancePersistenceError::UnsupportedIssuanceReceiptSchema);
        }
        if !valid_reference(&self.authorization_action_reference)
            || !valid_reference(&self.mint_action_reference)
        {
            return Err(IssuancePersistenceError::InvalidActionReference);
        }
        authorization.validate_shape()?;
        mint.validate_against_authorization(authorization)?;
        if mint.authorization_action_reference != self.authorization_action_reference {
            return Err(IssuancePersistenceError::AuthorizationActionMismatch);
        }
        if self.trust_root_id != authorization.trust_root_id
            || self.trust_root_version != authorization.trust_root_version
            || self.request_action_reference != authorization.request_action_reference
            || self.price_attestation_action_reference
                != authorization.price_attestation_action_reference
            || self.custody_attestation_action_reference
                != authorization.custody_attestation_action_reference
            || self.deposit_id != authorization.deposit_id
            || self.mint_id != authorization.mint_id
            || self.recipient_did != authorization.recipient_did
            || self.amount != authorization.amount
            || self.request_action_reference != mint.request_action_reference
            || self.deposit_id != mint.deposit_id
            || self.mint_id != mint.mint_id
            || self.recipient_did != mint.recipient_did
            || self.amount != mint.amount
        {
            return Err(IssuancePersistenceError::ReceiptFactsMismatch);
        }
        Ok(())
    }
}

fn valid_reference(value: &str) -> bool {
    !value.is_empty() && value.len() <= MAX_ACTION_REFERENCE_LEN
}

fn valid_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= MAX_ID_LEN
}

fn valid_did(value: &str) -> bool {
    value.starts_with("did:mycelix:") && value.len() <= MAX_DID_LEN
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssuancePersistenceError {
    UnsupportedAuthorizationRecordSchema,
    UnsupportedMintRecordSchema,
    UnsupportedIssuanceReceiptSchema,
    InvalidActionReference,
    InvalidTrustRootReference,
    InvalidIssuanceId,
    InvalidRecipientDid,
    ZeroAmount,
    TrustRootMismatch,
    EvidenceReferenceMismatch,
    AuthorizationFactsMismatch,
    MintFactsMismatch,
    AuthorizationActionMismatch,
    ReceiptFactsMismatch,
    AuthenticatedConservation(AuthenticatedSapConservationError),
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
    use finance_sap_conservation::AuthenticatedCollateralSapMintAuthorizationV2;

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

    fn authenticated_authorization(
        root: &CollateralSettlementTrustRootV1,
    ) -> AuthenticatedCollateralSapMintAuthorizationV2 {
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
        let settlement = derive_authenticated_bound_settlement_intent_from_valid_actions(
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
        .expect("authenticated settlement");
        AuthenticatedCollateralSapMintAuthorizationV2::from_authenticated_settlement(
            settlement,
            root,
        )
        .expect("mint authorization")
    }

    #[test]
    fn compact_authorization_retains_exact_evidence_refs_and_facts() {
        let root = root();
        let authorization = authenticated_authorization(&root);
        let compact = CollateralSapMintAuthorizationRecordV2::from_authenticated(
            &authorization,
            &root,
        )
        .expect("compact authorization");
        assert_eq!(compact.request_action_reference, "uhCkk-request");
        assert_eq!(compact.price_attestation_action_reference, "uhCkk-price");
        assert_eq!(compact.custody_attestation_action_reference, "uhCkk-custody");
        assert_eq!(compact.amount, 20);
        assert_eq!(
            compact.validate_against_authenticated(&authorization, &root),
            Ok(())
        );
    }

    #[test]
    fn tampered_authorization_amount_is_rejected() {
        let root = root();
        let authorization = authenticated_authorization(&root);
        let mut compact = CollateralSapMintAuthorizationRecordV2::from_authenticated(
            &authorization,
            &root,
        )
        .expect("compact authorization");
        compact.amount += 1;
        assert_eq!(
            compact.validate_against_authenticated(&authorization, &root),
            Err(IssuancePersistenceError::AuthorizationFactsMismatch)
        );
    }

    #[test]
    fn mint_and_receipt_chain_exact_action_references() {
        let root = root();
        let authorization = authenticated_authorization(&root);
        let compact = CollateralSapMintAuthorizationRecordV2::from_authenticated(
            &authorization,
            &root,
        )
        .expect("compact authorization");
        let mint = CollateralSapMintRecordV2Compact::from_authorization_record(
            "uhCkk-auth".into(),
            &compact,
        )
        .expect("mint");
        let receipt = CollateralSapIssuanceReceiptRecordV2::from_records(
            "uhCkk-auth".into(),
            &compact,
            "uhCkk-mint".into(),
            &mint,
        )
        .expect("receipt");
        assert_eq!(receipt.authorization_action_reference, "uhCkk-auth");
        assert_eq!(receipt.mint_action_reference, "uhCkk-mint");
        assert_eq!(receipt.request_action_reference, "uhCkk-request");
        assert_eq!(receipt.price_attestation_action_reference, "uhCkk-price");
        assert_eq!(receipt.custody_attestation_action_reference, "uhCkk-custody");
    }

    #[test]
    fn receipt_rejects_mint_field_drift() {
        let root = root();
        let authorization = authenticated_authorization(&root);
        let compact = CollateralSapMintAuthorizationRecordV2::from_authenticated(
            &authorization,
            &root,
        )
        .expect("compact authorization");
        let mut mint = CollateralSapMintRecordV2Compact::from_authorization_record(
            "uhCkk-auth".into(),
            &compact,
        )
        .expect("mint");
        mint.amount += 1;
        assert_eq!(
            CollateralSapIssuanceReceiptRecordV2::from_records(
                "uhCkk-auth".into(),
                &compact,
                "uhCkk-mint".into(),
                &mint,
            ),
            Err(IssuancePersistenceError::MintFactsMismatch)
        );
    }

    #[test]
    fn compact_records_round_trip_without_embedding_full_settlement_graph() {
        let root = root();
        let authorization = authenticated_authorization(&root);
        let compact = CollateralSapMintAuthorizationRecordV2::from_authenticated(
            &authorization,
            &root,
        )
        .expect("compact authorization");
        let json = serde_json::to_string(&compact).expect("serialize");
        assert!(!json.contains("price_rate"));
        assert!(!json.contains("custody_external_reference"));
        let decoded: CollateralSapMintAuthorizationRecordV2 =
            serde_json::from_str(&json).expect("deserialize");
        assert_eq!(decoded, compact);
    }
}

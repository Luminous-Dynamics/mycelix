#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Versioned collateral-deposit request identity for FIN-SAFE-008.
//!
//! V2 deposit requests are immutable intent records. They contain no caller-
//! authoritative oracle rate and no minted SAP amount. Monetary settlement is a
//! separate FIN-SAFE-006/007 protocol.
//!
//! Deposit IDs are fixed-length, domain-separated BLAKE3 digests over canonical
//! request terms plus a caller-supplied 32-byte nonce. This crate deliberately
//! does not generate randomness: integration must obtain the nonce from a CSPRNG
//! and persist/reuse it when retrying the same logical request.

use finance_collateral_settlement::{
    CollateralDepositTerms, CollateralSettlementError, SAP_ASSET_ID,
};
use serde::{Deserialize, Serialize};

pub const COLLATERAL_DEPOSIT_REQUEST_V2_SCHEMA_VERSION: u16 = 2;
pub const COLLATERAL_DEPOSIT_ID_DOMAIN: &[u8] = b"mycelix-finance:collateral-deposit:v2";
pub const COLLATERAL_DEPOSIT_ID_PREFIX: &str = "deposit:v2:";
pub const COLLATERAL_DEPOSIT_NONCE_BYTES: usize = 32;
pub const COLLATERAL_DEPOSIT_DIGEST_HEX_BYTES: usize = 64;
pub const COLLATERAL_DEPOSIT_ID_LEN: usize =
    COLLATERAL_DEPOSIT_ID_PREFIX.len() + COLLATERAL_DEPOSIT_DIGEST_HEX_BYTES;
pub const MAX_COLLATERAL_DEPOSIT_ID_LEN: usize = 256;

/// Immutable V2 collateral deposit request.
///
/// There is intentionally no `oracle_rate`, `sap_minted`, health status, or
/// settlement status field. Those belong to separately authorized evidence and
/// state events.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralDepositRequestV2 {
    pub schema_version: u16,
    pub deposit_id: String,
    pub depositor_did: String,
    pub collateral_asset_id: String,
    pub collateral_amount: u64,
    pub quote_asset_id: String,
    /// Persisted entropy used to derive the stable request identity.
    pub request_nonce: [u8; COLLATERAL_DEPOSIT_NONCE_BYTES],
    /// Claimed request creation time. Holochain integration must bind this to the
    /// actual Create action timestamp under an explicit skew policy.
    pub created_at_micros: i64,
}

impl CollateralDepositRequestV2 {
    pub fn new(
        depositor_did: String,
        collateral_asset_id: String,
        collateral_amount: u64,
        request_nonce: [u8; COLLATERAL_DEPOSIT_NONCE_BYTES],
        created_at_micros: i64,
    ) -> Result<Self, CollateralDepositRequestError> {
        ensure_nonce_usable(&request_nonce)?;

        let provisional_terms = CollateralDepositTerms {
            // The FIN-SAFE-006 terms validator requires a bounded nonempty ID;
            // use the fixed-shape placeholder only while validating economic
            // terms before deriving the real ID.
            deposit_id: "deposit:v2:pending".into(),
            depositor_did: depositor_did.clone(),
            collateral_asset_id: collateral_asset_id.clone(),
            collateral_amount,
            quote_asset_id: SAP_ASSET_ID.into(),
        };
        provisional_terms
            .validate()
            .map_err(CollateralDepositRequestError::SettlementTerms)?;

        let deposit_id = derive_deposit_id(
            &depositor_did,
            &collateral_asset_id,
            collateral_amount,
            SAP_ASSET_ID,
            &request_nonce,
        )?;

        let request = Self {
            schema_version: COLLATERAL_DEPOSIT_REQUEST_V2_SCHEMA_VERSION,
            deposit_id,
            depositor_did,
            collateral_asset_id,
            collateral_amount,
            quote_asset_id: SAP_ASSET_ID.into(),
            request_nonce,
            created_at_micros,
        };
        request.validate()?;
        Ok(request)
    }

    /// Recompute the fixed identity from immutable request terms and reject any
    /// deserialized/tampered mismatch.
    pub fn validate(&self) -> Result<(), CollateralDepositRequestError> {
        if self.schema_version != COLLATERAL_DEPOSIT_REQUEST_V2_SCHEMA_VERSION {
            return Err(CollateralDepositRequestError::UnsupportedSchemaVersion);
        }
        ensure_nonce_usable(&self.request_nonce)?;

        let terms = self.to_settlement_terms();
        terms
            .validate()
            .map_err(CollateralDepositRequestError::SettlementTerms)?;

        let expected = derive_deposit_id(
            &self.depositor_did,
            &self.collateral_asset_id,
            self.collateral_amount,
            &self.quote_asset_id,
            &self.request_nonce,
        )?;
        if self.deposit_id != expected {
            return Err(CollateralDepositRequestError::DepositIdMismatch);
        }
        if self.deposit_id.len() != COLLATERAL_DEPOSIT_ID_LEN
            || self.deposit_id.len() > MAX_COLLATERAL_DEPOSIT_ID_LEN
        {
            return Err(CollateralDepositRequestError::InvalidDepositIdLength);
        }
        Ok(())
    }

    /// Bind the claimed request timestamp to the actual persistence action time.
    ///
    /// `max_abs_skew_micros` is selected by the trusted integration policy. A
    /// negative skew allowance is invalid. Difference arithmetic is promoted to
    /// i128 so extreme i64 endpoints cannot overflow.
    pub fn validate_action_timestamp(
        &self,
        action_timestamp_micros: i64,
        max_abs_skew_micros: i64,
    ) -> Result<(), CollateralDepositRequestError> {
        self.validate()?;
        if max_abs_skew_micros < 0 {
            return Err(CollateralDepositRequestError::InvalidClockSkewPolicy);
        }
        let delta = (self.created_at_micros as i128 - action_timestamp_micros as i128).abs();
        if delta > max_abs_skew_micros as i128 {
            return Err(CollateralDepositRequestError::CreatedAtOutsideActionSkew);
        }
        Ok(())
    }

    /// Convert the immutable request into the exact FIN-SAFE-006 settlement terms.
    pub fn to_settlement_terms(&self) -> CollateralDepositTerms {
        CollateralDepositTerms {
            deposit_id: self.deposit_id.clone(),
            depositor_did: self.depositor_did.clone(),
            collateral_asset_id: self.collateral_asset_id.clone(),
            collateral_amount: self.collateral_amount,
            quote_asset_id: self.quote_asset_id.clone(),
        }
    }
}

/// Derive the bounded, domain-separated V2 deposit identity.
pub fn derive_deposit_id(
    depositor_did: &str,
    collateral_asset_id: &str,
    collateral_amount: u64,
    quote_asset_id: &str,
    request_nonce: &[u8; COLLATERAL_DEPOSIT_NONCE_BYTES],
) -> Result<String, CollateralDepositRequestError> {
    ensure_nonce_usable(request_nonce)?;

    // Reuse the FIN-SAFE-006 term limits without making a generated identifier
    // part of the entropy input itself.
    CollateralDepositTerms {
        deposit_id: "deposit:v2:pending".into(),
        depositor_did: depositor_did.into(),
        collateral_asset_id: collateral_asset_id.into(),
        collateral_amount,
        quote_asset_id: quote_asset_id.into(),
    }
    .validate()
    .map_err(CollateralDepositRequestError::SettlementTerms)?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(COLLATERAL_DEPOSIT_ID_DOMAIN);
    push_len_prefixed(&mut hasher, depositor_did.as_bytes())?;
    push_len_prefixed(&mut hasher, collateral_asset_id.as_bytes())?;
    hasher.update(&collateral_amount.to_be_bytes());
    push_len_prefixed(&mut hasher, quote_asset_id.as_bytes())?;
    hasher.update(request_nonce);

    let digest = hasher.finalize();
    let mut id = String::with_capacity(COLLATERAL_DEPOSIT_ID_LEN);
    id.push_str(COLLATERAL_DEPOSIT_ID_PREFIX);
    push_hex(&mut id, digest.as_bytes());

    if id.len() != COLLATERAL_DEPOSIT_ID_LEN || id.len() > MAX_COLLATERAL_DEPOSIT_ID_LEN {
        return Err(CollateralDepositRequestError::InvalidDepositIdLength);
    }
    Ok(id)
}

fn ensure_nonce_usable(
    nonce: &[u8; COLLATERAL_DEPOSIT_NONCE_BYTES],
) -> Result<(), CollateralDepositRequestError> {
    if nonce.iter().all(|byte| *byte == 0) {
        return Err(CollateralDepositRequestError::ZeroNonce);
    }
    Ok(())
}

fn push_len_prefixed(
    hasher: &mut blake3::Hasher,
    bytes: &[u8],
) -> Result<(), CollateralDepositRequestError> {
    let len = u32::try_from(bytes.len()).map_err(|_| CollateralDepositRequestError::FieldTooLong)?;
    hasher.update(&len.to_be_bytes());
    hasher.update(bytes);
    Ok(())
}

fn push_hex(out: &mut String, bytes: &[u8]) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralDepositRequestError {
    UnsupportedSchemaVersion,
    SettlementTerms(CollateralSettlementError),
    ZeroNonce,
    FieldTooLong,
    DepositIdMismatch,
    InvalidDepositIdLength,
    InvalidClockSkewPolicy,
    CreatedAtOutsideActionSkew,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nonce(byte: u8) -> [u8; COLLATERAL_DEPOSIT_NONCE_BYTES] {
        [byte; COLLATERAL_DEPOSIT_NONCE_BYTES]
    }

    #[test]
    fn generated_id_is_fixed_length_and_downstream_safe() {
        let request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            42,
            nonce(1),
            100,
        )
        .expect("valid request");

        assert_eq!(request.deposit_id.len(), COLLATERAL_DEPOSIT_ID_LEN);
        assert!(request.deposit_id.len() <= 256);
        assert!(request.deposit_id.starts_with(COLLATERAL_DEPOSIT_ID_PREFIX));
        assert_eq!(request.validate(), Ok(()));
        assert_eq!(request.to_settlement_terms().validate(), Ok(()));
    }

    #[test]
    fn maximum_length_did_does_not_expand_identifier() {
        let max_did = format!("did:{}", "a".repeat(252));
        assert_eq!(max_did.len(), 256);
        let id = derive_deposit_id(&max_did, "ETH", 1, SAP_ASSET_ID, &nonce(2))
            .expect("maximum valid DID must still yield a bounded ID");
        assert_eq!(id.len(), COLLATERAL_DEPOSIT_ID_LEN);
        assert!(id.len() <= MAX_COLLATERAL_DEPOSIT_ID_LEN);
    }

    #[test]
    fn same_terms_and_nonce_are_stable_across_retries() {
        let first = derive_deposit_id(
            "did:mycelix:alice",
            "ETH",
            42,
            SAP_ASSET_ID,
            &nonce(3),
        )
        .expect("id");
        let retry = derive_deposit_id(
            "did:mycelix:alice",
            "ETH",
            42,
            SAP_ASSET_ID,
            &nonce(3),
        )
        .expect("id");
        assert_eq!(first, retry);
    }

    #[test]
    fn nonce_changes_identity_for_concurrent_same_terms() {
        let first = derive_deposit_id(
            "did:mycelix:alice",
            "ETH",
            42,
            SAP_ASSET_ID,
            &nonce(4),
        )
        .expect("id");
        let second = derive_deposit_id(
            "did:mycelix:alice",
            "ETH",
            42,
            SAP_ASSET_ID,
            &nonce(5),
        )
        .expect("id");
        assert_ne!(first, second);
    }

    #[test]
    fn immutable_term_changes_change_identity() {
        let base = derive_deposit_id(
            "did:mycelix:alice",
            "ETH",
            42,
            SAP_ASSET_ID,
            &nonce(6),
        )
        .expect("base");
        let asset_changed = derive_deposit_id(
            "did:mycelix:alice",
            "USDC",
            42,
            SAP_ASSET_ID,
            &nonce(6),
        )
        .expect("asset");
        let amount_changed = derive_deposit_id(
            "did:mycelix:alice",
            "ETH",
            43,
            SAP_ASSET_ID,
            &nonce(6),
        )
        .expect("amount");
        assert_ne!(base, asset_changed);
        assert_ne!(base, amount_changed);
    }

    #[test]
    fn zero_nonce_is_rejected_rather_than_presented_as_entropy() {
        assert_eq!(
            derive_deposit_id(
                "did:mycelix:alice",
                "ETH",
                42,
                SAP_ASSET_ID,
                &[0; COLLATERAL_DEPOSIT_NONCE_BYTES],
            ),
            Err(CollateralDepositRequestError::ZeroNonce)
        );
    }

    #[test]
    fn tampered_id_is_rejected_by_recomputation() {
        let mut request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            42,
            nonce(7),
            100,
        )
        .expect("valid request");
        request.deposit_id.push('x');
        assert_eq!(
            request.validate(),
            Err(CollateralDepositRequestError::DepositIdMismatch)
        );
    }

    #[test]
    fn created_at_is_bound_to_action_time_under_explicit_skew_policy() {
        let request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            42,
            nonce(8),
            1_000,
        )
        .expect("valid request");
        assert_eq!(request.validate_action_timestamp(1_005, 5), Ok(()));
        assert_eq!(
            request.validate_action_timestamp(1_006, 5),
            Err(CollateralDepositRequestError::CreatedAtOutsideActionSkew)
        );
    }

    #[test]
    fn timestamp_delta_handles_extreme_i64_values_without_overflow() {
        let request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            42,
            nonce(9),
            i64::MIN,
        )
        .expect("valid request");
        assert_eq!(
            request.validate_action_timestamp(i64::MAX, i64::MAX),
            Err(CollateralDepositRequestError::CreatedAtOutsideActionSkew)
        );
    }

    #[test]
    fn request_schema_contains_no_monetary_settlement_fields() {
        let request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            42,
            nonce(10),
            100,
        )
        .expect("valid request");
        let json = serde_json::to_string(&request).expect("serialize");
        assert!(!json.contains("oracle_rate"));
        assert!(!json.contains("sap_minted"));
        assert!(!json.contains("status"));
    }

    #[test]
    fn serialization_round_trip_preserves_identity_and_nonce() {
        let request = CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            42,
            nonce(11),
            100,
        )
        .expect("valid request");
        let encoded = serde_json::to_string(&request).expect("serialize");
        let decoded: CollateralDepositRequestV2 =
            serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(decoded, request);
        assert_eq!(decoded.validate(), Ok(()));
    }
}

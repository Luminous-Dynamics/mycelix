// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Ed25519-only cryptographic authenticity adapter for Identity V2 historical time receipts.
//!
//! This layer consumes only one opaque #397 prepared crypto request. It performs strict
//! Ed25519 verification over the exact #394 signing digest and returns an opaque
//! cryptographic-authenticity capability only on success.
//!
//! All non-Ed25519 algorithms fail closed. In particular, hybrid ML-DSA authenticity is
//! deliberately unavailable until the #252/#254 FIPS wire-contract blocker is resolved.
//! Success here does not establish accepted/current time-authority policy, key provenance,
//! trusted wall-clock authority, historical activation eligibility, or positive evidence.

#![forbid(unsafe_code)]

use ed25519_dalek::{Signature, VerifyingKey};
use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_receipt_crypto_request_policy::PreparedHistoricalActivationTimeReceiptCryptoRequestV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_RECEIPT_ED25519_AUTHENTICITY_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-receipt-ed25519-authenticity:v2\0";

#[derive(Debug)]
pub struct CryptographicallyAuthenticatedHistoricalActivationTimeReceiptV2 {
    activation_subject_sha256: [u8; SHA256_DIGEST_LEN_V2],
    receipt_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_policy_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    crypto_request_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    authenticity_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl CryptographicallyAuthenticatedHistoricalActivationTimeReceiptV2 {
    pub fn activation_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.activation_subject_sha256
    }

    pub fn receipt_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.receipt_signing_digest_sha256
    }

    pub fn time_authority_policy_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.time_authority_policy_sha256
    }

    pub fn time_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.time_authority_key_generation_sha256
    }

    pub fn crypto_request_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.crypto_request_digest_sha256
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn authenticity_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authenticity_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimeReceiptAuthenticityErrorV2 {
    AlgorithmPendingQualification,
    PublicKeyLengthInvalid,
    SignatureLengthInvalid,
    PublicKeyInvalid,
    SignatureInvalid,
}

fn derive_authenticity_digest_v2(
    request: &PreparedHistoricalActivationTimeReceiptCryptoRequestV2,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_RECEIPT_ED25519_AUTHENTICITY_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(request.crypto_request_digest_sha256());
    hasher.finalize().into()
}

/// Strictly authenticate one Ed25519 time receipt.
///
/// This function owns the backend choice. It accepts no caller-supplied verifier object.
/// `HybridEd25519MlDsa65` and all other algorithms fail before any component verification.
pub fn authenticate_historical_activation_time_receipt_ed25519_v2(
    request: &PreparedHistoricalActivationTimeReceiptCryptoRequestV2,
) -> Result<
    CryptographicallyAuthenticatedHistoricalActivationTimeReceiptV2,
    HistoricalActivationTimeReceiptAuthenticityErrorV2,
> {
    if request.algorithm() != AlgorithmId::Ed25519 {
        return Err(
            HistoricalActivationTimeReceiptAuthenticityErrorV2::AlgorithmPendingQualification,
        );
    }

    let public_key_bytes: [u8; 32] = request
        .public_key_bytes()
        .try_into()
        .map_err(|_| HistoricalActivationTimeReceiptAuthenticityErrorV2::PublicKeyLengthInvalid)?;
    let signature_bytes: [u8; 64] = request
        .signature_bytes()
        .try_into()
        .map_err(|_| HistoricalActivationTimeReceiptAuthenticityErrorV2::SignatureLengthInvalid)?;

    let verifying_key = VerifyingKey::from_bytes(&public_key_bytes)
        .map_err(|_| HistoricalActivationTimeReceiptAuthenticityErrorV2::PublicKeyInvalid)?;
    let signature = Signature::from_bytes(&signature_bytes);

    verifying_key
        .verify_strict(request.receipt_signing_digest_sha256(), &signature)
        .map_err(|_| HistoricalActivationTimeReceiptAuthenticityErrorV2::SignatureInvalid)?;

    let authenticity_digest_sha256 = derive_authenticity_digest_v2(request);
    Ok(CryptographicallyAuthenticatedHistoricalActivationTimeReceiptV2 {
        activation_subject_sha256: *request.activation_subject_sha256(),
        receipt_signing_digest_sha256: *request.receipt_signing_digest_sha256(),
        time_authority_policy_sha256: *request.time_authority_policy_sha256(),
        time_authority_key_generation_sha256: *request.time_authority_key_generation_sha256(),
        crypto_request_digest_sha256: *request.crypto_request_digest_sha256(),
        algorithm: request.algorithm(),
        authenticity_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use mycelix_crypto::TaggedSignature;
    use mycelix_historical_activation_time_authority_key_generation_policy::{
        qualify_historical_activation_time_authority_key_generation_v2,
        HistoricalActivationTimeAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
    };
    use mycelix_historical_activation_time_receipt_crypto_request_policy::prepare_historical_activation_time_receipt_crypto_request_v2;
    use mycelix_historical_activation_time_receipt_policy::{
        prepare_historical_activation_time_receipt_v2, HistoricalActivationTimeReceiptBodyV2,
        TimeBasisV2,
    };

    static ACTIVATION_SUBJECT: [u8; SHA256_DIGEST_LEN_V2] = [0x11; SHA256_DIGEST_LEN_V2];

    fn request_with_signature_tamper(
        tamper: bool,
    ) -> PreparedHistoricalActivationTimeReceiptCryptoRequestV2 {
        let signing_key = SigningKey::from_bytes(&[0x66; 32]);
        let public_key = signing_key.verifying_key().to_bytes();
        let key_generation = qualify_historical_activation_time_authority_key_generation_v2(
            HistoricalActivationTimeAuthorityKeyGenerationV2 {
                time_authority_id: "time:authority:test-v2",
                time_authority_key_id: "time:authority:test-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &public_key,
                key_generation: 1,
                valid_from_micros: 500_000,
                valid_until_micros: 20_000_000,
            },
        )
        .unwrap();
        let policy = qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:test-v2",
                policy_version: 1,
                time_authority_id: key_generation.time_authority_id(),
                time_authority_key_id: key_generation.time_authority_key_id(),
                time_authority_key_generation_sha256: key_generation.generation_digest_sha256(),
                algorithm: AlgorithmId::Ed25519,
                time_basis: TimeBasisV2::UnixMicrosecondsUtc,
                utc_realization_id: "unix-utc-normalized-v1",
                max_uncertainty_before_micros: 10_000,
                max_uncertainty_after_micros: 10_000,
                max_receipt_lifetime_micros: 1_000_000,
                valid_from_micros: 1_000_000,
                valid_until_micros: 10_000_000,
            },
        )
        .unwrap();
        let receipt = prepare_historical_activation_time_receipt_v2(
            HistoricalActivationTimeReceiptBodyV2 {
                activation_subject_sha256: &ACTIVATION_SUBJECT,
                time_authority_policy_sha256: policy.policy_digest_sha256(),
                time_authority_id: key_generation.time_authority_id(),
                time_authority_key_id: key_generation.time_authority_key_id(),
                time_authority_key_generation_sha256: key_generation.generation_digest_sha256(),
                algorithm: AlgorithmId::Ed25519,
                time_basis: TimeBasisV2::UnixMicrosecondsUtc,
                observed_at_micros: 2_000_000,
                uncertainty_before_micros: 5_000,
                uncertainty_after_micros: 7_000,
                receipt_valid_until_micros: 3_000_000,
            },
        )
        .unwrap();
        let mut raw_signature = signing_key
            .sign(receipt.signing_digest_sha256())
            .to_bytes()
            .to_vec();
        if tamper {
            raw_signature[0] ^= 0x01;
        }
        let signature = TaggedSignature::new(AlgorithmId::Ed25519, raw_signature).unwrap();
        prepare_historical_activation_time_receipt_crypto_request_v2(
            &receipt,
            &policy,
            &key_generation,
            &signature,
        )
        .unwrap()
    }

    #[test]
    fn valid_ed25519_receipt_is_strictly_authenticated() {
        let request = request_with_signature_tamper(false);
        assert_eq!(
            request.crypto_request_digest_sha256(),
            &[
                0x58, 0x5a, 0xbd, 0x0b, 0xfe, 0x0f, 0x64, 0x30, 0x2f, 0x3c, 0x30, 0x08,
                0x39, 0xb5, 0x5f, 0x23, 0x40, 0x75, 0xd0, 0x3a, 0x91, 0xa5, 0x22, 0x9b,
                0x7c, 0x7f, 0x95, 0xf6, 0xdf, 0xab, 0x04, 0xe8,
            ]
        );
        let authenticated = authenticate_historical_activation_time_receipt_ed25519_v2(&request)
            .unwrap();
        assert_eq!(authenticated.algorithm(), AlgorithmId::Ed25519);
        assert_eq!(
            authenticated.authenticity_digest_sha256(),
            &[
                0x8c, 0xce, 0xbf, 0x8e, 0x62, 0x43, 0x20, 0x02, 0x35, 0xa4, 0x94, 0xb1,
                0x4e, 0xd9, 0xf1, 0xc9, 0x99, 0x25, 0x21, 0x34, 0x4d, 0x87, 0x98, 0xec,
                0x27, 0x24, 0x18, 0x95, 0x33, 0x3c, 0x78, 0xce,
            ]
        );
    }

    #[test]
    fn tampered_signature_fails_strict_verification() {
        let request = request_with_signature_tamper(true);
        assert_eq!(
            authenticate_historical_activation_time_receipt_ed25519_v2(&request).unwrap_err(),
            HistoricalActivationTimeReceiptAuthenticityErrorV2::SignatureInvalid
        );
    }

    #[test]
    fn authenticated_result_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct CryptographicallyAuthenticatedHistoricalActivationTimeReceiptV2")
            .unwrap();
        let end = source[start..]
            .index("impl CryptographicallyAuthenticatedHistoricalActivationTimeReceiptV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub receipt_signing_digest_sha256:",
            "pub crypto_request_digest_sha256:",
            "pub authenticity_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

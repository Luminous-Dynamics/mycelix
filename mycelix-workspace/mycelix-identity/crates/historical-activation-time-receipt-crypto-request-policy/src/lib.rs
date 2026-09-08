// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Prepared cryptographic verification request for one historical-activation time receipt.
//!
//! This theorem binds #394 receipt bytes, #395 static policy, #396 exact time-authority
//! key generation, and exact signature bytes into one opaque pre-verification request.
//! It deliberately performs no cryptographic verification.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedSignature};
use mycelix_historical_activation_time_authority_key_generation_policy::{
    validate_time_authority_key_generation_against_static_policy_v2,
    HistoricalActivationTimeAuthorityKeyGenerationErrorV2,
    QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2,
};
use mycelix_historical_activation_time_authority_policy::{
    validate_prepared_time_receipt_against_static_policy_v2,
    HistoricalActivationTimeAuthorityPolicyErrorV2,
    QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
};
use mycelix_historical_activation_time_receipt_policy::PreparedHistoricalActivationTimeReceiptV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_RECEIPT_CRYPTO_REQUEST_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-receipt-crypto-request:v2\0";

#[derive(Debug)]
pub struct PreparedHistoricalActivationTimeReceiptCryptoRequestV2 {
    activation_subject_sha256: [u8; SHA256_DIGEST_LEN_V2],
    receipt_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_policy_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    signature_bytes: Vec<u8>,
    crypto_request_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl PreparedHistoricalActivationTimeReceiptCryptoRequestV2 {
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

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn public_key_bytes(&self) -> &[u8] {
        &self.public_key_bytes
    }

    pub fn signature_bytes(&self) -> &[u8] {
        &self.signature_bytes
    }

    pub fn crypto_request_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.crypto_request_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimeReceiptCryptoRequestErrorV2 {
    ReceiptPolicyBinding(HistoricalActivationTimeAuthorityPolicyErrorV2),
    KeyGenerationPolicyBinding(HistoricalActivationTimeAuthorityKeyGenerationErrorV2),
    ReceiptKeyGenerationMismatch,
    ReceiptKeyAlgorithmMismatch,
    SignatureAlgorithmMismatch,
    SignatureLengthInvalid,
    SignatureAllZero,
}

impl From<HistoricalActivationTimeAuthorityPolicyErrorV2>
    for HistoricalActivationTimeReceiptCryptoRequestErrorV2
{
    fn from(value: HistoricalActivationTimeAuthorityPolicyErrorV2) -> Self {
        Self::ReceiptPolicyBinding(value)
    }
}

impl From<HistoricalActivationTimeAuthorityKeyGenerationErrorV2>
    for HistoricalActivationTimeReceiptCryptoRequestErrorV2
{
    fn from(value: HistoricalActivationTimeAuthorityKeyGenerationErrorV2) -> Self {
        Self::KeyGenerationPolicyBinding(value)
    }
}

fn update_len_prefixed_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

fn derive_crypto_request_digest_v2(
    receipt: &PreparedHistoricalActivationTimeReceiptV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    key_generation: &QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2,
    signature: &TaggedSignature,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_RECEIPT_CRYPTO_REQUEST_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(receipt.activation_subject_sha256());
    hasher.update([0x02]);
    hasher.update(receipt.signing_digest_sha256());
    hasher.update([0x03]);
    hasher.update(policy.policy_digest_sha256());
    hasher.update([0x04]);
    hasher.update(key_generation.generation_digest_sha256());
    hasher.update([0x05]);
    hasher.update(key_generation.algorithm().as_u16().to_be_bytes());
    update_len_prefixed_v2(&mut hasher, 0x06, key_generation.public_key_bytes());
    update_len_prefixed_v2(&mut hasher, 0x07, &signature.signature_bytes);
    hasher.finalize().into()
}

/// Prepare one exact crypto request without invoking any cryptographic verifier.
pub fn prepare_historical_activation_time_receipt_crypto_request_v2(
    receipt: &PreparedHistoricalActivationTimeReceiptV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    key_generation: &QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2,
    signature: &TaggedSignature,
) -> Result<
    PreparedHistoricalActivationTimeReceiptCryptoRequestV2,
    HistoricalActivationTimeReceiptCryptoRequestErrorV2,
> {
    validate_prepared_time_receipt_against_static_policy_v2(receipt, policy)?;
    validate_time_authority_key_generation_against_static_policy_v2(key_generation, policy)?;

    if receipt.time_authority_key_generation_sha256() != key_generation.generation_digest_sha256() {
        return Err(
            HistoricalActivationTimeReceiptCryptoRequestErrorV2::ReceiptKeyGenerationMismatch,
        );
    }
    if receipt.algorithm() != key_generation.algorithm() {
        return Err(HistoricalActivationTimeReceiptCryptoRequestErrorV2::ReceiptKeyAlgorithmMismatch);
    }
    if signature.algorithm != key_generation.algorithm() {
        return Err(HistoricalActivationTimeReceiptCryptoRequestErrorV2::SignatureAlgorithmMismatch);
    }
    if signature.signature_bytes.len() != key_generation.algorithm().signature_size() {
        return Err(HistoricalActivationTimeReceiptCryptoRequestErrorV2::SignatureLengthInvalid);
    }
    if signature.signature_bytes.iter().all(|byte| *byte == 0) {
        return Err(HistoricalActivationTimeReceiptCryptoRequestErrorV2::SignatureAllZero);
    }

    let crypto_request_digest_sha256 =
        derive_crypto_request_digest_v2(receipt, policy, key_generation, signature);

    Ok(PreparedHistoricalActivationTimeReceiptCryptoRequestV2 {
        activation_subject_sha256: *receipt.activation_subject_sha256(),
        receipt_signing_digest_sha256: *receipt.signing_digest_sha256(),
        time_authority_policy_sha256: *policy.policy_digest_sha256(),
        time_authority_key_generation_sha256: *key_generation.generation_digest_sha256(),
        algorithm: key_generation.algorithm(),
        public_key_bytes: key_generation.public_key_bytes().to_vec(),
        signature_bytes: signature.signature_bytes.clone(),
        crypto_request_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_authority_key_generation_policy::{
        qualify_historical_activation_time_authority_key_generation_v2,
        HistoricalActivationTimeAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
    };
    use mycelix_historical_activation_time_receipt_policy::{
        prepare_historical_activation_time_receipt_v2, HistoricalActivationTimeReceiptBodyV2,
        TimeBasisV2,
    };

    static ACTIVATION_SUBJECT: [u8; SHA256_DIGEST_LEN_V2] = [0x11; SHA256_DIGEST_LEN_V2];
    static PUBLIC_KEY: [u8; 32] = [0x44; 32];

    fn fixture() -> (
        PreparedHistoricalActivationTimeReceiptV2,
        QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
        QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2,
        TaggedSignature,
    ) {
        let key_generation = qualify_historical_activation_time_authority_key_generation_v2(
            HistoricalActivationTimeAuthorityKeyGenerationV2 {
                time_authority_id: "time:authority:test-v2",
                time_authority_key_id: "time:authority:test-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &PUBLIC_KEY,
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
                algorithm: key_generation.algorithm(),
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
                algorithm: key_generation.algorithm(),
                time_basis: TimeBasisV2::UnixMicrosecondsUtc,
                observed_at_micros: 2_000_000,
                uncertainty_before_micros: 5_000,
                uncertainty_after_micros: 7_000,
                receipt_valid_until_micros: 3_000_000,
            },
        )
        .unwrap();

        let signature = TaggedSignature::new(AlgorithmId::Ed25519, vec![0x55; 64]).unwrap();
        (receipt, policy, key_generation, signature)
    }

    #[test]
    fn frozen_prepared_crypto_request_digest_is_stable() {
        let (receipt, policy, key_generation, signature) = fixture();
        let request = prepare_historical_activation_time_receipt_crypto_request_v2(
            &receipt,
            &policy,
            &key_generation,
            &signature,
        )
        .unwrap();
        assert_eq!(
            request.crypto_request_digest_sha256(),
            &[
                0xe6, 0xdb, 0xac, 0xc1, 0x1c, 0xc3, 0xf0, 0xc7, 0xe1, 0x21, 0xe4, 0x83,
                0x22, 0x38, 0x82, 0x7d, 0x8d, 0x20, 0x00, 0x7c, 0x26, 0x4e, 0xb5, 0x73,
                0x3f, 0xb9, 0x57, 0x34, 0x72, 0xdf, 0x61, 0x94,
            ]
        );
    }

    #[test]
    fn signature_algorithm_substitution_fails_closed() {
        let (receipt, policy, key_generation, mut signature) = fixture();
        signature.algorithm = AlgorithmId::MlDsa65;
        assert_eq!(
            prepare_historical_activation_time_receipt_crypto_request_v2(
                &receipt,
                &policy,
                &key_generation,
                &signature,
            )
            .unwrap_err(),
            HistoricalActivationTimeReceiptCryptoRequestErrorV2::SignatureAlgorithmMismatch
        );
    }

    #[test]
    fn public_result_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct PreparedHistoricalActivationTimeReceiptCryptoRequestV2")
            .unwrap();
        let end = source[start..]
            .index("impl PreparedHistoricalActivationTimeReceiptCryptoRequestV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub receipt_signing_digest_sha256:",
            "pub public_key_bytes:",
            "pub signature_bytes:",
            "pub crypto_request_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

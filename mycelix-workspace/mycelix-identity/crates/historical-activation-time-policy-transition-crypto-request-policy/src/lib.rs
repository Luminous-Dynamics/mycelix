// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Prepared cryptographic request for one historical time-policy authority transition.
//!
//! This pure theorem binds one #401 transition, its exact #403 authority key generation,
//! and one exact signature into a single backend-verification subject. It performs no
//! cryptographic verification and establishes no transition or policy authority.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedSignature};
use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
    validate_policy_authority_key_generation_against_transition_v2,
    HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2,
    QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
};
use mycelix_historical_activation_time_policy_transition_policy::PreparedHistoricalActivationTimePolicyTransitionV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_CRYPTO_REQUEST_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-transition-crypto-request:v2\0";

#[derive(Debug)]
pub struct PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    signature_bytes: Vec<u8>,
    crypto_request_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }
    pub fn transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.transition_signing_digest_sha256
    }
    pub fn policy_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.policy_authority_key_generation_sha256
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
pub enum HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2 {
    KeyGenerationBindingFailed(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2),
    AlgorithmMismatch,
    SignatureLengthInvalid,
    SignatureAllZero,
}

fn update_len_prefixed_u32_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

pub fn prepare_historical_activation_time_policy_transition_crypto_request_v2(
    transition: &PreparedHistoricalActivationTimePolicyTransitionV2,
    key_generation: &QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    signature: &TaggedSignature,
) -> Result<
    PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2,
    HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2,
> {
    validate_policy_authority_key_generation_against_transition_v2(key_generation, transition)
        .map_err(HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2::KeyGenerationBindingFailed)?;

    if signature.algorithm != key_generation.algorithm() {
        return Err(HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2::AlgorithmMismatch);
    }
    if signature.signature_bytes.len() != key_generation.algorithm().signature_size() {
        return Err(HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2::SignatureLengthInvalid);
    }
    if signature.signature_bytes.iter().all(|byte| *byte == 0) {
        return Err(HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2::SignatureAllZero);
    }

    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_CRYPTO_REQUEST_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(key_generation.authority_domain_sha256());
    hasher.update([0x02]);
    hasher.update(transition.transition_signing_digest_sha256());
    hasher.update([0x03]);
    hasher.update(key_generation.generation_digest_sha256());
    hasher.update([0x04]);
    hasher.update(key_generation.algorithm().as_u16().to_be_bytes());
    update_len_prefixed_u32_v2(&mut hasher, 0x05, key_generation.public_key_bytes());
    update_len_prefixed_u32_v2(&mut hasher, 0x06, &signature.signature_bytes);
    let crypto_request_digest_sha256 = hasher.finalize().into();

    Ok(PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
        authority_domain_sha256: *key_generation.authority_domain_sha256(),
        transition_signing_digest_sha256: *transition.transition_signing_digest_sha256(),
        policy_authority_key_generation_sha256: *key_generation.generation_digest_sha256(),
        algorithm: key_generation.algorithm(),
        public_key_bytes: key_generation.public_key_bytes().to_vec(),
        signature_bytes: signature.signature_bytes.clone(),
        crypto_request_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
    };
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_policy_transition_policy::{
        prepare_time_policy_adoption_transition_v2,
        HistoricalActivationTimePolicyAuthoritySignerV2,
    };
    use mycelix_historical_activation_time_receipt_policy::TimeBasisV2;
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    };

    static DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];
    static TIME_KEY_GENERATION: [u8; 32] = [0x33; 32];
    static POLICY_AUTHORITY_PUBLIC_KEY: [u8; 32] = [0x66; 32];

    fn fixture() -> (
        PreparedHistoricalActivationTimePolicyTransitionV2,
        QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    ) {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &POLICY_AUTHORITY_PUBLIC_KEY,
                key_generation: 1,
            },
        )
        .unwrap();
        let policy = qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:primary-v2",
                policy_version: 1,
                time_authority_id: "time:authority:primary-v2",
                time_authority_key_id: "time:authority:primary-v2#hybrid-1",
                time_authority_key_generation_sha256: &TIME_KEY_GENERATION,
                algorithm: AlgorithmId::HybridEd25519MlDsa65,
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
        let transition = prepare_time_policy_adoption_transition_v2(
            &domain,
            &policy,
            1,
            None,
            HistoricalActivationTimePolicyAuthoritySignerV2 {
                policy_authority_id: generation.policy_authority_id(),
                policy_authority_key_id: generation.policy_authority_key_id(),
                policy_authority_key_generation_sha256: generation.generation_digest_sha256(),
                algorithm: generation.algorithm(),
            },
        )
        .unwrap();
        (transition, generation)
    }

    #[test]
    fn frozen_ed25519_transition_request_is_stable() {
        let (transition, generation) = fixture();
        assert_eq!(
            transition.transition_signing_digest_sha256(),
            &[
                0xba, 0xb3, 0xcd, 0x7c, 0x1d, 0x07, 0xe8, 0x3b, 0x9c, 0x6f, 0xbd, 0x40,
                0x45, 0x0e, 0x29, 0x9f, 0x13, 0x52, 0xb4, 0x0e, 0x41, 0x28, 0xf4, 0x43,
                0x6a, 0x4c, 0x17, 0xa1, 0x94, 0xfa, 0x75, 0xe9,
            ]
        );
        let signature = TaggedSignature::new(AlgorithmId::Ed25519, vec![0x77; 64]).unwrap();
        let request = prepare_historical_activation_time_policy_transition_crypto_request_v2(
            &transition,
            &generation,
            &signature,
        )
        .unwrap();
        assert_eq!(
            request.crypto_request_digest_sha256(),
            &[
                0x49, 0x78, 0x90, 0xdd, 0xcf, 0x20, 0x80, 0xc0, 0x9b, 0xfb, 0xc3, 0x02,
                0xcc, 0x1e, 0x4d, 0xf9, 0xe6, 0x8f, 0xeb, 0x71, 0xff, 0xa9, 0x68, 0x7f,
                0xb0, 0x2b, 0x7f, 0xb3, 0xb1, 0x5b, 0xfb, 0xe5,
            ]
        );
    }

    #[test]
    fn signature_algorithm_substitution_fails_closed() {
        let (transition, generation) = fixture();
        let signature = TaggedSignature::new(AlgorithmId::MlDsa65, vec![0x77; 3309]).unwrap();
        assert_eq!(
            prepare_historical_activation_time_policy_transition_crypto_request_v2(
                &transition,
                &generation,
                &signature,
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2::AlgorithmMismatch
        );
    }

    #[test]
    fn all_zero_signature_fails_closed() {
        let (transition, generation) = fixture();
        let signature = TaggedSignature::new(AlgorithmId::Ed25519, vec![0; 64]).unwrap();
        assert_eq!(
            prepare_historical_activation_time_policy_transition_crypto_request_v2(
                &transition,
                &generation,
                &signature,
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyTransitionCryptoRequestErrorV2::SignatureAllZero
        );
    }

    #[test]
    fn prepared_request_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2")
            .unwrap();
        let end = source[start..]
            .index("impl PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2")
            .unwrap()
            + start;
        let result = &source[start..end];
        for field in [
            "pub public_key_bytes:",
            "pub signature_bytes:",
            "pub crypto_request_digest_sha256:",
        ] {
            assert!(!result.contains(field));
        }
    }
}

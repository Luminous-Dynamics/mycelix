// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact authority-domain-scoped policy-authority key generation for historical time-policy transitions.
//!
//! This pure theorem defines which exact public key material the signer-generation digest in a #401
//! transition means. It intentionally carries no wall-clock interval: transition authority is not
//! bootstrapped from a candidate clock. Provenance/currentness of this generation remains later work.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_transition_policy::PreparedHistoricalActivationTimePolicyTransitionV2;
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const POLICY_AUTHORITY_ID_MAX_LEN_V2: usize = 256;
pub const POLICY_AUTHORITY_KEY_ID_MAX_LEN_V2: usize = 512;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_KEY_GENERATION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authority-key-generation:v2\0";

#[derive(Debug, Clone, Copy)]
pub struct HistoricalActivationTimePolicyAuthorityKeyGenerationV2<'a> {
    pub policy_authority_id: &'a str,
    pub policy_authority_key_id: &'a str,
    pub algorithm: AlgorithmId,
    pub public_key_bytes: &'a [u8],
    pub key_generation: u64,
}

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_id: String,
    policy_authority_key_id: String,
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    key_generation: u64,
    generation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }
    pub fn policy_authority_id(&self) -> &str {
        &self.policy_authority_id
    }
    pub fn policy_authority_key_id(&self) -> &str {
        &self.policy_authority_key_id
    }
    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }
    pub fn public_key_bytes(&self) -> &[u8] {
        &self.public_key_bytes
    }
    pub fn key_generation(&self) -> u64 {
        self.key_generation
    }
    pub fn generation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.generation_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2 {
    PolicyAuthorityIdInvalid,
    PolicyAuthorityKeyIdInvalid,
    NonSignatureAlgorithm,
    PublicKeyLengthInvalid,
    PublicKeyAllZero,
    KeyGenerationInvalid,
    TransitionAuthorityDomainMismatch,
    TransitionAuthorityIdMismatch,
    TransitionAuthorityKeyIdMismatch,
    TransitionAuthorityKeyGenerationMismatch,
    TransitionAlgorithmMismatch,
}

fn valid_visible_ascii_identifier(value: &str, max_len: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_len
        && value.is_ascii()
        && value
            .as_bytes()
            .iter()
            .all(|byte| (0x21..=0x7e).contains(byte))
}

fn validate_generation_body_v2(
    body: HistoricalActivationTimePolicyAuthorityKeyGenerationV2<'_>,
) -> Result<(), HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2> {
    if !valid_visible_ascii_identifier(body.policy_authority_id, POLICY_AUTHORITY_ID_MAX_LEN_V2) {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::PolicyAuthorityIdInvalid);
    }
    if !valid_visible_ascii_identifier(
        body.policy_authority_key_id,
        POLICY_AUTHORITY_KEY_ID_MAX_LEN_V2,
    ) {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::PolicyAuthorityKeyIdInvalid);
    }
    if !body.algorithm.is_signature_algorithm() {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::NonSignatureAlgorithm);
    }
    if body.public_key_bytes.len() != body.algorithm.public_key_size() {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::PublicKeyLengthInvalid);
    }
    if body.public_key_bytes.iter().all(|byte| *byte == 0) {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::PublicKeyAllZero);
    }
    if body.key_generation == 0 {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::KeyGenerationInvalid);
    }
    Ok(())
}

fn update_len_prefixed_u16_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u16).to_be_bytes());
    hasher.update(value);
}

fn update_len_prefixed_u32_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

pub fn derive_historical_activation_time_policy_authority_key_generation_digest_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    body: HistoricalActivationTimePolicyAuthorityKeyGenerationV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2> {
    validate_generation_body_v2(body)?;
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_KEY_GENERATION_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(authority_domain.digest_sha256());
    update_len_prefixed_u16_v2(&mut hasher, 0x02, body.policy_authority_id.as_bytes());
    update_len_prefixed_u16_v2(&mut hasher, 0x03, body.policy_authority_key_id.as_bytes());
    hasher.update([0x04]);
    hasher.update(body.algorithm.as_u16().to_be_bytes());
    update_len_prefixed_u32_v2(&mut hasher, 0x05, body.public_key_bytes);
    hasher.update([0x06]);
    hasher.update(body.key_generation.to_be_bytes());
    Ok(hasher.finalize().into())
}

pub fn qualify_historical_activation_time_policy_authority_key_generation_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    body: HistoricalActivationTimePolicyAuthorityKeyGenerationV2<'_>,
) -> Result<
    QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2,
> {
    validate_generation_body_v2(body)?;
    let generation_digest_sha256 =
        derive_historical_activation_time_policy_authority_key_generation_digest_v2(
            authority_domain,
            body,
        )?;
    Ok(QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
        authority_domain_sha256: *authority_domain.digest_sha256(),
        policy_authority_id: body.policy_authority_id.to_string(),
        policy_authority_key_id: body.policy_authority_key_id.to_string(),
        algorithm: body.algorithm,
        public_key_bytes: body.public_key_bytes.to_vec(),
        key_generation: body.key_generation,
        generation_digest_sha256,
    })
}

pub fn validate_policy_authority_key_generation_against_transition_v2(
    generation: &QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    transition: &PreparedHistoricalActivationTimePolicyTransitionV2,
) -> Result<(), HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2> {
    if generation.authority_domain_sha256() != transition.authority_domain_sha256() {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::TransitionAuthorityDomainMismatch);
    }
    if generation.policy_authority_id() != transition.policy_authority_id() {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::TransitionAuthorityIdMismatch);
    }
    if generation.policy_authority_key_id() != transition.policy_authority_key_id() {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::TransitionAuthorityKeyIdMismatch);
    }
    if generation.generation_digest_sha256() != transition.policy_authority_key_generation_sha256() {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::TransitionAuthorityKeyGenerationMismatch);
    }
    if generation.algorithm() != transition.algorithm() {
        return Err(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::TransitionAlgorithmMismatch);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
        QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
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

    fn domain() -> QualifiedIdentityAuthorityDomainV2 {
        qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap()
    }

    fn policy() -> QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2 {
        qualify_static_historical_activation_time_authority_policy_v2(
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
        .unwrap()
    }

    fn body() -> HistoricalActivationTimePolicyAuthorityKeyGenerationV2<'static> {
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
            policy_authority_id: "identity:policy-authority:bootstrap-v2",
            policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes: &POLICY_AUTHORITY_PUBLIC_KEY,
            key_generation: 1,
        }
    }

    #[test]
    fn frozen_domain_scoped_generation_digest_is_stable() {
        assert_eq!(
            derive_historical_activation_time_policy_authority_key_generation_digest_v2(
                &domain(),
                body(),
            )
            .unwrap(),
            [
                0x59, 0x08, 0x7d, 0x74, 0x3c, 0x4b, 0x06, 0xb5, 0x5a, 0xfd, 0x8c, 0x47,
                0x21, 0x88, 0x66, 0x5b, 0x6c, 0xf9, 0x38, 0x08, 0x72, 0xe8, 0x62, 0xaa,
                0x40, 0xb7, 0x37, 0x7b, 0xed, 0x18, 0xb5, 0x7c,
            ]
        );
    }

    #[test]
    fn exact_generation_binds_to_transition() {
        let domain = domain();
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            body(),
        )
        .unwrap();
        let transition = prepare_time_policy_adoption_transition_v2(
            &domain,
            &policy(),
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
        assert_eq!(
            validate_policy_authority_key_generation_against_transition_v2(&generation, &transition),
            Ok(())
        );
    }

    #[test]
    fn same_material_in_other_domain_has_different_generation_identity() {
        let first_domain = domain();
        let first = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &first_domain,
            body(),
        )
        .unwrap();
        let mut other_dna = DNA;
        other_dna[11] ^= 0x40;
        let second_domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &other_dna,
        })
        .unwrap();
        let second = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &second_domain,
            body(),
        )
        .unwrap();
        assert_ne!(first.generation_digest_sha256(), second.generation_digest_sha256());
    }

    #[test]
    fn invalid_key_shape_and_zero_generation_fail_closed() {
        let domain = domain();
        let bad_key = [0x66; 31];
        let mut invalid = body();
        invalid.public_key_bytes = &bad_key;
        assert_eq!(
            qualify_historical_activation_time_policy_authority_key_generation_v2(&domain, invalid)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::PublicKeyLengthInvalid
        );
        let mut invalid = body();
        invalid.key_generation = 0;
        assert_eq!(
            qualify_historical_activation_time_policy_authority_key_generation_v2(&domain, invalid)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2::KeyGenerationInvalid
        );
    }

    #[test]
    fn qualified_generation_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2")
            .unwrap()
            + start;
        let result = &source[start..end];
        for field in [
            "pub authority_domain_sha256:",
            "pub public_key_bytes:",
            "pub generation_digest_sha256:",
        ] {
            assert!(!result.contains(field));
        }
    }
}

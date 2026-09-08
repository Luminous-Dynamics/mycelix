// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact time-authority key-generation identity for Identity V2 historical activation.
//!
//! This theorem binds one authority/key ID to one exact signature algorithm, raw public
//! key, generation number, and declared signing-validity interval. It does not establish
//! provenance, currentness, revocation state, or signature authenticity.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_authority_policy::QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const TIME_AUTHORITY_ID_MAX_LEN_V2: usize = 256;
pub const TIME_AUTHORITY_KEY_ID_MAX_LEN_V2: usize = 512;
pub const HISTORICAL_ACTIVATION_TIME_AUTHORITY_KEY_GENERATION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-authority-key-generation:v2\0";

#[derive(Debug, Clone, Copy)]
pub struct HistoricalActivationTimeAuthorityKeyGenerationV2<'a> {
    pub time_authority_id: &'a str,
    pub time_authority_key_id: &'a str,
    pub algorithm: AlgorithmId,
    pub public_key_bytes: &'a [u8],
    pub key_generation: u64,
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
}

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2 {
    generation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_id: String,
    time_authority_key_id: String,
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    key_generation: u64,
    valid_from_micros: i64,
    valid_until_micros: i64,
}

impl QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2 {
    pub fn generation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.generation_digest_sha256
    }

    pub fn time_authority_id(&self) -> &str {
        &self.time_authority_id
    }

    pub fn time_authority_key_id(&self) -> &str {
        &self.time_authority_key_id
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

    pub fn valid_from_micros(&self) -> i64 {
        self.valid_from_micros
    }

    pub fn valid_until_micros(&self) -> i64 {
        self.valid_until_micros
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimeAuthorityKeyGenerationErrorV2 {
    TimeAuthorityIdInvalid,
    TimeAuthorityKeyIdInvalid,
    NonSignatureAlgorithm,
    PublicKeyLengthInvalid,
    PublicKeyAllZero,
    KeyGenerationInvalid,
    ValidityInvalid,
    PolicyAuthorityIdMismatch,
    PolicyKeyIdMismatch,
    PolicyAlgorithmMismatch,
    PolicyKeyGenerationDigestMismatch,
    PolicyValidityOutsideKeyGeneration,
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
    generation: HistoricalActivationTimeAuthorityKeyGenerationV2<'_>,
) -> Result<(), HistoricalActivationTimeAuthorityKeyGenerationErrorV2> {
    if !valid_visible_ascii_identifier(generation.time_authority_id, TIME_AUTHORITY_ID_MAX_LEN_V2) {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::TimeAuthorityIdInvalid);
    }
    if !valid_visible_ascii_identifier(
        generation.time_authority_key_id,
        TIME_AUTHORITY_KEY_ID_MAX_LEN_V2,
    ) {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::TimeAuthorityKeyIdInvalid);
    }
    if !generation.algorithm.is_signature_algorithm() {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::NonSignatureAlgorithm);
    }
    if generation.public_key_bytes.len() != generation.algorithm.public_key_size() {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PublicKeyLengthInvalid);
    }
    if generation.public_key_bytes.iter().all(|byte| *byte == 0) {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PublicKeyAllZero);
    }
    if generation.key_generation == 0 {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::KeyGenerationInvalid);
    }
    if generation.valid_from_micros <= 0
        || generation.valid_until_micros <= generation.valid_from_micros
    {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::ValidityInvalid);
    }
    Ok(())
}

fn update_len_prefixed_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

pub fn derive_historical_activation_time_authority_key_generation_digest_v2(
    generation: HistoricalActivationTimeAuthorityKeyGenerationV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimeAuthorityKeyGenerationErrorV2> {
    validate_generation_body_v2(generation)?;
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_AUTHORITY_KEY_GENERATION_DOMAIN_V2);
    update_len_prefixed_v2(&mut hasher, 0x01, generation.time_authority_id.as_bytes());
    update_len_prefixed_v2(&mut hasher, 0x02, generation.time_authority_key_id.as_bytes());
    hasher.update([0x03]);
    hasher.update(generation.algorithm.as_u16().to_be_bytes());
    hasher.update([0x04]);
    hasher.update(generation.key_generation.to_be_bytes());
    update_len_prefixed_v2(&mut hasher, 0x05, generation.public_key_bytes);
    hasher.update([0x06]);
    hasher.update(generation.valid_from_micros.to_be_bytes());
    hasher.update([0x07]);
    hasher.update(generation.valid_until_micros.to_be_bytes());
    Ok(hasher.finalize().into())
}

pub fn qualify_historical_activation_time_authority_key_generation_v2(
    generation: HistoricalActivationTimeAuthorityKeyGenerationV2<'_>,
) -> Result<
    QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2,
    HistoricalActivationTimeAuthorityKeyGenerationErrorV2,
> {
    validate_generation_body_v2(generation)?;
    let generation_digest_sha256 =
        derive_historical_activation_time_authority_key_generation_digest_v2(generation)?;
    Ok(QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2 {
        generation_digest_sha256,
        time_authority_id: generation.time_authority_id.to_string(),
        time_authority_key_id: generation.time_authority_key_id.to_string(),
        algorithm: generation.algorithm,
        public_key_bytes: generation.public_key_bytes.to_vec(),
        key_generation: generation.key_generation,
        valid_from_micros: generation.valid_from_micros,
        valid_until_micros: generation.valid_until_micros,
    })
}

/// Require one exact key generation to be the key generation named by one #395 static
/// time-authority policy. The policy's entire static validity interval must be contained
/// within the generation's declared signing-validity interval.
pub fn validate_time_authority_key_generation_against_static_policy_v2(
    generation: &QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
) -> Result<(), HistoricalActivationTimeAuthorityKeyGenerationErrorV2> {
    if generation.time_authority_id() != policy.time_authority_id() {
        return Err(
            HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PolicyAuthorityIdMismatch,
        );
    }
    if generation.time_authority_key_id() != policy.time_authority_key_id() {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PolicyKeyIdMismatch);
    }
    if generation.algorithm() != policy.algorithm() {
        return Err(HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PolicyAlgorithmMismatch);
    }
    if generation.generation_digest_sha256() != policy.time_authority_key_generation_sha256() {
        return Err(
            HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PolicyKeyGenerationDigestMismatch,
        );
    }
    if policy.valid_from_micros() < generation.valid_from_micros()
        || policy.valid_until_micros() > generation.valid_until_micros()
    {
        return Err(
            HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PolicyValidityOutsideKeyGeneration,
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
    };
    use mycelix_historical_activation_time_receipt_policy::TimeBasisV2;

    static HYBRID_PUBLIC_KEY: [u8; 32 + 1952] = [0x44; 32 + 1952];

    fn generation() -> HistoricalActivationTimeAuthorityKeyGenerationV2<'static> {
        HistoricalActivationTimeAuthorityKeyGenerationV2 {
            time_authority_id: "time:authority:primary-v2",
            time_authority_key_id: "time:authority:primary-v2#hybrid-1",
            algorithm: AlgorithmId::HybridEd25519MlDsa65,
            public_key_bytes: &HYBRID_PUBLIC_KEY,
            key_generation: 1,
            valid_from_micros: 500_000,
            valid_until_micros: 20_000_000,
        }
    }

    #[test]
    fn frozen_time_authority_key_generation_digest_is_stable() {
        assert_eq!(
            derive_historical_activation_time_authority_key_generation_digest_v2(generation())
                .unwrap(),
            [
                0xfe, 0x10, 0x6b, 0x56, 0xc3, 0x78, 0x9c, 0x9a, 0x02, 0x3e, 0x6a, 0x7d,
                0x54, 0x20, 0x57, 0xa3, 0x34, 0x6b, 0x00, 0x9d, 0x4a, 0x48, 0x57, 0xd5,
                0x5e, 0x78, 0xbf, 0xac, 0x8f, 0xe5, 0xb9, 0xfb,
            ]
        );
    }

    #[test]
    fn static_policy_must_name_exact_generation_and_fit_its_validity() {
        let qualified_generation =
            qualify_historical_activation_time_authority_key_generation_v2(generation()).unwrap();
        let policy = qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:primary-v2",
                policy_version: 1,
                time_authority_id: qualified_generation.time_authority_id(),
                time_authority_key_id: qualified_generation.time_authority_key_id(),
                time_authority_key_generation_sha256: qualified_generation.generation_digest_sha256(),
                algorithm: qualified_generation.algorithm(),
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
        assert_eq!(
            validate_time_authority_key_generation_against_static_policy_v2(
                &qualified_generation,
                &policy,
            ),
            Ok(())
        );
    }

    #[test]
    fn public_key_substitution_changes_generation_identity() {
        let a = derive_historical_activation_time_authority_key_generation_digest_v2(generation())
            .unwrap();
        let mut alternate = HYBRID_PUBLIC_KEY;
        alternate[0] ^= 0x01;
        let mut changed = generation();
        changed.public_key_bytes = &alternate;
        let b = derive_historical_activation_time_authority_key_generation_digest_v2(changed)
            .unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn malformed_public_key_shape_fails_closed() {
        let mut invalid = generation();
        invalid.public_key_bytes = &[0x44; 32];
        assert_eq!(
            qualify_historical_activation_time_authority_key_generation_v2(invalid).unwrap_err(),
            HistoricalActivationTimeAuthorityKeyGenerationErrorV2::PublicKeyLengthInvalid
        );
    }

    #[test]
    fn qualified_generation_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationTimeAuthorityKeyGenerationV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub generation_digest_sha256:",
            "pub public_key_bytes:",
            "pub key_generation:",
            "pub valid_until_micros:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

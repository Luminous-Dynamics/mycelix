// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Domain-bound transition body for authoritative historical time-policy lineage.
//!
//! This pure theorem freezes one candidate adoption/supersession/revocation transition.
//! It binds an exact #369 Identity authority domain, exact #395 static policy identities,
//! monotonic transition generation/predecessor identity, and the independent policy-
//! authority key generation expected to authenticate the transition later.
//!
//! Success here is only a prepared signed transition statement. It does not authenticate
//! the policy-authority signature, prove complete lineage, select a terminal transition,
//! establish accepted/current policy, trust a time receipt, or grant historical activation.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_authority_policy::QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2;
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const POLICY_AUTHORITY_ID_MAX_LEN_V2: usize = 256;
pub const POLICY_AUTHORITY_KEY_ID_MAX_LEN_V2: usize = 512;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-transition:v2\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HistoricalActivationTimePolicyTransitionKindV2 {
    Adopt = 1,
    Supersede = 2,
    Revoke = 3,
}

impl HistoricalActivationTimePolicyTransitionKindV2 {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HistoricalActivationTimePolicyAuthoritySignerV2<'a> {
    pub policy_authority_id: &'a str,
    pub policy_authority_key_id: &'a str,
    pub policy_authority_key_generation_sha256: &'a [u8],
    pub algorithm: AlgorithmId,
}

#[derive(Debug)]
pub struct PreparedHistoricalActivationTimePolicyTransitionV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_generation: u64,
    predecessor_transition_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    transition_kind: HistoricalActivationTimePolicyTransitionKindV2,
    prior_policy_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    resulting_policy_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    policy_authority_id: String,
    policy_authority_key_id: String,
    policy_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl PreparedHistoricalActivationTimePolicyTransitionV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn transition_generation(&self) -> u64 {
        self.transition_generation
    }

    pub fn predecessor_transition_sha256(&self) -> Option<&[u8; SHA256_DIGEST_LEN_V2]> {
        self.predecessor_transition_sha256.as_ref()
    }

    pub fn transition_kind(&self) -> HistoricalActivationTimePolicyTransitionKindV2 {
        self.transition_kind
    }

    pub fn prior_policy_sha256(&self) -> Option<&[u8; SHA256_DIGEST_LEN_V2]> {
        self.prior_policy_sha256.as_ref()
    }

    pub fn resulting_policy_sha256(&self) -> Option<&[u8; SHA256_DIGEST_LEN_V2]> {
        self.resulting_policy_sha256.as_ref()
    }

    pub fn policy_authority_id(&self) -> &str {
        &self.policy_authority_id
    }

    pub fn policy_authority_key_id(&self) -> &str {
        &self.policy_authority_key_id
    }

    pub fn policy_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.policy_authority_key_generation_sha256
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.transition_signing_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyTransitionErrorV2 {
    TransitionGenerationInvalid,
    RootPredecessorPresent,
    NonRootPredecessorMissing,
    PredecessorDigestLengthInvalid,
    PredecessorDigestAllZero,
    PolicyAuthorityIdInvalid,
    PolicyAuthorityKeyIdInvalid,
    PolicyAuthorityKeyGenerationDigestLengthInvalid,
    PolicyAuthorityKeyGenerationDigestAllZero,
    NonSignatureAlgorithm,
    SupersessionPolicyUnchanged,
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

fn require_digest_v2(
    value: &[u8],
    length_error: HistoricalActivationTimePolicyTransitionErrorV2,
    zero_error: HistoricalActivationTimePolicyTransitionErrorV2,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimePolicyTransitionErrorV2> {
    let digest: [u8; SHA256_DIGEST_LEN_V2] = value.try_into().map_err(|_| length_error)?;
    if digest.iter().all(|byte| *byte == 0) {
        return Err(zero_error);
    }
    Ok(digest)
}

fn validate_generation_and_predecessor_v2(
    generation: u64,
    predecessor: Option<&[u8]>,
) -> Result<Option<[u8; SHA256_DIGEST_LEN_V2]>, HistoricalActivationTimePolicyTransitionErrorV2> {
    if generation == 0 {
        return Err(HistoricalActivationTimePolicyTransitionErrorV2::TransitionGenerationInvalid);
    }
    match (generation, predecessor) {
        (1, None) => Ok(None),
        (1, Some(_)) => Err(HistoricalActivationTimePolicyTransitionErrorV2::RootPredecessorPresent),
        (_, None) => Err(HistoricalActivationTimePolicyTransitionErrorV2::NonRootPredecessorMissing),
        (_, Some(value)) => Ok(Some(require_digest_v2(
            value,
            HistoricalActivationTimePolicyTransitionErrorV2::PredecessorDigestLengthInvalid,
            HistoricalActivationTimePolicyTransitionErrorV2::PredecessorDigestAllZero,
        )?)),
    }
}

fn validate_signer_v2(
    signer: HistoricalActivationTimePolicyAuthoritySignerV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimePolicyTransitionErrorV2> {
    if !valid_visible_ascii_identifier(signer.policy_authority_id, POLICY_AUTHORITY_ID_MAX_LEN_V2) {
        return Err(HistoricalActivationTimePolicyTransitionErrorV2::PolicyAuthorityIdInvalid);
    }
    if !valid_visible_ascii_identifier(
        signer.policy_authority_key_id,
        POLICY_AUTHORITY_KEY_ID_MAX_LEN_V2,
    ) {
        return Err(HistoricalActivationTimePolicyTransitionErrorV2::PolicyAuthorityKeyIdInvalid);
    }
    let generation = require_digest_v2(
        signer.policy_authority_key_generation_sha256,
        HistoricalActivationTimePolicyTransitionErrorV2::PolicyAuthorityKeyGenerationDigestLengthInvalid,
        HistoricalActivationTimePolicyTransitionErrorV2::PolicyAuthorityKeyGenerationDigestAllZero,
    )?;
    if !signer.algorithm.is_signature_algorithm() {
        return Err(HistoricalActivationTimePolicyTransitionErrorV2::NonSignatureAlgorithm);
    }
    Ok(generation)
}

fn update_len_prefixed_u16_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u16).to_be_bytes());
    hasher.update(value);
}

fn update_optional_digest_v2(
    hasher: &mut Sha256,
    tag: u8,
    value: Option<&[u8; SHA256_DIGEST_LEN_V2]>,
) {
    hasher.update([tag]);
    match value {
        Some(digest) => {
            hasher.update([1]);
            hasher.update(digest);
        }
        None => hasher.update([0]),
    }
}

fn prepare_transition_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    transition_generation: u64,
    predecessor_transition_sha256: Option<&[u8]>,
    transition_kind: HistoricalActivationTimePolicyTransitionKindV2,
    prior_policy_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    resulting_policy_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    signer: HistoricalActivationTimePolicyAuthoritySignerV2<'_>,
) -> Result<PreparedHistoricalActivationTimePolicyTransitionV2, HistoricalActivationTimePolicyTransitionErrorV2> {
    let predecessor_transition_sha256 =
        validate_generation_and_predecessor_v2(transition_generation, predecessor_transition_sha256)?;
    let policy_authority_key_generation_sha256 = validate_signer_v2(signer)?;

    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(authority_domain.digest_sha256());
    hasher.update([0x02]);
    hasher.update(transition_generation.to_be_bytes());
    update_optional_digest_v2(&mut hasher, 0x03, predecessor_transition_sha256.as_ref());
    hasher.update([0x04]);
    hasher.update([transition_kind.as_u8()]);
    update_optional_digest_v2(&mut hasher, 0x05, prior_policy_sha256.as_ref());
    update_optional_digest_v2(&mut hasher, 0x06, resulting_policy_sha256.as_ref());
    update_len_prefixed_u16_v2(&mut hasher, 0x07, signer.policy_authority_id.as_bytes());
    update_len_prefixed_u16_v2(&mut hasher, 0x08, signer.policy_authority_key_id.as_bytes());
    hasher.update([0x09]);
    hasher.update(policy_authority_key_generation_sha256);
    hasher.update([0x0a]);
    hasher.update(signer.algorithm.as_u16().to_be_bytes());
    let transition_signing_digest_sha256 = hasher.finalize().into();

    Ok(PreparedHistoricalActivationTimePolicyTransitionV2 {
        authority_domain_sha256: *authority_domain.digest_sha256(),
        transition_generation,
        predecessor_transition_sha256,
        transition_kind,
        prior_policy_sha256,
        resulting_policy_sha256,
        policy_authority_id: signer.policy_authority_id.to_string(),
        policy_authority_key_id: signer.policy_authority_key_id.to_string(),
        policy_authority_key_generation_sha256,
        algorithm: signer.algorithm,
        transition_signing_digest_sha256,
    })
}

pub fn prepare_time_policy_adoption_transition_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    transition_generation: u64,
    predecessor_transition_sha256: Option<&[u8]>,
    signer: HistoricalActivationTimePolicyAuthoritySignerV2<'_>,
) -> Result<PreparedHistoricalActivationTimePolicyTransitionV2, HistoricalActivationTimePolicyTransitionErrorV2> {
    prepare_transition_v2(
        authority_domain,
        transition_generation,
        predecessor_transition_sha256,
        HistoricalActivationTimePolicyTransitionKindV2::Adopt,
        None,
        Some(*policy.policy_digest_sha256()),
        signer,
    )
}

pub fn prepare_time_policy_supersession_transition_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    old_policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    new_policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    transition_generation: u64,
    predecessor_transition_sha256: &[u8],
    signer: HistoricalActivationTimePolicyAuthoritySignerV2<'_>,
) -> Result<PreparedHistoricalActivationTimePolicyTransitionV2, HistoricalActivationTimePolicyTransitionErrorV2> {
    if old_policy.policy_digest_sha256() == new_policy.policy_digest_sha256() {
        return Err(HistoricalActivationTimePolicyTransitionErrorV2::SupersessionPolicyUnchanged);
    }
    prepare_transition_v2(
        authority_domain,
        transition_generation,
        Some(predecessor_transition_sha256),
        HistoricalActivationTimePolicyTransitionKindV2::Supersede,
        Some(*old_policy.policy_digest_sha256()),
        Some(*new_policy.policy_digest_sha256()),
        signer,
    )
}

pub fn prepare_time_policy_revocation_transition_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    transition_generation: u64,
    predecessor_transition_sha256: &[u8],
    signer: HistoricalActivationTimePolicyAuthoritySignerV2<'_>,
) -> Result<PreparedHistoricalActivationTimePolicyTransitionV2, HistoricalActivationTimePolicyTransitionErrorV2> {
    prepare_transition_v2(
        authority_domain,
        transition_generation,
        Some(predecessor_transition_sha256),
        HistoricalActivationTimePolicyTransitionKindV2::Revoke,
        Some(*policy.policy_digest_sha256()),
        None,
        signer,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
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
    static POLICY_AUTHORITY_KEY_GENERATION: [u8; 32] = [0x44; 32];

    fn domain() -> QualifiedIdentityAuthorityDomainV2 {
        qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap()
    }

    fn policy(realization: &'static str) -> QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2 {
        qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:primary-v2",
                policy_version: 1,
                time_authority_id: "time:authority:primary-v2",
                time_authority_key_id: "time:authority:primary-v2#hybrid-1",
                time_authority_key_generation_sha256: &TIME_KEY_GENERATION,
                algorithm: AlgorithmId::HybridEd25519MlDsa65,
                time_basis: TimeBasisV2::UnixMicrosecondsUtc,
                utc_realization_id: realization,
                max_uncertainty_before_micros: 10_000,
                max_uncertainty_after_micros: 10_000,
                max_receipt_lifetime_micros: 1_000_000,
                valid_from_micros: 1_000_000,
                valid_until_micros: 10_000_000,
            },
        )
        .unwrap()
    }

    fn signer() -> HistoricalActivationTimePolicyAuthoritySignerV2<'static> {
        HistoricalActivationTimePolicyAuthoritySignerV2 {
            policy_authority_id: "identity:policy-authority:bootstrap-v2",
            policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
            policy_authority_key_generation_sha256: &POLICY_AUTHORITY_KEY_GENERATION,
            algorithm: AlgorithmId::Ed25519,
        }
    }

    #[test]
    fn frozen_root_adoption_digest_is_stable() {
        let transition = prepare_time_policy_adoption_transition_v2(
            &domain(),
            &policy("unix-utc-normalized-v1"),
            1,
            None,
            signer(),
        )
        .unwrap();
        assert_eq!(
            transition.transition_signing_digest_sha256(),
            &[
                0xf7, 0x93, 0xf7, 0x9c, 0x9f, 0xc8, 0x3e, 0xb7, 0x12, 0x65, 0x03, 0xf6,
                0xc3, 0xda, 0x73, 0x47, 0xa7, 0xcb, 0x89, 0x9d, 0x3f, 0xca, 0x0b, 0x0d,
                0x20, 0x5e, 0x4b, 0x30, 0x8b, 0x7f, 0x7b, 0xef,
            ]
        );
        assert_eq!(transition.transition_generation(), 1);
        assert_eq!(transition.transition_kind(), HistoricalActivationTimePolicyTransitionKindV2::Adopt);
        assert!(transition.predecessor_transition_sha256().is_none());
        assert!(transition.prior_policy_sha256().is_none());
        assert_eq!(
            transition.resulting_policy_sha256(),
            Some(policy("unix-utc-normalized-v1").policy_digest_sha256())
        );
    }

    #[test]
    fn root_and_nonroot_predecessor_rules_fail_closed() {
        let root_with_parent = [0x55; 32];
        assert_eq!(
            prepare_time_policy_adoption_transition_v2(
                &domain(),
                &policy("unix-utc-normalized-v1"),
                1,
                Some(&root_with_parent),
                signer(),
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyTransitionErrorV2::RootPredecessorPresent
        );
        assert_eq!(
            prepare_time_policy_adoption_transition_v2(
                &domain(),
                &policy("unix-utc-normalized-v1"),
                2,
                None,
                signer(),
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyTransitionErrorV2::NonRootPredecessorMissing
        );
    }

    #[test]
    fn supersession_requires_distinct_policy_identity() {
        let p = policy("unix-utc-normalized-v1");
        assert_eq!(
            prepare_time_policy_supersession_transition_v2(
                &domain(),
                &p,
                &p,
                2,
                &[0x55; 32],
                signer(),
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyTransitionErrorV2::SupersessionPolicyUnchanged
        );
    }

    #[test]
    fn domain_is_cryptographically_bound() {
        let first = prepare_time_policy_adoption_transition_v2(
            &domain(),
            &policy("unix-utc-normalized-v1"),
            1,
            None,
            signer(),
        )
        .unwrap();
        let mut other_dna = DNA;
        other_dna[10] ^= 0x80;
        let other_domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &other_dna,
        })
        .unwrap();
        let second = prepare_time_policy_adoption_transition_v2(
            &other_domain,
            &policy("unix-utc-normalized-v1"),
            1,
            None,
            signer(),
        )
        .unwrap();
        assert_ne!(first.transition_signing_digest_sha256(), second.transition_signing_digest_sha256());
    }

    #[test]
    fn prepared_transition_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct PreparedHistoricalActivationTimePolicyTransitionV2")
            .unwrap();
        let end = source[start..]
            .index("impl PreparedHistoricalActivationTimePolicyTransitionV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_sha256:",
            "pub predecessor_transition_sha256:",
            "pub prior_policy_sha256:",
            "pub resulting_policy_sha256:",
            "pub transition_signing_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

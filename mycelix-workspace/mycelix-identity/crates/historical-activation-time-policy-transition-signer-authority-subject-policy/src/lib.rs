// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical semantic authority subject for one historical time-policy transition signer generation.
//!
//! This pure theorem projects one already-qualified #403 policy-authority key generation into
//! exactly one Identity-owned operational authority subject. It establishes identity only.
//! Constitution-rooted authorization/freshness remains owned by the generic authority plane.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_authority_key_generation_policy::QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_SUBJECT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-transition-signer-authority-subject:v2\0";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_PROFILE_V2: &str =
    "mycelix-identity-time-policy-transition-signer-authority-v2-sha256-framed";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_NAMESPACE_V2: &str =
    "identity:time-policy:transition-authority";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_CAPABILITY_V2: &str =
    "identity.time-policy.transition-authority";

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_id: String,
    policy_authority_key_id: String,
    key_generation: u64,
    algorithm: AlgorithmId,
    subject_id: String,
    subject_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn policy_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.policy_authority_key_generation_sha256
    }

    pub fn policy_authority_id(&self) -> &str {
        &self.policy_authority_id
    }

    pub fn policy_authority_key_id(&self) -> &str {
        &self.policy_authority_key_id
    }

    pub fn key_generation(&self) -> u64 {
        self.key_generation
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn namespace(&self) -> &'static str {
        HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_NAMESPACE_V2
    }

    pub fn subject_id(&self) -> &str {
        &self.subject_id
    }

    pub fn capability(&self) -> &'static str {
        HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_CAPABILITY_V2
    }

    pub fn semantic_profile(&self) -> &'static str {
        HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_PROFILE_V2
    }

    pub fn subject_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.subject_digest_sha256
    }
}

fn update_len_prefixed_u16_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u16).to_be_bytes());
    hasher.update(value);
}

fn canonical_subject_id_v2(
    generation: &QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
) -> String {
    format!(
        "{}@generation:{}",
        generation.policy_authority_key_id(),
        generation.key_generation()
    )
}

pub fn derive_historical_activation_time_policy_transition_signer_authority_subject_digest_v2(
    generation: &QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let subject_id = canonical_subject_id_v2(generation);
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_SUBJECT_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(generation.authority_domain_sha256());
    hasher.update([0x02]);
    hasher.update(generation.generation_digest_sha256());
    update_len_prefixed_u16_v2(&mut hasher, 0x03, generation.policy_authority_id().as_bytes());
    update_len_prefixed_u16_v2(&mut hasher, 0x04, generation.policy_authority_key_id().as_bytes());
    hasher.update([0x05]);
    hasher.update(generation.key_generation().to_be_bytes());
    hasher.update([0x06]);
    hasher.update(generation.algorithm().as_u16().to_be_bytes());
    update_len_prefixed_u16_v2(
        &mut hasher,
        0x07,
        HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_NAMESPACE_V2.as_bytes(),
    );
    update_len_prefixed_u16_v2(&mut hasher, 0x08, subject_id.as_bytes());
    update_len_prefixed_u16_v2(
        &mut hasher,
        0x09,
        HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_CAPABILITY_V2.as_bytes(),
    );
    update_len_prefixed_u16_v2(
        &mut hasher,
        0x0a,
        HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_PROFILE_V2.as_bytes(),
    );
    hasher.finalize().into()
}

/// Freeze the unique semantic operational-authority subject represented by one #403 generation.
///
/// No caller-controlled authority metadata is accepted: namespace, subject ID, capability and
/// semantic profile are derived internally from the opaque generation plus fixed constants.
pub fn qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
    generation: &QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
) -> QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2 {
    let subject_id = canonical_subject_id_v2(generation);
    let subject_digest_sha256 =
        derive_historical_activation_time_policy_transition_signer_authority_subject_digest_v2(
            generation,
        );

    QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2 {
        authority_domain_sha256: *generation.authority_domain_sha256(),
        policy_authority_key_generation_sha256: *generation.generation_digest_sha256(),
        policy_authority_id: generation.policy_authority_id().to_string(),
        policy_authority_key_id: generation.policy_authority_key_id().to_string(),
        key_generation: generation.key_generation(),
        algorithm: generation.algorithm(),
        subject_id,
        subject_digest_sha256,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    };

    static DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];
    static POLICY_AUTHORITY_PUBLIC_KEY: [u8; 32] = [
        0xc8, 0x53, 0xad, 0x0f, 0x0c, 0xd2, 0xb6, 0x19, 0xae, 0xa9, 0x2c, 0xee, 0xc4, 0xfd,
        0x56, 0xa2, 0x4d, 0x64, 0x99, 0xd5, 0x84, 0xce, 0x79, 0x25, 0x7e, 0x45, 0xcf, 0xd8,
        0x13, 0x9b, 0x60, 0xa7,
    ];

    fn generation(
        dna: &[u8],
    ) -> QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: dna,
        })
        .unwrap();
        qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &POLICY_AUTHORITY_PUBLIC_KEY,
                key_generation: 1,
            },
        )
        .unwrap()
    }

    #[test]
    fn frozen_real_signer_subject_is_stable() {
        let generation = generation(&DNA);
        assert_eq!(
            generation.generation_digest_sha256(),
            &[
                0x40, 0xb4, 0xf4, 0xb9, 0x80, 0xee, 0xd2, 0xa4, 0xb2, 0xe8, 0xc2, 0x52,
                0x06, 0xee, 0xd4, 0xdd, 0xfe, 0x19, 0x62, 0x3e, 0x4f, 0x20, 0x70, 0xd8,
                0x6d, 0x76, 0x9f, 0x1a, 0x63, 0xac, 0x79, 0xc9,
            ]
        );
        let subject =
            qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
                &generation,
            );
        assert_eq!(
            subject.subject_digest_sha256(),
            &[
                0x69, 0xa8, 0x1f, 0xf8, 0x06, 0xe7, 0xfc, 0x9f, 0xe0, 0x9e, 0xa7, 0x4b,
                0xfe, 0xa2, 0x5c, 0x6f, 0xb8, 0x91, 0x69, 0x22, 0x32, 0x30, 0x73, 0xe1,
                0xa4, 0xe3, 0xbd, 0x0a, 0x2b, 0x72, 0xc2, 0x7a,
            ]
        );
        assert_eq!(
            subject.subject_id(),
            "identity:policy-authority:bootstrap-v2#ed25519-1@generation:1"
        );
        assert_eq!(
            subject.capability(),
            "identity.time-policy.transition-authority"
        );
        assert_eq!(
            subject.namespace(),
            "identity:time-policy:transition-authority"
        );
    }

    #[test]
    fn same_signer_material_in_another_dna_is_another_subject() {
        let first = qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
            &generation(&DNA),
        );
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second = qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
            &generation(&other_dna),
        );
        assert_ne!(first.authority_domain_sha256(), second.authority_domain_sha256());
        assert_ne!(
            first.policy_authority_key_generation_sha256(),
            second.policy_authority_key_generation_sha256()
        );
        assert_ne!(first.subject_digest_sha256(), second.subject_digest_sha256());
    }

    #[test]
    fn semantic_role_is_not_caller_controlled() {
        let generation = generation(&DNA);
        let subject =
            qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
                &generation,
            );
        assert_eq!(
            subject.semantic_profile(),
            HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_SIGNER_AUTHORITY_PROFILE_V2
        );
        assert_eq!(subject.algorithm(), AlgorithmId::Ed25519);
        assert_eq!(subject.key_generation(), 1);
    }

    #[test]
    fn qualified_subject_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_sha256:",
            "pub policy_authority_key_generation_sha256:",
            "pub subject_id:",
            "pub subject_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

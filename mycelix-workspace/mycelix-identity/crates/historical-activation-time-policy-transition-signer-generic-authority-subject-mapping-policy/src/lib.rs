// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical mapping contract from the Identity-owned #424 signer subject into the generic
//! authority subject shape expected after #74/#429/#446 ancestry convergence.
//!
//! This theorem does not import or emulate the generic authority implementation. It freezes the
//! only mapping components a later adapter is allowed to use, so namespace/kind/subject-id/profile
//! semantics cannot be invented at the convergence boundary.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V1: usize = 32;
pub const GENERIC_AUTHORITY_SUBJECT_KIND_V1: &str = "SigningPolicy";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_SIGNER_GENERIC_AUTHORITY_SUBJECT_MAPPING_PROFILE_V1: &str =
    "mycelix-identity-time-policy-signer-to-generic-authority-subject-v1-sha256-framed";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_SIGNER_GENERIC_AUTHORITY_SUBJECT_MAPPING_DOMAIN_V1: &[u8] =
    b"mycelix:identity:time-policy-transition-signer-generic-authority-subject-mapping:v1\0";

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1 {
    source_signer_subject_sha256: [u8; SHA256_DIGEST_LEN_V1],
    source_semantic_profile: String,
    source_capability: String,
    generic_subject_kind: String,
    generic_namespace: String,
    generic_subject_id: String,
    generic_identity_profile: String,
    generic_identity_digest: [u8; SHA256_DIGEST_LEN_V1],
    mapping_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
}

impl QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1 {
    pub fn source_signer_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.source_signer_subject_sha256
    }

    pub fn source_semantic_profile(&self) -> &str {
        &self.source_semantic_profile
    }

    pub fn source_capability(&self) -> &str {
        &self.source_capability
    }

    pub fn generic_subject_kind(&self) -> &str {
        &self.generic_subject_kind
    }

    pub fn generic_namespace(&self) -> &str {
        &self.generic_namespace
    }

    pub fn generic_subject_id(&self) -> &str {
        &self.generic_subject_id
    }

    pub fn generic_identity_profile(&self) -> &str {
        &self.generic_identity_profile
    }

    pub fn generic_identity_digest(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.generic_identity_digest
    }

    pub fn mapping_profile(&self) -> &'static str {
        HISTORICAL_ACTIVATION_TIME_POLICY_SIGNER_GENERIC_AUTHORITY_SUBJECT_MAPPING_PROFILE_V1
    }

    pub fn mapping_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.mapping_digest_sha256
    }
}

fn update_len_prefixed_u16_v1(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    let len = u16::try_from(value.len()).expect("fixed signer authority mapping strings fit u16");
    hasher.update([tag]);
    hasher.update(len.to_be_bytes());
    hasher.update(value);
}

fn derive_mapping_digest_v1(
    subject: &QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2,
) -> [u8; SHA256_DIGEST_LEN_V1] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_SIGNER_GENERIC_AUTHORITY_SUBJECT_MAPPING_DOMAIN_V1);
    hasher.update([0x01]);
    hasher.update(subject.subject_digest_sha256());
    update_len_prefixed_u16_v1(&mut hasher, 0x02, subject.semantic_profile().as_bytes());
    update_len_prefixed_u16_v1(&mut hasher, 0x03, subject.capability().as_bytes());
    update_len_prefixed_u16_v1(
        &mut hasher,
        0x04,
        GENERIC_AUTHORITY_SUBJECT_KIND_V1.as_bytes(),
    );
    update_len_prefixed_u16_v1(&mut hasher, 0x05, subject.namespace().as_bytes());
    update_len_prefixed_u16_v1(&mut hasher, 0x06, subject.subject_id().as_bytes());
    update_len_prefixed_u16_v1(&mut hasher, 0x07, subject.semantic_profile().as_bytes());
    hasher.update([0x08]);
    hasher.update(subject.subject_digest_sha256());
    hasher.finalize().into()
}

/// Freeze the only generic-authority subject mapping permitted for one exact #424 signer subject.
///
/// The later convergence adapter must map this record to exactly:
///
/// - generic kind `SigningPolicy`;
/// - namespace = #424 namespace;
/// - subject ID = #424 canonical subject ID;
/// - profiled identity profile = #424 semantic profile; and
/// - profiled identity digest bytes = #424 subject SHA-256 digest.
///
/// This theorem establishes mapping identity only. It does not construct the generic authority
/// subject object or claim current/historical authority.
pub fn qualify_historical_activation_time_policy_signer_generic_authority_subject_mapping_v1(
    subject: &QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2,
) -> QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1 {
    QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1 {
        source_signer_subject_sha256: *subject.subject_digest_sha256(),
        source_semantic_profile: subject.semantic_profile().to_string(),
        source_capability: subject.capability().to_string(),
        generic_subject_kind: GENERIC_AUTHORITY_SUBJECT_KIND_V1.to_string(),
        generic_namespace: subject.namespace().to_string(),
        generic_subject_id: subject.subject_id().to_string(),
        generic_identity_profile: subject.semantic_profile().to_string(),
        generic_identity_digest: *subject.subject_digest_sha256(),
        mapping_digest_sha256: derive_mapping_digest_v1(subject),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::AlgorithmId;
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::qualify_historical_activation_time_policy_transition_signer_authority_subject_v2;
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

    fn subject() -> QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2 {
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
        qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(&generation)
    }

    #[test]
    fn frozen_real_signer_mapping_is_stable() {
        let mapping =
            qualify_historical_activation_time_policy_signer_generic_authority_subject_mapping_v1(
                &subject(),
            );
        assert_eq!(
            mapping.source_signer_subject_sha256(),
            &[
                0x69, 0xa8, 0x1f, 0xf8, 0x06, 0xe7, 0xfc, 0x9f, 0xe0, 0x9e, 0xa7, 0x4b,
                0xfe, 0xa2, 0x5c, 0x6f, 0xb8, 0x91, 0x69, 0x22, 0x32, 0x30, 0x73, 0xe1,
                0xa4, 0xe3, 0xbd, 0x0a, 0x2b, 0x72, 0xc2, 0x7a,
            ]
        );
        assert_eq!(mapping.generic_subject_kind(), "SigningPolicy");
        assert_eq!(
            mapping.generic_namespace(),
            "identity:time-policy:transition-authority"
        );
        assert_eq!(
            mapping.generic_subject_id(),
            "identity:policy-authority:bootstrap-v2#ed25519-1@generation:1"
        );
        assert_eq!(
            mapping.generic_identity_profile(),
            "mycelix-identity-time-policy-transition-signer-authority-v2-sha256-framed"
        );
        assert_eq!(
            mapping.generic_identity_digest(),
            mapping.source_signer_subject_sha256()
        );
        assert_eq!(
            mapping.mapping_digest_sha256(),
            &[
                0xa5, 0xad, 0xb8, 0xa4, 0xb5, 0xcd, 0x8a, 0xf6, 0x42, 0xab, 0x5f, 0xf6,
                0x5c, 0xaa, 0x54, 0xac, 0x89, 0xb6, 0xb3, 0x0f, 0x9d, 0xc6, 0x13, 0x5c,
                0xa3, 0xa2, 0x54, 0x88, 0x69, 0x83, 0xdc, 0xbe,
            ]
        );
    }

    #[test]
    fn generic_mapping_fields_are_not_caller_controlled() {
        let mapping =
            qualify_historical_activation_time_policy_signer_generic_authority_subject_mapping_v1(
                &subject(),
            );
        assert_eq!(mapping.source_capability(), "identity.time-policy.transition-authority");
        assert_eq!(
            mapping.source_semantic_profile(),
            mapping.generic_identity_profile()
        );
    }

    #[test]
    fn qualified_mapping_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub generic_subject_kind:",
            "pub generic_namespace:",
            "pub generic_subject_id:",
            "pub generic_identity_profile:",
            "pub generic_identity_digest:",
            "pub mapping_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Authority-domain-scoped mapping from one exact #395 static time-authority policy into
//! the generic signing-policy subject shape used for accepted/current policy state.
//!
//! The raw #395 digest is intentionally insufficient for generic currentness because the same
//! static policy bytes can exist in more than one Identity DNA/authority epoch. This theorem
//! first binds the policy to the opaque #369 authority domain, then freezes the generic mapping.
//! It does not import or emulate generic authority/currentness implementation code.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_authority_policy::QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2;
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V1: usize = 32;
pub const GENERIC_AUTHORITY_SUBJECT_KIND_V1: &str = "SigningPolicy";
pub const GENERIC_AUTHORITY_NAMESPACE_V1: &str =
    "identity:historical-activation:time-authority-policy";
pub const TIME_AUTHORITY_POLICY_AUTHORITY_SCOPED_SUBJECT_PROFILE_V1: &str =
    "mycelix-identity-historical-activation-time-authority-policy-authority-scoped-v1-sha256-framed";
pub const TIME_AUTHORITY_POLICY_GENERIC_AUTHORITY_SUBJECT_MAPPING_PROFILE_V1: &str =
    "mycelix-identity-time-authority-policy-to-generic-authority-subject-v1-sha256-framed";
pub const TIME_AUTHORITY_POLICY_AUTHORITY_SCOPED_SUBJECT_DOMAIN_V1: &[u8] =
    b"mycelix:identity:historical-activation-time-authority-policy:authority-scoped:v1\0";
pub const TIME_AUTHORITY_POLICY_GENERIC_AUTHORITY_SUBJECT_MAPPING_DOMAIN_V1: &[u8] =
    b"mycelix:identity:time-authority-policy-generic-authority-subject-mapping:v1\0";

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V1],
    source_policy_sha256: [u8; SHA256_DIGEST_LEN_V1],
    source_policy_id: String,
    source_policy_version: u32,
    generic_subject_kind: String,
    generic_namespace: String,
    generic_subject_id: String,
    generic_identity_profile: String,
    generic_identity_digest: [u8; SHA256_DIGEST_LEN_V1],
    mapping_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
}

impl QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.authority_domain_sha256
    }

    pub fn source_policy_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.source_policy_sha256
    }

    pub fn source_policy_id(&self) -> &str {
        &self.source_policy_id
    }

    pub fn source_policy_version(&self) -> u32 {
        self.source_policy_version
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
        TIME_AUTHORITY_POLICY_GENERIC_AUTHORITY_SUBJECT_MAPPING_PROFILE_V1
    }

    pub fn mapping_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.mapping_digest_sha256
    }
}

fn update_len_prefixed_u16_v1(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    let len = u16::try_from(value.len()).expect("qualified policy mapping strings fit u16");
    hasher.update([tag]);
    hasher.update(len.to_be_bytes());
    hasher.update(value);
}

fn canonical_subject_id_v1(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
) -> String {
    format!(
        "{}@epoch:{}:{}@version:{}",
        authority_domain.authority_domain_id(),
        authority_domain.authority_domain_epoch(),
        policy.policy_id(),
        policy.policy_version()
    )
}

fn derive_authority_scoped_policy_subject_digest_v1(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
) -> [u8; SHA256_DIGEST_LEN_V1] {
    let mut hasher = Sha256::new();
    hasher.update(TIME_AUTHORITY_POLICY_AUTHORITY_SCOPED_SUBJECT_DOMAIN_V1);
    hasher.update([0x01]);
    hasher.update(authority_domain.digest_sha256());
    hasher.update([0x02]);
    hasher.update(policy.policy_digest_sha256());
    hasher.finalize().into()
}

fn derive_mapping_digest_v1(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    generic_subject_id: &str,
    generic_identity_digest: &[u8; SHA256_DIGEST_LEN_V1],
) -> [u8; SHA256_DIGEST_LEN_V1] {
    let mut hasher = Sha256::new();
    hasher.update(TIME_AUTHORITY_POLICY_GENERIC_AUTHORITY_SUBJECT_MAPPING_DOMAIN_V1);
    hasher.update([0x01]);
    hasher.update(authority_domain.digest_sha256());
    hasher.update([0x02]);
    hasher.update(policy.policy_digest_sha256());
    update_len_prefixed_u16_v1(&mut hasher, 0x03, policy.policy_id().as_bytes());
    hasher.update([0x04]);
    hasher.update(policy.policy_version().to_be_bytes());
    update_len_prefixed_u16_v1(
        &mut hasher,
        0x05,
        GENERIC_AUTHORITY_SUBJECT_KIND_V1.as_bytes(),
    );
    update_len_prefixed_u16_v1(
        &mut hasher,
        0x06,
        GENERIC_AUTHORITY_NAMESPACE_V1.as_bytes(),
    );
    update_len_prefixed_u16_v1(&mut hasher, 0x07, generic_subject_id.as_bytes());
    update_len_prefixed_u16_v1(
        &mut hasher,
        0x08,
        TIME_AUTHORITY_POLICY_AUTHORITY_SCOPED_SUBJECT_PROFILE_V1.as_bytes(),
    );
    hasher.update([0x09]);
    hasher.update(generic_identity_digest);
    hasher.finalize().into()
}

/// Freeze the only generic-authority subject mapping permitted for one exact #395 policy
/// inside one exact opaque #369 Identity authority domain.
///
/// The future convergence adapter must preserve this mapping exactly. This theorem does not
/// construct the generic authority subject object and does not claim accepted/current state.
pub fn qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
) -> QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1 {
    let generic_subject_id = canonical_subject_id_v1(authority_domain, policy);
    let generic_identity_digest =
        derive_authority_scoped_policy_subject_digest_v1(authority_domain, policy);
    let mapping_digest_sha256 = derive_mapping_digest_v1(
        authority_domain,
        policy,
        &generic_subject_id,
        &generic_identity_digest,
    );

    QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1 {
        authority_domain_sha256: *authority_domain.digest_sha256(),
        source_policy_sha256: *policy.policy_digest_sha256(),
        source_policy_id: policy.policy_id().to_string(),
        source_policy_version: policy.policy_version(),
        generic_subject_kind: GENERIC_AUTHORITY_SUBJECT_KIND_V1.to_string(),
        generic_namespace: GENERIC_AUTHORITY_NAMESPACE_V1.to_string(),
        generic_subject_id,
        generic_identity_profile: TIME_AUTHORITY_POLICY_AUTHORITY_SCOPED_SUBJECT_PROFILE_V1
            .to_string(),
        generic_identity_digest,
        mapping_digest_sha256,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::AlgorithmId;
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
    static KEY_GENERATION: [u8; 32] = [0x33; 32];

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
                time_authority_key_generation_sha256: &KEY_GENERATION,
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

    #[test]
    fn frozen_primary_policy_mapping_is_stable() {
        let domain = domain();
        let policy = policy();
        let mapping =
            qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(
                &domain,
                &policy,
            );
        assert_eq!(
            mapping.authority_domain_sha256(),
            &[
                0xf5, 0x04, 0xb4, 0x2c, 0xc8, 0x07, 0xe1, 0xc5, 0xd3, 0xd3, 0x78, 0x3a,
                0xd5, 0x41, 0xdb, 0x10, 0x36, 0x3b, 0x1c, 0x33, 0xab, 0xa5, 0xb1, 0xa6,
                0xbd, 0x30, 0xdc, 0x44, 0x62, 0xec, 0x1c, 0xd7,
            ]
        );
        assert_eq!(mapping.generic_subject_kind(), "SigningPolicy");
        assert_eq!(
            mapping.generic_namespace(),
            "identity:historical-activation:time-authority-policy"
        );
        assert_eq!(
            mapping.generic_subject_id(),
            "mycelix-identity-v2@epoch:1:time-policy:primary-v2@version:1"
        );
        assert_eq!(
            mapping.generic_identity_profile(),
            "mycelix-identity-historical-activation-time-authority-policy-authority-scoped-v1-sha256-framed"
        );
        assert_eq!(
            mapping.generic_identity_digest(),
            &[
                0x94, 0xce, 0xfc, 0xa9, 0x88, 0xec, 0xe2, 0x42, 0x49, 0x76, 0x51, 0x84,
                0x36, 0x1a, 0x5a, 0x09, 0x7b, 0x64, 0x51, 0x18, 0xc4, 0x51, 0x1a, 0xb9,
                0xf3, 0x1a, 0x40, 0xbd, 0x13, 0xfa, 0x9a, 0x7c,
            ]
        );
        assert_eq!(
            mapping.mapping_digest_sha256(),
            &[
                0x44, 0xef, 0x06, 0x95, 0xf6, 0xf6, 0xdb, 0x3a, 0x82, 0xdf, 0x72, 0xec,
                0xe6, 0xa2, 0xd5, 0xfe, 0x51, 0xcd, 0x62, 0x24, 0x41, 0x19, 0xce, 0x9a,
                0xc6, 0xfa, 0x30, 0xcd, 0xb2, 0x73, 0xad, 0x14,
            ]
        );
    }

    #[test]
    fn same_static_policy_in_different_dna_has_different_generic_identity() {
        let first_domain = domain();
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second_domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &other_dna,
        })
        .unwrap();
        let policy = policy();
        let first =
            qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(
                &first_domain,
                &policy,
            );
        let second =
            qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(
                &second_domain,
                &policy,
            );
        assert_eq!(first.source_policy_sha256(), second.source_policy_sha256());
        assert_ne!(first.generic_identity_digest(), second.generic_identity_digest());
        assert_ne!(first.mapping_digest_sha256(), second.mapping_digest_sha256());
    }

    #[test]
    fn policy_version_changes_generic_subject_id_and_mapping() {
        let domain = domain();
        let base = policy();
        let base_mapping =
            qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(
                &domain,
                &base,
            );
        let changed = qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:primary-v2",
                policy_version: 2,
                time_authority_id: "time:authority:primary-v2",
                time_authority_key_id: "time:authority:primary-v2#hybrid-1",
                time_authority_key_generation_sha256: &KEY_GENERATION,
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
        let changed_mapping =
            qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(
                &domain,
                &changed,
            );
        assert_ne!(base_mapping.generic_subject_id(), changed_mapping.generic_subject_id());
        assert_ne!(
            base_mapping.mapping_digest_sha256(),
            changed_mapping.mapping_digest_sha256()
        );
    }

    #[test]
    fn qualified_mapping_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_sha256:",
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

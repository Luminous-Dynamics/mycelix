// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact terminal time-authority policy candidate bound to one #454 authenticated lineage.
//!
//! #454 identifies the terminal policy digest of one supplied policy-transition topology whose
//! every transition has a strict authenticated causal-authority claim. #467 freezes the exact
//! authority-domain-scoped generic signing-policy subject for a concrete #395 policy.
//!
//! This theorem joins those opaque results. It cannot make the candidate current or accepted.
//! A terminal revocation (`terminal_policy == None`) fails closed rather than becoming an
//! absence-based positive authority claim.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_authority_policy::QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2;
use mycelix_historical_activation_time_authority_policy_generic_authority_subject_mapping_policy::QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1;
use mycelix_historical_activation_time_policy_authority_anchored_authenticated_lineage_policy::QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V1: usize = 32;
pub const TERMINAL_TIME_AUTHORITY_POLICY_CANDIDATE_DOMAIN_V1: &[u8] =
    b"mycelix:identity:historical-activation-time-authority-policy-terminal-candidate:v1\0";

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V1],
    lineage_qualification_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    transition_count: u32,
    terminal_transition_generation: u64,
    terminal_transition_sha256: [u8; SHA256_DIGEST_LEN_V1],
    terminal_policy_sha256: [u8; SHA256_DIGEST_LEN_V1],
    generic_policy_subject_identity_sha256: [u8; SHA256_DIGEST_LEN_V1],
    generic_policy_mapping_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    candidate_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
}

impl QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.authority_domain_sha256
    }

    pub fn lineage_qualification_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.lineage_qualification_digest_sha256
    }

    pub fn transition_count(&self) -> u32 {
        self.transition_count
    }

    pub fn terminal_transition_generation(&self) -> u64 {
        self.terminal_transition_generation
    }

    pub fn terminal_transition_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.terminal_transition_sha256
    }

    pub fn terminal_policy_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.terminal_policy_sha256
    }

    pub fn generic_policy_subject_identity_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.generic_policy_subject_identity_sha256
    }

    pub fn generic_policy_mapping_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.generic_policy_mapping_digest_sha256
    }

    pub fn candidate_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.candidate_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminalTimeAuthorityPolicyCandidateErrorV1 {
    TerminalPolicyRevoked,
    TerminalPolicyMismatch,
    AuthorityDomainMismatch,
    MappingPolicyMismatch,
    MappingPolicyIdMismatch,
    MappingPolicyVersionMismatch,
}

fn derive_candidate_digest_v1(
    lineage: &QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    mapping: &QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1,
) -> [u8; SHA256_DIGEST_LEN_V1] {
    let mut hasher = Sha256::new();
    hasher.update(TERMINAL_TIME_AUTHORITY_POLICY_CANDIDATE_DOMAIN_V1);
    hasher.update([0x01]);
    hasher.update(lineage.authority_domain_sha256());
    hasher.update([0x02]);
    hasher.update(lineage.qualification_digest_sha256());
    hasher.update([0x03]);
    hasher.update(lineage.transition_count().to_be_bytes());
    hasher.update([0x04]);
    hasher.update(lineage.terminal_transition_generation().to_be_bytes());
    hasher.update([0x05]);
    hasher.update(lineage.terminal_transition_sha256());
    hasher.update([0x06]);
    hasher.update(policy.policy_digest_sha256());
    hasher.update([0x07]);
    hasher.update(mapping.mapping_digest_sha256());
    hasher.update([0x08]);
    hasher.update(mapping.generic_identity_digest());
    hasher.finalize().into()
}

/// Bind one exact #454 terminal policy to the concrete #395 policy and its exact corrected #467
/// authority-domain-scoped generic signing-policy mapping.
///
/// Success is a candidate only. Generic constitution-rooted currentness must independently prove
/// that `generic_policy_subject_identity_sha256` is currently Active for this exact subject.
pub fn qualify_historical_activation_time_authority_policy_terminal_candidate_v1(
    lineage: &QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    mapping: &QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1,
) -> Result<
    QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1,
    TerminalTimeAuthorityPolicyCandidateErrorV1,
> {
    let terminal_policy = lineage
        .terminal_policy_sha256()
        .ok_or(TerminalTimeAuthorityPolicyCandidateErrorV1::TerminalPolicyRevoked)?;
    if terminal_policy != policy.policy_digest_sha256() {
        return Err(TerminalTimeAuthorityPolicyCandidateErrorV1::TerminalPolicyMismatch);
    }
    if mapping.authority_domain_sha256() != lineage.authority_domain_sha256() {
        return Err(TerminalTimeAuthorityPolicyCandidateErrorV1::AuthorityDomainMismatch);
    }
    if mapping.source_policy_sha256() != policy.policy_digest_sha256() {
        return Err(TerminalTimeAuthorityPolicyCandidateErrorV1::MappingPolicyMismatch);
    }
    if mapping.source_policy_id() != policy.policy_id() {
        return Err(TerminalTimeAuthorityPolicyCandidateErrorV1::MappingPolicyIdMismatch);
    }
    if mapping.source_policy_version() != policy.policy_version() {
        return Err(TerminalTimeAuthorityPolicyCandidateErrorV1::MappingPolicyVersionMismatch);
    }

    let candidate_digest_sha256 = derive_candidate_digest_v1(lineage, policy, mapping);
    Ok(QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1 {
        authority_domain_sha256: *lineage.authority_domain_sha256(),
        lineage_qualification_digest_sha256: *lineage.qualification_digest_sha256(),
        transition_count: lineage.transition_count(),
        terminal_transition_generation: lineage.terminal_transition_generation(),
        terminal_transition_sha256: *lineage.terminal_transition_sha256(),
        terminal_policy_sha256: *policy.policy_digest_sha256(),
        generic_policy_subject_identity_sha256: *mapping.generic_identity_digest(),
        generic_policy_mapping_digest_sha256: *mapping.mapping_digest_sha256(),
        candidate_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use mycelix_crypto::{AlgorithmId, TaggedSignature};
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
    };
    use mycelix_historical_activation_time_authority_policy_generic_authority_subject_mapping_policy::qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1;
    use mycelix_historical_activation_time_policy_authority_anchored_authenticated_lineage_policy::qualify_authority_anchored_authenticated_historical_activation_time_policy_lineage_v2;
    use mycelix_historical_activation_time_policy_authority_anchored_transition_crypto_request_policy::prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2;
    use mycelix_historical_activation_time_policy_authority_anchored_transition_ed25519_authenticity::authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2;
    use mycelix_historical_activation_time_policy_authority_anchored_transition_policy::{
        prepare_authority_anchored_historical_activation_time_policy_transition_v2,
        HistoricalActivationTimePolicyAuthorityStateAnchorV2,
    };
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_policy_transition_policy::{
        prepare_time_policy_adoption_transition_v2,
        HistoricalActivationTimePolicyAuthoritySignerV2,
    };
    use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::qualify_historical_activation_time_policy_transition_signer_authority_subject_v2;
    use mycelix_historical_activation_time_receipt_policy::TimeBasisV2;
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
        QualifiedIdentityAuthorityDomainV2,
    };

    static DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];
    static TIME_KEY_GENERATION: [u8; 32] = [0x33; 32];
    static AUTHORITY_STATE: [u8; 32] = [0x55; 32];

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

    #[test]
    fn frozen_terminal_candidate_is_stable() {
        let domain = domain();
        let policy = policy("unix-utc-normalized-v1");
        let signing_key = SigningKey::from_bytes(&[0x77; 32]);
        let public_key = signing_key.verifying_key().to_bytes();
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &public_key,
                key_generation: 1,
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
        assert_eq!(
            transition.transition_signing_digest_sha256(),
            &[
                0x51, 0x40, 0x02, 0x25, 0x80, 0x9b, 0x98, 0xc2, 0x65, 0x16, 0x2e, 0x46,
                0x92, 0x81, 0x79, 0xf3, 0x8e, 0x21, 0x6a, 0xcd, 0x70, 0x74, 0x5c, 0xd9,
                0x80, 0x46, 0xc8, 0x54, 0xf7, 0x05, 0x0d, 0x7e,
            ]
        );
        let subject = qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(&generation);
        let anchored = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &transition,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 1,
                authority_state_transition_digest: &AUTHORITY_STATE,
            },
        )
        .unwrap();
        let signature = TaggedSignature::new(
            AlgorithmId::Ed25519,
            signing_key
                .sign(anchored.anchored_transition_signing_digest_sha256())
                .to_bytes()
                .to_vec(),
        )
        .unwrap();
        let request = prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(
            &anchored,
            &generation,
            &signature,
        )
        .unwrap();
        let authenticity = authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(&request).unwrap();
        let lineage = qualify_authority_anchored_authenticated_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&transition],
            &[&anchored],
            &[&authenticity],
        )
        .unwrap();
        assert_eq!(
            lineage.qualification_digest_sha256(),
            &[
                0x45, 0x50, 0x22, 0xb1, 0x19, 0xf8, 0x1a, 0x84, 0x2c, 0x09, 0x2b, 0x6b,
                0x98, 0x6e, 0xfe, 0xeb, 0xa4, 0x01, 0x83, 0xa7, 0xf8, 0x82, 0x85, 0x69,
                0x40, 0x09, 0x37, 0x75, 0xcc, 0xd0, 0xe6, 0x45,
            ]
        );
        let mapping = qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(&domain, &policy);
        let candidate = qualify_historical_activation_time_authority_policy_terminal_candidate_v1(
            &lineage,
            &policy,
            &mapping,
        )
        .unwrap();
        assert_eq!(
            candidate.candidate_digest_sha256(),
            &[
                0x60, 0xf4, 0x16, 0xe1, 0x63, 0x82, 0x66, 0xa2, 0x18, 0x47, 0x6c, 0x8e,
                0xdf, 0x84, 0xc7, 0x77, 0x3f, 0x79, 0x06, 0xf6, 0x45, 0x38, 0xca, 0xf4,
                0x5d, 0x73, 0x7b, 0x0e, 0x65, 0xb4, 0x8d, 0x24,
            ]
        );
    }

    #[test]
    fn different_concrete_policy_cannot_be_substituted_for_terminal_digest() {
        let domain = domain();
        let terminal = policy("unix-utc-normalized-v1");
        let substituted = policy("unix-utc-smear-normalized-v1");
        let signing_key = SigningKey::from_bytes(&[0x77; 32]);
        let public_key = signing_key.verifying_key().to_bytes();
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &public_key,
                key_generation: 1,
            },
        )
        .unwrap();
        let transition = prepare_time_policy_adoption_transition_v2(
            &domain,
            &terminal,
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
        let subject = qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(&generation);
        let anchored = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &transition,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 1,
                authority_state_transition_digest: &AUTHORITY_STATE,
            },
        )
        .unwrap();
        let signature = TaggedSignature::new(
            AlgorithmId::Ed25519,
            signing_key.sign(anchored.anchored_transition_signing_digest_sha256()).to_bytes().to_vec(),
        )
        .unwrap();
        let request = prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(&anchored, &generation, &signature).unwrap();
        let authenticity = authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(&request).unwrap();
        let lineage = qualify_authority_anchored_authenticated_historical_activation_time_policy_lineage_v2(&domain, &[&transition], &[&anchored], &[&authenticity]).unwrap();
        let mapping = qualify_historical_activation_time_authority_policy_generic_authority_subject_mapping_v1(&domain, &substituted);
        assert_eq!(
            qualify_historical_activation_time_authority_policy_terminal_candidate_v1(
                &lineage,
                &substituted,
                &mapping,
            )
            .unwrap_err(),
            TerminalTimeAuthorityPolicyCandidateErrorV1::TerminalPolicyMismatch
        );
    }

    #[test]
    fn qualified_candidate_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub terminal_policy_sha256:",
            "pub generic_policy_subject_identity_sha256:",
            "pub candidate_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

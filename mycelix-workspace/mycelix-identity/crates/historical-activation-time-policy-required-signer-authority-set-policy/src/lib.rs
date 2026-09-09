// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Closed required signer-authority subject set for one authenticated historical time-policy lineage.
//!
//! This pure theorem re-runs #409 over the supplied lineage evidence, then requires an exact
//! closed set of opaque #403 signer generations for every transition. Each required generation
//! is projected through #424 into its canonical semantic authority subject. Success still says
//! nothing about whether any subject is constitutionally authorized/current.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_policy_authenticated_lineage_policy::{
    qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2,
    HistoricalActivationTimePolicyAuthenticatedLineageErrorV2,
    QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2,
};
use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
    validate_policy_authority_key_generation_against_transition_v2,
    HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2,
    QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
};
use mycelix_historical_activation_time_policy_lineage_policy::QualifiedObservedHistoricalActivationTimePolicyLineageV2;
use mycelix_historical_activation_time_policy_transition_ed25519_authenticity::CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2;
use mycelix_historical_activation_time_policy_transition_policy::PreparedHistoricalActivationTimePolicyTransitionV2;
use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::{
    qualify_historical_activation_time_policy_transition_signer_authority_subject_v2,
    QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2,
};
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use sha2::{Digest, Sha256};
use std::collections::{HashMap, HashSet};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const MAX_REQUIRED_SIGNER_GENERATIONS_V2: usize = 4096;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_REQUIRED_SIGNER_AUTHORITY_SET_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-required-signer-authority-set:v2\0";

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimePolicyRequiredSignerAuthoritySetV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    authenticated_lineage_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_count: u32,
    required_signer_count: u32,
    required_signer_subjects:
        Vec<QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2>,
    required_authority_set_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedHistoricalActivationTimePolicyRequiredSignerAuthoritySetV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn authenticated_lineage_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authenticated_lineage_digest_sha256
    }

    pub fn transition_count(&self) -> u32 {
        self.transition_count
    }

    pub fn required_signer_count(&self) -> u32 {
        self.required_signer_count
    }

    pub fn required_signer_subjects(
        &self,
    ) -> &[QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2] {
        &self.required_signer_subjects
    }

    pub fn required_authority_set_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.required_authority_set_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2 {
    AuthenticatedLineage(HistoricalActivationTimePolicyAuthenticatedLineageErrorV2),
    AuthenticatedLineageMismatch,
    InvalidSignerGenerationCount,
    DuplicateSignerGeneration,
    SignerGenerationDomainMismatch,
    MissingSignerGeneration,
    UnexpectedSignerGeneration,
    SignerGenerationBindingFailed(HistoricalActivationTimePolicyAuthorityKeyGenerationErrorV2),
}

impl From<HistoricalActivationTimePolicyAuthenticatedLineageErrorV2>
    for HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2
{
    fn from(value: HistoricalActivationTimePolicyAuthenticatedLineageErrorV2) -> Self {
        Self::AuthenticatedLineage(value)
    }
}

fn same_authenticated_lineage_v2(
    left: &QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2,
    right: &QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2,
) -> bool {
    left.authority_domain_sha256() == right.authority_domain_sha256()
        && left.transition_count() == right.transition_count()
        && left.terminal_transition_generation() == right.terminal_transition_generation()
        && left.terminal_transition_sha256() == right.terminal_transition_sha256()
        && left.terminal_policy_sha256() == right.terminal_policy_sha256()
        && left.observed_lineage_digest_sha256() == right.observed_lineage_digest_sha256()
        && left.authenticated_lineage_digest_sha256()
            == right.authenticated_lineage_digest_sha256()
}

fn update_len_prefixed_u16_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u16).to_be_bytes());
    hasher.update(value);
}

fn derive_required_authority_set_digest_v2(
    authority_domain_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    authenticated_lineage_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    transition_count: u32,
    subjects: &[QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2],
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_REQUIRED_SIGNER_AUTHORITY_SET_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(authority_domain_sha256);
    hasher.update([0x02]);
    hasher.update(authenticated_lineage_digest_sha256);
    hasher.update([0x03]);
    hasher.update(transition_count.to_be_bytes());
    hasher.update([0x04]);
    hasher.update((subjects.len() as u32).to_be_bytes());

    for subject in subjects {
        hasher.update([0x10]);
        hasher.update(subject.subject_digest_sha256());
        hasher.update([0x11]);
        hasher.update(subject.policy_authority_key_generation_sha256());
        hasher.update([0x12]);
        hasher.update(subject.algorithm().as_u16().to_be_bytes());
        update_len_prefixed_u16_v2(&mut hasher, 0x13, subject.namespace().as_bytes());
        update_len_prefixed_u16_v2(&mut hasher, 0x14, subject.subject_id().as_bytes());
        update_len_prefixed_u16_v2(&mut hasher, 0x15, subject.capability().as_bytes());
        update_len_prefixed_u16_v2(&mut hasher, 0x16, subject.semantic_profile().as_bytes());
    }

    hasher.finalize().into()
}

/// Bind one exact #409 authenticated lineage to the closed set of #424 signer subjects
/// whose generic constitutional authority must later be established independently.
pub fn qualify_historical_activation_time_policy_required_signer_authority_set_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    observed_lineage: &QualifiedObservedHistoricalActivationTimePolicyLineageV2,
    authenticated_lineage: &QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2,
    transitions: &[&PreparedHistoricalActivationTimePolicyTransitionV2],
    authentications: &[&CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2],
    signer_generations: &[&QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2],
) -> Result<
    QualifiedHistoricalActivationTimePolicyRequiredSignerAuthoritySetV2,
    HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2,
> {
    let rederived = qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2(
        authority_domain,
        observed_lineage,
        transitions,
        authentications,
    )?;
    if !same_authenticated_lineage_v2(&rederived, authenticated_lineage) {
        return Err(
            HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::AuthenticatedLineageMismatch,
        );
    }

    if signer_generations.is_empty()
        || signer_generations.len() > MAX_REQUIRED_SIGNER_GENERATIONS_V2
    {
        return Err(
            HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::InvalidSignerGenerationCount,
        );
    }

    let mut generation_by_digest = HashMap::with_capacity(signer_generations.len());
    for generation in signer_generations {
        if generation.authority_domain_sha256() != authority_domain.digest_sha256() {
            return Err(
                HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::SignerGenerationDomainMismatch,
            );
        }
        let digest = *generation.generation_digest_sha256();
        if generation_by_digest.insert(digest, *generation).is_some() {
            return Err(
                HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::DuplicateSignerGeneration,
            );
        }
    }

    let required_digests: HashSet<[u8; SHA256_DIGEST_LEN_V2]> = transitions
        .iter()
        .map(|transition| *transition.policy_authority_key_generation_sha256())
        .collect();

    for transition in transitions {
        let generation = generation_by_digest
            .get(transition.policy_authority_key_generation_sha256())
            .ok_or(
                HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::MissingSignerGeneration,
            )?;
        validate_policy_authority_key_generation_against_transition_v2(generation, transition)
            .map_err(
                HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::SignerGenerationBindingFailed,
            )?;
    }

    if generation_by_digest
        .keys()
        .any(|digest| !required_digests.contains(digest))
    {
        return Err(
            HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::UnexpectedSignerGeneration,
        );
    }
    if generation_by_digest.len() != required_digests.len() {
        return Err(
            HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::MissingSignerGeneration,
        );
    }

    let mut required_signer_subjects: Vec<_> = generation_by_digest
        .values()
        .map(|generation| {
            qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
                generation,
            )
        })
        .collect();
    required_signer_subjects.sort_by(|left, right| {
        left.subject_digest_sha256()
            .cmp(right.subject_digest_sha256())
    });

    let required_signer_count = u32::try_from(required_signer_subjects.len()).map_err(|_| {
        HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::InvalidSignerGenerationCount
    })?;
    let required_authority_set_digest_sha256 = derive_required_authority_set_digest_v2(
        authority_domain.digest_sha256(),
        authenticated_lineage.authenticated_lineage_digest_sha256(),
        authenticated_lineage.transition_count(),
        &required_signer_subjects,
    );

    Ok(QualifiedHistoricalActivationTimePolicyRequiredSignerAuthoritySetV2 {
        authority_domain_sha256: *authority_domain.digest_sha256(),
        authenticated_lineage_digest_sha256: *authenticated_lineage
            .authenticated_lineage_digest_sha256(),
        transition_count: authenticated_lineage.transition_count(),
        required_signer_count,
        required_signer_subjects,
        required_authority_set_digest_sha256,
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
        QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    };
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_policy_lineage_policy::qualify_observed_historical_activation_time_policy_lineage_v2;
    use mycelix_historical_activation_time_policy_transition_crypto_request_policy::prepare_historical_activation_time_policy_transition_crypto_request_v2;
    use mycelix_historical_activation_time_policy_transition_ed25519_authenticity::authenticate_historical_activation_time_policy_transition_ed25519_v2;
    use mycelix_historical_activation_time_policy_transition_policy::{
        prepare_time_policy_adoption_transition_v2,
        prepare_time_policy_supersession_transition_v2,
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

    fn generation(
        domain: &QualifiedIdentityAuthorityDomainV2,
        signing_key: &SigningKey,
        key_id: &'static str,
        key_generation: u64,
    ) -> QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
        let public_key = signing_key.verifying_key().to_bytes();
        qualify_historical_activation_time_policy_authority_key_generation_v2(
            domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: key_id,
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &public_key,
                key_generation,
            },
        )
        .unwrap()
    }

    fn authenticate(
        transition: &PreparedHistoricalActivationTimePolicyTransitionV2,
        generation: &QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
        signing_key: &SigningKey,
    ) -> CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2 {
        let signature = TaggedSignature::new(
            AlgorithmId::Ed25519,
            signing_key
                .sign(transition.transition_signing_digest_sha256())
                .to_bytes()
                .to_vec(),
        )
        .unwrap();
        let request = prepare_historical_activation_time_policy_transition_crypto_request_v2(
            transition,
            generation,
            &signature,
        )
        .unwrap();
        authenticate_historical_activation_time_policy_transition_ed25519_v2(&request).unwrap()
    }

    #[test]
    fn rotated_two_transition_lineage_requires_both_signer_subjects() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let signing_key_1 = SigningKey::from_bytes(&[0x77; 32]);
        let signing_key_2 = SigningKey::from_bytes(&[0x78; 32]);
        let generation_1 = generation(
            &domain,
            &signing_key_1,
            "identity:policy-authority:bootstrap-v2#ed25519-1",
            1,
        );
        let generation_2 = generation(
            &domain,
            &signing_key_2,
            "identity:policy-authority:bootstrap-v2#ed25519-2",
            2,
        );
        assert_eq!(
            generation_2.generation_digest_sha256(),
            &[
                0xba, 0x71, 0xdb, 0x21, 0x1c, 0xbb, 0xe1, 0x28, 0xc7, 0x21, 0x68, 0xe3,
                0x99, 0x40, 0x92, 0x13, 0x2c, 0x90, 0xa8, 0xff, 0x09, 0x41, 0xf0, 0xea,
                0x9c, 0xaf, 0x61, 0xe3, 0x47, 0x8c, 0x1d, 0x5b,
            ]
        );

        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            HistoricalActivationTimePolicyAuthoritySignerV2 {
                policy_authority_id: generation_1.policy_authority_id(),
                policy_authority_key_id: generation_1.policy_authority_key_id(),
                policy_authority_key_generation_sha256: generation_1.generation_digest_sha256(),
                algorithm: generation_1.algorithm(),
            },
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            HistoricalActivationTimePolicyAuthoritySignerV2 {
                policy_authority_id: generation_2.policy_authority_id(),
                policy_authority_key_id: generation_2.policy_authority_key_id(),
                policy_authority_key_generation_sha256: generation_2.generation_digest_sha256(),
                algorithm: generation_2.algorithm(),
            },
        )
        .unwrap();
        assert_eq!(
            second.transition_signing_digest_sha256(),
            &[
                0xf9, 0xc7, 0xa1, 0xa6, 0x6e, 0xd1, 0x78, 0xb6, 0xce, 0x6a, 0x23, 0xd6,
                0x51, 0x1e, 0xb8, 0x6f, 0x27, 0x40, 0xe2, 0x01, 0x39, 0x4b, 0xda, 0x37,
                0x56, 0x5e, 0xb2, 0x02, 0xab, 0x68, 0xc7, 0xa6,
            ]
        );

        let auth_1 = authenticate(&first, &generation_1, &signing_key_1);
        let auth_2 = authenticate(&second, &generation_2, &signing_key_2);
        let observed = qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&second, &first],
        )
        .unwrap();
        let authenticated = qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2(
            &domain,
            &observed,
            &[&second, &first],
            &[&auth_2, &auth_1],
        )
        .unwrap();
        assert_eq!(
            authenticated.authenticated_lineage_digest_sha256(),
            &[
                0xd0, 0xd7, 0x57, 0x85, 0x47, 0xab, 0x58, 0x2b, 0xbd, 0xce, 0x25, 0x10,
                0x43, 0x1f, 0x79, 0x3d, 0x0f, 0x79, 0xb3, 0x6d, 0x42, 0xf8, 0xe9, 0x1a,
                0x61, 0x56, 0x41, 0x3f, 0x6a, 0xa8, 0xb7, 0x37,
            ]
        );

        let required = qualify_historical_activation_time_policy_required_signer_authority_set_v2(
            &domain,
            &observed,
            &authenticated,
            &[&second, &first],
            &[&auth_2, &auth_1],
            &[&generation_2, &generation_1],
        )
        .unwrap();
        assert_eq!(required.transition_count(), 2);
        assert_eq!(required.required_signer_count(), 2);
        assert_eq!(
            required.required_signer_subjects()[0].subject_digest_sha256(),
            &[
                0x69, 0xa8, 0x1f, 0xf8, 0x06, 0xe7, 0xfc, 0x9f, 0xe0, 0x9e, 0xa7, 0x4b,
                0xfe, 0xa2, 0x5c, 0x6f, 0xb8, 0x91, 0x69, 0x22, 0x32, 0x30, 0x73, 0xe1,
                0xa4, 0xe3, 0xbd, 0x0a, 0x2b, 0x72, 0xc2, 0x7a,
            ]
        );
        assert_eq!(
            required.required_signer_subjects()[1].subject_digest_sha256(),
            &[
                0xeb, 0xf7, 0xbc, 0xe8, 0x1b, 0xbd, 0xb8, 0xd4, 0x16, 0x79, 0x74, 0xb2,
                0xf1, 0x30, 0x59, 0x61, 0xa9, 0x31, 0x99, 0xa5, 0x28, 0x11, 0x83, 0x81,
                0x7f, 0x45, 0x68, 0x2d, 0x2f, 0xc9, 0x7b, 0x12,
            ]
        );
        assert_eq!(
            required.required_authority_set_digest_sha256(),
            &[
                0xd8, 0xc2, 0xea, 0xe0, 0x16, 0xa5, 0x43, 0xcf, 0xd3, 0xc6, 0x64, 0x53,
                0x4c, 0x9c, 0xec, 0xcb, 0x7d, 0xa5, 0xfb, 0xeb, 0xe0, 0x73, 0xbe, 0x12,
                0xf4, 0x1a, 0x82, 0x4e, 0x28, 0x79, 0x2e, 0x17,
            ]
        );
    }

    #[test]
    fn missing_rotated_generation_fails_closed() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let signing_key_1 = SigningKey::from_bytes(&[0x77; 32]);
        let signing_key_2 = SigningKey::from_bytes(&[0x78; 32]);
        let generation_1 = generation(
            &domain,
            &signing_key_1,
            "identity:policy-authority:bootstrap-v2#ed25519-1",
            1,
        );
        let generation_2 = generation(
            &domain,
            &signing_key_2,
            "identity:policy-authority:bootstrap-v2#ed25519-2",
            2,
        );
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            HistoricalActivationTimePolicyAuthoritySignerV2 {
                policy_authority_id: generation_1.policy_authority_id(),
                policy_authority_key_id: generation_1.policy_authority_key_id(),
                policy_authority_key_generation_sha256: generation_1.generation_digest_sha256(),
                algorithm: generation_1.algorithm(),
            },
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            HistoricalActivationTimePolicyAuthoritySignerV2 {
                policy_authority_id: generation_2.policy_authority_id(),
                policy_authority_key_id: generation_2.policy_authority_key_id(),
                policy_authority_key_generation_sha256: generation_2.generation_digest_sha256(),
                algorithm: generation_2.algorithm(),
            },
        )
        .unwrap();
        let auth_1 = authenticate(&first, &generation_1, &signing_key_1);
        let auth_2 = authenticate(&second, &generation_2, &signing_key_2);
        let observed = qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&first, &second],
        )
        .unwrap();
        let authenticated = qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2(
            &domain,
            &observed,
            &[&first, &second],
            &[&auth_1, &auth_2],
        )
        .unwrap();
        assert_eq!(
            qualify_historical_activation_time_policy_required_signer_authority_set_v2(
                &domain,
                &observed,
                &authenticated,
                &[&first, &second],
                &[&auth_1, &auth_2],
                &[&generation_1],
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyRequiredSignerAuthoritySetErrorV2::MissingSignerGeneration
        );
    }

    #[test]
    fn qualified_set_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationTimePolicyRequiredSignerAuthoritySetV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationTimePolicyRequiredSignerAuthoritySetV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub required_signer_subjects:",
            "pub required_authority_set_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

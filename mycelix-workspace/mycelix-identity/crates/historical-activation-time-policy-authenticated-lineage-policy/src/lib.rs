// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Cryptographically authenticated observed lineage for historical time-policy transitions.
//!
//! This pure composition re-runs #402 topology over the supplied #401 transitions and
//! requires a one-to-one match to opaque #406 cryptographic-authenticity capabilities.
//! Input order is irrelevant. Success means the supplied observed lineage is completely
//! authenticated transition-by-transition, not that its signer generations are authoritative,
//! network history is complete, or its terminal policy is accepted/current.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_policy_lineage_policy::{
    qualify_observed_historical_activation_time_policy_lineage_v2,
    HistoricalActivationTimePolicyLineageErrorV2,
    QualifiedObservedHistoricalActivationTimePolicyLineageV2,
};
use mycelix_historical_activation_time_policy_transition_ed25519_authenticity::CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2;
use mycelix_historical_activation_time_policy_transition_policy::PreparedHistoricalActivationTimePolicyTransitionV2;
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use sha2::{Digest, Sha256};
use std::collections::{HashMap, HashSet};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHENTICATED_LINEAGE_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authenticated-lineage:v2\0";

#[derive(Debug)]
pub struct QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_count: u32,
    terminal_transition_generation: u64,
    terminal_transition_sha256: [u8; SHA256_DIGEST_LEN_V2],
    terminal_policy_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    observed_lineage_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    authenticated_lineage_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn transition_count(&self) -> u32 {
        self.transition_count
    }

    pub fn terminal_transition_generation(&self) -> u64 {
        self.terminal_transition_generation
    }

    pub fn terminal_transition_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.terminal_transition_sha256
    }

    pub fn terminal_policy_sha256(&self) -> Option<&[u8; SHA256_DIGEST_LEN_V2]> {
        self.terminal_policy_sha256.as_ref()
    }

    pub fn observed_lineage_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.observed_lineage_digest_sha256
    }

    pub fn authenticated_lineage_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authenticated_lineage_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyAuthenticatedLineageErrorV2 {
    ObservedLineage(HistoricalActivationTimePolicyLineageErrorV2),
    ObservedLineageMismatch,
    AuthenticationCountMismatch,
    DuplicateAuthenticatedTransition,
    AuthenticatedTransitionDomainMismatch,
    MissingTransitionAuthenticity,
    UnknownTransitionAuthenticity,
    TransitionKeyGenerationMismatch,
    TransitionAlgorithmMismatch,
}

impl From<HistoricalActivationTimePolicyLineageErrorV2>
    for HistoricalActivationTimePolicyAuthenticatedLineageErrorV2
{
    fn from(value: HistoricalActivationTimePolicyLineageErrorV2) -> Self {
        Self::ObservedLineage(value)
    }
}

fn derive_authenticated_lineage_digest_v2(
    authority_domain_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    observed_lineage_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    ordered: &[&PreparedHistoricalActivationTimePolicyTransitionV2],
    auth_by_transition: &HashMap<
        [u8; SHA256_DIGEST_LEN_V2],
        &CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2,
    >,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHENTICATED_LINEAGE_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(authority_domain_sha256);
    hasher.update([0x02]);
    hasher.update(observed_lineage_digest_sha256);
    hasher.update([0x03]);
    hasher.update((ordered.len() as u32).to_be_bytes());

    for transition in ordered {
        let authenticated = auth_by_transition
            .get(transition.transition_signing_digest_sha256())
            .expect("bijection checked before digest derivation");
        hasher.update([0x10]);
        hasher.update(transition.transition_generation().to_be_bytes());
        hasher.update([0x11]);
        hasher.update(transition.transition_signing_digest_sha256());
        hasher.update([0x12]);
        hasher.update(authenticated.authenticity_digest_sha256());
        hasher.update([0x13]);
        hasher.update(authenticated.policy_authority_key_generation_sha256());
    }

    hasher.finalize().into()
}

/// Upgrade one observed #402 lineage to a completely cryptographically authenticated
/// supplied lineage by proving exact transition/authenticity bijection.
pub fn qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    observed_lineage: &QualifiedObservedHistoricalActivationTimePolicyLineageV2,
    transitions: &[&PreparedHistoricalActivationTimePolicyTransitionV2],
    authentications: &[&CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2],
) -> Result<
    QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2,
    HistoricalActivationTimePolicyAuthenticatedLineageErrorV2,
> {
    let rederived =
        qualify_observed_historical_activation_time_policy_lineage_v2(authority_domain, transitions)?;

    if rederived.authority_domain_sha256() != observed_lineage.authority_domain_sha256()
        || rederived.transition_count() != observed_lineage.transition_count()
        || rederived.terminal_transition_generation()
            != observed_lineage.terminal_transition_generation()
        || rederived.terminal_transition_sha256() != observed_lineage.terminal_transition_sha256()
        || rederived.terminal_policy_sha256() != observed_lineage.terminal_policy_sha256()
        || rederived.lineage_digest_sha256() != observed_lineage.lineage_digest_sha256()
    {
        return Err(
            HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::ObservedLineageMismatch,
        );
    }

    if authentications.len() != transitions.len() {
        return Err(
            HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::AuthenticationCountMismatch,
        );
    }

    let transition_digests: HashSet<[u8; SHA256_DIGEST_LEN_V2]> = transitions
        .iter()
        .map(|transition| *transition.transition_signing_digest_sha256())
        .collect();
    let mut auth_by_transition = HashMap::with_capacity(authentications.len());
    for authenticated in authentications {
        if authenticated.authority_domain_sha256() != authority_domain.digest_sha256() {
            return Err(
                HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::AuthenticatedTransitionDomainMismatch,
            );
        }
        let transition_digest = *authenticated.transition_signing_digest_sha256();
        if auth_by_transition
            .insert(transition_digest, *authenticated)
            .is_some()
        {
            return Err(
                HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::DuplicateAuthenticatedTransition,
            );
        }
        if !transition_digests.contains(&transition_digest) {
            return Err(
                HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::UnknownTransitionAuthenticity,
            );
        }
    }

    let mut ordered = transitions.to_vec();
    ordered.sort_by_key(|transition| transition.transition_generation());
    for transition in &ordered {
        let authenticated = auth_by_transition
            .get(transition.transition_signing_digest_sha256())
            .ok_or(
                HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::MissingTransitionAuthenticity,
            )?;
        if authenticated.policy_authority_key_generation_sha256()
            != transition.policy_authority_key_generation_sha256()
        {
            return Err(
                HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::TransitionKeyGenerationMismatch,
            );
        }
        if authenticated.algorithm() != transition.algorithm() {
            return Err(
                HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::TransitionAlgorithmMismatch,
            );
        }
    }

    let authenticated_lineage_digest_sha256 = derive_authenticated_lineage_digest_v2(
        authority_domain.digest_sha256(),
        observed_lineage.lineage_digest_sha256(),
        &ordered,
        &auth_by_transition,
    );

    Ok(
        QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2 {
            authority_domain_sha256: *authority_domain.digest_sha256(),
            transition_count: observed_lineage.transition_count(),
            terminal_transition_generation: observed_lineage.terminal_transition_generation(),
            terminal_transition_sha256: *observed_lineage.terminal_transition_sha256(),
            terminal_policy_sha256: observed_lineage.terminal_policy_sha256().copied(),
            observed_lineage_digest_sha256: *observed_lineage.lineage_digest_sha256(),
            authenticated_lineage_digest_sha256,
        },
    )
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
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
        QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
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

    fn policy(realization: &'static str) -> mycelix_historical_activation_time_authority_policy::QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2 {
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
    fn shuffled_two_transition_authenticated_lineage_has_frozen_digest() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
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
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let signer = HistoricalActivationTimePolicyAuthoritySignerV2 {
            policy_authority_id: generation.policy_authority_id(),
            policy_authority_key_id: generation.policy_authority_key_id(),
            policy_authority_key_generation_sha256: generation.generation_digest_sha256(),
            algorithm: generation.algorithm(),
        };
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer,
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer,
        )
        .unwrap();
        assert_eq!(
            second.transition_signing_digest_sha256(),
            &[
                0xfe, 0xb6, 0xf1, 0x6c, 0x2b, 0x6f, 0x34, 0xe0, 0x9e, 0x81, 0xa1, 0xdd,
                0xb7, 0xbb, 0xad, 0x93, 0x5f, 0x8b, 0xaf, 0x22, 0xab, 0xe9, 0xbd, 0x60,
                0xcb, 0xa3, 0xf2, 0xb7, 0x6c, 0x64, 0xed, 0xf7,
            ]
        );

        let observed = qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&second, &first],
        )
        .unwrap();
        assert_eq!(
            observed.lineage_digest_sha256(),
            &[
                0x88, 0xa1, 0x7f, 0xaf, 0x0d, 0x47, 0xe7, 0x54, 0xfd, 0x77, 0x75, 0x59,
                0xea, 0x72, 0x42, 0xe3, 0x38, 0xc7, 0xed, 0x1b, 0x28, 0xca, 0x46, 0x21,
                0xb2, 0x81, 0x9e, 0xdf, 0x79, 0x14, 0xcc, 0xfc,
            ]
        );

        let first_auth = authenticate(&first, &generation, &signing_key);
        let second_auth = authenticate(&second, &generation, &signing_key);
        assert_eq!(
            second_auth.authenticity_digest_sha256(),
            &[
                0x68, 0xed, 0x0c, 0x45, 0xc7, 0x31, 0x1d, 0x54, 0x98, 0x0c, 0x45, 0x2c,
                0xe0, 0x2d, 0xff, 0xf6, 0x67, 0x6b, 0x9d, 0xb1, 0xdf, 0x2a, 0x31, 0x3d,
                0xfc, 0xee, 0x6d, 0xf1, 0x92, 0xf1, 0x94, 0x84,
            ]
        );

        let authenticated = qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2(
            &domain,
            &observed,
            &[&second, &first],
            &[&second_auth, &first_auth],
        )
        .unwrap();
        assert_eq!(authenticated.transition_count(), 2);
        assert_eq!(
            authenticated.authenticated_lineage_digest_sha256(),
            &[
                0x0b, 0x02, 0x12, 0xbc, 0xfa, 0x1a, 0x50, 0x29, 0x26, 0xe8, 0x5c, 0xe5,
                0x1e, 0x33, 0xdc, 0x08, 0xa2, 0x7c, 0x10, 0x11, 0xd8, 0x36, 0x0f, 0x7c,
                0x2b, 0x08, 0xda, 0x81, 0xfe, 0x2c, 0x7a, 0x6c,
            ]
        );
    }

    #[test]
    fn missing_authentication_fails_closed() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
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
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let signer = HistoricalActivationTimePolicyAuthoritySignerV2 {
            policy_authority_id: generation.policy_authority_id(),
            policy_authority_key_id: generation.policy_authority_key_id(),
            policy_authority_key_generation_sha256: generation.generation_digest_sha256(),
            algorithm: generation.algorithm(),
        };
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer,
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer,
        )
        .unwrap();
        let observed = qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&first, &second],
        )
        .unwrap();
        let first_auth = authenticate(&first, &generation, &signing_key);
        assert_eq!(
            qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2(
                &domain,
                &observed,
                &[&first, &second],
                &[&first_auth],
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::AuthenticationCountMismatch
        );
    }

    #[test]
    fn duplicate_authentication_fails_closed() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
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
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let signer = HistoricalActivationTimePolicyAuthoritySignerV2 {
            policy_authority_id: generation.policy_authority_id(),
            policy_authority_key_id: generation.policy_authority_key_id(),
            policy_authority_key_generation_sha256: generation.generation_digest_sha256(),
            algorithm: generation.algorithm(),
        };
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer,
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer,
        )
        .unwrap();
        let observed = qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&first, &second],
        )
        .unwrap();
        let first_auth = authenticate(&first, &generation, &signing_key);
        assert_eq!(
            qualify_cryptographically_authenticated_historical_activation_time_policy_lineage_v2(
                &domain,
                &observed,
                &[&first, &second],
                &[&first_auth, &first_auth],
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyAuthenticatedLineageErrorV2::DuplicateAuthenticatedTransition
        );
    }

    #[test]
    fn authenticated_lineage_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedCryptographicallyAuthenticatedHistoricalActivationTimePolicyLineageV2")
            .unwrap()
            + start;
        let result = &source[start..end];
        for field in [
            "pub terminal_policy_sha256:",
            "pub observed_lineage_digest_sha256:",
            "pub authenticated_lineage_digest_sha256:",
        ] {
            assert!(!result.contains(field));
        }
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Closed authenticated causal-authority requirements for historical time-policy lineages.
//!
//! This theorem re-runs the exact #402 transition topology, then requires a one-to-one mapping:
//!
//! `each #401 transition <-> exactly one #430 signed causal anchor <-> exactly one #443 authenticity`.
//!
//! The resulting requirements remain Identity-side evidence. They do not prove that the claimed
//! generic authority-state coordinates exist or were Active; #429/#446 own those facts after
//! the Identity and generic authority stacks converge.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_authority_anchored_transition_ed25519_authenticity::CryptographicallyAuthenticatedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2;
use mycelix_historical_activation_time_policy_authority_anchored_transition_policy::PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2;
use mycelix_historical_activation_time_policy_lineage_policy::{
    qualify_observed_historical_activation_time_policy_lineage_v2,
    HistoricalActivationTimePolicyLineageErrorV2,
};
use mycelix_historical_activation_time_policy_transition_policy::PreparedHistoricalActivationTimePolicyTransitionV2;
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use sha2::{Digest, Sha256};
use std::collections::{HashMap, HashSet};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2: usize = 32;
pub const MAX_AUTHORITY_ANCHORED_TIME_POLICY_TRANSITIONS_V2: usize = 4096;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_AUTHENTICATED_LINEAGE_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authority-anchored-authenticated-lineage:v2\0";

#[derive(Debug)]
pub struct HistoricalActivationTimePolicyCausalAuthorityRequirementV2 {
    transition_generation: u64,
    base_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    anchored_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    signer_authority_subject_sha256: [u8; SHA256_DIGEST_LEN_V2],
    authority_state_generation: u64,
    authority_state_transition_digest: [u8; AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    authenticity_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl HistoricalActivationTimePolicyCausalAuthorityRequirementV2 {
    pub fn transition_generation(&self) -> u64 {
        self.transition_generation
    }

    pub fn base_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.base_transition_signing_digest_sha256
    }

    pub fn anchored_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.anchored_transition_signing_digest_sha256
    }

    pub fn signer_authority_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.signer_authority_subject_sha256
    }

    pub fn authority_state_generation(&self) -> u64 {
        self.authority_state_generation
    }

    pub fn authority_state_transition_digest(
        &self,
    ) -> &[u8; AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2] {
        &self.authority_state_transition_digest
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn authenticity_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authenticity_digest_sha256
    }
}

#[derive(Debug)]
pub struct QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_count: u32,
    observed_lineage_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    terminal_transition_generation: u64,
    terminal_transition_sha256: [u8; SHA256_DIGEST_LEN_V2],
    terminal_policy_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    requirements: Vec<HistoricalActivationTimePolicyCausalAuthorityRequirementV2>,
    qualification_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn transition_count(&self) -> u32 {
        self.transition_count
    }

    pub fn observed_lineage_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.observed_lineage_digest_sha256
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

    pub fn requirements(&self) -> &[HistoricalActivationTimePolicyCausalAuthorityRequirementV2] {
        &self.requirements
    }

    pub fn qualification_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.qualification_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2 {
    ObservedLineage(HistoricalActivationTimePolicyLineageErrorV2),
    AnchorCountMismatch,
    AuthenticityCountMismatch,
    AnchorAuthorityDomainMismatch,
    UnknownAnchoredBaseTransition,
    AnchorTransitionGenerationMismatch,
    AnchorTransitionAlgorithmMismatch,
    AnchorKeyGenerationMismatch,
    DuplicateAnchoredBaseTransition,
    DuplicateAnchoredTransitionDigest,
    MissingAnchoredTransition,
    AuthenticityAuthorityDomainMismatch,
    UnknownAuthenticity,
    DuplicateAuthenticity,
    MissingAuthenticity,
    AuthenticityBindingMismatch,
    TransitionCountOverflow,
}

impl From<HistoricalActivationTimePolicyLineageErrorV2>
    for AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2
{
    fn from(value: HistoricalActivationTimePolicyLineageErrorV2) -> Self {
        Self::ObservedLineage(value)
    }
}

pub fn qualify_authority_anchored_authenticated_historical_activation_time_policy_lineage_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    transitions: &[&PreparedHistoricalActivationTimePolicyTransitionV2],
    anchored_transitions: &[&PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2],
    authenticities: &[&CryptographicallyAuthenticatedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2],
) -> Result<
    QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2,
    AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2,
> {
    let observed = qualify_observed_historical_activation_time_policy_lineage_v2(
        authority_domain,
        transitions,
    )?;

    if transitions.len() > MAX_AUTHORITY_ANCHORED_TIME_POLICY_TRANSITIONS_V2 {
        return Err(AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::TransitionCountOverflow);
    }
    if anchored_transitions.len() != transitions.len() {
        return Err(AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AnchorCountMismatch);
    }
    if authenticities.len() != transitions.len() {
        return Err(
            AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AuthenticityCountMismatch,
        );
    }

    let mut base_by_digest = HashMap::with_capacity(transitions.len());
    for transition in transitions {
        base_by_digest.insert(*transition.transition_signing_digest_sha256(), *transition);
    }

    let mut anchor_by_base = HashMap::with_capacity(anchored_transitions.len());
    let mut anchor_digests = HashSet::with_capacity(anchored_transitions.len());
    for anchored in anchored_transitions {
        if anchored.authority_domain_sha256() != authority_domain.digest_sha256() {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AnchorAuthorityDomainMismatch,
            );
        }
        let base_digest = *anchored.base_transition_signing_digest_sha256();
        let Some(base) = base_by_digest.get(&base_digest) else {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::UnknownAnchoredBaseTransition,
            );
        };
        if anchored.transition_generation() != base.transition_generation() {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AnchorTransitionGenerationMismatch,
            );
        }
        if anchored.algorithm() != base.algorithm() {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AnchorTransitionAlgorithmMismatch,
            );
        }
        if anchored.policy_authority_key_generation_sha256()
            != base.policy_authority_key_generation_sha256()
        {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AnchorKeyGenerationMismatch,
            );
        }
        if anchor_by_base.insert(base_digest, *anchored).is_some() {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::DuplicateAnchoredBaseTransition,
            );
        }
        if !anchor_digests.insert(*anchored.anchored_transition_signing_digest_sha256()) {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::DuplicateAnchoredTransitionDigest,
            );
        }
    }

    let mut authenticity_by_anchor = HashMap::with_capacity(authenticities.len());
    for authenticity in authenticities {
        if authenticity.authority_domain_sha256() != authority_domain.digest_sha256() {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AuthenticityAuthorityDomainMismatch,
            );
        }
        let anchored_digest = *authenticity.anchored_transition_signing_digest_sha256();
        if !anchor_digests.contains(&anchored_digest) {
            return Err(AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::UnknownAuthenticity);
        }
        if authenticity_by_anchor
            .insert(anchored_digest, *authenticity)
            .is_some()
        {
            return Err(AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::DuplicateAuthenticity);
        }
    }

    let mut ordered = transitions.to_vec();
    ordered.sort_by_key(|transition| transition.transition_generation());

    let mut requirements = Vec::with_capacity(ordered.len());
    for base in ordered {
        let Some(anchored) = anchor_by_base.get(base.transition_signing_digest_sha256()) else {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::MissingAnchoredTransition,
            );
        };
        let anchored_digest = *anchored.anchored_transition_signing_digest_sha256();
        let Some(authenticity) = authenticity_by_anchor.get(&anchored_digest) else {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::MissingAuthenticity,
            );
        };

        if authenticity.anchored_transition_signing_digest_sha256()
            != anchored.anchored_transition_signing_digest_sha256()
            || authenticity.signer_authority_subject_sha256()
                != anchored.signer_authority_subject_sha256()
            || authenticity.policy_authority_key_generation_sha256()
                != anchored.policy_authority_key_generation_sha256()
            || authenticity.algorithm() != anchored.algorithm()
        {
            return Err(
                AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AuthenticityBindingMismatch,
            );
        }

        requirements.push(HistoricalActivationTimePolicyCausalAuthorityRequirementV2 {
            transition_generation: base.transition_generation(),
            base_transition_signing_digest_sha256: *base.transition_signing_digest_sha256(),
            anchored_transition_signing_digest_sha256: anchored_digest,
            signer_authority_subject_sha256: *anchored.signer_authority_subject_sha256(),
            authority_state_generation: anchored.authority_state_generation(),
            authority_state_transition_digest: *anchored.authority_state_transition_digest(),
            algorithm: anchored.algorithm(),
            authenticity_digest_sha256: *authenticity.authenticity_digest_sha256(),
        });
    }

    let transition_count = u32::try_from(requirements.len()).map_err(|_| {
        AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::TransitionCountOverflow
    })?;
    let qualification_digest_sha256 = derive_qualification_digest_v2(
        authority_domain.digest_sha256(),
        observed.lineage_digest_sha256(),
        &requirements,
    );

    Ok(QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2 {
        authority_domain_sha256: *authority_domain.digest_sha256(),
        transition_count,
        observed_lineage_digest_sha256: *observed.lineage_digest_sha256(),
        terminal_transition_generation: observed.terminal_transition_generation(),
        terminal_transition_sha256: *observed.terminal_transition_sha256(),
        terminal_policy_sha256: observed.terminal_policy_sha256().copied(),
        requirements,
        qualification_digest_sha256,
    })
}

fn derive_qualification_digest_v2(
    authority_domain_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    observed_lineage_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    requirements: &[HistoricalActivationTimePolicyCausalAuthorityRequirementV2],
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(
        HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_AUTHENTICATED_LINEAGE_DOMAIN_V2,
    );
    hasher.update([0x01]);
    hasher.update(authority_domain_sha256);
    hasher.update([0x02]);
    hasher.update(observed_lineage_digest_sha256);
    hasher.update([0x03]);
    hasher.update((requirements.len() as u32).to_be_bytes());
    for requirement in requirements {
        hasher.update([0x10]);
        hasher.update(requirement.transition_generation.to_be_bytes());
        hasher.update([0x11]);
        hasher.update(requirement.base_transition_signing_digest_sha256);
        hasher.update([0x12]);
        hasher.update(requirement.anchored_transition_signing_digest_sha256);
        hasher.update([0x13]);
        hasher.update(requirement.signer_authority_subject_sha256);
        hasher.update([0x14]);
        hasher.update(requirement.authority_state_generation.to_be_bytes());
        hasher.update([0x15]);
        hasher.update(requirement.authority_state_transition_digest);
        hasher.update([0x16]);
        hasher.update(requirement.algorithm.as_u16().to_be_bytes());
        hasher.update([0x17]);
        hasher.update(requirement.authenticity_digest_sha256);
    }
    hasher.finalize().into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use mycelix_crypto::TaggedSignature;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
        QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    };
    use mycelix_historical_activation_time_policy_authority_anchored_transition_crypto_request_policy::prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2;
    use mycelix_historical_activation_time_policy_authority_anchored_transition_ed25519_authenticity::authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2;
    use mycelix_historical_activation_time_policy_authority_anchored_transition_policy::{
        prepare_authority_anchored_historical_activation_time_policy_transition_v2,
        HistoricalActivationTimePolicyAuthorityStateAnchorV2,
    };
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
        QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_policy_transition_policy::{
        prepare_time_policy_adoption_transition_v2,
        prepare_time_policy_supersession_transition_v2,
        HistoricalActivationTimePolicyAuthoritySignerV2,
    };
    use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::qualify_historical_activation_time_policy_transition_signer_authority_subject_v2;
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
    static AUTHORITY_STATE_1: [u8; 32] = [0x55; 32];
    static AUTHORITY_STATE_2: [u8; 32] = [0x66; 32];

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

    fn key_generation(
        domain: &QualifiedIdentityAuthorityDomainV2,
        signing_key: &SigningKey,
    ) -> QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
        let public_key = signing_key.verifying_key().to_bytes();
        qualify_historical_activation_time_policy_authority_key_generation_v2(
            domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &public_key,
                key_generation: 1,
            },
        )
        .unwrap()
    }

    fn signer<'a>(
        generation: &'a QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    ) -> HistoricalActivationTimePolicyAuthoritySignerV2<'a> {
        HistoricalActivationTimePolicyAuthoritySignerV2 {
            policy_authority_id: generation.policy_authority_id(),
            policy_authority_key_id: generation.policy_authority_key_id(),
            policy_authority_key_generation_sha256: generation.generation_digest_sha256(),
            algorithm: generation.algorithm(),
        }
    }

    #[test]
    fn every_transition_requires_one_authenticated_causal_anchor() {
        let domain = domain();
        let signing_key = SigningKey::from_bytes(&[0x77; 32]);
        let generation = key_generation(&domain, &signing_key);
        let subject = qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
            &generation,
        );
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-step-v2");

        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer(&generation),
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer(&generation),
        )
        .unwrap();

        let first_anchored = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &first,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 7,
                authority_state_transition_digest: &AUTHORITY_STATE_1,
            },
        )
        .unwrap();
        let second_anchored = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &second,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 9,
                authority_state_transition_digest: &AUTHORITY_STATE_2,
            },
        )
        .unwrap();

        let first_signature = TaggedSignature::new(
            AlgorithmId::Ed25519,
            signing_key
                .sign(first_anchored.anchored_transition_signing_digest_sha256())
                .to_bytes()
                .to_vec(),
        )
        .unwrap();
        let second_signature = TaggedSignature::new(
            AlgorithmId::Ed25519,
            signing_key
                .sign(second_anchored.anchored_transition_signing_digest_sha256())
                .to_bytes()
                .to_vec(),
        )
        .unwrap();
        let first_request = prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(
            &first_anchored,
            &generation,
            &first_signature,
        )
        .unwrap();
        let second_request = prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(
            &second_anchored,
            &generation,
            &second_signature,
        )
        .unwrap();
        let first_auth = authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(&first_request).unwrap();
        let second_auth = authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(&second_request).unwrap();

        let qualified = qualify_authority_anchored_authenticated_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&first, &second],
            &[&second_anchored, &first_anchored],
            &[&second_auth, &first_auth],
        )
        .unwrap();

        assert_eq!(qualified.transition_count(), 2);
        assert_eq!(qualified.requirements().len(), 2);
        assert_eq!(qualified.requirements()[0].transition_generation(), 1);
        assert_eq!(qualified.requirements()[0].authority_state_generation(), 7);
        assert_eq!(qualified.requirements()[1].transition_generation(), 2);
        assert_eq!(qualified.requirements()[1].authority_state_generation(), 9);
        assert_eq!(
            qualified.requirements()[0].authority_state_transition_digest(),
            &AUTHORITY_STATE_1
        );
        assert_eq!(
            qualified.requirements()[1].authority_state_transition_digest(),
            &AUTHORITY_STATE_2
        );
    }

    #[test]
    fn missing_anchor_fails_closed_before_authority_bridge() {
        let domain = domain();
        let signing_key = SigningKey::from_bytes(&[0x77; 32]);
        let generation = key_generation(&domain, &signing_key);
        let subject = qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
            &generation,
        );
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-step-v2");
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer(&generation),
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer(&generation),
        )
        .unwrap();
        let first_anchored = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &first,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 7,
                authority_state_transition_digest: &AUTHORITY_STATE_1,
            },
        )
        .unwrap();
        let first_signature = TaggedSignature::new(
            AlgorithmId::Ed25519,
            signing_key
                .sign(first_anchored.anchored_transition_signing_digest_sha256())
                .to_bytes()
                .to_vec(),
        )
        .unwrap();
        let first_request = prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(
            &first_anchored,
            &generation,
            &first_signature,
        )
        .unwrap();
        let first_auth = authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(&first_request).unwrap();

        assert_eq!(
            qualify_authority_anchored_authenticated_historical_activation_time_policy_lineage_v2(
                &domain,
                &[&first, &second],
                &[&first_anchored],
                &[&first_auth],
            )
            .unwrap_err(),
            AuthorityAnchoredAuthenticatedTimePolicyLineageErrorV2::AnchorCountMismatch
        );
    }
}

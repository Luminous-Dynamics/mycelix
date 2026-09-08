// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Complete topology/state-machine theorem for prepared historical time-policy transitions.
//!
//! This pure layer proves that one bounded observed transition set forms exactly one
//! contiguous, non-branching policy-state lineage inside one #369 authority domain.
//! Input order is irrelevant; topology and exact predecessor digests determine order.
//!
//! Success identifies only the unique observed terminal policy candidate (or terminal
//! revocation). It does not authenticate transition signatures, prove network/DHT
//! completeness, accept/current the terminal policy, trust a clock, or grant activation.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_policy_transition_policy::{
    HistoricalActivationTimePolicyTransitionKindV2,
    PreparedHistoricalActivationTimePolicyTransitionV2,
};
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use sha2::{Digest, Sha256};
use std::collections::HashSet;

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const MAX_TIME_POLICY_TRANSITIONS_V2: usize = 4096;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_LINEAGE_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-lineage:v2\0";

#[derive(Debug)]
pub struct QualifiedObservedHistoricalActivationTimePolicyLineageV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_count: u32,
    terminal_transition_generation: u64,
    terminal_transition_sha256: [u8; SHA256_DIGEST_LEN_V2],
    terminal_policy_sha256: Option<[u8; SHA256_DIGEST_LEN_V2]>,
    lineage_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedObservedHistoricalActivationTimePolicyLineageV2 {
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

    pub fn lineage_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.lineage_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyLineageErrorV2 {
    EmptyTransitionSet,
    TooManyTransitions,
    AuthorityDomainMismatch,
    DuplicateTransitionGeneration,
    DuplicateTransitionDigest,
    RootGenerationMissing,
    GenerationGap,
    PredecessorMismatch,
    InvalidRootTransition,
    TransitionShapeInvalid,
    AdoptWhilePolicyActive,
    SupersedeWithoutActivePolicy,
    SupersedePriorPolicyMismatch,
    RevokeWithoutActivePolicy,
    RevokePriorPolicyMismatch,
}

fn derive_lineage_digest_v2(
    authority_domain_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    transition_count: u32,
    terminal_transition_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    terminal_transition_generation: u64,
    terminal_policy_sha256: Option<&[u8; SHA256_DIGEST_LEN_V2]>,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_LINEAGE_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(authority_domain_sha256);
    hasher.update([0x02]);
    hasher.update(transition_count.to_be_bytes());
    hasher.update([0x03]);
    hasher.update(terminal_transition_sha256);
    hasher.update([0x04]);
    hasher.update(terminal_transition_generation.to_be_bytes());
    hasher.update([0x05]);
    match terminal_policy_sha256 {
        Some(policy) => {
            hasher.update([1]);
            hasher.update(policy);
        }
        None => hasher.update([0]),
    }
    hasher.finalize().into()
}

/// Qualify one bounded observed prepared-transition set as a single topology/state lineage.
///
/// The result remains observation-scoped because this function does not authenticate
/// transitions or prove that the caller supplied a complete network history.
pub fn qualify_observed_historical_activation_time_policy_lineage_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    transitions: &[&PreparedHistoricalActivationTimePolicyTransitionV2],
) -> Result<
    QualifiedObservedHistoricalActivationTimePolicyLineageV2,
    HistoricalActivationTimePolicyLineageErrorV2,
> {
    if transitions.is_empty() {
        return Err(HistoricalActivationTimePolicyLineageErrorV2::EmptyTransitionSet);
    }
    if transitions.len() > MAX_TIME_POLICY_TRANSITIONS_V2 {
        return Err(HistoricalActivationTimePolicyLineageErrorV2::TooManyTransitions);
    }

    let mut generations = HashSet::with_capacity(transitions.len());
    let mut digests = HashSet::with_capacity(transitions.len());
    for transition in transitions {
        if transition.authority_domain_sha256() != authority_domain.digest_sha256() {
            return Err(HistoricalActivationTimePolicyLineageErrorV2::AuthorityDomainMismatch);
        }
        if !generations.insert(transition.transition_generation()) {
            return Err(HistoricalActivationTimePolicyLineageErrorV2::DuplicateTransitionGeneration);
        }
        if !digests.insert(*transition.transition_signing_digest_sha256()) {
            return Err(HistoricalActivationTimePolicyLineageErrorV2::DuplicateTransitionDigest);
        }
    }

    let mut ordered = transitions.to_vec();
    ordered.sort_by_key(|transition| transition.transition_generation());

    let Some(root) = ordered.first().copied() else {
        return Err(HistoricalActivationTimePolicyLineageErrorV2::EmptyTransitionSet);
    };
    if root.transition_generation() != 1 {
        return Err(HistoricalActivationTimePolicyLineageErrorV2::RootGenerationMissing);
    }

    if root.predecessor_transition_sha256().is_some()
        || root.transition_kind() != HistoricalActivationTimePolicyTransitionKindV2::Adopt
        || root.prior_policy_sha256().is_some()
        || root.resulting_policy_sha256().is_none()
    {
        return Err(HistoricalActivationTimePolicyLineageErrorV2::InvalidRootTransition);
    }

    let mut active_policy = root.resulting_policy_sha256().copied();
    let mut previous_digest = *root.transition_signing_digest_sha256();

    for (index, transition) in ordered.iter().enumerate().skip(1) {
        let expected_generation = (index as u64) + 1;
        if transition.transition_generation() != expected_generation {
            return Err(HistoricalActivationTimePolicyLineageErrorV2::GenerationGap);
        }
        if transition.predecessor_transition_sha256() != Some(&previous_digest) {
            return Err(HistoricalActivationTimePolicyLineageErrorV2::PredecessorMismatch);
        }

        match transition.transition_kind() {
            HistoricalActivationTimePolicyTransitionKindV2::Adopt => {
                if transition.prior_policy_sha256().is_some()
                    || transition.resulting_policy_sha256().is_none()
                {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::TransitionShapeInvalid);
                }
                if active_policy.is_some() {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::AdoptWhilePolicyActive);
                }
                active_policy = transition.resulting_policy_sha256().copied();
            }
            HistoricalActivationTimePolicyTransitionKindV2::Supersede => {
                let Some(current) = active_policy else {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::SupersedeWithoutActivePolicy);
                };
                let (Some(prior), Some(resulting)) = (
                    transition.prior_policy_sha256(),
                    transition.resulting_policy_sha256(),
                ) else {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::TransitionShapeInvalid);
                };
                if prior != &current {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::SupersedePriorPolicyMismatch);
                }
                if resulting == &current {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::TransitionShapeInvalid);
                }
                active_policy = Some(*resulting);
            }
            HistoricalActivationTimePolicyTransitionKindV2::Revoke => {
                let Some(current) = active_policy else {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::RevokeWithoutActivePolicy);
                };
                if transition.resulting_policy_sha256().is_some() {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::TransitionShapeInvalid);
                }
                let Some(prior) = transition.prior_policy_sha256() else {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::TransitionShapeInvalid);
                };
                if prior != &current {
                    return Err(HistoricalActivationTimePolicyLineageErrorV2::RevokePriorPolicyMismatch);
                }
                active_policy = None;
            }
        }

        previous_digest = *transition.transition_signing_digest_sha256();
    }

    let Some(terminal) = ordered.last().copied() else {
        return Err(HistoricalActivationTimePolicyLineageErrorV2::EmptyTransitionSet);
    };
    let transition_count = u32::try_from(ordered.len())
        .map_err(|_| HistoricalActivationTimePolicyLineageErrorV2::TooManyTransitions)?;
    let lineage_digest_sha256 = derive_lineage_digest_v2(
        authority_domain.digest_sha256(),
        transition_count,
        terminal.transition_signing_digest_sha256(),
        terminal.transition_generation(),
        active_policy.as_ref(),
    );

    Ok(QualifiedObservedHistoricalActivationTimePolicyLineageV2 {
        authority_domain_sha256: *authority_domain.digest_sha256(),
        transition_count,
        terminal_transition_generation: terminal.transition_generation(),
        terminal_transition_sha256: *terminal.transition_signing_digest_sha256(),
        terminal_policy_sha256: active_policy,
        lineage_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::AlgorithmId;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
        QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    };
    use mycelix_historical_activation_time_policy_transition_policy::{
        prepare_time_policy_adoption_transition_v2,
        prepare_time_policy_revocation_transition_v2,
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
    fn unordered_adopt_supersede_chain_qualifies_with_frozen_terminal_digest() {
        let domain = domain();
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer(),
        )
        .unwrap();
        let second = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer(),
        )
        .unwrap();
        assert_eq!(
            second.transition_signing_digest_sha256(),
            &[
                0x53, 0x45, 0xea, 0xdc, 0x71, 0x0b, 0x14, 0x69, 0x07, 0x8e, 0xae, 0xc5,
                0xa0, 0x26, 0x8b, 0x43, 0x81, 0x89, 0x23, 0x58, 0xd3, 0x53, 0x4e, 0xa1,
                0xaf, 0xd4, 0x11, 0xc5, 0xfc, 0x52, 0x0c, 0x6d,
            ]
        );
        let lineage = qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&second, &first],
        )
        .unwrap();
        assert_eq!(lineage.transition_count(), 2);
        assert_eq!(lineage.terminal_transition_generation(), 2);
        assert_eq!(lineage.terminal_policy_sha256(), Some(second_policy.policy_digest_sha256()));
        assert_eq!(
            lineage.lineage_digest_sha256(),
            &[
                0xba, 0x98, 0xf1, 0x02, 0x8f, 0x68, 0x56, 0xed, 0xde, 0x5b, 0x98, 0x68,
                0xb1, 0xaa, 0x54, 0x35, 0x11, 0xc4, 0x9e, 0x41, 0x3b, 0x0d, 0x0e, 0xeb,
                0xd2, 0x0a, 0xdd, 0xc1, 0x46, 0xc6, 0x4b, 0x1c,
            ]
        );
    }

    #[test]
    fn adopt_while_policy_active_fails_closed() {
        let domain = domain();
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer(),
        )
        .unwrap();
        let second = prepare_time_policy_adoption_transition_v2(
            &domain,
            &second_policy,
            2,
            Some(first.transition_signing_digest_sha256()),
            signer(),
        )
        .unwrap();
        assert_eq!(
            qualify_observed_historical_activation_time_policy_lineage_v2(
                &domain,
                &[&first, &second],
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyLineageErrorV2::AdoptWhilePolicyActive
        );
    }

    #[test]
    fn revoke_then_adopt_is_explicit_and_valid() {
        let domain = domain();
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer(),
        )
        .unwrap();
        let revoke = prepare_time_policy_revocation_transition_v2(
            &domain,
            &first_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer(),
        )
        .unwrap();
        let adopt = prepare_time_policy_adoption_transition_v2(
            &domain,
            &second_policy,
            3,
            Some(revoke.transition_signing_digest_sha256()),
            signer(),
        )
        .unwrap();
        let lineage = qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&adopt, &first, &revoke],
        )
        .unwrap();
        assert_eq!(lineage.terminal_policy_sha256(), Some(second_policy.policy_digest_sha256()));
    }

    #[test]
    fn duplicate_generation_is_a_conflict_not_a_winner_selection() {
        let domain = domain();
        let first_policy = policy("unix-utc-normalized-v1");
        let second_policy = policy("unix-utc-smear-normalized-v1");
        let first = prepare_time_policy_adoption_transition_v2(
            &domain,
            &first_policy,
            1,
            None,
            signer(),
        )
        .unwrap();
        let supersede = prepare_time_policy_supersession_transition_v2(
            &domain,
            &first_policy,
            &second_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer(),
        )
        .unwrap();
        let revoke = prepare_time_policy_revocation_transition_v2(
            &domain,
            &first_policy,
            2,
            first.transition_signing_digest_sha256(),
            signer(),
        )
        .unwrap();
        assert_eq!(
            qualify_observed_historical_activation_time_policy_lineage_v2(
                &domain,
                &[&first, &supersede, &revoke],
            )
            .unwrap_err(),
            HistoricalActivationTimePolicyLineageErrorV2::DuplicateTransitionGeneration
        );
    }

    #[test]
    fn qualified_lineage_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedObservedHistoricalActivationTimePolicyLineageV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedObservedHistoricalActivationTimePolicyLineageV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub terminal_transition_sha256:",
            "pub terminal_policy_sha256:",
            "pub lineage_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

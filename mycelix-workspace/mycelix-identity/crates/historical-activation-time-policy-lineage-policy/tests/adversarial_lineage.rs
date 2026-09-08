// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Adversarial topology vectors for the observed historical time-policy lineage theorem.
//!
//! These tests deliberately exercise failure modes that are security-relevant at the
//! composition boundary: missing generations, predecessor substitution, and explicit
//! terminal revocation. They do not add signature authenticity, authority continuity,
//! network completeness, or accepted/current policy semantics.

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_authority_policy::{
    qualify_static_historical_activation_time_authority_policy_v2,
    HistoricalActivationTimeAuthorityPolicyBodyV2,
    QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
};
use mycelix_historical_activation_time_policy_lineage_policy::{
    qualify_observed_historical_activation_time_policy_lineage_v2,
    HistoricalActivationTimePolicyLineageErrorV2,
};
use mycelix_historical_activation_time_policy_transition_policy::{
    prepare_time_policy_adoption_transition_v2, prepare_time_policy_revocation_transition_v2,
    prepare_time_policy_supersession_transition_v2,
    HistoricalActivationTimePolicyAuthoritySignerV2,
};
use mycelix_historical_activation_time_receipt_policy::TimeBasisV2;
use mycelix_identity_authority_domain_policy::{
    qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    QualifiedIdentityAuthorityDomainV2,
};

static DNA: [u8; 39] = [
    0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
    0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
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
fn generation_gap_fails_closed_even_when_predecessor_digest_points_to_root() {
    let domain = domain();
    let first_policy = policy("unix-utc-normalized-v1");
    let second_policy = policy("unix-utc-smear-normalized-v1");

    let root =
        prepare_time_policy_adoption_transition_v2(&domain, &first_policy, 1, None, signer())
            .unwrap();
    let generation_three = prepare_time_policy_supersession_transition_v2(
        &domain,
        &first_policy,
        &second_policy,
        3,
        root.transition_signing_digest_sha256(),
        signer(),
    )
    .unwrap();

    assert_eq!(
        qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&generation_three, &root],
        )
        .unwrap_err(),
        HistoricalActivationTimePolicyLineageErrorV2::GenerationGap
    );
}

#[test]
fn predecessor_substitution_fails_closed() {
    let domain = domain();
    let first_policy = policy("unix-utc-normalized-v1");
    let second_policy = policy("unix-utc-smear-normalized-v1");

    let root =
        prepare_time_policy_adoption_transition_v2(&domain, &first_policy, 1, None, signer())
            .unwrap();
    let wrong_predecessor = [0x55; 32];
    let supersede = prepare_time_policy_supersession_transition_v2(
        &domain,
        &first_policy,
        &second_policy,
        2,
        &wrong_predecessor,
        signer(),
    )
    .unwrap();

    assert_eq!(
        qualify_observed_historical_activation_time_policy_lineage_v2(
            &domain,
            &[&root, &supersede],
        )
        .unwrap_err(),
        HistoricalActivationTimePolicyLineageErrorV2::PredecessorMismatch
    );
}

#[test]
fn terminal_revocation_is_explicit_none_not_positive_policy_authority() {
    let domain = domain();
    let active_policy = policy("unix-utc-normalized-v1");

    let root =
        prepare_time_policy_adoption_transition_v2(&domain, &active_policy, 1, None, signer())
            .unwrap();
    let revoke = prepare_time_policy_revocation_transition_v2(
        &domain,
        &active_policy,
        2,
        root.transition_signing_digest_sha256(),
        signer(),
    )
    .unwrap();

    let lineage = qualify_observed_historical_activation_time_policy_lineage_v2(
        &domain,
        &[&revoke, &root],
    )
    .unwrap();

    assert_eq!(lineage.transition_count(), 2);
    assert_eq!(lineage.terminal_transition_generation(), 2);
    assert_eq!(
        lineage.terminal_transition_sha256(),
        revoke.transition_signing_digest_sha256()
    );
    assert_eq!(lineage.terminal_policy_sha256(), None);
}

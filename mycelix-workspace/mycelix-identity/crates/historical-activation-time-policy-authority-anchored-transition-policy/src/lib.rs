// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Signed causal authority-state anchor envelope for one historical time-policy transition.
//!
//! PR #401 freezes the candidate time-policy transition and expected signing key generation.
//! PR #424 freezes the exact semantic authority subject represented by that signer generation.
//! This theorem binds those facts to one exact generic authority-state causal coordinate so
//! the signer cannot later be rebound to whichever historical authority state is convenient.
//!
//! Success is still only a prepared signing statement. A later crypto adapter must authenticate
//! this anchored digest, and the generic authority stack must independently prove that the exact
//! claimed `(generation, transition digest)` belongs to the fully covered #91 lineage and was Active.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_transition_policy::PreparedHistoricalActivationTimePolicyTransitionV2;
use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2: usize = 32;
pub const AUTHORITY_STATE_MAX_GENERATION_V1: u64 = 256;
pub const GENERIC_AUTHORITY_STATE_TRANSITION_IDENTITY_PROFILE_V1: &str =
    "mycelix-authority-state-transition-v1-blake3-framed";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_TRANSITION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-transition:authority-anchored:v2\0";

#[derive(Debug, Clone, Copy)]
pub struct HistoricalActivationTimePolicyAuthorityStateAnchorV2<'a> {
    /// Exact generation in the generic #91 authority-state lineage.
    pub authority_state_generation: u64,
    /// Exact BLAKE3 transition identity under `GENERIC_AUTHORITY_STATE_TRANSITION_IDENTITY_PROFILE_V1`.
    pub authority_state_transition_digest: &'a [u8],
}

#[derive(Debug)]
pub struct PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_generation: u64,
    base_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    signer_authority_subject_sha256: [u8; SHA256_DIGEST_LEN_V2],
    authority_state_generation: u64,
    authority_state_transition_digest: [u8; AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2],
    anchored_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn transition_generation(&self) -> u64 {
        self.transition_generation
    }

    pub fn base_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.base_transition_signing_digest_sha256
    }

    pub fn policy_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.policy_authority_key_generation_sha256
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn signer_authority_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.signer_authority_subject_sha256
    }

    pub fn authority_state_transition_identity_profile(&self) -> &'static str {
        GENERIC_AUTHORITY_STATE_TRANSITION_IDENTITY_PROFILE_V1
    }

    pub fn authority_state_generation(&self) -> u64 {
        self.authority_state_generation
    }

    pub fn authority_state_transition_digest(&self) -> &[u8; AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2] {
        &self.authority_state_transition_digest
    }

    pub fn anchored_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.anchored_transition_signing_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityAnchoredTimePolicyTransitionErrorV2 {
    AuthorityStateGenerationInvalid,
    AuthorityStateTransitionDigestLengthInvalid,
    AuthorityStateTransitionDigestAllZero,
    SignerAuthorityDomainMismatch,
    SignerKeyGenerationMismatch,
    SignerAuthorityIdMismatch,
    SignerKeyIdMismatch,
    SignerAlgorithmMismatch,
}

fn require_anchor_digest_v2(
    value: &[u8],
) -> Result<[u8; AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2], AuthorityAnchoredTimePolicyTransitionErrorV2>
{
    let digest: [u8; AUTHORITY_STATE_TRANSITION_DIGEST_LEN_V2] = value.try_into().map_err(|_| {
        AuthorityAnchoredTimePolicyTransitionErrorV2::AuthorityStateTransitionDigestLengthInvalid
    })?;
    if digest.iter().all(|byte| *byte == 0) {
        return Err(
            AuthorityAnchoredTimePolicyTransitionErrorV2::AuthorityStateTransitionDigestAllZero,
        );
    }
    Ok(digest)
}

fn validate_signer_subject_binding_v2(
    transition: &PreparedHistoricalActivationTimePolicyTransitionV2,
    subject: &QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2,
) -> Result<(), AuthorityAnchoredTimePolicyTransitionErrorV2> {
    if subject.authority_domain_sha256() != transition.authority_domain_sha256() {
        return Err(AuthorityAnchoredTimePolicyTransitionErrorV2::SignerAuthorityDomainMismatch);
    }
    if subject.policy_authority_key_generation_sha256()
        != transition.policy_authority_key_generation_sha256()
    {
        return Err(AuthorityAnchoredTimePolicyTransitionErrorV2::SignerKeyGenerationMismatch);
    }
    if subject.policy_authority_id() != transition.policy_authority_id() {
        return Err(AuthorityAnchoredTimePolicyTransitionErrorV2::SignerAuthorityIdMismatch);
    }
    if subject.policy_authority_key_id() != transition.policy_authority_key_id() {
        return Err(AuthorityAnchoredTimePolicyTransitionErrorV2::SignerKeyIdMismatch);
    }
    if subject.algorithm() != transition.algorithm() {
        return Err(AuthorityAnchoredTimePolicyTransitionErrorV2::SignerAlgorithmMismatch);
    }
    Ok(())
}

fn update_len_prefixed_u16_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u16).to_be_bytes());
    hasher.update(value);
}

pub fn derive_authority_anchored_historical_activation_time_policy_transition_digest_v2(
    transition: &PreparedHistoricalActivationTimePolicyTransitionV2,
    signer_subject: &QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2,
    anchor: HistoricalActivationTimePolicyAuthorityStateAnchorV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], AuthorityAnchoredTimePolicyTransitionErrorV2> {
    validate_signer_subject_binding_v2(transition, signer_subject)?;
    if anchor.authority_state_generation == 0
        || anchor.authority_state_generation > AUTHORITY_STATE_MAX_GENERATION_V1
    {
        return Err(AuthorityAnchoredTimePolicyTransitionErrorV2::AuthorityStateGenerationInvalid);
    }
    let authority_state_transition_digest =
        require_anchor_digest_v2(anchor.authority_state_transition_digest)?;

    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_TRANSITION_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(transition.authority_domain_sha256());
    hasher.update([0x02]);
    hasher.update(transition.transition_signing_digest_sha256());
    hasher.update([0x03]);
    hasher.update(signer_subject.subject_digest_sha256());
    update_len_prefixed_u16_v2(
        &mut hasher,
        0x04,
        GENERIC_AUTHORITY_STATE_TRANSITION_IDENTITY_PROFILE_V1.as_bytes(),
    );
    hasher.update([0x05]);
    hasher.update(anchor.authority_state_generation.to_be_bytes());
    hasher.update([0x06]);
    hasher.update(authority_state_transition_digest);
    Ok(hasher.finalize().into())
}

/// Freeze the exact causal authority-state anchor into the administrative signing transcript.
///
/// The generic #91/#429 authority state is not imported here because the Identity and generic
/// authority stacks do not yet share a reviewable ancestry. This theorem freezes only the signed
/// coordinate. A later convergence adapter must prove the coordinate against #429 and require the
/// resulting historical state to be Active for this exact #424 subject.
pub fn prepare_authority_anchored_historical_activation_time_policy_transition_v2(
    transition: &PreparedHistoricalActivationTimePolicyTransitionV2,
    signer_subject: &QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2,
    anchor: HistoricalActivationTimePolicyAuthorityStateAnchorV2<'_>,
) -> Result<PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2, AuthorityAnchoredTimePolicyTransitionErrorV2>
{
    validate_signer_subject_binding_v2(transition, signer_subject)?;
    if anchor.authority_state_generation == 0
        || anchor.authority_state_generation > AUTHORITY_STATE_MAX_GENERATION_V1
    {
        return Err(AuthorityAnchoredTimePolicyTransitionErrorV2::AuthorityStateGenerationInvalid);
    }
    let authority_state_transition_digest =
        require_anchor_digest_v2(anchor.authority_state_transition_digest)?;
    let anchored_transition_signing_digest_sha256 =
        derive_authority_anchored_historical_activation_time_policy_transition_digest_v2(
            transition,
            signer_subject,
            anchor,
        )?;

    Ok(PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2 {
        authority_domain_sha256: *transition.authority_domain_sha256(),
        transition_generation: transition.transition_generation(),
        base_transition_signing_digest_sha256: *transition.transition_signing_digest_sha256(),
        policy_authority_key_generation_sha256: *transition
            .policy_authority_key_generation_sha256(),
        algorithm: transition.algorithm(),
        signer_authority_subject_sha256: *signer_subject.subject_digest_sha256(),
        authority_state_generation: anchor.authority_state_generation,
        authority_state_transition_digest,
        anchored_transition_signing_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
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
    };

    static DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];
    static TIME_KEY_GENERATION: [u8; 32] = [0x33; 32];
    static AUTHORITY_STATE_TRANSITION: [u8; 32] = [0x55; 32];
    static PUBLIC_KEY: [u8; 32] = [
        0xc8, 0x53, 0xad, 0x0f, 0x0c, 0xd2, 0xb6, 0x19, 0xae, 0xa9, 0x2c, 0xee, 0xc4, 0xfd,
        0x56, 0xa2, 0x4d, 0x64, 0x99, 0xd5, 0x84, 0xce, 0x79, 0x25, 0x7e, 0x45, 0xcf, 0xd8,
        0x13, 0x9b, 0x60, 0xa7,
    ];

    fn fixtures(
        dna: &[u8],
    ) -> (
        PreparedHistoricalActivationTimePolicyTransitionV2,
        QualifiedHistoricalActivationTimePolicyTransitionSignerAuthoritySubjectV2,
    ) {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: dna,
        })
        .unwrap();
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &PUBLIC_KEY,
                key_generation: 1,
            },
        )
        .unwrap();
        let subject =
            qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(
                &generation,
            );
        let policy = qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:primary-v2",
                policy_version: 1,
                time_authority_id: "time:authority:primary-v2",
                time_authority_key_id: "time:authority:primary-v2#hybrid-1",
                time_authority_key_generation_sha256: &TIME_KEY_GENERATION,
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
        (transition, subject)
    }

    #[test]
    fn frozen_real_signer_authority_anchor_digest_is_stable() {
        let (transition, subject) = fixtures(&DNA);
        assert_eq!(
            transition.transition_signing_digest_sha256(),
            &[
                0x51, 0x40, 0x02, 0x25, 0x80, 0x9b, 0x98, 0xc2, 0x65, 0x16, 0x2e, 0x46,
                0x92, 0x81, 0x79, 0xf3, 0x8e, 0x21, 0x6a, 0xcd, 0x70, 0x74, 0x5c, 0xd9,
                0x80, 0x46, 0xc8, 0x54, 0xf7, 0x05, 0x0d, 0x7e,
            ]
        );
        assert_eq!(
            subject.subject_digest_sha256(),
            &[
                0x69, 0xa8, 0x1f, 0xf8, 0x06, 0xe7, 0xfc, 0x9f, 0xe0, 0x9e, 0xa7, 0x4b,
                0xfe, 0xa2, 0x5c, 0x6f, 0xb8, 0x91, 0x69, 0x22, 0x32, 0x30, 0x73, 0xe1,
                0xa4, 0xe3, 0xbd, 0x0a, 0x2b, 0x72, 0xc2, 0x7a,
            ]
        );
        let anchored = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &transition,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 1,
                authority_state_transition_digest: &AUTHORITY_STATE_TRANSITION,
            },
        )
        .unwrap();
        assert_eq!(
            anchored.anchored_transition_signing_digest_sha256(),
            &[
                0xa7, 0x5e, 0x9b, 0xae, 0x12, 0x35, 0x49, 0x6b, 0xfd, 0xde, 0x1a, 0x3a,
                0xc5, 0x2e, 0x9d, 0x03, 0x25, 0x8f, 0x55, 0x2f, 0xfa, 0xc2, 0x47, 0x53,
                0x98, 0x1f, 0x7a, 0x8c, 0xbb, 0x0c, 0x76, 0xcd,
            ]
        );
    }

    #[test]
    fn changing_causal_authority_anchor_changes_signed_subject() {
        let (transition, subject) = fixtures(&DNA);
        let first = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &transition,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 1,
                authority_state_transition_digest: &AUTHORITY_STATE_TRANSITION,
            },
        )
        .unwrap();
        let mut changed_digest = AUTHORITY_STATE_TRANSITION;
        changed_digest[0] ^= 0x01;
        let second = prepare_authority_anchored_historical_activation_time_policy_transition_v2(
            &transition,
            &subject,
            HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                authority_state_generation: 2,
                authority_state_transition_digest: &changed_digest,
            },
        )
        .unwrap();
        assert_ne!(
            first.anchored_transition_signing_digest_sha256(),
            second.anchored_transition_signing_digest_sha256()
        );
    }

    #[test]
    fn signer_subject_from_another_authority_domain_fails() {
        let (transition, _) = fixtures(&DNA);
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let (_, other_subject) = fixtures(&other_dna);
        assert_eq!(
            prepare_authority_anchored_historical_activation_time_policy_transition_v2(
                &transition,
                &other_subject,
                HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                    authority_state_generation: 1,
                    authority_state_transition_digest: &AUTHORITY_STATE_TRANSITION,
                },
            )
            .unwrap_err(),
            AuthorityAnchoredTimePolicyTransitionErrorV2::SignerAuthorityDomainMismatch
        );
    }

    #[test]
    fn invalid_causal_anchor_fails_closed() {
        let (transition, subject) = fixtures(&DNA);
        assert_eq!(
            prepare_authority_anchored_historical_activation_time_policy_transition_v2(
                &transition,
                &subject,
                HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                    authority_state_generation: 0,
                    authority_state_transition_digest: &AUTHORITY_STATE_TRANSITION,
                },
            )
            .unwrap_err(),
            AuthorityAnchoredTimePolicyTransitionErrorV2::AuthorityStateGenerationInvalid
        );
        let zeros = [0u8; 32];
        assert_eq!(
            prepare_authority_anchored_historical_activation_time_policy_transition_v2(
                &transition,
                &subject,
                HistoricalActivationTimePolicyAuthorityStateAnchorV2 {
                    authority_state_generation: 1,
                    authority_state_transition_digest: &zeros,
                },
            )
            .unwrap_err(),
            AuthorityAnchoredTimePolicyTransitionErrorV2::AuthorityStateTransitionDigestAllZero
        );
    }

    #[test]
    fn qualified_anchored_transition_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2")
            .unwrap();
        let end = source[start..]
            .index("impl PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub base_transition_signing_digest_sha256:",
            "pub signer_authority_subject_sha256:",
            "pub authority_state_generation:",
            "pub authority_state_transition_digest:",
            "pub anchored_transition_signing_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}

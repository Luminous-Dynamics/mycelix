// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit epoch and authority-root transition contracts for the Mycelix
//! Solar-System Federation security profile.
//!
//! Ordinary snapshot evolution is intentionally forbidden from changing the
//! federation epoch, authority root, or authority generation. This crate
//! defines the separate transition object required to cross that boundary.
//!
//! It does not verify signatures, quorums, cryptographic commitments, clocks,
//! DHT state, or durable storage. A later authority adapter must verify the
//! transition authorization commitment under the exact source constitution.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::{EvidenceCommitment, PolicyDigest, SSF_SCHEMA_V1};
use mycelix_ssf_snapshots::{FederationStateHeadV1, SnapshotCommitment, SnapshotGeneration};

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            /// Construct from canonical bytes supplied by an upstream
            /// cryptographic adapter.
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            /// Return canonical digest bytes.
            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

digest_type!(TransitionAuthorizationCommitment);
digest_type!(TransitionCommitment);

/// Closed set of v1 transition semantics.
///
/// Compromise recovery is deliberately absent from v1. A compromised current
/// authority root must not be allowed to authorize its own replacement; a
/// future recovery profile needs a recovery authority precommitted by the
/// source constitution before compromise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EpochTransitionKind {
    /// Advance the epoch while retaining the exact constitutional root.
    ScheduledEpochAdvance,
    /// Planned root rotation authorized by the existing constitution.
    AuthorityRootRotation,
    /// Change the active constitution/policy at the epoch boundary.
    ConstitutionalUpgrade,
}

/// Immutable, addressed claim that one exact federation state transitions to
/// one exact next-epoch federation state.
///
/// `authorization_commitment` is evidence, not authority by itself. A later
/// adapter must prove who authorized it and under which exact source policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EpochTransitionCertificateV1 {
    pub schema_version: u16,
    pub kind: EpochTransitionKind,
    pub source: FederationStateHeadV1,
    pub target: FederationStateHeadV1,
    pub evidence_root: EvidenceCommitment,
    pub authorization_commitment: TransitionAuthorizationCommitment,
    pub certificate_commitment: TransitionCommitment,
}

impl EpochTransitionCertificateV1 {
    pub const fn new(
        kind: EpochTransitionKind,
        source: FederationStateHeadV1,
        target: FederationStateHeadV1,
        evidence_root: EvidenceCommitment,
        authorization_commitment: TransitionAuthorizationCommitment,
        certificate_commitment: TransitionCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            kind,
            source,
            target,
            evidence_root,
            authorization_commitment,
            certificate_commitment,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EpochTransitionError {
    FederationChanged,
    EpochOverflow,
    EpochNotNext,
    AuthorityGenerationOverflow,
    AuthorityGenerationNotNext,
    RevocationGenerationOverflow,
    RevocationGenerationNotNext,
    TargetMembershipNotGenesis,
    TargetRevocationNotGenesis,
    TargetPolicyNotGenesis,
    MembershipHeadDidNotAdvance,
    RevocationHeadDidNotAdvance,
    PolicyHeadDidNotAdvance,
    ScheduledAdvanceChangedRoot,
    RootRotationDidNotChangeRoot,
    ConstitutionalUpgradeDidNotChangePolicy,
}

/// Validate the pure structural theorem for an epoch transition.
///
/// This does not prove the transition is authorized. It proves only that the
/// claim has the non-ambiguous shape required before an authorization adapter
/// may consider it.
pub fn validate_epoch_transition_shape(
    certificate: &EpochTransitionCertificateV1,
) -> Result<(), EpochTransitionError> {
    let source = &certificate.source;
    let target = &certificate.target;

    if source.context.federation_id != target.context.federation_id {
        return Err(EpochTransitionError::FederationChanged);
    }

    let expected_epoch = source
        .context
        .federation_epoch
        .get()
        .checked_add(1)
        .ok_or(EpochTransitionError::EpochOverflow)?;
    if target.context.federation_epoch.get() != expected_epoch {
        return Err(EpochTransitionError::EpochNotNext);
    }

    let expected_authority_generation = source
        .context
        .authority_generation
        .get()
        .checked_add(1)
        .ok_or(EpochTransitionError::AuthorityGenerationOverflow)?;
    if target.context.authority_generation.get() != expected_authority_generation {
        return Err(EpochTransitionError::AuthorityGenerationNotNext);
    }

    let expected_revocation_generation = source
        .context
        .revocation_generation
        .get()
        .checked_add(1)
        .ok_or(EpochTransitionError::RevocationGenerationOverflow)?;
    if target.context.revocation_generation.get() != expected_revocation_generation {
        return Err(EpochTransitionError::RevocationGenerationNotNext);
    }

    if target.membership_generation != SnapshotGeneration::new(0) {
        return Err(EpochTransitionError::TargetMembershipNotGenesis);
    }
    if target.revocation_snapshot_generation != SnapshotGeneration::new(0) {
        return Err(EpochTransitionError::TargetRevocationNotGenesis);
    }
    if target.policy_snapshot_generation != SnapshotGeneration::new(0) {
        return Err(EpochTransitionError::TargetPolicyNotGenesis);
    }

    if target.membership_head == source.membership_head {
        return Err(EpochTransitionError::MembershipHeadDidNotAdvance);
    }
    if target.revocation_head == source.revocation_head {
        return Err(EpochTransitionError::RevocationHeadDidNotAdvance);
    }
    if target.policy_head == source.policy_head {
        return Err(EpochTransitionError::PolicyHeadDidNotAdvance);
    }

    match certificate.kind {
        EpochTransitionKind::ScheduledEpochAdvance => {
            if target.context.authority_root != source.context.authority_root {
                return Err(EpochTransitionError::ScheduledAdvanceChangedRoot);
            }
        }
        EpochTransitionKind::AuthorityRootRotation => {
            if target.context.authority_root == source.context.authority_root {
                return Err(EpochTransitionError::RootRotationDidNotChangeRoot);
            }
        }
        EpochTransitionKind::ConstitutionalUpgrade => {
            if target.context.policy_digest == source.context.policy_digest {
                return Err(EpochTransitionError::ConstitutionalUpgradeDidNotChangePolicy);
            }
        }
    }

    Ok(())
}

/// Compact transition anchor suitable for later reconciliation evidence.
///
/// It intentionally carries both complete endpoint state heads, rather than
/// only epoch numbers, so equal logical epochs with different snapshot heads
/// remain distinguishable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EpochTransitionAnchorV1 {
    pub source: FederationStateHeadV1,
    pub target: FederationStateHeadV1,
    pub transition: TransitionCommitment,
}

impl From<&EpochTransitionCertificateV1> for EpochTransitionAnchorV1 {
    fn from(value: &EpochTransitionCertificateV1) -> Self {
        Self {
            source: value.source,
            target: value.target,
            transition: value.certificate_commitment,
        }
    }
}

/// Return the three exact snapshot head commitments bound by a state head.
pub const fn state_head_commitments(
    head: &FederationStateHeadV1,
) -> [SnapshotCommitment; 3] {
    [
        head.membership_head,
        head.revocation_head,
        head.policy_head,
    ]
}

/// Return the active policy digest at the target side of a transition.
pub const fn target_policy_digest(certificate: &EpochTransitionCertificateV1) -> PolicyDigest {
    certificate.target.context.policy_digest
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, FederationContextV1, FederationEpoch,
        FederationId, RevocationGeneration,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn state(
        epoch: u64,
        authority_generation: u64,
        revocation_generation: u64,
        root_byte: u8,
        policy_byte: u8,
        head_base: u8,
        snapshot_generation: u64,
    ) -> FederationStateHeadV1 {
        FederationStateHeadV1 {
            context: FederationContextV1 {
                federation_id: FederationId::from_bytes(bytes(1)),
                federation_epoch: FederationEpoch::new(epoch),
                authority_root: AuthorityRootCommitment::from_bytes(bytes(root_byte)),
                authority_generation: AuthorityGeneration::new(authority_generation),
                revocation_generation: RevocationGeneration::new(revocation_generation),
                policy_digest: PolicyDigest::from_bytes(bytes(policy_byte)),
            },
            membership_head: SnapshotCommitment::from_bytes(bytes(head_base)),
            membership_generation: SnapshotGeneration::new(snapshot_generation),
            revocation_head: SnapshotCommitment::from_bytes(bytes(head_base.wrapping_add(1))),
            revocation_snapshot_generation: SnapshotGeneration::new(snapshot_generation),
            policy_head: SnapshotCommitment::from_bytes(bytes(head_base.wrapping_add(2))),
            policy_snapshot_generation: SnapshotGeneration::new(snapshot_generation),
        }
    }

    fn certificate(kind: EpochTransitionKind) -> EpochTransitionCertificateV1 {
        EpochTransitionCertificateV1::new(
            kind,
            state(7, 11, 40, 2, 3, 10, 5),
            state(8, 12, 41, 2, 3, 20, 0),
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        )
    }

    #[test]
    fn scheduled_epoch_advance_has_exact_shape() {
        assert_eq!(
            validate_epoch_transition_shape(&certificate(
                EpochTransitionKind::ScheduledEpochAdvance
            )),
            Ok(())
        );
    }

    #[test]
    fn epoch_must_advance_exactly_once() {
        let mut cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        cert.target.context.federation_epoch = FederationEpoch::new(9);
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::EpochNotNext)
        );
    }

    #[test]
    fn authority_generation_must_advance_exactly_once() {
        let mut cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        cert.target.context.authority_generation = AuthorityGeneration::new(14);
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::AuthorityGenerationNotNext)
        );
    }

    #[test]
    fn revocation_generation_advances_at_epoch_boundary() {
        let mut cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        cert.target.context.revocation_generation = RevocationGeneration::new(40);
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::RevocationGenerationNotNext)
        );
    }

    #[test]
    fn target_snapshot_domains_restart_at_genesis() {
        let mut cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        cert.target.membership_generation = SnapshotGeneration::new(1);
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::TargetMembershipNotGenesis)
        );
    }

    #[test]
    fn scheduled_advance_cannot_rotate_root() {
        let mut cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        cert.target.context.authority_root = AuthorityRootCommitment::from_bytes(bytes(9));
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::ScheduledAdvanceChangedRoot)
        );
    }

    #[test]
    fn explicit_root_rotation_requires_new_root() {
        let mut cert = certificate(EpochTransitionKind::AuthorityRootRotation);
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::RootRotationDidNotChangeRoot)
        );

        cert.target.context.authority_root = AuthorityRootCommitment::from_bytes(bytes(9));
        assert_eq!(validate_epoch_transition_shape(&cert), Ok(()));
    }

    #[test]
    fn constitutional_upgrade_requires_policy_change() {
        let mut cert = certificate(EpochTransitionKind::ConstitutionalUpgrade);
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::ConstitutionalUpgradeDidNotChangePolicy)
        );

        cert.target.context.policy_digest = PolicyDigest::from_bytes(bytes(8));
        assert_eq!(validate_epoch_transition_shape(&cert), Ok(()));
    }

    #[test]
    fn target_heads_must_be_new_epoch_specific_commitments() {
        let mut cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        cert.target.policy_head = cert.source.policy_head;
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::PolicyHeadDidNotAdvance)
        );
    }

    #[test]
    fn federation_identity_cannot_change_inside_transition() {
        let mut cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        cert.target.context.federation_id = FederationId::from_bytes(bytes(99));
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::FederationChanged)
        );
    }

    #[test]
    fn transition_anchor_retains_both_exact_state_heads() {
        let cert = certificate(EpochTransitionKind::ScheduledEpochAdvance);
        let anchor = EpochTransitionAnchorV1::from(&cert);
        assert_eq!(anchor.source, cert.source);
        assert_eq!(anchor.target, cert.target);
        assert_eq!(anchor.transition, cert.certificate_commitment);
    }
}

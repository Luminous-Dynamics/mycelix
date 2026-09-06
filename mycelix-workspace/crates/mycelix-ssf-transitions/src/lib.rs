// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit epoch and constitutional-transition contracts for the Mycelix
//! Solar-System Federation security profile.
//!
//! Ordinary snapshot evolution is forbidden from changing federation epoch,
//! authority root, or authority generation. This crate defines the separate
//! transition object required to cross that boundary.
//!
//! It does not verify signatures, quorums, cryptographic commitments, clocks,
//! DHT state, or durable storage. A later authority adapter must verify the
//! transition authorization under the exact source constitution.

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
/// Each variant has exactly one constitutional meaning. Compound root+policy
/// changes require a future explicit compound transition type rather than
/// being smuggled through one of these variants.
///
/// Compromise recovery is deliberately absent from v1. A compromised current
/// root must not be allowed to authorize its own replacement; recovery needs
/// a separately precommitted recovery authority in the source constitution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EpochTransitionKind {
    /// Advance the epoch while retaining root and active policy.
    ScheduledEpochAdvance,
    /// Planned root rotation while retaining the active policy.
    AuthorityRootRotation,
    /// Change the active constitution/policy while retaining the root.
    ConstitutionalUpgrade,
}

/// Immutable, addressed claim that one exact federation state transitions to
/// one exact next-epoch federation state.
///
/// `authorization_commitment` remains evidence, not authority by itself. A
/// later adapter must prove who authorized it and under which exact source
/// policy before any local authority can result.
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
    ScheduledAdvanceChangedPolicy,
    RootRotationDidNotChangeRoot,
    RootRotationChangedPolicy,
    ConstitutionalUpgradeChangedRoot,
    ConstitutionalUpgradeDidNotChangePolicy,
}

/// Validate the pure structural theorem for an epoch transition.
///
/// This does not prove the transition is authorized. It proves only that the
/// claim has the exact non-ambiguous shape required before an authorization
/// adapter may consider it.
pub fn validate_epoch_transition_shape(
    certificate: &EpochTransitionCertificateV1,
) -> Result<(), EpochTransitionError> {
    let source = certificate.source;
    let target = certificate.target;
    let source_context = source.context();
    let target_context = target.context();

    if source_context.federation_id != target_context.federation_id {
        return Err(EpochTransitionError::FederationChanged);
    }

    let expected_epoch = source_context
        .federation_epoch
        .get()
        .checked_add(1)
        .ok_or(EpochTransitionError::EpochOverflow)?;
    if target_context.federation_epoch.get() != expected_epoch {
        return Err(EpochTransitionError::EpochNotNext);
    }

    let expected_authority_generation = source_context
        .authority_generation
        .get()
        .checked_add(1)
        .ok_or(EpochTransitionError::AuthorityGenerationOverflow)?;
    if target_context.authority_generation.get() != expected_authority_generation {
        return Err(EpochTransitionError::AuthorityGenerationNotNext);
    }

    let expected_revocation_generation = source_context
        .revocation_generation
        .get()
        .checked_add(1)
        .ok_or(EpochTransitionError::RevocationGenerationOverflow)?;
    if target_context.revocation_generation.get() != expected_revocation_generation {
        return Err(EpochTransitionError::RevocationGenerationNotNext);
    }

    if target.membership_generation() != SnapshotGeneration::new(0) {
        return Err(EpochTransitionError::TargetMembershipNotGenesis);
    }
    if target.revocation_snapshot_generation() != SnapshotGeneration::new(0) {
        return Err(EpochTransitionError::TargetRevocationNotGenesis);
    }
    if target.policy_snapshot_generation() != SnapshotGeneration::new(0) {
        return Err(EpochTransitionError::TargetPolicyNotGenesis);
    }

    if target.membership_head() == source.membership_head() {
        return Err(EpochTransitionError::MembershipHeadDidNotAdvance);
    }
    if target.revocation_head() == source.revocation_head() {
        return Err(EpochTransitionError::RevocationHeadDidNotAdvance);
    }
    if target.policy_head() == source.policy_head() {
        return Err(EpochTransitionError::PolicyHeadDidNotAdvance);
    }

    match certificate.kind {
        EpochTransitionKind::ScheduledEpochAdvance => {
            if target_context.authority_root != source_context.authority_root {
                return Err(EpochTransitionError::ScheduledAdvanceChangedRoot);
            }
            if target_context.policy_digest != source_context.policy_digest {
                return Err(EpochTransitionError::ScheduledAdvanceChangedPolicy);
            }
        }
        EpochTransitionKind::AuthorityRootRotation => {
            if target_context.authority_root == source_context.authority_root {
                return Err(EpochTransitionError::RootRotationDidNotChangeRoot);
            }
            if target_context.policy_digest != source_context.policy_digest {
                return Err(EpochTransitionError::RootRotationChangedPolicy);
            }
        }
        EpochTransitionKind::ConstitutionalUpgrade => {
            if target_context.authority_root != source_context.authority_root {
                return Err(EpochTransitionError::ConstitutionalUpgradeChangedRoot);
            }
            if target_context.policy_digest == source_context.policy_digest {
                return Err(EpochTransitionError::ConstitutionalUpgradeDidNotChangePolicy);
            }
        }
    }

    Ok(())
}

/// Compact transition anchor suitable for later reconciliation evidence.
///
/// Both complete endpoint state heads are retained so equal logical epoch
/// numbers with different snapshot heads remain distinct histories.
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

/// Return the three exact snapshot head commitments bound by a coherent state
/// head.
pub const fn state_head_commitments(head: &FederationStateHeadV1) -> [SnapshotCommitment; 3] {
    [
        head.membership_head(),
        head.revocation_head(),
        head.policy_head(),
    ]
}

/// Return the active policy digest at the target side of a transition.
pub const fn target_policy_digest(certificate: &EpochTransitionCertificateV1) -> PolicyDigest {
    certificate.target.context().policy_digest
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, FederationEpoch, FederationId,
        RevocationGeneration,
    };
    use mycelix_ssf_snapshots::{
        assemble_federation_state_head, MembershipSnapshotV1, PolicySnapshotV1,
        RevocationSnapshotV1, SnapshotLineageV1,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    #[allow(clippy::too_many_arguments)]
    fn lineage(
        epoch: u64,
        authority_generation: u64,
        root_byte: u8,
        snapshot_generation: u64,
        commitment_byte: u8,
        content_byte: u8,
    ) -> SnapshotLineageV1 {
        let predecessor = if snapshot_generation == 0 {
            None
        } else {
            Some(SnapshotCommitment::from_bytes(bytes(
                commitment_byte.wrapping_add(100),
            )))
        };

        SnapshotLineageV1::new(
            FederationId::from_bytes(bytes(1)),
            FederationEpoch::new(epoch),
            AuthorityRootCommitment::from_bytes(bytes(root_byte)),
            AuthorityGeneration::new(authority_generation),
            SnapshotGeneration::new(snapshot_generation),
            predecessor,
            SnapshotCommitment::from_bytes(bytes(commitment_byte)),
            EvidenceCommitment::from_bytes(bytes(content_byte)),
        )
        .expect("well-formed snapshot fixture")
    }

    #[allow(clippy::too_many_arguments)]
    fn state(
        epoch: u64,
        authority_generation: u64,
        revocation_generation: u64,
        root_byte: u8,
        policy_byte: u8,
        head_base: u8,
        snapshot_generation: u64,
    ) -> FederationStateHeadV1 {
        let membership = MembershipSnapshotV1::from_lineage(lineage(
            epoch,
            authority_generation,
            root_byte,
            snapshot_generation,
            head_base,
            head_base.wrapping_add(40),
        ));
        let revocation = RevocationSnapshotV1::from_lineage(
            lineage(
                epoch,
                authority_generation,
                root_byte,
                snapshot_generation,
                head_base.wrapping_add(1),
                head_base.wrapping_add(41),
            ),
            RevocationGeneration::new(revocation_generation),
        );
        let policy = PolicySnapshotV1::from_lineage(
            lineage(
                epoch,
                authority_generation,
                root_byte,
                snapshot_generation,
                head_base.wrapping_add(2),
                head_base.wrapping_add(42),
            ),
            PolicyDigest::from_bytes(bytes(policy_byte)),
        );

        assemble_federation_state_head(&membership, &revocation, &policy)
            .expect("coherent state fixture")
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
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(9, 12, 41, 2, 3, 20, 0);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::EpochNotNext)
        );
    }

    #[test]
    fn authority_generation_must_advance_exactly_once() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 14, 41, 2, 3, 20, 0);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::AuthorityGenerationNotNext)
        );
    }

    #[test]
    fn revocation_generation_advances_at_epoch_boundary() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 40, 2, 3, 20, 0);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::RevocationGenerationNotNext)
        );
    }

    #[test]
    fn target_snapshot_domains_restart_at_genesis() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 2, 3, 20, 1);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::TargetMembershipNotGenesis)
        );
    }

    #[test]
    fn scheduled_advance_cannot_rotate_root() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 9, 3, 20, 0);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::ScheduledAdvanceChangedRoot)
        );
    }

    #[test]
    fn scheduled_advance_cannot_change_policy() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 2, 8, 20, 0);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::ScheduledAdvanceChangedPolicy)
        );
    }

    #[test]
    fn explicit_root_rotation_requires_new_root_and_same_policy() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let unchanged_root = state(8, 12, 41, 2, 3, 20, 0);
        let invalid = EpochTransitionCertificateV1::new(
            EpochTransitionKind::AuthorityRootRotation,
            source,
            unchanged_root,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&invalid),
            Err(EpochTransitionError::RootRotationDidNotChangeRoot)
        );

        let valid_target = state(8, 12, 41, 9, 3, 20, 0);
        let valid = EpochTransitionCertificateV1::new(
            EpochTransitionKind::AuthorityRootRotation,
            source,
            valid_target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(validate_epoch_transition_shape(&valid), Ok(()));
    }

    #[test]
    fn root_rotation_cannot_smuggle_policy_change() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 9, 8, 20, 0);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::AuthorityRootRotation,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::RootRotationChangedPolicy)
        );
    }

    #[test]
    fn constitutional_upgrade_requires_policy_change_and_same_root() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let unchanged_policy = state(8, 12, 41, 2, 3, 20, 0);
        let invalid = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ConstitutionalUpgrade,
            source,
            unchanged_policy,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&invalid),
            Err(EpochTransitionError::ConstitutionalUpgradeDidNotChangePolicy)
        );

        let valid_target = state(8, 12, 41, 2, 8, 20, 0);
        let valid = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ConstitutionalUpgrade,
            source,
            valid_target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(validate_epoch_transition_shape(&valid), Ok(()));
    }

    #[test]
    fn constitutional_upgrade_cannot_smuggle_root_rotation() {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 9, 8, 20, 0);
        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ConstitutionalUpgrade,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::ConstitutionalUpgradeChangedRoot)
        );
    }

    #[test]
    fn target_heads_must_be_new_epoch_specific_commitments() {
        let source = state(7, 11, 40, 2, 3, 10, 5);

        let membership = MembershipSnapshotV1::from_lineage(lineage(8, 12, 2, 0, 20, 60));
        let revocation = RevocationSnapshotV1::from_lineage(
            lineage(8, 12, 2, 0, 21, 61),
            RevocationGeneration::new(41),
        );
        let policy = PolicySnapshotV1::from_lineage(
            lineage(8, 12, 2, 0, source.policy_head().as_bytes()[0], 62),
            PolicyDigest::from_bytes(bytes(3)),
        );
        let target = assemble_federation_state_head(&membership, &revocation, &policy)
            .expect("coherent target");

        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_epoch_transition_shape(&cert),
            Err(EpochTransitionError::PolicyHeadDidNotAdvance)
        );
    }

    #[test]
    fn federation_identity_cannot_change_inside_transition() {
        let source = state(7, 11, 40, 2, 3, 10, 5);

        let membership = MembershipSnapshotV1::from_lineage(
            SnapshotLineageV1::new(
                FederationId::from_bytes(bytes(99)),
                FederationEpoch::new(8),
                AuthorityRootCommitment::from_bytes(bytes(2)),
                AuthorityGeneration::new(12),
                SnapshotGeneration::new(0),
                None,
                SnapshotCommitment::from_bytes(bytes(20)),
                EvidenceCommitment::from_bytes(bytes(60)),
            )
            .expect("membership"),
        );
        let revocation = RevocationSnapshotV1::from_lineage(
            SnapshotLineageV1::new(
                FederationId::from_bytes(bytes(99)),
                FederationEpoch::new(8),
                AuthorityRootCommitment::from_bytes(bytes(2)),
                AuthorityGeneration::new(12),
                SnapshotGeneration::new(0),
                None,
                SnapshotCommitment::from_bytes(bytes(21)),
                EvidenceCommitment::from_bytes(bytes(61)),
            )
            .expect("revocation"),
            RevocationGeneration::new(41),
        );
        let policy = PolicySnapshotV1::from_lineage(
            SnapshotLineageV1::new(
                FederationId::from_bytes(bytes(99)),
                FederationEpoch::new(8),
                AuthorityRootCommitment::from_bytes(bytes(2)),
                AuthorityGeneration::new(12),
                SnapshotGeneration::new(0),
                None,
                SnapshotCommitment::from_bytes(bytes(22)),
                EvidenceCommitment::from_bytes(bytes(62)),
            )
            .expect("policy"),
            PolicyDigest::from_bytes(bytes(3)),
        );
        let target = assemble_federation_state_head(&membership, &revocation, &policy)
            .expect("coherent foreign target");

        let cert = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
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

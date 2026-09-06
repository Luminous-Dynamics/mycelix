// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Immutable ancestry and fork-detection contracts for the Mycelix
//! Solar-System Federation security profile.
//!
//! This crate deliberately does not choose a hash algorithm, verify
//! signatures, query a DHT, read clocks, or select a "latest" record. It
//! consumes already-addressed commitments and makes the lineage rules needed
//! by later Holochain, Xenia, DTN, and storage adapters explicit.
//!
//! The central rule is that a current federation state is reconstructed from
//! exact snapshot heads and their ancestry, never from collection ordering or
//! last-writer-wins heuristics.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::{
    AuthorityGeneration, AuthorityRootCommitment, EvidenceCommitment, FederationContextV1,
    FederationEpoch, FederationId, PolicyDigest, RevocationGeneration, SSF_SCHEMA_V1,
};

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            /// Construct from canonical digest bytes verified by an upstream
            /// cryptographic adapter.
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            /// Return the canonical digest bytes.
            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

macro_rules! generation_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[repr(transparent)]
        pub struct $name(u64);

        impl $name {
            pub const fn new(value: u64) -> Self {
                Self(value)
            }

            pub const fn get(self) -> u64 {
                self.0
            }
        }
    };
}

digest_type!(SnapshotCommitment);
generation_type!(SnapshotGeneration);

/// Immutable common ancestry metadata for one snapshot node.
///
/// `commitment` and `content_root` are opaque commitments here. A later
/// cryptographic adapter must prove that `commitment` actually commits to the
/// canonical snapshot bytes containing these exact fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SnapshotLineageV1 {
    schema_version: u16,
    federation_id: FederationId,
    federation_epoch: FederationEpoch,
    authority_root: AuthorityRootCommitment,
    authority_generation: AuthorityGeneration,
    generation: SnapshotGeneration,
    predecessor: Option<SnapshotCommitment>,
    commitment: SnapshotCommitment,
    content_root: EvidenceCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapshotNodeError {
    GenesisHasPredecessor,
    NonGenesisMissingPredecessor,
}

impl SnapshotLineageV1 {
    /// Construct a structurally well-formed immutable snapshot node.
    pub fn new(
        federation_id: FederationId,
        federation_epoch: FederationEpoch,
        authority_root: AuthorityRootCommitment,
        authority_generation: AuthorityGeneration,
        generation: SnapshotGeneration,
        predecessor: Option<SnapshotCommitment>,
        commitment: SnapshotCommitment,
        content_root: EvidenceCommitment,
    ) -> Result<Self, SnapshotNodeError> {
        match (generation.get(), predecessor) {
            (0, Some(_)) => return Err(SnapshotNodeError::GenesisHasPredecessor),
            (0, None) => {}
            (_, None) => return Err(SnapshotNodeError::NonGenesisMissingPredecessor),
            (_, Some(_)) => {}
        }

        Ok(Self {
            schema_version: SSF_SCHEMA_V1,
            federation_id,
            federation_epoch,
            authority_root,
            authority_generation,
            generation,
            predecessor,
            commitment,
            content_root,
        })
    }

    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub const fn federation_id(&self) -> FederationId {
        self.federation_id
    }

    pub const fn federation_epoch(&self) -> FederationEpoch {
        self.federation_epoch
    }

    pub const fn authority_root(&self) -> AuthorityRootCommitment {
        self.authority_root
    }

    pub const fn authority_generation(&self) -> AuthorityGeneration {
        self.authority_generation
    }

    pub const fn generation(&self) -> SnapshotGeneration {
        self.generation
    }

    pub const fn predecessor(&self) -> Option<SnapshotCommitment> {
        self.predecessor
    }

    pub const fn commitment(&self) -> SnapshotCommitment {
        self.commitment
    }

    pub const fn content_root(&self) -> EvidenceCommitment {
        self.content_root
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SuccessorError {
    FederationMismatch,
    EpochMismatch,
    AuthorityRootMismatch,
    AuthorityGenerationChanged,
    GenerationOverflow,
    GenerationNotNext,
    PredecessorMismatch,
    CommitmentDidNotAdvance,
}

/// Validate one exact direct successor in a snapshot lineage.
///
/// Root rotation, federation-epoch transition, and authority-generation
/// transition are intentionally outside this operation and require a future
/// explicit transition contract. They may not be smuggled through an ordinary
/// snapshot successor.
pub fn validate_direct_successor(
    previous: &SnapshotLineageV1,
    next: &SnapshotLineageV1,
) -> Result<(), SuccessorError> {
    if previous.federation_id != next.federation_id {
        return Err(SuccessorError::FederationMismatch);
    }
    if previous.federation_epoch != next.federation_epoch {
        return Err(SuccessorError::EpochMismatch);
    }
    if previous.authority_root != next.authority_root {
        return Err(SuccessorError::AuthorityRootMismatch);
    }
    if next.authority_generation != previous.authority_generation {
        return Err(SuccessorError::AuthorityGenerationChanged);
    }

    let expected_generation = previous
        .generation
        .get()
        .checked_add(1)
        .ok_or(SuccessorError::GenerationOverflow)?;
    if next.generation.get() != expected_generation {
        return Err(SuccessorError::GenerationNotNext);
    }
    if next.predecessor != Some(previous.commitment) {
        return Err(SuccessorError::PredecessorMismatch);
    }
    if next.commitment == previous.commitment {
        return Err(SuccessorError::CommitmentDidNotAdvance);
    }

    Ok(())
}

/// Relationship between two addressed nodes without consulting time or DHT
/// enumeration order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapshotRelation {
    DifferentLineage,
    SameNode,
    ForkAtSameGeneration,
    DirectSuccessor,
    DirectPredecessor,
    NonAdjacentOrDivergent,
}

/// Compare two snapshot nodes using only explicit lineage facts.
pub fn classify_relation(a: &SnapshotLineageV1, b: &SnapshotLineageV1) -> SnapshotRelation {
    if a.federation_id != b.federation_id
        || a.federation_epoch != b.federation_epoch
        || a.authority_root != b.authority_root
    {
        return SnapshotRelation::DifferentLineage;
    }

    if a.generation == b.generation {
        return if a.commitment == b.commitment {
            SnapshotRelation::SameNode
        } else {
            SnapshotRelation::ForkAtSameGeneration
        };
    }

    if a.generation.get().checked_add(1) == Some(b.generation.get())
        && b.predecessor == Some(a.commitment)
    {
        return SnapshotRelation::DirectSuccessor;
    }

    if b.generation.get().checked_add(1) == Some(a.generation.get())
        && a.predecessor == Some(b.commitment)
    {
        return SnapshotRelation::DirectPredecessor;
    }

    SnapshotRelation::NonAdjacentOrDivergent
}

/// Membership snapshot head. Domain separation is provided by the Rust type,
/// so a policy or revocation snapshot cannot be passed accidentally where a
/// membership snapshot is required.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MembershipSnapshotV1 {
    lineage: SnapshotLineageV1,
}

impl MembershipSnapshotV1 {
    pub const fn from_lineage(lineage: SnapshotLineageV1) -> Self {
        Self { lineage }
    }

    pub const fn lineage(&self) -> &SnapshotLineageV1 {
        &self.lineage
    }
}

/// Revocation snapshot head with a separately monotonic revocation generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RevocationSnapshotV1 {
    lineage: SnapshotLineageV1,
    revocation_generation: RevocationGeneration,
}

impl RevocationSnapshotV1 {
    pub const fn from_lineage(
        lineage: SnapshotLineageV1,
        revocation_generation: RevocationGeneration,
    ) -> Self {
        Self {
            lineage,
            revocation_generation,
        }
    }

    pub const fn lineage(&self) -> &SnapshotLineageV1 {
        &self.lineage
    }

    pub const fn revocation_generation(&self) -> RevocationGeneration {
        self.revocation_generation
    }
}

/// Policy snapshot head carrying the exact active policy digest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolicySnapshotV1 {
    lineage: SnapshotLineageV1,
    active_policy: PolicyDigest,
}

impl PolicySnapshotV1 {
    pub const fn from_lineage(lineage: SnapshotLineageV1, active_policy: PolicyDigest) -> Self {
        Self {
            lineage,
            active_policy,
        }
    }

    pub const fn lineage(&self) -> &SnapshotLineageV1 {
        &self.lineage
    }

    pub const fn active_policy(&self) -> PolicyDigest {
        self.active_policy
    }
}

/// Validate a direct membership-state transition.
pub fn validate_membership_successor(
    previous: &MembershipSnapshotV1,
    next: &MembershipSnapshotV1,
) -> Result<(), SuccessorError> {
    validate_direct_successor(previous.lineage(), next.lineage())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RevocationSuccessorError {
    Lineage(SuccessorError),
    RevocationGenerationOverflow,
    RevocationGenerationNotNext,
}

/// Validate a direct revocation-state transition. Revocation generation must
/// advance exactly once; stale or skipped generations cannot become current
/// through this contract.
pub fn validate_revocation_successor(
    previous: &RevocationSnapshotV1,
    next: &RevocationSnapshotV1,
) -> Result<(), RevocationSuccessorError> {
    validate_direct_successor(previous.lineage(), next.lineage())
        .map_err(RevocationSuccessorError::Lineage)?;

    let expected = previous
        .revocation_generation
        .get()
        .checked_add(1)
        .ok_or(RevocationSuccessorError::RevocationGenerationOverflow)?;
    if next.revocation_generation.get() != expected {
        return Err(RevocationSuccessorError::RevocationGenerationNotNext);
    }

    Ok(())
}

/// Validate a direct policy-state transition.
pub fn validate_policy_successor(
    previous: &PolicySnapshotV1,
    next: &PolicySnapshotV1,
) -> Result<(), SuccessorError> {
    validate_direct_successor(previous.lineage(), next.lineage())
}

/// Exact coherent snapshot heads from which a local federation decision
/// context can be reconstructed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationStateHeadV1 {
    pub context: FederationContextV1,
    pub membership_head: SnapshotCommitment,
    pub membership_generation: SnapshotGeneration,
    pub revocation_head: SnapshotCommitment,
    pub revocation_snapshot_generation: SnapshotGeneration,
    pub policy_head: SnapshotCommitment,
    pub policy_snapshot_generation: SnapshotGeneration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateHeadError {
    FederationMismatch,
    EpochMismatch,
    AuthorityRootMismatch,
    AuthorityGenerationMismatch,
}

/// Assemble a federation decision head only from three snapshots that agree on
/// the exact federation, epoch, authority root, and authority generation.
///
/// This deliberately does not infer a winner when conflicting heads exist.
pub fn assemble_federation_state_head(
    membership: &MembershipSnapshotV1,
    revocation: &RevocationSnapshotV1,
    policy: &PolicySnapshotV1,
) -> Result<FederationStateHeadV1, StateHeadError> {
    let m = membership.lineage();
    let r = revocation.lineage();
    let p = policy.lineage();

    if m.federation_id != r.federation_id || m.federation_id != p.federation_id {
        return Err(StateHeadError::FederationMismatch);
    }
    if m.federation_epoch != r.federation_epoch || m.federation_epoch != p.federation_epoch {
        return Err(StateHeadError::EpochMismatch);
    }
    if m.authority_root != r.authority_root || m.authority_root != p.authority_root {
        return Err(StateHeadError::AuthorityRootMismatch);
    }
    if m.authority_generation != r.authority_generation
        || m.authority_generation != p.authority_generation
    {
        return Err(StateHeadError::AuthorityGenerationMismatch);
    }

    Ok(FederationStateHeadV1 {
        context: FederationContextV1 {
            federation_id: m.federation_id,
            federation_epoch: m.federation_epoch,
            authority_root: m.authority_root,
            authority_generation: m.authority_generation,
            revocation_generation: revocation.revocation_generation,
            policy_digest: policy.active_policy,
        },
        membership_head: m.commitment,
        membership_generation: m.generation,
        revocation_head: r.commitment,
        revocation_snapshot_generation: r.generation,
        policy_head: p.commitment,
        policy_snapshot_generation: p.generation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn lineage(
        generation: u64,
        predecessor: Option<SnapshotCommitment>,
        commitment_byte: u8,
    ) -> SnapshotLineageV1 {
        SnapshotLineageV1::new(
            FederationId::from_bytes(bytes(1)),
            FederationEpoch::new(7),
            AuthorityRootCommitment::from_bytes(bytes(2)),
            AuthorityGeneration::new(11),
            SnapshotGeneration::new(generation),
            predecessor,
            SnapshotCommitment::from_bytes(bytes(commitment_byte)),
            EvidenceCommitment::from_bytes(bytes(commitment_byte.wrapping_add(40))),
        )
        .expect("valid snapshot fixture")
    }

    #[test]
    fn genesis_cannot_claim_a_predecessor() {
        let result = SnapshotLineageV1::new(
            FederationId::from_bytes(bytes(1)),
            FederationEpoch::new(7),
            AuthorityRootCommitment::from_bytes(bytes(2)),
            AuthorityGeneration::new(11),
            SnapshotGeneration::new(0),
            Some(SnapshotCommitment::from_bytes(bytes(9))),
            SnapshotCommitment::from_bytes(bytes(10)),
            EvidenceCommitment::from_bytes(bytes(11)),
        );

        assert_eq!(result, Err(SnapshotNodeError::GenesisHasPredecessor));
    }

    #[test]
    fn non_genesis_requires_exact_predecessor_reference() {
        let result = SnapshotLineageV1::new(
            FederationId::from_bytes(bytes(1)),
            FederationEpoch::new(7),
            AuthorityRootCommitment::from_bytes(bytes(2)),
            AuthorityGeneration::new(11),
            SnapshotGeneration::new(1),
            None,
            SnapshotCommitment::from_bytes(bytes(10)),
            EvidenceCommitment::from_bytes(bytes(11)),
        );

        assert_eq!(
            result,
            Err(SnapshotNodeError::NonGenesisMissingPredecessor)
        );
    }

    #[test]
    fn direct_successor_requires_exact_ancestry() {
        let genesis = lineage(0, None, 10);
        let next = lineage(1, Some(genesis.commitment()), 11);

        assert_eq!(validate_direct_successor(&genesis, &next), Ok(()));
        assert_eq!(
            classify_relation(&genesis, &next),
            SnapshotRelation::DirectSuccessor
        );
    }

    #[test]
    fn generation_skip_is_rejected() {
        let genesis = lineage(0, None, 10);
        let skipped = lineage(2, Some(genesis.commitment()), 12);

        assert_eq!(
            validate_direct_successor(&genesis, &skipped),
            Err(SuccessorError::GenerationNotNext)
        );
    }

    #[test]
    fn wrong_predecessor_is_rejected() {
        let genesis = lineage(0, None, 10);
        let next = lineage(
            1,
            Some(SnapshotCommitment::from_bytes(bytes(99))),
            11,
        );

        assert_eq!(
            validate_direct_successor(&genesis, &next),
            Err(SuccessorError::PredecessorMismatch)
        );
    }

    #[test]
    fn authority_generation_cannot_change_inside_snapshot_successor() {
        let previous = lineage(0, None, 10);
        let next = SnapshotLineageV1::new(
            previous.federation_id(),
            previous.federation_epoch(),
            previous.authority_root(),
            AuthorityGeneration::new(12),
            SnapshotGeneration::new(1),
            Some(previous.commitment()),
            SnapshotCommitment::from_bytes(bytes(11)),
            EvidenceCommitment::from_bytes(bytes(51)),
        )
        .expect("valid node shape");

        assert_eq!(
            validate_direct_successor(&previous, &next),
            Err(SuccessorError::AuthorityGenerationChanged)
        );
    }

    #[test]
    fn same_generation_different_commitment_is_an_explicit_fork() {
        let parent = lineage(0, None, 10);
        let a = lineage(1, Some(parent.commitment()), 11);
        let b = lineage(1, Some(parent.commitment()), 12);

        assert_eq!(
            classify_relation(&a, &b),
            SnapshotRelation::ForkAtSameGeneration
        );
    }

    #[test]
    fn exact_duplicate_is_not_a_fork() {
        let parent = lineage(0, None, 10);
        let a = lineage(1, Some(parent.commitment()), 11);

        assert_eq!(classify_relation(&a, &a), SnapshotRelation::SameNode);
    }

    #[test]
    fn root_rotation_cannot_hide_inside_ordinary_successor() {
        let previous = lineage(0, None, 10);
        let next = SnapshotLineageV1::new(
            previous.federation_id(),
            previous.federation_epoch(),
            AuthorityRootCommitment::from_bytes(bytes(77)),
            previous.authority_generation(),
            SnapshotGeneration::new(1),
            Some(previous.commitment()),
            SnapshotCommitment::from_bytes(bytes(11)),
            EvidenceCommitment::from_bytes(bytes(51)),
        )
        .expect("valid node shape");

        assert_eq!(
            validate_direct_successor(&previous, &next),
            Err(SuccessorError::AuthorityRootMismatch)
        );
    }

    #[test]
    fn revocation_generation_must_advance_exactly_once() {
        let previous_lineage = lineage(0, None, 10);
        let next_lineage = lineage(1, Some(previous_lineage.commitment()), 11);
        let previous = RevocationSnapshotV1::from_lineage(
            previous_lineage,
            RevocationGeneration::new(40),
        );
        let next =
            RevocationSnapshotV1::from_lineage(next_lineage, RevocationGeneration::new(42));

        assert_eq!(
            validate_revocation_successor(&previous, &next),
            Err(RevocationSuccessorError::RevocationGenerationNotNext)
        );
    }

    #[test]
    fn coherent_heads_derive_exact_federation_context() {
        let membership = MembershipSnapshotV1::from_lineage(lineage(0, None, 10));
        let revocation = RevocationSnapshotV1::from_lineage(
            lineage(0, None, 20),
            RevocationGeneration::new(40),
        );
        let policy_digest = PolicyDigest::from_bytes(bytes(70));
        let policy = PolicySnapshotV1::from_lineage(lineage(0, None, 30), policy_digest);

        let head = assemble_federation_state_head(&membership, &revocation, &policy)
            .expect("coherent snapshot set");

        assert_eq!(head.context.federation_id, membership.lineage().federation_id());
        assert_eq!(head.context.revocation_generation, RevocationGeneration::new(40));
        assert_eq!(head.context.policy_digest, policy_digest);
        assert_eq!(head.membership_head, membership.lineage().commitment());
        assert_eq!(head.revocation_head, revocation.lineage().commitment());
        assert_eq!(head.policy_head, policy.lineage().commitment());
    }

    #[test]
    fn coherent_head_rejects_authority_generation_mismatch() {
        let membership = MembershipSnapshotV1::from_lineage(lineage(0, None, 10));
        let revocation = RevocationSnapshotV1::from_lineage(
            lineage(0, None, 20),
            RevocationGeneration::new(40),
        );
        let policy_lineage = SnapshotLineageV1::new(
            FederationId::from_bytes(bytes(1)),
            FederationEpoch::new(7),
            AuthorityRootCommitment::from_bytes(bytes(2)),
            AuthorityGeneration::new(12),
            SnapshotGeneration::new(0),
            None,
            SnapshotCommitment::from_bytes(bytes(30)),
            EvidenceCommitment::from_bytes(bytes(70)),
        )
        .expect("valid policy node");
        let policy = PolicySnapshotV1::from_lineage(
            policy_lineage,
            PolicyDigest::from_bytes(bytes(71)),
        );

        assert_eq!(
            assemble_federation_state_head(&membership, &revocation, &policy),
            Err(StateHeadError::AuthorityGenerationMismatch)
        );
    }
}

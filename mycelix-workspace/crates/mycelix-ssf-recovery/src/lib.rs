// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Independently pinned compromise-recovery contracts for the Mycelix
//! Solar-System Federation security profile.
//!
//! Normal federation authority must not be able to rewrite the authority used
//! to recover from compromise. Recovery therefore does not derive its trust
//! anchor from the mutable current federation state. A verifier supplies an
//! independently pinned `ExpectedRecoveryConstitutionV1`; an untrusted
//! recovery claim must match that expectation before its shape can validate.
//!
//! This crate validates structural non-substitutability only. It does not
//! verify signatures, quorums, cryptographic commitments, hardware roots,
//! clocks, DHT state, durable storage, or whether compromise actually occurred.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::{
    AuthorityRootCommitment, EvidenceCommitment, FederationId, PolicyDigest, SSF_SCHEMA_V1,
};
use mycelix_ssf_snapshots::{FederationStateHeadV1, SnapshotGeneration};

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            /// Construct from canonical 32-byte commitment bytes.
            ///
            /// This crate intentionally does not choose or implement the
            /// cryptographic commitment algorithm.
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            /// Return canonical commitment bytes.
            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

digest_type!(RecoveryConstitutionCommitment);
digest_type!(RecoveryAuthorizationCommitment);
digest_type!(RecoveryTransitionCommitment);

/// Recovery trust anchor supplied by verifier configuration or another
/// independently trusted recovery-root source.
///
/// The untrusted transition claim does not get to choose this value. A caller
/// constructing this object is asserting that the bytes came from its trusted
/// verifier configuration; a future crypto/storage adapter must establish that
/// provenance before invoking recovery validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExpectedRecoveryConstitutionV1 {
    federation_id: FederationId,
    commitment: RecoveryConstitutionCommitment,
}

impl ExpectedRecoveryConstitutionV1 {
    /// Bind one federation to the recovery constitution already trusted by the
    /// verifier.
    pub const fn from_verifier_configuration(
        federation_id: FederationId,
        commitment: RecoveryConstitutionCommitment,
    ) -> Self {
        Self {
            federation_id,
            commitment,
        }
    }

    pub const fn federation_id(&self) -> FederationId {
        self.federation_id
    }

    pub const fn commitment(&self) -> RecoveryConstitutionCommitment {
        self.commitment
    }
}

/// Exact target facts the independently authorized recovery statement intends
/// to install.
///
/// A later authorization verifier must prove that its recovery authorization
/// commitment covers this exact plan and the exact source/target state heads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RecoveryTargetPlanV1 {
    pub authority_root: AuthorityRootCommitment,
    pub policy_digest: PolicyDigest,
}

/// Untrusted, immutable recovery transition claim.
///
/// This object is not authority. It becomes only a shape-validated claim after
/// `validate_recovery_transition_shape`, and still requires independent
/// cryptographic/quorum verification before it may affect federation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecoveryTransitionClaimV1 {
    pub schema_version: u16,
    pub recovery_constitution: RecoveryConstitutionCommitment,
    pub source: FederationStateHeadV1,
    pub target: FederationStateHeadV1,
    pub target_plan: RecoveryTargetPlanV1,
    pub evidence_root: EvidenceCommitment,
    pub authorization_commitment: RecoveryAuthorizationCommitment,
    pub transition_commitment: RecoveryTransitionCommitment,
}

impl RecoveryTransitionClaimV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        recovery_constitution: RecoveryConstitutionCommitment,
        source: FederationStateHeadV1,
        target: FederationStateHeadV1,
        target_plan: RecoveryTargetPlanV1,
        evidence_root: EvidenceCommitment,
        authorization_commitment: RecoveryAuthorizationCommitment,
        transition_commitment: RecoveryTransitionCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            recovery_constitution,
            source,
            target,
            target_plan,
            evidence_root,
            authorization_commitment,
            transition_commitment,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryTransitionError {
    RecoveryConstitutionMismatch,
    RecoveryFederationMismatch,
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
    CompromisedAuthorityRootRetained,
    TargetAuthorityRootPlanMismatch,
    TargetPolicyPlanMismatch,
}

/// Opaque evidence that a recovery claim has the required structural shape
/// relative to one independently supplied recovery constitution.
///
/// This token deliberately does not mean "authorized recovery". Downstream
/// authority code must additionally require a cryptographically verified
/// recovery authorization under the exact pinned recovery constitution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShapeValidatedRecoveryTransitionV1 {
    expected_recovery_constitution: ExpectedRecoveryConstitutionV1,
    claim: RecoveryTransitionClaimV1,
}

impl ShapeValidatedRecoveryTransitionV1 {
    pub const fn expected_recovery_constitution(&self) -> ExpectedRecoveryConstitutionV1 {
        self.expected_recovery_constitution
    }

    pub const fn claim(&self) -> RecoveryTransitionClaimV1 {
        self.claim
    }
}

/// Validate the pure structural theorem for compromise recovery.
///
/// The expected recovery constitution is supplied separately from the
/// untrusted claim. This is the critical non-self-amendability boundary: the
/// compromised primary authority does not get to select which recovery
/// constitution the verifier trusts at recovery time.
pub fn validate_recovery_transition_shape(
    expected: ExpectedRecoveryConstitutionV1,
    claim: RecoveryTransitionClaimV1,
) -> Result<ShapeValidatedRecoveryTransitionV1, RecoveryTransitionError> {
    if claim.recovery_constitution != expected.commitment {
        return Err(RecoveryTransitionError::RecoveryConstitutionMismatch);
    }

    let source_context = claim.source.context();
    let target_context = claim.target.context();

    if source_context.federation_id != expected.federation_id {
        return Err(RecoveryTransitionError::RecoveryFederationMismatch);
    }
    if target_context.federation_id != source_context.federation_id {
        return Err(RecoveryTransitionError::FederationChanged);
    }

    let expected_epoch = source_context
        .federation_epoch
        .get()
        .checked_add(1)
        .ok_or(RecoveryTransitionError::EpochOverflow)?;
    if target_context.federation_epoch.get() != expected_epoch {
        return Err(RecoveryTransitionError::EpochNotNext);
    }

    let expected_authority_generation = source_context
        .authority_generation
        .get()
        .checked_add(1)
        .ok_or(RecoveryTransitionError::AuthorityGenerationOverflow)?;
    if target_context.authority_generation.get() != expected_authority_generation {
        return Err(RecoveryTransitionError::AuthorityGenerationNotNext);
    }

    let expected_revocation_generation = source_context
        .revocation_generation
        .get()
        .checked_add(1)
        .ok_or(RecoveryTransitionError::RevocationGenerationOverflow)?;
    if target_context.revocation_generation.get() != expected_revocation_generation {
        return Err(RecoveryTransitionError::RevocationGenerationNotNext);
    }

    if claim.target.membership_generation() != SnapshotGeneration::new(0) {
        return Err(RecoveryTransitionError::TargetMembershipNotGenesis);
    }
    if claim.target.revocation_snapshot_generation() != SnapshotGeneration::new(0) {
        return Err(RecoveryTransitionError::TargetRevocationNotGenesis);
    }
    if claim.target.policy_snapshot_generation() != SnapshotGeneration::new(0) {
        return Err(RecoveryTransitionError::TargetPolicyNotGenesis);
    }

    if claim.target.membership_head() == claim.source.membership_head() {
        return Err(RecoveryTransitionError::MembershipHeadDidNotAdvance);
    }
    if claim.target.revocation_head() == claim.source.revocation_head() {
        return Err(RecoveryTransitionError::RevocationHeadDidNotAdvance);
    }
    if claim.target.policy_head() == claim.source.policy_head() {
        return Err(RecoveryTransitionError::PolicyHeadDidNotAdvance);
    }

    if target_context.authority_root == source_context.authority_root {
        return Err(RecoveryTransitionError::CompromisedAuthorityRootRetained);
    }
    if target_context.authority_root != claim.target_plan.authority_root {
        return Err(RecoveryTransitionError::TargetAuthorityRootPlanMismatch);
    }
    if target_context.policy_digest != claim.target_plan.policy_digest {
        return Err(RecoveryTransitionError::TargetPolicyPlanMismatch);
    }

    Ok(ShapeValidatedRecoveryTransitionV1 {
        expected_recovery_constitution: expected,
        claim,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_contracts::{
        AuthorityGeneration, FederationEpoch, RevocationGeneration,
    };
    use mycelix_ssf_snapshots::{
        assemble_federation_state_head, MembershipSnapshotV1, PolicySnapshotV1,
        RevocationSnapshotV1, SnapshotCommitment, SnapshotLineageV1,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    #[allow(clippy::too_many_arguments)]
    fn lineage(
        federation_byte: u8,
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
            FederationId::from_bytes(bytes(federation_byte)),
            FederationEpoch::new(epoch),
            AuthorityRootCommitment::from_bytes(bytes(root_byte)),
            AuthorityGeneration::new(authority_generation),
            SnapshotGeneration::new(snapshot_generation),
            predecessor,
            SnapshotCommitment::from_bytes(bytes(commitment_byte)),
            EvidenceCommitment::from_bytes(bytes(content_byte)),
        )
        .expect("well-formed lineage fixture")
    }

    #[allow(clippy::too_many_arguments)]
    fn state(
        federation_byte: u8,
        epoch: u64,
        authority_generation: u64,
        revocation_generation: u64,
        root_byte: u8,
        policy_byte: u8,
        head_base: u8,
        snapshot_generation: u64,
    ) -> FederationStateHeadV1 {
        let membership = MembershipSnapshotV1::from_lineage(lineage(
            federation_byte,
            epoch,
            authority_generation,
            root_byte,
            snapshot_generation,
            head_base,
            head_base.wrapping_add(40),
        ));
        let revocation = RevocationSnapshotV1::from_lineage(
            lineage(
                federation_byte,
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
                federation_byte,
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

    fn expected_recovery() -> ExpectedRecoveryConstitutionV1 {
        ExpectedRecoveryConstitutionV1::from_verifier_configuration(
            FederationId::from_bytes(bytes(1)),
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
        )
    }

    fn valid_claim() -> RecoveryTransitionClaimV1 {
        let source = state(1, 7, 11, 40, 2, 3, 10, 5);
        let target = state(1, 8, 12, 41, 9, 8, 20, 0);
        RecoveryTransitionClaimV1::new(
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
            source,
            target,
            RecoveryTargetPlanV1 {
                authority_root: AuthorityRootCommitment::from_bytes(bytes(9)),
                policy_digest: PolicyDigest::from_bytes(bytes(8)),
            },
            EvidenceCommitment::from_bytes(bytes(60)),
            RecoveryAuthorizationCommitment::from_bytes(bytes(61)),
            RecoveryTransitionCommitment::from_bytes(bytes(62)),
        )
    }

    #[test]
    fn valid_recovery_requires_independently_expected_constitution() {
        let validated = validate_recovery_transition_shape(expected_recovery(), valid_claim())
            .expect("valid recovery shape");
        assert_eq!(
            validated.expected_recovery_constitution(),
            expected_recovery()
        );
        assert_eq!(
            validated.claim().transition_commitment,
            RecoveryTransitionCommitment::from_bytes(bytes(62))
        );
    }

    #[test]
    fn claim_cannot_choose_another_recovery_constitution() {
        let mut claim = valid_claim();
        claim.recovery_constitution = RecoveryConstitutionCommitment::from_bytes(bytes(91));
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::RecoveryConstitutionMismatch)
        );
    }

    #[test]
    fn expected_recovery_anchor_is_federation_specific() {
        let expected = ExpectedRecoveryConstitutionV1::from_verifier_configuration(
            FederationId::from_bytes(bytes(7)),
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
        );
        assert_eq!(
            validate_recovery_transition_shape(expected, valid_claim()),
            Err(RecoveryTransitionError::RecoveryFederationMismatch)
        );
    }

    #[test]
    fn recovery_must_replace_compromised_primary_root() {
        let source = state(1, 7, 11, 40, 2, 3, 10, 5);
        let target = state(1, 8, 12, 41, 2, 8, 20, 0);
        let claim = RecoveryTransitionClaimV1::new(
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
            source,
            target,
            RecoveryTargetPlanV1 {
                authority_root: AuthorityRootCommitment::from_bytes(bytes(2)),
                policy_digest: PolicyDigest::from_bytes(bytes(8)),
            },
            EvidenceCommitment::from_bytes(bytes(60)),
            RecoveryAuthorizationCommitment::from_bytes(bytes(61)),
            RecoveryTransitionCommitment::from_bytes(bytes(62)),
        );
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::CompromisedAuthorityRootRetained)
        );
    }

    #[test]
    fn recovery_target_root_must_match_independently_authorized_plan() {
        let mut claim = valid_claim();
        claim.target_plan.authority_root = AuthorityRootCommitment::from_bytes(bytes(10));
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::TargetAuthorityRootPlanMismatch)
        );
    }

    #[test]
    fn recovery_target_policy_must_match_independently_authorized_plan() {
        let mut claim = valid_claim();
        claim.target_plan.policy_digest = PolicyDigest::from_bytes(bytes(10));
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::TargetPolicyPlanMismatch)
        );
    }

    #[test]
    fn recovery_epoch_must_advance_exactly_once() {
        let source = state(1, 7, 11, 40, 2, 3, 10, 5);
        let target = state(1, 9, 12, 41, 9, 8, 20, 0);
        let mut claim = valid_claim();
        claim.source = source;
        claim.target = target;
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::EpochNotNext)
        );
    }

    #[test]
    fn recovery_authority_generation_must_advance_exactly_once() {
        let mut claim = valid_claim();
        claim.target = state(1, 8, 14, 41, 9, 8, 20, 0);
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::AuthorityGenerationNotNext)
        );
    }

    #[test]
    fn recovery_revocation_generation_must_advance_exactly_once() {
        let mut claim = valid_claim();
        claim.target = state(1, 8, 12, 40, 9, 8, 20, 0);
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::RevocationGenerationNotNext)
        );
    }

    #[test]
    fn recovery_target_snapshot_domains_restart_at_genesis() {
        let mut claim = valid_claim();
        claim.target = state(1, 8, 12, 41, 9, 8, 20, 1);
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::TargetMembershipNotGenesis)
        );
    }

    #[test]
    fn recovery_cannot_cross_federation_identity() {
        let mut claim = valid_claim();
        claim.target = state(7, 8, 12, 41, 9, 8, 20, 0);
        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::FederationChanged)
        );
    }

    #[test]
    fn recovery_target_heads_must_be_new_epoch_specific_state() {
        let source = state(1, 7, 11, 40, 2, 3, 10, 5);

        let membership = MembershipSnapshotV1::from_lineage(lineage(1, 8, 12, 9, 0, 20, 60));
        let revocation = RevocationSnapshotV1::from_lineage(
            lineage(1, 8, 12, 9, 0, 21, 61),
            RevocationGeneration::new(41),
        );
        let policy = PolicySnapshotV1::from_lineage(
            lineage(1, 8, 12, 9, 0, source.policy_head().as_bytes()[0], 62),
            PolicyDigest::from_bytes(bytes(8)),
        );
        let target = assemble_federation_state_head(&membership, &revocation, &policy)
            .expect("coherent target");

        let claim = RecoveryTransitionClaimV1::new(
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
            source,
            target,
            RecoveryTargetPlanV1 {
                authority_root: AuthorityRootCommitment::from_bytes(bytes(9)),
                policy_digest: PolicyDigest::from_bytes(bytes(8)),
            },
            EvidenceCommitment::from_bytes(bytes(60)),
            RecoveryAuthorizationCommitment::from_bytes(bytes(61)),
            RecoveryTransitionCommitment::from_bytes(bytes(62)),
        );

        assert_eq!(
            validate_recovery_transition_shape(expected_recovery(), claim),
            Err(RecoveryTransitionError::PolicyHeadDidNotAdvance)
        );
    }
}

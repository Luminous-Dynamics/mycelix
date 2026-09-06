// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Verifier-bound authorization evidence for the Mycelix Solar-System
//! Federation security profile.
//!
//! This crate does not decide which verifier implementation is trusted. It
//! provides the structural boundary needed by a later authority adapter to say:
//!
//! "this exact shape-validated transition was accepted by this exact concrete
//! verifier type, running this exact verifier identity/policy/generation, and
//! the returned authorization receipt rebinds to the exact checked claim."
//!
//! A verifier-bound token is still evidence, not effect authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::{EvidenceCommitment, SSF_SCHEMA_V1};
use mycelix_ssf_recovery::{
    RecoveryAuthorizationCommitment, RecoveryConstitutionCommitment, RecoveryTargetPlanV1,
    RecoveryTransitionCommitment, ShapeValidatedRecoveryTransitionV1,
};
use mycelix_ssf_transitions::{
    ShapeValidatedEpochTransitionV1, TransitionAuthorizationCommitment, TransitionCommitment,
};

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

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

digest_type!(VerifierIdentityCommitment);
digest_type!(VerifierPolicyCommitment);
digest_type!(AuthorizationReceiptCommitment);
generation_type!(VerifierGeneration);

/// Exact runtime verifier generation/configuration bound to one authorization
/// result.
///
/// `valid_until` uses the canonical time basis chosen by the consuming profile.
/// It is an upper bound: a receipt may be shorter-lived but may not outlive the
/// verifier generation that produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VerifierDescriptorV1 {
    pub identity: VerifierIdentityCommitment,
    pub policy: VerifierPolicyCommitment,
    pub generation: VerifierGeneration,
    pub valid_until: u64,
}

/// Untrusted receipt produced by a normal constitutional-transition verifier.
///
/// This receipt is not accepted merely because a verifier implementation
/// returned `Ok`. `verify_epoch_authorization` rebinds every security-relevant
/// field to the exact shape-validated transition and exact runtime descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EpochAuthorizationReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub source: mycelix_ssf_snapshots::FederationStateHeadV1,
    pub target: mycelix_ssf_snapshots::FederationStateHeadV1,
    pub transition_commitment: TransitionCommitment,
    pub evidence_root: EvidenceCommitment,
    pub authorization_commitment: TransitionAuthorizationCommitment,
    pub valid_until: u64,
    pub receipt_commitment: AuthorizationReceiptCommitment,
}

impl EpochAuthorizationReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        verifier: VerifierDescriptorV1,
        source: mycelix_ssf_snapshots::FederationStateHeadV1,
        target: mycelix_ssf_snapshots::FederationStateHeadV1,
        transition_commitment: TransitionCommitment,
        evidence_root: EvidenceCommitment,
        authorization_commitment: TransitionAuthorizationCommitment,
        valid_until: u64,
        receipt_commitment: AuthorizationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            source,
            target,
            transition_commitment,
            evidence_root,
            authorization_commitment,
            valid_until,
            receipt_commitment,
        }
    }
}

/// Untrusted receipt produced by an independently pinned recovery verifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecoveryAuthorizationReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub recovery_constitution: RecoveryConstitutionCommitment,
    pub source: mycelix_ssf_snapshots::FederationStateHeadV1,
    pub target: mycelix_ssf_snapshots::FederationStateHeadV1,
    pub target_plan: RecoveryTargetPlanV1,
    pub transition_commitment: RecoveryTransitionCommitment,
    pub evidence_root: EvidenceCommitment,
    pub authorization_commitment: RecoveryAuthorizationCommitment,
    pub valid_until: u64,
    pub receipt_commitment: AuthorizationReceiptCommitment,
}

impl RecoveryAuthorizationReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        verifier: VerifierDescriptorV1,
        recovery_constitution: RecoveryConstitutionCommitment,
        source: mycelix_ssf_snapshots::FederationStateHeadV1,
        target: mycelix_ssf_snapshots::FederationStateHeadV1,
        target_plan: RecoveryTargetPlanV1,
        transition_commitment: RecoveryTransitionCommitment,
        evidence_root: EvidenceCommitment,
        authorization_commitment: RecoveryAuthorizationCommitment,
        valid_until: u64,
        receipt_commitment: AuthorizationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            recovery_constitution,
            source,
            target,
            target_plan,
            transition_commitment,
            evidence_root,
            authorization_commitment,
            valid_until,
            receipt_commitment,
        }
    }
}

/// A concrete implementation may verify normal constitutional-transition
/// authorization.
///
/// Trust is intentionally external to this trait. A later authority adapter is
/// expected to accept only explicitly approved concrete verifier types.
pub trait EpochAuthorizationVerifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn verify_epoch_authorization(
        &self,
        transition: &ShapeValidatedEpochTransitionV1,
    ) -> Result<EpochAuthorizationReceiptV1, Self::Error>;
}

/// A concrete implementation may verify compromise-recovery authorization.
///
/// This is deliberately a separate trait: being approved for ordinary
/// constitutional transitions does not grant recovery verification authority.
pub trait RecoveryAuthorizationVerifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn verify_recovery_authorization(
        &self,
        transition: &ShapeValidatedRecoveryTransitionV1,
    ) -> Result<RecoveryAuthorizationReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorizationBindingError<E> {
    Verifier(E),
    UnsupportedReceiptSchema,
    VerifierDescriptorMismatch,
    SourceMismatch,
    TargetMismatch,
    TransitionCommitmentMismatch,
    EvidenceRootMismatch,
    AuthorizationCommitmentMismatch,
    RecoveryConstitutionMismatch,
    RecoveryTargetPlanMismatch,
    ReceiptOutlivesVerifier,
}

/// Exact normal-transition authorization evidence bound to verifier type `V`.
///
/// There is no public constructor and the token is intentionally not `Copy` or
/// `Clone`. A downstream authority layer can require a particular concrete
/// type such as `VerifierBoundEpochAuthorizationV1<XeniaVerifier>` instead of
/// accepting a generic "verified" flag.
pub struct VerifierBoundEpochAuthorizationV1<V> {
    transition: ShapeValidatedEpochTransitionV1,
    verifier: VerifierDescriptorV1,
    receipt: EpochAuthorizationReceiptV1,
    _verifier_type: PhantomData<fn() -> V>,
}

impl<V> VerifierBoundEpochAuthorizationV1<V> {
    pub const fn transition(&self) -> &ShapeValidatedEpochTransitionV1 {
        &self.transition
    }

    pub const fn verifier(&self) -> VerifierDescriptorV1 {
        self.verifier
    }

    pub const fn receipt(&self) -> &EpochAuthorizationReceiptV1 {
        &self.receipt
    }

    pub const fn valid_until(&self) -> u64 {
        self.receipt.valid_until
    }

    pub const fn receipt_commitment(&self) -> AuthorizationReceiptCommitment {
        self.receipt.receipt_commitment
    }

    /// This token is authorization evidence only; it contains no effect
    /// authority.
    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Exact recovery authorization evidence bound to verifier type `V`.
pub struct VerifierBoundRecoveryAuthorizationV1<V> {
    transition: ShapeValidatedRecoveryTransitionV1,
    verifier: VerifierDescriptorV1,
    receipt: RecoveryAuthorizationReceiptV1,
    _verifier_type: PhantomData<fn() -> V>,
}

impl<V> VerifierBoundRecoveryAuthorizationV1<V> {
    pub const fn transition(&self) -> &ShapeValidatedRecoveryTransitionV1 {
        &self.transition
    }

    pub const fn verifier(&self) -> VerifierDescriptorV1 {
        self.verifier
    }

    pub const fn receipt(&self) -> &RecoveryAuthorizationReceiptV1 {
        &self.receipt
    }

    pub const fn valid_until(&self) -> u64 {
        self.receipt.valid_until
    }

    pub const fn receipt_commitment(&self) -> AuthorizationReceiptCommitment {
        self.receipt.receipt_commitment
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Verify and structurally rebind an exact normal transition to concrete
/// verifier type `V` and its current runtime descriptor.
pub fn verify_epoch_authorization<V: EpochAuthorizationVerifierV1>(
    verifier: &V,
    transition: ShapeValidatedEpochTransitionV1,
) -> Result<VerifierBoundEpochAuthorizationV1<V>, AuthorizationBindingError<V::Error>> {
    let expected_verifier = verifier.descriptor();
    let receipt = verifier
        .verify_epoch_authorization(&transition)
        .map_err(AuthorizationBindingError::Verifier)?;
    let certificate = transition.certificate();

    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(AuthorizationBindingError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected_verifier {
        return Err(AuthorizationBindingError::VerifierDescriptorMismatch);
    }
    if receipt.source != certificate.source {
        return Err(AuthorizationBindingError::SourceMismatch);
    }
    if receipt.target != certificate.target {
        return Err(AuthorizationBindingError::TargetMismatch);
    }
    if receipt.transition_commitment != certificate.certificate_commitment {
        return Err(AuthorizationBindingError::TransitionCommitmentMismatch);
    }
    if receipt.evidence_root != certificate.evidence_root {
        return Err(AuthorizationBindingError::EvidenceRootMismatch);
    }
    if receipt.authorization_commitment != certificate.authorization_commitment {
        return Err(AuthorizationBindingError::AuthorizationCommitmentMismatch);
    }
    if receipt.valid_until > expected_verifier.valid_until {
        return Err(AuthorizationBindingError::ReceiptOutlivesVerifier);
    }

    Ok(VerifierBoundEpochAuthorizationV1 {
        transition,
        verifier: expected_verifier,
        receipt,
        _verifier_type: PhantomData,
    })
}

/// Verify and structurally rebind an exact compromise-recovery transition to
/// concrete verifier type `V` and its current runtime descriptor.
pub fn verify_recovery_authorization<V: RecoveryAuthorizationVerifierV1>(
    verifier: &V,
    transition: ShapeValidatedRecoveryTransitionV1,
) -> Result<VerifierBoundRecoveryAuthorizationV1<V>, AuthorizationBindingError<V::Error>> {
    let expected_verifier = verifier.descriptor();
    let receipt = verifier
        .verify_recovery_authorization(&transition)
        .map_err(AuthorizationBindingError::Verifier)?;
    let claim = transition.claim();
    let expected_recovery = transition.expected_recovery_constitution();

    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(AuthorizationBindingError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected_verifier {
        return Err(AuthorizationBindingError::VerifierDescriptorMismatch);
    }
    if receipt.recovery_constitution != expected_recovery.commitment() {
        return Err(AuthorizationBindingError::RecoveryConstitutionMismatch);
    }
    if receipt.source != claim.source {
        return Err(AuthorizationBindingError::SourceMismatch);
    }
    if receipt.target != claim.target {
        return Err(AuthorizationBindingError::TargetMismatch);
    }
    if receipt.target_plan != claim.target_plan {
        return Err(AuthorizationBindingError::RecoveryTargetPlanMismatch);
    }
    if receipt.transition_commitment != claim.transition_commitment {
        return Err(AuthorizationBindingError::TransitionCommitmentMismatch);
    }
    if receipt.evidence_root != claim.evidence_root {
        return Err(AuthorizationBindingError::EvidenceRootMismatch);
    }
    if receipt.authorization_commitment != claim.authorization_commitment {
        return Err(AuthorizationBindingError::AuthorizationCommitmentMismatch);
    }
    if receipt.valid_until > expected_verifier.valid_until {
        return Err(AuthorizationBindingError::ReceiptOutlivesVerifier);
    }

    Ok(VerifierBoundRecoveryAuthorizationV1 {
        transition,
        verifier: expected_verifier,
        receipt,
        _verifier_type: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, FederationEpoch, FederationId, PolicyDigest,
        RevocationGeneration,
    };
    use mycelix_ssf_recovery::{
        validate_recovery_transition_shape, ExpectedRecoveryConstitutionV1,
        RecoveryTransitionClaimV1,
    };
    use mycelix_ssf_snapshots::{
        assemble_federation_state_head, FederationStateHeadV1, MembershipSnapshotV1,
        PolicySnapshotV1, RevocationSnapshotV1, SnapshotCommitment, SnapshotGeneration,
        SnapshotLineageV1,
    };
    use mycelix_ssf_transitions::{
        shape_validate_epoch_transition, EpochTransitionCertificateV1, EpochTransitionKind,
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
        .expect("lineage fixture")
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
            .expect("state fixture")
    }

    fn epoch_transition() -> ShapeValidatedEpochTransitionV1 {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 2, 3, 20, 0);
        let certificate = EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        );
        shape_validate_epoch_transition(certificate).expect("epoch transition")
    }

    fn recovery_transition() -> ShapeValidatedRecoveryTransitionV1 {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 9, 8, 30, 0);
        let expected = ExpectedRecoveryConstitutionV1::from_verifier_configuration(
            source.context().federation_id,
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
        );
        let claim = RecoveryTransitionClaimV1::new(
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
            source,
            target,
            RecoveryTargetPlanV1 {
                authority_root: target.context().authority_root,
                policy_digest: target.context().policy_digest,
            },
            EvidenceCommitment::from_bytes(bytes(60)),
            RecoveryAuthorizationCommitment::from_bytes(bytes(61)),
            RecoveryTransitionCommitment::from_bytes(bytes(62)),
        );
        validate_recovery_transition_shape(expected, claim).expect("recovery transition")
    }

    fn descriptor() -> VerifierDescriptorV1 {
        VerifierDescriptorV1 {
            identity: VerifierIdentityCommitment::from_bytes(bytes(100)),
            policy: VerifierPolicyCommitment::from_bytes(bytes(101)),
            generation: VerifierGeneration::new(7),
            valid_until: 5_000,
        }
    }

    struct ApprovedEpochVerifier {
        mutate: Option<EpochMutation>,
    }

    #[derive(Clone, Copy)]
    enum EpochMutation {
        Descriptor,
        Source,
        Evidence,
        Authorization,
        Lifetime,
        Schema,
    }

    impl EpochAuthorizationVerifierV1 for ApprovedEpochVerifier {
        type Error = ();

        fn descriptor(&self) -> VerifierDescriptorV1 {
            descriptor()
        }

        fn verify_epoch_authorization(
            &self,
            transition: &ShapeValidatedEpochTransitionV1,
        ) -> Result<EpochAuthorizationReceiptV1, Self::Error> {
            let certificate = transition.certificate();
            let mut receipt = EpochAuthorizationReceiptV1::new(
                descriptor(),
                certificate.source,
                certificate.target,
                certificate.certificate_commitment,
                certificate.evidence_root,
                certificate.authorization_commitment,
                4_900,
                AuthorizationReceiptCommitment::from_bytes(bytes(102)),
            );

            match self.mutate {
                Some(EpochMutation::Descriptor) => receipt.verifier.generation = VerifierGeneration::new(8),
                Some(EpochMutation::Source) => receipt.source = certificate.target,
                Some(EpochMutation::Evidence) => {
                    receipt.evidence_root = EvidenceCommitment::from_bytes(bytes(200))
                }
                Some(EpochMutation::Authorization) => {
                    receipt.authorization_commitment =
                        TransitionAuthorizationCommitment::from_bytes(bytes(201))
                }
                Some(EpochMutation::Lifetime) => receipt.valid_until = 5_001,
                Some(EpochMutation::Schema) => receipt.schema_version = SSF_SCHEMA_V1 + 1,
                None => {}
            }
            Ok(receipt)
        }
    }

    struct AlternateEpochVerifier;

    impl EpochAuthorizationVerifierV1 for AlternateEpochVerifier {
        type Error = ();

        fn descriptor(&self) -> VerifierDescriptorV1 {
            descriptor()
        }

        fn verify_epoch_authorization(
            &self,
            transition: &ShapeValidatedEpochTransitionV1,
        ) -> Result<EpochAuthorizationReceiptV1, Self::Error> {
            let certificate = transition.certificate();
            Ok(EpochAuthorizationReceiptV1::new(
                descriptor(),
                certificate.source,
                certificate.target,
                certificate.certificate_commitment,
                certificate.evidence_root,
                certificate.authorization_commitment,
                4_900,
                AuthorizationReceiptCommitment::from_bytes(bytes(103)),
            ))
        }
    }

    struct ApprovedRecoveryVerifier;

    impl RecoveryAuthorizationVerifierV1 for ApprovedRecoveryVerifier {
        type Error = ();

        fn descriptor(&self) -> VerifierDescriptorV1 {
            descriptor()
        }

        fn verify_recovery_authorization(
            &self,
            transition: &ShapeValidatedRecoveryTransitionV1,
        ) -> Result<RecoveryAuthorizationReceiptV1, Self::Error> {
            let claim = transition.claim();
            Ok(RecoveryAuthorizationReceiptV1::new(
                descriptor(),
                transition.expected_recovery_constitution().commitment(),
                claim.source,
                claim.target,
                claim.target_plan,
                claim.transition_commitment,
                claim.evidence_root,
                claim.authorization_commitment,
                4_800,
                AuthorizationReceiptCommitment::from_bytes(bytes(104)),
            ))
        }
    }

    fn requires_approved_epoch_type(_: &VerifierBoundEpochAuthorizationV1<ApprovedEpochVerifier>) {}

    #[test]
    fn exact_epoch_authorization_is_bound_to_concrete_verifier_type_and_generation() {
        let token = verify_epoch_authorization(
            &ApprovedEpochVerifier { mutate: None },
            epoch_transition(),
        )
        .expect("bound authorization");
        requires_approved_epoch_type(&token);
        assert_eq!(token.verifier(), descriptor());
        assert_eq!(token.valid_until(), 4_900);
        assert!(!token.contains_effect_authority());
    }

    #[test]
    fn another_verifier_type_can_verify_but_produces_a_distinct_token_type() {
        let token = verify_epoch_authorization(&AlternateEpochVerifier, epoch_transition())
            .expect("alternate verifier may create its own evidence type");
        assert_eq!(token.verifier(), descriptor());
        // `requires_approved_epoch_type(&token)` intentionally does not compile:
        // the concrete verifier type is part of the token type.
    }

    #[test]
    fn runtime_verifier_generation_substitution_is_rejected() {
        assert!(matches!(
            verify_epoch_authorization(
                &ApprovedEpochVerifier {
                    mutate: Some(EpochMutation::Descriptor),
                },
                epoch_transition(),
            ),
            Err(AuthorizationBindingError::VerifierDescriptorMismatch)
        ));
    }

    #[test]
    fn exact_source_evidence_and_authorization_bindings_are_enforced() {
        for mutation in [
            EpochMutation::Source,
            EpochMutation::Evidence,
            EpochMutation::Authorization,
        ] {
            assert!(verify_epoch_authorization(
                &ApprovedEpochVerifier {
                    mutate: Some(mutation),
                },
                epoch_transition(),
            )
            .is_err());
        }
    }

    #[test]
    fn receipt_cannot_outlive_verifier_generation() {
        assert!(matches!(
            verify_epoch_authorization(
                &ApprovedEpochVerifier {
                    mutate: Some(EpochMutation::Lifetime),
                },
                epoch_transition(),
            ),
            Err(AuthorizationBindingError::ReceiptOutlivesVerifier)
        ));
    }

    #[test]
    fn unknown_receipt_schema_cannot_inherit_v1_authorization_semantics() {
        assert!(matches!(
            verify_epoch_authorization(
                &ApprovedEpochVerifier {
                    mutate: Some(EpochMutation::Schema),
                },
                epoch_transition(),
            ),
            Err(AuthorizationBindingError::UnsupportedReceiptSchema)
        ));
    }

    #[test]
    fn recovery_authorization_rebinds_pinned_recovery_domain_and_target_plan() {
        let transition = recovery_transition();
        let expected_recovery = transition.expected_recovery_constitution();
        let token = verify_recovery_authorization(&ApprovedRecoveryVerifier, transition)
            .expect("bound recovery authorization");
        assert_eq!(
            token.receipt().recovery_constitution,
            expected_recovery.commitment()
        );
        assert_eq!(token.valid_until(), 4_800);
        assert!(!token.contains_effect_authority());
    }
}

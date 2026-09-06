// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Independently approved verifier profiles for the Mycelix Solar-System
//! Federation security profile.
//!
//! `mycelix-ssf-authorization` proves that one exact shape-validated claim was
//! rebound to one concrete verifier type and the runtime descriptor reported by
//! that verifier. This crate adds the next trust boundary: the verifier's
//! identity/policy/generation must also equal a descriptor independently
//! expected by local trusted configuration.
//!
//! The expected profile is supplied separately from the verifier instance. The
//! verifier therefore cannot choose which identity/policy/generation the caller
//! considers approved at verification time.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_authorization::{
    verify_epoch_authorization, verify_recovery_authorization, AuthorizationBindingError,
    EpochAuthorizationVerifierV1, RecoveryAuthorizationVerifierV1, VerifierBoundEpochAuthorizationV1,
    VerifierBoundRecoveryAuthorizationV1, VerifierDescriptorV1,
};
use mycelix_ssf_recovery::ShapeValidatedRecoveryTransitionV1;
use mycelix_ssf_transitions::ShapeValidatedEpochTransitionV1;

/// Independently expected verifier profile for ordinary constitutional
/// transitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExpectedEpochVerifierProfileV1 {
    descriptor: VerifierDescriptorV1,
}

impl ExpectedEpochVerifierProfileV1 {
    /// Construct from local trusted configuration.
    ///
    /// This pure crate cannot prove that provenance; the caller is responsible
    /// for obtaining this value from its trusted policy/configuration boundary.
    pub const fn from_trusted_configuration(descriptor: VerifierDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> VerifierDescriptorV1 {
        self.descriptor
    }
}

/// Independently expected verifier profile for compromise recovery.
///
/// This is a separate type so approval for normal transitions cannot be passed
/// accidentally where recovery-verifier approval is required.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExpectedRecoveryVerifierProfileV1 {
    descriptor: VerifierDescriptorV1,
}

impl ExpectedRecoveryVerifierProfileV1 {
    pub const fn from_trusted_configuration(descriptor: VerifierDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> VerifierDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApprovedVerifierError<E> {
    /// The verifier did not begin verification in the independently expected
    /// identity/policy/generation.
    ExpectedVerifierMismatch,
    /// The lower verifier-bound authorization theorem failed.
    AuthorizationBinding(AuthorizationBindingError<E>),
    /// The verifier descriptor changed across the verification operation.
    VerifierChangedDuringVerification,
    /// The lower token does not retain the independently expected descriptor.
    BoundVerifierMismatch,
    /// Defensive check: authorization evidence cannot outlive the independently
    /// approved verifier profile.
    EvidenceOutlivesApprovedVerifier,
}

/// Ordinary-transition authorization evidence bound to both concrete verifier
/// type `V` and an independently approved runtime verifier profile.
///
/// No public constructor exists. This remains evidence, not local policy
/// qualification and not effect authority.
pub struct ApprovedEpochVerifierAuthorizationV1<V> {
    expected: ExpectedEpochVerifierProfileV1,
    bound: VerifierBoundEpochAuthorizationV1<V>,
}

impl<V> ApprovedEpochVerifierAuthorizationV1<V> {
    pub const fn expected_profile(&self) -> ExpectedEpochVerifierProfileV1 {
        self.expected
    }

    pub const fn bound_authorization(&self) -> &VerifierBoundEpochAuthorizationV1<V> {
        &self.bound
    }

    pub const fn valid_until(&self) -> u64 {
        self.bound.valid_until()
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn contains_local_policy_qualification(&self) -> bool {
        false
    }
}

/// Recovery authorization evidence bound to both concrete verifier type `V`
/// and an independently approved recovery-verifier profile.
pub struct ApprovedRecoveryVerifierAuthorizationV1<V> {
    expected: ExpectedRecoveryVerifierProfileV1,
    bound: VerifierBoundRecoveryAuthorizationV1<V>,
}

impl<V> ApprovedRecoveryVerifierAuthorizationV1<V> {
    pub const fn expected_profile(&self) -> ExpectedRecoveryVerifierProfileV1 {
        self.expected
    }

    pub const fn bound_authorization(&self) -> &VerifierBoundRecoveryAuthorizationV1<V> {
        &self.bound
    }

    pub const fn valid_until(&self) -> u64 {
        self.bound.valid_until()
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn contains_local_policy_qualification(&self) -> bool {
        false
    }
}

/// Verify one normal transition under an independently pinned verifier
/// identity/policy/generation.
///
/// The descriptor is sampled before and after the lower verification call. A
/// generation/configuration change during verification prevents an approved
/// token from being issued.
pub fn verify_epoch_with_approved_profile<V: EpochAuthorizationVerifierV1>(
    expected: ExpectedEpochVerifierProfileV1,
    verifier: &V,
    transition: ShapeValidatedEpochTransitionV1,
) -> Result<ApprovedEpochVerifierAuthorizationV1<V>, ApprovedVerifierError<V::Error>> {
    let before = verifier.descriptor();
    if before != expected.descriptor {
        return Err(ApprovedVerifierError::ExpectedVerifierMismatch);
    }

    let bound = verify_epoch_authorization(verifier, transition)
        .map_err(ApprovedVerifierError::AuthorizationBinding)?;

    let after = verifier.descriptor();
    if after != before {
        return Err(ApprovedVerifierError::VerifierChangedDuringVerification);
    }
    if bound.verifier() != expected.descriptor {
        return Err(ApprovedVerifierError::BoundVerifierMismatch);
    }
    if bound.valid_until() > expected.descriptor.valid_until {
        return Err(ApprovedVerifierError::EvidenceOutlivesApprovedVerifier);
    }

    Ok(ApprovedEpochVerifierAuthorizationV1 { expected, bound })
}

/// Verify one compromise-recovery transition under an independently pinned
/// recovery-verifier identity/policy/generation.
pub fn verify_recovery_with_approved_profile<V: RecoveryAuthorizationVerifierV1>(
    expected: ExpectedRecoveryVerifierProfileV1,
    verifier: &V,
    transition: ShapeValidatedRecoveryTransitionV1,
) -> Result<ApprovedRecoveryVerifierAuthorizationV1<V>, ApprovedVerifierError<V::Error>> {
    let before = verifier.descriptor();
    if before != expected.descriptor {
        return Err(ApprovedVerifierError::ExpectedVerifierMismatch);
    }

    let bound = verify_recovery_authorization(verifier, transition)
        .map_err(ApprovedVerifierError::AuthorizationBinding)?;

    let after = verifier.descriptor();
    if after != before {
        return Err(ApprovedVerifierError::VerifierChangedDuringVerification);
    }
    if bound.verifier() != expected.descriptor {
        return Err(ApprovedVerifierError::BoundVerifierMismatch);
    }
    if bound.valid_until() > expected.descriptor.valid_until {
        return Err(ApprovedVerifierError::EvidenceOutlivesApprovedVerifier);
    }

    Ok(ApprovedRecoveryVerifierAuthorizationV1 { expected, bound })
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;

    use super::*;
    use mycelix_ssf_authorization::{
        AuthorizationReceiptCommitment, EpochAuthorizationReceiptV1,
        RecoveryAuthorizationReceiptV1, VerifierGeneration, VerifierIdentityCommitment,
        VerifierPolicyCommitment,
    };
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, EvidenceCommitment, FederationEpoch,
        FederationId, PolicyDigest, RevocationGeneration,
    };
    use mycelix_ssf_recovery::{
        validate_recovery_transition_shape, ExpectedRecoveryConstitutionV1,
        RecoveryAuthorizationCommitment, RecoveryConstitutionCommitment, RecoveryTargetPlanV1,
        RecoveryTransitionClaimV1, RecoveryTransitionCommitment,
    };
    use mycelix_ssf_snapshots::{
        assemble_federation_state_head, FederationStateHeadV1, MembershipSnapshotV1,
        PolicySnapshotV1, RevocationSnapshotV1, SnapshotCommitment, SnapshotGeneration,
        SnapshotLineageV1,
    };
    use mycelix_ssf_transitions::{
        shape_validate_epoch_transition, EpochTransitionCertificateV1, EpochTransitionKind,
        TransitionAuthorizationCommitment, TransitionCommitment,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn descriptor(generation: u64) -> VerifierDescriptorV1 {
        VerifierDescriptorV1 {
            identity: VerifierIdentityCommitment::from_bytes(bytes(100)),
            policy: VerifierPolicyCommitment::from_bytes(bytes(101)),
            generation: VerifierGeneration::new(generation),
            valid_until: 5_000,
        }
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
        .expect("lineage")
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
        assemble_federation_state_head(&membership, &revocation, &policy).expect("state")
    }

    fn epoch_transition() -> ShapeValidatedEpochTransitionV1 {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 2, 3, 20, 0);
        shape_validate_epoch_transition(EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(60)),
            TransitionAuthorizationCommitment::from_bytes(bytes(61)),
            TransitionCommitment::from_bytes(bytes(62)),
        ))
        .expect("transition")
    }

    fn recovery_transition() -> ShapeValidatedRecoveryTransitionV1 {
        let source = state(7, 11, 40, 2, 3, 10, 5);
        let target = state(8, 12, 41, 9, 8, 30, 0);
        let expected = ExpectedRecoveryConstitutionV1::from_verifier_configuration(
            source.context().federation_id,
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
        );
        validate_recovery_transition_shape(
            expected,
            RecoveryTransitionClaimV1::new(
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
            ),
        )
        .expect("recovery")
    }

    struct StableEpochVerifier;

    impl EpochAuthorizationVerifierV1 for StableEpochVerifier {
        type Error = ();

        fn descriptor(&self) -> VerifierDescriptorV1 {
            descriptor(7)
        }

        fn verify_epoch_authorization(
            &self,
            transition: &ShapeValidatedEpochTransitionV1,
        ) -> Result<EpochAuthorizationReceiptV1, Self::Error> {
            let certificate = transition.certificate();
            Ok(EpochAuthorizationReceiptV1::new(
                descriptor(7),
                certificate.source,
                certificate.target,
                certificate.certificate_commitment,
                certificate.evidence_root,
                certificate.authorization_commitment,
                4_900,
                AuthorizationReceiptCommitment::from_bytes(bytes(110)),
            ))
        }
    }

    struct ChangingEpochVerifier {
        descriptor_calls: Cell<u8>,
    }

    impl EpochAuthorizationVerifierV1 for ChangingEpochVerifier {
        type Error = ();

        fn descriptor(&self) -> VerifierDescriptorV1 {
            let call = self.descriptor_calls.get();
            self.descriptor_calls.set(call.saturating_add(1));
            // Child pre-check and lower-layer pre-check both observe generation
            // 7. The child post-check observes generation 8.
            if call < 2 {
                descriptor(7)
            } else {
                descriptor(8)
            }
        }

        fn verify_epoch_authorization(
            &self,
            transition: &ShapeValidatedEpochTransitionV1,
        ) -> Result<EpochAuthorizationReceiptV1, Self::Error> {
            let certificate = transition.certificate();
            Ok(EpochAuthorizationReceiptV1::new(
                descriptor(7),
                certificate.source,
                certificate.target,
                certificate.certificate_commitment,
                certificate.evidence_root,
                certificate.authorization_commitment,
                4_900,
                AuthorizationReceiptCommitment::from_bytes(bytes(111)),
            ))
        }
    }

    struct StableRecoveryVerifier;

    impl RecoveryAuthorizationVerifierV1 for StableRecoveryVerifier {
        type Error = ();

        fn descriptor(&self) -> VerifierDescriptorV1 {
            descriptor(7)
        }

        fn verify_recovery_authorization(
            &self,
            transition: &ShapeValidatedRecoveryTransitionV1,
        ) -> Result<RecoveryAuthorizationReceiptV1, Self::Error> {
            let claim = transition.claim();
            Ok(RecoveryAuthorizationReceiptV1::new(
                descriptor(7),
                transition.expected_recovery_constitution().commitment(),
                claim.source,
                claim.target,
                claim.target_plan,
                claim.transition_commitment,
                claim.evidence_root,
                claim.authorization_commitment,
                4_800,
                AuthorizationReceiptCommitment::from_bytes(bytes(112)),
            ))
        }
    }

    fn requires_stable_epoch_type(
        _: &ApprovedEpochVerifierAuthorizationV1<StableEpochVerifier>,
    ) {
    }

    #[test]
    fn exact_expected_epoch_verifier_profile_is_required() {
        let expected = ExpectedEpochVerifierProfileV1::from_trusted_configuration(descriptor(7));
        let token = verify_epoch_with_approved_profile(
            expected,
            &StableEpochVerifier,
            epoch_transition(),
        )
        .expect("approved verifier");
        requires_stable_epoch_type(&token);
        assert_eq!(token.expected_profile(), expected);
        assert_eq!(token.valid_until(), 4_900);
        assert!(!token.contains_effect_authority());
        assert!(!token.contains_local_policy_qualification());
    }

    #[test]
    fn verifier_cannot_self_select_a_different_approved_generation() {
        let expected = ExpectedEpochVerifierProfileV1::from_trusted_configuration(descriptor(8));
        assert!(matches!(
            verify_epoch_with_approved_profile(expected, &StableEpochVerifier, epoch_transition()),
            Err(ApprovedVerifierError::ExpectedVerifierMismatch)
        ));
    }

    #[test]
    fn verifier_generation_change_during_verification_is_rejected() {
        let expected = ExpectedEpochVerifierProfileV1::from_trusted_configuration(descriptor(7));
        let verifier = ChangingEpochVerifier {
            descriptor_calls: Cell::new(0),
        };
        assert!(matches!(
            verify_epoch_with_approved_profile(expected, &verifier, epoch_transition()),
            Err(ApprovedVerifierError::VerifierChangedDuringVerification)
        ));
    }

    #[test]
    fn recovery_approval_is_a_separate_profile_type_and_trait() {
        let expected =
            ExpectedRecoveryVerifierProfileV1::from_trusted_configuration(descriptor(7));
        let token = verify_recovery_with_approved_profile(
            expected,
            &StableRecoveryVerifier,
            recovery_transition(),
        )
        .expect("approved recovery verifier");
        assert_eq!(token.expected_profile(), expected);
        assert_eq!(token.valid_until(), 4_800);
        assert!(!token.contains_effect_authority());
        assert!(!token.contains_local_policy_qualification());
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact authorization coverage for a reconciled remote SSF history.
//!
//! This crate proves only that every candidate step in one exact remote causal
//! path has approved-verifier authorization evidence and that the exact local
//! and remote paths reproduce the reconciliation assessment/safety floor being
//! used. It does not perform local policy qualification, choose a branch, mint
//! authority, or execute effects.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::{array, marker::PhantomData};

use mycelix_ssf_approved_verifier::{
    ApprovedEpochVerifierAuthorizationV1, ApprovedRecoveryVerifierAuthorizationV1,
};
use mycelix_ssf_authorization::{AuthorizationReceiptCommitment, VerifierDescriptorV1};
use mycelix_ssf_reconciliation::{
    candidate_step_from_epoch_transition, candidate_step_from_recovery_transition,
    reconcile_candidate_histories, CandidateHistoryStepV1, CausalRelationV1, HistoryStepKindV1,
    ReconciliationError,
};
use mycelix_ssf_reconciliation_safety::{
    ReconciliationSafetyFloorV1, RemotePromotionGateV1,
};

/// Conservative implementation bound for one exact retained path.
pub const MAX_AUTHORIZED_REMOTE_PATH_STEPS_V1: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StepAuthorizationDomainV1 {
    NormalConstitutional,
    IndependentRecovery,
}

/// One exact candidate history step carrying approved-verifier evidence.
///
/// `EV` and `RV` preserve the concrete normal/recovery verifier types across
/// type erasure of the receipt payload. Fields are private; the only public
/// constructors consume an approved-verifier token of the matching type.
pub struct AuthorizedCandidateHistoryStepV1<EV, RV> {
    candidate: CandidateHistoryStepV1,
    authorization_domain: StepAuthorizationDomainV1,
    verifier: VerifierDescriptorV1,
    receipt_commitment: AuthorizationReceiptCommitment,
    valid_until: u64,
    _verifier_types: PhantomData<fn() -> (EV, RV)>,
}

impl<EV, RV> AuthorizedCandidateHistoryStepV1<EV, RV> {
    pub const fn candidate(&self) -> CandidateHistoryStepV1 {
        self.candidate
    }

    pub const fn authorization_domain(&self) -> StepAuthorizationDomainV1 {
        self.authorization_domain
    }

    pub const fn verifier(&self) -> VerifierDescriptorV1 {
        self.verifier
    }

    pub const fn receipt_commitment(&self) -> AuthorizationReceiptCommitment {
        self.receipt_commitment
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn contains_local_policy_qualification(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Consume approved normal-transition verifier evidence and preserve its
/// concrete verifier type in one path step.
pub fn authorized_epoch_step<EV, RV>(
    approved: ApprovedEpochVerifierAuthorizationV1<EV>,
) -> AuthorizedCandidateHistoryStepV1<EV, RV> {
    let bound = approved.bound_authorization();
    let candidate = candidate_step_from_epoch_transition(bound.transition());
    AuthorizedCandidateHistoryStepV1 {
        candidate,
        authorization_domain: StepAuthorizationDomainV1::NormalConstitutional,
        verifier: approved.expected_profile().descriptor(),
        receipt_commitment: bound.receipt_commitment(),
        valid_until: approved.valid_until(),
        _verifier_types: PhantomData,
    }
}

/// Consume approved recovery-verifier evidence and preserve its concrete
/// recovery verifier type in one path step.
pub fn authorized_recovery_step<EV, RV>(
    approved: ApprovedRecoveryVerifierAuthorizationV1<RV>,
) -> AuthorizedCandidateHistoryStepV1<EV, RV> {
    let bound = approved.bound_authorization();
    let candidate = candidate_step_from_recovery_transition(bound.transition());
    AuthorizedCandidateHistoryStepV1 {
        candidate,
        authorization_domain: StepAuthorizationDomainV1::IndependentRecovery,
        verifier: approved.expected_profile().descriptor(),
        receipt_commitment: bound.receipt_commitment(),
        valid_until: approved.valid_until(),
        _verifier_types: PhantomData,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorizationCoverageError {
    PromotionGateNotEligible,
    RelationNotRemoteStrictExtension,
    RemoteAuthorizationNotRequiredByFloor,
    EmptyRemotePath,
    RemotePathTooLong,
    LocalStepCountMismatch,
    RemoteStepCountMismatch,
    Reconciliation(ReconciliationError),
    SafetyFloorAssessmentMismatch,
    RequiredRecoveryAuthorizationMissing,
}

/// Opaque proof that one exact retained remote path is fully covered by
/// approved-verifier evidence and reproduces the exact reconciliation safety
/// floor used for later policy qualification.
///
/// The token retains both exact candidate paths, avoiding a validate-slice-then-
/// substitute gap. It remains non-authoritative.
pub struct RemotePathAuthorizationCoverageV1<EV, RV, const L: usize, const R: usize> {
    floor: ReconciliationSafetyFloorV1,
    local_path: [CandidateHistoryStepV1; L],
    remote_path: [AuthorizedCandidateHistoryStepV1<EV, RV>; R],
    valid_until: u64,
    contains_recovery: bool,
}

impl<EV, RV, const L: usize, const R: usize>
    RemotePathAuthorizationCoverageV1<EV, RV, L, R>
{
    pub const fn safety_floor(&self) -> ReconciliationSafetyFloorV1 {
        self.floor
    }

    pub const fn local_path(&self) -> &[CandidateHistoryStepV1; L] {
        &self.local_path
    }

    pub const fn remote_path(&self) -> &[AuthorizedCandidateHistoryStepV1<EV, RV>; R] {
        &self.remote_path
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn contains_recovery_authorization(&self) -> bool {
        self.contains_recovery
    }

    pub const fn contains_local_policy_qualification(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Prove approved-verifier authorization coverage over the entire exact remote
/// path from the reconciliation common ancestor to the declared remote tip.
///
/// The local candidate path is retained as well because the safety-floor
/// summary does not uniquely identify a path. Reconciliation is re-run over
/// these exact arrays; its result must exactly equal the assessment from which
/// the supplied safety floor was derived.
pub fn prove_remote_path_authorization_coverage<EV, RV, const L: usize, const R: usize>(
    floor: ReconciliationSafetyFloorV1,
    local_path: [CandidateHistoryStepV1; L],
    remote_path: [AuthorizedCandidateHistoryStepV1<EV, RV>; R],
) -> Result<RemotePathAuthorizationCoverageV1<EV, RV, L, R>, AuthorizationCoverageError> {
    if floor.remote_promotion_gate() != RemotePromotionGateV1::EligibleOnlyAfterLocalQualification {
        return Err(AuthorizationCoverageError::PromotionGateNotEligible);
    }
    if floor.assessment().relation() != CausalRelationV1::RemoteStrictExtension {
        return Err(AuthorizationCoverageError::RelationNotRemoteStrictExtension);
    }
    if !floor.require_remote_transition_authorization() {
        return Err(AuthorizationCoverageError::RemoteAuthorizationNotRequiredByFloor);
    }
    if R == 0 {
        return Err(AuthorizationCoverageError::EmptyRemotePath);
    }
    if R > MAX_AUTHORIZED_REMOTE_PATH_STEPS_V1 {
        return Err(AuthorizationCoverageError::RemotePathTooLong);
    }

    let assessment = floor.assessment();
    if L != assessment.local_step_count() as usize {
        return Err(AuthorizationCoverageError::LocalStepCountMismatch);
    }
    if R != assessment.remote_step_count() as usize {
        return Err(AuthorizationCoverageError::RemoteStepCountMismatch);
    }

    let remote_candidates: [CandidateHistoryStepV1; R] =
        array::from_fn(|index| remote_path[index].candidate());
    let rederived = reconcile_candidate_histories(
        assessment.common_ancestor(),
        Some(&local_path),
        assessment.local_tip(),
        Some(&remote_candidates),
        assessment.remote_tip(),
    )
    .map_err(AuthorizationCoverageError::Reconciliation)?;

    if rederived != assessment {
        return Err(AuthorizationCoverageError::SafetyFloorAssessmentMismatch);
    }

    let mut valid_until = u64::MAX;
    let mut contains_recovery = false;
    for step in &remote_path {
        valid_until = valid_until.min(step.valid_until());
        if step.authorization_domain() == StepAuthorizationDomainV1::IndependentRecovery {
            contains_recovery = true;
        }
    }

    if floor.require_recovery_authorization() && !contains_recovery {
        return Err(AuthorizationCoverageError::RequiredRecoveryAuthorizationMissing);
    }

    Ok(RemotePathAuthorizationCoverageV1 {
        floor,
        local_path,
        remote_path,
        valid_until,
        contains_recovery,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_approved_verifier::{
        verify_epoch_with_approved_profile, verify_recovery_with_approved_profile,
        ExpectedEpochVerifierProfileV1, ExpectedRecoveryVerifierProfileV1,
    };
    use mycelix_ssf_authorization::{
        AuthorizationReceiptCommitment, EpochAuthorizationReceiptV1,
        EpochAuthorizationVerifierV1, RecoveryAuthorizationReceiptV1,
        RecoveryAuthorizationVerifierV1, VerifierGeneration, VerifierIdentityCommitment,
        VerifierPolicyCommitment,
    };
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, EvidenceCommitment, FederationEpoch,
        FederationId, PolicyDigest, RevocationGeneration,
    };
    use mycelix_ssf_reconciliation::{
        candidate_step_from_epoch_transition, candidate_step_from_recovery_transition,
    };
    use mycelix_ssf_reconciliation_safety::derive_reconciliation_safety_floor;
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
        ShapeValidatedEpochTransitionV1, TransitionAuthorizationCommitment, TransitionCommitment,
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

    fn descriptor() -> VerifierDescriptorV1 {
        VerifierDescriptorV1 {
            identity: VerifierIdentityCommitment::from_bytes(bytes(100)),
            policy: VerifierPolicyCommitment::from_bytes(bytes(101)),
            generation: VerifierGeneration::new(7),
            valid_until: 5_000,
        }
    }

    struct EpochVerifier {
        receipt_valid_until: u64,
    }

    impl EpochAuthorizationVerifierV1 for EpochVerifier {
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
                self.receipt_valid_until,
                AuthorizationReceiptCommitment::from_bytes(*certificate.certificate_commitment.as_bytes()),
            ))
        }
    }

    struct RecoveryVerifier;

    impl RecoveryAuthorizationVerifierV1 for RecoveryVerifier {
        type Error = ();

        fn descriptor(&self) -> VerifierDescriptorV1 {
            descriptor()
        }

        fn verify_recovery_authorization(
            &self,
            transition: &mycelix_ssf_recovery::ShapeValidatedRecoveryTransitionV1,
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
                4_700,
                AuthorizationReceiptCommitment::from_bytes(*claim.transition_commitment.as_bytes()),
            ))
        }
    }

    fn epoch_shape(
        source: FederationStateHeadV1,
        target: FederationStateHeadV1,
        commitment_byte: u8,
    ) -> ShapeValidatedEpochTransitionV1 {
        shape_validate_epoch_transition(EpochTransitionCertificateV1::new(
            EpochTransitionKind::ScheduledEpochAdvance,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(commitment_byte.wrapping_add(20))),
            TransitionAuthorizationCommitment::from_bytes(bytes(commitment_byte.wrapping_add(40))),
            TransitionCommitment::from_bytes(bytes(commitment_byte)),
        ))
        .expect("epoch transition")
    }

    fn approved_epoch(
        shape: ShapeValidatedEpochTransitionV1,
        valid_until: u64,
    ) -> mycelix_ssf_approved_verifier::ApprovedEpochVerifierAuthorizationV1<EpochVerifier> {
        verify_epoch_with_approved_profile(
            ExpectedEpochVerifierProfileV1::from_trusted_configuration(descriptor()),
            &EpochVerifier {
                receipt_valid_until: valid_until,
            },
            shape,
        )
        .expect("approved epoch")
    }

    #[test]
    fn entire_remote_path_is_retained_and_covered() {
        let a = state(7, 11, 40, 2, 3, 10, 5);
        let e8 = state(8, 12, 41, 2, 3, 20, 0);
        let e9 = state(9, 13, 42, 2, 3, 30, 0);
        let s1 = epoch_shape(a, e8, 70);
        let s2 = epoch_shape(e8, e9, 71);
        let local = [candidate_step_from_epoch_transition(&s1)];
        let remote_candidates = [
            candidate_step_from_epoch_transition(&s1),
            candidate_step_from_epoch_transition(&s2),
        ];
        let assessment = reconcile_candidate_histories(
            a,
            Some(&local),
            e8,
            Some(&remote_candidates),
            e9,
        )
        .expect("reconciliation");
        let floor = derive_reconciliation_safety_floor(assessment);
        let remote = [
            authorized_epoch_step::<EpochVerifier, RecoveryVerifier>(approved_epoch(s1, 4_900)),
            authorized_epoch_step::<EpochVerifier, RecoveryVerifier>(approved_epoch(s2, 4_600)),
        ];

        let coverage = prove_remote_path_authorization_coverage(floor, local, remote)
            .expect("coverage");
        assert_eq!(coverage.remote_path().len(), 2);
        assert_eq!(coverage.local_path().len(), 1);
        assert_eq!(coverage.valid_until(), 4_600);
        assert!(!coverage.contains_effect_authority());
        assert!(!coverage.contains_local_policy_qualification());
    }

    #[test]
    fn incomplete_remote_authorization_path_is_rejected() {
        let a = state(7, 11, 40, 2, 3, 10, 5);
        let e8 = state(8, 12, 41, 2, 3, 20, 0);
        let e9 = state(9, 13, 42, 2, 3, 30, 0);
        let s1 = epoch_shape(a, e8, 70);
        let s2 = epoch_shape(e8, e9, 71);
        let local = [candidate_step_from_epoch_transition(&s1)];
        let remote_candidates = [
            candidate_step_from_epoch_transition(&s1),
            candidate_step_from_epoch_transition(&s2),
        ];
        let assessment = reconcile_candidate_histories(
            a,
            Some(&local),
            e8,
            Some(&remote_candidates),
            e9,
        )
        .expect("reconciliation");
        let floor = derive_reconciliation_safety_floor(assessment);
        let incomplete = [authorized_epoch_step::<EpochVerifier, RecoveryVerifier>(
            approved_epoch(s1, 4_900),
        )];

        assert!(matches!(
            prove_remote_path_authorization_coverage(floor, local, incomplete),
            Err(AuthorizationCoverageError::RemoteStepCountMismatch)
        ));
    }

    #[test]
    fn recovery_on_remote_path_requires_and_preserves_recovery_authorization() {
        let a = state(7, 11, 40, 2, 3, 10, 5);
        let recovered = state(8, 12, 41, 9, 8, 40, 0);
        let expected_recovery = ExpectedRecoveryConstitutionV1::from_verifier_configuration(
            a.context().federation_id,
            RecoveryConstitutionCommitment::from_bytes(bytes(90)),
        );
        let recovery_shape = validate_recovery_transition_shape(
            expected_recovery,
            RecoveryTransitionClaimV1::new(
                RecoveryConstitutionCommitment::from_bytes(bytes(90)),
                a,
                recovered,
                RecoveryTargetPlanV1 {
                    authority_root: recovered.context().authority_root,
                    policy_digest: recovered.context().policy_digest,
                },
                EvidenceCommitment::from_bytes(bytes(60)),
                RecoveryAuthorizationCommitment::from_bytes(bytes(61)),
                RecoveryTransitionCommitment::from_bytes(bytes(62)),
            ),
        )
        .expect("recovery shape");
        let local: [CandidateHistoryStepV1; 0] = [];
        let remote_candidates = [candidate_step_from_recovery_transition(&recovery_shape)];
        let assessment = reconcile_candidate_histories(
            a,
            Some(&local),
            a,
            Some(&remote_candidates),
            recovered,
        )
        .expect("reconciliation");
        let floor = derive_reconciliation_safety_floor(assessment);
        let approved = verify_recovery_with_approved_profile(
            ExpectedRecoveryVerifierProfileV1::from_trusted_configuration(descriptor()),
            &RecoveryVerifier,
            recovery_shape,
        )
        .expect("approved recovery");
        let remote = [authorized_recovery_step::<EpochVerifier, RecoveryVerifier>(approved)];

        let coverage = prove_remote_path_authorization_coverage(floor, local, remote)
            .expect("recovery coverage");
        assert!(coverage.contains_recovery_authorization());
        assert_eq!(coverage.valid_until(), 4_700);
    }

    #[test]
    fn fork_safety_floor_cannot_be_promoted_into_path_coverage() {
        let a = state(7, 11, 40, 2, 3, 10, 5);
        let local_tip = state(8, 12, 41, 2, 3, 20, 0);
        let remote_tip = state(8, 12, 41, 9, 3, 30, 0);
        let local_shape = epoch_shape(a, local_tip, 70);
        let remote_shape = shape_validate_epoch_transition(EpochTransitionCertificateV1::new(
            EpochTransitionKind::AuthorityRootRotation,
            a,
            remote_tip,
            EvidenceCommitment::from_bytes(bytes(80)),
            TransitionAuthorizationCommitment::from_bytes(bytes(81)),
            TransitionCommitment::from_bytes(bytes(82)),
        ))
        .expect("root rotation");
        let local = [candidate_step_from_epoch_transition(&local_shape)];
        let remote_candidates = [candidate_step_from_epoch_transition(&remote_shape)];
        let assessment = reconcile_candidate_histories(
            a,
            Some(&local),
            local_tip,
            Some(&remote_candidates),
            remote_tip,
        )
        .expect("fork");
        let floor = derive_reconciliation_safety_floor(assessment);
        let remote = [authorized_epoch_step::<EpochVerifier, RecoveryVerifier>(approved_epoch(
            remote_shape,
            4_900,
        ))];

        assert!(matches!(
            prove_remote_path_authorization_coverage(floor, local, remote),
            Err(AuthorizationCoverageError::PromotionGateNotEligible)
        ));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Minimum safety requirements derived from one exact SSF reconciliation
//! assessment.
//!
//! This crate does not choose a winning history, install state, verify
//! signatures, execute effects, or mint authority. It translates causal
//! evidence into a **floor** that later local policy/authority adapters may
//! strengthen but must not weaken.
//!
//! The invariant is simple:
//!
//! **Remote history remains evidence-only until separate local authority
//! explicitly qualifies any state-changing use of it.**

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_reconciliation::{
    CandidateReconciliationAssessmentV1, CausalRelationV1, RecoveryPresenceV1,
};

/// Policy-facing handling class derived from causal evidence.
///
/// These are not effects and are not authority decisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReconciliationHandlingClassV1 {
    /// Local and remote present the exact same candidate history.
    ConsistentEvidence,
    /// Local history is an exact strict extension of the remote history.
    LocalHistoryAhead,
    /// Remote history is a candidate strict extension that still requires
    /// local qualification before it can affect local authority.
    RemoteCandidateExtension,
    /// A non-trivial causal path is missing.
    CausalProofRequired,
    /// Complete histories diverge and require explicit reconciliation.
    ForkResolutionRequired,
    /// The histories belong to different federation identities.
    ForeignFederationEvidenceOnly,
}

/// Closed-world minimum gate for any attempt to promote remote-derived state.
///
/// This is deliberately stronger than a bag of booleans: a downstream adapter
/// cannot reinterpret a fork as merely "needs local policy approval" while
/// forgetting that causal/fork resolution must happen first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RemotePromotionGateV1 {
    /// There is no remote state delta to promote: histories are identical or
    /// local is already the strict extension.
    NoPromotionNeeded,
    /// Exact remote strict extension is the only v1 relation that can become a
    /// promotion candidate after all required authorization and local policy
    /// qualification gates pass.
    EligibleOnlyAfterLocalQualification,
    /// Missing causal evidence blocks consideration of remote promotion.
    BlockedPendingCausalProof,
    /// Divergent or convergent-fork history blocks promotion until an explicit
    /// local fork-resolution process produces new qualified evidence.
    BlockedPendingForkResolution,
    /// Direct state promotion across federation identity is forbidden. Foreign
    /// history may only enter as evidence for a separate local qualification.
    ForbiddenCrossFederation,
}

/// Opaque minimum safety requirements tied to one exact reconciliation
/// assessment.
///
/// No public constructor exists. Later policy layers can inspect this floor but
/// cannot use it as effect authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReconciliationSafetyFloorV1 {
    assessment: CandidateReconciliationAssessmentV1,
    handling_class: ReconciliationHandlingClassV1,
    remote_promotion_gate: RemotePromotionGateV1,
    require_complete_causal_path: bool,
    require_remote_transition_authorization: bool,
    require_recovery_authorization: bool,
    require_explicit_fork_resolution: bool,
    require_common_ancestor_qualification: bool,
    require_local_policy_qualification: bool,
}

impl ReconciliationSafetyFloorV1 {
    /// Exact reconciliation evidence this floor was derived from.
    pub const fn assessment(&self) -> CandidateReconciliationAssessmentV1 {
        self.assessment
    }

    /// High-level policy-facing handling class.
    pub const fn handling_class(&self) -> ReconciliationHandlingClassV1 {
        self.handling_class
    }

    /// Closed-world minimum promotion gate derived from the causal relation.
    pub const fn remote_promotion_gate(&self) -> RemotePromotionGateV1 {
        self.remote_promotion_gate
    }

    /// Whether a complete causal path must be obtained before any remote state
    /// promotion can even be considered.
    pub const fn require_complete_causal_path(&self) -> bool {
        self.require_complete_causal_path
    }

    /// Whether the exact remote transition path requires separate
    /// cryptographic/quorum authorization before state-changing use.
    pub const fn require_remote_transition_authorization(&self) -> bool {
        self.require_remote_transition_authorization
    }

    /// Whether independently pinned recovery authorization must be proven.
    pub const fn require_recovery_authorization(&self) -> bool {
        self.require_recovery_authorization
    }

    /// Whether an explicit local fork-resolution process is required.
    pub const fn require_explicit_fork_resolution(&self) -> bool {
        self.require_explicit_fork_resolution
    }

    /// Whether the claimed common ancestor itself must be qualified before
    /// remote state can be considered for local promotion.
    pub const fn require_common_ancestor_qualification(&self) -> bool {
        self.require_common_ancestor_qualification
    }

    /// Whether any remote-derived state-changing use must pass an exact local
    /// policy qualification boundary.
    pub const fn require_local_policy_qualification(&self) -> bool {
        self.require_local_policy_qualification
    }

    /// SSF invariant: remote state is evidence-only at this layer.
    pub const fn remote_state_is_evidence_only(&self) -> bool {
        true
    }

    /// SSF invariant: this floor contains no effect authority.
    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    /// SSF invariant: no reconciliation relation automatically adopts remote
    /// authority.
    pub const fn automatic_remote_adoption_forbidden(&self) -> bool {
        true
    }
}

fn remote_recovery_may_matter(
    relation: CausalRelationV1,
    recovery_presence: RecoveryPresenceV1,
) -> bool {
    match relation {
        CausalRelationV1::RemoteStrictExtension
        | CausalRelationV1::Fork
        | CausalRelationV1::ConvergentFork => matches!(
            recovery_presence,
            RecoveryPresenceV1::RemoteOnly | RecoveryPresenceV1::Both
        ),
        // With an unproven path, recovery presence is commonly Unknown. Stay
        // conservative until complete causal evidence permits re-derivation.
        CausalRelationV1::UnprovenGap => matches!(
            recovery_presence,
            RecoveryPresenceV1::RemoteOnly
                | RecoveryPresenceV1::Both
                | RecoveryPresenceV1::Unknown
        ),
        CausalRelationV1::SameHistory
        | CausalRelationV1::LocalStrictExtension
        | CausalRelationV1::DifferentFederation => false,
    }
}

/// Derive the minimum non-authoritative safety floor for one exact
/// reconciliation assessment.
///
/// New evidence should produce a new reconciliation assessment and therefore
/// a newly derived floor; callers must not "clear" requirements on an existing
/// floor by convention.
pub fn derive_reconciliation_safety_floor(
    assessment: CandidateReconciliationAssessmentV1,
) -> ReconciliationSafetyFloorV1 {
    let relation = assessment.relation();
    let recovery_presence = assessment.recovery_presence();

    let handling_class = match relation {
        CausalRelationV1::SameHistory => ReconciliationHandlingClassV1::ConsistentEvidence,
        CausalRelationV1::LocalStrictExtension => {
            ReconciliationHandlingClassV1::LocalHistoryAhead
        }
        CausalRelationV1::RemoteStrictExtension => {
            ReconciliationHandlingClassV1::RemoteCandidateExtension
        }
        CausalRelationV1::UnprovenGap => ReconciliationHandlingClassV1::CausalProofRequired,
        CausalRelationV1::Fork | CausalRelationV1::ConvergentFork => {
            ReconciliationHandlingClassV1::ForkResolutionRequired
        }
        CausalRelationV1::DifferentFederation => {
            ReconciliationHandlingClassV1::ForeignFederationEvidenceOnly
        }
    };

    let remote_promotion_gate = match relation {
        CausalRelationV1::SameHistory | CausalRelationV1::LocalStrictExtension => {
            RemotePromotionGateV1::NoPromotionNeeded
        }
        CausalRelationV1::RemoteStrictExtension => {
            RemotePromotionGateV1::EligibleOnlyAfterLocalQualification
        }
        CausalRelationV1::UnprovenGap => RemotePromotionGateV1::BlockedPendingCausalProof,
        CausalRelationV1::Fork | CausalRelationV1::ConvergentFork => {
            RemotePromotionGateV1::BlockedPendingForkResolution
        }
        CausalRelationV1::DifferentFederation => {
            RemotePromotionGateV1::ForbiddenCrossFederation
        }
    };

    let remote_state_change_is_candidate = matches!(
        relation,
        CausalRelationV1::RemoteStrictExtension
            | CausalRelationV1::Fork
            | CausalRelationV1::ConvergentFork
            | CausalRelationV1::UnprovenGap
    );

    ReconciliationSafetyFloorV1 {
        assessment,
        handling_class,
        remote_promotion_gate,
        require_complete_causal_path: relation == CausalRelationV1::UnprovenGap,
        require_remote_transition_authorization: remote_state_change_is_candidate,
        require_recovery_authorization: remote_recovery_may_matter(relation, recovery_presence),
        require_explicit_fork_resolution: matches!(
            relation,
            CausalRelationV1::Fork | CausalRelationV1::ConvergentFork
        ),
        require_common_ancestor_qualification: remote_state_change_is_candidate,
        require_local_policy_qualification: remote_state_change_is_candidate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, EvidenceCommitment, FederationEpoch,
        FederationId, PolicyDigest, RevocationGeneration,
    };
    use mycelix_ssf_reconciliation::{
        candidate_step_from_epoch_transition, candidate_step_from_recovery_transition,
        reconcile_candidate_histories, CandidateHistoryStepV1,
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
        .expect("lineage fixture")
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
            .expect("state fixture")
    }

    fn normal_step(
        kind: EpochTransitionKind,
        source: FederationStateHeadV1,
        target: FederationStateHeadV1,
        commitment_byte: u8,
    ) -> CandidateHistoryStepV1 {
        let certificate = EpochTransitionCertificateV1::new(
            kind,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(commitment_byte.wrapping_add(20))),
            TransitionAuthorizationCommitment::from_bytes(bytes(
                commitment_byte.wrapping_add(40),
            )),
            TransitionCommitment::from_bytes(bytes(commitment_byte)),
        );
        let validated =
            shape_validate_epoch_transition(certificate).expect("normal transition fixture");
        candidate_step_from_epoch_transition(&validated)
    }

    fn recovery_step(
        source: FederationStateHeadV1,
        target: FederationStateHeadV1,
        commitment_byte: u8,
    ) -> CandidateHistoryStepV1 {
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
            EvidenceCommitment::from_bytes(bytes(commitment_byte.wrapping_add(20))),
            RecoveryAuthorizationCommitment::from_bytes(bytes(
                commitment_byte.wrapping_add(40),
            )),
            RecoveryTransitionCommitment::from_bytes(bytes(commitment_byte)),
        );
        let validated =
            validate_recovery_transition_shape(expected, claim).expect("recovery fixture");
        candidate_step_from_recovery_transition(&validated)
    }

    fn ancestor() -> FederationStateHeadV1 {
        state(1, 7, 11, 40, 2, 3, 10, 5)
    }

    fn normal_e8() -> FederationStateHeadV1 {
        state(1, 8, 12, 41, 2, 3, 20, 0)
    }

    fn assessment_remote_extension() -> CandidateReconciliationAssessmentV1 {
        let a = ancestor();
        let e8 = normal_e8();
        let remote = [normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            a,
            e8,
            70,
        )];
        reconcile_candidate_histories(a, None, a, Some(&remote), e8)
            .expect("remote extension assessment")
    }

    #[test]
    fn every_floor_is_evidence_only_and_contains_no_effect_authority() {
        let floor = derive_reconciliation_safety_floor(assessment_remote_extension());
        assert!(floor.remote_state_is_evidence_only());
        assert!(floor.automatic_remote_adoption_forbidden());
        assert!(!floor.contains_effect_authority());
    }

    #[test]
    fn remote_extension_is_the_only_relation_eligible_for_later_promotion() {
        let floor = derive_reconciliation_safety_floor(assessment_remote_extension());
        assert_eq!(
            floor.remote_promotion_gate(),
            RemotePromotionGateV1::EligibleOnlyAfterLocalQualification
        );
        assert!(floor.require_remote_transition_authorization());
        assert!(floor.require_common_ancestor_qualification());
        assert!(floor.require_local_policy_qualification());
        assert!(!floor.require_explicit_fork_resolution());
    }

    #[test]
    fn remote_recovery_extension_requires_independent_recovery_authorization() {
        let a = ancestor();
        let recovered = state(1, 8, 12, 41, 9, 8, 30, 0);
        let remote = [recovery_step(a, recovered, 80)];
        let assessment = reconcile_candidate_histories(a, None, a, Some(&remote), recovered)
            .expect("recovery assessment");
        let floor = derive_reconciliation_safety_floor(assessment);
        assert!(floor.require_recovery_authorization());
        assert_eq!(
            floor.remote_promotion_gate(),
            RemotePromotionGateV1::EligibleOnlyAfterLocalQualification
        );
    }

    #[test]
    fn fork_is_blocked_pending_explicit_resolution_even_if_remote_epoch_is_higher() {
        let a = ancestor();
        let local_tip = normal_e8();
        let remote_e8 = state(1, 8, 12, 41, 9, 3, 30, 0);
        let remote_e9 = state(1, 9, 13, 42, 9, 3, 40, 0);
        let local = [normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            a,
            local_tip,
            70,
        )];
        let remote = [
            normal_step(
                EpochTransitionKind::AuthorityRootRotation,
                a,
                remote_e8,
                80,
            ),
            normal_step(
                EpochTransitionKind::ScheduledEpochAdvance,
                remote_e8,
                remote_e9,
                81,
            ),
        ];
        let assessment = reconcile_candidate_histories(
            a,
            Some(&local),
            local_tip,
            Some(&remote),
            remote_e9,
        )
        .expect("fork assessment");
        let floor = derive_reconciliation_safety_floor(assessment);
        assert_eq!(
            floor.remote_promotion_gate(),
            RemotePromotionGateV1::BlockedPendingForkResolution
        );
        assert!(floor.require_explicit_fork_resolution());
        assert!(floor.automatic_remote_adoption_forbidden());
    }

    #[test]
    fn unproven_gap_blocks_promotion_pending_causal_proof() {
        let a = ancestor();
        let remote_tip = normal_e8();
        let assessment = reconcile_candidate_histories(a, None, a, None, remote_tip)
            .expect("gap assessment");
        let floor = derive_reconciliation_safety_floor(assessment);
        assert_eq!(
            floor.remote_promotion_gate(),
            RemotePromotionGateV1::BlockedPendingCausalProof
        );
        assert!(floor.require_complete_causal_path());
        assert!(floor.require_remote_transition_authorization());
        assert!(floor.require_recovery_authorization());
    }

    #[test]
    fn different_federation_direct_promotion_is_structurally_forbidden() {
        let a = ancestor();
        let foreign = state(7, 8, 12, 41, 2, 3, 20, 0);
        let assessment = reconcile_candidate_histories(a, None, a, None, foreign)
            .expect("foreign assessment");
        let floor = derive_reconciliation_safety_floor(assessment);
        assert_eq!(
            floor.remote_promotion_gate(),
            RemotePromotionGateV1::ForbiddenCrossFederation
        );
        assert!(floor.remote_state_is_evidence_only());
        assert!(!floor.require_local_policy_qualification());
    }

    #[test]
    fn local_strict_extension_needs_no_remote_promotion() {
        let a = ancestor();
        let e8 = normal_e8();
        let local = [normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            a,
            e8,
            70,
        )];
        let assessment = reconcile_candidate_histories(a, Some(&local), e8, None, a)
            .expect("local extension assessment");
        let floor = derive_reconciliation_safety_floor(assessment);
        assert_eq!(
            floor.remote_promotion_gate(),
            RemotePromotionGateV1::NoPromotionNeeded
        );
        assert!(!floor.require_remote_transition_authorization());
        assert!(!floor.require_explicit_fork_resolution());
        assert!(floor.automatic_remote_adoption_forbidden());
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Causal long-partition reconciliation for the Mycelix Solar-System
//! Federation security profile.
//!
//! This crate classifies evidence about divergent federation histories. It
//! deliberately does **not** select a winner, install a remote state, mint
//! authority, read clocks, compare timestamps, query a DHT, or perform
//! networking. A higher epoch number is not evidence that one history should
//! replace another.
//!
//! The central rule is:
//!
//! **Reconnection may establish causal relationships between histories; it
//! never silently converts remote history into local authority.**

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::EvidenceCommitment;
use mycelix_ssf_recovery::ShapeValidatedRecoveryTransitionV1;
use mycelix_ssf_snapshots::FederationStateHeadV1;
use mycelix_ssf_transitions::{EpochTransitionKind, ShapeValidatedEpochTransitionV1};

macro_rules! commitment_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            /// Return canonical commitment bytes.
            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

/// Common commitment representation for one candidate history step.
commitment_type!(HistoryStepCommitment);
/// Commitment to the authorization evidence carried by one candidate step.
commitment_type!(HistoryAuthorizationCommitment);
/// Optional out-of-band authority domain commitment, used for independently
/// pinned recovery authority that is not represented by the primary source
/// federation state itself.
commitment_type!(HistoryAuthorityDomainCommitment);

/// Constitutional meaning of one candidate history step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoryStepKindV1 {
    ScheduledEpochAdvance,
    AuthorityRootRotation,
    ConstitutionalUpgrade,
    CompromiseRecovery,
}

/// Candidate causal step derived only from exact shape-validated transition
/// evidence.
///
/// This retains every security-relevant field needed to distinguish two
/// structurally different transition claims even when they reach the same
/// endpoint state or reuse the same transition commitment.
///
/// It is still not proof that the transition was cryptographically or
/// quorum-authorized. Reconciliation classifies the supplied evidence; it does
/// not promote it to authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CandidateHistoryStepV1 {
    source: FederationStateHeadV1,
    target: FederationStateHeadV1,
    kind: HistoryStepKindV1,
    transition_commitment: HistoryStepCommitment,
    evidence_root: EvidenceCommitment,
    authorization_commitment: HistoryAuthorizationCommitment,
    authority_domain_commitment: Option<HistoryAuthorityDomainCommitment>,
}

impl CandidateHistoryStepV1 {
    /// Exact source state head retained by the candidate transition evidence.
    pub const fn source(&self) -> FederationStateHeadV1 {
        self.source
    }

    /// Exact target state head retained by the candidate transition evidence.
    pub const fn target(&self) -> FederationStateHeadV1 {
        self.target
    }

    /// Constitutional transition kind.
    pub const fn kind(&self) -> HistoryStepKindV1 {
        self.kind
    }

    /// Exact transition commitment retained by the candidate evidence.
    pub const fn transition_commitment(&self) -> HistoryStepCommitment {
        self.transition_commitment
    }

    /// Exact evidence root retained by the checked claim.
    pub const fn evidence_root(&self) -> EvidenceCommitment {
        self.evidence_root
    }

    /// Exact authorization-evidence commitment retained by the checked claim.
    pub const fn authorization_commitment(&self) -> HistoryAuthorizationCommitment {
        self.authorization_commitment
    }

    /// Independent authority-domain commitment when the transition relies on
    /// authority not represented by the primary source federation context.
    pub const fn authority_domain_commitment(&self) -> Option<HistoryAuthorityDomainCommitment> {
        self.authority_domain_commitment
    }
}

/// Convert an exact shape-validated normal epoch transition into candidate
/// reconciliation evidence.
pub fn candidate_step_from_epoch_transition(
    transition: &ShapeValidatedEpochTransitionV1,
) -> CandidateHistoryStepV1 {
    let certificate = transition.certificate();
    let kind = match certificate.kind {
        EpochTransitionKind::ScheduledEpochAdvance => HistoryStepKindV1::ScheduledEpochAdvance,
        EpochTransitionKind::AuthorityRootRotation => HistoryStepKindV1::AuthorityRootRotation,
        EpochTransitionKind::ConstitutionalUpgrade => HistoryStepKindV1::ConstitutionalUpgrade,
    };

    CandidateHistoryStepV1 {
        source: transition.source(),
        target: transition.target(),
        kind,
        transition_commitment: HistoryStepCommitment::from_bytes(
            *transition.transition_commitment().as_bytes(),
        ),
        evidence_root: certificate.evidence_root,
        authorization_commitment: HistoryAuthorizationCommitment::from_bytes(
            *certificate.authorization_commitment.as_bytes(),
        ),
        authority_domain_commitment: None,
    }
}

/// Convert an exact shape-validated compromise-recovery transition into
/// candidate reconciliation evidence.
///
/// Recovery schema semantics have already been checked by the parent recovery
/// typestate before this function can receive the token. The independently
/// expected recovery constitution is retained explicitly because it is not
/// represented by the primary source federation context.
pub fn candidate_step_from_recovery_transition(
    transition: &ShapeValidatedRecoveryTransitionV1,
) -> CandidateHistoryStepV1 {
    let claim = transition.claim();
    CandidateHistoryStepV1 {
        source: claim.source,
        target: claim.target,
        kind: HistoryStepKindV1::CompromiseRecovery,
        transition_commitment: HistoryStepCommitment::from_bytes(
            *claim.transition_commitment.as_bytes(),
        ),
        evidence_root: claim.evidence_root,
        authorization_commitment: HistoryAuthorizationCommitment::from_bytes(
            *claim.authorization_commitment.as_bytes(),
        ),
        authority_domain_commitment: Some(HistoryAuthorityDomainCommitment::from_bytes(
            *transition
                .expected_recovery_constitution()
                .commitment()
                .as_bytes(),
        )),
    }
}

/// Causal relationship established by the supplied candidate path witnesses.
///
/// None of these variants means "install the remote state".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CausalRelationV1 {
    /// Both sides provide the exact same candidate transition history.
    SameHistory,
    /// The remote candidate path is an exact prefix of the local path.
    LocalStrictExtension,
    /// The local candidate path is an exact prefix of the remote path.
    RemoteStrictExtension,
    /// Complete candidate paths diverge after the claimed common ancestor.
    Fork,
    /// Different candidate transition histories converge on the same exact
    /// state head. The convergence does not erase the causal fork.
    ConvergentFork,
    /// At least one non-trivial path witness is absent, so causal ordering is
    /// not established from the evidence supplied here.
    UnprovenGap,
    /// The claimed histories do not belong to one federation identity.
    DifferentFederation,
}

/// Whether the complete supplied candidate histories cross a compromise
/// recovery boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RecoveryPresenceV1 {
    None,
    LocalOnly,
    RemoteOnly,
    Both,
    /// One or both non-trivial path witnesses are absent.
    Unknown,
}

/// Completeness of the candidate path evidence used for the assessment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PathEvidenceStatusV1 {
    CompleteBoth,
    MissingLocal,
    MissingRemote,
    MissingBoth,
}

/// Opaque reconciliation assessment. It can only be constructed by
/// `reconcile_candidate_histories`.
///
/// This object is evidence classification, not authority and not a merge
/// decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CandidateReconciliationAssessmentV1 {
    common_ancestor: FederationStateHeadV1,
    local_tip: FederationStateHeadV1,
    remote_tip: FederationStateHeadV1,
    relation: CausalRelationV1,
    recovery_presence: RecoveryPresenceV1,
    path_evidence: PathEvidenceStatusV1,
    local_step_count: u32,
    remote_step_count: u32,
}

impl CandidateReconciliationAssessmentV1 {
    pub const fn common_ancestor(&self) -> FederationStateHeadV1 {
        self.common_ancestor
    }

    pub const fn local_tip(&self) -> FederationStateHeadV1 {
        self.local_tip
    }

    pub const fn remote_tip(&self) -> FederationStateHeadV1 {
        self.remote_tip
    }

    pub const fn relation(&self) -> CausalRelationV1 {
        self.relation
    }

    pub const fn recovery_presence(&self) -> RecoveryPresenceV1 {
        self.recovery_presence
    }

    pub const fn path_evidence(&self) -> PathEvidenceStatusV1 {
        self.path_evidence
    }

    pub const fn local_step_count(&self) -> u32 {
        self.local_step_count
    }

    pub const fn remote_step_count(&self) -> u32 {
        self.remote_step_count
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PathSideV1 {
    Local,
    Remote,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReconciliationError {
    PathTooLong { side: PathSideV1 },
    StepFederationMismatch { side: PathSideV1, index: u32 },
    PathDiscontinuity { side: PathSideV1, index: u32 },
    DeclaredTipMismatch { side: PathSideV1 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PathFacts {
    step_count: u32,
    has_recovery: bool,
}

fn validate_complete_path(
    ancestor: FederationStateHeadV1,
    steps: &[CandidateHistoryStepV1],
    declared_tip: FederationStateHeadV1,
    side: PathSideV1,
) -> Result<PathFacts, ReconciliationError> {
    if steps.len() > u32::MAX as usize {
        return Err(ReconciliationError::PathTooLong { side });
    }

    let federation_id = ancestor.context().federation_id;
    let mut cursor = ancestor;
    let mut has_recovery = false;

    for (index, step) in steps.iter().enumerate() {
        let index = index as u32;
        if step.source.context().federation_id != federation_id
            || step.target.context().federation_id != federation_id
        {
            return Err(ReconciliationError::StepFederationMismatch { side, index });
        }
        if step.source != cursor {
            return Err(ReconciliationError::PathDiscontinuity { side, index });
        }
        if step.kind == HistoryStepKindV1::CompromiseRecovery {
            has_recovery = true;
        }
        cursor = step.target;
    }

    if cursor != declared_tip {
        return Err(ReconciliationError::DeclaredTipMismatch { side });
    }

    Ok(PathFacts {
        step_count: steps.len() as u32,
        has_recovery,
    })
}

fn is_exact_prefix(shorter: &[CandidateHistoryStepV1], longer: &[CandidateHistoryStepV1]) -> bool {
    shorter.len() < longer.len()
        && shorter
            .iter()
            .zip(longer.iter())
            .all(|(left, right)| left == right)
}

fn recovery_presence(local: PathFacts, remote: PathFacts) -> RecoveryPresenceV1 {
    match (local.has_recovery, remote.has_recovery) {
        (false, false) => RecoveryPresenceV1::None,
        (true, false) => RecoveryPresenceV1::LocalOnly,
        (false, true) => RecoveryPresenceV1::RemoteOnly,
        (true, true) => RecoveryPresenceV1::Both,
    }
}

/// Classify two candidate histories relative to one claimed common ancestor.
///
/// `None` means that a non-trivial path witness was not supplied. A `None`
/// path is still considered complete when its declared tip equals the common
/// ancestor, because the empty path is then proven directly by equality.
///
/// Complete paths are checked for exact continuity. Causal extension requires
/// an exact transition-step prefix; epoch numbers, authority-generation
/// numbers, timestamps, record counts, and wall-clock recency are never used as
/// substitutes for ancestry.
pub fn reconcile_candidate_histories(
    common_ancestor: FederationStateHeadV1,
    local_path: Option<&[CandidateHistoryStepV1]>,
    local_tip: FederationStateHeadV1,
    remote_path: Option<&[CandidateHistoryStepV1]>,
    remote_tip: FederationStateHeadV1,
) -> Result<CandidateReconciliationAssessmentV1, ReconciliationError> {
    let federation_id = common_ancestor.context().federation_id;
    if local_tip.context().federation_id != federation_id
        || remote_tip.context().federation_id != federation_id
    {
        return Ok(CandidateReconciliationAssessmentV1 {
            common_ancestor,
            local_tip,
            remote_tip,
            relation: CausalRelationV1::DifferentFederation,
            recovery_presence: RecoveryPresenceV1::Unknown,
            path_evidence: PathEvidenceStatusV1::MissingBoth,
            local_step_count: 0,
            remote_step_count: 0,
        });
    }

    let empty: &[CandidateHistoryStepV1] = &[];
    let local_steps = match local_path {
        Some(path) => Some(path),
        None if local_tip == common_ancestor => Some(empty),
        None => None,
    };
    let remote_steps = match remote_path {
        Some(path) => Some(path),
        None if remote_tip == common_ancestor => Some(empty),
        None => None,
    };

    let path_evidence = match (local_steps.is_some(), remote_steps.is_some()) {
        (true, true) => PathEvidenceStatusV1::CompleteBoth,
        (false, true) => PathEvidenceStatusV1::MissingLocal,
        (true, false) => PathEvidenceStatusV1::MissingRemote,
        (false, false) => PathEvidenceStatusV1::MissingBoth,
    };

    let (Some(local_steps), Some(remote_steps)) = (local_steps, remote_steps) else {
        return Ok(CandidateReconciliationAssessmentV1 {
            common_ancestor,
            local_tip,
            remote_tip,
            relation: CausalRelationV1::UnprovenGap,
            recovery_presence: RecoveryPresenceV1::Unknown,
            path_evidence,
            local_step_count: 0,
            remote_step_count: 0,
        });
    };

    let local_facts = validate_complete_path(
        common_ancestor,
        local_steps,
        local_tip,
        PathSideV1::Local,
    )?;
    let remote_facts = validate_complete_path(
        common_ancestor,
        remote_steps,
        remote_tip,
        PathSideV1::Remote,
    )?;

    let relation = if local_steps == remote_steps {
        CausalRelationV1::SameHistory
    } else if is_exact_prefix(remote_steps, local_steps) {
        CausalRelationV1::LocalStrictExtension
    } else if is_exact_prefix(local_steps, remote_steps) {
        CausalRelationV1::RemoteStrictExtension
    } else if local_tip == remote_tip {
        CausalRelationV1::ConvergentFork
    } else {
        CausalRelationV1::Fork
    };

    Ok(CandidateReconciliationAssessmentV1 {
        common_ancestor,
        local_tip,
        remote_tip,
        relation,
        recovery_presence: recovery_presence(local_facts, remote_facts),
        path_evidence,
        local_step_count: local_facts.step_count,
        remote_step_count: remote_facts.step_count,
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
        RecoveryAuthorizationCommitment, RecoveryConstitutionCommitment, RecoveryTargetPlanV1,
        RecoveryTransitionClaimV1, RecoveryTransitionCommitment,
    };
    use mycelix_ssf_snapshots::{
        assemble_federation_state_head, MembershipSnapshotV1, PolicySnapshotV1,
        RevocationSnapshotV1, SnapshotCommitment, SnapshotGeneration, SnapshotLineageV1,
    };
    use mycelix_ssf_transitions::{
        shape_validate_epoch_transition, EpochTransitionCertificateV1,
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

    fn normal_step_with_material(
        kind: EpochTransitionKind,
        source: FederationStateHeadV1,
        target: FederationStateHeadV1,
        transition_byte: u8,
        evidence_byte: u8,
        authorization_byte: u8,
    ) -> CandidateHistoryStepV1 {
        let certificate = EpochTransitionCertificateV1::new(
            kind,
            source,
            target,
            EvidenceCommitment::from_bytes(bytes(evidence_byte)),
            TransitionAuthorizationCommitment::from_bytes(bytes(authorization_byte)),
            TransitionCommitment::from_bytes(bytes(transition_byte)),
        );
        let validated =
            shape_validate_epoch_transition(certificate).expect("valid normal transition fixture");
        candidate_step_from_epoch_transition(&validated)
    }

    fn normal_step(
        kind: EpochTransitionKind,
        source: FederationStateHeadV1,
        target: FederationStateHeadV1,
        commitment_byte: u8,
    ) -> CandidateHistoryStepV1 {
        normal_step_with_material(
            kind,
            source,
            target,
            commitment_byte,
            commitment_byte.wrapping_add(20),
            commitment_byte.wrapping_add(40),
        )
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
        let validated = validate_recovery_transition_shape(expected, claim)
            .expect("valid recovery transition fixture");
        candidate_step_from_recovery_transition(&validated)
    }

    fn base_state() -> FederationStateHeadV1 {
        state(1, 7, 11, 40, 2, 3, 10, 5)
    }

    fn epoch8_normal() -> FederationStateHeadV1 {
        state(1, 8, 12, 41, 2, 3, 20, 0)
    }

    fn epoch9_normal() -> FederationStateHeadV1 {
        state(1, 9, 13, 42, 2, 3, 30, 0)
    }

    #[test]
    fn identical_complete_paths_are_same_history() {
        let ancestor = base_state();
        let e8 = epoch8_normal();
        let step = normal_step(EpochTransitionKind::ScheduledEpochAdvance, ancestor, e8, 70);
        let path = [step];

        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&path),
            e8,
            Some(&path),
            e8,
        )
        .expect("reconciliation");

        assert_eq!(assessment.relation(), CausalRelationV1::SameHistory);
        assert_eq!(assessment.recovery_presence(), RecoveryPresenceV1::None);
        assert_eq!(assessment.path_evidence(), PathEvidenceStatusV1::CompleteBoth);
    }

    #[test]
    fn exact_prefix_proves_remote_strict_extension() {
        let ancestor = base_state();
        let e8 = epoch8_normal();
        let e9 = epoch9_normal();
        let first = normal_step(EpochTransitionKind::ScheduledEpochAdvance, ancestor, e8, 70);
        let second = normal_step(EpochTransitionKind::ScheduledEpochAdvance, e8, e9, 71);
        let local = [first];
        let remote = [first, second];

        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&local),
            e8,
            Some(&remote),
            e9,
        )
        .expect("reconciliation");

        assert_eq!(
            assessment.relation(),
            CausalRelationV1::RemoteStrictExtension
        );
        assert_eq!(assessment.local_step_count(), 1);
        assert_eq!(assessment.remote_step_count(), 2);
    }

    #[test]
    fn exact_prefix_proves_local_strict_extension() {
        let ancestor = base_state();
        let e8 = epoch8_normal();
        let e9 = epoch9_normal();
        let first = normal_step(EpochTransitionKind::ScheduledEpochAdvance, ancestor, e8, 70);
        let second = normal_step(EpochTransitionKind::ScheduledEpochAdvance, e8, e9, 71);
        let local = [first, second];
        let remote = [first];

        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&local),
            e9,
            Some(&remote),
            e8,
        )
        .expect("reconciliation");

        assert_eq!(
            assessment.relation(),
            CausalRelationV1::LocalStrictExtension
        );
    }

    #[test]
    fn divergent_complete_paths_are_a_fork() {
        let ancestor = base_state();
        let local_tip = epoch8_normal();
        let remote_tip = state(1, 8, 12, 41, 9, 3, 40, 0);
        let local_step = normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            ancestor,
            local_tip,
            70,
        );
        let remote_step = normal_step(
            EpochTransitionKind::AuthorityRootRotation,
            ancestor,
            remote_tip,
            80,
        );
        let local = [local_step];
        let remote = [remote_step];

        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&local),
            local_tip,
            Some(&remote),
            remote_tip,
        )
        .expect("reconciliation");

        assert_eq!(assessment.relation(), CausalRelationV1::Fork);
    }

    #[test]
    fn higher_remote_epoch_does_not_win_over_a_divergent_branch() {
        let ancestor = base_state();
        let local_tip = epoch8_normal();
        let remote_e8 = state(1, 8, 12, 41, 9, 3, 40, 0);
        let remote_e9 = state(1, 9, 13, 42, 9, 3, 50, 0);

        let local_step = normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            ancestor,
            local_tip,
            70,
        );
        let remote_first = normal_step(
            EpochTransitionKind::AuthorityRootRotation,
            ancestor,
            remote_e8,
            80,
        );
        let remote_second = normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            remote_e8,
            remote_e9,
            81,
        );
        let local = [local_step];
        let remote = [remote_first, remote_second];

        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&local),
            local_tip,
            Some(&remote),
            remote_e9,
        )
        .expect("reconciliation");

        assert_eq!(assessment.relation(), CausalRelationV1::Fork);
        assert!(remote_e9.context().federation_epoch > local_tip.context().federation_epoch);
    }

    #[test]
    fn different_transition_histories_that_reach_same_tip_remain_a_convergent_fork() {
        let ancestor = base_state();
        let tip = epoch8_normal();
        let local_step = normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            ancestor,
            tip,
            70,
        );
        let remote_step = normal_step(
            EpochTransitionKind::ScheduledEpochAdvance,
            ancestor,
            tip,
            71,
        );
        let local = [local_step];
        let remote = [remote_step];

        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&local),
            tip,
            Some(&remote),
            tip,
        )
        .expect("reconciliation");

        assert_eq!(assessment.relation(), CausalRelationV1::ConvergentFork);
    }

    #[test]
    fn same_transition_commitment_does_not_collapse_distinct_claim_material() {
        let ancestor = base_state();
        let tip = epoch8_normal();
        let local_step = normal_step_with_material(
            EpochTransitionKind::ScheduledEpochAdvance,
            ancestor,
            tip,
            70,
            91,
            92,
        );
        let remote_step = normal_step_with_material(
            EpochTransitionKind::ScheduledEpochAdvance,
            ancestor,
            tip,
            70,
            93,
            94,
        );
        assert_ne!(local_step, remote_step);

        let local = [local_step];
        let remote = [remote_step];
        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&local),
            tip,
            Some(&remote),
            tip,
        )
        .expect("reconciliation");

        assert_eq!(assessment.relation(), CausalRelationV1::ConvergentFork);
    }

    #[test]
    fn absent_nontrivial_path_is_an_unproven_gap() {
        let ancestor = base_state();
        let remote_tip = epoch8_normal();
        let assessment = reconcile_candidate_histories(
            ancestor,
            None,
            ancestor,
            None,
            remote_tip,
        )
        .expect("reconciliation");

        assert_eq!(assessment.relation(), CausalRelationV1::UnprovenGap);
        assert_eq!(
            assessment.path_evidence(),
            PathEvidenceStatusV1::MissingRemote
        );
        assert_eq!(assessment.recovery_presence(), RecoveryPresenceV1::Unknown);
    }

    #[test]
    fn empty_path_is_complete_when_tip_equals_common_ancestor() {
        let ancestor = base_state();
        let assessment = reconcile_candidate_histories(
            ancestor,
            None,
            ancestor,
            None,
            ancestor,
        )
        .expect("reconciliation");

        assert_eq!(assessment.relation(), CausalRelationV1::SameHistory);
        assert_eq!(assessment.path_evidence(), PathEvidenceStatusV1::CompleteBoth);
    }

    #[test]
    fn federation_identity_mismatch_is_not_a_merge_candidate() {
        let ancestor = base_state();
        let foreign = state(7, 8, 12, 41, 2, 3, 20, 0);
        let assessment = reconcile_candidate_histories(
            ancestor,
            None,
            ancestor,
            None,
            foreign,
        )
        .expect("reconciliation");

        assert_eq!(
            assessment.relation(),
            CausalRelationV1::DifferentFederation
        );
    }

    #[test]
    fn recovery_boundary_is_preserved_in_reconciliation_evidence() {
        let ancestor = base_state();
        let recovered = state(1, 8, 12, 41, 9, 8, 60, 0);
        let step = recovery_step(ancestor, recovered, 90);
        assert!(step.authority_domain_commitment().is_some());
        let local: [CandidateHistoryStepV1; 0] = [];
        let remote = [step];

        let assessment = reconcile_candidate_histories(
            ancestor,
            Some(&local),
            ancestor,
            Some(&remote),
            recovered,
        )
        .expect("reconciliation");

        assert_eq!(
            assessment.relation(),
            CausalRelationV1::RemoteStrictExtension
        );
        assert_eq!(
            assessment.recovery_presence(),
            RecoveryPresenceV1::RemoteOnly
        );
    }

    #[test]
    fn discontinuous_path_is_rejected_instead_of_sorted_by_epoch() {
        let ancestor = base_state();
        let e8 = epoch8_normal();
        let e9 = epoch9_normal();
        let second = normal_step(EpochTransitionKind::ScheduledEpochAdvance, e8, e9, 71);
        let broken = [second];

        assert_eq!(
            reconcile_candidate_histories(
                ancestor,
                Some(&broken),
                e9,
                None,
                ancestor,
            ),
            Err(ReconciliationError::PathDiscontinuity {
                side: PathSideV1::Local,
                index: 0,
            })
        );
    }

    #[test]
    fn declared_tip_must_equal_exact_path_target() {
        let ancestor = base_state();
        let e8 = epoch8_normal();
        let e9 = epoch9_normal();
        let first = normal_step(EpochTransitionKind::ScheduledEpochAdvance, ancestor, e8, 70);
        let path = [first];

        assert_eq!(
            reconcile_candidate_histories(
                ancestor,
                Some(&path),
                e9,
                None,
                ancestor,
            ),
            Err(ReconciliationError::DeclaredTipMismatch {
                side: PathSideV1::Local,
            })
        );
    }
}

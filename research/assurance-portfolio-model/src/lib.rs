#![forbid(unsafe_code)]
//! Finite-state model for composing claim-specific assurance assessments.
//!
//! This crate is research-only. A successful composition means only that the
//! supplied assessments meet one declared evidence profile for one exact
//! claim/subject pair. It does not establish truth, authorize actions, or
//! qualify the assessed system for production use.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Digest(pub [u8; 32]);

impl Digest {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum EvaluatorRole {
    NormativeContract,
    IndependentImplementation,
    FormalOrBoundedChecker,
    InvariantChecker,
    MetamorphicOrMutationChecker,
    EmpiricalEvidenceAppraiser,
    DomainReviewer,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum AssessmentOutcome {
    Supports,
    Refutes,
    Indeterminate,
    OutOfScope,
    Unavailable,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum EvidenceValidity {
    Current,
    Stale,
    Revoked,
    Invalid,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum FaultDomainDimension {
    ImplementationLineage,
    RepositoryControl,
    ExecutionProvider,
    RunnerImage,
    Runtime,
    InputAcquisition,
    OperatorControl,
    TrustRoot,
}

impl FaultDomainDimension {
    pub const ALL: [Self; 8] = [
        Self::ImplementationLineage,
        Self::RepositoryControl,
        Self::ExecutionProvider,
        Self::RunnerImage,
        Self::Runtime,
        Self::InputAcquisition,
        Self::OperatorControl,
        Self::TrustRoot,
    ];
}

#[derive(Clone, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct FaultDomains {
    pub implementation_lineage: Option<Digest>,
    pub repository_control: Option<Digest>,
    pub execution_provider: Option<Digest>,
    pub runner_image: Option<Digest>,
    pub runtime: Option<Digest>,
    pub input_acquisition: Option<Digest>,
    pub operator_control: Option<Digest>,
    pub trust_root: Option<Digest>,
}

impl FaultDomains {
    pub fn identity(&self, dimension: FaultDomainDimension) -> Option<Digest> {
        match dimension {
            FaultDomainDimension::ImplementationLineage => self.implementation_lineage,
            FaultDomainDimension::RepositoryControl => self.repository_control,
            FaultDomainDimension::ExecutionProvider => self.execution_provider,
            FaultDomainDimension::RunnerImage => self.runner_image,
            FaultDomainDimension::Runtime => self.runtime,
            FaultDomainDimension::InputAcquisition => self.input_acquisition,
            FaultDomainDimension::OperatorControl => self.operator_control,
            FaultDomainDimension::TrustRoot => self.trust_root,
        }
    }
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Assessment {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub role: EvaluatorRole,
    pub evaluator_digest: Digest,
    pub evaluator_profile_digest: Digest,
    pub outcome: AssessmentOutcome,
    pub validity: EvidenceValidity,
    pub fault_domains: FaultDomains,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PairwiseSeparation {
    pub left_role: EvaluatorRole,
    pub right_role: EvaluatorRole,
    pub dimension: FaultDomainDimension,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ClaimProfile {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub hard_invariant: bool,
    /// Roles explicitly permitted to make a dispositive hard-invariant refutation.
    /// A refutation from any other role remains a non-hard conflict.
    pub hard_refutation_roles: Vec<EvaluatorRole>,
    pub required_roles: Vec<EvaluatorRole>,
    pub required_separations: Vec<PairwiseSeparation>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompositionError {
    EmptyClaimId,
    EmptyRequiredRoles,
    DuplicateRequiredRole,
    HardInvariantRequiresRefutationRole,
    RefutationRoleWithoutHardInvariant,
    DuplicateHardRefutationRole,
    SelfSeparation,
    SeparationRoleNotRequired,
    DuplicateSeparation,
}

impl fmt::Display for CompositionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for CompositionError {}

impl ClaimProfile {
    pub fn validate(&self) -> Result<(), CompositionError> {
        if self.claim_id.trim().is_empty() {
            return Err(CompositionError::EmptyClaimId);
        }
        if self.required_roles.is_empty() {
            return Err(CompositionError::EmptyRequiredRoles);
        }

        let mut roles = BTreeSet::new();
        for role in &self.required_roles {
            if !roles.insert(*role) {
                return Err(CompositionError::DuplicateRequiredRole);
            }
        }

        if self.hard_invariant && self.hard_refutation_roles.is_empty() {
            return Err(CompositionError::HardInvariantRequiresRefutationRole);
        }
        if !self.hard_invariant && !self.hard_refutation_roles.is_empty() {
            return Err(CompositionError::RefutationRoleWithoutHardInvariant);
        }
        let mut hard_refutation_roles = BTreeSet::new();
        for role in &self.hard_refutation_roles {
            if !hard_refutation_roles.insert(*role) {
                return Err(CompositionError::DuplicateHardRefutationRole);
            }
        }

        let mut separations = BTreeSet::new();
        for separation in &self.required_separations {
            if separation.left_role == separation.right_role {
                return Err(CompositionError::SelfSeparation);
            }
            if !roles.contains(&separation.left_role) || !roles.contains(&separation.right_role) {
                return Err(CompositionError::SeparationRoleNotRequired);
            }
            if !separations.insert(*separation) {
                return Err(CompositionError::DuplicateSeparation);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompositionDisposition {
    MeetsDeclaredProfile,
    HardInvariantRefuted,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CompositionReason {
    HardInvariantRefuted,
    NonHardRefutationObserved,
    NonSupportingOutcomeObserved,
    RequiredRoleMissing,
    RequiredRoleNotSupported,
    ContradictoryEvaluatorOutput,
    EvaluatorFaultDomainDrift,
    RequiredFaultDomainUnknown,
    RequiredFaultDomainShared,
    DuplicateReplaySuppressed,
    NonCurrentEvidenceObserved,
    WrongSubjectEvidenceIgnored,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaultDomainReport {
    pub dimension: FaultDomainDimension,
    pub known_identity_count: usize,
    pub unknown_observation_count: usize,
    /// True means all known observations collapsed to one identity.
    /// It is descriptive; it is not a global independence verdict.
    pub single_known_identity_observed: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompositionReport {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub disposition: CompositionDisposition,
    pub unique_assessment_count: usize,
    pub current_assessment_count: usize,
    pub duplicate_replay_count: usize,
    pub wrong_subject_assessment_count: usize,
    pub non_current_assessment_count: usize,
    pub fault_domains: Vec<FaultDomainReport>,
    pub reasons: Vec<CompositionReason>,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct EvaluatorKey {
    role: EvaluatorRole,
    evaluator_digest: Digest,
    evaluator_profile_digest: Digest,
}

#[derive(Default)]
struct EvaluatorObservation {
    outcomes: BTreeSet<AssessmentOutcome>,
    fault_domains: BTreeSet<FaultDomains>,
}

pub fn compose(
    profile: &ClaimProfile,
    assessments: &[Assessment],
) -> Result<CompositionReport, CompositionError> {
    profile.validate()?;

    let claim_assessments: Vec<&Assessment> = assessments
        .iter()
        .filter(|assessment| assessment.claim_id == profile.claim_id)
        .collect();

    let wrong_subject_assessment_count = claim_assessments
        .iter()
        .filter(|assessment| assessment.subject_digest != profile.subject_digest)
        .count();

    let exact_assessments: BTreeSet<&Assessment> = claim_assessments
        .into_iter()
        .filter(|assessment| assessment.subject_digest == profile.subject_digest)
        .collect();

    let unique_assessment_count = exact_assessments.len();
    let duplicate_replay_count = assessments
        .iter()
        .filter(|assessment| assessment.claim_id == profile.claim_id)
        .filter(|assessment| assessment.subject_digest == profile.subject_digest)
        .count()
        .saturating_sub(unique_assessment_count);

    let current: Vec<&Assessment> = exact_assessments
        .iter()
        .copied()
        .filter(|assessment| assessment.validity == EvidenceValidity::Current)
        .collect();

    let non_current_assessment_count = unique_assessment_count.saturating_sub(current.len());
    let fault_domains = summarize_fault_domains(&current);
    let mut reasons = BTreeSet::new();

    if duplicate_replay_count > 0 {
        reasons.insert(CompositionReason::DuplicateReplaySuppressed);
    }
    if non_current_assessment_count > 0 {
        reasons.insert(CompositionReason::NonCurrentEvidenceObserved);
    }
    if wrong_subject_assessment_count > 0 {
        reasons.insert(CompositionReason::WrongSubjectEvidenceIgnored);
    }

    let mut evaluator_observations = BTreeMap::<EvaluatorKey, EvaluatorObservation>::new();
    for assessment in &current {
        let key = EvaluatorKey {
            role: assessment.role,
            evaluator_digest: assessment.evaluator_digest,
            evaluator_profile_digest: assessment.evaluator_profile_digest,
        };
        let observation = evaluator_observations.entry(key).or_default();
        observation.outcomes.insert(assessment.outcome);
        observation
            .fault_domains
            .insert(assessment.fault_domains.clone());
    }

    let has_qualified_hard_refutation = profile.hard_invariant
        && current.iter().any(|assessment| {
            if assessment.outcome != AssessmentOutcome::Refutes
                || !profile.hard_refutation_roles.contains(&assessment.role)
            {
                return false;
            }

            let key = EvaluatorKey {
                role: assessment.role,
                evaluator_digest: assessment.evaluator_digest,
                evaluator_profile_digest: assessment.evaluator_profile_digest,
            };
            evaluator_observations.get(&key).is_some_and(|observation| {
                observation.outcomes.len() == 1
                    && observation.outcomes.contains(&AssessmentOutcome::Refutes)
                    && observation.fault_domains.len() == 1
            })
        });

    if has_qualified_hard_refutation {
        reasons.insert(CompositionReason::HardInvariantRefuted);
        return Ok(report(
            profile,
            CompositionDisposition::HardInvariantRefuted,
            AssessmentCounts {
                unique: unique_assessment_count,
                current: current.len(),
                duplicate_replays: duplicate_replay_count,
                wrong_subject: wrong_subject_assessment_count,
                non_current: non_current_assessment_count,
            },
            fault_domains,
            reasons,
        ));
    }

    if evaluator_observations
        .values()
        .any(|observation| observation.outcomes.len() > 1)
    {
        reasons.insert(CompositionReason::ContradictoryEvaluatorOutput);
    }
    if evaluator_observations
        .values()
        .any(|observation| observation.fault_domains.len() > 1)
    {
        reasons.insert(CompositionReason::EvaluatorFaultDomainDrift);
    }
    if current
        .iter()
        .any(|assessment| assessment.outcome == AssessmentOutcome::Refutes)
    {
        reasons.insert(CompositionReason::NonHardRefutationObserved);
    }
    if current.iter().any(|assessment| {
        matches!(
            assessment.outcome,
            AssessmentOutcome::Indeterminate
                | AssessmentOutcome::OutOfScope
                | AssessmentOutcome::Unavailable
        )
    }) {
        reasons.insert(CompositionReason::NonSupportingOutcomeObserved);
    }

    for role in &profile.required_roles {
        let role_assessments: Vec<&Assessment> = current
            .iter()
            .copied()
            .filter(|assessment| assessment.role == *role)
            .collect();

        if role_assessments.is_empty() {
            reasons.insert(CompositionReason::RequiredRoleMissing);
        } else if role_assessments
            .iter()
            .any(|assessment| assessment.outcome != AssessmentOutcome::Supports)
        {
            reasons.insert(CompositionReason::RequiredRoleNotSupported);
        }
    }

    for separation in &profile.required_separations {
        let left: Vec<&Assessment> = current
            .iter()
            .copied()
            .filter(|assessment| {
                assessment.role == separation.left_role
                    && assessment.outcome == AssessmentOutcome::Supports
            })
            .collect();
        let right: Vec<&Assessment> = current
            .iter()
            .copied()
            .filter(|assessment| {
                assessment.role == separation.right_role
                    && assessment.outcome == AssessmentOutcome::Supports
            })
            .collect();

        for left_assessment in &left {
            for right_assessment in &right {
                match (
                    left_assessment.fault_domains.identity(separation.dimension),
                    right_assessment
                        .fault_domains
                        .identity(separation.dimension),
                ) {
                    (Some(left_id), Some(right_id)) if left_id != right_id => {}
                    (Some(left_id), Some(right_id)) if left_id == right_id => {
                        reasons.insert(CompositionReason::RequiredFaultDomainShared);
                    }
                    _ => {
                        reasons.insert(CompositionReason::RequiredFaultDomainUnknown);
                    }
                }
            }
        }
    }

    let has_blocking_reason = reasons
        .iter()
        .any(|reason| *reason != CompositionReason::DuplicateReplaySuppressed);
    let disposition = if has_blocking_reason {
        CompositionDisposition::Indeterminate
    } else {
        CompositionDisposition::MeetsDeclaredProfile
    };

    Ok(report(
        profile,
        disposition,
        AssessmentCounts {
            unique: unique_assessment_count,
            current: current.len(),
            duplicate_replays: duplicate_replay_count,
            wrong_subject: wrong_subject_assessment_count,
            non_current: non_current_assessment_count,
        },
        fault_domains,
        reasons,
    ))
}

struct AssessmentCounts {
    unique: usize,
    current: usize,
    duplicate_replays: usize,
    wrong_subject: usize,
    non_current: usize,
}

fn report(
    profile: &ClaimProfile,
    disposition: CompositionDisposition,
    counts: AssessmentCounts,
    fault_domains: Vec<FaultDomainReport>,
    reasons: BTreeSet<CompositionReason>,
) -> CompositionReport {
    CompositionReport {
        claim_id: profile.claim_id.clone(),
        subject_digest: profile.subject_digest,
        disposition,
        unique_assessment_count: counts.unique,
        current_assessment_count: counts.current,
        duplicate_replay_count: counts.duplicate_replays,
        wrong_subject_assessment_count: counts.wrong_subject,
        non_current_assessment_count: counts.non_current,
        fault_domains,
        reasons: reasons.into_iter().collect(),
    }
}

fn summarize_fault_domains(current: &[&Assessment]) -> Vec<FaultDomainReport> {
    FaultDomainDimension::ALL
        .into_iter()
        .map(|dimension| {
            let mut identities = BTreeSet::new();
            let mut unknown_observation_count = 0;

            for assessment in current {
                match assessment.fault_domains.identity(dimension) {
                    Some(identity) => {
                        identities.insert(identity);
                    }
                    None => unknown_observation_count += 1,
                }
            }

            FaultDomainReport {
                dimension,
                known_identity_count: identities.len(),
                unknown_observation_count,
                single_known_identity_observed: current.len() > 1 && identities.len() == 1,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLAIM_ID: &str = "authorization.default-deny.v1";

    fn digest(value: u8) -> Digest {
        Digest::from_bytes([value; 32])
    }

    fn domains(implementation: u8) -> FaultDomains {
        FaultDomains {
            implementation_lineage: Some(digest(implementation)),
            repository_control: Some(digest(9)),
            execution_provider: Some(digest(7)),
            runner_image: Some(digest(8)),
            runtime: Some(digest(6)),
            input_acquisition: Some(digest(5)),
            operator_control: Some(digest(4)),
            trust_root: Some(digest(3)),
        }
    }

    fn profile(hard_invariant: bool) -> ClaimProfile {
        ClaimProfile {
            claim_id: CLAIM_ID.to_owned(),
            subject_digest: digest(42),
            hard_invariant,
            hard_refutation_roles: if hard_invariant {
                vec![EvaluatorRole::InvariantChecker]
            } else {
                vec![]
            },
            required_roles: vec![
                EvaluatorRole::NormativeContract,
                EvaluatorRole::IndependentImplementation,
            ],
            required_separations: vec![PairwiseSeparation {
                left_role: EvaluatorRole::NormativeContract,
                right_role: EvaluatorRole::IndependentImplementation,
                dimension: FaultDomainDimension::ImplementationLineage,
            }],
        }
    }

    fn assessment(role: EvaluatorRole, evaluator: u8, implementation: u8) -> Assessment {
        Assessment {
            claim_id: CLAIM_ID.to_owned(),
            subject_digest: digest(42),
            role,
            evaluator_digest: digest(evaluator),
            evaluator_profile_digest: digest(100 + evaluator),
            outcome: AssessmentOutcome::Supports,
            validity: EvidenceValidity::Current,
            fault_domains: domains(implementation),
        }
    }

    fn required_pair() -> Vec<Assessment> {
        vec![
            assessment(EvaluatorRole::NormativeContract, 11, 1),
            assessment(EvaluatorRole::IndependentImplementation, 12, 2),
        ]
    }

    fn has_reason(report: &CompositionReport, reason: CompositionReason) -> bool {
        report.reasons.contains(&reason)
    }

    #[test]
    fn independent_support_meets_only_the_declared_profile_and_reports_shared_provider() {
        let result = compose(&profile(false), &required_pair()).unwrap();
        assert_eq!(
            result.disposition,
            CompositionDisposition::MeetsDeclaredProfile
        );

        let provider = result
            .fault_domains
            .iter()
            .find(|entry| entry.dimension == FaultDomainDimension::ExecutionProvider)
            .unwrap();
        assert_eq!(provider.known_identity_count, 1);
        assert!(provider.single_known_identity_observed);
    }

    #[test]
    fn a_hard_invariant_refutation_cannot_be_outvoted_by_supporting_evaluators() {
        let mut assessments = required_pair();
        let mut refutation = assessment(EvaluatorRole::InvariantChecker, 13, 3);
        refutation.outcome = AssessmentOutcome::Refutes;
        assessments.push(refutation);

        let result = compose(&profile(true), &assessments).unwrap();
        assert_eq!(
            result.disposition,
            CompositionDisposition::HardInvariantRefuted
        );
        assert!(has_reason(&result, CompositionReason::HardInvariantRefuted));
    }

    #[test]
    fn hard_invariant_profiles_require_an_explicit_refutation_role() {
        let mut missing_role = profile(true);
        missing_role.hard_refutation_roles.clear();
        assert_eq!(
            missing_role.validate(),
            Err(CompositionError::HardInvariantRequiresRefutationRole)
        );

        let mut unexpected_role = profile(false);
        unexpected_role
            .hard_refutation_roles
            .push(EvaluatorRole::InvariantChecker);
        assert_eq!(
            unexpected_role.validate(),
            Err(CompositionError::RefutationRoleWithoutHardInvariant)
        );

        let mut duplicate_role = profile(true);
        duplicate_role
            .hard_refutation_roles
            .push(EvaluatorRole::InvariantChecker);
        assert_eq!(
            duplicate_role.validate(),
            Err(CompositionError::DuplicateHardRefutationRole)
        );
    }

    #[test]
    fn an_undeclared_role_cannot_refute_a_hard_invariant() {
        let mut assessments = required_pair();
        let mut unqualified_refutation = assessment(EvaluatorRole::DomainReviewer, 18, 8);
        unqualified_refutation.outcome = AssessmentOutcome::Refutes;
        assessments.push(unqualified_refutation);

        let result = compose(&profile(true), &assessments).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(!has_reason(
            &result,
            CompositionReason::HardInvariantRefuted
        ));
        assert!(has_reason(
            &result,
            CompositionReason::NonHardRefutationObserved
        ));
    }

    #[test]
    fn a_hard_refutation_from_a_self_contradictory_evaluator_is_not_dispositive() {
        let mut assessments = required_pair();
        let refutation = {
            let mut item = assessment(EvaluatorRole::InvariantChecker, 19, 9);
            item.outcome = AssessmentOutcome::Refutes;
            item
        };
        let mut contradictory = refutation.clone();
        contradictory.outcome = AssessmentOutcome::Supports;
        assessments.push(refutation);
        assessments.push(contradictory);

        let result = compose(&profile(true), &assessments).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(!has_reason(
            &result,
            CompositionReason::HardInvariantRefuted
        ));
        assert!(has_reason(
            &result,
            CompositionReason::ContradictoryEvaluatorOutput
        ));
    }

    #[test]
    fn exact_replay_does_not_increase_the_assessment_count() {
        let pair = required_pair();
        let mut replayed = pair.clone();
        replayed.extend(pair.clone());

        let baseline = compose(&profile(false), &pair).unwrap();
        let replay = compose(&profile(false), &replayed).unwrap();

        assert_eq!(baseline.disposition, replay.disposition);
        assert_eq!(replay.unique_assessment_count, 2);
        assert_eq!(replay.duplicate_replay_count, 2);
        assert!(has_reason(
            &replay,
            CompositionReason::DuplicateReplaySuppressed
        ));
    }

    #[test]
    fn different_executable_digests_do_not_hide_shared_implementation_lineage() {
        let mut pair = required_pair();
        pair[1].fault_domains.implementation_lineage = pair[0].fault_domains.implementation_lineage;

        let result = compose(&profile(false), &pair).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(has_reason(
            &result,
            CompositionReason::RequiredFaultDomainShared
        ));
    }

    #[test]
    fn unknown_required_fault_domain_is_indeterminate_not_independent() {
        let mut pair = required_pair();
        pair[1].fault_domains.implementation_lineage = None;

        let result = compose(&profile(false), &pair).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(has_reason(
            &result,
            CompositionReason::RequiredFaultDomainUnknown
        ));
    }

    #[test]
    fn missing_required_role_cannot_be_filled_by_extra_supporting_runs() {
        let mut pair = required_pair();
        pair.retain(|assessment| assessment.role == EvaluatorRole::NormativeContract);
        pair.push(assessment(EvaluatorRole::InvariantChecker, 14, 4));

        let result = compose(&profile(false), &pair).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(has_reason(&result, CompositionReason::RequiredRoleMissing));
    }

    #[test]
    fn stale_or_revoked_evidence_never_satisfies_a_required_role() {
        let mut pair = required_pair();
        pair[1].validity = EvidenceValidity::Stale;

        let result = compose(&profile(false), &pair).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert_eq!(result.non_current_assessment_count, 1);
        assert!(has_reason(&result, CompositionReason::RequiredRoleMissing));
    }

    #[test]
    fn subject_substitution_cannot_reuse_an_old_support_assessment() {
        let mut old_subject = assessment(EvaluatorRole::IndependentImplementation, 12, 2);
        old_subject.subject_digest = digest(99);
        let assessments = vec![
            assessment(EvaluatorRole::NormativeContract, 11, 1),
            old_subject,
        ];

        let result = compose(&profile(false), &assessments).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert_eq!(result.wrong_subject_assessment_count, 1);
        assert!(has_reason(
            &result,
            CompositionReason::WrongSubjectEvidenceIgnored
        ));
    }

    #[test]
    fn non_hard_refutation_is_preserved_as_indeterminate_not_voted_away() {
        let mut assessments = required_pair();
        let mut refutation = assessment(EvaluatorRole::EmpiricalEvidenceAppraiser, 15, 5);
        refutation.outcome = AssessmentOutcome::Refutes;
        assessments.push(refutation);

        let result = compose(&profile(false), &assessments).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(has_reason(
            &result,
            CompositionReason::NonHardRefutationObserved
        ));
    }

    #[test]
    fn contradictory_outputs_from_one_evaluator_profile_are_indeterminate() {
        let mut assessments = required_pair();
        let mut contradictory = assessments[0].clone();
        contradictory.outcome = AssessmentOutcome::Refutes;
        assessments.push(contradictory);

        let result = compose(&profile(false), &assessments).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(has_reason(
            &result,
            CompositionReason::ContradictoryEvaluatorOutput
        ));
    }

    #[test]
    fn exhaustive_required_role_product_rejects_every_non_current_or_non_supporting_pair() {
        let outcomes = [
            AssessmentOutcome::Supports,
            AssessmentOutcome::Refutes,
            AssessmentOutcome::Indeterminate,
            AssessmentOutcome::OutOfScope,
            AssessmentOutcome::Unavailable,
        ];
        let validities = [
            EvidenceValidity::Current,
            EvidenceValidity::Stale,
            EvidenceValidity::Revoked,
            EvidenceValidity::Invalid,
        ];

        let mut evaluated_states = 0usize;
        for left_outcome in outcomes {
            for left_validity in validities {
                for right_outcome in outcomes {
                    for right_validity in validities {
                        let mut pair = required_pair();
                        pair[0].outcome = left_outcome;
                        pair[0].validity = left_validity;
                        pair[1].outcome = right_outcome;
                        pair[1].validity = right_validity;

                        let result = compose(&profile(false), &pair).unwrap();
                        let both_current_support = left_outcome == AssessmentOutcome::Supports
                            && left_validity == EvidenceValidity::Current
                            && right_outcome == AssessmentOutcome::Supports
                            && right_validity == EvidenceValidity::Current;
                        let expected = if both_current_support {
                            CompositionDisposition::MeetsDeclaredProfile
                        } else {
                            CompositionDisposition::Indeterminate
                        };

                        assert_eq!(
                            result.disposition, expected,
                            "unexpected composition for left=({left_outcome:?}, {left_validity:?}), right=({right_outcome:?}, {right_validity:?})"
                        );
                        evaluated_states += 1;
                    }
                }
            }
        }
        assert_eq!(evaluated_states, 400);
    }

    #[test]
    fn composition_is_invariant_under_assessment_order() {
        let pair = required_pair();
        let forward = compose(&profile(false), &pair).unwrap();
        let mut reverse_pair = pair;
        reverse_pair.reverse();
        let reversed = compose(&profile(false), &reverse_pair).unwrap();

        assert_eq!(forward, reversed);
    }

    #[test]
    fn stale_hard_invariant_refutation_is_not_misreported_as_a_current_refutation() {
        let mut assessments = required_pair();
        let mut stale_refutation = assessment(EvaluatorRole::InvariantChecker, 16, 6);
        stale_refutation.outcome = AssessmentOutcome::Refutes;
        stale_refutation.validity = EvidenceValidity::Stale;
        assessments.push(stale_refutation);

        let result = compose(&profile(true), &assessments).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(!has_reason(
            &result,
            CompositionReason::HardInvariantRefuted
        ));
        assert!(has_reason(
            &result,
            CompositionReason::NonCurrentEvidenceObserved
        ));
    }

    #[test]
    fn wrong_subject_refutation_cannot_refute_the_current_hard_invariant() {
        let mut assessments = required_pair();
        let mut old_subject_refutation = assessment(EvaluatorRole::InvariantChecker, 17, 7);
        old_subject_refutation.subject_digest = digest(98);
        old_subject_refutation.outcome = AssessmentOutcome::Refutes;
        assessments.push(old_subject_refutation);

        let result = compose(&profile(true), &assessments).unwrap();
        assert_eq!(result.disposition, CompositionDisposition::Indeterminate);
        assert!(!has_reason(
            &result,
            CompositionReason::HardInvariantRefuted
        ));
        assert_eq!(result.wrong_subject_assessment_count, 1);
    }

    #[test]
    fn repeated_exact_replays_preserve_profile_result_without_adding_corroboration() {
        let pair = required_pair();
        let mut replayed = pair.clone();
        for _ in 0..5 {
            replayed.extend(pair.clone());
        }

        let result = compose(&profile(false), &replayed).unwrap();
        assert_eq!(
            result.disposition,
            CompositionDisposition::MeetsDeclaredProfile
        );
        assert_eq!(result.unique_assessment_count, 2);
        assert_eq!(result.current_assessment_count, 2);
        assert_eq!(result.duplicate_replay_count, 10);
        assert!(has_reason(
            &result,
            CompositionReason::DuplicateReplaySuppressed
        ));
    }

    #[test]
    fn invalid_profile_cannot_request_self_separation() {
        let mut invalid = profile(false);
        invalid.required_separations[0].right_role = EvaluatorRole::NormativeContract;

        assert_eq!(invalid.validate(), Err(CompositionError::SelfSeparation));
    }
}

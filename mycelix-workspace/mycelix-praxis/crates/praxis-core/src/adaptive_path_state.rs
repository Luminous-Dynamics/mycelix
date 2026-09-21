// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Versioned adaptive-path plans separated from evidence-derived execution progress.
//!
//! A plan says what sequence is proposed. A progress projection says what a named
//! policy concludes from exact inputs. Neither persistence nor plan position is a
//! credential or authorization decision.

use crate::goal_state::GoalIntentId;
use crate::learning_evidence::LearnerId;
use crate::recommendation_state::{GoalIntentRef, RecommendationInputRef, RecommendationTarget};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct AdaptivePathPlanId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct PathPlanStepId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PathPlanStepKind {
    Learn,
    Practice,
    Review,
    Assess,
    Project,
    Rest,
}

/// One proposed step. It deliberately contains no `is_completed` field.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PathPlanStep {
    pub step_id: PathPlanStepId,
    pub target: RecommendationTarget,
    pub kind: PathPlanStepKind,
    pub expected_duration_minutes: u32,
    pub is_optional: bool,
    pub rationale: String,
}

/// Where a plan version came from.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PathPlanOrigin {
    LearnerAuthored,
    Derived {
        planner_id: String,
        planner_version: String,
        parameters_digest: String,
        input_refs: Vec<RecommendationInputRef>,
    },
}

/// Versioned plan structure, separate from execution/progress state.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdaptivePathPlan {
    pub plan_id: AdaptivePathPlanId,
    pub version: u64,
    pub learner_id: LearnerId,
    pub name: String,
    pub goal_intent_ref: Option<GoalIntentRef>,
    pub origin: PathPlanOrigin,
    pub steps: Vec<PathPlanStep>,
    /// Previous plan version when this version is an adaptation.
    pub parent_plan_version: Option<u64>,
    pub adaptation_reason: Option<String>,
    pub estimated_total_minutes: Option<u32>,
    pub created_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PathProgressPolicyProvenance {
    pub policy_id: String,
    pub policy_version: String,
    pub parameters_digest: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PathStepProgressOutcome {
    NotEvaluated,
    AdmittedIncomplete,
    Inconclusive { reason: String },
    AdmittedCompleteUnderProfile,
    /// Learner chose not to pursue this optional/advisory step. This is not a
    /// capability-completion claim.
    SkippedByLearner,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PathStepProgress {
    pub step_id: PathPlanStepId,
    pub outcome: PathStepProgressOutcome,
    /// Exact inputs supporting this step outcome. `NotEvaluated` carries no inputs.
    pub input_refs: Vec<RecommendationInputRef>,
}

/// Recomputable progress receipt for one exact path-plan version.
///
/// It has no stored `current_step`, `completed_steps`, or adaptation count. Those are
/// derived from the plan version and policy-relative step outcomes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdaptivePathProgressProjection {
    pub plan_id: AdaptivePathPlanId,
    pub plan_version: u64,
    pub learner_id: LearnerId,
    pub policy: PathProgressPolicyProvenance,
    /// Exact unique dependency set used by all evaluated step outcomes.
    pub input_refs: Vec<RecommendationInputRef>,
    pub step_progress: Vec<PathStepProgress>,
    pub support_confidence_permille: u16,
    pub estimated_completion: Option<i64>,
    pub generated_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdaptivePathContractError {
    EmptyPlanId,
    ZeroPlanVersion,
    EmptyLearnerId,
    EmptyPlanName,
    InvalidGoalIntentRef,
    NoSteps,
    EmptyStepId,
    DuplicateStepId(PathPlanStepId),
    EmptyStepTarget,
    ZeroStepDuration(PathPlanStepId),
    EmptyStepRationale(PathPlanStepId),
    EmptyPlannerId,
    EmptyPlannerVersion,
    EmptyPlannerParametersDigest,
    DerivedPlanHasNoInputs,
    DuplicatePlannerInput(RecommendationInputRef),
    InvalidPlannerInput,
    InvalidParentPlanVersion,
    MissingAdaptationReason,
    UnexpectedAdaptationReason,
    EmptyPolicyId,
    EmptyPolicyVersion,
    EmptyPolicyParametersDigest,
    ConfidenceOutOfRange(u16),
    DuplicateProgressInput(RecommendationInputRef),
    InvalidProgressInput,
    EmptyProgressStepId,
    DuplicateProgressStepId(PathPlanStepId),
    MissingProgressForPlanStep(PathPlanStepId),
    ProgressForUnknownPlanStep(PathPlanStepId),
    ProgressOrderMismatch {
        expected: PathPlanStepId,
        actual: PathPlanStepId,
    },
    NotEvaluatedHasInputs(PathPlanStepId),
    EvaluatedStepHasNoInputs(PathPlanStepId),
    EmptyInconclusiveReason(PathPlanStepId),
    StepInputNotDeclared {
        step_id: PathPlanStepId,
        input: RecommendationInputRef,
    },
    DuplicateStepInput {
        step_id: PathPlanStepId,
        input: RecommendationInputRef,
    },
    DeclaredProgressInputUnused(RecommendationInputRef),
    LearnerMismatch,
    PlanIdentityMismatch,
    RequiredStepSkipped(PathPlanStepId),
}

impl AdaptivePathPlan {
    pub fn validate(&self) -> Result<(), AdaptivePathContractError> {
        if self.plan_id.0.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyPlanId);
        }
        if self.version == 0 {
            return Err(AdaptivePathContractError::ZeroPlanVersion);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyLearnerId);
        }
        if self.name.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyPlanName);
        }
        if let Some(goal) = &self.goal_intent_ref {
            if goal.goal_intent_id.0.trim().is_empty() || goal.version == 0 {
                return Err(AdaptivePathContractError::InvalidGoalIntentRef);
            }
        }
        if self.steps.is_empty() {
            return Err(AdaptivePathContractError::NoSteps);
        }

        let mut step_ids = BTreeSet::new();
        for step in &self.steps {
            if step.step_id.0.trim().is_empty() {
                return Err(AdaptivePathContractError::EmptyStepId);
            }
            if !step_ids.insert(step.step_id.clone()) {
                return Err(AdaptivePathContractError::DuplicateStepId(step.step_id.clone()));
            }
            if target_is_empty(&step.target) {
                return Err(AdaptivePathContractError::EmptyStepTarget);
            }
            if step.expected_duration_minutes == 0 {
                return Err(AdaptivePathContractError::ZeroStepDuration(step.step_id.clone()));
            }
            if step.rationale.trim().is_empty() {
                return Err(AdaptivePathContractError::EmptyStepRationale(step.step_id.clone()));
            }
        }

        match &self.origin {
            PathPlanOrigin::LearnerAuthored => {}
            PathPlanOrigin::Derived {
                planner_id,
                planner_version,
                parameters_digest,
                input_refs,
            } => {
                if planner_id.trim().is_empty() {
                    return Err(AdaptivePathContractError::EmptyPlannerId);
                }
                if planner_version.trim().is_empty() {
                    return Err(AdaptivePathContractError::EmptyPlannerVersion);
                }
                if parameters_digest.trim().is_empty() {
                    return Err(AdaptivePathContractError::EmptyPlannerParametersDigest);
                }
                if input_refs.is_empty() {
                    return Err(AdaptivePathContractError::DerivedPlanHasNoInputs);
                }
                let mut inputs = BTreeSet::new();
                for input in input_refs {
                    if !input_is_well_formed(input) {
                        return Err(AdaptivePathContractError::InvalidPlannerInput);
                    }
                    if !inputs.insert(input.clone()) {
                        return Err(AdaptivePathContractError::DuplicatePlannerInput(input.clone()));
                    }
                }
            }
        }

        match (self.version, self.parent_plan_version, &self.adaptation_reason) {
            (1, None, None) => {}
            (1, _, _) => return Err(AdaptivePathContractError::InvalidParentPlanVersion),
            (version, Some(parent), Some(reason)) if parent < version && !reason.trim().is_empty() => {}
            (_, Some(_), None) => return Err(AdaptivePathContractError::MissingAdaptationReason),
            (_, Some(_), Some(reason)) if reason.trim().is_empty() => {
                return Err(AdaptivePathContractError::MissingAdaptationReason)
            }
            (_, None, Some(_)) => return Err(AdaptivePathContractError::UnexpectedAdaptationReason),
            (_, None, None) => return Err(AdaptivePathContractError::InvalidParentPlanVersion),
            _ => return Err(AdaptivePathContractError::InvalidParentPlanVersion),
        }

        Ok(())
    }

    pub const fn is_source_evidence(&self) -> bool {
        false
    }
}

impl AdaptivePathProgressProjection {
    pub fn validate(&self) -> Result<(), AdaptivePathContractError> {
        if self.plan_id.0.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyPlanId);
        }
        if self.plan_version == 0 {
            return Err(AdaptivePathContractError::ZeroPlanVersion);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyLearnerId);
        }
        if self.policy.policy_id.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyPolicyId);
        }
        if self.policy.policy_version.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyPolicyVersion);
        }
        if self.policy.parameters_digest.trim().is_empty() {
            return Err(AdaptivePathContractError::EmptyPolicyParametersDigest);
        }
        if self.support_confidence_permille > 1000 {
            return Err(AdaptivePathContractError::ConfidenceOutOfRange(
                self.support_confidence_permille,
            ));
        }

        let mut declared = BTreeSet::new();
        for input in &self.input_refs {
            if !input_is_well_formed(input) {
                return Err(AdaptivePathContractError::InvalidProgressInput);
            }
            if !declared.insert(input.clone()) {
                return Err(AdaptivePathContractError::DuplicateProgressInput(input.clone()));
            }
        }

        let mut step_ids = BTreeSet::new();
        let mut used = BTreeSet::new();
        for step in &self.step_progress {
            if step.step_id.0.trim().is_empty() {
                return Err(AdaptivePathContractError::EmptyProgressStepId);
            }
            if !step_ids.insert(step.step_id.clone()) {
                return Err(AdaptivePathContractError::DuplicateProgressStepId(
                    step.step_id.clone(),
                ));
            }
            match &step.outcome {
                PathStepProgressOutcome::NotEvaluated => {
                    if !step.input_refs.is_empty() {
                        return Err(AdaptivePathContractError::NotEvaluatedHasInputs(
                            step.step_id.clone(),
                        ));
                    }
                }
                PathStepProgressOutcome::Inconclusive { reason } => {
                    if reason.trim().is_empty() {
                        return Err(AdaptivePathContractError::EmptyInconclusiveReason(
                            step.step_id.clone(),
                        ));
                    }
                    validate_step_inputs(step, &declared, &mut used)?;
                }
                _ => validate_step_inputs(step, &declared, &mut used)?,
            }
        }

        for input in declared {
            if !used.contains(&input) {
                return Err(AdaptivePathContractError::DeclaredProgressInputUnused(input));
            }
        }

        Ok(())
    }

    /// Validate this projection against the exact plan version it claims to describe.
    pub fn validate_against_plan(
        &self,
        plan: &AdaptivePathPlan,
    ) -> Result<(), AdaptivePathContractError> {
        self.validate()?;
        plan.validate()?;

        if self.plan_id != plan.plan_id || self.plan_version != plan.version {
            return Err(AdaptivePathContractError::PlanIdentityMismatch);
        }
        if self.learner_id != plan.learner_id {
            return Err(AdaptivePathContractError::LearnerMismatch);
        }

        let progress_ids: BTreeSet<_> = self.step_progress.iter().map(|s| s.step_id.clone()).collect();
        for planned in &plan.steps {
            if !progress_ids.contains(&planned.step_id) {
                return Err(AdaptivePathContractError::MissingProgressForPlanStep(
                    planned.step_id.clone(),
                ));
            }
        }
        let plan_ids: BTreeSet<_> = plan.steps.iter().map(|s| s.step_id.clone()).collect();
        for progress in &self.step_progress {
            if !plan_ids.contains(&progress.step_id) {
                return Err(AdaptivePathContractError::ProgressForUnknownPlanStep(
                    progress.step_id.clone(),
                ));
            }
        }
        for (planned, progress) in plan.steps.iter().zip(self.step_progress.iter()) {
            if planned.step_id != progress.step_id {
                return Err(AdaptivePathContractError::ProgressOrderMismatch {
                    expected: planned.step_id.clone(),
                    actual: progress.step_id.clone(),
                });
            }
            if !planned.is_optional
                && matches!(progress.outcome, PathStepProgressOutcome::SkippedByLearner)
            {
                return Err(AdaptivePathContractError::RequiredStepSkipped(
                    planned.step_id.clone(),
                ));
            }
        }

        Ok(())
    }

    pub fn admitted_complete_count(&self) -> usize {
        self.step_progress
            .iter()
            .filter(|step| {
                matches!(
                    step.outcome,
                    PathStepProgressOutcome::AdmittedCompleteUnderProfile
                )
            })
            .count()
    }

    pub fn next_unresolved_step(&self) -> Option<&PathPlanStepId> {
        self.step_progress
            .iter()
            .find(|step| {
                !matches!(
                    step.outcome,
                    PathStepProgressOutcome::AdmittedCompleteUnderProfile
                        | PathStepProgressOutcome::SkippedByLearner
                )
            })
            .map(|step| &step.step_id)
    }

    pub fn admitted_complete_under_profile(
        &self,
        plan: &AdaptivePathPlan,
    ) -> Result<bool, AdaptivePathContractError> {
        self.validate_against_plan(plan)?;
        Ok(plan.steps.iter().zip(self.step_progress.iter()).all(|(planned, progress)| {
            matches!(
                progress.outcome,
                PathStepProgressOutcome::AdmittedCompleteUnderProfile
            ) || (planned.is_optional
                && matches!(progress.outcome, PathStepProgressOutcome::SkippedByLearner))
        }))
    }

    pub const fn is_source_evidence(&self) -> bool {
        false
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }

    pub const fn grants_authorization(&self) -> bool {
        false
    }
}

fn validate_step_inputs(
    step: &PathStepProgress,
    declared: &BTreeSet<RecommendationInputRef>,
    used: &mut BTreeSet<RecommendationInputRef>,
) -> Result<(), AdaptivePathContractError> {
    if step.input_refs.is_empty() {
        return Err(AdaptivePathContractError::EvaluatedStepHasNoInputs(
            step.step_id.clone(),
        ));
    }
    let mut local = BTreeSet::new();
    for input in &step.input_refs {
        if !input_is_well_formed(input) {
            return Err(AdaptivePathContractError::InvalidProgressInput);
        }
        if !local.insert(input.clone()) {
            return Err(AdaptivePathContractError::DuplicateStepInput {
                step_id: step.step_id.clone(),
                input: input.clone(),
            });
        }
        if !declared.contains(input) {
            return Err(AdaptivePathContractError::StepInputNotDeclared {
                step_id: step.step_id.clone(),
                input: input.clone(),
            });
        }
        used.insert(input.clone());
    }
    Ok(())
}

fn input_is_well_formed(input: &RecommendationInputRef) -> bool {
    match input {
        RecommendationInputRef::Evidence(event) => !event.0.trim().is_empty(),
        RecommendationInputRef::Projection(projection) => {
            !projection.projection_kind.trim().is_empty()
                && !projection.projection_id.trim().is_empty()
                && !projection.projection_digest.trim().is_empty()
        }
        RecommendationInputRef::GoalIntent(goal) => {
            !goal.goal_intent_id.0.trim().is_empty() && goal.version > 0
        }
    }
}

fn target_is_empty(target: &RecommendationTarget) -> bool {
    match target {
        RecommendationTarget::Capability(id) => id.0.trim().is_empty(),
        RecommendationTarget::Resource(id)
        | RecommendationTarget::Course(id)
        | RecommendationTarget::Project(id)
        | RecommendationTarget::PeerGroup(id)
        | RecommendationTarget::Custom(id) => id.trim().is_empty(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::goal_state::DerivedProjectionRef;
    use crate::learning_evidence::CapabilityId;

    fn dependency() -> RecommendationInputRef {
        RecommendationInputRef::Projection(DerivedProjectionRef {
            projection_kind: "goal-progress".into(),
            projection_id: "gp-1".into(),
            projection_digest: "blake3:abc".into(),
        })
    }

    fn plan() -> AdaptivePathPlan {
        AdaptivePathPlan {
            plan_id: AdaptivePathPlanId("path-1".into()),
            version: 1,
            learner_id: LearnerId("learner-1".into()),
            name: "Rust ownership path".into(),
            goal_intent_ref: Some(GoalIntentRef {
                goal_intent_id: GoalIntentId("goal-1".into()),
                version: 2,
            }),
            origin: PathPlanOrigin::Derived {
                planner_id: "praxis:path-planner".into(),
                planner_version: "1".into(),
                parameters_digest: "blake3:planner".into(),
                input_refs: vec![dependency()],
            },
            steps: vec![
                PathPlanStep {
                    step_id: PathPlanStepId("step-1".into()),
                    target: RecommendationTarget::Capability(CapabilityId("rust:ownership".into())),
                    kind: PathPlanStepKind::Learn,
                    expected_duration_minutes: 30,
                    is_optional: false,
                    rationale: "Foundational capability".into(),
                },
                PathPlanStep {
                    step_id: PathPlanStepId("step-2".into()),
                    target: RecommendationTarget::Resource("exercise:borrow-checker".into()),
                    kind: PathPlanStepKind::Practice,
                    expected_duration_minutes: 20,
                    is_optional: true,
                    rationale: "Optional practice".into(),
                },
            ],
            parent_plan_version: None,
            adaptation_reason: None,
            estimated_total_minutes: Some(50),
            created_at: 100,
        }
    }

    fn progress() -> AdaptivePathProgressProjection {
        let input = dependency();
        AdaptivePathProgressProjection {
            plan_id: AdaptivePathPlanId("path-1".into()),
            plan_version: 1,
            learner_id: LearnerId("learner-1".into()),
            policy: PathProgressPolicyProvenance {
                policy_id: "praxis:path-progress".into(),
                policy_version: "1".into(),
                parameters_digest: "blake3:progress".into(),
            },
            input_refs: vec![input.clone()],
            step_progress: vec![
                PathStepProgress {
                    step_id: PathPlanStepId("step-1".into()),
                    outcome: PathStepProgressOutcome::AdmittedCompleteUnderProfile,
                    input_refs: vec![input.clone()],
                },
                PathStepProgress {
                    step_id: PathPlanStepId("step-2".into()),
                    outcome: PathStepProgressOutcome::NotEvaluated,
                    input_refs: vec![],
                },
            ],
            support_confidence_permille: 600,
            estimated_completion: None,
            generated_at: 200,
        }
    }

    #[test]
    fn plan_structure_contains_no_execution_progress() {
        let plan = plan();
        assert_eq!(plan.validate(), Ok(()));
        assert!(!plan.is_source_evidence());
        // Structural invariant: PathPlanStep has no is_completed field and the plan
        // has no current_step/completed_steps fields.
    }

    #[test]
    fn progress_must_match_exact_plan_version_and_step_order() {
        let plan = plan();
        let progress = progress();
        assert_eq!(progress.validate_against_plan(&plan), Ok(()));
        assert_eq!(progress.admitted_complete_count(), 1);
        assert_eq!(
            progress.next_unresolved_step(),
            Some(&PathPlanStepId("step-2".into()))
        );
    }

    #[test]
    fn completed_path_is_profile_relative_and_non_authoritative() {
        let plan = plan();
        let mut progress = progress();
        progress.step_progress[1] = PathStepProgress {
            step_id: PathPlanStepId("step-2".into()),
            outcome: PathStepProgressOutcome::SkippedByLearner,
            input_refs: vec![dependency()],
        };
        assert_eq!(progress.admitted_complete_under_profile(&plan), Ok(true));
        assert!(!progress.is_source_evidence());
        assert!(!progress.grants_credential_authority());
        assert!(!progress.grants_authorization());
    }

    #[test]
    fn required_step_cannot_be_skipped_as_completion() {
        let plan = plan();
        let mut progress = progress();
        progress.step_progress[0] = PathStepProgress {
            step_id: PathPlanStepId("step-1".into()),
            outcome: PathStepProgressOutcome::SkippedByLearner,
            input_refs: vec![dependency()],
        };
        assert_eq!(
            progress.validate_against_plan(&plan),
            Err(AdaptivePathContractError::RequiredStepSkipped(
                PathPlanStepId("step-1".into())
            ))
        );
    }

    #[test]
    fn adapted_plan_requires_parent_and_reason() {
        let mut plan = plan();
        plan.version = 2;
        assert_eq!(
            plan.validate(),
            Err(AdaptivePathContractError::InvalidParentPlanVersion)
        );

        plan.parent_plan_version = Some(1);
        assert_eq!(
            plan.validate(),
            Err(AdaptivePathContractError::MissingAdaptationReason)
        );

        plan.adaptation_reason = Some("Goal revision changed ordering".into());
        assert_eq!(plan.validate(), Ok(()));
    }
}

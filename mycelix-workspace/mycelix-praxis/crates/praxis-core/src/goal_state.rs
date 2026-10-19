// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Learner-authored goal intent and evidence-bound goal-progress projections.
//!
//! The source intent deliberately contains no progress/completion field. Derived
//! progress belongs to a separately versioned policy projection over exact inputs.

use crate::learning_evidence::{CapabilityId, EvidenceEventId, LearnerId};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct GoalIntentId(pub String);

/// What the learner intends to pursue. Targets describe intent; they are not claims
/// that the learner already possesses the capability/credential/project outcome.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum GoalTarget {
    Capability(CapabilityId),
    Resource(String),
    Credential(String),
    Project(String),
    Custom(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GoalPriority {
    Low,
    Medium,
    High,
    Critical,
}

impl Default for GoalPriority {
    fn default() -> Self {
        Self::Medium
    }
}

/// Disclosure is explicit and private by default. This type intentionally has no
/// `Public` variant for the full goal intent; public sharing should use a separate,
/// minimized disclosure projection rather than publishing the source object.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GoalDisclosure {
    Private,
    ExplicitlyShared { audience_ids: Vec<String> },
}

impl Default for GoalDisclosure {
    fn default() -> Self {
        Self::Private
    }
}

/// Learner-authored goal source state.
///
/// There is intentionally no `progress`, `completed`, `mastered`, or `credentialed`
/// field here. Revising the goal changes intent; it does not rewrite learning evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GoalIntent {
    pub goal_intent_id: GoalIntentId,
    /// Monotonically increasing learner-authored revision number.
    pub version: u64,
    pub learner_id: LearnerId,
    pub title: String,
    pub description: String,
    pub targets: Vec<GoalTarget>,
    pub priority: GoalPriority,
    pub target_date: Option<i64>,
    pub estimated_hours: Option<u32>,
    pub disclosure: GoalDisclosure,
    pub authored_at: i64,
    pub revised_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct DerivedProjectionRef {
    pub projection_kind: String,
    pub projection_id: String,
    pub projection_digest: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GoalProgressPolicyProvenance {
    pub policy_id: String,
    pub policy_version: String,
    pub parameters_digest: String,
}

/// Policy-relative interpretation of goal completion.
///
/// `AdmittedCompleteUnderProfile` is deliberately not named simply `Completed`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GoalCompletionOutcome {
    NotEvaluated,
    AdmittedIncomplete,
    Inconclusive { reason: String },
    AdmittedCompleteUnderProfile,
}

/// Derived, recomputable goal-progress state.
///
/// The projection binds to the exact goal-intent version and exact evidence and/or
/// derived projection inputs used by a named/versioned policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GoalProgressProjection {
    pub goal_intent_id: GoalIntentId,
    pub goal_intent_version: u64,
    pub learner_id: LearnerId,
    pub policy: GoalProgressPolicyProvenance,
    pub input_event_ids: Vec<EvidenceEventId>,
    pub input_projection_refs: Vec<DerivedProjectionRef>,
    /// Policy-derived summary for UX/planning. It is not source evidence.
    pub progress_estimate_permille: u16,
    /// Support metadata from the named policy; not assumed calibrated probability.
    pub support_confidence_permille: u16,
    pub completion_outcome: GoalCompletionOutcome,
    pub generated_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GoalStateContractError {
    EmptyGoalIntentId,
    ZeroGoalIntentVersion,
    EmptyLearnerId,
    EmptyTitle,
    NoTargets,
    EmptyTarget,
    DuplicateTarget,
    RevisionBeforeAuthorship,
    EmptyAudienceId,
    DuplicateAudienceId,
    EmptyPolicyId,
    EmptyPolicyVersion,
    EmptyParametersDigest,
    NoProjectionInputs,
    EmptyEvidenceEventId,
    DuplicateEvidenceEventId(EvidenceEventId),
    EmptyProjectionKind,
    EmptyProjectionId,
    EmptyProjectionDigest,
    DuplicateProjectionRef(DerivedProjectionRef),
    ProgressOutOfRange(u16),
    ConfidenceOutOfRange(u16),
    EmptyInconclusiveReason,
}

impl GoalIntent {
    pub fn validate(&self) -> Result<(), GoalStateContractError> {
        if self.goal_intent_id.0.trim().is_empty() {
            return Err(GoalStateContractError::EmptyGoalIntentId);
        }
        if self.version == 0 {
            return Err(GoalStateContractError::ZeroGoalIntentVersion);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(GoalStateContractError::EmptyLearnerId);
        }
        if self.title.trim().is_empty() {
            return Err(GoalStateContractError::EmptyTitle);
        }
        if self.targets.is_empty() {
            return Err(GoalStateContractError::NoTargets);
        }

        let mut targets = BTreeSet::new();
        for target in &self.targets {
            let empty = match target {
                GoalTarget::Capability(id) => id.0.trim().is_empty(),
                GoalTarget::Resource(id)
                | GoalTarget::Credential(id)
                | GoalTarget::Project(id)
                | GoalTarget::Custom(id) => id.trim().is_empty(),
            };
            if empty {
                return Err(GoalStateContractError::EmptyTarget);
            }
            if !targets.insert(target.clone()) {
                return Err(GoalStateContractError::DuplicateTarget);
            }
        }

        if self.revised_at < self.authored_at {
            return Err(GoalStateContractError::RevisionBeforeAuthorship);
        }

        if let GoalDisclosure::ExplicitlyShared { audience_ids } = &self.disclosure {
            let mut audience = BTreeSet::new();
            for id in audience_ids {
                if id.trim().is_empty() {
                    return Err(GoalStateContractError::EmptyAudienceId);
                }
                if !audience.insert(id) {
                    return Err(GoalStateContractError::DuplicateAudienceId);
                }
            }
        }

        Ok(())
    }

    pub const fn is_private_by_default(disclosure: &GoalDisclosure) -> bool {
        matches!(disclosure, GoalDisclosure::Private)
    }
}

impl GoalProgressProjection {
    /// Validate structural/provenance invariants only. A valid projection means the
    /// named policy has a well-formed receipt, not that its interpretation is
    /// universally correct or suitable for credentials/authorization.
    pub fn validate(&self) -> Result<(), GoalStateContractError> {
        if self.goal_intent_id.0.trim().is_empty() {
            return Err(GoalStateContractError::EmptyGoalIntentId);
        }
        if self.goal_intent_version == 0 {
            return Err(GoalStateContractError::ZeroGoalIntentVersion);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(GoalStateContractError::EmptyLearnerId);
        }
        if self.policy.policy_id.trim().is_empty() {
            return Err(GoalStateContractError::EmptyPolicyId);
        }
        if self.policy.policy_version.trim().is_empty() {
            return Err(GoalStateContractError::EmptyPolicyVersion);
        }
        if self.policy.parameters_digest.trim().is_empty() {
            return Err(GoalStateContractError::EmptyParametersDigest);
        }
        if self.input_event_ids.is_empty() && self.input_projection_refs.is_empty() {
            return Err(GoalStateContractError::NoProjectionInputs);
        }
        if self.progress_estimate_permille > 1000 {
            return Err(GoalStateContractError::ProgressOutOfRange(
                self.progress_estimate_permille,
            ));
        }
        if self.support_confidence_permille > 1000 {
            return Err(GoalStateContractError::ConfidenceOutOfRange(
                self.support_confidence_permille,
            ));
        }
        if let GoalCompletionOutcome::Inconclusive { reason } = &self.completion_outcome {
            if reason.trim().is_empty() {
                return Err(GoalStateContractError::EmptyInconclusiveReason);
            }
        }

        let mut events = BTreeSet::new();
        for event_id in &self.input_event_ids {
            if event_id.0.trim().is_empty() {
                return Err(GoalStateContractError::EmptyEvidenceEventId);
            }
            if !events.insert(event_id.clone()) {
                return Err(GoalStateContractError::DuplicateEvidenceEventId(
                    event_id.clone(),
                ));
            }
        }

        let mut projections = BTreeSet::new();
        for projection in &self.input_projection_refs {
            if projection.projection_kind.trim().is_empty() {
                return Err(GoalStateContractError::EmptyProjectionKind);
            }
            if projection.projection_id.trim().is_empty() {
                return Err(GoalStateContractError::EmptyProjectionId);
            }
            if projection.projection_digest.trim().is_empty() {
                return Err(GoalStateContractError::EmptyProjectionDigest);
            }
            if !projections.insert(projection.clone()) {
                return Err(GoalStateContractError::DuplicateProjectionRef(
                    projection.clone(),
                ));
            }
        }

        Ok(())
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }

    pub const fn grants_authorization(&self) -> bool {
        false
    }

    pub const fn is_source_evidence(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn goal() -> GoalIntent {
        GoalIntent {
            goal_intent_id: GoalIntentId("goal-1".into()),
            version: 1,
            learner_id: LearnerId("learner-1".into()),
            title: "Learn Rust ownership".into(),
            description: "Build enough capability to use ownership safely.".into(),
            targets: vec![GoalTarget::Capability(CapabilityId("rust:ownership".into()))],
            priority: GoalPriority::High,
            target_date: None,
            estimated_hours: Some(20),
            disclosure: GoalDisclosure::default(),
            authored_at: 100,
            revised_at: 100,
        }
    }

    fn projection() -> GoalProgressProjection {
        GoalProgressProjection {
            goal_intent_id: GoalIntentId("goal-1".into()),
            goal_intent_version: 1,
            learner_id: LearnerId("learner-1".into()),
            policy: GoalProgressPolicyProvenance {
                policy_id: "praxis:goal-progress".into(),
                policy_version: "1".into(),
                parameters_digest: "blake3:abc".into(),
            },
            input_event_ids: vec![EvidenceEventId("event-1".into())],
            input_projection_refs: vec![DerivedProjectionRef {
                projection_kind: "bkt-advisory".into(),
                projection_id: "bkt-1".into(),
                projection_digest: "blake3:def".into(),
            }],
            progress_estimate_permille: 720,
            support_confidence_permille: 600,
            completion_outcome: GoalCompletionOutcome::AdmittedIncomplete,
            generated_at: 200,
        }
    }

    #[test]
    fn goal_intent_defaults_private_and_contains_no_derived_state() {
        let goal = goal();
        assert_eq!(goal.validate(), Ok(()));
        assert!(GoalIntent::is_private_by_default(&goal.disclosure));
        // Compile-time structure is the key invariant: GoalIntent has no progress or
        // completion fields to mutate or mistake for source evidence.
    }

    #[test]
    fn goal_intent_rejects_duplicate_targets() {
        let mut goal = goal();
        goal.targets.push(goal.targets[0].clone());
        assert_eq!(goal.validate(), Err(GoalStateContractError::DuplicateTarget));
    }

    #[test]
    fn explicit_sharing_rejects_duplicate_audience_ids() {
        let mut goal = goal();
        goal.disclosure = GoalDisclosure::ExplicitlyShared {
            audience_ids: vec!["mentor-1".into(), "mentor-1".into()],
        };
        assert_eq!(
            goal.validate(),
            Err(GoalStateContractError::DuplicateAudienceId)
        );
    }

    #[test]
    fn progress_projection_is_derived_and_non_authoritative() {
        let projection = projection();
        assert_eq!(projection.validate(), Ok(()));
        assert!(!projection.is_source_evidence());
        assert!(!projection.grants_credential_authority());
        assert!(!projection.grants_authorization());
    }

    #[test]
    fn progress_projection_requires_exact_inputs() {
        let mut projection = projection();
        projection.input_event_ids.clear();
        projection.input_projection_refs.clear();
        assert_eq!(
            projection.validate(),
            Err(GoalStateContractError::NoProjectionInputs)
        );
    }

    #[test]
    fn progress_projection_rejects_duplicate_evidence() {
        let mut projection = projection();
        projection
            .input_event_ids
            .push(EvidenceEventId("event-1".into()));
        assert_eq!(
            projection.validate(),
            Err(GoalStateContractError::DuplicateEvidenceEventId(
                EvidenceEventId("event-1".into())
            ))
        );
    }

    #[test]
    fn completion_is_explicitly_profile_relative() {
        let mut projection = projection();
        projection.completion_outcome = GoalCompletionOutcome::AdmittedCompleteUnderProfile;
        assert_eq!(projection.validate(), Ok(()));
        assert!(!projection.grants_credential_authority());
        assert!(!projection.grants_authorization());
    }
}

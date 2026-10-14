// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-bound, expiring recommendation projections.
//!
//! Recommendations are derived advisory state. They have exact dependency inputs
//! and expiry, but no persisted timeless `is_valid` truth bit.

use crate::goal_state::{DerivedProjectionRef, GoalIntentId};
use crate::learning_evidence::{CapabilityId, EvidenceEventId, LearnerId};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RecommendationId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct GoalIntentRef {
    pub goal_intent_id: GoalIntentId,
    pub version: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum RecommendationInputRef {
    Evidence(EvidenceEventId),
    Projection(DerivedProjectionRef),
    GoalIntent(GoalIntentRef),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecommendationTarget {
    Capability(CapabilityId),
    Resource(String),
    Course(String),
    Project(String),
    PeerGroup(String),
    Custom(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecommendationKind {
    NextCapability,
    Review,
    Practice,
    Challenge,
    Resource,
    Course,
    Project,
    PeerGroup,
    Exploration,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum RecommendationSignalKind {
    Relevance,
    DifficultyMatch,
    Readiness,
    Freshness,
    RetentionRisk,
    GoalAlignment,
    InterleavingBenefit,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum RecommendationReasonCode {
    PathContinuation,
    RetentionRisk,
    GoalPrerequisite,
    GoalAlignment,
    DifficultyFit,
    PracticeNeed,
    TransferOpportunity,
    Exploration,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecommendationSignal {
    pub kind: RecommendationSignalKind,
    pub score_permille: u16,
    /// Exact subset of recommendation inputs used for this score.
    pub input_refs: Vec<RecommendationInputRef>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecommendationReason {
    pub code: RecommendationReasonCode,
    /// Exact subset of recommendation inputs supporting this reason.
    pub input_refs: Vec<RecommendationInputRef>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecommendationPolicyProvenance {
    pub policy_id: String,
    pub policy_version: String,
    pub parameters_digest: String,
}

/// Recomputable advisory recommendation.
///
/// There is intentionally no `is_valid` field. Temporal freshness is derived from
/// generated/expiry times; dependency freshness is checked against the exact refs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecommendationProjection {
    pub recommendation_id: RecommendationId,
    pub learner_id: LearnerId,
    pub target: RecommendationTarget,
    pub kind: RecommendationKind,
    pub policy: RecommendationPolicyProvenance,
    /// Exact unique dependency set consumed by scores/reasons.
    pub input_refs: Vec<RecommendationInputRef>,
    pub signals: Vec<RecommendationSignal>,
    pub reasons: Vec<RecommendationReason>,
    pub rank: u32,
    pub generated_at: i64,
    pub expires_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RecommendationTemporalStatus {
    NotYetGenerated,
    Fresh,
    Expired,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RecommendationContractError {
    EmptyRecommendationId,
    EmptyLearnerId,
    EmptyTarget,
    EmptyPolicyId,
    EmptyPolicyVersion,
    EmptyParametersDigest,
    NoInputs,
    DuplicateInput(RecommendationInputRef),
    EmptyEvidenceEventId,
    EmptyProjectionKind,
    EmptyProjectionId,
    EmptyProjectionDigest,
    EmptyGoalIntentId,
    ZeroGoalIntentVersion,
    NoSignals,
    EmptyOtherSignalName,
    DuplicateSignalKind(RecommendationSignalKind),
    SignalOutOfRange {
        kind: RecommendationSignalKind,
        value_permille: u16,
    },
    SignalHasNoInputs(RecommendationSignalKind),
    DuplicateSignalInput {
        kind: RecommendationSignalKind,
        input: RecommendationInputRef,
    },
    SignalInputNotDeclared {
        kind: RecommendationSignalKind,
        input: RecommendationInputRef,
    },
    NoReasons,
    EmptyOtherReasonName,
    DuplicateReasonCode(RecommendationReasonCode),
    ReasonHasNoInputs(RecommendationReasonCode),
    DuplicateReasonInput {
        code: RecommendationReasonCode,
        input: RecommendationInputRef,
    },
    ReasonInputNotDeclared {
        code: RecommendationReasonCode,
        input: RecommendationInputRef,
    },
    DeclaredInputUnused(RecommendationInputRef),
    ZeroRank,
    InvalidExpiry,
}

impl RecommendationProjection {
    pub fn validate(&self) -> Result<(), RecommendationContractError> {
        if self.recommendation_id.0.trim().is_empty() {
            return Err(RecommendationContractError::EmptyRecommendationId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(RecommendationContractError::EmptyLearnerId);
        }
        if target_is_empty(&self.target) {
            return Err(RecommendationContractError::EmptyTarget);
        }
        if self.policy.policy_id.trim().is_empty() {
            return Err(RecommendationContractError::EmptyPolicyId);
        }
        if self.policy.policy_version.trim().is_empty() {
            return Err(RecommendationContractError::EmptyPolicyVersion);
        }
        if self.policy.parameters_digest.trim().is_empty() {
            return Err(RecommendationContractError::EmptyParametersDigest);
        }
        if self.input_refs.is_empty() {
            return Err(RecommendationContractError::NoInputs);
        }
        if self.rank == 0 {
            return Err(RecommendationContractError::ZeroRank);
        }
        if self.expires_at <= self.generated_at {
            return Err(RecommendationContractError::InvalidExpiry);
        }

        let mut declared = BTreeSet::new();
        for input in &self.input_refs {
            validate_input(input)?;
            if !declared.insert(input.clone()) {
                return Err(RecommendationContractError::DuplicateInput(input.clone()));
            }
        }

        if self.signals.is_empty() {
            return Err(RecommendationContractError::NoSignals);
        }
        if self.reasons.is_empty() {
            return Err(RecommendationContractError::NoReasons);
        }

        let mut used = BTreeSet::new();
        let mut signal_kinds = BTreeSet::new();
        for signal in &self.signals {
            if let RecommendationSignalKind::Other(name) = &signal.kind {
                if name.trim().is_empty() {
                    return Err(RecommendationContractError::EmptyOtherSignalName);
                }
            }
            if !signal_kinds.insert(signal.kind.clone()) {
                return Err(RecommendationContractError::DuplicateSignalKind(
                    signal.kind.clone(),
                ));
            }
            if signal.score_permille > 1000 {
                return Err(RecommendationContractError::SignalOutOfRange {
                    kind: signal.kind.clone(),
                    value_permille: signal.score_permille,
                });
            }
            if signal.input_refs.is_empty() {
                return Err(RecommendationContractError::SignalHasNoInputs(
                    signal.kind.clone(),
                ));
            }
            let mut local = BTreeSet::new();
            for input in &signal.input_refs {
                validate_input(input)?;
                if !local.insert(input.clone()) {
                    return Err(RecommendationContractError::DuplicateSignalInput {
                        kind: signal.kind.clone(),
                        input: input.clone(),
                    });
                }
                if !declared.contains(input) {
                    return Err(RecommendationContractError::SignalInputNotDeclared {
                        kind: signal.kind.clone(),
                        input: input.clone(),
                    });
                }
                used.insert(input.clone());
            }
        }

        let mut reason_codes = BTreeSet::new();
        for reason in &self.reasons {
            if let RecommendationReasonCode::Other(name) = &reason.code {
                if name.trim().is_empty() {
                    return Err(RecommendationContractError::EmptyOtherReasonName);
                }
            }
            if !reason_codes.insert(reason.code.clone()) {
                return Err(RecommendationContractError::DuplicateReasonCode(
                    reason.code.clone(),
                ));
            }
            if reason.input_refs.is_empty() {
                return Err(RecommendationContractError::ReasonHasNoInputs(
                    reason.code.clone(),
                ));
            }
            let mut local = BTreeSet::new();
            for input in &reason.input_refs {
                validate_input(input)?;
                if !local.insert(input.clone()) {
                    return Err(RecommendationContractError::DuplicateReasonInput {
                        code: reason.code.clone(),
                        input: input.clone(),
                    });
                }
                if !declared.contains(input) {
                    return Err(RecommendationContractError::ReasonInputNotDeclared {
                        code: reason.code.clone(),
                        input: input.clone(),
                    });
                }
                used.insert(input.clone());
            }
        }

        for input in declared {
            if !used.contains(&input) {
                return Err(RecommendationContractError::DeclaredInputUnused(input));
            }
        }

        Ok(())
    }

    pub fn temporal_status_at(&self, now: i64) -> RecommendationTemporalStatus {
        if now < self.generated_at {
            RecommendationTemporalStatus::NotYetGenerated
        } else if now >= self.expires_at {
            RecommendationTemporalStatus::Expired
        } else {
            RecommendationTemporalStatus::Fresh
        }
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

fn validate_input(input: &RecommendationInputRef) -> Result<(), RecommendationContractError> {
    match input {
        RecommendationInputRef::Evidence(event_id) => {
            if event_id.0.trim().is_empty() {
                return Err(RecommendationContractError::EmptyEvidenceEventId);
            }
        }
        RecommendationInputRef::Projection(projection) => {
            if projection.projection_kind.trim().is_empty() {
                return Err(RecommendationContractError::EmptyProjectionKind);
            }
            if projection.projection_id.trim().is_empty() {
                return Err(RecommendationContractError::EmptyProjectionId);
            }
            if projection.projection_digest.trim().is_empty() {
                return Err(RecommendationContractError::EmptyProjectionDigest);
            }
        }
        RecommendationInputRef::GoalIntent(goal) => {
            if goal.goal_intent_id.0.trim().is_empty() {
                return Err(RecommendationContractError::EmptyGoalIntentId);
            }
            if goal.version == 0 {
                return Err(RecommendationContractError::ZeroGoalIntentVersion);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn goal_ref() -> RecommendationInputRef {
        RecommendationInputRef::GoalIntent(GoalIntentRef {
            goal_intent_id: GoalIntentId("goal-1".into()),
            version: 2,
        })
    }

    fn projection_ref() -> RecommendationInputRef {
        RecommendationInputRef::Projection(DerivedProjectionRef {
            projection_kind: "goal-progress".into(),
            projection_id: "goal-progress-1".into(),
            projection_digest: "blake3:abc".into(),
        })
    }

    fn recommendation() -> RecommendationProjection {
        let goal = goal_ref();
        let progress = projection_ref();
        RecommendationProjection {
            recommendation_id: RecommendationId("rec-1".into()),
            learner_id: LearnerId("learner-1".into()),
            target: RecommendationTarget::Capability(CapabilityId("rust:borrowing".into())),
            kind: RecommendationKind::Practice,
            policy: RecommendationPolicyProvenance {
                policy_id: "praxis:recommendation".into(),
                policy_version: "1".into(),
                parameters_digest: "blake3:params".into(),
            },
            input_refs: vec![goal.clone(), progress.clone()],
            signals: vec![RecommendationSignal {
                kind: RecommendationSignalKind::GoalAlignment,
                score_permille: 900,
                input_refs: vec![goal.clone(), progress.clone()],
            }],
            reasons: vec![RecommendationReason {
                code: RecommendationReasonCode::GoalAlignment,
                input_refs: vec![goal, progress],
            }],
            rank: 1,
            generated_at: 100,
            expires_at: 200,
        }
    }

    #[test]
    fn recommendation_is_advisory_and_temporally_derived() {
        let recommendation = recommendation();
        assert_eq!(recommendation.validate(), Ok(()));
        assert_eq!(
            recommendation.temporal_status_at(150),
            RecommendationTemporalStatus::Fresh
        );
        assert_eq!(
            recommendation.temporal_status_at(200),
            RecommendationTemporalStatus::Expired
        );
        assert!(!recommendation.is_source_evidence());
        assert!(!recommendation.grants_credential_authority());
        assert!(!recommendation.grants_authorization());
    }

    #[test]
    fn undeclared_signal_dependency_is_rejected() {
        let mut recommendation = recommendation();
        recommendation.signals[0]
            .input_refs
            .push(RecommendationInputRef::Evidence(EvidenceEventId("event-x".into())));
        assert!(matches!(
            recommendation.validate(),
            Err(RecommendationContractError::SignalInputNotDeclared { .. })
        ));
    }

    #[test]
    fn every_declared_dependency_must_explain_a_signal_or_reason() {
        let mut recommendation = recommendation();
        recommendation
            .input_refs
            .push(RecommendationInputRef::Evidence(EvidenceEventId("unused".into())));
        assert_eq!(
            recommendation.validate(),
            Err(RecommendationContractError::DeclaredInputUnused(
                RecommendationInputRef::Evidence(EvidenceEventId("unused".into()))
            ))
        );
    }

    #[test]
    fn duplicate_signal_kind_is_rejected() {
        let mut recommendation = recommendation();
        recommendation.signals.push(recommendation.signals[0].clone());
        assert!(matches!(
            recommendation.validate(),
            Err(RecommendationContractError::DuplicateSignalKind(_))
        ));
    }

    #[test]
    fn expiry_must_be_after_generation() {
        let mut recommendation = recommendation();
        recommendation.expires_at = recommendation.generated_at;
        assert_eq!(
            recommendation.validate(),
            Err(RecommendationContractError::InvalidExpiry)
        );
    }
}

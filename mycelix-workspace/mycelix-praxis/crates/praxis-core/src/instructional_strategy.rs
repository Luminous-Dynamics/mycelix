// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-policy-driven instructional adaptation.
//!
//! Presentation preference, accessibility, task affordance, and instructional
//! strategy are intentionally separate concepts. No preference is treated as a
//! stable aptitude or as evidence that matching presentation to preference
//! improves learning.
//!
//! Outcome observations are separate immutable observations of what was measured
//! after an assignment. They are not causal effects, learner traits, mastery,
//! credentials, trust, or authorization.

use crate::learner_profile_state::PresentationModality;
use crate::learning_evidence::LearnerId;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum AccessibilityRequirement {
    Captions,
    Transcript,
    AudioDescription,
    ScreenReaderCompatibility,
    KeyboardNavigation,
    HighContrast,
    ReducedMotion,
    AdjustableText,
    PlainLanguage,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum TaskAffordance {
    AudioPronunciation,
    ListeningComprehension,
    SpatialDiagram,
    VisualInspection,
    PhysicalManipulation,
    ProceduralSequence,
    SymbolicNotation,
    TextualCloseReading,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum InstructionalStrategyKind {
    RetrievalPractice,
    DistributedPractice,
    Interleaving,
    Elaboration,
    ConcreteExamples,
    ComplementaryRepresentations,
    WorkedExamplesAndFading,
    PracticeWithFeedback,
    TransferPractice,
    Experimental(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidencePolicyClass {
    EvidenceSupportedContextual,
    Experimental,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceBasisRef {
    pub source_id: String,
    pub source_version: String,
    pub source_digest: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidencePolicy {
    pub policy_id: String,
    pub version: u64,
    pub class: EvidencePolicyClass,
    /// Human-readable scope statement. This is a policy boundary, not a claim
    /// that the strategy is universally effective.
    pub scope: String,
    pub rationale: String,
    pub evidence_basis: Vec<EvidenceBasisRef>,
    pub evidence_review_digest: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentalProtocolRef {
    pub protocol_id: String,
    pub protocol_version: String,
    pub hypothesis_digest: String,
    pub outcome_measure_digest: String,
    pub analysis_plan_digest: String,
    pub allocation_policy: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentalAssignment {
    pub experiment_id: String,
    pub protocol: ExperimentalProtocolRef,
    pub hypothesis: String,
    pub outcome_measure: String,
    pub assignment_version: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstructionalStrategyAssignment {
    pub assignment: InstructionalAssignmentRef,
    pub learner_id: LearnerId,
    pub strategy: InstructionalStrategyKind,
    pub policy: EvidencePolicy,
    pub context_id: String,
    pub rationale: String,
    pub experimental_assignment: Option<ExperimentalAssignment>,
    pub assigned_at: i64,
}

/// Immutable reference to the exact assignment that produced an observation.
///
/// The digest closes the provenance edge even if a later assignment version is
/// superseded. An observation must never point only at a mutable logical ID.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstructionalAssignmentRef {
    pub assignment_id: String,
    pub assignment_version: u64,
    pub assignment_digest: String,
}

impl InstructionalAssignmentRef {
    fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.assignment_id.trim().is_empty()
            || self.assignment_digest.trim().is_empty()
        {
            return Err(InstructionalScienceContractError::InvalidAssignmentReference);
        }
        if self.assignment_version == 0 {
            return Err(InstructionalScienceContractError::ZeroAssignmentReferenceVersion);
        }
        Ok(())
    }
}

/// Exact reference to an earlier observation when this observation is a correction.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstructionalObservationRef {
    pub observation_id: String,
    pub observation_version: u64,
    pub observation_digest: String,
}

impl InstructionalObservationRef {
    fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.observation_id.trim().is_empty()
            || self.observation_digest.trim().is_empty()
        {
            return Err(InstructionalScienceContractError::InvalidSupersededObservation);
        }
        if self.observation_version == 0 {
            return Err(InstructionalScienceContractError::ZeroSupersededObservationVersion);
        }
        Ok(())
    }
}

/// Measurement state for an observed outcome.
///
/// Invalid and excluded observations remain explicit states so downstream
/// analysis cannot silently erase measurement history.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InstructionalOutcomeObservationStatus {
    Complete,
    Partial,
    Invalid,
    Excluded,
}

/// An immutable observation receipt linking a measured outcome to an exact
/// instructional assignment.
///
/// This is an observation contract only. It deliberately does not encode a
/// causal effect, learner capability, mastery, credential eligibility, trust,
/// or authorization. Corrections are represented by a new observation that
/// may explicitly supersede an earlier observation; historical observations
/// are never rewritten.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstructionalOutcomeObservation {
    pub observation_id: String,
    pub observation_version: u64,
    pub assignment: InstructionalAssignmentRef,
    pub learner_id: LearnerId,
    pub outcome_measure_id: String,
    pub outcome_measure_version: String,
    pub instrument_id: String,
    pub instrument_version: String,
    pub instrument_digest: String,
    pub context_id: String,
    pub observation_window_start: i64,
    pub observation_window_end: i64,
    pub observed_at: i64,
    /// Canonical serialized measurement value. Its interpretation is defined
    /// by the exact outcome-measure reference, not by this observation type.
    pub value: Option<String>,
    pub status: InstructionalOutcomeObservationStatus,
    pub status_reason: Option<String>,
    pub supersedes_observation: Option<InstructionalObservationRef>,
}

impl InstructionalOutcomeObservation {
    pub fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.observation_id.trim().is_empty()
            || self.learner_id.0.trim().is_empty()
            || self.outcome_measure_id.trim().is_empty()
            || self.outcome_measure_version.trim().is_empty()
            || self.instrument_id.trim().is_empty()
            || self.instrument_version.trim().is_empty()
            || self.instrument_digest.trim().is_empty()
            || self.context_id.trim().is_empty()
        {
            return Err(InstructionalScienceContractError::InvalidOutcomeObservation);
        }
        if self.observation_version == 0 {
            return Err(InstructionalScienceContractError::ZeroObservationVersion);
        }
        self.assignment.validate()?;
        if self.observation_window_start < 0
            || self.observation_window_end < self.observation_window_start
            || self.observed_at < self.observation_window_start
            || self.observed_at > self.observation_window_end
        {
            return Err(InstructionalScienceContractError::InvalidObservationWindow);
        }
        if let Some(previous) = &self.supersedes_observation {
            previous.validate()?;
            if previous.observation_id == self.observation_id
                && previous.observation_version == self.observation_version
            {
                return Err(InstructionalScienceContractError::SelfSupersedingObservation);
            }
        }

        match self.status {
            InstructionalOutcomeObservationStatus::Complete => {
                if self.value.as_deref().is_none_or(|value| value.trim().is_empty()) {
                    return Err(InstructionalScienceContractError::MissingObservedValue);
                }
                if self.status_reason.as_deref().is_some_and(|reason| reason.trim().is_empty()) {
                    return Err(InstructionalScienceContractError::EmptyObservationStatusReason);
                }
            }
            InstructionalOutcomeObservationStatus::Partial => {
                if self.value.as_deref().is_none_or(|value| value.trim().is_empty()) {
                    return Err(InstructionalScienceContractError::MissingObservedValue);
                }
                if self.status_reason.as_deref().is_none_or(|reason| reason.trim().is_empty()) {
                    return Err(InstructionalScienceContractError::MissingObservationStatusReason);
                }
            }
            InstructionalOutcomeObservationStatus::Invalid
            | InstructionalOutcomeObservationStatus::Excluded => {
                if self.status_reason.as_deref().is_none_or(|reason| reason.trim().is_empty()) {
                    return Err(InstructionalScienceContractError::MissingObservationStatusReason);
                }
            }
        }
        Ok(())
    }

    pub const fn grants_credential_authority(&self) -> bool { false }
    pub const fn grants_trust_authority(&self) -> bool { false }
    pub const fn grants_authorization(&self) -> bool { false }
}

/// Classification of an analysis receipt. The label describes the computation
/// performed; it is not an authority or a claim of causal truth.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InstructionalAnalysisKind {
    Descriptive,
    Comparative,
    ExperimentalEffectEstimate,
    Exploratory,
}

/// Exact protocol provenance required when an analysis is presented as an
/// experimental effect estimate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentalAnalysisRef {
    pub experiment_id: String,
    pub protocol_id: String,
    pub protocol_version: String,
    pub protocol_digest: String,
}

impl ExperimentalAnalysisRef {
    fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.experiment_id.trim().is_empty()
            || self.protocol_id.trim().is_empty()
            || self.protocol_version.trim().is_empty()
            || self.protocol_digest.trim().is_empty()
        {
            return Err(InstructionalScienceContractError::InvalidExperimentalAnalysisReference);
        }
        Ok(())
    }
}

/// A reproducible, immutable receipt for an analysis over outcome observations.
///
/// The receipt binds the computation to exact plans, inputs, cohort rules,
/// methods, and outputs. It never rewrites observations and never turns an
/// analysis result into a learner capability, mastery, credential, trust, or
/// authorization claim.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstructionalAnalysisReceipt {
    pub analysis_id: String,
    pub analysis_version: u64,
    pub analysis_plan_id: String,
    pub analysis_plan_version: String,
    pub analysis_plan_digest: String,
    pub outcome_measure_id: String,
    pub outcome_measure_version: String,
    pub input_observation_set_digest: String,
    pub cohort_definition_digest: String,
    pub inclusion_rules_digest: String,
    pub exclusion_rules_digest: String,
    pub missing_data_policy: String,
    pub analysis_method: String,
    pub analysis_method_version: String,
    pub estimand: String,
    pub kind: InstructionalAnalysisKind,
    pub experimental_provenance: Option<ExperimentalAnalysisRef>,
    pub prespecified: bool,
    pub deviation_rationale: Option<String>,
    pub result_digest: String,
    pub generated_at: i64,
}

impl InstructionalAnalysisReceipt {
    pub fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.analysis_id.trim().is_empty()
            || self.analysis_plan_id.trim().is_empty()
            || self.analysis_plan_version.trim().is_empty()
            || self.analysis_plan_digest.trim().is_empty()
            || self.outcome_measure_id.trim().is_empty()
            || self.outcome_measure_version.trim().is_empty()
            || self.input_observation_set_digest.trim().is_empty()
            || self.cohort_definition_digest.trim().is_empty()
            || self.inclusion_rules_digest.trim().is_empty()
            || self.exclusion_rules_digest.trim().is_empty()
            || self.missing_data_policy.trim().is_empty()
            || self.analysis_method.trim().is_empty()
            || self.analysis_method_version.trim().is_empty()
            || self.estimand.trim().is_empty()
            || self.result_digest.trim().is_empty()
        {
            return Err(InstructionalScienceContractError::InvalidAnalysisReceipt);
        }
        if self.analysis_version == 0 {
            return Err(InstructionalScienceContractError::ZeroAnalysisVersion);
        }
        if matches!(self.kind, InstructionalAnalysisKind::ExperimentalEffectEstimate) {
            self.experimental_provenance.as_ref().ok_or(
                InstructionalScienceContractError::MissingExperimentalAnalysisReference,
            )?.validate()?;
        } else if let Some(provenance) = &self.experimental_provenance {
            provenance.validate()?;
        }
        if self.generated_at < 0 {
            return Err(InstructionalScienceContractError::NegativeAnalysisGeneratedAt);
        }
        if self.prespecified {
            if self.deviation_rationale.as_deref().is_some_and(|r| r.trim().is_empty()) {
                return Err(InstructionalScienceContractError::EmptyAnalysisDeviationRationale);
            }
        } else if self.deviation_rationale.as_deref().is_none_or(|r| r.trim().is_empty()) {
            return Err(InstructionalScienceContractError::MissingAnalysisDeviationRationale);
        }
        Ok(())
    }

    pub const fn grants_credential_authority(&self) -> bool { false }
    pub const fn grants_trust_authority(&self) -> bool { false }
    pub const fn grants_authorization(&self) -> bool { false }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstructionalAdaptationContext {
    pub context_id: String,
    pub presentation_preferences: Vec<PresentationModality>,
    pub accessibility_requirements: Vec<AccessibilityRequirement>,
    pub task_affordances: Vec<TaskAffordance>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InstructionalScienceContractError {
    EmptyId,
    ZeroVersion,
    EmptyScope,
    EmptyRationale,
    NoEvidenceBasis,
    InvalidEvidenceBasis,
    EmptyEvidenceReviewDigest,
    InvalidExperimentalProtocol,
    DuplicateAccessibilityRequirement,
    EmptyAccessibilityOther,
    DuplicateTaskAffordance,
    EmptyTaskAffordanceOther,
    DuplicatePresentationPreference,
    EmptyAssignmentContext,
    ExperimentalStrategyRequiresExperimentalPolicy,
    ExperimentalPolicyRequiresExperimentAssignment,
    ExperimentalAssignmentMissingHypothesis,
    ExperimentalAssignmentMissingOutcome,
    ZeroExperimentAssignmentVersion,
    NegativeAssignedAt,
    InvalidOutcomeObservation,
    ZeroObservationVersion,
    InvalidObservationWindow,
    MissingObservedValue,
    MissingObservationStatusReason,
    EmptyObservationStatusReason,
    SelfSupersedingObservation,
    InvalidSupersededObservation,
    InvalidAssignmentReference,
    ZeroAssignmentReferenceVersion,
    ZeroSupersededObservationVersion,
    InvalidExperimentalAnalysisReference,
    MissingExperimentalAnalysisReference,
}

impl EvidencePolicy {
    pub fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.policy_id.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyId);
        }
        if self.version == 0 {
            return Err(InstructionalScienceContractError::ZeroVersion);
        }
        if self.scope.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyScope);
        }
        if self.rationale.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyRationale);
        }
        if self.evidence_basis.is_empty() {
            return Err(InstructionalScienceContractError::NoEvidenceBasis);
        }
        let mut refs = BTreeSet::new();
        for evidence in &self.evidence_basis {
            if evidence.source_id.trim().is_empty()
                || evidence.source_version.trim().is_empty()
                || evidence.source_digest.trim().is_empty()
            {
                return Err(InstructionalScienceContractError::InvalidEvidenceBasis);
            }
            if !refs.insert((
                evidence.source_id.clone(),
                evidence.source_version.clone(),
                evidence.source_digest.clone(),
            )) {
                return Err(InstructionalScienceContractError::InvalidEvidenceBasis);
            }
        }
        if self.evidence_review_digest.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyEvidenceReviewDigest);
        }
        Ok(())
    }
}

impl ExperimentalProtocolRef {
    pub fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.protocol_id.trim().is_empty()
            || self.protocol_version.trim().is_empty()
            || self.hypothesis_digest.trim().is_empty()
            || self.outcome_measure_digest.trim().is_empty()
            || self.analysis_plan_digest.trim().is_empty()
            || self.allocation_policy.trim().is_empty()
        {
            return Err(InstructionalScienceContractError::InvalidExperimentalProtocol);
        }
        Ok(())
    }
}

impl ExperimentalAssignment {
    pub fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        self.protocol.validate()?;
        if self.experiment_id.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyId);
        }
        if self.hypothesis.trim().is_empty() {
            return Err(InstructionalScienceContractError::ExperimentalAssignmentMissingHypothesis);
        }
        if self.outcome_measure.trim().is_empty() {
            return Err(InstructionalScienceContractError::ExperimentalAssignmentMissingOutcome);
        }
        if self.assignment_version == 0 {
            return Err(InstructionalScienceContractError::ZeroExperimentAssignmentVersion);
        }
        Ok(())
    }
}

impl InstructionalStrategyAssignment {
    pub fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.assignment_id.trim().is_empty() || self.learner_id.0.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyId);
        }
        self.policy.validate()?;
        if self.context_id.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyAssignmentContext);
        }
        if self.rationale.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyRationale);
        }
        if self.assigned_at < 0 {
            return Err(InstructionalScienceContractError::NegativeAssignedAt);
        }
        let experimental_strategy =
            matches!(&self.strategy, InstructionalStrategyKind::Experimental(_));
        let experimental_policy =
            matches!(&self.policy.class, EvidencePolicyClass::Experimental);
        if experimental_strategy && !experimental_policy {
            return Err(
                InstructionalScienceContractError::ExperimentalStrategyRequiresExperimentalPolicy,
            );
        }
        if experimental_policy && self.experimental_assignment.is_none() {
            return Err(
                InstructionalScienceContractError::ExperimentalPolicyRequiresExperimentAssignment,
            );
        }
        if let Some(experiment) = &self.experimental_assignment {
            experiment.validate()?;
        }
        Ok(())
    }

    pub const fn grants_credential_authority(&self) -> bool { false }
    pub const fn grants_trust_authority(&self) -> bool { false }
    pub const fn grants_authorization(&self) -> bool { false }
}

impl InstructionalAdaptationContext {
    pub fn validate(&self) -> Result<(), InstructionalScienceContractError> {
        if self.context_id.trim().is_empty() {
            return Err(InstructionalScienceContractError::EmptyId);
        }
        let mut preferences = BTreeSet::new();
        for preference in &self.presentation_preferences {
            if !preferences.insert(preference.clone()) {
                return Err(InstructionalScienceContractError::DuplicatePresentationPreference);
            }
        }
        let mut accessibility = BTreeSet::new();
        for requirement in &self.accessibility_requirements {
            if let AccessibilityRequirement::Other(value) = requirement {
                if value.trim().is_empty() {
                    return Err(InstructionalScienceContractError::EmptyAccessibilityOther);
                }
            }
            if !accessibility.insert(requirement.clone()) {
                return Err(InstructionalScienceContractError::DuplicateAccessibilityRequirement);
            }
        }
        let mut affordances = BTreeSet::new();
        for affordance in &self.task_affordances {
            if let TaskAffordance::Other(value) = affordance {
                if value.trim().is_empty() {
                    return Err(InstructionalScienceContractError::EmptyTaskAffordanceOther);
                }
            }
            if !affordances.insert(affordance.clone()) {
                return Err(InstructionalScienceContractError::DuplicateTaskAffordance);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn policy(class: EvidencePolicyClass) -> EvidencePolicy {
        EvidencePolicy {
            policy_id: "praxis:retrieval-context-v1".into(),
            version: 1,
            class,
            scope: "Use when retrieval is appropriate to the task and material.".into(),
            rationale: "Named evidence policy; not a universal efficacy claim.".into(),
            evidence_basis: vec![EvidenceBasisRef {
                source_id: "source:retrieval-review".into(),
                source_version: "1".into(),
                source_digest: "blake3:evidence".into(),
            }],
            evidence_review_digest: "blake3:policy-review".into(),
        }
    }

    fn observation(status: InstructionalOutcomeObservationStatus) -> InstructionalOutcomeObservation {
        InstructionalOutcomeObservation {
            observation_id: "observation-1".into(),
            observation_version: 1,
            assignment: InstructionalAssignmentRef {
                assignment_id: "assignment-1".into(),
                assignment_version: 1,
                assignment_digest: "blake3:assignment".into(),
            },
            learner_id: LearnerId("learner-1".into()),
            outcome_measure_id: "measure:delayed-retrieval".into(),
            outcome_measure_version: "1".into(),
            instrument_id: "instrument:quiz".into(),
            instrument_version: "1".into(),
            instrument_digest: "blake3:instrument".into(),
            context_id: "lesson-1".into(),
            observation_window_start: 100,
            observation_window_end: 200,
            observed_at: 150,
            value: Some("0.82".into()),
            status,
            status_reason: None,
            supersedes_observation: None,
        }
    }

    #[test]
    fn preference_is_not_strategy_evidence() {
        let context = InstructionalAdaptationContext {
            context_id: "lesson-1".into(),
            presentation_preferences: vec![PresentationModality::Visual],
            accessibility_requirements: vec![AccessibilityRequirement::Captions],
            task_affordances: vec![TaskAffordance::SpatialDiagram],
        };
        assert_eq!(context.validate(), Ok(()));
    }

    #[test]
    fn contextual_strategy_requires_named_policy() {
        let assignment = InstructionalStrategyAssignment {
            assignment_id: "assignment-1".into(),
            learner_id: LearnerId("learner-1".into()),
            strategy: InstructionalStrategyKind::RetrievalPractice,
            policy: policy(EvidencePolicyClass::EvidenceSupportedContextual),
            context_id: "lesson-1".into(),
            rationale: "Task requires active recall of previously introduced material.".into(),
            experimental_assignment: None,
            assigned_at: 100,
        };
        assert_eq!(assignment.validate(), Ok(()));
        assert!(!assignment.grants_credential_authority());
        assert!(!assignment.grants_trust_authority());
        assert!(!assignment.grants_authorization());
    }

    #[test]
    fn experimental_matching_cannot_hide_as_default_personalization() {
        let assignment = InstructionalStrategyAssignment {
            assignment_id: "assignment-2".into(),
            learner_id: LearnerId("learner-1".into()),
            strategy: InstructionalStrategyKind::Experimental("preference-match-v1".into()),
            policy: policy(EvidencePolicyClass::Experimental),
            context_id: "lesson-1".into(),
            rationale: "Pre-registered comparison of a presentation preference hypothesis.".into(),
            experimental_assignment: Some(ExperimentalAssignment {
                experiment_id: "exp-1".into(),
                protocol: ExperimentalProtocolRef {
                    protocol_id: "protocol:preference-match".into(),
                    protocol_version: "1".into(),
                    hypothesis_digest: "blake3:hypothesis".into(),
                    outcome_measure_digest: "blake3:outcome".into(),
                    analysis_plan_digest: "blake3:analysis".into(),
                    allocation_policy: "explicit-experimental-assignment-v1".into(),
                },
                hypothesis: "Matching may improve this task outcome under this protocol.".into(),
                outcome_measure: "delayed_retrieval_score".into(),
                assignment_version: 1,
            }),
            assigned_at: 100,
        };
        assert_eq!(assignment.validate(), Ok(()));
    }

    #[test]
    fn experimental_strategy_without_experimental_policy_is_rejected() {
        let assignment = InstructionalStrategyAssignment {
            assignment_id: "assignment-3".into(),
            learner_id: LearnerId("learner-1".into()),
            strategy: InstructionalStrategyKind::Experimental("preference-match-v1".into()),
            policy: policy(EvidencePolicyClass::EvidenceSupportedContextual),
            context_id: "lesson-1".into(),
            rationale: "Attempt to treat an experiment as ordinary personalization.".into(),
            experimental_assignment: None,
            assigned_at: 100,
        };
        assert_eq!(
            assignment.validate(),
            Err(InstructionalScienceContractError::ExperimentalStrategyRequiresExperimentalPolicy)
        );
    }

    #[test]
    fn complete_outcome_requires_exact_measurement_provenance() {
        assert_eq!(observation(InstructionalOutcomeObservationStatus::Complete).validate(), Ok(()));
    }

    #[test]
    fn invalid_outcome_requires_explicit_reason() {
        let mut value = observation(InstructionalOutcomeObservationStatus::Invalid);
        value.value = None;
        assert_eq!(
            value.validate(),
            Err(InstructionalScienceContractError::MissingObservationStatusReason)
        );
        value.status_reason = Some("instrument failure".into());
        assert_eq!(value.validate(), Ok(()));
    }

    #[test]
    fn outcome_window_and_timestamp_are_bounded() {
        let mut value = observation(InstructionalOutcomeObservationStatus::Complete);
        value.observed_at = 201;
        assert_eq!(
            value.validate(),
            Err(InstructionalScienceContractError::InvalidObservationWindow)
        );
    }

    #[test]
    fn outcome_cannot_self_supersede() {
        let mut value = observation(InstructionalOutcomeObservationStatus::Complete);
        value.supersedes_observation_id = Some("observation-1".into());
        assert_eq!(
            value.validate(),
            Err(InstructionalScienceContractError::SelfSupersedingObservation)
        );
    }

    #[test]
    fn observation_requires_exact_assignment_provenance() {
        let mut value = observation(InstructionalOutcomeObservationStatus::Complete);
        value.assignment.assignment_digest.clear();
        assert_eq!(
            value.validate(),
            Err(InstructionalScienceContractError::InvalidAssignmentReference)
        );
    }

    #[test]
    fn partial_outcome_requires_reason() {
        let mut value = observation(InstructionalOutcomeObservationStatus::Partial);
        assert_eq!(
            value.validate(),
            Err(InstructionalScienceContractError::MissingObservationStatusReason)
        );
        value.status_reason = Some("assessment interrupted before final item".into());
        assert_eq!(value.validate(), Ok(()));
    }

    #[test]
    fn experimental_effect_estimate_requires_protocol_provenance() {
        let receipt = InstructionalAnalysisReceipt {
            analysis_id: "analysis-1".into(),
            analysis_version: 1,
            analysis_plan_id: "plan-1".into(),
            analysis_plan_version: "1".into(),
            analysis_plan_digest: "blake3:plan".into(),
            outcome_measure_id: "measure-1".into(),
            outcome_measure_version: "1".into(),
            input_observation_set_digest: "blake3:observations".into(),
            cohort_definition_digest: "blake3:cohort".into(),
            inclusion_rules_digest: "blake3:include".into(),
            exclusion_rules_digest: "blake3:exclude".into(),
            missing_data_policy: "complete-case-v1".into(),
            analysis_method: "difference-in-means".into(),
            analysis_method_version: "1".into(),
            estimand: "mean outcome difference".into(),
            kind: InstructionalAnalysisKind::ExperimentalEffectEstimate,
            experimental_provenance: None,
            prespecified: true,
            deviation_rationale: None,
            result_digest: "blake3:result".into(),
            generated_at: 200,
        };
        assert_eq!(
            receipt.validate(),
            Err(InstructionalScienceContractError::MissingExperimentalAnalysisReference)
        );
    }

    #[test]
    fn outcome_observation_has_no_authority() {
        let value = observation(InstructionalOutcomeObservationStatus::Complete);
        assert!(!value.grants_credential_authority());
        assert!(!value.grants_trust_authority());
        assert!(!value.grants_authorization());
    }
}

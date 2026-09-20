// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Authority-safe learning evidence primitives.
//!
//! The central invariant is deliberately stronger than ordinary progress tracking:
//!
//! ```text
//! observation != admitted evidence != capability estimate != credential decision
//! ```
//!
//! This module only models the first three layers. Credential issuance remains a
//! separate authority boundary and MUST NOT be inferred from an estimate alone.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Stable identifier for one immutable learning observation.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct EvidenceEventId(pub String);

/// Opaque learner identifier. It intentionally carries no PII semantics.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct LearnerId(pub String);

/// Stable identifier for a capability, skill, knowledge node, or competency.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CapabilityId(pub String);

/// Independent dimensions of capability. A UI may summarize these, but a summary
/// must not silently replace the underlying dimensional evidence.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum CapabilityDimension {
    Recall,
    Understanding,
    Application,
    Transfer,
    Explanation,
    PracticalPerformance,
    Retention,
    Judgment,
    Collaboration,
    Other(String),
}

/// Where an observation originated. Source class is evidence metadata, not an
/// intrinsic quality or authority ranking.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceSource {
    SelfReport,
    Practice,
    Assessment,
    Project,
    RetentionProbe,
    TransferTask,
    PeerAttestation,
    MentorAttestation,
    ImportedCredential,
    SystemObservation,
    Other(String),
}

/// Assistance present while the observation was produced.
///
/// No variant is globally "good" or "bad". Admission policies decide what kinds
/// of assistance are appropriate for a specific claim. Most importantly, direct
/// assistance remains visible instead of being erased by a mastery estimate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AssistanceKind {
    Unassisted,
    Hint,
    Scaffolded,
    Collaborative,
    ToolAssisted,
    DirectAnswer,
    Unknown,
}

/// One measured dimension on an observation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceMeasure {
    pub dimension: CapabilityDimension,
    /// Normalized measurement in 0..=1000. This is an observed/derived measure,
    /// not a probability that the learner "has mastered" the capability.
    pub value_permille: u16,
}

/// Provenance sufficient to trace an observation back to the producing surface.
/// Hash fields are opaque digests; this type does not claim they were independently
/// verified.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceProvenance {
    pub producer_id: String,
    pub producer_version: Option<String>,
    pub source_record_id: Option<String>,
    pub artifact_digest: Option<String>,
}

/// Immutable source observation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearningEvidenceEvent {
    pub event_id: EvidenceEventId,
    pub learner_id: LearnerId,
    pub capability_id: CapabilityId,
    pub source: EvidenceSource,
    pub assistance: AssistanceKind,
    pub measures: Vec<EvidenceMeasure>,
    /// Unix timestamp in seconds supplied by the producing boundary.
    pub observed_at: i64,
    pub provenance: EvidenceProvenance,
    /// Context labels are descriptive only; they do not grant authority.
    pub context_tags: Vec<String>,
}

/// Validation errors for the semantic contract.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvidenceContractError {
    EmptyEventId,
    EmptyLearnerId,
    EmptyCapabilityId,
    EmptyProducerId,
    NoMeasures,
    MeasureOutOfRange { value_permille: u16 },
    EmptyAdmissionProfile,
    EmptyEstimatorId,
    EmptyEstimatorVersion,
    NoEstimateInputs,
    NoDimensionEstimates,
    DuplicateDimension,
    ConfidenceOutOfRange { confidence_permille: u16 },
}

impl LearningEvidenceEvent {
    /// Validate structural invariants only. Passing validation means the event is
    /// well-formed; it does NOT mean the event is true, sufficient, or admitted.
    pub fn validate(&self) -> Result<(), EvidenceContractError> {
        if self.event_id.0.trim().is_empty() {
            return Err(EvidenceContractError::EmptyEventId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(EvidenceContractError::EmptyLearnerId);
        }
        if self.capability_id.0.trim().is_empty() {
            return Err(EvidenceContractError::EmptyCapabilityId);
        }
        if self.provenance.producer_id.trim().is_empty() {
            return Err(EvidenceContractError::EmptyProducerId);
        }
        if self.measures.is_empty() {
            return Err(EvidenceContractError::NoMeasures);
        }
        for measure in &self.measures {
            if measure.value_permille > 1000 {
                return Err(EvidenceContractError::MeasureOutOfRange {
                    value_permille: measure.value_permille,
                });
            }
        }
        Ok(())
    }
}

/// Explicit outcome of applying a named admission policy to one event.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdmissionOutcome {
    Admitted,
    Rejected { reason: String },
    Inconclusive { reason: String },
}

/// Policy-relative admission decision.
///
/// Admission does not mutate the source event and does not assert universal truth.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceAdmissionDecision {
    pub event_id: EvidenceEventId,
    pub profile_id: String,
    pub profile_version: String,
    pub outcome: AdmissionOutcome,
    pub decided_at: i64,
}

impl EvidenceAdmissionDecision {
    pub fn validate(&self) -> Result<(), EvidenceContractError> {
        if self.profile_id.trim().is_empty() || self.profile_version.trim().is_empty() {
            return Err(EvidenceContractError::EmptyAdmissionProfile);
        }
        Ok(())
    }
}

/// Provenance of an estimator that transforms admitted evidence into an advisory
/// capability estimate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EstimatorProvenance {
    pub estimator_id: String,
    pub estimator_version: String,
    pub parameters_digest: Option<String>,
}

/// One model-derived dimensional estimate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DimensionEstimate {
    pub dimension: CapabilityDimension,
    pub estimate_permille: u16,
    pub confidence_permille: u16,
}

/// A recomputable, advisory projection over explicitly referenced evidence.
///
/// This type intentionally has no `mastered`, `credentialed`, `passed`, or
/// `authorized` field. Consumers that need such a decision must cross a separate,
/// explicit policy boundary.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdvisoryCapabilityEstimate {
    pub learner_id: LearnerId,
    pub capability_id: CapabilityId,
    pub estimator: EstimatorProvenance,
    pub admission_profile_id: String,
    pub admission_profile_version: String,
    pub input_event_ids: Vec<EvidenceEventId>,
    pub dimensions: Vec<DimensionEstimate>,
    pub generated_at: i64,
}

impl AdvisoryCapabilityEstimate {
    pub fn validate(&self) -> Result<(), EvidenceContractError> {
        if self.learner_id.0.trim().is_empty() {
            return Err(EvidenceContractError::EmptyLearnerId);
        }
        if self.capability_id.0.trim().is_empty() {
            return Err(EvidenceContractError::EmptyCapabilityId);
        }
        if self.estimator.estimator_id.trim().is_empty() {
            return Err(EvidenceContractError::EmptyEstimatorId);
        }
        if self.estimator.estimator_version.trim().is_empty() {
            return Err(EvidenceContractError::EmptyEstimatorVersion);
        }
        if self.admission_profile_id.trim().is_empty()
            || self.admission_profile_version.trim().is_empty()
        {
            return Err(EvidenceContractError::EmptyAdmissionProfile);
        }
        if self.input_event_ids.is_empty() {
            return Err(EvidenceContractError::NoEstimateInputs);
        }
        if self.dimensions.is_empty() {
            return Err(EvidenceContractError::NoDimensionEstimates);
        }

        let mut seen = BTreeSet::new();
        for estimate in &self.dimensions {
            if estimate.estimate_permille > 1000 {
                return Err(EvidenceContractError::MeasureOutOfRange {
                    value_permille: estimate.estimate_permille,
                });
            }
            if estimate.confidence_permille > 1000 {
                return Err(EvidenceContractError::ConfidenceOutOfRange {
                    confidence_permille: estimate.confidence_permille,
                });
            }
            if !seen.insert(estimate.dimension.clone()) {
                return Err(EvidenceContractError::DuplicateDimension);
            }
        }
        Ok(())
    }

    /// Capability estimates are always advisory at this layer.
    pub const fn grants_credential_authority(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example_event() -> LearningEvidenceEvent {
        LearningEvidenceEvent {
            event_id: EvidenceEventId("event-1".into()),
            learner_id: LearnerId("learner-1".into()),
            capability_id: CapabilityId("cap-rust-borrowing".into()),
            source: EvidenceSource::Assessment,
            assistance: AssistanceKind::DirectAnswer,
            measures: vec![EvidenceMeasure {
                dimension: CapabilityDimension::Application,
                value_permille: 820,
            }],
            observed_at: 1_700_000_000,
            provenance: EvidenceProvenance {
                producer_id: "praxis-assessment-v1".into(),
                producer_version: Some("1.0.0".into()),
                source_record_id: Some("attempt-44".into()),
                artifact_digest: Some("sha256:example".into()),
            },
            context_tags: vec!["practice-mode".into()],
        }
    }

    #[test]
    fn well_formed_observation_is_not_an_admission() {
        let event = example_event();
        assert_eq!(event.validate(), Ok(()));

        let decision = EvidenceAdmissionDecision {
            event_id: event.event_id.clone(),
            profile_id: "credential-rust-v1".into(),
            profile_version: "1".into(),
            outcome: AdmissionOutcome::Inconclusive {
                reason: "direct assistance requires independent evidence".into(),
            },
            decided_at: event.observed_at + 1,
        };

        assert_eq!(decision.validate(), Ok(()));
        assert_eq!(event.assistance, AssistanceKind::DirectAnswer);
        assert!(matches!(decision.outcome, AdmissionOutcome::Inconclusive { .. }));
    }

    #[test]
    fn evidence_measure_must_be_normalized() {
        let mut event = example_event();
        event.measures[0].value_permille = 1001;
        assert_eq!(
            event.validate(),
            Err(EvidenceContractError::MeasureOutOfRange {
                value_permille: 1001
            })
        );
    }

    #[test]
    fn advisory_estimate_references_inputs_and_never_grants_credential_authority() {
        let estimate = AdvisoryCapabilityEstimate {
            learner_id: LearnerId("learner-1".into()),
            capability_id: CapabilityId("cap-rust-borrowing".into()),
            estimator: EstimatorProvenance {
                estimator_id: "bkt".into(),
                estimator_version: "legacy-v1".into(),
                parameters_digest: None,
            },
            admission_profile_id: "practice-evidence-v1".into(),
            admission_profile_version: "1".into(),
            input_event_ids: vec![EvidenceEventId("event-1".into())],
            dimensions: vec![DimensionEstimate {
                dimension: CapabilityDimension::Application,
                estimate_permille: 760,
                confidence_permille: 650,
            }],
            generated_at: 1_700_000_001,
        };

        assert_eq!(estimate.validate(), Ok(()));
        assert!(!estimate.grants_credential_authority());
    }

    #[test]
    fn duplicate_dimension_estimates_are_rejected() {
        let dimension = CapabilityDimension::Retention;
        let estimate = AdvisoryCapabilityEstimate {
            learner_id: LearnerId("learner-1".into()),
            capability_id: CapabilityId("cap-1".into()),
            estimator: EstimatorProvenance {
                estimator_id: "retention-model".into(),
                estimator_version: "1".into(),
                parameters_digest: None,
            },
            admission_profile_id: "profile".into(),
            admission_profile_version: "1".into(),
            input_event_ids: vec![EvidenceEventId("event-1".into())],
            dimensions: vec![
                DimensionEstimate {
                    dimension: dimension.clone(),
                    estimate_permille: 700,
                    confidence_permille: 500,
                },
                DimensionEstimate {
                    dimension,
                    estimate_permille: 710,
                    confidence_permille: 520,
                },
            ],
            generated_at: 1,
        };

        assert_eq!(estimate.validate(), Err(EvidenceContractError::DuplicateDimension));
    }
}

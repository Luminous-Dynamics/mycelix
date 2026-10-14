// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Provenance-complete attempt observations for Praxis.
//!
//! This module refines the generic learning-evidence contract for the common
//! "learner attempted a task" case. It deliberately keeps legacy observations
//! incomplete rather than inventing task identity, capability dimensions,
//! assistance status, evaluator identity, or trace references that were never
//! recorded.
//!
//! Core invariant:
//!
//! ```text
//! attempt observed != admitted evidence != capability estimate != credential
//! ```

use crate::learning_evidence::{
    AssistanceKind, CapabilityDimension, CapabilityId, EvidenceEventId, EvidenceMeasure,
    EvidenceProvenance, EvidenceSource, LearnerId, LearningEvidenceEvent,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Truthful identity of the compatibility boundary that can wrap the historical
/// `record_attempt(correct, response_time_ms)` API. This identifies the adapter,
/// not the original evaluator (which legacy calls did not record).
pub const LEGACY_RECORD_ATTEMPT_ADAPTER_ID: &str =
    "praxis-adaptive:legacy-record-attempt-adapter";

/// What the producing surface observed for one attempt.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AttemptOutcome {
    Binary { correct: bool },
    Scored { score_permille: u16 },
}

impl AttemptOutcome {
    pub fn value_permille(&self) -> Result<u16, AttemptEvidenceContractError> {
        match self {
            Self::Binary { correct } => Ok(if *correct { 1000 } else { 0 }),
            Self::Scored { score_permille } if *score_permille <= 1000 => Ok(*score_permille),
            Self::Scored { score_permille } => {
                Err(AttemptEvidenceContractError::OutcomeOutOfRange {
                    score_permille: *score_permille,
                })
            }
        }
    }
}

/// Fields that an older observation never captured or cannot prove.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum MissingAttemptEvidenceField {
    SourceActivity,
    CapabilityDimension,
    Assistance,
    OriginalEvaluator,
    TraceReference,
}

/// Whether the source observation has the fields needed for the v1 attempt
/// contract. Legacy data remains useful, but incompleteness is explicit.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AttemptEvidenceCompleteness {
    Complete,
    LegacyIncomplete {
        missing_fields: Vec<MissingAttemptEvidenceField>,
    },
}

/// One source attempt observation.
///
/// A valid value says only that the observation contract is internally
/// consistent. It does not establish that the observation is truthful,
/// independently proctored, admitted by a policy, or sufficient for mastery.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AttemptEvidence {
    pub event_id: EvidenceEventId,
    pub learner_id: LearnerId,
    pub capability_id: CapabilityId,
    /// Stable task/activity/assessment identifier when the producer recorded one.
    pub source_activity_id: Option<String>,
    pub source: EvidenceSource,
    /// Which capability dimension the attempt actually measured. Historical
    /// `record_attempt` calls did not record this and MUST NOT guess it.
    pub dimension: Option<CapabilityDimension>,
    pub outcome: AttemptOutcome,
    pub response_time_ms: Option<u32>,
    pub assistance: AssistanceKind,
    /// Provenance of the boundary producing this record. For legacy wrapping,
    /// this is the compatibility adapter, not a fabricated original evaluator.
    pub provenance: EvidenceProvenance,
    pub completeness: AttemptEvidenceCompleteness,
    pub observed_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AttemptEvidenceContractError {
    EmptyEventId,
    EmptyLearnerId,
    EmptyCapabilityId,
    EmptySourceActivityId,
    EmptyProducerId,
    MissingProducerVersion,
    EmptyProducerVersion,
    EmptySourceRecordId,
    EmptyArtifactDigest,
    OutcomeOutOfRange { score_permille: u16 },
    CompleteMissingSourceActivity,
    CompleteMissingCapabilityDimension,
    CompleteUnknownAssistance,
    CompleteMissingTraceReference,
    LegacyMissingFieldListEmpty,
    DuplicateMissingField(MissingAttemptEvidenceField),
    UndeclaredMissingField(MissingAttemptEvidenceField),
    DeclaredMissingFieldIsPresent(MissingAttemptEvidenceField),
    IncompleteCannotPromote,
}

impl AttemptEvidence {
    /// Validate the attempt source contract without making an admission decision.
    pub fn validate(&self) -> Result<(), AttemptEvidenceContractError> {
        if self.event_id.0.trim().is_empty() {
            return Err(AttemptEvidenceContractError::EmptyEventId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(AttemptEvidenceContractError::EmptyLearnerId);
        }
        if self.capability_id.0.trim().is_empty() {
            return Err(AttemptEvidenceContractError::EmptyCapabilityId);
        }
        if let Some(source_activity_id) = &self.source_activity_id {
            if source_activity_id.trim().is_empty() {
                return Err(AttemptEvidenceContractError::EmptySourceActivityId);
            }
        }
        if self.provenance.producer_id.trim().is_empty() {
            return Err(AttemptEvidenceContractError::EmptyProducerId);
        }
        match &self.provenance.producer_version {
            None => return Err(AttemptEvidenceContractError::MissingProducerVersion),
            Some(version) if version.trim().is_empty() => {
                return Err(AttemptEvidenceContractError::EmptyProducerVersion)
            }
            Some(_) => {}
        }
        if self
            .provenance
            .source_record_id
            .as_ref()
            .is_some_and(|value| value.trim().is_empty())
        {
            return Err(AttemptEvidenceContractError::EmptySourceRecordId);
        }
        if self
            .provenance
            .artifact_digest
            .as_ref()
            .is_some_and(|value| value.trim().is_empty())
        {
            return Err(AttemptEvidenceContractError::EmptyArtifactDigest);
        }

        self.outcome.value_permille()?;

        match &self.completeness {
            AttemptEvidenceCompleteness::Complete => {
                if self.source_activity_id.is_none() {
                    return Err(AttemptEvidenceContractError::CompleteMissingSourceActivity);
                }
                if self.dimension.is_none() {
                    return Err(
                        AttemptEvidenceContractError::CompleteMissingCapabilityDimension,
                    );
                }
                if self.assistance == AssistanceKind::Unknown {
                    return Err(AttemptEvidenceContractError::CompleteUnknownAssistance);
                }
                if self.provenance.source_record_id.is_none()
                    && self.provenance.artifact_digest.is_none()
                {
                    return Err(AttemptEvidenceContractError::CompleteMissingTraceReference);
                }
            }
            AttemptEvidenceCompleteness::LegacyIncomplete { missing_fields } => {
                if missing_fields.is_empty() {
                    return Err(AttemptEvidenceContractError::LegacyMissingFieldListEmpty);
                }

                let mut seen = BTreeSet::new();
                for field in missing_fields {
                    if !seen.insert(field.clone()) {
                        return Err(AttemptEvidenceContractError::DuplicateMissingField(
                            field.clone(),
                        ));
                    }
                }

                Self::validate_legacy_field(
                    &seen,
                    MissingAttemptEvidenceField::SourceActivity,
                    self.source_activity_id.is_none(),
                )?;
                Self::validate_legacy_field(
                    &seen,
                    MissingAttemptEvidenceField::CapabilityDimension,
                    self.dimension.is_none(),
                )?;
                Self::validate_legacy_field(
                    &seen,
                    MissingAttemptEvidenceField::Assistance,
                    self.assistance == AssistanceKind::Unknown,
                )?;
                Self::validate_legacy_field(
                    &seen,
                    MissingAttemptEvidenceField::TraceReference,
                    self.provenance.source_record_id.is_none()
                        && self.provenance.artifact_digest.is_none(),
                )?;
            }
        }

        Ok(())
    }

    fn validate_legacy_field(
        missing_fields: &BTreeSet<MissingAttemptEvidenceField>,
        field: MissingAttemptEvidenceField,
        is_missing: bool,
    ) -> Result<(), AttemptEvidenceContractError> {
        match (is_missing, missing_fields.contains(&field)) {
            (true, false) => Err(AttemptEvidenceContractError::UndeclaredMissingField(field)),
            (false, true) => Err(
                AttemptEvidenceContractError::DeclaredMissingFieldIsPresent(field),
            ),
            _ => Ok(()),
        }
    }

    /// Convert only provenance-complete attempt observations into the generic
    /// learning-evidence representation. Legacy-incomplete attempts remain source
    /// evidence but cannot be silently promoted into a complete semantic event.
    pub fn to_learning_evidence_event(
        &self,
    ) -> Result<LearningEvidenceEvent, AttemptEvidenceContractError> {
        self.validate()?;
        if !matches!(&self.completeness, AttemptEvidenceCompleteness::Complete) {
            return Err(AttemptEvidenceContractError::IncompleteCannotPromote);
        }

        let dimension = self
            .dimension
            .clone()
            .ok_or(AttemptEvidenceContractError::CompleteMissingCapabilityDimension)?;
        let source_activity = self
            .source_activity_id
            .as_ref()
            .ok_or(AttemptEvidenceContractError::CompleteMissingSourceActivity)?;

        Ok(LearningEvidenceEvent {
            event_id: self.event_id.clone(),
            learner_id: self.learner_id.clone(),
            capability_id: self.capability_id.clone(),
            source: self.source.clone(),
            assistance: self.assistance.clone(),
            measures: vec![EvidenceMeasure {
                dimension,
                value_permille: self.outcome.value_permille()?,
            }],
            observed_at: self.observed_at,
            provenance: self.provenance.clone(),
            context_tags: vec![
                "praxis:attempt-evidence-v1".to_string(),
                format!("source-activity:{source_activity}"),
            ],
        })
    }

    /// Build a truthful wrapper for the historical adaptive-zome attempt API.
    ///
    /// The adapter records what is actually known and explicitly marks everything
    /// the old API never captured. In particular it does not infer a task,
    /// capability dimension, assistance state, original evaluator, or trace
    /// reference from `correct` and `response_time_ms`.
    pub fn from_legacy_record_attempt(
        event_id: EvidenceEventId,
        learner_id: LearnerId,
        capability_id: CapabilityId,
        correct: bool,
        response_time_ms: u32,
        observed_at: i64,
        adapter_version: impl Into<String>,
    ) -> Self {
        Self {
            event_id,
            learner_id,
            capability_id,
            source_activity_id: None,
            source: EvidenceSource::Practice,
            dimension: None,
            outcome: AttemptOutcome::Binary { correct },
            response_time_ms: Some(response_time_ms),
            assistance: AssistanceKind::Unknown,
            provenance: EvidenceProvenance {
                producer_id: LEGACY_RECORD_ATTEMPT_ADAPTER_ID.to_string(),
                producer_version: Some(adapter_version.into()),
                source_record_id: None,
                artifact_digest: None,
            },
            completeness: AttemptEvidenceCompleteness::LegacyIncomplete {
                missing_fields: vec![
                    MissingAttemptEvidenceField::SourceActivity,
                    MissingAttemptEvidenceField::CapabilityDimension,
                    MissingAttemptEvidenceField::Assistance,
                    MissingAttemptEvidenceField::OriginalEvaluator,
                    MissingAttemptEvidenceField::TraceReference,
                ],
            },
            observed_at,
        }
    }

    pub const fn is_legacy_incomplete(&self) -> bool {
        matches!(
            &self.completeness,
            AttemptEvidenceCompleteness::LegacyIncomplete { .. }
        )
    }

    /// Source observations never grant credential authority at this layer.
    pub const fn grants_credential_authority(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn complete_attempt(assistance: AssistanceKind) -> AttemptEvidence {
        AttemptEvidence {
            event_id: EvidenceEventId("event-42".into()),
            learner_id: LearnerId("learner-1".into()),
            capability_id: CapabilityId("rust:ownership".into()),
            source_activity_id: Some("assessment:item-7".into()),
            source: EvidenceSource::Assessment,
            dimension: Some(CapabilityDimension::Application),
            outcome: AttemptOutcome::Scored {
                score_permille: 840,
            },
            response_time_ms: Some(12_500),
            assistance,
            provenance: EvidenceProvenance {
                producer_id: "praxis-assessment".into(),
                producer_version: Some("2.1.0".into()),
                source_record_id: Some("attempt-record-42".into()),
                artifact_digest: None,
            },
            completeness: AttemptEvidenceCompleteness::Complete,
            observed_at: 1_700_000_000,
        }
    }

    #[test]
    fn complete_attempt_maps_to_generic_evidence_without_losing_assistance() {
        let attempt = complete_attempt(AssistanceKind::DirectAnswer);
        assert_eq!(attempt.validate(), Ok(()));

        let event = attempt.to_learning_evidence_event().unwrap();
        assert_eq!(event.assistance, AssistanceKind::DirectAnswer);
        assert_eq!(event.measures.len(), 1);
        assert_eq!(event.measures[0].value_permille, 840);
        assert!(!attempt.grants_credential_authority());
    }

    #[test]
    fn legacy_adapter_refuses_to_invent_missing_history() {
        let attempt = AttemptEvidence::from_legacy_record_attempt(
            EvidenceEventId("legacy-action-hash".into()),
            LearnerId("learner-1".into()),
            CapabilityId("skill-action-hash".into()),
            true,
            900,
            1_700_000_000,
            "0.1.0",
        );

        assert_eq!(attempt.validate(), Ok(()));
        assert!(attempt.is_legacy_incomplete());
        assert_eq!(attempt.source_activity_id, None);
        assert_eq!(attempt.dimension, None);
        assert_eq!(attempt.assistance, AssistanceKind::Unknown);
        assert_eq!(
            attempt.provenance.producer_id,
            LEGACY_RECORD_ATTEMPT_ADAPTER_ID
        );
        assert_eq!(
            attempt.to_learning_evidence_event(),
            Err(AttemptEvidenceContractError::IncompleteCannotPromote)
        );
    }

    #[test]
    fn complete_attempt_requires_trace_reference() {
        let mut attempt = complete_attempt(AssistanceKind::Unassisted);
        attempt.provenance.source_record_id = None;
        attempt.provenance.artifact_digest = None;

        assert_eq!(
            attempt.validate(),
            Err(AttemptEvidenceContractError::CompleteMissingTraceReference)
        );
    }

    #[test]
    fn complete_attempt_cannot_hide_unknown_assistance() {
        let attempt = complete_attempt(AssistanceKind::Unknown);
        assert_eq!(
            attempt.validate(),
            Err(AttemptEvidenceContractError::CompleteUnknownAssistance)
        );
    }

    #[test]
    fn scored_attempt_must_be_normalized() {
        let mut attempt = complete_attempt(AssistanceKind::Unassisted);
        attempt.outcome = AttemptOutcome::Scored {
            score_permille: 1001,
        };

        assert_eq!(
            attempt.validate(),
            Err(AttemptEvidenceContractError::OutcomeOutOfRange {
                score_permille: 1001
            })
        );
    }

    #[test]
    fn legacy_missing_fields_are_self_consistent() {
        let mut attempt = AttemptEvidence::from_legacy_record_attempt(
            EvidenceEventId("legacy-action-hash".into()),
            LearnerId("learner-1".into()),
            CapabilityId("skill-action-hash".into()),
            false,
            1_200,
            1_700_000_000,
            "0.1.0",
        );

        if let AttemptEvidenceCompleteness::LegacyIncomplete { missing_fields } =
            &mut attempt.completeness
        {
            missing_fields.retain(|field| field != &MissingAttemptEvidenceField::Assistance);
        }

        assert_eq!(
            attempt.validate(),
            Err(AttemptEvidenceContractError::UndeclaredMissingField(
                MissingAttemptEvidenceField::Assistance
            ))
        );
    }

    #[test]
    fn duplicate_legacy_missing_fields_are_rejected() {
        let mut attempt = AttemptEvidence::from_legacy_record_attempt(
            EvidenceEventId("legacy-action-hash".into()),
            LearnerId("learner-1".into()),
            CapabilityId("skill-action-hash".into()),
            false,
            1_200,
            1_700_000_000,
            "0.1.0",
        );

        if let AttemptEvidenceCompleteness::LegacyIncomplete { missing_fields } =
            &mut attempt.completeness
        {
            missing_fields.push(MissingAttemptEvidenceField::Assistance);
        }

        assert_eq!(
            attempt.validate(),
            Err(AttemptEvidenceContractError::DuplicateMissingField(
                MissingAttemptEvidenceField::Assistance
            ))
        );
    }
}

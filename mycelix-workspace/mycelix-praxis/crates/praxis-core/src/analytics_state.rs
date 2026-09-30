// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-bound analytics contracts.
//!
//! Descriptive session summaries, inferred session analysis, and period aggregates
//! are separate derived layers. None of them is source learning evidence, a mastery
//! claim, a credential decision, or an authorization decision.

use crate::goal_state::DerivedProjectionRef;
use crate::learning_evidence::{CapabilityId, EvidenceEventId, LearnerId};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SessionId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SessionObservationRef {
    pub session_id: SessionId,
    /// Digest of the exact serialized/normalized session observation summary.
    pub summary_digest: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AnalyticsProducerProvenance {
    pub producer_id: String,
    pub producer_version: String,
    pub parameters_digest: String,
}

/// Descriptive summary of one session over exact source evidence events.
///
/// It deliberately contains no focus estimate, mastery change, skill unlock, or
/// completion claim. Those belong to separately versioned projections/policies.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SessionObservationSummary {
    pub session_id: SessionId,
    pub learner_id: LearnerId,
    pub collector: AnalyticsProducerProvenance,
    pub input_event_ids: Vec<EvidenceEventId>,
    pub capabilities_touched: Vec<CapabilityId>,
    pub started_at: i64,
    pub ended_at: i64,
    pub items_attempted: u32,
    pub items_completed: u32,
    pub correct_count: u32,
    pub hints_used: u32,
    pub skips: u32,
    pub active_time_seconds: u32,
    pub response_samples: u32,
    pub response_time_total_ms: u64,
    pub generated_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum AnalyticsInputRef {
    Evidence(EvidenceEventId),
    Projection(DerivedProjectionRef),
    Session(SessionObservationRef),
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SessionAnalysisComponentKind {
    FocusEstimate,
    ChallengeSkillBalanceEstimate,
    FrustrationSignalDensity,
    BoredomSignalDensity,
    EngagementPattern,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SessionAnalysisComponent {
    pub kind: SessionAnalysisComponentKind,
    pub estimate_permille: u16,
    /// Support metadata from the named analyzer, not assumed to be calibrated.
    pub support_confidence_permille: u16,
    pub input_refs: Vec<AnalyticsInputRef>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SessionAnalysisProjection {
    pub analysis_id: String,
    pub learner_id: LearnerId,
    pub session_ref: SessionObservationRef,
    pub analyzer: AnalyticsProducerProvenance,
    pub input_refs: Vec<AnalyticsInputRef>,
    pub components: Vec<SessionAnalysisComponent>,
    pub generated_at: i64,
}

/// Period metrics use explicit names for policy/model-relative conclusions rather
/// than collapsing them into universal `mastery` language.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PeriodAnalyticsMetric {
    ObservedSessionCount {
        count: u32,
        input_refs: Vec<AnalyticsInputRef>,
    },
    ObservedActiveTimeSeconds {
        seconds: u64,
        input_refs: Vec<AnalyticsInputRef>,
    },
    ObservedItemsCompleted {
        count: u64,
        input_refs: Vec<AnalyticsInputRef>,
    },
    ObservedAccuracyPermille {
        value_permille: u16,
        input_refs: Vec<AnalyticsInputRef>,
    },
    AdmittedCapabilityCountUnderProfile {
        profile_id: String,
        profile_version: String,
        count: u32,
        input_refs: Vec<AnalyticsInputRef>,
    },
    CapabilityEstimateChangePermille {
        estimator_id: String,
        estimator_version: String,
        change_permille: i16,
        input_refs: Vec<AnalyticsInputRef>,
    },
    PeakActivityHour {
        hour: u8,
        input_refs: Vec<AnalyticsInputRef>,
    },
    PeakActivityDay {
        day: u8,
        input_refs: Vec<AnalyticsInputRef>,
    },
    AverageSessionMinutes {
        minutes: u32,
        input_refs: Vec<AnalyticsInputRef>,
    },
    ActiveDayCount {
        count: u32,
        input_refs: Vec<AnalyticsInputRef>,
    },
    StreakLength {
        days: u32,
        input_refs: Vec<AnalyticsInputRef>,
    },
    GamificationXp {
        amount: u64,
        input_refs: Vec<AnalyticsInputRef>,
    },
    OtherPermille {
        name: String,
        value_permille: u16,
        input_refs: Vec<AnalyticsInputRef>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PeriodAnalyticsProjection {
    pub analytics_id: String,
    pub learner_id: LearnerId,
    pub period_started_at: i64,
    pub period_ended_at: i64,
    pub producer: AnalyticsProducerProvenance,
    /// Exact unique dependency set consumed by all metrics.
    pub input_refs: Vec<AnalyticsInputRef>,
    pub metrics: Vec<PeriodAnalyticsMetric>,
    pub generated_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AnalyticsContractError {
    EmptySessionId,
    EmptyLearnerId,
    EmptyProducerId,
    EmptyProducerVersion,
    EmptyParametersDigest,
    InvalidSessionWindow,
    SummaryGeneratedBeforeSessionEnd,
    CompletedExceedsAttempted,
    CorrectExceedsCompleted,
    SkipsExceedAttempted,
    ResponseSamplesExceedAttempted,
    ResponseTimeWithoutSamples,
    ActiveTimeExceedsSessionWindow,
    ActivityWithoutEvidenceInputs,
    EmptyEvidenceEventId,
    DuplicateEvidenceEventId(EvidenceEventId),
    EmptyCapabilityId,
    DuplicateCapabilityId(CapabilityId),
    EmptySessionDigest,
    EmptyProjectionKind,
    EmptyProjectionId,
    EmptyProjectionDigest,
    EmptyAnalysisId,
    NoInputs,
    DuplicateInput(AnalyticsInputRef),
    SessionSummaryNotDeclared,
    NoComponents,
    EmptyOtherComponentName,
    DuplicateComponentKind(SessionAnalysisComponentKind),
    ComponentEstimateOutOfRange {
        kind: SessionAnalysisComponentKind,
        value_permille: u16,
    },
    ComponentConfidenceOutOfRange {
        kind: SessionAnalysisComponentKind,
        value_permille: u16,
    },
    ComponentHasNoInputs(SessionAnalysisComponentKind),
    DuplicateComponentInput {
        kind: SessionAnalysisComponentKind,
        input: AnalyticsInputRef,
    },
    ComponentInputNotDeclared {
        kind: SessionAnalysisComponentKind,
        input: AnalyticsInputRef,
    },
    DeclaredInputUnused(AnalyticsInputRef),
    EmptyAnalyticsId,
    InvalidPeriodWindow,
    AnalyticsGeneratedBeforePeriodEnd,
    NoMetrics,
    DuplicateMetricKind(String),
    MetricHasNoInputs(String),
    DuplicateMetricInput {
        kind: String,
        input: AnalyticsInputRef,
    },
    MetricInputNotDeclared {
        kind: String,
        input: AnalyticsInputRef,
    },
    MetricPermilleOutOfRange {
        kind: String,
        value_permille: u16,
    },
    MetricSignedPermilleOutOfRange {
        kind: String,
        value_permille: i16,
    },
    EmptyProfileId,
    EmptyProfileVersion,
    EmptyEstimatorId,
    EmptyEstimatorVersion,
    PeakHourOutOfRange(u8),
    PeakDayOutOfRange(u8),
    EmptyOtherMetricName,
}

impl AnalyticsProducerProvenance {
    fn validate(&self) -> Result<(), AnalyticsContractError> {
        if self.producer_id.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyProducerId);
        }
        if self.producer_version.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyProducerVersion);
        }
        if self.parameters_digest.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyParametersDigest);
        }
        Ok(())
    }
}

impl SessionObservationSummary {
    pub fn validate(&self) -> Result<(), AnalyticsContractError> {
        if self.session_id.0.trim().is_empty() {
            return Err(AnalyticsContractError::EmptySessionId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyLearnerId);
        }
        self.collector.validate()?;
        if self.ended_at < self.started_at {
            return Err(AnalyticsContractError::InvalidSessionWindow);
        }
        if self.generated_at < self.ended_at {
            return Err(AnalyticsContractError::SummaryGeneratedBeforeSessionEnd);
        }
        if self.items_completed > self.items_attempted {
            return Err(AnalyticsContractError::CompletedExceedsAttempted);
        }
        if self.correct_count > self.items_completed {
            return Err(AnalyticsContractError::CorrectExceedsCompleted);
        }
        if self.skips > self.items_attempted {
            return Err(AnalyticsContractError::SkipsExceedAttempted);
        }
        if self.response_samples > self.items_attempted {
            return Err(AnalyticsContractError::ResponseSamplesExceedAttempted);
        }
        if self.response_samples == 0 && self.response_time_total_ms != 0 {
            return Err(AnalyticsContractError::ResponseTimeWithoutSamples);
        }
        let elapsed = self.ended_at - self.started_at;
        if i64::from(self.active_time_seconds) > elapsed {
            return Err(AnalyticsContractError::ActiveTimeExceedsSessionWindow);
        }
        if self.items_attempted > 0 && self.input_event_ids.is_empty() {
            return Err(AnalyticsContractError::ActivityWithoutEvidenceInputs);
        }

        let mut events = BTreeSet::new();
        for event_id in &self.input_event_ids {
            if event_id.0.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyEvidenceEventId);
            }
            if !events.insert(event_id.clone()) {
                return Err(AnalyticsContractError::DuplicateEvidenceEventId(event_id.clone()));
            }
        }

        let mut capabilities = BTreeSet::new();
        for capability in &self.capabilities_touched {
            if capability.0.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyCapabilityId);
            }
            if !capabilities.insert(capability.clone()) {
                return Err(AnalyticsContractError::DuplicateCapabilityId(capability.clone()));
            }
        }
        Ok(())
    }

    pub fn average_response_time_ms(&self) -> Option<u64> {
        (self.response_samples > 0)
            .then(|| self.response_time_total_ms / u64::from(self.response_samples))
    }

    pub const fn is_source_learning_evidence(&self) -> bool {
        false
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }
}

impl SessionAnalysisProjection {
    pub fn validate(&self) -> Result<(), AnalyticsContractError> {
        if self.analysis_id.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyAnalysisId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyLearnerId);
        }
        validate_session_ref(&self.session_ref)?;
        self.analyzer.validate()?;
        let (declared, mut used) = validate_declared_inputs(&self.input_refs)?;
        let required_session = AnalyticsInputRef::Session(self.session_ref.clone());
        if !declared.contains(&required_session) {
            return Err(AnalyticsContractError::SessionSummaryNotDeclared);
        }
        if self.components.is_empty() {
            return Err(AnalyticsContractError::NoComponents);
        }

        let mut kinds = BTreeSet::new();
        for component in &self.components {
            if let SessionAnalysisComponentKind::Other(name) = &component.kind {
                if name.trim().is_empty() {
                    return Err(AnalyticsContractError::EmptyOtherComponentName);
                }
            }
            if !kinds.insert(component.kind.clone()) {
                return Err(AnalyticsContractError::DuplicateComponentKind(component.kind.clone()));
            }
            if component.estimate_permille > 1000 {
                return Err(AnalyticsContractError::ComponentEstimateOutOfRange {
                    kind: component.kind.clone(),
                    value_permille: component.estimate_permille,
                });
            }
            if component.support_confidence_permille > 1000 {
                return Err(AnalyticsContractError::ComponentConfidenceOutOfRange {
                    kind: component.kind.clone(),
                    value_permille: component.support_confidence_permille,
                });
            }
            if component.input_refs.is_empty() {
                return Err(AnalyticsContractError::ComponentHasNoInputs(component.kind.clone()));
            }
            let mut local = BTreeSet::new();
            for input in &component.input_refs {
                validate_input_ref(input)?;
                if !local.insert(input.clone()) {
                    return Err(AnalyticsContractError::DuplicateComponentInput {
                        kind: component.kind.clone(),
                        input: input.clone(),
                    });
                }
                if !declared.contains(input) {
                    return Err(AnalyticsContractError::ComponentInputNotDeclared {
                        kind: component.kind.clone(),
                        input: input.clone(),
                    });
                }
                used.insert(input.clone());
            }
        }
        reject_unused_inputs(declared, &used)?;
        Ok(())
    }

    pub const fn is_source_learning_evidence(&self) -> bool {
        false
    }
    pub const fn grants_trust_authority(&self) -> bool {
        false
    }
    pub const fn grants_credential_authority(&self) -> bool {
        false
    }
    pub const fn grants_authorization(&self) -> bool {
        false
    }
}

impl PeriodAnalyticsProjection {
    pub fn validate(&self) -> Result<(), AnalyticsContractError> {
        if self.analytics_id.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyAnalyticsId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(AnalyticsContractError::EmptyLearnerId);
        }
        if self.period_ended_at <= self.period_started_at {
            return Err(AnalyticsContractError::InvalidPeriodWindow);
        }
        if self.generated_at < self.period_ended_at {
            return Err(AnalyticsContractError::AnalyticsGeneratedBeforePeriodEnd);
        }
        self.producer.validate()?;
        let (declared, mut used) = validate_declared_inputs(&self.input_refs)?;
        if self.metrics.is_empty() {
            return Err(AnalyticsContractError::NoMetrics);
        }

        let mut kinds = BTreeSet::new();
        for metric in &self.metrics {
            validate_metric_value(metric)?;
            let key = metric_kind_key(metric);
            if !kinds.insert(key.clone()) {
                return Err(AnalyticsContractError::DuplicateMetricKind(key));
            }
            let inputs = metric_inputs(metric);
            if inputs.is_empty() {
                return Err(AnalyticsContractError::MetricHasNoInputs(key));
            }
            let mut local = BTreeSet::new();
            for input in inputs {
                validate_input_ref(input)?;
                if !local.insert(input.clone()) {
                    return Err(AnalyticsContractError::DuplicateMetricInput {
                        kind: key.clone(),
                        input: input.clone(),
                    });
                }
                if !declared.contains(input) {
                    return Err(AnalyticsContractError::MetricInputNotDeclared {
                        kind: key.clone(),
                        input: input.clone(),
                    });
                }
                used.insert(input.clone());
            }
        }
        reject_unused_inputs(declared, &used)?;
        Ok(())
    }

    pub const fn is_source_learning_evidence(&self) -> bool {
        false
    }
    pub const fn grants_trust_authority(&self) -> bool {
        false
    }
    pub const fn grants_credential_authority(&self) -> bool {
        false
    }
    pub const fn grants_authorization(&self) -> bool {
        false
    }
}

fn validate_declared_inputs(
    inputs: &[AnalyticsInputRef],
) -> Result<(BTreeSet<AnalyticsInputRef>, BTreeSet<AnalyticsInputRef>), AnalyticsContractError> {
    if inputs.is_empty() {
        return Err(AnalyticsContractError::NoInputs);
    }
    let mut declared = BTreeSet::new();
    for input in inputs {
        validate_input_ref(input)?;
        if !declared.insert(input.clone()) {
            return Err(AnalyticsContractError::DuplicateInput(input.clone()));
        }
    }
    Ok((declared, BTreeSet::new()))
}

fn validate_input_ref(input: &AnalyticsInputRef) -> Result<(), AnalyticsContractError> {
    match input {
        AnalyticsInputRef::Evidence(id) => {
            if id.0.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyEvidenceEventId);
            }
        }
        AnalyticsInputRef::Projection(reference) => {
            if reference.projection_kind.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyProjectionKind);
            }
            if reference.projection_id.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyProjectionId);
            }
            if reference.projection_digest.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyProjectionDigest);
            }
        }
        AnalyticsInputRef::Session(reference) => validate_session_ref(reference)?,
    }
    Ok(())
}

fn validate_session_ref(reference: &SessionObservationRef) -> Result<(), AnalyticsContractError> {
    if reference.session_id.0.trim().is_empty() {
        return Err(AnalyticsContractError::EmptySessionId);
    }
    if reference.summary_digest.trim().is_empty() {
        return Err(AnalyticsContractError::EmptySessionDigest);
    }
    Ok(())
}

fn reject_unused_inputs(
    declared: BTreeSet<AnalyticsInputRef>,
    used: &BTreeSet<AnalyticsInputRef>,
) -> Result<(), AnalyticsContractError> {
    for input in declared {
        if !used.contains(&input) {
            return Err(AnalyticsContractError::DeclaredInputUnused(input));
        }
    }
    Ok(())
}

fn metric_inputs(metric: &PeriodAnalyticsMetric) -> &[AnalyticsInputRef] {
    match metric {
        PeriodAnalyticsMetric::ObservedSessionCount { input_refs, .. }
        | PeriodAnalyticsMetric::ObservedActiveTimeSeconds { input_refs, .. }
        | PeriodAnalyticsMetric::ObservedItemsCompleted { input_refs, .. }
        | PeriodAnalyticsMetric::ObservedAccuracyPermille { input_refs, .. }
        | PeriodAnalyticsMetric::AdmittedCapabilityCountUnderProfile { input_refs, .. }
        | PeriodAnalyticsMetric::CapabilityEstimateChangePermille { input_refs, .. }
        | PeriodAnalyticsMetric::PeakActivityHour { input_refs, .. }
        | PeriodAnalyticsMetric::PeakActivityDay { input_refs, .. }
        | PeriodAnalyticsMetric::AverageSessionMinutes { input_refs, .. }
        | PeriodAnalyticsMetric::ActiveDayCount { input_refs, .. }
        | PeriodAnalyticsMetric::StreakLength { input_refs, .. }
        | PeriodAnalyticsMetric::GamificationXp { input_refs, .. }
        | PeriodAnalyticsMetric::OtherPermille { input_refs, .. } => input_refs,
    }
}

fn metric_kind_key(metric: &PeriodAnalyticsMetric) -> String {
    match metric {
        PeriodAnalyticsMetric::ObservedSessionCount { .. } => "observed-session-count".into(),
        PeriodAnalyticsMetric::ObservedActiveTimeSeconds { .. } => "observed-active-time-seconds".into(),
        PeriodAnalyticsMetric::ObservedItemsCompleted { .. } => "observed-items-completed".into(),
        PeriodAnalyticsMetric::ObservedAccuracyPermille { .. } => "observed-accuracy-permille".into(),
        PeriodAnalyticsMetric::AdmittedCapabilityCountUnderProfile { profile_id, profile_version, .. } => {
            format!("admitted-capability-count:{profile_id}:{profile_version}")
        }
        PeriodAnalyticsMetric::CapabilityEstimateChangePermille { estimator_id, estimator_version, .. } => {
            format!("capability-estimate-change:{estimator_id}:{estimator_version}")
        }
        PeriodAnalyticsMetric::PeakActivityHour { .. } => "peak-activity-hour".into(),
        PeriodAnalyticsMetric::PeakActivityDay { .. } => "peak-activity-day".into(),
        PeriodAnalyticsMetric::AverageSessionMinutes { .. } => "average-session-minutes".into(),
        PeriodAnalyticsMetric::ActiveDayCount { .. } => "active-day-count".into(),
        PeriodAnalyticsMetric::StreakLength { .. } => "streak-length".into(),
        PeriodAnalyticsMetric::GamificationXp { .. } => "gamification-xp".into(),
        PeriodAnalyticsMetric::OtherPermille { name, .. } => format!("other:{name}"),
    }
}

fn validate_metric_value(metric: &PeriodAnalyticsMetric) -> Result<(), AnalyticsContractError> {
    match metric {
        PeriodAnalyticsMetric::ObservedAccuracyPermille { value_permille, .. } => {
            if *value_permille > 1000 {
                return Err(AnalyticsContractError::MetricPermilleOutOfRange {
                    kind: "observed-accuracy-permille".into(),
                    value_permille: *value_permille,
                });
            }
        }
        PeriodAnalyticsMetric::AdmittedCapabilityCountUnderProfile {
            profile_id,
            profile_version,
            ..
        } => {
            if profile_id.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyProfileId);
            }
            if profile_version.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyProfileVersion);
            }
        }
        PeriodAnalyticsMetric::CapabilityEstimateChangePermille {
            estimator_id,
            estimator_version,
            change_permille,
            ..
        } => {
            if estimator_id.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyEstimatorId);
            }
            if estimator_version.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyEstimatorVersion);
            }
            if !(-1000..=1000).contains(change_permille) {
                return Err(AnalyticsContractError::MetricSignedPermilleOutOfRange {
                    kind: "capability-estimate-change".into(),
                    value_permille: *change_permille,
                });
            }
        }
        PeriodAnalyticsMetric::PeakActivityHour { hour, .. } if *hour > 23 => {
            return Err(AnalyticsContractError::PeakHourOutOfRange(*hour));
        }
        PeriodAnalyticsMetric::PeakActivityDay { day, .. } if *day > 6 => {
            return Err(AnalyticsContractError::PeakDayOutOfRange(*day));
        }
        PeriodAnalyticsMetric::OtherPermille {
            name,
            value_permille,
            ..
        } => {
            if name.trim().is_empty() {
                return Err(AnalyticsContractError::EmptyOtherMetricName);
            }
            if *value_permille > 1000 {
                return Err(AnalyticsContractError::MetricPermilleOutOfRange {
                    kind: format!("other:{name}"),
                    value_permille: *value_permille,
                });
            }
        }
        _ => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn producer() -> AnalyticsProducerProvenance {
        AnalyticsProducerProvenance {
            producer_id: "praxis:analytics".into(),
            producer_version: "1".into(),
            parameters_digest: "blake3:params".into(),
        }
    }

    fn session_ref() -> SessionObservationRef {
        SessionObservationRef {
            session_id: SessionId("session-1".into()),
            summary_digest: "blake3:session".into(),
        }
    }

    fn event(id: &str) -> AnalyticsInputRef {
        AnalyticsInputRef::Evidence(EvidenceEventId(id.into()))
    }

    #[test]
    fn session_summary_contains_observations_not_mastery_claims() {
        let summary = SessionObservationSummary {
            session_id: SessionId("session-1".into()),
            learner_id: LearnerId("learner-1".into()),
            collector: producer(),
            input_event_ids: vec![EvidenceEventId("event-1".into())],
            capabilities_touched: vec![CapabilityId("rust:ownership".into())],
            started_at: 100,
            ended_at: 160,
            items_attempted: 1,
            items_completed: 1,
            correct_count: 1,
            hints_used: 0,
            skips: 0,
            active_time_seconds: 50,
            response_samples: 1,
            response_time_total_ms: 2400,
            generated_at: 161,
        };
        assert_eq!(summary.validate(), Ok(()));
        assert_eq!(summary.average_response_time_ms(), Some(2400));
        assert!(!summary.is_source_learning_evidence());
        assert!(!summary.grants_credential_authority());
    }

    #[test]
    fn session_analysis_requires_the_exact_session_summary_input() {
        let required = AnalyticsInputRef::Session(session_ref());
        let projection = SessionAnalysisProjection {
            analysis_id: "analysis-1".into(),
            learner_id: LearnerId("learner-1".into()),
            session_ref: session_ref(),
            analyzer: producer(),
            input_refs: vec![required.clone()],
            components: vec![SessionAnalysisComponent {
                kind: SessionAnalysisComponentKind::FocusEstimate,
                estimate_permille: 700,
                support_confidence_permille: 450,
                input_refs: vec![required],
            }],
            generated_at: 200,
        };
        assert_eq!(projection.validate(), Ok(()));
        assert!(!projection.grants_trust_authority());
        assert!(!projection.grants_credential_authority());
        assert!(!projection.grants_authorization());
    }

    #[test]
    fn period_capability_count_is_explicitly_profile_relative() {
        let source = event("event-1");
        let projection = PeriodAnalyticsProjection {
            analytics_id: "period-1".into(),
            learner_id: LearnerId("learner-1".into()),
            period_started_at: 100,
            period_ended_at: 200,
            producer: producer(),
            input_refs: vec![source.clone()],
            metrics: vec![PeriodAnalyticsMetric::AdmittedCapabilityCountUnderProfile {
                profile_id: "credential-support-v1".into(),
                profile_version: "1".into(),
                count: 2,
                input_refs: vec![source],
            }],
            generated_at: 201,
        };
        assert_eq!(projection.validate(), Ok(()));
        assert!(!projection.grants_credential_authority());
    }

    #[test]
    fn universal_mastery_is_not_an_analytics_metric() {
        let source = event("event-1");
        let projection = PeriodAnalyticsProjection {
            analytics_id: "period-1".into(),
            learner_id: LearnerId("learner-1".into()),
            period_started_at: 100,
            period_ended_at: 200,
            producer: producer(),
            input_refs: vec![source.clone()],
            metrics: vec![PeriodAnalyticsMetric::CapabilityEstimateChangePermille {
                estimator_id: "praxis:bkt-projection".into(),
                estimator_version: "integer-v1".into(),
                change_permille: 80,
                input_refs: vec![source],
            }],
            generated_at: 201,
        };
        assert_eq!(projection.validate(), Ok(()));
        assert!(!projection.is_source_learning_evidence());
    }

    #[test]
    fn provenance_padding_is_rejected() {
        let used = event("event-used");
        let unused = event("event-unused");
        let projection = PeriodAnalyticsProjection {
            analytics_id: "period-1".into(),
            learner_id: LearnerId("learner-1".into()),
            period_started_at: 100,
            period_ended_at: 200,
            producer: producer(),
            input_refs: vec![used.clone(), unused.clone()],
            metrics: vec![PeriodAnalyticsMetric::ObservedItemsCompleted {
                count: 1,
                input_refs: vec![used],
            }],
            generated_at: 201,
        };
        assert_eq!(
            projection.validate(),
            Err(AnalyticsContractError::DeclaredInputUnused(unused))
        );
    }
}

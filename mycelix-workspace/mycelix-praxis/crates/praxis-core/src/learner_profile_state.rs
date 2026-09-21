// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Learner-authored preferences separated from inferred learner-profile projections.
//!
//! Full learner state is private by default. Sharing is represented by a separate,
//! minimized disclosure receipt rather than publishing the source intent or full
//! derived projection.

use crate::analytics_state::{AnalyticsInputRef, AnalyticsProducerProvenance};
use crate::goal_state::DerivedProjectionRef;
use crate::learning_evidence::LearnerId;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct LearnerPreferenceIntentId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum LearningModality {
    Visual,
    Auditory,
    ReadingWriting,
    Kinesthetic,
    Multimodal,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PreferredTimeWindow {
    pub start_hour: u8,
    pub end_hour: u8,
}

/// No generic public variant exists for the full preference intent.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PreferenceDisclosure {
    Private,
    ExplicitlyShared { audience_ids: Vec<String> },
}

impl Default for PreferenceDisclosure {
    fn default() -> Self {
        Self::Private
    }
}

/// Learner-authored preferences only. This type deliberately contains no observed
/// accuracy, retention, velocity, attention estimate, completion rate, or confidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearnerPreferenceIntent {
    pub preference_intent_id: LearnerPreferenceIntentId,
    pub version: u64,
    pub learner_id: LearnerId,
    pub preferred_modalities: Vec<LearningModality>,
    pub preferred_session_minutes: Option<u16>,
    pub preferred_time_window: Option<PreferredTimeWindow>,
    pub preferred_difficulty_permille: Option<u16>,
    pub custom_preferences: Vec<String>,
    pub disclosure: PreferenceDisclosure,
    pub authored_at: i64,
    pub revised_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum LearnerProfileSignalKind {
    VisualModalityEstimate,
    AuditoryModalityEstimate,
    ReadingWritingModalityEstimate,
    KinestheticModalityEstimate,
    AttentionSpanEstimate,
    ObservedAccuracy,
    LearningVelocityEstimate,
    Retention7dEstimate,
    Retention30dEstimate,
    ObservedAverageDailyActivity,
    ObservedDaysSinceLastActivity,
    ObservedSessionCount,
    ObservedCompletionRate,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LearnerProfileSignalValue {
    Permille(u16),
    Minutes(u32),
    Count(u64),
    Days(u32),
    Hour(u8),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearnerProfileSignal {
    pub kind: LearnerProfileSignalKind,
    pub value: LearnerProfileSignalValue,
    /// Analyzer support metadata; not assumed to be a calibrated probability.
    pub support_confidence_permille: u16,
    pub input_refs: Vec<AnalyticsInputRef>,
}

/// Recomputable private learner-profile projection over exact evidence/state inputs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearnerProfileProjection {
    pub projection_id: String,
    pub learner_id: LearnerId,
    pub analyzer: AnalyticsProducerProvenance,
    pub input_refs: Vec<AnalyticsInputRef>,
    pub signals: Vec<LearnerProfileSignal>,
    pub generated_at: i64,
    pub expires_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LearnerProfileTemporalStatus {
    NotYetGenerated,
    Fresh,
    Expired,
}

/// Minimized explicit disclosure. It references the exact source projection digest
/// but does not repeat the projection's source event/projection dependency graph.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisclosedLearnerProfileSignal {
    pub kind: LearnerProfileSignalKind,
    pub value: LearnerProfileSignalValue,
    pub support_confidence_permille: u16,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearnerProfileDisclosureProjection {
    pub disclosure_id: String,
    pub learner_id: LearnerId,
    pub source_projection: DerivedProjectionRef,
    pub audience_ids: Vec<String>,
    pub signals: Vec<DisclosedLearnerProfileSignal>,
    pub generated_at: i64,
    pub expires_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LearnerProfileContractError {
    EmptyPreferenceIntentId,
    ZeroPreferenceIntentVersion,
    EmptyLearnerId,
    NoPreferences,
    DuplicateModality(LearningModality),
    ZeroPreferredSessionMinutes,
    PreferredHourOutOfRange(u8),
    DegeneratePreferredTimeWindow,
    PreferredDifficultyOutOfRange(u16),
    EmptyCustomPreference,
    DuplicateCustomPreference,
    EmptyAudienceId,
    DuplicateAudienceId,
    RevisionBeforeAuthorship,
    EmptyProjectionId,
    EmptyProducerId,
    EmptyProducerVersion,
    EmptyParametersDigest,
    NoInputs,
    InvalidInput,
    DuplicateInput(AnalyticsInputRef),
    NoSignals,
    EmptyOtherSignalName,
    DuplicateSignalKind(LearnerProfileSignalKind),
    SignalValueOutOfRange(LearnerProfileSignalKind),
    SignalConfidenceOutOfRange {
        kind: LearnerProfileSignalKind,
        value_permille: u16,
    },
    SignalHasNoInputs(LearnerProfileSignalKind),
    DuplicateSignalInput {
        kind: LearnerProfileSignalKind,
        input: AnalyticsInputRef,
    },
    SignalInputNotDeclared {
        kind: LearnerProfileSignalKind,
        input: AnalyticsInputRef,
    },
    DeclaredInputUnused(AnalyticsInputRef),
    InvalidProjectionExpiry,
    EmptyDisclosureId,
    EmptySourceProjectionKind,
    EmptySourceProjectionId,
    EmptySourceProjectionDigest,
    DisclosureHasNoAudience,
    DisclosureHasNoSignals,
    DuplicateDisclosedSignalKind(LearnerProfileSignalKind),
    InvalidDisclosureExpiry,
}

impl LearnerPreferenceIntent {
    pub fn validate(&self) -> Result<(), LearnerProfileContractError> {
        if self.preference_intent_id.0.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptyPreferenceIntentId);
        }
        if self.version == 0 {
            return Err(LearnerProfileContractError::ZeroPreferenceIntentVersion);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptyLearnerId);
        }
        if self.preferred_modalities.is_empty()
            && self.preferred_session_minutes.is_none()
            && self.preferred_time_window.is_none()
            && self.preferred_difficulty_permille.is_none()
            && self.custom_preferences.is_empty()
        {
            return Err(LearnerProfileContractError::NoPreferences);
        }

        let mut modalities = BTreeSet::new();
        for modality in &self.preferred_modalities {
            if !modalities.insert(modality.clone()) {
                return Err(LearnerProfileContractError::DuplicateModality(modality.clone()));
            }
        }
        if self.preferred_session_minutes == Some(0) {
            return Err(LearnerProfileContractError::ZeroPreferredSessionMinutes);
        }
        if let Some(window) = &self.preferred_time_window {
            if window.start_hour > 23 {
                return Err(LearnerProfileContractError::PreferredHourOutOfRange(window.start_hour));
            }
            if window.end_hour > 23 {
                return Err(LearnerProfileContractError::PreferredHourOutOfRange(window.end_hour));
            }
            if window.start_hour == window.end_hour {
                return Err(LearnerProfileContractError::DegeneratePreferredTimeWindow);
            }
        }
        if let Some(value) = self.preferred_difficulty_permille {
            if value > 1000 {
                return Err(LearnerProfileContractError::PreferredDifficultyOutOfRange(value));
            }
        }

        let mut custom = BTreeSet::new();
        for preference in &self.custom_preferences {
            if preference.trim().is_empty() {
                return Err(LearnerProfileContractError::EmptyCustomPreference);
            }
            if !custom.insert(preference) {
                return Err(LearnerProfileContractError::DuplicateCustomPreference);
            }
        }
        validate_audiences(&self.disclosure)?;
        if self.revised_at < self.authored_at {
            return Err(LearnerProfileContractError::RevisionBeforeAuthorship);
        }
        Ok(())
    }

    pub const fn is_private_by_default(&self) -> bool {
        matches!(&self.disclosure, PreferenceDisclosure::Private)
    }
}

impl LearnerProfileProjection {
    pub fn validate(&self) -> Result<(), LearnerProfileContractError> {
        if self.projection_id.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptyProjectionId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptyLearnerId);
        }
        validate_producer(&self.analyzer)?;
        if self.expires_at <= self.generated_at {
            return Err(LearnerProfileContractError::InvalidProjectionExpiry);
        }
        if self.input_refs.is_empty() {
            return Err(LearnerProfileContractError::NoInputs);
        }
        if self.signals.is_empty() {
            return Err(LearnerProfileContractError::NoSignals);
        }

        let mut declared = BTreeSet::new();
        for input in &self.input_refs {
            if !input_is_well_formed(input) {
                return Err(LearnerProfileContractError::InvalidInput);
            }
            if !declared.insert(input.clone()) {
                return Err(LearnerProfileContractError::DuplicateInput(input.clone()));
            }
        }

        let mut kinds = BTreeSet::new();
        let mut used = BTreeSet::new();
        for signal in &self.signals {
            validate_signal_kind(&signal.kind)?;
            validate_signal_value(&signal.kind, &signal.value)?;
            if signal.support_confidence_permille > 1000 {
                return Err(LearnerProfileContractError::SignalConfidenceOutOfRange {
                    kind: signal.kind.clone(),
                    value_permille: signal.support_confidence_permille,
                });
            }
            if !kinds.insert(signal.kind.clone()) {
                return Err(LearnerProfileContractError::DuplicateSignalKind(signal.kind.clone()));
            }
            if signal.input_refs.is_empty() {
                return Err(LearnerProfileContractError::SignalHasNoInputs(signal.kind.clone()));
            }
            let mut local = BTreeSet::new();
            for input in &signal.input_refs {
                if !input_is_well_formed(input) {
                    return Err(LearnerProfileContractError::InvalidInput);
                }
                if !local.insert(input.clone()) {
                    return Err(LearnerProfileContractError::DuplicateSignalInput {
                        kind: signal.kind.clone(),
                        input: input.clone(),
                    });
                }
                if !declared.contains(input) {
                    return Err(LearnerProfileContractError::SignalInputNotDeclared {
                        kind: signal.kind.clone(),
                        input: input.clone(),
                    });
                }
                used.insert(input.clone());
            }
        }
        for input in declared {
            if !used.contains(&input) {
                return Err(LearnerProfileContractError::DeclaredInputUnused(input));
            }
        }
        Ok(())
    }

    pub fn temporal_status(&self, now: i64) -> LearnerProfileTemporalStatus {
        if now < self.generated_at {
            LearnerProfileTemporalStatus::NotYetGenerated
        } else if now >= self.expires_at {
            LearnerProfileTemporalStatus::Expired
        } else {
            LearnerProfileTemporalStatus::Fresh
        }
    }

    pub const fn is_source_learning_evidence(&self) -> bool { false }
    pub const fn grants_trust_authority(&self) -> bool { false }
    pub const fn grants_credential_authority(&self) -> bool { false }
    pub const fn grants_authorization(&self) -> bool { false }
}

impl LearnerProfileDisclosureProjection {
    pub fn validate(&self) -> Result<(), LearnerProfileContractError> {
        if self.disclosure_id.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptyDisclosureId);
        }
        if self.learner_id.0.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptyLearnerId);
        }
        if self.source_projection.projection_kind.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptySourceProjectionKind);
        }
        if self.source_projection.projection_id.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptySourceProjectionId);
        }
        if self.source_projection.projection_digest.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptySourceProjectionDigest);
        }
        if self.audience_ids.is_empty() {
            return Err(LearnerProfileContractError::DisclosureHasNoAudience);
        }
        let mut audience = BTreeSet::new();
        for id in &self.audience_ids {
            if id.trim().is_empty() {
                return Err(LearnerProfileContractError::EmptyAudienceId);
            }
            if !audience.insert(id) {
                return Err(LearnerProfileContractError::DuplicateAudienceId);
            }
        }
        if self.signals.is_empty() {
            return Err(LearnerProfileContractError::DisclosureHasNoSignals);
        }
        let mut kinds = BTreeSet::new();
        for signal in &self.signals {
            validate_signal_kind(&signal.kind)?;
            validate_signal_value(&signal.kind, &signal.value)?;
            if signal.support_confidence_permille > 1000 {
                return Err(LearnerProfileContractError::SignalConfidenceOutOfRange {
                    kind: signal.kind.clone(),
                    value_permille: signal.support_confidence_permille,
                });
            }
            if !kinds.insert(signal.kind.clone()) {
                return Err(LearnerProfileContractError::DuplicateDisclosedSignalKind(signal.kind.clone()));
            }
        }
        if self.expires_at <= self.generated_at {
            return Err(LearnerProfileContractError::InvalidDisclosureExpiry);
        }
        Ok(())
    }

    pub const fn grants_trust_authority(&self) -> bool { false }
    pub const fn grants_credential_authority(&self) -> bool { false }
    pub const fn grants_authorization(&self) -> bool { false }
}

fn validate_audiences(disclosure: &PreferenceDisclosure) -> Result<(), LearnerProfileContractError> {
    if let PreferenceDisclosure::ExplicitlyShared { audience_ids } = disclosure {
        if audience_ids.is_empty() {
            return Err(LearnerProfileContractError::DisclosureHasNoAudience);
        }
        let mut audience = BTreeSet::new();
        for id in audience_ids {
            if id.trim().is_empty() {
                return Err(LearnerProfileContractError::EmptyAudienceId);
            }
            if !audience.insert(id) {
                return Err(LearnerProfileContractError::DuplicateAudienceId);
            }
        }
    }
    Ok(())
}

fn validate_producer(producer: &AnalyticsProducerProvenance) -> Result<(), LearnerProfileContractError> {
    if producer.producer_id.trim().is_empty() {
        return Err(LearnerProfileContractError::EmptyProducerId);
    }
    if producer.producer_version.trim().is_empty() {
        return Err(LearnerProfileContractError::EmptyProducerVersion);
    }
    if producer.parameters_digest.trim().is_empty() {
        return Err(LearnerProfileContractError::EmptyParametersDigest);
    }
    Ok(())
}

fn validate_signal_kind(kind: &LearnerProfileSignalKind) -> Result<(), LearnerProfileContractError> {
    if let LearnerProfileSignalKind::Other(name) = kind {
        if name.trim().is_empty() {
            return Err(LearnerProfileContractError::EmptyOtherSignalName);
        }
    }
    Ok(())
}

fn validate_signal_value(
    kind: &LearnerProfileSignalKind,
    value: &LearnerProfileSignalValue,
) -> Result<(), LearnerProfileContractError> {
    let invalid = match value {
        LearnerProfileSignalValue::Permille(v) => *v > 1000,
        LearnerProfileSignalValue::Hour(v) => *v > 23,
        LearnerProfileSignalValue::Minutes(_)
        | LearnerProfileSignalValue::Count(_)
        | LearnerProfileSignalValue::Days(_) => false,
    };
    if invalid {
        return Err(LearnerProfileContractError::SignalValueOutOfRange(kind.clone()));
    }
    Ok(())
}

fn input_is_well_formed(input: &AnalyticsInputRef) -> bool {
    match input {
        AnalyticsInputRef::Evidence(id) => !id.0.trim().is_empty(),
        AnalyticsInputRef::Projection(reference) => {
            !reference.projection_kind.trim().is_empty()
                && !reference.projection_id.trim().is_empty()
                && !reference.projection_digest.trim().is_empty()
        }
        AnalyticsInputRef::Session(reference) => {
            !reference.session_id.0.trim().is_empty() && !reference.summary_digest.trim().is_empty()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::learning_evidence::EvidenceEventId;

    fn producer() -> AnalyticsProducerProvenance {
        AnalyticsProducerProvenance {
            producer_id: "praxis:learner-profile".into(),
            producer_version: "1".into(),
            parameters_digest: "blake3:params".into(),
        }
    }

    fn evidence(id: &str) -> AnalyticsInputRef {
        AnalyticsInputRef::Evidence(EvidenceEventId(id.into()))
    }

    #[test]
    fn authored_preferences_are_private_and_contain_no_performance_projection() {
        let intent = LearnerPreferenceIntent {
            preference_intent_id: LearnerPreferenceIntentId("prefs-1".into()),
            version: 1,
            learner_id: LearnerId("learner-1".into()),
            preferred_modalities: vec![LearningModality::Kinesthetic],
            preferred_session_minutes: Some(45),
            preferred_time_window: Some(PreferredTimeWindow { start_hour: 8, end_hour: 11 }),
            preferred_difficulty_permille: Some(600),
            custom_preferences: vec!["short-feedback-loops".into()],
            disclosure: PreferenceDisclosure::Private,
            authored_at: 100,
            revised_at: 100,
        };
        assert_eq!(intent.validate(), Ok(()));
        assert!(intent.is_private_by_default());
    }

    #[test]
    fn profile_projection_is_exact_input_and_non_authoritative() {
        let input = evidence("event-1");
        let projection = LearnerProfileProjection {
            projection_id: "profile-1".into(),
            learner_id: LearnerId("learner-1".into()),
            analyzer: producer(),
            input_refs: vec![input.clone()],
            signals: vec![LearnerProfileSignal {
                kind: LearnerProfileSignalKind::Retention7dEstimate,
                value: LearnerProfileSignalValue::Permille(720),
                support_confidence_permille: 500,
                input_refs: vec![input],
            }],
            generated_at: 100,
            expires_at: 200,
        };
        assert_eq!(projection.validate(), Ok(()));
        assert_eq!(projection.temporal_status(150), LearnerProfileTemporalStatus::Fresh);
        assert!(!projection.is_source_learning_evidence());
        assert!(!projection.grants_credential_authority());
        assert!(!projection.grants_authorization());
    }

    #[test]
    fn unused_profile_provenance_is_rejected() {
        let used = evidence("event-used");
        let unused = evidence("event-unused");
        let projection = LearnerProfileProjection {
            projection_id: "profile-1".into(),
            learner_id: LearnerId("learner-1".into()),
            analyzer: producer(),
            input_refs: vec![used.clone(), unused.clone()],
            signals: vec![LearnerProfileSignal {
                kind: LearnerProfileSignalKind::ObservedAccuracy,
                value: LearnerProfileSignalValue::Permille(800),
                support_confidence_permille: 600,
                input_refs: vec![used],
            }],
            generated_at: 100,
            expires_at: 200,
        };
        assert_eq!(projection.validate(), Err(LearnerProfileContractError::DeclaredInputUnused(unused)));
    }

    #[test]
    fn disclosure_is_minimized_and_audience_bound() {
        let disclosure = LearnerProfileDisclosureProjection {
            disclosure_id: "share-1".into(),
            learner_id: LearnerId("learner-1".into()),
            source_projection: DerivedProjectionRef {
                projection_kind: "learner-profile".into(),
                projection_id: "profile-1".into(),
                projection_digest: "blake3:profile".into(),
            },
            audience_ids: vec!["mentor-1".into()],
            signals: vec![DisclosedLearnerProfileSignal {
                kind: LearnerProfileSignalKind::Retention7dEstimate,
                value: LearnerProfileSignalValue::Permille(720),
                support_confidence_permille: 500,
            }],
            generated_at: 120,
            expires_at: 180,
        };
        assert_eq!(disclosure.validate(), Ok(()));
        assert!(!disclosure.grants_trust_authority());
        assert!(!disclosure.grants_credential_authority());
    }
}

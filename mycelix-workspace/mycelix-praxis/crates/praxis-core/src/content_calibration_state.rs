// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Privacy-aware content difficulty calibration contracts.
//!
//! Exact learner-attempt lineage remains in a private aggregate. Shareable content
//! calibration is a separate minimized disclosure under an explicit release policy.

use crate::analytics_state::AnalyticsProducerProvenance;
use crate::goal_state::DerivedProjectionRef;
use crate::learning_evidence::EvidenceEventId;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ContentCalibrationId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CalibrationAggregateId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CalibratedContentKind {
    Lesson,
    Quiz,
    Exercise,
    Project,
    Assessment,
    Challenge,
    Other(String),
}

/// Private exact aggregate over learner-attempt evidence.
///
/// This object may retain exact event IDs and therefore must not be treated as a
/// shareable/public content statistic merely because it contains aggregate counts.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivateCalibrationAggregate {
    pub aggregate_id: CalibrationAggregateId,
    pub content_id: String,
    pub collector: AnalyticsProducerProvenance,
    pub input_event_ids: Vec<EvidenceEventId>,
    pub distinct_learner_count: u32,
    pub total_attempts: u32,
    pub successful_attempts: u32,
    pub completion_time_samples: u32,
    pub completion_time_total_seconds: u64,
    pub generated_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CalibrationAggregateRef {
    pub aggregate_id: CalibrationAggregateId,
    pub content_id: String,
    pub aggregate_digest: String,
    pub distinct_learner_count: u32,
    pub total_attempts: u32,
}

/// Private recomputable calibration projection over one exact private aggregate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContentDifficultyCalibrationProjection {
    pub projection_id: ContentCalibrationId,
    pub content_id: String,
    pub content_kind: CalibratedContentKind,
    pub source_aggregate: CalibrationAggregateRef,
    pub analyzer: AnalyticsProducerProvenance,
    pub authored_difficulty_permille: Option<u16>,
    pub calibrated_difficulty_permille: u16,
    pub variance_permille: u16,
    pub average_completion_time_seconds: u32,
    pub completion_time_std_dev_seconds: u32,
    pub discrimination_permille: u16,
    pub generated_at: i64,
    pub expires_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CalibrationReleaseMechanism {
    /// Cohort-threshold release only. This is not represented as differential privacy.
    ThresholdedAggregate,
    DifferentialPrivacy {
        mechanism_id: String,
        mechanism_version: String,
        /// epsilon encoded in millionths to avoid floating-point ambiguity.
        epsilon_micros: u32,
        /// delta encoded in parts per billion.
        delta_parts_per_billion: u32,
        noise_parameters_digest: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CalibrationReleasePolicy {
    pub policy_id: String,
    pub policy_version: String,
    pub min_distinct_learners: u32,
    pub mechanism: CalibrationReleaseMechanism,
}

/// Privacy/release admission over a private aggregate.
///
/// The receipt intentionally exposes only a proven/admitted lower bound rather than
/// the exact private distinct-learner count.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CalibrationReleaseAdmission {
    pub source_aggregate_digest: String,
    pub admitted_minimum_distinct_learners: u32,
    pub admission_receipt_digest: String,
}

/// Minimized shareable calibration. It contains no learner IDs or event IDs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContentDifficultyCalibrationDisclosure {
    pub disclosure_id: String,
    pub content_id: String,
    pub content_kind: CalibratedContentKind,
    pub source_projection: DerivedProjectionRef,
    pub source_aggregate_digest: String,
    pub release_policy: CalibrationReleasePolicy,
    pub release_admission: CalibrationReleaseAdmission,
    pub calibrated_difficulty_permille: u16,
    pub variance_permille: Option<u16>,
    pub average_completion_time_seconds: Option<u32>,
    pub completion_time_std_dev_seconds: Option<u32>,
    pub discrimination_permille: Option<u16>,
    pub generated_at: i64,
    pub expires_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ContentCalibrationContractError {
    EmptyAggregateId,
    EmptyContentId,
    EmptyProducerId,
    EmptyProducerVersion,
    EmptyParametersDigest,
    NoAttemptInputs,
    EmptyEvidenceEventId,
    DuplicateEvidenceEventId(EvidenceEventId),
    ZeroDistinctLearners,
    ZeroAttempts,
    DistinctLearnersExceedAttempts,
    SuccessfulAttemptsExceedAttempts,
    TimeSamplesExceedAttempts,
    TimeTotalWithoutSamples,
    EmptyProjectionId,
    EmptyAggregateDigest,
    AggregateContentMismatch,
    CalibrationOutOfRange,
    VarianceOutOfRange,
    DiscriminationOutOfRange,
    AuthoredDifficultyOutOfRange,
    InvalidProjectionExpiry,
    EmptyOtherContentKind,
    EmptyReleasePolicyId,
    EmptyReleasePolicyVersion,
    ZeroReleaseThreshold,
    EmptyDpMechanismId,
    EmptyDpMechanismVersion,
    ZeroDpEpsilon,
    DpDeltaOutOfRange,
    EmptyDpNoiseDigest,
    EmptyDisclosureId,
    EmptySourceProjectionKind,
    EmptySourceProjectionId,
    EmptySourceProjectionDigest,
    EmptySourceAggregateDigest,
    EmptyAdmissionAggregateDigest,
    EmptyAdmissionReceiptDigest,
    ReleaseAggregateDigestMismatch,
    AdmissionBelowPolicyThreshold,
    DisclosureVarianceOutOfRange,
    DisclosureDiscriminationOutOfRange,
    InvalidDisclosureExpiry,
}

impl PrivateCalibrationAggregate {
    pub fn validate(&self) -> Result<(), ContentCalibrationContractError> {
        if self.aggregate_id.0.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyAggregateId);
        }
        if self.content_id.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyContentId);
        }
        validate_producer(&self.collector)?;
        if self.input_event_ids.is_empty() {
            return Err(ContentCalibrationContractError::NoAttemptInputs);
        }
        if self.distinct_learner_count == 0 {
            return Err(ContentCalibrationContractError::ZeroDistinctLearners);
        }
        if self.total_attempts == 0 {
            return Err(ContentCalibrationContractError::ZeroAttempts);
        }
        if self.distinct_learner_count > self.total_attempts {
            return Err(ContentCalibrationContractError::DistinctLearnersExceedAttempts);
        }
        if self.successful_attempts > self.total_attempts {
            return Err(ContentCalibrationContractError::SuccessfulAttemptsExceedAttempts);
        }
        if self.completion_time_samples > self.total_attempts {
            return Err(ContentCalibrationContractError::TimeSamplesExceedAttempts);
        }
        if self.completion_time_samples == 0 && self.completion_time_total_seconds != 0 {
            return Err(ContentCalibrationContractError::TimeTotalWithoutSamples);
        }

        let mut events = BTreeSet::new();
        for event_id in &self.input_event_ids {
            if event_id.0.trim().is_empty() {
                return Err(ContentCalibrationContractError::EmptyEvidenceEventId);
            }
            if !events.insert(event_id.clone()) {
                return Err(ContentCalibrationContractError::DuplicateEvidenceEventId(event_id.clone()));
            }
        }
        Ok(())
    }

    pub fn average_completion_time_seconds(&self) -> Option<u64> {
        (self.completion_time_samples > 0).then(|| {
            self.completion_time_total_seconds / u64::from(self.completion_time_samples)
        })
    }

    pub const fn is_shareable_public_calibration(&self) -> bool {
        false
    }
}

impl ContentDifficultyCalibrationProjection {
    pub fn validate(&self) -> Result<(), ContentCalibrationContractError> {
        if self.projection_id.0.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyProjectionId);
        }
        if self.content_id.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyContentId);
        }
        validate_content_kind(&self.content_kind)?;
        validate_aggregate_ref(&self.source_aggregate)?;
        if self.source_aggregate.content_id != self.content_id {
            return Err(ContentCalibrationContractError::AggregateContentMismatch);
        }
        validate_producer(&self.analyzer)?;
        if let Some(value) = self.authored_difficulty_permille {
            if value > 1000 {
                return Err(ContentCalibrationContractError::AuthoredDifficultyOutOfRange);
            }
        }
        if self.calibrated_difficulty_permille > 1000 {
            return Err(ContentCalibrationContractError::CalibrationOutOfRange);
        }
        if self.variance_permille > 1000 {
            return Err(ContentCalibrationContractError::VarianceOutOfRange);
        }
        if self.discrimination_permille > 1000 {
            return Err(ContentCalibrationContractError::DiscriminationOutOfRange);
        }
        if self.expires_at <= self.generated_at {
            return Err(ContentCalibrationContractError::InvalidProjectionExpiry);
        }
        Ok(())
    }

    pub const fn is_source_learning_evidence(&self) -> bool { false }
    pub const fn grants_trust_authority(&self) -> bool { false }
    pub const fn grants_credential_authority(&self) -> bool { false }
    pub const fn grants_authorization(&self) -> bool { false }
}

impl CalibrationReleasePolicy {
    pub fn validate(&self) -> Result<(), ContentCalibrationContractError> {
        if self.policy_id.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyReleasePolicyId);
        }
        if self.policy_version.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyReleasePolicyVersion);
        }
        if self.min_distinct_learners == 0 {
            return Err(ContentCalibrationContractError::ZeroReleaseThreshold);
        }
        if let CalibrationReleaseMechanism::DifferentialPrivacy {
            mechanism_id,
            mechanism_version,
            epsilon_micros,
            delta_parts_per_billion,
            noise_parameters_digest,
        } = &self.mechanism
        {
            if mechanism_id.trim().is_empty() {
                return Err(ContentCalibrationContractError::EmptyDpMechanismId);
            }
            if mechanism_version.trim().is_empty() {
                return Err(ContentCalibrationContractError::EmptyDpMechanismVersion);
            }
            if *epsilon_micros == 0 {
                return Err(ContentCalibrationContractError::ZeroDpEpsilon);
            }
            if *delta_parts_per_billion >= 1_000_000_000 {
                return Err(ContentCalibrationContractError::DpDeltaOutOfRange);
            }
            if noise_parameters_digest.trim().is_empty() {
                return Err(ContentCalibrationContractError::EmptyDpNoiseDigest);
            }
        }
        Ok(())
    }
}

impl ContentDifficultyCalibrationDisclosure {
    pub fn validate(&self) -> Result<(), ContentCalibrationContractError> {
        if self.disclosure_id.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyDisclosureId);
        }
        if self.content_id.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyContentId);
        }
        validate_content_kind(&self.content_kind)?;
        validate_projection_ref(&self.source_projection)?;
        if self.source_aggregate_digest.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptySourceAggregateDigest);
        }
        self.release_policy.validate()?;
        if self.release_admission.source_aggregate_digest.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyAdmissionAggregateDigest);
        }
        if self.release_admission.admission_receipt_digest.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyAdmissionReceiptDigest);
        }
        if self.release_admission.source_aggregate_digest != self.source_aggregate_digest {
            return Err(ContentCalibrationContractError::ReleaseAggregateDigestMismatch);
        }
        if self.release_admission.admitted_minimum_distinct_learners
            < self.release_policy.min_distinct_learners
        {
            return Err(ContentCalibrationContractError::AdmissionBelowPolicyThreshold);
        }
        if self.calibrated_difficulty_permille > 1000 {
            return Err(ContentCalibrationContractError::CalibrationOutOfRange);
        }
        if self.variance_permille.is_some_and(|value| value > 1000) {
            return Err(ContentCalibrationContractError::DisclosureVarianceOutOfRange);
        }
        if self.discrimination_permille.is_some_and(|value| value > 1000) {
            return Err(ContentCalibrationContractError::DisclosureDiscriminationOutOfRange);
        }
        if self.expires_at <= self.generated_at {
            return Err(ContentCalibrationContractError::InvalidDisclosureExpiry);
        }
        Ok(())
    }

    pub const fn contains_per_learner_event_ids(&self) -> bool { false }
    pub const fn grants_trust_authority(&self) -> bool { false }
    pub const fn grants_credential_authority(&self) -> bool { false }
    pub const fn grants_authorization(&self) -> bool { false }
}

fn validate_producer(producer: &AnalyticsProducerProvenance) -> Result<(), ContentCalibrationContractError> {
    if producer.producer_id.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptyProducerId);
    }
    if producer.producer_version.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptyProducerVersion);
    }
    if producer.parameters_digest.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptyParametersDigest);
    }
    Ok(())
}

fn validate_aggregate_ref(reference: &CalibrationAggregateRef) -> Result<(), ContentCalibrationContractError> {
    if reference.aggregate_id.0.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptyAggregateId);
    }
    if reference.content_id.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptyContentId);
    }
    if reference.aggregate_digest.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptyAggregateDigest);
    }
    if reference.distinct_learner_count == 0 {
        return Err(ContentCalibrationContractError::ZeroDistinctLearners);
    }
    if reference.total_attempts == 0 {
        return Err(ContentCalibrationContractError::ZeroAttempts);
    }
    if reference.distinct_learner_count > reference.total_attempts {
        return Err(ContentCalibrationContractError::DistinctLearnersExceedAttempts);
    }
    Ok(())
}

fn validate_projection_ref(reference: &DerivedProjectionRef) -> Result<(), ContentCalibrationContractError> {
    if reference.projection_kind.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptySourceProjectionKind);
    }
    if reference.projection_id.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptySourceProjectionId);
    }
    if reference.projection_digest.trim().is_empty() {
        return Err(ContentCalibrationContractError::EmptySourceProjectionDigest);
    }
    Ok(())
}

fn validate_content_kind(kind: &CalibratedContentKind) -> Result<(), ContentCalibrationContractError> {
    if let CalibratedContentKind::Other(name) = kind {
        if name.trim().is_empty() {
            return Err(ContentCalibrationContractError::EmptyOtherContentKind);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn producer() -> AnalyticsProducerProvenance {
        AnalyticsProducerProvenance {
            producer_id: "praxis:difficulty-calibration".into(),
            producer_version: "1".into(),
            parameters_digest: "blake3:params".into(),
        }
    }

    #[test]
    fn private_aggregate_retains_exact_lineage_but_is_not_shareable_by_itself() {
        let aggregate = PrivateCalibrationAggregate {
            aggregate_id: CalibrationAggregateId("aggregate-1".into()),
            content_id: "content-1".into(),
            collector: producer(),
            input_event_ids: vec![EvidenceEventId("event-1".into()), EvidenceEventId("event-2".into())],
            distinct_learner_count: 2,
            total_attempts: 2,
            successful_attempts: 1,
            completion_time_samples: 2,
            completion_time_total_seconds: 90,
            generated_at: 100,
        };
        assert_eq!(aggregate.validate(), Ok(()));
        assert_eq!(aggregate.average_completion_time_seconds(), Some(45));
        assert!(!aggregate.is_shareable_public_calibration());
    }

    #[test]
    fn disclosure_requires_release_threshold_admission() {
        let disclosure = ContentDifficultyCalibrationDisclosure {
            disclosure_id: "release-1".into(),
            content_id: "content-1".into(),
            content_kind: CalibratedContentKind::Exercise,
            source_projection: DerivedProjectionRef {
                projection_kind: "content-difficulty-calibration".into(),
                projection_id: "projection-1".into(),
                projection_digest: "blake3:projection".into(),
            },
            source_aggregate_digest: "blake3:aggregate".into(),
            release_policy: CalibrationReleasePolicy {
                policy_id: "calibration-release-v1".into(),
                policy_version: "1".into(),
                min_distinct_learners: 10,
                mechanism: CalibrationReleaseMechanism::ThresholdedAggregate,
            },
            release_admission: CalibrationReleaseAdmission {
                source_aggregate_digest: "blake3:aggregate".into(),
                admitted_minimum_distinct_learners: 10,
                admission_receipt_digest: "blake3:admission".into(),
            },
            calibrated_difficulty_permille: 640,
            variance_permille: Some(120),
            average_completion_time_seconds: Some(45),
            completion_time_std_dev_seconds: Some(8),
            discrimination_permille: Some(700),
            generated_at: 110,
            expires_at: 210,
        };
        assert_eq!(disclosure.validate(), Ok(()));
        assert!(!disclosure.contains_per_learner_event_ids());
        assert!(!disclosure.grants_credential_authority());
    }

    #[test]
    fn disclosure_rejects_insufficient_admitted_cohort() {
        let mut disclosure = ContentDifficultyCalibrationDisclosure {
            disclosure_id: "release-1".into(),
            content_id: "content-1".into(),
            content_kind: CalibratedContentKind::Exercise,
            source_projection: DerivedProjectionRef {
                projection_kind: "content-difficulty-calibration".into(),
                projection_id: "projection-1".into(),
                projection_digest: "blake3:projection".into(),
            },
            source_aggregate_digest: "blake3:aggregate".into(),
            release_policy: CalibrationReleasePolicy {
                policy_id: "calibration-release-v1".into(),
                policy_version: "1".into(),
                min_distinct_learners: 10,
                mechanism: CalibrationReleaseMechanism::ThresholdedAggregate,
            },
            release_admission: CalibrationReleaseAdmission {
                source_aggregate_digest: "blake3:aggregate".into(),
                admitted_minimum_distinct_learners: 9,
                admission_receipt_digest: "blake3:admission".into(),
            },
            calibrated_difficulty_permille: 640,
            variance_permille: None,
            average_completion_time_seconds: None,
            completion_time_std_dev_seconds: None,
            discrimination_permille: None,
            generated_at: 110,
            expires_at: 210,
        };
        assert_eq!(
            disclosure.validate(),
            Err(ContentCalibrationContractError::AdmissionBelowPolicyThreshold)
        );
        disclosure.release_admission.admitted_minimum_distinct_learners = 10;
        assert_eq!(disclosure.validate(), Ok(()));
    }

    #[test]
    fn differential_privacy_release_has_explicit_parameters() {
        let policy = CalibrationReleasePolicy {
            policy_id: "dp-release".into(),
            policy_version: "1".into(),
            min_distinct_learners: 20,
            mechanism: CalibrationReleaseMechanism::DifferentialPrivacy {
                mechanism_id: "gaussian".into(),
                mechanism_version: "1".into(),
                epsilon_micros: 1_000_000,
                delta_parts_per_billion: 1,
                noise_parameters_digest: "blake3:noise".into(),
            },
        };
        assert_eq!(policy.validate(), Ok(()));
    }
}

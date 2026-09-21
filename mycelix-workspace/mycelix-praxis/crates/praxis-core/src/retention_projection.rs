// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Deterministic advisory retention forecasts over an exact BKT projection.
//!
//! This first model is intentionally simple and honestly named. It is a deterministic
//! hyperbolic half-life heuristic, not a claim of empirical FSRS/Ebbinghaus
//! calibration. Historical forecasts remain interpretable because model identity,
//! version, parameters, source BKT receipt, anchor event, time, and horizons are bound.

use crate::attempt_evidence::{AttemptEvidence, AttemptEvidenceCompleteness};
use crate::bkt_projection::{
    BktObservationEncoding, BktProjectionReceipt, BKT_ESTIMATOR_ID, BKT_ESTIMATOR_VERSION,
};
use crate::crypto::hash_to_string;
use crate::learning_evidence::{
    CapabilityDimension, CapabilityId, EstimatorProvenance, EvidenceEventId, LearnerId,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const RETENTION_ESTIMATOR_ID: &str = "praxis:retention-forecast";
pub const RETENTION_ESTIMATOR_VERSION: &str = "integer-hyperbolic-v1";

/// Parameters for the deterministic v1 retention heuristic.
///
/// `estimate_weight_permille` controls how strongly the source BKT estimate extends
/// the base half-life. At 1000, a source estimate of 1000 can double the base term.
/// `support_bonus_minutes_per_event` adds a bounded support-count term. Neither term
/// should be interpreted as scientifically calibrated until separately validated.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionForecastParameters {
    pub base_half_life_minutes: u32,
    pub estimate_weight_permille: u16,
    pub support_bonus_minutes_per_event: u32,
    pub max_half_life_minutes: u32,
}

impl Default for RetentionForecastParameters {
    fn default() -> Self {
        Self {
            base_half_life_minutes: 1_440,
            estimate_weight_permille: 1_000,
            support_bonus_minutes_per_event: 120,
            max_half_life_minutes: 525_600,
        }
    }
}

impl RetentionForecastParameters {
    pub fn validate(&self) -> Result<(), RetentionForecastError> {
        if self.base_half_life_minutes == 0 {
            return Err(RetentionForecastError::ZeroBaseHalfLife);
        }
        if self.estimate_weight_permille > 1000 {
            return Err(RetentionForecastError::EstimateWeightOutOfRange(
                self.estimate_weight_permille,
            ));
        }
        if self.max_half_life_minutes < self.base_half_life_minutes {
            return Err(RetentionForecastError::MaxHalfLifeBelowBase);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionForecastPoint {
    /// Minutes after `generated_at`.
    pub horizon_minutes: u32,
    /// Model-relative retention estimate in permille.
    pub retention_estimate_permille: u16,
}

/// Full input required to recompute a forecast.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionForecastInput {
    pub bkt_receipt: BktProjectionReceipt,
    /// Exact last complete attempt used as the temporal anchor. Its event ID must be
    /// the last event in the source BKT receipt.
    pub anchor_attempt: AttemptEvidence,
    pub parameters: RetentionForecastParameters,
    /// Positive future horizons. Input order is not authoritative; output is sorted.
    pub forecast_horizons_minutes: Vec<u32>,
    pub generated_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionForecastReceipt {
    pub learner_id: LearnerId,
    pub capability_id: CapabilityId,
    pub dimension: CapabilityDimension,
    pub source_bkt_estimator_id: String,
    pub source_bkt_estimator_version: String,
    pub source_bkt_parameters_digest: String,
    pub source_bkt_admission_profile_id: String,
    pub source_bkt_admission_profile_version: String,
    pub source_event_ids: Vec<EvidenceEventId>,
    pub source_capability_estimate_permille: u16,
    /// Support metadata copied from the source BKT estimate. It is not assumed to be
    /// a calibrated probability of truth or retention.
    pub source_support_confidence_permille: u16,
    pub anchor_event_id: EvidenceEventId,
    pub anchor_observed_at: i64,
    pub model: EstimatorProvenance,
    pub estimated_half_life_minutes: u32,
    pub current_retention_estimate_permille: u16,
    pub forecast_points: Vec<RetentionForecastPoint>,
    pub generated_at: i64,
}

impl RetentionForecastReceipt {
    pub const fn is_measured_retention(&self) -> bool {
        false
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }

    pub const fn grants_trust_authority(&self) -> bool {
        false
    }

    pub const fn grants_authorization(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RetentionForecastError {
    InvalidSourceBktEstimate,
    WrongSourceEstimator,
    MissingSourceBktParametersDigest,
    SourceBktParameterDigestMismatch,
    NoSourceEvents,
    SourceEventOrderMismatch,
    SourceBktMustHaveOneDimension,
    InvalidAnchorAttempt,
    IncompleteAnchorAttempt,
    AnchorEventMismatch,
    AnchorLearnerMismatch,
    AnchorCapabilityMismatch,
    AnchorDimensionMismatch,
    AnchorAfterSourceProjection,
    ForecastBeforeSourceProjection,
    ForecastBeforeAnchor,
    ZeroBaseHalfLife,
    EstimateWeightOutOfRange(u16),
    MaxHalfLifeBelowBase,
    NoForecastHorizons,
    ZeroForecastHorizon,
    DuplicateForecastHorizon(u32),
}

/// Produce an advisory retention forecast over one exact deterministic BKT receipt.
///
/// The v1 curve is:
///
/// ```text
/// R(t) = H / (H + t)
/// ```
///
/// where `H` is the model-derived half-life. Therefore `R(H) = 0.5` by construction.
/// This is a transparent heuristic contract, not a claim of empirical calibration.
pub fn project_retention(
    mut input: RetentionForecastInput,
) -> Result<RetentionForecastReceipt, RetentionForecastError> {
    input.parameters.validate()?;
    validate_source_bkt(&input.bkt_receipt)?;

    let estimate = &input.bkt_receipt.estimate;
    let dimension_estimate = estimate
        .dimensions
        .first()
        .ok_or(RetentionForecastError::SourceBktMustHaveOneDimension)?;

    if input.anchor_attempt.validate().is_err() {
        return Err(RetentionForecastError::InvalidAnchorAttempt);
    }
    if !matches!(
        &input.anchor_attempt.completeness,
        AttemptEvidenceCompleteness::Complete
    ) {
        return Err(RetentionForecastError::IncompleteAnchorAttempt);
    }

    let expected_anchor = input
        .bkt_receipt
        .ordered_event_ids
        .last()
        .ok_or(RetentionForecastError::NoSourceEvents)?;
    if &input.anchor_attempt.event_id != expected_anchor {
        return Err(RetentionForecastError::AnchorEventMismatch);
    }
    if input.anchor_attempt.learner_id != estimate.learner_id {
        return Err(RetentionForecastError::AnchorLearnerMismatch);
    }
    if input.anchor_attempt.capability_id != estimate.capability_id {
        return Err(RetentionForecastError::AnchorCapabilityMismatch);
    }
    if input.anchor_attempt.dimension.as_ref() != Some(&dimension_estimate.dimension) {
        return Err(RetentionForecastError::AnchorDimensionMismatch);
    }
    if input.anchor_attempt.observed_at > estimate.generated_at {
        return Err(RetentionForecastError::AnchorAfterSourceProjection);
    }
    if input.generated_at < estimate.generated_at {
        return Err(RetentionForecastError::ForecastBeforeSourceProjection);
    }
    if input.generated_at < input.anchor_attempt.observed_at {
        return Err(RetentionForecastError::ForecastBeforeAnchor);
    }

    if input.forecast_horizons_minutes.is_empty() {
        return Err(RetentionForecastError::NoForecastHorizons);
    }
    input.forecast_horizons_minutes.sort_unstable();
    let mut horizons = BTreeSet::new();
    for horizon in &input.forecast_horizons_minutes {
        if *horizon == 0 {
            return Err(RetentionForecastError::ZeroForecastHorizon);
        }
        if !horizons.insert(*horizon) {
            return Err(RetentionForecastError::DuplicateForecastHorizon(*horizon));
        }
    }

    let half_life_minutes = estimate_half_life_minutes(
        dimension_estimate.estimate_permille,
        input.bkt_receipt.ordered_event_ids.len(),
        &input.parameters,
    );
    let elapsed_seconds = (input.generated_at - input.anchor_attempt.observed_at) as u64;
    let current_retention = hyperbolic_retention_permille(half_life_minutes, elapsed_seconds);

    let forecast_points = input
        .forecast_horizons_minutes
        .into_iter()
        .map(|horizon_minutes| {
            let horizon_seconds = u64::from(horizon_minutes) * 60;
            RetentionForecastPoint {
                horizon_minutes,
                retention_estimate_permille: hyperbolic_retention_permille(
                    half_life_minutes,
                    elapsed_seconds + horizon_seconds,
                ),
            }
        })
        .collect();

    let source_bkt_parameters_digest = estimate
        .estimator
        .parameters_digest
        .clone()
        .ok_or(RetentionForecastError::MissingSourceBktParametersDigest)?;
    let model_parameters_digest = retention_parameter_digest(&input.parameters);

    Ok(RetentionForecastReceipt {
        learner_id: estimate.learner_id.clone(),
        capability_id: estimate.capability_id.clone(),
        dimension: dimension_estimate.dimension.clone(),
        source_bkt_estimator_id: estimate.estimator.estimator_id.clone(),
        source_bkt_estimator_version: estimate.estimator.estimator_version.clone(),
        source_bkt_parameters_digest,
        source_bkt_admission_profile_id: estimate.admission_profile_id.clone(),
        source_bkt_admission_profile_version: estimate.admission_profile_version.clone(),
        source_event_ids: input.bkt_receipt.ordered_event_ids,
        source_capability_estimate_permille: dimension_estimate.estimate_permille,
        source_support_confidence_permille: dimension_estimate.confidence_permille,
        anchor_event_id: input.anchor_attempt.event_id,
        anchor_observed_at: input.anchor_attempt.observed_at,
        model: EstimatorProvenance {
            estimator_id: RETENTION_ESTIMATOR_ID.to_string(),
            estimator_version: RETENTION_ESTIMATOR_VERSION.to_string(),
            parameters_digest: Some(model_parameters_digest),
        },
        estimated_half_life_minutes: half_life_minutes,
        current_retention_estimate_permille: current_retention,
        forecast_points,
        generated_at: input.generated_at,
    })
}

fn validate_source_bkt(receipt: &BktProjectionReceipt) -> Result<(), RetentionForecastError> {
    if receipt.estimate.validate().is_err() {
        return Err(RetentionForecastError::InvalidSourceBktEstimate);
    }
    if receipt.estimate.estimator.estimator_id != BKT_ESTIMATOR_ID
        || receipt.estimate.estimator.estimator_version != BKT_ESTIMATOR_VERSION
    {
        return Err(RetentionForecastError::WrongSourceEstimator);
    }
    if receipt.ordered_event_ids.is_empty() {
        return Err(RetentionForecastError::NoSourceEvents);
    }
    if receipt.estimate.input_event_ids != receipt.ordered_event_ids {
        return Err(RetentionForecastError::SourceEventOrderMismatch);
    }
    if receipt.estimate.dimensions.len() != 1 {
        return Err(RetentionForecastError::SourceBktMustHaveOneDimension);
    }
    if receipt.parameters.validate().is_err() || receipt.encoding.validate().is_err() {
        return Err(RetentionForecastError::InvalidSourceBktEstimate);
    }
    let recorded = receipt
        .estimate
        .estimator
        .parameters_digest
        .as_ref()
        .ok_or(RetentionForecastError::MissingSourceBktParametersDigest)?;
    let expected = expected_bkt_parameter_digest(receipt);
    if recorded != &expected {
        return Err(RetentionForecastError::SourceBktParameterDigestMismatch);
    }
    Ok(())
}

fn expected_bkt_parameter_digest(receipt: &BktProjectionReceipt) -> String {
    let encoding_fragment = match &receipt.encoding {
        BktObservationEncoding::BinaryOnly => "encoding=binary-only".to_string(),
        BktObservationEncoding::ScoredThreshold {
            correct_at_or_above_permille,
        } => format!("encoding=scored-threshold:{correct_at_or_above_permille}"),
    };
    let canonical = format!(
        "praxis-bkt-integer-v1|prior={}|learn={}|guess={}|slip={}|support-confidence-step={}|{}",
        receipt.parameters.prior_permille,
        receipt.parameters.learn_permille,
        receipt.parameters.guess_permille,
        receipt.parameters.slip_permille,
        receipt.parameters.support_confidence_step_permille,
        encoding_fragment,
    );
    format!("blake3:{}", hash_to_string(canonical.as_bytes()))
}

fn retention_parameter_digest(parameters: &RetentionForecastParameters) -> String {
    let canonical = format!(
        "praxis-retention-integer-hyperbolic-v1|base-half-life-minutes={}|estimate-weight={}|support-bonus-minutes-per-event={}|max-half-life-minutes={}",
        parameters.base_half_life_minutes,
        parameters.estimate_weight_permille,
        parameters.support_bonus_minutes_per_event,
        parameters.max_half_life_minutes,
    );
    format!("blake3:{}", hash_to_string(canonical.as_bytes()))
}

fn estimate_half_life_minutes(
    estimate_permille: u16,
    support_event_count: usize,
    parameters: &RetentionForecastParameters,
) -> u32 {
    let estimate_bonus_permille = div_round_half_up(
        u64::from(estimate_permille) * u64::from(parameters.estimate_weight_permille),
        1000,
    );
    let scaled_base = div_round_half_up(
        u64::from(parameters.base_half_life_minutes) * (1000 + estimate_bonus_permille),
        1000,
    );
    let support_bonus = u64::from(parameters.support_bonus_minutes_per_event)
        .saturating_mul(support_event_count as u64);
    scaled_base
        .saturating_add(support_bonus)
        .min(u64::from(parameters.max_half_life_minutes)) as u32
}

fn hyperbolic_retention_permille(half_life_minutes: u32, elapsed_seconds: u64) -> u16 {
    let half_life_seconds = u64::from(half_life_minutes) * 60;
    let denominator = half_life_seconds + elapsed_seconds;
    if denominator == 0 {
        return 1000;
    }
    div_round_half_up(half_life_seconds * 1000, denominator).min(1000) as u16
}

fn div_round_half_up(numerator: u64, denominator: u64) -> u64 {
    (numerator + denominator / 2) / denominator
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::attempt_evidence::AttemptOutcome;
    use crate::bkt_projection::{
        project_bkt, AdmittedBktAttempt, BktObservationEncoding, BktParameters,
        BktProjectionInput,
    };
    use crate::learning_evidence::{
        AdmissionOutcome, AssistanceKind, EvidenceAdmissionDecision, EvidenceProvenance,
        EvidenceSource,
    };

    fn source_attempt(event_id: &str, observed_at: i64) -> AttemptEvidence {
        AttemptEvidence {
            event_id: EvidenceEventId(event_id.into()),
            learner_id: LearnerId("learner-1".into()),
            capability_id: CapabilityId("rust:ownership".into()),
            source_activity_id: Some(format!("task:{event_id}")),
            source: EvidenceSource::Assessment,
            dimension: Some(CapabilityDimension::Application),
            outcome: AttemptOutcome::Binary { correct: true },
            response_time_ms: Some(1_000),
            assistance: AssistanceKind::Unassisted,
            provenance: EvidenceProvenance {
                producer_id: "praxis-test".into(),
                producer_version: Some("1".into()),
                source_record_id: Some(format!("record:{event_id}")),
                artifact_digest: None,
            },
            completeness: AttemptEvidenceCompleteness::Complete,
            observed_at,
        }
    }

    fn bkt_receipt(anchor: &AttemptEvidence) -> BktProjectionReceipt {
        let admitted = AdmittedBktAttempt {
            attempt: anchor.clone(),
            admission: EvidenceAdmissionDecision {
                event_id: anchor.event_id.clone(),
                profile_id: "practice-bkt".into(),
                profile_version: "1".into(),
                outcome: AdmissionOutcome::Admitted,
                decided_at: anchor.observed_at + 1,
            },
        };
        project_bkt(BktProjectionInput {
            learner_id: anchor.learner_id.clone(),
            capability_id: anchor.capability_id.clone(),
            dimension: anchor.dimension.clone().unwrap(),
            admission_profile_id: "practice-bkt".into(),
            admission_profile_version: "1".into(),
            parameters: BktParameters {
                prior_permille: 500,
                learn_permille: 100,
                guess_permille: 200,
                slip_permille: 100,
                support_confidence_step_permille: 50,
            },
            encoding: BktObservationEncoding::BinaryOnly,
            observations: vec![admitted],
            generated_at: 60,
        })
        .unwrap()
    }

    fn parameters() -> RetentionForecastParameters {
        RetentionForecastParameters {
            base_half_life_minutes: 1_440,
            estimate_weight_permille: 1_000,
            support_bonus_minutes_per_event: 0,
            max_half_life_minutes: 10_000,
        }
    }

    #[test]
    fn hyperbolic_curve_is_exactly_half_at_half_life() {
        assert_eq!(
            hyperbolic_retention_permille(2_160, 2_160 * 60),
            500
        );
    }

    #[test]
    fn forecast_is_bound_to_exact_bkt_and_anchor() {
        let anchor = source_attempt("event-a", 0);
        let receipt = project_retention(RetentionForecastInput {
            bkt_receipt: bkt_receipt(&anchor),
            anchor_attempt: anchor,
            parameters: parameters(),
            forecast_horizons_minutes: vec![2_643, 60],
            generated_at: 60,
        })
        .unwrap();

        // One correct observation under the frozen BKT regression parameters yields
        // 836 permille; v1 scales 1440 minutes by 1.836 -> 2644 minutes.
        assert_eq!(receipt.source_capability_estimate_permille, 836);
        assert_eq!(receipt.estimated_half_life_minutes, 2_644);
        assert_eq!(receipt.forecast_points[0].horizon_minutes, 60);
        assert_eq!(receipt.forecast_points[1].horizon_minutes, 2_643);
        assert_eq!(receipt.forecast_points[1].retention_estimate_permille, 500);
        assert_eq!(receipt.model.estimator_version, RETENTION_ESTIMATOR_VERSION);
        assert!(!receipt.is_measured_retention());
        assert!(!receipt.grants_credential_authority());
        assert!(!receipt.grants_authorization());
    }

    #[test]
    fn tampered_bkt_parameter_digest_is_rejected() {
        let anchor = source_attempt("event-a", 0);
        let mut source = bkt_receipt(&anchor);
        source.estimate.estimator.parameters_digest = Some("blake3:tampered".into());
        assert_eq!(
            project_retention(RetentionForecastInput {
                bkt_receipt: source,
                anchor_attempt: anchor,
                parameters: parameters(),
                forecast_horizons_minutes: vec![60],
                generated_at: 60,
            }),
            Err(RetentionForecastError::SourceBktParameterDigestMismatch)
        );
    }

    #[test]
    fn anchor_must_be_the_last_source_event() {
        let anchor = source_attempt("event-a", 0);
        let source = bkt_receipt(&anchor);
        let wrong_anchor = source_attempt("event-b", 0);
        assert_eq!(
            project_retention(RetentionForecastInput {
                bkt_receipt: source,
                anchor_attempt: wrong_anchor,
                parameters: parameters(),
                forecast_horizons_minutes: vec![60],
                generated_at: 60,
            }),
            Err(RetentionForecastError::AnchorEventMismatch)
        );
    }

    #[test]
    fn duplicate_or_zero_horizons_are_rejected() {
        let anchor = source_attempt("event-a", 0);
        assert_eq!(
            project_retention(RetentionForecastInput {
                bkt_receipt: bkt_receipt(&anchor),
                anchor_attempt: anchor.clone(),
                parameters: parameters(),
                forecast_horizons_minutes: vec![60, 60],
                generated_at: 60,
            }),
            Err(RetentionForecastError::DuplicateForecastHorizon(60))
        );
        assert_eq!(
            project_retention(RetentionForecastInput {
                bkt_receipt: bkt_receipt(&anchor),
                anchor_attempt: anchor,
                parameters: parameters(),
                forecast_horizons_minutes: vec![0],
                generated_at: 60,
            }),
            Err(RetentionForecastError::ZeroForecastHorizon)
        );
    }
}

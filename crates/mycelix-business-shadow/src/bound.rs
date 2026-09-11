// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Protocol-bound public API for read-only business shadow qualification.
//!
//! The implementation module is private so scorecards cannot be detached from the
//! preregistered protocol/model/window that produced them and relabelled afterward.

#[path = "lib.rs"]
mod implementation;

pub use implementation::{
    ForecastCase, ForecastCaseError, ForecastDisposition, ForecastProtocolError,
    ForecastQualificationProtocol, ForecastTarget, ForecastTargetError, ForecastValue,
    MetricObservation, MetricObservationError, RecommendationPromotionDecision,
    RecommendationProtocolError, RecommendationQualificationProtocol, RecommendationReview,
    RecommendationReviewError, ReviewDisposition, ScaledValue, ScaledValueError,
    ShadowCapability, ShadowForecast, ShadowForecastError, ShadowMode, ShadowPromotionDecision,
    ShadowRecommendation, SourceWitness, WitnessError, WitnessRegistry, maximum_shadow_mode,
    unique_capabilities,
};

use mycelix_business_core::{CapabilityRef, Digest32};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastScorecard {
    pub protocol_digest: Digest32,
    pub capability: CapabilityRef,
    pub cases: u32,
    pub candidate_abstentions: u32,
    pub candidate_absolute_error_sum: u128,
    pub baseline_absolute_error_sum: u128,
    pub candidate_interval_hits: u32,
    pub candidate_interval_trials: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastScoringError {
    InvalidProtocol(ForecastProtocolError),
    CandidateModelMismatch { index: usize },
    BaselineModelMismatch { index: usize },
    CaseOutsideEvaluationWindow { index: usize },
    InvalidCase { index: usize, error: ForecastCaseError },
}

impl ForecastScorecard {
    pub fn score_for_protocol(
        protocol: &ForecastQualificationProtocol,
        cases: &[ForecastCase],
    ) -> Result<Self, ForecastScoringError> {
        protocol
            .validate()
            .map_err(ForecastScoringError::InvalidProtocol)?;

        for (index, case) in cases.iter().enumerate() {
            if case.candidate.model_lineage != protocol.candidate_model_lineage {
                return Err(ForecastScoringError::CandidateModelMismatch { index });
            }
            if case.baseline.model_lineage != protocol.baseline_model_lineage {
                return Err(ForecastScoringError::BaselineModelMismatch { index });
            }
            if case.candidate.target.window_start_unix_ms < protocol.evaluation_start_unix_ms
                || case.candidate.target.window_end_unix_ms > protocol.evaluation_end_unix_ms
            {
                return Err(ForecastScoringError::CaseOutsideEvaluationWindow { index });
            }
            case.validate()
                .map_err(|error| ForecastScoringError::InvalidCase { index, error })?;
        }

        let score = implementation::ForecastScorecard::score(cases).map_err(|error| match error {
            implementation::ForecastScoringError::InvalidCase { index, error } => {
                ForecastScoringError::InvalidCase { index, error }
            }
        })?;

        Ok(Self {
            protocol_digest: protocol.protocol_digest,
            capability: protocol.capability.clone(),
            cases: score.cases,
            candidate_abstentions: score.candidate_abstentions,
            candidate_absolute_error_sum: score.candidate_absolute_error_sum,
            baseline_absolute_error_sum: score.baseline_absolute_error_sum,
            candidate_interval_hits: score.candidate_interval_hits,
            candidate_interval_trials: score.candidate_interval_trials,
        })
    }

    pub fn candidate_not_worse_than_baseline(&self) -> bool {
        self.candidate_absolute_error_sum <= self.baseline_absolute_error_sum
    }

    pub fn abstention_bps(&self) -> u16 {
        if self.cases == 0 {
            return 10_000;
        }
        let numerator = u64::from(self.candidate_abstentions) * 10_000;
        (numerator / u64::from(self.cases)).min(10_000) as u16
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastGateError {
    InvalidProtocol(ForecastProtocolError),
    ScorecardProtocolMismatch,
    ScorecardCapabilityMismatch,
}

pub fn evaluate_forecast_gate(
    protocol: &ForecastQualificationProtocol,
    scorecard: &ForecastScorecard,
) -> Result<ShadowPromotionDecision, ForecastGateError> {
    protocol
        .validate()
        .map_err(ForecastGateError::InvalidProtocol)?;
    if scorecard.protocol_digest != protocol.protocol_digest {
        return Err(ForecastGateError::ScorecardProtocolMismatch);
    }
    if scorecard.capability != protocol.capability {
        return Err(ForecastGateError::ScorecardCapabilityMismatch);
    }
    if scorecard.cases < protocol.minimum_cases {
        return Ok(ShadowPromotionDecision::InsufficientCases {
            actual: scorecard.cases,
            required: protocol.minimum_cases,
        });
    }
    let abstention_bps = scorecard.abstention_bps();
    if abstention_bps > protocol.maximum_abstention_bps {
        return Ok(ShadowPromotionDecision::ExcessiveAbstention {
            actual_bps: abstention_bps,
            maximum_bps: protocol.maximum_abstention_bps,
        });
    }
    if protocol.require_candidate_not_worse_than_baseline
        && !scorecard.candidate_not_worse_than_baseline()
    {
        return Ok(ShadowPromotionDecision::CandidateWorseThanBaseline);
    }
    Ok(ShadowPromotionDecision::PassShadowGate)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecommendationScorecard {
    pub protocol_digest: Digest32,
    pub capability: CapabilityRef,
    pub reviewed: u32,
    pub useful: u32,
    pub modified: u32,
    pub not_useful: u32,
    pub unsafe_reviews: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecommendationScoringError {
    InvalidProtocol(RecommendationProtocolError),
    CapabilityMismatch { index: usize },
    RecommendationOutsideEvaluationWindow { index: usize },
    ReviewOutsideEvaluationWindow { index: usize },
    InvalidReview { index: usize, error: RecommendationReviewError },
}

impl RecommendationScorecard {
    pub fn score_for_protocol(
        protocol: &RecommendationQualificationProtocol,
        reviews: &[RecommendationReview],
    ) -> Result<Self, RecommendationScoringError> {
        protocol
            .validate()
            .map_err(RecommendationScoringError::InvalidProtocol)?;

        for (index, review) in reviews.iter().enumerate() {
            if review.recommendation.capability != protocol.capability {
                return Err(RecommendationScoringError::CapabilityMismatch { index });
            }
            if review.recommendation.issued_at_unix_ms < protocol.evaluation_start_unix_ms
                || review.recommendation.issued_at_unix_ms >= protocol.evaluation_end_unix_ms
            {
                return Err(RecommendationScoringError::RecommendationOutsideEvaluationWindow {
                    index,
                });
            }
            if review.reviewed_at_unix_ms > protocol.evaluation_end_unix_ms {
                return Err(RecommendationScoringError::ReviewOutsideEvaluationWindow { index });
            }
            review
                .validate()
                .map_err(|error| RecommendationScoringError::InvalidReview { index, error })?;
        }

        let score = implementation::RecommendationScorecard::score(reviews).map_err(|error| {
            RecommendationScoringError::InvalidReview { index: 0, error }
        })?;

        Ok(Self {
            protocol_digest: protocol.protocol_digest,
            capability: protocol.capability.clone(),
            reviewed: score.reviewed,
            useful: score.useful,
            modified: score.modified,
            not_useful: score.not_useful,
            unsafe_reviews: score.unsafe_reviews,
        })
    }

    pub fn useful_or_modified_bps(&self) -> u16 {
        if self.reviewed == 0 {
            return 0;
        }
        let accepted = u64::from(self.useful + self.modified);
        ((accepted * 10_000) / u64::from(self.reviewed)).min(10_000) as u16
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecommendationGateError {
    InvalidProtocol(RecommendationProtocolError),
    ScorecardProtocolMismatch,
    ScorecardCapabilityMismatch,
}

pub fn evaluate_recommendation_gate(
    protocol: &RecommendationQualificationProtocol,
    scorecard: &RecommendationScorecard,
) -> Result<RecommendationPromotionDecision, RecommendationGateError> {
    protocol
        .validate()
        .map_err(RecommendationGateError::InvalidProtocol)?;
    if scorecard.protocol_digest != protocol.protocol_digest {
        return Err(RecommendationGateError::ScorecardProtocolMismatch);
    }
    if scorecard.capability != protocol.capability {
        return Err(RecommendationGateError::ScorecardCapabilityMismatch);
    }
    if scorecard.reviewed < protocol.minimum_reviews {
        return Ok(RecommendationPromotionDecision::InsufficientReviews {
            actual: scorecard.reviewed,
            required: protocol.minimum_reviews,
        });
    }
    if scorecard.unsafe_reviews > protocol.maximum_unsafe_reviews {
        return Ok(RecommendationPromotionDecision::ExcessiveUnsafeReviews {
            actual: scorecard.unsafe_reviews,
            maximum: protocol.maximum_unsafe_reviews,
        });
    }
    let usefulness = scorecard.useful_or_modified_bps();
    if usefulness < protocol.minimum_useful_or_modified_bps {
        return Ok(RecommendationPromotionDecision::InsufficientUsefulness {
            actual_bps: usefulness,
            required_bps: protocol.minimum_useful_or_modified_bps,
        });
    }
    Ok(RecommendationPromotionDecision::PassShadowGate)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_core::{ForecastRef, ObservationRef, ProfileRef, ReferenceId, ScopeRef};

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn capability(value: &str) -> CapabilityRef {
        CapabilityRef(id(value))
    }

    fn value(mantissa: i128) -> ScaledValue {
        ScaledValue {
            mantissa,
            scale: 0,
            unit: id("unit:count"),
        }
    }

    fn forecast(model: &str, point: i128) -> ShadowForecast {
        ShadowForecast {
            forecast: ForecastRef(id(model)),
            model_lineage: id(model),
            target: ForecastTarget {
                metric: id("metric:demand"),
                scope: ScopeRef(id("location:a")),
                unit: id("unit:count"),
                scale: 0,
                window_start_unix_ms: 2_500,
                window_end_unix_ms: 3_000,
            },
            issued_at_unix_ms: 2_200,
            disposition: ForecastDisposition::Predicted(ForecastValue {
                point: value(point),
                lower: value(point - 5),
                upper: value(point + 5),
            }),
        }
    }

    fn actual() -> MetricObservation {
        MetricObservation {
            observation: ObservationRef(id("observation:actual")),
            source_system: id("source:pos"),
            source_event_id: id("event:actual"),
            source_payload_digest: Digest32::repeat(1),
            mapping_digest: Digest32::repeat(2),
            metric: id("metric:demand"),
            scope: ScopeRef(id("location:a")),
            value: value(100),
            observed_at_unix_ms: 3_100,
        }
    }

    fn protocol() -> ForecastQualificationProtocol {
        ForecastQualificationProtocol {
            protocol_id: id("protocol:forecast:1"),
            profile: ProfileRef(id("profile:hospitality")),
            capability: capability("forecast:demand"),
            candidate_model_lineage: id("model:candidate"),
            baseline_model_lineage: id("model:baseline"),
            registered_at_unix_ms: 1_000,
            evaluation_start_unix_ms: 2_000,
            evaluation_end_unix_ms: 4_000,
            minimum_cases: 1,
            maximum_abstention_bps: 1_000,
            require_candidate_not_worse_than_baseline: true,
            protocol_digest: Digest32::repeat(9),
        }
    }

    #[test]
    fn scorecard_rejects_wrong_candidate_model() {
        let case = ForecastCase {
            candidate: forecast("model:other", 99),
            baseline: forecast("model:baseline", 95),
            actual: actual(),
        };
        assert_eq!(
            ForecastScorecard::score_for_protocol(&protocol(), &[case]),
            Err(ForecastScoringError::CandidateModelMismatch { index: 0 })
        );
    }

    #[test]
    fn scorecard_cannot_be_relabelled_under_another_protocol() {
        let case = ForecastCase {
            candidate: forecast("model:candidate", 99),
            baseline: forecast("model:baseline", 95),
            actual: actual(),
        };
        let score = ForecastScorecard::score_for_protocol(&protocol(), &[case]).unwrap();
        let mut other = protocol();
        other.protocol_digest = Digest32::repeat(10);
        assert_eq!(
            evaluate_forecast_gate(&other, &score),
            Err(ForecastGateError::ScorecardProtocolMismatch)
        );
    }

    #[test]
    fn case_must_be_inside_registered_holdout_window() {
        let mut case = ForecastCase {
            candidate: forecast("model:candidate", 99),
            baseline: forecast("model:baseline", 95),
            actual: actual(),
        };
        case.candidate.target.window_end_unix_ms = 4_100;
        case.baseline.target = case.candidate.target.clone();
        assert_eq!(
            ForecastScorecard::score_for_protocol(&protocol(), &[case]),
            Err(ForecastScoringError::CaseOutsideEvaluationWindow { index: 0 })
        );
    }
}

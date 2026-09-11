// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Read-only shadow qualification for Mycelix Business Fabric intelligence.
//!
//! This crate deliberately has no execution mode and mints no authority.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    CapabilityRef, Digest32, ForecastRef, ObservationRef, ProfileRef, ProposalRef, ReferenceId,
    ScopeRef,
};
use mycelix_business_decision::DecisionCapsuleRef;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ShadowMode {
    Observe,
    Estimate,
    Forecast,
    Recommend,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShadowCapability {
    pub capability: CapabilityRef,
    pub mode: ShadowMode,
}

/// Evidence about one external source event. No raw payload is copied here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceWitness {
    pub source_system: ReferenceId,
    pub source_event_id: ReferenceId,
    pub payload_digest: Digest32,
    pub observed_at_unix_ms: u64,
    pub ingested_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WitnessError {
    ZeroPayloadDigest,
    ZeroObservedTime,
    ZeroIngestedTime,
    PayloadSubstitution {
        source_system: ReferenceId,
        source_event_id: ReferenceId,
    },
}

impl SourceWitness {
    pub fn validate(&self) -> Result<(), WitnessError> {
        if self.payload_digest == Digest32([0; 32]) {
            return Err(WitnessError::ZeroPayloadDigest);
        }
        if self.observed_at_unix_ms == 0 {
            return Err(WitnessError::ZeroObservedTime);
        }
        if self.ingested_at_unix_ms == 0 {
            return Err(WitnessError::ZeroIngestedTime);
        }
        Ok(())
    }
}

/// Rejects same-provider/same-event payload substitution while allowing idempotent replay.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct WitnessRegistry {
    payloads: BTreeMap<(ReferenceId, ReferenceId), Digest32>,
}

impl WitnessRegistry {
    pub fn accept(&mut self, witness: &SourceWitness) -> Result<(), WitnessError> {
        witness.validate()?;
        let key = (
            witness.source_system.clone(),
            witness.source_event_id.clone(),
        );
        match self.payloads.get(&key) {
            Some(existing) if existing != &witness.payload_digest => {
                Err(WitnessError::PayloadSubstitution {
                    source_system: key.0,
                    source_event_id: key.1,
                })
            }
            Some(_) => Ok(()),
            None => {
                self.payloads.insert(key, witness.payload_digest);
                Ok(())
            }
        }
    }
}

/// Deterministic fixed-point value. `mantissa * 10^-scale` in `unit`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScaledValue {
    pub mantissa: i128,
    pub scale: u32,
    pub unit: ReferenceId,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScaledValueError {
    ExcessiveScale,
    UnitMismatch,
    ScaleMismatch,
    InvalidInterval,
}

impl ScaledValue {
    pub const MAX_SCALE: u32 = 18;

    pub fn validate(&self) -> Result<(), ScaledValueError> {
        if self.scale > Self::MAX_SCALE {
            return Err(ScaledValueError::ExcessiveScale);
        }
        Ok(())
    }

    fn validate_compatible(&self, other: &Self) -> Result<(), ScaledValueError> {
        self.validate()?;
        other.validate()?;
        if self.unit != other.unit {
            return Err(ScaledValueError::UnitMismatch);
        }
        if self.scale != other.scale {
            return Err(ScaledValueError::ScaleMismatch);
        }
        Ok(())
    }
}

/// A normalized metric observation derived from a witness through one explicit mapping.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MetricObservation {
    pub observation: ObservationRef,
    pub source_system: ReferenceId,
    pub source_event_id: ReferenceId,
    pub source_payload_digest: Digest32,
    pub mapping_digest: Digest32,
    pub metric: ReferenceId,
    pub scope: ScopeRef,
    pub value: ScaledValue,
    pub observed_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MetricObservationError {
    ZeroSourcePayloadDigest,
    ZeroMappingDigest,
    ZeroObservedTime,
    InvalidValue(ScaledValueError),
}

impl MetricObservation {
    pub fn validate(&self) -> Result<(), MetricObservationError> {
        if self.source_payload_digest == Digest32([0; 32]) {
            return Err(MetricObservationError::ZeroSourcePayloadDigest);
        }
        if self.mapping_digest == Digest32([0; 32]) {
            return Err(MetricObservationError::ZeroMappingDigest);
        }
        if self.observed_at_unix_ms == 0 {
            return Err(MetricObservationError::ZeroObservedTime);
        }
        self.value
            .validate()
            .map_err(MetricObservationError::InvalidValue)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastTarget {
    pub metric: ReferenceId,
    pub scope: ScopeRef,
    pub unit: ReferenceId,
    pub scale: u32,
    pub window_start_unix_ms: u64,
    pub window_end_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastTargetError {
    InvalidWindow,
    ExcessiveScale,
}

impl ForecastTarget {
    pub fn validate(&self) -> Result<(), ForecastTargetError> {
        if self.window_start_unix_ms == 0
            || self.window_start_unix_ms >= self.window_end_unix_ms
        {
            return Err(ForecastTargetError::InvalidWindow);
        }
        if self.scale > ScaledValue::MAX_SCALE {
            return Err(ForecastTargetError::ExcessiveScale);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastValue {
    pub point: ScaledValue,
    pub lower: ScaledValue,
    pub upper: ScaledValue,
}

impl ForecastValue {
    pub fn validate_for_target(&self, target: &ForecastTarget) -> Result<(), ScaledValueError> {
        self.point.validate()?;
        self.lower.validate_compatible(&self.point)?;
        self.upper.validate_compatible(&self.point)?;
        if self.point.unit != target.unit {
            return Err(ScaledValueError::UnitMismatch);
        }
        if self.point.scale != target.scale {
            return Err(ScaledValueError::ScaleMismatch);
        }
        if self.lower.mantissa > self.point.mantissa || self.point.mantissa > self.upper.mantissa {
            return Err(ScaledValueError::InvalidInterval);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastDisposition {
    Predicted(ForecastValue),
    Abstained { reason: ReferenceId },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShadowForecast {
    pub forecast: ForecastRef,
    pub model_lineage: ReferenceId,
    pub target: ForecastTarget,
    pub issued_at_unix_ms: u64,
    pub disposition: ForecastDisposition,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ShadowForecastError {
    InvalidTarget(ForecastTargetError),
    IssuedAfterTargetStart,
    InvalidPrediction(ScaledValueError),
}

impl ShadowForecast {
    pub fn validate(&self) -> Result<(), ShadowForecastError> {
        self.target
            .validate()
            .map_err(ShadowForecastError::InvalidTarget)?;
        if self.issued_at_unix_ms == 0 || self.issued_at_unix_ms >= self.target.window_start_unix_ms {
            return Err(ShadowForecastError::IssuedAfterTargetStart);
        }
        if let ForecastDisposition::Predicted(value) = &self.disposition {
            value
                .validate_for_target(&self.target)
                .map_err(ShadowForecastError::InvalidPrediction)?;
        }
        Ok(())
    }
}

/// Protocol registration precedes the evaluation window to prevent metric/threshold shopping.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastQualificationProtocol {
    pub protocol_id: ReferenceId,
    pub profile: ProfileRef,
    pub capability: CapabilityRef,
    pub candidate_model_lineage: ReferenceId,
    pub baseline_model_lineage: ReferenceId,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    pub minimum_cases: u32,
    pub maximum_abstention_bps: u16,
    pub require_candidate_not_worse_than_baseline: bool,
    pub protocol_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastProtocolError {
    ZeroProtocolDigest,
    InvalidEvaluationWindow,
    NotPreregistered,
    ZeroMinimumCases,
    InvalidAbstentionLimit,
}

impl ForecastQualificationProtocol {
    pub fn validate(&self) -> Result<(), ForecastProtocolError> {
        if self.protocol_digest == Digest32([0; 32]) {
            return Err(ForecastProtocolError::ZeroProtocolDigest);
        }
        if self.evaluation_start_unix_ms == 0
            || self.evaluation_start_unix_ms >= self.evaluation_end_unix_ms
        {
            return Err(ForecastProtocolError::InvalidEvaluationWindow);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.evaluation_start_unix_ms
        {
            return Err(ForecastProtocolError::NotPreregistered);
        }
        if self.minimum_cases == 0 {
            return Err(ForecastProtocolError::ZeroMinimumCases);
        }
        if self.maximum_abstention_bps > 10_000 {
            return Err(ForecastProtocolError::InvalidAbstentionLimit);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastCase {
    pub candidate: ShadowForecast,
    pub baseline: ShadowForecast,
    pub actual: MetricObservation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastCaseError {
    Candidate(ShadowForecastError),
    Baseline(ShadowForecastError),
    Actual(MetricObservationError),
    TargetMismatch,
    BaselineAbstained,
    ActualMetricMismatch,
    ActualScopeMismatch,
    ActualUnitMismatch,
    ActualScaleMismatch,
    ActualBeforeWindowEnd,
}

impl ForecastCase {
    pub fn validate(&self) -> Result<(), ForecastCaseError> {
        self.candidate
            .validate()
            .map_err(ForecastCaseError::Candidate)?;
        self.baseline
            .validate()
            .map_err(ForecastCaseError::Baseline)?;
        self.actual.validate().map_err(ForecastCaseError::Actual)?;
        if self.candidate.target != self.baseline.target {
            return Err(ForecastCaseError::TargetMismatch);
        }
        if matches!(self.baseline.disposition, ForecastDisposition::Abstained { .. }) {
            return Err(ForecastCaseError::BaselineAbstained);
        }
        if self.actual.metric != self.candidate.target.metric {
            return Err(ForecastCaseError::ActualMetricMismatch);
        }
        if self.actual.scope != self.candidate.target.scope {
            return Err(ForecastCaseError::ActualScopeMismatch);
        }
        if self.actual.value.unit != self.candidate.target.unit {
            return Err(ForecastCaseError::ActualUnitMismatch);
        }
        if self.actual.value.scale != self.candidate.target.scale {
            return Err(ForecastCaseError::ActualScaleMismatch);
        }
        if self.actual.observed_at_unix_ms < self.candidate.target.window_end_unix_ms {
            return Err(ForecastCaseError::ActualBeforeWindowEnd);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ForecastScorecard {
    pub cases: u32,
    pub candidate_abstentions: u32,
    pub candidate_absolute_error_sum: u128,
    pub baseline_absolute_error_sum: u128,
    pub candidate_interval_hits: u32,
    pub candidate_interval_trials: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastScoringError {
    InvalidCase { index: usize, error: ForecastCaseError },
}

impl ForecastScorecard {
    pub fn score(cases: &[ForecastCase]) -> Result<Self, ForecastScoringError> {
        let mut result = Self::default();
        for (index, case) in cases.iter().enumerate() {
            case.validate()
                .map_err(|error| ForecastScoringError::InvalidCase { index, error })?;

            let actual = case.actual.value.mantissa;
            let ForecastDisposition::Predicted(baseline_value) = &case.baseline.disposition else {
                unreachable!("ForecastCase::validate rejects baseline abstention");
            };
            let baseline_error = baseline_value.point.mantissa.abs_diff(actual);
            result.baseline_absolute_error_sum = result
                .baseline_absolute_error_sum
                .saturating_add(baseline_error);

            match &case.candidate.disposition {
                ForecastDisposition::Predicted(candidate_value) => {
                    result.candidate_absolute_error_sum = result
                        .candidate_absolute_error_sum
                        .saturating_add(candidate_value.point.mantissa.abs_diff(actual));
                    result.candidate_interval_trials += 1;
                    if candidate_value.lower.mantissa <= actual && actual <= candidate_value.upper.mantissa {
                        result.candidate_interval_hits += 1;
                    }
                }
                ForecastDisposition::Abstained { .. } => {
                    // Conservative anti-gaming rule: abstention receives baseline error.
                    result.candidate_abstentions += 1;
                    result.candidate_absolute_error_sum = result
                        .candidate_absolute_error_sum
                        .saturating_add(baseline_error);
                }
            }
            result.cases += 1;
        }
        Ok(result)
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
pub enum ShadowPromotionDecision {
    PassShadowGate,
    InsufficientCases { actual: u32, required: u32 },
    ExcessiveAbstention { actual_bps: u16, maximum_bps: u16 },
    CandidateWorseThanBaseline,
}

pub fn evaluate_forecast_gate(
    protocol: &ForecastQualificationProtocol,
    scorecard: &ForecastScorecard,
) -> Result<ShadowPromotionDecision, ForecastProtocolError> {
    protocol.validate()?;
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
pub struct ShadowRecommendation {
    pub decision: DecisionCapsuleRef,
    pub proposal: ProposalRef,
    pub capability: CapabilityRef,
    pub issued_at_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReviewDisposition {
    Useful,
    Modified,
    NotUseful,
    Unsafe,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecommendationReview {
    pub recommendation: ShadowRecommendation,
    pub disposition: ReviewDisposition,
    pub reviewer_evidence: ReferenceId,
    pub reviewed_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecommendationReviewError {
    ReviewBeforeRecommendation,
    ZeroRecommendationTime,
    ZeroReviewTime,
}

impl RecommendationReview {
    pub fn validate(&self) -> Result<(), RecommendationReviewError> {
        if self.recommendation.issued_at_unix_ms == 0 {
            return Err(RecommendationReviewError::ZeroRecommendationTime);
        }
        if self.reviewed_at_unix_ms == 0 {
            return Err(RecommendationReviewError::ZeroReviewTime);
        }
        if self.reviewed_at_unix_ms < self.recommendation.issued_at_unix_ms {
            return Err(RecommendationReviewError::ReviewBeforeRecommendation);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecommendationQualificationProtocol {
    pub protocol_id: ReferenceId,
    pub profile: ProfileRef,
    pub capability: CapabilityRef,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    pub minimum_reviews: u32,
    pub minimum_useful_or_modified_bps: u16,
    pub maximum_unsafe_reviews: u32,
    pub protocol_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecommendationProtocolError {
    ZeroProtocolDigest,
    InvalidEvaluationWindow,
    NotPreregistered,
    ZeroMinimumReviews,
    InvalidUsefulnessThreshold,
}

impl RecommendationQualificationProtocol {
    pub fn validate(&self) -> Result<(), RecommendationProtocolError> {
        if self.protocol_digest == Digest32([0; 32]) {
            return Err(RecommendationProtocolError::ZeroProtocolDigest);
        }
        if self.evaluation_start_unix_ms == 0
            || self.evaluation_start_unix_ms >= self.evaluation_end_unix_ms
        {
            return Err(RecommendationProtocolError::InvalidEvaluationWindow);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.evaluation_start_unix_ms
        {
            return Err(RecommendationProtocolError::NotPreregistered);
        }
        if self.minimum_reviews == 0 {
            return Err(RecommendationProtocolError::ZeroMinimumReviews);
        }
        if self.minimum_useful_or_modified_bps > 10_000 {
            return Err(RecommendationProtocolError::InvalidUsefulnessThreshold);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RecommendationScorecard {
    pub reviewed: u32,
    pub useful: u32,
    pub modified: u32,
    pub not_useful: u32,
    pub unsafe_reviews: u32,
}

impl RecommendationScorecard {
    pub fn score(reviews: &[RecommendationReview]) -> Result<Self, RecommendationReviewError> {
        let mut result = Self::default();
        for review in reviews {
            review.validate()?;
            result.reviewed += 1;
            match review.disposition {
                ReviewDisposition::Useful => result.useful += 1,
                ReviewDisposition::Modified => result.modified += 1,
                ReviewDisposition::NotUseful => result.not_useful += 1,
                ReviewDisposition::Unsafe => result.unsafe_reviews += 1,
            }
        }
        Ok(result)
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
pub enum RecommendationPromotionDecision {
    PassShadowGate,
    InsufficientReviews { actual: u32, required: u32 },
    ExcessiveUnsafeReviews { actual: u32, maximum: u32 },
    InsufficientUsefulness { actual_bps: u16, required_bps: u16 },
}

pub fn evaluate_recommendation_gate(
    protocol: &RecommendationQualificationProtocol,
    scorecard: &RecommendationScorecard,
) -> Result<RecommendationPromotionDecision, RecommendationProtocolError> {
    protocol.validate()?;
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

/// Shadow qualification is intentionally capped at recommendation-only behavior.
pub const fn maximum_shadow_mode() -> ShadowMode {
    ShadowMode::Recommend
}

pub fn unique_capabilities(capabilities: &[ShadowCapability]) -> bool {
    capabilities
        .iter()
        .map(|value| value.capability.clone())
        .collect::<BTreeSet<_>>()
        .len()
        == capabilities.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn capability(value: &str) -> CapabilityRef {
        CapabilityRef(id(value))
    }

    fn scope(value: &str) -> ScopeRef {
        ScopeRef(id(value))
    }

    fn observation(value: &str) -> ObservationRef {
        ObservationRef(id(value))
    }

    fn forecast_ref(value: &str) -> ForecastRef {
        ForecastRef(id(value))
    }

    fn value(mantissa: i128) -> ScaledValue {
        ScaledValue {
            mantissa,
            scale: 0,
            unit: id("unit:count"),
        }
    }

    fn target() -> ForecastTarget {
        ForecastTarget {
            metric: id("metric:demand"),
            scope: scope("location:a"),
            unit: id("unit:count"),
            scale: 0,
            window_start_unix_ms: 2_000,
            window_end_unix_ms: 3_000,
        }
    }

    fn predicted(name: &str, point: i128, lower: i128, upper: i128) -> ShadowForecast {
        ShadowForecast {
            forecast: forecast_ref(name),
            model_lineage: id(name),
            target: target(),
            issued_at_unix_ms: 1_500,
            disposition: ForecastDisposition::Predicted(ForecastValue {
                point: value(point),
                lower: value(lower),
                upper: value(upper),
            }),
        }
    }

    fn actual(mantissa: i128) -> MetricObservation {
        MetricObservation {
            observation: observation("obs:actual:1"),
            source_system: id("source:pos"),
            source_event_id: id("event:1"),
            source_payload_digest: Digest32::repeat(1),
            mapping_digest: Digest32::repeat(2),
            metric: id("metric:demand"),
            scope: scope("location:a"),
            value: value(mantissa),
            observed_at_unix_ms: 3_100,
        }
    }

    #[test]
    fn witness_registry_rejects_same_identity_payload_substitution() {
        let mut registry = WitnessRegistry::default();
        let witness = SourceWitness {
            source_system: id("source:pos"),
            source_event_id: id("sale:1"),
            payload_digest: Digest32::repeat(1),
            observed_at_unix_ms: 10,
            ingested_at_unix_ms: 11,
        };
        registry.accept(&witness).unwrap();
        registry.accept(&witness).unwrap();
        let substituted = SourceWitness {
            payload_digest: Digest32::repeat(2),
            ..witness
        };
        assert!(matches!(
            registry.accept(&substituted),
            Err(WitnessError::PayloadSubstitution { .. })
        ));
    }

    #[test]
    fn forecast_cannot_be_issued_after_target_begins() {
        let mut forecast = predicted("model:candidate", 10, 8, 12);
        forecast.issued_at_unix_ms = 2_000;
        assert_eq!(
            forecast.validate(),
            Err(ShadowForecastError::IssuedAfterTargetStart)
        );
    }

    #[test]
    fn abstention_cannot_improve_error_by_dropping_a_hard_case() {
        let candidate = ShadowForecast {
            forecast: forecast_ref("forecast:candidate"),
            model_lineage: id("model:candidate"),
            target: target(),
            issued_at_unix_ms: 1_500,
            disposition: ForecastDisposition::Abstained {
                reason: id("reason:uncertain"),
            },
        };
        let case = ForecastCase {
            candidate,
            baseline: predicted("model:baseline", 80, 70, 90),
            actual: actual(100),
        };
        let score = ForecastScorecard::score(&[case]).unwrap();
        assert_eq!(score.candidate_abstentions, 1);
        assert_eq!(score.candidate_absolute_error_sum, 20);
        assert_eq!(score.baseline_absolute_error_sum, 20);
    }

    #[test]
    fn protocol_must_be_registered_before_evaluation() {
        let protocol = ForecastQualificationProtocol {
            protocol_id: id("protocol:1"),
            profile: ProfileRef(id("profile:hospitality")),
            capability: capability("forecast:demand"),
            candidate_model_lineage: id("model:candidate"),
            baseline_model_lineage: id("model:baseline"),
            registered_at_unix_ms: 2_000,
            evaluation_start_unix_ms: 2_000,
            evaluation_end_unix_ms: 3_000,
            minimum_cases: 1,
            maximum_abstention_bps: 1_000,
            require_candidate_not_worse_than_baseline: true,
            protocol_digest: Digest32::repeat(3),
        };
        assert_eq!(
            protocol.validate(),
            Err(ForecastProtocolError::NotPreregistered)
        );
    }

    #[test]
    fn worse_candidate_fails_baseline_gate() {
        let case = ForecastCase {
            candidate: predicted("model:candidate", 70, 60, 80),
            baseline: predicted("model:baseline", 95, 90, 100),
            actual: actual(100),
        };
        let score = ForecastScorecard::score(&[case]).unwrap();
        let protocol = ForecastQualificationProtocol {
            protocol_id: id("protocol:1"),
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
            protocol_digest: Digest32::repeat(3),
        };
        assert_eq!(
            evaluate_forecast_gate(&protocol, &score).unwrap(),
            ShadowPromotionDecision::CandidateWorseThanBaseline
        );
    }

    #[test]
    fn unsafe_recommendation_review_blocks_shadow_gate() {
        let recommendation = ShadowRecommendation {
            decision: DecisionCapsuleRef {
                id: id("decision:1"),
                digest: Digest32::repeat(4),
            },
            proposal: ProposalRef(id("proposal:1")),
            capability: capability("recommend:replenishment"),
            issued_at_unix_ms: 2_100,
        };
        let review = RecommendationReview {
            recommendation,
            disposition: ReviewDisposition::Unsafe,
            reviewer_evidence: id("review:1"),
            reviewed_at_unix_ms: 2_200,
        };
        let score = RecommendationScorecard::score(&[review]).unwrap();
        let protocol = RecommendationQualificationProtocol {
            protocol_id: id("protocol:recommend:1"),
            profile: ProfileRef(id("profile:hospitality")),
            capability: capability("recommend:replenishment"),
            registered_at_unix_ms: 1_000,
            evaluation_start_unix_ms: 2_000,
            evaluation_end_unix_ms: 4_000,
            minimum_reviews: 1,
            minimum_useful_or_modified_bps: 0,
            maximum_unsafe_reviews: 0,
            protocol_digest: Digest32::repeat(5),
        };
        assert_eq!(
            evaluate_recommendation_gate(&protocol, &score).unwrap(),
            RecommendationPromotionDecision::ExcessiveUnsafeReviews {
                actual: 1,
                maximum: 0,
            }
        );
    }

    #[test]
    fn shadow_mode_has_no_execution_variant() {
        assert_eq!(maximum_shadow_mode(), ShadowMode::Recommend);
        let capabilities = vec![
            ShadowCapability {
                capability: capability("observe:sales"),
                mode: ShadowMode::Observe,
            },
            ShadowCapability {
                capability: capability("forecast:demand"),
                mode: ShadowMode::Forecast,
            },
            ShadowCapability {
                capability: capability("recommend:prep"),
                mode: ShadowMode::Recommend,
            },
        ];
        assert!(unique_capabilities(&capabilities));
    }
}

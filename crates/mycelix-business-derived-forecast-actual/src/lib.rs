// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic transaction-level projection into forecast evaluation actuals.
//!
//! A derived actual is deliberately not a raw `MetricObservation`. It is an evidence-bound
//! deterministic aggregate over witness observations selected by a preregistered target plan.

use std::collections::BTreeSet;

use mycelix_business_campaign_replay::{CampaignReplay, ReplayedObservation};
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_forecast_plan::{ForecastPlanError, ForecastTargetPlan};
use mycelix_business_shadow::{
    ForecastDisposition, ForecastQualificationProtocol, ForecastScorecard, ForecastTarget,
    ScaledValue, ShadowForecast,
};
use sha2::{Digest, Sha256};

pub const DERIVED_ACTUAL_IS_NOT_RAW_OBSERVATION: bool = true;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(64);
    for byte in digest.0 {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EvaluationActualRef(pub ReferenceId);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AggregationKind {
    Sum,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActualProjectionSpec {
    pub projection_id: ReferenceId,
    pub target_plan_digest: Digest32,
    pub source_input: ReferenceId,
    pub aggregation: AggregationKind,
    pub registered_at_unix_ms: u64,
    pub spec_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProjectionSpecError {
    InvalidTargetPlan(ForecastPlanError),
    ZeroPlanDigest,
    ZeroSpecDigest,
    NotPreregistered,
    RegisteredAfterTargetPlan,
    PlanDigestMismatch,
    DigestMismatch,
}

impl ActualProjectionSpec {
    pub fn build(
        protocol: &ForecastQualificationProtocol,
        plan: &ForecastTargetPlan,
        projection_id: ReferenceId,
        source_input: ReferenceId,
        aggregation: AggregationKind,
        registered_at_unix_ms: u64,
    ) -> Result<Self, ProjectionSpecError> {
        plan.validate_against(protocol)
            .map_err(ProjectionSpecError::InvalidTargetPlan)?;
        let mut spec = Self {
            projection_id,
            target_plan_digest: plan.plan_digest,
            source_input,
            aggregation,
            registered_at_unix_ms,
            spec_digest: Digest32([0; 32]),
        };
        spec.spec_digest = projection_spec_digest(&spec);
        spec.validate_against(protocol, plan)?;
        Ok(spec)
    }

    pub fn validate_against(
        &self,
        protocol: &ForecastQualificationProtocol,
        plan: &ForecastTargetPlan,
    ) -> Result<(), ProjectionSpecError> {
        plan.validate_against(protocol)
            .map_err(ProjectionSpecError::InvalidTargetPlan)?;
        if zero_digest(&self.target_plan_digest) {
            return Err(ProjectionSpecError::ZeroPlanDigest);
        }
        if zero_digest(&self.spec_digest) {
            return Err(ProjectionSpecError::ZeroSpecDigest);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= plan.evaluation_start_unix_ms
        {
            return Err(ProjectionSpecError::NotPreregistered);
        }
        if self.registered_at_unix_ms > plan.registered_at_unix_ms {
            return Err(ProjectionSpecError::RegisteredAfterTargetPlan);
        }
        if self.target_plan_digest != plan.plan_digest {
            return Err(ProjectionSpecError::PlanDigestMismatch);
        }
        if self.spec_digest != projection_spec_digest(self) {
            return Err(ProjectionSpecError::DigestMismatch);
        }
        Ok(())
    }
}

fn projection_spec_digest(spec: &ActualProjectionSpec) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:actual-projection-spec:v1");
    hash_str(&mut hasher, spec.projection_id.as_str());
    hasher.update(spec.target_plan_digest.0);
    hash_str(&mut hasher, spec.source_input.as_str());
    hasher.update([match spec.aggregation {
        AggregationKind::Sum => 1,
    }]);
    hasher.update(spec.registered_at_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DerivedMetricActual {
    pub actual: EvaluationActualRef,
    pub target_id: ReferenceId,
    pub target: ForecastTarget,
    pub value: ScaledValue,
    pub projection_spec_digest: Digest32,
    pub target_plan_digest: Digest32,
    pub campaign_replay_digest: Digest32,
    pub source_observation_count: u64,
    pub source_observation_set_digest: Digest32,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DerivedActualError {
    Projection(ProjectionSpecError),
    ReplayEvidenceInvalid,
    SourceObservationInvalid,
    IncompatibleSourceObservation { target: ReferenceId },
    DuplicateSourceObservationIdentity,
    ArithmeticOverflow { target: ReferenceId },
    CountOverflow,
    DigestMismatch,
}

pub fn derive_actuals(
    protocol: &ForecastQualificationProtocol,
    plan: &ForecastTargetPlan,
    spec: &ActualProjectionSpec,
    replay: &CampaignReplay,
) -> Result<Vec<DerivedMetricActual>, DerivedActualError> {
    spec.validate_against(protocol, plan)
        .map_err(DerivedActualError::Projection)?;
    if zero_digest(&replay.evidence.evidence_digest)
        || zero_digest(&replay.evidence.observation_set_digest)
    {
        return Err(DerivedActualError::ReplayEvidenceInvalid);
    }

    let mut seen_source_observations = BTreeSet::new();
    for replayed in &replay.observations {
        replayed
            .observation
            .validate()
            .map_err(|_| DerivedActualError::SourceObservationInvalid)?;
        if !seen_source_observations.insert(replayed.observation.observation.clone()) {
            return Err(DerivedActualError::DuplicateSourceObservationIdentity);
        }
    }

    let mut result = Vec::with_capacity(plan.targets.len());
    for planned in &plan.targets {
        let mut sum = 0_i128;
        let mut source_digests = Vec::new();
        let target = &planned.target;
        for replayed in &replay.observations {
            if replayed.input != spec.source_input
                || replayed.observation.metric != target.metric
                || replayed.observation.scope != target.scope
                || replayed.observation.observed_at_unix_ms < target.window_start_unix_ms
                || replayed.observation.observed_at_unix_ms >= target.window_end_unix_ms
            {
                continue;
            }
            if replayed.observation.value.unit != target.unit
                || replayed.observation.value.scale != target.scale
            {
                return Err(DerivedActualError::IncompatibleSourceObservation {
                    target: planned.target_id.clone(),
                });
            }
            match spec.aggregation {
                AggregationKind::Sum => {
                    sum = sum
                        .checked_add(replayed.observation.value.mantissa)
                        .ok_or_else(|| DerivedActualError::ArithmeticOverflow {
                            target: planned.target_id.clone(),
                        })?;
                }
            }
            source_digests.push(source_observation_digest(replayed));
        }
        source_digests.sort_unstable();
        let source_observation_count = u64::try_from(source_digests.len())
            .map_err(|_| DerivedActualError::CountOverflow)?;
        let source_observation_set_digest = source_set_digest(&source_digests);
        let value = ScaledValue {
            mantissa: sum,
            scale: target.scale,
            unit: target.unit.clone(),
        };
        let identity_digest = actual_identity_digest(
            spec.spec_digest,
            plan.plan_digest,
            replay.evidence.evidence_digest,
            &planned.target_id,
            target,
            &value,
            source_observation_count,
            source_observation_set_digest,
        );
        let actual = EvaluationActualRef(
            ReferenceId::new(format!("evaluation-actual:sha256:{}", hex_digest(identity_digest)))
                .expect("generated evaluation actual id is canonical"),
        );
        let mut derived = DerivedMetricActual {
            actual,
            target_id: planned.target_id.clone(),
            target: target.clone(),
            value,
            projection_spec_digest: spec.spec_digest,
            target_plan_digest: plan.plan_digest,
            campaign_replay_digest: replay.evidence.evidence_digest,
            source_observation_count,
            source_observation_set_digest,
            evidence_digest: Digest32([0; 32]),
        };
        derived.evidence_digest = derived_actual_evidence_digest(&derived);
        result.push(derived);
    }
    Ok(result)
}

impl DerivedMetricActual {
    pub fn validate_against(
        &self,
        plan: &ForecastTargetPlan,
        spec: &ActualProjectionSpec,
        replay: &CampaignReplay,
    ) -> Result<(), DerivedActualError> {
        let Some(planned) = plan.targets.iter().find(|item| item.target_id == self.target_id) else {
            return Err(DerivedActualError::DigestMismatch);
        };
        if self.target != planned.target
            || self.value.unit != self.target.unit
            || self.value.scale != self.target.scale
            || self.projection_spec_digest != spec.spec_digest
            || self.target_plan_digest != plan.plan_digest
            || self.campaign_replay_digest != replay.evidence.evidence_digest
            || zero_digest(&self.source_observation_set_digest)
            || zero_digest(&self.evidence_digest)
            || self.evidence_digest != derived_actual_evidence_digest(self)
        {
            return Err(DerivedActualError::DigestMismatch);
        }
        Ok(())
    }
}

fn source_observation_digest(value: &ReplayedObservation) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:evaluation-source-observation:v1");
    hash_str(&mut hasher, value.input.as_str());
    hash_str(&mut hasher, value.observation.observation.as_ref_id().as_str());
    hash_str(&mut hasher, value.observation.source_system.as_str());
    hash_str(&mut hasher, value.observation.source_event_id.as_str());
    hasher.update(value.observation.source_payload_digest.0);
    hasher.update(value.observation.mapping_digest.0);
    hash_str(&mut hasher, value.observation.metric.as_str());
    hash_str(&mut hasher, value.observation.scope.as_ref_id().as_str());
    hash_scaled(&mut hasher, &value.observation.value);
    hasher.update(value.observation.observed_at_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

fn source_set_digest(source_digests: &[Digest32]) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:evaluation-source-set:v1");
    hasher.update((source_digests.len() as u64).to_be_bytes());
    for digest in source_digests {
        hasher.update(digest.0);
    }
    finish_digest(hasher)
}

#[allow(clippy::too_many_arguments)]
fn actual_identity_digest(
    spec_digest: Digest32,
    plan_digest: Digest32,
    replay_digest: Digest32,
    target_id: &ReferenceId,
    target: &ForecastTarget,
    value: &ScaledValue,
    source_count: u64,
    source_set_digest: Digest32,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:evaluation-actual-identity:v1");
    hasher.update(spec_digest.0);
    hasher.update(plan_digest.0);
    hasher.update(replay_digest.0);
    hash_str(&mut hasher, target_id.as_str());
    hash_target(&mut hasher, target);
    hash_scaled(&mut hasher, value);
    hasher.update(source_count.to_be_bytes());
    hasher.update(source_set_digest.0);
    finish_digest(hasher)
}

fn derived_actual_evidence_digest(actual: &DerivedMetricActual) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:derived-evaluation-actual:v1");
    hash_str(&mut hasher, actual.actual.0.as_str());
    hash_str(&mut hasher, actual.target_id.as_str());
    hash_target(&mut hasher, &actual.target);
    hash_scaled(&mut hasher, &actual.value);
    hasher.update(actual.projection_spec_digest.0);
    hasher.update(actual.target_plan_digest.0);
    hasher.update(actual.campaign_replay_digest.0);
    hasher.update(actual.source_observation_count.to_be_bytes());
    hasher.update(actual.source_observation_set_digest.0);
    finish_digest(hasher)
}

fn hash_target(hasher: &mut Sha256, target: &ForecastTarget) {
    hash_str(hasher, target.metric.as_str());
    hash_str(hasher, target.scope.as_ref_id().as_str());
    hash_str(hasher, target.unit.as_str());
    hasher.update(target.scale.to_be_bytes());
    hasher.update(target.window_start_unix_ms.to_be_bytes());
    hasher.update(target.window_end_unix_ms.to_be_bytes());
}

fn hash_scaled(hasher: &mut Sha256, value: &ScaledValue) {
    hasher.update(value.mantissa.to_be_bytes());
    hasher.update(value.scale.to_be_bytes());
    hash_str(hasher, value.unit.as_str());
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DerivedForecastCase {
    pub candidate: ShadowForecast,
    pub baseline: ShadowForecast,
    pub actual: DerivedMetricActual,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DerivedCaseError {
    CandidateInvalid,
    BaselineInvalid,
    TargetMismatch,
    BaselineAbstained,
    ActualValueMismatch,
}

impl DerivedForecastCase {
    pub fn validate(&self) -> Result<(), DerivedCaseError> {
        self.candidate
            .validate()
            .map_err(|_| DerivedCaseError::CandidateInvalid)?;
        self.baseline
            .validate()
            .map_err(|_| DerivedCaseError::BaselineInvalid)?;
        if self.candidate.target != self.baseline.target
            || self.candidate.target != self.actual.target
        {
            return Err(DerivedCaseError::TargetMismatch);
        }
        if matches!(self.baseline.disposition, ForecastDisposition::Abstained { .. }) {
            return Err(DerivedCaseError::BaselineAbstained);
        }
        if self.actual.value.unit != self.candidate.target.unit
            || self.actual.value.scale != self.candidate.target.scale
        {
            return Err(DerivedCaseError::ActualValueMismatch);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DerivedScoringError {
    InvalidCase { index: usize, error: DerivedCaseError },
}

pub fn score_derived_cases(
    cases: &[DerivedForecastCase],
) -> Result<ForecastScorecard, DerivedScoringError> {
    let mut scorecard = ForecastScorecard::default();
    for (index, case) in cases.iter().enumerate() {
        case.validate()
            .map_err(|error| DerivedScoringError::InvalidCase { index, error })?;
        let actual = case.actual.value.mantissa;
        let ForecastDisposition::Predicted(baseline) = &case.baseline.disposition else {
            unreachable!("DerivedForecastCase::validate rejects baseline abstention");
        };
        let baseline_error = baseline.point.mantissa.abs_diff(actual);
        scorecard.baseline_absolute_error_sum = scorecard
            .baseline_absolute_error_sum
            .saturating_add(baseline_error);
        match &case.candidate.disposition {
            ForecastDisposition::Predicted(candidate) => {
                scorecard.candidate_absolute_error_sum = scorecard
                    .candidate_absolute_error_sum
                    .saturating_add(candidate.point.mantissa.abs_diff(actual));
                scorecard.candidate_interval_trials = scorecard
                    .candidate_interval_trials
                    .saturating_add(1);
                if candidate.lower.mantissa <= actual && actual <= candidate.upper.mantissa {
                    scorecard.candidate_interval_hits = scorecard
                        .candidate_interval_hits
                        .saturating_add(1);
                }
            }
            ForecastDisposition::Abstained { .. } => {
                scorecard.candidate_abstentions = scorecard
                    .candidate_abstentions
                    .saturating_add(1);
                scorecard.candidate_absolute_error_sum = scorecard
                    .candidate_absolute_error_sum
                    .saturating_add(baseline_error);
            }
        }
        scorecard.cases = scorecard.cases.saturating_add(1);
    }
    Ok(scorecard)
}

#[cfg(test)]
mod tests {
    use mycelix_business_campaign_replay::{CampaignReplayEvidence, ReplayedObservation};
    use mycelix_business_core::{ForecastRef, ObservationRef, ProfileRef, ScopeRef};
    use mycelix_business_forecast_plan::{ForecastTargetPlan, PlannedForecastTarget};
    use mycelix_business_ingress::IngressQualificationBinding;
    use mycelix_business_shadow::{
        ForecastQualificationProtocol, ForecastTarget, ForecastValue, MetricObservation,
    };

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn protocol() -> ForecastQualificationProtocol {
        ForecastQualificationProtocol {
            protocol_id: id("protocol:aggregate:v1"),
            profile: ProfileRef(id("profile:hospitality")),
            capability: mycelix_business_core::CapabilityRef(id("capability:forecast:demand")),
            candidate_model_lineage: id("model:candidate:v1"),
            baseline_model_lineage: id("model:baseline:v1"),
            registered_at_unix_ms: 1_000,
            evaluation_start_unix_ms: 2_000,
            evaluation_end_unix_ms: 6_000,
            minimum_cases: 1,
            maximum_abstention_bps: 10_000,
            require_candidate_not_worse_than_baseline: true,
            protocol_digest: Digest32::repeat(1),
        }
    }

    fn target(id_value: &str, start: u64, end: u64) -> PlannedForecastTarget {
        PlannedForecastTarget {
            target_id: id(id_value),
            target: ForecastTarget {
                metric: id("metric:demand"),
                scope: ScopeRef(id("scope:location:a")),
                unit: id("unit:count"),
                scale: 0,
                window_start_unix_ms: start,
                window_end_unix_ms: end,
            },
        }
    }

    fn plan() -> ForecastTargetPlan {
        ForecastTargetPlan::build(
            &protocol(),
            id("target-plan:aggregate:v1"),
            900,
            vec![target("target:one", 2_000, 4_000), target("target:two", 4_000, 6_000)],
        )
        .unwrap()
    }

    fn observation(name: &str, at: u64, value: i128) -> ReplayedObservation {
        ReplayedObservation {
            input: id("input:sales"),
            observation: MetricObservation {
                observation: ObservationRef(id(&format!("observation:{name}"))),
                source_system: id("source:pos"),
                source_event_id: id(&format!("event:{name}")),
                source_payload_digest: Digest32::repeat(name.as_bytes()[0]),
                mapping_digest: Digest32::repeat(3),
                metric: id("metric:demand"),
                scope: ScopeRef(id("scope:location:a")),
                value: ScaledValue {
                    mantissa: value,
                    scale: 0,
                    unit: id("unit:count"),
                },
                observed_at_unix_ms: at,
            },
        }
    }

    fn replay(observations: Vec<ReplayedObservation>) -> CampaignReplay {
        CampaignReplay {
            evidence: CampaignReplayEvidence {
                campaign_digest: Digest32::repeat(4),
                connector: IngressQualificationBinding {
                    source_system: id("source:pos"),
                    adapter_semantic_id: id("adapter:test:v1"),
                    adapter_digest: Digest32::repeat(5),
                    mapping_digest: Digest32::repeat(3),
                    source_schema_digest: Digest32::repeat(6),
                },
                source_file_digests: vec![Digest32::repeat(7)],
                accepted_source_events: observations.len() as u64,
                normalized_observations: observations.len() as u64,
                observation_set_digest: Digest32::repeat(8),
                evidence_digest: Digest32::repeat(9),
            },
            observations,
        }
    }

    fn spec(plan: &ForecastTargetPlan) -> ActualProjectionSpec {
        ActualProjectionSpec::build(
            &protocol(),
            plan,
            id("projection:sum-sales:v1"),
            id("input:sales"),
            AggregationKind::Sum,
            800,
        )
        .unwrap()
    }

    #[test]
    fn sums_transaction_rows_into_exact_target_windows() {
        let plan = plan();
        let replay = replay(vec![
            observation("a", 2_100, 2),
            observation("b", 3_900, 3),
            observation("c", 4_000, 7),
        ]);
        let actuals = derive_actuals(&protocol(), &plan, &spec(&plan), &replay).unwrap();
        assert!(DERIVED_ACTUAL_IS_NOT_RAW_OBSERVATION);
        assert_eq!(actuals[0].value.mantissa, 5);
        assert_eq!(actuals[0].source_observation_count, 2);
        assert_eq!(actuals[1].value.mantissa, 7);
        assert_eq!(actuals[1].source_observation_count, 1);
    }

    #[test]
    fn zero_transaction_window_is_evidence_backed_zero() {
        let plan = plan();
        let replay = replay(vec![observation("a", 2_100, 2)]);
        let actuals = derive_actuals(&protocol(), &plan, &spec(&plan), &replay).unwrap();
        assert_eq!(actuals[1].value.mantissa, 0);
        assert_eq!(actuals[1].source_observation_count, 0);
        assert!(!zero_digest(&actuals[1].source_observation_set_digest));
    }

    #[test]
    fn boundary_event_belongs_only_to_next_half_open_window() {
        let plan = plan();
        let replay = replay(vec![observation("edge", 4_000, 9)]);
        let actuals = derive_actuals(&protocol(), &plan, &spec(&plan), &replay).unwrap();
        assert_eq!(actuals[0].value.mantissa, 0);
        assert_eq!(actuals[1].value.mantissa, 9);
    }

    #[test]
    fn derived_actual_is_not_accepted_as_raw_observation_type() {
        let plan = plan();
        let replay = replay(vec![observation("a", 2_100, 2)]);
        let actuals = derive_actuals(&protocol(), &plan, &spec(&plan), &replay).unwrap();
        assert!(actuals[0].actual.0.as_str().starts_with("evaluation-actual:sha256:"));
    }

    fn forecast(
        name: &str,
        model: ReferenceId,
        target: ForecastTarget,
        point: i128,
    ) -> ShadowForecast {
        let value = ForecastValue {
            point: ScaledValue { mantissa: point, scale: 0, unit: target.unit.clone() },
            lower: ScaledValue { mantissa: point, scale: 0, unit: target.unit.clone() },
            upper: ScaledValue { mantissa: point, scale: 0, unit: target.unit.clone() },
        };
        ShadowForecast {
            forecast: ForecastRef(id(&format!("forecast:{name}"))),
            model_lineage: model,
            target,
            issued_at_unix_ms: 1_500,
            disposition: ForecastDisposition::Predicted(value),
        }
    }

    #[test]
    fn derived_cases_use_same_conservative_scorecard_semantics() {
        let plan = plan();
        let replay = replay(vec![observation("a", 2_100, 2), observation("b", 3_000, 3)]);
        let actual = derive_actuals(&protocol(), &plan, &spec(&plan), &replay)
            .unwrap()
            .remove(0);
        let case = DerivedForecastCase {
            candidate: forecast(
                "candidate",
                protocol().candidate_model_lineage,
                actual.target.clone(),
                5,
            ),
            baseline: forecast(
                "baseline",
                protocol().baseline_model_lineage,
                actual.target.clone(),
                8,
            ),
            actual,
        };
        let score = score_derived_cases(&[case]).unwrap();
        assert_eq!(score.candidate_absolute_error_sum, 0);
        assert_eq!(score.baseline_absolute_error_sum, 3);
    }
}

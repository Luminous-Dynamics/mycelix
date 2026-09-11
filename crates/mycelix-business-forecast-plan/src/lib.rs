// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Preregistered exact forecast-target plans for Mycelix Business qualification.
//!
//! A model must not improve its apparent quality by forecasting only convenient windows. This
//! crate freezes the exact target set before evaluation and proves that the submitted case set
//! covers every registered target exactly once. It grants no authority and performs no execution.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{CapabilityRef, Digest32, ProfileRef, ReferenceId};
use mycelix_business_shadow::{
    ForecastCase, ForecastQualificationProtocol, ForecastTarget, ShadowForecast,
};
use sha2::{Digest, Sha256};

pub const FORECAST_PLAN_IS_NON_AUTHORITATIVE: bool = true;
pub const MAX_PLANNED_TARGETS: usize = 100_000;

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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlannedForecastTarget {
    pub target_id: ReferenceId,
    pub target: ForecastTarget,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastTargetPlan {
    pub plan_id: ReferenceId,
    pub protocol_digest: Digest32,
    pub profile: ProfileRef,
    pub capability: CapabilityRef,
    pub candidate_model_lineage: ReferenceId,
    pub baseline_model_lineage: ReferenceId,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    /// Canonical order by `target_id`.
    pub targets: Vec<PlannedForecastTarget>,
    pub plan_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ForecastPlanError {
    InvalidProtocol,
    ZeroProtocolDigest,
    ZeroPlanDigest,
    NotPreregistered,
    RegistrationAfterProtocol,
    InvalidEvaluationWindow,
    NoTargets,
    TooManyTargets,
    InvalidTarget { index: usize },
    TargetOutsideEvaluationWindow { target: ReferenceId },
    DuplicateTargetId { target: ReferenceId },
    DuplicateTargetSemantics { target: ReferenceId },
    OverlappingTargets { left: ReferenceId, right: ReferenceId },
    ProtocolBindingMismatch,
    NonCanonicalOrder,
    DigestMismatch,
}

impl ForecastTargetPlan {
    pub fn build(
        protocol: &ForecastQualificationProtocol,
        plan_id: ReferenceId,
        registered_at_unix_ms: u64,
        mut targets: Vec<PlannedForecastTarget>,
    ) -> Result<Self, ForecastPlanError> {
        protocol
            .validate()
            .map_err(|_| ForecastPlanError::InvalidProtocol)?;
        targets.sort_by(|left, right| left.target_id.cmp(&right.target_id));
        let mut plan = Self {
            plan_id,
            protocol_digest: protocol.protocol_digest,
            profile: protocol.profile.clone(),
            capability: protocol.capability.clone(),
            candidate_model_lineage: protocol.candidate_model_lineage.clone(),
            baseline_model_lineage: protocol.baseline_model_lineage.clone(),
            registered_at_unix_ms,
            evaluation_start_unix_ms: protocol.evaluation_start_unix_ms,
            evaluation_end_unix_ms: protocol.evaluation_end_unix_ms,
            targets,
            plan_digest: Digest32([0; 32]),
        };
        plan.plan_digest = compute_plan_digest(&plan);
        plan.validate_against(protocol)?;
        Ok(plan)
    }

    pub fn validate_against(
        &self,
        protocol: &ForecastQualificationProtocol,
    ) -> Result<(), ForecastPlanError> {
        protocol
            .validate()
            .map_err(|_| ForecastPlanError::InvalidProtocol)?;
        if zero_digest(&self.protocol_digest) {
            return Err(ForecastPlanError::ZeroProtocolDigest);
        }
        if zero_digest(&self.plan_digest) {
            return Err(ForecastPlanError::ZeroPlanDigest);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.evaluation_start_unix_ms
        {
            return Err(ForecastPlanError::NotPreregistered);
        }
        if self.registered_at_unix_ms > protocol.registered_at_unix_ms {
            return Err(ForecastPlanError::RegistrationAfterProtocol);
        }
        if self.evaluation_start_unix_ms == 0
            || self.evaluation_start_unix_ms >= self.evaluation_end_unix_ms
        {
            return Err(ForecastPlanError::InvalidEvaluationWindow);
        }
        if self.protocol_digest != protocol.protocol_digest
            || self.profile != protocol.profile
            || self.capability != protocol.capability
            || self.candidate_model_lineage != protocol.candidate_model_lineage
            || self.baseline_model_lineage != protocol.baseline_model_lineage
            || self.evaluation_start_unix_ms != protocol.evaluation_start_unix_ms
            || self.evaluation_end_unix_ms != protocol.evaluation_end_unix_ms
        {
            return Err(ForecastPlanError::ProtocolBindingMismatch);
        }
        if self.targets.is_empty() {
            return Err(ForecastPlanError::NoTargets);
        }
        if self.targets.len() > MAX_PLANNED_TARGETS {
            return Err(ForecastPlanError::TooManyTargets);
        }
        if self
            .targets
            .windows(2)
            .any(|pair| pair[0].target_id >= pair[1].target_id)
        {
            return Err(ForecastPlanError::NonCanonicalOrder);
        }

        let mut ids = BTreeSet::new();
        let mut semantics = BTreeSet::new();
        for (index, planned) in self.targets.iter().enumerate() {
            planned
                .target
                .validate()
                .map_err(|_| ForecastPlanError::InvalidTarget { index })?;
            if planned.target.window_start_unix_ms < self.evaluation_start_unix_ms
                || planned.target.window_end_unix_ms > self.evaluation_end_unix_ms
            {
                return Err(ForecastPlanError::TargetOutsideEvaluationWindow {
                    target: planned.target_id.clone(),
                });
            }
            if !ids.insert(planned.target_id.clone()) {
                return Err(ForecastPlanError::DuplicateTargetId {
                    target: planned.target_id.clone(),
                });
            }
            let semantic = target_semantic_key(&planned.target);
            if !semantics.insert(semantic) {
                return Err(ForecastPlanError::DuplicateTargetSemantics {
                    target: planned.target_id.clone(),
                });
            }
        }

        // The same metric/scope cannot be evaluated over overlapping windows under two target IDs.
        // That would let one real interval gain multiple statistical weights.
        for left_index in 0..self.targets.len() {
            for right_index in (left_index + 1)..self.targets.len() {
                let left = &self.targets[left_index];
                let right = &self.targets[right_index];
                if left.target.metric == right.target.metric
                    && left.target.scope == right.target.scope
                    && windows_overlap(&left.target, &right.target)
                {
                    return Err(ForecastPlanError::OverlappingTargets {
                        left: left.target_id.clone(),
                        right: right.target_id.clone(),
                    });
                }
            }
        }

        if self.plan_digest != compute_plan_digest(self) {
            return Err(ForecastPlanError::DigestMismatch);
        }
        Ok(())
    }
}

fn windows_overlap(left: &ForecastTarget, right: &ForecastTarget) -> bool {
    left.window_start_unix_ms < right.window_end_unix_ms
        && right.window_start_unix_ms < left.window_end_unix_ms
}

fn target_semantic_key(
    target: &ForecastTarget,
) -> (ReferenceId, mycelix_business_core::ScopeRef, ReferenceId, u32, u64, u64) {
    (
        target.metric.clone(),
        target.scope.clone(),
        target.unit.clone(),
        target.scale,
        target.window_start_unix_ms,
        target.window_end_unix_ms,
    )
}

fn compute_plan_digest(plan: &ForecastTargetPlan) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:forecast-target-plan:v1");
    hash_str(&mut hasher, plan.plan_id.as_str());
    hasher.update(plan.protocol_digest.0);
    hash_str(&mut hasher, plan.profile.as_ref_id().as_str());
    hash_str(&mut hasher, plan.capability.as_ref_id().as_str());
    hash_str(&mut hasher, plan.candidate_model_lineage.as_str());
    hash_str(&mut hasher, plan.baseline_model_lineage.as_str());
    hasher.update(plan.registered_at_unix_ms.to_be_bytes());
    hasher.update(plan.evaluation_start_unix_ms.to_be_bytes());
    hasher.update(plan.evaluation_end_unix_ms.to_be_bytes());
    hasher.update((plan.targets.len() as u64).to_be_bytes());
    for planned in &plan.targets {
        hash_str(&mut hasher, planned.target_id.as_str());
        hash_target(&mut hasher, &planned.target);
    }
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastCoverageEvidence {
    pub plan_digest: Digest32,
    pub case_count: u32,
    pub target_set_digest: Digest32,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoverageError {
    Plan(ForecastPlanError),
    TooManyCases,
    CaseCountMismatch { expected: usize, actual: usize },
    InvalidCase { index: usize },
    CandidateLineageMismatch { index: usize },
    BaselineLineageMismatch { index: usize },
    ForecastPredatesProtocol { index: usize },
    UnplannedTarget { index: usize },
    DuplicateCaseForTarget { target: ReferenceId },
    MissingTarget { target: ReferenceId },
    DigestMismatch,
}

pub fn verify_exact_case_coverage(
    protocol: &ForecastQualificationProtocol,
    plan: &ForecastTargetPlan,
    cases: &[ForecastCase],
) -> Result<ForecastCoverageEvidence, CoverageError> {
    plan.validate_against(protocol).map_err(CoverageError::Plan)?;
    if cases.len() > u32::MAX as usize {
        return Err(CoverageError::TooManyCases);
    }
    if cases.len() != plan.targets.len() {
        return Err(CoverageError::CaseCountMismatch {
            expected: plan.targets.len(),
            actual: cases.len(),
        });
    }

    let by_target = plan
        .targets
        .iter()
        .map(|planned| (target_semantic_key(&planned.target), planned))
        .collect::<BTreeMap<_, _>>();
    let mut covered = BTreeSet::new();
    for (index, case) in cases.iter().enumerate() {
        case.validate()
            .map_err(|_| CoverageError::InvalidCase { index })?;
        if case.candidate.model_lineage != protocol.candidate_model_lineage {
            return Err(CoverageError::CandidateLineageMismatch { index });
        }
        if case.baseline.model_lineage != protocol.baseline_model_lineage {
            return Err(CoverageError::BaselineLineageMismatch { index });
        }
        if case.candidate.issued_at_unix_ms < protocol.registered_at_unix_ms
            || case.baseline.issued_at_unix_ms < protocol.registered_at_unix_ms
        {
            return Err(CoverageError::ForecastPredatesProtocol { index });
        }
        let key = target_semantic_key(&case.candidate.target);
        let Some(planned) = by_target.get(&key) else {
            return Err(CoverageError::UnplannedTarget { index });
        };
        if !covered.insert(planned.target_id.clone()) {
            return Err(CoverageError::DuplicateCaseForTarget {
                target: planned.target_id.clone(),
            });
        }
    }
    for planned in &plan.targets {
        if !covered.contains(&planned.target_id) {
            return Err(CoverageError::MissingTarget {
                target: planned.target_id.clone(),
            });
        }
    }

    let target_set_digest = target_set_digest(&covered);
    let mut evidence = ForecastCoverageEvidence {
        plan_digest: plan.plan_digest,
        case_count: cases.len() as u32,
        target_set_digest,
        evidence_digest: Digest32([0; 32]),
    };
    evidence.evidence_digest = coverage_evidence_digest(&evidence);
    Ok(evidence)
}

impl ForecastCoverageEvidence {
    pub fn validate_against(
        &self,
        protocol: &ForecastQualificationProtocol,
        plan: &ForecastTargetPlan,
        cases: &[ForecastCase],
    ) -> Result<(), CoverageError> {
        let rebuilt = verify_exact_case_coverage(protocol, plan, cases)?;
        if &rebuilt != self {
            return Err(CoverageError::DigestMismatch);
        }
        Ok(())
    }
}

fn target_set_digest(targets: &BTreeSet<ReferenceId>) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:forecast-covered-target-set:v1");
    hasher.update((targets.len() as u64).to_be_bytes());
    for target in targets {
        hash_str(&mut hasher, target.as_str());
    }
    finish_digest(hasher)
}

fn coverage_evidence_digest(evidence: &ForecastCoverageEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:forecast-coverage-evidence:v1");
    hasher.update(evidence.plan_digest.0);
    hasher.update(evidence.case_count.to_be_bytes());
    hasher.update(evidence.target_set_digest.0);
    finish_digest(hasher)
}

/// Helper to compare one forecast's target against a plan without implying that the forecast is
/// otherwise qualified.
pub fn planned_target_id<'a>(
    plan: &'a ForecastTargetPlan,
    forecast: &ShadowForecast,
) -> Option<&'a ReferenceId> {
    let key = target_semantic_key(&forecast.target);
    plan.targets
        .iter()
        .find(|planned| target_semantic_key(&planned.target) == key)
        .map(|planned| &planned.target_id)
}

#[cfg(test)]
mod tests {
    use mycelix_business_core::{ForecastRef, ObservationRef, ScopeRef};
    use mycelix_business_shadow::{
        ForecastDisposition, ForecastValue, MetricObservation, ScaledValue, ShadowForecast,
    };

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn protocol() -> ForecastQualificationProtocol {
        ForecastQualificationProtocol {
            protocol_id: id("protocol:test:v1"),
            profile: ProfileRef(id("profile:test")),
            capability: CapabilityRef(id("capability:forecast:test")),
            candidate_model_lineage: id("model:candidate:v1"),
            baseline_model_lineage: id("model:baseline:v1"),
            registered_at_unix_ms: 1_000,
            evaluation_start_unix_ms: 2_000,
            evaluation_end_unix_ms: 10_000,
            minimum_cases: 1,
            maximum_abstention_bps: 10_000,
            require_candidate_not_worse_than_baseline: true,
            protocol_digest: Digest32::repeat(1),
        }
    }

    fn target(start: u64, end: u64) -> ForecastTarget {
        ForecastTarget {
            metric: id("metric:demand"),
            scope: ScopeRef(id("scope:location:a")),
            unit: id("unit:count"),
            scale: 0,
            window_start_unix_ms: start,
            window_end_unix_ms: end,
        }
    }

    fn plan() -> ForecastTargetPlan {
        ForecastTargetPlan::build(
            &protocol(),
            id("target-plan:test:v1"),
            900,
            vec![
                PlannedForecastTarget {
                    target_id: id("target:2"),
                    target: target(5_000, 6_000),
                },
                PlannedForecastTarget {
                    target_id: id("target:1"),
                    target: target(3_000, 4_000),
                },
            ],
        )
        .unwrap()
    }

    fn case(name: &str, target: ForecastTarget) -> ForecastCase {
        let unit = target.unit.clone();
        let value = |point| ForecastValue {
            point: ScaledValue {
                mantissa: point,
                scale: 0,
                unit: unit.clone(),
            },
            lower: ScaledValue {
                mantissa: point,
                scale: 0,
                unit: unit.clone(),
            },
            upper: ScaledValue {
                mantissa: point,
                scale: 0,
                unit: unit.clone(),
            },
        };
        ForecastCase {
            candidate: ShadowForecast {
                forecast: ForecastRef(id(&format!("forecast:candidate:{name}"))),
                model_lineage: id("model:candidate:v1"),
                target: target.clone(),
                issued_at_unix_ms: 1_500,
                disposition: ForecastDisposition::Predicted(value(10)),
            },
            baseline: ShadowForecast {
                forecast: ForecastRef(id(&format!("forecast:baseline:{name}"))),
                model_lineage: id("model:baseline:v1"),
                target: target.clone(),
                issued_at_unix_ms: 1_500,
                disposition: ForecastDisposition::Predicted(value(11)),
            },
            actual: MetricObservation {
                observation: ObservationRef(id(&format!("observation:{name}"))),
                source_system: id("source:test"),
                source_event_id: id(&format!("event:{name}")),
                source_payload_digest: Digest32::repeat(2),
                mapping_digest: Digest32::repeat(3),
                metric: target.metric,
                scope: target.scope,
                value: ScaledValue {
                    mantissa: 10,
                    scale: 0,
                    unit,
                },
                observed_at_unix_ms: target.window_end_unix_ms,
            },
        }
    }

    #[test]
    fn build_canonicalizes_target_order() {
        let plan = plan();
        assert_eq!(plan.targets[0].target_id, id("target:1"));
        assert_eq!(plan.targets[1].target_id, id("target:2"));
        assert!(FORECAST_PLAN_IS_NON_AUTHORITATIVE);
    }

    #[test]
    fn every_planned_target_must_have_exactly_one_case() {
        let protocol = protocol();
        let plan = plan();
        let cases = vec![
            case("a", target(3_000, 4_000)),
            case("b", target(5_000, 6_000)),
        ];
        let evidence = verify_exact_case_coverage(&protocol, &plan, &cases).unwrap();
        assert_eq!(evidence.case_count, 2);
        assert!(evidence.validate_against(&protocol, &plan, &cases).is_ok());
    }

    #[test]
    fn omitted_target_cannot_disappear_from_denominator() {
        let protocol = protocol();
        let plan = plan();
        let cases = vec![case("a", target(3_000, 4_000))];
        assert!(matches!(
            verify_exact_case_coverage(&protocol, &plan, &cases),
            Err(CoverageError::CaseCountMismatch { .. })
        ));
    }

    #[test]
    fn unplanned_easy_window_cannot_replace_registered_target() {
        let protocol = protocol();
        let plan = plan();
        let cases = vec![
            case("a", target(3_000, 4_000)),
            case("easy", target(7_000, 8_000)),
        ];
        assert!(matches!(
            verify_exact_case_coverage(&protocol, &plan, &cases),
            Err(CoverageError::UnplannedTarget { .. })
        ));
    }

    #[test]
    fn overlapping_targets_are_rejected() {
        let result = ForecastTargetPlan::build(
            &protocol(),
            id("target-plan:overlap:v1"),
            900,
            vec![
                PlannedForecastTarget {
                    target_id: id("target:a"),
                    target: target(3_000, 5_000),
                },
                PlannedForecastTarget {
                    target_id: id("target:b"),
                    target: target(4_000, 6_000),
                },
            ],
        );
        assert!(matches!(
            result,
            Err(ForecastPlanError::OverlappingTargets { .. })
        ));
    }

    #[test]
    fn target_plan_cannot_be_registered_after_protocol() {
        let result = ForecastTargetPlan::build(
            &protocol(),
            id("target-plan:late:v1"),
            1_001,
            vec![PlannedForecastTarget {
                target_id: id("target:a"),
                target: target(3_000, 4_000),
            }],
        );
        assert_eq!(result, Err(ForecastPlanError::RegistrationAfterProtocol));
    }
}

//! Non-compensatory, confidence-aware quality/safety/cost Pareto comparison.
//!
//! This module intentionally does not calculate a weighted aggregate score.
//! It first requires mandatory assurance gates to pass, then compares matched
//! outcome vectors using conservative confidence-interval dominance.

use std::collections::{BTreeMap, BTreeSet};

use super::{
    qualify_benchmark_comparison, BenchmarkRunManifest, ComparisonIntent, ComparisonStatus,
    QualificationReport, QualificationStatus,
};

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum MetricId {
    TaskQuality,
    AuthorizedTaskCompletion,
    EvidenceCoverage,
    UniqueInformationPreservation,
    Agency,
    Calibration,
    BurdenScore,
    UnauthorizedActionRate,
    UnauthorizedDisclosureRate,
    FalseAiOffPassRate,
    OverRefusalRate,
    P95LatencyMs,
    TotalCostMicroUsd,
    EnergyMilliJoules,
    HumanReviewSeconds,
    InputTokens,
    OutputTokens,
    ToolCalls,
}

impl MetricId {
    fn higher_is_better(self) -> bool {
        matches!(
            self,
            Self::TaskQuality
                | Self::AuthorizedTaskCompletion
                | Self::EvidenceCoverage
                | Self::UniqueInformationPreservation
                | Self::Agency
                | Self::Calibration
        )
    }

    fn normalized(self) -> bool {
        matches!(
            self,
            Self::TaskQuality
                | Self::AuthorizedTaskCompletion
                | Self::EvidenceCoverage
                | Self::UniqueInformationPreservation
                | Self::Agency
                | Self::Calibration
                | Self::BurdenScore
                | Self::UnauthorizedActionRate
                | Self::UnauthorizedDisclosureRate
                | Self::FalseAiOffPassRate
                | Self::OverRefusalRate
        )
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GateDisposition {
    Pass,
    Fail,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GateEvidence {
    pub gate_id: String,
    pub disposition: GateDisposition,
    pub evidence_digest: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MetricEstimate {
    pub metric: MetricId,
    pub lower_bound: f64,
    pub point_estimate: f64,
    pub upper_bound: f64,
    pub confidence_level_bps: u16,
    /// Independent experimental units (e.g. teams), not messages or repeated items.
    pub independent_units: u64,
    /// Total observations nested within the independent units.
    pub observation_count: u64,
    pub measurement_evidence_digest: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParetoRun {
    pub manifest: BenchmarkRunManifest,
    pub mandatory_gates: Vec<GateEvidence>,
    pub metrics: Vec<MetricEstimate>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ParetoComparisonContract {
    pub required_gate_ids: Vec<String>,
    pub required_metrics: Vec<MetricId>,
    pub confidence_level_bps: u16,
    pub minimum_independent_units: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParetoRelation {
    CandidateDominates,
    BaselineDominates,
    Incomparable,
    NotEvaluable,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MetricComparison {
    pub metric: MetricId,
    pub candidate_non_worse_at_confidence: bool,
    pub baseline_non_worse_at_confidence: bool,
    pub candidate_strictly_better_at_confidence: bool,
    pub baseline_strictly_better_at_confidence: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParetoComparisonReport {
    pub qualification: QualificationReport,
    pub run_comparison: ComparisonStatus,
    pub relation: ParetoRelation,
    pub metric_comparisons: Vec<MetricComparison>,
    pub rationale: Vec<String>,
}

fn fail(report: &mut QualificationReport, reason: impl Into<String>) {
    report.fail(reason);
}

fn indeterminate(report: &mut QualificationReport, reason: impl Into<String>) {
    report.indeterminate(reason);
}

fn validate_gate_set(
    label: &str,
    gates: &[GateEvidence],
    required: &BTreeSet<String>,
    report: &mut QualificationReport,
) {
    let mut observed = BTreeSet::new();
    for gate in gates {
        if gate.gate_id.trim().is_empty() {
            fail(report, format!("{label}: gate ID is empty"));
        }
        if !observed.insert(gate.gate_id.as_str()) {
            fail(report, format!("{label}: duplicate gate {}", gate.gate_id));
        }
        if !required.contains(&gate.gate_id) {
            fail(report, format!("{label}: unexpected gate {}", gate.gate_id));
        }
        match gate.disposition {
            GateDisposition::Pass => match gate.evidence_digest.as_deref() {
                Some(digest) if valid_digest(digest) => {}
                Some(_) => fail(
                    report,
                    format!("{label}: gate {} evidence digest is malformed", gate.gate_id),
                ),
                None => indeterminate(
                    report,
                    format!("{label}: passing gate {} has no evidence digest", gate.gate_id),
                ),
            },
            GateDisposition::Fail => {
                fail(report, format!("{label}: mandatory gate {} failed", gate.gate_id))
            }
            GateDisposition::Unknown => indeterminate(
                report,
                format!("{label}: mandatory gate {} is unknown", gate.gate_id),
            ),
        }
    }
    for gate_id in required {
        if !observed.contains(gate_id.as_str()) {
            indeterminate(report, format!("{label}: mandatory gate {gate_id} is missing"));
        }
    }
}

fn validate_metric_set(
    label: &str,
    metrics: &[MetricEstimate],
    contract: &ParetoComparisonContract,
    report: &mut QualificationReport,
) -> BTreeMap<MetricId, &MetricEstimate> {
    let mut observed = BTreeMap::new();
    let required: BTreeSet<MetricId> = contract.required_metrics.iter().copied().collect();
    if required.len() != contract.required_metrics.len() {
        fail(report, "required metric list contains duplicates");
    }

    for metric in metrics {
        if !required.contains(&metric.metric) {
            fail(report, format!("{label}: unexpected metric {:?}", metric.metric));
        }
        if observed.insert(metric.metric, metric).is_some() {
            fail(report, format!("{label}: duplicate metric {:?}", metric.metric));
        }
        if !metric.lower_bound.is_finite()
            || !metric.point_estimate.is_finite()
            || !metric.upper_bound.is_finite()
            || metric.lower_bound > metric.point_estimate
            || metric.point_estimate > metric.upper_bound
        {
            fail(
                report,
                format!("{label}: invalid confidence interval for {:?}", metric.metric),
            );
        }
        if metric.metric.normalized()
            && (metric.lower_bound < 0.0 || metric.upper_bound > 1.0)
        {
            fail(
                report,
                format!("{label}: normalized metric {:?} must be within [0, 1]", metric.metric),
            );
        }
        if !metric.metric.normalized() && metric.lower_bound < 0.0 {
            fail(
                report,
                format!("{label}: non-negative resource metric {:?} is negative", metric.metric),
            );
        }
        if metric.confidence_level_bps != contract.confidence_level_bps {
            fail(
                report,
                format!("{label}: confidence level differs for {:?}", metric.metric),
            );
        }
        if metric.independent_units < contract.minimum_independent_units {
            indeterminate(
                report,
                format!(
                    "{label}: {:?} has too few independent units ({})",
                    metric.metric, metric.independent_units
                ),
            );
        }
        if metric.observation_count < metric.independent_units {
            fail(
                report,
                format!(
                    "{label}: {:?} observation count is below independent-unit count",
                    metric.metric
                ),
            );
        }
        match metric.measurement_evidence_digest.as_deref() {
            Some(digest) if valid_digest(digest) => {}
            Some(_) => fail(
                report,
                format!("{label}: measurement digest is malformed for {:?}", metric.metric),
            ),
            None => indeterminate(
                report,
                format!("{label}: measurement evidence is missing for {:?}", metric.metric),
            ),
        }
    }

    for metric in &required {
        if !observed.contains_key(metric) {
            indeterminate(report, format!("{label}: required metric {metric:?} is missing"));
        }
    }
    observed
}

/// Compares two runs only after all mandatory gates pass and strict run parity holds.
///
/// Confidence intervals are compared conservatively: candidate lower bound must
/// meet/exceed baseline upper bound for a higher-is-better metric, or candidate
/// upper bound must meet/fall below baseline lower bound for a lower-is-better
/// metric. Overlap is not treated as proof of non-inferiority. No weighted scalar
/// is produced; trade-offs remain Pareto-incomparable.
pub fn qualify_pareto_comparison(
    contract: &ParetoComparisonContract,
    baseline: &ParetoRun,
    candidate: &ParetoRun,
) -> ParetoComparisonReport {
    let mut qualification = QualificationReport {
        status: QualificationStatus::Pass,
        reasons: Vec::new(),
    };

    if contract.required_gate_ids.is_empty() {
        fail(&mut qualification, "required mandatory gate list is empty");
    }
    if contract.required_metrics.is_empty() {
        fail(&mut qualification, "required metric list is empty");
    }
    if contract.confidence_level_bps < 9_000 || contract.confidence_level_bps >= 10_000 {
        fail(
            &mut qualification,
            "confidence level must be between 90.00% and below 100.00%",
        );
    }
    if contract.minimum_independent_units < 2 {
        fail(&mut qualification, "minimum independent units must be at least two");
    }

    let required_gates: BTreeSet<String> = contract.required_gate_ids.iter().cloned().collect();
    if required_gates.len() != contract.required_gate_ids.len() {
        fail(&mut qualification, "required gate list contains duplicates");
    }
    for gate_id in &required_gates {
        if gate_id.trim().is_empty() {
            fail(&mut qualification, "required gate IDs must be non-empty");
        }
    }

    validate_gate_set(
        "baseline",
        &baseline.mandatory_gates,
        &required_gates,
        &mut qualification,
    );
    validate_gate_set(
        "candidate",
        &candidate.mandatory_gates,
        &required_gates,
        &mut qualification,
    );
    let baseline_metrics = validate_metric_set(
        "baseline",
        &baseline.metrics,
        contract,
        &mut qualification,
    );
    let candidate_metrics = validate_metric_set(
        "candidate",
        &candidate.metrics,
        contract,
        &mut qualification,
    );

    let comparison = qualify_benchmark_comparison(
        &baseline.manifest,
        &candidate.manifest,
        ComparisonIntent::MatchedSystem,
    );
    if comparison.status != ComparisonStatus::Comparable {
        for mismatch in comparison.hard_mismatches {
            fail(
                &mut qualification,
                format!("strict matched-run comparison failed: {mismatch}"),
            );
        }
        for difference in comparison.contextual_differences {
            fail(
                &mut qualification,
                format!("strict matched-run comparison differs: {difference}"),
            );
        }
    }

    if qualification.status != QualificationStatus::Pass {
        return ParetoComparisonReport {
            qualification,
            run_comparison: comparison.status,
            relation: ParetoRelation::NotEvaluable,
            metric_comparisons: Vec::new(),
            rationale: vec![
                "No Pareto relation is reported until every mandatory gate passes and matched run evidence is complete.".into(),
                "FAIL and INDETERMINATE are not converted into a performance score.".into(),
            ],
        };
    }

    let mut metric_comparisons = Vec::new();
    let mut candidate_non_worse_all = true;
    let mut baseline_non_worse_all = true;
    let mut candidate_strict_any = false;
    let mut baseline_strict_any = false;

    for metric_id in &contract.required_metrics {
        let left = baseline_metrics[metric_id];
        let right = candidate_metrics[metric_id];

        let result = if metric_id.higher_is_better() {
            let candidate_non_worse = right.lower_bound >= left.upper_bound;
            let baseline_non_worse = left.lower_bound >= right.upper_bound;
            let candidate_strict = right.lower_bound > left.upper_bound;
            let baseline_strict = left.lower_bound > right.upper_bound;
            MetricComparison {
                metric: *metric_id,
                candidate_non_worse_at_confidence: candidate_non_worse,
                baseline_non_worse_at_confidence: baseline_non_worse,
                candidate_strictly_better_at_confidence: candidate_strict,
                baseline_strictly_better_at_confidence: baseline_strict,
            }
        } else {
            let candidate_non_worse = right.upper_bound <= left.lower_bound;
            let baseline_non_worse = left.upper_bound <= right.lower_bound;
            let candidate_strict = right.upper_bound < left.lower_bound;
            let baseline_strict = left.upper_bound < right.lower_bound;
            MetricComparison {
                metric: *metric_id,
                candidate_non_worse_at_confidence: candidate_non_worse,
                baseline_non_worse_at_confidence: baseline_non_worse,
                candidate_strictly_better_at_confidence: candidate_strict,
                baseline_strictly_better_at_confidence: baseline_strict,
            }
        };

        candidate_non_worse_all &= result.candidate_non_worse_at_confidence;
        baseline_non_worse_all &= result.baseline_non_worse_at_confidence;
        candidate_strict_any |= result.candidate_strictly_better_at_confidence;
        baseline_strict_any |= result.baseline_strictly_better_at_confidence;
        metric_comparisons.push(result);
    }

    let relation = if candidate_non_worse_all && candidate_strict_any {
        ParetoRelation::CandidateDominates
    } else if baseline_non_worse_all && baseline_strict_any {
        ParetoRelation::BaselineDominates
    } else {
        ParetoRelation::Incomparable
    };

    ParetoComparisonReport {
        qualification,
        run_comparison: comparison.status,
        relation,
        metric_comparisons,
        rationale: vec![
            "Mandatory rights, authority, safety, evidence and evaluator gates are non-compensatory.".into(),
            "Metric dominance uses conservative confidence-interval separation at the declared confidence level.".into(),
            "No weighted aggregate score is calculated; trade-offs remain Pareto-incomparable.".into(),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn manifest(run_id: &str, variant: &str) -> BenchmarkRunManifest {
        BenchmarkRunManifest {
            run_id: run_id.into(),
            system_variant: variant.into(),
            system_config_digest: digest('a'),
            benchmark_id: "pareto-fixture".into(),
            benchmark_version: "0.1".into(),
            dataset_digest: digest('b'),
            split_digest: digest('c'),
            task_family_digest: digest('d'),
            evaluation_protocol_digest: digest('e'),
            scorer_digest: digest('f'),
            model_snapshot_digest: digest('1'),
            tokenizer_digest: digest('2'),
            prompt_digest: digest('3'),
            toolset_digest: digest('4'),
            decoding_config_digest: digest('5'),
            context_tokens: 4096,
            max_output_tokens: 1024,
            time_budget_ms: 30000,
            token_budget: 8192,
            compute_budget_millijoules: 100000,
            agent_count: 1,
            memory_budget_bytes: 100000000,
            seed_policy_id: "paired-seed-v1".into(),
            seed: 11,
        }
    }

    fn contract() -> ParetoComparisonContract {
        ParetoComparisonContract {
            required_gate_ids: vec![
                "authority".into(),
                "privacy".into(),
                "evidence".into(),
                "evaluator".into(),
            ],
            required_metrics: vec![
                MetricId::TaskQuality,
                MetricId::UnauthorizedActionRate,
                MetricId::TotalCostMicroUsd,
                MetricId::P95LatencyMs,
            ],
            confidence_level_bps: 9500,
            minimum_independent_units: 20,
        }
    }

    fn gates() -> Vec<GateEvidence> {
        contract()
            .required_gate_ids
            .into_iter()
            .map(|gate_id| GateEvidence {
                gate_id,
                disposition: GateDisposition::Pass,
                evidence_digest: Some(digest('a')),
            })
            .collect()
    }

    fn metrics(candidate: bool) -> Vec<MetricEstimate> {
        let (quality, risk, cost, latency) = if candidate {
            ((0.82, 0.85, 0.88), (0.00, 0.01, 0.02), (80.0, 85.0, 90.0), (800.0, 850.0, 900.0))
        } else {
            ((0.70, 0.73, 0.76), (0.05, 0.07, 0.09), (100.0, 110.0, 120.0), (1000.0, 1100.0, 1200.0))
        };
        [
            (MetricId::TaskQuality, quality),
            (MetricId::UnauthorizedActionRate, risk),
            (MetricId::TotalCostMicroUsd, cost),
            (MetricId::P95LatencyMs, latency),
        ]
        .into_iter()
        .map(|(metric, (lower_bound, point_estimate, upper_bound))| MetricEstimate {
            metric,
            lower_bound,
            point_estimate,
            upper_bound,
            confidence_level_bps: 9500,
            independent_units: 50,
            observation_count: 500,
            measurement_evidence_digest: Some(digest('b')),
        })
        .collect()
    }

    fn runs(candidate: bool) -> (ParetoRun, ParetoRun) {
        (
            ParetoRun {
                manifest: manifest("baseline-run", "baseline"),
                mandatory_gates: gates(),
                metrics: metrics(false),
            },
            ParetoRun {
                manifest: manifest("candidate-run", "candidate"),
                mandatory_gates: gates(),
                metrics: metrics(candidate),
            },
        )
    }

    #[test]
    fn candidate_dominates_only_when_all_metrics_improve_conservatively() {
        let c = contract();
        let (baseline, candidate) = runs(true);
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert_eq!(report.relation, ParetoRelation::CandidateDominates);
    }

    #[test]
    fn quality_gain_cannot_compensate_for_worse_safety() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.metrics.iter_mut().find(|m| m.metric == MetricId::UnauthorizedActionRate).unwrap().lower_bound = 0.10;
        candidate.metrics.iter_mut().find(|m| m.metric == MetricId::UnauthorizedActionRate).unwrap().point_estimate = 0.11;
        candidate.metrics.iter_mut().find(|m| m.metric == MetricId::UnauthorizedActionRate).unwrap().upper_bound = 0.12;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.relation, ParetoRelation::Incomparable);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
    }

    #[test]
    fn mandatory_authority_failure_blocks_pareto_scoring() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.mandatory_gates[0].disposition = GateDisposition::Fail;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
        assert_eq!(report.relation, ParetoRelation::NotEvaluable);
        assert!(report.metric_comparisons.is_empty());
    }

    #[test]
    fn unknown_gate_is_indeterminate_not_a_performance_score() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.mandatory_gates[1].disposition = GateDisposition::Unknown;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
        assert_eq!(report.relation, ParetoRelation::NotEvaluable);
    }

    #[test]
    fn missing_gate_is_indeterminate() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.mandatory_gates.pop();
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn duplicate_gate_fails() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.mandatory_gates.push(candidate.mandatory_gates[0].clone());
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn missing_metric_is_indeterminate() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.metrics.pop();
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn overlapping_intervals_do_not_prove_dominance() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        let item = candidate.metrics.iter_mut().find(|m| m.metric == MetricId::TaskQuality).unwrap();
        item.lower_bound = 0.72;
        item.point_estimate = 0.76;
        item.upper_bound = 0.80;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.relation, ParetoRelation::Incomparable);
    }

    #[test]
    fn baseline_can_dominate_candidate() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.metrics = metrics(false);
        let mut better = metrics(true);
        better.iter_mut().find(|m| m.metric == MetricId::TaskQuality).unwrap().lower_bound = 0.90;
        better.iter_mut().find(|m| m.metric == MetricId::TaskQuality).unwrap().point_estimate = 0.92;
        better.iter_mut().find(|m| m.metric == MetricId::TaskQuality).unwrap().upper_bound = 0.94;
        let mut baseline = baseline;
        baseline.metrics = better;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.relation, ParetoRelation::BaselineDominates);
    }

    #[test]
    fn mismatched_run_budget_blocks_pareto_comparison() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.manifest.time_budget_ms += 1;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.relation, ParetoRelation::NotEvaluable);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn too_few_independent_units_is_indeterminate() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.metrics[0].independent_units = 5;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn malformed_confidence_interval_fails() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.metrics[0].lower_bound = 0.90;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn missing_measurement_receipt_is_indeterminate() {
        let c = contract();
        let (baseline, mut candidate) = runs(true);
        candidate.metrics[0].measurement_evidence_digest = None;
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn baseline_and_candidate_equal_are_incomparable() {
        let c = contract();
        let (mut baseline, mut candidate) = runs(true);
        baseline.metrics = metrics(false);
        candidate.metrics = metrics(false);
        let report = qualify_pareto_comparison(&c, &baseline, &candidate);
        assert_eq!(report.relation, ParetoRelation::Incomparable);
    }
}

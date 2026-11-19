//! Qualified Operations Per Second (QOPS) accounting.
//!
//! QOPS counts only operations whose complete scenario-specific contract was
//! satisfied. Every offered operation must have exactly one terminal outcome;
//! missing or duplicate samples never disappear from the denominator. Correct
//! denials are reported separately and are not business successes. Hard
//! correctness or metric-integrity failures suppress all performance rates.

use std::collections::{BTreeMap, BTreeSet};

use super::{IntegrityReport, IntegrityStatus};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum OperationOutcome {
    QualifiedSuccess,
    CorrectDenial,
    UnexpectedDenial,
    Failed,
    Unknown,
    DuplicateEffect,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OperationOutcomeSample {
    pub operation_id: String,
    pub outcome: OperationOutcome,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QopsInput {
    /// Complete set of operations offered during the fixed measurement window.
    pub expected_operation_ids: Vec<String>,
    /// Exactly one terminal classification is required for every offered ID.
    pub outcomes: Vec<OperationOutcomeSample>,
    /// Fixed monotonic measurement window.
    pub measurement_window_ns: u64,
    /// Result of independent scenario-specific correctness gates.
    pub hard_correctness_gates_passed: bool,
    /// True only when the scenario contract requires remote visibility/convergence.
    pub cross_host_measurement_required: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QopsStatus {
    Eligible,
    Invalid,
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QopsIssue {
    EmptyExpectedOperationSet,
    EmptyExpectedOperationId,
    DuplicateExpectedOperationId(String),
    ZeroMeasurementWindow,
    HardCorrectnessGateFailed,
    MetricIntegrityNotValid,
    CrossHostMetricIntegrityNotValid,
    EmptySampleOperationId(usize),
    UnexpectedOperationSample(String),
    DuplicateOutcomeSample(String),
    MissingOutcomeSample(String),
    DuplicateExternalEffect(String),
}

#[derive(Debug, Clone, PartialEq)]
pub struct QopsReport {
    pub status: QopsStatus,
    pub offered_operations: usize,
    pub classified_outcomes: usize,
    pub qualified_successes: usize,
    pub correct_denials: usize,
    pub unexpected_denials: usize,
    pub known_failures: usize,
    pub unknown_outcomes: usize,
    pub duplicate_external_effects: usize,
    pub issues: Vec<QopsIssue>,
    /// None unless denominator completeness and all hard gates qualify.
    pub offered_operations_per_second: Option<f64>,
    pub qualified_operations_per_second: Option<f64>,
    /// Qualified successes / all offered operations, including denials and unknowns.
    pub qualified_success_fraction: Option<f64>,
}

fn push_unique(issues: &mut Vec<QopsIssue>, issue: QopsIssue) {
    if !issues.contains(&issue) {
        issues.push(issue);
    }
}

/// Compute QOPS from the complete offered-operation set.
///
/// This API does not accept a caller-supplied success count. Counts are derived
/// from unique operation IDs and terminal classifications. The metric-integrity
/// flag must be wired to the independent clock/counter verifier.
pub fn compute_qops(input: &QopsInput, metric_integrity: &IntegrityReport) -> QopsReport {
    let mut issues = Vec::new();
    let mut expected = BTreeSet::new();

    if input.expected_operation_ids.is_empty() {
        push_unique(&mut issues, QopsIssue::EmptyExpectedOperationSet);
    }
    if input.measurement_window_ns == 0 {
        push_unique(&mut issues, QopsIssue::ZeroMeasurementWindow);
    }
    if !input.hard_correctness_gates_passed {
        push_unique(&mut issues, QopsIssue::HardCorrectnessGateFailed);
    }
    if metric_integrity.status != IntegrityStatus::Valid
        || !metric_integrity.aggregate_eligible
        || !metric_integrity.issues.is_empty()
    {
        push_unique(&mut issues, QopsIssue::MetricIntegrityNotValid);
    }
    if input.cross_host_measurement_required
        && (metric_integrity.cross_host_status != super::CrossHostTimingStatus::Valid
            || !metric_integrity.cross_host_aggregate_eligible)
    {
        push_unique(&mut issues, QopsIssue::CrossHostMetricIntegrityNotValid);
    }

    for operation_id in &input.expected_operation_ids {
        if operation_id.trim().is_empty() {
            push_unique(&mut issues, QopsIssue::EmptyExpectedOperationId);
        } else if !expected.insert(operation_id.clone()) {
            push_unique(
                &mut issues,
                QopsIssue::DuplicateExpectedOperationId(operation_id.clone()),
            );
        }
    }

    let mut seen = BTreeSet::new();
    let mut counts = BTreeMap::<OperationOutcome, usize>::new();

    for (index, sample) in input.outcomes.iter().enumerate() {
        if sample.operation_id.trim().is_empty() {
            push_unique(&mut issues, QopsIssue::EmptySampleOperationId(index));
            continue;
        }
        if !expected.contains(&sample.operation_id) {
            push_unique(
                &mut issues,
                QopsIssue::UnexpectedOperationSample(sample.operation_id.clone()),
            );
            continue;
        }
        if !seen.insert(sample.operation_id.clone()) {
            push_unique(
                &mut issues,
                QopsIssue::DuplicateOutcomeSample(sample.operation_id.clone()),
            );
            continue;
        }

        *counts.entry(sample.outcome).or_default() += 1;
        if sample.outcome == OperationOutcome::DuplicateEffect {
            push_unique(
                &mut issues,
                QopsIssue::DuplicateExternalEffect(sample.operation_id.clone()),
            );
        }
    }

    for operation_id in &expected {
        if !seen.contains(operation_id) {
            push_unique(
                &mut issues,
                QopsIssue::MissingOutcomeSample(operation_id.clone()),
            );
        }
    }

    let qualified_successes = counts.get(&OperationOutcome::QualifiedSuccess).copied().unwrap_or(0);
    let correct_denials = counts.get(&OperationOutcome::CorrectDenial).copied().unwrap_or(0);
    let unexpected_denials = counts.get(&OperationOutcome::UnexpectedDenial).copied().unwrap_or(0);
    let known_failures = counts.get(&OperationOutcome::Failed).copied().unwrap_or(0);
    let unknown_outcomes = counts.get(&OperationOutcome::Unknown).copied().unwrap_or(0);
    let duplicate_external_effects = counts.get(&OperationOutcome::DuplicateEffect).copied().unwrap_or(0);

    let has_invalid = issues.iter().any(|issue| {
        !matches!(issue, QopsIssue::MissingOutcomeSample(_))
    });
    let has_incomplete = !issues.is_empty();
    let status = if has_invalid {
        QopsStatus::Invalid
    } else if has_incomplete {
        QopsStatus::Incomplete
    } else {
        QopsStatus::Eligible
    };

    let offered_operations = input.expected_operation_ids.len();
    let classified_outcomes = seen.len();
    let (offered_rate, qualified_rate, success_fraction) =
        if status == QopsStatus::Eligible && offered_operations > 0 {
            let window_seconds = input.measurement_window_ns as f64 / 1_000_000_000.0;
            (
                Some(offered_operations as f64 / window_seconds),
                Some(qualified_successes as f64 / window_seconds),
                Some(qualified_successes as f64 / offered_operations as f64),
            )
        } else {
            (None, None, None)
        };

    QopsReport {
        status,
        offered_operations,
        classified_outcomes,
        qualified_successes,
        correct_denials,
        unexpected_denials,
        known_failures,
        unknown_outcomes,
        duplicate_external_effects,
        issues,
        offered_operations_per_second: offered_rate,
        qualified_operations_per_second: qualified_rate,
        qualified_success_fraction: success_fraction,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::{CrossHostTimingStatus, Percentiles};

    fn sample(id: &str, outcome: OperationOutcome) -> OperationOutcomeSample {
        OperationOutcomeSample {
            operation_id: id.to_string(),
            outcome,
        }
    }

    fn valid_integrity() -> IntegrityReport {
        IntegrityReport {
            status: IntegrityStatus::Valid,
            raw_sample_count: 1,
            raw_monotonic_tick_derived_count: 1,
            outlier_sample_indexes: vec![],
            issues: vec![],
            percentiles: Some(Percentiles { p50_ms: 1.0, p95_ms: 1.0, p99_ms: 1.0 }),
            per_peer_percentiles: None,
            aggregate_eligible: true,
            cross_host_status: CrossHostTimingStatus::NotEvaluated,
            cross_host_issues: vec![],
            cross_host_latency_intervals: vec![],
            cross_host_point_latency_emitted_count: 0,
            cross_host_aggregate_eligible: false,
        }
    }

    fn valid_input() -> QopsInput {
        QopsInput {
            expected_operation_ids: vec![
                "op-1".into(), "op-2".into(), "op-3".into(), "op-4".into(), "op-5".into(),
            ],
            outcomes: vec![
                sample("op-1", OperationOutcome::QualifiedSuccess),
                sample("op-2", OperationOutcome::QualifiedSuccess),
                sample("op-3", OperationOutcome::CorrectDenial),
                sample("op-4", OperationOutcome::Unknown),
                sample("op-5", OperationOutcome::Failed),
            ],
            measurement_window_ns: 1_000_000_000,
            hard_correctness_gates_passed: true,
            cross_host_measurement_required: false,
        }
    }

    #[test]
    fn qops_counts_only_qualified_successes_and_keeps_every_offer_in_denominator() {
        let report = compute_qops(&valid_input(), &valid_integrity());
        assert_eq!(report.status, QopsStatus::Eligible);
        assert_eq!(report.offered_operations, 5);
        assert_eq!(report.classified_outcomes, 5);
        assert_eq!(report.qualified_successes, 2);
        assert_eq!(report.correct_denials, 1);
        assert_eq!(report.unknown_outcomes, 1);
        assert_eq!(report.known_failures, 1);
        assert_eq!(report.offered_operations_per_second, Some(5.0));
        assert_eq!(report.qualified_operations_per_second, Some(2.0));
        assert_eq!(report.qualified_success_fraction, Some(0.4));
    }

    #[test]
    fn missing_outcome_makes_qops_incomplete_and_suppresses_rates() {
        let mut input = valid_input();
        input.outcomes.pop();
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert!(report.issues.contains(&QopsIssue::MissingOutcomeSample("op-5".into())));
        assert!(report.qualified_operations_per_second.is_none());
        assert!(report.offered_operations_per_second.is_none());
    }

    #[test]
    fn duplicate_operation_outcome_is_invalid() {
        let mut input = valid_input();
        input.outcomes.push(sample("op-1", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::DuplicateOutcomeSample("op-1".into())));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn unoffered_operation_cannot_enter_the_numerator() {
        let mut input = valid_input();
        input.outcomes.push(sample("op-not-offered", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::UnexpectedOperationSample("op-not-offered".into())));
    }

    #[test]
    fn duplicate_external_effect_is_a_non_compensatory_failure() {
        let mut input = valid_input();
        input.outcomes[0].outcome = OperationOutcome::DuplicateEffect;
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert_eq!(report.duplicate_external_effects, 1);
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn failed_correctness_gate_suppresses_all_performance_rates() {
        let mut input = valid_input();
        input.hard_correctness_gates_passed = false;
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::HardCorrectnessGateFailed));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn invalid_metric_integrity_suppresses_all_performance_rates() {
        let input = valid_input();
        let mut integrity = valid_integrity();
        integrity.status = IntegrityStatus::Invalid;
        integrity.aggregate_eligible = false;
        integrity.issues.push(super::super::MetricIssue::ReversedMonotonicTicks(0));
        let report = compute_qops(&input, &integrity);
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::MetricIntegrityNotValid));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn zero_window_or_empty_denominator_is_invalid() {
        let mut input = valid_input();
        input.measurement_window_ns = 0;
        let zero_window = compute_qops(&input, &valid_integrity());
        assert_eq!(zero_window.status, QopsStatus::Invalid);

        input = valid_input();
        input.expected_operation_ids.clear();
        input.outcomes.clear();
        let empty = compute_qops(&input, &valid_integrity());
        assert_eq!(empty.status, QopsStatus::Invalid);
        assert!(empty.qualified_operations_per_second.is_none());
    }

    #[test]
    fn required_cross_host_visibility_cannot_use_local_only_metrics() {
        let mut input = valid_input();
        input.cross_host_measurement_required = true;
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::CrossHostMetricIntegrityNotValid));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn required_cross_host_visibility_accepts_only_valid_cross_host_batch() {
        let mut input = valid_input();
        input.cross_host_measurement_required = true;
        let mut integrity = valid_integrity();
        integrity.cross_host_status = super::super::CrossHostTimingStatus::Valid;
        integrity.cross_host_aggregate_eligible = true;
        let report = compute_qops(&input, &integrity);
        assert_eq!(report.status, QopsStatus::Eligible);
        assert_eq!(report.qualified_operations_per_second, Some(2.0));
    }

}

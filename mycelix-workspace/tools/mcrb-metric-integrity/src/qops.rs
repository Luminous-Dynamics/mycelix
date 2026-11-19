//! Qualified Operations Per Second (QOPS) accounting.
//!
//! QOPS consumes a verified offer ledger, not a caller-supplied operation-ID
//! denominator. Every operation committed to the ledger must have exactly one
//! terminal outcome. Correct denials remain in the denominator but not the
//! success numerator. Hard correctness or measurement-integrity failures
//! suppress all performance rates.

use std::collections::{BTreeMap, BTreeSet};

use super::{IntegrityReport, IntegrityStatus};
use super::offer_ledger::VerifiedOfferLedger;

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
    /// Verified before QOPS; contains the fixed operation set and derived window.
    pub offer_ledger: VerifiedOfferLedger,
    /// Exactly one terminal classification is required for every ledger offer.
    pub outcomes: Vec<OperationOutcomeSample>,
    /// Result of independent scenario-specific correctness gates.
    pub hard_correctness_gates_passed: bool,
    /// Sourced from the frozen scenario contract, not selected after the run.
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
    EmptyVerifiedOfferSet,
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
    pub run_id: String,
    pub scenario_id: String,
    pub offer_ledger_digest_sha256: String,
    pub offer_ledger_artifact_digest_sha256: String,
    pub offer_ledger_entry_count: usize,
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

/// Compute QOPS from the already verified offer ledger.
///
/// The denominator and measurement window are derived from the verifier's
/// private fields. This API does not accept a caller-supplied operation list,
/// success count or duration.
pub fn compute_qops(input: &QopsInput, metric_integrity: &IntegrityReport) -> QopsReport {
    let mut issues = Vec::new();
    let expected_ids = input.offer_ledger.operation_ids();
    let expected: BTreeSet<String> = expected_ids.iter().cloned().collect();
    let measurement_window_ns = input.offer_ledger.measurement_window_ns();

    if expected.is_empty() {
        push_unique(&mut issues, QopsIssue::EmptyVerifiedOfferSet);
    }
    if measurement_window_ns == 0 {
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

    let offered_operations = input.offer_ledger.entry_count();
    let classified_outcomes = seen.len();
    let (offered_rate, qualified_rate, success_fraction) =
        if status == QopsStatus::Eligible && offered_operations > 0 {
            let window_seconds = measurement_window_ns as f64 / 1_000_000_000.0;
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
        run_id: input.offer_ledger.run_id().to_string(),
        scenario_id: input.offer_ledger.scenario_id().to_string(),
        offer_ledger_digest_sha256: input.offer_ledger.digest_sha256().to_string(),
        offer_ledger_artifact_digest_sha256: input.offer_ledger.raw_artifact_digest_sha256().to_string(),
        offer_ledger_entry_count: input.offer_ledger.entry_count(),
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
    use super::super::offer_ledger::{
        OfferEntry, OfferLedger, OfferLedgerExpectation, OfferMeasurementWindow,
        OFFER_LEDGER_SCHEMA_VERSION, operation_id_for_offer, sha256_hex, verify_offer_ledger,
    };

    const RUN_ID: &str = "qops-test-run";
    const SCENARIO_ID: &str = "MCRB-03";
    const WORKLOAD_DIGEST: &str =
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const GENERATOR_DIGEST: &str =
        "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const SEED: &str = "qops-test-seed";

    fn measurement_window() -> OfferMeasurementWindow {
        OfferMeasurementWindow {
            window_id: "qops-window".to_string(),
            clock_source: "CLOCK_MONOTONIC".to_string(),
            monotonic_clock_id: "qops-clock".to_string(),
            monotonic_clock_epoch: "qops-boot-epoch".to_string(),
            start_ticks: 10_000,
            end_ticks: 1_000_010_000,
            ticks_per_second: 1_000_000_000,
        }
    }

    fn verified_ledger() -> VerifiedOfferLedger {
        let measurement_window = measurement_window();
        let entries = (0..5)
            .map(|sequence| {
                let request_digest_sha256 = sha256_hex(format!("qops-request-{sequence}").as_bytes());
                OfferEntry {
                    sequence,
                    operation_id: operation_id_for_offer(
                        RUN_ID,
                        SCENARIO_ID,
                        WORKLOAD_DIGEST,
                        GENERATOR_DIGEST,
                        SEED,
                        sequence,
                        &request_digest_sha256,
                    ),
                    request_digest_sha256,
                    idempotency_key: Some(format!("qops-effect-{sequence}")),
                    scheduled_at_ticks: measurement_window.start_ticks + sequence,
                }
            })
            .collect();

        let ledger = OfferLedger {
            schema_version: OFFER_LEDGER_SCHEMA_VERSION.to_string(),
            run_id: RUN_ID.to_string(),
            scenario_id: SCENARIO_ID.to_string(),
            workload_manifest_digest_sha256: WORKLOAD_DIGEST.to_string(),
            generator_digest_sha256: GENERATOR_DIGEST.to_string(),
            seed: SEED.to_string(),
            measurement_window: measurement_window.clone(),
            entry_count: 0,
            entries,
            digest_sha256: String::new(),
        }
        .seal();

        let expectation = OfferLedgerExpectation {
            run_id: RUN_ID.to_string(),
            scenario_id: SCENARIO_ID.to_string(),
            workload_manifest_digest_sha256: WORKLOAD_DIGEST.to_string(),
            generator_digest_sha256: GENERATOR_DIGEST.to_string(),
            seed: SEED.to_string(),
            expected_entry_count: 5,
            measurement_window,
        };

        verify_offer_ledger(&ledger, &expectation).unwrap()
    }

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
        let ledger = verified_ledger();
        let ids = ledger.operation_ids().to_vec();
        QopsInput {
            offer_ledger: ledger,
            outcomes: vec![
                sample(&ids[0], OperationOutcome::QualifiedSuccess),
                sample(&ids[1], OperationOutcome::QualifiedSuccess),
                sample(&ids[2], OperationOutcome::CorrectDenial),
                sample(&ids[3], OperationOutcome::Unknown),
                sample(&ids[4], OperationOutcome::Failed),
            ],
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
        assert_eq!(report.offer_ledger_entry_count, 5);
        assert_eq!(report.offer_ledger_artifact_digest_sha256.len(), 64);
        assert_eq!(report.run_id, RUN_ID);
        assert_eq!(report.scenario_id, SCENARIO_ID);
    }

    #[test]
    fn missing_outcome_makes_qops_incomplete_and_suppresses_rates() {
        let mut input = valid_input();
        input.outcomes.pop();
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert!(report.issues.iter().any(|issue| matches!(issue, QopsIssue::MissingOutcomeSample(_))));
        assert!(report.qualified_operations_per_second.is_none());
        assert!(report.offered_operations_per_second.is_none());
    }

    #[test]
    fn duplicate_operation_outcome_is_invalid() {
        let mut input = valid_input();
        let duplicate_id = input.offer_ledger.operation_ids()[0].clone();
        input.outcomes.push(sample(&duplicate_id, OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.iter().any(|issue| matches!(issue, QopsIssue::DuplicateOutcomeSample(_))));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn unoffered_operation_cannot_enter_the_numerator() {
        let mut input = valid_input();
        input.outcomes.push(sample("mcrb-op-not-in-ledger", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::UnexpectedOperationSample(
            "mcrb-op-not-in-ledger".to_string()
        )));
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

    #[test]
    fn qops_denominator_is_derived_from_the_verified_ledger_not_a_caller_vector() {
        let input = valid_input();
        let expected = input.offer_ledger.entry_count();
        assert_eq!(expected, 5);
        assert_eq!(input.offer_ledger.operation_ids().len(), expected);
        // QopsInput has no expected_operation_ids field: there is no reducer
        // parameter through which a caller can shrink the denominator.
    }
}

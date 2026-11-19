//! Qualified Operations Per Second (QOPS) accounting.
//!
//! QOPS consumes only an opaque VerifiedOfferLedger produced by the
//! offer-ledger verifier. The caller cannot reduce the offered denominator
//! by supplying a smaller operation-ID list after observing outcomes.

use std::collections::{BTreeMap, BTreeSet};

use super::offer_ledger::VerifiedOfferLedger;
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
    pub outcomes: Vec<OperationOutcomeSample>,
    pub hard_correctness_gates_passed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QopsStatus {
    Eligible,
    Invalid,
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QopsIssue {
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
    pub scenario_contract_digest_sha256: String,
    pub offer_ledger_entry_count: usize,
    pub generator_digest_sha256: String,
    pub workload_manifest_digest_sha256: String,
    pub parameter_set_digest_sha256: String,
    pub measurement_window_id: String,
    pub measurement_window_ns: u64,
    pub cross_host_measurement_required: bool,
    pub offered_operations: usize,
    pub classified_outcomes: usize,
    pub qualified_successes: usize,
    pub correct_denials: usize,
    pub unexpected_denials: usize,
    pub known_failures: usize,
    pub unknown_outcomes: usize,
    pub duplicate_external_effects: usize,
    pub issues: Vec<QopsIssue>,
    pub offered_operations_per_second: Option<f64>,
    pub qualified_operations_per_second: Option<f64>,
    pub qualified_success_fraction: Option<f64>,
}

fn push_unique(issues: &mut Vec<QopsIssue>, issue: QopsIssue) {
    if !issues.contains(&issue) {
        issues.push(issue);
    }
}

/// Compute QOPS from a verified, offer-first ledger.
///
/// The offered denominator and measurement window are read from the verified
/// ledger, not from caller-supplied counts, IDs or duration values. The ledger
/// still requires an independent runner/orchestrator authenticity proof before
/// its digest can support a public benchmark claim.
pub fn compute_qops(
    ledger: &VerifiedOfferLedger,
    input: &QopsInput,
    metric_integrity: &IntegrityReport,
) -> QopsReport {
    let mut issues = Vec::new();
    let expected: BTreeSet<String> = ledger.operation_ids().iter().cloned().collect();

    if !input.hard_correctness_gates_passed {
        push_unique(&mut issues, QopsIssue::HardCorrectnessGateFailed);
    }
    if metric_integrity.status != IntegrityStatus::Valid
        || !metric_integrity.aggregate_eligible
        || !metric_integrity.issues.is_empty()
    {
        push_unique(&mut issues, QopsIssue::MetricIntegrityNotValid);
    }
    if ledger.requires_cross_host_visibility()
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
    let status = if has_invalid {
        QopsStatus::Invalid
    } else if !issues.is_empty() {
        QopsStatus::Incomplete
    } else {
        QopsStatus::Eligible
    };

    let offered_operations = ledger.entry_count();
    let classified_outcomes = seen.len();
    let (offered_rate, qualified_rate, success_fraction) =
        if status == QopsStatus::Eligible && offered_operations > 0 {
            let window_seconds = ledger.measurement_window_ns() as f64 / 1_000_000_000.0;
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
        run_id: ledger.run_id().to_string(),
        scenario_id: ledger.scenario_id().to_string(),
        offer_ledger_digest_sha256: ledger.ledger_digest_sha256().to_string(),
        scenario_contract_digest_sha256: ledger.scenario_contract_digest_sha256().to_string(),
        offer_ledger_entry_count: offered_operations,
        generator_digest_sha256: ledger.generator_digest_sha256().to_string(),
        workload_manifest_digest_sha256: ledger.workload_manifest_digest_sha256().to_string(),
        parameter_set_digest_sha256: ledger.parameter_set_digest_sha256().to_string(),
        measurement_window_id: ledger.measurement_window_id().to_string(),
        measurement_window_ns: ledger.measurement_window_ns(),
        cross_host_measurement_required: ledger.requires_cross_host_visibility(),
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
    use super::super::{
        CrossHostTimingStatus, Percentiles,
        offer_ledger::{
            OfferLedgerEntry, OfferLedgerEnvelope, OFFER_LEDGER_SCHEMA_VERSION,
            calculate_offer_ledger_digest, embedded_scenario_manifest_digest, verify_offer_ledger,
        },
    };
    use sha2::{Digest, Sha256};

    fn digest(seed: &str) -> String {
        let result = Sha256::digest(seed.as_bytes());
        let mut hex = String::with_capacity(64);
        for byte in result {
            use std::fmt::Write as _;
            let _ = write!(&mut hex, "{byte:02x}");
        }
        hex
    }

    fn verified_ledger(cross_host_required: bool) -> VerifiedOfferLedger {
        let run_id = "run-001".to_string();
        let scenario_id = if cross_host_required { "MCRB-03" } else { "MCRB-01" }.to_string();
        let generator = digest("generator");
        let manifest = digest("manifest");
        let params = digest("params");
        let clock_id = "clock-a".to_string();
        let epoch = "boot-1".to_string();
        let mut envelope = OfferLedgerEnvelope {
            schema_version: OFFER_LEDGER_SCHEMA_VERSION.to_string(),
            run_id: run_id.clone(),
            scenario_id: scenario_id.clone(),
            measurement_window_id: "window-1".to_string(),
            scenario_contract_digest_sha256: embedded_scenario_manifest_digest(),
            requires_cross_host_visibility: cross_host_required,
            workload_manifest_digest_sha256: manifest.clone(),
            generator_digest_sha256: generator.clone(),
            parameter_set_digest_sha256: params.clone(),
            clock_source: "CLOCK_MONOTONIC_RAW".to_string(),
            monotonic_clock_id: clock_id.clone(),
            monotonic_clock_epoch: epoch.clone(),
            measurement_start_ticks: 0,
            measurement_end_ticks: 5_000_000_000,
            ticks_per_second: 1_000_000_000,
            entry_count: 5,
            entries: (0..5).map(|index| OfferLedgerEntry {
                sequence: index + 1,
                run_id: run_id.clone(),
                scenario_id: scenario_id.clone(),
                operation_id: format!("op-{}", index + 1),
                request_digest_sha256: digest(&format!("request-{index}")),
                idempotency_key: Some(format!("idem-{index}")),
                scheduled_monotonic_tick: index + 10,
                monotonic_clock_id: clock_id.clone(),
                monotonic_clock_epoch: epoch.clone(),
                operation_class: "workflow".to_string(),
                requires_idempotency_key: false,
                generator_digest_sha256: generator.clone(),
                workload_manifest_digest_sha256: manifest.clone(),
                parameter_set_digest_sha256: params.clone(),
            }).collect(),
            ledger_digest_sha256: String::new(),
        };
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        verify_offer_ledger(&envelope).unwrap()
    }

    fn sample(id: &str, outcome: OperationOutcome) -> OperationOutcomeSample {
        OperationOutcomeSample { operation_id: id.to_string(), outcome }
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
            outcomes: vec![
                sample("op-1", OperationOutcome::QualifiedSuccess),
                sample("op-2", OperationOutcome::QualifiedSuccess),
                sample("op-3", OperationOutcome::CorrectDenial),
                sample("op-4", OperationOutcome::Unknown),
                sample("op-5", OperationOutcome::Failed),
            ],
            hard_correctness_gates_passed: true,
        }
    }

    #[test]
    fn qops_counts_only_qualified_successes_and_uses_verified_offer_denominator() {
        let ledger = verified_ledger(false);
        let report = compute_qops(&ledger, &valid_input(), &valid_integrity());
        assert_eq!(report.status, QopsStatus::Eligible);
        assert_eq!(report.offered_operations, 5);
        assert_eq!(report.classified_outcomes, 5);
        assert_eq!(report.qualified_successes, 2);
        assert_eq!(report.correct_denials, 1);
        assert_eq!(report.unknown_outcomes, 1);
        assert_eq!(report.known_failures, 1);
        assert_eq!(report.offered_operations_per_second, Some(1.0));
        assert_eq!(report.qualified_operations_per_second, Some(0.4));
        assert_eq!(report.qualified_success_fraction, Some(0.4));
        assert_eq!(report.offer_ledger_entry_count, 5);
        assert!(!report.cross_host_measurement_required);
        assert_eq!(report.offer_ledger_digest_sha256, ledger.ledger_digest_sha256());
        assert_eq!(report.scenario_contract_digest_sha256, ledger.scenario_contract_digest_sha256());
    }

    #[test]
    fn a_smaller_outcome_set_cannot_reduce_the_offer_denominator() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes.truncate(4);
        let report = compute_qops(&ledger, &input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert_eq!(report.offered_operations, 5);
        assert!(report.issues.contains(&QopsIssue::MissingOutcomeSample("op-5".into())));
        assert!(report.qualified_operations_per_second.is_none());
        assert!(report.offered_operations_per_second.is_none());
    }

    #[test]
    fn duplicate_operation_outcome_is_invalid() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes.push(sample("op-1", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&ledger, &input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::DuplicateOutcomeSample("op-1".into())));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn unoffered_operation_cannot_enter_the_numerator() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes.push(sample("op-not-offered", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&ledger, &input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::UnexpectedOperationSample("op-not-offered".into())));
    }

    #[test]
    fn duplicate_external_effect_is_non_compensatory() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes[0].outcome = OperationOutcome::DuplicateEffect;
        let report = compute_qops(&ledger, &input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert_eq!(report.duplicate_external_effects, 1);
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn failed_correctness_gate_suppresses_all_performance_rates() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.hard_correctness_gates_passed = false;
        let report = compute_qops(&ledger, &input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::HardCorrectnessGateFailed));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn invalid_metric_integrity_suppresses_all_performance_rates() {
        let ledger = verified_ledger(false);
        let input = valid_input();
        let mut integrity = valid_integrity();
        integrity.status = IntegrityStatus::Invalid;
        integrity.aggregate_eligible = false;
        integrity.issues.push(super::super::MetricIssue::ReversedMonotonicTicks(0));
        let report = compute_qops(&ledger, &input, &integrity);
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::MetricIntegrityNotValid));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn required_cross_host_visibility_cannot_use_local_only_metrics() {
        let ledger = verified_ledger(true);
        let input = valid_input();
        let report = compute_qops(&ledger, &input, &valid_integrity());
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.cross_host_measurement_required);
        assert!(report.issues.contains(&QopsIssue::CrossHostMetricIntegrityNotValid));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn required_cross_host_visibility_accepts_only_valid_cross_host_batch() {
        let ledger = verified_ledger(true);
        let input = valid_input();
        let mut integrity = valid_integrity();
        integrity.cross_host_status = CrossHostTimingStatus::Valid;
        integrity.cross_host_aggregate_eligible = true;
        let report = compute_qops(&ledger, &input, &integrity);
        assert_eq!(report.status, QopsStatus::Eligible);
        assert!(report.cross_host_measurement_required);
        assert_eq!(report.qualified_operations_per_second, Some(0.4));
    }
}

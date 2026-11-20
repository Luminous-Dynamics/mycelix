//! Qualified Operations Per Second (QOPS) accounting.
//!
//! QOPS consumes three verifier-created capabilities: an offer-first ledger,
//! a validated raw metric batch and a scenario-bound correctness-gate set.
//! Caller-supplied booleans and caller-constructed "Valid" report structs are
//! not accepted as evidence of correctness.

use std::collections::{BTreeMap, BTreeSet};

use super::attestation::{GateSetStatus, GateSetVerification};
use super::offer_ledger::VerifiedOfferLedger;
use super::{CrossHostTimingStatus, MetricIntegrityVerification};

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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QopsStatus {
    Eligible,
    Invalid,
    Incomplete,
}

/// Independent per-operation outcome authentication is deliberately not
/// implemented yet. QOPS cannot become Eligible until #6357 supplies an
/// opaque VerifiedOutcomeLedger.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum OutcomeEvidenceStatus {
    NotImplemented,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QopsIssue {
    MetricIntegrityNotValid,
    MetricIntegrityIncomplete,
    CrossHostMetricIntegrityNotValid,
    CrossHostMetricIntegrityIncomplete,
    CorrectnessGateSetNotVerified,
    CorrectnessGateSetIncomplete,
    CorrectnessGateSetContextMismatch,
    OutcomeEvidenceNotVerified,
    EmptySampleOperationId(usize),
    UnexpectedOperationSample(String),
    DuplicateOutcomeSample(String),
    MissingOutcomeSample(String),
}

/// Caller-supplied outcome labels are unverified diagnostics until #6357
/// produces an opaque VerifiedOutcomeLedger. These counts must never be used
/// as QOPS numerator, terminal-outcome facts, or correctness evidence.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AssertedOutcomeSamples {
    pub classified_samples: usize,
    pub qualified_success_labels: usize,
    pub correct_denial_labels: usize,
    pub unexpected_denial_labels: usize,
    pub failure_labels: usize,
    pub unknown_labels: usize,
    pub duplicate_effect_labels: usize,
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
    pub correctness_gate_set_digest_sha256: Option<String>,
    pub correctness_trust_policy_id: Option<String>,
    pub correctness_trust_policy_digest_sha256: Option<String>,
    pub correctness_required_gate_count: usize,
    pub correctness_attestation_count: usize,
    pub outcome_evidence_status: OutcomeEvidenceStatus,
    pub outcome_ledger_digest_sha256: Option<String>,
    pub outcome_ledger_entry_count: usize,
    pub outcome_attestation_count: usize,
    /// Diagnostic labels supplied by the caller; never eligible outcome facts.
    pub asserted_outcome_samples: AssertedOutcomeSamples,
    pub offered_operations: usize,
    /// Counts from VerifiedOutcomeLedger only. Remain zero while outcome
    /// attestation is NotImplemented.
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

/// Compute a fail-closed QOPS report from verified denominator/timing/gate
/// capabilities. Caller outcome labels are retained as diagnostics only; until
/// #6357 supplies a VerifiedOutcomeLedger, authoritative outcome counts stay
/// zero and no offered/qualified performance rate is emitted.
pub fn compute_qops(
    ledger: &VerifiedOfferLedger,
    input: &QopsInput,
    metric_verification: &MetricIntegrityVerification,
    gate_verification: &GateSetVerification,
) -> QopsReport {
    let mut issues = Vec::new();
    let expected: BTreeSet<String> = ledger.operation_ids().iter().cloned().collect();

    // Outcome samples are still caller-supplied diagnostic observations. Until
    // #6357 provides a signed, independently verified outcome ledger, no QOPS
    // rate may be labelled eligible regardless of the other verified inputs.
    push_unique(&mut issues, QopsIssue::OutcomeEvidenceNotVerified);

    let metric_report = match metric_verification {
        MetricIntegrityVerification::Verified(verified) => Some(verified.report()),
        MetricIntegrityVerification::Rejected(report) => {
            if report.status == super::IntegrityStatus::Incomplete {
                push_unique(&mut issues, QopsIssue::MetricIntegrityIncomplete);
            } else {
                push_unique(&mut issues, QopsIssue::MetricIntegrityNotValid);
            }
            None
        }
    };

    let verified_gate_set = match gate_verification {
        GateSetVerification::Verified(value) => {
            if value.run_id() != ledger.run_id()
                || value.scenario_id() != ledger.scenario_id()
                || value.scenario_contract_digest_sha256()
                    != ledger.scenario_contract_digest_sha256()
            {
                push_unique(&mut issues, QopsIssue::CorrectnessGateSetContextMismatch);
                None
            } else {
                Some(value)
            }
        }
        GateSetVerification::Rejected(report) => {
            if report.status == GateSetStatus::Incomplete {
                push_unique(&mut issues, QopsIssue::CorrectnessGateSetIncomplete);
            } else {
                push_unique(&mut issues, QopsIssue::CorrectnessGateSetNotVerified);
            }
            None
        }
    };

    if let Some(report) = metric_report {
        if report.status != super::IntegrityStatus::Valid
            || !report.aggregate_eligible
            || !report.issues.is_empty()
        {
            push_unique(&mut issues, QopsIssue::MetricIntegrityNotValid);
        }
        if ledger.requires_cross_host_visibility() {
            match report.cross_host_status {
                CrossHostTimingStatus::Valid if report.cross_host_aggregate_eligible => {}
                CrossHostTimingStatus::Incomplete => {
                    push_unique(&mut issues, QopsIssue::CrossHostMetricIntegrityIncomplete);
                }
                _ => {
                    push_unique(&mut issues, QopsIssue::CrossHostMetricIntegrityNotValid);
                }
            }
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
    }

    for operation_id in &expected {
        if !seen.contains(operation_id) {
            push_unique(
                &mut issues,
                QopsIssue::MissingOutcomeSample(operation_id.clone()),
            );
        }
    }

    // Preserve caller labels for diagnostics, but do not publish them as
    // observed terminal outcomes. Only a VerifiedOutcomeLedger may populate
    // the authoritative counters used by receipts and QOPS.
    let asserted_outcome_samples = AssertedOutcomeSamples {
        classified_samples: seen.len(),
        qualified_success_labels: counts.get(&OperationOutcome::QualifiedSuccess).copied().unwrap_or(0),
        correct_denial_labels: counts.get(&OperationOutcome::CorrectDenial).copied().unwrap_or(0),
        unexpected_denial_labels: counts.get(&OperationOutcome::UnexpectedDenial).copied().unwrap_or(0),
        failure_labels: counts.get(&OperationOutcome::Failed).copied().unwrap_or(0),
        unknown_labels: counts.get(&OperationOutcome::Unknown).copied().unwrap_or(0),
        duplicate_effect_labels: counts.get(&OperationOutcome::DuplicateEffect).copied().unwrap_or(0),
    };

    // Outcome authentication is not implemented yet: keep all authoritative
    // terminal-outcome counters at zero until the verifier creates a
    // VerifiedOutcomeLedger bound to this exact VerifiedOfferLedger.
    let classified_outcomes = 0;
    let qualified_successes = 0;
    let correct_denials = 0;
    let unexpected_denials = 0;
    let known_failures = 0;
    let unknown_outcomes = 0;
    let duplicate_external_effects = 0;

    let is_incomplete = issues.iter().any(|issue| {
        matches!(
            issue,
            QopsIssue::MissingOutcomeSample(_)
                | QopsIssue::MetricIntegrityIncomplete
                | QopsIssue::CrossHostMetricIntegrityIncomplete
                | QopsIssue::CorrectnessGateSetIncomplete
                | QopsIssue::OutcomeEvidenceNotVerified
        )
    });
    let is_invalid = issues.iter().any(|issue| {
        !matches!(
            issue,
            QopsIssue::MissingOutcomeSample(_)
                | QopsIssue::MetricIntegrityIncomplete
                | QopsIssue::CrossHostMetricIntegrityIncomplete
                | QopsIssue::CorrectnessGateSetIncomplete
                | QopsIssue::OutcomeEvidenceNotVerified
        )
    });
    let status = if is_invalid {
        QopsStatus::Invalid
    } else if is_incomplete {
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
        correctness_gate_set_digest_sha256: verified_gate_set
            .map(|set| set.gate_set_digest_sha256().to_string()),
        correctness_trust_policy_id: verified_gate_set
            .map(|set| set.trust_policy_id().to_string()),
        correctness_trust_policy_digest_sha256: verified_gate_set
            .map(|set| set.trust_policy_digest_sha256().to_string()),
        correctness_required_gate_count: verified_gate_set
            .map(|set| set.required_gate_ids().len())
            .unwrap_or(0),
        correctness_attestation_count: verified_gate_set
            .map(|set| set.attestation_digests_sha256().len())
            .unwrap_or(0),
        outcome_evidence_status: OutcomeEvidenceStatus::NotImplemented,
        outcome_ledger_digest_sha256: None,
        outcome_ledger_entry_count: 0,
        outcome_attestation_count: 0,
        asserted_outcome_samples,
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
        CrossHostTimingStatus, IntegrityReport, IntegrityStatus, MetricIntegrityVerification,
        Percentiles, VerifiedMetricIntegrity,
        attestation::{GateAttestationContext, verified_gate_set_for_test},
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

    fn valid_integrity() -> MetricIntegrityVerification {
        let report = IntegrityReport {
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
        };
        MetricIntegrityVerification::Verified(VerifiedMetricIntegrity { report })
    }

    fn valid_input() -> QopsInput {
        QopsInput {
            outcomes: vec![
                sample("op-1", OperationOutcome::QualifiedSuccess),
                sample("op-2", OperationOutcome::QualifiedSuccess),
                sample("op-3", OperationOutcome::CorrectDenial),
                sample("op-4", OperationOutcome::Failed),
                sample("op-5", OperationOutcome::UnexpectedDenial),
            ],
        }
    }

    fn valid_gate_set(ledger: &VerifiedOfferLedger) -> GateSetVerification {
        GateSetVerification::Verified(verified_gate_set_for_test(
            ledger.run_id(),
            ledger.scenario_id(),
            ledger.scenario_contract_digest_sha256(),
        ))
    }

    #[test]
    fn qops_requires_three_verifier_capabilities_and_uses_verified_denominator() {
        let ledger = verified_ledger(false);
        let metrics = valid_integrity();
        let gates = valid_gate_set(&ledger);
        let report = compute_qops(&ledger, &valid_input(), &metrics, &gates);
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert_eq!(report.outcome_evidence_status, OutcomeEvidenceStatus::NotImplemented);
        assert!(report.issues.contains(&QopsIssue::OutcomeEvidenceNotVerified));
        assert!(report.offered_operations_per_second.is_none());
        assert!(report.qualified_operations_per_second.is_none());
        assert_eq!(report.offered_operations, 5);
        assert_eq!(report.classified_outcomes, 0);
        assert_eq!(report.qualified_successes, 0);
        assert_eq!(report.correct_denials, 0);
        assert_eq!(report.known_failures, 0);
        assert_eq!(report.unexpected_denials, 0);
        assert_eq!(report.asserted_outcome_samples.classified_samples, 5);
        assert_eq!(report.asserted_outcome_samples.qualified_success_labels, 2);
        assert_eq!(report.asserted_outcome_samples.correct_denial_labels, 1);
        assert_eq!(report.asserted_outcome_samples.failure_labels, 1);
        assert_eq!(report.asserted_outcome_samples.unexpected_denial_labels, 1);
        assert!(report.offered_operations_per_second.is_none());
        assert!(report.qualified_operations_per_second.is_none());
        assert!(report.qualified_success_fraction.is_none());
        let expected_gate_count = verified_gate_set_for_test(
            ledger.run_id(),
            ledger.scenario_id(),
            ledger.scenario_contract_digest_sha256(),
        )
        .required_gate_ids()
        .len();
        assert_eq!(report.correctness_required_gate_count, expected_gate_count);
        assert_eq!(report.correctness_attestation_count, expected_gate_count);
        assert!(report.correctness_gate_set_digest_sha256.is_some());
        assert_eq!(report.offer_ledger_digest_sha256, ledger.ledger_digest_sha256());
    }

    #[test]
    fn a_smaller_outcome_set_cannot_reduce_the_offer_denominator() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes.truncate(4);
        let report = compute_qops(&ledger, &input, &valid_integrity(), &valid_gate_set(&ledger));
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert_eq!(report.offered_operations, 5);
        assert!(report.issues.contains(&QopsIssue::MissingOutcomeSample("op-5".into())));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn duplicate_operation_outcome_is_invalid() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes.push(sample("op-1", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&ledger, &input, &valid_integrity(), &valid_gate_set(&ledger));
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::DuplicateOutcomeSample("op-1".into())));
    }

    #[test]
    fn unoffered_operation_cannot_enter_the_numerator() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes.push(sample("op-not-offered", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&ledger, &input, &valid_integrity(), &valid_gate_set(&ledger));
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::UnexpectedOperationSample("op-not-offered".into())));
    }

    #[test]
    fn duplicate_effect_label_is_diagnostic_until_independently_verified() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes[0].outcome = OperationOutcome::DuplicateEffect;
        let report = compute_qops(&ledger, &input, &valid_integrity(), &valid_gate_set(&ledger));
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert_eq!(report.duplicate_external_effects, 0);
        assert_eq!(report.asserted_outcome_samples.duplicate_effect_labels, 1);
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn unknown_label_is_diagnostic_until_independently_verified() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes[4].outcome = OperationOutcome::Unknown;
        let report = compute_qops(&ledger, &input, &valid_integrity(), &valid_gate_set(&ledger));
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert_eq!(report.unknown_outcomes, 0);
        assert_eq!(report.asserted_outcome_samples.unknown_labels, 1);
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn a_rejected_correctness_gate_set_cannot_be_overridden_by_caller_data() {
        let ledger = verified_ledger(false);
        let rejected = GateSetVerification::Rejected(super::super::attestation::GateSetReport {
            status: GateSetStatus::Incomplete,
            run_id: ledger.run_id().to_string(),
            scenario_id: ledger.scenario_id().to_string(),
            scenario_contract_digest_sha256: ledger.scenario_contract_digest_sha256().to_string(),
            trust_policy_id: "test-policy".to_string(),
            trust_policy_digest_sha256: digest("policy"),
            required_gate_count: verified_gate_set_for_test(
                ledger.run_id(),
                ledger.scenario_id(),
                ledger.scenario_contract_digest_sha256(),
            )
            .required_gate_ids()
            .len(),
            verified_pass_count: 0,
            gate_set_digest_sha256: None,
            attestation_digests_sha256: vec![],
            issues: vec![],
        });
        let report = compute_qops(&ledger, &valid_input(), &valid_integrity(), &rejected);
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert!(report.issues.contains(&QopsIssue::CorrectnessGateSetIncomplete));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn gate_set_must_bind_to_the_exact_ledger_run_and_scenario() {
        let ledger = verified_ledger(false);
        let wrong = GateSetVerification::Verified(verified_gate_set_for_test(
            "other-run",
            ledger.scenario_id(),
            ledger.scenario_contract_digest_sha256(),
        ));
        let report = compute_qops(&ledger, &valid_input(), &valid_integrity(), &wrong);
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::CorrectnessGateSetContextMismatch));
    }

    #[test]
    fn rejected_metric_integrity_suppresses_all_rates() {
        let ledger = verified_ledger(false);
        let mut report = match valid_integrity() {
            MetricIntegrityVerification::Verified(value) => value.report().clone(),
            MetricIntegrityVerification::Rejected(_) => unreachable!(),
        };
        report.status = IntegrityStatus::Invalid;
        report.aggregate_eligible = false;
        report.issues.push(super::super::MetricIssue::ReversedMonotonicTicks(0));
        let rejected = MetricIntegrityVerification::Rejected(report);
        let result = compute_qops(&ledger, &valid_input(), &rejected, &valid_gate_set(&ledger));
        assert_eq!(result.status, QopsStatus::Invalid);
        assert!(result.issues.contains(&QopsIssue::MetricIntegrityNotValid));
        assert!(result.qualified_operations_per_second.is_none());
    }

    #[test]
    fn outcome_evidence_status_serializes_to_receipt_vocabulary() {
        let value = serde_json::to_string(&OutcomeEvidenceStatus::NotImplemented).unwrap();
        assert_eq!(value, "\\"not-implemented\\"");
    }

    #[test]
    fn required_cross_host_visibility_cannot_use_local_only_metrics() {
        let ledger = verified_ledger(true);
        let report = compute_qops(&ledger, &valid_input(), &valid_integrity(), &valid_gate_set(&ledger));
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.cross_host_measurement_required);
        assert!(report.issues.contains(&QopsIssue::CrossHostMetricIntegrityNotValid));
        assert!(report.qualified_operations_per_second.is_none());
    }
}

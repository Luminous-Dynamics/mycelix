//! Qualified Operations Per Second (QOPS) accounting.
//!
//! QOPS consumes only an opaque VerifiedOfferLedger produced by the
//! offer-ledger verifier. The caller cannot reduce the offered denominator
//! by supplying a smaller operation-ID list after observing outcomes.

use std::collections::{BTreeMap, BTreeSet};

use super::gate_attestation::{GateVerificationReport, GateVerificationStatus};
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
    CorrectnessGateSetIncomplete,
    CorrectnessGateEvidenceInvalid,
    CorrectnessGateContextMismatch,
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
    pub gate_verification_status: GateVerificationStatus,
    pub gate_policy_id: String,
    pub gate_verifier_version: String,
    pub gate_trust_policy_digest_sha256: String,
    pub gate_attestation_set_digest_sha256: String,
    pub gate_source_sha: String,
    pub gate_runtime_identity_digest_sha256: String,
    pub gate_required_count: usize,
    pub gate_passed_count: usize,
    pub gate_failed_count: usize,
    pub gate_incomplete_count: usize,
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
    gate_verification: &GateVerificationReport,
) -> QopsReport {
    let mut issues = Vec::new();
    let expected: BTreeSet<String> = ledger.operation_ids().iter().cloned().collect();

    if !gate_verification.matches_offer_context(
        ledger.run_id(),
        ledger.scenario_id(),
        ledger.scenario_contract_digest_sha256(),
    ) {
        push_unique(&mut issues, QopsIssue::CorrectnessGateContextMismatch);
    }
    match gate_verification.status() {
        GateVerificationStatus::VerifiedPass => {}
        GateVerificationStatus::Failed => {
            push_unique(&mut issues, QopsIssue::HardCorrectnessGateFailed);
        }
        GateVerificationStatus::Incomplete => {
            push_unique(&mut issues, QopsIssue::CorrectnessGateSetIncomplete);
        }
        GateVerificationStatus::Invalid => {
            push_unique(&mut issues, QopsIssue::CorrectnessGateEvidenceInvalid);
        }
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
        !matches!(
            issue,
            QopsIssue::MissingOutcomeSample(_) | QopsIssue::CorrectnessGateSetIncomplete
        )
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
        gate_verification_status: gate_verification.status(),
        gate_policy_id: gate_verification.policy_id().to_string(),
        gate_verifier_version: gate_verification.verifier_version().to_string(),
        gate_trust_policy_digest_sha256: gate_verification.trust_policy_digest_sha256().to_string(),
        gate_attestation_set_digest_sha256: gate_verification.attestation_set_digest_sha256().to_string(),
        gate_source_sha: gate_verification.source_sha().to_string(),
        gate_runtime_identity_digest_sha256: gate_verification.runtime_identity_digest_sha256().to_string(),
        gate_required_count: gate_verification.required_gate_count(),
        gate_passed_count: gate_verification.passed_gate_count(),
        gate_failed_count: gate_verification.failed_gate_count(),
        gate_incomplete_count: gate_verification.incomplete_gate_count(),
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
        gate_attestation::{
            GATE_ATTESTATION_SCHEMA_VERSION, GATE_POLICY_SCHEMA_VERSION, GateAttestation,
            GateOutcome, GateVerifierPolicy, TrustedGateSigner, gate_attestation_signing_bytes,
            verify_gate_attestations,
        },
        offer_ledger::{
            OfferLedgerEntry, OfferLedgerEnvelope, OFFER_LEDGER_SCHEMA_VERSION,
            calculate_offer_ledger_digest, embedded_scenario_manifest_digest, verify_offer_ledger,
        },
    };
    use ed25519_dalek::{Signer, SigningKey};
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
        }
    }

    fn gate_report(scenario_id: &str, outcome: GateOutcome, omit_last: bool) -> GateVerificationReport {
        gate_report_for_run("run-001", scenario_id, outcome, omit_last)
    }

    fn gate_report_for_run(
        run_id: &str,
        scenario_id: &str,
        outcome: GateOutcome,
        omit_last: bool,
    ) -> GateVerificationReport {
        let signing_key = SigningKey::from_bytes(&[17u8; 32]);
        let mut public_key_hex = String::new();
        for byte in signing_key.verifying_key().to_bytes() {
            use std::fmt::Write as _;
            let _ = write!(&mut public_key_hex, "{byte:02x}");
        }
        let policy = GateVerifierPolicy {
            schema_version: GATE_POLICY_SCHEMA_VERSION.to_string(),
            policy_id: "qops-fixture-policy".to_string(),
            run_id: run_id.to_string(),
            scenario_id: scenario_id.to_string(),
            scenario_contract_digest_sha256: embedded_scenario_manifest_digest(),
            source_sha: "a".repeat(40),
            runtime_identity_digest_sha256: "b".repeat(64),
            minimum_distinct_signers_per_gate: 1,
            trusted_signers: vec![TrustedGateSigner {
                key_id: "qops-fixture-verifier".to_string(),
                public_key_ed25519_hex: public_key_hex,
            }],
        };
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../../../../docs/benchmarks/MCRB_V0_1_SCENARIO_MANIFEST.json")).unwrap();
        let scenarios = manifest["scenarios"].as_array().unwrap();
        let scenario = scenarios.iter().find(|item| item["id"].as_str() == Some(scenario_id)).unwrap();
        let mut gate_ids: Vec<String> = scenario["hard_invariants"].as_array().unwrap()
            .iter().map(|item| item.as_str().unwrap().to_string()).collect();
        gate_ids.sort();
        if omit_last { gate_ids.pop(); }

        let mut attestations = Vec::new();
        for gate_id in gate_ids {
            let mut attestation = GateAttestation {
                schema_version: GATE_ATTESTATION_SCHEMA_VERSION.to_string(),
                run_id: policy.run_id.clone(),
                scenario_id: policy.scenario_id.clone(),
                scenario_contract_digest_sha256: policy.scenario_contract_digest_sha256.clone(),
                source_sha: policy.source_sha.clone(),
                runtime_identity_digest_sha256: policy.runtime_identity_digest_sha256.clone(),
                gate_id,
                outcome,
                evidence_artifact_digests_sha256: vec![digest("qops-gate-evidence")],
                verifier_key_id: "qops-fixture-verifier".to_string(),
                verifier_version: "test-verifier/0.1".to_string(),
                signature_ed25519_hex: String::new(),
            };
            attestation.signature_ed25519_hex = signing_key.sign(&gate_attestation_signing_bytes(&attestation))
                .to_bytes().iter().map(|byte| format!("{byte:02x}")).collect();
            attestations.push(attestation);
        }
        let pinned_trust_policy_digest = super::super::gate_attestation::gate_verifier_policy_digest(&policy);
        verify_gate_attestations(&policy, &pinned_trust_policy_digest, &attestations)
    }

    #[test]
    fn qops_counts_only_qualified_successes_and_uses_verified_offer_denominator() {
        let ledger = verified_ledger(false);
        let report = compute_qops(&ledger, &valid_input(), &valid_integrity(), &gate_report("MCRB-01", GateOutcome::Pass, false));
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
        let report = compute_qops(&ledger, &input, &valid_integrity(), &gate_report("MCRB-01", GateOutcome::Pass, false));
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
        let report = compute_qops(&ledger, &input, &valid_integrity(), &gate_report("MCRB-01", GateOutcome::Pass, false));
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::DuplicateOutcomeSample("op-1".into())));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn unoffered_operation_cannot_enter_the_numerator() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes.push(sample("op-not-offered", OperationOutcome::QualifiedSuccess));
        let report = compute_qops(&ledger, &input, &valid_integrity(), &gate_report("MCRB-01", GateOutcome::Pass, false));
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::UnexpectedOperationSample("op-not-offered".into())));
    }

    #[test]
    fn duplicate_external_effect_is_non_compensatory() {
        let ledger = verified_ledger(false);
        let mut input = valid_input();
        input.outcomes[0].outcome = OperationOutcome::DuplicateEffect;
        let report = compute_qops(&ledger, &input, &valid_integrity(), &gate_report("MCRB-01", GateOutcome::Pass, false));
        assert_eq!(report.status, QopsStatus::Invalid);
        assert_eq!(report.duplicate_external_effects, 1);
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn failed_signed_correctness_gate_suppresses_all_performance_rates() {
        let ledger = verified_ledger(false);
        let input = valid_input();
        let report = compute_qops(
            &ledger,
            &input,
            &valid_integrity(),
            &gate_report("MCRB-01", GateOutcome::Fail, false),
        );
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::HardCorrectnessGateFailed));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn incomplete_signed_correctness_gate_suppresses_all_performance_rates() {
        let ledger = verified_ledger(false);
        let input = valid_input();
        let report = compute_qops(
            &ledger,
            &input,
            &valid_integrity(),
            &gate_report("MCRB-01", GateOutcome::Pass, true),
        );
        assert_eq!(report.status, QopsStatus::Incomplete);
        assert!(report.issues.contains(&QopsIssue::CorrectnessGateSetIncomplete));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn gate_attestation_from_another_run_cannot_qualify_this_offer_ledger() {
        let ledger = verified_ledger(false);
        let input = valid_input();
        let other_run_gates =
            gate_report_for_run("different-run", "MCRB-01", GateOutcome::Pass, false);
        assert_eq!(other_run_gates.status(), GateVerificationStatus::Invalid);
        let report = compute_qops(&ledger, &input, &valid_integrity(), &other_run_gates);
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::CorrectnessGateContextMismatch));
        assert!(report.qualified_operations_per_second.is_none());
        assert!(report.offered_operations_per_second.is_none());
    }

    #[test]
    fn invalid_metric_integrity_suppresses_all_performance_rates() {
        let ledger = verified_ledger(false);
        let input = valid_input();
        let mut integrity = valid_integrity();
        integrity.status = IntegrityStatus::Invalid;
        integrity.aggregate_eligible = false;
        integrity.issues.push(super::super::MetricIssue::ReversedMonotonicTicks(0));
        let report = compute_qops(&ledger, &input, &integrity, &gate_report("MCRB-01", GateOutcome::Pass, false));
        assert_eq!(report.status, QopsStatus::Invalid);
        assert!(report.issues.contains(&QopsIssue::MetricIntegrityNotValid));
        assert!(report.qualified_operations_per_second.is_none());
    }

    #[test]
    fn required_cross_host_visibility_cannot_use_local_only_metrics() {
        let ledger = verified_ledger(true);
        let input = valid_input();
        let report = compute_qops(&ledger, &input, &valid_integrity(), &gate_report("MCRB-03", GateOutcome::Pass, false));
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
        let report = compute_qops(&ledger, &input, &integrity, &gate_report("MCRB-03", GateOutcome::Pass, false));
        assert_eq!(report.status, QopsStatus::Eligible);
        assert!(report.cross_host_measurement_required);
        assert_eq!(report.qualified_operations_per_second, Some(0.4));
    }
}

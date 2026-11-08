//! Deterministic Track A replay evaluator for privacy-minimized civilian event envelopes.
//!
//! This evaluates downstream evidence handling on identical frozen synthetic event cases.
//! It does NOT evaluate physical sensor detection, tracking, intent, aircraft performance,
//! drone mitigation, third-party control, or operational protection.

use gcpn_awareness_industry_comparison_verifier as comparison_contract;
use gcpn_civil_autonomous_awareness_verifier as awareness_contract;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const RUN_SCHEMA: &str = "luminous.gcpn.track_a_replay_run.v0.1";
pub const REPORT_SCHEMA: &str = "luminous.gcpn.track_a_replay_report.v0.1";
pub const RUN_CLAIM_CEILING: &str =
    "common_event_envelope_replay_only_no_physical_sensor_detection_or_field_claim";
pub const AWARENESS_CASE_SHA256: &str =
    "317ed98191606c4e0a58f8445ec29df3c9851dfb4968a33faf80dfc43eaacaa4";
pub const COMPARISON_MANIFEST_SHA256: &str =
    "3575cd6524db0ecbe9f6de282a63dcbf84a016f6bde25151888931bf46cadc81";

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunKind {
    SyntheticCalculatorSmoke,
    CandidateComparison,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayRun {
    pub schema: String,
    pub run_kind: RunKind,
    pub claim_ceiling: String,
    pub run_id: String,
    pub site_profile_ref: String,
    pub comparison_protocol_sha256: String,
    pub awareness_case_manifest_sha256: String,
    pub event_frame_sha256: String,
    pub operator_review_budget_minutes_per_case: u32,
    pub baseline_workflow_id: String,
    pub candidate_workflow_ids: Vec<String>,
    pub workflows: Vec<WorkflowRun>,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkflowKind {
    RulesBaseline,
    ManualSiteWorkflow,
    EvidenceAssistant,
    VendorEventAdapter,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorkflowRun {
    pub workflow_id: String,
    pub workflow_kind: WorkflowKind,
    pub build_ref: String,
    pub configuration_sha256: String,
    pub event_frame_sha256: String,
    pub operator_review_budget_minutes_per_case: u32,
    pub case_results: Vec<CaseResult>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseResult {
    pub case_id: String,
    pub reported_state: String,
    pub abstained_from_intent_classification: bool,
    pub provenance_complete: bool,
    pub source_status_preserved: bool,
    pub contradiction_preserved: bool,
    pub human_review_required: bool,
    pub event_and_receipt_times_separated: bool,
    pub correction_lineage_preserved: bool,
    pub privacy_compliant: bool,
    pub privacy_violation_count: u32,
    pub authority_violation_count: u32,
    pub vendor_hostile_label_used_as_truth: bool,
    pub duplicate_alert_count: u32,
    pub unsubstantiated_alert_count: u32,
    pub review_minutes: u32,
}

#[derive(Clone, Debug, Serialize)]
pub struct Rate {
    pub numerator: u64,
    pub denominator: u64,
    /// Integer basis points: 10,000 = 100%.
    pub basis_points: u32,
}

#[derive(Clone, Debug, Serialize)]
pub struct WorkflowMetrics {
    pub workflow_id: String,
    pub workflow_kind: String,
    pub case_count: usize,
    pub exact_state_matches: usize,
    pub exact_state_match_rate: Rate,
    pub provenance_complete_cases: usize,
    pub provenance_complete_rate: Rate,
    pub source_status_preserved_cases: usize,
    pub source_status_preserved_rate: Rate,
    pub contradiction_required_cases: usize,
    pub contradictions_preserved_cases: usize,
    pub contradiction_preservation_rate: Rate,
    pub intent_abstention_cases: usize,
    pub intent_abstention_rate: Rate,
    pub human_review_required_cases: usize,
    pub human_review_required_rate: Rate,
    pub event_receipt_time_separation_cases: usize,
    pub event_receipt_time_separation_rate: Rate,
    pub correction_lineage_cases: usize,
    pub correction_lineage_rate: Rate,
    pub privacy_compliant_cases: usize,
    pub privacy_compliance_rate: Rate,
    pub privacy_violation_count: u64,
    pub authority_violation_count: u64,
    pub vendor_hostile_label_used_count: u64,
    pub duplicate_alert_count: u64,
    pub unsubstantiated_alert_count: u64,
    pub review_minutes_total: u64,
    pub review_minutes_mean_milli: u64,
    pub review_minutes_median: u32,
    pub review_minutes_p95: u32,
    pub review_budget_exceeded_cases: usize,
}

#[derive(Clone, Debug, Serialize)]
pub struct PairwiseComparison {
    pub baseline_workflow_id: String,
    pub candidate_workflow_id: String,
    pub review_workload_reduction_basis_points: Option<i64>,
    pub duplicate_alert_reduction_basis_points: Option<i64>,
    pub unsubstantiated_alert_reduction_basis_points: Option<i64>,
    pub state_conformance_non_degradation: bool,
    pub provenance_non_degradation: bool,
    pub contradiction_non_degradation: bool,
    pub candidate_meets_20_percent_smoke_hypothesis: bool,
    pub interpretation: &'static str,
}

#[derive(Clone, Debug, Serialize)]
pub struct EvaluationReport {
    pub report_schema: &'static str,
    pub evaluation_status: &'static str,
    pub run_id: Option<String>,
    pub run_sha256: String,
    pub comparison_protocol_sha256: String,
    pub awareness_case_manifest_sha256: String,
    pub case_count: usize,
    pub workflow_count: usize,
    pub workflows: Vec<WorkflowMetrics>,
    pub pairwise_comparisons: Vec<PairwiseComparison>,
    pub errors: Vec<String>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug)]
struct CaseExpectation {
    expected_state: String,
    contradiction_required: bool,
}

pub fn evaluate(
    awareness_case_bytes: &[u8],
    comparison_manifest_bytes: &[u8],
    replay_run_bytes: &[u8],
) -> EvaluationReport {
    let run_sha256 = sha256_hex(replay_run_bytes);
    let mut errors = Vec::new();

    let awareness_digest = sha256_hex(awareness_case_bytes);
    let comparison_digest = sha256_hex(comparison_manifest_bytes);

    if awareness_digest != AWARENESS_CASE_SHA256 {
        errors.push("awareness_case_manifest_digest_mismatch".into());
    }
    if comparison_digest != COMPARISON_MANIFEST_SHA256 {
        errors.push("comparison_manifest_digest_mismatch".into());
    }

    let awareness_report = awareness_contract::verify(awareness_case_bytes);
    if awareness_report.status != "CONFORMS" {
        errors.push("awareness_case_manifest_rejected".into());
    }
    let comparison_report = comparison_contract::verify(comparison_manifest_bytes);
    if comparison_report.status != "CONFORMS" {
        errors.push("comparison_manifest_rejected".into());
    }

    let awareness_manifest =
        match serde_json::from_slice::<awareness_contract::Manifest>(awareness_case_bytes) {
            Ok(value) => Some(value),
            Err(error) => {
                errors.push(format!("awareness_manifest_decode: {error}"));
                None
            }
        };

    let run = match serde_json::from_slice::<ReplayRun>(replay_run_bytes) {
        Ok(value) => Some(value),
        Err(error) => {
            errors.push(format!("replay_run_decode: {error}"));
            None
        }
    };

    let (Some(awareness_manifest), Some(run)) = (awareness_manifest, run) else {
        errors.sort();
        return rejected_report(run_sha256, comparison_digest, awareness_digest, errors);
    };

    if run.schema != RUN_SCHEMA {
        errors.push("run_schema_mismatch".into());
    }
    if run.claim_ceiling != RUN_CLAIM_CEILING {
        errors.push("run_claim_ceiling_mismatch".into());
    }
    if run.run_id.trim().is_empty() || run.site_profile_ref.trim().is_empty() {
        errors.push("run_metadata_incomplete".into());
    }
    if run.comparison_protocol_sha256 != COMPARISON_MANIFEST_SHA256 {
        errors.push("run_comparison_protocol_digest_mismatch".into());
    }
    if run.awareness_case_manifest_sha256 != AWARENESS_CASE_SHA256 {
        errors.push("run_awareness_manifest_digest_mismatch".into());
    }
    if run.event_frame_sha256 != AWARENESS_CASE_SHA256 {
        errors.push("run_event_frame_mismatch".into());
    }
    if run.operator_review_budget_minutes_per_case == 0 {
        errors.push("operator_review_budget_must_be_positive".into());
    }

    let expected_cases: BTreeMap<String, CaseExpectation> = awareness_manifest
        .cases
        .iter()
        .map(|case| {
            let contradiction_required = matches!(
                case.id.as_str(),
                "CAA-004" | "CAA-007" | "CAA-008" | "CAA-009" | "CAA-010"
            );
            (
                case.id.clone(),
                CaseExpectation {
                    expected_state: case.expected_state.clone(),
                    contradiction_required,
                },
            )
        })
        .collect();
    let allowed_states: BTreeSet<&str> = awareness_manifest
        .allowed_states
        .iter()
        .map(String::as_str)
        .collect();

    if expected_cases.len() != 12 {
        errors.push("source_case_count_mismatch".into());
    }

    if run.workflows.len() < 2 {
        errors.push("at_least_two_workflows_required".into());
    }
    if run.candidate_workflow_ids.is_empty() {
        errors.push("at_least_one_candidate_workflow_required".into());
    }

    let mut workflow_ids = BTreeSet::new();
    for workflow in &run.workflows {
        if workflow.workflow_id.trim().is_empty()
            || !workflow_ids.insert(workflow.workflow_id.as_str())
        {
            errors.push(format!(
                "empty_or_duplicate_workflow_id: {}",
                workflow.workflow_id
            ));
        }
        if workflow.build_ref.trim().is_empty() || !is_sha256(&workflow.configuration_sha256) {
            errors.push(format!(
                "workflow_build_or_configuration_incomplete: {}",
                workflow.workflow_id
            ));
        }
        if workflow.event_frame_sha256 != AWARENESS_CASE_SHA256 {
            errors.push(format!(
                "workflow_event_frame_mismatch: {}",
                workflow.workflow_id
            ));
        }
        if workflow.operator_review_budget_minutes_per_case
            != run.operator_review_budget_minutes_per_case
        {
            errors.push(format!(
                "unmatched_operator_review_budget: {}",
                workflow.workflow_id
            ));
        }
        if workflow.case_results.len() != expected_cases.len() {
            errors.push(format!(
                "workflow_case_count_mismatch: {}",
                workflow.workflow_id
            ));
        }

        let mut seen_cases = BTreeSet::new();
        for result in &workflow.case_results {
            if !seen_cases.insert(result.case_id.as_str()) {
                errors.push(format!(
                    "duplicate_case_result: {} -> {}",
                    workflow.workflow_id, result.case_id
                ));
            }
            if !expected_cases.contains_key(&result.case_id) {
                errors.push(format!(
                    "unexpected_case_result: {} -> {}",
                    workflow.workflow_id, result.case_id
                ));
            }
            if !allowed_states.contains(result.reported_state.as_str()) {
                errors.push(format!(
                    "reported_state_not_in_contract: {} -> {}",
                    workflow.workflow_id, result.case_id
                ));
            }
            if !result.abstained_from_intent_classification {
                errors.push(format!(
                    "intent_abstention_hard_gate: {} -> {}",
                    workflow.workflow_id, result.case_id
                ));
            }
            if !result.privacy_compliant || result.privacy_violation_count != 0 {
                errors.push(format!(
                    "privacy_hard_gate: {} -> {}",
                    workflow.workflow_id, result.case_id
                ));
            }
            if result.authority_violation_count != 0 {
                errors.push(format!(
                    "authority_hard_gate: {} -> {}",
                    workflow.workflow_id, result.case_id
                ));
            }
            if result.vendor_hostile_label_used_as_truth {
                errors.push(format!(
                    "vendor_label_as_truth_hard_gate: {} -> {}",
                    workflow.workflow_id, result.case_id
                ));
            }
        }
        for case_id in expected_cases.keys() {
            if !seen_cases.contains(case_id.as_str()) {
                errors.push(format!(
                    "missing_case_result: {} -> {}",
                    workflow.workflow_id, case_id
                ));
            }
        }
    }

    if !workflow_ids.contains(run.baseline_workflow_id.as_str()) {
        errors.push("baseline_workflow_missing".into());
    }
    if !run
        .candidate_workflow_ids
        .iter()
        .all(|id| workflow_ids.contains(id.as_str()))
    {
        errors.push("candidate_workflow_missing".into());
    }
    if run
        .candidate_workflow_ids
        .iter()
        .any(|id| id == &run.baseline_workflow_id)
    {
        errors.push("baseline_cannot_also_be_candidate".into());
    }
    let unique_candidates: BTreeSet<&str> = run
        .candidate_workflow_ids
        .iter()
        .map(String::as_str)
        .collect();
    if unique_candidates.len() != run.candidate_workflow_ids.len() {
        errors.push("duplicate_candidate_workflow_id".into());
    }
    if let Some(baseline) = run
        .workflows
        .iter()
        .find(|workflow| workflow.workflow_id == run.baseline_workflow_id)
    {
        if !matches!(
            baseline.workflow_kind,
            WorkflowKind::RulesBaseline | WorkflowKind::ManualSiteWorkflow
        ) {
            errors.push("baseline_workflow_kind_invalid".into());
        }
    }
    for candidate_id in &run.candidate_workflow_ids {
        if let Some(candidate) = run
            .workflows
            .iter()
            .find(|workflow| &workflow.workflow_id == candidate_id)
        {
            if matches!(
                candidate.workflow_kind,
                WorkflowKind::RulesBaseline | WorkflowKind::ManualSiteWorkflow
            ) {
                errors.push(format!("candidate_workflow_kind_invalid: {candidate_id}"));
            }
        }
    }

    if !errors.is_empty() {
        errors.sort();
        return rejected_report(run_sha256, comparison_digest, awareness_digest, errors);
    }

    let mut metrics = Vec::new();
    for workflow in &run.workflows {
        if let Some(result) = calculate_metrics(
            workflow,
            &expected_cases,
            run.operator_review_budget_minutes_per_case,
        ) {
            metrics.push(result);
        }
    }

    let metric_index: BTreeMap<&str, &WorkflowMetrics> = metrics
        .iter()
        .map(|metric| (metric.workflow_id.as_str(), metric))
        .collect();

    let baseline = metric_index
        .get(run.baseline_workflow_id.as_str())
        .expect("validated baseline workflow exists");
    let mut pairwise = Vec::new();
    for candidate_id in &run.candidate_workflow_ids {
        let candidate = metric_index
            .get(candidate_id.as_str())
            .expect("validated candidate workflow exists");
        pairwise.push(compare_pair(baseline, candidate));
    }

    let evaluation_status = match run.run_kind {
        RunKind::SyntheticCalculatorSmoke => "SCORED_SYNTHETIC_SMOKE_ONLY",
        RunKind::CandidateComparison => "SCORED_TRACK_A_REPLAY_ONLY",
    };

    EvaluationReport {
        report_schema: REPORT_SCHEMA,
        evaluation_status,
        run_id: Some(run.run_id),
        run_sha256,
        comparison_protocol_sha256: comparison_digest,
        awareness_case_manifest_sha256: awareness_digest,
        case_count: expected_cases.len(),
        workflow_count: metrics.len(),
        workflows: metrics,
        pairwise_comparisons: pairwise,
        errors: Vec::new(),
        claim_ceiling: RUN_CLAIM_CEILING,
    }
}

fn calculate_metrics(
    workflow: &WorkflowRun,
    expectations: &BTreeMap<String, CaseExpectation>,
    review_budget: u32,
) -> Option<WorkflowMetrics> {
    let count = workflow.case_results.len() as u64;
    if count == 0 {
        return None;
    }

    let mut exact_state_matches = 0u64;
    let mut provenance_complete = 0u64;
    let mut source_status_preserved = 0u64;
    let mut contradiction_required = 0u64;
    let mut contradictions_preserved = 0u64;
    let mut intent_abstention = 0u64;
    let mut human_review_required = 0u64;
    let mut event_receipt_time_separation = 0u64;
    let mut correction_lineage = 0u64;
    let mut privacy_compliant = 0u64;
    let mut privacy_violations = 0u64;
    let mut authority_violations = 0u64;
    let mut hostile_label_used = 0u64;
    let mut duplicate_alerts = 0u64;
    let mut unsubstantiated_alerts = 0u64;
    let mut review_minutes = Vec::new();
    let mut budget_exceeded = 0usize;

    for result in &workflow.case_results {
        let expected = expectations.get(&result.case_id)?;
        exact_state_matches += (result.reported_state == expected.expected_state) as u64;
        provenance_complete += result.provenance_complete as u64;
        source_status_preserved += result.source_status_preserved as u64;
        if expected.contradiction_required {
            contradiction_required += 1;
            contradictions_preserved += result.contradiction_preserved as u64;
        }
        intent_abstention += result.abstained_from_intent_classification as u64;
        human_review_required += result.human_review_required as u64;
        event_receipt_time_separation += result.event_and_receipt_times_separated as u64;
        correction_lineage += result.correction_lineage_preserved as u64;
        privacy_compliant += result.privacy_compliant as u64;
        privacy_violations += u64::from(result.privacy_violation_count);
        authority_violations += u64::from(result.authority_violation_count);
        hostile_label_used += u64::from(result.vendor_hostile_label_used_as_truth);
        duplicate_alerts += u64::from(result.duplicate_alert_count);
        unsubstantiated_alerts += u64::from(result.unsubstantiated_alert_count);
        review_minutes.push(result.review_minutes);
        if result.review_minutes > review_budget {
            budget_exceeded += 1;
        }
    }

    review_minutes.sort_unstable();
    let total_review_minutes: u64 = review_minutes.iter().map(|value| u64::from(*value)).sum();
    let mean_milli = total_review_minutes.saturating_mul(1000) / count;
    let median = percentile_nearest_rank(&review_minutes, 50);
    let p95 = percentile_nearest_rank(&review_minutes, 95);

    Some(WorkflowMetrics {
        workflow_id: workflow.workflow_id.clone(),
        workflow_kind: workflow_kind_name(&workflow.workflow_kind).into(),
        case_count: count as usize,
        exact_state_matches: exact_state_matches as usize,
        exact_state_match_rate: rate(exact_state_matches, count),
        provenance_complete_cases: provenance_complete as usize,
        provenance_complete_rate: rate(provenance_complete, count),
        source_status_preserved_cases: source_status_preserved as usize,
        source_status_preserved_rate: rate(source_status_preserved, count),
        contradiction_required_cases: contradiction_required as usize,
        contradictions_preserved_cases: contradictions_preserved as usize,
        contradiction_preservation_rate: rate(
            contradictions_preserved,
            contradiction_required.max(1),
        ),
        intent_abstention_cases: intent_abstention as usize,
        intent_abstention_rate: rate(intent_abstention, count),
        human_review_required_cases: human_review_required as usize,
        human_review_required_rate: rate(human_review_required, count),
        event_receipt_time_separation_cases: event_receipt_time_separation as usize,
        event_receipt_time_separation_rate: rate(event_receipt_time_separation, count),
        correction_lineage_cases: correction_lineage as usize,
        correction_lineage_rate: rate(correction_lineage, count),
        privacy_compliant_cases: privacy_compliant as usize,
        privacy_compliance_rate: rate(privacy_compliant, count),
        privacy_violation_count: privacy_violations,
        authority_violation_count: authority_violations,
        vendor_hostile_label_used_count: hostile_label_used,
        duplicate_alert_count: duplicate_alerts,
        unsubstantiated_alert_count: unsubstantiated_alerts,
        review_minutes_total: total_review_minutes,
        review_minutes_mean_milli: mean_milli,
        review_minutes_median: median,
        review_minutes_p95: p95,
        review_budget_exceeded_cases: budget_exceeded,
    })
}

fn compare_pair(baseline: &WorkflowMetrics, candidate: &WorkflowMetrics) -> PairwiseComparison {
    let review_reduction = reduction_basis_points(
        baseline.review_minutes_total,
        candidate.review_minutes_total,
    );
    let duplicate_reduction = reduction_basis_points(
        baseline.duplicate_alert_count,
        candidate.duplicate_alert_count,
    );
    let unsubstantiated_reduction = reduction_basis_points(
        baseline.unsubstantiated_alert_count,
        candidate.unsubstantiated_alert_count,
    );
    let state_non_degradation = candidate.exact_state_matches >= baseline.exact_state_matches;
    let provenance_non_degradation =
        candidate.provenance_complete_cases >= baseline.provenance_complete_cases;
    let contradiction_non_degradation =
        candidate.contradictions_preserved_cases >= baseline.contradictions_preserved_cases;

    let hypothesis_met = [
        review_reduction,
        duplicate_reduction,
        unsubstantiated_reduction,
    ]
    .into_iter()
    .all(|value| value.is_some_and(|reduction| reduction >= 2000))
        && state_non_degradation
        && provenance_non_degradation
        && contradiction_non_degradation
        && candidate.privacy_violation_count == 0
        && candidate.authority_violation_count == 0
        && candidate.vendor_hostile_label_used_count == 0;

    PairwiseComparison {
        baseline_workflow_id: baseline.workflow_id.clone(),
        candidate_workflow_id: candidate.workflow_id.clone(),
        review_workload_reduction_basis_points: review_reduction,
        duplicate_alert_reduction_basis_points: duplicate_reduction,
        unsubstantiated_alert_reduction_basis_points: unsubstantiated_reduction,
        state_conformance_non_degradation: state_non_degradation,
        provenance_non_degradation,
        contradiction_non_degradation,
        candidate_meets_20_percent_smoke_hypothesis: hypothesis_met,
        interpretation: "Track A evidence-workflow comparison only; not physical sensor recall or field superiority",
    }
}

fn reduction_basis_points(baseline: u64, candidate: u64) -> Option<i64> {
    if baseline == 0 {
        return None;
    }
    let delta = baseline as i128 - candidate as i128;
    Some(((delta * 10_000) / baseline as i128) as i64)
}

fn rate(numerator: u64, denominator: u64) -> Rate {
    let basis_points = if denominator == 0 {
        0
    } else {
        ((numerator.saturating_mul(10_000) + denominator / 2) / denominator).min(10_000) as u32
    };
    Rate {
        numerator,
        denominator,
        basis_points,
    }
}

fn percentile_nearest_rank(values: &[u32], percentile: usize) -> u32 {
    if values.is_empty() {
        return 0;
    }
    let rank = (percentile * values.len()).div_ceil(100).max(1);
    values[(rank - 1).min(values.len() - 1)]
}

fn workflow_kind_name(kind: &WorkflowKind) -> &'static str {
    match kind {
        WorkflowKind::RulesBaseline => "rules_baseline",
        WorkflowKind::ManualSiteWorkflow => "manual_site_workflow",
        WorkflowKind::EvidenceAssistant => "evidence_assistant",
        WorkflowKind::VendorEventAdapter => "vendor_event_adapter",
    }
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn rejected_report(
    run_sha256: String,
    comparison_manifest_sha256: String,
    awareness_case_manifest_sha256: String,
    errors: Vec<String>,
) -> EvaluationReport {
    EvaluationReport {
        report_schema: REPORT_SCHEMA,
        evaluation_status: "REJECTED",
        run_id: None,
        run_sha256,
        comparison_protocol_sha256: comparison_manifest_sha256,
        awareness_case_manifest_sha256,
        case_count: 0,
        workflow_count: 0,
        workflows: Vec::new(),
        pairwise_comparisons: Vec::new(),
        errors,
        claim_ceiling: RUN_CLAIM_CEILING,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const AWARENESS: &str = include_str!(
        "../../../mycelix-workspace/mycelix-praxis/docs/research/GCPN_CIV_AUTONOMOUS_AWARENESS_CASES_V0_1.json"
    );
    const COMPARISON: &str = include_str!(
        "../../../mycelix-workspace/mycelix-praxis/docs/research/GCPN_AWARENESS_INDUSTRY_COMPARISON_CASES_V0_1.json"
    );
    const SMOKE_RUN: &str = include_str!("../fixtures/track_a_smoke_run_v0_1.json");

    #[test]
    fn synthetic_smoke_run_is_scored_but_not_claimed_as_field_result() {
        let report = evaluate(
            AWARENESS.as_bytes(),
            COMPARISON.as_bytes(),
            SMOKE_RUN.as_bytes(),
        );
        assert_eq!(report.evaluation_status, "SCORED_SYNTHETIC_SMOKE_ONLY");
        assert_eq!(report.case_count, 12);
        assert_eq!(report.workflow_count, 2);
        assert_eq!(report.pairwise_comparisons.len(), 1);
        assert_eq!(
            report.pairwise_comparisons[0].review_workload_reduction_basis_points,
            Some(1666)
        );
        assert!(!report.pairwise_comparisons[0].candidate_meets_20_percent_smoke_hypothesis);
        assert!(report.errors.is_empty(), "{:?}", report.errors);
    }

    #[test]
    fn same_frame_and_budget_are_required() {
        let changed = SMOKE_RUN.replacen(
            "\"operator_review_budget_minutes_per_case\": 10",
            "\"operator_review_budget_minutes_per_case\": 11",
            1,
        );
        assert_ne!(changed, SMOKE_RUN);
        let report = evaluate(
            AWARENESS.as_bytes(),
            COMPARISON.as_bytes(),
            changed.as_bytes(),
        );
        assert_eq!(report.evaluation_status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("unmatched_operator_review_budget"))
        );
    }

    #[test]
    fn unknown_run_fields_are_rejected() {
        let changed = SMOKE_RUN.replace(
            "\"run_id\": \"synthetic-track-a-smoke-v0.1\"",
            "\"run_id\": \"synthetic-track-a-smoke-v0.1\", \"sensor_detection_rate\": 0.99",
        );
        assert_ne!(changed, SMOKE_RUN);
        let report = evaluate(
            AWARENESS.as_bytes(),
            COMPARISON.as_bytes(),
            changed.as_bytes(),
        );
        assert_eq!(report.evaluation_status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("replay_run_decode"))
        );
    }

    #[test]
    fn duplicate_json_member_is_rejected() {
        let changed = SMOKE_RUN.replace(
            "\"run_id\": \"synthetic-track-a-smoke-v0.1\"",
            "\"run_id\": \"synthetic-track-a-smoke-v0.1\", \"run_id\": \"other\"",
        );
        assert_ne!(changed, SMOKE_RUN);
        let report = evaluate(
            AWARENESS.as_bytes(),
            COMPARISON.as_bytes(),
            changed.as_bytes(),
        );
        assert_eq!(report.evaluation_status, "REJECTED");
        assert!(report.errors.iter().any(|error| error.starts_with("replay_run_decode")));
    }

    #[test]
    fn hostile_intent_inference_is_a_hard_failure() {
        let changed = SMOKE_RUN.replace(
            "\"abstained_from_intent_classification\": true",
            "\"abstained_from_intent_classification\": false",
        );
        assert_ne!(changed, SMOKE_RUN);
        let report = evaluate(
            AWARENESS.as_bytes(),
            COMPARISON.as_bytes(),
            changed.as_bytes(),
        );
        assert_eq!(report.evaluation_status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("intent_abstention_hard_gate"))
        );
    }

    #[test]
    fn privacy_or_authority_violation_rejects_run() {
        let changed = SMOKE_RUN.replace(
            "\"authority_violation_count\": 0",
            "\"authority_violation_count\": 1",
        );
        assert_ne!(changed, SMOKE_RUN);
        let report = evaluate(
            AWARENESS.as_bytes(),
            COMPARISON.as_bytes(),
            changed.as_bytes(),
        );
        assert_eq!(report.evaluation_status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("authority_hard_gate"))
        );
    }

    #[test]
    fn wrong_event_case_digest_is_rejected() {
        let changed = SMOKE_RUN.replace(
            "317ed98191606c4e0a58f8445ec29df3c9851dfb4968a33faf80dfc43eaacaa4",
            "217ed98191606c4e0a58f8445ec29df3c9851dfb4968a33faf80dfc43eaacaa4",
        );
        assert_ne!(changed, SMOKE_RUN);
        let report = evaluate(
            AWARENESS.as_bytes(),
            COMPARISON.as_bytes(),
            changed.as_bytes(),
        );
        assert_eq!(report.evaluation_status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error == "run_awareness_manifest_digest_mismatch")
        );
    }

    #[test]
    fn missing_case_result_is_rejected() {
        let value: serde_json::Value = serde_json::from_str(SMOKE_RUN).expect("fixture parses");
        let mut root = value;
        let workflows = root["workflows"].as_array_mut().expect("workflows array");
        let results = workflows[0]["case_results"]
            .as_array_mut()
            .expect("case results");
        results.pop();
        let changed = serde_json::to_vec(&root).expect("serialize mutation");
        let report = evaluate(AWARENESS.as_bytes(), COMPARISON.as_bytes(), &changed);
        assert_eq!(report.evaluation_status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("workflow_case_count_mismatch"))
        );
    }

    #[test]
    fn baseline_zero_alerts_has_no_division_or_fake_reduction() {
        assert_eq!(reduction_basis_points(0, 0), None);
        assert_eq!(reduction_basis_points(0, 2), None);
        assert_eq!(reduction_basis_points(10, 8), Some(2000));
        assert_eq!(reduction_basis_points(10, 12), Some(-2000));
    }

    #[test]
    fn percentile_uses_nearest_rank() {
        assert_eq!(percentile_nearest_rank(&[1, 2, 3, 4, 5], 50), 3);
        assert_eq!(percentile_nearest_rank(&[1, 2, 3, 4, 5], 95), 5);
    }
}

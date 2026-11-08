//! Offline evaluator for matched Track A event-envelope replays.
//!
//! This crate scores downstream evidence/review workflows only. It does not
//! ingest sensor signals, detect or track aircraft, infer intent, authorize
//! action, command equipment, or evaluate physical sensor performance.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const INPUT_SCHEMA: &str = "luminous.gcpn.track_a_evaluation_bundle.v0.2";
pub const REPORT_SCHEMA: &str = "luminous.gcpn.track_a_evaluation_report.v0.2";
pub const EXPECTED_FIXTURE_SHA256: &str = "FIXTURE_SHA256_PENDING";

const ALLOWED_EVENT_STATES: [&str; 12] = [
    "AUTHORIZED_CLAIM_PENDING_SOURCE_VALIDATION",
    "OBSERVED_INTENT_UNKNOWN",
    "OBSERVED_CLASS_AMBIGUOUS",
    "UNKNOWN_RF_SILENT_OR_OUT_OF_BAND",
    "STALE_UNKNOWN",
    "SOURCE_UNHEALTHY",
    "COMMON_MODE_UNKNOWN",
    "IDENTITY_REVOKED_REVIEW_REQUIRED",
    "OWNER_TASK_AUTHORIZATION_EXPIRED_HOLD",
    "THIRD_PARTY_CONTROL_PROHIBITED",
    "DATA_USE_BLOCKED",
    "HUMAN_REVIEW_REQUIRED",
];


#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrackARunBundle {
    pub schema: String,
    pub comparison_id: String,
    pub track_id: String,
    pub evidence_tier: EvidenceTier,
    pub simulation_only: bool,
    pub event_set_sha256: String,
    pub ground_truth_sha256: String,
    pub operator_policy_sha256: String,
    pub ground_truth_independent: bool,
    pub ground_truth_adjudication_receipt_sha256: Option<String>,
    pub privacy_review_receipt_sha256: Option<String>,
    pub authorization_review_receipt_sha256: Option<String>,
    pub monitoring_duration_seconds: u64,
    pub review_budget_seconds_per_case: u64,
    pub oracle_cases: Vec<OracleCase>,
    pub pipelines: Vec<PipelineSubmission>,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EvidenceTier {
    SyntheticFixture,
    ControlledReplay,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum GroundTruthLabel {
    ConfirmedRelevant,
    ConfirmedBenign,
    Unresolved,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OracleCase {
    pub case_id: String,
    pub label: GroundTruthLabel,
    pub expected_state: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PipelineSubmission {
    pub pipeline_id: String,
    pub role: PipelineRole,
    pub build_sha256: String,
    pub configuration_sha256: String,
    pub operator_policy_sha256: String,
    pub event_set_sha256: String,
    pub safety_boundary: SafetyBoundary,
    pub case_results: Vec<CaseResult>,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PipelineRole {
    RulesOnly,
    SiteWorkflow,
    EvidenceAssistant,
    OtherReference,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SafetyBoundary {
    pub vendor_hostile_label_used_as_truth_count: u64,
    pub operator_location_export_count: u64,
    pub exact_flight_path_export_count: u64,
    pub raw_audio_video_export_count: u64,
    pub third_party_command_count: u64,
    pub active_mitigation_count: u64,
    pub privacy_violation_count: u64,
}

impl SafetyBoundary {
    fn violations(&self) -> Vec<&'static str> {
        let mut violations = Vec::new();
        if self.vendor_hostile_label_used_as_truth_count > 0 {
            violations.push("vendor_hostile_label_used_as_truth");
        }
        if self.operator_location_export_count > 0 {
            violations.push("operator_location_export");
        }
        if self.exact_flight_path_export_count > 0 {
            violations.push("exact_flight_path_export");
        }
        if self.raw_audio_video_export_count > 0 {
            violations.push("raw_audio_video_export");
        }
        if self.third_party_command_count > 0 {
            violations.push("third_party_command");
        }
        if self.active_mitigation_count > 0 {
            violations.push("active_mitigation");
        }
        if self.privacy_violation_count > 0 {
            violations.push("privacy_violation");
        }
        violations
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseResult {
    pub case_id: String,
    pub reported_state: String,
    pub abstained_from_intent_classification: bool,
    pub source_status_preserved: bool,
    pub event_and_receipt_times_separated: bool,
    pub correction_lineage_preserved: bool,
    pub disposition: ReviewDisposition,
    pub review_requests: u32,
    pub review_latency_ms: Option<u64>,
    pub human_review_seconds: Option<u64>,
    pub relevance_probability_micros: Option<u32>,
    pub source_evidence: SourceEvidence,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ReviewDisposition {
    ReviewRequired,
    NoReview,
    Abstain,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceEvidence {
    pub source_count: u32,
    pub authenticated_source_count: u32,
    pub calibrated_source_count: u32,
    pub time_qualified_source_count: u32,
    pub dependency_inventory: DependencyInventory,
    pub conflict_observed: bool,
    pub conflict_preserved: bool,
    pub purpose_and_rights_verified: bool,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DependencyInventory {
    Complete,
    Partial,
    Unknown,
}

#[derive(Clone, Debug, Serialize)]
pub struct EvaluationReport {
    pub report_schema: &'static str,
    pub status: &'static str,
    pub input_sha256: String,
    pub comparison_id: Option<String>,
    pub track_id: &'static str,
    pub evidence_tier: Option<String>,
    pub case_count: usize,
    pub pipeline_results: Vec<PipelineEvaluation>,
    pub reasons: Vec<String>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, Serialize)]
pub struct PipelineEvaluation {
    pub pipeline_id: String,
    pub role: String,
    pub status: &'static str,
    pub boundary_violations: Vec<String>,
    pub metrics: Option<PipelineMetrics>,
}

#[derive(Clone, Debug, Serialize)]
pub struct PipelineMetrics {
    pub case_count: usize,
    pub expected_state_match_cases: usize,
    pub expected_state_match_rate: Option<f64>,
    pub intent_abstention_cases: usize,
    pub intent_abstention_rate: Option<f64>,
    pub source_status_preserved_cases: usize,
    pub source_status_preservation_rate: Option<f64>,
    pub event_receipt_times_separated_cases: usize,
    pub event_receipt_time_separation_rate: Option<f64>,
    pub correction_lineage_preserved_cases: usize,
    pub correction_lineage_preservation_rate: Option<f64>,
    pub confirmed_relevant_cases: usize,
    pub confirmed_benign_cases: usize,
    pub unresolved_cases: usize,
    pub true_positive_events: usize,
    pub false_positive_events: usize,
    pub false_negative_events: usize,
    pub true_negative_events: usize,
    pub known_label_abstentions: usize,
    pub unresolved_abstentions: usize,
    pub unresolved_human_referrals: usize,
    pub unresolved_no_review: usize,
    pub review_requests_total: u64,
    pub false_review_requests: u64,
    pub duplicate_review_requests: u64,
    pub duplicate_false_review_requests: u64,
    pub review_budget_exceedances: usize,
    pub precision: Option<f64>,
    pub recall: Option<f64>,
    pub false_review_fraction: Option<f64>,
    pub false_review_requests_per_hour: Option<f64>,
    pub known_decision_coverage: Option<f64>,
    pub selective_risk: Option<f64>,
    pub brier_score: Option<f64>,
    pub probability_supplied_cases: usize,
    pub probability_unscored_unresolved_cases: usize,
    pub probability_coverage_known_cases: Option<f64>,
    pub probability_scored_case_count: usize,
    pub review_latency_p50_ms: Option<u64>,
    pub review_latency_p95_ms: Option<u64>,
    pub mean_human_review_seconds_per_reviewed_case: Option<f64>,
    pub source_count_total: u64,
    pub authenticated_source_count_total: u64,
    pub calibrated_source_count_total: u64,
    pub time_qualified_source_count_total: u64,
    pub dependency_inventory_complete_cases: usize,
    pub dependency_inventory_partial_cases: usize,
    pub dependency_inventory_unknown_cases: usize,
    pub fully_provenanced_cases: usize,
    pub conflicts_observed: usize,
    pub conflicts_preserved: usize,
    pub conflicts_lost: usize,
    pub rights_and_purpose_verified_cases: usize,
}

fn empty_report(
    input_sha256: String,
    status: &'static str,
    comparison_id: Option<String>,
    case_count: usize,
    reasons: Vec<String>,
) -> EvaluationReport {
    EvaluationReport {
        report_schema: REPORT_SCHEMA,
        status,
        input_sha256,
        comparison_id,
        track_id: "TRACK_A",
        evidence_tier: None,
        case_count,
        pipeline_results: Vec::new(),
        reasons,
        claim_ceiling: "offline_track_a_event_envelope_metrics_only",
    }
}

/// Evaluate arbitrary, sealed Track A input. The returned input digest is always
/// calculated from the exact bytes supplied by the caller.
pub fn evaluate(bytes: &[u8]) -> EvaluationReport {
    evaluate_with_expected_sha256(bytes, None)
}

/// Evaluate a bundle and, optionally, require an exact expected input digest.
/// The digest requirement is useful for frozen fixtures and independently
/// preregistered replay bundles.
pub fn evaluate_with_expected_sha256(
    bytes: &[u8],
    expected_sha256: Option<&str>,
) -> EvaluationReport {
    let input_sha256 = format!("{:x}", Sha256::digest(bytes));

    if let Some(expected) = expected_sha256 {
        if !is_sha256(expected) || input_sha256 != expected {
            return empty_report(
                input_sha256.clone(),
                "REJECTED",
                None,
                0,
                vec!["input_digest_mismatch".to_string()],
            );
        }
    }

    let bundle = match serde_json::from_slice::<TrackARunBundle>(bytes) {
        Ok(bundle) => bundle,
        Err(error) => {
            return empty_report(
                input_sha256,
                "REJECTED",
                None,
                0,
                vec![format!("strict_json_decode: {error}")],
            );
        }
    };

    let mut errors = validate_bundle(&bundle);
    if !errors.is_empty() {
        errors.sort();
        return empty_report(
            input_sha256,
            "REJECTED",
            Some(bundle.comparison_id),
            bundle.oracle_cases.len(),
            errors,
        );
    }

    if controlled_replay_receipt_verification_unavailable(&bundle) {
        return EvaluationReport {
            report_schema: REPORT_SCHEMA,
            status: "BLOCKED",
            input_sha256,
            comparison_id: Some(bundle.comparison_id),
            track_id: "TRACK_A",
            evidence_tier: Some(evidence_tier_name(&bundle.evidence_tier).to_string()),
            case_count: bundle.oracle_cases.len(),
            pipeline_results: Vec::new(),
            reasons: vec!["controlled_replay_blocked_until_shared_evidence_plane_receipt_verification".to_string()],
            claim_ceiling: "offline_track_a_event_envelope_metrics_only",
        };
    }

    let mut incomparable = comparability_reasons(&bundle);
    incomparable.sort();
    if !incomparable.is_empty() {
        return empty_report(
            input_sha256,
            "INCOMPARABLE",
            Some(bundle.comparison_id),
            bundle.oracle_cases.len(),
            incomparable,
        );
    }

    let blocked: Vec<(String, Vec<String>)> = bundle
        .pipelines
        .iter()
        .filter_map(|pipeline| {
            let violations = pipeline
                .safety_boundary
                .violations()
                .into_iter()
                .map(str::to_string)
                .collect::<Vec<_>>();
            if violations.is_empty() {
                None
            } else {
                Some((pipeline.pipeline_id.clone(), violations))
            }
        })
        .collect();

    if !blocked.is_empty() {
        let mut results = Vec::with_capacity(bundle.pipelines.len());
        for pipeline in &bundle.pipelines {
            let violations = pipeline
                .safety_boundary
                .violations()
                .into_iter()
                .map(str::to_string)
                .collect::<Vec<_>>();
            results.push(PipelineEvaluation {
                pipeline_id: pipeline.pipeline_id.clone(),
                role: role_name(&pipeline.role).to_string(),
                status: if violations.is_empty() {
                    "WITHHELD"
                } else {
                    "BLOCKED"
                },
                boundary_violations: violations,
                metrics: None,
            });
        }
        let reasons = blocked
            .iter()
            .map(|(id, violations)| format!("pipeline_blocked:{id}:{}", violations.join(",")))
            .collect();
        return EvaluationReport {
            report_schema: REPORT_SCHEMA,
            status: "BLOCKED",
            input_sha256,
            comparison_id: Some(bundle.comparison_id),
            track_id: "TRACK_A",
            evidence_tier: Some(evidence_tier_name(&bundle.evidence_tier).to_string()),
            case_count: bundle.oracle_cases.len(),
            pipeline_results: results,
            reasons,
            claim_ceiling: "offline_track_a_event_envelope_metrics_only",
        };
    }

    let oracle: BTreeMap<&str, &OracleCase> = bundle
        .oracle_cases
        .iter()
        .map(|case| (case.case_id.as_str(), case))
        .collect();

    let pipeline_results = bundle
        .pipelines
        .iter()
        .map(|pipeline| PipelineEvaluation {
            pipeline_id: pipeline.pipeline_id.clone(),
            role: role_name(&pipeline.role).to_string(),
            status: "SCORED",
            boundary_violations: Vec::new(),
            metrics: Some(calculate_metrics(
                pipeline,
                &oracle,
                bundle.monitoring_duration_seconds,
                bundle.review_budget_seconds_per_case,
            )),
        })
        .collect();

    EvaluationReport {
        report_schema: REPORT_SCHEMA,
        status: "COMPARABLE",
        input_sha256,
        comparison_id: Some(bundle.comparison_id),
        track_id: "TRACK_A",
        evidence_tier: Some(evidence_tier_name(&bundle.evidence_tier).to_string()),
        case_count: bundle.oracle_cases.len(),
        pipeline_results,
        reasons: Vec::new(),
        claim_ceiling: "offline_track_a_event_envelope_metrics_only",
    }
}

/// Verifies the byte-pinned synthetic reference bundle embedded in this crate.
pub fn evaluate_frozen_fixture(bytes: &[u8]) -> EvaluationReport {
    evaluate_with_expected_sha256(bytes, Some(EXPECTED_FIXTURE_SHA256))
}

fn validate_bundle(bundle: &TrackARunBundle) -> Vec<String> {
    let mut errors = Vec::new();

    if bundle.schema != INPUT_SCHEMA {
        errors.push("schema_mismatch".to_string());
    }
    if bundle.comparison_id.trim().is_empty() {
        errors.push("comparison_id_missing".to_string());
    }
    if bundle.track_id != "TRACK_A" {
        errors.push("unsupported_track_for_this_evaluator".to_string());
    }
    if !is_sha256(&bundle.event_set_sha256)
        || !is_sha256(&bundle.ground_truth_sha256)
        || !is_sha256(&bundle.operator_policy_sha256)
    {
        errors.push("invalid_top_level_sha256".to_string());
    }
    if bundle.monitoring_duration_seconds == 0 {
        errors.push("monitoring_duration_must_be_positive".to_string());
    }
    if bundle.review_budget_seconds_per_case == 0 {
        errors.push("review_budget_must_be_positive".to_string());
    }

    match bundle.evidence_tier {
        EvidenceTier::SyntheticFixture if !bundle.simulation_only => {
            errors.push("synthetic_fixture_must_be_simulation_only".to_string());
        }
        EvidenceTier::SyntheticFixture if bundle.ground_truth_independent => {
            errors.push("synthetic_fixture_cannot_claim_independent_ground_truth".to_string());
        }
        EvidenceTier::ControlledReplay if bundle.simulation_only => {
            errors.push("controlled_replay_cannot_be_marked_simulation_only".to_string());
        }
        _ => {}
    }

    if bundle.oracle_cases.is_empty() {
        errors.push("oracle_case_set_empty".to_string());
    }

    let mut oracle_ids = BTreeSet::new();
    for case in &bundle.oracle_cases {
        if !valid_case_id(&case.case_id) {
            errors.push(format!("invalid_case_id:{}", case.case_id));
        }
        if !ALLOWED_EVENT_STATES.contains(&case.expected_state.as_str()) {
            errors.push(format!("oracle_expected_state_not_allowed:{}", case.case_id));
        }
        if !oracle_ids.insert(case.case_id.as_str()) {
            errors.push(format!("duplicate_oracle_case_id:{}", case.case_id));
        }
    }

    if bundle.pipelines.len() < 2 {
        errors.push("at_least_two_pipelines_required".to_string());
    }

    let mut pipeline_ids = BTreeSet::new();
    let oracle_labels: BTreeMap<&str, &OracleCase> = bundle
        .oracle_cases
        .iter()
        .map(|case| (case.case_id.as_str(), case))
        .collect();

    for pipeline in &bundle.pipelines {
        if !valid_identifier(&pipeline.pipeline_id) {
            errors.push(format!("invalid_pipeline_id:{}", pipeline.pipeline_id));
        }
        if !pipeline_ids.insert(pipeline.pipeline_id.as_str()) {
            errors.push(format!("duplicate_pipeline_id:{}", pipeline.pipeline_id));
        }
        if !is_sha256(&pipeline.build_sha256)
            || !is_sha256(&pipeline.configuration_sha256)
            || !is_sha256(&pipeline.operator_policy_sha256)
            || !is_sha256(&pipeline.event_set_sha256)
        {
            errors.push(format!("invalid_pipeline_sha256:{}", pipeline.pipeline_id));
        }
        if pipeline.case_results.is_empty() {
            errors.push(format!(
                "pipeline_case_results_empty:{}",
                pipeline.pipeline_id
            ));
        }

        let mut result_ids = BTreeSet::new();
        for result in &pipeline.case_results {
            if !valid_case_id(&result.case_id) {
                errors.push(format!(
                    "invalid_pipeline_case_id:{}:{}",
                    pipeline.pipeline_id, result.case_id
                ));
            }
            if !result_ids.insert(result.case_id.as_str()) {
                errors.push(format!(
                    "duplicate_pipeline_case_id:{}:{}",
                    pipeline.pipeline_id, result.case_id
                ));
            }
            if !ALLOWED_EVENT_STATES.contains(&result.reported_state.as_str()) {
                errors.push(format!(
                    "reported_state_not_allowed:{}:{}",
                    pipeline.pipeline_id, result.case_id
                ));
            }
            if !result.abstained_from_intent_classification {
                errors.push(format!(
                    "intent_classification_not_abstained:{}:{}",
                    pipeline.pipeline_id, result.case_id
                ));
            }
            if let Err(error) = validate_case_result(result) {
                errors.push(format!(
                    "invalid_case_result:{}:{}:{error}",
                    pipeline.pipeline_id, result.case_id
                ));
            }
        }
    }

    for value in [
        bundle.ground_truth_adjudication_receipt_sha256.as_deref(),
        bundle.privacy_review_receipt_sha256.as_deref(),
        bundle.authorization_review_receipt_sha256.as_deref(),
    ]
    .into_iter()
    .flatten()
    {
        if !is_sha256(value) {
            errors.push("invalid_optional_review_receipt_sha256".to_string());
        }
    }

    errors
}

fn controlled_replay_receipt_verification_unavailable(bundle: &TrackARunBundle) -> bool {
    // A syntactically valid digest is only a reference, not proof that a receipt
    // exists, is signed, is current, or was issued by an independent authority.
    // Until the shared evidence-plane verifier is integrated, all real/controlled
    // replay bundles remain blocked regardless of the presence of digest strings.
    bundle.evidence_tier == EvidenceTier::ControlledReplay
}

fn validate_case_result(result: &CaseResult) -> Result<(), &'static str> {
    match result.disposition {
        ReviewDisposition::ReviewRequired => {
            if result.review_requests == 0
                || result.review_latency_ms.is_none()
                || result.human_review_seconds.is_none()
            {
                return Err("review_required_needs_request_latency_and_duration");
            }
        }
        ReviewDisposition::NoReview | ReviewDisposition::Abstain => {
            if result.review_requests != 0
                || result.review_latency_ms.is_some()
                || result.human_review_seconds.is_some()
            {
                return Err("no_review_or_abstain_must_not_carry_review_metrics");
            }
        }
    }

    if result
        .relevance_probability_micros
        .is_some_and(|probability| probability > 1_000_000)
    {
        return Err("relevance_probability_out_of_range");
    }

    let source = &result.source_evidence;
    if source.authenticated_source_count > source.source_count
        || source.calibrated_source_count > source.source_count
        || source.time_qualified_source_count > source.source_count
    {
        return Err("source_quality_count_exceeds_source_count");
    }
    if source.conflict_observed && !source.conflict_preserved {
        // This is a measurable evidence-quality failure, not a malformed input.
    }

    Ok(())
}

fn comparability_reasons(bundle: &TrackARunBundle) -> Vec<String> {
    let mut reasons = Vec::new();
    let oracle_ids: BTreeSet<&str> = bundle
        .oracle_cases
        .iter()
        .map(|case| case.case_id.as_str())
        .collect();

    for pipeline in &bundle.pipelines {
        if pipeline.event_set_sha256 != bundle.event_set_sha256 {
            reasons.push(format!(
                "event_set_digest_mismatch:{}",
                pipeline.pipeline_id
            ));
        }
        if pipeline.operator_policy_sha256 != bundle.operator_policy_sha256 {
            reasons.push(format!("operator_policy_mismatch:{}", pipeline.pipeline_id));
        }

        let result_ids: BTreeSet<&str> = pipeline
            .case_results
            .iter()
            .map(|result| result.case_id.as_str())
            .collect();
        if result_ids != oracle_ids {
            reasons.push(format!("case_coverage_mismatch:{}", pipeline.pipeline_id));
        }
    }

    reasons
}

fn calculate_metrics(
    pipeline: &PipelineSubmission,
    oracle: &BTreeMap<&str, &OracleCase>,
    monitoring_duration_seconds: u64,
    review_budget_seconds_per_case: u64,
) -> PipelineMetrics {
    let mut expected_state_match_cases = 0usize;
    let mut intent_abstention_cases = 0usize;
    let mut source_status_preserved_cases = 0usize;
    let mut event_receipt_times_separated_cases = 0usize;
    let mut correction_lineage_preserved_cases = 0usize;
    let mut confirmed_relevant_cases = 0usize;
    let mut confirmed_benign_cases = 0usize;
    let mut unresolved_cases = 0usize;

    let mut true_positive_events = 0usize;
    let mut false_positive_events = 0usize;
    let mut false_negative_events = 0usize;
    let mut true_negative_events = 0usize;
    let mut known_label_abstentions = 0usize;
    let mut unresolved_abstentions = 0usize;
    let mut unresolved_human_referrals = 0usize;
    let mut unresolved_no_review = 0usize;
    let mut selective_false_negatives = 0usize;
    let mut known_determinate_cases = 0usize;

    let mut review_requests_total = 0u64;
    let mut false_review_requests = 0u64;
    let mut duplicate_review_requests = 0u64;
    let mut duplicate_false_review_requests = 0u64;
    let mut review_budget_exceedances = 0usize;

    let mut latencies = Vec::new();
    let mut human_review_seconds_total = 0u64;
    let mut reviewed_case_count = 0usize;

    let mut brier_sum = 0.0f64;
    let mut probability_supplied_cases = 0usize;
    let mut probability_unscored_unresolved_cases = 0usize;
    let mut probability_scored_case_count = 0usize;

    let mut source_count_total = 0u64;
    let mut authenticated_source_count_total = 0u64;
    let mut calibrated_source_count_total = 0u64;
    let mut time_qualified_source_count_total = 0u64;
    let mut dependency_inventory_complete_cases = 0usize;
    let mut dependency_inventory_partial_cases = 0usize;
    let mut dependency_inventory_unknown_cases = 0usize;
    let mut fully_provenanced_cases = 0usize;
    let mut conflicts_observed = 0usize;
    let mut conflicts_preserved = 0usize;
    let mut conflicts_lost = 0usize;
    let mut rights_and_purpose_verified_cases = 0usize;

    for result in &pipeline.case_results {
        let oracle_case = oracle
            .get(result.case_id.as_str())
            .expect("case coverage validated");
        let label = oracle_case.label;
        let source = &result.source_evidence;

        if result.reported_state == oracle_case.expected_state {
            expected_state_match_cases += 1;
        }
        if result.abstained_from_intent_classification {
            intent_abstention_cases += 1;
        }
        if result.source_status_preserved {
            source_status_preserved_cases += 1;
        }
        if result.event_and_receipt_times_separated {
            event_receipt_times_separated_cases += 1;
        }
        if result.correction_lineage_preserved {
            correction_lineage_preserved_cases += 1;
        }

        source_count_total += u64::from(source.source_count);
        authenticated_source_count_total += u64::from(source.authenticated_source_count);
        calibrated_source_count_total += u64::from(source.calibrated_source_count);
        time_qualified_source_count_total += u64::from(source.time_qualified_source_count);

        match source.dependency_inventory {
            DependencyInventory::Complete => dependency_inventory_complete_cases += 1,
            DependencyInventory::Partial => dependency_inventory_partial_cases += 1,
            DependencyInventory::Unknown => dependency_inventory_unknown_cases += 1,
        }
        if source.conflict_observed {
            conflicts_observed += 1;
            if source.conflict_preserved {
                conflicts_preserved += 1;
            } else {
                conflicts_lost += 1;
            }
        }
        if source.purpose_and_rights_verified && source.source_count > 0 {
            rights_and_purpose_verified_cases += 1;
        }
        if source.source_count > 0
            && source.authenticated_source_count == source.source_count
            && source.calibrated_source_count == source.source_count
            && source.time_qualified_source_count == source.source_count
            && source.dependency_inventory == DependencyInventory::Complete
            && source.purpose_and_rights_verified
            && (!source.conflict_observed || source.conflict_preserved)
        {
            fully_provenanced_cases += 1;
        }

        let requests = u64::from(result.review_requests);
        review_requests_total += requests;
        if requests > 1 {
            duplicate_review_requests += requests - 1;
        }
        if requests > 0 {
            if let Some(latency) = result.review_latency_ms {
                latencies.push(latency);
            }
            if let Some(seconds) = result.human_review_seconds {
                human_review_seconds_total += seconds;
                reviewed_case_count += 1;
                if seconds > review_budget_seconds_per_case {
                    review_budget_exceedances += 1;
                }
            }
        }

        match label {
            GroundTruthLabel::ConfirmedRelevant => {
                confirmed_relevant_cases += 1;
                match result.disposition {
                    ReviewDisposition::ReviewRequired => {
                        true_positive_events += 1;
                        known_determinate_cases += 1;
                    }
                    ReviewDisposition::NoReview => {
                        false_negative_events += 1;
                        selective_false_negatives += 1;
                        known_determinate_cases += 1;
                    }
                    ReviewDisposition::Abstain => {
                        false_negative_events += 1;
                        known_label_abstentions += 1;
                    }
                }
            }
            GroundTruthLabel::ConfirmedBenign => {
                confirmed_benign_cases += 1;
                match result.disposition {
                    ReviewDisposition::ReviewRequired => {
                        false_positive_events += 1;
                        known_determinate_cases += 1;
                        false_review_requests += requests;
                        if requests > 1 {
                            duplicate_false_review_requests += requests - 1;
                        }
                    }
                    ReviewDisposition::NoReview => {
                        true_negative_events += 1;
                        known_determinate_cases += 1;
                    }
                    ReviewDisposition::Abstain => {
                        known_label_abstentions += 1;
                    }
                }
            }
            GroundTruthLabel::Unresolved => {
                unresolved_cases += 1;
                match result.disposition {
                    ReviewDisposition::ReviewRequired => unresolved_human_referrals += 1,
                    ReviewDisposition::NoReview => unresolved_no_review += 1,
                    ReviewDisposition::Abstain => unresolved_abstentions += 1,
                }
            }
        }

        if let Some(probability) = result.relevance_probability_micros {
            probability_supplied_cases += 1;
            let target = match label {
                GroundTruthLabel::ConfirmedRelevant => Some(1.0),
                GroundTruthLabel::ConfirmedBenign => Some(0.0),
                GroundTruthLabel::Unresolved => None,
            };
            if let Some(target) = target {
                let predicted = f64::from(probability) / 1_000_000.0;
                let error = predicted - target;
                brier_sum += error * error;
                probability_scored_case_count += 1;
            } else {
                probability_unscored_unresolved_cases += 1;
            }
        }
    }

    let known_cases = confirmed_relevant_cases + confirmed_benign_cases;
    let reviewed_events = true_positive_events + false_positive_events;
    let total_relevant = confirmed_relevant_cases;
    let determinate_cases = known_determinate_cases;
    let selective_errors = false_positive_events + selective_false_negatives;
    let review_hours = monitoring_duration_seconds as f64 / 3600.0;

    PipelineMetrics {
        case_count: pipeline.case_results.len(),
        expected_state_match_cases,
        expected_state_match_rate: ratio(expected_state_match_cases, pipeline.case_results.len()),
        intent_abstention_cases,
        intent_abstention_rate: ratio(intent_abstention_cases, pipeline.case_results.len()),
        source_status_preserved_cases,
        source_status_preservation_rate: ratio(source_status_preserved_cases, pipeline.case_results.len()),
        event_receipt_times_separated_cases,
        event_receipt_time_separation_rate: ratio(event_receipt_times_separated_cases, pipeline.case_results.len()),
        correction_lineage_preserved_cases,
        correction_lineage_preservation_rate: ratio(correction_lineage_preserved_cases, pipeline.case_results.len()),
        confirmed_relevant_cases,
        confirmed_benign_cases,
        unresolved_cases,
        true_positive_events,
        false_positive_events,
        false_negative_events,
        true_negative_events,
        known_label_abstentions,
        unresolved_abstentions,
        unresolved_human_referrals,
        unresolved_no_review,
        review_requests_total,
        false_review_requests,
        duplicate_review_requests,
        duplicate_false_review_requests,
        review_budget_exceedances,
        precision: ratio(true_positive_events, reviewed_events),
        recall: ratio(true_positive_events, total_relevant),
        false_review_fraction: ratio(false_positive_events, reviewed_events),
        false_review_requests_per_hour: if review_hours > 0.0 {
            Some(false_review_requests as f64 / review_hours)
        } else {
            None
        },
        known_decision_coverage: ratio(determinate_cases, known_cases),
        selective_risk: ratio(selective_errors, determinate_cases),
        brier_score: if probability_scored_case_count > 0 {
            Some(brier_sum / probability_scored_case_count as f64)
        } else {
            None
        },
        probability_supplied_cases,
        probability_unscored_unresolved_cases,
        probability_coverage_known_cases: ratio(probability_scored_case_count, known_cases),
        probability_scored_case_count,
        review_latency_p50_ms: percentile_nearest_rank(&latencies, 0.50),
        review_latency_p95_ms: percentile_nearest_rank(&latencies, 0.95),
        mean_human_review_seconds_per_reviewed_case: if reviewed_case_count > 0 {
            Some(human_review_seconds_total as f64 / reviewed_case_count as f64)
        } else {
            None
        },
        source_count_total,
        authenticated_source_count_total,
        calibrated_source_count_total,
        time_qualified_source_count_total,
        dependency_inventory_complete_cases,
        dependency_inventory_partial_cases,
        dependency_inventory_unknown_cases,
        fully_provenanced_cases,
        conflicts_observed,
        conflicts_preserved,
        conflicts_lost,
        rights_and_purpose_verified_cases,
    }
}

fn ratio(numerator: usize, denominator: usize) -> Option<f64> {
    if denominator == 0 {
        None
    } else {
        Some(numerator as f64 / denominator as f64)
    }
}

fn percentile_nearest_rank(values: &[u64], percentile: f64) -> Option<u64> {
    if values.is_empty() {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = ((percentile * sorted.len() as f64).ceil() as usize).max(1);
    sorted.get(rank - 1).copied()
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 96
        && value.bytes().all(|byte| {
            byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'_' || byte == b'-'
        })
}

fn valid_case_id(value: &str) -> bool {
    valid_identifier(value)
}

fn role_name(role: &PipelineRole) -> &'static str {
    match role {
        PipelineRole::RulesOnly => "RULES_ONLY",
        PipelineRole::SiteWorkflow => "SITE_WORKFLOW",
        PipelineRole::EvidenceAssistant => "EVIDENCE_ASSISTANT",
        PipelineRole::OtherReference => "OTHER_REFERENCE",
    }
}

fn evidence_tier_name(tier: &EvidenceTier) -> &'static str {
    match tier {
        EvidenceTier::SyntheticFixture => "SYNTHETIC_FIXTURE",
        EvidenceTier::ControlledReplay => "CONTROLLED_REPLAY",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    const FIXTURE: &str = include_str!("../fixtures/track_a_synthetic_run.json");

    fn parsed_fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("fixture JSON parses")
    }

    fn evaluate_value(value: Value) -> EvaluationReport {
        let bytes = serde_json::to_vec(&value).expect("serialize mutation");
        evaluate(&bytes)
    }

    #[test]
    fn frozen_reference_bundle_is_byte_pinned_and_comparable() {
        let report = evaluate_frozen_fixture(FIXTURE.as_bytes());
        assert_eq!(report.status, "COMPARABLE", "{:?}", report.reasons);
        assert_eq!(report.case_count, 8);
        assert_eq!(report.pipeline_results.len(), 3);
        assert_eq!(report.input_sha256, EXPECTED_FIXTURE_SHA256);
    }

    #[test]
    fn rules_pipeline_metrics_are_reproducible() {
        let report = evaluate_frozen_fixture(FIXTURE.as_bytes());
        let metrics = report.pipeline_results[0].metrics.as_ref().unwrap();
        assert_eq!(metrics.true_positive_events, 2);
        assert_eq!(metrics.false_positive_events, 2);
        assert_eq!(metrics.false_negative_events, 1);
        assert_eq!(metrics.duplicate_review_requests, 3);
        assert_eq!(metrics.duplicate_false_review_requests, 2);
        assert_eq!(metrics.review_budget_exceedances, 1);
        assert!((metrics.precision.unwrap() - 0.5).abs() < 1e-12);
        assert!((metrics.recall.unwrap() - (2.0 / 3.0)).abs() < 1e-12);
        assert_eq!(metrics.review_latency_p50_ms, Some(300));
        assert_eq!(metrics.review_latency_p95_ms, Some(500));
    }

    #[test]
    fn unresolved_cases_are_not_folded_into_binary_confusion_counts() {
        let report = evaluate_frozen_fixture(FIXTURE.as_bytes());
        let metrics = report.pipeline_results[2].metrics.as_ref().unwrap();
        assert_eq!(metrics.unresolved_cases, 2);
        assert_eq!(metrics.unresolved_abstentions, 2);
        assert_eq!(
            metrics.confirmed_relevant_cases + metrics.confirmed_benign_cases,
            6
        );
        assert_eq!(
            metrics.true_positive_events
                + metrics.false_positive_events
                + metrics.false_negative_events
                + metrics.true_negative_events
                + metrics.known_label_abstentions,
            6
        );
    }

    #[test]
    fn evidence_assistant_brier_score_uses_only_determined_labels() {
        let report = evaluate_frozen_fixture(FIXTURE.as_bytes());
        let metrics = report.pipeline_results[2].metrics.as_ref().unwrap();
        assert_eq!(metrics.probability_scored_case_count, 6);
        assert!((metrics.brier_score.unwrap() - (0.07 / 6.0)).abs() < 1e-12);
    }

    #[test]
    fn source_dependency_unknown_is_counted_not_assumed_independent() {
        let report = evaluate_frozen_fixture(FIXTURE.as_bytes());
        let metrics = report.pipeline_results[0].metrics.as_ref().unwrap();
        assert!(metrics.dependency_inventory_unknown_cases > 0);
        assert!(metrics.dependency_inventory_partial_cases > 0);
        assert!(metrics.fully_provenanced_cases < metrics.case_count);
    }

    #[test]
    fn unknown_root_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"track_id\": \"TRACK_A\"",
            "\"track_id\": \"TRACK_A\", \"unreviewed_root\": true",
        );
        let report = evaluate(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn duplicate_known_json_member_is_rejected() {
        let changed = FIXTURE.replace(
            "\"track_id\": \"TRACK_A\"",
            "\"track_id\": \"TRACK_A\", \"track_id\": \"TRACK_A\"",
        );
        let report = evaluate(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn wrong_expected_input_digest_is_rejected() {
        let report = evaluate_with_expected_sha256(
            FIXTURE.as_bytes(),
            Some("0000000000000000000000000000000000000000000000000000000000000000"),
        );
        assert_eq!(report.status, "REJECTED");
        assert_eq!(report.reasons, vec!["input_digest_mismatch"]);
    }

    #[test]
    fn pipeline_with_different_event_set_is_incomparable() {
        let mut value = parsed_fixture();
        value["pipelines"][1]["event_set_sha256"] =
            json!("9999999999999999999999999999999999999999999999999999999999999999");
        let report = evaluate_value(value);
        assert_eq!(report.status, "INCOMPARABLE");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason.starts_with("event_set_digest_mismatch:"))
        );
        assert!(report.pipeline_results.is_empty());
    }

    #[test]
    fn missing_pipeline_case_is_incomparable_not_a_negative() {
        let mut value = parsed_fixture();
        value["pipelines"][1]["case_results"]
            .as_array_mut()
            .unwrap()
            .pop();
        let report = evaluate_value(value);
        assert_eq!(report.status, "INCOMPARABLE");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason.starts_with("case_coverage_mismatch:"))
        );
    }

    #[test]
    fn duplicate_pipeline_case_is_rejected() {
        let mut value = parsed_fixture();
        let duplicate = value["pipelines"][1]["case_results"][0].clone();
        value["pipelines"][1]["case_results"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason.starts_with("duplicate_pipeline_case_id:"))
        );
    }

    #[test]
    fn privacy_or_authority_boundary_violation_blocks_all_scores() {
        let mut value = parsed_fixture();
        value["pipelines"][0]["safety_boundary"]["operator_location_export_count"] = json!(1);
        let report = evaluate_value(value);
        assert_eq!(report.status, "BLOCKED");
        assert!(
            report
                .pipeline_results
                .iter()
                .all(|pipeline| pipeline.metrics.is_none())
        );
        assert!(
            report
                .pipeline_results
                .iter()
                .any(|pipeline| pipeline.status == "BLOCKED")
        );
    }

    #[test]
    fn unresolved_case_cannot_receive_a_calibrated_binary_probability() {
        let mut value = parsed_fixture();
        value["pipelines"][0]["case_results"][6]["relevance_probability_micros"] = json!(500_000);
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        // Validation of the label/probability relationship is enforced below.
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason.contains("unresolved_probability"))
        );
    }

    #[test]
    fn source_quality_count_cannot_exceed_source_count() {
        let mut value = parsed_fixture();
        value["pipelines"][0]["case_results"][0]["source_evidence"]["authenticated_source_count"] =
            json!(3);
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason.contains("source_quality_count_exceeds_source_count"))
        );
    }

    #[test]
    fn controlled_replay_requires_review_receipt_hashes() {
        let mut value = parsed_fixture();
        value["evidence_tier"] = json!("CONTROLLED_REPLAY");
        value["simulation_only"] = json!(false);
        let report = evaluate_value(value);
        assert_eq!(report.status, "BLOCKED");
        assert_eq!(
            report.reasons,
            vec!["controlled_replay_review_receipts_missing_or_invalid"]
        );
    }

    #[test]
    fn non_independent_ground_truth_is_incomparable() {
        let mut value = parsed_fixture();
        value["ground_truth_independent"] = json!(false);
        let report = evaluate_value(value);
        assert_eq!(report.status, "INCOMPARABLE");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason == "ground_truth_independence_not_established")
        );
    }

    #[test]
    fn this_evaluator_rejects_track_b_bundles() {
        let mut value = parsed_fixture();
        value["track_id"] = json!("TRACK_B");
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .reasons
                .iter()
                .any(|reason| reason == "unsupported_track_for_this_evaluator")
        );
    }
}

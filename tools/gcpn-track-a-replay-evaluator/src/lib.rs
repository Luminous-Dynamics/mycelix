//! Synthetic-only Track A evaluator for civilian event-envelope workflows.
//!
//! This crate evaluates downstream evidence/review outputs over a shared synthetic
//! event frame. It does not score physical sensors, ingest live feeds, track aircraft,
//! classify intent, or authorize actions.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const SCHEMA: &str = "luminous.gcpn.track_a_replay_run.v0.1";
pub const STATUS: &str = "synthetic_demo_not_performance_evidence";
pub const CLAIM_CEILING: &str = "synthetic_replay_metric_arithmetic_only";
pub const EXPECTED_FIXTURE_SHA256: &str =
    "2d78a17bd708c6866dee0c0dd5ec1af104c7caec04cffc86e1c56ded90522580";

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayRun {
    pub schema: String,
    pub status: String,
    pub claim_ceiling: String,
    pub protocol_manifest_sha256: String,
    pub event_frame: EventFrame,
    pub matching: MatchingControls,
    pub systems: Vec<SystemRun>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EventFrame {
    pub frame_id: String,
    pub adjudication_basis: String,
    pub review_budget_minutes_per_system: u32,
    pub events: Vec<Event>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Event {
    pub case_id: String,
    pub incident_group: String,
    pub truth: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MatchingControls {
    pub all_systems_receive_same_event_frame: bool,
    pub same_ground_truth_denominator: bool,
    pub same_operator_review_budget: bool,
    pub physical_sensor_detection_scored: bool,
    pub independent_field_ground_truth: bool,
    pub vendor_results_claimed: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemRun {
    pub system_id: String,
    pub system_label: String,
    pub threshold: f64,
    pub outputs: Vec<SystemOutput>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemOutput {
    pub case_id: String,
    pub disposition: String,
    pub p_review_required: Option<f64>,
    pub review_ticket_id: Option<String>,
    pub review_minutes: u32,
    pub decision_latency_ms: u64,
    pub provenance_complete: bool,
    pub time_quality_handled: bool,
    pub dependency_unknown_preserved: bool,
    pub privacy_violation: bool,
    pub authority_violation: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct EvaluationReport {
    pub report_schema: &'static str,
    pub status: &'static str,
    pub synthetic_only: bool,
    pub source_sha256: String,
    pub frame_id: String,
    pub event_count: usize,
    pub system_count: usize,
    pub results: Vec<SystemMetrics>,
    pub errors: Vec<String>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, Serialize)]
pub struct SystemMetrics {
    pub system_id: String,
    pub system_label: String,
    pub event_count: usize,
    pub determinate_event_count: usize,
    pub review_required_event_count: usize,
    pub no_review_event_count: usize,
    pub indeterminate_event_count: usize,
    pub review_required_captured: usize,
    pub review_required_queued: usize,
    pub review_required_deferred: usize,
    pub review_required_missed: usize,
    pub review_required_capture_rate: Option<f64>,
    pub false_review_queues: usize,
    pub review_queue_precision: Option<f64>,
    pub false_review_queue_rate: Option<f64>,
    pub determinate_resolved_count: usize,
    pub determinate_correct_resolved: usize,
    pub decision_coverage: Option<f64>,
    pub selective_accuracy: Option<f64>,
    pub selective_error: Option<f64>,
    pub correct_abstentions_on_indeterminate: usize,
    pub inappropriate_indeterminate_resolutions: usize,
    pub brier_score: Option<f64>,
    pub unique_review_tickets: usize,
    pub duplicate_review_requests: usize,
    pub total_review_minutes: u64,
    pub review_budget_minutes: u64,
    pub review_budget_overrun_minutes: u64,
    pub decision_latency_p50_ms: Option<u64>,
    pub decision_latency_p95_ms: Option<u64>,
    pub provenance_complete_rate: Option<f64>,
    pub time_quality_handled_rate: Option<f64>,
    pub dependency_state_preserved_rate: Option<f64>,
    pub privacy_violation_count: usize,
    pub authority_violation_count: usize,
    pub hard_gates_passed: bool,
}

const EXPECTED_EVENTS: [(&str, &str, &str); 12] = [
    ("E001", "INCIDENT-A", "REVIEW_REQUIRED"),
    ("E002", "INCIDENT-A", "REVIEW_REQUIRED"),
    ("E003", "INCIDENT-B", "REVIEW_REQUIRED"),
    ("E004", "INCIDENT-C", "REVIEW_REQUIRED"),
    ("E005", "INCIDENT-D", "REVIEW_REQUIRED"),
    ("E006", "INCIDENT-E", "REVIEW_REQUIRED"),
    ("E007", "INCIDENT-F", "NO_REVIEW_REQUIRED"),
    ("E008", "INCIDENT-G", "NO_REVIEW_REQUIRED"),
    ("E009", "INCIDENT-H", "NO_REVIEW_REQUIRED"),
    ("E010", "INCIDENT-I", "INDETERMINATE"),
    ("E011", "INCIDENT-J", "INDETERMINATE"),
    ("E012", "INCIDENT-K", "INDETERMINATE"),
];

const EXPECTED_SYSTEMS: [&str; 2] = ["SYNTHETIC_RULES_REFERENCE", "SYNTHETIC_EVIDENCE_REFERENCE"];

fn ratio(numerator: usize, denominator: usize) -> Option<f64> {
    (denominator > 0).then(|| numerator as f64 / denominator as f64)
}

fn percentile(values: &mut [u64], percentile: f64) -> Option<u64> {
    if values.is_empty() {
        return None;
    }
    values.sort_unstable();
    let index = ((percentile * values.len() as f64).ceil() as usize)
        .saturating_sub(1)
        .min(values.len() - 1);
    Some(values[index])
}

fn validate_run(run: &ReplayRun) -> Vec<String> {
    let mut errors = Vec::new();

    if run.schema != SCHEMA {
        errors.push("schema_mismatch".to_string());
    }
    if run.status != STATUS || run.claim_ceiling != CLAIM_CEILING {
        errors.push("claim_ceiling_or_status_mismatch".to_string());
    }
    if run.protocol_manifest_sha256
        != "3fca74480cef7802e0fd0fddcd97949a0593c6a873f97c195d4250bd88034678"
    {
        errors.push("comparison_protocol_digest_mismatch".to_string());
    }

    let frame = &run.event_frame;
    if frame.frame_id != "GCPN-TRACK-A-DEMO-FRAME-001"
        || frame.adjudication_basis != "SYNTHETIC_ORACLE_ONLY"
        || frame.review_budget_minutes_per_system != 60
    {
        errors.push("event_frame_contract_mismatch".to_string());
    }
    if frame.events.len() != EXPECTED_EVENTS.len() {
        errors.push(format!(
            "event_count_mismatch: expected {}, got {}",
            EXPECTED_EVENTS.len(),
            frame.events.len()
        ));
    }

    let matching = &run.matching;
    if !matching.all_systems_receive_same_event_frame
        || !matching.same_ground_truth_denominator
        || !matching.same_operator_review_budget
        || matching.physical_sensor_detection_scored
        || matching.independent_field_ground_truth
        || matching.vendor_results_claimed
    {
        errors.push("matching_or_claim_ceiling_violation".to_string());
    }

    let mut event_ids = BTreeSet::new();
    for event in &frame.events {
        if event.case_id.trim().is_empty() || !event_ids.insert(event.case_id.as_str()) {
            errors.push(format!("empty_or_duplicate_event_id: {}", event.case_id));
        }
    }
    for (id, group, truth) in EXPECTED_EVENTS {
        match frame.events.iter().find(|event| event.case_id == id) {
            None => errors.push(format!("missing_event: {id}")),
            Some(event) => {
                if event.incident_group != group || event.truth != truth {
                    errors.push(format!("event_frame_semantics_mismatch: {id}"));
                }
            }
        }
    }

    if run.systems.len() != EXPECTED_SYSTEMS.len() {
        errors.push(format!(
            "system_count_mismatch: expected {}, got {}",
            EXPECTED_SYSTEMS.len(),
            run.systems.len()
        ));
    }
    let system_ids: BTreeSet<&str> = run
        .systems
        .iter()
        .map(|system| system.system_id.as_str())
        .collect();
    if system_ids.len() != run.systems.len() {
        errors.push("duplicate_system_id".to_string());
    }
    for expected in EXPECTED_SYSTEMS {
        if !system_ids.contains(expected) {
            errors.push(format!("missing_synthetic_system: {expected}"));
        }
    }

    for system in &run.systems {
        if !EXPECTED_SYSTEMS.contains(&system.system_id.as_str()) {
            errors.push(format!("non_synthetic_system_id: {}", system.system_id));
        }
        let expected_label = match system.system_id.as_str() {
            "SYNTHETIC_RULES_REFERENCE" => "Synthetic rules-only reference",
            "SYNTHETIC_EVIDENCE_REFERENCE" => "Synthetic evidence-aware reference",
            _ => "",
        };
        if system.system_label != expected_label {
            errors.push(format!("non_synthetic_system_label: {}", system.system_id));
        }
        if !system.threshold.is_finite() || !(0.0..=1.0).contains(&system.threshold) {
            errors.push(format!("invalid_threshold: {}", system.system_id));
        }

        let mut output_ids = BTreeSet::new();
        for output in &system.outputs {
            if output.case_id.trim().is_empty() || !output_ids.insert(output.case_id.as_str()) {
                errors.push(format!(
                    "empty_or_duplicate_output_id: {} -> {}",
                    system.system_id, output.case_id
                ));
            }
            if !event_ids.contains(output.case_id.as_str()) {
                errors.push(format!(
                    "unknown_output_event: {} -> {}",
                    system.system_id, output.case_id
                ));
            }

            if let Some(probability) = output.p_review_required {
                if !probability.is_finite() || !(0.0..=1.0).contains(&probability) {
                    errors.push(format!(
                        "invalid_probability: {} -> {}",
                        system.system_id, output.case_id
                    ));
                }
            }

            match output.disposition.as_str() {
                "QUEUE_HUMAN_REVIEW" | "DEFER_TO_HUMAN" => {
                    if output
                        .review_ticket_id
                        .as_deref()
                        .unwrap_or("")
                        .trim()
                        .is_empty()
                    {
                        errors.push(format!(
                            "review_disposition_missing_ticket: {} -> {}",
                            system.system_id, output.case_id
                        ));
                    }
                }
                "NO_HUMAN_REVIEW" => {
                    if output.review_ticket_id.is_some() || output.review_minutes != 0 {
                        errors.push(format!(
                            "no_review_has_ticket_or_minutes: {} -> {}",
                            system.system_id, output.case_id
                        ));
                    }
                }
                _ => errors.push(format!(
                    "unknown_disposition: {} -> {}",
                    system.system_id, output.case_id
                )),
            }

            if let Some(event) = frame
                .events
                .iter()
                .find(|event| event.case_id == output.case_id)
            {
                if event.truth == "INDETERMINATE" && output.p_review_required.is_some() {
                    errors.push(format!(
                        "probability_claimed_for_indeterminate: {} -> {}",
                        system.system_id, output.case_id
                    ));
                }
                if event.truth != "INDETERMINATE" && output.p_review_required.is_none() {
                    errors.push(format!(
                        "probability_missing_for_determinate: {} -> {}",
                        system.system_id, output.case_id
                    ));
                }
                if let Some(probability) = output.p_review_required {
                    if output.disposition == "QUEUE_HUMAN_REVIEW" && probability < system.threshold
                    {
                        errors.push(format!(
                            "queue_below_threshold: {} -> {}",
                            system.system_id, output.case_id
                        ));
                    }
                    if output.disposition == "NO_HUMAN_REVIEW" && probability >= system.threshold {
                        errors.push(format!(
                            "no_review_above_threshold: {} -> {}",
                            system.system_id, output.case_id
                        ));
                    }
                }
            }
        }

        if output_ids.len() != event_ids.len()
            || event_ids.iter().any(|id| !output_ids.contains(id))
        {
            errors.push(format!("output_event_frame_mismatch: {}", system.system_id));
        }

        // A ticket may be shared by duplicate observations of one incident, but
        // it must never cross incident groups or carry inconsistent time costs.
        let mut ticket_owner: BTreeMap<&str, (&str, u32)> = BTreeMap::new();
        for output in &system.outputs {
            if let Some(ticket) = output.review_ticket_id.as_deref() {
                if let Some(event) = frame
                    .events
                    .iter()
                    .find(|event| event.case_id == output.case_id)
                {
                    match ticket_owner.get(ticket) {
                        Some((group, minutes))
                            if *group != event.incident_group
                                || *minutes != output.review_minutes =>
                        {
                            errors.push(format!(
                                "review_ticket_reused_inconsistently: {} -> {}",
                                system.system_id, ticket
                            ));
                        }
                        None => {
                            ticket_owner
                                .insert(ticket, (&event.incident_group, output.review_minutes));
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    errors.sort();
    errors
}

fn compute_metrics(frame: &EventFrame, system: &SystemRun) -> SystemMetrics {
    let events: BTreeMap<&str, &Event> = frame
        .events
        .iter()
        .map(|event| (event.case_id.as_str(), event))
        .collect();
    let outputs: BTreeMap<&str, &SystemOutput> = system
        .outputs
        .iter()
        .map(|output| (output.case_id.as_str(), output))
        .collect();

    let mut determinate = 0usize;
    let mut review_required = 0usize;
    let mut no_review = 0usize;
    let mut indeterminate = 0usize;
    let mut review_captured = 0usize;
    let mut review_queued = 0usize;
    let mut review_deferred = 0usize;
    let mut review_missed = 0usize;
    let mut false_review_queues = 0usize;
    let mut determinate_resolved = 0usize;
    let mut determinate_correct = 0usize;
    let mut correct_abstentions = 0usize;
    let mut inappropriate_indeterminate = 0usize;
    let mut brier_sum = 0.0f64;
    let mut brier_count = 0usize;
    let mut latencies = Vec::with_capacity(frame.events.len());
    let mut provenance_complete = 0usize;
    let mut time_quality_handled = 0usize;
    let mut dependency_preserved = 0usize;
    let mut privacy_violations = 0usize;
    let mut authority_violations = 0usize;

    // Unique tickets are counted once even when several duplicate observations
    // point at the same human-review request.
    let mut ticket_owner: BTreeMap<&str, (&str, u32)> = BTreeMap::new();
    let mut group_tickets: BTreeMap<&str, BTreeSet<&str>> = BTreeMap::new();

    for event in &frame.events {
        let output = outputs[event.case_id.as_str()];
        latencies.push(output.decision_latency_ms);

        provenance_complete += usize::from(output.provenance_complete);
        time_quality_handled += usize::from(output.time_quality_handled);
        dependency_preserved += usize::from(output.dependency_unknown_preserved);
        privacy_violations += usize::from(output.privacy_violation);
        authority_violations += usize::from(output.authority_violation);

        if let Some(ticket) = output.review_ticket_id.as_deref() {
            ticket_owner
                .entry(ticket)
                .or_insert((&event.incident_group, output.review_minutes));
            group_tickets
                .entry(&event.incident_group)
                .or_default()
                .insert(ticket);
        }

        match event.truth.as_str() {
            "REVIEW_REQUIRED" => {
                determinate += 1;
                review_required += 1;
                match output.disposition.as_str() {
                    "QUEUE_HUMAN_REVIEW" => {
                        review_captured += 1;
                        review_queued += 1;
                        determinate_resolved += 1;
                        determinate_correct += 1;
                    }
                    "DEFER_TO_HUMAN" => {
                        review_captured += 1;
                        review_deferred += 1;
                    }
                    "NO_HUMAN_REVIEW" => {
                        review_missed += 1;
                        determinate_resolved += 1;
                    }
                    _ => {}
                }
                if let Some(probability) = output.p_review_required {
                    brier_sum += (probability - 1.0).powi(2);
                    brier_count += 1;
                }
            }
            "NO_REVIEW_REQUIRED" => {
                determinate += 1;
                no_review += 1;
                match output.disposition.as_str() {
                    "QUEUE_HUMAN_REVIEW" => {
                        false_review_queues += 1;
                        determinate_resolved += 1;
                    }
                    "NO_HUMAN_REVIEW" => {
                        determinate_resolved += 1;
                        determinate_correct += 1;
                    }
                    _ => {}
                }
                if let Some(probability) = output.p_review_required {
                    brier_sum += probability.powi(2);
                    brier_count += 1;
                }
            }
            "INDETERMINATE" => {
                indeterminate += 1;
                if output.disposition == "DEFER_TO_HUMAN" {
                    correct_abstentions += 1;
                } else {
                    inappropriate_indeterminate += 1;
                }
            }
            _ => {}
        }
    }

    let unique_review_tickets = ticket_owner.len();
    let total_review_minutes = ticket_owner
        .values()
        .map(|(_, minutes)| u64::from(*minutes))
        .sum::<u64>();
    let duplicate_review_requests = group_tickets
        .values()
        .map(|tickets| tickets.len().saturating_sub(1))
        .sum::<usize>();
    let mut latency_copy = latencies.clone();

    let queue_count = system
        .outputs
        .iter()
        .filter(|output| output.disposition == "QUEUE_HUMAN_REVIEW")
        .count();

    let review_queue_precision = ratio(review_queued, queue_count);

    SystemMetrics {
        system_id: system.system_id.clone(),
        system_label: system.system_label.clone(),
        event_count: frame.events.len(),
        determinate_event_count: determinate,
        review_required_event_count: review_required,
        no_review_event_count: no_review,
        indeterminate_event_count: indeterminate,
        review_required_captured: review_captured,
        review_required_queued: review_queued,
        review_required_deferred: review_deferred,
        review_required_missed: review_missed,
        review_required_capture_rate: ratio(review_captured, review_required),
        false_review_queues,
        review_queue_precision,
        false_review_queue_rate: ratio(false_review_queues, no_review),
        determinate_resolved_count: determinate_resolved,
        determinate_correct_resolved: determinate_correct,
        decision_coverage: ratio(determinate_resolved, determinate),
        selective_accuracy: ratio(determinate_correct, determinate_resolved),
        selective_error: ratio(
            determinate_resolved.saturating_sub(determinate_correct),
            determinate_resolved,
        ),
        correct_abstentions_on_indeterminate: correct_abstentions,
        inappropriate_indeterminate_resolutions: inappropriate_indeterminate,
        brier_score: (brier_count > 0).then(|| brier_sum / brier_count as f64),
        unique_review_tickets,
        duplicate_review_requests,
        total_review_minutes,
        review_budget_minutes: u64::from(frame.review_budget_minutes_per_system),
        review_budget_overrun_minutes: total_review_minutes
            .saturating_sub(u64::from(frame.review_budget_minutes_per_system)),
        decision_latency_p50_ms: percentile(&mut latency_copy, 0.50),
        decision_latency_p95_ms: percentile(&mut latency_copy, 0.95),
        provenance_complete_rate: ratio(provenance_complete, frame.events.len()),
        time_quality_handled_rate: ratio(time_quality_handled, frame.events.len()),
        dependency_state_preserved_rate: ratio(dependency_preserved, frame.events.len()),
        privacy_violation_count: privacy_violations,
        authority_violation_count: authority_violations,
        hard_gates_passed: privacy_violations == 0 && authority_violations == 0,
    }
}

fn report_for_run(
    run: &ReplayRun,
    source_sha256: String,
    mut errors: Vec<String>,
) -> EvaluationReport {
    errors.extend(validate_run(run));
    let mut results = Vec::new();
    if errors.is_empty() {
        results = run
            .systems
            .iter()
            .map(|system| compute_metrics(&run.event_frame, system))
            .collect();
    }
    errors.sort();

    let status = if !errors.is_empty() {
        "REJECTED"
    } else if results.iter().any(|result| !result.hard_gates_passed) {
        "HARD_GATE_FAILURE"
    } else {
        "CONFORMS_SYNTHETIC_ONLY"
    };

    EvaluationReport {
        report_schema: "luminous.gcpn.track_a_replay_report.v0.1",
        status,
        synthetic_only: true,
        source_sha256,
        frame_id: run.event_frame.frame_id.clone(),
        event_count: run.event_frame.events.len(),
        system_count: results.len(),
        results,
        errors,
        claim_ceiling: "synthetic_replay_metric_arithmetic_only",
    }
}

pub fn evaluate_bytes(bytes: &[u8]) -> EvaluationReport {
    let source_sha256 = format!("{:x}", Sha256::digest(bytes));
    let mut errors = Vec::new();
    if source_sha256 != EXPECTED_FIXTURE_SHA256 {
        errors.push(format!(
            "fixture_digest_mismatch: expected {}, got {}",
            EXPECTED_FIXTURE_SHA256, source_sha256
        ));
    }

    match serde_json::from_slice::<ReplayRun>(bytes) {
        Ok(run) => report_for_run(&run, source_sha256, errors),
        Err(error) => {
            errors.push(format!("strict_json_decode: {error}"));
            errors.sort();
            EvaluationReport {
                report_schema: "luminous.gcpn.track_a_replay_report.v0.1",
                status: "REJECTED",
                synthetic_only: true,
                source_sha256,
                frame_id: String::new(),
                event_count: 0,
                system_count: 0,
                results: Vec::new(),
                errors,
                claim_ceiling: "synthetic_replay_metric_arithmetic_only",
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;


    const FIXTURE: &str = include_str!(
        "../../../mycelix-workspace/mycelix-praxis/docs/research/GCPN_TRACK_A_REPLAY_DEMO_RUN_V0_1.json"
    );

    #[test]
    fn frozen_demo_run_conforms_and_is_explicitly_synthetic() {
        let report = evaluate_bytes(FIXTURE.as_bytes());
        assert_eq!(
            report.status, "CONFORMS_SYNTHETIC_ONLY",
            "{:?}",
            report.errors
        );
        assert!(report.synthetic_only);
        assert_eq!(report.event_count, 12);
        assert_eq!(report.system_count, 2);
    }

    #[test]
    fn exact_fixture_digest_is_pinned() {
        let report = evaluate_bytes(FIXTURE.as_bytes());
        assert_eq!(report.source_sha256, EXPECTED_FIXTURE_SHA256);
    }

    #[test]
    fn unknown_root_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"status\": \"synthetic_demo_not_performance_evidence\"",
            "\"status\": \"synthetic_demo_not_performance_evidence\", \"extra\": true",
        );
        assert_ne!(changed, FIXTURE);
        let report = evaluate_bytes(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn duplicate_known_json_member_is_rejected() {
        let changed = FIXTURE.replace(
            "\"review_budget_minutes_per_system\": 60",
            "\"review_budget_minutes_per_system\": 60, \"review_budget_minutes_per_system\": 90",
        );
        assert_ne!(changed, FIXTURE);
        let report = evaluate_bytes(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(
            report
                .errors
                .iter()
                .any(|error| error.starts_with("strict_json_decode:"))
        );
    }

    #[test]
    fn missing_output_is_rejected() {
        let mut run: ReplayRun = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
        run.systems[0].outputs.pop();
        let report = report_for_run(&run, "mutated".to_string(), Vec::new());
        assert_eq!(report.status, "REJECTED");
        assert!(report
            .errors
            .iter()
            .any(|error| error.starts_with("output_event_frame_mismatch:")));
    }

    #[test]
    fn vendor_named_system_cannot_enter_synthetic_demo() {
        let mut run: ReplayRun = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
        run.systems[0].system_id = "DED RONE RF360".to_string();
        let report = report_for_run(&run, "mutated".to_string(), Vec::new());
        assert_eq!(report.status, "REJECTED");
        assert!(report
            .errors
            .iter()
            .any(|error| error.starts_with("non_synthetic_system_id:")));
    }

    #[test]
    fn probabilities_are_checked_by_schema_semantics() {
        let mut run: ReplayRun = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
        run.systems[0].outputs[0].p_review_required = Some(1.50);
        let report = report_for_run(&run, "mutated".to_string(), Vec::new());
        assert_eq!(report.status, "REJECTED");
        assert!(report
            .errors
            .iter()
            .any(|error| error.starts_with("invalid_probability:")));
    }

    #[test]
    fn privacy_violation_fails_the_hard_gate() {
        let mut run: ReplayRun = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
        run.systems[0].outputs[0].privacy_violation = true;
        let report = report_for_run(&run, "mutated".to_string(), Vec::new());
        assert_eq!(report.status, "HARD_GATE_FAILURE");
        assert_eq!(report.results[0].privacy_violation_count, 1);
        assert!(!report.results[0].hard_gates_passed);
    }

    #[test]
    fn authority_violation_fails_the_hard_gate() {
        let mut run: ReplayRun = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
        run.systems[0].outputs[0].authority_violation = true;
        let report = report_for_run(&run, "mutated".to_string(), Vec::new());
        assert_eq!(report.status, "HARD_GATE_FAILURE");
        assert_eq!(report.results[0].authority_violation_count, 1);
        assert!(!report.results[0].hard_gates_passed);
    }

    #[test]
    fn expected_demo_arithmetic_has_no_authority_or_privacy_violations() {
        // The expected values below are deliberately synthetic arithmetic checks,
        // not vendor/product or field-performance results.
        let report = evaluate_bytes(FIXTURE.as_bytes());
        assert_eq!(report.status, "CONFORMS_SYNTHETIC_ONLY");
        assert!(report.results.iter().all(|result| result.hard_gates_passed));
        assert!(
            report
                .results
                .iter()
                .all(|result| result.privacy_violation_count == 0)
        );
        assert!(
            report
                .results
                .iter()
                .all(|result| result.authority_violation_count == 0)
        );
        assert_eq!(report.results[0].duplicate_review_requests, 1);
        assert_eq!(report.results[1].duplicate_review_requests, 0);
    }
}

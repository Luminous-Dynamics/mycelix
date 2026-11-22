//! Synthetic known-truth evaluator for event sampling schedules.
//!
//! The candidate schedule is kept separate from the evaluator-only truth file.
//! Every candidate observation must match an exact timestamp and value from
//! the high-rate source. No hidden observation is interpolated.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic evaluator only. The truth catalogue is evaluator-only; candidate values must be present in the high-rate source. No unseen observation is interpolated. Results are not physical sensor energy, field emissions, causal management effects or validated counterfactual replay.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Truth {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    trace_id: String,
    baseline_value: f64,
    detection_threshold_above_baseline: f64,
    time_resolution_hours: f64,
    samples: Vec<TruthSample>,
    pulses: Vec<Pulse>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TruthSample {
    time_hours: f64,
    response_value: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Pulse {
    pulse_id: String,
    start_hours: f64,
    end_hours: f64,
    minimum_detected_samples: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Schedule {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    schedule_id: String,
    policy_name: String,
    maximum_sample_budget: usize,
    maximum_gap_for_pulse_integral_hours: f64,
    wakeup_gap_threshold_hours: f64,
    resource_cost_model: ResourceCostModel,
    observations: Vec<CandidateObservation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ResourceCostModel {
    sample_energy_joules: f64,
    wake_energy_joules: f64,
    compute_cycles_per_sample: u64,
    transmitted_bytes_per_sample: u64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CandidateObservation {
    observation_id: String,
    decision_time_hours: f64,
    time_hours: f64,
    available_observation_ids: Vec<String>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    trace_id: String,
    schedule_id: String,
    policy_name: String,
    truth_sha256: String,
    schedule_sha256: String,
    policy_evaluator_isolation_status: &'static str,
    truth_observation_count: usize,
    candidate_sample_count: usize,
    sample_fraction: f64,
    maximum_sample_budget: usize,
    budget_status: &'static str,
    maximum_schedule_gap_hours: f64,
    wake_count: usize,
    pulse_count: usize,
    detected_pulse_count: usize,
    pulse_recall: f64,
    missed_pulse_truth_exposure_unit_hours: f64,
    false_alarm_count: usize,
    resource_vector: ResourceVector,
    pulses: Vec<PulseResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct ResourceVector {
    sample_energy_joules: f64,
    wake_energy_joules: f64,
    compute_cycles: u64,
    transmitted_bytes: u64,
}

#[derive(Debug, Serialize)]
struct PulseResult {
    pulse_id: String,
    truth_peak_value: f64,
    truth_peak_time_hours: f64,
    truth_positive_anomaly_integral_unit_hours: f64,
    candidate_samples_in_window: usize,
    detected: bool,
    candidate_peak_value: Option<f64>,
    candidate_peak_absolute_error: Option<f64>,
    candidate_integral_unit_hours: Option<f64>,
    integral_status: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    candidate_timestamps_bind_to_truth_source: bool,
    strict_schedule_order_pass: bool,
    no_duplicate_sample_time_pass: bool,
    no_interpolation_pass: bool,
    decision_information_causality_pass: bool,
    no_scalar_cost_aggregation_pass: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn validate_truth(t: &Truth) -> Result<(), String> {
    if t.schema_version != "soil-event-truth-v0.1"
        || t.fixture_status != FIXTURE_STATUS
        || t.evidence_class != "synthetic_evaluator_only"
    {
        return Err("truth input must use the synthetic evaluator-only schema".into());
    }
    if t.trace_id.trim().is_empty() || t.samples.len() < 2 || t.pulses.is_empty() {
        return Err("truth trace needs ID, at least two samples and pulse definitions".into());
    }
    finite("baseline_value", t.baseline_value)?;
    finite(
        "detection_threshold_above_baseline",
        t.detection_threshold_above_baseline,
    )?;
    finite("time_resolution_hours", t.time_resolution_hours)?;
    if t.detection_threshold_above_baseline < 0.0 || t.time_resolution_hours <= 0.0 {
        return Err("truth threshold and time resolution must be non-negative/positive".into());
    }
    for (i, sample) in t.samples.iter().enumerate() {
        finite("truth sample time", sample.time_hours)?;
        finite("truth response", sample.response_value)?;
        if i > 0 {
            let gap = sample.time_hours - t.samples[i - 1].time_hours;
            if gap <= 0.0 {
                return Err("truth sample times must increase strictly".into());
            }
            if (gap - t.time_resolution_hours).abs() > 1e-8 {
                return Err("truth source must use its declared fixed time resolution".into());
            }
        }
    }
    let mut pulse_ids = BTreeSet::new();
    for p in &t.pulses {
        if p.pulse_id.trim().is_empty() || !pulse_ids.insert(p.pulse_id.clone()) {
            return Err("pulse IDs must be non-empty and unique".into());
        }
        finite("pulse start", p.start_hours)?;
        finite("pulse end", p.end_hours)?;
        if p.start_hours >= p.end_hours || p.minimum_detected_samples == 0 {
            return Err("pulse windows and sample requirements must be valid".into());
        }
        if !t.samples.iter().any(|s| s.time_hours == p.start_hours)
            || !t.samples.iter().any(|s| s.time_hours == p.end_hours)
        {
            return Err(format!(
                "pulse {} boundaries must coincide with truth samples",
                p.pulse_id
            ));
        }
    }
    Ok(())
}

fn validate_schedule(s: &Schedule, t: &Truth) -> Result<(), String> {
    if s.schema_version != "soil-event-schedule-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic_candidate_policy"
    {
        return Err("schedule input must use the synthetic candidate schema".into());
    }
    if s.schedule_id.trim().is_empty()
        || s.policy_name.trim().is_empty()
        || s.maximum_sample_budget == 0
        || s.observations.is_empty()
    {
        return Err("schedule ID, policy, budget and observations are required".into());
    }
    for (name, value) in [
        (
            "maximum_gap_for_pulse_integral_hours",
            s.maximum_gap_for_pulse_integral_hours,
        ),
        ("wakeup_gap_threshold_hours", s.wakeup_gap_threshold_hours),
        (
            "sample_energy_joules",
            s.resource_cost_model.sample_energy_joules,
        ),
        (
            "wake_energy_joules",
            s.resource_cost_model.wake_energy_joules,
        ),
    ] {
        finite(name, value)?;
        if value < 0.0 {
            return Err(format!("{name} cannot be negative"));
        }
    }
    if s.maximum_gap_for_pulse_integral_hours <= 0.0 || s.wakeup_gap_threshold_hours <= 0.0 {
        return Err("gap thresholds must be positive".into());
    }

    // A candidate file is a frozen schedule/action trace, not a source of
    // response values. The evaluator joins exact timestamps to hidden truth
    // only after the trace has been submitted.
    let mut ids = BTreeSet::new();
    let mut previously_sampled: BTreeMap<String, f64> = BTreeMap::new();
    let mut prior_sample_time: Option<f64> = None;
    let mut prior_decision_time: Option<f64> = None;

    for observation in &s.observations {
        if observation.observation_id.trim().is_empty()
            || !ids.insert(observation.observation_id.clone())
        {
            return Err("candidate observation IDs must be non-empty and unique".into());
        }
        finite("candidate decision time", observation.decision_time_hours)?;
        finite("candidate observation time", observation.time_hours)?;

        if observation.decision_time_hours > observation.time_hours {
            return Err(format!(
                "decision for {} occurs after its requested observation time",
                observation.observation_id
            ));
        }
        if let Some(previous) = prior_decision_time
            && observation.decision_time_hours <= previous
        {
            return Err("candidate decision times must increase strictly".into());
        }
        if let Some(previous) = prior_sample_time
            && observation.time_hours <= previous
        {
            return Err("candidate observation times must increase strictly".into());
        }

        let mut available_ids = BTreeSet::new();
        for available_id in &observation.available_observation_ids {
            if !available_ids.insert(available_id) {
                return Err(format!(
                    "decision for {} repeats an available observation ID",
                    observation.observation_id
                ));
            }
            let sampled_at = previously_sampled.get(available_id).ok_or_else(|| {
                format!(
                    "decision for {} references an unknown or future observation {}",
                    observation.observation_id, available_id
                )
            })?;
            if *sampled_at > observation.decision_time_hours {
                return Err(format!(
                    "decision for {} references observation {} before it was sampled",
                    observation.observation_id, available_id
                ));
            }
        }

        if !t
            .samples
            .iter()
            .any(|truth| truth.time_hours == observation.time_hours)
        {
            return Err(format!(
                "candidate time {} is not present in known-truth source",
                observation.time_hours
            ));
        }

        prior_sample_time = Some(observation.time_hours);
        prior_decision_time = Some(observation.decision_time_hours);
        previously_sampled.insert(
            observation.observation_id.clone(),
            observation.time_hours,
        );
    }
    Ok(())
}

fn integrate(samples: &[TruthSample], baseline: f64) -> f64 {
    samples
        .windows(2)
        .map(|pair| {
            let a = (pair[0].response_value - baseline).max(0.0);
            let b = (pair[1].response_value - baseline).max(0.0);
            0.5 * (a + b) * (pair[1].time_hours - pair[0].time_hours)
        })
        .sum()
}

fn truth_value_at(t: &Truth, time_hours: f64) -> f64 {
    t.samples
        .iter()
        .find(|sample| sample.time_hours == time_hours)
        .expect("schedule validation requires exact truth timestamp")
        .response_value
}

fn pulse_result(t: &Truth, s: &Schedule, p: &Pulse) -> PulseResult {
    let truth: Vec<&TruthSample> = t
        .samples
        .iter()
        .filter(|x| x.time_hours >= p.start_hours && x.time_hours <= p.end_hours)
        .collect();
    let truth_peak = truth
        .iter()
        .copied()
        .max_by(|a, b| a.response_value.total_cmp(&b.response_value))
        .expect("validated pulse boundaries ensure truth support");
    let truth_integral = integrate(
        &t.samples
            .iter()
            .filter(|x| x.time_hours >= p.start_hours && x.time_hours <= p.end_hours)
            .cloned()
            .collect::<Vec<_>>(),
        t.baseline_value,
    );
    let observed: Vec<&CandidateObservation> = s
        .observations
        .iter()
        .filter(|x| x.time_hours >= p.start_hours && x.time_hours <= p.end_hours)
        .collect();
    let observed_values: Vec<(&CandidateObservation, f64)> = observed
        .iter()
        .map(|sample| (*sample, truth_value_at(t, sample.time_hours)))
        .collect();
    let observed_peak = observed_values
        .iter()
        .copied()
        .max_by(|a, b| a.1.total_cmp(&b.1));
    let detected = observed_values.len() >= p.minimum_detected_samples
        && observed_values.iter().any(|(_, value)| {
            *value - t.baseline_value > t.detection_threshold_above_baseline
        });
    let candidate_peak = if detected {
        observed_peak.map(|(_, value)| value)
    } else {
        None
    };
    let peak_error = candidate_peak.map(|value| (value - truth_peak.response_value).abs());

    let has_start = observed.iter().any(|x| x.time_hours == p.start_hours);
    let has_end = observed.iter().any(|x| x.time_hours == p.end_hours);
    let (candidate_integral, integral_status) = if !detected {
        (None, "not_detected")
    } else if !has_start || !has_end {
        (None, "missing_event_boundary")
    } else {
        let gap_exceeded = observed.windows(2).any(|pair| {
            pair[1].time_hours - pair[0].time_hours > s.maximum_gap_for_pulse_integral_hours
        });
        if gap_exceeded {
            (None, "gap_exceeded")
        } else {
            let value: f64 = observed_values
                .windows(2)
                .map(|pair| {
                    let a = (pair[0].1 - t.baseline_value).max(0.0);
                    let b = (pair[1].1 - t.baseline_value).max(0.0);
                    0.5 * (a + b) * (pair[1].0.time_hours - pair[0].0.time_hours)
                })
                .sum();
            (Some(value), "qualified_complete_pulse_integral")
        }
    };

    PulseResult {
        pulse_id: p.pulse_id.clone(),
        truth_peak_value: truth_peak.response_value,
        truth_peak_time_hours: truth_peak.time_hours,
        truth_positive_anomaly_integral_unit_hours: truth_integral,
        candidate_samples_in_window: observed.len(),
        detected,
        candidate_peak_value: candidate_peak,
        candidate_peak_absolute_error: peak_error,
        candidate_integral_unit_hours: candidate_integral,
        integral_status,
    }
}

fn build_report(t: &Truth, s: &Schedule, truth_digest: String, schedule_digest: String) -> Report {
    let pulses: Vec<PulseResult> = t.pulses.iter().map(|p| pulse_result(t, s, p)).collect();
    let detected = pulses.iter().filter(|p| p.detected).count();
    let missed_exposure = pulses
        .iter()
        .filter(|p| !p.detected)
        .map(|p| p.truth_positive_anomaly_integral_unit_hours)
        .sum();
    let false_alarm_count = s
        .observations
        .iter()
        .filter(|observation| {
            !t.pulses.iter().any(|pulse| {
                observation.time_hours >= pulse.start_hours
                    && observation.time_hours <= pulse.end_hours
            }) && truth_value_at(t, observation.time_hours) - t.baseline_value
                > t.detection_threshold_above_baseline
        })
        .count();
    let max_gap = s
        .observations
        .windows(2)
        .map(|pair| pair[1].time_hours - pair[0].time_hours)
        .max_by(f64::total_cmp)
        .unwrap_or(0.0);
    let wake_count = if s.observations.is_empty() {
        0
    } else {
        1 + s
            .observations
            .windows(2)
            .filter(|pair| pair[1].time_hours - pair[0].time_hours > s.wakeup_gap_threshold_hours)
            .count()
    };
    let count = s.observations.len() as u64;
    let cost = &s.resource_cost_model;

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        trace_id: t.trace_id.clone(),
        schedule_id: s.schedule_id.clone(),
        policy_name: s.policy_name.clone(),
        truth_sha256: truth_digest,
        schedule_sha256: schedule_digest,
        policy_evaluator_isolation_status: "not_enforced_by_offline_evaluator",
        truth_observation_count: t.samples.len(),
        candidate_sample_count: s.observations.len(),
        sample_fraction: s.observations.len() as f64 / t.samples.len() as f64,
        maximum_sample_budget: s.maximum_sample_budget,
        budget_status: if s.observations.len() <= s.maximum_sample_budget {
            "within_budget"
        } else {
            "over_budget"
        },
        maximum_schedule_gap_hours: max_gap,
        wake_count,
        pulse_count: pulses.len(),
        detected_pulse_count: detected,
        pulse_recall: detected as f64 / pulses.len() as f64,
        missed_pulse_truth_exposure_unit_hours: missed_exposure,
        false_alarm_count,
        resource_vector: ResourceVector {
            sample_energy_joules: count as f64 * cost.sample_energy_joules,
            wake_energy_joules: wake_count as f64 * cost.wake_energy_joules,
            compute_cycles: count.saturating_mul(cost.compute_cycles_per_sample),
            transmitted_bytes: count.saturating_mul(cost.transmitted_bytes_per_sample),
        },
        pulses,
        controls: Controls {
            candidate_timestamps_bind_to_truth_source: true,
            strict_schedule_order_pass: true,
            no_duplicate_sample_time_pass: true,
            no_interpolation_pass: true,
            decision_information_causality_pass: true,
            no_scalar_cost_aggregation_pass: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(
    truth_path: &Path,
    schedule_path: &Path,
    report_path: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    if report_path.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "report already exists").into());
    }
    let truth_bytes = fs::read(truth_path)?;
    let schedule_bytes = fs::read(schedule_path)?;
    let truth: Truth = serde_json::from_slice(&truth_bytes)?;
    let schedule: Schedule = serde_json::from_slice(&schedule_bytes)?;
    validate_truth(&truth).map_err(io::Error::other)?;
    validate_schedule(&schedule, &truth).map_err(io::Error::other)?;
    let td = format!("{:x}", Sha256::digest(&truth_bytes));
    let sd = format!("{:x}", Sha256::digest(&schedule_bytes));
    let report = build_report(&truth, &schedule, td, sd);
    let bytes = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(report_path)?;
    file.write_all(&bytes)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        eprintln!(
            "usage: event-sampling-audit <evaluator-truth.json> <candidate-schedule.json> <new-report.json>"
        );
        std::process::exit(2);
    }
    if let Err(e) = run(
        Path::new(&args[1]),
        Path::new(&args[2]),
        Path::new(&args[3]),
    ) {
        eprintln!("event sampling audit failed: {e}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load() -> (Truth, Schedule) {
        let t: Truth =
            serde_json::from_str(include_str!("../fixtures/synthetic-event-truth.json")).unwrap();
        let s: Schedule = serde_json::from_str(include_str!(
            "../fixtures/synthetic-candidate-schedule.json"
        ))
        .unwrap();
        (t, s)
    }

    #[test]
    fn adaptive_burst_captures_wide_event_but_misses_narrow_event() {
        let (t, s) = load();
        validate_truth(&t).unwrap();
        validate_schedule(&s, &t).unwrap();
        let r = build_report(&t, &s, "0".repeat(64), "1".repeat(64));
        assert_eq!(r.candidate_sample_count, 54);
        assert_eq!(r.budget_status, "within_budget");
        assert_eq!(r.detected_pulse_count, 1);
        assert!((r.pulse_recall - 0.5).abs() < 1e-12);
        assert!(r.pulses[0].detected);
        assert!(!r.pulses[1].detected);
        assert!(r.missed_pulse_truth_exposure_unit_hours > 0.0);
        assert_eq!(
            r.pulses[0].integral_status,
            "qualified_complete_pulse_integral"
        );
        assert_eq!(r.pulses[1].integral_status, "not_detected");
    }

    #[test]
    fn fixed_thirty_minute_schedule_misses_both_short_pulses() {
        let t: Truth =
            serde_json::from_str(include_str!("../fixtures/synthetic-event-truth.json")).unwrap();
        let s: Schedule = serde_json::from_str(include_str!(
            "../fixtures/synthetic-fixed-30m-schedule.json"
        ))
        .unwrap();
        validate_truth(&t).unwrap();
        validate_schedule(&s, &t).unwrap();
        let report = build_report(&t, &s, "0".repeat(64), "1".repeat(64));
        assert_eq!(report.candidate_sample_count, 49);
        assert_eq!(report.detected_pulse_count, 0);
        assert_eq!(report.pulse_recall, 0.0);
        assert_eq!(report.pulses[0].integral_status, "not_detected");
        assert_eq!(report.pulses[1].integral_status, "not_detected");
    }

    #[test]
    fn fixed_five_minute_schedule_captures_both_pulses_but_exceeds_budget() {
        let t: Truth =
            serde_json::from_str(include_str!("../fixtures/synthetic-event-truth.json")).unwrap();
        let s: Schedule =
            serde_json::from_str(include_str!("../fixtures/synthetic-fixed-5m-schedule.json"))
                .unwrap();
        validate_truth(&t).unwrap();
        validate_schedule(&s, &t).unwrap();
        let report = build_report(&t, &s, "0".repeat(64), "1".repeat(64));
        assert_eq!(report.candidate_sample_count, 289);
        assert_eq!(report.detected_pulse_count, 2);
        assert_eq!(report.pulse_recall, 1.0);
        assert_eq!(report.budget_status, "over_budget");
        assert!(
            report
                .pulses
                .iter()
                .all(|pulse| { pulse.integral_status == "qualified_complete_pulse_integral" })
        );
    }

    #[test]
    fn candidate_cannot_request_an_unobserved_timestamp() {
        let (t, mut s) = load();
        s.observations[0].time_hours += 0.01;
        assert!(validate_schedule(&s, &t).is_err());
    }

    #[test]
    fn decision_cannot_reference_future_or_unknown_observation() {
        let (t, mut s) = load();
        let future_id = s.observations.last().unwrap().observation_id.clone();
        s.observations[0].available_observation_ids.push(future_id);
        assert!(validate_schedule(&s, &t).is_err());

        let (t, mut s) = load();
        s.observations[0]
            .available_observation_ids
            .push("not-a-scheduled-observation".into());
        assert!(validate_schedule(&s, &t).is_err());
    }

    #[test]
    fn decision_cannot_use_an_observation_before_it_was_acquired() {
        let (t, mut s) = load();
        s.observations[1].decision_time_hours = s.observations[0].time_hours - 0.01;
        assert!(validate_schedule(&s, &t).is_err());
    }

    #[test]
    fn offline_evaluator_does_not_claim_process_isolation() {
        let (t, s) = load();
        let report = build_report(&t, &s, "0".repeat(64), "1".repeat(64));
        assert_eq!(
            report.policy_evaluator_isolation_status,
            "not_enforced_by_offline_evaluator"
        );
        assert!(report.controls.decision_information_causality_pass);
    }

    #[test]
    fn duplicate_and_nonmonotonic_candidate_times_are_rejected() {
        let (t, mut s) = load();
        s.observations[1].time_hours = s.observations[0].time_hours;
        assert!(validate_schedule(&s, &t).is_err());
    }

    #[test]
    fn evaluator_rejects_nonuniform_source_resolution() {
        let (mut t, _) = load();
        t.samples[2].time_hours += 0.001;
        assert!(validate_truth(&t).is_err());
    }

    #[test]
    fn truth_pulse_requires_boundaries_on_dense_source() {
        let (mut t, _) = load();
        t.pulses[0].end_hours = 6.49;
        assert!(validate_truth(&t).is_err());
    }

    #[test]
    fn sample_over_budget_is_reported_not_hidden() {
        let (t, mut s) = load();
        s.maximum_sample_budget = 10;
        let r = build_report(&t, &s, "0".repeat(64), "1".repeat(64));
        assert_eq!(r.budget_status, "over_budget");
    }

    #[test]
    fn resource_costs_remain_a_vector() {
        let (t, s) = load();
        let r = build_report(&t, &s, "0".repeat(64), "1".repeat(64));
        assert!(r.resource_vector.sample_energy_joules > 0.0);
        assert!(r.resource_vector.compute_cycles > 0);
        assert!(r.resource_vector.transmitted_bytes > 0);
    }
}

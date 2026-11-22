//! Synthetic-only known-truth evaluator for submitted sensor schedules.
//!
//! Policy schedules and evaluator truth are separate CLI inputs. The policy
//! schema intentionally has no measurement or latent-truth fields. The scorer
//! resolves only exact requested timestamps; it never interpolates.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const POLICY_STATUS: &str = "synthetic_policy_schedule_only_not_empirical";
const TRUTH_STATUS: &str = "synthetic_evaluator_only_truth_not_policy_input";
const REPORT_STATUS: &str = "synthetic_event_capture_score_not_empirical";
const BOUNDARY: &str = "Synthetic known-truth schedule evaluation only. Static fixture schedules are not live sensing policies. Separate inputs do not by themselves prove future policy truth-blindness; a sandbox must enforce process/data isolation. No field pulse, emission factor or operational sensing capability is qualified.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolicySubmission {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    submission_id: String,
    policy_resource_limits: ResourceLimits,
    policies: Vec<Policy>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ResourceLimits {
    maximum_samples_per_policy: usize,
    sample_cost_units: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Policy {
    policy_id: String,
    policy_family: String,
    policy_version: String,
    policy_code_sha256: String,
    policy_config_sha256: String,
    random_seed: Option<u64>,
    requested_times_hours: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Truth {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    response_variable: String,
    response_unit: String,
    measurement_operator: MeasurementOperator,
    baseline_window: BaselineWindow,
    maximum_event_integration_gap_hours: f64,
    truth_series: Vec<TruthPoint>,
    events: Vec<Event>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MeasurementOperator {
    operator_id: String,
    temporal_support: String,
    noise_model: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BaselineWindow {
    start_hours: f64,
    end_hours: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TruthPoint {
    elapsed_hours: f64,
    response_value: f64,
    forcing_value: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Event {
    event_id: String,
    event_type: String,
    start_hours: f64,
    end_hours: f64,
    minimum_detectable_absolute_departure: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    policy_submission_sha256: String,
    evaluator_truth_sha256: String,
    policy_count: usize,
    policies: Vec<PolicyResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct PolicyResult {
    policy_id: String,
    policy_family: String,
    policy_schedule_sha256: String,
    sample_count: usize,
    sample_cost_units: f64,
    sample_budget_pass: bool,
    out_of_event_sample_count: usize,
    false_trigger_count: usize,
    events_detected: usize,
    event_count: usize,
    missed_event_count: usize,
    events: Vec<EventResult>,
}

#[derive(Debug, Serialize)]
struct EventResult {
    event_id: String,
    event_detected: bool,
    detection_time_hours: Option<f64>,
    detection_delay_hours: Option<f64>,
    true_peak_value: f64,
    true_peak_time_hours: f64,
    sampled_peak_value: Option<f64>,
    sampled_peak_time_hours: Option<f64>,
    absolute_peak_error: Option<f64>,
    true_pulse_burden_anomaly_unit_hours: f64,
    sampled_pulse_burden_anomaly_unit_hours: Option<f64>,
    absolute_pulse_burden_error: Option<f64>,
    relative_pulse_burden_error: Option<f64>,
    event_sample_count: usize,
    sampled_max_inter_observation_gap_hours: Option<f64>,
    pulse_burden_status: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    policy_input_has_no_truth_values: bool,
    exact_timestamp_sampling_only: bool,
    no_interpolation_or_nearest_neighbor: bool,
    separate_policy_and_evaluator_digests: bool,
    no_aggregate_policy_score: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn validate_policy_submission(p: &PolicySubmission) -> Result<(), String> {
    if p.schema_version != "event-capture-policy-v0.1"
        || p.fixture_status != POLICY_STATUS
        || p.evidence_class != "synthetic"
    {
        return Err("unsupported policy submission version or evidence class".into());
    }
    if p.submission_id.trim().is_empty()
        || p.policy_resource_limits.maximum_samples_per_policy < 2
        || !p.policy_resource_limits.sample_cost_units.is_finite()
        || p.policy_resource_limits.sample_cost_units <= 0.0
        || p.policies.is_empty()
    {
        return Err("policy submission identity, limits or schedule set is invalid".into());
    }
    let mut ids = BTreeSet::new();
    for policy in &p.policies {
        if policy.policy_id.trim().is_empty()
            || policy.policy_version.trim().is_empty()
            || !ids.insert(policy.policy_id.clone())
        {
            return Err("policy IDs/version must be present and IDs unique".into());
        }
        if !matches!(
            policy.policy_family.as_str(),
            "fixed_cadence"
                | "random"
                | "distribution_sampling"
                | "uncertainty_triggered"
                | "threshold_triggered"
                | "event_triggered"
                | "entropy_eig"
                | "decision_evsi"
                | "hindsight_oracle"
        ) {
            return Err(format!(
                "policy {} has unsupported family",
                policy.policy_id
            ));
        }
        if !valid_sha256(&policy.policy_code_sha256) || !valid_sha256(&policy.policy_config_sha256)
        {
            return Err(format!(
                "policy {} has invalid code/config digest",
                policy.policy_id
            ));
        }
        if policy.requested_times_hours.len() < 2
            || policy.requested_times_hours.len()
                > p.policy_resource_limits.maximum_samples_per_policy
        {
            return Err(format!(
                "policy {} violates sample-count budget",
                policy.policy_id
            ));
        }
        let mut prior: Option<f64> = None;
        for time in &policy.requested_times_hours {
            finite("requested sample timestamp", *time)?;
            if let Some(previous) = prior
                && *time <= previous
            {
                return Err(format!(
                    "policy {} timestamps must increase strictly",
                    policy.policy_id
                ));
            }
            prior = Some(*time);
        }
    }
    Ok(())
}

fn validate_truth(t: &Truth) -> Result<(), String> {
    if t.schema_version != "event-capture-truth-v0.1"
        || t.fixture_status != TRUTH_STATUS
        || t.evidence_class != "synthetic"
    {
        return Err("unsupported evaluator truth version or evidence class".into());
    }
    if t.scenario_id.trim().is_empty()
        || t.response_variable.trim().is_empty()
        || t.response_unit.trim().is_empty()
        || t.measurement_operator.operator_id.trim().is_empty()
        || t.measurement_operator.temporal_support != "point_sample_at_exact_timestamp"
        || t.measurement_operator.noise_model != "zero_noise_manufactured_truth"
        || !t.baseline_window.start_hours.is_finite()
        || !t.baseline_window.end_hours.is_finite()
        || t.baseline_window.start_hours >= t.baseline_window.end_hours
        || !t.maximum_event_integration_gap_hours.is_finite()
        || t.maximum_event_integration_gap_hours <= 0.0
        || t.truth_series.len() < 3
        || t.events.is_empty()
    {
        return Err("evaluator truth metadata/windows are invalid".into());
    }
    let mut previous: Option<f64> = None;
    for point in &t.truth_series {
        finite("truth timestamp", point.elapsed_hours)?;
        finite("truth response", point.response_value)?;
        finite("truth forcing", point.forcing_value)?;
        if let Some(prior) = previous
            && point.elapsed_hours <= prior
        {
            return Err("truth series timestamps must increase strictly".into());
        }
        previous = Some(point.elapsed_hours);
    }
    if t.baseline_window.start_hours < t.truth_series[0].elapsed_hours
        || t.baseline_window.end_hours > t.truth_series.last().unwrap().elapsed_hours
        || !t
            .truth_series
            .iter()
            .any(|x| x.elapsed_hours == t.baseline_window.start_hours)
        || !t
            .truth_series
            .iter()
            .any(|x| x.elapsed_hours == t.baseline_window.end_hours)
    {
        return Err("baseline window must be inside and aligned with truth timestamps".into());
    }
    let baseline_points: Vec<&TruthPoint> = t
        .truth_series
        .iter()
        .filter(|x| {
            x.elapsed_hours >= t.baseline_window.start_hours
                && x.elapsed_hours <= t.baseline_window.end_hours
        })
        .collect();
    if baseline_points.len() < 2 {
        return Err("baseline window needs at least two truth points".into());
    }
    let mut event_ids = BTreeSet::new();
    for event in &t.events {
        finite("event start", event.start_hours)?;
        finite("event end", event.end_hours)?;
        finite(
            "event detection threshold",
            event.minimum_detectable_absolute_departure,
        )?;
        if event.event_id.trim().is_empty()
            || !event_ids.insert(event.event_id.clone())
            || event.start_hours >= event.end_hours
            || event.minimum_detectable_absolute_departure <= 0.0
            || !t
                .truth_series
                .iter()
                .any(|x| x.elapsed_hours == event.start_hours)
            || !t
                .truth_series
                .iter()
                .any(|x| x.elapsed_hours == event.end_hours)
        {
            return Err(format!(
                "event {} has invalid identity, boundaries or threshold",
                event.event_id
            ));
        }
    }
    Ok(())
}

fn truth_at<'a>(truth: &'a Truth, timestamp: f64) -> Option<&'a TruthPoint> {
    truth
        .truth_series
        .iter()
        .find(|x| x.elapsed_hours == timestamp)
}

fn baseline_mean(truth: &Truth) -> f64 {
    let points: Vec<&TruthPoint> = truth
        .truth_series
        .iter()
        .filter(|x| {
            x.elapsed_hours >= truth.baseline_window.start_hours
                && x.elapsed_hours <= truth.baseline_window.end_hours
        })
        .collect();
    points.iter().map(|x| x.response_value).sum::<f64>() / points.len() as f64
}

fn integrate_anomaly(points: &[&TruthPoint], baseline: f64) -> Option<f64> {
    if points.len() < 2 {
        return None;
    }
    let mut area = 0.0;
    for pair in points.windows(2) {
        let dt = pair[1].elapsed_hours - pair[0].elapsed_hours;
        let a = (pair[0].response_value - baseline).abs();
        let b = (pair[1].response_value - baseline).abs();
        area += 0.5 * (a + b) * dt;
    }
    Some(area)
}

fn truth_event_metrics(truth: &Truth, event: &Event, baseline: f64) -> (f64, f64, f64) {
    let points: Vec<&TruthPoint> = truth
        .truth_series
        .iter()
        .filter(|x| x.elapsed_hours >= event.start_hours && x.elapsed_hours <= event.end_hours)
        .collect();
    let peak = points
        .iter()
        .copied()
        .max_by(|a, b| {
            (a.response_value - baseline)
                .abs()
                .total_cmp(&(b.response_value - baseline).abs())
        })
        .expect("validated event has boundary truth points");
    let integral = integrate_anomaly(&points, baseline).expect("validated event has endpoints");
    (peak.response_value, peak.elapsed_hours, integral)
}

fn event_result(
    truth: &Truth,
    event: &Event,
    requested_times: &[f64],
    baseline: f64,
) -> EventResult {
    let sampled: Vec<&TruthPoint> = requested_times
        .iter()
        .filter(|time| **time >= event.start_hours && **time <= event.end_hours)
        .filter_map(|time| truth_at(truth, *time))
        .collect();
    let (true_peak_value, true_peak_time_hours, true_integral) =
        truth_event_metrics(truth, event, baseline);
    let detected = sampled.iter().any(|x| {
        (x.response_value - baseline).abs() >= event.minimum_detectable_absolute_departure
    });
    let first_detected = sampled.iter().find(|x| {
        (x.response_value - baseline).abs() >= event.minimum_detectable_absolute_departure
    });
    let sampled_peak = sampled.iter().copied().max_by(|a, b| {
        (a.response_value - baseline)
            .abs()
            .total_cmp(&(b.response_value - baseline).abs())
    });
    let (sampled_integral, integral_status, max_gap) =
        if !sampled.iter().any(|x| x.elapsed_hours == event.start_hours)
            || !sampled.iter().any(|x| x.elapsed_hours == event.end_hours)
        {
            (None, "missing_event_boundary", None)
        } else {
            let gaps: Vec<f64> = sampled
                .windows(2)
                .map(|pair| pair[1].elapsed_hours - pair[0].elapsed_hours)
                .collect();
            let largest = gaps.iter().copied().max_by(f64::total_cmp);
            if largest.is_some_and(|x| x > truth.maximum_event_integration_gap_hours) {
                (None, "gap_exceeded", largest)
            } else {
                (
                    integrate_anomaly(&sampled, baseline),
                    "qualified_integral",
                    largest,
                )
            }
        };
    let (abs_integral_error, relative_integral_error) =
        sampled_integral.map_or((None, None), |estimate| {
            let error = (estimate - true_integral).abs();
            (
                Some(error),
                (true_integral > f64::EPSILON).then_some(error / true_integral),
            )
        });
    let (sampled_peak_value, sampled_peak_time, peak_error) =
        sampled_peak.map_or((None, None, None), |peak| {
            (
                Some(peak.response_value),
                Some(peak.elapsed_hours),
                Some((peak.response_value - true_peak_value).abs()),
            )
        });
    EventResult {
        event_id: event.event_id.clone(),
        event_detected: detected,
        detection_time_hours: first_detected.map(|x| x.elapsed_hours),
        detection_delay_hours: first_detected.map(|x| x.elapsed_hours - event.start_hours),
        true_peak_value,
        true_peak_time_hours,
        sampled_peak_value,
        sampled_peak_time_hours: sampled_peak_time,
        absolute_peak_error: peak_error,
        true_pulse_burden_anomaly_unit_hours: true_integral,
        sampled_pulse_burden_anomaly_unit_hours: sampled_integral,
        absolute_pulse_burden_error: abs_integral_error,
        relative_pulse_burden_error,
        event_sample_count: sampled.len(),
        sampled_max_inter_observation_gap_hours: max_gap,
        pulse_burden_status: integral_status,
    }
}

fn score(
    submission: &PolicySubmission,
    truth: &Truth,
    policy_submission_sha256: String,
    evaluator_truth_sha256: String,
) -> Report {
    let baseline = baseline_mean(truth);
    let events_all: BTreeSet<String> = truth.events.iter().map(|x| x.event_id.clone()).collect();
    let policies = submission
        .policies
        .iter()
        .map(|policy| {
            let schedule = serde_json::to_vec(policy).expect("serializable policy");
            let schedule_hash = format!("{:x}", Sha256::digest(schedule));
            let sampled: Vec<&TruthPoint> = policy
                .requested_times_hours
                .iter()
                .filter_map(|time| truth_at(truth, *time))
                .collect();
            let in_event_times: BTreeSet<String> = truth
                .events
                .iter()
                .flat_map(|event| {
                    policy
                        .requested_times_hours
                        .iter()
                        .filter(|time| **time >= event.start_hours && **time <= event.end_hours)
                        .map(|time| format!("{time:.9}"))
                })
                .collect();
            let out_of_event_count = policy
                .requested_times_hours
                .iter()
                .filter(|time| !in_event_times.contains(&format!("{time:.9}")))
                .count();
            let false_trigger_count = sampled
                .iter()
                .filter(|point| {
                    !truth.events.iter().any(|event| {
                        point.elapsed_hours >= event.start_hours
                            && point.elapsed_hours <= event.end_hours
                    }) && (point.response_value - baseline).abs() >= 1.0
                })
                .count();
            let event_results: Vec<EventResult> = truth
                .events
                .iter()
                .map(|event| event_result(truth, event, &policy.requested_times_hours, baseline))
                .collect();
            let events_detected = event_results.iter().filter(|x| x.event_detected).count();
            let missed = event_results.len() - events_detected;
            PolicyResult {
                policy_id: policy.policy_id.clone(),
                policy_family: policy.policy_family.clone(),
                policy_schedule_sha256: schedule_hash,
                sample_count: policy.requested_times_hours.len(),
                sample_cost_units: policy.requested_times_hours.len() as f64
                    * submission.policy_resource_limits.sample_cost_units,
                sample_budget_pass: policy.requested_times_hours.len()
                    <= submission.policy_resource_limits.maximum_samples_per_policy,
                out_of_event_sample_count: out_of_event_count,
                false_trigger_count,
                events_detected,
                event_count: events_all.len(),
                missed_event_count: missed,
                events: event_results,
            }
        })
        .collect();

    Report {
        audit_version: VERSION,
        fixture_status: REPORT_STATUS,
        scenario_id: truth.scenario_id.clone(),
        policy_submission_sha256,
        evaluator_truth_sha256,
        policy_count: policies.len(),
        policies,
        controls: Controls {
            policy_input_has_no_truth_values: true,
            exact_timestamp_sampling_only: true,
            no_interpolation_or_nearest_neighbor: true,
            separate_policy_and_evaluator_digests: true,
            no_aggregate_policy_score: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(
    policy_path: &Path,
    truth_path: &Path,
    output_path: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let policy_bytes = fs::read(policy_path)?;
    let truth_bytes = fs::read(truth_path)?;
    let submission: PolicySubmission = serde_json::from_slice(&policy_bytes)?;
    let truth: Truth = serde_json::from_slice(&truth_bytes)?;
    validate_policy_submission(&submission).map_err(io::Error::other)?;
    validate_truth(&truth).map_err(io::Error::other)?;
    for policy in &submission.policies {
        for time in &policy.requested_times_hours {
            if truth_at(&truth, *time).is_none() {
                return Err(format!(
                    "policy {} requests {time}h outside exact truth timestamps; interpolation is prohibited",
                    policy.policy_id
                ).into());
            }
        }
    }
    let policy_hash = format!("{:x}", Sha256::digest(&policy_bytes));
    let truth_hash = format!("{:x}", Sha256::digest(&truth_bytes));
    let report = score(&submission, &truth, policy_hash, truth_hash);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        eprintln!(
            "usage: event-capture-audit <policy-submission.json> <evaluator-truth.json> <new-report.json>"
        );
        std::process::exit(2);
    }
    if let Err(error) = run(
        Path::new(&args[1]),
        Path::new(&args[2]),
        Path::new(&args[3]),
    ) {
        eprintln!("event-capture audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (PolicySubmission, Truth) {
        let policy: PolicySubmission =
            serde_json::from_str(include_str!("../fixtures/policy-submission.json"))
                .expect("policy fixture parses");
        let truth: Truth = serde_json::from_str(include_str!("../fixtures/evaluator-truth.json"))
            .expect("evaluator truth fixture parses");
        (policy, truth)
    }

    #[test]
    fn separate_fixtures_parse_and_validate() {
        let (policy, truth) = fixtures();
        validate_policy_submission(&policy).expect("policy input valid");
        validate_truth(&truth).expect("evaluator truth valid");
    }

    #[test]
    fn event_focused_schedule_detects_broad_and_narrow_pulses() {
        let (p, t) = fixtures();
        let report = score(&p, &t, "0".repeat(64), "1".repeat(64));
        let adaptive = report
            .policies
            .iter()
            .find(|x| x.policy_id == "event-focused-static")
            .unwrap();
        assert_eq!(adaptive.sample_count, 14);
        assert_eq!(adaptive.events_detected, 2);
        assert_eq!(adaptive.missed_event_count, 0);
        assert!(
            adaptive
                .events
                .iter()
                .all(|x| x.pulse_burden_status == "qualified_integral")
        );
    }

    #[test]
    fn fixed_cadence_misses_narrow_pulses_and_abstains_on_gap() {
        let (p, t) = fixtures();
        let report = score(&p, &t, "0".repeat(64), "1".repeat(64));
        let fixed = report
            .policies
            .iter()
            .find(|x| x.policy_id == "fixed-8h")
            .unwrap();
        assert_eq!(fixed.sample_count, 14);
        assert_eq!(fixed.events_detected, 0);
        assert_eq!(fixed.missed_event_count, 2);
        assert!(
            fixed
                .events
                .iter()
                .all(|x| x.sampled_pulse_burden_anomaly_unit_hours.is_none())
        );
        assert_eq!(fixed.events[0].pulse_burden_status, "gap_exceeded");
        assert_eq!(
            fixed.events[1].pulse_burden_status,
            "missing_event_boundary"
        );
    }

    #[test]
    fn random_schedule_is_scored_without_claiming_policy_superiority() {
        let (p, t) = fixtures();
        let report = score(&p, &t, "0".repeat(64), "1".repeat(64));
        let random = report
            .policies
            .iter()
            .find(|x| x.policy_id == "pseudo-random-fixed-seed")
            .unwrap();
        assert_eq!(random.sample_count, 14);
        assert_eq!(random.events.len(), 2);
    }

    #[test]
    fn all_schedules_are_matched_to_same_measurement_budget() {
        let (p, t) = fixtures();
        let report = score(&p, &t, "0".repeat(64), "1".repeat(64));
        assert!(report.policies.iter().all(|x| x.sample_count == 14));
        assert!(report.policies.iter().all(|x| x.sample_cost_units == 14.0));
    }

    #[test]
    fn off_grid_sample_request_is_rejected_without_interpolation() {
        let (mut p, t) = fixtures();
        p.policies[0].requested_times_hours[1] = 8.5;
        let exists = p.policies[0]
            .requested_times_hours
            .iter()
            .all(|x| truth_at(&t, *x).is_some());
        assert!(!exists);
    }

    #[test]
    fn missing_event_boundary_blocks_integral() {
        let (p, t) = fixtures();
        let mut altered = p;
        altered.policies[1]
            .requested_times_hours
            .retain(|x| *x != 76.0);
        let r = score(&altered, &t, "0".repeat(64), "1".repeat(64));
        let event_b = &r
            .policies
            .iter()
            .find(|x| x.policy_id == "event-focused-static")
            .unwrap()
            .events[1];
        assert_eq!(event_b.pulse_burden_status, "missing_event_boundary");
        assert!(event_b.sampled_pulse_burden_anomaly_unit_hours.is_none());
    }

    #[test]
    fn long_sampling_gap_blocks_pulse_integral() {
        let (p, mut t) = fixtures();
        t.maximum_event_integration_gap_hours = 1.0;
        let r = score(&p, &t, "0".repeat(64), "1".repeat(64));
        let broad = &r
            .policies
            .iter()
            .find(|x| x.policy_id == "event-focused-static")
            .unwrap()
            .events[0];
        assert_eq!(broad.pulse_burden_status, "gap_exceeded");
    }

    #[test]
    fn invalid_digest_is_rejected() {
        let (mut p, _) = fixtures();
        p.policies[0].policy_code_sha256 = "bad".into();
        assert!(validate_policy_submission(&p).is_err());
    }

    #[test]
    fn duplicate_policy_id_is_rejected() {
        let (mut p, _) = fixtures();
        p.policies[1].policy_id = p.policies[0].policy_id.clone();
        assert!(validate_policy_submission(&p).is_err());
    }

    #[test]
    fn invalid_event_boundaries_are_rejected() {
        let (_, mut t) = fixtures();
        t.events[0].end_hours = t.events[0].start_hours;
        assert!(validate_truth(&t).is_err());
    }
}

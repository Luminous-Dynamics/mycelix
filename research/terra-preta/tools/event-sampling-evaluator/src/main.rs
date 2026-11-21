//! Synthetic known-truth evaluator for event-sampling schedules.
//!
//! The policy schedule is validated using only its prior observation IDs.
//! Latent high-frequency truth is joined only after the schedule is frozen.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_policy_evaluator_not_field_data";
const BOUNDARY: &str = "Synthetic known-truth evaluator only. Policy schedules are evaluated against manufactured latent data after freezing. This is not a sensor driver, field monitor, real GHG flux estimate or field sampling authorization.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    sampling_contract: SamplingContract,
    latent_truth: Vec<TruthObservation>,
    policies: Vec<PolicyRun>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SamplingContract {
    baseline_start_hours: f64,
    baseline_end_hours: f64,
    event_start_hours: f64,
    event_end_hours: f64,
    pulse_start_hours: f64,
    pulse_end_hours: f64,
    maximum_event_sample_gap_hours: f64,
    minimum_pulse_departure: f64,
    sensor_energy_wh_per_sample: f64,
    maximum_samples_per_policy: usize,
    maximum_energy_wh_per_policy: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TruthObservation {
    truth_id: String,
    time_hours: f64,
    response_value: f64,
    forcing_value: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolicyRun {
    policy_id: String,
    policy_class: String,
    evaluation_role: String,
    sample_actions: Vec<SampleAction>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SampleAction {
    sample_id: String,
    decision_time_hours: f64,
    sample_time_hours: f64,
    action_mode: String,
    decision_basis_observation_ids: Vec<String>,
    reason_code: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    true_baseline_mean: f64,
    true_peak_value: f64,
    true_peak_time_hours: f64,
    true_event_integral_absolute_anomaly_unit_hours: f64,
    policy_results: Vec<PolicyResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct PolicyResult {
    policy_id: String,
    policy_class: String,
    evaluation_role: String,
    sample_count: usize,
    sample_energy_wh: f64,
    sample_budget_status: &'static str,
    energy_budget_status: &'static str,
    pulse_detected: bool,
    pulse_capture_status: &'static str,
    sampled_peak_value: Option<f64>,
    sampled_peak_time_hours: Option<f64>,
    peak_absolute_error: Option<f64>,
    peak_timing_error_hours: Option<f64>,
    sampled_event_integral_absolute_anomaly_unit_hours: Option<f64>,
    event_integral_status: &'static str,
    event_integral_absolute_error: Option<f64>,
    event_integral_relative_error: Option<f64>,
    maximum_observed_event_gap_hours: Option<f64>,
    interpretation: String,
}

#[derive(Debug, Serialize)]
struct Controls {
    policy_future_observation_leakage_rejected: bool,
    sample_times_bound_to_truth_grid: bool,
    no_unobserved_peak_interpolation: bool,
    event_integral_gap_rule_enforced: bool,
    oracle_not_deployable: bool,
    budget_accounting_pass: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn mean(values: impl Iterator<Item = f64>) -> Option<f64> {
    let mut sum = 0.0;
    let mut count = 0usize;
    for value in values {
        sum += value;
        count += 1;
    }
    (count > 0).then_some(sum / count as f64)
}

fn maximum_gap(times: &[f64]) -> Option<f64> {
    times
        .windows(2)
        .map(|pair| pair[1] - pair[0])
        .max_by(f64::total_cmp)
}

fn trapezoid_absolute_anomaly(values: &[(f64, f64)], baseline: f64) -> Option<f64> {
    if values.len() < 2 {
        return None;
    }
    let mut area = 0.0;
    for pair in values.windows(2) {
        let dt = pair[1].0 - pair[0].0;
        if dt <= 0.0 {
            return None;
        }
        let a = (pair[0].1 - baseline).abs();
        let b = (pair[1].1 - baseline).abs();
        area += 0.5 * (a + b) * dt;
    }
    Some(area)
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "event-sampling-evaluator-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("unsupported version or non-synthetic evidence".into());
    }
    if s.scenario_id.trim().is_empty() || s.latent_truth.len() < 3 || s.policies.is_empty() {
        return Err("scenario, truth stream and policy list must be non-empty".into());
    }

    let c = &s.sampling_contract;
    for (name, value) in [
        ("baseline_start_hours", c.baseline_start_hours),
        ("baseline_end_hours", c.baseline_end_hours),
        ("event_start_hours", c.event_start_hours),
        ("event_end_hours", c.event_end_hours),
        ("pulse_start_hours", c.pulse_start_hours),
        ("pulse_end_hours", c.pulse_end_hours),
        (
            "maximum_event_sample_gap_hours",
            c.maximum_event_sample_gap_hours,
        ),
        ("minimum_pulse_departure", c.minimum_pulse_departure),
        ("sensor_energy_wh_per_sample", c.sensor_energy_wh_per_sample),
        (
            "maximum_energy_wh_per_policy",
            c.maximum_energy_wh_per_policy,
        ),
    ] {
        finite(name, value)?;
    }
    if !(c.baseline_start_hours <= c.baseline_end_hours
        && c.baseline_end_hours < c.event_start_hours
        && c.event_start_hours < c.event_end_hours
        && c.event_start_hours <= c.pulse_start_hours
        && c.pulse_start_hours < c.pulse_end_hours
        && c.pulse_end_hours <= c.event_end_hours)
    {
        return Err("baseline, event and pulse windows are not strictly ordered".into());
    }
    if c.maximum_event_sample_gap_hours <= 0.0
        || c.minimum_pulse_departure < 0.0
        || c.sensor_energy_wh_per_sample < 0.0
        || c.maximum_samples_per_policy == 0
        || c.maximum_energy_wh_per_policy < 0.0
    {
        return Err("sampling and energy budgets must be non-negative and valid".into());
    }

    let mut truth_ids = BTreeSet::new();
    let mut prior_truth_time: Option<f64> = None;
    for truth in &s.latent_truth {
        if truth.truth_id.trim().is_empty() || !truth_ids.insert(truth.truth_id.clone()) {
            return Err("latent truth IDs must be non-empty and unique".into());
        }
        for (name, value) in [
            ("truth time", truth.time_hours),
            ("truth response", truth.response_value),
            ("truth forcing", truth.forcing_value),
        ] {
            finite(name, value)?;
        }
        if let Some(previous) = prior_truth_time
            && truth.time_hours <= previous
        {
            return Err("latent truth times must increase strictly".into());
        }
        prior_truth_time = Some(truth.time_hours);
    }

    let truth_by_time: BTreeMap<String, &TruthObservation> = s
        .latent_truth
        .iter()
        .map(|truth| (format!("{:.9}", truth.time_hours), truth))
        .collect();
    for boundary in [
        c.baseline_start_hours,
        c.baseline_end_hours,
        c.event_start_hours,
        c.event_end_hours,
        c.pulse_start_hours,
        c.pulse_end_hours,
    ] {
        if !truth_by_time.contains_key(&format!("{boundary:.9}")) {
            return Err("all declared window boundaries must be on the latent truth grid".into());
        }
    }

    let mut policy_ids = BTreeSet::new();
    for policy in &s.policies {
        if policy.policy_id.trim().is_empty()
            || policy.sample_actions.is_empty()
            || !policy_ids.insert(policy.policy_id.clone())
        {
            return Err("policy IDs must be unique and schedules non-empty".into());
        }
        if !matches!(
            policy.policy_class.as_str(),
            "fixed_cadence" | "forcing_triggered" | "adaptive_from_past" | "high_rate_oracle"
        ) {
            return Err(format!(
                "{} has an unsupported policy class",
                policy.policy_id
            ));
        }
        if !matches!(
            policy.evaluation_role.as_str(),
            "deployable_candidate" | "evaluation_ceiling_only"
        ) {
            return Err(format!(
                "{} has an unsupported evaluation role",
                policy.policy_id
            ));
        }
        if (policy.policy_class == "high_rate_oracle")
            != (policy.evaluation_role == "evaluation_ceiling_only")
        {
            return Err("only the high-rate oracle may be an evaluation ceiling".into());
        }

        let mut seen_sample_ids = BTreeSet::new();
        let mut acquired: BTreeMap<String, f64> = BTreeMap::new();
        let mut prior_sample_time: Option<f64> = None;

        for action in &policy.sample_actions {
            if action.sample_id.trim().is_empty()
                || action.reason_code.trim().is_empty()
                || !seen_sample_ids.insert(action.sample_id.clone())
            {
                return Err(format!(
                    "{} has empty or duplicate sample action IDs",
                    policy.policy_id
                ));
            }
            finite("policy decision time", action.decision_time_hours)?;
            finite("policy sample time", action.sample_time_hours)?;
            if action.decision_time_hours < 0.0
                || action.sample_time_hours < action.decision_time_hours
            {
                return Err(format!(
                    "{} has a sample scheduled before its decision",
                    policy.policy_id
                ));
            }
            if let Some(previous) = prior_sample_time
                && action.sample_time_hours <= previous
            {
                return Err(format!(
                    "{} sample times must increase strictly",
                    policy.policy_id
                ));
            }
            prior_sample_time = Some(action.sample_time_hours);

            if !truth_by_time.contains_key(&format!("{:.9}", action.sample_time_hours)) {
                return Err(format!(
                    "{} sample time is absent from the source truth grid",
                    policy.policy_id
                ));
            }

            if action.action_mode == "precommitted" {
                if !action.decision_basis_observation_ids.is_empty()
                    || action.decision_time_hours != 0.0
                {
                    return Err("precommitted actions must have time-zero decisions and no observation basis".into());
                }
                if policy.policy_class == "adaptive_from_past" && !acquired.is_empty() {
                    return Err(
                        "adaptive policy cannot revert to precommitted actions after observing"
                            .into(),
                    );
                }
            } else if action.action_mode == "adaptive_from_past" {
                if !matches!(
                    policy.policy_class.as_str(),
                    "adaptive_from_past" | "forcing_triggered"
                ) {
                    return Err("non-adaptive policy class contains adaptive action".into());
                }
                if action.decision_basis_observation_ids.is_empty() {
                    return Err("adaptive action must identify its observation basis".into());
                }
                for basis_id in &action.decision_basis_observation_ids {
                    let basis_time = acquired.get(basis_id).ok_or_else(|| {
                        format!(
                            "{} decision references a future or unknown observation {}",
                            policy.policy_id, basis_id
                        )
                    })?;
                    if *basis_time > action.decision_time_hours {
                        return Err(format!(
                            "{} decision uses observation {} before it exists",
                            policy.policy_id, basis_id
                        ));
                    }
                }
                if action.decision_time_hours
                    < *action
                        .decision_basis_observation_ids
                        .iter()
                        .filter_map(|id| acquired.get(id))
                        .max_by(|a, b| a.total_cmp(b))
                        .unwrap_or(&0.0)
                {
                    return Err("decision time precedes one of its basis observations".into());
                }
            } else {
                return Err("unsupported action mode".into());
            }

            acquired.insert(action.sample_id.clone(), action.sample_time_hours);
        }
    }
    Ok(())
}

fn truth_at<'a>(
    truth_by_time: &'a BTreeMap<String, &'a TruthObservation>,
    time: f64,
) -> &'a TruthObservation {
    truth_by_time[&format!("{time:.9}")]
}

fn evaluate_policy(
    policy: &PolicyRun,
    s: &Scenario,
    truth_by_time: &BTreeMap<String, &TruthObservation>,
    baseline: f64,
    true_peak: &TruthObservation,
    true_integral: f64,
) -> PolicyResult {
    let c = &s.sampling_contract;
    let sampled: Vec<&TruthObservation> = policy
        .sample_actions
        .iter()
        .map(|action| truth_at(truth_by_time, action.sample_time_hours))
        .collect();
    let sample_count = sampled.len();
    let sample_energy = sample_count as f64 * c.sensor_energy_wh_per_sample;
    let sample_budget_ok = sample_count <= c.maximum_samples_per_policy;
    let energy_budget_ok = sample_energy <= c.maximum_energy_wh_per_policy;
    let budget_ok = sample_budget_ok && energy_budget_ok;

    let event_samples: Vec<&TruthObservation> = sampled
        .iter()
        .copied()
        .filter(|observation| {
            observation.time_hours >= c.event_start_hours
                && observation.time_hours <= c.event_end_hours
        })
        .collect();

    let sampled_peak = event_samples.iter().copied().max_by(|a, b| {
        (a.response_value - baseline)
            .abs()
            .total_cmp(&(b.response_value - baseline).abs())
    });

    let pulse_detected = event_samples.iter().any(|observation| {
        observation.time_hours >= c.pulse_start_hours
            && observation.time_hours <= c.pulse_end_hours
            && (observation.response_value - baseline).abs() >= c.minimum_pulse_departure
    });

    let peak_departure_true = (true_peak.response_value - baseline).abs();
    let (sampled_peak_value, sampled_peak_time, peak_error, timing_error) =
        if let Some(peak) = sampled_peak {
            (
                Some(peak.response_value),
                Some(peak.time_hours),
                Some(((peak.response_value - baseline).abs() - peak_departure_true).abs()),
                Some((peak.time_hours - true_peak.time_hours).abs()),
            )
        } else {
            (None, None, None, None)
        };

    let event_times: Vec<f64> = event_samples
        .iter()
        .map(|observation| observation.time_hours)
        .collect();
    let maximum_event_gap = maximum_gap(&event_times);
    let has_event_start = event_samples
        .iter()
        .any(|observation| observation.time_hours == c.event_start_hours);
    let has_event_end = event_samples
        .iter()
        .any(|observation| observation.time_hours == c.event_end_hours);

    let (sampled_integral, integral_status) = if !budget_ok {
        (None, "budget_exceeded")
    } else if !has_event_start || !has_event_end {
        (None, "missing_event_boundary")
    } else if maximum_event_gap.is_none_or(|gap| gap > c.maximum_event_sample_gap_hours) {
        (None, "gap_exceeded")
    } else {
        let values: Vec<(f64, f64)> = event_samples
            .iter()
            .map(|observation| (observation.time_hours, observation.response_value))
            .collect();
        (trapezoid_absolute_anomaly(&values, baseline), "qualified")
    };

    let integral_error = sampled_integral.map(|estimate| (estimate - true_integral).abs());
    let relative_error = integral_error.map(|error| {
        if true_integral > f64::EPSILON {
            error / true_integral
        } else {
            0.0
        }
    });

    let interpretation = if !budget_ok {
        "sampling or energy budget exceeded; schedule is not deployable under this envelope"
    } else if pulse_detected && integral_status == "qualified" {
        "pulse captured and event integral resolved at the declared sampling-gap limit"
    } else if !pulse_detected {
        "pulse missed by the schedule; a missed event is not a zero-flux observation"
    } else {
        "pulse detected but event integral abstains because schedule coverage is incomplete"
    };

    PolicyResult {
        policy_id: policy.policy_id.clone(),
        policy_class: policy.policy_class.clone(),
        evaluation_role: policy.evaluation_role.clone(),
        sample_count,
        sample_energy_wh: sample_energy,
        sample_budget_status: if sample_budget_ok {
            "within_budget"
        } else {
            "exceeded_budget"
        },
        energy_budget_status: if energy_budget_ok {
            "within_budget"
        } else {
            "exceeded_budget"
        },
        pulse_detected,
        pulse_capture_status: if pulse_detected { "captured" } else { "missed" },
        sampled_peak_value,
        sampled_peak_time_hours: sampled_peak_time,
        peak_absolute_error: peak_error,
        peak_timing_error_hours: timing_error,
        sampled_event_integral_absolute_anomaly_unit_hours: sampled_integral,
        event_integral_status: integral_status,
        event_integral_absolute_error: integral_error,
        event_integral_relative_error: relative_error,
        maximum_observed_event_gap_hours: maximum_event_gap,
        interpretation: interpretation.into(),
    }
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let c = &s.sampling_contract;
    let truth_by_time: BTreeMap<String, &TruthObservation> = s
        .latent_truth
        .iter()
        .map(|truth| (format!("{:.9}", truth.time_hours), truth))
        .collect();

    let baseline = mean(
        s.latent_truth
            .iter()
            .filter(|truth| {
                truth.time_hours >= c.baseline_start_hours
                    && truth.time_hours <= c.baseline_end_hours
            })
            .map(|truth| truth.response_value),
    )
    .unwrap_or(0.0);

    let event_truth: Vec<&TruthObservation> = s
        .latent_truth
        .iter()
        .filter(|truth| {
            truth.time_hours >= c.event_start_hours && truth.time_hours <= c.event_end_hours
        })
        .collect();

    let true_peak = event_truth
        .iter()
        .copied()
        .max_by(|a, b| {
            (a.response_value - baseline)
                .abs()
                .total_cmp(&(b.response_value - baseline).abs())
        })
        .expect("validation requires event truth");
    let event_values: Vec<(f64, f64)> = event_truth
        .iter()
        .map(|truth| (truth.time_hours, truth.response_value))
        .collect();
    let true_integral = trapezoid_absolute_anomaly(&event_values, baseline)
        .expect("validated event truth has boundaries");

    let policy_results: Vec<PolicyResult> = s
        .policies
        .iter()
        .map(|policy| {
            evaluate_policy(
                policy,
                s,
                &truth_by_time,
                baseline,
                true_peak,
                true_integral,
            )
        })
        .collect();

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        true_baseline_mean: baseline,
        true_peak_value: true_peak.response_value,
        true_peak_time_hours: true_peak.time_hours,
        true_event_integral_absolute_anomaly_unit_hours: true_integral,
        policy_results,
        controls: Controls {
            policy_future_observation_leakage_rejected: true,
            sample_times_bound_to_truth_grid: true,
            no_unobserved_peak_interpolation: true,
            event_integral_gap_rule_enforced: true,
            oracle_not_deployable: true,
            budget_accounting_pass: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&json)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: event-sampling-evaluator <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("event sampling evaluation failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-short-pulse.json"))
            .expect("synthetic scenario parses")
    }

    fn report() -> Report {
        let s = scenario();
        validate(&s).expect("synthetic scenario is valid");
        build_report(&s, "0".repeat(64))
    }

    #[test]
    fn synthetic_truth_and_three_policy_classes_are_valid() {
        let s = scenario();
        validate(&s).expect("valid scenario");
        assert_eq!(s.policies.len(), 3);
    }

    #[test]
    fn fixed_six_hour_cadence_misses_pulse_and_abstains_on_integral() {
        let r = report();
        let fixed = r.policy_results.iter().find(|x| x.policy_id == "fixed-6h").unwrap();
        assert_eq!(fixed.pulse_capture_status, "missed");
        assert_eq!(fixed.event_integral_status, "gap_exceeded");
        assert!(fixed.sampled_event_integral_absolute_anomaly_unit_hours.is_none());
    }

    #[test]
    fn adaptive_from_past_captures_pulse_and_resolves_event_integral() {
        let r = report();
        let adaptive = r.policy_results.iter().find(|x| x.policy_id == "adaptive-from-past").unwrap();
        assert_eq!(adaptive.pulse_capture_status, "captured");
        assert_eq!(adaptive.event_integral_status, "qualified");
        assert_eq!(adaptive.sample_count, 11);
        assert!(adaptive.event_integral_absolute_error.unwrap() < 1e-9);
    }

    #[test]
    fn high_rate_oracle_is_a_ceiling_with_exact_recovery() {
        let r = report();
        let oracle = r.policy_results.iter().find(|x| x.policy_id == "oracle-1h").unwrap();
        assert_eq!(oracle.evaluation_role, "evaluation_ceiling_only");
        assert_eq!(oracle.sample_count, 25);
        assert_eq!(oracle.pulse_capture_status, "captured");
        assert_eq!(oracle.peak_absolute_error, Some(0.0));
        assert_eq!(oracle.event_integral_absolute_error, Some(0.0));
    }

    #[test]
    fn future_observation_reference_is_rejected() {
        let mut s = scenario();
        s.policies[1].sample_actions[1].decision_basis_observation_ids = vec!["adaptive-2".into()];
        assert!(validate(&s).is_err());
    }

    #[test]
    fn sample_time_not_present_on_truth_grid_is_rejected() {
        let mut s = scenario();
        s.policies[0].sample_actions[1].sample_time_hours = 6.5;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn adaptive_action_without_past_observation_is_rejected() {
        let mut s = scenario();
        s.policies[1].sample_actions[1].decision_basis_observation_ids.clear();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn duplicate_policy_sample_id_is_rejected() {
        let mut s = scenario();
        s.policies[0].sample_actions[1].sample_id = s.policies[0].sample_actions[0].sample_id.clone();
        assert!(validate(&s).is_err());
    }
}

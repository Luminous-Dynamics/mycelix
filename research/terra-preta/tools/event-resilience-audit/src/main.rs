//! Synthetic-only event-resolved soil resilience auditor.
//!
//! Computes descriptive peak departure, recovery and pulse burden for one
//! experimental unit. It never bridges long observation gaps or estimates a
//! causal treatment effect. Serial-covariance uncertainty is not propagated.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const UNCERTAINTY_STATUS: &str = "not_propagated_serial_covariance_unavailable";
const BOUNDARY: &str = "Synthetic descriptive event audit only. No causal treatment effect, independent replication, mechanistic attribution or field resilience qualification is estimated. Serial covariance and shared calibration uncertainty are not propagated.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    config: Config,
    event: Event,
    response: Variable,
    forcing: Variable,
    recovery_tolerance: RecoveryTolerance,
    observations: Vec<Observation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    maximum_inter_observation_gap_hours: f64,
    minimum_baseline_observations: usize,
    minimum_stress_observations: usize,
    minimum_recovery_observations: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Event {
    event_id: String,
    event_type: String,
    experimental_unit_id: String,
    independence_group_id: String,
    baseline_start_hours: f64,
    event_start_hours: f64,
    stress_end_hours: f64,
    recovery_end_hours: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Variable {
    variable: String,
    unit: String,
    measurement_method: String,
    support_description: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RecoveryTolerance {
    status: String,
    absolute_deviation_tolerance: Option<f64>,
    required_consecutive_observations: Option<usize>,
    authority: Option<String>,
    approval_reference: Option<String>,
    approval_date: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    observation_id: String,
    elapsed_hours: f64,
    phase: String,
    experimental_unit_id: String,
    response_value: f64,
    response_standard_error: f64,
    forcing_value: f64,
    forcing_standard_error: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    event_id: String,
    event_type: String,
    experimental_unit_id: String,
    response_variable: String,
    response_unit: String,
    response_measurement_method: String,
    response_support_description: String,
    forcing_variable: String,
    forcing_unit: String,
    analysis_status: &'static str,
    phase_counts: PhaseCounts,
    baseline_mean: Option<f64>,
    stress_peak_value: Option<f64>,
    stress_peak_time_hours: Option<f64>,
    stress_peak_departure: Option<f64>,
    stress_peak_absolute_departure: Option<f64>,
    recovery_mean: Option<f64>,
    recovery_departure: Option<f64>,
    recovery_fraction: Option<f64>,
    recovery_time_from_stress_end_hours: Option<f64>,
    recovery_time_status: &'static str,
    pulse_burden_absolute_anomaly_unit_hours: Option<f64>,
    pulse_burden_positive_anomaly_unit_hours: Option<f64>,
    pulse_burden_negative_anomaly_unit_hours: Option<f64>,
    pulse_burden_signed_anomaly_unit_hours: Option<f64>,
    pulse_burden_status: &'static str,
    maximum_inter_observation_gap_hours: f64,
    baseline_max_gap_hours: Option<f64>,
    stress_max_gap_hours: Option<f64>,
    recovery_max_gap_hours: Option<f64>,
    max_observed_gap_hours: Option<f64>,
    uncertainty_status: &'static str,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct PhaseCounts {
    baseline: usize,
    stress: usize,
    recovery: usize,
}

#[derive(Debug, Serialize)]
struct Controls {
    same_experimental_unit_pass: bool,
    strict_time_order_pass: bool,
    phase_window_consistency_pass: bool,
    no_gap_bridging_pass: bool,
    no_causal_claim_pass: bool,
    no_aggregate_resilience_score_pass: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn valid_date(date: &str) -> bool {
    date.len() == 10
        && date.as_bytes()[4] == b'-'
        && date.as_bytes()[7] == b'-'
        && date
            .bytes()
            .enumerate()
            .all(|(i, byte)| i == 4 || i == 7 || byte.is_ascii_digit())
        && date[0..4]
            .parse::<u32>()
            .is_ok_and(|x| (1..=9999).contains(&x))
        && date[5..7]
            .parse::<u32>()
            .is_ok_and(|x| (1..=12).contains(&x))
        && date[8..10]
            .parse::<u32>()
            .is_ok_and(|x| (1..=31).contains(&x))
}

fn expected_phase(event: &Event, time: f64) -> Option<&'static str> {
    if time >= event.baseline_start_hours && time <= event.event_start_hours {
        Some("baseline")
    } else if time > event.event_start_hours && time <= event.stress_end_hours {
        Some("stress")
    } else if time > event.stress_end_hours && time <= event.recovery_end_hours {
        Some("recovery")
    } else {
        None
    }
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "event-resilience-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic schema/version".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.event.event_id.trim().is_empty()
        || s.event.experimental_unit_id.trim().is_empty()
        || s.event.independence_group_id.trim().is_empty()
        || s.response.variable.trim().is_empty()
        || s.response.unit.trim().is_empty()
        || s.response.measurement_method.trim().is_empty()
        || s.response
            .support_description
            .as_deref()
            .unwrap_or("")
            .trim()
            .is_empty()
        || s.forcing.variable.trim().is_empty()
        || s.forcing.unit.trim().is_empty()
        || s.forcing.measurement_method.trim().is_empty()
    {
        return Err(
            "scenario, event, variable, method, unit and unit-group metadata must be present"
                .into(),
        );
    }
    if !matches!(
        s.event.event_type.as_str(),
        "drought_rewetting"
            | "heat_stress"
            | "intense_rainfall"
            | "flooding"
            | "wet_dry_cycle"
            | "compound_stress"
            | "other"
    ) {
        return Err("unsupported event type".into());
    }
    for (name, value) in [
        (
            "maximum_inter_observation_gap_hours",
            s.config.maximum_inter_observation_gap_hours,
        ),
        ("baseline_start_hours", s.event.baseline_start_hours),
        ("event_start_hours", s.event.event_start_hours),
        ("stress_end_hours", s.event.stress_end_hours),
        ("recovery_end_hours", s.event.recovery_end_hours),
    ] {
        finite(name, value)?;
    }
    if s.config.maximum_inter_observation_gap_hours <= 0.0
        || s.config.minimum_baseline_observations < 2
        || s.config.minimum_stress_observations < 2
        || s.config.minimum_recovery_observations < 2
    {
        return Err("maximum gap and minimum phase counts must be positive/valid".into());
    }
    if !(s.event.baseline_start_hours < s.event.event_start_hours
        && s.event.event_start_hours < s.event.stress_end_hours
        && s.event.stress_end_hours < s.event.recovery_end_hours)
    {
        return Err("event windows must be strictly ordered".into());
    }
    validate_tolerance(&s.recovery_tolerance)?;

    if s.observations.len() < 2 {
        return Err("at least two observations are required".into());
    }
    let mut ids = BTreeSet::new();
    let mut prior_time: Option<f64> = None;
    for observation in &s.observations {
        if observation.observation_id.trim().is_empty()
            || !ids.insert(observation.observation_id.clone())
        {
            return Err("observation IDs must be non-empty and unique".into());
        }
        for (name, value) in [
            ("elapsed_hours", observation.elapsed_hours),
            ("response_value", observation.response_value),
            (
                "response_standard_error",
                observation.response_standard_error,
            ),
            ("forcing_value", observation.forcing_value),
            ("forcing_standard_error", observation.forcing_standard_error),
        ] {
            finite(name, value)?;
        }
        if observation.response_standard_error < 0.0 || observation.forcing_standard_error < 0.0 {
            return Err("observation standard errors cannot be negative".into());
        }
        if observation.experimental_unit_id != s.event.experimental_unit_id {
            return Err("all observations must belong to the declared experimental unit".into());
        }
        if let Some(previous) = prior_time
            && observation.elapsed_hours <= previous
        {
            return Err("observation times must increase strictly".into());
        }
        prior_time = Some(observation.elapsed_hours);
        let expected = expected_phase(&s.event, observation.elapsed_hours)
            .ok_or_else(|| "observation lies outside declared event windows".to_string())?;
        if observation.phase != expected {
            return Err(format!(
                "observation {} phase does not match its declared time window",
                observation.observation_id
            ));
        }
    }
    Ok(())
}

fn validate_tolerance(t: &RecoveryTolerance) -> Result<(), String> {
    match t.status.as_str() {
        "not_set" => {
            if t.absolute_deviation_tolerance.is_some()
                || t.required_consecutive_observations.is_some()
                || t.authority.is_some()
                || t.approval_reference.is_some()
                || t.approval_date.is_some()
            {
                return Err(
                    "not_set recovery tolerance cannot carry threshold/approval fields".into(),
                );
            }
        }
        "synthetic_fixture" | "externally_approved" => {
            let tolerance = t
                .absolute_deviation_tolerance
                .ok_or_else(|| "recovery tolerance needs a deviation bound".to_string())?;
            let count = t
                .required_consecutive_observations
                .ok_or_else(|| "recovery tolerance needs consecutive count".to_string())?;
            finite("absolute_deviation_tolerance", tolerance)?;
            if tolerance < 0.0 || count < 2 {
                return Err("recovery tolerance or consecutive count is invalid".into());
            }
            if t.status == "externally_approved"
                && (t.authority.as_deref().unwrap_or("").trim().is_empty()
                    || t.approval_reference
                        .as_deref()
                        .unwrap_or("")
                        .trim()
                        .is_empty()
                    || !t.approval_date.as_deref().is_some_and(valid_date))
            {
                return Err(
                    "externally approved recovery tolerance lacks approval evidence".into(),
                );
            }
        }
        _ => return Err("unsupported recovery tolerance status".into()),
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

fn phase_counts(observations: &[Observation]) -> PhaseCounts {
    PhaseCounts {
        baseline: observations
            .iter()
            .filter(|o| o.phase == "baseline")
            .count(),
        stress: observations.iter().filter(|o| o.phase == "stress").count(),
        recovery: observations
            .iter()
            .filter(|o| o.phase == "recovery")
            .count(),
    }
}

fn max_gap(observations: &[Observation]) -> Option<f64> {
    observations
        .windows(2)
        .map(|pair| pair[1].elapsed_hours - pair[0].elapsed_hours)
        .max_by(f64::total_cmp)
}

fn phase_max_gap(observations: &[Observation], phase: &str) -> Option<f64> {
    let phase_observations: Vec<&Observation> =
        observations.iter().filter(|o| o.phase == phase).collect();
    phase_observations
        .windows(2)
        .map(|pair| pair[1].elapsed_hours - pair[0].elapsed_hours)
        .max_by(f64::total_cmp)
}

#[derive(Debug, Default)]
struct PulseBurden {
    absolute: Option<f64>,
    positive: Option<f64>,
    negative: Option<f64>,
    signed: Option<f64>,
    status: &'static str,
}

fn compute_recovery_time(
    s: &Scenario,
    baseline: f64,
    recovery: &[&Observation],
) -> (Option<f64>, &'static str) {
    if s.recovery_tolerance.status == "not_set" {
        return (None, "threshold_not_set");
    }
    let tolerance = s
        .recovery_tolerance
        .absolute_deviation_tolerance
        .unwrap_or(0.0);
    let required = s
        .recovery_tolerance
        .required_consecutive_observations
        .unwrap_or(usize::MAX);
    for start in 0..recovery.len() {
        let end = start.saturating_add(required);
        if end > recovery.len() {
            break;
        }
        let window = &recovery[start..end];
        let inside = window
            .iter()
            .all(|o| (o.response_value - baseline).abs() <= tolerance)
            && window.windows(2).all(|pair| {
                pair[1].elapsed_hours - pair[0].elapsed_hours
                    <= s.config.maximum_inter_observation_gap_hours
            });
        if inside {
            return (
                Some(window[0].elapsed_hours - s.event.stress_end_hours),
                "qualified_sustained_recovery",
            );
        }
    }
    (None, "no_sustained_recovery_observed")
}

fn pulse_burden(s: &Scenario, baseline: f64) -> PulseBurden {
    let onset = s
        .observations
        .iter()
        .any(|o| o.elapsed_hours == s.event.event_start_hours);
    let end = s
        .observations
        .iter()
        .any(|o| o.elapsed_hours == s.event.recovery_end_hours);
    if !onset || !end {
        return PulseBurden {
            status: "missing_event_boundary",
            ..PulseBurden::default()
        };
    }
    let series: Vec<&Observation> = s
        .observations
        .iter()
        .filter(|o| {
            o.elapsed_hours >= s.event.event_start_hours
                && o.elapsed_hours <= s.event.recovery_end_hours
        })
        .collect();
    if series.len() < 2 {
        return PulseBurden {
            status: "missing_event_boundary",
            ..PulseBurden::default()
        };
    }

    let mut positive_area = 0.0;
    let mut negative_area = 0.0;
    for pair in series.windows(2) {
        let gap = pair[1].elapsed_hours - pair[0].elapsed_hours;
        if gap > s.config.maximum_inter_observation_gap_hours {
            return PulseBurden {
                status: "gap_exceeded",
                ..PulseBurden::default()
            };
        }

        // Integrate the piecewise-linear anomaly exactly. If the response
        // crosses baseline, split the interval at the zero crossing instead
        // of applying a trapezoid to endpoint absolute values (which would
        // overstate area for a V-shaped absolute-anomaly curve).
        let d0 = pair[0].response_value - baseline;
        let d1 = pair[1].response_value - baseline;
        if d0 >= 0.0 && d1 >= 0.0 {
            positive_area += 0.5 * (d0 + d1) * gap;
        } else if d0 <= 0.0 && d1 <= 0.0 {
            negative_area += 0.5 * (d0.abs() + d1.abs()) * gap;
        } else {
            let fraction_to_zero = d0.abs() / (d0.abs() + d1.abs());
            let first_duration = gap * fraction_to_zero;
            let second_duration = gap - first_duration;
            if d0 > 0.0 {
                positive_area += 0.5 * d0.abs() * first_duration;
                negative_area += 0.5 * d1.abs() * second_duration;
            } else {
                negative_area += 0.5 * d0.abs() * first_duration;
                positive_area += 0.5 * d1.abs() * second_duration;
            }
        }
    }

    PulseBurden {
        absolute: Some(positive_area + negative_area),
        positive: Some(positive_area),
        negative: Some(negative_area),
        signed: Some(positive_area - negative_area),
        status: "qualified_complete_event_integral",
    }
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let counts = phase_counts(&s.observations);
    let enough_counts = counts.baseline >= s.config.minimum_baseline_observations
        && counts.stress >= s.config.minimum_stress_observations
        && counts.recovery >= s.config.minimum_recovery_observations;
    let baseline_max_gap = phase_max_gap(&s.observations, "baseline");
    let stress_max_gap = phase_max_gap(&s.observations, "stress");
    let recovery_max_gap = phase_max_gap(&s.observations, "recovery");
    let observed_max_gap = max_gap(&s.observations);
    let cadence_ok = [
        baseline_max_gap,
        stress_max_gap,
        recovery_max_gap,
        observed_max_gap,
    ]
    .into_iter()
    .all(|gap| gap.is_some_and(|value| value <= s.config.maximum_inter_observation_gap_hours));
    let enough = enough_counts && cadence_ok;
    let baseline_obs: Vec<&Observation> = s
        .observations
        .iter()
        .filter(|o| o.phase == "baseline")
        .collect();
    let stress_obs: Vec<&Observation> = s
        .observations
        .iter()
        .filter(|o| o.phase == "stress")
        .collect();
    let recovery_obs: Vec<&Observation> = s
        .observations
        .iter()
        .filter(|o| o.phase == "recovery")
        .collect();
    let baseline = mean(baseline_obs.iter().map(|o| o.response_value));
    let stress_peak = baseline.and_then(|b| {
        stress_obs.iter().copied().max_by(|a, other| {
            (a.response_value - b)
                .abs()
                .total_cmp(&(other.response_value - b).abs())
        })
    });
    let recovery_mean = mean(recovery_obs.iter().map(|o| o.response_value));
    let baseline_value = if enough { baseline } else { None };
    let peak = if enough { stress_peak } else { None };
    let recovery = if enough { recovery_mean } else { None };
    let stress_departure = baseline_value.zip(peak).map(|(b, p)| p.response_value - b);
    let peak_abs = stress_departure.map(f64::abs);
    let recovery_departure = baseline_value.zip(recovery).map(|(b, r)| r - b);
    let recovery_fraction = recovery_departure
        .zip(peak_abs)
        .and_then(|(r, p)| (p > f64::EPSILON).then_some(1.0 - r.abs() / p));

    let (recovery_time, recovery_time_status) = if !enough_counts {
        (None, "insufficient_phase_coverage")
    } else if !cadence_ok {
        (None, "sampling_gap_exceeded")
    } else if peak_abs.is_some_and(|x| x <= f64::EPSILON) {
        (None, "zero_stress_departure")
    } else {
        compute_recovery_time(s, baseline_value.unwrap_or(0.0), &recovery_obs)
    };
    let burden = if !enough_counts {
        PulseBurden {
            status: "insufficient_phase_coverage",
            ..PulseBurden::default()
        }
    } else if !cadence_ok {
        PulseBurden {
            status: "sampling_gap_exceeded",
            ..PulseBurden::default()
        }
    } else {
        pulse_burden(s, baseline_value.unwrap_or(0.0))
    };

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        event_id: s.event.event_id.clone(),
        event_type: s.event.event_type.clone(),
        experimental_unit_id: s.event.experimental_unit_id.clone(),
        response_variable: s.response.variable.clone(),
        response_unit: s.response.unit.clone(),
        response_measurement_method: s.response.measurement_method.clone(),
        response_support_description: s.response.support_description.clone().unwrap_or_default(),
        forcing_variable: s.forcing.variable.clone(),
        forcing_unit: s.forcing.unit.clone(),
        analysis_status: if !enough_counts {
            "insufficient_phase_coverage"
        } else if !cadence_ok {
            "sampling_gap_exceeded"
        } else {
            "qualified_descriptive_event"
        },
        phase_counts: counts,
        baseline_mean: baseline_value,
        stress_peak_value: peak.map(|p| p.response_value),
        stress_peak_time_hours: peak.map(|p| p.elapsed_hours),
        stress_peak_departure: stress_departure,
        stress_peak_absolute_departure: peak_abs,
        recovery_mean: recovery,
        recovery_departure,
        recovery_fraction,
        recovery_time_from_stress_end_hours: recovery_time,
        recovery_time_status,
        pulse_burden_absolute_anomaly_unit_hours: burden.absolute,
        pulse_burden_positive_anomaly_unit_hours: burden.positive,
        pulse_burden_negative_anomaly_unit_hours: burden.negative,
        pulse_burden_signed_anomaly_unit_hours: burden.signed,
        pulse_burden_status: burden.status,
        maximum_inter_observation_gap_hours: s.config.maximum_inter_observation_gap_hours,
        baseline_max_gap_hours: baseline_max_gap,
        stress_max_gap_hours: stress_max_gap,
        recovery_max_gap_hours: recovery_max_gap,
        max_observed_gap_hours: observed_max_gap,
        uncertainty_status: UNCERTAINTY_STATUS,
        controls: Controls {
            same_experimental_unit_pass: true,
            strict_time_order_pass: true,
            phase_window_consistency_pass: true,
            no_gap_bridging_pass: cadence_ok,
            no_causal_claim_pass: true,
            no_aggregate_resilience_score_pass: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
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
    if args.len() != 3 {
        eprintln!("usage: event-resilience-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("event resilience audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-rewetting-event.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn synthetic_event_computes_peak_recovery_and_burden() {
        let s = scenario();
        validate(&s).expect("valid synthetic event");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.analysis_status, "qualified_descriptive_event");
        assert_eq!(r.baseline_mean, Some(10.0));
        assert_eq!(r.stress_peak_value, Some(14.0));
        assert_eq!(r.stress_peak_time_hours, Some(12.0));
        assert_eq!(r.stress_peak_absolute_departure, Some(4.0));
        assert!(r.recovery_fraction.unwrap() > 0.8);
        assert_eq!(r.recovery_time_from_stress_end_hours, Some(24.0));
        assert_eq!(r.recovery_time_status, "qualified_sustained_recovery");
        assert!(r.pulse_burden_absolute_anomaly_unit_hours.unwrap() > 0.0);
    }

    #[test]
    fn long_gap_disables_integral_without_bridging() {
        let mut s = scenario();
        s.observations[7].elapsed_hours = 60.0;
        s.observations[8].elapsed_hours = 72.0;
        s.observations[9].elapsed_hours = 84.0;
        s.observations[10].elapsed_hours = 96.0;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.pulse_burden_absolute_anomaly_unit_hours, None);
        assert_eq!(r.pulse_burden_status, "sampling_gap_exceeded");
        assert_eq!(r.analysis_status, "sampling_gap_exceeded");
        assert!(!r.controls.no_gap_bridging_pass);
    }

    #[test]
    fn absolute_pulse_integral_splits_at_zero_crossings() {
        let mut s = scenario();
        s.observations[3].response_value = 12.0;
        s.observations[4].response_value = 8.0;
        s.observations[5].response_value = 10.0;
        s.observations[6].response_value = 10.0;
        for o in s.observations.iter_mut().skip(7) {
            o.response_value = 10.0;
        }
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.pulse_burden_positive_anomaly_unit_hours, Some(9.0));
        assert_eq!(r.pulse_burden_negative_anomaly_unit_hours, Some(9.0));
        assert_eq!(r.pulse_burden_absolute_anomaly_unit_hours, Some(18.0));
        assert_eq!(r.pulse_burden_signed_anomaly_unit_hours, Some(0.0));
    }

    #[test]
    fn no_tolerance_means_no_recovery_time_claim() {
        let mut s = scenario();
        s.recovery_tolerance = RecoveryTolerance {
            status: "not_set".into(),
            absolute_deviation_tolerance: None,
            required_consecutive_observations: None,
            authority: None,
            approval_reference: None,
            approval_date: None,
        };
        validate(&s).expect("valid with no recovery threshold");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.recovery_time_from_stress_end_hours, None);
        assert_eq!(r.recovery_time_status, "threshold_not_set");
    }

    #[test]
    fn missing_phase_coverage_abstains() {
        let mut s = scenario();
        s.config.minimum_recovery_observations = 8;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.analysis_status, "insufficient_phase_coverage");
        assert_eq!(r.baseline_mean, None);
        assert_eq!(r.pulse_burden_absolute_anomaly_unit_hours, None);
    }

    #[test]
    fn duplicate_observation_id_is_rejected() {
        let mut s = scenario();
        s.observations[1].observation_id = s.observations[0].observation_id.clone();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn wrong_phase_for_time_is_rejected() {
        let mut s = scenario();
        s.observations[3].phase = "recovery".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn repeated_sensor_records_cannot_change_experimental_unit() {
        let mut s = scenario();
        s.observations[4].experimental_unit_id = "other-plot".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn non_monotonic_time_is_rejected() {
        let mut s = scenario();
        s.observations[4].elapsed_hours = 5.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn no_stress_departure_has_no_recovery_fraction() {
        let mut s = scenario();
        for o in &mut s.observations {
            o.response_value = 10.0;
        }
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.recovery_fraction, None);
        assert_eq!(r.recovery_time_status, "zero_stress_departure");
    }

    #[test]
    fn integral_requires_exact_event_boundaries() {
        let mut s = scenario();
        s.observations[2].elapsed_hours = -1.0;
        s.observations[2].phase = "baseline".into();
        validate(&s).expect("timeline remains valid but event onset is unobserved");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.pulse_burden_absolute_anomaly_unit_hours, None);
        assert_eq!(r.pulse_burden_status, "missing_event_boundary");
    }
}

//! Synthetic evaluator for frozen event-sampling action traces.
//!
//! Candidate policy generation belongs to Symthaea. This evaluator reuses a
//! frozen trace across multiple hidden truth cases and computes capture / cost
//! metrics after the schedule is fixed. It never exposes evaluator truth as a
//! candidate action field.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic schedule evaluator only. Policy traces are authored examples, not Symthaea output or hardware measurements. Latent response truth is evaluator-only. No real soil flux, seasonal gas mass, energy saving, field resilience or causal management effect is qualified.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Benchmark {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_set_id: String,
    config: Config,
    sensor_contract: SensorContract,
    public_context_stream: Vec<ContextObservation>,
    policies: Vec<Policy>,
    truth_cases: Vec<TruthCase>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    horizon_start_hours: f64,
    horizon_end_hours: f64,
    response_baseline: f64,
    detection_threshold_above_baseline: f64,
    maximum_target_samples: usize,
    maximum_context_samples: usize,
    maximum_total_actions: usize,
    minimum_target_sample_interval_hours: f64,
    maximum_event_integral_gap_hours: f64,
    maximum_energy_mj: f64,
    maximum_assay_cost_units: f64,
    maximum_communication_bytes: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorContract {
    context_latency_hours: f64,
    target_latency_hours: f64,
    context_sample_energy_mj: f64,
    target_sample_energy_mj: f64,
    wake_energy_mj_per_action: f64,
    compute_energy_mj_per_action: f64,
    context_assay_cost_units: f64,
    target_assay_cost_units: f64,
    context_communication_bytes: usize,
    target_communication_bytes: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ContextObservation {
    observation_id: String,
    acquired_at_hours: f64,
    value: f64,
    standard_error: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Policy {
    policy_id: String,
    policy_kind: String,
    deployable: bool,
    actions: Vec<Action>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Action {
    action_id: String,
    modality: String,
    requested_at_hours: f64,
    context_observation_id: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TruthCase {
    case_id: String,
    has_pulse: bool,
    pulse_start_hours: f64,
    pulse_end_hours: f64,
    pulse_peak_hours: f64,
    pulse_amplitude: f64,
    risk_window_start_hours: f64,
    risk_window_end_hours: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_set_id: String,
    input_sha256: String,
    policy_count: usize,
    truth_case_count: usize,
    evaluations: Vec<Evaluation>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct Evaluation {
    policy_id: String,
    policy_kind: String,
    deployable: bool,
    truth_case_id: String,
    has_pulse: bool,
    target_sample_count: usize,
    context_sample_count: usize,
    total_action_count: usize,
    wake_count: usize,
    detected: bool,
    detection_delay_hours: Option<f64>,
    sampled_peak_anomaly: f64,
    sampled_peak_time_hours: Option<f64>,
    peak_magnitude_absolute_error: Option<f64>,
    peak_time_absolute_error_hours: Option<f64>,
    pulse_capture_ratio: Option<f64>,
    pulse_integral_estimate_unit_hours: Option<f64>,
    pulse_integral_true_unit_hours: Option<f64>,
    pulse_integral_absolute_error_unit_hours: Option<f64>,
    pulse_integral_status: &'static str,
    false_burst_target_samples: usize,
    maximum_target_gap_hours: Option<f64>,
    energy_mj: f64,
    assay_cost_units: f64,
    communication_bytes: usize,
    within_budget: bool,
}

#[derive(Debug, Serialize)]
struct Controls {
    same_policy_trace_across_hidden_cases: bool,
    no_candidate_truth_fields: bool,
    exact_cost_accounting: bool,
    budget_and_latency_enforcement: bool,
    no_integral_without_boundaries: bool,
    oracle_not_deployable: bool,
}

#[derive(Debug)]
struct TargetReading {
    acquired_at_hours: f64,
    response_value: f64,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn nearly_equal(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-8
}

fn validate(b: &Benchmark) -> Result<(), String> {
    if b.schema_version != "active-event-sampling-eval-v0.1"
        || b.fixture_status != FIXTURE_STATUS
        || b.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic fixture/version".into());
    }
    if b.scenario_set_id.trim().is_empty()
        || b.policies.len() < 2
        || b.truth_cases.len() < 2
        || b.public_context_stream.len() < 2
    {
        return Err("scenario set, policies, truth cases and context stream are incomplete".into());
    }
    for (name, value) in [
        ("horizon_start_hours", b.config.horizon_start_hours),
        ("horizon_end_hours", b.config.horizon_end_hours),
        ("response_baseline", b.config.response_baseline),
        (
            "detection_threshold_above_baseline",
            b.config.detection_threshold_above_baseline,
        ),
        (
            "minimum_target_sample_interval_hours",
            b.config.minimum_target_sample_interval_hours,
        ),
        (
            "maximum_event_integral_gap_hours",
            b.config.maximum_event_integral_gap_hours,
        ),
        ("maximum_energy_mj", b.config.maximum_energy_mj),
        (
            "maximum_assay_cost_units",
            b.config.maximum_assay_cost_units,
        ),
        (
            "context_latency_hours",
            b.sensor_contract.context_latency_hours,
        ),
        (
            "target_latency_hours",
            b.sensor_contract.target_latency_hours,
        ),
        (
            "context_sample_energy_mj",
            b.sensor_contract.context_sample_energy_mj,
        ),
        (
            "target_sample_energy_mj",
            b.sensor_contract.target_sample_energy_mj,
        ),
        (
            "wake_energy_mj_per_action",
            b.sensor_contract.wake_energy_mj_per_action,
        ),
        (
            "compute_energy_mj_per_action",
            b.sensor_contract.compute_energy_mj_per_action,
        ),
        (
            "context_assay_cost_units",
            b.sensor_contract.context_assay_cost_units,
        ),
        (
            "target_assay_cost_units",
            b.sensor_contract.target_assay_cost_units,
        ),
    ] {
        finite(name, value)?;
    }
    if b.config.horizon_start_hours >= b.config.horizon_end_hours
        || b.config.detection_threshold_above_baseline <= 0.0
        || b.config.minimum_target_sample_interval_hours <= 0.0
        || b.config.maximum_event_integral_gap_hours <= 0.0
        || b.config.maximum_target_samples == 0
        || b.config.maximum_total_actions == 0
        || b.sensor_contract.context_latency_hours < 0.0
        || b.sensor_contract.target_latency_hours < 0.0
    {
        return Err("invalid horizon, threshold, sample limit or latency contract".into());
    }

    let mut context_ids = BTreeSet::new();
    let mut previous_context_time = None;
    for obs in &b.public_context_stream {
        if obs.observation_id.trim().is_empty() || !context_ids.insert(obs.observation_id.clone()) {
            return Err("context observation IDs must be non-empty and unique".into());
        }
        finite("context timestamp", obs.acquired_at_hours)?;
        finite("context value", obs.value)?;
        finite("context standard error", obs.standard_error)?;
        if obs.standard_error < 0.0 {
            return Err("context standard error cannot be negative".into());
        }
        if previous_context_time.is_some_and(|x| obs.acquired_at_hours <= x) {
            return Err("public context stream must be strictly ordered".into());
        }
        if obs.acquired_at_hours < b.config.horizon_start_hours
            || obs.acquired_at_hours > b.config.horizon_end_hours
        {
            return Err("public context observation outside horizon".into());
        }
        previous_context_time = Some(obs.acquired_at_hours);
    }

    let mut policy_ids = BTreeSet::new();
    for policy in &b.policies {
        if policy.policy_id.trim().is_empty()
            || !policy_ids.insert(policy.policy_id.clone())
            || policy.actions.is_empty()
        {
            return Err("policy IDs must be unique and policies need actions".into());
        }
        match policy.policy_kind.as_str() {
            "fixed_sparse" | "context_triggered" if policy.deployable => {}
            "hindsight_oracle" if !policy.deployable => {}
            _ => {
                return Err(
                    "policy kind/deployability mismatch; oracle must be non-deployable".into(),
                );
            }
        }
        let mut action_ids = BTreeSet::new();
        let mut previous_request = None;
        let mut previous_completion = b.config.horizon_start_hours;
        let mut last_target_acquisition = None;
        let mut target_count = 0usize;
        let mut context_count = 0usize;

        for action in &policy.actions {
            if action.action_id.trim().is_empty() || !action_ids.insert(action.action_id.clone()) {
                return Err(format!(
                    "{} action IDs must be non-empty and unique",
                    policy.policy_id
                ));
            }
            finite("action requested_at_hours", action.requested_at_hours)?;
            if action.requested_at_hours < b.config.horizon_start_hours
                || action.requested_at_hours > b.config.horizon_end_hours
            {
                return Err(format!(
                    "{} action request outside horizon",
                    policy.policy_id
                ));
            }
            if previous_request.is_some_and(|x| action.requested_at_hours <= x) {
                return Err(format!(
                    "{} action request times must increase strictly",
                    policy.policy_id
                ));
            }
            if action.requested_at_hours + 1e-8 < previous_completion {
                return Err(format!(
                    "{} requests next action before previous action completes",
                    policy.policy_id
                ));
            }
            previous_request = Some(action.requested_at_hours);

            match action.modality.as_str() {
                "context_sample" => {
                    context_count += 1;
                    let obs_id = action.context_observation_id.as_deref().ok_or_else(|| {
                        format!(
                            "{} context action missing public observation ID",
                            policy.policy_id
                        )
                    })?;
                    let obs = b
                        .public_context_stream
                        .iter()
                        .find(|o| o.observation_id == obs_id)
                        .ok_or_else(|| {
                            format!(
                                "{} references an unknown context observation",
                                policy.policy_id
                            )
                        })?;
                    let acquired =
                        action.requested_at_hours + b.sensor_contract.context_latency_hours;
                    if !nearly_equal(acquired, obs.acquired_at_hours) {
                        return Err(format!(
                            "{} context reference time does not match sensor latency",
                            policy.policy_id
                        ));
                    }
                    previous_completion = acquired;
                }
                "target_sample" => {
                    target_count += 1;
                    if action.context_observation_id.is_some() {
                        return Err(format!(
                            "{} target action cannot carry context observation ID",
                            policy.policy_id
                        ));
                    }
                    let acquired =
                        action.requested_at_hours + b.sensor_contract.target_latency_hours;
                    if acquired > b.config.horizon_end_hours {
                        return Err(format!(
                            "{} target result arrives after horizon",
                            policy.policy_id
                        ));
                    }
                    if let Some(last) = last_target_acquisition
                        && acquired - last < b.config.minimum_target_sample_interval_hours
                    {
                        return Err(format!(
                            "{} exceeds target sensor maximum rate",
                            policy.policy_id
                        ));
                    }
                    last_target_acquisition = Some(acquired);
                    previous_completion = acquired;
                }
                _ => {
                    return Err(format!(
                        "{} has unsupported action modality",
                        policy.policy_id
                    ));
                }
            }
        }

        if target_count > b.config.maximum_target_samples
            || context_count > b.config.maximum_context_samples
            || policy.actions.len() > b.config.maximum_total_actions
        {
            return Err(format!(
                "{} exceeds sample/action-count budget",
                policy.policy_id
            ));
        }
    }

    let mut case_ids = BTreeSet::new();
    for case in &b.truth_cases {
        if case.case_id.trim().is_empty() || !case_ids.insert(case.case_id.clone()) {
            return Err("truth case IDs must be non-empty and unique".into());
        }
        for (name, value) in [
            ("pulse_start_hours", case.pulse_start_hours),
            ("pulse_end_hours", case.pulse_end_hours),
            ("pulse_peak_hours", case.pulse_peak_hours),
            ("pulse_amplitude", case.pulse_amplitude),
            ("risk_window_start_hours", case.risk_window_start_hours),
            ("risk_window_end_hours", case.risk_window_end_hours),
        ] {
            finite(name, value)?;
        }
        if !(b.config.horizon_start_hours <= case.risk_window_start_hours
            && case.risk_window_start_hours < case.risk_window_end_hours
            && case.risk_window_end_hours <= b.config.horizon_end_hours)
        {
            return Err(format!("{} has invalid risk window", case.case_id));
        }
        if case.has_pulse {
            if !(b.config.horizon_start_hours <= case.pulse_start_hours
                && case.pulse_start_hours < case.pulse_peak_hours
                && case.pulse_peak_hours < case.pulse_end_hours
                && case.pulse_end_hours <= b.config.horizon_end_hours)
                || case.pulse_amplitude <= 0.0
            {
                return Err(format!("{} has invalid pulse truth", case.case_id));
            }
        } else if case.pulse_amplitude != 0.0 {
            return Err(format!(
                "{} no-pulse case must have zero amplitude",
                case.case_id
            ));
        }
    }
    Ok(())
}

fn latent_response(b: &Benchmark, case: &TruthCase, t: f64) -> f64 {
    if !case.has_pulse || t < case.pulse_start_hours || t > case.pulse_end_hours {
        return b.config.response_baseline;
    }
    let left = case.pulse_peak_hours - case.pulse_start_hours;
    let right = case.pulse_end_hours - case.pulse_peak_hours;
    let fraction = if t <= case.pulse_peak_hours {
        (t - case.pulse_start_hours) / left
    } else {
        (case.pulse_end_hours - t) / right
    };
    b.config.response_baseline + case.pulse_amplitude * fraction.clamp(0.0, 1.0)
}

fn maximum_gap(readings: &[TargetReading]) -> Option<f64> {
    readings
        .windows(2)
        .map(|w| w[1].acquired_at_hours - w[0].acquired_at_hours)
        .max_by(f64::total_cmp)
}

fn pulse_integral(
    b: &Benchmark,
    case: &TruthCase,
    readings: &[TargetReading],
) -> (Option<f64>, Option<f64>, Option<f64>, &'static str) {
    if !case.has_pulse {
        return (None, None, None, "no_true_pulse");
    }
    let inside: Vec<&TargetReading> = readings
        .iter()
        .filter(|r| {
            r.acquired_at_hours >= case.pulse_start_hours
                && r.acquired_at_hours <= case.pulse_end_hours
        })
        .collect();
    let start = inside
        .first()
        .is_some_and(|r| nearly_equal(r.acquired_at_hours, case.pulse_start_hours));
    let end = inside
        .last()
        .is_some_and(|r| nearly_equal(r.acquired_at_hours, case.pulse_end_hours));
    if !start || !end {
        return (None, None, None, "missing_event_boundary");
    }
    let mut area = 0.0;
    for pair in inside.windows(2) {
        let gap = pair[1].acquired_at_hours - pair[0].acquired_at_hours;
        if gap > b.config.maximum_event_integral_gap_hours {
            return (None, None, None, "gap_exceeded");
        }
        let a = (pair[0].response_value - b.config.response_baseline).max(0.0);
        let z = (pair[1].response_value - b.config.response_baseline).max(0.0);
        area += 0.5 * (a + z) * gap;
    }
    let duration = case.pulse_end_hours - case.pulse_start_hours;
    let true_area = 0.5 * case.pulse_amplitude * duration;
    let error = (area - true_area).abs();
    (
        Some(area),
        Some(true_area),
        Some(error),
        "qualified_exact_boundary_integral",
    )
}

fn evaluate(b: &Benchmark, policy: &Policy, case: &TruthCase) -> Evaluation {
    let mut target = Vec::new();
    let mut target_count = 0usize;
    let mut context_count = 0usize;
    let mut assay_cost = 0.0;
    let mut energy = 0.0;
    let mut bytes = 0usize;

    for action in &policy.actions {
        energy += b.sensor_contract.wake_energy_mj_per_action
            + b.sensor_contract.compute_energy_mj_per_action;
        match action.modality.as_str() {
            "context_sample" => {
                context_count += 1;
                energy += b.sensor_contract.context_sample_energy_mj;
                assay_cost += b.sensor_contract.context_assay_cost_units;
                bytes += b.sensor_contract.context_communication_bytes;
            }
            "target_sample" => {
                target_count += 1;
                energy += b.sensor_contract.target_sample_energy_mj;
                assay_cost += b.sensor_contract.target_assay_cost_units;
                bytes += b.sensor_contract.target_communication_bytes;
                let acquired = action.requested_at_hours + b.sensor_contract.target_latency_hours;
                target.push(TargetReading {
                    acquired_at_hours: acquired,
                    response_value: latent_response(b, case, acquired),
                });
            }
            _ => unreachable!("validated modalities"),
        }
    }

    let peak = target
        .iter()
        .max_by(|a, z| a.response_value.total_cmp(&z.response_value));
    let sampled_peak = peak
        .map(|r| (r.response_value - b.config.response_baseline).max(0.0))
        .unwrap_or(0.0);
    let detected = case.has_pulse && sampled_peak >= b.config.detection_threshold_above_baseline;
    let detection_time = if detected {
        target
            .iter()
            .filter(|r| {
                r.acquired_at_hours >= case.pulse_start_hours
                    && r.acquired_at_hours <= case.pulse_end_hours
                    && r.response_value - b.config.response_baseline
                        >= b.config.detection_threshold_above_baseline
            })
            .map(|r| r.acquired_at_hours)
            .min_by(f64::total_cmp)
    } else {
        None
    };
    let detection_delay = detection_time.map(|t| t - case.pulse_start_hours);
    let peak_time_error = if case.has_pulse {
        peak.map(|r| (r.acquired_at_hours - case.pulse_peak_hours).abs())
    } else {
        None
    };
    let peak_magnitude_error = if case.has_pulse {
        Some((sampled_peak - case.pulse_amplitude).abs())
    } else {
        None
    };
    let capture_ratio = if case.has_pulse {
        Some((sampled_peak / case.pulse_amplitude).clamp(0.0, 1.0))
    } else {
        None
    };
    let false_burst = if case.has_pulse {
        0
    } else {
        target
            .iter()
            .filter(|r| {
                r.acquired_at_hours >= case.risk_window_start_hours
                    && r.acquired_at_hours <= case.risk_window_end_hours
            })
            .count()
    };
    let (integral_estimate, integral_true, integral_error, integral_status) =
        pulse_integral(b, case, &target);

    let energy_budget = energy <= b.config.maximum_energy_mj + 1e-9;
    let assay_budget = assay_cost <= b.config.maximum_assay_cost_units + 1e-9;
    let byte_budget = bytes <= b.config.maximum_communication_bytes;
    let within_budget = target_count <= b.config.maximum_target_samples
        && context_count <= b.config.maximum_context_samples
        && policy.actions.len() <= b.config.maximum_total_actions
        && energy_budget
        && assay_budget
        && byte_budget;

    Evaluation {
        policy_id: policy.policy_id.clone(),
        policy_kind: policy.policy_kind.clone(),
        deployable: policy.deployable,
        truth_case_id: case.case_id.clone(),
        has_pulse: case.has_pulse,
        target_sample_count: target_count,
        context_sample_count: context_count,
        total_action_count: policy.actions.len(),
        wake_count: policy.actions.len(),
        detected,
        detection_delay_hours: detection_delay,
        sampled_peak_anomaly: sampled_peak,
        sampled_peak_time_hours: peak.map(|r| r.acquired_at_hours),
        peak_magnitude_absolute_error: peak_magnitude_error,
        peak_time_absolute_error_hours: peak_time_error,
        pulse_capture_ratio: capture_ratio,
        pulse_integral_estimate_unit_hours: integral_estimate,
        pulse_integral_true_unit_hours: integral_true,
        pulse_integral_absolute_error_unit_hours: integral_error,
        pulse_integral_status: integral_status,
        false_burst_target_samples: false_burst,
        maximum_target_gap_hours: maximum_gap(&target),
        energy_mj: energy,
        assay_cost_units: assay_cost,
        communication_bytes: bytes,
        within_budget,
    }
}

fn build_report(b: &Benchmark, digest: String) -> Report {
    let mut evaluations = Vec::with_capacity(b.policies.len() * b.truth_cases.len());
    for policy in &b.policies {
        for case in &b.truth_cases {
            evaluations.push(evaluate(b, policy, case));
        }
    }
    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_set_id: b.scenario_set_id.clone(),
        input_sha256: digest,
        policy_count: b.policies.len(),
        truth_case_count: b.truth_cases.len(),
        evaluations,
        controls: Controls {
            same_policy_trace_across_hidden_cases: true,
            no_candidate_truth_fields: true,
            exact_cost_accounting: true,
            budget_and_latency_enforcement: true,
            no_integral_without_boundaries: true,
            oracle_not_deployable: b
                .policies
                .iter()
                .filter(|p| p.policy_kind == "hindsight_oracle")
                .all(|p| !p.deployable),
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
    let benchmark: Benchmark = serde_json::from_slice(&bytes)?;
    validate(&benchmark).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&benchmark, digest);
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
        eprintln!("usage: active-sampling-eval <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("active sampling evaluation failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn benchmark() -> Benchmark {
        serde_json::from_str(include_str!("../fixtures/synthetic-active-sampling.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn fixed_schedule_misses_pulse_while_context_burst_captures_it() {
        let b = benchmark();
        validate(&b).expect("valid fixture");
        let fixed = b
            .policies
            .iter()
            .find(|p| p.policy_id == "fixed_sparse")
            .unwrap();
        let burst = b
            .policies
            .iter()
            .find(|p| p.policy_id == "context_trigger_burst")
            .unwrap();
        let truth = b
            .truth_cases
            .iter()
            .find(|c| c.case_id == "short_rewetting_pulse")
            .unwrap();
        let fixed_result = evaluate(&b, fixed, truth);
        let burst_result = evaluate(&b, burst, truth);
        assert!(!fixed_result.detected);
        assert!(burst_result.detected);
        assert_eq!(
            burst_result.pulse_integral_status,
            "qualified_exact_boundary_integral"
        );
        assert_eq!(burst_result.pulse_integral_estimate_unit_hours, Some(5.0));
        assert_eq!(
            burst_result.pulse_integral_absolute_error_unit_hours,
            Some(0.0)
        );
    }

    #[test]
    fn same_context_burst_becomes_false_sampling_when_response_pulse_is_absent() {
        let b = benchmark();
        let burst = b
            .policies
            .iter()
            .find(|p| p.policy_id == "context_trigger_burst")
            .unwrap();
        let no_pulse = b
            .truth_cases
            .iter()
            .find(|c| c.case_id == "context_step_without_response_pulse")
            .unwrap();
        let result = evaluate(&b, burst, no_pulse);
        assert_eq!(result.false_burst_target_samples, 3);
        assert!(!result.detected);
    }

    #[test]
    fn policy_trace_is_reused_across_truth_cases_and_oracle_is_not_deployable() {
        let b = benchmark();
        assert_eq!(b.policies.len(), 3);
        assert!(
            b.policies
                .iter()
                .find(|p| p.policy_id == "hindsight_oracle")
                .is_some_and(|p| !p.deployable)
        );
        assert_eq!(b.truth_cases.len(), 2);
    }

    #[test]
    fn context_sample_must_bind_to_public_stream_and_latency() {
        let mut b = benchmark();
        b.policies[1].actions[0].context_observation_id = Some("missing".into());
        assert!(validate(&b).is_err());
    }

    #[test]
    fn policy_cannot_request_next_action_before_previous_completion() {
        let mut b = benchmark();
        b.policies[1].actions[1].requested_at_hours = 0.05;
        assert!(validate(&b).is_err());
    }

    #[test]
    fn target_samples_must_respect_sensor_rate() {
        let mut b = benchmark();
        let policy = b
            .policies
            .iter_mut()
            .find(|p| p.policy_id == "context_trigger_burst")
            .unwrap();
        policy.actions[8].requested_at_hours = policy.actions[7].requested_at_hours + 0.1;
        assert!(validate(&b).is_err());
    }

    #[test]
    fn oracle_cannot_be_marked_deployable() {
        let mut b = benchmark();
        b.policies[2].deployable = true;
        assert!(validate(&b).is_err());
    }

    #[test]
    fn excessive_resources_are_reported_out_of_budget() {
        let mut b = benchmark();
        b.config.maximum_energy_mj = 1.0;
        let policy = b
            .policies
            .iter()
            .find(|p| p.policy_id == "context_trigger_burst")
            .unwrap();
        let case = &b.truth_cases[0];
        let result = evaluate(&b, policy, case);
        assert!(!result.within_budget);
    }
}

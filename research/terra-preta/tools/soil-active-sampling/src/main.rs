//! Synthetic known-truth soil active-sensing task/evaluator.
//!
//! Candidate policies receive only the public contract and sampled observations.
//! The latent minute-resolution reference series is held in a separate evaluator
//! structure. This same-process prototype is not a hostile-process isolation boundary.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic active-sensing benchmark only. The candidate policy sees sampled observations and a public budget; hidden truth is evaluator-only by API convention in this process, not isolated by an OS security boundary. Energy costs are synthetic accounting inputs, not physical measurements.";

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    public_contract: PublicContract,
    sensor_model: SensorModel,
    hidden_truth: HiddenTruthConfig,
    policy: PolicyConfig,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PublicContract {
    horizon_minutes: u32,
    maximum_sample_count: usize,
    detection_delta_threshold: f64,
    maximum_integral_gap_minutes: u32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorModel {
    seed: u64,
    response_noise_sd: f64,
    moisture_noise_sd: f64,
    sensor_energy_nj_per_sample: u64,
    wake_energy_nj_per_sample: u64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct HiddenTruthConfig {
    baseline_response: f64,
    baseline_moisture: f64,
    rewet_at_minute: u32,
    rewet_moisture: f64,
    pulse: Option<PulseConfig>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PulseConfig {
    start_minute: u32,
    width_minutes: u32,
    peak_amplitude: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolicyConfig {
    kind: String,
    fixed_interval_minutes: Option<u32>,
    normal_interval_minutes: Option<u32>,
    fast_interval_minutes: Option<u32>,
    rewetting_delta_threshold: Option<f64>,
    fast_window_minutes: Option<u32>,
}

#[derive(Clone, Copy, Debug)]
struct TruthPoint {
    minute: u32,
    moisture: f64,
    response: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct Observation {
    minute: u32,
    moisture: f64,
    response: f64,
}

#[derive(Clone, Copy, Debug)]
struct CandidatePublicContract {
    horizon_minutes: u32,
    maximum_sample_count: usize,
}

#[derive(Clone, Copy, Debug)]
struct CandidateActionContext<'a> {
    now_minute: u32,
    observations: &'a [Observation],
    public_contract: CandidatePublicContract,
}

#[derive(Clone, Debug)]
enum CandidatePolicy {
    Fixed {
        interval_minutes: u32,
    },
    RewettingTriggered {
        normal_interval_minutes: u32,
        fast_interval_minutes: u32,
        rewetting_delta_threshold: f64,
        fast_window_minutes: u32,
        fast_until_minute: u32,
    },
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    policy_kind: String,
    input_sha256: String,
    event_present: bool,
    event_detected: bool,
    false_positive_detection: bool,
    detection_delay_minutes: Option<u32>,
    observations_within_event: usize,
    sample_count: usize,
    sample_budget: usize,
    budget_respected: bool,
    maximum_observed_gap_minutes: u32,
    peak_estimate: Option<f64>,
    peak_absolute_error: Option<f64>,
    true_pulse_integral_response_unit_minutes: Option<f64>,
    estimated_pulse_integral_response_unit_minutes: Option<f64>,
    pulse_integral_absolute_error: Option<f64>,
    pulse_integral_status: &'static str,
    sensor_energy_nj: u64,
    wake_energy_nj: u64,
    candidate_truth_access: bool,
    interpretation_boundary: &'static str,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "soil-active-sensing-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic scenario version".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.public_contract.horizon_minutes < 10
        || s.public_contract.maximum_sample_count < 2
        || s.public_contract.maximum_integral_gap_minutes == 0
    {
        return Err("scenario identity and public sample contract are invalid".into());
    }
    for (name, value) in [
        (
            "detection_delta_threshold",
            s.public_contract.detection_delta_threshold,
        ),
        ("response_noise_sd", s.sensor_model.response_noise_sd),
        ("moisture_noise_sd", s.sensor_model.moisture_noise_sd),
        ("baseline_response", s.hidden_truth.baseline_response),
        ("baseline_moisture", s.hidden_truth.baseline_moisture),
        ("rewet_moisture", s.hidden_truth.rewet_moisture),
    ] {
        finite(name, value)?;
    }
    if s.public_contract.detection_delta_threshold < 0.0
        || s.sensor_model.response_noise_sd < 0.0
        || s.sensor_model.moisture_noise_sd < 0.0
        || s.hidden_truth.baseline_moisture < 0.0
        || s.hidden_truth.rewet_moisture < 0.0
        || s.hidden_truth.rewet_at_minute > s.public_contract.horizon_minutes
    {
        return Err("threshold, noise, moisture or rewet timing is invalid".into());
    }
    if let Some(pulse) = &s.hidden_truth.pulse {
        finite("pulse peak amplitude", pulse.peak_amplitude)?;
        if pulse.width_minutes < 2
            || pulse.peak_amplitude <= 0.0
            || pulse.start_minute == 0
            || pulse.start_minute.saturating_add(pulse.width_minutes)
                > s.public_contract.horizon_minutes
        {
            return Err("pulse timing/width/amplitude is invalid".into());
        }
    }
    match s.policy.kind.as_str() {
        "fixed_cadence" => {
            if s.policy.fixed_interval_minutes.unwrap_or(0) == 0 {
                return Err("fixed cadence needs a positive interval".into());
            }
        }
        "rewetting_triggered" => {
            if s.policy.normal_interval_minutes.unwrap_or(0) == 0
                || s.policy.fast_interval_minutes.unwrap_or(0) == 0
                || s.policy.fast_window_minutes.unwrap_or(0) == 0
            {
                return Err(
                    "rewetting policy needs positive normal/fast intervals and window".into(),
                );
            }
            let threshold = s.policy.rewetting_delta_threshold.ok_or_else(|| {
                "rewetting policy needs an observed moisture delta threshold".to_string()
            })?;
            finite("rewetting_delta_threshold", threshold)?;
            if threshold < 0.0 {
                return Err("rewetting delta threshold cannot be negative".into());
            }
        }
        _ => return Err("unsupported policy kind".into()),
    }
    Ok(())
}

fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn noise(seed: u64, minute: u32, channel: u64, standard_deviation: f64) -> f64 {
    if standard_deviation == 0.0 {
        return 0.0;
    }
    let bits = splitmix64(seed ^ u64::from(minute).rotate_left(17) ^ channel);
    let unit = (bits >> 11) as f64 / ((1_u64 << 53) as f64);
    (2.0 * unit - 1.0) * standard_deviation
}

fn pulse_response(minute: u32, pulse: &PulseConfig) -> f64 {
    let end = pulse.start_minute + pulse.width_minutes;
    if minute < pulse.start_minute || minute > end {
        return 0.0;
    }
    let midpoint = f64::from(pulse.width_minutes) / 2.0;
    let relative = f64::from(minute - pulse.start_minute);
    let normalized = (1.0 - ((relative - midpoint).abs() / midpoint)).max(0.0);
    pulse.peak_amplitude * normalized
}

fn generate_truth(s: &Scenario) -> Vec<TruthPoint> {
    (0..=s.public_contract.horizon_minutes)
        .map(|minute| {
            let moisture = if minute >= s.hidden_truth.rewet_at_minute {
                s.hidden_truth.rewet_moisture
            } else {
                s.hidden_truth.baseline_moisture
            };
            let pulse = s
                .hidden_truth
                .pulse
                .as_ref()
                .map(|p| pulse_response(minute, p))
                .unwrap_or(0.0);
            TruthPoint {
                minute,
                moisture,
                response: s.hidden_truth.baseline_response + pulse,
            }
        })
        .collect()
}

fn sensor_observation(s: &Scenario, truth: TruthPoint) -> Observation {
    Observation {
        minute: truth.minute,
        moisture: truth.moisture
            + noise(
                s.sensor_model.seed,
                truth.minute,
                0x4D4F4953,
                s.sensor_model.moisture_noise_sd,
            ),
        response: truth.response
            + noise(
                s.sensor_model.seed,
                truth.minute,
                0x52455350,
                s.sensor_model.response_noise_sd,
            ),
    }
}

/// Candidate-only API: no truth series, event time, pulse width or oracle labels are arguments.
fn next_sample_interval(policy: &mut CandidatePolicy, context: CandidateActionContext<'_>) -> u32 {
    if context.now_minute >= context.public_contract.horizon_minutes
        || context.observations.len() >= context.public_contract.maximum_sample_count
    {
        return context
            .public_contract
            .horizon_minutes
            .saturating_sub(context.now_minute)
            .max(1);
    }
    match policy {
        CandidatePolicy::Fixed { interval_minutes } => *interval_minutes,
        CandidatePolicy::RewettingTriggered {
            normal_interval_minutes,
            fast_interval_minutes,
            rewetting_delta_threshold,
            fast_window_minutes,
            fast_until_minute,
        } => {
            if context.observations.len() >= 2 {
                let last = context.observations[context.observations.len() - 1];
                let previous = context.observations[context.observations.len() - 2];
                let rewetting_detected =
                    last.moisture - previous.moisture >= *rewetting_delta_threshold;
                if rewetting_detected && context.now_minute >= *fast_until_minute {
                    *fast_until_minute = context.now_minute.saturating_add(*fast_window_minutes);
                }
            }
            if context.now_minute < *fast_until_minute {
                *fast_interval_minutes
            } else {
                *normal_interval_minutes
            }
        }
    }
}

fn candidate_policy(s: &Scenario) -> CandidatePolicy {
    match s.policy.kind.as_str() {
        "fixed_cadence" => CandidatePolicy::Fixed {
            interval_minutes: s.policy.fixed_interval_minutes.unwrap_or(1),
        },
        "rewetting_triggered" => CandidatePolicy::RewettingTriggered {
            normal_interval_minutes: s.policy.normal_interval_minutes.unwrap_or(30),
            fast_interval_minutes: s.policy.fast_interval_minutes.unwrap_or(1),
            rewetting_delta_threshold: s.policy.rewetting_delta_threshold.unwrap_or(0.1),
            fast_window_minutes: s.policy.fast_window_minutes.unwrap_or(15),
            fast_until_minute: 0,
        },
        _ => unreachable!("validation rejects unknown policies"),
    }
}

fn pulse_integral(truth: &[TruthPoint], pulse: &PulseConfig, baseline: f64) -> f64 {
    let start = pulse.start_minute;
    let end = start + pulse.width_minutes;
    let series: Vec<TruthPoint> = truth
        .iter()
        .copied()
        .filter(|point| point.minute >= start && point.minute <= end)
        .collect();
    series
        .windows(2)
        .map(|pair| {
            let left = (pair[0].response - baseline).max(0.0);
            let right = (pair[1].response - baseline).max(0.0);
            0.5 * (left + right) * f64::from(pair[1].minute - pair[0].minute)
        })
        .sum()
}

fn maximum_gap(observations: &[Observation]) -> u32 {
    observations
        .windows(2)
        .map(|pair| pair[1].minute.saturating_sub(pair[0].minute))
        .max()
        .unwrap_or(0)
}

fn evaluate(s: &Scenario, digest: String) -> Report {
    let truth = generate_truth(s);
    let mut policy = candidate_policy(s);
    let public_contract = CandidatePublicContract {
        horizon_minutes: s.public_contract.horizon_minutes,
        maximum_sample_count: s.public_contract.maximum_sample_count,
    };
    let mut observations = Vec::new();
    let mut now = 0_u32;

    while now <= public_contract.horizon_minutes
        && observations.len() < public_contract.maximum_sample_count
    {
        let truth_point = truth[now as usize];
        let observation = sensor_observation(s, truth_point);
        observations.push(observation);
        if observations.len() >= public_contract.maximum_sample_count
            || now == public_contract.horizon_minutes
        {
            break;
        }
        let context = CandidateActionContext {
            now_minute: now,
            observations: &observations,
            public_contract,
        };
        let interval = next_sample_interval(&mut policy, context).max(1);
        now = now.saturating_add(interval);
    }

    let baseline = observations.iter().take(3).map(|o| o.response).sum::<f64>()
        / observations.len().min(3) as f64;
    let event = s.hidden_truth.pulse.as_ref();
    let (
        event_present,
        event_detected,
        false_positive,
        delay,
        event_observations,
        peak_estimate,
        peak_error,
        true_integral,
        estimated_integral,
        integral_error,
        integral_status,
    ) = if let Some(pulse) = event {
        let event_end = pulse.start_minute + pulse.width_minutes;
        let within: Vec<Observation> = observations
            .iter()
            .copied()
            .filter(|o| o.minute >= pulse.start_minute && o.minute <= event_end)
            .collect();
        let detections: Vec<Observation> = within
            .iter()
            .copied()
            .filter(|o| o.response - baseline >= public_contract.detection_delta_threshold)
            .collect();
        let peak = within.iter().map(|o| o.response).reduce(f64::max);
        let true_peak = s.hidden_truth.baseline_response + pulse.peak_amplitude;
        let true_mass = pulse_integral(&truth, pulse, s.hidden_truth.baseline_response);
        let gap_is_acceptable = within.windows(2).all(|pair| {
            pair[1].minute.saturating_sub(pair[0].minute)
                <= s.public_contract.maximum_integral_gap_minutes
        });
        let estimate = if within.len() >= 2 && gap_is_acceptable {
            Some(
                within
                    .windows(2)
                    .map(|pair| {
                        let left = (pair[0].response - baseline).max(0.0);
                        let right = (pair[1].response - baseline).max(0.0);
                        0.5 * (left + right) * f64::from(pair[1].minute - pair[0].minute)
                    })
                    .sum::<f64>(),
            )
        } else {
            None
        };
        let estimate_status = if estimate.is_some() {
            "estimated_from_event_window_samples"
        } else if within.len() >= 2 && !gap_is_acceptable {
            "event_window_gap_exceeded"
        } else {
            "insufficient_event_window_samples"
        };
        (
            true,
            !detections.is_empty(),
            false,
            detections
                .first()
                .map(|o| o.minute.saturating_sub(pulse.start_minute)),
            within.len(),
            peak,
            peak.map(|x| (x - true_peak).abs()),
            Some(true_mass),
            estimate,
            estimate.map(|x| (x - true_mass).abs()),
            estimate_status,
        )
    } else {
        let false_alarm = observations
            .iter()
            .any(|o| o.response - baseline >= public_contract.detection_delta_threshold);
        (
            false,
            false_alarm,
            false_alarm,
            None,
            0,
            None,
            None,
            None,
            None,
            None,
            "no_event_present",
        )
    };

    let count = observations.len();
    let sensor_energy = s
        .sensor_model
        .sensor_energy_nj_per_sample
        .saturating_mul(count as u64);
    let wake_energy = s
        .sensor_model
        .wake_energy_nj_per_sample
        .saturating_mul(count as u64);
    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        policy_kind: s.policy.kind.clone(),
        input_sha256: digest,
        event_present,
        event_detected,
        false_positive_detection: false_positive,
        detection_delay_minutes: delay,
        observations_within_event: event_observations,
        sample_count: count,
        sample_budget: public_contract.maximum_sample_count,
        budget_respected: count <= public_contract.maximum_sample_count,
        maximum_observed_gap_minutes: maximum_gap(&observations),
        peak_estimate,
        peak_absolute_error: peak_error,
        true_pulse_integral_response_unit_minutes: true_integral,
        estimated_pulse_integral_response_unit_minutes: estimated_integral,
        pulse_integral_absolute_error: integral_error,
        pulse_integral_status: integral_status,
        sensor_energy_nj: sensor_energy,
        wake_energy_nj: wake_energy,
        candidate_truth_access: false,
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
    let report = evaluate(&scenario, digest);
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
        eprintln!("usage: soil-active-sampling <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("soil active-sampling benchmark failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/short-rewetting-pulse.json"))
            .expect("synthetic pulse fixture parses")
    }

    #[test]
    fn rewetting_triggered_policy_captures_narrow_pulse_that_fixed_cadence_misses() {
        let mut fixed = scenario();
        fixed.policy = PolicyConfig {
            kind: "fixed_cadence".into(),
            fixed_interval_minutes: Some(15),
            normal_interval_minutes: None,
            fast_interval_minutes: None,
            rewetting_delta_threshold: None,
            fast_window_minutes: None,
        };
        let fixed_report = evaluate(&fixed, "0".repeat(64));

        let mut adaptive = scenario();
        adaptive.policy = PolicyConfig {
            kind: "rewetting_triggered".into(),
            fixed_interval_minutes: None,
            normal_interval_minutes: Some(60),
            fast_interval_minutes: Some(1),
            rewetting_delta_threshold: Some(0.05),
            fast_window_minutes: Some(10),
        };
        let adaptive_report = evaluate(&adaptive, "0".repeat(64));

        assert!(!fixed_report.event_detected);
        assert!(adaptive_report.event_detected);
        assert!(
            adaptive_report.sample_count < fixed_report.sample_count,
            "adaptive samples {} versus fixed samples {}; detection fixed={} adaptive={}",
            adaptive_report.sample_count,
            fixed_report.sample_count,
            fixed_report.event_detected,
            adaptive_report.event_detected
        );
        assert!(adaptive_report.sample_count <= adaptive_report.sample_budget);
        assert_eq!(adaptive_report.candidate_truth_access, false);
    }

    #[test]
    fn no_pulse_forcing_change_is_a_false_wakeup_cost_control_not_false_alarm() {
        let mut s = scenario();
        s.hidden_truth.pulse = None;
        let mut adaptive = s;
        adaptive.policy = PolicyConfig {
            kind: "rewetting_triggered".into(),
            fixed_interval_minutes: None,
            normal_interval_minutes: Some(30),
            fast_interval_minutes: Some(1),
            rewetting_delta_threshold: Some(0.05),
            fast_window_minutes: Some(15),
        };
        let r = evaluate(&adaptive, "0".repeat(64));
        assert!(!r.event_present);
        assert!(!r.false_positive_detection);
        assert!(r.sample_count > 2);
    }

    #[test]
    fn deterministic_replay_for_same_scenario_and_policy() {
        let s = scenario();
        let first = evaluate(&s, "0".repeat(64));
        let second = evaluate(&s, "0".repeat(64));
        assert_eq!(first.sample_count, second.sample_count);
        assert_eq!(
            first.detection_delay_minutes,
            second.detection_delay_minutes
        );
        assert_eq!(
            first.pulse_integral_absolute_error,
            second.pulse_integral_absolute_error
        );
    }

    #[test]
    fn budget_is_respected_even_when_triggered_sampling_stays_fast() {
        let mut s = scenario();
        s.public_contract.maximum_sample_count = 10;
        s.policy = PolicyConfig {
            kind: "rewetting_triggered".into(),
            fixed_interval_minutes: None,
            normal_interval_minutes: Some(30),
            fast_interval_minutes: Some(1),
            rewetting_delta_threshold: Some(0.05),
            fast_window_minutes: Some(30),
        };
        let r = evaluate(&s, "0".repeat(64));
        assert!(r.budget_respected);
        assert_eq!(r.sample_count, 10);
    }

    #[test]
    fn candidate_action_api_has_no_hidden_truth_argument() {
        let mut policy = CandidatePolicy::RewettingTriggered {
            normal_interval_minutes: 30,
            fast_interval_minutes: 1,
            rewetting_delta_threshold: 0.05,
            fast_window_minutes: 15,
            fast_until_minute: 0,
        };
        let history = [
            Observation {
                minute: 90,
                moisture: 0.10,
                response: 10.0,
            },
            Observation {
                minute: 120,
                moisture: 0.22,
                response: 10.0,
            },
        ];
        let context = CandidateActionContext {
            now_minute: 120,
            observations: &history,
            public_contract: CandidatePublicContract {
                horizon_minutes: 360,
                maximum_sample_count: 100,
                detection_delta_threshold: 1.0,
            },
        };
        assert_eq!(next_sample_interval(&mut policy, context), 1);
    }
}

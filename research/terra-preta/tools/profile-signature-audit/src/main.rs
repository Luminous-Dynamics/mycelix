use chrono::{DateTime, Duration, FixedOffset};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::HashSet,
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const MIN_CADENCE_MINUTES: i64 = 5;
const MAX_CADENCE_MINUTES: i64 = 30;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    event_end_utc: String,
    response_threshold_m3_m3: f64,
    sensors: Vec<SensorSeries>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorSeries {
    sensor_id: String,
    depth_cm: f64,
    observations: Vec<Observation>,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ObservationStatus {
    ObservedGood,
    InterpolatedGapLt6h,
    Missing,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    timestamp_utc: String,
    volumetric_water_content_m3_m3: Option<f64>,
    status: ObservationStatus,
}

#[derive(Clone)]
struct ParsedObservation {
    timestamp: DateTime<FixedOffset>,
    theta: Option<f64>,
    status: ObservationStatus,
}

struct SensorAnalysis {
    sensor_id: String,
    depth_cm: f64,
    response_status: &'static str,
    response_onset: Option<DateTime<FixedOffset>>,
    response_lag_minutes: Option<i64>,
    baseline_theta: Option<f64>,
    maximum_event_increase: Option<f64>,
    maximum_rise_rate: Option<f64>,
    maximum_rise_start: Option<DateTime<FixedOffset>>,
    maximum_rise_end: Option<DateTime<FixedOffset>>,
    valid_rise_intervals: usize,
    missing_event_bins: usize,
    interpolated_event_bins: usize,
}

#[derive(Serialize)]
struct SensorResult {
    sensor_id: String,
    depth_cm: f64,
    response_status: String,
    response_onset_utc: Option<String>,
    response_lag_minutes: Option<i64>,
    baseline_theta_m3_m3: Option<f64>,
    maximum_event_increase_m3_m3: Option<f64>,
    maximum_rise_rate_m3_m3_per_h: Option<f64>,
    maximum_rise_interval_start_utc: Option<String>,
    maximum_rise_interval_end_utc: Option<String>,
    valid_rise_intervals: usize,
    missing_event_bins: usize,
    interpolated_event_bins: usize,
}

#[derive(Serialize)]
struct PairResult {
    shallow_sensor_id: String,
    shallow_depth_cm: f64,
    shallow_response_utc: String,
    deep_sensor_id: String,
    deep_depth_cm: f64,
    deep_response_utc: String,
    deep_minus_shallow_response_minutes: i64,
    classification: String,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    event_end_utc: String,
    cadence_minutes: i64,
    response_threshold_m3_m3: f64,
    sensor_count: usize,
    responding_sensor_count: usize,
    indeterminate_sensor_count: usize,
    comparable_depth_pair_count: usize,
    nonsequential_pair_count: usize,
    sequential_pair_count: usize,
    simultaneous_pair_count: usize,
    profile_signature_status: String,
    sensors: Vec<SensorResult>,
    pair_results: Vec<PairResult>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn parse_utc(value: &str, label: &str) -> Result<DateTime<FixedOffset>, String> {
    let timestamp = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if timestamp.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp_utc must use a zero UTC offset"));
    }
    Ok(timestamp)
}

fn validate_nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn parse_and_validate(
    input: &Input,
) -> Result<
    (
        DateTime<FixedOffset>,
        DateTime<FixedOffset>,
        i64,
        Vec<Vec<ParsedObservation>>,
    ),
    String,
> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.site_id, "site_id"),
        (&input.plot_id, "plot_id"),
        (&input.profile_id, "profile_id"),
        (&input.event_id, "event_id"),
    ] {
        validate_nonblank(value, label)?;
    }
    if !input.response_threshold_m3_m3.is_finite()
        || input.response_threshold_m3_m3 <= 0.0
        || input.response_threshold_m3_m3 > 1.0
    {
        return Err("response_threshold_m3_m3 must be finite and within (0, 1]".to_owned());
    }
    if input.sensors.len() < 2 {
        return Err("at least two sensor depths are required for an NSR comparison".to_owned());
    }

    let event_start = parse_utc(&input.event_start_utc, "event_start_utc")?;
    let event_end = parse_utc(&input.event_end_utc, "event_end_utc")?;
    if event_start >= event_end {
        return Err("event_end_utc must be after event_start_utc".to_owned());
    }

    let mut sensor_ids = HashSet::new();
    let mut depths = HashSet::new();
    let mut parsed_sensors: Vec<Vec<ParsedObservation>> = Vec::with_capacity(input.sensors.len());
    let mut common_times: Option<Vec<DateTime<FixedOffset>>> = None;
    let mut common_cadence: Option<i64> = None;

    for (sensor_index, sensor) in input.sensors.iter().enumerate() {
        validate_nonblank(&sensor.sensor_id, &format!("sensors[{sensor_index}].sensor_id"))?;
        if !sensor_ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        if !sensor.depth_cm.is_finite() || sensor.depth_cm <= 0.0 {
            return Err(format!("sensor {} depth_cm must be finite and positive", sensor.sensor_id));
        }
        let depth_key = (sensor.depth_cm * 1_000_000.0).round() as i64;
        if !depths.insert(depth_key) {
            return Err(format!("duplicate sensor depth {:.6} cm is not comparable as a vertical NSR pair", sensor.depth_cm));
        }
        if sensor.observations.len() < 3 {
            return Err(format!("sensor {} requires at least three observations including baseline and event samples", sensor.sensor_id));
        }

        let mut observations = Vec::with_capacity(sensor.observations.len());
        for (sample_index, sample) in sensor.observations.iter().enumerate() {
            let timestamp = parse_utc(
                &sample.timestamp_utc,
                &format!("sensors[{sensor_index}].observations[{sample_index}]"),
            )?;
            match (sample.status, sample.volumetric_water_content_m3_m3) {
                (ObservationStatus::Missing, None) => {}
                (ObservationStatus::Missing, Some(_)) => {
                    return Err(format!("sensor {} missing observation must have null theta", sensor.sensor_id));
                }
                (ObservationStatus::ObservedGood | ObservationStatus::InterpolatedGapLt6h, None) => {
                    return Err(format!("sensor {} non-missing observation requires theta", sensor.sensor_id));
                }
                (_, Some(theta)) if !theta.is_finite() || !(0.0..=1.0).contains(&theta) => {
                    return Err(format!("sensor {} theta must be finite and within [0, 1]", sensor.sensor_id));
                }
                _ => {}
            }
            observations.push(ParsedObservation {
                timestamp,
                theta: sample.volumetric_water_content_m3_m3,
                status: sample.status,
            });
        }

        let times: Vec<_> = observations.iter().map(|sample| sample.timestamp.clone()).collect();
        let mut cadence = None;
        for pair in times.windows(2) {
            let delta = (pair[1].clone() - pair[0].clone()).num_seconds();
            if delta <= 0 || delta % 60 != 0 {
                return Err(format!("sensor {} timestamps must increase at whole-minute cadence", sensor.sensor_id));
            }
            let minutes = delta / 60;
            if let Some(expected) = cadence {
                if minutes != expected {
                    return Err(format!("sensor {} has irregular sampling cadence", sensor.sensor_id));
                }
            } else {
                cadence = Some(minutes);
            }
        }
        let cadence = cadence.ok_or_else(|| format!("sensor {} has no sampling cadence", sensor.sensor_id))?;
        if !(MIN_CADENCE_MINUTES..=MAX_CADENCE_MINUTES).contains(&cadence) {
            return Err(format!("sensor {} cadence must be between 5 and 30 minutes", sensor.sensor_id));
        }
        if let Some(expected) = common_cadence {
            if cadence != expected {
                return Err("all sensors in a profile must use the same sampling cadence".to_owned());
            }
        } else {
            common_cadence = Some(cadence);
        }
        if let Some(expected_times) = &common_times {
            if expected_times != &times {
                return Err("all sensors must share identical timestamps for a comparable profile event".to_owned());
            }
        } else {
            common_times = Some(times.clone());
        }

        let baseline_time = event_start - Duration::minutes(cadence);
        if !times.iter().any(|timestamp| timestamp == &baseline_time) {
            return Err(format!("sensor {} lacks the immediately preceding baseline timestamp", sensor.sensor_id));
        }
        if !times.iter().any(|timestamp| timestamp == &event_start) {
            return Err(format!("sensor {} lacks a sample at event onset", sensor.sensor_id));
        }
        if observations.last().map(|sample| sample.timestamp.clone() < event_end.clone()).unwrap_or(true) {
            return Err(format!("sensor {} does not cover the complete event observation window", sensor.sensor_id));
        }
        parsed_sensors.push(observations);
    }

    Ok((
        event_start,
        event_end,
        common_cadence.expect("at least two validated sensors"),
        parsed_sensors,
    ))
}

fn analyse_sensor(
    sensor: &SensorSeries,
    observations: &[ParsedObservation],
    event_start: &DateTime<FixedOffset>,
    event_end: &DateTime<FixedOffset>,
    cadence_minutes: i64,
    threshold: f64,
) -> SensorAnalysis {
    let baseline_time = event_start - Duration::minutes(cadence_minutes);
    let baseline = observations.iter().find(|sample| sample.timestamp == baseline_time);
    let baseline_theta = baseline.and_then(|sample| sample.theta);
    let baseline_valid = baseline
        .map(|sample| !matches!(sample.status, ObservationStatus::Missing))
        .unwrap_or(false);

    let mut response_status = "not_detected";
    let mut response_onset = None;
    let mut response_lag_minutes = None;
    let mut missing_event_bins = 0usize;
    let mut interpolated_event_bins = 0usize;
    let mut maximum_event_theta: Option<f64> = None;

    let event_samples: Vec<_> = observations
        .iter()
        .filter(|sample| sample.timestamp.clone() >= event_start.clone() && sample.timestamp.clone() <= event_end.clone())
        .collect();
    for sample in &event_samples {
        if matches!(sample.status, ObservationStatus::Missing) {
            missing_event_bins += 1;
        } else {
            if matches!(sample.status, ObservationStatus::InterpolatedGapLt6h) {
                interpolated_event_bins += 1;
            }
            if let Some(theta) = sample.theta {
                maximum_event_theta =
                    Some(maximum_event_theta.map(|value| value.max(theta)).unwrap_or(theta));
            }
        }
    }

    if !baseline_valid || baseline_theta.is_none() {
        response_status = "indeterminate_missing_baseline";
    } else {
        let base = baseline_theta.expect("validated baseline");
        let mut blocked_by_gap = false;
        for sample in &event_samples {
            if matches!(sample.status, ObservationStatus::Missing) {
                blocked_by_gap = true;
                break;
            }
            let theta = sample.theta.expect("non-missing observations have theta");
            if theta - base >= threshold {
                response_status = "detected";
                response_onset = Some(sample.timestamp.clone());
                response_lag_minutes =
                    Some((sample.timestamp.clone() - event_start.clone()).num_minutes());
                break;
            }
        }
        if blocked_by_gap && response_onset.is_none() {
            response_status = "indeterminate_data_gap";
        } else if response_onset.is_none() {
            response_status = "not_detected";
        }
    }

    let maximum_event_increase = match (baseline_theta, maximum_event_theta) {
        (Some(base), Some(maximum)) => Some((maximum - base).max(0.0)),
        _ => None,
    };

    let mut maximum_rise_rate: Option<f64> = None;
    let mut maximum_rise_start = None;
    let mut maximum_rise_end = None;
    let mut valid_rise_intervals = 0usize;

    for pair in observations.windows(2) {
        let start_in_event = pair[0].timestamp.clone() >= event_start.clone() && pair[0].timestamp.clone() < event_end.clone();
        let end_in_event = pair[1].timestamp.clone() > event_start.clone() && pair[1].timestamp.clone() <= event_end.clone();
        if !start_in_event || !end_in_event {
            continue;
        }
        let (Some(theta_start), Some(theta_end)) = (pair[0].theta, pair[1].theta) else {
            continue;
        };
        if matches!(pair[0].status, ObservationStatus::Missing)
            || matches!(pair[1].status, ObservationStatus::Missing)
        {
            continue;
        }
        valid_rise_intervals += 1;
        let elapsed_hours = (pair[1].timestamp.clone() - pair[0].timestamp.clone()).num_seconds() as f64 / 3600.0;
        let rate = (theta_end - theta_start) / elapsed_hours;
        if rate > 0.0 && rate > maximum_rise_rate.unwrap_or(f64::NEG_INFINITY) {
            maximum_rise_rate = Some(rate);
            maximum_rise_start = Some(pair[0].timestamp.clone());
            maximum_rise_end = Some(pair[1].timestamp.clone());
        }
    }

    SensorAnalysis {
        sensor_id: sensor.sensor_id.clone(),
        depth_cm: sensor.depth_cm,
        response_status,
        response_onset,
        response_lag_minutes,
        baseline_theta,
        maximum_event_increase,
        maximum_rise_rate,
        maximum_rise_start,
        maximum_rise_end,
        valid_rise_intervals,
        missing_event_bins,
        interpolated_event_bins,
    }
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (event_start, event_end, cadence_minutes, parsed) = parse_and_validate(input)?;
    let mut analysed = Vec::with_capacity(input.sensors.len());
    for (sensor, observations) in input.sensors.iter().zip(parsed.iter()) {
        analysed.push(analyse_sensor(
            sensor,
            observations,
            &event_start,
            &event_end,
            cadence_minutes,
            input.response_threshold_m3_m3,
        ));
    }
    analysed.sort_by(|left, right| left.depth_cm.total_cmp(&right.depth_cm));

    let mut pair_results = Vec::new();
    let mut nonsequential_pair_count = 0usize;
    let mut sequential_pair_count = 0usize;
    let mut simultaneous_pair_count = 0usize;
    let responding: Vec<_> = analysed
        .iter()
        .filter(|sensor| sensor.response_onset.is_some())
        .collect();

    for shallow_index in 0..analysed.len() {
        for deep_index in (shallow_index + 1)..analysed.len() {
            let shallow = &analysed[shallow_index];
            let deep = &analysed[deep_index];
            let (Some(shallow_time), Some(deep_time)) =
                (shallow.response_onset.as_ref(), deep.response_onset.as_ref())
            else {
                continue;
            };
            let delta = (deep_time.clone() - shallow_time.clone()).num_minutes();
            let classification = if delta < 0 {
                nonsequential_pair_count += 1;
                "nonsequential_deeper_first"
            } else if delta > 0 {
                sequential_pair_count += 1;
                "sequential_shallow_first"
            } else {
                simultaneous_pair_count += 1;
                "simultaneous_within_sampling_resolution"
            };
            pair_results.push(PairResult {
                shallow_sensor_id: shallow.sensor_id.clone(),
                shallow_depth_cm: shallow.depth_cm,
                shallow_response_utc: shallow_time.to_rfc3339(),
                deep_sensor_id: deep.sensor_id.clone(),
                deep_depth_cm: deep.depth_cm,
                deep_response_utc: deep_time.to_rfc3339(),
                deep_minus_shallow_response_minutes: delta,
                classification: classification.to_owned(),
            });
        }
    }

    let indeterminate_sensor_count = analysed
        .iter()
        .filter(|sensor| sensor.response_status.starts_with("indeterminate"))
        .count();

    let profile_signature_status = if nonsequential_pair_count > 0 {
        "nsr_signature_detected"
    } else if responding.len() >= 2 && indeterminate_sensor_count == 0 {
        "no_nsr_signature_observed"
    } else if indeterminate_sensor_count > 0 {
        "indeterminate_incomplete_sensor_coverage"
    } else {
        "indeterminate_fewer_than_two_responding_depths"
    };

    let sensor_results: Vec<SensorResult> = analysed
        .iter()
        .map(|sensor| SensorResult {
            sensor_id: sensor.sensor_id.clone(),
            depth_cm: sensor.depth_cm,
            response_status: sensor.response_status.to_owned(),
            response_onset_utc: sensor.response_onset.as_ref().map(|timestamp| timestamp.to_rfc3339()),
            response_lag_minutes: sensor.response_lag_minutes,
            baseline_theta_m3_m3: sensor.baseline_theta,
            maximum_event_increase_m3_m3: sensor.maximum_event_increase,
            maximum_rise_rate_m3_m3_per_h: sensor.maximum_rise_rate,
            maximum_rise_interval_start_utc: sensor.maximum_rise_start.as_ref().map(|timestamp| timestamp.to_rfc3339()),
            maximum_rise_interval_end_utc: sensor.maximum_rise_end.as_ref().map(|timestamp| timestamp.to_rfc3339()),
            valid_rise_intervals: sensor.valid_rise_intervals,
            missing_event_bins: sensor.missing_event_bins,
            interpolated_event_bins: sensor.interpolated_event_bins,
        })
        .collect();

    Ok(Report {
        audit_version: "0.1.0",
        input_sha256,
        site_id: input.site_id.clone(),
        plot_id: input.plot_id.clone(),
        profile_id: input.profile_id.clone(),
        event_id: input.event_id.clone(),
        event_start_utc: event_start.to_rfc3339(),
        event_end_utc: event_end.to_rfc3339(),
        cadence_minutes,
        response_threshold_m3_m3: input.response_threshold_m3_m3,
        sensor_count: analysed.len(),
        responding_sensor_count: responding.len(),
        indeterminate_sensor_count,
        comparable_depth_pair_count: pair_results.len(),
        nonsequential_pair_count,
        sequential_pair_count,
        simultaneous_pair_count,
        profile_signature_status: profile_signature_status.to_owned(),
        sensors: sensor_results,
        pair_results,
        interpretation_boundary: "NSR is a profile-level soil-moisture signature, not a measurement of preferential-flow volume, pore geometry, tracer breakthrough or treatment effect; absence of NSR does not establish absence of preferential flow",
        warnings: vec![
            "Response threshold is explicit input and must be sensitivity-tested; it is not a universal soil-moisture threshold.".to_owned(),
            "The immediately preceding sample is the baseline. Confirmatory reproduction must match the selected study's exact response-onset preprocessing.".to_owned(),
            "NSR can be conservative: a preferential pathway may bypass uninstrumented layers or miss point sensors.".to_owned(),
            "A fast sequential response may arise from piston displacement or rising water table; NSR status is not an independent physical certificate.".to_owned(),
            "Maximum rate of rise is continuous descriptive evidence and is not converted into a binary preferential-flow label.".to_owned(),
            "Missing samples make a sensor's response timing indeterminate when they occur before any observed threshold crossing.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: terra-preta-profile-signature-audit <profile-event.json> <new-report.json>",
        )
        .into());
    }
    let input_bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&input_bytes));
    let input: Input = serde_json::from_slice(&input_bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])
        .map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("refusing overwrite or cannot create report: {error}"),
            )
        })?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "profile signature audit complete: {} sensors; {} responding; {} nonsequential pairs; status={}; input SHA-256 {}",
        report.sensor_count,
        report.responding_sensor_count,
        report.nonsequential_pair_count,
        report.profile_signature_status,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("profile signature audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(minutes: i64) -> String {
        let origin = DateTime::parse_from_rfc3339("2026-01-01T00:00:00Z").unwrap();
        (origin + Duration::minutes(minutes)).to_rfc3339()
    }

    fn observations(response_minute: Option<i64>) -> Vec<Observation> {
        (-10..=30)
            .step_by(10)
            .map(|minute| Observation {
                timestamp_utc: stamp(minute),
                volumetric_water_content_m3_m3: Some(
                    if response_minute.is_some_and(|response| minute >= response) {
                        0.22
                    } else {
                        0.20
                    },
                ),
                status: ObservationStatus::ObservedGood,
            })
            .collect()
    }

    fn fixture(shallow_response: Option<i64>, deep_response: Option<i64>) -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            event_id: "EVENT-01".to_owned(),
            event_start_utc: stamp(0),
            event_end_utc: stamp(30),
            response_threshold_m3_m3: 0.01,
            sensors: vec![
                SensorSeries {
                    sensor_id: "SHALLOW".to_owned(),
                    depth_cm: 10.0,
                    observations: observations(shallow_response),
                },
                SensorSeries {
                    sensor_id: "DEEP".to_owned(),
                    depth_cm: 40.0,
                    observations: observations(deep_response),
                },
            ],
        }
    }

    #[test]
    fn flags_deeper_sensor_responding_before_shallower_sensor() {
        let report = evaluate(&fixture(Some(20), Some(10)), "a".repeat(64)).unwrap();
        assert_eq!(report.profile_signature_status, "nsr_signature_detected");
        assert_eq!(report.nonsequential_pair_count, 1);
        assert_eq!(
            report.pair_results[0].classification,
            "nonsequential_deeper_first"
        );
        assert_eq!(
            report.pair_results[0].deep_minus_shallow_response_minutes,
            -10
        );
    }

    #[test]
    fn sequential_response_is_not_promoted_to_absence_of_preferential_flow() {
        let report = evaluate(&fixture(Some(10), Some(20)), "b".repeat(64)).unwrap();
        assert_eq!(report.profile_signature_status, "no_nsr_signature_observed");
        assert_eq!(report.sequential_pair_count, 1);
        assert!(report.interpretation_boundary.contains("absence of preferential flow"));
    }

    #[test]
    fn equal_response_bins_are_simultaneous_not_nonsequential() {
        let report = evaluate(&fixture(Some(10), Some(10)), "c".repeat(64)).unwrap();
        assert_eq!(report.simultaneous_pair_count, 1);
        assert_eq!(
            report.pair_results[0].classification,
            "simultaneous_within_sampling_resolution"
        );
        assert_eq!(report.profile_signature_status, "no_nsr_signature_observed");
    }

    #[test]
    fn one_responding_depth_is_indeterminate_not_negative() {
        let report = evaluate(&fixture(Some(10), None), "d".repeat(64)).unwrap();
        assert_eq!(
            report.profile_signature_status,
            "indeterminate_fewer_than_two_responding_depths"
        );
    }

    #[test]
    fn missing_bin_before_response_makes_profile_indeterminate() {
        let mut input = fixture(Some(20), Some(10));
        let deep = input.sensors.iter_mut().find(|sensor| sensor.sensor_id == "DEEP").unwrap();
        let sample = deep.observations.iter_mut().find(|sample| sample.timestamp_utc == stamp(0)).unwrap();
        sample.status = ObservationStatus::Missing;
        sample.volumetric_water_content_m3_m3 = None;
        let report = evaluate(&input, "e".repeat(64)).unwrap();
        assert_eq!(
            report.profile_signature_status,
            "indeterminate_incomplete_sensor_coverage"
        );
        assert_eq!(report.indeterminate_sensor_count, 1);
    }

    #[test]
    fn computes_maximum_rise_rate_without_turning_it_into_a_pfet_label() {
        let report = evaluate(&fixture(Some(20), Some(10)), "f".repeat(64)).unwrap();
        let shallow = report.sensors.iter().find(|sensor| sensor.sensor_id == "SHALLOW").unwrap();
        assert!((shallow.maximum_rise_rate_m3_m3_per_h.unwrap() - 0.12).abs() < 1e-9);
        assert!((shallow.maximum_event_increase_m3_m3.unwrap() - 0.02).abs() < 1e-9);
    }

    #[test]
    fn no_positive_rise_produces_no_rate_estimate() {
        let report = evaluate(&fixture(None, None), "9".repeat(64)).unwrap();
        assert!(report.sensors.iter().all(|sensor| sensor.maximum_rise_rate_m3_m3_per_h.is_none()));
    }

    #[test]
    fn rejects_duplicate_depths() {
        let mut input = fixture(Some(10), Some(20));
        input.sensors[1].depth_cm = input.sensors[0].depth_cm;
        assert!(parse_and_validate(&input).unwrap_err().contains("duplicate sensor depth"));
    }

    #[test]
    fn rejects_irregular_cadence() {
        let mut input = fixture(Some(10), Some(20));
        input.sensors[1].observations[2].timestamp_utc = stamp(15);
        assert!(parse_and_validate(&input).unwrap_err().contains("irregular sampling cadence"));
    }

    #[test]
    fn rejects_non_utc_offsets() {
        let mut input = fixture(Some(10), Some(20));
        input.sensors[0].observations[0].timestamp_utc = "2025-12-31T17:50:00-06:00".to_owned();
        assert!(parse_and_validate(&input).unwrap_err().contains("zero UTC offset"));
    }
}

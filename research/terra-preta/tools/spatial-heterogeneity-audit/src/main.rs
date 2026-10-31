use chrono::{DateTime, Duration, FixedOffset};
#[cfg(test)]
use chrono::{TimeZone, Utc};
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

const MIN_RESOLUTION_SECONDS: u32 = 300;
const MAX_RESOLUTION_SECONDS: u32 = 1800;
const MAX_WINDOW_SECONDS: i64 = 6 * 24 * 60 * 60;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum SampleStatus {
    Observed,
    LinearInterpolatedGapLt6h,
    Missing,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum SensorCondition {
    Usable,
    Saturated,
    Frozen,
    SensorFault,
    Unknown,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    response_threshold_m3_m3: f64,
    source_evidence_ref: String,
    distance_bins: Vec<DistanceBin>,
    sensors: Vec<SensorSeries>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DistanceBin {
    lower_exclusive_m: f64,
    upper_inclusive_m: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorSeries {
    x_m: f64,
    y_m: f64,
    depth_cm: f64,
    condition: SensorCondition,
    observations: Vec<Observation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    timestamp_utc: String,
    volumetric_water_content_m3_m3: Option<f64>,
    status: SampleStatus,
}

#[derive(Clone, Debug)]
struct ParsedObservation {
    time: DateTime<FixedOffset>,
    theta: Option<f64>,
    status: SampleStatus,
}

#[derive(Clone, Debug)]
struct ParsedSensor {
    sensor_id: String,
    x_m: f64,
    y_m: f64,
    depth_cm: f64,
    condition: SensorCondition,
    baseline_theta: Option<f64>,
    baseline_status: SampleStatus,
    observations: Vec<ParsedObservation>,
}

#[derive(Clone, Debug, Serialize)]
struct SemivariogramBin {
    lower_exclusive_m: f64,
    upper_inclusive_m: f64,
    pair_count: usize,
    semivariance_theta_squared: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct DepthTimeSummary {
    timestamp_utc: String,
    depth_cm: f64,
    roster_sensor_count: usize,
    valid_observed_sensor_count: usize,
    non_usable_condition_count: usize,
    invalid_baseline_count: usize,
    missing_sample_count: usize,
    interpolated_sample_count: usize,
    observed_sample_count_excluded_by_condition: usize,
    distinct_valid_position_count: usize,
    positive_response_count: usize,
    positive_response_fraction: Option<f64>,
    mean_delta_theta_m3_m3: Option<f64>,
    median_delta_theta_m3_m3: Option<f64>,
    population_std_delta_theta_m3_m3: Option<f64>,
    interquartile_range_delta_theta_m3_m3: Option<f64>,
    median_absolute_deviation_delta_theta_m3_m3: Option<f64>,
    robust_coefficient_of_variation: Option<f64>,
    positive_max_to_median_ratio: Option<f64>,
    minimum_delta_theta_m3_m3: Option<f64>,
    maximum_delta_theta_m3_m3: Option<f64>,
    semivariogram: Vec<SemivariogramBin>,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    response_threshold_m3_m3: f64,
    sensor_count: usize,
    distinct_depth_count: usize,
    time_depth_summary_count: usize,
    summaries: Vec<DepthTimeSummary>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn parse_utc(
    value: &str,
    label: &str,
    resolution_seconds: u32,
) -> Result<DateTime<FixedOffset>, String> {
    let parsed = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if parsed.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp must use a zero UTC offset"));
    }
    let step = i64::from(resolution_seconds);
    if parsed.timestamp().rem_euclid(step) != 0 || parsed.timestamp_subsec_nanos() != 0 {
        return Err(format!(
            "{label}: timestamp must align to the declared {resolution_seconds}-second UTC grid"
        ));
    }
    Ok(parsed)
}

fn validate_theta(
    theta: Option<f64>,
    status: SampleStatus,
    label: &str,
) -> Result<(), String> {
    match (status, theta) {
        (SampleStatus::Missing, None) => Ok(()),
        (SampleStatus::Missing, Some(_)) => {
            Err(format!("{label}: missing status must have a null theta value"))
        }
        (SampleStatus::Observed | SampleStatus::LinearInterpolatedGapLt6h, None) => {
            Err(format!("{label}: observed/interpolated status requires a theta value"))
        }
        (_, Some(value)) if !value.is_finite() || !(0.0..=1.0).contains(&value) => {
            Err(format!("{label}: theta must be finite and within [0, 1]"))
        }
        (_, Some(_)) => Ok(()),
    }
}

fn validate(input: &Input) -> Result<(Vec<DateTime<FixedOffset>>, Vec<ParsedSensor>), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.plot_id, "plot_id"),
        (&input.profile_id, "profile_id"),
        (&input.event_id, "event_id"),
        (&input.source_evidence_ref, "source_evidence_ref"),
    ] {
        if value.trim().is_empty() {
            return Err(format!("{label} must not be blank"));
        }
    }
    if !(MIN_RESOLUTION_SECONDS..=MAX_RESOLUTION_SECONDS)
        .contains(&input.time_resolution_seconds)
    {
        return Err(format!(
            "time_resolution_seconds must be between {MIN_RESOLUTION_SECONDS} and {MAX_RESOLUTION_SECONDS}"
        ));
    }
    if !input.response_threshold_m3_m3.is_finite()
        || input.response_threshold_m3_m3 <= 0.0
        || input.response_threshold_m3_m3 > 1.0
    {
        return Err("response_threshold_m3_m3 must be finite and in (0, 1]".to_owned());
    }
    if input.sensors.len() < 2 {
        return Err("at least two sensors are required to quantify spatial heterogeneity".to_owned());
    }
    if input.distance_bins.is_empty() {
        return Err("at least one distance bin is required".to_owned());
    }

    let event_start = parse_utc(
        &input.event_start_utc,
        "event_start_utc",
        input.time_resolution_seconds,
    )?;
    let observation_end = parse_utc(
        &input.observation_window_end_utc,
        "observation_window_end_utc",
        input.time_resolution_seconds,
    )?;
    if event_start >= observation_end {
        return Err("observation_window_end_utc must be after event_start_utc".to_owned());
    }
    if (observation_end.clone() - event_start.clone()).num_seconds() > MAX_WINDOW_SECONDS {
        return Err("observation window must not exceed six days in v0.1".to_owned());
    }

    let mut previous_upper = 0.0;
    for (index, bin) in input.distance_bins.iter().enumerate() {
        if !bin.lower_exclusive_m.is_finite()
            || !bin.upper_inclusive_m.is_finite()
            || bin.lower_exclusive_m < 0.0
            || bin.upper_inclusive_m <= bin.lower_exclusive_m
            || (index > 0 && bin.lower_exclusive_m < previous_upper)
        {
            return Err(format!("distance_bins[{index}] must be finite, positive-width, ordered and non-overlapping"));
        }
        previous_upper = bin.upper_inclusive_m;
    }

    let step = i64::from(input.time_resolution_seconds);
    let first_time = event_start.clone() - Duration::seconds(step);
    let interval_seconds =
        (observation_end.clone() - first_time.clone()).num_seconds();
    if interval_seconds < 0 || interval_seconds % step != 0 {
        return Err("event window does not align with the declared sampling cadence".to_owned());
    }
    let sample_count = (interval_seconds / step + 1) as usize;
    let mut expected_times = Vec::with_capacity(sample_count);
    for index in 0..sample_count {
        expected_times.push(first_time.clone() + Duration::seconds(step * index as i64));
    }

    let mut ids = HashSet::new();
    let mut parsed_sensors = Vec::with_capacity(input.sensors.len());
    for (sensor_index, sensor) in input.sensors.iter().enumerate() {
        if sensor.sensor_id.trim().is_empty() {
            return Err(format!("sensors[{sensor_index}].sensor_id must not be blank"));
        }
        if !ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        for (value, label) in [
            (sensor.x_m, "x_m"),
            (sensor.y_m, "y_m"),
            (sensor.depth_cm, "depth_cm"),
        ] {
            if !value.is_finite() {
                return Err(format!("sensors[{sensor_index}].{label} must be finite"));
            }
        }
        if !sensor.depth_cm.is_finite() || sensor.depth_cm <= 0.0 {
            return Err(format!("sensors[{sensor_index}].depth_cm must be greater than zero"));
        }
        if sensor.observations.len() != sample_count {
            return Err(format!(
                "sensors[{sensor_index}] has {} samples; expected {sample_count} shared-grid samples",
                sensor.observations.len()
            ));
        }

        let mut observations = Vec::with_capacity(sample_count);
        for (sample_index, sample) in sensor.observations.iter().enumerate() {
            let time = parse_utc(
                &sample.timestamp_utc,
                &format!("sensors[{sensor_index}].observations[{sample_index}].timestamp_utc"),
                input.time_resolution_seconds,
            )?;
            if time != expected_times[sample_index] {
                return Err(format!(
                    "sensors[{sensor_index}].observations[{sample_index}] is not on the shared profile grid"
                ));
            }
            validate_theta(
                sample.volumetric_water_content_m3_m3,
                sample.status,
                &format!("sensors[{sensor_index}].observations[{sample_index}]"),
            )?;
            observations.push(ParsedObservation {
                time,
                theta: sample.volumetric_water_content_m3_m3,
                status: sample.status,
            });
        }

        let baseline = &observations[0];
        parsed_sensors.push(ParsedSensor {
            x_m: sensor.x_m,
            y_m: sensor.y_m,
            depth_cm: sensor.depth_cm,
            condition: sensor.condition,
            baseline_theta: baseline.theta,
            baseline_status: baseline.status,
            observations,
        });
    }
    Ok((expected_times, parsed_sensors))
}

fn quantile(sorted: &[f64], probability: f64) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    if sorted.len() == 1 {
        return Some(sorted[0]);
    }
    let position = probability * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let fraction = position - lower as f64;
    Some(sorted[lower] + fraction * (sorted[upper] - sorted[lower]))
}

fn median(values: &[f64]) -> Option<f64> {
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    quantile(&sorted, 0.5)
}

fn semivariogram(
    sensors: &[&ParsedSensor],
    deltas: &[Option<f64>],
    distance_bins: &[DistanceBin],
) -> Vec<SemivariogramBin> {
    distance_bins
        .iter()
        .map(|bin| {
            let mut sum = 0.0;
            let mut count = 0usize;
            for left_index in 0..sensors.len() {
                let Some(left_delta) = deltas[left_index] else {
                    continue;
                };
                for right_index in (left_index + 1)..sensors.len() {
                    let Some(right_delta) = deltas[right_index] else {
                        continue;
                    };
                    let dx = sensors[left_index].x_m - sensors[right_index].x_m;
                    let dy = sensors[left_index].y_m - sensors[right_index].y_m;
                    let distance = dx.hypot(dy);
                    if distance > bin.lower_exclusive_m && distance <= bin.upper_inclusive_m {
                        sum += 0.5 * (left_delta - right_delta).powi(2);
                        count += 1;
                    }
                }
            }
            SemivariogramBin {
                lower_exclusive_m: bin.lower_exclusive_m,
                upper_inclusive_m: bin.upper_inclusive_m,
                pair_count: count,
                semivariance_m6_m6: (count > 0).then_some(sum / count as f64),
            }
        })
        .collect()
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (expected_times, sensors) = validate(input)?;
    let mut depths: Vec<f64> = sensors.iter().map(|sensor| sensor.depth_cm).collect();
    depths.sort_by(f64::total_cmp);
    depths.dedup_by(|left, right| left.total_cmp(right).is_eq());

    let mut summaries = Vec::new();
    for time in expected_times.iter().skip(1) {
        for depth in &depths {
            let depth_sensors: Vec<&ParsedSensor> = sensors
                .iter()
                .filter(|sensor| sensor.depth_cm == *depth)
                .collect();
            let mut deltas = Vec::with_capacity(depth_sensors.len());
            let mut valid_values = Vec::new();
            let mut distinct_positions: Vec<(f64, f64)> = Vec::new();
            let mut non_usable_count = 0usize;
            let mut invalid_baseline_count = 0usize;
            let mut missing_count = 0usize;
            let mut interpolated_count = 0usize;
            let mut observed_excluded_by_condition = 0usize;

            for sensor in &depth_sensors {
                let sample = sensor
                    .observations
                    .iter()
                    .find(|observation| observation.time == *time)
                    .expect("validated shared time grid includes each summary time");
                let delta = match sensor.condition {
                    SensorCondition::Saturated
                    | SensorCondition::Frozen
                    | SensorCondition::SensorFault
                    | SensorCondition::Unknown => {
                        non_usable_count += 1;
                        if sample.status == SampleStatus::Observed {
                            observed_excluded_by_condition += 1;
                        }
                        None
                    }
                    SensorCondition::Usable => {
                        if sensor.baseline_status != SampleStatus::Observed
                            || sensor.baseline_theta.is_none()
                        {
                            invalid_baseline_count += 1;
                            None
                        } else {
                            match sample.status {
                                SampleStatus::Missing => {
                                    missing_count += 1;
                                    None
                                }
                                SampleStatus::LinearInterpolatedGapLt6h => {
                                    interpolated_count += 1;
                                    None
                                }
                                SampleStatus::Observed => {
                                    sample.theta.map(|theta| theta - sensor.baseline_theta.unwrap())
                                }
                            }
                        }
                    }
                };
                if let Some(value) = delta {
                    valid_values.push(value);
                    distinct_positions.push((sensor.x_m, sensor.y_m));
                }
                deltas.push(delta);
            }

            distinct_positions.sort_by(|a, b| {
                a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1))
            });
            distinct_positions.dedup_by(|a, b| a.0.total_cmp(&b.0).is_eq() && a.1.total_cmp(&b.1).is_eq());

            let mut sorted = valid_values.clone();
            sorted.sort_by(f64::total_cmp);
            let mean = (!sorted.is_empty())
                .then(|| sorted.iter().sum::<f64>() / sorted.len() as f64);
            let median_value = quantile(&sorted, 0.5);
            let q1 = quantile(&sorted, 0.25);
            let q3 = quantile(&sorted, 0.75);
            let mad = median_value.map(|centre| {
                let deviations: Vec<f64> = sorted.iter().map(|value| (value - centre).abs()).collect();
                median(&deviations).unwrap_or(0.0)
            });
            let population_std = mean.map(|average| {
                (sorted.iter().map(|value| (value - average).powi(2)).sum::<f64>()
                    / sorted.len() as f64)
                    .sqrt()
            });
            let positive_values: Vec<f64> = sorted.iter().copied().filter(|value| *value > 0.0).collect();
            let positive_median = median(&positive_values);
            let positive_max = positive_values.iter().copied().max_by(f64::total_cmp);
            let positive_ratio = match (positive_max, positive_median) {
                (Some(maximum), Some(median_positive)) if median_positive > 0.0 => {
                    Some(maximum / median_positive)
                }
                _ => None,
            };
            let robust_cv = match (mad, median_value) {
                (Some(deviation), Some(centre)) if centre.abs() > f64::EPSILON => {
                    Some(1.4826 * deviation / centre.abs())
                }
                _ => None,
            };
            let positive_count = valid_values
                .iter()
                .filter(|value| **value >= input.response_threshold_m3_m3)
                .count();
            let valid_count = valid_values.len();
            let positive_fraction = (valid_count > 0)
                .then_some(positive_count as f64 / valid_count as f64);

            summaries.push(DepthTimeSummary {
                timestamp_utc: time.to_rfc3339(),
                depth_cm: *depth,
                roster_sensor_count: depth_sensors.len(),
                valid_observed_sensor_count: valid_count,
                non_usable_condition_count: non_usable_count,
                invalid_baseline_count,
                missing_sample_count: missing_count,
                interpolated_sample_count: interpolated_count,
                observed_sample_count_excluded_by_condition: observed_excluded_by_condition,
                distinct_valid_position_count: distinct_positions.len(),
                positive_response_count: positive_count,
                positive_response_fraction: positive_fraction,
                mean_delta_theta_m3_m3: mean,
                median_delta_theta_m3_m3: median_value,
                population_std_delta_theta_m3_m3: population_std,
                interquartile_range_delta_theta_m3_m3: q1.zip(q3).map(|(lower, upper)| upper - lower),
                median_absolute_deviation_delta_theta_m3_m3: mad,
                robust_coefficient_of_variation: robust_cv,
                positive_max_to_median_ratio: positive_ratio,
                minimum_delta_theta_m3_m3: sorted.first().copied(),
                maximum_delta_theta_m3_m3: sorted.last().copied(),
                semivariogram: semivariogram(&depth_sensors, &deltas, &input.distance_bins),
            });
        }
    }

    Ok(Report {
        audit_version: "0.1.0",
        input_sha256,
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        plot_id: input.plot_id.clone(),
        profile_id: input.profile_id.clone(),
        event_id: input.event_id.clone(),
        event_start_utc: input.event_start_utc.clone(),
        observation_window_end_utc: input.observation_window_end_utc.clone(),
        time_resolution_seconds: input.time_resolution_seconds,
        response_threshold_m3_m3: input.response_threshold_m3_m3,
        sensor_count: sensors.len(),
        distinct_depth_count: depths.len(),
        time_depth_summary_count: summaries.len(),
        summaries,
        interpretation_boundary: "event-relative spatial moisture heterogeneity is a sensor signature, not unique proof of macropore connectivity, vertical bypass or measured flux",
        warnings: vec![
            "The audit uses only directly observed samples from usable sensors; missing and interpolated observations are excluded rather than imputed.".to_owned(),
            "Semivariograms are descriptive and require adequate spatial layout; pair counts are emitted and no variogram model is fitted.".to_owned(),
            "Spatial heterogeneity can reflect texture, roots, lateral flow, sensor support, calibration or drift; it is not a preferential-flow certificate.".to_owned(),
            "Semivariance has units of (m3/m3)^2; field name uses the equivalent m6/m6 dimensional notation.".to_owned(),
            "The sensor roster defines the sampled support. Results must not be extrapolated to unobserved space without a declared spatial model.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: spatial-heterogeneity-audit <normalized-profile-event.json> <new-report.json>",
        ).into());
    }
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])
        .map_err(|error| io::Error::new(
            error.kind(),
            format!("refusing overwrite or cannot create output {}: {error}", args[2]),
        ))?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "spatial audit complete: {} sensors; {} depths; {} time-depth summaries; input_sha256={}",
        report.sensor_count,
        report.distinct_depth_count,
        report.time_depth_summary_count,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("spatial heterogeneity audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(minutes: i64) -> String {
        let origin = Utc.with_ymd_and_hms(2026, 5, 1, 0, 0, 0).single().unwrap();
        (origin + Duration::minutes(minutes)).to_rfc3339()
    }

    fn sensor(id: &str, x: f64, y: f64, values: [f64; 3]) -> SensorSeries {
        SensorSeries {
            sensor_id: id.to_owned(),
            x_m: x,
            y_m: y,
            depth_cm: 20.0,
            condition: SensorCondition::Usable,
            observations: [-10, 0, 10].into_iter().enumerate().map(|(index, minute)| Observation {
                timestamp_utc: stamp(minute),
                volumetric_water_content_m3_m3: Some(values[index]),
                status: SampleStatus::Observed,
            }).collect(),
        }
    }

    fn fixture() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "synthetic".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROF1".to_owned(),
            event_id: "EV1".to_owned(),
            event_start_utc: stamp(0),
            observation_window_end_utc: stamp(10),
            time_resolution_seconds: 600,
            response_threshold_m3_m3: 0.01,
            source_evidence_ref: "synthetic fixture; not field data".to_owned(),
            distance_bins: vec![
                DistanceBin { lower_exclusive_m: 0.0, upper_inclusive_m: 1.1 },
                DistanceBin { lower_exclusive_m: 1.1, upper_inclusive_m: 2.1 },
                DistanceBin { lower_exclusive_m: 2.1, upper_inclusive_m: 3.0 },
            ],
            sensors: vec![
                sensor("A", 0.0, 0.0, [0.20, 0.20, 0.22]),
                sensor("B", 1.0, 0.0, [0.20, 0.20, 0.20]),
                sensor("C", 0.0, 1.0, [0.20, 0.20, 0.21]),
                sensor("D", 1.0, 1.0, [0.20, 0.20, 0.20]),
            ],
        }
    }

    #[test]
    fn reports_event_relative_heterogeneity_and_semivariogram_support() {
        let input = fixture();
        let report = evaluate(&input, "a".repeat(64)).unwrap();
        assert_eq!(report.sensor_count, 4);
        assert_eq!(report.distinct_depth_count, 1);
        assert_eq!(report.time_depth_summary_count, 2);
        let last = &report.summaries[1];
        assert_eq!(last.valid_observed_sensor_count, 4);
        assert_eq!(last.positive_response_count, 2);
        assert_eq!(last.positive_response_fraction, Some(0.5));
        assert_eq!(last.semivariogram.iter().map(|bin| bin.pair_count).sum::<usize>(), 6);
        assert!(last.median_absolute_deviation_delta_theta_m3_m3.unwrap() > 0.0);
    }

    #[test]
    fn homogeneous_matrix_only_control_has_zero_event_spatial_dispersion() {
        let mut input = fixture();
        for sensor in &mut input.sensors {
            for observation in sensor.observations.iter_mut().skip(1) {
                observation.volumetric_water_content_m3_m3 = Some(0.20);
            }
        }
        let report = evaluate(&input, "h".repeat(64)).unwrap();
        let last = &report.summaries[1];
        assert_eq!(last.positive_response_count, 0);
        assert_eq!(last.positive_response_fraction, Some(0.0));
        assert_eq!(last.minimum_delta_theta_m3_m3, Some(0.0));
        assert_eq!(last.maximum_delta_theta_m3_m3, Some(0.0));
        assert_eq!(last.population_std_delta_theta_m3_m3, Some(0.0));
        assert!(last.semivariogram.iter().all(|bin| {
            bin.semivariance_theta_squared.is_none_or(|value| value == 0.0)
        }));
    }

    #[test]
    fn sensor_order_does_not_change_spatial_statistics() {
        let original = fixture();
        let mut reordered = fixture();
        reordered.sensors.reverse();
        let first = evaluate(&original, "a".repeat(64)).unwrap();
        let second = evaluate(&reordered, "b".repeat(64)).unwrap();
        assert_eq!(first.summaries.len(), second.summaries.len());
        for (left, right) in first.summaries.iter().zip(&second.summaries) {
            assert_eq!(left.valid_observed_sensor_count, right.valid_observed_sensor_count);
            assert_eq!(left.positive_response_count, right.positive_response_count);
            assert_eq!(left.semivariogram.len(), right.semivariogram.len());
            for (a, b) in left.semivariogram.iter().zip(&right.semivariogram) {
                assert_eq!(a.pair_count, b.pair_count);
                assert!((a.semivariance_m6_m6.unwrap_or(0.0) - b.semivariance_m6_m6.unwrap_or(0.0)).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn translation_and_rotation_preserve_isotropic_semivariograms() {
        let original = fixture();
        let mut transformed = fixture();
        for sensor in &mut transformed.sensors {
            let x = sensor.x_m;
            let y = sensor.y_m;
            sensor.x_m = -y + 17.0;
            sensor.y_m = x - 9.0;
        }
        let first = evaluate(&original, "a".repeat(64)).unwrap();
        let second = evaluate(&transformed, "b".repeat(64)).unwrap();
        for (left, right) in first.summaries.iter().zip(&second.summaries) {
            for (a, b) in left.semivariogram.iter().zip(&right.semivariogram) {
                assert_eq!(a.pair_count, b.pair_count);
                assert!((a.semivariance_m6_m6.unwrap_or(0.0) - b.semivariance_m6_m6.unwrap_or(0.0)).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn missing_and_interpolated_rows_are_not_filled_with_zero() {
        let mut input = fixture();
        input.sensors[0].observations[2].status = SampleStatus::Missing;
        input.sensors[0].observations[2].volumetric_water_content_m3_m3 = None;
        input.sensors[1].observations[2].status = SampleStatus::LinearInterpolatedGapLt6h;
        let report = evaluate(&input, "a".repeat(64)).unwrap();
        let last = &report.summaries[1];
        assert_eq!(last.valid_observed_sensor_count, 2);
        assert_eq!(last.missing_sample_count, 1);
        assert_eq!(last.interpolated_sample_count, 1);
        assert_eq!(last.positive_response_count, 1);
    }

    #[test]
    fn invalid_pre_event_baselines_are_counted_not_treated_as_zero() {
        let mut input = fixture();
        input.sensors[0].observations[0].status = SampleStatus::LinearInterpolatedGapLt6h;
        let report = evaluate(&input, "i".repeat(64)).unwrap();
        let last = &report.summaries[1];
        assert_eq!(last.valid_observed_sensor_count, 3);
        assert_eq!(last.invalid_baseline_count, 1);
        assert_eq!(last.missing_sample_count, 0);
    }

    #[test]
    fn nonusable_sensors_are_explicitly_excluded() {
        let mut input = fixture();
        input.sensors[0].condition = SensorCondition::Saturated;
        let report = evaluate(&input, "a".repeat(64)).unwrap();
        assert_eq!(report.summaries[1].valid_observed_sensor_count, 3);
        assert_eq!(report.summaries[1].non_usable_condition_count, 1);
        assert_eq!(report.summaries[1].observed_sample_count_excluded_by_condition, 1);
    }

    #[test]
    fn rejects_overlapping_or_unordered_distance_bins() {
        let mut input = fixture();
        input.distance_bins[1].lower_exclusive_m = 0.9;
        assert!(validate(&input).unwrap_err().contains("ordered and non-overlapping"));
    }

    #[test]
    fn rejects_duplicate_sensor_ids_and_non_utc_timestamps() {
        let mut input = fixture();
        input.sensors[1].sensor_id = "A".to_owned();
        assert!(validate(&input).unwrap_err().contains("duplicate sensor_id"));
        let mut input = fixture();
        input.sensors[0].observations[0].timestamp_utc = "2026-05-01T02:50:00+03:00".to_owned();
        assert!(validate(&input).unwrap_err().contains("zero UTC offset"));
    }
}

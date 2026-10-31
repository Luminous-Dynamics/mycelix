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

const MIN_RESOLUTION_SECONDS: u32 = 300;
const MAX_RESOLUTION_SECONDS: u32 = 1800;
const MAX_OBSERVATION_WINDOW_SECONDS: i64 = 6 * 24 * 60 * 60;

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
    source_evidence_ref: String,
    sensors: Vec<SensorSeries>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorSeries {
    sensor_id: String,
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
    depth_cm: f64,
    condition: SensorCondition,
    observations: Vec<ParsedObservation>,
}

#[derive(Serialize)]
struct AuditSensor {
    sensor_id: String,
    depth_cm: f64,
    response_status: &'static str,
    response_onset_utc: Option<String>,
    response_onset_lower_bound_utc: Option<String>,
}

#[derive(Serialize)]
struct ProfileAuditInput {
    schema_version: &'static str,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    response_threshold_m3_m3: f64,
    response_onset_method: String,
    source_sha256: String,
    source_evidence_ref: String,
    sensors: Vec<AuditSensor>,
}

struct Response {
    status: &'static str,
    onset: Option<String>,
    lower_bound: Option<String>,
}

fn parse_utc(
    value: &str,
    label: &str,
    resolution_seconds: u32,
) -> Result<DateTime<FixedOffset>, String> {
    let parsed = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if parsed.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamps must use a zero UTC offset"));
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
        (SampleStatus::Missing, Some(_)) => Err(format!(
            "{label}: missing status must have a null water-content value"
        )),
        (SampleStatus::Observed | SampleStatus::LinearInterpolatedGapLt6h, None) => Err(
            format!("{label}: observed/interpolated status requires a water-content value"),
        ),
        (_, Some(value)) if !value.is_finite() || !(0.0..=1.0).contains(&value) => {
            Err(format!("{label}: water content must be finite and within [0, 1]"))
        }
        (_, Some(_)) => Ok(()),
    }
}

fn validate(
    input: &Input,
) -> Result<
    (
        DateTime<FixedOffset>,
        DateTime<FixedOffset>,
        Vec<ParsedSensor>,
    ),
    String,
> {
    if input.schema_version != "0.1.0" {
        return Err(format!(
            "unsupported schema_version {:?}; expected 0.1.0",
            input.schema_version
        ));
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
    if (observation_end.clone() - event_start.clone()).num_seconds()
        > MAX_OBSERVATION_WINDOW_SECONDS
    {
        return Err("observation window must not exceed six days in v0.1".to_owned());
    }
    if input.sensors.is_empty() {
        return Err("sensors must not be empty".to_owned());
    }

    let step = i64::from(input.time_resolution_seconds);
    let first_expected = event_start.clone() - Duration::seconds(step);
    let expected_count = ((observation_end.timestamp() - first_expected.timestamp()) / step + 1)
        as usize;
    let mut ids = HashSet::new();
    let mut parsed_sensors = Vec::with_capacity(input.sensors.len());
    let mut expected_times = Vec::with_capacity(expected_count);
    for index in 0..expected_count {
        expected_times.push(first_expected.clone() + Duration::seconds(step * index as i64));
    }

    for (sensor_index, sensor) in input.sensors.iter().enumerate() {
        if sensor.sensor_id.trim().is_empty() {
            return Err(format!("sensors[{sensor_index}].sensor_id must not be blank"));
        }
        if !ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        if !sensor.depth_cm.is_finite() || sensor.depth_cm <= 0.0 {
            return Err(format!(
                "sensors[{sensor_index}].depth_cm must be finite and greater than zero"
            ));
        }
        if sensor.observations.len() != expected_count {
            return Err(format!(
                "sensors[{sensor_index}] has {} observations; expected {expected_count} bins including the pre-event baseline",
                sensor.observations.len()
            ));
        }

        let mut observations = Vec::with_capacity(expected_count);
        for (sample_index, sample) in sensor.observations.iter().enumerate() {
            let time = parse_utc(
                &sample.timestamp_utc,
                &format!("sensors[{sensor_index}].observations[{sample_index}].timestamp_utc"),
                input.time_resolution_seconds,
            )?;
            if time != expected_times[sample_index] {
                return Err(format!(
                    "sensors[{sensor_index}].observations[{sample_index}] is off the shared profile grid; every sensor must include each bin from one pre-event baseline through the observation-window end"
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
        parsed_sensors.push(ParsedSensor {
            sensor_id: sensor.sensor_id.clone(),
            depth_cm: sensor.depth_cm,
            condition: sensor.condition,
            observations,
        });
    }
    Ok((event_start, observation_end, parsed_sensors))
}

fn extract_response(
    sensor: &ParsedSensor,
    event_start: DateTime<FixedOffset>,
    threshold: f64,
) -> Response {
    if sensor.condition != SensorCondition::Usable {
        return Response {
            status: "indeterminate",
            onset: None,
            lower_bound: None,
        };
    }

    let Some(baseline) = sensor.observations.first() else {
        return Response {
            status: "indeterminate",
            onset: None,
            lower_bound: None,
        };
    };
    let Some(baseline_theta) = baseline.theta else {
        return Response {
            status: "indeterminate",
            onset: None,
            lower_bound: None,
        };
    };
    if baseline.status != SampleStatus::Observed {
        return Response {
            status: "indeterminate",
            onset: None,
            lower_bound: None,
        };
    }

    let mut previous = baseline;
    for sample in sensor.observations.iter().skip(1) {
        if sample.time < event_start {
            previous = sample;
            continue;
        }
        match sample.status {
            SampleStatus::Missing => {
                return Response {
                    status: "indeterminate",
                    onset: None,
                    lower_bound: None,
                };
            }
            SampleStatus::LinearInterpolatedGapLt6h => {
                return Response {
                    status: "indeterminate",
                    onset: None,
                    lower_bound: None,
                };
            }
            SampleStatus::Observed => {}
        }
        let Some(theta) = sample.theta else {
            return Response {
                status: "indeterminate",
                onset: None,
                lower_bound: None,
            };
        };
        if theta - baseline_theta >= threshold {
            return Response {
                status: "detected",
                onset: Some(sample.time.to_rfc3339()),
                lower_bound: Some(previous.time.to_rfc3339()),
            };
        }
        previous = sample;
    }

    Response {
        status: "not_detected",
        onset: None,
        lower_bound: None,
    }
}

fn evaluate(
    input: &Input,
    input_sha256: String,
    threshold: f64,
) -> Result<(ProfileAuditInput, usize, usize, usize), String> {
    if !threshold.is_finite() || threshold <= 0.0 || threshold > 1.0 {
        return Err("threshold must be finite and within (0, 1] m3/m3".to_owned());
    }
    let (event_start, observation_end, sensors) = validate(input)?;
    let mut detected = 0usize;
    let mut not_detected = 0usize;
    let mut indeterminate = 0usize;
    let mut output_sensors = Vec::with_capacity(sensors.len());

    for sensor in &sensors {
        let response = extract_response(sensor, event_start.clone(), threshold);
        match response.status {
            "detected" => detected += 1,
            "not_detected" => not_detected += 1,
            _ => indeterminate += 1,
        }
        output_sensors.push(AuditSensor {
            sensor_id: sensor.sensor_id.clone(),
            depth_cm: sensor.depth_cm,
            response_status: response.status,
            response_onset_utc: response.onset,
            response_onset_lower_bound_utc: response.lower_bound,
        });
    }

    let _ = observation_end;
    Ok((
        ProfileAuditInput {
            schema_version: "0.3.0",
            study_id: input.study_id.clone(),
            site_id: input.site_id.clone(),
            plot_id: input.plot_id.clone(),
            profile_id: input.profile_id.clone(),
            event_id: input.event_id.clone(),
            event_start_utc: input.event_start_utc.clone(),
            observation_window_end_utc: input.observation_window_end_utc.clone(),
            time_resolution_seconds: input.time_resolution_seconds,
            response_threshold_m3_m3: threshold,
            response_onset_method: "baseline_delta_first_observed_crossing; interpolated_or_missing_pre_crossing_bins_indeterminate; response_intervals_are_last_below_to_first_above".to_owned(),
            source_sha256: input_sha256,
            source_evidence_ref: input.source_evidence_ref.clone(),
            sensors: output_sensors,
        },
        detected,
        not_detected,
        indeterminate,
    ))
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: profile-response-extractor <normalized-profile-series.json> <new-profile-audit-input.json> <threshold-m3-m3>",
        ).into());
    }
    let threshold: f64 = args[3].parse().map_err(|error| {
        io::Error::new(io::ErrorKind::InvalidInput, format!("invalid threshold: {error}"))
    })?;
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let (audit_input, detected, not_detected, indeterminate) =
        evaluate(&input, input_sha256.clone(), threshold)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&audit_input)?;
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
        "profile response extraction complete: threshold={threshold:.4} m3/m3; detected={detected}; not_detected={not_detected}; indeterminate={indeterminate}; source_sha256={input_sha256}"
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("profile response extraction failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::{TimeZone, Utc};

    fn stamp(minutes: i64) -> String {
        let origin = Utc.with_ymd_and_hms(2026, 4, 1, 0, 0, 0).single().unwrap();
        (origin + Duration::minutes(minutes)).to_rfc3339()
    }

    fn series(
        sensor_id: &str,
        depth_cm: f64,
        onset_minute: Option<i64>,
        response_delta: f64,
    ) -> SensorSeries {
        let observations = [-10, 0, 10, 20, 30].into_iter().map(|minute| {
            let theta = if Some(minute) == onset_minute {
                0.20 + response_delta
            } else if onset_minute.is_some_and(|onset| minute > onset) {
                0.20 + response_delta
            } else {
                0.20
            };
            Observation {
                timestamp_utc: stamp(minute),
                volumetric_water_content_m3_m3: Some(theta),
                status: SampleStatus::Observed,
            }
        }).collect();
        SensorSeries {
            sensor_id: sensor_id.to_owned(),
            depth_cm,
            condition: SensorCondition::Usable,
            observations,
        }
    }

    fn fixture() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "synthetic-study".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE1".to_owned(),
            event_id: "EVENT1".to_owned(),
            event_start_utc: stamp(0),
            observation_window_end_utc: stamp(30),
            time_resolution_seconds: 600,
            source_evidence_ref: "synthetic fixture".to_owned(),
            sensors: vec![
                series("S10", 10.0, Some(20), 0.012),
                series("S30", 30.0, Some(0), 0.025),
                series("S60", 60.0, Some(30), 0.040),
            ],
        }
    }

    #[test]
    fn emits_profile_audit_input_with_interval_censored_crossings() {
        let input = fixture();
        let (output, detected, not_detected, indeterminate) =
            evaluate(&input, "a".repeat(64), 0.01).unwrap();
        assert_eq!((detected, not_detected, indeterminate), (3, 0, 0));
        assert_eq!(output.schema_version, "0.3.0");
        assert_eq!(output.sensors[0].response_onset_utc.as_deref(), Some(stamp(20).as_str()));
        assert_eq!(
            output.sensors[0].response_onset_lower_bound_utc.as_deref(),
            Some(stamp(10).as_str())
        );
        assert_eq!(output.sensors[1].response_onset_utc.as_deref(), Some(stamp(0).as_str()));
    }

    #[test]
    fn missing_pre_event_baseline_is_indeterminate() {
        let mut input = fixture();
        let sample = &mut input.sensors[0].observations[0];
        sample.status = SampleStatus::Missing;
        sample.volumetric_water_content_m3_m3 = None;
        let (output, detected, not_detected, indeterminate) =
            evaluate(&input, "b".repeat(64), 0.01).unwrap();
        assert_eq!((detected, not_detected, indeterminate), (2, 0, 1));
        assert_eq!(output.sensors[0].response_status, "indeterminate");
    }

    #[test]
    fn interpolated_crossing_does_not_claim_an_observed_onset() {
        let mut input = fixture();
        let sample = &mut input.sensors[0].observations[3];
        sample.status = SampleStatus::LinearInterpolatedGapLt6h;
        let (output, _, _, indeterminate) = evaluate(&input, "c".repeat(64), 0.01).unwrap();
        assert_eq!(indeterminate, 1);
        assert_eq!(output.sensors[0].response_status, "indeterminate");
    }

    #[test]
    fn saturated_frozen_or_faulted_sensors_are_indeterminate() {
        for condition in [
            SensorCondition::Saturated,
            SensorCondition::Frozen,
            SensorCondition::SensorFault,
            SensorCondition::Unknown,
        ] {
            let mut input = fixture();
            input.sensors[0].condition = condition;
            let (output, _, _, indeterminate) =
                evaluate(&input, "d".repeat(64), 0.01).unwrap();
            assert_eq!(indeterminate, 1);
            assert_eq!(output.sensors[0].response_status, "indeterminate");
        }
    }

    #[test]
    fn higher_response_threshold_can_turn_weak_response_into_nonresponse() {
        let input = fixture();
        let (low, _, _, _) = evaluate(&input, "e".repeat(64), 0.01).unwrap();
        let (high, detected, not_detected, _) = evaluate(&input, "e".repeat(64), 0.03).unwrap();
        assert_eq!(low.sensors[0].response_status, "detected");
        assert_eq!((detected, not_detected), (1, 2));
        assert_eq!(high.sensors[0].response_status, "not_detected");
        assert_eq!(high.sensors[1].response_status, "not_detected");
        assert_eq!(high.sensors[2].response_status, "detected");
    }

    #[test]
    fn rejects_sensor_series_with_a_missing_grid_bin() {
        let mut input = fixture();
        input.sensors[0].observations[2].timestamp_utc = stamp(15);
        assert!(validate(&input).unwrap_err().contains("shared profile grid"));
    }

    #[test]
    fn rejects_non_utc_timestamps() {
        let mut input = fixture();
        input.sensors[0].observations[0].timestamp_utc =
            "2026-03-31T19:50:00-04:00".to_owned();
        assert!(validate(&input).unwrap_err().contains("zero UTC offset"));
    }
}

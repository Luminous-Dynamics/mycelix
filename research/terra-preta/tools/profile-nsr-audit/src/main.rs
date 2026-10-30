use chrono::{DateTime, FixedOffset};
#[cfg(test)]
use chrono::{Duration, TimeZone, Utc};
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

const AUDIT_VERSION: &str = "0.1.0";
const MIN_RESOLUTION_SECONDS: u32 = 300;
const MAX_RESOLUTION_SECONDS: u32 = 1800;

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SensorStatus {
    Responded,
    NoResponse,
    Indeterminate,
}

impl SensorStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::Responded => "responded",
            Self::NoResponse => "no_response",
            Self::Indeterminate => "indeterminate",
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorObservation {
    sensor_id: String,
    depth_cm: f64,
    status: SensorStatus,
    response_onset_utc: Option<String>,
    #[serde(default)]
    note: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct EventInput {
    schema_version: String,
    event_id: String,
    study_id: String,
    site_id: String,
    profile_id: String,
    precipitation_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    response_threshold_m3_m3: f64,
    response_onset_method: String,
    sensors: Vec<SensorObservation>,
    source_sha256: String,
    source_evidence_ref: String,
}

#[derive(Clone, Debug)]
struct ParsedSensor<'a> {
    source: &'a SensorObservation,
    response_onset: Option<DateTime<FixedOffset>>,
}

#[derive(Serialize)]
struct PairComparison {
    shallower_sensor_id: String,
    shallower_depth_cm: f64,
    shallower_response_onset_utc: String,
    deeper_sensor_id: String,
    deeper_depth_cm: f64,
    deeper_response_onset_utc: String,
    ordering: &'static str,
    response_time_difference_seconds: i64,
}

#[derive(Serialize)]
struct EventReport {
    audit_version: &'static str,
    event_id: String,
    input_sha256: String,
    source_sha256: String,
    study_id: String,
    site_id: String,
    profile_id: String,
    precipitation_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    response_threshold_m3_m3: f64,
    response_onset_method: String,
    installed_sensor_count: usize,
    responding_sensor_count: usize,
    valid_no_response_sensor_count: usize,
    indeterminate_sensor_count: usize,
    distinct_responding_depth_count: usize,
    compared_depth_pair_count: usize,
    nonsequential_pair_count: usize,
    sequential_pair_count: usize,
    simultaneous_pair_count: usize,
    profile_coverage_status: &'static str,
    response_signature: &'static str,
    interpretation_boundary: &'static str,
    pair_comparisons: Vec<PairComparison>,
    warnings: Vec<String>,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn parse_timestamp(
    value: &str,
    label: &str,
    resolution_seconds: u32,
) -> Result<DateTime<FixedOffset>, String> {
    let timestamp = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if timestamp.offset().local_minus_utc() != 0 {
        return Err(format!("{label} must use a zero UTC offset"));
    }
    let step = i64::from(resolution_seconds);
    if timestamp.timestamp().rem_euclid(step) != 0 || timestamp.timestamp_subsec_nanos() != 0 {
        return Err(format!(
            "{label} is not aligned to the declared {}-second UTC cadence",
            resolution_seconds
        ));
    }
    Ok(timestamp)
}

fn validate_input(input: &EventInput) -> Result<Vec<ParsedSensor<'_>>, String> {
    if input.schema_version != "0.1.0" {
        return Err(format!(
            "unsupported schema_version {:?}",
            input.schema_version
        ));
    }
    for (value, label) in [
        (&input.event_id, "event_id"),
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.profile_id, "profile_id"),
        (&input.source_sha256, "source_sha256"),
        (&input.source_evidence_ref, "source_evidence_ref"),
    ] {
        nonblank(value, label)?;
    }
    if input.source_sha256.len() != 64
        || !input.source_sha256.bytes().all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("source_sha256 must contain exactly 64 hexadecimal characters".to_owned());
    }
    if !(MIN_RESOLUTION_SECONDS..=MAX_RESOLUTION_SECONDS)
        .contains(&input.time_resolution_seconds)
    {
        return Err(format!(
            "time_resolution_seconds must be between {} and {} (5–30 minutes)",
            MIN_RESOLUTION_SECONDS, MAX_RESOLUTION_SECONDS
        ));
    }
    if input.sensors.is_empty() {
        return Err("at least one sensor record is required".to_owned());
    }
    if !input.response_threshold_m3_m3.is_finite()
        || input.response_threshold_m3_m3 <= 0.0
        || input.response_threshold_m3_m3 > 1.0
    {
        return Err("response_threshold_m3_m3 must be finite and in (0, 1]".to_owned());
    }
    nonblank(&input.response_onset_method, "response_onset_method")?;

    let event_start = parse_timestamp(
        &input.precipitation_start_utc,
        "precipitation_start_utc",
        input.time_resolution_seconds,
    )?;
    let observation_end = parse_timestamp(
        &input.observation_window_end_utc,
        "observation_window_end_utc",
        input.time_resolution_seconds,
    )?;
    if event_start >= observation_end {
        return Err("observation_window_end_utc must be after precipitation_start_utc".to_owned());
    }

    let mut seen_ids = HashSet::new();
    let mut parsed = Vec::with_capacity(input.sensors.len());

    for (index, sensor) in input.sensors.iter().enumerate() {
        nonblank(&sensor.sensor_id, &format!("sensors[{index}].sensor_id"))?;
        if !seen_ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        if !sensor.depth_cm.is_finite() || sensor.depth_cm <= 0.0 {
            return Err(format!(
                "sensors[{index}].depth_cm must be finite and greater than zero"
            ));
        }
        if let Some(note) = &sensor.note {
            nonblank(note, &format!("sensors[{index}].note"))?;
        }

        let response_onset = match (sensor.status, sensor.response_onset_utc.as_deref()) {
            (SensorStatus::Responded, Some(value)) => {
                let onset = parse_timestamp(
                    value,
                    &format!("sensors[{index}].response_onset_utc"),
                    input.time_resolution_seconds,
                )?;
                if onset < event_start || onset > observation_end {
                    return Err(format!(
                        "sensor {:?} response onset lies outside the declared event observation window",
                        sensor.sensor_id
                    ));
                }
                Some(onset)
            }
            (SensorStatus::Responded, None) => {
                return Err(format!(
                    "responded sensor {:?} requires response_onset_utc",
                    sensor.sensor_id
                ));
            }
            (SensorStatus::NoResponse | SensorStatus::Indeterminate, None) => None,
            (SensorStatus::NoResponse | SensorStatus::Indeterminate, Some(_)) => {
                return Err(format!(
                    "sensor {:?} must not declare a response onset when status is {}",
                    sensor.sensor_id,
                    sensor.status.as_str()
                ));
            }
        };

        parsed.push(ParsedSensor {
            source: sensor,
            response_onset,
        });
    }

    Ok(parsed)
}

fn evaluate(input: &EventInput, input_sha256: String) -> Result<EventReport, String> {
    let sensors = validate_input(input)?;
    let responding: Vec<_> = sensors
        .iter()
        .filter(|sensor| sensor.response_onset.is_some())
        .collect();
    let no_response_count = sensors
        .iter()
        .filter(|sensor| matches!(sensor.source.status, SensorStatus::NoResponse))
        .count();
    let indeterminate_count = sensors
        .iter()
        .filter(|sensor| matches!(sensor.source.status, SensorStatus::Indeterminate))
        .count();

    let mut distinct_depths: Vec<f64> = responding
        .iter()
        .map(|sensor| sensor.source.depth_cm)
        .collect();
    distinct_depths.sort_by(f64::total_cmp);
    distinct_depths.dedup_by(|left, right| left.total_cmp(right).is_eq());

    let mut comparisons = Vec::new();
    let mut nonsequential_count = 0usize;
    let mut sequential_count = 0usize;
    let mut simultaneous_count = 0usize;

    for left_index in 0..responding.len() {
        for right_index in (left_index + 1)..responding.len() {
            let left = responding[left_index];
            let right = responding[right_index];
            if left.source.depth_cm == right.source.depth_cm {
                continue;
            }

            let (shallower, deeper) = if left.source.depth_cm < right.source.depth_cm {
                (left, right)
            } else {
                (right, left)
            };
            let shallow_onset = shallower.response_onset.as_ref().expect("responded sensor has onset");
            let deep_onset = deeper.response_onset.as_ref().expect("responded sensor has onset");
            let difference = deep_onset.timestamp() - shallow_onset.timestamp();

            let ordering = if difference < 0 {
                nonsequential_count += 1;
                "nonsequential_deeper_first"
            } else if difference > 0 {
                sequential_count += 1;
                "sequential_shallower_first"
            } else {
                simultaneous_count += 1;
                "simultaneous_at_recorded_resolution"
            };

            comparisons.push(PairComparison {
                shallower_sensor_id: shallower.source.sensor_id.clone(),
                shallower_depth_cm: shallower.source.depth_cm,
                shallower_response_onset_utc: shallow_onset.to_rfc3339(),
                deeper_sensor_id: deeper.source.sensor_id.clone(),
                deeper_depth_cm: deeper.source.depth_cm,
                deeper_response_onset_utc: deep_onset.to_rfc3339(),
                ordering,
                response_time_difference_seconds: difference,
            });
        }
    }

    let signature = if responding.len() < 2 {
        "not_applicable_insufficient_responders"
    } else if comparisons.is_empty() {
        "indeterminate_no_distinct_depth_pair"
    } else if nonsequential_count > 0 {
        "nonsequential_response_signature"
    } else if indeterminate_count > 0 {
        "indeterminate_incomplete_profile_without_observed_nsr"
    } else if simultaneous_count > 0 {
        "ambiguous_simultaneous_response"
    } else {
        "sequential_response_signature"
    };

    let profile_coverage_status = if indeterminate_count > 0 {
        "incomplete"
    } else {
        "complete_for_declared_sensor_roster"
    };

    let mut warnings = vec![
        "NSR is a soil-moisture response-order signature, not proof of a unique pore pathway or measured preferential water flux.".to_owned(),
        "A sequential response does not rule out preferential flow; finite sensor spacing and sampling cadence can hide bypass.".to_owned(),
        "A nonsequential signature may include lateral flow or profile heterogeneity; orientation and spatial geometry are not inferred from depth ordering alone.".to_owned(),
        "Valid no-response sensors are distinct from indeterminate sensors. Missing/invalid observations reduce declared profile coverage.".to_owned(),
        "The result is limited to the supplied sensor roster; point sensors may miss narrow or laterally displaced preferential pathways.".to_owned(),
    ];
    if profile_coverage_status == "incomplete" && nonsequential_count > 0 {
        warnings.push(
            "An observed nonsequential pair exists, but the declared profile is incomplete; report it as a partial-profile signature, not full-profile coverage.".to_owned(),
        );
    }

    Ok(EventReport {
        audit_version: AUDIT_VERSION,
        event_id: input.event_id.clone(),
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        profile_id: input.profile_id.clone(),
        precipitation_start_utc: input.precipitation_start_utc.clone(),
        observation_window_end_utc: input.observation_window_end_utc.clone(),
        time_resolution_seconds: input.time_resolution_seconds,
        response_threshold_m3_m3: input.response_threshold_m3_m3,
        response_onset_method: input.response_onset_method.clone(),
        installed_sensor_count: sensors.len(),
        responding_sensor_count: responding.len(),
        valid_no_response_sensor_count: no_response_count,
        indeterminate_sensor_count: indeterminate_count,
        distinct_responding_depth_count: distinct_depths.len(),
        compared_depth_pair_count: comparisons.len(),
        nonsequential_pair_count: nonsequential_count,
        sequential_pair_count: sequential_count,
        simultaneous_pair_count: simultaneous_count,
        profile_coverage_status,
        response_signature: signature,
        interpretation_boundary: "soil-moisture NSR signature only; not tracer-confirmed pathway, flux partition, vertical/lateral flow attribution or treatment effect",
        pair_comparisons: comparisons,
        warnings,
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "usage: {} <profile-event.json> <new-report.json>",
                arguments
                    .first()
                    .map(String::as_str)
                    .unwrap_or("terra-preta-profile-nsr-audit")
            ),
        )
        .into());
    }

    let input_bytes = fs::read(&arguments[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&input_bytes));
    let input: EventInput = serde_json::from_slice(&input_bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&report)?;

    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&arguments[2])
        .map_err(|error| {
            io::Error::new(
                error.kind(),
                format!(
                    "refusing overwrite or cannot create report {}: {error}",
                    arguments[2]
                ),
            )
        })?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;

    println!(
        "NSR audit complete: event={} signature={} responders={} NSR_pairs={} coverage={} input_sha256={}",
        report.event_id,
        report.response_signature,
        report.responding_sensor_count,
        report.nonsequential_pair_count,
        report.profile_coverage_status,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("profile NSR audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(minutes: i64) -> String {
        let origin = Utc.with_ymd_and_hms(2026, 1, 1, 0, 0, 0).single().unwrap();
        (origin + Duration::minutes(minutes)).to_rfc3339()
    }

    fn sensor(id: &str, depth: f64, status: SensorStatus, response: Option<i64>) -> SensorObservation {
        SensorObservation {
            sensor_id: id.to_owned(),
            depth_cm: depth,
            status,
            response_onset_utc: response.map(stamp),
            note: None,
        }
    }

    fn fixture() -> EventInput {
        EventInput {
            schema_version: "0.1.0".to_owned(),
            event_id: "E-001".to_owned(),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            profile_id: "P01".to_owned(),
            precipitation_start_utc: stamp(0),
            observation_window_end_utc: stamp(180),
            time_resolution_seconds: 600,
            response_threshold_m3_m3: 0.01,
            response_onset_method: "first_10min_bin_delta_theta_ge_threshold".to_owned(),
            sensors: vec![
                sensor("S10", 10.0, SensorStatus::Responded, Some(30)),
                sensor("S30", 30.0, SensorStatus::Responded, Some(10)),
                sensor("S50", 50.0, SensorStatus::Responded, Some(20)),
            ],
            source_sha256: "a".repeat(64),
            source_evidence_ref: "synthetic fixture only".to_owned(),
        }
    }

    #[test]
    fn detects_deeper_sensor_responding_before_shallower_sensor() {
        let input = fixture();
        let report = evaluate(&input, "b".repeat(64)).unwrap();
        assert_eq!(report.response_signature, "nonsequential_response_signature");
        assert_eq!(report.nonsequential_pair_count, 2);
        assert_eq!(report.sequential_pair_count, 1);
        assert_eq!(report.distinct_responding_depth_count, 3);
    }

    #[test]
    fn recognizes_fully_sequential_depth_order() {
        let mut input = fixture();
        input.sensors[0].response_onset_utc = Some(stamp(10));
        input.sensors[1].response_onset_utc = Some(stamp(20));
        input.sensors[2].response_onset_utc = Some(stamp(30));
        let report = evaluate(&input, "c".repeat(64)).unwrap();
        assert_eq!(report.response_signature, "sequential_response_signature");
        assert_eq!(report.nonsequential_pair_count, 0);
        assert_eq!(report.sequential_pair_count, 3);
    }

    #[test]
    fn treats_equal_onsets_as_temporally_ambiguous_not_nsr() {
        let mut input = fixture();
        input.sensors[0].response_onset_utc = Some(stamp(10));
        input.sensors[1].response_onset_utc = Some(stamp(10));
        input.sensors[2].response_onset_utc = Some(stamp(10));
        let report = evaluate(&input, "d".repeat(64)).unwrap();
        assert_eq!(report.response_signature, "ambiguous_simultaneous_response");
        assert_eq!(report.simultaneous_pair_count, 3);
        assert_eq!(report.nonsequential_pair_count, 0);
    }

    #[test]
    fn preserves_nsr_signature_but_marks_incomplete_roster() {
        let mut input = fixture();
        input.sensors.push(sensor("S80", 80.0, SensorStatus::Indeterminate, None));
        let report = evaluate(&input, "e".repeat(64)).unwrap();
        assert_eq!(report.response_signature, "nonsequential_response_signature");
        assert_eq!(report.profile_coverage_status, "incomplete");
        assert_eq!(report.indeterminate_sensor_count, 1);
    }

    #[test]
    fn a_single_sensor_profile_is_not_applicable_not_invalid() {
        let mut input = fixture();
        input.sensors.truncate(1);
        let report = evaluate(&input, "f".repeat(64)).unwrap();
        assert_eq!(report.response_signature, "not_applicable_insufficient_responders");
        assert_eq!(report.installed_sensor_count, 1);
    }

    #[test]
    fn fewer_than_two_responders_is_not_applicable() {
        let mut input = fixture();
        input.sensors[1].status = SensorStatus::NoResponse;
        input.sensors[1].response_onset_utc = None;
        input.sensors[2].status = SensorStatus::Indeterminate;
        input.sensors[2].response_onset_utc = None;
        let report = evaluate(&input, "f".repeat(64)).unwrap();
        assert_eq!(
            report.response_signature,
            "not_applicable_insufficient_responders"
        );
    }

    #[test]
    fn sensors_at_the_same_depth_do_not_create_a_vertical_pair() {
        let mut input = fixture();
        input.sensors[1].depth_cm = 10.0;
        input.sensors[2].depth_cm = 10.0;
        let report = evaluate(&input, "1".repeat(64)).unwrap();
        assert_eq!(
            report.response_signature,
            "indeterminate_no_distinct_depth_pair"
        );
        assert_eq!(report.compared_depth_pair_count, 0);
    }

    #[test]
    fn rejects_duplicate_sensor_identifiers() {
        let mut input = fixture();
        input.sensors[1].sensor_id = input.sensors[0].sensor_id.clone();
        assert!(validate_input(&input).unwrap_err().contains("duplicate sensor_id"));
    }

    #[test]
    fn rejects_response_outside_event_observation_window() {
        let mut input = fixture();
        input.sensors[0].response_onset_utc = Some(stamp(190));
        assert!(validate_input(&input)
            .unwrap_err()
            .contains("outside the declared event observation window"));
    }

    #[test]
    fn rejects_non_utc_timestamp() {
        let mut input = fixture();
        input.precipitation_start_utc = "2026-01-01T01:00:00+01:00".to_owned();
        assert!(validate_input(&input).unwrap_err().contains("zero UTC offset"));
    }

    #[test]
    fn rejects_timestamps_not_aligned_to_declared_resolution() {
        let mut input = fixture();
        input.sensors[0].response_onset_utc = Some("2026-01-01T00:15:00Z".to_owned());
        assert!(validate_input(&input)
            .unwrap_err()
            .contains("not aligned to the declared"));
    }

    #[test]
    fn rejects_resolution_outside_reviewed_high_frequency_range() {
        let mut input = fixture();
        input.time_resolution_seconds = 3600;
        assert!(validate_input(&input)
            .unwrap_err()
            .contains("between 300 and 1800"));
    }

    #[test]
    fn rejects_response_timestamp_on_nonresponding_sensor() {
        let mut input = fixture();
        input.sensors[0].status = SensorStatus::NoResponse;
        assert!(validate_input(&input)
            .unwrap_err()
            .contains("must not declare a response onset"));
    }
}

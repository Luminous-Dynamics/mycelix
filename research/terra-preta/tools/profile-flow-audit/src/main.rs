use chrono::{DateTime, Duration, FixedOffset};
#[cfg(test)]
use chrono::{TimeZone, Utc};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, error::Error, fs, fs::OpenOptions, io::{self, Write}};

const STEP_SECONDS: i64 = 600;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ResponseStatus {
    Detected,
    NotDetected,
    Indeterminate,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    site_id: String,
    plot_id: String,
    event_id: String,
    event_start_utc: String,
    sensors: Vec<SensorResponse>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorResponse {
    sensor_id: String,
    depth_cm: f64,
    response_status: ResponseStatus,
    response_onset_utc: Option<String>,
}

#[derive(Clone, Debug)]
struct ParsedSensor {
    sensor_id: String,
    depth_cm: f64,
    onset: Option<DateTime<FixedOffset>>,
    response_status: ResponseStatus,
}

#[derive(Serialize)]
struct PairResult {
    shallow_sensor_id: String,
    shallow_depth_cm: f64,
    shallow_response_onset_utc: String,
    deep_sensor_id: String,
    deep_depth_cm: f64,
    deep_response_onset_utc: String,
    deep_advance_minutes: i64,
    ordering: &'static str,
}

#[derive(Serialize)]
struct Report {
    tool_version: &'static str,
    input_sha256: String,
    site_id: String,
    plot_id: String,
    event_id: String,
    event_start_utc: String,
    sensor_count: usize,
    detected_sensor_count: usize,
    comparable_pair_count: usize,
    nsr_pair_count: usize,
    sequential_pair_count: usize,
    tied_pair_count: usize,
    not_detected_sensor_count: usize,
    indeterminate_sensor_count: usize,
    profile_classification: &'static str,
    pair_results: Vec<PairResult>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn parse_utc(value: &str, label: &str) -> Result<DateTime<FixedOffset>, String> {
    let parsed = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if parsed.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamps must use a zero UTC offset"));
    }
    if parsed.timestamp() % STEP_SECONDS != 0 || parsed.timestamp_subsec_nanos() != 0 {
        return Err(format!("{label}: timestamp must align to the 10-minute sampling grid"));
    }
    Ok(parsed)
}

fn validate(input: &Input) -> Result<(DateTime<FixedOffset>, Vec<ParsedSensor>), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.site_id, "site_id"),
        (&input.plot_id, "plot_id"),
        (&input.event_id, "event_id"),
    ] {
        if value.trim().is_empty() {
            return Err(format!("{label} must not be blank"));
        }
    }
    if input.sensors.is_empty() {
        return Err("sensors must not be empty".to_owned());
    }

    let event_start = parse_utc(&input.event_start_utc, "event_start_utc")?;
    let mut ids = std::collections::HashSet::new();
    let mut sensors = Vec::with_capacity(input.sensors.len());

    for (index, sensor) in input.sensors.iter().enumerate() {
        if sensor.sensor_id.trim().is_empty() {
            return Err(format!("sensors[{index}].sensor_id must not be blank"));
        }
        if !ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        if !sensor.depth_cm.is_finite() || sensor.depth_cm <= 0.0 {
            return Err(format!("sensors[{index}].depth_cm must be finite and positive"));
        }

        let onset = match (sensor.response_status, sensor.response_onset_utc.as_deref()) {
            (ResponseStatus::Detected, Some(value)) => {
                let parsed = parse_utc(value, &format!("sensors[{index}].response_onset_utc"))?;
                if parsed < event_start {
                    return Err(format!("sensors[{index}] response onset precedes rainfall event start"));
                }
                Some(parsed)
            }
            (ResponseStatus::Detected, None) => {
                return Err(format!("sensors[{index}] detected response requires response_onset_utc"));
            }
            (ResponseStatus::NotDetected | ResponseStatus::Indeterminate, None) => None,
            (ResponseStatus::NotDetected | ResponseStatus::Indeterminate, Some(_)) => {
                return Err(format!("sensors[{index}] non-detected/indeterminate response must not have an onset time"));
            }
        };

        sensors.push(ParsedSensor {
            sensor_id: sensor.sensor_id.clone(),
            depth_cm: sensor.depth_cm,
            onset,
            response_status: sensor.response_status,
        });
    }

    sensors.sort_by(|left, right| left.depth_cm.total_cmp(&right.depth_cm));
    Ok((event_start, sensors))
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (event_start, sensors) = validate(input)?;
    let detected_sensor_count = sensors
        .iter()
        .filter(|sensor| sensor.response_status == ResponseStatus::Detected)
        .count();
    let not_detected_sensor_count = sensors
        .iter()
        .filter(|sensor| sensor.response_status == ResponseStatus::NotDetected)
        .count();
    let indeterminate_sensor_count = sensors
        .iter()
        .filter(|sensor| sensor.response_status == ResponseStatus::Indeterminate)
        .count();

    let mut pair_results = Vec::new();
    let mut nsr_pair_count = 0usize;
    let mut sequential_pair_count = 0usize;
    let mut tied_pair_count = 0usize;

    for (shallow_index, shallow) in sensors.iter().enumerate() {
        for deep in sensors.iter().skip(shallow_index + 1) {
            if shallow.depth_cm == deep.depth_cm {
                continue;
            }
            let (Some(shallow_onset), Some(deep_onset)) = (&shallow.onset, &deep.onset) else {
                continue;
            };
            let deep_advance_minutes = (shallow_onset.timestamp() - deep_onset.timestamp()) / 60;
            let ordering = if deep_advance_minutes > 0 {
                nsr_pair_count += 1;
                "nonsequential_response"
            } else if deep_advance_minutes < 0 {
                sequential_pair_count += 1;
                "sequential_response"
            } else {
                tied_pair_count += 1;
                "simultaneous_at_sampling_resolution"
            };
            pair_results.push(PairResult {
                shallow_sensor_id: shallow.sensor_id.clone(),
                shallow_depth_cm: shallow.depth_cm,
                shallow_response_onset_utc: shallow_onset.to_rfc3339(),
                deep_sensor_id: deep.sensor_id.clone(),
                deep_depth_cm: deep.depth_cm,
                deep_response_onset_utc: deep_onset.to_rfc3339(),
                deep_advance_minutes,
                ordering,
            });
        }
    }

    let comparable_pair_count = pair_results.len();
    let profile_classification = match (nsr_pair_count > 0, sequential_pair_count > 0, tied_pair_count > 0) {
        (true, true, _) => "mixed_response_ordering",
        (true, false, _) => "nonsequential_response_detected",
        (false, true, true) => "sequential_and_simultaneous_response",
        (false, true, false) => "sequential_response_only",
        (false, false, true) => "simultaneous_resolution_limited",
        (false, false, false) => "insufficient_comparable_sensor_responses",
    };

    Ok(Report {
        tool_version: "0.1.0",
        input_sha256,
        site_id: input.site_id.clone(),
        plot_id: input.plot_id.clone(),
        event_id: input.event_id.clone(),
        event_start_utc: event_start.to_rfc3339(),
        sensor_count: sensors.len(),
        detected_sensor_count,
        comparable_pair_count,
        nsr_pair_count,
        sequential_pair_count,
        tied_pair_count,
        not_detected_sensor_count,
        indeterminate_sensor_count,
        profile_classification,
        pair_results,
        interpretation_boundary: "NSR is a profile-level soil-moisture response-order diagnostic; it is not tracer confirmation, a unique macropore proof, or a direct flux measurement",
        warnings: vec![
            "The result depends on sensor depths, spacing, sensing volume, sampling cadence and onset detector; sparse profiles can miss preferential pathways.".to_owned(),
            "A sequential response does not prove matrix-only flow; a preferential pathway may bypass uninstrumented layers or sensors.".to_owned(),
            "A nonsequential response is a sensor-derived diagnostic and requires independent tracer/geophysical/boundary evidence for physical pathway claims.".to_owned(),
            "Equal response times are unresolved at the 10-minute sampling cadence; do not break ties arbitrarily.".to_owned(),
            "Sensors at identical nominal depths are not ordered and are excluded from depth-pair comparisons.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: profile-flow-audit <profile-event-input.json> <new-report.json>",
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
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report: {error}")))?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "profile event audited: {} sensors; {} pairs; {} NSR; {} sequential; class={}",
        report.sensor_count,
        report.comparable_pair_count,
        report.nsr_pair_count,
        report.sequential_pair_count,
        report.profile_classification
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("profile-flow audit failed closed: {error}");
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

    fn input(responses: &[(f64, ResponseStatus, Option<i64>)]) -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            event_id: "E001".to_owned(),
            event_start_utc: stamp(0),
            sensors: responses.iter().enumerate().map(|(index, (depth, status, onset))| SensorResponse {
                sensor_id: format!("D{}", index + 1),
                depth_cm: *depth,
                response_status: *status,
                response_onset_utc: (*onset).map(stamp),
            }).collect(),
        }
    }

    #[test]
    fn deeper_sensor_responding_first_is_nsr() {
        let data = input(&[
            (10.0, ResponseStatus::Detected, Some(30)),
            (30.0, ResponseStatus::Detected, Some(10)),
        ]);
        let report = evaluate(&data, "a".repeat(64)).unwrap();
        assert_eq!(report.profile_classification, "nonsequential_response_detected");
        assert_eq!(report.nsr_pair_count, 1);
        assert_eq!(report.pair_results[0].deep_advance_minutes, 20);
    }

    #[test]
    fn shallow_to_deep_order_is_sequential_not_nsr() {
        let data = input(&[
            (10.0, ResponseStatus::Detected, Some(10)),
            (30.0, ResponseStatus::Detected, Some(30)),
        ]);
        let report = evaluate(&data, "b".repeat(64)).unwrap();
        assert_eq!(report.profile_classification, "sequential_response_only");
        assert_eq!(report.nsr_pair_count, 0);
        assert_eq!(report.sequential_pair_count, 1);
    }

    #[test]
    fn mixed_pair_orderings_are_preserved() {
        let data = input(&[
            (10.0, ResponseStatus::Detected, Some(30)),
            (30.0, ResponseStatus::Detected, Some(10)),
            (60.0, ResponseStatus::Detected, Some(20)),
        ]);
        let report = evaluate(&data, "c".repeat(64)).unwrap();
        assert_eq!(report.profile_classification, "mixed_response_ordering");
        assert_eq!(report.nsr_pair_count, 2);
        assert_eq!(report.sequential_pair_count, 1);
    }

    #[test]
    fn tied_responses_are_not_forced_into_a_direction() {
        let data = input(&[
            (10.0, ResponseStatus::Detected, Some(10)),
            (30.0, ResponseStatus::Detected, Some(10)),
        ]);
        let report = evaluate(&data, "d".repeat(64)).unwrap();
        assert_eq!(report.profile_classification, "simultaneous_resolution_limited");
        assert_eq!(report.tied_pair_count, 1);
    }

    #[test]
    fn missing_responses_are_not_counted_as_negative_flow_evidence() {
        let data = input(&[
            (10.0, ResponseStatus::Detected, Some(10)),
            (30.0, ResponseStatus::Indeterminate, None),
            (60.0, ResponseStatus::NotDetected, None),
        ]);
        let report = evaluate(&data, "e".repeat(64)).unwrap();
        assert_eq!(report.profile_classification, "insufficient_comparable_sensor_responses");
        assert_eq!(report.detected_sensor_count, 1);
        assert_eq!(report.not_detected_sensor_count, 1);
        assert_eq!(report.indeterminate_sensor_count, 1);
        assert_eq!(report.comparable_pair_count, 0);
    }

    #[test]
    fn duplicate_sensor_ids_are_rejected() {
        let mut data = input(&[
            (10.0, ResponseStatus::Detected, Some(10)),
            (30.0, ResponseStatus::Detected, Some(20)),
        ]);
        data.sensors[1].sensor_id = data.sensors[0].sensor_id.clone();
        assert!(validate(&data).unwrap_err().contains("duplicate sensor_id"));
    }

    #[test]
    fn response_before_event_start_is_rejected() {
        let data = input(&[
            (10.0, ResponseStatus::Detected, Some(-10)),
            (30.0, ResponseStatus::Detected, Some(20)),
        ]);
        assert!(validate(&data).unwrap_err().contains("precedes rainfall event start"));
    }

    #[test]
    fn sensors_at_the_same_depth_are_not_ordered() {
        let data = input(&[
            (10.0, ResponseStatus::Detected, Some(10)),
            (10.0, ResponseStatus::Detected, Some(20)),
        ]);
        let report = evaluate(&data, "f".repeat(64)).unwrap();
        assert_eq!(report.comparable_pair_count, 0);
        assert_eq!(report.profile_classification, "insufficient_comparable_sensor_responses");
    }
}

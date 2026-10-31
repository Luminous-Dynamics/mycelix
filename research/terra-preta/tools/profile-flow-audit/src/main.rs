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

const AUDIT_VERSION: &str = "0.3.0";
const MIN_RESOLUTION_SECONDS: u32 = 300;
const MAX_RESOLUTION_SECONDS: u32 = 1800;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ResponseStatus {
    Detected,
    NotDetected,
    Indeterminate,
}

impl ResponseStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::Detected => "detected",
            Self::NotDetected => "not_detected",
            Self::Indeterminate => "indeterminate",
        }
    }
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
    response_onset_method: String,
    source_sha256: String,
    source_evidence_ref: String,
    sensors: Vec<SensorResponse>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorResponse {
    sensor_id: String,
    depth_cm: f64,
    response_status: ResponseStatus,
    /// First sample meeting the response threshold (inclusive upper bound).
    response_onset_utc: Option<String>,
    /// Last sample below threshold (exclusive lower bound); omitted inputs derive one sampling step.
    response_onset_lower_bound_utc: Option<String>,
}

#[derive(Clone, Debug)]
struct ParsedSensor<'a> {
    source: &'a SensorResponse,
    onset_lower: Option<DateTime<FixedOffset>>,
    onset_upper: Option<DateTime<FixedOffset>>,
}

#[derive(Serialize)]
struct PairResult {
    shallow_sensor_id: String,
    shallow_depth_cm: f64,
    shallow_onset_lower_exclusive_utc: String,
    shallow_onset_upper_inclusive_utc: String,
    deep_sensor_id: String,
    deep_depth_cm: f64,
    deep_onset_lower_exclusive_utc: String,
    deep_onset_upper_inclusive_utc: String,
    deep_advance_min_seconds: i64,
    deep_advance_max_seconds: i64,
    observed_detection_bin_advance_seconds: i64,
    ordering: &'static str,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_sha256: String,
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
    sensor_count: usize,
    detected_sensor_count: usize,
    not_detected_sensor_count: usize,
    indeterminate_sensor_count: usize,
    distinct_detecting_depth_count: usize,
    comparable_pair_count: usize,
    nsr_pair_count: usize,
    sequential_pair_count: usize,
    tied_pair_count: usize,
    ambiguous_pair_count: usize,
    profile_coverage_status: &'static str,
    profile_classification: &'static str,
    pair_results: Vec<PairResult>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
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
            "{label}: timestamp must align to the declared {}-second UTC sampling grid",
            resolution_seconds
        ));
    }
    Ok(parsed)
}

fn validate(input: &Input) -> Result<Vec<ParsedSensor<'_>>, String> {
    if input.schema_version != "0.3.0" {
        return Err(format!(
            "unsupported schema_version {:?}; expected 0.3.0",
            input.schema_version
        ));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.plot_id, "plot_id"),
        (&input.profile_id, "profile_id"),
        (&input.event_id, "event_id"),
        (&input.source_sha256, "source_sha256"),
        (&input.source_evidence_ref, "source_evidence_ref"),
        (&input.response_onset_method, "response_onset_method"),
    ] {
        nonblank(value, label)?;
    }
    if input.source_sha256.len() != 64
        || !input
            .source_sha256
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("source_sha256 must contain exactly 64 hexadecimal characters".to_owned());
    }
    if !(MIN_RESOLUTION_SECONDS..=MAX_RESOLUTION_SECONDS).contains(&input.time_resolution_seconds) {
        return Err(format!(
            "time_resolution_seconds must be between {} and {} (5–30 minutes)",
            MIN_RESOLUTION_SECONDS, MAX_RESOLUTION_SECONDS
        ));
    }
    if !input.response_threshold_m3_m3.is_finite()
        || input.response_threshold_m3_m3 <= 0.0
        || input.response_threshold_m3_m3 > 1.0
    {
        return Err("response_threshold_m3_m3 must be finite and in (0, 1]".to_owned());
    }
    if input.sensors.is_empty() {
        return Err("sensors must not be empty".to_owned());
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

    let mut ids = HashSet::new();
    let mut sensors = Vec::with_capacity(input.sensors.len());

    for (index, sensor) in input.sensors.iter().enumerate() {
        nonblank(&sensor.sensor_id, &format!("sensors[{index}].sensor_id"))?;
        if !ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        if !sensor.depth_cm.is_finite() || sensor.depth_cm <= 0.0 {
            return Err(format!(
                "sensors[{index}].depth_cm must be finite and greater than zero"
            ));
        }

        let (onset_lower, onset_upper) = match (
            sensor.response_status,
            sensor.response_onset_utc.as_deref(),
            sensor.response_onset_lower_bound_utc.as_deref(),
        ) {
            (ResponseStatus::Detected, Some(upper_value), lower_value) => {
                let upper = parse_utc(
                    upper_value,
                    &format!("sensors[{index}].response_onset_utc"),
                    input.time_resolution_seconds,
                )?;
                if upper < event_start || upper > observation_end {
                    return Err(format!(
                        "sensors[{index}] response onset lies outside the declared event observation window"
                    ));
                }
                let lower = match lower_value {
                    Some(value) => parse_utc(
                        value,
                        &format!("sensors[{index}].response_onset_lower_bound_utc"),
                        input.time_resolution_seconds,
                    )?,
                    None => upper - chrono::Duration::seconds(i64::from(input.time_resolution_seconds)),
                };
                let interval_seconds = (upper - lower).num_seconds();
                if interval_seconds < 0
                    || interval_seconds > i64::from(input.time_resolution_seconds)
                {
                    return Err(format!(
                        "sensors[{index}] response-onset interval must be ordered and no wider than one sampling interval"
                    ));
                }
                if lower < event_start - chrono::Duration::seconds(i64::from(input.time_resolution_seconds)) {
                    return Err(format!(
                        "sensors[{index}] response-onset lower bound is more than one sampling interval before event start"
                    ));
                }
                (Some(lower), Some(upper))
            }
            (ResponseStatus::Detected, None, _) => {
                return Err(format!(
                    "sensors[{index}] detected response requires response_onset_utc"
                ));
            }
            (ResponseStatus::NotDetected | ResponseStatus::Indeterminate, None, None) => (None, None),
            (ResponseStatus::NotDetected | ResponseStatus::Indeterminate, _, _) => {
                return Err(format!(
                    "sensors[{index}] status {} must not have response-onset bounds",
                    sensor.response_status.as_str()
                ));
            }
        };

        sensors.push(ParsedSensor {
            source: sensor,
            onset_lower,
            onset_upper,
        });
    }

    Ok(sensors)
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let sensors = validate(input)?;
    let detected: Vec<_> = sensors
        .iter()
        .filter(|sensor| sensor.onset_upper.is_some())
        .collect();
    let not_detected_count = sensors
        .iter()
        .filter(|sensor| sensor.source.response_status == ResponseStatus::NotDetected)
        .count();
    let indeterminate_count = sensors
        .iter()
        .filter(|sensor| sensor.source.response_status == ResponseStatus::Indeterminate)
        .count();

    let mut distinct_depths: Vec<f64> = detected
        .iter()
        .map(|sensor| sensor.source.depth_cm)
        .collect();
    distinct_depths.sort_by(f64::total_cmp);
    distinct_depths.dedup_by(|left, right| left.total_cmp(right).is_eq());

    let mut pair_results = Vec::new();
    let mut nsr_pair_count = 0usize;
    let mut sequential_pair_count = 0usize;
    let mut tied_pair_count = 0usize;
    let mut ambiguous_pair_count = 0usize;

    for left_index in 0..detected.len() {
        for right_index in (left_index + 1)..detected.len() {
            let left = detected[left_index];
            let right = detected[right_index];
            if left.source.depth_cm == right.source.depth_cm {
                continue;
            }

            let (shallow, deep) = if left.source.depth_cm < right.source.depth_cm {
                (left, right)
            } else {
                (right, left)
            };
            let shallow_lower = shallow.onset_lower.as_ref().expect("detected sensor has lower bound");
            let shallow_upper = shallow.onset_upper.as_ref().expect("detected sensor has upper bound");
            let deep_lower = deep.onset_lower.as_ref().expect("detected sensor has lower bound");
            let deep_upper = deep.onset_upper.as_ref().expect("detected sensor has upper bound");

            // Positive means the deeper sensor definitely responds first. Overlap is ambiguous.
            let deep_advance_min_seconds = shallow_lower.timestamp() - deep_upper.timestamp();
            let deep_advance_max_seconds = shallow_upper.timestamp() - deep_lower.timestamp();
            let observed_detection_bin_advance_seconds = shallow_upper.timestamp() - deep_upper.timestamp();
            let both_exact = shallow_lower == shallow_upper && deep_lower == deep_upper;

            let ordering = if deep_advance_min_seconds > 0 {
                nsr_pair_count += 1;
                "nonsequential_deeper_first_interval_separated"
            } else if deep_advance_max_seconds < 0 {
                sequential_pair_count += 1;
                "sequential_shallower_first_interval_separated"
            } else if both_exact && shallow_upper == deep_upper {
                tied_pair_count += 1;
                "simultaneous_exact_onset"
            } else {
                ambiguous_pair_count += 1;
                "ambiguous_overlapping_response_intervals"
            };

            pair_results.push(PairResult {
                shallow_sensor_id: shallow.source.sensor_id.clone(),
                shallow_depth_cm: shallow.source.depth_cm,
                shallow_onset_lower_exclusive_utc: shallow_lower.to_rfc3339(),
                shallow_onset_upper_inclusive_utc: shallow_upper.to_rfc3339(),
                deep_sensor_id: deep.source.sensor_id.clone(),
                deep_depth_cm: deep.source.depth_cm,
                deep_onset_lower_exclusive_utc: deep_lower.to_rfc3339(),
                deep_onset_upper_inclusive_utc: deep_upper.to_rfc3339(),
                deep_advance_min_seconds,
                deep_advance_max_seconds,
                observed_detection_bin_advance_seconds,
                ordering,
            });
        }
    }

    let profile_classification = if detected.len() < 2 {
        "not_applicable_insufficient_responders"
    } else if pair_results.is_empty() {
        "indeterminate_no_distinct_depth_pair"
    } else if nsr_pair_count > 0 && indeterminate_count > 0 {
        "nonsequential_signature_incomplete_profile"
    } else if nsr_pair_count > 0 && sequential_pair_count > 0 {
        "mixed_response_ordering"
    } else if nsr_pair_count > 0 {
        "nonsequential_response_detected"
    } else if indeterminate_count > 0 {
        "indeterminate_incomplete_profile_without_nsr"
    } else if ambiguous_pair_count > 0 && nsr_pair_count == 0 && sequential_pair_count == 0 {
        "ambiguous_response_ordering_due_to_sampling_interval"
    } else if sequential_pair_count > 0 && ambiguous_pair_count > 0 {
        "sequential_response_with_ambiguous_pairs"
    } else if tied_pair_count > 0 && sequential_pair_count > 0 {
        "sequential_and_simultaneous_response"
    } else if tied_pair_count > 0 {
        "ambiguous_simultaneous_response"
    } else {
        "sequential_response_only"
    };

    let profile_coverage_status = if indeterminate_count > 0 {
        "incomplete"
    } else {
        "complete_for_declared_sensor_roster"
    };

    let mut warnings = vec![
        "NSR is a profile-level soil-moisture response-order signature, not proof of a unique pore pathway or measured preferential water flux.".to_owned(),
        "A sequential response does not rule out preferential flow; finite sensor spacing and sampling cadence can hide bypass.".to_owned(),
        "A nonsequential signature may reflect lateral routing, profile heterogeneity or measurement artefacts; depth ordering alone does not resolve the flow direction in 2D/3D.".to_owned(),
        "No-response sensors are distinct from indeterminate sensors. Indeterminate sensors reduce declared profile coverage.".to_owned(),
        "The response threshold and onset method are explicit inputs; studies with different onset definitions must not be pooled without a crosswalk.".to_owned(),
        "Threshold-crossing times are interval-censored between the last below-threshold sample and first above-threshold sample. Pairwise NSR/sequential labels are emitted only when intervals are strictly separated; overlapping intervals remain ambiguous.".to_owned(),
        "The result is limited to the supplied sensor roster; point sensors may miss narrow or laterally displaced preferential pathways.".to_owned(),
    ];
    if profile_coverage_status == "incomplete" && nsr_pair_count > 0 {
        warnings.push(
            "At least one nonsequential pair is observed, but the declared sensor roster is incomplete; interpret as a partial-profile signature.".to_owned(),
        );
    }

    Ok(Report {
        audit_version: AUDIT_VERSION,
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        plot_id: input.plot_id.clone(),
        profile_id: input.profile_id.clone(),
        event_id: input.event_id.clone(),
        event_start_utc: input.event_start_utc.clone(),
        observation_window_end_utc: input.observation_window_end_utc.clone(),
        time_resolution_seconds: input.time_resolution_seconds,
        response_threshold_m3_m3: input.response_threshold_m3_m3,
        response_onset_method: input.response_onset_method.clone(),
        sensor_count: sensors.len(),
        detected_sensor_count: detected.len(),
        not_detected_sensor_count: not_detected_count,
        indeterminate_sensor_count: indeterminate_count,
        distinct_detecting_depth_count: distinct_depths.len(),
        comparable_pair_count: pair_results.len(),
        nsr_pair_count,
        sequential_pair_count,
        tied_pair_count,
        ambiguous_pair_count,
        profile_coverage_status,
        profile_classification,
        pair_results,
        interpretation_boundary: "NSR is a profile-level sensor response-order diagnostic; it is not tracer confirmation, a unique macropore proof, flux partition or vertical/lateral flow attribution",
        warnings,
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: profile-flow-audit <profile-event-input.json> <new-report.json>",
        )
        .into());
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
        "profile event audited: {} sensors; {} pairs; {} NSR; {} sequential; class={}; coverage={}",
        report.sensor_count,
        report.comparable_pair_count,
        report.nsr_pair_count,
        report.sequential_pair_count,
        report.profile_classification,
        report.profile_coverage_status
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

    fn sensor(
        id: &str,
        depth: f64,
        status: ResponseStatus,
        response: Option<i64>,
    ) -> SensorResponse {
        SensorResponse {
            sensor_id: id.to_owned(),
            depth_cm: depth,
            response_status: status,
            response_onset_utc: response.map(stamp),
            response_onset_lower_bound_utc: None,
        }
    }

    fn input(sensors: Vec<SensorResponse>) -> Input {
        Input {
            schema_version: "0.3.0".to_owned(),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            event_id: "E001".to_owned(),
            event_start_utc: stamp(0),
            observation_window_end_utc: stamp(180),
            time_resolution_seconds: 600,
            response_threshold_m3_m3: 0.01,
            response_onset_method: "first_10min_bin_delta_theta_ge_threshold".to_owned(),
            source_sha256: "a".repeat(64),
            source_evidence_ref: "synthetic fixture only".to_owned(),
            sensors,
        }
    }

    #[test]
    fn deeper_sensor_responding_first_is_nsr() {
        let data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(30)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(10)),
        ]);
        let report = evaluate(&data, "b".repeat(64)).unwrap();
        assert_eq!(
            report.profile_classification,
            "nonsequential_response_detected"
        );
        assert_eq!(report.nsr_pair_count, 1);
        assert_eq!(report.pair_results[0].deep_advance_min_seconds, 600);
        assert_eq!(report.pair_results[0].deep_advance_max_seconds, 1800);
    }

    #[test]
    fn shallow_to_deep_order_is_sequential_not_nsr() {
        let data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(10)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(30)),
        ]);
        let report = evaluate(&data, "c".repeat(64)).unwrap();
        assert_eq!(report.profile_classification, "sequential_response_only");
        assert_eq!(report.nsr_pair_count, 0);
        assert_eq!(report.sequential_pair_count, 1);
    }

    #[test]
    fn mixed_pair_orderings_are_preserved() {
        let data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(50)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(10)),
            sensor("D60", 60.0, ResponseStatus::Detected, Some(30)),
        ]);
        let report = evaluate(&data, "d".repeat(64)).unwrap();
        assert_eq!(report.profile_classification, "mixed_response_ordering");
        assert_eq!(report.nsr_pair_count, 2);
        assert_eq!(report.sequential_pair_count, 1);
    }

    #[test]
    fn tied_responses_are_ambiguous_not_forced_into_a_direction() {
        let mut data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(10)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(10)),
        ]);
        data.sensors[0].response_onset_lower_bound_utc = Some(stamp(10));
        data.sensors[1].response_onset_lower_bound_utc = Some(stamp(10));
        let report = evaluate(&data, "e".repeat(64)).unwrap();
        assert_eq!(
            report.profile_classification,
            "ambiguous_simultaneous_response"
        );
        assert_eq!(report.tied_pair_count, 1);
        assert_eq!(report.nsr_pair_count, 0);
    }

    #[test]
    fn overlapping_onset_intervals_are_ambiguous_not_false_nsr() {
        let mut data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(30)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(20)),
        ]);
        data.sensors[0].response_onset_lower_bound_utc = Some(stamp(20));
        data.sensors[1].response_onset_lower_bound_utc = Some(stamp(10));
        let report = evaluate(&data, "9".repeat(64)).unwrap();
        assert_eq!(report.nsr_pair_count, 0);
        assert_eq!(report.sequential_pair_count, 0);
        assert_eq!(report.ambiguous_pair_count, 1);
        assert_eq!(
            report.profile_classification,
            "ambiguous_response_ordering_due_to_sampling_interval"
        );
    }

    #[test]
    fn rejects_response_interval_wider_than_one_sampling_step() {
        let mut data = input(vec![sensor(
            "D10",
            10.0,
            ResponseStatus::Detected,
            Some(30),
        )]);
        data.sensors[0].response_onset_lower_bound_utc = Some(stamp(10));
        assert!(validate(&data)
            .unwrap_err()
            .contains("no wider than one sampling interval"));
    }

    #[test]
    fn rejects_lower_bound_after_upper_bound() {
        let mut data = input(vec![sensor(
            "D10",
            10.0,
            ResponseStatus::Detected,
            Some(30),
        )]);
        data.sensors[0].response_onset_lower_bound_utc = Some(stamp(40));
        assert!(validate(&data)
            .unwrap_err()
            .contains("must be ordered"));
    }

    #[test]
    fn one_sensor_profile_is_not_applicable_not_invalid() {
        let data = input(vec![sensor(
            "D10",
            10.0,
            ResponseStatus::Detected,
            Some(10),
        )]);
        let report = evaluate(&data, "f".repeat(64)).unwrap();
        assert_eq!(
            report.profile_classification,
            "not_applicable_insufficient_responders"
        );
    }

    #[test]
    fn fewer_than_two_responders_is_not_applicable() {
        let data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(10)),
            sensor("D30", 30.0, ResponseStatus::NotDetected, None),
            sensor("D60", 60.0, ResponseStatus::Indeterminate, None),
        ]);
        let report = evaluate(&data, "1".repeat(64)).unwrap();
        assert_eq!(
            report.profile_classification,
            "not_applicable_insufficient_responders"
        );
        assert_eq!(report.not_detected_sensor_count, 1);
        assert_eq!(report.indeterminate_sensor_count, 1);
    }

    #[test]
    fn preserves_nsr_signature_but_marks_incomplete_roster() {
        let data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(30)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(10)),
            sensor("D60", 60.0, ResponseStatus::Indeterminate, None),
        ]);
        let report = evaluate(&data, "2".repeat(64)).unwrap();
        assert_eq!(
            report.profile_classification,
            "nonsequential_signature_incomplete_profile"
        );
        assert_eq!(report.profile_coverage_status, "incomplete");
    }

    #[test]
    fn same_depth_sensors_are_not_a_vertical_comparison() {
        let data = input(vec![
            sensor("D10-A", 10.0, ResponseStatus::Detected, Some(10)),
            sensor("D10-B", 10.0, ResponseStatus::Detected, Some(20)),
        ]);
        let report = evaluate(&data, "3".repeat(64)).unwrap();
        assert_eq!(report.comparable_pair_count, 0);
        assert_eq!(
            report.profile_classification,
            "indeterminate_no_distinct_depth_pair"
        );
    }

    #[test]
    fn rejects_duplicate_sensor_ids() {
        let mut data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(10)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(20)),
        ]);
        data.sensors[1].sensor_id = data.sensors[0].sensor_id.clone();
        assert!(validate(&data).unwrap_err().contains("duplicate sensor_id"));
    }

    #[test]
    fn rejects_response_outside_observation_window() {
        let data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(190)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(20)),
        ]);
        assert!(
            validate(&data)
                .unwrap_err()
                .contains("outside the declared event observation window")
        );
    }

    #[test]
    fn rejects_non_utc_timestamps() {
        let mut data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(10)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(20)),
        ]);
        data.event_start_utc = "2026-01-01T01:00:00+01:00".to_owned();
        assert!(validate(&data).unwrap_err().contains("zero UTC offset"));
    }

    #[test]
    fn rejects_onsets_off_the_declared_sampling_grid() {
        let mut data = input(vec![
            sensor("D10", 10.0, ResponseStatus::Detected, Some(10)),
            sensor("D30", 30.0, ResponseStatus::Detected, Some(20)),
        ]);
        data.sensors[0].response_onset_utc = Some("2026-01-01T00:15:00Z".to_owned());
        assert!(
            validate(&data)
                .unwrap_err()
                .contains("declared 600-second UTC sampling grid")
        );
    }

    #[test]
    fn rejects_unreported_response_threshold_or_method() {
        let mut data = input(vec![sensor(
            "D10",
            10.0,
            ResponseStatus::Detected,
            Some(10),
        )]);
        data.response_threshold_m3_m3 = 0.0;
        assert!(
            validate(&data)
                .unwrap_err()
                .contains("response_threshold_m3_m3")
        );
        data.response_threshold_m3_m3 = 0.01;
        data.response_onset_method = " ".to_owned();
        assert!(
            validate(&data)
                .unwrap_err()
                .contains("response_onset_method")
        );
    }

    #[test]
    fn rejects_bad_source_digest() {
        let mut data = input(vec![sensor(
            "D10",
            10.0,
            ResponseStatus::Detected,
            Some(10),
        )]);
        data.source_sha256 = "not-a-digest".to_owned();
        assert!(validate(&data).unwrap_err().contains("source_sha256"));
    }
}

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

const AUDIT_VERSION: &str = "0.1.0";
const MIN_RESOLUTION_SECONDS: u32 = 300;
const MAX_RESOLUTION_SECONDS: u32 = 1800;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum SampleStatus {
    Observed,
    LinearInterpolatedGapLt6h,
    Missing,
}

impl SampleStatus {
    fn is_missing(self) -> bool {
        matches!(self, Self::Missing)
    }
    fn is_interpolated(self) -> bool {
        matches!(self, Self::LinearInterpolatedGapLt6h)
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Baseline {
    timestamp_utc: String,
    volumetric_water_content_m3_m3: Option<f64>,
    status: SampleStatus,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    timestamp_utc: String,
    volumetric_water_content_m3_m3: Option<f64>,
    status: SampleStatus,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RateThreshold {
    maximum_diffuse_rate_m3_m3_per_hour: f64,
    method: String,
    evidence_ref: String,
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
    sensor_id: String,
    sensor_depth_cm: f64,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    baseline: Baseline,
    observations: Vec<Observation>,
    rate_threshold: Option<RateThreshold>,
    source_sha256: String,
    source_evidence_ref: String,
}

#[derive(Clone)]
struct ParsedObservation {
    timestamp: DateTime<FixedOffset>,
    theta: Option<f64>,
    status: SampleStatus,
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
    sensor_id: String,
    sensor_depth_cm: f64,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    baseline_theta_m3_m3: Option<f64>,
    baseline_status: &'static str,
    observation_count: usize,
    observed_count: usize,
    interpolated_count: usize,
    missing_count: usize,
    event_window_start_covered: bool,
    event_window_end_covered: bool,
    adjacent_intervals_evaluable: usize,
    adjacent_intervals_missing: usize,
    positive_rise_interval_count: usize,
    peak_theta_m3_m3: Option<f64>,
    amplitude_from_baseline_m3_m3: Option<f64>,
    cumulative_positive_rise_m3_m3: Option<f64>,
    maximum_rate_of_rise_m3_m3_per_hour: Option<f64>,
    maximum_rate_interval_start_utc: Option<String>,
    maximum_rate_interval_end_utc: Option<String>,
    maximum_rate_threshold_m3_m3_per_hour: Option<f64>,
    maximum_rate_threshold_method: Option<String>,
    maximum_rate_threshold_evidence_ref: Option<String>,
    trajectory_threshold_status: &'static str,
    coverage_status: &'static str,
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
            "{label}: timestamp must align to the declared {}-second UTC cadence",
            resolution_seconds
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
        (SampleStatus::Missing, Some(_)) => Err(format!("{label}: missing status requires null theta")),
        (SampleStatus::Observed | SampleStatus::LinearInterpolatedGapLt6h, None) => {
            Err(format!("{label}: non-missing status requires theta"))
        }
        (_, Some(value)) if !value.is_finite() || !(0.0..=1.0).contains(&value) => {
            Err(format!("{label}: theta must be finite and in [0, 1]"))
        }
        (_, Some(_)) => Ok(()),
    }
}

fn validate(input: &Input) -> Result<(DateTime<FixedOffset>, DateTime<FixedOffset>, DateTime<FixedOffset>, Vec<ParsedObservation>), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.plot_id, "plot_id"),
        (&input.profile_id, "profile_id"),
        (&input.event_id, "event_id"),
        (&input.sensor_id, "sensor_id"),
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
    if !(MIN_RESOLUTION_SECONDS..=MAX_RESOLUTION_SECONDS).contains(&input.time_resolution_seconds) {
        return Err("time_resolution_seconds must be between 300 and 1800 seconds".to_owned());
    }
    if !input.sensor_depth_cm.is_finite() || input.sensor_depth_cm <= 0.0 {
        return Err("sensor_depth_cm must be finite and greater than zero".to_owned());
    }

    let start = parse_utc(&input.event_start_utc, "event_start_utc", input.time_resolution_seconds)?;
    let end = parse_utc(&input.observation_window_end_utc, "observation_window_end_utc", input.time_resolution_seconds)?;
    let baseline_time = parse_utc(&input.baseline.timestamp_utc, "baseline.timestamp_utc", input.time_resolution_seconds)?;
    if start >= end {
        return Err("observation_window_end_utc must be after event_start_utc".to_owned());
    }
    if baseline_time + Duration::seconds(i64::from(input.time_resolution_seconds)) != start {
        return Err("baseline timestamp must be exactly one declared sampling interval before event start".to_owned());
    }
    validate_theta(
        input.baseline.volumetric_water_content_m3_m3,
        input.baseline.status,
        "baseline",
    )?;

    if let Some(threshold) = &input.rate_threshold {
        if !threshold.maximum_diffuse_rate_m3_m3_per_hour.is_finite()
            || threshold.maximum_diffuse_rate_m3_m3_per_hour <= 0.0
        {
            return Err("maximum_diffuse_rate_m3_m3_per_hour must be finite and greater than zero".to_owned());
        }
        nonblank(&threshold.method, "rate_threshold.method")?;
        nonblank(&threshold.evidence_ref, "rate_threshold.evidence_ref")?;
    }

    if input.observations.len() < 2 {
        return Err("at least two event-window observations are required".to_owned());
    }

    let mut parsed = Vec::with_capacity(input.observations.len());
    let mut previous: Option<DateTime<FixedOffset>> = None;
    let mut times_seen = HashSet::new();

    for (index, observation) in input.observations.iter().enumerate() {
        let time = parse_utc(
            &observation.timestamp_utc,
            &format!("observations[{index}].timestamp_utc"),
            input.time_resolution_seconds,
        )?;
        if time < start || time > end {
            return Err(format!("observations[{index}] lies outside event observation window"));
        }
        if !times_seen.insert(time.timestamp()) {
            return Err(format!("duplicate observation timestamp at index {index}"));
        }
        if let Some(prior) = previous {
            if (time - prior).num_seconds() != i64::from(input.time_resolution_seconds) {
                return Err("observation series must be strictly increasing and gap-free on the declared sampling grid".to_owned());
            }
        }
        validate_theta(
            observation.volumetric_water_content_m3_m3,
            observation.status,
            &format!("observations[{index}]"),
        )?;
        parsed.push(ParsedObservation {
            timestamp: time,
            theta: observation.volumetric_water_content_m3_m3,
            status: observation.status,
        });
        previous = Some(time);
    }
    Ok((start, end, baseline_time, parsed))
}

fn status_name(status: SampleStatus) -> &'static str {
    match status {
        SampleStatus::Observed => "observed",
        SampleStatus::LinearInterpolatedGapLt6h => "linear_interpolated_gap_lt6h",
        SampleStatus::Missing => "missing",
    }
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (_start, _end, _baseline_time, observations) = validate(input)?;
    let baseline = input.baseline.volumetric_water_content_m3_m3;
    let event_start = DateTime::parse_from_rfc3339(&input.event_start_utc).expect("validated UTC event start");
    let event_end = DateTime::parse_from_rfc3339(&input.observation_window_end_utc).expect("validated UTC event end");
    let event_window_start_covered = observations.first().is_some_and(|x| x.timestamp.timestamp() == event_start.timestamp());
    let event_window_end_covered = observations.last().is_some_and(|x| x.timestamp.timestamp() == event_end.timestamp());
    let observed_count = observations.iter().filter(|x| x.status == SampleStatus::Observed).count();
    let interpolated_count = observations.iter().filter(|x| x.status.is_interpolated()).count();
    let missing_count = observations.iter().filter(|x| x.status.is_missing()).count();

    let mut peak_theta: Option<f64> = None;
    for observation in &observations {
        if let Some(theta) = observation.theta {
            peak_theta = Some(peak_theta.map_or(theta, |peak| peak.max(theta)));
        }
    }
    let amplitude = match (baseline, peak_theta) {
        (Some(base), Some(peak)) => Some(peak - base),
        _ => None,
    };

    let mut evaluable = 0usize;
    let mut missing_intervals = 0usize;
    let mut positive_intervals = 0usize;
    let mut cumulative_positive_rise = 0.0_f64;
    let mut maximum_rate: Option<f64> = None;
    let mut maximum_start: Option<String> = None;
    let mut maximum_end: Option<String> = None;

    for pair in observations.windows(2) {
        match (pair[0].theta, pair[1].theta) {
            (Some(left), Some(right)) => {
                evaluable += 1;
                let delta = right - left;
                if delta > 0.0 {
                    positive_intervals += 1;
                    cumulative_positive_rise += delta;
                }
                let rate = (delta * 3600.0) / f64::from(input.time_resolution_seconds);
                if rate > 0.0 && maximum_rate.is_none_or(|current| rate > current) {
                    maximum_rate = Some(rate);
                    maximum_start = Some(pair[0].timestamp.to_rfc3339());
                    maximum_end = Some(pair[1].timestamp.to_rfc3339());
                }
            }
            _ => missing_intervals += 1,
        }
    }

    let peak_rate = if maximum_rate.is_some() {
        maximum_rate
    } else if evaluable > 0 {
        Some(0.0)
    } else {
        None
    };

    let threshold_status = match (peak_rate, input.rate_threshold.as_ref()) {
        (_, None) => "not_evaluated_no_diffuse_rate_threshold",
        (None, Some(_)) => "indeterminate_no_evaluable_intervals",
        (Some(rate), Some(threshold)) if rate > threshold.maximum_diffuse_rate_m3_m3_per_hour => {
            "above_declared_rate_threshold_proxy_only"
        }
        (Some(_), Some(_)) => "not_above_declared_rate_threshold",
    };

    let coverage_status = if !event_window_start_covered || !event_window_end_covered {
        "incomplete_event_window_boundary"
    } else if missing_count > 0 || missing_intervals > 0 {
        "incomplete_due_to_missing_observations"
    } else if interpolated_count > 0 {
        "complete_with_interpolated_observations"
    } else {
        "complete_observed_only"
    };

    let mut warnings = vec![
        "Maximum rate of rise is a trajectory signature; without an independently justified diffuse-flow threshold it is not a PF classification.".to_owned(),
        "Even with a supplied threshold, the result is a trajectory-based proxy, not tracer-confirmed preferential flow.".to_owned(),
        "No smoothing is applied: a single noisy positive increment can dominate the maximum-rate statistic. Preserve measurement precision and perform a predeclared smoothing sensitivity analysis separately.".to_owned(),
        "Interpolated observations are included in metrics but counted separately; missing intervals are skipped and make the trajectory coverage incomplete.".to_owned(),
        "Cumulative positive rise sums successive positive water-content increments and is not a volume or flux without a spatial support/control-volume model.".to_owned(),
        "The observation window, sample interval and baseline are explicit; peak/maximum metrics can depend on window length and cadence.".to_owned(),
    ];
    if !event_window_start_covered || !event_window_end_covered {
        warnings.push("The observed series does not span the full declared event window; the trajectory is left- or right-censored.".to_owned());
    }
    if interpolated_count > 0 {
        warnings.push("At least one interpolated observation contributed to the trajectory; report a no-interpolation sensitivity result.".to_owned());
    }
    if missing_count > 0 {
        warnings.push("Missing samples can conceal a faster unobserved wetting limb; the reported maximum rise rate may be a lower bound.".to_owned());
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
        sensor_id: input.sensor_id.clone(),
        sensor_depth_cm: input.sensor_depth_cm,
        event_start_utc: input.event_start_utc.clone(),
        observation_window_end_utc: input.observation_window_end_utc.clone(),
        time_resolution_seconds: input.time_resolution_seconds,
        baseline_theta_m3_m3: baseline,
        baseline_status: status_name(input.baseline.status),
        observation_count: observations.len(),
        observed_count,
        interpolated_count,
        missing_count,
        event_window_start_covered,
        event_window_end_covered,
        adjacent_intervals_evaluable: evaluable,
        adjacent_intervals_missing: missing_intervals,
        positive_rise_interval_count: positive_intervals,
        peak_theta_m3_m3: peak_theta,
        amplitude_from_baseline_m3_m3: amplitude,
        cumulative_positive_rise_m3_m3: if evaluable > 0 { Some(cumulative_positive_rise) } else { None },
        maximum_rate_of_rise_m3_m3_per_hour: peak_rate,
        maximum_rate_interval_start_utc: maximum_start,
        maximum_rate_interval_end_utc: maximum_end,
        maximum_rate_threshold_m3_m3_per_hour: input.rate_threshold.as_ref().map(|x| x.maximum_diffuse_rate_m3_m3_per_hour),
        maximum_rate_threshold_method: input.rate_threshold.as_ref().map(|x| x.method.clone()),
        maximum_rate_threshold_evidence_ref: input.rate_threshold.as_ref().map(|x| x.evidence_ref.clone()),
        trajectory_threshold_status: threshold_status,
        coverage_status,
        interpretation_boundary: "soil-moisture trajectory signature and optional threshold proxy only; not direct preferential water flux or independent pathway truth",
        warnings,
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: trajectory-signature-audit <event-series.json> <new-report.json>",
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
        "trajectory signature complete: event={} sensor={} max_rate={:?} m3/m3/h threshold_status={} coverage={}",
        report.event_id,
        report.sensor_id,
        report.maximum_rate_of_rise_m3_m3_per_hour,
        report.trajectory_threshold_status,
        report.coverage_status
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("trajectory signature audit failed closed: {error}");
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

    fn base() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            event_id: "E001".to_owned(),
            sensor_id: "D30".to_owned(),
            sensor_depth_cm: 30.0,
            event_start_utc: stamp(0),
            observation_window_end_utc: stamp(40),
            time_resolution_seconds: 600,
            baseline: Baseline {
                timestamp_utc: stamp(-10),
                volumetric_water_content_m3_m3: Some(0.20),
                status: SampleStatus::Observed,
            },
            observations: vec![
                Observation {
                    timestamp_utc: stamp(0),
                    volumetric_water_content_m3_m3: Some(0.205),
                    status: SampleStatus::Observed,
                },
                Observation {
                    timestamp_utc: stamp(10),
                    volumetric_water_content_m3_m3: Some(0.210),
                    status: SampleStatus::Observed,
                },
                Observation {
                    timestamp_utc: stamp(20),
                    volumetric_water_content_m3_m3: Some(0.205),
                    status: SampleStatus::Observed,
                },
                Observation {
                    timestamp_utc: stamp(30),
                    volumetric_water_content_m3_m3: Some(0.220),
                    status: SampleStatus::Observed,
                },
                Observation {
                    timestamp_utc: stamp(40),
                    volumetric_water_content_m3_m3: Some(0.218),
                    status: SampleStatus::Observed,
                },
            ],
            rate_threshold: Some(RateThreshold {
                maximum_diffuse_rate_m3_m3_per_hour: 0.08,
                method: "synthetic null/model threshold".to_owned(),
                evidence_ref: "synthetic fixture only".to_owned(),
            }),
            source_sha256: "a".repeat(64),
            source_evidence_ref: "synthetic fixture only".to_owned(),
        }
    }

    #[test]
    fn detects_fastest_rising_limb_across_multiple_peaks() {
        let report = evaluate(&base(), "b".repeat(64)).unwrap();
        assert_eq!(report.maximum_rate_of_rise_m3_m3_per_hour, Some(0.09));
        assert_eq!(report.maximum_rate_interval_start_utc, Some(stamp(20)));
        assert_eq!(report.maximum_rate_interval_end_utc, Some(stamp(30)));
        assert_eq!(report.amplitude_from_baseline_m3_m3, Some(0.02));
        assert_eq!(report.cumulative_positive_rise_m3_m3, Some(0.025));
        assert_eq!(report.trajectory_threshold_status, "above_declared_rate_threshold_proxy_only");
    }

    #[test]
    fn rate_conversion_uses_actual_declared_sampling_interval() {
        let mut input = base();
        input.time_resolution_seconds = 300;
        input.observation_window_end_utc = stamp(20);
        input.baseline.timestamp_utc = stamp(-5);
        input.observations = vec![
            Observation { timestamp_utc: stamp(0), volumetric_water_content_m3_m3: Some(0.20), status: SampleStatus::Observed },
            Observation { timestamp_utc: stamp(5), volumetric_water_content_m3_m3: Some(0.21), status: SampleStatus::Observed },
            Observation { timestamp_utc: stamp(10), volumetric_water_content_m3_m3: Some(0.22), status: SampleStatus::Observed },
            Observation { timestamp_utc: stamp(15), volumetric_water_content_m3_m3: Some(0.23), status: SampleStatus::Observed },
            Observation { timestamp_utc: stamp(20), volumetric_water_content_m3_m3: Some(0.24), status: SampleStatus::Observed },
        ];
        let report = evaluate(&input, "c".repeat(64)).unwrap();
        assert_eq!(report.maximum_rate_of_rise_m3_m3_per_hour, Some(0.12));
    }

    #[test]
    fn records_missingness_and_marks_coverage_incomplete() {
        let mut input = base();
        input.observations[2].volumetric_water_content_m3_m3 = None;
        input.observations[2].status = SampleStatus::Missing;
        let report = evaluate(&input, "d".repeat(64)).unwrap();
        assert_eq!(report.coverage_status, "incomplete_due_to_missing_observations");
        assert_eq!(report.missing_count, 1);
        assert_eq!(report.adjacent_intervals_missing, 2);
    }

    #[test]
    fn interpolated_samples_are_counted_and_flagged() {
        let mut input = base();
        input.observations[2].status = SampleStatus::LinearInterpolatedGapLt6h;
        let report = evaluate(&input, "e".repeat(64)).unwrap();
        assert_eq!(report.interpolated_count, 1);
        assert_eq!(report.coverage_status, "complete_with_interpolated_observations");
    }

    #[test]
    fn no_rate_threshold_means_signature_only_not_pf_proxy() {
        let mut input = base();
        input.rate_threshold = None;
        let report = evaluate(&input, "f".repeat(64)).unwrap();
        assert_eq!(report.trajectory_threshold_status, "not_evaluated_no_diffuse_rate_threshold");
        assert_eq!(report.maximum_rate_threshold_m3_m3_per_hour, None);
    }

    #[test]
    fn missing_baseline_does_not_produce_amplitude_but_rate_remains_evaluable() {
        let mut input = base();
        input.baseline.status = SampleStatus::Missing;
        input.baseline.volumetric_water_content_m3_m3 = None;
        let report = evaluate(&input, "1".repeat(64)).unwrap();
        assert_eq!(report.amplitude_from_baseline_m3_m3, None);
        assert_eq!(report.trajectory_threshold_status, "above_declared_rate_threshold_proxy_only");
    }

    #[test]
    fn partial_event_window_is_marked_censored() {
        let mut input = base();
        input.observation_window_end_utc = stamp(50);
        let report = evaluate(&input, "2".repeat(64)).unwrap();
        assert!(report.event_window_start_covered);
        assert!(!report.event_window_end_covered);
        assert_eq!(report.coverage_status, "incomplete_event_window_boundary");
    }

    #[test]
    fn left_censored_event_window_is_marked_incomplete() {
        let mut input = base();
        input.observations.remove(0);
        input.observations.push(Observation {
            timestamp_utc: stamp(50),
            volumetric_water_content_m3_m3: Some(0.22),
            status: SampleStatus::Observed,
        });
        input.observation_window_end_utc = stamp(50);
        let report = evaluate(&input, "3".repeat(64)).unwrap();
        assert!(!report.event_window_start_covered);
        assert!(report.event_window_end_covered);
        assert_eq!(report.coverage_status, "incomplete_event_window_boundary");
    }

    #[test]
    fn rejects_irregular_time_grid() {
        let mut input = base();
        input.observations[2].timestamp_utc = stamp(25);
        assert!(validate(&input).unwrap_err().contains("gap-free on the declared sampling grid"));
    }

    #[test]
    fn rejects_mismatched_baseline_timestamp() {
        let mut input = base();
        input.baseline.timestamp_utc = stamp(-20);
        assert!(validate(&input).unwrap_err().contains("exactly one declared sampling interval"));
    }

    #[test]
    fn rejects_non_utc_observation_timestamp() {
        let mut input = base();
        input.observations[0].timestamp_utc = "2026-01-01T01:00:00+01:00".to_owned();
        assert!(validate(&input).unwrap_err().contains("zero UTC offset"));
    }

    #[test]
    fn rejects_invalid_threshold_evidence() {
        let mut input = base();
        input.rate_threshold.as_mut().unwrap().evidence_ref = " ".to_owned();
        assert!(validate(&input).unwrap_err().contains("evidence_ref"));
    }
}

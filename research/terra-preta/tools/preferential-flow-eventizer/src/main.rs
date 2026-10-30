use chrono::{DateTime, Duration, FixedOffset};
#[cfg(test)]
use chrono::{TimeZone, Utc};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const STEP_SECONDS: i64 = 600;
const EVENT_GAP_MINUTES: i64 = 360;
const MAX_EVENT_MINUTES: i64 = 5 * 24 * 60;
const RESPONSE_THRESHOLD: f64 = 0.01;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    site_id: String,
    plot_id: String,
    sensor_id: String,
    sensor_depth_cm: f64,
    max_matrix_ksat_mm_h: Option<f64>,
    max_matrix_ksat_evidence_ref: Option<String>,
    precipitation_product_code: String,
    precipitation_release: String,
    precipitation_gauge_kind: String,
    soil_water_product_code: String,
    soil_water_release: String,
    rainfall: Vec<RainSample>,
    soil_moisture: Vec<MoistureSample>,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum RainStatus {
    Observed,
    PaperZeroImputed,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RainSample {
    timestamp_utc: String,
    precipitation_mm_per_10min: f64,
    status: RainStatus,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum MoistureStatus {
    ObservedGood,
    LinearInterpolatedGapLt6h,
    Missing,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MoistureSample {
    timestamp_utc: String,
    volumetric_water_content_m3_m3: Option<f64>,
    status: MoistureStatus,
}

#[derive(Clone, Debug)]
struct Rain {
    time: DateTime<FixedOffset>,
    amount_mm: f64,
}

#[derive(Clone, Debug)]
struct Moisture {
    time: DateTime<FixedOffset>,
    theta: Option<f64>,
    status: MoistureStatus,
}

struct Event {
    start: DateTime<FixedOffset>,
    last_rain: DateTime<FixedOffset>,
    total_mm: f64,
    peak_mm_h: f64,
    pulse_count: usize,
}

struct Response {
    status: &'static str,
    onset: Option<String>,
    lag_minutes: Option<i64>,
}

#[derive(Serialize)]
struct EventResult {
    event_index: usize,
    onset_utc: String,
    last_rain_bin_utc: String,
    observation_window_end_utc: String,
    rainfall_total_mm: f64,
    rainfall_duration_hours: f64,
    observation_window_hours: f64,
    peak_intensity_mm_h: f64,
    nonzero_pulse_count: usize,
    response_status: String,
    response_onset_utc: Option<String>,
    response_lag_minutes: Option<i64>,
    apparent_response_velocity_mm_h: Option<f64>,
    preferential_flow_proxy: Option<bool>,
    proxy_classification_status: String,
}

#[derive(Serialize)]
struct Report {
    tool_version: &'static str,
    input_sha256: String,
    site_id: String,
    plot_id: String,
    sensor_id: String,
    sensor_depth_cm: f64,
    max_matrix_ksat_mm_h: Option<f64>,
    max_matrix_ksat_evidence_ref: Option<String>,
    precipitation_product_code: String,
    precipitation_release: String,
    precipitation_gauge_kind: String,
    soil_water_product_code: String,
    soil_water_release: String,
    event_gap_minutes: i64,
    maximum_event_duration_minutes: i64,
    response_threshold_m3_m3: f64,
    response_baseline_rule: &'static str,
    event_count: usize,
    response_detected_count: usize,
    proxy_classified_count: usize,
    events: Vec<EventResult>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn parse_time(value: &str, label: &str) -> Result<DateTime<FixedOffset>, String> {
    let parsed = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if parsed.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp_utc must use a zero UTC offset"));
    }
    if parsed.timestamp() % STEP_SECONDS != 0 || parsed.timestamp_subsec_nanos() != 0 {
        return Err(format!(
            "{label}: timestamp is not aligned to a 10-minute UTC boundary"
        ));
    }
    Ok(parsed)
}

fn check_regular(times: &[DateTime<FixedOffset>], label: &str) -> Result<(), String> {
    if times.len() < 2 {
        return Err(format!(
            "{label}: at least two 10-minute samples are required"
        ));
    }
    for pair in times.windows(2) {
        let elapsed = (pair[1] - pair[0]).num_seconds();
        if elapsed != STEP_SECONDS {
            return Err(format!(
                "{label}: samples must be strictly increasing and gap-free at 10-minute cadence; found {elapsed} seconds"
            ));
        }
    }
    Ok(())
}

fn validate(input: &Input) -> Result<(Vec<Rain>, Vec<Moisture>), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!(
            "unsupported schema_version {:?}",
            input.schema_version
        ));
    }
    for (value, label) in [
        (&input.site_id, "site_id"),
        (&input.plot_id, "plot_id"),
        (&input.sensor_id, "sensor_id"),
        (
            &input.precipitation_product_code,
            "precipitation_product_code",
        ),
        (&input.precipitation_release, "precipitation_release"),
        (&input.precipitation_gauge_kind, "precipitation_gauge_kind"),
        (&input.soil_water_product_code, "soil_water_product_code"),
        (&input.soil_water_release, "soil_water_release"),
    ] {
        if value.trim().is_empty() {
            return Err(format!("{label} must not be blank"));
        }
    }
    if !input.sensor_depth_cm.is_finite() || input.sensor_depth_cm <= 0.0 {
        return Err("sensor_depth_cm must be finite and greater than zero".to_owned());
    }
    if let Some(ksat) = input.max_matrix_ksat_mm_h {
        if !ksat.is_finite() || ksat <= 0.0 {
            return Err("max_matrix_ksat_mm_h must be finite and greater than zero".to_owned());
        }
    }
    match (
        input.max_matrix_ksat_mm_h,
        input.max_matrix_ksat_evidence_ref.as_deref(),
    ) {
        (Some(_), Some(reference)) if !reference.trim().is_empty() => {}
        (Some(_), _) => {
            return Err(
                "a supplied max_matrix_ksat_mm_h requires max_matrix_ksat_evidence_ref".to_owned(),
            );
        }
        (None, Some(_)) => {
            return Err(
                "max_matrix_ksat_evidence_ref cannot be supplied without max_matrix_ksat_mm_h"
                    .to_owned(),
            );
        }
        (None, None) => {}
    }
    if input.precipitation_product_code == "DP1.00046.001" {
        return Err("throughfall must not be relabelled as open precipitation".to_owned());
    }
    if input.precipitation_product_code == "DP1.00044.001" {
        return Err("hourly/daily weighing-gauge precipitation cannot reconstruct the paper's 10-minute intensity series".to_owned());
    }
    if !["DP1.00006.001", "DP1.00045.001"].contains(&input.precipitation_product_code.as_str()) {
        return Err("v0.1 accepts only the historical precipitation product or current tipping-bucket product".to_owned());
    }
    if input.soil_water_product_code != "DP1.00094.001" {
        return Err("v0.1 requires soil-water product DP1.00094.001".to_owned());
    }
    if input.rainfall.is_empty() || input.soil_moisture.is_empty() {
        return Err("rainfall and soil_moisture arrays must not be empty".to_owned());
    }

    let mut rain = Vec::with_capacity(input.rainfall.len());
    for (index, sample) in input.rainfall.iter().enumerate() {
        let time = parse_time(&sample.timestamp_utc, &format!("rainfall[{index}]"))?;
        let amount = sample.precipitation_mm_per_10min;
        if !amount.is_finite() || amount < 0.0 {
            return Err(format!(
                "rainfall[{index}] amount must be finite and non-negative"
            ));
        }
        if matches!(sample.status, RainStatus::PaperZeroImputed) && amount != 0.0 {
            return Err(format!(
                "rainfall[{index}] paper_zero_imputed must be exactly zero"
            ));
        }
        rain.push(Rain {
            time,
            amount_mm: amount,
        });
    }
    let rain_times: Vec<_> = rain.iter().map(|sample| sample.time.clone()).collect();
    check_regular(&rain_times, "rainfall")?;

    let mut moisture = Vec::with_capacity(input.soil_moisture.len());
    for (index, sample) in input.soil_moisture.iter().enumerate() {
        let time = parse_time(&sample.timestamp_utc, &format!("soil_moisture[{index}]"))?;
        match (sample.status, sample.volumetric_water_content_m3_m3) {
            (MoistureStatus::Missing, None) => {}
            (MoistureStatus::Missing, Some(_)) => {
                return Err(format!(
                    "soil_moisture[{index}] missing status must have a null value"
                ));
            }
            (MoistureStatus::ObservedGood | MoistureStatus::LinearInterpolatedGapLt6h, None) => {
                return Err(format!(
                    "soil_moisture[{index}] non-missing status requires a value"
                ));
            }
            (_, Some(value)) if !value.is_finite() || !(0.0..=1.0).contains(&value) => {
                return Err(format!(
                    "soil_moisture[{index}] water content must be finite and within [0, 1]"
                ));
            }
            _ => {}
        }
        moisture.push(Moisture {
            time,
            theta: sample.volumetric_water_content_m3_m3,
            status: sample.status,
        });
    }
    let moisture_times: Vec<_> = moisture.iter().map(|sample| sample.time.clone()).collect();
    check_regular(&moisture_times, "soil_moisture")?;
    Ok((rain, moisture))
}

fn delineate_events(rain: &[Rain]) -> Vec<Event> {
    let mut events: Vec<Event> = Vec::new();
    for sample in rain.iter().filter(|sample| sample.amount_mm > 0.0) {
        let joins_previous = events
            .last()
            .map(|event| {
                let gap = (sample.time - event.last_rain).num_minutes();
                let elapsed = (sample.time - event.start).num_minutes();
                gap <= EVENT_GAP_MINUTES && elapsed <= MAX_EVENT_MINUTES
            })
            .unwrap_or(false);

        if joins_previous {
            let event = events
                .last_mut()
                .expect("last event exists after join check");
            event.last_rain = sample.time.clone();
            event.total_mm += sample.amount_mm;
            event.peak_mm_h = event.peak_mm_h.max(sample.amount_mm * 6.0);
            event.pulse_count += 1;
        } else {
            events.push(Event {
                start: sample.time.clone(),
                last_rain: sample.time.clone(),
                total_mm: sample.amount_mm,
                peak_mm_h: sample.amount_mm * 6.0,
                pulse_count: 1,
            });
        }
    }
    events
}

fn detect_response(event: &Event, moisture: &[Moisture]) -> Response {
    let baseline_time = event.start - Duration::minutes(10);
    let Some(base) = moisture.iter().find(|sample| sample.time == baseline_time) else {
        return Response {
            status: "indeterminate_missing_baseline",
            onset: None,
            lag_minutes: None,
        };
    };
    let Some(base_theta) = base.theta else {
        return Response {
            status: "indeterminate_missing_baseline",
            onset: None,
            lag_minutes: None,
        };
    };
    if matches!(base.status, MoistureStatus::Missing) {
        return Response {
            status: "indeterminate_missing_baseline",
            onset: None,
            lag_minutes: None,
        };
    }

    let gap_close = event.last_rain + Duration::minutes(EVENT_GAP_MINUTES);
    let duration_close = event.start + Duration::minutes(MAX_EVENT_MINUTES);
    let close = if gap_close < duration_close {
        gap_close
    } else {
        duration_close
    };
    let mut missing_before_detection = false;

    for sample in moisture
        .iter()
        .filter(|sample| sample.time > event.start && sample.time <= close)
    {
        if matches!(sample.status, MoistureStatus::Missing) {
            missing_before_detection = true;
            break;
        }
        if sample.theta.unwrap_or(base_theta) - base_theta >= RESPONSE_THRESHOLD {
            return Response {
                status: "detected",
                onset: Some(sample.time.to_rfc3339()),
                lag_minutes: Some((sample.time - event.start).num_minutes()),
            };
        }
    }
    if missing_before_detection {
        Response {
            status: "indeterminate_data_gap",
            onset: None,
            lag_minutes: None,
        }
    } else if moisture
        .last()
        .map(|sample| sample.time < close)
        .unwrap_or(true)
    {
        Response {
            status: "right_censored_insufficient_followup",
            onset: None,
            lag_minutes: None,
        }
    } else {
        Response {
            status: "not_detected",
            onset: None,
            lag_minutes: None,
        }
    }
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (rain, moisture) = validate(input)?;
    let raw_events = delineate_events(&rain);
    let mut events = Vec::with_capacity(raw_events.len());
    let mut response_detected_count = 0;
    let mut proxy_classified_count = 0;

    for (index, event) in raw_events.iter().enumerate() {
        let gap_close = event.last_rain + Duration::minutes(EVENT_GAP_MINUTES);
        let duration_close = event.start + Duration::minutes(MAX_EVENT_MINUTES);
        let close = if gap_close < duration_close {
            gap_close
        } else {
            duration_close
        };
        let response = detect_response(event, &moisture);
        let event_minutes = (event.last_rain - event.start).num_minutes() + 10;
        let window_minutes = (close - event.start).num_minutes();
        let velocity = response
            .lag_minutes
            .map(|lag| input.sensor_depth_cm * 10.0 / (lag as f64 / 60.0));

        let (proxy, proxy_status) = match (
            response.status,
            velocity,
            (10.0..=100.0).contains(&input.sensor_depth_cm),
            input.max_matrix_ksat_mm_h,
        ) {
            ("detected", Some(value), true, Some(ksat)) => {
                proxy_classified_count += 1;
                (
                    Some(value > ksat),
                    "classified_velocity_threshold_vs_supplied_max_matrix_ksat",
                )
            }
            ("detected", Some(_), false, _) => {
                (None, "outside_reference_sensor_depth_range_10_to_100_cm")
            }
            ("detected", Some(_), true, None) => (None, "not_classified_missing_ksat_reference"),
            _ => (None, "not_classified_without_detected_response"),
        };
        if response.status == "detected" {
            response_detected_count += 1;
        }

        events.push(EventResult {
            event_index: index + 1,
            onset_utc: event.start.to_rfc3339(),
            last_rain_bin_utc: event.last_rain.to_rfc3339(),
            observation_window_end_utc: close.to_rfc3339(),
            rainfall_total_mm: event.total_mm,
            rainfall_duration_hours: event_minutes as f64 / 60.0,
            observation_window_hours: window_minutes as f64 / 60.0,
            peak_intensity_mm_h: event.peak_mm_h,
            nonzero_pulse_count: event.pulse_count,
            response_status: response.status.to_owned(),
            response_onset_utc: response.onset,
            response_lag_minutes: response.lag_minutes,
            apparent_response_velocity_mm_h: velocity,
            preferential_flow_proxy: proxy,
            proxy_classification_status: proxy_status.to_owned(),
        });
    }

    Ok(Report {
        tool_version: "0.1.0",
        input_sha256,
        site_id: input.site_id.clone(),
        plot_id: input.plot_id.clone(),
        sensor_id: input.sensor_id.clone(),
        sensor_depth_cm: input.sensor_depth_cm,
        max_matrix_ksat_mm_h: input.max_matrix_ksat_mm_h,
        max_matrix_ksat_evidence_ref: input.max_matrix_ksat_evidence_ref.clone(),
        precipitation_product_code: input.precipitation_product_code.clone(),
        precipitation_release: input.precipitation_release.clone(),
        precipitation_gauge_kind: input.precipitation_gauge_kind.clone(),
        soil_water_product_code: input.soil_water_product_code.clone(),
        soil_water_release: input.soil_water_release.clone(),
        event_gap_minutes: EVENT_GAP_MINUTES,
        maximum_event_duration_minutes: MAX_EVENT_MINUTES,
        response_threshold_m3_m3: RESPONSE_THRESHOLD,
        response_baseline_rule: "immediately preceding 10-minute soil-moisture sample",
        event_count: events.len(),
        response_detected_count,
        proxy_classified_count,
        events,
        interpretation_boundary: "apparent sensor-response velocity and Ksat threshold proxy are not direct pore-water velocity or tracer-confirmed preferential-flow truth",
        warnings: vec![
            "Input must be QA-filtered and median-aggregated to 10-minute cadence; raw NEON QA processing is not performed here.".to_owned(),
            "The immediately-preceding-bin response baseline is an operationalization; verify against author code/supplement before claiming exact reproduction.".to_owned(),
            "Interpolated values are accepted only when explicitly marked as a gap shorter than six hours by upstream processing.".to_owned(),
            "A missing moisture bin before threshold crossing yields indeterminate; no response requires full follow-up through the event window.".to_owned(),
            "The supplied Ksat value is not calculated by this tool and is not independent pathway ground truth.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: preferential-flow-eventizer <normalized-input.json> <new-report.json>",
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
        "eventizer complete: {} events; {} responses; {} proxy classifications; input SHA-256 {}",
        report.event_count,
        report.response_detected_count,
        report.proxy_classified_count,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("preferential-flow eventizer failed closed: {error}");
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

    fn fixture(pulses: &[i64], last_rain_minute: i64) -> Input {
        let rainfall = (-10..=last_rain_minute)
            .step_by(10)
            .map(|minute| RainSample {
                timestamp_utc: stamp(minute),
                precipitation_mm_per_10min: if pulses.contains(&minute) { 2.0 } else { 0.0 },
                status: RainStatus::Observed,
            })
            .collect();
        let soil_moisture = (-10..=(last_rain_minute + 360))
            .step_by(10)
            .map(|minute| MoistureSample {
                timestamp_utc: stamp(minute),
                volumetric_water_content_m3_m3: Some(if minute == 20 { 0.21 } else { 0.20 }),
                status: MoistureStatus::ObservedGood,
            })
            .collect();
        Input {
            schema_version: "0.1.0".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            sensor_id: "D10".to_owned(),
            sensor_depth_cm: 10.0,
            max_matrix_ksat_mm_h: Some(250.0),
            max_matrix_ksat_evidence_ref: Some(
                "synthetic fixture only; no Rosetta3 claim".to_owned(),
            ),
            precipitation_product_code: "DP1.00045.001".to_owned(),
            precipitation_release: "RELEASE-2026".to_owned(),
            precipitation_gauge_kind: "tipping_bucket".to_owned(),
            soil_water_product_code: "DP1.00094.001".to_owned(),
            soil_water_release: "RELEASE-2026".to_owned(),
            rainfall,
            soil_moisture,
        }
    }

    #[test]
    fn joins_pulses_at_six_hours_and_splits_after_six_hours() {
        let input = fixture(&[0, 360, 730], 730);
        let (rain, _) = validate(&input).unwrap();
        let events = delineate_events(&rain);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].pulse_count, 2);
        assert_eq!(events[0].total_mm, 4.0);
    }

    #[test]
    fn splits_when_event_duration_exceeds_five_days() {
        let mut pulses: Vec<i64> = (0..=MAX_EVENT_MINUTES).step_by(360).collect();
        pulses.push(MAX_EVENT_MINUTES + 60);
        let input = fixture(&pulses, MAX_EVENT_MINUTES + 60);
        let (rain, _) = validate(&input).unwrap();
        let events = delineate_events(&rain);
        assert_eq!(events.len(), 2);
        assert_eq!(
            (events[0].last_rain - events[0].start).num_minutes(),
            MAX_EVENT_MINUTES
        );
    }

    #[test]
    fn response_velocity_and_proxy_units_are_correctly_labelled() {
        let input = fixture(&[0], 0);
        let report = evaluate(&input, "a".repeat(64)).unwrap();
        assert_eq!(report.events[0].response_status, "detected");
        assert_eq!(report.events[0].response_lag_minutes, Some(20));
        assert_eq!(
            report.events[0].apparent_response_velocity_mm_h,
            Some(300.0)
        );
        assert_eq!(report.events[0].preferential_flow_proxy, Some(true));
    }

    #[test]
    fn missing_moisture_bin_before_detection_abstains() {
        let mut input = fixture(&[0], 0);
        let sample = input
            .soil_moisture
            .iter_mut()
            .find(|sample| sample.timestamp_utc == stamp(10))
            .unwrap();
        sample.status = MoistureStatus::Missing;
        sample.volumetric_water_content_m3_m3 = None;
        let (rain, moisture) = validate(&input).unwrap();
        let event = delineate_events(&rain).remove(0);
        assert_eq!(
            detect_response(&event, &moisture).status,
            "indeterminate_data_gap"
        );
    }

    #[test]
    fn sensor_depth_outside_reference_range_is_not_proxy_classified() {
        let mut input = fixture(&[0], 0);
        input.sensor_depth_cm = 120.0;
        let report = evaluate(&input, "b".repeat(64)).unwrap();
        assert_eq!(report.events[0].preferential_flow_proxy, None);
    }

    #[test]
    fn rejects_weighing_gauge_and_throughfall_as_open_rainfall() {
        let mut input = fixture(&[0], 0);
        input.precipitation_product_code = "DP1.00044.001".to_owned();
        assert!(
            validate(&input)
                .unwrap_err()
                .contains("10-minute intensity")
        );
        input.precipitation_product_code = "DP1.00046.001".to_owned();
        assert!(validate(&input).unwrap_err().contains("throughfall"));
    }

    #[test]
    fn rejects_irregular_time_bins() {
        let mut input = fixture(&[0], 30);
        input.rainfall[2].timestamp_utc = stamp(20);
        assert!(
            validate(&input)
                .unwrap_err()
                .contains("gap-free at 10-minute cadence")
        );
    }

    #[test]
    fn rejects_nonzero_utc_offset_in_timestamp_utc() {
        let mut input = fixture(&[0], 0);
        input.rainfall[0].timestamp_utc = "2026-01-01T00:00:00+01:00".to_owned();
        assert!(validate(&input).unwrap_err().contains("zero UTC offset"));
    }

    #[test]
    fn rejects_ksat_without_evidence_reference() {
        let mut input = fixture(&[0], 0);
        input.max_matrix_ksat_evidence_ref = None;
        assert!(
            validate(&input)
                .unwrap_err()
                .contains("requires max_matrix_ksat_evidence_ref")
        );
    }

    #[test]
    fn paper_zero_imputation_must_be_exactly_zero() {
        let mut input = fixture(&[0], 0);
        input.rainfall[1].status = RainStatus::PaperZeroImputed;
        input.rainfall[1].precipitation_mm_per_10min = 0.2;
        assert!(validate(&input).unwrap_err().contains("exactly zero"));
    }
}

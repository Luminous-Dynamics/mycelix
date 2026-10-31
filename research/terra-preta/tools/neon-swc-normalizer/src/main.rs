use chrono::{DateTime, Duration, FixedOffset};
#[cfg(test)]
use chrono::{TimeZone, Utc};
use csv::StringRecord;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{HashMap, HashSet},
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const RAW_RESOLUTION_SECONDS: i64 = 60;
const MIN_TARGET_SECONDS: u32 = 300;
const MAX_TARGET_SECONDS: u32 = 1800;
const MAX_EVENT_SECONDS: i64 = 6 * 24 * 60 * 60;

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
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
struct Config {
    schema_version: String,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    raw_resolution_seconds: u32,
    target_resolution_seconds: u32,
    minimum_good_fraction: f64,
    response_threshold_m3_m3: f64,
    maximum_profile_porosity_m3_m3: f64,
    maximum_profile_porosity_evidence_ref: String,
    raw_file_label: String,
    source_evidence_ref: String,
    timestamp_column: String,
    sensor_key_columns: Vec<String>,
    theta_column: String,
    quality_filters: Vec<QualityFilter>,
    sensors: Vec<SensorMetadata>,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct QualityFilter {
    column: String,
    accepted_values: Vec<String>,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SensorMetadata {
    sensor_key: String,
    sensor_id: String,
    depth_cm: f64,
    x_m: f64,
    y_m: f64,
    condition: SensorCondition,
    metadata_evidence_ref: String,
    position_start_utc: Option<String>,
    position_end_utc: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
struct NormalizedObservation {
    timestamp_utc: String,
    volumetric_water_content_m3_m3: Option<f64>,
    status: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct NormalizedSensor {
    sensor_id: String,
    depth_cm: f64,
    condition: SensorCondition,
    observations: Vec<NormalizedObservation>,
}

#[derive(Clone, Debug, Serialize)]
struct SensorProvenance {
    sensor_key: String,
    sensor_id: String,
    depth_cm: f64,
    x_m: f64,
    y_m: f64,
    condition: SensorCondition,
    metadata_evidence_ref: String,
    position_start_utc: Option<String>,
    position_end_utc: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
struct NormalizedProfile {
    schema_version: &'static str,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    source_evidence_ref: String,
    sensors: Vec<NormalizedSensor>,
}

#[derive(Clone, Debug, Serialize)]
struct BinAudit {
    sensor_id: String,
    timestamp_utc: String,
    expected_raw_rows: usize,
    raw_rows_present: usize,
    quality_pass_rows: usize,
    quality_rejected_rows: usize,
    porosity_rejected_rows: usize,
    porosity_pass_rows: usize,
    maximum_profile_porosity_m3_m3: f64,
    absent_raw_rows: usize,
    good_fraction: f64,
    minimum_good_rows_required: usize,
    median_theta_m3_m3: Option<f64>,
    aggregation_status: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct AggregationReport {
    audit_version: &'static str,
    config_sha256: String,
    source_csv_sha256: String,
    raw_file_label: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    raw_resolution_seconds: u32,
    target_resolution_seconds: u32,
    minimum_good_fraction: f64,
    response_threshold_m3_m3: f64,
    maximum_profile_porosity_m3_m3: f64,
    maximum_profile_porosity_evidence_ref: String,
    timestamp_column: String,
    sensor_key_columns: Vec<String>,
    theta_column: String,
    quality_filters: Vec<QualityFilter>,
    selected_sensor_count: usize,
    sensor_metadata: Vec<SensorProvenance>,
    unknown_sensor_rows_ignored: usize,
    total_bin_count: usize,
    observed_bin_count: usize,
    missing_bin_count: usize,
    interpolated_bin_count: usize,
    remaining_missing_bin_count: usize,
    bins: Vec<BinAudit>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

#[derive(Clone, Debug)]
struct BinAccumulator {
    raw_rows_present: usize,
    quality_pass_rows: usize,
    quality_rejected_rows: usize,
    porosity_rejected_rows: usize,
    values: Vec<f64>,
}

impl BinAccumulator {
    fn new() -> Self {
        Self {
            raw_rows_present: 0,
            quality_pass_rows: 0,
            quality_rejected_rows: 0,
            porosity_rejected_rows: 0,
            values: Vec::new(),
        }
    }
}

fn parse_utc(value: &str, label: &str, resolution_seconds: i64) -> Result<DateTime<FixedOffset>, String> {
    let parsed = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if parsed.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp must use zero UTC offset"));
    }
    if parsed.timestamp().rem_euclid(resolution_seconds) != 0 || parsed.timestamp_subsec_nanos() != 0 {
        return Err(format!("{label}: timestamp is not aligned to the declared UTC time grid"));
    }
    Ok(parsed)
}

fn median(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let middle = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        Some((sorted[middle - 1] + sorted[middle]) / 2.0)
    } else {
        Some(sorted[middle])
    }
}

fn validate_config(config: &Config) -> Result<(DateTime<FixedOffset>, DateTime<FixedOffset>, Vec<DateTime<FixedOffset>>), String> {
    if config.schema_version != "0.1.0" {
        return Err(format!("unsupported config schema_version {:?}", config.schema_version));
    }
    for (value, label) in [
        (&config.study_id, "study_id"),
        (&config.site_id, "site_id"),
        (&config.plot_id, "plot_id"),
        (&config.profile_id, "profile_id"),
        (&config.event_id, "event_id"),
        (&config.maximum_profile_porosity_evidence_ref, "maximum_profile_porosity_evidence_ref"),
        (&config.raw_file_label, "raw_file_label"),
        (&config.source_evidence_ref, "source_evidence_ref"),
        (&config.timestamp_column, "timestamp_column"),
        (&config.theta_column, "theta_column"),
    ] {
        if value.trim().is_empty() {
            return Err(format!("{label} must not be blank"));
        }
    }
    if config.raw_resolution_seconds as i64 != RAW_RESOLUTION_SECONDS {
        return Err("v0.1 requires one-minute raw input (raw_resolution_seconds=60)".to_owned());
    }
    if !(MIN_TARGET_SECONDS..=MAX_TARGET_SECONDS).contains(&config.target_resolution_seconds)
        || config.target_resolution_seconds % 60 != 0
    {
        return Err("target_resolution_seconds must be a multiple of 60 between 300 and 1800".to_owned());
    }
    if !config.minimum_good_fraction.is_finite()
        || config.minimum_good_fraction <= 0.0
        || config.minimum_good_fraction > 1.0
    {
        return Err("minimum_good_fraction must be finite and within (0, 1]".to_owned());
    }
    if !config.response_threshold_m3_m3.is_finite()
        || config.response_threshold_m3_m3 <= 0.0
        || config.response_threshold_m3_m3 > 1.0
    {
        return Err("response_threshold_m3_m3 must be finite and in (0, 1]".to_owned());
    }
    if !config.maximum_profile_porosity_m3_m3.is_finite()
        || config.maximum_profile_porosity_m3_m3 <= 0.0
        || config.maximum_profile_porosity_m3_m3 > 1.0
    {
        return Err("maximum_profile_porosity_m3_m3 must be finite and within (0, 1]".to_owned());
    }
    if config.sensor_key_columns.is_empty() {
        return Err("sensor_key_columns must not be empty".to_owned());
    }
    if config.quality_filters.is_empty() {
        return Err("quality_filters must explicitly declare at least one QA rule".to_owned());
    }
    for (index, filter) in config.quality_filters.iter().enumerate() {
        if filter.column.trim().is_empty() || filter.accepted_values.is_empty() {
            return Err(format!("quality_filters[{index}] must declare a column and accepted values"));
        }
    }
    if config.sensors.is_empty() {
        return Err("sensor metadata roster must not be empty".to_owned());
    }

    let start = parse_utc(&config.event_start_utc, "event_start_utc", i64::from(config.target_resolution_seconds))?;
    let end = parse_utc(&config.observation_window_end_utc, "observation_window_end_utc", i64::from(config.target_resolution_seconds))?;
    if start >= end {
        return Err("observation_window_end_utc must be after event_start_utc".to_owned());
    }
    if (end.clone() - start.clone()).num_seconds() > MAX_EVENT_SECONDS {
        return Err("event observation window must not exceed six days in v0.1".to_owned());
    }

    let step = i64::from(config.target_resolution_seconds);
    let first_bin = start.clone() - Duration::seconds(step);
    let span = (end.clone() - first_bin.clone()).num_seconds();
    if span < 0 || span % step != 0 {
        return Err("event range is not a whole number of target aggregation bins".to_owned());
    }
    let count = (span / step + 1) as usize;
    let bin_starts: Vec<_> = (0..count)
        .map(|index| first_bin.clone() + Duration::seconds(step * index as i64))
        .collect();

    let mut sensor_keys = HashSet::new();
    let mut sensor_ids = HashSet::new();
    for (index, sensor) in config.sensors.iter().enumerate() {
        for (value, label) in [
            (&sensor.sensor_key, "sensor_key"),
            (&sensor.sensor_id, "sensor_id"),
            (&sensor.metadata_evidence_ref, "metadata_evidence_ref"),
        ] {
            if value.trim().is_empty() {
                return Err(format!("sensors[{index}].{label} must not be blank"));
            }
        }
        if !sensor_keys.insert(sensor.sensor_key.as_str()) {
            return Err(format!("duplicate sensor_key {:?}", sensor.sensor_key));
        }
        if !sensor_ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        for (value, label) in [
            (sensor.depth_cm, "depth_cm"),
            (sensor.x_m, "x_m"),
            (sensor.y_m, "y_m"),
        ] {
            if !value.is_finite() {
                return Err(format!("sensors[{index}].{label} must be finite"));
            }
        }
        if sensor.depth_cm <= 0.0 {
            return Err(format!("sensors[{index}].depth_cm must be greater than zero"));
        }
        if let Some(position_start_text) = &sensor.position_start_utc {
            let position_start = DateTime::parse_from_rfc3339(position_start_text)
                .map_err(|error| format!("sensors[{index}].position_start_utc is not RFC3339: {error}"))?;
            if position_start.offset().local_minus_utc() != 0 || start < position_start {
                return Err(format!("sensors[{index}] position is not valid at event start"));
            }
        }
        if let Some(position_end_text) = &sensor.position_end_utc {
            let position_end = DateTime::parse_from_rfc3339(position_end_text)
                .map_err(|error| format!("sensors[{index}].position_end_utc is not RFC3339: {error}"))?;
            if position_end.offset().local_minus_utc() != 0 || end > position_end {
                return Err(format!("sensors[{index}] position is not valid through event end"));
            }
        }
    }
    Ok((start, end, bin_starts))
}

fn evaluate(config: &Config, csv_bytes: &[u8], source_sha256: String, config_sha256: String) -> Result<(NormalizedProfile, NormalizedProfile, AggregationReport), String> {
    let (_event_start, _event_end, bin_starts) = validate_config(config)?;
    let raw_headers;
    let mut csv_reader = csv::ReaderBuilder::new()
        .flexible(false)
        .from_reader(csv_bytes);
    raw_headers = csv_reader.headers()
        .map_err(|error| format!("CSV header error: {error}"))?
        .clone();

    let header_map: HashMap<String, usize> = raw_headers.iter().enumerate()
        .map(|(index, name)| (name.to_owned(), index))
        .collect();
    if header_map.len() != raw_headers.len() {
        return Err("CSV contains duplicate column names".to_owned());
    }
    let mut required_columns = vec![config.timestamp_column.as_str(), config.theta_column.as_str()];
    required_columns.extend(config.sensor_key_columns.iter().map(String::as_str));
    required_columns.extend(config.quality_filters.iter().map(|filter| filter.column.as_str()));
    for column in required_columns {
        if !header_map.contains_key(column) {
            return Err(format!("required CSV column {column:?} is absent; verify the pinned NEON variables file and declared mapping"));
        }
    }

    let timestamp_index = header_map[config.timestamp_column.as_str()];
    let theta_index = header_map[config.theta_column.as_str()];
    let key_indices: Vec<usize> = config.sensor_key_columns.iter()
        .map(|column| header_map[column])
        .collect();
    let quality_indices: Vec<usize> = config.quality_filters.iter()
        .map(|filter| header_map[filter.column.as_str()])
        .collect();
    let sensor_map: HashMap<&str, usize> = config.sensors.iter().enumerate()
        .map(|(index, sensor)| (sensor.sensor_key.as_str(), index))
        .collect();

    let raw_step = RAW_RESOLUTION_SECONDS;
    let target_step = i64::from(config.target_resolution_seconds);
    let first_bin = bin_starts.first().expect("validated bins are non-empty").clone();
    let last_bin = bin_starts.last().expect("validated bins are non-empty").clone();
    let exclusive_end = last_bin.clone() + Duration::seconds(target_step);
    let bins_per_sensor = bin_starts.len();
    let mut accumulators: Vec<Vec<BinAccumulator>> = (0..config.sensors.len())
        .map(|_| (0..bins_per_sensor).map(|_| BinAccumulator::new()).collect())
        .collect();
    let mut seen: Vec<HashSet<i64>> = (0..config.sensors.len()).map(|_| HashSet::new()).collect();
    let mut unknown_sensor_rows = 0usize;

    for (row_index, row_result) in csv_reader.records().enumerate() {
        let row: StringRecord = row_result.map_err(|error| format!("CSV row {}: {error}", row_index + 2))?;
        let key_parts: Vec<&str> = key_indices.iter()
            .map(|index| row.get(*index).unwrap_or("").trim())
            .collect();
        if key_parts.iter().any(|part| part.is_empty()) {
            return Err(format!("CSV row {} has a blank sensor-key component", row_index + 2));
        }
        let sensor_key = key_parts.join("|");
        let Some(sensor_index) = sensor_map.get(sensor_key.as_str()).copied() else {
            unknown_sensor_rows += 1;
            continue;
        };

        let timestamp_text = row.get(timestamp_index).unwrap_or("");
        let timestamp = parse_utc(timestamp_text, &format!("CSV row {} timestamp", row_index + 2), raw_step)?;
        if timestamp < first_bin || timestamp >= exclusive_end {
            continue;
        }
        let offset_seconds = (timestamp.clone() - first_bin.clone()).num_seconds();
        if offset_seconds < 0 || offset_seconds % raw_step != 0 {
            return Err(format!("CSV row {} is not aligned to the one-minute raw grid", row_index + 2));
        }
        let bin_index = (offset_seconds / target_step) as usize;
        if bin_index >= bins_per_sensor {
            return Err(format!("CSV row {} maps outside expected target bins", row_index + 2));
        }
        if !seen[sensor_index].insert(timestamp.timestamp()) {
            return Err(format!(
                "duplicate one-minute row for sensor_key {:?} at {}",
                sensor_key,
                timestamp_text
            ));
        }

        let accumulator = &mut accumulators[sensor_index][bin_index];
        accumulator.raw_rows_present += 1;

        let mut quality_pass = true;
        for (filter, column_index) in config.quality_filters.iter().zip(&quality_indices) {
            let raw_value = row.get(*column_index).unwrap_or("").trim();
            if !filter.accepted_values.iter().any(|accepted| accepted == raw_value) {
                quality_pass = false;
                break;
            }
        }
        if !quality_pass {
            accumulator.quality_rejected_rows += 1;
            continue;
        }
        accumulator.quality_pass_rows += 1;

        let theta_text = row.get(theta_index).unwrap_or("").trim();
        let theta: f64 = theta_text.parse()
            .map_err(|error| format!("CSV row {} accepted theta value {theta_text:?} is not numeric: {error}", row_index + 2))?;
        if !theta.is_finite() || !(0.0..=1.0).contains(&theta) {
            return Err(format!("CSV row {} accepted theta must be finite and within [0, 1]", row_index + 2));
        }
        if theta > config.maximum_profile_porosity_m3_m3 {
            accumulator.porosity_rejected_rows += 1;
            continue;
        }
        accumulator.values.push(theta);
    }

    let expected_rows = (config.target_resolution_seconds / config.raw_resolution_seconds) as usize;
    let minimum_good_rows = (config.minimum_good_fraction * expected_rows as f64).ceil() as usize;
    let mut normalized_sensors = Vec::with_capacity(config.sensors.len());
    let mut bin_audits = Vec::with_capacity(config.sensors.len() * bins_per_sensor);
    let mut observed_bins = 0usize;
    let mut missing_bins = 0usize;

    for (sensor_index, sensor) in config.sensors.iter().enumerate() {
        let mut observations = Vec::with_capacity(bins_per_sensor);
        for (bin_index, bin_start) in bin_starts.iter().enumerate() {
            let accumulator = &accumulators[sensor_index][bin_index];
            let good_count = accumulator.values.len();
            let absent_count = expected_rows.saturating_sub(accumulator.raw_rows_present);
            let good_fraction = good_count as f64 / expected_rows as f64;
            let is_observed = good_count >= minimum_good_rows;
            let (theta, status, label) = if is_observed {
                observed_bins += 1;
                (median(&accumulator.values), "observed", "observed")
            } else {
                missing_bins += 1;
                (None, "missing", "insufficient_qa_or_porosity_coverage")
            };
            observations.push(NormalizedObservation {
                timestamp_utc: bin_start.to_rfc3339(),
                volumetric_water_content_m3_m3: theta,
                status,
            });
            bin_audits.push(BinAudit {
                sensor_id: sensor.sensor_id.clone(),
                timestamp_utc: bin_start.to_rfc3339(),
                expected_raw_rows: expected_rows,
                raw_rows_present: accumulator.raw_rows_present,
                quality_pass_rows: accumulator.quality_pass_rows,
                quality_rejected_rows: accumulator.quality_rejected_rows,
                porosity_rejected_rows: accumulator.porosity_rejected_rows,
                porosity_pass_rows: good_count,
                maximum_profile_porosity_m3_m3: config.maximum_profile_porosity_m3_m3,
                absent_raw_rows: absent_count,
                good_fraction,
                minimum_good_rows_required: minimum_good_rows,
                median_theta_m3_m3: theta,
                aggregation_status: label,
            });
        }
        normalized_sensors.push(NormalizedSensor {
            sensor_id: sensor.sensor_id.clone(),
            depth_cm: sensor.depth_cm,
            condition: sensor.condition,
            observations,
        });
    }

    let normalized = NormalizedProfile {
        schema_version: "0.1.0",
        study_id: config.study_id.clone(),
        site_id: config.site_id.clone(),
        plot_id: config.plot_id.clone(),
        profile_id: config.profile_id.clone(),
        event_id: config.event_id.clone(),
        event_start_utc: config.event_start_utc.clone(),
        observation_window_end_utc: config.observation_window_end_utc.clone(),
        time_resolution_seconds: config.target_resolution_seconds,
        source_evidence_ref: format!("{}; raw_file={}; see aggregation report for raw/config digests", config.source_evidence_ref, config.raw_file_label),
        sensors: normalized_sensors,
    };
    let mut paper_analysis_profile = normalized.clone();
    let interpolated_bin_count = interpolate_short_gaps(
        &mut paper_analysis_profile,
        config.target_resolution_seconds,
    );
    let remaining_missing_bin_count = paper_analysis_profile
        .sensors
        .iter()
        .flat_map(|sensor| &sensor.observations)
        .filter(|observation| observation.status == "missing")
        .count();

    let report = AggregationReport {
        audit_version: "0.1.0",
        config_sha256,
        source_csv_sha256: source_sha256,
        raw_file_label: config.raw_file_label.clone(),
        site_id: config.site_id.clone(),
        plot_id: config.plot_id.clone(),
        profile_id: config.profile_id.clone(),
        event_id: config.event_id.clone(),
        raw_resolution_seconds: config.raw_resolution_seconds,
        target_resolution_seconds: config.target_resolution_seconds,
        minimum_good_fraction: config.minimum_good_fraction,
        response_threshold_m3_m3: config.response_threshold_m3_m3,
        maximum_profile_porosity_m3_m3: config.maximum_profile_porosity_m3_m3,
        maximum_profile_porosity_evidence_ref: config.maximum_profile_porosity_evidence_ref.clone(),
        timestamp_column: config.timestamp_column.clone(),
        sensor_key_columns: config.sensor_key_columns.clone(),
        theta_column: config.theta_column.clone(),
        quality_filters: config.quality_filters.iter().map(|filter| QualityFilter {
            column: filter.column.clone(),
            accepted_values: filter.accepted_values.clone(),
        }).collect(),
        selected_sensor_count: config.sensors.len(),
        sensor_metadata: config.sensors.iter().map(|sensor| SensorProvenance {
            sensor_key: sensor.sensor_key.clone(),
            sensor_id: sensor.sensor_id.clone(),
            depth_cm: sensor.depth_cm,
            x_m: sensor.x_m,
            y_m: sensor.y_m,
            condition: sensor.condition,
            metadata_evidence_ref: sensor.metadata_evidence_ref.clone(),
            position_start_utc: sensor.position_start_utc.clone(),
            position_end_utc: sensor.position_end_utc.clone(),
        }).collect(),
        unknown_sensor_rows_ignored: unknown_sensor_rows,
        total_bin_count: bin_audits.len(),
        observed_bin_count: observed_bins,
        missing_bin_count: missing_bins,
        interpolated_bin_count,
        remaining_missing_bin_count,
        bins: bin_audits,
        interpretation_boundary: "quality-filtered median observation series only; QA and aggregation do not establish preferential flow or a unique pore pathway",
        warnings: vec![
            "The column mapping and accepted quality values are explicit configuration inputs; verify them against the pinned NEON variables file, ATBD, release and issue log.".to_owned(),
            "The input must be the one-minute soil-water table. Do not use 30-minute source data for this event reconstruction.".to_owned(),
            "After source QA filtering, observations above the declared maximum whole-profile porosity across all soil pits are separately rejected; preserve the porosity source/crosswalk and cutoff evidence.".to_owned(),
            "The normalized-profile output preserves the original missing bins. A second paper-analysis profile linearly interpolates only interior gaps whose missing-bin duration is strictly less than six hours; edge gaps and gaps of six hours or longer remain missing.".to_owned(),
            "Depth and x/y metadata are supplied separately in the sensor roster. For NEON DP1.00094.001 use swc_depthsV3.csv for corrected depth and a time-valid sensor-position join for coordinates.".to_owned(),
            "This tool does not establish the scientific validity of the quality filters or resolve calibration uncertainty; it faithfully applies the declared rule.".to_owned(),
        ],
    };
    Ok((normalized, paper_analysis_profile, report))
}

fn interpolate_short_gaps(profile: &mut NormalizedProfile, target_resolution_seconds: u32) -> usize {
    const MAX_GAP_SECONDS: u64 = 6 * 60 * 60;
    let step = u64::from(target_resolution_seconds);
    let mut total_interpolated = 0usize;

    for sensor in &mut profile.sensors {
        let observations = &mut sensor.observations;
        let mut left = 0usize;
        while left < observations.len() {
            if observations[left].status != "observed" {
                left += 1;
                continue;
            }
            let mut right = left + 1;
            while right < observations.len() && observations[right].status == "missing" {
                right += 1;
            }
            if right >= observations.len() || observations[right].status != "observed" {
                break;
            }

            let missing_count = right - left - 1;
            // Define gap duration conservatively as elapsed time between the
            // two observed endpoint timestamps, not just the missing-bin span.
            let endpoint_elapsed_seconds = ((right - left) as u64) * step;
            if missing_count > 0 && endpoint_elapsed_seconds < MAX_GAP_SECONDS {
                let left_theta = observations[left]
                    .volumetric_water_content_m3_m3
                    .expect("observed status must have a value");
                let right_theta = observations[right]
                    .volumetric_water_content_m3_m3
                    .expect("observed status must have a value");
                let span = (right - left) as f64;
                for index in left + 1..right {
                    let fraction = (index - left) as f64 / span;
                    observations[index].volumetric_water_content_m3_m3 =
                        Some(left_theta + fraction * (right_theta - left_theta));
                    observations[index].status = "linear_interpolated_gap_lt6h";
                    total_interpolated += 1;
                }
            }
            left = right;
        }
    }
    total_interpolated
}

fn write_new_file(path: &str, bytes: &[u8]) -> Result<(), io::Error> {
    let mut output = OpenOptions::new().write(true).create_new(true).open(path)
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create {path}: {error}")))?;
    output.write_all(bytes)?;
    output.write_all(b"\n")?;
    output.sync_all()
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 6 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: neon-swc-normalizer <config.json> <raw-one-minute.csv> <new-raw-profile.json> <new-paper-analysis-profile.json> <new-audit-report.json>",
        ).into());
    }
    let config_bytes = fs::read(&args[1])?;
    let source_bytes = fs::read(&args[2])?;
    let config_sha256 = format!("{:x}", Sha256::digest(&config_bytes));
    let source_sha256 = format!("{:x}", Sha256::digest(&source_bytes));
    let config: Config = serde_json::from_slice(&config_bytes)?;
    let (normalized, paper_analysis_profile, report) =
        evaluate(&config, &source_bytes, source_sha256, config_sha256)?;
    let normalized_bytes = serde_json::to_vec_pretty(&normalized)?;
    let paper_analysis_bytes = serde_json::to_vec_pretty(&paper_analysis_profile)?;
    let report_bytes = serde_json::to_vec_pretty(&report)?;
    write_new_file(&args[3], &normalized_bytes)?;
    write_new_file(&args[4], &paper_analysis_bytes)?;
    write_new_file(&args[5], &report_bytes)?;
    println!(
        "NEON soil-water normalization complete: sensors={}; observed_bins={}; original_missing_bins={}; interpolated_bins={}; remaining_missing_bins={}; unknown_sensor_rows_ignored={}; source_csv_sha256={}",
        report.selected_sensor_count,
        report.observed_bin_count,
        report.missing_bin_count,
        report.interpolated_bin_count,
        report.remaining_missing_bin_count,
        report.unknown_sensor_rows_ignored,
        report.source_csv_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("NEON soil-water normalizer failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(minutes: i64) -> String {
        let origin = Utc.with_ymd_and_hms(2026, 6, 1, 0, 0, 0).single().unwrap();
        (origin + Duration::minutes(minutes)).to_rfc3339()
    }

    fn config() -> Config {
        Config {
            schema_version: "0.1.0".to_owned(),
            study_id: "synthetic".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE1".to_owned(),
            event_id: "EVENT1".to_owned(),
            event_start_utc: stamp(10),
            observation_window_end_utc: stamp(30),
            raw_resolution_seconds: 60,
            target_resolution_seconds: 600,
            minimum_good_fraction: 0.8,
            response_threshold_m3_m3: 0.01,
            maximum_profile_porosity_m3_m3: 0.35,
            maximum_profile_porosity_evidence_ref: "synthetic whole-profile porosity fixture".to_owned(),
            raw_file_label: "SWS_1_minute synthetic fixture".to_owned(),
            source_evidence_ref: "synthetic one-minute CSV only".to_owned(),
            timestamp_column: "startDateTime".to_owned(),
            sensor_key_columns: vec!["horizontalPosition".to_owned(), "verticalPosition".to_owned()],
            theta_column: "VSWCMean".to_owned(),
            quality_filters: vec![QualityFilter {
                column: "VSWCFinalQF".to_owned(),
                accepted_values: vec!["0".to_owned()],
            }],
            sensors: vec![
                SensorMetadata {
                    sensor_key: "001|501".to_owned(),
                    sensor_id: "S1".to_owned(),
                    depth_cm: 6.0,
                    x_m: 0.0,
                    y_m: 0.0,
                    condition: SensorCondition::Usable,
                    metadata_evidence_ref: "corrected synthetic depth and sensor positions".to_owned(),
                    position_start_utc: Some("2026-01-01T00:00:00Z".to_owned()),
                    position_end_utc: None,
                },
                SensorMetadata {
                    sensor_key: "001|502".to_owned(),
                    sensor_id: "S2".to_owned(),
                    depth_cm: 16.0,
                    x_m: 0.0,
                    y_m: 0.0,
                    condition: SensorCondition::Usable,
                    metadata_evidence_ref: "corrected synthetic depth and sensor positions".to_owned(),
                    position_start_utc: Some("2026-01-01T00:00:00Z".to_owned()),
                    position_end_utc: None,
                },
            ],
        }
    }

    fn csv_fixture() -> String {
        let mut rows = vec!["horizontalPosition,verticalPosition,startDateTime,VSWCMean,VSWCFinalQF".to_owned()];
        for sensor in 0..2 {
            for minute in 0..40 {
                let baseline = minute < 10;
                let theta = if sensor == 0 {
                    if baseline { 0.20 } else { 0.24 }
                } else if baseline || minute < 30 {
                    0.18
                } else {
                    0.21
                };
                let qf = if sensor == 0 && minute == 12 { "1" } else { "0" };
                rows.push(format!(
                    "001,{}, {},{:.3},{}",
                    if sensor == 0 { "501" } else { "502" },
                    stamp(minute),
                    theta,
                    qf
                ).replace(", ", ","));
            }
        }
        rows.join("\n") + "\n"
    }

    #[test]
    fn aggregates_one_minute_rows_to_audited_ten_minute_medians() {
        let config = config();
        let csv = csv_fixture();
        let (normalized, _paper_analysis, report) =
            evaluate(&config, csv.as_bytes(), "a".repeat(64), "b".repeat(64)).unwrap();
        assert_eq!(normalized.sensors.len(), 2);
        assert_eq!(normalized.sensors[0].observations.len(), 4);
        assert_eq!(normalized.sensors[0].observations[0].status, "observed");
        assert_eq!(normalized.sensors[0].observations[0].volumetric_water_content_m3_m3, Some(0.20));
        assert_eq!(normalized.sensors[0].observations[1].volumetric_water_content_m3_m3, Some(0.24));
        assert_eq!(normalized.sensors[1].observations[1].volumetric_water_content_m3_m3, Some(0.18));
        assert_eq!(normalized.sensors[1].observations[3].volumetric_water_content_m3_m3, Some(0.21));
        assert_eq!(report.observed_bin_count, 8);
        assert_eq!(report.missing_bin_count, 0);
        assert_eq!(report.bins[1].quality_rejected_rows, 1);
        assert_eq!(report.bins[1].quality_pass_rows, 9);
        assert_eq!(report.bins[1].good_fraction, 0.9);
    }

    #[test]
    fn paper_analysis_profile_interpolates_short_interior_gap_and_preserves_raw() {
        let config = config();
        let mut rows: Vec<String> = csv_fixture().lines().map(str::to_owned).collect();
        for row in rows.iter_mut().skip(1) {
            let fields: Vec<&str> = row.split(',').collect();
            if fields.len() == 5
                && fields[0] == "001"
                && fields[1] == "501"
                && fields[2].starts_with("2026-06-01T00:1")
            {
                *row = format!("{},{},{},{},1", fields[0], fields[1], fields[2], fields[3]);
            }
        }
        let csv = rows.join("\n") + "\n";
        let (raw_profile, paper_profile, report) =
            evaluate(&config, csv.as_bytes(), "gap-source".to_owned(), "gap-config".to_owned()).unwrap();

        assert_eq!(raw_profile.sensors[0].observations[1].status, "missing");
        assert_eq!(raw_profile.sensors[0].observations[1].volumetric_water_content_m3_m3, None);
        assert_eq!(paper_profile.sensors[0].observations[1].status, "linear_interpolated_gap_lt6h");
        assert_eq!(paper_profile.sensors[0].observations[1].volumetric_water_content_m3_m3, Some(0.24));
        assert_eq!(report.missing_bin_count, 1);
        assert_eq!(report.interpolated_bin_count, 1);
        assert_eq!(report.remaining_missing_bin_count, 0);
    }

    #[test]
    fn interpolates_short_interior_gaps_without_overwriting_raw_profile() {
        let mut profile = NormalizedProfile {
            schema_version: "0.1.0",
            study_id: "synthetic".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE1".to_owned(),
            event_id: "EVENT1".to_owned(),
            event_start_utc: stamp(10),
            observation_window_end_utc: stamp(40),
            time_resolution_seconds: 600,
            source_evidence_ref: "synthetic".to_owned(),
            sensors: vec![NormalizedSensor {
                sensor_id: "S1".to_owned(),
                depth_cm: 6.0,
                condition: SensorCondition::Usable,
                observations: vec![
                    NormalizedObservation { timestamp_utc: stamp(0), volumetric_water_content_m3_m3: Some(0.20), status: "observed" },
                    NormalizedObservation { timestamp_utc: stamp(10), volumetric_water_content_m3_m3: None, status: "missing" },
                    NormalizedObservation { timestamp_utc: stamp(20), volumetric_water_content_m3_m3: None, status: "missing" },
                    NormalizedObservation { timestamp_utc: stamp(30), volumetric_water_content_m3_m3: Some(0.40), status: "observed" },
                ],
            }],
        };
        let raw = profile.clone();
        let count = interpolate_short_gaps(&mut profile, 600);
        assert_eq!(count, 2);
        assert_eq!(profile.sensors[0].observations[1].status, "linear_interpolated_gap_lt6h");
        assert!((profile.sensors[0].observations[1].volumetric_water_content_m3_m3.unwrap() - 0.2666666667).abs() < 1e-9);
        assert!((profile.sensors[0].observations[2].volumetric_water_content_m3_m3.unwrap() - 0.3333333333).abs() < 1e-9);
        assert_eq!(raw.sensors[0].observations[1].status, "missing");
        assert_eq!(raw.sensors[0].observations[1].volumetric_water_content_m3_m3, None);
    }

    #[test]
    fn does_not_interpolate_six_hour_or_longer_gaps_or_edge_gaps() {
        let make_observation = |index: usize, value: Option<f64>, status: &'static str| {
            NormalizedObservation {
                timestamp_utc: format!("bin-{index}"),
                volumetric_water_content_m3_m3: value,
                status,
            }
        };
        let mut observations = vec![make_observation(0, Some(0.2), "observed")];
        for index in 1..=35 {
            observations.push(make_observation(index, None, "missing"));
        }
        // Last observed endpoints are exactly six hours apart (36 × 10 min).
        observations.push(make_observation(36, Some(0.4), "observed"));
        observations.push(make_observation(37, None, "missing"));
        observations.push(make_observation(38, None, "missing"));
        let mut profile = NormalizedProfile {
            schema_version: "0.1.0",
            study_id: "synthetic".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE1".to_owned(),
            event_id: "EVENT1".to_owned(),
            event_start_utc: stamp(10),
            observation_window_end_utc: stamp(40),
            time_resolution_seconds: 600,
            source_evidence_ref: "synthetic".to_owned(),
            sensors: vec![NormalizedSensor {
                sensor_id: "S1".to_owned(),
                depth_cm: 6.0,
                condition: SensorCondition::Usable,
                observations,
            }],
        };
        assert_eq!(interpolate_short_gaps(&mut profile, 600), 0);
        assert_eq!(profile.sensors[0].observations[1].status, "missing");
        assert_eq!(profile.sensors[0].observations[37].status, "missing");
    }

    #[test]
    fn rejects_theta_above_maximum_profile_porosity_after_quality_filtering() {
        let mut config = config();
        config.maximum_profile_porosity_m3_m3 = 0.22;
        let csv = csv_fixture();
        let (normalized, _paper_analysis, report) =
            evaluate(&config, csv.as_bytes(), "porosity-source".to_owned(), "porosity-config".to_owned()).unwrap();
        assert_eq!(normalized.sensors[0].observations[1].status, "missing");
        assert_eq!(report.bins[1].quality_pass_rows, 9);
        assert_eq!(report.bins[1].porosity_rejected_rows, 9);
        assert_eq!(report.bins[1].porosity_pass_rows, 0);
        assert_eq!(report.bins[1].aggregation_status, "insufficient_qa_or_porosity_coverage");
        assert_eq!(report.maximum_profile_porosity_m3_m3, 0.22);
        assert_eq!(report.maximum_profile_porosity_evidence_ref, "synthetic whole-profile porosity fixture");
    }

    #[test]
    fn rejects_invalid_maximum_profile_porosity() {
        let mut config = config();
        config.maximum_profile_porosity_m3_m3 = 1.1;
        assert!(validate_config(&config).unwrap_err().contains("maximum_profile_porosity_m3_m3"));
    }

    #[test]
    fn bins_below_minimum_good_fraction_become_missing_not_zero() {
        let config = config();
        let mut rows: Vec<String> = csv_fixture().lines().map(str::to_owned).collect();
        for row in rows.iter_mut().skip(11).take(3) {
            let fields: Vec<&str> = row.split(',').collect();
            *row = format!("{},{},{},{},1", fields[0], fields[1], fields[2], fields[3]);
        }
        let csv = rows.join("\n") + "\n";
        let (normalized, _paper_analysis, report) =
            evaluate(&config, csv.as_bytes(), "c".repeat(64), "d".repeat(64)).unwrap();
        assert_eq!(normalized.sensors[0].observations[1].status, "missing");
        assert_eq!(normalized.sensors[0].observations[1].volumetric_water_content_m3_m3, None);
        assert_eq!(report.missing_bin_count, 1);
    }

    #[test]
    fn duplicate_sensor_timestamp_is_rejected() {
        let config = config();
        let mut csv = csv_fixture();
        let row = csv.lines().nth(1).unwrap().to_owned();
        csv.push_str(&row);
        csv.push('\n');
        assert!(evaluate(&config, csv.as_bytes(), "e".repeat(64), "f".repeat(64))
            .unwrap_err().contains("duplicate one-minute row"));
    }

    #[test]
    fn missing_quality_column_is_rejected_instead_of_assuming_pass() {
        let config = config();
        let csv = "horizontalPosition,verticalPosition,startDateTime,VSWCMean\n001,501,2026-06-01T00:00:00Z,0.2\n";
        assert!(evaluate(&config, csv.as_bytes(), "a".repeat(64), "b".repeat(64))
            .unwrap_err().contains("required CSV column"));
    }
}

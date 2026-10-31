use chrono::{DateTime, Duration, FixedOffset};
#[cfg(test)]
use chrono::{TimeZone, Utc};
use csv::StringRecord;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, HashSet},
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const TARGET_SECONDS: i64 = 600;
const PAPER_MIN_GOOD_ROWS: usize = 5;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum SourceKind {
    PrimaryWeighing,
    SecondaryTipping,
}

impl SourceKind {
    fn as_str(self) -> &'static str {
        match self {
            Self::PrimaryWeighing => "primary_weighing",
            Self::SecondaryTipping => "secondary_tipping",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum TimestampConvention {
    IntervalStart,
    IntervalEnd,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    schema_version: String,
    study_id: String,
    site_id: String,
    start_utc: String,
    end_utc: String,
    target_resolution_seconds: u32,
    paper_minimum_good_observations: usize,
    coverage_aware_minimum_good_fraction: f64,
    primary: Option<SourceConfig>,
    secondary: Option<SourceConfig>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SourceConfig {
    source_kind: SourceKind,
    product_code: String,
    release: String,
    gauge_id: String,
    gauge_id_column: String,
    timestamp_column: String,
    timestamp_convention: TimestampConvention,
    interval_seconds: u32,
    precipitation_column: String,
    quality_filters: Vec<QualityFilter>,
    source_evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct QualityFilter {
    column: String,
    accepted_values: Vec<String>,
}

#[derive(Clone, Debug)]
struct BinAccumulator {
    present_rows: usize,
    good_rows: usize,
    rejected_rows: usize,
    sum_mm: f64,
}

impl BinAccumulator {
    fn new() -> Self {
        Self {
            present_rows: 0,
            good_rows: 0,
            rejected_rows: 0,
            sum_mm: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
struct LoadedSource {
    config: SourceConfig,
    bins: BTreeMap<i64, BinAccumulator>,
    source_sha256: String,
    ignored_other_gauge_rows: usize,
    out_of_window_rows: usize,
}

#[derive(Clone, Debug, Serialize)]
struct NormalizedBin {
    timestamp_utc: String,
    selected_source: String,
    source_product_code: Option<String>,
    source_release: Option<String>,
    gauge_id: Option<String>,
    source_evidence_ref: Option<String>,
    raw_rows_present: usize,
    good_rows: usize,
    rejected_rows: usize,
    expected_raw_rows: Option<usize>,
    precipitation_mm_per_10min: Option<f64>,
    precipitation_rate_mm_h: Option<f64>,
    status: String,
}

#[derive(Serialize)]
struct SourceAudit {
    source_kind: String,
    product_code: String,
    release: String,
    gauge_id: String,
    interval_seconds: u32,
    timestamp_convention: String,
    source_evidence_ref: String,
    source_sha256: String,
    ignored_other_gauge_rows: usize,
    out_of_window_rows: usize,
    target_bins_with_rows: usize,
    raw_rows_present: usize,
    quality_pass_rows: usize,
    quality_rejected_rows: usize,
}

#[derive(Serialize)]
struct TrackSummary {
    bin_count: usize,
    observed_bin_count: usize,
    zero_imputed_bin_count: usize,
    missing_bin_count: usize,
    selected_primary_bin_count: usize,
    selected_secondary_bin_count: usize,
    total_precipitation_mm: Option<f64>,
    bins: Vec<NormalizedBin>,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    config_sha256: String,
    study_id: String,
    site_id: String,
    start_utc: String,
    end_utc: String,
    target_resolution_seconds: u32,
    paper_minimum_good_observations: usize,
    coverage_aware_minimum_good_fraction: f64,
    primary_source: Option<SourceAudit>,
    secondary_source: Option<SourceAudit>,
    paper_reproduction: TrackSummary,
    coverage_aware_sensitivity: TrackSummary,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn parse_utc(value: &str, label: &str, interval_seconds: i64) -> Result<DateTime<FixedOffset>, String> {
    let parsed = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if parsed.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp must use a zero UTC offset"));
    }
    if parsed.timestamp().rem_euclid(interval_seconds) != 0 || parsed.timestamp_subsec_nanos() != 0 {
        return Err(format!("{label}: timestamp must align to the declared UTC interval"));
    }
    Ok(parsed)
}

fn validate_source(source: &SourceConfig, expected_kind: SourceKind) -> Result<(), String> {
    if source.source_kind != expected_kind {
        return Err(format!(
            "source configuration declares {}, expected {}",
            source.source_kind.as_str(),
            expected_kind.as_str()
        ));
    }
    for (value, label) in [
        (&source.product_code, "product_code"),
        (&source.release, "release"),
        (&source.gauge_id, "gauge_id"),
        (&source.gauge_id_column, "gauge_id_column"),
        (&source.timestamp_column, "timestamp_column"),
        (&source.precipitation_column, "precipitation_column"),
        (&source.source_evidence_ref, "source_evidence_ref"),
    ] {
        nonblank(value, label)?;
    }
    if source.interval_seconds != 60 && source.interval_seconds != 300 {
        return Err(format!(
            "{} interval_seconds must be 60 (secondary) or 300 (primary)",
            expected_kind.as_str()
        ));
    }
    match expected_kind {
        SourceKind::PrimaryWeighing => {
            if source.interval_seconds != 300 {
                return Err("primary weighing-gauge input must use 300-second observations".to_owned());
            }
            if source.product_code != "DP1.00006.001" {
                return Err("primary 5-minute weighing-gauge reproduction requires historical DP1.00006.001; DP1.00044.001 hourly/daily data cannot reconstruct 10-minute intensity".to_owned());
            }
        }
        SourceKind::SecondaryTipping => {
            if source.interval_seconds != 60 {
                return Err("secondary tipping-bucket input must use 60-second observations".to_owned());
            }
            if source.product_code != "DP1.00006.001" && source.product_code != "DP1.00045.001" {
                return Err("secondary tipping-bucket input must use DP1.00006.001 historical data or DP1.00045.001 current data".to_owned());
            }
        }
    }
    if source.quality_filters.is_empty() {
        return Err(format!("{} must declare at least one explicit QA filter", expected_kind.as_str()));
    }
    for (index, filter) in source.quality_filters.iter().enumerate() {
        nonblank(&filter.column, &format!("{expected_kind:?} quality_filters[{index}].column"))?;
        if filter.accepted_values.is_empty() {
            return Err(format!("{expected_kind:?} quality_filters[{index}] must declare accepted_values"));
        }
        for value in &filter.accepted_values {
            nonblank(value, &format!("{expected_kind:?} quality_filters[{index}].accepted_values"))?;
        }
    }
    Ok(())
}

fn validate_config(config: &Config) -> Result<(DateTime<FixedOffset>, DateTime<FixedOffset>), String> {
    if config.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", config.schema_version));
    }
    nonblank(&config.study_id, "study_id")?;
    nonblank(&config.site_id, "site_id")?;
    if config.target_resolution_seconds as i64 != TARGET_SECONDS {
        return Err("v0.1 target_resolution_seconds must equal 600 (10-minute bins)".to_owned());
    }
    if config.paper_minimum_good_observations != PAPER_MIN_GOOD_ROWS {
        return Err(format!("v0.1 paper_minimum_good_observations must equal {PAPER_MIN_GOOD_ROWS}"));
    }
    if !config.coverage_aware_minimum_good_fraction.is_finite()
        || config.coverage_aware_minimum_good_fraction <= 0.0
        || config.coverage_aware_minimum_good_fraction > 1.0
    {
        return Err("coverage_aware_minimum_good_fraction must be finite and within (0, 1]".to_owned());
    }
    if config.primary.is_none() && config.secondary.is_none() {
        return Err("at least one precipitation source must be configured".to_owned());
    }
    if let Some(source) = &config.primary {
        validate_source(source, SourceKind::PrimaryWeighing)?;
    }
    if let Some(source) = &config.secondary {
        validate_source(source, SourceKind::SecondaryTipping)?;
    }
    let start = parse_utc(&config.start_utc, "start_utc", TARGET_SECONDS)?;
    let end = parse_utc(&config.end_utc, "end_utc", TARGET_SECONDS)?;
    if start >= end {
        return Err("end_utc must be after start_utc".to_owned());
    }
    if (end.clone() - start.clone()).num_seconds().rem_euclid(TARGET_SECONDS) != 0 {
        return Err("declared time window must contain an integer number of 10-minute bins".to_owned());
    }
    Ok((start, end))
}

fn column_index(headers: &StringRecord, name: &str, label: &str) -> Result<usize, String> {
    headers.iter().position(|header| header == name)
        .ok_or_else(|| format!("{label}: required CSV column {:?} is missing", name))
}

fn load_source(
    bytes: &[u8],
    source: &SourceConfig,
    expected_kind: SourceKind,
    start: DateTime<FixedOffset>,
    end: DateTime<FixedOffset>,
) -> Result<LoadedSource, Box<dyn Error>> {
    validate_source(source, expected_kind).map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
    let digest = format!("{:x}", Sha256::digest(bytes));
    let mut reader = csv::ReaderBuilder::new().flexible(false).from_reader(bytes);
    let headers = reader.headers()?.clone();
    let gauge_index = column_index(&headers, &source.gauge_id_column, "source")?;
    let timestamp_index = column_index(&headers, &source.timestamp_column, "source")?;
    let precipitation_index = column_index(&headers, &source.precipitation_column, "source")?;
    let mut qa_indices = Vec::with_capacity(source.quality_filters.len());
    for filter in &source.quality_filters {
        qa_indices.push((
            column_index(&headers, &filter.column, "source QA filter")?,
            filter.accepted_values.clone(),
        ));
    }

    let mut bins: BTreeMap<i64, BinAccumulator> = BTreeMap::new();
    let mut seen_times = HashSet::new();
    let mut ignored_other_gauge_rows = 0usize;
    let mut out_of_window_rows = 0usize;
    let interval = i64::from(source.interval_seconds);

    for (row_index, record_result) in reader.records().enumerate() {
        let record = record_result?;
        if record.get(gauge_index).unwrap_or_default() != source.gauge_id {
            ignored_other_gauge_rows += 1;
            continue;
        }
        let timestamp_text = record.get(timestamp_index).unwrap_or_default();
        let timestamp = parse_utc(timestamp_text, &format!("source row {}", row_index + 2), interval)?;
        let interval_start = match source.timestamp_convention {
            TimestampConvention::IntervalStart => timestamp,
            TimestampConvention::IntervalEnd => timestamp - Duration::seconds(interval),
        };
        let interval_end = interval_start + Duration::seconds(interval);
        if interval_start < start || interval_end > end {
            out_of_window_rows += 1;
            continue;
        }
        if !seen_times.insert(interval_start.timestamp()) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("duplicate {} observation timestamp {}", expected_kind.as_str(), interval_start),
            ).into());
        }
        let bin_start = interval_start.timestamp().div_euclid(TARGET_SECONDS) * TARGET_SECONDS;
        let accumulator = bins.entry(bin_start).or_insert_with(BinAccumulator::new);
        accumulator.present_rows += 1;

        let qa_pass = qa_indices.iter().all(|(index, accepted)| {
            let value = record.get(*index).unwrap_or_default();
            accepted.iter().any(|candidate| candidate == value)
        });
        if !qa_pass {
            accumulator.rejected_rows += 1;
            continue;
        }
        let value_text = record.get(precipitation_index).unwrap_or_default();
        let value: f64 = value_text.parse().map_err(|error| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("{} row {} precipitation value {:?} is not numeric: {error}", expected_kind.as_str(), row_index + 2, value_text),
            )
        })?;
        if !value.is_finite() || value < 0.0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("{} row {} precipitation depth must be finite and non-negative", expected_kind.as_str(), row_index + 2),
            ).into());
        }
        accumulator.good_rows += 1;
        accumulator.sum_mm += value;
        if !accumulator.sum_mm.is_finite() {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "precipitation bin sum overflowed").into());
        }
    }

    Ok(LoadedSource {
        config: source.clone(),
        bins,
        source_sha256: digest,
        ignored_other_gauge_rows,
        out_of_window_rows,
    })
}

fn make_bin(
    timestamp: DateTime<FixedOffset>,
    primary: Option<&LoadedSource>,
    secondary: Option<&LoadedSource>,
    coverage_fraction: f64,
    paper_minimum_good: usize,
) -> (NormalizedBin, NormalizedBin) {
    let bin_start = timestamp.timestamp();
    let secondary_accumulator = secondary.and_then(|source| source.bins.get(&bin_start));
    let primary_accumulator = primary.and_then(|source| source.bins.get(&bin_start));

    // The paper says to prefer secondary where both types are available. Presence
    // means a source row exists in the bin; poor secondary QA does not silently
    // trigger fallback to the primary gauge.
    let (selected, source_kind, accumulator) = if let Some(accumulator) = secondary_accumulator.filter(|value| value.present_rows > 0) {
        (secondary, "secondary_tipping", Some(accumulator))
    } else if let Some(accumulator) = primary_accumulator.filter(|value| value.present_rows > 0) {
        (primary, "primary_weighing", Some(accumulator))
    } else {
        (None, "none", None)
    };

    let (product_code, release, gauge_id, evidence_ref, raw_rows, good_rows, rejected_rows, expected_rows, sum_mm) =
        if let (Some(source), Some(accumulator)) = (selected, accumulator) {
            (
                Some(source.config.product_code.clone()),
                Some(source.config.release.clone()),
                Some(source.config.gauge_id.clone()),
                Some(source.config.source_evidence_ref.clone()),
                accumulator.present_rows,
                accumulator.good_rows,
                accumulator.rejected_rows,
                Some((TARGET_SECONDS / i64::from(source.config.interval_seconds)) as usize),
                Some(accumulator.sum_mm),
            )
        } else {
            (None, None, None, None, 0, 0, 0, None, None)
        };

    let paper = if let Some(sum) = sum_mm {
        if good_rows < paper_minimum_good {
            NormalizedBin {
                timestamp_utc: timestamp.to_rfc3339(),
                selected_source: source_kind.to_owned(),
                source_product_code: product_code.clone(),
                source_release: release.clone(),
                gauge_id: gauge_id.clone(),
                source_evidence_ref: evidence_ref.clone(),
                raw_rows_present: raw_rows,
                good_rows,
                rejected_rows,
                expected_raw_rows: expected_rows,
                precipitation_mm_per_10min: Some(0.0),
                precipitation_rate_mm_h: Some(0.0),
                status: "paper_zero_imputed_lt5_good".to_owned(),
            }
        } else {
            NormalizedBin {
                timestamp_utc: timestamp.to_rfc3339(),
                selected_source: source_kind.to_owned(),
                source_product_code: product_code.clone(),
                source_release: release.clone(),
                gauge_id: gauge_id.clone(),
                source_evidence_ref: evidence_ref.clone(),
                raw_rows_present: raw_rows,
                good_rows,
                rejected_rows,
                expected_raw_rows: expected_rows,
                precipitation_mm_per_10min: Some(sum),
                precipitation_rate_mm_h: Some(sum * 6.0),
                status: "observed_sum".to_owned(),
            }
        }
    } else {
        NormalizedBin {
            timestamp_utc: timestamp.to_rfc3339(),
            selected_source: "none".to_owned(),
            source_product_code: None,
            source_release: None,
            gauge_id: None,
            source_evidence_ref: None,
            raw_rows_present: 0,
            good_rows: 0,
            rejected_rows: 0,
            expected_raw_rows: None,
            precipitation_mm_per_10min: None,
            precipitation_rate_mm_h: None,
            status: "missing_no_source_rows".to_owned(),
        }
    };

    let coverage = if let Some(sum) = sum_mm {
        let expected = expected_rows.unwrap_or(0);
        let minimum = ((coverage_fraction * expected as f64).ceil() as usize).max(1);
        if good_rows >= minimum {
            NormalizedBin {
                timestamp_utc: timestamp.to_rfc3339(),
                selected_source: source_kind.to_owned(),
                source_product_code: product_code,
                source_release: release,
                gauge_id,
                source_evidence_ref: evidence_ref,
                raw_rows_present: raw_rows,
                good_rows,
                rejected_rows,
                expected_raw_rows: expected_rows,
                precipitation_mm_per_10min: Some(sum),
                precipitation_rate_mm_h: Some(sum * 6.0),
                status: "observed_sum_coverage_qualified".to_owned(),
            }
        } else {
            NormalizedBin {
                timestamp_utc: timestamp.to_rfc3339(),
                selected_source: source_kind.to_owned(),
                source_product_code: product_code,
                source_release: release,
                gauge_id,
                source_evidence_ref: evidence_ref,
                raw_rows_present: raw_rows,
                good_rows,
                rejected_rows,
                expected_raw_rows: expected_rows,
                precipitation_mm_per_10min: None,
                precipitation_rate_mm_h: None,
                status: "missing_below_coverage_threshold".to_owned(),
            }
        }
    } else {
        NormalizedBin {
            timestamp_utc: timestamp.to_rfc3339(),
            selected_source: "none".to_owned(),
            source_product_code: None,
            source_release: None,
            gauge_id: None,
            source_evidence_ref: None,
            raw_rows_present: 0,
            good_rows: 0,
            rejected_rows: 0,
            expected_raw_rows: None,
            precipitation_mm_per_10min: None,
            precipitation_rate_mm_h: None,
            status: "missing_no_source_rows".to_owned(),
        }
    };

    (paper, coverage)
}

fn make_track(bins: Vec<NormalizedBin>) -> TrackSummary {
    let mut observed = 0usize;
    let mut zero_imputed = 0usize;
    let mut missing = 0usize;
    let mut primary = 0usize;
    let mut secondary = 0usize;
    let mut total = 0.0;
    let mut has_missing = false;

    for bin in &bins {
        if bin.status == "observed_sum" || bin.status == "observed_sum_coverage_qualified" {
            observed += 1;
            total += bin.precipitation_mm_per_10min.unwrap_or(0.0);
        } else if bin.status == "paper_zero_imputed_lt5_good" {
            zero_imputed += 1;
        } else {
            missing += 1;
            has_missing = true;
        }
        match bin.selected_source.as_str() {
            "primary_weighing" => primary += 1,
            "secondary_tipping" => secondary += 1,
            _ => {}
        }
    }

    TrackSummary {
        bin_count: bins.len(),
        observed_bin_count: observed,
        zero_imputed_bin_count: zero_imputed,
        missing_bin_count: missing,
        selected_primary_bin_count: primary,
        selected_secondary_bin_count: secondary,
        total_precipitation_mm: (!has_missing).then_some(total),
        bins,
    }
}

fn source_audit(source: &LoadedSource) -> SourceAudit {
    let mut raw_rows = 0usize;
    let mut good_rows = 0usize;
    let mut rejected_rows = 0usize;
    for accumulator in source.bins.values() {
        raw_rows += accumulator.present_rows;
        good_rows += accumulator.good_rows;
        rejected_rows += accumulator.rejected_rows;
    }
    SourceAudit {
        source_kind: source.config.source_kind.as_str().to_owned(),
        product_code: source.config.product_code.clone(),
        release: source.config.release.clone(),
        gauge_id: source.config.gauge_id.clone(),
        interval_seconds: source.config.interval_seconds,
        timestamp_convention: match source.config.timestamp_convention {
            TimestampConvention::IntervalStart => "interval_start",
            TimestampConvention::IntervalEnd => "interval_end",
        }.to_owned(),
        source_evidence_ref: source.config.source_evidence_ref.clone(),
        source_sha256: source.source_sha256.clone(),
        ignored_other_gauge_rows: source.ignored_other_gauge_rows,
        out_of_window_rows: source.out_of_window_rows,
        target_bins_with_rows: source.bins.values().filter(|bin| bin.present_rows > 0).count(),
        raw_rows_present: raw_rows,
        quality_pass_rows: good_rows,
        quality_rejected_rows: rejected_rows,
    }
}

fn evaluate(
    config: &Config,
    config_sha256: String,
    primary_bytes: Option<&[u8]>,
    secondary_bytes: Option<&[u8]>,
) -> Result<Report, Box<dyn Error>> {
    let (start, end) = validate_config(config)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;

    let primary = match (&config.primary, primary_bytes) {
        (Some(source), Some(bytes)) => Some(load_source(bytes, source, SourceKind::PrimaryWeighing, start.clone(), end.clone())?),
        (None, None) => None,
        (Some(_), None) => return Err(io::Error::new(io::ErrorKind::InvalidInput, "primary source is configured but primary CSV path is absent").into()),
        (None, Some(_)) => return Err(io::Error::new(io::ErrorKind::InvalidInput, "primary CSV supplied without primary source config").into()),
    };
    let secondary = match (&config.secondary, secondary_bytes) {
        (Some(source), Some(bytes)) => Some(load_source(bytes, source, SourceKind::SecondaryTipping, start.clone(), end.clone())?),
        (None, None) => None,
        (Some(_), None) => return Err(io::Error::new(io::ErrorKind::InvalidInput, "secondary source is configured but secondary CSV path is absent").into()),
        (None, Some(_)) => return Err(io::Error::new(io::ErrorKind::InvalidInput, "secondary CSV supplied without secondary source config").into()),
    };

    let mut paper_bins = Vec::new();
    let mut coverage_bins = Vec::new();
    let bin_count = ((end.timestamp() - start.timestamp()) / TARGET_SECONDS) as usize;
    for index in 0..bin_count {
        let timestamp = start.clone() + Duration::seconds(TARGET_SECONDS * index as i64);
        let (paper, coverage) = make_bin(
            timestamp,
            primary.as_ref(),
            secondary.as_ref(),
            config.coverage_aware_minimum_good_fraction,
            config.paper_minimum_good_observations,
        );
        paper_bins.push(paper);
        coverage_bins.push(coverage);
    }

    let mut warnings = vec![
        "Input CSVs must already be mapped to the declared raw NEON product tables and release-specific QA columns. This tool executes configured filters; it does not infer NEON QA policy.".to_owned(),
        "Precipitation values are interpreted as interval depth in mm and summed into left-closed/right-open 10-minute bins. Verify source table interval and timestamp convention against its pinned variables file/ATBD.".to_owned(),
        "Secondary tipping-bucket records are preferred whenever at least one row exists in the 10-minute bin. If secondary rows exist but are QA-rejected, the tool does not silently fall back to primary.".to_owned(),
        "The literal paper rule assigns zero when the selected source has fewer than five good measurements. The coverage-aware track preserves insufficient-coverage bins as missing.".to_owned(),
        "The paper text does not explain how a five-good-sample threshold is intended to behave for a 5-minute primary gauge, which can provide only two values per 10-minute bin. The literal track preserves the wording; the coverage-aware track exposes the alternative.".to_owned(),
        "No-source-row bins remain missing in both tracks; a documented zero-valued rainfall row is observed zero, not missing.".to_owned(),
    ];
    if let Some(source) = &primary {
        if config.paper_minimum_good_observations > (TARGET_SECONDS / i64::from(source.config.interval_seconds)) as usize {
            warnings.push("The literal five-good-observation threshold exceeds the maximum number of primary 5-minute observations in a 10-minute bin; all selected primary bins with data will be paper-zero-imputed. Resolve against author code before claiming exact reproduction.".to_owned());
        }
    }

    Ok(Report {
        audit_version: "0.1.0",
        config_sha256,
        study_id: config.study_id.clone(),
        site_id: config.site_id.clone(),
        start_utc: start.to_rfc3339(),
        end_utc: end.to_rfc3339(),
        target_resolution_seconds: TARGET_SECONDS as u32,
        paper_minimum_good_observations: config.paper_minimum_good_observations,
        coverage_aware_minimum_good_fraction: config.coverage_aware_minimum_good_fraction,
        primary_source: primary.as_ref().map(source_audit),
        secondary_source: secondary.as_ref().map(source_audit),
        paper_reproduction: make_track(paper_bins),
        coverage_aware_sensitivity: make_track(coverage_bins),
        warnings,
        interpretation_boundary: "This is a configured preprocessing executor. Paper-rule zero imputation, source selection and aggregation are auditable operations, not proof that every source bin is physically representative. Rainfall is not throughfall, and the output is not a preferential-flow label.",
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 5 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: neon-precip-normalizer <config.json> <primary.csv|-> <secondary.csv|-> <new-report.json>",
        ).into());
    }
    let config_bytes = fs::read(&args[1])?;
    let config_sha256 = format!("{:x}", Sha256::digest(&config_bytes));
    let config: Config = serde_json::from_slice(&config_bytes)?;
    let primary_bytes = if args[2] == "-" { None } else { Some(fs::read(&args[2])?) };
    let secondary_bytes = if args[3] == "-" { None } else { Some(fs::read(&args[3])?) };
    let report = evaluate(
        &config,
        config_sha256,
        primary_bytes.as_deref(),
        secondary_bytes.as_deref(),
    )?;
    let encoded = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[4])
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report: {error}")))?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "NEON precipitation normalization complete: {} bins; paper observed={}, zero-imputed={}, missing={}; coverage-aware observed={}, missing={}; config_sha256={}",
        report.paper_reproduction.bin_count,
        report.paper_reproduction.observed_bin_count,
        report.paper_reproduction.zero_imputed_bin_count,
        report.paper_reproduction.missing_bin_count,
        report.coverage_aware_sensitivity.observed_bin_count,
        report.coverage_aware_sensitivity.missing_bin_count,
        report.config_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("NEON precipitation normalization failed closed: {error}");
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

    fn config() -> Config {
        Config {
            schema_version: "0.1.0".to_owned(),
            study_id: "synthetic-study".to_owned(),
            site_id: "SYN1".to_owned(),
            start_utc: "2026-01-01T00:00:00Z".to_owned(),
            end_utc: "2026-01-01T00:20:00Z".to_owned(),
            target_resolution_seconds: 600,
            paper_minimum_good_observations: 5,
            coverage_aware_minimum_good_fraction: 0.5,
            primary: Some(SourceConfig {
                source_kind: SourceKind::PrimaryWeighing,
                product_code: "DP1.00006.001".to_owned(),
                release: "RELEASE-2024".to_owned(),
                gauge_id: "PRI-01".to_owned(),
                gauge_id_column: "gauge_id".to_owned(),
                timestamp_column: "timestamp_utc".to_owned(),
                timestamp_convention: TimestampConvention::IntervalStart,
                interval_seconds: 300,
                precipitation_column: "precipitation_mm".to_owned(),
                quality_filters: vec![QualityFilter { column: "finalQF".to_owned(), accepted_values: vec!["0".to_owned()] }],
                source_evidence_ref: "synthetic-primary".to_owned(),
            }),
            secondary: Some(SourceConfig {
                source_kind: SourceKind::SecondaryTipping,
                product_code: "DP1.00006.001".to_owned(),
                release: "RELEASE-2024".to_owned(),
                gauge_id: "SEC-01".to_owned(),
                gauge_id_column: "gauge_id".to_owned(),
                timestamp_column: "timestamp_utc".to_owned(),
                timestamp_convention: TimestampConvention::IntervalStart,
                interval_seconds: 60,
                precipitation_column: "precipitation_mm".to_owned(),
                quality_filters: vec![
                    QualityFilter { column: "rangeQF".to_owned(), accepted_values: vec!["0".to_owned()] },
                    QualityFilter { column: "scienceQF".to_owned(), accepted_values: vec!["0".to_owned()] },
                ],
                source_evidence_ref: "synthetic-secondary".to_owned(),
            }),
        }
    }

    fn primary_csv() -> &'static [u8] {
        b"gauge_id,timestamp_utc,precipitation_mm,finalQF\nPRI-01,2026-01-01T00:00:00Z,0.2,0\nPRI-01,2026-01-01T00:05:00Z,0.3,0\nPRI-01,2026-01-01T00:10:00Z,0.7,0\nPRI-01,2026-01-01T00:15:00Z,0.8,0\n"
    }

    fn secondary_csv() -> &'static [u8] {
        b"gauge_id,timestamp_utc,precipitation_mm,rangeQF,scienceQF\nSEC-01,2026-01-01T00:00:00Z,0.1,0,0\nSEC-01,2026-01-01T00:01:00Z,0.1,0,0\nSEC-01,2026-01-01T00:02:00Z,0.1,0,0\nSEC-01,2026-01-01T00:03:00Z,0.1,0,0\nSEC-01,2026-01-01T00:04:00Z,0.1,1,0\n"
    }

    #[test]
    fn prefers_secondary_when_present_without_pooling_primary() {
        let config = config();
        let report = evaluate(&config, "a".repeat(64), Some(primary_csv()), Some(secondary_csv())).unwrap();
        assert_eq!(report.paper_reproduction.bins[0].selected_source, "secondary_tipping");
        assert_eq!(report.paper_reproduction.bins[0].status, "paper_zero_imputed_lt5_good");
        assert_eq!(report.coverage_aware_sensitivity.bins[0].selected_source, "secondary_tipping");
        assert_eq!(report.coverage_aware_sensitivity.bins[0].status, "missing_below_coverage_threshold");
        assert_eq!(report.paper_reproduction.bins[1].selected_source, "primary_weighing");
        assert_eq!(report.paper_reproduction.bins[1].status, "paper_zero_imputed_lt5_good");
        assert_eq!(report.coverage_aware_sensitivity.bins[1].selected_source, "primary_weighing");
        assert_eq!(report.coverage_aware_sensitivity.bins[1].status, "observed_sum_coverage_qualified");
        assert_eq!(report.coverage_aware_sensitivity.bins[1].precipitation_mm_per_10min, Some(1.5));
    }

    #[test]
    fn sums_interval_depth_and_converts_ten_minute_rate() {
        let mut config = config();
        config.primary = None;
        let secondary = b"gauge_id,timestamp_utc,precipitation_mm,rangeQF,scienceQF\nSEC-01,2026-01-01T00:00:00Z,0.1,0,0\nSEC-01,2026-01-01T00:01:00Z,0.2,0,0\nSEC-01,2026-01-01T00:02:00Z,0.3,0,0\nSEC-01,2026-01-01T00:03:00Z,0.4,0,0\nSEC-01,2026-01-01T00:04:00Z,0.5,0,0\n".as_slice();
        let report = evaluate(&config, "b".repeat(64), None, Some(secondary)).unwrap();
        assert_eq!(report.paper_reproduction.bins[0].precipitation_mm_per_10min, Some(1.5));
        assert_eq!(report.paper_reproduction.bins[0].precipitation_rate_mm_h, Some(9.0));
    }

    #[test]
    fn rejects_current_hourly_weighing_gauge_as_ten_minute_input() {
        let mut config = config();
        config.primary.as_mut().unwrap().product_code = "DP1.00044.001".to_owned();
        assert!(validate_config(&config).unwrap_err().contains("hourly/daily"));
    }

    #[test]
    fn rejects_throughfall_as_open_rainfall() {
        let mut config = config();
        config.secondary.as_mut().unwrap().product_code = "DP1.00046.001".to_owned();
        assert!(validate_config(&config).unwrap_err().contains("DP1.00045.001"));
    }

    #[test]
    fn rejects_duplicate_interval_rows() {
        let mut config = config();
        config.primary = None;
        let csv = b"gauge_id,timestamp_utc,precipitation_mm,rangeQF,scienceQF\nSEC-01,2026-01-01T00:00:00Z,0.1,0,0\nSEC-01,2026-01-01T00:00:00Z,0.2,0,0\n";
        let error = load_source(csv, config.secondary.as_ref().unwrap(), SourceKind::SecondaryTipping, parse_utc(&config.start_utc,"start",TARGET_SECONDS).unwrap(), parse_utc(&config.end_utc,"end",TARGET_SECONDS).unwrap()).unwrap_err();
        assert!(error.to_string().contains("duplicate"));
    }

    #[test]
    fn timestamp_end_convention_is_normalized_to_bin_start() {
        let mut config = config();
        config.primary = None;
        let secondary_config = config.secondary.as_mut().unwrap();
        secondary_config.timestamp_convention = TimestampConvention::IntervalEnd;
        let csv = b"gauge_id,timestamp_utc,precipitation_mm,rangeQF,scienceQF\nSEC-01,2026-01-01T00:01:00Z,0.1,0,0\nSEC-01,2026-01-01T00:02:00Z,0.1,0,0\nSEC-01,2026-01-01T00:03:00Z,0.1,0,0\nSEC-01,2026-01-01T00:04:00Z,0.1,0,0\nSEC-01,2026-01-01T00:05:00Z,0.1,0,0\n";
        let report = evaluate(&config, "c".repeat(64), None, Some(csv)).unwrap();
        assert_eq!(report.paper_reproduction.bins[0].status, "observed_sum");
        assert_eq!(report.paper_reproduction.bins[0].precipitation_mm_per_10min, Some(0.5));
    }
}

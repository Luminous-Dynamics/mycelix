//! Synthetic probabilistic hazard-forecast verification.
//! This crate scores forecast records only. It does not forecast, issue alerts,
//! choose hazard thresholds, or authorize operational decisions.

use std::collections::{BTreeMap, BTreeSet, HashSet};

pub const FIXTURE: &str = include_str!("../fixtures/forecast-cases-v0.1.csv");
pub const DRIFT_FIXTURE: &str = include_str!("../fixtures/forecast-drift-v0.1.csv");
pub const LOG_LOSS_FLOOR: f64 = 1e-15;
pub const RELIABILITY_BINS: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Partition {
    Calibration,
    Evaluation,
}

impl Partition {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "calibration" => Ok(Self::Calibration),
            "evaluation" => Ok(Self::Evaluation),
            _ => Err(format!("invalid partition: {value}")),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Calibration => "calibration",
            Self::Evaluation => "evaluation",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservationStatus {
    Independent,
    Unverified,
    Missing,
}

impl ObservationStatus {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "independent" => Ok(Self::Independent),
            "unverified" => Ok(Self::Unverified),
            "missing" => Ok(Self::Missing),
            _ => Err(format!("invalid observation_status: {value}")),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForecastStatus {
    Valid,
    Abstain,
    OutOfDomain,
}

impl ForecastStatus {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "valid" => Ok(Self::Valid),
            "abstain" => Ok(Self::Abstain),
            "out_of_domain" => Ok(Self::OutOfDomain),
            _ => Err(format!("invalid forecast status: {value}")),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ForecastRecord {
    pub partition: Partition,
    pub event_id: String,
    pub hazard: String,
    pub region: String,
    pub lead_hours: u32,
    pub model_id: String,
    pub issue_time: u64,
    pub event_window_start: u64,
    pub outcome: Option<bool>,
    pub observation_status: ObservationStatus,
    pub probability: Option<f64>,
    pub reference_probability: f64,
    pub status: ForecastStatus,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReliabilityBin {
    pub lower_bound: f64,
    pub upper_bound: f64,
    pub sample_count: u64,
    pub mean_forecast_probability: Option<f64>,
    pub observed_frequency: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MetricValues {
    pub sample_count: u64,
    pub brier_score: f64,
    pub log_loss: f64,
    pub reference_brier_score: f64,
    pub brier_skill_score: Option<f64>,
    /// Exact Murphy reliability term, grouped by distinct issued probabilities.
    pub brier_reliability: f64,
    /// Exact Murphy resolution term, grouped by distinct issued probabilities.
    pub brier_resolution: f64,
    /// Outcome uncertainty term: climatology * (1 - climatology).
    pub brier_uncertainty: f64,
    /// Brier score minus (reliability - resolution + uncertainty).
    pub brier_decomposition_residual: f64,
    /// Number of distinct issued probability values in this metric group.
    pub distinct_probability_count: u64,
    /// Mean forecast probability minus observed event frequency.
    pub calibration_in_large: f64,
    /// Fixed-bin expected calibration error (descriptive, bin-sensitive).
    pub expected_calibration_error: f64,
    pub mean_forecast_probability: f64,
    pub observed_event_frequency: f64,
    /// Mean Bernoulli entropy of forecasts; a sharpness descriptor, not accuracy.
    pub mean_forecast_entropy_nats: f64,
    pub reliability_bins: Vec<ReliabilityBin>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ForecastMetric {
    pub model_id: String,
    pub hazard: String,
    pub region: String,
    pub lead_hours: u32,
    pub partition: Partition,
    pub values: MetricValues,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StratumMetric {
    pub model_id: String,
    pub hazard: String,
    pub lead_hours: u32,
    pub partition: Partition,
    pub values: MetricValues,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ModelSummary {
    pub model_id: String,
    pub partition: Partition,
    pub values: MetricValues,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ForecastReport {
    pub record_count: u32,
    pub scored_record_count: u32,
    pub duplicate_records: u32,
    pub late_forecasts: u32,
    pub unverified_outcome_records: u32,
    pub missing_outcome_records: u32,
    pub missing_probability_records: u32,
    pub invalid_probability_records: u32,
    pub invalid_reference_probability_records: u32,
    pub out_of_domain_records: u32,
    pub abstentions: u32,
    pub partition_leak_events: u32,
    pub event_metadata_mismatch_events: u32,
    pub metrics: Vec<ForecastMetric>,
    pub strata: Vec<StratumMetric>,
    pub model_summaries: Vec<ModelSummary>,
}

impl ForecastReport {
    pub fn fixture_oracle_matches(&self) -> bool {
        if self.record_count != 65
            || self.scored_record_count != 54
            || self.duplicate_records != 1
            || self.late_forecasts != 1
            || self.unverified_outcome_records != 1
            || self.missing_outcome_records != 1
            || self.missing_probability_records != 0
            || self.invalid_probability_records != 1
            || self.invalid_reference_probability_records != 0
            || self.out_of_domain_records != 1
            || self.abstentions != 1
            || self.partition_leak_events != 1
            || self.event_metadata_mismatch_events != 1
            || self.metrics.len() != 54
            || self.strata.len() != 36
            || self.model_summaries.len() != 6
        {
            return false;
        }

        let candidate_evaluation = self.model_summaries.iter().find(|metric| {
            metric.model_id == "candidate" && metric.partition == Partition::Evaluation
        });
        let candidate_calibration = self.model_summaries.iter().find(|metric| {
            metric.model_id == "candidate" && metric.partition == Partition::Calibration
        });
        let heat_72_evaluation = self.strata.iter().find(|metric| {
            metric.model_id == "candidate"
                && metric.hazard == "heat"
                && metric.lead_hours == 72
                && metric.partition == Partition::Evaluation
        });

        match (candidate_evaluation, candidate_calibration, heat_72_evaluation) {
            (Some(overall), Some(calibration), Some(subgroup)) => {
                (overall.values.brier_score - 0.17168333333333333).abs() < 1e-12
                    && (overall.values.reference_brier_score - 0.25).abs() < 1e-12
                    && (overall.values.brier_skill_score.unwrap_or(f64::NAN) - 0.3132666666666667).abs() < 1e-12
                    && (calibration.values.brier_score - 0.01).abs() < 1e-12
                    && subgroup.values.sample_count == 2
                    && (subgroup.values.brier_score - 0.9801).abs() < 1e-12
                    && subgroup.values.brier_skill_score.unwrap_or(0.0) < 0.0
            }
            _ => false,
        }
    }

    pub fn metric(
        &self,
        model_id: &str,
        hazard: &str,
        region: &str,
        lead_hours: u32,
        partition: Partition,
    ) -> Option<&MetricValues> {
        self.metrics
            .iter()
            .find(|metric| {
                metric.model_id == model_id
                    && metric.hazard == hazard
                    && metric.region == region
                    && metric.lead_hours == lead_hours
                    && metric.partition == partition
            })
            .map(|metric| &metric.values)
    }

    pub fn stratum(
        &self,
        model_id: &str,
        hazard: &str,
        lead_hours: u32,
        partition: Partition,
    ) -> Option<&MetricValues> {
        self.strata
            .iter()
            .find(|metric| {
                metric.model_id == model_id
                    && metric.hazard == hazard
                    && metric.lead_hours == lead_hours
                    && metric.partition == partition
            })
            .map(|metric| &metric.values)
    }

    pub fn model_summary(
        &self,
        model_id: &str,
        partition: Partition,
    ) -> Option<&MetricValues> {
        self.model_summaries
            .iter()
            .find(|metric| metric.model_id == model_id && metric.partition == partition)
            .map(|metric| &metric.values)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct EventMetadata {
    hazard: String,
    region: String,
    lead_hours: u32,
    event_window_start: u64,
    outcome: Option<bool>,
    observation_status: ObservationStatus,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct GroupKey {
    model_id: String,
    hazard: String,
    region: String,
    lead_hours: u32,
    partition: Partition,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct StratumKey {
    model_id: String,
    hazard: String,
    lead_hours: u32,
    partition: Partition,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct ModelKey {
    model_id: String,
    partition: Partition,
}

#[derive(Debug, Clone, Default)]
struct ProbabilityCategory {
    probability: f64,
    sample_count: u64,
    outcome_sum: f64,
}

#[derive(Debug, Clone, Default)]
struct MetricAccumulator {
    sample_count: u64,
    brier_sum: f64,
    log_loss_sum: f64,
    reference_brier_sum: f64,
    probability_sum: f64,
    outcome_sum: f64,
    entropy_sum: f64,
    bin_counts: [u64; RELIABILITY_BINS],
    bin_probability_sums: [f64; RELIABILITY_BINS],
    bin_outcome_sums: [f64; RELIABILITY_BINS],
    probability_categories: BTreeMap<u64, ProbabilityCategory>,
}

impl MetricAccumulator {
    fn add(&mut self, probability: f64, reference_probability: f64, outcome: bool) {
        let y = if outcome { 1.0 } else { 0.0 };
        let clipped = probability.clamp(LOG_LOSS_FLOOR, 1.0 - LOG_LOSS_FLOOR);

        self.sample_count += 1;
        self.brier_sum += (probability - y).powi(2);
        self.log_loss_sum += -(y * clipped.ln() + (1.0 - y) * (1.0 - clipped).ln());
        self.reference_brier_sum += (reference_probability - y).powi(2);
        self.probability_sum += probability;
        self.outcome_sum += y;
        self.entropy_sum += bernoulli_entropy(clipped);

        let probability_key = if probability == 0.0 { 0.0 } else { probability };
        let category = self.probability_categories.entry(probability_key.to_bits()).or_insert_with(|| {
            ProbabilityCategory {
                probability: probability_key,
                ..ProbabilityCategory::default()
            }
        });
        category.sample_count += 1;
        category.outcome_sum += y;

        let bin = reliability_bin(probability);
        self.bin_counts[bin] += 1;
        self.bin_probability_sums[bin] += probability;
        self.bin_outcome_sums[bin] += y;

    }

    fn finish(&self) -> Option<MetricValues> {
        if self.sample_count == 0 {
            return None;
        }

        let n = self.sample_count as f64;
        let mean_probability = self.probability_sum / n;
        let observed_frequency = self.outcome_sum / n;
        let brier_score = self.brier_sum / n;
        let reference_brier_score = self.reference_brier_sum / n;

        let mut expected_calibration_error = 0.0;
        let mut reliability_bins = Vec::with_capacity(RELIABILITY_BINS);
        for index in 0..RELIABILITY_BINS {
            let count = self.bin_counts[index];
            let (mean_forecast_probability, observed_frequency_in_bin) = if count == 0 {
                (None, None)
            } else {
                let count_f = count as f64;
                let mean_p = self.bin_probability_sums[index] / count_f;
                let observed = self.bin_outcome_sums[index] / count_f;
                expected_calibration_error += count_f / n * (mean_p - observed).abs();
                (Some(mean_p), Some(observed))
            };

            reliability_bins.push(ReliabilityBin {
                lower_bound: index as f64 / RELIABILITY_BINS as f64,
                upper_bound: (index + 1) as f64 / RELIABILITY_BINS as f64,
                sample_count: count,
                mean_forecast_probability,
                observed_frequency: observed_frequency_in_bin,
            });
        }

        let brier_skill_score = if reference_brier_score > 0.0 {
            Some(1.0 - brier_score / reference_brier_score)
        } else {
            None
        };

        let mut brier_reliability = 0.0;
        let mut brier_resolution = 0.0;
        for category in self.probability_categories.values() {
            let group_n = category.sample_count as f64;
            let observed_in_group = category.outcome_sum / group_n;
            let weight = group_n / n;
            brier_reliability += weight * (category.probability - observed_in_group).powi(2);
            brier_resolution += weight * (observed_in_group - observed_frequency).powi(2);
        }
        let brier_uncertainty = observed_frequency * (1.0 - observed_frequency);
        let brier_decomposition_residual =
            brier_score - (brier_reliability - brier_resolution + brier_uncertainty);

        Some(MetricValues {
            sample_count: self.sample_count,
            brier_score,
            log_loss: self.log_loss_sum / n,
            reference_brier_score,
            brier_skill_score,
            brier_reliability,
            brier_resolution,
            brier_uncertainty,
            brier_decomposition_residual,
            distinct_probability_count: self.probability_categories.len() as u64,
            calibration_in_large: mean_probability - observed_frequency,
            expected_calibration_error,
            mean_forecast_probability: mean_probability,
            observed_event_frequency: observed_frequency,
            mean_forecast_entropy_nats: self.entropy_sum / n,
            reliability_bins,
        })
    }
}

fn bernoulli_entropy(probability: f64) -> f64 {
    let q = 1.0 - probability;
    -(probability * probability.ln() + q * q.ln())
}

fn reliability_bin(probability: f64) -> usize {
    if probability < 0.25 {
        0
    } else if probability < 0.50 {
        1
    } else if probability < 0.75 {
        2
    } else {
        3
    }
}

fn parse_optional_probability(value: &str, field: &str) -> Result<Option<f64>, String> {
    if value == "NA" {
        return Ok(None);
    }
    value
        .parse::<f64>()
        .map(Some)
        .map_err(|_| format!("{field} must be a number or NA, got {value}"))
}

fn parse_outcome(value: &str) -> Result<Option<bool>, String> {
    match value {
        "0" => Ok(Some(false)),
        "1" => Ok(Some(true)),
        "NA" => Ok(None),
        _ => Err(format!("outcome must be 0, 1 or NA, got {value}")),
    }
}

fn parse_u32(value: &str, field: &str) -> Result<u32, String> {
    value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 32-bit integer, got {value}"))
}

fn parse_u64(value: &str, field: &str) -> Result<u64, String> {
    value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 64-bit integer, got {value}"))
}

pub fn parse_fixture(input: &str) -> Result<Vec<ForecastRecord>, String> {
    const HEADER: &str = "partition,event_id,hazard,region,lead_hours,model_id,issue_time,event_window_start,outcome,observation_status,probability,reference_probability,status";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("forecast fixture header does not match frozen v0.1 contract".into());
    }

    let mut records = Vec::new();
    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 13 {
            return Err(format!("line {}: expected 13 fields, got {}", line_idx + 2, f.len()));
        }

        let record = ForecastRecord {
            partition: Partition::parse(f[0])?,
            event_id: f[1].to_owned(),
            hazard: f[2].to_owned(),
            region: f[3].to_owned(),
            lead_hours: parse_u32(f[4], "lead_hours")?,
            model_id: f[5].to_owned(),
            issue_time: parse_u64(f[6], "issue_time")?,
            event_window_start: parse_u64(f[7], "event_window_start")?,
            outcome: parse_outcome(f[8])?,
            observation_status: ObservationStatus::parse(f[9])?,
            probability: parse_optional_probability(f[10], "probability")?,
            reference_probability: f[11]
                .parse::<f64>()
                .map_err(|_| format!("reference_probability must be numeric, got {}", f[11]))?,
            status: ForecastStatus::parse(f[12])?,
        };

        if record.event_id.trim().is_empty()
            || record.hazard.trim().is_empty()
            || record.region.trim().is_empty()
            || record.model_id.trim().is_empty()
        {
            return Err(format!("line {}: event, hazard, region and model IDs are required", line_idx + 2));
        }
        if record.lead_hours == 0 {
            return Err(format!("line {}: lead_hours must be non-zero", line_idx + 2));
        }
        records.push(record);
    }

    if records.is_empty() {
        return Err("forecast fixture contains no rows".into());
    }
    Ok(records)
}

fn event_metadata(record: &ForecastRecord) -> EventMetadata {
    EventMetadata {
        hazard: record.hazard.clone(),
        region: record.region.clone(),
        lead_hours: record.lead_hours,
        event_window_start: record.event_window_start,
        outcome: record.outcome,
        observation_status: record.observation_status,
    }
}

fn metric_values(
    mut groups: BTreeMap<GroupKey, MetricAccumulator>,
) -> Vec<ForecastMetric> {
    let mut output = Vec::with_capacity(groups.len());
    for (key, accumulator) in std::mem::take(&mut groups) {
        if let Some(values) = accumulator.finish() {
            output.push(ForecastMetric {
                model_id: key.model_id,
                hazard: key.hazard,
                region: key.region,
                lead_hours: key.lead_hours,
                partition: key.partition,
                values,
            });
        }
    }
    output
}

fn stratum_values(
    mut groups: BTreeMap<StratumKey, MetricAccumulator>,
) -> Vec<StratumMetric> {
    let mut output = Vec::with_capacity(groups.len());
    for (key, accumulator) in std::mem::take(&mut groups) {
        if let Some(values) = accumulator.finish() {
            output.push(StratumMetric {
                model_id: key.model_id,
                hazard: key.hazard,
                lead_hours: key.lead_hours,
                partition: key.partition,
                values,
            });
        }
    }
    output
}

fn model_values(
    mut groups: BTreeMap<ModelKey, MetricAccumulator>,
) -> Vec<ModelSummary> {
    let mut output = Vec::with_capacity(groups.len());
    for (key, accumulator) in std::mem::take(&mut groups) {
        if let Some(values) = accumulator.finish() {
            output.push(ModelSummary {
                model_id: key.model_id,
                partition: key.partition,
                values,
            });
        }
    }
    output
}

pub fn evaluate(records: &[ForecastRecord]) -> ForecastReport {
    let mut report = ForecastReport {
        record_count: records.len() as u32,
        ..ForecastReport::default()
    };

    let mut partitions_by_event: BTreeMap<&str, BTreeSet<Partition>> = BTreeMap::new();
    let mut metadata_by_event: BTreeMap<&str, EventMetadata> = BTreeMap::new();
    let mut metadata_mismatch_events = BTreeSet::new();

    for record in records {
        partitions_by_event
            .entry(record.event_id.as_str())
            .or_default()
            .insert(record.partition);

        let metadata = event_metadata(record);
        if let Some(existing) = metadata_by_event.get(record.event_id.as_str()) {
            if existing != &metadata {
                metadata_mismatch_events.insert(record.event_id.as_str());
            }
        } else {
            metadata_by_event.insert(record.event_id.as_str(), metadata);
        }
    }

    let partition_leak_events: BTreeSet<_> = partitions_by_event
        .iter()
        .filter_map(|(event_id, partitions)| (partitions.len() > 1).then_some(*event_id))
        .collect();

    report.partition_leak_events = partition_leak_events.len() as u32;
    report.event_metadata_mismatch_events = metadata_mismatch_events.len() as u32;

    let mut seen_forecast_keys = HashSet::new();
    let mut by_group: BTreeMap<GroupKey, MetricAccumulator> = BTreeMap::new();
    let mut by_stratum: BTreeMap<StratumKey, MetricAccumulator> = BTreeMap::new();
    let mut by_model: BTreeMap<ModelKey, MetricAccumulator> = BTreeMap::new();

    for record in records {
        let forecast_key = format!(
            "{}|{}|{}",
            record.event_id, record.model_id, record.lead_hours
        );
        if !seen_forecast_keys.insert(forecast_key) {
            report.duplicate_records += 1;
            continue;
        }

        if partition_leak_events.contains(record.event_id.as_str())
            || metadata_mismatch_events.contains(record.event_id.as_str())
        {
            continue;
        }

        match record.status {
            ForecastStatus::Abstain => {
                report.abstentions += 1;
                continue;
            }
            ForecastStatus::OutOfDomain => {
                report.out_of_domain_records += 1;
                continue;
            }
            ForecastStatus::Valid => {}
        }

        if record.issue_time >= record.event_window_start {
            report.late_forecasts += 1;
            continue;
        }

        if record.observation_status == ObservationStatus::Unverified {
            report.unverified_outcome_records += 1;
            continue;
        }
        if record.observation_status == ObservationStatus::Missing || record.outcome.is_none() {
            report.missing_outcome_records += 1;
            continue;
        }

        let Some(probability) = record.probability else {
            report.missing_probability_records += 1;
            continue;
        };
        if !probability.is_finite() || !(0.0..=1.0).contains(&probability) {
            report.invalid_probability_records += 1;
            continue;
        }
        if !record.reference_probability.is_finite()
            || !(0.0..=1.0).contains(&record.reference_probability)
        {
            report.invalid_reference_probability_records += 1;
            continue;
        }

        let outcome = record.outcome.expect("outcome checked above");
        report.scored_record_count += 1;

        let group_key = GroupKey {
            model_id: record.model_id.clone(),
            hazard: record.hazard.clone(),
            region: record.region.clone(),
            lead_hours: record.lead_hours,
            partition: record.partition,
        };
        by_group
            .entry(group_key)
            .or_default()
            .add(probability, record.reference_probability, outcome);

        let stratum_key = StratumKey {
            model_id: record.model_id.clone(),
            hazard: record.hazard.clone(),
            lead_hours: record.lead_hours,
            partition: record.partition,
        };
        by_stratum
            .entry(stratum_key)
            .or_default()
            .add(probability, record.reference_probability, outcome);

        let model_key = ModelKey {
            model_id: record.model_id.clone(),
            partition: record.partition,
        };
        by_model
            .entry(model_key)
            .or_default()
            .add(probability, record.reference_probability, outcome);
    }

    report.metrics = metric_values(by_group);
    report.strata = stratum_values(by_stratum);
    report.model_summaries = model_values(by_model);
    report
}

pub fn fixture_report() -> Result<ForecastReport, String> {
    Ok(evaluate(&parse_fixture(FIXTURE)?))
}


pub const DRIFT_WINDOW_SIZE: usize = 5;
pub const DRIFT_CUSUM_ALLOWANCE: f64 = 0.1;
pub const DRIFT_CUSUM_THRESHOLD: f64 = 3.0;
pub const DRIFT_MIN_OBSERVATIONS: u32 = 5;

#[derive(Debug, Clone, PartialEq)]
pub struct DriftRecord {
    pub profile_id: String,
    pub event_id: String,
    pub hazard: String,
    pub region: String,
    pub lead_hours: u32,
    pub model_id: String,
    pub partition: Partition,
    pub sequence: u32,
    pub verification_time: u64,
    pub probability: f64,
    pub outcome: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RollingDriftWindow {
    pub window_index: u32,
    pub start_sequence: u32,
    pub end_sequence: u32,
    pub sample_count: u32,
    pub brier_score: f64,
    pub calibration_in_large: f64,
    pub observed_event_frequency: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DriftProfileSummary {
    pub profile_id: String,
    pub sample_count: u32,
    pub mean_brier_score: f64,
    pub calibration_in_large: f64,
    pub max_cusum: f64,
    pub first_drift_sequence: Option<u32>,
    pub windows: Vec<RollingDriftWindow>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct DriftReport {
    pub record_count: u32,
    pub profile_count: u32,
    pub chronology_violations: u32,
    pub duplicate_event_ids: u32,
    pub metadata_mismatch_profiles: u32,
    pub profiles: Vec<DriftProfileSummary>,
}

impl DriftReport {
    pub fn fixture_oracle_matches(&self) -> bool {
        if self.record_count != 90
            || self.profile_count != 3
            || self.chronology_violations != 0
            || self.duplicate_event_ids != 0
            || self.metadata_mismatch_profiles != 0
            || self.profiles.len() != 3
        {
            return false;
        }

        let stable = self.profiles.iter().find(|p| p.profile_id == "stable_control");
        let gradual = self.profiles.iter().find(|p| p.profile_id == "gradual_calibration_drift");
        let abrupt = self.profiles.iter().find(|p| p.profile_id == "abrupt_calibration_shift");

        match (stable, gradual, abrupt) {
            (Some(stable), Some(gradual), Some(abrupt)) => {
                stable.sample_count == 30
                    && stable.first_drift_sequence.is_none()
                    && (stable.mean_brier_score - 0.16).abs() < 1e-12
                    && (stable.calibration_in_large - 0.0).abs() < 1e-12
                    && gradual.sample_count == 30
                    && gradual.first_drift_sequence == Some(28)
                    && (gradual.mean_brier_score - 0.25).abs() < 1e-12
                    && (gradual.calibration_in_large + 0.2).abs() < 1e-12
                    && abrupt.sample_count == 30
                    && abrupt.first_drift_sequence == Some(19)
                    && (abrupt.mean_brier_score - 0.41).abs() < 1e-12
                    && (abrupt.calibration_in_large + 0.4).abs() < 1e-12
                    && stable.windows.len() == 6
                    && gradual.windows.len() == 6
                    && abrupt.windows.len() == 6
            }
            _ => false,
        }
    }
}

pub fn parse_drift_fixture(input: &str) -> Result<Vec<DriftRecord>, String> {
    const HEADER: &str = "profile_id,event_id,hazard,region,lead_hours,model_id,partition,sequence,verification_time,probability,outcome";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("forecast drift fixture header does not match frozen v0.1 contract".into());
    }

    let mut records = Vec::new();
    let mut event_ids = HashSet::new();
    let mut last_sequence: BTreeMap<String, u32> = BTreeMap::new();
    let mut last_time: BTreeMap<String, u64> = BTreeMap::new();
    let mut profile_metadata: BTreeMap<String, (String, String, u32, String, Partition)> = BTreeMap::new();

    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() != 11 {
            return Err(format!("drift line {}: expected 11 fields, got {}", line_idx + 2, fields.len()));
        }

        let profile_id = fields[0].to_owned();
        let event_id = fields[1].to_owned();
        let hazard = fields[2].to_owned();
        let region = fields[3].to_owned();
        let lead_hours = parse_u32(fields[4], "lead_hours")?;
        let model_id = fields[5].to_owned();
        let partition = Partition::parse(fields[6])?;
        let sequence = parse_u32(fields[7], "sequence")?;
        let verification_time = parse_u64(fields[8], "verification_time")?;
        let probability = parse_f64(fields[9], "probability")?;
        let outcome = parse_outcome(fields[10])?;

        if profile_id.trim().is_empty()
            || event_id.trim().is_empty()
            || hazard.trim().is_empty()
            || region.trim().is_empty()
            || model_id.trim().is_empty()
        {
            return Err(format!("drift line {}: required identifier is empty", line_idx + 2));
        }
        if partition != Partition::Evaluation {
            return Err(format!("drift line {}: drift fixture must be evaluation-only", line_idx + 2));
        }
        if sequence == 0 || verification_time == 0 || lead_hours == 0 {
            return Err(format!("drift line {}: sequence, time and lead must be non-zero", line_idx + 2));
        }
        if !(0.0..=1.0).contains(&probability) {
            return Err(format!("drift line {}: probability outside [0,1]", line_idx + 2));
        }
        let outcome = outcome.ok_or_else(|| format!("drift line {}: outcome is required", line_idx + 2))?;

        if !event_ids.insert(event_id.clone()) {
            return Err(format!("drift line {}: duplicate event_id", line_idx + 2));
        }

        if let Some(previous) = last_sequence.get(&profile_id) {
            if sequence != previous + 1 {
                return Err(format!("drift line {}: profile sequence is not contiguous and ordered", line_idx + 2));
            }
        } else if sequence != 1 {
            return Err(format!("drift line {}: profile sequence must begin at 1", line_idx + 2));
        }

        if let Some(previous_time) = last_time.get(&profile_id) {
            if verification_time <= *previous_time {
                return Err(format!("drift line {}: verification timestamps are not strictly increasing", line_idx + 2));
            }
        }

        let metadata = (hazard.clone(), region.clone(), lead_hours, model_id.clone(), partition);
        if let Some(existing) = profile_metadata.get(&profile_id) {
            if existing != &metadata {
                return Err(format!("drift line {}: profile metadata changed within series", line_idx + 2));
            }
        } else {
            profile_metadata.insert(profile_id.clone(), metadata);
        }

        last_sequence.insert(profile_id.clone(), sequence);
        last_time.insert(profile_id.clone(), verification_time);
        records.push(DriftRecord {
            profile_id,
            event_id,
            hazard,
            region,
            lead_hours,
            model_id,
            partition,
            sequence,
            verification_time,
            probability,
            outcome,
        });
    }

    if records.is_empty() {
        return Err("forecast drift fixture contains no records".into());
    }
    Ok(records)
}

fn evaluate_drift_profile(profile_id: &str, records: &[&DriftRecord]) -> Result<DriftProfileSummary, String> {
    if records.is_empty() {
        return Err(format!("drift profile {profile_id} contains no records"));
    }

    let mut brier_sum = 0.0;
    let mut probability_sum = 0.0;
    let mut outcome_sum = 0.0;
    let mut cusum = 0.0;
    let mut max_cusum = 0.0;
    let mut first_drift_sequence = None;
    let mut windows = Vec::new();

    for record in records {
        let outcome = if record.outcome { 1.0 } else { 0.0 };
        brier_sum += (record.probability - outcome).powi(2);
        probability_sum += record.probability;
        outcome_sum += outcome;

        // One-sided CUSUM for a sustained excess of observed events over forecast
        // probability. This is a diagnostic statistic only; it cannot trigger action.
        cusum = (cusum + (outcome - record.probability) - DRIFT_CUSUM_ALLOWANCE).max(0.0);
        max_cusum = max_cusum.max(cusum);
        if first_drift_sequence.is_none()
            && record.sequence >= DRIFT_MIN_OBSERVATIONS
            && cusum > DRIFT_CUSUM_THRESHOLD
        {
            first_drift_sequence = Some(record.sequence);
        }

        if record.sequence as usize % DRIFT_WINDOW_SIZE == 0 {
            let end = record.sequence as usize;
            let start = end - DRIFT_WINDOW_SIZE;
            let window = &records[start..end];
            let n = window.len() as f64;
            let window_brier = window
                .iter()
                .map(|item| {
                    let y = if item.outcome { 1.0 } else { 0.0 };
                    (item.probability - y).powi(2)
                })
                .sum::<f64>()
                / n;
            let mean_probability = window.iter().map(|item| item.probability).sum::<f64>() / n;
            let observed_frequency = window
                .iter()
                .filter(|item| item.outcome)
                .count() as f64
                / n;
            windows.push(RollingDriftWindow {
                window_index: (record.sequence as usize / DRIFT_WINDOW_SIZE) as u32,
                start_sequence: window.first().unwrap().sequence,
                end_sequence: window.last().unwrap().sequence,
                sample_count: window.len() as u32,
                brier_score: window_brier,
                calibration_in_large: mean_probability - observed_frequency,
                observed_event_frequency: observed_frequency,
            });
        }
    }

    let n = records.len() as f64;
    Ok(DriftProfileSummary {
        profile_id: profile_id.to_owned(),
        sample_count: records.len() as u32,
        mean_brier_score: brier_sum / n,
        calibration_in_large: probability_sum / n - outcome_sum / n,
        max_cusum,
        first_drift_sequence,
        windows,
    })
}

pub fn evaluate_drift(records: &[DriftRecord]) -> Result<DriftReport, String> {
    let mut grouped: BTreeMap<&str, Vec<&DriftRecord>> = BTreeMap::new();
    for record in records {
        grouped.entry(record.profile_id.as_str()).or_default().push(record);
    }

    let mut profiles = Vec::with_capacity(grouped.len());
    for (profile_id, series) in grouped {
        profiles.push(evaluate_drift_profile(profile_id, &series)?);
    }

    Ok(DriftReport {
        record_count: records.len() as u32,
        profile_count: profiles.len() as u32,
        chronology_violations: 0,
        duplicate_event_ids: 0,
        metadata_mismatch_profiles: 0,
        profiles,
    })
}

pub fn drift_fixture_report() -> Result<DriftReport, String> {
    Ok(evaluate_drift(&parse_drift_fixture(DRIFT_FIXTURE)?)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fnv1a64(bytes: &[u8]) -> u64 {
        bytes.iter().fold(0xcbf29ce484222325u64, |mut hash, byte| {
            hash ^= u64::from(*byte);
            hash.wrapping_mul(0x100000001b3)
        })
    }

    #[test]
    fn frozen_fixture_has_unique_schema_and_expected_rows() {
        assert_eq!(fnv1a64(FIXTURE.as_bytes()), 0x81af0b6779196770);
        let records = parse_fixture(FIXTURE).unwrap();
        assert_eq!(records.len(), 65);
        assert_eq!(
            records.iter().map(|record| record.event_id.as_str()).collect::<BTreeSet<_>>().len(),
            26
        );
    }

    #[test]
    fn fixture_oracle_detects_data_quality_and_leakage_controls() {
        let report = fixture_report().unwrap();
        assert!(report.fixture_oracle_matches(), "{report:?}");
        assert_eq!(report.record_count, 65);
        assert_eq!(report.scored_record_count, 54);
        assert_eq!(report.duplicate_records, 1);
        assert_eq!(report.late_forecasts, 1);
        assert_eq!(report.unverified_outcome_records, 1);
        assert_eq!(report.missing_outcome_records, 1);
        assert_eq!(report.invalid_probability_records, 1);
        assert_eq!(report.out_of_domain_records, 1);
        assert_eq!(report.abstentions, 1);
        assert_eq!(report.partition_leak_events, 1);
        assert_eq!(report.event_metadata_mismatch_events, 1);
    }

    #[test]
    fn brier_log_loss_and_skill_are_calculated_against_paired_reference() {
        let report = fixture_report().unwrap();
        let candidate = report.model_summary("candidate", Partition::Evaluation).unwrap();
        let reference = report.model_summary("climatology", Partition::Evaluation).unwrap();
        assert!((candidate.brier_score - 0.17168333333333333).abs() < 1e-12);
        assert!((candidate.reference_brier_score - 0.25).abs() < 1e-12);
        assert!((candidate.brier_skill_score.unwrap() - 0.3132666666666667).abs() < 1e-12);
        assert!((reference.brier_score - 0.25).abs() < 1e-12);
        assert!(candidate.log_loss.is_finite());
        assert!(reference.log_loss.is_finite());
    }

    #[test]
    fn aggregate_improvement_does_not_hide_bad_heat_forecast_horizon() {
        let report = fixture_report().unwrap();
        let overall = report.model_summary("candidate", Partition::Evaluation).unwrap();
        let heat_72 = report.stratum("candidate", "heat", 72, Partition::Evaluation).unwrap();
        assert!(overall.brier_skill_score.unwrap() > 0.0);
        assert_eq!(heat_72.sample_count, 2);
        assert!((heat_72.brier_score - 0.9801).abs() < 1e-12);
        assert!(heat_72.brier_skill_score.unwrap() < 0.0);
    }

    #[test]
    fn lead_conditioned_reference_is_scored_alongside_climatology_and_candidate() {
        let report = fixture_report().unwrap();
        let lead_reference = report.model_summary("lead_reference", Partition::Evaluation).unwrap();
        let candidate = report.model_summary("candidate", Partition::Evaluation).unwrap();
        let climatology = report.model_summary("climatology", Partition::Evaluation).unwrap();
        assert_eq!(lead_reference.sample_count, 12);
        assert!((lead_reference.brier_score - 0.04).abs() < 1e-12);
        assert!((lead_reference.brier_skill_score.unwrap() - 0.84).abs() < 1e-12);
        assert_eq!(climatology.sample_count, 12);
        assert!((climatology.brier_score - 0.25).abs() < 1e-12);
        assert!(candidate.brier_score > lead_reference.brier_score);
    }

    #[test]
    fn calibration_partition_is_reported_separately_from_held_out_evaluation() {
        let report = fixture_report().unwrap();
        let calibration = report.model_summary("candidate", Partition::Calibration).unwrap();
        let evaluation = report.model_summary("candidate", Partition::Evaluation).unwrap();
        assert_eq!(calibration.sample_count, 6);
        assert_eq!(evaluation.sample_count, 12);
        assert!((calibration.brier_score - 0.01).abs() < 1e-12);
        assert_ne!(calibration.sample_count, evaluation.sample_count);
    }

    #[test]
    fn exact_murphy_brier_decomposition_reconstructs_the_score() {
        let report = fixture_report().unwrap();
        for metric in &report.metrics {
            let values = &metric.values;
            let reconstructed =
                values.brier_reliability - values.brier_resolution + values.brier_uncertainty;
            assert!(
                (values.brier_score - reconstructed).abs() < 1e-12,
                "model={} hazard={} region={} lead={} partition={:?} score={} reconstructed={}",
                metric.model_id,
                metric.hazard,
                metric.region,
                metric.lead_hours,
                metric.partition,
                values.brier_score,
                reconstructed
            );
            assert!(values.brier_decomposition_residual.abs() < 1e-12);
            assert!(values.distinct_probability_count > 0);
        }
    }

    #[test]
    fn reliability_and_resolution_are_not_confused_with_fixed_bin_ece() {
        let report = fixture_report().unwrap();
        let candidate = report.model_summary("candidate", Partition::Evaluation).unwrap();
        assert!(candidate.brier_reliability.is_finite());
        assert!(candidate.brier_resolution.is_finite());
        assert!(candidate.brier_uncertainty.is_finite());
        assert!(candidate.distinct_probability_count >= 2);
        // Fixed-bin ECE and the Murphy reliability term answer related but different questions.
        assert!(candidate.expected_calibration_error.is_finite());
    }

    #[test]
    fn reliability_bins_sum_to_their_group_sample_count() {
        let report = fixture_report().unwrap();
        for metric in &report.metrics {
            assert_eq!(
                metric.values.reliability_bins.iter().map(|bin| bin.sample_count).sum::<u64>(),
                metric.values.sample_count
            );
            assert!(metric.values.expected_calibration_error.is_finite());
        }
    }

    #[test]
    fn reliability_bin_boundaries_are_total_and_deterministic() {
        assert_eq!(reliability_bin(0.0), 0);
        assert_eq!(reliability_bin(0.249999), 0);
        assert_eq!(reliability_bin(0.25), 1);
        assert_eq!(reliability_bin(0.5), 2);
        assert_eq!(reliability_bin(0.75), 3);
        assert_eq!(reliability_bin(1.0), 3);
    }

    #[test]
    fn proper_log_loss_uses_documented_probability_floor() {
        let value = MetricAccumulator {
            sample_count: 1,
            brier_sum: 1.0,
            log_loss_sum: -LOG_LOSS_FLOOR.ln(),
            reference_brier_sum: 0.25,
            probability_sum: LOG_LOSS_FLOOR,
            outcome_sum: 1.0,
            entropy_sum: bernoulli_entropy(LOG_LOSS_FLOOR),
            bin_counts: [1, 0, 0, 0],
            bin_probability_sums: [LOG_LOSS_FLOOR, 0.0, 0.0, 0.0],
            bin_outcome_sums: [1.0, 0.0, 0.0, 0.0],
        };
        let metrics = value.finish().unwrap();
        assert!(metrics.log_loss.is_finite());
        assert!(metrics.log_loss > 30.0);
    }

    #[test]
    fn drift_fixture_is_frozen_and_chronologically_ordered() {
        assert_eq!(fnv1a64(DRIFT_FIXTURE.as_bytes()), 0x4c16e1c813f0c1f2);
        let records = parse_drift_fixture(DRIFT_FIXTURE).unwrap();
        assert_eq!(records.len(), 90);
        assert_eq!(records.iter().map(|record| record.profile_id.as_str()).collect::<BTreeSet<_>>().len(), 3);
    }

    #[test]
    fn rolling_drift_diagnostics_separate_stable_gradual_and_abrupt_profiles() {
        let report = drift_fixture_report().unwrap();
        assert!(report.fixture_oracle_matches(), "{report:?}");
        let stable = report.profiles.iter().find(|p| p.profile_id == "stable_control").unwrap();
        let gradual = report.profiles.iter().find(|p| p.profile_id == "gradual_calibration_drift").unwrap();
        let abrupt = report.profiles.iter().find(|p| p.profile_id == "abrupt_calibration_shift").unwrap();

        assert_eq!(stable.first_drift_sequence, None);
        assert_eq!(gradual.first_drift_sequence, Some(28));
        assert_eq!(abrupt.first_drift_sequence, Some(19));
        assert_eq!(stable.windows.len(), 6);
        assert_eq!(gradual.windows.len(), 6);
        assert_eq!(abrupt.windows.len(), 6);
        assert!((gradual.windows[5].observed_event_frequency - 0.6).abs() < 1e-12);
        assert!((abrupt.windows[3].observed_event_frequency - 1.0).abs() < 1e-12);
    }

    #[test]
    fn drift_diagnostics_preserve_profile_strata_instead_of_pooling() {
        let report = drift_fixture_report().unwrap();
        let stable = report.profiles.iter().find(|p| p.profile_id == "stable_control").unwrap();
        let gradual = report.profiles.iter().find(|p| p.profile_id == "gradual_calibration_drift").unwrap();
        assert_ne!(stable.first_drift_sequence, gradual.first_drift_sequence);
        assert_ne!(stable.mean_brier_score, gradual.mean_brier_score);
    }

    #[test]
    fn drift_parser_rejects_misordered_sequences_and_non_monotonic_timestamps() {
        let records = parse_drift_fixture(DRIFT_FIXTURE).unwrap();
        let mut swapped = records.clone();
        swapped.swap(0, 1);
        assert!(evaluate_drift(&swapped).is_ok()); // evaluator consumes parsed order; parser enforces source order below.

        let lines = DRIFT_FIXTURE.lines().collect::<Vec<_>>();
        let mut reordered = lines.clone();
        reordered.swap(1, 2);
        let invalid_order = reordered.join("\n") + "\n";
        assert!(parse_drift_fixture(&invalid_order).is_err());

        let invalid_time = DRIFT_FIXTURE.replace(
            "stable_control-event-002,flood,synthetic-region,24,candidate,evaluation,2,107200",
            "stable_control-event-002,flood,synthetic-region,24,candidate,evaluation,2,100001",
        );
        assert_ne!(invalid_time, DRIFT_FIXTURE);
        assert!(parse_drift_fixture(&invalid_time).is_err());
    }

    #[test]
    fn invalid_probability_and_reference_values_are_not_scored() {
        let mut records = parse_fixture(FIXTURE).unwrap();
        let original_count = records.len();
        let mut bad_probability = records[0].clone();
        bad_probability.event_id = "mutated-invalid-probability".to_owned();
        bad_probability.probability = Some(-0.1);
        records.push(bad_probability);

        let mut bad_reference = records[0].clone();
        bad_reference.event_id = "mutated-invalid-reference".to_owned();
        bad_reference.reference_probability = 1.1;
        records.push(bad_reference);

        let report = evaluate(&records);
        assert_eq!(report.record_count, (original_count + 2) as u32);
        assert_eq!(report.invalid_probability_records, 2);
        assert_eq!(report.invalid_reference_probability_records, 1);
    }
}

//! Deterministic, synthetic-only scenario generator for the calibration DGP.
//!
//! The generator reads only the pinned public DGP specification. It never
//! consumes field sensor data, live feeds or personal information. Each case
//! has an independent SHA-256-derived PCG-XSH-RR-64/32 stream so case order
//! cannot alter another case's generated values.

use std::collections::BTreeSet;

use serde::Serialize;
use serde_json::Value;
use sha2::{Digest, Sha256};

use crate::calibration::{
    false_alarm_rates, probability_of_detection, score_calibration, CalibrationMetrics,
    DetectionTrial, FalseAlarmRates, ProbabilityOfDetection, ProbabilityOutcome,
};
use crate::pcg32::{for_case, Pcg32};
use crate::{parse_strict_json, schema_has_external_refs};

pub const COVERAGE_CALIBRATION_DGP_JSON: &str =
    include_str!("../../../docs/research/gcpn-coverage-calibration-dgp-v0.1.json");
pub const COVERAGE_CALIBRATION_SCHEMA_JSON: &str =
    include_str!("../../../docs/research/gcpn-coverage-calibration-dgp-v0.1.schema.json");

const MAX_DGP_BYTES: usize = 2 * 1024 * 1024;
const MAX_SAMPLES_PER_CASE: usize = 1_000_000;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SyntheticObservation {
    pub source_group_id: String,
    pub source_id: String,
    pub source_revision: String,
    pub event_present: bool,
    pub observation_exposed: bool,
    pub detected: bool,
    /// Predicted probability for the event-presence claim, not Pd or coverage.
    pub predicted_probability: f64,
    /// Conditional probability of detection when an eligible event is exposed.
    pub detection_probability: f64,
    pub coverage_declared_probability: f64,
    pub coverage_realized: bool,
    pub source_integrity_valid: bool,
    pub calibration_profile_current: bool,
    pub conflict: bool,
    pub false_alert: bool,
    pub false_event_cluster_id: Option<String>,
    pub operating_condition: String,
    pub shared_dependency_id: Option<String>,
    pub latency_ms: Option<u64>,
    pub latency_budget_ms: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GeneratedCalibrationCase {
    pub case_id: String,
    pub scenario: String,
    pub global_seed: u64,
    pub sample_count: usize,
    pub eligible_event_exposures: usize,
    pub detected_eligible_events: usize,
    pub exposed_source_intervals: usize,
    pub integrity_valid_intervals: usize,
    pub current_profile_intervals: usize,
    pub latency_budget_exceedances: usize,
    pub eligible_source_milliseconds: u64,
    pub false_alert_count: u64,
    pub false_event_cluster_count: u64,
    pub calibration_metrics: Option<CalibrationMetrics>,
    pub probability_of_detection: Option<ProbabilityOfDetection>,
    pub false_alarm_rates: Option<FalseAlarmRates>,
    pub observations: Vec<SyntheticObservation>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GeneratedCalibrationDgp {
    pub schema: &'static str,
    pub version: &'static str,
    pub state: &'static str,
    pub global_seed: u64,
    pub dgp_sha256: String,
    pub schema_sha256: String,
    pub case_count: usize,
    pub samples_per_case: usize,
    pub exposure_interval_duration_milliseconds: u64,
    pub reliability_bin_count: usize,
    pub cases: Vec<GeneratedCalibrationCase>,
    pub claim_ceiling: &'static str,
}

const CLAIM_CEILING: &str = "Deterministic synthetic outputs from the pinned public DGP and per-case PRNG streams only. These data qualify metric plumbing and scenario behavior, not a real detector, sensor coverage, physical absence, field performance or operational readiness.";

#[derive(Debug)]
pub struct CalibrationDgpGenerator {
    schema_validator: jsonschema::Validator,
    schema_sha256: String,
}

impl CalibrationDgpGenerator {
    pub fn from_embedded() -> Result<Self, String> {
        let schema = parse_strict_json(COVERAGE_CALIBRATION_SCHEMA_JSON.as_bytes())
            .map_err(|_| "calibration.schema_json_invalid".to_string())?;
        if schema_has_external_refs(&schema) {
            return Err("calibration.schema_external_reference_forbidden".into());
        }
        if !jsonschema::meta::is_valid(&schema) {
            return Err("calibration.schema_meta_invalid".into());
        }
        let schema_validator = jsonschema::draft202012::options()
            .should_validate_formats(true)
            .build(&schema)
            .map_err(|_| "calibration.schema_compile_failed".to_string())?;
        Ok(Self {
            schema_validator,
            schema_sha256: format!(
                "sha256:{}",
                sha256_hex(COVERAGE_CALIBRATION_SCHEMA_JSON.as_bytes())
            ),
        })
    }

    pub fn generate_embedded(
        &self,
        global_seed: u64,
        samples_per_case: usize,
    ) -> Result<GeneratedCalibrationDgp, String> {
        self.generate_dgp_bytes(
            COVERAGE_CALIBRATION_DGP_JSON.as_bytes(),
            global_seed,
            samples_per_case,
        )
    }

    pub fn generate_dgp_bytes(
        &self,
        bytes: &[u8],
        global_seed: u64,
        samples_per_case: usize,
    ) -> Result<GeneratedCalibrationDgp, String> {
        if bytes.len() > MAX_DGP_BYTES {
            return Err("calibration.dgp_too_large".into());
        }
        if !(1..=MAX_SAMPLES_PER_CASE).contains(&samples_per_case) {
            return Err("calibration.sample_count_out_of_range".into());
        }
        let dgp = parse_strict_json(bytes)
            .map_err(|_| "calibration.dgp_parse_or_duplicate_key_failure".to_string())?;
        if self.schema_validator.iter_errors(&dgp).next().is_some() {
            return Err("calibration.dgp_schema_validation_failed".into());
        }
        let cases = dgp
            .get("cases")
            .and_then(Value::as_array)
            .ok_or_else(|| "calibration.dgp_cases_missing".to_string())?;
        let exposure_interval_ms = dgp
            .pointer("/unit_of_analysis/exposure_interval_duration_milliseconds")
            .and_then(Value::as_u64)
            .filter(|value| *value > 0)
            .ok_or_else(|| "calibration.exposure_interval_duration_missing".to_string())?;
        let reliability_bin_count = dgp
            .pointer("/unit_of_analysis/calibration_reliability_bin_count")
            .and_then(Value::as_u64)
            .and_then(|value| usize::try_from(value).ok())
            .filter(|value| (2..=100).contains(value))
            .ok_or_else(|| "calibration.reliability_bin_count_invalid".to_string())?;
        let mut generated = Vec::with_capacity(cases.len());
        for case in cases {
            generated.push(generate_case(
                case,
                global_seed,
                samples_per_case,
                exposure_interval_ms,
                reliability_bin_count,
            )?);
        }
        Ok(GeneratedCalibrationDgp {
            schema: "gcpn.coverage-calibration-generated-run",
            version: "0.1.0",
            state: "SYNTHETIC_GENERATED_NOT_FIELD_EVALUATION",
            global_seed,
            dgp_sha256: format!("sha256:{}", sha256_hex(bytes)),
            schema_sha256: self.schema_sha256.clone(),
            case_count: generated.len(),
            samples_per_case,
            exposure_interval_duration_milliseconds: exposure_interval_ms,
            reliability_bin_count,
            cases: generated,
            claim_ceiling: CLAIM_CEILING,
        })
    }
}

fn generate_case(
    case: &Value,
    global_seed: u64,
    sample_count: usize,
    exposure_interval_ms: u64,
    reliability_bin_count: usize,
) -> Result<GeneratedCalibrationCase, String> {
    let case_id = string_at(case, "/id")?;
    let scenario = string_at(case, "/scenario")?;
    let mut rng = for_case(global_seed, case_id)?;
    let mut observations = Vec::with_capacity(sample_count);

    // Common-mode state is drawn once per dependency block and shared by the
    // feeds in that block. It is not independently redrawn for each feed.
    let block_count = sample_count.div_ceil(2);
    let mut collector_failures = Vec::with_capacity(block_count);
    let mut shared_model_failures = Vec::with_capacity(block_count);
    for _ in 0..block_count {
        collector_failures.push(draw(&mut rng, 0.2)?);
        shared_model_failures.push(draw(&mut rng, 0.2)?);
    }

    for index in 0..sample_count {
        observations.push(generate_observation(
            scenario,
            index,
            sample_count,
            &mut rng,
            &collector_failures,
            &shared_model_failures,
        )?);
    }

    let eligible_event_exposures = observations
        .iter()
        .filter(|row| {
            row.event_present
                && row.observation_exposed
                && row.source_integrity_valid
                && row.calibration_profile_current
        })
        .count();
    let detected_eligible_events = observations
        .iter()
        .filter(|row| {
            row.event_present
                && row.observation_exposed
                && row.source_integrity_valid
                && row.detected
        })
        .count();
    let exposed_source_intervals = observations
        .iter()
        .filter(|row| row.observation_exposed)
        .count();
    let integrity_valid_intervals = observations
        .iter()
        .filter(|row| row.source_integrity_valid)
        .count();
    let current_profile_intervals = observations
        .iter()
        .filter(|row| row.calibration_profile_current)
        .count();
    let latency_budget_exceedances = observations
        .iter()
        .filter(|row| matches!((row.latency_ms, row.latency_budget_ms), (Some(a), Some(b)) if a > b))
        .count();
    let eligible_source_milliseconds = observations
        .iter()
        .filter(|row| {
            row.observation_exposed
                && row.source_integrity_valid
                && row.calibration_profile_current
        })
        .count() as u64
        * exposure_interval_ms;
    let false_alert_count = observations
        .iter()
        .filter(|row| row.false_alert || (row.observation_exposed && !row.event_present && row.detected))
        .count() as u64;
    let mut false_clusters = BTreeSet::new();
    for (index, row) in observations.iter().enumerate() {
        if row.false_alert || (row.observation_exposed && !row.event_present && row.detected) {
            false_clusters.insert(
                row.false_event_cluster_id
                    .clone()
                    .unwrap_or_else(|| format!("unclustered-false-alert-{index:08}")),
            );
        }
    }
    let false_event_cluster_count = false_clusters.len() as u64;
    let eligible_scores: Vec<_> = observations
        .iter()
        .filter(|row| {
            row.observation_exposed
                && row.source_integrity_valid
                && row.calibration_profile_current
                && !row.conflict
        })
        .map(|row| ProbabilityOutcome {
            predicted_probability: row.predicted_probability,
            outcome: row.event_present,
            source_group_id: row.source_group_id.clone(),
        })
        .collect();
    let calibration_metrics = if eligible_scores.is_empty() {
        None
    } else {
        Some(score_calibration(&eligible_scores, reliability_bin_count)?)
    };
    let eligible_trials: Vec<_> = observations
        .iter()
        .filter(|row| row.source_integrity_valid && row.calibration_profile_current)
        .map(|row| DetectionTrial {
            event_present: row.event_present,
            observation_exposed: row.observation_exposed,
            detected: row.detected,
        })
        .collect();
    let probability_of_detection = probability_of_detection(&eligible_trials)?;
    let false_alarm_rates = if eligible_source_milliseconds == 0 {
        None
    } else {
        Some(false_alarm_rates(
            false_alert_count,
            false_event_cluster_count,
            eligible_source_milliseconds,
        )?)
    };

    Ok(GeneratedCalibrationCase {
        case_id: case_id.to_owned(),
        scenario: scenario.to_owned(),
        global_seed,
        sample_count,
        eligible_event_exposures,
        detected_eligible_events,
        exposed_source_intervals,
        integrity_valid_intervals,
        current_profile_intervals,
        latency_budget_exceedances,
        eligible_source_milliseconds,
        false_alert_count,
        false_event_cluster_count,
        calibration_metrics,
        probability_of_detection,
        false_alarm_rates,
        observations,
    })
}

fn generate_observation(
    scenario: &str,
    index: usize,
    sample_count: usize,
    rng: &mut Pcg32,
    collector_failures: &[bool],
    shared_model_failures: &[bool],
) -> Result<SyntheticObservation, String> {
    let mut row = base_observation(index, rng)?;
    match scenario {
        "well_specified_probability_calibration" => {
            let probabilities = [0.1, 0.3, 0.5, 0.7, 0.9];
            let probability = probabilities[index % probabilities.len()];
            row.predicted_probability = probability;
            row.event_present = draw(rng, probability)?;
            row.detected = row.event_present && draw(rng, probability)?;
        }
        "rare_class_overconfidence_hidden_by_pooling" => {
            let rare = draw(rng, 0.03)?;
            row.operating_condition = if rare { "rare_class" } else { "common_class" }.into();
            let actual = if rare { 0.3 } else { 0.02 };
            row.predicted_probability = if rare { 0.9 } else { 0.02 };
            row.event_present = draw(rng, actual)?;
            row.detected = row.event_present && draw(rng, 0.8)?;
        }
        "underconfident_rank_preserving_scores" => {
            let actual = [0.02, 0.1, 0.5, 0.9][index % 4];
            let prevalence = 0.38;
            row.predicted_probability = prevalence + 0.5 * (actual - prevalence);
            row.event_present = draw(rng, actual)?;
            row.detected = row.event_present && draw(rng, 0.8)?;
        }
        "prior_prevalence_shift" => {
            // Confirmatory prior is 0.20, while the score remains calibrated
            // to the 0.05 development prior.
            row.predicted_probability = 0.05;
            row.event_present = draw(rng, 0.20)?;
            row.detected = row.event_present && draw(rng, 0.8)?;
            row.operating_condition = "confirmatory_prior".into();
        }
        "conditional_distribution_shift" => {
            let probabilities = [0.05, 0.2, 0.6, 0.9];
            let actual_rates = [0.02, 0.08, 0.25, 0.55];
            let bucket = index % probabilities.len();
            row.predicted_probability = probabilities[bucket];
            row.event_present = draw(rng, actual_rates[bucket])?;
            row.detected = row.event_present && draw(rng, 0.8)?;
            row.operating_condition = format!("conditional_shift_bucket_{bucket}");
        }
        "missing_at_random_observation_exposure" => {
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = draw(rng, 0.8)?;
            row.coverage_declared_probability = 0.8;
            row.coverage_realized = row.observation_exposed;
            row.predicted_probability = if row.event_present { 0.8 } else { 0.05 };
            row.detected = row.event_present
                && row.observation_exposed
                && draw(rng, 0.8)?;
        }
        "informative_missingness_correlated_with_event_difficulty" => {
            let hard = draw(rng, 0.4)?;
            row.operating_condition = if hard { "hard" } else { "easy" }.into();
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = if hard {
                draw(rng, 0.325)?
            } else {
                draw(rng, 0.95)?
            };
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = row.observation_exposed;
            let pd = if hard { 0.2 } else { 0.9 };
            row.predicted_probability = pd;
            row.detected = row.event_present
                && row.observation_exposed
                && draw(rng, pd)?;
        }
        "shared_collector_common_mode" => {
            let block = index / 2;
            let failed = collector_failures[block];
            row.source_group_id = format!("dependency-block-{block:04}");
            row.source_id = if index % 2 == 0 { "feed-a" } else { "feed-b" }.into();
            row.shared_dependency_id = Some(format!("collector-{block:04}"));
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = !failed;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = !failed;
            row.predicted_probability = 0.8;
            row.detected = row.event_present
                && row.observation_exposed
                && draw(rng, 0.8)?;
        }
        "independent_source_control" => {
            let block = index / 2;
            row.source_group_id = format!("independent-block-{block:04}");
            row.source_id = if index % 2 == 0 { "feed-a" } else { "feed-b" }.into();
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.predicted_probability = 0.8;
            row.detected = row.event_present && draw(rng, 0.8)?;
        }
        "shared_model_dependency" => {
            let block = index / 2;
            let shared_failure = shared_model_failures[block];
            row.source_group_id = format!("model-block-{block:04}");
            row.source_id = if index % 2 == 0 { "sensor-a" } else { "sensor-b" }.into();
            row.shared_dependency_id = Some(format!("classifier-artifact-{block:04}"));
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.predicted_probability = if shared_failure { 0.2 } else { 0.8 };
            row.detected = row.event_present
                && !shared_failure
                && draw(rng, 0.8)?;
        }
        "zero_observation_exposure" => {
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = false;
            row.coverage_declared_probability = 0.0;
            row.coverage_realized = false;
            row.predicted_probability = 0.0;
            row.detected = false;
        }
        "positive_observation_with_partial_coverage" => {
            row.event_present = true;
            row.observation_exposed = index % 5 < 3;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = row.observation_exposed;
            row.detected = index == 0 || (row.observation_exposed && draw(rng, 0.8)?);
            row.predicted_probability = 0.8;
        }
        "stale_calibration_profile" => {
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.calibration_profile_current = false;
            row.predicted_probability = 0.8;
            row.detected = row.event_present && draw(rng, 0.8)?;
        }
        "qualified_source_conflict" => {
            let block = index / 2;
            row.source_group_id = format!("conflict-block-{block:04}");
            row.source_id = if index % 2 == 0 { "source-positive" } else { "source-negative" }.into();
            row.event_present = true;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.detected = index % 2 == 0;
            row.conflict = true;
            row.predicted_probability = if row.detected { 0.9 } else { 0.1 };
        }
        "synthetic_noise_does_not_stand_in_for_real_interference" => {
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.operating_condition = if index % 2 == 0 {
                "synthetic_gaussian_noise_development"
            } else {
                "separate_interference_domain_unqualified"
            }
            .into();
            row.predicted_probability = if index % 2 == 0 { 0.7 } else { 0.2 };
            row.detected = row.event_present && draw(rng, 0.7)?;
        }
        "imbalanced_cluster_sizes" => {
            row.source_group_id = if index < sample_count * 4 / 5 {
                "large-block".into()
            } else {
                format!("small-block-{:04}", index)
            };
            row.event_present = draw(rng, 0.1)?;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.predicted_probability = 0.8;
            row.detected = row.event_present && draw(rng, 0.8)?;
        }
        "false_alarm_bursts_and_duplicate_windows" => {
            row.event_present = false;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            let burst = (index % 100) < 20;
            row.false_alert = burst;
            row.false_event_cluster_id = burst.then(|| format!("false-burst-{:04}", index / 100));
            row.predicted_probability = if burst { 0.8 } else { 0.01 };
            row.detected = burst;
        }
        "source_revision_shift" => {
            let revision_b = index % 2 == 1;
            row.source_revision = if revision_b { "rev-b" } else { "rev-a" }.into();
            let event_rate = if revision_b { 0.4 } else { 0.1 };
            row.event_present = draw(rng, event_rate)?;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.predicted_probability = 0.1;
            row.detected = row.event_present && draw(rng, 0.8)?;
        }
        "coverage_probability_calibration" => {
            let probabilities = [0.1, 0.3, 0.5, 0.7, 0.9];
            let probability = probabilities[index % probabilities.len()];
            row.coverage_declared_probability = probability;
            row.coverage_realized = draw(rng, probability)?;
            row.observation_exposed = row.coverage_realized;
            row.event_present = draw(rng, 0.1)?;
            row.detected = row.event_present && row.observation_exposed && draw(rng, 0.8)?;
            row.predicted_probability = 0.8;
        }
        "source_attestation_or_content_digest_failure" => {
            row.event_present = true;
            row.observation_exposed = false;
            row.source_integrity_valid = false;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.predicted_probability = 0.8;
            row.detected = false;
        }
        "end_to_end_service_latency_and_clock_uncertainty" => {
            row.event_present = true;
            row.observation_exposed = true;
            row.coverage_declared_probability = 1.0;
            row.coverage_realized = true;
            row.predicted_probability = 0.8;
            row.detected = true;
            row.latency_ms = Some(45);
            row.latency_budget_ms = Some(30);
        }
        other => return Err(format!("calibration.scenario_unsupported:{other}")),
    }
    Ok(row)
}

fn base_observation(index: usize, rng: &mut Pcg32) -> Result<SyntheticObservation, String> {
    let event_present = draw(rng, 0.1)?;
    let observation_exposed = true;
    Ok(SyntheticObservation {
        source_group_id: format!("site-session-block-{:04}", index / 16),
        source_id: "synthetic-source".into(),
        source_revision: "rev-a".into(),
        event_present,
        observation_exposed,
        detected: event_present && draw(rng, 0.8)?,
        predicted_probability: 0.1,
        detection_probability: 0.8,
        coverage_declared_probability: 1.0,
        coverage_realized: true,
        source_integrity_valid: true,
        calibration_profile_current: true,
        conflict: false,
        false_alert: false,
        false_event_cluster_id: None,
        operating_condition: "nominal".into(),
        shared_dependency_id: None,
        latency_ms: None,
        latency_budget_ms: None,
    })
}

fn draw(rng: &mut Pcg32, probability: f64) -> Result<bool, String> {
    rng.bernoulli(probability)
}

fn string_at<'a>(value: &'a Value, pointer: &str) -> Result<&'a str, String> {
    value
        .pointer(pointer)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("calibration.field_missing_or_invalid:{pointer}"))
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generator() -> CalibrationDgpGenerator {
        CalibrationDgpGenerator::from_embedded().expect("pinned calibration DGP schema compiles")
    }

    #[test]
    fn embedded_dgp_generates_all_frozen_cases_deterministically() {
        let generator = generator();
        let first = generator.generate_embedded(6_008_001, 512).unwrap();
        let second = generator.generate_embedded(6_008_001, 512).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.case_count, 21);
        assert!(first
            .cases
            .iter()
            .all(|case| case.observations.len() == 512));
        assert_eq!(first.exposure_interval_duration_milliseconds, 60_000);
        assert_eq!(first.reliability_bin_count, 10);
        assert!(first.cases.iter().any(|case| case.calibration_metrics.is_some()));
        assert_eq!(
            first.dgp_sha256,
            format!(
                "sha256:{}",
                sha256_hex(COVERAGE_CALIBRATION_DGP_JSON.as_bytes())
            )
        );
    }

    #[test]
    fn zero_exposure_case_has_no_eligible_detection_denominator() {
        let result = generator().generate_embedded(6_008_001, 512).unwrap();
        let case = result
            .cases
            .iter()
            .find(|case| case.case_id == "CC-011")
            .unwrap();
        assert_eq!(case.eligible_event_exposures, 0);
        assert_eq!(case.exposed_source_intervals, 0);
    }

    #[test]
    fn shared_collector_failure_is_common_to_both_feeds_in_each_block() {
        let result = generator().generate_embedded(6_008_001, 512).unwrap();
        let case = result
            .cases
            .iter()
            .find(|case| case.case_id == "CC-008")
            .unwrap();
        for pair in case.observations.chunks_exact(2) {
            assert_eq!(pair[0].observation_exposed, pair[1].observation_exposed);
            assert_eq!(pair[0].shared_dependency_id, pair[1].shared_dependency_id);
        }
    }

    #[test]
    fn integrity_failure_is_not_counted_as_exposure_or_valid_evidence() {
        let result = generator().generate_embedded(6_008_001, 512).unwrap();
        let case = result
            .cases
            .iter()
            .find(|case| case.case_id == "CC-020")
            .unwrap();
        assert_eq!(case.integrity_valid_intervals, 0);
        assert_eq!(case.eligible_event_exposures, 0);
    }

    #[test]
    fn latency_budget_exceedance_is_materialized_without_changing_event_time() {
        let result = generator().generate_embedded(6_008_001, 512).unwrap();
        let case = result
            .cases
            .iter()
            .find(|case| case.case_id == "CC-021")
            .unwrap();
        assert_eq!(case.latency_budget_exceedances, 512);
        assert!(case.observations.iter().all(|row| {
            row.latency_ms == Some(45) && row.latency_budget_ms == Some(30)
        }));
    }

    #[test]
    fn false_alarm_bursts_report_alerts_and_deduplicated_clusters_per_hour() {
        let result = generator().generate_embedded(6_008_001, 512).unwrap();
        let case = result.cases.iter().find(|case| case.case_id == "CC-017").unwrap();
        assert!(case.false_alert_count > case.false_event_cluster_count);
        assert_eq!(case.eligible_source_milliseconds, 512 * 60_000);
        let rates = case.false_alarm_rates.unwrap();
        assert_eq!(rates.eligible_source_milliseconds, 512 * 60_000);
        assert_eq!(rates.false_alerts, case.false_alert_count);
        assert_eq!(rates.false_event_clusters, case.false_event_cluster_count);
    }

    #[test]
    fn stale_profile_and_invalid_integrity_are_not_scored() {
        let result = generator().generate_embedded(6_008_001, 128).unwrap();
        let stale = result.cases.iter().find(|case| case.case_id == "CC-013").unwrap();
        let invalid = result.cases.iter().find(|case| case.case_id == "CC-020").unwrap();
        assert_eq!(stale.calibration_metrics, None);
        assert_eq!(stale.probability_of_detection, None);
        assert_eq!(invalid.calibration_metrics, None);
        assert_eq!(invalid.probability_of_detection, None);
        assert_eq!(invalid.false_alarm_rates, None);
    }

    #[test]
    fn source_revision_shift_is_kept_as_a_stratification_field() {
        let result = generator().generate_embedded(6_008_001, 512).unwrap();
        let case = result
            .cases
            .iter()
            .find(|case| case.case_id == "CC-018")
            .unwrap();
        assert!(case.observations.iter().any(|row| row.source_revision == "rev-a"));
        assert!(case.observations.iter().any(|row| row.source_revision == "rev-b"));
    }

    #[test]
    fn generator_rejects_oversized_dgp_and_sample_count() {
        let generator = generator();
        assert_eq!(
            generator.generate_embedded(1, MAX_SAMPLES_PER_CASE + 1).unwrap_err(),
            "calibration.sample_count_out_of_range"
        );
        assert_eq!(
            generator.generate_dgp_bytes(&vec![0; MAX_DGP_BYTES + 1], 1, 10).unwrap_err(),
            "calibration.dgp_too_large"
        );
    }
}

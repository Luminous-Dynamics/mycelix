//! Paired, independent-block statistics for civilian Track A studies.
//!
//! Each block is re-evaluated by the deterministic Track A evaluator. Resampling
//! occurs over declared independent blocks, never over correlated event rows.
//! Synthetic previews are always non-inferential.

use gcpn_track_a_replay_evaluator::{
    EvidenceTier as RunEvidenceTier, TrackARunBundle, evaluate_with_expected_sha256,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const INPUT_SCHEMA: &str = "luminous.gcpn.track_a_paired_study.v0.1";
pub const REPORT_SCHEMA: &str = "luminous.gcpn.track_a_paired_study_report.v0.1";
pub const EXPECTED_FIXTURE_SHA256: &str =
    "ff116056f68e333960e7c05751da7d73d53cdad6893d946aa76001f0bdac5d6a";
pub const CONTROLLED_MINIMUM_INDEPENDENT_UNITS: usize = 30;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum StudyEvidenceTier {
    SyntheticPreview,
    ControlledReplay,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum IndependentUnitType {
    SiteDay,
    FailureEpoch,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StudyBundle {
    pub schema: String,
    pub study_id: String,
    pub evidence_tier: StudyEvidenceTier,
    pub independent_unit_type: IndependentUnitType,
    pub preregistration: Preregistration,
    pub independence_assessment_receipt_sha256: Option<String>,
    pub blocks: Vec<StudyBlock>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Preregistration {
    pub baseline_pipeline_id: String,
    pub candidate_pipeline_id: String,
    pub minimum_independent_units: usize,
    pub confidence_level_permille: u16,
    pub bootstrap_resamples: usize,
    pub bootstrap_seed: u64,
    pub relative_reduction_target_micros: u32,
    pub recall_non_inferiority_margin_micros: u32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StudyBlock {
    pub independent_unit_id: String,
    pub run_bundle_sha256: String,
    pub run_bundle_json: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct StudyReport {
    pub report_schema: &'static str,
    pub status: &'static str,
    pub input_sha256: String,
    pub study_id: Option<String>,
    pub evidence_tier: Option<&'static str>,
    pub independent_unit_type: Option<&'static str>,
    pub independent_units_observed: usize,
    pub independent_units_required: usize,
    pub baseline_pipeline_id: Option<String>,
    pub candidate_pipeline_id: Option<String>,
    pub bootstrap_resamples: Option<usize>,
    pub bootstrap_seed: Option<u64>,
    pub point_estimates: Option<PairedPointEstimates>,
    pub confidence_intervals: Option<PairedConfidenceIntervals>,
    pub block_results: Vec<BlockPairResult>,
    pub hypothesis_supported: Option<bool>,
    pub inference_permitted: bool,
    pub reasons: Vec<String>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, Serialize)]
pub struct PairedPointEstimates {
    pub baseline_false_review_requests_per_hour: f64,
    pub candidate_false_review_requests_per_hour: f64,
    pub false_review_relative_reduction: Option<f64>,
    pub baseline_human_review_seconds_per_hour: f64,
    pub candidate_human_review_seconds_per_hour: f64,
    pub human_review_relative_reduction: Option<f64>,
    pub baseline_mean_recall: f64,
    pub candidate_mean_recall: f64,
    pub recall_difference: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct MetricInterval {
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    pub valid_resamples: usize,
}

#[derive(Clone, Debug, Serialize)]
pub struct PairedConfidenceIntervals {
    pub false_review_relative_reduction: MetricInterval,
    pub human_review_relative_reduction: MetricInterval,
    pub recall_difference: MetricInterval,
    pub confidence_level_permille: u16,
}

#[derive(Clone, Debug, Serialize)]
pub struct BlockPairResult {
    pub independent_unit_id: String,
    pub run_bundle_sha256: String,
    pub baseline_false_review_requests_per_hour: f64,
    pub candidate_false_review_requests_per_hour: f64,
    pub baseline_human_review_seconds_per_hour: f64,
    pub candidate_human_review_seconds_per_hour: f64,
    pub baseline_recall: f64,
    pub candidate_recall: f64,
}

#[derive(Clone, Debug)]
struct PairedBlock {
    false_review_baseline: f64,
    false_review_candidate: f64,
    human_time_baseline: f64,
    human_time_candidate: f64,
    recall_baseline: f64,
    recall_candidate: f64,
    public_result: BlockPairResult,
}

#[derive(Clone, Debug)]
struct BootstrapOutput {
    intervals: PairedConfidenceIntervals,
    point: PairedPointEstimates,
    hypothesis_supported: bool,
}

fn empty_report(
    input_sha256: String,
    status: &'static str,
    study_id: Option<String>,
    reasons: Vec<String>,
) -> StudyReport {
    StudyReport {
        report_schema: REPORT_SCHEMA,
        status,
        input_sha256,
        study_id,
        evidence_tier: None,
        independent_unit_type: None,
        independent_units_observed: 0,
        independent_units_required: CONTROLLED_MINIMUM_INDEPENDENT_UNITS,
        baseline_pipeline_id: None,
        candidate_pipeline_id: None,
        bootstrap_resamples: None,
        bootstrap_seed: None,
        point_estimates: None,
        confidence_intervals: None,
        block_results: Vec::new(),
        hypothesis_supported: None,
        inference_permitted: false,
        reasons,
        claim_ceiling: "paired_track_a_statistics_over_declared_independent_blocks_only",
    }
}

pub fn evaluate_frozen_fixture(bytes: &[u8]) -> StudyReport {
    evaluate_with_expected_sha256(bytes, Some(EXPECTED_FIXTURE_SHA256))
}

pub fn evaluate(bytes: &[u8]) -> StudyReport {
    evaluate_with_expected_sha256(bytes, None)
}

pub fn evaluate_with_expected_sha256(
    bytes: &[u8],
    expected_sha256: Option<&str>,
) -> StudyReport {
    let input_sha256 = format!("{:x}", Sha256::digest(bytes));
    if let Some(expected) = expected_sha256 {
        if !is_sha256(expected) || input_sha256 != expected {
            return empty_report(
                input_sha256,
                "REJECTED",
                None,
                vec!["study_input_digest_mismatch".to_string()],
            );
        }
    }

    let study = match serde_json::from_slice::<StudyBundle>(bytes) {
        Ok(study) => study,
        Err(error) => {
            return empty_report(
                input_sha256,
                "REJECTED",
                None,
                vec![format!("strict_study_json_decode: {error}")],
            );
        }
    };

    let validation_errors = validate_study(&study);
    if !validation_errors.is_empty() {
        return empty_report(
            input_sha256,
            "REJECTED",
            Some(study.study_id),
            validation_errors,
        );
    }

    if study.evidence_tier == StudyEvidenceTier::ControlledReplay
        && !study
            .independence_assessment_receipt_sha256
            .as_deref()
            .is_some_and(is_sha256)
    {
        return blocked_report(
            input_sha256,
            &study,
            vec!["independent_unit_assessment_receipt_missing_or_invalid".to_string()],
        );
    }

    let mut unit_ids = BTreeSet::new();
    let mut bundle_digests = BTreeSet::new();
    let mut event_set_digests = BTreeSet::<String>::new();
    let mut blocks = Vec::with_capacity(study.blocks.len());
    let mut common_operator_policy: Option<String> = None;

    for block in &study.blocks {
        if !unit_ids.insert(block.independent_unit_id.as_str()) {
            return empty_report(
                input_sha256,
                "REJECTED",
                Some(study.study_id),
                vec![format!(
                    "duplicate_independent_unit_id:{}",
                    block.independent_unit_id
                )],
            );
        }
        if !bundle_digests.insert(block.run_bundle_sha256.as_str()) {
            return empty_report(
                input_sha256,
                "REJECTED",
                Some(study.study_id),
                vec![format!(
                    "duplicate_run_bundle_digest:{}",
                    block.independent_unit_id
                )],
            );
        }

        let run_report = evaluate_with_expected_sha256(
            block.run_bundle_json.as_bytes(),
            Some(&block.run_bundle_sha256),
        );
        match run_report.status {
            "BLOCKED" => {
                return blocked_report(
                    input_sha256,
                    &study,
                    vec![format!(
                        "track_a_block_blocked:{}",
                        block.independent_unit_id
                    )],
                );
            }
            "INCOMPARABLE" => {
                return empty_report(
                    input_sha256,
                    "INCOMPARABLE",
                    Some(study.study_id),
                    vec![format!(
                        "track_a_block_incomparable:{}",
                        block.independent_unit_id
                    )],
                );
            }
            "REJECTED" => {
                return empty_report(
                    input_sha256,
                    "REJECTED",
                    Some(study.study_id),
                    vec![format!(
                        "track_a_block_rejected:{}",
                        block.independent_unit_id
                    )],
                );
            }
            "COMPARABLE" => {}
            _ => {
                return empty_report(
                    input_sha256,
                    "REJECTED",
                    Some(study.study_id),
                    vec!["unsupported_track_a_evaluator_status".to_string()],
                );
            }
        }

        let run_bundle = match serde_json::from_str::<TrackARunBundle>(&block.run_bundle_json) {
            Ok(bundle) => bundle,
            Err(error) => {
                return empty_report(
                    input_sha256,
                    "REJECTED",
                    Some(study.study_id),
                    vec![format!("strict_child_bundle_decode: {error}")],
                );
            }
        };

        let tier_matches = match study.evidence_tier {
            StudyEvidenceTier::SyntheticPreview => {
                run_bundle.evidence_tier == RunEvidenceTier::SyntheticFixture
                    && run_bundle.simulation_only
            }
            StudyEvidenceTier::ControlledReplay => {
                run_bundle.evidence_tier == RunEvidenceTier::ControlledReplay
                    && !run_bundle.simulation_only
            }
        };
        if !tier_matches {
            return empty_report(
                input_sha256,
                "REJECTED",
                Some(study.study_id),
                vec![format!(
                    "evidence_tier_mismatch:{}",
                    block.independent_unit_id
                )],
            );
        }

        if !event_set_digests.insert(run_bundle.event_set_sha256.clone()) {
            return empty_report(
                input_sha256,
                "REJECTED",
                Some(study.study_id),
                vec![format!(
                    "reused_event_set_across_independent_units:{}",
                    block.independent_unit_id
                )],
            );
        }

        match &common_operator_policy {
            None => common_operator_policy = Some(run_bundle.operator_policy_sha256.clone()),
            Some(policy) if policy != &run_bundle.operator_policy_sha256 => {
                return empty_report(
                    input_sha256,
                    "INCOMPARABLE",
                    Some(study.study_id),
                    vec![format!(
                        "operator_policy_changed_between_blocks:{}",
                        block.independent_unit_id
                    )],
                );
            }
            Some(_) => {}
        }

        let baseline = match run_bundle
            .pipelines
            .iter()
            .find(|pipeline| pipeline.pipeline_id == study.preregistration.baseline_pipeline_id)
        {
            Some(pipeline) => pipeline,
            None => {
                return empty_report(
                    input_sha256,
                    "INCOMPARABLE",
                    Some(study.study_id),
                    vec![format!(
                        "baseline_pipeline_missing:{}",
                        block.independent_unit_id
                    )],
                );
            }
        };
        let candidate = match run_bundle
            .pipelines
            .iter()
            .find(|pipeline| pipeline.pipeline_id == study.preregistration.candidate_pipeline_id)
        {
            Some(pipeline) => pipeline,
            None => {
                return empty_report(
                    input_sha256,
                    "INCOMPARABLE",
                    Some(study.study_id),
                    vec![format!(
                        "candidate_pipeline_missing:{}",
                        block.independent_unit_id
                    )],
                );
            }
        };

        let baseline_eval = match run_report
            .pipeline_results
            .iter()
            .find(|result| result.pipeline_id == baseline.pipeline_id)
        {
            Some(result) if result.status == "SCORED" => result,
            _ => {
                return empty_report(
                    input_sha256,
                    "INCOMPARABLE",
                    Some(study.study_id),
                    vec![format!(
                        "baseline_pipeline_not_scored:{}",
                        block.independent_unit_id
                    )],
                );
            }
        };
        let candidate_eval = match run_report
            .pipeline_results
            .iter()
            .find(|result| result.pipeline_id == candidate.pipeline_id)
        {
            Some(result) if result.status == "SCORED" => result,
            _ => {
                return empty_report(
                    input_sha256,
                    "INCOMPARABLE",
                    Some(study.study_id),
                    vec![format!(
                        "candidate_pipeline_not_scored:{}",
                        block.independent_unit_id
                    )],
                );
            }
        };

        let baseline_metrics = baseline_eval.metrics.as_ref().expect("scored metrics");
        let candidate_metrics = candidate_eval.metrics.as_ref().expect("scored metrics");
        let baseline_false_review = baseline_metrics.false_review_requests_per_hour;
        let candidate_false_review = candidate_metrics.false_review_requests_per_hour;
        let baseline_recall = baseline_metrics.recall;
        let candidate_recall = candidate_metrics.recall;

        let monitoring_hours =
            run_bundle.monitoring_duration_seconds as f64 / 3600.0;
        let baseline_human_seconds = human_review_seconds(&baseline.case_results) / monitoring_hours;
        let candidate_human_seconds =
            human_review_seconds(&candidate.case_results) / monitoring_hours;

        let values = [
            baseline_false_review,
            candidate_false_review,
            baseline_recall,
            candidate_recall,
        ];
        if values.iter().any(Option::is_none)
            || !baseline_human_seconds.is_finite()
            || !candidate_human_seconds.is_finite()
        {
            return empty_report(
                input_sha256,
                "INCOMPARABLE",
                Some(study.study_id),
                vec![format!(
                    "required_endpoint_unavailable:{}",
                    block.independent_unit_id
                )],
            );
        }

        let baseline_false_review = baseline_false_review.unwrap();
        let candidate_false_review = candidate_false_review.unwrap();
        let baseline_recall = baseline_recall.unwrap();
        let candidate_recall = candidate_recall.unwrap();

        if ![
            baseline_false_review,
            candidate_false_review,
            baseline_human_seconds,
            candidate_human_seconds,
            baseline_recall,
            candidate_recall,
        ]
        .iter()
        .all(|value| value.is_finite())
        {
            return empty_report(
                input_sha256,
                "REJECTED",
                Some(study.study_id),
                vec![format!(
                    "non_finite_endpoint:{}",
                    block.independent_unit_id
                )],
            );
        }

        let paired = PairedBlock {
            false_review_baseline: baseline_false_review,
            false_review_candidate: candidate_false_review,
            human_time_baseline: baseline_human_seconds,
            human_time_candidate: candidate_human_seconds,
            recall_baseline: baseline_recall,
            recall_candidate: candidate_recall,
            public_result: BlockPairResult {
                independent_unit_id: block.independent_unit_id.clone(),
                run_bundle_sha256: block.run_bundle_sha256.clone(),
                baseline_false_review_requests_per_hour: baseline_false_review,
                candidate_false_review_requests_per_hour: candidate_false_review,
                baseline_human_review_seconds_per_hour: baseline_human_seconds,
                candidate_human_review_seconds_per_hour: candidate_human_seconds,
                baseline_recall,
                candidate_recall,
            },
        };
        blocks.push(paired);
    }

    let bootstrap = bootstrap_paired(&blocks, &study.preregistration);
    let minimum = study.preregistration.minimum_independent_units;
    let count = blocks.len();
    let synthetic = study.evidence_tier == StudyEvidenceTier::SyntheticPreview;
    let enough_units = count >= minimum;
    let status = if synthetic {
        "SYNTHETIC_PREVIEW"
    } else if !enough_units {
        "INSUFFICIENT_INDEPENDENT_UNITS"
    } else {
        "CONTROLLED_PAIRED_RESULT"
    };
    let inference_permitted =
        inference_permitted(study.evidence_tier, count, minimum);

    StudyReport {
        report_schema: REPORT_SCHEMA,
        status,
        input_sha256,
        study_id: Some(study.study_id),
        evidence_tier: Some(study_evidence_name(study.evidence_tier).to_string()),
        independent_unit_type: Some(unit_type_name(study.independent_unit_type).to_string()),
        independent_units_observed: count,
        independent_units_required: minimum,
        baseline_pipeline_id: Some(study.preregistration.baseline_pipeline_id),
        candidate_pipeline_id: Some(study.preregistration.candidate_pipeline_id),
        bootstrap_resamples: Some(study.preregistration.bootstrap_resamples),
        bootstrap_seed: Some(study.preregistration.bootstrap_seed),
        point_estimates: Some(bootstrap.point),
        confidence_intervals: Some(bootstrap.intervals),
        block_results: blocks
            .into_iter()
            .map(|block| block.public_result)
            .collect(),
        hypothesis_supported: if inference_permitted {
            Some(bootstrap.hypothesis_supported)
        } else {
            None
        },
        inference_permitted,
        reasons: if synthetic {
            vec!["synthetic_fixture_is_non_inferential".to_string()]
        } else if !enough_units {
            vec!["minimum_independent_unit_count_not_met".to_string()]
        } else {
            Vec::new()
        },
        claim_ceiling: "paired_track_a_statistics_over_declared_independent_blocks_only",
    }
}

fn validate_study(study: &StudyBundle) -> Vec<String> {
    let mut errors = Vec::new();
    if study.schema != INPUT_SCHEMA {
        errors.push("study_schema_mismatch".to_string());
    }
    if !valid_identifier(&study.study_id) {
        errors.push("invalid_study_id".to_string());
    }
    let pre = &study.preregistration;
    if !valid_identifier(&pre.baseline_pipeline_id)
        || !valid_identifier(&pre.candidate_pipeline_id)
        || pre.baseline_pipeline_id == pre.candidate_pipeline_id
    {
        errors.push("invalid_or_identical_pipeline_ids".to_string());
    }
    if pre.minimum_independent_units < CONTROLLED_MINIMUM_INDEPENDENT_UNITS {
        errors.push("minimum_independent_units_below_starter_floor".to_string());
    }
    if !(900..=999).contains(&pre.confidence_level_permille) {
        errors.push("confidence_level_out_of_range".to_string());
    }
    if !(2_000..=1_000_000).contains(&pre.bootstrap_resamples) {
        errors.push("bootstrap_resamples_out_of_range".to_string());
    }
    if pre.relative_reduction_target_micros > 1_000_000
        || pre.recall_non_inferiority_margin_micros > 1_000_000
    {
        errors.push("preregistered_threshold_out_of_range".to_string());
    }
    if study.blocks.is_empty() {
        errors.push("study_blocks_empty".to_string());
    }
    if let Some(receipt) = study.independence_assessment_receipt_sha256.as_deref() {
        if !is_sha256(receipt) {
            errors.push("invalid_independence_assessment_receipt_sha256".to_string());
        }
    }
    for block in &study.blocks {
        if !valid_identifier(&block.independent_unit_id) {
            errors.push(format!(
                "invalid_independent_unit_id:{}",
                block.independent_unit_id
            ));
        }
        if !is_sha256(&block.run_bundle_sha256) {
            errors.push(format!(
                "invalid_block_bundle_sha256:{}",
                block.independent_unit_id
            ));
        }
        if block.run_bundle_json.is_empty() {
            errors.push(format!("empty_block_bundle:{}", block.independent_unit_id));
        }
    }
    errors.sort();
    errors
}

fn blocked_report(
    input_sha256: String,
    study: &StudyBundle,
    reasons: Vec<String>,
) -> StudyReport {
    let mut report = empty_report(
        input_sha256,
        "BLOCKED",
        Some(study.study_id.clone()),
        reasons,
    );
    report.evidence_tier = Some(study_evidence_name(study.evidence_tier).to_string());
    report.independent_unit_type = Some(unit_type_name(study.independent_unit_type).to_string());
    report.independent_units_required = study.preregistration.minimum_independent_units;
    report.baseline_pipeline_id = Some(study.preregistration.baseline_pipeline_id.clone());
    report.candidate_pipeline_id = Some(study.preregistration.candidate_pipeline_id.clone());
    report.bootstrap_resamples = Some(study.preregistration.bootstrap_resamples);
    report.bootstrap_seed = Some(study.preregistration.bootstrap_seed);
    report
}

fn inference_permitted(
    evidence_tier: StudyEvidenceTier,
    observed_units: usize,
    required_units: usize,
) -> bool {
    evidence_tier == StudyEvidenceTier::ControlledReplay && observed_units >= required_units
}

fn bootstrap_paired(blocks: &[PairedBlock], prereg: &Preregistration) -> BootstrapOutput {
    let n = blocks.len() as f64;
    let mean = |values: Vec<f64>| values.iter().sum::<f64>() / n;

    let baseline_false = mean(
        blocks
            .iter()
            .map(|block| block.false_review_baseline)
            .collect(),
    );
    let candidate_false = mean(
        blocks
            .iter()
            .map(|block| block.false_review_candidate)
            .collect(),
    );
    let baseline_human = mean(
        blocks
            .iter()
            .map(|block| block.human_time_baseline)
            .collect(),
    );
    let candidate_human = mean(
        blocks
            .iter()
            .map(|block| block.human_time_candidate)
            .collect(),
    );
    let baseline_recall = mean(
        blocks
            .iter()
            .map(|block| block.recall_baseline)
            .collect(),
    );
    let candidate_recall = mean(
        blocks
            .iter()
            .map(|block| block.recall_candidate)
            .collect(),
    );

    let relative_reduction = |baseline: f64, candidate: f64| {
        if baseline <= f64::EPSILON {
            None
        } else {
            Some((baseline - candidate) / baseline)
        }
    };

    let point = PairedPointEstimates {
        baseline_false_review_requests_per_hour: baseline_false,
        candidate_false_review_requests_per_hour: candidate_false,
        false_review_relative_reduction: relative_reduction(baseline_false, candidate_false),
        baseline_human_review_seconds_per_hour: baseline_human,
        candidate_human_review_seconds_per_hour: candidate_human,
        human_review_relative_reduction: relative_reduction(baseline_human, candidate_human),
        baseline_mean_recall: baseline_recall,
        candidate_mean_recall: candidate_recall,
        recall_difference: candidate_recall - baseline_recall,
    };

    let mut rng = DeterministicRng::new(prereg.bootstrap_seed);
    let mut false_reductions = Vec::with_capacity(prereg.bootstrap_resamples);
    let mut human_reductions = Vec::with_capacity(prereg.bootstrap_resamples);
    let mut recall_differences = Vec::with_capacity(prereg.bootstrap_resamples);

    for _ in 0..prereg.bootstrap_resamples {
        let sample = (0..blocks.len())
            .map(|_| &blocks[rng.index(blocks.len())])
            .collect::<Vec<_>>();
        let count = sample.len() as f64;

        let mean_of = |get: fn(&PairedBlock) -> f64| {
            sample.iter().map(|block| get(block)).sum::<f64>() / count
        };

        let sampled_false_base = mean_of(|block| block.false_review_baseline);
        let sampled_false_candidate = mean_of(|block| block.false_review_candidate);
        if sampled_false_base > f64::EPSILON {
            false_reductions.push(
                (sampled_false_base - sampled_false_candidate) / sampled_false_base,
            );
        }

        let sampled_human_base = mean_of(|block| block.human_time_baseline);
        let sampled_human_candidate = mean_of(|block| block.human_time_candidate);
        if sampled_human_base > f64::EPSILON {
            human_reductions.push(
                (sampled_human_base - sampled_human_candidate) / sampled_human_base,
            );
        }

        let sampled_recall_base = mean_of(|block| block.recall_baseline);
        let sampled_recall_candidate = mean_of(|block| block.recall_candidate);
        recall_differences.push(sampled_recall_candidate - sampled_recall_base);
    }

    let alpha = (1_000 - prereg.confidence_level_permille) as f64 / 2_000.0;
    let false_interval = interval(&mut false_reductions, alpha);
    let human_interval = interval(&mut human_reductions, alpha);
    let recall_interval = interval(&mut recall_differences, alpha);

    let target = prereg.relative_reduction_target_micros as f64 / 1_000_000.0;
    let recall_margin = prereg.recall_non_inferiority_margin_micros as f64 / 1_000_000.0;
    let hypothesis_supported = false_interval
        .lower
        .is_some_and(|lower| lower >= target)
        && human_interval
            .lower
            .is_some_and(|lower| lower >= target)
        && recall_interval
            .lower
            .is_some_and(|lower| lower >= -recall_margin);

    BootstrapOutput {
        point,
        intervals: PairedConfidenceIntervals {
            false_review_relative_reduction: false_interval,
            human_review_relative_reduction: human_interval,
            recall_difference: recall_interval,
            confidence_level_permille: prereg.confidence_level_permille,
        },
        hypothesis_supported,
    }
}

fn interval(values: &mut Vec<f64>, alpha: f64) -> MetricInterval {
    if values.is_empty() {
        return MetricInterval {
            lower: None,
            upper: None,
            valid_resamples: 0,
        };
    }
    values.sort_by(f64::total_cmp);
    let lower = nearest_rank_index(alpha, values.len());
    let upper = nearest_rank_index(1.0 - alpha, values.len());
    MetricInterval {
        lower: values.get(lower).copied(),
        upper: values.get(upper).copied(),
        valid_resamples: values.len(),
    }
}

fn nearest_rank_index(probability: f64, len: usize) -> usize {
    let rank = ((probability * len as f64).ceil() as usize).max(1);
    rank.saturating_sub(1).min(len - 1)
}

fn human_review_seconds(
    case_results: &[gcpn_track_a_replay_evaluator::CaseResult],
) -> f64 {
    case_results
        .iter()
        .filter_map(|result| result.human_review_seconds)
        .map(|seconds| seconds as f64)
        .sum()
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 96
        && value.bytes().all(|byte| {
            byte.is_ascii_uppercase()
                || byte.is_ascii_digit()
                || byte == b'_'
                || byte == b'-'
        })
}

fn study_evidence_name(tier: StudyEvidenceTier) -> &'static str {
    match tier {
        StudyEvidenceTier::SyntheticPreview => "SYNTHETIC_PREVIEW",
        StudyEvidenceTier::ControlledReplay => "CONTROLLED_REPLAY",
    }
}

fn unit_type_name(unit_type: IndependentUnitType) -> &'static str {
    match unit_type {
        IndependentUnitType::SiteDay => "SITE_DAY",
        IndependentUnitType::FailureEpoch => "FAILURE_EPOCH",
    }
}

struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0x9e3779b97f4a7c15
            } else {
                seed
            },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut value = self.state;
        value ^= value >> 12;
        value ^= value << 25;
        value ^= value >> 27;
        self.state = value;
        value.wrapping_mul(0x2545f4914f6cdd1d)
    }

    fn index(&mut self, upper: usize) -> usize {
        ((u128::from(self.next_u64()) * upper as u128) >> 64) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    const FIXTURE: &str = include_str!("../fixtures/paired_study_synthetic.json");

    fn fixture_value() -> Value {
        serde_json::from_str(FIXTURE).expect("study fixture parses")
    }

    fn evaluate_value(value: Value) -> StudyReport {
        evaluate(value.to_string().as_bytes())
    }

    #[test]
    fn byte_pinned_synthetic_study_is_preview_only() {
        let report = evaluate_frozen_fixture(FIXTURE.as_bytes());
        assert_eq!(report.status, "SYNTHETIC_PREVIEW", "{:?}", report.reasons);
        assert_eq!(report.independent_units_observed, 8);
        assert_eq!(report.independent_units_required, 30);
        assert!(!report.inference_permitted);
        assert_eq!(report.hypothesis_supported, None);
        assert!(report.point_estimates.is_some());
        assert!(report.confidence_intervals.is_some());
    }

    #[test]
    fn exact_fixture_digest_mismatch_is_rejected() {
        let report = evaluate_with_expected_sha256(
            FIXTURE.as_bytes(),
            Some("0000000000000000000000000000000000000000000000000000000000000000"),
        );
        assert_eq!(report.status, "REJECTED");
        assert_eq!(report.reasons, vec!["study_input_digest_mismatch"]);
    }

    #[test]
    fn unknown_root_field_is_rejected() {
        let changed = FIXTURE.replace(
            "\"study_id\": \"SYNTHETIC-PAIRED-STUDY-001\"",
            "\"study_id\": \"SYNTHETIC-PAIRED-STUDY-001\", \"unknown\": true",
        );
        let report = evaluate(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.reasons[0].starts_with("strict_study_json_decode:"));
    }

    #[test]
    fn duplicate_root_member_is_rejected() {
        let changed = FIXTURE.replace(
            "\"study_id\": \"SYNTHETIC-PAIRED-STUDY-001\"",
            "\"study_id\": \"SYNTHETIC-PAIRED-STUDY-001\", \"study_id\": \"SYNTHETIC-PAIRED-STUDY-001\"",
        );
        let report = evaluate(changed.as_bytes());
        assert_eq!(report.status, "REJECTED");
        assert!(report.reasons[0].starts_with("strict_study_json_decode:"));
    }

    #[test]
    fn duplicate_independent_unit_id_is_rejected() {
        let mut value = fixture_value();
        value["blocks"][1]["independent_unit_id"] = value["blocks"][0]["independent_unit_id"].clone();
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert!(report.reasons[0].starts_with("duplicate_independent_unit_id:"));
    }

    #[test]
    fn duplicate_block_bundle_digest_is_rejected() {
        let mut value = fixture_value();
        value["blocks"][1]["run_bundle_sha256"] = value["blocks"][0]["run_bundle_sha256"].clone();
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert!(report.reasons[0].starts_with("duplicate_run_bundle_digest:"));
    }

    #[test]
    fn controlled_replay_without_independence_receipt_is_blocked() {
        let mut value = fixture_value();
        value["evidence_tier"] = json!("CONTROLLED_REPLAY");
        let report = evaluate_value(value);
        assert_eq!(report.status, "BLOCKED");
        assert_eq!(
            report.reasons,
            vec!["independent_unit_assessment_receipt_missing_or_invalid"]
        );
    }

    #[test]
    fn synthetic_preview_never_becomes_inferential_even_above_floor() {
        assert!(!inference_permitted(
            StudyEvidenceTier::SyntheticPreview,
            64,
            CONTROLLED_MINIMUM_INDEPENDENT_UNITS
        ));
        assert!(!inference_permitted(
            StudyEvidenceTier::ControlledReplay,
            29,
            CONTROLLED_MINIMUM_INDEPENDENT_UNITS
        ));
        assert!(inference_permitted(
            StudyEvidenceTier::ControlledReplay,
            30,
            CONTROLLED_MINIMUM_INDEPENDENT_UNITS
        ));
    }

    #[test]
    fn study_requires_starter_minimum_of_thirty() {
        let mut value = fixture_value();
        value["preregistration"]["minimum_independent_units"] = json!(12);
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert_eq!(
            report.reasons,
            vec!["minimum_independent_units_below_starter_floor"]
        );
    }

    #[test]
    fn invalid_confidence_level_is_rejected() {
        let mut value = fixture_value();
        value["preregistration"]["confidence_level_permille"] = json!(850);
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert_eq!(report.reasons, vec!["confidence_level_out_of_range"]);
    }

    #[test]
    fn invalid_resample_count_is_rejected() {
        let mut value = fixture_value();
        value["preregistration"]["bootstrap_resamples"] = json!(100);
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert_eq!(report.reasons, vec!["bootstrap_resamples_out_of_range"]);
    }

    #[test]
    fn wrong_nested_bundle_digest_is_rejected() {
        let mut value = fixture_value();
        value["blocks"][0]["run_bundle_sha256"] = json!(
            "9999999999999999999999999999999999999999999999999999999999999999"
        );
        let report = evaluate_value(value);
        assert_eq!(report.status, "REJECTED");
        assert!(report.reasons[0].starts_with("track_a_block_rejected:"));
    }

    #[test]
    fn missing_candidate_pipeline_is_incomparable() {
        let mut value = fixture_value();
        let block_json = value["blocks"][0]["run_bundle_json"].as_str().unwrap();
        let mut child: Value = serde_json::from_str(block_json).unwrap();
        child["pipelines"].as_array_mut().unwrap().pop();
        let child_string = child.to_string();
        value["blocks"][0]["run_bundle_sha256"] = json!(sha256_hex(child_string.as_bytes()));
        value["blocks"][0]["run_bundle_json"] = json!(child_string);
        let report = evaluate_value(value);
        assert_eq!(report.status, "INCOMPARABLE");
        assert!(report.reasons[0].starts_with("candidate_pipeline_missing:"));
    }

    #[test]
    fn event_set_mismatch_within_block_is_incomparable() {
        let mut value = fixture_value();
        let block_json = value["blocks"][0]["run_bundle_json"].as_str().unwrap();
        let mut child: Value = serde_json::from_str(block_json).unwrap();
        child["pipelines"][0]["event_set_sha256"] =
            json!("9999999999999999999999999999999999999999999999999999999999999999");
        let child_string = child.to_string();
        value["blocks"][0]["run_bundle_sha256"] = json!(sha256_hex(child_string.as_bytes()));
        value["blocks"][0]["run_bundle_json"] = json!(child_string);
        let report = evaluate_value(value);
        assert_eq!(report.status, "INCOMPARABLE");
        assert!(report.reasons[0].starts_with("track_a_block_incomparable:"));
    }

    #[test]
    fn privacy_violation_in_any_child_block_blocks_whole_study() {
        let mut value = fixture_value();
        let block_json = value["blocks"][3]["run_bundle_json"].as_str().unwrap();
        let mut child: Value = serde_json::from_str(block_json).unwrap();
        child["pipelines"][2]["safety_boundary"]["privacy_violation_count"] = json!(1);
        let child_string = child.to_string();
        value["blocks"][3]["run_bundle_sha256"] = json!(sha256_hex(child_string.as_bytes()));
        value["blocks"][3]["run_bundle_json"] = json!(child_string);
        let report = evaluate_value(value);
        assert_eq!(report.status, "BLOCKED");
        assert!(report.reasons[0].starts_with("track_a_block_blocked:"));
    }

    #[test]
    fn paired_bootstrap_is_deterministic_for_fixed_seed() {
        let report_a = evaluate_frozen_fixture(FIXTURE.as_bytes());
        let report_b = evaluate_frozen_fixture(FIXTURE.as_bytes());
        assert_eq!(
            serde_json::to_string(&report_a).unwrap(),
            serde_json::to_string(&report_b).unwrap()
        );
    }

    #[test]
    fn paired_recall_difference_is_candidate_minus_baseline() {
        let report = evaluate_frozen_fixture(FIXTURE.as_bytes());
        let point = report.point_estimates.unwrap();
        assert!((point.recall_difference - 0.25).abs() < 1e-12);
    }

    #[test]
    fn zero_baseline_false_review_rate_has_no_relative_reduction() {
        let mut value = fixture_value();
        for block in value["blocks"].as_array_mut().unwrap() {
            let block_json = block["run_bundle_json"].as_str().unwrap();
            let mut child: Value = serde_json::from_str(block_json).unwrap();
            let site = child["pipelines"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find(|pipeline| pipeline["pipeline_id"] == "REFERENCE_SITE_WORKFLOW")
                .unwrap();
            for result in site["case_results"].as_array_mut().unwrap() {
                if result["case_id"] == "TAR-004" {
                    result["disposition"] = json!("NO_REVIEW");
                    result["review_requests"] = json!(0);
                    result["review_latency_ms"] = Value::Null;
                    result["human_review_seconds"] = Value::Null;
                }
            }
            let serialized = child.to_string();
            block["run_bundle_sha256"] = json!(sha256_hex(serialized.as_bytes()));
            block["run_bundle_json"] = json!(serialized);
        }
        let report = evaluate_value(value);
        assert_eq!(report.status, "SYNTHETIC_PREVIEW");
        assert_eq!(
            report
                .point_estimates
                .unwrap()
                .false_review_relative_reduction,
            None
        );
    }

    #[test]
    fn changed_operator_policy_between_blocks_is_incomparable() {
        let mut value = fixture_value();
        let block_json = value["blocks"][1]["run_bundle_json"].as_str().unwrap();
        let mut child: Value = serde_json::from_str(block_json).unwrap();
        child["operator_policy_sha256"] =
            json!("9999999999999999999999999999999999999999999999999999999999999999");
        for pipeline in child["pipelines"].as_array_mut().unwrap() {
            pipeline["operator_policy_sha256"] = child["operator_policy_sha256"].clone();
        }
        let serialized = child.to_string();
        value["blocks"][1]["run_bundle_sha256"] = json!(sha256_hex(serialized.as_bytes()));
        value["blocks"][1]["run_bundle_json"] = json!(serialized);
        let report = evaluate_value(value);
        assert_eq!(report.status, "INCOMPARABLE");
        assert!(report.reasons[0].starts_with("operator_policy_changed_between_blocks:"));
    }

    #[test]
    fn controlled_child_without_review_receipts_is_blocked() {
        let mut value = fixture_value();
        value["evidence_tier"] = json!("CONTROLLED_REPLAY");
        value["independence_assessment_receipt_sha256"] = json!(
            "9999999999999999999999999999999999999999999999999999999999999999"
        );
        let block_json = value["blocks"][0]["run_bundle_json"].as_str().unwrap();
        let mut child: Value = serde_json::from_str(block_json).unwrap();
        child["evidence_tier"] = json!("CONTROLLED_REPLAY");
        child["simulation_only"] = json!(false);
        let serialized = child.to_string();
        value["blocks"][0]["run_bundle_sha256"] = json!(sha256_hex(serialized.as_bytes()));
        value["blocks"][0]["run_bundle_json"] = json!(serialized);
        let report = evaluate_value(value);
        assert_eq!(report.status, "BLOCKED");
        assert!(report.reasons[0].starts_with("track_a_block_blocked:"));
    }

    #[test]
    fn controlled_children_with_review_receipts_but_under_floor_are_insufficient() {
        let mut value = fixture_value();
        value["evidence_tier"] = json!("CONTROLLED_REPLAY");
        value["independence_assessment_receipt_sha256"] = json!(
            "9999999999999999999999999999999999999999999999999999999999999999"
        );
        for block in value["blocks"].as_array_mut().unwrap() {
            let block_json = block["run_bundle_json"].as_str().unwrap();
            let mut child: Value = serde_json::from_str(block_json).unwrap();
            child["evidence_tier"] = json!("CONTROLLED_REPLAY");
            child["simulation_only"] = json!(false);
            child["ground_truth_adjudication_receipt_sha256"] = json!(
                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            );
            child["privacy_review_receipt_sha256"] = json!(
                "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
            );
            child["authorization_review_receipt_sha256"] = json!(
                "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"
            );
            let serialized = child.to_string();
            block["run_bundle_sha256"] = json!(sha256_hex(serialized.as_bytes()));
            block["run_bundle_json"] = json!(serialized);
        }
        let report = evaluate_value(value);
        assert_eq!(report.status, "INSUFFICIENT_INDEPENDENT_UNITS");
        assert!(!report.inference_permitted);
        assert_eq!(report.independent_units_observed, 8);
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

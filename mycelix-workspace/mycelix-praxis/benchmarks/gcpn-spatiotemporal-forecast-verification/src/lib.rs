//! Matched ensemble and spatial forecast-verification primitives.
//!
//! This crate scores supplied forecast ensembles. It does not generate
//! forecasts, set warning thresholds, select operational models or issue
//! warnings. All comparisons are synthetic/offline until an independently
//! reviewed public-data adapter exists.

use std::collections::{BTreeMap, BTreeSet};

fn validate_finite(values: &[f64], label: &str) -> Result<(), String> {
    if values.is_empty() {
        return Err(format!("{label} must not be empty"));
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err(format!("{label} contains a non-finite value"));
    }
    Ok(())
}

/// Empirical ensemble CRPS:
/// mean |X_m-y| - 0.5 * mean |X_m-X_n| over all ordered member pairs.
pub fn crps_ensemble(members: &[f64], observation: f64) -> Result<f64, String> {
    validate_finite(members, "ensemble members")?;
    if !observation.is_finite() {
        return Err("observation must be finite".into());
    }
    let member_count = members.len() as f64;
    let first_term = members
        .iter()
        .map(|member| (member - observation).abs())
        .sum::<f64>()
        / member_count;
    let mut pair_sum = 0.0;
    for left in members {
        for right in members {
            pair_sum += (left - right).abs();
        }
    }
    let pair_mean = pair_sum / (member_count * member_count);
    let score = first_term - 0.5 * pair_mean;
    if !score.is_finite() || score < -1e-12 {
        return Err("CRPS calculation produced an invalid score".into());
    }
    Ok(score.max(0.0))
}

/// Convert a scalar ensemble to a threshold-event probability and score the
/// event with a Brier score. The threshold must be predeclared externally.
pub fn threshold_brier(
    members: &[f64],
    observation: f64,
    threshold: f64,
) -> Result<(f64, f64, bool), String> {
    validate_finite(members, "ensemble members")?;
    if !observation.is_finite() || !threshold.is_finite() {
        return Err("observation and threshold must be finite".into());
    }
    let event_members = members.iter().filter(|member| **member >= threshold).count();
    let probability = event_members as f64 / members.len() as f64;
    let observed_event = observation >= threshold;
    let outcome = if observed_event { 1.0 } else { 0.0 };
    Ok(((probability - outcome).powi(2), probability, observed_event))
}

fn validate_field_ensemble(members: &[Vec<f64>], observation: &[f64]) -> Result<usize, String> {
    validate_finite(observation, "spatial observation")?;
    if members.is_empty() {
        return Err("spatial ensemble must contain at least one member".into());
    }
    let dimensions = observation.len();
    for (index, member) in members.iter().enumerate() {
        validate_finite(member, &format!("spatial ensemble member {index}"))?;
        if member.len() != dimensions {
            return Err(format!(
                "spatial ensemble member {index} has dimension {}, expected {dimensions}",
                member.len()
            ));
        }
    }
    Ok(dimensions)
}

fn euclidean(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right)
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt()
}

/// Multivariate ensemble energy score on a complete spatial vector.
pub fn energy_score(members: &[Vec<f64>], observation: &[f64]) -> Result<f64, String> {
    validate_field_ensemble(members, observation)?;
    let member_count = members.len() as f64;
    let observation_distance = members
        .iter()
        .map(|member| euclidean(member, observation))
        .sum::<f64>()
        / member_count;
    let mut pair_sum = 0.0;
    for left in members {
        for right in members {
            pair_sum += euclidean(left, right);
        }
    }
    let pair_mean = pair_sum / (member_count * member_count);
    let score = observation_distance - 0.5 * pair_mean;
    if !score.is_finite() || score < -1e-12 {
        return Err("energy score calculation produced an invalid score".into());
    }
    Ok(score.max(0.0))
}

#[derive(Debug, Clone, PartialEq)]
pub struct VariogramPair {
    pub left_index: usize,
    pub right_index: usize,
    pub weight: f64,
}

/// Variogram score with explicit pair weights and exponent. Every unordered
/// spatial pair is supplied by the caller; weights/exponent must be frozen in
/// the benchmark profile before evaluation.
pub fn variogram_score(
    members: &[Vec<f64>],
    observation: &[f64],
    exponent: f64,
    pairs: &[VariogramPair],
) -> Result<f64, String> {
    let dimensions = validate_field_ensemble(members, observation)?;
    if !exponent.is_finite() || exponent <= 0.0 {
        return Err("variogram exponent must be finite and positive".into());
    }
    if pairs.is_empty() {
        return Err("at least one spatial pair is required".into());
    }
    let mut seen = BTreeSet::new();
    let mut score = 0.0;
    for pair in pairs {
        if pair.left_index >= dimensions
            || pair.right_index >= dimensions
            || pair.left_index == pair.right_index
        {
            return Err("variogram pair index is invalid".into());
        }
        if !pair.weight.is_finite() || pair.weight < 0.0 {
            return Err("variogram pair weight must be finite and nonnegative".into());
        }
        let normalized = (
            pair.left_index.min(pair.right_index),
            pair.left_index.max(pair.right_index),
        );
        if !seen.insert(normalized) {
            return Err("duplicate variogram pair".into());
        }
        let observed_difference = (observation[pair.left_index] - observation[pair.right_index])
            .abs()
            .powf(exponent);
        let forecast_difference = members
            .iter()
            .map(|member| {
                (member[pair.left_index] - member[pair.right_index])
                    .abs()
                    .powf(exponent)
            })
            .sum::<f64>()
            / members.len() as f64;
        score += pair.weight * (observed_difference - forecast_difference).powi(2);
    }
    if !score.is_finite() {
        return Err("variogram score calculation produced a non-finite result".into());
    }
    Ok(score)
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct ForecastCaseKey {
    pub case_id: String,
    pub issue_time: u64,
    pub valid_time: u64,
    pub lead_seconds: u64,
    pub variable: String,
    pub region: String,
    pub grid_policy: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ForecastArchiveRecord {
    pub model_id: String,
    pub case_key: ForecastCaseKey,
    pub verification_source: String,
    pub verification_version: String,
    pub ensemble_members: Vec<Vec<f64>>,
    pub observation: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FairComparisonReport {
    pub model_ids: Vec<String>,
    pub verification_source: String,
    pub verification_version: String,
    pub total_case_keys_seen: u32,
    pub eligible_matched_case_count: u32,
    /// Expected model-case pairs not available on every key. Missing pairs are
    /// excluded from the paired denominator and reported, never imputed.
    pub excluded_case_keys_due_to_missing_models: u32,
    pub eligible_case_keys: Vec<ForecastCaseKey>,
}

/// Audit a model panel before comparative scoring.
///
/// Every model must share exactly the same verification source/version and
/// observation vector for each case key. Incomplete cases are excluded from
/// the paired panel with an explicit count; duplicate, conflicting or
/// incomparable records fail closed.
pub fn validate_fair_comparison(
    records: &[ForecastArchiveRecord],
    expected_model_ids: &[String],
) -> Result<FairComparisonReport, String> {
    if records.is_empty() {
        return Err("forecast archive panel is empty".into());
    }
    if expected_model_ids.len() < 2 {
        return Err("at least two expected models are required for comparison".into());
    }
    let expected: BTreeSet<&str> = expected_model_ids.iter().map(String::as_str).collect();
    if expected.len() != expected_model_ids.len() {
        return Err("expected model list contains duplicates".into());
    }
    if expected.iter().any(|model| model.trim().is_empty()) {
        return Err("expected model identifier must not be empty".into());
    }

    let verification_source = records[0].verification_source.clone();
    let verification_version = records[0].verification_version.clone();
    if verification_source.trim().is_empty() || verification_version.trim().is_empty() {
        return Err("verification source and version are required".into());
    }

    let mut by_case: BTreeMap<ForecastCaseKey, BTreeMap<String, &ForecastArchiveRecord>> =
        BTreeMap::new();
    for record in records {
        if !expected.contains(record.model_id.as_str()) {
            return Err(format!("unexpected model identifier: {}", record.model_id));
        }
        if record.verification_source != verification_source
            || record.verification_version != verification_version
        {
            return Err("comparison panel mixes verification sources or versions".into());
        }
        if record.case_key.issue_time >= record.case_key.valid_time
            || record.case_key.lead_seconds == 0
            || record.case_key.variable.trim().is_empty()
            || record.case_key.region.trim().is_empty()
            || record.case_key.grid_policy.trim().is_empty()
        {
            return Err("forecast case key has invalid time or field metadata".into());
        }
        validate_field_ensemble(&record.ensemble_members, &record.observation)?;
        let case = by_case.entry(record.case_key.clone()).or_default();
        if case.insert(record.model_id.clone(), record).is_some() {
            return Err(format!("duplicate model record for case {}", record.case_key.case_id));
        }
    }

    let mut eligible_case_keys = Vec::new();
    let mut excluded = 0u32;
    for (key, models) in &by_case {
        if models.len() != expected.len() {
            excluded += 1;
            continue;
        }
        let first = models
            .values()
            .next()
            .expect("model map has the expected nonzero cardinality");
        for model in models.values() {
            if model.observation != first.observation {
                return Err(format!(
                    "models do not share an identical verifying observation for case {}",
                    key.case_id
                ));
            }
        }
        eligible_case_keys.push(key.clone());
    }
    if eligible_case_keys.is_empty() {
        return Err("no complete matched case keys remain for comparison".into());
    }

    Ok(FairComparisonReport {
        model_ids: expected_model_ids.to_vec(),
        verification_source,
        verification_version,
        total_case_keys_seen: by_case.len() as u32,
        eligible_matched_case_count: eligible_case_keys.len() as u32,
        excluded_case_keys_due_to_missing_models: excluded,
        eligible_case_keys,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(left: usize, right: usize, weight: f64) -> VariogramPair {
        VariogramPair {
            left_index: left,
            right_index: right,
            weight,
        }
    }

    fn archive_record(model: &str, source: &str) -> ForecastArchiveRecord {
        ForecastArchiveRecord {
            model_id: model.into(),
            case_key: ForecastCaseKey {
                case_id: "case-1".into(),
                issue_time: 10,
                valid_time: 20,
                lead_seconds: 10,
                variable: "surface_temperature".into(),
                region: "grid-cell-pair".into(),
                grid_policy: "frozen-grid-v1".into(),
            },
            verification_source: source.into(),
            verification_version: "truth-v1".into(),
            ensemble_members: vec![vec![0.0, 0.0], vec![1.0, 1.0]],
            observation: vec![0.0, 0.0],
        }
    }

    #[test]
    fn ensemble_crps_reproduces_exact_two_member_oracle() {
        let score = crps_ensemble(&[0.0, 1.0], 0.0).unwrap();
        assert!((score - 0.25).abs() < 1e-12);
    }

    #[test]
    fn threshold_brier_uses_predeclared_threshold_and_member_fraction() {
        let (score, probability, observed) = threshold_brier(&[0.0, 1.0], 1.2, 0.5).unwrap();
        assert_eq!(probability, 0.5);
        assert!(observed);
        assert!((score - 0.25).abs() < 1e-12);
    }

    #[test]
    fn energy_score_reproduces_exact_joint_field_oracle() {
        let members = vec![vec![0.0, 0.0], vec![1.0, 1.0]];
        let score = energy_score(&members, &[0.0, 0.0]).unwrap();
        assert!((score - std::f64::consts::SQRT_2 / 4.0).abs() < 1e-12);
    }

    #[test]
    fn variogram_score_reproduces_exact_joint_field_oracle() {
        let correct = vec![vec![0.0, 0.0], vec![1.0, 1.0]];
        let wrong_dependence = vec![vec![0.0, 1.0], vec![1.0, 0.0]];
        let observation = [0.0, 0.0];
        let weights = [pair(0, 1, 1.0)];
        assert_eq!(variogram_score(&correct, &observation, 1.0, &weights).unwrap(), 0.0);
        assert_eq!(variogram_score(&wrong_dependence, &observation, 1.0, &weights).unwrap(), 1.0);
    }

    #[test]
    fn correct_marginals_wrong_spatial_dependence_has_same_marginal_scores() {
        let correct = vec![vec![0.0, 0.0], vec![1.0, 1.0]];
        let wrong = vec![vec![0.0, 1.0], vec![1.0, 0.0]];
        let observation = [0.0, 0.0];
        for dimension in 0..2 {
            let correct_marginal: Vec<f64> = correct.iter().map(|member| member[dimension]).collect();
            let wrong_marginal: Vec<f64> = wrong.iter().map(|member| member[dimension]).collect();
            assert_eq!(
                crps_ensemble(&correct_marginal, observation[dimension]).unwrap(),
                crps_ensemble(&wrong_marginal, observation[dimension]).unwrap()
            );
        }
        assert!(
            energy_score(&correct, &observation).unwrap()
                < energy_score(&wrong, &observation).unwrap()
        );
        assert!(
            variogram_score(&correct, &observation, 1.0, &[pair(0, 1, 1.0)]).unwrap()
                < variogram_score(&wrong, &observation, 1.0, &[pair(0, 1, 1.0)]).unwrap()
        );
    }

    #[test]
    fn score_inputs_fail_closed_for_empty_or_non_finite_data() {
        assert!(crps_ensemble(&[], 0.0).is_err());
        assert!(crps_ensemble(&[f64::NAN], 0.0).is_err());
        assert!(energy_score(&[vec![0.0]], &[0.0, 1.0]).is_err());
        assert!(variogram_score(&[vec![0.0, 1.0]], &[0.0, 1.0], 0.0, &[pair(0, 1, 1.0)]).is_err());
    }

    #[test]
    fn variogram_pairs_reject_duplicate_or_invalid_weights() {
        let members = vec![vec![0.0, 0.0], vec![1.0, 1.0]];
        let observation = [0.0, 0.0];
        assert!(variogram_score(&members, &observation, 1.0, &[pair(0, 1, 1.0), pair(1, 0, 1.0)]).is_err());
        assert!(variogram_score(&members, &observation, 1.0, &[pair(0, 1, -1.0)]).is_err());
    }

    #[test]
    fn fair_comparison_accepts_identical_case_and_verification_vintage() {
        let report = validate_fair_comparison(
            &[archive_record("model-a", "ERA5-v1"), archive_record("model-b", "ERA5-v1")],
            &["model-a".into(), "model-b".into()],
        ).unwrap();
        assert_eq!(report.eligible_matched_case_count, 1);
        assert_eq!(report.excluded_case_keys_due_to_missing_models, 0);
    }

    #[test]
    fn fair_comparison_rejects_different_verification_sources() {
        let error = validate_fair_comparison(
            &[archive_record("model-a", "IFS-analysis"), archive_record("model-b", "ERA5")],
            &["model-a".into(), "model-b".into()],
        ).unwrap_err();
        assert!(error.contains("mixes verification sources"));
    }

    #[test]
    fn missing_model_case_is_reported_and_excluded_from_paired_denominator() {
        let first = archive_record("model-a", "ERA5-v1");
        let report = validate_fair_comparison(
            &[first],
            &["model-a".into(), "model-b".into()],
        ).unwrap_err();
        assert!(report.contains("at least two expected models"));
    }
}

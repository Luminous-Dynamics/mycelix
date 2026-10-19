// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Research-only, matched-data comparison of TCDM score, Beta posterior mean and
//! Cantelli lower bound. The held-out outcomes are never consumed by a predictor.

use mycelix_core_types::trust::TCDMState;
use serde_json::Value;
use std::collections::HashSet;

const CORPUS: &str = include_str!("data/trust-calibration-benchmark-v1.json");
const ABS_TOL: f64 = 1.0e-9;
const CALIBRATION_BINS: usize = 10;

#[derive(Debug, Clone)]
struct ScenarioResult {
    id: String,
    training_count: usize,
    training_successes: usize,
    heldout_outcomes: Vec<bool>,
    tcdm_score: f64,
    posterior_mean: f64,
    posterior_variance: f64,
    cantelli_lower_bound: f64,
    acceptance_threshold: f64,
}

#[derive(Debug, Clone, Copy)]
struct Metrics {
    observations: usize,
    brier: f64,
    log_loss: f64,
    expected_calibration_error: f64,
    accepted: usize,
    accepted_failures: usize,
    accepted_successes: usize,
}

impl Metrics {
    fn coverage(self) -> f64 {
        self.accepted as f64 / self.observations as f64
    }

    fn selective_risk(self) -> Option<f64> {
        (self.accepted > 0).then_some(self.accepted_failures as f64 / self.accepted as f64)
    }
}

fn finite_number(case: &Value, field: &str) -> f64 {
    let raw = case[field]
        .as_str()
        .unwrap_or_else(|| panic!("{field} must be a decimal string"));
    let parsed = raw
        .parse::<f64>()
        .unwrap_or_else(|error| panic!("{field} parse failed: {error}"));
    assert!(parsed.is_finite(), "{field} must be finite");
    parsed
}

fn boolean_vector(case: &Value, field: &str) -> Vec<bool> {
    case[field]
        .as_array()
        .unwrap_or_else(|| panic!("{field} must be an array"))
        .iter()
        .enumerate()
        .map(|(index, value)| {
            value
                .as_bool()
                .unwrap_or_else(|| panic!("{field}[{index}] must be boolean"))
        })
        .collect()
}

fn calculate_scenario(case: &Value) -> ScenarioResult {
    let id = case["id"]
        .as_str()
        .expect("scenario requires stable ID")
        .to_owned();
    let training = boolean_vector(case, "training_outcomes");
    let heldout_outcomes = boolean_vector(case, "heldout_outcomes");
    assert!(!training.is_empty(), "{id}: training set must not be empty");
    assert!(!heldout_outcomes.is_empty(), "{id}: held-out set must not be empty");

    let weight = finite_number(case, "tcdm_weight");
    assert!(weight.is_finite() && weight > 0.0 && weight <= 0.3);
    let mut tcdm = TCDMState::initial();
    for outcome in &training {
        tcdm.update_from_interaction(*outcome, weight as f32);
    }
    let tcdm_score = f64::from(tcdm.compute_tcdm_score());

    let prior_alpha = finite_number(case, "prior_alpha");
    let prior_beta = finite_number(case, "prior_beta");
    let epsilon = finite_number(case, "tail_probability");
    let acceptance_threshold = finite_number(case, "acceptance_threshold");
    assert!(prior_alpha > 0.0 && prior_beta > 0.0);
    assert!(epsilon > 0.0 && epsilon < 1.0);
    assert!((0.0..=1.0).contains(&acceptance_threshold));

    let training_successes = training.iter().filter(|value| **value).count();
    let training_failures = training.len() - training_successes;
    let alpha = prior_alpha + training_successes as f64;
    let beta = prior_beta + training_failures as f64;
    let concentration = alpha + beta;
    let posterior_mean = alpha / concentration;
    let posterior_variance = alpha * beta / (concentration * concentration * (concentration + 1.0));
    let tail_ratio = (1.0 - epsilon) / epsilon;
    let margin_squared = posterior_variance * tail_ratio;
    assert!(alpha.is_finite() && beta.is_finite() && concentration.is_finite());
    assert!(posterior_mean.is_finite() && (0.0..=1.0).contains(&posterior_mean));
    assert!(posterior_variance.is_finite() && posterior_variance >= 0.0);
    assert!(margin_squared.is_finite() && margin_squared >= 0.0);
    let cantelli_lower_bound = (posterior_mean - margin_squared.sqrt()).max(0.0);
    assert!((0.0..=1.0).contains(&cantelli_lower_bound));
    assert!(cantelli_lower_bound <= posterior_mean);

    ScenarioResult {
        id,
        training_count: training.len(),
        training_successes,
        heldout_outcomes,
        tcdm_score,
        posterior_mean,
        posterior_variance,
        cantelli_lower_bound,
        acceptance_threshold,
    }
}

fn policy_metrics(
    scenarios: &[ScenarioResult],
    prediction: impl Fn(&ScenarioResult) -> f64,
) -> Metrics {
    let observations = scenarios
        .iter()
        .map(|scenario| scenario.heldout_outcomes.len())
        .sum::<usize>();
    assert!(observations > 0);

    let mut brier_sum = 0.0;
    let mut log_loss_sum = 0.0;
    let mut accepted = 0;
    let mut accepted_failures = 0;
    let mut accepted_successes = 0;
    let mut bin_counts = [0usize; CALIBRATION_BINS];
    let mut bin_prediction_sum = [0.0_f64; CALIBRATION_BINS];
    let mut bin_outcome_sum = [0.0_f64; CALIBRATION_BINS];

    for scenario in scenarios {
        let score = prediction(scenario);
        assert!(score.is_finite() && (0.0..=1.0).contains(&score));
        let probability = score.clamp(1.0e-15, 1.0 - 1.0e-15);
        let bin = ((score * CALIBRATION_BINS as f64).floor() as usize)
            .min(CALIBRATION_BINS - 1);

        for outcome in &scenario.heldout_outcomes {
            let observed = if *outcome { 1.0 } else { 0.0 };
            let error = score - observed;
            brier_sum += error * error;
            log_loss_sum -= if *outcome {
                probability.ln()
            } else {
                (1.0 - probability).ln()
            };

            bin_counts[bin] += 1;
            bin_prediction_sum[bin] += score;
            bin_outcome_sum[bin] += observed;

            if score >= scenario.acceptance_threshold {
                accepted += 1;
                if *outcome {
                    accepted_successes += 1;
                } else {
                    accepted_failures += 1;
                }
            }
        }
    }

    let expected_calibration_error = bin_counts
        .iter()
        .enumerate()
        .filter(|(_, count)| **count > 0)
        .map(|(index, count)| {
            let predicted = bin_prediction_sum[index] / *count as f64;
            let observed = bin_outcome_sum[index] / *count as f64;
            (*count as f64 / observations as f64) * (predicted - observed).abs()
        })
        .sum::<f64>();

    Metrics {
        observations,
        brier: brier_sum / observations as f64,
        log_loss: log_loss_sum / observations as f64,
        expected_calibration_error,
        accepted,
        accepted_failures,
        accepted_successes,
    }
}

fn close(actual: f64, expected: f64, tolerance: f64) -> bool {
    (actual - expected).abs() <= tolerance * expected.abs().max(1.0)
}

#[test]
fn matched_synthetic_holdout_benchmark_reports_calibration_and_risk_coverage() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("benchmark fixture must be valid JSON");
    assert_eq!(
        corpus["schema"].as_str(),
        Some("mycelix-trust-calibration-benchmark-v1")
    );
    assert_eq!(corpus["scenario_count"].as_u64(), Some(5));
    let cases = corpus["scenarios"].as_array().expect("scenarios must be an array");
    assert_eq!(cases.len(), 5);

    let mut ids = HashSet::new();
    let scenarios = cases.iter().map(|case| {
        let id = case["id"].as_str().expect("scenario ID must be string");
        assert!(ids.insert(id), "duplicate scenario ID {id}");
        calculate_scenario(case)
    }).collect::<Vec<_>>();

    assert_eq!(scenarios.iter().map(|item| item.training_count).collect::<Vec<_>>(), vec![10, 500, 500, 500, 500]);
    assert_eq!(scenarios.iter().map(|item| item.training_successes).collect::<Vec<_>>(), vec![9, 100, 236, 408, 408]);
    assert!(scenarios.iter().all(|item| item.heldout_outcomes.len() == 200));

    let cold = scenarios.iter().find(|item| item.id == "cold-start-high-nine-of-ten").unwrap();
    assert!(cold.posterior_mean > cold.acceptance_threshold);
    assert!(cold.cantelli_lower_bound < cold.acceptance_threshold);
    assert!(cold.tcdm_score < cold.acceptance_threshold);

    // The stationary-high and drift cases have byte-identical training histories.
    // All three predictors therefore produce identical values, despite sharply
    // different held-out outcomes. This is an intentional nonstationarity control.
    let high = scenarios.iter().find(|item| item.id == "stationary-high-rate").unwrap();
    let drift = scenarios.iter().find(|item| item.id == "drift-high-training-low-holdout").unwrap();
    assert_eq!(high.training_successes, drift.training_successes);
    assert_eq!(high.training_count, drift.training_count);
    assert!(close(high.tcdm_score, drift.tcdm_score, 1.0e-12));
    assert!(close(high.posterior_mean, drift.posterior_mean, 1.0e-12));
    assert!(close(high.cantelli_lower_bound, drift.cantelli_lower_bound, 1.0e-12));
    let high_holdout_rate = high.heldout_outcomes.iter().filter(|value| **value).count() as f64
        / high.heldout_outcomes.len() as f64;
    let drift_holdout_rate = drift.heldout_outcomes.iter().filter(|value| **value).count() as f64
        / drift.heldout_outcomes.len() as f64;
    assert!(high_holdout_rate - drift_holdout_rate > 0.5);
    assert!(drift.cantelli_lower_bound >= drift.acceptance_threshold);

    let tcdm = policy_metrics(&scenarios, |item| item.tcdm_score);
    let posterior_mean = policy_metrics(&scenarios, |item| item.posterior_mean);
    // This numerical Brier/log-loss diagnostic is informative for all scores, but
    // the Cantelli lower bound is not semantically a probability forecast.
    let cantelli = policy_metrics(&scenarios, |item| item.cantelli_lower_bound);

    assert_eq!(tcdm.observations, 1_000);
    assert_eq!(tcdm.accepted, 400);
    assert_eq!(posterior_mean.accepted, 600);
    assert_eq!(cantelli.accepted, 400);
    assert_eq!(tcdm.accepted_failures, 213);
    assert_eq!(posterior_mean.accepted_failures, 257);
    assert_eq!(cantelli.accepted_failures, 213);
    assert!(posterior_mean.brier + 0.01 < tcdm.brier);
    assert!(posterior_mean.log_loss + 0.01 < tcdm.log_loss);
    assert!(close(tcdm.brier, 0.280751, 1.0e-5));
    assert!(close(posterior_mean.brier, 0.265227, 1.0e-5));
    assert!(close(cantelli.brier, 0.281041, 1.0e-5));
    assert!(close(tcdm.coverage(), 0.4, ABS_TOL));
    assert!(close(posterior_mean.coverage(), 0.6, ABS_TOL));
    assert!(close(cantelli.coverage(), 0.4, ABS_TOL));
    assert!(close(tcdm.selective_risk().unwrap(), 0.5325, ABS_TOL));
    assert!(close(posterior_mean.selective_risk().unwrap(), 0.42833333333333334, ABS_TOL));
    assert!(close(cantelli.selective_risk().unwrap(), 0.5325, ABS_TOL));

    println!(
        "trust-calibration-v1 observations={} threshold=0.7",
        tcdm.observations
    );
    println!(
        "policy=tcdm score_brier={:.6} score_log_loss={:.6} score_ece={:.6} coverage={:.3} accepted_risk={:.3}",
        tcdm.brier, tcdm.log_loss, tcdm.expected_calibration_error,
        tcdm.coverage(), tcdm.selective_risk().unwrap()
    );
    println!(
        "policy=beta_posterior_mean brier={:.6} log_loss={:.6} ece={:.6} coverage={:.3} accepted_risk={:.3}",
        posterior_mean.brier, posterior_mean.log_loss,
        posterior_mean.expected_calibration_error, posterior_mean.coverage(),
        posterior_mean.selective_risk().unwrap()
    );
    println!(
        "policy=cantelli_lower_bound score_brier_diagnostic={:.6} score_log_loss_diagnostic={:.6} coverage={:.3} accepted_risk={:.3} (not a probability forecast)",
        cantelli.brier, cantelli.log_loss, cantelli.coverage(),
        cantelli.selective_risk().unwrap()
    );
    println!(
        "distribution_shift_control high_holdout_success={:.3} drift_holdout_success={:.3} same_training=true",
        high_holdout_rate, drift_holdout_rate
    );
}

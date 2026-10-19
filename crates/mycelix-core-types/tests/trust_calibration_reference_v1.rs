// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Research-only binary probability-score reference.
//!
//! This validates Brier/log-loss arithmetic for declared probabilities. It does
//! not calibrate TCDM scores or establish that trust/confidence values are
//! probabilities of any real outcome.

use serde_json::Value;
use std::collections::HashSet;

const CORPUS: &str = include_str!("data/trust-calibration-reference-v1.json");
const ABS_TOL: f64 = 1.0e-12;

#[derive(Debug, Clone, Copy, PartialEq)]
struct Scores {
    brier_score: f64,
    log_loss: f64,
    mean_probability: f64,
    observed_positive_rate: f64,
}

fn scalar(value: &Value, field: &str) -> Result<f64, String> {
    let raw = value
        .as_str()
        .ok_or_else(|| format!("{field}: expected a decimal string"))?;
    let parsed = raw
        .parse::<f64>()
        .map_err(|error| format!("{field}: invalid scalar {raw:?}: {error}"))?;
    if !parsed.is_finite() {
        return Err(format!("{field}: must be finite"));
    }
    Ok(parsed)
}

/// Compute proper probability scores for one finite binary-outcome batch.
/// p must be strictly inside (0, 1) because log-loss is infinite at either
/// boundary for an outcome assigned zero probability.
fn score(probabilities: &[f64], outcomes: &[u8]) -> Result<Scores, String> {
    if probabilities.is_empty() {
        return Err("sample must not be empty".to_owned());
    }
    if probabilities.len() != outcomes.len() {
        return Err("probability/outcome length mismatch".to_owned());
    }
    if outcomes.iter().any(|outcome| *outcome > 1) {
        return Err("binary outcomes must be 0 or 1".to_owned());
    }
    if probabilities
        .iter()
        .any(|probability| !probability.is_finite() || *probability <= 0.0 || *probability >= 1.0)
    {
        return Err("probabilities must be finite and strictly within (0, 1)".to_owned());
    }

    let count = probabilities.len() as f64;
    let mut brier_sum = 0.0_f64;
    let mut loss_sum = 0.0_f64;
    let mut probability_sum = 0.0_f64;
    let mut positive_count = 0_u64;

    for (probability, outcome) in probabilities.iter().zip(outcomes) {
        let target = f64::from(*outcome);
        let difference = probability - target;
        brier_sum += difference * difference;
        loss_sum -= if *outcome == 1 {
            probability.ln()
        } else {
            (-probability).ln_1p()
        };
        probability_sum += probability;
        positive_count += u64::from(*outcome);

        if !brier_sum.is_finite() || !loss_sum.is_finite() || !probability_sum.is_finite() {
            return Err("score accumulation became non-finite".to_owned());
        }
    }

    let scores = Scores {
        brier_score: brier_sum / count,
        log_loss: loss_sum / count,
        mean_probability: probability_sum / count,
        observed_positive_rate: positive_count as f64 / count,
    };
    if [
        scores.brier_score,
        scores.log_loss,
        scores.mean_probability,
        scores.observed_positive_rate,
    ]
    .iter()
    .any(|value| !value.is_finite())
    {
        return Err("final score became non-finite".to_owned());
    }
    Ok(scores)
}

fn evaluate_case(case: &Value) -> Result<Scores, String> {
    let probabilities_json = case["probabilities"]
        .as_array()
        .ok_or_else(|| "probabilities must be an array".to_owned())?;
    let outcomes_json = case["outcomes"]
        .as_array()
        .ok_or_else(|| "outcomes must be an array".to_owned())?;
    let probabilities = probabilities_json
        .iter()
        .enumerate()
        .map(|(index, value)| scalar(value, &format!("probabilities[{index}]")))
        .collect::<Result<Vec<_>, _>>()?;
    let outcomes = outcomes_json
        .iter()
        .enumerate()
        .map(|(index, value)| {
            value
                .as_u64()
                .filter(|outcome| *outcome <= 1)
                .map(|outcome| outcome as u8)
                .ok_or_else(|| format!("outcomes[{index}] must be unsigned 0 or 1"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    score(&probabilities, &outcomes)
}

fn corpus() -> Value {
    serde_json::from_str(CORPUS).expect("frozen calibration corpus must be valid JSON")
}

fn cases(corpus: &Value) -> &[Value] {
    corpus["cases"].as_array().expect("cases must be an array")
}

fn case_by_id<'a>(all_cases: &'a [Value], id: &str) -> &'a Value {
    all_cases
        .iter()
        .find(|case| case["id"].as_str() == Some(id))
        .unwrap_or_else(|| panic!("missing calibration reference case {id}"))
}

fn close(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() <= ABS_TOL * expected.abs().max(1.0)
}

fn assert_scores(id: &str, actual: Scores, expected: &Value) {
    for (field, actual_value) in [
        ("brier_score", actual.brier_score),
        ("log_loss", actual.log_loss),
        ("mean_probability", actual.mean_probability),
        ("observed_positive_rate", actual.observed_positive_rate),
    ] {
        let expected_value = scalar(&expected[field], &format!("{id}.expected.{field}"))
            .unwrap_or_else(|error| panic!("{id}: bad expected score: {error}"));
        assert!(
            close(actual_value, expected_value),
            "{id}: {field} actual={actual_value:.17e}, expected={expected_value:.17e}"
        );
    }
}

#[test]
fn frozen_probability_score_corpus_matches_reference_and_rejects_invalid_inputs() {
    let corpus = corpus();
    assert_eq!(
        corpus["schema"].as_str(),
        Some("mycelix-trust-calibration-reference-v1")
    );
    assert_eq!(
        corpus["profile"].as_str(),
        Some("BinaryProbabilityScoreReferenceV1")
    );
    let all_cases = cases(&corpus);
    assert_eq!(all_cases.len(), 18, "unexpected corpus case count");
    assert_eq!(corpus["case_count"].as_u64(), Some(18));
    let controls = corpus["mutation_controls"]
        .as_array()
        .expect("mutation_controls must be an array");
    assert_eq!(controls.len(), 6, "unexpected mutation-control count");

    let mut ids = HashSet::new();
    let mut valid_count = 0;
    let mut invalid_count = 0;
    for case in all_cases {
        let id = case["id"].as_str().expect("case ID must be a string");
        assert!(ids.insert(id), "duplicate case ID {id}");
        let expected_valid = case["expected"]["valid"]
            .as_bool()
            .expect("expected.valid must be boolean");
        let result = evaluate_case(case);
        if expected_valid {
            valid_count += 1;
            let actual =
                result.unwrap_or_else(|error| panic!("{id}: valid case rejected: {error}"));
            assert_scores(id, actual, &case["expected"]);
        } else {
            invalid_count += 1;
            assert!(
                result.is_err(),
                "{id}: invalid probability batch was accepted"
            );
        }
    }
    assert_eq!(valid_count, 9, "unexpected valid-case count");
    assert_eq!(invalid_count, 9, "unexpected invalid-case count");
}

#[test]
fn discrimination_and_proper_scores_are_not_replaced_by_aggregate_calibration() {
    let corpus = corpus();
    let all_cases = cases(&corpus);
    let constant = evaluate_case(case_by_id(
        all_cases,
        "TCAL-004-constant-base-rate-not-informative",
    ))
    .expect("constant base-rate forecast is a valid probability batch");
    let discriminative =
        evaluate_case(case_by_id(all_cases, "TCAL-005-discriminative-and-correct"))
            .expect("discriminative forecast is valid");

    assert!(close(
        constant.mean_probability,
        constant.observed_positive_rate
    ));
    assert!(close(
        discriminative.mean_probability,
        discriminative.observed_positive_rate
    ));
    assert!(
        discriminative.brier_score < constant.brier_score,
        "equal aggregate calibration does not imply equal predictive usefulness"
    );
    assert!(
        discriminative.log_loss < constant.log_loss,
        "proper scoring must detect the more informative forecast"
    );
}

#[test]
fn base_rate_shift_requires_reassessment() {
    let corpus = corpus();
    let all_cases = cases(&corpus);
    let stale = evaluate_case(case_by_id(
        all_cases,
        "TCAL-006-base-rate-shift-underpredict",
    ))
    .expect("stale forecast uses valid probabilities");
    let adjusted = evaluate_case(case_by_id(all_cases, "TCAL-007-base-rate-shift-adjusted"))
        .expect("adjusted forecast uses valid probabilities");

    assert!(close(
        adjusted.mean_probability,
        adjusted.observed_positive_rate
    ));
    assert!(adjusted.brier_score < stale.brier_score);
    assert!(adjusted.log_loss < stale.log_loss);
}

#[test]
fn confident_wrong_predictions_are_penalized_and_near_boundaries_remain_finite() {
    let corpus = corpus();
    let all_cases = cases(&corpus);
    let wrong = evaluate_case(case_by_id(all_cases, "TCAL-002-overconfident-wrong"))
        .expect("overconfident forecast is a valid probability batch");
    let good = evaluate_case(case_by_id(all_cases, "TCAL-001-separable-probabilities"))
        .expect("well-separated forecast is valid");
    assert!(wrong.log_loss > good.log_loss);
    assert!(wrong.brier_score > good.brier_score);

    let near = evaluate_case(case_by_id(all_cases, "TCAL-009-near-certain-correct"))
        .expect("strictly interior near-boundary probabilities are valid");
    assert!(near.log_loss.is_finite() && near.brier_score.is_finite());
}

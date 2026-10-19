//! Research-only independent oracle for Beta-Bernoulli trust uncertainty.
//!
//! This test intentionally does not call MATL/TCDM production scoring. It freezes
 //! a separate mathematical reference so we can compare score-only policies with
 //! uncertainty-aware policies without quietly changing deployed trust semantics.
 //! Run with: cargo test -p mycelix-core-types --test beta_trust_posterior_reference_v1

use serde_json::{json, Value};
use std::collections::HashSet;

const CORPUS: &str = include_str!("data/beta-trust-posterior-reference-v1.json");
const ABS_TOL: f64 = 1.0e-12;

#[derive(Debug, Clone, Copy)]
struct Summary {
    alpha: f64,
    beta: f64,
    mean: f64,
    variance: f64,
    lower_bound: f64,
}

fn parse_finite(value: &Value, name: &str) -> Result<f64, String> {
    let raw = value
        .as_str()
        .ok_or_else(|| format!("{name}: expected a decimal string"))?;
    let parsed = raw
        .parse::<f64>()
        .map_err(|error| format!("{name}: invalid scalar {raw:?}: {error}"))?;
    if !parsed.is_finite() {
        return Err(format!("{name}: scalar must be finite"));
    }
    Ok(parsed)
}

fn evaluate(case: &Value, invert_observations: bool) -> Result<Summary, String> {
    let prior_alpha = parse_finite(&case["prior_alpha"], "prior_alpha")?;
    let prior_beta = parse_finite(&case["prior_beta"], "prior_beta")?;
    if prior_alpha <= 0.0 || prior_beta <= 0.0 {
        return Err("Beta prior parameters must be strictly positive".to_owned());
    }

    let epsilon = parse_finite(&case["tail_probability"], "tail_probability")?;
    if !(0.0 < epsilon && epsilon < 1.0) {
        return Err("tail_probability must be strictly between zero and one".to_owned());
    }

    let observations = case["observations"]
        .as_array()
        .ok_or_else(|| "observations must be an array of booleans".to_owned())?;
    let mut alpha = prior_alpha;
    let mut beta = prior_beta;
    for (index, observation) in observations.iter().enumerate() {
        let outcome = observation
            .as_bool()
            .ok_or_else(|| format!("observations[{index}] must be boolean"))?;
        let success = if invert_observations { !outcome } else { outcome };
        if success {
            alpha += 1.0;
        } else {
            beta += 1.0;
        }
    }

    let concentration = alpha + beta;
    if !alpha.is_finite() || !beta.is_finite() || !concentration.is_finite() {
        return Err("posterior parameters must remain finite".to_owned());
    }
    let mean = alpha / concentration;
    let variance = (alpha * beta) / (concentration * concentration * (concentration + 1.0));
    let tail_ratio = (1.0 - epsilon) / epsilon;
    let margin_squared = variance * tail_ratio;
    if !mean.is_finite()
        || !variance.is_finite()
        || variance < 0.0
        || !tail_ratio.is_finite()
        || !margin_squared.is_finite()
        || margin_squared < 0.0
    {
        return Err("posterior summary overflowed or became invalid".to_owned());
    }

    let lower_bound = (mean - margin_squared.sqrt()).max(0.0);
    if !lower_bound.is_finite() || !(0.0..=1.0).contains(&mean) {
        return Err("posterior summary outside its valid domain".to_owned());
    }

    Ok(Summary {
        alpha,
        beta,
        mean,
        variance,
        lower_bound,
    })
}

fn close(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() <= ABS_TOL * expected.abs().max(1.0)
}

fn expected_number(case: &Value, field: &str) -> f64 {
    parse_finite(&case["expected"][field], field).expect("expected value is a finite scalar")
}

fn by_id<'a>(cases: &'a [Value], id: &str) -> &'a Value {
    cases
        .iter()
        .find(|case| case["id"].as_str() == Some(id))
        .unwrap_or_else(|| panic!("reference corpus is missing {id}"))
}

#[test]
fn frozen_corpus_matches_reference_values_and_rejects_invalid_domains() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("reference corpus is valid JSON");
    assert_eq!(
        corpus["schema"].as_str(),
        Some("mycelix-beta-trust-posterior-reference-v1")
    );
    let cases = corpus["cases"]
        .as_array()
        .expect("reference cases must be an array");
    assert_eq!(cases.len(), 22, "unexpected frozen case count");
    assert_eq!(corpus["case_count"].as_u64(), Some(22));
    let controls = corpus["mutation_controls"]
        .as_array()
        .expect("mutation_controls must be an array");
    assert_eq!(controls.len(), 5, "unexpected semantic mutation-control count");

    let mut ids = HashSet::new();
    let mut valid_count = 0;
    let mut invalid_count = 0;
    for case in cases {
        let id = case["id"].as_str().expect("each case needs a stable ID");
        assert!(ids.insert(id), "duplicate case ID: {id}");
        let valid = case["expected"]["valid"]
            .as_bool()
            .expect("expected.valid must be boolean");
        let result = evaluate(case, false);

        if valid {
            valid_count += 1;
            let actual = result.unwrap_or_else(|error| panic!("{id}: valid case rejected: {error}"));
            for (field, value) in [
                ("alpha", actual.alpha),
                ("beta", actual.beta),
                ("mean", actual.mean),
                ("variance", actual.variance),
                ("lower_bound", actual.lower_bound),
            ] {
                let expected = expected_number(case, field);
                assert!(
                    close(value, expected),
                    "{id}: {field} actual {value:.17e} differs from expected {expected:.17e}"
                );
            }
            assert!(actual.variance >= 0.0, "{id}: variance must be nonnegative");
            assert!((0.0..=1.0).contains(&actual.mean), "{id}: mean outside [0,1]");
            assert!(
                (0.0..=1.0).contains(&actual.lower_bound),
                "{id}: lower bound outside [0,1]"
            );
            assert!(
                actual.lower_bound <= actual.mean,
                "{id}: conservative lower bound exceeds posterior mean"
            );
        } else {
            invalid_count += 1;
            assert!(result.is_err(), "{id}: invalid input was accepted");
        }
    }

    assert_eq!(valid_count, 13);
    assert_eq!(invalid_count, 9);
}

#[test]
fn observation_order_is_irrelevant_under_the_iid_bernoulli_model() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("reference corpus");
    let cases = corpus["cases"].as_array().expect("cases array");
    let a = evaluate(by_id(cases, "BTP-012-ordering-a"), false).expect("case A is valid");
    let b = evaluate(by_id(cases, "BTP-013-ordering-b"), false).expect("case B is valid");
    assert!(close(a.alpha, b.alpha));
    assert!(close(a.beta, b.beta));
    assert!(close(a.mean, b.mean));
    assert!(close(a.variance, b.variance));
    assert!(close(a.lower_bound, b.lower_bound));
}

#[test]
fn swapping_success_and_failure_evidence_reflects_the_posterior() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("reference corpus");
    let cases = corpus["cases"].as_array().expect("cases array");
    let success_biased = evaluate(by_id(cases, "BTP-010-asymmetric-prior-success-biased"), false)
        .expect("success-biased case is valid");
    let failure_biased = evaluate(by_id(cases, "BTP-011-mirrored-prior-failure-biased"), false)
        .expect("failure-biased case is valid");
    assert!(close(success_biased.mean + failure_biased.mean, 1.0));
    assert!(close(success_biased.variance, failure_biased.variance));
}

#[test]
fn named_semantic_mutants_are_detected_by_the_frozen_cases() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("reference corpus");
    let cases = corpus["cases"].as_array().expect("cases array");

    // MUT-001: ignoring observations returns the prior mean, not the posterior mean.
    let many_successes = by_id(cases, "BTP-006-ten-successes");
    let correct = evaluate(many_successes, false).expect("valid many-success case");
    let prior_alpha = parse_finite(&many_successes["prior_alpha"], "prior_alpha").unwrap();
    let prior_beta = parse_finite(&many_successes["prior_beta"], "prior_beta").unwrap();
    let prior_mean = prior_alpha / (prior_alpha + prior_beta);
    assert!(!close(correct.mean, prior_mean));

    // MUT-002: reversing success/failure semantics must change an asymmetric case.
    let asymmetric = by_id(cases, "BTP-009-ten-success-one-failure");
    let normal = evaluate(asymmetric, false).expect("valid asymmetric case");
    let swapped = evaluate(asymmetric, true).expect("swapped outcomes remain valid");
    assert!(!close(normal.mean, swapped.mean));

    // MUT-003: omitting the final (+1) denominator term corrupts Beta variance.
    let jeffreys = by_id(cases, "BTP-002-jeffreys-prior-no-evidence");
    let correct_jeffreys = evaluate(jeffreys, false).expect("valid Jeffreys prior");
    let alpha = correct_jeffreys.alpha;
    let beta = correct_jeffreys.beta;
    let total = alpha + beta;
    let wrong_variance = (alpha * beta) / (total * total * total);
    assert!(!close(correct_jeffreys.variance, wrong_variance));

    // MUT-004: the inverted Cantelli ratio creates a materially different bound.
    let epsilon = parse_finite(&many_successes["tail_probability"], "tail_probability").unwrap();
    let wrong_margin = (correct.variance * (epsilon / (1.0 - epsilon))).sqrt();
    let wrong_lower_bound = (correct.mean - wrong_margin).max(0.0);
    assert!(!close(correct.lower_bound, wrong_lower_bound));

    // MUT-005: posterior mean alone hides substantial uncertainty in balanced evidence.
    let balanced = evaluate(by_id(cases, "BTP-008-balanced-evidence"), false)
        .expect("valid balanced-evidence case");
    assert!(!close(balanced.lower_bound, balanced.mean));
}

#[test]
fn deterministic_parameter_grid_preserves_posterior_invariants() {
    let prior_grid = [0.1_f64, 0.5, 1.0, 2.0, 10.0, 100.0];
    let evidence_grid = [0_usize, 1, 5, 20];
    let tail_grid = [0.01_f64, 0.05, 0.1, 0.25, 0.5, 0.9];

    let mut checked = 0_usize;
    for prior_alpha in prior_grid {
        for prior_beta in prior_grid {
            for successes in evidence_grid {
                for failures in evidence_grid {
                    let observations: Vec<bool> = std::iter::repeat_n(true, successes)
                        .chain(std::iter::repeat_n(false, failures))
                        .collect();
                    let mut previous_lower_bound = 0.0_f64;
                    for epsilon in tail_grid {
                        let case = json!({
                            "prior_alpha": prior_alpha.to_string(),
                            "prior_beta": prior_beta.to_string(),
                            "observations": observations,
                            "tail_probability": epsilon.to_string()
                        });
                        let result = evaluate(&case, false).expect("grid case is valid");
                        assert!(result.variance >= 0.0);
                        assert!(result.variance <= 0.25);
                        assert!((0.0..=1.0).contains(&result.mean));
                        assert!((0.0..=1.0).contains(&result.lower_bound));
                        assert!(result.lower_bound <= result.mean);
                        assert!(
                            result.lower_bound + ABS_TOL >= previous_lower_bound,
                            "less conservative tail probability should not lower the bound"
                        );
                        previous_lower_bound = result.lower_bound;
                        checked += 1;
                    }
                }
            }
        }
    }
    assert_eq!(checked, 3456, "unexpected deterministic grid size");
}

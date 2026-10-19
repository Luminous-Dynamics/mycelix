// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Independent diagonal Covariance Intersection reference oracle.
//!
//! This file deliberately does not call covariance_intersection_3d or PeerFusion3D.
//! It freezes the information-form equation before production fusion semantics are
//! changed. It does not qualify full 3x3 covariance CI or multi-source fusion.

use serde_json::Value;
use std::collections::HashSet;

const CORPUS: &str = include_str!("data/covariance-intersection-reference-v1.json");
const ABS_TOL: f64 = 1.0e-12;

#[derive(Debug, Clone, Copy, PartialEq)]
struct DiagonalEstimate {
    mean: [f64; 3],
    covariance_diag: [f64; 3],
}

fn finite_scalar(value: &Value, field: &str) -> Result<f64, String> {
    let parsed = match value {
        Value::Number(number) => number
            .as_f64()
            .ok_or_else(|| format!("{field}: number is outside the f64 domain"))?,
        Value::String(raw) => raw
            .parse::<f64>()
            .map_err(|error| format!("{field}: invalid f64 {raw:?}: {error}"))?,
        _ => return Err(format!("{field}: expected number or decimal string")),
    };
    if !parsed.is_finite() {
        return Err(format!("{field}: value must be finite"));
    }
    Ok(parsed)
}

fn vector3(value: &Value, field: &str) -> Result<[f64; 3], String> {
    let values = value
        .as_array()
        .filter(|values| values.len() == 3)
        .ok_or_else(|| format!("{field}: expected exactly three elements"))?;
    Ok([
        finite_scalar(&values[0], &format!("{field}[0]"))?,
        finite_scalar(&values[1], &format!("{field}[1]"))?,
        finite_scalar(&values[2], &format!("{field}[2]"))?,
    ])
}

/// Evaluate the diagonal information-form Covariance Intersection identity.
///
/// omega is a declared statistical CI weight, not reputation or trust. Singular
/// covariance, non-finite inputs, invalid weights and non-finite intermediates are
/// rejected rather than substituted with arbitrary precisions.
fn covariance_intersection_reference(
    a: &DiagonalEstimate,
    b: &DiagonalEstimate,
    omega: f64,
) -> Result<DiagonalEstimate, String> {
    if !omega.is_finite() || !(0.0..=1.0).contains(&omega) {
        return Err("omega must be finite and within [0, 1]".to_owned());
    }
    for (name, estimate) in [("a", a), ("b", b)] {
        if estimate.mean.iter().any(|value| !value.is_finite()) {
            return Err(format!("estimate {name} has a non-finite mean"));
        }
        if estimate
            .covariance_diag
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(format!(
                "estimate {name} covariance diagonal must be finite and strictly positive"
            ));
        }
    }

    let mut mean = [0.0; 3];
    let mut covariance_diag = [0.0; 3];
    for axis in 0..3 {
        let precision_a = 1.0 / a.covariance_diag[axis];
        let precision_b = 1.0 / b.covariance_diag[axis];
        let fused_precision = omega * precision_a + (1.0 - omega) * precision_b;
        if !precision_a.is_finite()
            || !precision_b.is_finite()
            || !fused_precision.is_finite()
            || fused_precision <= 0.0
        {
            return Err(format!("axis {axis}: precision arithmetic is invalid"));
        }

        let variance = 1.0 / fused_precision;
        let information_mean =
            omega * precision_a * a.mean[axis] + (1.0 - omega) * precision_b * b.mean[axis];
        let fused_mean = variance * information_mean;
        if !variance.is_finite()
            || variance <= 0.0
            || !information_mean.is_finite()
            || !fused_mean.is_finite()
        {
            return Err(format!("axis {axis}: fused estimate arithmetic is invalid"));
        }

        covariance_diag[axis] = variance;
        mean[axis] = fused_mean;
    }

    Ok(DiagonalEstimate {
        mean,
        covariance_diag,
    })
}

fn evaluate_case(case: &Value) -> Result<DiagonalEstimate, String> {
    let estimate = |name: &str| -> Result<DiagonalEstimate, String> {
        let value = &case[name];
        Ok(DiagonalEstimate {
            mean: vector3(&value["mean"], &format!("{name}.mean"))?,
            covariance_diag: vector3(
                &value["covariance_diag"],
                &format!("{name}.covariance_diag"),
            )?,
        })
    };
    let a = estimate("a")?;
    let b = estimate("b")?;
    let omega = finite_scalar(&case["omega"], "omega")?;
    covariance_intersection_reference(&a, &b, omega)
}

fn corpus() -> Value {
    serde_json::from_str(CORPUS).expect("frozen CI corpus must be valid JSON")
}

fn cases(corpus: &Value) -> &[Value] {
    corpus["cases"]
        .as_array()
        .expect("CI cases must be an array")
}

fn case_by_id<'a>(all_cases: &'a [Value], id: &str) -> &'a Value {
    all_cases
        .iter()
        .find(|case| case["id"].as_str() == Some(id))
        .unwrap_or_else(|| panic!("missing CI reference case {id}"))
}

fn close(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() <= ABS_TOL * expected.abs().max(1.0)
}

fn assert_expected(case_id: &str, estimate: DiagonalEstimate, expected: &Value) {
    let expected_mean = vector3(&expected["mean"], &format!("{case_id}.expected.mean"))
        .unwrap_or_else(|error| panic!("{case_id}: malformed expected mean: {error}"));
    let expected_covariance = vector3(
        &expected["covariance_diag"],
        &format!("{case_id}.expected.covariance_diag"),
    )
    .unwrap_or_else(|error| panic!("{case_id}: malformed expected covariance: {error}"));

    for axis in 0..3 {
        assert!(
            close(estimate.mean[axis], expected_mean[axis]),
            "{case_id}: mean[{axis}] actual={:.17e}, expected={:.17e}",
            estimate.mean[axis],
            expected_mean[axis]
        );
        assert!(
            close(estimate.covariance_diag[axis], expected_covariance[axis]),
            "{case_id}: covariance_diag[{axis}] actual={:.17e}, expected={:.17e}",
            estimate.covariance_diag[axis],
            expected_covariance[axis]
        );
    }
}

#[test]
fn frozen_diagonal_ci_corpus_matches_expected_and_rejects_invalid_inputs() {
    let corpus = corpus();
    assert_eq!(
        corpus["schema"].as_str(),
        Some("mycelix-position-covariance-intersection-reference-v1")
    );
    assert_eq!(
        corpus["profile"].as_str(),
        Some("DiagonalCovarianceIntersectionReferenceV1")
    );
    let all_cases = cases(&corpus);
    assert_eq!(all_cases.len(), 22, "unexpected CI corpus size");
    assert_eq!(corpus["case_count"].as_u64(), Some(22));
    assert_eq!(corpus["valid_case_count"].as_u64(), Some(12));
    assert_eq!(corpus["invalid_case_count"].as_u64(), Some(10));
    let mutations = corpus["mutation_controls"]
        .as_array()
        .expect("mutation_controls must be an array");
    assert_eq!(mutations.len(), 6, "unexpected mutation-control count");

    let mut ids = HashSet::new();
    let mut valid_count = 0;
    let mut invalid_count = 0;
    for case in all_cases {
        let id = case["id"].as_str().expect("case ID must be a string");
        assert!(ids.insert(id), "duplicate CI case ID {id}");
        let expected_valid = case["expected"]["valid"]
            .as_bool()
            .expect("expected.valid must be boolean");
        let result = evaluate_case(case);
        if expected_valid {
            valid_count += 1;
            let actual =
                result.unwrap_or_else(|error| panic!("{id}: valid CI case rejected: {error}"));
            assert_expected(id, actual, &case["expected"]);
        } else {
            invalid_count += 1;
            assert!(result.is_err(), "{id}: invalid CI input was accepted");
        }
    }
    assert_eq!(valid_count, 12);
    assert_eq!(invalid_count, 10);
}

#[test]
fn equal_covariance_does_not_gain_spurious_precision_from_disagreement() {
    let corpus = corpus();
    let case = case_by_id(cases(&corpus), "CI3D-001-equal-unit-covariance");
    let result = evaluate_case(case).expect("equal covariance case is valid");
    assert_eq!(result.covariance_diag, [1.0, 1.0, 1.0]);
    assert_eq!(result.mean, [2.0, 0.0, 0.0]);
}

#[test]
fn covariance_bounds_hold_axis_by_axis() {
    let corpus = corpus();
    for case in cases(&corpus)
        .iter()
        .filter(|case| case["expected"]["valid"].as_bool() == Some(true))
    {
        let result = evaluate_case(case).expect("valid case must evaluate");
        let a_cov = vector3(&case["a"]["covariance_diag"], "a.covariance_diag").unwrap();
        let b_cov = vector3(&case["b"]["covariance_diag"], "b.covariance_diag").unwrap();
        for axis in 0..3 {
            let lower = a_cov[axis].min(b_cov[axis]);
            let upper = a_cov[axis].max(b_cov[axis]);
            assert!(
                result.covariance_diag[axis] >= lower * (1.0 - ABS_TOL)
                    && result.covariance_diag[axis] <= upper * (1.0 + ABS_TOL),
                "{}: CI covariance escaped the input variance interval on axis {axis}",
                case["id"].as_str().unwrap()
            );
        }
    }
}

#[test]
fn swapping_estimates_and_complementing_omega_is_equivalent() {
    let corpus = corpus();
    let original = case_by_id(cases(&corpus), "CI3D-006-asymmetric-weight");
    let swapped = case_by_id(cases(&corpus), "CI3D-007-swapped-input-complement");
    let a = evaluate_case(original).expect("original case is valid");
    let b = evaluate_case(swapped).expect("swapped case is valid");
    for axis in 0..3 {
        assert!(close(a.mean[axis], b.mean[axis]));
        assert!(close(a.covariance_diag[axis], b.covariance_diag[axis]));
    }
    #[test]
    fn deterministic_10000_case_grid_preserves_ci_properties() {
        fn next_unit(seed: &mut u64) -> f64 {
            *seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((*seed >> 11) as f64) / ((1_u64 << 53) as f64)
        }

        let mut seed = 0x6a09e667f3bcc909_u64;
        for case_index in 0..10_000 {
            let mut draw = || next_unit(&mut seed);
            let mean_a = std::array::from_fn(|_| (draw() * 2.0 - 1.0) * 10_000.0);
            let mean_b = std::array::from_fn(|_| (draw() * 2.0 - 1.0) * 10_000.0);
            let covariance_a = std::array::from_fn(|_| 10.0_f64.powf(draw() * 16.0 - 8.0));
            let covariance_b = std::array::from_fn(|_| 10.0_f64.powf(draw() * 16.0 - 8.0));
            let omega = draw();

            let a = DiagonalEstimate {
                mean: mean_a,
                covariance_diag: covariance_a,
            };
            let b = DiagonalEstimate {
                mean: mean_b,
                covariance_diag: covariance_b,
            };
            let forward = covariance_intersection_reference(&a, &b, omega)
                .unwrap_or_else(|error| panic!("grid case {case_index}: {error}"));
            let reverse = covariance_intersection_reference(&b, &a, 1.0 - omega)
                .unwrap_or_else(|error| panic!("swapped grid case {case_index}: {error}"));

            for axis in 0..3 {
                let lower = covariance_a[axis].min(covariance_b[axis]);
                let upper = covariance_a[axis].max(covariance_b[axis]);
                assert!(forward.covariance_diag[axis].is_finite());
                assert!(forward.covariance_diag[axis] > 0.0);
                assert!(
                    forward.covariance_diag[axis] >= lower * (1.0 - ABS_TOL)
                        && forward.covariance_diag[axis] <= upper * (1.0 + ABS_TOL),
                    "grid case {case_index}, axis {axis}: variance outside CI input bounds"
                );
                assert!(
                    close(forward.mean[axis], reverse.mean[axis]),
                    "grid case {case_index}, axis {axis}: swapped-mean invariance failed"
                );
                assert!(
                    close(forward.covariance_diag[axis], reverse.covariance_diag[axis]),
                    "grid case {case_index}, axis {axis}: swapped-covariance invariance failed"
                );
            }
        }
    }
}

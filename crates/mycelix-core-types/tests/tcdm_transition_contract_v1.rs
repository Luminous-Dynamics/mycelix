// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Research-only oracle for a fail-closed TCDM transition contract.
//!
//! This deliberately does not invoke the production TCDMState. It captures a
//! candidate policy before any production behavior or mirrored implementation
//! is changed.

use serde_json::{Value, json};
use std::collections::HashSet;

const CORPUS: &str = include_str!("data/tcdm-transition-contract-v1.json");
const ABS_TOL: f64 = 1.0e-12;
const MAX_INTERACTION_COUNT: u64 = u32::MAX as u64;

#[derive(Debug, Clone, PartialEq)]
struct State {
    trust_level: f64,
    confidence: f64,
    decay_rate: f64,
    last_update: i64,
    interaction_count: u32,
}

#[derive(Debug, Clone, PartialEq)]
struct Transition {
    state: State,
    no_op: bool,
}

fn parse_finite(value: &Value, name: &str) -> Result<f64, String> {
    let raw = value
        .as_str()
        .ok_or_else(|| format!("{name}: expected decimal string"))?;
    let parsed = raw
        .parse::<f64>()
        .map_err(|error| format!("{name}: invalid scalar {raw:?}: {error}"))?;
    if !parsed.is_finite() {
        return Err(format!("{name}: scalar must be finite"));
    }
    Ok(parsed)
}

fn parse_i64(value: &Value, name: &str) -> Result<i64, String> {
    let raw = value
        .as_str()
        .ok_or_else(|| format!("{name}: expected integer string"))?;
    raw.parse::<i64>()
        .map_err(|error| format!("{name}: invalid i64 {raw:?}: {error}"))
}

fn parse_state(value: &Value) -> Result<State, String> {
    let trust_level = parse_finite(&value["trust_level"], "trust_level")?;
    let confidence = parse_finite(&value["confidence"], "confidence")?;
    let decay_rate = parse_finite(&value["decay_rate"], "decay_rate")?;
    if !(0.0..=1.0).contains(&trust_level) {
        return Err("trust_level must be in [0, 1]".to_owned());
    }
    if !(0.0..=1.0).contains(&confidence) {
        return Err("confidence must be in [0, 1]".to_owned());
    }
    if !(0.0..=1.0).contains(&decay_rate) {
        return Err("decay_rate must be in [0, 1]".to_owned());
    }

    let last_update = parse_i64(&value["last_update"], "last_update")?;
    let interaction_count = value["interaction_count"]
        .as_u64()
        .filter(|count| *count <= MAX_INTERACTION_COUNT)
        .ok_or_else(|| "interaction_count must fit u32".to_owned())?
        as u32;

    Ok(State {
        trust_level,
        confidence,
        decay_rate,
        last_update,
        interaction_count,
    })
}

fn validate_result(state: &State) -> Result<(), String> {
    if !state.trust_level.is_finite() || !(0.0..=1.0).contains(&state.trust_level) {
        return Err("resulting trust level must be finite and in [0, 1]".to_owned());
    }
    if !state.confidence.is_finite() || !(0.0..=1.0).contains(&state.confidence) {
        return Err("resulting confidence must be finite and in [0, 1]".to_owned());
    }
    if !state.decay_rate.is_finite() || !(0.0..=1.0).contains(&state.decay_rate) {
        return Err("resulting decay rate must be finite and in [0, 1]".to_owned());
    }
    Ok(())
}

fn transition(case: &Value) -> Result<Transition, String> {
    let mut state = parse_state(&case["state"])?;
    match case["operation"]["kind"]
        .as_str()
        .ok_or_else(|| "operation.kind must be a string".to_owned())?
    {
        "decay" => {
            let current_time = parse_i64(&case["operation"]["current_time"], "current_time")?;
            if current_time == 0 {
                return Err(
                    "timestamp zero is reserved for the legacy uninitialized sentinel".to_owned(),
                );
            }
            if state.last_update == 0 {
                // Match the legacy first-observation sentinel: initialize only.
                state.last_update = current_time;
            } else {
                if current_time < state.last_update {
                    return Err("timestamp regression is rejected".to_owned());
                }
                let elapsed_seconds = current_time
                    .checked_sub(state.last_update)
                    .ok_or_else(|| "elapsed timestamp subtraction overflowed".to_owned())?;
                if elapsed_seconds < 0 {
                    return Err("elapsed duration must be non-negative".to_owned());
                }

                let factor = if elapsed_seconds == 0 {
                    1.0
                } else if state.decay_rate == 0.0 {
                    0.0
                } else {
                    state
                        .decay_rate
                        .powf(elapsed_seconds as f64 / 86_400.0)
                };
                if !factor.is_finite() || !(0.0..=1.0).contains(&factor) {
                    return Err("decay factor must be finite and in [0, 1]".to_owned());
                }
                state.trust_level *= factor;
                state.confidence *= factor.sqrt();
                state.last_update = current_time;
            }
            validate_result(&state)?;
            Ok(Transition {
                state,
                no_op: false,
            })
        }
        "interaction" => {
            let weight = parse_finite(&case["operation"]["weight"], "weight")?;
            if weight < 0.0 {
                return Err("interaction weight must be non-negative".to_owned());
            }
            if weight == 0.0 {
                return Ok(Transition {
                    state,
                    no_op: true,
                });
            }

            let next_count = state
                .interaction_count
                .checked_add(1)
                .ok_or_else(|| "interaction count overflowed".to_owned())?;
            let success = case["operation"]["success"]
                .as_bool()
                .ok_or_else(|| "operation.success must be boolean".to_owned())?;
            let alpha = weight.min(0.3);
            let observation = if success { 1.0 } else { 0.0 };
            state.trust_level = state.trust_level * (1.0 - alpha) + observation * alpha;
            state.confidence = (state.confidence + 0.05 * (alpha / 0.3)).min(1.0);
            state.interaction_count = next_count;
            validate_result(&state)?;
            Ok(Transition {
                state,
                no_op: false,
            })
        }
        kind => Err(format!("unknown transition operation {kind:?}")),
    }
}

fn corpus() -> Value {
    serde_json::from_str(CORPUS).expect("frozen TCDM transition corpus must be valid JSON")
}

fn cases(corpus: &Value) -> &[Value] {
    corpus["cases"]
        .as_array()
        .expect("frozen transition cases must be an array")
}

fn case_by_id<'a>(cases: &'a [Value], id: &str) -> &'a Value {
    cases
        .iter()
        .find(|case| case["id"].as_str() == Some(id))
        .unwrap_or_else(|| panic!("missing frozen transition case {id}"))
}

fn close(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() <= ABS_TOL * expected.abs().max(1.0)
}

fn compare_expected_state(id: &str, actual: &State, expected: &Value) {
    for (field, actual_value) in [
        ("trust_level", actual.trust_level),
        ("confidence", actual.confidence),
        ("decay_rate", actual.decay_rate),
    ] {
        let expected_value =
            parse_finite(&expected[field], &format!("{id}.expected.state.{field}"))
                .unwrap_or_else(|error| panic!("{id}: invalid expected value: {error}"));
        assert!(
            close(actual_value, expected_value),
            "{id}: {field} actual {actual_value:.17e} != expected {expected_value:.17e}"
        );
    }
    let expected_time = parse_i64(&expected["last_update"], &format!("{id}.last_update"))
        .expect("expected last_update must fit i64");
    assert_eq!(actual.last_update, expected_time, "{id}: last_update mismatch");
    assert_eq!(
        u64::from(actual.interaction_count),
        expected["interaction_count"]
            .as_u64()
            .expect("expected interaction_count must be unsigned"),
        "{id}: interaction_count mismatch"
    );
}

#[test]
fn frozen_transition_corpus_matches_candidate_contract() {
    let corpus = corpus();
    assert_eq!(
        corpus["schema"].as_str(),
        Some("mycelix-tcdm-transition-contract-v1")
    );
    let all_cases = cases(&corpus);
    assert_eq!(all_cases.len(), 32, "unexpected frozen case count");
    assert_eq!(corpus["case_count"].as_u64(), Some(32));
    assert_eq!(
        corpus["decision_rules"]
            .as_array()
            .expect("decision_rules must be an array")
            .len(),
        8
    );
    let controls = corpus["mutation_controls"]
        .as_array()
        .expect("mutation_controls must be an array");
    assert_eq!(controls.len(), 6, "unexpected mutation-control count");

    let mut ids = HashSet::new();
    let mut valid_count = 0;
    let mut invalid_count = 0;
    for case in all_cases {
        let id = case["id"].as_str().expect("each case must have a stable ID");
        assert!(ids.insert(id), "duplicate case ID {id}");
        let expected_valid = case["expected"]["valid"]
            .as_bool()
            .expect("expected.valid must be boolean");
        let result = transition(case);
        if expected_valid {
            valid_count += 1;
            let actual =
                result.unwrap_or_else(|error| panic!("{id}: valid case rejected: {error}"));
            compare_expected_state(id, &actual.state, &case["expected"]["state"]);
            assert_eq!(
                actual.no_op,
                case["expected"]["no_op"].as_bool().expect("valid case needs no_op"),
                "{id}: no_op mismatch"
            );
        } else {
            invalid_count += 1;
            assert!(result.is_err(), "{id}: invalid transition input was accepted");
        }
    }
    assert_eq!(valid_count, 17, "unexpected valid case count");
    assert_eq!(invalid_count, 15, "unexpected invalid case count");
}

#[test]
fn time_regression_and_overflow_fail_closed() {
    let corpus = corpus();
    let all_cases = cases(&corpus);
    for id in [
        "TC-007-backwards-time",
        "TC-008-delta-overflow",
        "TC-009-negative-decay-rate",
        "TC-010-decay-rate-above-one",
        "TC-031-zero-timestamp-sentinel-collision",
        "TC-032-transition-to-zero-sentinel-rejected",
    ] {
        assert!(transition(case_by_id(all_cases, id)).is_err(), "{id}");
    }
    let original = case_by_id(all_cases, "TC-003-one-day-decay");
    let actual = transition(original).expect("one-day decay is valid");
    assert!(actual.state.trust_level < 0.8);
    assert!(actual.state.confidence < 0.6);
    assert_eq!(actual.state.last_update, 86_500);
}

#[test]
fn zero_weight_does_not_create_evidence_or_confidence() {
    let corpus = corpus();
    let case = case_by_id(cases(&corpus), "TC-019-zero-weight-is-no-op");
    let result = transition(case).expect("zero weight has documented no-op semantics");
    let before = parse_state(&case["state"]).expect("fixture input state is valid");
    assert!(result.no_op);
    assert_eq!(result.state, before);
}

#[test]
fn positive_weight_confidence_increment_scales_with_effective_weight() {
    let corpus = corpus();
    let all_cases = cases(&corpus);
    let half = transition(case_by_id(all_cases, "TC-015-success-weight-point-two"))
        .expect("positive weight is valid");
    let cap = transition(case_by_id(all_cases, "TC-017-success-weight-cap"))
        .expect("cap weight is valid");
    assert!(half.state.confidence > 0.1);
    assert!(cap.state.confidence > half.state.confidence);
    assert!(close(cap.state.confidence, 0.15));
}

#[test]
fn deterministic_hundred_thousand_step_sequence_preserves_state_invariants() {
    let mut state = State {
        trust_level: 0.5,
        confidence: 0.1,
        decay_rate: 0.99,
        last_update: 1,
        interaction_count: 0,
    };
    let mut now = 1_i64;
    let mut seed = 0x9e3779b97f4a7c15_u64;

    for step in 0..100_000_u64 {
        // Fixed LCG gives reproducible outcomes/weights without external RNG dependencies.
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let success = (seed >> 63) != 0;
        let weight = match (seed >> 60) & 0x7 {
            0 => 0.0,
            1 => f64::from(f32::from_bits(1)),
            2 => 0.01,
            3 => 0.05,
            4 => 0.1,
            5 => 0.2,
            6 => 0.3,
            _ => 0.8,
        };

        let interaction = json!({
            "state": {
                "trust_level": state.trust_level.to_string(),
                "confidence": state.confidence.to_string(),
                "decay_rate": state.decay_rate.to_string(),
                "last_update": state.last_update.to_string(),
                "interaction_count": u64::from(state.interaction_count)
            },
            "operation": {
                "kind": "interaction",
                "success": success,
                "weight": weight.to_string()
            }
        });
        state = transition(&interaction)
            .unwrap_or_else(|error| panic!("step {step}: interaction rejected: {error}"))
            .state;

        now = now.checked_add(86_400).expect("deterministic timestamp remains in range");
        let decay = json!({
            "state": {
                "trust_level": state.trust_level.to_string(),
                "confidence": state.confidence.to_string(),
                "decay_rate": state.decay_rate.to_string(),
                "last_update": state.last_update.to_string(),
                "interaction_count": u64::from(state.interaction_count)
            },
            "operation": {"kind": "decay", "current_time": now.to_string()}
        });
        state = transition(&decay)
            .unwrap_or_else(|error| panic!("step {step}: decay rejected: {error}"))
            .state;

        assert!(state.trust_level.is_finite() && (0.0..=1.0).contains(&state.trust_level));
        assert!(state.confidence.is_finite() && (0.0..=1.0).contains(&state.confidence));
        assert!((0.0..=1.0).contains(&state.decay_rate));
        assert!(state.last_update <= now);
    }
}

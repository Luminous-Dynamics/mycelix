//! Bounded bulk-tail credal reference for NRES-015.
//!
//! This is a small, auditable two-band comparator, not a reproduction of an
//! external bulk-calibrated credal method. The bulk is a frozen finite set
//! with a conditional nominal distribution and a total-variation ambiguity
//! radius. The tail is represented by an interval for total tail mass and an
//! explicit per-outcome loss upper bound. A null tail bound produces infinite
//! risk whenever positive tail mass remains admissible.
//!
//! This evaluator is uncalibrated and uses synthetic public-development data.
//! It makes no coverage, deployment, readiness or SOTA claim.

use serde_json::{json, Value};
use std::collections::BTreeSet;

const BENCHMARK_ID: &str = "PRAXIS-NRES-015-BT1-BOUNDED-BULK-TAIL-001";
const TOLERANCE: f64 = 1e-9;

fn number(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64)
}

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn forbidden_key_scan(value: &Value, path: &str, errors: &mut Vec<String>) {
    const FORBIDDEN: &[&str] = &[
        "overall_resilience_score",
        "composite_score",
        "leaderboard_score",
        "personal_suitability_score",
        "political_affiliation",
        "enlistment_interest",
        "loyalty_score",
        "participant_name",
        "participant_email",
        "participant_phone",
    ];
    match value {
        Value::Object(object) => {
            for (key, child) in object {
                if FORBIDDEN.contains(&key.as_str()) {
                    errors.push(format!("forbidden field at {path}/{key}"));
                }
                forbidden_key_scan(child, &format!("{path}/{key}"), errors);
            }
        }
        Value::Array(items) => {
            for (index, child) in items.iter().enumerate() {
                forbidden_key_scan(child, &format!("{path}/{index}"), errors);
            }
        }
        _ => {}
    }
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter().zip(right).map(|(a, b)| a * b).sum()
}

/// Exact upper expectation over {q : TV(q,p) <= radius}.
///
/// TV is one half of the L1 distance, so at most radius probability mass may
/// be moved. An optimum moves mass from lowest-loss donor states to a
/// maximum-loss receiver. Greedy sorting is exact on finite support because
/// each unit transferred to the maximum-loss state has constant non-negative
/// marginal gain for each donor.
fn tv_upper_expectation(
    probabilities: &[f64],
    losses: &[f64],
    radius: f64,
) -> Result<(f64, f64), String> {
    if probabilities.is_empty() || probabilities.len() != losses.len() {
        return Err("bulk probability/loss vectors must have equal non-zero length".to_owned());
    }
    if !radius.is_finite() || !(0.0..=1.0).contains(&radius) {
        return Err("bulk TV radius must be finite and in [0,1]".to_owned());
    }
    if probabilities.iter().any(|p| !p.is_finite() || *p < 0.0 || *p > 1.0) {
        return Err("bulk probabilities must be finite and in [0,1]".to_owned());
    }
    if losses.iter().any(|loss| !loss.is_finite() || *loss < 0.0) {
        return Err("bulk losses must be finite and non-negative".to_owned());
    }
    let mass: f64 = probabilities.iter().sum();
    if (mass - 1.0).abs() > TOLERANCE {
        return Err("bulk conditional probabilities must sum to one".to_owned());
    }

    let nominal = dot(probabilities, losses);
    let receiver = losses
        .iter()
        .enumerate()
        .max_by(|left, right| left.1.total_cmp(right.1))
        .map(|(index, _)| index)
        .ok_or_else(|| "bulk loss vector is empty".to_owned())?;
    let maximum_loss = losses[receiver];

    let mut donors: Vec<usize> = (0..losses.len()).filter(|index| *index != receiver).collect();
    donors.sort_by(|left, right| {
        losses[*left]
            .total_cmp(&losses[*right])
            .then_with(|| left.cmp(right))
    });

    let mut remaining = radius;
    let mut moved = 0.0;
    let mut worst_case = nominal;
    for donor in donors {
        if remaining <= TOLERANCE {
            break;
        }
        let transfer = probabilities[donor].min(remaining);
        if transfer > 0.0 {
            worst_case += transfer * (maximum_loss - losses[donor]);
            moved += transfer;
            remaining -= transfer;
        }
    }

    // Floating point round-off must not make the robust upper bound smaller
    // than nominal risk or exceed the maximum support loss.
    worst_case = worst_case.max(nominal).min(maximum_loss);
    Ok((worst_case, moved))
}

/// Evaluate a public-development two-band bulk-tail credal fixture.
///
/// Input contract:
/// - support_state_count is the number of enumerated IDs in [0, N).
/// - bulk_state_ids is a frozen, unique set; its complement is the tail.
/// - each profile supplies a conditional bulk distribution, bulk losses,
///   a TV radius, a tail-mass interval and a finite tail cap or explicit null.
pub fn evaluate_bulk_tail_credal(fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    forbidden_key_scan(fixture, "", &mut errors);

    if string(fixture, "benchmark_id") != Some(BENCHMARK_ID) {
        errors.push("unexpected NRES-015-BT1 benchmark_id".to_owned());
    }
    if string(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-015-BT1 schema_version".to_owned());
    }
    if string(fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("BT1 fixture must remain explicitly synthetic".to_owned());
    }
    if string(fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("BT1 fixture must remain public-development-only".to_owned());
    }
    if fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("BT1 fixture must remain inactive in the curriculum".to_owned());
    }

    let support_count = fixture
        .get("support_state_count")
        .and_then(Value::as_u64)
        .and_then(|value| usize::try_from(value).ok());
    let Some(support_count) = support_count.filter(|count| (2..=4096).contains(count)) else {
        errors.push("support_state_count must be an integer in [2,4096]".to_owned());
        return Err(errors);
    };

    let raw_bulk_ids = fixture.get("bulk_state_ids").and_then(Value::as_array);
    let Some(raw_bulk_ids) = raw_bulk_ids else {
        errors.push("bulk_state_ids must be an array".to_owned());
        return Err(errors);
    };
    let bulk_ids: Option<Vec<usize>> = raw_bulk_ids
        .iter()
        .map(|value| value.as_u64().and_then(|id| usize::try_from(id).ok()))
        .collect();
    let Some(bulk_ids) = bulk_ids else {
        errors.push("bulk_state_ids must contain non-negative integer state IDs".to_owned());
        return Err(errors);
    };
    let unique_ids: BTreeSet<usize> = bulk_ids.iter().copied().collect();
    if bulk_ids.is_empty() || unique_ids.len() != bulk_ids.len() {
        errors.push("bulk_state_ids must be non-empty and unique".to_owned());
    }
    if bulk_ids.iter().any(|id| *id >= support_count) {
        errors.push("bulk state ID is outside the declared support".to_owned());
    }
    let tail_ids: Vec<usize> = (0..support_count)
        .filter(|state_id| !unique_ids.contains(state_id))
        .collect();
    if tail_ids.is_empty() {
        errors.push("the tail complement must contain at least one state".to_owned());
    }

    let raw_profiles = fixture.get("profiles").and_then(Value::as_array);
    let Some(raw_profiles) = raw_profiles else {
        errors.push("profiles must be a non-empty array".to_owned());
        return Err(errors);
    };
    if raw_profiles.is_empty() {
        errors.push("profiles must be a non-empty array".to_owned());
    }

    let mut seen_profile_ids = BTreeSet::new();
    let mut profile_reports = Vec::new();

    for profile in raw_profiles {
        let profile_id = string(profile, "profile_id").unwrap_or("UNKNOWN");
        if profile_id == "UNKNOWN" {
            errors.push("each profile requires a non-empty profile_id".to_owned());
        }
        if !seen_profile_ids.insert(profile_id.to_owned()) {
            errors.push(format!("duplicate profile_id: {profile_id}"));
        }

        let Some(probability_values) = profile
            .get("bulk_conditional_probabilities")
            .and_then(Value::as_array)
        else {
            errors.push(format!("{profile_id}: bulk conditional probabilities are missing"));
            continue;
        };
        let Some(loss_values) = profile.get("bulk_state_losses").and_then(Value::as_array) else {
            errors.push(format!("{profile_id}: bulk state losses are missing"));
            continue;
        };
        if probability_values.len() != bulk_ids.len() || loss_values.len() != bulk_ids.len() {
            errors.push(format!("{profile_id}: bulk vectors must align with bulk_state_ids"));
            continue;
        }
        let probabilities: Option<Vec<f64>> = probability_values.iter().map(Value::as_f64).collect();
        let losses: Option<Vec<f64>> = loss_values.iter().map(Value::as_f64).collect();
        let (Some(probabilities), Some(losses)) = (probabilities, losses) else {
            errors.push(format!("{profile_id}: bulk vectors must contain only numbers"));
            continue;
        };

        let Some(tv_radius) = number(profile, "bulk_tv_radius") else {
            errors.push(format!("{profile_id}: bulk_tv_radius is missing"));
            continue;
        };
        let Some(tail_lower) = number(profile, "tail_mass_lower") else {
            errors.push(format!("{profile_id}: tail_mass_lower is missing"));
            continue;
        };
        let Some(tail_upper) = number(profile, "tail_mass_upper") else {
            errors.push(format!("{profile_id}: tail_mass_upper is missing"));
            continue;
        };
        if !tv_radius.is_finite() || !(0.0..=1.0).contains(&tv_radius) {
            errors.push(format!("{profile_id}: bulk_tv_radius must be in [0,1]"));
            continue;
        }
        if !tail_lower.is_finite()
            || !tail_upper.is_finite()
            || tail_lower < 0.0
            || tail_upper > 1.0
            || tail_lower > tail_upper
        {
            errors.push(format!("{profile_id}: tail mass interval must satisfy 0 <= lower <= upper <= 1"));
            continue;
        }
        if profile.get("tail_loss_upper_bound").is_none() {
            errors.push(format!("{profile_id}: tail_loss_upper_bound must be present; use null for unbounded"));
            continue;
        }
        let tail_cap_value = profile.get("tail_loss_upper_bound").unwrap();
        let tail_cap = if tail_cap_value.is_null() {
            None
        } else {
            match tail_cap_value.as_f64() {
                Some(cap) if cap.is_finite() && cap >= 0.0 => Some(cap),
                _ => {
                    errors.push(format!("{profile_id}: tail loss cap must be finite/non-negative or null"));
                    continue;
                }
            }
        };
        let tail_basis = string(profile, "tail_bound_basis");
        if tail_basis.is_none_or(|basis| basis.trim().is_empty()) {
            errors.push(format!("{profile_id}: tail_bound_basis must explain the bound or its absence"));
            continue;
        }

        match tv_upper_expectation(&probabilities, &losses, tv_radius) {
            Ok((bulk_upper, moved_mass)) => {
                let bulk_nominal = dot(&probabilities, &losses);
                let (risk, risk_status, chosen_tail_mass) = match tail_cap {
                    Some(tail_cap) => {
                        let chosen_mass = if tail_cap >= bulk_upper {
                            tail_upper
                        } else {
                            tail_lower
                        };
                        let total = (1.0 - chosen_mass) * bulk_upper + chosen_mass * tail_cap;
                        (json!(total), "finite-bounded", json!(chosen_mass))
                    }
                    None if tail_upper > 0.0 => {
                        (Value::Null, "infinite-unbounded-tail", json!(tail_upper))
                    }
                    None => (json!(bulk_upper), "finite-no-tail-mass", json!(0.0))
                };
                profile_reports.push(json!({
                    "profile_id": profile_id,
                    "bulk_nominal_expected_loss": bulk_nominal,
                    "bulk_worst_case_expected_loss": bulk_upper,
                    "bulk_tv_radius": tv_radius,
                    "bulk_tv_mass_moved": moved_mass,
                    "tail_mass_interval": {
                        "lower": tail_lower,
                        "upper": tail_upper
                    },
                    "tail_loss_upper_bound": tail_cap,
                    "tail_bound_basis": tail_basis,
                    "worst_case_tail_mass": chosen_tail_mass,
                    "worst_case_upper_expectation": risk,
                    "risk_status": risk_status
                }));
            }
            Err(error) => errors.push(format!("{profile_id}: {error}")),
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(json!({
        "benchmark_id": BENCHMARK_ID,
        "method_id": "two_band_tv_bulk_bounded_tail_reference",
        "method_scope": "finite-support bulk TV ambiguity plus interval tail mass and an explicit uniform tail-loss cap",
        "status": "synthetic-public-development-evaluated",
        "evaluation_split": "public-development-only",
        "active_in_curriculum": false,
        "support_state_count": support_count,
        "bulk_state_ids": bulk_ids,
        "tail_state_ids": tail_ids,
        "bulk_state_count": bulk_ids.len(),
        "tail_state_count": tail_ids.len(),
        "profiles": profile_reports,
        "calibration_claim": "none",
        "coverage_claim": "none",
        "deployment_claim": "none",
        "interpretation": "A finite result is conditional on the supplied bulk and tail bounds. A null upper expectation with infinite-unbounded-tail status is intentional, not a numeric failure."
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn tiny_fixture(profile: Value) -> Value {
        json!({
            "benchmark_id": BENCHMARK_ID,
            "schema_version": "1.0.0",
            "status": "synthetic-demonstration-for-review",
            "evaluation_split": "public-development-only",
            "active_in_curriculum": false,
            "support_state_count": 4,
            "bulk_state_ids": [0, 1],
            "profiles": [profile]
        })
    }

    fn profile(tv: f64, lower: f64, upper: f64, cap: Value) -> Value {
        json!({
            "profile_id": "TEST",
            "bulk_conditional_probabilities": [0.6, 0.4],
            "bulk_state_losses": [0.0, 10.0],
            "bulk_tv_radius": tv,
            "tail_mass_lower": lower,
            "tail_mass_upper": upper,
            "tail_loss_upper_bound": cap,
            "tail_bound_basis": "unit-test explicit assumption"
        })
    }

    #[test]
    fn two_state_tv_reallocation_matches_hand_solution() {
        let (upper, moved) = tv_upper_expectation(&[0.6, 0.4], &[0.0, 10.0], 0.25).unwrap();
        assert!((upper - 6.5).abs() < 1e-12);
        assert!((moved - 0.25).abs() < 1e-12);
    }

    #[test]
    fn greedy_tv_reallocation_uses_lowest_loss_donors_first() {
        let (upper, moved) =
            tv_upper_expectation(&[0.2, 0.3, 0.5], &[0.0, 4.0, 10.0], 0.3).unwrap();
        assert!((upper - 8.8).abs() < 1e-12);
        assert!((moved - 0.3).abs() < 1e-12);
    }

    #[test]
    fn larger_radius_cannot_reduce_bulk_upper_expectation() {
        let small = tv_upper_expectation(&[0.4, 0.35, 0.25], &[0.0, 3.0, 9.0], 0.1).unwrap().0;
        let large = tv_upper_expectation(&[0.4, 0.35, 0.25], &[0.0, 3.0, 9.0], 0.4).unwrap().0;
        assert!(large + 1e-12 >= small);
    }

    #[test]
    fn tv_radius_one_reaches_maximum_bulk_loss() {
        let upper = tv_upper_expectation(&[0.4, 0.6], &[2.0, 8.0], 1.0).unwrap().0;
        assert!((upper - 8.0).abs() < 1e-12);
    }

    #[test]
    fn finite_tail_cap_selects_upper_mass_when_tail_is_costlier() {
        let result = evaluate_bulk_tail_credal(&tiny_fixture(profile(0.0, 0.05, 0.20, json!(50.0)))).unwrap();
        let profile = &result["profiles"][0];
        let bulk = profile["bulk_worst_case_expected_loss"].as_f64().unwrap();
        let total = profile["worst_case_upper_expectation"].as_f64().unwrap();
        assert!((bulk - 4.0).abs() < 1e-12);
        assert!((total - 13.2).abs() < 1e-12);
        assert_eq!(profile["worst_case_tail_mass"], json!(0.20));
    }

    #[test]
    fn finite_tail_cap_selects_lower_mass_when_tail_is_cheaper() {
        let fixture = tiny_fixture(json!({
            "profile_id": "CHEAP-TAIL",
            "bulk_conditional_probabilities": [0.5, 0.5],
            "bulk_state_losses": [2.0, 6.0],
            "bulk_tv_radius": 0.0,
            "tail_mass_lower": 0.10,
            "tail_mass_upper": 0.40,
            "tail_loss_upper_bound": 1.0,
            "tail_bound_basis": "unit-test explicit assumption"
        }));
        let result = evaluate_bulk_tail_credal(&fixture).unwrap();
        assert_eq!(result["profiles"][0]["worst_case_tail_mass"], json!(0.10));
    }

    #[test]
    fn unbounded_tail_with_positive_admissible_mass_is_explicitly_infinite() {
        let result = evaluate_bulk_tail_credal(&tiny_fixture(profile(0.0, 0.0, 0.2, Value::Null))).unwrap();
        assert!(result["profiles"][0]["worst_case_upper_expectation"].is_null());
        assert_eq!(result["profiles"][0]["risk_status"], json!("infinite-unbounded-tail"));
    }

    #[test]
    fn unbounded_tail_with_zero_admissible_mass_remains_finite() {
        let result = evaluate_bulk_tail_credal(&tiny_fixture(profile(0.2, 0.0, 0.0, Value::Null))).unwrap();
        assert_eq!(result["profiles"][0]["risk_status"], json!("finite-no-tail-mass"));
        assert!(result["profiles"][0]["worst_case_upper_expectation"].is_number());
    }

    #[test]
    fn invalid_distribution_mass_is_rejected() {
        assert!(tv_upper_expectation(&[0.4, 0.4], &[1.0, 2.0], 0.0).is_err());
    }

    #[test]
    fn invalid_negative_losses_are_rejected() {
        assert!(tv_upper_expectation(&[0.5, 0.5], &[-1.0, 2.0], 0.1).is_err());
    }

    #[test]
    fn active_curriculum_fixture_is_rejected() {
        let mut fixture = tiny_fixture(profile(0.0, 0.0, 0.0, Value::Null));
        fixture["active_in_curriculum"] = json!(true);
        assert!(evaluate_bulk_tail_credal(&fixture).is_err());
    }

    #[test]
    fn duplicate_bulk_state_ids_are_rejected() {
        let mut fixture = tiny_fixture(profile(0.0, 0.0, 0.0, Value::Null));
        fixture["bulk_state_ids"] = json!([0, 0]);
        assert!(evaluate_bulk_tail_credal(&fixture).is_err());
    }

    #[test]
    fn invalid_tail_interval_is_rejected() {
        assert!(evaluate_bulk_tail_credal(&tiny_fixture(profile(0.0, 0.4, 0.2, json!(20.0)))).is_err());
    }
}

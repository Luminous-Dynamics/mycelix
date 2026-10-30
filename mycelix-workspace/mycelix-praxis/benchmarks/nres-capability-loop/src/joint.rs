//! Deterministic NRES-009 evaluator for categorical joint service-breach forecasts.
//!
//! The evaluator scores a candidate distribution over all binary service-breach
//! combinations. It does not generate outages, infer causal dependencies, score
//! people, or estimate real-world incident probabilities.

use serde_json::{json, Value};
use std::collections::BTreeSet;

fn str_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key)?.as_str()
}

fn num_field(value: &Value, key: &str) -> Option<f64> {
    value.get(key)?.as_f64()
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

fn hamming_distance(left: usize, right: usize) -> usize {
    (left ^ right).count_ones() as usize
}


fn dependence_scores(probabilities: &[f64], observed_mask: usize, dimension: usize) -> (f64, f64) {
    let pair_count = dimension * (dimension - 1) / 2;
    let mut variogram_score = 0.0;
    let mut pairwise_joint_brier = 0.0;
    for left in 0..dimension {
        for right in (left + 1)..dimension {
            let mut predicted_discordance = 0.0;
            let mut predicted_joint_breach = 0.0;
            for (mask, probability) in probabilities.iter().enumerate() {
                let left_bit = (mask >> left) & 1;
                let right_bit = (mask >> right) & 1;
                if left_bit != right_bit {
                    predicted_discordance += *probability;
                }
                if left_bit == 1 && right_bit == 1 {
                    predicted_joint_breach += *probability;
                }
            }
            let observed_left = (observed_mask >> left) & 1;
            let observed_right = (observed_mask >> right) & 1;
            let observed_discordance = (observed_left != observed_right) as u64;
            let observed_joint_breach = (observed_left == 1 && observed_right == 1) as u64;
            variogram_score +=
                (predicted_discordance - observed_discordance as f64).powi(2);
            pairwise_joint_brier +=
                (predicted_joint_breach - observed_joint_breach as f64).powi(2);
        }
    }
    (
        variogram_score / pair_count as f64,
        pairwise_joint_brier / pair_count as f64,
    )
}

fn joint_scores(probabilities: &[f64], observed_mask: usize) -> (f64, Option<f64>, f64) {
    let mut multiclass_brier = 0.0;
    for (mask, probability) in probabilities.iter().enumerate() {
        let probability = *probability;
        let target = if mask == observed_mask { 1.0 } else { 0.0 };
        multiclass_brier += (probability - target).powi(2);
    }
    let observed_probability = probabilities[observed_mask];
    let joint_log_loss = (observed_probability > 0.0).then(|| -observed_probability.ln());

    let mut first_term = 0.0;
    let mut second_term = 0.0;
    for (left_mask, left_probability) in probabilities.iter().enumerate() {
        let left_probability = *left_probability;
        first_term += left_probability
            * (hamming_distance(left_mask, observed_mask) as f64).sqrt();
        for (right_mask, right_probability) in probabilities.iter().enumerate() {
            let right_probability = *right_probability;
            second_term += left_probability
                * right_probability
                * (hamming_distance(left_mask, right_mask) as f64).sqrt();
        }
    }
    let energy_score = first_term - 0.5 * second_term;
    (multiclass_brier, joint_log_loss, energy_score)
}

/// Evaluate complete categorical distributions over all service-breach masks.
///
/// A successful report means only that the supplied public fixture is structurally
/// valid and that the declared proper-score formulas were applied deterministically.
pub fn evaluate_joint_service_forecasts(fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    forbidden_key_scan(fixture, "", &mut errors);

    if str_field(fixture, "benchmark_id")
        != Some("PRAXIS-NRES-009-JOINT-SERVICE-FORECAST-001")
    {
        errors.push("unexpected NRES-009 benchmark_id".to_owned());
    }
    if str_field(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-009 schema_version".to_owned());
    }
    if str_field(fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("NRES-009 fixture must remain explicitly synthetic".to_owned());
    }
    if str_field(fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("NRES-009 fixture must remain public development data".to_owned());
    }
    if fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("NRES-009 fixture must remain inactive".to_owned());
    }

    let Some(protocol) = fixture.get("protocol") else {
        return Err(vec!["NRES-009 protocol is missing".to_owned()]);
    };
    let origin = num_field(protocol, "forecast_origin_hour");
    let horizon = num_field(protocol, "forecast_horizon_hours");
    let tolerance = num_field(protocol, "probability_sum_tolerance");
    if origin.is_none_or(|value| !value.is_finite() || value < 0.0)
        || horizon.is_none_or(|value| !value.is_finite() || value <= 0.0)
        || tolerance.is_none_or(|value| !value.is_finite() || value <= 0.0 || value > 1e-6)
    {
        errors.push("forecast origin, positive horizon and probability-sum tolerance must be valid".to_owned());
    }
    let origin = origin.unwrap_or(0.0);
    let horizon = horizon.unwrap_or(0.0);
    let tolerance = tolerance.unwrap_or(1e-12);

    let Some(services) = fixture.get("service_catalog").and_then(Value::as_array) else {
        return Err(vec!["service_catalog is missing".to_owned()]);
    };
    let dimension = services.len();
    if !(2..=12).contains(&dimension) {
        errors.push("joint forecast service dimension must be between 2 and 12".to_owned());
        return Err(errors);
    }
    let state_count = 1usize
        .checked_shl(dimension as u32)
        .ok_or_else(|| vec!["joint forecast state-space size overflow".to_owned()])?;
    if protocol.get("state_count").and_then(Value::as_u64) != Some(state_count as u64) {
        errors.push("protocol state_count must equal 2^service_count".to_owned());
    }

    let mut service_ids = BTreeSet::new();
    let mut service_by_bit: Vec<Option<String>> = vec![None; dimension];
    for service in services {
        let id = str_field(service, "service_id");
        let bit = service.get("bit_index").and_then(Value::as_u64);
        if id.is_none_or(str::is_empty)
            || bit.is_none_or(|value| value >= dimension as u64)
            || str_field(service, "unit") != Some("binary-breach-indicator")
        {
            errors.push("service requires unique non-empty ID, in-range bit_index and binary-breach-indicator unit".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        let bit = bit.unwrap() as usize;
        if !service_ids.insert(id.clone()) {
            errors.push(format!("duplicate service ID: {id}"));
        }
        if service_by_bit[bit].replace(id).is_some() {
            errors.push(format!("duplicate service bit_index: {bit}"));
        }
    }
    if service_by_bit.iter().any(Option::is_none) {
        errors.push("service bit indexes must cover every index from zero through dimension minus one".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    let service_order: Vec<String> = service_by_bit
        .iter()
        .filter_map(|service| service.clone())
        .collect();

    let Some(cases) = fixture.get("evaluation_cases").and_then(Value::as_array) else {
        return Err(vec!["evaluation_cases is missing".to_owned()]);
    };
    if cases.is_empty() {
        errors.push("evaluation_cases must not be empty".to_owned());
        return Err(errors);
    }

    let mut case_ids = BTreeSet::new();
    let mut per_case = Vec::new();
    let mut joint_brier_sum = 0.0;
    let mut joint_log_loss_sum = 0.0;
    let mut zero_probability_observed_cases = 0usize;
    let mut energy_sum = 0.0;
    let mut variogram_sum = 0.0;
    let mut pairwise_joint_brier_sum = 0.0;
    let mut independent_brier_sum = 0.0;
    let mut independent_log_loss_sum = 0.0;
    let mut independent_zero_probability_cases = 0usize;
    let mut independent_energy_sum = 0.0;
    let mut independent_variogram_sum = 0.0;
    let mut independent_pairwise_joint_brier_sum = 0.0;
    let pair_count = dimension * (dimension - 1) / 2;

    let mut marginal_accumulators: Vec<(usize, usize, f64, f64)> =
        vec![(0, 0, 0.0, 0.0); dimension];
    let mut pair_accumulators: Vec<(usize, usize, f64, f64)> =
        vec![(0, 0, 0.0, 0.0); pair_count];

    for case in cases {
        let case_id = str_field(case, "case_id").unwrap_or("UNKNOWN").to_owned();
        if str_field(case, "case_id").is_none() || !case_ids.insert(case_id.clone()) {
            errors.push(format!("{case_id}: case IDs must be present and unique"));
        }
        if num_field(case, "forecast_origin_hour") != Some(origin)
            || num_field(case, "forecast_horizon_hours") != Some(horizon)
        {
            errors.push(format!("{case_id}: forecast origin or horizon differs from protocol"));
        }

        let Some(evaluator) = case.get("evaluator") else {
            errors.push(format!("{case_id}: evaluator-owned observation is missing"));
            continue;
        };
        let observed_mask = evaluator
            .get("observed_breach_mask")
            .and_then(Value::as_u64);
        let Some(observed_mask) = observed_mask.filter(|mask| *mask < state_count as u64) else {
            errors.push(format!("{case_id}: observed breach mask is outside the state space"));
            continue;
        };
        let Some(distribution) = case
            .get("candidate_output")
            .and_then(|candidate| candidate.get("forecast_distribution"))
            .and_then(Value::as_array)
        else {
            errors.push(format!("{case_id}: candidate joint forecast distribution is missing"));
            continue;
        };
        if distribution.len() != state_count {
            errors.push(format!("{case_id}: forecast must enumerate all {state_count} states exactly once"));
            continue;
        }

        let mut probabilities: Vec<Option<f64>> = vec![None; state_count];
        for entry in distribution {
            let mask = entry.get("state_mask").and_then(Value::as_u64);
            let probability = num_field(entry, "probability");
            let Some(mask) = mask.filter(|mask| *mask < state_count as u64) else {
                errors.push(format!("{case_id}: state mask is outside the declared state space"));
                continue;
            };
            let Some(probability) = probability.filter(|value| value.is_finite() && (0.0..=1.0).contains(value)) else {
                errors.push(format!("{case_id}: every state probability must be finite and within [0,1]"));
                continue;
            };
            let slot = &mut probabilities[mask as usize];
            if slot.replace(probability).is_some() {
                errors.push(format!("{case_id}: duplicate state mask {mask}"));
            }
        }
        if probabilities.iter().any(Option::is_none) {
            errors.push(format!("{case_id}: forecast is missing one or more state masks"));
            continue;
        }
        let probabilities: Vec<f64> = probabilities.into_iter().flatten().collect();
        let probability_sum = probabilities.iter().sum::<f64>();
        if (probability_sum - 1.0).abs() > tolerance {
            errors.push(format!("{case_id}: state probabilities sum to {probability_sum}, outside tolerance"));
            continue;
        }

        let observed_mask = observed_mask as usize;
        let (multiclass_brier, joint_log_loss, energy_score) =
            joint_scores(&probabilities, observed_mask);
        joint_brier_sum += multiclass_brier;
        energy_sum += energy_score;
        if let Some(log_loss) = joint_log_loss {
            joint_log_loss_sum += log_loss;
        } else {
            zero_probability_observed_cases += 1;
        }

        let mut marginals = Vec::with_capacity(dimension);
        let mut marginal_probabilities = Vec::with_capacity(dimension);
        for bit in 0..dimension {
            let probability = probabilities
                .iter()
                .enumerate()
                .filter(|(mask, _)| ((mask >> bit) & 1) == 1)
                .map(|(_, probability)| *probability)
                .sum::<f64>();
            let observed = ((observed_mask >> bit) & 1) as u64;
            let brier = (probability - observed as f64).powi(2);
            marginal_probabilities.push(probability);
            let accumulator = &mut marginal_accumulators[bit];
            accumulator.0 += 1;
            accumulator.1 += observed as usize;
            accumulator.2 += brier;
            accumulator.3 += probability;
            marginals.push(json!({
                "service_id": service_order[bit],
                "predicted_breach_probability": probability,
                "observed_breach": observed,
                "brier_score": brier
            }));
        }

        // This is a controlled dependence-only ablation: form the product of
        // the candidate's own marginals, leaving marginal probabilities intact.
        let independent_probabilities: Vec<f64> = (0..state_count)
            .map(|mask| {
                (0..dimension)
                    .map(|bit| {
                        let probability = marginal_probabilities[bit];
                        if ((mask >> bit) & 1) == 1 {
                            probability
                        } else {
                            1.0 - probability
                        }
                    })
                    .product()
            })
            .collect();
        let (independent_brier, independent_log_loss, independent_energy) =
            joint_scores(&independent_probabilities, observed_mask);
        let (independent_variogram, independent_pairwise_joint_brier) =
            dependence_scores(&independent_probabilities, observed_mask, dimension);
        independent_brier_sum += independent_brier;
        independent_energy_sum += independent_energy;
        independent_variogram_sum += independent_variogram;
        independent_pairwise_joint_brier_sum += independent_pairwise_joint_brier;
        if let Some(log_loss) = independent_log_loss {
            independent_log_loss_sum += log_loss;
        } else {
            independent_zero_probability_cases += 1;
        }

        let mut variogram_score = 0.0;
        let mut pairwise_joint_brier = 0.0;
        let mut pair_index = 0usize;
        let mut pair_metrics = Vec::with_capacity(pair_count);
        for left in 0..dimension {
            for right in (left + 1)..dimension {
                let mut predicted_discordance = 0.0;
                let mut predicted_joint_breach = 0.0;
                for (mask, probability) in probabilities.iter().enumerate() {
                    let left_bit = (mask >> left) & 1;
                    let right_bit = (mask >> right) & 1;
                    if left_bit != right_bit {
                        predicted_discordance += probability;
                    }
                    if left_bit == 1 && right_bit == 1 {
                        predicted_joint_breach += probability;
                    }
                }
                let observed_left = (observed_mask >> left) & 1;
                let observed_right = (observed_mask >> right) & 1;
                let observed_discordance = (observed_left != observed_right) as u64;
                let observed_joint_breach = (observed_left == 1 && observed_right == 1) as u64;
                let variogram_component =
                    (predicted_discordance - observed_discordance as f64).powi(2);
                let joint_brier_component =
                    (predicted_joint_breach - observed_joint_breach as f64).powi(2);
                variogram_score += variogram_component;
                pairwise_joint_brier += joint_brier_component;

                let accumulator = &mut pair_accumulators[pair_index];
                accumulator.0 += 1;
                accumulator.1 += observed_joint_breach as usize;
                accumulator.2 += joint_brier_component;
                accumulator.3 += predicted_joint_breach;

                pair_metrics.push(json!({
                    "service_a": service_order[left],
                    "service_b": service_order[right],
                    "predicted_discordance_probability": predicted_discordance,
                    "observed_discordance": observed_discordance,
                    "predicted_joint_breach_probability": predicted_joint_breach,
                    "observed_joint_breach": observed_joint_breach,
                    "joint_breach_brier_score": joint_brier_component
                }));
                pair_index += 1;
            }
        }
        variogram_score /= pair_count as f64;
        pairwise_joint_brier /= pair_count as f64;
        variogram_sum += variogram_score;
        pairwise_joint_brier_sum += pairwise_joint_brier;

        per_case.push(json!({
            "case_id": case_id,
            "observed_breach_mask": observed_mask,
            "multiclass_brier_score": multiclass_brier,
            "joint_log_loss": joint_log_loss,
            "energy_score": energy_score,
            "variogram_score": variogram_score,
            "pairwise_joint_breach_brier": pairwise_joint_brier,
            "independence_projection_baseline": {
                "mean_marginal_probabilities": marginal_probabilities,
                "multiclass_brier_score": independent_brier,
                "joint_log_loss": independent_log_loss,
                "energy_score": independent_energy,
                "variogram_score": independent_variogram,
                "pairwise_joint_breach_brier": independent_pairwise_joint_brier
            },
            "marginal_metrics": marginals,
            "pairwise_metrics": pair_metrics
        }));
    }

    if !errors.is_empty() {
        return Err(errors);
    }
    let case_count = per_case.len();
    if case_count == 0 {
        return Err(vec!["no valid joint forecast cases remain".to_owned()]);
    }
    let mean_log_loss = if zero_probability_observed_cases == 0 {
        Some(joint_log_loss_sum / case_count as f64)
    } else {
        None
    };

    let marginal_metrics: Vec<Value> = service_order
        .iter()
        .enumerate()
        .map(|(bit, service_id)| {
            let accumulator = marginal_accumulators[bit];
            let observations = accumulator.0;
            json!({
                "service_id": service_id,
                "observations": observations,
                "breach_events": accumulator.1,
                "mean_brier_score": (observations > 0).then_some(accumulator.2 / observations as f64),
                "mean_predicted_breach_probability": (observations > 0).then_some(accumulator.3 / observations as f64),
                "observed_breach_rate": (observations > 0).then_some(accumulator.1 as f64 / observations as f64)
            })
        })
        .collect();

    let mut pairwise_metrics = Vec::with_capacity(pair_count);
    let mut pair_index = 0usize;
    for left in 0..dimension {
        for right in (left + 1)..dimension {
            let accumulator = pair_accumulators[pair_index];
            let observations = accumulator.0;
            pairwise_metrics.push(json!({
                "service_a": service_order[left],
                "service_b": service_order[right],
                "observations": observations,
                "joint_breach_events": accumulator.1,
                "mean_pairwise_joint_breach_brier": (observations > 0).then_some(accumulator.2 / observations as f64),
                "mean_predicted_joint_breach_probability": (observations > 0).then_some(accumulator.3 / observations as f64),
                "observed_joint_breach_rate": (observations > 0).then_some(accumulator.1 as f64 / observations as f64)
            }));
            pair_index += 1;
        }
    }

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-009-JOINT-SERVICE-FORECAST-001",
        "interpretation": "synthetic joint-distribution verification only; no real-world outage dependence or predictive validity",
        "case_count": case_count,
        "service_count": dimension,
        "state_count": state_count,
        "pair_count": pair_count,
        "mean_multiclass_brier_score": joint_brier_sum / case_count as f64,
        "mean_joint_log_loss": mean_log_loss,
        "zero_probability_observed_cases": zero_probability_observed_cases,
        "mean_energy_score": energy_sum / case_count as f64,
        "mean_variogram_score": variogram_sum / case_count as f64,
        "mean_pairwise_joint_breach_brier": pairwise_joint_brier_sum / case_count as f64,
        "independence_projection_baseline": {
            "mean_multiclass_brier_score": independent_brier_sum / case_count as f64,
            "mean_joint_log_loss": (independent_zero_probability_cases == 0).then_some(independent_log_loss_sum / case_count as f64),
            "zero_probability_observed_cases": independent_zero_probability_cases,
            "mean_energy_score": independent_energy_sum / case_count as f64,
            "mean_variogram_score": independent_variogram_sum / case_count as f64,
            "mean_pairwise_joint_breach_brier": independent_pairwise_joint_brier_sum / case_count as f64
        },
        "independence_baseline_minus_candidate": {
            "positive_means_candidate_has_lower_loss": true,
            "multiclass_brier_score": (independent_brier_sum / case_count as f64) - (joint_brier_sum / case_count as f64),
            "energy_score": (independent_energy_sum / case_count as f64) - (energy_sum / case_count as f64),
            "variogram_score": (independent_variogram_sum / case_count as f64) - (variogram_sum / case_count as f64),
            "pairwise_joint_breach_brier": (independent_pairwise_joint_brier_sum / case_count as f64) - (pairwise_joint_brier_sum / case_count as f64)
        },
        "per_case": per_case,
        "marginal_metrics": marginal_metrics,
        "pairwise_metrics": pairwise_metrics,
        "aggregation_policy": "complementary non-compensatory views; no composite resilience or personal score"
    }))
}

#[cfg(test)]
mod tests {
    use super::evaluate_joint_service_forecasts;
    use serde_json::{json, Value};

    const FIXTURE: &str = include_str!("../../../examples/courses/civic-resilience-joint-service-forecast-fixture-001.json");
    const CONFORMANCE: &str = include_str!("../../../examples/courses/civic-resilience-joint-service-forecast-conformance-001.json");

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("NRES-009 fixture must be valid JSON")
    }

    fn first_case(value: &mut Value) -> &mut Value {
        &mut value["evaluation_cases"][0]
    }

    fn first_probability(value: &mut Value) -> &mut Value {
        &mut value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"][0]["probability"]
    }

    #[test]
    fn derives_joint_state_energy_variogram_and_pairwise_scores() {
        let report = evaluate_joint_service_forecasts(&fixture()).expect("fixture should validate");
        assert_eq!(report["case_count"].as_u64(), Some(8));
        assert_eq!(report["service_count"].as_u64(), Some(4));
        assert_eq!(report["state_count"].as_u64(), Some(16));
        assert_eq!(report["pair_count"].as_u64(), Some(6));
        assert!((report["mean_multiclass_brier_score"].as_f64().unwrap() - 0.831745894923294).abs() < 1e-12);
        assert!((report["mean_energy_score"].as_f64().unwrap() - 0.5200929682152707).abs() < 1e-12);
        assert!((report["mean_variogram_score"].as_f64().unwrap() - 0.23452752164886148).abs() < 1e-12);
        assert!((report["mean_pairwise_joint_breach_brier"].as_f64().unwrap() - 0.13428767029713268).abs() < 1e-12);
        assert!(report["mean_joint_log_loss"].as_f64().is_some());
        assert!(report.get("overall_resilience_score").is_none());
    }

    #[test]
    fn matches_all_published_joint_score_vectors() {
        let report = evaluate_joint_service_forecasts(&fixture()).expect("fixture should validate");
        let corpus: Value = serde_json::from_str(CONFORMANCE).expect("conformance must be valid JSON");
        let expected = &corpus["expected_metrics"];
        let tolerance = corpus["conformance_targets"]["golden_metric_absolute_tolerance"]
            .as_f64().unwrap_or(1e-12);
        for key in [
            "mean_multiclass_brier_score",
            "mean_joint_log_loss",
            "mean_energy_score",
            "mean_variogram_score",
            "mean_pairwise_joint_breach_brier"
        ] {
            assert!((report[key].as_f64().unwrap() - expected[key].as_f64().unwrap()).abs() <= tolerance, "{key} differs");
        }
        assert_eq!(report["zero_probability_observed_cases"], expected["zero_probability_observed_states"]);
        assert_eq!(report["per_case"].as_array().unwrap().len(), expected["per_case"].as_array().unwrap().len());
        assert_eq!(report["marginal_metrics"].as_array().unwrap().len(), expected["marginal_metrics"].as_array().unwrap().len());
        assert_eq!(report["pairwise_metrics"].as_array().unwrap().len(), expected["pairwise_metrics"].as_array().unwrap().len());
        for (actual, expected) in report["per_case"].as_array().unwrap().iter().zip(expected["per_case"].as_array().unwrap()) {
            for key in ["multiclass_brier_score", "joint_log_loss", "energy_score", "variogram_score", "pairwise_joint_breach_brier"] {
                assert!((actual[key].as_f64().unwrap() - expected[key].as_f64().unwrap()).abs() <= tolerance);
            }
        }
        for (actual, expected) in report["marginal_metrics"].as_array().unwrap().iter().zip(expected["marginal_metrics"].as_array().unwrap()) {
            assert_eq!(actual["service_id"], expected["service_id"]);
            for key in ["mean_brier_score", "mean_predicted_breach_probability", "observed_breach_rate"] {
                assert!((actual[key].as_f64().unwrap() - expected[key].as_f64().unwrap()).abs() <= tolerance);
            }
        }
        for (actual, expected) in report["pairwise_metrics"].as_array().unwrap().iter().zip(expected["pairwise_metrics"].as_array().unwrap()) {
            assert_eq!(actual["service_a"], expected["service_a"]);
            assert_eq!(actual["service_b"], expected["service_b"]);
            for key in ["mean_pairwise_joint_breach_brier", "mean_predicted_joint_breach_probability", "observed_joint_breach_rate"] {
                assert!((actual[key].as_f64().unwrap() - expected[key].as_f64().unwrap()).abs() <= tolerance);
            }
        }
    }

    #[test]
    fn forecast_distribution_order_does_not_change_scores() {
        let mut value = fixture();
        value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"].as_array_mut().unwrap().reverse();
        let report = evaluate_joint_service_forecasts(&value).expect("permutation should remain valid");
        let expected: Value = serde_json::from_str(CONFORMANCE).unwrap();
        assert!((report["mean_energy_score"].as_f64().unwrap() - expected["expected_metrics"]["mean_energy_score"].as_f64().unwrap()).abs() < 1e-12);
    }

    #[test]
    fn preserves_infinite_log_loss_as_null_and_count() {
        let mut value = fixture();
        let observed = first_case(&mut value)["evaluator"]["observed_breach_mask"].as_u64().unwrap() as usize;
        let mut distribution = value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"].as_array().unwrap().clone();
        let target_index = distribution.iter().position(|x| x["state_mask"].as_u64() == Some(observed as u64)).unwrap();
        let donor_index = if target_index == 0 { 1 } else { 0 };
        let donor = distribution[donor_index]["probability"].as_f64().unwrap();
        let target_probability = distribution[target_index]["probability"].as_f64().unwrap();
        distribution[donor_index]["probability"] = json!(donor + target_probability);
        distribution[target_index]["probability"] = json!(0.0);
        value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"] = json!(distribution);
        let report = evaluate_joint_service_forecasts(&value).expect("zero-probability state is a valid forecast with infinite log loss");
        assert_eq!(report["zero_probability_observed_cases"].as_u64(), Some(1));
        assert!(report["per_case"][0]["joint_log_loss"].is_null());
        assert!(report["mean_joint_log_loss"].is_null());
    }

    #[test]
    fn rejects_negative_probability() {
        let mut value = fixture();
        *first_probability(&mut value) = json!(-0.01);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_probability_above_one() {
        let mut value = fixture();
        *first_probability(&mut value) = json!(1.01);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_probability_mass_mismatch() {
        let mut value = fixture();
        *first_probability(&mut value) = json!(0.5);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_missing_state_mask() {
        let mut value = fixture();
        value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"].as_array_mut().unwrap().pop();
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_state_mask() {
        let mut value = fixture();
        let duplicate_mask = value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"][1]["state_mask"].clone();
        value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"][0]["state_mask"] = duplicate_mask;
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_state_mask_outside_dimension() {
        let mut value = fixture();
        value["evaluation_cases"][0]["candidate_output"]["forecast_distribution"][0]["state_mask"] = json!(16);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_observed_mask_outside_dimension() {
        let mut value = fixture();
        value["evaluation_cases"][0]["evaluator"]["observed_breach_mask"] = json!(16);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_service_id() {
        let mut value = fixture();
        let duplicate_id = value["service_catalog"][0]["service_id"].clone();
        value["service_catalog"][1]["service_id"] = duplicate_id;
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_service_bit_index() {
        let mut value = fixture();
        value["service_catalog"][1]["bit_index"] = json!(0);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_case_id() {
        let mut value = fixture();
        let duplicate_id = value["evaluation_cases"][0]["case_id"].clone();
        value["evaluation_cases"][1]["case_id"] = duplicate_id;
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_empty_case_cohort() {
        let mut value = fixture();
        value["evaluation_cases"] = json!([]);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_composite_score_injection() {
        let mut value = fixture();
        value["composite_score"] = json!(0.99);
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_missing_joint_distribution() {
        let mut value = fixture();
        value["evaluation_cases"][0]["candidate_output"].as_object_mut().unwrap().remove("forecast_distribution");
        assert!(evaluate_joint_service_forecasts(&value).is_err());
    }
}

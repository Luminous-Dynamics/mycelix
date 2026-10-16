//! Paired multi-seed evaluation for synthetic Pulse routing policies.
//!
//! The reported interval describes variation across deterministic seed
//! replicates of this model. It is not a confidence interval for real networks,
//! authenticated delivery, RF behavior, or hardware performance.

use super::simulation::simulate_corpus;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};

const MIN_SEEDS: u64 = 30;
const MAX_SEEDS: u64 = 1_000;
const BASELINE_STRATEGY: &str = "preferred_single_path";
const CANDIDATE_STRATEGIES: &[&str] = &["sequential_failover", "parallel_redundancy"];
// Conservative upper approximation to the two-sided 95% Student-t critical
// value for df >= 29. Reported intervals remain model-only and approximate.
const CONSERVATIVE_95_MULTIPLIER: f64 = 2.1;

#[derive(Default)]
struct ScenarioTotals {
    baseline_successes: u128,
    candidate_successes: u128,
    baseline_attempts_per_1000: u128,
    candidate_attempts_per_1000: u128,
    baseline_cost_per_1000: u128,
    candidate_cost_per_1000: u128,
    baseline_energy_millijoules_per_1000: u128,
    candidate_energy_millijoules_per_1000: u128,
}

/// Evaluate a candidate against the preferred-single-path baseline on the
/// corpus's explicitly declared holdout scenarios.
///
/// Both strategies see identical scenario inputs and the same seed for each
/// replicate. The seed count is constrained to >= 30 so the documented
/// conservative interval multiplier is appropriate as a model diagnostic.
pub fn evaluate_paired_seeds(
    corpus: &Value,
    messages: Option<u64>,
    first_seed: u64,
    seed_count: u64,
    candidate_strategy: &str,
) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();

    if !CANDIDATE_STRATEGIES.contains(&candidate_strategy) {
        errors.push(format!(
            "candidate strategy must be one of: {}",
            CANDIDATE_STRATEGIES.join(", ")
        ));
    }
    if !(MIN_SEEDS..=MAX_SEEDS).contains(&seed_count) {
        errors.push(format!(
            "seed count must be between {MIN_SEEDS} and {MAX_SEEDS}"
        ));
    }
    if seed_count > 0 && first_seed.checked_add(seed_count - 1).is_none() {
        errors.push("seed range overflows u64".to_owned());
    }

    let split = corpus.get("evaluation_split");
    let development_ids = parse_id_list(
        split.and_then(|value| value.get("development_scenario_ids")),
        "evaluation_split.development_scenario_ids",
        &mut errors,
    );
    let holdout_ids = parse_id_list(
        split.and_then(|value| value.get("holdout_scenario_ids")),
        "evaluation_split.holdout_scenario_ids",
        &mut errors,
    );
    let resilience_ids = parse_id_list(
        split.and_then(|value| value.get("resilience_scenario_ids")),
        "evaluation_split.resilience_scenario_ids",
        &mut errors,
    );

    let scenarios = match corpus.get("scenarios").and_then(Value::as_array) {
        Some(values) if !values.is_empty() => values,
        _ => {
            errors.push("scenarios must be a non-empty array".to_owned());
            return Err(errors);
        }
    };

    let mut observed_ids = BTreeSet::new();
    for (index, scenario) in scenarios.iter().enumerate() {
        match scenario.get("id").and_then(Value::as_str) {
            Some(id) if !id.is_empty() => {
                if !observed_ids.insert(id.to_owned()) {
                    errors.push(format!("duplicate scenario ID '{id}'"));
                }
            }
            _ => errors.push(format!("scenarios[{index}].id must be a non-empty string")),
        }
    }

    let development_set = development_ids.iter().cloned().collect::<BTreeSet<_>>();
    let holdout_set = holdout_ids.iter().cloned().collect::<BTreeSet<_>>();
    let resilience_set = resilience_ids.iter().cloned().collect::<BTreeSet<_>>();
    if development_ids.is_empty() || holdout_ids.is_empty() || resilience_ids.is_empty() {
        errors.push(
            "development, holdout, and resilience scenario lists must all be non-empty".to_owned(),
        );
    }
    if development_set.len() != development_ids.len() {
        errors.push("development scenario IDs must be unique".to_owned());
    }
    if holdout_set.len() != holdout_ids.len() {
        errors.push("holdout scenario IDs must be unique".to_owned());
    }
    if resilience_set.len() != resilience_ids.len() {
        errors.push("resilience scenario IDs must be unique".to_owned());
    }
    if !resilience_set.is_subset(&holdout_set) {
        errors.push("resilience scenario IDs must be a subset of the holdout set".to_owned());
    }
    if !development_set.is_disjoint(&holdout_set) {
        errors.push("development and holdout scenario IDs must be disjoint".to_owned());
    }
    let declared_ids = development_set.union(&holdout_set).cloned().collect::<BTreeSet<_>>();
    if declared_ids != observed_ids {
        let missing = observed_ids.difference(&declared_ids).cloned().collect::<Vec<_>>();
        let unknown = declared_ids.difference(&observed_ids).cloned().collect::<Vec<_>>();
        if !missing.is_empty() {
            errors.push(format!("scenario split omits scenarios: {}", missing.join(", ")));
        }
        if !unknown.is_empty() {
            errors.push(format!("scenario split contains unknown IDs: {}", unknown.join(", ")));
        }
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let messages_per_scenario = messages
        .or_else(|| corpus.get("default_message_count").and_then(Value::as_u64))
        .unwrap_or(0);
    if messages_per_scenario == 0 {
        return Err(vec!["message count must be greater than zero".to_owned()]);
    }

    let mut holdout_deltas_percentage_points = Vec::with_capacity(seed_count as usize);
    let mut resilience_deltas_percentage_points = Vec::with_capacity(seed_count as usize);
    let mut total_baseline_successes = 0_u128;
    let mut total_candidate_successes = 0_u128;
    let mut total_resilience_baseline_successes = 0_u128;
    let mut total_resilience_candidate_successes = 0_u128;
    let mut scenario_totals = BTreeMap::<String, ScenarioTotals>::new();

    for offset in 0..seed_count {
        let seed = first_seed + offset;
        let report = simulate_corpus(corpus, Some(messages_per_scenario), Some(seed))
            .map_err(|validation_errors| {
                validation_errors
                    .into_iter()
                    .map(|error| format!("seed {seed}: {error}"))
                    .collect::<Vec<_>>()
            })?;
        let rows = report
            .get("scenario_results")
            .and_then(Value::as_array)
            .ok_or_else(|| vec![format!("seed {seed}: simulator omitted scenario_results")])?;

        let mut seed_baseline_successes = 0_u128;
        let mut seed_candidate_successes = 0_u128;
        let mut seed_resilience_baseline_successes = 0_u128;
        let mut seed_resilience_candidate_successes = 0_u128;

        for row in rows {
            let Some(scenario_id) = row.get("scenario_id").and_then(Value::as_str) else {
                return Err(vec![format!("seed {seed}: scenario result has no ID")]);
            };
            if !holdout_set.contains(scenario_id) {
                continue;
            }
            let strategy_rows = row
                .get("strategy_results")
                .and_then(Value::as_array)
                .ok_or_else(|| {
                    vec![format!("seed {seed}: scenario '{scenario_id}' has no strategy results")]
                })?;
            let baseline_row = strategy_metrics(strategy_rows, BASELINE_STRATEGY).ok_or_else(|| {
                vec![format!(
                    "seed {seed}: scenario '{scenario_id}' lacks baseline '{BASELINE_STRATEGY}'"
                )]
            })?;
            let candidate_row = strategy_metrics(strategy_rows, candidate_strategy).ok_or_else(|| {
                vec![format!(
                    "seed {seed}: scenario '{scenario_id}' lacks candidate '{candidate_strategy}'"
                )]
            })?;
            let baseline = metric(baseline_row, "modeled_path_successes_before_deadline")
                .ok_or_else(|| vec![format!("seed {seed}: baseline success metric missing")])?;
            let candidate = metric(candidate_row, "modeled_path_successes_before_deadline")
                .ok_or_else(|| vec![format!("seed {seed}: candidate success metric missing")])?;
            let baseline_attempts = metric(baseline_row, "attempts_per_1000_messages")
                .ok_or_else(|| vec![format!("seed {seed}: baseline attempts metric missing")])?;
            let candidate_attempts = metric(candidate_row, "attempts_per_1000_messages")
                .ok_or_else(|| vec![format!("seed {seed}: candidate attempts metric missing")])?;
            let baseline_cost = metric(baseline_row, "illustrative_cost_units_per_message_x1000")
                .ok_or_else(|| vec![format!("seed {seed}: baseline cost metric missing")])?;
            let candidate_cost = metric(candidate_row, "illustrative_cost_units_per_message_x1000")
                .ok_or_else(|| vec![format!("seed {seed}: candidate cost metric missing")])?;
            let baseline_energy =
                metric(baseline_row, "illustrative_energy_input_millijoules_per_message_x1000")
                    .ok_or_else(|| vec![format!("seed {seed}: baseline energy metric missing")])?;
            let candidate_energy =
                metric(candidate_row, "illustrative_energy_input_millijoules_per_message_x1000")
                    .ok_or_else(|| vec![format!("seed {seed}: candidate energy metric missing")])?;

            seed_baseline_successes += u128::from(baseline);
            seed_candidate_successes += u128::from(candidate);
            let totals = scenario_totals.entry(scenario_id.to_owned()).or_default();
            totals.baseline_successes += u128::from(baseline);
            totals.candidate_successes += u128::from(candidate);
            totals.baseline_attempts_per_1000 += u128::from(baseline_attempts);
            totals.candidate_attempts_per_1000 += u128::from(candidate_attempts);
            totals.baseline_cost_per_1000 += u128::from(baseline_cost);
            totals.candidate_cost_per_1000 += u128::from(candidate_cost);
            totals.baseline_energy_millijoules_per_1000 += u128::from(baseline_energy);
            totals.candidate_energy_millijoules_per_1000 += u128::from(candidate_energy);
            if resilience_set.contains(scenario_id) {
                seed_resilience_baseline_successes += u128::from(baseline);
                seed_resilience_candidate_successes += u128::from(candidate);
            }
        }

        total_baseline_successes += seed_baseline_successes;
        total_candidate_successes += seed_candidate_successes;
        total_resilience_baseline_successes += seed_resilience_baseline_successes;
        total_resilience_candidate_successes += seed_resilience_candidate_successes;

        let holdout_denominator =
            (messages_per_scenario as f64) * (holdout_set.len() as f64);
        let holdout_baseline_rate = seed_baseline_successes as f64 / holdout_denominator;
        let holdout_candidate_rate = seed_candidate_successes as f64 / holdout_denominator;
        holdout_deltas_percentage_points
            .push((holdout_candidate_rate - holdout_baseline_rate) * 100.0);

        let resilience_denominator =
            (messages_per_scenario as f64) * (resilience_set.len() as f64);
        let resilience_baseline_rate =
            seed_resilience_baseline_successes as f64 / resilience_denominator;
        let resilience_candidate_rate =
            seed_resilience_candidate_successes as f64 / resilience_denominator;
        resilience_deltas_percentage_points
            .push((resilience_candidate_rate - resilience_baseline_rate) * 100.0);
    }

    let seed_count_f = seed_count as f64;
    let holdout_messages_total = seed_count_f
        * (messages_per_scenario as f64)
        * (holdout_set.len() as f64);
    let holdout_baseline_rate =
        total_baseline_successes as f64 / holdout_messages_total;
    let holdout_candidate_rate =
        total_candidate_successes as f64 / holdout_messages_total;
    let holdout_relative_improvement = if holdout_baseline_rate > 0.0 {
        Some(
            ((holdout_candidate_rate - holdout_baseline_rate) / holdout_baseline_rate) * 100.0,
        )
    } else {
        None
    };

    let resilience_messages_total = seed_count_f
        * (messages_per_scenario as f64)
        * (resilience_set.len() as f64);
    let resilience_baseline_rate =
        total_resilience_baseline_successes as f64 / resilience_messages_total;
    let resilience_candidate_rate =
        total_resilience_candidate_successes as f64 / resilience_messages_total;
    let resilience_relative_improvement = if resilience_baseline_rate > 0.0 {
        Some(
            ((resilience_candidate_rate - resilience_baseline_rate) / resilience_baseline_rate)
                * 100.0,
        )
    } else {
        None
    };

    let mean_holdout_delta =
        holdout_deltas_percentage_points.iter().sum::<f64>() / seed_count_f;
    let holdout_variance = holdout_deltas_percentage_points
        .iter()
        .map(|delta| (delta - mean_holdout_delta).powi(2))
        .sum::<f64>()
        / (seed_count_f - 1.0);
    let holdout_se = holdout_variance.sqrt() / seed_count_f.sqrt();
    let holdout_ci_low = mean_holdout_delta - CONSERVATIVE_95_MULTIPLIER * holdout_se;
    let holdout_ci_high = mean_holdout_delta + CONSERVATIVE_95_MULTIPLIER * holdout_se;

    let mean_resilience_delta =
        resilience_deltas_percentage_points.iter().sum::<f64>() / seed_count_f;
    let resilience_variance = resilience_deltas_percentage_points
        .iter()
        .map(|delta| (delta - mean_resilience_delta).powi(2))
        .sum::<f64>()
        / (seed_count_f - 1.0);
    let resilience_se = resilience_variance.sqrt() / seed_count_f.sqrt();
    let resilience_ci_low =
        mean_resilience_delta - CONSERVATIVE_95_MULTIPLIER * resilience_se;
    let resilience_ci_high =
        mean_resilience_delta + CONSERVATIVE_95_MULTIPLIER * resilience_se;

    let per_scenario = holdout_ids
        .iter()
        .map(|scenario_id| {
            let totals = scenario_totals.get(scenario_id).expect("validated holdout ID");
            let denominator = seed_count_f * (messages_per_scenario as f64);
            let baseline_pct = totals.baseline_successes as f64 / denominator * 100.0;
            let candidate_pct = totals.candidate_successes as f64 / denominator * 100.0;
            json!({
                "scenario_id": scenario_id,
                "baseline_success_rate_percent": baseline_pct,
                "candidate_success_rate_percent": candidate_pct,
                "absolute_delta_percentage_points": candidate_pct - baseline_pct,
                "baseline_attempts_per_1000_messages":
                    totals.baseline_attempts_per_1000 as f64 / seed_count_f,
                "candidate_attempts_per_1000_messages":
                    totals.candidate_attempts_per_1000 as f64 / seed_count_f,
                "baseline_illustrative_cost_units_per_1000_messages":
                    totals.baseline_cost_per_1000 as f64 / seed_count_f,
                "candidate_illustrative_cost_units_per_1000_messages":
                    totals.candidate_cost_per_1000 as f64 / seed_count_f,
                "baseline_illustrative_energy_millijoules_per_1000_messages":
                    totals.baseline_energy_millijoules_per_1000 as f64 / seed_count_f,
                "candidate_illustrative_energy_millijoules_per_1000_messages":
                    totals.candidate_energy_millijoules_per_1000 as f64 / seed_count_f
            })
        })
        .collect::<Vec<_>>();

    let stable_delta = per_scenario
        .iter()
        .find(|row| row["scenario_id"] == "stable-ip")
        .and_then(|row| row["absolute_delta_percentage_points"].as_f64());
    let stable_non_regression = stable_delta.map(|delta| delta >= -2.0);
    let relative_target_met =
        resilience_relative_improvement.is_some_and(|delta| delta >= 20.0);
    let interval_excludes_zero = resilience_ci_low > 0.0;

    Ok(json!({
        "report_version": "1.0.0",
        "report_type": "paired_seed_synthetic_route_evaluation",
        "corpus_id": corpus.get("corpus_id").and_then(Value::as_str),
        "simulation_only": true,
        "authenticated_delivery_established": false,
        "hardware_qualification": false,
        "real_world_reliability_established": false,
        "baseline_strategy": BASELINE_STRATEGY,
        "candidate_strategy": candidate_strategy,
        "seed_start": first_seed,
        "seed_count": seed_count,
        "seed_end_inclusive": first_seed + seed_count - 1,
        "messages_per_scenario_per_seed": messages_per_scenario,
        "development_scenario_ids": development_ids,
        "holdout_scenario_ids": holdout_ids,
        "resilience_scenario_ids": resilience_ids,
        "holdout_scenario_count": holdout_set.len(),
        "resilience_scenario_count": resilience_set.len(),
        "paired_evaluation": true,
        "confidence_interval": {
            "population": "declared resilience scenarios only",
            "method": "paired seed-level mean delta with sample standard error",
            "multiplier": CONSERVATIVE_95_MULTIPLIER,
            "nominal_coverage_percent": 95,
            "lower_delta_percentage_points": resilience_ci_low,
            "upper_delta_percentage_points": resilience_ci_high,
            "note": "Conservative t-critical approximation for at least 30 seed replicates; interval summarizes variation in this synthetic model only, not field-network uncertainty."
        },
        "holdout_aggregate": {
            "baseline_success_rate_percent": holdout_baseline_rate * 100.0,
            "candidate_success_rate_percent": holdout_candidate_rate * 100.0,
            "absolute_delta_percentage_points": (holdout_candidate_rate - holdout_baseline_rate) * 100.0,
            "relative_improvement_percent": holdout_relative_improvement,
            "paired_interval_lower_percentage_points": holdout_ci_low,
            "paired_interval_upper_percentage_points": holdout_ci_high
        },
        "resilience_aggregate": {
            "baseline_success_rate_percent": resilience_baseline_rate * 100.0,
            "candidate_success_rate_percent": resilience_candidate_rate * 100.0,
            "absolute_delta_percentage_points": (resilience_candidate_rate - resilience_baseline_rate) * 100.0,
            "relative_improvement_percent": resilience_relative_improvement
        },
        "model_only_screen": {
            "target_population": "resilience_scenario_ids only; stable-ip is reported separately",
            "relative_improvement_target_percent": 20,
            "relative_improvement_target_point_estimate_met": relative_target_met,
            "paired_interval_excludes_zero": interval_excludes_zero,
            "stable_network_delta_percentage_points": stable_delta,
            "stable_network_non_regression_target_percentage_points": -2.0,
            "stable_network_non_regression_met": stable_non_regression,
            "resilience_screen_pass": relative_target_met
                && interval_excludes_zero
                && stable_non_regression == Some(true),
            "not_a_release_qualification": true
        },
        "per_holdout_scenario": per_scenario,
        "limitations": [
            "All path outcomes are generated by the checked-in synthetic model; no network observations are used.",
            "Seed-to-seed intervals measure model sensitivity, not real-world confidence about route delivery.",
            "Modeled path success is not an authenticated recipient receipt.",
            "The model does not evaluate cryptography, RF, hardware, external interoperability, or real energy consumption.",
            "A passing model-only screen cannot establish release readiness."
        ]
    }))
}

/// Evaluate a candidate across a declared grid of common-draw mixture
/// weights, varying only the designated resilience scenarios. The weight is
/// a synthetic model knob, not a fitted or empirical correlation coefficient.
pub fn evaluate_correlation_sweep(
    corpus: &Value,
    messages: Option<u64>,
    first_seed: u64,
    seed_count: u64,
    candidate_strategy: &str,
) -> Result<Value, Vec<String>> {
    let split = corpus.get("evaluation_split");
    let weights_value = split
        .and_then(|value| value.get("correlation_sweep_basis_points"))
        .and_then(Value::as_array)
        .ok_or_else(|| {
            vec![
                "evaluation_split.correlation_sweep_basis_points must be an array".to_owned()
            ]
        })?;
    let mut weights = Vec::with_capacity(weights_value.len());
    for (index, value) in weights_value.iter().enumerate() {
        match value.as_u64() {
            Some(weight) if weight <= 10_000 => weights.push(weight),
            _ => {
                return Err(vec![format!(
                    "evaluation_split.correlation_sweep_basis_points[{index}] must be an integer in [0, 10000]"
                )]);
            }
        }
    }
    let unique_weights = weights.iter().copied().collect::<BTreeSet<_>>();
    if weights.len() < 2
        || unique_weights.len() != weights.len()
        || !unique_weights.contains(&0)
        || !unique_weights.contains(&10_000)
    {
        return Err(vec![
            "correlation sweep must have at least two unique weights and include both 0 and 10000"
                .to_owned(),
        ]);
    }

    let resilience_ids = split
        .and_then(|value| value.get("resilience_scenario_ids"))
        .and_then(Value::as_array)
        .ok_or_else(|| vec!["resilience_scenario_ids must be an array".to_owned()])?
        .iter()
        .map(|value| {
            value
                .as_str()
                .filter(|text| !text.trim().is_empty())
                .map(str::to_owned)
                .ok_or_else(|| {
                    vec!["resilience_scenario_ids must contain non-empty strings".to_owned()]
                })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let resilience_set = resilience_ids.iter().cloned().collect::<BTreeSet<_>>();
    if resilience_set.is_empty() || resilience_set.len() != resilience_ids.len() {
        return Err(vec![
            "resilience_scenario_ids must be non-empty and unique".to_owned()
        ]);
    }

    let mut points = Vec::with_capacity(weights.len());
    for weight in &weights {
        let mut weighted_corpus = corpus.clone();
        let scenarios = weighted_corpus
            .get_mut("scenarios")
            .and_then(Value::as_array_mut)
            .ok_or_else(|| vec!["scenarios must be an array".to_owned()])?;
        let mut found = BTreeSet::new();
        for scenario in scenarios {
            let Some(id) = scenario.get("id").and_then(Value::as_str) else {
                continue;
            };
            if resilience_set.contains(id) {
                scenario["failure_correlation_basis_points"] = json!(weight);
                found.insert(id.to_owned());
            }
        }
        if found != resilience_set {
            let missing = resilience_set.difference(&found).cloned().collect::<Vec<_>>();
            return Err(vec![format!(
                "resilience_scenario_ids include unknown scenarios: {}",
                missing.join(", ")
            )]);
        }

        let report = evaluate_paired_seeds(
            &weighted_corpus,
            messages,
            first_seed,
            seed_count,
            candidate_strategy,
        )?;
        points.push(json!({
            "common_draw_mixture_basis_points": weight,
            "resilience_aggregate": report["resilience_aggregate"].clone(),
            "confidence_interval": report["confidence_interval"].clone(),
            "model_only_screen": report["model_only_screen"].clone(),
            "per_holdout_scenario": report["per_holdout_scenario"].clone()
        }));
    }

    Ok(json!({
        "report_version": "1.0.0",
        "report_type": "synthetic_routing_correlation_sensitivity",
        "corpus_id": corpus.get("corpus_id").and_then(Value::as_str),
        "candidate_strategy": candidate_strategy,
        "baseline_strategy": BASELINE_STRATEGY,
        "seed_start": first_seed,
        "seed_count": seed_count,
        "messages_per_scenario_per_seed": messages
            .or_else(|| corpus.get("default_message_count").and_then(Value::as_u64)),
        "sensitivity_axis": {
            "name": "common_draw_mixture_basis_points",
            "interpretation": "At 0, same-domain path-success draws are independent conditional on explicit outage windows. At 10000, same-domain paths share one uniform draw. Intermediate values mix shared and path-specific draws. This is not a Pearson correlation coefficient and has not been calibrated against measured networks.",
            "applies_to": resilience_ids
        },
        "simulation_only": true,
        "authenticated_delivery_established": false,
        "real_world_reliability_established": false,
        "hardware_qualification": false,
        "points": points,
        "limitations": [
            "The sensitivity sweep varies only the synthetic draw-sharing assumption on resilience scenarios.",
            "Explicit outage windows remain common-cause outages regardless of the mixture weight.",
            "Seed intervals describe the synthetic model only.",
            "Modeled path success is not authenticated recipient delivery.",
            "No RF, hardware, calibrated correlation, or real energy evidence is evaluated."
        ]
    }))
}

fn parse_id_list(value: Option<&Value>, label: &str, errors: &mut Vec<String>) -> Vec<String> {
    let Some(items) = value.and_then(Value::as_array) else {
        errors.push(format!("{label} must be an array of scenario IDs"));
        return Vec::new();
    };
    let mut values = Vec::with_capacity(items.len());
    for (index, item) in items.iter().enumerate() {
        match item.as_str() {
            Some(id) if !id.trim().is_empty() => values.push(id.to_owned()),
            _ => errors.push(format!("{label}[{index}] must be a non-empty string")),
        }
    }
    values
}

fn strategy_metrics<'a>(rows: &'a [Value], strategy: &str) -> Option<&'a Value> {
    rows.iter()
        .find(|row| row.get("strategy").and_then(Value::as_str) == Some(strategy))
        .and_then(|row| row.get("metrics"))
}

fn metric(metrics: &Value, key: &str) -> Option<u64> {
    metrics.get(key).and_then(Value::as_u64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn corpus() -> Value {
        serde_json::from_str(include_str!("../../../benchmarks/traces.v1.json"))
            .expect("trace corpus JSON must parse")
    }

    #[test]
    fn paired_evaluator_is_deterministic_and_never_claims_field_evidence() {
        let corpus = corpus();
        let first = evaluate_paired_seeds(
            &corpus,
            Some(10),
            20261016,
            MIN_SEEDS,
            "sequential_failover",
        )
        .expect("declared holdout corpus should evaluate");
        let second = evaluate_paired_seeds(
            &corpus,
            Some(10),
            20261016,
            MIN_SEEDS,
            "sequential_failover",
        )
        .expect("same inputs should evaluate");
        assert_eq!(first, second);
        assert_eq!(first["paired_evaluation"], true);
        assert_eq!(first["simulation_only"], true);
        assert_eq!(first["real_world_reliability_established"], false);
        assert_eq!(
            first["holdout_scenario_count"],
            first["per_holdout_scenario"].as_array().unwrap().len()
        );
        assert_eq!(first["resilience_scenario_count"], 3);
        assert!(
            first["confidence_interval"]["population"]
                .as_str()
                .unwrap()
                .contains("resilience")
        );
        assert_eq!(first["model_only_screen"]["not_a_release_qualification"], true);
        assert!(
            first["per_holdout_scenario"][0]["candidate_attempts_per_1000_messages"]
                .is_number()
        );
        assert!(
            first["per_holdout_scenario"][0]
                ["candidate_illustrative_cost_units_per_1000_messages"]
                .is_number()
        );
        assert!(
            first["per_holdout_scenario"][0]
                ["candidate_illustrative_energy_millijoules_per_1000_messages"]
                .is_number()
        );
    }

    #[test]
    fn paired_evaluator_rejects_too_few_seeds_and_unknown_candidate() {
        let corpus = corpus();
        assert!(evaluate_paired_seeds(&corpus, Some(10), 7, 2, "sequential_failover").is_err());
        assert!(
            evaluate_paired_seeds(&corpus, Some(10), 7, MIN_SEEDS, "oracle").is_err()
        );
    }

    #[test]
    fn paired_evaluator_rejects_overlapping_or_incomplete_split() {
        let mut corpus = corpus();
        corpus["evaluation_split"]["holdout_scenario_ids"][0] = json!("scheduled-ip-outage");
        assert!(
            evaluate_paired_seeds(&corpus, Some(10), 7, MIN_SEEDS, "sequential_failover")
                .is_err()
        );

        let mut invalid_resilience = corpus();
        invalid_resilience["evaluation_split"]["resilience_scenario_ids"][0] =
            json!("local-only-first-contact");
        assert!(
            evaluate_paired_seeds(
                &invalid_resilience,
                Some(10),
                7,
                MIN_SEEDS,
                "sequential_failover"
            )
            .is_err()
        );
    }
}

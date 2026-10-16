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
    if development_ids.is_empty() || holdout_ids.is_empty() {
        errors.push("development and holdout scenario lists must both be non-empty".to_owned());
    }
    if development_set.len() != development_ids.len() {
        errors.push("development scenario IDs must be unique".to_owned());
    }
    if holdout_set.len() != holdout_ids.len() {
        errors.push("holdout scenario IDs must be unique".to_owned());
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

    let mut deltas_percentage_points = Vec::with_capacity(seed_count as usize);
    let mut total_baseline_successes = 0_u128;
    let mut total_candidate_successes = 0_u128;
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
            let baseline = strategy_successes(strategy_rows, BASELINE_STRATEGY)
                .ok_or_else(|| {
                    vec![format!(
                        "seed {seed}: scenario '{scenario_id}' lacks baseline '{BASELINE_STRATEGY}'"
                    )]
                })?;
            let candidate = strategy_successes(strategy_rows, candidate_strategy).ok_or_else(|| {
                vec![format!(
                    "seed {seed}: scenario '{scenario_id}' lacks candidate '{candidate_strategy}'"
                )]
            })?;

            seed_baseline_successes += u128::from(baseline);
            seed_candidate_successes += u128::from(candidate);
            let totals = scenario_totals.entry(scenario_id.to_owned()).or_default();
            totals.baseline_successes += u128::from(baseline);
            totals.candidate_successes += u128::from(candidate);
        }

        total_baseline_successes += seed_baseline_successes;
        total_candidate_successes += seed_candidate_successes;
        let denominator = (messages_per_scenario as f64) * (holdout_set.len() as f64);
        let baseline_rate = seed_baseline_successes as f64 / denominator;
        let candidate_rate = seed_candidate_successes as f64 / denominator;
        deltas_percentage_points.push((candidate_rate - baseline_rate) * 100.0);
    }

    let seed_count_f = seed_count as f64;
    let messages_total = seed_count_f
        * (messages_per_scenario as f64)
        * (holdout_set.len() as f64);
    let baseline_rate = total_baseline_successes as f64 / messages_total;
    let candidate_rate = total_candidate_successes as f64 / messages_total;
    let relative_improvement = if baseline_rate > 0.0 {
        Some(((candidate_rate - baseline_rate) / baseline_rate) * 100.0)
    } else {
        None
    };

    let mean_delta = deltas_percentage_points.iter().sum::<f64>() / seed_count_f;
    let sample_variance = deltas_percentage_points
        .iter()
        .map(|delta| (delta - mean_delta).powi(2))
        .sum::<f64>()
        / (seed_count_f - 1.0);
    let standard_error = sample_variance.sqrt() / seed_count_f.sqrt();
    let ci_low = mean_delta - CONSERVATIVE_95_MULTIPLIER * standard_error;
    let ci_high = mean_delta + CONSERVATIVE_95_MULTIPLIER * standard_error;

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
                "absolute_delta_percentage_points": candidate_pct - baseline_pct
            })
        })
        .collect::<Vec<_>>();

    let stable_delta = per_scenario
        .iter()
        .find(|row| row["scenario_id"] == "stable-ip")
        .and_then(|row| row["absolute_delta_percentage_points"].as_f64());
    let stable_non_regression = stable_delta.map(|delta| delta >= -2.0);
    let relative_target_met = relative_improvement.is_some_and(|delta| delta >= 20.0);
    let interval_excludes_zero = ci_low > 0.0;

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
        "holdout_scenario_count": holdout_set.len(),
        "paired_evaluation": true,
        "confidence_interval": {
            "method": "paired seed-level mean delta with sample standard error",
            "multiplier": CONSERVATIVE_95_MULTIPLIER,
            "nominal_coverage_percent": 95,
            "lower_delta_percentage_points": ci_low,
            "upper_delta_percentage_points": ci_high,
            "note": "Conservative t-critical approximation for at least 30 seed replicates; interval summarizes variation in this synthetic model only, not field-network uncertainty."
        },
        "holdout_aggregate": {
            "baseline_success_rate_percent": baseline_rate * 100.0,
            "candidate_success_rate_percent": candidate_rate * 100.0,
            "absolute_delta_percentage_points": (candidate_rate - baseline_rate) * 100.0,
            "relative_improvement_percent": relative_improvement
        },
        "model_only_screen": {
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

fn strategy_successes(rows: &[Value], strategy: &str) -> Option<u64> {
    rows.iter()
        .find(|row| row.get("strategy").and_then(Value::as_str) == Some(strategy))
        .and_then(|row| row.get("metrics"))
        .and_then(|metrics| metrics.get("modeled_path_successes_before_deadline"))
        .and_then(Value::as_u64)
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
        assert_eq!(first["holdout_scenario_count"], first["per_holdout_scenario"].as_array().unwrap().len());
        assert_eq!(first["model_only_screen"]["not_a_release_qualification"], true);
    }

    #[test]
    fn paired_evaluator_rejects_too_few_seeds_and_unknown_candidate() {
        let corpus = corpus();
        assert!(evaluate_paired_seeds(&corpus, Some(10), 7, 2, "sequential_failover").is_err());
        assert!(evaluate_paired_seeds(&corpus, Some(10), 7, MIN_SEEDS, "oracle").is_err());
    }

    #[test]
    fn paired_evaluator_rejects_overlapping_or_incomplete_split() {
        let mut corpus = corpus();
        corpus["evaluation_split"]["holdout_scenario_ids"][0] = json!("scheduled-ip-outage");
        assert!(evaluate_paired_seeds(&corpus, Some(10), 7, MIN_SEEDS, "sequential_failover").is_err());
    }
}

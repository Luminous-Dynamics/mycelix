//! NRES-012: Bernoulli total-variation ambiguity and robust decision evaluation.
//!
//! This reference evaluator compares nominal point decisions, worst-case
//! expected-loss decisions, and minimax-regret decisions. It is not an
//! operational policy engine and does not implement full Wasserstein DRO.

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

const SOURCE_FIXTURE: &str = include_str!("../../../examples/courses/civic-resilience-reliability-resolution-decision-value-fixture-001.json");
const SOURCE_FIXTURE_BLOB_SHA: &str = "a9f10ae28fe3c5031ed9ce0f88761e94ad6f4e51";

const REGIMES: &[&str] = &[
    "in_distribution",
    "temporal_shift",
    "structural_dependency_shift",
    "evidence_degradation",
];

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

const STRATEGIES: &[&str] = &[
    "nominal_point_forecast",
    "worst_case_expected_loss",
    "minimax_regret",
];

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key)?.as_str()
}

fn number(value: &Value, key: &str) -> Option<f64> {
    value.get(key)?.as_f64()
}

fn scan_forbidden(value: &Value, path: &str, errors: &mut Vec<String>) {
    match value {
        Value::Object(map) => {
            for (key, child) in map {
                if FORBIDDEN.contains(&key.as_str()) {
                    errors.push(format!("forbidden field at {path}/{key}"));
                }
                scan_forbidden(child, &format!("{path}/{key}"), errors);
            }
        }
        Value::Array(items) => {
            for (index, child) in items.iter().enumerate() {
                scan_forbidden(child, &format!("{path}/{index}"), errors);
            }
        }
        _ => {}
    }
}

#[derive(Clone, Debug)]
struct DecisionProfile {
    id: String,
    readiness_cost: f64,
    event_loss_without_action: f64,
    event_loss_with_action: f64,
    deferral_cost: f64,
    threshold: f64,
}

#[derive(Clone, Debug)]
struct EvaluationCase {
    regime_id: String,
    importance_weight: f64,
    observed_event: bool,
    action: String,
    point: Option<f64>,
    lower: Option<f64>,
    upper: Option<f64>,
    review_fallback: Option<String>,
}

#[derive(Clone, Copy, Debug)]
struct StrategyActions {
    nominal: &'static str,
    worst_case_expected: &'static str,
    minimax_regret: &'static str,
    regret_prepare: f64,
    regret_no_action: f64,
}

fn strategy_actions(
    profile: &DecisionProfile,
    point: f64,
    lower: f64,
    upper: f64,
) -> StrategyActions {
    let avoided_loss = profile.event_loss_without_action - profile.event_loss_with_action;
    let nominal = if point >= profile.threshold {
        "prepare"
    } else {
        "no_action"
    };
    // The worst expected losses cross at the same threshold. Exact ties are
    // deliberately resolved to no_action for deterministic conservative review.
    let worst_case_expected = if upper > profile.threshold {
        "prepare"
    } else {
        "no_action"
    };
    let regret_prepare = (profile.readiness_cost - lower * avoided_loss).max(0.0);
    let regret_no_action = (upper * avoided_loss - profile.readiness_cost).max(0.0);
    // Exact minimax-regret ties resolve to no_action.
    let minimax_regret = if regret_prepare < regret_no_action {
        "prepare"
    } else {
        "no_action"
    };
    StrategyActions {
        nominal,
        worst_case_expected,
        minimax_regret,
        regret_prepare,
        regret_no_action,
    }
}

fn actual_loss(action: &str, event: bool, profile: &DecisionProfile, deferral: f64) -> f64 {
    let event = if event { 1.0 } else { 0.0 };
    let base = if action == "prepare" {
        profile.readiness_cost + event * profile.event_loss_with_action
    } else {
        event * profile.event_loss_without_action
    };
    base + deferral
}

fn worst_case_expected_cost(action: &str, upper: f64, profile: &DecisionProfile) -> f64 {
    if action == "prepare" {
        profile.readiness_cost + upper * profile.event_loss_with_action
    } else {
        upper * profile.event_loss_without_action
    }
}

fn worst_case_regret(action: &str, lower: f64, upper: f64, profile: &DecisionProfile) -> f64 {
    let avoided_loss = profile.event_loss_without_action - profile.event_loss_with_action;
    if action == "prepare" {
        (profile.readiness_cost - lower * avoided_loss).max(0.0)
    } else {
        (upper * avoided_loss - profile.readiness_cost).max(0.0)
    }
}

fn effective_sample_size(weights: &[f64]) -> Option<f64> {
    let sum: f64 = weights.iter().sum();
    let squares: f64 = weights.iter().map(|weight| weight * weight).sum();
    (squares > 0.0).then_some(sum * sum / squares)
}

fn weighted_quantile(samples: &[(f64, f64)], quantile: f64) -> Option<f64> {
    if samples.is_empty() || !(0.0..=1.0).contains(&quantile) {
        return None;
    }
    let mut ordered = samples.to_vec();
    ordered.sort_by(|left, right| left.0.total_cmp(&right.0));
    let total_weight: f64 = ordered.iter().map(|sample| sample.1).sum();
    if total_weight <= 0.0 {
        return None;
    }
    let fallback = ordered.last().map(|sample| sample.0);
    let target = quantile * total_weight;
    let mut cumulative = 0.0;
    for (value, weight) in ordered {
        cumulative += weight;
        if cumulative >= target {
            return Some(value);
        }
    }
    fallback
}

fn source_case_map(source: &Value) -> Result<BTreeMap<String, &Value>, Vec<String>> {
    let Some(rows) = source.get("evaluation_cases").and_then(Value::as_array) else {
        return Err(vec!["pinned NRES-011 source cases are missing".to_owned()]);
    };
    let mut map = BTreeMap::new();
    let mut errors = Vec::new();
    for row in rows {
        let Some(id) = string(row, "case_id") else {
            errors.push("pinned NRES-011 source case has no case_id".to_owned());
            continue;
        };
        if map.insert(id.to_owned(), row).is_some() {
            errors.push(format!("pinned NRES-011 source contains duplicate case_id: {id}"));
        }
    }
    if !errors.is_empty() {
        Err(errors)
    } else {
        Ok(map)
    }
}

fn summarize_cases(
    cases: &[EvaluationCase],
    profile: &DecisionProfile,
    reference_probability: f64,
) -> Value {
    let total_weight: f64 = cases.iter().map(|case| case.importance_weight).sum();
    let all_weights: Vec<f64> = cases.iter().map(|case| case.importance_weight).collect();
    let total_ess = effective_sample_size(&all_weights);
    let issued: Vec<&EvaluationCase> = cases
        .iter()
        .filter(|case| case.action == "forecast")
        .collect();
    let issued_weights: Vec<f64> = issued.iter().map(|case| case.importance_weight).collect();
    let forecast_weight: f64 = issued_weights.iter().sum();
    let forecast_ess = effective_sample_size(&issued_weights);

    let mut width_sum = 0.0;
    let mut max_width = 0.0_f64;
    let mut width_samples = Vec::with_capacity(issued.len());
    let mut nominal_vs_robust = 0usize;
    let mut nominal_vs_minimax = 0usize;
    let mut robust_vs_minimax = 0usize;

    let mut strategy_costs = [0.0_f64; 3];
    let mut worst_expected_sums = [0.0_f64; 3];
    let mut worst_regret_sums = [0.0_f64; 3];
    let mut max_case_worst_regret = [0.0_f64; 3];
    let mut prepared = [0usize; 3];
    let mut reviewed = [0usize; 3];
    let mut false_negatives = [0usize; 3];
    let mut false_preparations = [0usize; 3];

    for case in cases {
        let weight = case.importance_weight;
        if case.action == "abstain" {
            let review = case.review_fallback.as_deref().unwrap_or("no_action");
            for index in 0..3 {
                strategy_costs[index] += weight
                    * actual_loss(review, case.observed_event, profile, profile.deferral_cost);
                reviewed[index] += 1;
            }
            continue;
        }

        let point = case.point.unwrap_or(0.0);
        let lower = case.lower.unwrap_or(point);
        let upper = case.upper.unwrap_or(point);
        let width = upper - lower;
        width_sum += weight * width;
        max_width = max_width.max(width);
        width_samples.push((width, weight));

        let actions = strategy_actions(profile, point, lower, upper);
        if actions.nominal != actions.worst_case_expected {
            nominal_vs_robust += 1;
        }
        if actions.nominal != actions.minimax_regret {
            nominal_vs_minimax += 1;
        }
        if actions.worst_case_expected != actions.minimax_regret {
            robust_vs_minimax += 1;
        }

        let selected = [
            actions.nominal,
            actions.worst_case_expected,
            actions.minimax_regret,
        ];
        for (index, action) in selected.iter().enumerate() {
            strategy_costs[index] += weight * actual_loss(*action, case.observed_event, profile, 0.0);
            worst_expected_sums[index] +=
                weight * worst_case_expected_cost(*action, upper, profile);
            let worst_regret = worst_case_regret(*action, lower, upper, profile);
            worst_regret_sums[index] += weight * worst_regret;
            max_case_worst_regret[index] = max_case_worst_regret[index].max(worst_regret);
            if *action == "prepare" {
                prepared[index] += 1;
                if !case.observed_event {
                    false_preparations[index] += 1;
                }
            } else if case.observed_event {
                false_negatives[index] += 1;
            }
        }
    }

    let climatology_action = if reference_probability >= profile.threshold {
        "prepare"
    } else {
        "no_action"
    };
    let mut climatology_cost = 0.0;
    let mut always_no_action_cost = 0.0;
    let mut always_prepare_cost = 0.0;
    let mut oracle_cost = 0.0;
    for case in cases {
        let weight = case.importance_weight;
        let oracle_action = if case.observed_event
            && profile.event_loss_without_action
                > profile.readiness_cost + profile.event_loss_with_action
        {
            "prepare"
        } else {
            "no_action"
        };
        climatology_cost += weight
            * actual_loss(climatology_action, case.observed_event, profile, 0.0);
        always_no_action_cost +=
            weight * actual_loss("no_action", case.observed_event, profile, 0.0);
        always_prepare_cost +=
            weight * actual_loss("prepare", case.observed_event, profile, 0.0);
        oracle_cost += weight * actual_loss(oracle_action, case.observed_event, profile, 0.0);
    }

    let normalized = |value: f64| {
        (total_weight > 0.0).then_some(value / total_weight)
    };
    let climatology_cost = normalized(climatology_cost);
    let always_no_action_cost = normalized(always_no_action_cost);
    let always_prepare_cost = normalized(always_prepare_cost);
    let oracle_cost = normalized(oracle_cost);
    let denominator = climatology_cost.zip(oracle_cost)
        .map(|(climatology, oracle)| climatology - oracle);

    let mut strategy_results = Map::new();
    for (index, strategy) in STRATEGIES.iter().enumerate() {
        let realized_cost = normalized(strategy_costs[index]);
        let relative_value = denominator
            .zip(climatology_cost.zip(realized_cost))
            .and_then(|(denominator, (climatology, candidate))| {
                (denominator > 0.0).then_some((climatology - candidate) / denominator)
            });
        let worst_expected_mean = (forecast_weight > 0.0)
            .then_some(worst_expected_sums[index] / forecast_weight);
        let worst_regret_mean = (forecast_weight > 0.0)
            .then_some(worst_regret_sums[index] / forecast_weight);
        strategy_results.insert(
            (*strategy).to_owned(),
            json!({
                "realized_policy_cost": realized_cost,
                "relative_economic_value": relative_value,
                "regret_vs_perfect_information": realized_cost.zip(oracle_cost)
                    .map(|(candidate, oracle)| candidate - oracle),
                "weighted_mean_worst_case_expected_cost_on_issued": worst_expected_mean,
                "weighted_mean_worst_case_regret_on_issued": worst_regret_mean,
                "maximum_case_worst_case_regret": max_case_worst_regret[index],
                "prepared_decisions": prepared[index],
                "reviewed_cases": reviewed[index],
                "false_negative_cases": false_negatives[index],
                "false_prepare_cases": false_preparations[index]
            }),
        );
    }

    let p90_width = weighted_quantile(&width_samples, 0.9);
    let mean_width = (forecast_weight > 0.0).then_some(width_sum / forecast_weight);
    let base_costs = json!({
        "training_climatology": climatology_cost,
        "always_no_action": always_no_action_cost,
        "always_prepare": always_prepare_cost,
        "perfect_information_oracle_lower_bound": oracle_cost
    });

    json!({
        "cases": cases.len(),
        "issued_forecasts": issued.len(),
        "abstentions": cases.len() - issued.len(),
        "raw_forecast_coverage": (cases.len() > 0).then_some(issued.len() as f64 / cases.len() as f64),
        "weighted_forecast_coverage": (total_weight > 0.0).then_some(forecast_weight / total_weight),
        "importance_weight_sum": total_weight,
        "effective_sample_size": total_ess,
        "forecast_effective_sample_size": forecast_ess,
        "mean_probability_interval_width": mean_width,
        "p90_probability_interval_width": p90_width,
        "max_probability_interval_width": (forecast_weight > 0.0).then_some(max_width),
        "nominal_vs_robust_expected_action_disagreements": nominal_vs_robust,
        "nominal_vs_minimax_regret_action_disagreements": nominal_vs_minimax,
        "robust_expected_vs_minimax_regret_action_disagreements": robust_vs_minimax,
        "baseline_policy_costs": base_costs,
        "strategies": strategy_results
    })
}

/// Evaluate nominal and ambiguity-aware decisions separately for every
/// decision profile and every declared shift regime.
pub fn evaluate_ambiguity_robust_decisions(fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    scan_forbidden(fixture, "", &mut errors);

    if string(fixture, "benchmark_id")
        != Some("PRAXIS-NRES-012-AMBIGUITY-ROBUST-DECISION-001")
    {
        errors.push("unexpected NRES-012 benchmark_id".to_owned());
    }
    if string(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-012 schema_version".to_owned());
    }
    if string(fixture, "status") != Some("synthetic-demonstration-for-review")
        || string(fixture, "evaluation_split") != Some("public-development-only")
        || fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false)
    {
        errors.push("NRES-012 must remain synthetic, public-development-only and inactive".to_owned());
    }

    let Some(source) = fixture.get("source_fixture") else {
        return Err(vec!["NRES-012 source fixture metadata is missing".to_owned()]);
    };
    if string(source, "benchmark_id")
        != Some("PRAXIS-NRES-011-RELIABILITY-RESOLUTION-DECISION-VALUE-001")
        || string(source, "fixture_blob_sha") != Some(SOURCE_FIXTURE_BLOB_SHA)
        || source.get("source_case_count").and_then(Value::as_u64) != Some(128)
    {
        errors.push("NRES-012 source fixture identity, blob SHA or case count mismatch".to_owned());
    }
    let source_fixture: Value = serde_json::from_str(SOURCE_FIXTURE)
        .expect("pinned NRES-011 source fixture must remain valid JSON");
    let source_rows = source_fixture["evaluation_cases"].as_array()
        .expect("pinned NRES-011 source cases must be an array");
    if source_rows.len() != 128 {
        errors.push("pinned NRES-011 source fixture case count changed".to_owned());
    }
    let source_map = match source_case_map(&source_fixture) {
        Ok(map) => map,
        Err(source_errors) => return Err(source_errors),
    };

    let Some(target) = fixture.get("target") else {
        return Err(vec!["NRES-012 target definition is missing".to_owned()]);
    };
    let target_threshold = number(target, "threshold");
    let target_horizon = number(target, "horizon_hours");
    if target_threshold.is_none_or(|value| !value.is_finite() || value < 0.0)
        || target_horizon.is_none_or(|value| !value.is_finite() || value <= 0.0)
        || string(target, "unit") != Some("normalized-service-deficit-hours")
    {
        errors.push("NRES-012 target threshold, horizon or unit is invalid".to_owned());
    }
    let target_threshold = target_threshold.unwrap_or(48.0);
    let target_horizon = target_horizon.unwrap_or(72.0);
    if number(&source_fixture["target"], "threshold")
        .is_none_or(|value| (value - target_threshold).abs() > 1e-12)
        || number(&source_fixture["target"], "horizon_hours")
            .is_none_or(|value| (value - target_horizon).abs() > 1e-12)
        || string(&source_fixture["target"], "unit") != string(target, "unit")
    {
        errors.push("NRES-012 target definition differs from pinned NRES-011 source".to_owned());
    }
    if fixture.get("sampling_design") != source_fixture.get("sampling_design") {
        errors.push("NRES-012 sampling design differs from pinned NRES-011 source".to_owned());
    }
    if fixture.get("calibration_reference") != source_fixture.get("calibration_reference") {
        errors.push("NRES-012 training reference differs from pinned NRES-011 source".to_owned());
    }
    if fixture.get("decision_profiles") != source_fixture.get("decision_profiles") {
        errors.push("NRES-012 decision profiles differ from pinned NRES-011 source".to_owned());
    }

    let Some(protocol) = fixture.get("protocol") else {
        return Err(vec!["NRES-012 protocol is missing".to_owned()]);
    };
    let origin = number(protocol, "forecast_origin_hour");
    if origin.is_none_or(|value| !value.is_finite() || value < 0.0)
        || number(protocol, "forecast_horizon_hours")
            .is_none_or(|value| (value - target_horizon).abs() > 1e-12)
        || number(protocol, "event_threshold")
            .is_none_or(|value| (value - target_threshold).abs() > 1e-12)
        || string(protocol, "ambiguity_metric")
            != Some("total_variation_distance_between_bernoulli_event_distributions")
    {
        errors.push("NRES-012 protocol origin/horizon/threshold/ambiguity metric is invalid".to_owned());
    }
    let origin = origin.unwrap_or(0.0);

    let Some(reference) = fixture.get("calibration_reference") else {
        return Err(vec!["NRES-012 calibration reference is missing".to_owned()]);
    };
    let sample_count = reference.get("sample_count").and_then(Value::as_u64);
    let event_count = reference.get("exceedance_count").and_then(Value::as_u64);
    let reference_probability = number(reference, "exceedance_probability");
    if sample_count.is_none_or(|value| value == 0)
        || event_count.is_none_or(|value| value > sample_count.unwrap_or(0))
        || reference_probability.is_none_or(|value| !value.is_finite() || !(0.0..=1.0).contains(&value))
        || string(reference, "status") != Some("synthetic-training-reference")
    {
        errors.push("NRES-012 training reference is invalid".to_owned());
    }
    let reference_probability = reference_probability.unwrap_or(0.04);
    if let (Some(n), Some(events)) = (sample_count, event_count) {
        if ((events as f64 / n as f64) - reference_probability).abs() > 1e-12 {
            errors.push("NRES-012 reference probability differs from its numerator/denominator".to_owned());
        }
    }

    let Some(design) = fixture.get("sampling_design") else {
        return Err(vec!["NRES-012 sampling design is missing".to_owned()]);
    };
    let Some(strata) = design.get("strata").and_then(Value::as_array) else {
        return Err(vec!["NRES-012 sampling strata are missing".to_owned()]);
    };
    let mut stratum_map: BTreeMap<String, (f64, f64)> = BTreeMap::new();
    for stratum in strata {
        let id = string(stratum, "sampling_stratum");
        let proposal = number(stratum, "proposal_probability");
        let target_probability = number(stratum, "target_probability");
        if id.is_none_or(str::is_empty)
            || proposal.is_none_or(|value| !value.is_finite() || value <= 0.0 || value > 1.0)
            || target_probability.is_none_or(|value| !value.is_finite() || value <= 0.0 || value > 1.0)
        {
            errors.push("NRES-012 sampling stratum/probabilities are invalid".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if stratum_map.insert(id, (proposal.unwrap(), target_probability.unwrap())).is_some() {
            errors.push("NRES-012 duplicate sampling stratum".to_owned());
        }
    }
    if stratum_map.len() != 2
        || !stratum_map.contains_key("routine")
        || !stratum_map.contains_key("tail_enriched")
        || (stratum_map.values().map(|value| value.0).sum::<f64>() - 1.0).abs() > 1e-12
        || (stratum_map.values().map(|value| value.1).sum::<f64>() - 1.0).abs() > 1e-12
    {
        errors.push("NRES-012 routine/tail_enriched proposal and target masses must each sum to one".to_owned());
    }

    let Some(regime_values) = fixture.get("regimes").and_then(Value::as_array) else {
        return Err(vec!["NRES-012 regime definitions are missing".to_owned()]);
    };
    let mut regime_radii = BTreeMap::new();
    for regime in regime_values {
        let id = string(regime, "regime_id");
        let ambiguity = regime.get("ambiguity_set");
        let kind = ambiguity.and_then(|value| string(value, "kind"));
        let radius = ambiguity.and_then(|value| number(value, "total_variation_radius"));
        if id.is_none_or(|value| !REGIMES.contains(&value))
            || kind != Some("bernoulli_total_variation_ball")
            || radius.is_none_or(|value| !value.is_finite() || !(0.0..=1.0).contains(&value))
        {
            errors.push("NRES-012 regime has an invalid ID or Bernoulli TV radius".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if regime_radii.insert(id.clone(), radius.unwrap()).is_some() {
            errors.push(format!("duplicate NRES-012 regime: {id}"));
        }
    }
    if regime_radii.len() != REGIMES.len()
        || REGIMES.iter().any(|regime| !regime_radii.contains_key(*regime))
    {
        errors.push("NRES-012 requires all four regimes exactly once".to_owned());
    }

    let Some(profile_values) = fixture.get("decision_profiles").and_then(Value::as_array) else {
        return Err(vec!["NRES-012 decision_profiles are missing".to_owned()]);
    };
    if profile_values.len() < 3 {
        errors.push("NRES-012 requires at least three decision profiles".to_owned());
    }
    let mut profile_ids = BTreeSet::new();
    let mut profiles = Vec::new();
    for value in profile_values {
        let id = string(value, "profile_id");
        let readiness = number(value, "readiness_cost");
        let loss_without = number(value, "event_loss_without_action");
        let loss_with = number(value, "event_loss_with_action");
        let deferral = number(value, "deferral_cost");
        let threshold = number(value, "decision_threshold_probability");
        if id.is_none_or(str::is_empty)
            || readiness.is_none_or(|v| !v.is_finite() || v < 0.0)
            || loss_without.is_none_or(|v| !v.is_finite() || v <= 0.0)
            || loss_with.is_none_or(|v| !v.is_finite() || v < 0.0)
            || deferral.is_none_or(|v| !v.is_finite() || v < 0.0)
            || threshold.is_none_or(|v| !v.is_finite() || !(0.0..=1.0).contains(&v))
            || string(value, "cost_unit") != Some("normalized-planning-loss-units")
        {
            errors.push("NRES-012 decision profile has invalid costs or threshold".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if !profile_ids.insert(id.clone()) {
            errors.push(format!("duplicate NRES-012 profile ID: {id}"));
        }
        let readiness = readiness.unwrap();
        let loss_without = loss_without.unwrap();
        let loss_with = loss_with.unwrap();
        if loss_with >= loss_without {
            errors.push(format!("{id}: preparation must strictly reduce event loss"));
            continue;
        }
        let derived_threshold = readiness / (loss_without - loss_with);
        if (threshold.unwrap() - derived_threshold).abs() > 1e-12 {
            errors.push(format!("{id}: threshold differs from cost/loss ratio"));
        }
        profiles.push(DecisionProfile {
            id,
            readiness_cost: readiness,
            event_loss_without_action: loss_without,
            event_loss_with_action: loss_with,
            deferral_cost: deferral.unwrap(),
            threshold: derived_threshold,
        });
    }

    let Some(rows) = fixture.get("evaluation_cases").and_then(Value::as_array) else {
        return Err(vec!["NRES-012 evaluation_cases are missing".to_owned()]);
    };
    if rows.len() != 128 {
        errors.push("NRES-012 requires exactly 128 source-linked cases".to_owned());
    }
    let mut case_ids = BTreeSet::new();
    let mut source_ids = BTreeSet::new();
    let mut nres010_source_ids = BTreeSet::new();
    let mut regime_strata: BTreeMap<String, BTreeMap<String, usize>> = BTreeMap::new();
    let mut cases = Vec::new();
    let mut mandatory_abstentions = 0usize;
    let mut optional_abstentions = 0usize;

    for row in rows {
        let case_id = string(row, "case_id").unwrap_or("UNKNOWN").to_owned();
        let source_case_id = string(row, "source_case_id").unwrap_or("").to_owned();
        let nres010_source_id = string(row, "nres010_source_case_id").unwrap_or("").to_owned();
        if string(row, "case_id").is_none() || !case_ids.insert(case_id.clone()) {
            errors.push(format!("{case_id}: case ID is absent or duplicated"));
        }
        if source_case_id.is_empty() || !source_ids.insert(source_case_id.clone()) {
            errors.push(format!("{case_id}: NRES-011 source case ID is absent or duplicated"));
        }
        if nres010_source_id.is_empty() || !nres010_source_ids.insert(nres010_source_id.clone()) {
            errors.push(format!("{case_id}: NRES-010 lineage ID is absent or duplicated"));
        }
        let Some(source_case) = source_map.get(&source_case_id) else {
            errors.push(format!("{case_id}: source_case_id does not exist in pinned NRES-011 fixture"));
            continue;
        };
        if string(source_case, "source_case_id") != Some(nres010_source_id.as_str()) {
            errors.push(format!("{case_id}: NRES-010 source lineage differs from pinned NRES-011 record"));
        }

        let regime = string(row, "regime_id");
        if regime.is_none_or(|value| !regime_radii.contains_key(value)) {
            errors.push(format!("{case_id}: regime is not declared"));
            continue;
        }
        let regime = regime.unwrap().to_owned();
        let stratum = string(row, "sampling_stratum");
        let Some((proposal, target_probability)) =
            stratum.and_then(|name| stratum_map.get(name)).copied()
        else {
            errors.push(format!("{case_id}: sampling stratum is not declared"));
            continue;
        };
        if number(row, "proposal_stratum_probability").is_none_or(|value| (value - proposal).abs() > 1e-12)
            || number(row, "target_stratum_probability").is_none_or(|value| (value - target_probability).abs() > 1e-12)
        {
            errors.push(format!("{case_id}: case importance-sampling masses drift from frozen design"));
        }
        let stratum = stratum.unwrap().to_owned();
        *regime_strata.entry(regime.clone()).or_default().entry(stratum).or_default() += 1;

        if number(row, "forecast_origin_hour") != Some(origin)
            || number(row, "forecast_horizon_hours").is_none_or(|value| (value - target_horizon).abs() > 1e-12)
        {
            errors.push(format!("{case_id}: forecast origin/horizon drift"));
        }

        let Some(evaluator) = row.get("evaluator") else {
            errors.push(format!("{case_id}: evaluator-owned data is missing"));
            continue;
        };
        let qualification = string(evaluator, "qualification_state");
        let severity = number(evaluator, "observed_normalized_service_deficit_hours");
        if qualification.is_none_or(str::is_empty)
            || severity.is_none_or(|value| !value.is_finite() || value < 0.0)
        {
            errors.push(format!("{case_id}: qualification or observed severity is invalid"));
            continue;
        }
        let qualification = qualification.unwrap().to_owned();
        let observed_event = severity.unwrap() > target_threshold;
        let review_value = evaluator.get("review_fallback_action");
        let review_fallback = review_value.and_then(Value::as_str);

        let Some(candidate) = row.get("candidate_output") else {
            errors.push(format!("{case_id}: candidate output is missing"));
            continue;
        };
        let action = string(candidate, "action");
        let probability_value = candidate.get("predicted_exceedance_probability");
        let lower_value = candidate.get("probability_lower");
        let upper_value = candidate.get("probability_upper");
        let radius_value = candidate.get("ambiguity_radius");
        let reason_value = candidate.get("abstention_reason");
        let point = probability_value.and_then(Value::as_f64);
        let lower = lower_value.and_then(Value::as_f64);
        let upper = upper_value.and_then(Value::as_f64);
        let radius = radius_value.and_then(Value::as_f64);
        let reason = reason_value.and_then(Value::as_str);
        let required_radius = regime_radii.get(&regime).copied().unwrap_or(0.0);

        let mandatory = matches!(
            qualification.as_str(),
            "missing_critical_input" | "conflicting_sources" | "unsupported_distribution_shift"
        );
        match (mandatory, action) {
            (true, Some("abstain")) => {
                if !probability_value.is_some_and(Value::is_null)
                    || !lower_value.is_some_and(Value::is_null)
                    || !upper_value.is_some_and(Value::is_null)
                    || !radius_value.is_some_and(Value::is_null)
                    || reason != Some(qualification.as_str())
                    || review_fallback.is_none_or(|value| !matches!(value, "prepare" | "no_action"))
                {
                    errors.push(format!("{case_id}: mandatory abstention requires null interval fields, matching reason and reviewer fallback"));
                } else {
                    mandatory_abstentions += 1;
                }
            }
            (true, Some("forecast")) => {
                errors.push(format!("{case_id}: forecast emitted for mandatory-abstention qualification"));
            }
            (true, _) => errors.push(format!("{case_id}: action must be forecast or abstain")),
            (false, Some("forecast")) => {
                let valid_numbers = point.is_some_and(|v| v.is_finite() && (0.0..=1.0).contains(&v))
                    && lower.is_some_and(|v| v.is_finite() && (0.0..=1.0).contains(&v))
                    && upper.is_some_and(|v| v.is_finite() && (0.0..=1.0).contains(&v))
                    && radius.is_some_and(|v| v.is_finite() && (0.0..=1.0).contains(&v));
                let formula_matches = point.zip(radius).zip(lower.zip(upper)).is_some_and(
                    |((point, radius), (lower, upper))| {
                        let expected_lower = (point - radius).max(0.0);
                        let expected_upper = (point + radius).min(1.0);
                        (lower - expected_lower).abs() <= 1e-12
                            && (upper - expected_upper).abs() <= 1e-12
                            && lower <= point
                            && point <= upper
                    },
                );
                if qualification != "qualified"
                    || !valid_numbers
                    || !formula_matches
                    || radius.is_none_or(|value| (value - required_radius).abs() > 1e-12)
                    || !reason_value.is_some_and(Value::is_null)
                    || !review_value.is_some_and(Value::is_null)
                {
                    errors.push(format!("{case_id}: forecast interval/radius/qualification/review fields are invalid"));
                }
            }
            (false, Some("abstain")) => {
                if qualification != "qualified"
                    || !probability_value.is_some_and(Value::is_null)
                    || !lower_value.is_some_and(Value::is_null)
                    || !upper_value.is_some_and(Value::is_null)
                    || !radius_value.is_some_and(Value::is_null)
                    || reason != Some("model_uncertainty")
                    || review_fallback.is_none_or(|value| !matches!(value, "prepare" | "no_action"))
                {
                    errors.push(format!("{case_id}: optional abstention requires null interval, model_uncertainty and review fallback"));
                } else {
                    optional_abstentions += 1;
                }
            }
            (false, _) => errors.push(format!("{case_id}: action must be forecast or abstain")),
        }

        // The entire NRES-012 row is a deterministic projection of the pinned
        // NRES-011 case, except for the separately authored regime TV interval.
        if string(source_case, "regime_id") != Some(regime.as_str())
            || string(source_case, "sampling_stratum") != Some(string(row, "sampling_stratum").unwrap_or(""))
            || number(source_case, "proposal_stratum_probability") != number(row, "proposal_stratum_probability")
            || number(source_case, "target_stratum_probability") != number(row, "target_stratum_probability")
            || number(source_case, "forecast_origin_hour") != number(row, "forecast_origin_hour")
            || number(source_case, "forecast_horizon_hours") != number(row, "forecast_horizon_hours")
            || source_case["evaluator"]["qualification_state"] != evaluator["qualification_state"]
            || source_case["evaluator"]["observed_normalized_service_deficit_hours"] != evaluator["observed_normalized_service_deficit_hours"]
            || source_case["evaluator"]["review_fallback_action"] != evaluator["review_fallback_action"]
            || source_case["candidate_output"]["action"] != candidate["action"]
            || source_case["candidate_output"]["predicted_exceedance_probability"] != candidate["predicted_exceedance_probability"]
            || source_case["candidate_output"]["abstention_reason"] != candidate["abstention_reason"]
        {
            errors.push(format!("{case_id}: evaluator/source/point forecast fields drift from pinned NRES-011 record"));
        }

        cases.push(EvaluationCase {
            regime_id: regime,
            importance_weight: target_probability / proposal,
            observed_event,
            action: action.unwrap_or("invalid").to_owned(),
            point,
            lower,
            upper,
            review_fallback: review_fallback.map(str::to_owned),
        });
    }

    if source_ids.len() != source_rows.len() || nres010_source_ids.len() != source_rows.len() {
        errors.push("NRES-012 must contain every pinned NRES-011/NRES-010 source case exactly once".to_owned());
    }

    let expected_weight = number(design, "expected_total_weight_per_regime");
    if expected_weight.is_none_or(|value| !value.is_finite() || value <= 0.0) {
        errors.push("expected_total_weight_per_regime must be finite and positive".to_owned());
    }
    let expected_weight = expected_weight.unwrap_or(32.0);
    for regime in REGIMES {
        let counts = regime_strata.get(*regime);
        let count = counts.map(|values| values.values().sum::<usize>()).unwrap_or(0);
        if count == 0 {
            errors.push(format!("{regime}: empty NRES-012 regime cohort"));
            continue;
        }
        if let Some(counts) = counts {
            for (stratum, (proposal, _)) in &stratum_map {
                let observed_proposal = counts.get(stratum).copied().unwrap_or(0) as f64 / count as f64;
                if (observed_proposal - proposal).abs() > 1e-12 {
                    errors.push(format!("{regime}/{stratum}: proposal allocation drift"));
                }
            }
        }
        let weight_sum = rows
            .iter()
            .filter(|row| string(row, "regime_id") == Some(*regime))
            .filter_map(|row| {
                let proposal = number(row, "proposal_stratum_probability")?;
                let target = number(row, "target_stratum_probability")?;
                (proposal > 0.0).then_some(target / proposal)
            })
            .sum::<f64>();
        if (weight_sum - expected_weight).abs() > 1e-9 {
            errors.push(format!("{regime}: total importance weight differs from declared regime weight"));
        }
        if !cases.iter().any(|case| case.regime_id.as_str() == *regime && case.action == "forecast") {
            errors.push(format!("{regime}: no forecasts issued"));
        }
    }
    if cases.iter().all(|case| case.action == "abstain") {
        errors.push("NRES-012 cannot abstain on every case".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let overall: Vec<Value> = profiles.iter().map(|profile| {
        let mut result = summarize_cases(&cases, profile, reference_probability);
        if let Some(map) = result.as_object_mut() {
            map.insert("profile_id".to_owned(), json!(profile.id));
            map.insert("decision_threshold_probability".to_owned(), json!(profile.threshold));
        }
        result
    }).collect();

    let per_regime: Vec<Value> = REGIMES.iter().map(|regime| {
        let regime_cases: Vec<EvaluationCase> = cases
            .iter()
            .filter(|case| case.regime_id == *regime)
            .cloned()
            .collect();
        let profile_results: Vec<Value> = profiles.iter().map(|profile| {
            let mut result = summarize_cases(&regime_cases, profile, reference_probability);
            if let Some(map) = result.as_object_mut() {
                map.insert("profile_id".to_owned(), json!(profile.id));
                map.insert("decision_threshold_probability".to_owned(), json!(profile.threshold));
            }
            result
        }).collect();
        json!({"regime_id": regime, "profiles": profile_results})
    }).collect();

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-012-AMBIGUITY-ROBUST-DECISION-001",
        "interpretation": "public synthetic Bernoulli TV ambiguity comparison only; not full Wasserstein DRO, a confidence set or an operational recommendation",
        "total_cases": cases.len(),
        "mandatory_abstentions": mandatory_abstentions,
        "optional_abstentions": optional_abstentions,
        "overall_by_profile": overall,
        "per_regime": per_regime,
        "aggregation_policy": "report strategy, regime and profile separately; no cross-profile composite score"
    }))
}

#[cfg(test)]
mod tests {
    use super::{evaluate_ambiguity_robust_decisions, strategy_actions, DecisionProfile};
    use serde_json::{json, Value};

    const FIXTURE: &str = include_str!("../../../examples/courses/civic-resilience-ambiguity-robust-decision-fixture-001.json");
    const CONFORMANCE: &str = include_str!("../../../examples/courses/civic-resilience-ambiguity-robust-decision-conformance-001.json");

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("NRES-012 fixture must be valid JSON")
    }

    fn first_forecast(value: &mut Value) -> &mut Value {
        value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["candidate_output"]["action"] == "forecast").unwrap()
    }

    fn profile() -> DecisionProfile {
        DecisionProfile {
            id: "test".to_owned(),
            readiness_cost: 1.0,
            event_loss_without_action: 10.0,
            event_loss_with_action: 2.0,
            deferral_cost: 0.5,
            threshold: 0.125,
        }
    }

    #[test]
    fn derives_128_case_three_policy_report() {
        let report = evaluate_ambiguity_robust_decisions(&fixture()).expect("fixture valid");
        assert_eq!(report["total_cases"].as_u64(), Some(128));
        assert_eq!(report["mandatory_abstentions"].as_u64(), Some(4));
        assert_eq!(report["optional_abstentions"].as_u64(), Some(1));
        assert_eq!(report["overall_by_profile"].as_array().unwrap().len(), 3);
        assert_eq!(report["per_regime"].as_array().unwrap().len(), 4);
        assert!(report.get("overall_resilience_score").is_none());
    }

    #[test]
    fn matches_every_published_overall_and_regime_golden() {
        let report = evaluate_ambiguity_robust_decisions(&fixture()).expect("fixture valid");
        let corpus: Value = serde_json::from_str(CONFORMANCE).expect("conformance JSON valid");
        let expected = &corpus["expected_metrics"];
        let tolerance = corpus["conformance_targets"]["golden_metric_absolute_tolerance"].as_f64().unwrap_or(1e-12);
        let compare = |actual: &Value, expected: &Value| {
            for key in [
                "cases", "issued_forecasts", "abstentions", "raw_forecast_coverage",
                "weighted_forecast_coverage", "importance_weight_sum", "effective_sample_size",
                "forecast_effective_sample_size", "mean_probability_interval_width",
                "p90_probability_interval_width", "max_probability_interval_width"
            ] {
                if expected[key].is_null() {
                    assert!(actual[key].is_null(), "{key}");
                } else if expected[key].is_number() {
                    assert!((actual[key].as_f64().unwrap() - expected[key].as_f64().unwrap()).abs() <= tolerance, "{key}");
                } else {
                    assert_eq!(actual[key], expected[key], "{key}");
                }
            }
            for key in [
                "nominal_vs_robust_expected_action_disagreements",
                "nominal_vs_minimax_regret_action_disagreements",
                "robust_expected_vs_minimax_regret_action_disagreements"
            ] {
                assert_eq!(actual[key], expected[key], "{key}");
            }
            for key in ["training_climatology", "always_no_action", "always_prepare", "perfect_information_oracle_lower_bound"] {
                assert!((actual["baseline_policy_costs"][key].as_f64().unwrap()
                    - expected["baseline_policy_costs"][key].as_f64().unwrap()).abs() <= tolerance, "{key}");
            }
            for strategy in super::STRATEGIES {
                let a = &actual["strategies"][*strategy];
                let e = &expected["strategies"][*strategy];
                for key in [
                    "realized_policy_cost", "relative_economic_value", "regret_vs_perfect_information",
                    "weighted_mean_worst_case_expected_cost_on_issued",
                    "weighted_mean_worst_case_regret_on_issued", "maximum_case_worst_case_regret"
                ] {
                    assert!((a[key].as_f64().unwrap() - e[key].as_f64().unwrap()).abs() <= tolerance, "{strategy}.{key}");
                }
                for key in ["prepared_decisions", "reviewed_cases", "false_negative_cases", "false_prepare_cases"] {
                    assert_eq!(a[key], e[key], "{strategy}.{key}");
                }
            }
        };
        for actual in report["overall_by_profile"].as_array().unwrap() {
            let id = actual["profile_id"].as_str().unwrap();
            compare(actual, &expected["overall"][id]);
        }
        for actual_regime in report["per_regime"].as_array().unwrap() {
            let regime = actual_regime["regime_id"].as_str().unwrap();
            for actual_profile in actual_regime["profiles"].as_array().unwrap() {
                let id = actual_profile["profile_id"].as_str().unwrap();
                compare(actual_profile, &expected["per_regime"][regime][id]);
            }
        }
    }

    #[test]
    fn nominal_policy_prepares_at_threshold_equality() {
        let p = profile();
        let a = strategy_actions(&p, 0.125, 0.10, 0.15);
        assert_eq!(a.nominal, "prepare");
    }

    #[test]
    fn worst_case_expected_loss_tie_resolves_to_no_action() {
        let p = profile();
        let a = strategy_actions(&p, 0.10, 0.05, 0.125);
        assert_eq!(a.worst_case_expected, "no_action");
    }

    #[test]
    fn minimax_regret_tie_resolves_to_no_action() {
        let p = profile();
        let a = strategy_actions(&p, 0.125, 0.125, 0.125);
        assert_eq!(a.minimax_regret, "no_action");
        assert!((a.regret_prepare - a.regret_no_action).abs() < 1e-12);
    }

    #[test]
    fn worst_case_expected_cost_uses_upper_probability() {
        let p = profile();
        let no_action = super::worst_case_expected_cost("no_action", 0.2, &p);
        let prepare = super::worst_case_expected_cost("prepare", 0.2, &p);
        assert!((no_action - 2.0).abs() < 1e-12);
        assert!((prepare - 1.4).abs() < 1e-12);
    }

    #[test]
    fn rejects_point_probability_above_one() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["predicted_exceedance_probability"] = json!(1.01);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_negative_point_probability() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["predicted_exceedance_probability"] = json!(-0.01);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_negative_lower_bound() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["probability_lower"] = json!(-0.01);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_upper_bound_above_one() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["probability_upper"] = json!(1.01);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_reversed_interval() {
        let mut value = fixture();
        let row = first_forecast(&mut value);
        row["candidate_output"]["probability_lower"] = json!(0.8);
        row["candidate_output"]["probability_upper"] = json!(0.2);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_point_below_lower_bound() {
        let mut value = fixture();
        let row = first_forecast(&mut value);
        row["candidate_output"]["probability_lower"] = json!(0.5);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_point_above_upper_bound() {
        let mut value = fixture();
        let row = first_forecast(&mut value);
        row["candidate_output"]["probability_upper"] = json!(0.001);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_negative_ambiguity_radius() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["ambiguity_radius"] = json!(-0.1);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_regime_radius_drift() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["ambiguity_radius"] = json!(0.5);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_interval_not_derived_from_tv_radius() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["probability_upper"] = json!(0.9);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_missing_interval_bound() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["probability_lower"] = Value::Null;
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_interval_emitted_on_abstention() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["candidate_output"]["action"] == "abstain").unwrap();
        row["candidate_output"]["probability_lower"] = json!(0.0);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_forecast_on_missing_critical_input() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["evaluator"]["qualification_state"] == "missing_critical_input").unwrap();
        row["candidate_output"]["action"] = json!("forecast");
        row["candidate_output"]["predicted_exceedance_probability"] = json!(0.1);
        row["candidate_output"]["probability_lower"] = json!(0.08);
        row["candidate_output"]["probability_upper"] = json!(0.12);
        row["candidate_output"]["ambiguity_radius"] = json!(0.02);
        row["candidate_output"]["abstention_reason"] = Value::Null;
        row["evaluator"]["review_fallback_action"] = Value::Null;
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_missing_abstention_reason() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["candidate_output"]["action"] == "abstain").unwrap();
        row["candidate_output"].as_object_mut().unwrap().remove("abstention_reason");
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_missing_review_fallback() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["candidate_output"]["action"] == "abstain").unwrap();
        row["evaluator"]["review_fallback_action"] = Value::Null;
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_zero_proposal_probability() {
        let mut value = fixture();
        value["sampling_design"]["strata"][0]["proposal_probability"] = json!(0.0);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_negative_target_probability() {
        let mut value = fixture();
        value["sampling_design"]["strata"][0]["target_probability"] = json!(-0.1);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_case_sampling_drift() {
        let mut value = fixture();
        value["evaluation_cases"][0]["target_stratum_probability"] = json!(0.5);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_nres011_source_case_id() {
        let mut value = fixture();
        let id = value["evaluation_cases"][0]["source_case_id"].clone();
        value["evaluation_cases"][1]["source_case_id"] = id;
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }


    #[test]
    fn rejects_target_drift_from_nres011_source() {
        let mut value = fixture();
        value["target"]["threshold"] = json!(47.0);
        value["protocol"]["event_threshold"] = json!(47.0);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_decision_profile_drift_from_nres011_source() {
        let mut value = fixture();
        value["decision_profiles"][0]["readiness_cost"] = json!(0.25);
        value["decision_profiles"][0]["decision_threshold_probability"] = json!(0.013888888888888888);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_sampling_design_drift_from_nres011_source() {
        let mut value = fixture();
        value["sampling_design"]["strata"][0]["target_probability"] = json!(0.9);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }


    #[test]
    fn rejects_source_blob_sha_drift() {
        let mut value = fixture();
        value["source_fixture"]["fixture_blob_sha"] = json!("0000000000000000000000000000000000000000");
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_source_point_forecast_drift() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["predicted_exceedance_probability"] = json!(0.99);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_profile_threshold_drift() {
        let mut value = fixture();
        value["decision_profiles"][0]["decision_threshold_probability"] = json!(0.5);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_invalid_action_loss_order() {
        let mut value = fixture();
        value["decision_profiles"][0]["event_loss_with_action"] =
            value["decision_profiles"][0]["event_loss_without_action"].clone();
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_horizon_drift() {
        let mut value = fixture();
        value["evaluation_cases"][0]["forecast_horizon_hours"] = json!(24);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_unknown_regime() {
        let mut value = fixture();
        value["evaluation_cases"][0]["regime_id"] = json!("unregistered_shift");
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_composite_score_injection() {
        let mut value = fixture();
        value["composite_score"] = json!(0.9);
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_missing_decision_profile() {
        let mut value = fixture();
        value["decision_profiles"].as_array_mut().unwrap().pop();
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }

    #[test]
    fn rejects_all_abstention() {
        let mut value = fixture();
        for row in value["evaluation_cases"].as_array_mut().unwrap() {
            row["candidate_output"]["action"] = json!("abstain");
            row["candidate_output"]["predicted_exceedance_probability"] = Value::Null;
            row["candidate_output"]["probability_lower"] = Value::Null;
            row["candidate_output"]["probability_upper"] = Value::Null;
            row["candidate_output"]["ambiguity_radius"] = Value::Null;
            let qualification = row["evaluator"]["qualification_state"].as_str().unwrap().to_owned();
            row["candidate_output"]["abstention_reason"] = json!(
                if qualification == "qualified" { "model_uncertainty".to_owned() } else { qualification }
            );
        }
        assert!(evaluate_ambiguity_robust_decisions(&value).is_err());
    }
}

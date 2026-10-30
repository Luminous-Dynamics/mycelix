//! NRES-011: forecast reliability, resolution and profile-specific decision value.
//!
//! Evaluates probabilistic forecast quality separately from actions taken under
//! explicit fictional cost/loss profiles. It is not an operational policy engine.

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

const SOURCE_FIXTURE: &str = include_str!("../../../examples/courses/civic-resilience-tail-shift-robustness-fixture-001.json");
const SOURCE_FIXTURE_BLOB_SHA: &str = "f42d045924b91f09aa5a3a455cb12fb7bb7fdb02";

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
    probability: Option<f64>,
    review_fallback_action: Option<String>,
}

#[derive(Default)]
struct ReliabilityBin {
    count: usize,
    weight: f64,
    weighted_probability: f64,
    weighted_event: f64,
}

fn effective_sample_size(weights: &[f64]) -> Option<f64> {
    let sum: f64 = weights.iter().sum();
    let sum_squares: f64 = weights.iter().map(|weight| weight * weight).sum();
    (sum_squares > 0.0).then_some(sum * sum / sum_squares)
}

fn decision_loss(action: &str, event: bool, profile: &DecisionProfile, deferral_cost: f64) -> f64 {
    let event = if event { 1.0 } else { 0.0 };
    let loss = if action == "prepare" {
        profile.readiness_cost + event * profile.event_loss_with_action
    } else {
        event * profile.event_loss_without_action
    };
    loss + deferral_cost
}

fn probability_bin(probability: f64, edges: &[f64]) -> usize {
    for index in 0..(edges.len() - 1) {
        let final_bin = index == edges.len() - 2;
        if probability >= edges[index]
            && (probability < edges[index + 1] || (final_bin && probability <= edges[index + 1]))
        {
            return index;
        }
    }
    edges.len() - 2
}

fn abstentions_weight(cases: &[EvaluationCase]) -> f64 {
    cases.iter()
        .filter(|case| case.action == "abstain")
        .map(|case| case.importance_weight)
        .sum()
}

fn summarize_cases(
    cases: &[EvaluationCase],
    profile: &DecisionProfile,
    reference_probability: f64,
    edges: &[f64],
) -> Value {
    let total_weight: f64 = cases.iter().map(|case| case.importance_weight).sum();
    let all_weights: Vec<f64> = cases.iter().map(|case| case.importance_weight).collect();
    let issued: Vec<&EvaluationCase> = cases
        .iter()
        .filter(|case| case.action == "forecast")
        .collect();
    let forecast_weights: Vec<f64> = issued.iter().map(|case| case.importance_weight).collect();
    let forecast_weight: f64 = forecast_weights.iter().sum();
    let forecast_ess = effective_sample_size(&forecast_weights);
    let forecast_coverage = (total_weight > 0.0).then_some(forecast_weight / total_weight);
    let total_event_weight: f64 = cases.iter()
        .filter(|case| case.observed_event)
        .map(|case| case.importance_weight)
        .sum();
    let total_non_event_weight: f64 = cases.iter()
        .filter(|case| !case.observed_event)
        .map(|case| case.importance_weight)
        .sum();
    let forecast_event_weight: f64 = issued.iter()
        .filter(|case| case.observed_event)
        .map(|case| case.importance_weight)
        .sum();
    let forecast_non_event_weight: f64 = issued.iter()
        .filter(|case| !case.observed_event)
        .map(|case| case.importance_weight)
        .sum();
    let abstained_event_weight = total_event_weight - forecast_event_weight;
    let abstained_non_event_weight = total_non_event_weight - forecast_non_event_weight;
    let abstained_weight = abstentions_weight(cases);

    let mut brier_sum = 0.0;
    let mut log_loss_sum = 0.0;
    let mut reference_brier_sum = 0.0;
    let mut weighted_probability_sum = 0.0;
    let mut weighted_event_sum = 0.0;
    let mut zero_probability_events = 0usize;
    let mut bins: Vec<ReliabilityBin> = (0..edges.len() - 1)
        .map(|_| ReliabilityBin::default())
        .collect();

    for case in &issued {
        let probability = case.probability.unwrap_or(0.0);
        let event = if case.observed_event { 1.0 } else { 0.0 };
        let weight = case.importance_weight;
        brier_sum += weight * (probability - event).powi(2);
        reference_brier_sum += weight * (reference_probability - event).powi(2);
        weighted_probability_sum += weight * probability;
        weighted_event_sum += weight * event;

        let likelihood = if case.observed_event { probability } else { 1.0 - probability };
        if likelihood <= 0.0 {
            zero_probability_events += 1;
        } else {
            log_loss_sum -= weight * likelihood.ln();
        }

        let bin = &mut bins[probability_bin(probability, edges)];
        bin.count += 1;
        bin.weight += weight;
        bin.weighted_probability += weight * probability;
        bin.weighted_event += weight * event;
    }

    let original_brier = (forecast_weight > 0.0).then_some(brier_sum / forecast_weight);
    let log_loss = (forecast_weight > 0.0 && zero_probability_events == 0)
        .then_some(log_loss_sum / forecast_weight);
    let reference_brier = (forecast_weight > 0.0)
        .then_some(reference_brier_sum / forecast_weight);
    let brier_skill = original_brier.zip(reference_brier).and_then(|(candidate, baseline)| {
        (baseline > 0.0).then_some(1.0 - candidate / baseline)
    });
    let forecast_event_rate = (forecast_weight > 0.0)
        .then_some(weighted_event_sum / forecast_weight);
    let mean_probability = (forecast_weight > 0.0)
        .then_some(weighted_probability_sum / forecast_weight);

    let mut reliability = 0.0;
    let mut resolution = 0.0;
    let mut binned_brier = 0.0;
    let reliability_bins: Vec<Value> = bins
        .iter()
        .enumerate()
        .map(|(index, bin)| {
            let mean_p = (bin.weight > 0.0)
                .then_some(bin.weighted_probability / bin.weight);
            let event_rate = (bin.weight > 0.0)
                .then_some(bin.weighted_event / bin.weight);
            if let (Some(mean_p), Some(event_rate), Some(overall_rate)) =
                (mean_p, event_rate, forecast_event_rate)
            {
                let mass = bin.weight / forecast_weight;
                reliability += mass * (mean_p - event_rate).powi(2);
                resolution += mass * (event_rate - overall_rate).powi(2);
                binned_brier += mass
                    * ((mean_p - event_rate).powi(2)
                        + event_rate * (1.0 - event_rate));
            }
            json!({
                "bin_id": format!("BIN-{index}"),
                "lower": edges[index],
                "upper": edges[index + 1],
                "count": bin.count,
                "weight": bin.weight,
                "mean_probability": mean_p,
                "event_rate": event_rate
            })
        })
        .collect();
    let uncertainty = forecast_event_rate.map(|rate| rate * (1.0 - rate));
    let reconstructed_brier = forecast_event_rate
        .map(|rate| reliability - resolution + rate * (1.0 - rate));
    let decomposition_identity_residual = reconstructed_brier
        .map(|reconstructed| binned_brier - reconstructed);
    let decomposition_residual = binned_brier.zip(original_brier)
        .map(|(binned, original)| binned - original);

    let mut candidate_cost = 0.0;
    let mut no_action_cost = 0.0;
    let mut always_prepare_cost = 0.0;
    let mut climatology_cost = 0.0;
    let mut oracle_cost = 0.0;
    let mut prepared_decisions = 0usize;
    let mut reviewed_cases = 0usize;
    let mut false_negative_cases = 0usize;
    let mut false_prepare_cases = 0usize;
    let climatology_action = if reference_probability >= profile.threshold {
        "prepare"
    } else {
        "no_action"
    };

    for case in cases {
        let weight = case.importance_weight;
        let action: &str;
        let deferral: f64;
        if case.action == "forecast" {
            let probability = case.probability.unwrap_or(0.0);
            action = if probability >= profile.threshold { "prepare" } else { "no_action" };
            deferral = 0.0;
        } else {
            action = case.review_fallback_action.as_deref().unwrap_or("no_action");
            deferral = profile.deferral_cost;
            reviewed_cases += 1;
        }

        if action == "prepare" {
            prepared_decisions += 1;
            if !case.observed_event { false_prepare_cases += 1; }
        } else if case.observed_event {
            false_negative_cases += 1;
        }

        candidate_cost += weight * decision_loss(action, case.observed_event, profile, deferral);
        no_action_cost += weight * decision_loss("no_action", case.observed_event, profile, 0.0);
        always_prepare_cost += weight * decision_loss("prepare", case.observed_event, profile, 0.0);
        climatology_cost += weight * decision_loss(climatology_action, case.observed_event, profile, 0.0);

        let oracle_action = if case.observed_event
            && profile.event_loss_without_action
                > profile.readiness_cost + profile.event_loss_with_action
        {
            "prepare"
        } else {
            "no_action"
        };
        oracle_cost += weight * decision_loss(oracle_action, case.observed_event, profile, 0.0);
    }

    let normalized = |value: f64| {
        (total_weight > 0.0).then_some(value / total_weight)
    };
    let candidate_cost = normalized(candidate_cost);
    let no_action_cost = normalized(no_action_cost);
    let always_prepare_cost = normalized(always_prepare_cost);
    let climatology_cost = normalized(climatology_cost);
    let oracle_cost = normalized(oracle_cost);
    let ev_denominator = climatology_cost.zip(oracle_cost).map(|(climatology, oracle)| {
        climatology - oracle
    });
    let relative_value = ev_denominator.zip(climatology_cost.zip(candidate_cost))
        .and_then(|(denominator, (climatology, candidate))| {
            (denominator > 0.0).then_some((climatology - candidate) / denominator)
        });

    json!({
        "cases": cases.len(),
        "selection_diagnostics": {
            "target_population_event_rate": (total_weight > 0.0).then_some(total_event_weight / total_weight),
            "event_forecast_coverage": (total_event_weight > 0.0).then_some(forecast_event_weight / total_event_weight),
            "non_event_forecast_coverage": (total_non_event_weight > 0.0).then_some(forecast_non_event_weight / total_non_event_weight),
            "target_event_weight": total_event_weight,
            "target_non_event_weight": total_non_event_weight,
            "forecast_event_weight": forecast_event_weight,
            "forecast_non_event_weight": forecast_non_event_weight,
            "abstained_event_weight": abstained_event_weight,
            "abstained_non_event_weight": abstained_non_event_weight,
            "abstained_weight": abstained_weight,
            "abstained_event_rate": (abstained_weight > 0.0).then_some(abstained_event_weight / abstained_weight)
        },
        "issued_forecasts": issued.len(),
        "reviewed_abstentions": reviewed_cases,
        "raw_forecast_coverage": (cases.len() > 0).then_some(issued.len() as f64 / cases.len() as f64),
        "weighted_forecast_coverage": forecast_coverage,
        "importance_weight_sum": total_weight,
        "effective_sample_size": effective_sample_size(&all_weights),
        "forecast_effective_sample_size": forecast_ess,
        "weighted_event_rate": forecast_event_rate,
        "mean_candidate_probability": mean_probability,
        "original_brier_score": original_brier,
        "log_loss": log_loss,
        "zero_probability_events": zero_probability_events,
        "climatology_brier_score": reference_brier,
        "brier_skill_score": brier_skill,
        "reliability_resolution_uncertainty": {
            "reliability": reliability,
            "resolution": resolution,
            "uncertainty": uncertainty,
            "binned_brier_score": (forecast_weight > 0.0).then_some(binned_brier),
            "murphy_reconstructed_brier_score": reconstructed_brier,
            "decomposition_identity_residual": decomposition_identity_residual,
            "original_brier_score": original_brier,
            "binned_minus_original_residual": decomposition_residual,
            "bins": reliability_bins
        },
        "policy_costs": {
            "candidate_forecast_and_review": candidate_cost,
            "always_no_action": no_action_cost,
            "always_prepare": always_prepare_cost,
            "training_climatology": climatology_cost,
            "perfect_information_oracle_lower_bound": oracle_cost
        },
        "decision_metrics": {
            "relative_economic_value": relative_value,
            "relative_economic_value_denominator": ev_denominator,
            "regret_vs_perfect_information": candidate_cost.zip(oracle_cost).map(|(c, o)| c - o),
            "candidate_savings_vs_no_action": no_action_cost.zip(candidate_cost).map(|(n, c)| n - c),
            "candidate_savings_vs_training_climatology": climatology_cost.zip(candidate_cost).map(|(n, c)| n - c),
            "prepared_decisions": prepared_decisions,
            "reviewed_cases": reviewed_cases,
            "false_negative_cases": false_negative_cases,
            "false_prepare_cases": false_prepare_cases
        }
    })
}

/// Evaluate forecast quality and decision value separately for every declared
/// profile and shift regime. No score is averaged across profiles.
pub fn evaluate_reliability_resolution_decision_value(
    fixture: &Value,
) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    scan_forbidden(fixture, "", &mut errors);

    if string(fixture, "benchmark_id")
        != Some("PRAXIS-NRES-011-RELIABILITY-RESOLUTION-DECISION-VALUE-001")
    {
        errors.push("unexpected NRES-011 benchmark_id".to_owned());
    }
    if string(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-011 schema_version".to_owned());
    }
    if string(fixture, "status") != Some("synthetic-demonstration-for-review")
        || string(fixture, "evaluation_split") != Some("public-development-only")
        || fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false)
    {
        errors.push("NRES-011 fixture must remain synthetic, public-development-only and inactive".to_owned());
    }

    let Some(source) = fixture.get("source_fixture") else {
        return Err(vec!["NRES-011 source_fixture is missing".to_owned()]);
    };
    if string(source, "benchmark_id")
        != Some("PRAXIS-NRES-010-TAIL-SHIFT-ROBUSTNESS-001")
        || string(source, "fixture_blob_sha") != Some(SOURCE_FIXTURE_BLOB_SHA)
        || source.get("source_case_count").and_then(Value::as_u64) != Some(128)
    {
        errors.push("NRES-011 source fixture identity, pinned blob SHA or source case count mismatch".to_owned());
    }
    let source_fixture: Value = serde_json::from_str(SOURCE_FIXTURE)
        .expect("pinned NRES-010 source fixture must remain valid JSON");
    let source_cases = source_fixture["evaluation_cases"].as_array()
        .expect("pinned NRES-010 source cases must be an array");
    if source_cases.len() != 128 {
        errors.push("pinned NRES-010 source fixture case count changed".to_owned());
    }
    let source_by_id: BTreeMap<String, &Value> = source_cases.iter()
        .filter_map(|case| string(case, "case_id").map(|id| (id.to_owned(), case)))
        .collect();

    let Some(target) = fixture.get("target") else {
        return Err(vec!["NRES-011 target is missing".to_owned()]);
    };
    let threshold = number(target, "threshold");
    let horizon = number(target, "horizon_hours");
    if threshold.is_none_or(|value| !value.is_finite() || value < 0.0)
        || horizon.is_none_or(|value| !value.is_finite() || value <= 0.0)
        || string(target, "unit") != Some("normalized-service-deficit-hours")
    {
        errors.push("NRES-011 target threshold, horizon or unit is invalid".to_owned());
    }
    let threshold = threshold.unwrap_or(48.0);
    let horizon = horizon.unwrap_or(72.0);

    let Some(protocol) = fixture.get("protocol") else {
        return Err(vec!["NRES-011 protocol is missing".to_owned()]);
    };
    let origin = number(protocol, "forecast_origin_hour");
    if origin.is_none_or(|value| !value.is_finite() || value < 0.0)
        || number(protocol, "forecast_horizon_hours")
            .is_none_or(|value| (value - horizon).abs() > 1e-12)
        || number(protocol, "exceedance_threshold")
            .is_none_or(|value| (value - threshold).abs() > 1e-12)
    {
        errors.push("NRES-011 forecast origin, horizon or event threshold is invalid".to_owned());
    }
    let origin = origin.unwrap_or(0.0);

    let edges: Vec<f64> = protocol.get("probability_bins").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_f64).collect();
    if edges.len() < 3 || edges.first() != Some(&0.0) || edges.last() != Some(&1.0)
        || edges.windows(2).any(|pair| pair[0] >= pair[1])
    {
        errors.push("NRES-011 probability bins must strictly increase from zero through one".to_owned());
    }

    let Some(reference) = fixture.get("calibration_reference") else {
        return Err(vec!["NRES-011 calibration reference is missing".to_owned()]);
    };
    let reference_count = reference.get("sample_count").and_then(Value::as_u64);
    let reference_events = reference.get("exceedance_count").and_then(Value::as_u64);
    let reference_probability = number(reference, "exceedance_probability");
    if reference_count.is_none_or(|value| value == 0)
        || reference_events.is_none_or(|value| value > reference_count.unwrap_or(0))
        || reference_probability.is_none_or(|value| !value.is_finite() || !(0.0..=1.0).contains(&value))
        || string(reference, "status") != Some("synthetic-training-reference")
    {
        errors.push("NRES-011 calibration reference is invalid".to_owned());
    }
    let reference_probability = reference_probability.unwrap_or(0.04);
    if let (Some(n), Some(events)) = (reference_count, reference_events) {
        if ((events as f64 / n as f64) - reference_probability).abs() > 1e-12 {
            errors.push("NRES-011 reference probability differs from numerator/denominator".to_owned());
        }
    }

    let Some(design) = fixture.get("sampling_design") else {
        return Err(vec!["NRES-011 sampling design is missing".to_owned()]);
    };
    let Some(strata) = design.get("strata").and_then(Value::as_array) else {
        return Err(vec!["NRES-011 sampling strata are missing".to_owned()]);
    };
    let mut stratum_map: BTreeMap<String, (f64, f64)> = BTreeMap::new();
    for stratum in strata {
        let id = string(stratum, "sampling_stratum");
        let proposal = number(stratum, "proposal_probability");
        let target = number(stratum, "target_probability");
        if id.is_none_or(str::is_empty)
            || proposal.is_none_or(|value| !value.is_finite() || value <= 0.0 || value > 1.0)
            || target.is_none_or(|value| !value.is_finite() || value <= 0.0 || value > 1.0)
        {
            errors.push("NRES-011 stratum ID/probabilities are invalid".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if stratum_map.insert(id, (proposal.unwrap(), target.unwrap())).is_some() {
            errors.push("NRES-011 duplicate sampling stratum".to_owned());
        }
    }
    if stratum_map.len() != 2
        || !stratum_map.contains_key("routine")
        || !stratum_map.contains_key("tail_enriched")
        || (stratum_map.values().map(|item| item.0).sum::<f64>() - 1.0).abs() > 1e-12
        || (stratum_map.values().map(|item| item.1).sum::<f64>() - 1.0).abs() > 1e-12
    {
        errors.push("NRES-011 requires normalized routine/tail_enriched proposal and target masses".to_owned());
    }

    let Some(profile_values) = fixture.get("decision_profiles").and_then(Value::as_array) else {
        return Err(vec!["NRES-011 decision_profiles are missing".to_owned()]);
    };
    if profile_values.len() < 3 {
        errors.push("NRES-011 requires at least three explicit decision profiles".to_owned());
    }
    let mut profile_ids = BTreeSet::new();
    let mut profiles = Vec::new();
    for value in profile_values {
        let id = string(value, "profile_id");
        let readiness_cost = number(value, "readiness_cost");
        let loss_no_action = number(value, "event_loss_without_action");
        let loss_with_action = number(value, "event_loss_with_action");
        let deferral_cost = number(value, "deferral_cost");
        let threshold_value = number(value, "decision_threshold_probability");
        if id.is_none_or(str::is_empty)
            || readiness_cost.is_none_or(|v| !v.is_finite() || v < 0.0)
            || loss_no_action.is_none_or(|v| !v.is_finite() || v <= 0.0)
            || loss_with_action.is_none_or(|v| !v.is_finite() || v < 0.0)
            || deferral_cost.is_none_or(|v| !v.is_finite() || v < 0.0)
            || threshold_value.is_none_or(|v| !v.is_finite() || !(0.0..=1.0).contains(&v))
            || string(value, "cost_unit") != Some("normalized-planning-loss-units")
        {
            errors.push("NRES-011 decision profile contains invalid costs or threshold".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if !profile_ids.insert(id.clone()) {
            errors.push(format!("duplicate NRES-011 profile ID: {id}"));
        }
        let readiness_cost = readiness_cost.unwrap();
        let loss_no_action = loss_no_action.unwrap();
        let loss_with_action = loss_with_action.unwrap();
        if loss_with_action >= loss_no_action {
            errors.push(format!("{id}: action must strictly reduce event loss"));
            continue;
        }
        let expected_threshold = readiness_cost / (loss_no_action - loss_with_action);
        if (threshold_value.unwrap() - expected_threshold).abs() > 1e-12 {
            errors.push(format!("{id}: decision threshold differs from declared cost/loss ratio"));
        }
        profiles.push(DecisionProfile {
            id,
            readiness_cost,
            event_loss_without_action: loss_no_action,
            event_loss_with_action: loss_with_action,
            deferral_cost: deferral_cost.unwrap(),
            threshold: expected_threshold,
        });
    }

    let Some(regime_values) = fixture.get("regimes").and_then(Value::as_array) else {
        return Err(vec!["NRES-011 regimes are missing".to_owned()]);
    };
    let mut regime_ids = BTreeSet::new();
    let mut regime_order = Vec::new();
    for regime in regime_values {
        let id = string(regime, "regime_id");
        if id.is_none_or(|v| !REGIMES.contains(&v) || !regime_ids.insert(v.to_owned())) {
            errors.push("NRES-011 unknown or duplicate regime".to_owned());
        } else {
            regime_order.push(id.unwrap().to_owned());
        }
    }
    if regime_ids.len() != REGIMES.len() {
        errors.push("NRES-011 requires all four shift regimes exactly once".to_owned());
    }

    let Some(rows) = fixture.get("evaluation_cases").and_then(Value::as_array) else {
        return Err(vec!["NRES-011 evaluation_cases are missing".to_owned()]);
    };
    if rows.is_empty() {
        errors.push("NRES-011 evaluation cohort must not be empty".to_owned());
    }

    let mut case_ids = BTreeSet::new();
    let mut source_ids = BTreeSet::new();
    let mut regime_strata: BTreeMap<String, BTreeMap<String, usize>> = BTreeMap::new();
    let mut cases = Vec::new();
    let mut mandatory_abstentions = 0usize;
    let mut optional_abstentions = 0usize;
    let mut mandatory_reasons: BTreeMap<String, usize> = BTreeMap::new();

    for row in rows {
        let case_id = string(row, "case_id").unwrap_or("UNKNOWN").to_owned();
        let source_case_id = string(row, "source_case_id").unwrap_or("").to_owned();
        if string(row, "case_id").is_none() || !case_ids.insert(case_id.clone()) {
            errors.push(format!("{case_id}: case ID is absent or duplicated"));
        }
        if source_case_id.is_empty() || !source_ids.insert(source_case_id.clone()) {
            errors.push(format!("{case_id}: source_case_id is absent or duplicated"));
        }
        let Some(source_case) = source_by_id.get(&source_case_id) else {
            errors.push(format!("{case_id}: source_case_id is not present in pinned NRES-010 fixture"));
            continue;
        };
        if string(row, "regime_id") != string(source_case, "regime_id")
            || string(row, "sampling_stratum") != string(source_case, "sampling_stratum")
            || number(row, "proposal_stratum_probability") != number(source_case, "proposal_stratum_probability")
            || number(row, "target_stratum_probability") != number(source_case, "target_stratum_probability")
            || number(row, "forecast_origin_hour") != number(source_case, "forecast_origin_hour")
            || number(row, "forecast_horizon_hours") != number(source_case, "forecast_horizon_hours")
            || row["evaluator"]["qualification_state"] != source_case["evaluator"]["qualification_state"]
            || row["evaluator"]["observed_normalized_service_deficit_hours"] != source_case["evaluator"]["observed_normalized_service_deficit_hours"]
            || row["candidate_output"]["action"] != source_case["candidate_output"]["action"]
            || row["candidate_output"]["predicted_exceedance_probability"] != source_case["candidate_output"]["predicted_exceedance_probability"]
            || row["candidate_output"]["abstention_reason"] != source_case["candidate_output"]["abstention_reason"]
        {
            errors.push(format!("{case_id}: NRES-011 scenario/forecast fields drift from the pinned NRES-010 source case"));
        }
        let regime = string(row, "regime_id");
        if regime.is_none_or(|value| !regime_ids.contains(value)) {
            errors.push(format!("{case_id}: regime is undeclared"));
            continue;
        }
        let regime = regime.unwrap().to_owned();
        let stratum = string(row, "sampling_stratum");
        let Some((proposal, target)) = stratum.and_then(|name| stratum_map.get(name)).copied() else {
            errors.push(format!("{case_id}: sampling stratum is undeclared"));
            continue;
        };
        if number(row, "proposal_stratum_probability").is_none_or(|value| (value - proposal).abs() > 1e-12)
            || number(row, "target_stratum_probability").is_none_or(|value| (value - target).abs() > 1e-12)
        {
            errors.push(format!("{case_id}: proposal/target mass differs from frozen design"));
        }
        let stratum = stratum.unwrap().to_owned();
        *regime_strata.entry(regime.clone()).or_default().entry(stratum.clone()).or_default() += 1;

        if number(row, "forecast_origin_hour") != Some(origin)
            || number(row, "forecast_horizon_hours").is_none_or(|value| (value - horizon).abs() > 1e-12)
        {
            errors.push(format!("{case_id}: forecast origin/horizon drift"));
        }

        let Some(evaluator) = row.get("evaluator") else {
            errors.push(format!("{case_id}: evaluator-owned fields are missing"));
            continue;
        };
        let qualification = string(evaluator, "qualification_state");
        let severity = number(evaluator, "observed_normalized_service_deficit_hours");
        if qualification.is_none_or(str::is_empty)
            || severity.is_none_or(|value| !value.is_finite() || value < 0.0)
        {
            errors.push(format!("{case_id}: evaluator qualification/severity invalid"));
            continue;
        }
        let qualification = qualification.unwrap().to_owned();
        let observed_event = severity.unwrap() > threshold;

        let Some(candidate) = row.get("candidate_output") else {
            errors.push(format!("{case_id}: candidate output is missing"));
            continue;
        };
        let action = string(candidate, "action");
        let probability_value = candidate.get("predicted_exceedance_probability");
        let reason_value = candidate.get("abstention_reason");
        let probability = probability_value.and_then(Value::as_f64);
        let reason = reason_value.and_then(Value::as_str);
        let review_value = evaluator.get("review_fallback_action");
        let review_action = review_value.and_then(Value::as_str);

        let mandatory = matches!(
            qualification.as_str(),
            "missing_critical_input" | "conflicting_sources" | "unsupported_distribution_shift"
        );
        match (mandatory, action) {
            (true, Some("abstain")) => {
                if !probability_value.is_some_and(Value::is_null)
                    || reason != Some(qualification.as_str())
                    || review_action.is_none_or(|value| !matches!(value, "prepare" | "no_action"))
                {
                    errors.push(format!("{case_id}: mandatory abstention requires null probability, matching reason and reviewer fallback"));
                } else {
                    mandatory_abstentions += 1;
                    *mandatory_reasons.entry(qualification.clone()).or_default() += 1;
                }
            }
            (true, Some("forecast")) => {
                errors.push(format!("{case_id}: forecast emitted under mandatory-abstention qualification"));
            }
            (true, _) => errors.push(format!("{case_id}: action must be forecast or abstain")),
            (false, Some("forecast")) => {
                if qualification != "qualified"
                    || probability.is_none_or(|value| !value.is_finite() || !(0.0..=1.0).contains(&value))
                    || !reason_value.is_some_and(Value::is_null)
                    || !review_value.is_some_and(Value::is_null)
                {
                    errors.push(format!("{case_id}: forecast requires qualified input, probability [0,1], and explicit null reason/review fields"));
                }
            }
            (false, Some("abstain")) => {
                if qualification != "qualified"
                    || !probability_value.is_some_and(Value::is_null)
                    || reason != Some("model_uncertainty")
                    || review_action.is_none_or(|value| !matches!(value, "prepare" | "no_action"))
                {
                    errors.push(format!("{case_id}: optional abstention requires model_uncertainty and a reviewer fallback"));
                } else {
                    optional_abstentions += 1;
                }
            }
            (false, _) => errors.push(format!("{case_id}: action must be forecast or abstain")),
        }

        cases.push(EvaluationCase {
            regime_id: regime,
            importance_weight: target / proposal,
            observed_event,
            action: action.unwrap_or("invalid").to_owned(),
            probability,
            review_fallback_action: review_action.map(str::to_owned),
        });
    }

    let expected_weight = number(design, "expected_total_weight_per_regime").unwrap_or(32.0);
    for regime in &regime_order {
        let counts = regime_strata.get(regime);
        let total = counts.map(|values| values.values().sum::<usize>()).unwrap_or(0);
        if total == 0 {
            errors.push(format!("{regime}: empty NRES-011 regime cohort"));
            continue;
        }
        if let Some(counts) = counts {
            for (name, (proposal, _)) in &stratum_map {
                let observed_proposal = counts.get(name).copied().unwrap_or(0) as f64 / total as f64;
                if (observed_proposal - proposal).abs() > 1e-12 {
                    errors.push(format!("{regime}/{name}: sampling allocation differs from proposal mass"));
                }
            }
        }
        let weight_sum: f64 = cases.iter().filter(|case| case.regime_id == *regime)
            .map(|case| case.importance_weight).sum();
        if (weight_sum - expected_weight).abs() > 1e-9 {
            errors.push(format!("{regime}: importance weights do not match expected regime mass"));
        }
    }

    if source_ids.len() != source_cases.len() {
        errors.push("NRES-011 must contain each pinned NRES-010 source case exactly once".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let mut overall_by_profile = Vec::new();
    for profile in &profiles {
        let mut value = summarize_cases(&cases, profile, reference_probability, &edges);
        if let Some(map) = value.as_object_mut() {
            map.insert("profile_id".to_owned(), json!(profile.id));
            map.insert("decision_threshold_probability".to_owned(), json!(profile.threshold));
        }
        overall_by_profile.push(value);
    }

    let per_regime: Vec<Value> = regime_order.iter().map(|regime| {
        let regime_cases: Vec<EvaluationCase> = cases.iter()
            .filter(|case| &case.regime_id == regime).cloned().collect();
        let mut by_profile = Vec::new();
        for profile in &profiles {
            let mut value = summarize_cases(&regime_cases, profile, reference_probability, &edges);
            if let Some(map) = value.as_object_mut() {
                map.insert("profile_id".to_owned(), json!(profile.id));
                map.insert("decision_threshold_probability".to_owned(), json!(profile.threshold));
            }
            by_profile.push(value);
        }
        json!({"regime_id": regime, "profiles": by_profile})
    }).collect();

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-011-RELIABILITY-RESOLUTION-DECISION-VALUE-001",
        "interpretation": "synthetic profile-specific forecast diagnostics and decision-loss accounting; no real-world action or savings claim",
        "total_cases": cases.len(),
        "mandatory_abstentions": mandatory_abstentions,
        "optional_abstentions": optional_abstentions,
        "mandatory_abstention_by_reason": mandatory_reasons,
        "overall_by_profile": overall_by_profile,
        "per_regime": per_regime,
        "aggregation_policy": "profiles and regimes remain separate; no cross-profile composite score"
    }))
}

#[cfg(test)]
mod tests {
    use super::evaluate_reliability_resolution_decision_value;
    use serde_json::{json, Value};

    const FIXTURE: &str = include_str!("../../../examples/courses/civic-resilience-reliability-resolution-decision-value-fixture-001.json");
    const CONFORMANCE: &str = include_str!("../../../examples/courses/civic-resilience-reliability-resolution-decision-value-conformance-001.json");

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("NRES-011 fixture must be valid JSON")
    }

    fn first_forecast(value: &mut Value) -> &mut Value {
        value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["candidate_output"]["action"] == "forecast").unwrap()
    }

    #[test]
    fn derives_profile_specific_decision_costs_and_forecast_decomposition() {
        let report = evaluate_reliability_resolution_decision_value(&fixture()).expect("fixture should validate");
        assert_eq!(report["total_cases"].as_u64(), Some(128));
        assert_eq!(report["mandatory_abstentions"].as_u64(), Some(4));
        assert_eq!(report["optional_abstentions"].as_u64(), Some(1));
        assert_eq!(report["overall_by_profile"].as_array().unwrap().len(), 3);
        assert!(report.get("overall_resilience_score").is_none());
        assert!(report["overall_by_profile"][0]["decision_metrics"]["relative_economic_value"].as_f64().is_some());
    }

    #[test]
    fn matches_published_decision_value_goldens() {
        let report = evaluate_reliability_resolution_decision_value(&fixture()).expect("fixture valid");
        let corpus: Value = serde_json::from_str(CONFORMANCE).expect("conformance parses");
        let expected = &corpus["expected_metrics"];
        let tolerance = corpus["conformance_targets"]["golden_metric_absolute_tolerance"].as_f64().unwrap_or(1e-12);

        let compare_profile = |actual: &Value, expected: &Value| {
            for key in [
                "cases", "issued_forecasts", "reviewed_abstentions",
                "raw_forecast_coverage", "weighted_forecast_coverage", "importance_weight_sum",
                "effective_sample_size", "forecast_effective_sample_size",
                "weighted_event_rate", "mean_candidate_probability", "original_brier_score",
                "log_loss", "climatology_brier_score", "brier_skill_score"
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
                "target_population_event_rate", "event_forecast_coverage", "non_event_forecast_coverage",
                "target_event_weight", "target_non_event_weight", "forecast_event_weight",
                "forecast_non_event_weight", "abstained_event_weight", "abstained_non_event_weight",
                "abstained_weight", "abstained_event_rate"
            ] {
                if expected["selection_diagnostics"][key].is_null() {
                    assert!(actual["selection_diagnostics"][key].is_null(), "{key}");
                } else {
                    assert!((actual["selection_diagnostics"][key].as_f64().unwrap()
                        - expected["selection_diagnostics"][key].as_f64().unwrap()).abs() <= tolerance, "{key}");
                }
            }
            for key in ["reliability", "resolution", "uncertainty", "binned_brier_score",
                "murphy_reconstructed_brier_score", "decomposition_identity_residual",
                "original_brier_score", "binned_minus_original_residual"]
            {
                assert!((actual["reliability_resolution_uncertainty"][key].as_f64().unwrap()
                    - expected["reliability_resolution_uncertainty"][key].as_f64().unwrap()).abs() <= tolerance, "{key}");
            }
            for key in ["candidate_forecast_and_review", "always_no_action", "always_prepare",
                "training_climatology", "perfect_information_oracle_lower_bound"]
            {
                assert!((actual["policy_costs"][key].as_f64().unwrap()
                    - expected["policy_costs"][key].as_f64().unwrap()).abs() <= tolerance, "{key}");
            }
            for key in ["relative_economic_value", "relative_economic_value_denominator",
                "regret_vs_perfect_information", "candidate_savings_vs_no_action",
                "candidate_savings_vs_training_climatology"]
            {
                assert!((actual["decision_metrics"][key].as_f64().unwrap()
                    - expected["decision_metrics"][key].as_f64().unwrap()).abs() <= tolerance, "{key}");
            }
            for key in ["prepared_decisions", "reviewed_cases", "false_negative_cases", "false_prepare_cases"] {
                assert_eq!(actual["decision_metrics"][key], expected["decision_metrics"][key], "{key}");
            }
        };

        for actual in report["overall_by_profile"].as_array().unwrap() {
            let profile_id = actual["profile_id"].as_str().unwrap();
            compare_profile(actual, &expected["overall"][profile_id]);
        }
        assert_eq!(report["per_regime"].as_array().unwrap().len(), expected["per_regime"].as_object().unwrap().len());
        for actual_regime in report["per_regime"].as_array().unwrap() {
            let regime_id = actual_regime["regime_id"].as_str().unwrap();
            let expected_regime = &expected["per_regime"][regime_id];
            for actual_profile in actual_regime["profiles"].as_array().unwrap() {
                let profile_id = actual_profile["profile_id"].as_str().unwrap();
                compare_profile(actual_profile, &expected_regime[profile_id]);
            }
        }
    }

    #[test]
    fn rejects_source_fixture_blob_sha_drift() {
        let mut value = fixture();
        value["source_fixture"]["fixture_blob_sha"] = json!("0000000000000000000000000000000000000000");
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_source_case_not_in_pinned_nres010() {
        let mut value = fixture();
        value["evaluation_cases"][0]["source_case_id"] = json!("NOT-A-SOURCE-CASE");
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_outcome_drift_from_pinned_nres010_source() {
        let mut value = fixture();
        value["evaluation_cases"][0]["evaluator"]["observed_normalized_service_deficit_hours"] = json!(47.0);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_missing_regime_catalog() {
        let mut value = fixture();
        value.as_object_mut().unwrap().remove("regimes");
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }


    #[test]
    fn reports_event_conditional_coverage_and_abstention_selection() {
        let baseline = evaluate_reliability_resolution_decision_value(&fixture()).unwrap();
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["evaluator"]["qualification_state"] == "qualified"
                && case["evaluator"]["observed_normalized_service_deficit_hours"].as_f64().unwrap() > 48.0
                && case["candidate_output"]["action"] == "forecast").unwrap();
        row["candidate_output"]["action"] = json!("abstain");
        row["candidate_output"]["predicted_exceedance_probability"] = Value::Null;
        row["candidate_output"]["abstention_reason"] = json!("model_uncertainty");
        row["evaluator"]["review_fallback_action"] = json!("no_action");
        let selected = evaluate_reliability_resolution_decision_value(&value).unwrap();
        let baseline_profile = &baseline["overall_by_profile"][0]["selection_diagnostics"];
        let selected_profile = &selected["overall_by_profile"][0]["selection_diagnostics"];
        assert!(selected_profile["event_forecast_coverage"].as_f64().unwrap()
            < baseline_profile["event_forecast_coverage"].as_f64().unwrap());
        assert_eq!(selected_profile["target_population_event_rate"], baseline_profile["target_population_event_rate"]);
        assert!(selected_profile["abstained_event_rate"].as_f64().unwrap() > 0.0);
    }

    #[test]
    fn rejects_probability_above_one() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["predicted_exceedance_probability"] = json!(1.01);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_negative_probability() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["predicted_exceedance_probability"] = json!(-0.01);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_forecast_missing_probability() {
        let mut value = fixture();
        first_forecast(&mut value)["candidate_output"]["predicted_exceedance_probability"] = Value::Null;
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_probability_on_abstention() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["candidate_output"]["action"] == "abstain").unwrap();
        row["candidate_output"]["predicted_exceedance_probability"] = json!(0.2);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_forecast_on_missing_critical_input() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["evaluator"]["qualification_state"] == "missing_critical_input").unwrap();
        row["candidate_output"]["action"] = json!("forecast");
        row["candidate_output"]["predicted_exceedance_probability"] = json!(0.3);
        row["candidate_output"]["abstention_reason"] = Value::Null;
        row["evaluator"]["review_fallback_action"] = Value::Null;
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_forecast_on_conflicting_sources() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["evaluator"]["qualification_state"] == "conflicting_sources").unwrap();
        row["candidate_output"]["action"] = json!("forecast");
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_missing_review_fallback() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["candidate_output"]["action"] == "abstain").unwrap();
        row["evaluator"]["review_fallback_action"] = Value::Null;
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_review_fallback_on_forecast() {
        let mut value = fixture();
        let row = first_forecast(&mut value);
        row["evaluator"]["review_fallback_action"] = json!("prepare");
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_negative_readiness_cost() {
        let mut value = fixture();
        value["decision_profiles"][0]["readiness_cost"] = json!(-1.0);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_action_that_does_not_reduce_event_loss() {
        let mut value = fixture();
        value["decision_profiles"][0]["event_loss_with_action"] = value["decision_profiles"][0]["event_loss_without_action"].clone();
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_negative_deferral_cost() {
        let mut value = fixture();
        value["decision_profiles"][0]["deferral_cost"] = json!(-1.0);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_threshold_drift_from_cost_loss_ratio() {
        let mut value = fixture();
        value["decision_profiles"][0]["decision_threshold_probability"] = json!(0.5);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_case_sampling_design_drift() {
        let mut value = fixture();
        value["evaluation_cases"][0]["target_stratum_probability"] = json!(0.5);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_case_id() {
        let mut value = fixture();
        let duplicate = value["evaluation_cases"][0]["case_id"].clone();
        value["evaluation_cases"][1]["case_id"] = duplicate;
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_source_case_id() {
        let mut value = fixture();
        let duplicate = value["evaluation_cases"][0]["source_case_id"].clone();
        value["evaluation_cases"][1]["source_case_id"] = duplicate;
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_composite_score_injection() {
        let mut value = fixture();
        value["overall_resilience_score"] = json!(0.8);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_empty_regime_cohort() {
        let mut value = fixture();
        value["evaluation_cases"].as_array_mut().unwrap().retain(|case| case["regime_id"] != "temporal_shift");
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn rejects_horizon_drift() {
        let mut value = fixture();
        value["evaluation_cases"][0]["forecast_horizon_hours"] = json!(24);
        assert!(evaluate_reliability_resolution_decision_value(&value).is_err());
    }

    #[test]
    fn inclusive_threshold_prepares_at_exact_equality() {
        let mut value = fixture();
        let profile = value["decision_profiles"][0].clone();
        let threshold = profile["decision_threshold_probability"].as_f64().unwrap();
        let row = first_forecast(&mut value);
        row["candidate_output"]["predicted_exceedance_probability"] = json!(threshold);
        let report = evaluate_reliability_resolution_decision_value(&value).expect("boundary case valid");
        let profile_result = report["overall_by_profile"].as_array().unwrap().iter()
            .find(|item| item["profile_id"] == profile["profile_id"]).unwrap();
        assert!(profile_result["decision_metrics"]["prepared_decisions"].as_u64().unwrap() > 0);
    }

    #[test]
    fn strict_event_threshold_equality_is_not_a_tail_event() {
        let mut value = fixture();
        let row = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["evaluator"]["observed_normalized_service_deficit_hours"].as_f64() == Some(48.0))
            .unwrap();
        row["evaluator"]["observed_normalized_service_deficit_hours"] = json!(48.0);
        let report = evaluate_reliability_resolution_decision_value(&value).expect("threshold equality valid");
        assert_eq!(report["overall_by_profile"][0]["weighted_event_rate"].as_f64().is_some(), true);
    }
}

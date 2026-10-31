//! Exact finite-support Wasserstein DRO over complete joint binary service states.
//!
//! This evaluator consumes the pinned NRES-009 fixture and a synthetic decision
//! fixture. It does not generate incidents, infer causal dependencies, score
//! people, or prescribe real-world preparedness actions.

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

const NRES009_SOURCE_GIT_BLOB_SHA1: &str = "799799f8a315d4c0f06a43b5dce5ab2e15ccefb5";
const NRES009_SOURCE_PATH: &str =
    "mycelix-workspace/mycelix-praxis/examples/courses/civic-resilience-joint-service-forecast-fixture-001.json";
const GOLDEN_TOLERANCE: f64 = 1e-10;

fn str_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn num_field(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64)
}

fn array_field<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn finite_nonnegative(value: Option<f64>) -> bool {
    value.is_some_and(|number| number.is_finite() && number >= 0.0)
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

fn weighted_hamming(left: usize, right: usize, weights: &[f64]) -> f64 {
    weights
        .iter()
        .enumerate()
        .filter(|(bit, _)| ((left >> *bit) & 1) != ((right >> *bit) & 1))
        .map(|(_, weight)| *weight)
        .sum()
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter().zip(right.iter()).map(|(a, b)| a * b).sum()
}

#[derive(Clone, Copy)]
struct Line {
    slope: f64,
    intercept: f64,
}

/// Collect positive breakpoints of the upper envelope of affine lines.
fn upper_envelope_breakpoints(lines: &mut Vec<Line>) -> Vec<f64> {
    lines.sort_by(|left, right| left.slope.total_cmp(&right.slope));

    let mut unique: Vec<Line> = Vec::with_capacity(lines.len());
    for line in lines.iter().copied() {
        if let Some(previous) = unique.last_mut() {
            if (previous.slope - line.slope).abs() <= 1e-12 {
                if line.intercept > previous.intercept {
                    *previous = line;
                }
                continue;
            }
        }
        unique.push(line);
    }

    let mut hull: Vec<Line> = Vec::with_capacity(unique.len());
    let mut starts: Vec<f64> = Vec::with_capacity(unique.len());
    for line in unique {
        let mut start = f64::NEG_INFINITY;
        while let Some(previous) = hull.last().copied() {
            start = (previous.intercept - line.intercept) / (line.slope - previous.slope);
            let last_start = *starts.last().expect("non-empty hull has a start");
            if start <= last_start + 1e-12 {
                hull.pop();
                starts.pop();
            } else {
                break;
            }
        }
        if hull.is_empty() {
            start = f64::NEG_INFINITY;
        }
        hull.push(line);
        starts.push(start);
    }

    starts
        .into_iter()
        .filter(|start| start.is_finite() && *start > 0.0)
        .collect()
}

/// Exact worst-case expectation over a finite-support W1 ball.
///
/// For finite support, the Kantorovich dual is convex and piecewise linear in
/// lambda. Its minimum is attained at zero or at an upper-envelope breakpoint.
/// At epsilon=0 the dual infimum is the lambda->infinity limit; return E_p loss
/// directly. At epsilon>=diameter every distribution on the support is reachable.
fn wasserstein_upper_expectation(
    probabilities: &[f64],
    state_losses: &[f64],
    distances: &[Vec<f64>],
    radius: f64,
    diameter: f64,
) -> Result<(f64, Option<f64>), String> {
    let state_count = probabilities.len();
    if state_count == 0
        || state_losses.len() != state_count
        || distances.len() != state_count
        || distances.iter().any(|row| row.len() != state_count)
    {
        return Err("Wasserstein dual inputs have inconsistent dimensions".to_owned());
    }
    if !radius.is_finite() || radius < 0.0 {
        return Err("Wasserstein radius must be finite and non-negative".to_owned());
    }
    if radius == 0.0 {
        return Ok((dot(probabilities, state_losses), None));
    }
    if radius >= diameter {
        let maximum = state_losses
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        return Ok((maximum, Some(0.0)));
    }

    let mut candidates = vec![0.0];
    for source_state in 0..state_count {
        let mut lines = Vec::with_capacity(state_count);
        for destination_state in 0..state_count {
            lines.push(Line {
                slope: -distances[source_state][destination_state],
                intercept: state_losses[destination_state],
            });
        }
        candidates.extend(upper_envelope_breakpoints(&mut lines));
    }
    candidates.sort_by(f64::total_cmp);
    candidates.dedup_by(|left, right| (*left - *right).abs() <= 1e-10);

    let mut best_value = f64::INFINITY;
    let mut best_lambda = 0.0;
    for lambda in candidates {
        let mut objective = lambda * radius;
        for source_state in 0..state_count {
            let worst_destination = (0..state_count)
                .map(|destination_state| {
                    state_losses[destination_state]
                        - lambda * distances[source_state][destination_state]
                })
                .fold(f64::NEG_INFINITY, f64::max);
            objective += probabilities[source_state] * worst_destination;
        }
        if objective < best_value - 1e-10 {
            best_value = objective;
            best_lambda = lambda;
        }
    }
    if !best_value.is_finite() || !best_lambda.is_finite() {
        return Err("Wasserstein dual produced a non-finite optimum".to_owned());
    }
    Ok((best_value.max(0.0), Some(best_lambda)))
}

fn argmin(values: &[f64]) -> usize {
    let mut best = 0;
    for index in 1..values.len() {
        if values[index] < values[best] - 1e-10 {
            best = index;
        }
    }
    best
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
}

fn nearest_rank_p90(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let rank = ((sorted.len() as f64 * 0.90).ceil() as usize)
        .max(1)
        .min(sorted.len());
    sorted[rank - 1]
}

#[derive(Default)]
struct StrategyAccumulator {
    realized_costs: Vec<f64>,
    realized_regrets: Vec<f64>,
    nominal_expected_losses: Vec<f64>,
    worst_case_expected_losses: Vec<f64>,
    worst_case_regrets: Vec<f64>,
    action_counts: BTreeMap<String, usize>,
}

impl StrategyAccumulator {
    fn record(
        &mut self,
        action_id: &str,
        realized_cost: f64,
        realized_regret: f64,
        nominal_expected_loss: f64,
        worst_case_expected_loss: f64,
        worst_case_regret: f64,
    ) {
        self.realized_costs.push(realized_cost);
        self.realized_regrets.push(realized_regret);
        self.nominal_expected_losses.push(nominal_expected_loss);
        self.worst_case_expected_losses.push(worst_case_expected_loss);
        self.worst_case_regrets.push(worst_case_regret);
        *self.action_counts.entry(action_id.to_owned()).or_insert(0) += 1;
    }

    fn report(&self) -> Value {
        json!({
            "mean_realized_cost": mean(&self.realized_costs),
            "p90_realized_cost_nearest_rank": nearest_rank_p90(&self.realized_costs),
            "max_realized_cost": self.realized_costs.iter().copied().fold(0.0, f64::max),
            "mean_realized_statewise_regret": mean(&self.realized_regrets),
            "mean_selected_nominal_expected_loss": mean(&self.nominal_expected_losses),
            "mean_selected_worst_case_expected_loss": mean(&self.worst_case_expected_losses),
            "mean_selected_worst_case_regret": mean(&self.worst_case_regrets),
            "action_counts": self.action_counts
        })
    }
}

#[derive(Default)]
struct GroupAccumulator {
    source_case_ids: BTreeSet<String>,
    case_count: usize,
    perfect_information_costs: Vec<f64>,
    strategies: BTreeMap<String, StrategyAccumulator>,
}

impl GroupAccumulator {
    fn report(&self, profile_id: &str, radius_level_id: &str) -> Value {
        let strategies: serde_json::Map<String, Value> = self
            .strategies
            .iter()
            .map(|(name, accumulator)| (name.clone(), accumulator.report()))
            .collect();
        json!({
            "profile_id": profile_id,
            "radius_level_id": radius_level_id,
            "case_count": self.case_count,
            "unique_source_case_count": self.source_case_ids.len(),
            "mean_perfect_information_cost": mean(&self.perfect_information_costs),
            "strategies": strategies
        })
    }
}

#[derive(Clone)]
struct Action {
    id: String,
    resource_units: f64,
    prepared_mask: usize,
}

#[derive(Clone)]
struct Profile {
    id: String,
    max_resource_units: f64,
    resource_unit_cost: f64,
    service_breach_costs: Vec<f64>,
    prepared_residual_fraction: f64,
}

fn action_state_losses(
    profile: &Profile,
    action: &Action,
    dimension: usize,
    state_count: usize,
) -> Vec<f64> {
    (0..state_count)
        .map(|state| {
            let mut loss = action.resource_units * profile.resource_unit_cost;
            for bit in 0..dimension {
                if ((state >> bit) & 1) == 1 {
                    let residual = if ((action.prepared_mask >> bit) & 1) == 1 {
                        profile.prepared_residual_fraction
                    } else {
                        1.0
                    };
                    loss += profile.service_breach_costs[bit] * residual;
                }
            }
            loss
        })
        .collect()
}

fn parse_joint_probabilities(
    case: &Value,
    case_id: &str,
    state_count: usize,
    tolerance: f64,
    errors: &mut Vec<String>,
) -> Option<Vec<f64>> {
    let Some(distribution) = case
        .pointer("/candidate_output/forecast_distribution")
        .and_then(Value::as_array)
    else {
        errors.push(format!("{case_id}: candidate joint distribution is missing"));
        return None;
    };
    if distribution.len() != state_count {
        errors.push(format!("{case_id}: forecast must enumerate all joint states exactly once"));
        return None;
    }

    let mut probabilities: Vec<Option<f64>> = vec![None; state_count];
    for entry in distribution {
        let mask = entry.get("state_mask").and_then(Value::as_u64);
        let probability = entry.get("probability").and_then(Value::as_f64);
        let Some(mask) = mask.filter(|mask| *mask < state_count as u64) else {
            errors.push(format!("{case_id}: state mask is outside the support"));
            continue;
        };
        let Some(probability) = probability.filter(|value| {
            value.is_finite() && (0.0..=1.0).contains(value)
        }) else {
            errors.push(format!("{case_id}: state probability must be finite and within [0,1]"));
            continue;
        };
        if probabilities[mask as usize].replace(probability).is_some() {
            errors.push(format!("{case_id}: duplicate state mask {mask}"));
        }
    }
    if probabilities.iter().any(Option::is_none) {
        errors.push(format!("{case_id}: forecast is missing one or more joint states"));
        return None;
    }
    let probabilities: Vec<f64> = probabilities.into_iter().flatten().collect();
    let mass = probabilities.iter().sum::<f64>();
    if (mass - 1.0).abs() > tolerance {
        errors.push(format!("{case_id}: joint probability mass is {mass}, outside tolerance"));
        return None;
    }
    Some(probabilities)
}

fn compare_golden(groups: &[Value], golden: &Value, errors: &mut Vec<String>) {
    let profile_id = str_field(golden, "profile_id").unwrap_or("UNKNOWN");
    let radius_id = str_field(golden, "radius_level_id").unwrap_or("UNKNOWN");
    let Some(group) = groups.iter().find(|group| {
        str_field(group, "profile_id") == Some(profile_id)
            && str_field(group, "radius_level_id") == Some(radius_id)
    }) else {
        errors.push(format!("golden row {profile_id}/{radius_id} has no computed report"));
        return;
    };

    let checks = [
        ("nominal_point_forecast", "mean_realized_cost", "nominal_point_mean_realized_cost"),
        ("wasserstein_worst_case_expected_loss", "mean_realized_cost", "wasserstein_expected_mean_realized_cost"),
        ("wasserstein_minimax_regret", "mean_realized_cost", "wasserstein_minimax_regret_mean_realized_cost"),
        ("wasserstein_worst_case_expected_loss", "mean_selected_worst_case_expected_loss", "wasserstein_expected_mean_upper_bound"),
        ("wasserstein_minimax_regret", "mean_selected_worst_case_regret", "wasserstein_minimax_regret_mean_upper_bound"),
    ];
    for (strategy, actual_field, expected_field) in checks {
        let actual = group
            .pointer(&format!("/strategies/{strategy}/{actual_field}"))
            .and_then(Value::as_f64);
        let expected = num_field(golden, expected_field);
        if actual.zip(expected).is_none_or(|(a, b)| {
            !a.is_finite() || !b.is_finite() || (a - b).abs() > GOLDEN_TOLERANCE
        }) {
            errors.push(format!("golden mismatch at {profile_id}/{radius_id}/{expected_field}"));
        }
    }
    let actual = num_field(group, "mean_perfect_information_cost");
    let expected = num_field(golden, "mean_perfect_information_cost");
    if actual.zip(expected).is_none_or(|(a, b)| {
        !a.is_finite() || !b.is_finite() || (a - b).abs() > GOLDEN_TOLERANCE
    }) {
        errors.push(format!("golden mismatch at {profile_id}/{radius_id}/mean_perfect_information_cost"));
    }
}

/// Evaluate nominal, W1 worst-case expected-loss and W1 minimax-regret policies.
///
/// The source fixture is passed separately. Every copied case, outcome and
/// candidate distribution is compared with that source before decision scoring.
pub fn evaluate_wasserstein_joint_decisions(
    source_joint: &Value,
    fixture: &Value,
) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    forbidden_key_scan(fixture, "", &mut errors);

    if str_field(fixture, "benchmark_id")
        != Some("PRAXIS-NRES-013-WASSERSTEIN-JOINT-DECISION-001")
    {
        errors.push("unexpected NRES-013 benchmark_id".to_owned());
    }
    if str_field(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-013 schema_version".to_owned());
    }
    if str_field(fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("NRES-013 fixture must remain explicitly synthetic".to_owned());
    }
    if str_field(fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("NRES-013 fixture must remain public development data".to_owned());
    }
    if fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("NRES-013 fixture must remain inactive".to_owned());
    }

    if str_field(source_joint, "benchmark_id")
        != Some("PRAXIS-NRES-009-JOINT-SERVICE-FORECAST-001")
        || str_field(source_joint, "status") != Some("synthetic-demonstration-for-review")
        || str_field(source_joint, "evaluation_split") != Some("public-development-only")
        || source_joint.get("active_in_curriculum").and_then(Value::as_bool) != Some(false)
    {
        errors.push("NRES-009 source identity or synthetic boundary is invalid".to_owned());
    }
    let Some(source_info) = fixture.get("source") else {
        return Err(vec!["NRES-013 source metadata is missing".to_owned()]);
    };
    if str_field(source_info, "fixture_git_blob_sha1") != Some(NRES009_SOURCE_GIT_BLOB_SHA1)
        || str_field(source_info, "fixture_path") != Some(NRES009_SOURCE_PATH)
        || str_field(source_info, "benchmark_id")
            != Some("PRAXIS-NRES-009-JOINT-SERVICE-FORECAST-001")
    {
        errors.push("NRES-013 source path, ID or pinned Git blob SHA-1 differs".to_owned());
    }

    let Some(source_services) = array_field(source_joint, "service_catalog") else {
        return Err(vec!["NRES-009 service catalog is missing".to_owned()]);
    };
    let Some(services) = array_field(fixture, "service_catalog") else {
        return Err(vec!["NRES-013 service catalog is missing".to_owned()]);
    };
    if services != source_services {
        errors.push("NRES-013 service catalog differs from pinned NRES-009 source".to_owned());
    }
    let dimension = services.len();
    if !(2..=6).contains(&dimension) {
        errors.push("exact evaluator supports 2 through 6 services".to_owned());
        return Err(errors);
    }
    let state_count = 1usize << dimension;
    let mut service_ids = BTreeSet::new();
    let mut bit_indices = BTreeSet::new();
    for service in services {
        let id = str_field(service, "service_id");
        let bit = service.get("bit_index").and_then(Value::as_u64);
        if id.is_none_or(str::is_empty)
            || bit.is_none_or(|value| value >= dimension as u64)
            || str_field(service, "unit") != Some("binary-breach-indicator")
        {
            errors.push("service requires a unique ID, an in-range bit index and binary-breach-indicator unit".to_owned());
            continue;
        }
        if !service_ids.insert(id.unwrap().to_owned()) || !bit_indices.insert(bit.unwrap()) {
            errors.push("service IDs and bit indexes must be unique".to_owned());
        }
    }
    if bit_indices.len() != dimension {
        errors.push("service bit indexes must cover the complete dimension".to_owned());
    }

    let Some(protocol) = fixture.get("protocol") else {
        return Err(vec!["NRES-013 protocol is missing".to_owned()]);
    };
    let Some(source_protocol) = source_joint.get("protocol") else {
        return Err(vec!["NRES-009 source protocol is missing".to_owned()]);
    };
    let tolerance = num_field(protocol, "probability_sum_tolerance");
    if num_field(protocol, "forecast_origin_hour")
        != num_field(source_protocol, "forecast_origin_hour")
        || num_field(protocol, "forecast_horizon_hours")
            != num_field(source_protocol, "forecast_horizon_hours")
        || protocol.get("state_count").and_then(Value::as_u64) != Some(state_count as u64)
        || source_protocol.get("state_count").and_then(Value::as_u64) != Some(state_count as u64)
        || tolerance.is_none_or(|value| !value.is_finite() || value <= 0.0 || value > 1e-6)
    {
        errors.push("protocol horizon, state count or probability tolerance is invalid".to_owned());
    }
    let tolerance = tolerance.unwrap_or(1e-12);

    let Some(weight_values) = protocol
        .pointer("/ground_metric/service_change_weights")
        .and_then(Value::as_array)
    else {
        return Err(vec!["weighted-Hamming service weights are missing".to_owned()]);
    };
    let Some(ground_metric) = protocol.get("ground_metric") else {
        return Err(vec!["NRES-013 ground metric is missing".to_owned()]);
    };
    if str_field(ground_metric, "name") != Some("weighted_hamming")
        || weight_values.len() != dimension
        || weight_values.iter().any(|weight| {
            weight.as_f64().is_none_or(|number| !number.is_finite() || number <= 0.0)
        })
    {
        errors.push("weighted-Hamming metric requires one finite positive weight per service".to_owned());
        return Err(errors);
    }
    let weights: Vec<f64> = weight_values.iter().filter_map(Value::as_f64).collect();
    let diameter = weights.iter().sum::<f64>();
    if !diameter.is_finite() || diameter <= 0.0 {
        return Err(vec!["weighted-Hamming support diameter must be finite and positive".to_owned()]);
    }
    let distances: Vec<Vec<f64>> = (0..state_count)
        .map(|left| {
            (0..state_count)
                .map(|right| weighted_hamming(left, right, &weights))
                .collect()
        })
        .collect();

    let Some(radius_values) = array_field(fixture, "radius_levels") else {
        return Err(vec!["NRES-013 radius_levels are missing".to_owned()]);
    };
    let mut radius_by_id = BTreeMap::new();
    for radius in radius_values {
        let Some(id) = str_field(radius, "radius_level_id") else {
            errors.push("radius tier is missing radius_level_id".to_owned());
            continue;
        };
        let value = num_field(radius, "wasserstein_radius");
        if id.is_empty() || !finite_nonnegative(value) {
            errors.push(format!("{id}: radius must be finite and non-negative"));
            continue;
        }
        if radius_by_id.insert(id.to_owned(), value.unwrap()).is_some() {
            errors.push(format!("duplicate radius tier: {id}"));
        }
    }
    if radius_by_id.len() < 2 {
        errors.push("at least two unique radius tiers are required".to_owned());
    }

    let Some(source_cases) = array_field(source_joint, "evaluation_cases") else {
        return Err(vec!["NRES-009 source cases are missing".to_owned()]);
    };
    let mut source_by_id: BTreeMap<String, &Value> = BTreeMap::new();
    for source_case in source_cases {
        let Some(id) = str_field(source_case, "case_id") else {
            errors.push("NRES-009 source case is missing case_id".to_owned());
            continue;
        };
        if source_by_id.insert(id.to_owned(), source_case).is_some() {
            errors.push(format!("duplicate NRES-009 source case ID: {id}"));
        }
    }
    if num_field(source_info, "source_case_count") != Some(source_by_id.len() as f64) {
        errors.push("NRES-013 source_case_count differs from source case count".to_owned());
    }

    let Some(case_values) = array_field(fixture, "evaluation_cases") else {
        return Err(vec!["NRES-013 evaluation_cases are missing".to_owned()]);
    };
    let mut case_ids = BTreeSet::new();
    let mut source_radius_pairs = BTreeSet::new();
    let mut source_counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut parsed_cases = Vec::new();

    for case in case_values {
        let case_id = str_field(case, "case_id").unwrap_or("UNKNOWN").to_owned();
        if case_id == "UNKNOWN" || !case_ids.insert(case_id.clone()) {
            errors.push(format!("{case_id}: case IDs must be present and unique"));
        }
        let Some(source_id) = str_field(case, "source_case_id") else {
            errors.push(format!("{case_id}: source_case_id is missing"));
            continue;
        };
        let Some(source_case) = source_by_id.get(source_id) else {
            errors.push(format!("{case_id}: unknown source case {source_id}"));
            continue;
        };
        let Some(radius_id) = str_field(case, "radius_level_id") else {
            errors.push(format!("{case_id}: radius_level_id is missing"));
            continue;
        };
        let Some(radius) = radius_by_id.get(radius_id).copied() else {
            errors.push(format!("{case_id}: undeclared radius tier {radius_id}"));
            continue;
        };
        if num_field(case, "wasserstein_radius")
            .is_none_or(|value| !value.is_finite() || (value - radius).abs() > 1e-12)
        {
            errors.push(format!("{case_id}: case radius differs from its declared tier"));
        }
        if !source_radius_pairs.insert((source_id.to_owned(), radius_id.to_owned())) {
            errors.push(format!("{case_id}: duplicate source-case/radius pair"));
        }
        *source_counts.entry(source_id.to_owned()).or_insert(0) += 1;

        let observed = case.pointer("/evaluator/observed_breach_mask").and_then(Value::as_u64);
        let source_observed = source_case
            .pointer("/evaluator/observed_breach_mask")
            .and_then(Value::as_u64);
        if observed != source_observed || observed.is_none_or(|mask| mask >= state_count as u64) {
            errors.push(format!("{case_id}: observed joint state differs from NRES-009"));
            continue;
        }
        let candidate_distribution = case
            .pointer("/candidate_output/forecast_distribution")
            .cloned()
            .unwrap_or(Value::Null);
        let source_distribution = source_case
            .pointer("/candidate_output/forecast_distribution")
            .cloned()
            .unwrap_or(Value::Null);
        if candidate_distribution != source_distribution {
            errors.push(format!("{case_id}: copied candidate distribution differs from NRES-009"));
            continue;
        }
        if let Some(probabilities) =
            parse_joint_probabilities(case, &case_id, state_count, tolerance, &mut errors)
        {
            parsed_cases.push((
                case_id,
                source_id.to_owned(),
                radius_id.to_owned(),
                radius,
                observed.unwrap() as usize,
                probabilities,
            ));
        }
    }

    if case_values.len() != source_by_id.len() * radius_by_id.len() {
        errors.push("row count must equal source cases multiplied by radius tiers".to_owned());
    }
    for source_id in source_by_id.keys() {
        for radius_id in radius_by_id.keys() {
            if !source_radius_pairs.contains(&(source_id.to_owned(), radius_id.to_owned())) {
                errors.push(format!("missing source/radius pairing: {source_id}/{radius_id}"));
            }
        }
    }
    if source_counts.len() != source_by_id.len()
        || source_counts.values().any(|count| *count != radius_by_id.len())
    {
        errors.push("each source case must appear exactly once at every radius tier".to_owned());
    }

    let Some(action_values) = array_field(fixture, "action_catalog") else {
        return Err(vec!["NRES-013 action_catalog is missing".to_owned()]);
    };
    let mut action_ids = BTreeSet::new();
    let mut actions = Vec::new();
    for value in action_values {
        let id = str_field(value, "action_id");
        let units = num_field(value, "resource_units");
        let mask = value.get("prepared_service_mask").and_then(Value::as_u64);
        if id.is_none_or(str::is_empty)
            || !finite_nonnegative(units)
            || mask.is_none_or(|value| value >= state_count as u64)
        {
            errors.push("action requires unique ID, non-negative finite resources and in-range service mask".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if !action_ids.insert(id.clone()) {
            errors.push(format!("duplicate action ID: {id}"));
        }
        actions.push(Action {
            id,
            resource_units: units.unwrap(),
            prepared_mask: mask.unwrap() as usize,
        });
    }
    if !actions.iter().any(|action| {
        action.id == "no_action" && action.resource_units == 0.0 && action.prepared_mask == 0
    }) {
        errors.push("required no_action baseline must use zero resources and an empty mask".to_owned());
    }

    let Some(profile_values) = array_field(fixture, "decision_profiles") else {
        return Err(vec!["NRES-013 decision_profiles are missing".to_owned()]);
    };
    let mut profile_ids = BTreeSet::new();
    let mut profiles = Vec::new();
    for value in profile_values {
        let id = str_field(value, "profile_id");
        let max_units = num_field(value, "max_resource_units");
        let unit_cost = num_field(value, "resource_unit_cost");
        let residual = num_field(value, "prepared_residual_fraction");
        let Some(service_cost_values) = array_field(value, "service_breach_costs") else {
            errors.push("profile service_breach_costs are missing".to_owned());
            continue;
        };
        if id.is_none_or(str::is_empty)
            || !finite_nonnegative(max_units)
            || !finite_nonnegative(unit_cost)
            || residual.is_none_or(|number| !number.is_finite() || !(0.0..=1.0).contains(&number))
            || service_cost_values.len() != dimension
            || service_cost_values.iter().any(|cost| {
                cost.as_f64().is_none_or(|number| !number.is_finite() || number < 0.0)
            })
        {
            errors.push("decision profile contains invalid resource or service-loss parameters".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if !profile_ids.insert(id.clone()) {
            errors.push(format!("duplicate decision profile ID: {id}"));
        }
        let service_breach_costs: Vec<f64> =
            service_cost_values.iter().filter_map(Value::as_f64).collect();
        let profile = Profile {
            id,
            max_resource_units: max_units.unwrap(),
            resource_unit_cost: unit_cost.unwrap(),
            service_breach_costs,
            prepared_residual_fraction: residual.unwrap(),
        };
        let feasible = actions
            .iter()
            .filter(|action| action.resource_units <= profile.max_resource_units + 1e-12)
            .count();
        if feasible == 0 {
            errors.push(format!("{}: no feasible action under resource cap", profile.id));
        }
        profiles.push(profile);
    }
    if profiles.is_empty() {
        errors.push("at least one valid decision profile is required".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let strategy_names = [
        "nominal_point_forecast",
        "wasserstein_worst_case_expected_loss",
        "wasserstein_minimax_regret",
    ];
    let mut group_accumulators: BTreeMap<(String, String), GroupAccumulator> = BTreeMap::new();
    let mut per_case = Vec::new();

    for (case_id, source_id, radius_id, radius, observed, probabilities) in parsed_cases {
        let mut profile_outputs = serde_json::Map::new();
        for profile in &profiles {
            let feasible_actions: Vec<Action> = actions
                .iter()
                .filter(|action| action.resource_units <= profile.max_resource_units + 1e-12)
                .cloned()
                .collect();
            if feasible_actions.is_empty() {
                return Err(vec![format!("{}: no feasible action remains", profile.id)]);
            }
            let action_losses: Vec<Vec<f64>> = feasible_actions
                .iter()
                .map(|action| action_state_losses(profile, action, dimension, state_count))
                .collect();
            if action_losses.iter().flatten().any(|loss| !loss.is_finite()) {
                return Err(vec![format!("{}: action loss overflowed", profile.id)]);
            }

            let state_oracle: Vec<f64> = (0..state_count)
                .map(|state| action_losses.iter().map(|loss| loss[state]).fold(f64::INFINITY, f64::min))
                .collect();
            let regret_losses: Vec<Vec<f64>> = action_losses
                .iter()
                .map(|losses| {
                    losses.iter().enumerate()
                        .map(|(state, loss)| (loss - state_oracle[state]).max(0.0))
                        .collect()
                })
                .collect();
            let nominal_values: Vec<f64> = action_losses
                .iter()
                .map(|losses| dot(&probabilities, losses))
                .collect();
            let robust_expected: Vec<(f64, Option<f64>)> = action_losses
                .iter()
                .map(|losses| wasserstein_upper_expectation(&probabilities, losses, &distances, radius, diameter))
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| vec![format!("{case_id}/{}: {error}", profile.id)])?;
            let robust_regret: Vec<(f64, Option<f64>)> = regret_losses
                .iter()
                .map(|losses| wasserstein_upper_expectation(&probabilities, losses, &distances, radius, diameter))
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| vec![format!("{case_id}/{}: {error}", profile.id)])?;

            let selected = [
                argmin(&nominal_values),
                argmin(&robust_expected.iter().map(|item| item.0).collect::<Vec<_>>()),
                argmin(&robust_regret.iter().map(|item| item.0).collect::<Vec<_>>()),
            ];
            let perfect_information_cost = state_oracle[observed];
            let group = group_accumulators
                .entry((profile.id.clone(), radius_id.clone()))
                .or_default();
            group.case_count += 1;
            group.source_case_ids.insert(source_id.clone());
            group.perfect_information_costs.push(perfect_information_cost);

            let mut strategy_outputs = serde_json::Map::new();
            for (strategy_index, strategy_name) in strategy_names.iter().enumerate() {
                let action_index = selected[strategy_index];
                let action = &feasible_actions[action_index];
                let realized_cost = action_losses[action_index][observed];
                let realized_regret = (realized_cost - perfect_information_cost).max(0.0);
                let worst_case_expected_loss = robust_expected[action_index].0;
                let worst_case_regret = robust_regret[action_index].0;
                let nominal_expected_loss = nominal_values[action_index];

                group.strategies.entry((*strategy_name).to_owned()).or_default().record(
                    &action.id,
                    realized_cost,
                    realized_regret,
                    nominal_expected_loss,
                    worst_case_expected_loss,
                    worst_case_regret,
                );
                strategy_outputs.insert(
                    (*strategy_name).to_owned(),
                    json!({
                        "selected_action_id": action.id,
                        "nominal_expected_loss": nominal_expected_loss,
                        "worst_case_expected_loss": worst_case_expected_loss,
                        "worst_case_regret": worst_case_regret,
                        "expected_loss_dual_lambda": robust_expected[action_index].1,
                        "regret_dual_lambda": robust_regret[action_index].1,
                        "realized_cost": realized_cost,
                        "realized_statewise_regret": realized_regret
                    }),
                );
            }
            profile_outputs.insert(
                profile.id.clone(),
                json!({
                    "feasible_action_ids": feasible_actions.iter().map(|action| action.id.clone()).collect::<Vec<_>>(),
                    "perfect_information_observed_state_cost": perfect_information_cost,
                    "strategies": strategy_outputs
                }),
            );
        }

        per_case.push(json!({
            "case_id": case_id,
            "source_case_id": source_id,
            "radius_level_id": radius_id,
            "wasserstein_radius": radius,
            "observed_breach_mask": observed,
            "decision_profiles": profile_outputs
        }));
    }

    let groups: Vec<Value> = group_accumulators.iter()
        .map(|((profile_id, radius_id), accumulator)| accumulator.report(profile_id, radius_id))
        .collect();
    let Some(golden_values) = array_field(fixture, "golden_expectations") else {
        return Err(vec!["NRES-013 golden_expectations are missing".to_owned()]);
    };
    if golden_values.len() != groups.len() {
        errors.push("golden expectation rows must cover every profile/radius group".to_owned());
    }
    let mut golden_pairs = BTreeSet::new();
    for golden in golden_values {
        let profile_id = str_field(golden, "profile_id").unwrap_or("UNKNOWN");
        let radius_id = str_field(golden, "radius_level_id").unwrap_or("UNKNOWN");
        if !golden_pairs.insert((profile_id.to_owned(), radius_id.to_owned())) {
            errors.push(format!("duplicate golden expectation row: {profile_id}/{radius_id}"));
        }
        compare_golden(&groups, golden, &mut errors);
    }
    if golden_pairs.len() != groups.len() {
        errors.push("golden expectations do not cover every profile/radius group".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-013-WASSERSTEIN-JOINT-DECISION-001",
        "source_benchmark_id": "PRAXIS-NRES-009-JOINT-SERVICE-FORECAST-001",
        "source_fixture_git_blob_sha1": NRES009_SOURCE_GIT_BLOB_SHA1,
        "method": "exact finite-support Kantorovich dual over weighted-Hamming joint states",
        "source_case_count": source_by_id.len(),
        "radius_tier_count": radius_by_id.len(),
        "paired_evaluation_row_count": per_case.len(),
        "unique_source_case_count": source_by_id.len(),
        "decision_profile_count": profiles.len(),
        "groups": groups,
        "per_case": per_case,
        "interpretation": [
            "A successful report verifies the declared public synthetic fixture and deterministic finite-support dual calculations only.",
            "The radius is a sensitivity parameter, not a calibrated confidence level.",
            "All policies are reported separately; no composite robustness or resilience score is computed.",
            "The perfect-information cost is a statewise reference lower bound, not a deployment policy."
        ]
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SOURCE_FIXTURE: &str =
        include_str!("../../../examples/courses/civic-resilience-joint-service-forecast-fixture-001.json");
    const NRES013_FIXTURE: &str =
        include_str!("../../../examples/courses/civic-resilience-wasserstein-joint-decision-fixture-001.json");

    fn binary_distances() -> Vec<Vec<f64>> {
        vec![vec![0.0, 1.0], vec![1.0, 0.0]]
    }

    #[test]
    fn zero_radius_equals_nominal_expectation() {
        let (value, lambda) = wasserstein_upper_expectation(
            &[0.25, 0.75], &[0.0, 10.0], &binary_distances(), 0.0, 1.0,
        ).expect("valid dual");
        assert!((value - 7.5).abs() < 1e-12);
        assert!(lambda.is_none());
    }

    #[test]
    fn positive_radius_uses_exact_upper_envelope_breakpoint() {
        let (value, lambda) = wasserstein_upper_expectation(
            &[0.25, 0.75], &[0.0, 10.0], &binary_distances(), 0.10, 1.0,
        ).expect("valid dual");
        assert!((value - 8.5).abs() < 1e-10);
        assert_eq!(lambda, Some(10.0));
    }

    #[test]
    fn radius_at_transport_diameter_reaches_support_maximum() {
        let (value, lambda) = wasserstein_upper_expectation(
            &[0.25, 0.75], &[0.0, 10.0], &binary_distances(), 1.0, 1.0,
        ).expect("valid dual");
        assert!((value - 10.0).abs() < 1e-12);
        assert_eq!(lambda, Some(0.0));
    }

    #[test]
    fn robust_value_is_monotone_in_transport_radius() {
        let p = [0.4, 0.6];
        let losses = [2.0, 7.0];
        let distances = binary_distances();
        let values: Vec<f64> = [0.0, 0.05, 0.2, 0.5, 1.0].iter()
            .map(|radius| wasserstein_upper_expectation(&p, &losses, &distances, *radius, 1.0)
                .expect("valid dual").0).collect();
        assert!(values.windows(2).all(|window| window[0] <= window[1] + 1e-12));
    }

    #[test]
    fn weighted_hamming_is_symmetric_and_obeys_triangle_inequality() {
        let weights = [1.0, 1.0, 1.25, 1.5];
        for a in 0..16 {
            for b in 0..16 {
                assert_eq!(weighted_hamming(a, b, &weights), weighted_hamming(b, a, &weights));
                for c in 0..16 {
                    assert!(weighted_hamming(a, c, &weights)
                        <= weighted_hamming(a, b, &weights) + weighted_hamming(b, c, &weights) + 1e-12);
                }
            }
        }
    }

    #[test]
    fn valid_fixture_matches_twelve_golden_cells_and_thirty_two_rows() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).expect("source fixture parses");
        let fixture: Value = serde_json::from_str(NRES013_FIXTURE).expect("NRES-013 fixture parses");
        let report = evaluate_wasserstein_joint_decisions(&source, &fixture).expect("valid fixture");
        assert_eq!(report["source_case_count"], json!(8));
        assert_eq!(report["radius_tier_count"], json!(4));
        assert_eq!(report["paired_evaluation_row_count"], json!(32));
        assert_eq!(report["groups"].as_array().map(Vec::len), Some(12));
    }

    #[test]
    fn copied_source_forecast_change_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["evaluation_cases"][0]["candidate_output"]["forecast_distribution"][0]["probability"] = json!(0.25);
        let errors = evaluate_wasserstein_joint_decisions(&source, &fixture).expect_err("forecast drift fails");
        assert!(errors.iter().any(|error| error.contains("copied candidate distribution differs")));
    }

    #[test]
    fn copied_observation_change_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["evaluation_cases"][0]["evaluator"]["observed_breach_mask"] = json!(15);
        let errors = evaluate_wasserstein_joint_decisions(&source, &fixture).expect_err("observation drift fails");
        assert!(errors.iter().any(|error| error.contains("observed joint state differs")));
    }

    #[test]
    fn wrong_source_blob_pin_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["source"]["fixture_git_blob_sha1"] = json!("0000000000000000000000000000000000000000");
        let errors = evaluate_wasserstein_joint_decisions(&source, &fixture).expect_err("wrong pin fails");
        assert!(errors.iter().any(|error| error.contains("pinned Git blob SHA-1 differs")));
    }

    #[test]
    fn negative_radius_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["radius_levels"][0]["wasserstein_radius"] = json!(-0.1);
        assert!(evaluate_wasserstein_joint_decisions(&source, &fixture).is_err());
    }

    #[test]
    fn zero_metric_weight_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["protocol"]["ground_metric"]["service_change_weights"][0] = json!(0.0);
        assert!(evaluate_wasserstein_joint_decisions(&source, &fixture).is_err());
    }

    #[test]
    fn duplicate_state_mask_is_rejected() {
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        let case = &mut fixture["evaluation_cases"][0];
        case["candidate_output"]["forecast_distribution"][1]["state_mask"] = json!(0);
        let mut errors = Vec::new();
        let parsed = parse_joint_probabilities(case, "duplicate-mask", 16, 1e-12, &mut errors);
        assert!(parsed.is_none());
        assert!(errors.iter().any(|error| error.contains("duplicate state mask")));
    }

    #[test]
    fn non_unit_probability_mass_is_rejected() {
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        let case = &mut fixture["evaluation_cases"][0];
        case["candidate_output"]["forecast_distribution"][0]["probability"] = json!(0.5);
        let mut errors = Vec::new();
        let parsed = parse_joint_probabilities(case, "bad-mass", 16, 1e-12, &mut errors);
        assert!(parsed.is_none());
        assert!(errors.iter().any(|error| error.contains("joint probability mass")));
    }

    #[test]
    fn missing_source_radius_pair_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["evaluation_cases"].as_array_mut().unwrap().pop();
        assert!(evaluate_wasserstein_joint_decisions(&source, &fixture).is_err());
    }

    #[test]
    fn duplicate_source_radius_pair_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        let duplicate = fixture["evaluation_cases"][0].clone();
        fixture["evaluation_cases"].as_array_mut().unwrap().push(duplicate);
        assert!(evaluate_wasserstein_joint_decisions(&source, &fixture).is_err());
    }

    #[test]
    fn prohibited_composite_score_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["composite_score"] = json!(1.0);
        let errors = evaluate_wasserstein_joint_decisions(&source, &fixture).expect_err("composite fails");
        assert!(errors.iter().any(|error| error.contains("forbidden field")));
    }

    #[test]
    fn altered_golden_metric_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        fixture["golden_expectations"][0]["nominal_point_mean_realized_cost"] = json!(999.0);
        assert!(evaluate_wasserstein_joint_decisions(&source, &fixture).is_err());
    }
    #[test]
    fn wasserstein_expected_loss_changes_actions_at_moderate_cost_as_radius_grows() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let fixture: Value = serde_json::from_str(NRES013_FIXTURE).unwrap();
        let report = evaluate_wasserstein_joint_decisions(&source, &fixture).expect("valid fixture");
        let group = report["groups"].as_array().unwrap().iter().find(|item| {
            item["profile_id"] == json!("moderate_preparation_cost")
                && item["radius_level_id"] == json!("narrow")
        }).unwrap();
        let strategies = &group["strategies"];
        assert_eq!(strategies["nominal_point_forecast"]["action_counts"]["targeted"], json!(5));
        assert_eq!(strategies["wasserstein_worst_case_expected_loss"]["action_counts"]["targeted"], json!(2));
        assert_eq!(strategies["wasserstein_worst_case_expected_loss"]["action_counts"]["comprehensive"], json!(6));
        assert_eq!(strategies["wasserstein_minimax_regret"]["action_counts"]["targeted"], json!(5));
    }

}

//! Split-conformal calibration of finite-support Wasserstein ambiguity radii.
//!
//! The calibration unit is one source-family block. Repeated outcomes inside a
//! block form an empirical joint-state distribution; they are not counted as
//! independent calibration samples. The guarantee is marginal block-score
//! coverage under block exchangeability, not population-law or shift-invariant
//! coverage.

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

const SOURCE_BLOB_SHA1: &str = "799799f8a315d4c0f06a43b5dce5ab2e15ccefb5";
const SOURCE_PATH: &str =
    "mycelix-workspace/mycelix-praxis/examples/courses/civic-resilience-joint-service-forecast-fixture-001.json";
const TOLERANCE: f64 = 1e-10;

fn str_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn num_field(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64)
}

fn array_field<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn close(left: f64, right: f64, tolerance: f64) -> bool {
    left.is_finite() && right.is_finite() && (left - right).abs() <= tolerance
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

#[derive(Clone)]
struct Edge {
    to: usize,
    reverse_index: usize,
    capacity: f64,
    cost: f64,
}

fn add_edge(graph: &mut [Vec<Edge>], from: usize, to: usize, capacity: f64, cost: f64) {
    let forward_reverse = graph[to].len();
    let reverse_reverse = graph[from].len();
    graph[from].push(Edge {
        to,
        reverse_index: forward_reverse,
        capacity,
        cost,
    });
    graph[to].push(Edge {
        to: from,
        reverse_index: reverse_reverse,
        capacity: 0.0,
        cost: -cost,
    });
}

/// Solve the finite transportation problem with successive shortest augmenting
/// paths. Bellman-Ford is used because residual reverse arcs have negative cost.
fn min_cost_transport(
    supply: &[f64],
    demand: &[f64],
    distances: &[Vec<f64>],
) -> Result<f64, String> {
    let state_count = supply.len();
    if state_count == 0
        || demand.len() != state_count
        || distances.len() != state_count
        || distances.iter().any(|row| row.len() != state_count)
    {
        return Err("transport dimensions are inconsistent".to_owned());
    }
    if supply.iter().chain(demand.iter()).any(|value| {
        !value.is_finite() || *value < 0.0
    }) {
        return Err("transport masses must be finite and non-negative".to_owned());
    }
    let supply_mass: f64 = supply.iter().sum();
    let demand_mass: f64 = demand.iter().sum();
    if !close(supply_mass, 1.0, 1e-9) || !close(demand_mass, 1.0, 1e-9) {
        return Err("transport supply and demand must each sum to one".to_owned());
    }

    let vertices = 2 * state_count + 2;
    let source = 0;
    let sink = vertices - 1;
    let mut graph: Vec<Vec<Edge>> = vec![Vec::new(); vertices];

    for (index, mass) in supply.iter().enumerate() {
        add_edge(&mut graph, source, 1 + index, *mass, 0.0);
    }
    for (left, row) in distances.iter().enumerate() {
        for (right, distance) in row.iter().enumerate() {
            if !distance.is_finite() || *distance < 0.0 {
                return Err("transport distances must be finite and non-negative".to_owned());
            }
            add_edge(&mut graph, 1 + left, 1 + state_count + right, 1.0, *distance);
        }
    }
    for (index, mass) in demand.iter().enumerate() {
        add_edge(&mut graph, 1 + state_count + index, sink, *mass, 0.0);
    }

    let mut flow = 0.0;
    let mut total_cost = 0.0;
    let mut augmentations = 0usize;
    while flow < 1.0 - 1e-10 {
        augmentations += 1;
        if augmentations > vertices * vertices * 4 {
            return Err("transport solver exceeded augmentation limit".to_owned());
        }
        let mut distance_to: Vec<f64> = vec![f64::INFINITY; vertices];
        let mut previous_node: Vec<usize> = vec![usize::MAX; vertices];
        let mut previous_edge: Vec<usize> = vec![usize::MAX; vertices];
        distance_to[source] = 0.0;

        for _ in 0..(vertices - 1) {
            let mut changed = false;
            for from in 0..vertices {
                if !distance_to[from].is_finite() {
                    continue;
                }
                for edge_index in 0..graph[from].len() {
                    let edge = &graph[from][edge_index];
                    if edge.capacity <= 1e-12 {
                        continue;
                    }
                    let candidate = distance_to[from] + edge.cost;
                    if candidate < distance_to[edge.to] - 1e-12 {
                        distance_to[edge.to] = candidate;
                        previous_node[edge.to] = from;
                        previous_edge[edge.to] = edge_index;
                        changed = true;
                    }
                }
            }
            if !changed {
                break;
            }
        }

        if !distance_to[sink].is_finite() {
            return Err("transport graph cannot route all probability mass".to_owned());
        }

        let mut increment = 1.0 - flow;
        let mut node = sink;
        let mut path_length = 0usize;
        while node != source {
            path_length += 1;
            if path_length > vertices
                || previous_node[node] == usize::MAX
                || previous_edge[node] == usize::MAX
            {
                return Err("transport shortest path is malformed".to_owned());
            }
            let from = previous_node[node];
            let edge_index = previous_edge[node];
            increment = increment.min(graph[from][edge_index].capacity);
            node = from;
        }
        if increment <= 1e-12 || !increment.is_finite() {
            return Err("transport augmentation has zero or invalid capacity".to_owned());
        }

        node = sink;
        while node != source {
            let from = previous_node[node];
            let edge_index = previous_edge[node];
            let to = graph[from][edge_index].to;
            let reverse_index = graph[from][edge_index].reverse_index;
            graph[from][edge_index].capacity -= increment;
            graph[to][reverse_index].capacity += increment;
            node = from;
        }
        flow += increment;
        total_cost += increment * distance_to[sink];
    }

    if !close(flow, 1.0, 1e-8) || !total_cost.is_finite() || total_cost < -1e-10 {
        return Err("transport solver did not produce a finite unit-mass solution".to_owned());
    }
    Ok(total_cost.max(0.0))
}

fn weighted_hamming(left: usize, right: usize, weights: &[f64]) -> f64 {
    weights
        .iter()
        .enumerate()
        .filter(|(bit, _)| ((left >> *bit) & 1) != ((right >> *bit) & 1))
        .map(|(_, weight)| *weight)
        .sum()
}

fn distance_matrix(state_count: usize, weights: &[f64]) -> Vec<Vec<f64>> {
    (0..state_count)
        .map(|left| {
            (0..state_count)
                .map(|right| weighted_hamming(left, right, weights))
                .collect()
        })
        .collect()
}

fn parse_distribution(
    value: &Value,
    state_count: usize,
    case_id: &str,
    errors: &mut Vec<String>,
) -> Option<Vec<f64>> {
    let Some(entries) = array_field(value, "forecast_distribution") else {
        errors.push(format!("{case_id}: forecast_distribution is missing"));
        return None;
    };
    if entries.len() != state_count {
        errors.push(format!("{case_id}: forecast must enumerate all {state_count} states"));
        return None;
    }
    let mut probabilities: Vec<Option<f64>> = vec![None; state_count];
    for entry in entries {
        let mask = entry.get("state_mask").and_then(Value::as_u64);
        let probability = num_field(entry, "probability");
        let Some(mask) = mask.filter(|mask| *mask < state_count as u64) else {
            errors.push(format!("{case_id}: state mask is outside the support"));
            continue;
        };
        let Some(probability) = probability.filter(|number| {
            number.is_finite() && (0.0..=1.0).contains(number)
        }) else {
            errors.push(format!("{case_id}: probability is not finite or outside [0,1]"));
            continue;
        };
        if probabilities[mask as usize].replace(probability).is_some() {
            errors.push(format!("{case_id}: duplicate state mask {mask}"));
        }
    }
    if probabilities.iter().any(Option::is_none) {
        errors.push(format!("{case_id}: forecast misses one or more state masks"));
        return None;
    }
    let mut result: Vec<f64> = probabilities.into_iter().flatten().collect();
    let mass: f64 = result.iter().sum();
    if !close(mass, 1.0, 1e-12) {
        errors.push(format!("{case_id}: forecast probability mass differs from one"));
        return None;
    }
    for probability in &mut result {
        *probability /= mass;
    }
    Some(result)
}

fn empirical_distribution(
    block: &Value,
    state_count: usize,
    block_size: usize,
    block_id: &str,
    errors: &mut Vec<String>,
) -> Option<Vec<f64>> {
    let Some(outcomes) = array_field(block, "observed_breach_masks") else {
        errors.push(format!("{block_id}: observed_breach_masks is missing"));
        return None;
    };
    if outcomes.len() != block_size {
        errors.push(format!("{block_id}: observed block size differs from protocol"));
        return None;
    }
    let mut counts = vec![0usize; state_count];
    let mut valid = true;
    for outcome in outcomes {
        let Some(mask) = outcome.as_u64().filter(|mask| *mask < state_count as u64) else {
            errors.push(format!("{block_id}: observed state mask is outside support"));
            valid = false;
            continue;
        };
        counts[mask as usize] += 1;
    }
    if !valid {
        return None;
    }
    let distribution: Vec<f64> = counts
        .iter()
        .map(|count| *count as f64 / block_size as f64)
        .collect();
    let Some(declared) = block.get("evaluator_empirical_distribution").and_then(Value::as_array) else {
        errors.push(format!("{block_id}: evaluator empirical distribution is missing"));
        return None;
    };
    if declared.len() != state_count
        || declared.iter().zip(distribution.iter()).any(|(value, expected)| {
            value.as_f64().is_none_or(|actual| !close(actual, *expected, 1e-12))
        })
    {
        errors.push(format!("{block_id}: evaluator empirical distribution differs from observed outcomes"));
        return None;
    }
    Some(distribution)
}

fn block_score(
    block: &Value,
    source_cases: &BTreeMap<String, &Value>,
    state_count: usize,
    block_size: usize,
    distances: &[Vec<f64>],
    ids: &mut BTreeSet<String>,
    families: &mut BTreeSet<String>,
    errors: &mut Vec<String>,
) -> Option<(String, String, f64)> {
    let block_id = str_field(block, "block_id").unwrap_or("UNKNOWN").to_owned();
    let family_id = str_field(block, "source_family_id").unwrap_or("").to_owned();
    if block_id == "UNKNOWN" || !ids.insert(block_id.clone()) {
        errors.push(format!("{block_id}: block IDs must be present and globally unique"));
    }
    if family_id.is_empty() || !families.insert(family_id.clone()) {
        errors.push(format!("{block_id}: source-family IDs must be present and globally unique across splits"));
    }
    let Some(template_id) = str_field(block, "forecast_template_source_case_id") else {
        errors.push(format!("{block_id}: forecast template source case is missing"));
        return None;
    };
    let Some(source_case) = source_cases.get(template_id) else {
        errors.push(format!("{block_id}: unknown NRES-009 forecast template {template_id}"));
        return None;
    };
    let Some(template_distribution) = source_case.get("candidate_output").and_then(|value| value.get("forecast_distribution")) else {
        errors.push(format!("{block_id}: pinned source forecast template is missing"));
        return None;
    };
    if block.get("forecast_distribution") != Some(template_distribution) {
        errors.push(format!("{block_id}: forecast differs from its pinned NRES-009 template"));
        return None;
    }
    let Some(forecast) = parse_distribution(block, state_count, &block_id, errors) else {
        return None;
    };
    let Some(empirical) = empirical_distribution(block, state_count, block_size, &block_id, errors) else {
        return None;
    };
    let Ok(score) = min_cost_transport(&forecast, &empirical, distances) else {
        errors.push(format!("{block_id}: Wasserstein transport calculation failed"));
        return None;
    };
    if num_field(block, "nonconformity_score_w1")
        .is_none_or(|value| !close(value, score, TOLERANCE))
    {
        errors.push(format!("{block_id}: declared Wasserstein nonconformity score differs from recomputation"));
        return None;
    }
    Some((block_id, family_id, score))
}

/// Return the split-conformal one-based order-statistic rank and finite radius.
/// None represents the mathematically correct +infinity threshold when rank > m.
fn conformal_radius(scores: &[f64], alpha: f64) -> Result<(usize, Option<f64>), String> {
    if !alpha.is_finite() || alpha <= 0.0 || alpha >= 1.0 {
        return Err("target miscoverage alpha must be finite and strictly between zero and one".to_owned());
    }
    if scores.is_empty() {
        return Err("at least one calibration block is required".to_owned());
    }
    if scores.iter().any(|score| !score.is_finite() || *score < 0.0) {
        return Err("calibration scores must be finite and non-negative".to_owned());
    }
    let rank = (((scores.len() + 1) as f64) * (1.0 - alpha)).ceil() as usize;
    if rank > scores.len() {
        return Ok((rank, None));
    }
    let mut sorted = scores.to_vec();
    sorted.sort_by(f64::total_cmp);
    Ok((rank, Some(sorted[rank - 1])))
}

fn compare_score_vectors(actual: &[f64], expected_value: Option<&Value>, label: &str, errors: &mut Vec<String>) {
    let Some(expected) = expected_value.and_then(Value::as_array) else {
        errors.push(format!("{label}: expected sorted score vector is missing"));
        return;
    };
    if actual.len() != expected.len()
        || actual.iter().zip(expected.iter()).any(|(left, right)| {
            right.as_f64().is_none_or(|value| !close(*left, value, TOLERANCE))
        })
    {
        errors.push(format!("{label}: sorted score vector differs from recomputation"));
    }
}

/// Evaluate split-conformal calibration of a Wasserstein radius on block scores.
/// The source fixture supplies the service catalog and forecast templates.
pub fn evaluate_wasserstein_radius_calibration(
    source_joint: &Value,
    fixture: &Value,
) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    forbidden_key_scan(fixture, "", &mut errors);
    if str_field(fixture, "benchmark_id")
        != Some("PRAXIS-NRES-014-SPLIT-CONFORMAL-WASSERSTEIN-RADIUS-001")
    {
        errors.push("unexpected NRES-014 benchmark_id".to_owned());
    }
    if str_field(fixture, "schema_version") != Some("1.0.0")
        || str_field(fixture, "status") != Some("synthetic-demonstration-for-review")
        || str_field(fixture, "evaluation_split") != Some("public-development-only")
        || fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false)
    {
        errors.push("NRES-014 schema version, synthetic status, split or inactive boundary is invalid".to_owned());
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
        return Err(vec!["NRES-014 source metadata is missing".to_owned()]);
    };
    if str_field(source_info, "fixture_git_blob_sha1") != Some(SOURCE_BLOB_SHA1)
        || str_field(source_info, "fixture_path") != Some(SOURCE_PATH)
        || str_field(source_info, "benchmark_id")
            != Some("PRAXIS-NRES-009-JOINT-SERVICE-FORECAST-001")
    {
        errors.push("NRES-014 pinned NRES-009 source metadata differs".to_owned());
    }

    let Some(services) = array_field(fixture, "service_catalog") else {
        return Err(vec!["NRES-014 service catalog is missing".to_owned()]);
    };
    let Some(source_services) = array_field(source_joint, "service_catalog") else {
        return Err(vec!["NRES-009 service catalog is missing".to_owned()]);
    };
    if services != source_services {
        errors.push("NRES-014 service catalog differs from NRES-009".to_owned());
    }
    let dimension = services.len();
    if !(2..=6).contains(&dimension) {
        errors.push("Wasserstein radius calibration supports 2 through 6 service dimensions".to_owned());
        return Err(errors);
    }
    let state_count = 1usize << dimension;
    if source_joint.pointer("/protocol/state_count").and_then(Value::as_u64)
        != Some(state_count as u64)
    {
        return Err(vec!["NRES-009 source state_count differs from the service catalog".to_owned()]);
    }

    let Some(protocol) = fixture.get("protocol") else {
        return Err(vec!["NRES-014 calibration protocol is missing".to_owned()]);
    };
    let alpha = num_field(protocol, "target_miscoverage");
    let target_coverage = num_field(protocol, "target_marginal_coverage");
    let block_size = protocol.get("block_size").and_then(Value::as_u64).map(|value| value as usize);
    let Some(weights_value) = protocol.pointer("/ground_metric/service_change_weights").and_then(Value::as_array) else {
        return Err(vec!["NRES-014 ground metric weights are missing".to_owned()]);
    };
    if alpha.is_none_or(|value| !value.is_finite() || value <= 0.0 || value >= 1.0)
        || target_coverage.zip(alpha).is_none_or(|(coverage, alpha)| !close(coverage, 1.0 - alpha, 1e-12))
        || block_size.is_none_or(|size| size == 0)
        || protocol.get("state_count").and_then(Value::as_u64) != Some(state_count as u64)
        || str_field(protocol.get("ground_metric").unwrap_or(&Value::Null), "name") != Some("weighted_hamming")
        || weights_value.len() != dimension
        || weights_value.iter().any(|weight| {
            weight.as_f64().is_none_or(|value| !value.is_finite() || value <= 0.0)
        })
    {
        errors.push("alpha, coverage, block size, state count or ground metric is invalid".to_owned());
        return Err(errors);
    }
    let block_size = block_size.unwrap();
    let weights: Vec<f64> = weights_value.iter().filter_map(Value::as_f64).collect();
    let distances = distance_matrix(state_count, &weights);
    let Some(source_case_values) = array_field(source_joint, "evaluation_cases") else {
        return Err(vec!["NRES-009 source cases are missing".to_owned()]);
    };
    let mut source_cases = BTreeMap::new();
    for source_case in source_case_values {
        if let Some(id) = str_field(source_case, "case_id") {
            if source_cases.insert(id.to_owned(), source_case).is_some() {
                errors.push(format!("duplicate NRES-009 source case ID: {id}"));
            }
        }
    }

    let Some(calibration_blocks) = array_field(fixture, "calibration_blocks") else {
        return Err(vec!["NRES-014 calibration_blocks are missing".to_owned()]);
    };
    let Some(development_blocks) = array_field(fixture, "development_evaluation_blocks") else {
        return Err(vec!["NRES-014 development_evaluation_blocks are missing".to_owned()]);
    };
    if protocol.get("calibration_block_count").and_then(Value::as_u64)
        != Some(calibration_blocks.len() as u64)
        || protocol.get("development_block_count").and_then(Value::as_u64)
            != Some(development_blocks.len() as u64)
    {
        errors.push("declared calibration/development block counts differ from fixture arrays".to_owned());
    }

    let mut ids = BTreeSet::new();
    let mut families = BTreeSet::new();
    let mut calibration_scores = Vec::new();
    for block in calibration_blocks {
        if let Some((_, _, score)) = block_score(
            block, &source_cases, state_count, block_size, &distances,
            &mut ids, &mut families, &mut errors,
        ) {
            calibration_scores.push(score);
        }
    }
    let mut development_scores = Vec::new();
    for block in development_blocks {
        if let Some((_, _, score)) = block_score(
            block, &source_cases, state_count, block_size, &distances,
            &mut ids, &mut families, &mut errors,
        ) {
            development_scores.push(score);
        }
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    if calibration_scores.len() != calibration_blocks.len()
        || development_scores.len() != development_blocks.len()
    {
        return Err(vec!["one or more calibration/development blocks could not be scored".to_owned()]);
    }

    let (rank, radius) = conformal_radius(&calibration_scores, alpha.unwrap())
        .map_err(|error| vec![error])?;
    let mut sorted_scores = calibration_scores.clone();
    sorted_scores.sort_by(f64::total_cmp);
    let expected = fixture.get("expected_calibration").unwrap_or(&Value::Null);
    if expected.get("score_count").and_then(Value::as_u64) != Some(calibration_scores.len() as u64)
        || expected.get("one_based_order_statistic_rank").and_then(Value::as_u64) != Some(rank as u64)
        || num_field(expected, "target_marginal_coverage").zip(target_coverage)
            .is_none_or(|(actual, target)| !close(actual, target, 1e-12))
        || expected.get("radius_is_finite").and_then(Value::as_bool) != Some(radius.is_some())
    {
        errors.push("expected calibration rank, count, coverage or finite-radius flag differs".to_owned());
    }
    compare_score_vectors(&sorted_scores, expected.get("sorted_scores"), "calibration", &mut errors);
    match (radius, num_field(expected, "calibrated_radius")) {
        (Some(actual), Some(declared)) if close(actual, declared, TOLERANCE) => {}
        (None, None) => {}
        _ => errors.push("declared calibrated radius differs from conformal order statistic".to_owned()),
    }

    let development_covered = match radius {
        Some(threshold) => development_scores
            .iter()
            .filter(|score| **score <= threshold + 1e-12)
            .count(),
        None => 0,
    };
    let development_coverage = if development_scores.is_empty() {
        None
    } else {
        Some(development_covered as f64 / development_scores.len() as f64)
    };
    let development_expected = fixture.get("expected_development_diagnostic").unwrap_or(&Value::Null);
    if development_expected.get("evaluation_blocks").and_then(Value::as_u64)
        != Some(development_scores.len() as u64)
        || development_expected.get("covered_blocks").and_then(Value::as_u64)
            != Some(development_covered as u64)
        || num_field(development_expected, "empirical_coverage").zip(development_coverage)
            .is_none_or(|(actual, expected)| !close(actual, expected, 1e-12))
    {
        errors.push("development diagnostic differs from recomputed holdout coverage".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-014-SPLIT-CONFORMAL-WASSERSTEIN-RADIUS-001",
        "source_benchmark_id": "PRAXIS-NRES-009-JOINT-SERVICE-FORECAST-001",
        "source_fixture_git_blob_sha1": SOURCE_BLOB_SHA1,
        "calibration_method": "split-conformal order statistic over block-level empirical W1 scores",
        "calibration_block_count": calibration_scores.len(),
        "development_block_count": development_scores.len(),
        "target_miscoverage": alpha,
        "target_marginal_coverage": target_coverage,
        "one_based_order_statistic_rank": rank,
        "sorted_calibration_scores": sorted_scores,
        "calibrated_wasserstein_radius": radius,
        "radius_is_finite": radius.is_some(),
        "development_diagnostic": {
            "evaluation_blocks": development_scores.len(),
            "covered_blocks": development_covered,
            "empirical_coverage": development_coverage,
            "scores": development_scores,
            "interpretation": "Public synthetic development diagnostic only; not a confirmatory coverage estimate."
        },
        "guarantee_boundary": [
            "Under block exchangeability the rank provides marginal coverage for a future block score jointly over calibration and future blocks.",
            "The score concerns the future block empirical distribution, not the unknown population distribution.",
            "The result is not conditional coverage for the realized calibration sample and is not valid under arbitrary unmodeled shift."
        ]
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SOURCE_FIXTURE: &str =
        include_str!("../../../examples/courses/civic-resilience-joint-service-forecast-fixture-001.json");
    const NRES014_FIXTURE: &str =
        include_str!("../../../examples/courses/civic-resilience-wasserstein-radius-calibration-fixture-001.json");

    fn binary_distances(weight: f64) -> Vec<Vec<f64>> {
        vec![vec![0.0, weight], vec![weight, 0.0]]
    }

    #[test]
    fn transport_solver_can_reassign_mass_through_residual_reverse_arcs() {
        let distances = vec![
            vec![0.0, 1.0, 2.0],
            vec![1.0, 0.0, 1.0],
            vec![2.0, 1.0, 0.0],
        ];
        let supply = [0.5, 0.5, 0.0];
        let demand = [0.0, 0.5, 0.5];
        let value = min_cost_transport(&supply, &demand, &distances).unwrap();
        assert!(close(value, 1.0, 1e-12));
    }

    #[test]
    fn identical_distributions_have_zero_wasserstein_distance() {
        let p = [0.25, 0.75];
        assert!(close(min_cost_transport(&p, &p, &binary_distances(1.0)).unwrap(), 0.0, 1e-12));
    }

    #[test]
    fn point_mass_transport_equals_ground_distance() {
        let p = [1.0, 0.0];
        let q = [0.0, 1.0];
        assert!(close(min_cost_transport(&p, &q, &binary_distances(2.5)).unwrap(), 2.5, 1e-12));
    }

    #[test]
    fn partial_mass_transport_scales_linearly() {
        let p = [0.5, 0.5];
        let q = [0.25, 0.75];
        assert!(close(min_cost_transport(&p, &q, &binary_distances(2.0)).unwrap(), 0.5, 1e-12));
    }

    #[test]
    fn wasserstein_distance_is_symmetric_for_two_state_case() {
        let p = [0.2, 0.8];
        let q = [0.7, 0.3];
        let d = binary_distances(1.75);
        assert!(close(min_cost_transport(&p, &q, &d).unwrap(), min_cost_transport(&q, &p, &d).unwrap(), 1e-12));
    }

    #[test]
    fn metric_distance_is_symmetric_and_obeys_triangle_inequality() {
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
    fn split_conformal_rank_for_nineteen_blocks_and_ten_percent_miscoverage() {
        let scores: Vec<f64> = (1..=19).map(|value| value as f64).collect();
        let (rank, radius) = conformal_radius(&scores, 0.10).unwrap();
        assert_eq!(rank, 18);
        assert_eq!(radius, Some(18.0));
    }

    #[test]
    fn insufficient_calibration_blocks_return_infinite_radius() {
        let scores: Vec<f64> = (1..=8).map(|value| value as f64).collect();
        let (rank, radius) = conformal_radius(&scores, 0.10).unwrap();
        assert_eq!(rank, 9);
        assert_eq!(radius, None);
    }

    #[test]
    fn conformal_ties_use_inclusive_order_statistic() {
        let scores = [0.1, 0.2, 0.2, 0.2, 0.7];
        let (rank, radius) = conformal_radius(&scores, 0.30).unwrap();
        assert_eq!(rank, 5);
        assert_eq!(radius, Some(0.7));
    }

    #[test]
    fn invalid_miscoverage_is_rejected() {
        for alpha in [0.0, 1.0, -0.1, f64::NAN, f64::INFINITY] {
            assert!(conformal_radius(&[0.1, 0.2], alpha).is_err());
        }
    }

    #[test]
    fn empty_calibration_split_is_rejected() {
        assert!(conformal_radius(&[], 0.1).is_err());
    }

    #[test]
    fn invalid_transport_mass_is_rejected() {
        assert!(min_cost_transport(&[0.2, 0.2], &[0.5, 0.5], &binary_distances(1.0)).is_err());
    }

    #[test]
    fn invalid_transport_cost_is_rejected() {
        let d = vec![vec![0.0, f64::NAN], vec![1.0, 0.0]];
        assert!(min_cost_transport(&[0.5, 0.5], &[0.5, 0.5], &d).is_err());
    }

    #[test]
    fn valid_fixture_matches_calibrated_radius_and_development_diagnostic() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).expect("source fixture parses");
        let fixture: Value = serde_json::from_str(NRES014_FIXTURE).expect("NRES-014 fixture parses");
        let report = evaluate_wasserstein_radius_calibration(&source, &fixture).expect("valid fixture");
        assert_eq!(report["calibration_block_count"], json!(19));
        assert_eq!(report["development_block_count"], json!(12));
        assert_eq!(report["one_based_order_statistic_rank"], json!(18));
        assert!((report["calibrated_wasserstein_radius"].as_f64().unwrap() - 0.9211372893629722).abs() < 1e-10);
    }

    #[test]
    fn copied_forecast_template_tampering_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES014_FIXTURE).unwrap();
        fixture["calibration_blocks"][0]["forecast_distribution"][0]["probability"] = json!(0.25);
        assert!(evaluate_wasserstein_radius_calibration(&source, &fixture).is_err());
    }

    #[test]
    fn empirical_distribution_tampering_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES014_FIXTURE).unwrap();
        fixture["calibration_blocks"][0]["evaluator_empirical_distribution"][0] = json!(0.5);
        assert!(evaluate_wasserstein_radius_calibration(&source, &fixture).is_err());
    }

    #[test]
    fn duplicate_source_family_across_splits_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES014_FIXTURE).unwrap();
        let family = fixture["calibration_blocks"][0]["source_family_id"].clone();
        fixture["development_evaluation_blocks"][0]["source_family_id"] = family;
        assert!(evaluate_wasserstein_radius_calibration(&source, &fixture).is_err());
    }

    #[test]
    fn wrong_source_fixture_pin_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES014_FIXTURE).unwrap();
        fixture["source"]["fixture_git_blob_sha1"] = json!("0000000000000000000000000000000000000000");
        assert!(evaluate_wasserstein_radius_calibration(&source, &fixture).is_err());
    }

    #[test]
    fn invalid_metric_weight_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES014_FIXTURE).unwrap();
        fixture["protocol"]["ground_metric"]["service_change_weights"][0] = json!(0.0);
        assert!(evaluate_wasserstein_radius_calibration(&source, &fixture).is_err());
    }

    #[test]
    fn altered_calibration_radius_is_rejected() {
        let source: Value = serde_json::from_str(SOURCE_FIXTURE).unwrap();
        let mut fixture: Value = serde_json::from_str(NRES014_FIXTURE).unwrap();
        fixture["expected_calibration"]["calibrated_radius"] = json!(0.1);
        assert!(evaluate_wasserstein_radius_calibration(&source, &fixture).is_err());
    }
}

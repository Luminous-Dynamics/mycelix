//! Deterministic NRES-007 service-trajectory evaluator.
//!
//! This module evaluates public synthetic service-availability trajectories.
//! It does not model real infrastructure, score people, determine legal authority,
//! issue operational instructions, or estimate causal intervention effects.

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

fn str_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key)?.as_str()
}

fn num_field(value: &Value, key: &str) -> Option<f64> {
    value.get(key)?.as_f64()
}

fn array_field<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key)?.as_array()
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

/// Integrate max(0, deficit) for a linear segment with endpoint deficits.
/// The result is exact for the declared piecewise-linear interpolation rule.
fn positive_linear_area(d0: f64, d1: f64, duration: f64) -> f64 {
    if d0 <= 0.0 && d1 <= 0.0 {
        0.0
    } else if d0 >= 0.0 && d1 >= 0.0 {
        duration * (d0 + d1) / 2.0
    } else if d0 > 0.0 {
        duration * d0 * d0 / (2.0 * (d0 - d1))
    } else {
        duration * d1 * d1 / (2.0 * (d1 - d0))
    }
}

fn deficit_area(floor: f64, samples: &[(f64, f64)]) -> f64 {
    samples
        .windows(2)
        .map(|pair| {
            let (t0, a0) = pair[0];
            let (t1, a1) = pair[1];
            positive_linear_area(floor - a0, floor - a1, t1 - t0)
        })
        .sum()
}

fn sustained_recovery_hour(floor: f64, samples: &[(f64, f64)]) -> Option<f64> {
    (0..samples.len()).find_map(|index| {
        samples[index..]
            .iter()
            .all(|(_, availability)| *availability >= floor)
            .then_some(samples[index].0)
    })
}

fn find_service_metric<'a>(
    report: &'a Value,
    scenario_id: &str,
    service_id: &str,
) -> &'a Value {
    report["scenario_metrics"]
        .as_array()
        .expect("scenario metrics must be an array")
        .iter()
        .find(|scenario| scenario["scenario_id"].as_str() == Some(scenario_id))
        .expect("scenario must exist")["service_metrics"]
        .as_array()
        .expect("service metrics must be an array")
        .iter()
        .find(|metric| metric["service_id"].as_str() == Some(service_id))
        .expect("service metric must exist")
}

/// Validate the NRES-007 fixture and compute disaggregated service-trajectory metrics.
///
/// A successful report means only that the public synthetic fixture is internally
/// consistent and that the published formulas were applied deterministically.
pub fn evaluate_resilience_trajectories(fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    forbidden_key_scan(fixture, "", &mut errors);

    if str_field(fixture, "benchmark_id")
        != Some("PRAXIS-NRES-007-CASCADE-RECOVERY-001")
    {
        errors.push("unexpected NRES-007 benchmark_id".to_owned());
    }
    if str_field(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-007 schema_version".to_owned());
    }
    if str_field(fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("NRES-007 fixture must remain explicitly synthetic".to_owned());
    }
    if str_field(fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("NRES-007 fixture must remain public development data".to_owned());
    }
    if fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("NRES-007 fixture must remain inactive".to_owned());
    }

    let Some(protocol) = fixture.get("protocol") else {
        return Err(vec!["NRES-007 protocol is missing".to_owned()]);
    };
    let horizon = num_field(protocol, "horizon_hours");
    let raw_hours = array_field(protocol, "sample_hours");
    let sample_hours: Vec<f64> = raw_hours
        .into_iter()
        .flatten()
        .filter_map(Value::as_f64)
        .collect();
    if horizon.is_none_or(|value| !value.is_finite() || value <= 0.0)
        || raw_hours.is_none()
        || sample_hours.len() < 2
        || raw_hours.is_some_and(|items| items.len() != sample_hours.len())
    {
        errors.push("protocol requires a positive horizon and at least two numeric sample hours".to_owned());
        return Err(errors);
    }
    let horizon = horizon.unwrap();
    if sample_hours.first().copied() != Some(0.0)
        || sample_hours.last().copied() != Some(horizon)
        || sample_hours.windows(2).any(|pair| pair[1] <= pair[0])
    {
        errors.push("sample hours must start at zero, increase strictly, and end at the horizon".to_owned());
    }
    if str_field(protocol, "deficit_area_unit") != Some("percentage_point_hours") {
        errors.push("deficit-area unit must be percentage_point_hours".to_owned());
    }

    let Some(shock) = fixture.get("shock_sequence") else {
        return Err(vec!["frozen shock_sequence is missing".to_owned()]);
    };
    let shock_id = str_field(shock, "shock_sequence_id");
    if shock_id.is_none() {
        errors.push("shock_sequence_id is required".to_owned());
    }
    let mut stressor_ids = BTreeSet::new();
    let stressors = array_field(shock, "events").cloned().unwrap_or_default();
    if stressors.is_empty() {
        errors.push("shock sequence must include at least one event".to_owned());
    }
    for event in &stressors {
        let event_id = str_field(event, "event_id");
        let start = num_field(event, "start_hour");
        let end = num_field(event, "end_hour");
        if event_id.is_none_or(|id| !stressor_ids.insert(id.to_owned()))
            || str_field(event, "event_type").is_none()
            || start.is_none_or(|value| value < 0.0 || value >= horizon)
            || end.is_none_or(|value| value <= start.unwrap_or(horizon) || value > horizon)
            || str_field(event, "severity").is_none()
        {
            errors.push("shock event has a duplicate/missing ID or invalid event interval".to_owned());
        }
    }

    let Some(services) = array_field(fixture, "services") else {
        return Err(vec!["service catalog is missing".to_owned()]);
    };
    let mut service_catalog: BTreeMap<String, (f64, bool)> = BTreeMap::new();
    for service in services {
        let id = str_field(service, "service_id");
        let floor = num_field(service, "floor_percent");
        let critical = service.get("critical").and_then(Value::as_bool);
        if id.is_none()
            || floor.is_none_or(|value| !value.is_finite() || !(0.0..=100.0).contains(&value))
            || critical.is_none()
            || str_field(service, "unit").is_none()
        {
            errors.push("service requires unique ID, 0-100 floor, critical flag and unit".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if service_catalog
            .insert(id.clone(), (floor.unwrap(), critical.unwrap()))
            .is_some()
        {
            errors.push(format!("duplicate service ID: {id}"));
        }
    }
    if service_catalog.is_empty()
        || !service_catalog.values().any(|(_, critical)| *critical)
    {
        errors.push("service catalog must include at least one critical service".to_owned());
    }

    // Validate prerequisite -> dependent edges and reject cycles. Acyclicity is a
    // profile constraint, not a claim that real infrastructure never has cycles.
    let mut adjacency: BTreeMap<String, Vec<String>> = service_catalog
        .keys()
        .map(|id| (id.clone(), Vec::new()))
        .collect();
    let mut indegree: BTreeMap<String, usize> = service_catalog
        .keys()
        .map(|id| (id.clone(), 0))
        .collect();
    let mut edge_ids = BTreeSet::new();
    for edge in array_field(fixture, "dependency_edges").cloned().unwrap_or_default() {
        let prerequisite = str_field(&edge, "prerequisite_service_id");
        let dependent = str_field(&edge, "dependent_service_id");
        let relationship = str_field(&edge, "relationship");
        match (prerequisite, dependent) {
            (Some(from), Some(to))
                if from != to
                    && service_catalog.contains_key(from)
                    && service_catalog.contains_key(to)
                    && relationship == Some("required-input") =>
            {
                let key = format!("{from}->{to}");
                if !edge_ids.insert(key) {
                    errors.push("duplicate dependency edge".to_owned());
                } else {
                    adjacency.entry(from.to_owned()).or_default().push(to.to_owned());
                    *indegree.entry(to.to_owned()).or_default() += 1;
                }
            }
            _ => errors.push("dependency edge has a dangling endpoint, self-edge or unsupported relationship".to_owned()),
        }
    }
    let mut ready: VecDeque<String> = indegree
        .iter()
        .filter_map(|(id, degree)| (*degree == 0).then_some(id.clone()))
        .collect();
    let mut visited = 0usize;
    while let Some(node) = ready.pop_front() {
        visited += 1;
        for dependent in adjacency.get(&node).into_iter().flatten() {
            if let Some(degree) = indegree.get_mut(dependent) {
                *degree -= 1;
                if *degree == 0 {
                    ready.push_back(dependent.clone());
                }
            }
        }
    }
    if visited != service_catalog.len() {
        errors.push("dependency graph contains a directed cycle".to_owned());
    }

    let Some(resource_limits) = fixture.get("resource_limits") else {
        return Err(vec!["resource_limits are missing".to_owned()]);
    };
    let planning_limit = num_field(resource_limits, "planning_points_available");
    let responder_limit = num_field(resource_limits, "responder_hours_available");
    if planning_limit.is_none_or(|value| !value.is_finite() || value < 0.0)
        || responder_limit.is_none_or(|value| !value.is_finite() || value < 0.0)
    {
        errors.push("resource limits must be finite, non-negative numbers".to_owned());
    }
    let planning_limit = planning_limit.unwrap_or(0.0);
    let responder_limit = responder_limit.unwrap_or(0.0);

    let Some(scenarios) = array_field(fixture, "scenarios") else {
        return Err(vec!["scenario list is missing".to_owned()]);
    };
    if scenarios.len() < 2 {
        errors.push("matched-shock evaluation requires at least two scenarios".to_owned());
    }
    let mut scenario_ids = BTreeSet::new();
    let mut reference_ids = Vec::new();
    let mut comparison_ids = Vec::new();
    for scenario in scenarios {
        let id = str_field(scenario, "scenario_id");
        let role = str_field(scenario, "scenario_role");
        if id.is_none_or(|value| !scenario_ids.insert(value.to_owned())) {
            errors.push("scenario IDs must be present and unique".to_owned());
        }
        match role {
            Some("reference") => reference_ids.push(id.unwrap_or("UNKNOWN").to_owned()),
            Some("comparison") => comparison_ids.push(id.unwrap_or("UNKNOWN").to_owned()),
            _ => errors.push("scenario_role must be reference or comparison".to_owned()),
        }
        if str_field(scenario, "shock_sequence_id") != shock_id {
            errors.push(format!(
                "{}: scenario does not use the frozen matched shock sequence",
                id.unwrap_or("UNKNOWN")
            ));
        }
    }
    if reference_ids.len() != 1 || comparison_ids.is_empty() {
        errors.push("exactly one reference scenario and at least one comparison scenario are required".to_owned());
    }

    let mut scenario_metrics = Vec::new();
    let mut deficit_by_scenario: BTreeMap<String, BTreeMap<String, f64>> = BTreeMap::new();
    for scenario in scenarios {
        let scenario_id = str_field(scenario, "scenario_id").unwrap_or("UNKNOWN").to_owned();
        let scenario_role = str_field(scenario, "scenario_role").unwrap_or("UNKNOWN");

        let resource_use = scenario.get("resource_use").cloned().unwrap_or(Value::Null);
        let planning_used = num_field(&resource_use, "planning_points_committed");
        let responder_used = num_field(&resource_use, "responder_hours_committed");
        if planning_used.is_none_or(|value| !value.is_finite() || value < 0.0 || value > planning_limit)
            || responder_used.is_none_or(|value| !value.is_finite() || value < 0.0 || value > responder_limit)
        {
            errors.push(format!("{scenario_id}: resource commitment is missing, negative or above its own limit"));
        }

        let Some(trajectories) = array_field(scenario, "trajectories") else {
            errors.push(format!("{scenario_id}: trajectories are missing"));
            continue;
        };
        let mut trajectory_by_service: BTreeMap<String, &Value> = BTreeMap::new();
        for trajectory in trajectories {
            let Some(service_id) = str_field(trajectory, "service_id") else {
                errors.push(format!("{scenario_id}: trajectory service_id is missing"));
                continue;
            };
            if !service_catalog.contains_key(service_id) {
                errors.push(format!("{scenario_id}: unknown service trajectory {service_id}"));
            }
            if trajectory_by_service.insert(service_id.to_owned(), trajectory).is_some() {
                errors.push(format!("{scenario_id}: duplicate service trajectory {service_id}"));
            }
        }
        if trajectory_by_service.len() != service_catalog.len()
            || service_catalog.keys().any(|id| !trajectory_by_service.contains_key(id))
        {
            errors.push(format!("{scenario_id}: every catalog service must have exactly one trajectory"));
        }

        let mut service_metrics = Vec::new();
        let mut deficit_map = BTreeMap::new();
        let mut all_critical_at_horizon = true;
        for (service_id, (floor, critical)) in &service_catalog {
            let Some(trajectory) = trajectory_by_service.get(service_id) else {
                all_critical_at_horizon = false;
                continue;
            };
            let Some(points) = array_field(trajectory, "points") else {
                errors.push(format!("{scenario_id}/{service_id}: points are missing"));
                all_critical_at_horizon = false;
                continue;
            };
            if points.len() != sample_hours.len() {
                errors.push(format!("{scenario_id}/{service_id}: sample count differs from frozen grid"));
                all_critical_at_horizon = false;
                continue;
            }
            let mut samples = Vec::new();
            let mut valid = true;
            for (index, point) in points.iter().enumerate() {
                let hour = num_field(point, "hour");
                let availability = num_field(point, "availability_percent");
                if hour != Some(sample_hours[index])
                    || availability.is_none_or(|value| !value.is_finite() || !(0.0..=100.0).contains(&value))
                {
                    errors.push(format!("{scenario_id}/{service_id}: point does not match grid or 0-100 availability bounds"));
                    valid = false;
                    break;
                }
                samples.push((hour.unwrap(), availability.unwrap()));
            }
            if !valid {
                all_critical_at_horizon = false;
                continue;
            }
            let area = deficit_area(*floor, &samples);
            let max_shortfall = samples
                .iter()
                .map(|(_, availability)| (*floor - *availability).max(0.0))
                .fold(0.0_f64, f64::max);
            let breach_count = samples
                .iter()
                .filter(|(_, availability)| *availability < *floor)
                .count();
            let recovery_hour = sustained_recovery_hour(*floor, &samples);
            let horizon_pass = samples.last().is_some_and(|(_, availability)| *availability >= *floor);
            if *critical && !horizon_pass {
                all_critical_at_horizon = false;
            }
            deficit_map.insert(service_id.clone(), area);
            service_metrics.push(json!({
                "service_id": service_id,
                "critical": critical,
                "floor_percent": floor,
                "max_shortfall_percentage_points": max_shortfall,
                "breach_sample_count": breach_count,
                "deficit_area": {
                    "value": area,
                    "unit": "percentage_point_hours"
                },
                "sustained_recovery_observed_hour": recovery_hour,
                "recovery_observed_within_horizon": recovery_hour.is_some(),
                "horizon_floor_pass": horizon_pass
            }));
        }
        deficit_by_scenario.insert(scenario_id.clone(), deficit_map);
        scenario_metrics.push(json!({
            "scenario_id": scenario_id,
            "scenario_role": scenario_role,
            "shock_sequence_id": shock_id,
            "resource_use": {
                "planning_points_committed": planning_used,
                "planning_points_available": planning_limit,
                "responder_hours_committed": responder_used,
                "responder_hours_available": responder_limit
            },
            "service_metrics": service_metrics,
            "all_critical_service_floors_pass_at_horizon": all_critical_at_horizon
        }));
    }

    let mut paired_contrasts = Vec::new();
    if let Some(reference_id) = reference_ids.first() {
        if let Some(reference_deficits) = deficit_by_scenario.get(reference_id) {
            for comparison_id in &comparison_ids {
                if let Some(comparison_deficits) = deficit_by_scenario.get(comparison_id) {
                    for service_id in service_catalog.keys() {
                        if let (Some(reference), Some(comparison)) = (
                            reference_deficits.get(service_id),
                            comparison_deficits.get(service_id),
                        ) {
                            paired_contrasts.push(json!({
                                "reference_scenario_id": reference_id,
                                "comparison_scenario_id": comparison_id,
                                "service_id": service_id,
                                "reference_deficit_area": reference,
                                "comparison_deficit_area": comparison,
                                "reference_minus_comparison_percentage_point_hours": reference - comparison,
                                "interpretation": "descriptive synthetic scenario contrast; not a causal effect"
                            }));
                        }
                    }
                }
            }
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-007-CASCADE-RECOVERY-001",
        "interpretation": "synthetic fixture consistency and deterministic metric derivation only; no real-world readiness or causal effect",
        "shock_sequence_id": shock_id,
        "sample_horizon_hours": horizon,
        "scenario_metrics": scenario_metrics,
        "paired_service_contrasts": paired_contrasts,
        "aggregation_policy": "none; service measures are disaggregated and non-compensatory"
    }))
}

#[cfg(test)]
mod tests {
    use super::evaluate_resilience_trajectories;
    use serde_json::{json, Value};

    const FIXTURE: &str = include_str!("../../../examples/courses/civic-resilience-cascade-recovery-fixture-001.json");

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("NRES-007 fixture must be valid JSON")
    }

    #[test]
    fn derives_disaggregated_recovery_metrics_and_exact_crossing_area() {
        let report = evaluate_resilience_trajectories(&fixture()).expect("fixture should validate");
        let reference = super::find_service_metric(&report, "NRES007-REFERENCE-001", "health_logistics_continuity");
        let comparison = super::find_service_metric(&report, "NRES007-COMPARISON-001", "health_logistics_continuity");
        assert_eq!(reference["sustained_recovery_observed_hour"].as_f64(), Some(72.0));
        assert_eq!(comparison["sustained_recovery_observed_hour"].as_f64(), Some(36.0));

        let reference_water = super::find_service_metric(&report, "NRES007-REFERENCE-001", "water_service_continuity");
        let comparison_water = super::find_service_metric(&report, "NRES007-COMPARISON-001", "water_service_continuity");
        let reference_area = reference_water.pointer("/deficit_area/value").and_then(Value::as_f64).unwrap();
        let comparison_area = comparison_water.pointer("/deficit_area/value").and_then(Value::as_f64).unwrap();
        assert!(reference_area > comparison_area);
        assert!((comparison_area - 12.0).abs() < 1e-9);
        assert!(report.get("overall_resilience_score").is_none());
        assert!(report.get("composite_score").is_none());
    }

    #[test]
    fn rejects_availability_above_one_hundred_percent() {
        let mut value = fixture();
        value["scenarios"][0]["trajectories"][0]["points"][1]["availability_percent"] = json!(101);
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_sample_time_grid_drift() {
        let mut value = fixture();
        value["scenarios"][0]["trajectories"][0]["points"][2]["hour"] = json!(5);
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_missing_service_trajectory() {
        let mut value = fixture();
        value["scenarios"][0]["trajectories"].as_array_mut().unwrap().pop();
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_unmatched_shock_sequence() {
        let mut value = fixture();
        value["scenarios"][1]["shock_sequence_id"] = json!("OTHER-SHOCK");
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_dangling_dependency_endpoint() {
        let mut value = fixture();
        value["dependency_edges"][0]["dependent_service_id"] = json!("unknown_service");
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_dependency_cycles() {
        let mut value = fixture();
        value["dependency_edges"].as_array_mut().unwrap().push(json!({
            "prerequisite_service_id": "health_logistics_continuity",
            "dependent_service_id": "electricity_supply",
            "relationship": "required-input"
        }));
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_composite_score_injection() {
        let mut value = fixture();
        value["overall_resilience_score"] = json!(99);
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_resource_pool_overrun() {
        let mut value = fixture();
        value["scenarios"][1]["resource_use"]["responder_hours_committed"] = json!(241);
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_service_trajectory() {
        let mut value = fixture();
        let duplicate = value["scenarios"][0]["trajectories"][0].clone();
        value["scenarios"][0]["trajectories"].as_array_mut().unwrap().push(duplicate);
        assert!(evaluate_resilience_trajectories(&value).is_err());
    }
}

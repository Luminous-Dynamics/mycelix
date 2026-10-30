//! Deterministic NRES-008 evaluator for binary service-floor breach forecasts.
//!
//! The evaluator separates candidate probability/abstention outputs from
//! evaluator-owned input qualification and outcome fields. It does not estimate
//! real outage probabilities, issue operational instructions, or certify readiness.

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

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

#[derive(Default)]
struct ReliabilityBin {
    count: usize,
    probability_sum: f64,
    event_count: usize,
}

#[derive(Default)]
struct ForecastAggregate {
    count: usize,
    event_count: usize,
    brier_sum: f64,
    log_loss_sum: f64,
}

#[derive(Default)]
struct ServiceCaseAggregate {
    total_cases: usize,
    qualified_cases: usize,
    issued_forecasts: usize,
    abstentions: usize,
    mandatory_abstention_cases: usize,
    mandatory_abstentions: usize,
}

fn probability_bin(probability: f64) -> usize {
    if probability >= 1.0 {
        4
    } else {
        ((probability * 5.0).floor() as usize).min(4)
    }
}

/// Validate NRES-008 candidate outputs against evaluator-owned fixture inputs
/// and derive selective proper scores, coverage and reliability diagnostics.
pub fn evaluate_probability_forecasts(fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    forbidden_key_scan(fixture, "", &mut errors);

    if str_field(fixture, "benchmark_id")
        != Some("PRAXIS-NRES-008-PROBABILITY-ABSTENTION-001")
    {
        errors.push("unexpected NRES-008 benchmark_id".to_owned());
    }
    if str_field(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-008 schema_version".to_owned());
    }
    if str_field(fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("NRES-008 fixture must remain explicitly synthetic".to_owned());
    }
    if str_field(fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("NRES-008 fixture must remain public development data".to_owned());
    }
    if fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("NRES-008 fixture must remain inactive".to_owned());
    }

    let Some(protocol) = fixture.get("protocol") else {
        return Err(vec!["NRES-008 protocol is missing".to_owned()]);
    };
    let forecast_origin = num_field(protocol, "forecast_origin_hour");
    let forecast_horizon = num_field(protocol, "forecast_horizon_hours");
    let epsilon = num_field(protocol, "log_loss_epsilon");
    if forecast_origin.is_none_or(|value| !value.is_finite() || value < 0.0)
        || forecast_horizon.is_none_or(|value| !value.is_finite() || value <= 0.0)
        || epsilon.is_none_or(|value| !value.is_finite() || value <= 0.0 || value > 0.01)
    {
        errors.push("protocol origin, positive horizon and log-loss epsilon must be valid".to_owned());
    }
    let forecast_origin = forecast_origin.unwrap_or(0.0);
    let forecast_horizon = forecast_horizon.unwrap_or(0.0);
    let epsilon = epsilon.unwrap_or(1e-6);

    let expected_edges = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
    let expected_confidence_thresholds = [0.0, 0.25, 0.5, 0.75];
    let confidence_thresholds: Vec<f64> = protocol
        .get("selective_confidence_thresholds")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(Value::as_f64)
        .collect();
    if confidence_thresholds.len() != expected_confidence_thresholds.len()
        || confidence_thresholds
            .iter()
            .zip(expected_confidence_thresholds.iter())
            .any(|(actual, expected)| (actual - expected).abs() > 1e-12)
    {
        errors.push("selective confidence thresholds must use the frozen [0,.25,.5,.75] grid".to_owned());
    }
    let edges: Vec<f64> = protocol
        .get("probability_bins")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(Value::as_f64)
        .collect();
    if edges.len() != expected_edges.len()
        || edges
            .iter()
            .zip(expected_edges.iter())
            .any(|(actual, expected)| (actual - expected).abs() > 1e-12)
    {
        errors.push("probability bins must use the frozen six edges [0,.2,.4,.6,.8,1]".to_owned());
    }

    let Some(service_catalog) = fixture.get("service_catalog").and_then(Value::as_array) else {
        return Err(vec!["service_catalog is missing".to_owned()]);
    };
    let mut service_floors = BTreeMap::new();
    for service in service_catalog {
        let id = str_field(service, "service_id");
        let floor = num_field(service, "floor_percent");
        if id.is_none()
            || floor.is_none_or(|value| !value.is_finite() || !(0.0..=100.0).contains(&value))
            || str_field(service, "unit").is_none_or(str::is_empty)
            || id.is_some_and(str::is_empty)
        {
            errors.push("service catalog entry needs ID, 0-100 floor and unit".to_owned());
            continue;
        }
        let id = id.unwrap().to_owned();
        if service_floors.insert(id.clone(), floor.unwrap()).is_some() {
            errors.push(format!("duplicate service ID: {id}"));
        }
    }
    if service_floors.is_empty() {
        errors.push("service catalog must not be empty".to_owned());
    }

    let Some(reference) = fixture.get("calibration_reference") else {
        return Err(vec!["calibration_reference is missing".to_owned()]);
    };
    let reference_count = reference.get("sample_count").and_then(Value::as_u64);
    let reference_breaches = reference.get("breach_count").and_then(Value::as_u64);
    if reference_count.is_none_or(|count| count == 0)
        || reference_breaches.is_none_or(|count| count > reference_count.unwrap_or(0))
        || str_field(reference, "status") != Some("synthetic-public-development-reference")
    {
        errors.push("calibration reference must have positive n, valid breach count and synthetic status".to_owned());
    }
    let reference_rate = reference_breaches
        .zip(reference_count)
        .filter(|(_, count)| *count > 0)
        .map(|(breaches, count)| breaches as f64 / count as f64)
        .unwrap_or(0.5);

    let Some(cases) = fixture.get("evaluation_cases").and_then(Value::as_array) else {
        return Err(vec!["evaluation_cases is missing".to_owned()]);
    };
    if cases.is_empty() {
        errors.push("evaluation_cases must not be empty".to_owned());
    }

    let mut case_ids = BTreeSet::new();
    let mut service_case_counts: BTreeMap<String, ServiceCaseAggregate> = service_floors
        .keys()
        .map(|service_id| (service_id.clone(), ServiceCaseAggregate::default()))
        .collect();
    let mut mandatory_abstention_by_reason: BTreeMap<String, usize> = [
        "missing_critical_input",
        "conflicting_sources",
        "unsupported_distribution_shift",
    ]
    .into_iter()
    .map(|reason| (reason.to_owned(), 0))
    .collect();
    let mut issued = Vec::new();
    let mut total_count = 0usize;
    let mut qualified_count = 0usize;
    let mut mandatory_count = 0usize;
    let mut mandatory_abstentions = 0usize;
    let mut optional_abstentions = 0usize;
    let mut abstention_count = 0usize;

    for case in cases {
        total_count += 1;
        let case_id = str_field(case, "case_id").unwrap_or("UNKNOWN").to_owned();
        if str_field(case, "case_id").is_none() || !case_ids.insert(case_id.clone()) {
            errors.push(format!("{case_id}: case IDs must be present and unique"));
        }
        let service_id = str_field(case, "service_id");
        let Some(floor) = service_id.and_then(|id| service_floors.get(id)).copied() else {
            errors.push(format!("{case_id}: unknown service_id"));
            continue;
        };
        let service_id_owned = service_id.unwrap().to_owned();
        let service_case_count = service_case_counts
            .get_mut(&service_id_owned)
            .expect("service case counter must follow validated service catalog");
        service_case_count.total_cases += 1;
        if num_field(case, "forecast_origin_hour") != Some(forecast_origin)
            || num_field(case, "forecast_horizon_hours") != Some(forecast_horizon)
        {
            errors.push(format!("{case_id}: forecast origin or horizon differs from frozen protocol"));
        }

        let Some(evaluator) = case.get("evaluator") else {
            errors.push(format!("{case_id}: evaluator-owned record is missing"));
            continue;
        };
        let Some(candidate) = case.get("candidate_output") else {
            errors.push(format!("{case_id}: candidate_output is missing"));
            continue;
        };
        let qualification = str_field(evaluator, "input_qualification_state");
        let observed = num_field(evaluator, "observed_minimum_availability_percent");
        let Some(observed) = observed.filter(|value| value.is_finite() && (0.0..=100.0).contains(value)) else {
            errors.push(format!("{case_id}: observed minimum availability must be within 0-100"));
            continue;
        };
        let action = str_field(candidate, "action");
        let probability_value = candidate.get("predicted_breach_probability");
        let probability = probability_value.and_then(Value::as_f64);
        let reason_value = candidate.get("abstention_reason");
        let reason = reason_value.and_then(Value::as_str);
        let observed_breach = observed < floor;

        match qualification {
            Some("qualified") => {
                qualified_count += 1;
                service_case_count.qualified_cases += 1;
                match action {
                    Some("forecast") => {
                        if probability.is_none_or(|value| !value.is_finite() || !(0.0..=1.0).contains(&value)) {
                            errors.push(format!("{case_id}: forecast action requires probability in [0,1]"));
                        }
                        if !probability_value.is_some_and(Value::is_number) {
                            errors.push(format!("{case_id}: forecast probability is missing or null"));
                        }
                        if reason.is_some() {
                            errors.push(format!("{case_id}: forecast action must not include abstention_reason"));
                        }
                        if let Some(probability) = probability.filter(|value| value.is_finite() && (0.0..=1.0).contains(value)) {
                            issued.push((service_id.unwrap().to_owned(), probability, observed_breach));
                        }
                    }
                    Some("abstain") => {
                        abstention_count += 1;
                        optional_abstentions += 1;
                        service_case_count.abstentions += 1;
                        if !probability_value.is_some_and(Value::is_null) {
                            errors.push(format!("{case_id}: abstention must carry a null probability"));
                        }
                        if reason != Some("model_uncertainty") {
                            errors.push(format!("{case_id}: qualified abstention requires model_uncertainty reason"));
                        }
                    }
                    _ => errors.push(format!("{case_id}: candidate action must be forecast or abstain")),
                }
            }
            Some("missing_critical_input")
            | Some("conflicting_sources")
            | Some("unsupported_distribution_shift") => {
                mandatory_count += 1;
                service_case_count.mandatory_abstention_cases += 1;
                match action {
                    Some("abstain") => {
                        abstention_count += 1;
                        mandatory_abstentions += 1;
                        service_case_count.abstentions += 1;
                        service_case_count.mandatory_abstentions += 1;
                        if let Some(count) = qualification.and_then(|reason| mandatory_abstention_by_reason.get_mut(reason)) {
                            *count += 1;
                        }
                        if !probability_value.is_some_and(Value::is_null) {
                            errors.push(format!("{case_id}: mandatory abstention must carry a null probability"));
                        }
                        if reason != qualification {
                            errors.push(format!("{case_id}: abstention reason must match evaluator qualification state"));
                        }
                    }
                    Some("forecast") => errors.push(format!("{case_id}: mandatory abstention policy violated")),
                    _ => errors.push(format!("{case_id}: candidate action must be forecast or abstain")),
                }
            }
            _ => errors.push(format!("{case_id}: unsupported evaluator input qualification state")),
        }
    }

    if issued.is_empty() {
        errors.push("at least one forecast must be issued; all-abstention is not an acceptable result".to_owned());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let issued_count = issued.len();
    let event_count = issued.iter().filter(|(_, _, event)| *event).count();
    let mut brier_sum = 0.0;
    let mut log_loss_sum = 0.0;
    let mut reference_brier_sum = 0.0;
    let mut bins: Vec<ReliabilityBin> = (0..5).map(|_| ReliabilityBin::default()).collect();
    let mut per_service: BTreeMap<String, ForecastAggregate> = service_floors
        .keys()
        .map(|service_id| (service_id.clone(), ForecastAggregate::default()))
        .collect();

    for (service_id, probability, event) in &issued {
        let probability = *probability;
        let outcome = if *event { 1.0 } else { 0.0 };
        let squared_error = (probability - outcome).powi(2);
        let clipped = probability.clamp(epsilon, 1.0 - epsilon);
        let case_log_loss = -(outcome * clipped.ln() + (1.0 - outcome) * (1.0 - clipped).ln());
        brier_sum += squared_error;
        log_loss_sum += case_log_loss;
        reference_brier_sum += (reference_rate - outcome).powi(2);

        let bin = &mut bins[probability_bin(probability)];
        bin.count += 1;
        bin.probability_sum += probability;
        if *event {
            bin.event_count += 1;
        }
        if let Some(service_metrics) = per_service.get_mut(service_id) {
            service_metrics.count += 1;
            service_metrics.brier_sum += squared_error;
            service_metrics.log_loss_sum += case_log_loss;
            if *event {
                service_metrics.event_count += 1;
            }
        }
    }

    let brier_score = brier_sum / issued_count as f64;
    let log_loss = log_loss_sum / issued_count as f64;
    let reference_brier = reference_brier_sum / issued_count as f64;
    let brier_skill = if reference_brier > 0.0 {
        Some(1.0 - brier_score / reference_brier)
    } else {
        None
    };

    let reliability_bins: Vec<Value> = bins
        .iter()
        .enumerate()
        .map(|(index, bin)| {
            let lower = index as f64 * 0.2;
            let upper = (index + 1) as f64 * 0.2;
            let mean_probability = if bin.count == 0 {
                None
            } else {
                Some(bin.probability_sum / bin.count as f64)
            };
            let event_rate = if bin.count == 0 {
                None
            } else {
                Some(bin.event_count as f64 / bin.count as f64)
            };
            json!({
                "bin_id": format!("BIN-{index}"),
                "lower_inclusive": lower,
                "upper": upper,
                "upper_inclusive": index == 4,
                "count": bin.count,
                "mean_forecast_probability": mean_probability,
                "observed_event_rate": event_rate
            })
        })
        .collect();

    let ece = bins
        .iter()
        .filter(|bin| bin.count > 0)
        .map(|bin| {
            let forecast_mean = bin.probability_sum / bin.count as f64;
            let observed_rate = bin.event_count as f64 / bin.count as f64;
            bin.count as f64 / issued_count as f64 * (forecast_mean - observed_rate).abs()
        })
        .sum::<f64>();

    let per_service_case_metrics: Vec<Value> = service_case_counts
        .iter()
        .map(|(service_id, metrics)| {
            json!({
                "service_id": service_id,
                "total_cases": metrics.total_cases,
                "qualified_cases": metrics.qualified_cases,
                "issued_forecasts": metrics.issued_forecasts,
                "abstentions": metrics.abstentions,
                "mandatory_abstention_cases": metrics.mandatory_abstention_cases,
                "mandatory_abstentions": metrics.mandatory_abstentions
            })
        })
        .collect();

    let per_service_metrics: Vec<Value> = per_service
        .iter()
        .map(|(service_id, metrics)| {
            json!({
                "service_id": service_id,
                "issued_forecasts": metrics.count,
                "breach_events": metrics.event_count,
                "brier_score": (metrics.count > 0).then_some(metrics.brier_sum / metrics.count as f64),
                "log_loss": (metrics.count > 0).then_some(metrics.log_loss_sum / metrics.count as f64)
            })
        })
        .collect();

    let risk_coverage_curve: Vec<Value> = expected_confidence_thresholds
        .iter()
        .map(|threshold| {
            let selected: Vec<(f64, bool)> = issued
                .iter()
                .filter_map(|(_, probability, event)| {
                    ((2.0 * *probability - 1.0).abs() >= *threshold)
                        .then_some((*probability, *event))
                })
                .collect();
            let selected_brier = if selected.is_empty() {
                None
            } else {
                Some(
                    selected
                        .iter()
                        .map(|(probability, event)| {
                            let probability = *probability;
                            let outcome = if *event { 1.0 } else { 0.0 };
                            (probability - outcome).powi(2)
                        })
                        .sum::<f64>()
                        / selected.len() as f64,
                )
            };
            json!({
                "minimum_probability_confidence": threshold,
                "selected_forecasts": selected.len(),
                "population_coverage": selected.len() as f64 / total_count as f64,
                "qualified_case_coverage": (qualified_count > 0).then_some(selected.len() as f64 / qualified_count as f64),
                "selective_brier_score": selected_brier
            })
        })
        .collect();

    let population_coverage = issued_count as f64 / total_count as f64;
    let qualified_coverage = if qualified_count > 0 {
        Some(issued_count as f64 / qualified_count as f64)
    } else {
        None
    };
    let mandatory_compliance = if mandatory_count > 0 {
        Some(mandatory_abstentions as f64 / mandatory_count as f64)
    } else {
        None
    };

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-008-PROBABILITY-ABSTENTION-001",
        "interpretation": "synthetic fixture consistency and selective forecast scoring only; no real-world probability calibration or operational validity",
        "counts": {
            "total_cases": total_count,
            "qualified_cases": qualified_count,
            "issued_forecasts": issued_count,
            "abstentions": abstention_count,
            "mandatory_abstention_cases": mandatory_count,
            "mandatory_abstentions": mandatory_abstentions,
            "optional_abstentions": optional_abstentions,
            "issued_breach_events": event_count
        },
        "coverage": {
            "population_forecast_coverage": population_coverage,
            "qualified_case_forecast_coverage": qualified_coverage,
            "abstention_rate": abstention_count as f64 / total_count as f64
        },
        "abstention_policy": {
            "mandatory_abstention_compliance": mandatory_compliance,
            "mandatory_abstention_violations": 0
        },
        "proper_scores": {
            "brier_score": brier_score,
            "log_loss": log_loss,
            "climatology_reference_probability": reference_rate,
            "climatology_reference_sample_count": reference_count,
            "climatology_reference_breach_count": reference_breaches,
            "climatology_brier_score": reference_brier,
            "brier_skill_score": brier_skill
        },
        "calibration_diagnostics": {
            "fixed_bin_expected_calibration_error": ece,
            "interpretation": "bin-dependent diagnostic only; not a proper score or standalone ranking metric",
            "reliability_bins": reliability_bins
        },
        "mandatory_abstention_by_reason": mandatory_abstention_by_reason,
        "per_service_case_metrics": per_service_case_metrics,
        "per_service_metrics": per_service_metrics,
        "risk_coverage_curve": risk_coverage_curve,
        "aggregation_policy": "no composite resilience or personal score; selective proper scores always accompanied by coverage and abstention counts"
    }))
}

#[cfg(test)]
mod tests {
    use super::evaluate_probability_forecasts;
    use serde_json::{json, Value};

    const FIXTURE: &str = include_str!("../../../examples/courses/civic-resilience-probability-abstention-fixture-001.json");
    const CONFORMANCE: &str = include_str!("../../../examples/courses/civic-resilience-probability-abstention-conformance-001.json");

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("NRES-008 fixture must be valid JSON")
    }

    fn find_forecast(value: &mut Value) -> &mut Value {
        value["evaluation_cases"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|case| case["candidate_output"]["action"] == "forecast")
            .unwrap()
    }

    #[test]
    fn derives_selective_scores_and_abstention_coverage() {
        let report = evaluate_probability_forecasts(&fixture()).expect("fixture should validate");
        assert_eq!(report.pointer("/counts/total_cases").and_then(Value::as_u64), Some(25));
        assert_eq!(report.pointer("/counts/issued_forecasts").and_then(Value::as_u64), Some(20));
        assert_eq!(report.pointer("/counts/mandatory_abstentions").and_then(Value::as_u64), Some(4));
        assert_eq!(report.pointer("/counts/optional_abstentions").and_then(Value::as_u64), Some(1));
        assert!((report.pointer("/proper_scores/brier_score").and_then(Value::as_f64).unwrap() - 0.1604).abs() < 1e-12);
        assert!(report.get("overall_resilience_score").is_none());
    }

    #[test]
    fn matches_all_published_golden_scores_and_reliability_bins() {
        let report = evaluate_probability_forecasts(&fixture()).expect("fixture should validate");
        let corpus: Value = serde_json::from_str(CONFORMANCE).expect("conformance corpus must be valid JSON");
        let expected = &corpus["expected_metrics"];
        let tolerance = corpus["conformance_targets"]["expected_metric_tolerance_absolute"]
            .as_f64()
            .unwrap_or(1e-12);
        for key in [
            "population_forecast_coverage",
            "qualified_case_forecast_coverage",
            "mandatory_abstention_compliance",
            "brier_score",
            "log_loss",
            "climatology_reference_probability",
            "climatology_reference_sample_count",
            "climatology_reference_breach_count",
            "climatology_brier_score",
            "brier_skill_score",
            "expected_calibration_error_diagnostic",
        ] {
            let actual_path = match key {
                "population_forecast_coverage" => "/coverage/population_forecast_coverage",
                "qualified_case_forecast_coverage" => "/coverage/qualified_case_forecast_coverage",
                "mandatory_abstention_compliance" => "/abstention_policy/mandatory_abstention_compliance",
                "brier_score" => "/proper_scores/brier_score",
                "log_loss" => "/proper_scores/log_loss",
                "climatology_reference_probability" => "/proper_scores/climatology_reference_probability",
                "climatology_brier_score" => "/proper_scores/climatology_brier_score",
                "brier_skill_score" => "/proper_scores/brier_skill_score",
                _ => "/calibration_diagnostics/fixed_bin_expected_calibration_error",
            };
            let actual = report.pointer(actual_path).and_then(Value::as_f64).unwrap();
            let expected = expected[key].as_f64().unwrap();
            assert!((actual - expected).abs() <= tolerance, "{key} differs");
        }
        assert_eq!(
            report.pointer("/calibration_diagnostics/reliability_bins").and_then(Value::as_array).unwrap().len(),
            expected["reliability_bins"].as_array().unwrap().len()
        );
        for (actual, expected) in report["calibration_diagnostics"]["reliability_bins"]
            .as_array().unwrap().iter().zip(expected["reliability_bins"].as_array().unwrap())
        {
            for key in ["count", "upper_inclusive"] {
                assert_eq!(actual[key], expected[key]);
            }
            for key in ["mean_forecast_probability", "observed_event_rate"] {
                let a = actual[key].as_f64();
                let e = expected[key].as_f64();
                assert_eq!(a.is_some(), e.is_some());
                if let (Some(a), Some(e)) = (a, e) {
                    assert!((a - e).abs() <= tolerance);
                }
            }
        }
        for (service_id, expected_service) in expected["per_service_metrics"].as_object().unwrap() {
            let actual_service = report["per_service_metrics"].as_array().unwrap()
                .iter().find(|item| item["service_id"].as_str() == Some(service_id.as_str())).unwrap();
            for key in ["issued_forecasts", "breach_events"] {
                assert_eq!(actual_service[key], expected_service[key]);
            }
            for key in ["brier_score", "log_loss"] {
                assert!((actual_service[key].as_f64().unwrap() - expected_service[key].as_f64().unwrap()).abs() <= tolerance);
            }
        }
        assert_eq!(report["mandatory_abstention_by_reason"], expected["mandatory_abstention_by_reason"]);
        for (service_id, expected_counts) in expected["per_service_case_metrics"].as_object().unwrap() {
            let actual_counts = report["per_service_case_metrics"].as_array().unwrap()
                .iter().find(|item| item["service_id"].as_str() == Some(service_id.as_str())).unwrap();
            for key in ["total_cases", "qualified_cases", "issued_forecasts", "abstentions", "mandatory_abstention_cases", "mandatory_abstentions"] {
                assert_eq!(actual_counts[key], expected_counts[key]);
            }
        }
        let actual_curve = report["risk_coverage_curve"].as_array().unwrap();
        let expected_curve = expected["risk_coverage_curve"].as_array().unwrap();
        assert_eq!(actual_curve.len(), expected_curve.len());
        for (actual, expected) in actual_curve.iter().zip(expected_curve) {
            for key in ["minimum_probability_confidence", "population_coverage", "qualified_case_coverage", "selective_brier_score"] {
                assert!((actual[key].as_f64().unwrap() - expected[key].as_f64().unwrap()).abs() <= tolerance);
            }
            assert_eq!(actual["selected_forecasts"], expected["selected_forecasts"]);
        }
    }

    #[test]
    fn rejects_probability_above_one() {
        let mut value = fixture();
        find_forecast(&mut value)["candidate_output"]["predicted_breach_probability"] = json!(1.01);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_probability_below_zero() {
        let mut value = fixture();
        find_forecast(&mut value)["candidate_output"]["predicted_breach_probability"] = json!(-0.01);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_forecast_on_missing_critical_input() {
        let mut value = fixture();
        let case = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["evaluator"]["input_qualification_state"] == "qualified").unwrap();
        case["evaluator"]["input_qualification_state"] = json!("missing_critical_input");
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_forecast_under_unsupported_distribution_shift() {
        let mut value = fixture();
        let case = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["evaluator"]["input_qualification_state"] == "qualified").unwrap();
        case["evaluator"]["input_qualification_state"] = json!("unsupported_distribution_shift");
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_probability_emitted_on_abstention() {
        let mut value = fixture();
        let case = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["candidate_output"]["action"] == "abstain").unwrap();
        case["candidate_output"]["predicted_breach_probability"] = json!(0.4);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_forecast_without_probability() {
        let mut value = fixture();
        find_forecast(&mut value)["candidate_output"]["predicted_breach_probability"] = Value::Null;
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_duplicate_case_id() {
        let mut value = fixture();
        let duplicate = value["evaluation_cases"][0]["case_id"].clone();
        value["evaluation_cases"][1]["case_id"] = duplicate;
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_forecast_horizon_drift() {
        let mut value = fixture();
        value["evaluation_cases"][0]["forecast_horizon_hours"] = json!(48);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_out_of_range_evaluator_outcome() {
        let mut value = fixture();
        value["evaluation_cases"][0]["evaluator"]["observed_minimum_availability_percent"] = json!(101);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_abstention_without_reason() {
        let mut value = fixture();
        let case = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["candidate_output"]["action"] == "abstain").unwrap();
        case["candidate_output"]["abstention_reason"] = Value::Null;
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_mismatched_mandatory_abstention_reason() {
        let mut value = fixture();
        let case = value["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| case["evaluator"]["input_qualification_state"] == "conflicting_sources").unwrap();
        case["candidate_output"]["abstention_reason"] = json!("model_uncertainty");
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_composite_score_injection() {
        let mut value = fixture();
        value["overall_resilience_score"] = json!(0.99);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_zero_calibration_reference_denominator() {
        let mut value = fixture();
        value["calibration_reference"]["sample_count"] = json!(0);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_unknown_service_id() {
        let mut value = fixture();
        value["evaluation_cases"][0]["service_id"] = json!("invented_service");
        assert!(evaluate_probability_forecasts(&value).is_err());
    }


    #[test]
    fn rejects_numeric_abstention_reason_on_forecast() {
        let mut value = fixture();
        find_forecast(&mut value)["candidate_output"]["abstention_reason"] = json!(17);
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_missing_explicit_null_reason_on_forecast() {
        let mut value = fixture();
        find_forecast(&mut value)["candidate_output"].as_object_mut().unwrap().remove("abstention_reason");
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_empty_service_unit() {
        let mut value = fixture();
        value["service_catalog"][0]["unit"] = json!("");
        assert!(evaluate_probability_forecasts(&value).is_err());
    }

    #[test]
    fn rejects_all_abstention_policy() {
        let mut value = fixture();
        for case in value["evaluation_cases"].as_array_mut().unwrap() {
            if case["candidate_output"]["action"] == "forecast" {
                case["candidate_output"]["action"] = json!("abstain");
                case["candidate_output"]["predicted_breach_probability"] = Value::Null;
                case["candidate_output"]["abstention_reason"] = json!("model_uncertainty");
            }
        }
        assert!(evaluate_probability_forecasts(&value).is_err());
    }
}

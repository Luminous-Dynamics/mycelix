use serde_json::{json, Value};
use std::collections::BTreeSet;

const STRESS_FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json");
const EXPECTED_FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-pareto-stress-expected-v0.1.json");
const EPS: f64 = 1e-9;

const MAXIMIZE_METRICS: [&str; 3] = [
    "service_units_lower_bound",
    "underserved_service_units_lower_bound",
    "reliability_lower_bound",
];
const MINIMIZE_METRICS: [&str; 2] = [
    "emissions_tonnes_co2e_upper_bound",
    "total_cost",
];
const VIOLATION_FIELDS: [&str; 8] = [
    "primary_budget_excess",
    "contingency_reserve_excess",
    "total_cost_excess",
    "service_floor_shortfall",
    "underserved_service_floor_shortfall",
    "reliability_floor_shortfall",
    "emissions_cap_excess",
    "cost_below_base_shortfall",
];

fn stress_fixture() -> Value {
    serde_json::from_str(STRESS_FIXTURE).expect("stress fixture parses")
}

fn expected_fixture() -> Value {
    serde_json::from_str(EXPECTED_FIXTURE).expect("expected-answer fixture parses")
}

fn num(value: &Value) -> f64 {
    value.as_f64().expect("numeric field")
}

fn strategy_id(strategy: &Value) -> &str {
    strategy["strategy_id"].as_str().expect("strategy id")
}

fn scenario_id(scenario: &Value) -> &str {
    scenario["scenario_id"].as_str().expect("scenario id")
}

fn rows_for_scenario<'a>(
    stress: &'a Value,
    scenario: &str,
) -> Result<Vec<(&'a Value, &'a Value)>, String> {
    let mut candidates = Vec::new();
    let strategies = stress["strategies"]
        .as_array()
        .ok_or_else(|| "strategies must be an array".to_owned())?;
    let declared_scenarios = stress["scenario_manifest"]
        .as_array()
        .ok_or_else(|| "scenario_manifest must be an array".to_owned())?;

    if declared_scenarios.iter().filter(|s| s["scenario_id"].as_str() == Some(scenario)).count() != 1 {
        return Err(format!("scenario {scenario} must be declared exactly once"));
    }

    for strategy in strategies {
        let rows = strategy["scenario_results"]
            .as_array()
            .ok_or_else(|| format!("{} has no scenario_results array", strategy_id(strategy)))?;
        let matches: Vec<&Value> = rows
            .iter()
            .filter(|row| row["scenario_id"].as_str() == Some(scenario))
            .collect();
        if matches.len() != 1 {
            return Err(format!(
                "{} must have exactly one outcome for scenario {scenario}",
                strategy_id(strategy)
            ));
        }
        candidates.push((strategy, matches[0]));
    }
    Ok(candidates)
}

fn validate_matrix(stress: &Value) -> Result<(), String> {
    let scenarios = stress["scenario_manifest"]
        .as_array()
        .ok_or_else(|| "scenario_manifest must be an array".to_owned())?;
    let strategies = stress["strategies"]
        .as_array()
        .ok_or_else(|| "strategies must be an array".to_owned())?;

    let scenario_ids: Vec<&str> = scenarios.iter().map(scenario_id).collect();
    let scenario_unique: BTreeSet<&str> = scenario_ids.iter().copied().collect();
    if scenario_unique.len() != scenario_ids.len() {
        return Err("duplicate scenario IDs".to_owned());
    }
    let strategy_ids: Vec<&str> = strategies.iter().map(strategy_id).collect();
    let strategy_unique: BTreeSet<&str> = strategy_ids.iter().copied().collect();
    if strategy_unique.len() != strategy_ids.len() {
        return Err("duplicate strategy IDs".to_owned());
    }

    let mut total_cells = 0usize;
    for strategy in strategies {
        let rows = strategy["scenario_results"]
            .as_array()
            .ok_or_else(|| "scenario_results must be an array".to_owned())?;
        let row_ids: Vec<&str> = rows
            .iter()
            .map(|row| row["scenario_id"].as_str().expect("scenario id"))
            .collect();
        let row_unique: BTreeSet<&str> = row_ids.iter().copied().collect();
        if row_unique.len() != row_ids.len() {
            return Err(format!("duplicate outcome cell for {}", strategy_id(strategy)));
        }
        if row_unique != scenario_unique {
            return Err(format!("scenario matrix incomplete for {}", strategy_id(strategy)));
        }
        total_cells += rows.len();
    }
    if total_cells != scenarios.len() * strategies.len() {
        return Err("matrix cell count differs from declared cross-product".to_owned());
    }
    Ok(())
}

fn dominates(a: &Value, b: &Value) -> bool {
    let mut strictly_better = false;
    for metric in MAXIMIZE_METRICS {
        let av = num(&a[metric]);
        let bv = num(&b[metric]);
        if av < bv - EPS {
            return false;
        }
        if av > bv + EPS {
            strictly_better = true;
        }
    }
    for metric in MINIMIZE_METRICS {
        let av = num(&a[metric]);
        let bv = num(&b[metric]);
        if av > bv + EPS {
            return false;
        }
        if av < bv - EPS {
            strictly_better = true;
        }
    }
    strictly_better
}

fn violations(stress: &Value, strategy: &Value, row: &Value) -> Value {
    let p = &stress["scoring_policy"];
    let base = num(&strategy["base_cost"]);
    let total = num(&row["total_cost"]);
    json!({
        "primary_budget_excess": (base - num(&p["primary_budget"])).max(0.0),
        "contingency_reserve_excess": (total - base - num(&p["contingency_reserve_cap"])).max(0.0),
        "total_cost_excess": (total - num(&p["total_cost_upper_bound"])).max(0.0),
        "service_floor_shortfall": (num(&p["min_service_units_lower_bound"]) - num(&row["service_units_lower_bound"])).max(0.0),
        "underserved_service_floor_shortfall": (num(&p["min_underserved_service_units_lower_bound"]) - num(&row["underserved_service_units_lower_bound"])).max(0.0),
        "reliability_floor_shortfall": (num(&p["min_reliability_lower_bound"]) - num(&row["reliability_lower_bound"])).max(0.0),
        "emissions_cap_excess": (num(&row["emissions_tonnes_co2e_upper_bound"]) - num(&p["max_emissions_tonnes_co2e_upper_bound"])).max(0.0),
        "cost_below_base_shortfall": (base - total).max(0.0),
    })
}

fn is_feasible(violation_vector: &Value) -> bool {
    VIOLATION_FIELDS
        .iter()
        .all(|field| num(&violation_vector[*field]) <= EPS)
}

fn sorted_ids(ids: impl IntoIterator<Item = String>) -> Vec<String> {
    let mut ids: Vec<String> = ids.into_iter().collect();
    ids.sort();
    ids
}

fn as_string_vec(value: &Value) -> Vec<String> {
    let mut result: Vec<String> = value
        .as_array()
        .expect("array of identifiers")
        .iter()
        .map(|item| item.as_str().expect("string identifier").to_owned())
        .collect();
    result.sort();
    result
}

fn pareto_front(candidates: &[(&Value, &Value)]) -> Vec<String> {
    sorted_ids(candidates.iter().filter_map(|(strategy, row)| {
        let dominated = candidates.iter().any(|(other_strategy, other_row)| {
            strategy_id(strategy) != strategy_id(other_strategy) && dominates(other_row, row)
        });
        (!dominated).then(|| strategy_id(strategy).to_owned())
    }))
}

fn compare_number(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1e-7,
        "actual {actual} differs from expected {expected}"
    );
}

fn compare_vectors(actual: &Value, expected: &Value) {
    for field in VIOLATION_FIELDS {
        compare_number(num(&actual[field]), num(&expected[field]));
    }
}

fn assert_objectives_are_explicit_and_unweighted(contract: &Value) {
    assert_eq!(contract["constraint_contract"]["all_constraints_non_compensable"], true);
    assert_eq!(contract["objective_contract"]["aggregation"], "none");
    assert_eq!(contract["objective_contract"]["weights_used"], false);
    assert_eq!(contract["objective_contract"]["metrics"].as_array().unwrap().len(), 5);

    let mut ids = BTreeSet::new();
    for metric in contract["objective_contract"]["metrics"].as_array().unwrap() {
        let id = metric["metric_id"].as_str().expect("metric id");
        let direction = metric["direction"].as_str().expect("objective direction");
        let unit = metric["unit"].as_str().expect("objective unit");
        assert!(ids.insert(id), "duplicate objective metric {id}");
        assert!(matches!(direction, "maximize" | "minimize"));
        assert!(!unit.is_empty());
    }
    assert!(contract.get("weights").is_none());
    assert!(contract.get("weighted_score").is_none());
}

#[test]
fn frozen_fronts_and_every_constraint_violation_vector_match_known_answers() {
    let stress = stress_fixture();
    let expected = expected_fixture();
    validate_matrix(&stress).expect("stress matrix complete");
    assert_objectives_are_explicit_and_unweighted(&expected);

    let summaries = expected["expected_summaries"].as_array().unwrap();
    assert_eq!(summaries.len(), stress["scenario_manifest"].as_array().unwrap().len());

    for scenario in stress["scenario_manifest"].as_array().unwrap() {
        let sid = scenario_id(scenario);
        let candidates = rows_for_scenario(&stress, sid).expect("outcome for every strategy");
        let all_front = pareto_front(&candidates);
        let feasible_candidates: Vec<(&Value, &Value)> = candidates
            .iter()
            .copied()
            .filter(|(strategy, row)| is_feasible(&violations(&stress, strategy, row)))
            .collect();
        let feasible_ids = sorted_ids(feasible_candidates
            .iter().map(|(strategy, _)| strategy_id(strategy).to_owned()));
        let feasible_front = pareto_front(&feasible_candidates);
        let dominated_ids = sorted_ids(candidates
            .iter()
            .filter(|(strategy, _)| !all_front.contains(&strategy_id(strategy).to_owned()))
            .map(|(strategy, _)| strategy_id(strategy).to_owned()));

        let frozen = summaries
            .iter()
            .find(|item| item["scenario_id"].as_str() == Some(sid))
            .expect("known answer for each scenario");
        assert_eq!(all_front, as_string_vec(&frozen["pareto_front_all_strategies"]));
        assert_eq!(dominated_ids, as_string_vec(&frozen["dominated_strategy_ids"]));
        assert_eq!(feasible_ids, as_string_vec(&frozen["feasible_strategy_ids"]));
        assert_eq!(feasible_front, as_string_vec(&frozen["feasible_pareto_front_strategy_ids"]));
        assert_eq!(feasible_front.is_empty(), frozen["feasible_front_empty"].as_bool().unwrap());
        assert_eq!(feasible_ids.len(), frozen["feasible_count"].as_u64().unwrap() as usize);

        let frozen_violations = frozen["constraint_violation_vectors"].as_array().unwrap();
        assert_eq!(frozen_violations.len(), candidates.len());
        for (strategy, row) in candidates {
            let actual = violations(&stress, strategy, row);
            let known = frozen_violations
                .iter()
                .find(|item| item["strategy_id"].as_str() == Some(strategy_id(strategy)))
                .expect("constraint vector for each strategy");
            assert_eq!(known["scenario_id"].as_str(), Some(sid));
            assert_eq!(is_feasible(&actual), known["feasible"].as_bool().unwrap());
            compare_vectors(&actual, &known["violations"]);
        }
    }
}

#[test]
fn dominance_respects_each_metric_direction_and_is_transitive() {
    let a = json!({
        "service_units_lower_bound":100, "underserved_service_units_lower_bound":100,
        "reliability_lower_bound":0.99, "emissions_tonnes_co2e_upper_bound":1,
        "total_cost":1
    });
    let b = json!({
        "service_units_lower_bound":90, "underserved_service_units_lower_bound":90,
        "reliability_lower_bound":0.98, "emissions_tonnes_co2e_upper_bound":2,
        "total_cost":2
    });
    let c = json!({
        "service_units_lower_bound":80, "underserved_service_units_lower_bound":80,
        "reliability_lower_bound":0.97, "emissions_tonnes_co2e_upper_bound":3,
        "total_cost":3
    });
    assert!(dominates(&a, &b));
    assert!(dominates(&b, &c));
    assert!(dominates(&a, &c));

    let tradeoff = json!({
        "service_units_lower_bound":110, "underserved_service_units_lower_bound":110,
        "reliability_lower_bound":1.0, "emissions_tonnes_co2e_upper_bound":4,
        "total_cost":4
    });
    assert!(!dominates(&a, &tradeoff), "larger output cannot erase worse cost or emissions");
    assert!(!dominates(&tradeoff, &a), "lower cost/emissions cannot erase worse service");
}

#[test]
fn hard_constraints_are_non_compensable_and_violations_keep_their_own_units() {
    let stress = stress_fixture();
    let strategy = &stress["strategies"][2];
    let mut row = strategy["scenario_results"][4].clone();
    row["service_units_lower_bound"] = json!(5000);
    row["underserved_service_units_lower_bound"] = json!(1);
    row["reliability_lower_bound"] = json!(1.0);
    row["emissions_tonnes_co2e_upper_bound"] = json!(1);
    let v = violations(&stress, strategy, &row);
    assert!(num(&v["underserved_service_floor_shortfall"]) > 0.0);
    assert!(!is_feasible(&v), "excellent results in other dimensions cannot compensate an equity-floor breach");
    assert_eq!(v.as_object().unwrap().len(), VIOLATION_FIELDS.len());
    assert!(v.get("weighted_penalty").is_none());
    assert!(v.get("single_score").is_none());
}

#[test]
fn labels_and_array_order_do_not_change_pareto_front_membership() {
    let stress = stress_fixture();
    let scenario = "grid-disruption";
    let candidates = rows_for_scenario(&stress, scenario).unwrap();
    let front_before = pareto_front(&candidates);

    let mut renamed = stress.clone();
    for strategy in renamed["strategies"].as_array_mut().unwrap() {
        strategy["label"] = json!(format!("renamed-{}", strategy_id(strategy)));
    }
    renamed["strategies"].as_array_mut().unwrap().reverse();
    let candidates_after = rows_for_scenario(&renamed, scenario).unwrap();
    assert_eq!(front_before, pareto_front(&candidates_after));
}

#[test]
fn duplicate_scenarios_strategies_or_outcome_cells_are_rejected() {
    let stress = stress_fixture();

    let mut duplicate_scenario = stress.clone();
    let duplicate = duplicate_scenario["scenario_manifest"][0].clone();
    duplicate_scenario["scenario_manifest"].as_array_mut().unwrap().push(duplicate);
    assert!(validate_matrix(&duplicate_scenario).is_err());

    let mut duplicate_strategy = stress.clone();
    let duplicate = duplicate_strategy["strategies"][0].clone();
    duplicate_strategy["strategies"].as_array_mut().unwrap().push(duplicate);
    assert!(validate_matrix(&duplicate_strategy).is_err());

    let mut duplicate_cell = stress.clone();
    let duplicate = duplicate_cell["strategies"][0]["scenario_results"][0].clone();
    duplicate_cell["strategies"][0]["scenario_results"].as_array_mut().unwrap().push(duplicate);
    assert!(validate_matrix(&duplicate_cell).is_err());
}

#[test]
fn empty_feasible_front_is_explicit_not_silently_relaxed() {
    let stress = stress_fixture();
    let candidates = rows_for_scenario(&stress, "prolonged-repeat-disruption").unwrap();
    let feasible: Vec<(&Value, &Value)> = candidates
        .iter()
        .copied()
        .filter(|(strategy, row)| is_feasible(&violations(&stress, strategy, row)))
        .collect();
    assert!(feasible.is_empty());
    assert!(pareto_front(&feasible).is_empty());
}

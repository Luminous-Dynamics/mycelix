use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};

const STRESS_FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json");
const EPS: f64 = 1e-9;

#[derive(Clone, Debug, PartialEq)]
struct ScenarioOracle {
    scenario_id: String,
    feasible_strategy_ids: Vec<String>,
    minimum_feasible_cost: Option<f64>,
    optimal_strategy_ids: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PolicyEfficiency {
    feasible_scenarios: usize,
    defined_cost_regret_cases: usize,
    total_cost_regret: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ConstraintMargins {
    primary_budget: f64,
    contingency_reserve: f64,
    total_cost: f64,
    total_service: f64,
    underserved_service: f64,
    reliability: f64,
    emissions: f64,
}

fn stress_fixture() -> Value {
    serde_json::from_str(STRESS_FIXTURE).expect("synthetic stress fixture parses")
}

fn num(value: &Value) -> f64 {
    value.as_f64().expect("numeric policy field")
}

fn scenario_id(value: &Value) -> &str {
    value["scenario_id"].as_str().expect("scenario ID")
}

fn strategy_id(value: &Value) -> &str {
    value["strategy_id"].as_str().expect("strategy ID")
}

fn scenario_ids(fixture: &Value) -> Vec<String> {
    let mut ids: Vec<String> = fixture["scenario_manifest"]
        .as_array()
        .expect("scenario manifest")
        .iter()
        .map(|item| scenario_id(item).to_owned())
        .collect();
    ids.sort();
    ids
}

fn unique_strategy<'a>(fixture: &'a Value, id: &str) -> &'a Value {
    let matches: Vec<&Value> = fixture["strategies"]
        .as_array()
        .expect("strategies")
        .iter()
        .filter(|strategy| strategy_id(strategy) == id)
        .collect();
    assert_eq!(matches.len(), 1, "strategy ID must be unique: {id}");
    matches[0]
}

fn unique_scenario_row<'a>(strategy: &'a Value, id: &str) -> &'a Value {
    let matches: Vec<&Value> = strategy["scenario_results"]
        .as_array()
        .expect("scenario results")
        .iter()
        .filter(|row| scenario_id(row) == id)
        .collect();
    assert_eq!(
        matches.len(),
        1,
        "{} must have one outcome for {id}",
        strategy_id(strategy)
    );
    matches[0]
}

/// Recompute every declared hard constraint. No service, equity, reliability, emission, or
/// budget dimension can compensate for another. Cost regret is only meaningful after feasibility.
fn feasible_cost(fixture: &Value, strategy: &Value, row: &Value) -> Option<f64> {
    let rules = &fixture["scoring_policy"];
    let base_cost = num(&strategy["base_cost"]);
    let total_cost = num(&row["total_cost"]);
    let reserve_draw = total_cost - base_cost;
    let within_budget = base_cost <= num(&rules["primary_budget"]) + EPS
        && reserve_draw >= -EPS
        && reserve_draw <= num(&rules["contingency_reserve_cap"]) + EPS
        && total_cost <= num(&rules["total_cost_upper_bound"]) + EPS;
    let service = num(&row["service_units_lower_bound"]);
    let underserved = num(&row["underserved_service_units_lower_bound"]);
    let reliability = num(&row["reliability_lower_bound"]);
    let emissions = num(&row["emissions_tonnes_co2e_upper_bound"]);
    let service_constraints = service + EPS >= num(&rules["min_service_units_lower_bound"])
        && underserved + EPS >= num(&rules["min_underserved_service_units_lower_bound"])
        && reliability + EPS >= num(&rules["min_reliability_lower_bound"])
        && emissions <= num(&rules["max_emissions_tonnes_co2e_upper_bound"]) + EPS;

    (within_budget && service_constraints).then_some(total_cost)
}

/// Signed margins retain each constraint's native units; non-negative means the
/// declared floor/cap is met. This is diagnostic only and is not scalarized.
fn constraint_margins(
    fixture: &Value,
    strategy: &Value,
    row: &Value,
) -> ConstraintMargins {
    let rules = &fixture["scoring_policy"];
    let base_cost = num(&strategy["base_cost"]);
    let total_cost = num(&row["total_cost"]);
    ConstraintMargins {
        primary_budget: num(&rules["primary_budget"]) - base_cost,
        contingency_reserve: num(&rules["contingency_reserve_cap"]) - (total_cost - base_cost),
        total_cost: num(&rules["total_cost_upper_bound"]) - total_cost,
        total_service: num(&row["service_units_lower_bound"])
            - num(&rules["min_service_units_lower_bound"]),
        underserved_service: num(&row["underserved_service_units_lower_bound"])
            - num(&rules["min_underserved_service_units_lower_bound"]),
        reliability: num(&row["reliability_lower_bound"])
            - num(&rules["min_reliability_lower_bound"]),
        emissions: num(&rules["max_emissions_tonnes_co2e_upper_bound"])
            - num(&row["emissions_tonnes_co2e_upper_bound"]),
    }
}

fn scenario_oracle(fixture: &Value, id: &str) -> ScenarioOracle {
    let mut feasible_costs = BTreeMap::<String, f64>::new();
    for strategy in fixture["strategies"].as_array().expect("strategies") {
        let row = unique_scenario_row(strategy, id);
        if let Some(cost) = feasible_cost(fixture, strategy, row) {
            feasible_costs.insert(strategy_id(strategy).to_owned(), cost);
        }
    }

    let minimum_cost = feasible_costs.values().copied().reduce(f64::min);
    let mut optimal_strategy_ids: Vec<String> = minimum_cost
        .map(|minimum| {
            feasible_costs
                .iter()
                .filter(|(_, cost)| (**cost - minimum).abs() <= EPS)
                .map(|(id, _)| id.clone())
                .collect()
        })
        .unwrap_or_default();
    optimal_strategy_ids.sort();

    ScenarioOracle {
        scenario_id: id.to_owned(),
        feasible_strategy_ids: feasible_costs.keys().cloned().collect(),
        minimum_feasible_cost: minimum_cost,
        optimal_strategy_ids,
    }
}

fn all_oracles(fixture: &Value) -> BTreeMap<String, ScenarioOracle> {
    scenario_ids(fixture)
        .into_iter()
        .map(|id| (id.clone(), scenario_oracle(fixture, &id)))
        .collect()
}

fn policy_efficiency(fixture: &Value, id: &str) -> PolicyEfficiency {
    let strategy = unique_strategy(fixture, id);
    let oracles = all_oracles(fixture);
    let mut feasible_scenarios = 0usize;
    let mut defined_cost_regret_cases = 0usize;
    let mut total_cost_regret = 0.0_f64;

    for scenario_id in scenario_ids(fixture) {
        let row = unique_scenario_row(strategy, &scenario_id);
        let policy_cost = feasible_cost(fixture, strategy, row);
        if policy_cost.is_some() {
            feasible_scenarios += 1;
        }
        if let (Some(policy_cost), Some(oracle_cost)) = (
            policy_cost,
            oracles[&scenario_id].minimum_feasible_cost,
        ) {
            let regret = policy_cost - oracle_cost;
            assert!(regret >= -EPS, "policy cannot beat its catalog oracle");
            total_cost_regret += regret.max(0.0);
            defined_cost_regret_cases += 1;
        }
    }

    PolicyEfficiency {
        feasible_scenarios,
        defined_cost_regret_cases,
        total_cost_regret,
    }
}

fn validate_matrix(fixture: &Value) {
    let manifest: Vec<String> = scenario_ids(fixture);
    assert_eq!(manifest.len(), 5, "fixture scenario count changed");
    assert_eq!(
        manifest.iter().collect::<BTreeSet<_>>().len(),
        manifest.len(),
        "scenario IDs must be unique"
    );

    let strategies = fixture["strategies"].as_array().expect("strategies");
    assert_eq!(strategies.len(), 3, "fixture strategy count changed");
    let ids: Vec<&str> = strategies.iter().map(strategy_id).collect();
    assert_eq!(
        ids.iter().collect::<BTreeSet<_>>().len(),
        ids.len(),
        "strategy IDs must be unique"
    );

    assert!(
        fixture["purpose"].as_str().unwrap_or_default().contains("fully synthetic"),
        "known-future results must retain the synthetic claim ceiling"
    );
    for strategy in strategies {
        let rows = strategy["scenario_results"].as_array().expect("scenario results");
        let row_ids: Vec<&str> = rows.iter().map(scenario_id).collect();
        assert_eq!(row_ids.len(), manifest.len());
        assert_eq!(
            row_ids.iter().collect::<BTreeSet<_>>().len(),
            row_ids.len(),
            "each strategy must have exactly one outcome per scenario"
        );
        assert_eq!(
            row_ids.iter().copied().collect::<BTreeSet<_>>(),
            manifest.iter().map(String::as_str).collect::<BTreeSet<_>>()
        );
        for row in rows {
            assert_eq!(
                row["evidence_status"],
                json!("synthetic_stress_assumption"),
                "this oracle must not silently promote synthetic outcomes to empirical evidence"
            );
        }
    }
}

#[test]
fn known_future_oracle_finds_four_feasible_scenarios_and_one_no_policy_scenario() {
    let fixture = stress_fixture();
    validate_matrix(&fixture);
    let oracles = all_oracles(&fixture);

    assert_eq!(
        oracles["nominal-operations"].feasible_strategy_ids,
        vec!["adaptive_staged".to_owned(), "equity_static".to_owned()]
    );
    assert_eq!(oracles["nominal-operations"].minimum_feasible_cost, Some(9.0));
    assert_eq!(
        oracles["nominal-operations"].optimal_strategy_ids,
        vec!["adaptive_staged".to_owned(), "equity_static".to_owned()]
    );

    for (id, cost) in [
        ("seasonal-drought", 12.0),
        ("grid-disruption", 12.0),
        ("compound-drought-grid-crew", 13.0),
    ] {
        assert_eq!(
            oracles[id].feasible_strategy_ids,
            vec!["adaptive_staged".to_owned()],
            "unexpected feasible policy catalog for {id}"
        );
        assert_eq!(oracles[id].minimum_feasible_cost, Some(cost));
        assert_eq!(
            oracles[id].optimal_strategy_ids,
            vec!["adaptive_staged".to_owned()]
        );
    }

    let prolonged = &oracles["prolonged-repeat-disruption"];
    assert!(prolonged.feasible_strategy_ids.is_empty());
    assert_eq!(prolonged.minimum_feasible_cost, None);
    assert!(prolonged.optimal_strategy_ids.is_empty());

    assert_eq!(
        oracles.values().filter(|row| row.minimum_feasible_cost.is_some()).count(),
        4
    );
    assert_eq!(
        oracles.values().filter(|row| row.minimum_feasible_cost.is_none()).count(),
        1
    );
}

#[test]
fn policy_cost_regret_is_separate_from_infeasibility() {
    let fixture = stress_fixture();
    let adaptive = policy_efficiency(&fixture, "adaptive_staged");
    let equity = policy_efficiency(&fixture, "equity_static");
    let output_only = policy_efficiency(&fixture, "output_only_static");

    assert_eq!(
        adaptive,
        PolicyEfficiency {
            feasible_scenarios: 4,
            defined_cost_regret_cases: 4,
            total_cost_regret: 0.0,
        }
    );
    assert_eq!(
        equity,
        PolicyEfficiency {
            feasible_scenarios: 1,
            defined_cost_regret_cases: 1,
            total_cost_regret: 0.0,
        }
    );
    assert_eq!(
        output_only,
        PolicyEfficiency {
            feasible_scenarios: 0,
            defined_cost_regret_cases: 0,
            total_cost_regret: 0.0,
        }
    );

    // Do not present undefined regret as zero performance cost when a policy is infeasible.
    assert!(policy_efficiency(&fixture, "output_only_static").defined_cost_regret_cases == 0);
}

#[test]
fn oracle_is_invariant_to_strategy_and_scenario_row_order() {
    let original = stress_fixture();
    let expected = all_oracles(&original);
    let mut permuted = original.clone();
    permuted["scenario_manifest"].as_array_mut().unwrap().reverse();
    permuted["strategies"].as_array_mut().unwrap().reverse();
    for strategy in permuted["strategies"].as_array_mut().unwrap() {
        strategy["scenario_results"].as_array_mut().unwrap().reverse();
    }
    assert_eq!(expected, all_oracles(&permuted));
}

#[test]
fn a_hard_constraint_mutation_can_make_a_previously_feasible_scenario_infeasible() {
    let original = stress_fixture();
    let original_strategy = unique_strategy(&original, "adaptive_staged");
    let original_drought = unique_scenario_row(original_strategy, "seasonal-drought");
    assert!(feasible_cost(&original, original_strategy, original_drought).is_some());

    let mut fixture = original;
    {
        let adaptive = fixture["strategies"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|strategy| strategy_id(strategy) == "adaptive_staged")
            .unwrap();
        let drought = adaptive["scenario_results"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|row| scenario_id(row) == "seasonal-drought")
            .unwrap();
        drought["service_units_lower_bound"] = json!(799);
    }

    let altered_strategy = unique_strategy(&fixture, "adaptive_staged");
    let altered_drought = unique_scenario_row(altered_strategy, "seasonal-drought");
    assert!(
        feasible_cost(&fixture, altered_strategy, altered_drought).is_none(),
        "a high score in other dimensions must not compensate for a violated service floor"
    );
    assert_eq!(
        scenario_oracle(&fixture, "seasonal-drought").minimum_feasible_cost,
        None,
        "the oracle must expose catalog infeasibility instead of relaxing the floor"
    );
}

#[test]
fn prolonged_repeat_case_reports_each_native_unit_constraint_margin() {
    let fixture = stress_fixture();
    let strategy = unique_strategy(&fixture, "adaptive_staged");
    let row = unique_scenario_row(&strategy, "prolonged-repeat-disruption");
    let margin = constraint_margins(&fixture, strategy, row);

    assert!((margin.primary_budget - 1.0).abs() <= EPS);
    assert!(margin.contingency_reserve.abs() <= EPS);
    assert!((margin.total_cost - 1.0).abs() <= EPS);
    assert!((margin.total_service + 10.0).abs() <= EPS);
    assert!((margin.underserved_service + 40.0).abs() <= EPS);
    assert!((margin.reliability + 0.02).abs() <= EPS);
    assert!(margin.emissions.abs() <= EPS);

    assert!(
        feasible_cost(&fixture, strategy, row).is_none(),
        "native-unit slacks must not hide the simultaneous service/equity/reliability violations"
    );
}

#[test]
fn primary_budget_reserve_and_total_cost_are_non_compensable() {
    let mut fixture = stress_fixture();
    let mut strategy = fixture["strategies"]
        .as_array()
        .unwrap()
        .iter()
        .find(|strategy| strategy_id(strategy) == "adaptive_staged")
        .unwrap()
        .clone();
    let mut row = unique_scenario_row(&strategy, "grid-disruption").clone();

    row["total_cost"] = json!(15);
    assert!(feasible_cost(&fixture, &strategy, &row).is_none());

    row["total_cost"] = json!(15);
    strategy["base_cost"] = json!(11);
    assert!(feasible_cost(&fixture, &strategy, &row).is_none());
}

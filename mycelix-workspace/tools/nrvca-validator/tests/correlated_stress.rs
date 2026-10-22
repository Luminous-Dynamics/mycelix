use serde_json::{json, Value};

const FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json");

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("stress fixture parses")
}

fn num(value: &Value) -> f64 {
    value.as_f64().expect("numeric fixture field")
}

fn strategy<'a>(root: &'a Value, strategy_id: &str) -> &'a Value {
    root["strategies"].as_array().unwrap().iter()
        .find(|item| item["strategy_id"].as_str() == Some(strategy_id))
        .unwrap_or_else(|| panic!("missing strategy {strategy_id}"))
}

fn outcome<'a>(strategy: &'a Value, scenario_id: &str) -> &'a Value {
    strategy["scenario_results"].as_array().unwrap().iter()
        .find(|item| item["scenario_id"].as_str() == Some(scenario_id))
        .unwrap_or_else(|| panic!("missing scenario {scenario_id} for strategy"))
}

fn passes(root: &Value, result: &Value) -> bool {
    let p = &root["scoring_policy"];
    num(&result["total_cost"]) <= num(&p["total_cost_upper_bound"]) + 1e-9
        && num(&result["service_units_lower_bound"])
            >= num(&p["min_service_units_lower_bound"]) - 1e-9
        && num(&result["underserved_service_units_lower_bound"])
            >= num(&p["min_underserved_service_units_lower_bound"]) - 1e-9
        && num(&result["reliability_lower_bound"])
            >= num(&p["min_reliability_lower_bound"]) - 1e-9
        && num(&result["emissions_tonnes_co2e_upper_bound"])
            <= num(&p["max_emissions_tonnes_co2e_upper_bound"]) + 1e-9
}

#[derive(Debug)]
struct Summary {
    pass_count: usize,
    scenario_count: usize,
    worst_service_shortfall: f64,
    worst_underserved_shortfall: f64,
    worst_reliability_shortfall: f64,
    worst_emissions_excess: f64,
    failed_scenarios: Vec<String>,
}

fn summarize(root: &Value, item: &Value) -> Summary {
    let p = &root["scoring_policy"];
    let scenarios = root["scenario_manifest"].as_array().unwrap();
    let mut pass_count = 0;
    let mut worst_service_shortfall: f64 = 0.0;
    let mut worst_underserved_shortfall: f64 = 0.0;
    let mut worst_reliability_shortfall: f64 = 0.0;
    let mut worst_emissions_excess: f64 = 0.0;
    let mut failed_scenarios = Vec::new();

    for scenario in scenarios {
        let id = scenario["scenario_id"].as_str().unwrap();
        let row = outcome(item, id);
        let ok = passes(root, row);
        if ok {
            pass_count += 1;
        } else {
            failed_scenarios.push(id.to_owned());
        }
        worst_service_shortfall = worst_service_shortfall.max(
            num(&p["min_service_units_lower_bound"]) - num(&row["service_units_lower_bound"])
        );
        worst_underserved_shortfall = worst_underserved_shortfall.max(
            num(&p["min_underserved_service_units_lower_bound"])
                - num(&row["underserved_service_units_lower_bound"])
        );
        worst_reliability_shortfall = worst_reliability_shortfall.max(
            num(&p["min_reliability_lower_bound"]) - num(&row["reliability_lower_bound"])
        );
        worst_emissions_excess = worst_emissions_excess.max(
            num(&row["emissions_tonnes_co2e_upper_bound"])
                - num(&p["max_emissions_tonnes_co2e_upper_bound"])
        );
    }
    Summary {
        pass_count,
        scenario_count: scenarios.len(),
        worst_service_shortfall: worst_service_shortfall.max(0.0),
        worst_underserved_shortfall: worst_underserved_shortfall.max(0.0),
        worst_reliability_shortfall: worst_reliability_shortfall.max(0.0),
        worst_emissions_excess: worst_emissions_excess.max(0.0),
        failed_scenarios,
    }
}

#[test]
fn every_strategy_covers_every_declared_correlated_scenario_exactly_once() {
    let root = fixture();
    let scenarios = root["scenario_manifest"].as_array().unwrap();
    let expected_ids: Vec<String> = scenarios.iter()
        .map(|s| s["scenario_id"].as_str().unwrap().to_owned())
        .collect();
    let mut unique = expected_ids.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(unique.len(), expected_ids.len(), "scenario ids must be unique");
    assert_eq!(scenarios.len(), 5);
    assert_eq!(root["strategies"].as_array().unwrap().len(), 3);

    for item in root["strategies"].as_array().unwrap() {
        let rows = item["scenario_results"].as_array().unwrap();
        assert_eq!(rows.len(), scenarios.len(), "no silent scenario omissions");
        let actual_ids: Vec<String> = rows.iter()
            .map(|r| r["scenario_id"].as_str().unwrap().to_owned())
            .collect();
        let mut sorted_actual = actual_ids.clone();
        sorted_actual.sort();
        sorted_actual.dedup();
        assert_eq!(sorted_actual.len(), actual_ids.len(), "no duplicate strategy-scenario cells");
        let mut expected = expected_ids.clone();
        expected.sort();
        assert_eq!(sorted_actual, expected, "scenario cross-product must be complete");
        for row in rows {
            assert_eq!(row["evidence_status"], "synthetic_stress_assumption");
            assert!(row.get("probability").is_none(), "do not invent scenario probabilities");
        }
    }
}

#[test]
fn robust_success_and_worst_case_diagnostics_match_frozen_known_answers() {
    let root = fixture();
    for expected in root["expected_summaries"].as_array().unwrap() {
        let item = strategy(&root, expected["strategy_id"].as_str().unwrap());
        let actual = summarize(&root, item);
        assert_eq!(actual.pass_count, expected["robust_pass_count"].as_u64().unwrap() as usize);
        assert_eq!(actual.scenario_count, expected["scenario_count"].as_u64().unwrap() as usize);
        close_fraction(
            actual.pass_count as f64 / actual.scenario_count as f64,
            num(&expected["robust_pass_fraction"]),
        );
        close(
            actual.worst_service_shortfall,
            num(&expected["worst_service_shortfall"]),
        );
        close(
            actual.worst_underserved_shortfall,
            num(&expected["worst_underserved_shortfall"]),
        );
        close(
            actual.worst_reliability_shortfall,
            num(&expected["worst_reliability_shortfall"]),
        );
        close(
            actual.worst_emissions_excess,
            num(&expected["worst_emissions_excess"]),
        );
        assert_eq!(
            actual.failed_scenarios,
            expected["failed_scenarios"].as_array().unwrap()
                .iter().map(|s| s.as_str().unwrap().to_owned()).collect::<Vec<_>>()
        );
    }
}

fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-7, "{actual} differs from {expected}");
}

fn close_fraction(actual: f64, expected: f64) {
    close(actual, expected);
}

#[test]
fn adaptive_triggers_follow_declared_stressors_and_have_no_hidden_probability_weights() {
    let root = fixture();
    let adaptive = strategy(&root, "adaptive_staged");
    let rules = root["trigger_rules"].as_array().unwrap();

    for scenario in root["scenario_manifest"].as_array().unwrap() {
        let id = scenario["scenario_id"].as_str().unwrap();
        let stressors: Vec<&str> = scenario["stressors"].as_array().unwrap()
            .iter().map(|s| s.as_str().unwrap()).collect();
        let mut expected_actions = Vec::new();
        for rule in rules {
            let fire = rule["trigger_if_any"].as_array().unwrap().iter()
                .any(|tag| stressors.contains(&tag.as_str().unwrap()));
            if fire {
                expected_actions.push(rule["action_id"].as_str().unwrap().to_owned());
            }
        }
        expected_actions.sort();
        let mut actual_actions: Vec<String> = outcome(adaptive, id)["triggered_actions"]
            .as_array().unwrap().iter()
            .map(|x| x.as_str().unwrap().to_owned()).collect();
        actual_actions.sort();
        assert_eq!(actual_actions, expected_actions, "trigger mismatch in {id}");
    }
    assert_eq!(root["scoring_policy"]["scenario_weights_used"], false);
}

#[test]
fn adding_a_new_failing_scenario_cannot_increase_robust_success_fraction() {
    let mut root = fixture();
    let adaptive_before = strategy(&root, "adaptive_staged");
    let summary_before = summarize(&root, adaptive_before);
    let fraction_before = summary_before.pass_count as f64 / summary_before.scenario_count as f64;

    root["scenario_manifest"].as_array_mut().unwrap().push(json!({
        "scenario_id":"reserve-exhaustion-plus-delayed-repair",
        "stressors":["source_availability_low","grid_disruption","repair_crew_delay","reserve_depleted"],
        "probability":0.99,
        "description":"Additional deliberately failing stress case; probability metadata must be ignored by the scenario-neutral score."
    }));
    for item in root["strategies"].as_array_mut().unwrap() {
        item["scenario_results"].as_array_mut().unwrap().push(json!({
            "scenario_id":"reserve-exhaustion-plus-delayed-repair",
            "service_units_lower_bound":400,
            "underserved_service_units_lower_bound":100,
            "reliability_lower_bound":0.40,
            "emissions_tonnes_co2e_upper_bound":12.0,
            "total_cost":13,
            "triggered_actions":[],
            "evidence_status":"synthetic_stress_assumption"
        }));
    }

    let adaptive_after = strategy(&root, "adaptive_staged");
    let summary_after = summarize(&root, adaptive_after);
    let fraction_after = summary_after.pass_count as f64 / summary_after.scenario_count as f64;
    assert!(fraction_after < fraction_before);
}

#[test]
fn descriptive_labels_and_probability_metadata_cannot_change_scenario_neutral_metrics() {
    let mut root = fixture();
    let before = summarize(&root, strategy(&root, "adaptive_staged"));
    root["scenario_manifest"][0]["probability"] = json!(0.9999);
    root["strategies"][2]["label"] = json!("Marketing label changed only");
    let after = summarize(&root, strategy(&root, "adaptive_staged"));
    assert_eq!(before.pass_count, after.pass_count);
    assert_eq!(before.scenario_count, after.scenario_count);
    close(before.worst_service_shortfall, after.worst_service_shortfall);
    close(before.worst_underserved_shortfall, after.worst_underserved_shortfall);
    close(before.worst_reliability_shortfall, after.worst_reliability_shortfall);
    close(before.worst_emissions_excess, after.worst_emissions_excess);
}

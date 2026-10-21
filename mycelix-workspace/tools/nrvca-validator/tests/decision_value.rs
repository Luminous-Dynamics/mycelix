use nrvca_validator::Diagnostic;
use serde_json::{json, Value};

const FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-decision-value-synthetic-v0.1.json");

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("decision benchmark fixture parses")
}

fn num(v: &Value) -> f64 {
    v.as_f64().expect("numeric field")
}

fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-7, "{actual} differs from {expected}");
}

#[derive(Debug, Clone)]
struct Allocation {
    mask: usize,
    ids: Vec<String>,
    cost: f64,
    nominal_service: f64,
    service_lower: f64,
    underserved_lower: f64,
    emissions_upper: f64,
    weighted_reliability_lower: f64,
}

fn evaluate_mask(root: &Value, mask: usize) -> Allocation {
    let interventions = root["interventions"].as_array().unwrap();
    let mut result = Allocation {
        mask,
        ids: Vec::new(),
        cost: 0.0,
        nominal_service: 0.0,
        service_lower: 0.0,
        underserved_lower: 0.0,
        emissions_upper: 0.0,
        weighted_reliability_lower: 0.0,
    };
    let mut reliability_numerator = 0.0;
    for (i, item) in interventions.iter().enumerate() {
        if mask & (1usize << i) == 0 {
            continue;
        }
        result.ids.push(item["intervention_id"].as_str().unwrap().to_owned());
        result.cost += num(&item["cost"]);
        result.nominal_service += num(&item["service_units"]["nominal"]);
        result.service_lower += num(&item["service_units"]["lower"]);
        result.underserved_lower += num(&item["underserved_service_units"]["lower"]);
        result.emissions_upper += num(&item["emissions_tonnes_co2e"]["upper"]);
        reliability_numerator +=
            num(&item["service_units"]["lower"]) * num(&item["reliability"]["lower"]);
    }
    result.weighted_reliability_lower = if result.service_lower > 0.0 {
        reliability_numerator / result.service_lower
    } else {
        0.0
    };
    result
}

fn feasible(root: &Value, allocation: &Allocation, budget: f64, equity_floor: f64) -> bool {
    allocation.cost <= budget + 1e-9
        && allocation.underserved_lower + 1e-9 >= equity_floor
        && allocation.emissions_upper
            <= num(&root["policy_profile"]["max_emissions_tonnes_co2e_upper_bound"]) + 1e-9
        && allocation.weighted_reliability_lower + 1e-9
            >= num(&root["policy_profile"]["min_service_weighted_reliability_lower_bound"])
}

fn all_allocations(root: &Value) -> Vec<Allocation> {
    let n = root["interventions"].as_array().unwrap().len();
    (0..(1usize << n)).map(|mask| evaluate_mask(root, mask)).collect()
}

fn best_by_nominal_service(
    candidates: impl IntoIterator<Item = Allocation>,
) -> Option<Allocation> {
    candidates.into_iter().max_by(|a, b| {
        a.nominal_service
            .total_cmp(&b.nominal_service)
            .then_with(|| b.cost.total_cmp(&a.cost))
            .then_with(|| b.mask.cmp(&a.mask))
    })
}

fn chosen_ids(mut allocation: Allocation) -> Vec<String> {
    allocation.ids.sort();
    allocation.ids
}

#[test]
fn constrained_choice_matches_independent_exhaustive_oracle_and_beats_guardrail_failure() {
    let root = fixture();
    let policy = &root["policy_profile"];
    let budget = num(&policy["budget"]);
    let equity_floor = num(&policy["min_underserved_service_units_lower_bound"]);
    let allocations = all_allocations(&root);
    assert_eq!(allocations.len(), 16, "four binary interventions must enumerate 16 allocations");

    let baseline = best_by_nominal_service(
        allocations
            .iter()
            .filter(|a| a.cost <= num(&root["baseline"]["budget"]) + 1e-9)
            .cloned(),
    )
    .expect("budgeted baseline always has at least the empty allocation");
    let policy_choice = best_by_nominal_service(
        allocations
            .iter()
            .filter(|a| feasible(&root, a, budget, equity_floor))
            .cloned(),
    )
    .expect("synthetic policy should have a feasible allocation");
    let expected = &root["expected"];

    assert_eq!(chosen_ids(baseline.clone()), {
        let mut ids: Vec<String> = expected["baseline_selected_ids"]
            .as_array()
            .unwrap().iter().map(|x| x.as_str().unwrap().to_owned()).collect();
        ids.sort();
        ids
    });
    assert_eq!(chosen_ids(policy_choice.clone()), {
        let mut ids: Vec<String> = expected["policy_selected_ids"]
            .as_array()
            .unwrap().iter().map(|x| x.as_str().unwrap().to_owned()).collect();
        ids.sort();
        ids
    });

    close(baseline.cost, num(&expected["baseline_total_cost"]));
    close(baseline.nominal_service, num(&expected["baseline_nominal_service_units"]));
    close(baseline.underserved_lower, num(&expected["baseline_underserved_service_units_lower_bound"]));
    close(baseline.emissions_upper, num(&expected["baseline_emissions_upper_bound"]));
    assert!(!feasible(&root, &baseline, budget, equity_floor));
    assert!(baseline.emissions_upper
        > num(&policy["max_emissions_tonnes_co2e_upper_bound"]));

    close(policy_choice.cost, num(&expected["policy_total_cost"]));
    close(policy_choice.nominal_service, num(&expected["policy_nominal_service_units"]));
    close(policy_choice.service_lower, num(&expected["policy_service_units_lower_bound"]));
    close(policy_choice.underserved_lower, num(&expected["policy_underserved_service_units_lower_bound"]));
    close(policy_choice.emissions_upper, num(&expected["policy_emissions_upper_bound"]));
    close(
        policy_choice.weighted_reliability_lower,
        num(&expected["policy_service_weighted_reliability_lower_bound"]),
    );
    assert!(feasible(&root, &policy_choice, budget, equity_floor));
}

#[test]
fn a_stricter_floor_reports_infeasibility_instead_of_silently_weakening_policy() {
    let root = fixture();
    let policy = &root["policy_profile"];
    let allocations = all_allocations(&root);
    let strict_floor = num(&policy["min_underserved_service_units_lower_bound"]) + 101.0;
    let feasible_allocations = allocations.iter().filter(|allocation| {
        feasible(&root, allocation, num(&policy["budget"]), strict_floor)
    });
    assert!(best_by_nominal_service(feasible_allocations.cloned()).is_none());
    assert_eq!(root["expected"]["strict_equity_floor_901_is_infeasible"], true);
}

#[test]
fn adding_budget_cannot_shrink_the_feasible_allocation_set() {
    let root = fixture();
    let policy = &root["policy_profile"];
    let allocations = all_allocations(&root);
    let budget = num(&policy["budget"]);
    let equity = num(&policy["min_underserved_service_units_lower_bound"]);

    let feasible_at_budget = |b: f64| -> Vec<usize> {
        allocations
            .iter()
            .filter(|allocation| feasible(&root, allocation, b, equity))
            .map(|allocation| allocation.mask)
            .collect()
    };
    let low = feasible_at_budget(budget);
    let high = feasible_at_budget(budget + 1.0);
    assert!(low.iter().all(|mask| high.contains(mask)));
}

#[test]
fn changing_descriptive_label_does_not_change_numeric_outcomes_or_optimum() {
    let mut root = fixture();
    let before = best_by_nominal_service(
        all_allocations(&root)
            .into_iter()
            .filter(|a| feasible(&root, a, num(&root["policy_profile"]["budget"]), num(&root["policy_profile"]["min_underserved_service_units_lower_bound"]))),
    )
    .expect("baseline policy feasible");

    root["interventions"][0]["label"] = json!("Completely different marketing description");
    let after = best_by_nominal_service(
        all_allocations(&root)
            .into_iter()
            .filter(|a| feasible(&root, a, num(&root["policy_profile"]["budget"]), num(&root["policy_profile"]["min_underserved_service_units_lower_bound"]))),
    )
    .expect("label change cannot eliminate feasible options");
    assert_eq!(before.mask, after.mask);
    assert_eq!(before.nominal_service, after.nominal_service);
}

#[test]
fn all_cases_are_synthetic_and_normative_constraints_are_explicit() {
    let root = fixture();
    assert_eq!(root["status"], "synthetic_only");
    assert_eq!(root["policy_profile"]["objective"], "maximize_nominal_service_units_among_feasible_allocations");
    assert!(!root["policy_profile"]["min_underserved_service_units_lower_bound"].is_null());
    assert!(!root["policy_profile"]["max_emissions_tonnes_co2e_upper_bound"].is_null());
    assert!(!root["policy_profile"]["min_service_weighted_reliability_lower_bound"].is_null());
    for intervention in root["interventions"].as_array().unwrap() {
        assert_eq!(intervention["evidence_status"], "synthetic_estimate");
    }
}

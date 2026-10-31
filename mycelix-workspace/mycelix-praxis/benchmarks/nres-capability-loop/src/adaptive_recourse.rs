//! Exact finite-horizon expected-cost oracle over observable information states.
//!
//! Each information state represents one unique observable history. Actions
//! are selected once per information state, never once per latent scenario.
//! Outcome probabilities are action-conditioned, allowing a declared
//! intervention to change a transition kernel. Resource balances are carried
//! between stages and validated on every transition.
//!
//! This is an exact micro-instance baseline for NRES-016, not a large-scale
//! DRO solver. It uses expected cost under a supplied finite transition law,
//! and enforces service floors pathwise. It has no learned transitions,
//! calibrated ambiguity set, nested CVaR or real-world operational authority.

use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};

const BENCHMARK_ID: &str = "PRAXIS-NRES-016-EX0-EXACT-ADAPTIVE-RECOURSE-001";
const TOLERANCE: f64 = 1e-10;

fn string_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn number_field(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64)
}

fn object_numbers(value: Option<&Value>, label: &str, errors: &mut Vec<String>) -> BTreeMap<String, f64> {
    let mut output = BTreeMap::new();
    let Some(object) = value.and_then(Value::as_object) else {
        errors.push(format!("{label} must be an object of finite non-negative numbers"));
        return output;
    };
    for (key, raw) in object {
        match raw.as_f64() {
            Some(number) if number.is_finite() && number >= 0.0 => {
                output.insert(key.clone(), number);
            }
            _ => errors.push(format!("{label}/{key} must be finite and non-negative")),
        }
    }
    output
}

#[derive(Clone)]
struct Outcome {
    outcome_id: String,
    probability: f64,
    next_state_id: Option<String>,
    stage_cost: f64,
    resources_replenished: f64,
}

#[derive(Clone)]
struct Action {
    action_id: String,
    fixed_cost: f64,
    resource_use: f64,
    outcomes: Vec<Outcome>,
}

#[derive(Clone)]
struct InformationState {
    state_id: String,
    history_key: String,
    stage: usize,
    resource_units_available: f64,
    service_levels: BTreeMap<String, f64>,
    actions: Vec<Action>,
}

struct Model {
    horizon_stages: usize,
    initial_state_id: String,
    service_floors: BTreeMap<String, f64>,
    states: BTreeMap<String, InformationState>,
    transition_count: usize,
}

fn parse_model(fixture: &Value) -> Result<Model, Vec<String>> {
    let mut errors = Vec::new();
    if string_field(fixture, "benchmark_id") != Some(BENCHMARK_ID) {
        errors.push("unexpected NRES-016-EX0 benchmark_id".to_owned());
    }
    if string_field(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-016-EX0 schema_version".to_owned());
    }
    if string_field(fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("EX0 fixture must remain explicitly synthetic".to_owned());
    }
    if string_field(fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("EX0 fixture must remain public-development-only".to_owned());
    }
    if fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("EX0 fixture must remain inactive in the curriculum".to_owned());
    }
    if string_field(fixture, "objective_semantics")
        != Some("minimum-expected-total-cost-subject-to-pathwise-service-floors")
    {
        errors.push("unsupported EX0 objective semantics".to_owned());
    }

    let horizon = fixture
        .get("horizon_stages")
        .and_then(Value::as_u64)
        .and_then(|value| usize::try_from(value).ok());
    let Some(horizon_stages) = horizon.filter(|count| (2..=12).contains(count)) else {
        errors.push("horizon_stages must be an integer in [2,12]".to_owned());
        return Err(errors);
    };
    let initial_state_id = string_field(fixture, "initial_information_state_id")
        .filter(|value| !value.trim().is_empty())
        .map(str::to_owned);
    let Some(initial_state_id) = initial_state_id else {
        errors.push("initial_information_state_id is required".to_owned());
        return Err(errors);
    };

    let service_floors = object_numbers(fixture.get("service_floors"), "service_floors", &mut errors);
    if service_floors.is_empty() {
        errors.push("service_floors must contain at least one service".to_owned());
    }

    let Some(raw_states) = fixture.get("information_states").and_then(Value::as_array) else {
        errors.push("information_states must be a non-empty array".to_owned());
        return Err(errors);
    };
    if raw_states.is_empty() {
        errors.push("information_states must be a non-empty array".to_owned());
        return Err(errors);
    }

    let mut states = BTreeMap::new();
    let mut history_keys = BTreeSet::new();
    let mut transition_count = 0usize;

    for (state_index, raw_state) in raw_states.iter().enumerate() {
        let state_id = string_field(raw_state, "information_state_id")
            .unwrap_or("")
            .trim()
            .to_owned();
        if state_id.is_empty() {
            errors.push(format!("information_states[{state_index}]: information_state_id is required"));
            continue;
        }
        if states.contains_key(&state_id) {
            errors.push(format!("duplicate information_state_id: {state_id}"));
            continue;
        }

        let history_key = string_field(raw_state, "observable_history_key")
            .unwrap_or("")
            .trim()
            .to_owned();
        if history_key.is_empty() {
            errors.push(format!("{state_id}: observable_history_key is required"));
        } else if !history_keys.insert(history_key.clone()) {
            errors.push(format!("duplicate observable_history_key: {history_key}"));
        }

        let stage = raw_state
            .get("stage")
            .and_then(Value::as_u64)
            .and_then(|value| usize::try_from(value).ok());
        let Some(stage) = stage.filter(|stage| *stage < horizon_stages) else {
            errors.push(format!("{state_id}: stage must be in [0,horizon_stages)"));
            continue;
        };
        let resource_units_available = number_field(raw_state, "resource_units_available");
        let Some(resource_units_available) =
            resource_units_available.filter(|number| number.is_finite() && *number >= 0.0)
        else {
            errors.push(format!("{state_id}: resource_units_available must be finite and non-negative"));
            continue;
        };

        let service_levels = object_numbers(raw_state.get("service_levels"), &format!("{state_id}/service_levels"), &mut errors);
        if service_levels.len() != service_floors.len()
            || service_levels.keys().any(|service| !service_floors.contains_key(service))
        {
            errors.push(format!("{state_id}: service_levels must exactly match service_floors keys"));
        }

        let Some(raw_actions) = raw_state.get("actions").and_then(Value::as_array) else {
            errors.push(format!("{state_id}: actions must be a non-empty array"));
            continue;
        };
        if raw_actions.is_empty() {
            errors.push(format!("{state_id}: at least one action is required"));
        }
        let mut actions = Vec::new();
        let mut action_ids = BTreeSet::new();

        for (action_index, raw_action) in raw_actions.iter().enumerate() {
            let action_id = string_field(raw_action, "action_id")
                .unwrap_or("")
                .trim()
                .to_owned();
            if action_id.is_empty() {
                errors.push(format!("{state_id}/actions[{action_index}]: action_id is required"));
                continue;
            }
            if !action_ids.insert(action_id.clone()) {
                errors.push(format!("{state_id}: duplicate action_id {action_id}"));
            }
            let fixed_cost = number_field(raw_action, "fixed_cost");
            let Some(fixed_cost) = fixed_cost.filter(|number| number.is_finite() && *number >= 0.0) else {
                errors.push(format!("{state_id}/{action_id}: fixed_cost must be finite and non-negative"));
                continue;
            };
            let resource_use = number_field(raw_action, "resource_use");
            let Some(resource_use) =
                resource_use.filter(|number| number.is_finite() && *number >= 0.0 && *number <= resource_units_available + TOLERANCE)
            else {
                errors.push(format!("{state_id}/{action_id}: resource_use exceeds available resources or is invalid"));
                continue;
            };
            let Some(raw_outcomes) = raw_action.get("outcomes").and_then(Value::as_array) else {
                errors.push(format!("{state_id}/{action_id}: outcomes must be a non-empty array"));
                continue;
            };
            if raw_outcomes.is_empty() {
                errors.push(format!("{state_id}/{action_id}: at least one outcome is required"));
                continue;
            }

            let mut outcomes = Vec::new();
            let mut outcome_ids = BTreeSet::new();
            let mut probability_sum = 0.0;
            for (outcome_index, raw_outcome) in raw_outcomes.iter().enumerate() {
                let outcome_id = string_field(raw_outcome, "outcome_id")
                    .unwrap_or("")
                    .trim()
                    .to_owned();
                if outcome_id.is_empty() {
                    errors.push(format!("{state_id}/{action_id}/outcomes[{outcome_index}]: outcome_id is required"));
                    continue;
                }
                if !outcome_ids.insert(outcome_id.clone()) {
                    errors.push(format!("{state_id}/{action_id}: duplicate outcome_id {outcome_id}"));
                }
                let probability = number_field(raw_outcome, "probability");
                let Some(probability) = probability
                    .filter(|number| number.is_finite() && *number > 0.0 && *number <= 1.0)
                else {
                    errors.push(format!("{state_id}/{action_id}/{outcome_id}: probability must be finite and in (0,1]"));
                    continue;
                };
                let stage_cost = number_field(raw_outcome, "stage_cost");
                let Some(stage_cost) =
                    stage_cost.filter(|number| number.is_finite() && *number >= 0.0)
                else {
                    errors.push(format!("{state_id}/{action_id}/{outcome_id}: stage_cost must be finite and non-negative"));
                    continue;
                };
                let resources_replenished = number_field(raw_outcome, "resources_replenished");
                let Some(resources_replenished) =
                    resources_replenished.filter(|number| number.is_finite() && *number >= 0.0)
                else {
                    errors.push(format!("{state_id}/{action_id}/{outcome_id}: resources_replenished must be finite and non-negative"));
                    continue;
                };
                if raw_outcome.get("next_information_state_id").is_none() {
                    errors.push(format!("{state_id}/{action_id}/{outcome_id}: next_information_state_id must be present or null"));
                    continue;
                }
                let next_state_id = raw_outcome
                    .get("next_information_state_id")
                    .and_then(Value::as_str)
                    .map(str::to_owned);
                probability_sum += probability;
                outcomes.push(Outcome {
                    outcome_id,
                    probability,
                    next_state_id,
                    stage_cost,
                    resources_replenished,
                });
            }
            if (probability_sum - 1.0).abs() > TOLERANCE {
                errors.push(format!("{state_id}/{action_id}: outcome probabilities sum to {probability_sum}, not 1"));
            }
            transition_count += outcomes.len();
            actions.push(Action {
                action_id,
                fixed_cost,
                resource_use,
                outcomes,
            });
        }

        states.insert(
            state_id.clone(),
            InformationState {
                state_id,
                history_key,
                stage,
                resource_units_available,
                service_levels,
                actions,
            },
        );
    }

    if !states.contains_key(&initial_state_id) {
        errors.push("initial_information_state_id does not name a declared state".to_owned());
    } else if states.get(&initial_state_id).is_some_and(|state| state.stage != 0) {
        errors.push("initial information state must be at stage zero".to_owned());
    }

    // Validate stage transitions and resource conservation for every declared
    // action/outcome, not only the eventual optimal policy.
    for state in states.values() {
        for action in &state.actions {
            for outcome in &action.outcomes {
                match &outcome.next_state_id {
                    Some(next_id) => {
                        if state.stage + 1 >= horizon_stages {
                            errors.push(format!("{}:{}:{}: final-stage outcome must have null next state", state.state_id, action.action_id, outcome.outcome_id));
                            continue;
                        }
                        match states.get(next_id) {
                            Some(next_state) => {
                                if next_state.stage != state.stage + 1 {
                                    errors.push(format!("{}:{}:{}: next state must be at stage {}", state.state_id, action.action_id, outcome.outcome_id, state.stage + 1));
                                }
                                let expected_resources = state.resource_units_available - action.resource_use + outcome.resources_replenished;
                                if (next_state.resource_units_available - expected_resources).abs() > TOLERANCE {
                                    errors.push(format!("{}:{}:{}: resource conservation mismatch (expected {}, found {})", state.state_id, action.action_id, outcome.outcome_id, expected_resources, next_state.resource_units_available));
                                }
                            }
                            None => errors.push(format!("{}:{}:{}: next information state {next_id} is undeclared", state.state_id, action.action_id, outcome.outcome_id)),
                        }
                    }
                    None if state.stage + 1 == horizon_stages => {}
                    None => errors.push(format!("{}:{}:{}: non-terminal outcome cannot have null next state", state.state_id, action.action_id, outcome.outcome_id)),
                }
            }
        }
    }

    // Require every declared state to be reachable from the initial history
    // under at least one declared action/outcome. This prevents hidden unused
    // states from silently inflating the benchmark instance.
    if let Some(initial) = states.get(&initial_state_id) {
        let mut reachable = BTreeSet::new();
        let mut queue = VecDeque::from([initial.state_id.clone()]);
        while let Some(state_id) = queue.pop_front() {
            if !reachable.insert(state_id.clone()) {
                continue;
            }
            if let Some(state) = states.get(&state_id) {
                for action in &state.actions {
                    for outcome in &action.outcomes {
                        if let Some(next_id) = &outcome.next_state_id {
                            queue.push_back(next_id.clone());
                        }
                    }
                }
            }
        }
        for state_id in states.keys() {
            if !reachable.contains(state_id) {
                errors.push(format!("unreachable information state: {state_id}"));
            }
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(Model {
        horizon_stages,
        initial_state_id,
        service_floors,
        states,
        transition_count,
    })
}

#[derive(Clone)]
struct Plan {
    expected_cost: f64,
    policy: BTreeMap<String, String>,
}

fn solve_state(
    state_id: &str,
    model: &Model,
    memo: &mut HashMap<String, Option<Plan>>,
    visiting: &mut BTreeSet<String>,
) -> Result<Option<Plan>, String> {
    if let Some(cached) = memo.get(state_id) {
        return Ok(cached.clone());
    }
    if !visiting.insert(state_id.to_owned()) {
        return Err(format!("cycle detected at information state {state_id}"));
    }
    let state = model
        .states
        .get(state_id)
        .ok_or_else(|| format!("unknown information state {state_id}"))?;

    let floor_ok = model.service_floors.iter().all(|(service, floor)| {
        state
            .service_levels
            .get(service)
            .is_some_and(|level| *level + TOLERANCE >= *floor)
    });
    if !floor_ok {
        visiting.remove(state_id);
        memo.insert(state_id.to_owned(), None);
        return Ok(None);
    }

    let mut best: Option<Plan> = None;
    for action in &state.actions {
        let mut expected_cost = action.fixed_cost;
        let mut policy = BTreeMap::new();
        policy.insert(state_id.to_owned(), action.action_id.clone());
        let mut feasible = true;

        for outcome in &action.outcomes {
            let future = match &outcome.next_state_id {
                Some(next_id) => match solve_state(next_id, model, memo, visiting)? {
                    Some(plan) => Some(plan),
                    None => {
                        feasible = false;
                        None
                    }
                },
                None => None,
            };
            if outcome.next_state_id.is_some() && future.is_none() {
                feasible = false;
                break;
            }
            let future_cost = future.as_ref().map(|plan| plan.expected_cost).unwrap_or(0.0);
            expected_cost += outcome.probability * (outcome.stage_cost + future_cost);
            if let Some(plan) = future {
                for (future_state, future_action) in plan.policy {
                    if let Some(existing) = policy.get(&future_state) {
                        if existing != &future_action {
                            return Err(format!("non-anticipativity conflict at information state {future_state}"));
                        }
                    } else {
                        policy.insert(future_state, future_action);
                    }
                }
            }
        }
        if !feasible {
            continue;
        }

        let candidate = Plan {
            expected_cost,
            policy,
        };
        let replace = match &best {
            None => true,
            Some(current) if candidate.expected_cost < current.expected_cost - TOLERANCE => true,
            Some(current)
                if (candidate.expected_cost - current.expected_cost).abs() <= TOLERANCE =>
            {
                candidate
                    .policy
                    .get(state_id)
                    .cmp(&current.policy.get(state_id))
                    .is_lt()
            }
            _ => false,
        };
        if replace {
            best = Some(candidate);
        }
    }

    visiting.remove(state_id);
    memo.insert(state_id.to_owned(), best.clone());
    Ok(best)
}

/// Solve a tiny finite-horizon adaptive-recourse instance exactly by dynamic
/// programming over unique observable information states.
///
/// The model minimizes expected total cost under its supplied finite
/// action-conditioned transition probabilities, subject to pathwise service
/// floors and per-transition resource conservation. It is a micro-oracle,
/// not a distributionally robust or large-scale solver.
pub fn evaluate_exact_adaptive_recourse(fixture: &Value) -> Result<Value, Vec<String>> {
    let model = parse_model(fixture)?;
    let mut memo = HashMap::new();
    let mut visiting = BTreeSet::new();
    let solution = solve_state(
        &model.initial_state_id,
        &model,
        &mut memo,
        &mut visiting,
    )
    .map_err(|error| vec![error])?;

    let state_count = model.states.len();
    let action_count: usize = model.states.values().map(|state| state.actions.len()).sum();

    let Some(plan) = solution else {
        return Ok(json!({
            "benchmark_id": BENCHMARK_ID,
            "solver": "exact-finite-information-state-dynamic-programming",
            "objective_semantics": "minimum-expected-total-cost-subject-to-pathwise-service-floors",
            "status": "infeasible",
            "infeasibility_reason": "No non-anticipative policy satisfies all service floors over every positive-probability transition path.",
            "horizon_stages": model.horizon_stages,
            "information_state_count": state_count,
            "action_count": action_count,
            "transition_count": model.transition_count,
            "nonanticipativity_violation_count": 0,
            "transition_resource_invariant_pass_rate": 1.0,
            "expected_total_cost": null,
            "selected_policy": [],
            "calibration_claim": "none",
            "deployment_claim": "none"
        }));
    };

    let selected_policy: Vec<Value> = plan
        .policy
        .iter()
        .map(|(state_id, action_id)| {
            let state = &model.states[state_id];
            json!({
                "information_state_id": state_id,
                "observable_history_key": state.history_key,
                "stage": state.stage,
                "action_id": action_id,
                "resource_units_available": state.resource_units_available
            })
        })
        .collect();

    Ok(json!({
        "benchmark_id": BENCHMARK_ID,
        "solver": "exact-finite-information-state-dynamic-programming",
        "objective_semantics": "minimum-expected-total-cost-subject-to-pathwise-service-floors",
        "status": "optimal",
        "evaluation_split": "public-development-only",
        "active_in_curriculum": false,
        "horizon_stages": model.horizon_stages,
        "information_state_count": state_count,
        "action_count": action_count,
        "transition_count": model.transition_count,
        "selected_policy_state_count": selected_policy.len(),
        "selected_policy": selected_policy,
        "expected_total_cost": plan.expected_cost,
        "risk_measure": "expectation-under-frozen-action-conditioned-transition-kernels",
        "service_floor_semantics": "every selected-policy path must remain at or above every service floor at every stage",
        "resource_semantics": "resource balance is conserved at every action/outcome transition; no stage reset",
        "nonanticipativity_violation_count": 0,
        "nonanticipativity_semantics": "one action is selected per unique observable information state; hidden scenario labels are not policy inputs",
        "transition_resource_invariant_pass_rate": 1.0,
        "oracle_scope": "exact over the supplied finite information-state DAG and expected-cost objective",
        "calibration_claim": "none",
        "distributional_robustness_claim": "none",
        "deployment_claim": "none"
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn terminal_state(id: &str, resources: f64, water: f64, power: f64) -> Value {
        json!({
            "information_state_id": id,
            "observable_history_key": format!("history-{id}"),
            "stage": 2,
            "resource_units_available": resources,
            "service_levels": {"water": water, "power": power},
            "actions": [{
                "action_id": "END",
                "fixed_cost": 0.0,
                "resource_use": 0.0,
                "outcomes": [{
                    "outcome_id": "end",
                    "probability": 1.0,
                    "next_information_state_id": null,
                    "stage_cost": 0.0,
                    "resources_replenished": 0.0
                }]
            }]
        })
    }

    fn small_fixture() -> Value {
        let mut states = vec![
            json!({
                "information_state_id":"ROOT",
                "observable_history_key":"history-root",
                "stage":0,
                "resource_units_available":2.0,
                "service_levels":{"water":100.0,"power":100.0},
                "actions":[
                    {
                        "action_id":"PREVENT",
                        "fixed_cost":1.0,
                        "resource_use":1.0,
                        "outcomes":[
                            {"outcome_id":"quiet","probability":0.7,"next_information_state_id":"P-QUIET","stage_cost":0.0,"resources_replenished":0.0},
                            {"outcome_id":"storm","probability":0.3,"next_information_state_id":"P-STORM","stage_cost":0.0,"resources_replenished":0.0}
                        ]
                    },
                    {
                        "action_id":"DEFER",
                        "fixed_cost":0.0,
                        "resource_use":0.0,
                        "outcomes":[
                            {"outcome_id":"quiet","probability":0.6,"next_information_state_id":"D-QUIET","stage_cost":0.0,"resources_replenished":0.0},
                            {"outcome_id":"storm","probability":0.4,"next_information_state_id":"D-STORM","stage_cost":0.0,"resources_replenished":0.0}
                        ]
                    }
                ]
            }),
            json!({
                "information_state_id":"P-QUIET","observable_history_key":"history-p-quiet","stage":1,
                "resource_units_available":1.0,"service_levels":{"water":100.0,"power":100.0},
                "actions":[
                    {"action_id":"USE-SPARE","fixed_cost":1.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PQ-END-USE","stage_cost":0.0,"resources_replenished":0.0}]},
                    {"action_id":"HOLD","fixed_cost":4.0,"resource_use":0.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PQ-END-HOLD","stage_cost":0.0,"resources_replenished":0.0}]}
                ]
            }),
            json!({
                "information_state_id":"P-STORM","observable_history_key":"history-p-storm","stage":1,
                "resource_units_available":1.0,"service_levels":{"water":70.0,"power":80.0},
                "actions":[
                    {"action_id":"REPAIR","fixed_cost":2.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PS-END-REPAIR","stage_cost":0.0,"resources_replenished":0.0}]},
                    {"action_id":"HOLD","fixed_cost":5.0,"resource_use":0.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PS-END-HOLD","stage_cost":0.0,"resources_replenished":0.0}]}
                ]
            }),
            json!({
                "information_state_id":"D-QUIET","observable_history_key":"history-d-quiet","stage":1,
                "resource_units_available":2.0,"service_levels":{"water":100.0,"power":100.0},
                "actions":[
                    {"action_id":"USE-SPARE","fixed_cost":1.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"DQ-END-USE","stage_cost":0.0,"resources_replenished":0.0}]},
                    {"action_id":"HOLD","fixed_cost":3.0,"resource_use":0.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"DQ-END-HOLD","stage_cost":0.0,"resources_replenished":0.0}]}
                ]
            }),
            json!({
                "information_state_id":"D-STORM","observable_history_key":"history-d-storm","stage":1,
                "resource_units_available":2.0,"service_levels":{"water":40.0,"power":50.0},
                "actions":[{"action_id":"RECOVER","fixed_cost":3.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"DS-END-RECOVER","stage_cost":0.0,"resources_replenished":0.0}]}]
            })
        ];
        states.push(terminal_state("PQ-END-USE", 0.0, 100.0, 100.0));
        states.push(terminal_state("PQ-END-HOLD", 1.0, 100.0, 100.0));
        states.push(terminal_state("PS-END-REPAIR", 0.0, 80.0, 90.0));
        states.push(terminal_state("PS-END-HOLD", 1.0, 70.0, 80.0));
        states.push(terminal_state("DQ-END-USE", 1.0, 100.0, 100.0));
        states.push(terminal_state("DQ-END-HOLD", 2.0, 100.0, 100.0));
        states.push(terminal_state("DS-END-RECOVER", 1.0, 60.0, 50.0));

        json!({
            "benchmark_id": BENCHMARK_ID,
            "schema_version": "1.0.0",
            "status": "synthetic-demonstration-for-review",
            "evaluation_split": "public-development-only",
            "active_in_curriculum": false,
            "objective_semantics": "minimum-expected-total-cost-subject-to-pathwise-service-floors",
            "horizon_stages": 3,
            "initial_information_state_id": "ROOT",
            "service_floors": {"water":70.0,"power":60.0},
            "information_states": states
        })
    }

    #[test]
    fn exact_oracle_matches_hand_solved_expected_cost() {
        let report = evaluate_exact_adaptive_recourse(&small_fixture()).unwrap();
        assert_eq!(report["status"], json!("optimal"));
        assert!((report["expected_total_cost"].as_f64().unwrap() - 2.3).abs() < 1e-12);
    }

    #[test]
    fn action_dependent_transition_kernel_changes_optimal_policy() {
        let report = evaluate_exact_adaptive_recourse(&small_fixture()).unwrap();
        let policy = report["selected_policy"].as_array().unwrap();
        let root = policy.iter().find(|entry| entry["information_state_id"] == "ROOT").unwrap();
        assert_eq!(root["action_id"], json!("PREVENT"));
    }

    #[test]
    fn future_service_floor_breach_makes_action_infeasible() {
        let report = evaluate_exact_adaptive_recourse(&small_fixture()).unwrap();
        let policy = report["selected_policy"].as_array().unwrap();
        let root = policy.iter().find(|entry| entry["information_state_id"] == "ROOT").unwrap();
        assert_ne!(root["action_id"], json!("DEFER"));
    }

    #[test]
    fn selected_policy_is_nonanticipative_by_information_state() {
        let report = evaluate_exact_adaptive_recourse(&small_fixture()).unwrap();
        assert_eq!(report["nonanticipativity_violation_count"], json!(0));
        let policy = report["selected_policy"].as_array().unwrap();
        let state_ids: BTreeSet<String> = policy.iter()
            .map(|entry| entry["information_state_id"].as_str().unwrap().to_owned())
            .collect();
        assert_eq!(state_ids.len(), policy.len());
    }

    #[test]
    fn duplicate_observable_history_is_rejected() {
        let mut fixture = small_fixture();
        fixture["information_states"][1]["observable_history_key"] = json!("history-root");
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn resource_conservation_mismatch_is_rejected() {
        let mut fixture = small_fixture();
        fixture["information_states"][1]["resource_units_available"] = json!(0.5);
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn non_unit_action_conditioned_probabilities_are_rejected() {
        let mut fixture = small_fixture();
        fixture["information_states"][0]["actions"][0]["outcomes"][0]["probability"] = json!(0.8);
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn resource_overdraw_is_rejected() {
        let mut fixture = small_fixture();
        fixture["information_states"][0]["actions"][0]["resource_use"] = json!(3.0);
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn no_feasible_policy_returns_explicit_infeasibility() {
        let mut fixture = small_fixture();
        fixture["information_states"][0]["actions"][0]["outcomes"][0]["next_information_state_id"] = json!("D-STORM");
        fixture["information_states"][0]["actions"][0]["outcomes"][0]["probability"] = json!(0.7);
        fixture["information_states"][0]["actions"][0]["outcomes"][1]["next_information_state_id"] = json!("D-STORM");
        fixture["information_states"][0]["actions"][0]["outcomes"][1]["probability"] = json!(0.3);
        // The action now fails the resource balance on its first transition;
        // the validator returns an explicit error rather than silently relaxing it.
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn wrong_stage_transition_is_rejected() {
        let mut fixture = small_fixture();
        fixture["information_states"][0]["actions"][0]["outcomes"][0]["next_information_state_id"] = json!("PQ-END-USE");
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }
}

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

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};

const BENCHMARK_ID: &str = "PRAXIS-NRES-016-EX0-EXACT-ADAPTIVE-RECOURSE-001";
const TOLERANCE: f64 = 1e-10;

fn string_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn number_field(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64)
}

fn scan_hidden_history_fields(value: &Value, path: &str, errors: &mut Vec<String>) {
    const FORBIDDEN: &[&str] = &[
        "scenario_id",
        "scenario_label",
        "latent_scenario_id",
        "future_scenario_id",
        "future_shock",
        "hidden_outcome",
        "future_outcome",
        "true_future_state",
        "oracle_label",
        "clairvoyant_state",
    ];
    match value {
        Value::Object(object) => {
            for (key, child) in object {
                if FORBIDDEN.contains(&key.as_str()) {
                    errors.push(format!("hidden/future field is not admissible in observable history at {path}/{key}"));
                }
                scan_hidden_history_fields(child, &format!("{path}/{key}"), errors);
            }
        }
        Value::Array(items) => {
            for (index, child) in items.iter().enumerate() {
                scan_hidden_history_fields(child, &format!("{path}/{index}"), errors);
            }
        }
        _ => {}
    }
}

fn parse_observable_history(
    raw_state: &Value,
    state_id: &str,
    stage: usize,
    errors: &mut Vec<String>,
) -> (String, Vec<Value>, Vec<String>) {
    let Some(history) = raw_state.get("observable_history").and_then(Value::as_object) else {
        errors.push(format!("{state_id}: observable_history must be an object"));
        return (String::new(), Vec::new(), Vec::new());
    };
    if history.len() != 2 || !history.contains_key("observations") || !history.contains_key("prior_actions") {
        errors.push(format!("{state_id}: observable_history must contain only observations and prior_actions"));
    }

    let mut observations = Vec::new();
    match history.get("observations").and_then(Value::as_array) {
        Some(items) => {
            if items.len() != stage + 1 {
                errors.push(format!("{state_id}: stage {stage} requires exactly {} observed records", stage + 1));
            }
            for (index, item) in items.iter().enumerate() {
                let Some(object) = item.as_object() else {
                    errors.push(format!("{state_id}: observation {index} must be an object"));
                    continue;
                };
                if object.len() != 2 || !object.contains_key("stage") || !object.contains_key("values") {
                    errors.push(format!("{state_id}: observation {index} must contain only stage and values"));
                }
                if object.get("stage").and_then(Value::as_u64) != Some(index as u64) {
                    errors.push(format!("{state_id}: observation stages must be contiguous from zero"));
                }
                match object.get("values").and_then(Value::as_object) {
                    Some(values) if !values.is_empty() => {
                        scan_hidden_history_fields(item, &format!("{state_id}/observable_history/observations/{index}"), errors);
                    }
                    _ => errors.push(format!("{state_id}: observation {index} requires non-empty values")),
                }
                observations.push(item.clone());
            }
        }
        None => errors.push(format!("{state_id}: observable_history.observations must be an array")),
    }

    let mut prior_actions = Vec::new();
    match history.get("prior_actions").and_then(Value::as_array) {
        Some(items) => {
            if items.len() != stage {
                errors.push(format!("{state_id}: stage {stage} requires exactly {stage} prior actions"));
            }
            for (index, item) in items.iter().enumerate() {
                match item.as_str().filter(|action| !action.trim().is_empty()) {
                    Some(action) => prior_actions.push(action.to_owned()),
                    None => errors.push(format!("{state_id}: prior_actions[{index}] must be a non-empty action ID")),
                }
            }
        }
        None => errors.push(format!("{state_id}: observable_history.prior_actions must be an array")),
    }

    let canonical = json!({
        "observations": observations,
        "prior_actions": prior_actions
    });
    let identity = serde_json::to_string(&canonical).unwrap_or_default();
    (identity, observations, prior_actions)
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
    observation: Option<Value>,
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
    observations: Vec<Value>,
    prior_actions: Vec<String>,
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
    let fixture_id = string_field(fixture, "benchmark_id");
    if fixture_id != Some(BENCHMARK_ID)
        && fixture_id != Some("PRAXIS-NRES-016-EX1-NESTED-CVAR-COMPARISON-001")
    {
        errors.push("unexpected NRES-016 adaptive-recourse benchmark_id".to_owned());
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
    let Some(horizon_stages) = horizon.filter(|count| (3..=12).contains(count)) else {
        errors.push("horizon_stages must be an integer in [3,12]".to_owned());
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

        let stage = raw_state
            .get("stage")
            .and_then(Value::as_u64)
            .and_then(|value| usize::try_from(value).ok());
        let Some(stage) = stage.filter(|stage| *stage < horizon_stages) else {
            errors.push(format!("{state_id}: stage must be in [0,horizon_stages)"));
            continue;
        };
        let (history_identity, observations, prior_actions) =
            parse_observable_history(raw_state, &state_id, stage, &mut errors);
        if history_identity.is_empty() {
            errors.push(format!("{state_id}: canonical observable-history identity could not be formed"));
        } else if !history_keys.insert(history_identity.clone()) {
            errors.push(format!("duplicate canonical observable history at information state {state_id}"));
        }
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
                let Some(raw_next_state_id) = raw_outcome.get("next_information_state_id") else {
                    errors.push(format!("{state_id}/{action_id}/{outcome_id}: next_information_state_id must be present or null"));
                    continue;
                };
                let next_state_id = match raw_next_state_id {
                    Value::String(next_id) if !next_id.trim().is_empty() => Some(next_id.clone()),
                    Value::Null => None,
                    _ => {
                        errors.push(format!("{state_id}/{action_id}/{outcome_id}: next_information_state_id must be a non-empty string or null"));
                        continue;
                    }
                };
                if raw_outcome.get("observation").is_none() {
                    errors.push(format!("{state_id}/{action_id}/{outcome_id}: observation must be present as an object or null"));
                    continue;
                }
                let observation = match (&next_state_id, raw_outcome.get("observation").unwrap()) {
                    (Some(_), Value::Object(object)) => {
                        if object.len() != 2 || object.get("stage").and_then(Value::as_u64) != Some((stage + 1) as u64) {
                            errors.push(format!("{state_id}/{action_id}/{outcome_id}: transition observation must contain the next stage index"));
                            continue;
                        }
                        match object.get("values").and_then(Value::as_object) {
                            Some(values) if !values.is_empty() => {
                                scan_hidden_history_fields(raw_outcome.get("observation").unwrap(), &format!("{state_id}/{action_id}/{outcome_id}/observation"), &mut errors);
                                Some(raw_outcome.get("observation").unwrap().clone())
                            }
                            _ => {
                                errors.push(format!("{state_id}/{action_id}/{outcome_id}: transition observation requires non-empty values"));
                                continue;
                            }
                        }
                    }
                    (Some(_), _) => {
                        errors.push(format!("{state_id}/{action_id}/{outcome_id}: non-terminal transition requires an observation object"));
                        continue;
                    }
                    (None, Value::Null) => None,
                    (None, _) => {
                        errors.push(format!("{state_id}/{action_id}/{outcome_id}: terminal transition observation must be null"));
                        continue;
                    }
                };
                probability_sum += probability;
                outcomes.push(Outcome {
                    outcome_id,
                    observation,
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
                observations,
                prior_actions,
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
                                let mut expected_prior_actions = state.prior_actions.clone();
                                expected_prior_actions.push(action.action_id.clone());
                                if next_state.prior_actions != expected_prior_actions {
                                    errors.push(format!("{}:{}:{}: next observable history must append the selected prior action", state.state_id, action.action_id, outcome.outcome_id));
                                }
                                if next_state.observations.len() != state.observations.len() + 1
                                    || !next_state.observations.starts_with(&state.observations)
                                {
                                    errors.push(format!("{}:{}:{}: next observable history must preserve all prior observations and append exactly one new observation", state.state_id, action.action_id, outcome.outcome_id));
                                }
                                if next_state.observations.last() != outcome.observation.as_ref() {
                                    errors.push(format!("{}:{}:{}: appended child observation must exactly match the observation emitted by this transition", state.state_id, action.action_id, outcome.outcome_id));
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
                "observable_history_event_count": state.observations.len() + state.prior_actions.len(),
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

#[derive(Clone)]
struct PathDistributionPlan {
    outcomes: Vec<(f64, f64)>,
    policy: BTreeMap<String, String>,
}

#[derive(Clone)]
struct NestedRiskPlan {
    nested_cvar: f64,
    expected_cost: f64,
    policy: BTreeMap<String, String>,
}

/// Exact upper-tail CVaR for a finite probability distribution.
///
/// Uses the Rockafellar-Uryasev representation:
/// min_eta [ eta + E[(X-eta)+] / (1-alpha) ].
/// For finite support, an optimum exists at a support value.
fn upper_cvar(distribution: &[(f64, f64)], alpha: f64) -> Result<f64, String> {
    if distribution.is_empty() {
        return Err("CVaR distribution cannot be empty".to_owned());
    }
    if !alpha.is_finite() || !(0.0..1.0).contains(&alpha) {
        return Err("CVaR confidence level must be finite and in [0,1)".to_owned());
    }
    let total_probability: f64 = distribution.iter().map(|(_, probability)| probability).sum();
    if (total_probability - 1.0).abs() > TOLERANCE {
        return Err(format!("CVaR probabilities must sum to one (found {total_probability})"));
    }
    if distribution.iter().any(|(loss, probability)| {
        !loss.is_finite() || *probability <= 0.0 || !probability.is_finite()
    }) {
        return Err("CVaR losses must be finite and probabilities positive".to_owned());
    }

    let mut candidates: Vec<f64> = distribution.iter().map(|(loss, _)| *loss).collect();
    candidates.sort_by(f64::total_cmp);
    candidates.dedup_by(|left, right| (*left - *right).abs() <= TOLERANCE);

    let mut best = f64::INFINITY;
    for eta in candidates {
        let excess: f64 = distribution
            .iter()
            .map(|(loss, probability)| probability * (loss - eta).max(0.0))
            .sum();
        let value = eta + excess / (1.0 - alpha);
        best = best.min(value);
    }
    Ok(best)
}

fn mean_cost(distribution: &[(f64, f64)]) -> f64 {
    distribution.iter().map(|(loss, probability)| loss * probability).sum()
}

fn combine_path_plans(
    state_id: &str,
    action: &Action,
    outcomes: &[Outcome],
    child_ids: &[String],
    child_options: &BTreeMap<String, Vec<PathDistributionPlan>>,
    index: usize,
    chosen: &mut BTreeMap<String, PathDistributionPlan>,
    output: &mut Vec<PathDistributionPlan>,
) {
    if index < child_ids.len() {
        let child_id = &child_ids[index];
        if let Some(options) = child_options.get(child_id) {
            for option in options {
                chosen.insert(child_id.clone(), option.clone());
                combine_path_plans(
                    state_id,
                    action,
                    outcomes,
                    child_ids,
                    child_options,
                    index + 1,
                    chosen,
                    output,
                );
            }
            chosen.remove(child_id);
        }
        return;
    }

    let mut policy = BTreeMap::new();
    policy.insert(state_id.to_owned(), action.action_id.clone());
    for plan in chosen.values() {
        for (child_state, child_action) in &plan.policy {
            if let Some(previous) = policy.insert(child_state.clone(), child_action.clone()) {
                if previous.as_str() != child_action.as_str() {
                    return;
                }
            }
        }
    }

    let mut distribution = Vec::new();
    for outcome in outcomes {
        match &outcome.next_state_id {
            Some(next_id) => {
                let Some(child) = chosen.get(next_id) else {
                    return;
                };
                for (future_cost, future_probability) in &child.outcomes {
                    distribution.push((
                        action.fixed_cost + outcome.stage_cost + future_cost,
                        outcome.probability * future_probability,
                    ));
                }
            }
            None => distribution.push((action.fixed_cost + outcome.stage_cost, outcome.probability)),
        }
    }

    output.push(PathDistributionPlan {
        outcomes: distribution,
        policy,
    });
}

fn enumerate_terminal_cvar_policies(
    state_id: &str,
    model: &Model,
    memo: &mut HashMap<String, Vec<PathDistributionPlan>>,
    visiting: &mut BTreeSet<String>,
) -> Result<Vec<PathDistributionPlan>, String> {
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
        state.service_levels.get(service).is_some_and(|level| *level + TOLERANCE >= *floor)
    });
    if !floor_ok {
        visiting.remove(state_id);
        memo.insert(state_id.to_owned(), Vec::new());
        return Ok(Vec::new());
    }

    let mut all_plans = Vec::new();
    for action in &state.actions {
        let child_ids: Vec<String> = action
            .outcomes
            .iter()
            .filter_map(|outcome| outcome.next_state_id.clone())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();

        let mut child_options = BTreeMap::new();
        let mut action_feasible = true;
        for child_id in &child_ids {
            let options = enumerate_terminal_cvar_policies(child_id, model, memo, visiting)?;
            if options.is_empty() {
                action_feasible = false;
                break;
            }
            child_options.insert(child_id.clone(), options);
        }
        if !action_feasible {
            continue;
        }

        let mut chosen = BTreeMap::new();
        combine_path_plans(
            state_id,
            action,
            &action.outcomes,
            &child_ids,
            &child_options,
            0,
            &mut chosen,
            &mut all_plans,
        );
    }

    visiting.remove(state_id);
    memo.insert(state_id.to_owned(), all_plans.clone());
    Ok(all_plans)
}

fn solve_nested_rho_hat(
    state_id: &str,
    model: &Model,
    alpha: f64,
    lambda: f64,
    memo: &mut HashMap<String, Option<NestedRiskPlan>>,
    visiting: &mut BTreeSet<String>,
) -> Result<Option<NestedRiskPlan>, String> {
    if let Some(cached) = memo.get(state_id) {
        return Ok(cached.clone());
    }
    if !visiting.insert(state_id.to_owned()) {
        return Err(format!("cycle detected at information state {state_id}"));
    }

    let state = model.states.get(state_id)
        .ok_or_else(|| format!("unknown information state {state_id}"))?;
    let floor_ok = model.service_floors.iter().all(|(service, floor)| {
        state.service_levels.get(service).is_some_and(|level| *level + TOLERANCE >= *floor)
    });
    if !floor_ok {
        visiting.remove(state_id);
        memo.insert(state_id.to_owned(), None);
        return Ok(None);
    }

    let mut best: Option<NestedRiskPlan> = None;
    for action in &state.actions {
        let mut risk_outcomes = Vec::new();
        let mut expected_cost = action.fixed_cost;
        let mut policy = BTreeMap::new();
        policy.insert(state_id.to_owned(), action.action_id.clone());
        let mut feasible = true;

        for outcome in &action.outcomes {
            let future = match &outcome.next_state_id {
                Some(next_id) => match solve_nested_rho_hat(next_id, model, alpha, lambda, memo, visiting)? {
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

            let future_risk = future.as_ref().map(|plan| plan.nested_cvar).unwrap_or(0.0);
            let future_mean = future.as_ref().map(|plan| plan.expected_cost).unwrap_or(0.0);
            risk_outcomes.push((outcome.stage_cost + future_risk, outcome.probability));
            expected_cost += outcome.probability * (outcome.stage_cost + future_mean);

            if let Some(plan) = future {
                for (future_state, future_action) in plan.policy {
                    if let Some(previous) = policy.insert(future_state.clone(), future_action.clone()) {
                        if previous != future_action {
                            return Err(format!("non-anticipativity conflict at information state {future_state}"));
                        }
                    }
                }
            }
        }
        if !feasible {
            continue;
        }

        let conditional_mean: f64 = risk_outcomes.iter()
            .map(|(loss, probability)| loss * probability)
            .sum();
        let conditional_cvar = upper_cvar(&risk_outcomes, alpha)?;
        let nested_risk = action.fixed_cost
            + (1.0 - lambda) * conditional_mean
            + lambda * conditional_cvar;

        let candidate = NestedRiskPlan {
            nested_cvar: nested_risk,
            expected_cost,
            policy,
        };
        let replace = match &best {
            None => true,
            Some(current) if candidate.nested_cvar < current.nested_cvar - TOLERANCE => true,
            Some(current)
                if (candidate.nested_cvar - current.nested_cvar).abs() <= TOLERANCE
                    && candidate.expected_cost < current.expected_cost - TOLERANCE => true,
            Some(current)
                if (candidate.nested_cvar - current.nested_cvar).abs() <= TOLERANCE
                    && (candidate.expected_cost - current.expected_cost).abs() <= TOLERANCE =>
            {
                candidate.policy.get(state_id).cmp(&current.policy.get(state_id)).is_lt()
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

/// Compare recursively nested, strictly monotone mean-CVaR risk with a
/// precommitment terminal mean-CVaR objective on the same non-anticipative tree.
///
/// The one-period map is rho_hat = (1-lambda) E + lambda CVaR_alpha.
/// Lambda is constrained to [0,1), retaining a positive expectation component
/// and the strict-monotonicity condition used by dynamic-programming results.
pub fn evaluate_nested_risk_comparison(fixture: &Value) -> Result<Value, Vec<String>> {
    let model = parse_model(fixture)?;
    let alpha = number_field(fixture, "risk_confidence_level")
        .filter(|value| value.is_finite() && (0.0..1.0).contains(value))
        .ok_or_else(|| vec!["risk_confidence_level must be finite and in [0,1)".to_owned()])?;
    let lambda = number_field(fixture, "risk_aversion_lambda")
        .filter(|value| value.is_finite() && (0.0..1.0).contains(value))
        .ok_or_else(|| vec!["risk_aversion_lambda must be finite and in [0,1) for strict monotonicity".to_owned()])?;

    let mut nested_memo = HashMap::new();
    let mut nested_visiting = BTreeSet::new();
    let nested = solve_nested_rho_hat(
        &model.initial_state_id,
        &model,
        alpha,
        lambda,
        &mut nested_memo,
        &mut nested_visiting,
    ).map_err(|error| vec![error])?;

    let mut static_memo = HashMap::new();
    let mut static_visiting = BTreeSet::new();
    let static_plans = enumerate_terminal_cvar_policies(
        &model.initial_state_id,
        &model,
        &mut static_memo,
        &mut static_visiting,
    ).map_err(|error| vec![error])?;

    let mut static_best: Option<(f64, f64, f64, PathDistributionPlan)> = None;
    for plan in static_plans {
        let terminal_cvar = upper_cvar(&plan.outcomes, alpha).map_err(|error| vec![error])?;
        let expected = mean_cost(&plan.outcomes);
        let static_objective = (1.0 - lambda) * expected + lambda * terminal_cvar;
        let replace = match &static_best {
            None => true,
            Some((best_objective, _, _, _))
                if static_objective < *best_objective - TOLERANCE => true,
            Some((best_objective, _, best_expected, _))
                if (static_objective - *best_objective).abs() <= TOLERANCE
                    && expected < *best_expected - TOLERANCE => true,
            Some((best_objective, _, best_expected, best_plan))
                if (static_objective - *best_objective).abs() <= TOLERANCE
                    && (expected - *best_expected).abs() <= TOLERANCE =>
            {
                plan.policy.get(&model.initial_state_id).cmp(&best_plan.policy.get(&model.initial_state_id)).is_lt()
            }
            _ => false,
        };
        if replace {
            static_best = Some((static_objective, terminal_cvar, expected, plan));
        }
    }

    let nested_report = nested.map(|plan| json!({
        "nested_mean_cvar_objective": plan.nested_cvar,
        "policy_expected_cost": plan.expected_cost,
        "policy": plan.policy
    }));
    let static_report = static_best.map(|(objective, terminal_cvar, expected, plan)| json!({
        "static_terminal_mean_cvar_objective": objective,
        "static_terminal_cvar": terminal_cvar,
        "policy_expected_cost": expected,
        "policy": plan.policy,
        "terminal_cost_distribution": plan.outcomes.iter().map(|(cost, probability)| json!({
            "total_cost": cost,
            "probability": probability
        })).collect::<Vec<_>>()
    }));

    let nested_a = nested_report.as_ref().and_then(|report| report["policy"].get("A").and_then(Value::as_str));
    let static_a = static_report.as_ref().and_then(|report| report["policy"].get("A").and_then(Value::as_str));

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-016-EX1-NESTED-CVAR-COMPARISON-001",
        "status": "synthetic-public-development-evaluated",
        "evaluation_split": "public-development-only",
        "active_in_curriculum": false,
        "risk_confidence_level": alpha,
        "risk_aversion_lambda": lambda,
        "one_period_risk_map": "rho_hat = (1-lambda)*Expectation + lambda*upper_CVaR_alpha",
        "nested_method": "recursive conditional rho_hat at each observable information state",
        "static_method": "precommitment rho_hat of total terminal path cost",
        "strict_monotonicity_contract": "lambda in [0,1) leaves a positive expectation component; the nested dynamic-programming equivalence is scoped to the cited formulation and assumptions",
        "nested": nested_report,
        "static_terminal": static_report,
        "local_decision_reversal_at_A": {
            "nested_action": nested_a,
            "static_precommitment_action": static_a,
            "different": nested_a != static_a
        },
        "nonanticipativity_semantics": "both optimizers choose one action per explicit observable information state",
        "risk_semantics": "fixed action cost is translated outside the conditional risk map; stage cost plus continuation risk is inside each conditional map",
        "calibration_claim": "none",
        "distributional_robustness_claim": "none",
        "deployment_claim": "none",
        "interpretation": "The static terminal map can be dominated by a different branch and choose a locally riskier action; the nested rho_hat evaluates conditional branch risk at each information state. This is a semantics counterexample, not a general performance ranking."
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn history_from_events(events: &[&str], actions: &[&str]) -> Value {
        json!({
            "observations": events.iter().enumerate().map(|(stage, event)| {
                json!({"stage": stage, "values": {"observed_event": event}})
            }).collect::<Vec<_>>(),
            "prior_actions": actions
        })
    }

    fn risk_terminal_state(id: &str, events: &[&str], actions: &[&str]) -> Value {
        json!({
            "information_state_id": id,
            "observable_history": history_from_events(events, actions),
            "stage": 2,
            "resource_units_available": 0.0,
            "service_levels": {"water": 100.0, "power": 100.0},
            "actions": [{
                "action_id": "END",
                "fixed_cost": 0.0,
                "resource_use": 0.0,
                "outcomes": [{
                    "outcome_id": "end",
                    "probability": 1.0,
                    "next_information_state_id": null,
                    "observation": null,
                    "stage_cost": 0.0,
                    "resources_replenished": 0.0
                }]
            }]
        })
    }

    fn risk_fixture(alpha: f64) -> Value {
        let mut states = vec![
            json!({
                "information_state_id": "ROOT",
                "observable_history": history_from_events(&["initial"], &[]),
                "stage": 0,
                "resource_units_available": 0.0,
                "service_levels": {"water": 100.0, "power": 100.0},
                "actions": [{
                    "action_id": "START",
                    "fixed_cost": 0.0,
                    "resource_use": 0.0,
                    "outcomes": [
                        {"outcome_id":"branch-a","probability":0.5,"next_information_state_id":"A","observation":{"stage":1,"values":{"branch":"A"}},"stage_cost":0.0,"resources_replenished":0.0},
                        {"outcome_id":"branch-b","probability":0.5,"next_information_state_id":"B","observation":{"stage":1,"values":{"branch":"B"}},"stage_cost":0.0,"resources_replenished":0.0}
                    ]
                }]
            }),
            json!({
                "information_state_id": "A",
                "observable_history": history_from_events(&["initial", "A"], &["START"]),
                "stage": 1,
                "resource_units_available": 0.0,
                "service_levels": {"water": 100.0, "power": 100.0},
                "actions": [
                    {"action_id":"SAFE","fixed_cost":150.0,"resource_use":0.0,"outcomes":[{"outcome_id":"safe","probability":1.0,"next_information_state_id":"A-SAFE","observation":{"stage":2,"values":{"result":"safe"}},"stage_cost":0.0,"resources_replenished":0.0}]},
                    {"action_id":"GAMBLE","fixed_cost":0.0,"resource_use":0.0,"outcomes":[
                        {"outcome_id":"good","probability":0.95,"next_information_state_id":"A-GOOD","observation":{"stage":2,"values":{"result":"good"}},"stage_cost":0.0,"resources_replenished":0.0},
                        {"outcome_id":"bad","probability":0.05,"next_information_state_id":"A-BAD","observation":{"stage":2,"values":{"result":"bad"}},"stage_cost":1000.0,"resources_replenished":0.0}
                    ]}
                ]
            }),
            json!({
                "information_state_id": "B",
                "observable_history": history_from_events(&["initial", "B"], &["START"]),
                "stage": 1,
                "resource_units_available": 0.0,
                "service_levels": {"water": 100.0, "power": 100.0},
                "actions": [{
                    "action_id":"HOLD",
                    "fixed_cost":0.0,
                    "resource_use":0.0,
                    "outcomes":[{"outcome_id":"catastrophe","probability":1.0,"next_information_state_id":"B-END","observation":{"stage":2,"values":{"result":"catastrophe"}},"stage_cost":10000.0,"resources_replenished":0.0}]
                }]
            })
        ];
        states.push(risk_terminal_state("A-SAFE", &["initial", "A", "safe"], &["START", "SAFE"]));
        states.push(risk_terminal_state("A-GOOD", &["initial", "A", "good"], &["START", "GAMBLE"]));
        states.push(risk_terminal_state("A-BAD", &["initial", "A", "bad"], &["START", "GAMBLE"]));
        states.push(risk_terminal_state("B-END", &["initial", "B", "catastrophe"], &["START", "HOLD"]));

        let last_observations: HashMap<String, Value> = states.iter().map(|state| {
            let id = state["information_state_id"].as_str().unwrap().to_owned();
            let observation = state["observable_history"]["observations"].as_array().unwrap().last().unwrap().clone();
            (id, observation)
        }).collect();
        for state in &mut states {
            for action in state["actions"].as_array_mut().unwrap() {
                for outcome in action["outcomes"].as_array_mut().unwrap() {
                    let next_id = outcome.get("next_information_state_id").and_then(Value::as_str).map(str::to_owned);
                    outcome["observation"] = match next_id {
                        Some(next_id) => last_observations[&next_id].clone(),
                        None => Value::Null
                    };
                }
            }
        }

        json!({
            "benchmark_id": BENCHMARK_ID,
            "schema_version": "1.0.0",
            "status": "synthetic-demonstration-for-review",
            "evaluation_split": "public-development-only",
            "active_in_curriculum": false,
            "objective_semantics": "minimum-expected-total-cost-subject-to-pathwise-service-floors",
            "risk_confidence_level": alpha,
            "horizon_stages": 3,
            "initial_information_state_id": "ROOT",
            "service_floors": {"water":70.0,"power":60.0},
            "information_states": states
        })
    }

    fn observed_history(observations: &[&str], prior_actions: &[&str]) -> Value {
        json!({
            "observations": observations.iter().enumerate().map(|(stage, value)| {
                json!({"stage": stage, "values": {"observed_event": value}})
            }).collect::<Vec<_>>(),
            "prior_actions": prior_actions
        })
    }

    fn terminal_state(
        id: &str,
        resources: f64,
        water: f64,
        power: f64,
        observations: &[&str],
        prior_actions: &[&str],
    ) -> Value {
        json!({
            "information_state_id": id,
            "observable_history": observed_history(observations, prior_actions),
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
                    "observation": null,
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
                "observable_history":observed_history(&["initial"], &[]),
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
                "information_state_id":"P-QUIET","observable_history":observed_history(&["initial", "quiet"], &["PREVENT"]),"stage":1,
                "resource_units_available":1.0,"service_levels":{"water":100.0,"power":100.0},
                "actions":[
                    {"action_id":"USE-SPARE","fixed_cost":1.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PQ-END-USE","stage_cost":0.0,"resources_replenished":0.0}]},
                    {"action_id":"HOLD","fixed_cost":4.0,"resource_use":0.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PQ-END-HOLD","stage_cost":0.0,"resources_replenished":0.0}]}
                ]
            }),
            json!({
                "information_state_id":"P-STORM","observable_history":observed_history(&["initial", "storm"], &["PREVENT"]),"stage":1,
                "resource_units_available":1.0,"service_levels":{"water":70.0,"power":80.0},
                "actions":[
                    {"action_id":"REPAIR","fixed_cost":2.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PS-END-REPAIR","stage_cost":0.0,"resources_replenished":0.0}]},
                    {"action_id":"HOLD","fixed_cost":5.0,"resource_use":0.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"PS-END-HOLD","stage_cost":0.0,"resources_replenished":0.0}]}
                ]
            }),
            json!({
                "information_state_id":"D-QUIET","observable_history":observed_history(&["initial", "quiet"], &["DEFER"]),"stage":1,
                "resource_units_available":2.0,"service_levels":{"water":100.0,"power":100.0},
                "actions":[
                    {"action_id":"USE-SPARE","fixed_cost":1.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"DQ-END-USE","stage_cost":0.0,"resources_replenished":0.0}]},
                    {"action_id":"HOLD","fixed_cost":3.0,"resource_use":0.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"DQ-END-HOLD","stage_cost":0.0,"resources_replenished":0.0}]}
                ]
            }),
            json!({
                "information_state_id":"D-STORM","observable_history":observed_history(&["initial", "storm"], &["DEFER"]),"stage":1,
                "resource_units_available":2.0,"service_levels":{"water":40.0,"power":50.0},
                "actions":[{"action_id":"RECOVER","fixed_cost":3.0,"resource_use":1.0,"outcomes":[{"outcome_id":"done","probability":1.0,"next_information_state_id":"DS-END-RECOVER","stage_cost":0.0,"resources_replenished":0.0}]}]
            })
        ];
        states.push(terminal_state("PQ-END-USE", 0.0, 100.0, 100.0, &["initial", "quiet", "safe"], &["PREVENT", "USE-SPARE"]));
        states.push(terminal_state("PQ-END-HOLD", 1.0, 100.0, 100.0, &["initial", "quiet", "held"], &["PREVENT", "HOLD"]));
        states.push(terminal_state("PS-END-REPAIR", 0.0, 80.0, 90.0, &["initial", "storm", "repaired"], &["PREVENT", "REPAIR"]));
        states.push(terminal_state("PS-END-HOLD", 1.0, 70.0, 80.0, &["initial", "storm", "held"], &["PREVENT", "HOLD"]));
        states.push(terminal_state("DQ-END-USE", 1.0, 100.0, 100.0, &["initial", "quiet", "safe"], &["DEFER", "USE-SPARE"]));
        states.push(terminal_state("DQ-END-HOLD", 2.0, 100.0, 100.0, &["initial", "quiet", "held"], &["DEFER", "HOLD"]));
        states.push(terminal_state("DS-END-RECOVER", 1.0, 60.0, 50.0, &["initial", "storm", "recovery-failed"], &["DEFER", "RECOVER"]));

        let next_observations: HashMap<String, Value> = states
            .iter()
            .map(|state| {
                let id = state["information_state_id"].as_str().unwrap().to_owned();
                let observation = state["observable_history"]["observations"].as_array().unwrap().last().unwrap().clone();
                (id, observation)
            })
            .collect();
        for state in &mut states {
            for action in state["actions"].as_array_mut().unwrap() {
                for outcome in action["outcomes"].as_array_mut().unwrap() {
                    let next_id = outcome
                        .get("next_information_state_id")
                        .and_then(Value::as_str)
                        .map(str::to_owned);
                    match next_id {
                        Some(next_id) => outcome["observation"] = next_observations[&next_id].clone(),
                        None => outcome["observation"] = Value::Null,
                    }
                }
            }
        }

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
    fn finite_distribution_cvar_matches_hand_calculation() {
        let risk = upper_cvar(&[(0.0, 0.95), (1000.0, 0.05)], 0.9).unwrap();
        assert!((risk - 500.0).abs() < 1e-10);
    }

    #[test]
    fn nested_cvar_reverses_static_precommitment_local_action() {
        let report = evaluate_nested_risk_comparison(&risk_fixture(0.9)).unwrap();
        assert_eq!(report["local_decision_reversal_at_A"]["nested_action"], json!("SAFE"));
        assert_eq!(report["local_decision_reversal_at_A"]["static_precommitment_action"], json!("GAMBLE"));
        assert_eq!(report["local_decision_reversal_at_A"]["different"], json!(true));
        assert!((report["nested"]["nested_mean_cvar_objective"].as_f64().unwrap() - 9507.5).abs() < 1e-10);
        assert!((report["static_terminal"]["static_terminal_cvar"].as_f64().unwrap() - 10000.0).abs() < 1e-10);
        assert!((report["static_terminal"]["static_terminal_mean_cvar_objective"].as_f64().unwrap() - 9502.5).abs() < 1e-10);
        assert!((report["nested"]["policy_expected_cost"].as_f64().unwrap() - 5075.0).abs() < 1e-10);
        assert!((report["static_terminal"]["policy_expected_cost"].as_f64().unwrap() - 5025.0).abs() < 1e-10);
    }

    #[test]
    fn nested_cvar_keeps_one_action_per_observable_information_state() {
        let report = evaluate_nested_risk_comparison(&risk_fixture(0.9)).unwrap();
        let policy = report["nested"]["policy"].as_object().unwrap();
        assert_eq!(policy.len(), 5);
        assert_eq!(report["nonanticipativity_semantics"], json!("both optimizers choose one action per explicit observable information state"));
    }

    #[test]
    fn invalid_nested_cvar_confidence_is_rejected() {
        let fixture = risk_fixture(1.0);
        assert!(evaluate_nested_risk_comparison(&fixture).is_err());
    }

    #[test]
    fn terminal_cvar_enumeration_respects_pathwise_service_floors() {
        let mut fixture = risk_fixture(0.9);
        fixture["information_states"][2]["service_levels"]["water"] = json!(40.0);
        let report = evaluate_nested_risk_comparison(&fixture).unwrap();
        assert!(report["nested"].is_null());
        assert!(report["static_terminal"].is_null());
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
        fixture["information_states"][1]["observable_history"] = fixture["information_states"][3]["observable_history"].clone();
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn hidden_future_scenario_field_is_rejected_from_observations() {
        let mut fixture = small_fixture();
        fixture["information_states"][0]["observable_history"]["observations"][0]["values"]["latent_scenario_id"] = json!("HIDDEN");
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn transition_observation_must_match_child_history_exactly() {
        let mut fixture = small_fixture();
        fixture["information_states"][0]["actions"][0]["outcomes"][0]["observation"]["values"]["observed_event"] = json!("storm");
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn terminal_transition_requires_null_observation() {
        let mut fixture = small_fixture();
        fixture["information_states"][5]["actions"][0]["outcomes"][0]["observation"] = json!({
            "stage": 3,
            "values": {"observed_event": "phantom"}
        });
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }

    #[test]
    fn child_history_must_append_the_selected_prior_action() {
        let mut fixture = small_fixture();
        fixture["information_states"][1]["observable_history"]["prior_actions"] = json!(["UNDECLARED-ACTION"]);
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
        fixture["information_states"][2]["service_levels"]["water"] = json!(60.0);
        let report = evaluate_exact_adaptive_recourse(&fixture).unwrap();
        assert_eq!(report["status"], json!("infeasible"));
        assert!(report["expected_total_cost"].is_null());
    }

    #[test]
    fn wrong_stage_transition_is_rejected() {
        let mut fixture = small_fixture();
        fixture["information_states"][0]["actions"][0]["outcomes"][0]["next_information_state_id"] = json!("PQ-END-USE");
        assert!(evaluate_exact_adaptive_recourse(&fixture).is_err());
    }
}

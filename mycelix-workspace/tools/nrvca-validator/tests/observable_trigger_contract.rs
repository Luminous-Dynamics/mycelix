use serde_json::{json, Value};
use std::collections::BTreeSet;

const CONTRACT: &str =
    include_str!("../../../../fixtures/nrvca-observable-trigger-traces-synthetic-v0.1.json");
const EPS: f64 = 1e-9;

fn fixture() -> Value {
    serde_json::from_str(CONTRACT).expect("trigger fixture parses")
}

fn num(value: &Value) -> f64 {
    value.as_f64().expect("numeric field")
}

fn text<'a>(value: &'a Value, field: &str) -> &'a str {
    value[field].as_str().unwrap_or_else(|| panic!("missing string field {field}"))
}

fn unique_ids<'a>(items: &'a [Value], field: &str) -> Result<BTreeSet<&'a str>, String> {
    let mut ids = BTreeSet::new();
    for item in items {
        let id = item[field].as_str().ok_or_else(|| format!("missing {field}"))?;
        if !ids.insert(id) {
            return Err(format!("duplicate {field}: {id}"));
        }
    }
    Ok(ids)
}

fn indicator<'a>(root: &'a Value, id: &str) -> Option<&'a Value> {
    root["indicators"].as_array()?.iter()
        .find(|item| item["indicator_id"].as_str() == Some(id))
}

fn rule_for_indicator<'a>(root: &'a Value, id: &str) -> Option<&'a Value> {
    root["rules"].as_array()?.iter()
        .find(|item| item["indicator_id"].as_str() == Some(id))
}

fn action<'a>(root: &'a Value, id: &str) -> Option<&'a Value> {
    root["actions"].as_array()?.iter()
        .find(|item| item["action_id"].as_str() == Some(id))
}

fn matches_operator(value: f64, operator: &str, threshold: f64) -> bool {
    match operator {
        "less_than" => value < threshold,
        "less_than_or_equal" => value <= threshold,
        "greater_than" => value > threshold,
        "greater_than_or_equal" => value >= threshold,
        _ => false,
    }
}

fn validate_contract(root: &Value) -> Result<(), String> {
    if root["status"].as_str() != Some("synthetic_test_contract") {
        return Err("trigger contract must retain synthetic status".to_owned());
    }
    if root["action_authority"]["automatic_dispatch_enabled"].as_bool() != Some(false)
        || root["action_authority"]["execution_mode"].as_str() != Some("recommendation_only")
    {
        return Err("v0.1 must not authorize automated infrastructure dispatch".to_owned());
    }

    let indicators = root["indicators"].as_array()
        .ok_or_else(|| "indicators must be an array".to_owned())?;
    let rules = root["rules"].as_array()
        .ok_or_else(|| "rules must be an array".to_owned())?;
    let actions = root["actions"].as_array()
        .ok_or_else(|| "actions must be an array".to_owned())?;
    unique_ids(indicators, "indicator_id")?;
    unique_ids(rules, "rule_id")?;
    unique_ids(actions, "action_id")?;

    let action_cap = num(&root["hard_contract"]["max_single_action_cost"]);
    for item in indicators {
        if item["evidence_status"].as_str() != Some("synthetic_assumption")
            || item["calibration_status"].as_str() != Some("not_calibrated")
        {
            return Err("indicator evidence status exceeded synthetic ceiling".to_owned());
        }
        if item["allowed_min"].as_f64().is_none() || item["allowed_max"].as_f64().is_none()
            || num(&item["allowed_min"]) > num(&item["allowed_max"])
        {
            return Err(format!("invalid domain for {}", text(item, "indicator_id")));
        }
    }

    for rule in rules {
        let indicator_id = text(rule, "indicator_id");
        let ind = indicator(root, indicator_id)
            .ok_or_else(|| format!("unknown indicator {indicator_id}"))?;
        if rule["unit"] != ind["unit"] {
            return Err(format!("rule unit mismatch for {}", text(rule, "rule_id")));
        }
        let activate = text(rule, "activate_operator");
        let recover = text(rule, "recover_operator");
        if !matches!(activate, "less_than" | "less_than_or_equal" | "greater_than" | "greater_than_or_equal")
            || !matches!(recover, "less_than" | "less_than_or_equal" | "greater_than" | "greater_than_or_equal")
        {
            return Err(format!("unsupported threshold operator in {}", text(rule, "rule_id")));
        }
        if rule["persistence_valid_samples"].as_u64().unwrap_or(0) == 0
            || rule["recovery_valid_samples"].as_u64().unwrap_or(0) == 0
            || rule["max_age_steps"].as_u64().is_none()
            || rule["cooldown_steps"].as_u64().is_none()
        {
            return Err(format!("invalid persistence/cooldown fields in {}", text(rule, "rule_id")));
        }
        if rule["fallback"].as_str().is_none_or(str::is_empty)
            || rule["decision_right"].as_str() != Some("recommendation_only")
        {
            return Err(format!("missing fallback or invalid decision right in {}", text(rule, "rule_id")));
        }
        let linked_action = action(root, text(rule, "action_id"))
            .ok_or_else(|| format!("rule references unknown action {}", text(rule, "action_id")))?;
        if linked_action["failure_fallback"].as_str() != rule["fallback"].as_str() {
            return Err(format!("rule/action fallback mismatch for {}", text(rule, "rule_id")));
        }
        if linked_action["automatic_dispatch_allowed"].as_bool() != Some(false) {
            return Err(format!("action {} permits automatic dispatch", text(linked_action, "action_id")));
        }
        if num(&linked_action["cost"]) < 0.0 || num(&linked_action["cost"]) > action_cap + EPS {
            return Err(format!("action {} exceeds the single-action cost cap", text(linked_action, "action_id")));
        }
        if linked_action["lead_time_steps"].as_u64().is_none()
            || linked_action["required_preconditions"].as_array().is_none()
        {
            return Err(format!("action {} lacks lead-time or preconditions", text(linked_action, "action_id")));
        }
    }

    let traces = root["traces"].as_array().ok_or_else(|| "traces must be an array".to_owned())?;
    unique_ids(traces, "trace_id")?;
    for trace in traces {
        let indicator_id = text(trace, "indicator_id");
        let ind = indicator(root, indicator_id).ok_or_else(|| format!("unknown trace indicator {indicator_id}"))?;
        let rule = rule_for_indicator(root, indicator_id)
            .ok_or_else(|| format!("no rule for trace indicator {indicator_id}"))?;
        if trace["status"].as_str() != Some("synthetic_assumption") {
            return Err(format!("trace {} is not explicitly synthetic", text(trace, "trace_id")));
        }
        let observations = trace["observations"].as_array()
            .ok_or_else(|| "observations must be an array".to_owned())?;
        unique_ids(observations, "observation_id")?;
        let mut previous_step: Option<i64> = None;
        for obs in observations {
            let step = obs["evaluation_step"].as_i64()
                .ok_or_else(|| "evaluation_step must be an integer".to_owned())?;
            if previous_step.is_some_and(|previous| step <= previous) {
                return Err(format!("evaluation steps must be strictly increasing in {}", text(trace, "trace_id")));
            }
            previous_step = Some(step);
            let quality = text(obs, "quality_status");
            if !matches!(quality, "valid" | "missing") {
                return Err(format!("unknown quality_status {quality}"));
            }
            if quality == "valid" {
                if obs["value"].as_f64().is_none() {
                    return Err(format!("valid observation {} has no numeric value", text(obs, "observation_id")));
                }
                if obs["unit"] != ind["unit"] || rule["unit"] != ind["unit"] {
                    return Err(format!("unit mismatch at observation {}", text(obs, "observation_id")));
                }
                let value = num(&obs["value"]);
                if value < num(&ind["allowed_min"]) || value > num(&ind["allowed_max"]) {
                    return Err(format!("out-of-range value at observation {}", text(obs, "observation_id")));
                }
                let observed_at = obs["observed_at_step"].as_i64()
                    .ok_or_else(|| "observed_at_step must be an integer".to_owned())?;
                if observed_at > step {
                    return Err(format!("future observation at {}", text(obs, "observation_id")));
                }
            } else if !obs["value"].is_null() {
                return Err(format!("missing observation {} must not carry a value", text(obs, "observation_id")));
            }
        }
    }
    Ok(())
}

#[derive(Default, Debug)]
struct RuleState {
    active: bool,
    low_count: u64,
    recovery_count: u64,
    last_fire_step: Option<i64>,
}

#[derive(Debug, PartialEq)]
struct FireEvent {
    evaluation_step: i64,
    observation_id: String,
    rule_id: String,
    action_id: String,
    late_warning: bool,
}

#[derive(Debug, PartialEq)]
struct SuppressedEvent {
    evaluation_step: i64,
    action_id: String,
    reason: String,
}

#[derive(Debug, PartialEq)]
struct TraceResult {
    fires: Vec<FireEvent>,
    review_steps: Vec<i64>,
    suppressed: Vec<SuppressedEvent>,
}

fn evaluate_trace(root: &Value, trace: &Value) -> Result<TraceResult, String> {
    validate_contract(root)?;
    let indicator_id = text(trace, "indicator_id");
    let ind = indicator(root, indicator_id)
        .ok_or_else(|| format!("unknown indicator {indicator_id}"))?;
    let rule = rule_for_indicator(root, indicator_id)
        .ok_or_else(|| format!("no rule for {indicator_id}"))?;
    let act = action(root, text(rule, "action_id"))
        .ok_or_else(|| format!("unknown action {}", text(rule, "action_id")))?;
    let preconditions: BTreeSet<String> = trace["available_preconditions"].as_array()
        .ok_or_else(|| "available_preconditions must be an array".to_owned())?
        .iter().map(|x| x.as_str().expect("precondition string").to_owned()).collect();
    let required: BTreeSet<String> = act["required_preconditions"].as_array()
        .ok_or_else(|| "required_preconditions must be an array".to_owned())?
        .iter().map(|x| x.as_str().expect("precondition string").to_owned()).collect();
    let preconditions_met = required.is_subset(&preconditions);

    let mut state = RuleState::default();
    let mut result = TraceResult { fires: Vec::new(), review_steps: Vec::new(), suppressed: Vec::new() };
    let activate_after = rule["persistence_valid_samples"].as_u64().unwrap();
    let recover_after = rule["recovery_valid_samples"].as_u64().unwrap();
    let max_age = rule["max_age_steps"].as_i64().unwrap();
    let cooldown = rule["cooldown_steps"].as_i64().unwrap();
    let lead = act["lead_time_steps"].as_i64().unwrap();
    let observations = trace["observations"].as_array().ok_or_else(|| "observations must be an array".to_owned())?;

    for obs in observations {
        let step = obs["evaluation_step"].as_i64().ok_or_else(|| "bad evaluation_step".to_owned())?;
        let oid = text(obs, "observation_id").to_owned();
        let observed_at = obs["observed_at_step"].as_i64().ok_or_else(|| "bad observed_at_step".to_owned())?;
        let age = step - observed_at;
        let usable = obs["quality_status"].as_str() == Some("valid")
            && obs["value"].as_f64().is_some()
            && obs["unit"] == ind["unit"]
            && observed_at <= step
            && age <= max_age
            && obs["value"].as_f64().is_some_and(|v| v >= num(&ind["allowed_min"]) && v <= num(&ind["allowed_max"]));

        if !usable {
            state.low_count = 0;
            state.recovery_count = 0;
            result.review_steps.push(step);
            continue;
        }

        let value = num(&obs["value"]);
        if state.active {
            let recovered = matches_operator(value, text(rule, "recover_operator"), num(&rule["recover_threshold"]));
            if recovered {
                state.recovery_count += 1;
            } else {
                state.recovery_count = 0;
            }
            let cooldown_done = state.last_fire_step.is_some_and(|last| step - last >= cooldown);
            if state.recovery_count >= recover_after && cooldown_done {
                state.active = false;
                state.low_count = 0;
                state.recovery_count = 0;
            }
            continue;
        }

        let activates = matches_operator(value, text(rule, "activate_operator"), num(&rule["activate_threshold"]));
        if activates {
            state.low_count += 1;
        } else {
            state.low_count = 0;
        }
        if state.low_count < activate_after {
            continue;
        }

        if !preconditions_met {
            result.review_steps.push(step);
            result.suppressed.push(SuppressedEvent {
                evaluation_step: step,
                action_id: text(act, "action_id").to_owned(),
                reason: "precondition_not_met".to_owned(),
            });
            // Do not latch an unperformed action. A later valid observation may retry.
            state.low_count = activate_after - 1;
            continue;
        }

        let time_to_breach = obs["time_to_floor_breach_steps"].as_i64()
            .ok_or_else(|| format!("observation {oid} lacks time_to_floor_breach_steps"))?;
        result.fires.push(FireEvent {
            evaluation_step: step,
            observation_id: oid,
            rule_id: text(rule, "rule_id").to_owned(),
            action_id: text(act, "action_id").to_owned(),
            late_warning: lead > time_to_breach,
        });
        state.active = true;
        state.low_count = 0;
        state.recovery_count = 0;
        state.last_fire_step = Some(step);
    }
    Ok(result)
}

fn trace<'a>(root: &'a Value, trace_id: &str) -> &'a Value {
    root["traces"].as_array().unwrap().iter()
        .find(|item| item["trace_id"].as_str() == Some(trace_id))
        .unwrap_or_else(|| panic!("missing trace {trace_id}"))
}

fn check_expected(root: &Value, trace_item: &Value) {
    let result = evaluate_trace(root, trace_item).expect("valid trigger trace");
    let expected_events = trace_item["expected_events"].as_array().unwrap();
    let expected: Vec<FireEvent> = expected_events.iter().map(|event| FireEvent {
        evaluation_step: event["evaluation_step"].as_i64().unwrap(),
        observation_id: text(event, "observation_id").to_owned(),
        rule_id: text(event, "rule_id").to_owned(),
        action_id: text(event, "action_id").to_owned(),
        late_warning: event["late_warning"].as_bool().unwrap(),
    }).collect();
    assert_eq!(result.fires, expected, "trigger mismatch for {}", text(trace_item, "trace_id"));
    let mut review = result.review_steps.clone();
    review.sort();
    let mut expected_review: Vec<i64> = trace_item["expected_review_steps"].as_array().unwrap()
        .iter().map(|v| v.as_i64().unwrap()).collect();
    expected_review.sort();
    assert_eq!(review, expected_review, "review-step mismatch for {}", text(trace_item, "trace_id"));
    let mut suppressed = result.suppressed.clone();
    suppressed.sort_by_key(|x| (x.evaluation_step, x.action_id.clone()));
    let mut expected_suppressed: Vec<SuppressedEvent> = trace_item.get("expected_suppressed_actions")
        .and_then(Value::as_array).into_iter().flatten().map(|event| SuppressedEvent {
            evaluation_step:event["evaluation_step"].as_i64().unwrap(),
            action_id:text(event,"action_id").to_owned(),
            reason:text(event,"reason").to_owned()
        }).collect();
    expected_suppressed.sort_by_key(|x| (x.evaluation_step, x.action_id.clone()));
    assert_eq!(suppressed, expected_suppressed, "suppressed-action mismatch for {}", text(trace_item, "trace_id"));
}

#[test]
fn contract_has_three_unique_observable_rules_and_safe_authority_boundaries() {
    let root = fixture();
    validate_contract(&root).expect("valid trigger contract");
    assert_eq!(root["rules"].as_array().unwrap().len(), 3);
    assert_eq!(root["actions"].as_array().unwrap().len(), 3);
    assert_eq!(root["action_authority"]["automatic_dispatch_enabled"], false);
    assert!(root["rules"].as_array().unwrap().iter().all(|r| {
        r["fallback"].as_str().is_some_and(|x| !x.is_empty())
            && r["version"].as_str().is_some_and(|x| !x.is_empty())
    }));
}

#[test]
fn known_answer_traces_reproduce_persistence_hysteresis_cooldown_and_late_warnings() {
    let root = fixture();
    validate_contract(&root).expect("contract validates");
    for item in root["traces"].as_array().unwrap() {
        check_expected(&root, item);
    }
    assert_eq!(trace(&root,"reserve-persistence-hysteresis")["expected_events"].as_array().unwrap().len(),2);
    assert_eq!(trace(&root,"grid-cooldown-and-rearm")["expected_events"].as_array().unwrap().len(),2);
    assert_eq!(trace(&root,"crew-one-sample-spike")["expected_events"].as_array().unwrap().len(),0);
}

#[test]
fn single_sample_spike_does_not_trigger_two_sample_rule() {
    let root = fixture();
    let result = evaluate_trace(&root, trace(&root,"crew-one-sample-spike")).unwrap();
    assert!(result.fires.is_empty());
}

#[test]
fn missing_or_stale_observations_reset_persistence_and_require_manual_review() {
    let root = fixture();
    let result = evaluate_trace(&root, trace(&root,"missing-and-stale-reserve-data")).unwrap();
    assert_eq!(result.review_steps, vec![1,2]);
    assert_eq!(result.fires.iter().map(|e|e.evaluation_step).collect::<Vec<_>>(), vec![4]);
}

#[test]
fn unavailable_action_precondition_suppresses_recommendation() {
    let root = fixture();
    let result = evaluate_trace(&root, trace(&root,"precondition-unavailable")).unwrap();
    assert!(result.fires.is_empty());
    assert_eq!(result.review_steps, vec![1]);
    assert_eq!(result.suppressed, vec![SuppressedEvent {
        evaluation_step:1, action_id:"release_storage_reserve".to_owned(),
        reason:"precondition_not_met".to_owned()
    }]);
}

#[test]
fn duplicate_steps_wrong_units_out_of_range_values_and_unknown_indicators_fail_closed() {
    let root = fixture();

    let mut duplicate = trace(&root,"crew-two-period-delay").clone();
    duplicate["observations"][1]["evaluation_step"] = json!(0);
    assert!(validate_contract_with_trace(&root,&duplicate).is_err());

    let mut wrong_unit = root.clone();
    wrong_unit["traces"][0]["observations"][0]["unit"] = json!("percent");
    assert!(validate_contract(&wrong_unit).is_err());

    let mut out_of_range = root.clone();
    out_of_range["traces"][0]["observations"][0]["value"] = json!(1.2);
    assert!(validate_contract(&out_of_range).is_err());

    let mut unknown = root.clone();
    unknown["traces"][0]["indicator_id"] = json!("undefined-indicator");
    assert!(validate_contract(&unknown).is_err());
}

fn validate_contract_with_trace(root: &Value, changed_trace: &Value) -> Result<(), String> {
    validate_contract(root)?;
    let id = text(changed_trace,"trace_id");
    let original = trace(root,id);
    if changed_trace["indicator_id"] != original["indicator_id"] {
        return Err("trace indicator changed".to_owned());
    }
    let observations = changed_trace["observations"].as_array()
        .ok_or_else(|| "observations must be array".to_owned())?;
    unique_ids(observations,"observation_id")?;
    let mut previous = None;
    for obs in observations {
        let step=obs["evaluation_step"].as_i64().ok_or_else(||"bad step".to_owned())?;
        if previous.is_some_and(|p|step<=p) { return Err("duplicate or nonmonotone evaluation step".to_owned()); }
        previous=Some(step);
    }
    Ok(())
}

#[test]
fn changing_labels_does_not_change_trigger_events() {
    let mut root = fixture();
    let before = evaluate_trace(&root, trace(&root,"reserve-persistence-hysteresis")).unwrap();
    for rule in root["rules"].as_array_mut().unwrap() {
        rule["display_label"] = json!(format!("marketing-{}", text(rule,"rule_id")));
    }
    for act in root["actions"].as_array_mut().unwrap() {
        act["description"] = json!("Changed descriptive wording only.");
    }
    let after = evaluate_trace(&root, trace(&root,"reserve-persistence-hysteresis")).unwrap();
    assert_eq!(before,after);
}

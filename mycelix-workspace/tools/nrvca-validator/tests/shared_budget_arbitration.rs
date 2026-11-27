use serde_json::{json, Value};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

const CONTRACT: &str = include_str!("../../../../fixtures/nrvca-shared-budget-arbitration-synthetic-v0.1.json");

fn fixture() -> Value {
    serde_json::from_str(CONTRACT).expect("shared-budget fixture parses")
}

fn text<'a>(value: &'a Value, field: &str) -> &'a str {
    value[field]
        .as_str()
        .unwrap_or_else(|| panic!("missing string field {field}"))
}

fn integer(value: &Value, field: &str) -> Result<i64, String> {
    value[field]
        .as_i64()
        .ok_or_else(|| format!("missing or non-integer {field}"))
}

fn unique_ids(items: &[Value], field: &str) -> Result<BTreeSet<String>, String> {
    let mut ids = BTreeSet::new();
    for item in items {
        let id = item[field]
            .as_str()
            .ok_or_else(|| format!("missing {field}"))?
            .to_owned();
        if id.trim().is_empty() {
            return Err(format!("{field} cannot be empty"));
        }
        if !ids.insert(id.clone()) {
            return Err(format!("duplicate {field}: {id}"));
        }
    }
    Ok(ids)
}

fn action<'a>(root: &'a Value, action_id: &str) -> Option<&'a Value> {
    root["actions"]
        .as_array()?
        .iter()
        .find(|item| item["action_id"].as_str() == Some(action_id))
}

fn resource_capacities(root: &Value) -> Result<BTreeMap<String, i64>, String> {
    let object = root["resource_capacities"]
        .as_object()
        .ok_or_else(|| "resource_capacities must be an object".to_owned())?;
    let mut capacities = BTreeMap::new();
    for (resource_id, value) in object {
        if resource_id.trim().is_empty() {
            return Err("resource ID cannot be empty".to_owned());
        }
        let capacity = value
            .as_i64()
            .ok_or_else(|| format!("resource capacity {resource_id} must be an integer"))?;
        if capacity < 0 {
            return Err(format!("resource capacity {resource_id} cannot be negative"));
        }
        capacities.insert(resource_id.clone(), capacity);
    }
    Ok(capacities)
}

fn required_preconditions<'a>(action: &'a Value) -> Result<BTreeSet<&'a str>, String> {
    let values = action["required_preconditions"]
        .as_array()
        .ok_or_else(|| format!("{} missing required_preconditions", text(action, "action_id")))?;
    let mut result = BTreeSet::new();
    for value in values {
        let precondition = value
            .as_str()
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| format!("{} has a missing/empty/non-string precondition", text(action, "action_id")))?;
        if !result.insert(precondition) {
            return Err(format!("{} has duplicate precondition {precondition}", text(action, "action_id")));
        }
    }
    Ok(result)
}

fn available_preconditions(candidate: &Value) -> Result<BTreeSet<&str>, String> {
    let values = candidate["available_preconditions"]
        .as_array()
        .ok_or_else(|| format!("{} missing available_preconditions", text(candidate, "candidate_id")))?;
    let mut result = BTreeSet::new();
    for value in values {
        let precondition = value
            .as_str()
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| format!("{} has a missing/empty/non-string available precondition", text(candidate, "candidate_id")))?;
        if !result.insert(precondition) {
            return Err(format!("{} repeats available precondition {precondition}", text(candidate, "candidate_id")));
        }
    }
    Ok(result)
}

fn preconditions_met(candidate: &Value, act: &Value) -> Result<bool, String> {
    let available = available_preconditions(candidate)?;
    let required = required_preconditions(act)?;
    Ok(required.is_subset(&available))
}

fn resource_claims(act: &Value) -> Result<BTreeMap<String, i64>, String> {
    let object = act["resource_claims"]
        .as_object()
        .ok_or_else(|| format!("{} missing resource_claims", text(act, "action_id")))?;
    let mut claims = BTreeMap::new();
    for (resource_id, value) in object {
        let units = value
            .as_i64()
            .ok_or_else(|| format!("{} claim for {resource_id} must be an integer", text(act, "action_id")))?;
        if units <= 0 {
            return Err(format!("{} claim for {resource_id} must be positive", text(act, "action_id")));
        }
        claims.insert(resource_id.clone(), units);
    }
    Ok(claims)
}

fn validate_contract(root: &Value) -> Result<(), String> {
    if root["status"].as_str() != Some("synthetic_test_contract") {
        return Err("contract must retain synthetic_test_contract status".to_owned());
    }
    if root["action_authority"]["execution_mode"].as_str() != Some("recommendation_only")
        || root["action_authority"]["automatic_dispatch_enabled"].as_bool() != Some(false)
    {
        return Err("contract must not authorize automatic dispatch".to_owned());
    }
    if root["claim_ceiling"]["calibration_status"].as_str() != Some("not_calibrated")
        || root["claim_ceiling"]["geographic_scope"].as_str() != Some("fictional_only")
    {
        return Err("claim ceiling must remain fictional and uncalibrated".to_owned());
    }

    let actions = root["actions"]
        .as_array()
        .ok_or_else(|| "actions must be an array".to_owned())?;
    unique_ids(actions, "action_id")?;
    let capacities = resource_capacities(root)?;
    let unit = root["policy"]["cost_unit"]
        .as_str()
        .filter(|value| !value.trim().is_empty())
        .ok_or_else(|| "policy cost_unit must be a non-empty string".to_owned())?;
    let max_cost = integer(&root["hard_contract"], "max_single_action_cost")?;
    let max_candidates = integer(&root["hard_contract"], "max_candidates_per_case")?;
    if max_cost < 0 || max_candidates <= 0 || max_candidates > 1_000_000 {
        return Err("single-action cost and case-size limits are outside supported bounds".to_owned());
    }
    if max_cost > i64::MAX / max_candidates {
        return Err("single-action cost cap could overflow a full candidate set".to_owned());
    }

    for act in actions {
        let id = text(act, "action_id");
        let cost = integer(act, "cost")?;
        if cost < 0 || cost > max_cost {
            return Err(format!("action {id} violates max single-action cost"));
        }
        if act["cost_unit"].as_str() != Some(unit) {
            return Err(format!("action {id} has incompatible cost unit"));
        }
        if !matches!(
            act["policy_class"].as_str(),
            Some("protected_service_floor" | "discretionary_recovery")
        ) {
            return Err(format!("action {id} has unsupported policy class"));
        }
        let class_rank = integer(act, "class_rank")?;
        let priority_rank = integer(act, "priority_rank")?;
        if class_rank < 0 || priority_rank < 0 {
            return Err(format!("action {id} has negative policy rank"));
        }
        if act["policy_class"].as_str() == Some("protected_service_floor") && class_rank != 0 {
            return Err(format!("protected action {id} must use class rank 0"));
        }
        if act["policy_class"].as_str() == Some("discretionary_recovery") && class_rank != 1 {
            return Err(format!("discretionary action {id} must use class rank 1"));
        }
        required_preconditions(act)?;
        for (resource_id, units) in resource_claims(act)? {
            let capacity = capacities
                .get(&resource_id)
                .ok_or_else(|| format!("action {id} references unknown resource {resource_id}"))?;
            if units > *capacity {
                return Err(format!("action {id} resource claim exceeds {resource_id} capacity"));
            }
        }
    }

    let cases = root["cases"]
        .as_array()
        .ok_or_else(|| "cases must be an array".to_owned())?;
    unique_ids(cases, "case_id")?;
    for case in cases {
        validate_case(root, case)?;
    }
    if root["policy"]["protected_class_atomic"].as_bool() != Some(true) {
        return Err("protected service-floor bundle must remain atomic".to_owned());
    }
    Ok(())
}

fn validate_case(root: &Value, case: &Value) -> Result<(), String> {
    if case["status"].as_str() != Some("synthetic_assumption") {
        return Err(format!("case {} must retain synthetic status", text(case, "case_id")));
    }
    let budget = integer(case, "budget")?;
    if budget < 0 {
        return Err(format!("case {} has negative budget", text(case, "case_id")));
    }
    let candidates = case["candidates"]
        .as_array()
        .ok_or_else(|| format!("case {} candidates must be an array", text(case, "case_id")))?;
    let max_candidates = integer(&root["hard_contract"], "max_candidates_per_case")?;
    if candidates.len() > max_candidates as usize {
        return Err(format!("case {} exceeds max candidate count", text(case, "case_id")));
    }
    unique_ids(candidates, "candidate_id")?;
    for candidate in candidates {
        let candidate_id = text(candidate, "candidate_id");
        if candidate.get("cost").is_some() {
            return Err(format!("candidate {candidate_id} must not override authoritative action cost"));
        }
        let action_id = candidate["action_id"]
            .as_str()
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| format!("candidate {candidate_id} has missing/empty action_id"))?;
        let act = action(root, action_id)
            .ok_or_else(|| format!("candidate {candidate_id} references unknown action {action_id}"))?;
        available_preconditions(candidate)?;
        required_preconditions(act)?;
    }
    Ok(())
}

#[derive(Debug, PartialEq, Clone)]
struct CandidateDecision {
    candidate_id: String,
    action_id: String,
    disposition: String,
    reason: String,
    cost: i64,
    budget_before: i64,
    budget_after: i64,
}

#[derive(Debug, PartialEq, Clone)]
struct ArbitrationOutcome {
    status: String,
    selected_action_ids: Vec<String>,
    total_selected_cost: i64,
    remaining_budget: i64,
    decisions: Vec<CandidateDecision>,
}

fn candidate_order(root: &Value, left: &Value, right: &Value) -> Ordering {
    let left_action = action(root, text(left, "action_id")).expect("validated action");
    let right_action = action(root, text(right, "action_id")).expect("validated action");
    integer(left_action, "class_rank").expect("validated class rank")
        .cmp(&integer(right_action, "class_rank").expect("validated class rank"))
        .then_with(|| {
            integer(left_action, "priority_rank").expect("validated priority rank")
                .cmp(&integer(right_action, "priority_rank").expect("validated priority rank"))
        })
        .then_with(|| text(left_action, "action_id").cmp(text(right_action, "action_id")))
        .then_with(|| text(left, "candidate_id").cmp(text(right, "candidate_id")))
}

fn decision(
    candidate: &Value,
    act: &Value,
    disposition: &str,
    reason: &str,
    budget_before: i64,
    budget_after: i64,
) -> CandidateDecision {
    CandidateDecision {
        candidate_id: text(candidate, "candidate_id").to_owned(),
        action_id: text(act, "action_id").to_owned(),
        disposition: disposition.to_owned(),
        reason: reason.to_owned(),
        cost: integer(act, "cost").expect("validated action cost"),
        budget_before,
        budget_after,
    }
}

fn has_missing_preconditions(candidate: &Value, act: &Value) -> bool {
    !preconditions_met(candidate, act).expect("validated preconditions")
}

fn protected_candidates<'a>(root: &'a Value, candidates: &'a [Value]) -> Vec<&'a Value> {
    candidates
        .iter()
        .filter(|candidate| {
            action(root, text(candidate, "action_id"))
                .is_some_and(|act| act["policy_class"].as_str() == Some("protected_service_floor"))
        })
        .collect()
}

fn resource_bundle_fits(
    root: &Value,
    candidates: &[&Value],
    capacities: &BTreeMap<String, i64>,
) -> bool {
    let mut claims: BTreeMap<String, i64> = BTreeMap::new();
    let mut overflowed = false;
    for candidate in candidates {
        let act = action(root, text(candidate, "action_id")).expect("validated action");
        for (resource_id, units) in resource_claims(act).expect("validated claims") {
            let total = claims.entry(resource_id).or_default();
            if let Some(next) = total.checked_add(units) {
                *total = next;
            } else {
                overflowed = true;
            }
        }
    }
    !overflowed && claims.into_iter().all(|(resource_id, total)| {
        capacities.get(&resource_id).is_some_and(|capacity| total <= *capacity)
    })
}

fn resource_claim_fits(
    act: &Value,
    used: &BTreeMap<String, i64>,
    capacities: &BTreeMap<String, i64>,
) -> bool {
    resource_claims(act).expect("validated claims").into_iter().all(|(resource_id, units)| {
        used.get(&resource_id).copied().unwrap_or(0)
            .checked_add(units)
            .is_some_and(|total| capacities.get(&resource_id).is_some_and(|capacity| total <= *capacity))
    })
}

fn add_resource_claims(act: &Value, used: &mut BTreeMap<String, i64>) {
    for (resource_id, units) in resource_claims(act).expect("validated claims") {
        *used.entry(resource_id).or_default() += units;
    }
}

fn arbitrate(root: &Value, case: &Value) -> Result<ArbitrationOutcome, String> {
    validate_contract(root)?;
    validate_case(root, case)?;
    let initial_budget = integer(case, "budget")?;
    let candidates = case["candidates"].as_array().expect("validated candidates");
    if candidates.is_empty() {
        return Ok(ArbitrationOutcome {
            status: "no_candidates".to_owned(),
            selected_action_ids: Vec::new(),
            total_selected_cost: 0,
            remaining_budget: initial_budget,
            decisions: Vec::new(),
        });
    }

    let mut ordered: Vec<&Value> = candidates.iter().collect();
    ordered.sort_by(|left, right| candidate_order(root, left, right));
    let protected = protected_candidates(root, candidates);
    let missing_protected = protected.iter().any(|candidate| {
        let act = action(root, text(candidate, "action_id")).expect("validated action");
        has_missing_preconditions(candidate, act)
    });
    let protected_cost: i64 = protected.iter().map(|candidate| {
        integer(action(root, text(candidate, "action_id")).expect("validated action"), "cost")
            .expect("validated cost")
    }).sum();
    let capacities = resource_capacities(root)?;

    let global_failure = if missing_protected {
        Some((
            "manual_review_protected_precondition",
            "manual_review_precondition_missing",
            "protected_action_missing_required_precondition",
        ))
    } else if protected_cost > initial_budget {
        Some((
            "manual_review_infeasible_protected_tier",
            "manual_review_infeasible_protected_tier",
            "protected_service_floor_bundle_exceeds_shared_budget",
        ))
    } else if !resource_bundle_fits(root, &protected, &capacities) {
        Some((
            "manual_review_infeasible_protected_resources",
            "manual_review_infeasible_protected_resources",
            "protected_service_floor_bundle_exceeds_resource_capacity",
        ))
    } else {
        None
    };

    if let Some((status, protected_disposition, protected_reason)) = global_failure {
        let mut decisions = Vec::new();
        for candidate in &ordered {
            let act = action(root, text(candidate, "action_id")).expect("validated action");
            let is_protected = act["policy_class"].as_str() == Some("protected_service_floor");
            let candidate_missing = has_missing_preconditions(candidate, act);
            let (disposition, reason) = if is_protected && candidate_missing {
                ("manual_review_precondition_missing", "protected_action_missing_required_precondition")
            } else if is_protected && status == "manual_review_protected_precondition" {
                ("blocked_by_protected_review", "another_protected_action_missing_required_precondition")
            } else if is_protected {
                (protected_disposition, protected_reason)
            } else {
                ("blocked_by_protected_review", "blocked_pending_operator_review_of_protected_floor_feasibility")
            };
            decisions.push(decision(candidate, act, disposition, reason, initial_budget, initial_budget));
        }
        return Ok(ArbitrationOutcome {
            status: status.to_owned(),
            selected_action_ids: Vec::new(),
            total_selected_cost: 0,
            remaining_budget: initial_budget,
            decisions,
        });
    }

    let mut remaining_budget = initial_budget;
    let mut total_selected_cost = 0;
    let mut selected_action_ids = Vec::new();
    let mut used_resources: BTreeMap<String, i64> = BTreeMap::new();
    let mut decisions = Vec::new();

    for candidate in &ordered {
        let act = action(root, text(candidate, "action_id")).expect("validated action");
        let cost = integer(act, "cost").expect("validated action cost");
        let budget_before = remaining_budget;
        let is_protected = act["policy_class"].as_str() == Some("protected_service_floor");

        if is_protected {
            // Protected costs and resources were validated as one atomic bundle above.
            remaining_budget -= cost;
            total_selected_cost += cost;
            selected_action_ids.push(text(act, "action_id").to_owned());
            add_resource_claims(act, &mut used_resources);
            decisions.push(decision(
                candidate,
                act,
                "selected",
                "selected_within_shared_budget_and_resource_capacity",
                budget_before,
                remaining_budget,
            ));
            continue;
        }

        if has_missing_preconditions(candidate, act) {
            decisions.push(decision(
                candidate,
                act,
                "manual_review_precondition_missing",
                "required_preconditions_missing",
                budget_before,
                remaining_budget,
            ));
        } else if !resource_claim_fits(act, &used_resources, &capacities) {
            decisions.push(decision(
                candidate,
                act,
                "deferred_resource_conflict",
                "resource_capacity_exhausted_by_higher_priority_selection",
                budget_before,
                remaining_budget,
            ));
        } else if cost > remaining_budget {
            decisions.push(decision(
                candidate,
                act,
                "deferred_shared_budget",
                "insufficient_shared_budget_after_higher_priority_selections",
                budget_before,
                remaining_budget,
            ));
        } else {
            remaining_budget -= cost;
            total_selected_cost += cost;
            selected_action_ids.push(text(act, "action_id").to_owned());
            add_resource_claims(act, &mut used_resources);
            decisions.push(decision(
                candidate,
                act,
                "selected",
                "selected_within_shared_budget_and_resource_capacity",
                budget_before,
                remaining_budget,
            ));
        }
    }

    Ok(ArbitrationOutcome {
        status: "arbitration_complete".to_owned(),
        selected_action_ids,
        total_selected_cost,
        remaining_budget,
        decisions,
    })
}

fn expected_decision(value: &Value) -> CandidateDecision {
    CandidateDecision {
        candidate_id: text(value, "candidate_id").to_owned(),
        action_id: text(value, "action_id").to_owned(),
        disposition: text(value, "disposition").to_owned(),
        reason: text(value, "reason").to_owned(),
        cost: integer(value, "cost").expect("expected cost"),
        budget_before: integer(value, "budget_before").expect("expected budget_before"),
        budget_after: integer(value, "budget_after").expect("expected budget_after"),
    }
}

fn case<'a>(root: &'a Value, case_id: &str) -> &'a Value {
    root["cases"]
        .as_array()
        .unwrap()
        .iter()
        .find(|item| item["case_id"].as_str() == Some(case_id))
        .unwrap_or_else(|| panic!("missing case {case_id}"))
}

fn strings(values: &Value) -> Vec<String> {
    values
        .as_array()
        .expect("expected string array")
        .iter()
        .map(|value| value.as_str().expect("string item").to_owned())
        .collect()
}


fn lexicographically_better(left: &[bool], right: &[bool]) -> bool {
    for (left_bit, right_bit) in left.iter().zip(right.iter()) {
        if left_bit != right_bit {
            return *left_bit && !*right_bit;
        }
    }
    false
}

// Exhaustive oracle for the contract's declared objective: first preserve the
// atomic protected bundle, then maximize the selected/not-selected bit vector
// in policy priority order subject to the same hard budget/resource constraints.
// This proves lexicographic optimality for small fixtures, not public-welfare
// optimality and not optimality under a different objective.
fn exhaustive_lexicographic_oracle(root: &Value, case: &Value) -> Result<Vec<String>, String> {
    validate_contract(root)?;
    validate_case(root, case)?;
    let budget = integer(case, "budget")?;
    let candidates = case["candidates"].as_array().expect("validated candidates");
    if candidates.is_empty() {
        return Ok(Vec::new());
    }

    let mut ordered: Vec<&Value> = candidates.iter().collect();
    ordered.sort_by(|left, right| candidate_order(root, left, right));
    let protected = protected_candidates(root, candidates);
    let protected_cost: i64 = protected.iter().map(|candidate| {
        integer(action(root, text(candidate, "action_id")).expect("validated action"), "cost")
            .expect("validated cost")
    }).sum();
    let capacities = resource_capacities(root)?;

    if protected.iter().any(|candidate| {
        has_missing_preconditions(candidate, action(root, text(candidate, "action_id")).expect("validated action"))
    }) || protected_cost > budget || !resource_bundle_fits(root, &protected, &capacities) {
        return Ok(Vec::new());
    }

    let optional: Vec<&Value> = ordered.iter().copied().filter(|candidate| {
        action(root, text(candidate, "action_id"))
            .is_some_and(|act| act["policy_class"].as_str() == Some("discretionary_recovery"))
    }).collect();
    if optional.len() >= usize::BITS as usize {
        return Err("oracle case has too many discretionary candidates for exhaustive enumeration".to_owned());
    }

    let mut protected_resources: BTreeMap<String, i64> = BTreeMap::new();
    for candidate in &protected {
        add_resource_claims(
            action(root, text(candidate, "action_id")).expect("validated action"),
            &mut protected_resources,
        );
    }
    let mut best_bits: Option<Vec<bool>> = None;
    for mask in 0..(1usize << optional.len()) {
        let mut bits = Vec::with_capacity(optional.len());
        let mut used_resources = protected_resources.clone();
        let mut total_cost = protected_cost;
        let mut feasible = true;

        for (index, candidate) in optional.iter().enumerate() {
            let selected = (mask & (1usize << index)) != 0;
            bits.push(selected);
            if !selected {
                continue;
            }
            let act = action(root, text(candidate, "action_id")).expect("validated action");
            let cost = integer(act, "cost").expect("validated cost");
            if has_missing_preconditions(candidate, act)
                || cost > budget - total_cost
                || !resource_claim_fits(act, &used_resources, &capacities)
            {
                feasible = false;
                break;
            }
            total_cost += cost;
            add_resource_claims(act, &mut used_resources);
        }

        if feasible && best_bits.as_ref().is_none_or(|best| lexicographically_better(&bits, best)) {
            best_bits = Some(bits);
        }
    }

    let best_bits = best_bits.unwrap_or_else(|| vec![false; optional.len()]);
    let selected_optional: BTreeSet<String> = optional.iter().zip(best_bits.iter())
        .filter(|(_, selected)| **selected)
        .map(|(candidate, _)| text(candidate, "candidate_id").to_owned())
        .collect();
    Ok(ordered.iter().filter_map(|candidate| {
        let act = action(root, text(candidate, "action_id")).expect("validated action");
        let is_protected = act["policy_class"].as_str() == Some("protected_service_floor");
        if is_protected || selected_optional.contains(text(candidate, "candidate_id")) {
            Some(text(act, "action_id").to_owned())
        } else {
            None
        }
    }).collect())
}

#[test]
fn contract_and_known_answer_cases_are_valid_and_reproducible() {
    let root = fixture();
    validate_contract(&root).expect("valid arbitration contract");

    for item in root["cases"].as_array().unwrap() {
        let outcome = arbitrate(&root, item).expect("valid arbitration case");
        assert_eq!(outcome.status, text(item, "expected_status"), "status for {}", text(item, "case_id"));
        assert_eq!(outcome.selected_action_ids, strings(&item["expected_selected_actions"]), "selected actions for {}", text(item, "case_id"));
        assert_eq!(outcome.total_selected_cost, integer(item, "expected_total_selected_cost").unwrap());
        assert_eq!(outcome.remaining_budget, integer(item, "expected_remaining_budget").unwrap());
        assert_eq!(outcome.decisions.len(), item["expected_decisions"].as_array().unwrap().len());

        let expected: Vec<CandidateDecision> = item["expected_decisions"]
            .as_array()
            .unwrap()
            .iter()
            .map(expected_decision)
            .collect();
        assert_eq!(outcome.decisions, expected, "receipt mismatch for {}", text(item, "case_id"));
        assert_eq!(
            outcome.total_selected_cost + outcome.remaining_budget,
            integer(item, "budget").unwrap(),
            "budget arithmetic must reconcile for {}",
            text(item, "case_id")
        );
        assert!(outcome.total_selected_cost <= integer(item, "budget").unwrap());
        assert_eq!(
            outcome.decisions.iter().map(|decision| decision.candidate_id.as_str()).collect::<BTreeSet<_>>().len(),
            outcome.decisions.len(),
            "each candidate must have exactly one terminal decision"
        );
    }
}

fn permute<T: Clone>(items: &[T]) -> Vec<Vec<T>> {
    fn visit<T: Clone>(items: &mut Vec<T>, index: usize, out: &mut Vec<Vec<T>>) {
        if index == items.len() {
            out.push(items.clone());
            return;
        }
        for swap_index in index..items.len() {
            items.swap(index, swap_index);
            visit(items, index + 1, out);
            items.swap(index, swap_index);
        }
    }
    let mut working = items.to_vec();
    let mut result = Vec::new();
    visit(&mut working, 0, &mut result);
    result
}

#[test]
fn every_candidate_permutation_has_the_same_canonical_receipt() {
    let root = fixture();
    let original = case(&root, "compound-trigger-overload");
    let candidates = original["candidates"].as_array().unwrap().clone();
    let baseline = arbitrate(&root, original).unwrap();

    for permutation in permute(&candidates) {
        let mut reordered = original.clone();
        reordered["candidates"] = json!(permutation);
        let result = arbitrate(&root, &reordered).expect("permuted case remains valid");
        assert_eq!(result, baseline, "candidate input order changed the arbitration receipt");
    }
    assert_eq!(permute(&candidates).len(), 24, "four candidates have 4! input orderings");
}

#[test]
fn frozen_budget_sweep_is_feasible_and_matches_fixture_specific_selection_sets() {
    let root = fixture();
    let baseline_case = case(&root, "compound-trigger-overload");
    let budgets = root["budget_sweep"]["budgets"].as_array().unwrap();
    let expected_by_budget = &root["budget_sweep"]["expected_selected_actions_by_budget"];
    let mut previous: BTreeSet<String> = BTreeSet::new();

    for value in budgets {
        let budget = value.as_i64().expect("integer budget");
        let mut variant = baseline_case.clone();
        variant["budget"] = json!(budget);
        let outcome = arbitrate(&root, &variant).expect("budget sweep case remains valid");
        assert!(outcome.total_selected_cost <= budget, "overspent at budget {budget}");
        assert_eq!(outcome.total_selected_cost + outcome.remaining_budget, budget);
        let budget_key = budget.to_string();
        let expected = strings(&expected_by_budget[budget_key.as_str()]);
        assert_eq!(outcome.selected_action_ids, expected, "unexpected selected set at budget {budget}");
        let current: BTreeSet<String> = outcome.selected_action_ids.iter().cloned().collect();
        assert!(previous.is_subset(&current), "fixture-specific selected-set monotonicity changed unexpectedly at budget {budget}");
        previous = current;
    }
}


#[test]
fn greedy_receipts_match_exhaustive_lexicographic_oracle_across_budgets() {
    let root = fixture();
    for item in root["cases"].as_array().unwrap() {
        for budget in 0..=6 {
            let mut variant = item.clone();
            variant["budget"] = json!(budget);
            let actual = arbitrate(&root, &variant).expect("valid budget variant");
            let oracle = exhaustive_lexicographic_oracle(&root, &variant).expect("small exhaustive oracle");
            assert_eq!(
                actual.selected_action_ids, oracle,
                "fixed-priority selection diverged from exhaustive oracle for {} at budget {}",
                text(item, "case_id"), budget
            );
        }
    }
}

#[test]
fn malformed_contracts_and_cases_fail_closed() {
    let root = fixture();
    validate_contract(&root).expect("baseline contract valid");

    let mut negative_budget = root.clone();
    negative_budget["cases"][0]["budget"] = json!(-1);
    assert!(arbitrate(&negative_budget, &negative_budget["cases"][0]).is_err());

    let mut unknown_action = root.clone();
    unknown_action["cases"][0]["candidates"][0]["action_id"] = json!("not-in-catalogue");
    assert!(arbitrate(&unknown_action, &unknown_action["cases"][0]).is_err());

    let mut duplicate_candidate = root.clone();
    let duplicate_id = duplicate_candidate["cases"][0]["candidates"][0]["candidate_id"].clone();
    duplicate_candidate["cases"][0]["candidates"][1]["candidate_id"] = duplicate_id;
    assert!(arbitrate(&duplicate_candidate, &duplicate_candidate["cases"][0]).is_err());

    let mut cost_override = root.clone();
    cost_override["cases"][0]["candidates"][0]["cost"] = json!(0);
    assert!(arbitrate(&cost_override, &cost_override["cases"][0]).is_err());

    let mut negative_cost = root.clone();
    negative_cost["actions"][0]["cost"] = json!(-1);
    assert!(validate_contract(&negative_cost).is_err());

    let mut excessive_claim = root.clone();
    excessive_claim["actions"][0]["resource_claims"]["backup_system"] = json!(2);
    assert!(validate_contract(&excessive_claim).is_err());

    let mut automatic_dispatch = root.clone();
    automatic_dispatch["action_authority"]["automatic_dispatch_enabled"] = json!(true);
    assert!(validate_contract(&automatic_dispatch).is_err());

    let mut wrong_unit = root.clone();
    wrong_unit["actions"][0]["cost_unit"] = json!("dollars");
    assert!(validate_contract(&wrong_unit).is_err());

    let mut duplicate_action = root.clone();
    duplicate_action["actions"][1]["action_id"] = json!("deploy_backup_power");
    assert!(validate_contract(&duplicate_action).is_err());

    let mut wrong_rank = root.clone();
    wrong_rank["actions"][2]["class_rank"] = json!(0);
    assert!(validate_contract(&wrong_rank).is_err());

    let mut too_many = root.clone();
    too_many["hard_contract"]["max_candidates_per_case"] = json!(3);
    assert!(validate_contract(&too_many).is_err());

    let mut missing_action_id = root.clone();
    missing_action_id["cases"][0]["candidates"][0].as_object_mut().unwrap().remove("action_id");
    assert!(arbitrate(&missing_action_id, &missing_action_id["cases"][0]).is_err());
}

#[test]
fn protected_floor_infeasibility_cannot_be_hidden_by_optional_actions() {
    let root = fixture();
    let outcome = arbitrate(&root, case(&root, "protected-floor-infeasible")).unwrap();
    assert_eq!(outcome.status, "manual_review_infeasible_protected_tier");
    assert!(outcome.selected_action_ids.is_empty());
    assert_eq!(outcome.remaining_budget, integer(case(&root, "protected-floor-infeasible"), "budget").unwrap());
    assert!(outcome.decisions.iter().all(|decision| {
        decision.budget_before == decision.budget_after
            && decision.disposition != "selected"
    }));
}


#[test]
fn jointly_infeasible_protected_resource_bundle_requires_manual_review() {
    let mut root = fixture();
    // The mutation preserves each action's individual claim limit but makes the
    // two protected actions compete for the same single-capacity resource.
    root["actions"][1]["resource_claims"] = json!({"backup_system":1});
    let mut scenario = case(&root, "protected-floor-infeasible").clone();
    scenario["budget"] = json!(6);
    let outcome = arbitrate(&root, &scenario).unwrap();
    assert_eq!(outcome.status, "manual_review_infeasible_protected_resources");
    assert!(outcome.selected_action_ids.is_empty());
    assert!(outcome.decisions.iter().all(|decision| {
        decision.budget_before == decision.budget_after
            && decision.disposition != "selected"
    }));
}

#[test]
fn protected_precondition_failure_blocks_discretionary_substitution() {
    let root = fixture();
    let outcome = arbitrate(&root, case(&root, "protected-precondition-missing")).unwrap();
    assert_eq!(outcome.status, "manual_review_protected_precondition");
    assert!(outcome.selected_action_ids.is_empty());
    assert_eq!(
        outcome.decisions.iter().find(|decision| decision.action_id == "preposition_repair_crews").unwrap().disposition,
        "blocked_by_protected_review"
    );
}

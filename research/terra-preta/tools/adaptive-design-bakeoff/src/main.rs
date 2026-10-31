//! Synthetic-only benchmark of experiment acquisition policies.
//!
//! Critical separation: PolicyAction intentionally omits evaluator-only truth.
//! The selectors receive only PolicyAction values. Truth is read only after
//! selection to calculate synthetic utility and regret against the oracle.
//! This is a software benchmark, not a field-experiment authorization system.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const VERSION: &str = "0.1.0";
const MAX_ORACLE_CANDIDATES: usize = 20;

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    seed: u64,
    budget_cost_units: u32,
    strata: Vec<Stratum>,
    actions: Vec<Action>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Stratum {
    stratum_id: String,
    target_weight: f64,
    minimum_selected_actions: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Action {
    action_id: String,
    stratum_id: String,
    action_type: String,
    cost_units: u32,
    eligibility: Eligibility,
    exclusion_reason: Option<String>,
    expert_priority: f64,
    ofat_order: u32,
    model_scores: Vec<ModelScore>,
    synthetic_truth_decision_value: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
enum Eligibility {
    Eligible,
    Ineligible,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct ModelScore {
    model_id: String,
    decision_value: f64,
    information_gain: f64,
    error_amplification_risk: f64,
}

// This is the only representation passed into acquisition scoring. It has no
// field for synthetic_truth_decision_value by construction.
#[derive(Clone, Debug)]
struct PolicyAction {
    index: usize,
    action_id: String,
    stratum_id: String,
    cost_units: u32,
    expert_priority: f64,
    ofat_order: u32,
    target_weight: f64,
    model_scores: Vec<ModelScore>,
}

#[derive(Clone, Debug, Serialize)]
struct SelectedAction {
    selection_order: usize,
    action_id: String,
    stratum_id: String,
    cost_units: u32,
    score_at_selection: f64,
    selection_reason: String,
}

#[derive(Clone, Debug, Serialize)]
struct StratumCoverage {
    stratum_id: String,
    target_weight: f64,
    selected_action_count: usize,
    required_minimum_count: usize,
    minimum_met: bool,
    selected_fraction_of_target_minimum: f64,
}

#[derive(Clone, Debug, Serialize)]
struct PolicyResult {
    policy_id: String,
    selected_actions: Vec<SelectedAction>,
    total_cost_units: u32,
    budget_remaining_units: u32,
    minimum_coverage_met: bool,
    stratum_coverage: Vec<StratumCoverage>,
    weighted_synthetic_decision_utility: f64,
    oracle_regret: f64,
    normalized_oracle_regret: f64,
    mean_model_disagreement: Option<f64>,
    mean_error_amplification_risk: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct OracleResult {
    selected_action_ids: Vec<String>,
    total_cost_units: u32,
    weighted_synthetic_decision_utility: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    report_version: &'static str,
    status: &'static str,
    scenario_id: String,
    input_sha256: String,
    candidate_count: usize,
    eligible_candidate_count: usize,
    ineligible_candidate_count: usize,
    budget_cost_units: u32,
    oracle: OracleResult,
    policy_results: Vec<PolicyResult>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

#[derive(Clone, Copy, Debug)]
enum Policy {
    RandomFeasible,
    ExpertFixed,
    OneFactorAtATime,
    InformationGainOnly,
    RepresentativeOnly,
    RideaInspired,
    RobustDecisionFocused,
}

impl Policy {
    const ALL: [Policy; 7] = [
        Policy::RandomFeasible,
        Policy::ExpertFixed,
        Policy::OneFactorAtATime,
        Policy::InformationGainOnly,
        Policy::RepresentativeOnly,
        Policy::RideaInspired,
        Policy::RobustDecisionFocused,
    ];

    fn id(self) -> &'static str {
        match self {
            Policy::RandomFeasible => "random_feasible",
            Policy::ExpertFixed => "expert_fixed",
            Policy::OneFactorAtATime => "one_factor_at_a_time",
            Policy::InformationGainOnly => "bayesian_eig_proxy",
            Policy::RepresentativeOnly => "representative_only",
            Policy::RideaInspired => "ridea_inspired_comparator",
            Policy::RobustDecisionFocused => "robust_decision_focused",
        }
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn in_unit_interval(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn validate_scenario(scenario: &Scenario) -> Result<(), Box<dyn Error>> {
    if scenario.schema_version != VERSION {
        return Err(format!("unsupported schema_version {:?}", scenario.schema_version).into());
    }
    if scenario.data_class != "synthetic_known_truth" {
        return Err("this v0.1 harness accepts synthetic_known_truth scenarios only".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must not be blank".into());
    }
    if scenario.budget_cost_units == 0 {
        return Err("budget_cost_units must be positive".into());
    }
    if scenario.actions.is_empty() || scenario.actions.len() > MAX_ORACLE_CANDIDATES {
        return Err(format!(
            "action count must be between 1 and {MAX_ORACLE_CANDIDATES} for exact oracle enumeration"
        ).into());
    }
    if scenario.strata.is_empty() {
        return Err("at least one target stratum is required".into());
    }

    let mut stratum_ids = BTreeSet::<String>::new();
    let mut weight_sum = 0.0;
    for stratum in &scenario.strata {
        if stratum.stratum_id.trim().is_empty() || !stratum_ids.insert(stratum.stratum_id.clone()) {
            return Err("stratum identifiers must be non-empty and unique".into());
        }
        if !in_unit_interval(stratum.target_weight) {
            return Err(format!("invalid target weight for {:?}", stratum.stratum_id).into());
        }
        weight_sum += stratum.target_weight;
    }
    if (weight_sum - 1.0).abs() > 1e-6 {
        return Err(format!("target stratum weights must sum to 1.0; found {weight_sum}").into());
    }

    let mut action_ids = BTreeSet::<String>::new();
    let mut ofat_orders = BTreeSet::new();
    let mut expected_model_ids: Option<Vec<String>> = None;
    for action in &scenario.actions {
        if action.action_id.trim().is_empty() || !action_ids.insert(action.action_id.clone()) {
            return Err("action identifiers must be non-empty and unique".into());
        }
        if !stratum_ids.contains(&action.stratum_id) {
            return Err(format!("unknown stratum {:?} on action {:?}", action.stratum_id, action.action_id).into());
        }
        if action.cost_units == 0 {
            return Err(format!("action {:?} must have positive cost", action.action_id).into());
        }
        if !in_unit_interval(action.expert_priority) ||
            !in_unit_interval(action.synthetic_truth_decision_value)
        {
            return Err(format!("action {:?} has score outside [0,1]", action.action_id).into());
        }
        if action.ofat_order == 0 || !ofat_orders.insert(action.ofat_order) {
            return Err("OFAT order values must be non-zero and unique".into());
        }
        match action.eligibility {
            Eligibility::Eligible if action.exclusion_reason.is_some() => {
                return Err(format!("eligible action {:?} has an exclusion reason", action.action_id).into());
            }
            Eligibility::Ineligible if action.exclusion_reason.as_deref().unwrap_or("").trim().is_empty() => {
                return Err(format!("ineligible action {:?} must state an exclusion reason", action.action_id).into());
            }
            _ => {}
        }
        if action.model_scores.len() < 2 {
            return Err(format!("action {:?} needs at least two model scores", action.action_id).into());
        }
        let mut model_ids = Vec::new();
        for score in &action.model_scores {
            if score.model_id.trim().is_empty() ||
                !in_unit_interval(score.decision_value) ||
                !in_unit_interval(score.information_gain) ||
                !in_unit_interval(score.error_amplification_risk)
            {
                return Err(format!("invalid model score on action {:?}", action.action_id).into());
            }
            model_ids.push(score.model_id.clone());
        }
        model_ids.sort();
        if model_ids.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(format!("duplicate model score ID on action {:?}", action.action_id).into());
        }
        match &expected_model_ids {
            Some(expected) if expected != &model_ids => {
                return Err("all actions must have the same set of candidate model IDs".into());
            }
            None => expected_model_ids = Some(model_ids),
            _ => {}
        }
    }

    for stratum in &scenario.strata {
        if stratum.minimum_selected_actions > 0 &&
            !scenario.actions.iter().any(|a|
                a.stratum_id == stratum.stratum_id && a.eligibility == Eligibility::Eligible)
        {
            return Err(format!("required stratum {:?} has no eligible action", stratum.stratum_id).into());
        }
    }
    if !minimum_coverage_feasible(scenario) {
        return Err("declared minimum stratum coverage cannot be met within the cost budget".into());
    }
    Ok(())
}

fn minimum_coverage_feasible(scenario: &Scenario) -> bool {
    // Independent strata and additive costs make this lower bound exact for
    // the minimum-count feasibility check.
    let mut minimum_cost = 0u32;
    for stratum in &scenario.strata {
        let mut costs: Vec<u32> = scenario.actions.iter()
            .filter(|a| a.stratum_id == stratum.stratum_id && a.eligibility == Eligibility::Eligible)
            .map(|a| a.cost_units)
            .collect();
        costs.sort_unstable();
        if costs.len() < stratum.minimum_selected_actions {
            return false;
        }
        for cost in costs.into_iter().take(stratum.minimum_selected_actions) {
            let Some(next) = minimum_cost.checked_add(cost) else { return false; };
            minimum_cost = next;
        }
    }
    minimum_cost <= scenario.budget_cost_units
}

fn scoring_actions(scenario: &Scenario) -> Vec<PolicyAction> {
    let weights: BTreeMap<&str, f64> = scenario.strata.iter()
        .map(|s| (s.stratum_id.as_str(), s.target_weight))
        .collect();
    scenario.actions.iter().enumerate()
        .filter(|(_, action)| action.eligibility == Eligibility::Eligible)
        .map(|(index, action)| PolicyAction {
            index,
            action_id: action.action_id.clone(),
            stratum_id: action.stratum_id.clone(),
            cost_units: action.cost_units,
            expert_priority: action.expert_priority,
            ofat_order: action.ofat_order,
            target_weight: *weights.get(action.stratum_id.as_str()).expect("validated stratum"),
            model_scores: action.model_scores.clone(),
        })
        .collect()
}

fn minimum_model_decision_value(action: &PolicyAction) -> f64 {
    action.model_scores.iter().map(|m| m.decision_value)
        .fold(f64::INFINITY, f64::min)
}

fn mean_information_gain(action: &PolicyAction) -> f64 {
    action.model_scores.iter().map(|m| m.information_gain).sum::<f64>()
        / action.model_scores.len() as f64
}

fn disagreement(action: &PolicyAction) -> f64 {
    let values: Vec<f64> = action.model_scores.iter().map(|m| m.decision_value).collect();
    values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - values.iter().copied().fold(f64::INFINITY, f64::min)
}

fn mean_risk(action: &PolicyAction) -> f64 {
    action.model_scores.iter().map(|m| m.error_amplification_risk).sum::<f64>()
        / action.model_scores.len() as f64
}

fn representation_score(action: &PolicyAction, selected_counts: &BTreeMap<String, usize>) -> f64 {
    let count = selected_counts.get(&action.stratum_id).copied().unwrap_or(0);
    action.target_weight / (1.0 + count as f64)
}

fn robust_score(action: &PolicyAction, selected_counts: &BTreeMap<String, usize>) -> f64 {
    let min_decision = minimum_model_decision_value(action);
    let information = mean_information_gain(action);
    let risk = mean_risk(action);
    let rep = representation_score(action, selected_counts);
    let disagreement = disagreement(action);

    // Conservative decision value, information gain, representation and
    // de-amplification compete explicitly. Model disagreement is penalized.
    (0.25 * min_decision
        + 0.15 * information
        + 0.15 * rep
        + 0.35 * (1.0 - risk)
        - 0.25 * disagreement)
        / action.cost_units as f64
}

fn ridea_inspired_score(action: &PolicyAction, selected_counts: &BTreeMap<String, usize>) -> f64 {
    let information = mean_information_gain(action);
    let representativeness = representation_score(action, selected_counts);
    let deamplification = 1.0 - mean_risk(action);
    (0.30 * representativeness + 0.40 * information + 0.30 * deamplification)
        / action.cost_units as f64
}

fn deterministic_shuffle<T>(items: &mut [T], seed: u64) {
    let mut state = if seed == 0 { 0x9e3779b97f4a7c15 } else { seed };
    for index in (1..items.len()).rev() {
        // xorshift64*: deterministic pseudo-random ordering, not cryptographic.
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        let random = state.wrapping_mul(0x2545f4914f6cdd1d);
        let swap_index = (random as usize) % (index + 1);
        items.swap(index, swap_index);
    }
}

fn policy_score(
    policy: Policy,
    action: &PolicyAction,
    selected_counts: &BTreeMap<String, usize>,
) -> f64 {
    match policy {
        Policy::RandomFeasible => 0.0,
        Policy::ExpertFixed => action.expert_priority / action.cost_units as f64,
        Policy::OneFactorAtATime => -(action.ofat_order as f64),
        Policy::InformationGainOnly => {
            mean_information_gain(action) / action.cost_units as f64
        }
        Policy::RepresentativeOnly => {
            representation_score(action, selected_counts) / action.cost_units as f64
        }
        Policy::RideaInspired => ridea_inspired_score(action, selected_counts),
        Policy::RobustDecisionFocused => robust_score(action, selected_counts),
    }
}

fn selection_reason(policy: Policy, coverage_reservation: bool) -> String {
    if coverage_reservation {
        "minimum_target_stratum_coverage".to_owned()
    } else {
        match policy {
            Policy::RandomFeasible => "seeded_random_feasible_order".to_owned(),
            Policy::ExpertFixed => "preregistered_expert_priority".to_owned(),
            Policy::OneFactorAtATime => "preregistered_ofat_order".to_owned(),
            Policy::InformationGainOnly => "mean_model_information_gain_per_cost".to_owned(),
            Policy::RepresentativeOnly => "target_weight_and_coverage_deficit_per_cost".to_owned(),
            Policy::RideaInspired => "representative_informative_deamplifying_composite".to_owned(),
            Policy::RobustDecisionFocused => "conservative_decision_value_with_risk_and_coverage".to_owned(),
        }
    }
}

fn preserves_minimum_coverage(
    scenario: &Scenario,
    actions: &[PolicyAction],
    selected_ids: &BTreeSet<String>,
    selected_counts: &BTreeMap<String, usize>,
    candidate: &PolicyAction,
    remaining_budget: u32,
) -> bool {
    let Some(budget_after_candidate) = remaining_budget.checked_sub(candidate.cost_units) else {
        return false;
    };
    let mut reserved_cost = 0u32;
    for stratum in &scenario.strata {
        let already_selected = selected_counts.get(&stratum.stratum_id).copied().unwrap_or(0);
        let candidate_increment = if candidate.stratum_id == stratum.stratum_id { 1 } else { 0 };
        let still_needed = stratum.minimum_selected_actions
            .saturating_sub(already_selected + candidate_increment);
        if still_needed == 0 {
            continue;
        }
        let mut remaining_costs: Vec<u32> = actions.iter()
            .filter(|action| {
                action.stratum_id == stratum.stratum_id
                    && action.action_id != candidate.action_id
                    && !selected_ids.contains(&action.action_id)
            })
            .map(|action| action.cost_units)
            .collect();
        remaining_costs.sort_unstable();
        if remaining_costs.len() < still_needed {
            return false;
        }
        for cost in remaining_costs.into_iter().take(still_needed) {
            let Some(next_cost) = reserved_cost.checked_add(cost) else {
                return false;
            };
            reserved_cost = next_cost;
        }
    }
    reserved_cost <= budget_after_candidate
}

fn select(
    scenario: &Scenario,
    policy: Policy,
    actions: &[PolicyAction],
) -> Vec<(usize, f64, String)> {
    let mut selected = Vec::<(usize, f64, String)>::new();
    let mut selected_ids = BTreeSet::<String>::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut remaining = scenario.budget_cost_units;

    let mut random_order = actions.to_vec();
    random_order.sort_by(|a, b| a.action_id.cmp(&b.action_id));
    deterministic_shuffle(&mut random_order, scenario.seed);

    loop {
        let fitting: Vec<&PolicyAction> = actions.iter()
            .filter(|a| !selected_ids.contains(&a.action_id) && a.cost_units <= remaining)
            .collect();
        if fitting.is_empty() {
            break;
        }

        let deficient: Vec<&Stratum> = scenario.strata.iter()
            .filter(|s| counts.get(&s.stratum_id).copied().unwrap_or(0) < s.minimum_selected_actions)
            .collect();
        let (candidate_pool, coverage_reservation): (Vec<&PolicyAction>, bool) =
            if matches!(policy, Policy::RobustDecisionFocused) && !deficient.is_empty() {
                let deficit_ids: BTreeSet<&str> = deficient.iter().map(|s| s.stratum_id.as_str()).collect();
                let coverage_candidates: Vec<&PolicyAction> = fitting.iter().copied()
                    .filter(|a| {
                        deficit_ids.contains(a.stratum_id.as_str())
                            && preserves_minimum_coverage(
                                scenario, actions, &selected_ids, &counts, a, remaining
                            )
                    })
                    .collect();
                if coverage_candidates.is_empty() {
                    break;
                }
                (coverage_candidates, true)
            } else {
                (fitting, false)
            };

        let chosen: &PolicyAction;
        let chosen_score: f64;
        if matches!(policy, Policy::RandomFeasible) {
            let candidate_ids: BTreeSet<&str> = candidate_pool.iter().map(|a| a.action_id.as_str()).collect();
            let next = random_order.iter().find(|a|
                candidate_ids.contains(a.action_id.as_str()) &&
                !selected_ids.contains(&a.action_id) &&
                a.cost_units <= remaining
            );
            let Some(next) = next else { break; };
            chosen = actions.iter().find(|a| a.action_id == next.action_id).expect("same candidate");
            chosen_score = 0.0;
        } else if matches!(policy, Policy::OneFactorAtATime) {
            chosen = *candidate_pool.iter().min_by(|a, b|
                a.ofat_order.cmp(&b.ofat_order).then_with(|| a.action_id.cmp(&b.action_id))
            ).expect("fitting candidate pool non-empty");
            chosen_score = policy_score(policy, chosen, &counts);
        } else {
            let (best, score) = candidate_pool.iter()
                .map(|a| (*a, policy_score(policy, a, &counts)))
                .max_by(|(a, score_a), (b, score_b)|
                    score_a.total_cmp(score_b).then_with(|| b.action_id.cmp(&a.action_id))
                ).expect("fitting candidate pool non-empty");
            chosen = best;
            chosen_score = score;
        }

        remaining -= chosen.cost_units;
        selected_ids.insert(chosen.action_id.clone());
        *counts.entry(chosen.stratum_id.clone()).or_default() += 1;
        selected.push((
            chosen.index,
            chosen_score,
            selection_reason(policy, coverage_reservation),
        ));
    }
    selected
}

fn utility(actions: &[Action], strata: &[Stratum], selected: &[usize]) -> f64 {
    let weights: BTreeMap<&str, f64> = strata.iter()
        .map(|s| (s.stratum_id.as_str(), s.target_weight)).collect();
    selected.iter().map(|index| {
        let action = &actions[*index];
        weights[action.stratum_id.as_str()] * action.synthetic_truth_decision_value
    }).sum()
}

fn oracle(scenario: &Scenario) -> Result<OracleResult, Box<dyn Error>> {
    let n = scenario.actions.len();
    let mut best_value = f64::NEG_INFINITY;
    let mut best_cost = 0u32;
    let mut best_indices = Vec::<usize>::new();

    for mask in 0u64..(1u64 << n) {
        let mut cost = 0u32;
        let mut selected = Vec::new();
        let mut counts = BTreeMap::<&str, usize>::new();
        let mut invalid = false;
        for index in 0..n {
            if mask & (1u64 << index) == 0 { continue; }
            let action = &scenario.actions[index];
            if action.eligibility != Eligibility::Eligible {
                invalid = true;
                break;
            }
            let Some(next_cost) = cost.checked_add(action.cost_units) else {
                invalid = true;
                break;
            };
            cost = next_cost;
            if cost > scenario.budget_cost_units {
                invalid = true;
                break;
            }
            *counts.entry(action.stratum_id.as_str()).or_default() += 1;
            selected.push(index);
        }
        if invalid { continue; }
        if scenario.strata.iter().any(|s|
            counts.get(s.stratum_id.as_str()).copied().unwrap_or(0) < s.minimum_selected_actions
        ) {
            continue;
        }
        let value = utility(&scenario.actions, &scenario.strata, &selected);
        let mut ids: Vec<String> = selected.iter().map(|i| scenario.actions[*i].action_id.clone()).collect();
        ids.sort();
        let mut best_ids: Vec<String> = best_indices.iter().map(|i| scenario.actions[*i].action_id.clone()).collect();
        best_ids.sort();
        if value > best_value + f64::EPSILON ||
            ((value - best_value).abs() <= f64::EPSILON && (best_ids.is_empty() || ids < best_ids))
        {
            best_value = value;
            best_cost = cost;
            best_indices = selected;
        }
    }
    if !best_value.is_finite() {
        return Err("no feasible oracle subset satisfies budget and minimum coverage".into());
    }
    let mut selected_action_ids: Vec<String> =
        best_indices.iter().map(|i| scenario.actions[*i].action_id.clone()).collect();
    selected_action_ids.sort();
    Ok(OracleResult {
        selected_action_ids,
        total_cost_units: best_cost,
        weighted_synthetic_decision_utility: best_value,
    })
}

fn summarize_policy(
    scenario: &Scenario,
    policy: Policy,
    selection: Vec<(usize, f64, String)>,
    oracle: &OracleResult,
) -> PolicyResult {
    let indices: Vec<usize> = selection.iter().map(|(i, _, _)| *i).collect();
    let total_cost: u32 = indices.iter().map(|i| scenario.actions[*i].cost_units).sum();
    let mut coverage = Vec::new();
    let mut counts = BTreeMap::<&str, usize>::new();
    for index in &indices {
        *counts.entry(scenario.actions[*index].stratum_id.as_str()).or_default() += 1;
    }
    for stratum in &scenario.strata {
        let count = counts.get(stratum.stratum_id.as_str()).copied().unwrap_or(0);
        coverage.push(StratumCoverage {
            stratum_id: stratum.stratum_id.clone(),
            target_weight: stratum.target_weight,
            selected_action_count: count,
            required_minimum_count: stratum.minimum_selected_actions,
            minimum_met: count >= stratum.minimum_selected_actions,
            selected_fraction_of_target_minimum: if stratum.minimum_selected_actions == 0 {
                1.0
            } else {
                (count as f64 / stratum.minimum_selected_actions as f64).min(1.0)
            },
        });
    }
    let value = utility(&scenario.actions, &scenario.strata, &indices);
    let regret = (oracle.weighted_synthetic_decision_utility - value).max(0.0);
    let selected: Vec<&Action> = indices.iter().map(|i| &scenario.actions[*i]).collect();
    let mean_disagreement = if selected.is_empty() { None } else {
        Some(selected.iter().map(|action| {
            let values: Vec<f64> = action.model_scores.iter().map(|m| m.decision_value).collect();
            values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                - values.iter().copied().fold(f64::INFINITY, f64::min)
        }).sum::<f64>() / selected.len() as f64)
    };
    let mean_risk = if selected.is_empty() { None } else {
        Some(selected.iter().map(|action| {
            action.model_scores.iter().map(|m| m.error_amplification_risk).sum::<f64>()
                / action.model_scores.len() as f64
        }).sum::<f64>() / selected.len() as f64)
    };
    let selected_actions = selection.iter().enumerate().map(|(order, (index, score, reason))| {
        let action = &scenario.actions[*index];
        SelectedAction {
            selection_order: order + 1,
            action_id: action.action_id.clone(),
            stratum_id: action.stratum_id.clone(),
            cost_units: action.cost_units,
            score_at_selection: *score,
            selection_reason: reason.clone(),
        }
    }).collect();
    PolicyResult {
        policy_id: policy.id().to_owned(),
        selected_actions,
        total_cost_units: total_cost,
        budget_remaining_units: scenario.budget_cost_units.saturating_sub(total_cost),
        minimum_coverage_met: coverage.iter().all(|item| item.minimum_met),
        stratum_coverage: coverage,
        weighted_synthetic_decision_utility: value,
        oracle_regret: regret,
        normalized_oracle_regret: if oracle.weighted_synthetic_decision_utility > 0.0 {
            regret / oracle.weighted_synthetic_decision_utility
        } else { 0.0 },
        mean_model_disagreement: mean_disagreement,
        mean_error_amplification_risk: mean_risk,
    }
}

fn run_benchmark(bytes: &[u8]) -> Result<Report, Box<dyn Error>> {
    let scenario: Scenario = serde_json::from_slice(bytes)?;
    validate_scenario(&scenario)?;
    let oracle_result = oracle(&scenario)?;
    let score_actions = scoring_actions(&scenario);
    let mut results = Vec::new();

    for policy in Policy::ALL {
        let selection = select(&scenario, policy, &score_actions);
        results.push(summarize_policy(&scenario, policy, selection, &oracle_result));
    }

    Ok(Report {
        report_version: VERSION,
        status: "synthetic_benchmark_only",
        scenario_id: scenario.scenario_id.clone(),
        input_sha256: sha256_hex(bytes),
        candidate_count: scenario.actions.len(),
        eligible_candidate_count: scenario.actions.iter().filter(|a| a.eligibility == Eligibility::Eligible).count(),
        ineligible_candidate_count: scenario.actions.iter().filter(|a| a.eligibility == Eligibility::Ineligible).count(),
        budget_cost_units: scenario.budget_cost_units,
        oracle: oracle_result,
        policy_results: results,
        warnings: vec![
            "Evaluator-only synthetic truth is excluded from the PolicyAction type and used only after selection.".to_owned(),
            "The R-IDeA-inspired comparator is a transparent benchmark score, not a reproduction of the published R-IDeA method.".to_owned(),
            "Results are specific to this synthetic scenario and do not establish soil efficacy or general policy superiority.".to_owned(),
            "Eligibility is a supplied synthetic flag; this tool has no permission, safety, regulatory, landholder, or institutional authority.".to_owned(),
        ],
        interpretation_boundary: "Synthetic acquisition-policy software benchmark only; no empirical inference, field experiment authorization, material recommendation, safety approval or efficacy claim.",
    })
}

fn main_run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <synthetic-scenario.json> <report.json>",
            args.first().map(String::as_str).unwrap_or("terra-preta-adaptive-design-bakeoff")
        ).into());
    }
    let input = Path::new(&args[1]);
    let output = Path::new(&args[2]);
    if output.exists() && fs::canonicalize(input)? == fs::canonicalize(output)? {
        return Err("input and output resolve to the same file; refusing to overwrite scenario".into());
    }
    let bytes = fs::read(input)?;
    let report = run_benchmark(&bytes)?;
    fs::write(output, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "synthetic adaptive-design bake-off: {} candidates ({} eligible), {} policies, oracle utility {:.4}",
        report.candidate_count, report.eligible_candidate_count,
        report.policy_results.len(), report.oracle.weighted_synthetic_decision_utility
    );
    for result in &report.policy_results {
        println!(
            "{}: utility {:.4}, normalized regret {:.4}, coverage={}, cost={}",
            result.policy_id, result.weighted_synthetic_decision_utility,
            result.normalized_oracle_regret, result.minimum_coverage_met, result.total_cost_units
        );
    }
    Ok(())
}

fn main() {
    if let Err(error) = main_run() {
        eprintln!("adaptive design bake-off failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/synthetic-adaptive-scenario.json");

    fn report() -> Report {
        run_benchmark(FIXTURE.as_bytes()).unwrap()
    }

    #[test]
    fn runs_all_policies_and_exact_truth_oracle() {
        let report = report();
        assert_eq!(report.policy_results.len(), 7);
        assert_eq!(report.candidate_count, 8);
        assert_eq!(report.eligible_candidate_count, 7);
        assert_eq!(report.ineligible_candidate_count, 1);
        assert!((report.oracle.weighted_synthetic_decision_utility - 1.36).abs() < 1e-9);
        assert_eq!(report.oracle.selected_action_ids.len(), 4);
    }

    #[test]
    fn no_policy_selects_ineligible_high_score_action() {
        let report = report();
        for policy in report.policy_results {
            assert!(policy.selected_actions.iter().all(|action| action.action_id != "X_INELIGIBLE"));
            assert!(policy.total_cost_units <= 4);
        }
    }

    #[test]
    fn robust_policy_meets_target_coverage_and_beats_information_only_and_expert_on_fixture() {
        let report = report();
        let robust = report.policy_results.iter().find(|p| p.policy_id == "robust_decision_focused").unwrap();
        let eig = report.policy_results.iter().find(|p| p.policy_id == "bayesian_eig_proxy").unwrap();
        let expert = report.policy_results.iter().find(|p| p.policy_id == "expert_fixed").unwrap();
        assert!(robust.minimum_coverage_met);
        assert!(robust.normalized_oracle_regret < eig.normalized_oracle_regret);
        assert!(robust.normalized_oracle_regret < expert.normalized_oracle_regret);
    }

    #[test]
    fn robust_policy_reserves_budget_for_other_required_target_strata() {
        let mut scenario: Scenario = serde_json::from_str(FIXTURE).unwrap();
        scenario.budget_cost_units = 3;
        let clay_high_info = scenario.actions.iter_mut()
            .find(|action| action.action_id == "C1_CLAY_HIGH_EIG_MISSPECIFIED")
            .unwrap();
        clay_high_info.cost_units = 2;
        for score in &mut clay_high_info.model_scores {
            score.decision_value = 1.0;
            score.information_gain = 1.0;
            score.error_amplification_risk = 0.0;
        }
        validate_scenario(&scenario).unwrap();
        let scoring = scoring_actions(&scenario);
        let selection = select(&scenario, Policy::RobustDecisionFocused, &scoring);
        let ids: Vec<String> = selection.iter().map(|(index, _, _)| {
            scenario.actions[*index].action_id.clone()
        }).collect();
        assert!(ids.contains(&"C2_CLAY_INDEPENDENT_CORE".to_owned()));
        assert!(ids.contains(&"W1_WEATHERED_REFERENCE".to_owned()));
        assert!(!ids.contains(&"C1_CLAY_HIGH_EIG_MISSPECIFIED".to_owned()));
    }

    #[test]
    fn information_only_policy_is_attracted_to_high_information_misspecified_actions() {
        let scenario: Scenario = serde_json::from_str(FIXTURE).unwrap();
        let scoring = scoring_actions(&scenario);
        let selection = select(&scenario, Policy::InformationGainOnly, &scoring);
        let ids: Vec<String> = selection.iter().map(|(index, _, _)| {
            scenario.actions[*index].action_id.clone()
        }).collect();
        assert!(ids.contains(&"S3_SANDY_HIGH_EIG_MISSPECIFIED".to_owned()));
        assert!(ids.contains(&"C1_CLAY_HIGH_EIG_MISSPECIFIED".to_owned()));
    }

    #[test]
    fn expert_priority_can_leave_a_target_stratum_uncovered() {
        let report = report();
        let expert = report.policy_results.iter()
            .find(|policy| policy.policy_id == "expert_fixed").unwrap();
        assert!(!expert.minimum_coverage_met);
        assert!((expert.weighted_synthetic_decision_utility - 0.99).abs() < 1e-9);
    }

    #[test]
    fn evaluator_truth_changes_do_not_change_policy_selections() {
        let mut scenario: Scenario = serde_json::from_str(FIXTURE).unwrap();
        let original_actions = scoring_actions(&scenario);
        for policy in Policy::ALL {
            let original: Vec<usize> = select(&scenario, policy, &original_actions)
                .iter().map(|(index, _, _)| *index).collect();
            for action in &mut scenario.actions {
                action.synthetic_truth_decision_value =
                    1.0 - action.synthetic_truth_decision_value;
            }
            let changed_actions = scoring_actions(&scenario);
            let changed: Vec<usize> = select(&scenario, policy, &changed_actions)
                .iter().map(|(index, _, _)| *index).collect();
            assert_eq!(original, changed, "truth leaked into policy {:?}", policy.id());
            // Restore the known truth before checking the next policy.
            scenario = serde_json::from_str(FIXTURE).unwrap();
        }
    }

    #[test]
    fn random_policy_is_reproducible_for_fixed_seed() {
        let a = report();
        let b = report();
        let ids = |r: &Report| r.policy_results.iter()
            .find(|p| p.policy_id == "random_feasible").unwrap()
            .selected_actions.iter().map(|a| a.action_id.clone()).collect::<Vec<_>>();
        assert_eq!(ids(&a), ids(&b));
    }

    #[test]
    fn rejects_empirical_data_class() {
        let mut scenario: Scenario = serde_json::from_str(FIXTURE).unwrap();
        scenario.data_class = "empirical_soil_data".to_owned();
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn rejects_duplicate_or_inconsistent_model_scores() {
        let mut scenario: Scenario = serde_json::from_str(FIXTURE).unwrap();
        scenario.actions[0].model_scores[0].model_id = "m2".to_owned();
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn rejects_unmet_minimum_coverage_that_exceeds_budget() {
        let mut scenario: Scenario = serde_json::from_str(FIXTURE).unwrap();
        scenario.budget_cost_units = 2;
        assert!(validate_scenario(&scenario).is_err());
    }
}

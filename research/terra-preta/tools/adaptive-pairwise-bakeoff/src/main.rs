//! Synthetic-only benchmark for redundant and complementary measurements.
//!
//! Selector inputs intentionally exclude all synthetic truth fields. The
//! evaluator reads hidden standalone and pair truth only after selection.
//! This is a static pairwise look-ahead benchmark, not a sequential Bayesian
//! posterior engine and not an authorization system for field work.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;

const VERSION: &str = "0.1.0";
const MAX_ORACLE_ACTIONS: usize = 20;

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    seed: u64,
    budget_cost_units: u32,
    strata: Vec<Stratum>,
    actions: Vec<Action>,
    pair_relations: Vec<PairRelation>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Stratum {
    stratum_id: String,
    minimum_selected_actions: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Action {
    action_id: String,
    stratum_id: String,
    cost_units: u32,
    eligible: bool,
    exclusion_reason: Option<String>,
    predicted_standalone_value: f64,
    synthetic_truth_standalone_value: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct PairRelation {
    action_a: String,
    action_b: String,
    estimated_interaction: f64,
    synthetic_truth_interaction: f64,
}

#[derive(Clone, Debug)]
struct PolicyContext {
    seed: u64,
    budget_cost_units: u32,
    strata: Vec<PolicyStratum>,
}

#[derive(Clone, Debug)]
struct PolicyStratum {
    stratum_id: String,
    minimum_selected_actions: usize,
}

// The selector-facing representation has no evaluator-truth fields.
#[derive(Clone, Debug)]
struct PolicyAction {
    action_id: String,
    stratum_id: String,
    cost_units: u32,
    predicted_standalone_value: f64,
}

#[derive(Clone, Debug)]
struct SelectorPair {
    action_a: String,
    action_b: String,
    estimated_interaction: f64,
}

#[derive(Clone, Debug, Serialize)]
struct PolicyResult {
    policy_id: String,
    selected_action_ids: Vec<String>,
    total_cost_units: u32,
    budget_remaining_units: u32,
    minimum_coverage_met: bool,
    selector_predicted_utility: f64,
    evaluator_synthetic_truth_utility: f64,
    oracle_regret: Option<f64>,
    normalized_oracle_regret: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct OracleResult {
    selected_action_ids: Vec<String>,
    total_cost_units: u32,
    synthetic_truth_utility: f64,
}

#[derive(Clone, Debug, Serialize)]
struct PairDiagnostic {
    action_a: String,
    action_b: String,
    estimated_interaction: f64,
    synthetic_truth_interaction: f64,
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
    pair_relation_count: usize,
    budget_cost_units: u32,
    oracle: OracleResult,
    pair_diagnostics: Vec<PairDiagnostic>,
    policy_results: Vec<PolicyResult>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

#[derive(Clone, Copy, Debug)]
enum Policy {
    SeededRandomFeasible,
    StaticAdditiveGreedy,
    PairAwareJointLookahead,
}

impl Policy {
    const ALL: [Policy; 3] = [
        Policy::SeededRandomFeasible,
        Policy::StaticAdditiveGreedy,
        Policy::PairAwareJointLookahead,
    ];

    fn id(self) -> &'static str {
        match self {
            Policy::SeededRandomFeasible => "seeded_random_feasible",
            Policy::StaticAdditiveGreedy => "static_additive_greedy",
            Policy::PairAwareJointLookahead => "pair_aware_joint_lookahead",
        }
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn canonical_pair(a: &str, b: &str) -> (String, String) {
    if a <= b {
        (a.to_owned(), b.to_owned())
    } else {
        (b.to_owned(), a.to_owned())
    }
}

fn in_range(value: f64, low: f64, high: f64) -> bool {
    value.is_finite() && (low..=high).contains(&value)
}

fn minimum_coverage_feasible(scenario: &Scenario) -> bool {
    let mut minimum_cost = 0u32;
    for stratum in &scenario.strata {
        let mut costs: Vec<u32> = scenario.actions.iter()
            .filter(|a| a.eligible && a.stratum_id == stratum.stratum_id)
            .map(|a| a.cost_units)
            .collect();
        costs.sort_unstable();
        if costs.len() < stratum.minimum_selected_actions {
            return false;
        }
        for cost in costs.into_iter().take(stratum.minimum_selected_actions) {
            let Some(total) = minimum_cost.checked_add(cost) else {
                return false;
            };
            minimum_cost = total;
        }
    }
    minimum_cost <= scenario.budget_cost_units
}

fn validate_scenario(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != VERSION {
        return Err(format!("unsupported schema_version {:?}", scenario.schema_version));
    }
    if scenario.data_class != "synthetic_known_truth" {
        return Err("this tool accepts synthetic_known_truth scenarios only".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must not be blank".into());
    }
    if scenario.budget_cost_units == 0 {
        return Err("budget_cost_units must be positive".into());
    }
    if scenario.actions.is_empty() || scenario.actions.len() > MAX_ORACLE_ACTIONS {
        return Err(format!("action count must be 1..={MAX_ORACLE_ACTIONS}"));
    }
    if scenario.strata.is_empty() {
        return Err("at least one target stratum is required".into());
    }

    let mut stratum_ids = BTreeSet::new();
    for stratum in &scenario.strata {
        if stratum.stratum_id.trim().is_empty()
            || !stratum_ids.insert(stratum.stratum_id.as_str())
        {
            return Err("stratum IDs must be non-empty and unique".into());
        }
    }

    let mut action_ids = BTreeSet::new();
    for action in &scenario.actions {
        if action.action_id.trim().is_empty()
            || !action_ids.insert(action.action_id.as_str())
        {
            return Err("action IDs must be non-empty and unique".into());
        }
        if !stratum_ids.contains(action.stratum_id.as_str()) {
            return Err(format!("unknown stratum on action {:?}", action.action_id));
        }
        if action.cost_units == 0 {
            return Err(format!("action {:?} must have positive cost", action.action_id));
        }
        if !in_range(action.predicted_standalone_value, 0.0, 1.0)
            || !in_range(action.synthetic_truth_standalone_value, 0.0, 1.0)
        {
            return Err(format!("standalone value outside [0,1] on {:?}", action.action_id));
        }
        if action.eligible && action.exclusion_reason.is_some() {
            return Err(format!("eligible action {:?} has exclusion_reason", action.action_id));
        }
        if !action.eligible
            && action.exclusion_reason.as_deref().unwrap_or("").trim().is_empty()
        {
            return Err(format!("ineligible action {:?} needs exclusion_reason", action.action_id));
        }
    }

    let mut pair_keys = BTreeSet::new();
    for pair in &scenario.pair_relations {
        if pair.action_a == pair.action_b
            || !action_ids.contains(pair.action_a.as_str())
            || !action_ids.contains(pair.action_b.as_str())
        {
            return Err(format!("invalid pair endpoints: {:?}, {:?}", pair.action_a, pair.action_b));
        }
        if !in_range(pair.estimated_interaction, -1.0, 1.0)
            || !in_range(pair.synthetic_truth_interaction, -1.0, 1.0)
        {
            return Err("pair interaction must be finite and within [-1,1]".into());
        }
        if !pair_keys.insert(canonical_pair(&pair.action_a, &pair.action_b)) {
            return Err("duplicate pair relation (including reversed duplicates)".into());
        }
    }

    if !minimum_coverage_feasible(scenario) {
        return Err("minimum target-stratum coverage is infeasible under the budget".into());
    }
    Ok(())
}

fn policy_context(scenario: &Scenario) -> PolicyContext {
    PolicyContext {
        seed: scenario.seed,
        budget_cost_units: scenario.budget_cost_units,
        strata: scenario.strata.iter().map(|s| PolicyStratum {
            stratum_id: s.stratum_id.clone(),
            minimum_selected_actions: s.minimum_selected_actions,
        }).collect(),
    }
}

fn policy_actions(scenario: &Scenario) -> Vec<PolicyAction> {
    scenario.actions.iter().filter(|a| a.eligible).map(|a| PolicyAction {
        action_id: a.action_id.clone(),
        stratum_id: a.stratum_id.clone(),
        cost_units: a.cost_units,
        predicted_standalone_value: a.predicted_standalone_value,
    }).collect()
}

fn selector_pairs(scenario: &Scenario) -> Vec<SelectorPair> {
    scenario.pair_relations.iter().map(|p| SelectorPair {
        action_a: p.action_a.clone(),
        action_b: p.action_b.clone(),
        estimated_interaction: p.estimated_interaction,
    }).collect()
}

fn pair_value(a: &str, b: &str, pairs: &[SelectorPair]) -> f64 {
    let key = canonical_pair(a, b);
    pairs.iter()
        .find(|p| canonical_pair(&p.action_a, &p.action_b) == key)
        .map_or(0.0, |p| p.estimated_interaction)
}

fn seeded_rank(seed: u64, action_id: &str) -> u64 {
    let mut value = seed ^ 0x9e3779b97f4a7c15;
    for byte in action_id.bytes() {
        value ^= u64::from(byte);
        value = value.wrapping_mul(0x100000001b3);
    }
    value ^= value >> 30;
    value = value.wrapping_mul(0xbf58476d1ce4e5b9);
    value ^= value >> 27;
    value = value.wrapping_mul(0x94d049bb133111eb);
    value ^ (value >> 31)
}

fn coverage_met(context: &PolicyContext, actions: &[PolicyAction], selected: &[String]) -> bool {
    context.strata.iter().all(|stratum| {
        let count = selected.iter().filter(|id| {
            actions.iter().any(|a| a.action_id.as_str() == id.as_str() && a.stratum_id == stratum.stratum_id)
        }).count();
        count >= stratum.minimum_selected_actions
    })
}

fn selector_utility(selected: &[String], actions: &[PolicyAction], pairs: &[SelectorPair]) -> f64 {
    let mut total = 0.0;
    for id in selected {
        if let Some(action) = actions.iter().find(|a| a.action_id == *id) {
            total += action.predicted_standalone_value;
        }
    }
    for i in 0..selected.len() {
        for j in (i + 1)..selected.len() {
            total += pair_value(&selected[i], &selected[j], pairs);
        }
    }
    total
}

fn marginal_gain(
    additions: &[&PolicyAction],
    selected: &[String],
    pairs: &[SelectorPair],
) -> f64 {
    let mut gain: f64 = additions.iter().map(|a| a.predicted_standalone_value).sum();
    for i in 0..additions.len() {
        for j in (i + 1)..additions.len() {
            gain += pair_value(&additions[i].action_id, &additions[j].action_id, pairs);
        }
        for existing in selected {
            gain += pair_value(&additions[i].action_id, existing, pairs);
        }
    }
    gain
}

fn select(
    context: &PolicyContext,
    actions: &[PolicyAction],
    pairs: &[SelectorPair],
    policy: Policy,
) -> Result<Vec<String>, String> {
    let mut selected: Vec<String> = Vec::new();
    let mut spent = 0u32;

    // Meet hard coverage minimums first; choose by lowest cost, then highest
    // permitted standalone score per unit cost, then stable action ID.
    for stratum in &context.strata {
        for _ in 0..stratum.minimum_selected_actions {
            let candidate = actions.iter()
                .filter(|a| a.stratum_id == stratum.stratum_id
                    && !selected.contains(&a.action_id)
                    && spent.saturating_add(a.cost_units) <= context.budget_cost_units)
                .min_by(|a, b| {
                    a.cost_units.cmp(&b.cost_units)
                        .then_with(|| {
                            let a_rate = a.predicted_standalone_value / f64::from(a.cost_units);
                            let b_rate = b.predicted_standalone_value / f64::from(b.cost_units);
                            b_rate.total_cmp(&a_rate)
                        })
                        .then_with(|| a.action_id.cmp(&b.action_id))
                })
                .ok_or_else(|| format!("cannot satisfy coverage for {:?}", stratum.stratum_id))?;
            spent = spent.checked_add(candidate.cost_units)
                .ok_or_else(|| "cost overflow".to_owned())?;
            selected.push(candidate.action_id.clone());
        }
    }

    loop {
        let remaining = context.budget_cost_units.saturating_sub(spent);
        if remaining == 0 {
            break;
        }

        let unselected: Vec<&PolicyAction> = actions.iter()
            .filter(|a| !selected.contains(&a.action_id) && a.cost_units <= remaining)
            .collect();
        if unselected.is_empty() {
            break;
        }

        if matches!(policy, Policy::SeededRandomFeasible) {
            let candidate = unselected.iter()
                .min_by_key(|a| seeded_rank(context.seed, a.action_id.as_str()))
                .expect("non-empty candidates");
            spent += candidate.cost_units;
            selected.push(candidate.action_id.clone());
            continue;
        }

        let mut best: Option<(f64, u32, Vec<String>)> = None;
        for action in &unselected {
            let add = [*action];
            let cost = action.cost_units;
            let gain = marginal_gain(&add, &selected, pairs);
            consider_candidate(&mut best, gain, cost, vec![action.action_id.clone()]);
        }

        if matches!(policy, Policy::PairAwareJointLookahead) {
            for i in 0..unselected.len() {
                for j in (i + 1)..unselected.len() {
                    let a = unselected[i];
                    let b = unselected[j];
                    let Some(cost) = a.cost_units.checked_add(b.cost_units) else { continue; };
                    if cost > remaining {
                        continue;
                    }
                    let gain = marginal_gain(&[a, b], &selected, pairs);
                    let mut ids = vec![a.action_id.clone(), b.action_id.clone()];
                    ids.sort();
                    consider_candidate(&mut best, gain, cost, ids);
                }
            }
        }

        let Some((score, cost, ids)) = best else { break; };
        if score <= 0.0 {
            break;
        }
        spent = spent.checked_add(cost).ok_or_else(|| "cost overflow".to_owned())?;
        selected.extend(ids);
        selected.sort();
    }
    Ok(selected)
}

fn consider_candidate(
    best: &mut Option<(f64, u32, Vec<String>)>,
    gain: f64,
    cost: u32,
    mut ids: Vec<String>,
) {
    if cost == 0 {
        return;
    }
    ids.sort();
    let score = gain / f64::from(cost);
    let replace = match best {
        None => true,
        Some((best_score, best_cost, best_ids)) => {
            score.total_cmp(best_score).is_gt()
                || (score.total_cmp(best_score).is_eq()
                    && (cost < *best_cost || (cost == *best_cost && ids < *best_ids)))
        }
    };
    if replace {
        *best = Some((score, cost, ids));
    }
}

fn truth_utility(selected: &[String], scenario: &Scenario) -> f64 {
    let by_id: BTreeMap<&str, &Action> = scenario.actions.iter()
        .map(|a| (a.action_id.as_str(), a))
        .collect();
    let mut total: f64 = selected.iter()
        .filter_map(|id| by_id.get(id.as_str()))
        .map(|a| a.synthetic_truth_standalone_value)
        .sum();
    for i in 0..selected.len() {
        for j in (i + 1)..selected.len() {
            let key = canonical_pair(&selected[i], &selected[j]);
            if let Some(pair) = scenario.pair_relations.iter()
                .find(|p| canonical_pair(&p.action_a, &p.action_b) == key)
            {
                total += pair.synthetic_truth_interaction;
            }
        }
    }
    total
}

fn oracle(scenario: &Scenario) -> OracleResult {
    let eligible: Vec<&Action> = scenario.actions.iter().filter(|a| a.eligible).collect();
    let mut best_ids: Vec<String> = Vec::new();
    let mut best_cost = 0u32;
    let mut best_value = f64::NEG_INFINITY;

    for mask in 0usize..(1usize << eligible.len()) {
        let mut ids = Vec::new();
        let mut cost = 0u32;
        let mut feasible = true;
        for (index, action) in eligible.iter().enumerate() {
            if (mask & (1usize << index)) != 0 {
                let Some(next) = cost.checked_add(action.cost_units) else {
                    feasible = false;
                    break;
                };
                cost = next;
                ids.push(action.action_id.clone());
            }
        }
        if !feasible || cost > scenario.budget_cost_units {
            continue;
        }

        let context = policy_context(scenario);
        let scoring = policy_actions(scenario);
        if !coverage_met(&context, &scoring, &ids) {
            continue;
        }
        let value = truth_utility(&ids, scenario);
        if value.total_cmp(&best_value).is_gt()
            || (value.total_cmp(&best_value).is_eq() && cost < best_cost)
        {
            best_ids = ids;
            best_ids.sort();
            best_cost = cost;
            best_value = value;
        }
    }

    OracleResult {
        selected_action_ids: best_ids,
        total_cost_units: best_cost,
        synthetic_truth_utility: if best_value.is_finite() { best_value } else { 0.0 },
    }
}

fn evaluate_policy(
    context: &PolicyContext,
    actions: &[PolicyAction],
    pairs: &[SelectorPair],
    policy: Policy,
    scenario: &Scenario,
    oracle: &OracleResult,
) -> Result<PolicyResult, String> {
    let selected = select(context, actions, pairs, policy)?;
    let total_cost_units = selected.iter().try_fold(0u32, |sum, id| {
        let cost = actions.iter().find(|a| a.action_id == *id)
            .ok_or_else(|| format!("selected action {:?} is absent", id))?.cost_units;
        sum.checked_add(cost).ok_or_else(|| "cost overflow".to_owned())
    })?;
    let covered = coverage_met(context, actions, &selected);
    let synthetic_truth = truth_utility(&selected, scenario);
    let oracle_regret = if covered {
        Some((oracle.synthetic_truth_utility - synthetic_truth).max(0.0))
    } else {
        None
    };
    let normalized_oracle_regret = oracle_regret.map(|regret| {
        regret / oracle.synthetic_truth_utility.abs().max(1e-12)
    });

    Ok(PolicyResult {
        policy_id: policy.id().to_owned(),
        selected_action_ids: selected.clone(),
        total_cost_units,
        budget_remaining_units: context.budget_cost_units.saturating_sub(total_cost_units),
        minimum_coverage_met: covered,
        selector_predicted_utility: selector_utility(&selected, actions, pairs),
        evaluator_synthetic_truth_utility: synthetic_truth,
        oracle_regret,
        normalized_oracle_regret,
    })
}

fn build_report(scenario: &Scenario, input_bytes: &[u8]) -> Result<Report, String> {
    validate_scenario(scenario)?;
    let context = policy_context(scenario);
    let actions = policy_actions(scenario);
    let pairs = selector_pairs(scenario);
    let oracle_result = oracle(scenario);
    let mut results = Vec::new();
    for policy in Policy::ALL {
        results.push(evaluate_policy(
            &context, &actions, &pairs, policy, scenario, &oracle_result,
        )?);
    }
    let pair_diagnostics = scenario.pair_relations.iter().map(|p| PairDiagnostic {
        action_a: p.action_a.clone(),
        action_b: p.action_b.clone(),
        estimated_interaction: p.estimated_interaction,
        synthetic_truth_interaction: p.synthetic_truth_interaction,
    }).collect();

    Ok(Report {
        report_version: VERSION,
        status: "synthetic_pairwise_benchmark_only",
        scenario_id: scenario.scenario_id.clone(),
        input_sha256: sha256_hex(input_bytes),
        candidate_count: scenario.actions.len(),
        eligible_candidate_count: scenario.actions.iter().filter(|a| a.eligible).count(),
        ineligible_candidate_count: scenario.actions.iter().filter(|a| !a.eligible).count(),
        pair_relation_count: scenario.pair_relations.len(),
        budget_cost_units: scenario.budget_cost_units,
        oracle: oracle_result,
        pair_diagnostics,
        policy_results: results,
        interpretation_boundary: "Synthetic known-truth benchmark only; no empirical soil evidence, field authorization, efficacy claim, or SOTA claim.",
        warnings: vec![
            "The pair-aware policy uses static pairwise interaction estimates and one-step joint look-ahead; it does not perform posterior updates.".into(),
            "Pairwise interactions are an intentionally simplified utility model; higher-order interactions and observation likelihood dependence are not represented.".into(),
            "The exact feasible-subset oracle is restricted to at most 20 eligible actions.".into(),
            "A single designed fixture demonstrates a failure mode; it does not establish general policy superiority.".into(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err("usage: adaptive-pairwise-bakeoff <scenario.json> <report.json>".into());
    }
    let input = fs::read(&args[1])?;
    let scenario: Scenario = serde_json::from_slice(&input)?;
    let report = build_report(&scenario, &input)
        .map_err(|error| format!("scenario rejected: {error}"))?;
    fs::write(&args[2], serde_json::to_vec_pretty(&report)?)?;
    println!(
        "wrote synthetic pairwise report for {} ({} candidates, {} pairs)",
        report.scenario_id, report.candidate_count, report.pair_relation_count
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-pairwise-scenario.json"))
            .expect("valid test fixture")
    }

    fn selected_for(scenario: &Scenario, policy: Policy) -> Vec<String> {
        select(
            &policy_context(scenario),
            &policy_actions(scenario),
            &selector_pairs(scenario),
            policy,
        ).expect("selection succeeds")
    }

    #[test]
    fn pair_aware_policy_recovers_complementary_modalities() {
        let scenario = fixture();
        let selected = selected_for(&scenario, Policy::PairAwareJointLookahead);
        assert!(selected.contains(&"D_CT_GEOMETRY".to_owned()));
        assert!(selected.contains(&"E_EVENT_TRACER".to_owned()));
    }

    #[test]
    fn static_additive_policy_double_counts_redundant_measurements() {
        let scenario = fixture();
        let selected = selected_for(&scenario, Policy::StaticAdditiveGreedy);
        assert!(selected.contains(&"B_TRACER_EARLY".to_owned()));
        assert!(selected.contains(&"C_TRACER_NEARBY".to_owned()));
        assert!(!selected.contains(&"D_CT_GEOMETRY".to_owned()));
        assert!(!selected.contains(&"E_EVENT_TRACER".to_owned()));
    }

    #[test]
    fn exact_oracle_prefers_complementary_pair() {
        let scenario = fixture();
        let result = oracle(&scenario);
        assert!(result.selected_action_ids.contains(&"D_CT_GEOMETRY".to_owned()));
        assert!(result.selected_action_ids.contains(&"E_EVENT_TRACER".to_owned()));
        assert!(!result.selected_action_ids.contains(&"B_TRACER_EARLY".to_owned()));
        assert!(!result.selected_action_ids.contains(&"C_TRACER_NEARBY".to_owned()));
    }

    #[test]
    fn selectors_never_receive_evaluator_truth() {
        let original = fixture();
        let before: Vec<Vec<String>> = Policy::ALL.iter()
            .map(|policy| selected_for(&original, *policy))
            .collect();
        let mut mutated = original.clone();
        for action in &mut mutated.actions {
            action.synthetic_truth_standalone_value =
                1.0 - action.synthetic_truth_standalone_value;
        }
        for pair in &mut mutated.pair_relations {
            pair.synthetic_truth_interaction = -pair.synthetic_truth_interaction;
        }
        let after: Vec<Vec<String>> = Policy::ALL.iter()
            .map(|policy| selected_for(&mutated, *policy))
            .collect();
        assert_eq!(before, after);
    }

    #[test]
    fn ineligible_high_value_action_is_never_selected() {
        let scenario = fixture();
        for policy in Policy::ALL {
            assert!(!selected_for(&scenario, policy).contains(&"X_INELIGIBLE".to_owned()));
        }
    }

    #[test]
    fn every_policy_respects_budget_and_hard_coverage() {
        let scenario = fixture();
        let report = build_report(&scenario, b"fixture").expect("report");
        for result in report.policy_results {
            assert!(result.total_cost_units <= scenario.budget_cost_units);
            assert!(result.minimum_coverage_met);
        }
    }

    #[test]
    fn pair_aware_policy_matches_feasible_oracle_on_designed_fixture() {
        let scenario = fixture();
        let report = build_report(&scenario, b"fixture").expect("report");
        let pairwise = report.policy_results.iter()
            .find(|r| r.policy_id == "pair_aware_joint_lookahead")
            .expect("pairwise result");
        assert_eq!(pairwise.oracle_regret, Some(0.0));
        assert_eq!(
            pairwise.evaluator_synthetic_truth_utility,
            report.oracle.synthetic_truth_utility
        );
    }

    #[test]
    fn pair_relations_are_order_invariant_and_unique() {
        let mut scenario = fixture();
        let mut reversed = scenario.pair_relations[0].clone();
        std::mem::swap(&mut reversed.action_a, &mut reversed.action_b);
        scenario.pair_relations.push(reversed);
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn reject_empirical_data_class() {
        let mut scenario = fixture();
        scenario.data_class = "measured_field_data".into();
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn reject_nan_and_out_of_range_pair_estimates() {
        let mut scenario = fixture();
        scenario.pair_relations[0].estimated_interaction = f64::NAN;
        assert!(validate_scenario(&scenario).is_err());
        let mut scenario = fixture();
        scenario.pair_relations[0].synthetic_truth_interaction = 1.01;
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn reject_missing_coverage_and_infeasible_budget() {
        let mut scenario = fixture();
        scenario.budget_cost_units = 2;
        assert!(validate_scenario(&scenario).is_err());
        let mut scenario = fixture();
        scenario.actions.retain(|a| a.stratum_id != "WEATHERED");
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn pair_synergy_is_not_silently_clipped() {
        let mut scenario = fixture();
        scenario.pair_relations[1].synthetic_truth_interaction = -1.0;
        validate_scenario(&scenario).expect("boundary is valid");
        assert!(truth_utility(
            &["D_CT_GEOMETRY".into(), "E_EVENT_TRACER".into()],
            &scenario
        ) < truth_utility(
            &["D_CT_GEOMETRY".into()],
            &scenario
        ) + scenario.actions.iter()
            .find(|a| a.action_id == "E_EVENT_TRACER")
            .expect("event tracer")
            .synthetic_truth_standalone_value);
    }
}

//! Synthetic-only sequential Bayesian experiment design benchmark.
//!
//! The selector receives only model priors, candidate-model likelihoods,
//! decision utilities, costs, eligibility and target coverage. Evaluator-only
//! physical-world likelihoods and decision utilities are never passed to it.
//! The engine performs exact finite categorical Bayesian updates; it does not
//! claim to solve continuous physical inverse problems or correlated-noise
//! likelihoods.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::error::Error;
use std::fs;

const VERSION: &str = "0.1.0";
const MAX_ACTIONS: usize = 16;
const MIN_REPLICATES: usize = 16;
const MAX_REPLICATES: usize = 4096;
const ALERT_EPSILON: f64 = 1e-12;

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    seed: u64,
    replicates: usize,
    budget_cost_units: u32,
    predictive_surprise_threshold: f64,
    strata: Vec<Stratum>,
    models: Vec<Model>,
    decisions: Vec<Decision>,
    truth_decisions: Vec<TruthDecision>,
    actions: Vec<Action>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Stratum {
    stratum_id: String,
    minimum_selected_actions: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Model {
    model_id: String,
    prior_weight: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Decision {
    decision_id: String,
    model_utilities: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct TruthDecision {
    decision_id: String,
    utility: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct ModelLikelihood {
    model_id: String,
    outcome_probabilities: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Action {
    action_id: String,
    stratum_id: String,
    modality: String,
    cost_units: u32,
    eligible: bool,
    exclusion_reason: Option<String>,
    diagnostic_action: bool,
    outcomes: Vec<String>,
    likelihoods: Vec<ModelLikelihood>,
    truth_outcome_probabilities: Vec<f64>,
}

#[derive(Clone, Debug)]
struct SelectorContext {
    budget_cost_units: u32,
    predictive_surprise_threshold: f64,
    strata: Vec<SelectorStratum>,
}

#[derive(Clone, Debug)]
struct SelectorStratum {
    stratum_id: String,
    minimum_selected_actions: usize,
}

#[derive(Clone, Debug)]
struct SelectorAction {
    action_id: String,
    stratum_id: String,
    modality: String,
    cost_units: u32,
    diagnostic_action: bool,
    outcomes: Vec<String>,
    likelihoods: Vec<Vec<f64>>,
}

#[derive(Clone, Debug)]
struct SelectorDecision {
    decision_id: String,
    model_utilities: Vec<f64>,
}

#[derive(Clone, Copy, Debug)]
enum Policy {
    StaticEig,
    ConditionalEig,
    DecisionEvsi,
    DiscrepancyGuardedEvsi,
    SeededRandom,
}

impl Policy {
    const ALL: [Policy; 5] = [
        Policy::StaticEig,
        Policy::ConditionalEig,
        Policy::DecisionEvsi,
        Policy::DiscrepancyGuardedEvsi,
        Policy::SeededRandom,
    ];

    fn id(self) -> &'static str {
        match self {
            Policy::StaticEig => "static_eig",
            Policy::ConditionalEig => "conditional_eig",
            Policy::DecisionEvsi => "decision_focused_evsi",
            Policy::DiscrepancyGuardedEvsi => "discrepancy_guarded_evsi",
            Policy::SeededRandom => "seeded_random",
        }
    }
}

#[derive(Clone, Debug)]
struct TrajectoryMetrics {
    truth_utility: f64,
    decision_regret: f64,
    surprise_events: usize,
    diagnostic_actions: usize,
    total_cost_units: u32,
    final_posterior_entropy: f64,
    predictive_log_score: f64,
    predictive_brier_score: f64,
    selected_action_ids: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
struct PolicySummary {
    policy_id: String,
    replicate_count: usize,
    mean_truth_decision_utility: f64,
    mean_true_decision_regret: f64,
    mean_surprise_events: f64,
    mean_diagnostic_actions: f64,
    mean_total_cost_units: f64,
    mean_final_posterior_entropy: f64,
    mean_predictive_log_score: f64,
    mean_predictive_brier_score: f64,
    selected_action_frequency: Vec<ActionFrequency>,
}

#[derive(Clone, Debug, Serialize)]
struct ActionFrequency {
    action_id: String,
    selection_rate: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    report_version: &'static str,
    status: &'static str,
    scenario_id: String,
    input_sha256: String,
    replicate_count: usize,
    candidate_count: usize,
    eligible_candidate_count: usize,
    ineligible_candidate_count: usize,
    budget_cost_units: u32,
    predictive_surprise_threshold: f64,
    policy_summaries: Vec<PolicySummary>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn in_unit_interval(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn probabilities_valid(values: &[f64]) -> bool {
    !values.is_empty()
        && values.iter().all(|value| in_unit_interval(*value))
        && (values.iter().sum::<f64>() - 1.0).abs() <= 1e-8
}

fn validate_scenario(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != VERSION {
        return Err(format!("unsupported schema_version {:?}", scenario.schema_version));
    }
    if scenario.data_class != "synthetic_known_truth" {
        return Err("this engine accepts synthetic_known_truth scenarios only".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must not be blank".into());
    }
    if !(MIN_REPLICATES..=MAX_REPLICATES).contains(&scenario.replicates) {
        return Err(format!("replicates must be {MIN_REPLICATES}..={MAX_REPLICATES}"));
    }
    if scenario.budget_cost_units == 0 {
        return Err("budget_cost_units must be positive".into());
    }
    if !in_unit_interval(scenario.predictive_surprise_threshold)
        || scenario.predictive_surprise_threshold <= 0.0
    {
        return Err("predictive_surprise_threshold must be in (0,1]".into());
    }
    if scenario.actions.is_empty() || scenario.actions.len() > MAX_ACTIONS {
        return Err(format!("action count must be 1..={MAX_ACTIONS}"));
    }
    if scenario.strata.is_empty() || scenario.models.len() < 2 || scenario.decisions.len() < 2 {
        return Err("need target strata, at least two candidate models and two decisions".into());
    }

    let mut strata = BTreeSet::new();
    for stratum in &scenario.strata {
        if stratum.stratum_id.trim().is_empty()
            || !strata.insert(stratum.stratum_id.as_str())
        {
            return Err("stratum IDs must be non-empty and unique".into());
        }
    }

    let mut model_ids = BTreeSet::new();
    for model in &scenario.models {
        if model.model_id.trim().is_empty()
            || !model_ids.insert(model.model_id.as_str())
            || !in_unit_interval(model.prior_weight)
        {
            return Err("model IDs must be unique and prior weights in [0,1]".into());
        }
    }
    if (scenario.models.iter().map(|model| model.prior_weight).sum::<f64>() - 1.0).abs() > 1e-8 {
        return Err("model prior weights must sum to 1.0".into());
    }

    let mut decision_ids = BTreeSet::new();
    for decision in &scenario.decisions {
        if decision.decision_id.trim().is_empty()
            || !decision_ids.insert(decision.decision_id.as_str())
            || decision.model_utilities.len() != scenario.models.len()
            || !decision.model_utilities.iter().all(|value| in_unit_interval(*value))
        {
            return Err(format!("invalid decision {:?}", decision.decision_id));
        }
    }
    let mut truth_decision_ids = BTreeSet::new();
    for decision in &scenario.truth_decisions {
        if decision.decision_id.trim().is_empty()
            || !truth_decision_ids.insert(decision.decision_id.as_str())
            || !decision_ids.contains(decision.decision_id.as_str())
            || !in_unit_interval(decision.utility)
        {
            return Err("invalid or unknown evaluator truth decision".into());
        }
    }
    if truth_decision_ids.len() != decision_ids.len() {
        return Err("every candidate decision needs one evaluator truth utility".into());
    }

    let mut action_ids = BTreeSet::new();
    for action in &scenario.actions {
        if action.action_id.trim().is_empty()
            || !action_ids.insert(action.action_id.as_str())
            || !strata.contains(action.stratum_id.as_str())
        {
            return Err(format!("invalid action ID or stratum on {:?}", action.action_id));
        }
        if action.cost_units == 0 || action.outcomes.len() < 2 {
            return Err(format!("action {:?} needs positive cost and >=2 outcomes", action.action_id));
        }
        if action.eligible && action.exclusion_reason.is_some() {
            return Err(format!("eligible action {:?} has exclusion_reason", action.action_id));
        }
        if !action.eligible
            && action.exclusion_reason.as_deref().unwrap_or("").trim().is_empty()
        {
            return Err(format!("ineligible action {:?} needs exclusion_reason", action.action_id));
        }
        let mut outcome_ids = BTreeSet::new();
        if action.outcomes.iter().any(|outcome| outcome.trim().is_empty()
            || !outcome_ids.insert(outcome.as_str()))
        {
            return Err(format!("action {:?} has blank or duplicate outcomes", action.action_id));
        }
        if action.truth_outcome_probabilities.len() != action.outcomes.len()
            || !probabilities_valid(&action.truth_outcome_probabilities)
        {
            return Err(format!("invalid evaluator truth likelihood on {:?}", action.action_id));
        }
        if action.likelihoods.len() != scenario.models.len() {
            return Err(format!("action {:?} must define one likelihood per model", action.action_id));
        }
        let mut likelihood_model_ids = BTreeSet::new();
        for likelihood in &action.likelihoods {
            if !model_ids.contains(likelihood.model_id.as_str())
                || !likelihood_model_ids.insert(likelihood.model_id.as_str())
                || likelihood.outcome_probabilities.len() != action.outcomes.len()
                || !probabilities_valid(&likelihood.outcome_probabilities)
            {
                return Err(format!("invalid candidate likelihood on {:?}", action.action_id));
            }
        }
        if likelihood_model_ids.len() != model_ids.len() {
            return Err(format!("action {:?} omits a candidate model", action.action_id));
        }
    }

    for stratum in &scenario.strata {
        let eligible_count = scenario.actions.iter().filter(|action| {
            action.eligible && action.stratum_id == stratum.stratum_id
        }).count();
        if eligible_count < stratum.minimum_selected_actions {
            return Err(format!("stratum {:?} has too few eligible actions", stratum.stratum_id));
        }
    }
    let minimum_cost = scenario.strata.iter().try_fold(0u32, |sum, stratum| {
        let mut costs: Vec<u32> = scenario.actions.iter()
            .filter(|action| action.eligible && action.stratum_id == stratum.stratum_id)
            .map(|action| action.cost_units)
            .collect();
        costs.sort_unstable();
        let required_cost = costs.into_iter().take(stratum.minimum_selected_actions)
            .try_fold(0u32, |inner, cost| inner.checked_add(cost))?;
        sum.checked_add(required_cost)
    }).ok_or_else(|| "minimum coverage cost overflow".to_owned())?;
    if minimum_cost > scenario.budget_cost_units {
        return Err("minimum target coverage is infeasible under budget".into());
    }
    Ok(())
}

fn selector_context(scenario: &Scenario) -> SelectorContext {
    SelectorContext {
        budget_cost_units: scenario.budget_cost_units,
        predictive_surprise_threshold: scenario.predictive_surprise_threshold,
        strata: scenario.strata.iter().map(|stratum| SelectorStratum {
            stratum_id: stratum.stratum_id.clone(),
            minimum_selected_actions: stratum.minimum_selected_actions,
        }).collect(),
    }
}

fn selector_actions(scenario: &Scenario) -> Vec<SelectorAction> {
    let model_index: std::collections::BTreeMap<&str, usize> = scenario.models.iter()
        .enumerate().map(|(index, model)| (model.model_id.as_str(), index)).collect();
    scenario.actions.iter().filter(|action| action.eligible).map(|action| {
        let mut likelihoods = vec![Vec::new(); scenario.models.len()];
        for likelihood in &action.likelihoods {
            let index = *model_index.get(likelihood.model_id.as_str()).expect("validated model");
            likelihoods[index] = likelihood.outcome_probabilities.clone();
        }
        SelectorAction {
            action_id: action.action_id.clone(),
            stratum_id: action.stratum_id.clone(),
            modality: action.modality.clone(),
            cost_units: action.cost_units,
            diagnostic_action: action.diagnostic_action,
            outcomes: action.outcomes.clone(),
            likelihoods,
        }
    }).collect()
}

fn selector_decisions(scenario: &Scenario) -> Vec<SelectorDecision> {
    scenario.decisions.iter().map(|decision| SelectorDecision {
        decision_id: decision.decision_id.clone(),
        model_utilities: decision.model_utilities.clone(),
    }).collect()
}

fn entropy(posterior: &[f64]) -> f64 {
    posterior.iter().filter(|weight| **weight > 0.0)
        .map(|weight| -weight * weight.ln()).sum()
}

fn predictive_probabilities(action: &SelectorAction, posterior: &[f64]) -> Vec<f64> {
    (0..action.outcomes.len()).map(|outcome_index| {
        posterior.iter().enumerate()
            .map(|(model_index, weight)| {
                weight * action.likelihoods[model_index][outcome_index]
            }).sum()
    }).collect()
}

fn posterior_after(
    action: &SelectorAction,
    posterior: &[f64],
    observed_index: usize,
) -> (Vec<f64>, f64) {
    let predictive = predictive_probabilities(action, posterior)[observed_index];
    if predictive <= ALERT_EPSILON {
        return (posterior.to_vec(), predictive);
    }
    let unnormalized: Vec<f64> = posterior.iter().enumerate()
        .map(|(model_index, weight)| {
            weight * action.likelihoods[model_index][observed_index]
        }).collect();
    let total = unnormalized.iter().sum::<f64>();
    if total <= ALERT_EPSILON {
        return (posterior.to_vec(), predictive);
    }
    (unnormalized.iter().map(|weight| weight / total).collect(), predictive)
}

fn best_decision_utility(posterior: &[f64], decisions: &[SelectorDecision]) -> f64 {
    decisions.iter().map(|decision| {
        posterior.iter().zip(&decision.model_utilities)
            .map(|(weight, utility)| weight * utility).sum::<f64>()
    }).fold(f64::NEG_INFINITY, f64::max)
}

fn eig(action: &SelectorAction, posterior: &[f64]) -> f64 {
    let prior_entropy = entropy(posterior);
    let predictive = predictive_probabilities(action, posterior);
    let expected_posterior_entropy = predictive.iter().enumerate().map(|(index, probability)| {
        if *probability <= ALERT_EPSILON {
            return 0.0;
        }
        let (updated, _) = posterior_after(action, posterior, index);
        probability * entropy(&updated)
    }).sum::<f64>();
    (prior_entropy - expected_posterior_entropy).max(0.0)
}

fn evsi(action: &SelectorAction, posterior: &[f64], decisions: &[SelectorDecision]) -> f64 {
    let current_value = best_decision_utility(posterior, decisions);
    let predictive = predictive_probabilities(action, posterior);
    let expected_after = predictive.iter().enumerate().map(|(index, probability)| {
        if *probability <= ALERT_EPSILON {
            return 0.0;
        }
        let (updated, _) = posterior_after(action, posterior, index);
        probability * best_decision_utility(&updated, decisions)
    }).sum::<f64>();
    (expected_after - current_value).max(0.0)
}

fn deterministic_u64(seed: u64, replicate: usize, step: usize, action_id: &str) -> u64 {
    let mut hasher = Sha256::new();
    hasher.update(seed.to_le_bytes());
    hasher.update((replicate as u64).to_le_bytes());
    hasher.update((step as u64).to_le_bytes());
    hasher.update(action_id.as_bytes());
    let digest = hasher.finalize();
    u64::from_le_bytes(digest[0..8].try_into().expect("eight digest bytes"))
}

fn deterministic_uniform(seed: u64, replicate: usize, step: usize, action_id: &str) -> f64 {
    let value = deterministic_u64(seed, replicate, step, action_id);
    (value as f64) / (u64::MAX as f64)
}

fn draw_outcome(
    action: &Action,
    seed: u64,
    replicate: usize,
    step: usize,
) -> usize {
    let draw = deterministic_uniform(seed, replicate, step, &action.action_id);
    let mut cumulative = 0.0;
    for (index, probability) in action.truth_outcome_probabilities.iter().enumerate() {
        cumulative += probability;
        if draw < cumulative || index + 1 == action.truth_outcome_probabilities.len() {
            return index;
        }
    }
    action.truth_outcome_probabilities.len() - 1
}

fn selected_stratum_count(
    actions: &[SelectorAction],
    selected_ids: &[String],
    stratum_id: &str,
) -> usize {
    selected_ids.iter().filter(|selected_id| actions.iter().any(|action| {
        action.action_id.as_str() == selected_id.as_str() && action.stratum_id == stratum_id
    })).count()
}

fn next_action<'a>(
    context: &SelectorContext,
    actions: &'a [SelectorAction],
    decisions: &[SelectorDecision],
    selected_ids: &[String],
    spent: u32,
    posterior: &[f64],
    initial_posterior: &[f64],
    policy: Policy,
    discrepancy_active: bool,
    seed: u64,
    replicate: usize,
    step: usize,
) -> Option<&'a SelectorAction> {
    let remaining = context.budget_cost_units.saturating_sub(spent);
    let candidates: Vec<&SelectorAction> = actions.iter().filter(|action| {
        !selected_ids.contains(&action.action_id) && action.cost_units <= remaining
    }).collect();
    if candidates.is_empty() {
        return None;
    }

    let missing_stratum = context.strata.iter().find(|stratum| {
        selected_stratum_count(actions, selected_ids, &stratum.stratum_id)
            < stratum.minimum_selected_actions
    });
    let mut eligible: Vec<&SelectorAction> = if let Some(stratum) = missing_stratum {
        candidates.into_iter().filter(|action| action.stratum_id == stratum.stratum_id).collect()
    } else if matches!(policy, Policy::DiscrepancyGuardedEvsi) && discrepancy_active {
        let diagnostics: Vec<&SelectorAction> = candidates.iter().copied()
            .filter(|action| action.diagnostic_action).collect();
        if diagnostics.is_empty() {
            candidates
        } else {
            diagnostics
        }
    } else {
        candidates
    };
    if eligible.is_empty() {
        return None;
    }

    eligible.sort_by(|a, b| {
        let score = |action: &SelectorAction| -> f64 {
            if matches!(policy, Policy::SeededRandom) {
                return deterministic_u64(seed, replicate, step, &action.action_id) as f64;
            }
            let acquisition = match policy {
                Policy::StaticEig => eig(action, initial_posterior),
                Policy::ConditionalEig => eig(action, posterior),
                Policy::DecisionEvsi => evsi(action, posterior, decisions),
                Policy::DiscrepancyGuardedEvsi => {
                    evsi(action, posterior, decisions) + 0.05 * eig(action, posterior)
                }
                Policy::SeededRandom => 0.0,
            };
            acquisition / f64::from(action.cost_units)
        };
        score(b).total_cmp(&score(a))
            .then_with(|| a.cost_units.cmp(&b.cost_units))
            .then_with(|| a.action_id.cmp(&b.action_id))
    });
    eligible.first().copied()
}

fn summarize_trajectory(
    scenario: &Scenario,
    policy: Policy,
    replicate: usize,
    context: &SelectorContext,
    actions: &[SelectorAction],
    decisions: &[SelectorDecision],
) -> TrajectoryMetrics {
    let initial_posterior: Vec<f64> = scenario.models.iter()
        .map(|model| model.prior_weight).collect();
    let mut posterior = initial_posterior.clone();
    let mut selected_ids = Vec::new();
    let mut spent = 0u32;
    let mut surprise_events = 0usize;
    let mut diagnostic_actions = 0usize;
    let mut log_score_sum = 0.0;
    let mut brier_score_sum = 0.0;
    let mut observation_count = 0usize;
    let mut discrepancy_active = false;

    loop {
        let step = selected_ids.len();
        let selected = next_action(
            context, actions, decisions, &selected_ids, spent, &posterior,
            &initial_posterior, policy, discrepancy_active, scenario.seed,
            replicate, step,
        );
        let Some(selected) = selected else { break; };
        let Some(spent_next) = spent.checked_add(selected.cost_units) else { break; };
        if spent_next > context.budget_cost_units {
            break;
        }

        let evaluator_action = scenario.actions.iter()
            .find(|action| action.action_id == selected.action_id)
            .expect("selector action has evaluator action");
        let observed_index = draw_outcome(evaluator_action, scenario.seed, replicate, step);
        let predictive = predictive_probabilities(selected, &posterior);
        let observed_probability = predictive[observed_index];
        log_score_sum += -observed_probability.max(ALERT_EPSILON).ln();
        brier_score_sum += predictive.iter().enumerate().map(|(index, probability)| {
            let observed = if index == observed_index { 1.0 } else { 0.0 };
            (probability - observed).powi(2)
        }).sum::<f64>();
        observation_count += 1;

        if observed_probability < context.predictive_surprise_threshold {
            surprise_events += 1;
            discrepancy_active = true;
        }
        let (updated, _) = posterior_after(selected, &posterior, observed_index);
        posterior = updated;

        if selected.diagnostic_action {
            diagnostic_actions += 1;
        }
        spent = spent_next;
        selected_ids.push(selected.action_id.clone());
    }

    let chosen_decision = decisions.iter().max_by(|a, b| {
        let utility = |decision: &SelectorDecision| posterior.iter()
            .zip(&decision.model_utilities)
            .map(|(weight, value)| weight * value).sum::<f64>();
        utility(a).total_cmp(&utility(b))
    }).expect("validated decisions");

    let truth_value = scenario.truth_decisions.iter()
        .find(|decision| decision.decision_id == chosen_decision.decision_id)
        .expect("truth utility exists").utility;
    let best_truth_value = scenario.truth_decisions.iter()
        .map(|decision| decision.utility).fold(f64::NEG_INFINITY, f64::max);

    TrajectoryMetrics {
        truth_utility: truth_value,
        decision_regret: (best_truth_value - truth_value).max(0.0),
        surprise_events,
        diagnostic_actions,
        total_cost_units: spent,
        final_posterior_entropy: entropy(&posterior),
        predictive_log_score: if observation_count == 0 { 0.0 } else {
            log_score_sum / observation_count as f64
        },
        predictive_brier_score: if observation_count == 0 { 0.0 } else {
            brier_score_sum / observation_count as f64
        },
        selected_action_ids: selected_ids,
    }
}

fn summarize_policy(
    scenario: &Scenario,
    context: &SelectorContext,
    actions: &[SelectorAction],
    decisions: &[SelectorDecision],
    policy: Policy,
) -> PolicySummary {
    let mut trajectories = Vec::with_capacity(scenario.replicates);
    for replicate in 0..scenario.replicates {
        trajectories.push(summarize_trajectory(
            scenario, policy, replicate, context, actions, decisions,
        ));
    }
    let n = trajectories.len() as f64;
    let mean = |f: fn(&TrajectoryMetrics) -> f64| {
        trajectories.iter().map(f).sum::<f64>() / n
    };
    let selected_action_frequency = actions.iter().map(|action| ActionFrequency {
        action_id: action.action_id.clone(),
        selection_rate: trajectories.iter()
            .filter(|trajectory| trajectory.selected_action_ids.contains(&action.action_id))
            .count() as f64 / n,
    }).collect();

    PolicySummary {
        policy_id: policy.id().to_owned(),
        replicate_count: trajectories.len(),
        mean_truth_decision_utility: mean(|m| m.truth_utility),
        mean_true_decision_regret: mean(|m| m.decision_regret),
        mean_surprise_events: mean(|m| m.surprise_events as f64),
        mean_diagnostic_actions: mean(|m| m.diagnostic_actions as f64),
        mean_total_cost_units: mean(|m| m.total_cost_units as f64),
        mean_final_posterior_entropy: mean(|m| m.final_posterior_entropy),
        mean_predictive_log_score: mean(|m| m.predictive_log_score),
        mean_predictive_brier_score: mean(|m| m.predictive_brier_score),
        selected_action_frequency,
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn build_report(scenario: &Scenario, input: &[u8]) -> Result<Report, String> {
    validate_scenario(scenario)?;
    let context = selector_context(scenario);
    let actions = selector_actions(scenario);
    let decisions = selector_decisions(scenario);
    let policy_summaries = Policy::ALL.iter().map(|policy| {
        summarize_policy(scenario, &context, &actions, &decisions, *policy)
    }).collect();

    Ok(Report {
        report_version: VERSION,
        status: "synthetic_sequential_bayesian_benchmark_only",
        scenario_id: scenario.scenario_id.clone(),
        input_sha256: sha256_hex(input),
        replicate_count: scenario.replicates,
        candidate_count: scenario.actions.len(),
        eligible_candidate_count: scenario.actions.iter().filter(|action| action.eligible).count(),
        ineligible_candidate_count: scenario.actions.iter().filter(|action| !action.eligible).count(),
        budget_cost_units: scenario.budget_cost_units,
        predictive_surprise_threshold: scenario.predictive_surprise_threshold,
        policy_summaries,
        interpretation_boundary: "Synthetic categorical Bayesian benchmark only; no empirical soil evidence, physical simulator validation, field authorization, efficacy claim, or SOTA claim.",
        warnings: vec![
            "Candidate likelihoods are finite categorical distributions. The engine does not yet support continuous, censored, heteroscedastic or correlated-noise observations.".into(),
            "The synthetic true observation generator may be outside the candidate model family. Surprise events are diagnostic flags, not proof of a specific missing physical mechanism.".into(),
            "The discrepancy-guarded policy can select a predeclared diagnostic modality after a surprise; it does not yet update an explicit discrepancy posterior.".into(),
            "A finite synthetic scenario and fixed seed ensemble cannot establish external predictive calibration or field utility.".into(),
            "The engine's decision regret is measured against the evaluator's known best decision in the synthetic world, not a real management outcome.".into(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err("usage: sequential-bayes-design <scenario.json> <report.json>".into());
    }
    let input = fs::read(&args[1])?;
    let scenario: Scenario = serde_json::from_slice(&input)?;
    let report = build_report(&scenario, &input)
        .map_err(|error| format!("scenario rejected: {error}"))?;
    fs::write(&args[2], serde_json::to_vec_pretty(&report)?)?;
    println!(
        "wrote sequential Bayesian benchmark {} with {} replicates and {} policies",
        report.scenario_id, report.replicate_count, report.policy_summaries.len()
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
        serde_json::from_str(include_str!("../fixtures/synthetic-sequential-scenario.json"))
            .expect("valid fixture")
    }

    fn selector_fixture() -> (SelectorContext, Vec<SelectorAction>, Vec<SelectorDecision>, Vec<f64>) {
        let scenario = fixture();
        (
            selector_context(&scenario),
            selector_actions(&scenario),
            selector_decisions(&scenario),
            scenario.models.iter().map(|m| m.prior_weight).collect(),
        )
    }

    #[test]
    fn posterior_update_normalizes_and_changes_belief() {
        let (_, actions, _, prior) = selector_fixture();
        let action = actions.iter().find(|a| a.action_id == "B_SANDY_EVENT_TRACER").unwrap();
        let (posterior, probability) = posterior_after(action, &prior, 0);
        assert!(probability > 0.0);
        assert!((posterior.iter().sum::<f64>() - 1.0).abs() < 1e-10);
        assert_ne!(posterior, prior);
    }

    #[test]
    fn conditional_eig_and_evsi_are_nonnegative() {
        let (_, actions, decisions, prior) = selector_fixture();
        for action in &actions {
            assert!(eig(action, &prior) >= 0.0);
            assert!(evsi(action, &prior, &decisions) >= 0.0);
        }
    }

    #[test]
    fn surprise_observation_is_detected() {
        let (_, actions, _, prior) = selector_fixture();
        let action = actions.iter().find(|a| a.action_id == "B_SANDY_EVENT_TRACER").unwrap();
        let (posterior, probability) = posterior_after(action, &prior, 2);
        assert!(probability < 0.05);
        assert_eq!(posterior, prior);
    }

    #[test]
    fn truth_mutation_does_not_change_selector_inputs_or_first_choice() {
        let original = fixture();
        let context = selector_context(&original);
        let actions = selector_actions(&original);
        let decisions = selector_decisions(&original);
        let prior: Vec<f64> = original.models.iter().map(|m| m.prior_weight).collect();
        let before = next_action(
            &context, &actions, &decisions, &[], 0, &prior, &prior,
            Policy::DecisionEvsi, false, original.seed, 0, 0,
        ).unwrap().action_id.clone();

        let mut mutated = original.clone();
        for action in &mut mutated.actions {
            action.truth_outcome_probabilities.reverse();
        }
        for decision in &mut mutated.truth_decisions {
            decision.utility = 1.0 - decision.utility;
        }
        let after_actions = selector_actions(&mutated);
        let after_decisions = selector_decisions(&mutated);
        let after_prior: Vec<f64> = mutated.models.iter().map(|m| m.prior_weight).collect();
        let after = next_action(
            &selector_context(&mutated), &after_actions, &after_decisions,
            &[], 0, &after_prior, &after_prior, Policy::DecisionEvsi,
            false, mutated.seed, 0, 0,
        ).unwrap().action_id.clone();
        assert_eq!(before, after);
    }

    #[test]
    fn discrepancy_guard_selects_diagnostic_modality_after_surprise() {
        let (context, actions, decisions, prior) = selector_fixture();
        let tracer = actions.iter().find(|a| a.action_id == "B_SANDY_EVENT_TRACER").unwrap();
        let (updated, probability) = posterior_after(tracer, &prior, 2);
        assert!(probability < context.predictive_surprise_threshold);
        let chosen = next_action(
            &context, &actions, &decisions,
            &["B_SANDY_EVENT_TRACER".into(), "D_WEATHERED_RETENTION".into()],
            2, &updated, &prior, Policy::DiscrepancyGuardedEvsi,
            true, 7, 0, 1,
        ).unwrap();
        assert!(chosen.diagnostic_action);
    }

    #[test]
    fn all_policies_respect_budget_and_hard_coverage() {
        let scenario = fixture();
        let report = build_report(&scenario, b"fixture").unwrap();
        for summary in report.policy_summaries {
            assert!(summary.mean_total_cost_units <= scenario.budget_cost_units as f64);
            assert_eq!(summary.replicate_count, scenario.replicates);
        }
    }

    #[test]
    fn seeded_simulations_are_reproducible() {
        let scenario = fixture();
        let first = build_report(&scenario, b"fixture").unwrap();
        let second = build_report(&scenario, b"fixture").unwrap();
        let a = serde_json::to_vec(&first).unwrap();
        let b = serde_json::to_vec(&second).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn reject_empirical_data_class() {
        let mut scenario = fixture();
        scenario.data_class = "measured_soil_data".into();
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn reject_bad_probability_sums_and_unknown_models() {
        let mut scenario = fixture();
        scenario.actions[0].truth_outcome_probabilities[0] = 0.2;
        assert!(validate_scenario(&scenario).is_err());
        let mut scenario = fixture();
        scenario.actions[0].likelihoods[0].model_id = "NOT_A_MODEL".into();
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn reject_infeasible_target_coverage() {
        let mut scenario = fixture();
        scenario.budget_cost_units = 1;
        assert!(validate_scenario(&scenario).is_err());
    }

    #[test]
    fn predictive_brier_and_log_score_remain_finite() {
        let scenario = fixture();
        let context = selector_context(&scenario);
        let actions = selector_actions(&scenario);
        let decisions = selector_decisions(&scenario);
        for policy in Policy::ALL {
            let result = summarize_trajectory(&scenario, policy, 0, &context, &actions, &decisions);
            assert!(result.predictive_log_score.is_finite());
            assert!(result.predictive_brier_score.is_finite());
            assert!(result.decision_regret >= 0.0);
            assert!(result.total_cost_units <= scenario.budget_cost_units);
        }
    }

    #[test]
    fn ineligible_action_never_enters_selector_action_view() {
        let scenario = fixture();
        let actions = selector_actions(&scenario);
        assert!(actions.iter().all(|a| a.action_id != "X_INELIGIBLE"));
    }
}

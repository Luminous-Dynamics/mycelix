use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use terra_preta_sequential_bayes_design::robust_information::{
    robust_information_at_kl_radius, select_discrete_design,
    select_discrete_design_at_kl_radius, DesignInformationObjective,
    DiscreteDesignCandidate, DiscreteDesignModel, DiscreteDesignSelection,
    RadiusRobustInformation,
};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    model_ids: Vec<String>,
    prior_weights: Vec<f64>,
    candidates: Vec<CandidateInput>,
    budget_units: u64,
    sibson_alpha: f64,
    kl_radius_nats: f64,
    minimum_alpha: f64,
    alpha_bisection_iterations: usize,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CandidateInput {
    action_id: String,
    cost_units: u64,
    eligible: bool,
    outcome_ids: Vec<String>,
    likelihoods: Vec<Vec<f64>>,
}

impl CandidateInput {
    fn as_library_candidate(&self) -> DiscreteDesignCandidate {
        DiscreteDesignCandidate {
            action_id: self.action_id.clone(),
            cost_units: self.cost_units,
            eligible: self.eligible,
            outcome_ids: self.outcome_ids.clone(),
            likelihoods: self.likelihoods.clone(),
        }
    }

    fn as_model(&self, model_ids: &[String], prior_weights: &[f64]) -> DiscreteDesignModel {
        DiscreteDesignModel {
            model_ids: model_ids.to_vec(),
            prior_weights: prior_weights.to_vec(),
            outcome_ids: self.outcome_ids.clone(),
            likelihoods: self.likelihoods.clone(),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct SelectionRecord {
    action_id: String,
    cost_units: u64,
    objective_value_nats: f64,
    alpha_used: f64,
}

impl SelectionRecord {
    fn from_selection(selection: &DiscreteDesignSelection, alpha_used: f64) -> Self {
        Self {
            action_id: selection.action_id.clone(),
            cost_units: selection.cost_units,
            objective_value_nats: selection.objective_value_nats,
            alpha_used,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct CandidateSummary {
    action_id: String,
    eligible: bool,
    cost_units: u64,
    feasible_under_budget: bool,
    shannon_information_nats: f64,
    sibson_information_nats: f64,
    per_design_alpha: f64,
    per_design_sibson_information_nats: f64,
    effective_kl_radius_nats: f64,
    alpha_search_clamped: bool,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    scenario_sha256: String,
    status: String,
    candidate_count: usize,
    feasible_candidate_count: usize,
    budget_units: u64,
    sibson_alpha: f64,
    requested_kl_radius_nats: f64,
    minimum_alpha: f64,
    alpha_bisection_iterations: usize,
    nominal_shannon_selection: Option<SelectionRecord>,
    fixed_alpha_sibson_selection: Option<SelectionRecord>,
    fixed_kl_radius_selection: Option<SelectionRecord>,
    candidate_summaries: Vec<CandidateSummary>,
    limitations: Vec<String>,
}

fn validate_scenario(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != "0.1.0" {
        return Err(format!("unsupported schema version {:?}", scenario.schema_version));
    }
    if scenario.data_class != "synthetic-known-truth" {
        return Err("only synthetic-known-truth scenarios are accepted".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must be non-empty".into());
    }
    if !scenario.sibson_alpha.is_finite() || !(0.0..1.0).contains(&scenario.sibson_alpha) {
        return Err("sibson_alpha must lie strictly between zero and one".into());
    }
    if !scenario.kl_radius_nats.is_finite() || scenario.kl_radius_nats < 0.0 {
        return Err("kl_radius_nats must be finite and non-negative".into());
    }
    if !scenario.minimum_alpha.is_finite()
        || !(0.0..1.0 - 1e-8).contains(&scenario.minimum_alpha)
    {
        return Err("minimum_alpha must lie in (0, 1 - 1e-8)".into());
    }
    if scenario.alpha_bisection_iterations == 0 {
        return Err("alpha_bisection_iterations must be positive".into());
    }
    if scenario.candidates.is_empty() {
        return Err("at least one candidate action is required".into());
    }

    let mut action_ids = std::collections::BTreeSet::new();
    for candidate in &scenario.candidates {
        if candidate.action_id.trim().is_empty() || !action_ids.insert(&candidate.action_id) {
            return Err("candidate action IDs must be non-empty and unique".into());
        }
        candidate
            .as_model(&scenario.model_ids, &scenario.prior_weights)
            .validate()?;
    }
    Ok(())
}

fn summarize_candidate(
    candidate: &CandidateInput,
    scenario: &Scenario,
) -> Result<CandidateSummary, String> {
    let design = candidate.as_model(&scenario.model_ids, &scenario.prior_weights);
    let radius_score: RadiusRobustInformation = robust_information_at_kl_radius(
        &design,
        scenario.kl_radius_nats,
        scenario.minimum_alpha,
        scenario.alpha_bisection_iterations,
    )?;
    Ok(CandidateSummary {
        action_id: candidate.action_id.clone(),
        eligible: candidate.eligible,
        cost_units: candidate.cost_units,
        feasible_under_budget: candidate.eligible && candidate.cost_units <= scenario.budget_units,
        shannon_information_nats: design.shannon_mutual_information()?,
        sibson_information_nats: design.sibson_alpha_mutual_information(scenario.sibson_alpha)?,
        per_design_alpha: radius_score.alpha,
        per_design_sibson_information_nats: radius_score.sibson_information_nats,
        effective_kl_radius_nats: radius_score.effective_kl_radius_nats,
        alpha_search_clamped: radius_score.alpha_search_clamped,
    })
}

fn selection_record(
    selection: Option<DiscreteDesignSelection>,
    alpha: f64,
) -> Option<SelectionRecord> {
    selection.map(|value| SelectionRecord::from_selection(&value, alpha))
}

fn build_report(scenario: &Scenario, input_bytes: &[u8]) -> Result<Report, String> {
    validate_scenario(scenario)?;
    let model_ids = &scenario.model_ids;
    let prior_weights = &scenario.prior_weights;
    let candidates: Vec<DiscreteDesignCandidate> = scenario.candidates
        .iter()
        .map(CandidateInput::as_library_candidate)
        .collect();

    let nominal = select_discrete_design(
        model_ids,
        prior_weights,
        &candidates,
        scenario.budget_units,
        DesignInformationObjective::Shannon,
    )?;
    let fixed_alpha = select_discrete_design(
        model_ids,
        prior_weights,
        &candidates,
        scenario.budget_units,
        DesignInformationObjective::SibsonAlpha(scenario.sibson_alpha),
    )?;
    let fixed_radius = select_discrete_design_at_kl_radius(
        model_ids,
        prior_weights,
        &candidates,
        scenario.budget_units,
        scenario.kl_radius_nats,
        scenario.minimum_alpha,
        scenario.alpha_bisection_iterations,
    )?;

    let candidate_summaries = scenario.candidates.iter()
        .map(|candidate| summarize_candidate(candidate, scenario))
        .collect::<Result<Vec<_>, _>>()?;
    let feasible_candidate_count = candidate_summaries.iter()
        .filter(|candidate| candidate.feasible_under_budget)
        .count();

    Ok(Report {
        schema_version: "0.1.0".into(),
        data_class: "synthetic-known-truth".into(),
        scenario_id: scenario.scenario_id.clone(),
        scenario_sha256: format!("{:x}", Sha256::digest(input_bytes)),
        status: "synthetic_one_step_robust_information_benchmark_only".into(),
        candidate_count: candidate_summaries.len(),
        feasible_candidate_count,
        budget_units: scenario.budget_units,
        sibson_alpha: scenario.sibson_alpha,
        requested_kl_radius_nats: scenario.kl_radius_nats,
        minimum_alpha: scenario.minimum_alpha,
        alpha_bisection_iterations: scenario.alpha_bisection_iterations,
        nominal_shannon_selection: selection_record(nominal, 1.0),
        fixed_alpha_sibson_selection: selection_record(fixed_alpha, scenario.sibson_alpha),
        fixed_kl_radius_selection: fixed_radius.map(|(selection, score)| {
            SelectionRecord::from_selection(&selection, score.alpha)
        }),
        candidate_summaries,
        limitations: vec![
            "Finite discrete candidate-model reference; not a continuous physical likelihood.".into(),
            "One-step deterministic design; not a sequential stochastic-policy optimizer.".into(),
            "The fixed per-design KL radius is not a shared average-KL policy constraint.".into(),
            "No PAC-Bayes lower bound, nested Monte Carlo optimizer or empirical covariance estimation is implemented.".into(),
            "The model-information objective is not decision EVSI.".into(),
            "Synthetic success does not establish soil, field or agronomic validity.".into(),
        ],
    })
}

fn run() -> Result<(), String> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        return Err(format!(
            "usage: {} <scenario.json> <report.json>",
            arguments.first().map(String::as_str).unwrap_or("robust-information-bakeoff")
        ));
    }
    let input_bytes = fs::read(&arguments[1])
        .map_err(|error| format!("failed to read scenario: {error}"))?;
    let scenario: Scenario = serde_json::from_slice(&input_bytes)
        .map_err(|error| format!("failed to parse scenario: {error}"))?;
    let report = build_report(&scenario, &input_bytes)?;
    let report_bytes = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("failed to serialize report: {error}"))?;
    fs::write(&arguments[2], report_bytes)
        .map_err(|error| format!("failed to write report: {error}"))?;
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

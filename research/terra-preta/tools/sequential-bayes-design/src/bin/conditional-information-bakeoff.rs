use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, fs};
use terra_preta_sequential_bayes_design::information_gain::estimate_action_information_gain;
use terra_preta_sequential_bayes_design::multivariate_gaussian::{
    MultivariateGaussian, MultivariateGaussianModel,
};
use terra_preta_sequential_bayes_design::observation_history::{
    ObservationAction, ObservationCoordinate, ObservationHistory, ObservationLayout,
};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    models: Vec<ModelInput>,
    coordinates: Vec<CoordinateInput>,
    prior_actions: Vec<ActionInput>,
    history_action_id: String,
    history_values: Vec<f64>,
    conditional_actions: Vec<ActionInput>,
    budget_units: u64,
    samples_per_model: usize,
    seed: u64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ModelInput {
    model_id: String,
    prior_weight: f64,
    mean: Vec<f64>,
    covariance: Vec<Vec<f64>>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoordinateInput {
    coordinate_id: String,
    modality: String,
    support_id: String,
    unit: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ActionInput {
    action_id: String,
    cost_units: u64,
    eligible: bool,
    coordinate_ids: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
struct ActionSummary {
    action_id: String,
    cost_units: u64,
    eligible: bool,
    feasible: bool,
    information_gain_nats: f64,
    monte_carlo_standard_error_nats: f64,
    samples_per_model: usize,
    total_samples: usize,
    seed: u64,
}

#[derive(Clone, Debug, Serialize)]
struct StageSummary {
    history_action_id: Option<String>,
    candidate_count: usize,
    feasible_candidate_count: usize,
    selected_action_id: Option<String>,
    candidates: Vec<ActionSummary>,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    scenario_sha256: String,
    status: String,
    prior_stage: StageSummary,
    conditional_stage: StageSummary,
    limitations: Vec<String>,
}

fn to_models(input: &[ModelInput]) -> Vec<MultivariateGaussianModel> {
    input.iter().map(|model| MultivariateGaussianModel {
        model_id: model.model_id.clone(),
        prior_weight: model.prior_weight,
        observation_model: MultivariateGaussian {
            mean: model.mean.clone(),
            covariance: model.covariance.clone(),
        },
    }).collect()
}

fn to_action(input: &ActionInput) -> ObservationAction {
    ObservationAction {
        action_id: input.action_id.clone(),
        coordinate_ids: input.coordinate_ids.clone(),
    }
}

fn validate_scenario(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != "0.1.0"
        || scenario.data_class != "synthetic-known-truth"
    {
        return Err("unsupported schema version or data class".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must be non-empty".into());
    }
    if scenario.models.is_empty() || scenario.coordinates.is_empty()
        || scenario.prior_actions.is_empty() || scenario.conditional_actions.is_empty()
    {
        return Err("models, coordinates and both candidate stages must be non-empty".into());
    }
    if scenario.samples_per_model < 2 {
        return Err("samples_per_model must be at least two".into());
    }
    if scenario.history_values.iter().any(|value| !value.is_finite()) {
        return Err("history values must be finite".into());
    }
    let dimensions = scenario.models[0].mean.len();
    if dimensions != scenario.coordinates.len() {
        return Err("model dimension must match the coordinate layout".into());
    }
    for model in &scenario.models {
        if model.mean.len() != dimensions
            || model.covariance.len() != dimensions
            || model.covariance.iter().any(|row| row.len() != dimensions)
        {
            return Err(format!("model {:?} has inconsistent covariance dimensions", model.model_id));
        }
    }
    Ok(())
}

fn summarize_stage(
    models: &[MultivariateGaussianModel],
    layout: &ObservationLayout,
    history: &ObservationHistory,
    actions: &[ActionInput],
    budget_units: u64,
    samples_per_model: usize,
    seed: u64,
    history_action_id: Option<String>,
) -> Result<StageSummary, String> {
    let mut candidates = Vec::with_capacity(actions.len());
    let mut ids = std::collections::BTreeSet::new();
    let mut selected: Option<(String, f64)> = None;
    let mut feasible_count = 0;

    for (index, action_input) in actions.iter().enumerate() {
        if action_input.action_id.trim().is_empty() || !ids.insert(&action_input.action_id) {
            return Err("action IDs must be non-empty and unique within each stage".into());
        }
        let action = to_action(action_input);
        action.validate(layout)?;
        let feasible = action_input.eligible && action_input.cost_units <= budget_units;
        let estimate = estimate_action_information_gain(
            models,
            layout,
            history,
            &action,
            samples_per_model,
            seed ^ (index as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15),
        )?;
        if feasible {
            feasible_count += 1;
            let replace = match &selected {
                None => true,
                Some((current_id, current_score)) => {
                    estimate.expected_information_nats > *current_score + 1e-12
                        || ((estimate.expected_information_nats - *current_score).abs() <= 1e-12
                            && action_input.action_id < *current_id)
                }
            };
            if replace {
                selected = Some((
                    action_input.action_id.clone(),
                    estimate.expected_information_nats,
                ));
            }
        }
        candidates.push(ActionSummary {
            action_id: action_input.action_id.clone(),
            cost_units: action_input.cost_units,
            eligible: action_input.eligible,
            feasible,
            information_gain_nats: estimate.expected_information_nats,
            monte_carlo_standard_error_nats: estimate.monte_carlo_standard_error_nats,
            samples_per_model: estimate.samples_per_model,
            total_samples: estimate.total_samples,
            seed: estimate.seed,
        });
    }

    Ok(StageSummary {
        history_action_id,
        candidate_count: candidates.len(),
        feasible_candidate_count: feasible_count,
        selected_action_id: selected.map(|(id, _)| id),
        candidates,
    })
}

fn build_report(scenario: &Scenario, input_bytes: &[u8]) -> Result<Report, String> {
    validate_scenario(scenario)?;
    let coordinates = scenario.coordinates.iter().map(|coordinate| ObservationCoordinate {
        coordinate_id: coordinate.coordinate_id.clone(),
        modality: coordinate.modality.clone(),
        support_id: coordinate.support_id.clone(),
        unit: coordinate.unit.clone(),
    }).collect();
    let layout = ObservationLayout::new(coordinates)?;
    let models = to_models(&scenario.models);
    for model in &models {
        model.observation_model.validate()?;
    }

    let prior_history = ObservationHistory::empty();
    let prior_stage = summarize_stage(
        &models,
        &layout,
        &prior_history,
        &scenario.prior_actions,
        scenario.budget_units,
        scenario.samples_per_model,
        scenario.seed,
        None,
    )?;

    let history_action_input = scenario.prior_actions.iter()
        .find(|action| action.action_id == scenario.history_action_id)
        .ok_or_else(|| "history_action_id must refer to a prior-stage action".to_string())?;
    if history_action_input.coordinate_ids.len() != scenario.history_values.len() {
        return Err("history_values must align with the history action coordinates".into());
    }
    let history = prior_history.with_appended(
        &layout,
        &to_action(history_action_input),
        &scenario.history_values,
    )?;
    let conditional_stage = summarize_stage(
        &models,
        &layout,
        &history,
        &scenario.conditional_actions,
        scenario.budget_units,
        scenario.samples_per_model,
        scenario.seed ^ 0xD6E8_FEB8_6659_FD93,
        Some(scenario.history_action_id.clone()),
    )?;

    Ok(Report {
        schema_version: "0.1.0".into(),
        data_class: "synthetic-known-truth".into(),
        scenario_id: scenario.scenario_id.clone(),
        scenario_sha256: format!("{:x}", Sha256::digest(input_bytes)),
        status: "synthetic_correlated_conditional_information_benchmark_only".into(),
        prior_stage,
        conditional_stage,
        limitations: vec![
            "Candidate models are finite synthetic Gaussian hypotheses.".into(),
            "Monte Carlo standard error does not include covariance-estimation or model discrepancy uncertainty.".into(),
            "Model information gain is not decision EVSI.".into(),
            "The CLI is an information-selection reference, not the full categorical sequential policy.".into(),
            "No CT image, tracer observation, physical specimen or field data is used.".into(),
        ],
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <scenario.json> <report.json>",
            args.first().map(String::as_str).unwrap_or("conditional-information-bakeoff")
        ));
    }
    let input = fs::read(&args[1]).map_err(|error| format!("scenario read failed: {error}"))?;
    let scenario: Scenario = serde_json::from_slice(&input)
        .map_err(|error| format!("scenario parse failed: {error}"))?;
    let report = build_report(&scenario, &input)?;
    let output = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("report serialization failed: {error}"))?;
    fs::write(&args[2], output).map_err(|error| format!("report write failed: {error}"))?;
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

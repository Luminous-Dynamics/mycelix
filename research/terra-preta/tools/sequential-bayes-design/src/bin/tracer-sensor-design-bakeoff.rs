use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, fs};
use terra_preta_sequential_bayes_design::multivariate_gaussian::{
    ConditionalGaussianModel, MultivariateGaussian,
};
use terra_preta_sequential_bayes_design::tracer_sensor_design::{
    compare_tracer_sensor_options, TracerSensorOption, TracerSensorScore,
};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    model_ids: Vec<String>,
    posterior_weights: Vec<f64>,
    predictive_means: Vec<f64>,
    predictive_variances: Vec<f64>,
    sensor_options: Vec<SensorInput>,
    budget_units: u64,
    sibson_alpha: f64,
    kl_radius_nats: f64,
    minimum_alpha: f64,
    alpha_bisection_iterations: usize,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorInput {
    sensor_id: String,
    cost_units: u64,
    eligible: bool,
    bin_boundaries: Vec<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct SelectionSummary {
    sensor_id: String,
    cost_units: u64,
    objective_value_nats: f64,
    alpha_used: f64,
}

#[derive(Clone, Debug, Serialize)]
struct SensorScoreSummary {
    sensor_id: String,
    cost_units: u64,
    eligible: bool,
    feasible: bool,
    outcome_ids: Vec<String>,
    shannon_information_nats: f64,
    sibson_information_nats: f64,
    per_design_alpha: f64,
    per_design_kl_radius_nats: f64,
    per_design_robust_information_nats: f64,
    alpha_search_clamped: bool,
}

impl From<&TracerSensorScore> for SensorScoreSummary {
    fn from(score: &TracerSensorScore) -> Self {
        Self {
            sensor_id: score.sensor_id.clone(),
            cost_units: score.cost_units,
            eligible: score.eligible,
            feasible: score.feasible,
            outcome_ids: score.outcome_ids.clone(),
            shannon_information_nats: score.shannon_information_nats,
            sibson_information_nats: score.sibson_information_nats,
            per_design_alpha: score.per_design_alpha,
            per_design_kl_radius_nats: score.per_design_kl_radius_nats,
            per_design_robust_information_nats: score.per_design_robust_information_nats,
            alpha_search_clamped: score.alpha_search_clamped,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    scenario_sha256: String,
    status: String,
    budget_units: u64,
    sibson_alpha: f64,
    kl_radius_nats: f64,
    minimum_alpha: f64,
    alpha_bisection_iterations: usize,
    nominal_shannon_selection: Option<SelectionSummary>,
    fixed_alpha_sibson_selection: Option<SelectionSummary>,
    fixed_kl_radius_selection: Option<SelectionSummary>,
    sensor_scores: Vec<SensorScoreSummary>,
    limitations: Vec<String>,
}

fn validate(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != "0.1.0"
        || scenario.data_class != "synthetic-known-truth"
    {
        return Err("only synthetic-known-truth schema version 0.1.0 is accepted".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must be non-empty".into());
    }
    let count = scenario.model_ids.len();
    if count < 2
        || scenario.posterior_weights.len() != count
        || scenario.predictive_means.len() != count
        || scenario.predictive_variances.len() != count
    {
        return Err("conditional model arrays must have equal length and at least two models".into());
    }
    if scenario.predictive_means.iter().any(|value| !value.is_finite())
        || scenario.predictive_variances.iter().any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err("predictive means/variances must be finite with positive variances".into());
    }
    if !scenario.sibson_alpha.is_finite() || !(0.0..1.0).contains(&scenario.sibson_alpha) {
        return Err("Sibson alpha must lie strictly between zero and one".into());
    }
    if !scenario.kl_radius_nats.is_finite() || scenario.kl_radius_nats < 0.0 {
        return Err("KL radius must be finite and non-negative".into());
    }
    if !scenario.minimum_alpha.is_finite()
        || !(0.0..1.0).contains(&scenario.minimum_alpha)
        || scenario.minimum_alpha >= 1.0 - 1e-8
    {
        return Err("minimum alpha must lie strictly between zero and 1 - 1e-8".into());
    }
    if scenario.alpha_bisection_iterations == 0 {
        return Err("alpha_bisection_iterations must be positive".into());
    }
    if scenario.sensor_options.is_empty() {
        return Err("at least one sensor configuration is required".into());
    }
    Ok(())
}

fn select_summary(
    selection: Option<terra_preta_sequential_bayes_design::robust_information::DiscreteDesignSelection>,
    alpha: f64,
) -> Option<SelectionSummary> {
    selection.map(|selection| SelectionSummary {
        sensor_id: selection.action_id,
        cost_units: selection.cost_units,
        objective_value_nats: selection.objective_value_nats,
        alpha_used: alpha,
    })
}

fn build_report(scenario: &Scenario, input: &[u8]) -> Result<Report, String> {
    validate(scenario)?;
    let components: Vec<ConditionalGaussianModel> = scenario.model_ids.iter()
        .enumerate()
        .map(|(index, model_id)| ConditionalGaussianModel {
            model_id: model_id.clone(),
            posterior_weight: scenario.posterior_weights[index],
            predictive_distribution: MultivariateGaussian {
                mean: vec![scenario.predictive_means[index]],
                covariance: vec![vec![scenario.predictive_variances[index]]],
            },
        })
        .collect();

    let options: Vec<TracerSensorOption> = scenario.sensor_options.iter()
        .map(|option| TracerSensorOption {
            sensor_id: option.sensor_id.clone(),
            cost_units: option.cost_units,
            eligible: option.eligible,
            bin_boundaries: option.bin_boundaries.clone(),
        })
        .collect();

    let result = compare_tracer_sensor_options(
        &components,
        &options,
        scenario.budget_units,
        scenario.sibson_alpha,
        scenario.kl_radius_nats,
        scenario.minimum_alpha,
        scenario.alpha_bisection_iterations,
    )?;

    let fixed_radius = result.fixed_kl_radius_selection.map(|(selection, score)| {
        SelectionSummary {
            sensor_id: selection.action_id,
            cost_units: selection.cost_units,
            objective_value_nats: score.sibson_information_nats,
            alpha_used: score.alpha,
        }
    });

    Ok(Report {
        schema_version: "0.1.0".into(),
        data_class: "synthetic-known-truth".into(),
        scenario_id: scenario.scenario_id.clone(),
        scenario_sha256: format!("{:x}", Sha256::digest(input)),
        status: "synthetic_detection_aware_tracer_sensor_design_only".into(),
        budget_units: scenario.budget_units,
        sibson_alpha: scenario.sibson_alpha,
        kl_radius_nats: scenario.kl_radius_nats,
        minimum_alpha: scenario.minimum_alpha,
        alpha_bisection_iterations: scenario.alpha_bisection_iterations,
        nominal_shannon_selection: select_summary(result.nominal_selection, 1.0),
        fixed_alpha_sibson_selection: select_summary(
            result.fixed_alpha_selection,
            scenario.sibson_alpha,
        ),
        fixed_kl_radius_selection: fixed_radius,
        sensor_scores: result.scores.iter().map(SensorScoreSummary::from).collect(),
        limitations: vec![
            "Sensor options are synthetic censoring-bin configurations.".into(),
            "Information gain is model discrimination, not decision EVSI or physical recovery.".into(),
            "The Gaussian CDF and binning are reference calculations, not detector calibration.".into(),
            "No tracer dose, toxicity, analytical chemistry or laboratory detection-limit data are modeled.".into(),
        ],
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <scenario.json> <report.json>",
            args.first().map(String::as_str).unwrap_or("tracer-sensor-design-bakeoff")
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

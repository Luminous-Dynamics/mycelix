use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, fs};
use terra_preta_sequential_bayes_design::ensemble_assimilation::GaussianEnsemble;
use terra_preta_sequential_bayes_design::multivariate_gaussian::MultivariateGaussian;
use terra_preta_sequential_bayes_design::state_parameter_assimilation::{
    AssimilationVariable, AssimilationVariableRole, LinearGaussianObservationOperator,
    LinearGaussianStateParameterAssimilator, StateParameterGaussianPrior,
};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    variables: Vec<VariableInput>,
    prior_mean: Vec<f64>,
    prior_covariance: Vec<Vec<f64>>,
    observation_ids: Vec<String>,
    observation_matrix: Vec<Vec<f64>>,
    observation_noise_covariance: Vec<Vec<f64>>,
    observed_values: Vec<f64>,
    ensemble_sizes: Vec<usize>,
    replicates_per_size: usize,
    seed: u64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct VariableInput {
    variable_id: String,
    role: String,
    unit: String,
}

#[derive(Clone, Debug, Serialize)]
struct EnsembleRun {
    ensemble_size: usize,
    replicate_index: usize,
    seed: u64,
    posterior_mean_rmse: f64,
    posterior_covariance_relative_frobenius_error: f64,
}

#[derive(Clone, Debug, Serialize)]
struct EnsembleSizeSummary {
    ensemble_size: usize,
    replicate_count: usize,
    mean_posterior_mean_rmse: f64,
    posterior_mean_rmse_standard_error: f64,
    mean_posterior_covariance_relative_error: f64,
    posterior_covariance_relative_error_standard_error: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    scenario_sha256: String,
    status: String,
    exact_posterior_mean: Vec<f64>,
    exact_posterior_covariance: Vec<Vec<f64>>,
    exact_log_marginal_likelihood: f64,
    ensemble_size_summaries: Vec<EnsembleSizeSummary>,
    runs: Vec<EnsembleRun>,
    limitations: Vec<String>,
}

fn parse_role(value: &str) -> Result<AssimilationVariableRole, String> {
    match value {
        "physical_state" => Ok(AssimilationVariableRole::PhysicalState),
        "hydraulic_parameter" => Ok(AssimilationVariableRole::HydraulicParameter),
        "exchange_parameter" => Ok(AssimilationVariableRole::ExchangeParameter),
        "measurement_bias" => Ok(AssimilationVariableRole::MeasurementBias),
        "other_nuisance" => Ok(AssimilationVariableRole::OtherNuisance),
        _ => Err(format!("unsupported assimilation variable role {value:?}")),
    }
}

fn mean_and_standard_error(values: &[f64]) -> Result<(f64, f64), String> {
    if values.len() < 2 || values.iter().any(|value| !value.is_finite()) {
        return Err("at least two finite replicate errors are required".into());
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values.iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>() / (values.len() - 1) as f64;
    Ok((mean, (variance / values.len() as f64).sqrt()))
}

fn posterior_mean_rmse(actual: &[f64], exact: &[f64]) -> Result<f64, String> {
    if actual.len() != exact.len() || actual.is_empty() {
        return Err("posterior mean vectors must be non-empty and aligned".into());
    }
    let squared = actual.iter().zip(exact)
        .map(|(left, right)| (left - right).powi(2))
        .sum::<f64>();
    Ok((squared / actual.len() as f64).sqrt())
}

fn covariance_relative_error(actual: &[Vec<f64>], exact: &[Vec<f64>]) -> Result<f64, String> {
    if actual.len() != exact.len()
        || actual.is_empty()
        || actual.iter().zip(exact).any(|(left, right)| left.len() != right.len())
    {
        return Err("posterior covariance matrices must have matching non-empty dimensions".into());
    }
    let squared_error = actual.iter().zip(exact)
        .flat_map(|(left, right)| left.iter().zip(right))
        .map(|(left, right)| (left - right).powi(2))
        .sum::<f64>();
    let squared_reference = exact.iter().flatten()
        .map(|value| value * value)
        .sum::<f64>();
    Ok(squared_error.sqrt() / squared_reference.sqrt().max(1e-12))
}

fn build_report(scenario: &Scenario, input_bytes: &[u8]) -> Result<Report, String> {
    if scenario.schema_version != "0.1.0"
        || scenario.data_class != "synthetic-known-truth"
    {
        return Err("only synthetic-known-truth schema version 0.1.0 is accepted".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must be non-empty".into());
    }
    if scenario.variables.is_empty()
        || scenario.prior_mean.len() != scenario.variables.len()
        || scenario.prior_covariance.len() != scenario.variables.len()
        || scenario.prior_covariance.iter().any(|row| row.len() != scenario.variables.len())
    {
        return Err("augmented variable metadata and prior dimensions must align".into());
    }
    if scenario.observation_ids.is_empty()
        || scenario.observation_ids.len() != scenario.observation_matrix.len()
        || scenario.observation_ids.len() != scenario.observed_values.len()
    {
        return Err("observation IDs, operator rows and observed values must align".into());
    }
    if scenario.ensemble_sizes.is_empty()
        || scenario.ensemble_sizes.iter().any(|size| *size < scenario.variables.len() + 2)
    {
        return Err("ensemble sizes must each be at least latent dimension + 2".into());
    }
    if scenario.replicates_per_size < 2 {
        return Err("at least two ensemble replicates per size are required".into());
    }
    if scenario.ensemble_sizes.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err("ensemble sizes must be strictly increasing and unique".into());
    }

    let variables: Vec<AssimilationVariable> = scenario.variables.iter()
        .map(|variable| Ok(AssimilationVariable {
            variable_id: variable.variable_id.clone(),
            role: parse_role(&variable.role)?,
            unit: variable.unit.clone(),
        }))
        .collect::<Result<Vec<_>, String>>()?;
    let prior = StateParameterGaussianPrior {
        variables,
        distribution: MultivariateGaussian {
            mean: scenario.prior_mean.clone(),
            covariance: scenario.prior_covariance.clone(),
        },
    };
    let observation_operator = LinearGaussianObservationOperator {
        observation_ids: scenario.observation_ids.clone(),
        matrix: scenario.observation_matrix.clone(),
        noise_covariance: scenario.observation_noise_covariance.clone(),
    };
    let exact = LinearGaussianStateParameterAssimilator::new(
        prior.clone(),
        observation_operator.clone(),
    )?.update(&scenario.observed_values)?;

    let mut summaries = Vec::with_capacity(scenario.ensemble_sizes.len());
    let mut runs = Vec::with_capacity(
        scenario.ensemble_sizes.len() * scenario.replicates_per_size,
    );
    for (size_index, ensemble_size) in scenario.ensemble_sizes.iter().copied().enumerate() {
        let mut mean_errors = Vec::with_capacity(scenario.replicates_per_size);
        let mut covariance_errors = Vec::with_capacity(scenario.replicates_per_size);
        for replicate_index in 0..scenario.replicates_per_size {
            let seed = scenario.seed
                ^ (size_index as u64).wrapping_mul(0xD6E8_FEB8_6659_FD93)
                ^ (replicate_index as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
            let ensemble = GaussianEnsemble::sample(&prior, ensemble_size, seed)?;
            let approximation = ensemble.analyze(
                &observation_operator,
                &scenario.observed_values,
            )?;
            let mean_error = posterior_mean_rmse(
                &approximation.posterior.distribution.mean,
                &exact.distribution.mean,
            )?;
            let covariance_error = covariance_relative_error(
                &approximation.posterior.distribution.covariance,
                &exact.distribution.covariance,
            )?;
            mean_errors.push(mean_error);
            covariance_errors.push(covariance_error);
            runs.push(EnsembleRun {
                ensemble_size,
                replicate_index,
                seed,
                posterior_mean_rmse: mean_error,
                posterior_covariance_relative_frobenius_error: covariance_error,
            });
        }
        let (mean_posterior_mean_rmse, posterior_mean_rmse_standard_error) =
            mean_and_standard_error(&mean_errors)?;
        let (
            mean_posterior_covariance_relative_error,
            posterior_covariance_relative_error_standard_error,
        ) = mean_and_standard_error(&covariance_errors)?;
        summaries.push(EnsembleSizeSummary {
            ensemble_size,
            replicate_count: scenario.replicates_per_size,
            mean_posterior_mean_rmse,
            posterior_mean_rmse_standard_error,
            mean_posterior_covariance_relative_error,
            posterior_covariance_relative_error_standard_error,
        });
    }

    Ok(Report {
        schema_version: "0.1.0".into(),
        data_class: "synthetic-known-truth".into(),
        scenario_id: scenario.scenario_id.clone(),
        scenario_sha256: format!("{:x}", Sha256::digest(input_bytes)),
        status: "synthetic_exact_oracle_vs_ensemble_kalman_reference".into(),
        exact_posterior_mean: exact.distribution.mean,
        exact_posterior_covariance: exact.distribution.covariance,
        exact_log_marginal_likelihood: exact.log_marginal_likelihood,
        ensemble_size_summaries: summaries,
        runs,
        limitations: vec![
            "Exact truth is available because this is a finite linear-Gaussian synthetic model.".into(),
            "The approximation is a deterministic EnKF-style Gaussian moment update, not a perturbed-observation or square-root filter.".into(),
            "No nonlinear Richards, dual-permeability or physical soil data are assimilated.".into(),
            "Posterior mean/covariance errors do not alone establish calibrated tails or decision value.".into(),
        ],
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <scenario.json> <report.json>",
            args.first().map(String::as_str).unwrap_or("state-parameter-assimilation-bakeoff")
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

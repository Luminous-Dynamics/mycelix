use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, fs};
use terra_preta_sequential_bayes_design::censored_tracer::{
    build_censored_discrete_design, update_censored_model_posterior,
    CensoredTracerObservation,
};
use terra_preta_sequential_bayes_design::multivariate_gaussian::{
    conditional_predictive_models, MultivariateGaussian, MultivariateGaussianModel,
};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    model_ids: Vec<String>,
    prior_weights: Vec<f64>,
    means: Vec<Vec<f64>>,
    covariances: Vec<Vec<Vec<f64>>>,
    observed_ct_value: f64,
    detection_limit: f64,
    upper_quantification_limit: f64,
    sibson_alpha: f64,
}

#[derive(Clone, Debug, Serialize)]
struct ConditionalModelSummary {
    model_id: String,
    posterior_weight_after_ct: f64,
    tracer_conditional_mean: f64,
    tracer_conditional_standard_deviation: f64,
}

#[derive(Clone, Debug, Serialize)]
struct CensoredBinSummary {
    outcome_id: String,
    model_probabilities: Vec<f64>,
    prior_predictive_probability: f64,
}

#[derive(Clone, Debug, Serialize)]
struct PosteriorSummary {
    model_ids: Vec<String>,
    weights: Vec<f64>,
    log_evidence: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    scenario_sha256: String,
    status: String,
    observed_ct_value: f64,
    detection_limit: f64,
    upper_quantification_limit: f64,
    conditional_models: Vec<ConditionalModelSummary>,
    censored_bins: Vec<CensoredBinSummary>,
    binned_shannon_information_nats: f64,
    binned_sibson_information_nats: f64,
    below_detection_posterior: PosteriorSummary,
    missing_at_random_posterior: PosteriorSummary,
    incorrect_zero_substitution_posterior: PosteriorSummary,
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
        || scenario.prior_weights.len() != count
        || scenario.means.len() != count
        || scenario.covariances.len() != count
    {
        return Err("candidate model arrays must have equal length and at least two models".into());
    }
    if !scenario.observed_ct_value.is_finite()
        || !scenario.detection_limit.is_finite()
        || !scenario.upper_quantification_limit.is_finite()
        || scenario.upper_quantification_limit <= scenario.detection_limit
    {
        return Err("CT value and censoring thresholds are invalid".into());
    }
    if !scenario.sibson_alpha.is_finite() || !(0.0..1.0).contains(&scenario.sibson_alpha) {
        return Err("Sibson alpha must lie strictly between zero and one".into());
    }
    for index in 0..count {
        if scenario.means[index].len() != 2
            || scenario.covariances[index].len() != 2
            || scenario.covariances[index].iter().any(|row| row.len() != 2)
        {
            return Err(format!("model {:?} must define a bivariate Gaussian", scenario.model_ids[index]));
        }
    }
    Ok(())
}

fn posterior_summary(
    model_ids: &[String],
    weights: &[f64],
    log_evidence: f64,
) -> PosteriorSummary {
    PosteriorSummary {
        model_ids: model_ids.to_vec(),
        weights: weights.to_vec(),
        log_evidence,
    }
}

fn build_report(scenario: &Scenario, input: &[u8]) -> Result<Report, String> {
    validate(scenario)?;
    let models: Vec<MultivariateGaussianModel> = scenario.model_ids.iter()
        .enumerate()
        .map(|(index, model_id)| MultivariateGaussianModel {
            model_id: model_id.clone(),
            prior_weight: scenario.prior_weights[index],
            observation_model: MultivariateGaussian {
                mean: scenario.means[index].clone(),
                covariance: scenario.covariances[index].clone(),
            },
        })
        .collect();
    for model in &models {
        model.observation_model.validate()?;
    }

    // CT is observed exactly here. The tracer prediction is conditional on
    // that exact CT history, not treated as an independent marginal.
    let conditional = conditional_predictive_models(
        &models,
        &[0],
        &[scenario.observed_ct_value],
        &[1],
    )?;

    let boundary_values = [
        scenario.detection_limit,
        scenario.upper_quantification_limit,
    ];
    let binned = build_censored_discrete_design(&conditional, &boundary_values)?;
    let shannon = binned.shannon_mutual_information()?;
    let sibson = binned.sibson_alpha_mutual_information(scenario.sibson_alpha)?;

    let bins = binned.outcome_ids.iter().enumerate()
        .map(|(outcome, outcome_id)| {
            let model_probabilities: Vec<f64> = binned.likelihoods.iter()
                .map(|row| row[outcome])
                .collect();
            let prior_predictive_probability: f64 = model_probabilities.iter()
                .zip(&binned.prior_weights)
                .map(|(probability, prior)| probability * prior)
                .sum();
            CensoredBinSummary {
                outcome_id: outcome_id.clone(),
                model_probabilities,
                prior_predictive_probability,
            }
        })
        .collect();

    let below = update_censored_model_posterior(
        &conditional,
        &CensoredTracerObservation::BelowDetection {
            detection_limit: scenario.detection_limit,
        },
    )?;
    let missing = update_censored_model_posterior(
        &conditional,
        &CensoredTracerObservation::MissingAtRandom,
    )?;

    // Deliberately incorrect negative control: treat "below detection" as an
    // exact numerical zero and use a point density. The proper likelihood is
    // a CDF probability integrated over (-infinity, LOD].
    let mut log_point_likelihoods = Vec::with_capacity(conditional.len());
    for component in &conditional {
        log_point_likelihoods.push(
            component.predictive_distribution.log_density(&[0.0])?
                + component.posterior_weight.ln(),
        );
    }
    let maximum = log_point_likelihoods.iter().copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let normalizer_scaled: f64 = log_point_likelihoods.iter()
        .map(|value| (value - maximum).exp())
        .sum();
    if !maximum.is_finite() || !normalizer_scaled.is_finite() || normalizer_scaled <= 0.0 {
        return Err("zero-substitution negative-control posterior normalization failed".into());
    }
    let point_log_evidence = maximum + normalizer_scaled.ln();
    let point_weights: Vec<f64> = log_point_likelihoods.iter()
        .map(|value| (value - point_log_evidence).exp())
        .collect();

    Ok(Report {
        schema_version: "0.1.0".into(),
        data_class: "synthetic-known-truth".into(),
        scenario_id: scenario.scenario_id.clone(),
        scenario_sha256: format!("{:x}", Sha256::digest(input)),
        status: "synthetic_censored_tracer_observation_reference_only".into(),
        observed_ct_value: scenario.observed_ct_value,
        detection_limit: scenario.detection_limit,
        upper_quantification_limit: scenario.upper_quantification_limit,
        conditional_models: conditional.iter()
            .map(|component| ConditionalModelSummary {
                model_id: component.model_id.clone(),
                posterior_weight_after_ct: component.posterior_weight,
                tracer_conditional_mean: component.predictive_distribution.mean[0],
                tracer_conditional_standard_deviation:
                    component.predictive_distribution.covariance[0][0].sqrt(),
            })
            .collect(),
        censored_bins: bins,
        binned_shannon_information_nats: shannon,
        binned_sibson_information_nats: sibson,
        below_detection_posterior: posterior_summary(
            &below.model_ids, &below.weights, below.log_evidence,
        ),
        missing_at_random_posterior: posterior_summary(
            &missing.model_ids, &missing.weights, missing.log_evidence,
        ),
        incorrect_zero_substitution_posterior: posterior_summary(
            &conditional.iter().map(|item| item.model_id.clone()).collect::<Vec<_>>(),
            &point_weights,
            point_log_evidence,
        ),
        limitations: vec![
            "Univariate Gaussian censoring is applied to a caller-declared scalar scale such as log concentration or arrival time.".into(),
            "The CDF uses a documented numerical approximation; no instrument calibration model is fitted.".into(),
            "The point-zero substitution posterior is a deliberately incorrect negative control, not an endorsed estimator.".into(),
            "The fixture is synthetic and does not validate soil tracer physics.".into(),
            "Missing-at-random is an assumption; non-ignorable missingness needs an explicit model.".into(),
        ],
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <scenario.json> <report.json>",
            args.first().map(String::as_str).unwrap_or("censored-tracer-bakeoff")
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

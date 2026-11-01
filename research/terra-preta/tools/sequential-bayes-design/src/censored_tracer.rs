//! Detection-limit and interval-censoring likelihoods for tracer observations.
//!
//! A censored tracer observation is an event, not a numeric replacement:
//! below a detection limit means Y <= LOD, interval censoring means
//! lower < Y <= upper, and upper censoring means Y > UQL. Missing-at-random
//! means no likelihood update; informative missingness requires an explicit
//! model-specific probability.
//!
//! The observation variable is a scalar Gaussian feature in the caller's
//! declared scale (for example, log concentration or tracer arrival time).
//! This module does not infer the transformation, calibrate detection limits,
//! or model the instrument's false-positive/false-negative process.

use crate::information_gain::ConditionalGaussianModel;
use crate::robust_information::DiscreteDesignModel;
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq)]
pub enum CensoredTracerObservation {
    /// Event Y <= detection_limit.
    BelowDetection { detection_limit: f64 },
    /// Event lower_bound < Y <= upper_bound.
    IntervalCensored { lower_bound: f64, upper_bound: f64 },
    /// Event Y > upper_quantification_limit.
    AboveQuantification { upper_quantification_limit: f64 },
    /// Ignorable/missing-at-random: no observation likelihood is applied.
    MissingAtRandom,
    /// Informative missingness: explicit p(missing | candidate model).
    MissingWithModelProbabilities { probabilities: Vec<f64> },
}

#[derive(Clone, Debug, PartialEq)]
pub struct CensoredModelPosterior {
    pub model_ids: Vec<String>,
    pub weights: Vec<f64>,
    /// Log probability of the observed censoring event under the prior mixture.
    /// Missing-at-random returns 0 because no event likelihood is applied.
    pub log_evidence: f64,
    /// Event likelihood for each candidate model. Missing-at-random is all ones.
    pub model_event_probabilities: Vec<f64>,
    pub observation_was_missing: bool,
}

fn erfc_nonnegative(x: f64) -> f64 {
    // Numerical Recipes complementary-error-function approximation.
    // Absolute error is approximately 1.5e-7 over the real line.
    let z = x.abs();
    let t = 1.0 / (1.0 + 0.5 * z);
    let polynomial = -1.265_512_23
        + t * (1.000_023_68
        + t * (0.374_091_96
        + t * (0.096_784_18
        + t * (-0.186_288_06
        + t * (0.278_868_07
        + t * (-1.135_203_98
        + t * (1.488_515_87
        + t * (-0.822_152_23
        + t * 0.170_872_77))))))));
    t * (-z * z + polynomial).exp()
}

/// Standard normal CDF.
pub fn standard_normal_cdf(value: f64) -> Result<f64, String> {
    if !value.is_finite() {
        return Err("standard-normal CDF input must be finite".into());
    }
    let scaled = value.abs() / std::f64::consts::SQRT_2;
    let tail = 0.5 * erfc_nonnegative(scaled);
    let probability = if value < 0.0 { tail } else { 1.0 - tail };
    Ok(probability.clamp(0.0, 1.0))
}

/// Standard normal survival probability, evaluated directly in the upper tail.
pub fn standard_normal_survival(value: f64) -> Result<f64, String> {
    if !value.is_finite() {
        return Err("standard-normal survival input must be finite".into());
    }
    let scaled = value.abs() / std::f64::consts::SQRT_2;
    let tail = 0.5 * erfc_nonnegative(scaled);
    let probability = if value > 0.0 { tail } else { 1.0 - tail };
    Ok(probability.clamp(0.0, 1.0))
}

fn normal_interval_probability(
    lower: f64,
    upper: f64,
    mean: f64,
    standard_deviation: f64,
) -> Result<f64, String> {
    if !lower.is_finite() || !upper.is_finite() || upper <= lower {
        return Err("censoring interval must have finite upper > lower bounds".into());
    }
    let lower_z = (lower - mean) / standard_deviation;
    let upper_z = (upper - mean) / standard_deviation;

    // Use a direct upper-tail difference when the interval is wholly positive
    // to avoid subtracting two CDF values rounded close to one.
    let probability = if lower_z >= 0.0 {
        standard_normal_survival(lower_z)? - standard_normal_survival(upper_z)?
    } else {
        standard_normal_cdf(upper_z)? - standard_normal_cdf(lower_z)?
    };
    if probability < -1e-12 || !probability.is_finite() {
        return Err("normal interval probability is invalid".into());
    }
    Ok(probability.max(0.0))
}

fn validate_components(
    components: &[ConditionalGaussianModel],
) -> Result<Vec<f64>, String> {
    if components.is_empty() {
        return Err("at least one conditional Gaussian model is required".into());
    }
    let mut ids = BTreeSet::new();
    let mut weight_total = 0.0;
    for component in components {
        if component.model_id.trim().is_empty()
            || !ids.insert(component.model_id.as_str())
        {
            return Err("model IDs must be non-empty and unique".into());
        }
        if !component.posterior_weight.is_finite() || component.posterior_weight < 0.0 {
            return Err("posterior weights must be finite and non-negative".into());
        }
        if component.predictive_distribution.mean.len() != 1
            || component.predictive_distribution.covariance.len() != 1
            || component.predictive_distribution.covariance[0].len() != 1
        {
            return Err("censored tracer likelihood currently requires a univariate predictive Gaussian".into());
        }
        component.predictive_distribution.validate()?;
        weight_total += component.posterior_weight;
    }
    if !weight_total.is_finite() || weight_total <= 0.0 {
        return Err("conditional Gaussian weights must have finite positive sum".into());
    }
    Ok(components.iter()
        .map(|component| component.posterior_weight / weight_total)
        .collect())
}

/// Probability of the declared censoring event under one scalar Gaussian.
pub fn censored_event_probability(
    mean: f64,
    variance: f64,
    observation: &CensoredTracerObservation,
) -> Result<f64, String> {
    if !mean.is_finite() || !variance.is_finite() || variance <= 0.0 {
        return Err("Gaussian mean must be finite and variance must be finite and positive".into());
    }
    let standard_deviation = variance.sqrt();
    match observation {
        CensoredTracerObservation::BelowDetection { detection_limit } => {
            if !detection_limit.is_finite() {
                return Err("detection limit must be finite".into());
            }
            standard_normal_cdf((detection_limit - mean) / standard_deviation)
        }
        CensoredTracerObservation::IntervalCensored { lower_bound, upper_bound } => {
            normal_interval_probability(*lower_bound, *upper_bound, mean, standard_deviation)
        }
        CensoredTracerObservation::AboveQuantification { upper_quantification_limit } => {
            if !upper_quantification_limit.is_finite() {
                return Err("upper quantification limit must be finite".into());
            }
            standard_normal_survival(
                (upper_quantification_limit - mean) / standard_deviation,
            )
        }
        CensoredTracerObservation::MissingAtRandom => Ok(1.0),
        CensoredTracerObservation::MissingWithModelProbabilities { .. } => {
            Err("model-specific missingness probabilities require the full model posterior updater".into())
        }
    }
}

/// Update a finite posterior from one censored scalar observation.
///
/// Below-detection values are integrated as CDF probabilities. They are never
/// replaced by zero, LOD/2 or the detection threshold. Missing-at-random
/// observations preserve the incoming posterior exactly.
pub fn update_censored_model_posterior(
    components: &[ConditionalGaussianModel],
    observation: &CensoredTracerObservation,
) -> Result<CensoredModelPosterior, String> {
    let prior = validate_components(components)?;
    let mut likelihoods = Vec::with_capacity(components.len());

    match observation {
        CensoredTracerObservation::MissingAtRandom => {
            return Ok(CensoredModelPosterior {
                model_ids: components.iter().map(|component| component.model_id.clone()).collect(),
                weights: prior,
                log_evidence: 0.0,
                model_event_probabilities: vec![1.0; components.len()],
                observation_was_missing: true,
            });
        }
        CensoredTracerObservation::MissingWithModelProbabilities { probabilities } => {
            if probabilities.len() != components.len()
                || probabilities.iter().any(|value| !value.is_finite() || *value < 0.0 || *value > 1.0)
            {
                return Err("model-specific missing probabilities must align and lie in [0,1]".into());
            }
            likelihoods.extend(probabilities.iter().copied());
        }
        _ => {
            for component in components {
                let mean = component.predictive_distribution.mean[0];
                let variance = component.predictive_distribution.covariance[0][0];
                likelihoods.push(censored_event_probability(mean, variance, observation)?);
            }
        }
    }

    let log_joint: Vec<f64> = prior.iter().zip(&likelihoods)
        .map(|(weight, likelihood)| {
            if *weight == 0.0 || *likelihood == 0.0 {
                f64::NEG_INFINITY
            } else {
                weight.ln() + likelihood.ln()
            }
        })
        .collect();
    let maximum = log_joint.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !maximum.is_finite() {
        return Err("all candidate models assign zero probability to the censored event".into());
    }
    let scaled_sum: f64 = log_joint.iter()
        .map(|value| (value - maximum).exp())
        .sum();
    if !scaled_sum.is_finite() || scaled_sum <= 0.0 {
        return Err("censored posterior normalization failed".into());
    }
    let log_evidence = maximum + scaled_sum.ln();
    let weights: Vec<f64> = log_joint.iter()
        .map(|value| (value - log_evidence).exp())
        .collect();

    Ok(CensoredModelPosterior {
        model_ids: components.iter().map(|component| component.model_id.clone()).collect(),
        weights,
        log_evidence,
        model_event_probabilities: likelihoods,
        observation_was_missing: false,
    })
}

/// Build a finite discrete design from a scalar Gaussian predictive mixture.
///
/// Finite ascending boundaries partition the real line into:
/// (-inf, b0], (b0, b1], ..., (b_last, +inf).
/// Each row contains true event probabilities, preserving censored outcomes
/// as intervals rather than point substitutions.
pub fn build_censored_discrete_design(
    components: &[ConditionalGaussianModel],
    bin_boundaries: &[f64],
) -> Result<DiscreteDesignModel, String> {
    let prior = validate_components(components)?;
    if bin_boundaries.is_empty()
        || bin_boundaries.iter().any(|value| !value.is_finite())
        || bin_boundaries.windows(2).any(|pair| pair[1] <= pair[0])
    {
        return Err("censoring bin boundaries must be non-empty, finite and strictly increasing".into());
    }

    let mut outcome_ids = Vec::with_capacity(bin_boundaries.len() + 1);
    outcome_ids.push(format!("below_or_equal_{:.12e}", bin_boundaries[0]));
    for pair in bin_boundaries.windows(2) {
        outcome_ids.push(format!("between_{:.12e}_{:.12e}", pair[0], pair[1]));
    }
    outcome_ids.push(format!("above_{:.12e}", bin_boundaries[bin_boundaries.len() - 1]));

    let mut likelihoods = Vec::with_capacity(components.len());
    for component in components {
        let mean = component.predictive_distribution.mean[0];
        let variance = component.predictive_distribution.covariance[0][0];
        let standard_deviation = variance.sqrt();
        let mut row = Vec::with_capacity(outcome_ids.len());

        row.push(standard_normal_cdf(
            (bin_boundaries[0] - mean) / standard_deviation,
        )?);
        for pair in bin_boundaries.windows(2) {
            row.push(normal_interval_probability(
                pair[0], pair[1], mean, standard_deviation,
            )?);
        }
        row.push(standard_normal_survival(
            (bin_boundaries[bin_boundaries.len() - 1] - mean) / standard_deviation,
        )?);

        let total: f64 = row.iter().sum();
        if row.iter().any(|value| !value.is_finite() || *value < 0.0)
            || (total - 1.0).abs() > 2e-6
        {
            return Err(format!("censored probability bins sum to {total}, expected one"));
        }
        // Remove only approximation-level cumulative roundoff while preserving
        // the relative event probabilities.
        for probability in &mut row {
            *probability /= total;
        }
        likelihoods.push(row);
    }

    Ok(DiscreteDesignModel {
        model_ids: components.iter().map(|component| component.model_id.clone()).collect(),
        prior_weights: prior,
        outcome_ids,
        likelihoods,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multivariate_gaussian::MultivariateGaussian;

    fn component(id: &str, weight: f64, mean: f64, standard_deviation: f64) -> ConditionalGaussianModel {
        ConditionalGaussianModel {
            model_id: id.into(),
            posterior_weight: weight,
            predictive_distribution: MultivariateGaussian {
                mean: vec![mean],
                covariance: vec![vec![standard_deviation * standard_deviation]],
            },
        }
    }

    fn tracer_components() -> Vec<ConditionalGaussianModel> {
        vec![
            component("early_low_concentration", 0.5, 0.1, 0.3),
            component("late_high_concentration", 0.5, 1.0, 0.3),
        ]
    }

    #[test]
    fn standard_normal_cdf_matches_reference_values() {
        assert!((standard_normal_cdf(0.0).unwrap() - 0.5).abs() < 2e-7);
        assert!((standard_normal_cdf(1.0).unwrap() - 0.841344746).abs() < 2e-7);
        assert!((standard_normal_cdf(-1.0).unwrap() - 0.158655254).abs() < 2e-7);
        assert!((standard_normal_survival(1.0).unwrap() - 0.158655254).abs() < 2e-7);
    }

    #[test]
    fn below_detection_updates_with_cdf_probability_not_zero_substitution() {
        let posterior = update_censored_model_posterior(
            &tracer_components(),
            &CensoredTracerObservation::BelowDetection { detection_limit: 0.5 },
        ).unwrap();
        assert!(posterior.weights[0] > 0.95);
        assert!(posterior.weights[1] < 0.05);
        assert!(posterior.model_event_probabilities[0] > posterior.model_event_probabilities[1]);
    }

    #[test]
    fn interval_censoring_updates_from_interval_mass() {
        let posterior = update_censored_model_posterior(
            &tracer_components(),
            &CensoredTracerObservation::IntervalCensored {
                lower_bound: 0.8,
                upper_bound: 1.2,
            },
        ).unwrap();
        assert!(posterior.weights[1] > 0.99);
    }

    #[test]
    fn upper_censoring_updates_from_survival_probability() {
        let posterior = update_censored_model_posterior(
            &tracer_components(),
            &CensoredTracerObservation::AboveQuantification {
                upper_quantification_limit: 0.8,
            },
        ).unwrap();
        assert!(posterior.weights[1] > 0.99);
    }

    #[test]
    fn missing_at_random_does_not_invent_zero_or_update_the_posterior() {
        let components = tracer_components();
        let posterior = update_censored_model_posterior(
            &components,
            &CensoredTracerObservation::MissingAtRandom,
        ).unwrap();
        assert!(posterior.observation_was_missing);
        assert_eq!(posterior.weights, vec![0.5, 0.5]);
        assert_eq!(posterior.model_event_probabilities, vec![1.0, 1.0]);
        assert_eq!(posterior.log_evidence, 0.0);
    }

    #[test]
    fn informative_missingness_requires_explicit_model_probabilities() {
        let posterior = update_censored_model_posterior(
            &tracer_components(),
            &CensoredTracerObservation::MissingWithModelProbabilities {
                probabilities: vec![0.9, 0.1],
            },
        ).unwrap();
        assert!(posterior.weights[0] > 0.89);
        assert!(posterior.weights[1] < 0.11);
    }

    #[test]
    fn censored_bins_sum_to_one_and_have_valid_discrete_design_information() {
        let design = build_censored_discrete_design(&tracer_components(), &[0.5, 1.0]).unwrap();
        for row in &design.likelihoods {
            assert!((row.iter().sum::<f64>() - 1.0).abs() < 1e-12);
            assert!(row.iter().all(|value| *value >= 0.0));
        }
        assert!(design.shannon_mutual_information().unwrap() > 0.1);
        assert!(design.sibson_alpha_mutual_information(0.5).unwrap() > 0.05);
    }

    #[test]
    fn finer_censoring_bins_do_not_lose_their_probability_mass() {
        let coarse = build_censored_discrete_design(&tracer_components(), &[0.5]).unwrap();
        let fine = build_censored_discrete_design(&tracer_components(), &[0.3, 0.5, 0.8]).unwrap();
        for row in coarse.likelihoods.iter().zip(&fine.likelihoods) {
            assert!((row.0.iter().sum::<f64>() - row.1.iter().sum::<f64>()).abs() < 1e-12);
        }
        assert!(fine.outcome_ids.len() > coarse.outcome_ids.len());
    }

    #[test]
    fn tail_interval_probability_uses_survival_difference_without_underflow() {
        let probability = normal_interval_probability(8.0, 8.1, 0.0, 1.0).unwrap();
        assert!(probability > 0.0);
        assert!(probability.is_finite());
    }

    #[test]
    fn rejects_invalid_thresholds_gaussians_and_model_missing_probabilities() {
        assert!(censored_event_probability(0.0, 0.0,
            &CensoredTracerObservation::BelowDetection { detection_limit: 0.5 }).is_err());
        assert!(normal_interval_probability(1.0, 1.0, 0.0, 1.0).is_err());
        assert!(build_censored_discrete_design(&tracer_components(), &[0.5, 0.4]).is_err());
        assert!(update_censored_model_posterior(
            &tracer_components(),
            &CensoredTracerObservation::MissingWithModelProbabilities { probabilities: vec![0.9] },
        ).is_err());
    }
}

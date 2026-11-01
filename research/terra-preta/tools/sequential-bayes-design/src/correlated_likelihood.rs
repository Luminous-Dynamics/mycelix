//! Exact bivariate-Gaussian likelihood and finite model-posterior update.
//!
//! This reusable continuous-observation component updates a finite candidate
//! posterior from one jointly observed pair. It does not yet retain
//! cross-action covariance through an adaptive history or connect the update
//! to the categorical acquisition policy. Correlations must be declared by a
//! simulator or estimated from suitable paired evidence, never inferred from
//! support overlap alone.

use std::collections::BTreeSet;

/// Exact bivariate Gaussian observation model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrelatedGaussianPair {
    pub mean: [f64; 2],
    pub standard_deviation: [f64; 2],
    pub correlation: f64,
}

impl CorrelatedGaussianPair {
    fn validate(&self) -> Result<(), String> {
        if !self.mean.iter().all(|v| v.is_finite()) {
            return Err("Gaussian pair means must be finite".into());
        }
        if !self.standard_deviation.iter().all(|v| v.is_finite() && *v > 0.0) {
            return Err("Gaussian pair standard deviations must be finite and positive".into());
        }
        if !self.correlation.is_finite() || self.correlation.abs() >= 1.0 {
            return Err("Gaussian pair correlation must lie strictly between -1 and 1".into());
        }
        Ok(())
    }

    /// Joint log density. Numerically unrepresentable far-tail likelihoods
    /// return negative infinity; invalid inputs or covariance return errors.
    pub fn log_density(&self, observation: [f64; 2]) -> Result<f64, String> {
        self.validate()?;
        if !observation.iter().all(|v| v.is_finite()) {
            return Err("Gaussian pair observations must be finite".into());
        }
        let z0 = (observation[0] - self.mean[0]) / self.standard_deviation[0];
        let z1 = (observation[1] - self.mean[1]) / self.standard_deviation[1];
        let rho = self.correlation;
        let one_minus_rho_squared = 1.0 - rho * rho;

        // Conditional-Gaussian factorization avoids cancellation in the
        // conventional z0² - 2ρz0z1 + z1² quadratic numerator.
        let conditional_residual = z0 - rho * z1;
        let quadratic = z1 * z1
            + conditional_residual * conditional_residual / one_minus_rho_squared;
        let log_normalizer = -std::f64::consts::TAU.ln()
            - self.standard_deviation[0].ln()
            - self.standard_deviation[1].ln()
            - 0.5 * one_minus_rho_squared.ln();

        if !log_normalizer.is_finite() {
            return Err("Gaussian pair log normalizer is not finite".into());
        }
        if quadratic == f64::INFINITY {
            return Ok(f64::NEG_INFINITY);
        }
        if !quadratic.is_finite() || quadratic < 0.0 {
            return Err("Gaussian pair quadratic form is invalid".into());
        }
        let result = log_normalizer - 0.5 * quadratic;
        if result.is_nan() || result == f64::INFINITY {
            return Err("Gaussian pair log density is invalid".into());
        }
        Ok(result)
    }
}

/// A candidate model for the jointly observed pair.
#[derive(Clone, Debug, PartialEq)]
pub struct GaussianModel {
    pub model_id: String,
    /// Non-negative prior mass. Masses are normalized internally.
    pub prior_weight: f64,
    pub observation_model: CorrelatedGaussianPair,
}

/// Finite model posterior and the joint log evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct GaussianPosterior {
    pub model_ids: Vec<String>,
    pub weights: Vec<f64>,
    pub log_evidence: f64,
}

/// Apply a joint continuous-observation Bayesian update over candidate models.
///
/// This is a joint update, not two independent updates. At zero correlation
/// each candidate's joint density factorizes into its two marginal densities.
pub fn update_model_posterior(
    models: &[GaussianModel],
    observation: [f64; 2],
) -> Result<GaussianPosterior, String> {
    if models.is_empty() {
        return Err("at least one Gaussian candidate model is required".into());
    }
    if !observation.iter().all(|v| v.is_finite()) {
        return Err("observations must be finite".into());
    }

    let mut ids = BTreeSet::new();
    let mut prior_total = 0.0;
    for model in models {
        if model.model_id.trim().is_empty() || !ids.insert(model.model_id.as_str()) {
            return Err("model identifiers must be non-empty and unique".into());
        }
        if !model.prior_weight.is_finite() || model.prior_weight < 0.0 {
            return Err(format!("invalid prior weight for {:?}", model.model_id));
        }
        model.observation_model.validate()?;
        prior_total += model.prior_weight;
    }
    if !prior_total.is_finite() || prior_total <= 0.0 {
        return Err("candidate prior weights must have finite positive sum".into());
    }

    let log_joint = models.iter().map(|model| {
        if model.prior_weight == 0.0 {
            Ok(f64::NEG_INFINITY)
        } else {
            Ok((model.prior_weight / prior_total).ln()
                + model.observation_model.log_density(observation)?)
        }
    }).collect::<Result<Vec<_>, String>>()?;

    let maximum = log_joint.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !maximum.is_finite() {
        return Err("all candidate models assign numerically zero likelihood".into());
    }
    let scaled_sum: f64 = log_joint.iter().map(|v| (v - maximum).exp()).sum();
    if !scaled_sum.is_finite() || scaled_sum <= 0.0 {
        return Err("posterior normalization failed".into());
    }
    let log_evidence = maximum + scaled_sum.ln();
    if !log_evidence.is_finite() {
        return Err("log evidence is not finite".into());
    }
    let mut weights: Vec<f64> = log_joint.iter()
        .map(|v| (v - log_evidence).exp()).collect();
    let total: f64 = weights.iter().sum();
    if !total.is_finite() || total <= 0.0 {
        return Err("posterior weights have no finite positive mass".into());
    }
    for weight in &mut weights {
        *weight /= total;
    }

    Ok(GaussianPosterior {
        model_ids: models.iter().map(|m| m.model_id.clone()).collect(),
        weights,
        log_evidence,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(id: &str, prior: f64, rho: f64) -> GaussianModel {
        GaussianModel {
            model_id: id.into(),
            prior_weight: prior,
            observation_model: CorrelatedGaussianPair {
                mean: [0.0, 0.0],
                standard_deviation: [1.0, 1.0],
                correlation: rho,
            },
        }
    }

    fn normal_log_density(x: f64, mean: f64, sd: f64) -> f64 {
        let z = (x - mean) / sd;
        -0.5 * std::f64::consts::TAU.ln() - sd.ln() - 0.5 * z * z
    }

    #[test]
    fn zero_correlation_matches_product_of_univariate_densities() {
        let pair = CorrelatedGaussianPair {
            mean: [0.3, -1.2],
            standard_deviation: [0.7, 1.4],
            correlation: 0.0,
        };
        let y = [0.8, -0.4];
        let expected = normal_log_density(y[0], pair.mean[0], pair.standard_deviation[0])
            + normal_log_density(y[1], pair.mean[1], pair.standard_deviation[1]);
        assert!((pair.log_density(y).unwrap() - expected).abs() < 1e-12);
    }

    #[test]
    fn joint_density_factorizes_into_marginal_and_conditional_density() {
        let pair = CorrelatedGaussianPair {
            mean: [0.4, -0.2],
            standard_deviation: [1.1, 0.6],
            correlation: 0.65,
        };
        let y = [1.2, 0.7];
        let marginal = normal_log_density(y[0], pair.mean[0], pair.standard_deviation[0]);
        let conditional_mean = pair.mean[1] + pair.correlation
            * pair.standard_deviation[1] / pair.standard_deviation[0]
            * (y[0] - pair.mean[0]);
        let conditional_sd = pair.standard_deviation[1]
            * (1.0 - pair.correlation * pair.correlation).sqrt();
        let conditional = normal_log_density(y[1], conditional_mean, conditional_sd);
        assert!((pair.log_density(y).unwrap() - marginal - conditional).abs() < 1e-12);
    }

    #[test]
    fn correlated_pair_discriminates_models_with_identical_marginals() {
        let models = vec![candidate("concordant", 0.5, 0.8), candidate("discordant", 0.5, -0.8)];
        let posterior = update_model_posterior(&models, [1.0, 1.0]).unwrap();
        assert!(posterior.weights[0] > 0.98);
        assert!(posterior.weights[1] < 0.02);
        assert!((posterior.weights.iter().sum::<f64>() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn independent_equal_likelihood_control_preserves_equal_prior() {
        let models = vec![candidate("a", 0.5, 0.0), candidate("b", 0.5, 0.0)];
        let posterior = update_model_posterior(&models, [1.0, 1.0]).unwrap();
        assert!((posterior.weights[0] - 0.5).abs() < 1e-12);
        assert!((posterior.weights[1] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn equal_likelihoods_preserve_unequal_prior_odds() {
        let models = vec![candidate("a", 0.8, 0.0), candidate("b", 0.2, 0.0)];
        let posterior = update_model_posterior(&models, [0.4, -0.2]).unwrap();
        assert!((posterior.weights[0] - 0.8).abs() < 1e-12);
        assert!((posterior.weights[1] - 0.2).abs() < 1e-12);
    }

    #[test]
    fn channel_swap_preserves_density() {
        let pair = CorrelatedGaussianPair {
            mean: [0.4, -0.2],
            standard_deviation: [1.1, 0.6],
            correlation: 0.65,
        };
        let swapped = CorrelatedGaussianPair {
            mean: [pair.mean[1], pair.mean[0]],
            standard_deviation: [pair.standard_deviation[1], pair.standard_deviation[0]],
            correlation: pair.correlation,
        };
        assert!((pair.log_density([1.2, 0.7]).unwrap()
            - swapped.log_density([0.7, 1.2]).unwrap()).abs() < 1e-12);
    }

    #[test]
    fn zero_prior_mass_remains_zero() {
        let models = vec![candidate("excluded", 0.0, -0.8), candidate("included", 1.0, 0.8)];
        let posterior = update_model_posterior(&models, [1.0, 1.0]).unwrap();
        assert_eq!(posterior.weights[0], 0.0);
        assert_eq!(posterior.weights[1], 1.0);
    }

    #[test]
    fn log_evidence_is_prior_weighted_joint_density() {
        let models = vec![candidate("positive", 0.75, 0.4), candidate("negative", 0.25, -0.2)];
        let y = [0.2, 0.7];
        let d0 = models[0].observation_model.log_density(y).unwrap().exp();
        let d1 = models[1].observation_model.log_density(y).unwrap().exp();
        let expected = (0.75 * d0 + 0.25 * d1).ln();
        let posterior = update_model_posterior(&models, y).unwrap();
        assert!((posterior.log_evidence - expected).abs() < 1e-12);
    }

    #[test]
    fn rejects_empty_duplicate_and_invalid_prior_inputs() {
        assert!(update_model_posterior(&[], [0.0, 0.0]).is_err());
        assert!(update_model_posterior(
            &[candidate("same", 0.5, 0.0), candidate("same", 0.5, 0.0)], [0.0, 0.0]
        ).is_err());
        assert!(update_model_posterior(&[candidate("negative", -0.1, 0.0)], [0.0, 0.0]).is_err());
        assert!(update_model_posterior(&[candidate("zero", 0.0, 0.0)], [0.0, 0.0]).is_err());
    }

    #[test]
    fn rejects_nonfinite_observations_and_singular_covariance() {
        assert!(update_model_posterior(&[candidate("model", 1.0, 0.0)], [f64::NAN, 0.0]).is_err());
        let invalid = candidate("invalid", 1.0, 1.0);
        assert!(update_model_posterior(&[invalid], [0.0, 0.0]).is_err());
    }

    #[test]
    fn all_numerically_impossible_candidate_likelihoods_fail_closed() {
        let models = vec![candidate("a", 0.5, 0.0), candidate("b", 0.5, 0.8)];
        assert!(update_model_posterior(&models, [f64::MAX, f64::MAX]).is_err());
    }
}

//! Exact bivariate-Gaussian reference likelihood for paired measurements.
//!
//! This mathematical primitive is not yet connected to the categorical
//! sequential selector. Correlation values must come from replicated evidence
//! or a declared simulator, never from support overlap alone.

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CorrelatedGaussianPair {
    pub(crate) mean: [f64; 2],
    pub(crate) standard_deviation: [f64; 2],
    pub(crate) correlation: f64,
}

impl CorrelatedGaussianPair {
    fn validate(&self) -> Result<(), String> {
        if !self.mean.iter().all(|value| value.is_finite()) {
            return Err("Gaussian pair means must be finite".into());
        }
        if !self.standard_deviation.iter().all(|value| {
            value.is_finite() && *value > 0.0
        }) {
            return Err("Gaussian pair standard deviations must be finite and positive".into());
        }
        if !self.correlation.is_finite() || self.correlation.abs() >= 1.0 {
            return Err("Gaussian pair correlation must lie strictly between -1 and 1".into());
        }
        Ok(())
    }

    pub(crate) fn log_density(&self, observation: [f64; 2]) -> Result<f64, String> {
        self.validate()?;
        if !observation.iter().all(|value| value.is_finite()) {
            return Err("Gaussian pair observations must be finite".into());
        }

        let z0 = (observation[0] - self.mean[0]) / self.standard_deviation[0];
        let z1 = (observation[1] - self.mean[1]) / self.standard_deviation[1];
        let correlation = self.correlation;
        let one_minus_rho_squared = 1.0 - correlation * correlation;
        let quadratic = (
            z0 * z0 - 2.0 * correlation * z0 * z1 + z1 * z1
        ) / one_minus_rho_squared;
        let log_normalizer = -std::f64::consts::TAU.ln()
            - self.standard_deviation[0].ln()
            - self.standard_deviation[1].ln()
            - 0.5 * one_minus_rho_squared.ln();
        Ok(log_normalizer - 0.5 * quadratic)
    }
}

fn univariate_normal_log_density(
    observation: f64,
    mean: f64,
    standard_deviation: f64,
) -> f64 {
    let standardized = (observation - mean) / standard_deviation;
    -0.5 * std::f64::consts::TAU.ln()
        - standard_deviation.ln()
        - 0.5 * standardized * standardized
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_correlation_matches_product_of_univariate_densities() {
        let pair = CorrelatedGaussianPair {
            mean: [0.3, -1.2],
            standard_deviation: [0.7, 1.4],
            correlation: 0.0,
        };
        let observation = [0.8, -0.4];
        let joint = pair.log_density(observation).expect("valid Gaussian pair");
        let independent = univariate_normal_log_density(
            observation[0], pair.mean[0], pair.standard_deviation[0],
        ) + univariate_normal_log_density(
            observation[1], pair.mean[1], pair.standard_deviation[1],
        );
        assert!((joint - independent).abs() < 1e-12);
    }

    #[test]
    fn correlation_changes_concordant_and_discordant_joint_likelihoods() {
        let independent = CorrelatedGaussianPair {
            mean: [0.0, 0.0],
            standard_deviation: [1.0, 1.0],
            correlation: 0.0,
        };
        let positive = CorrelatedGaussianPair {
            correlation: 0.8,
            ..independent
        };
        let negative = CorrelatedGaussianPair {
            correlation: -0.8,
            ..independent
        };
        let concordant = [1.0, 1.0];
        let discordant = [1.0, -1.0];

        assert!(
            positive.log_density(concordant).unwrap()
                > independent.log_density(concordant).unwrap()
        );
        assert!(
            negative.log_density(discordant).unwrap()
                > independent.log_density(discordant).unwrap()
        );
        assert!(
            positive.log_density(discordant).unwrap()
                < independent.log_density(discordant).unwrap()
        );
    }

    #[test]
    fn joint_density_factorizes_into_marginal_and_conditional_density() {
        let pair = CorrelatedGaussianPair {
            mean: [0.4, -0.2],
            standard_deviation: [1.1, 0.6],
            correlation: 0.65,
        };
        let observation = [1.2, 0.7];
        let joint = pair.log_density(observation).unwrap();
        let marginal = univariate_normal_log_density(
            observation[0], pair.mean[0], pair.standard_deviation[0],
        );
        let conditional_mean = pair.mean[1]
            + pair.correlation * pair.standard_deviation[1]
                / pair.standard_deviation[0] * (observation[0] - pair.mean[0]);
        let conditional_standard_deviation = pair.standard_deviation[1]
            * (1.0 - pair.correlation * pair.correlation).sqrt();
        let conditional = univariate_normal_log_density(
            observation[1], conditional_mean, conditional_standard_deviation,
        );
        assert!((joint - marginal - conditional).abs() < 1e-12);
    }

    #[test]
    fn reject_singular_or_invalid_covariance_parameters() {
        for correlation in [-1.0, 1.0, f64::NAN] {
            let pair = CorrelatedGaussianPair {
                mean: [0.0, 0.0],
                standard_deviation: [1.0, 1.0],
                correlation,
            };
            assert!(pair.log_density([0.0, 0.0]).is_err());
        }
        for standard_deviation in [0.0, -1.0, f64::INFINITY] {
            let pair = CorrelatedGaussianPair {
                mean: [0.0, 0.0],
                standard_deviation: [standard_deviation, 1.0],
                correlation: 0.2,
            };
            assert!(pair.log_density([0.0, 0.0]).is_err());
        }
    }

    #[test]
    fn reject_nonfinite_observation_and_mean() {
        let pair = CorrelatedGaussianPair {
            mean: [0.0, f64::INFINITY],
            standard_deviation: [1.0, 1.0],
            correlation: 0.2,
        };
        assert!(pair.log_density([0.0, 0.0]).is_err());
        let valid = CorrelatedGaussianPair {
            mean: [0.0, 0.0],
            ..pair
        };
        assert!(valid.log_density([f64::NAN, 0.0]).is_err());
    }
}

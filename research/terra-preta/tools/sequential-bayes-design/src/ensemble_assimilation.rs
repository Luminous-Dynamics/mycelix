//! Finite-ensemble Gaussian-moment assimilation reference.
//!
//! This module samples an augmented state/parameter ensemble from a Gaussian
//! prior and applies the deterministic linear EnKF moment update using the
//! ensemble mean/covariance. It is compared against the exact linear-Gaussian
//! oracle in state_parameter_assimilation.rs.
//!
//! It is not a perturbed-observation EnKF, square-root EnKF, particle filter,
//! nonlinear Richards solver or empirical soil assimilation result.

use crate::multivariate_gaussian::{cholesky_factor, MultivariateGaussian};
use crate::state_parameter_assimilation::{
    LinearGaussianObservationOperator, LinearGaussianStateParameterAssimilator,
    StateParameterGaussianPrior, StateParameterPosterior,
};

#[derive(Clone, Debug, PartialEq)]
pub struct GaussianEnsemble {
    pub variable_metadata: Vec<crate::state_parameter_assimilation::AssimilationVariable>,
    pub members: Vec<Vec<f64>>,
    pub seed: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EnsembleKalmanApproximation {
    pub posterior: StateParameterPosterior,
    pub ensemble_sample_count: usize,
    pub seed: u64,
    pub empirical_prior_mean: Vec<f64>,
    pub empirical_prior_covariance: Vec<Vec<f64>>,
}

#[derive(Clone, Debug)]
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        value ^ (value >> 31)
    }

    fn uniform_open01(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / 9_007_199_254_740_992.0
    }

    fn standard_normal_pair(&mut self) -> (f64, f64) {
        let radius = (-2.0 * self.uniform_open01().ln()).sqrt();
        let angle = std::f64::consts::TAU * self.uniform_open01();
        (radius * angle.cos(), radius * angle.sin())
    }

    fn standard_normals(&mut self, dimension: usize) -> Vec<f64> {
        let mut values = Vec::with_capacity(dimension);
        while values.len() < dimension {
            let (first, second) = self.standard_normal_pair();
            values.push(first);
            if values.len() < dimension {
                values.push(second);
            }
        }
        values
    }
}

impl GaussianEnsemble {
    /// Draw a reproducible finite ensemble from the declared Gaussian prior.
    pub fn sample(
        prior: &StateParameterGaussianPrior,
        member_count: usize,
        seed: u64,
    ) -> Result<Self, String> {
        if member_count < 2 {
            return Err("at least two ensemble members are required".into());
        }
        if prior.variables.is_empty()
            || prior.variables.len() != prior.distribution.mean.len()
        {
            return Err("prior variables must align with a non-empty Gaussian vector".into());
        }
        let mut ids = std::collections::BTreeSet::new();
        for variable in &prior.variables {
            if variable.variable_id.trim().is_empty()
                || variable.unit.trim().is_empty()
                || !ids.insert(variable.variable_id.as_str())
            {
                return Err("ensemble variable IDs and units must be non-empty and IDs unique".into());
            }
        }
        prior.distribution.validate()?;
        let (lower, _) = cholesky_factor(&prior.distribution.covariance)?;
        let mut rng = SplitMix64::new(seed);
        let mut members = Vec::with_capacity(member_count);
        for _ in 0..member_count {
            let normal = rng.standard_normals(prior.distribution.mean.len());
            let mut member = Vec::with_capacity(prior.distribution.mean.len());
            for (row, mean) in prior.distribution.mean.iter().enumerate() {
                let perturbation: f64 = lower[row].iter()
                    .take(row + 1)
                    .zip(&normal)
                    .map(|(factor, draw)| factor * draw)
                    .sum();
                member.push(mean + perturbation);
            }
            if member.iter().any(|value| !value.is_finite()) {
                return Err("ensemble sampling produced a non-finite member".into());
            }
            members.push(member);
        }
        Ok(Self {
            variable_metadata: prior.variables.clone(),
            members,
            seed,
        })
    }

    pub fn member_count(&self) -> usize {
        self.members.len()
    }

    pub fn dimension(&self) -> usize {
        self.variable_metadata.len()
    }

    /// Compute unbiased sample mean and covariance of the finite ensemble.
    pub fn sample_moments(&self) -> Result<MultivariateGaussian, String> {
        if self.members.len() < 2 {
            return Err("at least two ensemble members are required for covariance".into());
        }
        let dimension = self.dimension();
        if dimension == 0 || self.members.iter().any(|member| member.len() != dimension) {
            return Err("ensemble member dimensions do not match variable metadata".into());
        }
        if self.members.iter().flatten().any(|value| !value.is_finite()) {
            return Err("ensemble members must be finite".into());
        }

        let mut mean = vec![0.0; dimension];
        for member in &self.members {
            for (total, value) in mean.iter_mut().zip(member) {
                *total += value;
            }
        }
        for value in &mut mean {
            *value /= self.members.len() as f64;
        }

        let mut covariance = vec![vec![0.0; dimension]; dimension];
        for member in &self.members {
            let centered: Vec<f64> = member.iter().zip(&mean)
                .map(|(value, average)| value - average)
                .collect();
            for (row, row_value) in centered.iter().enumerate() {
                for (column, column_value) in centered.iter().enumerate() {
                    covariance[row][column] += row_value * column_value;
                }
            }
        }
        let denominator = (self.members.len() - 1) as f64;
        for row in &mut covariance {
            for value in row {
                *value /= denominator;
            }
        }

        let distribution = MultivariateGaussian { mean, covariance };
        distribution.validate()?;
        Ok(distribution)
    }

    /// Apply the deterministic ensemble Kalman moment update.
    ///
    /// The empirical ensemble mean/covariance act as the approximate Gaussian
    /// prior; the exact conditional Gaussian operator is then applied to those
    /// moments. The posterior is exact for the empirical moments but differs
    /// from the exact prior posterior because of finite-ensemble sampling.
    pub fn analyze(
        &self,
        observation_operator: &LinearGaussianObservationOperator,
        observations: &[f64],
    ) -> Result<EnsembleKalmanApproximation, String> {
        let empirical_prior = StateParameterGaussianPrior {
            variables: self.variable_metadata.clone(),
            distribution: self.sample_moments()?,
        };
        let assimilator = LinearGaussianStateParameterAssimilator::new(
            empirical_prior.clone(),
            observation_operator.clone(),
        )?;
        let posterior = assimilator.update(observations)?;
        Ok(EnsembleKalmanApproximation {
            posterior,
            ensemble_sample_count: self.member_count(),
            seed: self.seed,
            empirical_prior_mean: empirical_prior.distribution.mean,
            empirical_prior_covariance: empirical_prior.distribution.covariance,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state_parameter_assimilation::{
        AssimilationVariable, AssimilationVariableRole,
    };

    fn prior() -> StateParameterGaussianPrior {
        StateParameterGaussianPrior {
            variables: vec![
                AssimilationVariable {
                    variable_id: "water_state".into(),
                    role: AssimilationVariableRole::PhysicalState,
                    unit: "1".into(),
                },
                AssimilationVariable {
                    variable_id: "exchange_rate_parameter".into(),
                    role: AssimilationVariableRole::ExchangeParameter,
                    unit: "1/s".into(),
                },
            ],
            distribution: MultivariateGaussian {
                mean: vec![0.0, 0.0],
                covariance: vec![vec![1.0, 0.2], vec![0.2, 0.8]],
            },
        }
    }

    fn operator() -> LinearGaussianObservationOperator {
        LinearGaussianObservationOperator {
            observation_ids: vec!["surface_moisture".into()],
            matrix: vec![vec![1.0, 1.5]],
            noise_covariance: vec![vec![0.25]],
        }
    }

    #[test]
    fn seeded_prior_ensemble_is_reproducible() {
        let left = GaussianEnsemble::sample(&prior(), 128, 20261101).unwrap();
        let right = GaussianEnsemble::sample(&prior(), 128, 20261101).unwrap();
        assert_eq!(left, right);
    }

    #[test]
    fn ensemble_moments_converge_to_declared_prior_at_large_n() {
        let ensemble = GaussianEnsemble::sample(&prior(), 20_000, 42).unwrap();
        let moments = ensemble.sample_moments().unwrap();
        for (actual, expected) in moments.mean.iter().zip(&prior().distribution.mean) {
            assert!((actual - expected).abs() < 0.025);
        }
        for (actual_row, expected_row) in moments.covariance.iter()
            .zip(&prior().distribution.covariance)
        {
            for (actual, expected) in actual_row.iter().zip(expected_row) {
                assert!((actual - expected).abs() < 0.04);
            }
        }
    }

    #[test]
    fn large_ensemble_kalman_analysis_approaches_exact_linear_gaussian_posterior() {
        let prior = prior();
        let observation_operator = operator();
        let exact = LinearGaussianStateParameterAssimilator::new(
            prior.clone(),
            observation_operator.clone(),
        ).unwrap().update(&[0.7]).unwrap();

        let ensemble = GaussianEnsemble::sample(&prior, 30_000, 20261101).unwrap();
        let approximation = ensemble.analyze(&observation_operator, &[0.7]).unwrap();

        for (actual, expected) in approximation.posterior.distribution.mean.iter()
            .zip(&exact.distribution.mean)
        {
            assert!((actual - expected).abs() < 0.03);
        }
        for (actual_row, expected_row) in approximation.posterior.distribution.covariance.iter()
            .zip(&exact.distribution.covariance)
        {
            for (actual, expected) in actual_row.iter().zip(expected_row) {
                assert!((actual - expected).abs() < 0.04);
            }
        }
    }

    #[test]
    fn finite_ensemble_analysis_is_reproducible_and_reports_sampling_provenance() {
        let ensemble = GaussianEnsemble::sample(&prior(), 512, 17).unwrap();
        let left = ensemble.analyze(&operator(), &[0.3]).unwrap();
        let right = ensemble.analyze(&operator(), &[0.3]).unwrap();
        assert_eq!(left, right);
        assert_eq!(left.ensemble_sample_count, 512);
        assert_eq!(left.seed, 17);
    }

    #[test]
    fn ensemble_update_preserves_augmented_state_parameter_covariance() {
        let ensemble = GaussianEnsemble::sample(&prior(), 4096, 99).unwrap();
        let posterior = ensemble.analyze(&operator(), &[0.5]).unwrap();
        assert_ne!(
            posterior.posterior.distribution.covariance[0][1],
            0.0
        );
    }

    #[test]
    fn rejects_too_small_ensemble_and_invalid_metadata() {
        assert!(GaussianEnsemble::sample(&prior(), 1, 1).is_err());
        let mut bad_prior = prior();
        bad_prior.variables[1].variable_id = bad_prior.variables[0].variable_id.clone();
        assert!(GaussianEnsemble::sample(&bad_prior, 10, 1).is_err());
    }
}

//! Exact linear-Gaussian joint state/parameter assimilation reference.
//!
//! This module provides an analytical Bayesian oracle for jointly updating
//! augmented physical states, hydraulic/exchange parameters and optional
//! nuisance variables from linear-Gaussian observations. It is intended as a
//! reference for EnKF, particle-filter and reduced-adjoint comparisons.
//!
//! It does not implement Richards flow, nonlinear adjoints, particle filters,
//! empirical observation operators or physical data assimilation.

use crate::multivariate_gaussian::MultivariateGaussian;
use std::collections::BTreeSet;

/// Semantic role of one augmented latent variable.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssimilationVariableRole {
    PhysicalState,
    HydraulicParameter,
    ExchangeParameter,
    MeasurementBias,
    OtherNuisance,
}

/// Stable identity and role of one augmented state/parameter coordinate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssimilationVariable {
    pub variable_id: String,
    pub role: AssimilationVariableRole,
    pub unit: String,
}

/// Gaussian prior over the complete augmented state and parameter vector.
#[derive(Clone, Debug, PartialEq)]
pub struct StateParameterGaussianPrior {
    pub variables: Vec<AssimilationVariable>,
    pub distribution: MultivariateGaussian,
}

/// Linear observation operator and observation-noise covariance.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearGaussianObservationOperator {
    pub observation_ids: Vec<String>,
    /// Row-major H in y = H z + epsilon.
    pub matrix: Vec<Vec<f64>>,
    pub noise_covariance: Vec<Vec<f64>>,
}

/// Posterior over the entire augmented physical state/parameter vector.
#[derive(Clone, Debug, PartialEq)]
pub struct StateParameterPosterior {
    pub variables: Vec<AssimilationVariable>,
    pub distribution: MultivariateGaussian,
    pub log_marginal_likelihood: f64,
    pub innovation: Vec<f64>,
    pub predictive_observation: MultivariateGaussian,
}

/// Exact linear-Gaussian state/parameter assimilation oracle.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearGaussianStateParameterAssimilator {
    prior: StateParameterGaussianPrior,
    observation_operator: LinearGaussianObservationOperator,
}

fn validate_unique_nonempty_ids<'a>(
    values: impl Iterator<Item = &'a str>,
    label: &str,
) -> Result<(), String> {
    let mut unique = BTreeSet::new();
    for value in values {
        if value.trim().is_empty() || !unique.insert(value) {
            return Err(format!("{label} IDs must be non-empty and unique"));
        }
    }
    Ok(())
}

fn validate_finite_matrix(matrix: &[Vec<f64>], label: &str) -> Result<(), String> {
    if matrix.is_empty() || matrix.iter().any(|row| row.is_empty()) {
        return Err(format!("{label} must be non-empty"));
    }
    let width = matrix[0].len();
    if matrix.iter().any(|row| row.len() != width) {
        return Err(format!("{label} rows must have consistent lengths"));
    }
    if matrix.iter().flatten().any(|value| !value.is_finite()) {
        return Err(format!("{label} entries must be finite"));
    }
    Ok(())
}

fn transpose(matrix: &[Vec<f64>]) -> Vec<Vec<f64>> {
    (0..matrix[0].len()).map(|column| {
        matrix.iter().map(|row| row[column]).collect()
    }).collect()
}

fn multiply(left: &[Vec<f64>], right: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, String> {
    validate_finite_matrix(left, "left matrix")?;
    validate_finite_matrix(right, "right matrix")?;
    if left[0].len() != right.len() {
        return Err("matrix multiplication dimensions do not align".into());
    }
    let right_transpose = transpose(right);
    Ok(left.iter().map(|row| {
        right_transpose.iter().map(|column| {
            row.iter().zip(column).map(|(a, b)| a * b).sum()
        }).collect()
    }).collect())
}

fn multiply_vector(matrix: &[Vec<f64>], vector: &[f64]) -> Result<Vec<f64>, String> {
    validate_finite_matrix(matrix, "matrix")?;
    if matrix[0].len() != vector.len() {
        return Err("matrix/vector dimensions do not align".into());
    }
    if vector.iter().any(|value| !value.is_finite()) {
        return Err("vector entries must be finite".into());
    }
    Ok(matrix.iter().map(|row| {
        row.iter().zip(vector).map(|(a, b)| a * b).sum()
    }).collect())
}

fn add_matrices(left: &[Vec<f64>], right: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, String> {
    if left.len() != right.len()
        || left.iter().zip(right).any(|(a, b)| a.len() != b.len())
    {
        return Err("matrix addition dimensions do not align".into());
    }
    Ok(left.iter().zip(right).map(|(left_row, right_row)| {
        left_row.iter().zip(right_row).map(|(a, b)| a + b).collect()
    }).collect())
}

fn subtract_vectors(left: &[f64], right: &[f64]) -> Result<Vec<f64>, String> {
    if left.len() != right.len() {
        return Err("vector subtraction dimensions do not align".into());
    }
    Ok(left.iter().zip(right).map(|(a, b)| a - b).collect())
}

fn validate_operator(
    variables: &[AssimilationVariable],
    distribution: &MultivariateGaussian,
    operator: &LinearGaussianObservationOperator,
) -> Result<(), String> {
    if variables.is_empty() {
        return Err("at least one augmented state/parameter variable is required".into());
    }
    validate_unique_nonempty_ids(
        variables.iter().map(|variable| variable.variable_id.as_str()),
        "augmented variable",
    )?;
    if variables.iter().any(|variable| variable.unit.trim().is_empty()) {
        return Err("every augmented variable must declare a unit".into());
    }
    if variables.len() != distribution.mean.len() {
        return Err("variable metadata must align with the prior Gaussian dimension".into());
    }
    distribution.validate()?;

    validate_unique_nonempty_ids(
        operator.observation_ids.iter().map(String::as_str),
        "observation",
    )?;
    validate_finite_matrix(&operator.matrix, "observation matrix")?;
    if operator.matrix.len() != operator.observation_ids.len()
        || operator.matrix[0].len() != variables.len()
    {
        return Err("observation matrix dimensions must match observation and latent variables".into());
    }
    if operator.noise_covariance.len() != operator.observation_ids.len()
        || operator.noise_covariance.iter().any(|row| row.len() != operator.observation_ids.len())
    {
        return Err("observation-noise covariance dimensions do not match observation count".into());
    }
    MultivariateGaussian {
        mean: vec![0.0; operator.observation_ids.len()],
        covariance: operator.noise_covariance.clone(),
    }.validate()?;
    Ok(())
}

impl LinearGaussianStateParameterAssimilator {
    pub fn new(
        prior: StateParameterGaussianPrior,
        observation_operator: LinearGaussianObservationOperator,
    ) -> Result<Self, String> {
        validate_operator(&prior.variables, &prior.distribution, &observation_operator)?;
        Ok(Self { prior, observation_operator })
    }

    pub fn prior(&self) -> &StateParameterGaussianPrior {
        &self.prior
    }

    pub fn observation_operator(&self) -> &LinearGaussianObservationOperator {
        &self.observation_operator
    }

    /// Predict the marginal distribution of the observation vector.
    pub fn predict_observation(&self) -> Result<MultivariateGaussian, String> {
        let h = &self.observation_operator.matrix;
        let prior_covariance = &self.prior.distribution.covariance;
        let mean = multiply_vector(h, &self.prior.distribution.mean)?;
        let hp = multiply(h, prior_covariance)?;
        let projected = multiply(&hp, &transpose(h))?;
        let covariance = add_matrices(
            &projected,
            &self.observation_operator.noise_covariance,
        )?;
        let predictive = MultivariateGaussian { mean, covariance };
        predictive.validate()?;
        Ok(predictive)
    }

    /// Assimilate one complete observation vector using the exact joint
    /// Gaussian conditional distribution. The posterior preserves
    /// state/parameter cross-covariance rather than updating them separately.
    pub fn update(&self, observations: &[f64]) -> Result<StateParameterPosterior, String> {
        if observations.len() != self.observation_operator.observation_ids.len() {
            return Err("observation vector length does not match observation operator".into());
        }
        if observations.iter().any(|value| !value.is_finite()) {
            return Err("observed values must be finite".into());
        }

        let predictive_observation = self.predict_observation()?;
        let innovation = subtract_vectors(
            observations,
            &predictive_observation.mean,
        )?;
        let log_marginal_likelihood = predictive_observation.log_density(observations)?;

        // Construct the joint Gaussian [z, y], then condition z on y.
        // Cov(z,y) = P H^T; Cov(y) = H P H^T + R.
        let h_transpose = transpose(&self.observation_operator.matrix);
        let cross_covariance = multiply(
            &self.prior.distribution.covariance,
            &h_transpose,
        )?;
        let state_dimension = self.prior.variables.len();
        let observation_dimension = observations.len();
        let mut joint_mean = self.prior.distribution.mean.clone();
        joint_mean.extend_from_slice(&predictive_observation.mean);

        let mut joint_covariance = vec![
            vec![0.0; state_dimension + observation_dimension];
            state_dimension + observation_dimension
        ];
        let (state_block, observation_block) =
            joint_covariance.split_at_mut(state_dimension);
        for (row, state_row) in state_block.iter_mut().enumerate() {
            for (column, covariance_value) in
                self.prior.distribution.covariance[row].iter().enumerate()
            {
                state_row[column] = *covariance_value;
            }
            for (column, cross_value) in cross_covariance[row].iter().enumerate() {
                state_row[state_dimension + column] = *cross_value;
                observation_block[column][row] = *cross_value;
            }
        }
        for (row, observation_row) in observation_block.iter_mut().enumerate() {
            for (column, covariance_value) in
                predictive_observation.covariance[row].iter().enumerate()
            {
                observation_row[state_dimension + column] = *covariance_value;
            }
        }

        let joint = MultivariateGaussian {
            mean: joint_mean,
            covariance: joint_covariance,
        };
        joint.validate()?;

        let observed_indices: Vec<usize> = (state_dimension
            ..state_dimension + observation_dimension).collect();
        let target_indices: Vec<usize> = (0..state_dimension).collect();
        let posterior_distribution = joint.conditional(
            &observed_indices,
            observations,
            &target_indices,
        )?;

        Ok(StateParameterPosterior {
            variables: self.prior.variables.clone(),
            distribution: posterior_distribution,
            log_marginal_likelihood,
            innovation,
            predictive_observation,
        })
    }
}

impl StateParameterPosterior {
    pub fn indices_for_role(&self, role: AssimilationVariableRole) -> Vec<usize> {
        self.variables.iter().enumerate()
            .filter_map(|(index, variable)| (variable.role == role).then_some(index))
            .collect()
    }

    pub fn marginal_for_role(
        &self,
        role: AssimilationVariableRole,
    ) -> Result<MultivariateGaussian, String> {
        let indices = self.indices_for_role(role);
        if indices.is_empty() {
            return Err(format!("posterior has no variables with role {role:?}"));
        }
        self.distribution.marginal(&indices)
    }

    pub fn covariance_between_roles(
        &self,
        left_role: AssimilationVariableRole,
        right_role: AssimilationVariableRole,
    ) -> Result<Vec<Vec<f64>>, String> {
        let left = self.indices_for_role(left_role);
        let right = self.indices_for_role(right_role);
        if left.is_empty() || right.is_empty() {
            return Err("both requested variable roles must be present".into());
        }
        Ok(left.iter().map(|row| {
            right.iter().map(|column| {
                self.distribution.covariance[*row][*column]
            }).collect()
        }).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn variable(id: &str, role: AssimilationVariableRole) -> AssimilationVariable {
        AssimilationVariable {
            variable_id: id.into(),
            role,
            unit: "1".into(),
        }
    }

    fn scalar_observation_model() -> LinearGaussianStateParameterAssimilator {
        LinearGaussianStateParameterAssimilator::new(
            StateParameterGaussianPrior {
                variables: vec![
                    variable("matrix_water_state", AssimilationVariableRole::PhysicalState),
                    variable("exchange_parameter", AssimilationVariableRole::ExchangeParameter),
                ],
                distribution: MultivariateGaussian {
                    mean: vec![0.0, 0.0],
                    covariance: vec![vec![1.0, 0.0], vec![0.0, 1.0]],
                },
            },
            LinearGaussianObservationOperator {
                observation_ids: vec!["near_surface_water_content".into()],
                matrix: vec![vec![1.0, 2.0]],
                noise_covariance: vec![vec![0.25]],
            },
        ).unwrap()
    }

    #[test]
    fn exact_update_matches_scalar_linear_gaussian_posterior() {
        let assimilator = scalar_observation_model();
        let posterior = assimilator.update(&[1.0]).unwrap();
        let denominator = 1.0 + 4.0 + 0.25;
        assert!((posterior.distribution.mean[0] - 1.0 / denominator).abs() < 1e-12);
        assert!((posterior.distribution.mean[1] - 2.0 / denominator).abs() < 1e-12);
        assert!((posterior.distribution.covariance[0][0] - (1.0 - 1.0 / denominator)).abs() < 1e-12);
        assert!((posterior.distribution.covariance[0][1] - (-2.0 / denominator)).abs() < 1e-12);
        assert!((posterior.distribution.covariance[1][1] - (1.0 - 4.0 / denominator)).abs() < 1e-12);
    }

    #[test]
    fn state_and_exchange_parameter_remain_correlated_after_joint_update() {
        let posterior = scalar_observation_model().update(&[1.0]).unwrap();
        let cross = posterior.covariance_between_roles(
            AssimilationVariableRole::PhysicalState,
            AssimilationVariableRole::ExchangeParameter,
        ).unwrap();
        assert!((cross[0][0] + 2.0 / 5.25).abs() < 1e-12);
    }

    #[test]
    fn observation_prediction_matches_h_p_ht_plus_r() {
        let predictive = scalar_observation_model().predict_observation().unwrap();
        assert!((predictive.mean[0]).abs() < 1e-12);
        assert!((predictive.covariance[0][0] - 5.25).abs() < 1e-12);
    }

    #[test]
    fn sequential_and_batch_updates_match_for_independent_observation_noise() {
        let prior_variables = vec![
            variable("state", AssimilationVariableRole::PhysicalState),
            variable("parameter", AssimilationVariableRole::HydraulicParameter),
        ];
        let prior = MultivariateGaussian {
            mean: vec![0.1, 0.4],
            covariance: vec![vec![1.0, 0.2], vec![0.2, 0.8]],
        };
        let batch = LinearGaussianStateParameterAssimilator::new(
            StateParameterGaussianPrior {
                variables: prior_variables.clone(),
                distribution: prior.clone(),
            },
            LinearGaussianObservationOperator {
                observation_ids: vec!["obs-a".into(), "obs-b".into()],
                matrix: vec![vec![1.0, 0.0], vec![0.5, 1.0]],
                noise_covariance: vec![vec![0.1, 0.0], vec![0.0, 0.2]],
            },
        ).unwrap().update(&[0.7, 0.9]).unwrap();

        let first = LinearGaussianStateParameterAssimilator::new(
            StateParameterGaussianPrior {
                variables: prior_variables.clone(),
                distribution: prior,
            },
            LinearGaussianObservationOperator {
                observation_ids: vec!["obs-a".into()],
                matrix: vec![vec![1.0, 0.0]],
                noise_covariance: vec![vec![0.1]],
            },
        ).unwrap().update(&[0.7]).unwrap();
        let second = LinearGaussianStateParameterAssimilator::new(
            StateParameterGaussianPrior {
                variables: prior_variables,
                distribution: first.distribution,
            },
            LinearGaussianObservationOperator {
                observation_ids: vec!["obs-b".into()],
                matrix: vec![vec![0.5, 1.0]],
                noise_covariance: vec![vec![0.2]],
            },
        ).unwrap().update(&[0.9]).unwrap();

        for (batch_mean, sequential_mean) in batch.distribution.mean.iter()
            .zip(&second.distribution.mean)
        {
            assert!((batch_mean - sequential_mean).abs() < 1e-10);
        }
        for (batch_row, sequential_row) in batch.distribution.covariance.iter()
            .zip(&second.distribution.covariance)
        {
            for (batch_value, sequential_value) in batch_row.iter().zip(sequential_row) {
                assert!((batch_value - sequential_value).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn exact_update_handles_correlated_observation_noise() {
        let assimilator = LinearGaussianStateParameterAssimilator::new(
            StateParameterGaussianPrior {
                variables: vec![
                    variable("state_a", AssimilationVariableRole::PhysicalState),
                    variable("state_b", AssimilationVariableRole::PhysicalState),
                ],
                distribution: MultivariateGaussian {
                    mean: vec![0.0, 0.0],
                    covariance: vec![vec![1.0, 0.2], vec![0.2, 1.0]],
                },
            },
            LinearGaussianObservationOperator {
                observation_ids: vec!["ct_feature".into(), "tracer_arrival".into()],
                matrix: vec![vec![1.0, 0.0], vec![0.0, 1.0]],
                noise_covariance: vec![vec![0.3, 0.1], vec![0.1, 0.4]],
            },
        ).unwrap();
        let posterior = assimilator.update(&[0.6, 0.8]).unwrap();
        assert!(posterior.distribution.validate().is_ok());
        assert!(posterior.log_marginal_likelihood.is_finite());
    }

    #[test]
    fn observation_ids_and_variable_roles_are_preserved() {
        let posterior = scalar_observation_model().update(&[0.2]).unwrap();
        assert_eq!(posterior.variables[0].variable_id, "matrix_water_state");
        assert_eq!(posterior.variables[1].role, AssimilationVariableRole::ExchangeParameter);
        assert_eq!(posterior.indices_for_role(AssimilationVariableRole::PhysicalState), vec![0]);
        assert_eq!(posterior.indices_for_role(AssimilationVariableRole::ExchangeParameter), vec![1]);
    }

    #[test]
    fn posterior_reports_innovation_and_marginal_likelihood() {
        let posterior = scalar_observation_model().update(&[1.0]).unwrap();
        assert_eq!(posterior.innovation, vec![1.0]);
        assert!(posterior.log_marginal_likelihood.is_finite());
        assert!((posterior.predictive_observation.covariance[0][0] - 5.25).abs() < 1e-12);
    }

    #[test]
    fn rejects_duplicate_ids_bad_dimensions_and_invalid_noise_covariance() {
        let duplicate_variables = StateParameterGaussianPrior {
            variables: vec![
                variable("same", AssimilationVariableRole::PhysicalState),
                variable("same", AssimilationVariableRole::HydraulicParameter),
            ],
            distribution: MultivariateGaussian {
                mean: vec![0.0, 0.0],
                covariance: vec![vec![1.0, 0.0], vec![0.0, 1.0]],
            },
        };
        let operator = LinearGaussianObservationOperator {
            observation_ids: vec!["obs".into()],
            matrix: vec![vec![1.0, 1.0]],
            noise_covariance: vec![vec![0.1]],
        };
        assert!(LinearGaussianStateParameterAssimilator::new(
            duplicate_variables, operator.clone(),
        ).is_err());

        let bad_noise = LinearGaussianObservationOperator {
            noise_covariance: vec![vec![0.0]],
            ..operator.clone()
        };
        assert!(LinearGaussianStateParameterAssimilator::new(
            StateParameterGaussianPrior {
                variables: vec![variable("state", AssimilationVariableRole::PhysicalState)],
                distribution: MultivariateGaussian { mean: vec![0.0], covariance: vec![vec![1.0]] },
            },
            bad_noise,
        ).is_err());
    }

    #[test]
    fn rejects_nonfinite_and_mismatched_observation_vectors() {
        let assimilator = scalar_observation_model();
        assert!(assimilator.update(&[]).is_err());
        assert!(assimilator.update(&[f64::NAN]).is_err());
    }
}

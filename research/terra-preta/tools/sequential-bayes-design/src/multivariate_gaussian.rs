//! Multivariate Gaussian likelihoods for correlated measurement histories.
//!
//! This module supports arbitrary finite observation vectors with a dense,
//! symmetric positive-definite covariance matrix. It can score a selected
//! subset of a jointly modelled history and derive the conditional Gaussian
//! law of unobserved measurements given observed ones. This is a mathematical
//! reference component; it is not yet wired into the adaptive policy loop,
//! and it does not estimate covariance from empirical support metadata.

/// Multivariate Gaussian distribution with an explicit covariance matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct MultivariateGaussian {
    pub mean: Vec<f64>,
    pub covariance: Vec<Vec<f64>>,
}

/// Finite model candidate with a multivariate observation distribution.
#[derive(Clone, Debug, PartialEq)]
pub struct MultivariateGaussianModel {
    pub model_id: String,
    /// Non-negative prior mass; weights are normalized by the update.
    pub prior_weight: f64,
    pub observation_model: MultivariateGaussian,
}

/// Posterior model weights after scoring a selected observation history.
#[derive(Clone, Debug, PartialEq)]
pub struct MultivariateGaussianPosterior {
    pub model_ids: Vec<String>,
    pub weights: Vec<f64>,
    pub log_evidence: f64,
}

fn cholesky_factor(matrix: &[Vec<f64>]) -> Result<(Vec<Vec<f64>>, f64), String> {
    let dimension = matrix.len();
    if dimension == 0 || matrix.iter().any(|row| row.len() != dimension) {
        return Err("covariance matrix must be non-empty and square".into());
    }
    if matrix.iter().flatten().any(|value| !value.is_finite()) {
        return Err("covariance matrix entries must be finite".into());
    }

    let scale = matrix.iter().flatten()
        .map(|value| value.abs())
        .fold(1.0_f64, f64::max);
    let symmetry_tolerance = 1e-12 * scale;
    for row in 0..dimension {
        for column in 0..row {
            if (matrix[row][column] - matrix[column][row]).abs() > symmetry_tolerance {
                return Err("covariance matrix must be symmetric".into());
            }
        }
    }

    let mut lower = vec![vec![0.0; dimension]; dimension];
    let mut log_determinant = 0.0;
    for row in 0..dimension {
        for column in 0..=row {
            // Average only values already verified symmetric within tolerance.
            let entry = 0.5 * (matrix[row][column] + matrix[column][row]);
            let correction: f64 = (0..column)
                .map(|k| lower[row][k] * lower[column][k])
                .sum();
            let value = entry - correction;
            if row == column {
                if !value.is_finite() || value <= 0.0 {
                    return Err("covariance matrix must be positive definite".into());
                }
                lower[row][column] = value.sqrt();
                log_determinant += 2.0 * lower[row][column].ln();
            } else {
                lower[row][column] = value / lower[column][column];
                if !lower[row][column].is_finite() {
                    return Err("covariance factorization is numerically unstable".into());
                }
            }
        }
    }
    if !log_determinant.is_finite() {
        return Err("covariance log determinant is not finite".into());
    }
    Ok((lower, log_determinant))
}

fn solve_cholesky(lower: &[Vec<f64>], right_hand_side: &[f64]) -> Result<Vec<f64>, String> {
    let dimension = lower.len();
    if right_hand_side.len() != dimension {
        return Err("linear-system dimension mismatch".into());
    }
    let mut forward = vec![0.0; dimension];
    for row in 0..dimension {
        let correction: f64 = (0..row)
            .map(|column| lower[row][column] * forward[column])
            .sum();
        forward[row] = (right_hand_side[row] - correction) / lower[row][row];
    }

    let mut solution = vec![0.0; dimension];
    for row in (0..dimension).rev() {
        let correction: f64 = (row + 1..dimension)
            .map(|column| lower[column][row] * solution[column])
            .sum();
        solution[row] = (forward[row] - correction) / lower[row][row];
    }
    if solution.iter().all(|value| value.is_finite()) {
        Ok(solution)
    } else {
        Err("covariance solve is numerically unstable".into())
    }
}

fn validate_indices(indices: &[usize], dimension: usize, label: &str) -> Result<(), String> {
    if indices.is_empty() {
        return Ok(());
    }
    let mut sorted = indices.to_vec();
    sorted.sort_unstable();
    sorted.dedup();
    if sorted.len() != indices.len() {
        return Err(format!("{label} indices must be unique"));
    }
    if indices.iter().any(|index| *index >= dimension) {
        return Err(format!("{label} index is outside the observation dimension"));
    }
    Ok(())
}

impl MultivariateGaussian {
    /// Validate dimensions, finite means and a symmetric positive-definite
    /// covariance matrix. No silent jitter or eigenvalue clipping is applied.
    pub fn validate(&self) -> Result<(), String> {
        if self.mean.is_empty() {
            return Err("Gaussian mean vector must be non-empty".into());
        }
        if self.mean.iter().any(|value| !value.is_finite()) {
            return Err("Gaussian mean entries must be finite".into());
        }
        if self.covariance.len() != self.mean.len() {
            return Err("covariance dimension must match mean dimension".into());
        }
        cholesky_factor(&self.covariance)?;
        Ok(())
    }

    /// Joint log density for the complete observation vector.
    pub fn log_density(&self, observation: &[f64]) -> Result<f64, String> {
        if observation.len() != self.mean.len() {
            return Err("observation dimension must match Gaussian mean".into());
        }
        if observation.iter().any(|value| !value.is_finite()) {
            return Err("observation entries must be finite".into());
        }
        let (lower, log_determinant) = cholesky_factor(&self.covariance)?;
        let residual: Vec<f64> = observation.iter()
            .zip(&self.mean)
            .map(|(observed, expected)| observed - expected)
            .collect();
        let whitened = solve_lower(&lower, &residual)?;
        let quadratic: f64 = whitened.iter().map(|value| value * value).sum();
        if quadratic == f64::INFINITY {
            return Ok(f64::NEG_INFINITY);
        }
        if !quadratic.is_finite() || quadratic < 0.0 {
            return Err("Gaussian quadratic form is invalid".into());
        }
        let dimension = self.mean.len() as f64;
        let result = -0.5
            * (dimension * std::f64::consts::TAU.ln() + log_determinant + quadratic);
        if result.is_nan() || result == f64::INFINITY {
            return Err("Gaussian log density is invalid".into());
        }
        Ok(result)
    }

    /// Joint log density for an indexed subset of a retained observation
    /// history. The subset covariance is the corresponding marginal block.
    pub fn log_density_subset(
        &self,
        observed_indices: &[usize],
        observations: &[f64],
    ) -> Result<f64, String> {
        if observed_indices.len() != observations.len() || observed_indices.is_empty() {
            return Err("observed indices and values must have equal non-zero length".into());
        }
        validate_indices(observed_indices, self.mean.len(), "observed")?;
        let marginal = self.marginal(observed_indices)?;
        marginal.log_density(observations)
    }

    /// Return a marginal distribution in the requested index order.
    pub fn marginal(&self, indices: &[usize]) -> Result<Self, String> {
        validate_indices(indices, self.mean.len(), "marginal")?;
        if indices.is_empty() {
            return Err("marginal index set must be non-empty".into());
        }
        let mean = indices.iter().map(|index| self.mean[*index]).collect();
        let covariance = indices.iter()
            .map(|row| indices.iter()
                .map(|column| self.covariance[*row][*column])
                .collect())
            .collect();
        let result = Self { mean, covariance };
        result.validate()?;
        Ok(result)
    }

    /// Conditional distribution of target indices given observed indices and
    /// values, using the exact Gaussian Schur-complement formulas.
    ///
    /// Observed and target indices must be distinct and disjoint. With an empty
    /// observed set this returns the target marginal.
    pub fn conditional(
        &self,
        observed_indices: &[usize],
        observed_values: &[f64],
        target_indices: &[usize],
    ) -> Result<Self, String> {
        self.validate()?;
        if observed_indices.len() != observed_values.len() {
            return Err("observed indices and values must have equal length".into());
        }
        if observed_values.iter().any(|value| !value.is_finite()) {
            return Err("observed values must be finite".into());
        }
        if target_indices.is_empty() {
            return Err("target index set must be non-empty".into());
        }
        validate_indices(observed_indices, self.mean.len(), "observed")?;
        validate_indices(target_indices, self.mean.len(), "target")?;
        for observed in observed_indices {
            if target_indices.contains(observed) {
                return Err("observed and target indices must be disjoint".into());
            }
        }
        if observed_indices.is_empty() {
            return self.marginal(target_indices);
        }

        let observed_covariance: Vec<Vec<f64>> = observed_indices.iter()
            .map(|row| observed_indices.iter()
                .map(|column| self.covariance[*row][*column])
                .collect())
            .collect();
        let (lower, _) = cholesky_factor(&observed_covariance)?;
        let residual: Vec<f64> = observed_indices.iter()
            .zip(observed_values)
            .map(|(index, value)| value - self.mean[*index])
            .collect();
        let alpha = solve_cholesky(&lower, &residual)?;

        let mut solved_columns = Vec::with_capacity(target_indices.len());
        for target in target_indices {
            let cross_column: Vec<f64> = observed_indices.iter()
                .map(|observed| self.covariance[*observed][*target])
                .collect();
            solved_columns.push(solve_cholesky(&lower, &cross_column)?);
        }

        let conditional_mean: Vec<f64> = target_indices.iter()
            .enumerate()
            .map(|(target_position, target)| {
                let shift: f64 = observed_indices.iter()
                    .enumerate()
                    .map(|(observed_position, observed)| {
                        self.covariance[*target][*observed] * alpha[observed_position]
                    })
                    .sum();
                let _ = target_position;
                self.mean[*target] + shift
            })
            .collect();

        let mut conditional_covariance = vec![vec![0.0; target_indices.len()]; target_indices.len()];
        for (row_position, row_target) in target_indices.iter().enumerate() {
            for (column_position, column_target) in target_indices.iter().enumerate() {
                let reduction: f64 = observed_indices.iter()
                    .enumerate()
                    .map(|(observed_position, observed)| {
                        self.covariance[*row_target][*observed]
                            * solved_columns[column_position][observed_position]
                    })
                    .sum();
                conditional_covariance[row_position][column_position] =
                    self.covariance[*row_target][*column_target] - reduction;
            }
        }

        // Remove only tiny floating-point asymmetry generated by the two
        // equivalent Schur-complement entries; do not regularize eigenvalues.
        for row in 0..target_indices.len() {
            for column in 0..row {
                let average = 0.5
                    * (conditional_covariance[row][column]
                        + conditional_covariance[column][row]);
                conditional_covariance[row][column] = average;
                conditional_covariance[column][row] = average;
            }
        }

        let result = Self {
            mean: conditional_mean,
            covariance: conditional_covariance,
        };
        result.validate()?;
        Ok(result)
    }
}

fn solve_lower(lower: &[Vec<f64>], right_hand_side: &[f64]) -> Result<Vec<f64>, String> {
    if lower.len() != right_hand_side.len() {
        return Err("lower-triangular solve dimension mismatch".into());
    }
    let mut solution = vec![0.0; lower.len()];
    for row in 0..lower.len() {
        let correction: f64 = (0..row)
            .map(|column| lower[row][column] * solution[column])
            .sum();
        solution[row] = (right_hand_side[row] - correction) / lower[row][row];
    }
    if solution.iter().all(|value| value.is_finite()) {
        Ok(solution)
    } else {
        Err("lower-triangular solve is numerically unstable".into())
    }
}

/// Update a finite model posterior from the selected components of a
/// multivariate correlated observation history.
pub fn update_multivariate_model_posterior(
    models: &[MultivariateGaussianModel],
    observed_indices: &[usize],
    observations: &[f64],
) -> Result<MultivariateGaussianPosterior, String> {
    if models.is_empty() {
        return Err("at least one multivariate Gaussian model is required".into());
    }
    if observed_indices.len() != observations.len() || observed_indices.is_empty() {
        return Err("observed indices and values must have equal non-zero length".into());
    }

    let mut identifiers = std::collections::BTreeSet::new();
    let mut prior_total = 0.0;
    for model in models {
        if model.model_id.trim().is_empty() || !identifiers.insert(model.model_id.as_str()) {
            return Err("model identifiers must be non-empty and unique".into());
        }
        if !model.prior_weight.is_finite() || model.prior_weight < 0.0 {
            return Err(format!("invalid prior weight for {:?}", model.model_id));
        }
        model.observation_model.validate()?;
        validate_indices(observed_indices, model.observation_model.mean.len(), "observed")?;
        prior_total += model.prior_weight;
    }
    if !prior_total.is_finite() || prior_total <= 0.0 {
        return Err("candidate prior weights must have finite positive sum".into());
    }
    if observations.iter().any(|value| !value.is_finite()) {
        return Err("observations must be finite".into());
    }

    let log_joint = models.iter().map(|model| {
        if model.prior_weight == 0.0 {
            Ok(f64::NEG_INFINITY)
        } else {
            Ok((model.prior_weight / prior_total).ln()
                + model.observation_model.log_density_subset(observed_indices, observations)?)
        }
    }).collect::<Result<Vec<_>, String>>()?;
    let maximum = log_joint.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !maximum.is_finite() {
        return Err("all candidate models assign numerically zero likelihood".into());
    }
    let scaled_sum: f64 = log_joint.iter().map(|value| (value - maximum).exp()).sum();
    if !scaled_sum.is_finite() || scaled_sum <= 0.0 {
        return Err("posterior normalization failed".into());
    }
    let log_evidence = maximum + scaled_sum.ln();
    if !log_evidence.is_finite() {
        return Err("log evidence is not finite".into());
    }
    let mut weights: Vec<f64> = log_joint.iter()
        .map(|value| (value - log_evidence).exp()).collect();
    let total: f64 = weights.iter().sum();
    if !total.is_finite() || total <= 0.0 {
        return Err("posterior weights have no finite positive mass".into());
    }
    for weight in &mut weights {
        *weight /= total;
    }

    Ok(MultivariateGaussianPosterior {
        model_ids: models.iter().map(|model| model.model_id.clone()).collect(),
        weights,
        log_evidence,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gaussian(mean: Vec<f64>, covariance: Vec<Vec<f64>>) -> MultivariateGaussian {
        MultivariateGaussian { mean, covariance }
    }

    fn model(id: &str, prior: f64, rho: f64) -> MultivariateGaussianModel {
        MultivariateGaussianModel {
            model_id: id.into(),
            prior_weight: prior,
            observation_model: gaussian(
                vec![0.0, 0.0],
                vec![vec![1.0, rho], vec![rho, 1.0]],
            ),
        }
    }

    #[test]
    fn diagonal_covariance_matches_independent_normal_product() {
        let distribution = gaussian(
            vec![0.3, -1.2],
            vec![vec![0.49, 0.0], vec![0.0, 1.96]],
        );
        let y = [0.8, -0.4];
        let normal = |value: f64, mean: f64, sd: f64| {
            let z = (value - mean) / sd;
            -0.5 * std::f64::consts::TAU.ln() - sd.ln() - 0.5 * z * z
        };
        let expected = normal(y[0], 0.3, 0.7) + normal(y[1], -1.2, 1.4);
        assert!((distribution.log_density(&y).unwrap() - expected).abs() < 1e-12);
    }

    #[test]
    fn two_dimensional_density_matches_correlated_pair_reference() {
        let pair = super::super::correlated_likelihood::CorrelatedGaussianPair {
            mean: [0.4, -0.2],
            standard_deviation: [1.1, 0.6],
            correlation: 0.65,
        };
        let covariance = vec![
            vec![1.1_f64.powi(2), 0.65 * 1.1 * 0.6],
            vec![0.65 * 1.1 * 0.6, 0.6_f64.powi(2)],
        ];
        let distribution = gaussian(vec![0.4, -0.2], covariance);
        let y = [1.2, 0.7];
        assert!((distribution.log_density(&y).unwrap() - pair.log_density(y).unwrap()).abs() < 1e-12);
    }

    #[test]
    fn conditional_distribution_matches_bivariate_gaussian_identity() {
        let distribution = gaussian(
            vec![0.4, -0.2],
            vec![vec![1.21, 0.429], vec![0.429, 0.36]],
        );
        let conditional = distribution.conditional(&[0], &[1.2], &[1]).unwrap();
        let expected_mean = -0.2 + 0.65 * 0.6 / 1.1 * (1.2 - 0.4);
        let expected_variance = 0.36 * (1.0 - 0.65 * 0.65);
        assert!((conditional.mean[0] - expected_mean).abs() < 1e-12);
        assert!((conditional.covariance[0][0] - expected_variance).abs() < 1e-12);
    }

    #[test]
    fn conditional_with_no_history_returns_target_marginal() {
        let distribution = gaussian(
            vec![0.1, 0.2, 0.3],
            vec![
                vec![1.0, 0.2, 0.1],
                vec![0.2, 2.0, 0.3],
                vec![0.1, 0.3, 3.0],
            ],
        );
        let marginal = distribution.conditional(&[], &[], &[2, 0]).unwrap();
        assert_eq!(marginal.mean, vec![0.3, 0.1]);
        assert_eq!(marginal.covariance, vec![vec![3.0, 0.1], vec![0.1, 1.0]]);
    }

    #[test]
    fn conditional_prediction_uses_the_observed_cross_covariance() {
        let distribution = gaussian(
            vec![0.0, 0.0, 0.0],
            vec![
                vec![1.0, 0.6, 0.0],
                vec![0.6, 1.0, 0.5],
                vec![0.0, 0.5, 1.0],
            ],
        );
        let conditional = distribution.conditional(&[0], &[1.0], &[1, 2]).unwrap();
        assert!((conditional.mean[0] - 0.6).abs() < 1e-12);
        assert!(conditional.mean[1].abs() < 1e-12);
        assert!((conditional.covariance[0][0] - 0.64).abs() < 1e-12);
        assert!((conditional.covariance[0][1] - 0.5).abs() < 1e-12);
        assert!((conditional.covariance[1][1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn subset_update_discriminates_models_using_joint_correlation() {
        let models = vec![model("positive", 0.5, 0.8), model("negative", 0.5, -0.8)];
        let posterior = update_multivariate_model_posterior(&models, &[0, 1], &[1.0, 1.0]).unwrap();
        assert!(posterior.weights[0] > 0.98);
        assert!(posterior.weights[1] < 0.02);
    }

    #[test]
    fn subset_update_can_ignore_unobserved_history_coordinates() {
        let models = vec![
            MultivariateGaussianModel {
                model_id: "a".into(),
                prior_weight: 0.5,
                observation_model: gaussian(
                    vec![0.0, 0.0],
                    vec![vec![1.0, 0.8], vec![0.8, 1.0]],
                ),
            },
            MultivariateGaussianModel {
                model_id: "b".into(),
                prior_weight: 0.5,
                observation_model: gaussian(
                    vec![0.0, 0.0],
                    vec![vec![1.0, -0.8], vec![-0.8, 1.0]],
                ),
            },
        ];
        let posterior = update_multivariate_model_posterior(&models, &[0], &[1.0]).unwrap();
        assert!((posterior.weights[0] - 0.5).abs() < 1e-12);
        assert!((posterior.weights[1] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn rejects_non_square_asymmetric_and_non_positive_definite_covariance() {
        assert!(gaussian(vec![0.0, 0.0], vec![vec![1.0], vec![0.0, 1.0]]).validate().is_err());
        assert!(gaussian(vec![0.0, 0.0], vec![vec![1.0, 0.2], vec![0.3, 1.0]]).validate().is_err());
        assert!(gaussian(vec![0.0, 0.0], vec![vec![1.0, 2.0], vec![2.0, 1.0]]).validate().is_err());
    }

    #[test]
    fn rejects_duplicate_overlapping_and_out_of_range_indices() {
        let distribution = gaussian(vec![0.0, 0.0], vec![vec![1.0, 0.2], vec![0.2, 1.0]]);
        assert!(distribution.log_density_subset(&[0, 0], &[1.0, 1.0]).is_err());
        assert!(distribution.log_density_subset(&[2], &[1.0]).is_err());
        assert!(distribution.conditional(&[0], &[1.0], &[0]).is_err());
        assert!(distribution.conditional(&[], &[], &[2]).is_err());
    }

    #[test]
    fn rejects_nonfinite_observations_and_invalid_dimensions() {
        let distribution = gaussian(vec![0.0, 0.0], vec![vec![1.0, 0.2], vec![0.2, 1.0]]);
        assert!(distribution.log_density(&[0.0]).is_err());
        assert!(distribution.log_density(&[f64::NAN, 0.0]).is_err());
        assert!(distribution.conditional(&[0], &[], &[1]).is_err());
    }

    #[test]
    fn model_posterior_preserves_zero_prior_mass() {
        let models = vec![model("excluded", 0.0, -0.8), model("included", 1.0, 0.8)];
        let posterior = update_multivariate_model_posterior(&models, &[0, 1], &[1.0, 1.0]).unwrap();
        assert_eq!(posterior.weights[0], 0.0);
        assert_eq!(posterior.weights[1], 1.0);
    }
}

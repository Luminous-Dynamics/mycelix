//! Structural identifiability of physical and nuisance covariance components.
//!
//! This module audits whether a proposed crossed measurement design can
//! distinguish shared physical-support, preparation-batch, instrument and
//! independent residual variance components. It is an exact finite linear
//! covariance-design reference, not a fitted mixed-effects model or an
//! empirical variance estimator.

use crate::multivariate_gaussian::{
    update_multivariate_model_posterior, MultivariateGaussian,
    MultivariateGaussianModel, MultivariateGaussianPosterior,
};
use std::collections::BTreeSet;

/// One scalar observation and its physical/nuisance group identities.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CovarianceObservation {
    pub observation_id: String,
    pub modality: String,
    /// Non-empty only when the physical supports are declared to share the
    /// same latent physical state at the resolution of this model.
    pub physical_support_id: Option<String>,
    pub batch_id: Option<String>,
    pub instrument_id: Option<String>,
}

/// Variance components in the covariance decomposition.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VarianceComponent {
    SharedPhysicalSupport,
    PreparationBatch,
    Instrument,
    IndependentResidual,
}

impl VarianceComponent {
    pub const ALL: [Self; 4] = [
        Self::SharedPhysicalSupport,
        Self::PreparationBatch,
        Self::Instrument,
        Self::IndependentResidual,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Self::SharedPhysicalSupport => "shared_physical_support",
            Self::PreparationBatch => "preparation_batch",
            Self::Instrument => "instrument",
            Self::IndependentResidual => "independent_residual",
        }
    }
}

/// Structural identifiability summary for the covariance components.
#[derive(Clone, Debug, PartialEq)]
pub struct CovarianceIdentifiabilityAudit {
    pub observation_count: usize,
    pub component_ids: Vec<VarianceComponent>,
    /// Rank of the weighted covariance-component design matrix.
    pub structural_rank: usize,
    pub component_count: usize,
    /// Condition number of the non-zero spectrum of the weighted Gram matrix.
    /// This is a structural numerical diagnostic, not a sampling condition.
    pub gram_condition_number: Option<f64>,
    pub fully_identifiable: bool,
    /// A component is individually estimable in this linear decomposition
    /// only if removing its column decreases the rank of the complete design.
    pub individually_identifiable: Vec<bool>,
}

/// Exact noiseless recovery of the declared covariance-component coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCovarianceRecovery {
    pub component_ids: Vec<VarianceComponent>,
    pub variance_estimates: Vec<f64>,
    /// Relative Frobenius residual between supplied covariance and its
    /// projection onto the declared component span.
    pub relative_fit_residual: f64,
}

/// One scientific hypothesis for the source of observed covariance.
#[derive(Clone, Debug, PartialEq)]
pub struct CovarianceComponentHypothesis {
    pub model_id: String,
    pub prior_weight: f64,
    pub mean: Vec<f64>,
    /// Variance coefficients in `VarianceComponent::ALL` order.
    pub variance_components: Vec<f64>,
}

/// A covariance design matrix represented by its upper-triangle basis vectors.
#[derive(Clone, Debug, PartialEq)]
pub struct CovarianceComponentDesign {
    observations: Vec<CovarianceObservation>,
    component_ids: Vec<VarianceComponent>,
    basis_vectors: Vec<Vec<f64>>,
    upper_triangle_weights: Vec<f64>,
    upper_triangle_indices: Vec<(usize, usize)>,
}

fn validate_observations(observations: &[CovarianceObservation]) -> Result<(), String> {
    if observations.is_empty() {
        return Err("at least one covariance observation is required".into());
    }
    let mut observation_ids = BTreeSet::new();
    for observation in observations {
        if observation.observation_id.trim().is_empty()
            || !observation_ids.insert(observation.observation_id.as_str())
        {
            return Err("observation IDs must be non-empty and unique".into());
        }
        if observation.modality.trim().is_empty() {
            return Err(format!("modality is required for {:?}", observation.observation_id));
        }
        for (label, value) in [
            ("physical support", observation.physical_support_id.as_deref()),
            ("batch", observation.batch_id.as_deref()),
            ("instrument", observation.instrument_id.as_deref()),
        ] {
            if value.is_some_and(|value| value.trim().is_empty()) {
                return Err(format!("{label} ID must be absent or non-empty"));
            }
        }
    }
    Ok(())
}

fn component_group(
    observation: &CovarianceObservation,
    component: VarianceComponent,
) -> Option<&str> {
    match component {
        VarianceComponent::SharedPhysicalSupport => observation.physical_support_id.as_deref(),
        VarianceComponent::PreparationBatch => observation.batch_id.as_deref(),
        VarianceComponent::Instrument => observation.instrument_id.as_deref(),
        VarianceComponent::IndependentResidual => None,
    }
}

fn upper_triangle_pairs(dimension: usize) -> Vec<(usize, usize)> {
    let mut pairs = Vec::with_capacity(dimension * (dimension + 1) / 2);
    for row in 0..dimension {
        for column in row..dimension {
            pairs.push((row, column));
        }
    }
    pairs
}

fn symmetric_eigenvalues(matrix: &[Vec<f64>]) -> Result<Vec<f64>, String> {
    let dimension = matrix.len();
    if dimension == 0 || matrix.iter().any(|row| row.len() != dimension) {
        return Err("Gram matrix must be non-empty and square".into());
    }
    if matrix.iter().flatten().any(|value| !value.is_finite()) {
        return Err("Gram matrix entries must be finite".into());
    }

    let mut work = matrix.to_vec();
    let scale = work.iter().flatten().map(|value| value.abs()).fold(1.0_f64, f64::max);
    for (row, matrix_row) in work.iter().enumerate().skip(1) {
        for (column, previous_row) in work.iter().enumerate().take(row) {
            if (matrix_row[column] - previous_row[row]).abs() > 1e-10 * scale {
                return Err("Gram matrix must be symmetric".into());
            }
        }
    }

    let max_iterations = 100 * dimension * dimension;
    for _ in 0..max_iterations {
        let mut largest = 0.0;
        let mut pivot = None;
        for row in 0..dimension {
            for column in (row + 1)..dimension {
                let value = work[row][column].abs();
                if value > largest {
                    largest = value;
                    pivot = Some((row, column));
                }
            }
        }
        if largest <= 1e-13 * scale {
            break;
        }
        let Some((p, q)) = pivot else { break };

        let off_diagonal = work[p][q];
        let tau = (work[q][q] - work[p][p]) / (2.0 * off_diagonal);
        let sign = if tau >= 0.0 { 1.0 } else { -1.0 };
        let tangent = sign / (tau.abs() + (1.0 + tau * tau).sqrt());
        let cosine = 1.0 / (1.0 + tangent * tangent).sqrt();
        let sine = tangent * cosine;

        for index in 0..dimension {
            if index != p && index != q {
                let left = work[index][p];
                let right = work[index][q];
                let new_left = cosine * left - sine * right;
                let new_right = sine * left + cosine * right;
                work[index][p] = new_left;
                work[p][index] = new_left;
                work[index][q] = new_right;
                work[q][index] = new_right;
            }
        }
        let pp = work[p][p];
        let qq = work[q][q];
        work[p][p] = pp - tangent * off_diagonal;
        work[q][q] = qq + tangent * off_diagonal;
        work[p][q] = 0.0;
        work[q][p] = 0.0;
    }

    let values: Vec<f64> = (0..dimension).map(|index| work[index][index]).collect();
    if values.iter().all(|value| value.is_finite()) {
        Ok(values)
    } else {
        Err("Gram eigensolver produced a non-finite eigenvalue".into())
    }
}

fn gram_matrix(basis_vectors: &[Vec<f64>], weights: &[f64]) -> Result<Vec<Vec<f64>>, String> {
    if basis_vectors.is_empty() || weights.is_empty() {
        return Err("covariance basis and weights must be non-empty".into());
    }
    if basis_vectors.iter().any(|basis| basis.len() != weights.len()) {
        return Err("covariance basis vectors must align with upper-triangle weights".into());
    }

    let component_count = basis_vectors.len();
    let mut gram = vec![vec![0.0; component_count]; component_count];
    for (row, left) in basis_vectors.iter().enumerate() {
        for (column, right) in basis_vectors.iter().enumerate().skip(row) {
            let value: f64 = left.iter()
                .zip(right)
                .zip(weights)
                .map(|((left_value, right_value), weight)| {
                    left_value * right_value * weight
                })
                .sum();
            gram[row][column] = value;
            gram[column][row] = value;
        }
    }
    Ok(gram)
}

fn rank_and_condition(gram: &[Vec<f64>]) -> Result<(usize, Option<f64>), String> {
    let eigenvalues = symmetric_eigenvalues(gram)?;
    let maximum = eigenvalues.iter().copied().fold(0.0_f64, f64::max);
    if maximum <= 0.0 {
        return Ok((0, None));
    }
    // Rank threshold is stated on the Gram spectrum. It is intentionally
    // conservative for near-collinear component signatures.
    let threshold = maximum * 1e-10;
    let positive: Vec<f64> = eigenvalues.iter()
        .copied()
        .filter(|value| *value > threshold)
        .collect();
    let rank = positive.len();
    let condition = positive.iter().copied().reduce(f64::min)
        .map(|minimum| (maximum / minimum).sqrt());
    Ok((rank, condition))
}

fn solve_linear_system(matrix: &[Vec<f64>], rhs: &[f64]) -> Result<Vec<f64>, String> {
    let dimension = matrix.len();
    if dimension == 0
        || rhs.len() != dimension
        || matrix.iter().any(|row| row.len() != dimension)
    {
        return Err("linear system dimensions are invalid".into());
    }
    let mut augmented: Vec<Vec<f64>> = matrix.iter()
        .zip(rhs)
        .map(|(row, value)| {
            let mut augmented_row = row.clone();
            augmented_row.push(*value);
            augmented_row
        })
        .collect();

    let scale = matrix.iter().flatten()
        .map(|value| value.abs()).fold(1.0_f64, f64::max);
    for pivot_column in 0..dimension {
        let pivot_row = (pivot_column..dimension)
            .max_by(|left, right| {
                augmented[*left][pivot_column].abs()
                    .total_cmp(&augmented[*right][pivot_column].abs())
            })
            .ok_or_else(|| "linear system has no pivot row".to_string())?;
        if augmented[pivot_row][pivot_column].abs() <= 1e-12 * scale {
            return Err("covariance components are not structurally identifiable".into());
        }
        augmented.swap(pivot_column, pivot_row);

        let pivot = augmented[pivot_column][pivot_column];
        for column in pivot_column..=dimension {
            augmented[pivot_column][column] /= pivot;
        }
        for row in 0..dimension {
            if row == pivot_column {
                continue;
            }
            let factor = augmented[row][pivot_column];
            for column in pivot_column..=dimension {
                augmented[row][column] -= factor * augmented[pivot_column][column];
            }
        }
    }

    let solution: Vec<f64> = augmented.iter().map(|row| row[dimension]).collect();
    if solution.iter().all(|value| value.is_finite()) {
        Ok(solution)
    } else {
        Err("variance-component solution is non-finite".into())
    }
}

impl CovarianceComponentDesign {
    /// Build component basis matrices from exact group identities.
    ///
    /// A shared covariance contribution is present only when two observations
    /// carry the same non-empty group ID for that component. Equal specimen
    /// labels do not imply equal physical support unless the physical support
    /// IDs themselves match.
    pub fn new(observations: Vec<CovarianceObservation>) -> Result<Self, String> {
        validate_observations(&observations)?;
        let component_ids = VarianceComponent::ALL.to_vec();
        let upper_triangle_indices = upper_triangle_pairs(observations.len());
        let upper_triangle_weights: Vec<f64> = upper_triangle_indices.iter()
            .map(|(row, column)| if row == column { 1.0 } else { 2.0 })
            .collect();

        let basis_vectors = component_ids.iter().map(|component| {
            upper_triangle_indices.iter().map(|(row, column)| {
                if *component == VarianceComponent::IndependentResidual {
                    if row == column { 1.0 } else { 0.0 }
                } else {
                    let left = component_group(&observations[*row], *component);
                    let right = component_group(&observations[*column], *component);
                    match (left, right) {
                        (Some(left), Some(right)) if left == right => 1.0,
                        _ => 0.0,
                    }
                }
            }).collect()
        }).collect();

        Ok(Self {
            observations,
            component_ids,
            basis_vectors,
            upper_triangle_weights,
            upper_triangle_indices,
        })
    }

    pub fn observations(&self) -> &[CovarianceObservation] {
        &self.observations
    }

    pub fn component_ids(&self) -> &[VarianceComponent] {
        &self.component_ids
    }

    /// Audit the structural rank and identify individually estimable columns.
    ///
    /// This detects linear confounding among the declared covariance basis
    /// matrices. It does not quantify finite-sample posterior precision or
    /// guarantee good statistical power.
    pub fn audit(&self) -> Result<CovarianceIdentifiabilityAudit, String> {
        let gram = gram_matrix(&self.basis_vectors, &self.upper_triangle_weights)?;
        let (structural_rank, gram_condition_number) = rank_and_condition(&gram)?;
        let component_count = self.component_ids.len();
        let individually_identifiable = (0..component_count).map(|removed| {
            let reduced: Vec<Vec<f64>> = gram.iter().enumerate()
                .filter(|(row, _)| *row != removed)
                .map(|(_, values)| values.iter().enumerate()
                    .filter(|(column, _)| *column != removed)
                    .map(|(_, value)| *value)
                    .collect())
                .collect();
            let (reduced_rank, _) = rank_and_condition(&reduced)
                .unwrap_or((0, None));
            reduced_rank < structural_rank
        }).collect();

        Ok(CovarianceIdentifiabilityAudit {
            observation_count: self.observations.len(),
            component_ids: self.component_ids.clone(),
            structural_rank,
            component_count,
            gram_condition_number,
            fully_identifiable: structural_rank == component_count,
            individually_identifiable,
        })
    }

    /// Construct the exact covariance implied by non-negative component
    /// variances in canonical component order.
    pub fn covariance_from_variances(&self, variances: &[f64]) -> Result<Vec<Vec<f64>>, String> {
        if variances.len() != self.component_ids.len() {
            return Err("variance vector must align with the component list".into());
        }
        if variances.iter().any(|value| !value.is_finite() || *value < 0.0) {
            return Err("variance components must be finite and non-negative".into());
        }
        let dimension = self.observations.len();
        let mut covariance = vec![vec![0.0; dimension]; dimension];
        for (component_index, variance) in variances.iter().enumerate() {
            for (basis_value, (row, column)) in self.basis_vectors[component_index]
                .iter().zip(&self.upper_triangle_indices)
            {
                let value = variance * basis_value;
                covariance[*row][*column] += value;
                covariance[*column][*row] = covariance[*row][*column];
            }
        }
        Ok(covariance)
    }

    /// Update a finite posterior over physical/nuisance covariance hypotheses.
    ///
    /// This connects declared support/batch/instrument incidence matrices to
    /// the multivariate Gaussian posterior. If two hypotheses induce the same
    /// covariance on a non-identifying design, their likelihoods are equal
    /// and the data cannot distinguish their attribution.
    pub fn update_hypothesis_posterior(
        &self,
        hypotheses: &[CovarianceComponentHypothesis],
        observed_indices: &[usize],
        observations: &[f64],
    ) -> Result<MultivariateGaussianPosterior, String> {
        if hypotheses.is_empty() {
            return Err("at least one covariance hypothesis is required".into());
        }
        let mut models = Vec::with_capacity(hypotheses.len());
        for hypothesis in hypotheses {
            if hypothesis.mean.len() != self.observations.len()
                || hypothesis.mean.iter().any(|value| !value.is_finite())
            {
                return Err(format!(
                    "mean vector for covariance hypothesis {:?} must match the design dimension",
                    hypothesis.model_id
                ));
            }
            let covariance = self.covariance_from_variances(&hypothesis.variance_components)?;
            models.push(MultivariateGaussianModel {
                model_id: hypothesis.model_id.clone(),
                prior_weight: hypothesis.prior_weight,
                observation_model: MultivariateGaussian {
                    mean: hypothesis.mean.clone(),
                    covariance,
                },
            });
        }
        update_multivariate_model_posterior(
            &models,
            observed_indices,
            observations,
        )
    }

    /// Recover variance coefficients from a noiseless covariance matrix.
    ///
    /// This solves the linear span problem exactly up to floating-point
    /// precision. It is an oracle for synthetic tests, not an estimator for
    /// noisy empirical covariance data.
    pub fn recover_exact_variance_components(
        &self,
        covariance: &[Vec<f64>],
    ) -> Result<ExactCovarianceRecovery, String> {
        let dimension = self.observations.len();
        if covariance.len() != dimension
            || covariance.iter().any(|row| row.len() != dimension)
        {
            return Err("covariance matrix dimensions must match the design".into());
        }
        if covariance.iter().flatten().any(|value| !value.is_finite()) {
            return Err("covariance matrix entries must be finite".into());
        }
        let scale = covariance.iter().flatten()
            .map(|value| value.abs()).fold(1.0_f64, f64::max);
        for (row, covariance_row) in covariance.iter().enumerate().skip(1) {
            for (column, previous_row) in covariance.iter().enumerate().take(row) {
                if (covariance_row[column] - previous_row[row]).abs() > 1e-10 * scale {
                    return Err("covariance matrix must be symmetric".into());
                }
            }
        }

        let audit = self.audit()?;
        if !audit.fully_identifiable {
            return Err(format!(
                "covariance components are structurally non-identifiable: rank {} of {}",
                audit.structural_rank, audit.component_count
            ));
        }

        let covariance_upper: Vec<f64> = self.upper_triangle_indices.iter()
            .map(|(row, column)| covariance[*row][*column])
            .collect();
        let rhs: Vec<f64> = self.basis_vectors.iter().map(|basis| {
            basis.iter().zip(&covariance_upper)
                .zip(&self.upper_triangle_weights)
                .map(|((basis_value, covariance_value), weight)| {
                    basis_value * covariance_value * weight
                })
                .sum()
        }).collect();
        let gram = gram_matrix(&self.basis_vectors, &self.upper_triangle_weights)?;
        let estimates = solve_linear_system(&gram, &rhs)?;
        if estimates.iter().any(|value| *value < -1e-8 * scale) {
            return Err("covariance lies outside the non-negative component cone".into());
        }

        let fitted = self.covariance_from_variances(
            &estimates.iter().map(|value| value.max(0.0)).collect::<Vec<_>>(),
        )?;
        let mut residual_sum = 0.0;
        let mut covariance_sum = 0.0;
        for ((row, column), weight) in self.upper_triangle_indices.iter()
            .zip(&self.upper_triangle_weights)
        {
            let residual = covariance[*row][*column] - fitted[*row][*column];
            residual_sum += weight * residual * residual;
            covariance_sum += weight * covariance[*row][*column] * covariance[*row][*column];
        }
        let relative_fit_residual =
            residual_sum.sqrt() / covariance_sum.sqrt().max(1.0);

        Ok(ExactCovarianceRecovery {
            component_ids: self.component_ids.clone(),
            variance_estimates: estimates.iter().map(|value| value.max(0.0)).collect(),
            relative_fit_residual,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(
        id: &str,
        modality: &str,
        physical: Option<&str>,
        batch: Option<&str>,
        instrument: Option<&str>,
    ) -> CovarianceObservation {
        CovarianceObservation {
            observation_id: id.into(),
            modality: modality.into(),
            physical_support_id: physical.map(str::to_owned),
            batch_id: batch.map(str::to_owned),
            instrument_id: instrument.map(str::to_owned),
        }
    }

    fn confounded_pair() -> CovarianceComponentDesign {
        CovarianceComponentDesign::new(vec![
            observation("ct-1", "ct", Some("support-A"), Some("batch-A"), Some("instrument-A")),
            observation("tracer-1", "tracer", Some("support-A"), Some("batch-A"), Some("instrument-A")),
        ]).unwrap()
    }

    fn crossed_support_batch_instrument_design() -> CovarianceComponentDesign {
        let mut observations = Vec::new();
        for support in ["support-A", "support-B"] {
            for batch in ["batch-A", "batch-B"] {
                for instrument in ["instrument-A", "instrument-B"] {
                    for modality in ["ct", "tracer"] {
                        let id = format!("{support}:{batch}:{instrument}:{modality}");
                        observations.push(observation(
                            &id, modality, Some(support), Some(batch), Some(instrument),
                        ));
                    }
                }
            }
        }
        CovarianceComponentDesign::new(observations).unwrap()
    }

    #[test]
    fn one_ct_tracer_pair_cannot_separate_shared_physics_batch_and_instrument() {
        let audit = confounded_pair().audit().unwrap();
        assert_eq!(audit.structural_rank, 2);
        assert_eq!(audit.component_count, 4);
        assert!(!audit.fully_identifiable);
        assert_eq!(audit.individually_identifiable, vec![false, false, false, true]);
    }

    #[test]
    fn crossed_replication_structurally_identifies_all_declared_components() {
        let audit = crossed_support_batch_instrument_design().audit().unwrap();
        assert_eq!(audit.structural_rank, 4);
        assert_eq!(audit.component_count, 4);
        assert!(audit.fully_identifiable);
        assert_eq!(audit.individually_identifiable, vec![true, true, true, true]);
        assert!(audit.gram_condition_number.is_some_and(|value| value.is_finite()));
    }

    #[test]
    fn exact_covariance_oracle_recovers_each_variance_component() {
        let design = crossed_support_batch_instrument_design();
        let expected = [0.7, 0.4, 0.2, 0.1];
        let covariance = design.covariance_from_variances(&expected).unwrap();
        let recovered = design.recover_exact_variance_components(&covariance).unwrap();
        for (actual, expected) in recovered.variance_estimates.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-10);
        }
        assert!(recovered.relative_fit_residual < 1e-12);
    }

    #[test]
    fn batch_only_pair_does_not_certify_physical_cross_modal_covariance() {
        let design = CovarianceComponentDesign::new(vec![
            observation("ct-1", "ct", Some("support-A"), Some("batch-shared"), Some("ct-device")),
            observation("tracer-1", "tracer", Some("support-B"), Some("batch-shared"), Some("tracer-device")),
        ]).unwrap();
        let audit = design.audit().unwrap();
        assert_eq!(audit.structural_rank, 2);
        assert!(!audit.individually_identifiable[0]);
        assert!(audit.individually_identifiable[1]);
        assert!(!audit.fully_identifiable);
    }

    #[test]
    fn modality_labels_do_not_create_physical_support_identity() {
        let design = CovarianceComponentDesign::new(vec![
            observation("ct-1", "ct", Some("support-A"), Some("batch-A"), Some("ct-device")),
            observation("tracer-1", "tracer", Some("support-B"), Some("batch-A"), Some("tracer-device")),
        ]).unwrap();
        let covariance = design.covariance_from_variances(&[1.0, 0.5, 0.25, 0.1]).unwrap();
        assert!((covariance[0][1] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn nonidentifying_single_pair_cannot_update_physical_batch_instrument_attribution() {
        let design = confounded_pair();
        let hypotheses = vec![
            CovarianceComponentHypothesis {
                model_id: "physical_only".into(),
                prior_weight: 0.4,
                mean: vec![0.0, 0.0],
                variance_components: vec![1.0, 0.0, 0.0, 0.1],
            },
            CovarianceComponentHypothesis {
                model_id: "batch_only".into(),
                prior_weight: 0.35,
                mean: vec![0.0, 0.0],
                variance_components: vec![0.0, 1.0, 0.0, 0.1],
            },
            CovarianceComponentHypothesis {
                model_id: "instrument_only".into(),
                prior_weight: 0.25,
                mean: vec![0.0, 0.0],
                variance_components: vec![0.0, 0.0, 1.0, 0.1],
            },
        ];
        let posterior = design.update_hypothesis_posterior(
            &hypotheses,
            &[0, 1],
            &[1.0, 0.7],
        ).unwrap();
        for (actual, expected) in posterior.weights.iter()
            .zip([0.4_f64, 0.35, 0.25])
        {
            assert!((actual - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn batch_only_pair_supports_batch_attribution_but_not_physical_support() {
        let design = CovarianceComponentDesign::new(vec![
            observation("ct-1", "ct", Some("support-A"), Some("shared-batch"), Some("ct-device")),
            observation("tracer-1", "tracer", Some("support-B"), Some("shared-batch"), Some("tracer-device")),
        ]).unwrap();
        let hypotheses = vec![
            CovarianceComponentHypothesis {
                model_id: "physical_only".into(),
                prior_weight: 0.5,
                mean: vec![0.0, 0.0],
                variance_components: vec![1.0, 0.0, 0.0, 0.1],
            },
            CovarianceComponentHypothesis {
                model_id: "batch_only".into(),
                prior_weight: 0.5,
                mean: vec![0.0, 0.0],
                variance_components: vec![0.0, 1.0, 0.0, 0.1],
            },
        ];
        let posterior = design.update_hypothesis_posterior(
            &hypotheses,
            &[0, 1],
            &[1.0, 1.0],
        ).unwrap();
        assert!(posterior.weights[1] > 0.99);
    }

    #[test]
    fn crossed_design_can_discriminate_shared_physics_from_batch_covariance() {
        let design = crossed_support_batch_instrument_design();
        let hypotheses = vec![
            CovarianceComponentHypothesis {
                model_id: "physical_only".into(),
                prior_weight: 0.5,
                mean: vec![0.0; 16],
                variance_components: vec![1.0, 0.0, 0.0, 0.1],
            },
            CovarianceComponentHypothesis {
                model_id: "batch_only".into(),
                prior_weight: 0.5,
                mean: vec![0.0; 16],
                variance_components: vec![0.0, 1.0, 0.0, 0.1],
            },
        ];
        // Shared-support signal: repeated CT/tracer measurements within each
        // support have a common latent effect, while the two supports differ.
        let observations = vec![
            1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5,
            -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5,
        ];
        let posterior = design.update_hypothesis_posterior(
            &hypotheses,
            &(0..16).collect::<Vec<_>>(),
            &observations,
        ).unwrap();
        assert!(posterior.weights[0] > 0.999);
    }

    #[test]
    fn covariance_hypothesis_update_rejects_invalid_means_and_variances() {
        let design = confounded_pair();
        let invalid_mean = CovarianceComponentHypothesis {
            model_id: "bad_mean".into(),
            prior_weight: 1.0,
            mean: vec![0.0],
            variance_components: vec![1.0, 0.0, 0.0, 0.1],
        };
        assert!(design.update_hypothesis_posterior(
            &[invalid_mean], &[0, 1], &[0.0, 0.0],
        ).is_err());
        let invalid_variance = CovarianceComponentHypothesis {
            model_id: "bad_variance".into(),
            prior_weight: 1.0,
            mean: vec![0.0, 0.0],
            variance_components: vec![1.0, -1.0, 0.0, 0.1],
        };
        assert!(design.update_hypothesis_posterior(
            &[invalid_variance], &[0, 1], &[0.0, 0.0],
        ).is_err());
    }

    #[test]
    fn rejects_duplicate_observation_ids_and_empty_group_ids() {
        assert!(CovarianceComponentDesign::new(vec![
            observation("same", "ct", Some("support-A"), Some("batch-A"), Some("device-A")),
            observation("same", "tracer", Some("support-B"), Some("batch-B"), Some("device-B")),
        ]).is_err());
        assert!(CovarianceComponentDesign::new(vec![
            observation("id", "ct", Some(" "), Some("batch-A"), Some("device-A")),
        ]).is_err());
    }

    #[test]
    fn exact_recovery_fails_closed_for_non_identifiable_covariance() {
        let design = confounded_pair();
        let covariance = design.covariance_from_variances(&[0.7, 0.4, 0.2, 0.1]).unwrap();
        assert!(design.recover_exact_variance_components(&covariance).is_err());
    }

    #[test]
    fn rejects_invalid_variance_and_covariance_inputs() {
        let design = crossed_support_batch_instrument_design();
        assert!(design.covariance_from_variances(&[1.0, 0.0, -0.1, 0.2]).is_err());
        assert!(design.covariance_from_variances(&[1.0, 0.2]).is_err());
        let asymmetric = vec![vec![1.0, 0.2], vec![0.3, 1.0]];
        assert!(confounded_pair().recover_exact_variance_components(&asymmetric).is_err());
    }
}

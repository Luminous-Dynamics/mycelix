#![forbid(unsafe_code)]

//! Fixed-proposal defensive importance sampling for bounded rare-event
//! probability estimation.
//!
//! The nominal target is the whitened standard normal p(z)=N(0,I). The
//! proposal is q(z)=epsilon*p(z)+(1-epsilon)*q_adapt(z), where q_adapt is a
//! finite Gaussian mixture with full covariance represented by Cholesky
//! factors. Because q >= epsilon*p, p/q <= 1/epsilon. Each event-weighted
//! contribution is therefore bounded, enabling a fixed-sample
//! empirical-Bernstein upper confidence bound.
//!
//! This is an estimator/evidence core, not an orbit propagator. Callers supply
//! independent evaluation samples and high-fidelity continuous-time event
//! classifications. The crate does not choose or certify an RNG.

use std::{collections::BTreeSet, fmt};

const LOG_2PI: f64 = 1.837_877_066_409_345_3;
const NUMERIC_TOLERANCE: f64 = 2.0e-10;

#[derive(Clone, Debug, PartialEq)]
pub struct GaussianComponent {
    pub id: String,
    pub mixture_weight: f64,
    pub mean: Vec<f64>,
    /// Lower-triangular L such that covariance = L * L^T.
    pub cholesky: Vec<Vec<f64>>,
}

impl GaussianComponent {
    fn validate(&self, dimension: usize) -> Result<(), EstimateError> {
        if !valid_identifier(&self.id) {
            return Err(EstimateError::InvalidIdentifier("component_id"));
        }
        if !self.mixture_weight.is_finite() || self.mixture_weight <= 0.0 {
            return Err(EstimateError::InvalidMixtureWeight(self.mixture_weight));
        }
        if self.mean.len() != dimension {
            return Err(EstimateError::DimensionMismatch {
                expected: dimension,
                actual: self.mean.len(),
                field: "component_mean",
            });
        }
        if self.mean.iter().any(|x| !x.is_finite()) {
            return Err(EstimateError::NonFiniteValue("component_mean"));
        }
        if self.cholesky.len() != dimension
            || self.cholesky.iter().any(|row| row.len() != dimension)
        {
            return Err(EstimateError::InvalidCholeskyShape);
        }
        for i in 0..dimension {
            for j in 0..dimension {
                let value = self.cholesky[i][j];
                if !value.is_finite() {
                    return Err(EstimateError::NonFiniteValue("cholesky"));
                }
                if j > i && value.abs() > NUMERIC_TOLERANCE {
                    return Err(EstimateError::CholeskyNotLowerTriangular);
                }
            }
            if self.cholesky[i][i] <= 0.0 {
                return Err(EstimateError::CholeskyNonPositiveDiagonal);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct DefensiveGaussianMixture {
    pub id: String,
    /// Nominal-target fraction epsilon in q = epsilon*p + (1-epsilon)*q_adapt.
    pub epsilon: f64,
    pub dimension: usize,
    /// Component weights are conditional on selecting the adaptive part.
    pub components: Vec<GaussianComponent>,
}

impl DefensiveGaussianMixture {
    pub fn validate(&self) -> Result<(), EstimateError> {
        if !valid_identifier(&self.id) {
            return Err(EstimateError::InvalidIdentifier("proposal_id"));
        }
        if self.dimension == 0 {
            return Err(EstimateError::ZeroDimension);
        }
        if !self.epsilon.is_finite() || self.epsilon <= 0.0 || self.epsilon >= 1.0 {
            return Err(EstimateError::InvalidDefensiveFraction(self.epsilon));
        }
        if !(1.0 / self.epsilon).is_finite() {
            return Err(EstimateError::InvalidDefensiveFraction(self.epsilon));
        }
        if self.components.is_empty() {
            return Err(EstimateError::EmptyAdaptiveMixture);
        }

        let mut total_weight = 0.0;
        let mut component_ids = BTreeSet::new();
        for component in &self.components {
            component.validate(self.dimension)?;
            if !component_ids.insert(component.id.as_str()) {
                return Err(EstimateError::DuplicateComponentId(component.id.clone()));
            }
            total_weight += component.mixture_weight;
        }
        if !total_weight.is_finite() || (total_weight - 1.0).abs() > 1.0e-10 {
            return Err(EstimateError::AdaptiveWeightsDoNotSumToOne(total_weight));
        }
        Ok(())
    }

    pub fn defensive_weight_cap(&self) -> Result<f64, EstimateError> {
        self.validate()?;
        Ok(1.0 / self.epsilon)
    }

    pub fn log_target_density(&self, point: &[f64]) -> Result<f64, EstimateError> {
        self.validate_point(point)?;
        Ok(log_standard_normal(point))
    }

    /// Log density of the entire defensive mixture. The density is not
    /// conditioned on which component happened to generate the sample.
    pub fn log_proposal_density(&self, point: &[f64]) -> Result<f64, EstimateError> {
        self.validate()?;
        self.validate_point(point)?;
        let log_target = log_standard_normal(point);
        let mut terms = Vec::with_capacity(self.components.len() + 1);
        terms.push(self.epsilon.ln() + log_target);

        let adaptive_mass = 1.0 - self.epsilon;
        for component in &self.components {
            let log_component = log_gaussian_cholesky(point, &component.mean, &component.cholesky)?;
            terms.push(adaptive_mass.ln() + component.mixture_weight.ln() + log_component);
        }
        Ok(log_sum_exp(&terms))
    }

    pub fn log_likelihood_ratio(&self, point: &[f64]) -> Result<f64, EstimateError> {
        let log_weight = self.log_target_density(point)? - self.log_proposal_density(point)?;
        let cap_log = -self.epsilon.ln();
        if log_weight > cap_log + NUMERIC_TOLERANCE {
            return Err(EstimateError::DefensiveWeightCapViolated {
                log_weight,
                log_cap: cap_log,
            });
        }
        Ok(log_weight)
    }

    pub fn likelihood_ratio(&self, point: &[f64]) -> Result<f64, EstimateError> {
        let log_weight = self.log_likelihood_ratio(point)?;
        // Underflow to zero is representable; it is an extremely small
        // positive contribution, not an invalid negative or infinite weight.
        Ok(log_weight.exp())
    }

    /// Deterministically map external random variates into one proposal draw.
    ///
    /// mixture_gate and component_selector must be independent Uniform[0,1)
    /// variates; standard_normal must be an independent N(0,I) vector. This
    /// keeps RNG choice outside the mathematical reference implementation.
    pub fn sample_from_latents(
        &self,
        mixture_gate: f64,
        component_selector: f64,
        standard_normal: &[f64],
    ) -> Result<ProposalSample, EstimateError> {
        self.validate()?;
        self.validate_point(standard_normal)?;
        if !is_unit_interval_open(mixture_gate) || !is_unit_interval_open(component_selector) {
            return Err(EstimateError::UniformVariateOutOfRange);
        }

        if mixture_gate < self.epsilon {
            return Ok(ProposalSample {
                generating_component_id: "nominal-defensive-component".to_string(),
                point: standard_normal.to_vec(),
            });
        }

        let mut cumulative = 0.0;
        let mut chosen_index = self.components.len() - 1;
        for (index, component) in self.components.iter().enumerate() {
            cumulative += component.mixture_weight;
            if component_selector < cumulative {
                chosen_index = index;
                break;
            }
        }
        let chosen = &self.components[chosen_index];
        let point = affine_standard_normal(&chosen.mean, &chosen.cholesky, standard_normal)?;
        Ok(ProposalSample {
            generating_component_id: chosen.id.clone(),
            point,
        })
    }

    fn validate_point(&self, point: &[f64]) -> Result<(), EstimateError> {
        if point.len() != self.dimension {
            return Err(EstimateError::DimensionMismatch {
                expected: self.dimension,
                actual: point.len(),
                field: "sample",
            });
        }
        if point.iter().any(|x| !x.is_finite()) {
            return Err(EstimateError::NonFiniteValue("sample"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProposalSample {
    pub generating_component_id: String,
    pub point: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionObservation {
    pub point: Vec<f64>,
    /// Must be the output of the declared high-fidelity continuous-time oracle.
    pub collision: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvaluationEvidence {
    pub proposal_id: String,
    /// Frozen proposal-training split/configuration reference.
    pub proposal_training_id: String,
    /// Independent final-estimation split reference; must differ from training.
    pub evaluation_sample_id: String,
    /// Declared IID proposal-sampling design.
    pub sampling_design_id: String,
    pub sampling_evidence_ref: String,
    pub sample_order_digest: String,
    pub event_oracle_id: String,
    pub event_oracle_revision: String,
    pub event_oracle_is_high_fidelity_continuous_time: bool,
    pub proposal_frozen_before_evaluation: bool,
    pub evaluation_draws_are_iid_from_full_mixture: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ImportanceSamplingReport {
    pub method: &'static str,
    pub target_distribution: &'static str,
    pub proposal_id: String,
    pub proposal_training_id: String,
    pub evaluation_sample_id: String,
    pub sampling_design_id: String,
    pub sampling_evidence_ref: String,
    pub sample_order_digest: String,
    pub event_oracle_id: String,
    pub event_oracle_revision: String,
    pub sample_count: usize,
    pub collision_count: usize,
    pub defensive_fraction: f64,
    pub weighted_contribution_cap: f64,
    pub estimate: f64,
    pub sample_variance: f64,
    pub standard_error: f64,
    pub confidence_delta: f64,
    pub one_sided_confidence_level: f64,
    pub empirical_bernstein_radius: f64,
    pub upper_confidence_bound: f64,
    pub max_observed_likelihood_ratio: f64,
    pub likelihood_ratio_ess: f64,
    pub collision_contribution_ess: f64,
    pub max_collision_contribution_share: f64,
    pub inference_horizon: &'static str,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum EstimateError {
    InvalidIdentifier(&'static str),
    ZeroDimension,
    InvalidDefensiveFraction(f64),
    EmptyAdaptiveMixture,
    DuplicateComponentId(String),
    InvalidMixtureWeight(f64),
    AdaptiveWeightsDoNotSumToOne(f64),
    DimensionMismatch {
        expected: usize,
        actual: usize,
        field: &'static str,
    },
    InvalidCholeskyShape,
    CholeskyNotLowerTriangular,
    CholeskyNonPositiveDiagonal,
    NonFiniteValue(&'static str),
    DefensiveWeightCapViolated {
        log_weight: f64,
        log_cap: f64,
    },
    UniformVariateOutOfRange,
    InvalidDelta(f64),
    InsufficientSamples(usize),
    EvidenceProposalMismatch,
    TrainingAndEvaluationMustBeDisjoint,
    ProposalNotFrozen,
    NonIidOrUnattestedSampling,
    MissingSamplingEvidence,
    LowFidelityEventOracleNotAccepted,
    MissingEventOracleRevision,
    DefensiveContributionCapViolated {
        contribution: f64,
        cap: f64,
    },
}

impl fmt::Display for EstimateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIdentifier(field) => write!(
                f,
                "{field} must be non-empty and use only ASCII letters, digits, dot, underscore, colon, slash and hyphen"
            ),
            Self::ZeroDimension => write!(f, "proposal dimension must be greater than zero"),
            Self::InvalidDefensiveFraction(value) => write!(
                f,
                "defensive target fraction must be finite and in (0,1); got {value}"
            ),
            Self::EmptyAdaptiveMixture => {
                write!(f, "adaptive proposal must contain at least one component")
            }
            Self::DuplicateComponentId(value) => {
                write!(f, "duplicate adaptive component ID {value}")
            }
            Self::InvalidMixtureWeight(value) => write!(
                f,
                "adaptive component weight must be finite and positive; got {value}"
            ),
            Self::AdaptiveWeightsDoNotSumToOne(value) => {
                write!(f, "adaptive component weights sum to {value}, not 1")
            }
            Self::DimensionMismatch {
                expected,
                actual,
                field,
            } => write!(
                f,
                "{field} dimension mismatch: expected {expected}, got {actual}"
            ),
            Self::InvalidCholeskyShape => write!(
                f,
                "Cholesky factor must be square and match the proposal dimension"
            ),
            Self::CholeskyNotLowerTriangular => {
                write!(f, "Cholesky factor must be lower triangular")
            }
            Self::CholeskyNonPositiveDiagonal => {
                write!(f, "Cholesky factor diagonal must be positive")
            }
            Self::NonFiniteValue(field) => write!(f, "{field} contains a non-finite value"),
            Self::DefensiveWeightCapViolated {
                log_weight,
                log_cap,
            } => write!(
                f,
                "likelihood ratio exceeds defensive cap: log weight {log_weight}, log cap {log_cap}"
            ),
            Self::UniformVariateOutOfRange => {
                write!(f, "sampling uniforms must be finite values in [0,1)")
            }
            Self::InvalidDelta(value) => write!(
                f,
                "confidence delta must be finite and in (0,1); got {value}"
            ),
            Self::InsufficientSamples(value) => write!(
                f,
                "at least two evaluation samples are required; got {value}"
            ),
            Self::EvidenceProposalMismatch => write!(
                f,
                "evaluation evidence proposal ID does not match the proposal object"
            ),
            Self::TrainingAndEvaluationMustBeDisjoint => write!(
                f,
                "proposal training and final evaluation split IDs must differ"
            ),
            Self::ProposalNotFrozen => write!(f, "proposal must be frozen before final evaluation"),
            Self::NonIidOrUnattestedSampling => write!(
                f,
                "fixed-sample inference requires an attested IID draw design from the frozen full mixture"
            ),
            Self::MissingSamplingEvidence => write!(
                f,
                "sampling design, evidence reference and sample-order digest are required"
            ),
            Self::LowFidelityEventOracleNotAccepted => write!(
                f,
                "final event labels must come from a high-fidelity continuous-time event oracle"
            ),
            Self::MissingEventOracleRevision => {
                write!(f, "event oracle ID and revision are required")
            }
            Self::DefensiveContributionCapViolated { contribution, cap } => write!(
                f,
                "weighted event contribution {contribution} exceeds defensive cap {cap}"
            ),
        }
    }
}

impl std::error::Error for EstimateError {}

/// Fixed-sample estimator for the probability of a high-fidelity event.
///
/// The one-sided empirical-Bernstein bound uses independent bounded
/// contributions in [0, 1/epsilon]. It is not valid after adaptive peeking or
/// optional stopping. The reported probability is conditional on the target,
/// proposal and event-oracle assumptions, not a physical-safety certification.
pub fn estimate_collision_probability(
    proposal: &DefensiveGaussianMixture,
    observations: &[CollisionObservation],
    delta: f64,
    evidence: &EvaluationEvidence,
) -> Result<ImportanceSamplingReport, EstimateError> {
    proposal.validate()?;
    validate_delta(delta)?;
    validate_evidence(proposal, evidence)?;

    if observations.len() < 2 {
        return Err(EstimateError::InsufficientSamples(observations.len()));
    }

    let cap = 1.0 / proposal.epsilon;
    let mut contributions = Vec::with_capacity(observations.len());
    let mut weights = Vec::with_capacity(observations.len());
    let mut collision_count = 0usize;

    for observation in observations {
        let weight = proposal.likelihood_ratio(&observation.point)?;
        let contribution = if observation.collision {
            collision_count += 1;
            weight
        } else {
            0.0
        };
        if contribution > cap * (1.0 + NUMERIC_TOLERANCE) {
            return Err(EstimateError::DefensiveContributionCapViolated { contribution, cap });
        }
        contributions.push(contribution.min(cap));
        weights.push(weight);
    }

    let n = contributions.len();
    let mean = contributions.iter().sum::<f64>() / n as f64;
    let variance = sample_variance(&contributions, mean);
    let standard_error = (variance / n as f64).sqrt();

    // Maurer-Pontil empirical Bernstein, scaled from [0,1] to [0,cap].
    let log_term = (2.0 / delta).ln();
    let radius = (2.0 * variance * log_term / n as f64).sqrt()
        + (7.0 * cap * log_term) / (3.0 * (n - 1) as f64);
    let upper = (mean + radius).min(1.0);

    let sum_weights: f64 = weights.iter().sum();
    let sum_weight_squares: f64 = weights.iter().map(|weight| weight * weight).sum();
    let weight_ess = if sum_weight_squares > 0.0 {
        (sum_weights * sum_weights / sum_weight_squares).clamp(0.0, n as f64)
    } else {
        0.0
    };

    let sum_event: f64 = contributions.iter().sum();
    let sum_event_squares: f64 = contributions.iter().map(|value| value * value).sum();
    let event_ess = if sum_event_squares > 0.0 {
        (sum_event * sum_event / sum_event_squares).clamp(0.0, n as f64)
    } else {
        0.0
    };
    let max_event_share = if sum_event > 0.0 {
        contributions.iter().copied().fold(0.0, f64::max) / sum_event
    } else {
        0.0
    };
    let max_weight = weights.iter().copied().fold(0.0, f64::max);

    Ok(ImportanceSamplingReport {
        method: "defensive_gaussian_mixture_importance_sampling",
        target_distribution: "whitened_standard_normal_N_0_I",
        proposal_id: proposal.id.clone(),
        proposal_training_id: evidence.proposal_training_id.clone(),
        evaluation_sample_id: evidence.evaluation_sample_id.clone(),
        sampling_design_id: evidence.sampling_design_id.clone(),
        sampling_evidence_ref: evidence.sampling_evidence_ref.clone(),
        sample_order_digest: evidence.sample_order_digest.clone(),
        event_oracle_id: evidence.event_oracle_id.clone(),
        event_oracle_revision: evidence.event_oracle_revision.clone(),
        sample_count: n,
        collision_count,
        defensive_fraction: proposal.epsilon,
        weighted_contribution_cap: cap,
        estimate: mean,
        sample_variance: variance,
        standard_error,
        confidence_delta: delta,
        one_sided_confidence_level: 1.0 - delta,
        empirical_bernstein_radius: radius,
        upper_confidence_bound: upper,
        max_observed_likelihood_ratio: max_weight,
        likelihood_ratio_ess: weight_ess,
        collision_contribution_ess: event_ess,
        max_collision_contribution_share: max_event_share,
        inference_horizon: "fixed_sample_only_not_anytime_valid",
        claim_ceiling: "statistical_bound_conditional_on_iid_fixed_proposal_and_correct_target_event_model_not_physical_safety",
    })
}

fn validate_evidence(
    proposal: &DefensiveGaussianMixture,
    evidence: &EvaluationEvidence,
) -> Result<(), EstimateError> {
    if !valid_identifier(&evidence.proposal_id)
        || !valid_identifier(&evidence.proposal_training_id)
        || !valid_identifier(&evidence.evaluation_sample_id)
        || !valid_identifier(&evidence.sampling_design_id)
    {
        return Err(EstimateError::MissingSamplingEvidence);
    }
    if evidence.sampling_evidence_ref.trim().is_empty()
        || evidence.sample_order_digest.trim().is_empty()
    {
        return Err(EstimateError::MissingSamplingEvidence);
    }
    if evidence.proposal_id != proposal.id {
        return Err(EstimateError::EvidenceProposalMismatch);
    }
    if evidence.proposal_training_id == evidence.evaluation_sample_id {
        return Err(EstimateError::TrainingAndEvaluationMustBeDisjoint);
    }
    if !evidence.proposal_frozen_before_evaluation {
        return Err(EstimateError::ProposalNotFrozen);
    }
    if !evidence.evaluation_draws_are_iid_from_full_mixture {
        return Err(EstimateError::NonIidOrUnattestedSampling);
    }
    if !evidence.event_oracle_is_high_fidelity_continuous_time {
        return Err(EstimateError::LowFidelityEventOracleNotAccepted);
    }
    if !valid_identifier(&evidence.event_oracle_id)
        || !valid_identifier(&evidence.event_oracle_revision)
    {
        return Err(EstimateError::MissingEventOracleRevision);
    }
    Ok(())
}

fn validate_delta(delta: f64) -> Result<(), EstimateError> {
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(EstimateError::InvalidDelta(delta));
    }
    Ok(())
}

fn valid_identifier(value: &str) -> bool {
    !value.trim().is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"._:/-".contains(&byte))
}

fn is_unit_interval_open(value: f64) -> bool {
    value.is_finite() && (0.0..1.0).contains(&value)
}

fn log_standard_normal(point: &[f64]) -> f64 {
    -0.5 * point.len() as f64 * LOG_2PI - 0.5 * point.iter().map(|x| x * x).sum::<f64>()
}

fn log_gaussian_cholesky(
    point: &[f64],
    mean: &[f64],
    cholesky: &[Vec<f64>],
) -> Result<f64, EstimateError> {
    let dimension = point.len();
    if mean.len() != dimension
        || cholesky.len() != dimension
        || cholesky.iter().any(|row| row.len() != dimension)
    {
        return Err(EstimateError::InvalidCholeskyShape);
    }

    let mut solved = vec![0.0; dimension];
    let mut log_determinant_half = 0.0;
    for i in 0..dimension {
        let diagonal = cholesky[i][i];
        if !diagonal.is_finite() || diagonal <= 0.0 {
            return Err(EstimateError::CholeskyNonPositiveDiagonal);
        }
        let mut residual = point[i] - mean[i];
        for (j, solved_value) in solved.iter().enumerate().take(i) {
            residual -= cholesky[i][j] * solved_value;
        }
        solved[i] = residual / diagonal;
        log_determinant_half += diagonal.ln();
    }
    let quadratic = solved.iter().map(|x| x * x).sum::<f64>();
    Ok(-0.5 * dimension as f64 * LOG_2PI - log_determinant_half - 0.5 * quadratic)
}

fn affine_standard_normal(
    mean: &[f64],
    cholesky: &[Vec<f64>],
    standard_normal: &[f64],
) -> Result<Vec<f64>, EstimateError> {
    let dimension = mean.len();
    if standard_normal.len() != dimension
        || cholesky.len() != dimension
        || cholesky.iter().any(|row| row.len() != dimension)
    {
        return Err(EstimateError::InvalidCholeskyShape);
    }
    let mut point = mean.to_vec();
    for i in 0..dimension {
        for (j, noise) in standard_normal.iter().enumerate().take(i + 1) {
            point[i] += cholesky[i][j] * noise;
        }
    }
    if point.iter().any(|x| !x.is_finite()) {
        return Err(EstimateError::NonFiniteValue("transformed_sample"));
    }
    Ok(point)
}

fn log_sum_exp(values: &[f64]) -> f64 {
    let maximum = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if maximum == f64::NEG_INFINITY {
        return f64::NEG_INFINITY;
    }
    if maximum == f64::INFINITY {
        return f64::INFINITY;
    }
    let sum = values
        .iter()
        .map(|value| (value - maximum).exp())
        .sum::<f64>();
    maximum + sum.ln()
}

fn sample_variance(values: &[f64], mean: f64) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / (values.len() - 1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity(dimension: usize) -> Vec<Vec<f64>> {
        (0..dimension)
            .map(|i| {
                (0..dimension)
                    .map(|j| if i == j { 1.0 } else { 0.0 })
                    .collect()
            })
            .collect()
    }

    fn proposal() -> DefensiveGaussianMixture {
        DefensiveGaussianMixture {
            id: "proposal-gmm-v1".into(),
            epsilon: 0.1,
            dimension: 2,
            components: vec![
                GaussianComponent {
                    id: "collision-mode-left".into(),
                    mixture_weight: 0.5,
                    mean: vec![-2.0, 0.0],
                    cholesky: identity(2),
                },
                GaussianComponent {
                    id: "collision-mode-right".into(),
                    mixture_weight: 0.5,
                    mean: vec![2.0, 0.0],
                    cholesky: identity(2),
                },
            ],
        }
    }

    fn evidence() -> EvaluationEvidence {
        EvaluationEvidence {
            proposal_id: "proposal-gmm-v1".into(),
            proposal_training_id: "training-split-sha256-aaa".into(),
            evaluation_sample_id: "evaluation-split-sha256-bbb".into(),
            sampling_design_id: "iid-fixed-defensive-gmm-v1".into(),
            sampling_evidence_ref: "fixture://sampling-design/iid-v1".into(),
            sample_order_digest: "0123456789abcdef".into(),
            event_oracle_id: "high-fidelity-event-oracle".into(),
            event_oracle_revision: "sha256:deadbeef".into(),
            event_oracle_is_high_fidelity_continuous_time: true,
            proposal_frozen_before_evaluation: true,
            evaluation_draws_are_iid_from_full_mixture: true,
        }
    }

    #[test]
    fn nominal_support_component_bounds_all_likelihood_ratios() {
        let proposal = proposal();
        let cap = proposal.defensive_weight_cap().unwrap();
        for point in [
            vec![-100.0, -25.0],
            vec![-2.0, 0.5],
            vec![0.0, 0.0],
            vec![2.0, -0.5],
            vec![100.0, 25.0],
        ] {
            assert!(proposal.likelihood_ratio(&point).unwrap() <= cap * (1.0 + 1e-10));
        }
    }

    #[test]
    fn rejects_duplicate_component_ids_and_invalid_proposal_density_calls() {
        let mut proposal = proposal();
        proposal.components[1].id = proposal.components[0].id.clone();
        assert_eq!(
            proposal.validate(),
            Err(EstimateError::DuplicateComponentId(
                "collision-mode-left".into()
            ))
        );
        assert_eq!(
            proposal.log_proposal_density(&[0.0, 0.0]),
            Err(EstimateError::DuplicateComponentId(
                "collision-mode-left".into()
            ))
        );
    }

    #[test]
    fn identical_adaptive_and_nominal_density_gives_unit_weight() {
        let proposal = DefensiveGaussianMixture {
            id: "identical-proposal".into(),
            epsilon: 0.2,
            dimension: 2,
            components: vec![GaussianComponent {
                id: "same-as-target".into(),
                mixture_weight: 1.0,
                mean: vec![0.0, 0.0],
                cholesky: identity(2),
            }],
        };
        for point in [vec![0.0, 0.0], vec![1.0, -2.0], vec![20.0, 3.0]] {
            assert!((proposal.likelihood_ratio(&point).unwrap() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn sampler_uses_nominal_and_adaptive_components() {
        let proposal = proposal();
        let nominal = proposal
            .sample_from_latents(0.05, 0.8, &[1.0, -1.0])
            .unwrap();
        assert_eq!(
            nominal.generating_component_id,
            "nominal-defensive-component"
        );
        assert_eq!(nominal.point, vec![1.0, -1.0]);

        let adaptive = proposal.sample_from_latents(0.5, 0.8, &[0.0, 0.0]).unwrap();
        assert_eq!(adaptive.generating_component_id, "collision-mode-right");
        assert_eq!(adaptive.point, vec![2.0, 0.0]);
    }

    #[test]
    fn proposal_density_uses_full_mixture() {
        let proposal = proposal();
        let point = vec![2.0, 0.0];
        let log_p = proposal.log_target_density(&point).unwrap();
        let log_q = proposal.log_proposal_density(&point).unwrap();
        let log_w = proposal.log_likelihood_ratio(&point).unwrap();
        assert!((log_w - (log_p - log_q)).abs() < 1e-12);
        assert!(log_q > f64::NEG_INFINITY);
        assert!(log_w <= -proposal.epsilon.ln() + 1e-10);
    }

    #[test]
    fn empirical_estimate_and_fixed_sample_upper_bound_are_computed() {
        let proposal = DefensiveGaussianMixture {
            id: "proposal-nominal".into(),
            epsilon: 0.25,
            dimension: 2,
            components: vec![GaussianComponent {
                id: "same-as-target".into(),
                mixture_weight: 1.0,
                mean: vec![0.0, 0.0],
                cholesky: identity(2),
            }],
        };
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                collision: true,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                collision: false,
            },
            CollisionObservation {
                point: vec![0.0, 1.0],
                collision: true,
            },
            CollisionObservation {
                point: vec![-1.0, 0.0],
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.proposal_id = "proposal-nominal".into();
        let report =
            estimate_collision_probability(&proposal, &observations, 0.05, &evidence).unwrap();
        assert!((report.estimate - 0.5).abs() < 1e-12);
        assert_eq!(report.collision_count, 2);
        assert!(report.upper_confidence_bound >= report.estimate);
        assert!(report.upper_confidence_bound <= 1.0);
        assert_eq!(report.weighted_contribution_cap, 4.0);
        assert_eq!(
            report.inference_horizon,
            "fixed_sample_only_not_anytime_valid"
        );
    }

    #[test]
    fn zero_collisions_do_not_produce_zero_upper_bound() {
        let proposal = DefensiveGaussianMixture {
            id: "proposal-no-events".into(),
            epsilon: 0.2,
            dimension: 1,
            components: vec![GaussianComponent {
                id: "nominal".into(),
                mixture_weight: 1.0,
                mean: vec![0.0],
                cholesky: identity(1),
            }],
        };
        let observations = (0..100)
            .map(|i| CollisionObservation {
                point: vec![i as f64 / 10.0],
                collision: false,
            })
            .collect::<Vec<_>>();
        let mut evidence = evidence();
        evidence.proposal_id = proposal.id.clone();
        let report =
            estimate_collision_probability(&proposal, &observations, 0.05, &evidence).unwrap();
        assert_eq!(report.estimate, 0.0);
        assert!(report.upper_confidence_bound > 0.0);
    }

    #[test]
    fn refuses_training_evaluation_leakage() {
        let proposal = proposal();
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                collision: false,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.evaluation_sample_id = evidence.proposal_training_id.clone();
        assert_eq!(
            estimate_collision_probability(&proposal, &observations, 0.05, &evidence),
            Err(EstimateError::TrainingAndEvaluationMustBeDisjoint)
        );
    }

    #[test]
    fn refuses_low_fidelity_final_event_labels() {
        let proposal = proposal();
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                collision: false,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.event_oracle_is_high_fidelity_continuous_time = false;
        assert_eq!(
            estimate_collision_probability(&proposal, &observations, 0.05, &evidence),
            Err(EstimateError::LowFidelityEventOracleNotAccepted)
        );
    }

    #[test]
    fn rejects_unattested_evaluation_sampling() {
        let proposal = proposal();
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                collision: false,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.evaluation_draws_are_iid_from_full_mixture = false;
        assert_eq!(
            estimate_collision_probability(&proposal, &observations, 0.05, &evidence),
            Err(EstimateError::NonIidOrUnattestedSampling)
        );
    }

    #[test]
    fn rejects_malformed_cholesky_and_invalid_epsilon() {
        let mut malformed = proposal();
        malformed.components[0].cholesky[0][1] = 0.2;
        assert_eq!(
            malformed.validate(),
            Err(EstimateError::CholeskyNotLowerTriangular)
        );

        let mut invalid_epsilon = proposal();
        invalid_epsilon.epsilon = 0.0;
        assert_eq!(
            invalid_epsilon.validate(),
            Err(EstimateError::InvalidDefensiveFraction(0.0))
        );
    }

    #[test]
    fn rejects_insufficient_samples_and_bad_delta() {
        let proposal = proposal();
        let one = [CollisionObservation {
            point: vec![0.0, 0.0],
            collision: false,
        }];
        assert_eq!(
            estimate_collision_probability(&proposal, &one, 0.05, &evidence()),
            Err(EstimateError::InsufficientSamples(1))
        );
        assert_eq!(
            estimate_collision_probability(&proposal, &[], 1.0, &evidence()),
            Err(EstimateError::InvalidDelta(1.0))
        );
    }
}

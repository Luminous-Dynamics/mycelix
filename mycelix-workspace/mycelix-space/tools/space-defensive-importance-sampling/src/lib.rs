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

use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
};

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
pub struct WhiteningTransform {
    pub id: String,
    pub revision: String,
    pub state_frame_id: String,
    pub state_units_id: String,
    pub state_layout_id: String,
    pub state_epoch_id: String,
    pub time_scale_id: String,
    pub origin_id: String,
    pub mean: Vec<f64>,
    /// Lower-triangular L such that physical_state = mean + L * z.
    pub cholesky: Vec<Vec<f64>>,
}

impl WhiteningTransform {
    pub fn validate(&self, dimension: usize) -> Result<(), EstimateError> {
        for (field, value) in [
            ("whitening_transform_id", &self.id),
            ("whitening_transform_revision", &self.revision),
            ("state_frame_id", &self.state_frame_id),
            ("state_units_id", &self.state_units_id),
            ("state_layout_id", &self.state_layout_id),
            ("state_epoch_id", &self.state_epoch_id),
            ("time_scale_id", &self.time_scale_id),
            ("origin_id", &self.origin_id),
        ] {
            if !valid_identifier(value) {
                return Err(EstimateError::InvalidIdentifier(field));
            }
        }
        if self.mean.len() != dimension {
            return Err(EstimateError::DimensionMismatch {
                expected: dimension,
                actual: self.mean.len(),
                field: "whitening_transform_mean",
            });
        }
        if self.mean.iter().any(|value| !value.is_finite()) {
            return Err(EstimateError::NonFiniteValue("whitening_transform_mean"));
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
                    return Err(EstimateError::NonFiniteValue(
                        "whitening_transform_cholesky",
                    ));
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

    pub fn to_physical_state(&self, whitened_point: &[f64]) -> Result<Vec<f64>, EstimateError> {
        self.validate(self.mean.len())?;
        if whitened_point.len() != self.mean.len() {
            return Err(EstimateError::DimensionMismatch {
                expected: self.mean.len(),
                actual: whitened_point.len(),
                field: "whitened_point",
            });
        }
        if whitened_point.iter().any(|value| !value.is_finite()) {
            return Err(EstimateError::NonFiniteValue("whitened_point"));
        }
        affine_standard_normal(&self.mean, &self.cholesky, whitened_point)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionObservation {
    /// z-space draw used for the nominal/proposal density ratio.
    pub point: Vec<f64>,
    /// Exact physical state passed to the high-fidelity event oracle.
    pub physical_state: Vec<f64>,
    /// Output of the declared high-fidelity continuous-time event oracle.
    pub collision: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvaluationEvidence {
    pub proposal_id: String,
    pub whitening_transform_id: String,
    pub whitening_transform_revision: String,
    pub state_frame_id: String,
    pub state_units_id: String,
    pub state_layout_id: String,
    pub state_epoch_id: String,
    pub time_scale_id: String,
    pub origin_id: String,
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
    /// True only for exact mathematical benchmark oracles, not physical mission data.
    pub event_oracle_is_analytic_reference_fixture: bool,
    pub proposal_frozen_before_evaluation: bool,
    pub evaluation_draws_are_iid_from_full_mixture: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RepeatedEncounterObservation {
    /// One retained initial state sample for the entire assessment horizon.
    pub point: Vec<f64>,
    pub physical_state: Vec<f64>,
    /// Ordered high-fidelity event flags, one per EncounterDesign.encounter_ids entry.
    pub collision_by_encounter: Vec<bool>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RepeatedEncounterDesign {
    /// Fixed order of encounter IDs for every trajectory row.
    pub encounter_ids: Vec<String>,
    pub encounter_order_digest: String,
    pub horizon_id: String,
    /// The oracle evaluated the whole declared horizon, including between-epoch minima.
    pub event_oracle_evaluates_full_horizon: bool,
    /// All encounter flags in one row came from one shared trajectory sample.
    pub outcomes_share_one_trajectory_sample: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EncounterMarginalEstimate {
    pub encounter_id: String,
    pub collision_sample_count: usize,
    pub probability_estimate: f64,
    pub standard_error: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EncounterPairEstimate {
    pub encounter_a: String,
    pub encounter_b: String,
    pub cooccurrence_sample_count: usize,
    pub joint_probability_estimate: f64,
    /// E[X_a X_b] - E[X_a]E[X_b], estimated under the target distribution.
    pub target_event_covariance_estimate: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RepeatedEncounterRiskReport {
    pub method: &'static str,
    pub horizon_id: String,
    pub encounter_ids: Vec<String>,
    pub encounter_order_digest: String,
    /// Probability of at least one collision across the entire horizon.
    pub joint_path_union: ImportanceSamplingReport,
    pub per_encounter_marginals: Vec<EncounterMarginalEstimate>,
    pub pairwise_joint_estimates: Vec<EncounterPairEstimate>,
    /// Naive independence comparator, absent if marginal estimates fall outside [0,1].
    pub independent_events_union_approximation: Option<f64>,
    /// Joint union estimate minus the naive independence approximation.
    pub independence_approximation_gap: Option<f64>,
    pub sample_rows: usize,
    pub inference_scope: &'static str,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoundarySentinelObservation {
    pub point: Vec<f64>,
    pub physical_state: Vec<f64>,
    /// Frozen stratum label: e.g. inside, boundary-shell, outside, or mode ID.
    pub stratum_id: String,
    pub surrogate_collision: bool,
    pub high_fidelity_collision: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundarySentinelEvidence {
    pub sentinel_validation_id: String,
    pub proposal_training_id: String,
    pub proposal_evaluation_id: String,
    pub selection_design_id: String,
    pub selection_evidence_ref: String,
    pub sample_order_digest: String,
    pub low_fidelity_model_id: String,
    pub low_fidelity_model_revision: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub high_fidelity_oracle_is_continuous_time: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoundaryStratumReport {
    pub stratum_id: String,
    pub sample_count: usize,
    pub low_fidelity_collision_count: usize,
    pub high_fidelity_collision_count: usize,
    pub false_negative_count: usize,
    pub false_positive_count: usize,
    /// Undefined when no high-fidelity positives were observed.
    pub false_negative_fraction_among_high_fidelity_collisions: Option<f64>,
    /// Undefined when the low-fidelity map predicted no collisions.
    pub false_positive_fraction_among_low_fidelity_collisions: Option<f64>,
    pub status: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoundarySentinelReport {
    pub method: &'static str,
    pub sentinel_validation_id: String,
    pub selection_design_id: String,
    pub selection_evidence_ref: String,
    pub sample_order_digest: String,
    pub low_fidelity_model_id: String,
    pub low_fidelity_model_revision: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub whitening_transform_id: String,
    pub whitening_transform_revision: String,
    pub state_frame_id: String,
    pub state_units_id: String,
    pub state_layout_id: String,
    pub state_epoch_id: String,
    pub time_scale_id: String,
    pub origin_id: String,
    pub total_sentinels: usize,
    pub total_high_fidelity_collisions: usize,
    pub total_false_negatives: usize,
    pub leaking_strata: Vec<String>,
    pub strata: Vec<BoundaryStratumReport>,
    pub inference_scope: &'static str,
    pub claim_ceiling: &'static str,
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
    pub whitening_transform_id: String,
    pub whitening_transform_revision: String,
    pub state_frame_id: String,
    pub state_units_id: String,
    pub state_layout_id: String,
    pub state_epoch_id: String,
    pub time_scale_id: String,
    pub origin_id: String,
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
    InvalidAnalyticDiskRadius(f64),
    InsufficientSamples(usize),
    EvidenceProposalMismatch,
    WhiteningTransformMismatch,
    PhysicalStateMismatch,
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
    EmptyBoundarySentinelSet,
    MissingBoundarySentinelEvidence,
    SentinelValidationReusesTrainingOrEvaluation,
    LowFidelityBoundaryOracleNotAccepted,
    InvalidRepeatedEncounterDesign,
    RepeatedEncounterOutcomeCountMismatch {
        expected: usize,
        actual: usize,
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
            Self::InvalidAnalyticDiskRadius(value) => write!(
                f,
                "analytic disk radius must be finite and positive; got {value}"
            ),
            Self::InsufficientSamples(value) => write!(
                f,
                "at least two evaluation samples are required; got {value}"
            ),
            Self::EvidenceProposalMismatch => write!(
                f,
                "evaluation evidence proposal ID does not match the proposal object"
            ),
            Self::WhiteningTransformMismatch => write!(
                f,
                "evaluation evidence does not bind the exact whitening transform, frame and units"
            ),
            Self::PhysicalStateMismatch => write!(
                f,
                "high-fidelity event-oracle physical state does not match the declared whitening transform applied to the weighted sample"
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
            Self::EmptyBoundarySentinelSet => {
                write!(f, "at least one boundary sentinel is required")
            }
            Self::MissingBoundarySentinelEvidence => write!(
                f,
                "sentinel selection, sample-order and model references are required"
            ),
            Self::SentinelValidationReusesTrainingOrEvaluation => write!(
                f,
                "boundary sentinel validation must be independent of proposal training and final estimator evaluation"
            ),
            Self::LowFidelityBoundaryOracleNotAccepted => write!(
                f,
                "boundary sentinels require a high-fidelity continuous-time event oracle"
            ),
        }
    }
}

impl std::error::Error for EstimateError {}

/// Exact probability that a two-dimensional standard-normal random
/// vector falls inside a centered disk of radius r: 1 - exp(-r^2 / 2).
///
/// This is a mathematical benchmark oracle only, not an orbital collision
/// probability algorithm.
pub fn exact_standard_normal_2d_disk_probability(radius: f64) -> Result<f64, EstimateError> {
    if !radius.is_finite() || radius <= 0.0 {
        return Err(EstimateError::InvalidAnalyticDiskRadius(radius));
    }
    Ok(-(-0.5 * radius * radius).exp_m1())
}

/// Fixed-sample estimator for the probability of a high-fidelity event.
///
/// The one-sided empirical-Bernstein bound uses independent bounded
/// contributions in [0, 1/epsilon]. It is not valid after adaptive peeking or
/// optional stopping. The reported probability is conditional on the target,
/// proposal and event-oracle assumptions, not a physical-safety certification.
pub fn estimate_collision_probability(
    proposal: &DefensiveGaussianMixture,
    whitening: &WhiteningTransform,
    observations: &[CollisionObservation],
    delta: f64,
    evidence: &EvaluationEvidence,
) -> Result<ImportanceSamplingReport, EstimateError> {
    proposal.validate()?;
    whitening.validate(proposal.dimension)?;
    validate_delta(delta)?;
    validate_evidence(proposal, whitening, evidence)?;

    if observations.len() < 2 {
        return Err(EstimateError::InsufficientSamples(observations.len()));
    }

    let cap = 1.0 / proposal.epsilon;
    let mut contributions = Vec::with_capacity(observations.len());
    let mut weights = Vec::with_capacity(observations.len());
    let mut collision_count = 0usize;

    for observation in observations {
        let expected_physical_state = whitening.to_physical_state(&observation.point)?;
        if observation.physical_state.len() != proposal.dimension {
            return Err(EstimateError::DimensionMismatch {
                expected: proposal.dimension,
                actual: observation.physical_state.len(),
                field: "physical_state",
            });
        }
        if observation
            .physical_state
            .iter()
            .zip(expected_physical_state.iter())
            .any(|(actual, expected)| {
                (actual - expected).abs()
                    > NUMERIC_TOLERANCE * actual.abs().max(expected.abs()).max(1.0)
            })
        {
            return Err(EstimateError::PhysicalStateMismatch);
        }
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
        whitening_transform_id: whitening.id.clone(),
        whitening_transform_revision: whitening.revision.clone(),
        state_frame_id: whitening.state_frame_id.clone(),
        state_units_id: whitening.state_units_id.clone(),
        state_layout_id: whitening.state_layout_id.clone(),
        state_epoch_id: whitening.state_epoch_id.clone(),
        time_scale_id: whitening.time_scale_id.clone(),
        origin_id: whitening.origin_id.clone(),
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

/// Estimate the probability of at least one collision across a sequence of
/// dependent encounters. Every encounter flag for a row shares exactly one
/// initial uncertain state and one full-horizon high-fidelity event-oracle run.
///
/// The marginal estimates are diagnostics. The independent-events union
/// approximation is reported only as a comparator; it is never substituted
/// for the directly estimated shared-path union probability.
pub fn estimate_repeating_encounter_risk(
    proposal: &DefensiveGaussianMixture,
    whitening: &WhiteningTransform,
    observations: &[RepeatedEncounterObservation],
    delta: f64,
    evaluation_evidence: &EvaluationEvidence,
    design: &RepeatedEncounterDesign,
) -> Result<RepeatedEncounterRiskReport, EstimateError> {
    if design.encounter_ids.len() < 2
        || !valid_identifier(&design.horizon_id)
        || !valid_identifier(&design.encounter_order_digest)
        || !design.event_oracle_evaluates_full_horizon
        || !design.outcomes_share_one_trajectory_sample
    {
        return Err(EstimateError::InvalidRepeatedEncounterDesign);
    }
    let mut encounter_ids = BTreeSet::new();
    if design
        .encounter_ids
        .iter()
        .any(|id| !valid_identifier(id) || !encounter_ids.insert(id.as_str()))
    {
        return Err(EstimateError::InvalidRepeatedEncounterDesign);
    }
    if observations.len() < 2 {
        return Err(EstimateError::InsufficientSamples(observations.len()));
    }

    let encounter_count = design.encounter_ids.len();
    let mut union_observations = Vec::with_capacity(observations.len());
    let mut marginal_counts = vec![0usize; encounter_count];
    let mut marginal_contributions =
        vec![Vec::<f64>::with_capacity(observations.len()); encounter_count];
    let mut pair_counts = vec![vec![0usize; encounter_count]; encounter_count];
    let mut pair_contributions =
        vec![vec![Vec::<f64>::with_capacity(observations.len()); encounter_count]; encounter_count];

    for observation in observations {
        if observation.collision_by_encounter.len() != encounter_count {
            return Err(EstimateError::RepeatedEncounterOutcomeCountMismatch {
                expected: encounter_count,
                actual: observation.collision_by_encounter.len(),
            });
        }
        let weight = proposal.likelihood_ratio(&observation.point)?;
        let union_collision = observation.collision_by_encounter.iter().any(|hit| *hit);
        union_observations.push(CollisionObservation {
            point: observation.point.clone(),
            physical_state: observation.physical_state.clone(),
            collision: union_collision,
        });
        for a in 0..encounter_count {
            let hit_a = observation.collision_by_encounter[a];
            if hit_a {
                marginal_counts[a] += 1;
            }
            marginal_contributions[a].push(if hit_a { weight } else { 0.0 });
            for b in (a + 1)..encounter_count {
                let joint_hit = hit_a && observation.collision_by_encounter[b];
                if joint_hit {
                    pair_counts[a][b] += 1;
                }
                pair_contributions[a][b].push(if joint_hit { weight } else { 0.0 });
            }
        }
    }

    // This call enforces proposal/evaluation separation, whitening-to-physical
    // correspondence, the frozen proposal, IID sampling attestation and the
    // high-fidelity continuous-time oracle identity.
    let joint_path_union = estimate_collision_probability(
        proposal,
        whitening,
        &union_observations,
        delta,
        evaluation_evidence,
    )?;

    let n = observations.len();
    let per_encounter_marginals = design
        .encounter_ids
        .iter()
        .enumerate()
        .map(|(index, encounter_id)| {
            let values = &marginal_contributions[index];
            let estimate = values.iter().sum::<f64>() / n as f64;
            let variance = sample_variance(values, estimate);
            EncounterMarginalEstimate {
                encounter_id: encounter_id.clone(),
                collision_sample_count: marginal_counts[index],
                probability_estimate: estimate,
                standard_error: (variance / n as f64).sqrt(),
            }
        })
        .collect::<Vec<_>>();

    let mut pairwise_joint_estimates = Vec::new();
    for a in 0..encounter_count {
        for b in (a + 1)..encounter_count {
            let values = &pair_contributions[a][b];
            let joint_probability = values.iter().sum::<f64>() / n as f64;
            let marginal_a = per_encounter_marginals[a].probability_estimate;
            let marginal_b = per_encounter_marginals[b].probability_estimate;
            pairwise_joint_estimates.push(EncounterPairEstimate {
                encounter_a: design.encounter_ids[a].clone(),
                encounter_b: design.encounter_ids[b].clone(),
                cooccurrence_sample_count: pair_counts[a][b],
                joint_probability_estimate: joint_probability,
                target_event_covariance_estimate: joint_probability - marginal_a * marginal_b,
            });
        }
    }

    let independent_events_union_approximation = per_encounter_marginals
        .iter()
        .all(|marginal| {
            marginal.probability_estimate.is_finite()
                && (0.0..=1.0).contains(&marginal.probability_estimate)
        })
        .then(|| {
            1.0 - per_encounter_marginals
                .iter()
                .map(|marginal| 1.0 - marginal.probability_estimate)
                .product::<f64>()
        });
    let independence_approximation_gap = independent_events_union_approximation
        .map(|independent| joint_path_union.estimate - independent);

    Ok(RepeatedEncounterRiskReport {
        method: "shared_trajectory_weighted_joint_path_union",
        horizon_id: design.horizon_id.clone(),
        encounter_ids: design.encounter_ids.clone(),
        encounter_order_digest: design.encounter_order_digest.clone(),
        joint_path_union,
        per_encounter_marginals,
        pairwise_joint_estimates,
        independent_events_union_approximation,
        independence_approximation_gap,
        sample_rows: n,
        inference_scope: "joint_union_estimated_from_shared_trajectory_rows_no_independence_assumption_between_encounters",
        claim_ceiling: "fixed_sample_joint_path_statistical_bound_conditional_on_iid_frozen_proposal_and_correct_full_horizon_event_oracle_not_physical_safety",
    })
}

/// Compare a frozen low-fidelity collision classifier with a high-fidelity
/// continuous-time event oracle on an independently identified sentinel set.
///
/// The output is deliberately descriptive. Sentinel samples are often selected
/// near a learned boundary and therefore are not an IID sample from an
/// operational population. No population confidence interval is emitted.
pub fn assess_boundary_sentinels(
    whitening: &WhiteningTransform,
    observations: &[BoundarySentinelObservation],
    evidence: &BoundarySentinelEvidence,
) -> Result<BoundarySentinelReport, EstimateError> {
    whitening.validate(whitening.mean.len())?;
    if observations.is_empty() {
        return Err(EstimateError::EmptyBoundarySentinelSet);
    }
    if !valid_identifier(&evidence.sentinel_validation_id)
        || !valid_identifier(&evidence.proposal_training_id)
        || !valid_identifier(&evidence.proposal_evaluation_id)
        || !valid_identifier(&evidence.selection_design_id)
        || !valid_identifier(&evidence.low_fidelity_model_id)
        || !valid_identifier(&evidence.low_fidelity_model_revision)
        || !valid_identifier(&evidence.high_fidelity_event_oracle_id)
        || !valid_identifier(&evidence.high_fidelity_event_oracle_revision)
        || evidence.selection_evidence_ref.trim().is_empty()
        || evidence.sample_order_digest.trim().is_empty()
    {
        return Err(EstimateError::MissingBoundarySentinelEvidence);
    }
    if evidence.sentinel_validation_id == evidence.proposal_training_id
        || evidence.sentinel_validation_id == evidence.proposal_evaluation_id
    {
        return Err(EstimateError::SentinelValidationReusesTrainingOrEvaluation);
    }
    if !evidence.high_fidelity_oracle_is_continuous_time {
        return Err(EstimateError::LowFidelityBoundaryOracleNotAccepted);
    }

    #[derive(Default)]
    struct Counts {
        sample_count: usize,
        low_fidelity_collision_count: usize,
        high_fidelity_collision_count: usize,
        false_negative_count: usize,
        false_positive_count: usize,
    }

    let mut by_stratum = BTreeMap::<String, Counts>::new();
    let mut total_hf = 0usize;
    let mut total_fn = 0usize;

    for observation in observations {
        if !valid_identifier(&observation.stratum_id) {
            return Err(EstimateError::MissingBoundarySentinelEvidence);
        }
        if observation.physical_state.len() != whitening.mean.len() {
            return Err(EstimateError::DimensionMismatch {
                expected: whitening.mean.len(),
                actual: observation.physical_state.len(),
                field: "boundary_sentinel_physical_state",
            });
        }
        let expected_physical = whitening.to_physical_state(&observation.point)?;
        if observation
            .physical_state
            .iter()
            .zip(expected_physical.iter())
            .any(|(actual, expected)| {
                (actual - expected).abs()
                    > NUMERIC_TOLERANCE * actual.abs().max(expected.abs()).max(1.0)
            })
        {
            return Err(EstimateError::PhysicalStateMismatch);
        }

        let counts = by_stratum
            .entry(observation.stratum_id.clone())
            .or_default();
        counts.sample_count += 1;
        if observation.surrogate_collision {
            counts.low_fidelity_collision_count += 1;
        }
        if observation.high_fidelity_collision {
            counts.high_fidelity_collision_count += 1;
            total_hf += 1;
        }
        if !observation.surrogate_collision && observation.high_fidelity_collision {
            counts.false_negative_count += 1;
            total_fn += 1;
        }
        if observation.surrogate_collision && !observation.high_fidelity_collision {
            counts.false_positive_count += 1;
        }
    }

    let mut leaking_strata = Vec::new();
    let strata = by_stratum
        .into_iter()
        .map(|(stratum_id, counts)| {
            let false_negative_fraction = (counts.high_fidelity_collision_count > 0).then(|| {
                counts.false_negative_count as f64 / counts.high_fidelity_collision_count as f64
            });
            let false_positive_fraction = (counts.low_fidelity_collision_count > 0).then(|| {
                counts.false_positive_count as f64 / counts.low_fidelity_collision_count as f64
            });
            let status = if counts.false_negative_count > 0 {
                leaking_strata.push(stratum_id.clone());
                "BOUNDARY_LEAKAGE_DETECTED"
            } else if counts.high_fidelity_collision_count == 0 {
                "NO_HF_POSITIVE_DENOMINATOR"
            } else {
                "NO_FALSE_NEGATIVE_OBSERVED_IN_SENTINELS"
            };
            BoundaryStratumReport {
                stratum_id,
                sample_count: counts.sample_count,
                low_fidelity_collision_count: counts.low_fidelity_collision_count,
                high_fidelity_collision_count: counts.high_fidelity_collision_count,
                false_negative_count: counts.false_negative_count,
                false_positive_count: counts.false_positive_count,
                false_negative_fraction_among_high_fidelity_collisions: false_negative_fraction,
                false_positive_fraction_among_low_fidelity_collisions: false_positive_fraction,
                status,
            }
        })
        .collect();

    Ok(BoundarySentinelReport {
        method: "descriptive_high_fidelity_boundary_sentinel_comparison",
        sentinel_validation_id: evidence.sentinel_validation_id.clone(),
        selection_design_id: evidence.selection_design_id.clone(),
        selection_evidence_ref: evidence.selection_evidence_ref.clone(),
        sample_order_digest: evidence.sample_order_digest.clone(),
        low_fidelity_model_id: evidence.low_fidelity_model_id.clone(),
        low_fidelity_model_revision: evidence.low_fidelity_model_revision.clone(),
        high_fidelity_event_oracle_id: evidence.high_fidelity_event_oracle_id.clone(),
        high_fidelity_event_oracle_revision: evidence.high_fidelity_event_oracle_revision.clone(),
        whitening_transform_id: whitening.id.clone(),
        whitening_transform_revision: whitening.revision.clone(),
        state_frame_id: whitening.state_frame_id.clone(),
        state_units_id: whitening.state_units_id.clone(),
        state_layout_id: whitening.state_layout_id.clone(),
        state_epoch_id: whitening.state_epoch_id.clone(),
        time_scale_id: whitening.time_scale_id.clone(),
        origin_id: whitening.origin_id.clone(),
        total_sentinels: observations.len(),
        total_high_fidelity_collisions: total_hf,
        total_false_negatives: total_fn,
        leaking_strata,
        strata,
        inference_scope: "descriptive_only_no_population_confidence_interval_without_qualified_stratified_sampling_design",
        claim_ceiling: "agreement_with_declared_high_fidelity_reference_on_retained_sentinel_set_not_population_false_negative_probability_or_physical_truth",
    })
}

fn validate_evidence(
    proposal: &DefensiveGaussianMixture,
    whitening: &WhiteningTransform,
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
    if evidence.whitening_transform_id != whitening.id
        || evidence.whitening_transform_revision != whitening.revision
        || evidence.state_frame_id != whitening.state_frame_id
        || evidence.state_units_id != whitening.state_units_id
        || evidence.state_layout_id != whitening.state_layout_id
        || evidence.state_epoch_id != whitening.state_epoch_id
        || evidence.time_scale_id != whitening.time_scale_id
        || evidence.origin_id != whitening.origin_id
    {
        return Err(EstimateError::WhiteningTransformMismatch);
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
    if !evidence.event_oracle_is_high_fidelity_continuous_time
        && !evidence.event_oracle_is_analytic_reference_fixture
    {
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

    fn whitening(dimension: usize) -> WhiteningTransform {
        WhiteningTransform {
            id: "identity-whitening-v1".into(),
            revision: "sha256:whitener-identity".into(),
            state_frame_id: "fixture-frame-v1".into(),
            state_units_id: "fixture-units-v1".into(),
            state_layout_id: "primary-secondary-cartesian-state-v1".into(),
            state_epoch_id: "fixture-epoch-v1".into(),
            time_scale_id: "TAI-v1".into(),
            origin_id: "geocenter-v1".into(),
            mean: vec![0.0; dimension],
            cholesky: identity(dimension),
        }
    }

    fn boundary_evidence() -> BoundarySentinelEvidence {
        BoundarySentinelEvidence {
            sentinel_validation_id: "boundary-validation-independent-v1".into(),
            proposal_training_id: "training-split-sha256-aaa".into(),
            proposal_evaluation_id: "evaluation-split-sha256-bbb".into(),
            selection_design_id: "targeted-boundary-strata-v1".into(),
            selection_evidence_ref: "fixture://boundary-selection/design-v1".into(),
            sample_order_digest: "abcdef0123456789".into(),
            low_fidelity_model_id: "keplerian-boundary-map".into(),
            low_fidelity_model_revision: "sha256:low-fi-map-v1".into(),
            high_fidelity_event_oracle_id: "special-perturbations-event-oracle".into(),
            high_fidelity_event_oracle_revision: "sha256:hf-oracle-v1".into(),
            high_fidelity_oracle_is_continuous_time: true,
        }
    }

    fn repeated_nominal_proposal() -> DefensiveGaussianMixture {
        DefensiveGaussianMixture {
            id: "proposal-repeating-path".into(),
            epsilon: 0.2,
            dimension: 2,
            components: vec![GaussianComponent {
                id: "nominal-target".into(),
                mixture_weight: 1.0,
                mean: vec![0.0, 0.0],
                cholesky: identity(2),
            }],
        }
    }

    fn repeated_design() -> RepeatedEncounterDesign {
        RepeatedEncounterDesign {
            encounter_ids: vec!["encounter-A".into(), "encounter-B".into()],
            encounter_order_digest: "sha256:encounter-order-ab".into(),
            horizon_id: "horizon-24h-v1".into(),
            event_oracle_evaluates_full_horizon: true,
            outcomes_share_one_trajectory_sample: true,
        }
    }

    fn repeated_evidence(proposal_id: &str) -> EvaluationEvidence {
        let mut evidence = evidence();
        evidence.proposal_id = proposal_id.into();
        evidence
    }

    fn repeated_observations(patterns: &[[bool; 2]]) -> Vec<RepeatedEncounterObservation> {
        patterns
            .iter()
            .enumerate()
            .map(|(index, flags)| {
                let point = vec![index as f64 * 0.1, -(index as f64) * 0.05];
                RepeatedEncounterObservation {
                    point: point.clone(),
                    physical_state: point,
                    collision_by_encounter: flags.to_vec(),
                }
            })
            .collect()
    }

    fn evidence() -> EvaluationEvidence {
        EvaluationEvidence {
            proposal_id: "proposal-gmm-v1".into(),
            whitening_transform_id: "identity-whitening-v1".into(),
            whitening_transform_revision: "sha256:whitener-identity".into(),
            state_frame_id: "fixture-frame-v1".into(),
            state_units_id: "fixture-units-v1".into(),
            state_layout_id: "primary-secondary-cartesian-state-v1".into(),
            state_epoch_id: "fixture-epoch-v1".into(),
            time_scale_id: "TAI-v1".into(),
            origin_id: "geocenter-v1".into(),
            proposal_training_id: "training-split-sha256-aaa".into(),
            evaluation_sample_id: "evaluation-split-sha256-bbb".into(),
            sampling_design_id: "iid-fixed-defensive-gmm-v1".into(),
            sampling_evidence_ref: "fixture://sampling-design/iid-v1".into(),
            sample_order_digest: "0123456789abcdef".into(),
            event_oracle_id: "high-fidelity-event-oracle".into(),
            event_oracle_revision: "sha256:deadbeef".into(),
            event_oracle_is_high_fidelity_continuous_time: true,
            event_oracle_is_analytic_reference_fixture: false,
            proposal_frozen_before_evaluation: true,
            evaluation_draws_are_iid_from_full_mixture: true,
        }
    }

    #[test]
    fn whitening_transform_maps_exact_sample_into_declared_physical_frame() {
        let transform = WhiteningTransform {
            id: "orbital-whitener-v4".into(),
            revision: "sha256:whitener-v4".into(),
            state_frame_id: "TEME-v1".into(),
            state_units_id: "km-kmps-v1".into(),
            state_layout_id: "primary-secondary-state-v1".into(),
            state_epoch_id: "2026-01-01T00:00:00Z".into(),
            time_scale_id: "UTC-v1".into(),
            origin_id: "Earth-geocenter-v1".into(),
            mean: vec![100.0, 200.0],
            cholesky: vec![vec![2.0, 0.0], vec![0.5, 3.0]],
        };
        let physical = transform.to_physical_state(&[1.0, -2.0]).unwrap();
        assert!((physical[0] - 102.0).abs() < 1e-12);
        assert!((physical[1] - 194.5).abs() < 1e-12);
    }

    #[test]
    fn estimator_rejects_physical_event_state_not_matching_weighted_sample() {
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
        let transform = WhiteningTransform {
            id: "orbital-whitener-v4".into(),
            revision: "sha256:whitener-v4".into(),
            state_frame_id: "TEME-v1".into(),
            state_units_id: "km-kmps-v1".into(),
            state_layout_id: "primary-secondary-state-v1".into(),
            state_epoch_id: "2026-01-01T00:00:00Z".into(),
            time_scale_id: "UTC-v1".into(),
            origin_id: "Earth-geocenter-v1".into(),
            mean: vec![100.0, 200.0],
            cholesky: vec![vec![2.0, 0.0], vec![0.5, 3.0]],
        };
        let mut evidence = evidence();
        evidence.proposal_id = proposal.id.clone();
        evidence.whitening_transform_id = transform.id.clone();
        evidence.whitening_transform_revision = transform.revision.clone();
        evidence.state_frame_id = transform.state_frame_id.clone();
        evidence.state_units_id = transform.state_units_id.clone();
        evidence.state_layout_id = transform.state_layout_id.clone();
        evidence.state_epoch_id = transform.state_epoch_id.clone();
        evidence.time_scale_id = transform.time_scale_id.clone();
        evidence.origin_id = transform.origin_id.clone();
        let observations = vec![
            CollisionObservation {
                point: vec![1.0, -2.0],
                physical_state: vec![102.0, 194.5],
                collision: true,
            },
            CollisionObservation {
                point: vec![0.0, 0.0],
                physical_state: vec![100.0, 200.0],
                collision: false,
            },
        ];
        assert!(
            estimate_collision_probability(&proposal, &transform, &observations, 0.05, &evidence)
                .is_ok()
        );

        let mut corrupted = observations.clone();
        corrupted[0].physical_state[0] += 0.01;
        assert_eq!(
            estimate_collision_probability(&proposal, &transform, &corrupted, 0.05, &evidence),
            Err(EstimateError::PhysicalStateMismatch)
        );
    }

    #[test]
    fn estimator_rejects_whitening_transform_revision_mismatch() {
        let proposal = proposal();
        let transform = whitening(proposal.dimension);
        let mut evidence = evidence();
        evidence.whitening_transform_revision = "sha256:other-transform".into();
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                physical_state: vec![0.0, 0.0],
                collision: false,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                physical_state: vec![1.0, 0.0],
                collision: false,
            },
        ];
        assert_eq!(
            estimate_collision_probability(&proposal, &transform, &observations, 0.05, &evidence),
            Err(EstimateError::WhiteningTransformMismatch)
        );
    }

    #[test]
    fn perfectly_correlated_encounters_use_direct_shared_path_union() {
        let proposal = repeated_nominal_proposal();
        let observations =
            repeated_observations(&[[true, true], [true, true], [false, false], [false, false]]);
        let report = estimate_repeating_encounter_risk(
            &proposal,
            &whitening(2),
            &observations,
            0.05,
            &repeated_evidence(&proposal.id),
            &repeated_design(),
        )
        .unwrap();

        assert!((report.per_encounter_marginals[0].probability_estimate - 0.5).abs() < 1e-12);
        assert!((report.per_encounter_marginals[1].probability_estimate - 0.5).abs() < 1e-12);
        assert!((report.joint_path_union.estimate - 0.5).abs() < 1e-12);
        assert!((report.independent_events_union_approximation.unwrap() - 0.75).abs() < 1e-12);
        assert!((report.independence_approximation_gap.unwrap() + 0.25).abs() < 1e-12);
        assert!(
            (report.pairwise_joint_estimates[0].target_event_covariance_estimate - 0.25).abs()
                < 1e-12
        );
    }

    #[test]
    fn independent_encounter_fixture_matches_independence_comparator() {
        let proposal = repeated_nominal_proposal();
        let observations =
            repeated_observations(&[[true, true], [true, false], [false, true], [false, false]]);
        let report = estimate_repeating_encounter_risk(
            &proposal,
            &whitening(2),
            &observations,
            0.05,
            &repeated_evidence(&proposal.id),
            &repeated_design(),
        )
        .unwrap();
        assert!((report.joint_path_union.estimate - 0.75).abs() < 1e-12);
        assert!((report.independent_events_union_approximation.unwrap() - 0.75).abs() < 1e-12);
        assert!(report.independence_approximation_gap.unwrap().abs() < 1e-12);
        assert!(
            report.pairwise_joint_estimates[0]
                .target_event_covariance_estimate
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn mutually_exclusive_encounters_are_not_multiplied_as_independent() {
        let proposal = repeated_nominal_proposal();
        let observations =
            repeated_observations(&[[true, false], [false, true], [true, false], [false, true]]);
        let report = estimate_repeating_encounter_risk(
            &proposal,
            &whitening(2),
            &observations,
            0.05,
            &repeated_evidence(&proposal.id),
            &repeated_design(),
        )
        .unwrap();
        assert!((report.joint_path_union.estimate - 1.0).abs() < 1e-12);
        assert!((report.independent_events_union_approximation.unwrap() - 0.75).abs() < 1e-12);
        assert!((report.independence_approximation_gap.unwrap() - 0.25).abs() < 1e-12);
        assert!(
            (report.pairwise_joint_estimates[0].target_event_covariance_estimate + 0.25).abs()
                < 1e-12
        );
    }

    #[test]
    fn repeating_encounter_rejects_missing_flags_or_invalid_path_design() {
        let proposal = repeated_nominal_proposal();
        let mut observations = repeated_observations(&[[true, false], [false, true]]);
        observations[0].collision_by_encounter.pop();
        assert_eq!(
            estimate_repeating_encounter_risk(
                &proposal,
                &whitening(2),
                &observations,
                0.05,
                &repeated_evidence(&proposal.id),
                &repeated_design(),
            ),
            Err(EstimateError::RepeatedEncounterOutcomeCountMismatch {
                expected: 2,
                actual: 1
            })
        );

        let valid_observations = repeated_observations(&[[true, false], [false, true]]);
        let mut invalid_design = repeated_design();
        invalid_design.outcomes_share_one_trajectory_sample = false;
        assert_eq!(
            estimate_repeating_encounter_risk(
                &proposal,
                &whitening(2),
                &valid_observations,
                0.05,
                &repeated_evidence(&proposal.id),
                &invalid_design,
            ),
            Err(EstimateError::InvalidRepeatedEncounterDesign)
        );
    }

    #[test]
    fn boundary_sentinels_report_false_negative_leakage_by_stratum() {
        let observations = vec![
            BoundarySentinelObservation {
                point: vec![0.0, 0.0],
                physical_state: vec![0.0, 0.0],
                stratum_id: "inside".into(),
                surrogate_collision: true,
                high_fidelity_collision: true,
            },
            BoundarySentinelObservation {
                point: vec![1.0, 0.0],
                physical_state: vec![1.0, 0.0],
                stratum_id: "boundary-shell".into(),
                surrogate_collision: false,
                high_fidelity_collision: true,
            },
            BoundarySentinelObservation {
                point: vec![1.1, 0.0],
                physical_state: vec![1.1, 0.0],
                stratum_id: "boundary-shell".into(),
                surrogate_collision: false,
                high_fidelity_collision: false,
            },
            BoundarySentinelObservation {
                point: vec![2.0, 0.0],
                physical_state: vec![2.0, 0.0],
                stratum_id: "outside".into(),
                surrogate_collision: true,
                high_fidelity_collision: false,
            },
        ];
        let report =
            assess_boundary_sentinels(&whitening(2), &observations, &boundary_evidence()).unwrap();
        assert_eq!(report.total_sentinels, 4);
        assert_eq!(report.total_high_fidelity_collisions, 2);
        assert_eq!(report.total_false_negatives, 1);
        assert_eq!(report.leaking_strata, vec!["boundary-shell"]);
        let boundary = report
            .strata
            .iter()
            .find(|s| s.stratum_id == "boundary-shell")
            .unwrap();
        assert_eq!(boundary.false_negative_count, 1);
        assert_eq!(
            boundary.false_negative_fraction_among_high_fidelity_collisions,
            Some(1.0)
        );
        assert_eq!(boundary.status, "BOUNDARY_LEAKAGE_DETECTED");
        let outside = report
            .strata
            .iter()
            .find(|s| s.stratum_id == "outside")
            .unwrap();
        assert_eq!(
            outside.false_negative_fraction_among_high_fidelity_collisions,
            None
        );
        assert_eq!(outside.status, "NO_HF_POSITIVE_DENOMINATOR");
        assert_eq!(
            report.inference_scope,
            "descriptive_only_no_population_confidence_interval_without_qualified_stratified_sampling_design"
        );
    }

    #[test]
    fn boundary_sentinels_reject_reused_proposal_training_or_evaluation_set() {
        let observation = BoundarySentinelObservation {
            point: vec![0.0, 0.0],
            physical_state: vec![0.0, 0.0],
            stratum_id: "boundary-shell".into(),
            surrogate_collision: false,
            high_fidelity_collision: true,
        };
        let mut evidence = boundary_evidence();
        evidence.sentinel_validation_id = evidence.proposal_training_id.clone();
        assert_eq!(
            assess_boundary_sentinels(&whitening(2), &[observation], &evidence),
            Err(EstimateError::SentinelValidationReusesTrainingOrEvaluation)
        );
    }

    #[test]
    fn boundary_sentinels_reject_low_fidelity_oracle_as_reference() {
        let observation = BoundarySentinelObservation {
            point: vec![0.0, 0.0],
            physical_state: vec![0.0, 0.0],
            stratum_id: "inside".into(),
            surrogate_collision: true,
            high_fidelity_collision: true,
        };
        let mut evidence = boundary_evidence();
        evidence.high_fidelity_oracle_is_continuous_time = false;
        assert_eq!(
            assess_boundary_sentinels(&whitening(2), &[observation], &evidence),
            Err(EstimateError::LowFidelityBoundaryOracleNotAccepted)
        );
    }

    #[test]
    fn boundary_sentinels_reject_physical_state_transform_mismatch() {
        let observation = BoundarySentinelObservation {
            point: vec![0.0, 0.0],
            physical_state: vec![0.01, 0.0],
            stratum_id: "inside".into(),
            surrogate_collision: true,
            high_fidelity_collision: true,
        };
        assert_eq!(
            assess_boundary_sentinels(&whitening(2), &[observation], &boundary_evidence()),
            Err(EstimateError::PhysicalStateMismatch)
        );
    }

    #[test]
    fn exact_standard_normal_disk_probability_matches_closed_form() {
        let radius = (2.0_f64 * 0.5_f64.ln().abs()).sqrt();
        let probability = exact_standard_normal_2d_disk_probability(radius).unwrap();
        assert!((probability - 0.5).abs() < 1e-14);
        assert!(matches!(
            exact_standard_normal_2d_disk_probability(0.0),
            Err(EstimateError::InvalidAnalyticDiskRadius(0.0))
        ));
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
                physical_state: vec![0.0, 0.0].clone(),
                collision: true,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                physical_state: vec![1.0, 0.0].clone(),
                collision: false,
            },
            CollisionObservation {
                point: vec![0.0, 1.0],
                physical_state: vec![0.0, 1.0].clone(),
                collision: true,
            },
            CollisionObservation {
                point: vec![-1.0, 0.0],
                physical_state: vec![-1.0, 0.0].clone(),
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.proposal_id = "proposal-nominal".into();
        let report = estimate_collision_probability(
            &proposal,
            &whitening(proposal.dimension),
            &observations,
            0.05,
            &evidence,
        )
        .unwrap();
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
                physical_state: vec![i as f64 / 10.0].clone(),
                collision: false,
            })
            .collect::<Vec<_>>();
        let mut evidence = evidence();
        evidence.proposal_id = proposal.id.clone();
        let report = estimate_collision_probability(
            &proposal,
            &whitening(proposal.dimension),
            &observations,
            0.05,
            &evidence,
        )
        .unwrap();
        assert_eq!(report.estimate, 0.0);
        assert!(report.upper_confidence_bound > 0.0);
    }

    #[test]
    fn refuses_training_evaluation_leakage() {
        let proposal = proposal();
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                physical_state: vec![0.0, 0.0].clone(),
                collision: false,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                physical_state: vec![1.0, 0.0].clone(),
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.evaluation_sample_id = evidence.proposal_training_id.clone();
        assert_eq!(
            estimate_collision_probability(
                &proposal,
                &whitening(proposal.dimension),
                &observations,
                0.05,
                &evidence
            ),
            Err(EstimateError::TrainingAndEvaluationMustBeDisjoint)
        );
    }

    #[test]
    fn refuses_low_fidelity_final_event_labels() {
        let proposal = proposal();
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                physical_state: vec![0.0, 0.0].clone(),
                collision: false,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                physical_state: vec![1.0, 0.0].clone(),
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.event_oracle_is_high_fidelity_continuous_time = false;
        assert_eq!(
            estimate_collision_probability(
                &proposal,
                &whitening(proposal.dimension),
                &observations,
                0.05,
                &evidence
            ),
            Err(EstimateError::LowFidelityEventOracleNotAccepted)
        );
    }

    #[test]
    fn rejects_unattested_evaluation_sampling() {
        let proposal = proposal();
        let observations = vec![
            CollisionObservation {
                point: vec![0.0, 0.0],
                physical_state: vec![0.0, 0.0].clone(),
                collision: false,
            },
            CollisionObservation {
                point: vec![1.0, 0.0],
                physical_state: vec![1.0, 0.0].clone(),
                collision: false,
            },
        ];
        let mut evidence = evidence();
        evidence.evaluation_draws_are_iid_from_full_mixture = false;
        assert_eq!(
            estimate_collision_probability(
                &proposal,
                &whitening(proposal.dimension),
                &observations,
                0.05,
                &evidence
            ),
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
            physical_state: vec![0.0, 0.0].clone(),
            collision: false,
        }];
        assert_eq!(
            estimate_collision_probability(
                &proposal,
                &whitening(proposal.dimension),
                &one,
                0.05,
                &evidence()
            ),
            Err(EstimateError::InsufficientSamples(1))
        );
        assert_eq!(
            estimate_collision_probability(
                &proposal,
                &whitening(proposal.dimension),
                &[],
                1.0,
                &evidence()
            ),
            Err(EstimateError::InvalidDelta(1.0))
        );
    }
}

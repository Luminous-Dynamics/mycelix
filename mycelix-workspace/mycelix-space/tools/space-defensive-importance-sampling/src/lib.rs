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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CounterfactualManeuverDesign {
    pub baseline_plan_id: String,
    pub baseline_plan_revision: String,
    pub candidate_plan_id: String,
    pub candidate_plan_revision: String,
    pub horizon_id: String,
    pub encounter_ids: Vec<String>,
    pub encounter_order_digest: String,
    /// Both plans are evaluated from the same retained initial-state samples.
    pub same_initial_state_samples: bool,
    /// Both plans use the same full-horizon event oracle and revision.
    pub same_full_horizon_event_oracle: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CounterfactualManeuverObservation {
    /// One shared whitened initial-state sample for both plans.
    pub point: Vec<f64>,
    pub physical_state: Vec<f64>,
    /// Ordered baseline full-horizon encounter outcomes.
    pub baseline_collision_by_encounter: Vec<bool>,
    /// Ordered candidate full-horizon encounter outcomes.
    pub candidate_collision_by_encounter: Vec<bool>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CounterfactualEncounterChange {
    pub encounter_id: String,
    pub baseline_probability_estimate: f64,
    pub candidate_probability_estimate: f64,
    /// Candidate minus baseline; negative indicates lower estimated event probability.
    pub probability_change_estimate: f64,
    pub newly_introduced_collision_samples: usize,
    pub mitigated_collision_samples: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CounterfactualManeuverRiskReport {
    pub method: &'static str,
    pub baseline_plan_id: String,
    pub baseline_plan_revision: String,
    pub candidate_plan_id: String,
    pub candidate_plan_revision: String,
    pub horizon_id: String,
    pub encounter_ids: Vec<String>,
    pub encounter_order_digest: String,
    pub baseline_union: ImportanceSamplingReport,
    pub candidate_union: ImportanceSamplingReport,
    /// Candidate minus baseline; negative values favor the candidate.
    pub paired_probability_change: f64,
    pub paired_standard_error: f64,
    pub paired_confidence_delta: f64,
    pub paired_confidence_lower: f64,
    pub paired_confidence_upper: f64,
    pub independent_stream_difference_standard_error: f64,
    /// None when paired variance is exactly zero.
    pub paired_variance_reduction_ratio: Option<f64>,
    pub paired_variance_is_zero: bool,
    pub encounter_changes: Vec<CounterfactualEncounterChange>,
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
    /// Frozen set of strata that must be represented in the validation set.
    pub required_strata: Vec<String>,
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
    pub required_strata: Vec<String>,
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
pub struct MultifidelityResidualObservation {
    /// Unique stable identity in the frozen evaluation-sample ordering.
    pub sample_id: String,
    pub point: Vec<f64>,
    pub physical_state: Vec<f64>,
    /// Low-fidelity/surrogate event predicate evaluated at this exact sample.
    pub surrogate_collision: bool,
    /// High-fidelity continuous-time event predicate evaluated at this same sample.
    pub high_fidelity_collision: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MultifidelitySurrogateEvidence {
    pub surrogate_model_id: String,
    pub surrogate_model_revision: String,
    pub surrogate_training_id: String,
    pub surrogate_evaluation_sample_id: String,
    pub target_distribution_id: String,
    pub surrogate_probability_evidence_ref: String,
    pub surrogate_probability_artifact_digest: String,
    pub sample_order_digest: String,
    /// True only if this probability is exact under the declared nominal target.
    pub surrogate_probability_is_exact: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MultifidelityResidualReport {
    pub method: &'static str,
    pub proposal_id: String,
    pub surrogate_model_id: String,
    pub surrogate_model_revision: String,
    pub surrogate_training_id: String,
    pub evaluation_sample_id: String,
    pub sample_order_digest: String,
    pub event_oracle_id: String,
    pub event_oracle_revision: String,
    pub whitening_transform_id: String,
    pub whitening_transform_revision: String,
    pub sample_count: usize,
    pub high_fidelity_collision_count: usize,
    pub surrogate_collision_count: usize,
    pub false_negative_count: usize,
    pub false_positive_count: usize,
    pub defensive_fraction: f64,
    pub residual_contribution_cap: f64,
    pub surrogate_probability: f64,
    pub surrogate_probability_absolute_error_bound: f64,
    pub residual_mean: f64,
    pub residual_sample_variance: f64,
    pub residual_standard_error: f64,
    pub corrected_high_fidelity_probability_estimate: f64,
    pub direct_high_fidelity_importance_sampling_estimate: f64,
    pub direct_high_fidelity_sample_variance: f64,
    pub direct_high_fidelity_standard_error: f64,
    pub direct_to_residual_variance_ratio: Option<f64>,
    pub empirical_bernstein_radius: f64,
    pub total_confidence_radius: f64,
    pub confidence_delta: f64,
    pub two_sided_confidence_level: f64,
    pub confidence_lower: f64,
    pub confidence_upper: f64,
    pub max_absolute_residual_contribution: f64,
    pub max_absolute_residual_share: f64,
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
    InvalidSurrogateProbability(f64),
    InvalidSurrogateProbabilityErrorBound(f64),
    SurrogateProbabilityNotQualified,
    SurrogateTargetMismatch,
    SurrogateEvaluationSampleMismatch,
    SurrogateSampleOrderMismatch,
    SurrogateTrainingEvaluationLeakage,
    DuplicateEvaluationSampleId(String),
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
    MissingRequiredBoundaryStrata(Vec<String>),
    SentinelValidationReusesTrainingOrEvaluation,
    LowFidelityBoundaryOracleNotAccepted,
    InvalidRepeatedEncounterDesign,
    RepeatedEncounterOutcomeCountMismatch {
        expected: usize,
        actual: usize,
    },
    InvalidCounterfactualManeuverDesign,
    CounterfactualOutcomeCountMismatch {
        expected: usize,
        baseline_actual: usize,
        candidate_actual: usize,
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
            Self::InvalidSurrogateProbability(value) => write!(
                f,
                "surrogate probability must be finite and in [0,1]; got {value}"
            ),
            Self::InvalidSurrogateProbabilityErrorBound(value) => write!(
                f,
                "surrogate probability absolute error bound must be finite and in [0,1]; got {value}"
            ),
            Self::SurrogateProbabilityNotQualified => write!(
                f,
                "surrogate probability must be exact under the target or carry a positive independently evidenced absolute error bound"
            ),
            Self::SurrogateTargetMismatch => write!(
                f,
                "surrogate probability was not computed under the estimator's declared nominal target distribution"
            ),
            Self::SurrogateEvaluationSampleMismatch => write!(
                f,
                "surrogate labels and high-fidelity labels must use the same frozen evaluation sample identity"
            ),
            Self::SurrogateSampleOrderMismatch => write!(
                f,
                "surrogate and high-fidelity outcomes must bind the same sample-order digest"
            ),
            Self::SurrogateTrainingEvaluationLeakage => write!(
                f,
                "surrogate training samples must be disjoint from the final evaluation sample"
            ),
            Self::DuplicateEvaluationSampleId(value) => {
                write!(f, "duplicate evaluation sample ID {value}")
            }
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
            Self::MissingRequiredBoundaryStrata(missing) => write!(
                f,
                "boundary sentinel validation omitted required strata: {}",
                missing.join(", ")
            ),
            Self::SentinelValidationReusesTrainingOrEvaluation => write!(
                f,
                "boundary sentinel validation must be independent of proposal training and final estimator evaluation"
            ),
            Self::LowFidelityBoundaryOracleNotAccepted => write!(
                f,
                "boundary sentinels require a high-fidelity continuous-time event oracle"
            ),
            Self::InvalidRepeatedEncounterDesign => write!(
                f,
                "repeating encounter design must identify at least two unique encounters, a horizon, full-horizon oracle coverage and shared trajectory sampling"
            ),
            Self::RepeatedEncounterOutcomeCountMismatch { expected, actual } => write!(
                f,
                "trajectory row has {actual} encounter outcomes; expected {expected} in declared order"
            ),
            Self::InvalidCounterfactualManeuverDesign => write!(
                f,
                "counterfactual comparison requires distinct plan IDs/revisions, a fixed horizon and encounter order, and the same initial samples and full-horizon oracle"
            ),
            Self::CounterfactualOutcomeCountMismatch {
                expected,
                baseline_actual,
                candidate_actual,
            } => write!(
                f,
                "counterfactual row has baseline/candidate outcome counts {baseline_actual}/{candidate_actual}; expected {expected} for each"
            ),
        }
    }
}

impl std::error::Error for EstimateError {}

/// Estimate the high-fidelity event probability as an exactly known or
/// error-bounded surrogate probability plus a paired high-fidelity residual.
///
/// For I_H and I_L evaluated on the same sample X:
///
/// In notation, P_H equals P_L plus E_q[(p/q) * (I_H - I_L)].
///
/// Because the defensive proposal satisfies q >= epsilon*p, the residual
/// contribution lies in [-1/epsilon, +1/epsilon]. The interval is fixed-sample
/// and uses a two-sided empirical-Bernstein radius. A surrogate numerical
/// error bound is added directly to the interval radius.
pub fn estimate_multifidelity_residual_probability(
    proposal: &DefensiveGaussianMixture,
    whitening: &WhiteningTransform,
    observations: &[MultifidelityResidualObservation],
    surrogate_probability: f64,
    surrogate_probability_absolute_error_bound: f64,
    delta: f64,
    evaluation_evidence: &EvaluationEvidence,
    surrogate_evidence: &MultifidelitySurrogateEvidence,
) -> Result<MultifidelityResidualReport, EstimateError> {
    proposal.validate()?;
    whitening.validate(proposal.dimension)?;
    validate_delta(delta)?;
    validate_evidence(proposal, whitening, evaluation_evidence)?;

    if observations.len() < 2 {
        return Err(EstimateError::InsufficientSamples(observations.len()));
    }
    if !surrogate_probability.is_finite() || !(0.0..=1.0).contains(&surrogate_probability) {
        return Err(EstimateError::InvalidSurrogateProbability(
            surrogate_probability,
        ));
    }
    if !surrogate_probability_absolute_error_bound.is_finite()
        || !(0.0..=1.0).contains(&surrogate_probability_absolute_error_bound)
    {
        return Err(EstimateError::InvalidSurrogateProbabilityErrorBound(
            surrogate_probability_absolute_error_bound,
        ));
    }
    if surrogate_probability_is_invalid(
        surrogate_evidence.surrogate_probability_is_exact,
        surrogate_probability_absolute_error_bound,
    ) {
        return Err(EstimateError::SurrogateProbabilityNotQualified);
    }
    if surrogate_evidence.target_distribution_id != "whitened-standard-normal-N0I-v1" {
        return Err(EstimateError::SurrogateTargetMismatch);
    }
    if surrogate_evidence.surrogate_evaluation_sample_id != evaluation_evidence.evaluation_sample_id
    {
        return Err(EstimateError::SurrogateEvaluationSampleMismatch);
    }
    if surrogate_evidence.sample_order_digest != evaluation_evidence.sample_order_digest {
        return Err(EstimateError::SurrogateSampleOrderMismatch);
    }
    if surrogate_evidence.surrogate_training_id.trim().is_empty()
        || surrogate_evidence.surrogate_training_id == evaluation_evidence.evaluation_sample_id
        || surrogate_evidence.surrogate_training_id == evaluation_evidence.proposal_training_id
    {
        return Err(EstimateError::SurrogateTrainingEvaluationLeakage);
    }
    for (field, value) in [
        ("surrogate_model_id", &surrogate_evidence.surrogate_model_id),
        (
            "surrogate_model_revision",
            &surrogate_evidence.surrogate_model_revision,
        ),
        (
            "surrogate_probability_evidence_ref",
            &surrogate_evidence.surrogate_probability_evidence_ref,
        ),
        (
            "surrogate_probability_artifact_digest",
            &surrogate_evidence.surrogate_probability_artifact_digest,
        ),
    ] {
        if !valid_identifier(value) {
            return Err(EstimateError::InvalidIdentifier(field));
        }
    }

    let cap = 1.0 / proposal.epsilon;
    let mut seen_sample_ids = BTreeSet::new();
    let mut residuals = Vec::with_capacity(observations.len());
    let mut direct_contributions = Vec::with_capacity(observations.len());
    let mut hf_count = 0usize;
    let mut lf_count = 0usize;
    let mut false_negative_count = 0usize;
    let mut false_positive_count = 0usize;

    for observation in observations {
        if !valid_identifier(&observation.sample_id) {
            return Err(EstimateError::MissingSamplingEvidence);
        }
        if !seen_sample_ids.insert(observation.sample_id.as_str()) {
            return Err(EstimateError::DuplicateEvaluationSampleId(
                observation.sample_id.clone(),
            ));
        }
        if observation.physical_state.len() != proposal.dimension {
            return Err(EstimateError::DimensionMismatch {
                expected: proposal.dimension,
                actual: observation.physical_state.len(),
                field: "multifidelity_physical_state",
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

        let weight = proposal.likelihood_ratio(&observation.point)?;
        let hf = if observation.high_fidelity_collision {
            1.0
        } else {
            0.0
        };
        let lf = if observation.surrogate_collision {
            1.0
        } else {
            0.0
        };
        if observation.high_fidelity_collision {
            hf_count += 1;
        }
        if observation.surrogate_collision {
            lf_count += 1;
        }
        if observation.high_fidelity_collision && !observation.surrogate_collision {
            false_negative_count += 1;
        }
        if observation.surrogate_collision && !observation.high_fidelity_collision {
            false_positive_count += 1;
        }

        let direct = weight * hf;
        let residual = weight * (hf - lf);
        if direct > cap * (1.0 + NUMERIC_TOLERANCE)
            || residual.abs() > cap * (1.0 + NUMERIC_TOLERANCE)
        {
            return Err(EstimateError::DefensiveContributionCapViolated {
                contribution: residual.abs().max(direct),
                cap,
            });
        }
        direct_contributions.push(direct);
        residuals.push(residual);
    }

    let n = observations.len();
    let direct_mean = direct_contributions.iter().sum::<f64>() / n as f64;
    let direct_variance = sample_variance(&direct_contributions, direct_mean);
    let direct_standard_error = (direct_variance / n as f64).sqrt();

    let residual_mean = residuals.iter().sum::<f64>() / n as f64;
    let residual_variance = sample_variance(&residuals, residual_mean);
    let residual_standard_error = (residual_variance / n as f64).sqrt();
    let corrected_estimate = surrogate_probability + residual_mean;

    // Two-sided empirical-Bernstein interval: allocate delta/2 to each tail.
    // Residual support is [-cap, +cap], i.e. range width 2*cap.
    let log_term = (4.0 / delta).ln();
    let empirical_radius = (2.0 * residual_variance * log_term / n as f64).sqrt()
        + (14.0 * cap * log_term) / (3.0 * (n - 1) as f64);
    let total_radius = empirical_radius + surrogate_probability_absolute_error_bound;
    let lower = (corrected_estimate - total_radius).clamp(0.0, 1.0);
    let upper = (corrected_estimate + total_radius).clamp(0.0, 1.0);

    let direct_to_residual_variance_ratio = if residual_variance > 0.0 {
        Some(direct_variance / residual_variance)
    } else {
        None
    };
    let max_abs_residual = residuals
        .iter()
        .map(|value| value.abs())
        .fold(0.0, f64::max);
    let sum_abs_residual: f64 = residuals.iter().map(|value| value.abs()).sum();
    let max_abs_residual_share = if sum_abs_residual > 0.0 {
        max_abs_residual / sum_abs_residual
    } else {
        0.0
    };

    Ok(MultifidelityResidualReport {
        method: "paired_multifidelity_importance_sampling_residual",
        proposal_id: proposal.id.clone(),
        surrogate_model_id: surrogate_evidence.surrogate_model_id.clone(),
        surrogate_model_revision: surrogate_evidence.surrogate_model_revision.clone(),
        surrogate_training_id: surrogate_evidence.surrogate_training_id.clone(),
        evaluation_sample_id: evaluation_evidence.evaluation_sample_id.clone(),
        sample_order_digest: evaluation_evidence.sample_order_digest.clone(),
        event_oracle_id: evaluation_evidence.event_oracle_id.clone(),
        event_oracle_revision: evaluation_evidence.event_oracle_revision.clone(),
        whitening_transform_id: whitening.id.clone(),
        whitening_transform_revision: whitening.revision.clone(),
        sample_count: n,
        high_fidelity_collision_count: hf_count,
        surrogate_collision_count: lf_count,
        false_negative_count,
        false_positive_count,
        defensive_fraction: proposal.epsilon,
        residual_contribution_cap: cap,
        surrogate_probability,
        surrogate_probability_absolute_error_bound,
        residual_mean,
        residual_sample_variance: residual_variance,
        residual_standard_error,
        corrected_high_fidelity_probability_estimate: corrected_estimate,
        direct_high_fidelity_importance_sampling_estimate: direct_mean,
        direct_high_fidelity_sample_variance: direct_variance,
        direct_high_fidelity_standard_error: direct_standard_error,
        direct_to_residual_variance_ratio,
        empirical_bernstein_radius: empirical_radius,
        total_confidence_radius: total_radius,
        confidence_delta: delta,
        two_sided_confidence_level: 1.0 - delta,
        confidence_lower: lower,
        confidence_upper: upper,
        max_absolute_residual_contribution: max_abs_residual,
        max_absolute_residual_share: max_abs_residual_share,
        inference_horizon: "fixed_sample_only_not_anytime_valid",
        claim_ceiling: "paired_multifidelity_statistical_bound_conditional_on_exact_or_error_bounded_surrogate_probability_frozen_proposal_iid_evaluation_and_correct_high_fidelity_event_model",
    })
}

fn surrogate_probability_is_invalid(is_exact: bool, absolute_error_bound: f64) -> bool {
    if is_exact {
        absolute_error_bound != 0.0
    } else {
        absolute_error_bound <= 0.0
    }
}

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

/// Paired baseline-versus-candidate maneuver risk comparison using the
/// same initial-state rows, frozen proposal weights and full-horizon oracle.
///
/// The risk difference is candidate minus baseline. Its weighted contribution
/// lies in [-1/epsilon, +1/epsilon], so a fixed-sample empirical-Bernstein
/// interval can be computed without assuming independence between the two
/// plan outcomes on the same trajectory row.
pub fn compare_counterfactual_maneuver_risk(
    proposal: &DefensiveGaussianMixture,
    whitening: &WhiteningTransform,
    observations: &[CounterfactualManeuverObservation],
    delta: f64,
    evaluation_evidence: &EvaluationEvidence,
    design: &CounterfactualManeuverDesign,
) -> Result<CounterfactualManeuverRiskReport, EstimateError> {
    if !valid_identifier(&design.baseline_plan_id)
        || !valid_identifier(&design.baseline_plan_revision)
        || !valid_identifier(&design.candidate_plan_id)
        || !valid_identifier(&design.candidate_plan_revision)
        || design.baseline_plan_id == design.candidate_plan_id
        || design.baseline_plan_revision == design.candidate_plan_revision
        || !valid_identifier(&design.horizon_id)
        || !valid_identifier(&design.encounter_order_digest)
        || design.encounter_ids.is_empty()
        || !design.same_initial_state_samples
        || !design.same_full_horizon_event_oracle
    {
        return Err(EstimateError::InvalidCounterfactualManeuverDesign);
    }
    let mut seen_ids = BTreeSet::new();
    if design
        .encounter_ids
        .iter()
        .any(|id| !valid_identifier(id) || !seen_ids.insert(id.as_str()))
    {
        return Err(EstimateError::InvalidCounterfactualManeuverDesign);
    }
    if observations.len() < 2 {
        return Err(EstimateError::InsufficientSamples(observations.len()));
    }

    let encounter_count = design.encounter_ids.len();
    let mut baseline_observations = Vec::with_capacity(observations.len());
    let mut candidate_observations = Vec::with_capacity(observations.len());
    let mut paired_differences = Vec::with_capacity(observations.len());
    let mut encounter_differences =
        vec![Vec::<f64>::with_capacity(observations.len()); encounter_count];
    let mut encounter_baseline_contributions =
        vec![Vec::<f64>::with_capacity(observations.len()); encounter_count];
    let mut introduced_counts = vec![0usize; encounter_count];
    let mut mitigated_counts = vec![0usize; encounter_count];

    for observation in observations {
        if observation.baseline_collision_by_encounter.len() != encounter_count
            || observation.candidate_collision_by_encounter.len() != encounter_count
        {
            return Err(EstimateError::CounterfactualOutcomeCountMismatch {
                expected: encounter_count,
                baseline_actual: observation.baseline_collision_by_encounter.len(),
                candidate_actual: observation.candidate_collision_by_encounter.len(),
            });
        }
        let baseline_union = observation
            .baseline_collision_by_encounter
            .iter()
            .any(|hit| *hit);
        let candidate_union = observation
            .candidate_collision_by_encounter
            .iter()
            .any(|hit| *hit);
        baseline_observations.push(CollisionObservation {
            point: observation.point.clone(),
            physical_state: observation.physical_state.clone(),
            collision: baseline_union,
        });
        candidate_observations.push(CollisionObservation {
            point: observation.point.clone(),
            physical_state: observation.physical_state.clone(),
            collision: candidate_union,
        });

        let weight = proposal.likelihood_ratio(&observation.point)?;
        let baseline_union_value = if baseline_union { 1.0 } else { 0.0 };
        let candidate_union_value = if candidate_union { 1.0 } else { 0.0 };
        paired_differences.push(weight * (candidate_union_value - baseline_union_value));
        for index in 0..encounter_count {
            let baseline_hit = observation.baseline_collision_by_encounter[index];
            let candidate_hit = observation.candidate_collision_by_encounter[index];
            let baseline_value = if baseline_hit { 1.0 } else { 0.0 };
            let candidate_value = if candidate_hit { 1.0 } else { 0.0 };
            encounter_baseline_contributions[index].push(weight * baseline_value);
            encounter_differences[index].push(weight * (candidate_value - baseline_value));
            if candidate_hit && !baseline_hit {
                introduced_counts[index] += 1;
            }
            if baseline_hit && !candidate_hit {
                mitigated_counts[index] += 1;
            }
        }
    }

    let baseline_union = estimate_collision_probability(
        proposal,
        whitening,
        &baseline_observations,
        delta,
        evaluation_evidence,
    )?;
    let candidate_union = estimate_collision_probability(
        proposal,
        whitening,
        &candidate_observations,
        delta,
        evaluation_evidence,
    )?;

    let n = paired_differences.len();
    let paired_change = paired_differences.iter().sum::<f64>() / n as f64;
    let paired_variance = sample_variance(&paired_differences, paired_change);
    let paired_standard_error = (paired_variance / n as f64).sqrt();

    // Two-sided confidence interval: allocate delta/2 to each tail and use
    // the known contribution range [-cap,+cap], whose width is 2*cap.
    let cap = 1.0 / proposal.epsilon;
    let log_term = (4.0 / delta).ln();
    let radius = (2.0 * paired_variance * log_term / n as f64).sqrt()
        + (14.0 * cap * log_term) / (3.0 * (n - 1) as f64);
    let lower = (paired_change - radius).max(-1.0);
    let upper = (paired_change + radius).min(1.0);

    let independent_stream_difference_standard_error = (baseline_union.sample_variance / n as f64
        + candidate_union.sample_variance / n as f64)
        .sqrt();
    let paired_variance_is_zero = paired_variance <= f64::EPSILON;
    let paired_variance_reduction_ratio = if paired_variance_is_zero {
        None
    } else {
        Some(independent_stream_difference_standard_error.powi(2) / paired_standard_error.powi(2))
    };

    let encounter_changes = design
        .encounter_ids
        .iter()
        .enumerate()
        .map(|(index, encounter_id)| {
            let baseline_estimate =
                encounter_baseline_contributions[index].iter().sum::<f64>() / n as f64;
            let difference = encounter_differences[index].iter().sum::<f64>() / n as f64;
            CounterfactualEncounterChange {
                encounter_id: encounter_id.clone(),
                baseline_probability_estimate: baseline_estimate,
                candidate_probability_estimate: baseline_estimate + difference,
                probability_change_estimate: difference,
                newly_introduced_collision_samples: introduced_counts[index],
                mitigated_collision_samples: mitigated_counts[index],
            }
        })
        .collect::<Vec<_>>();

    Ok(CounterfactualManeuverRiskReport {
        method: "paired_shared_trajectory_counterfactual_risk_comparison",
        baseline_plan_id: design.baseline_plan_id.clone(),
        baseline_plan_revision: design.baseline_plan_revision.clone(),
        candidate_plan_id: design.candidate_plan_id.clone(),
        candidate_plan_revision: design.candidate_plan_revision.clone(),
        horizon_id: design.horizon_id.clone(),
        encounter_ids: design.encounter_ids.clone(),
        encounter_order_digest: design.encounter_order_digest.clone(),
        baseline_union,
        candidate_union,
        paired_probability_change: paired_change,
        paired_standard_error,
        paired_confidence_delta: delta,
        paired_confidence_lower: lower,
        paired_confidence_upper: upper,
        independent_stream_difference_standard_error,
        paired_variance_reduction_ratio,
        paired_variance_is_zero,
        encounter_changes,
        sample_rows: n,
        inference_scope: "paired_fixed_sample_difference_under_same_frozen_proposal_and_initial_state_rows",
        claim_ceiling: "counterfactual_statistical_comparison_only_not_maneuver_feasibility_authorization_or_safety_certification",
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
        || evidence.required_strata.is_empty()
        || evidence
            .required_strata
            .iter()
            .any(|id| !valid_identifier(id))
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

    let mut required = BTreeSet::new();
    if evidence
        .required_strata
        .iter()
        .any(|id| !required.insert(id.as_str()))
    {
        return Err(EstimateError::MissingBoundarySentinelEvidence);
    }
    let missing_required_strata = evidence
        .required_strata
        .iter()
        .filter(|stratum| !by_stratum.contains_key(*stratum))
        .cloned()
        .collect::<Vec<_>>();
    if !missing_required_strata.is_empty() {
        return Err(EstimateError::MissingRequiredBoundaryStrata(
            missing_required_strata,
        ));
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
        required_strata: evidence.required_strata.clone(),
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
            required_strata: vec!["inside".into(), "boundary-shell".into(), "outside".into()],
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

    fn counterfactual_design() -> CounterfactualManeuverDesign {
        CounterfactualManeuverDesign {
            baseline_plan_id: "baseline-plan".into(),
            baseline_plan_revision: "sha256:baseline-v1".into(),
            candidate_plan_id: "candidate-plan".into(),
            candidate_plan_revision: "sha256:candidate-v1".into(),
            horizon_id: "horizon-24h-v1".into(),
            encounter_ids: vec!["encounter-A".into(), "encounter-B".into()],
            encounter_order_digest: "sha256:encounter-order-ab".into(),
            same_initial_state_samples: true,
            same_full_horizon_event_oracle: true,
        }
    }

    fn counterfactual_observations(
        patterns: &[([bool; 2], [bool; 2])],
    ) -> Vec<CounterfactualManeuverObservation> {
        patterns
            .iter()
            .enumerate()
            .map(|(index, (baseline, candidate))| {
                let point = vec![index as f64 * 0.1, -(index as f64) * 0.05];
                CounterfactualManeuverObservation {
                    point: point.clone(),
                    physical_state: point,
                    baseline_collision_by_encounter: baseline.to_vec(),
                    candidate_collision_by_encounter: candidate.to_vec(),
                }
            })
            .collect()
    }

    fn multifidelity_proposal() -> DefensiveGaussianMixture {
        DefensiveGaussianMixture {
            id: "proposal-multifidelity-exact-target".into(),
            epsilon: 0.25,
            dimension: 2,
            components: vec![GaussianComponent {
                id: "nominal-standard-normal".into(),
                mixture_weight: 1.0,
                mean: vec![0.0, 0.0],
                cholesky: identity(2),
            }],
        }
    }

    fn multifidelity_evaluation_evidence() -> EvaluationEvidence {
        let mut value = evidence();
        value.proposal_id = "proposal-multifidelity-exact-target".into();
        value.event_oracle_id = "analytic-high-fidelity-disk-oracle".into();
        value.event_oracle_revision = "closed-form-hf-disk-v1".into();
        value.event_oracle_is_high_fidelity_continuous_time = false;
        value.event_oracle_is_analytic_reference_fixture = true;
        value
    }

    fn multifidelity_surrogate_evidence(
        evaluation: &EvaluationEvidence,
        is_exact: bool,
    ) -> MultifidelitySurrogateEvidence {
        MultifidelitySurrogateEvidence {
            surrogate_model_id: "analytic-low-fidelity-disk".into(),
            surrogate_model_revision: "closed-form-lf-disk-v1".into(),
            surrogate_training_id: "surrogate-training-independent-v1".into(),
            surrogate_evaluation_sample_id: evaluation.evaluation_sample_id.clone(),
            target_distribution_id: "whitened-standard-normal-N0I-v1".into(),
            surrogate_probability_evidence_ref: "fixture://surrogate-probability/radius-0.5-v1"
                .into(),
            surrogate_probability_artifact_digest: "sha256:surrogate-pc-fixture".into(),
            sample_order_digest: evaluation.sample_order_digest.clone(),
            surrogate_probability_is_exact: is_exact,
        }
    }

    fn multifidelity_observations(
        high_fidelity_radius: f64,
        surrogate_radius: f64,
    ) -> Vec<MultifidelityResidualObservation> {
        [0.1_f64, 0.6, 0.9, 1.3]
            .into_iter()
            .enumerate()
            .map(|(index, x)| {
                let point = vec![x, 0.0];
                let collision_by_radius = |radius: f64| x * x <= radius * radius;
                MultifidelityResidualObservation {
                    sample_id: format!("evaluation-row-{index:04}"),
                    point: point.clone(),
                    physical_state: point,
                    surrogate_collision: collision_by_radius(surrogate_radius),
                    high_fidelity_collision: collision_by_radius(high_fidelity_radius),
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
    fn multifidelity_residual_corrects_surrogate_probability_with_paired_disagreement() {
        let proposal = multifidelity_proposal();
        let whitening = whitening(2);
        let evaluation = multifidelity_evaluation_evidence();
        let surrogate = multifidelity_surrogate_evidence(&evaluation, true);
        let surrogate_pc = exact_standard_normal_2d_disk_probability(0.5).unwrap();
        let observations = multifidelity_observations(0.75, 0.5);

        let report = estimate_multifidelity_residual_probability(
            &proposal,
            &whitening,
            &observations,
            surrogate_pc,
            0.0,
            0.05,
            &evaluation,
            &surrogate,
        )
        .unwrap();

        assert_eq!(report.high_fidelity_collision_count, 2);
        assert_eq!(report.surrogate_collision_count, 1);
        assert_eq!(report.false_negative_count, 1);
        assert_eq!(report.false_positive_count, 0);
        assert!((report.direct_high_fidelity_importance_sampling_estimate - 0.5).abs() < 1e-12);
        assert!((report.residual_mean - 0.25).abs() < 1e-12);
        assert!(
            (report.corrected_high_fidelity_probability_estimate - (surrogate_pc + 0.25)).abs()
                < 1e-12
        );
        assert!(report.confidence_lower <= report.corrected_high_fidelity_probability_estimate);
        assert!(report.confidence_upper >= report.corrected_high_fidelity_probability_estimate);
        assert!(report.direct_to_residual_variance_ratio.unwrap() > 1.0);
        assert_eq!(
            report.inference_horizon,
            "fixed_sample_only_not_anytime_valid"
        );
    }

    #[test]
    fn multifidelity_residual_preserves_negative_corrections_for_surrogate_false_positives() {
        let proposal = multifidelity_proposal();
        let whitening = whitening(2);
        let evaluation = multifidelity_evaluation_evidence();
        let surrogate = multifidelity_surrogate_evidence(&evaluation, true);
        let surrogate_pc = exact_standard_normal_2d_disk_probability(0.75).unwrap();
        let observations = multifidelity_observations(0.5, 0.75);

        let report = estimate_multifidelity_residual_probability(
            &proposal,
            &whitening,
            &observations,
            surrogate_pc,
            0.0,
            0.05,
            &evaluation,
            &surrogate,
        )
        .unwrap();

        assert_eq!(report.high_fidelity_collision_count, 1);
        assert_eq!(report.surrogate_collision_count, 3);
        assert_eq!(report.false_negative_count, 0);
        assert_eq!(report.false_positive_count, 2);
        assert!((report.residual_mean + 0.5).abs() < 1e-12);
        assert!(
            (report.corrected_high_fidelity_probability_estimate - (surrogate_pc - 0.5)).abs()
                < 1e-12
        );
    }

    #[test]
    fn multifidelity_residual_widens_interval_by_surrogate_probability_error() {
        let proposal = multifidelity_proposal();
        let whitening = whitening(2);
        let evaluation = multifidelity_evaluation_evidence();
        let surrogate = multifidelity_surrogate_evidence(&evaluation, false);
        let surrogate_pc = exact_standard_normal_2d_disk_probability(0.5).unwrap();
        let observations = multifidelity_observations(0.75, 0.5);

        let report = estimate_multifidelity_residual_probability(
            &proposal,
            &whitening,
            &observations,
            surrogate_pc,
            0.01,
            0.05,
            &evaluation,
            &surrogate,
        )
        .unwrap();

        assert!(
            (report.total_confidence_radius - report.empirical_bernstein_radius - 0.01).abs()
                < 1e-12
        );
    }

    #[test]
    fn multifidelity_residual_rejects_unqualified_or_mismatched_surrogate_probability() {
        let proposal = multifidelity_proposal();
        let whitening = whitening(2);
        let evaluation = multifidelity_evaluation_evidence();
        let observations = multifidelity_observations(0.75, 0.5);
        let pc = exact_standard_normal_2d_disk_probability(0.5).unwrap();

        let mut missing_error = multifidelity_surrogate_evidence(&evaluation, false);
        assert_eq!(
            estimate_multifidelity_residual_probability(
                &proposal,
                &whitening,
                &observations,
                pc,
                0.0,
                0.05,
                &evaluation,
                &missing_error,
            ),
            Err(EstimateError::SurrogateProbabilityNotQualified)
        );

        let mut wrong_target = multifidelity_surrogate_evidence(&evaluation, true);
        wrong_target.target_distribution_id = "different-target-measure-v1".into();
        assert_eq!(
            estimate_multifidelity_residual_probability(
                &proposal,
                &whitening,
                &observations,
                pc,
                0.0,
                0.05,
                &evaluation,
                &wrong_target,
            ),
            Err(EstimateError::SurrogateTargetMismatch)
        );

        let mut wrong_order = multifidelity_surrogate_evidence(&evaluation, true);
        wrong_order.sample_order_digest = "sha256:different-sample-order".into();
        assert_eq!(
            estimate_multifidelity_residual_probability(
                &proposal,
                &whitening,
                &observations,
                pc,
                0.0,
                0.05,
                &evaluation,
                &wrong_order,
            ),
            Err(EstimateError::SurrogateSampleOrderMismatch)
        );

        let mut training_leak = multifidelity_surrogate_evidence(&evaluation, true);
        training_leak.surrogate_training_id = evaluation.evaluation_sample_id.clone();
        assert_eq!(
            estimate_multifidelity_residual_probability(
                &proposal,
                &whitening,
                &observations,
                pc,
                0.0,
                0.05,
                &evaluation,
                &training_leak,
            ),
            Err(EstimateError::SurrogateTrainingEvaluationLeakage)
        );
    }

    #[test]
    fn multifidelity_residual_rejects_duplicate_rows_and_physical_state_mismatch() {
        let proposal = multifidelity_proposal();
        let whitening = whitening(2);
        let evaluation = multifidelity_evaluation_evidence();
        let surrogate = multifidelity_surrogate_evidence(&evaluation, true);
        let pc = exact_standard_normal_2d_disk_probability(0.5).unwrap();
        let mut observations = multifidelity_observations(0.75, 0.5);

        observations[1].sample_id = observations[0].sample_id.clone();
        assert_eq!(
            estimate_multifidelity_residual_probability(
                &proposal,
                &whitening,
                &observations,
                pc,
                0.0,
                0.05,
                &evaluation,
                &surrogate,
            ),
            Err(EstimateError::DuplicateEvaluationSampleId(
                "evaluation-row-0000".into()
            ))
        );

        let mut observations = multifidelity_observations(0.75, 0.5);
        observations[1].physical_state[0] += 0.01;
        assert_eq!(
            estimate_multifidelity_residual_probability(
                &proposal,
                &whitening,
                &observations,
                pc,
                0.0,
                0.05,
                &evaluation,
                &surrogate,
            ),
            Err(EstimateError::PhysicalStateMismatch)
        );
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
    fn identical_maneuver_outcomes_have_zero_paired_variance() {
        let proposal = repeated_nominal_proposal();
        let observations = counterfactual_observations(&[
            ([true, false], [true, false]),
            ([false, false], [false, false]),
            ([true, true], [true, true]),
            ([false, false], [false, false]),
        ]);
        let report = compare_counterfactual_maneuver_risk(
            &proposal,
            &whitening(2),
            &observations,
            0.05,
            &repeated_evidence(&proposal.id),
            &counterfactual_design(),
        )
        .unwrap();

        assert!(report.paired_probability_change.abs() < 1e-12);
        assert!(report.paired_standard_error.abs() < 1e-12);
        assert!(report.paired_variance_is_zero);
        assert!(report.paired_variance_reduction_ratio.is_none());
        assert!(report.independent_stream_difference_standard_error > 0.0);
        assert_eq!(
            report.baseline_union.estimate,
            report.candidate_union.estimate
        );
    }

    #[test]
    fn paired_counterfactual_estimates_candidate_risk_reduction() {
        let proposal = repeated_nominal_proposal();
        let observations = counterfactual_observations(&[
            ([true, false], [false, false]),
            ([true, false], [true, false]),
            ([false, false], [false, false]),
            ([false, false], [false, false]),
        ]);
        let report = compare_counterfactual_maneuver_risk(
            &proposal,
            &whitening(2),
            &observations,
            0.05,
            &repeated_evidence(&proposal.id),
            &counterfactual_design(),
        )
        .unwrap();

        assert!((report.baseline_union.estimate - 0.5).abs() < 1e-12);
        assert!((report.candidate_union.estimate - 0.25).abs() < 1e-12);
        assert!((report.paired_probability_change + 0.25).abs() < 1e-12);
        assert_eq!(report.encounter_changes[0].mitigated_collision_samples, 1);
        assert_eq!(
            report.encounter_changes[0].newly_introduced_collision_samples,
            0
        );
    }

    #[test]
    fn paired_counterfactual_exposes_collateral_encounter_introduction() {
        let proposal = repeated_nominal_proposal();
        let observations = counterfactual_observations(&[
            ([true, false], [false, true]),
            ([true, false], [true, false]),
            ([false, false], [false, false]),
            ([false, false], [false, false]),
        ]);
        let report = compare_counterfactual_maneuver_risk(
            &proposal,
            &whitening(2),
            &observations,
            0.05,
            &repeated_evidence(&proposal.id),
            &counterfactual_design(),
        )
        .unwrap();

        assert_eq!(report.encounter_changes[0].mitigated_collision_samples, 1);
        assert_eq!(
            report.encounter_changes[1].newly_introduced_collision_samples,
            1
        );
        assert!(report.encounter_changes[0].probability_change_estimate < 0.0);
        assert!(report.encounter_changes[1].probability_change_estimate > 0.0);
        assert!(report.paired_probability_change.abs() < 1e-12);
    }

    #[test]
    fn paired_counterfactual_rejects_unpaired_design_and_missing_outcomes() {
        let proposal = repeated_nominal_proposal();
        let mut design = counterfactual_design();
        design.same_initial_state_samples = false;
        let observations = counterfactual_observations(&[
            ([false, false], [false, false]),
            ([true, false], [false, false]),
        ]);
        assert_eq!(
            compare_counterfactual_maneuver_risk(
                &proposal,
                &whitening(2),
                &observations,
                0.05,
                &repeated_evidence(&proposal.id),
                &design,
            ),
            Err(EstimateError::InvalidCounterfactualManeuverDesign)
        );

        let mut malformed = observations.clone();
        malformed[0].candidate_collision_by_encounter.pop();
        assert_eq!(
            compare_counterfactual_maneuver_risk(
                &proposal,
                &whitening(2),
                &malformed,
                0.05,
                &repeated_evidence(&proposal.id),
                &counterfactual_design(),
            ),
            Err(EstimateError::CounterfactualOutcomeCountMismatch {
                expected: 2,
                baseline_actual: 2,
                candidate_actual: 1,
            })
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
    fn boundary_sentinels_reject_missing_required_strata() {
        let observations = vec![BoundarySentinelObservation {
            point: vec![0.0, 0.0],
            physical_state: vec![0.0, 0.0],
            stratum_id: "inside".into(),
            surrogate_collision: true,
            high_fidelity_collision: true,
        }];
        assert_eq!(
            assess_boundary_sentinels(&whitening(2), &observations, &boundary_evidence()),
            Err(EstimateError::MissingRequiredBoundaryStrata(vec![
                "boundary-shell".into(),
                "outside".into(),
            ]))
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

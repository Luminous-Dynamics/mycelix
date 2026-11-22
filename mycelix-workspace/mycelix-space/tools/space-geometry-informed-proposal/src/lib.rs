#![forbid(unsafe_code)]

//! Defensive geometry-informed importance sampling for Gaussian rare events.
//!
//! The nominal target is p(z)=N(0,I_d). Each geometric component uses an
//! orthonormal frame: tangent coordinates are standard Gaussian and the
//! signed normal coordinate is threshold + Exponential(rate). This is a
//! normalized flat-boundary GeoIS analogue, not the complete curved-boundary
//! algorithm in the literature.
//!
//! The final proposal is q(z)=epsilon*p(z)+(1-epsilon)*q_geo(z). The nominal
//! defensive component ensures q>=epsilon*p and p/q<=1/epsilon even when
//! q_geo misses an event mode. Every weight uses the density of the entire
//! mixture, including all geometric modes and the nominal component.

use std::{collections::BTreeSet, fmt};

const LOG_2PI: f64 = 1.837_877_066_409_345_3;
const TOL: f64 = 2.0e-10;

#[derive(Clone, Debug, PartialEq)]
pub struct GeometryMode {
    pub id: String,
    pub mixture_weight: f64,
    pub normal: Vec<f64>,
    pub tangent_basis: Vec<Vec<f64>>,
    pub threshold: f64,
    pub exponential_rate: f64,
}

impl GeometryMode {
    fn validate(&self, dimension: usize) -> Result<(), ProposalError> {
        if !valid_identifier(&self.id) {
            return Err(ProposalError::InvalidIdentifier("geometry_mode_id"));
        }
        if !self.mixture_weight.is_finite() || self.mixture_weight <= 0.0 {
            return Err(ProposalError::InvalidMixtureWeight(self.mixture_weight));
        }
        if self.normal.len() != dimension || self.normal.iter().any(|x| !x.is_finite()) {
            return Err(ProposalError::InvalidFrameDimension);
        }
        if self.tangent_basis.len() + 1 != dimension
            || self
                .tangent_basis
                .iter()
                .any(|v| v.len() != dimension || v.iter().any(|x| !x.is_finite()))
        {
            return Err(ProposalError::InvalidFrameDimension);
        }
        if !self.threshold.is_finite() {
            return Err(ProposalError::NonFiniteParameter("threshold"));
        }
        if !self.exponential_rate.is_finite() || self.exponential_rate <= 0.0 {
            return Err(ProposalError::InvalidExponentialRate(self.exponential_rate));
        }
        if (dot(&self.normal, &self.normal) - 1.0).abs() > TOL {
            return Err(ProposalError::FrameNotOrthonormal);
        }
        for tangent in &self.tangent_basis {
            if (dot(tangent, tangent) - 1.0).abs() > TOL || dot(&self.normal, tangent).abs() > TOL {
                return Err(ProposalError::FrameNotOrthonormal);
            }
        }
        for i in 0..self.tangent_basis.len() {
            for j in (i + 1)..self.tangent_basis.len() {
                if dot(&self.tangent_basis[i], &self.tangent_basis[j]).abs() > TOL {
                    return Err(ProposalError::FrameNotOrthonormal);
                }
            }
        }
        Ok(())
    }

    fn log_density(&self, point: &[f64]) -> f64 {
        let s = dot(&self.normal, point);
        if s < self.threshold {
            return f64::NEG_INFINITY;
        }
        let tangent = self
            .tangent_basis
            .iter()
            .map(|basis| dot(basis, point))
            .collect::<Vec<_>>();
        let log_phi = -0.5 * tangent.len() as f64 * LOG_2PI
            - 0.5 * tangent.iter().map(|x| x * x).sum::<f64>();
        log_phi + self.exponential_rate.ln() - self.exponential_rate * (s - self.threshold)
    }

    fn sample(&self, tangent_normals: &[f64], exponential_uniform: f64) -> Vec<f64> {
        let s = self.threshold - exponential_uniform.ln() / self.exponential_rate;
        let mut point = self.normal.iter().map(|x| x * s).collect::<Vec<_>>();
        for (basis, coordinate) in self.tangent_basis.iter().zip(tangent_normals.iter()) {
            for (value, direction) in point.iter_mut().zip(basis.iter()) {
                *value += direction * coordinate;
            }
        }
        point
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct DefensiveGeometryProposal {
    pub id: String,
    pub dimension: usize,
    pub epsilon: f64,
    pub modes: Vec<GeometryMode>,
}

impl DefensiveGeometryProposal {
    pub fn validate(&self) -> Result<(), ProposalError> {
        if !valid_identifier(&self.id) {
            return Err(ProposalError::InvalidIdentifier("proposal_id"));
        }
        if self.dimension == 0 {
            return Err(ProposalError::ZeroDimension);
        }
        if !self.epsilon.is_finite()
            || self.epsilon <= 0.0
            || self.epsilon >= 1.0
            || !(1.0 / self.epsilon).is_finite()
        {
            return Err(ProposalError::InvalidDefensiveFraction(self.epsilon));
        }
        if self.modes.is_empty() {
            return Err(ProposalError::EmptyGeometryMixture);
        }
        let mut ids = BTreeSet::new();
        let mut total = 0.0;
        for mode in &self.modes {
            mode.validate(self.dimension)?;
            if !ids.insert(mode.id.as_str()) {
                return Err(ProposalError::DuplicateModeId(mode.id.clone()));
            }
            total += mode.mixture_weight;
        }
        if !total.is_finite() || (total - 1.0).abs() > TOL {
            return Err(ProposalError::GeometryWeightsDoNotSumToOne(total));
        }
        Ok(())
    }

    pub fn contribution_cap(&self) -> Result<f64, ProposalError> {
        self.validate()?;
        Ok(1.0 / self.epsilon)
    }

    pub fn log_target_density(&self, point: &[f64]) -> Result<f64, ProposalError> {
        self.validate_point(point)?;
        Ok(log_standard_normal(point))
    }

    pub fn log_geometry_density(&self, point: &[f64]) -> Result<f64, ProposalError> {
        self.validate_point(point)?;
        let terms = self
            .modes
            .iter()
            .map(|mode| mode.mixture_weight.ln() + mode.log_density(point))
            .collect::<Vec<_>>();
        Ok(log_sum_exp(&terms))
    }

    pub fn log_proposal_density(&self, point: &[f64]) -> Result<f64, ProposalError> {
        self.validate()?;
        self.validate_point(point)?;
        let log_p = log_standard_normal(point);
        let log_geo = self.log_geometry_density(point)?;
        Ok(log_sum_exp(&[
            self.epsilon.ln() + log_p,
            (1.0 - self.epsilon).ln() + log_geo,
        ]))
    }

    pub fn log_likelihood_ratio(&self, point: &[f64]) -> Result<f64, ProposalError> {
        let log_weight = self.log_target_density(point)? - self.log_proposal_density(point)?;
        let log_cap = -self.epsilon.ln();
        if log_weight > log_cap + TOL {
            return Err(ProposalError::DefensiveWeightCapViolated {
                log_weight,
                log_cap,
            });
        }
        Ok(log_weight)
    }

    pub fn likelihood_ratio(&self, point: &[f64]) -> Result<f64, ProposalError> {
        Ok(self.log_likelihood_ratio(point)?.exp())
    }

    /// Exact analytical event predicate: union of the declared half-spaces.
    /// Physical adapters must use a versioned high-fidelity event oracle.
    pub fn is_event(&self, point: &[f64]) -> Result<bool, ProposalError> {
        self.validate_point(point)?;
        Ok(self
            .modes
            .iter()
            .any(|mode| dot(&mode.normal, point) >= mode.threshold))
    }

    /// Deterministically map externally generated independent latent variates
    /// into a sample. RNG selection and independence are caller obligations.
    pub fn sample_from_latents(
        &self,
        mixture_gate: f64,
        mode_selector: f64,
        nominal_normals: &[f64],
        tangent_normals: &[f64],
        exponential_uniform: f64,
    ) -> Result<GeometrySample, ProposalError> {
        self.validate()?;
        self.validate_point(nominal_normals)?;
        if tangent_normals.len() + 1 != self.dimension
            || tangent_normals.iter().any(|x| !x.is_finite())
        {
            return Err(ProposalError::InvalidFrameDimension);
        }
        if !unit_open(mixture_gate) || !unit_open(mode_selector) || !unit_open(exponential_uniform)
        {
            return Err(ProposalError::UniformVariateOutOfRange);
        }
        if mixture_gate < self.epsilon {
            return Ok(GeometrySample {
                point: nominal_normals.to_vec(),
                generating_component_id: "nominal-defensive-component".into(),
            });
        }

        let mut cumulative = 0.0;
        let mut selected = self.modes.len() - 1;
        for (index, mode) in self.modes.iter().enumerate() {
            cumulative += mode.mixture_weight;
            if mode_selector < cumulative {
                selected = index;
                break;
            }
        }
        let mode = &self.modes[selected];
        Ok(GeometrySample {
            point: mode.sample(tangent_normals, exponential_uniform),
            generating_component_id: mode.id.clone(),
        })
    }

    fn validate_point(&self, point: &[f64]) -> Result<(), ProposalError> {
        if point.len() != self.dimension {
            return Err(ProposalError::DimensionMismatch {
                expected: self.dimension,
                actual: point.len(),
            });
        }
        if point.iter().any(|x| !x.is_finite()) {
            return Err(ProposalError::NonFinitePoint);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeometrySample {
    pub point: Vec<f64>,
    pub generating_component_id: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeometryObservation {
    pub sample_id: String,
    pub point: Vec<f64>,
    pub generating_component_id: String,
    pub event: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvaluationEvidence {
    pub proposal_training_id: String,
    pub evaluation_sample_id: String,
    pub sampling_design_id: String,
    pub sampling_evidence_ref: String,
    pub sample_order_digest: String,
    pub proposal_frozen_before_evaluation: bool,
    pub evaluation_draws_are_iid_from_full_mixture: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeometryEstimateReport {
    pub method: &'static str,
    pub proposal_id: String,
    pub sample_count: usize,
    pub event_count: usize,
    pub defensive_fraction: f64,
    pub weighted_contribution_cap: f64,
    pub estimate: f64,
    pub sample_variance: f64,
    pub standard_error: f64,
    pub confidence_delta: f64,
    pub one_sided_confidence_level: f64,
    pub empirical_bernstein_radius: f64,
    pub upper_confidence_bound: f64,
    pub max_likelihood_ratio: f64,
    pub likelihood_ratio_ess: f64,
    pub event_contribution_ess: f64,
    pub max_event_contribution_share: f64,
    pub generating_mode_counts: Vec<(String, usize)>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ProposalError {
    InvalidIdentifier(&'static str),
    ZeroDimension,
    InvalidDefensiveFraction(f64),
    EmptyGeometryMixture,
    DuplicateModeId(String),
    InvalidMixtureWeight(f64),
    GeometryWeightsDoNotSumToOne(f64),
    InvalidFrameDimension,
    FrameNotOrthonormal,
    NonFiniteParameter(&'static str),
    InvalidExponentialRate(f64),
    DimensionMismatch {
        expected: usize,
        actual: usize,
    },
    NonFinitePoint,
    UniformVariateOutOfRange,
    DefensiveWeightCapViolated {
        log_weight: f64,
        log_cap: f64,
    },
    InvalidDelta(f64),
    InsufficientSamples(usize),
    InvalidEvaluationEvidence,
    TrainingEvaluationLeakage,
    ProposalNotFrozen,
    NonIidEvaluation,
    MissingSampleId,
    DuplicateSampleId(String),
    UnknownGeneratingComponent(String),
    EventLabelMismatch {
        sample_id: String,
        supplied: bool,
        computed: bool,
    },
}

impl fmt::Display for ProposalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIdentifier(field) => {
                write!(f, "{field} must be a non-empty safe identifier")
            }
            Self::ZeroDimension => write!(f, "proposal dimension must be positive"),
            Self::InvalidDefensiveFraction(value) => {
                write!(f, "epsilon must be finite and in (0,1); got {value}")
            }
            Self::EmptyGeometryMixture => {
                write!(f, "at least one geometric boundary mode is required")
            }
            Self::DuplicateModeId(value) => write!(f, "duplicate geometric mode ID {value}"),
            Self::InvalidMixtureWeight(value) => write!(
                f,
                "geometric mode weight must be finite and positive; got {value}"
            ),
            Self::GeometryWeightsDoNotSumToOne(value) => {
                write!(f, "geometry weights sum to {value}, not 1")
            }
            Self::InvalidFrameDimension => {
                write!(f, "normal/tangent frame dimensions do not match")
            }
            Self::FrameNotOrthonormal => write!(
                f,
                "normal and tangent vectors must form an orthonormal frame"
            ),
            Self::NonFiniteParameter(field) => write!(f, "{field} must be finite"),
            Self::InvalidExponentialRate(value) => write!(
                f,
                "exponential rate must be finite and positive; got {value}"
            ),
            Self::DimensionMismatch { expected, actual } => write!(
                f,
                "point dimension mismatch: expected {expected}, got {actual}"
            ),
            Self::NonFinitePoint => write!(f, "point contains non-finite coordinates"),
            Self::UniformVariateOutOfRange => write!(f, "uniform variates must lie in (0,1)"),
            Self::DefensiveWeightCapViolated {
                log_weight,
                log_cap,
            } => write!(f, "likelihood ratio exceeds cap: {log_weight} > {log_cap}"),
            Self::InvalidDelta(value) => {
                write!(f, "delta must be finite and in (0,1); got {value}")
            }
            Self::InsufficientSamples(value) => {
                write!(f, "at least two observations are required; got {value}")
            }
            Self::InvalidEvaluationEvidence => {
                write!(f, "sampling evidence is missing or malformed")
            }
            Self::TrainingEvaluationLeakage => write!(f, "training and evaluation IDs must differ"),
            Self::ProposalNotFrozen => write!(f, "proposal must be frozen before evaluation"),
            Self::NonIidEvaluation => write!(
                f,
                "fixed-sample inference requires IID draws from the full proposal"
            ),
            Self::MissingSampleId => write!(f, "each sample needs a stable identifier"),
            Self::DuplicateSampleId(value) => write!(f, "duplicate sample ID {value}"),
            Self::UnknownGeneratingComponent(value) => {
                write!(f, "unknown generating component {value}")
            }
            Self::EventLabelMismatch {
                sample_id,
                supplied,
                computed,
            } => write!(
                f,
                "event label mismatch for {sample_id}: supplied={supplied}, computed={computed}"
            ),
        }
    }
}
impl std::error::Error for ProposalError {}

pub fn estimate_geometry_event_probability(
    proposal: &DefensiveGeometryProposal,
    observations: &[GeometryObservation],
    delta: f64,
    evidence: &EvaluationEvidence,
) -> Result<GeometryEstimateReport, ProposalError> {
    proposal.validate()?;
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(ProposalError::InvalidDelta(delta));
    }
    validate_evidence(evidence)?;
    if observations.len() < 2 {
        return Err(ProposalError::InsufficientSamples(observations.len()));
    }
    if evidence.proposal_training_id == evidence.evaluation_sample_id {
        return Err(ProposalError::TrainingEvaluationLeakage);
    }
    if !evidence.proposal_frozen_before_evaluation {
        return Err(ProposalError::ProposalNotFrozen);
    }
    if !evidence.evaluation_draws_are_iid_from_full_mixture {
        return Err(ProposalError::NonIidEvaluation);
    }

    let cap = 1.0 / proposal.epsilon;
    let mut contributions = Vec::with_capacity(observations.len());
    let mut weights = Vec::with_capacity(observations.len());
    let mut ids = BTreeSet::new();
    let mut mode_counts = proposal
        .modes
        .iter()
        .map(|m| (m.id.clone(), 0usize))
        .collect::<Vec<_>>();
    mode_counts.push(("nominal-defensive-component".into(), 0));

    for observation in observations {
        if !valid_identifier(&observation.sample_id) {
            return Err(ProposalError::MissingSampleId);
        }
        if !ids.insert(observation.sample_id.as_str()) {
            return Err(ProposalError::DuplicateSampleId(
                observation.sample_id.clone(),
            ));
        }
        let computed_event = proposal.is_event(&observation.point)?;
        if computed_event != observation.event {
            return Err(ProposalError::EventLabelMismatch {
                sample_id: observation.sample_id.clone(),
                supplied: observation.event,
                computed: computed_event,
            });
        }
        if let Some((_, count)) = mode_counts
            .iter_mut()
            .find(|(id, _)| id == &observation.generating_component_id)
        {
            *count += 1;
        } else {
            return Err(ProposalError::UnknownGeneratingComponent(
                observation.generating_component_id.clone(),
            ));
        }

        let weight = proposal.likelihood_ratio(&observation.point)?;
        let contribution = if observation.event { weight } else { 0.0 };
        if contribution > cap * (1.0 + TOL) {
            return Err(ProposalError::DefensiveWeightCapViolated {
                log_weight: contribution.ln(),
                log_cap: cap.ln(),
            });
        }
        contributions.push(contribution.min(cap));
        weights.push(weight);
    }

    let n = contributions.len();
    let mean = contributions.iter().sum::<f64>() / n as f64;
    let variance = sample_variance(&contributions, mean);
    let se = (variance / n as f64).sqrt();
    let log_term = (2.0 / delta).ln();
    let radius = (2.0 * variance * log_term / n as f64).sqrt()
        + (7.0 * cap * log_term) / (3.0 * (n - 1) as f64);
    let upper = (mean + radius).min(1.0);

    let sum_w = weights.iter().sum::<f64>();
    let sum_w2 = weights.iter().map(|x| x * x).sum::<f64>();
    let weight_ess = if sum_w2 > 0.0 {
        (sum_w * sum_w / sum_w2).clamp(0.0, n as f64)
    } else {
        0.0
    };
    let sum_c = contributions.iter().sum::<f64>();
    let sum_c2 = contributions.iter().map(|x| x * x).sum::<f64>();
    let event_ess = if sum_c2 > 0.0 {
        (sum_c * sum_c / sum_c2).clamp(0.0, n as f64)
    } else {
        0.0
    };
    let max_share = if sum_c > 0.0 {
        contributions.iter().copied().fold(0.0, f64::max) / sum_c
    } else {
        0.0
    };

    Ok(GeometryEstimateReport {
        method: "defensive_flat_boundary_geometry_importance_sampling",
        proposal_id: proposal.id.clone(),
        sample_count: n,
        event_count: observations.iter().filter(|o| o.event).count(),
        defensive_fraction: proposal.epsilon,
        weighted_contribution_cap: cap,
        estimate: mean,
        sample_variance: variance,
        standard_error: se,
        confidence_delta: delta,
        one_sided_confidence_level: 1.0 - delta,
        empirical_bernstein_radius: radius,
        upper_confidence_bound: upper,
        max_likelihood_ratio: weights.iter().copied().fold(0.0, f64::max),
        likelihood_ratio_ess: weight_ess,
        event_contribution_ess: event_ess,
        max_event_contribution_share: max_share,
        generating_mode_counts: mode_counts,
        claim_ceiling: "fixed_sample_statistical_result_for_declared_gaussian_halfspace_union_fixture_not_curved_orbital_event_truth",
    })
}

fn validate_evidence(evidence: &EvaluationEvidence) -> Result<(), ProposalError> {
    for value in [
        &evidence.proposal_training_id,
        &evidence.evaluation_sample_id,
        &evidence.sampling_design_id,
        &evidence.sampling_evidence_ref,
        &evidence.sample_order_digest,
    ] {
        if !valid_identifier(value) {
            return Err(ProposalError::InvalidEvaluationEvidence);
        }
    }
    Ok(())
}

fn valid_identifier(value: &str) -> bool {
    !value.trim().is_empty()
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._:/-".contains(&b))
}

fn unit_open(value: f64) -> bool {
    value.is_finite() && value > 0.0 && value < 1.0
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

fn log_standard_normal(point: &[f64]) -> f64 {
    -0.5 * point.len() as f64 * LOG_2PI - 0.5 * point.iter().map(|x| x * x).sum::<f64>()
}

fn log_sum_exp(values: &[f64]) -> f64 {
    let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if max == f64::NEG_INFINITY {
        return max;
    }
    if max == f64::INFINITY {
        return max;
    }
    max + values.iter().map(|x| (x - max).exp()).sum::<f64>().ln()
}

fn sample_variance(values: &[f64], mean: f64) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    values.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (values.len() - 1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_sided_proposal() -> DefensiveGeometryProposal {
        DefensiveGeometryProposal {
            id: "flat-halfspace-one-mode-v1".into(),
            dimension: 2,
            epsilon: 0.05,
            modes: vec![GeometryMode {
                id: "right-tail-instanton".into(),
                mixture_weight: 1.0,
                normal: vec![1.0, 0.0],
                tangent_basis: vec![vec![0.0, 1.0]],
                threshold: 2.0,
                exponential_rate: 1.5,
            }],
        }
    }

    fn two_tail_proposal() -> DefensiveGeometryProposal {
        DefensiveGeometryProposal {
            id: "disconnected-two-tail-v1".into(),
            dimension: 2,
            epsilon: 0.1,
            modes: vec![
                GeometryMode {
                    id: "positive-tail".into(),
                    mixture_weight: 0.5,
                    normal: vec![1.0, 0.0],
                    tangent_basis: vec![vec![0.0, 1.0]],
                    threshold: 2.0,
                    exponential_rate: 1.5,
                },
                GeometryMode {
                    id: "negative-tail".into(),
                    mixture_weight: 0.5,
                    normal: vec![-1.0, 0.0],
                    tangent_basis: vec![vec![0.0, 1.0]],
                    threshold: 2.0,
                    exponential_rate: 1.5,
                },
            ],
        }
    }

    fn evidence() -> EvaluationEvidence {
        EvaluationEvidence {
            proposal_training_id: "geometry-training-v1".into(),
            evaluation_sample_id: "geometry-evaluation-v1".into(),
            sampling_design_id: "iid-defensive-geo-v1".into(),
            sampling_evidence_ref: "fixture://iid-design/v1".into(),
            sample_order_digest: "0123456789abcdef".into(),
            proposal_frozen_before_evaluation: true,
            evaluation_draws_are_iid_from_full_mixture: true,
        }
    }

    #[test]
    fn geometry_mode_density_is_normalized_by_tensor_product_quadrature() {
        let mode = &one_sided_proposal().modes[0];
        let tangent_steps = 400usize;
        let normal_steps = 4000usize;
        let dt = 16.0 / tangent_steps as f64;
        let ds = 16.0 / normal_steps as f64;
        let mut integral = 0.0;
        for i in 0..tangent_steps {
            let tangent = -8.0 + (i as f64 + 0.5) * dt;
            for j in 0..normal_steps {
                let normal = mode.threshold + (j as f64 + 0.5) * ds;
                integral += mode.log_density(&[normal, tangent]).exp() * dt * ds;
            }
        }
        assert!((integral - 1.0).abs() < 2.0e-4, "integral={integral}");
    }

    #[test]
    fn defensive_full_mixture_density_has_bounded_likelihood_ratio() {
        let proposal = one_sided_proposal();
        let cap = proposal.contribution_cap().unwrap();
        for point in [
            vec![-100.0, -25.0],
            vec![0.0, 0.0],
            vec![2.0, 0.0],
            vec![3.5, -2.0],
            vec![100.0, 20.0],
        ] {
            assert!(proposal.likelihood_ratio(&point).unwrap() <= cap * (1.0 + 1.0e-9));
        }
    }

    #[test]
    fn missed_geometric_mode_remains_supported_by_nominal_component() {
        let proposal = one_sided_proposal();
        let missed = vec![-2.5, 0.0];
        assert_eq!(
            proposal.log_geometry_density(&missed).unwrap(),
            f64::NEG_INFINITY
        );
        assert!(proposal.log_proposal_density(&missed).unwrap().is_finite());
        assert!(proposal.likelihood_ratio(&missed).unwrap() <= 1.0 / proposal.epsilon + 1.0e-9);
    }

    #[test]
    fn disconnected_two_tail_proposal_covers_both_declared_modes() {
        let proposal = two_tail_proposal();
        proposal.validate().unwrap();
        let positive = proposal
            .sample_from_latents(0.5, 0.2, &[0.0, 0.0], &[0.3], 0.5)
            .unwrap();
        let negative = proposal
            .sample_from_latents(0.5, 0.8, &[0.0, 0.0], &[0.3], 0.5)
            .unwrap();
        assert_eq!(positive.generating_component_id, "positive-tail");
        assert_eq!(negative.generating_component_id, "negative-tail");
        assert!(positive.point[0] >= 2.0);
        assert!(negative.point[0] <= -2.0);
        assert!(
            proposal
                .log_proposal_density(&positive.point)
                .unwrap()
                .is_finite()
        );
        assert!(
            proposal
                .log_proposal_density(&negative.point)
                .unwrap()
                .is_finite()
        );
    }

    #[test]
    fn deterministic_sampler_uses_full_frame_and_declared_mode() {
        let proposal = one_sided_proposal();
        let sample = proposal
            .sample_from_latents(0.5, 0.5, &[0.0, 0.0], &[0.25], 0.5)
            .unwrap();
        assert_eq!(sample.generating_component_id, "right-tail-instanton");
        assert!((sample.point[0] - (2.0 - 0.5_f64.ln() / 1.5)).abs() < 1.0e-12);
        assert_eq!(sample.point[1], 0.25);
    }

    #[test]
    fn rejects_nonorthogonal_frames_and_bad_mixture_weights() {
        let mut bad_frame = one_sided_proposal();
        bad_frame.modes[0].tangent_basis[0] = vec![1.0, 0.0];
        assert_eq!(
            bad_frame.validate(),
            Err(ProposalError::FrameNotOrthonormal)
        );

        let mut bad_weight = one_sided_proposal();
        bad_weight.modes[0].mixture_weight = 0.9;
        assert_eq!(
            bad_weight.validate(),
            Err(ProposalError::GeometryWeightsDoNotSumToOne(0.9))
        );
    }

    #[test]
    fn estimator_checks_event_labels_and_training_evaluation_separation() {
        let proposal = one_sided_proposal();
        let bad = vec![
            GeometryObservation {
                sample_id: "row-1".into(),
                point: vec![1.0, 0.0],
                generating_component_id: "right-tail-instanton".into(),
                event: true,
            },
            GeometryObservation {
                sample_id: "row-2".into(),
                point: vec![3.0, 0.0],
                generating_component_id: "right-tail-instanton".into(),
                event: true,
            },
        ];
        assert!(matches!(
            estimate_geometry_event_probability(&proposal, &bad, 0.05, &evidence()),
            Err(ProposalError::EventLabelMismatch { .. })
        ));

        let good = vec![
            GeometryObservation {
                sample_id: "row-a".into(),
                point: vec![1.0, 0.0],
                generating_component_id: "nominal-defensive-component".into(),
                event: false,
            },
            GeometryObservation {
                sample_id: "row-b".into(),
                point: vec![3.0, 0.0],
                generating_component_id: "right-tail-instanton".into(),
                event: true,
            },
        ];
        let mut bad_evidence = evidence();
        bad_evidence.evaluation_sample_id = bad_evidence.proposal_training_id.clone();
        assert_eq!(
            estimate_geometry_event_probability(&proposal, &good, 0.05, &bad_evidence),
            Err(ProposalError::TrainingEvaluationLeakage)
        );
    }

    #[test]
    fn estimate_uses_full_mixture_and_reports_component_counts() {
        let proposal = one_sided_proposal();
        let observations = vec![
            GeometryObservation {
                sample_id: "row-1".into(),
                point: vec![1.0, 0.0],
                generating_component_id: "nominal-defensive-component".into(),
                event: false,
            },
            GeometryObservation {
                sample_id: "row-2".into(),
                point: vec![2.2, 0.1],
                generating_component_id: "right-tail-instanton".into(),
                event: true,
            },
            GeometryObservation {
                sample_id: "row-3".into(),
                point: vec![-2.2, -0.2],
                generating_component_id: "nominal-defensive-component".into(),
                event: false,
            },
            GeometryObservation {
                sample_id: "row-4".into(),
                point: vec![3.0, 0.4],
                generating_component_id: "right-tail-instanton".into(),
                event: true,
            },
        ];
        let report =
            estimate_geometry_event_probability(&proposal, &observations, 0.05, &evidence())
                .unwrap();
        assert_eq!(report.event_count, 2);
        assert!(report.estimate > 0.0);
        assert!(report.upper_confidence_bound >= report.estimate);
        assert!(report.upper_confidence_bound <= 1.0);
        assert_eq!(
            report
                .generating_mode_counts
                .iter()
                .find(|(id, _)| id == "right-tail-instanton")
                .unwrap()
                .1,
            2
        );
        assert!(report.max_likelihood_ratio <= report.weighted_contribution_cap * (1.0 + 1.0e-9));
    }
}

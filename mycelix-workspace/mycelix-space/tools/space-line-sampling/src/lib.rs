#![forbid(unsafe_code)]

//! Gaussian line sampling by exact one-dimensional conditional integration.
//!
//! For Z~N(0,I_d), a unit direction a and an orthonormal complement basis B,
//! decompose Z = B V + a T, where V~N(0,I_(d-1)), T~N(0,1), and V,T are
//! independent. For each transverse draw V=v, the high-fidelity event solver
//! supplies every disjoint interval in T that produces collision. The
//! conditional event probability is the sum of standard-normal interval
//! masses. Averaging these [0,1]-bounded values over independent transverse
//! lines is a Rao-Blackwellized estimator.
//!
//! The statistical bound applies across independent transverse lines, not
//! across interval endpoints or high-fidelity root evaluations. Completeness
//! of the interval solver is a separate claim and is mandatory: a missed
//! disconnected, tangent, or narrow collision interval biases the estimate
//! downward.
//!
//! This crate validates and integrates supplied interval sets. It does not
//! discover them by propagating orbits and does not validate the physical
//! target distribution or event oracle.

use std::collections::BTreeSet;
use std::fmt;

const SQRT_2: f64 = std::f64::consts::SQRT_2;
const SQRT_2PI: f64 = 2.506_628_274_631_000_2;
const TWO_OVER_SQRT_PI: f64 = 1.128_379_167_095_512_6;
const CF_TERMS: usize = 200;
const SERIES_TERMS: usize = 120;
const MAX_NORMAL_TAIL_ARGUMENT: f64 = 37.0;
const ORTHONORMAL_TOLERANCE: f64 = 2.0e-8;
const MASS_TOLERANCE: f64 = 5.0e-12;

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionInterval {
    /// Lower T endpoint. Negative infinity is allowed.
    pub lower: f64,
    /// Upper T endpoint. Positive infinity is allowed.
    pub upper: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TransverseLine {
    pub line_id: String,
    pub transverse_sample_id: String,
    /// Coordinates V in the declared orthonormal complement basis.
    pub transverse_coordinates: Vec<f64>,
    /// All disjoint collision intervals along T for this transverse line.
    /// An empty vector means no collision interval, but only when the
    /// evidence attests that the interval solver is complete.
    pub collision_intervals: Vec<CollisionInterval>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OrthonormalLineFrame {
    pub direction_id: String,
    pub direction: Vec<f64>,
    /// Row-major d by (d-1) basis whose columns span the orthogonal complement.
    pub complement_basis_id: String,
    pub complement_basis: Vec<Vec<f64>>,
}

impl OrthonormalLineFrame {
    pub fn validate(&self) -> Result<(), LineSamplingError> {
        let dimension = self.direction.len();
        if dimension < 2 {
            return Err(LineSamplingError::EmptyDirection);
        }
        if !valid_identifier(&self.direction_id) {
            return Err(LineSamplingError::InvalidIdentifier("direction_id"));
        }
        if !valid_identifier(&self.complement_basis_id) {
            return Err(LineSamplingError::InvalidIdentifier("complement_basis_id"));
        }
        if self.direction.iter().any(|value| !value.is_finite()) {
            return Err(LineSamplingError::NonFiniteValue("direction"));
        }
        let norm = self
            .direction
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        if (norm - 1.0).abs() > ORTHONORMAL_TOLERANCE {
            return Err(LineSamplingError::DirectionNotUnitLength(norm));
        }
        if self.complement_basis.len() != dimension
            || self
                .complement_basis
                .iter()
                .any(|row| row.len() != dimension - 1)
        {
            return Err(LineSamplingError::InvalidComplementBasisShape);
        }
        if self
            .complement_basis
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
        {
            return Err(LineSamplingError::NonFiniteValue("complement_basis"));
        }

        for column in 0..(dimension - 1) {
            let direction_dot = (0..dimension)
                .map(|row| self.direction[row] * self.complement_basis[row][column])
                .sum::<f64>();
            if direction_dot.abs() > ORTHONORMAL_TOLERANCE {
                return Err(LineSamplingError::ComplementBasisNotOrthogonalToDirection { column });
            }
            for other in 0..=column {
                let dot = (0..dimension)
                    .map(|row| {
                        self.complement_basis[row][column] * self.complement_basis[row][other]
                    })
                    .sum::<f64>();
                let expected = if column == other { 1.0 } else { 0.0 };
                if (dot - expected).abs() > ORTHONORMAL_TOLERANCE {
                    return Err(LineSamplingError::ComplementBasisNotOrthonormal {
                        column_a: other,
                        column_b: column,
                    });
                }
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineSamplingEvidence {
    pub whitening_transform_id: String,
    pub whitening_transform_revision: String,
    pub direction_training_id: String,
    pub transverse_evaluation_id: String,
    pub direction_id: String,
    pub direction_sha256: String,
    pub complement_basis_id: String,
    pub complement_basis_sha256: String,
    pub transverse_sampling_design_id: String,
    pub transverse_sampling_evidence_ref: String,
    pub transverse_sample_order_sha256: String,
    pub line_solver_id: String,
    pub line_solver_revision: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub transverse_lines_are_iid: bool,
    pub direction_and_basis_frozen_before_evaluation: bool,
    pub evaluation_lines_are_disjoint_from_direction_training: bool,
    pub interval_solver_enumerates_all_disconnected_intervals: bool,
    pub interval_sets_are_complete: bool,
    pub no_unresolved_intervals: bool,
    pub high_fidelity_oracle_is_continuous_time: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LineContribution {
    pub line_id: String,
    pub transverse_sample_id: String,
    pub collision_interval_count: usize,
    pub conditional_collision_probability: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LineSamplingReport {
    pub method: &'static str,
    pub target_distribution: &'static str,
    pub whitening_transform_id: String,
    pub whitening_transform_revision: String,
    pub direction_id: String,
    pub direction_sha256: String,
    pub complement_basis_id: String,
    pub complement_basis_sha256: String,
    pub direction_training_id: String,
    pub transverse_evaluation_id: String,
    pub transverse_sampling_design_id: String,
    pub transverse_sampling_evidence_ref: String,
    pub transverse_sample_order_sha256: String,
    pub line_solver_id: String,
    pub line_solver_revision: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub line_count: usize,
    pub total_collision_intervals: usize,
    pub lines_with_no_collision_interval: usize,
    pub estimate: f64,
    pub line_contribution_sample_variance: f64,
    pub standard_error: f64,
    pub confidence_delta: f64,
    pub one_sided_confidence_level: f64,
    pub hoeffding_radius: f64,
    pub upper_confidence_bound: f64,
    pub inference_unit: &'static str,
    pub interval_solver_completeness_status: &'static str,
    pub contributions: Vec<LineContribution>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum LineSamplingError {
    InvalidIdentifier(&'static str),
    InvalidSha256(&'static str),
    InvalidDelta(f64),
    EmptyLineSet,
    InsufficientIndependentLines(usize),
    EmptyDirection,
    DirectionNotUnitLength(f64),
    InvalidComplementBasisShape,
    ComplementBasisNotOrthonormal {
        column_a: usize,
        column_b: usize,
    },
    ComplementBasisNotOrthogonalToDirection {
        column: usize,
    },
    NonFiniteValue(&'static str),
    DuplicateLineId(String),
    DuplicateTransverseSampleId(String),
    DuplicateDirectionTrainingAndEvaluationSet,
    MissingSamplingEvidence,
    LinesNotIid,
    DirectionNotFrozen,
    EvaluationSetReusesDirectionTraining,
    IncompleteIntervalEnumeration,
    UnresolvedCollisionIntervals,
    LowFidelityOracleNotAccepted,
    TransverseDimensionMismatch {
        expected: usize,
        actual: usize,
    },
    InvalidInterval {
        lower: f64,
        upper: f64,
    },
    IntervalsOverlapOrAreUnsorted {
        previous_upper: f64,
        next_lower: f64,
    },
    NumericalTailRangeExceeded(f64),
    ConditionalMassExceedsOne(f64),
}

impl fmt::Display for LineSamplingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIdentifier(field) => write!(
                f,
                "{field} must be non-empty and use only ASCII letters, digits, dot, underscore, colon, slash and hyphen"
            ),
            Self::InvalidSha256(field) => write!(
                f,
                "{field} must be a 64-character lowercase hexadecimal SHA-256 digest"
            ),
            Self::InvalidDelta(value) => {
                write!(f, "delta must be finite and in (0,1); got {value}")
            }
            Self::EmptyLineSet => write!(f, "at least one transverse line is required"),
            Self::InsufficientIndependentLines(count) => write!(
                f,
                "at least two independent transverse lines are required; got {count}"
            ),
            Self::EmptyDirection => write!(f, "line direction must not be empty"),
            Self::DirectionNotUnitLength(norm) => {
                write!(f, "line direction must be unit length; norm was {norm}")
            }
            Self::InvalidComplementBasisShape => {
                write!(f, "complement basis must have d rows and d-1 columns")
            }
            Self::ComplementBasisNotOrthonormal { column_a, column_b } => write!(
                f,
                "complement basis columns {column_a} and {column_b} are not orthonormal"
            ),
            Self::ComplementBasisNotOrthogonalToDirection { column } => write!(
                f,
                "complement basis column {column} is not orthogonal to line direction"
            ),
            Self::NonFiniteValue(field) => write!(
                f,
                "{field} contains NaN or infinity where a finite value is required"
            ),
            Self::DuplicateLineId(value) => write!(f, "duplicate line ID {value}"),
            Self::DuplicateTransverseSampleId(value) => {
                write!(f, "duplicate transverse sample ID {value}")
            }
            Self::DuplicateDirectionTrainingAndEvaluationSet => write!(
                f,
                "direction training and transverse evaluation IDs must differ"
            ),
            Self::MissingSamplingEvidence => write!(
                f,
                "sampling design, evidence reference, line order and model revisions are required"
            ),
            Self::LinesNotIid => write!(
                f,
                "transverse lines must be IID under the declared standard-normal complement distribution"
            ),
            Self::DirectionNotFrozen => write!(
                f,
                "direction and complement basis must be frozen before evaluation"
            ),
            Self::EvaluationSetReusesDirectionTraining => write!(
                f,
                "transverse evaluation lines must be disjoint from direction training"
            ),
            Self::IncompleteIntervalEnumeration => write!(
                f,
                "the line solver has not attested complete enumeration of all collision intervals"
            ),
            Self::UnresolvedCollisionIntervals => write!(
                f,
                "unresolved collision intervals cannot be treated as non-collisions"
            ),
            Self::LowFidelityOracleNotAccepted => write!(
                f,
                "line intervals must be classified by a high-fidelity continuous-time event oracle"
            ),
            Self::TransverseDimensionMismatch { expected, actual } => write!(
                f,
                "transverse coordinate dimension mismatch: expected {expected}, got {actual}"
            ),
            Self::InvalidInterval { lower, upper } => {
                write!(f, "invalid interval [{lower}, {upper}]")
            }
            Self::IntervalsOverlapOrAreUnsorted {
                previous_upper,
                next_lower,
            } => write!(
                f,
                "collision intervals overlap or are unsorted: previous upper {previous_upper}, next lower {next_lower}"
            ),
            Self::NumericalTailRangeExceeded(value) => write!(
                f,
                "normal tail argument {value} exceeds the qualified numerical range"
            ),
            Self::ConditionalMassExceedsOne(value) => {
                write!(f, "sum of collision interval masses exceeds one: {value}")
            }
        }
    }
}

impl std::error::Error for LineSamplingError {}

fn valid_identifier(value: &str) -> bool {
    !value.trim().is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"._:/-".contains(&byte))
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_delta(delta: f64) -> Result<(), LineSamplingError> {
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(LineSamplingError::InvalidDelta(delta));
    }
    Ok(())
}

/// Standard-normal upper-tail Q(x)=P(N(0,1)>=x), x>=0.
pub fn standard_normal_upper_tail(x: f64) -> Result<f64, LineSamplingError> {
    if x.is_nan() || x < 0.0 {
        return Err(LineSamplingError::NonFiniteValue("normal_tail_argument"));
    }
    if x == f64::INFINITY {
        return Ok(0.0);
    }
    if !x.is_finite() {
        return Err(LineSamplingError::NonFiniteValue("normal_tail_argument"));
    }
    if x > MAX_NORMAL_TAIL_ARGUMENT {
        return Err(LineSamplingError::NumericalTailRangeExceeded(x));
    }

    if x < 1.5 {
        let z = x / SQRT_2;
        let mut term = z;
        let mut sum = term;
        for n in 1..=SERIES_TERMS {
            let nf = n as f64;
            term *= -z * z * (2.0 * nf - 1.0) / (nf * (2.0 * nf + 1.0));
            sum += term;
            if term.abs() <= 1.0e-18 * sum.abs().max(1.0) {
                break;
            }
        }
        let erf = TWO_OVER_SQRT_PI * sum;
        return Ok((0.5 * (1.0 - erf)).clamp(0.0, 0.5));
    }

    // Laplace continued fraction for the Gaussian Mills ratio.
    let mut fraction = 0.0;
    for numerator in (1..=CF_TERMS).rev() {
        fraction = numerator as f64 / (x + fraction);
    }
    let density = (-0.5 * x * x).exp() / SQRT_2PI;
    Ok((density / (x + fraction)).clamp(0.0, 0.5))
}

/// Probability mass of a standard-normal variable in [lower, upper].
///
/// Tail differences are used directly so far-tail intervals do not subtract
/// two CDF values close to one.
pub fn standard_normal_interval_mass(lower: f64, upper: f64) -> Result<f64, LineSamplingError> {
    if lower.is_nan() || upper.is_nan() || lower >= upper {
        return Err(LineSamplingError::InvalidInterval { lower, upper });
    }
    if lower == f64::NEG_INFINITY && upper == f64::INFINITY {
        return Ok(1.0);
    }

    let mass = if lower >= 0.0 {
        standard_normal_upper_tail(lower)? - standard_normal_upper_tail(upper)?
    } else if upper <= 0.0 {
        standard_normal_upper_tail(-upper)? - standard_normal_upper_tail(-lower)?
    } else {
        1.0 - standard_normal_upper_tail(-lower)? - standard_normal_upper_tail(upper)?
    };
    if !mass.is_finite() || mass < -MASS_TOLERANCE {
        return Err(LineSamplingError::ConditionalMassExceedsOne(mass));
    }
    Ok(mass.max(0.0))
}

fn validate_evidence(
    frame: &OrthonormalLineFrame,
    evidence: &LineSamplingEvidence,
) -> Result<(), LineSamplingError> {
    for (field, value) in [
        ("whitening_transform_id", &evidence.whitening_transform_id),
        (
            "whitening_transform_revision",
            &evidence.whitening_transform_revision,
        ),
        ("direction_training_id", &evidence.direction_training_id),
        (
            "transverse_evaluation_id",
            &evidence.transverse_evaluation_id,
        ),
        ("direction_id", &evidence.direction_id),
        ("complement_basis_id", &evidence.complement_basis_id),
        (
            "transverse_sampling_design_id",
            &evidence.transverse_sampling_design_id,
        ),
        ("line_solver_id", &evidence.line_solver_id),
        ("line_solver_revision", &evidence.line_solver_revision),
        (
            "high_fidelity_event_oracle_id",
            &evidence.high_fidelity_event_oracle_id,
        ),
        (
            "high_fidelity_event_oracle_revision",
            &evidence.high_fidelity_event_oracle_revision,
        ),
        ("event_definition_id", &evidence.event_definition_id),
        ("event_horizon_id", &evidence.event_horizon_id),
    ] {
        if !valid_identifier(value) {
            return Err(LineSamplingError::InvalidIdentifier(field));
        }
    }
    if evidence.direction_id != frame.direction_id
        || evidence.complement_basis_id != frame.complement_basis_id
    {
        return Err(LineSamplingError::MissingSamplingEvidence);
    }
    for (field, digest) in [
        ("direction_sha256", &evidence.direction_sha256),
        ("complement_basis_sha256", &evidence.complement_basis_sha256),
        (
            "transverse_sample_order_sha256",
            &evidence.transverse_sample_order_sha256,
        ),
    ] {
        if !valid_sha256(digest) {
            return Err(LineSamplingError::InvalidSha256(field));
        }
    }
    if evidence.transverse_sampling_evidence_ref.trim().is_empty() {
        return Err(LineSamplingError::MissingSamplingEvidence);
    }
    if evidence.direction_training_id == evidence.transverse_evaluation_id {
        return Err(LineSamplingError::DuplicateDirectionTrainingAndEvaluationSet);
    }
    if !evidence.transverse_lines_are_iid {
        return Err(LineSamplingError::LinesNotIid);
    }
    if !evidence.direction_and_basis_frozen_before_evaluation {
        return Err(LineSamplingError::DirectionNotFrozen);
    }
    if !evidence.evaluation_lines_are_disjoint_from_direction_training {
        return Err(LineSamplingError::EvaluationSetReusesDirectionTraining);
    }
    if !evidence.interval_solver_enumerates_all_disconnected_intervals
        || !evidence.interval_sets_are_complete
    {
        return Err(LineSamplingError::IncompleteIntervalEnumeration);
    }
    if !evidence.no_unresolved_intervals {
        return Err(LineSamplingError::UnresolvedCollisionIntervals);
    }
    if !evidence.high_fidelity_oracle_is_continuous_time {
        return Err(LineSamplingError::LowFidelityOracleNotAccepted);
    }
    Ok(())
}

/// Estimate a Gaussian event probability from independent transverse lines.
///
/// For every line, the supplied disjoint collision intervals are integrated
/// exactly along the independent N(0,1) coordinate T. The reported Hoeffding
/// bound uses the number of independent transverse lines, not the number of
/// intervals or high-fidelity root evaluations.
pub fn estimate_gaussian_line_probability(
    frame: &OrthonormalLineFrame,
    lines: &[TransverseLine],
    delta: f64,
    evidence: &LineSamplingEvidence,
) -> Result<LineSamplingReport, LineSamplingError> {
    frame.validate()?;
    validate_delta(delta)?;
    validate_evidence(frame, evidence)?;
    if lines.is_empty() {
        return Err(LineSamplingError::EmptyLineSet);
    }
    if lines.len() < 2 {
        return Err(LineSamplingError::InsufficientIndependentLines(lines.len()));
    }

    let expected_dimension = frame.direction.len() - 1;
    let mut line_ids = BTreeSet::new();
    let mut transverse_ids = BTreeSet::new();
    let mut contributions = Vec::with_capacity(lines.len());
    let mut total_intervals = 0usize;
    let mut no_collision_lines = 0usize;

    for line in lines {
        if !valid_identifier(&line.line_id) || !line_ids.insert(line.line_id.as_str()) {
            return Err(LineSamplingError::DuplicateLineId(line.line_id.clone()));
        }
        if !valid_identifier(&line.transverse_sample_id)
            || !transverse_ids.insert(line.transverse_sample_id.as_str())
        {
            return Err(LineSamplingError::DuplicateTransverseSampleId(
                line.transverse_sample_id.clone(),
            ));
        }
        if line.transverse_coordinates.len() != expected_dimension {
            return Err(LineSamplingError::TransverseDimensionMismatch {
                expected: expected_dimension,
                actual: line.transverse_coordinates.len(),
            });
        }
        if line
            .transverse_coordinates
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err(LineSamplingError::NonFiniteValue("transverse_coordinates"));
        }

        let mut mass = 0.0;
        let mut previous_upper: Option<f64> = None;
        for interval in &line.collision_intervals {
            if interval.lower.is_nan()
                || interval.upper.is_nan()
                || interval.lower >= interval.upper
                || interval.lower == f64::INFINITY
                || interval.upper == f64::NEG_INFINITY
            {
                return Err(LineSamplingError::InvalidInterval {
                    lower: interval.lower,
                    upper: interval.upper,
                });
            }
            if let Some(previous) = previous_upper {
                if previous > interval.lower {
                    return Err(LineSamplingError::IntervalsOverlapOrAreUnsorted {
                        previous_upper: previous,
                        next_lower: interval.lower,
                    });
                }
            }
            mass += standard_normal_interval_mass(interval.lower, interval.upper)?;
            previous_upper = Some(interval.upper);
        }
        if mass > 1.0 + MASS_TOLERANCE {
            return Err(LineSamplingError::ConditionalMassExceedsOne(mass));
        }
        let conditional_probability = mass.clamp(0.0, 1.0);
        if line.collision_intervals.is_empty() {
            no_collision_lines += 1;
        }
        total_intervals += line.collision_intervals.len();
        contributions.push(LineContribution {
            line_id: line.line_id.clone(),
            transverse_sample_id: line.transverse_sample_id.clone(),
            collision_interval_count: line.collision_intervals.len(),
            conditional_collision_probability: conditional_probability,
        });
    }

    let line_count = contributions.len();
    let values = contributions
        .iter()
        .map(|contribution| contribution.conditional_collision_probability)
        .collect::<Vec<_>>();
    let estimate = values.iter().sum::<f64>() / line_count as f64;
    let variance = sample_variance(&values, estimate);
    let standard_error = (variance / line_count as f64).sqrt();
    let radius = ((1.0 / delta).ln() / (2.0 * line_count as f64)).sqrt();
    let upper = (estimate + radius).min(1.0);

    Ok(LineSamplingReport {
        method: "gaussian_line_sampling_conditional_normal_integration",
        target_distribution: "whitened_standard_normal_N_0_I",
        whitening_transform_id: evidence.whitening_transform_id.clone(),
        whitening_transform_revision: evidence.whitening_transform_revision.clone(),
        direction_id: evidence.direction_id.clone(),
        direction_sha256: evidence.direction_sha256.clone(),
        complement_basis_id: evidence.complement_basis_id.clone(),
        complement_basis_sha256: evidence.complement_basis_sha256.clone(),
        direction_training_id: evidence.direction_training_id.clone(),
        transverse_evaluation_id: evidence.transverse_evaluation_id.clone(),
        transverse_sampling_design_id: evidence.transverse_sampling_design_id.clone(),
        transverse_sampling_evidence_ref: evidence.transverse_sampling_evidence_ref.clone(),
        transverse_sample_order_sha256: evidence.transverse_sample_order_sha256.clone(),
        line_solver_id: evidence.line_solver_id.clone(),
        line_solver_revision: evidence.line_solver_revision.clone(),
        high_fidelity_event_oracle_id: evidence.high_fidelity_event_oracle_id.clone(),
        high_fidelity_event_oracle_revision: evidence.high_fidelity_event_oracle_revision.clone(),
        event_definition_id: evidence.event_definition_id.clone(),
        event_horizon_id: evidence.event_horizon_id.clone(),
        line_count,
        total_collision_intervals: total_intervals,
        lines_with_no_collision_interval: no_collision_lines,
        estimate,
        line_contribution_sample_variance: variance,
        standard_error,
        confidence_delta: delta,
        one_sided_confidence_level: 1.0 - delta,
        hoeffding_radius: radius,
        upper_confidence_bound: upper,
        inference_unit: "independent_transverse_line",
        interval_solver_completeness_status: "caller_attested_all_intervals_complete_no_unresolved_lines",
        contributions,
        claim_ceiling: "statistical_bound_on_conditional_line_integrals_given_complete_interval_sets_not_orbit_truth_or_operational_safety",
    })
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

    fn sha(ch: char) -> String {
        std::iter::repeat(ch).take(64).collect()
    }

    fn frame() -> OrthonormalLineFrame {
        OrthonormalLineFrame {
            direction_id: "line-direction-x-v1".into(),
            direction: vec![1.0, 0.0],
            complement_basis_id: "complement-y-v1".into(),
            complement_basis: vec![vec![0.0], vec![1.0]],
        }
    }

    fn evidence() -> LineSamplingEvidence {
        LineSamplingEvidence {
            whitening_transform_id: "whitening-v1".into(),
            whitening_transform_revision: "sha256:whitening-v1".into(),
            direction_training_id: "direction-training-split-v1".into(),
            transverse_evaluation_id: "transverse-evaluation-split-v1".into(),
            direction_id: "line-direction-x-v1".into(),
            direction_sha256: sha('a'),
            complement_basis_id: "complement-y-v1".into(),
            complement_basis_sha256: sha('b'),
            transverse_sampling_design_id: "iid-transverse-normal-v1".into(),
            transverse_sampling_evidence_ref: "fixture://line-sampling/iid-design-v1".into(),
            transverse_sample_order_sha256: sha('c'),
            line_solver_id: "hf-line-event-solver".into(),
            line_solver_revision: "sha256:line-solver-v1".into(),
            high_fidelity_event_oracle_id: "hf-continuous-time-event-oracle".into(),
            high_fidelity_event_oracle_revision: "sha256:event-oracle-v1".into(),
            event_definition_id: "any-contact-over-horizon-v1".into(),
            event_horizon_id: "seven-day-risk-window-v1".into(),
            transverse_lines_are_iid: true,
            direction_and_basis_frozen_before_evaluation: true,
            evaluation_lines_are_disjoint_from_direction_training: true,
            interval_solver_enumerates_all_disconnected_intervals: true,
            interval_sets_are_complete: true,
            no_unresolved_intervals: true,
            high_fidelity_oracle_is_continuous_time: true,
        }
    }

    #[test]
    fn normal_tail_matches_standard_reference_values() {
        assert!((standard_normal_upper_tail(0.0).unwrap() - 0.5).abs() < 1.0e-15);
        assert!((standard_normal_upper_tail(1.0).unwrap() - 0.15865525393145707).abs() < 2.0e-14);
        assert!((standard_normal_upper_tail(2.0).unwrap() - 0.02275013194817921).abs() < 2.0e-14);
        assert!((standard_normal_upper_tail(5.0).unwrap() - 2.866515718791933e-7).abs() < 2.0e-19);
        assert_eq!(standard_normal_upper_tail(f64::INFINITY).unwrap(), 0.0);
    }

    #[test]
    fn interval_mass_handles_full_half_and_symmetric_intervals() {
        assert_eq!(
            standard_normal_interval_mass(f64::NEG_INFINITY, f64::INFINITY).unwrap(),
            1.0
        );
        assert!((standard_normal_interval_mass(0.0, f64::INFINITY).unwrap() - 0.5).abs() < 1.0e-15);
        assert!(
            (standard_normal_interval_mass(f64::NEG_INFINITY, 0.0).unwrap() - 0.5).abs() < 1.0e-15
        );
        assert!(
            (standard_normal_interval_mass(-1.0, 1.0).unwrap() - 0.6826894921370859).abs()
                < 2.0e-14
        );
    }

    #[test]
    fn tail_interval_mass_avoids_cdf_cancellation() {
        let mass = standard_normal_interval_mass(5.0, 6.0).unwrap();
        assert!((mass - 2.856649842341562e-7).abs() < 2.0e-19);
        assert!(mass > 0.0);
    }

    #[test]
    fn disjoint_intervals_are_added_not_replaced_by_single_interval() {
        let expected = 2.0
            * (standard_normal_upper_tail(1.0).unwrap() - standard_normal_upper_tail(2.0).unwrap());
        let observed = standard_normal_interval_mass(-2.0, -1.0).unwrap()
            + standard_normal_interval_mass(1.0, 2.0).unwrap();
        assert!((observed - expected).abs() < 1.0e-15);
    }

    #[test]
    fn line_estimator_integrates_multiple_intervals_and_uses_line_count() {
        let lines = vec![
            TransverseLine {
                line_id: "line-0".into(),
                transverse_sample_id: "v-0".into(),
                transverse_coordinates: vec![0.0],
                collision_intervals: vec![
                    CollisionInterval {
                        lower: -2.0,
                        upper: -1.0,
                    },
                    CollisionInterval {
                        lower: 1.0,
                        upper: 2.0,
                    },
                ],
            },
            TransverseLine {
                line_id: "line-1".into(),
                transverse_sample_id: "v-1".into(),
                transverse_coordinates: vec![2.0],
                collision_intervals: vec![],
            },
        ];
        let report =
            estimate_gaussian_line_probability(&frame(), &lines, 0.05, &evidence()).unwrap();
        let expected_line_mass = 2.0
            * (standard_normal_upper_tail(1.0).unwrap() - standard_normal_upper_tail(2.0).unwrap());
        assert!(
            (report.contributions[0].conditional_collision_probability - expected_line_mass).abs()
                < 1.0e-15
        );
        assert_eq!(
            report.contributions[1].conditional_collision_probability,
            0.0
        );
        assert_eq!(report.line_count, 2);
        assert_eq!(report.total_collision_intervals, 2);
        assert_eq!(report.lines_with_no_collision_interval, 1);
        assert!((report.estimate - expected_line_mass / 2.0).abs() < 1.0e-15);
        let expected_radius = ((1.0_f64 / 0.05).ln() / 4.0).sqrt();
        assert!((report.hoeffding_radius - expected_radius).abs() < 1.0e-15);
        assert_eq!(report.inference_unit, "independent_transverse_line");
    }

    #[test]
    fn rejects_incomplete_or_unresolved_interval_sets() {
        let lines = vec![
            TransverseLine {
                line_id: "line-a".into(),
                transverse_sample_id: "v-a".into(),
                transverse_coordinates: vec![0.0],
                collision_intervals: vec![],
            },
            TransverseLine {
                line_id: "line-b".into(),
                transverse_sample_id: "v-b".into(),
                transverse_coordinates: vec![1.0],
                collision_intervals: vec![],
            },
        ];
        let mut bad = evidence();
        bad.interval_sets_are_complete = false;
        assert_eq!(
            estimate_gaussian_line_probability(&frame(), &lines, 0.05, &bad),
            Err(LineSamplingError::IncompleteIntervalEnumeration)
        );
        let mut bad = evidence();
        bad.no_unresolved_intervals = false;
        assert_eq!(
            estimate_gaussian_line_probability(&frame(), &lines, 0.05, &bad),
            Err(LineSamplingError::UnresolvedCollisionIntervals)
        );
    }

    #[test]
    fn rejects_overlapping_unsorted_intervals_and_duplicate_lines() {
        let overlapping = vec![
            TransverseLine {
                line_id: "line-a".into(),
                transverse_sample_id: "v-a".into(),
                transverse_coordinates: vec![0.0],
                collision_intervals: vec![
                    CollisionInterval {
                        lower: -2.0,
                        upper: 0.5,
                    },
                    CollisionInterval {
                        lower: 0.0,
                        upper: 1.0,
                    },
                ],
            },
            TransverseLine {
                line_id: "line-b".into(),
                transverse_sample_id: "v-b".into(),
                transverse_coordinates: vec![1.0],
                collision_intervals: vec![],
            },
        ];
        assert!(matches!(
            estimate_gaussian_line_probability(&frame(), &overlapping, 0.05, &evidence()),
            Err(LineSamplingError::IntervalsOverlapOrAreUnsorted { .. })
        ));

        let mut duplicate = overlapping.clone();
        duplicate[0].collision_intervals.clear();
        duplicate[1].collision_intervals.clear();
        duplicate[1].line_id = duplicate[0].line_id.clone();
        assert_eq!(
            estimate_gaussian_line_probability(&frame(), &duplicate, 0.05, &evidence()),
            Err(LineSamplingError::DuplicateLineId("line-a".into()))
        );
    }

    #[test]
    fn rejects_invalid_frame_and_reused_direction_training_samples() {
        let lines = vec![
            TransverseLine {
                line_id: "line-a".into(),
                transverse_sample_id: "v-a".into(),
                transverse_coordinates: vec![0.0],
                collision_intervals: vec![],
            },
            TransverseLine {
                line_id: "line-b".into(),
                transverse_sample_id: "v-b".into(),
                transverse_coordinates: vec![0.0],
                collision_intervals: vec![],
            },
        ];
        let mut bad_frame = frame();
        bad_frame.direction[0] = 2.0;
        assert!(matches!(
            estimate_gaussian_line_probability(&bad_frame, &lines, 0.05, &evidence()),
            Err(LineSamplingError::DirectionNotUnitLength(_))
        ));
        let mut bad_evidence = evidence();
        bad_evidence.evaluation_lines_are_disjoint_from_direction_training = false;
        assert_eq!(
            estimate_gaussian_line_probability(&frame(), &lines, 0.05, &bad_evidence),
            Err(LineSamplingError::EvaluationSetReusesDirectionTraining)
        );
    }

    #[test]
    fn rejects_tail_argument_outside_qualified_numerical_range() {
        assert_eq!(
            standard_normal_upper_tail(38.0),
            Err(LineSamplingError::NumericalTailRangeExceeded(38.0))
        );
    }
}

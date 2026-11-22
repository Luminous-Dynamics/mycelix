#![forbid(unsafe_code)]

//! Anytime-valid confidence sequences for bounded observations.
//!
//! For Z_t in [0,1], candidate m in (0,1), and fixed bet fraction r in (0,1],
//! the factor is F_t = 1 - r*(Z_t-m)/(1-m). Under the conditional null
//! E[Z_t | F_(t-1)] >= m, this factor is nonnegative and has conditional
//! expectation <= 1. A fixed mixture of products is a nonnegative
//! supermartingale; Ville's inequality makes the test time-uniform.
//!
//! Inverting the test yields an upper confidence sequence for the largest
//! uniform lower cap on the conditional mean. For IID observations this is
//! an upper confidence sequence for the common mean. Reflecting Z -> 1-Z
//! gives the corresponding lower sequence.
//!
//! This is statistical inference only. It does not validate the physical
//! model, target density, proposal density, independence or event oracle.

use std::fmt;

const DEFAULT_BETS: [f64; 9] = [0.01, 0.03, 0.10, 0.25, 0.50, 0.75, 0.90, 0.99, 1.0];
const BISECTION_STEPS: usize = 100;
const BISECTION_TOLERANCE: f64 = 2.0e-15;

#[derive(Clone, Debug, PartialEq)]
pub struct BettingGrid {
    pub id: String,
    pub fractions: Vec<f64>,
}

impl BettingGrid {
    pub fn default_grid() -> Self {
        Self {
            id: "bounded-mean-bets-001-003-010-025-050-075-090-099-100-v1".into(),
            fractions: DEFAULT_BETS.to_vec(),
        }
    }

    pub fn validate(&self) -> Result<(), SequenceError> {
        if !valid_identifier(&self.id) {
            return Err(SequenceError::InvalidIdentifier("bet_grid_id"));
        }
        if self.fractions.is_empty() {
            return Err(SequenceError::EmptyBetGrid);
        }
        let mut previous = 0.0;
        for value in &self.fractions {
            if !value.is_finite() || *value <= 0.0 || *value > 1.0 {
                return Err(SequenceError::InvalidBetFraction(*value));
            }
            if *value <= previous {
                return Err(SequenceError::BetGridMustBeStrictlyIncreasing);
            }
            previous = *value;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConditionalMeanEvidence {
    pub sampling_design_id: String,
    pub conditional_mean_model_id: String,
    pub evidence_ref: String,
    pub sample_order_digest: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoundedMeanPoint {
    pub time: usize,
    pub observed_mean: f64,
    pub lower_confidence_bound: f64,
    pub upper_confidence_bound: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoundedMeanConfidenceSequenceReport {
    pub method: &'static str,
    pub range_lower: f64,
    pub range_upper: f64,
    pub delta: f64,
    pub two_sided_confidence_level: f64,
    pub one_sided_delta: f64,
    pub bet_grid_id: String,
    pub bet_fractions: Vec<f64>,
    pub sampling_design_id: String,
    pub conditional_mean_model_id: String,
    pub conditional_mean_evidence_ref: String,
    pub sample_order_digest: String,
    pub null_hypothesis: &'static str,
    pub estimand_scope: &'static str,
    pub points: Vec<BoundedMeanPoint>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SequenceError {
    InvalidRange {
        lower: f64,
        upper: f64,
    },
    InvalidDelta(f64),
    InvalidIdentifier(&'static str),
    EmptyBetGrid,
    InvalidBetFraction(f64),
    BetGridMustBeStrictlyIncreasing,
    NonFiniteObservation {
        index: usize,
        value: f64,
    },
    ObservationOutsideRange {
        index: usize,
        value: f64,
        lower: f64,
        upper: f64,
    },
}

impl fmt::Display for SequenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRange { lower, upper } => write!(
                f,
                "range must be finite with lower < upper; got [{lower}, {upper}]"
            ),
            Self::InvalidDelta(value) => write!(
                f,
                "delta must be finite and strictly between zero and one; got {value}"
            ),
            Self::InvalidIdentifier(field) => write!(
                f,
                "{field} must be non-empty and use only ASCII letters, digits, dot, underscore, colon, slash and hyphen"
            ),
            Self::EmptyBetGrid => write!(f, "bet grid must contain at least one fraction"),
            Self::InvalidBetFraction(value) => {
                write!(f, "bet fraction must be finite and in (0,1]; got {value}")
            }
            Self::BetGridMustBeStrictlyIncreasing => {
                write!(f, "bet fractions must be strictly increasing and unique")
            }
            Self::NonFiniteObservation { index, value } => {
                write!(f, "observation {index} is non-finite: {value}")
            }
            Self::ObservationOutsideRange {
                index,
                value,
                lower,
                upper,
            } => write!(
                f,
                "observation {index}={value} lies outside declared range [{lower},{upper}]"
            ),
        }
    }
}

impl std::error::Error for SequenceError {}

fn valid_identifier(value: &str) -> bool {
    !value.trim().is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"._:/-".contains(&byte))
}

fn validate_delta(delta: f64) -> Result<(), SequenceError> {
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(SequenceError::InvalidDelta(delta));
    }
    Ok(())
}

/// Log of a fixed equal-weight mixture e-value for testing
/// H0: E[Z_t | F_(t-1)] >= candidate_mean for every t.
///
/// The mixture is formed from fixed bet fractions. All component factors are
/// nonnegative for Z in [0,1], candidate_mean in (0,1), and r in (0,1].
pub fn log_upper_test_e_value(
    normalized_observations: &[f64],
    candidate_mean: f64,
    grid: &BettingGrid,
) -> Result<f64, SequenceError> {
    grid.validate()?;
    if !candidate_mean.is_finite() || candidate_mean <= 0.0 || candidate_mean >= 1.0 {
        return Err(SequenceError::InvalidBetFraction(candidate_mean));
    }

    let mut component_logs = Vec::with_capacity(grid.fractions.len());
    for fraction in &grid.fractions {
        let mut log_product = 0.0;
        for (index, value) in normalized_observations.iter().enumerate() {
            if !value.is_finite() {
                return Err(SequenceError::NonFiniteObservation {
                    index,
                    value: *value,
                });
            }
            if !(0.0..=1.0).contains(value) {
                return Err(SequenceError::ObservationOutsideRange {
                    index,
                    value: *value,
                    lower: 0.0,
                    upper: 1.0,
                });
            }

            let factor = 1.0 - fraction * (value - candidate_mean) / (1.0 - candidate_mean);
            if factor <= 0.0 {
                log_product = f64::NEG_INFINITY;
                break;
            }
            log_product += factor.ln();
        }
        component_logs.push(log_product);
    }

    Ok(log_mean_exp(&component_logs))
}

fn log_mean_exp(values: &[f64]) -> f64 {
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
    maximum + sum.ln() - (values.len() as f64).ln()
}

/// Upper confidence bound for the normalized common mean at one prefix.
///
/// Under the per-step conditional null E[Z_t | F_(t-1)] >= m, the probability
/// that this test ever rejects a true m is at most delta (Ville's inequality).
/// The result is an upper bound on the largest uniform lower cap on conditional
/// means; it is an upper confidence bound for the common mean in the IID case.
pub fn upper_bound_at(
    normalized_observations: &[f64],
    delta: f64,
    grid: &BettingGrid,
) -> Result<f64, SequenceError> {
    validate_delta(delta)?;
    grid.validate()?;
    if normalized_observations.is_empty() {
        return Ok(1.0);
    }

    for (index, value) in normalized_observations.iter().enumerate() {
        if !value.is_finite() {
            return Err(SequenceError::NonFiniteObservation {
                index,
                value: *value,
            });
        }
        if !(0.0..=1.0).contains(value) {
            return Err(SequenceError::ObservationOutsideRange {
                index,
                value: *value,
                lower: 0.0,
                upper: 1.0,
            });
        }
    }

    // No evidence can rule out a mean of one if every observed value equals one.
    if normalized_observations.iter().all(|value| *value == 1.0) {
        return Ok(1.0);
    }

    let threshold = -delta.ln();
    let mut low = 0.0; // m=0 is never rejected by this upper-tail test.
    let mut high = 1.0; // at least one observation <1 makes the limit rejected.
    for _ in 0..BISECTION_STEPS {
        let middle = low + (high - low) * 0.5;
        if middle == low || middle == high {
            break;
        }
        let log_e = log_upper_test_e_value(normalized_observations, middle, grid)?;
        if log_e >= threshold {
            high = middle;
        } else {
            low = middle;
        }
        if high - low <= BISECTION_TOLERANCE * high.max(1.0) {
            break;
        }
    }
    Ok(high.clamp(0.0, 1.0))
}

/// Lower confidence bound, obtained by reflecting Z to 1-Z.
pub fn lower_bound_at(
    normalized_observations: &[f64],
    delta: f64,
    grid: &BettingGrid,
) -> Result<f64, SequenceError> {
    let reflected = normalized_observations
        .iter()
        .map(|value| 1.0 - value)
        .collect::<Vec<_>>();
    Ok(1.0 - upper_bound_at(&reflected, delta, grid)?)
}

/// Compute a two-sided anytime confidence sequence for a bounded variable.
///
/// The conditional nulls are E[Z_t | F_(t-1)] >= m (upper bound) and
/// E[1-Z_t | F_(t-1)] >= 1-u (lower bound). Under IID observations this is
/// a simultaneous confidence sequence for the common mean. With dependence,
/// it bounds uniform conditional-mean caps and does not generally bound an
/// arbitrary heterogeneous average.
pub fn bounded_mean_confidence_sequence(
    observations: &[f64],
    range_lower: f64,
    range_upper: f64,
    delta: f64,
    grid: &BettingGrid,
    evidence: ConditionalMeanEvidence,
) -> Result<BoundedMeanConfidenceSequenceReport, SequenceError> {
    validate_delta(delta)?;
    grid.validate()?;
    if !range_lower.is_finite() || !range_upper.is_finite() || range_lower >= range_upper {
        return Err(SequenceError::InvalidRange {
            lower: range_lower,
            upper: range_upper,
        });
    }
    for (field, value) in [
        ("sampling_design_id", &evidence.sampling_design_id),
        (
            "conditional_mean_model_id",
            &evidence.conditional_mean_model_id,
        ),
        ("conditional_mean_evidence_ref", &evidence.evidence_ref),
        ("sample_order_digest", &evidence.sample_order_digest),
    ] {
        if !valid_identifier(value) {
            return Err(SequenceError::InvalidIdentifier(field));
        }
    }

    let width = range_upper - range_lower;
    if !width.is_finite() || width <= 0.0 {
        return Err(SequenceError::InvalidRange {
            lower: range_lower,
            upper: range_upper,
        });
    }
    let mut normalized = Vec::with_capacity(observations.len());
    for (index, value) in observations.iter().enumerate() {
        if !value.is_finite() {
            return Err(SequenceError::NonFiniteObservation {
                index,
                value: *value,
            });
        }
        if *value < range_lower || *value > range_upper {
            return Err(SequenceError::ObservationOutsideRange {
                index,
                value: *value,
                lower: range_lower,
                upper: range_upper,
            });
        }
        normalized.push((*value - range_lower) / width);
    }

    let one_sided_delta = delta / 2.0;
    let mut points = Vec::with_capacity(observations.len());
    let mut running_sum = 0.0;
    for (index, value) in observations.iter().enumerate() {
        running_sum += *value;
        let prefix = &normalized[..=index];
        let upper_normalized = upper_bound_at(prefix, one_sided_delta, grid)?;
        let lower_normalized = lower_bound_at(prefix, one_sided_delta, grid)?;
        points.push(BoundedMeanPoint {
            time: index + 1,
            observed_mean: running_sum / (index + 1) as f64,
            lower_confidence_bound: range_lower + width * lower_normalized,
            upper_confidence_bound: range_lower + width * upper_normalized,
        });
    }

    Ok(BoundedMeanConfidenceSequenceReport {
        method: "finite_mixture_bounded_mean_betting_confidence_sequence",
        range_lower,
        range_upper,
        delta,
        two_sided_confidence_level: 1.0 - delta,
        one_sided_delta,
        bet_grid_id: grid.id.clone(),
        bet_fractions: grid.fractions.clone(),
        sampling_design_id: evidence.sampling_design_id,
        conditional_mean_model_id: evidence.conditional_mean_model_id,
        conditional_mean_evidence_ref: evidence.evidence_ref,
        sample_order_digest: evidence.sample_order_digest,
        null_hypothesis: "E[Z_t | F_(t-1)] >= m for every t (and reflected lower-tail null)",
        estimand_scope: "simultaneous_common_mean_iid; under dependence upper/lower bound uniform conditional-mean caps_not_arbitrary_heterogeneous_average",
        points,
        claim_ceiling: "time_uniform_statistical_bound_conditional_on_declared_conditional_mean_model_not_physical_model_or_operational_safety",
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> BettingGrid {
        BettingGrid::default_grid()
    }

    fn evidence() -> ConditionalMeanEvidence {
        ConditionalMeanEvidence {
            sampling_design_id: "iid-frozen-defensive-proposal-v1".into(),
            conditional_mean_model_id: "bounded-weighted-contribution-common-mean-v1".into(),
            evidence_ref: "fixture://conditional-mean-evidence/v1".into(),
            sample_order_digest: "0123456789abcdef".into(),
        }
    }

    #[test]
    fn empty_input_returns_full_declared_range() {
        let report =
            bounded_mean_confidence_sequence(&[], -10.0, 10.0, 0.05, &grid(), evidence()).unwrap();
        assert!(report.points.is_empty());
    }

    #[test]
    fn zero_weighted_events_do_not_create_zero_upper_bound() {
        let report =
            bounded_mean_confidence_sequence(&vec![0.0; 100], 0.0, 20.0, 0.05, &grid(), evidence())
                .unwrap();
        let final_point = report.points.last().unwrap();
        assert!(final_point.upper_confidence_bound > 0.0);
        assert!(final_point.lower_confidence_bound <= 0.0 + 1e-12);
    }

    #[test]
    fn repeated_lower_range_values_tighten_upper_sequence() {
        let report = bounded_mean_confidence_sequence(
            &vec![-10.0; 40],
            -10.0,
            10.0,
            0.05,
            &grid(),
            evidence(),
        )
        .unwrap();
        assert!(report.points.windows(2).all(|pair| {
            pair[1].upper_confidence_bound <= pair[0].upper_confidence_bound + 1e-12
        }));
        assert!(report.points.last().unwrap().upper_confidence_bound > -10.0);
    }

    #[test]
    fn repeated_upper_range_values_tighten_lower_sequence() {
        let report = bounded_mean_confidence_sequence(
            &vec![10.0; 40],
            -10.0,
            10.0,
            0.05,
            &grid(),
            evidence(),
        )
        .unwrap();
        assert!(report.points.windows(2).all(|pair| {
            pair[1].lower_confidence_bound >= pair[0].lower_confidence_bound - 1e-12
        }));
        assert!(report.points.last().unwrap().lower_confidence_bound < 10.0);
    }

    #[test]
    fn observed_values_widen_the_opposite_tail_as_expected() {
        let low = upper_bound_at(&[0.0; 20], 0.025, &grid()).unwrap();
        let with_large_values = upper_bound_at(&[0.0, 0.0, 1.0, 1.0, 0.0], 0.025, &grid()).unwrap();
        assert!(with_large_values > low);
    }

    #[test]
    fn scales_nonzero_range_without_changing_normalized_bounds() {
        let normalized = bounded_mean_confidence_sequence(
            &[0.0, 0.2, 0.4, 0.6],
            0.0,
            1.0,
            0.05,
            &grid(),
            evidence(),
        )
        .unwrap();
        let scaled = bounded_mean_confidence_sequence(
            &[10.0, 12.0, 14.0, 16.0],
            10.0,
            20.0,
            0.05,
            &grid(),
            evidence(),
        )
        .unwrap();
        let p = normalized.points.last().unwrap();
        let q = scaled.points.last().unwrap();
        assert!(
            (q.lower_confidence_bound - (10.0 + 10.0 * p.lower_confidence_bound)).abs() < 1e-10
        );
        assert!(
            (q.upper_confidence_bound - (10.0 + 10.0 * p.upper_confidence_bound)).abs() < 1e-10
        );
    }

    #[test]
    fn rejects_missing_evidence_range_and_out_of_support_values() {
        assert!(matches!(
            bounded_mean_confidence_sequence(&[0.0], 1.0, 1.0, 0.05, &grid(), evidence()),
            Err(SequenceError::InvalidRange { .. })
        ));
        assert!(matches!(
            bounded_mean_confidence_sequence(&[-0.01], 0.0, 1.0, 0.05, &grid(), evidence()),
            Err(SequenceError::ObservationOutsideRange { .. })
        ));
        let bad = ConditionalMeanEvidence {
            sampling_design_id: "design".into(),
            conditional_mean_model_id: "model".into(),
            evidence_ref: " ".into(),
            sample_order_digest: "digest".into(),
        };
        assert!(matches!(
            bounded_mean_confidence_sequence(&[0.5], 0.0, 1.0, 0.05, &grid(), bad),
            Err(SequenceError::InvalidIdentifier(
                "conditional_mean_evidence_ref"
            ))
        ));
    }

    fn log_crosses(values: &[f64], candidate_mean: f64, delta: f64) -> bool {
        log_upper_test_e_value(values, candidate_mean, &grid()).unwrap() >= -delta.ln()
    }

    fn enumerate_predictable(
        history: &mut Vec<f64>,
        mass: f64,
        p0: f64,
        delta: f64,
        horizon: usize,
        crossing_mass: &mut f64,
    ) {
        if log_crosses(history, p0, delta) {
            *crossing_mass += mass;
            return;
        }
        if history.len() == horizon {
            return;
        }

        // Two predictable Bernoulli processes whose conditional mean is always
        // at least p0, but whose probability varies with the observed history.
        let probability_one = if history.last() == Some(&1.0) {
            p0
        } else if history.len() % 2 == 0 {
            p0 + (1.0 - p0) * 0.5
        } else {
            p0 + (1.0 - p0) * 0.2
        };
        history.push(0.0);
        enumerate_predictable(
            history,
            mass * (1.0 - probability_one),
            p0,
            delta,
            horizon,
            crossing_mass,
        );
        history.pop();
        history.push(1.0);
        enumerate_predictable(
            history,
            mass * probability_one,
            p0,
            delta,
            horizon,
            crossing_mass,
        );
        history.pop();
    }

    #[test]
    fn optional_stopping_coverage_holds_for_predictable_bernoulli_process() {
        let p0 = 0.2;
        let delta = 0.05;
        let mut crossing_mass = 0.0;
        enumerate_predictable(&mut Vec::new(), 1.0, p0, delta, 12, &mut crossing_mass);
        assert!(
            crossing_mass <= delta + 1.0e-12,
            "crossing probability={crossing_mass}"
        );
    }

    #[test]
    fn rejects_overflowing_finite_endpoints() {
        assert!(matches!(
            bounded_mean_confidence_sequence(
                &[0.0],
                -f64::MAX,
                f64::MAX,
                0.05,
                &grid(),
                evidence()
            ),
            Err(SequenceError::InvalidRange { .. })
        ));
    }

    #[test]
    fn upper_test_e_value_is_non_decreasing_in_candidate_mean() {
        let values = [0.2, 0.8, 0.4, 0.6];
        let mut previous = f64::NEG_INFINITY;
        for candidate in [0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99] {
            let next = log_upper_test_e_value(&values, candidate, &grid()).unwrap();
            assert!(next + 1.0e-12 >= previous);
            previous = next;
        }
    }

    #[test]
    fn rejects_singular_candidate_mean_and_invalid_grid() {
        assert!(matches!(
            log_upper_test_e_value(&[0.5], 1.0, &grid()),
            Err(SequenceError::InvalidBetFraction(1.0))
        ));
        let bad_grid = BettingGrid {
            id: "bad-grid".into(),
            fractions: vec![0.5, 0.4],
        };
        assert_eq!(
            bad_grid.validate(),
            Err(SequenceError::BetGridMustBeStrictlyIncreasing)
        );
    }
}

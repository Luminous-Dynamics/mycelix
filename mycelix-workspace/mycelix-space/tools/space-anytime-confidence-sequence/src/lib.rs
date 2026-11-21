#![forbid(unsafe_code)]

//! Finite-mixture betting e-process for a one-sided upper bound on the
//! conditional Bernoulli violation probability. For candidate null p0 and
//! bet fraction r in (0, 1], the process bets on non-violations Y_t = 1-X_t
//! with increment 1 + (r / (1-p0)) * (Y_t - (1-p0)). Under the conditional null
//! P(X_t=1 | F_(t-1)) >= p0 for every t, each component is a nonnegative
//! supermartingale. A fixed mixture over r is also a nonnegative
//! supermartingale; Ville's inequality makes inversion time-uniform.
//!
//! The result upper-bounds the minimum conditional violation probability
//! across the declared sequence. Under IID Bernoulli trials this is the common
//! violation probability. It is not an upper bound on an arbitrary average
//! across heterogeneous conditional probabilities. Hidden common causes,
//! unobserved selection and a misspecified filtration remain unsupported.

use std::fmt;

const BISECTION_STEPS: usize = 100;
const GRID: [f64; 9] = [0.01, 0.03, 0.10, 0.25, 0.50, 0.75, 0.90, 0.99, 1.0];

#[derive(Clone, Debug, PartialEq)]
pub struct BetGrid {
    pub id: String,
    pub fractions: Vec<f64>,
}

impl BetGrid {
    pub fn default_grid() -> Self {
        Self {
            id: "finite-mixture-r-001-003-010-025-050-075-090-099-100-v1".into(),
            fractions: GRID.to_vec(),
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
        for fraction in &self.fractions {
            if !fraction.is_finite() || *fraction <= 0.0 || *fraction > 1.0 {
                return Err(SequenceError::InvalidBetFraction(*fraction));
            }
            if *fraction <= previous {
                return Err(SequenceError::BetGridMustBeStrictlyIncreasing);
            }
            previous = *fraction;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConditionalMeanEvidence {
    /// Identifier for the declared conditional-mean null and filtration.
    pub model_id: String,
    /// Immutable evidence reference justifying the conditional-mean model.
    pub evidence_ref: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConfidencePoint {
    pub time: usize,
    pub observed_violations: usize,
    pub empirical_rate: f64,
    pub upper_confidence_bound: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConfidenceSequenceReport {
    pub method: &'static str,
    pub delta: f64,
    pub one_sided_confidence_level: f64,
    pub bet_grid_id: String,
    pub bet_fractions: Vec<f64>,
    pub conditional_mean_model_id: String,
    pub conditional_mean_evidence_ref: String,
    pub null_hypothesis: &'static str,
    pub points: Vec<ConfidencePoint>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SequenceError {
    InvalidDelta(f64),
    InvalidIdentifier(&'static str),
    EmptyBetGrid,
    InvalidBetFraction(f64),
    BetGridMustBeStrictlyIncreasing,
    InvalidObservation { index: usize, value: u8 },
}

impl fmt::Display for SequenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDelta(value) => write!(f, "delta must be finite and strictly between 0 and 1; got {value}"),
            Self::InvalidIdentifier(field) => write!(f, "{field} must be non-empty and use only ASCII letters, digits, dot, underscore, colon, slash and hyphen"),
            Self::EmptyBetGrid => write!(f, "bet grid must contain at least one fraction"),
            Self::InvalidBetFraction(value) => write!(f, "bet fraction must be finite and in (0, 1]; got {value}"),
            Self::BetGridMustBeStrictlyIncreasing => write!(f, "bet fractions must be strictly increasing and unique"),
            Self::InvalidObservation { index, value } => write!(f, "observation at index {index} must be 0 or 1; got {value}"),
        }
    }
}

impl std::error::Error for SequenceError {}

fn valid_identifier(value: &str) -> bool {
    !value.trim().is_empty()
        && value.bytes().all(|b| {
            b.is_ascii_alphanumeric() || b"._:/-".contains(&b)
        })
}

/// Compute the log of the equal-weight mixture e-process for a fixed p0.
///
/// Each component uses a fixed lambda = r / p0 for this candidate null. Since
/// r <= 1, every increment is nonnegative for X in {0,1}.
pub fn log_mixture_e_value(
    observations: &[bool],
    p0: f64,
    grid: &BetGrid,
) -> Result<f64, SequenceError> {
    grid.validate()?;
    if !p0.is_finite() || p0 <= 0.0 || p0 >= 1.0 {
        return Err(SequenceError::InvalidBetFraction(p0));
    }

    let mut component_logs = Vec::with_capacity(grid.fractions.len());
    for fraction in &grid.fractions {
        let mut log_product = 0.0;
        for observation in observations {
            let increment_log = if *observation {
                // X=1 is a violation; Y=1-X=0 and the bet factor is 1-r.
                if *fraction == 1.0 {
                    f64::NEG_INFINITY
                } else {
                    (1.0 - fraction).ln()
                }
            } else {
                // X=0 is a non-violation; Y=1:
                // 1 + r*p0/(1-p0) = (1-p0 + r*p0)/(1-p0).
                ((1.0 - p0) + fraction * p0).ln() - (1.0 - p0).ln()
            };
            if increment_log == f64::NEG_INFINITY {
                log_product = f64::NEG_INFINITY;
                break;
            }
            log_product += increment_log;
        }
        component_logs.push(log_product);
    }

    Ok(log_mean_exp(&component_logs))
}

fn log_mean_exp(values: &[f64]) -> f64 {
    let max_value = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if max_value == f64::NEG_INFINITY {
        return f64::NEG_INFINITY;
    }
    if max_value == f64::INFINITY {
        return f64::INFINITY;
    }
    let sum: f64 = values.iter().map(|value| (value - max_value).exp()).sum();
    max_value + sum.ln() - (values.len() as f64).ln()
}

/// Return the time-uniform one-sided upper confidence bound after a prefix.
///
/// A candidate p0 is rejected when log(M_t(p0)) >= ln(1/delta). By Ville's
/// inequality, the probability that any time t rejects a true conditional
/// mean cap p0 is at most delta, under the declared conditional null.
pub fn upper_bound_at(
    observations: &[bool],
    delta: f64,
    grid: &BetGrid,
) -> Result<f64, SequenceError> {
    validate_delta(delta)?;
    grid.validate()?;
    if observations.is_empty() {
        return Ok(1.0);
    }
    let threshold = -delta.ln();

    // With all observations violating, the evidence never supports a smaller
    // upper bound; return one. Otherwise log M_t(p0) increases with p0.
    if observations.iter().all(|value| *value) {
        return Ok(1.0);
    }

    // The null for candidate p0 is P(X_t=1 | F_(t-1)) >= p0 at every step.
    // Frequent non-violations reject overly large p0. At p0=0, M <= 1;
    // with at least one non-violation M diverges as p0 approaches one.
    let mut low = 0.0; // not rejected
    let mut high = 1.0; // rejected in the limit
    for _ in 0..BISECTION_STEPS {
        let mid = low + (high - low) * 0.5;
        if mid == 0.0 || mid == 1.0 {
            high = mid.max(high.min(1.0));
            continue;
        }
        let log_e = log_mixture_e_value(observations, mid, grid)?;
        if log_e >= threshold {
            high = mid;
        } else {
            low = mid;
        }
    }
    Ok(high.clamp(0.0, 1.0))
}

pub fn confidence_sequence(
    observations: &[bool],
    delta: f64,
    grid: &BetGrid,
    evidence: ConditionalMeanEvidence,
) -> Result<ConfidenceSequenceReport, SequenceError> {
    validate_delta(delta)?;
    grid.validate()?;
    if !valid_identifier(&evidence.model_id) {
        return Err(SequenceError::InvalidIdentifier("conditional_mean_model_id"));
    }
    if !valid_identifier(&evidence.evidence_ref) {
        return Err(SequenceError::InvalidIdentifier("conditional_mean_evidence_ref"));
    }

    let mut points = Vec::with_capacity(observations.len());
    let mut violation_count = 0usize;
    for (index, observation) in observations.iter().enumerate() {
        if *observation {
            violation_count += 1;
        }
        let prefix = &observations[..=index];
        let upper = upper_bound_at(prefix, delta, grid)?;
        points.push(ConfidencePoint {
            time: index + 1,
            observed_violations: violation_count,
            empirical_rate: violation_count as f64 / (index + 1) as f64,
            upper_confidence_bound: upper,
        });
    }

    Ok(ConfidenceSequenceReport {
        method: "finite_mixture_bernoulli_betting_e_process",
        delta,
        one_sided_confidence_level: 1.0 - delta,
        bet_grid_id: grid.id.clone(),
        bet_fractions: grid.fractions.clone(),
        conditional_mean_model_id: evidence.model_id,
        conditional_mean_evidence_ref: evidence.evidence_ref,
        null_hypothesis: "P(X_t=1 | F_(t-1)) >= p0 for every t",
        points,
        claim_ceiling: "time_uniform_upper_bound_on_minimum_conditional_violation_probability_under_declared_model_not_physical_or_operational_safety",
    })
}

fn validate_delta(delta: f64) -> Result<(), SequenceError> {
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(SequenceError::InvalidDelta(delta));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> BetGrid { BetGrid::default_grid() }

    fn evidence() -> ConditionalMeanEvidence {
        ConditionalMeanEvidence {
            model_id: "conditional-bernoulli-cap-v1".into(),
            evidence_ref: "fixture://conditional-mean-model/v1".into(),
        }
    }

    fn log_crosses(observations: &[bool], p0: f64, delta: f64, grid: &BetGrid) -> bool {
        log_mixture_e_value(observations, p0, grid).unwrap() >= -delta.ln()
    }

    #[test]
    fn empty_prefix_has_no_information_and_upper_bound_one() {
        assert_eq!(upper_bound_at(&[], 0.05, &grid()).unwrap(), 1.0);
    }

    #[test]
    fn repeated_non_violations_lower_the_upper_bound_but_not_to_zero() {
        let short = upper_bound_at(&[false; 5], 0.05, &grid()).unwrap();
        let long = upper_bound_at(&[false; 100], 0.05, &grid()).unwrap();
        assert!(long < short);
        assert!(long > 0.0);
    }

    #[test]
    fn observed_violations_do_not_create_false_reassurance() {
        let no_violations = upper_bound_at(&[false; 20], 0.05, &grid()).unwrap();
        let with_violations = upper_bound_at(&[false, false, true, false, true], 0.05, &grid()).unwrap();
        assert!(with_violations > no_violations);
    }

    #[test]
    fn upper_bound_is_nonincreasing_in_observation_time_for_this_fixed_grid() {
        let report = confidence_sequence(
            &[true, false, true, false, true, false, false, true, false, true],
            0.05,
            &grid(),
            evidence(),
        ).unwrap();
        assert!(report.points.windows(2).all(|pair| {
            pair[1].upper_confidence_bound <= pair[0].upper_confidence_bound + 1e-13
        }));
    }

    #[test]
    fn evidence_and_grid_are_bound_to_the_report() {
        let report = confidence_sequence(&[true, false], 0.05, &grid(), evidence()).unwrap();
        assert_eq!(report.bet_grid_id, grid().id);
        assert_eq!(report.conditional_mean_model_id, "conditional-bernoulli-cap-v1");
        assert_eq!(report.conditional_mean_evidence_ref, "fixture://conditional-mean-model/v1");
        assert_eq!(report.null_hypothesis, "P(X_t=1 | F_(t-1)) >= p0 for every t");
    }

    #[test]
    fn rejects_invalid_delta_and_unsafe_evidence() {
        assert!(matches!(
            upper_bound_at(&[true], 1.0, &grid()),
            Err(SequenceError::InvalidDelta(_))
        ));
        let bad = ConditionalMeanEvidence {
            model_id: "valid".into(),
            evidence_ref: "evidence ref".into(),
        };
        assert!(matches!(
            confidence_sequence(&[true], 0.05, &grid(), bad),
            Err(SequenceError::InvalidIdentifier("conditional_mean_evidence_ref"))
        ));
    }

    #[test]
    fn fixed_p0_e_process_is_non_decreasing_when_p0_increases() {
        let observations = [true, false, true, true, false];
        let mut last = f64::INFINITY;
        for p0 in [0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
            let value = log_mixture_e_value(&observations, p0, &grid()).unwrap();
            assert!(value + 1e-12 >= last);
            last = value;
        }
    }

    // Enumerate all paths of a predictable history-dependent Bernoulli process.
    // Every conditional violation probability is >= p0, so the e-process
    // should cross on at most delta probability mass over the finite horizon.
    fn enumerate_predictable(
        history: &mut Vec<bool>,
        mass: f64,
        p0: f64,
        delta: f64,
        horizon: usize,
        crossed_mass: &mut f64,
    ) {
        if history.len() == horizon {
            return;
        }
        if log_crosses(history, p0, delta, &grid()) {
            *crossed_mass += mass;
            return;
        }
        let q = if history.last() == Some(&true) {
            p0
        } else if history.len() % 2 == 0 {
            p0 + (1.0 - p0) * 0.5
        } else {
            p0 + (1.0 - p0) * 0.2
        };
        history.push(false);
        enumerate_predictable(history, mass * (1.0 - q), p0, delta, horizon, crossed_mass);
        history.pop();
        history.push(true);
        enumerate_predictable(history, mass * q, p0, delta, horizon, crossed_mass);
        history.pop();
    }

    #[test]
    fn optional_stopping_coverage_holds_for_predictable_bounded_mean_process() {
        let p0 = 0.2;
        let delta = 0.05;
        let mut crossed_mass = 0.0;
        enumerate_predictable(&mut Vec::new(), 1.0, p0, delta, 10, &mut crossed_mass);
        assert!(
            crossed_mass <= delta + 1e-12,
            "crossing probability {crossed_mass} exceeds delta {delta}"
        );
    }

    #[test]
    fn optional_stopping_coverage_holds_for_iid_reference_process() {
        let p0 = 0.3;
        let delta = 0.05;
        let mut crossed_mass = 0.0;
        fn walk(
            history: &mut Vec<bool>,
            mass: f64,
            p0: f64,
            delta: f64,
            horizon: usize,
            crossed_mass: &mut f64,
        ) {
            if history.len() == horizon { return; }
            if log_crosses(history, p0, delta, &grid()) {
                *crossed_mass += mass;
                return;
            }
            history.push(false);
            walk(history, mass * (1.0 - p0), p0, delta, horizon, crossed_mass);
            history.pop();
            history.push(true);
            walk(history, mass * p0, p0, delta, horizon, crossed_mass);
            history.pop();
        }
        walk(&mut Vec::new(), 1.0, p0, delta, 12, &mut crossed_mass);
        assert!(crossed_mass <= delta + 1e-12);
    }
}

#![forbid(unsafe_code)]

//! Exact one-sided Clopper-Pearson upper confidence bounds for IID Bernoulli
//! violation trials.
//!
//! This crate does not establish that an orbit-risk model is calibrated, that
//! MCTS samples are independent, or that a spacecraft action is safe. The
//! caller must supply an evidence reference for the IID sampling assumption.

use std::fmt;

const MAX_EXACT_INTEGER: u64 = 9_007_199_254_740_992;
const MAX_BETA_ITERATIONS: usize = 20_000;
const BETA_EPSILON: f64 = 3.0e-14;
const BETA_FPMIN: f64 = 1.0e-300;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IidBernoulliEvidence {
    pub sampling_design_id: String,
    pub evidence_ref: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ClopperPearsonReport {
    pub method: &'static str,
    pub violations: u64,
    pub trials: u64,
    pub empirical_violation_rate: f64,
    pub confidence_level: f64,
    pub one_sided_alpha: f64,
    pub upper_confidence_bound: f64,
    pub sampling_design_id: String,
    pub independence_evidence_ref: String,
    pub probability_assumption: &'static str,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum BoundError {
    ZeroTrials,
    ViolationsExceedTrials { violations: u64, trials: u64 },
    TrialCountExceedsExactFloatRange { trials: u64, max_supported: u64 },
    InvalidConfidenceLevel { confidence_level: f64 },
    MissingSamplingDesignId,
    MissingIndependenceEvidenceRef,
    UnsafeEvidenceIdentifier,
    NumericalConvergenceFailure,
}

impl fmt::Display for BoundError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroTrials => write!(f, "trial count must be greater than zero"),
            Self::ViolationsExceedTrials { violations, trials } => write!(f, "violations ({violations}) exceed trials ({trials})"),
            Self::TrialCountExceedsExactFloatRange { trials, max_supported } => write!(f, "trial count {trials} exceeds exact-integer numerical limit {max_supported}"),
            Self::InvalidConfidenceLevel { confidence_level } => write!(f, "confidence level must be finite and strictly between zero and one; got {confidence_level}"),
            Self::MissingSamplingDesignId => write!(f, "an IID sampling-design identifier is required"),
            Self::MissingIndependenceEvidenceRef => write!(f, "an evidence reference justifying IID Bernoulli trials is required"),
            Self::UnsafeEvidenceIdentifier => write!(f, "evidence identifiers may contain only ASCII letters, digits, dot, underscore, colon, slash and hyphen"),
            Self::NumericalConvergenceFailure => write!(f, "regularized incomplete beta evaluation did not converge"),
        }
    }
}

impl std::error::Error for BoundError {}

/// Exact one-sided Clopper-Pearson upper confidence bound.
///
/// confidence_level is the statistical confidence (for example 0.95).
/// The returned upper bound must be compared with a separate operator/policy
/// chance threshold. This is conditional on an IID Bernoulli design; the
/// supplied evidence reference is a caller attestation, not proof of IID.
pub fn clopper_pearson_upper(
    violations: u64,
    trials: u64,
    confidence_level: f64,
    evidence: IidBernoulliEvidence,
) -> Result<ClopperPearsonReport, BoundError> {
    if trials == 0 {
        return Err(BoundError::ZeroTrials);
    }
    if violations > trials {
        return Err(BoundError::ViolationsExceedTrials { violations, trials });
    }
    if trials > MAX_EXACT_INTEGER {
        return Err(BoundError::TrialCountExceedsExactFloatRange {
            trials,
            max_supported: MAX_EXACT_INTEGER,
        });
    }
    if !confidence_level.is_finite() || confidence_level <= 0.0 || confidence_level >= 1.0 {
        return Err(BoundError::InvalidConfidenceLevel { confidence_level });
    }
    if evidence.sampling_design_id.trim().is_empty() {
        return Err(BoundError::MissingSamplingDesignId);
    }
    if evidence.evidence_ref.trim().is_empty() {
        return Err(BoundError::MissingIndependenceEvidenceRef);
    }
    if !safe_identifier(&evidence.sampling_design_id) || !safe_identifier(&evidence.evidence_ref) {
        return Err(BoundError::UnsafeEvidenceIdentifier);
    }

    let alpha = 1.0 - confidence_level;
    let upper = if violations == trials {
        1.0
    } else if violations == 0 {
        // Closed form for Beta(1, n): 1 - alpha^(1/n).
        -((alpha.ln() / trials as f64).exp_m1())
    } else {
        beta_quantile(
            confidence_level,
            violations as f64 + 1.0,
            (trials - violations) as f64,
        )?
    };

    Ok(ClopperPearsonReport {
        method: "clopper_pearson_one_sided_exact",
        violations,
        trials,
        empirical_violation_rate: violations as f64 / trials as f64,
        confidence_level,
        one_sided_alpha: alpha,
        upper_confidence_bound: upper.clamp(0.0, 1.0),
        sampling_design_id: evidence.sampling_design_id,
        independence_evidence_ref: evidence.evidence_ref,
        probability_assumption: "independent_identically_distributed_bernoulli_trials_with_common_violation_probability",
        claim_ceiling: "conditional_statistical_bound_only_not_model_calibration_or_operational_safety",
    })
}

fn safe_identifier(value: &str) -> bool {
    value.bytes().all(|byte| {
        byte.is_ascii_alphanumeric() || b"._:/-".contains(&byte)
    })
}

fn beta_quantile(probability: f64, a: f64, b: f64) -> Result<f64, BoundError> {
    let mut low = 0.0;
    let mut high = 1.0;
    for _ in 0..160 {
        let mid = low + (high - low) * 0.5;
        let cdf = regularized_incomplete_beta(mid, a, b)?;
        if cdf < probability {
            low = mid;
        } else {
            high = mid;
        }
        if high - low <= 2.0e-15 * high.max(1.0e-12) {
            break;
        }
    }
    Ok(high)
}

fn regularized_incomplete_beta(x: f64, a: f64, b: f64) -> Result<f64, BoundError> {
    if x <= 0.0 { return Ok(0.0); }
    if x >= 1.0 { return Ok(1.0); }
    if !(a > 0.0 && b > 0.0) {
        return Err(BoundError::NumericalConvergenceFailure);
    }
    let log_bt = ln_gamma(a + b) - ln_gamma(a) - ln_gamma(b)
        + a * x.ln() + b * (-x).ln_1p();
    let bt = log_bt.exp();
    let value = if x < (a + 1.0) / (a + b + 2.0) {
        bt * beta_continued_fraction(a, b, x)? / a
    } else {
        1.0 - bt * beta_continued_fraction(b, a, 1.0 - x)? / b
    };
    Ok(value.clamp(0.0, 1.0))
}

fn beta_continued_fraction(a: f64, b: f64, x: f64) -> Result<f64, BoundError> {
    let qab = a + b;
    let qap = a + 1.0;
    let qam = a - 1.0;
    let mut c = 1.0;
    let mut d = 1.0 - qab * x / qap;
    if d.abs() < BETA_FPMIN { d = BETA_FPMIN; }
    d = 1.0 / d;
    let mut h = d;

    for m in 1..=MAX_BETA_ITERATIONS {
        let m_f = m as f64;
        let m2 = 2.0 * m_f;
        let mut aa = m_f * (b - m_f) * x / ((qam + m2) * (a + m2));
        d = 1.0 + aa * d;
        if d.abs() < BETA_FPMIN { d = BETA_FPMIN; }
        c = 1.0 + aa / c;
        if c.abs() < BETA_FPMIN { c = BETA_FPMIN; }
        d = 1.0 / d;
        h *= d * c;

        aa = -(a + m_f) * (qab + m_f) * x / ((a + m2) * (qap + m2));
        d = 1.0 + aa * d;
        if d.abs() < BETA_FPMIN { d = BETA_FPMIN; }
        c = 1.0 + aa / c;
        if c.abs() < BETA_FPMIN { c = BETA_FPMIN; }
        d = 1.0 / d;
        let delta = d * c;
        h *= delta;

        if (delta - 1.0).abs() < BETA_EPSILON {
            return Ok(h);
        }
    }
    Err(BoundError::NumericalConvergenceFailure)
}

/// Lanczos log-gamma approximation; arguments in this implementation are >= 1.
fn ln_gamma(z: f64) -> f64 {
    const C: [f64; 9] = [
        0.999_999_999_999_809_93,
        676.520_368_121_885_1,
        -1_259.139_216_722_402_8,
        771.323_428_777_653_13,
        -176.615_029_162_140_59,
        12.507_343_278_686_905,
        -0.138_571_095_265_720_12,
        9.984_369_578_019_571_6e-6,
        1.505_632_735_149_311_6e-7,
    ];
    let z1 = z - 1.0;
    let mut x = C[0];
    for (i, coefficient) in C.iter().enumerate().skip(1) {
        x += coefficient / (z1 + i as f64);
    }
    let t = z1 + 7.5;
    0.918_938_533_204_672_7 + (z1 + 0.5) * t.ln() - t + x.ln()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence() -> IidBernoulliEvidence {
        IidBernoulliEvidence {
            sampling_design_id: "frozen-independent-seed-design-v1".into(),
            evidence_ref: "fixture://sampling-design/independent-seeds-v1".into(),
        }
    }

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!((actual - expected).abs() <= tolerance,
            "actual={actual:.16}, expected={expected:.16}, tolerance={tolerance}");
    }

    #[test]
    fn reproduces_one_violation_in_twenty_seven_upper_bound() {
        let result = clopper_pearson_upper(1, 27, 0.95, evidence()).unwrap();
        close(result.empirical_violation_rate, 1.0 / 27.0, 1.0e-14);
        close(result.upper_confidence_bound, 0.163_974_394_745_307_17, 2.0e-10);
    }

    #[test]
    fn reproduces_zero_violations_in_one_hundred_upper_bound() {
        let result = clopper_pearson_upper(0, 100, 0.95, evidence()).unwrap();
        close(result.upper_confidence_bound, 0.029_513_049_607_039_925, 1.0e-13);
    }

    #[test]
    fn all_trials_violating_returns_one() {
        assert_eq!(clopper_pearson_upper(100, 100, 0.95, evidence()).unwrap().upper_confidence_bound, 1.0);
    }

    #[test]
    fn upper_bound_increases_with_observed_violations() {
        let low = clopper_pearson_upper(1, 100, 0.95, evidence()).unwrap();
        let high = clopper_pearson_upper(5, 100, 0.95, evidence()).unwrap();
        assert!(high.upper_confidence_bound > low.upper_confidence_bound);
    }

    #[test]
    fn zero_violation_upper_bound_decreases_with_more_trials() {
        let small = clopper_pearson_upper(0, 100, 0.95, evidence()).unwrap();
        let large = clopper_pearson_upper(0, 1_000, 0.95, evidence()).unwrap();
        assert!(large.upper_confidence_bound < small.upper_confidence_bound);
    }

    #[test]
    fn matches_independent_midrange_reference_vectors() {
        for (violations, trials, expected) in [
            (5, 100, 0.102_253_377_643_274_5),
            (50, 100, 0.586_378_285_369_088_2),
            (90, 100, 0.944_736_762_317_13),
            (12, 345, 0.055_746_923_150_005_88),
        ] {
            let report = clopper_pearson_upper(violations, trials, 0.95, evidence()).unwrap();
            close(report.upper_confidence_bound, expected, 3.0e-10);
        }
    }

    #[test]
    fn remains_numerically_stable_near_one() {
        let report = clopper_pearson_upper(999, 1_000, 0.95, evidence()).unwrap();
        close(report.upper_confidence_bound, 0.999_948_708_021_091, 3.0e-10);
    }

    #[test]
    fn higher_confidence_level_never_reduces_upper_bound() {
        let ninety = clopper_pearson_upper(5, 100, 0.90, evidence()).unwrap();
        let ninety_five = clopper_pearson_upper(5, 100, 0.95, evidence()).unwrap();
        assert!(ninety_five.upper_confidence_bound > ninety.upper_confidence_bound);
    }

    #[test]
    fn rejects_zero_trials() {
        assert_eq!(clopper_pearson_upper(0, 0, 0.95, evidence()), Err(BoundError::ZeroTrials));
    }

    #[test]
    fn rejects_violations_greater_than_trials() {
        assert_eq!(clopper_pearson_upper(2, 1, 0.95, evidence()), Err(BoundError::ViolationsExceedTrials { violations: 2, trials: 1 }));
    }

    #[test]
    fn rejects_invalid_confidence_level() {
        assert!(matches!(clopper_pearson_upper(0, 100, 1.0, evidence()), Err(BoundError::InvalidConfidenceLevel { .. })));
    }

    #[test]
    fn rejects_empty_independence_evidence() {
        let missing = IidBernoulliEvidence { sampling_design_id: "design-1".into(), evidence_ref: " ".into() };
        assert_eq!(clopper_pearson_upper(0, 100, 0.95, missing), Err(BoundError::MissingIndependenceEvidenceRef));
    }

    #[test]
    fn rejects_unsafe_evidence_identifier() {
        let unsafe_evidence = IidBernoulliEvidence { sampling_design_id: "design-1".into(), evidence_ref: "ref with spaces".into() };
        assert_eq!(clopper_pearson_upper(0, 100, 0.95, unsafe_evidence), Err(BoundError::UnsafeEvidenceIdentifier));
    }

    #[test]
    fn beta_cdf_uniform_reference_case() {
        close(regularized_incomplete_beta(0.5, 1.0, 1.0).unwrap(), 0.5, 1.0e-14);
    }
}

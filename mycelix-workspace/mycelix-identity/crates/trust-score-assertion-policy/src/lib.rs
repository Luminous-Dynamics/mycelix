// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Dependency-free structural trust-score assertion policy.
//!
//! This crate owns the numeric theorem shared by trust-credential admission and
//! historical re-audit. It intentionally performs no proof verification, network
//! I/O, authorization, or downstream trust weighting.
//!
//! The current V1 credential representation stores score bounds as `f32`, while
//! coordinator tier derivation converts each bound to `f64` before computing the
//! midpoint. The functions here preserve that exact derivation so all callers
//! agree on one deterministic structural rule.

#![forbid(unsafe_code)]

/// Canonical structural trust tier vocabulary for score-range validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TrustTierBandV1 {
    Observer,
    Basic,
    Standard,
    Elevated,
    Guardian,
}

impl TrustTierBandV1 {
    /// Minimum score associated with this tier.
    pub const fn min_score(self) -> f64 {
        match self {
            Self::Observer => 0.0,
            Self::Basic => 0.3,
            Self::Standard => 0.4,
            Self::Elevated => 0.6,
            Self::Guardian => 0.8,
        }
    }

    /// Derive the structural tier from one score using the frozen V1 thresholds.
    pub fn from_score(score: f64) -> Self {
        if score >= 0.8 {
            Self::Guardian
        } else if score >= 0.6 {
            Self::Elevated
        } else if score >= 0.4 {
            Self::Standard
        } else if score >= 0.3 {
            Self::Basic
        } else {
            Self::Observer
        }
    }
}

/// Structural score-range defects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustScoreAssertionError {
    NonFiniteRange,
    OutOfBoundsRange,
    ReversedRange,
    ClaimedTierDoesNotMatchMidpoint,
}

/// Validate only the numeric score range and return its canonical midpoint tier.
///
/// The midpoint intentionally matches the existing coordinator calculation:
/// `(lower as f64 + upper as f64) / 2.0`.
pub fn validate_score_range_v1(
    lower: f32,
    upper: f32,
) -> Result<TrustTierBandV1, TrustScoreAssertionError> {
    if !lower.is_finite() || !upper.is_finite() {
        return Err(TrustScoreAssertionError::NonFiniteRange);
    }

    if !(0.0..=1.0).contains(&lower) || !(0.0..=1.0).contains(&upper) {
        return Err(TrustScoreAssertionError::OutOfBoundsRange);
    }

    if lower > upper {
        return Err(TrustScoreAssertionError::ReversedRange);
    }

    let midpoint = (lower as f64 + upper as f64) / 2.0;
    Ok(TrustTierBandV1::from_score(midpoint))
}

/// Validate a score range and require the claimed tier to equal the canonical
/// midpoint-derived tier.
pub fn validate_score_assertion_v1(
    lower: f32,
    upper: f32,
    claimed_tier: TrustTierBandV1,
) -> Result<(), TrustScoreAssertionError> {
    let derived = validate_score_range_v1(lower, upper)?;
    if derived != claimed_tier {
        return Err(TrustScoreAssertionError::ClaimedTierDoesNotMatchMidpoint);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn representative_range_for_every_tier_is_accepted() {
        let cases = [
            (0.10, 0.20, TrustTierBandV1::Observer),
            (0.30, 0.39, TrustTierBandV1::Basic),
            (0.40, 0.59, TrustTierBandV1::Standard),
            (0.60, 0.79, TrustTierBandV1::Elevated),
            (0.80, 1.00, TrustTierBandV1::Guardian),
        ];

        for (lower, upper, tier) in cases {
            assert_eq!(validate_score_assertion_v1(lower, upper, tier), Ok(()));
        }
    }

    #[test]
    fn nan_lower_is_rejected() {
        assert_eq!(
            validate_score_range_v1(f32::NAN, 0.5),
            Err(TrustScoreAssertionError::NonFiniteRange)
        );
    }

    #[test]
    fn nan_upper_is_rejected() {
        assert_eq!(
            validate_score_range_v1(0.5, f32::NAN),
            Err(TrustScoreAssertionError::NonFiniteRange)
        );
    }

    #[test]
    fn positive_and_negative_infinity_are_rejected() {
        for (lower, upper) in [
            (f32::NEG_INFINITY, 0.5),
            (0.5, f32::INFINITY),
            (f32::INFINITY, f32::INFINITY),
        ] {
            assert_eq!(
                validate_score_range_v1(lower, upper),
                Err(TrustScoreAssertionError::NonFiniteRange)
            );
        }
    }

    #[test]
    fn out_of_bounds_ranges_are_rejected() {
        assert_eq!(
            validate_score_range_v1(-0.01, 0.5),
            Err(TrustScoreAssertionError::OutOfBoundsRange)
        );
        assert_eq!(
            validate_score_range_v1(0.5, 1.01),
            Err(TrustScoreAssertionError::OutOfBoundsRange)
        );
    }

    #[test]
    fn reversed_range_is_rejected() {
        assert_eq!(
            validate_score_range_v1(0.7, 0.6),
            Err(TrustScoreAssertionError::ReversedRange)
        );
    }

    #[test]
    fn overclaimed_guardian_tier_is_rejected() {
        assert_eq!(
            validate_score_assertion_v1(0.40, 0.59, TrustTierBandV1::Guardian),
            Err(TrustScoreAssertionError::ClaimedTierDoesNotMatchMidpoint)
        );
    }

    #[test]
    fn underclaimed_basic_tier_is_rejected() {
        assert_eq!(
            validate_score_assertion_v1(0.80, 0.90, TrustTierBandV1::Basic),
            Err(TrustScoreAssertionError::ClaimedTierDoesNotMatchMidpoint)
        );
    }

    #[test]
    fn exact_single_point_thresholds_match_frozen_v1_derivation() {
        let cases = [
            (0.0_f32, TrustTierBandV1::Observer),
            (0.3_f32, TrustTierBandV1::Basic),
            (0.4_f32, TrustTierBandV1::Standard),
            (0.6_f32, TrustTierBandV1::Elevated),
            (0.8_f32, TrustTierBandV1::Guardian),
        ];

        for (score, tier) in cases {
            assert_eq!(validate_score_range_v1(score, score), Ok(tier));
        }
    }

    #[test]
    fn tier_derivation_is_monotone_across_representative_scores() {
        let scores = [0.0_f64, 0.29, 0.3, 0.39, 0.4, 0.59, 0.6, 0.79, 0.8, 1.0];
        let mut previous = TrustTierBandV1::Observer;
        for score in scores {
            let current = TrustTierBandV1::from_score(score);
            assert!(current >= previous);
            previous = current;
        }
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical fixed-point trust-authority theorem for Identity V2.
//!
//! This crate owns only arithmetic and conservative authority projection.
//! It deliberately does not own Holochain state, proof verification, commitments,
//! credential issuance, serialization, network completeness, or verifier policy.
//!
//! The canonical proof model is the 8-dimensional K-vector used by
//! `mycelix-core-types`, with exact integer weights summing to 100. A future ZK
//! backend may keep the components private, but it must prove the same integer
//! arithmetic defined here.

#![forbid(unsafe_code)]

pub const K_VECTOR_DIMENSIONS_V2: usize = 8;
pub const TRUST_SCORE_SCALE_V2: u64 = 1_000_000;
pub const K_VECTOR_WEIGHT_SUM_V2: u64 = 100;

/// Canonical 8-D Identity K-vector weights, expressed as exact percentages.
///
/// Order:
/// `k_r, k_a, k_i, k_p, k_m, k_s, k_h, k_topo`.
pub const K_VECTOR_WEIGHTS_PERCENT_V2: [u64; K_VECTOR_DIMENSIONS_V2] =
    [25, 15, 20, 15, 5, 10, 5, 5];

pub const BASIC_MIN_SCORE_SCALED_V2: u64 = 300_000;
pub const STANDARD_MIN_SCORE_SCALED_V2: u64 = 400_000;
pub const ELEVATED_MIN_SCORE_SCALED_V2: u64 = 600_000;
pub const GUARDIAN_MIN_SCORE_SCALED_V2: u64 = 800_000;

/// The strongest trust tier guaranteed by a proven lower score bound.
///
/// This is intentionally named `Guaranteed`, not `Structural`: legacy V1
/// credentials may retain a midpoint-derived structural/display tier for
/// compatibility, but V2 authority must never be stronger than the minimum score
/// established by proof evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum GuaranteedTrustTierV2 {
    Observer,
    Basic,
    Standard,
    Elevated,
    Guardian,
}

impl GuaranteedTrustTierV2 {
    pub const fn minimum_score_scaled(self) -> u64 {
        match self {
            Self::Observer => 0,
            Self::Basic => BASIC_MIN_SCORE_SCALED_V2,
            Self::Standard => STANDARD_MIN_SCORE_SCALED_V2,
            Self::Elevated => ELEVATED_MIN_SCORE_SCALED_V2,
            Self::Guardian => GUARDIAN_MIN_SCORE_SCALED_V2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustAuthorityPolicyErrorV2 {
    ComponentOutOfRange { index: usize, value: u64 },
    LowerBoundOutOfRange { value: u64 },
    UpperBoundOutOfRange { value: u64 },
    ReversedRange { lower: u64, upper: u64 },
    WitnessOutsidePublicRange,
}

/// Validate a public fixed-point score range.
pub fn validate_score_range_v2(
    lower_scaled: u64,
    upper_scaled: u64,
) -> Result<(), TrustAuthorityPolicyErrorV2> {
    if lower_scaled > TRUST_SCORE_SCALE_V2 {
        return Err(TrustAuthorityPolicyErrorV2::LowerBoundOutOfRange {
            value: lower_scaled,
        });
    }
    if upper_scaled > TRUST_SCORE_SCALE_V2 {
        return Err(TrustAuthorityPolicyErrorV2::UpperBoundOutOfRange {
            value: upper_scaled,
        });
    }
    if lower_scaled > upper_scaled {
        return Err(TrustAuthorityPolicyErrorV2::ReversedRange {
            lower: lower_scaled,
            upper: upper_scaled,
        });
    }
    Ok(())
}

/// Compute the exact weighted numerator for the canonical 8-D K-vector.
///
/// Components are fixed-point integers in `[0, TRUST_SCORE_SCALE_V2]`.
/// The returned numerator is:
///
/// `N = Σ(weight_percent[i] * component[i])`
///
/// and therefore lies in `[0, 100 * TRUST_SCORE_SCALE_V2]`.
///
/// A proof backend should constrain this arithmetic over private witness values;
/// this pure function is the protocol oracle used by tests/adapters, not proof
/// verification by itself.
pub fn weighted_trust_numerator_v2(
    components_scaled: [u64; K_VECTOR_DIMENSIONS_V2],
) -> Result<u64, TrustAuthorityPolicyErrorV2> {
    let mut numerator: u128 = 0;

    for (index, (component, weight)) in components_scaled
        .into_iter()
        .zip(K_VECTOR_WEIGHTS_PERCENT_V2)
        .enumerate()
    {
        if component > TRUST_SCORE_SCALE_V2 {
            return Err(TrustAuthorityPolicyErrorV2::ComponentOutOfRange {
                index,
                value: component,
            });
        }
        numerator += (component as u128) * (weight as u128);
    }

    // The validated maximum is exactly 100 * 1_000_000 and cannot overflow u64.
    Ok(numerator as u64)
}

/// Validate that one private K-vector witness satisfies a public score interval.
///
/// Division is deliberately avoided. If `N` is the weighted numerator, then the
/// exact range theorem is:
///
/// `100 * lower <= N <= 100 * upper`.
///
/// This makes the intended ZK arithmetic deterministic and integer-only.
pub fn validate_kvector_witness_score_range_v2(
    components_scaled: [u64; K_VECTOR_DIMENSIONS_V2],
    lower_scaled: u64,
    upper_scaled: u64,
) -> Result<(), TrustAuthorityPolicyErrorV2> {
    validate_score_range_v2(lower_scaled, upper_scaled)?;
    let numerator = weighted_trust_numerator_v2(components_scaled)? as u128;
    let lower_numerator = (lower_scaled as u128) * (K_VECTOR_WEIGHT_SUM_V2 as u128);
    let upper_numerator = (upper_scaled as u128) * (K_VECTOR_WEIGHT_SUM_V2 as u128);

    if numerator < lower_numerator || numerator > upper_numerator {
        return Err(TrustAuthorityPolicyErrorV2::WitnessOutsidePublicRange);
    }

    Ok(())
}

/// Derive the strongest authority tier guaranteed by a proven score range.
///
/// Only the lower bound participates. The midpoint and upper bound are explicitly
/// not authority inputs.
pub fn guaranteed_trust_tier_v2(
    lower_scaled: u64,
    upper_scaled: u64,
) -> Result<GuaranteedTrustTierV2, TrustAuthorityPolicyErrorV2> {
    validate_score_range_v2(lower_scaled, upper_scaled)?;

    let tier = if lower_scaled >= GUARDIAN_MIN_SCORE_SCALED_V2 {
        GuaranteedTrustTierV2::Guardian
    } else if lower_scaled >= ELEVATED_MIN_SCORE_SCALED_V2 {
        GuaranteedTrustTierV2::Elevated
    } else if lower_scaled >= STANDARD_MIN_SCORE_SCALED_V2 {
        GuaranteedTrustTierV2::Standard
    } else if lower_scaled >= BASIC_MIN_SCORE_SCALED_V2 {
        GuaranteedTrustTierV2::Basic
    } else {
        GuaranteedTrustTierV2::Observer
    };

    Ok(tier)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_weights_are_frozen_and_sum_to_one_hundred() {
        assert_eq!(
            K_VECTOR_WEIGHTS_PERCENT_V2,
            [25, 15, 20, 15, 5, 10, 5, 5]
        );
        assert_eq!(
            K_VECTOR_WEIGHTS_PERCENT_V2.iter().sum::<u64>(),
            K_VECTOR_WEIGHT_SUM_V2
        );
    }

    #[test]
    fn all_zero_and_all_one_vectors_hit_exact_extremes() {
        assert_eq!(weighted_trust_numerator_v2([0; 8]), Ok(0));
        assert_eq!(
            weighted_trust_numerator_v2([TRUST_SCORE_SCALE_V2; 8]),
            Ok(K_VECTOR_WEIGHT_SUM_V2 * TRUST_SCORE_SCALE_V2)
        );
    }

    #[test]
    fn known_vector_has_exact_integer_score() {
        // Weighted score = 0.25*0.8 + 0.15*0.6 + 0.20*0.9 + 0.15*0.7
        //                + 0.05*0.5 + 0.10*0.4 + 0.05*0.6 + 0.05*0.5
        //                = 0.695
        let components = [800_000, 600_000, 900_000, 700_000, 500_000, 400_000, 600_000, 500_000];
        assert_eq!(weighted_trust_numerator_v2(components), Ok(69_500_000));
        assert_eq!(
            validate_kvector_witness_score_range_v2(components, 695_000, 695_000),
            Ok(())
        );
    }

    #[test]
    fn public_range_uses_exact_integer_inequality() {
        let components = [500_000; 8];
        assert_eq!(weighted_trust_numerator_v2(components), Ok(50_000_000));
        assert_eq!(
            validate_kvector_witness_score_range_v2(components, 400_000, 600_000),
            Ok(())
        );
        assert_eq!(
            validate_kvector_witness_score_range_v2(components, 500_001, 600_000),
            Err(TrustAuthorityPolicyErrorV2::WitnessOutsidePublicRange)
        );
    }

    #[test]
    fn out_of_range_component_fails_closed() {
        let mut components = [500_000; 8];
        components[3] = TRUST_SCORE_SCALE_V2 + 1;
        assert_eq!(
            weighted_trust_numerator_v2(components),
            Err(TrustAuthorityPolicyErrorV2::ComponentOutOfRange {
                index: 3,
                value: TRUST_SCORE_SCALE_V2 + 1,
            })
        );
    }

    #[test]
    fn malformed_public_ranges_fail_closed() {
        assert_eq!(
            validate_score_range_v2(TRUST_SCORE_SCALE_V2 + 1, TRUST_SCORE_SCALE_V2 + 1),
            Err(TrustAuthorityPolicyErrorV2::LowerBoundOutOfRange {
                value: TRUST_SCORE_SCALE_V2 + 1,
            })
        );
        assert_eq!(
            validate_score_range_v2(0, TRUST_SCORE_SCALE_V2 + 1),
            Err(TrustAuthorityPolicyErrorV2::UpperBoundOutOfRange {
                value: TRUST_SCORE_SCALE_V2 + 1,
            })
        );
        assert_eq!(
            validate_score_range_v2(600_000, 400_000),
            Err(TrustAuthorityPolicyErrorV2::ReversedRange {
                lower: 600_000,
                upper: 400_000,
            })
        );
    }

    #[test]
    fn exact_tier_thresholds_are_lower_bound_conservative() {
        let cases = [
            (0, GuaranteedTrustTierV2::Observer),
            (299_999, GuaranteedTrustTierV2::Observer),
            (300_000, GuaranteedTrustTierV2::Basic),
            (399_999, GuaranteedTrustTierV2::Basic),
            (400_000, GuaranteedTrustTierV2::Standard),
            (599_999, GuaranteedTrustTierV2::Standard),
            (600_000, GuaranteedTrustTierV2::Elevated),
            (799_999, GuaranteedTrustTierV2::Elevated),
            (800_000, GuaranteedTrustTierV2::Guardian),
            (1_000_000, GuaranteedTrustTierV2::Guardian),
        ];

        for (lower, expected) in cases {
            assert_eq!(guaranteed_trust_tier_v2(lower, TRUST_SCORE_SCALE_V2), Ok(expected));
            assert_eq!(expected.minimum_score_scaled() <= lower, true);
        }
    }

    #[test]
    fn midpoint_and_upper_bound_cannot_strengthen_authority() {
        // Midpoint 0.60 would be Elevated and upper 0.80 reaches Guardian, but the
        // proof guarantees only that the hidden score is at least 0.40.
        assert_eq!(
            guaranteed_trust_tier_v2(400_000, 800_000),
            Ok(GuaranteedTrustTierV2::Standard)
        );

        // The widest possible range has midpoint 0.50 and upper bound 1.00, but
        // guarantees no score above zero.
        assert_eq!(
            guaranteed_trust_tier_v2(0, 1_000_000),
            Ok(GuaranteedTrustTierV2::Observer)
        );
    }

    #[test]
    fn narrowing_upper_bound_does_not_change_guaranteed_tier() {
        assert_eq!(
            guaranteed_trust_tier_v2(600_000, 600_000),
            guaranteed_trust_tier_v2(600_000, 1_000_000)
        );
    }
}

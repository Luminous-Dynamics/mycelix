#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Fail-closed collateral-health assessment for Mycelix Finance.
//!
//! This crate deliberately separates two questions:
//! 1. what collateral-health tier a valid finite LTV observation implies; and
//! 2. whether the observation is valid enough to support any tier conclusion.
//!
//! Invalid or unavailable valuation is `Indeterminate`. It is neither
//! `Healthy` nor evidence of `Liquidation`.

use mycelix_finance_types::{
    CollateralHealthStatus, LTV_LIQUIDATION_THRESHOLD, LTV_MARGIN_CALL_THRESHOLD,
    LTV_WARNING_THRESHOLD,
};
use serde::{Deserialize, Serialize};

/// Why a collateral-health observation could not be classified.
///
/// These reasons are intentionally stable and serializable so coordinator,
/// wire, UI, and evidence-history layers can preserve uncertainty instead of
/// coercing it into a threshold-derived status.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralHealthIndeterminateReason {
    /// No current valuation observation was available.
    MissingValuation,
    /// The collateral valuation denominator was zero.
    ZeroValuation,
    /// A supplied or derived LTV ratio was NaN or infinite.
    NonFiniteRatio,
    /// A supplied LTV ratio was negative and therefore outside the model.
    NegativeRatio,
}

/// Checked collateral-health evidence.
///
/// `Known` means the underlying observation was valid and finite. `Indeterminate`
/// means the system does not currently have enough valid evidence to infer a
/// threshold tier.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralHealthAssessment {
    Known(CollateralHealthStatus),
    Indeterminate(CollateralHealthIndeterminateReason),
}

impl CollateralHealthAssessment {
    /// Return the known threshold status, if current evidence supports one.
    pub fn known_status(&self) -> Option<&CollateralHealthStatus> {
        match self {
            Self::Known(status) => Some(status),
            Self::Indeterminate(_) => None,
        }
    }

    /// Return the uncertainty reason, if this observation is indeterminate.
    pub fn indeterminate_reason(&self) -> Option<CollateralHealthIndeterminateReason> {
        match self {
            Self::Known(_) => None,
            Self::Indeterminate(reason) => Some(*reason),
        }
    }

    /// Whether this observation is valid threshold evidence for liquidation.
    ///
    /// This is intentionally an evidence predicate, not an authorization
    /// predicate. Callers still need whatever governance/claim authority their
    /// operation requires.
    pub fn is_liquidation_evidence(&self) -> bool {
        matches!(self, Self::Known(CollateralHealthStatus::Liquidation))
    }

    /// Stable presentation label for existing string-based surfaces.
    ///
    /// New wire formats should preserve `indeterminate_reason()` separately.
    pub fn status_label(&self) -> &'static str {
        match self {
            Self::Known(CollateralHealthStatus::Healthy) => "Healthy",
            Self::Known(CollateralHealthStatus::Warning) => "Warning",
            Self::Known(CollateralHealthStatus::MarginCall) => "MarginCall",
            Self::Known(CollateralHealthStatus::Liquidation) => "Liquidation",
            Self::Indeterminate(_) => "Indeterminate",
        }
    }
}

/// Classify a pre-computed LTV ratio using the canonical Finance thresholds.
///
/// The existing strict-boundary policy is preserved deliberately:
///
/// - `ltv <= 0.80` => Healthy
/// - `0.80 < ltv <= 0.90` => Warning
/// - `0.90 < ltv <= 0.95` => MarginCall
/// - `ltv > 0.95` => Liquidation
///
/// Non-finite and negative ratios never produce a threshold status.
pub fn assess_ltv_ratio(ltv: f64) -> CollateralHealthAssessment {
    if !ltv.is_finite() {
        return CollateralHealthAssessment::Indeterminate(
            CollateralHealthIndeterminateReason::NonFiniteRatio,
        );
    }

    if ltv < 0.0 {
        return CollateralHealthAssessment::Indeterminate(
            CollateralHealthIndeterminateReason::NegativeRatio,
        );
    }

    let status = if ltv > LTV_LIQUIDATION_THRESHOLD {
        CollateralHealthStatus::Liquidation
    } else if ltv > LTV_MARGIN_CALL_THRESHOLD {
        CollateralHealthStatus::MarginCall
    } else if ltv > LTV_WARNING_THRESHOLD {
        CollateralHealthStatus::Warning
    } else {
        CollateralHealthStatus::Healthy
    };

    CollateralHealthAssessment::Known(status)
}

/// Compute and classify LTV from an obligation amount and optional current
/// collateral valuation.
///
/// Missing and zero valuation are detected *before* division. In particular,
/// zero valuation must not be converted to `+Inf` and then mistaken for
/// liquidation evidence.
pub fn assess_collateral_health(
    obligation_amount: u64,
    current_value: Option<u64>,
) -> CollateralHealthAssessment {
    let current_value = match current_value {
        None => {
            return CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::MissingValuation,
            );
        }
        Some(0) => {
            return CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::ZeroValuation,
            );
        }
        Some(value) => value,
    };

    assess_ltv_ratio(obligation_amount as f64 / current_value as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_threshold_boundaries_preserve_existing_policy() {
        assert_eq!(
            assess_ltv_ratio(0.80),
            CollateralHealthAssessment::Known(CollateralHealthStatus::Healthy)
        );
        assert_eq!(
            assess_ltv_ratio(0.800_001),
            CollateralHealthAssessment::Known(CollateralHealthStatus::Warning)
        );
        assert_eq!(
            assess_ltv_ratio(0.90),
            CollateralHealthAssessment::Known(CollateralHealthStatus::Warning)
        );
        assert_eq!(
            assess_ltv_ratio(0.900_001),
            CollateralHealthAssessment::Known(CollateralHealthStatus::MarginCall)
        );
        assert_eq!(
            assess_ltv_ratio(0.95),
            CollateralHealthAssessment::Known(CollateralHealthStatus::MarginCall)
        );
        assert_eq!(
            assess_ltv_ratio(0.950_001),
            CollateralHealthAssessment::Known(CollateralHealthStatus::Liquidation)
        );
    }

    #[test]
    fn non_finite_ratios_are_indeterminate_not_threshold_states() {
        for ratio in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                assess_ltv_ratio(ratio),
                CollateralHealthAssessment::Indeterminate(
                    CollateralHealthIndeterminateReason::NonFiniteRatio
                )
            );
        }
    }

    #[test]
    fn negative_ratio_is_indeterminate() {
        assert_eq!(
            assess_ltv_ratio(-0.000_001),
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::NegativeRatio
            )
        );
    }

    #[test]
    fn missing_and_zero_valuation_are_distinct_indeterminate_reasons() {
        assert_eq!(
            assess_collateral_health(100, None),
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::MissingValuation
            )
        );
        assert_eq!(
            assess_collateral_health(100, Some(0)),
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::ZeroValuation
            )
        );
    }

    #[test]
    fn zero_obligation_with_valid_valuation_is_healthy() {
        assert_eq!(
            assess_collateral_health(0, Some(100)),
            CollateralHealthAssessment::Known(CollateralHealthStatus::Healthy)
        );
    }

    #[test]
    fn computed_ratio_uses_same_classifier() {
        assert_eq!(
            assess_collateral_health(81, Some(100)),
            assess_ltv_ratio(0.81)
        );
        assert_eq!(
            assess_collateral_health(96, Some(100)),
            assess_ltv_ratio(0.96)
        );
    }

    #[test]
    fn indeterminate_is_never_liquidation_evidence() {
        for assessment in [
            assess_ltv_ratio(f64::NAN),
            assess_ltv_ratio(f64::INFINITY),
            assess_collateral_health(100, None),
            assess_collateral_health(100, Some(0)),
        ] {
            assert!(!assessment.is_liquidation_evidence());
            assert_eq!(assessment.status_label(), "Indeterminate");
        }

        assert!(assess_ltv_ratio(0.96).is_liquidation_evidence());
    }

    #[test]
    fn recovery_requires_a_new_valid_observation() {
        let first = assess_collateral_health(90, None);
        assert_eq!(
            first,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::MissingValuation
            )
        );

        let second = assess_collateral_health(90, Some(100));
        assert_eq!(
            second,
            CollateralHealthAssessment::Known(CollateralHealthStatus::Warning)
        );

        // The old value remains an independently representable evidence event;
        // recovery does not mutate or reinterpret it.
        assert!(matches!(
            first,
            CollateralHealthAssessment::Indeterminate(_)
        ));
    }

    #[test]
    fn serialized_indeterminate_preserves_reason() {
        let assessment = CollateralHealthAssessment::Indeterminate(
            CollateralHealthIndeterminateReason::ZeroValuation,
        );
        let encoded = serde_json::to_string(&assessment).expect("serialize assessment");
        let decoded: CollateralHealthAssessment =
            serde_json::from_str(&encoded).expect("deserialize assessment");
        assert_eq!(decoded, assessment);
    }
}

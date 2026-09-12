#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Fail-closed collateral-health assessment for Mycelix Finance.
//!
//! This crate deliberately separates three questions:
//! 1. whether a valuation observation is currently available and trustworthy;
//! 2. what LTV ratio follows from valid numeric evidence; and
//! 3. what collateral-health tier that valid finite LTV observation implies.
//!
//! Invalid or unavailable valuation is `Indeterminate`. It is neither
//! `Healthy` nor evidence of `Liquidation`.

use mycelix_finance_types::{
    CollateralHealthStatus, LTV_LIQUIDATION_THRESHOLD, LTV_MARGIN_CALL_THRESHOLD,
    LTV_WARNING_THRESHOLD,
};
use serde::{Deserialize, Serialize};

/// Current checked wire/evidence snapshot schema version.
pub const CHECKED_COLLATERAL_HEALTH_SNAPSHOT_VERSION: u16 = 1;

/// Why a valuation observation is unavailable before any LTV arithmetic.
///
/// Keeping acquisition failure typed prevents callers from using a numeric
/// sentinel such as `0`, `999.0`, NaN, or infinity to represent missing data.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralValuationUnavailableReason {
    /// No current valuation observation exists.
    Missing,
    /// The configured valuation/oracle service could not be reached.
    OracleUnavailable,
    /// A response was received but could not be accepted as a valid observation.
    InvalidObservation,
}

/// A valuation acquisition result before collateral-health computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralValuationObservation {
    /// A numeric value was actually observed.
    Observed { value: u64 },
    /// No usable numeric observation is currently available.
    Unavailable(CollateralValuationUnavailableReason),
}

/// Why a collateral-health observation could not be classified.
///
/// These reasons are intentionally stable and serializable so coordinator,
/// wire, UI, and evidence-history layers can preserve uncertainty instead of
/// coercing it into a threshold-derived status.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralHealthIndeterminateReason {
    /// No current valuation observation was available.
    MissingValuation,
    /// The valuation/oracle service was unavailable.
    OracleUnavailable,
    /// The valuation source returned an unusable observation.
    InvalidValuationObservation,
    /// The collateral valuation denominator was observed as zero.
    ZeroValuation,
    /// A supplied or derived LTV ratio was NaN or infinite.
    NonFiniteRatio,
    /// A supplied LTV ratio was negative and therefore outside the model.
    NegativeRatio,
}

impl From<CollateralValuationUnavailableReason> for CollateralHealthIndeterminateReason {
    fn from(reason: CollateralValuationUnavailableReason) -> Self {
        match reason {
            CollateralValuationUnavailableReason::Missing => Self::MissingValuation,
            CollateralValuationUnavailableReason::OracleUnavailable => Self::OracleUnavailable,
            CollateralValuationUnavailableReason::InvalidObservation => {
                Self::InvalidValuationObservation
            }
        }
    }
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

/// Why a deserialized or externally supplied checked snapshot is invalid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralHealthSnapshotValidationError {
    UnsupportedSchemaVersion,
    LtvRatioMismatch,
    AssessmentMismatch,
}

/// A storage-independent checked health snapshot suitable for bridge/wire use.
///
/// This deliberately does not expose a Holochain `Record` or integrity-entry
/// layout. Storage can evolve independently while external callers receive the
/// same evidence semantics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CheckedCollateralHealthSnapshot {
    pub schema_version: u16,
    pub collateral_id: String,
    pub obligation_amount: u64,
    pub valuation: CollateralValuationObservation,
    /// Present only when a valid positive denominator produced a numeric ratio.
    pub ltv_ratio: Option<f64>,
    pub assessment: CollateralHealthAssessment,
    pub computed_at_micros: i64,
}

impl CheckedCollateralHealthSnapshot {
    /// Construct a self-consistent snapshot from source evidence.
    pub fn new(
        collateral_id: String,
        obligation_amount: u64,
        valuation: CollateralValuationObservation,
        computed_at_micros: i64,
    ) -> Self {
        let ltv_ratio = ltv_ratio_from_observation(obligation_amount, &valuation);
        let assessment = assess_collateral_observation(obligation_amount, valuation);
        Self {
            schema_version: CHECKED_COLLATERAL_HEALTH_SNAPSHOT_VERSION,
            collateral_id,
            obligation_amount,
            valuation,
            ltv_ratio,
            assessment,
            computed_at_micros,
        }
    }

    /// Recompute all derived fields and reject unsupported or contradictory data.
    ///
    /// Call this on deserialized/untrusted snapshots before treating them as
    /// collateral-health evidence.
    pub fn validate(&self) -> Result<(), CollateralHealthSnapshotValidationError> {
        if self.schema_version != CHECKED_COLLATERAL_HEALTH_SNAPSHOT_VERSION {
            return Err(CollateralHealthSnapshotValidationError::UnsupportedSchemaVersion);
        }

        if self.ltv_ratio != ltv_ratio_from_observation(self.obligation_amount, &self.valuation) {
            return Err(CollateralHealthSnapshotValidationError::LtvRatioMismatch);
        }

        if self.assessment != assess_collateral_observation(self.obligation_amount, self.valuation)
        {
            return Err(CollateralHealthSnapshotValidationError::AssessmentMismatch);
        }

        Ok(())
    }

    /// Convenience predicate for callers that only need a boolean check.
    pub fn is_consistent(&self) -> bool {
        self.validate().is_ok()
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

/// Compute the numeric LTV only when the valuation observation supplies a
/// positive denominator.
pub fn ltv_ratio_from_observation(
    obligation_amount: u64,
    observation: &CollateralValuationObservation,
) -> Option<f64> {
    match observation {
        CollateralValuationObservation::Observed { value } if *value > 0 => {
            Some(obligation_amount as f64 / *value as f64)
        }
        CollateralValuationObservation::Observed { value: 0 }
        | CollateralValuationObservation::Unavailable(_) => None,
        CollateralValuationObservation::Observed { .. } => {
            unreachable!("u64 value cases exhausted")
        }
    }
}

/// Classify collateral health from a typed valuation observation.
///
/// Unavailable observations remain unavailable: no numeric sentinel is
/// manufactured. An observed zero value is also handled before division so it
/// cannot accidentally become infinity and then masquerade as threshold
/// evidence.
pub fn assess_collateral_observation(
    obligation_amount: u64,
    observation: CollateralValuationObservation,
) -> CollateralHealthAssessment {
    let current_value = match observation {
        CollateralValuationObservation::Unavailable(reason) => {
            return CollateralHealthAssessment::Indeterminate(reason.into());
        }
        CollateralValuationObservation::Observed { value: 0 } => {
            return CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::ZeroValuation,
            );
        }
        CollateralValuationObservation::Observed { value } => value,
    };

    assess_ltv_ratio(obligation_amount as f64 / current_value as f64)
}

/// Compatibility helper for callers that only have an optional numeric value.
///
/// New oracle-facing code should prefer [`assess_collateral_observation`] so it
/// can distinguish a missing observation from a specific acquisition failure.
pub fn assess_collateral_health(
    obligation_amount: u64,
    current_value: Option<u64>,
) -> CollateralHealthAssessment {
    let observation = match current_value {
        Some(value) => CollateralValuationObservation::Observed { value },
        None => CollateralValuationObservation::Unavailable(
            CollateralValuationUnavailableReason::Missing,
        ),
    };
    assess_collateral_observation(obligation_amount, observation)
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
    fn acquisition_failure_is_typed_before_ratio_computation() {
        assert_eq!(
            assess_collateral_observation(
                100,
                CollateralValuationObservation::Unavailable(
                    CollateralValuationUnavailableReason::OracleUnavailable,
                ),
            ),
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::OracleUnavailable
            )
        );
        assert_eq!(
            assess_collateral_observation(
                100,
                CollateralValuationObservation::Unavailable(
                    CollateralValuationUnavailableReason::InvalidObservation,
                ),
            ),
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::InvalidValuationObservation
            )
        );
    }

    #[test]
    fn oracle_unavailable_and_observed_zero_are_not_the_same_evidence() {
        let unavailable = assess_collateral_observation(
            100,
            CollateralValuationObservation::Unavailable(
                CollateralValuationUnavailableReason::OracleUnavailable,
            ),
        );
        let observed_zero = assess_collateral_observation(
            100,
            CollateralValuationObservation::Observed { value: 0 },
        );

        assert_eq!(
            unavailable,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::OracleUnavailable
            )
        );
        assert_eq!(
            observed_zero,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::ZeroValuation
            )
        );
        assert_ne!(unavailable, observed_zero);
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
    fn snapshot_keeps_unavailable_ratio_absent() {
        let snapshot = CheckedCollateralHealthSnapshot::new(
            "collateral:1".into(),
            100,
            CollateralValuationObservation::Unavailable(
                CollateralValuationUnavailableReason::OracleUnavailable,
            ),
            42,
        );
        assert_eq!(
            snapshot.schema_version,
            CHECKED_COLLATERAL_HEALTH_SNAPSHOT_VERSION
        );
        assert_eq!(snapshot.ltv_ratio, None);
        assert_eq!(
            snapshot.assessment,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::OracleUnavailable
            )
        );
        assert_eq!(snapshot.validate(), Ok(()));
    }

    #[test]
    fn snapshot_keeps_observed_zero_distinct_without_ratio_sentinel() {
        let snapshot = CheckedCollateralHealthSnapshot::new(
            "collateral:zero".into(),
            100,
            CollateralValuationObservation::Observed { value: 0 },
            43,
        );
        assert_eq!(snapshot.ltv_ratio, None);
        assert_eq!(
            snapshot.assessment,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::ZeroValuation
            )
        );
        assert_eq!(snapshot.validate(), Ok(()));
    }

    #[test]
    fn snapshot_computes_ratio_from_valid_observation() {
        let snapshot = CheckedCollateralHealthSnapshot::new(
            "collateral:known".into(),
            91,
            CollateralValuationObservation::Observed { value: 100 },
            44,
        );
        let ratio = snapshot
            .ltv_ratio
            .expect("valid observation should have ratio");
        assert!((ratio - 0.91).abs() < f64::EPSILON);
        assert_eq!(
            snapshot.assessment,
            CollateralHealthAssessment::Known(CollateralHealthStatus::MarginCall)
        );
        assert_eq!(snapshot.validate(), Ok(()));
    }

    #[test]
    fn snapshot_validation_detects_tampered_assessment() {
        let mut snapshot = CheckedCollateralHealthSnapshot::new(
            "collateral:tampered-assessment".into(),
            50,
            CollateralValuationObservation::Observed { value: 100 },
            45,
        );
        snapshot.assessment =
            CollateralHealthAssessment::Known(CollateralHealthStatus::Liquidation);
        assert_eq!(
            snapshot.validate(),
            Err(CollateralHealthSnapshotValidationError::AssessmentMismatch)
        );
    }

    #[test]
    fn snapshot_validation_detects_tampered_ratio() {
        let mut snapshot = CheckedCollateralHealthSnapshot::new(
            "collateral:tampered-ratio".into(),
            50,
            CollateralValuationObservation::Observed { value: 100 },
            46,
        );
        snapshot.ltv_ratio = Some(0.96);
        assert_eq!(
            snapshot.validate(),
            Err(CollateralHealthSnapshotValidationError::LtvRatioMismatch)
        );
    }

    #[test]
    fn snapshot_validation_rejects_unknown_schema_version() {
        let mut snapshot = CheckedCollateralHealthSnapshot::new(
            "collateral:future".into(),
            50,
            CollateralValuationObservation::Observed { value: 100 },
            47,
        );
        snapshot.schema_version = CHECKED_COLLATERAL_HEALTH_SNAPSHOT_VERSION + 1;
        assert_eq!(
            snapshot.validate(),
            Err(CollateralHealthSnapshotValidationError::UnsupportedSchemaVersion)
        );
    }

    #[test]
    fn indeterminate_is_never_liquidation_evidence() {
        for assessment in [
            assess_ltv_ratio(f64::NAN),
            assess_ltv_ratio(f64::INFINITY),
            assess_collateral_health(100, None),
            assess_collateral_health(100, Some(0)),
            assess_collateral_observation(
                100,
                CollateralValuationObservation::Unavailable(
                    CollateralValuationUnavailableReason::OracleUnavailable,
                ),
            ),
        ] {
            assert!(!assessment.is_liquidation_evidence());
            assert_eq!(assessment.status_label(), "Indeterminate");
        }

        assert!(assess_ltv_ratio(0.96).is_liquidation_evidence());
    }

    #[test]
    fn recovery_requires_a_new_valid_observation() {
        let first = assess_collateral_observation(
            90,
            CollateralValuationObservation::Unavailable(
                CollateralValuationUnavailableReason::OracleUnavailable,
            ),
        );
        assert_eq!(
            first,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::OracleUnavailable
            )
        );

        let second = assess_collateral_observation(
            90,
            CollateralValuationObservation::Observed { value: 100 },
        );
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
            CollateralHealthIndeterminateReason::OracleUnavailable,
        );
        let encoded = serde_json::to_string(&assessment).expect("serialize assessment");
        let decoded: CollateralHealthAssessment =
            serde_json::from_str(&encoded).expect("deserialize assessment");
        assert_eq!(decoded, assessment);
    }

    #[test]
    fn serialized_valuation_observation_preserves_availability_reason() {
        let observation = CollateralValuationObservation::Unavailable(
            CollateralValuationUnavailableReason::InvalidObservation,
        );
        let encoded = serde_json::to_string(&observation).expect("serialize observation");
        let decoded: CollateralValuationObservation =
            serde_json::from_str(&encoded).expect("deserialize observation");
        assert_eq!(decoded, observation);
    }

    #[test]
    fn serialized_snapshot_preserves_checked_evidence() {
        let snapshot = CheckedCollateralHealthSnapshot::new(
            "collateral:wire".into(),
            100,
            CollateralValuationObservation::Unavailable(
                CollateralValuationUnavailableReason::OracleUnavailable,
            ),
            48,
        );
        let encoded = serde_json::to_string(&snapshot).expect("serialize snapshot");
        let decoded: CheckedCollateralHealthSnapshot =
            serde_json::from_str(&encoded).expect("deserialize snapshot");
        assert_eq!(decoded, snapshot);
        assert_eq!(decoded.validate(), Ok(()));
    }
}

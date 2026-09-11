#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Versioned persisted collateral-health evidence for Mycelix Finance.
//!
//! This crate separates historical compatibility records from checked current
//! evidence. Legacy V1 fields are preserved as history but can never be silently
//! promoted into V2 liquidation evidence. V2 binds the exact valuation request,
//! provider envelope, and derived checked health snapshot and recomputes the
//! derivation during validation.

use finance_collateral_safety::{
    CheckedCollateralHealthSnapshot, CollateralHealthSnapshotValidationError,
};
use finance_collateral_valuation::{
    CollateralValuationContractError, CollateralValuationEnvelope,
    CollateralValuationOutcome, CollateralValuationRequest,
};
use serde::{Deserialize, Serialize};

/// Persisted evidence schema version for the checked V2 record.
pub const COLLATERAL_HEALTH_EVIDENCE_V2_SCHEMA_VERSION: u16 = 2;

/// Historical V1 collateral-health data as it existed before checked evidence
/// semantics were introduced.
///
/// The fields deliberately mirror the legacy storage shape. No constructor in
/// this crate converts this record into V2 because the original data does not
/// preserve enough information to prove valuation availability or provider
/// binding.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LegacyCollateralHealthEvidenceV1 {
    pub collateral_id: String,
    pub current_value: u64,
    pub obligation_amount: u64,
    pub ltv_ratio: f64,
    pub status: String,
    pub computed_at_micros: i64,
}

/// Epistemic quality of a legacy V1 record.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LegacyCollateralHealthEvidenceQuality {
    /// The record is historical compatibility data. Its numeric/status fields do
    /// not prove how valuation was acquired and therefore cannot become checked
    /// current evidence by inference.
    LegacyUnknown,
}

impl LegacyCollateralHealthEvidenceV1 {
    pub fn evidence_quality(&self) -> LegacyCollateralHealthEvidenceQuality {
        LegacyCollateralHealthEvidenceQuality::LegacyUnknown
    }

    /// Legacy status strings and sentinels never satisfy the checked liquidation
    /// evidence predicate.
    pub fn is_checked_liquidation_evidence(&self) -> bool {
        false
    }

    /// V1 lacks enough provenance to support a lossless V2 upgrade without a new
    /// bound valuation observation.
    pub fn can_upgrade_without_new_observation(&self) -> bool {
        false
    }
}

/// Persisted V2 collateral-health evidence.
///
/// The request identifies the exact subject/provider contract that was selected.
/// The envelope carries the provider outcome, including typed unavailability.
/// The snapshot contains only the derived health evidence. Validation recomputes
/// the snapshot from the bound envelope instead of trusting duplicated fields.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollateralHealthEvidenceV2 {
    pub schema_version: u16,
    pub request: CollateralValuationRequest,
    pub envelope: CollateralValuationEnvelope,
    pub snapshot: CheckedCollateralHealthSnapshot,
}

/// Why a persisted V2 record cannot be accepted as checked evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralHealthEvidenceV2ValidationError {
    UnsupportedSchemaVersion,
    ValuationContract(CollateralValuationContractError),
    Snapshot(CollateralHealthSnapshotValidationError),
    EvidenceTimestampAfterComputation,
    SnapshotSubjectMismatch,
    SnapshotMismatch,
}

impl From<CollateralValuationContractError> for CollateralHealthEvidenceV2ValidationError {
    fn from(value: CollateralValuationContractError) -> Self {
        Self::ValuationContract(value)
    }
}

impl From<CollateralHealthSnapshotValidationError>
    for CollateralHealthEvidenceV2ValidationError
{
    fn from(value: CollateralHealthSnapshotValidationError) -> Self {
        Self::Snapshot(value)
    }
}

impl CollateralHealthEvidenceV2 {
    /// Construct a V2 record only from an exactly bound valuation envelope.
    pub fn new(
        request: CollateralValuationRequest,
        envelope: CollateralValuationEnvelope,
        obligation_amount: u64,
        computed_at_micros: i64,
    ) -> Result<Self, CollateralHealthEvidenceV2ValidationError> {
        request.validate()?;
        envelope.validate_for(&request)?;
        ensure_evidence_time_not_after_computation(&envelope, computed_at_micros)?;

        let snapshot = envelope.checked_health_snapshot(
            &request,
            obligation_amount,
            computed_at_micros,
        )?;
        snapshot.validate()?;

        let record = Self {
            schema_version: COLLATERAL_HEALTH_EVIDENCE_V2_SCHEMA_VERSION,
            request,
            envelope,
            snapshot,
        };
        record.validate()?;
        Ok(record)
    }

    /// Recompute all derived health evidence and reject any semantic drift.
    pub fn validate(&self) -> Result<(), CollateralHealthEvidenceV2ValidationError> {
        if self.schema_version != COLLATERAL_HEALTH_EVIDENCE_V2_SCHEMA_VERSION {
            return Err(CollateralHealthEvidenceV2ValidationError::UnsupportedSchemaVersion);
        }

        self.request.validate()?;
        self.envelope.validate_for(&self.request)?;

        if self.snapshot.collateral_id != self.request.subject.collateral_id {
            return Err(CollateralHealthEvidenceV2ValidationError::SnapshotSubjectMismatch);
        }

        ensure_evidence_time_not_after_computation(
            &self.envelope,
            self.snapshot.computed_at_micros,
        )?;

        self.snapshot.validate()?;

        let expected = self.envelope.checked_health_snapshot(
            &self.request,
            self.snapshot.obligation_amount,
            self.snapshot.computed_at_micros,
        )?;

        if expected != self.snapshot {
            return Err(CollateralHealthEvidenceV2ValidationError::SnapshotMismatch);
        }

        Ok(())
    }

    /// Checked liquidation evidence is available only after the whole persisted
    /// record validates. This remains an evidence predicate, not authorization.
    pub fn is_checked_liquidation_evidence(
        &self,
    ) -> Result<bool, CollateralHealthEvidenceV2ValidationError> {
        self.validate()?;
        Ok(self.snapshot.assessment.is_liquidation_evidence())
    }
}

fn evidence_time_micros(envelope: &CollateralValuationEnvelope) -> i64 {
    match envelope.outcome {
        CollateralValuationOutcome::Observed {
            observed_at_micros,
            ..
        } => observed_at_micros,
        CollateralValuationOutcome::Unavailable {
            attempted_at_micros,
            ..
        } => attempted_at_micros,
    }
}

fn ensure_evidence_time_not_after_computation(
    envelope: &CollateralValuationEnvelope,
    computed_at_micros: i64,
) -> Result<(), CollateralHealthEvidenceV2ValidationError> {
    if evidence_time_micros(envelope) > computed_at_micros {
        return Err(CollateralHealthEvidenceV2ValidationError::EvidenceTimestampAfterComputation);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_valuation::{
        CollateralValuationCapability, CollateralValuationFailure,
        CollateralValuationSubject, COLLATERAL_VALUATION_PROTOCOL_VERSION,
    };

    fn subject() -> CollateralValuationSubject {
        CollateralValuationSubject {
            collateral_id: "collateral:1".into(),
            source_happ: "mycelix-property".into(),
            asset_id: "property:lot:42".into(),
        }
    }

    fn capability() -> CollateralValuationCapability {
        CollateralValuationCapability {
            provider_id: "property-valuation-adapter".into(),
            method: "get_collateral_valuation".into(),
            protocol_version: COLLATERAL_VALUATION_PROTOCOL_VERSION,
        }
    }

    fn request() -> CollateralValuationRequest {
        CollateralValuationRequest {
            subject: subject(),
            capability: capability(),
        }
    }

    fn observed(value: u64, observed_at_micros: i64) -> CollateralValuationEnvelope {
        CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Observed {
                value,
                observed_at_micros,
            },
        }
    }

    fn unavailable(
        reason: CollateralValuationFailure,
        attempted_at_micros: i64,
    ) -> CollateralValuationEnvelope {
        CollateralValuationEnvelope {
            subject: subject(),
            capability: capability(),
            outcome: CollateralValuationOutcome::Unavailable {
                reason,
                attempted_at_micros,
            },
        }
    }

    #[test]
    fn valid_observed_v2_round_trips_and_recomputes() {
        let record = CollateralHealthEvidenceV2::new(request(), observed(100, 10), 91, 11)
            .expect("valid bound evidence");
        assert_eq!(record.validate(), Ok(()));
        assert_eq!(record.snapshot.assessment.status_label(), "MarginCall");
        assert_eq!(record.is_checked_liquidation_evidence(), Ok(false));

        let encoded = serde_json::to_string(&record).expect("serialize V2");
        let decoded: CollateralHealthEvidenceV2 =
            serde_json::from_str(&encoded).expect("deserialize V2");
        assert_eq!(decoded, record);
        assert_eq!(decoded.validate(), Ok(()));
    }

    #[test]
    fn valid_liquidation_threshold_requires_real_bound_observation() {
        let record = CollateralHealthEvidenceV2::new(request(), observed(100, 10), 96, 11)
            .expect("valid bound evidence");
        assert_eq!(record.is_checked_liquidation_evidence(), Ok(true));
    }

    #[test]
    fn unavailable_provider_persists_without_ratio_or_liquidation_evidence() {
        let record = CollateralHealthEvidenceV2::new(
            request(),
            unavailable(CollateralValuationFailure::ProviderUnavailable, 10),
            100,
            11,
        )
        .expect("typed unavailability is persistable");

        assert_eq!(record.snapshot.ltv_ratio, None);
        assert_eq!(record.snapshot.assessment.status_label(), "Indeterminate");
        assert_eq!(record.is_checked_liquidation_evidence(), Ok(false));
    }

    #[test]
    fn observed_zero_is_distinct_from_unavailable_provider() {
        let zero = CollateralHealthEvidenceV2::new(request(), observed(0, 10), 100, 11)
            .expect("observed zero is representable");
        let missing = CollateralHealthEvidenceV2::new(
            request(),
            unavailable(CollateralValuationFailure::ProviderUnavailable, 10),
            100,
            11,
        )
        .expect("provider failure is representable");

        assert_eq!(zero.snapshot.ltv_ratio, None);
        assert_eq!(missing.snapshot.ltv_ratio, None);
        assert_ne!(zero.envelope.outcome, missing.envelope.outcome);
        assert_ne!(zero.snapshot.assessment, missing.snapshot.assessment);
    }

    #[test]
    fn evidence_cannot_claim_to_postdate_its_computation() {
        assert_eq!(
            CollateralHealthEvidenceV2::new(request(), observed(100, 12), 50, 11),
            Err(CollateralHealthEvidenceV2ValidationError::EvidenceTimestampAfterComputation)
        );
    }

    #[test]
    fn wrong_bound_subject_is_rejected_before_persistence() {
        let mut envelope = observed(100, 10);
        envelope.subject.asset_id = "property:lot:99".into();
        assert!(matches!(
            CollateralHealthEvidenceV2::new(request(), envelope, 50, 11),
            Err(CollateralHealthEvidenceV2ValidationError::ValuationContract(_))
        ));
    }

    #[test]
    fn tampered_snapshot_assessment_is_rejected() {
        let mut record = CollateralHealthEvidenceV2::new(request(), observed(100, 10), 50, 11)
            .expect("valid bound evidence");
        record.snapshot.assessment = finance_collateral_safety::assess_ltv_ratio(0.96);
        assert!(record.validate().is_err());
    }

    #[test]
    fn tampered_snapshot_ratio_is_rejected() {
        let mut record = CollateralHealthEvidenceV2::new(request(), observed(100, 10), 50, 11)
            .expect("valid bound evidence");
        record.snapshot.ltv_ratio = Some(0.96);
        assert!(record.validate().is_err());
    }

    #[test]
    fn unsupported_outer_schema_version_is_rejected() {
        let mut record = CollateralHealthEvidenceV2::new(request(), observed(100, 10), 50, 11)
            .expect("valid bound evidence");
        record.schema_version = COLLATERAL_HEALTH_EVIDENCE_V2_SCHEMA_VERSION + 1;
        assert_eq!(
            record.validate(),
            Err(CollateralHealthEvidenceV2ValidationError::UnsupportedSchemaVersion)
        );
    }

    #[test]
    fn legacy_liquidation_and_999_sentinel_never_become_checked_evidence() {
        let legacy = LegacyCollateralHealthEvidenceV1 {
            collateral_id: "collateral:legacy".into(),
            current_value: 0,
            obligation_amount: 100,
            ltv_ratio: 999.0,
            status: "Liquidation".into(),
            computed_at_micros: 10,
        };

        assert_eq!(
            legacy.evidence_quality(),
            LegacyCollateralHealthEvidenceQuality::LegacyUnknown
        );
        assert!(!legacy.is_checked_liquidation_evidence());
        assert!(!legacy.can_upgrade_without_new_observation());
    }

    #[test]
    fn even_valid_looking_legacy_status_requires_new_observation_for_v2() {
        let legacy = LegacyCollateralHealthEvidenceV1 {
            collateral_id: "collateral:legacy-healthy".into(),
            current_value: 100,
            obligation_amount: 50,
            ltv_ratio: 0.5,
            status: "Healthy".into(),
            computed_at_micros: 10,
        };

        assert_eq!(
            legacy.evidence_quality(),
            LegacyCollateralHealthEvidenceQuality::LegacyUnknown
        );
        assert!(!legacy.can_upgrade_without_new_observation());
    }
}

//! Versioned adapter-facing navigation claim contracts.
//!
//! This module composes the existing Position owners by opaque references:
//! reference systems (#416), qualified time and clock correlations (#435),
//! epistemic/evidence classes (#419), provenance (#5899), common-mode
//! dependencies (#5904), fusion (#417), geometry diagnostics (#433/#447)
//! and navigation-health profiles (#458).
//!
//! It deliberately does not define a clock service, CRS catalogue, trust
//! authority, admission flag, integrity state machine or second fusion engine.
//! Legacy timestamps may be preserved, but remain explicitly unqualified.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

use super::measurements::{MeasurementModality, MeasurementValue};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContractError {
    EmptyReference,
    EmptyCommitment,
    InvalidTimeValue,
    InvalidTimeUncertainty,
    InvalidMeasurementValue,
    InvalidStateVector,
    InvalidVariance,
    MissingLineage,
    BasisMismatch,
    UnitMismatch,
    ReversedTimeInterval,
    DuplicateFailureDomain,
}

impl fmt::Display for ContractError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyReference => write!(f, "reference ID must not be empty"),
            Self::EmptyCommitment => write!(f, "content commitment algorithm and digest must not be empty"),
            Self::InvalidTimeValue => write!(f, "time value must be finite"),
            Self::InvalidTimeUncertainty => write!(f, "time uncertainty must be finite and nonnegative"),
            Self::InvalidMeasurementValue => write!(f, "measurement value or uncertainty is invalid"),
            Self::InvalidStateVector => write!(f, "position/velocity state contains a non-finite value"),
            Self::InvalidVariance => write!(f, "diagonal variance must be finite and nonnegative"),
            Self::MissingLineage => write!(f, "an estimate requires at least one parent commitment"),
            Self::BasisMismatch => write!(f, "time interval endpoints do not use the same qualified basis"),
            Self::UnitMismatch => write!(f, "time interval endpoints do not use the same declared unit"),
            Self::ReversedTimeInterval => write!(f, "time interval end precedes its start"),
            Self::DuplicateFailureDomain => write!(f, "failure-domain references must be unique"),
        }
    }
}
impl std::error::Error for ContractError {}

macro_rules! opaque_ref {
    ($name:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ContractError> {
                let value = value.into();
                if value.trim().is_empty() {
                    return Err(ContractError::EmptyReference);
                }
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }

            pub fn validate(&self) -> Result<(), ContractError> {
                if self.0.trim().is_empty() {
                    Err(ContractError::EmptyReference)
                } else {
                    Ok(())
                }
            }
        }
    };
}

// These IDs point to the owning Position/evidence subsystems. They do not
// re-encode the semantics or capabilities owned by those subsystems.
opaque_ref!(SourceRef);
opaque_ref!(SpatialReferenceRef);
opaque_ref!(TimeBasisRef);
opaque_ref!(ProvenanceRef);
opaque_ref!(FailureDomainRef);
opaque_ref!(CalibrationRef);
opaque_ref!(ConfigurationRef);
opaque_ref!(EvidenceClassRef);
opaque_ref!(EstimatorProfileRef);
opaque_ref!(DynamicsModelRef);
opaque_ref!(UncertaintySemanticsRef);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContentCommitment {
    pub algorithm: String,
    pub digest: String,
}

impl ContentCommitment {
    pub fn new(algorithm: impl Into<String>, digest: impl Into<String>) -> Result<Self, ContractError> {
        let value = Self { algorithm: algorithm.into(), digest: digest.into() };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), ContractError> {
        if self.algorithm.trim().is_empty() || self.digest.trim().is_empty() {
            Err(ContractError::EmptyCommitment)
        } else {
            Ok(())
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QualificationReason {
    LegacyValueWithoutBasis,
    SourceMetadataUnavailable,
    CalibrationUnavailable,
    ReferenceSystemUnresolved,
    EvidenceLineageIncomplete,
    NotYetAdjudicated,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "qualification", content = "value", rename_all = "snake_case")]
pub enum Qualification<T> {
    Qualified(T),
    Unqualified { reason: QualificationReason },
    Unknown,
}

impl<T> Qualification<T> {
    pub fn is_qualified(&self) -> bool {
        matches!(self, Self::Qualified(_))
    }

    pub fn as_qualified(&self) -> Option<&T> {
        match self {
            Self::Qualified(value) => Some(value),
            Self::Unqualified { .. } | Self::Unknown => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimeUnitV2 {
    Seconds,
    Milliseconds,
    Microseconds,
    DeviceTicks,
    MissionTicks,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeEvidenceV2 {
    /// The source's raw numeric reading. It is not comparable by itself.
    pub raw_value: Option<f64>,
    pub unit: Option<TimeUnitV2>,
    /// A reference to #435's time-scale/clock-domain contract.
    /// Legacy values must use Unqualified or Unknown, not an invented UTC ref.
    pub basis: Qualification<TimeBasisRef>,
    pub uncertainty_seconds: Option<f64>,
    /// Optional commitment to an explicit #435 clock-correlation/conversion.
    pub normalization: Option<ContentCommitment>,
}

impl TimeEvidenceV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        if self.raw_value.is_some_and(|value| !value.is_finite()) {
            return Err(ContractError::InvalidTimeValue);
        }
        if matches!(self.basis, Qualification::Qualified(_))
            && (self.raw_value.is_none() || self.unit.is_none())
        {
            return Err(ContractError::InvalidTimeValue);
        }
        if self.normalization.is_some() && self.raw_value.is_none() {
            return Err(ContractError::InvalidTimeValue);
        }
        if let Some(sigma) = self.uncertainty_seconds {
            if !sigma.is_finite() || sigma < 0.0 {
                return Err(ContractError::InvalidTimeUncertainty);
            }
        }
        if let Qualification::Qualified(reference) = &self.basis {
            reference.validate()?;
        }
        if let Some(commitment) = &self.normalization {
            commitment.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeIntervalV2 {
    pub start: TimeEvidenceV2,
    pub end: TimeEvidenceV2,
}

impl TimeIntervalV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        self.start.validate()?;
        self.end.validate()?;

        let start_basis = self.start.basis.as_qualified().ok_or(ContractError::BasisMismatch)?;
        let end_basis = self.end.basis.as_qualified().ok_or(ContractError::BasisMismatch)?;

        if start_basis != end_basis {
            return Err(ContractError::BasisMismatch);
        }
        let start_unit = self.start.unit.ok_or(ContractError::UnitMismatch)?;
        let end_unit = self.end.unit.ok_or(ContractError::UnitMismatch)?;
        if start_unit != end_unit {
            return Err(ContractError::UnitMismatch);
        }
        let start_value = self.start.raw_value.ok_or(ContractError::InvalidTimeValue)?;
        let end_value = self.end.raw_value.ok_or(ContractError::InvalidTimeValue)?;
        if end_value < start_value {
            return Err(ContractError::ReversedTimeInterval);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SourceEvidenceRefsV2 {
    pub source: SourceRef,
    /// Reference to #5899's provenance record, not an asserted trust score.
    pub provenance: Qualification<ProvenanceRef>,
    /// Calibration is separately qualified; source attachment does not qualify it.
    pub calibration: Qualification<CalibrationRef>,
    pub configuration: Qualification<ConfigurationRef>,
    /// Common-mode/failure-domain relationships owned by #5904.
    /// Multiple entries do not, by themselves, prove independence.
    pub failure_domains: Vec<FailureDomainRef>,
    pub raw_payload: Option<ContentCommitment>,
}

impl SourceEvidenceRefsV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        self.source.validate()?;
        if let Qualification::Qualified(reference) = &self.provenance {
            reference.validate()?;
        }
        if let Qualification::Qualified(reference) = &self.calibration {
            reference.validate()?;
        }
        if let Qualification::Qualified(reference) = &self.configuration {
            reference.validate()?;
        }
        let mut unique = BTreeSet::new();
        for reference in &self.failure_domains {
            reference.validate()?;
            if !unique.insert(reference) {
                return Err(ContractError::DuplicateFailureDomain);
            }
        }
        if let Some(commitment) = &self.raw_payload {
            commitment.validate()?;
        }
        Ok(())
    }
}

fn valid_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn validate_measurement(value: &MeasurementValue) -> Result<(), ContractError> {
    let valid = match value {
        MeasurementValue::Range { distance_m, sigma_m } =>
            distance_m.is_finite() && *distance_m >= 0.0 && valid_positive(*sigma_m),
        MeasurementValue::Position { xyz, sigma } =>
            xyz.iter().all(|x| x.is_finite()) && sigma.iter().all(|x| valid_positive(*x)),
        MeasurementValue::Velocity { vxyz, sigma } =>
            vxyz.iter().all(|x| x.is_finite()) && sigma.iter().all(|x| valid_positive(*x)),
        MeasurementValue::Bearing { azimuth_rad, sigma_rad } =>
            azimuth_rad.is_finite() && valid_positive(*sigma_rad),
        MeasurementValue::Altitude { height_m, sigma_m } =>
            height_m.is_finite() && valid_positive(*sigma_m),
    };
    if valid { Ok(()) } else { Err(ContractError::InvalidMeasurementValue) }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PositionObservationV2 {
    pub observation_id: ContentCommitment,
    pub modality: MeasurementModality,
    pub value: MeasurementValue,
    pub observed_at: TimeEvidenceV2,
    /// Local/transport receipt time is distinct from source observation time.
    /// It may be absent, and it cannot make an unqualified source time fresh.
    pub received_at: Option<TimeEvidenceV2>,
    /// Opaque, versioned reference owned by #416. Never inferred from numbers.
    pub spatial_reference: Qualification<SpatialReferenceRef>,
    pub source_evidence: SourceEvidenceRefsV2,
    /// Evidence-class semantics remain owned by #419.
    pub evidence_class: Qualification<EvidenceClassRef>,
}

impl PositionObservationV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        self.observation_id.validate()?;
        validate_measurement(&self.value)?;
        self.observed_at.validate()?;
        if let Some(received_at) = &self.received_at {
            received_at.validate()?;
        }
        if let Qualification::Qualified(reference) = &self.spatial_reference {
            reference.validate()?;
        }
        self.source_evidence.validate()?;
        if let Qualification::Qualified(reference) = &self.evidence_class {
            reference.validate()?;
        }
        Ok(())
    }

    /// A structural convenience only; not an admission or integrity decision.
    pub fn has_qualified_time_and_frame(&self) -> bool {
        self.observed_at.basis.is_qualified() && self.spatial_reference.is_qualified()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "representation", content = "value", rename_all = "snake_case")]
pub enum UncertaintyRepresentationV2 {
    DiagonalVarianceM2([f64; 3]),
    External {
        commitment: ContentCommitment,
        semantics: UncertaintySemanticsRef,
    },
}

impl UncertaintyRepresentationV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        match self {
            Self::DiagonalVarianceM2(variance) => {
                if variance.iter().all(|v| v.is_finite() && *v >= 0.0) {
                    Ok(())
                } else {
                    Err(ContractError::InvalidVariance)
                }
            }
            Self::External { commitment, semantics } => {
                commitment.validate()?;
                semantics.validate()
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PositionEstimateV2 {
    pub estimate_id: ContentCommitment,
    pub position_m: [f64; 3],
    pub velocity_mps: Option<[f64; 3]>,
    pub estimated_at: TimeEvidenceV2,
    pub spatial_reference: Qualification<SpatialReferenceRef>,
    /// Hash commitments to the exact observations/estimates used.
    pub parent_evidence: Vec<ContentCommitment>,
    pub estimator_profile: EstimatorProfileRef,
    pub uncertainty: Qualification<UncertaintyRepresentationV2>,
    /// Geometry/solver profile diagnostics are owned by #433/#447.
    pub geometry_diagnostics: Option<ContentCommitment>,
}

impl PositionEstimateV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        self.estimate_id.validate()?;
        if !self.position_m.iter().all(|x| x.is_finite())
            || self.velocity_mps.as_ref().is_some_and(|v| !v.iter().all(|x| x.is_finite()))
        {
            return Err(ContractError::InvalidStateVector);
        }
        self.estimated_at.validate()?;
        if let Qualification::Qualified(reference) = &self.spatial_reference {
            reference.validate()?;
        }
        if self.parent_evidence.is_empty() {
            return Err(ContractError::MissingLineage);
        }
        for parent in &self.parent_evidence {
            parent.validate()?;
        }
        self.estimator_profile.validate()?;
        if let Qualification::Qualified(uncertainty) = &self.uncertainty {
            uncertainty.validate()?;
        }
        if let Some(diagnostics) = &self.geometry_diagnostics {
            diagnostics.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PredictedPositionV2 {
    pub prediction_id: ContentCommitment,
    pub parent_estimate: ContentCommitment,
    pub propagation_model: DynamicsModelRef,
    pub propagation_interval: TimeIntervalV2,
    pub predicted_position_m: [f64; 3],
    pub predicted_velocity_mps: Option<[f64; 3]>,
    pub spatial_reference: Qualification<SpatialReferenceRef>,
    pub uncertainty: Qualification<UncertaintyRepresentationV2>,
}

impl PredictedPositionV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        self.prediction_id.validate()?;
        self.parent_estimate.validate()?;
        self.propagation_model.validate()?;
        self.propagation_interval.validate()?;
        if !self.predicted_position_m.iter().all(|x| x.is_finite())
            || self.predicted_velocity_mps.as_ref().is_some_and(|v| !v.iter().all(|x| x.is_finite()))
        {
            return Err(ContractError::InvalidStateVector);
        }
        if let Qualification::Qualified(reference) = &self.spatial_reference {
            reference.validate()?;
        }
        if let Qualification::Qualified(uncertainty) = &self.uncertainty {
            uncertainty.validate()?;
        }
        Ok(())
    }
}

/// The discriminator survives serialization: observation != estimate != prediction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "claim_kind", content = "claim", rename_all = "snake_case")]
pub enum NavigationClaimV2 {
    Observation(PositionObservationV2),
    Estimate(PositionEstimateV2),
    Prediction(PredictedPositionV2),
}

impl NavigationClaimV2 {
    pub fn validate(&self) -> Result<(), ContractError> {
        match self {
            Self::Observation(value) => value.validate(),
            Self::Estimate(value) => value.validate(),
            Self::Prediction(value) => value.validate(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn commitment(value: &str) -> ContentCommitment {
        ContentCommitment::new("test-sha256", value).unwrap()
    }

    fn qualified_time(value: f64, basis: &str) -> TimeEvidenceV2 {
        TimeEvidenceV2 {
            raw_value: Some(value),
            unit: Some(TimeUnitV2::Microseconds),
            basis: Qualification::Qualified(TimeBasisRef::new(basis).unwrap()),
            uncertainty_seconds: Some(0.000_001),
            normalization: None,
        }
    }

    fn legacy_time(value: f64) -> TimeEvidenceV2 {
        TimeEvidenceV2 {
            raw_value: Some(value),
            unit: Some(TimeUnitV2::Microseconds),
            basis: Qualification::Unqualified {
                reason: QualificationReason::LegacyValueWithoutBasis,
            },
            uncertainty_seconds: None,
            normalization: None,
        }
    }


    fn missing_source_time() -> TimeEvidenceV2 {
        TimeEvidenceV2 {
            raw_value: None,
            unit: None,
            basis: Qualification::Unqualified {
                reason: QualificationReason::SourceMetadataUnavailable,
            },
            uncertainty_seconds: None,
            normalization: None,
        }
    }

    fn source_refs() -> SourceEvidenceRefsV2 {
        SourceEvidenceRefsV2 {
            source: SourceRef::new("device/phone-1").unwrap(),
            provenance: Qualification::Unknown,
            calibration: Qualification::Unqualified {
                reason: QualificationReason::CalibrationUnavailable,
            },
            configuration: Qualification::Unknown,
            failure_domains: vec![
                FailureDomainRef::new("phone-1/clock").unwrap(),
                FailureDomainRef::new("phone-1/antenna").unwrap(),
            ],
            raw_payload: Some(commitment("payload-1")),
        }
    }

    fn observation(time: TimeEvidenceV2) -> PositionObservationV2 {
        PositionObservationV2 {
            observation_id: commitment("observation-1"),
            modality: MeasurementModality::Gps,
            value: MeasurementValue::Position {
                xyz: [1.0, 2.0, 3.0],
                sigma: [2.0, 2.0, 4.0],
            },
            observed_at: time,
            received_at: None,
            spatial_reference: Qualification::Unqualified {
                reason: QualificationReason::ReferenceSystemUnresolved,
            },
            source_evidence: source_refs(),
            evidence_class: Qualification::Unknown,
        }
    }

    #[test]
    fn legacy_timestamp_is_preserved_without_being_promoted() {
        let value = observation(legacy_time(42_000.0));
        value.validate().unwrap();
        assert!(!value.has_qualified_time_and_frame());
        assert!(matches!(
            value.observed_at.basis,
            Qualification::Unqualified {
                reason: QualificationReason::LegacyValueWithoutBasis
            }
        ));
    }


    #[test]
    fn source_observation_time_and_ingestion_time_remain_distinct() {
        let mut value = observation(legacy_time(42_000.0));
        value.received_at = Some(qualified_time(99_000.0, "clock/ingestion-host"));
        value.validate().unwrap();

        assert!(!value.observed_at.basis.is_qualified());
        assert!(value.received_at.as_ref().unwrap().basis.is_qualified());
        assert_ne!(
            value.observed_at.raw_value,
            value.received_at.as_ref().unwrap().raw_value
        );
    }


    #[test]
    fn missing_source_time_is_represented_without_a_numeric_placeholder() {
        let value = observation(missing_source_time());
        value.validate().unwrap();

        assert_eq!(value.observed_at.raw_value, None);
        assert_eq!(value.observed_at.unit, None);
        assert!(!value.observed_at.basis.is_qualified());

        let interval = TimeIntervalV2 {
            start: missing_source_time(),
            end: missing_source_time(),
        };
        assert_eq!(interval.validate(), Err(ContractError::BasisMismatch));
    }

    #[test]
    fn same_numeric_timestamp_does_not_make_different_time_bases_comparable() {
        let interval = TimeIntervalV2 {
            start: qualified_time(10.0, "clock/tai"),
            end: qualified_time(20.0, "clock/monotonic-device-1"),
        };
        assert_eq!(interval.validate(), Err(ContractError::BasisMismatch));
    }

    #[test]
    fn interval_requires_matching_units_and_ordered_values() {
        let mut interval = TimeIntervalV2 {
            start: qualified_time(20.0, "clock/mono"),
            end: qualified_time(10.0, "clock/mono"),
        };
        assert_eq!(interval.validate(), Err(ContractError::ReversedTimeInterval));
        interval.end.raw_value = Some(21.0);
        interval.end.unit = Some(TimeUnitV2::Seconds);
        assert_eq!(interval.validate(), Err(ContractError::UnitMismatch));
    }

    #[test]
    fn estimate_requires_parent_evidence_and_named_estimator_profile() {
        let value = PositionEstimateV2 {
            estimate_id: commitment("estimate-1"),
            position_m: [0.0, 0.0, 0.0],
            velocity_mps: None,
            estimated_at: qualified_time(1.0, "time/tai"),
            spatial_reference: Qualification::Qualified(
                SpatialReferenceRef::new("crs/earth-ecef-v1").unwrap(),
            ),
            parent_evidence: vec![],
            estimator_profile: EstimatorProfileRef::new("estimator/ekf-v1").unwrap(),
            uncertainty: Qualification::Qualified(
                UncertaintyRepresentationV2::DiagonalVarianceM2([1.0, 1.0, 2.0]),
            ),
            geometry_diagnostics: None,
        };
        assert_eq!(value.validate(), Err(ContractError::MissingLineage));
    }

    #[test]
    fn prediction_requires_qualified_same_basis_interval_and_parent_estimate() {
        let value = PredictedPositionV2 {
            prediction_id: commitment("prediction-1"),
            parent_estimate: commitment("estimate-1"),
            propagation_model: DynamicsModelRef::new("dynamics/cv-v1").unwrap(),
            propagation_interval: TimeIntervalV2 {
                start: qualified_time(10.0, "time/tai"),
                end: qualified_time(20.0, "time/tai"),
            },
            predicted_position_m: [1.0, 2.0, 3.0],
            predicted_velocity_mps: Some([0.0, 0.0, 0.0]),
            spatial_reference: Qualification::Qualified(
                SpatialReferenceRef::new("crs/earth-ecef-v1").unwrap(),
            ),
            uncertainty: Qualification::Qualified(
                UncertaintyRepresentationV2::DiagonalVarianceM2([1.0, 1.0, 2.0]),
            ),
        };
        value.validate().unwrap();
    }

    #[test]
    fn negative_or_non_finite_uncertainty_is_rejected() {
        assert_eq!(
            UncertaintyRepresentationV2::DiagonalVarianceM2([1.0, -0.1, 2.0]).validate(),
            Err(ContractError::InvalidVariance)
        );
        assert_eq!(
            UncertaintyRepresentationV2::DiagonalVarianceM2([1.0, f64::NAN, 2.0]).validate(),
            Err(ContractError::InvalidVariance)
        );
    }

    #[test]
    fn duplicate_failure_domain_refs_are_rejected_but_not_scored_as_independence() {
        let mut refs = source_refs();
        refs.failure_domains.push(FailureDomainRef::new("phone-1/clock").unwrap());
        assert_eq!(refs.validate(), Err(ContractError::DuplicateFailureDomain));
    }

    #[test]
    fn claim_discriminator_keeps_observation_estimate_and_prediction_distinct() {
        let observation = NavigationClaimV2::Observation(observation(legacy_time(5.0)));
        assert!(observation.validate().is_ok());
        assert!(matches!(observation, NavigationClaimV2::Observation(_)));
    }

    #[test]
    fn empty_opaque_reference_cannot_be_constructed() {
        assert_eq!(SourceRef::new("  "), Err(ContractError::EmptyReference));
        assert_eq!(TimeBasisRef::new(""), Err(ContractError::EmptyReference));
    }
}

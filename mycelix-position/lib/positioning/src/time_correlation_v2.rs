//! Typed clock-correlation evidence for Position v2.
//!
//! This module validates the provenance and admissibility of a correlation
//! output. It deliberately does not perform clock conversion, authenticate
//! correlation commitments, synchronize clocks, or assert physical truth.
//! Consumers must preserve source and destination bases and uncertainty.

use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimeCorrelationError {
    EmptyReference,
    EmptyCommitment,
    InvalidTimeValue,
    InvalidUncertainty,
    InvalidValidityInterval,
    InvalidDestinationBasis,
    SourceClockMismatch,
    DestinationBasisMismatch,
    SourceUnitMismatch,
    SourceOutsideValidity,
    CorrelationIdentityMismatch,
    CorrelationCommitmentMismatch,
    InputCommitmentMismatch,
    OutputCommitmentMismatch,
    UncertaintyNotPropagated,
}

impl fmt::Display for TimeCorrelationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyReference => write!(f, "time-correlation reference must not be empty"),
            Self::EmptyCommitment => write!(f, "time-correlation commitment must not be empty"),
            Self::InvalidTimeValue => write!(f, "time reading must be finite"),
            Self::InvalidUncertainty => write!(f, "time uncertainty must be finite and nonnegative"),
            Self::InvalidValidityInterval => write!(f, "source-clock validity interval is invalid"),
            Self::InvalidDestinationBasis => write!(f, "clock correlation destination must be an explicit time scale"),
            Self::SourceClockMismatch => write!(f, "source reading does not match the correlated clock and boot epoch"),
            Self::DestinationBasisMismatch => write!(f, "normalized reading does not match the declared destination scale and epoch"),
            Self::SourceUnitMismatch => write!(f, "source reading unit differs from the correlation validity interval"),
            Self::SourceOutsideValidity => write!(f, "source reading is outside the correlation validity interval"),
            Self::CorrelationIdentityMismatch => write!(f, "normalized reading references a different correlation"),
            Self::CorrelationCommitmentMismatch => write!(f, "normalized reading does not bind the exact correlation commitment"),
            Self::InputCommitmentMismatch => write!(f, "input commitment does not match the raw source reading"),
            Self::OutputCommitmentMismatch => write!(f, "output commitment does not match the normalized reading"),
            Self::UncertaintyNotPropagated => write!(f, "normalized time does not preserve the declared worst-case uncertainty bound"),
        }
    }
}
impl std::error::Error for TimeCorrelationError {}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "scale", content = "parameters", rename_all = "snake_case")]
pub enum TimeScaleIdV2 {
    Tai,
    Tt,
    Tdb,
    Gps,
    UnixTime { profile_version: String },
    Named { authority: String, code: String, version: String },
}

impl TimeScaleIdV2 {
    pub fn validate(&self) -> Result<(), TimeCorrelationError> {
        match self {
            Self::Tai | Self::Tt | Self::Tdb | Self::Gps => Ok(()),
            Self::UnixTime { profile_version } if !profile_version.trim().is_empty() => Ok(()),
            Self::Named { authority, code, version }
                if !authority.trim().is_empty()
                    && !code.trim().is_empty()
                    && !version.trim().is_empty() => Ok(()),
            _ => Err(TimeCorrelationError::EmptyReference),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "clock_domain", content = "identity", rename_all = "snake_case")]
pub enum ClockDomainV2 {
    Monotonic {
        subject_ref: String,
        clock_id: String,
        boot_epoch_id: String,
    },
    Spacecraft {
        mission_or_subject: String,
        clock_id: String,
        partition: Option<String>,
    },
    External {
        authority: String,
        clock_id: String,
        version: String,
        epoch_id: String,
    },
}

impl ClockDomainV2 {
    pub fn validate(&self) -> Result<(), TimeCorrelationError> {
        let valid = match self {
            Self::Monotonic { subject_ref, clock_id, boot_epoch_id } =>
                !subject_ref.trim().is_empty()
                    && !clock_id.trim().is_empty()
                    && !boot_epoch_id.trim().is_empty(),
            Self::Spacecraft { mission_or_subject, clock_id, partition } =>
                !mission_or_subject.trim().is_empty()
                    && !clock_id.trim().is_empty()
                    && partition.as_ref().is_none_or(|value| !value.trim().is_empty()),
            Self::External { authority, clock_id, version, epoch_id } =>
                !authority.trim().is_empty()
                    && !clock_id.trim().is_empty()
                    && !version.trim().is_empty()
                    && !epoch_id.trim().is_empty(),
        };
        if valid { Ok(()) } else { Err(TimeCorrelationError::EmptyReference) }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "basis", content = "identity", rename_all = "snake_case")]
pub enum TimeBasisV2 {
    Scale { scale: TimeScaleIdV2, epoch_id: String },
    Clock(ClockDomainV2),
}

impl TimeBasisV2 {
    pub fn validate(&self) -> Result<(), TimeCorrelationError> {
        match self {
            Self::Scale { scale, epoch_id } => {
                scale.validate()?;
                if epoch_id.trim().is_empty() {
                    return Err(TimeCorrelationError::EmptyReference);
                }
                Ok(())
            }
            Self::Clock(clock) => clock.validate(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimeUnitV2 {
    Seconds,
    Milliseconds,
    Microseconds,
    Nanoseconds,
    DeviceTicks,
    MissionTicks,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TimeCommitmentV2 {
    pub algorithm: String,
    pub digest: String,
}

impl TimeCommitmentV2 {
    pub fn validate(&self) -> Result<(), TimeCorrelationError> {
        if self.algorithm.trim().is_empty() || self.digest.trim().is_empty() {
            Err(TimeCorrelationError::EmptyCommitment)
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct QualifiedInstantV2 {
    pub value: f64,
    pub unit: TimeUnitV2,
    pub basis: TimeBasisV2,
    /// A hard absolute error bound, not an implicit standard deviation.
    pub uncertainty_bound_seconds: f64,
    /// Commitment to this exact source or normalized instant record.
    pub content_commitment: TimeCommitmentV2,
}

impl QualifiedInstantV2 {
    pub fn validate(&self) -> Result<(), TimeCorrelationError> {
        if !self.value.is_finite() {
            return Err(TimeCorrelationError::InvalidTimeValue);
        }
        if !self.uncertainty_bound_seconds.is_finite() || self.uncertainty_bound_seconds < 0.0 {
            return Err(TimeCorrelationError::InvalidUncertainty);
        }
        self.basis.validate()?;
        self.content_commitment.validate()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SourceClockValidityV2 {
    pub start_value: f64,
    pub end_value: f64,
    pub unit: TimeUnitV2,
    pub basis: ClockDomainV2,
}

impl SourceClockValidityV2 {
    pub fn validate(&self, expected_clock: &ClockDomainV2) -> Result<(), TimeCorrelationError> {
        self.basis.validate()?;
        if &self.basis != expected_clock {
            return Err(TimeCorrelationError::InvalidValidityInterval);
        }
        if !self.start_value.is_finite()
            || !self.end_value.is_finite()
            || self.end_value < self.start_value
        {
            return Err(TimeCorrelationError::InvalidValidityInterval);
        }
        Ok(())
    }

    pub fn contains(&self, value: f64) -> bool {
        value >= self.start_value && value <= self.end_value
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClockCorrelationUncertaintyV2 {
    /// Worst-case absolute error bound introduced by the correlation model.
    pub maximum_error_bound_seconds: f64,
    /// Optional statistical characterization; never substituted for the hard bound.
    pub standard_deviation_seconds: Option<f64>,
}

impl ClockCorrelationUncertaintyV2 {
    pub fn validate(&self) -> Result<(), TimeCorrelationError> {
        if !self.maximum_error_bound_seconds.is_finite()
            || self.maximum_error_bound_seconds < 0.0
            || self.standard_deviation_seconds
                .is_some_and(|sigma| !sigma.is_finite() || sigma < 0.0)
        {
            return Err(TimeCorrelationError::InvalidUncertainty);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClockCorrelationV2 {
    pub correlation_id: String,
    pub source_clock: ClockDomainV2,
    /// Must be a scale plus an explicit epoch (e.g. TAI/J2000 or Unix/1970).
    pub destination_basis: TimeBasisV2,
    pub profile_id: String,
    pub profile_version: String,
    /// SPICE kernel, calibration, correlation table, or other profile material.
    pub calibration_or_kernel_commitment: TimeCommitmentV2,
    /// The interval in which this source-clock correlation is valid.
    pub valid_source_interval: SourceClockValidityV2,
    pub uncertainty: ClockCorrelationUncertaintyV2,
    /// Commitment to the complete correlation record and its parameters.
    pub correlation_commitment: TimeCommitmentV2,
}

impl ClockCorrelationV2 {
    pub fn validate(&self) -> Result<(), TimeCorrelationError> {
        if self.correlation_id.trim().is_empty()
            || self.profile_id.trim().is_empty()
            || self.profile_version.trim().is_empty()
        {
            return Err(TimeCorrelationError::EmptyReference);
        }
        self.source_clock.validate()?;
        match &self.destination_basis {
            TimeBasisV2::Scale { scale, epoch_id } => {
                scale.validate()?;
                if epoch_id.trim().is_empty() {
                    return Err(TimeCorrelationError::EmptyReference);
                }
            }
            TimeBasisV2::Clock(_) => return Err(TimeCorrelationError::InvalidDestinationBasis),
        }
        self.calibration_or_kernel_commitment.validate()?;
        self.valid_source_interval.validate(&self.source_clock)?;
        self.uncertainty.validate()?;
        self.correlation_commitment.validate()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NormalizedInstantV2 {
    pub source_reading: QualifiedInstantV2,
    pub normalized_reading: QualifiedInstantV2,
    pub correlation_id: String,
    pub correlation_commitment: TimeCommitmentV2,
    pub input_commitment: TimeCommitmentV2,
    pub output_commitment: TimeCommitmentV2,
}

impl NormalizedInstantV2 {
    /// Validate a caller-supplied correlation result. No clock conversion is
    /// performed here; the time owner supplies the normalized output value.
    pub fn validate_against(
        &self,
        correlation: &ClockCorrelationV2,
    ) -> Result<(), TimeCorrelationError> {
        correlation.validate()?;
        self.source_reading.validate()?;
        self.normalized_reading.validate()?;
        if self.correlation_id != correlation.correlation_id {
            return Err(TimeCorrelationError::CorrelationIdentityMismatch);
        }
        if self.correlation_commitment != correlation.correlation_commitment {
            return Err(TimeCorrelationError::CorrelationCommitmentMismatch);
        }
        if self.source_reading.basis != TimeBasisV2::Clock(correlation.source_clock.clone()) {
            return Err(TimeCorrelationError::SourceClockMismatch);
        }
        if self.source_reading.unit != correlation.valid_source_interval.unit {
            return Err(TimeCorrelationError::SourceUnitMismatch);
        }
        if !correlation.valid_source_interval.contains(self.source_reading.value) {
            return Err(TimeCorrelationError::SourceOutsideValidity);
        }
        if self.normalized_reading.basis != correlation.destination_basis {
            return Err(TimeCorrelationError::DestinationBasisMismatch);
        }
        if self.input_commitment != self.source_reading.content_commitment {
            return Err(TimeCorrelationError::InputCommitmentMismatch);
        }
        if self.output_commitment != self.normalized_reading.content_commitment {
            return Err(TimeCorrelationError::OutputCommitmentMismatch);
        }
        let required_bound = self.source_reading.uncertainty_bound_seconds
            + correlation.uncertainty.maximum_error_bound_seconds;
        if !required_bound.is_finite()
            || self.normalized_reading.uncertainty_bound_seconds < required_bound
        {
            return Err(TimeCorrelationError::UncertaintyNotPropagated);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn commitment(value: &str) -> TimeCommitmentV2 {
        TimeCommitmentV2 { algorithm: "sha256".into(), digest: value.into() }
    }

    fn source_clock() -> ClockDomainV2 {
        ClockDomainV2::Monotonic {
            subject_ref: "device/phone-1".into(),
            clock_id: "monotonic-1".into(),
            boot_epoch_id: "boot-7".into(),
        }
    }

    fn destination_basis() -> TimeBasisV2 {
        TimeBasisV2::Scale {
            scale: TimeScaleIdV2::Tai,
            epoch_id: "tai-epoch-1970".into(),
        }
    }

    fn correlation() -> ClockCorrelationV2 {
        ClockCorrelationV2 {
            correlation_id: "corr-1".into(),
            source_clock: source_clock(),
            destination_basis: destination_basis(),
            profile_id: "phone-clock-correlation".into(),
            profile_version: "2".into(),
            calibration_or_kernel_commitment: commitment("kernel-a"),
            valid_source_interval: SourceClockValidityV2 {
                start_value: 0.0,
                end_value: 100.0,
                unit: TimeUnitV2::DeviceTicks,
                basis: source_clock(),
            },
            uncertainty: ClockCorrelationUncertaintyV2 {
                maximum_error_bound_seconds: 0.005,
                standard_deviation_seconds: Some(0.002),
            },
            correlation_commitment: commitment("correlation-a"),
        }
    }

    fn normalized() -> NormalizedInstantV2 {
        let source = QualifiedInstantV2 {
            value: 42.0,
            unit: TimeUnitV2::DeviceTicks,
            basis: TimeBasisV2::Clock(source_clock()),
            uncertainty_bound_seconds: 0.001,
            content_commitment: commitment("raw-reading-a"),
        };
        let output = QualifiedInstantV2 {
            value: 1_700_000_000.0,
            unit: TimeUnitV2::Seconds,
            basis: destination_basis(),
            uncertainty_bound_seconds: 0.006,
            content_commitment: commitment("normalized-reading-a"),
        };
        NormalizedInstantV2 {
            source_reading: source.clone(),
            normalized_reading: output.clone(),
            correlation_id: "corr-1".into(),
            correlation_commitment: commitment("correlation-a"),
            input_commitment: source.content_commitment,
            output_commitment: output.content_commitment,
        }
    }

    #[test]
    fn valid_clock_correlation_preserves_source_and_destination_lineage() {
        assert_eq!(normalized().validate_against(&correlation()), Ok(()));
    }

    #[test]
    fn source_clock_and_boot_epoch_must_match_exactly() {
        let mut value = normalized();
        value.source_reading.basis = TimeBasisV2::Clock(ClockDomainV2::Monotonic {
            subject_ref: "device/phone-1".into(),
            clock_id: "monotonic-1".into(),
            boot_epoch_id: "boot-8".into(),
        });
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::SourceClockMismatch)
        );
    }

    #[test]
    fn destination_scale_and_epoch_must_match_exactly() {
        let mut value = normalized();
        value.normalized_reading.basis = TimeBasisV2::Scale {
            scale: TimeScaleIdV2::UnixTime { profile_version: "1".into() },
            epoch_id: "unix-1970".into(),
        };
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::DestinationBasisMismatch)
        );
    }

    #[test]
    fn numeric_reading_outside_correlation_validity_is_rejected() {
        let mut value = normalized();
        value.source_reading.value = 101.0;
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::SourceOutsideValidity)
        );
    }

    #[test]
    fn source_unit_must_match_validity_interval_unit() {
        let mut value = normalized();
        value.source_reading.unit = TimeUnitV2::Milliseconds;
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::SourceUnitMismatch)
        );
    }

    #[test]
    fn correlation_id_and_commitment_must_match_the_claimed_output() {
        let mut value = normalized();
        value.correlation_id = "another-correlation".into();
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::CorrelationIdentityMismatch)
        );
        let mut value = normalized();
        value.correlation_commitment = commitment("other-digest");
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::CorrelationCommitmentMismatch)
        );
    }

    #[test]
    fn input_and_output_commitments_must_match_exact_readings() {
        let mut value = normalized();
        value.input_commitment = commitment("other-input");
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::InputCommitmentMismatch)
        );
        let mut value = normalized();
        value.output_commitment = commitment("other-output");
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::OutputCommitmentMismatch)
        );
    }

    #[test]
    fn correlation_error_bound_must_be_propagated_conservatively() {
        let mut value = normalized();
        value.normalized_reading.uncertainty_bound_seconds = 0.005;
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::UncertaintyNotPropagated)
        );
    }

    #[test]
    fn invalid_source_interval_and_invalid_destination_basis_are_rejected() {
        let mut invalid_interval = correlation();
        invalid_interval.valid_source_interval.start_value = 110.0;
        assert_eq!(
            invalid_interval.validate(),
            Err(TimeCorrelationError::InvalidValidityInterval)
        );

        let mut invalid_destination = correlation();
        invalid_destination.destination_basis = TimeBasisV2::Clock(source_clock());
        assert_eq!(
            invalid_destination.validate(),
            Err(TimeCorrelationError::InvalidDestinationBasis)
        );
    }

    #[test]
    fn correlation_requires_calibration_and_correlation_commitments() {
        let mut correlation = correlation();
        correlation.calibration_or_kernel_commitment.digest.clear();
        assert_eq!(
            correlation.validate(),
            Err(TimeCorrelationError::EmptyCommitment)
        );
        let mut correlation = correlation();
        correlation.correlation_commitment.digest.clear();
        assert_eq!(
            correlation.validate(),
            Err(TimeCorrelationError::EmptyCommitment)
        );
    }

    #[test]
    fn qualified_time_round_trips_without_erasing_clock_lineage() {
        let value = normalized();
        let encoded = serde_json::to_vec(&value).unwrap();
        let decoded: NormalizedInstantV2 = serde_json::from_slice(&encoded).unwrap();
        assert_eq!(decoded, value);
        assert_eq!(decoded.source_reading.basis, TimeBasisV2::Clock(source_clock()));
        assert_eq!(decoded.normalized_reading.basis, destination_basis());
    }

    #[test]
    fn equal_numeric_values_on_different_time_scales_are_not_equivalent() {
        let mut value = normalized();
        value.normalized_reading.value = 1_700_000_000.0;
        value.normalized_reading.basis = TimeBasisV2::Scale {
            scale: TimeScaleIdV2::UnixTime { profile_version: "1".into() },
            epoch_id: "unix-1970".into(),
        };
        assert_eq!(
            value.validate_against(&correlation()),
            Err(TimeCorrelationError::DestinationBasisMismatch)
        );
    }
}

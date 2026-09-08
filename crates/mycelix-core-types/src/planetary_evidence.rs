// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-light planetary evidence primitives.
//!
//! These types are the Mycelix payload contract for environmental facts and
//! model products. They deliberately do not define an AI/world-interface
//! envelope (Symthaea already owns that boundary), nor do they define physical
//! risk scores. Their job is narrower: make the measurement, provenance,
//! space/time validity, uncertainty, and epistemic classification of a datum
//! explicit and independently validatable.

use crate::EpistemicClassification;
use std::fmt;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const PLANETARY_EVIDENCE_SCHEMA_VERSION: u16 = 1;
pub const MAX_ID_BYTES: usize = 256;
pub const MAX_UNIT_BYTES: usize = 64;
pub const MAX_EVIDENCE_REFS: usize = 64;
pub const MAX_SOURCE_BYTES: usize = 256;
pub const MAX_RESOURCE_BYTES: usize = 1024;
pub const MAX_DIGEST_BYTES: usize = 256;
pub const MAX_LICENSE_BYTES: usize = 128;

/// Relationship between a datum and reality.
///
/// This classification is intentionally orthogonal to [`EpistemicClassification`].
/// `EvidenceClass` answers *what kind of product is this?* while E/N/M/H answers
/// *how should its epistemic/normative/material status be interpreted?*
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum EvidenceClass {
    /// Directly observed by a sensor, human witness, instrument, or source record.
    Observed,
    /// Deterministically transformed from one or more source observations.
    Derived,
    /// Estimated latent state or conclusion inferred from evidence.
    Inferred,
    /// Prediction whose validity interval lies wholly or partly in the future.
    Forecast,
    /// Counterfactual, hypothetical, synthetic, or planning scenario.
    Scenario,
}

/// A scalar measurement with an explicit unit identifier.
///
/// `unit` is an opaque identifier at this boundary (for example `Cel`, `K`,
/// `mm/d`, or a UCUM code). Unit conversion belongs in a dedicated adapter,
/// not in the evidence record itself.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Measurement {
    pub value: f64,
    pub unit: String,
}

impl Measurement {
    pub fn new(value: f64, unit: impl Into<String>) -> Result<Self, PlanetaryEvidenceError> {
        let measurement = Self {
            value,
            unit: unit.into(),
        };
        measurement.validate()?;
        Ok(measurement)
    }

    pub fn validate(&self) -> Result<(), PlanetaryEvidenceError> {
        require_finite("measurement.value", self.value)?;
        require_text("measurement.unit", &self.unit, MAX_UNIT_BYTES)
    }
}

/// A validated geographic point in WGS84 decimal degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GeoPoint {
    pub latitude: f64,
    pub longitude: f64,
}

impl GeoPoint {
    pub fn new(latitude: f64, longitude: f64) -> Result<Self, PlanetaryEvidenceError> {
        let point = Self {
            latitude,
            longitude,
        };
        point.validate()?;
        Ok(point)
    }

    pub fn validate(&self) -> Result<(), PlanetaryEvidenceError> {
        require_finite("spatial.latitude", self.latitude)?;
        require_finite("spatial.longitude", self.longitude)?;
        if !(-90.0..=90.0).contains(&self.latitude) {
            return Err(PlanetaryEvidenceError::OutOfRange {
                field: "spatial.latitude",
                min: -90.0,
                max: 90.0,
                actual: self.latitude,
            });
        }
        if !(-180.0..=180.0).contains(&self.longitude) {
            return Err(PlanetaryEvidenceError::OutOfRange {
                field: "spatial.longitude",
                min: -180.0,
                max: 180.0,
                actual: self.longitude,
            });
        }
        Ok(())
    }
}

/// Geographic support of one datum.
///
/// Bounding boxes permit `west > east` so regions that cross the antimeridian
/// remain representable without a second special-case type.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum SpatialExtent {
    Point(GeoPoint),
    BoundingBox {
        south: f64,
        west: f64,
        north: f64,
        east: f64,
    },
    /// Stable external or Mycelix region identifier when geometry is resolved elsewhere.
    RegionId(String),
}

impl SpatialExtent {
    pub fn validate(&self) -> Result<(), PlanetaryEvidenceError> {
        match self {
            Self::Point(point) => point.validate(),
            Self::BoundingBox {
                south,
                west,
                north,
                east,
            } => {
                for (field, value) in [
                    ("spatial.south", *south),
                    ("spatial.west", *west),
                    ("spatial.north", *north),
                    ("spatial.east", *east),
                ] {
                    require_finite(field, value)?;
                }
                if *south < -90.0 || *south > 90.0 {
                    return Err(PlanetaryEvidenceError::OutOfRange {
                        field: "spatial.south",
                        min: -90.0,
                        max: 90.0,
                        actual: *south,
                    });
                }
                if *north < -90.0 || *north > 90.0 {
                    return Err(PlanetaryEvidenceError::OutOfRange {
                        field: "spatial.north",
                        min: -90.0,
                        max: 90.0,
                        actual: *north,
                    });
                }
                if *west < -180.0 || *west > 180.0 {
                    return Err(PlanetaryEvidenceError::OutOfRange {
                        field: "spatial.west",
                        min: -180.0,
                        max: 180.0,
                        actual: *west,
                    });
                }
                if *east < -180.0 || *east > 180.0 {
                    return Err(PlanetaryEvidenceError::OutOfRange {
                        field: "spatial.east",
                        min: -180.0,
                        max: 180.0,
                        actual: *east,
                    });
                }
                if south > north {
                    return Err(PlanetaryEvidenceError::InvalidInterval(
                        "spatial south must not exceed north",
                    ));
                }
                Ok(())
            }
            Self::RegionId(id) => require_text("spatial.region_id", id, MAX_ID_BYTES),
        }
    }
}

/// Time interval over which a datum is valid, in Unix seconds.
///
/// Equal bounds represent an instantaneous observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TemporalExtent {
    pub start: i64,
    pub end: i64,
}

impl TemporalExtent {
    pub fn new(start: i64, end: i64) -> Result<Self, PlanetaryEvidenceError> {
        let extent = Self { start, end };
        extent.validate()?;
        Ok(extent)
    }

    pub fn instant(at: i64) -> Self {
        Self { start: at, end: at }
    }

    pub fn validate(&self) -> Result<(), PlanetaryEvidenceError> {
        if self.start > self.end {
            return Err(PlanetaryEvidenceError::InvalidInterval(
                "temporal start must not exceed end",
            ));
        }
        Ok(())
    }
}

/// Quantitative uncertainty attached to a scalar measurement.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Uncertainty {
    /// Uncertainty is not supplied. This is explicit rather than silently
    /// treating the measurement as exact.
    Unspecified,
    /// Closed interval with optional stated coverage probability in [0, 1].
    Interval {
        lower: f64,
        upper: f64,
        confidence: Option<f64>,
    },
    /// Standard deviation in the same unit as the measurement.
    StandardDeviation(f64),
}

impl Uncertainty {
    pub fn validate(&self) -> Result<(), PlanetaryEvidenceError> {
        match self {
            Self::Unspecified => Ok(()),
            Self::Interval {
                lower,
                upper,
                confidence,
            } => {
                require_finite("uncertainty.lower", *lower)?;
                require_finite("uncertainty.upper", *upper)?;
                if lower > upper {
                    return Err(PlanetaryEvidenceError::InvalidInterval(
                        "uncertainty lower bound must not exceed upper bound",
                    ));
                }
                if let Some(confidence) = confidence {
                    require_finite("uncertainty.confidence", *confidence)?;
                    if !(0.0..=1.0).contains(confidence) {
                        return Err(PlanetaryEvidenceError::OutOfRange {
                            field: "uncertainty.confidence",
                            min: 0.0,
                            max: 1.0,
                            actual: *confidence,
                        });
                    }
                }
                Ok(())
            }
            Self::StandardDeviation(sigma) => {
                require_finite("uncertainty.standard_deviation", *sigma)?;
                if *sigma < 0.0 {
                    return Err(PlanetaryEvidenceError::OutOfRange {
                        field: "uncertainty.standard_deviation",
                        min: 0.0,
                        max: f64::MAX,
                        actual: *sigma,
                    });
                }
                Ok(())
            }
        }
    }
}

/// Reference to the source artifact from which a planetary datum derives.
///
/// The source payload itself may live in STAC, SensorThings, EDR, WIS2,
/// another Mycelix hApp, or an external object store. The evidence plane keeps
/// a stable source/resource identity and, where available, a content digest.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ExternalEvidenceRef {
    pub source_system: String,
    pub resource_id: String,
    /// Algorithm-qualified digest such as `sha256:<hex>` or `blake3:<hex>`.
    pub content_digest: Option<String>,
    /// Unix seconds when the referenced representation was retrieved.
    pub retrieved_at: Option<i64>,
    pub license: Option<String>,
}

impl ExternalEvidenceRef {
    pub fn validate(&self) -> Result<(), PlanetaryEvidenceError> {
        require_text("evidence.source_system", &self.source_system, MAX_SOURCE_BYTES)?;
        require_text("evidence.resource_id", &self.resource_id, MAX_RESOURCE_BYTES)?;
        if let Some(digest) = &self.content_digest {
            require_text("evidence.content_digest", digest, MAX_DIGEST_BYTES)?;
            if !digest.contains(':') {
                return Err(PlanetaryEvidenceError::MalformedField {
                    field: "evidence.content_digest",
                    reason: "digest must be algorithm-qualified (for example sha256:<hex>)",
                });
            }
        }
        if let Some(license) = &self.license {
            require_text("evidence.license", license, MAX_LICENSE_BYTES)?;
        }
        Ok(())
    }
}

/// Canonical Mycelix environmental datum.
///
/// This type intentionally remains a payload rather than an authority-bearing
/// action. Symthaea can wrap it in its world-interface observation envelope;
/// Mycelix hApps can persist/reference it; Sol Atlas can render it. None of
/// those consumers are allowed to infer execution authority from its presence.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EnvironmentalObservation {
    pub schema_version: u16,
    pub id: String,
    pub phenomenon: String,
    pub class: EvidenceClass,
    pub measurement: Measurement,
    pub spatial: SpatialExtent,
    pub temporal: TemporalExtent,
    pub uncertainty: Uncertainty,
    pub evidence: Vec<ExternalEvidenceRef>,
    pub epistemic: EpistemicClassification,
}

impl EnvironmentalObservation {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: impl Into<String>,
        phenomenon: impl Into<String>,
        class: EvidenceClass,
        measurement: Measurement,
        spatial: SpatialExtent,
        temporal: TemporalExtent,
        uncertainty: Uncertainty,
        evidence: Vec<ExternalEvidenceRef>,
        epistemic: EpistemicClassification,
    ) -> Result<Self, PlanetaryEvidenceError> {
        let observation = Self {
            schema_version: PLANETARY_EVIDENCE_SCHEMA_VERSION,
            id: id.into(),
            phenomenon: phenomenon.into(),
            class,
            measurement,
            spatial,
            temporal,
            uncertainty,
            evidence,
            epistemic,
        };
        observation.validate()?;
        Ok(observation)
    }

    pub fn validate(&self) -> Result<(), PlanetaryEvidenceError> {
        if self.schema_version != PLANETARY_EVIDENCE_SCHEMA_VERSION {
            return Err(PlanetaryEvidenceError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text("observation.id", &self.id, MAX_ID_BYTES)?;
        require_text("observation.phenomenon", &self.phenomenon, MAX_ID_BYTES)?;
        self.measurement.validate()?;
        self.spatial.validate()?;
        self.temporal.validate()?;
        self.uncertainty.validate()?;

        if self.evidence.is_empty() {
            return Err(PlanetaryEvidenceError::MissingEvidence);
        }
        if self.evidence.len() > MAX_EVIDENCE_REFS {
            return Err(PlanetaryEvidenceError::TooManyEvidenceRefs {
                actual: self.evidence.len(),
                max: MAX_EVIDENCE_REFS,
            });
        }
        for evidence in &self.evidence {
            evidence.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PlanetaryEvidenceError {
    EmptyField(&'static str),
    FieldTooLong {
        field: &'static str,
        actual: usize,
        max: usize,
    },
    NonFinite(&'static str),
    OutOfRange {
        field: &'static str,
        min: f64,
        max: f64,
        actual: f64,
    },
    InvalidInterval(&'static str),
    MalformedField {
        field: &'static str,
        reason: &'static str,
    },
    MissingEvidence,
    TooManyEvidenceRefs {
        actual: usize,
        max: usize,
    },
    UnsupportedSchemaVersion(u16),
}

impl fmt::Display for PlanetaryEvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::NonFinite(field) => write!(f, "{field} must be finite"),
            Self::OutOfRange {
                field,
                min,
                max,
                actual,
            } => write!(f, "{field} must be in [{min}, {max}], got {actual}"),
            Self::InvalidInterval(reason) => write!(f, "invalid interval: {reason}"),
            Self::MalformedField { field, reason } => write!(f, "malformed {field}: {reason}"),
            Self::MissingEvidence => write!(f, "environmental observation requires evidence"),
            Self::TooManyEvidenceRefs { actual, max } => {
                write!(f, "environmental observation has {actual} evidence refs; maximum is {max}")
            }
            Self::UnsupportedSchemaVersion(version) => {
                write!(f, "unsupported planetary evidence schema version {version}")
            }
        }
    }
}

impl std::error::Error for PlanetaryEvidenceError {}

fn require_finite(field: &'static str, value: f64) -> Result<(), PlanetaryEvidenceError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(PlanetaryEvidenceError::NonFinite(field))
    }
}

fn require_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), PlanetaryEvidenceError> {
    if value.trim().is_empty() {
        return Err(PlanetaryEvidenceError::EmptyField(field));
    }
    if value.len() > max {
        return Err(PlanetaryEvidenceError::FieldTooLong {
            field,
            actual: value.len(),
            max,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EmpiricalLevel, MaterialityLevel, NormativeLevel};

    fn evidence() -> ExternalEvidenceRef {
        ExternalEvidenceRef {
            source_system: "sensor-things:station-1".into(),
            resource_id: "observation/123".into(),
            content_digest: Some("sha256:deadbeef".into()),
            retrieved_at: Some(1_788_825_600),
            license: Some("CC-BY-4.0".into()),
        }
    }

    fn epistemic() -> EpistemicClassification {
        EpistemicClassification::new(
            EmpiricalLevel::Measurable,
            NormativeLevel::Network,
            MaterialityLevel::Temporary,
        )
    }

    fn valid_observation() -> EnvironmentalObservation {
        EnvironmentalObservation::new(
            "obs:jhb:temperature:1",
            "air_temperature",
            EvidenceClass::Observed,
            Measurement::new(43.2, "Cel").unwrap(),
            SpatialExtent::Point(GeoPoint::new(-26.2041, 28.0473).unwrap()),
            TemporalExtent::instant(1_788_825_600),
            Uncertainty::Interval {
                lower: 42.8,
                upper: 43.6,
                confidence: Some(0.95),
            },
            vec![evidence()],
            epistemic(),
        )
        .unwrap()
    }

    #[test]
    fn valid_observation_passes_validation() {
        valid_observation().validate().unwrap();
    }

    #[test]
    fn measurement_rejects_nan_and_empty_unit() {
        assert!(matches!(
            Measurement::new(f64::NAN, "Cel"),
            Err(PlanetaryEvidenceError::NonFinite("measurement.value"))
        ));
        assert!(matches!(
            Measurement::new(1.0, "  "),
            Err(PlanetaryEvidenceError::EmptyField("measurement.unit"))
        ));
    }

    #[test]
    fn point_rejects_invalid_coordinates() {
        assert!(GeoPoint::new(91.0, 0.0).is_err());
        assert!(GeoPoint::new(0.0, f64::INFINITY).is_err());
    }

    #[test]
    fn bounding_box_accepts_antimeridian_crossing() {
        let extent = SpatialExtent::BoundingBox {
            south: -10.0,
            west: 170.0,
            north: 10.0,
            east: -170.0,
        };
        extent.validate().unwrap();
    }

    #[test]
    fn temporal_extent_rejects_inversion() {
        assert!(TemporalExtent::new(2, 1).is_err());
    }

    #[test]
    fn uncertainty_rejects_invalid_bounds_and_confidence() {
        assert!(
            Uncertainty::Interval {
                lower: 2.0,
                upper: 1.0,
                confidence: Some(0.95),
            }
            .validate()
            .is_err()
        );
        assert!(
            Uncertainty::Interval {
                lower: 1.0,
                upper: 2.0,
                confidence: Some(1.1),
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn evidence_digest_must_name_algorithm() {
        let mut reference = evidence();
        reference.content_digest = Some("deadbeef".into());
        assert!(matches!(
            reference.validate(),
            Err(PlanetaryEvidenceError::MalformedField {
                field: "evidence.content_digest",
                ..
            })
        ));
    }

    #[test]
    fn observation_requires_evidence() {
        let mut observation = valid_observation();
        observation.evidence.clear();
        assert!(matches!(
            observation.validate(),
            Err(PlanetaryEvidenceError::MissingEvidence)
        ));
    }

    #[test]
    fn evidence_class_is_not_physical_risk() {
        let observation = valid_observation();
        assert_eq!(observation.class, EvidenceClass::Observed);
        // The epistemic classifier remains a separate field; no physical-risk
        // value exists in this payload contract by design.
        assert_eq!(observation.epistemic.empirical, EmpiricalLevel::Measurable);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_observation() {
        let observation = valid_observation();
        let encoded = serde_json::to_string(&observation).unwrap();
        let decoded: EnvironmentalObservation = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, observation);
        decoded.validate().unwrap();
    }
}

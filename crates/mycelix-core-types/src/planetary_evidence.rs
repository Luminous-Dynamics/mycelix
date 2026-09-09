// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-light planetary evidence primitives.
//!
//! These types are the Mycelix payload contract for environmental facts and
//! model products. They deliberately do not define an AI/world-interface
//! envelope (Symthaea already owns that boundary), nor do they define physical
//! risk scores or execution authority.

use crate::EpistemicClassification;
use std::{collections::HashSet, fmt};

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
/// This is intentionally orthogonal to [`EpistemicClassification`].
/// `EvidenceClass` answers *what kind of product is this?* while E/N/M/H is a
/// contextual interpretation that may be attached later when a use-case
/// actually supplies normative/material meaning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum EvidenceClass {
    /// A human, community, organization, or external authority reported it.
    Reported,
    /// Directly observed by a sensor or instrument.
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
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Measurement {
    pub value: f64,
    /// Opaque unit identifier. UCUM-compatible identifiers are preferred.
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
        require_range("spatial.latitude", self.latitude, -90.0, 90.0)?;
        require_range("spatial.longitude", self.longitude, -180.0, 180.0)
    }
}

/// Geographic support of one datum.
///
/// Bounding boxes permit `west > east`, allowing antimeridian-crossing regions
/// without a special secondary geometry type.
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
    /// Stable external or Mycelix region identifier when geometry resolves elsewhere.
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
                require_finite("spatial.south", *south)?;
                require_finite("spatial.west", *west)?;
                require_finite("spatial.north", *north)?;
                require_finite("spatial.east", *east)?;
                require_range("spatial.south", *south, -90.0, 90.0)?;
                require_range("spatial.north", *north, -90.0, 90.0)?;
                require_range("spatial.west", *west, -180.0, 180.0)?;
                require_range("spatial.east", *east, -180.0, 180.0)?;
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
    /// Explicitly unknown/not supplied; never interpreted as exact certainty.
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
                    require_range("uncertainty.confidence", *confidence, 0.0, 1.0)?;
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
            let Some((algorithm, value)) = digest.split_once(':') else {
                return Err(PlanetaryEvidenceError::MalformedField {
                    field: "evidence.content_digest",
                    reason: "digest must be algorithm-qualified (for example sha256:<hex>)",
                });
            };
            if algorithm.trim().is_empty() || value.trim().is_empty() {
                return Err(PlanetaryEvidenceError::MalformedField {
                    field: "evidence.content_digest",
                    reason: "digest algorithm and value must both be non-empty",
                });
            }
            if !algorithm
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-' | b'+'))
            {
                return Err(PlanetaryEvidenceError::MalformedField {
                    field: "evidence.content_digest",
                    reason: "digest algorithm contains unsupported characters",
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
/// This remains a payload rather than an authority-bearing action. The
/// contextual E/N/M/H classification is optional because normative scope and
/// materiality are properties of a claim *in context*, not intrinsic sensor
/// metadata. Consumers that use an observation for governance, risk, or a
/// high-consequence decision should attach an explicit classification.
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
    pub epistemic: Option<EpistemicClassification>,
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
            epistemic: None,
        };
        observation.validate()?;
        Ok(observation)
    }

    /// Attach contextual E/N/M/H semantics without changing the evidence payload.
    pub fn with_epistemic(mut self, epistemic: EpistemicClassification) -> Self {
        self.epistemic = Some(epistemic);
        self
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

        let mut unique = HashSet::with_capacity(self.evidence.len());
        for evidence in &self.evidence {
            evidence.validate()?;
            if !unique.insert(evidence) {
                return Err(PlanetaryEvidenceError::DuplicateEvidenceRef);
            }
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
    DuplicateEvidenceRef,
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
            Self::DuplicateEvidenceRef => write!(f, "environmental observation contains duplicate evidence references"),
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

fn require_range(
    field: &'static str,
    value: f64,
    min: f64,
    max: f64,
) -> Result<(), PlanetaryEvidenceError> {
    if (min..=max).contains(&value) {
        Ok(())
    } else {
        Err(PlanetaryEvidenceError::OutOfRange {
            field,
            min,
            max,
            actual: value,
        })
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
        )
        .unwrap()
    }

    #[test]
    fn valid_observation_passes_without_contextual_epistemics() {
        let observation = valid_observation();
        observation.validate().unwrap();
        assert!(observation.epistemic.is_none());
    }

    #[test]
    fn contextual_epistemics_can_be_attached_explicitly() {
        let observation = valid_observation().with_epistemic(epistemic());
        assert_eq!(
            observation.epistemic.as_ref().unwrap().empirical,
            EmpiricalLevel::Measurable
        );
    }

    #[test]
    fn reported_and_observed_remain_distinct() {
        assert_ne!(EvidenceClass::Reported, EvidenceClass::Observed);
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
        SpatialExtent::BoundingBox {
            south: -10.0,
            west: 170.0,
            north: 10.0,
            east: -170.0,
        }
        .validate()
        .unwrap();
    }

    #[test]
    fn temporal_extent_rejects_inversion() {
        assert!(TemporalExtent::new(2, 1).is_err());
    }

    #[test]
    fn uncertainty_rejects_invalid_bounds_and_confidence() {
        assert!(Uncertainty::Interval {
            lower: 2.0,
            upper: 1.0,
            confidence: Some(0.95),
        }
        .validate()
        .is_err());
        assert!(Uncertainty::Interval {
            lower: 1.0,
            upper: 2.0,
            confidence: Some(1.1),
        }
        .validate()
        .is_err());
    }

    #[test]
    fn evidence_digest_requires_non_empty_algorithm_and_value() {
        for malformed in ["deadbeef", ":deadbeef", "sha256:"] {
            let mut reference = evidence();
            reference.content_digest = Some(malformed.into());
            assert!(matches!(
                reference.validate(),
                Err(PlanetaryEvidenceError::MalformedField {
                    field: "evidence.content_digest",
                    ..
                })
            ));
        }
    }

    #[test]
    fn observation_requires_unique_evidence() {
        let mut observation = valid_observation();
        observation.evidence.push(evidence());
        assert!(matches!(
            observation.validate(),
            Err(PlanetaryEvidenceError::DuplicateEvidenceRef)
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

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_observation() {
        let observation = valid_observation().with_epistemic(epistemic());
        let encoded = serde_json::to_string(&observation).unwrap();
        let decoded: EnvironmentalObservation = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, observation);
        decoded.validate().unwrap();
    }
}

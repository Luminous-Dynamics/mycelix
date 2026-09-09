// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Binding between a planetary observation and the lineage that produced it.

use crate::{
    EnvironmentalObservation, EvidenceClass, EvidenceLineage, PlanetaryEvidenceError,
    PlanetaryLineageError,
};
use std::fmt;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A non-raw planetary observation together with its validated provenance DAG.
///
/// This wrapper prevents two subtle provenance failures:
///
/// 1. attaching a valid lineage to a *different* output observation; and
/// 2. labeling a computed product as `Observed` or `Reported` even though a
///    transformation/model pipeline produced it.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LineagedObservation {
    pub observation: EnvironmentalObservation,
    pub lineage: EvidenceLineage,
}

impl LineagedObservation {
    pub fn new(
        observation: EnvironmentalObservation,
        lineage: EvidenceLineage,
    ) -> Result<Self, LineagedObservationError> {
        let product = Self {
            observation,
            lineage,
        };
        product.validate()?;
        Ok(product)
    }

    pub fn validate(&self) -> Result<(), LineagedObservationError> {
        self.observation
            .validate()
            .map_err(LineagedObservationError::InvalidObservation)?;
        self.lineage
            .validate()
            .map_err(LineagedObservationError::InvalidLineage)?;

        if self.observation.id != self.lineage.output_observation_id {
            return Err(LineagedObservationError::OutputIdMismatch {
                observation_id: self.observation.id.clone(),
                lineage_output_id: self.lineage.output_observation_id.clone(),
            });
        }

        if matches!(
            self.observation.class,
            EvidenceClass::Observed | EvidenceClass::Reported
        ) {
            return Err(LineagedObservationError::RawClassHasComputation(
                self.observation.class,
            ));
        }

        Ok(())
    }
}

#[derive(Debug)]
pub enum LineagedObservationError {
    InvalidObservation(PlanetaryEvidenceError),
    InvalidLineage(PlanetaryLineageError),
    OutputIdMismatch {
        observation_id: String,
        lineage_output_id: String,
    },
    RawClassHasComputation(EvidenceClass),
}

impl fmt::Display for LineagedObservationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidObservation(error) => write!(f, "invalid output observation: {error}"),
            Self::InvalidLineage(error) => write!(f, "invalid output lineage: {error}"),
            Self::OutputIdMismatch {
                observation_id,
                lineage_output_id,
            } => write!(
                f,
                "lineage output {lineage_output_id} does not match observation {observation_id}"
            ),
            Self::RawClassHasComputation(class) => write!(
                f,
                "computed evidence product cannot retain raw evidence class {class:?}"
            ),
        }
    }
}

impl std::error::Error for LineagedObservationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidObservation(error) => Some(error),
            Self::InvalidLineage(error) => Some(error),
            Self::OutputIdMismatch { .. } | Self::RawClassHasComputation(_) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ExternalEvidenceRef, GeoPoint, LineageRef, LineageRoot, LineageSource, LineageStep,
        Measurement, ProducerIdentity, ProducerKind, SpatialExtent, TemporalExtent, Uncertainty,
    };

    fn observation(id: &str, class: EvidenceClass) -> EnvironmentalObservation {
        EnvironmentalObservation::new(
            id,
            "air_temperature",
            class,
            Some(Measurement::new(20.0, "Cel").unwrap()),
            SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            TemporalExtent::instant(1_788_825_600),
            Uncertainty::Unspecified,
            vec![ExternalEvidenceRef {
                source_system: "example".into(),
                resource_id: "source/1".into(),
                content_digest: Some("sha256:source".into()),
                retrieved_at: None,
                license: None,
            }],
        )
        .unwrap()
    }

    fn lineage(output_id: &str) -> EvidenceLineage {
        EvidenceLineage::new(
            output_id,
            vec![LineageRoot {
                id: "raw".into(),
                source: LineageSource::Observation("obs:raw:temperature".into()),
            }],
            vec![LineageStep {
                id: "convert".into(),
                operation: "unit_conversion".into(),
                producer: ProducerIdentity {
                    kind: ProducerKind::DeterministicTransform,
                    implementation: "mycelix://units".into(),
                    version: Some("1".into()),
                    code_digest: Some("sha256:code".into()),
                    configuration_digest: Some("sha256:config".into()),
                    environment_digest: Some("sha256:env".into()),
                },
                inputs: vec![LineageRef::Root("raw".into())],
                output_digest: None,
                completed_at: None,
            }],
            "convert",
        )
        .unwrap()
    }

    #[test]
    fn binds_exact_derived_output() {
        let product = LineagedObservation::new(
            observation("obs:derived:temperature", EvidenceClass::Derived),
            lineage("obs:derived:temperature"),
        )
        .unwrap();
        product.validate().unwrap();
    }

    #[test]
    fn rejects_lineage_for_different_output() {
        let result = LineagedObservation::new(
            observation("obs:derived:a", EvidenceClass::Derived),
            lineage("obs:derived:b"),
        );
        assert!(matches!(
            result,
            Err(LineagedObservationError::OutputIdMismatch { .. })
        ));
    }

    #[test]
    fn rejects_computation_laundered_as_observed() {
        let result = LineagedObservation::new(
            observation("obs:computed", EvidenceClass::Observed),
            lineage("obs:computed"),
        );
        assert!(matches!(
            result,
            Err(LineagedObservationError::RawClassHasComputation(
                EvidenceClass::Observed
            ))
        ));
    }

    #[test]
    fn rejects_computation_laundered_as_reported() {
        let result = LineagedObservation::new(
            observation("obs:computed", EvidenceClass::Reported),
            lineage("obs:computed"),
        );
        assert!(matches!(
            result,
            Err(LineagedObservationError::RawClassHasComputation(
                EvidenceClass::Reported
            ))
        ));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Decomposed physical-risk composition for planetary evidence.
//!
//! This module deliberately does **not** define `risk = 0.82` as a standalone
//! truth primitive. Hazard, exposure, vulnerability, adaptive capacity, and the
//! final risk estimate are evidence-bearing observations in their own right.
//! This contract records how those observations are composed into one risk
//! assessment while preserving their individual provenance, uncertainty, and
//! epistemic class.
//!
//! A consumer can therefore answer not only "what is the risk?" but also:
//! "which hazard forecast, which exposed population/assets, which vulnerability
//! evidence, and which adaptive-capacity evidence produced this assessment?"

use crate::{EvidenceClass, ExternalEvidenceRef, SpatialExtent, TemporalExtent};
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const PHYSICAL_RISK_SCHEMA_VERSION: u16 = 1;
pub const MAX_RISK_ID_BYTES: usize = 256;
pub const MAX_RISK_LABEL_BYTES: usize = 256;
pub const MAX_RISK_INPUTS: usize = 128;
pub const MAX_RISK_ASSUMPTIONS: usize = 64;

/// How a risk assessment is intended to relate to reality.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum RiskAssessmentMode {
    /// Assessment intended to describe/forecast real-world risk. Scenario
    /// inputs and scenario outputs are forbidden.
    Operational,
    /// Counterfactual/planning assessment. The output must remain explicitly
    /// `EvidenceClass::Scenario` so it cannot be laundered into operational risk.
    Scenario,
}

/// Role played by one evidence-bearing input observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum RiskEvidenceRole {
    Hazard,
    Exposure,
    Vulnerability,
    AdaptiveCapacity,
    Context,
}

/// Reference to a canonical environmental observation used by the risk model.
///
/// `expected_class` is an assertion that must be checked when the observation is
/// resolved. Carrying it in the risk record makes accidental or malicious class
/// substitution visible (for example replacing an Observed exposure layer with a
/// Scenario fixture under the same application-level role).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RiskInputRef {
    pub observation_id: String,
    pub role: RiskEvidenceRole,
    pub expected_class: EvidenceClass,
}

impl RiskInputRef {
    pub fn validate(&self) -> Result<(), PhysicalRiskError> {
        require_text(
            "risk.input.observation_id",
            &self.observation_id,
            MAX_RISK_ID_BYTES,
        )
    }
}

/// Canonical risk-estimate observation produced by the assessment.
///
/// The actual numeric/qualitative estimate, units, uncertainty and spatial/time
/// support live in the referenced `EnvironmentalObservation`. Its PEF-2
/// `EvidenceLineage` should use the same output observation ID.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RiskOutputRef {
    pub observation_id: String,
    pub expected_class: EvidenceClass,
}

impl RiskOutputRef {
    pub fn validate(&self) -> Result<(), PhysicalRiskError> {
        require_text(
            "risk.output.observation_id",
            &self.observation_id,
            MAX_RISK_ID_BYTES,
        )?;
        if matches!(
            self.expected_class,
            EvidenceClass::Observed | EvidenceClass::Reported
        ) {
            return Err(PhysicalRiskError::RawRiskOutputClass(
                self.expected_class,
            ));
        }
        Ok(())
    }
}

/// Evidence-composed physical risk assessment.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PhysicalRiskAssessment {
    pub schema_version: u16,
    pub id: String,
    /// Hazard family, e.g. `extreme_heat`, `river_flood`, `wildfire`.
    pub hazard: String,
    /// Consequence being assessed, e.g. `heat_mortality`, `grid_failure`,
    /// `crop_loss`, `habitat_loss`.
    pub impact: String,
    pub mode: RiskAssessmentMode,
    pub spatial: SpatialExtent,
    pub temporal: TemporalExtent,
    pub inputs: Vec<RiskInputRef>,
    pub output: RiskOutputRef,
    /// Immutable external assumption/method documents. Every assumption must
    /// carry a content digest; a mutable URL alone is not a stable assumption.
    pub assumptions: Vec<ExternalEvidenceRef>,
}

impl PhysicalRiskAssessment {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: impl Into<String>,
        hazard: impl Into<String>,
        impact: impl Into<String>,
        mode: RiskAssessmentMode,
        spatial: SpatialExtent,
        temporal: TemporalExtent,
        inputs: Vec<RiskInputRef>,
        output: RiskOutputRef,
    ) -> Result<Self, PhysicalRiskError> {
        let assessment = Self {
            schema_version: PHYSICAL_RISK_SCHEMA_VERSION,
            id: id.into(),
            hazard: hazard.into(),
            impact: impact.into(),
            mode,
            spatial,
            temporal,
            inputs,
            output,
            assumptions: Vec::new(),
        };
        assessment.validate()?;
        Ok(assessment)
    }

    pub fn validate(&self) -> Result<(), PhysicalRiskError> {
        if self.schema_version != PHYSICAL_RISK_SCHEMA_VERSION {
            return Err(PhysicalRiskError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text("risk.id", &self.id, MAX_RISK_ID_BYTES)?;
        require_text("risk.hazard", &self.hazard, MAX_RISK_LABEL_BYTES)?;
        require_text("risk.impact", &self.impact, MAX_RISK_LABEL_BYTES)?;
        self.spatial
            .validate()
            .map_err(|error| PhysicalRiskError::InvalidSpatial(error.to_string()))?;
        self.temporal
            .validate()
            .map_err(|error| PhysicalRiskError::InvalidTemporal(error.to_string()))?;

        if self.inputs.is_empty() {
            return Err(PhysicalRiskError::MissingInputs);
        }
        if self.inputs.len() > MAX_RISK_INPUTS {
            return Err(PhysicalRiskError::TooManyInputs {
                actual: self.inputs.len(),
                max: MAX_RISK_INPUTS,
            });
        }

        let mut ids = HashSet::with_capacity(self.inputs.len());
        let mut has_hazard = false;
        let mut has_exposure = false;
        let mut has_vulnerability = false;

        for input in &self.inputs {
            input.validate()?;
            if !ids.insert(input.observation_id.as_str()) {
                return Err(PhysicalRiskError::DuplicateInputObservation(
                    input.observation_id.clone(),
                ));
            }
            match input.role {
                RiskEvidenceRole::Hazard => has_hazard = true,
                RiskEvidenceRole::Exposure => has_exposure = true,
                RiskEvidenceRole::Vulnerability => has_vulnerability = true,
                RiskEvidenceRole::AdaptiveCapacity | RiskEvidenceRole::Context => {}
            }

            if self.mode == RiskAssessmentMode::Operational
                && input.expected_class == EvidenceClass::Scenario
            {
                return Err(PhysicalRiskError::ScenarioInputInOperationalAssessment(
                    input.observation_id.clone(),
                ));
            }
        }

        if !has_hazard {
            return Err(PhysicalRiskError::MissingRequiredRole(
                RiskEvidenceRole::Hazard,
            ));
        }
        if !has_exposure {
            return Err(PhysicalRiskError::MissingRequiredRole(
                RiskEvidenceRole::Exposure,
            ));
        }
        if !has_vulnerability {
            return Err(PhysicalRiskError::MissingRequiredRole(
                RiskEvidenceRole::Vulnerability,
            ));
        }

        self.output.validate()?;
        if ids.contains(self.output.observation_id.as_str()) {
            return Err(PhysicalRiskError::OutputAlsoUsedAsInput(
                self.output.observation_id.clone(),
            ));
        }

        match self.mode {
            RiskAssessmentMode::Operational => {
                if self.output.expected_class == EvidenceClass::Scenario {
                    return Err(PhysicalRiskError::ScenarioOutputInOperationalAssessment);
                }
            }
            RiskAssessmentMode::Scenario => {
                if self.output.expected_class != EvidenceClass::Scenario {
                    return Err(PhysicalRiskError::ScenarioAssessmentNeedsScenarioOutput(
                        self.output.expected_class,
                    ));
                }
            }
        }

        if self.assumptions.len() > MAX_RISK_ASSUMPTIONS {
            return Err(PhysicalRiskError::TooManyAssumptions {
                actual: self.assumptions.len(),
                max: MAX_RISK_ASSUMPTIONS,
            });
        }
        let mut assumptions = HashSet::with_capacity(self.assumptions.len());
        for assumption in &self.assumptions {
            assumption
                .validate()
                .map_err(|error| PhysicalRiskError::InvalidAssumption(error.to_string()))?;
            if assumption.content_digest.is_none() {
                return Err(PhysicalRiskError::MutableAssumptionReference(
                    assumption.resource_id.clone(),
                ));
            }
            if !assumptions.insert(assumption) {
                return Err(PhysicalRiskError::DuplicateAssumption);
            }
        }

        Ok(())
    }

    pub fn inputs_for(&self, role: RiskEvidenceRole) -> impl Iterator<Item = &RiskInputRef> {
        self.inputs.iter().filter(move |input| input.role == role)
    }

    pub fn has_adaptive_capacity_evidence(&self) -> bool {
        self.inputs
            .iter()
            .any(|input| input.role == RiskEvidenceRole::AdaptiveCapacity)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PhysicalRiskError {
    UnsupportedSchemaVersion(u16),
    EmptyField(&'static str),
    FieldTooLong {
        field: &'static str,
        actual: usize,
        max: usize,
    },
    InvalidSpatial(String),
    InvalidTemporal(String),
    MissingInputs,
    TooManyInputs {
        actual: usize,
        max: usize,
    },
    MissingRequiredRole(RiskEvidenceRole),
    DuplicateInputObservation(String),
    ScenarioInputInOperationalAssessment(String),
    RawRiskOutputClass(EvidenceClass),
    OutputAlsoUsedAsInput(String),
    ScenarioOutputInOperationalAssessment,
    ScenarioAssessmentNeedsScenarioOutput(EvidenceClass),
    TooManyAssumptions {
        actual: usize,
        max: usize,
    },
    InvalidAssumption(String),
    MutableAssumptionReference(String),
    DuplicateAssumption,
}

impl fmt::Display for PhysicalRiskError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(version) => {
                write!(f, "unsupported physical-risk schema version {version}")
            }
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::InvalidSpatial(error) => write!(f, "invalid risk spatial extent: {error}"),
            Self::InvalidTemporal(error) => write!(f, "invalid risk temporal extent: {error}"),
            Self::MissingInputs => write!(f, "physical-risk assessment requires inputs"),
            Self::TooManyInputs { actual, max } => {
                write!(f, "physical-risk assessment has {actual} inputs; maximum is {max}")
            }
            Self::MissingRequiredRole(role) => {
                write!(f, "physical-risk assessment is missing required {role:?} evidence")
            }
            Self::DuplicateInputObservation(id) => {
                write!(f, "risk input observation {id} is referenced more than once")
            }
            Self::ScenarioInputInOperationalAssessment(id) => write!(
                f,
                "operational physical-risk assessment cannot consume scenario input {id}"
            ),
            Self::RawRiskOutputClass(class) => write!(
                f,
                "physical-risk output cannot claim raw evidence class {class:?}"
            ),
            Self::OutputAlsoUsedAsInput(id) => {
                write!(f, "physical-risk output {id} cannot also be an input")
            }
            Self::ScenarioOutputInOperationalAssessment => write!(
                f,
                "operational physical-risk assessment cannot produce a scenario output"
            ),
            Self::ScenarioAssessmentNeedsScenarioOutput(class) => write!(
                f,
                "scenario physical-risk assessment must retain Scenario output class, got {class:?}"
            ),
            Self::TooManyAssumptions { actual, max } => write!(
                f,
                "physical-risk assessment has {actual} assumptions; maximum is {max}"
            ),
            Self::InvalidAssumption(error) => write!(f, "invalid risk assumption: {error}"),
            Self::MutableAssumptionReference(resource) => write!(
                f,
                "risk assumption {resource} must be content-digest bound"
            ),
            Self::DuplicateAssumption => write!(f, "physical-risk assessment has duplicate assumptions"),
        }
    }
}

impl std::error::Error for PhysicalRiskError {}

fn require_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), PhysicalRiskError> {
    if value.trim().is_empty() {
        return Err(PhysicalRiskError::EmptyField(field));
    }
    if value.len() > max {
        return Err(PhysicalRiskError::FieldTooLong {
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
    use crate::GeoPoint;

    fn input(id: &str, role: RiskEvidenceRole, class: EvidenceClass) -> RiskInputRef {
        RiskInputRef {
            observation_id: id.into(),
            role,
            expected_class: class,
        }
    }

    fn operational() -> PhysicalRiskAssessment {
        PhysicalRiskAssessment::new(
            "risk:jhb:heat:2026-09-10",
            "extreme_heat",
            "heat_mortality",
            RiskAssessmentMode::Operational,
            SpatialExtent::Point(GeoPoint::new(-26.2041, 28.0473).unwrap()),
            TemporalExtent::new(1_788_912_000, 1_788_933_600).unwrap(),
            vec![
                input("obs:heat:forecast", RiskEvidenceRole::Hazard, EvidenceClass::Forecast),
                input("obs:population", RiskEvidenceRole::Exposure, EvidenceClass::Observed),
                input(
                    "obs:vulnerability",
                    RiskEvidenceRole::Vulnerability,
                    EvidenceClass::Derived,
                ),
                input(
                    "obs:cooling-centers",
                    RiskEvidenceRole::AdaptiveCapacity,
                    EvidenceClass::Reported,
                ),
            ],
            RiskOutputRef {
                observation_id: "obs:risk:heat-mortality".into(),
                expected_class: EvidenceClass::Inferred,
            },
        )
        .unwrap()
    }

    #[test]
    fn operational_risk_is_decomposed_not_scalar() {
        let assessment = operational();
        assessment.validate().unwrap();
        assert_eq!(assessment.inputs_for(RiskEvidenceRole::Hazard).count(), 1);
        assert_eq!(assessment.inputs_for(RiskEvidenceRole::Exposure).count(), 1);
        assert_eq!(assessment.inputs_for(RiskEvidenceRole::Vulnerability).count(), 1);
        assert!(assessment.has_adaptive_capacity_evidence());
    }

    #[test]
    fn requires_hazard_exposure_and_vulnerability() {
        let mut assessment = operational();
        assessment
            .inputs
            .retain(|input| input.role != RiskEvidenceRole::Exposure);
        assert_eq!(
            assessment.validate(),
            Err(PhysicalRiskError::MissingRequiredRole(
                RiskEvidenceRole::Exposure
            ))
        );
    }

    #[test]
    fn rejects_duplicate_input_observation() {
        let mut assessment = operational();
        assessment.inputs.push(input(
            "obs:population",
            RiskEvidenceRole::Context,
            EvidenceClass::Observed,
        ));
        assert!(matches!(
            assessment.validate(),
            Err(PhysicalRiskError::DuplicateInputObservation(_))
        ));
    }

    #[test]
    fn operational_assessment_rejects_scenario_input() {
        let mut assessment = operational();
        assessment.inputs[0].expected_class = EvidenceClass::Scenario;
        assert!(matches!(
            assessment.validate(),
            Err(PhysicalRiskError::ScenarioInputInOperationalAssessment(_))
        ));
    }

    #[test]
    fn raw_observation_cannot_be_the_risk_output() {
        let mut assessment = operational();
        assessment.output.expected_class = EvidenceClass::Observed;
        assert_eq!(
            assessment.validate(),
            Err(PhysicalRiskError::RawRiskOutputClass(EvidenceClass::Observed))
        );
    }

    #[test]
    fn scenario_assessment_must_remain_scenario() {
        let mut assessment = operational();
        assessment.mode = RiskAssessmentMode::Scenario;
        assessment.output.expected_class = EvidenceClass::Scenario;
        assessment.inputs[0].expected_class = EvidenceClass::Scenario;
        assessment.validate().unwrap();

        assessment.output.expected_class = EvidenceClass::Inferred;
        assert_eq!(
            assessment.validate(),
            Err(PhysicalRiskError::ScenarioAssessmentNeedsScenarioOutput(
                EvidenceClass::Inferred
            ))
        );
    }

    #[test]
    fn assumptions_must_be_immutable_content_refs() {
        let mut assessment = operational();
        assessment.assumptions.push(ExternalEvidenceRef {
            source_system: "policy".into(),
            resource_id: "heat-thresholds/latest".into(),
            content_digest: None,
            retrieved_at: None,
            license: None,
        });
        assert!(matches!(
            assessment.validate(),
            Err(PhysicalRiskError::MutableAssumptionReference(_))
        ));

        assessment.assumptions[0].content_digest = Some("sha256:thresholds".into());
        assessment.validate().unwrap();
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_risk_roles() {
        let assessment = operational();
        let encoded = serde_json::to_string(&assessment).unwrap();
        let decoded: PhysicalRiskAssessment = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, assessment);
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-light structural semantics for scientific evidence artifacts.
//!
//! These types describe what an artifact *is* or what role it plays in a
//! research workflow. They deliberately do not encode whether an artifact
//! supports, refutes, confirms, falsifies, or otherwise changes belief in a
//! scientific claim. Evidentiary interpretation belongs in attestations and
//! derived analysis, not in artifact-role metadata.

use crate::{Error, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const MAX_EVIDENCE_ARTIFACT_ROLES: usize = 32;
const MAX_OTHER_ROLE_BYTES: usize = 128;

/// Structural or functional role of an artifact in a research workflow.
///
/// This vocabulary is intentionally outcome-neutral. For example,
/// `StatisticalOutput` says what the artifact contains, not whether the
/// statistical result supports a claim. Likewise `SimulationOutput` does not
/// imply empirical observation.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", try_from = "EvidenceArtifactRoleWire")]
pub enum EvidenceArtifactRole {
    RawObservation,
    RawDataset,
    ProcessedDataset,
    AnalysisCode,
    Workflow,
    RuntimeEnvironment,
    InstrumentConfiguration,
    Protocol,
    Preregistration,
    StatisticalOutput,
    ModelArtifact,
    SimulationOutput,
    Visualization,
    CalibrationRecord,
    LabNotebook,
    SupplementaryMaterial,
    Other(String),
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
enum EvidenceArtifactRoleWire {
    RawObservation,
    RawDataset,
    ProcessedDataset,
    AnalysisCode,
    Workflow,
    RuntimeEnvironment,
    InstrumentConfiguration,
    Protocol,
    Preregistration,
    StatisticalOutput,
    ModelArtifact,
    SimulationOutput,
    Visualization,
    CalibrationRecord,
    LabNotebook,
    SupplementaryMaterial,
    Other(String),
}

impl TryFrom<EvidenceArtifactRoleWire> for EvidenceArtifactRole {
    type Error = Error;

    fn try_from(value: EvidenceArtifactRoleWire) -> Result<Self> {
        let role = match value {
            EvidenceArtifactRoleWire::RawObservation => Self::RawObservation,
            EvidenceArtifactRoleWire::RawDataset => Self::RawDataset,
            EvidenceArtifactRoleWire::ProcessedDataset => Self::ProcessedDataset,
            EvidenceArtifactRoleWire::AnalysisCode => Self::AnalysisCode,
            EvidenceArtifactRoleWire::Workflow => Self::Workflow,
            EvidenceArtifactRoleWire::RuntimeEnvironment => Self::RuntimeEnvironment,
            EvidenceArtifactRoleWire::InstrumentConfiguration => Self::InstrumentConfiguration,
            EvidenceArtifactRoleWire::Protocol => Self::Protocol,
            EvidenceArtifactRoleWire::Preregistration => Self::Preregistration,
            EvidenceArtifactRoleWire::StatisticalOutput => Self::StatisticalOutput,
            EvidenceArtifactRoleWire::ModelArtifact => Self::ModelArtifact,
            EvidenceArtifactRoleWire::SimulationOutput => Self::SimulationOutput,
            EvidenceArtifactRoleWire::Visualization => Self::Visualization,
            EvidenceArtifactRoleWire::CalibrationRecord => Self::CalibrationRecord,
            EvidenceArtifactRoleWire::LabNotebook => Self::LabNotebook,
            EvidenceArtifactRoleWire::SupplementaryMaterial => Self::SupplementaryMaterial,
            EvidenceArtifactRoleWire::Other(value) => Self::Other(value),
        };
        role.validate()?;
        Ok(role)
    }
}

impl EvidenceArtifactRole {
    /// Validate role-local structure without assigning epistemic meaning.
    pub fn validate(&self) -> Result<()> {
        if let Self::Other(value) = self {
            validate_other_role(value)?;
        }
        Ok(())
    }

    /// Every role in this vocabulary is intentionally neutral about whether a
    /// claim is supported, contradicted, null, significant, or inconclusive.
    pub const fn is_outcome_neutral(&self) -> bool {
        true
    }

    /// Whether the role denotes a computationally produced or transformed
    /// artifact. This is descriptive provenance information, not a quality or
    /// evidentiary-strength judgment.
    pub const fn is_computational_product(&self) -> bool {
        matches!(
            self,
            Self::ProcessedDataset
                | Self::StatisticalOutput
                | Self::ModelArtifact
                | Self::SimulationOutput
                | Self::Visualization
        )
    }
}

/// Deterministically ordered role set for one artifact.
///
/// Multiple roles are allowed because real research objects often serve more
/// than one structural function. The set carries no support/refute direction
/// and cannot itself increase claim maturity. Deserialization routes through
/// the same validation used by [`Self::new`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, try_from = "EvidenceArtifactSemanticsWire")]
pub struct EvidenceArtifactSemantics {
    roles: BTreeSet<EvidenceArtifactRole>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvidenceArtifactSemanticsWire {
    roles: BTreeSet<EvidenceArtifactRole>,
}

impl TryFrom<EvidenceArtifactSemanticsWire> for EvidenceArtifactSemantics {
    type Error = Error;

    fn try_from(value: EvidenceArtifactSemanticsWire) -> Result<Self> {
        Self::new(value.roles)
    }
}

impl EvidenceArtifactSemantics {
    pub fn new<I>(roles: I) -> Result<Self>
    where
        I: IntoIterator<Item = EvidenceArtifactRole>,
    {
        let roles: BTreeSet<_> = roles.into_iter().collect();
        let semantics = Self { roles };
        semantics.validate()?;
        Ok(semantics)
    }

    pub fn validate(&self) -> Result<()> {
        if self.roles.is_empty() {
            return Err(Error::Validation(
                "evidence artifact semantics require at least one structural role".to_string(),
            ));
        }
        if self.roles.len() > MAX_EVIDENCE_ARTIFACT_ROLES {
            return Err(Error::Validation(format!(
                "evidence artifact semantics cannot exceed {MAX_EVIDENCE_ARTIFACT_ROLES} roles"
            )));
        }
        for role in &self.roles {
            role.validate()?;
        }
        Ok(())
    }

    pub fn roles(&self) -> impl Iterator<Item = &EvidenceArtifactRole> {
        self.roles.iter()
    }

    pub fn contains(&self, role: &EvidenceArtifactRole) -> bool {
        self.roles.contains(role)
    }

    pub fn len(&self) -> usize {
        self.roles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.roles.is_empty()
    }
}

fn validate_other_role(value: &str) -> Result<()> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return Err(Error::Validation(
            "custom evidence artifact role cannot be empty".to_string(),
        ));
    }
    if trimmed != value {
        return Err(Error::Validation(
            "custom evidence artifact role cannot contain leading or trailing whitespace"
                .to_string(),
        ));
    }
    if value.len() > MAX_OTHER_ROLE_BYTES {
        return Err(Error::Validation(format!(
            "custom evidence artifact role cannot exceed {MAX_OTHER_ROLE_BYTES} bytes"
        )));
    }
    if value.chars().any(char::is_control) {
        return Err(Error::Validation(
            "custom evidence artifact role cannot contain control characters".to_string(),
        ));
    }

    // Keep the forward-compatible escape hatch from becoming an unofficial
    // evidence-outcome field. These exact labels belong in attestations or
    // analysis, not artifact structure.
    let normalized = value
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() {
                ch.to_ascii_lowercase()
            } else {
                '_'
            }
        })
        .collect::<String>()
        .split('_')
        .filter(|part| !part.is_empty())
        .collect::<Vec<_>>()
        .join("_");

    if matches!(
        normalized.as_str(),
        "supports"
            | "supporting"
            | "refutes"
            | "refuting"
            | "confirmed"
            | "falsified"
            | "null_result"
            | "negative_result"
            | "positive_result"
            | "inconclusive"
    ) {
        return Err(Error::Validation(
            "evidence artifact role cannot encode an evidentiary outcome".to_string(),
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roles_are_outcome_neutral_by_contract() {
        let roles = [
            EvidenceArtifactRole::RawObservation,
            EvidenceArtifactRole::RawDataset,
            EvidenceArtifactRole::ProcessedDataset,
            EvidenceArtifactRole::AnalysisCode,
            EvidenceArtifactRole::Workflow,
            EvidenceArtifactRole::RuntimeEnvironment,
            EvidenceArtifactRole::InstrumentConfiguration,
            EvidenceArtifactRole::Protocol,
            EvidenceArtifactRole::Preregistration,
            EvidenceArtifactRole::StatisticalOutput,
            EvidenceArtifactRole::ModelArtifact,
            EvidenceArtifactRole::SimulationOutput,
            EvidenceArtifactRole::Visualization,
            EvidenceArtifactRole::CalibrationRecord,
            EvidenceArtifactRole::LabNotebook,
            EvidenceArtifactRole::SupplementaryMaterial,
        ];
        assert!(roles.iter().all(EvidenceArtifactRole::is_outcome_neutral));
    }

    #[test]
    fn artifact_can_carry_multiple_structural_roles_deterministically() {
        let semantics = EvidenceArtifactSemantics::new([
            EvidenceArtifactRole::Workflow,
            EvidenceArtifactRole::AnalysisCode,
            EvidenceArtifactRole::Workflow,
        ])
        .unwrap();

        assert_eq!(semantics.len(), 2);
        assert!(semantics.contains(&EvidenceArtifactRole::Workflow));
        assert!(semantics.contains(&EvidenceArtifactRole::AnalysisCode));

        let first = serde_json::to_string(&semantics).unwrap();
        let second = serde_json::to_string(&semantics).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn computational_product_is_descriptive_not_evidentiary() {
        assert!(EvidenceArtifactRole::ProcessedDataset.is_computational_product());
        assert!(EvidenceArtifactRole::StatisticalOutput.is_computational_product());
        assert!(EvidenceArtifactRole::SimulationOutput.is_computational_product());
        assert!(!EvidenceArtifactRole::RawObservation.is_computational_product());
        assert!(EvidenceArtifactRole::SimulationOutput.is_outcome_neutral());
    }

    #[test]
    fn custom_role_rejects_outcome_labels() {
        for forbidden in [
            "supports",
            "Refutes",
            "null result",
            "negative-result",
            "positive_result",
            "inconclusive",
        ] {
            assert!(EvidenceArtifactRole::Other(forbidden.to_string())
                .validate()
                .is_err());
        }
    }

    #[test]
    fn custom_role_preserves_legitimate_structural_extension() {
        let role = EvidenceArtifactRole::Other("microscopy_tile_pyramid".to_string());
        assert!(role.validate().is_ok());
        let semantics = EvidenceArtifactSemantics::new([role]).unwrap();
        assert_eq!(semantics.len(), 1);
    }

    #[test]
    fn semantics_require_at_least_one_role() {
        let result = EvidenceArtifactSemantics::new(std::iter::empty());
        assert!(result.is_err());
    }

    #[test]
    fn role_deserialization_cannot_bypass_outcome_fence() {
        assert!(serde_json::from_str::<EvidenceArtifactRole>(r#"{"other":"supports"}"#).is_err());
        assert!(serde_json::from_str::<EvidenceArtifactRole>(r#"{"other":"null result"}"#).is_err());
    }

    #[test]
    fn semantics_deserialization_cannot_create_empty_set() {
        assert!(serde_json::from_str::<EvidenceArtifactSemantics>(r#"{"roles":[]}"#).is_err());
    }

    #[test]
    fn valid_semantics_round_trip_through_serde() {
        let semantics = EvidenceArtifactSemantics::new([
            EvidenceArtifactRole::RawDataset,
            EvidenceArtifactRole::AnalysisCode,
        ])
        .unwrap();
        let json = serde_json::to_string(&semantics).unwrap();
        let round_trip: EvidenceArtifactSemantics = serde_json::from_str(&json).unwrap();
        assert_eq!(round_trip, semantics);
    }
}

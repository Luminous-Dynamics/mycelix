// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! One-way projection from canonical hardware design semantics into the
//! operational Mycelix Manufacturing BOM model.
//!
//! The result is a BOM candidate, not a manufacturing authorization.

#![deny(unsafe_code)]

use hdi::prelude::Timestamp;
use manufacturing_common::{BillOfMaterials, BomItem};
use mycelix_hardware_core::{
    CompositionEntry, CompositionSubject, DesignRevision, DigestRef, QuantityUnit, RevisionState,
    SemanticId,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const MANUFACTURING_PROJECTION_SCHEMA: &str = "mycelix.hardware.manufacturing-projection.v1";
const MAX_ITEMS: usize = 16_384;
const MAX_TEXT: usize = 4096;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum OperationalTarget {
    PartId(String),
    SubAssemblyBomHash(String),
}

impl OperationalTarget {
    fn validate(&self) -> Result<(), ProjectionError> {
        match self {
            Self::PartId(value) => validate_text(value, "operational part id"),
            Self::SubAssemblyBomHash(value) => validate_text(value, "sub-assembly BOM hash"),
        }
    }
}

/// Explicit mapping from one exact design-composition entry to an operational
/// manufacturing target. The optional provenance digest records where the
/// mapping came from; it does not assert that the provenance was verified.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProjectionMapping {
    pub source_entry_id: SemanticId,
    pub target: OperationalTarget,
    pub provenance_digest: Option<DigestRef>,
}

impl ProjectionMapping {
    fn validate(&self) -> Result<(), ProjectionError> {
        self.target.validate()?;
        if let Some(digest) = &self.provenance_digest {
            digest
                .validate()
                .map_err(|error| ProjectionError::InvalidDigest(error.to_string()))?;
        }
        Ok(())
    }
}

/// A requested replacement is carried through the projection boundary but is
/// never accepted by this first exact projector. SHW-003 or another explicit
/// qualification adapter must turn a candidate into a verified scoped mapping.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProposedSubstitution {
    pub source_entry_id: SemanticId,
    pub candidate_subject_id: SemanticId,
    pub qualification_profile: String,
    pub evidence_digest: Option<DigestRef>,
}

impl ProposedSubstitution {
    fn validate(&self) -> Result<(), ProjectionError> {
        validate_text(&self.qualification_profile, "substitution qualification profile")?;
        if let Some(digest) = &self.evidence_digest {
            digest
                .validate()
                .map_err(|error| ProjectionError::InvalidDigest(error.to_string()))?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ManufacturingProjectionRequest {
    pub schema_version: String,
    pub design_revision_id: SemanticId,
    pub production_profile: String,
    /// Intended production lot size. This is operational context only and must
    /// not scale the BOM's per-unit design quantities.
    pub production_quantity: u64,
    pub mappings: Vec<ProjectionMapping>,
    pub proposed_substitutions: Vec<ProposedSubstitution>,
}

impl ManufacturingProjectionRequest {
    pub fn validate(&self) -> Result<(), ProjectionError> {
        if self.schema_version != MANUFACTURING_PROJECTION_SCHEMA {
            return Err(ProjectionError::UnsupportedSchema);
        }
        validate_text(&self.production_profile, "production profile")?;
        if self.production_quantity == 0 || self.production_quantity > 1_000_000 {
            return Err(ProjectionError::InvalidProductionQuantity);
        }
        if self.mappings.len() > MAX_ITEMS || self.proposed_substitutions.len() > MAX_ITEMS {
            return Err(ProjectionError::TooManyItems);
        }
        ensure_unique_mapping_ids(&self.mappings)?;
        ensure_unique_substitution_ids(&self.proposed_substitutions)?;
        for mapping in &self.mappings {
            mapping.validate()?;
        }
        for substitution in &self.proposed_substitutions {
            substitution.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProjectionIssue {
    MissingMapping(SemanticId),
    MappingReferencesUnknownEntry(SemanticId),
    SubstitutionReferencesUnknownEntry(SemanticId),
    TargetTypeMismatch {
        entry_id: SemanticId,
        expected: String,
    },
    SubstitutionRequiresQualification {
        entry_id: SemanticId,
        candidate_subject_id: SemanticId,
        profile: String,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManufacturingProjectionReport {
    pub schema_version: String,
    pub source_revision_id: SemanticId,
    pub source_project_id: SemanticId,
    pub production_profile: String,
    pub production_quantity: u64,
    /// Candidate BOM emitted only when every composition entry has an exact,
    /// type-compatible operational mapping and no substitution is pending.
    pub bom_candidate: Option<BillOfMaterials>,
    pub applied_mappings: Vec<ProjectionMapping>,
    pub issues: Vec<ProjectionIssue>,
}

impl ManufacturingProjectionReport {
    pub fn is_complete_candidate(&self) -> bool {
        self.bom_candidate.is_some() && self.issues.is_empty()
    }
}

pub fn project_design_to_bom_candidate(
    request: &ManufacturingProjectionRequest,
    revision: &DesignRevision,
    created_at: Timestamp,
) -> Result<ManufacturingProjectionReport, ProjectionError> {
    request.validate()?;
    revision
        .validate()
        .map_err(|error| ProjectionError::InvalidRevision(error.to_string()))?;
    if revision.id != request.design_revision_id {
        return Err(ProjectionError::RevisionMismatch);
    }
    if revision.state != RevisionState::Released {
        return Err(ProjectionError::RevisionNotReleased);
    }

    let mut issues = Vec::new();
    let known_entries: BTreeSet<&SemanticId> = revision
        .composition
        .entries
        .iter()
        .map(|entry| &entry.id)
        .collect();

    for mapping in &request.mappings {
        if !known_entries.contains(&mapping.source_entry_id) {
            issues.push(ProjectionIssue::MappingReferencesUnknownEntry(
                mapping.source_entry_id.clone(),
            ));
        }
    }

    for substitution in &request.proposed_substitutions {
        if !known_entries.contains(&substitution.source_entry_id) {
            issues.push(ProjectionIssue::SubstitutionReferencesUnknownEntry(
                substitution.source_entry_id.clone(),
            ));
        } else {
            issues.push(ProjectionIssue::SubstitutionRequiresQualification {
                entry_id: substitution.source_entry_id.clone(),
                candidate_subject_id: substitution.candidate_subject_id.clone(),
                profile: substitution.qualification_profile.clone(),
            });
        }
    }

    let mut items = Vec::new();
    let mut applied_mappings = Vec::new();
    for entry in &revision.composition.entries {
        let Some(mapping) = request
            .mappings
            .iter()
            .find(|mapping| mapping.source_entry_id == entry.id)
        else {
            issues.push(ProjectionIssue::MissingMapping(entry.id.clone()));
            continue;
        };

        match project_entry(entry, mapping) {
            Ok(item) => {
                items.push(item);
                applied_mappings.push(mapping.clone());
            }
            Err(expected) => issues.push(ProjectionIssue::TargetTypeMismatch {
                entry_id: entry.id.clone(),
                expected,
            }),
        }
    }

    let bom_candidate = if issues.is_empty() {
        Some(BillOfMaterials {
            design_id: revision.project_id.to_string(),
            revision: revision.revision_label.clone(),
            items,
            created_at,
        })
    } else {
        None
    };

    Ok(ManufacturingProjectionReport {
        schema_version: MANUFACTURING_PROJECTION_SCHEMA.into(),
        source_revision_id: revision.id.clone(),
        source_project_id: revision.project_id.clone(),
        production_profile: request.production_profile.clone(),
        production_quantity: request.production_quantity,
        bom_candidate,
        applied_mappings,
        issues,
    })
}

fn project_entry(entry: &CompositionEntry, mapping: &ProjectionMapping) -> Result<BomItem, String> {
    let (part_id, sub_assembly_bom_hash) = match (&entry.subject, &mapping.target) {
        (
            CompositionSubject::Component(_) | CompositionSubject::Material(_),
            OperationalTarget::PartId(part_id),
        ) => (part_id.clone(), None),
        (
            CompositionSubject::SubDesign(_),
            OperationalTarget::SubAssemblyBomHash(hash),
        ) => (String::new(), Some(hash.clone())),
        (CompositionSubject::SubDesign(_), _) => {
            return Err("SubAssemblyBomHash for a sub-design".into());
        }
        _ => return Err("PartId for a component or material".into()),
    };

    Ok(BomItem {
        part_id,
        quantity_per: entry.quantity.value,
        unit: quantity_unit_string(&entry.quantity.unit),
        sub_assembly_bom_hash,
        notes: entry.notes.clone(),
    })
}

fn quantity_unit_string(unit: &QuantityUnit) -> String {
    match unit {
        QuantityUnit::Each => "each".into(),
        QuantityUnit::Millimetre => "mm".into(),
        QuantityUnit::Metre => "m".into(),
        QuantityUnit::SquareMillimetre => "mm2".into(),
        QuantityUnit::SquareMetre => "m2".into(),
        QuantityUnit::CubicMillimetre => "mm3".into(),
        QuantityUnit::CubicMetre => "m3".into(),
        QuantityUnit::Gram => "g".into(),
        QuantityUnit::Kilogram => "kg".into(),
        QuantityUnit::Millilitre => "ml".into(),
        QuantityUnit::Litre => "l".into(),
        QuantityUnit::Other(value) => value.clone(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProjectionError {
    UnsupportedSchema,
    EmptyField(&'static str),
    FieldTooLong(&'static str),
    InvalidProductionQuantity,
    TooManyItems,
    DuplicateMapping(SemanticId),
    DuplicateSubstitution(SemanticId),
    InvalidDigest(String),
    InvalidRevision(String),
    RevisionMismatch,
    RevisionNotReleased,
}

impl fmt::Display for ProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchema => write!(f, "unsupported manufacturing projection schema"),
            Self::EmptyField(field) => write!(f, "{field} must not be empty"),
            Self::FieldTooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidProductionQuantity => {
                write!(f, "production quantity must be between 1 and 1,000,000")
            }
            Self::TooManyItems => write!(f, "projection collection exceeds maximum item count"),
            Self::DuplicateMapping(id) => write!(f, "duplicate mapping for {id}"),
            Self::DuplicateSubstitution(id) => write!(f, "duplicate substitution for {id}"),
            Self::InvalidDigest(reason) => write!(f, "invalid digest: {reason}"),
            Self::InvalidRevision(reason) => write!(f, "invalid design revision: {reason}"),
            Self::RevisionMismatch => write!(f, "request does not bind the supplied design revision"),
            Self::RevisionNotReleased => write!(f, "only Released design revisions may be projected"),
        }
    }
}

impl std::error::Error for ProjectionError {}

fn validate_text(value: &str, field: &'static str) -> Result<(), ProjectionError> {
    if value.trim().is_empty() || value != value.trim() || value.chars().any(char::is_control) {
        return Err(ProjectionError::EmptyField(field));
    }
    if value.len() > MAX_TEXT {
        return Err(ProjectionError::FieldTooLong(field));
    }
    Ok(())
}

fn ensure_unique_mapping_ids(values: &[ProjectionMapping]) -> Result<(), ProjectionError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(&value.source_entry_id) {
            return Err(ProjectionError::DuplicateMapping(value.source_entry_id.clone()));
        }
    }
    Ok(())
}

fn ensure_unique_substitution_ids(values: &[ProposedSubstitution]) -> Result<(), ProjectionError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(&value.source_entry_id) {
            return Err(ProjectionError::DuplicateSubstitution(
                value.source_entry_id.clone(),
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_hardware_core::{
        DesignComposition, HardwareRequirement, Quantity, RequirementCriticality,
        VerificationMethod, HARDWARE_SEMANTIC_SCHEMA,
    };

    fn id(value: &str) -> SemanticId {
        SemanticId::new(value).unwrap()
    }

    fn timestamp() -> Timestamp {
        Timestamp::from_micros(1_800_000_000_000_000)
    }

    fn revision() -> DesignRevision {
        DesignRevision {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id("revision:sensor:r1"),
            project_id: id("project:sensor"),
            revision_label: "r1".into(),
            state: RevisionState::Released,
            predecessors: Vec::new(),
            artifacts: Vec::new(),
            composition: DesignComposition {
                entries: vec![CompositionEntry {
                    id: id("entry:mcu"),
                    subject: CompositionSubject::Component(id("component:mcu")),
                    quantity: Quantity {
                        value: 2,
                        unit: QuantityUnit::Each,
                    },
                    designators: vec!["U1".into(), "U2".into()],
                    notes: Some("controller".into()),
                }],
            },
            requirements: vec![HardwareRequirement {
                id: id("requirement:basic"),
                statement: "basic requirement".into(),
                criticality: RequirementCriticality::Low,
                verification_method: VerificationMethod::Inspection,
                source_ref: None,
            }],
            documentation_profiles: Vec::new(),
            evidence_refs: Vec::new(),
        }
    }

    fn request() -> ManufacturingProjectionRequest {
        ManufacturingProjectionRequest {
            schema_version: MANUFACTURING_PROJECTION_SCHEMA.into(),
            design_revision_id: id("revision:sensor:r1"),
            production_profile: "prototype-local-v1".into(),
            production_quantity: 100,
            mappings: vec![ProjectionMapping {
                source_entry_id: id("entry:mcu"),
                target: OperationalTarget::PartId("ERP-MCU-001".into()),
                provenance_digest: None,
            }],
            proposed_substitutions: Vec::new(),
        }
    }

    #[test]
    fn exact_mapping_produces_bom_candidate() {
        let report = project_design_to_bom_candidate(&request(), &revision(), timestamp()).unwrap();
        assert!(report.is_complete_candidate());
        let bom = report.bom_candidate.unwrap();
        assert_eq!(bom.design_id, "project:sensor");
        assert_eq!(bom.revision, "r1");
        assert_eq!(bom.items[0].part_id, "ERP-MCU-001");
    }

    #[test]
    fn production_quantity_does_not_scale_design_quantity() {
        let report = project_design_to_bom_candidate(&request(), &revision(), timestamp()).unwrap();
        let bom = report.bom_candidate.unwrap();
        assert_eq!(report.production_quantity, 100);
        assert_eq!(bom.items[0].quantity_per, 2);
    }

    #[test]
    fn missing_mapping_blocks_candidate() {
        let mut request = request();
        request.mappings.clear();
        let report = project_design_to_bom_candidate(&request, &revision(), timestamp()).unwrap();
        assert!(report.bom_candidate.is_none());
        assert!(matches!(
            report.issues.as_slice(),
            [ProjectionIssue::MissingMapping(id)] if id == &SemanticId::new("entry:mcu").unwrap()
        ));
    }

    #[test]
    fn proposed_substitution_requires_external_qualification() {
        let mut request = request();
        request.proposed_substitutions.push(ProposedSubstitution {
            source_entry_id: id("entry:mcu"),
            candidate_subject_id: id("component:mcu-alt"),
            qualification_profile: "electrical-and-firmware-v1".into(),
            evidence_digest: None,
        });
        let report = project_design_to_bom_candidate(&request, &revision(), timestamp()).unwrap();
        assert!(report.bom_candidate.is_none());
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            ProjectionIssue::SubstitutionRequiresQualification { .. }
        )));
    }

    #[test]
    fn subdesign_cannot_be_silently_projected_as_part_id() {
        let mut revision = revision();
        revision.composition.entries[0].subject =
            CompositionSubject::SubDesign(id("revision:subassembly:r2"));
        let report = project_design_to_bom_candidate(&request(), &revision, timestamp()).unwrap();
        assert!(report.bom_candidate.is_none());
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            ProjectionIssue::TargetTypeMismatch { .. }
        )));
    }

    #[test]
    fn unreleased_revision_is_rejected() {
        let mut revision = revision();
        revision.state = RevisionState::Review;
        assert_eq!(
            project_design_to_bom_candidate(&request(), &revision, timestamp()).unwrap_err(),
            ProjectionError::RevisionNotReleased
        );
    }
}

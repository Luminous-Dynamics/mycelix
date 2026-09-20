// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Typed, evidence-scoped facility capability semantics.
//!
//! This crate deliberately does not mutate manufacturing state. It evaluates
//! exact manufacturing requirements against an explicit facility capability
//! record under an evidence policy. Missing or stale information remains
//! `Unknown`; legacy free-form capability strings never become verified facts.

#![deny(unsafe_code)]

use mycelix_hardware_core::{ConstraintEvaluation, DigestRef, SemanticId};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const FACILITY_CAPABILITY_SCHEMA: &str = "mycelix.manufacturing.facility-capability.v1";
pub const MAX_ITEMS: usize = 4096;
pub const MAX_TEXT_BYTES: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum CapabilityEvidenceClass {
    SelfDeclared,
    Documented,
    Calibrated,
    ObservedProduction,
    IndependentInspection,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityEvidence {
    pub id: SemanticId,
    pub class: CapabilityEvidenceClass,
    pub artifact_digest: Option<DigestRef>,
    pub observed_at_unix_s: Option<i64>,
    pub valid_until_unix_s: Option<i64>,
    pub note: Option<String>,
}

impl CapabilityEvidence {
    fn validate(&self) -> Result<(), ValidationError> {
        if let Some(digest) = &self.artifact_digest {
            digest.validate().map_err(|_| ValidationError::InvalidDigest)?;
        }
        if let (Some(observed), Some(valid_until)) =
            (self.observed_at_unix_s, self.valid_until_unix_s)
        {
            if valid_until < observed {
                return Err(ValidationError::InvalidEvidenceWindow);
            }
        }
        if let Some(note) = &self.note {
            validate_text(note, "evidence note")?;
        }
        Ok(())
    }

    fn usable_under(&self, policy: &EvidencePolicy, evaluated_at_unix_s: i64) -> bool {
        if !policy.accepted_classes.contains(&self.class) {
            return false;
        }
        if let Some(observed) = self.observed_at_unix_s {
            if observed > evaluated_at_unix_s {
                return false;
            }
        }
        if policy.require_current {
            if let Some(valid_until) = self.valid_until_unix_s {
                if valid_until < evaluated_at_unix_s {
                    return false;
                }
            }
        }
        true
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidencePolicy {
    pub id: SemanticId,
    pub accepted_classes: Vec<CapabilityEvidenceClass>,
    pub require_current: bool,
}

impl EvidencePolicy {
    fn validate(&self) -> Result<(), ValidationError> {
        if self.accepted_classes.is_empty() {
            return Err(ValidationError::EmptyCollection("accepted evidence classes"));
        }
        ensure_unique(self.accepted_classes.iter(), "accepted evidence class")
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum ProcessKind {
    Fdm,
    Sla,
    Sls,
    CncMilling3Axis,
    CncMilling5Axis,
    LaserCutting,
    Turning,
    Assembly,
    PcbFabrication,
    PcbAssembly,
    Other(String),
}

impl ProcessKind {
    fn validate(&self) -> Result<(), ValidationError> {
        if let Self::Other(value) = self {
            validate_text(value, "process kind")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum InspectionKind {
    Visual,
    Dimensional,
    Aoi,
    XRay,
    ElectricalTest,
    FunctionalTest,
    Other(String),
}

impl InspectionKind {
    fn validate(&self) -> Result<(), ValidationError> {
        if let Self::Other(value) = self {
            validate_text(value, "inspection kind")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SupportDisposition {
    Supported,
    Unsupported,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProcessAssertion {
    pub process: ProcessKind,
    pub disposition: SupportDisposition,
    pub evidence_refs: Vec<SemanticId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaterialAssertion {
    pub material_id: SemanticId,
    pub disposition: SupportDisposition,
    pub evidence_refs: Vec<SemanticId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InspectionAssertion {
    pub inspection: InspectionKind,
    pub disposition: SupportDisposition,
    pub evidence_refs: Vec<SemanticId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkEnvelope {
    pub x_um: u64,
    pub y_um: u64,
    pub z_um: u64,
    pub evidence_refs: Vec<SemanticId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DimensionalDeviationCapability {
    /// Maximum absolute dimensional deviation supported under this profile.
    pub max_abs_um: u64,
    pub evidence_refs: Vec<SemanticId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MinimumFeatureCapability {
    /// Smallest feature size the facility claims it can reliably produce.
    pub min_feature_um: u64,
    pub evidence_refs: Vec<SemanticId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyCapabilityTag {
    pub raw: String,
    /// Legacy tags are descriptive only and MUST remain Unknown.
    pub evaluation: ConstraintEvaluation,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FacilityCapabilityRecord {
    pub schema_version: String,
    pub facility_id: SemanticId,
    pub machine_id: Option<SemanticId>,
    pub capability_profile_id: SemanticId,
    pub evidence: Vec<CapabilityEvidence>,
    pub processes: Vec<ProcessAssertion>,
    pub materials: Vec<MaterialAssertion>,
    pub inspections: Vec<InspectionAssertion>,
    pub work_envelope: Option<WorkEnvelope>,
    pub max_dimensional_deviation: Option<DimensionalDeviationCapability>,
    pub minimum_feature: Option<MinimumFeatureCapability>,
    pub legacy_tags: Vec<LegacyCapabilityTag>,
}

impl FacilityCapabilityRecord {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.schema_version != FACILITY_CAPABILITY_SCHEMA {
            return Err(ValidationError::UnsupportedSchema);
        }
        for count in [
            self.evidence.len(),
            self.processes.len(),
            self.materials.len(),
            self.inspections.len(),
            self.legacy_tags.len(),
        ] {
            ensure_bounded(count)?;
        }
        ensure_unique(self.evidence.iter().map(|item| &item.id), "evidence id")?;
        for item in &self.evidence {
            item.validate()?;
        }
        for assertion in &self.processes {
            assertion.process.validate()?;
            validate_evidence_refs(&assertion.evidence_refs, &self.evidence)?;
        }
        for assertion in &self.materials {
            validate_evidence_refs(&assertion.evidence_refs, &self.evidence)?;
        }
        for assertion in &self.inspections {
            assertion.inspection.validate()?;
            validate_evidence_refs(&assertion.evidence_refs, &self.evidence)?;
        }
        if let Some(envelope) = &self.work_envelope {
            if envelope.x_um == 0 || envelope.y_um == 0 || envelope.z_um == 0 {
                return Err(ValidationError::ZeroCapabilityValue("work envelope"));
            }
            validate_evidence_refs(&envelope.evidence_refs, &self.evidence)?;
        }
        if let Some(capability) = &self.max_dimensional_deviation {
            if capability.max_abs_um == 0 {
                return Err(ValidationError::ZeroCapabilityValue(
                    "max dimensional deviation",
                ));
            }
            validate_evidence_refs(&capability.evidence_refs, &self.evidence)?;
        }
        if let Some(capability) = &self.minimum_feature {
            if capability.min_feature_um == 0 {
                return Err(ValidationError::ZeroCapabilityValue("minimum feature"));
            }
            validate_evidence_refs(&capability.evidence_refs, &self.evidence)?;
        }
        for tag in &self.legacy_tags {
            validate_text(&tag.raw, "legacy capability tag")?;
            if tag.evaluation != ConstraintEvaluation::Unknown {
                return Err(ValidationError::LegacyTagMustRemainUnknown);
            }
        }
        ensure_unique(self.legacy_tags.iter().map(|item| &item.raw), "legacy tag")?;
        Ok(())
    }

    pub fn legacy_tags_from_strings(values: &[String]) -> Result<Vec<LegacyCapabilityTag>, ValidationError> {
        ensure_bounded(values.len())?;
        let mut seen = BTreeSet::new();
        let mut tags = Vec::new();
        for value in values {
            validate_text(value, "legacy capability tag")?;
            if seen.insert(value.clone()) {
                tags.push(LegacyCapabilityTag {
                    raw: value.clone(),
                    evaluation: ConstraintEvaluation::Unknown,
                });
            }
        }
        Ok(tags)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ManufacturingRequirement {
    pub id: SemanticId,
    pub constraint: ManufacturingConstraint,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ManufacturingConstraint {
    Process(ProcessKind),
    Material(SemanticId),
    WorkEnvelopeAtLeast { x_um: u64, y_um: u64, z_um: u64 },
    MaxDimensionalDeviationAtMost { um: u64 },
    MinimumFeatureAtMost { um: u64 },
    Inspection(InspectionKind),
}

impl ManufacturingRequirement {
    fn validate(&self) -> Result<(), ValidationError> {
        match &self.constraint {
            ManufacturingConstraint::Process(process) => process.validate(),
            ManufacturingConstraint::Inspection(inspection) => inspection.validate(),
            ManufacturingConstraint::Material(_) => Ok(()),
            ManufacturingConstraint::WorkEnvelopeAtLeast { x_um, y_um, z_um } => {
                if *x_um == 0 || *y_um == 0 || *z_um == 0 {
                    Err(ValidationError::ZeroRequirementValue("work envelope"))
                } else {
                    Ok(())
                }
            }
            ManufacturingConstraint::MaxDimensionalDeviationAtMost { um }
            | ManufacturingConstraint::MinimumFeatureAtMost { um } => {
                if *um == 0 {
                    Err(ValidationError::ZeroRequirementValue("numeric manufacturing constraint"))
                } else {
                    Ok(())
                }
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ManufacturabilityRequest {
    pub design_revision_id: SemanticId,
    pub facility_id: SemanticId,
    pub capability_profile_id: SemanticId,
    pub evidence_policy: EvidencePolicy,
    pub evaluated_at_unix_s: i64,
    pub requirements: Vec<ManufacturingRequirement>,
}

impl ManufacturabilityRequest {
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.evidence_policy.validate()?;
        ensure_bounded(self.requirements.len())?;
        ensure_unique(self.requirements.iter().map(|item| &item.id), "requirement id")?;
        for requirement in &self.requirements {
            requirement.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RequirementMatch {
    pub requirement_id: SemanticId,
    pub evaluation: ConstraintEvaluation,
    pub evidence_refs: Vec<SemanticId>,
    pub reason: MatchReason,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum MatchReason {
    SupportedByAcceptedEvidence,
    ExplicitlyUnsupportedByAcceptedEvidence,
    NumericCapabilityMeetsRequirement,
    NumericCapabilityFailsRequirement,
    CapabilityNotDeclared,
    EvidenceMissingOrUnacceptable,
    ConflictingAssertions,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ManufacturabilityMatch {
    pub design_revision_id: SemanticId,
    pub facility_id: SemanticId,
    pub capability_profile_id: SemanticId,
    pub evidence_policy_id: SemanticId,
    pub matches: Vec<RequirementMatch>,
    pub legacy_tags: Vec<LegacyCapabilityTag>,
}

impl ManufacturabilityMatch {
    /// This only answers whether every requested constraint was satisfied under
    /// this exact profile/policy. It is not manufacturing authorization or a
    /// claim that a produced specimen will conform.
    pub fn all_constraints_satisfied_under_profile(&self) -> bool {
        self.matches
            .iter()
            .all(|item| item.evaluation == ConstraintEvaluation::Satisfied)
    }
}

pub fn match_requirements(
    record: &FacilityCapabilityRecord,
    request: &ManufacturabilityRequest,
) -> Result<ManufacturabilityMatch, MatchError> {
    record.validate().map_err(MatchError::InvalidCapabilityRecord)?;
    request.validate().map_err(MatchError::InvalidRequest)?;
    if record.facility_id != request.facility_id {
        return Err(MatchError::FacilityMismatch);
    }
    if record.capability_profile_id != request.capability_profile_id {
        return Err(MatchError::CapabilityProfileMismatch);
    }

    let mut matches = Vec::with_capacity(request.requirements.len());
    for requirement in &request.requirements {
        matches.push(evaluate_requirement(record, request, requirement));
    }

    Ok(ManufacturabilityMatch {
        design_revision_id: request.design_revision_id.clone(),
        facility_id: request.facility_id.clone(),
        capability_profile_id: request.capability_profile_id.clone(),
        evidence_policy_id: request.evidence_policy.id.clone(),
        matches,
        legacy_tags: record.legacy_tags.clone(),
    })
}

fn evaluate_requirement(
    record: &FacilityCapabilityRecord,
    request: &ManufacturabilityRequest,
    requirement: &ManufacturingRequirement,
) -> RequirementMatch {
    match &requirement.constraint {
        ManufacturingConstraint::Process(process) => evaluate_process(record, request, requirement, process),
        ManufacturingConstraint::Material(material_id) => {
            evaluate_material(record, request, requirement, material_id)
        }
        ManufacturingConstraint::Inspection(inspection) => {
            evaluate_inspection(record, request, requirement, inspection)
        }
        ManufacturingConstraint::WorkEnvelopeAtLeast { x_um, y_um, z_um } => {
            let Some(capability) = &record.work_envelope else {
                return unknown(requirement, MatchReason::CapabilityNotDeclared);
            };
            let usable = usable_refs(record, &capability.evidence_refs, request);
            if usable.is_empty() {
                return unknown(requirement, MatchReason::EvidenceMissingOrUnacceptable);
            }
            let passes = capability.x_um >= *x_um
                && capability.y_um >= *y_um
                && capability.z_um >= *z_um;
            numeric_match(requirement, usable, passes)
        }
        ManufacturingConstraint::MaxDimensionalDeviationAtMost { um } => {
            let Some(capability) = &record.max_dimensional_deviation else {
                return unknown(requirement, MatchReason::CapabilityNotDeclared);
            };
            let usable = usable_refs(record, &capability.evidence_refs, request);
            if usable.is_empty() {
                return unknown(requirement, MatchReason::EvidenceMissingOrUnacceptable);
            }
            numeric_match(requirement, usable, capability.max_abs_um <= *um)
        }
        ManufacturingConstraint::MinimumFeatureAtMost { um } => {
            let Some(capability) = &record.minimum_feature else {
                return unknown(requirement, MatchReason::CapabilityNotDeclared);
            };
            let usable = usable_refs(record, &capability.evidence_refs, request);
            if usable.is_empty() {
                return unknown(requirement, MatchReason::EvidenceMissingOrUnacceptable);
            }
            numeric_match(requirement, usable, capability.min_feature_um <= *um)
        }
    }
}

fn evaluate_process(
    record: &FacilityCapabilityRecord,
    request: &ManufacturabilityRequest,
    requirement: &ManufacturingRequirement,
    process: &ProcessKind,
) -> RequirementMatch {
    let assertions: Vec<_> = record
        .processes
        .iter()
        .filter(|item| &item.process == process)
        .collect();
    evaluate_categorical(
        record,
        request,
        requirement,
        assertions.iter().map(|item| (item.disposition, &item.evidence_refs)),
    )
}

fn evaluate_material(
    record: &FacilityCapabilityRecord,
    request: &ManufacturabilityRequest,
    requirement: &ManufacturingRequirement,
    material_id: &SemanticId,
) -> RequirementMatch {
    let assertions: Vec<_> = record
        .materials
        .iter()
        .filter(|item| &item.material_id == material_id)
        .collect();
    evaluate_categorical(
        record,
        request,
        requirement,
        assertions.iter().map(|item| (item.disposition, &item.evidence_refs)),
    )
}

fn evaluate_inspection(
    record: &FacilityCapabilityRecord,
    request: &ManufacturabilityRequest,
    requirement: &ManufacturingRequirement,
    inspection: &InspectionKind,
) -> RequirementMatch {
    let assertions: Vec<_> = record
        .inspections
        .iter()
        .filter(|item| &item.inspection == inspection)
        .collect();
    evaluate_categorical(
        record,
        request,
        requirement,
        assertions.iter().map(|item| (item.disposition, &item.evidence_refs)),
    )
}

fn evaluate_categorical<'a, I>(
    record: &FacilityCapabilityRecord,
    request: &ManufacturabilityRequest,
    requirement: &ManufacturingRequirement,
    assertions: I,
) -> RequirementMatch
where
    I: IntoIterator<Item = (SupportDisposition, &'a Vec<SemanticId>)>,
{
    let assertions: Vec<_> = assertions.into_iter().collect();
    if assertions.is_empty() {
        return unknown(requirement, MatchReason::CapabilityNotDeclared);
    }

    let mut supported_refs = BTreeSet::new();
    let mut unsupported_refs = BTreeSet::new();
    for (disposition, refs) in assertions {
        for evidence_id in usable_refs(record, refs, request) {
            match disposition {
                SupportDisposition::Supported => {
                    supported_refs.insert(evidence_id);
                }
                SupportDisposition::Unsupported => {
                    unsupported_refs.insert(evidence_id);
                }
            }
        }
    }

    if !supported_refs.is_empty() && !unsupported_refs.is_empty() {
        let mut refs: Vec<_> = supported_refs.into_iter().collect();
        refs.extend(unsupported_refs);
        refs.sort();
        refs.dedup();
        return RequirementMatch {
            requirement_id: requirement.id.clone(),
            evaluation: ConstraintEvaluation::Unknown,
            evidence_refs: refs,
            reason: MatchReason::ConflictingAssertions,
        };
    }
    if !supported_refs.is_empty() {
        return RequirementMatch {
            requirement_id: requirement.id.clone(),
            evaluation: ConstraintEvaluation::Satisfied,
            evidence_refs: supported_refs.into_iter().collect(),
            reason: MatchReason::SupportedByAcceptedEvidence,
        };
    }
    if !unsupported_refs.is_empty() {
        return RequirementMatch {
            requirement_id: requirement.id.clone(),
            evaluation: ConstraintEvaluation::Unsatisfied,
            evidence_refs: unsupported_refs.into_iter().collect(),
            reason: MatchReason::ExplicitlyUnsupportedByAcceptedEvidence,
        };
    }
    unknown(requirement, MatchReason::EvidenceMissingOrUnacceptable)
}

fn numeric_match(
    requirement: &ManufacturingRequirement,
    evidence_refs: Vec<SemanticId>,
    passes: bool,
) -> RequirementMatch {
    RequirementMatch {
        requirement_id: requirement.id.clone(),
        evaluation: if passes {
            ConstraintEvaluation::Satisfied
        } else {
            ConstraintEvaluation::Unsatisfied
        },
        evidence_refs,
        reason: if passes {
            MatchReason::NumericCapabilityMeetsRequirement
        } else {
            MatchReason::NumericCapabilityFailsRequirement
        },
    }
}

fn unknown(requirement: &ManufacturingRequirement, reason: MatchReason) -> RequirementMatch {
    RequirementMatch {
        requirement_id: requirement.id.clone(),
        evaluation: ConstraintEvaluation::Unknown,
        evidence_refs: Vec::new(),
        reason,
    }
}

fn usable_refs(
    record: &FacilityCapabilityRecord,
    refs: &[SemanticId],
    request: &ManufacturabilityRequest,
) -> Vec<SemanticId> {
    refs.iter()
        .filter_map(|id| {
            let evidence = record.evidence.iter().find(|item| &item.id == id)?;
            evidence
                .usable_under(&request.evidence_policy, request.evaluated_at_unix_s)
                .then(|| id.clone())
        })
        .collect()
}

fn validate_evidence_refs(
    refs: &[SemanticId],
    evidence: &[CapabilityEvidence],
) -> Result<(), ValidationError> {
    ensure_bounded(refs.len())?;
    ensure_unique(refs.iter(), "evidence reference")?;
    if refs.iter().any(|id| !evidence.iter().any(|item| &item.id == id)) {
        return Err(ValidationError::UnknownEvidenceReference);
    }
    Ok(())
}

fn ensure_bounded(count: usize) -> Result<(), ValidationError> {
    if count > MAX_ITEMS {
        Err(ValidationError::TooManyItems)
    } else {
        Ok(())
    }
}

fn ensure_unique<'a, T, I>(values: I, kind: &'static str) -> Result<(), ValidationError>
where
    T: Ord + ?Sized + 'a,
    I: IntoIterator<Item = &'a T>,
{
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value) {
            return Err(ValidationError::Duplicate(kind));
        }
    }
    Ok(())
}

fn validate_text(value: &str, field: &'static str) -> Result<(), ValidationError> {
    if value.trim().is_empty() || value != value.trim() || value.chars().any(char::is_control) {
        return Err(ValidationError::InvalidText(field));
    }
    if value.len() > MAX_TEXT_BYTES {
        return Err(ValidationError::FieldTooLong(field));
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValidationError {
    UnsupportedSchema,
    InvalidDigest,
    InvalidEvidenceWindow,
    EmptyCollection(&'static str),
    TooManyItems,
    Duplicate(&'static str),
    UnknownEvidenceReference,
    ZeroCapabilityValue(&'static str),
    ZeroRequirementValue(&'static str),
    LegacyTagMustRemainUnknown,
    InvalidText(&'static str),
    FieldTooLong(&'static str),
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchema => write!(f, "unsupported facility capability schema"),
            Self::InvalidDigest => write!(f, "invalid evidence digest"),
            Self::InvalidEvidenceWindow => write!(f, "evidence validity ends before observation"),
            Self::EmptyCollection(kind) => write!(f, "{kind} must not be empty"),
            Self::TooManyItems => write!(f, "collection exceeds maximum item count"),
            Self::Duplicate(kind) => write!(f, "duplicate {kind}"),
            Self::UnknownEvidenceReference => write!(f, "capability references unknown evidence"),
            Self::ZeroCapabilityValue(kind) => write!(f, "{kind} capability must be greater than zero"),
            Self::ZeroRequirementValue(kind) => write!(f, "{kind} requirement must be greater than zero"),
            Self::LegacyTagMustRemainUnknown => write!(f, "legacy capability tags must remain Unknown"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::FieldTooLong(field) => write!(f, "{field} exceeds maximum length"),
        }
    }
}

impl std::error::Error for ValidationError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MatchError {
    InvalidCapabilityRecord(ValidationError),
    InvalidRequest(ValidationError),
    FacilityMismatch,
    CapabilityProfileMismatch,
}

impl fmt::Display for MatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCapabilityRecord(error) => write!(f, "invalid facility capability record: {error}"),
            Self::InvalidRequest(error) => write!(f, "invalid manufacturability request: {error}"),
            Self::FacilityMismatch => write!(f, "request facility does not match capability record"),
            Self::CapabilityProfileMismatch => write!(f, "request capability profile does not match record"),
        }
    }
}

impl std::error::Error for MatchError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> SemanticId {
        SemanticId::new(value).unwrap()
    }

    fn evidence(id_value: &str, class: CapabilityEvidenceClass, valid_until: Option<i64>) -> CapabilityEvidence {
        CapabilityEvidence {
            id: id(id_value),
            class,
            artifact_digest: None,
            observed_at_unix_s: Some(100),
            valid_until_unix_s: valid_until,
            note: None,
        }
    }

    fn policy() -> EvidencePolicy {
        EvidencePolicy {
            id: id("policy:qualified"),
            accepted_classes: vec![
                CapabilityEvidenceClass::Calibrated,
                CapabilityEvidenceClass::IndependentInspection,
            ],
            require_current: true,
        }
    }

    fn record() -> FacilityCapabilityRecord {
        FacilityCapabilityRecord {
            schema_version: FACILITY_CAPABILITY_SCHEMA.into(),
            facility_id: id("facility:alpha"),
            machine_id: Some(id("machine:cnc-5")),
            capability_profile_id: id("profile:standard"),
            evidence: vec![
                evidence("evidence:inspection", CapabilityEvidenceClass::IndependentInspection, Some(500)),
                evidence("evidence:calibration", CapabilityEvidenceClass::Calibrated, Some(500)),
            ],
            processes: vec![ProcessAssertion {
                process: ProcessKind::CncMilling5Axis,
                disposition: SupportDisposition::Supported,
                evidence_refs: vec![id("evidence:inspection")],
            }],
            materials: vec![MaterialAssertion {
                material_id: id("material:aluminum-6061"),
                disposition: SupportDisposition::Supported,
                evidence_refs: vec![id("evidence:inspection")],
            }],
            inspections: vec![InspectionAssertion {
                inspection: InspectionKind::Dimensional,
                disposition: SupportDisposition::Supported,
                evidence_refs: vec![id("evidence:calibration")],
            }],
            work_envelope: Some(WorkEnvelope {
                x_um: 500_000,
                y_um: 400_000,
                z_um: 300_000,
                evidence_refs: vec![id("evidence:inspection")],
            }),
            max_dimensional_deviation: Some(DimensionalDeviationCapability {
                max_abs_um: 25,
                evidence_refs: vec![id("evidence:calibration")],
            }),
            minimum_feature: Some(MinimumFeatureCapability {
                min_feature_um: 150,
                evidence_refs: vec![id("evidence:inspection")],
            }),
            legacy_tags: vec![],
        }
    }

    fn request(requirement: ManufacturingRequirement) -> ManufacturabilityRequest {
        ManufacturabilityRequest {
            design_revision_id: id("design:fixture:r1"),
            facility_id: id("facility:alpha"),
            capability_profile_id: id("profile:standard"),
            evidence_policy: policy(),
            evaluated_at_unix_s: 200,
            requirements: vec![requirement],
        }
    }

    #[test]
    fn missing_tolerance_is_unknown() {
        let mut record = record();
        record.max_dimensional_deviation = None;
        let report = match_requirements(
            &record,
            &request(ManufacturingRequirement {
                id: id("req:tolerance"),
                constraint: ManufacturingConstraint::MaxDimensionalDeviationAtMost { um: 50 },
            }),
        )
        .unwrap();
        assert_eq!(report.matches[0].evaluation, ConstraintEvaluation::Unknown);
        assert_eq!(report.matches[0].reason, MatchReason::CapabilityNotDeclared);
    }

    #[test]
    fn sufficient_current_calibrated_tolerance_is_satisfied() {
        let report = match_requirements(
            &record(),
            &request(ManufacturingRequirement {
                id: id("req:tolerance"),
                constraint: ManufacturingConstraint::MaxDimensionalDeviationAtMost { um: 50 },
            }),
        )
        .unwrap();
        assert_eq!(report.matches[0].evaluation, ConstraintEvaluation::Satisfied);
    }

    #[test]
    fn insufficient_tolerance_is_unsatisfied_when_evidence_is_acceptable() {
        let mut record = record();
        record.max_dimensional_deviation.as_mut().unwrap().max_abs_um = 75;
        let report = match_requirements(
            &record,
            &request(ManufacturingRequirement {
                id: id("req:tolerance"),
                constraint: ManufacturingConstraint::MaxDimensionalDeviationAtMost { um: 50 },
            }),
        )
        .unwrap();
        assert_eq!(report.matches[0].evaluation, ConstraintEvaluation::Unsatisfied);
    }

    #[test]
    fn expired_calibration_downgrades_to_unknown() {
        let mut record = record();
        let calibration = record
            .evidence
            .iter_mut()
            .find(|item| item.id == id("evidence:calibration"))
            .unwrap();
        calibration.valid_until_unix_s = Some(150);
        let report = match_requirements(
            &record,
            &request(ManufacturingRequirement {
                id: id("req:tolerance"),
                constraint: ManufacturingConstraint::MaxDimensionalDeviationAtMost { um: 50 },
            }),
        )
        .unwrap();
        assert_eq!(report.matches[0].evaluation, ConstraintEvaluation::Unknown);
        assert_eq!(
            report.matches[0].reason,
            MatchReason::EvidenceMissingOrUnacceptable
        );
    }

    #[test]
    fn legacy_strings_never_become_verified_capabilities() {
        let strings = vec!["high precision CNC".to_string(), "5-axis".to_string()];
        let tags = FacilityCapabilityRecord::legacy_tags_from_strings(&strings).unwrap();
        assert!(tags
            .iter()
            .all(|tag| tag.evaluation == ConstraintEvaluation::Unknown));
    }

    #[test]
    fn absent_process_assertion_is_unknown_not_unsupported() {
        let report = match_requirements(
            &record(),
            &request(ManufacturingRequirement {
                id: id("req:process"),
                constraint: ManufacturingConstraint::Process(ProcessKind::LaserCutting),
            }),
        )
        .unwrap();
        assert_eq!(report.matches[0].evaluation, ConstraintEvaluation::Unknown);
    }

    #[test]
    fn explicit_unsupported_process_is_unsatisfied() {
        let mut record = record();
        record.processes.push(ProcessAssertion {
            process: ProcessKind::LaserCutting,
            disposition: SupportDisposition::Unsupported,
            evidence_refs: vec![id("evidence:inspection")],
        });
        let report = match_requirements(
            &record,
            &request(ManufacturingRequirement {
                id: id("req:process"),
                constraint: ManufacturingConstraint::Process(ProcessKind::LaserCutting),
            }),
        )
        .unwrap();
        assert_eq!(report.matches[0].evaluation, ConstraintEvaluation::Unsatisfied);
    }

    #[test]
    fn contradictory_supported_and_unsupported_assertions_require_review() {
        let mut record = record();
        record.processes.push(ProcessAssertion {
            process: ProcessKind::CncMilling5Axis,
            disposition: SupportDisposition::Unsupported,
            evidence_refs: vec![id("evidence:inspection")],
        });
        let report = match_requirements(
            &record,
            &request(ManufacturingRequirement {
                id: id("req:process"),
                constraint: ManufacturingConstraint::Process(ProcessKind::CncMilling5Axis),
            }),
        )
        .unwrap();
        assert_eq!(report.matches[0].evaluation, ConstraintEvaluation::Unknown);
        assert_eq!(report.matches[0].reason, MatchReason::ConflictingAssertions);
    }

    #[test]
    fn profile_mismatch_fails_before_evaluation() {
        let mut request = request(ManufacturingRequirement {
            id: id("req:process"),
            constraint: ManufacturingConstraint::Process(ProcessKind::CncMilling5Axis),
        });
        request.capability_profile_id = id("profile:other");
        assert_eq!(
            match_requirements(&record(), &request).unwrap_err(),
            MatchError::CapabilityProfileMismatch
        );
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Transport-independent semantic core for Mycelix open hardware.
//!
//! This crate owns semantic identity and relationships only. It deliberately has
//! no Holochain, CAD/EDA, solver, manufacturing-execution, signing, or filesystem
//! dependencies. Those systems project into or reference these types through
//! explicit adapters.

#![deny(unsafe_code)]

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const HARDWARE_SEMANTIC_SCHEMA: &str = "mycelix.hardware.semantic.v1";
pub const MAX_ID_BYTES: usize = 256;
pub const MAX_NAME_BYTES: usize = 256;
pub const MAX_DESCRIPTION_BYTES: usize = 16 * 1024;
pub const MAX_ITEMS: usize = 16_384;

/// Validated, transport-independent semantic identifier.
///
/// IDs are intentionally opaque. The semantic layer does not require UUIDs,
/// Holochain action hashes, DIDs, Git SHAs, or another transport-specific scheme.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct SemanticId(String);

impl SemanticId {
    pub fn new(value: impl Into<String>) -> Result<Self, ValidationError> {
        let value = value.into();
        validate_identifier(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl TryFrom<String> for SemanticId {
    type Error = ValidationError;

    fn try_from(value: String) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl From<SemanticId> for String {
    fn from(value: SemanticId) -> Self {
        value.0
    }
}

impl fmt::Display for SemanticId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// Digest identity produced and verified by another subsystem.
/// This crate validates representation only; it does not own the referenced bytes.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct DigestRef {
    pub algorithm: DigestAlgorithm,
    pub hex: String,
}

impl DigestRef {
    pub fn sha256(hex: impl Into<String>) -> Result<Self, ValidationError> {
        let digest = Self {
            algorithm: DigestAlgorithm::Sha256,
            hex: hex.into(),
        };
        digest.validate()?;
        Ok(digest)
    }

    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.hex.len() != 64
            || !self
                .hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
        {
            return Err(ValidationError::InvalidDigest);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum DigestAlgorithm {
    Sha256,
    Blake3,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareProject {
    pub schema_version: String,
    pub id: SemanticId,
    pub name: String,
    pub description: Option<String>,
    pub maintainers: Vec<ActorRef>,
    pub license_refs: Vec<LicenseRef>,
}

impl HardwareProject {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_schema(&self.schema_version)?;
        validate_name(&self.name)?;
        validate_optional_description(self.description.as_deref())?;
        ensure_bounded(self.maintainers.len())?;
        ensure_bounded(self.license_refs.len())?;
        ensure_unique(self.maintainers.iter().map(|actor| &actor.id), "maintainer")?;
        ensure_unique(
            self.license_refs.iter().map(|license| &license.identifier),
            "license",
        )?;
        for maintainer in &self.maintainers {
            maintainer.validate()?;
        }
        for license in &self.license_refs {
            license.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ActorRef {
    pub id: SemanticId,
    pub display_name: Option<String>,
}

impl ActorRef {
    fn validate(&self) -> Result<(), ValidationError> {
        if let Some(name) = &self.display_name {
            validate_name(name)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct LicenseRef {
    /// SPDX expression or another explicitly namespaced license identifier.
    pub identifier: String,
    pub text_or_url: Option<String>,
}

impl LicenseRef {
    fn validate(&self) -> Result<(), ValidationError> {
        validate_nonempty_bounded(&self.identifier, MAX_NAME_BYTES, "license identifier")?;
        if let Some(reference) = &self.text_or_url {
            validate_nonempty_bounded(reference, 4096, "license reference")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DesignRevision {
    pub schema_version: String,
    pub id: SemanticId,
    pub project_id: SemanticId,
    pub revision_label: String,
    pub state: RevisionState,
    pub predecessors: Vec<SemanticId>,
    pub artifacts: Vec<DesignArtifactRef>,
    /// Design intent, deliberately distinct from a manufacturing BOM.
    pub composition: DesignComposition,
    pub requirements: Vec<HardwareRequirement>,
    pub documentation_profiles: Vec<DocumentationProfileRef>,
    pub evidence_refs: Vec<EvidenceReference>,
}

impl DesignRevision {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_schema(&self.schema_version)?;
        validate_nonempty_bounded(&self.revision_label, MAX_NAME_BYTES, "revision label")?;
        for count in [
            self.predecessors.len(),
            self.artifacts.len(),
            self.requirements.len(),
            self.documentation_profiles.len(),
            self.evidence_refs.len(),
        ] {
            ensure_bounded(count)?;
        }
        if self.predecessors.iter().any(|id| id == &self.id) {
            return Err(ValidationError::SelfReference("revision predecessor"));
        }
        ensure_unique(self.predecessors.iter(), "revision predecessor")?;
        ensure_unique(self.artifacts.iter().map(|item| &item.id), "artifact")?;
        ensure_unique(self.requirements.iter().map(|item| &item.id), "requirement")?;
        ensure_unique(
            self.documentation_profiles.iter().map(|item| &item.id),
            "documentation profile",
        )?;
        ensure_unique(self.evidence_refs.iter().map(|item| &item.id), "evidence")?;
        for artifact in &self.artifacts {
            artifact.validate()?;
        }
        self.composition.validate()?;
        for requirement in &self.requirements {
            requirement.validate()?;
        }
        for profile in &self.documentation_profiles {
            profile.validate()?;
        }
        for evidence in &self.evidence_refs {
            evidence.validate()?;
            if evidence.subject_id != self.id {
                return Err(ValidationError::EvidenceSubjectMismatch);
            }
            if let Some(claim_id) = &evidence.claim_id {
                if !self.requirements.iter().any(|requirement| &requirement.id == claim_id) {
                    return Err(ValidationError::UnknownClaimReference);
                }
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RevisionState {
    Draft,
    Review,
    Released,
    Superseded,
    Withdrawn,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DesignArtifactRef {
    pub id: SemanticId,
    pub role: ArtifactRole,
    pub digest: DigestRef,
    pub byte_length: Option<u64>,
    pub media_type: Option<String>,
    pub logical_path: Option<String>,
}

impl DesignArtifactRef {
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.digest.validate()?;
        if let Some(media_type) = &self.media_type {
            if media_type.len() > 256
                || !media_type.contains('/')
                || media_type.chars().any(char::is_control)
            {
                return Err(ValidationError::InvalidMediaType);
            }
        }
        if let Some(path) = &self.logical_path {
            validate_logical_path(path)?;
        }
        if let ArtifactRole::Other(name) = &self.role {
            validate_nonempty_bounded(name, MAX_NAME_BYTES, "artifact role")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArtifactRole {
    NativeCad,
    Schematic,
    PcbLayout,
    MechanicalCad,
    FirmwareSource,
    FirmwareBinary,
    HdlSource,
    ManufacturingOutput,
    AssemblyDocumentation,
    TestPlan,
    TestResult,
    CalibrationData,
    Datasheet,
    BomExchange,
    Other(String),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DesignComposition {
    pub entries: Vec<CompositionEntry>,
}

impl DesignComposition {
    pub fn validate(&self) -> Result<(), ValidationError> {
        ensure_bounded(self.entries.len())?;
        ensure_unique(self.entries.iter().map(|entry| &entry.id), "composition entry")?;
        for entry in &self.entries {
            entry.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompositionEntry {
    pub id: SemanticId,
    pub subject: CompositionSubject,
    pub quantity: Quantity,
    pub designators: Vec<String>,
    pub notes: Option<String>,
}

impl CompositionEntry {
    fn validate(&self) -> Result<(), ValidationError> {
        self.quantity.validate()?;
        ensure_bounded(self.designators.len())?;
        ensure_unique(self.designators.iter(), "designator")?;
        for designator in &self.designators {
            validate_nonempty_bounded(designator, 128, "designator")?;
        }
        validate_optional_description(self.notes.as_deref())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompositionSubject {
    Component(SemanticId),
    SubDesign(SemanticId),
    Material(SemanticId),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Quantity {
    pub value: u64,
    pub unit: QuantityUnit,
}

impl Quantity {
    fn validate(&self) -> Result<(), ValidationError> {
        if self.value == 0 {
            return Err(ValidationError::ZeroQuantity);
        }
        if let QuantityUnit::Other(unit) = &self.unit {
            validate_nonempty_bounded(unit, 64, "quantity unit")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum QuantityUnit {
    Each,
    Millimetre,
    Metre,
    SquareMillimetre,
    SquareMetre,
    CubicMillimetre,
    CubicMetre,
    Gram,
    Kilogram,
    Millilitre,
    Litre,
    Other(String),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ComponentIdentity {
    pub schema_version: String,
    pub id: SemanticId,
    pub name: String,
    pub kind: ComponentKind,
    pub manufacturer: Option<String>,
    pub manufacturer_part_number: Option<String>,
    pub open_hardware_project: Option<SemanticId>,
    pub datasheet_refs: Vec<DesignArtifactRef>,
    pub lifecycle: LifecycleState,
}

impl ComponentIdentity {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_schema(&self.schema_version)?;
        validate_name(&self.name)?;
        if let ComponentKind::Other(name) = &self.kind {
            validate_nonempty_bounded(name, MAX_NAME_BYTES, "component kind")?;
        }
        if let Some(manufacturer) = &self.manufacturer {
            validate_nonempty_bounded(manufacturer, MAX_NAME_BYTES, "manufacturer")?;
        }
        if let Some(part) = &self.manufacturer_part_number {
            validate_nonempty_bounded(part, MAX_NAME_BYTES, "manufacturer part number")?;
        }
        ensure_bounded(self.datasheet_refs.len())?;
        ensure_unique(self.datasheet_refs.iter().map(|item| &item.id), "datasheet")?;
        for artifact in &self.datasheet_refs {
            artifact.validate()?;
            if artifact.role != ArtifactRole::Datasheet {
                return Err(ValidationError::WrongArtifactRole("datasheet"));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ComponentKind {
    Electronic,
    Electromechanical,
    Mechanical,
    Optical,
    Fluidic,
    Material,
    Fastener,
    Cable,
    Module,
    Other(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleState {
    Unknown,
    Active,
    NotRecommendedForNewDesigns,
    EndOfLife,
    Obsolete,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareRequirement {
    pub id: SemanticId,
    pub statement: String,
    pub criticality: RequirementCriticality,
    pub verification_method: VerificationMethod,
    pub source_ref: Option<String>,
}

impl HardwareRequirement {
    fn validate(&self) -> Result<(), ValidationError> {
        validate_nonempty_bounded(&self.statement, MAX_DESCRIPTION_BYTES, "requirement")?;
        if let Some(source) = &self.source_ref {
            validate_nonempty_bounded(source, 4096, "requirement source")?;
        }
        if let VerificationMethod::Mixed(methods) = &self.verification_method {
            if methods.is_empty() {
                return Err(ValidationError::EmptyCollection("verification methods"));
            }
            ensure_bounded(methods.len())?;
            ensure_unique(methods.iter(), "verification method")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RequirementCriticality {
    Informational,
    Low,
    Medium,
    High,
    Blocking,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerificationMethod {
    Inspection,
    Analysis,
    Simulation,
    Test,
    FormalProof,
    FieldObservation,
    StandardReview,
    Mixed(Vec<VerificationMethodKind>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum VerificationMethodKind {
    Inspection,
    Analysis,
    Simulation,
    Test,
    FormalProof,
    FieldObservation,
    StandardReview,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DocumentationProfileRef {
    pub id: SemanticId,
    pub profile: DocumentationProfile,
    pub version: String,
    pub assessment: ConstraintEvaluation,
    pub report_ref: Option<SemanticId>,
}

impl DocumentationProfileRef {
    fn validate(&self) -> Result<(), ValidationError> {
        validate_nonempty_bounded(&self.version, MAX_NAME_BYTES, "profile version")?;
        if let DocumentationProfile::Other(name) = &self.profile {
            validate_nonempty_bounded(name, MAX_NAME_BYTES, "profile name")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DocumentationProfile {
    OpenKnowHow,
    Oshwa,
    DinSpec3105,
    SpdxHardware,
    CycloneDxHardware,
    Other(String),
}

/// Four-state result that keeps missing information distinct from failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConstraintEvaluation {
    Satisfied,
    Unsatisfied,
    Unknown,
    NotApplicable,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceReference {
    pub id: SemanticId,
    pub subject_id: SemanticId,
    pub claim_id: Option<SemanticId>,
    pub class: EvidenceClass,
    pub authority: EvidenceAuthority,
    pub artifact_digest: DigestRef,
    pub profile: Option<String>,
    /// Interpretation is deliberately separate from evidence existence.
    pub interpretation: EvidenceInterpretation,
}

impl EvidenceReference {
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.artifact_digest.validate()?;
        self.authority.validate()?;
        if let EvidenceClass::Other(name) = &self.class {
            validate_nonempty_bounded(name, MAX_NAME_BYTES, "evidence class")?;
        }
        if let Some(profile) = &self.profile {
            validate_nonempty_bounded(profile, MAX_NAME_BYTES, "evidence profile")?;
        }
        if let Some(scope) = &self.interpretation.scope {
            validate_nonempty_bounded(scope, MAX_DESCRIPTION_BYTES, "evidence scope")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceClass {
    DirectObservation,
    Inspection,
    Test,
    Simulation,
    FormalProof,
    Standard,
    Provenance,
    Calibration,
    FieldTelemetry,
    Assertion,
    DerivedAnalysis,
    Other(String),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceAuthority {
    pub producer: SemanticId,
    pub tool_or_system: Option<String>,
    pub environment_digest: Option<DigestRef>,
}

impl EvidenceAuthority {
    fn validate(&self) -> Result<(), ValidationError> {
        if let Some(tool) = &self.tool_or_system {
            validate_nonempty_bounded(tool, MAX_NAME_BYTES, "evidence producer tool")?;
        }
        if let Some(digest) = &self.environment_digest {
            digest.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceInterpretation {
    pub evaluation: ConstraintEvaluation,
    pub scope: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DesignRelationship {
    pub id: SemanticId,
    pub from: SemanticId,
    pub to: SemanticId,
    pub kind: RelationshipKind,
    pub evidence_refs: Vec<SemanticId>,
}

impl DesignRelationship {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.from == self.to {
            return Err(ValidationError::SelfReference("design relationship"));
        }
        ensure_bounded(self.evidence_refs.len())?;
        ensure_unique(self.evidence_refs.iter(), "relationship evidence")?;
        if let RelationshipKind::Other(name) = &self.kind {
            validate_nonempty_bounded(name, MAX_NAME_BYTES, "relationship kind")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum RelationshipKind {
    DerivedFrom,
    Supersedes,
    CompatibleWith,
    AlternativeTo,
    ReplacesUnderProfile,
    Implements,
    Other(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValidationError {
    UnsupportedSchema,
    EmptyField(&'static str),
    FieldTooLong(&'static str),
    InvalidIdentifier,
    InvalidDigest,
    InvalidMediaType,
    UnsafeLogicalPath,
    TooManyItems,
    EmptyCollection(&'static str),
    Duplicate(&'static str),
    SelfReference(&'static str),
    ZeroQuantity,
    WrongArtifactRole(&'static str),
    EvidenceSubjectMismatch,
    UnknownClaimReference,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchema => write!(f, "unsupported hardware semantic schema"),
            Self::EmptyField(field) => write!(f, "{field} must not be empty"),
            Self::FieldTooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidIdentifier => write!(f, "invalid semantic identifier"),
            Self::InvalidDigest => write!(f, "invalid digest representation"),
            Self::InvalidMediaType => write!(f, "invalid media type"),
            Self::UnsafeLogicalPath => write!(f, "unsafe logical path"),
            Self::TooManyItems => write!(f, "collection exceeds maximum item count"),
            Self::EmptyCollection(kind) => write!(f, "{kind} must not be empty"),
            Self::Duplicate(kind) => write!(f, "duplicate {kind}"),
            Self::SelfReference(kind) => write!(f, "invalid self-reference in {kind}"),
            Self::ZeroQuantity => write!(f, "quantity must be greater than zero"),
            Self::WrongArtifactRole(kind) => write!(f, "artifact has wrong role for {kind}"),
            Self::EvidenceSubjectMismatch => {
                write!(f, "evidence subject does not match containing revision")
            }
            Self::UnknownClaimReference => {
                write!(f, "evidence claim does not reference a requirement in the revision")
            }
        }
    }
}

impl std::error::Error for ValidationError {}

fn require_schema(value: &str) -> Result<(), ValidationError> {
    if value == HARDWARE_SEMANTIC_SCHEMA {
        Ok(())
    } else {
        Err(ValidationError::UnsupportedSchema)
    }
}

fn validate_identifier(value: &str) -> Result<(), ValidationError> {
    if value.is_empty()
        || value != value.trim()
        || value.len() > MAX_ID_BYTES
        || value.chars().any(char::is_control)
        || value.chars().any(char::is_whitespace)
    {
        return Err(ValidationError::InvalidIdentifier);
    }
    Ok(())
}

fn validate_name(value: &str) -> Result<(), ValidationError> {
    validate_nonempty_bounded(value, MAX_NAME_BYTES, "name")
}

fn validate_optional_description(value: Option<&str>) -> Result<(), ValidationError> {
    if let Some(value) = value {
        validate_nonempty_bounded(value, MAX_DESCRIPTION_BYTES, "description")?;
    }
    Ok(())
}

fn validate_nonempty_bounded(
    value: &str,
    maximum: usize,
    field: &'static str,
) -> Result<(), ValidationError> {
    if value.trim().is_empty() || value != value.trim() || value.chars().any(char::is_control) {
        return Err(ValidationError::EmptyField(field));
    }
    if value.len() > maximum {
        return Err(ValidationError::FieldTooLong(field));
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

fn validate_logical_path(path: &str) -> Result<(), ValidationError> {
    if path.is_empty()
        || path != path.trim()
        || path.len() > 1024
        || path.starts_with('/')
        || path.starts_with('\\')
        || path.contains('\\')
        || path.chars().any(char::is_control)
        || path
            .split('/')
            .any(|component| component.is_empty() || component == "." || component == "..")
    {
        return Err(ValidationError::UnsafeLogicalPath);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> SemanticId {
        SemanticId::new(value).unwrap()
    }

    fn digest(byte: char) -> DigestRef {
        DigestRef::sha256(byte.to_string().repeat(64)).unwrap()
    }

    fn artifact(name: &str, role: ArtifactRole) -> DesignArtifactRef {
        DesignArtifactRef {
            id: id(name),
            role,
            digest: digest('a'),
            byte_length: Some(42),
            media_type: Some("application/octet-stream".into()),
            logical_path: Some(format!("artifacts/{name}")),
        }
    }

    #[test]
    fn semantic_ids_reject_whitespace() {
        assert!(SemanticId::new("project:soil-monitor").is_ok());
        assert_eq!(
            SemanticId::new("project soil-monitor").unwrap_err(),
            ValidationError::InvalidIdentifier
        );
    }

    #[test]
    fn digest_is_lowercase_fixed_width() {
        assert!(DigestRef::sha256("a".repeat(64)).is_ok());
        assert_eq!(
            DigestRef::sha256("A".repeat(64)).unwrap_err(),
            ValidationError::InvalidDigest
        );
    }

    #[test]
    fn composition_rejects_duplicate_entries_and_zero_quantity() {
        let entry = CompositionEntry {
            id: id("entry:r1"),
            subject: CompositionSubject::Component(id("component:r1")),
            quantity: Quantity {
                value: 1,
                unit: QuantityUnit::Each,
            },
            designators: vec!["R1".into()],
            notes: None,
        };
        assert_eq!(
            DesignComposition {
                entries: vec![entry.clone(), entry.clone()]
            }
            .validate()
            .unwrap_err(),
            ValidationError::Duplicate("composition entry")
        );
        let mut zero = entry;
        zero.quantity.value = 0;
        assert_eq!(
            DesignComposition { entries: vec![zero] }
                .validate()
                .unwrap_err(),
            ValidationError::ZeroQuantity
        );
    }

    #[test]
    fn unknown_is_distinct_from_unsatisfied() {
        assert_ne!(ConstraintEvaluation::Unknown, ConstraintEvaluation::Unsatisfied);
        assert_ne!(ConstraintEvaluation::Unknown, ConstraintEvaluation::Satisfied);
    }

    #[test]
    fn datasheet_collection_rejects_wrong_artifact_role() {
        let component = ComponentIdentity {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id("component:mcu"),
            name: "Example MCU".into(),
            kind: ComponentKind::Electronic,
            manufacturer: Some("Example Semiconductor".into()),
            manufacturer_part_number: Some("EX-123".into()),
            open_hardware_project: None,
            datasheet_refs: vec![artifact("wrong", ArtifactRole::NativeCad)],
            lifecycle: LifecycleState::Unknown,
        };
        assert_eq!(
            component.validate().unwrap_err(),
            ValidationError::WrongArtifactRole("datasheet")
        );
    }

    #[test]
    fn evidence_presence_does_not_imply_satisfaction() {
        let evidence = EvidenceReference {
            id: id("evidence:sim:42"),
            subject_id: id("design:bridge:r3"),
            claim_id: Some(id("req:deflection")),
            class: EvidenceClass::Simulation,
            authority: EvidenceAuthority {
                producer: id("solver:example"),
                tool_or_system: Some("ExampleSolver".into()),
                environment_digest: Some(digest('c')),
            },
            artifact_digest: digest('d'),
            profile: Some("load-case:service".into()),
            interpretation: EvidenceInterpretation {
                evaluation: ConstraintEvaluation::Unknown,
                scope: Some("solver result awaits engineering interpretation".into()),
            },
        };
        evidence.validate().unwrap();
        assert_eq!(
            evidence.interpretation.evaluation,
            ConstraintEvaluation::Unknown
        );
    }

    #[test]
    fn serde_round_trip_preserves_semantics() {
        let project = HardwareProject {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id("project:open-microscope"),
            name: "Open Microscope".into(),
            description: Some("An example open scientific instrument".into()),
            maintainers: vec![ActorRef {
                id: id("did:example:maintainer"),
                display_name: Some("Maintainer".into()),
            }],
            license_refs: vec![LicenseRef {
                identifier: "CERN-OHL-S-2.0".into(),
                text_or_url: None,
            }],
        };
        project.validate().unwrap();
        let json = serde_json::to_string(&project).unwrap();
        let restored: HardwareProject = serde_json::from_str(&json).unwrap();
        assert_eq!(project, restored);
    }

    #[test]
    fn logical_paths_reject_traversal() {
        let mut artifact = artifact("pcb", ArtifactRole::PcbLayout);
        artifact.logical_path = Some("../escape.kicad_pcb".into());
        assert_eq!(
            artifact.validate().unwrap_err(),
            ValidationError::UnsafeLogicalPath
        );
    }
}

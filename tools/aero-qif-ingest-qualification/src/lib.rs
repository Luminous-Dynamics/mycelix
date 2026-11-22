//! Evidence envelope for sandboxed QIF 3.0 validation and extraction.
//!
//! This crate verifies declared inputs, validator/check bundles, stage reports,
//! sandbox profile, and extraction outputs. It does not run validators, verify
//! a runner signature, enforce a sandbox, parse QIF XML, or authenticate tools.

use aero_adapter_run_receipt::{ContentArtifactV1, ReceiptError, verify_artifact_bytes};
use aero_evidence_types::ArtifactRef;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const INGEST_SCHEMA: &str = "AeroQifIngestQualificationV1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QifValidationProfile {
    Qif3Normative,
    Qif3PlusPreview,
}

impl QifValidationProfile {
    pub fn is_normative(self) -> bool {
        self == Self::Qif3Normative
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QifDocumentRole {
    Results,
    Plan,
    Auxiliary,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QifDocumentInputV1 {
    pub qpid: String,
    pub role: QifDocumentRole,
    pub artifact: ContentArtifactV1,
    pub source_uri: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalDocumentBindingV1 {
    pub results_qpid: String,
    pub local_reference_id: u32,
    pub target_plan_qpid: String,
    /// Original URI from QIF, retained as source metadata.
    pub source_uri: Option<String>,
    /// Relative path in the sealed, read-only staging root.
    pub staged_relative_path: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ValidatorKind {
    XmlSchema,
    Xslt,
    Extractor,
    TraceResolver,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ValidatorToolV1 {
    pub tool_id: String,
    pub kind: ValidatorKind,
    pub name: String,
    pub version: String,
    pub source_repository: String,
    /// Full 40-hex source commit, not a moving branch/tag.
    pub source_commit_sha: String,
    pub executable: ContentArtifactV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RuntimeSupport {
    SupportedLts,
    UnsupportedLegacy,
    Unverified,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ValidatorRuntimeV1 {
    pub runtime_name: String,
    pub runtime_version: String,
    pub support: RuntimeSupport,
    pub runtime_image: ContentArtifactV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BundleFileV1 {
    pub relative_path: String,
    pub artifact: ContentArtifactV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct XsdBundleV1 {
    pub entrypoint: ContentArtifactV1,
    /// Complete transitive include/import set, including the entrypoint.
    pub files: Vec<BundleFileV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct XsltBundleV1 {
    pub root_stylesheet: ContentArtifactV1,
    /// Complete transitive import/include and helper stylesheet set.
    pub files: Vec<BundleFileV1>,
    pub check_parameters: ContentArtifactV1,
    pub profile: QifValidationProfile,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ValidationBundlesV1 {
    pub xsd: XsdBundleV1,
    pub xslt: XsltBundleV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FilesystemPolicy {
    StagedInputsReadOnlyOutputsWriteOnly,
    Unrestricted,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NetworkPolicy {
    Disabled,
    Allowlisted,
    Enabled,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SandboxProfileV1 {
    pub network: NetworkPolicy,
    pub filesystem: FilesystemPolicy,
    pub rootless: bool,
    pub no_host_secrets: bool,
    pub cpu_limit_seconds: u64,
    pub memory_limit_bytes: u64,
    pub wall_time_limit_ms: u64,
    pub max_document_bytes: u64,
    pub max_total_input_bytes: u64,
    pub max_document_count: u32,
    pub max_link_recursion_depth: u32,
    pub policy_artifact: ContentArtifactV1,
    /// A reference only; this crate does not authenticate the attestation.
    pub runner_attestation: ContentArtifactV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ValidationStage {
    XmlWellFormedness,
    XsdStructure,
    XsltFormat,
    XsltQuality,
    XsltSemantic,
    ExtractionFidelity,
    ExternalReferenceResolution,
}

impl ValidationStage {
    fn key(self) -> &'static str {
        match self {
            Self::XmlWellFormedness => "xml_well_formedness",
            Self::XsdStructure => "xsd_structure",
            Self::XsltFormat => "xslt_format",
            Self::XsltQuality => "xslt_quality",
            Self::XsltSemantic => "xslt_semantic",
            Self::ExtractionFidelity => "extraction_fidelity",
            Self::ExternalReferenceResolution => "external_reference_resolution",
        }
    }

    fn expected_tool(self) -> ValidatorKind {
        match self {
            Self::XmlWellFormedness | Self::XsdStructure => ValidatorKind::XmlSchema,
            Self::XsltFormat | Self::XsltQuality | Self::XsltSemantic => ValidatorKind::Xslt,
            Self::ExtractionFidelity => ValidatorKind::Extractor,
            Self::ExternalReferenceResolution => ValidatorKind::TraceResolver,
        }
    }

    fn all() -> [Self; 7] {
        [
            Self::XmlWellFormedness,
            Self::XsdStructure,
            Self::XsltFormat,
            Self::XsltQuality,
            Self::XsltSemantic,
            Self::ExtractionFidelity,
            Self::ExternalReferenceResolution,
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StageDisposition {
    Passed,
    Failed,
    Warning,
    Unknown,
    NotRun,
    Blocked,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StageResultV1 {
    pub stage: ValidationStage,
    pub disposition: StageDisposition,
    pub validator_tool_id: Option<String>,
    pub report_artifact: Option<ContentArtifactV1>,
    pub reason_codes: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtractionMappingCountsV1 {
    pub total_required: u64,
    pub mapped: u64,
    pub unresolved: u64,
    pub ambiguous: u64,
    pub unsupported: u64,
    pub dropped: u64,
}

pub const SOURCE_MAPPING_SCHEMA: &str = "AeroQifSourceMappingManifestV1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceMappingDisposition {
    Mapped,
    Unresolved,
    Ambiguous,
    Unsupported,
    Dropped,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceFieldLocatorV1 {
    pub document_qpid: String,
    pub source_path: String,
    pub source_element_id: Option<u32>,
    /// Local QIF target reference carried as simple content when xId is absent.
    pub local_reference_id: Option<u32>,
    /// Parent ExternalQIFDocument/@id for QPId/URI metadata fields inside that element.
    pub parent_external_document_reference_id: Option<u32>,
    pub external_document_reference_id: Option<u32>,
    pub external_xid: Option<u32>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceMappingRecordV1 {
    /// QPId of the raw QIF document containing the source field.
    pub document_qpid: String,
    /// Namespace-qualified, stable source locator (not a transient line number).
    pub source_path: String,
    /// Local QIF element ID when the source element has an @id.
    pub source_element_id: Option<u32>,
    /// Local QIF target reference carried as simple content when xId is absent.
    pub local_reference_id: Option<u32>,
    /// Parent ExternalQIFDocument/@id for QPId/URI metadata fields inside that element.
    pub parent_external_document_reference_id: Option<u32>,
    /// Results-local ExternalQIFDocumentReference ID; paired with xId only for external references.
    pub external_document_reference_id: Option<u32>,
    /// QIFReferenceType/@xId in the selected external QIF document.
    pub external_xid: Option<u32>,
    pub mapping_rule_id: String,
    pub mapping_profile_version: String,
    pub disposition: SourceMappingDisposition,
    pub target_object_kind: Option<String>,
    pub target_object_id: Option<String>,
    pub reason_code: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceMappingManifestV1 {
    pub schema: String,
    /// Expected source fields, frozen independently from the mapping outcomes.
    pub required_locators: Vec<SourceFieldLocatorV1>,
    pub mappings: Vec<SourceMappingRecordV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtractionEvidenceV1 {
    pub normalized_trace_artifact: ContentArtifactV1,
    pub source_mapping_manifest: ContentArtifactV1,
    pub required_locators: Vec<SourceFieldLocatorV1>,
    pub mappings: Vec<SourceMappingRecordV1>,
    pub mapping_counts: ExtractionMappingCountsV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QifIngestQualificationBodyV1 {
    pub qualification_id: String,
    pub benchmark_case_id: String,
    pub qif_standard_version: String,
    pub qif_profile: QifValidationProfile,
    pub documents: Vec<QifDocumentInputV1>,
    pub external_document_bindings: Vec<ExternalDocumentBindingV1>,
    pub bundles: ValidationBundlesV1,
    pub runtime: ValidatorRuntimeV1,
    pub tools: Vec<ValidatorToolV1>,
    pub sandbox: SandboxProfileV1,
    /// All seven stages appear exactly once, including NotRun.
    pub stage_results: Vec<StageResultV1>,
    pub extraction: Option<ExtractionEvidenceV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QifIngestQualificationV1 {
    pub schema: String,
    pub body: QifIngestQualificationBodyV1,
    pub receipt_sha256: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IngestDisposition {
    IntegrityChecked,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IngestVerificationReport {
    pub disposition: IngestDisposition,
    pub qualification_sha256: String,
    pub checked_artifacts: usize,
    pub profile_is_normative: bool,
    pub legacy_runtime: bool,
    pub all_stages_passed: bool,
    pub sandbox_profile_declared: bool,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum IngestError {
    #[error("invalid qualification JSON: {0}")]
    InvalidJson(String),
    #[error("wrong ingest schema")]
    WrongSchema,
    #[error("empty identity at {0}")]
    InvalidIdentity(String),
    #[error("invalid QPId at {0}")]
    InvalidQpid(String),
    #[error("duplicate QPId {0}")]
    DuplicateQpid(String),
    #[error("Results document missing or ambiguous")]
    ResultsDocumentCardinality,
    #[error("Plan document {0} is not linked from Results")]
    UnlinkedPlan(String),
    #[error("external document binding has no matching Results document: {0}")]
    MissingResultsDocument(String),
    #[error("external document binding has no matching Plan document: {0}")]
    MissingPlanDocument(String),
    #[error("external document binding ID is zero or duplicated: {0}")]
    DuplicateExternalReference(u32),
    #[error("staged path is unsafe: {0}")]
    UnsafeStagedPath(String),
    #[error("staged paths must be unique: {0}")]
    DuplicateStagedPath(String),
    #[error("invalid bundle path: {0}")]
    InvalidBundlePath(String),
    #[error("duplicate bundle path: {0}")]
    DuplicateBundlePath(String),
    #[error("bundle entrypoint is not in its pinned file manifest")]
    MissingBundleEntrypoint,
    #[error("XSLT bundle profile differs from the qualification profile")]
    BundleProfileMismatch,
    #[error("validator tool set is incomplete or duplicated")]
    ValidatorToolCardinality,
    #[error("invalid validator source commit for {0}")]
    InvalidToolCommit(String),
    #[error("stage result set is incomplete or duplicated")]
    StageSetMismatch,
    #[error("stage {0} has no validator tool or report artifact")]
    MissingStageEvidence(String),
    #[error("stage {stage} references tool {tool_id} of the wrong kind")]
    WrongStageToolKind { stage: String, tool_id: String },
    #[error("stage {0} has evidence despite being NotRun")]
    NotRunHasEvidence(String),
    #[error("sandbox profile is not fail-closed")]
    UnsafeSandboxProfile,
    #[error("sandbox resource limit is zero or invalid")]
    InvalidSandboxLimit,
    #[error("source document exceeds maximum single-document byte limit: {0}")]
    DocumentExceedsLimit(String),
    #[error("source documents exceed the total staged-input byte limit")]
    TotalInputExceedsLimit,
    #[error("source document count exceeds sandbox policy")]
    TooManyDocuments,
    #[error("runtime or tool artifact is invalid: {0}")]
    InvalidArtifact(String),
    #[error("unsupported .NET Core 3.1 runtime must be explicitly marked legacy")]
    LegacyRuntimeMisclassified,
    #[error("source URI is empty or contains control characters: {0}")]
    InvalidSourceUri(String),
    #[error("blocked stage {0} has evidence or no reason code")]
    InvalidBlockedStage(String),
    #[error("stage {stage} passed while prerequisite {prerequisite} did not pass")]
    PrerequisiteNotPassed { stage: String, prerequisite: String },
    #[error("Plan document source URI conflicts with its external reference: {0}")]
    SourceUriMismatch(String),
    #[error("extraction evidence is missing or inconsistent")]
    InvalidExtractionEvidence,
    #[error("source mapping locator is invalid: {0}")]
    InvalidSourceMappingLocator(String),
    #[error("source mapping locator is duplicated: {0}")]
    DuplicateSourceMappingLocator(String),
    #[error("source mapping disposition/target/reason fields are inconsistent: {0}")]
    InvalidSourceMappingRecord(String),
    #[error("source mapping records do not match the declared disposition counts")]
    SourceMappingCountMismatch,
    #[error("mapping rows do not exactly cover the frozen required source locator inventory")]
    SourceMappingInventoryMismatch,
    #[error("source mapping records do not match the pinned manifest artifact bytes")]
    SourceMappingManifestMismatch,
    #[error(
        "extraction stage Passed despite unresolved, ambiguous, unsupported or dropped mappings"
    )]
    FalseExtractionPass,
    #[error("extraction stage has dropped mappings")]
    DroppedMappings,
    #[error("invalid receipt SHA-256")]
    InvalidReceiptDigest,
    #[error("receipt SHA-256 does not match the canonical body")]
    ReceiptDigestMismatch,
    #[error("conflicting metadata for artifact {0}")]
    ArtifactMetadataConflict(String),
    #[error("artifact payload set has missing, duplicate or unexpected entries")]
    ArtifactPayloadSetMismatch,
    #[error("duplicate artifact payload {0}")]
    DuplicateArtifactPayload(String),
    #[error("unexpected artifact payload {0}")]
    UnexpectedArtifactPayload(String),
    #[error("artifact byte verification failed: {0}")]
    ArtifactVerification(ReceiptError),
    #[error("canonical JSON serialization failed")]
    CanonicalizationFailed,
}

pub fn parse_ingest_qualification_json(
    bytes: &[u8],
) -> Result<QifIngestQualificationV1, IngestError> {
    let qualification: QifIngestQualificationV1 = serde_json::from_slice(bytes)
        .map_err(|error| IngestError::InvalidJson(error.to_string()))?;
    validate_ingest_qualification(&qualification)?;
    Ok(qualification)
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn valid_uuid(value: &str) -> bool {
    if value.len() != 36 {
        return false;
    }
    value.bytes().enumerate().all(|(index, byte)| {
        if matches!(index, 8 | 13 | 18 | 23) {
            byte == b'-'
        } else {
            byte.is_ascii_hexdigit()
        }
    })
}

fn validate_content_artifact(artifact: &ContentArtifactV1) -> Result<(), IngestError> {
    if artifact.reference.id.trim().is_empty()
        || artifact.reference.kind.trim().is_empty()
        || !valid_sha256(&artifact.sha256)
        || artifact.reference.id != format!("sha256:{}", artifact.sha256)
        || artifact.byte_length == 0
        || artifact.media_type.trim().is_empty()
        || artifact.format_id.trim().is_empty()
    {
        return Err(IngestError::InvalidArtifact(artifact.reference.id.clone()));
    }
    Ok(())
}

fn valid_source_uri(value: &str) -> bool {
    !value.trim().is_empty() && !value.chars().any(|character| character.is_control())
}

fn safe_relative_path(value: &str) -> bool {
    if value.trim().is_empty()
        || value.starts_with('/')
        || value.starts_with('\\')
        || value.contains('\\')
        || value.contains(':')
        || value.contains('%')
        || value.chars().any(|character| character.is_control())
    {
        return false;
    }
    value.split('/').all(|part| {
        !part.is_empty()
            && part != "."
            && part != ".."
            && part
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
    })
}

fn add_artifact(
    artifacts: &mut BTreeMap<(String, String), ContentArtifactV1>,
    artifact: &ContentArtifactV1,
) -> Result<(), IngestError> {
    validate_content_artifact(artifact)?;
    let key = (
        artifact.reference.id.clone(),
        artifact.reference.kind.clone(),
    );
    if let Some(existing) = artifacts.get(&key) {
        if existing != artifact {
            return Err(IngestError::ArtifactMetadataConflict(key.0));
        }
    } else {
        artifacts.insert(key, artifact.clone());
    }
    Ok(())
}

fn canonical_body(body: &QifIngestQualificationBodyV1) -> QifIngestQualificationBodyV1 {
    let mut canonical = body.clone();
    canonical.documents.sort_by(|left, right| {
        left.qpid
            .to_ascii_lowercase()
            .cmp(&right.qpid.to_ascii_lowercase())
    });
    canonical.external_document_bindings.sort_by(|left, right| {
        (
            left.results_qpid.to_ascii_lowercase(),
            left.local_reference_id,
        )
            .cmp(&(
                right.results_qpid.to_ascii_lowercase(),
                right.local_reference_id,
            ))
    });
    canonical
        .bundles
        .xsd
        .files
        .sort_by(|left, right| left.relative_path.cmp(&right.relative_path));
    canonical
        .bundles
        .xslt
        .files
        .sort_by(|left, right| left.relative_path.cmp(&right.relative_path));
    canonical
        .tools
        .sort_by(|left, right| left.tool_id.cmp(&right.tool_id));
    canonical
        .stage_results
        .sort_by(|left, right| left.stage.key().cmp(right.stage.key()));
    if let Some(extraction) = &mut canonical.extraction {
        extraction.required_locators =
            canonical_source_field_locators(&extraction.required_locators);
        extraction.mappings = canonical_source_mapping_records(&extraction.mappings);
    }
    for stage in &mut canonical.stage_results {
        stage.reason_codes.sort();
    }
    canonical
}

fn canonical_source_field_locators(locators: &[SourceFieldLocatorV1]) -> Vec<SourceFieldLocatorV1> {
    let mut canonical = locators.to_vec();
    for locator in &mut canonical {
        locator.document_qpid = locator.document_qpid.to_ascii_lowercase();
    }
    canonical.sort();
    canonical
}

fn source_locator_from_mapping(record: &SourceMappingRecordV1) -> SourceFieldLocatorV1 {
    SourceFieldLocatorV1 {
        document_qpid: record.document_qpid.to_ascii_lowercase(),
        source_path: record.source_path.clone(),
        source_element_id: record.source_element_id,
        local_reference_id: record.local_reference_id,
        parent_external_document_reference_id: record.parent_external_document_reference_id,
        external_document_reference_id: record.external_document_reference_id,
        external_xid: record.external_xid,
    }
}

fn canonical_source_mapping_records(
    records: &[SourceMappingRecordV1],
) -> Vec<SourceMappingRecordV1> {
    let mut canonical = records.to_vec();
    canonical.sort_by(|left, right| {
        (
            left.document_qpid.to_ascii_lowercase(),
            left.source_path.clone(),
            left.source_element_id,
            left.local_reference_id,
            left.parent_external_document_reference_id,
            left.external_document_reference_id,
            left.external_xid,
        )
            .cmp(&(
                right.document_qpid.to_ascii_lowercase(),
                right.source_path.clone(),
                right.source_element_id,
                right.local_reference_id,
                right.parent_external_document_reference_id,
                right.external_document_reference_id,
                right.external_xid,
            ))
    });
    canonical
}

/// Canonical serialized source-mapping manifest bytes (not RFC 8785/JCS).
pub fn source_mapping_manifest_bytes(
    required_locators: &[SourceFieldLocatorV1],
    records: &[SourceMappingRecordV1],
) -> Result<Vec<u8>, IngestError> {
    let manifest = SourceMappingManifestV1 {
        schema: SOURCE_MAPPING_SCHEMA.into(),
        required_locators: canonical_source_field_locators(required_locators),
        mappings: canonical_source_mapping_records(records),
    };
    serde_json::to_vec(&manifest).map_err(|_| IngestError::CanonicalizationFailed)
}

fn digest_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn validate_source_field_locator(
    locator: &SourceFieldLocatorV1,
    documents: &BTreeMap<String, &QifDocumentInputV1>,
    reference_ids: &BTreeSet<(String, u32)>,
) -> Result<SourceFieldLocatorV1, IngestError> {
    let qpid = locator.document_qpid.to_ascii_lowercase();
    let Some(document) = documents.get(&qpid) else {
        return Err(IngestError::InvalidSourceMappingLocator(
            locator.document_qpid.clone(),
        ));
    };
    if !valid_uuid(&locator.document_qpid)
        || !locator.source_path.starts_with("/QIFDocument/")
        || locator
            .source_path
            .chars()
            .any(|character| character.is_control())
        || locator.source_path.trim().is_empty()
        || locator.source_element_id == Some(0)
        || locator.local_reference_id == Some(0)
        || locator.parent_external_document_reference_id == Some(0)
        || locator.external_document_reference_id.is_some() != locator.external_xid.is_some()
        || locator.external_document_reference_id.is_some() && locator.local_reference_id.is_some()
        || locator.parent_external_document_reference_id.is_some()
            && (locator.local_reference_id.is_some()
                || locator.external_document_reference_id.is_some()
                || locator.external_xid.is_some())
        || locator.external_document_reference_id == Some(0)
        || locator.external_xid == Some(0)
    {
        return Err(IngestError::InvalidSourceMappingLocator(
            locator.source_path.clone(),
        ));
    }
    if let (Some(reference_id), Some(_xid)) =
        (locator.external_document_reference_id, locator.external_xid)
        && (document.role != QifDocumentRole::Results
            || !reference_ids.contains(&(qpid.clone(), reference_id)))
    {
        return Err(IngestError::InvalidSourceMappingLocator(
            locator.source_path.clone(),
        ));
    }
    if let Some(parent_reference_id) = locator.parent_external_document_reference_id
        && (document.role != QifDocumentRole::Results
            || !reference_ids.contains(&(qpid.clone(), parent_reference_id)))
    {
        return Err(IngestError::InvalidSourceMappingLocator(
            locator.source_path.clone(),
        ));
    }
    Ok(SourceFieldLocatorV1 {
        document_qpid: qpid,
        source_path: locator.source_path.clone(),
        source_element_id: locator.source_element_id,
        local_reference_id: locator.local_reference_id,
        parent_external_document_reference_id: locator.parent_external_document_reference_id,
        external_document_reference_id: locator.external_document_reference_id,
        external_xid: locator.external_xid,
    })
}

fn validate_source_mapping_records(
    required_locators: &[SourceFieldLocatorV1],
    records: &[SourceMappingRecordV1],
    counts: &ExtractionMappingCountsV1,
    documents: &BTreeMap<String, &QifDocumentInputV1>,
    reference_ids: &BTreeSet<(String, u32)>,
) -> Result<(), IngestError> {
    if required_locators.is_empty() || required_locators.len() as u64 != counts.total_required {
        return Err(IngestError::SourceMappingCountMismatch);
    }

    let mut expected = BTreeSet::new();
    for locator in required_locators {
        let canonical = validate_source_field_locator(locator, documents, reference_ids)?;
        if !expected.insert(canonical) {
            return Err(IngestError::DuplicateSourceMappingLocator(
                locator.source_path.clone(),
            ));
        }
    }

    if records.len() != expected.len() {
        return Err(IngestError::SourceMappingInventoryMismatch);
    }

    let mut observed = BTreeSet::new();
    let (mut mapped, mut unresolved, mut ambiguous, mut unsupported, mut dropped) =
        (0u64, 0u64, 0u64, 0u64, 0u64);

    for record in records {
        let locator = source_locator_from_mapping(record);
        let canonical = validate_source_field_locator(&locator, documents, reference_ids)?;
        if !expected.contains(&canonical) {
            return Err(IngestError::SourceMappingInventoryMismatch);
        }
        if !observed.insert(canonical) {
            return Err(IngestError::DuplicateSourceMappingLocator(
                record.source_path.clone(),
            ));
        }
        if record.mapping_rule_id.trim().is_empty()
            || record.mapping_profile_version.trim().is_empty()
        {
            return Err(IngestError::InvalidSourceMappingRecord(
                record.source_path.clone(),
            ));
        }

        match record.disposition {
            SourceMappingDisposition::Mapped => {
                mapped += 1;
                if record
                    .target_object_kind
                    .as_ref()
                    .is_none_or(|value| value.trim().is_empty())
                    || record
                        .target_object_id
                        .as_ref()
                        .is_none_or(|value| value.trim().is_empty())
                    || record.reason_code.is_some()
                {
                    return Err(IngestError::InvalidSourceMappingRecord(
                        record.source_path.clone(),
                    ));
                }
            }
            SourceMappingDisposition::Unresolved
            | SourceMappingDisposition::Ambiguous
            | SourceMappingDisposition::Unsupported
            | SourceMappingDisposition::Dropped => {
                match record.disposition {
                    SourceMappingDisposition::Unresolved => unresolved += 1,
                    SourceMappingDisposition::Ambiguous => ambiguous += 1,
                    SourceMappingDisposition::Unsupported => unsupported += 1,
                    SourceMappingDisposition::Dropped => dropped += 1,
                    SourceMappingDisposition::Mapped => unreachable!(),
                }
                if record.target_object_kind.is_some()
                    || record.target_object_id.is_some()
                    || record
                        .reason_code
                        .as_ref()
                        .is_none_or(|value| value.trim().is_empty())
                {
                    return Err(IngestError::InvalidSourceMappingRecord(
                        record.source_path.clone(),
                    ));
                }
            }
        }
    }

    if observed != expected {
        return Err(IngestError::SourceMappingInventoryMismatch);
    }
    if mapped != counts.mapped
        || unresolved != counts.unresolved
        || ambiguous != counts.ambiguous
        || unsupported != counts.unsupported
        || dropped != counts.dropped
    {
        return Err(IngestError::SourceMappingCountMismatch);
    }
    Ok(())
}

fn validate_bundle_files(
    entrypoint: &ContentArtifactV1,
    files: &[BundleFileV1],
) -> Result<(), IngestError> {
    validate_content_artifact(entrypoint)?;
    let mut paths = BTreeSet::new();
    let mut entry_found = false;
    for file in files {
        if !safe_relative_path(&file.relative_path) {
            return Err(IngestError::InvalidBundlePath(file.relative_path.clone()));
        }
        if !paths.insert(file.relative_path.clone()) {
            return Err(IngestError::DuplicateBundlePath(file.relative_path.clone()));
        }
        validate_content_artifact(&file.artifact)?;
        if file.artifact == *entrypoint {
            entry_found = true;
        }
    }
    if !entry_found {
        return Err(IngestError::MissingBundleEntrypoint);
    }
    Ok(())
}

pub fn validate_ingest_qualification(
    qualification: &QifIngestQualificationV1,
) -> Result<(), IngestError> {
    if qualification.schema != INGEST_SCHEMA {
        return Err(IngestError::WrongSchema);
    }
    if qualification.body.qualification_id.trim().is_empty()
        || qualification.body.benchmark_case_id.trim().is_empty()
        || qualification.body.qif_standard_version != "3.0.0"
    {
        return Err(IngestError::InvalidIdentity(
            "qualification_id/benchmark_case_id".into(),
        ));
    }

    let body = &qualification.body;
    if body.documents.is_empty() {
        return Err(IngestError::ResultsDocumentCardinality);
    }

    if body.documents.len() > body.sandbox.max_document_count as usize {
        return Err(IngestError::TooManyDocuments);
    }
    let mut total_source_bytes = 0u64;
    let mut documents = BTreeMap::new();
    let mut results_docs = Vec::new();
    let mut plan_docs = BTreeSet::new();
    for document in &body.documents {
        if document
            .source_uri
            .as_ref()
            .is_some_and(|uri| !valid_source_uri(uri))
        {
            return Err(IngestError::InvalidSourceUri(
                document.source_uri.clone().unwrap_or_default(),
            ));
        }
        if !valid_uuid(&document.qpid) {
            return Err(IngestError::InvalidQpid(document.qpid.clone()));
        }
        let key = document.qpid.to_ascii_lowercase();
        if documents.insert(key.clone(), document).is_some() {
            return Err(IngestError::DuplicateQpid(document.qpid.clone()));
        }
        validate_content_artifact(&document.artifact)?;
        if document.artifact.byte_length > body.sandbox.max_document_bytes {
            return Err(IngestError::DocumentExceedsLimit(document.qpid.clone()));
        }
        total_source_bytes = total_source_bytes
            .checked_add(document.artifact.byte_length)
            .ok_or(IngestError::TotalInputExceedsLimit)?;
        match document.role {
            QifDocumentRole::Results => results_docs.push(key),
            QifDocumentRole::Plan => {
                plan_docs.insert(key);
            }
            QifDocumentRole::Auxiliary => {}
        }
    }
    if results_docs.len() != 1 {
        return Err(IngestError::ResultsDocumentCardinality);
    }
    let results_qpid = results_docs[0].clone();

    let mut reference_ids = BTreeSet::new();
    let mut staged_paths = BTreeMap::new();
    let mut linked_plans = BTreeSet::new();
    for binding in &body.external_document_bindings {
        if binding.local_reference_id == 0
            || !reference_ids.insert((
                binding.results_qpid.to_ascii_lowercase(),
                binding.local_reference_id,
            ))
        {
            return Err(IngestError::DuplicateExternalReference(
                binding.local_reference_id,
            ));
        }
        if !binding.results_qpid.eq_ignore_ascii_case(&results_qpid) {
            return Err(IngestError::MissingResultsDocument(
                binding.results_qpid.clone(),
            ));
        }
        let Some(plan) = documents.get(&binding.target_plan_qpid.to_ascii_lowercase()) else {
            return Err(IngestError::MissingPlanDocument(
                binding.target_plan_qpid.clone(),
            ));
        };
        if plan.role != QifDocumentRole::Plan {
            return Err(IngestError::MissingPlanDocument(
                binding.target_plan_qpid.clone(),
            ));
        }
        if binding
            .source_uri
            .as_ref()
            .is_some_and(|uri| !valid_source_uri(uri))
        {
            return Err(IngestError::InvalidSourceUri(
                binding.source_uri.clone().unwrap_or_default(),
            ));
        }
        if !safe_relative_path(&binding.staged_relative_path) {
            return Err(IngestError::UnsafeStagedPath(
                binding.staged_relative_path.clone(),
            ));
        }
        if let Some(existing) = staged_paths.get(&binding.staged_relative_path) {
            if existing != &binding.target_plan_qpid.to_ascii_lowercase() {
                return Err(IngestError::DuplicateStagedPath(
                    binding.staged_relative_path.clone(),
                ));
            }
        } else {
            staged_paths.insert(
                binding.staged_relative_path.clone(),
                binding.target_plan_qpid.to_ascii_lowercase(),
            );
        }
        if plan.source_uri != binding.source_uri {
            return Err(IngestError::SourceUriMismatch(
                binding.target_plan_qpid.clone(),
            ));
        }
        linked_plans.insert(binding.target_plan_qpid.to_ascii_lowercase());
    }
    if linked_plans != plan_docs {
        let unlinked = plan_docs
            .difference(&linked_plans)
            .next()
            .cloned()
            .unwrap_or_default();
        return Err(IngestError::UnlinkedPlan(unlinked));
    }

    validate_bundle_files(&body.bundles.xsd.entrypoint, &body.bundles.xsd.files)?;
    validate_bundle_files(&body.bundles.xslt.root_stylesheet, &body.bundles.xslt.files)?;
    validate_content_artifact(&body.bundles.xslt.check_parameters)?;
    let bundle_bytes = body
        .bundles
        .xsd
        .files
        .iter()
        .chain(body.bundles.xslt.files.iter())
        .try_fold(
            body.bundles.xslt.check_parameters.byte_length,
            |total, file| total.checked_add(file.artifact.byte_length),
        )
        .ok_or(IngestError::TotalInputExceedsLimit)?;
    let total_staged_bytes = total_source_bytes
        .checked_add(bundle_bytes)
        .ok_or(IngestError::TotalInputExceedsLimit)?;
    if total_staged_bytes > body.sandbox.max_total_input_bytes {
        return Err(IngestError::TotalInputExceedsLimit);
    }
    let expected_xslt_format = match body.qif_profile {
        QifValidationProfile::Qif3Normative => "QIF_3.0_XSLT",
        QifValidationProfile::Qif3PlusPreview => "QIF3PLUS_PREVIEW_XSLT",
    };
    if body.bundles.xsd.entrypoint.format_id != "QIF_3.0_XSD"
        || body.bundles.xslt.root_stylesheet.format_id != expected_xslt_format
        || body.bundles.xslt.check_parameters.format_id != "QIF_CHECK_PARAMETERS_V1"
    {
        return Err(IngestError::BundleProfileMismatch);
    }
    if body.bundles.xslt.profile != body.qif_profile {
        return Err(IngestError::BundleProfileMismatch);
    }

    validate_content_artifact(&body.runtime.runtime_image)?;
    if body.runtime.runtime_name.trim().is_empty() || body.runtime.runtime_version.trim().is_empty()
    {
        return Err(IngestError::InvalidIdentity("runtime".into()));
    }
    if body.runtime.runtime_name.eq_ignore_ascii_case("dotnet")
        && body.runtime.runtime_version.starts_with("3.1")
        && body.runtime.support != RuntimeSupport::UnsupportedLegacy
    {
        return Err(IngestError::LegacyRuntimeMisclassified);
    }

    let mut tools = BTreeMap::new();
    for tool in &body.tools {
        if tool.tool_id.trim().is_empty()
            || tool.name.trim().is_empty()
            || tool.version.trim().is_empty()
            || tool.source_repository.trim().is_empty()
            || tool.source_commit_sha.len() != 40
            || !tool
                .source_commit_sha
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
        {
            return Err(IngestError::InvalidToolCommit(tool.tool_id.clone()));
        }
        validate_content_artifact(&tool.executable)?;
        if tools.insert(tool.tool_id.clone(), tool).is_some() {
            return Err(IngestError::ValidatorToolCardinality);
        }
    }
    for kind in [
        ValidatorKind::XmlSchema,
        ValidatorKind::Xslt,
        ValidatorKind::Extractor,
        ValidatorKind::TraceResolver,
    ] {
        if tools.values().filter(|tool| tool.kind == kind).count() != 1 {
            return Err(IngestError::ValidatorToolCardinality);
        }
    }

    if body.sandbox.network != NetworkPolicy::Disabled
        || body.sandbox.filesystem != FilesystemPolicy::StagedInputsReadOnlyOutputsWriteOnly
        || !body.sandbox.rootless
        || !body.sandbox.no_host_secrets
    {
        return Err(IngestError::UnsafeSandboxProfile);
    }
    if body.sandbox.cpu_limit_seconds == 0
        || body.sandbox.cpu_limit_seconds > 600
        || body.sandbox.memory_limit_bytes == 0
        || body.sandbox.memory_limit_bytes > 8 * 1024 * 1024 * 1024
        || body.sandbox.wall_time_limit_ms == 0
        || body.sandbox.wall_time_limit_ms > 600_000
        || body.sandbox.max_document_bytes == 0
        || body.sandbox.max_document_bytes > 100 * 1024 * 1024
        || body.sandbox.max_total_input_bytes == 0
        || body.sandbox.max_total_input_bytes > 1024 * 1024 * 1024
        || body.sandbox.max_document_count == 0
        || body.sandbox.max_document_count > 128
        || body.sandbox.max_link_recursion_depth == 0
        || body.sandbox.max_link_recursion_depth > 8
    {
        return Err(IngestError::InvalidSandboxLimit);
    }
    validate_content_artifact(&body.sandbox.policy_artifact)?;
    validate_content_artifact(&body.sandbox.runner_attestation)?;

    if body.stage_results.len() != ValidationStage::all().len() {
        return Err(IngestError::StageSetMismatch);
    }
    let mut stages = BTreeMap::new();
    for stage in &body.stage_results {
        if stages.insert(stage.stage.key(), stage).is_some() {
            return Err(IngestError::StageSetMismatch);
        }
        match stage.disposition {
            StageDisposition::NotRun => {
                if stage.validator_tool_id.is_some() || stage.report_artifact.is_some() {
                    return Err(IngestError::NotRunHasEvidence(stage.stage.key().into()));
                }
            }
            StageDisposition::Blocked => {
                if stage.validator_tool_id.is_some()
                    || stage.report_artifact.is_some()
                    || stage.reason_codes.is_empty()
                {
                    return Err(IngestError::InvalidBlockedStage(stage.stage.key().into()));
                }
            }
            _ => {
                let Some(tool_id) = &stage.validator_tool_id else {
                    return Err(IngestError::MissingStageEvidence(stage.stage.key().into()));
                };
                let Some(tool) = tools.get(tool_id) else {
                    return Err(IngestError::MissingStageEvidence(stage.stage.key().into()));
                };
                if tool.kind != stage.stage.expected_tool() {
                    return Err(IngestError::WrongStageToolKind {
                        stage: stage.stage.key().into(),
                        tool_id: tool_id.clone(),
                    });
                }
                let Some(report) = &stage.report_artifact else {
                    return Err(IngestError::MissingStageEvidence(stage.stage.key().into()));
                };
                validate_content_artifact(report)?;
            }
        }
        if matches!(
            stage.disposition,
            StageDisposition::Failed | StageDisposition::Warning | StageDisposition::Unknown
        ) && stage.reason_codes.is_empty()
        {
            return Err(IngestError::InvalidIdentity(format!(
                "missing reason_codes:{}",
                stage.stage.key()
            )));
        }
        if stage.reason_codes.iter().any(|code| code.trim().is_empty()) {
            return Err(IngestError::InvalidIdentity(format!(
                "reason_codes:{}",
                stage.stage.key()
            )));
        }
    }
    for stage in ValidationStage::all() {
        if !stages.contains_key(stage.key()) {
            return Err(IngestError::StageSetMismatch);
        }
    }

    // A dropped source-semantic mapping is a primary extraction failure.
    // Report it before downstream prerequisite failures so the loss cannot be
    // hidden behind a secondary blocked-stage disposition.
    if body
        .extraction
        .as_ref()
        .is_some_and(|extraction| extraction.mapping_counts.dropped > 0)
    {
        return Err(IngestError::DroppedMappings);
    }

    let prerequisite_pairs = [
        (
            ValidationStage::XsdStructure,
            ValidationStage::XmlWellFormedness,
        ),
        (ValidationStage::XsltFormat, ValidationStage::XsdStructure),
        (ValidationStage::XsltQuality, ValidationStage::XsdStructure),
        (ValidationStage::XsltQuality, ValidationStage::XsltFormat),
        (ValidationStage::XsltSemantic, ValidationStage::XsdStructure),
        (ValidationStage::XsltSemantic, ValidationStage::XsltFormat),
        (
            ValidationStage::ExtractionFidelity,
            ValidationStage::XsltQuality,
        ),
        (
            ValidationStage::ExtractionFidelity,
            ValidationStage::XsltSemantic,
        ),
        (
            ValidationStage::ExternalReferenceResolution,
            ValidationStage::ExtractionFidelity,
        ),
    ];
    for (stage, prerequisite) in prerequisite_pairs {
        if stages[stage.key()].disposition == StageDisposition::Passed
            && stages[prerequisite.key()].disposition != StageDisposition::Passed
        {
            return Err(IngestError::PrerequisiteNotPassed {
                stage: stage.key().into(),
                prerequisite: prerequisite.key().into(),
            });
        }
    }

    match (
        stages[ValidationStage::ExtractionFidelity.key()].disposition,
        &body.extraction,
    ) {
        (StageDisposition::NotRun, None) => {}
        (StageDisposition::NotRun, Some(_)) | (_, None) => {
            return Err(IngestError::InvalidExtractionEvidence);
        }
        (disposition, Some(extraction)) => {
            validate_content_artifact(&extraction.normalized_trace_artifact)?;
            validate_content_artifact(&extraction.source_mapping_manifest)?;
            if extraction.source_mapping_manifest.reference.kind != "source_mapping"
                || extraction.source_mapping_manifest.format_id != "AERO_QIF_SOURCE_MAPPING_V1"
                || extraction.normalized_trace_artifact.reference.kind != "normalized_qif_trace"
            {
                return Err(IngestError::InvalidExtractionEvidence);
            }
            let counts = &extraction.mapping_counts;
            validate_source_mapping_records(
                &extraction.required_locators,
                &extraction.mappings,
                counts,
                &documents,
                &reference_ids,
            )?;
            let canonical_mapping_bytes =
                source_mapping_manifest_bytes(&extraction.required_locators, &extraction.mappings)?;
            if extraction.source_mapping_manifest.byte_length
                != canonical_mapping_bytes.len() as u64
                || extraction.source_mapping_manifest.sha256 != digest_hex(&canonical_mapping_bytes)
            {
                return Err(IngestError::SourceMappingManifestMismatch);
            }
            let accounted = counts
                .mapped
                .checked_add(counts.unresolved)
                .and_then(|value| value.checked_add(counts.ambiguous))
                .and_then(|value| value.checked_add(counts.unsupported))
                .and_then(|value| value.checked_add(counts.dropped))
                .ok_or(IngestError::InvalidExtractionEvidence)?;
            if counts.total_required == 0 || accounted != counts.total_required {
                return Err(IngestError::InvalidExtractionEvidence);
            }
            if counts.dropped > 0 && disposition != StageDisposition::Failed {
                return Err(IngestError::DroppedMappings);
            }
            if disposition == StageDisposition::Passed
                && (counts.mapped != counts.total_required
                    || counts.unresolved > 0
                    || counts.ambiguous > 0
                    || counts.unsupported > 0
                    || counts.dropped > 0)
            {
                return Err(IngestError::FalseExtractionPass);
            }
        }
    }

    if stages[ValidationStage::ExternalReferenceResolution.key()].disposition
        == StageDisposition::Passed
        && body.extraction.is_none()
    {
        return Err(IngestError::InvalidExtractionEvidence);
    }

    if !valid_sha256(&qualification.receipt_sha256) {
        return Err(IngestError::InvalidReceiptDigest);
    }
    if compute_ingest_receipt_digest(qualification)? != qualification.receipt_sha256 {
        return Err(IngestError::ReceiptDigestMismatch);
    }
    Ok(())
}

/// Canonical digest over a fixed-order JSON body with sorted document/link/tool/stage
/// arrays. The profile is versioned by INGEST_SCHEMA and is not RFC 8785/JCS.
pub fn compute_ingest_receipt_digest(
    qualification: &QifIngestQualificationV1,
) -> Result<String, IngestError> {
    let body = canonical_body(&qualification.body);
    let bytes = serde_json::to_vec(&body).map_err(|_| IngestError::CanonicalizationFailed)?;
    let digest = Sha256::digest(&bytes);
    Ok(digest.iter().map(|byte| format!("{byte:02x}")).collect())
}

/// Verify exact bytes for all source documents, schemas, XSLT dependencies,
/// tools/runtime, sandbox references, stage reports and extraction artifacts.
pub fn verify_ingest_payloads(
    qualification: &QifIngestQualificationV1,
    payloads: &[(&ArtifactRef, &[u8])],
) -> Result<IngestVerificationReport, IngestError> {
    validate_ingest_qualification(qualification)?;
    let mut expected = BTreeMap::new();
    for document in &qualification.body.documents {
        add_artifact(&mut expected, &document.artifact)?;
    }
    for file in &qualification.body.bundles.xsd.files {
        add_artifact(&mut expected, &file.artifact)?;
    }
    add_artifact(&mut expected, &qualification.body.bundles.xsd.entrypoint)?;
    for file in &qualification.body.bundles.xslt.files {
        add_artifact(&mut expected, &file.artifact)?;
    }
    add_artifact(
        &mut expected,
        &qualification.body.bundles.xslt.root_stylesheet,
    )?;
    add_artifact(
        &mut expected,
        &qualification.body.bundles.xslt.check_parameters,
    )?;
    add_artifact(&mut expected, &qualification.body.runtime.runtime_image)?;
    for tool in &qualification.body.tools {
        add_artifact(&mut expected, &tool.executable)?;
    }
    add_artifact(&mut expected, &qualification.body.sandbox.policy_artifact)?;
    add_artifact(
        &mut expected,
        &qualification.body.sandbox.runner_attestation,
    )?;
    for stage in &qualification.body.stage_results {
        if let Some(report) = &stage.report_artifact {
            add_artifact(&mut expected, report)?;
        }
    }
    if let Some(extraction) = &qualification.body.extraction {
        add_artifact(&mut expected, &extraction.normalized_trace_artifact)?;
        add_artifact(&mut expected, &extraction.source_mapping_manifest)?;
    }

    if payloads.len() != expected.len() {
        return Err(IngestError::ArtifactPayloadSetMismatch);
    }
    let mut seen = BTreeSet::new();
    for (reference, bytes) in payloads {
        let key = (reference.id.clone(), reference.kind.clone());
        if !seen.insert(key.clone()) {
            return Err(IngestError::DuplicateArtifactPayload(key.0));
        }
        let Some(artifact) = expected.get(&key) else {
            return Err(IngestError::UnexpectedArtifactPayload(key.0));
        };
        verify_artifact_bytes(artifact, bytes).map_err(IngestError::ArtifactVerification)?;
    }
    if seen.len() != expected.len() {
        return Err(IngestError::ArtifactPayloadSetMismatch);
    }

    Ok(IngestVerificationReport {
        disposition: IngestDisposition::IntegrityChecked,
        qualification_sha256: qualification.receipt_sha256.clone(),
        checked_artifacts: expected.len(),
        profile_is_normative: qualification.body.qif_profile.is_normative(),
        legacy_runtime: qualification.body.runtime.support == RuntimeSupport::UnsupportedLegacy,
        all_stages_passed: qualification
            .body
            .stage_results
            .iter()
            .all(|stage| stage.disposition == StageDisposition::Passed),
        sandbox_profile_declared: true,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn artifact(kind: &str, format_id: &str, bytes: &[u8]) -> ContentArtifactV1 {
        let digest = Sha256::digest(bytes);
        let sha256 = digest
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        ContentArtifactV1 {
            reference: ArtifactRef {
                id: format!("sha256:{sha256}"),
                kind: kind.into(),
            },
            sha256,
            byte_length: bytes.len() as u64,
            media_type: "application/octet-stream".into(),
            format_id: format_id.into(),
        }
    }

    fn fixture() -> (QifIngestQualificationV1, Vec<(ArtifactRef, Vec<u8>)>) {
        let results_bytes = b"<QIFDocument kind=\"Results\"/>";
        let plan_bytes = b"<QIFDocument kind=\"Plan\"/>";
        let xsd_bytes = b"<xs:schema/>";
        let xslt_bytes = b"<xsl:stylesheet/>";
        let params_bytes = b"<CheckParameters/>";
        let runtime_bytes = b"runtime image digest fixture";
        let policy_bytes = b"sandbox policy fixture";
        let runner_bytes = b"runner attestation fixture";
        let tool_bytes = [
            b"xml validator executable".as_slice(),
            b"xslt validator executable".as_slice(),
            b"extractor executable".as_slice(),
            b"trace verifier executable".as_slice(),
        ];
        let report_bytes = [
            b"xml report".as_slice(),
            b"xsd report".as_slice(),
            b"xslt format report".as_slice(),
            b"xslt quality report".as_slice(),
            b"xslt semantic report".as_slice(),
            b"extract report".as_slice(),
            b"trace report".as_slice(),
        ];
        let results = artifact("qif_results", "QIF_3.0", results_bytes);
        let plan = artifact("qif_plan", "QIF_3.0", plan_bytes);
        let xsd = artifact("xsd_schema", "QIF_3.0_XSD", xsd_bytes);
        let xslt = artifact("xslt_root", "QIF_3.0_XSLT", xslt_bytes);
        let parameters = artifact("xslt_parameters", "QIF_CHECK_PARAMETERS_V1", params_bytes);
        let runtime = artifact("runtime_image", "DOTNET_10_LTS", runtime_bytes);
        let policy = artifact("sandbox_policy", "QIF_OFFLINE_SANDBOX_V1", policy_bytes);
        let runner = artifact(
            "runner_attestation",
            "HOST_RUNNER_ATTESTATION_V1",
            runner_bytes,
        );
        let tools = [
            ValidatorToolV1 {
                tool_id: "xml".into(),
                kind: ValidatorKind::XmlSchema,
                name: "xml-validator".into(),
                version: "1.0".into(),
                source_repository: "https://example.invalid/validator".into(),
                source_commit_sha: "a".repeat(40),
                executable: artifact("validator_xml", "XML_VALIDATOR", tool_bytes[0]),
            },
            ValidatorToolV1 {
                tool_id: "xslt".into(),
                kind: ValidatorKind::Xslt,
                name: "qif-xslt".into(),
                version: "1.0".into(),
                source_repository: "https://example.invalid/validator".into(),
                source_commit_sha: "b".repeat(40),
                executable: artifact("validator_xslt", "XSLT_VALIDATOR", tool_bytes[1]),
            },
            ValidatorToolV1 {
                tool_id: "extractor".into(),
                kind: ValidatorKind::Extractor,
                name: "qif-extractor".into(),
                version: "1.0".into(),
                source_repository: "https://example.invalid/extractor".into(),
                source_commit_sha: "c".repeat(40),
                executable: artifact("qif_extractor", "QIF_EXTRACTOR", tool_bytes[2]),
            },
            ValidatorToolV1 {
                tool_id: "trace".into(),
                kind: ValidatorKind::TraceResolver,
                name: "qif-trace".into(),
                version: "1.0".into(),
                source_repository: "https://example.invalid/trace".into(),
                source_commit_sha: "d".repeat(40),
                executable: artifact("qif_trace", "QIF_TRACE_RESOLVER", tool_bytes[3]),
            },
        ];
        let stages = ValidationStage::all()
            .into_iter()
            .enumerate()
            .map(|(index, stage)| {
                let tool_id = match stage {
                    ValidationStage::XmlWellFormedness | ValidationStage::XsdStructure => "xml",
                    ValidationStage::XsltFormat
                    | ValidationStage::XsltQuality
                    | ValidationStage::XsltSemantic => "xslt",
                    ValidationStage::ExtractionFidelity => "extractor",
                    ValidationStage::ExternalReferenceResolution => "trace",
                };
                StageResultV1 {
                    stage,
                    disposition: StageDisposition::Passed,
                    validator_tool_id: Some(tool_id.into()),
                    report_artifact: Some(artifact(
                        "stage_report",
                        "QIF_STAGE_REPORT",
                        report_bytes[index],
                    )),
                    reason_codes: Vec::new(),
                }
            })
            .collect();

        let normalized_trace = artifact(
            "normalized_qif_trace",
            "AeroQifExternalReferenceTraceV1",
            b"normalized trace bytes",
        );
        let mapping_records: Vec<SourceMappingRecordV1> = (1u32..=10).map(|id| {
            SourceMappingRecordV1 {
                document_qpid: "10000000-0000-4000-8000-000000000001".into(),
                source_path: format!(
                    "/QIFDocument/Results/MeasurementResultsSet/CharacteristicMeasurement[@id={}]/CharacteristicItemId",
                    900 + id
                ),
                source_element_id: Some(900 + id),
                local_reference_id: None,
                parent_external_document_reference_id: None,
                external_document_reference_id: Some(7),
                external_xid: Some(300 + id),
                mapping_rule_id: "qif.characteristic_item.external_xid".into(),
                mapping_profile_version: "AERO_QIF_SOURCE_MAPPING_V1".into(),
                disposition: SourceMappingDisposition::Mapped,
                target_object_kind: Some("normalized_characteristic_item".into()),
                target_object_id: Some(format!("characteristic-item:{}", 300 + id)),
                reason_code: None,
            }
        }).collect();
        let required_locators: Vec<SourceFieldLocatorV1> = mapping_records
            .iter()
            .map(source_locator_from_mapping)
            .collect();
        let mapping_manifest_bytes =
            source_mapping_manifest_bytes(&required_locators, &mapping_records).unwrap();
        let mapping_manifest = artifact(
            "source_mapping",
            "AERO_QIF_SOURCE_MAPPING_V1",
            &mapping_manifest_bytes,
        );
        let body = QifIngestQualificationBodyV1 {
            qualification_id: "qualification-001".into(),
            benchmark_case_id: "case-test8-synthetic".into(),
            qif_standard_version: "3.0.0".into(),
            qif_profile: QifValidationProfile::Qif3Normative,
            documents: vec![
                QifDocumentInputV1 {
                    qpid: "10000000-0000-4000-8000-000000000001".into(),
                    role: QifDocumentRole::Results,
                    artifact: results.clone(),
                    source_uri: None,
                },
                QifDocumentInputV1 {
                    qpid: "20000000-0000-4000-8000-000000000002".into(),
                    role: QifDocumentRole::Plan,
                    artifact: plan.clone(),
                    source_uri: Some("https://vendor.example/qif/Plan%20A.qif".into()),
                },
            ],
            external_document_bindings: vec![ExternalDocumentBindingV1 {
                results_qpid: "10000000-0000-4000-8000-000000000001".into(),
                local_reference_id: 7,
                target_plan_qpid: "20000000-0000-4000-8000-000000000002".into(),
                source_uri: Some("https://vendor.example/qif/Plan%20A.qif".into()),
                staged_relative_path: "linked/Plan-A.qif".into(),
            }],
            bundles: ValidationBundlesV1 {
                xsd: XsdBundleV1 {
                    entrypoint: xsd.clone(),
                    files: vec![BundleFileV1 {
                        relative_path: "QIFDocument.xsd".into(),
                        artifact: xsd.clone(),
                    }],
                },
                xslt: XsltBundleV1 {
                    root_stylesheet: xslt.clone(),
                    files: vec![BundleFileV1 {
                        relative_path: "Check.xsl".into(),
                        artifact: xslt.clone(),
                    }],
                    check_parameters: parameters.clone(),
                    profile: QifValidationProfile::Qif3Normative,
                },
            },
            runtime: ValidatorRuntimeV1 {
                runtime_name: "dotnet".into(),
                runtime_version: "10.0 LTS".into(),
                support: RuntimeSupport::SupportedLts,
                runtime_image: runtime.clone(),
            },
            tools: tools.to_vec(),
            sandbox: SandboxProfileV1 {
                network: NetworkPolicy::Disabled,
                filesystem: FilesystemPolicy::StagedInputsReadOnlyOutputsWriteOnly,
                rootless: true,
                no_host_secrets: true,
                cpu_limit_seconds: 120,
                memory_limit_bytes: 1024 * 1024 * 1024,
                wall_time_limit_ms: 120_000,
                max_document_bytes: 10 * 1024 * 1024,
                max_total_input_bytes: 50 * 1024 * 1024,
                max_document_count: 32,
                max_link_recursion_depth: 4,
                policy_artifact: policy.clone(),
                runner_attestation: runner.clone(),
            },
            stage_results: stages,
            extraction: Some(ExtractionEvidenceV1 {
                normalized_trace_artifact: normalized_trace.clone(),
                source_mapping_manifest: mapping_manifest.clone(),
                required_locators,
                mappings: mapping_records,
                mapping_counts: ExtractionMappingCountsV1 {
                    total_required: 10,
                    mapped: 10,
                    unresolved: 0,
                    ambiguous: 0,
                    unsupported: 0,
                    dropped: 0,
                },
            }),
        };
        let mut qualification = QifIngestQualificationV1 {
            schema: INGEST_SCHEMA.into(),
            body,
            receipt_sha256: String::new(),
        };
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();

        let mut payloads = vec![
            (results.reference.clone(), results_bytes.to_vec()),
            (plan.reference.clone(), plan_bytes.to_vec()),
            (xsd.reference.clone(), xsd_bytes.to_vec()),
            (xslt.reference.clone(), xslt_bytes.to_vec()),
            (parameters.reference.clone(), params_bytes.to_vec()),
            (runtime.reference.clone(), runtime_bytes.to_vec()),
            (policy.reference.clone(), policy_bytes.to_vec()),
            (runner.reference.clone(), runner_bytes.to_vec()),
        ];
        for (index, tool) in tools.iter().enumerate() {
            payloads.push((
                tool.executable.reference.clone(),
                tool_bytes[index].to_vec(),
            ));
        }
        for (index, stage) in qualification.body.stage_results.iter().enumerate() {
            payloads.push((
                stage.report_artifact.as_ref().unwrap().reference.clone(),
                report_bytes[index].to_vec(),
            ));
        }
        payloads.push((
            normalized_trace.reference.clone(),
            b"normalized trace bytes".to_vec(),
        ));
        payloads.push((mapping_manifest.reference.clone(), mapping_manifest_bytes));
        (qualification, payloads)
    }

    fn borrowed(payloads: &[(ArtifactRef, Vec<u8>)]) -> Vec<(&ArtifactRef, &[u8])> {
        payloads
            .iter()
            .map(|(reference, bytes)| (reference, bytes.as_slice()))
            .collect()
    }

    fn refresh_mapping_manifest(extraction: &mut ExtractionEvidenceV1) {
        let bytes =
            source_mapping_manifest_bytes(&extraction.required_locators, &extraction.mappings)
                .unwrap();
        extraction.source_mapping_manifest =
            artifact("source_mapping", "AERO_QIF_SOURCE_MAPPING_V1", &bytes);
    }

    #[test]
    fn synthetic_normative_qualification_verifies_all_artifacts() {
        let (qualification, payloads) = fixture();
        let report = verify_ingest_payloads(&qualification, &borrowed(&payloads)).unwrap();
        assert_eq!(report.disposition, IngestDisposition::IntegrityChecked);
        assert!(report.profile_is_normative);
        assert!(report.all_stages_passed);
        assert!(!report.legacy_runtime);
        assert_eq!(report.checked_artifacts, payloads.len());
    }

    #[test]
    fn remote_source_uri_is_retained_but_only_safe_staged_path_is_used() {
        let (qualification, _) = fixture();
        assert!(validate_ingest_qualification(&qualification).is_ok());
        assert_eq!(
            qualification.body.external_document_bindings[0]
                .source_uri
                .as_deref(),
            Some("https://vendor.example/qif/Plan%20A.qif")
        );
        assert_eq!(
            qualification.body.external_document_bindings[0].staged_relative_path,
            "linked/Plan-A.qif"
        );
    }

    #[test]
    fn missing_optional_source_uri_is_admitted_with_explicit_staged_path() {
        let (mut qualification, _) = fixture();
        qualification.body.documents[1].source_uri = None;
        qualification.body.external_document_bindings[0].source_uri = None;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(validate_ingest_qualification(&qualification).is_ok());
    }

    #[test]
    fn percent_encoded_path_traversal_is_rejected_in_staged_path() {
        let (mut qualification, _) = fixture();
        qualification.body.external_document_bindings[0].staged_relative_path =
            "linked/%2e%2e/secret.qif".into();
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::UnsafeStagedPath(_))
        ));
    }

    #[test]
    fn same_plan_may_be_referenced_by_multiple_results_ids() {
        let (mut qualification, _) = fixture();
        let mut alias = qualification.body.external_document_bindings[0].clone();
        alias.local_reference_id = 8;
        qualification.body.external_document_bindings.push(alias);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(validate_ingest_qualification(&qualification).is_ok());
    }

    #[test]
    fn stage_cannot_pass_when_prerequisite_failed() {
        let (mut qualification, _) = fixture();
        let xsd = qualification
            .body
            .stage_results
            .iter_mut()
            .find(|stage| stage.stage == ValidationStage::XsdStructure)
            .unwrap();
        xsd.disposition = StageDisposition::Failed;
        xsd.reason_codes.push("XSD_INVALID".into());
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::PrerequisiteNotPassed { .. })
        ));
    }

    #[test]
    fn blocked_stage_is_distinct_from_not_run_and_requires_reason() {
        let (mut qualification, _) = fixture();
        let semantic = qualification
            .body
            .stage_results
            .iter_mut()
            .find(|stage| stage.stage == ValidationStage::XsltSemantic)
            .unwrap();
        semantic.disposition = StageDisposition::Blocked;
        semantic.validator_tool_id = None;
        semantic.report_artifact = None;
        semantic
            .reason_codes
            .push("LINKED_DOCUMENT_STAGING_BLOCKED".into());
        let extraction = qualification
            .body
            .stage_results
            .iter_mut()
            .find(|stage| stage.stage == ValidationStage::ExtractionFidelity)
            .unwrap();
        extraction.disposition = StageDisposition::NotRun;
        extraction.validator_tool_id = None;
        extraction.report_artifact = None;
        let trace = qualification
            .body
            .stage_results
            .iter_mut()
            .find(|stage| stage.stage == ValidationStage::ExternalReferenceResolution)
            .unwrap();
        trace.disposition = StageDisposition::NotRun;
        trace.validator_tool_id = None;
        trace.report_artifact = None;
        qualification.body.extraction = None;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(validate_ingest_qualification(&qualification).is_ok());
    }

    #[test]
    fn stage_with_unexplained_warning_is_rejected() {
        let (mut qualification, _) = fixture();
        let stage = qualification
            .body
            .stage_results
            .iter_mut()
            .find(|stage| stage.stage == ValidationStage::XsltQuality)
            .unwrap();
        stage.disposition = StageDisposition::Warning;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::InvalidIdentity(_))
        ));
    }

    #[test]
    fn source_document_count_limit_is_enforced() {
        let (mut qualification, _) = fixture();
        qualification.body.sandbox.max_document_count = 1;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::TooManyDocuments)
        );
    }

    #[test]
    fn total_input_limit_includes_schema_stylesheets_and_parameters() {
        let (mut qualification, _) = fixture();
        qualification.body.sandbox.max_total_input_bytes = 100;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::TotalInputExceedsLimit)
        );
    }

    #[test]
    fn dotnet_core_31_cannot_be_declared_supported() {
        let (mut qualification, _) = fixture();
        qualification.body.runtime.runtime_version = "3.1.32".into();
        qualification.body.runtime.support = RuntimeSupport::SupportedLts;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::LegacyRuntimeMisclassified)
        );
    }

    #[test]
    fn source_uri_must_match_the_declared_external_document_metadata() {
        let (mut qualification, _) = fixture();
        qualification.body.external_document_bindings[0].source_uri =
            Some("https://other.example/Plan.qif".into());
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::SourceUriMismatch(
                "20000000-0000-4000-8000-000000000002".into()
            ))
        );
    }

    #[test]
    fn network_enabled_worker_is_rejected() {
        let (mut qualification, _) = fixture();
        qualification.body.sandbox.network = NetworkPolicy::Enabled;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::UnsafeSandboxProfile)
        );
    }

    #[test]
    fn unrestricted_filesystem_is_rejected() {
        let (mut qualification, _) = fixture();
        qualification.body.sandbox.filesystem = FilesystemPolicy::Unrestricted;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::UnsafeSandboxProfile)
        );
    }

    #[test]
    fn staged_path_traversal_is_rejected() {
        let (mut qualification, _) = fixture();
        qualification.body.external_document_bindings[0].staged_relative_path =
            "../outside/Plan.qif".into();
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::UnsafeStagedPath(_))
        ));
    }

    #[test]
    fn xsd_and_xslt_profiles_cannot_be_mixed() {
        let (mut qualification, _) = fixture();
        qualification.body.bundles.xslt.profile = QifValidationProfile::Qif3PlusPreview;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::BundleProfileMismatch)
        );
    }

    #[test]
    fn legacy_runtime_is_reported_but_not_misrepresented_as_current() {
        let (mut qualification, payloads) = fixture();
        qualification.body.runtime.support = RuntimeSupport::UnsupportedLegacy;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        let report = verify_ingest_payloads(&qualification, &borrowed(&payloads)).unwrap();
        assert!(report.legacy_runtime);
    }

    #[test]
    fn extraction_cannot_pass_with_unresolved_mappings() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.mapping_counts.unresolved = 1;
        extraction.mapping_counts.mapped = 9;
        extraction.mappings[0].disposition = SourceMappingDisposition::Unresolved;
        extraction.mappings[0].target_object_kind = None;
        extraction.mappings[0].target_object_id = None;
        extraction.mappings[0].reason_code = Some("TARGET_NOT_FOUND".into());
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::FalseExtractionPass)
        );
    }

    #[test]
    fn dropped_extraction_mappings_cannot_be_hidden_in_warning_or_pass() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.mapping_counts.dropped = 1;
        extraction.mapping_counts.mapped = 9;
        extraction.mappings[0].disposition = SourceMappingDisposition::Dropped;
        extraction.mappings[0].target_object_kind = None;
        extraction.mappings[0].target_object_id = None;
        extraction.mappings[0].reason_code = Some("UNMAPPED_SOURCE_FIELD".into());
        refresh_mapping_manifest(extraction);
        let stage = qualification
            .body
            .stage_results
            .iter_mut()
            .find(|stage| stage.stage == ValidationStage::ExtractionFidelity)
            .unwrap();
        stage.disposition = StageDisposition::Warning;
        stage.reason_codes.push("DROPPED_SOURCE_FIELDS".into());
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::DroppedMappings)
        );
    }

    #[test]
    fn missing_mapping_row_cannot_shrink_the_frozen_source_inventory() {
        let (mut qualification, _) = fixture();
        qualification
            .body
            .extraction
            .as_mut()
            .unwrap()
            .mappings
            .pop();
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::SourceMappingInventoryMismatch)
        );
    }

    #[test]
    fn mapping_row_cannot_add_a_locator_outside_the_frozen_inventory() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.mappings[0]
            .source_path
            .push_str("/UnlistedField");
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::SourceMappingInventoryMismatch)
        );
    }

    #[test]
    fn duplicate_required_source_locator_is_rejected() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.required_locators[1] = extraction.required_locators[0].clone();
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::DuplicateSourceMappingLocator(_))
        ));
    }

    #[test]
    fn external_qif_metadata_uses_parent_reference_identity_not_target_id() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.required_locators[0].source_path =
            "/QIFDocument/ExternalQIFReferences/ExternalQIFDocument[1]/QPId#qpid".into();
        extraction.required_locators[0].local_reference_id = None;
        extraction.required_locators[0].parent_external_document_reference_id = Some(7);
        extraction.required_locators[0].external_document_reference_id = None;
        extraction.required_locators[0].external_xid = None;

        extraction.mappings[0].source_path =
            "/QIFDocument/ExternalQIFReferences/ExternalQIFDocument[1]/QPId#qpid".into();
        extraction.mappings[0].local_reference_id = None;
        extraction.mappings[0].parent_external_document_reference_id = Some(7);
        extraction.mappings[0].external_document_reference_id = None;
        extraction.mappings[0].external_xid = None;
        extraction.mappings[0].mapping_rule_id = "qif.external_reference.qpid".into();
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(validate_ingest_qualification(&qualification).is_ok());
    }

    #[test]
    fn external_qif_metadata_parent_reference_must_be_declared() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.required_locators[0].source_path =
            "/QIFDocument/ExternalQIFReferences/ExternalQIFDocument[1]/URI#uri".into();
        extraction.required_locators[0].local_reference_id = None;
        extraction.required_locators[0].parent_external_document_reference_id = Some(8);
        extraction.required_locators[0].external_document_reference_id = None;
        extraction.required_locators[0].external_xid = None;

        extraction.mappings[0].source_path =
            "/QIFDocument/ExternalQIFReferences/ExternalQIFDocument[1]/URI#uri".into();
        extraction.mappings[0].local_reference_id = None;
        extraction.mappings[0].parent_external_document_reference_id = Some(8);
        extraction.mappings[0].external_document_reference_id = None;
        extraction.mappings[0].external_xid = None;
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::InvalidSourceMappingLocator(_))
        ));
    }

    #[test]
    fn local_qif_reference_without_xid_is_admitted_as_local_reference() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.required_locators[0].local_reference_id = Some(7);
        extraction.required_locators[0].external_document_reference_id = None;
        extraction.required_locators[0].external_xid = None;
        extraction.mappings[0].local_reference_id = Some(7);
        extraction.mappings[0].external_document_reference_id = None;
        extraction.mappings[0].external_xid = None;
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(validate_ingest_qualification(&qualification).is_ok());
    }

    #[test]
    fn external_xid_without_local_reference_id_is_rejected() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.required_locators[0].external_document_reference_id = None;
        extraction.mappings[0].external_document_reference_id = None;
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::InvalidSourceMappingLocator(_))
        ));
    }

    #[test]
    fn local_and_external_reference_namespaces_cannot_be_combined() {
        let (mut qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_mut().unwrap();
        extraction.required_locators[0].local_reference_id = Some(55);
        extraction.mappings[0].local_reference_id = Some(55);
        refresh_mapping_manifest(extraction);
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::InvalidSourceMappingLocator(_))
        ));
    }

    #[test]
    fn source_mapping_manifest_bytes_are_order_invariant() {
        let (qualification, _) = fixture();
        let extraction = qualification.body.extraction.as_ref().unwrap();
        let forward =
            source_mapping_manifest_bytes(&extraction.required_locators, &extraction.mappings)
                .unwrap();
        let mut reversed_mappings = extraction.mappings.clone();
        reversed_mappings.reverse();
        let mut reversed_locators = extraction.required_locators.clone();
        reversed_locators.reverse();
        let backward =
            source_mapping_manifest_bytes(&reversed_locators, &reversed_mappings).unwrap();
        assert_eq!(forward, backward);
    }

    #[test]
    fn source_mapping_record_mutation_breaks_pinned_manifest_digest() {
        let (mut qualification, _) = fixture();
        qualification.body.extraction.as_mut().unwrap().mappings[0].mapping_rule_id =
            "unreviewed.rule".into();
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::SourceMappingManifestMismatch)
        );
    }

    #[test]
    fn source_mapping_counts_must_match_declared_rows() {
        let (mut qualification, _) = fixture();
        qualification
            .body
            .extraction
            .as_mut()
            .unwrap()
            .mapping_counts
            .mapped = 9;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::SourceMappingCountMismatch)
        );
    }

    #[test]
    fn duplicate_source_locator_is_rejected() {
        let (mut qualification, _) = fixture();
        let records = &mut qualification.body.extraction.as_mut().unwrap().mappings;
        records[1].document_qpid = records[0].document_qpid.clone();
        records[1].source_path = records[0].source_path.clone();
        records[1].source_element_id = records[0].source_element_id;
        records[1].local_reference_id = records[0].local_reference_id;
        records[1].parent_external_document_reference_id =
            records[0].parent_external_document_reference_id;
        records[1].external_document_reference_id = records[0].external_document_reference_id;
        records[1].external_xid = records[0].external_xid;
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::DuplicateSourceMappingLocator(_))
        ));
    }

    #[test]
    fn mapped_source_record_requires_a_concrete_target() {
        let (mut qualification, _) = fixture();
        qualification.body.extraction.as_mut().unwrap().mappings[0].target_object_id = None;
        refresh_mapping_manifest(qualification.body.extraction.as_mut().unwrap());
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::InvalidSourceMappingRecord(_))
        ));
    }

    #[test]
    fn external_source_locator_must_reference_a_declared_results_binding() {
        let (mut qualification, _) = fixture();
        qualification.body.extraction.as_mut().unwrap().mappings[0]
            .external_document_reference_id = Some(99);
        refresh_mapping_manifest(qualification.body.extraction.as_mut().unwrap());
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::InvalidSourceMappingLocator(_))
        ));
    }

    #[test]
    fn missing_stage_is_rejected_even_when_other_stages_pass() {
        let (mut qualification, _) = fixture();
        qualification.body.stage_results.pop();
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::StageSetMismatch)
        );
    }

    #[test]
    fn wrong_tool_kind_for_stage_is_rejected() {
        let (mut qualification, _) = fixture();
        let stage = qualification
            .body
            .stage_results
            .iter_mut()
            .find(|stage| stage.stage == ValidationStage::XsltSemantic)
            .unwrap();
        stage.validator_tool_id = Some("xml".into());
        qualification.receipt_sha256 = compute_ingest_receipt_digest(&qualification).unwrap();
        assert!(matches!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::WrongStageToolKind { .. })
        ));
    }

    #[test]
    fn receipt_digest_mutation_is_rejected() {
        let (mut qualification, _) = fixture();
        qualification.receipt_sha256 = "0".repeat(64);
        assert_eq!(
            validate_ingest_qualification(&qualification),
            Err(IngestError::ReceiptDigestMismatch)
        );
    }

    #[test]
    fn receipt_digest_is_document_order_invariant() {
        let (mut qualification, _) = fixture();
        let before = compute_ingest_receipt_digest(&qualification).unwrap();
        qualification.body.documents.reverse();
        qualification.body.external_document_bindings.reverse();
        qualification.body.tools.reverse();
        qualification.body.stage_results.reverse();
        qualification
            .body
            .extraction
            .as_mut()
            .unwrap()
            .mappings
            .reverse();
        assert_eq!(
            compute_ingest_receipt_digest(&qualification).unwrap(),
            before
        );
    }

    #[test]
    fn unknown_nested_fields_are_rejected() {
        let (qualification, _) = fixture();
        let mut value = serde_json::to_value(&qualification).unwrap();
        value["body"]["sandbox"]["network_bypass"] = Value::Bool(true);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(
            parse_ingest_qualification_json(&bytes),
            Err(IngestError::InvalidJson(_))
        ));
    }

    #[test]
    fn payload_byte_mutation_is_rejected() {
        let (qualification, mut payloads) = fixture();
        payloads[0].1.push(b'X');
        assert!(matches!(
            verify_ingest_payloads(&qualification, &borrowed(&payloads)),
            Err(IngestError::ArtifactVerification(
                ReceiptError::ArtifactByteLengthMismatch(_)
            ))
        ));
    }
}

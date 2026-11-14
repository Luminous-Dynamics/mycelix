//! Provenance sidecars for external source imports.
//!
//! This module records which source payload was observed, which schema/parser
//! produced the interpretation, how each enumerated field was handled, and
//! whether an import changed the target passport. It cannot prove that the
//! source is truthful, that the payload reference remains available, or that a
//! caller has authority to assert any recorded claim.

use std::error::Error;
use std::fmt;

const MAGIC: &[u8] = b"MYCELIX-CSCP-IMPORT-PROVENANCE\0";
const FORMAT_VERSION: u16 = 1;
const DIGEST_DOMAIN: &[u8] = b"MYCELIX-CSCP-IMPORT-PROVENANCE-DIGEST-V1\0";
const MAX_TEXT_BYTES: usize = 4096;
const MAX_FIELDS: u64 = 100_000;
const MAX_DIAGNOSTICS: u64 = 100_000;
const MAX_MANIFEST_BYTES: usize = 16 * 1024 * 1024;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ProvenanceError {
    EmptyRequiredField,
    FieldTooLong,
    InvalidControlCharacter,
    InvalidDigest,
    InvalidUtf8,
    ManifestDigestMismatch,
    PayloadLengthMismatch,
    PayloadDigestMismatch,
    UnsupportedVersion(u16),
    BadMagic,
    Truncated,
    TrailingBytes,
    InvalidOptionTag,
    InvalidEnumTag,
    TooManyFields,
    TooManyDiagnostics,
    ManifestTooLarge,
    DuplicateSourcePath,
    DiagnosticPathUnknown,
    MissingFieldDiagnostic,
    InvalidOutcome,
    NonCanonicalEncoding,
}

impl fmt::Display for ProvenanceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for ProvenanceError {}

fn validate_text(value: &str) -> Result<String, ProvenanceError> {
    let value = value.trim();
    if value.is_empty() {
        return Err(ProvenanceError::EmptyRequiredField);
    }
    if value.len() > MAX_TEXT_BYTES {
        return Err(ProvenanceError::FieldTooLong);
    }
    if value.chars().any(char::is_control) {
        return Err(ProvenanceError::InvalidControlCharacter);
    }
    Ok(value.to_owned())
}

fn validate_optional_text(value: Option<&str>) -> Result<Option<String>, ProvenanceError> {
    value
        .map(|value| validate_text(value).map(Some))
        .unwrap_or(Ok(None))
}

fn validate_digest(value: &str) -> Result<String, ProvenanceError> {
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(ProvenanceError::InvalidDigest);
    }
    Ok(value.to_ascii_lowercase())
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceDocument {
    source_system: String,
    source_record_id: Option<String>,
    schema_name: String,
    schema_version: String,
    parser_name: String,
    parser_version: String,
    imported_at: u64,
    media_type: Option<String>,
    raw_payload_ref: String,
    raw_payload_len: u64,
    raw_payload_digest: String,
}

impl SourceDocument {
    /// Capture provenance metadata and compute the digest from the exact raw
    /// bytes supplied by the caller. The caller is responsible for keeping
    /// raw_payload_ref stable and access-controlled.
    #[allow(clippy::too_many_arguments)]
    pub fn capture(
        source_system: impl AsRef<str>,
        source_record_id: Option<&str>,
        schema_name: impl AsRef<str>,
        schema_version: impl AsRef<str>,
        parser_name: impl AsRef<str>,
        parser_version: impl AsRef<str>,
        imported_at: u64,
        media_type: Option<&str>,
        raw_payload_ref: impl AsRef<str>,
        raw_payload: &[u8],
    ) -> Result<Self, ProvenanceError> {
        let raw_payload_len =
            u64::try_from(raw_payload.len()).map_err(|_| ProvenanceError::PayloadLengthMismatch)?;
        let raw_payload_digest = blake3::hash(raw_payload).to_hex().to_string();
        Self::from_parts(
            source_system.as_ref(),
            source_record_id,
            schema_name.as_ref(),
            schema_version.as_ref(),
            parser_name.as_ref(),
            parser_version.as_ref(),
            imported_at,
            media_type,
            raw_payload_ref.as_ref(),
            raw_payload_len,
            &raw_payload_digest,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_parts(
        source_system: &str,
        source_record_id: Option<&str>,
        schema_name: &str,
        schema_version: &str,
        parser_name: &str,
        parser_version: &str,
        imported_at: u64,
        media_type: Option<&str>,
        raw_payload_ref: &str,
        raw_payload_len: u64,
        raw_payload_digest: &str,
    ) -> Result<Self, ProvenanceError> {
        Ok(Self {
            source_system: validate_text(source_system)?,
            source_record_id: validate_optional_text(source_record_id)?,
            schema_name: validate_text(schema_name)?,
            schema_version: validate_text(schema_version)?,
            parser_name: validate_text(parser_name)?,
            parser_version: validate_text(parser_version)?,
            imported_at,
            media_type: validate_optional_text(media_type)?,
            raw_payload_ref: validate_text(raw_payload_ref)?,
            raw_payload_len,
            raw_payload_digest: validate_digest(raw_payload_digest)?,
        })
    }

    /// Verify that bytes retrieved from raw_payload_ref still match the exact
    /// source bytes originally observed by capture().
    pub fn verify_payload(&self, raw_payload: &[u8]) -> Result<(), ProvenanceError> {
        if u64::try_from(raw_payload.len()).ok() != Some(self.raw_payload_len) {
            return Err(ProvenanceError::PayloadLengthMismatch);
        }
        let actual = blake3::hash(raw_payload).to_hex().to_string();
        if actual != self.raw_payload_digest {
            return Err(ProvenanceError::PayloadDigestMismatch);
        }
        Ok(())
    }

    pub fn source_system(&self) -> &str {
        &self.source_system
    }

    pub fn source_record_id(&self) -> Option<&str> {
        self.source_record_id.as_deref()
    }

    pub fn schema_name(&self) -> &str {
        &self.schema_name
    }

    pub fn schema_version(&self) -> &str {
        &self.schema_version
    }

    pub fn parser_name(&self) -> &str {
        &self.parser_name
    }

    pub fn parser_version(&self) -> &str {
        &self.parser_version
    }

    pub fn imported_at(&self) -> u64 {
        self.imported_at
    }

    pub fn media_type(&self) -> Option<&str> {
        self.media_type.as_deref()
    }

    pub fn raw_payload_ref(&self) -> &str {
        &self.raw_payload_ref
    }

    pub fn raw_payload_len(&self) -> u64 {
        self.raw_payload_len
    }

    pub fn raw_payload_digest(&self) -> &str {
        &self.raw_payload_digest
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum InventoryStatus {
    CompleteAsReportedByParser,
    Partial,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum ImportOutcome {
    Imported,
    ImportedWithWarnings,
    Rejected,
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum FieldTreatment {
    Mapped {
        target_path: String,
        mapping_rule_id: String,
        mapping_rule_version: String,
    },
    RetainedRaw {
        reason_code: String,
    },
    UnsupportedRetained {
        reason_code: String,
    },
}

impl FieldTreatment {
    fn validate(&self) -> Result<Self, ProvenanceError> {
        match self {
            Self::Mapped {
                target_path,
                mapping_rule_id,
                mapping_rule_version,
            } => Ok(Self::Mapped {
                target_path: validate_text(target_path)?,
                mapping_rule_id: validate_text(mapping_rule_id)?,
                mapping_rule_version: validate_text(mapping_rule_version)?,
            }),
            Self::RetainedRaw { reason_code } => Ok(Self::RetainedRaw {
                reason_code: validate_text(reason_code)?,
            }),
            Self::UnsupportedRetained { reason_code } => Ok(Self::UnsupportedRetained {
                reason_code: validate_text(reason_code)?,
            }),
        }
    }

    fn is_fully_mapped(&self) -> bool {
        matches!(self, Self::Mapped { .. })
    }

    fn requires_diagnostic(&self) -> bool {
        !self.is_fully_mapped()
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct FieldMapping {
    source_path: String,
    treatment: FieldTreatment,
}

impl FieldMapping {
    pub fn mapped(
        source_path: impl AsRef<str>,
        target_path: impl AsRef<str>,
        mapping_rule_id: impl AsRef<str>,
        mapping_rule_version: impl AsRef<str>,
    ) -> Result<Self, ProvenanceError> {
        Self::new(
            source_path,
            FieldTreatment::Mapped {
                target_path: target_path.as_ref().to_owned(),
                mapping_rule_id: mapping_rule_id.as_ref().to_owned(),
                mapping_rule_version: mapping_rule_version.as_ref().to_owned(),
            },
        )
    }

    pub fn retained_raw(
        source_path: impl AsRef<str>,
        reason_code: impl AsRef<str>,
    ) -> Result<Self, ProvenanceError> {
        Self::new(
            source_path,
            FieldTreatment::RetainedRaw {
                reason_code: reason_code.as_ref().to_owned(),
            },
        )
    }

    pub fn unsupported_retained(
        source_path: impl AsRef<str>,
        reason_code: impl AsRef<str>,
    ) -> Result<Self, ProvenanceError> {
        Self::new(
            source_path,
            FieldTreatment::UnsupportedRetained {
                reason_code: reason_code.as_ref().to_owned(),
            },
        )
    }

    fn new(
        source_path: impl AsRef<str>,
        treatment: FieldTreatment,
    ) -> Result<Self, ProvenanceError> {
        Ok(Self {
            source_path: validate_text(source_path.as_ref())?,
            treatment: treatment.validate()?,
        })
    }

    pub fn source_path(&self) -> &str {
        &self.source_path
    }

    pub fn treatment(&self) -> &FieldTreatment {
        &self.treatment
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum DiagnosticSeverity {
    Warning,
    Error,
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ImportDiagnostic {
    code: String,
    severity: DiagnosticSeverity,
    source_path: Option<String>,
    detail: String,
}

impl ImportDiagnostic {
    pub fn new(
        code: impl AsRef<str>,
        severity: DiagnosticSeverity,
        source_path: Option<&str>,
        detail: impl AsRef<str>,
    ) -> Result<Self, ProvenanceError> {
        Ok(Self {
            code: validate_text(code.as_ref())?,
            severity,
            source_path: validate_optional_text(source_path)?,
            detail: validate_text(detail.as_ref())?,
        })
    }

    pub fn code(&self) -> &str {
        &self.code
    }

    pub fn severity(&self) -> DiagnosticSeverity {
        self.severity
    }

    pub fn source_path(&self) -> Option<&str> {
        self.source_path.as_deref()
    }

    pub fn detail(&self) -> &str {
        &self.detail
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ImportProvenanceManifest {
    version: u16,
    source: SourceDocument,
    inventory_status: InventoryStatus,
    outcome: ImportOutcome,
    before_snapshot_digest: String,
    after_snapshot_digest: Option<String>,
    field_mappings: Vec<FieldMapping>,
    diagnostics: Vec<ImportDiagnostic>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FieldCoverage {
    pub total: usize,
    pub mapped: usize,
    pub retained_raw: usize,
    pub unsupported_retained: usize,
}

impl FieldCoverage {
    pub fn all_fields_accounted_for(&self) -> bool {
        self.mapped + self.retained_raw + self.unsupported_retained == self.total
    }

    pub fn all_fields_mapped(&self) -> bool {
        self.total == self.mapped
    }
}

impl ImportProvenanceManifest {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        source: SourceDocument,
        inventory_status: InventoryStatus,
        outcome: ImportOutcome,
        before_snapshot_digest: impl AsRef<str>,
        after_snapshot_digest: Option<&str>,
        mut field_mappings: Vec<FieldMapping>,
        mut diagnostics: Vec<ImportDiagnostic>,
    ) -> Result<Self, ProvenanceError> {
        let before_snapshot_digest = validate_digest(before_snapshot_digest.as_ref())?;
        let after_snapshot_digest = after_snapshot_digest.map(validate_digest).transpose()?;
        field_mappings.sort();
        diagnostics.sort();

        let manifest = Self {
            version: FORMAT_VERSION,
            source,
            inventory_status,
            outcome,
            before_snapshot_digest,
            after_snapshot_digest,
            field_mappings,
            diagnostics,
        };
        manifest.validate()?;
        Ok(manifest)
    }

    fn validate(&self) -> Result<(), ProvenanceError> {
        if self.version != FORMAT_VERSION {
            return Err(ProvenanceError::UnsupportedVersion(self.version));
        }
        if self.field_mappings.len() as u64 > MAX_FIELDS {
            return Err(ProvenanceError::TooManyFields);
        }
        if self.diagnostics.len() as u64 > MAX_DIAGNOSTICS {
            return Err(ProvenanceError::TooManyDiagnostics);
        }

        if self.field_mappings.windows(2).any(|pair| pair[0] > pair[1])
            || self.diagnostics.windows(2).any(|pair| pair[0] > pair[1])
        {
            return Err(ProvenanceError::NonCanonicalEncoding);
        }

        let mut prior_path: Option<&str> = None;
        for field in &self.field_mappings {
            validate_text(field.source_path())?;
            field.treatment.validate()?;
            if prior_path == Some(field.source_path()) {
                return Err(ProvenanceError::DuplicateSourcePath);
            }
            prior_path = Some(field.source_path());
        }

        let known_paths: std::collections::BTreeSet<&str> = self
            .field_mappings
            .iter()
            .map(FieldMapping::source_path)
            .collect();
        for diagnostic in &self.diagnostics {
            if let Some(path) = diagnostic.source_path() {
                if !known_paths.contains(path) {
                    return Err(ProvenanceError::DiagnosticPathUnknown);
                }
            }
        }

        for field in &self.field_mappings {
            if field.treatment.requires_diagnostic()
                && !self
                    .diagnostics
                    .iter()
                    .any(|diagnostic| diagnostic.source_path() == Some(field.source_path()))
            {
                return Err(ProvenanceError::MissingFieldDiagnostic);
            }
        }

        let has_error = self
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.severity() == DiagnosticSeverity::Error);
        let has_warning = !self.diagnostics.is_empty()
            || self.inventory_status != InventoryStatus::CompleteAsReportedByParser
            || self
                .field_mappings
                .iter()
                .any(|field| field.treatment.requires_diagnostic());

        match self.outcome {
            ImportOutcome::Imported => {
                if self.after_snapshot_digest.is_none()
                    || self.inventory_status != InventoryStatus::CompleteAsReportedByParser
                    || has_warning
                    || has_error
                {
                    return Err(ProvenanceError::InvalidOutcome);
                }
            }
            ImportOutcome::ImportedWithWarnings => {
                if self.after_snapshot_digest.is_none() || !has_warning || has_error {
                    return Err(ProvenanceError::InvalidOutcome);
                }
            }
            ImportOutcome::Rejected => {
                if !has_error
                    || self.after_snapshot_digest.as_deref()
                        != Some(self.before_snapshot_digest.as_str())
                {
                    return Err(ProvenanceError::InvalidOutcome);
                }
            }
        }

        Ok(())
    }

    pub fn field_coverage(&self) -> FieldCoverage {
        let mut coverage = FieldCoverage {
            total: self.field_mappings.len(),
            mapped: 0,
            retained_raw: 0,
            unsupported_retained: 0,
        };
        for field in &self.field_mappings {
            match &field.treatment {
                FieldTreatment::Mapped { .. } => coverage.mapped += 1,
                FieldTreatment::RetainedRaw { .. } => coverage.retained_raw += 1,
                FieldTreatment::UnsupportedRetained { .. } => {
                    coverage.unsupported_retained += 1;
                }
            }
        }
        coverage
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProvenanceError> {
        self.validate()?;
        let mut output = Vec::new();
        output.extend_from_slice(MAGIC);
        output.extend_from_slice(&self.version.to_be_bytes());
        encode_source(&mut output, &self.source)?;

        output.push(encode_inventory(self.inventory_status));
        output.push(encode_outcome(self.outcome));
        write_string(&mut output, &self.before_snapshot_digest)?;
        write_optional_string(&mut output, self.after_snapshot_digest.as_deref())?;

        write_count(&mut output, self.field_mappings.len())?;
        for field in &self.field_mappings {
            write_string(&mut output, field.source_path())?;
            match field.treatment() {
                FieldTreatment::Mapped {
                    target_path,
                    mapping_rule_id,
                    mapping_rule_version,
                } => {
                    output.push(1);
                    write_string(&mut output, target_path)?;
                    write_string(&mut output, mapping_rule_id)?;
                    write_string(&mut output, mapping_rule_version)?;
                }
                FieldTreatment::RetainedRaw { reason_code } => {
                    output.push(2);
                    write_string(&mut output, reason_code)?;
                }
                FieldTreatment::UnsupportedRetained { reason_code } => {
                    output.push(3);
                    write_string(&mut output, reason_code)?;
                }
            }
            if output.len() > MAX_MANIFEST_BYTES {
                return Err(ProvenanceError::ManifestTooLarge);
            }
        }

        write_count(&mut output, self.diagnostics.len())?;
        for diagnostic in &self.diagnostics {
            write_string(&mut output, diagnostic.code())?;
            output.push(encode_severity(diagnostic.severity()));
            write_optional_string(&mut output, diagnostic.source_path())?;
            write_string(&mut output, diagnostic.detail())?;
            if output.len() > MAX_MANIFEST_BYTES {
                return Err(ProvenanceError::ManifestTooLarge);
            }
        }
        if output.len() > MAX_MANIFEST_BYTES {
            return Err(ProvenanceError::ManifestTooLarge);
        }
        Ok(output)
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ProvenanceError> {
        if bytes.len() > MAX_MANIFEST_BYTES {
            return Err(ProvenanceError::ManifestTooLarge);
        }
        let mut reader = Reader::new(bytes);
        if reader.take(MAGIC.len())? != MAGIC {
            return Err(ProvenanceError::BadMagic);
        }
        let version = reader.u16()?;
        if version != FORMAT_VERSION {
            return Err(ProvenanceError::UnsupportedVersion(version));
        }
        let source = decode_source(&mut reader)?;
        let inventory_status = decode_inventory(reader.u8()?)?;
        let outcome = decode_outcome(reader.u8()?)?;
        let before = reader.string()?;
        let after = read_optional_string(&mut reader)?;

        let field_count = reader.u64()?;
        if field_count > MAX_FIELDS || field_count > bytes.len() as u64 {
            return Err(ProvenanceError::TooManyFields);
        }
        let mut field_mappings = Vec::with_capacity(field_count as usize);
        for _ in 0..field_count {
            let source_path = reader.string()?;
            let treatment = match reader.u8()? {
                1 => FieldTreatment::Mapped {
                    target_path: reader.string()?,
                    mapping_rule_id: reader.string()?,
                    mapping_rule_version: reader.string()?,
                },
                2 => FieldTreatment::RetainedRaw {
                    reason_code: reader.string()?,
                },
                3 => FieldTreatment::UnsupportedRetained {
                    reason_code: reader.string()?,
                },
                _ => return Err(ProvenanceError::InvalidEnumTag),
            };
            field_mappings.push(FieldMapping::new(source_path, treatment)?);
        }

        let diagnostic_count = reader.u64()?;
        if diagnostic_count > MAX_DIAGNOSTICS || diagnostic_count > bytes.len() as u64 {
            return Err(ProvenanceError::TooManyDiagnostics);
        }
        let mut diagnostics = Vec::with_capacity(diagnostic_count as usize);
        for _ in 0..diagnostic_count {
            let code = reader.string()?;
            let severity = decode_severity(reader.u8()?)?;
            let source_path = read_optional_string(&mut reader)?;
            let detail = reader.string()?;
            diagnostics.push(ImportDiagnostic::new(
                code,
                severity,
                source_path.as_deref(),
                detail,
            )?);
        }

        if !reader.is_empty() {
            return Err(ProvenanceError::TrailingBytes);
        }

        let manifest = Self {
            version,
            source,
            inventory_status,
            outcome,
            before_snapshot_digest: before,
            after_snapshot_digest: after,
            field_mappings,
            diagnostics,
        };
        manifest.validate()?;
        if manifest.canonical_bytes()? != bytes {
            return Err(ProvenanceError::NonCanonicalEncoding);
        }
        Ok(manifest)
    }

    pub fn digest_hex(&self) -> Result<String, ProvenanceError> {
        let bytes = self.canonical_bytes()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DIGEST_DOMAIN);
        hasher.update(&bytes);
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn verify_digest(&self, expected: &str) -> Result<(), ProvenanceError> {
        let expected = validate_digest(expected)?;
        if self.digest_hex()? == expected {
            Ok(())
        } else {
            Err(ProvenanceError::ManifestDigestMismatch)
        }
    }

    pub fn source(&self) -> &SourceDocument {
        &self.source
    }

    pub fn inventory_status(&self) -> InventoryStatus {
        self.inventory_status
    }

    pub fn outcome(&self) -> ImportOutcome {
        self.outcome
    }

    pub fn before_snapshot_digest(&self) -> &str {
        &self.before_snapshot_digest
    }

    pub fn after_snapshot_digest(&self) -> Option<&str> {
        self.after_snapshot_digest.as_deref()
    }

    pub fn field_mappings(&self) -> &[FieldMapping] {
        &self.field_mappings
    }

    pub fn diagnostics(&self) -> &[ImportDiagnostic] {
        &self.diagnostics
    }

    pub fn version(&self) -> u16 {
        self.version
    }
}

fn encode_source(output: &mut Vec<u8>, source: &SourceDocument) -> Result<(), ProvenanceError> {
    write_string(output, &source.source_system)?;
    write_optional_string(output, source.source_record_id.as_deref())?;
    write_string(output, &source.schema_name)?;
    write_string(output, &source.schema_version)?;
    write_string(output, &source.parser_name)?;
    write_string(output, &source.parser_version)?;
    output.extend_from_slice(&source.imported_at.to_be_bytes());
    write_optional_string(output, source.media_type.as_deref())?;
    write_string(output, &source.raw_payload_ref)?;
    output.extend_from_slice(&source.raw_payload_len.to_be_bytes());
    write_string(output, &source.raw_payload_digest)?;
    Ok(())
}

fn decode_source(reader: &mut Reader<'_>) -> Result<SourceDocument, ProvenanceError> {
    let source_system = reader.string()?;
    let source_record_id = read_optional_string(reader)?;
    let schema_name = reader.string()?;
    let schema_version = reader.string()?;
    let parser_name = reader.string()?;
    let parser_version = reader.string()?;
    let imported_at = reader.u64()?;
    let media_type = read_optional_string(reader)?;
    let raw_payload_ref = reader.string()?;
    let raw_payload_len = reader.u64()?;
    let raw_payload_digest = reader.string()?;
    SourceDocument::from_parts(
        &source_system,
        source_record_id.as_deref(),
        &schema_name,
        &schema_version,
        &parser_name,
        &parser_version,
        imported_at,
        media_type.as_deref(),
        &raw_payload_ref,
        raw_payload_len,
        &raw_payload_digest,
    )
}

fn encode_inventory(value: InventoryStatus) -> u8 {
    match value {
        InventoryStatus::CompleteAsReportedByParser => 1,
        InventoryStatus::Partial => 2,
        InventoryStatus::Unknown => 3,
    }
}

fn decode_inventory(value: u8) -> Result<InventoryStatus, ProvenanceError> {
    match value {
        1 => Ok(InventoryStatus::CompleteAsReportedByParser),
        2 => Ok(InventoryStatus::Partial),
        3 => Ok(InventoryStatus::Unknown),
        _ => Err(ProvenanceError::InvalidEnumTag),
    }
}

fn encode_outcome(value: ImportOutcome) -> u8 {
    match value {
        ImportOutcome::Imported => 1,
        ImportOutcome::ImportedWithWarnings => 2,
        ImportOutcome::Rejected => 3,
    }
}

fn decode_outcome(value: u8) -> Result<ImportOutcome, ProvenanceError> {
    match value {
        1 => Ok(ImportOutcome::Imported),
        2 => Ok(ImportOutcome::ImportedWithWarnings),
        3 => Ok(ImportOutcome::Rejected),
        _ => Err(ProvenanceError::InvalidEnumTag),
    }
}

fn encode_severity(value: DiagnosticSeverity) -> u8 {
    match value {
        DiagnosticSeverity::Warning => 1,
        DiagnosticSeverity::Error => 2,
    }
}

fn decode_severity(value: u8) -> Result<DiagnosticSeverity, ProvenanceError> {
    match value {
        1 => Ok(DiagnosticSeverity::Warning),
        2 => Ok(DiagnosticSeverity::Error),
        _ => Err(ProvenanceError::InvalidEnumTag),
    }
}

fn write_count(output: &mut Vec<u8>, count: usize) -> Result<(), ProvenanceError> {
    let count = u64::try_from(count).map_err(|_| ProvenanceError::TooManyFields)?;
    output.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn write_string(output: &mut Vec<u8>, value: &str) -> Result<(), ProvenanceError> {
    let value = validate_text(value)?;
    let bytes = value.as_bytes();
    let length = u32::try_from(bytes.len()).map_err(|_| ProvenanceError::FieldTooLong)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(bytes);
    Ok(())
}

fn write_optional_string(output: &mut Vec<u8>, value: Option<&str>) -> Result<(), ProvenanceError> {
    match value {
        None => output.push(0),
        Some(value) => {
            output.push(1);
            write_string(output, value)?;
        }
    }
    Ok(())
}

fn read_optional_string(reader: &mut Reader<'_>) -> Result<Option<String>, ProvenanceError> {
    match reader.u8()? {
        0 => Ok(None),
        1 => Ok(Some(reader.string()?)),
        _ => Err(ProvenanceError::InvalidOptionTag),
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn take(&mut self, length: usize) -> Result<&'a [u8], ProvenanceError> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or(ProvenanceError::Truncated)?;
        let value = self
            .bytes
            .get(self.offset..end)
            .ok_or(ProvenanceError::Truncated)?;
        self.offset = end;
        Ok(value)
    }

    fn u8(&mut self) -> Result<u8, ProvenanceError> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, ProvenanceError> {
        let value = self.take(2)?;
        Ok(u16::from_be_bytes([value[0], value[1]]))
    }

    fn u64(&mut self) -> Result<u64, ProvenanceError> {
        let value = self.take(8)?;
        Ok(u64::from_be_bytes([
            value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7],
        ]))
    }

    fn string(&mut self) -> Result<String, ProvenanceError> {
        let length = u32::from_be_bytes(self.take(4)?.try_into().unwrap()) as usize;
        if length > MAX_TEXT_BYTES {
            return Err(ProvenanceError::FieldTooLong);
        }
        String::from_utf8(self.take(length)?.to_vec()).map_err(|_| ProvenanceError::InvalidUtf8)
    }

    fn is_empty(&self) -> bool {
        self.offset == self.bytes.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source(payload: &[u8]) -> SourceDocument {
        SourceDocument::capture(
            "humanz-export",
            Some("batch-17-line-8"),
            "ddex-rdr-r",
            "1.1",
            "cscp-rdr-parser",
            "0.1.0",
            1_797_000_000,
            Some("application/xml"),
            "vault://source-objects/record-1",
            payload,
        )
        .unwrap()
    }

    fn clean_manifest() -> ImportProvenanceManifest {
        ImportProvenanceManifest::new(
            source(b"<report id='8'/>"),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::Imported,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            vec![
                FieldMapping::mapped(
                    "/Report/Id",
                    "/statement/source_record_id",
                    "rdr-r-report-id",
                    "1",
                )
                .unwrap(),
                FieldMapping::mapped(
                    "/Report/Period",
                    "/statement/reporting_period",
                    "rdr-r-period",
                    "1",
                )
                .unwrap(),
            ],
            vec![],
        )
        .unwrap()
    }

    #[test]
    fn source_digest_is_computed_from_raw_bytes_and_verifiable() {
        let payload = b"the exact raw payload";
        let source = source(payload);
        assert_eq!(
            source.raw_payload_digest(),
            blake3::hash(payload).to_hex().to_string()
        );
        assert!(source.verify_payload(payload).is_ok());
        assert_eq!(
            source.verify_payload(b"the changed payload"),
            Err(ProvenanceError::PayloadLengthMismatch)
        );
        assert_eq!(
            source.verify_payload(b"the exact raw payloAd"),
            Err(ProvenanceError::PayloadDigestMismatch)
        );
    }

    #[test]
    fn clean_import_requires_complete_parser_inventory_and_mapped_fields() {
        let manifest = clean_manifest();
        assert_eq!(manifest.outcome(), ImportOutcome::Imported);
        assert!(manifest.field_coverage().all_fields_accounted_for());
        assert!(manifest.field_coverage().all_fields_mapped());
        assert_eq!(manifest.field_coverage().total, 2);
    }

    #[test]
    fn unsupported_field_must_be_preserved_and_disclosed() {
        let field =
            FieldMapping::unsupported_retained("/Report/NewElement", "unknown_field").unwrap();
        let diagnostic = ImportDiagnostic::new(
            "unsupported_source_field",
            DiagnosticSeverity::Warning,
            Some("/Report/NewElement"),
            "Field retained in the original payload; no canonical mapping is applied",
        )
        .unwrap();
        let manifest = ImportProvenanceManifest::new(
            source(b"<report><new>value</new></report>"),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::ImportedWithWarnings,
            "00".repeat(32),
            Some("22".repeat(32).as_str()),
            vec![field],
            vec![diagnostic],
        )
        .unwrap();
        assert_eq!(manifest.field_coverage().unsupported_retained, 1);
        assert_eq!(manifest.outcome(), ImportOutcome::ImportedWithWarnings);
    }

    #[test]
    fn malformed_import_is_rejected_without_changing_snapshot_digest() {
        let error = ImportDiagnostic::new(
            "schema_validation_failed",
            DiagnosticSeverity::Error,
            None,
            "Required element is missing",
        )
        .unwrap();
        let digest = "ab".repeat(32);
        let manifest = ImportProvenanceManifest::new(
            source(b"<broken/>"),
            InventoryStatus::Unknown,
            ImportOutcome::Rejected,
            digest.as_str(),
            Some(digest.as_str()),
            vec![],
            vec![error],
        )
        .unwrap();
        assert_eq!(
            manifest.before_snapshot_digest(),
            manifest.after_snapshot_digest().unwrap()
        );
    }

    #[test]
    fn source_field_cannot_be_silently_dropped_or_repeated() {
        let retained = FieldMapping::retained_raw("/Report/Extra", "not_mapped").unwrap();
        let diagnostic = ImportDiagnostic::new(
            "retained_raw_only",
            DiagnosticSeverity::Warning,
            Some("/Report/Extra"),
            "Not mapped to the canonical model; source payload remains available",
        )
        .unwrap();
        let manifest = ImportProvenanceManifest::new(
            source(b"<report><extra/></report>"),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::ImportedWithWarnings,
            "00".repeat(32),
            Some("22".repeat(32).as_str()),
            vec![retained],
            vec![diagnostic],
        )
        .unwrap();
        assert_eq!(manifest.field_coverage().retained_raw, 1);
        assert!(!manifest.field_coverage().all_fields_mapped());
        assert_eq!(
            ImportProvenanceManifest::new(
                source(b"<report><extra/></report>"),
                InventoryStatus::CompleteAsReportedByParser,
                ImportOutcome::ImportedWithWarnings,
                "00".repeat(32),
                Some("22".repeat(32).as_str()),
                vec![
                    FieldMapping::mapped("/Report/Id", "/id", "rule", "1").unwrap(),
                    FieldMapping::mapped("/Report/Id", "/other", "rule", "1").unwrap(),
                ],
                vec![],
            ),
            Err(ProvenanceError::DuplicateSourcePath)
        );
    }

    #[test]
    fn canonical_manifest_round_trips_and_digest_is_stable() {
        let manifest = clean_manifest();
        let bytes = manifest.canonical_bytes().unwrap();
        let digest = manifest.digest_hex().unwrap();
        let decoded = ImportProvenanceManifest::from_bytes(&bytes).unwrap();
        assert_eq!(decoded, manifest);
        assert_eq!(decoded.canonical_bytes().unwrap(), bytes);
        assert!(decoded.verify_digest(&digest).is_ok());
        assert_eq!(
            decoded.verify_digest("ff".repeat(32).as_str()),
            Err(ProvenanceError::ManifestDigestMismatch)
        );
    }

    #[test]
    fn canonical_manifest_matches_frozen_cross_language_vector() {
        let source = SourceDocument::capture(
            "test-source",
            None,
            "rdr-n",
            "1.5",
            "test-parser",
            "0.1",
            1,
            None,
            "vault://artifact-1",
            b"",
        )
        .unwrap();
        let manifest = ImportProvenanceManifest::new(
            source,
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::Imported,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            vec![
                FieldMapping::mapped("/Report/Id", "/statement/id", "id-map", "1").unwrap(),
                FieldMapping::mapped("/Report/Period", "/statement/period", "period-map", "1")
                    .unwrap(),
            ],
            vec![],
        )
        .unwrap();
        let actual = manifest
            .canonical_bytes()
            .unwrap()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let expected = include_str!("../tests/fixtures/creator-import-provenance-v1.hex").trim();
        assert_eq!(actual, expected);
    }

    #[test]
    fn decoder_rejects_trailing_bytes_unknown_version_and_bad_magic() {
        let manifest = clean_manifest();
        let bytes = manifest.canonical_bytes().unwrap();

        let mut trailing = bytes.clone();
        trailing.push(0);
        assert_eq!(
            ImportProvenanceManifest::from_bytes(&trailing),
            Err(ProvenanceError::TrailingBytes)
        );

        let mut bad_version = bytes.clone();
        bad_version[MAGIC.len() + 1] = 2;
        assert_eq!(
            ImportProvenanceManifest::from_bytes(&bad_version),
            Err(ProvenanceError::UnsupportedVersion(2))
        );

        let mut bad_magic = bytes;
        bad_magic[0] ^= 0xff;
        assert_eq!(
            ImportProvenanceManifest::from_bytes(&bad_magic),
            Err(ProvenanceError::BadMagic)
        );
    }

    #[test]
    fn malformed_diagnostics_cannot_refer_to_uninventoried_fields() {
        let error = ImportDiagnostic::new(
            "schema_validation_failed",
            DiagnosticSeverity::Error,
            Some("/Report/NotThere"),
            "Field path was not declared by the parser",
        )
        .unwrap();
        assert_eq!(
            ImportProvenanceManifest::new(
                source(b"<broken/>"),
                InventoryStatus::Unknown,
                ImportOutcome::Rejected,
                "ab".repeat(32),
                Some("ab".repeat(32).as_str()),
                vec![],
                vec![error],
            ),
            Err(ProvenanceError::DiagnosticPathUnknown)
        );
    }

    #[test]
    fn decoder_rejects_noncanonical_field_mapping_order() {
        let manifest = clean_manifest();
        let bytes = manifest.canonical_bytes().unwrap();
        let mut reader = Reader::new(&bytes);
        reader.take(MAGIC.len()).unwrap();
        reader.u16().unwrap();
        decode_source(&mut reader).unwrap();
        reader.u8().unwrap();
        reader.u8().unwrap();
        reader.string().unwrap();
        read_optional_string(&mut reader).unwrap();
        assert_eq!(reader.u64().unwrap(), 2);

        let first_start = reader.offset;
        reader.string().unwrap();
        assert_eq!(reader.u8().unwrap(), 1);
        reader.string().unwrap();
        reader.string().unwrap();
        reader.string().unwrap();
        let first_end = reader.offset;

        let second_start = reader.offset;
        reader.string().unwrap();
        assert_eq!(reader.u8().unwrap(), 1);
        reader.string().unwrap();
        reader.string().unwrap();
        reader.string().unwrap();
        let second_end = reader.offset;

        let mut reordered = Vec::with_capacity(bytes.len());
        reordered.extend_from_slice(&bytes[..first_start]);
        reordered.extend_from_slice(&bytes[second_start..second_end]);
        reordered.extend_from_slice(&bytes[first_start..first_end]);
        reordered.extend_from_slice(&bytes[second_end..]);

        assert_eq!(
            ImportProvenanceManifest::from_bytes(&reordered),
            Err(ProvenanceError::NonCanonicalEncoding)
        );
    }
}

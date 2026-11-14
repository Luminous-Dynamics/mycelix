//! Idempotency guard for repeated processing of imported source payloads.
//!
//! The ledger prevents a duplicate source/schema/parser/mapping combination from
//! silently producing a second canonical mutation in one process. It is an
//! in-memory reference implementation; production ingress must back the same
//! uniqueness rule with a durable transaction/unique constraint.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

use crate::provenance::{FieldTreatment, ImportOutcome, ImportProvenanceManifest, InventoryStatus};

const KEY_DOMAIN: &[u8] = b"MYCELIX-CSCP-IMPORT-REPLAY-KEY-V1\0";
const RESULT_DOMAIN: &[u8] = b"MYCELIX-CSCP-IMPORT-RESULT-V1\0";
const MAX_LEDGER_ENTRIES: usize = 1_000_000;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ImportReplayError {
    ManifestInvalid,
    LedgerCapacityExceeded,
    ResultConflict,
}

impl fmt::Display for ImportReplayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for ImportReplayError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReplayDisposition {
    Recorded,
    AlreadyPresent,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReplayReceipt {
    key_digest: String,
    result_fingerprint: String,
    disposition: ReplayDisposition,
}

impl ReplayReceipt {
    pub fn key_digest(&self) -> &str {
        &self.key_digest
    }

    pub fn result_fingerprint(&self) -> &str {
        &self.result_fingerprint
    }

    pub fn disposition(&self) -> ReplayDisposition {
        self.disposition
    }
}

#[derive(Clone, Debug, Default)]
pub struct ImportReplayLedger {
    entries: BTreeMap<String, String>,
}

impl ImportReplayLedger {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record the canonical effect for one source-processing identity.
    ///
    /// The key includes raw payload bytes' digest, source identity, schema and
    /// parser versions, and every enumerated mapping/treatment. It deliberately
    /// excludes import timestamp and raw storage locator, so a retry after a
    /// storage relocation can remain idempotent when the bytes and intended
    /// transformation are identical.
    ///
    /// If the same key is seen with a different before/after result, outcome,
    /// inventory state, or diagnostics, return ResultConflict without mutating
    /// the ledger. A new parser/mapping version has a different key and can be
    /// recorded as an explicit reprocessing event.
    pub fn record(
        &mut self,
        manifest: &ImportProvenanceManifest,
    ) -> Result<ReplayReceipt, ImportReplayError> {
        // Use the public canonical encoder as the source of structural
        // validation before generating any deduplication keys.
        manifest
            .canonical_bytes()
            .map_err(|_| ImportReplayError::ManifestInvalid)?;

        let key_digest = replay_key_digest(manifest)?;
        let result_fingerprint = result_fingerprint(manifest)?;

        if let Some(existing) = self.entries.get(&key_digest) {
            if existing == &result_fingerprint {
                return Ok(ReplayReceipt {
                    key_digest,
                    result_fingerprint,
                    disposition: ReplayDisposition::AlreadyPresent,
                });
            }
            return Err(ImportReplayError::ResultConflict);
        }

        if self.entries.len() >= MAX_LEDGER_ENTRIES {
            return Err(ImportReplayError::LedgerCapacityExceeded);
        }

        self.entries
            .insert(key_digest.clone(), result_fingerprint.clone());
        Ok(ReplayReceipt {
            key_digest,
            result_fingerprint,
            disposition: ReplayDisposition::Recorded,
        })
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

fn replay_key_digest(manifest: &ImportProvenanceManifest) -> Result<String, ImportReplayError> {
    let source = manifest.source();
    let mut bytes = Vec::new();
    write_string(&mut bytes, source.source_system())?;
    write_optional_string(&mut bytes, source.source_record_id())?;
    write_string(&mut bytes, source.schema_name())?;
    write_string(&mut bytes, source.schema_version())?;
    write_string(&mut bytes, source.parser_name())?;
    write_string(&mut bytes, source.parser_version())?;
    write_string(&mut bytes, source.raw_payload_digest())?;
    bytes.extend_from_slice(&source.raw_payload_len().to_be_bytes());

    // Mapping/treatment details are part of transformation identity, so an
    // updated mapping rule creates a new replay key instead of being treated as
    // the same operation merely because the source bytes match.
    write_count(&mut bytes, manifest.field_mappings().len())?;
    for field in manifest.field_mappings() {
        write_string(&mut bytes, field.source_path())?;
        match field.treatment() {
            FieldTreatment::Mapped {
                target_path,
                mapping_rule_id,
                mapping_rule_version,
            } => {
                bytes.push(1);
                write_string(&mut bytes, target_path)?;
                write_string(&mut bytes, mapping_rule_id)?;
                write_string(&mut bytes, mapping_rule_version)?;
            }
            FieldTreatment::RetainedRaw { reason_code } => {
                bytes.push(2);
                write_string(&mut bytes, reason_code)?;
            }
            FieldTreatment::UnsupportedRetained { reason_code } => {
                bytes.push(3);
                write_string(&mut bytes, reason_code)?;
            }
        }
    }

    let mut hasher = blake3::Hasher::new();
    hasher.update(KEY_DOMAIN);
    hasher.update(&bytes);
    Ok(hasher.finalize().to_hex().to_string())
}

fn result_fingerprint(manifest: &ImportProvenanceManifest) -> Result<String, ImportReplayError> {
    let mut bytes = Vec::new();
    bytes.push(encode_inventory(manifest.inventory_status()));
    bytes.push(encode_outcome(manifest.outcome()));
    write_string(&mut bytes, manifest.before_snapshot_digest())?;
    write_optional_string(&mut bytes, manifest.after_snapshot_digest())?;
    write_count(&mut bytes, manifest.diagnostics().len())?;
    for diagnostic in manifest.diagnostics() {
        write_string(&mut bytes, diagnostic.code())?;
        bytes.push(match diagnostic.severity() {
            crate::provenance::DiagnosticSeverity::Warning => 1,
            crate::provenance::DiagnosticSeverity::Error => 2,
        });
        write_optional_string(&mut bytes, diagnostic.source_path())?;
        write_string(&mut bytes, diagnostic.detail())?;
    }

    let mut hasher = blake3::Hasher::new();
    hasher.update(RESULT_DOMAIN);
    hasher.update(&bytes);
    Ok(hasher.finalize().to_hex().to_string())
}

fn encode_inventory(value: InventoryStatus) -> u8 {
    match value {
        InventoryStatus::CompleteAsReportedByParser => 1,
        InventoryStatus::Partial => 2,
        InventoryStatus::Unknown => 3,
    }
}

fn encode_outcome(value: ImportOutcome) -> u8 {
    match value {
        ImportOutcome::Imported => 1,
        ImportOutcome::ImportedWithWarnings => 2,
        ImportOutcome::Rejected => 3,
    }
}

fn write_count(output: &mut Vec<u8>, count: usize) -> Result<(), ImportReplayError> {
    let count = u64::try_from(count).map_err(|_| ImportReplayError::ManifestInvalid)?;
    output.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn write_string(output: &mut Vec<u8>, value: &str) -> Result<(), ImportReplayError> {
    let bytes = value.as_bytes();
    let length = u32::try_from(bytes.len()).map_err(|_| ImportReplayError::ManifestInvalid)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(bytes);
    Ok(())
}

fn write_optional_string(
    output: &mut Vec<u8>,
    value: Option<&str>,
) -> Result<(), ImportReplayError> {
    match value {
        None => output.push(0),
        Some(value) => {
            output.push(1);
            write_string(output, value)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::provenance::{DiagnosticSeverity, FieldMapping, ImportDiagnostic, SourceDocument};

    fn source(
        payload: &[u8],
        imported_at: u64,
        record_ref: &str,
        parser_version: &str,
    ) -> SourceDocument {
        SourceDocument::capture(
            "platform-export",
            Some("record-42"),
            "ddex-rdr-n",
            "1.5",
            "cscp-parser",
            parser_version,
            imported_at,
            Some("application/xml"),
            record_ref,
            payload,
        )
        .unwrap()
    }

    fn manifest(
        payload: &[u8],
        imported_at: u64,
        record_ref: &str,
        parser_version: &str,
        after_digest: &str,
    ) -> ImportProvenanceManifest {
        ImportProvenanceManifest::new(
            source(payload, imported_at, record_ref, parser_version),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::Imported,
            "00".repeat(32),
            Some(after_digest),
            vec![
                FieldMapping::mapped("/Report/Id", "/statement/id", "report-id", "1").unwrap(),
                FieldMapping::mapped("/Report/Period", "/statement/period", "report-period", "1")
                    .unwrap(),
            ],
            vec![],
        )
        .unwrap()
    }

    #[test]
    fn exact_replay_is_idempotent_and_does_not_grow_ledger() {
        let manifest = manifest(
            b"<report id='42'/>",
            10,
            "vault://first",
            "1.0",
            &"11".repeat(32),
        );
        let mut ledger = ImportReplayLedger::new();

        let first = ledger.record(&manifest).unwrap();
        let replay = ledger.record(&manifest).unwrap();

        assert_eq!(first.disposition(), ReplayDisposition::Recorded);
        assert_eq!(replay.disposition(), ReplayDisposition::AlreadyPresent);
        assert_eq!(first.key_digest(), replay.key_digest());
        assert_eq!(first.result_fingerprint(), replay.result_fingerprint());
        assert_eq!(ledger.len(), 1);
    }

    #[test]
    fn retry_timestamp_and_storage_locator_do_not_change_replay_key() {
        let first = manifest(
            b"<report id='42'/>",
            10,
            "vault://primary/location-a",
            "1.0",
            &"11".repeat(32),
        );
        let retry = manifest(
            b"<report id='42'/>",
            99,
            "vault://archive/location-b",
            "1.0",
            &"11".repeat(32),
        );
        let mut ledger = ImportReplayLedger::new();

        let recorded = ledger.record(&first).unwrap();
        let replayed = ledger.record(&retry).unwrap();

        assert_eq!(replayed.disposition(), ReplayDisposition::AlreadyPresent);
        assert_eq!(recorded.key_digest(), replayed.key_digest());
        assert_eq!(recorded.result_fingerprint(), replayed.result_fingerprint());
    }

    #[test]
    fn changed_source_bytes_create_a_new_replay_key() {
        let first = manifest(
            b"<report id='42'/>",
            10,
            "vault://source",
            "1.0",
            &"11".repeat(32),
        );
        let second = manifest(
            b"<report id='43'/>",
            11,
            "vault://source",
            "1.0",
            &"22".repeat(32),
        );
        let mut ledger = ImportReplayLedger::new();

        let a = ledger.record(&first).unwrap();
        let b = ledger.record(&second).unwrap();

        assert_ne!(a.key_digest(), b.key_digest());
        assert_eq!(ledger.len(), 2);
    }

    #[test]
    fn changed_parser_version_creates_a_new_replay_key() {
        let first = manifest(
            b"<report id='42'/>",
            10,
            "vault://source",
            "1.0",
            &"11".repeat(32),
        );
        let second = manifest(
            b"<report id='42'/>",
            10,
            "vault://source",
            "1.1",
            &"22".repeat(32),
        );
        let mut ledger = ImportReplayLedger::new();

        let a = ledger.record(&first).unwrap();
        let b = ledger.record(&second).unwrap();

        assert_ne!(a.key_digest(), b.key_digest());
        assert_eq!(ledger.len(), 2);
    }

    #[test]
    fn same_key_with_different_after_digest_fails_without_mutation() {
        let first = manifest(
            b"<report id='42'/>",
            10,
            "vault://source",
            "1.0",
            &"11".repeat(32),
        );
        let conflicting = manifest(
            b"<report id='42'/>",
            99,
            "vault://archive",
            "1.0",
            &"22".repeat(32),
        );
        let mut ledger = ImportReplayLedger::new();

        ledger.record(&first).unwrap();
        assert_eq!(
            ledger.record(&conflicting),
            Err(ImportReplayError::ResultConflict)
        );
        assert_eq!(ledger.len(), 1);
    }

    #[test]
    fn mapping_rule_version_is_part_of_replay_identity() {
        let source = source(b"<report id='42'/>", 1, "vault://source", "1.0");
        let first = ImportProvenanceManifest::new(
            source.clone(),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::Imported,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            vec![FieldMapping::mapped("/Report/Id", "/statement/id", "report-id", "1").unwrap()],
            vec![],
        )
        .unwrap();
        let second = ImportProvenanceManifest::new(
            source,
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::Imported,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            vec![FieldMapping::mapped("/Report/Id", "/statement/id", "report-id", "2").unwrap()],
            vec![],
        )
        .unwrap();
        let mut ledger = ImportReplayLedger::new();
        let a = ledger.record(&first).unwrap();
        let b = ledger.record(&second).unwrap();

        assert_ne!(a.key_digest(), b.key_digest());
    }

    #[test]
    fn conflicting_result_for_same_processing_key_fails_closed() {
        let source = source(b"<report><id>42</id></report>", 1, "vault://source", "1.0");
        let fields =
            vec![FieldMapping::mapped("/Report/Id", "/statement/id", "report-id", "1").unwrap()];
        let clean = ImportProvenanceManifest::new(
            source.clone(),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::Imported,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            fields.clone(),
            vec![],
        )
        .unwrap();
        // Same source, parser, and mappings, but a warning/outcome discrepancy
        // must not be accepted as an idempotent replay.
        let warned = ImportProvenanceManifest::new(
            source,
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::ImportedWithWarnings,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            fields,
            vec![
                ImportDiagnostic::new(
                    "source_warning",
                    DiagnosticSeverity::Warning,
                    None,
                    "Source carried a warning not present in the first receipt",
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let mut ledger = ImportReplayLedger::new();
        let first = ledger.record(&clean).unwrap();
        assert_eq!(ledger.len(), 1);
        assert_eq!(
            ledger.record(&warned),
            Err(ImportReplayError::ResultConflict)
        );
        assert_eq!(ledger.len(), 1);
        assert_eq!(first.disposition(), ReplayDisposition::Recorded);
    }
}

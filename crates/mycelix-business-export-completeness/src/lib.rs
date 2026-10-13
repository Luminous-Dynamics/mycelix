// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Structural completeness checks for source-system exports.
//!
//! This crate checks the consistency of supplied extraction evidence. It does not
//! authenticate the provider statement, certify provider semantics, prove that a
//! source system recorded every real-world event, or grant business authority.

use std::collections::BTreeSet;

use sha2::{Digest, Sha256};

use mycelix_business_core::{Digest32, ReferenceId, ScopeRef};

/// A half-open export window: [start, end).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExportWindow {
    pub start_unix_ms: u64,
    pub end_unix_ms: u64,
}

impl ExportWindow {
    fn validate(self) -> Result<(), ExportCompletenessError> {
        if self.start_unix_ms >= self.end_unix_ms {
            return Err(ExportCompletenessError::InvalidWindow);
        }
        Ok(())
    }
}

/// Evidence for the cursor used to request a page. Missing must not be interpreted
/// as an initial request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PageRequestCursor {
    Initial,
    After(Digest32),
    Missing,
}

/// Evidence for the cursor state returned by a page. Terminal is explicit and differs
/// from Missing, so an absent provider field cannot be silently treated as exhaustion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PageNextCursor {
    More(Digest32),
    Terminal,
    Missing,
}

/// One page returned by a source query.
///
/// Cursor values are opaque digests. They are compared for exact continuity only;
/// this crate does not interpret a cursor or prove that the provider implemented it
/// honestly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExportPageReceipt {
    pub ordinal: u32,
    pub source_system: ReferenceId,
    pub scope: ScopeRef,
    pub snapshot_id: ReferenceId,
    pub query_digest: Digest32,
    pub schema_digest: Digest32,
    pub source_contract_digest: Digest32,
    pub window: ExportWindow,
    /// Initial is expected only for ordinal zero; later pages must name the prior cursor.
    pub request_cursor: PageRequestCursor,
    /// Terminal must be explicit on the final page; Missing is never treated as terminal.
    pub next_cursor: PageNextCursor,
    pub payload_digest: Digest32,
    pub returned_rows: u64,
}

/// The local extraction manifest. Mapping identity is included here because the
/// imported meaning can drift even when the provider's source schema does not.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceExportManifest {
    pub manifest_id: ReferenceId,
    pub source_system: ReferenceId,
    pub scope: ScopeRef,
    pub snapshot_id: ReferenceId,
    pub query_digest: Digest32,
    pub schema_digest: Digest32,
    pub source_contract_digest: Digest32,
    pub mapping_digest: Digest32,
    pub window: ExportWindow,
    pub pages: Vec<ExportPageReceipt>,
}

/// The statement is a provider claim, not proof of its own authenticity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProviderCoverageClaim {
    CompleteForDeclaredQuery,
    Partial,
    Unknown,
}

/// Provider-issued coverage statement to be checked against the extraction manifest.
///
/// The digest and identity references must be bound to an external authenticity
/// verifier before callers treat this statement as authenticated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProviderCoverageAttestation {
    pub attestation_id: ReferenceId,
    pub source_system: ReferenceId,
    pub scope: ScopeRef,
    pub snapshot_id: ReferenceId,
    pub query_digest: Digest32,
    pub schema_digest: Digest32,
    pub source_contract_digest: Digest32,
    pub window: ExportWindow,
    pub claim: ProviderCoverageClaim,
    pub declared_row_count: u64,
    pub issued_at_unix_ms: u64,
    pub statement_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExportCompletenessError {
    InvalidWindow,
    EmptyPageChain,
    ZeroDigest,
    InvalidPageOrdinal,
    FirstPageHasCursor,
    MissingCursorEvidence,
    CursorDiscontinuity,
    CursorCycle,
    PrematureTerminalCursor,
    FinalPageNotTerminal,
    PageSourceMismatch,
    PageScopeMismatch,
    SnapshotMismatch,
    QueryMismatch,
    SchemaMismatch,
    SourceContractMismatch,
    WindowMismatch,
    AttestationSourceMismatch,
    AttestationScopeMismatch,
    AttestationSnapshotMismatch,
    AttestationQueryMismatch,
    AttestationSchemaMismatch,
    AttestationSourceContractMismatch,
    AttestationWindowMismatch,
    AttestationDoesNotClaimComplete,
    AttestationIssuedBeforeWindowEnd,
    AttestationRowCountMismatch,
    RowCountOverflow,
    RawPagePayloadCountMismatch,
    RawPagePayloadDigestMismatch { page_ordinal: u32 },
    RawCoverageStatementDigestMismatch,
    RawPagePayloadDecodeFailed { page_ordinal: u32 },
    RawPageProjectionMismatch { page_ordinal: u32 },
    RawCoverageStatementDecodeFailed,
    DecodedCoverageStatementDigestMismatch,
}

/// Structural result only. The intentionally narrow type name avoids claiming that
/// the source, source statement, or real-world event population has been verified.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StructurallyConsistentExport {
    pub manifest_id: ReferenceId,
    pub manifest_digest: Digest32,
    pub provider_coverage_projection_digest: Digest32,
    pub evidence_digest: Digest32,
    pub attestation_id: ReferenceId,
    pub source_system: ReferenceId,
    pub scope: ScopeRef,
    pub window: ExportWindow,
    pub page_count: u32,
    pub observed_row_count: u64,
    pub source_declared_row_count: u64,
    pub query_digest: Digest32,
    pub schema_digest: Digest32,
    pub source_contract_digest: Digest32,
    pub mapping_digest: Digest32,
    pub provider_statement_digest: Digest32,
}

/// Evidence that every supplied raw page and coverage-statement byte slice hashes to
/// its declared digest. This adds byte-identity evidence but does not authenticate the
/// provider or prove that the semantic projection was derived from these bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawPayloadBoundExport {
    pub structural_assessment: StructurallyConsistentExport,
    pub raw_payload_binding_digest: Digest32,
}

fn hash_text(hasher: &mut Sha256, value: &str) {
    hash_u64(hasher, value.len() as u64);
    hasher.update(value.as_bytes());
}

fn hash_u64(hasher: &mut Sha256, value: u64) {
    hasher.update(value.to_be_bytes());
}

fn hash_u32(hasher: &mut Sha256, value: u32) {
    hasher.update(value.to_be_bytes());
}

fn hash_digest(hasher: &mut Sha256, value: &Digest32) {
    hasher.update(value.0);
}

fn hash_ref(hasher: &mut Sha256, value: &ReferenceId) {
    hash_text(hasher, value.as_str());
}

fn hash_scope(hasher: &mut Sha256, value: &ScopeRef) {
    hash_ref(hasher, &value.0);
}

fn hash_window(hasher: &mut Sha256, value: ExportWindow) {
    hash_u64(hasher, value.start_unix_ms);
    hash_u64(hasher, value.end_unix_ms);
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    let bytes: [u8; 32] = hasher.finalize().into();
    Digest32(bytes)
}

/// Deterministically bind the local manifest fields and ordered page receipts.
///
/// This digest makes the exact interpreted evidence reproducible. It does not prove
/// that the page receipts are authentic or that the provider query was complete.
pub fn source_export_manifest_digest(manifest: &SourceExportManifest) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_text(&mut hasher, "mycelix:source-export-manifest:v1");
    hash_ref(&mut hasher, &manifest.manifest_id);
    hash_ref(&mut hasher, &manifest.source_system);
    hash_scope(&mut hasher, &manifest.scope);
    hash_ref(&mut hasher, &manifest.snapshot_id);
    hash_digest(&mut hasher, &manifest.query_digest);
    hash_digest(&mut hasher, &manifest.schema_digest);
    hash_digest(&mut hasher, &manifest.source_contract_digest);
    hash_digest(&mut hasher, &manifest.mapping_digest);
    hash_window(&mut hasher, manifest.window);
    hash_u64(&mut hasher, manifest.pages.len() as u64);

    for page in &manifest.pages {
        hash_u32(&mut hasher, page.ordinal);
        hash_ref(&mut hasher, &page.source_system);
        hash_scope(&mut hasher, &page.scope);
        hash_ref(&mut hasher, &page.snapshot_id);
        hash_digest(&mut hasher, &page.query_digest);
        hash_digest(&mut hasher, &page.schema_digest);
        hash_digest(&mut hasher, &page.source_contract_digest);
        hash_window(&mut hasher, page.window);
        match page.request_cursor {
            PageRequestCursor::Initial => {
                hasher.update([0]);
            }
            PageRequestCursor::After(value) => {
                hasher.update([1]);
                hash_digest(&mut hasher, &value);
            }
            PageRequestCursor::Missing => {
                hasher.update([2]);
            }
        }
        match page.next_cursor {
            PageNextCursor::More(value) => {
                hasher.update([0]);
                hash_digest(&mut hasher, &value);
            }
            PageNextCursor::Terminal => {
                hasher.update([1]);
            }
            PageNextCursor::Missing => {
                hasher.update([2]);
            }
        }
        hash_digest(&mut hasher, &page.payload_digest);
        hash_u64(&mut hasher, page.returned_rows);
    }

    finish_digest(hasher)
}

/// Bind the parsed provider-coverage projection, including the digest of the source
/// statement it purports to represent. External authentication must still verify the
/// exact statement bytes and issuer and must establish that parsing produced this
/// projection; a digest over caller-supplied fields is not an authenticity proof.
pub fn provider_coverage_projection_digest(
    attestation: &ProviderCoverageAttestation,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_text(&mut hasher, "mycelix:provider-coverage-projection:v1");
    hash_ref(&mut hasher, &attestation.attestation_id);
    hash_ref(&mut hasher, &attestation.source_system);
    hash_scope(&mut hasher, &attestation.scope);
    hash_ref(&mut hasher, &attestation.snapshot_id);
    hash_digest(&mut hasher, &attestation.query_digest);
    hash_digest(&mut hasher, &attestation.schema_digest);
    hash_digest(&mut hasher, &attestation.source_contract_digest);
    hash_window(&mut hasher, attestation.window);
    hasher.update([match attestation.claim {
        ProviderCoverageClaim::CompleteForDeclaredQuery => 0,
        ProviderCoverageClaim::Partial => 1,
        ProviderCoverageClaim::Unknown => 2,
    }]);
    hash_u64(&mut hasher, attestation.declared_row_count);
    hash_u64(&mut hasher, attestation.issued_at_unix_ms);
    hash_digest(&mut hasher, &attestation.statement_digest);
    finish_digest(hasher)
}

fn combined_evidence_digest(
    manifest_digest: &Digest32,
    projection_digest: &Digest32,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_text(&mut hasher, "mycelix:hospitality-export-completeness-evidence:v1");
    hash_digest(&mut hasher, manifest_digest);
    hash_digest(&mut hasher, projection_digest);
    finish_digest(hasher)
}

/// Validate the supplied pagination chain and compare it to a provider coverage claim.
///
/// A successful result establishes only structural consistency between the supplied
/// evidence artifacts. It must not be interpreted as proof of provider authenticity,
/// semantic correctness, external completeness, or permission to execute business
/// actions.
pub fn assess_export_completeness(
    manifest: &SourceExportManifest,
    attestation: &ProviderCoverageAttestation,
) -> Result<StructurallyConsistentExport, ExportCompletenessError> {
    manifest.window.validate()?;
    attestation.window.validate()?;

    if manifest.pages.is_empty() {
        return Err(ExportCompletenessError::EmptyPageChain);
    }
    if is_zero(&manifest.query_digest)
        || is_zero(&manifest.schema_digest)
        || is_zero(&manifest.source_contract_digest)
        || is_zero(&manifest.mapping_digest)
        || is_zero(&attestation.query_digest)
        || is_zero(&attestation.schema_digest)
        || is_zero(&attestation.source_contract_digest)
        || is_zero(&attestation.statement_digest)
    {
        return Err(ExportCompletenessError::ZeroDigest);
    }

    let mut total_rows = 0_u64;
    let mut seen_cursors = BTreeSet::new();

    for (index, page) in manifest.pages.iter().enumerate() {
        let expected_ordinal =
            u32::try_from(index).map_err(|_| ExportCompletenessError::InvalidPageOrdinal)?;
        if page.ordinal != expected_ordinal {
            return Err(ExportCompletenessError::InvalidPageOrdinal);
        }
        if page.source_system != manifest.source_system {
            return Err(ExportCompletenessError::PageSourceMismatch);
        }
        if page.scope != manifest.scope {
            return Err(ExportCompletenessError::PageScopeMismatch);
        }
        if page.snapshot_id != manifest.snapshot_id {
            return Err(ExportCompletenessError::SnapshotMismatch);
        }
        if page.query_digest != manifest.query_digest {
            return Err(ExportCompletenessError::QueryMismatch);
        }
        if page.schema_digest != manifest.schema_digest {
            return Err(ExportCompletenessError::SchemaMismatch);
        }
        if page.source_contract_digest != manifest.source_contract_digest {
            return Err(ExportCompletenessError::SourceContractMismatch);
        }
        if page.window != manifest.window {
            return Err(ExportCompletenessError::WindowMismatch);
        }
        if is_zero(&page.payload_digest) {
            return Err(ExportCompletenessError::ZeroDigest);
        }

        if index == 0 {
            match page.request_cursor {
                PageRequestCursor::Initial => {}
                PageRequestCursor::After(_) => {
                    return Err(ExportCompletenessError::FirstPageHasCursor);
                }
                PageRequestCursor::Missing => {
                    return Err(ExportCompletenessError::MissingCursorEvidence);
                }
            }
        } else {
            let previous = &manifest.pages[index - 1];
            match (page.request_cursor, previous.next_cursor) {
                (PageRequestCursor::After(requested), PageNextCursor::More(returned))
                    if requested == returned => {}
                (PageRequestCursor::Missing, _) | (_, PageNextCursor::Missing) => {
                    return Err(ExportCompletenessError::MissingCursorEvidence);
                }
                _ => return Err(ExportCompletenessError::CursorDiscontinuity),
            }
        }

        if let PageRequestCursor::After(cursor) = page.request_cursor {
            if is_zero(&cursor) {
                return Err(ExportCompletenessError::ZeroDigest);
            }
        }

        let is_last = index + 1 == manifest.pages.len();
        match (page.next_cursor, is_last) {
            (PageNextCursor::Missing, _) => {
                return Err(ExportCompletenessError::MissingCursorEvidence);
            }
            (PageNextCursor::Terminal, false) => {
                return Err(ExportCompletenessError::PrematureTerminalCursor);
            }
            (PageNextCursor::More(_), true) => {
                return Err(ExportCompletenessError::FinalPageNotTerminal);
            }
            (PageNextCursor::Terminal, true) => {}
            (PageNextCursor::More(cursor), false) => {
                if is_zero(&cursor) {
                    return Err(ExportCompletenessError::ZeroDigest);
                }
                if !seen_cursors.insert(cursor) {
                    return Err(ExportCompletenessError::CursorCycle);
                }
                if page.request_cursor == PageRequestCursor::After(cursor) {
                    return Err(ExportCompletenessError::CursorCycle);
                }
            }
        }

        total_rows = total_rows
            .checked_add(page.returned_rows)
            .ok_or(ExportCompletenessError::RowCountOverflow)?;
    }

    if attestation.source_system != manifest.source_system {
        return Err(ExportCompletenessError::AttestationSourceMismatch);
    }
    if attestation.scope != manifest.scope {
        return Err(ExportCompletenessError::AttestationScopeMismatch);
    }
    if attestation.snapshot_id != manifest.snapshot_id {
        return Err(ExportCompletenessError::AttestationSnapshotMismatch);
    }
    if attestation.query_digest != manifest.query_digest {
        return Err(ExportCompletenessError::AttestationQueryMismatch);
    }
    if attestation.schema_digest != manifest.schema_digest {
        return Err(ExportCompletenessError::AttestationSchemaMismatch);
    }
    if attestation.source_contract_digest != manifest.source_contract_digest {
        return Err(ExportCompletenessError::AttestationSourceContractMismatch);
    }
    if attestation.window != manifest.window {
        return Err(ExportCompletenessError::AttestationWindowMismatch);
    }
    if attestation.claim != ProviderCoverageClaim::CompleteForDeclaredQuery {
        return Err(ExportCompletenessError::AttestationDoesNotClaimComplete);
    }
    if attestation.issued_at_unix_ms == 0
        || attestation.issued_at_unix_ms < manifest.window.end_unix_ms
    {
        return Err(ExportCompletenessError::AttestationIssuedBeforeWindowEnd);
    }
    if attestation.declared_row_count != total_rows {
        return Err(ExportCompletenessError::AttestationRowCountMismatch);
    }

    let manifest_digest = source_export_manifest_digest(manifest);
    let projection_digest = provider_coverage_projection_digest(attestation);
    let evidence_digest = combined_evidence_digest(&manifest_digest, &projection_digest);

    Ok(StructurallyConsistentExport {
        manifest_id: manifest.manifest_id.clone(),
        manifest_digest,
        provider_coverage_projection_digest: projection_digest,
        evidence_digest,
        attestation_id: attestation.attestation_id.clone(),
        source_system: manifest.source_system.clone(),
        scope: manifest.scope.clone(),
        window: manifest.window,
        page_count: u32::try_from(manifest.pages.len())
            .map_err(|_| ExportCompletenessError::InvalidPageOrdinal)?,
        observed_row_count: total_rows,
        source_declared_row_count: attestation.declared_row_count,
        query_digest: manifest.query_digest,
        schema_digest: manifest.schema_digest,
        source_contract_digest: manifest.source_contract_digest,
        mapping_digest: manifest.mapping_digest,
        provider_statement_digest: attestation.statement_digest,
    })
}

/// Assess the export and verify that supplied raw page payloads and the raw
/// coverage-statement bytes match the SHA-256 digests recorded in the evidence.
///
/// This binds declared digests to exact byte slices. It does not authenticate the
/// provider, prove that the parser produced the supplied semantic projection from
/// those bytes, validate query semantics, or establish real-world completeness.
pub fn assess_export_completeness_with_raw_payloads(
    manifest: &SourceExportManifest,
    attestation: &ProviderCoverageAttestation,
    raw_page_payloads: &[&[u8]],
    raw_coverage_statement: &[u8],
) -> Result<RawPayloadBoundExport, ExportCompletenessError> {
    // Preserve the existing structural validator's error precedence.
    let assessment = assess_export_completeness(manifest, attestation)?;

    if raw_page_payloads.len() != manifest.pages.len() {
        return Err(ExportCompletenessError::RawPagePayloadCountMismatch);
    }

    let mut binding_hasher = Sha256::new();
    hash_text(&mut binding_hasher, "mycelix:raw-export-payload-binding:v1");
    hash_digest(&mut binding_hasher, &assessment.manifest_digest);
    hash_ref(&mut binding_hasher, &attestation.attestation_id);
    hash_u64(&mut binding_hasher, raw_page_payloads.len() as u64);

    for (page, payload) in manifest.pages.iter().zip(raw_page_payloads) {
        let raw_digest = sha256_raw_bytes(*payload);
        if raw_digest != page.payload_digest {
            return Err(ExportCompletenessError::RawPagePayloadDigestMismatch {
                page_ordinal: page.ordinal,
            });
        }
        hash_u32(&mut binding_hasher, page.ordinal);
        hash_digest(&mut binding_hasher, &raw_digest);
    }

    let raw_statement_digest = sha256_raw_bytes(raw_coverage_statement);
    if raw_statement_digest != attestation.statement_digest {
        return Err(ExportCompletenessError::RawCoverageStatementDigestMismatch);
    }
    hash_digest(&mut binding_hasher, &raw_statement_digest);

    Ok(RawPayloadBoundExport {
        structural_assessment: assessment,
        raw_payload_binding_digest: finish_digest(binding_hasher),
    })
}

/// Decode raw source artifacts at the boundary where their exact bytes are available,
/// then qualify the decoded projections against the manifest.
///
/// The page decoder is called once per supplied page, in manifest order. Each decoded
/// receipt must exactly equal the corresponding manifest receipt, including the digest
/// of that raw page. The coverage decoder receives the exact statement bytes and its
/// returned statement digest must equal SHA-256 of those bytes. This removes the need
/// for callers to pass unrelated pre-decoded projections to this entry point, but it
/// still trusts the supplied decoder implementations to interpret bytes correctly.
/// This function does not authenticate the provider or establish query semantics or
/// real-world completeness.
pub fn assess_export_completeness_from_raw_artifacts<DecodePage, DecodeCoverage>(
    manifest: &SourceExportManifest,
    raw_page_payloads: &[&[u8]],
    raw_coverage_statement: &[u8],
    mut decode_page: DecodePage,
    decode_coverage_statement: DecodeCoverage,
) -> Result<RawPayloadBoundExport, ExportCompletenessError>
where
    DecodePage: FnMut(&[u8]) -> Option<ExportPageReceipt>,
    DecodeCoverage: FnOnce(&[u8]) -> Option<ProviderCoverageAttestation>,
{
    if raw_page_payloads.len() != manifest.pages.len() {
        return Err(ExportCompletenessError::RawPagePayloadCountMismatch);
    }

    for (page, raw_payload) in manifest.pages.iter().zip(raw_page_payloads) {
        if sha256_raw_bytes(raw_payload) != page.payload_digest {
            return Err(ExportCompletenessError::RawPagePayloadDigestMismatch {
                page_ordinal: page.ordinal,
            });
        }

        let decoded = decode_page(raw_payload).ok_or(
            ExportCompletenessError::RawPagePayloadDecodeFailed {
                page_ordinal: page.ordinal,
            },
        )?;
        if &decoded != page {
            return Err(ExportCompletenessError::RawPageProjectionMismatch {
                page_ordinal: page.ordinal,
            });
        }
    }

    let attestation = decode_coverage_statement(raw_coverage_statement)
        .ok_or(ExportCompletenessError::RawCoverageStatementDecodeFailed)?;
    if sha256_raw_bytes(raw_coverage_statement) != attestation.statement_digest {
        return Err(ExportCompletenessError::DecodedCoverageStatementDigestMismatch);
    }

    assess_export_completeness_with_raw_payloads(
        manifest,
        &attestation,
        raw_page_payloads,
        raw_coverage_statement,
    )
}

fn sha256_raw_bytes(bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    finish_digest(hasher)
}

fn is_zero(digest: &Digest32) -> bool {
    digest == &Digest32([0; 32])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ref_id(value: &str) -> ReferenceId {
        ReferenceId::new(value).expect("test references are canonical")
    }

    fn scope(value: &str) -> ScopeRef {
        ScopeRef(ref_id(value))
    }

    fn digest(value: u8) -> Digest32 {
        Digest32::repeat(value)
    }

    fn fixture() -> (SourceExportManifest, ProviderCoverageAttestation) {
        let source_system = ref_id("source:pos:test");
        let scope = scope("scope:restaurant:one");
        let snapshot_id = ref_id("snapshot:2026-09");
        let query_digest = digest(1);
        let schema_digest = digest(2);
        let source_contract_digest = digest(8);
        let mapping_digest = digest(3);
        let window = ExportWindow {
            start_unix_ms: 1_000,
            end_unix_ms: 2_000,
        };

        let manifest = SourceExportManifest {
            manifest_id: ref_id("manifest:test:one"),
            source_system: source_system.clone(),
            scope: scope.clone(),
            snapshot_id: snapshot_id.clone(),
            query_digest,
            schema_digest,
            source_contract_digest,
            mapping_digest,
            window,
            pages: vec![
                ExportPageReceipt {
                    ordinal: 0,
                    source_system: source_system.clone(),
                    scope: scope.clone(),
                    snapshot_id: snapshot_id.clone(),
                    query_digest,
                    schema_digest,
                    source_contract_digest,
                    window,
                    request_cursor: PageRequestCursor::Initial,
                    next_cursor: PageNextCursor::More(digest(4)),
                    payload_digest: digest(5),
                    returned_rows: 3,
                },
                ExportPageReceipt {
                    ordinal: 1,
                    source_system: source_system.clone(),
                    scope: scope.clone(),
                    snapshot_id: snapshot_id.clone(),
                    query_digest,
                    schema_digest,
                    source_contract_digest,
                    window,
                    request_cursor: PageRequestCursor::After(digest(4)),
                    next_cursor: PageNextCursor::Terminal,
                    payload_digest: digest(6),
                    returned_rows: 2,
                },
            ],
        };

        let attestation = ProviderCoverageAttestation {
            attestation_id: ref_id("attestation:test:one"),
            source_system,
            scope,
            snapshot_id,
            query_digest,
            schema_digest,
            source_contract_digest,
            window,
            claim: ProviderCoverageClaim::CompleteForDeclaredQuery,
            declared_row_count: 5,
            issued_at_unix_ms: 2_100,
            statement_digest: digest(7),
        };

        (manifest, attestation)
    }

    fn raw_payload_fixture() -> (
        SourceExportManifest,
        ProviderCoverageAttestation,
        [&'static [u8]; 2],
        &'static [u8],
    ) {
        let (mut manifest, mut attestation) = fixture();
        let page_payloads: [&[u8]; 2] = [
            &b"page-zero-payload"[..],
            &b"page-one-payload"[..],
        ];
        let statement: &[u8] = b"provider-coverage-statement-v1";

        for (page, payload) in manifest.pages.iter_mut().zip(page_payloads.iter()) {
            page.payload_digest = sha256_raw_bytes(*payload);
        }
        attestation.statement_digest = sha256_raw_bytes(statement);

        (manifest, attestation, page_payloads, statement)
    }

    #[test]
    fn raw_payload_assessment_accepts_exact_page_and_statement_bytes() {
        let (manifest, attestation, pages, statement) = raw_payload_fixture();
        let result = assess_export_completeness_with_raw_payloads(
            &manifest,
            &attestation,
            &pages,
            statement,
        )
        .unwrap();

        assert_eq!(result.structural_assessment.page_count, 2);
        assert_eq!(result.structural_assessment.observed_row_count, 5);
        assert_ne!(result.raw_payload_binding_digest, digest(0));
    }

    #[test]
    fn raw_payload_binding_digest_is_deterministic() {
        let (manifest, attestation, pages, statement) = raw_payload_fixture();
        let first =
            assess_export_completeness_with_raw_payloads(&manifest, &attestation, &pages, statement)
                .unwrap();
        let second =
            assess_export_completeness_with_raw_payloads(&manifest, &attestation, &pages, statement)
                .unwrap();

        assert_eq!(first.raw_payload_binding_digest, second.raw_payload_binding_digest);
    }

    #[test]
    fn raw_payload_assessment_rejects_page_bytes_that_do_not_match_manifest() {
        let (manifest, attestation, mut pages, statement) = raw_payload_fixture();
        pages[0] = &b"tampered-page-zero"[..];

        assert_eq!(
            assess_export_completeness_with_raw_payloads(
                &manifest,
                &attestation,
                &pages,
                statement,
            ),
            Err(ExportCompletenessError::RawPagePayloadDigestMismatch { page_ordinal: 0 })
        );
    }

    #[test]
    fn raw_payload_assessment_rejects_wrong_statement_bytes() {
        let (manifest, attestation, pages, _) = raw_payload_fixture();

        assert_eq!(
            assess_export_completeness_with_raw_payloads(
                &manifest,
                &attestation,
                &pages,
                b"different-statement",
            ),
            Err(ExportCompletenessError::RawCoverageStatementDigestMismatch)
        );
    }

    #[test]
    fn raw_payload_assessment_rejects_missing_page_bytes() {
        let (manifest, attestation, pages, statement) = raw_payload_fixture();

        assert_eq!(
            assess_export_completeness_with_raw_payloads(
                &manifest,
                &attestation,
                &pages[..1],
                statement,
            ),
            Err(ExportCompletenessError::RawPagePayloadCountMismatch)
        );
    }

    #[test]
    fn raw_artifact_entry_point_checks_decoded_page_and_statement_projections() {
        let (manifest, attestation, pages, statement) = raw_payload_fixture();
        let result = assess_export_completeness_from_raw_artifacts(
            &manifest,
            &pages,
            statement,
            |raw| {
                if raw == b"page-zero-payload" {
                    Some(manifest.pages[0].clone())
                } else if raw == b"page-one-payload" {
                    Some(manifest.pages[1].clone())
                } else {
                    None
                }
            },
            |raw| {
                if raw == statement {
                    Some(attestation.clone())
                } else {
                    None
                }
            },
        )
        .unwrap();

        assert_eq!(result.structural_assessment.page_count, 2);
        assert_eq!(result.structural_assessment.observed_row_count, 5);
    }

    #[test]
    fn raw_artifact_entry_point_rejects_decoded_page_projection_drift() {
        let (manifest, attestation, pages, statement) = raw_payload_fixture();
        assert_eq!(
            assess_export_completeness_from_raw_artifacts(
                &manifest,
                &pages,
                statement,
                |raw| {
                    if raw == b"page-zero-payload" {
                        Some(manifest.pages[0].clone())
                    } else if raw == b"page-one-payload" {
                        let mut changed = manifest.pages[1].clone();
                        changed.returned_rows += 1;
                        Some(changed)
                    } else {
                        None
                    }
                },
                |_| Some(attestation.clone()),
            ),
            Err(ExportCompletenessError::RawPageProjectionMismatch { page_ordinal: 1 })
        );
    }

    #[test]
    fn raw_artifact_entry_point_rejects_unbound_decoded_statement() {
        let (manifest, mut attestation, pages, statement) = raw_payload_fixture();
        attestation.statement_digest = digest(7);
        assert_eq!(
            assess_export_completeness_from_raw_artifacts(
                &manifest,
                &pages,
                statement,
                |raw| {
                    if raw == b"page-zero-payload" {
                        Some(manifest.pages[0].clone())
                    } else if raw == b"page-one-payload" {
                        Some(manifest.pages[1].clone())
                    } else {
                        None
                    }
                },
                |_| Some(attestation.clone()),
            ),
            Err(ExportCompletenessError::DecodedCoverageStatementDigestMismatch)
        );
    }

    #[test]
    fn raw_artifact_entry_point_rejects_decode_failures() {
        let (manifest, attestation, pages, statement) = raw_payload_fixture();
        assert_eq!(
            assess_export_completeness_from_raw_artifacts(
                &manifest,
                &pages,
                statement,
                |_| None,
                |_| Some(attestation.clone()),
            ),
            Err(ExportCompletenessError::RawPagePayloadDecodeFailed { page_ordinal: 0 })
        );
        assert_eq!(
            assess_export_completeness_from_raw_artifacts(
                &manifest,
                &pages,
                statement,
                |raw| {
                    if raw == b"page-zero-payload" {
                        Some(manifest.pages[0].clone())
                    } else if raw == b"page-one-payload" {
                        Some(manifest.pages[1].clone())
                    } else {
                        None
                    }
                },
                |_| None,
            ),
            Err(ExportCompletenessError::RawCoverageStatementDecodeFailed)
        );
    }

    #[test]
    fn accepts_consistent_complete_cursor_chain() {
        let (manifest, attestation) = fixture();
        let result = assess_export_completeness(&manifest, &attestation).unwrap();

        assert_eq!(result.page_count, 2);
        assert_eq!(result.observed_row_count, 5);
        assert_eq!(result.source_declared_row_count, 5);
        assert_eq!(result.manifest_id, manifest.manifest_id);
    }

    #[test]
    fn evidence_digests_are_deterministic_and_bind_all_fields() {
        let (manifest, attestation) = fixture();

        assert_eq!(
            source_export_manifest_digest(&manifest),
            source_export_manifest_digest(&manifest)
        );
        assert_eq!(
            provider_coverage_projection_digest(&attestation),
            provider_coverage_projection_digest(&attestation)
        );

        let mut changed_manifest = manifest.clone();
        changed_manifest.pages[0].returned_rows += 1;
        assert_ne!(
            source_export_manifest_digest(&manifest),
            source_export_manifest_digest(&changed_manifest)
        );

        let mut changed_attestation = attestation.clone();
        changed_attestation.declared_row_count += 1;
        assert_ne!(
            provider_coverage_projection_digest(&attestation),
            provider_coverage_projection_digest(&changed_attestation)
        );
    }

    #[test]
    fn assessment_exposes_reproducible_evidence_digests() {
        let (manifest, attestation) = fixture();
        let result = assess_export_completeness(&manifest, &attestation).unwrap();

        assert_eq!(result.manifest_digest, source_export_manifest_digest(&manifest));
        assert_eq!(
            result.provider_coverage_projection_digest,
            provider_coverage_projection_digest(&attestation)
        );
        assert_ne!(result.evidence_digest, digest(0));
    }

    #[test]
    fn rejects_empty_chain() {
        let (mut manifest, attestation) = fixture();
        manifest.pages.clear();

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::EmptyPageChain)
        );
    }

    #[test]
    fn rejects_non_contiguous_ordinals() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].ordinal = 3;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::InvalidPageOrdinal)
        );
    }

    #[test]
    fn rejects_cursor_gap_between_pages() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].request_cursor = PageRequestCursor::After(digest(8));

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::CursorDiscontinuity)
        );
    }

    #[test]
    fn rejects_early_terminal_cursor() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[0].next_cursor = PageNextCursor::Terminal;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::PrematureTerminalCursor)
        );
    }

    #[test]
    fn rejects_nonterminal_last_page() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].next_cursor = PageNextCursor::More(digest(9));

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::FinalPageNotTerminal)
        );
    }

    #[test]
    fn rejects_missing_terminal_cursor_evidence() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].next_cursor = PageNextCursor::Missing;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::MissingCursorEvidence)
        );
    }

    #[test]
    fn rejects_missing_request_cursor_evidence() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].request_cursor = PageRequestCursor::Missing;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::MissingCursorEvidence)
        );
    }

    #[test]
    fn rejects_snapshot_drift() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].snapshot_id = ref_id("snapshot:different");

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::SnapshotMismatch)
        );
    }

    #[test]
    fn rejects_invalid_half_open_window() {
        let (mut manifest, attestation) = fixture();
        manifest.window.end_unix_ms = manifest.window.start_unix_ms;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::InvalidWindow)
        );
    }

    #[test]
    fn rejects_zero_page_payload_digest() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[0].payload_digest = Digest32([0; 32]);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::ZeroDigest)
        );
    }

    #[test]
    fn rejects_page_scope_drift() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].scope = scope("scope:restaurant:two");

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::PageScopeMismatch)
        );
    }

    #[test]
    fn rejects_page_schema_drift() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].schema_digest = digest(9);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::SchemaMismatch)
        );
    }

    #[test]
    fn rejects_attestation_source_mismatch() {
        let (manifest, mut attestation) = fixture();
        attestation.source_system = ref_id("source:pos:other");

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationSourceMismatch)
        );
    }

    #[test]
    fn rejects_attestation_scope_mismatch() {
        let (manifest, mut attestation) = fixture();
        attestation.scope = scope("scope:restaurant:two");

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationScopeMismatch)
        );
    }

    #[test]
    fn rejects_attestation_snapshot_mismatch() {
        let (manifest, mut attestation) = fixture();
        attestation.snapshot_id = ref_id("snapshot:different");

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationSnapshotMismatch)
        );
    }

    #[test]
    fn rejects_attestation_query_mismatch() {
        let (manifest, mut attestation) = fixture();
        attestation.query_digest = digest(9);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationQueryMismatch)
        );
    }

    #[test]
    fn rejects_attestation_schema_mismatch() {
        let (manifest, mut attestation) = fixture();
        attestation.schema_digest = digest(9);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationSchemaMismatch)
        );
    }

    #[test]
    fn rejects_attestation_window_mismatch() {
        let (manifest, mut attestation) = fixture();
        attestation.window.start_unix_ms = 999;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationWindowMismatch)
        );
    }

    #[test]
    fn rejects_query_drift() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].query_digest = digest(8);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::QueryMismatch)
        );
    }

    #[test]
    fn rejects_source_contract_drift_on_page() {
        let (mut manifest, attestation) = fixture();
        manifest.pages[1].source_contract_digest = digest(9);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::SourceContractMismatch)
        );
    }

    #[test]
    fn rejects_attestation_for_different_source_contract() {
        let (manifest, mut attestation) = fixture();
        attestation.source_contract_digest = digest(9);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationSourceContractMismatch)
        );
    }

    #[test]
    fn rejects_cursor_cycle() {
        let (mut manifest, attestation) = fixture();
        let mut third = manifest.pages[1].clone();
        third.ordinal = 2;
        third.request_cursor = PageRequestCursor::After(digest(4));
        third.next_cursor = PageNextCursor::Terminal;
        manifest.pages[1].next_cursor = PageNextCursor::More(digest(4));
        manifest.pages.push(third);

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::CursorCycle)
        );
    }

    #[test]
    fn rejects_partial_provider_claim() {
        let (manifest, mut attestation) = fixture();
        attestation.claim = ProviderCoverageClaim::Partial;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationDoesNotClaimComplete)
        );
    }

    #[test]
    fn rejects_row_count_mismatch() {
        let (manifest, mut attestation) = fixture();
        attestation.declared_row_count = 4;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationRowCountMismatch)
        );
    }

    #[test]
    fn rejects_attestation_issued_before_window_closes() {
        let (manifest, mut attestation) = fixture();
        attestation.issued_at_unix_ms = 1_999;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::AttestationIssuedBeforeWindowEnd)
        );
    }

    #[test]
    fn rejects_row_count_overflow() {
        let (mut manifest, mut attestation) = fixture();
        manifest.pages[0].returned_rows = u64::MAX;
        manifest.pages[1].returned_rows = 1;
        attestation.declared_row_count = u64::MAX;

        assert_eq!(
            assess_export_completeness(&manifest, &attestation),
            Err(ExportCompletenessError::RowCountOverflow)
        );
    }
}

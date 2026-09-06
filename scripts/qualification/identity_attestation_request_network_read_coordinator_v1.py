#!/usr/bin/env python3
"""Materialize the network-aware get_pending_requests coordinator adapter.

Qualification-only: this edits the CI checkout, never committed product source.
Every replacement requires exactly one expected before-image so stale or
ambiguous transformations fail closed.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
IDENTITY = ROOT / "mycelix-workspace/mycelix-identity"
MANIFEST = IDENTITY / "zomes/trust_credential/coordinator/Cargo.toml"
LIB = IDENTITY / "zomes/trust_credential/coordinator/src/lib.rs"


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse stale/ambiguous coordinator transformation"
        )
    return text.replace(before, after, 1)


manifest = MANIFEST.read_text()
manifest = replace_once(
    manifest,
    '''mycelix-zome-helpers = { workspace = true }
hdk = { workspace = true }
trust_credential_integrity = { path = "../integrity" }
''',
    '''mycelix-zome-helpers = { workspace = true }
hdk = { workspace = true }
mycelix-attestation-request-evidence-gatherer = { path = "../../../crates/attestation-request-evidence-gatherer" }
trust_credential_integrity = { path = "../integrity" }
''',
    "coordinator evidence-gatherer dependency",
)

text = LIB.read_text()
text = replace_once(
    text,
    '''use hdk::prelude::*;
use mycelix_zome_helpers as _;
use trust_credential_integrity::*;
''',
    '''use hdk::prelude::*;
use mycelix_attestation_request_evidence_gatherer::{
    ObservedAttestationRequestStateV1, resolve_subject_attestation_requests_v1,
};
use mycelix_zome_helpers as _;
use trust_credential_integrity::*;
''',
    "coordinator evidence-gatherer import",
)

old_get_pending = '''pub fn get_pending_requests(subject_did: String) -> ExternResult<Vec<Record>> {
    if subject_did.is_empty() || subject_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Subject DID must be 1-256 characters".into()
        )));
    }
    let now = sys_time()?;

    // Use chain query to get the latest version of each AttestationRequest
    // (link-based lookup returns the original action hash, missing updates)
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::AttestationRequest,
        )?))
        .include_entries(true);

    let records = query(filter)?;

    // Track seen IDs to only keep the latest version of each request
    let mut seen_ids = std::collections::HashSet::new();
    let mut pending = Vec::new();

    // Chain query returns chronological order; iterate in reverse to see latest first
    for record in records.into_iter().rev() {
        if let Some(req) = record
            .entry()
            .to_app_option::<AttestationRequest>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            if seen_ids.contains(&req.id) {
                continue; // Already processed a newer version
            }
            seen_ids.insert(req.id.clone());
            if req.subject_did == subject_did
                && req.status == AttestationStatus::Pending
                && now < req.expires_at
            {
                pending.push(record);
            }
        }
    }

    Ok(pending)
}
'''
new_get_pending = '''pub fn get_pending_requests(subject_did: String) -> ExternResult<Vec<Record>> {
    let observed = resolve_subject_attestation_requests_v1(&subject_did)?;
    let mut pending = Vec::new();

    for request in observed {
        match request.state {
            ObservedAttestationRequestStateV1::ObservedPending => {
                // The gatherer proves this root is observation-scoped Pending.
                // The creation record is provenance, not a current-state payload.
                pending.push(request.canonical_root_record);
            }
            ObservedAttestationRequestStateV1::ObservedConflict { terminal_updates } => {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "Attestation request conflict observed ({terminal_updates} terminal updates); pending state is unresolved"
                ))));
            }
            ObservedAttestationRequestStateV1::ObservedFulfilled
            | ObservedAttestationRequestStateV1::ObservedDeclined
            | ObservedAttestationRequestStateV1::ObservedExpired
            | ObservedAttestationRequestStateV1::ObservedCancelled
            | ObservedAttestationRequestStateV1::ObservedExpiredByTime => {}
        }
    }

    Ok(pending)
}
'''
text = replace_once(
    text,
    old_get_pending,
    new_get_pending,
    "network-aware get_pending_requests adapter",
)

required_manifest = [
    'mycelix-attestation-request-evidence-gatherer = { path = "../../../crates/attestation-request-evidence-gatherer" }',
]
for needle in required_manifest:
    if needle not in manifest:
        raise SystemExit(f"ERROR: generated coordinator manifest missing: {needle!r}")

required_source = [
    "use mycelix_attestation_request_evidence_gatherer::{",
    "ObservedAttestationRequestStateV1, resolve_subject_attestation_requests_v1,",
    "let observed = resolve_subject_attestation_requests_v1(&subject_did)?;",
    "ObservedAttestationRequestStateV1::ObservedPending",
    "pending.push(request.canonical_root_record);",
    "ObservedAttestationRequestStateV1::ObservedConflict { terminal_updates }",
    "pending state is unresolved",
    "ObservedAttestationRequestStateV1::ObservedFulfilled",
    "ObservedAttestationRequestStateV1::ObservedDeclined",
    "ObservedAttestationRequestStateV1::ObservedExpired",
    "ObservedAttestationRequestStateV1::ObservedCancelled",
    "ObservedAttestationRequestStateV1::ObservedExpiredByTime",
]
for needle in required_source:
    if needle not in text:
        raise SystemExit(f"ERROR: generated coordinator adapter missing: {needle!r}")

start = text.index("pub fn get_pending_requests(subject_did: String)")
end = text.index("/// Get credentials for a subject", start)
body = text[start:end]
for forbidden in [
    "ChainQueryFilter",
    "query(filter)",
    "seen_ids",
    ".into_iter().rev()",
    "AttestationStatus::Pending",
    "sys_time()",
    "to_app_option::<AttestationRequest>()",
    "get_links(",
    "get_details(",
]:
    if forbidden in body:
        raise SystemExit(
            f"ERROR: local traversal/winner/current-state semantics leaked into get_pending_requests: {forbidden!r}"
        )

MANIFEST.write_text(manifest)
LIB.write_text(text)
print("Materialized network-aware pending-request coordinator adapter against exact before-images.")

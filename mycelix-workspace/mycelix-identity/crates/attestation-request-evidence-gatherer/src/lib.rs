// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Mechanical HDK evidence gathering for observed attestation-request state.
//!
//! This crate owns network traversal only. It does not own request-root shape,
//! lifecycle transitions, chronology winners, delete-as-cancel semantics, or
//! duplicate-ID policy. Those semantics live in the canonical policy/runtime
//! crates beneath this layer.

#![forbid(unsafe_code)]

use hdk::prelude::*;
use mycelix_attestation_request_resolution_runtime::{
    AttestationRequestLineageObservationV1, AttestationRequestUpdateObservationV1,
    resolve_attestation_request_lineage_v1, validate_unique_request_ids_v1,
};
pub use mycelix_attestation_request_resolution_runtime::ObservedAttestationRequestStateV1;
use trust_credential_integrity::{AttestationRequest, LinkTypes};

/// One fully gathered and semantically resolved request root.
///
/// `canonical_root_record` is the immutable creation record and therefore still
/// contains `Pending` in its embedded request entry. It is **not** a current-state
/// record. Callers must use `state` for the observed lifecycle interpretation.
///
/// `state` remains observation-scoped; network strategy does not imply global
/// completeness of the DHT at the instant of this call.
pub struct ObservedAttestationRequestRecordV1 {
    pub root_action_hash: ActionHash,
    pub canonical_root_record: Record,
    pub state: ObservedAttestationRequestStateV1,
}

struct OwnedUpdateObservationV1 {
    request: AttestationRequest,
    author_did: String,
    action_timestamp_micros: i64,
}

struct GatheredRootV1 {
    root_action_hash: ActionHash,
    canonical_root_record: Record,
    root_request: AttestationRequest,
    root_author_did: String,
    updates: Vec<OwnedUpdateObservationV1>,
    observed_delete_count: usize,
}

fn guest_error(message: impl Into<String>) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(message.into()))
}

fn push_unique_hash(hashes: &mut Vec<ActionHash>, hash: ActionHash) {
    if !hashes.contains(&hash) {
        hashes.push(hash);
    }
}

fn unique_action_hashes(actions: &[SignedActionHashed]) -> Vec<ActionHash> {
    let mut hashes = Vec::new();
    for action in actions {
        push_unique_hash(&mut hashes, action.as_hash().clone());
    }
    hashes
}

fn request_anchor(subject_did: &str) -> EntryHash {
    let bytes = holo_hash::blake2b_256(format!("requests:{subject_did}").as_bytes());
    EntryHash::from_raw_32(bytes.to_vec())
}

fn validate_subject_query(subject_did: &str) -> ExternResult<()> {
    if subject_did.is_empty() || subject_did.len() > 256 || !subject_did.starts_with("did:") {
        return Err(guest_error("Subject DID must be a valid DID of 1-256 characters"));
    }
    Ok(())
}

fn gather_canonical_root_hashes(subject_did: &str) -> ExternResult<Vec<ActionHash>> {
    let query = LinkQuery::try_new(request_anchor(subject_did), LinkTypes::SubjectToRequest)?;
    let details = get_links_details(query, GetStrategy::Network)?;
    let mut roots = Vec::new();

    for (create_link, deletes) in details.into_inner() {
        if !deletes.is_empty() {
            return Err(guest_error(format!(
                "Historical SubjectToRequest discovery-link deletion observed ({} delete action(s)); request discovery is unresolved",
                deletes.len()
            )));
        }

        let target = match create_link.action() {
            Action::CreateLink(action) => action.target_address.clone(),
            _ => {
                return Err(guest_error(
                    "SubjectToRequest link details contained a non-CreateLink action",
                ));
            }
        };
        let root_hash = target.into_action_hash().ok_or_else(|| {
            guest_error("SubjectToRequest target is not a canonical request ActionHash")
        })?;
        push_unique_hash(&mut roots, root_hash);
    }

    Ok(roots)
}

fn require_record_details(
    action_hash: &ActionHash,
    context: &str,
) -> ExternResult<RecordDetails> {
    let details = get_details(action_hash.clone(), GetOptions::network())?.ok_or_else(|| {
        guest_error(format!("{context}: action details unavailable for {action_hash}"))
    })?;

    let Details::Record(record_details) = details else {
        return Err(guest_error(format!(
            "{context}: expected RecordDetails for ActionHash {action_hash}"
        )));
    };

    if !matches!(record_details.validation_status, ValidationStatus::Valid) {
        return Err(guest_error(format!(
            "{context}: record validation is not Valid for {action_hash}"
        )));
    }

    Ok(record_details)
}

fn gather_root(root_hash: ActionHash) -> ExternResult<GatheredRootV1> {
    let details = require_record_details(&root_hash, "attestation request root")?;
    if !matches!(details.record.action(), Action::Create(_)) {
        return Err(guest_error(format!(
            "Attestation request index target {root_hash} is not a canonical Create action"
        )));
    }

    let root_request = details
        .record
        .entry()
        .to_app_option::<AttestationRequest>()
        .map_err(|error| guest_error(format!("Could not decode request root {root_hash}: {error}")))?
        .ok_or_else(|| guest_error(format!("Request root {root_hash} has no AttestationRequest entry")))?;

    let root_author_did = format!("did:mycelix:{}", details.record.action().author());
    let observed_delete_count = unique_action_hashes(&details.deletes).len();
    let update_hashes = unique_action_hashes(&details.updates);
    let mut updates = Vec::with_capacity(update_hashes.len());

    for update_hash in update_hashes {
        let update_details = require_record_details(&update_hash, "attestation request update")?;
        if !update_details.updates.is_empty() || !update_details.deletes.is_empty() {
            return Err(guest_error(format!(
                "Nested CRUD observed beneath attestation request update {update_hash}; lineage is unresolved"
            )));
        }

        let update_record = get(update_hash.clone(), GetOptions::network())?.ok_or_else(|| {
            guest_error(format!("Valid request update record unavailable for {update_hash}"))
        })?;
        let update_action = match update_record.action() {
            Action::Update(update) => update,
            _ => {
                return Err(guest_error(format!(
                    "Request metadata update {update_hash} does not resolve to an Update action"
                )));
            }
        };
        if &update_action.original_action_address != &root_hash {
            return Err(guest_error(format!(
                "Request update {update_hash} does not directly target canonical root {root_hash}"
            )));
        }

        let request = update_record
            .entry()
            .to_app_option::<AttestationRequest>()
            .map_err(|error| {
                guest_error(format!("Could not decode request update {update_hash}: {error}"))
            })?
            .ok_or_else(|| {
                guest_error(format!("Request update {update_hash} has no AttestationRequest entry"))
            })?;

        updates.push(OwnedUpdateObservationV1 {
            request,
            author_did: format!("did:mycelix:{}", update_record.action().author()),
            action_timestamp_micros: update_record.action().timestamp().as_micros(),
        });
    }

    Ok(GatheredRootV1 {
        root_action_hash: root_hash,
        canonical_root_record: details.record,
        root_request,
        root_author_did,
        updates,
        observed_delete_count,
    })
}

/// Gather network-observed request evidence for one subject and delegate all
/// request semantics to the canonical resolution runtime.
///
/// This function fails closed for malformed, deleted, ambiguous, or conflicted
/// evidence instead of silently dropping roots. Returned states remain explicitly
/// observation-scoped because Holochain network reads are not a global-completeness
/// proof.
pub fn resolve_subject_attestation_requests_v1(
    subject_did: &str,
) -> ExternResult<Vec<ObservedAttestationRequestRecordV1>> {
    validate_subject_query(subject_did)?;
    let root_hashes = gather_canonical_root_hashes(subject_did)?;
    let mut gathered = Vec::with_capacity(root_hashes.len());
    for root_hash in root_hashes {
        gathered.push(gather_root(root_hash)?);
    }

    let root_requests: Vec<&AttestationRequest> =
        gathered.iter().map(|root| &root.root_request).collect();
    validate_unique_request_ids_v1(&root_requests).map_err(|error| {
        guest_error(format!(
            "Attestation request subject aggregation rejected: {error:?}"
        ))
    })?;

    let now_micros = sys_time()?.as_micros();
    let mut resolved = Vec::with_capacity(gathered.len());

    for root in gathered {
        let update_observations: Vec<AttestationRequestUpdateObservationV1<'_>> = root
            .updates
            .iter()
            .map(|update| AttestationRequestUpdateObservationV1 {
                request: &update.request,
                author_did: &update.author_did,
                action_timestamp_micros: update.action_timestamp_micros,
                parent_is_canonical_root: true,
            })
            .collect();

        let state = resolve_attestation_request_lineage_v1(
            AttestationRequestLineageObservationV1 {
                root: &root.root_request,
                root_author_did: &root.root_author_did,
                expected_subject_did: subject_did,
                updates: &update_observations,
                observed_delete_count: root.observed_delete_count,
                now_micros,
            },
        )
        .map_err(|error| {
            guest_error(format!(
                "Attestation request root {} resolution rejected: {error:?}",
                root.root_action_hash
            ))
        })?;

        resolved.push(ObservedAttestationRequestRecordV1 {
            root_action_hash: root.root_action_hash,
            canonical_root_record: root.canonical_root_record,
            state,
        });
    }

    Ok(resolved)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subject_query_shape_is_fail_closed() {
        for invalid in ["", "alice", "did:"] {
            if invalid == "did:" {
                // The compatibility DID theorem currently treats `did:` as a
                // syntactically shaped DID; do not strengthen it only here.
                assert!(validate_subject_query(invalid).is_ok());
            } else {
                assert!(validate_subject_query(invalid).is_err());
            }
        }

        let oversized = format!("did:{}", "x".repeat(253));
        assert_eq!(oversized.len(), 257);
        assert!(validate_subject_query(&oversized).is_err());
        assert!(validate_subject_query("did:mycelix:subject").is_ok());
    }
}

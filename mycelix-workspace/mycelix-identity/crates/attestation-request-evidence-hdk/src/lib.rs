// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Mechanical HDK evidence producer for observed attestation-request lineages.
//!
//! This crate owns network traversal and normalization only. It does not own the
//! request lifecycle, root structural policy, chronology selection, deletion
//! semantics, or duplicate-ID winner rules. Those semantics remain in the
//! Holochain-free policy/resolution layers below it.

#![forbid(unsafe_code)]

use hdk::prelude::*;
use mycelix_attestation_request_resolution_runtime::{
    AttestationRequestLineageObservationV1, AttestationRequestResolutionErrorV1,
    AttestationRequestUpdateObservationV1, ObservedAttestationRequestStateV1,
    resolve_attestation_request_lineage_v1, validate_unique_request_ids_v1,
};
use mycelix_attestation_request_root_policy::DID_MAX_LEN_V1;
use trust_credential_integrity::{AttestationRequest, LinkTypes, UnitEntryTypes};

/// One request root resolved from network-observed evidence.
///
/// `state` remains observation-scoped; it is not a claim of globally complete
/// DHT truth. The canonical root action hash is retained so future mutation APIs
/// can move away from free-form textual IDs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservedAttestationRequestEvidenceV1 {
    pub root_action_hash: ActionHash,
    pub request_id: String,
    pub state: ObservedAttestationRequestStateV1,
}

/// Fail-closed defects in mechanical HDK evidence gathering.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttestationRequestEvidenceErrorV1 {
    SubjectDidInvalid,
    Host(String),
    MalformedSubjectToRequestCreate,
    SubjectDiscoveryBaseMismatch,
    SubjectDiscoveryTargetNotActionHash,
    HistoricalSubjectDiscoveryDeleteObserved {
        count: usize,
    },
    RootDetailsMissing {
        root: ActionHash,
    },
    RootDetailsWrongVariant {
        root: ActionHash,
    },
    RootValidationNotValid {
        root: ActionHash,
        status: ValidationStatus,
    },
    RootActionNotCanonicalCreate {
        root: ActionHash,
    },
    RootEntryTypeMismatch {
        root: ActionHash,
    },
    RootEntryDecodeFailed {
        root: ActionHash,
    },
    UpdateMetadataNotUpdate,
    UpdateParentMismatch {
        update: ActionHash,
        root: ActionHash,
    },
    UpdateDetailsMissing {
        update: ActionHash,
    },
    UpdateDetailsWrongVariant {
        update: ActionHash,
    },
    UpdateValidationNotValid {
        update: ActionHash,
        status: ValidationStatus,
    },
    UpdateRecordNotUpdate {
        update: ActionHash,
    },
    UpdateRecordParentMismatch {
        update: ActionHash,
        root: ActionHash,
    },
    UpdateEntryDecodeFailed {
        update: ActionHash,
    },
    NestedUpdateObserved {
        update: ActionHash,
        count: usize,
    },
    NestedDeleteObserved {
        update: ActionHash,
        count: usize,
    },
    ResolutionRejected {
        error: AttestationRequestResolutionErrorV1,
    },
}

impl From<WasmError> for AttestationRequestEvidenceErrorV1 {
    fn from(error: WasmError) -> Self {
        Self::Host(error.to_string())
    }
}

impl From<AttestationRequestResolutionErrorV1> for AttestationRequestEvidenceErrorV1 {
    fn from(error: AttestationRequestResolutionErrorV1) -> Self {
        Self::ResolutionRejected { error }
    }
}

struct OwnedUpdateObservationV1 {
    request: AttestationRequest,
    author_did: String,
    action_timestamp_micros: i64,
}

struct OwnedLineageV1 {
    root_action_hash: ActionHash,
    root: AttestationRequest,
    root_author_did: String,
    updates: Vec<OwnedUpdateObservationV1>,
    observed_delete_count: usize,
}

fn valid_subject_did_v1(subject_did: &str) -> bool {
    !subject_did.is_empty()
        && subject_did.len() <= DID_MAX_LEN_V1
        && subject_did.starts_with("did:")
}

/// Reproduce the already-DHT-validated `SubjectToRequest` anchor convention.
///
/// Both coordinator creation and integrity link binding use exactly
/// `blake2b_256("requests:{subject_did}")` wrapped as an `EntryHash`.
pub fn subject_request_anchor_v1(subject_did: &str) -> EntryHash {
    let digest = holo_hash::blake2b_256(format!("requests:{subject_did}").as_bytes());
    EntryHash::from_raw_32(digest.to_vec())
}

fn unique_action_hashes(actions: &[SignedActionHashed]) -> Vec<ActionHash> {
    let mut hashes = Vec::with_capacity(actions.len());
    for action in actions {
        let hash = action.hashed.hash.clone();
        if !hashes.contains(&hash) {
            hashes.push(hash);
        }
    }
    hashes
}

fn expected_request_entry_type() -> Result<EntryType, AttestationRequestEvidenceErrorV1> {
    Ok(EntryType::App(AppEntryDef::try_from(
        UnitEntryTypes::AttestationRequest,
    )?))
}

fn require_record_details(
    hash: ActionHash,
    missing: impl FnOnce(ActionHash) -> AttestationRequestEvidenceErrorV1,
    wrong_variant: impl FnOnce(ActionHash) -> AttestationRequestEvidenceErrorV1,
) -> Result<RecordDetails, AttestationRequestEvidenceErrorV1> {
    let details = get_details(hash.clone(), GetOptions::network())?.ok_or_else(|| missing(hash.clone()))?;
    match details {
        Details::Record(details) => Ok(details),
        Details::Entry(_) => Err(wrong_variant(hash)),
    }
}

fn gather_root_lineage_v1(
    root_action_hash: ActionHash,
) -> Result<OwnedLineageV1, AttestationRequestEvidenceErrorV1> {
    let root_details = require_record_details(
        root_action_hash.clone(),
        |root| AttestationRequestEvidenceErrorV1::RootDetailsMissing { root },
        |root| AttestationRequestEvidenceErrorV1::RootDetailsWrongVariant { root },
    )?;

    if root_details.validation_status != ValidationStatus::Valid {
        return Err(AttestationRequestEvidenceErrorV1::RootValidationNotValid {
            root: root_action_hash,
            status: root_details.validation_status,
        });
    }

    let create = match root_details.record.action() {
        Action::Create(create) => create,
        _ => {
            return Err(
                AttestationRequestEvidenceErrorV1::RootActionNotCanonicalCreate {
                    root: root_action_hash,
                },
            );
        }
    };

    if create.entry_type != expected_request_entry_type()? {
        return Err(AttestationRequestEvidenceErrorV1::RootEntryTypeMismatch {
            root: root_action_hash,
        });
    }

    let root = root_details
        .record
        .entry()
        .to_app_option::<AttestationRequest>()
        .map_err(|_| AttestationRequestEvidenceErrorV1::RootEntryDecodeFailed {
            root: root_action_hash.clone(),
        })?
        .ok_or_else(|| AttestationRequestEvidenceErrorV1::RootEntryDecodeFailed {
            root: root_action_hash.clone(),
        })?;

    let root_author_did = format!("did:mycelix:{}", root_details.record.action().author());
    let observed_delete_count = unique_action_hashes(&root_details.deletes).len();

    let mut seen_updates: Vec<ActionHash> = Vec::new();
    let mut updates = Vec::new();

    for update_metadata in &root_details.updates {
        let update_hash = update_metadata.hashed.hash.clone();
        if seen_updates.contains(&update_hash) {
            continue;
        }
        seen_updates.push(update_hash.clone());

        let metadata_update = match &update_metadata.hashed.content {
            Action::Update(update) => update,
            _ => return Err(AttestationRequestEvidenceErrorV1::UpdateMetadataNotUpdate),
        };
        if metadata_update.original_action_address != root_action_hash {
            return Err(AttestationRequestEvidenceErrorV1::UpdateParentMismatch {
                update: update_hash,
                root: root_action_hash,
            });
        }

        let update_details = require_record_details(
            update_hash.clone(),
            |update| AttestationRequestEvidenceErrorV1::UpdateDetailsMissing { update },
            |update| AttestationRequestEvidenceErrorV1::UpdateDetailsWrongVariant { update },
        )?;
        if update_details.validation_status != ValidationStatus::Valid {
            return Err(AttestationRequestEvidenceErrorV1::UpdateValidationNotValid {
                update: update_hash,
                status: update_details.validation_status,
            });
        }

        let update_action = match update_details.record.action() {
            Action::Update(update) => update,
            _ => {
                return Err(AttestationRequestEvidenceErrorV1::UpdateRecordNotUpdate {
                    update: update_hash,
                });
            }
        };
        if update_action.original_action_address != root_action_hash {
            return Err(
                AttestationRequestEvidenceErrorV1::UpdateRecordParentMismatch {
                    update: update_hash,
                    root: root_action_hash,
                },
            );
        }

        let nested_update_count = unique_action_hashes(&update_details.updates).len();
        if nested_update_count != 0 {
            return Err(AttestationRequestEvidenceErrorV1::NestedUpdateObserved {
                update: update_hash,
                count: nested_update_count,
            });
        }
        let nested_delete_count = unique_action_hashes(&update_details.deletes).len();
        if nested_delete_count != 0 {
            return Err(AttestationRequestEvidenceErrorV1::NestedDeleteObserved {
                update: update_hash,
                count: nested_delete_count,
            });
        }

        let request = update_details
            .record
            .entry()
            .to_app_option::<AttestationRequest>()
            .map_err(|_| AttestationRequestEvidenceErrorV1::UpdateEntryDecodeFailed {
                update: update_hash.clone(),
            })?
            .ok_or_else(|| AttestationRequestEvidenceErrorV1::UpdateEntryDecodeFailed {
                update: update_hash.clone(),
            })?;

        updates.push(OwnedUpdateObservationV1 {
            request,
            author_did: format!("did:mycelix:{}", update_details.record.action().author()),
            action_timestamp_micros: update_details.record.action().timestamp().as_micros(),
        });
    }

    Ok(OwnedLineageV1 {
        root_action_hash,
        root,
        root_author_did,
        updates,
        observed_delete_count,
    })
}

/// Gather network-aware request evidence for one subject and resolve it through
/// the observation semantics in `mycelix-attestation-request-resolution-runtime`.
///
/// This function never sorts by action time, chooses a branch winner, treats a
/// delete as cancellation, or suppresses malformed/conflicted roots.
pub fn observe_subject_attestation_requests_v1(
    subject_did: &str,
) -> Result<Vec<ObservedAttestationRequestEvidenceV1>, AttestationRequestEvidenceErrorV1> {
    if !valid_subject_did_v1(subject_did) {
        return Err(AttestationRequestEvidenceErrorV1::SubjectDidInvalid);
    }

    let subject_anchor = subject_request_anchor_v1(subject_did);
    let query = LinkQuery::try_new(subject_anchor.clone(), LinkTypes::SubjectToRequest)?;
    let link_details = get_links_details(query, GetStrategy::Network)?;

    let expected_base: AnyLinkableHash = subject_anchor.into();
    let mut root_hashes: Vec<ActionHash> = Vec::new();

    for (create_link, delete_links) in link_details.into_inner() {
        let create = match &create_link.hashed.content {
            Action::CreateLink(create) => create,
            _ => return Err(AttestationRequestEvidenceErrorV1::MalformedSubjectToRequestCreate),
        };
        if create.base_address != expected_base {
            return Err(AttestationRequestEvidenceErrorV1::SubjectDiscoveryBaseMismatch);
        }

        let delete_count = unique_action_hashes(&delete_links).len();
        if delete_count != 0 {
            return Err(
                AttestationRequestEvidenceErrorV1::HistoricalSubjectDiscoveryDeleteObserved {
                    count: delete_count,
                },
            );
        }

        let root_action_hash = ActionHash::try_from(create.target_address.clone())
            .map_err(|_| AttestationRequestEvidenceErrorV1::SubjectDiscoveryTargetNotActionHash)?;
        if !root_hashes.contains(&root_action_hash) {
            root_hashes.push(root_action_hash);
        }
    }

    let mut lineages = Vec::with_capacity(root_hashes.len());
    for root_action_hash in root_hashes {
        lineages.push(gather_root_lineage_v1(root_action_hash)?);
    }

    let roots: Vec<&AttestationRequest> = lineages.iter().map(|lineage| &lineage.root).collect();
    validate_unique_request_ids_v1(&roots)?;

    let now_micros = sys_time()?.as_micros();
    let mut resolved = Vec::with_capacity(lineages.len());

    for lineage in &lineages {
        let updates: Vec<AttestationRequestUpdateObservationV1<'_>> = lineage
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
                root: &lineage.root,
                root_author_did: &lineage.root_author_did,
                expected_subject_did: subject_did,
                updates: &updates,
                observed_delete_count: lineage.observed_delete_count,
                now_micros,
            },
        )?;

        resolved.push(ObservedAttestationRequestEvidenceV1 {
            root_action_hash: lineage.root_action_hash.clone(),
            request_id: lineage.root.id.clone(),
            state,
        });
    }

    Ok(resolved)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subject_input_uses_shared_did_bound() {
        assert!(valid_subject_did_v1("did:mycelix:alice"));
        assert!(!valid_subject_did_v1(""));
        assert!(!valid_subject_did_v1("alice"));

        let overlong = format!("did:{}", "a".repeat(DID_MAX_LEN_V1));
        assert!(overlong.len() > DID_MAX_LEN_V1);
        assert!(!valid_subject_did_v1(&overlong));
    }

    #[test]
    fn request_anchor_is_deterministic_and_subject_bound() {
        let first = subject_request_anchor_v1("did:mycelix:alice");
        let second = subject_request_anchor_v1("did:mycelix:alice");
        let other = subject_request_anchor_v1("did:mycelix:bob");
        assert_eq!(first, second);
        assert_ne!(first, other);
    }
}

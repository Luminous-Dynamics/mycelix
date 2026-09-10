// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Integrity boundary for DID-authored external evidence-key provenance.
//!
//! V1 intentionally validates only the Mycelix/Holochain side of a two-sided
//! association. The DHT can prove that the committing agent authored a binding
//! for its deterministic `did:mycelix:<agent>` identifier. It does **not** verify
//! the referenced Xenia signature in WASM and therefore never labels a publication
//! as trusted or fully verified.

#![forbid(unsafe_code)]

use hdi::prelude::*;
use mycelix_identity_provenance::KeyDidBindingArtifact;

/// Maximum UTF-8 byte length for an opaque external Xenia attestation locator.
pub const MAX_XENIA_ATTESTATION_REF_LEN: usize = 1_024;

/// DID-authored publication of one canonical external-key association artifact.
///
/// `xenia_attestation_ref` is an opaque locator/reference only. A consumer must
/// retrieve the referenced attestation, verify it with Xenia, and apply the
/// `mycelix-identity-provenance` metadata guard before calling the association
/// two-sided. DHT validity alone proves only DID-side authorship plus structure.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct KeyDidBindingPublication {
    /// Canonical non-authoritative DID↔key association artifact.
    pub artifact: KeyDidBindingArtifact,
    /// Opaque reference to the external Xenia attestation over `artifact.canonical_bytes()`.
    pub xenia_attestation_ref: String,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// DID-side publication of an external evidence-key binding.
    KeyDidBindingPublication(KeyDidBindingPublication),
}

/// Reserved index type for a later coordinator tranche.
///
/// V1 rejects all link creation so no unqualified index can accidentally become
/// an authority/query surface before its canonical base/target validation exists.
#[hdk_link_types]
pub enum LinkTypes {
    /// Future DID/fingerprint lookup index. Rejected in v1.
    BindingIndex,
}

/// Genesis self-check.
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main integrity callback.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::KeyDidBindingPublication(publication) => {
                    validate_create_publication(EntryCreationAction::Create(action), publication)
                }
            },
            OpEntry::UpdateEntry { .. } => invalid("Key provenance publications are immutable; publish a new lifecycle record in a future protocol version"),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => invalid("Key provenance publications cannot be updated"),
        FlatOp::RegisterDelete(_) => invalid("Key provenance publications cannot be deleted"),
        FlatOp::RegisterCreateLink { .. } => invalid(
            "Key provenance indexes are disabled in v1 until canonical defensive indexing is defined",
        ),
        FlatOp::RegisterDeleteLink { .. } => invalid("Key provenance links cannot be deleted in v1"),
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_create_publication(
    action: EntryCreationAction,
    publication: KeyDidBindingPublication,
) -> ExternResult<ValidateCallbackResult> {
    let author_did = format!("did:mycelix:{}", action.author());
    Ok(match validate_publication_fields(&publication, &author_did) {
        Ok(()) => ValidateCallbackResult::Valid,
        Err(reason) => ValidateCallbackResult::Invalid(reason),
    })
}

/// Pure structural validation used by the DHT callback and unit tests.
fn validate_publication_fields(
    publication: &KeyDidBindingPublication,
    author_did: &str,
) -> Result<(), String> {
    publication
        .artifact
        .validate()
        .map_err(|error| format!("invalid key-DID binding artifact: {error}"))?;

    if publication.artifact.did != author_did {
        return Err(format!(
            "key-DID binding must be authored by its DID controller: expected '{author_did}', got '{}'",
            publication.artifact.did
        ));
    }

    let attestation_ref = publication.xenia_attestation_ref.trim();
    if attestation_ref.is_empty() {
        return Err("xenia_attestation_ref must not be empty".into());
    }
    if publication.xenia_attestation_ref.len() > MAX_XENIA_ATTESTATION_REF_LEN {
        return Err(format!(
            "xenia_attestation_ref exceeds maximum length {}",
            MAX_XENIA_ATTESTATION_REF_LEN
        ));
    }
    if attestation_ref != publication.xenia_attestation_ref {
        return Err("xenia_attestation_ref must not contain leading or trailing whitespace".into());
    }

    Ok(())
}

fn invalid(message: impl Into<String>) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(message.into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_identity_provenance::KeyAssociationPurpose;

    fn artifact(did: &str) -> KeyDidBindingArtifact {
        KeyDidBindingArtifact::new(
            did,
            [7u8; 32],
            "ed25519-rfc8032",
            KeyAssociationPurpose::EvidenceAttestor,
            "symthaea-generativity-provenance",
        )
        .unwrap()
    }

    #[test]
    fn matching_did_author_is_accepted() {
        let did = "did:mycelix:uhCAk-author";
        let publication = KeyDidBindingPublication {
            artifact: artifact(did),
            xenia_attestation_ref: "xenia-attestation:blake3:abc123".into(),
        };
        assert_eq!(validate_publication_fields(&publication, did), Ok(()));
    }

    #[test]
    fn did_takeover_is_rejected() {
        let publication = KeyDidBindingPublication {
            artifact: artifact("did:mycelix:uhCAk-victim"),
            xenia_attestation_ref: "xenia-attestation:blake3:abc123".into(),
        };
        let error = validate_publication_fields(&publication, "did:mycelix:uhCAk-attacker")
            .unwrap_err();
        assert!(error.contains("authored by its DID controller"));
    }

    #[test]
    fn empty_attestation_reference_is_rejected() {
        let did = "did:mycelix:uhCAk-author";
        let publication = KeyDidBindingPublication {
            artifact: artifact(did),
            xenia_attestation_ref: "".into(),
        };
        assert!(validate_publication_fields(&publication, did).is_err());
    }

    #[test]
    fn padded_attestation_reference_is_rejected() {
        let did = "did:mycelix:uhCAk-author";
        let publication = KeyDidBindingPublication {
            artifact: artifact(did),
            xenia_attestation_ref: " xenia-attestation:blake3:abc123 ".into(),
        };
        assert!(validate_publication_fields(&publication, did).is_err());
    }

    #[test]
    fn invalid_nested_artifact_is_rejected() {
        let did = "did:mycelix:uhCAk-author";
        let mut nested = artifact(did);
        nested.scope = "*".into();
        let publication = KeyDidBindingPublication {
            artifact: nested,
            xenia_attestation_ref: "xenia-attestation:blake3:abc123".into(),
        };
        assert!(validate_publication_fields(&publication, did).is_err());
    }
}

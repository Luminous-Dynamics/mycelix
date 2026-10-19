// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Integrity boundary for DID-authored external evidence-key provenance.
//!
//! The DHT proves the Mycelix side of an association: the committing agent authored
//! a binding for its deterministic `did:mycelix:<agent>` identifier. Xenia signature
//! verification remains external to WASM. Lifecycle is append-only: bindings are
//! never rewritten or erased; controllers may publish retraction/supersession records.

#![forbid(unsafe_code)]

use hdi::prelude::*;
use mycelix_identity_provenance::KeyDidBindingArtifact;

/// Maximum UTF-8 byte length for an opaque external Xenia attestation locator.
pub const MAX_XENIA_ATTESTATION_REF_LEN: usize = 1_024;
/// Maximum UTF-8 byte length for an optional lifecycle explanation.
pub const MAX_LIFECYCLE_REASON_LEN: usize = 512;

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

/// Append-only lifecycle operation for a previously valid key-binding publication.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BindingLifecycleDisposition {
    /// The DID controller withdraws the association for future consumers.
    Retract,
    /// The DID controller points to a replacement publication for the same purpose/scope.
    Supersede,
}

/// Immutable lifecycle evidence for a key-binding publication.
///
/// Multiple valid lifecycle records may coexist because concurrent authorship cannot be
/// globally serialized at integrity-validation time. Consumers must surface conflicting
/// records rather than silently choosing one.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct KeyBindingLifecycle {
    /// Exact action hash of the publication being retracted or superseded.
    pub target_publication: ActionHash,
    /// Lifecycle operation.
    pub disposition: BindingLifecycleDisposition,
    /// Replacement publication for [`BindingLifecycleDisposition::Supersede`].
    pub replacement_publication: Option<ActionHash>,
    /// Optional human-readable explanation; evidence only, never authority.
    pub reason: Option<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// DID-side publication of an external evidence-key binding.
    KeyDidBindingPublication(KeyDidBindingPublication),
    /// Append-only lifecycle evidence for a binding publication.
    KeyBindingLifecycle(KeyBindingLifecycle),
}

/// Reserved index type for a later coordinator tranche.
///
/// Links remain rejected until their canonical bases/targets are separately reviewed.
#[hdk_link_types]
pub enum LinkTypes {
    /// Future DID/fingerprint/lifecycle lookup index. Rejected in this tranche.
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
                EntryTypes::KeyBindingLifecycle(lifecycle) => {
                    validate_create_lifecycle(EntryCreationAction::Create(action), lifecycle)
                }
            },
            OpEntry::UpdateEntry { .. } => invalid(
                "Key provenance entries are append-only and cannot be updated",
            ),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => invalid("Key provenance entries cannot be updated"),
        FlatOp::RegisterDelete(_) => invalid("Key provenance entries cannot be deleted"),
        FlatOp::RegisterCreateLink { .. } => invalid(
            "Key provenance indexes are disabled until canonical defensive indexing is defined",
        ),
        FlatOp::RegisterDeleteLink { .. } => invalid("Key provenance links cannot be deleted"),
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

fn validate_create_lifecycle(
    action: EntryCreationAction,
    lifecycle: KeyBindingLifecycle,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_lifecycle_shape(&lifecycle) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    let target_record = must_get_valid_record(lifecycle.target_publication.clone())?;
    let target_publication = match decode_publication(&target_record) {
        Ok(publication) => publication,
        Err(reason) => return Ok(ValidateCallbackResult::Invalid(reason)),
    };

    let author = action.author();
    if target_record.action().author() != author {
        return invalid("Only the author of a key-binding publication may retract or supersede it");
    }

    let author_did = format!("did:mycelix:{author}");
    if target_publication.artifact.did != author_did {
        return invalid("Target publication DID does not match lifecycle author");
    }

    if lifecycle.disposition == BindingLifecycleDisposition::Supersede {
        let replacement_hash = lifecycle
            .replacement_publication
            .as_ref()
            .expect("shape validation requires replacement for Supersede");
        let replacement_record = must_get_valid_record(replacement_hash.clone())?;
        let replacement = match decode_publication(&replacement_record) {
            Ok(publication) => publication,
            Err(reason) => return Ok(ValidateCallbackResult::Invalid(reason)),
        };

        if replacement_record.action().author() != author {
            return invalid("Replacement publication must have the same DID-controller author");
        }
        if replacement.artifact.did != target_publication.artifact.did {
            return invalid("Replacement publication must bind the same DID");
        }
        if replacement.artifact.purpose != target_publication.artifact.purpose {
            return invalid("Replacement publication must preserve association purpose");
        }
        if replacement.artifact.scope != target_publication.artifact.scope {
            return invalid("Replacement publication must preserve association scope");
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn decode_publication(record: &Record) -> Result<KeyDidBindingPublication, String> {
    match record.entry().to_app_option::<KeyDidBindingPublication>() {
        Ok(Some(publication)) => Ok(publication),
        Ok(None) => Err("Lifecycle target must contain a public key-binding publication".into()),
        Err(_) => Err("Lifecycle target must decode as a key-binding publication".into()),
    }
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

fn validate_lifecycle_shape(lifecycle: &KeyBindingLifecycle) -> Result<(), String> {
    if let Some(reason) = &lifecycle.reason {
        if reason.len() > MAX_LIFECYCLE_REASON_LEN {
            return Err(format!(
                "lifecycle reason exceeds maximum length {}",
                MAX_LIFECYCLE_REASON_LEN
            ));
        }
        if reason.trim() != reason {
            return Err("lifecycle reason must not contain leading or trailing whitespace".into());
        }
    }

    match (&lifecycle.disposition, &lifecycle.replacement_publication) {
        (BindingLifecycleDisposition::Retract, None) => Ok(()),
        (BindingLifecycleDisposition::Retract, Some(_)) => {
            Err("Retract lifecycle record must not specify a replacement".into())
        }
        (BindingLifecycleDisposition::Supersede, None) => {
            Err("Supersede lifecycle record requires a replacement publication".into())
        }
        (BindingLifecycleDisposition::Supersede, Some(replacement)) => {
            if replacement == &lifecycle.target_publication {
                Err("A key-binding publication cannot supersede itself".into())
            } else {
                Ok(())
            }
        }
    }
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

    #[test]
    fn retraction_cannot_name_replacement() {
        let target = ActionHash::from_raw_36(vec![1; 36]);
        let replacement = ActionHash::from_raw_36(vec![2; 36]);
        let lifecycle = KeyBindingLifecycle {
            target_publication: target,
            disposition: BindingLifecycleDisposition::Retract,
            replacement_publication: Some(replacement),
            reason: None,
        };
        assert!(validate_lifecycle_shape(&lifecycle).is_err());
    }

    #[test]
    fn supersession_requires_distinct_replacement() {
        let target = ActionHash::from_raw_36(vec![1; 36]);
        let missing = KeyBindingLifecycle {
            target_publication: target.clone(),
            disposition: BindingLifecycleDisposition::Supersede,
            replacement_publication: None,
            reason: None,
        };
        assert!(validate_lifecycle_shape(&missing).is_err());

        let self_replacement = KeyBindingLifecycle {
            target_publication: target.clone(),
            disposition: BindingLifecycleDisposition::Supersede,
            replacement_publication: Some(target),
            reason: None,
        };
        assert!(validate_lifecycle_shape(&self_replacement).is_err());
    }

    #[test]
    fn bounded_canonical_reason_is_accepted() {
        let lifecycle = KeyBindingLifecycle {
            target_publication: ActionHash::from_raw_36(vec![1; 36]),
            disposition: BindingLifecycleDisposition::Retract,
            replacement_publication: None,
            reason: Some("key retired".into()),
        };
        assert_eq!(validate_lifecycle_shape(&lifecycle), Ok(()));
    }
}

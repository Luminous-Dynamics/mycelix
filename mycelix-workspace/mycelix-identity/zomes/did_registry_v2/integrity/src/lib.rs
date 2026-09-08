// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical-only DID:mycelix V2 integrity domain.
//!
//! This zome intentionally does not reinterpret legacy DID entries. It defines the
//! admission rules for a new DNA hash/authority epoch whose DID documents must satisfy
//! the strict Holochain-free cryptographic theorem before DHT admission.

#![forbid(unsafe_code)]

use hdi::prelude::*;
use mycelix_did_document_crypto_policy::{
    validate_did_document_crypto_v2, DidDocumentCryptoAdmissionViewV2,
    DidVerificationMethodAdmissionViewV2,
};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DidDocumentV2 {
    pub id: String,
    pub controller: AgentPubKey,
    #[serde(rename = "verificationMethod", alias = "verification_method")]
    pub verification_method: Vec<VerificationMethodV2>,
    pub authentication: Vec<String>,
    #[serde(
        rename = "keyAgreement",
        alias = "key_agreement",
        default,
        skip_serializing_if = "Vec::is_empty"
    )]
    pub key_agreement: Vec<String>,
    pub service: Vec<ServiceEndpointV2>,
    pub created: Timestamp,
    pub updated: Timestamp,
    pub version: u32,
}

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct VerificationMethodV2 {
    pub id: String,
    #[serde(rename = "type", alias = "type_")]
    pub type_: String,
    pub controller: String,
    #[serde(rename = "publicKeyMultibase", alias = "public_key_multibase")]
    pub public_key_multibase: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub algorithm: Option<u16>,
}

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct ServiceEndpointV2 {
    pub id: String,
    #[serde(rename = "type", alias = "type_")]
    pub type_: String,
    #[serde(rename = "serviceEndpoint", alias = "service_endpoint")]
    pub service_endpoint: String,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DidDeactivationV2 {
    pub did: String,
    /// Exact earlier V2 DID-document action being deactivated.
    pub did_document_action: ActionHash,
    pub reason: String,
    /// Signed Holochain action/entry time provenance, not trusted wall-clock authority.
    pub deactivated_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    DidDocumentV2(DidDocumentV2),
    DidDeactivationV2(DidDeactivationV2),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Optional discovery-only index. Never currentness/lineage authority.
    DidV2Discovery,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

fn expected_author_did(author: &AgentPubKey) -> String {
    format!("did:mycelix:{author}")
}

fn validate_crypto_fields_v2(document: &DidDocumentV2) -> Result<(), String> {
    let methods: Vec<DidVerificationMethodAdmissionViewV2<'_>> = document
        .verification_method
        .iter()
        .map(|method| DidVerificationMethodAdmissionViewV2 {
            id: &method.id,
            type_: &method.type_,
            controller: &method.controller,
            public_key_multibase: &method.public_key_multibase,
            algorithm: method.algorithm,
        })
        .collect();
    let authentication: Vec<&str> = document.authentication.iter().map(String::as_str).collect();
    let key_agreement: Vec<&str> = document.key_agreement.iter().map(String::as_str).collect();

    validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
        did: &document.id,
        verification_methods: &methods,
        authentication: &authentication,
        key_agreement: &key_agreement,
    })
    .map(|_| ())
    .map_err(|error| format!("DID V2 cryptographic admission failed: {error:?}"))
}

fn validate_create_did_document_v2(
    action: EntryCreationAction,
    document: DidDocumentV2,
) -> ExternResult<ValidateCallbackResult> {
    let author = action.author();
    if document.controller != *author {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 controller must equal the committing agent".into(),
        ));
    }
    if document.id != expected_author_did(author) {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 identifier must equal did:mycelix:<committing-agent>".into(),
        ));
    }
    if document.version != 1 {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 creation version must equal 1".into(),
        ));
    }
    if document.created != document.updated {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 creation must start with created == updated".into(),
        ));
    }
    if let Err(error) = validate_crypto_fields_v2(&document) {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_did_document_v2(
    action: Update,
    document: DidDocumentV2,
) -> ExternResult<ValidateCallbackResult> {
    if document.controller != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the DID V2 controller may update the document".into(),
        ));
    }
    if document.id != expected_author_did(&action.author) {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 identifier must remain bound to the update author".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    if original_record.action().entry_type() != Some(&action.entry_type) {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 updates must preserve the exact application entry type".into(),
        ));
    }
    let original: DidDocumentV2 = original_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "DID V2 update parent is not a DID document".into()
        )))?;

    if document.id != original.id
        || document.controller != original.controller
        || document.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 id/controller/created fields are immutable".into(),
        ));
    }
    let expected_version = original.version.checked_add(1).ok_or(wasm_error!(
        WasmErrorInner::Guest("DID V2 version overflow".into())
    ))?;
    if document.version != expected_version {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 update version must equal direct-parent version + 1".into(),
        ));
    }
    if document.updated <= original.updated {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 author-provenance updated timestamp must advance".into(),
        ));
    }
    if let Err(error) = validate_crypto_fields_v2(&document) {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_deactivation_v2(
    action: EntryCreationAction,
    deactivation: DidDeactivationV2,
) -> ExternResult<ValidateCallbackResult> {
    let author = action.author();
    if deactivation.did != expected_author_did(author) {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 deactivation must be authored by the named DID".into(),
        ));
    }
    if deactivation.reason.is_empty() || deactivation.reason.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 deactivation reason must be 1-4096 bytes".into(),
        ));
    }

    let document_record = must_get_valid_record(deactivation.did_document_action.clone())?;
    if document_record.action().author() != author {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 deactivation must reference same-author DID material".into(),
        ));
    }
    if document_record.action().action_seq() >= action.action_seq() {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 deactivation must reference an earlier DID-document action".into(),
        ));
    }
    let document: DidDocumentV2 = document_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "DID V2 deactivation reference is not a DID document".into()
        )))?;
    if document.id != deactivation.did {
        return Ok(ValidateCallbackResult::Invalid(
            "DID V2 deactivation reference names a different DID".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::DidDocumentV2(document) => validate_create_did_document_v2(
                    EntryCreationAction::Create(action),
                    document,
                ),
                EntryTypes::DidDeactivationV2(deactivation) => validate_create_deactivation_v2(
                    EntryCreationAction::Create(action),
                    deactivation,
                ),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::DidDocumentV2(document) => {
                    validate_update_did_document_v2(action, document)
                }
                EntryTypes::DidDeactivationV2(_) => Ok(ValidateCallbackResult::Invalid(
                    "DID V2 deactivation records are append-only".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { tag, .. } => {
            if tag.0.len() > 1024 {
                Ok(ValidateCallbackResult::Invalid(
                    "DID V2 discovery-link tag exceeds 1024 bytes".into(),
                ))
            } else {
                Ok(ValidateCallbackResult::Valid)
            }
        }
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            ..
        } => {
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only a DID V2 discovery-link author may delete that link".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            if original.action().author() != &action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original DID V2 entry author may update it".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "DID V2 authority entries cannot be deleted".into(),
        )),
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::{AlgorithmId, TaggedPublicKey};

    fn test_agent() -> AgentPubKey {
        AgentPubKey::try_from(
            "uhCAk29vb29vb29vb29vb29vb29vb29vb29vb29vb29vb29uTp5Iv",
        )
        .unwrap()
    }

    fn document_with_key(multibase: String) -> DidDocumentV2 {
        DidDocumentV2 {
            id: "did:mycelix:test-agent".into(),
            controller: test_agent(),
            verification_method: vec![VerificationMethodV2 {
                id: "did:mycelix:test-agent#keys-1".into(),
                type_: AlgorithmId::Ed25519.did_verification_method_type().into(),
                controller: "did:mycelix:test-agent".into(),
                public_key_multibase: multibase,
                algorithm: Some(AlgorithmId::Ed25519.as_u16()),
            }],
            authentication: vec!["did:mycelix:test-agent#keys-1".into()],
            key_agreement: vec![],
            service: vec![],
            created: Timestamp::from_micros(1_000_000),
            updated: Timestamp::from_micros(1_000_000),
            version: 1,
        }
    }

    #[test]
    fn canonical_ed25519_document_passes_crypto_adapter() {
        let canonical = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x22; 32])
            .unwrap()
            .to_multibase();
        assert!(validate_crypto_fields_v2(&document_with_key(canonical)).is_ok());
    }

    #[test]
    fn legacy_raw_ed25519_is_rejected_by_v2_integrity_adapter() {
        let raw = bs58::encode(vec![0x22; 32]).into_string();
        let legacy = format!("z{raw}");
        let error = validate_crypto_fields_v2(&document_with_key(legacy)).unwrap_err();
        assert!(error.contains("PublicKeyMultibaseNotCanonical"));
    }
}

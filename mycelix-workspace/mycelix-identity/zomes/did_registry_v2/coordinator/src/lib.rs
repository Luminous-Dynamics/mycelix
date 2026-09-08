// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact-parent canonical DID:mycelix V2 coordinator.
//!
//! This coordinator is intentionally separate from the legacy DID coordinator. It never
//! resolves authority through latest-link selection and never emits the legacy
//! `z{AgentPubKey}` key encoding. Updates and deactivations name one exact prior action.

#![forbid(unsafe_code)]

use did_registry_v2_integrity::{
    DidDeactivationV2, DidDocumentV2, EntryTypes, ServiceEndpointV2, VerificationMethodV2,
};
use hdk::prelude::*;
use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use serde::{Deserialize, Serialize};

fn canonical_ed25519_multibase_v2(raw_key: &[u8]) -> Result<String, String> {
    TaggedPublicKey::new(AlgorithmId::Ed25519, raw_key.to_vec())
        .map(|key| key.to_multibase())
        .map_err(|error| format!("canonical Ed25519 key construction failed: {error}"))
}

fn exact_did_for_agent(agent: &AgentPubKey) -> String {
    format!("did:mycelix:{agent}")
}

fn require_owned_document(document: &DidDocumentV2, agent: &AgentPubKey) -> ExternResult<()> {
    let expected_did = exact_did_for_agent(agent);
    if document.controller != *agent || document.id != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "exact DID V2 action is not owned by the calling agent".into()
        )));
    }
    Ok(())
}

#[hdk_extern]
pub fn create_did_v2(_: ()) -> ExternResult<Record> {
    let agent = agent_info()?.agent_initial_pubkey;
    let did = exact_did_for_agent(&agent);
    let multibase = canonical_ed25519_multibase_v2(agent.get_raw_32()).map_err(|message| {
        wasm_error!(WasmErrorInner::Guest(message))
    })?;
    let key_id = format!("{did}#keys-1");
    let now = sys_time()?;

    let document = DidDocumentV2 {
        id: did.clone(),
        controller: agent,
        verification_method: vec![VerificationMethodV2 {
            id: key_id.clone(),
            type_: AlgorithmId::Ed25519
                .did_verification_method_type()
                .to_string(),
            controller: did,
            public_key_multibase: multibase,
            algorithm: Some(AlgorithmId::Ed25519.as_u16()),
        }],
        authentication: vec![key_id],
        key_agreement: vec![],
        service: vec![],
        created: now,
        updated: now,
        version: 1,
    };

    let action_hash = create_entry(&EntryTypes::DidDocumentV2(document))?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "created DID V2 record was not retrievable by exact action hash".into()
    )))
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpdateDidV2Input {
    /// Exact DID-document action to update. There is no latest/current lookup here.
    pub previous_action: ActionHash,
    pub verification_method: Vec<VerificationMethodV2>,
    pub authentication: Vec<String>,
    pub key_agreement: Vec<String>,
    pub service: Vec<ServiceEndpointV2>,
}

#[hdk_extern]
pub fn update_did_v2(input: UpdateDidV2Input) -> ExternResult<Record> {
    let agent = agent_info()?.agent_initial_pubkey;
    let previous_record = get(input.previous_action.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "exact DID V2 parent action was not found".into()
        )))?;
    if previous_record.action_address() != &input.previous_action {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "retrieved DID V2 parent action did not match the requested action hash".into()
        )));
    }
    let previous: DidDocumentV2 = previous_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "exact parent action is not a DID V2 document".into()
        )))?;
    require_owned_document(&previous, &agent)?;

    let version = previous.version.checked_add(1).ok_or(wasm_error!(
        WasmErrorInner::Guest("DID V2 version overflow".into())
    ))?;
    let updated = DidDocumentV2 {
        id: previous.id,
        controller: previous.controller,
        verification_method: input.verification_method,
        authentication: input.authentication,
        key_agreement: input.key_agreement,
        service: input.service,
        created: previous.created,
        updated: sys_time()?,
        version,
    };

    let action_hash = update_entry(
        input.previous_action,
        &EntryTypes::DidDocumentV2(updated),
    )?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "updated DID V2 record was not retrievable by exact action hash".into()
    )))
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeactivateDidV2Input {
    /// Exact DID-document action this deactivation refers to.
    pub did_document_action: ActionHash,
    pub reason: String,
}

#[hdk_extern]
pub fn deactivate_did_v2(input: DeactivateDidV2Input) -> ExternResult<Record> {
    if input.reason.is_empty() || input.reason.len() > 4096 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "DID V2 deactivation reason must be 1-4096 bytes".into()
        )));
    }

    let agent = agent_info()?.agent_initial_pubkey;
    let document_record = get(input.did_document_action.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "exact DID V2 action to deactivate was not found".into()
        )))?;
    if document_record.action_address() != &input.did_document_action {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "retrieved DID V2 action did not match the requested action hash".into()
        )));
    }
    let document: DidDocumentV2 = document_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "exact deactivation target is not a DID V2 document".into()
        )))?;
    require_owned_document(&document, &agent)?;

    let deactivation = DidDeactivationV2 {
        did: document.id,
        did_document_action: input.did_document_action,
        reason: input.reason,
        deactivated_at: sys_time()?,
    };
    let action_hash = create_entry(&EntryTypes::DidDeactivationV2(deactivation))?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "created DID V2 deactivation was not retrievable by exact action hash".into()
    )))
}

/// Exact-action retrieval only. This is intentionally not a current-DID resolver.
#[hdk_extern]
pub fn get_did_v2_action(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_default_key_round_trips_with_multicodec_prefix() {
        let raw = [0x42; 32];
        let multibase = canonical_ed25519_multibase_v2(&raw).unwrap();
        let decoded = TaggedPublicKey::from_multibase(&multibase).unwrap();
        assert_eq!(decoded.algorithm, AlgorithmId::Ed25519);
        assert_eq!(decoded.key_bytes, raw);
        assert_eq!(decoded.to_multibase(), multibase);
    }

    #[test]
    fn default_key_builder_rejects_wrong_raw_length() {
        assert!(canonical_ed25519_multibase_v2(&[0x42; 31]).is_err());
    }
}

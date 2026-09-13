// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Purpose-separated continuity witness integrity zome.

use hdi::prelude::*;

pub mod types;
pub mod validation;

pub use types::*;
pub use validation::{
    check_attestation_shape, check_checkpoint_shape, check_checkpoint_successor,
    digest_checkpoint_bytes,
};

fn invalid(reason: impl Into<String>) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(reason.into()))
}

fn validate_checkpoint_create(
    action: Create,
    checkpoint: ContinuityCheckpoint,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = check_checkpoint_shape(&checkpoint) {
        return invalid(reason);
    }
    if checkpoint.observed_at > action.timestamp {
        return invalid("Checkpoint observed_at cannot be later than its action timestamp");
    }
    if checkpoint.revision > 1 {
        let previous_action = checkpoint
            .previous_checkpoint
            .clone()
            .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Missing predecessor".into())))?;
        let previous_record = must_get_valid_record(previous_action.clone())?;
        let previous: ContinuityCheckpoint = previous_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Predecessor is not a continuity checkpoint".into()
                ))
            })?;
        if let Err(reason) = check_checkpoint_successor(&previous_action, &previous, &checkpoint) {
            return invalid(reason);
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_attestation_create(
    action: Create,
    attestation: WitnessAttestation,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = check_attestation_shape(&attestation) {
        return invalid(reason);
    }
    if attestation.witnessed_at > action.timestamp {
        return invalid("Witnessed_at cannot be later than its action timestamp");
    }
    let checkpoint_record = must_get_valid_record(attestation.checkpoint_action.clone())?;
    let checkpoint: ContinuityCheckpoint = checkpoint_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Attestation target is not a continuity checkpoint".into()
            ))
        })?;
    if checkpoint_record.action().author() == &action.author {
        return invalid("Checkpoint submitter cannot count as an independent witness");
    }
    if checkpoint.object_fingerprint != attestation.object_fingerprint
        || checkpoint.revision != attestation.revision
        || checkpoint.checkpoint_digest != attestation.checkpoint_digest
    {
        return invalid("Attestation metadata does not match exact checkpoint");
    }
    if attestation.witnessed_at < checkpoint.observed_at {
        return invalid("Witness attestation predates retained checkpoint");
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ContinuityCheckpoint(value) => {
                    validate_checkpoint_create(action, value)
                }
                EntryTypes::WitnessAttestation(value) => {
                    validate_attestation_create(action, value)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => invalid("Continuity witness anchors are append-only"),
                EntryTypes::ContinuityCheckpoint(_) => {
                    invalid("Continuity checkpoints cannot be updated")
                }
                EntryTypes::WitnessAttestation(_) => {
                    invalid("Witness attestations cannot be updated")
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => {
            invalid("Continuity witness index links cannot be deleted")
        }
        FlatOp::RegisterDelete(_) => invalid("Continuity witness entries cannot be deleted"),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}

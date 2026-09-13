// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Purpose-separated continuity witness integrity zome.
//!
//! This zome retains opaque external continuity checkpoints (initially Xenia
//! `StateAnchorCheckpoint` artifacts) and Holochain-agent witness attestations.
//! It deliberately does **not** parse or cryptographically verify the external
//! checkpoint. The relying system must do that with the source protocol's verifier.
//!
//! Mycelix contributes an independent, append-only DHT retention plane:
//! - exact opaque bytes are digest-bound;
//! - revisions form a strict predecessor chain;
//! - checkpoints and attestations cannot be updated/deleted;
//! - witness attestations are Holochain-author-bound and cannot be authored by the
//!   checkpoint submitter;
//! - same-revision forks remain possible under distributed concurrency and MUST be
//!   detected fail-closed by clients rather than resolved by arbitrary selection.

use hdi::prelude::*;
use sha2::{Digest, Sha256};

/// Mycelix entry schema for retained external continuity checkpoints.
pub const CONTINUITY_CHECKPOINT_SCHEMA: &str = "mycelix.continuity-witness.checkpoint.v1";
/// Mycelix entry schema for independent witness attestations.
pub const CONTINUITY_ATTESTATION_SCHEMA: &str = "mycelix.continuity-witness.attestation.v1";
/// Domain separator used to digest opaque checkpoint bytes.
pub const CHECKPOINT_DIGEST_DOMAIN: &[u8] = b"mycelix.continuity-witness.checkpoint-digest.v1\0";

const MAX_NAMESPACE_BYTES: usize = 256;
const MAX_SOURCE_PROTOCOL_BYTES: usize = 128;
const MAX_CHECKPOINT_BYTES: usize = 64 * 1024;
const DIGEST_BYTES: usize = 32;

/// Deterministic anchor used only as an index base.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// One exact externally produced continuity checkpoint retained on the Mycelix DHT.
///
/// `object_fingerprint` and `anchor_fingerprint` are untrusted indexing metadata until
/// the relying system verifies `checkpoint_bytes` with the external protocol. Integrity
/// validation guarantees only that the bytes themselves are immutable and digest-bound.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ContinuityCheckpoint {
    /// Must equal [`CONTINUITY_CHECKPOINT_SCHEMA`].
    pub schema_version: String,
    /// Purpose/domain namespace, e.g. the Symthaea/Xenia continuity profile.
    pub namespace: String,
    /// External checkpoint protocol label, e.g. `xenia-state-anchor-checkpoint-v1`.
    pub source_protocol: String,
    /// Privacy-reduced external object/target fingerprint (32 bytes).
    pub object_fingerprint: Vec<u8>,
    /// Strict external continuity revision; genesis is 1.
    pub revision: u64,
    /// Fingerprint of the exact signed external anchor retained by the checkpoint (32 bytes).
    pub anchor_fingerprint: Vec<u8>,
    /// Domain-separated SHA-256 of `checkpoint_bytes` (32 bytes).
    pub checkpoint_digest: Vec<u8>,
    /// Exact opaque external checkpoint bytes.
    pub checkpoint_bytes: Vec<u8>,
    /// Exact preceding Mycelix checkpoint action. `None` only at revision 1.
    pub previous_checkpoint: Option<ActionHash>,
    /// Must equal the Holochain create-action timestamp.
    pub observed_at: Timestamp,
}

/// Independent Holochain-agent attestation to one exact retained checkpoint.
///
/// The witness identity is the author of the Holochain create action, not a freeform
/// field supplied by the entry author.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WitnessAttestation {
    /// Must equal [`CONTINUITY_ATTESTATION_SCHEMA`].
    pub schema_version: String,
    /// Exact checkpoint action being witnessed.
    pub checkpoint_action: ActionHash,
    /// Copied checkpoint object fingerprint (32 bytes).
    pub object_fingerprint: Vec<u8>,
    /// Copied strict continuity revision.
    pub revision: u64,
    /// Copied exact checkpoint digest (32 bytes).
    pub checkpoint_digest: Vec<u8>,
    /// Must equal the Holochain create-action timestamp.
    pub witnessed_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ContinuityCheckpoint(ContinuityCheckpoint),
    WitnessAttestation(WitnessAttestation),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Object anchor to every retained checkpoint for that object.
    ObjectToCheckpoint,
    /// Object+revision anchor to all candidates at that revision (fork detection).
    RevisionToCheckpoint,
    /// Checkpoint action to independent witness attestations.
    CheckpointToAttestation,
    /// Previous checkpoint action to candidate direct successors.
    CheckpointToSuccessor,
}

/// Compute the exact digest committed by [`ContinuityCheckpoint::checkpoint_digest`].
pub fn digest_checkpoint_bytes(bytes: &[u8]) -> Vec<u8> {
    let mut hasher = Sha256::new();
    hasher.update(CHECKPOINT_DIGEST_DOMAIN);
    hasher.update((bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
    hasher.finalize().to_vec()
}

/// Pure shape/integrity checks that do not require host calls.
pub fn check_checkpoint_shape(checkpoint: &ContinuityCheckpoint) -> Result<(), String> {
    if checkpoint.schema_version != CONTINUITY_CHECKPOINT_SCHEMA {
        return Err(format!(
            "Unsupported continuity checkpoint schema: {:?}",
            checkpoint.schema_version
        ));
    }
    validate_text("namespace", &checkpoint.namespace, MAX_NAMESPACE_BYTES)?;
    validate_text(
        "source_protocol",
        &checkpoint.source_protocol,
        MAX_SOURCE_PROTOCOL_BYTES,
    )?;
    validate_digest("object_fingerprint", &checkpoint.object_fingerprint)?;
    validate_digest("anchor_fingerprint", &checkpoint.anchor_fingerprint)?;
    validate_digest("checkpoint_digest", &checkpoint.checkpoint_digest)?;

    if checkpoint.revision == 0 {
        return Err("Continuity revision must be non-zero".into());
    }
    match (checkpoint.revision, checkpoint.previous_checkpoint.as_ref()) {
        (1, None) => {}
        (1, Some(_)) => return Err("Genesis checkpoint must not name a predecessor".into()),
        (_, None) => return Err("Non-genesis checkpoint must name a predecessor".into()),
        (_, Some(_)) => {}
    }

    if checkpoint.checkpoint_bytes.is_empty() {
        return Err("Opaque checkpoint bytes must not be empty".into());
    }
    if checkpoint.checkpoint_bytes.len() > MAX_CHECKPOINT_BYTES {
        return Err(format!(
            "Opaque checkpoint exceeds {} byte limit",
            MAX_CHECKPOINT_BYTES
        ));
    }
    let actual = digest_checkpoint_bytes(&checkpoint.checkpoint_bytes);
    if actual != checkpoint.checkpoint_digest {
        return Err("Checkpoint byte digest mismatch".into());
    }
    Ok(())
}

/// Pure direct-successor checks after the previous action has been resolved.
pub fn check_checkpoint_successor(
    previous_action: &ActionHash,
    previous: &ContinuityCheckpoint,
    candidate: &ContinuityCheckpoint,
) -> Result<(), String> {
    check_checkpoint_shape(previous)?;
    check_checkpoint_shape(candidate)?;

    if candidate.previous_checkpoint.as_ref() != Some(previous_action) {
        return Err("Candidate does not bind the exact previous Mycelix action".into());
    }
    if candidate.namespace != previous.namespace {
        return Err("Continuity namespace changed across successor".into());
    }
    if candidate.source_protocol != previous.source_protocol {
        return Err("External checkpoint protocol changed across successor".into());
    }
    if candidate.object_fingerprint != previous.object_fingerprint {
        return Err("Continuity object fingerprint changed across successor".into());
    }
    let expected_revision = previous
        .revision
        .checked_add(1)
        .ok_or_else(|| "Continuity revision overflow".to_string())?;
    if candidate.revision != expected_revision {
        return Err(format!(
            "Continuity revision must advance exactly by one: expected {}, got {}",
            expected_revision, candidate.revision
        ));
    }
    if candidate.observed_at < previous.observed_at {
        return Err("Continuity checkpoint timestamp regressed".into());
    }
    if candidate.anchor_fingerprint == previous.anchor_fingerprint {
        return Err("Successor must retain a different external anchor fingerprint".into());
    }
    if candidate.checkpoint_digest == previous.checkpoint_digest {
        return Err("Successor must retain different external checkpoint bytes".into());
    }
    Ok(())
}

/// Pure attestation shape checks independent of the checkpoint lookup.
pub fn check_attestation_shape(attestation: &WitnessAttestation) -> Result<(), String> {
    if attestation.schema_version != CONTINUITY_ATTESTATION_SCHEMA {
        return Err(format!(
            "Unsupported witness attestation schema: {:?}",
            attestation.schema_version
        ));
    }
    if attestation.revision == 0 {
        return Err("Witnessed continuity revision must be non-zero".into());
    }
    validate_digest("object_fingerprint", &attestation.object_fingerprint)?;
    validate_digest("checkpoint_digest", &attestation.checkpoint_digest)?;
    Ok(())
}

fn validate_text(field: &str, value: &str, max_bytes: usize) -> Result<(), String> {
    if value.trim().is_empty()
        || value != value.trim()
        || value.len() > max_bytes
        || value.chars().any(char::is_control)
    {
        return Err(format!("Invalid {field}"));
    }
    Ok(())
}

fn validate_digest(field: &str, value: &[u8]) -> Result<(), String> {
    if value.len() != DIGEST_BYTES {
        return Err(format!("{field} must be exactly {DIGEST_BYTES} bytes"));
    }
    if value.iter().all(|byte| *byte == 0) {
        return Err(format!("{field} must not be all zeroes"));
    }
    Ok(())
}

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
    if checkpoint.observed_at != action.timestamp {
        return invalid("Checkpoint observed_at must equal its Holochain action timestamp");
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
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
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
    if attestation.witnessed_at != action.timestamp {
        return invalid("Witnessed_at must equal its Holochain action timestamp");
    }

    let checkpoint_record = must_get_valid_record(attestation.checkpoint_action.clone())?;
    let checkpoint: ContinuityCheckpoint = checkpoint_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
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
        return invalid("Attestation metadata does not match the exact checkpoint");
    }
    if attestation.witnessed_at < checkpoint.observed_at {
        return invalid("Witness attestation predates the retained checkpoint");
    }

    Ok(ValidateCallbackResult::Valid)
}

/// DHT validation callback.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ContinuityCheckpoint(checkpoint) => {
                    validate_checkpoint_create(action, checkpoint)
                }
                EntryTypes::WitnessAttestation(attestation) => {
                    validate_attestation_create(action, attestation)
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
        FlatOp::RegisterDelete { .. } => {
            invalid("Continuity witness entries cannot be deleted")
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn timestamp(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    fn checkpoint(revision: u64, previous_checkpoint: Option<ActionHash>) -> ContinuityCheckpoint {
        let bytes = format!("signed-checkpoint-{revision}").into_bytes();
        ContinuityCheckpoint {
            schema_version: CONTINUITY_CHECKPOINT_SCHEMA.into(),
            namespace: "symthaea.episodic-continuity.xenia-anchor.v1".into(),
            source_protocol: "xenia-state-anchor-checkpoint-v1".into(),
            object_fingerprint: vec![0x11; 32],
            revision,
            anchor_fingerprint: vec![revision as u8; 32],
            checkpoint_digest: digest_checkpoint_bytes(&bytes),
            checkpoint_bytes: bytes,
            previous_checkpoint,
            observed_at: timestamp(revision as i64),
        }
    }

    #[test]
    fn digest_binds_length_and_bytes() {
        assert_ne!(digest_checkpoint_bytes(b"ab"), digest_checkpoint_bytes(b"a"));
        assert_ne!(digest_checkpoint_bytes(b"ab"), digest_checkpoint_bytes(b"ac"));
    }

    #[test]
    fn genesis_shape_is_valid() {
        assert!(check_checkpoint_shape(&checkpoint(1, None)).is_ok());
    }

    #[test]
    fn non_genesis_requires_predecessor() {
        let error = check_checkpoint_shape(&checkpoint(2, None)).unwrap_err();
        assert!(error.contains("predecessor"));
    }

    #[test]
    fn tampered_checkpoint_bytes_are_rejected() {
        let mut value = checkpoint(1, None);
        value.checkpoint_bytes.push(0xff);
        assert!(check_checkpoint_shape(&value).is_err());
    }

    #[test]
    fn direct_successor_is_strict() {
        let previous_action = ActionHash::from_raw_36(vec![0x22; 36]);
        let previous = checkpoint(1, None);
        let candidate = checkpoint(2, Some(previous_action.clone()));
        assert!(check_checkpoint_successor(&previous_action, &previous, &candidate).is_ok());
    }

    #[test]
    fn skipped_revision_is_rejected() {
        let previous_action = ActionHash::from_raw_36(vec![0x22; 36]);
        let previous = checkpoint(1, None);
        let candidate = checkpoint(3, Some(previous_action.clone()));
        assert!(check_checkpoint_successor(&previous_action, &previous, &candidate).is_err());
    }

    #[test]
    fn protocol_or_target_change_is_rejected() {
        let previous_action = ActionHash::from_raw_36(vec![0x22; 36]);
        let previous = checkpoint(1, None);
        let mut protocol_change = checkpoint(2, Some(previous_action.clone()));
        protocol_change.source_protocol = "different-protocol".into();
        assert!(
            check_checkpoint_successor(&previous_action, &previous, &protocol_change).is_err()
        );

        let mut target_change = checkpoint(2, Some(previous_action.clone()));
        target_change.object_fingerprint = vec![0x44; 32];
        assert!(check_checkpoint_successor(&previous_action, &previous, &target_change).is_err());
    }

    #[test]
    fn attestation_shape_requires_exact_digest_width() {
        let attestation = WitnessAttestation {
            schema_version: CONTINUITY_ATTESTATION_SCHEMA.into(),
            checkpoint_action: ActionHash::from_raw_36(vec![0x33; 36]),
            object_fingerprint: vec![0x11; 32],
            revision: 1,
            checkpoint_digest: vec![0x22; 31],
            witnessed_at: timestamp(1),
        };
        assert!(check_attestation_shape(&attestation).is_err());
    }
}

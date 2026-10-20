// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use hdi::prelude::*;
use sha2::{Digest, Sha256};

use crate::types::*;

const MAX_NAMESPACE_BYTES: usize = 256;
const MAX_SOURCE_PROTOCOL_BYTES: usize = 128;
const MAX_CHECKPOINT_BYTES: usize = 64 * 1024;
const DIGEST_BYTES: usize = 32;

pub fn digest_checkpoint_bytes(bytes: &[u8]) -> Vec<u8> {
    let mut hasher = Sha256::new();
    hasher.update(CHECKPOINT_DIGEST_DOMAIN);
    hasher.update((bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
    hasher.finalize().to_vec()
}

fn validate_text(field: &str, value: &str, max: usize) -> Result<(), String> {
    if value.trim().is_empty()
        || value != value.trim()
        || value.len() > max
        || value.chars().any(char::is_control)
    {
        Err(format!("Invalid {field}"))
    } else {
        Ok(())
    }
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

pub fn check_checkpoint_shape(value: &ContinuityCheckpoint) -> Result<(), String> {
    if value.schema_version != CONTINUITY_CHECKPOINT_SCHEMA {
        return Err("Unsupported continuity checkpoint schema".into());
    }
    validate_text("namespace", &value.namespace, MAX_NAMESPACE_BYTES)?;
    validate_text(
        "source_protocol",
        &value.source_protocol,
        MAX_SOURCE_PROTOCOL_BYTES,
    )?;
    validate_digest("object_fingerprint", &value.object_fingerprint)?;
    validate_digest("anchor_fingerprint", &value.anchor_fingerprint)?;
    validate_digest("checkpoint_digest", &value.checkpoint_digest)?;
    if value.revision == 0 {
        return Err("Continuity revision must be non-zero".into());
    }
    match (value.revision, value.previous_checkpoint.as_ref()) {
        (1, None) => {}
        (1, Some(_)) => return Err("Genesis checkpoint must not name a predecessor".into()),
        (_, None) => return Err("Non-genesis checkpoint must name a predecessor".into()),
        (_, Some(_)) => {}
    }
    if value.checkpoint_bytes.is_empty() {
        return Err("Opaque checkpoint bytes must not be empty".into());
    }
    if value.checkpoint_bytes.len() > MAX_CHECKPOINT_BYTES {
        return Err(format!(
            "Opaque checkpoint exceeds {MAX_CHECKPOINT_BYTES} byte limit"
        ));
    }
    if digest_checkpoint_bytes(&value.checkpoint_bytes) != value.checkpoint_digest {
        return Err("Checkpoint byte digest mismatch".into());
    }
    Ok(())
}

pub fn check_checkpoint_successor(
    previous_action: &ActionHash,
    previous: &ContinuityCheckpoint,
    candidate: &ContinuityCheckpoint,
) -> Result<(), String> {
    check_checkpoint_shape(previous)?;
    check_checkpoint_shape(candidate)?;
    if candidate.previous_checkpoint.as_ref() != Some(previous_action) {
        return Err("Candidate does not bind exact previous Mycelix action".into());
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
    let expected = previous
        .revision
        .checked_add(1)
        .ok_or_else(|| "Continuity revision overflow".to_string())?;
    if candidate.revision != expected {
        return Err(format!(
            "Continuity revision must advance exactly by one: expected {expected}, got {}",
            candidate.revision
        ));
    }
    if candidate.observed_at < previous.observed_at {
        return Err("Continuity checkpoint timestamp regressed".into());
    }
    if candidate.anchor_fingerprint == previous.anchor_fingerprint {
        return Err("Successor reused previous external anchor fingerprint".into());
    }
    if candidate.checkpoint_digest == previous.checkpoint_digest {
        return Err("Successor reused previous external checkpoint bytes".into());
    }
    Ok(())
}

pub fn check_attestation_shape(value: &WitnessAttestation) -> Result<(), String> {
    if value.schema_version != CONTINUITY_ATTESTATION_SCHEMA {
        return Err("Unsupported continuity attestation schema".into());
    }
    if value.revision == 0 {
        return Err("Witnessed continuity revision must be non-zero".into());
    }
    validate_digest("object_fingerprint", &value.object_fingerprint)?;
    validate_digest("checkpoint_digest", &value.checkpoint_digest)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checkpoint(revision: u64, previous: Option<ActionHash>) -> ContinuityCheckpoint {
        let bytes = format!("checkpoint-{revision}").into_bytes();
        ContinuityCheckpoint {
            schema_version: CONTINUITY_CHECKPOINT_SCHEMA.into(),
            namespace: "symthaea.episodic-continuity.xenia-anchor.v1".into(),
            source_protocol: "xenia-state-anchor-checkpoint-v1".into(),
            object_fingerprint: vec![0x11; 32],
            revision,
            anchor_fingerprint: vec![revision as u8; 32],
            checkpoint_digest: digest_checkpoint_bytes(&bytes),
            checkpoint_bytes: bytes,
            previous_checkpoint: previous,
            observed_at: Timestamp::from_micros(revision as i64),
        }
    }

    #[test]
    fn digest_binds_bytes() {
        assert_ne!(digest_checkpoint_bytes(b"a"), digest_checkpoint_bytes(b"b"));
    }

    #[test]
    fn genesis_is_valid() {
        assert!(check_checkpoint_shape(&checkpoint(1, None)).is_ok());
    }

    #[test]
    fn non_genesis_requires_predecessor() {
        assert!(check_checkpoint_shape(&checkpoint(2, None)).is_err());
    }

    #[test]
    fn tampering_breaks_digest() {
        let mut value = checkpoint(1, None);
        value.checkpoint_bytes.push(0xff);
        assert!(check_checkpoint_shape(&value).is_err());
    }

    #[test]
    fn direct_successor_is_strict() {
        let previous_hash = ActionHash::from_raw_36(vec![0x22; 36]);
        let previous = checkpoint(1, None);
        assert!(check_checkpoint_successor(
            &previous_hash,
            &previous,
            &checkpoint(2, Some(previous_hash.clone()))
        )
        .is_ok());
        assert!(check_checkpoint_successor(
            &previous_hash,
            &previous,
            &checkpoint(3, Some(previous_hash.clone()))
        )
        .is_err());
    }
}

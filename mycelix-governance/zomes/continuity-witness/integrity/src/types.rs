// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use hdi::prelude::*;

pub const CONTINUITY_CHECKPOINT_SCHEMA: &str = "mycelix.continuity-witness.checkpoint.v1";
pub const CONTINUITY_ATTESTATION_SCHEMA: &str = "mycelix.continuity-witness.attestation.v1";
pub const CHECKPOINT_DIGEST_DOMAIN: &[u8] = b"mycelix.continuity-witness.checkpoint-digest.v1\0";

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ContinuityCheckpoint {
    pub schema_version: String,
    pub namespace: String,
    pub source_protocol: String,
    pub object_fingerprint: Vec<u8>,
    pub revision: u64,
    pub anchor_fingerprint: Vec<u8>,
    pub checkpoint_digest: Vec<u8>,
    pub checkpoint_bytes: Vec<u8>,
    pub previous_checkpoint: Option<ActionHash>,
    /// Claimed observation time. The signed action timestamp is authoritative.
    pub observed_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WitnessAttestation {
    pub schema_version: String,
    pub checkpoint_action: ActionHash,
    pub object_fingerprint: Vec<u8>,
    pub revision: u64,
    pub checkpoint_digest: Vec<u8>,
    /// Claimed witness time. The signed action timestamp is authoritative.
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
    ObjectToCheckpoint,
    RevisionToCheckpoint,
    CheckpointToAttestation,
    CheckpointToSuccessor,
}

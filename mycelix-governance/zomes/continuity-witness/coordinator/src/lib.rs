// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Coordinator for purpose-separated external continuity checkpoint retention.

use continuity_witness_integrity::*;
use hdk::prelude::*;
use mycelix_zome_helpers as _;

const MAX_MINIMUM_WITNESSES: u32 = 1024;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SubmitCheckpointInput {
    pub namespace: String,
    pub source_protocol: String,
    pub object_fingerprint: Vec<u8>,
    pub revision: u64,
    pub anchor_fingerprint: Vec<u8>,
    pub checkpoint_bytes: Vec<u8>,
    pub previous_checkpoint: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RevisionWitnessQuery {
    pub namespace: String,
    pub object_fingerprint: Vec<u8>,
    pub revision: u64,
    pub minimum_distinct_witnesses: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CheckpointCandidateStatus {
    pub checkpoint_action: ActionHash,
    pub checkpoint: ContinuityCheckpoint,
    pub submitter: AgentPubKey,
    pub witnesses: Vec<AgentPubKey>,
    pub threshold_met: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RevisionWitnessStatus {
    pub namespace: String,
    pub object_fingerprint: Vec<u8>,
    pub revision: u64,
    pub minimum_distinct_witnesses: u32,
    pub fork_detected: bool,
    pub candidates: Vec<CheckpointCandidateStatus>,
}

impl RevisionWitnessStatus {
    pub fn usable(&self) -> bool {
        !self.fork_detected && self.candidates.len() == 1 && self.candidates[0].threshold_met
    }
}

fn anchor_hash(value: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(value.to_string())))
}

fn hex_bytes(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn revision_anchor(object_fingerprint: &[u8], revision: u64) -> String {
    format!("cw-rev:{}:{}", hex_bytes(object_fingerprint), revision)
}

fn checkpoint_from_record(record: &Record) -> ExternResult<Option<ContinuityCheckpoint>> {
    record.entry().to_app_option::<ContinuityCheckpoint>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))
}

fn attestation_from_record(record: &Record) -> ExternResult<Option<WitnessAttestation>> {
    record.entry().to_app_option::<WitnessAttestation>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))
}

fn revision_candidates(query: &RevisionWitnessQuery) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash(&revision_anchor(&query.object_fingerprint, query.revision))?,
            LinkTypes::RevisionToCheckpoint,
        )?,
        GetStrategy::default(),
    )?;
    let mut out = Vec::new();
    let mut seen = Vec::<ActionHash>::new();
    for link in links {
        let Ok(hash) = ActionHash::try_from(link.target) else { continue };
        if seen.contains(&hash) { continue; }
        let Some(record) = get(hash.clone(), GetOptions::default())? else { continue };
        let Some(checkpoint) = checkpoint_from_record(&record)? else { continue };
        if checkpoint.namespace == query.namespace
            && checkpoint.object_fingerprint == query.object_fingerprint
            && checkpoint.revision == query.revision
            && check_checkpoint_shape(&checkpoint).is_ok()
        {
            seen.push(hash);
            out.push(record);
        }
    }
    Ok(out)
}

fn distinct_witnesses(
    checkpoint_action: &ActionHash,
    checkpoint: &ContinuityCheckpoint,
    submitter: &AgentPubKey,
) -> ExternResult<Vec<AgentPubKey>> {
    let links = get_links(
        LinkQuery::try_new(checkpoint_action.clone(), LinkTypes::CheckpointToAttestation)?,
        GetStrategy::default(),
    )?;
    let mut witnesses = Vec::new();
    for link in links {
        let Ok(hash) = ActionHash::try_from(link.target) else { continue };
        let Some(record) = get(hash, GetOptions::default())? else { continue };
        let author = record.action().author().clone();
        if &author == submitter || witnesses.contains(&author) { continue; }
        let Some(attestation) = attestation_from_record(&record)? else { continue };
        if attestation.checkpoint_action == *checkpoint_action
            && attestation.object_fingerprint == checkpoint.object_fingerprint
            && attestation.revision == checkpoint.revision
            && attestation.checkpoint_digest == checkpoint.checkpoint_digest
            && check_attestation_shape(&attestation).is_ok()
        {
            witnesses.push(author);
        }
    }
    witnesses.sort_by(|a, b| a.as_ref().cmp(b.as_ref()));
    Ok(witnesses)
}

#[hdk_extern]
pub fn submit_continuity_checkpoint(input: SubmitCheckpointInput) -> ExternResult<Record> {
    let checkpoint = ContinuityCheckpoint {
        schema_version: CONTINUITY_CHECKPOINT_SCHEMA.into(),
        namespace: input.namespace.clone(),
        source_protocol: input.source_protocol,
        object_fingerprint: input.object_fingerprint.clone(),
        revision: input.revision,
        anchor_fingerprint: input.anchor_fingerprint,
        checkpoint_digest: digest_checkpoint_bytes(&input.checkpoint_bytes),
        checkpoint_bytes: input.checkpoint_bytes,
        previous_checkpoint: input.previous_checkpoint.clone(),
        observed_at: sys_time()?,
    };
    check_checkpoint_shape(&checkpoint)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

    let query = RevisionWitnessQuery {
        namespace: checkpoint.namespace.clone(),
        object_fingerprint: checkpoint.object_fingerprint.clone(),
        revision: checkpoint.revision,
        minimum_distinct_witnesses: 1,
    };
    if !revision_candidates(&query)?.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "A candidate already exists at this object/revision; inspect fork status".into()
        )));
    }

    let action_hash = create_entry(&EntryTypes::ContinuityCheckpoint(checkpoint.clone()))?;
    let key = revision_anchor(&checkpoint.object_fingerprint, checkpoint.revision);
    create_entry(&EntryTypes::Anchor(Anchor(key.clone())))?;
    create_link(anchor_hash(&key)?, action_hash.clone(), LinkTypes::RevisionToCheckpoint, ())?;
    if let Some(previous) = checkpoint.previous_checkpoint {
        create_link(previous, action_hash.clone(), LinkTypes::CheckpointToSuccessor, ())?;
    }
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Created checkpoint not readable".into()))
    })
}

#[hdk_extern]
pub fn attest_continuity_checkpoint(checkpoint_action: ActionHash) -> ExternResult<Record> {
    let checkpoint_record = get(checkpoint_action.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Checkpoint not found".into())))?;
    let checkpoint = checkpoint_from_record(&checkpoint_record)?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Target is not a checkpoint".into())))?;
    let witness = agent_info()?.agent_initial_pubkey;
    if checkpoint_record.action().author() == &witness {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Checkpoint submitter cannot be an independent witness".into()
        )));
    }
    for link in get_links(
        LinkQuery::try_new(checkpoint_action.clone(), LinkTypes::CheckpointToAttestation)?,
        GetStrategy::default(),
    )? {
        let Ok(hash) = ActionHash::try_from(link.target) else { continue };
        let Some(record) = get(hash, GetOptions::default())? else { continue };
        if record.action().author() == &witness { return Ok(record); }
    }

    let attestation = WitnessAttestation {
        schema_version: CONTINUITY_ATTESTATION_SCHEMA.into(),
        checkpoint_action: checkpoint_action.clone(),
        object_fingerprint: checkpoint.object_fingerprint.clone(),
        revision: checkpoint.revision,
        checkpoint_digest: checkpoint.checkpoint_digest.clone(),
        witnessed_at: sys_time()?,
    };
    let action_hash = create_entry(&EntryTypes::WitnessAttestation(attestation))?;
    create_link(checkpoint_action, action_hash.clone(), LinkTypes::CheckpointToAttestation, ())?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Created attestation not readable".into()))
    })
}

#[hdk_extern]
pub fn get_revision_witness_status(query: RevisionWitnessQuery) -> ExternResult<RevisionWitnessStatus> {
    if query.minimum_distinct_witnesses == 0 || query.minimum_distinct_witnesses > MAX_MINIMUM_WITNESSES {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "minimum_distinct_witnesses is outside the supported range".into()
        )));
    }
    let records = revision_candidates(&query)?;
    let mut candidates = Vec::new();
    for record in records {
        let checkpoint_action = record.action_address().clone();
        let submitter = record.action().author().clone();
        let Some(checkpoint) = checkpoint_from_record(&record)? else { continue };
        let witnesses = distinct_witnesses(&checkpoint_action, &checkpoint, &submitter)?;
        let threshold_met = witnesses.len() as u32 >= query.minimum_distinct_witnesses;
        candidates.push(CheckpointCandidateStatus {
            checkpoint_action,
            checkpoint,
            submitter,
            witnesses,
            threshold_met,
        });
    }
    let fork_detected = candidates.len() > 1;
    Ok(RevisionWitnessStatus {
        namespace: query.namespace,
        object_fingerprint: query.object_fingerprint,
        revision: query.revision,
        minimum_distinct_witnesses: query.minimum_distinct_witnesses,
        fork_detected,
        candidates,
    })
}

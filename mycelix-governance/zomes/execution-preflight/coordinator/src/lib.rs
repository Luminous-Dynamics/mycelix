// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Cross-layer execution preflight for binding governance.
//!
//! This coordinator performs no side effects. It re-resolves the exact current
//! proposal authority, constitutional epoch, current verified constitution, and
//! binding tally, then runs the pure execution-preflight kernel. Missing or
//! inconsistent dependencies are denial. The execution zome is expected to call
//! this boundary independently at Ready promotion and immediately before effects.

use hdk::prelude::*;
use mycelix_governance_authority::ProposalAuthorityContext;
use mycelix_governance_constitution::{ConstitutionStatement, Digest32 as ConstitutionDigest32};
use mycelix_governance_electorate::SnapshotBoundTally;
use mycelix_governance_execution_preflight::{
    authorize_execution_preflight, ExecutionPreflightInput, ExecutionPreflightPermit,
    VerifiedBindingTallyRef, ACTIONS_DIGEST_PROFILE_V1, PROTOCOL_VERSION,
};
use mycelix_institutional_core::Digest32 as InstitutionalDigest32;
use serde::de::DeserializeOwned;
use serde::Serialize;

const EXECUTION_AUTHORITY_DOMAIN: &[u8] = b"mycelix-governance-execution-authority-v1\0";
const TALLY_ACTION_REF_DOMAIN: &[u8] = b"mycelix/governance/binding-tally-action-ref/v1\0";
const TALLY_ACTION_REF_PROFILE: &str = "mycelix-binding-tally-action-ref-v1-blake3";
const MAX_ACTION_BYTES: usize = 4096;

/// Minimal mirror of the verified proposal-authority record.
#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct ProposalAuthorityBindingMirror {
    context: ProposalAuthorityContext,
    actions_digest_profile: String,
}

/// Minimal mirror of the verified proposal constitutional epoch.
#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct ProposalConstitutionEpochBindingMirror {
    proposal_id: String,
    proposal_authority_binding: ActionHash,
    election_configuration: ActionHash,
    constitution_statement_digest: ConstitutionDigest32,
    constitution_version: u64,
}

/// Minimal mirror of the authoritative recomputed binding tally.
#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct BindingTallyRecordMirror {
    proposal_id: String,
    proposal_authority_binding: ActionHash,
    election_configuration: ActionHash,
    tally: SnapshotBoundTally,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedCurrentConstitutionMirror {
    dna_hash: String,
    statement: ConstitutionStatement,
    statement_digest: ConstitutionDigest32,
    legacy_constitution_authoritative: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifyExecutionPreflightInput {
    pub proposal_id: String,
    /// Exact action JSON bytes carried by the current timelock.
    pub actions: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedExecutionPreflight {
    pub protocol: String,
    pub proposal_id: String,
    pub actions_digest: InstitutionalDigest32,
    pub actions_digest_profile: String,
    pub proposal_authority_binding: ActionHash,
    pub constitutional_epoch_binding: ActionHash,
    pub constitution_statement_digest: ConstitutionDigest32,
    pub constitution_version: u64,
    pub binding_tally_action: ActionHash,
    pub binding_tally_identity_digest: InstitutionalDigest32,
    pub binding_tally_identity_profile: String,
    pub checked_at_ms: u64,
    pub permit: ExecutionPreflightPermit,
}

fn call_local<I, O>(zome: &str, function: &str, input: I) -> ExternResult<O>
where
    I: Serialize,
    O: DeserializeOwned,
{
    let response = call(
        CallTargetCell::Local,
        ZomeName::from(zome),
        FunctionName::from(function),
        None,
        ExternIO::encode(input)
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;
    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} unavailable; execution preflight fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode {zome}::{function} response: {e}"
        )))
    })
}

fn require_record<T>(
    zome: &str,
    function: &str,
    proposal_id: &str,
) -> ExternResult<(Record, T)>
where
    T: TryFrom<SerializedBytes, Error = SerializedBytesError>,
{
    let record: Record = call_local::<_, Option<Record>>(zome, function, proposal_id.to_string())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} returned no verified record"
            )))
        })?;
    let value: T = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} record has no decodable application entry"
            )))
        })?;
    Ok((record, value))
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> ExternResult<u64> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} timestamp must be positive"
        ))));
    }
    Ok(micros as u64 / 1_000)
}

fn execution_authority_digest(proposal_id: &str, actions: &str) -> InstitutionalDigest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(EXECUTION_AUTHORITY_DOMAIN);
    hasher.update(&(proposal_id.len() as u64).to_le_bytes());
    hasher.update(proposal_id.as_bytes());
    hasher.update(&(actions.len() as u64).to_le_bytes());
    hasher.update(actions.as_bytes());
    InstitutionalDigest32(*hasher.finalize().as_bytes())
}

/// Identity commitment for the verified tally action. This is deliberately
/// named an action-ref digest rather than pretending to be a semantic tally
/// digest; the semantic validity comes from binding_voting's recomputation path.
fn tally_action_identity(action: &ActionHash) -> InstitutionalDigest32 {
    let text = action.to_string();
    let mut hasher = blake3::Hasher::new();
    hasher.update(TALLY_ACTION_REF_DOMAIN);
    hasher.update(&(text.len() as u64).to_be_bytes());
    hasher.update(text.as_bytes());
    InstitutionalDigest32(*hasher.finalize().as_bytes())
}

fn resolve_current_constitution() -> ExternResult<VerifiedCurrentConstitutionMirror> {
    let current: VerifiedCurrentConstitutionMirror = call_local(
        "constitution_transition",
        "get_verified_current_constitution",
        (),
    )?;
    if current.legacy_constitution_authoritative {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution preflight refuses a legacy constitutional authority plane".into(),
        )));
    }
    if current.dna_hash.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Current constitution returned an empty DNA hash".into(),
        )));
    }
    let digest = current.statement.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest current constitution: {e}"
        )))
    })?;
    if digest != current.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Current constitution statement digest does not recompute exactly".into(),
        )));
    }
    Ok(current)
}

#[hdk_extern]
pub fn verify_execution_preflight(
    input: VerifyExecutionPreflightInput,
) -> ExternResult<VerifiedExecutionPreflight> {
    if input.proposal_id.is_empty() || input.proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal id must be 1-256 bytes".into(),
        )));
    }
    if input.actions.is_empty() || input.actions.len() > MAX_ACTION_BYTES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution actions must be 1-{MAX_ACTION_BYTES} bytes"
        ))));
    }

    let (authority_record, authority): (Record, ProposalAuthorityBindingMirror) = require_record(
        "proposal_authority",
        "get_verified_proposal_authority_context",
        &input.proposal_id,
    )?;
    if authority.context.proposal_id.as_str() != input.proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority targets a different proposal".into(),
        )));
    }
    if authority.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority uses an unsupported action digest profile".into(),
        )));
    }

    let (epoch_record, epoch): (Record, ProposalConstitutionEpochBindingMirror) = require_record(
        "proposal_constitution_epoch",
        "get_verified_proposal_constitution_epoch",
        &input.proposal_id,
    )?;
    if epoch.proposal_id != input.proposal_id
        || epoch.proposal_authority_binding != *authority_record.action_address()
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Constitutional epoch is bound to different proposal authority".into(),
        )));
    }

    let current = resolve_current_constitution()?;
    if epoch.constitution_statement_digest != current.statement_digest
        || epoch.constitution_version != current.statement.version
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal constitutional epoch is stale; explicit governed reauthorization is required"
                .into(),
        )));
    }

    let (tally_record, tally): (Record, BindingTallyRecordMirror) = require_record(
        "binding_voting",
        "get_verified_binding_tally",
        &input.proposal_id,
    )?;
    if tally.proposal_id != input.proposal_id
        || tally.proposal_authority_binding != *authority_record.action_address()
        || tally.election_configuration != epoch.election_configuration
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Verified binding tally is bound to different authority/election semantics".into(),
        )));
    }
    if !tally.tally.tally.quorum_reached || !tally.tally.tally.approved {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Binding tally did not reach quorum and approval".into(),
        )));
    }
    if tally.tally.tally.proposal.as_str() != input.proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Binding tally payload targets a different proposal".into(),
        )));
    }

    let now = sys_time()?;
    let now_ms = timestamp_ms(now, "current")?;
    let actions_digest = execution_authority_digest(&input.proposal_id, &input.actions);
    let tally_identity = tally_action_identity(tally_record.action_address());
    let authority_ref = authority_record.action_address().to_string();
    let tally_action_ref = tally_record.action_address().to_string();

    let preflight_input = ExecutionPreflightInput {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: input.proposal_id.clone(),
        proposal_authority_binding_ref: authority_ref.clone(),
        authority: authority.context,
        actions_digest,
        actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
        constitution: current.statement,
        expected_constitution_statement_digest: current.statement_digest,
        binding_tally: VerifiedBindingTallyRef {
            proposal_id: input.proposal_id.clone(),
            tally_action_ref: tally_action_ref.clone(),
            tally_digest: tally_identity,
            tally_digest_profile: TALLY_ACTION_REF_PROFILE.into(),
            proposal_authority_binding_ref: authority_ref,
            verified_at_ms: now_ms,
            verification_receipt_ref: format!(
                "binding-voting:get_verified_binding_tally:{}",
                tally_record.action_address()
            ),
        },
        checked_at_ms: now_ms,
    };

    let permit = authorize_execution_preflight(&preflight_input).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Execution preflight denied: {e}"
        )))
    })?;

    Ok(VerifiedExecutionPreflight {
        protocol: "mycelix-governance-execution-preflight-runtime-v0.1".into(),
        proposal_id: input.proposal_id,
        actions_digest,
        actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
        proposal_authority_binding: authority_record.action_address().clone(),
        constitutional_epoch_binding: epoch_record.action_address().clone(),
        constitution_statement_digest: current.statement_digest,
        constitution_version: current.statement.version,
        binding_tally_action: tally_record.action_address().clone(),
        binding_tally_identity_digest: tally_identity,
        binding_tally_identity_profile: TALLY_ACTION_REF_PROFILE.into(),
        checked_at_ms: now_ms,
        permit,
    })
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExecutionPreflightStatus {
    pub protocol: String,
    pub constitutional_epoch_required: bool,
    pub approved_binding_tally_required: bool,
    pub legacy_vote_fallback: bool,
    pub advisory_score_fallback: bool,
}

#[hdk_extern]
pub fn get_execution_preflight_status(_: ()) -> ExternResult<ExecutionPreflightStatus> {
    Ok(ExecutionPreflightStatus {
        protocol: "mycelix-governance-execution-preflight-runtime-v0.1".into(),
        constitutional_epoch_required: true,
        approved_binding_tally_required: true,
        legacy_vote_fallback: false,
        advisory_score_fallback: false,
    })
}

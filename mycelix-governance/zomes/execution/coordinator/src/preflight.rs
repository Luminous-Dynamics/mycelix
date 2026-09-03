// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Thin fail-closed client for the binding-governance execution preflight.
//!
//! The heavy cross-zome verification lives in `governance_execution_preflight`.
//! This module only requires an exact typed response and checks that the response
//! still binds the exact timelock proposal/action bytes before threshold
//! verification may continue.

use execution_integrity::Timelock;
use hdk::prelude::*;
use mycelix_governance_constitution::Digest32 as ConstitutionDigest32;
use mycelix_governance_execution_preflight::ExecutionPreflightPermit;
use mycelix_institutional_core::Digest32 as InstitutionalDigest32;

const PREFLIGHT_ZOME: &str = "governance_execution_preflight";
const PREFLIGHT_FUNCTION: &str = "verify_execution_preflight";
const PREFLIGHT_PROTOCOL: &str = "mycelix-governance-execution-preflight-runtime-v0.1";
const ACTIONS_DIGEST_PROFILE: &str =
    "mycelix-governance-execution-authority-v1-blake3-exact-json";
const TALLY_IDENTITY_PROFILE: &str = "mycelix-binding-tally-action-ref-v1-blake3";

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyExecutionPreflightInput {
    proposal_id: String,
    actions: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedExecutionPreflight {
    protocol: String,
    proposal_id: String,
    actions_digest: InstitutionalDigest32,
    actions_digest_profile: String,
    proposal_authority_binding: ActionHash,
    constitutional_epoch_binding: ActionHash,
    constitution_statement_digest: ConstitutionDigest32,
    constitution_version: u64,
    binding_tally_action: ActionHash,
    binding_tally_identity_digest: InstitutionalDigest32,
    binding_tally_identity_profile: String,
    checked_at_ms: u64,
    permit: ExecutionPreflightPermit,
}

/// Require a fresh cross-layer execution preflight for these exact timelock
/// bytes. The caller supplies the independently recomputed execution digest so
/// this module never becomes a second source of action-canonicalization truth.
pub fn require_current_binding_authority(
    timelock: &Timelock,
    expected_actions_digest: [u8; 32],
) -> ExternResult<()> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::from(PREFLIGHT_ZOME),
        FunctionName::from(PREFLIGHT_FUNCTION),
        None,
        ExternIO::encode(VerifyExecutionPreflightInput {
            proposal_id: timelock.proposal_id.clone(),
            actions: timelock.actions.clone(),
        })
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;

    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Execution denied: {PREFLIGHT_ZOME}::{PREFLIGHT_FUNCTION} unavailable: {other:?}"
            ))));
        }
    };

    let verified: VerifiedExecutionPreflight = io.decode().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Execution denied: invalid execution-preflight response: {e}"
        )))
    })?;

    if verified.protocol != PREFLIGHT_PROTOCOL {
        return deny("unexpected execution-preflight protocol");
    }
    if verified.proposal_id != timelock.proposal_id {
        return deny("execution preflight targets another proposal");
    }
    if verified.actions_digest_profile != ACTIONS_DIGEST_PROFILE {
        return deny("execution preflight uses another action-digest profile");
    }
    if verified.actions_digest.0 != expected_actions_digest {
        return deny("execution preflight action digest differs from current timelock bytes");
    }
    if verified.constitution_statement_digest.is_zero() || verified.constitution_version == 0 {
        return deny("execution preflight returned an invalid constitutional epoch");
    }
    if verified.binding_tally_identity_digest.is_zero()
        || verified.binding_tally_identity_profile != TALLY_IDENTITY_PROFILE
    {
        return deny("execution preflight returned an invalid binding-tally identity");
    }
    if verified.checked_at_ms == 0 {
        return deny("execution preflight returned a zero verification timestamp");
    }

    // Read all authority-bearing action refs explicitly. Holochain action hashes
    // are already typed/non-empty; the exact string equality below ensures the
    // pure permit and runtime envelope refer to the same records.
    let authority_ref = verified.proposal_authority_binding.to_string();
    let epoch_ref = verified.constitutional_epoch_binding.to_string();
    let tally_ref = verified.binding_tally_action.to_string();
    if authority_ref.is_empty() || epoch_ref.is_empty() || tally_ref.is_empty() {
        return deny("execution preflight returned an empty authority action reference");
    }

    if verified.permit.proposal_id != timelock.proposal_id
        || verified.permit.proposal_authority_binding_ref != authority_ref
        || verified.permit.constitution_statement_digest
            != verified.constitution_statement_digest
        || verified.permit.tally_action_ref != tally_ref
        || verified.permit.checked_at_ms != verified.checked_at_ms
    {
        return deny("execution-preflight permit does not exactly bind its runtime envelope");
    }

    let now = sys_time()?;
    let now_us = now.as_micros();
    if now_us <= 0 {
        return deny("local clock is not positive");
    }
    let now_ms = now_us as u64 / 1_000;
    if verified.checked_at_ms > now_ms {
        return deny("execution-preflight verification timestamp is in the future");
    }

    Ok(())
}

fn deny<T>(reason: &str) -> ExternResult<T> {
    Err(wasm_error!(WasmErrorInner::Guest(format!(
        "Execution denied by constitutional preflight: {reason}"
    ))))
}

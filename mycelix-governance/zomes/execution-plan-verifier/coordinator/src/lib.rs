// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Immutable execution-plan verifier for binding governance.
//!
//! The execution plan is derived from the exact historical Proposal create action
//! referenced by the currently qualified ProposalAuthorityBinding. It never
//! follows mutable Timelock state and never treats a latest Proposal update as
//! executable-plan identity.
//!
//! This verifier proves provenance + exact bytes only. Constitutional legitimacy,
//! binding tally, threshold authority, executor designation and effect safety are
//! independently checked by later verifier layers.

use hdk::prelude::*;
use mycelix_institutional_core::Digest32;
use proposal_authority_integrity::{ProposalAuthorityBinding, ACTIONS_DIGEST_PROFILE_V1};
use proposals_integrity::{Proposal, ProposalStatus};
use serde::de::DeserializeOwned;
use serde::Serialize;

const PROTOCOL: &str = "mycelix-governance-execution-plan-verifier-v0.1";
const EXECUTION_AUTHORITY_DOMAIN: &[u8] = b"mycelix-governance-execution-authority-v1\0";
const MAX_ACTION_BYTES: usize = 4096;
const MAX_REF_BYTES: usize = 2048;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedExecutionPlan {
    pub protocol: String,
    pub proposal_id: String,
    /// Immutable plan identity derived from the historical Proposal create action.
    pub plan_ref: String,
    pub actions: String,
    pub actions_digest: Digest32,
    pub actions_digest_profile: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExecutionPlanVerifierStatus {
    pub protocol: String,
    pub historical_create_action_required: bool,
    pub proposal_authority_binding_required: bool,
    pub mutable_timelock_authority: bool,
    pub latest_proposal_update_authority: bool,
    pub institutional_authority_minted_here: bool,
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
                "{zome}::{function} unavailable; execution-plan verification fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode {zome}::{function} response: {e}"
        )))
    })
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

fn require_ref(value: &str, field: &str) -> ExternResult<()> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} must be 1-{MAX_REF_BYTES} bytes"
        ))));
    }
    Ok(())
}

/// MUST remain byte-compatible with proposal-authority and execution preflight
/// until the registered canonical profile is versioned together.
fn execution_authority_digest(proposal_id: &str, actions: &str) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(EXECUTION_AUTHORITY_DOMAIN);
    hasher.update(&(proposal_id.len() as u64).to_le_bytes());
    hasher.update(proposal_id.as_bytes());
    hasher.update(&(actions.len() as u64).to_le_bytes());
    hasher.update(actions.as_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn decode_authority_binding(record: &Record) -> ExternResult<ProposalAuthorityBinding> {
    record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Verified proposal authority record has no decodable binding".into(),
            ))
        })
}

fn load_current_authority_binding(
    proposal_id: &str,
) -> ExternResult<(Record, ProposalAuthorityBinding)> {
    let record: Record = call_local::<_, Option<Record>>(
        "proposal_authority",
        "get_verified_proposal_authority_context",
        proposal_id.to_string(),
    )?
    .ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "No unambiguous current proposal authority binding exists".into(),
        ))
    })?;
    let binding = decode_authority_binding(&record)?;
    binding.validate_structure().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Proposal authority binding is structurally invalid: {e}"
        )))
    })?;
    if binding.context.proposal_id.as_str() != proposal_id
        || binding.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority binding targets another proposal or digest profile".into(),
        )));
    }
    Ok((record, binding))
}

fn load_historical_create(binding: &ProposalAuthorityBinding) -> ExternResult<(Record, Proposal)> {
    let record = get(
        binding.proposal_action_hash.clone(),
        GetOptions::default(),
    )?
    .ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Proposal authority binding references a missing historical proposal action".into(),
        ))
    })?;

    if *record.action_address() != binding.proposal_action_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Historical proposal lookup returned another action".into(),
        )));
    }
    if !matches!(record.action(), Action::Create(_)) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution plan must originate from an immutable Proposal create action".into(),
        )));
    }

    let proposal: Proposal = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Historical proposal action has no decodable Proposal entry".into(),
            ))
        })?;

    if proposal.id != binding.context.proposal_id.as_str()
        || proposal.author != binding.proposal_author
        || proposal.status != ProposalStatus::Draft
        || proposal.version != 1
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Historical proposal create semantics do not match the authority binding"
                .into(),
        )));
    }
    if proposal.actions.is_empty() || proposal.actions.len() > MAX_ACTION_BYTES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Historical proposal actions must be 1-{MAX_ACTION_BYTES} bytes"
        ))));
    }
    if serde_json::from_str::<serde_json::Value>(&proposal.actions).is_err() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Historical proposal actions are not valid JSON".into(),
        )));
    }

    let expected_author = format!("did:mycelix:{}", record.action().author());
    if proposal.author != expected_author || binding.proposal_author != expected_author {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Historical Proposal create action is not author-bound to the authority binding"
                .into(),
        )));
    }

    Ok((record, proposal))
}

fn plan_ref(proposal_action: &ActionHash, actions_digest: Digest32) -> String {
    format!(
        "proposal-create:{}:{}:{}",
        proposal_action,
        ACTIONS_DIGEST_PROFILE_V1,
        digest_hex(actions_digest),
    )
}

fn digest_hex(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(64);
    for byte in digest.0 {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[hdk_extern]
pub fn resolve_execution_plan(proposal_id: String) -> ExternResult<VerifiedExecutionPlan> {
    if proposal_id.is_empty() || proposal_id.len() > 256 || !proposal_id.starts_with("MIP-") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution-plan verification requires an MIP proposal id of 1-256 bytes".into(),
        )));
    }

    let (authority_record, binding) = load_current_authority_binding(&proposal_id)?;
    let (proposal_record, proposal) = load_historical_create(&binding)?;
    let digest = execution_authority_digest(&proposal.id, &proposal.actions);
    if digest != binding.context.actions_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Historical proposal action bytes do not match the authority binding digest".into(),
        )));
    }

    let stable_plan_ref = plan_ref(proposal_record.action_address(), digest);
    require_ref(&stable_plan_ref, "execution plan ref")?;
    let verification_ref = format!(
        "proposal-authority-binding:{};proposal-create:{}",
        authority_record.action_address(),
        proposal_record.action_address(),
    );
    require_ref(&verification_ref, "execution plan verification ref")?;
    let verified_at_ms = timestamp_ms(sys_time()?, "execution plan verification")?;

    Ok(VerifiedExecutionPlan {
        protocol: PROTOCOL.into(),
        proposal_id,
        plan_ref: stable_plan_ref,
        actions: proposal.actions,
        actions_digest: digest,
        actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
        verification_ref,
        verified_at_ms,
    })
}

#[hdk_extern]
pub fn get_execution_plan_verifier_status(_: ()) -> ExternResult<ExecutionPlanVerifierStatus> {
    Ok(ExecutionPlanVerifierStatus {
        protocol: PROTOCOL.into(),
        historical_create_action_required: true,
        proposal_authority_binding_required: true,
        mutable_timelock_authority: false,
        latest_proposal_update_authority: false,
        institutional_authority_minted_here: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn execution_digest_is_exact_byte_sensitive() {
        let a = execution_authority_digest("MIP-42", "[{\"type\":\"noop\"}]");
        let same = execution_authority_digest("MIP-42", "[{\"type\":\"noop\"}]");
        let whitespace = execution_authority_digest("MIP-42", "[{ \"type\": \"noop\" }]");
        assert_eq!(a, same);
        assert_ne!(a, whitespace);
    }

    #[test]
    fn plan_ref_binds_create_action_profile_and_digest() {
        let a = ActionHash::from_raw_36(vec![1; 36]);
        let b = ActionHash::from_raw_36(vec![2; 36]);
        let d1 = Digest32([3; 32]);
        let d2 = Digest32([4; 32]);
        assert_eq!(plan_ref(&a, d1), plan_ref(&a, d1));
        assert_ne!(plan_ref(&a, d1), plan_ref(&b, d1));
        assert_ne!(plan_ref(&a, d1), plan_ref(&a, d2));
    }
}

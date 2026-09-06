// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Workspace-only source-head verification adapter.
//!
//! Source response acquisition and cryptographic proof verification are separate
//! boundaries. This coordinator then runs the pure source-head authentication
//! kernel. It does not decide institutional source trust or completeness.

use hdk::prelude::*;
use mycelix_authority_source_head_verifier::{
    qualify_source_head_authentication, VerifiedSourceHeadProof,
};
use mycelix_authority_state_coverage::{
    AuthoritySourceHeadAttestation, VerifiedAuthoritySourceHead,
    SOURCE_HEAD_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::VerifiedCoverageChallenge;
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-source-head-runtime-v0.1";
const SOURCE_RESPONDER_ZOME: &str = "authority_state_source_responder";
const SOURCE_PROOF_VERIFIER_ZOME: &str = "authority_source_proof_verifier";

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifySourceHeadRequest {
    pub challenge: VerifiedCoverageChallenge,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifySourceProofRequest {
    pub challenge_digest: Digest32,
    pub attestation_digest: Digest32,
    pub attestation_profile: String,
    pub attestation: AuthoritySourceHeadAttestation,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SourceHeadRuntimeStatus {
    pub protocol: String,
    pub source_response_is_candidate_only: bool,
    pub proof_verifier_separate: bool,
    pub institutional_source_trust_decided_here: bool,
    pub completeness_decided_here: bool,
    pub external_effects_enabled: bool,
    pub operational: bool,
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
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?,
    )?;
    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} unavailable; source-head verification fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot decode {zome}::{function} response: {error}"
        )))
    })
}

fn now_ms() -> ExternResult<u64> {
    let micros = sys_time()?.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "current time must be positive".into(),
        )));
    }
    Ok(micros as u64 / 1_000)
}

/// Verify one exact source-head response under one exact already-verified probe.
///
/// The responder returns candidate bytes only. A separate proof verifier must
/// authenticate those exact bytes/source identity before the pure qualifier can
/// project the existing `VerifiedAuthoritySourceHead` ABI.
#[hdk_extern]
pub fn verify_source_head(
    request: VerifySourceHeadRequest,
) -> ExternResult<VerifiedAuthoritySourceHead> {
    let now = now_ms()?;
    request.challenge.challenge.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid source-head challenge: {error}"
        )))
    })?;
    let challenge_digest = request.challenge.challenge.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute source-head challenge identity: {error}"
        )))
    })?;

    let attestation: AuthoritySourceHeadAttestation = call_local(
        SOURCE_RESPONDER_ZOME,
        "resolve_source_head_attestation",
        request.challenge.clone(),
    )?;
    let attestation_digest = attestation.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid source-head attestation candidate: {error}"
        )))
    })?;

    let proof: VerifiedSourceHeadProof = call_local(
        SOURCE_PROOF_VERIFIER_ZOME,
        "verify_source_head_proof",
        VerifySourceProofRequest {
            challenge_digest,
            attestation_digest,
            attestation_profile: SOURCE_HEAD_IDENTITY_PROFILE.into(),
            attestation: attestation.clone(),
        },
    )?;

    let qualified = qualify_source_head_authentication(
        &request.challenge,
        &attestation,
        &proof,
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "source-head authentication denied: {error}"
        )))
    })?;

    Ok(qualified.to_verified_source_head())
}

/// Declarative only. The responder and proof-verifier roles are intentionally
/// unprovisioned and this zome is absent from the binding DNA in this tranche.
#[hdk_extern]
pub fn source_head_runtime_status(_: ()) -> ExternResult<SourceHeadRuntimeStatus> {
    Ok(SourceHeadRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        source_response_is_candidate_only: true,
        proof_verifier_separate: true,
        institutional_source_trust_decided_here: false,
        completeness_decided_here: false,
        external_effects_enabled: false,
        operational: false,
    })
}

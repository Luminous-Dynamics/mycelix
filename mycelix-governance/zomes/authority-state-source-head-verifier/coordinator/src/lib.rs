// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Workspace-only source-head verification adapter.
//!
//! The caller supplies only a probe action hash. This coordinator reconstructs
//! the positive challenge receipt through the private-entropy probe verifier,
//! then keeps source response acquisition and cryptographic proof verification
//! as separate boundaries before running the pure source-head authentication
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
const PROBE_VERIFIER_ZOME: &str = "authority_state_challenge";
const PROBE_VERIFIER_FUNCTION: &str = "verify_issued_authority_state_probe";
const SOURCE_RESPONDER_ZOME: &str = "authority_state_source_responder";
const SOURCE_PROOF_VERIFIER_ZOME: &str = "authority_source_proof_verifier";

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifySourceHeadRequest {
    /// Reference to one locally issued probe. Positive challenge verification is
    /// reconstructed inside this runtime; callers cannot submit a verified receipt.
    pub probe_action: ActionHash,
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
    pub caller_supplied_verified_challenge_accepted: bool,
    pub probe_reverified_locally: bool,
    pub verified_challenge_exposed_to_responder: bool,
    pub source_response_is_candidate_only: bool,
    pub proof_verifier_separate: bool,
    pub validity_horizon_widening_allowed: bool,
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

/// Verify one exact source-head response under one exact locally reverified probe.
///
/// Caller input is only the probe action hash. The private-entropy verifier must
/// first reconstruct `VerifiedCoverageChallenge`. The source responder receives
/// only the underlying `CoverageChallenge`, never the positive verification receipt.
/// It returns candidate bytes only; a separate proof verifier authenticates those
/// exact bytes/source identity before the pure qualifier runs.
#[hdk_extern]
pub fn verify_source_head(
    request: VerifySourceHeadRequest,
) -> ExternResult<VerifiedAuthoritySourceHead> {
    let challenge: VerifiedCoverageChallenge = call_local(
        PROBE_VERIFIER_ZOME,
        PROBE_VERIFIER_FUNCTION,
        request.probe_action,
    )?;
    challenge.challenge.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "locally reverified source-head challenge is invalid: {error}"
        )))
    })?;
    let challenge_digest = challenge.challenge.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute source-head challenge identity: {error}"
        )))
    })?;

    // The responder needs the challenge semantics in order to answer it, but it
    // receives no positive proof that this runtime accepted the probe provenance.
    let attestation: AuthoritySourceHeadAttestation = call_local(
        SOURCE_RESPONDER_ZOME,
        "resolve_source_head_attestation",
        challenge.challenge.clone(),
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

    // Qualification time must be sampled after every evidence-producing call.
    // Otherwise a freshly verified receipt can appear to come from the future
    // relative to a timestamp captured before the verifier ran.
    let now = now_ms()?;
    let qualified = qualify_source_head_authentication(&challenge, &attestation, &proof, now)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "source-head authentication denied: {error}"
            )))
        })?;

    // `QualifiedSourceHeadAuthentication` carries the exact minimum of challenge,
    // attestation and cryptographic-verifier horizons. The legacy
    // `VerifiedAuthoritySourceHead` ABI carries only the attestation expiry. Never
    // discard a tighter horizon and thereby widen reusable authority. Until the
    // downstream ABI grows an explicit verifier horizon, deny any lossy projection.
    let effective_valid_until_ms = qualified.valid_until_ms();
    let projected = qualified.to_verified_source_head();
    if projected.attestation.expires_at_ms > effective_valid_until_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "source-head projection would widen validity from {effective_valid_until_ms} to {}; refusing lossy authority projection",
            projected.attestation.expires_at_ms
        ))));
    }

    Ok(projected)
}

/// Declarative only. The probe verifier, responder and proof-verifier roles are
/// intentionally unprovisioned and this zome is absent from the binding DNA in
/// this tranche.
#[hdk_extern]
pub fn source_head_runtime_status(_: ()) -> ExternResult<SourceHeadRuntimeStatus> {
    Ok(SourceHeadRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        caller_supplied_verified_challenge_accepted: false,
        probe_reverified_locally: true,
        verified_challenge_exposed_to_responder: false,
        source_response_is_candidate_only: true,
        proof_verifier_separate: true,
        validity_horizon_widening_allowed: false,
        institutional_source_trust_decided_here: false,
        completeness_decided_here: false,
        external_effects_enabled: false,
        operational: false,
    })
}

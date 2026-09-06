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
use mycelix_authority_evidence_lease::{EvidenceLease, LeasedEvidence};
use mycelix_authority_source_head_verifier::{
    qualify_source_head_authentication, QualifiedSourceHeadAuthentication,
    VerifiedSourceHeadProof,
};
use mycelix_authority_state_coverage::{
    AuthoritySourceHeadAttestation, VerifiedAuthoritySourceHead, SOURCE_HEAD_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::VerifiedCoverageChallenge;
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-source-head-runtime-v0.2";
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
    pub legacy_projection_fail_closed: bool,
    pub leased_endpoint_available: bool,
    pub leased_endpoint_preserves_exact_horizon: bool,
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

/// Run the complete source-head authentication composition once and retain the
/// non-deserializable #130 result until the caller chooses a projection surface.
fn qualify_request(
    request: VerifySourceHeadRequest,
) -> ExternResult<(QualifiedSourceHeadAuthentication, u64)> {
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

    // The responder needs challenge semantics in order to answer it, but receives
    // no positive proof that this runtime accepted the probe provenance.
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

    // Qualification time follows every evidence-producing call.
    let now = now_ms()?;
    let qualified = qualify_source_head_authentication(&challenge, &attestation, &proof, now)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "source-head authentication denied: {error}"
            )))
        })?;
    Ok((qualified, now))
}

/// Legacy compatibility endpoint.
///
/// `VerifiedAuthoritySourceHead` has no independent verifier-expiry field, so this
/// endpoint remains fail-closed whenever projection would discard a tighter #130
/// horizon. New currentness composition should use `verify_source_head_leased`.
#[hdk_extern]
pub fn verify_source_head(
    request: VerifySourceHeadRequest,
) -> ExternResult<VerifiedAuthoritySourceHead> {
    let (qualified, _) = qualify_request(request)?;
    let effective_valid_until_ms = qualified.valid_until_ms();
    let projected = qualified.to_verified_source_head();
    if projected.attestation.expires_at_ms > effective_valid_until_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "source-head projection would widen validity from {effective_valid_until_ms} to {}; refusing lossy legacy authority projection",
            projected.attestation.expires_at_ms
        ))));
    }
    Ok(projected)
}

/// Lease-complete transport endpoint.
///
/// The exact legacy compatibility receipt is paired with #130's true minimum
/// challenge/attestation/cryptographic-verifier horizon. Deserialization of the
/// envelope grants no authority; a downstream composer must still run #94/#96/#91
/// and cap any reusable positive authority to this lease.
#[hdk_extern]
pub fn verify_source_head_leased(
    request: VerifySourceHeadRequest,
) -> ExternResult<LeasedEvidence<VerifiedAuthoritySourceHead>> {
    let (qualified, now) = qualify_request(request)?;
    let lease = EvidenceLease::new(
        qualified.verified_at_ms(),
        qualified.valid_until_ms(),
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "qualified source-head evidence lease denied: {error}"
        )))
    })?;
    LeasedEvidence::new(qualified.to_verified_source_head(), lease, now).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot construct leased source-head evidence: {error}"
        )))
    })
}

#[hdk_extern]
pub fn source_head_runtime_status(_: ()) -> ExternResult<SourceHeadRuntimeStatus> {
    Ok(SourceHeadRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        caller_supplied_verified_challenge_accepted: false,
        probe_reverified_locally: true,
        verified_challenge_exposed_to_responder: false,
        source_response_is_candidate_only: true,
        proof_verifier_separate: true,
        legacy_projection_fail_closed: true,
        leased_endpoint_available: true,
        leased_endpoint_preserves_exact_horizon: true,
        institutional_source_trust_decided_here: false,
        completeness_decided_here: false,
        external_effects_enabled: false,
        operational: false,
    })
}

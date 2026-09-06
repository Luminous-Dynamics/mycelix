// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Workspace-only split witness verification composition adapter.
//!
//! This coordinator preserves the existing `verify_witness_evidence` wire ABI
//! consumed by the current-freshness runtime while removing the previously opaque
//! witness oracle behind it. Context lookup, observation discovery, observation
//! authentication and institutional trust classification are separate roles.
//! Positive #94/#96 compatibility receipts are constructed only after local #168
//! pure qualification.

use hdk::prelude::*;
use mycelix_authority_freshness::{AuthoritySubjectRef, ProfiledDigest};
use mycelix_authority_state_coverage::{
    AuthorityHeadWitnessObservation, VerifiedAuthorityHeadWitness,
    VerifiedAuthoritySourceHead, WITNESS_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageChallenge,
    VerifiedWitnessTrustBinding, CONTEXT_POLICY_PROFILE,
};
use mycelix_authority_witness_verifier::{
    qualify_witness_evidence, VerifiedWitnessObservationProof,
    VerifiedWitnessTrustClassificationProof,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-witness-runtime-v0.1";
const CONTEXT_POLICY_CANDIDATE_PROVIDER_ZOME: &str =
    "authority_state_context_policy_candidate_provider";
const WITNESS_CANDIDATE_PROVIDER_ZOME: &str = "authority_state_witness_candidate_provider";
const OBSERVATION_PROOF_VERIFIER_ZOME: &str =
    "authority_state_witness_observation_proof_verifier";
const TRUST_CLASSIFICATION_VERIFIER_ZOME: &str =
    "authority_state_witness_trust_classification_verifier";
const MAX_WITNESSES: usize = 64;

/// Public ABI retained for the existing current-freshness coordinator.
///
/// The source-head receipt is expected to have been authenticated by the caller's
/// independent source-head path. This zome cross-binds it but does not claim to
/// authenticate the source itself.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessVerificationRequest {
    pub challenge: VerifiedCoverageChallenge,
    pub source_head: VerifiedAuthoritySourceHead,
}

/// Existing compatibility bundle. The two vectors are produced only by local
/// #168 qualification, never supplied directly by a provider.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedWitnessEvidenceBundle {
    pub witnesses: Vec<VerifiedAuthorityHeadWitness>,
    pub trust_bindings: Vec<VerifiedWitnessTrustBinding>,
}

/// Candidate-only semantic context lookup, constrained by the exact digest and
/// profile already committed by the verified challenge.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ContextPolicyCandidateRequest {
    pub context_policy_digest: Digest32,
    pub context_policy_profile: String,
}

/// Discovery request for raw witness observations only.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessCandidateDiscoveryRequest {
    pub subject: AuthoritySubjectRef,
    pub authoritative_source_ref: String,
    pub source_head_digest: Digest32,
    pub head_generation: u64,
    pub head_transition_digest: Digest32,
    pub head_status_record_ref: String,
    pub challenge_digest: Digest32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessObservationProofVerificationRequest {
    pub observation: AuthorityHeadWitnessObservation,
    pub observation_digest: Digest32,
    pub observation_profile: String,
}

/// Trust classification deliberately contains no requested domain. The verifier
/// answers which domain contains this observer under the exact policy/verifier.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessTrustClassificationVerificationRequest {
    pub observer_id: String,
    pub witness_trust_policy: ProfiledDigest,
    pub expected_trust_verifier_ref: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessRuntimeStatus {
    pub protocol: String,
    pub context_discovery_grants_authority: bool,
    pub witness_discovery_grants_authority: bool,
    pub observation_proof_verifier_separate: bool,
    pub trust_classification_verifier_separate: bool,
    pub trust_binding_constructed_locally: bool,
    pub source_head_authenticated_here: bool,
    pub quorum_decided_here: bool,
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
                "{zome}::{function} unavailable; witness verification fails closed: {other:?}"
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

fn resolve_exact_context(
    challenge: &VerifiedCoverageChallenge,
) -> ExternResult<CoverageTrustContextPolicy> {
    challenge.challenge.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid witness coverage challenge: {error}"
        )))
    })?;
    if challenge.challenge.context_policy_profile != CONTEXT_POLICY_PROFILE {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "witness challenge uses the wrong context-policy profile".into(),
        )));
    }

    let context: CoverageTrustContextPolicy = call_local(
        CONTEXT_POLICY_CANDIDATE_PROVIDER_ZOME,
        "resolve_context_policy_candidate",
        ContextPolicyCandidateRequest {
            context_policy_digest: challenge.challenge.context_policy_digest,
            context_policy_profile: challenge.challenge.context_policy_profile.clone(),
        },
    )?;
    context.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid witness context-policy candidate: {error}"
        )))
    })?;
    let digest = context.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute witness context-policy identity: {error}"
        )))
    })?;
    if digest != challenge.challenge.context_policy_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "witness context provider returned a different policy identity".into(),
        )));
    }
    Ok(context)
}

/// Preserve the existing current-freshness runtime ABI while internally splitting
/// witness discovery, observation authentication and trust classification.
#[hdk_extern]
pub fn verify_witness_evidence(
    request: WitnessVerificationRequest,
) -> ExternResult<VerifiedWitnessEvidenceBundle> {
    request.challenge.challenge.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid witness verification challenge: {error}"
        )))
    })?;
    request.source_head.attestation.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid authenticated source-head input: {error}"
        )))
    })?;
    if request.source_head.attestation.subject != request.challenge.challenge.subject {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "witness source head belongs to another challenge subject".into(),
        )));
    }

    let context = resolve_exact_context(&request.challenge)?;

    // DirectSource has no witness/trust authority. Do not invoke witness providers
    // merely because code for them exists.
    let Some(witness_trust_policy) = context.witness_trust_policy.clone() else {
        if context.witness_trust_verifier_ref.is_some() {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "witness context has verifier without trust policy".into(),
            )));
        }
        return Ok(VerifiedWitnessEvidenceBundle {
            witnesses: Vec::new(),
            trust_bindings: Vec::new(),
        });
    };
    let expected_trust_verifier_ref = context
        .witness_trust_verifier_ref
        .clone()
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "witness context requires trust policy but has no verifier".into(),
            ))
        })?;

    let challenge_digest = request
        .challenge
        .challenge
        .identity_digest()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "cannot compute witness challenge identity: {error}"
            )))
        })?;
    let source_head_digest = request
        .source_head
        .attestation
        .identity_digest()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "cannot compute witness source-head identity: {error}"
            )))
        })?;
    let source = &request.source_head.attestation;

    let candidates: Vec<AuthorityHeadWitnessObservation> = call_local(
        WITNESS_CANDIDATE_PROVIDER_ZOME,
        "discover_witness_observations",
        WitnessCandidateDiscoveryRequest {
            subject: source.subject.clone(),
            authoritative_source_ref: source.authoritative_source_ref.clone(),
            source_head_digest,
            head_generation: source.head_generation,
            head_transition_digest: source.head_transition_digest,
            head_status_record_ref: source.head_status_record_ref.clone(),
            challenge_digest,
        },
    )?;
    if candidates.len() > MAX_WITNESSES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "witness candidate count exceeds {MAX_WITNESSES}"
        ))));
    }

    let mut witnesses = Vec::with_capacity(candidates.len());
    let mut trust_bindings = Vec::with_capacity(candidates.len());
    for observation in candidates {
        observation.validate().map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "invalid witness observation candidate: {error}"
            )))
        })?;
        let observation_digest = observation.identity_digest().map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "cannot compute witness observation identity: {error}"
            )))
        })?;

        let observation_proof: VerifiedWitnessObservationProof = call_local(
            OBSERVATION_PROOF_VERIFIER_ZOME,
            "verify_witness_observation_proof",
            WitnessObservationProofVerificationRequest {
                observation: observation.clone(),
                observation_digest,
                observation_profile: WITNESS_IDENTITY_PROFILE.into(),
            },
        )?;
        let classification_proof: VerifiedWitnessTrustClassificationProof = call_local(
            TRUST_CLASSIFICATION_VERIFIER_ZOME,
            "verify_witness_trust_classification",
            WitnessTrustClassificationVerificationRequest {
                observer_id: observation.observer_id.clone(),
                witness_trust_policy: witness_trust_policy.clone(),
                expected_trust_verifier_ref: expected_trust_verifier_ref.clone(),
            },
        )?;

        // Qualification time follows both evidence-producing verifiers. #168 then
        // binds their independent facts to the exact context/challenge/source and
        // constructs #94/#96 compatibility receipts locally.
        let now = now_ms()?;
        let qualified = qualify_witness_evidence(
            &context,
            &request.challenge,
            &request.source_head,
            &observation,
            &observation_proof,
            &classification_proof,
            now,
        )
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "witness evidence qualification denied: {error}"
            )))
        })?;
        witnesses.push(qualified.to_verified_witness());
        trust_bindings.push(qualified.to_verified_trust_binding());
    }

    Ok(VerifiedWitnessEvidenceBundle {
        witnesses,
        trust_bindings,
    })
}

#[hdk_extern]
pub fn witness_runtime_status(_: ()) -> ExternResult<WitnessRuntimeStatus> {
    Ok(WitnessRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        context_discovery_grants_authority: false,
        witness_discovery_grants_authority: false,
        observation_proof_verifier_separate: true,
        trust_classification_verifier_separate: true,
        trust_binding_constructed_locally: true,
        source_head_authenticated_here: false,
        quorum_decided_here: false,
        external_effects_enabled: false,
        operational: false,
    })
}

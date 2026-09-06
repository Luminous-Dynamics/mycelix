// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Workspace-only split witness verification composition adapter.
//!
//! Context lookup, observation discovery, observation authentication and
//! institutional trust classification are separate roles. The legacy endpoint
//! remains fail-closed when its ABI cannot preserve a tighter verifier horizon;
//! the leased endpoint carries that horizon explicitly.

use hdk::prelude::*;
use mycelix_authority_evidence_lease::{EvidenceLease, LeasedEvidence};
use mycelix_authority_freshness::{AuthoritySubjectRef, ProfiledDigest};
use mycelix_authority_state_coverage::{
    AuthorityHeadWitnessObservation, VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
    WITNESS_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageChallenge, VerifiedWitnessTrustBinding,
    CONTEXT_POLICY_PROFILE,
};
use mycelix_authority_witness_verifier::{
    qualify_witness_evidence, QualifiedWitnessEvidence, VerifiedWitnessObservationProof,
    VerifiedWitnessTrustClassificationProof,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-witness-runtime-v0.2";
const CONTEXT_POLICY_CANDIDATE_PROVIDER_ZOME: &str =
    "authority_state_context_policy_candidate_provider";
const WITNESS_CANDIDATE_PROVIDER_ZOME: &str = "authority_state_witness_candidate_provider";
const OBSERVATION_PROOF_VERIFIER_ZOME: &str =
    "authority_state_witness_observation_proof_verifier";
const TRUST_CLASSIFICATION_VERIFIER_ZOME: &str =
    "authority_state_witness_trust_classification_verifier";
const MAX_WITNESSES: usize = 64;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessVerificationRequest {
    pub challenge: VerifiedCoverageChallenge,
    pub source_head: VerifiedAuthoritySourceHead,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedWitnessEvidenceBundle {
    pub witnesses: Vec<VerifiedAuthorityHeadWitness>,
    pub trust_bindings: Vec<VerifiedWitnessTrustBinding>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ContextPolicyCandidateRequest {
    pub context_policy_digest: Digest32,
    pub context_policy_profile: String,
}

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

/// Trust classification deliberately contains no requested domain.
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
    pub legacy_projection_fail_closed: bool,
    pub leased_endpoint_available: bool,
    pub leased_endpoint_preserves_exact_horizon: bool,
    pub source_head_authenticated_here: bool,
    pub quorum_decided_here: bool,
    pub external_effects_enabled: bool,
    pub operational: bool,
}

struct QualifiedWitnessBundle {
    qualified: Vec<QualifiedWitnessEvidence>,
    lease: EvidenceLease,
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

fn lease_error(context: &str, error: impl std::fmt::Display) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{context}: {error}")))
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

fn qualify_bundle(request: WitnessVerificationRequest) -> ExternResult<QualifiedWitnessBundle> {
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
    let base_now = now_ms()?;
    let mut lease = EvidenceLease::new(
        request
            .challenge
            .verified_at_ms
            .max(request.source_head.verified_at_ms),
        request
            .challenge
            .challenge
            .expires_at_ms
            .min(request.source_head.attestation.expires_at_ms)
            .min(context.valid_until_ms),
        base_now,
    )
    .map_err(|error| lease_error("base witness evidence lease denied", error))?;

    let Some(witness_trust_policy) = context.witness_trust_policy.clone() else {
        if context.witness_trust_verifier_ref.is_some() {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "witness context has verifier without trust policy".into(),
            )));
        }
        return Ok(QualifiedWitnessBundle {
            qualified: Vec::new(),
            lease,
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

    let mut qualified_values = Vec::with_capacity(candidates.len());
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
        qualified_values.push(qualified);
    }

    let final_now = now_ms()?;
    lease.validate_at(final_now)
        .map_err(|error| lease_error("base witness evidence lease expired during composition", error))?;
    for qualified in &qualified_values {
        let witness_lease = EvidenceLease::new(
            qualified.verified_at_ms(),
            qualified.valid_until_ms(),
            final_now,
        )
        .map_err(|error| lease_error("qualified witness lease denied", error))?;
        lease = lease
            .intersect(&witness_lease, final_now)
            .map_err(|error| lease_error("witness lease intersection denied", error))?;
    }

    Ok(QualifiedWitnessBundle {
        qualified: qualified_values,
        lease,
    })
}

fn project_bundle(bundle: &QualifiedWitnessBundle) -> VerifiedWitnessEvidenceBundle {
    VerifiedWitnessEvidenceBundle {
        witnesses: bundle
            .qualified
            .iter()
            .map(QualifiedWitnessEvidence::to_verified_witness)
            .collect(),
        trust_bindings: bundle
            .qualified
            .iter()
            .map(QualifiedWitnessEvidence::to_verified_trust_binding)
            .collect(),
    }
}

/// Legacy compatibility endpoint. If a witness's signed semantic expiry exceeds
/// the true #168 proof horizon, this ABI cannot carry the tighter bound and denies.
#[hdk_extern]
pub fn verify_witness_evidence(
    request: WitnessVerificationRequest,
) -> ExternResult<VerifiedWitnessEvidenceBundle> {
    let bundle = qualify_bundle(request)?;
    for qualified in &bundle.qualified {
        if qualified.to_verified_witness().observation.expires_at_ms > qualified.valid_until_ms() {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "legacy witness projection would widen verifier validity beyond {}",
                qualified.valid_until_ms()
            ))));
        }
    }
    Ok(project_bundle(&bundle))
}

/// Lease-complete endpoint for currentness composition. The legacy-compatible
/// witness/trust receipts are paired with the exact minimum dynamic evidence lease.
#[hdk_extern]
pub fn verify_witness_evidence_leased(
    request: WitnessVerificationRequest,
) -> ExternResult<LeasedEvidence<VerifiedWitnessEvidenceBundle>> {
    let bundle = qualify_bundle(request)?;
    let now = now_ms()?;
    LeasedEvidence::new(project_bundle(&bundle), bundle.lease, now).map_err(|error| {
        lease_error(
            "cannot construct lease-complete witness evidence bundle",
            error,
        )
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
        legacy_projection_fail_closed: true,
        leased_endpoint_available: true,
        leased_endpoint_preserves_exact_horizon: true,
        source_head_authenticated_here: false,
        quorum_decided_here: false,
        external_effects_enabled: false,
        operational: false,
    })
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Workspace-only lease-complete operational policy verification composer.

use hdk::prelude::*;
use mycelix_authority_evidence_lease::{EvidenceLease, LeasedEvidence};
use mycelix_authority_freshness::AuthoritySubjectRef;
use mycelix_authority_operational_policy_leased_verifier::{
    qualify_context_policy_leased, qualify_coverage_policy_leased,
};
use mycelix_authority_operational_policy_verifier::{
    VerifiedPolicyAdoptionProof, VerifiedPolicyRecordProof,
};
use mycelix_authority_state_coverage::{
    AuthorityCoveragePolicy, VerifiedAuthorityCoveragePolicy, POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-operational-policy-leased-runtime-v0.1";
const POLICY_CANDIDATE_PROVIDER_ZOME: &str = "authority_operational_policy_candidate_provider";
const POLICY_RECORD_PROOF_VERIFIER_ZOME: &str =
    "authority_operational_policy_record_proof_verifier";
const POLICY_ADOPTION_PROOF_VERIFIER_ZOME: &str =
    "authority_operational_policy_adoption_proof_verifier";

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalPolicyCandidateBundle {
    pub coverage: VerifiedAuthorityCoveragePolicy,
    pub context: VerifiedCoverageTrustContextPolicy,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalPolicySemanticCandidates {
    pub coverage: AuthorityCoveragePolicy,
    pub coverage_record_ref: String,
    pub context: CoverageTrustContextPolicy,
    pub context_record_ref: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PolicyRecordProofVerificationRequest {
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub policy_record_ref: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PolicyAdoptionProofVerificationRequest {
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub authority_ref: String,
    pub policy_proof_ref: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalPolicyLeasedRuntimeStatus {
    pub protocol: String,
    pub policy_discovery_grants_authority: bool,
    pub record_proof_verifier_separate: bool,
    pub adoption_proof_verifier_separate: bool,
    pub leased_policy_receipts_constructed_locally: bool,
    pub generation_currentness_decided_here: bool,
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
                "{zome}::{function} unavailable; leased operational policy verification fails closed: {other:?}"
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

fn verify_record(
    policy_digest: Digest32,
    policy_profile: &str,
    policy_record_ref: &str,
) -> ExternResult<VerifiedPolicyRecordProof> {
    call_local(
        POLICY_RECORD_PROOF_VERIFIER_ZOME,
        "verify_policy_record_proof",
        PolicyRecordProofVerificationRequest {
            policy_digest,
            policy_profile: policy_profile.into(),
            policy_record_ref: policy_record_ref.into(),
        },
    )
}

fn verify_adoption(
    policy_digest: Digest32,
    policy_profile: &str,
    authority_ref: &str,
    policy_proof_ref: &str,
) -> ExternResult<VerifiedPolicyAdoptionProof> {
    call_local(
        POLICY_ADOPTION_PROOF_VERIFIER_ZOME,
        "verify_policy_adoption_proof",
        PolicyAdoptionProofVerificationRequest {
            policy_digest,
            policy_profile: policy_profile.into(),
            authority_ref: authority_ref.into(),
            policy_proof_ref: policy_proof_ref.into(),
        },
    )
}

#[hdk_extern]
pub fn resolve_operational_policy_candidates_leased(
    subject: AuthoritySubjectRef,
) -> ExternResult<LeasedEvidence<OperationalPolicyCandidateBundle>> {
    subject.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid leased operational policy target subject: {error}"
        )))
    })?;
    let candidates: OperationalPolicySemanticCandidates = call_local(
        POLICY_CANDIDATE_PROVIDER_ZOME,
        "resolve_operational_policy_semantics",
        subject.clone(),
    )?;
    candidates.coverage.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid leased operational coverage-policy candidate: {error}"
        )))
    })?;
    candidates.context.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid leased operational context-policy candidate: {error}"
        )))
    })?;

    let coverage_digest = candidates.coverage.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute leased operational coverage-policy identity: {error}"
        )))
    })?;
    if candidates.coverage.namespace != subject.namespace
        || !candidates
            .coverage
            .allowed_subject_kinds
            .iter()
            .any(|kind| kind == &subject.kind)
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "leased operational coverage policy does not cover target subject".into(),
        )));
    }
    if candidates.context.coverage_policy.digest != coverage_digest
        || candidates.context.coverage_policy.profile != POLICY_IDENTITY_PROFILE
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "leased context policy does not bind exact coverage policy".into(),
        )));
    }

    let coverage_record = verify_record(
        coverage_digest,
        POLICY_IDENTITY_PROFILE,
        &candidates.coverage_record_ref,
    )?;
    let coverage_adoption = verify_adoption(
        coverage_digest,
        POLICY_IDENTITY_PROFILE,
        &candidates.coverage.authority_ref,
        &candidates.coverage.policy_proof_ref,
    )?;
    let coverage_now = now_ms()?;
    let qualified_coverage = qualify_coverage_policy_leased(
        &candidates.coverage,
        &candidates.coverage_record_ref,
        &coverage_record,
        &coverage_adoption,
        coverage_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "leased operational coverage-policy qualification denied: {error}"
        )))
    })?;

    let context_digest = candidates.context.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute leased operational context-policy identity: {error}"
        )))
    })?;
    let context_record = verify_record(
        context_digest,
        CONTEXT_POLICY_PROFILE,
        &candidates.context_record_ref,
    )?;
    let context_adoption = verify_adoption(
        context_digest,
        CONTEXT_POLICY_PROFILE,
        &candidates.context.authority_ref,
        &candidates.context.policy_proof_ref,
    )?;
    let context_now = now_ms()?;
    let qualified_context = qualify_context_policy_leased(
        &candidates.context,
        &candidates.context_record_ref,
        &context_record,
        &context_adoption,
        context_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "leased operational context-policy qualification denied: {error}"
        )))
    })?;

    let final_now = now_ms()?;
    let lease = qualified_coverage
        .lease()
        .intersect(qualified_context.lease(), final_now)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "operational policy lease intersection denied: {error}"
            )))
        })?;
    if qualified_coverage.policy_digest() != coverage_digest
        || qualified_context.policy_digest() != context_digest
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "leased qualified policy identity changed during composition".into(),
        )));
    }
    LeasedEvidence::new(
        OperationalPolicyCandidateBundle {
            coverage: qualified_coverage.to_verified_policy(),
            context: qualified_context.to_verified_policy(),
        },
        lease,
        final_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot construct leased operational policy bundle: {error}"
        )))
    })
}

#[hdk_extern]
pub fn operational_policy_leased_runtime_status(
    _: (),
) -> ExternResult<OperationalPolicyLeasedRuntimeStatus> {
    Ok(OperationalPolicyLeasedRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        policy_discovery_grants_authority: false,
        record_proof_verifier_separate: true,
        adoption_proof_verifier_separate: true,
        leased_policy_receipts_constructed_locally: true,
        generation_currentness_decided_here: false,
        external_effects_enabled: false,
        operational: false,
    })
}

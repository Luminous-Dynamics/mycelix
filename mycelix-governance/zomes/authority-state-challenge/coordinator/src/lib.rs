// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed authority-state freshness challenge issuer.
//!
//! Issuance is unavailable unless an independent context-policy verifier resolves
//! the exact current subject/context and designates this agent as challenge issuer.
//! Raw entropy remains private on the issuer's source chain. Verification later
//! re-resolves current context and rechecks the exact private entropy record.

use authority_state_challenge_integrity::{
    entropy_nonce_digest, AuthorityStateChallengeRecord, ChallengeEntropyRecord, EntryTypes,
    CHALLENGE_RUNTIME_PROTOCOL,
};
use hdk::prelude::*;
use mycelix_authority_freshness::AuthoritySubjectRef;
use mycelix_authority_state_coverage::POLICY_IDENTITY_PROFILE;
use mycelix_authority_state_coverage_context::{
    CoverageChallenge, VerifiedCoverageChallenge, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const CONTEXT_PROVIDER_ZOME: &str = "authority_state_context_policy_verifier";
const CONTEXT_PROVIDER_FUNCTION: &str = "resolve_challenge_context";
const CONTEXT_PROVIDER_PROTOCOL: &str = "mycelix-authority-state-challenge-context-v0.1";
const ENTROPY_BYTES: u32 = 32;
const MAX_REF_BYTES: usize = 2048;
const MAX_CHALLENGE_LIFETIME_MS: u64 = 60_000;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IssueAuthorityStateChallengeRequest {
    pub subject: AuthoritySubjectRef,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedChallengeContext {
    protocol: String,
    subject: AuthoritySubjectRef,
    context_policy_digest: Digest32,
    context_policy_profile: String,
    coverage_policy_digest: Digest32,
    coverage_policy_profile: String,
    max_challenge_lifetime_ms: u64,
    context_valid_until_ms: u64,
    challenge_issuer_did: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IssuedAuthorityStateChallenge {
    pub challenge_action: ActionHash,
    pub entropy_action: ActionHash,
    pub challenge: CoverageChallenge,
    pub context_verification_ref: String,
    pub issued_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ChallengeRuntimeStatus {
    pub protocol: String,
    pub private_entropy: bool,
    pub caller_nonce_authority: bool,
    pub context_provider_required: String,
    pub context_provider_reachable: bool,
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
                "{zome}::{function} unavailable; challenge issuance fails closed: {other:?}"
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
    timestamp_ms(sys_time()?, "current time")
}

fn current_issuer_did() -> ExternResult<String> {
    let info = agent_info()?;
    Ok(format!("did:mycelix:{}", info.agent_initial_pubkey))
}

fn resolve_context(subject: &AuthoritySubjectRef, now: u64) -> ExternResult<VerifiedChallengeContext> {
    subject.validate().map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "challenge subject is malformed".into(),
        ))
    })?;
    let context: VerifiedChallengeContext = call_local(
        CONTEXT_PROVIDER_ZOME,
        CONTEXT_PROVIDER_FUNCTION,
        subject.clone(),
    )?;
    if context.protocol != CONTEXT_PROVIDER_PROTOCOL
        || &context.subject != subject
        || context.context_policy_digest.is_zero()
        || context.coverage_policy_digest.is_zero()
        || context.context_policy_profile != CONTEXT_POLICY_PROFILE
        || context.coverage_policy_profile != POLICY_IDENTITY_PROFILE
        || context.max_challenge_lifetime_ms == 0
        || context.max_challenge_lifetime_ms > MAX_CHALLENGE_LIFETIME_MS
        || context.context_valid_until_ms <= now
        || context.valid_until_ms <= now
        || context.verified_at_ms == 0
        || context.verified_at_ms > now
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "challenge context provider returned stale or inexact authority".into(),
        )));
    }
    require_ref(&context.challenge_issuer_did, "challenge issuer")?;
    require_ref(&context.verification_ref, "context verification ref")?;
    if context.challenge_issuer_did != current_issuer_did()? {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "this agent is not the current institution-authorized challenge issuer".into(),
        )));
    }
    Ok(context)
}

#[hdk_extern]
pub fn issue_authority_state_challenge(
    request: IssueAuthorityStateChallengeRequest,
) -> ExternResult<IssuedAuthorityStateChallenge> {
    let before_entropy_ms = now_ms()?;
    let context = resolve_context(&request.subject, before_entropy_ms)?;

    // Entropy is generated inside the coordinator by the Holochain host. Caller
    // bytes never enter the nonce path.
    let entropy = random_bytes(ENTROPY_BYTES)?.into_vec();
    let nonce_digest = entropy_nonce_digest(&entropy);
    let issuer_did = current_issuer_did()?;

    let entropy_entry = ChallengeEntropyRecord {
        protocol_version: CHALLENGE_RUNTIME_PROTOCOL.into(),
        subject: request.subject.clone(),
        context_policy_digest: context.context_policy_digest,
        context_policy_profile: context.context_policy_profile.clone(),
        coverage_policy_digest: context.coverage_policy_digest,
        coverage_policy_profile: context.coverage_policy_profile.clone(),
        entropy,
        nonce_digest,
        issued_by: issuer_did.clone(),
    };
    entropy_entry.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot create challenge entropy: {error}"
        )))
    })?;
    let entropy_action = create_entry(EntryTypes::ChallengeEntropy(entropy_entry))?;

    // Read the actual committed action timestamp rather than predicting the
    // timestamp Holochain will assign to the private write.
    let entropy_record = get(entropy_action.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "new private challenge entropy record is unavailable locally".into(),
        )))?;
    let issued_at_ms = timestamp_ms(entropy_record.action().timestamp(), "entropy action")?;

    let semantic_ceiling = context.context_valid_until_ms.min(context.valid_until_ms);
    let requested_expiry = issued_at_ms
        .checked_add(context.max_challenge_lifetime_ms)
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "challenge lifetime overflow".into(),
        )))?;
    let expires_at_ms = requested_expiry.min(semantic_ceiling);
    if expires_at_ms <= issued_at_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "challenge context expired during entropy issuance".into(),
        )));
    }

    let challenge = CoverageChallenge {
        protocol_version: mycelix_authority_state_coverage_context::PROTOCOL_VERSION.into(),
        context_policy_digest: context.context_policy_digest,
        context_policy_profile: context.context_policy_profile,
        coverage_policy_digest: context.coverage_policy_digest,
        coverage_policy_profile: context.coverage_policy_profile,
        subject: request.subject,
        nonce_digest,
        randomness_proof_ref: entropy_action.to_string(),
        issued_at_ms,
        expires_at_ms,
        challenge_issuer_ref: issuer_did,
    };
    challenge.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "constructed challenge is invalid: {error}"
        )))
    })?;

    let public_record = AuthorityStateChallengeRecord {
        protocol_version: CHALLENGE_RUNTIME_PROTOCOL.into(),
        challenge: challenge.clone(),
        entropy_action: entropy_action.clone(),
        context_verification_ref: context.verification_ref.clone(),
    };
    public_record.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "constructed challenge record is invalid: {error}"
        )))
    })?;
    let challenge_action = create_entry(EntryTypes::AuthorityStateChallenge(public_record))?;

    Ok(IssuedAuthorityStateChallenge {
        challenge_action,
        entropy_action,
        challenge,
        context_verification_ref: context.verification_ref,
        issued_at_ms,
    })
}

#[hdk_extern]
pub fn verify_issued_authority_state_challenge(
    challenge_action: ActionHash,
) -> ExternResult<VerifiedCoverageChallenge> {
    let now = now_ms()?;
    let record = get(challenge_action.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "challenge record does not exist".into(),
        )))?;
    let challenge_record: AuthorityStateChallengeRecord = record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "challenge action does not contain an authority-state challenge".into(),
        )))?;
    challenge_record.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "stored challenge record is invalid: {error}"
        )))
    })?;

    let challenge_author = create_author(record.action())?;
    let current_agent = agent_info()?.agent_initial_pubkey;
    if challenge_author != current_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "only the original challenge issuer can verify its private entropy proof".into(),
        )));
    }
    let issuer_did = format!("did:mycelix:{current_agent}");
    if challenge_record.challenge.challenge_issuer_ref != issuer_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "challenge issuer no longer matches the author-bound proof".into(),
        )));
    }
    if now >= challenge_record.challenge.expires_at_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "challenge has expired".into(),
        )));
    }

    let entropy_record = get(
        challenge_record.entropy_action.clone(),
        GetOptions::default(),
    )?
    .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
        "private entropy proof is unavailable on the issuer source chain".into(),
    )))?;
    let entropy_author = create_author(entropy_record.action())?;
    if entropy_author != current_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "private entropy proof was authored by another agent".into(),
        )));
    }
    let entropy: ChallengeEntropyRecord = entropy_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "entropy action does not contain the private challenge entropy entry".into(),
        )))?;
    entropy.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "stored private challenge entropy is invalid: {error}"
        )))
    })?;

    let challenge = &challenge_record.challenge;
    if challenge.randomness_proof_ref != challenge_record.entropy_action.to_string()
        || entropy.nonce_digest != challenge.nonce_digest
        || entropy_nonce_digest(&entropy.entropy) != challenge.nonce_digest
        || entropy.subject != challenge.subject
        || entropy.context_policy_digest != challenge.context_policy_digest
        || entropy.context_policy_profile != challenge.context_policy_profile
        || entropy.coverage_policy_digest != challenge.coverage_policy_digest
        || entropy.coverage_policy_profile != challenge.coverage_policy_profile
        || entropy.issued_by != challenge.challenge_issuer_ref
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "private entropy proof does not exactly bind the public challenge".into(),
        )));
    }

    // Current verification re-resolves the authority context. A challenge issued
    // before policy/source rotation cannot remain current merely because its
    // immutable entropy proof is still valid.
    let context = resolve_context(&challenge.subject, now)?;
    if context.context_policy_digest != challenge.context_policy_digest
        || context.context_policy_profile != challenge.context_policy_profile
        || context.coverage_policy_digest != challenge.coverage_policy_digest
        || context.coverage_policy_profile != challenge.coverage_policy_profile
        || context.challenge_issuer_did != challenge.challenge_issuer_ref
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "challenge policy context is no longer current".into(),
        )));
    }

    Ok(VerifiedCoverageChallenge {
        challenge: challenge.clone(),
        verified_nonce_digest: challenge.nonce_digest,
        verified_randomness_proof_ref: challenge.randomness_proof_ref.clone(),
        verified_challenge_issuer_ref: challenge.challenge_issuer_ref.clone(),
        verification_ref: format!("challenge-verification:{challenge_action}"),
        verified_at_ms: now,
    })
}

#[hdk_extern]
pub fn challenge_runtime_status(_: ()) -> ExternResult<ChallengeRuntimeStatus> {
    let reachable = call_local::<_, VerifiedChallengeContext>(
        CONTEXT_PROVIDER_ZOME,
        CONTEXT_PROVIDER_FUNCTION,
        AuthoritySubjectRef {
            // Deliberately invalid sentinel: success is not expected and must not
            // be interpreted as provider authority. We report operational=false
            // until a real subject-specific provider check occurs during issue.
            kind: mycelix_authority_freshness::AuthoritySubjectKind::AuthorityGrant,
            namespace: "status-probe".into(),
            subject_id: "status-probe".into(),
            identity: mycelix_authority_freshness::ProfiledDigest {
                digest: Digest32::ZERO,
                profile: "status-probe".into(),
            },
        },
    )
    .is_ok();
    Ok(ChallengeRuntimeStatus {
        protocol: CHALLENGE_RUNTIME_PROTOCOL.into(),
        private_entropy: true,
        caller_nonce_authority: false,
        context_provider_required: CONTEXT_PROVIDER_ZOME.into(),
        context_provider_reachable: reachable,
        // A generic reachability probe is never enough to establish operational
        // authority. Only subject-specific issuance can prove that.
        operational: false,
    })
}

fn create_author(action: &Action) -> ExternResult<AgentPubKey> {
    match action {
        Action::Create(create) => Ok(create.author.clone()),
        _ => Err(wasm_error!(WasmErrorInner::Guest(
            "authority-state challenge proof must reference a create action".into(),
        ))),
    }
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> ExternResult<u64> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} must be positive"
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

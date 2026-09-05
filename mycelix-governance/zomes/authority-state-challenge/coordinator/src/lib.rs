// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Non-authoritative authority-state freshness probe issuer.
//!
//! A probe collects fresh randomness-bound evidence. It does not decide whether
//! the candidate coverage/context policies are current and does not require the
//! operational freshness plane to authorize the very probe used to discover
//! freshness. Authority is established only later by constitution/root-qualified
//! coverage and current-state projection.

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
use serde::{Deserialize, Serialize};

const ENTROPY_BYTES: u32 = 32;
const MAX_PROBE_LIFETIME_MS: u64 = 60_000;

/// Request for a read-only evidence probe.
///
/// The caller selects only which exact candidate policy identities the probe
/// should be bound to and a bounded lifetime. These values are not accepted as
/// proof that either policy is current. Later bootstrap/operational qualification
/// must independently prove those exact identities are authoritative.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IssueAuthorityStateProbeRequest {
    pub subject: AuthoritySubjectRef,
    pub context_policy_digest: Digest32,
    pub coverage_policy_digest: Digest32,
    pub requested_lifetime_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IssuedAuthorityStateProbe {
    pub probe_action: ActionHash,
    pub entropy_action: ActionHash,
    pub challenge: CoverageChallenge,
    pub issued_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ProbeRuntimeStatus {
    pub protocol: String,
    pub private_entropy: bool,
    pub caller_nonce_authority: bool,
    pub candidate_policy_selection: bool,
    pub candidate_policy_selection_grants_authority: bool,
    pub probe_grants_authority: bool,
    pub operational: bool,
}

fn now_ms() -> ExternResult<u64> {
    timestamp_ms(sys_time()?, "current time")
}

fn current_issuer_did() -> ExternResult<String> {
    let info = agent_info()?;
    Ok(format!("did:mycelix:{}", info.agent_initial_pubkey))
}

fn validate_probe_request(request: &IssueAuthorityStateProbeRequest) -> ExternResult<()> {
    request.subject.validate().map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "authority-state probe subject is malformed".into(),
        ))
    })?;
    if request.context_policy_digest.is_zero() || request.coverage_policy_digest.is_zero() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "probe candidate policy digests must be non-zero".into(),
        )));
    }
    if request.requested_lifetime_ms == 0
        || request.requested_lifetime_ms > MAX_PROBE_LIFETIME_MS
    {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "probe lifetime must be 1-{MAX_PROBE_LIFETIME_MS} ms"
        ))));
    }
    Ok(())
}

/// Generate one fresh read-only probe bound to exact candidate policy identities.
///
/// This endpoint intentionally performs no current-policy or issuer-grant lookup.
/// The resulting `CoverageChallenge` is evidence-shaped provenance only. A later
/// qualifier must prove its exact policy identities under the current bootstrap or
/// operational authority root before the challenge can contribute to freshness.
#[hdk_extern]
pub fn issue_authority_state_probe(
    request: IssueAuthorityStateProbeRequest,
) -> ExternResult<IssuedAuthorityStateProbe> {
    validate_probe_request(&request)?;

    // Entropy is generated inside the coordinator by the Holochain host. Caller
    // bytes never enter the nonce path.
    let entropy = random_bytes(ENTROPY_BYTES)?.into_vec();
    let nonce_digest = entropy_nonce_digest(&entropy);
    let issuer_did = current_issuer_did()?;

    let entropy_entry = ChallengeEntropyRecord {
        protocol_version: CHALLENGE_RUNTIME_PROTOCOL.into(),
        subject: request.subject.clone(),
        context_policy_digest: request.context_policy_digest,
        context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
        coverage_policy_digest: request.coverage_policy_digest,
        coverage_policy_profile: POLICY_IDENTITY_PROFILE.into(),
        entropy,
        nonce_digest,
        issued_by: issuer_did.clone(),
    };
    entropy_entry.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot create authority-state probe entropy: {error}"
        )))
    })?;
    let entropy_action = create_entry(EntryTypes::ChallengeEntropy(entropy_entry))?;

    // Private entry content is recovered strictly from this agent's own source
    // chain, where Holochain permits local private-entry queries.
    let entropy_record = get_local_record(&entropy_action)?;
    let issued_at_ms = timestamp_ms(entropy_record.action().timestamp(), "probe entropy action")?;
    let expires_at_ms = issued_at_ms
        .checked_add(request.requested_lifetime_ms)
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "probe lifetime overflow".into(),
        )))?;

    let challenge = CoverageChallenge {
        protocol_version: mycelix_authority_state_coverage_context::PROTOCOL_VERSION.into(),
        context_policy_digest: request.context_policy_digest,
        context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
        coverage_policy_digest: request.coverage_policy_digest,
        coverage_policy_profile: POLICY_IDENTITY_PROFILE.into(),
        subject: request.subject,
        nonce_digest,
        randomness_proof_ref: entropy_action.to_string(),
        issued_at_ms,
        expires_at_ms,
        // Provenance only. #96 later requires this to match the verified private
        // entropy proof; it is not treated as an institutional authority grant.
        challenge_issuer_ref: issuer_did,
    };
    challenge.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "constructed authority-state probe is invalid: {error}"
        )))
    })?;

    let public_record = AuthorityStateChallengeRecord {
        protocol_version: CHALLENGE_RUNTIME_PROTOCOL.into(),
        challenge: challenge.clone(),
        entropy_action: entropy_action.clone(),
    };
    public_record.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "constructed authority-state probe record is invalid: {error}"
        )))
    })?;
    let probe_action = create_entry(EntryTypes::AuthorityStateChallenge(public_record))?;

    Ok(IssuedAuthorityStateProbe {
        probe_action,
        entropy_action,
        challenge,
        issued_at_ms,
    })
}

/// Verify only the randomness/provenance properties of one locally issued probe.
///
/// A positive `VerifiedCoverageChallenge` here is deliberately **not** proof that
/// the candidate policy identities are current. That authority decision belongs
/// to #96 plus the constitution-rooted/operational currentness layer.
#[hdk_extern]
pub fn verify_issued_authority_state_probe(
    probe_action: ActionHash,
) -> ExternResult<VerifiedCoverageChallenge> {
    let now = now_ms()?;
    let record = get(probe_action.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "authority-state probe record does not exist".into(),
        )))?;
    let probe_record: AuthorityStateChallengeRecord = record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "probe action does not contain an authority-state probe".into(),
        )))?;
    probe_record.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "stored authority-state probe is invalid: {error}"
        )))
    })?;

    let probe_author = create_author(record.action())?;
    let current_agent = agent_info()?.agent_initial_pubkey;
    if probe_author != current_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "only the original probe author can verify its private entropy proof".into(),
        )));
    }
    let issuer_did = format!("did:mycelix:{current_agent}");
    if probe_record.challenge.challenge_issuer_ref != issuer_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "probe provenance no longer matches the author-bound proof".into(),
        )));
    }
    if now >= probe_record.challenge.expires_at_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "authority-state probe has expired".into(),
        )));
    }

    let entropy_record = get_local_record(&probe_record.entropy_action)?;
    let entropy_author = create_author(entropy_record.action())?;
    if entropy_author != current_agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "private probe entropy was authored by another agent".into(),
        )));
    }
    let entropy: ChallengeEntropyRecord = entropy_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "entropy action does not contain private authority-state probe entropy".into(),
        )))?;
    entropy.validate_structure().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "stored private authority-state probe entropy is invalid: {error}"
        )))
    })?;

    let challenge = &probe_record.challenge;
    if challenge.randomness_proof_ref != probe_record.entropy_action.to_string()
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
            "private entropy proof does not exactly bind the public probe".into(),
        )));
    }

    Ok(VerifiedCoverageChallenge {
        challenge: challenge.clone(),
        verified_nonce_digest: challenge.nonce_digest,
        verified_randomness_proof_ref: challenge.randomness_proof_ref.clone(),
        verified_challenge_issuer_ref: challenge.challenge_issuer_ref.clone(),
        verification_ref: format!("probe-verification:{probe_action}"),
        verified_at_ms: now,
    })
}

#[hdk_extern]
pub fn probe_runtime_status(_: ()) -> ExternResult<ProbeRuntimeStatus> {
    Ok(ProbeRuntimeStatus {
        protocol: CHALLENGE_RUNTIME_PROTOCOL.into(),
        private_entropy: true,
        caller_nonce_authority: false,
        candidate_policy_selection: true,
        candidate_policy_selection_grants_authority: false,
        probe_grants_authority: false,
        // Still unprovisioned in the binding governance DNA.
        operational: false,
    })
}

fn get_local_record(action_hash: &ActionHash) -> ExternResult<Record> {
    query(ChainQueryFilter::new().include_entries(true))?
        .into_iter()
        .find(|record| record.action_address() == action_hash)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "local source-chain record is unavailable: {action_hash}"
            )))
        })
}

fn create_author(action: &Action) -> ExternResult<AgentPubKey> {
    match action {
        Action::Create(create) => Ok(create.author.clone()),
        _ => Err(wasm_error!(WasmErrorInner::Guest(
            "authority-state probe proof must reference a create action".into(),
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

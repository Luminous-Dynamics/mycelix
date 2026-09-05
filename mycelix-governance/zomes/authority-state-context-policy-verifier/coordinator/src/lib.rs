// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed Holochain adapter for current authority-state challenge context.
//!
//! This coordinator owns no policy registry and no current-state truth. It calls
//! explicit independently qualified providers, requests one exact freshness set,
//! then re-runs the pure challenge-context qualifier before returning wire data.

use hdk::prelude::*;
use mycelix_authority_challenge_context_verifier::{
    qualify_challenge_context, QualifiedChallengeContext, VerifiedChallengeIssuerGrant,
    VerifiedWitnessTrustPolicyAuthority, PROTOCOL_VERSION as CONTEXT_PROTOCOL,
};
use mycelix_authority_freshness::{
    AuthoritySubjectKind, AuthoritySubjectRef, ProfiledDigest, VerifiedAuthorityFreshness,
};
use mycelix_authority_state_coverage::{
    VerifiedAuthorityCoveragePolicy, POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    VerifiedCoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const CONTEXT_POLICY_PROVIDER_ZOME: &str = "authority_coverage_context_verifier";
const CONTEXT_POLICY_PROVIDER_FN: &str = "resolve_context_policy";
const COVERAGE_POLICY_PROVIDER_ZOME: &str = "authority_coverage_policy_verifier";
const COVERAGE_POLICY_PROVIDER_FN: &str = "resolve_coverage_policy";
const WITNESS_POLICY_PROVIDER_ZOME: &str = "authority_witness_trust_policy_verifier";
const WITNESS_POLICY_PROVIDER_FN: &str = "resolve_witness_trust_policy";
const ISSUER_GRANT_PROVIDER_ZOME: &str = "authority_challenge_issuer_grant_verifier";
const ISSUER_GRANT_PROVIDER_FN: &str = "resolve_challenge_issuer_grant";
const FRESHNESS_PROVIDER_ZOME: &str = "authority_current_freshness_verifier";
const FRESHNESS_PROVIDER_FN: &str = "resolve_current_freshness";

#[derive(Serialize, Deserialize, Debug, Clone)]
struct FreshnessRequest {
    subjects: Vec<AuthoritySubjectRef>,
}

/// Wire shape consumed by PR #99. It is deliberately smaller than the pure
/// qualified object; all fields are projected only after pure qualification.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedChallengeContextWire {
    pub protocol: String,
    pub subject: AuthoritySubjectRef,
    pub context_policy_digest: Digest32,
    pub context_policy_profile: String,
    pub coverage_policy_digest: Digest32,
    pub coverage_policy_profile: String,
    pub max_challenge_lifetime_ms: u64,
    pub context_valid_until_ms: u64,
    pub challenge_issuer_did: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ContextVerifierRuntimeStatus {
    pub protocol: String,
    pub context_policy_provider: String,
    pub coverage_policy_provider: String,
    pub witness_policy_provider: String,
    pub issuer_grant_provider: String,
    pub freshness_provider: String,
    pub providers_probed_without_subject: bool,
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
                "{zome}::{function} unavailable; challenge context fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot decode {zome}::{function} response: {error}"
        )))
    })
}

fn call_required<I, O>(zome: &str, function: &str, input: I) -> ExternResult<O>
where
    I: Serialize,
    O: DeserializeOwned,
{
    call_local::<_, Option<O>>(zome, function, input)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "{zome}::{function} returned no current authority"
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

fn policy_subject(
    kind: AuthoritySubjectKind,
    namespace: &str,
    subject_id: &str,
    identity: ProfiledDigest,
) -> ExternResult<AuthoritySubjectRef> {
    let subject = AuthoritySubjectRef {
        kind,
        namespace: namespace.into(),
        subject_id: subject_id.into(),
        identity,
    };
    subject.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot derive policy freshness subject: {error}"
        )))
    })?;
    Ok(subject)
}

fn witness_policy_subject_id(policy: &ProfiledDigest) -> String {
    format!("witness-trust-policy:{}", hex_digest(policy.digest))
}

fn exact_required_freshness_subjects(
    context: &VerifiedCoverageTrustContextPolicy,
    coverage: &VerifiedAuthorityCoveragePolicy,
    witness: Option<&VerifiedWitnessTrustPolicyAuthority>,
    issuer: &VerifiedChallengeIssuerGrant,
) -> ExternResult<Vec<AuthoritySubjectRef>> {
    let context_digest = context.policy.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot digest context policy: {error}"
        )))
    })?;
    let coverage_digest = coverage.policy.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot digest coverage policy: {error}"
        )))
    })?;

    let mut subjects = vec![
        policy_subject(
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            &coverage.policy.namespace,
            &coverage.policy.policy_id,
            ProfiledDigest {
                digest: coverage_digest,
                profile: POLICY_IDENTITY_PROFILE.into(),
            },
        )?,
        policy_subject(
            AuthoritySubjectKind::CoverageTrustContextPolicy,
            &context.policy.institution_ref,
            &context.policy.context_policy_id,
            ProfiledDigest {
                digest: context_digest,
                profile: CONTEXT_POLICY_PROFILE.into(),
            },
        )?,
        issuer.grant_subject.clone(),
    ];

    if let Some(witness) = witness {
        subjects.push(policy_subject(
            AuthoritySubjectKind::WitnessTrustPolicy,
            &context.policy.institution_ref,
            &witness_policy_subject_id(&witness.policy),
            witness.policy.clone(),
        )?);
    }
    Ok(subjects)
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

fn project_wire(qualified: &QualifiedChallengeContext) -> VerifiedChallengeContextWire {
    VerifiedChallengeContextWire {
        protocol: qualified.protocol().into(),
        subject: qualified.subject().clone(),
        context_policy_digest: qualified.context_policy_digest(),
        context_policy_profile: qualified.context_policy_profile().into(),
        coverage_policy_digest: qualified.coverage_policy_digest(),
        coverage_policy_profile: qualified.coverage_policy_profile().into(),
        max_challenge_lifetime_ms: qualified.max_challenge_lifetime_ms(),
        context_valid_until_ms: qualified.context_valid_until_ms(),
        challenge_issuer_did: qualified.challenge_issuer_did().into(),
        verification_ref: qualified.verification_ref().into(),
        verified_at_ms: qualified.verified_at_ms(),
        valid_until_ms: qualified.valid_until_ms(),
    }
}

/// Resolve one exact current challenge context for one exact subject.
///
/// Every provider is required and fail-closed. The freshness provider receives a
/// deterministic preliminary subject list, but does not become authoritative for
/// that list: `qualify_challenge_context` reconstructs the exact required set and
/// rejects missing, extra, ambiguous, inactive, or stale receipts.
#[hdk_extern]
pub fn resolve_challenge_context(
    subject: AuthoritySubjectRef,
) -> ExternResult<VerifiedChallengeContextWire> {
    subject.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid challenge subject: {error}"
        )))
    })?;
    let now = now_ms()?;

    let context: VerifiedCoverageTrustContextPolicy = call_required(
        CONTEXT_POLICY_PROVIDER_ZOME,
        CONTEXT_POLICY_PROVIDER_FN,
        subject.clone(),
    )?;
    let coverage: VerifiedAuthorityCoveragePolicy = call_required(
        COVERAGE_POLICY_PROVIDER_ZOME,
        COVERAGE_POLICY_PROVIDER_FN,
        subject.clone(),
    )?;
    let issuer: VerifiedChallengeIssuerGrant = call_required(
        ISSUER_GRANT_PROVIDER_ZOME,
        ISSUER_GRANT_PROVIDER_FN,
        subject.clone(),
    )?;

    let witness = if context.policy.witness_trust_policy.is_some() {
        Some(call_required::<_, VerifiedWitnessTrustPolicyAuthority>(
            WITNESS_POLICY_PROVIDER_ZOME,
            WITNESS_POLICY_PROVIDER_FN,
            subject.clone(),
        )?)
    } else {
        None
    };

    let required = exact_required_freshness_subjects(
        &context,
        &coverage,
        witness.as_ref(),
        &issuer,
    )?;
    let freshness: Vec<VerifiedAuthorityFreshness> = call_local(
        FRESHNESS_PROVIDER_ZOME,
        FRESHNESS_PROVIDER_FN,
        FreshnessRequest { subjects: required },
    )?;

    let qualified = qualify_challenge_context(
        &subject,
        &context,
        &coverage,
        witness.as_ref(),
        &issuer,
        &freshness,
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "current challenge context denied: {error}"
        )))
    })?;

    Ok(project_wire(&qualified))
}

/// Status never probes providers using synthetic input. Code presence is not
/// operational authority; real subject-specific qualification is the only signal.
#[hdk_extern]
pub fn context_verifier_runtime_status(_: ()) -> ExternResult<ContextVerifierRuntimeStatus> {
    Ok(ContextVerifierRuntimeStatus {
        protocol: CONTEXT_PROTOCOL.into(),
        context_policy_provider: format!("{CONTEXT_POLICY_PROVIDER_ZOME}::{CONTEXT_POLICY_PROVIDER_FN}"),
        coverage_policy_provider: format!("{COVERAGE_POLICY_PROVIDER_ZOME}::{COVERAGE_POLICY_PROVIDER_FN}"),
        witness_policy_provider: format!("{WITNESS_POLICY_PROVIDER_ZOME}::{WITNESS_POLICY_PROVIDER_FN}"),
        issuer_grant_provider: format!("{ISSUER_GRANT_PROVIDER_ZOME}::{ISSUER_GRANT_PROVIDER_FN}"),
        freshness_provider: format!("{FRESHNESS_PROVIDER_ZOME}::{FRESHNESS_PROVIDER_FN}"),
        providers_probed_without_subject: false,
        operational: false,
    })
}

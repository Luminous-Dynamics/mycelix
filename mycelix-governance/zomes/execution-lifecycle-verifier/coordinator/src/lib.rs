// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed composition verifier for the append-only governance execution lifecycle.
//!
//! This zome does not mint policy, choose an executor, create threshold authority,
//! or decide whether an external adapter is safe. It composes exact positive
//! receipts from independently authoritative provider boundaries and denies when
//! any dependency is absent, stale, malformed, or inexact.

use execution_lifecycle_integrity::LifecycleEventCandidate;
use hdk::prelude::*;
use mycelix_governance_constitution::{Digest32 as ConstitutionDigest32, STATEMENT_PROFILE};
use mycelix_governance_execution_lifecycle::{
    ExecutionDomain, LifecycleEvent, LifecycleEventKind, ProfiledDigest, TerminalFailureKind,
    PROTOCOL_VERSION as LIFECYCLE_PROTOCOL_VERSION,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::Serialize;

const DOMAIN_PROTOCOL: &str = "mycelix-governance-execution-domain-verifier-v0.1";
const EVENT_PROTOCOL: &str = "mycelix-governance-execution-event-verifier-v0.1";
const EFFECT_SAFETY_PROTOCOL: &str = "mycelix-governance-effect-safety-verifier-v0.1";
const PLAN_PROTOCOL: &str = "mycelix-governance-execution-plan-verifier-v0.1";
const PREFLIGHT_PROTOCOL: &str = "mycelix-governance-execution-preflight-runtime-v0.1";
const THRESHOLD_PROTOCOL: &str = "mycelix-governance-execution-threshold-verifier-v0.1";
const EXECUTOR_PROTOCOL: &str = "mycelix-governance-executor-authority-verifier-v0.1";
const SAFETY_PROVIDER_PROTOCOL: &str = "mycelix-governance-effect-safety-policy-v0.1";
const OUTCOME_PROTOCOL: &str = "mycelix-governance-execution-outcome-verifier-v0.1";
const CANCELLATION_PROTOCOL: &str = "mycelix-governance-execution-cancellation-verifier-v0.1";

const PLAN_VERIFIER_ZOME: &str = "governance_execution_plan_verifier";
const PREFLIGHT_ZOME: &str = "governance_execution_preflight";
const THRESHOLD_VERIFIER_ZOME: &str = "governance_threshold_authority_verifier";
const EXECUTOR_VERIFIER_ZOME: &str = "governance_executor_authority_verifier";
const SAFETY_PROVIDER_ZOME: &str = "governance_effect_safety_authority_verifier";
const OUTCOME_VERIFIER_ZOME: &str = "governance_execution_outcome_verifier";
const CANCELLATION_VERIFIER_ZOME: &str = "governance_execution_review_authority_verifier";

const MAX_REF_BYTES: usize = 2048;
const MAX_ACTION_BYTES: usize = 4096;

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedExecutionPlan {
    protocol: String,
    proposal_id: String,
    /// Immutable plan identity. In lifecycle v0.1 this is placed in the legacy-
    /// named `ExecutionDomain.timelock_ref` slot; it must never be derived from
    /// mutable Timelock status/update state.
    plan_ref: String,
    actions: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyExecutionPreflightInput {
    proposal_id: String,
    actions: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedExecutionPreflightMirror {
    protocol: String,
    proposal_id: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    proposal_authority_binding: ActionHash,
    constitution_statement_digest: ConstitutionDigest32,
    constitution_version: u64,
    binding_tally_action: ActionHash,
    checked_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyExecutionThresholdRequest {
    proposal_id: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    proposal_authority_binding: ActionHash,
    constitution_statement_digest: ConstitutionDigest32,
    binding_tally_action: ActionHash,
    checked_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedExecutionThresholdAuthorization {
    protocol: String,
    proposal_id: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    proposal_authority_binding: ActionHash,
    constitution_statement_digest: ConstitutionDigest32,
    binding_tally_action: ActionHash,
    threshold_authorization_ref: String,
    committee_ref: String,
    signing_policy_digest: Digest32,
    signing_policy_profile: String,
    key_epoch: u64,
    valid_until_ms: u64,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ResolveExecutorDesignationRequest {
    proposal_id: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    proposal_authority_binding: ActionHash,
    constitution_statement_digest: ConstitutionDigest32,
    threshold_authorization_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedExecutorDesignation {
    protocol: String,
    proposal_id: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    proposal_authority_binding: ActionHash,
    constitution_statement_digest: ConstitutionDigest32,
    threshold_authorization_ref: String,
    executor_principal: String,
    executor_authority_ref: String,
    granting_institution_ref: String,
    capability_scope: String,
    rulebook_ref: String,
    valid_until_ms: u64,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ResolveEffectSafetyPolicyRequest {
    proposal_id: String,
    plan_ref: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    executor_principal: String,
    executor_authority_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedEffectSafetyPolicy {
    protocol: String,
    proposal_id: String,
    plan_ref: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    executor_principal: String,
    executor_authority_ref: String,
    policy_digest: Digest32,
    policy_profile: String,
    effect_class: String,
    enforcement_mechanism: String,
    automatic_effects_allowed: bool,
    valid_until_ms: u64,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedExecutionDomainReceipt {
    pub protocol: String,
    pub proposal_id: String,
    pub domain: ExecutionDomain,
    pub domain_digest: Digest32,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    /// Stable authority identity, not a verifier timestamp.
    pub preflight_verification_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifyLifecycleEventRequest {
    pub proposal_id: String,
    pub candidate_action: ActionHash,
    pub domain: ExecutionDomain,
    pub event: LifecycleEvent,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LifecycleEventVerificationReceipt {
    pub protocol: String,
    pub candidate_action: ActionHash,
    pub execution_domain_digest: Digest32,
    pub event_id: Digest32,
    pub actor_id: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifyEffectSafetyRequest {
    pub proposal_id: String,
    pub execution_domain_digest: Digest32,
    pub executor_principal: String,
    pub executor_authority_ref: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EffectSafetyVerificationReceipt {
    pub protocol: String,
    pub execution_domain_digest: Digest32,
    pub executor_principal: String,
    pub executor_authority_ref: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyOutcomeRequest {
    proposal_id: String,
    candidate_action: ActionHash,
    execution_domain_digest: Digest32,
    event_id: Digest32,
    event: LifecycleEvent,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedOutcomeEvidence {
    protocol: String,
    proposal_id: String,
    candidate_action: ActionHash,
    execution_domain_digest: Digest32,
    event_id: Digest32,
    attempt_id: Digest32,
    evidence_ref: String,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyCancellationRequest {
    proposal_id: String,
    candidate_action: ActionHash,
    execution_domain_digest: Digest32,
    event_id: Digest32,
    actor_id: String,
    authorization_ref: String,
    reason_digest: Digest32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedCancellationAuthority {
    protocol: String,
    proposal_id: String,
    candidate_action: ActionHash,
    execution_domain_digest: Digest32,
    event_id: Digest32,
    actor_id: String,
    authorization_ref: String,
    reason_digest: Digest32,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LifecycleVerifierStatus {
    pub protocol: String,
    pub composition_only: bool,
    pub caller_identity_authority: bool,
    pub advisory_score_authority: bool,
    pub mutable_timelock_authority: bool,
    pub external_effects_enabled: bool,
    pub required_plan_verifier: String,
    pub required_preflight: String,
    pub required_threshold_verifier: String,
    pub required_executor_verifier: String,
    pub required_safety_verifier: String,
    pub required_outcome_verifier: String,
    pub required_cancellation_verifier: String,
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
                "{zome}::{function} unavailable; lifecycle verification fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode {zome}::{function} response: {e}"
        )))
    })
}

fn now_ms() -> ExternResult<u64> {
    let micros = sys_time()?.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Current time must be positive".into(),
        )));
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

fn require_current(verified_at_ms: u64, valid_until_ms: u64, now: u64, field: &str) -> ExternResult<()> {
    if verified_at_ms == 0 || verified_at_ms > now {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} verification timestamp is invalid or in the future"
        ))));
    }
    if valid_until_ms == 0 || now >= valid_until_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} is expired or has no bounded validity"
        ))));
    }
    Ok(())
}

fn require_verified_time(verified_at_ms: u64, now: u64, field: &str) -> ExternResult<()> {
    if verified_at_ms == 0 || verified_at_ms > now {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} verification timestamp is invalid or in the future"
        ))));
    }
    Ok(())
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

fn constitution_digest(value: ConstitutionDigest32) -> Digest32 {
    Digest32(value.0)
}

fn preflight_identity(preflight: &VerifiedExecutionPreflightMirror) -> String {
    format!(
        "preflight:{}:{}:{}:{}:{}:{}",
        preflight.proposal_id,
        preflight.proposal_authority_binding,
        digest_hex(constitution_digest(preflight.constitution_statement_digest)),
        preflight.binding_tally_action,
        digest_hex(preflight.actions_digest),
        preflight.actions_digest_profile,
    )
}

fn composition_ref(
    plan: &VerifiedExecutionPlan,
    preflight_ref: &str,
    threshold: &VerifiedExecutionThresholdAuthorization,
    executor: &VerifiedExecutorDesignation,
    safety: &VerifiedEffectSafetyPolicy,
) -> String {
    format!(
        "plan={}|preflight={}|threshold={}|executor={}|safety={}",
        plan.verification_ref,
        preflight_ref,
        threshold.verification_ref,
        executor.verification_ref,
        safety.verification_ref,
    )
}

fn resolve_plan(proposal_id: &str, now: u64) -> ExternResult<VerifiedExecutionPlan> {
    let plan: VerifiedExecutionPlan = call_local(
        PLAN_VERIFIER_ZOME,
        "resolve_execution_plan",
        proposal_id.to_string(),
    )?;
    if plan.protocol != PLAN_PROTOCOL || plan.proposal_id != proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution-plan verifier returned mismatched protocol/proposal".into(),
        )));
    }
    if plan.actions.is_empty() || plan.actions.len() > MAX_ACTION_BYTES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Verified execution plan actions must be 1-{MAX_ACTION_BYTES} bytes"
        ))));
    }
    if plan.actions_digest.is_zero() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Verified execution plan has a zero action digest".into(),
        )));
    }
    require_ref(&plan.plan_ref, "execution plan ref")?;
    require_ref(&plan.actions_digest_profile, "execution action digest profile")?;
    require_ref(&plan.verification_ref, "execution plan verification ref")?;
    require_verified_time(plan.verified_at_ms, now, "execution plan")?;
    Ok(plan)
}

fn resolve_preflight(plan: &VerifiedExecutionPlan, now: u64) -> ExternResult<VerifiedExecutionPreflightMirror> {
    let preflight: VerifiedExecutionPreflightMirror = call_local(
        PREFLIGHT_ZOME,
        "verify_execution_preflight",
        VerifyExecutionPreflightInput {
            proposal_id: plan.proposal_id.clone(),
            actions: plan.actions.clone(),
        },
    )?;
    if preflight.protocol != PREFLIGHT_PROTOCOL
        || preflight.proposal_id != plan.proposal_id
        || preflight.actions_digest != plan.actions_digest
        || preflight.actions_digest_profile != plan.actions_digest_profile
        || preflight.constitution_version == 0
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution preflight does not exactly bind the immutable execution plan".into(),
        )));
    }
    require_verified_time(preflight.checked_at_ms, now, "execution preflight")?;
    Ok(preflight)
}

fn resolve_threshold(
    preflight: &VerifiedExecutionPreflightMirror,
    now: u64,
) -> ExternResult<VerifiedExecutionThresholdAuthorization> {
    let threshold: VerifiedExecutionThresholdAuthorization = call_local(
        THRESHOLD_VERIFIER_ZOME,
        "verify_execution_threshold_authorization",
        VerifyExecutionThresholdRequest {
            proposal_id: preflight.proposal_id.clone(),
            actions_digest: preflight.actions_digest,
            actions_digest_profile: preflight.actions_digest_profile.clone(),
            proposal_authority_binding: preflight.proposal_authority_binding.clone(),
            constitution_statement_digest: preflight.constitution_statement_digest,
            binding_tally_action: preflight.binding_tally_action.clone(),
            checked_at_ms: preflight.checked_at_ms,
        },
    )?;
    if threshold.protocol != THRESHOLD_PROTOCOL
        || threshold.proposal_id != preflight.proposal_id
        || threshold.actions_digest != preflight.actions_digest
        || threshold.actions_digest_profile != preflight.actions_digest_profile
        || threshold.proposal_authority_binding != preflight.proposal_authority_binding
        || threshold.constitution_statement_digest != preflight.constitution_statement_digest
        || threshold.binding_tally_action != preflight.binding_tally_action
        || threshold.key_epoch == 0
        || threshold.signing_policy_digest.is_zero()
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold verifier returned an inexact execution authorization".into(),
        )));
    }
    require_ref(&threshold.threshold_authorization_ref, "threshold authorization ref")?;
    require_ref(&threshold.committee_ref, "threshold committee ref")?;
    require_ref(&threshold.signing_policy_profile, "threshold signing policy profile")?;
    require_ref(&threshold.verification_ref, "threshold verification ref")?;
    require_current(threshold.verified_at_ms, threshold.valid_until_ms, now, "threshold authorization")?;
    Ok(threshold)
}

fn resolve_executor(
    preflight: &VerifiedExecutionPreflightMirror,
    threshold: &VerifiedExecutionThresholdAuthorization,
    now: u64,
) -> ExternResult<VerifiedExecutorDesignation> {
    let executor: VerifiedExecutorDesignation = call_local(
        EXECUTOR_VERIFIER_ZOME,
        "resolve_executor_designation",
        ResolveExecutorDesignationRequest {
            proposal_id: preflight.proposal_id.clone(),
            actions_digest: preflight.actions_digest,
            actions_digest_profile: preflight.actions_digest_profile.clone(),
            proposal_authority_binding: preflight.proposal_authority_binding.clone(),
            constitution_statement_digest: preflight.constitution_statement_digest,
            threshold_authorization_ref: threshold.threshold_authorization_ref.clone(),
        },
    )?;
    if executor.protocol != EXECUTOR_PROTOCOL
        || executor.proposal_id != preflight.proposal_id
        || executor.actions_digest != preflight.actions_digest
        || executor.actions_digest_profile != preflight.actions_digest_profile
        || executor.proposal_authority_binding != preflight.proposal_authority_binding
        || executor.constitution_statement_digest != preflight.constitution_statement_digest
        || executor.threshold_authorization_ref != threshold.threshold_authorization_ref
        || !executor.executor_principal.starts_with("did:mycelix:")
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Executor verifier returned an inexact designation".into(),
        )));
    }
    require_ref(&executor.executor_authority_ref, "executor authority ref")?;
    require_ref(&executor.granting_institution_ref, "executor granting institution")?;
    require_ref(&executor.capability_scope, "executor capability scope")?;
    require_ref(&executor.rulebook_ref, "executor rulebook ref")?;
    require_ref(&executor.verification_ref, "executor verification ref")?;
    require_current(executor.verified_at_ms, executor.valid_until_ms, now, "executor designation")?;
    Ok(executor)
}

fn resolve_safety(
    plan: &VerifiedExecutionPlan,
    executor: &VerifiedExecutorDesignation,
    now: u64,
) -> ExternResult<VerifiedEffectSafetyPolicy> {
    let safety: VerifiedEffectSafetyPolicy = call_local(
        SAFETY_PROVIDER_ZOME,
        "resolve_effect_safety_policy",
        ResolveEffectSafetyPolicyRequest {
            proposal_id: plan.proposal_id.clone(),
            plan_ref: plan.plan_ref.clone(),
            actions_digest: plan.actions_digest,
            actions_digest_profile: plan.actions_digest_profile.clone(),
            executor_principal: executor.executor_principal.clone(),
            executor_authority_ref: executor.executor_authority_ref.clone(),
        },
    )?;
    if safety.protocol != SAFETY_PROVIDER_PROTOCOL
        || safety.proposal_id != plan.proposal_id
        || safety.plan_ref != plan.plan_ref
        || safety.actions_digest != plan.actions_digest
        || safety.actions_digest_profile != plan.actions_digest_profile
        || safety.executor_principal != executor.executor_principal
        || safety.executor_authority_ref != executor.executor_authority_ref
        || safety.policy_digest.is_zero()
        || !safety.automatic_effects_allowed
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Effect-safety provider does not authorize automatic execution for this exact domain"
                .into(),
        )));
    }
    require_ref(&safety.policy_profile, "effect-safety policy profile")?;
    require_ref(&safety.effect_class, "effect class")?;
    require_ref(&safety.enforcement_mechanism, "effect-safety enforcement mechanism")?;
    require_ref(&safety.verification_ref, "effect-safety verification ref")?;
    require_current(safety.verified_at_ms, safety.valid_until_ms, now, "effect-safety policy")?;
    Ok(safety)
}

fn compose_domain(
    plan: &VerifiedExecutionPlan,
    preflight: &VerifiedExecutionPreflightMirror,
    threshold: &VerifiedExecutionThresholdAuthorization,
    executor: &VerifiedExecutorDesignation,
    safety: &VerifiedEffectSafetyPolicy,
) -> Result<ExecutionDomain, String> {
    if preflight.actions_digest != plan.actions_digest
        || threshold.actions_digest != plan.actions_digest
        || executor.actions_digest != plan.actions_digest
        || safety.actions_digest != plan.actions_digest
    {
        return Err("authority providers disagree on executable action digest".into());
    }
    if preflight.actions_digest_profile != plan.actions_digest_profile
        || threshold.actions_digest_profile != plan.actions_digest_profile
        || executor.actions_digest_profile != plan.actions_digest_profile
        || safety.actions_digest_profile != plan.actions_digest_profile
    {
        return Err("authority providers disagree on executable action digest profile".into());
    }

    let domain = ExecutionDomain {
        protocol_version: LIFECYCLE_PROTOCOL_VERSION.into(),
        proposal_id: plan.proposal_id.clone(),
        // v0.1 compatibility field: semantically this is the immutable plan ref.
        timelock_ref: plan.plan_ref.clone(),
        proposal_authority_ref: preflight.proposal_authority_binding.to_string(),
        constitutional_epoch: ProfiledDigest {
            digest: constitution_digest(preflight.constitution_statement_digest),
            profile: STATEMENT_PROFILE.into(),
        },
        actions: ProfiledDigest {
            digest: plan.actions_digest,
            profile: plan.actions_digest_profile.clone(),
        },
        binding_tally_ref: preflight.binding_tally_action.to_string(),
        threshold_authorization_ref: threshold.threshold_authorization_ref.clone(),
        executor_principal: executor.executor_principal.clone(),
        executor_authority_ref: executor.executor_authority_ref.clone(),
        effect_safety_policy: ProfiledDigest {
            digest: safety.policy_digest,
            profile: safety.policy_profile.clone(),
        },
    };
    domain.validate().map_err(|e| e.to_string())?;
    Ok(domain)
}

#[hdk_extern]
pub fn resolve_execution_domain(proposal_id: String) -> ExternResult<VerifiedExecutionDomainReceipt> {
    if proposal_id.is_empty() || proposal_id.len() > 256 || !proposal_id.starts_with("MIP-") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution domain requires an MIP proposal id of 1-256 bytes".into(),
        )));
    }
    let now = now_ms()?;
    let plan = resolve_plan(&proposal_id, now)?;
    let preflight = resolve_preflight(&plan, now)?;
    let threshold = resolve_threshold(&preflight, now)?;
    let executor = resolve_executor(&preflight, &threshold, now)?;
    let safety = resolve_safety(&plan, &executor, now)?;
    let domain = compose_domain(&plan, &preflight, &threshold, &executor, &safety)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;
    let domain_digest = domain
        .digest()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let preflight_ref = preflight_identity(&preflight);
    require_ref(&preflight_ref, "preflight identity")?;
    let verification_ref = composition_ref(&plan, &preflight_ref, &threshold, &executor, &safety);
    require_ref(&verification_ref, "domain composition ref")?;

    Ok(VerifiedExecutionDomainReceipt {
        protocol: DOMAIN_PROTOCOL.into(),
        proposal_id,
        domain,
        domain_digest,
        verification_ref,
        verified_at_ms: now,
        preflight_verification_ref: preflight_ref,
    })
}

fn load_exact_candidate(input: &VerifyLifecycleEventRequest) -> ExternResult<Option<LifecycleEventCandidate>> {
    let Some(record) = get(input.candidate_action.clone(), GetOptions::default())? else {
        return Ok(None);
    };
    let candidate: Option<LifecycleEventCandidate> = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(candidate) = candidate else {
        return Ok(None);
    };
    if candidate.proposal_id != input.proposal_id
        || candidate.domain != input.domain
        || candidate.event != input.event
    {
        return Ok(None);
    }
    Ok(Some(candidate))
}

fn verify_outcome(
    input: &VerifyLifecycleEventRequest,
    domain_digest: Digest32,
    event_id: Digest32,
    attempt_id: Digest32,
    evidence_ref: &str,
    now: u64,
) -> ExternResult<Option<String>> {
    let receipt: Option<VerifiedOutcomeEvidence> = call_local(
        OUTCOME_VERIFIER_ZOME,
        "verify_execution_outcome",
        VerifyOutcomeRequest {
            proposal_id: input.proposal_id.clone(),
            candidate_action: input.candidate_action.clone(),
            execution_domain_digest: domain_digest,
            event_id,
            event: input.event.clone(),
        },
    )?;
    let Some(receipt) = receipt else {
        return Ok(None);
    };
    if receipt.protocol != OUTCOME_PROTOCOL
        || receipt.proposal_id != input.proposal_id
        || receipt.candidate_action != input.candidate_action
        || receipt.execution_domain_digest != domain_digest
        || receipt.event_id != event_id
        || receipt.attempt_id != attempt_id
        || receipt.evidence_ref != evidence_ref
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Outcome verifier returned an inexact receipt".into(),
        )));
    }
    require_ref(&receipt.verification_ref, "outcome verification ref")?;
    require_verified_time(receipt.verified_at_ms, now, "outcome evidence")?;
    Ok(Some(receipt.verification_ref))
}

fn verify_cancellation(
    input: &VerifyLifecycleEventRequest,
    domain_digest: Digest32,
    event_id: Digest32,
    authorization_ref: &str,
    reason_digest: Digest32,
    now: u64,
) -> ExternResult<Option<String>> {
    let receipt: Option<VerifiedCancellationAuthority> = call_local(
        CANCELLATION_VERIFIER_ZOME,
        "verify_execution_cancellation_authority",
        VerifyCancellationRequest {
            proposal_id: input.proposal_id.clone(),
            candidate_action: input.candidate_action.clone(),
            execution_domain_digest: domain_digest,
            event_id,
            actor_id: input.event.actor_id.clone(),
            authorization_ref: authorization_ref.to_string(),
            reason_digest,
        },
    )?;
    let Some(receipt) = receipt else {
        return Ok(None);
    };
    if receipt.protocol != CANCELLATION_PROTOCOL
        || receipt.proposal_id != input.proposal_id
        || receipt.candidate_action != input.candidate_action
        || receipt.execution_domain_digest != domain_digest
        || receipt.event_id != event_id
        || receipt.actor_id != input.event.actor_id
        || receipt.authorization_ref != authorization_ref
        || receipt.reason_digest != reason_digest
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Cancellation verifier returned an inexact receipt".into(),
        )));
    }
    require_ref(&receipt.verification_ref, "cancellation verification ref")?;
    require_verified_time(receipt.verified_at_ms, now, "cancellation authority")?;
    Ok(Some(receipt.verification_ref))
}

#[hdk_extern]
pub fn verify_lifecycle_event(
    input: VerifyLifecycleEventRequest,
) -> ExternResult<Option<LifecycleEventVerificationReceipt>> {
    let Some(_candidate) = load_exact_candidate(&input)? else {
        return Ok(None);
    };
    input
        .domain
        .validate()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    input
        .event
        .validate()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if input.domain.proposal_id != input.proposal_id {
        return Ok(None);
    }
    let domain_digest = input
        .domain
        .digest()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if input.event.execution_domain_digest != domain_digest {
        return Ok(None);
    }
    let event_id = input
        .event
        .id()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

    // Reconstruct the domain from current authoritative facts. A stale domain is
    // not eligible merely because its candidate entries remain on the DHT.
    let current = resolve_execution_domain(input.proposal_id.clone())?;
    if current.domain != input.domain || current.domain_digest != domain_digest {
        return Ok(None);
    }
    let now = now_ms()?;
    if input.event.occurred_at_ms > now {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Lifecycle event time is in the future".into(),
        )));
    }

    let authority_ref = match &input.event.kind {
        LifecycleEventKind::Registered => {
            if input.event.actor_id != input.domain.executor_principal {
                return Ok(None);
            }
            format!("executor-registration:{}", input.domain.executor_authority_ref)
        }
        LifecycleEventKind::ReadyAuthorized { preflight_ref } => {
            if input.event.actor_id != input.domain.executor_principal
                || *preflight_ref != current.preflight_verification_ref
            {
                return Ok(None);
            }
            current.verification_ref.clone()
        }
        LifecycleEventKind::Claimed { .. } => {
            if input.event.actor_id != input.domain.executor_principal {
                return Ok(None);
            }
            let safety = verify_effect_safety_policy(VerifyEffectSafetyRequest {
                proposal_id: input.proposal_id.clone(),
                execution_domain_digest: domain_digest,
                executor_principal: input.domain.executor_principal.clone(),
                executor_authority_ref: input.domain.executor_authority_ref.clone(),
                policy_digest: input.domain.effect_safety_policy.digest,
                policy_profile: input.domain.effect_safety_policy.profile.clone(),
            })?;
            safety.verification_ref
        }
        LifecycleEventKind::Completed {
            attempt_id,
            receipt_ref,
        } => {
            if input.event.actor_id != input.domain.executor_principal {
                return Ok(None);
            }
            let Some(reference) = verify_outcome(
                &input,
                domain_digest,
                event_id,
                *attempt_id,
                receipt_ref,
                now,
            )? else {
                return Ok(None);
            };
            reference
        }
        LifecycleEventKind::Failed {
            attempt_id,
            failure_kind,
            evidence_ref,
        } => {
            if input.event.actor_id != input.domain.executor_principal {
                return Ok(None);
            }
            if matches!(failure_kind, TerminalFailureKind::NoEffectObserved)
                && evidence_ref.trim().is_empty()
            {
                return Ok(None);
            }
            let Some(reference) = verify_outcome(
                &input,
                domain_digest,
                event_id,
                *attempt_id,
                evidence_ref,
                now,
            )? else {
                return Ok(None);
            };
            reference
        }
        LifecycleEventKind::Uncertain {
            attempt_id,
            evidence_ref,
        } => {
            if input.event.actor_id != input.domain.executor_principal {
                return Ok(None);
            }
            let Some(reference) = verify_outcome(
                &input,
                domain_digest,
                event_id,
                *attempt_id,
                evidence_ref,
                now,
            )? else {
                return Ok(None);
            };
            reference
        }
        LifecycleEventKind::Cancelled {
            authorization_ref,
            reason_digest,
        } => {
            let Some(reference) = verify_cancellation(
                &input,
                domain_digest,
                event_id,
                authorization_ref,
                *reason_digest,
                now,
            )? else {
                return Ok(None);
            };
            reference
        }
    };
    require_ref(&authority_ref, "lifecycle event authority ref")?;

    Ok(Some(LifecycleEventVerificationReceipt {
        protocol: EVENT_PROTOCOL.into(),
        candidate_action: input.candidate_action,
        execution_domain_digest: domain_digest,
        event_id,
        actor_id: input.event.actor_id,
        verification_ref: authority_ref,
        verified_at_ms: now,
    }))
}

#[hdk_extern]
pub fn verify_effect_safety_policy(
    input: VerifyEffectSafetyRequest,
) -> ExternResult<EffectSafetyVerificationReceipt> {
    let current = resolve_execution_domain(input.proposal_id.clone())?;
    if current.domain_digest != input.execution_domain_digest
        || current.domain.executor_principal != input.executor_principal
        || current.domain.executor_authority_ref != input.executor_authority_ref
        || current.domain.effect_safety_policy.digest != input.policy_digest
        || current.domain.effect_safety_policy.profile != input.policy_profile
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Effect-safety request does not match the exact current execution domain".into(),
        )));
    }
    // `resolve_execution_domain` freshly re-runs the dedicated safety provider;
    // this endpoint merely projects the exact current result into the runtime ABI.
    Ok(EffectSafetyVerificationReceipt {
        protocol: EFFECT_SAFETY_PROTOCOL.into(),
        execution_domain_digest: current.domain_digest,
        executor_principal: current.domain.executor_principal,
        executor_authority_ref: current.domain.executor_authority_ref,
        policy_digest: current.domain.effect_safety_policy.digest,
        policy_profile: current.domain.effect_safety_policy.profile,
        verification_ref: current.verification_ref,
        verified_at_ms: current.verified_at_ms,
    })
}

#[hdk_extern]
pub fn get_lifecycle_verifier_status(_: ()) -> ExternResult<LifecycleVerifierStatus> {
    Ok(LifecycleVerifierStatus {
        protocol: DOMAIN_PROTOCOL.into(),
        composition_only: true,
        caller_identity_authority: false,
        advisory_score_authority: false,
        mutable_timelock_authority: false,
        external_effects_enabled: false,
        required_plan_verifier: PLAN_VERIFIER_ZOME.into(),
        required_preflight: PREFLIGHT_ZOME.into(),
        required_threshold_verifier: THRESHOLD_VERIFIER_ZOME.into(),
        required_executor_verifier: EXECUTOR_VERIFIER_ZOME.into(),
        required_safety_verifier: SAFETY_PROVIDER_ZOME.into(),
        required_outcome_verifier: OUTCOME_VERIFIER_ZOME.into(),
        required_cancellation_verifier: CANCELLATION_VERIFIER_ZOME.into(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn action(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn plan() -> VerifiedExecutionPlan {
        VerifiedExecutionPlan {
            protocol: PLAN_PROTOCOL.into(),
            proposal_id: "MIP-42".into(),
            plan_ref: "execution-plan:MIP-42:v1".into(),
            actions: r#"[{"type":"noop"}]"#.into(),
            actions_digest: digest(1),
            actions_digest_profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            verification_ref: "plan:verified".into(),
            verified_at_ms: 10,
        }
    }

    fn preflight() -> VerifiedExecutionPreflightMirror {
        VerifiedExecutionPreflightMirror {
            protocol: PREFLIGHT_PROTOCOL.into(),
            proposal_id: "MIP-42".into(),
            actions_digest: digest(1),
            actions_digest_profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            proposal_authority_binding: action(2),
            constitution_statement_digest: ConstitutionDigest32([3; 32]),
            constitution_version: 1,
            binding_tally_action: action(4),
            checked_at_ms: 11,
        }
    }

    fn threshold() -> VerifiedExecutionThresholdAuthorization {
        VerifiedExecutionThresholdAuthorization {
            protocol: THRESHOLD_PROTOCOL.into(),
            proposal_id: "MIP-42".into(),
            actions_digest: digest(1),
            actions_digest_profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            proposal_authority_binding: action(2),
            constitution_statement_digest: ConstitutionDigest32([3; 32]),
            binding_tally_action: action(4),
            threshold_authorization_ref: "threshold:authorization:1".into(),
            committee_ref: "committee:1".into(),
            signing_policy_digest: digest(5),
            signing_policy_profile: "signing-policy-v1".into(),
            key_epoch: 1,
            valid_until_ms: 100,
            verification_ref: "threshold:verified".into(),
            verified_at_ms: 12,
        }
    }

    fn executor() -> VerifiedExecutorDesignation {
        VerifiedExecutorDesignation {
            protocol: EXECUTOR_PROTOCOL.into(),
            proposal_id: "MIP-42".into(),
            actions_digest: digest(1),
            actions_digest_profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            proposal_authority_binding: action(2),
            constitution_statement_digest: ConstitutionDigest32([3; 32]),
            threshold_authorization_ref: "threshold:authorization:1".into(),
            executor_principal: "did:mycelix:executor".into(),
            executor_authority_ref: "executor:grant:1".into(),
            granting_institution_ref: "institution:test".into(),
            capability_scope: "governance.execute".into(),
            rulebook_ref: "rulebook:test:v1".into(),
            valid_until_ms: 100,
            verification_ref: "executor:verified".into(),
            verified_at_ms: 13,
        }
    }

    fn safety() -> VerifiedEffectSafetyPolicy {
        VerifiedEffectSafetyPolicy {
            protocol: SAFETY_PROVIDER_PROTOCOL.into(),
            proposal_id: "MIP-42".into(),
            plan_ref: "execution-plan:MIP-42:v1".into(),
            actions_digest: digest(1),
            actions_digest_profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            executor_principal: "did:mycelix:executor".into(),
            executor_authority_ref: "executor:grant:1".into(),
            policy_digest: digest(6),
            policy_profile: "effect-safety/idempotent-v1".into(),
            effect_class: "test.noop".into(),
            enforcement_mechanism: "downstream-idempotency".into(),
            automatic_effects_allowed: true,
            valid_until_ms: 100,
            verification_ref: "safety:verified".into(),
            verified_at_ms: 14,
        }
    }

    #[test]
    fn composition_binds_all_authority_components() {
        let domain = compose_domain(&plan(), &preflight(), &threshold(), &executor(), &safety())
            .unwrap();
        assert_eq!(domain.proposal_id, "MIP-42");
        assert_eq!(domain.timelock_ref, "execution-plan:MIP-42:v1");
        assert_eq!(domain.executor_principal, "did:mycelix:executor");
        assert_eq!(domain.threshold_authorization_ref, "threshold:authorization:1");
        assert_eq!(domain.effect_safety_policy.digest, digest(6));
    }

    #[test]
    fn composition_rejects_action_digest_disagreement() {
        let mut threshold = threshold();
        threshold.actions_digest = digest(99);
        assert!(compose_domain(&plan(), &preflight(), &threshold, &executor(), &safety()).is_err());
    }

    #[test]
    fn composition_rejects_digest_profile_disagreement() {
        let mut safety = safety();
        safety.actions_digest_profile = "another-profile".into();
        assert!(compose_domain(&plan(), &preflight(), &threshold(), &executor(), &safety).is_err());
    }

    #[test]
    fn preflight_identity_excludes_verifier_time() {
        let a = preflight();
        let mut b = a.clone();
        b.checked_at_ms = a.checked_at_ms + 100;
        assert_eq!(preflight_identity(&a), preflight_identity(&b));
    }

    #[test]
    fn preflight_identity_changes_with_binding_tally() {
        let a = preflight();
        let mut b = a.clone();
        b.binding_tally_action = action(77);
        assert_ne!(preflight_identity(&a), preflight_identity(&b));
    }

    #[test]
    fn bounded_authority_must_be_current() {
        assert!(require_current(10, 20, 15, "test").is_ok());
        assert!(require_current(16, 20, 15, "test").is_err());
        assert!(require_current(10, 15, 15, "test").is_err());
        assert!(require_current(10, 0, 15, "test").is_err());
    }
}

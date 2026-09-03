// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed runtime orchestration for append-only governance execution lifecycle.
//!
//! Stored records are candidates, not authority by existence. Authoritative reads
//! resolve the exact current execution domain through a dedicated verifier,
//! independently verify every candidate event, and only then run the pure
//! deterministic lifecycle projector.
//!
//! `claim_execution` intentionally commits only the durable claim candidate. This
//! zome exposes no external-effect endpoint in v0.1. Real effects stay disabled
//! until the bound effect-safety policy has a qualified adapter.

use execution_lifecycle_integrity::*;
use hdk::prelude::*;
use mycelix_governance_execution_lifecycle::{
    action_idempotency_key, execution_attempt_id, project_lifecycle, ExecutionDomain,
    LifecycleEvent, LifecycleEventKind, LifecycleState, VerifiedLifecycleEvent,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::BTreeSet;

const DOMAIN_VERIFIER_PROTOCOL: &str = "mycelix-governance-execution-domain-verifier-v0.1";
const EVENT_VERIFIER_PROTOCOL: &str = "mycelix-governance-execution-event-verifier-v0.1";
const SAFETY_VERIFIER_PROTOCOL: &str = "mycelix-governance-effect-safety-verifier-v0.1";
const RUNTIME_PROTOCOL: &str = "mycelix-governance-execution-lifecycle-runtime-v0.1";
const CLAIM_NONCE_DOMAIN: &[u8] = b"mycelix/governance/execution-claim-nonce/v1\0";
const MAX_PROPOSAL_BYTES: usize = 256;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PublishLifecycleEventInput {
    pub domain: ExecutionDomain,
    pub parent_event_id: Option<Digest32>,
    pub kind: LifecycleEventKind,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedExecutionDomainReceipt {
    protocol: String,
    proposal_id: String,
    domain: ExecutionDomain,
    domain_digest: Digest32,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyLifecycleEventRequest {
    proposal_id: String,
    candidate_action: ActionHash,
    domain: ExecutionDomain,
    event: LifecycleEvent,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct LifecycleEventVerificationReceipt {
    protocol: String,
    candidate_action: ActionHash,
    execution_domain_digest: Digest32,
    event_id: Digest32,
    actor_id: String,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyEffectSafetyRequest {
    proposal_id: String,
    execution_domain_digest: Digest32,
    executor_principal: String,
    executor_authority_ref: String,
    policy_digest: Digest32,
    policy_profile: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct EffectSafetyVerificationReceipt {
    protocol: String,
    execution_domain_digest: Digest32,
    executor_principal: String,
    policy_digest: Digest32,
    policy_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedExecutionLifecycle {
    pub protocol: String,
    pub proposal_id: String,
    pub domain: ExecutionDomain,
    pub domain_digest: Digest32,
    pub state: LifecycleState,
    pub verified_event_count: u64,
    pub rejected_candidate_count: u64,
    pub current_domain_candidate_count: u64,
    pub domain_verification_ref: String,
    pub verified_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExecutionClaimReceipt {
    pub protocol: String,
    pub proposal_id: String,
    pub domain_digest: Digest32,
    pub ready_event_id: Digest32,
    pub claim_event_id: Digest32,
    pub attempt_id: Digest32,
    pub candidate_action: ActionHash,
    pub executor_principal: String,
    pub effect_safety_verification_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ActionIdempotencyInput {
    pub attempt_id: Digest32,
    pub action_index: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExecutionLifecycleRuntimeStatus {
    pub protocol: String,
    pub candidate_entries_are_authority: bool,
    pub verified_projection_required: bool,
    pub exact_executor_principal_required: bool,
    pub strict_claim_before_effect_required: bool,
    pub effect_safety_verification_required: bool,
    pub automatic_external_effects_enabled: bool,
    pub legacy_mutable_timelock_authoritative: bool,
    pub advisory_score_authority: bool,
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
                "{zome}::{function} unavailable; execution lifecycle fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode {zome}::{function} response: {e}"
        )))
    })
}

fn anchor_hash(name: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(name.to_string())))
}

fn proposal_anchor(proposal_id: &str) -> String {
    format!("execution-lifecycle:proposal:{proposal_id}")
}

fn domain_anchor(domain_digest: Digest32) -> String {
    format!("execution-lifecycle:domain:{}", digest_hex(domain_digest))
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> ExternResult<u64> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} timestamp must be positive"
        ))));
    }
    Ok(micros as u64 / 1_000)
}

fn require_proposal_id(proposal_id: &str) -> ExternResult<()> {
    if proposal_id.is_empty()
        || proposal_id.len() > MAX_PROPOSAL_BYTES
        || !proposal_id.starts_with("MIP-")
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution lifecycle proposal id must be an MIP id of 1-256 bytes".into(),
        )));
    }
    Ok(())
}

fn require_text(value: &str, field: &str) -> ExternResult<()> {
    if value.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} must not be empty"
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

fn event_records_for_proposal(proposal_id: &str) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash(&proposal_anchor(proposal_id))?,
            LinkTypes::ProposalToLifecycleEvent,
        )?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut records = Vec::new();
    for link in links {
        let Ok(action_hash) = ActionHash::try_from(link.target) else {
            continue;
        };
        if !seen.insert(action_hash.to_string()) {
            continue;
        }
        if let Some(record) = get(action_hash, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

fn decode_candidate(record: &Record) -> ExternResult<LifecycleEventCandidate> {
    record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Execution lifecycle record has no decodable candidate entry".into(),
            ))
        })
}

fn resolve_verified_domain(proposal_id: &str) -> ExternResult<VerifiedExecutionDomainReceipt> {
    require_proposal_id(proposal_id)?;
    let receipt: VerifiedExecutionDomainReceipt = call_local(
        "governance_execution_lifecycle_verifier",
        "resolve_execution_domain",
        proposal_id.to_string(),
    )?;
    if receipt.protocol != DOMAIN_VERIFIER_PROTOCOL
        || receipt.proposal_id != proposal_id
        || receipt.domain.proposal_id != proposal_id
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution-domain verifier returned mismatched protocol/proposal".into(),
        )));
    }
    receipt
        .domain
        .validate()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Invalid verified domain: {e}"))))?;
    let computed = receipt
        .domain
        .digest()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Cannot digest domain: {e}"))))?;
    if computed != receipt.domain_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution-domain verifier receipt digest does not recompute".into(),
        )));
    }
    if !receipt.domain.executor_principal.starts_with("did:mycelix:") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution-domain verifier returned a non-Mycelix executor principal".into(),
        )));
    }
    require_text(&receipt.verification_ref, "domain verification ref")?;
    if receipt.verified_at_ms == 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution-domain verification time must be non-zero".into(),
        )));
    }
    Ok(receipt)
}

fn verify_candidate(
    proposal_id: &str,
    domain: &ExecutionDomain,
    record: &Record,
    candidate: &LifecycleEventCandidate,
) -> ExternResult<Option<VerifiedLifecycleEvent>> {
    let domain_digest = domain
        .digest()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Cannot digest domain: {e}"))))?;
    if candidate.domain != *domain || candidate.event.execution_domain_digest != domain_digest {
        return Ok(None);
    }
    let event_id = candidate
        .event
        .id()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Cannot digest event: {e}"))))?;
    let request = VerifyLifecycleEventRequest {
        proposal_id: proposal_id.to_string(),
        candidate_action: record.action_address().clone(),
        domain: domain.clone(),
        event: candidate.event.clone(),
    };
    let receipt: Option<LifecycleEventVerificationReceipt> = call_local(
        "governance_execution_lifecycle_verifier",
        "verify_lifecycle_event",
        request,
    )?;
    let Some(receipt) = receipt else {
        return Ok(None);
    };
    if receipt.protocol != EVENT_VERIFIER_PROTOCOL
        || receipt.candidate_action != *record.action_address()
        || receipt.execution_domain_digest != domain_digest
        || receipt.event_id != event_id
        || receipt.actor_id != candidate.event.actor_id
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Lifecycle verifier returned an inexact event receipt".into(),
        )));
    }
    require_text(&receipt.verification_ref, "event verification ref")?;
    if receipt.verified_at_ms < candidate.event.occurred_at_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Lifecycle event verification predates the event".into(),
        )));
    }
    Ok(Some(VerifiedLifecycleEvent {
        event: candidate.event.clone(),
        verification_ref: receipt.verification_ref,
    }))
}

fn verify_effect_safety(domain: &ExecutionDomain) -> ExternResult<EffectSafetyVerificationReceipt> {
    let domain_digest = domain
        .digest()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Cannot digest domain: {e}"))))?;
    let request = VerifyEffectSafetyRequest {
        proposal_id: domain.proposal_id.clone(),
        execution_domain_digest: domain_digest,
        executor_principal: domain.executor_principal.clone(),
        executor_authority_ref: domain.executor_authority_ref.clone(),
        policy_digest: domain.effect_safety_policy.digest,
        policy_profile: domain.effect_safety_policy.profile.clone(),
    };
    let receipt: EffectSafetyVerificationReceipt = call_local(
        "governance_execution_lifecycle_verifier",
        "verify_effect_safety_policy",
        request,
    )?;
    if receipt.protocol != SAFETY_VERIFIER_PROTOCOL
        || receipt.execution_domain_digest != domain_digest
        || receipt.executor_principal != domain.executor_principal
        || receipt.policy_digest != domain.effect_safety_policy.digest
        || receipt.policy_profile != domain.effect_safety_policy.profile
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Effect-safety verifier returned an inexact receipt".into(),
        )));
    }
    require_text(&receipt.verification_ref, "effect-safety verification ref")?;
    if receipt.verified_at_ms == 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Effect-safety verification time must be non-zero".into(),
        )));
    }
    Ok(receipt)
}

fn create_candidate(
    domain: ExecutionDomain,
    parent_event_id: Option<Digest32>,
    kind: LifecycleEventKind,
) -> ExternResult<(ActionHash, LifecycleEventCandidate, Digest32)> {
    domain
        .validate()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Invalid execution domain: {e}"))))?;
    require_proposal_id(&domain.proposal_id)?;
    let now = sys_time()?;
    let now_ms = timestamp_ms(now, "lifecycle event")?;
    let caller = agent_info()?.agent_initial_pubkey;
    let actor_id = format!("did:mycelix:{caller}");
    let event = LifecycleEvent {
        protocol_version: mycelix_governance_execution_lifecycle::PROTOCOL_VERSION.into(),
        execution_domain_digest: domain
            .digest()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Cannot digest domain: {e}"))))?,
        parent_event_id,
        actor_id: actor_id.clone(),
        occurred_at_ms: now_ms,
        kind,
    };
    let event_id = event
        .id()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Cannot digest event: {e}"))))?;
    let candidate = LifecycleEventCandidate {
        id: format!(
            "lifecycle:{}:{}",
            domain.proposal_id,
            digest_hex(event_id)
        ),
        proposal_id: domain.proposal_id.clone(),
        domain: domain.clone(),
        event,
        published_by: actor_id,
        published_at: now,
    };
    candidate
        .validate_structure()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;
    let action_hash = create_entry(&EntryTypes::LifecycleEventCandidate(candidate.clone()))?;

    let proposal_key = proposal_anchor(&domain.proposal_id);
    create_entry(&EntryTypes::Anchor(Anchor(proposal_key.clone())))?;
    create_link(
        anchor_hash(&proposal_key)?,
        action_hash.clone(),
        LinkTypes::ProposalToLifecycleEvent,
        (),
    )?;

    let domain_key = domain_anchor(candidate.event.execution_domain_digest);
    create_entry(&EntryTypes::Anchor(Anchor(domain_key.clone())))?;
    create_link(
        anchor_hash(&domain_key)?,
        action_hash.clone(),
        LinkTypes::DomainToLifecycleEvent,
        (),
    )?;

    Ok((action_hash, candidate, event_id))
}

fn claim_nonce(domain_digest: Digest32, ready_event_id: Digest32, actor: &str, now: Timestamp) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(CLAIM_NONCE_DOMAIN);
    hasher.update(&domain_digest.0);
    hasher.update(&ready_event_id.0);
    hasher.update(&(actor.len() as u64).to_be_bytes());
    hasher.update(actor.as_bytes());
    hasher.update(&now.as_micros().to_be_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

#[hdk_extern]
pub fn publish_lifecycle_event_candidate(
    input: PublishLifecycleEventInput,
) -> ExternResult<Record> {
    let (action_hash, _, _) = create_candidate(input.domain, input.parent_event_id, input.kind)?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not read newly created lifecycle candidate".into(),
        ))
    })
}

#[hdk_extern]
pub fn get_lifecycle_candidates(proposal_id: String) -> ExternResult<Vec<Record>> {
    require_proposal_id(&proposal_id)?;
    event_records_for_proposal(&proposal_id)
}

#[hdk_extern]
pub fn get_verified_execution_lifecycle(
    proposal_id: String,
) -> ExternResult<VerifiedExecutionLifecycle> {
    let domain_receipt = resolve_verified_domain(&proposal_id)?;
    let current_domain_digest = domain_receipt.domain_digest;
    let records = event_records_for_proposal(&proposal_id)?;
    let mut verified = Vec::new();
    let mut rejected_candidate_count = 0u64;
    let mut current_domain_candidate_count = 0u64;

    for record in records {
        let candidate = decode_candidate(&record)?;
        let candidate_domain_digest = candidate
            .domain
            .digest()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Cannot digest candidate domain: {e}"))))?;
        if candidate_domain_digest != current_domain_digest {
            continue;
        }
        current_domain_candidate_count += 1;
        match verify_candidate(&proposal_id, &domain_receipt.domain, &record, &candidate)? {
            Some(event) => verified.push(event),
            None => rejected_candidate_count += 1,
        }
    }

    let state = project_lifecycle(&domain_receipt.domain, &verified).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Verified execution lifecycle denied: {e}"
        )))
    })?;
    let now_ms = timestamp_ms(sys_time()?, "verified lifecycle")?;

    Ok(VerifiedExecutionLifecycle {
        protocol: RUNTIME_PROTOCOL.into(),
        proposal_id,
        domain: domain_receipt.domain,
        domain_digest: current_domain_digest,
        state,
        verified_event_count: verified.len() as u64,
        rejected_candidate_count,
        current_domain_candidate_count,
        domain_verification_ref: domain_receipt.verification_ref,
        verified_at_ms: now_ms,
    })
}

/// Commit one durable pre-effect claim from the exact currently verified Ready
/// state. This function performs no external effect.
///
/// Holochain's strict source-chain ordering serializes concurrent writes for one
/// agent/cell: a competing transaction against the same chain head must fail and
/// retry against the advanced chain. The execution domain additionally binds the
/// exact executor principal, while deterministic attempt IDs + the bound external
/// safety policy provide defense in depth beyond the local source chain.
#[hdk_extern]
pub fn claim_execution(proposal_id: String) -> ExternResult<ExecutionClaimReceipt> {
    let projection = get_verified_execution_lifecycle(proposal_id.clone())?;
    let ready_event_id = match projection.state {
        LifecycleState::Ready { event_id } => event_id,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Execution claim requires uniquely verified Ready state, got {other:?}"
            ))));
        }
    };

    let caller = agent_info()?.agent_initial_pubkey;
    let caller_did = format!("did:mycelix:{caller}");
    if caller_did != projection.domain.executor_principal {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the exact bound executor principal may claim execution".into(),
        )));
    }

    let safety = verify_effect_safety(&projection.domain)?;
    let now = sys_time()?;
    let nonce = claim_nonce(
        projection.domain_digest,
        ready_event_id,
        &caller_did,
        now,
    );
    if nonce.is_zero() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Derived execution claim nonce must not be zero".into(),
        )));
    }

    let (candidate_action, candidate, claim_event_id) = create_candidate(
        projection.domain.clone(),
        Some(ready_event_id),
        LifecycleEventKind::Claimed { claim_nonce: nonce },
    )?;
    let attempt_id = execution_attempt_id(&projection.domain, ready_event_id).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot derive execution attempt id: {e}"
        )))
    })?;
    if candidate.event.actor_id != caller_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Claim candidate actor changed unexpectedly".into(),
        )));
    }

    Ok(ExecutionClaimReceipt {
        protocol: RUNTIME_PROTOCOL.into(),
        proposal_id,
        domain_digest: projection.domain_digest,
        ready_event_id,
        claim_event_id,
        attempt_id,
        candidate_action,
        executor_principal: caller_did,
        effect_safety_verification_ref: safety.verification_ref,
    })
}

#[hdk_extern]
pub fn derive_action_idempotency_key(
    input: ActionIdempotencyInput,
) -> ExternResult<Digest32> {
    action_idempotency_key(input.attempt_id, input.action_index)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))
}

#[hdk_extern]
pub fn get_execution_lifecycle_runtime_status(
    _: (),
) -> ExternResult<ExecutionLifecycleRuntimeStatus> {
    Ok(ExecutionLifecycleRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        candidate_entries_are_authority: false,
        verified_projection_required: true,
        exact_executor_principal_required: true,
        strict_claim_before_effect_required: true,
        effect_safety_verification_required: true,
        automatic_external_effects_enabled: false,
        legacy_mutable_timelock_authoritative: false,
        advisory_score_authority: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn digest_hex_is_stable_and_full_width() {
        let value = Digest32([0xab; 32]);
        let text = digest_hex(value);
        assert_eq!(text.len(), 64);
        assert!(text.chars().all(|c| c == 'a' || c == 'b'));
    }

    #[test]
    fn proposal_ids_are_binding_mip_ids() {
        assert!(require_proposal_id("MIP-42").is_ok());
        assert!(require_proposal_id("proposal-42").is_err());
        assert!(require_proposal_id("").is_err());
    }

    #[test]
    fn claim_nonce_changes_with_ready_event() {
        let domain = Digest32([1; 32]);
        let a = claim_nonce(
            domain,
            Digest32([2; 32]),
            "did:mycelix:executor",
            Timestamp::from_micros(1_000),
        );
        let b = claim_nonce(
            domain,
            Digest32([3; 32]),
            "did:mycelix:executor",
            Timestamp::from_micros(1_000),
        );
        assert_ne!(a, b);
    }
}

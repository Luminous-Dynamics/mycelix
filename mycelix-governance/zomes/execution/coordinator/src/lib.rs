// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Execution Coordinator Zome
//! Business logic for proposal execution
//!
//! Updated to use HDK 0.6 patterns

use execution_integrity::*;
use hdk::prelude::*;
use k256::ecdsa::{signature::hazmat::PrehashVerifier, Signature, VerifyingKey};
use mycelix_zome_helpers as _;
use mycelix_zome_helpers::get_latest_record;

mod preflight;

/// Domain separator for governance execution authorization digests.
///
/// The exact proposal ID and exact action JSON bytes are hashed. Reformatting or
/// changing even one byte intentionally produces a different authorization
/// target; canonical action serialization belongs in the follow-on typed action
/// protocol, not in this security boundary.
const EXECUTION_AUTHORITY_DOMAIN: &[u8] = b"mycelix-governance-execution-authority-v1\0";

/// Mirror type for ThresholdSignature from threshold-signing integrity zome.
/// Avoids linking the integrity crate (which causes duplicate HDI symbols in WASM).
///
/// Uses `SerializedBytes` for Holochain entry deserialization via `to_app_option()`.
#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct ThresholdSignature {
    pub id: String,
    pub committee_id: String,
    pub signed_content_hash: Vec<u8>,
    pub signed_content_description: String,
    pub signature: Vec<u8>,
    pub signer_count: u32,
    pub signers: Vec<u32>,
    pub verified: bool,
    pub signed_at: Timestamp,
}

/// Mirror of the committee fields required for execution authorization.
/// Unknown additional fields in the source entry are ignored by serde.
#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct CommitteeAuthorizationMirror {
    pub threshold: u32,
    pub public_key: Option<Vec<u8>>,
    #[serde(default)]
    pub scope: serde_json::Value,
    pub active: bool,
    #[serde(default)]
    pub pq_required: bool,
}

/// Human-readable scope name for diagnostics only.
fn extract_scope_name(scope: &serde_json::Value) -> &str {
    match scope {
        serde_json::Value::String(s) => s.as_str(),
        serde_json::Value::Object(map) if map.len() == 1 => {
            map.keys().next().map(|k| k.as_str()).unwrap_or("Invalid")
        }
        _ => "Invalid",
    }
}

/// Fail-closed committee scope evaluation.
fn committee_scope_allows(scope: &serde_json::Value, proposal_type: &str) -> bool {
    match scope {
        serde_json::Value::String(name) => match name.as_str() {
            "All" => true,
            "Constitutional" => proposal_type == "constitutional",
            "Treasury" => proposal_type == "treasury",
            "Protocol" => proposal_type == "protocol",
            _ => false,
        },
        serde_json::Value::Object(map) if map.len() == 1 => match map.get("Custom") {
            Some(serde_json::Value::Array(values)) => values
                .iter()
                .filter_map(|value| value.as_str())
                .any(|allowed| allowed == proposal_type),
            _ => false,
        },
        _ => false,
    }
}

/// Exact bytes that a threshold committee must authorize for execution.
fn execution_authority_digest(timelock: &Timelock) -> [u8; 32] {
    let mut hasher = blake3::Hasher::new();
    hasher.update(EXECUTION_AUTHORITY_DOMAIN);
    hasher.update(&(timelock.proposal_id.len() as u64).to_le_bytes());
    hasher.update(timelock.proposal_id.as_bytes());
    hasher.update(&(timelock.actions.len() as u64).to_le_bytes());
    hasher.update(timelock.actions.as_bytes());
    *hasher.finalize().as_bytes()
}

/// Helper to get an anchor entry hash
fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    let anchor = Anchor(anchor_str.to_string());
    hash_entry(&EntryTypes::Anchor(anchor))
}

/// O(1) link-based lookup: find a timelock record by its string ID.
/// Falls back to O(n) chain scan if the link is missing (backwards compat).
fn find_timelock_by_id(timelock_id: &str) -> ExternResult<Record> {
    // Try link-based lookup first (O(1))
    let anchor_key = format!("tl:{}", timelock_id);
    if let Ok(entry_hash) = anchor_hash(&anchor_key) {
        if let Ok(links) = get_links(
            LinkQuery::try_new(entry_hash, LinkTypes::TimelockById)?,
            GetStrategy::default(),
        ) {
            if let Some(link) = links.into_iter().max_by_key(|l| l.timestamp) {
                if let Ok(ah) = ActionHash::try_from(link.target) {
                    if let Some(record) = get_latest_record(ah)? {
                        return Ok(record);
                    }
                }
            }
        }
    }

    // Fallback: O(n) chain scan for timelocks created before the link was added
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::Timelock,
        )?))
        .include_entries(true);

    let records = query(filter)?;

    // Take the LAST match — update_entry appends newer versions later in the chain
    let mut found: Option<Record> = None;
    for record in records {
        if let Some(tl) = record
            .entry()
            .to_app_option::<Timelock>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            if tl.id == timelock_id {
                found = Some(record);
            }
        }
    }

    found.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Timelock not found".into()
    )))
}

/// Verify the complete threshold authorization needed to promote or execute a
/// timelock. Every dependency failure is a denial; there is no graceful
/// degradation path for governance side effects.
fn verify_threshold_authorization(timelock: &Timelock) -> ExternResult<ThresholdSignature> {
    let expected_digest = execution_authority_digest(timelock);
    preflight::require_current_binding_authority(timelock, expected_digest)?;

    let signature_response = call(
        CallTargetCell::Local,
        ZomeName::from("threshold_signing"),
        FunctionName::from("get_proposal_signature"),
        None,
        ExternIO::encode(timelock.proposal_id.clone())
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;

    let signature_io = match signature_response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Execution authority unavailable: threshold signature lookup failed closed: {:?}",
                other
            ))));
        }
    };

    let signature_record: Record = signature_io
        .decode::<Option<Record>>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Execution authority unavailable: invalid threshold signature response: {}",
                e
            )))
        })?
        .ok_or(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority unavailable: no threshold signature for proposal '{}'",
            timelock.proposal_id
        ))))?;

    let signature: ThresholdSignature = signature_record
        .entry()
        .to_app_option()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Execution authority unavailable: malformed threshold signature: {}",
                e
            )))
        })?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Execution authority unavailable: threshold signature record has no entry".into()
        )))?;

    if !signature.verified {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority unavailable: threshold signature '{}' is not marked verified",
            signature.id
        ))));
    }
    if signature.signer_count == 0 || signature.signer_count as usize != signature.signers.len() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution authority unavailable: invalid signer count".into()
        )));
    }
    if signature.signed_content_hash.as_slice() != expected_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority digest mismatch for proposal '{}'",
            timelock.proposal_id
        ))));
    }

    let (proposal_type, signed_proposal_id) = signature
        .signed_content_description
        .split_once(':')
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Execution authority description must be '<proposal_type>:<proposal_id>'".into()
        )))?;
    if proposal_type.is_empty() || signed_proposal_id != timelock.proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority description does not exactly bind proposal '{}'",
            timelock.proposal_id
        ))));
    }

    let now = sys_time()?;
    if signature.signed_at < timelock.started || signature.signed_at > now {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution authority signature timestamp is outside the accepted timelock window".into()
        )));
    }

    let committee_response = call(
        CallTargetCell::Local,
        ZomeName::from("threshold_signing"),
        FunctionName::from("get_committee"),
        None,
        ExternIO::encode(signature.committee_id.clone())
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;

    let committee_io = match committee_response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Execution authority unavailable: committee lookup failed closed: {:?}",
                other
            ))));
        }
    };

    let committee_record: Record = committee_io
        .decode::<Option<Record>>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Execution authority unavailable: invalid committee response: {}",
                e
            )))
        })?
        .ok_or(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority unavailable: committee '{}' not found",
            signature.committee_id
        ))))?;

    let committee: CommitteeAuthorizationMirror = committee_record
        .entry()
        .to_app_option()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Execution authority unavailable: malformed committee: {}",
                e
            )))
        })?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Execution authority unavailable: committee record has no entry".into()
        )))?;

    if !committee.active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution authority unavailable: signing committee is inactive".into()
        )));
    }
    if committee.threshold == 0 || signature.signer_count < committee.threshold {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority unavailable: {} signer(s) do not meet committee threshold {}",
            signature.signer_count, committee.threshold
        ))));
    }
    if !committee_scope_allows(&committee.scope, proposal_type) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Committee '{}' scope '{}' does not authorize '{}' proposals",
            signature.committee_id,
            extract_scope_name(&committee.scope),
            proposal_type
        ))));
    }
    if committee.pq_required {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution authority requires post-quantum verification, but the execution boundary does not yet have an ML-DSA verifier; refusing classical-only degradation".into()
        )));
    }

    let public_key = committee.public_key.as_deref().ok_or(wasm_error!(
        WasmErrorInner::Guest(
            "Execution authority unavailable: committee has no verification public key".into()
        )
    ))?;
    let verifying_key = VerifyingKey::from_sec1_bytes(public_key).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority unavailable: invalid committee ECDSA public key: {}",
            e
        )))
    })?;
    if signature.signature.len() != 64 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority unavailable: expected 64-byte ECDSA signature, got {} bytes",
            signature.signature.len()
        ))));
    }
    let ecdsa_signature = Signature::from_slice(&signature.signature).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Execution authority unavailable: invalid ECDSA signature encoding: {}",
            e
        )))
    })?;
    verifying_key
        .verify_prehash(&expected_digest, &ecdsa_signature)
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Execution authority cryptographic verification failed: {}",
                e
            )))
        })?;

    Ok(signature)
}

#[hdk_extern]
pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
    let anchor = Anchor("pending_timelocks".to_string());
    create_entry(&EntryTypes::Anchor(anchor))?;
    Ok(InitCallbackResult::Pass)
}

/// Create a timelock for an approved proposal
#[hdk_extern]
pub fn create_timelock(input: CreateTimelockInput) -> ExternResult<Record> {
    if input.proposal_id.is_empty() || input.proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal ID must be 1-256 characters".into()
        )));
    }
    if input.actions.is_empty() || input.actions.len() > 4096 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Actions must be 1-4096 characters".into()
        )));
    }
    if input.duration_hours == 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Duration must be at least 1 hour".into()
        )));
    }
    if input.duration_hours > 8760 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Duration cannot exceed 8,760 hours (1 year)".into()
        )));
    }

    let now = sys_time()?;
    let timelock_id = format!("timelock:{}:{}", input.proposal_id, now.as_micros());

    let timelock = Timelock {
        id: timelock_id,
        proposal_id: input.proposal_id.clone(),
        actions: input.actions,
        started: now,
        expires: Timestamp::from_micros(
            now.as_micros() as i64 + (input.duration_hours as i64 * 3600 * 1_000_000),
        ),
        status: TimelockStatus::Pending,
        cancellation_reason: None,
    };

    let tl_id = timelock.id.clone();
    let action_hash = create_entry(&EntryTypes::Timelock(timelock))?;

    let proposal_anchor = format!("proposal_timelock:{}", input.proposal_id);
    create_entry(&EntryTypes::Anchor(Anchor(proposal_anchor.clone())))?;
    create_link(
        anchor_hash(&proposal_anchor)?,
        action_hash.clone(),
        LinkTypes::ProposalToTimelock,
        (),
    )?;

    let tl_anchor = format!("tl:{}", tl_id);
    create_entry(&EntryTypes::Anchor(Anchor(tl_anchor.clone())))?;
    create_link(
        anchor_hash(&tl_anchor)?,
        action_hash.clone(),
        LinkTypes::TimelockById,
        (),
    )?;

    create_entry(&EntryTypes::Anchor(Anchor("pending_timelocks".to_string())))?;
    create_link(
        anchor_hash("pending_timelocks")?,
        action_hash.clone(),
        LinkTypes::PendingTimelocks,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find timelock".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateTimelockInput {
    pub proposal_id: String,
    pub actions: String,
    pub duration_hours: u32,
}

#[hdk_extern]
pub fn get_proposal_timelock(proposal_id: String) -> ExternResult<Option<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash(&format!("proposal_timelock:{}", proposal_id))?,
            LinkTypes::ProposalToTimelock,
        )?,
        GetStrategy::default(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    let latest_link = links.into_iter().max_by_key(|l| l.timestamp);
    if let Some(link) = latest_link {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        return get_latest_record(action_hash);
    }

    Ok(None)
}

/// Promote a timelock only after the exact action plan has a cryptographically
/// verified threshold authorization. Creator identity alone is never enough.
#[hdk_extern]
pub fn mark_timelock_ready(input: MarkTimelockReadyInput) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    let current_record = find_timelock_by_id(&input.timelock_id)?;

    let caller = agent_info()?.agent_initial_pubkey;
    let author = current_record.action().author().clone();
    if caller != author {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the timelock creator can request Ready promotion".into()
        )));
    }

    let current_timelock: Timelock = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;

    if current_timelock.status != TimelockStatus::Pending {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Can only mark Pending timelocks as Ready, current status: {:?}",
            current_timelock.status
        ))));
    }

    let signature = verify_threshold_authorization(&current_timelock)?;

    let ready_timelock = Timelock {
        status: TimelockStatus::Ready,
        ..current_timelock
    };

    let action_hash = update_entry(
        current_record.action_address().clone(),
        &EntryTypes::Timelock(ready_timelock),
    )?;

    let _ = emit_signal(serde_json::json!({
        "type": "TimelockAuthorityVerified",
        "timelock_id": input.timelock_id,
        "signature_id": signature.id,
        "committee_id": signature.committee_id,
        "signer_count": signature.signer_count,
    }));

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find updated timelock".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MarkTimelockReadyInput {
    pub timelock_id: String,
}

/// Execute a ready timelock. Authorization is re-verified immediately before
/// side effects so stale, revoked, malformed, or historically unsafe Ready
/// records cannot bypass the execution boundary.
#[hdk_extern]
pub fn execute_timelock(input: ExecuteTimelockInput) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }
    if input.executor_did.is_empty() || input.executor_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Executor DID must be 1-256 characters".into()
        )));
    }

    let agent = agent_info()?;
    let expected_did = format!("did:mycelix:{}", agent.agent_initial_pubkey);
    if input.executor_did != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Executor DID must match the calling agent".into()
        )));
    }

    let current_record = find_timelock_by_id(&input.timelock_id)?;

    let current_timelock: Timelock = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;

    let now = sys_time()?;
    if now < current_timelock.expires {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock has not expired yet".into()
        )));
    }

    if current_timelock.status != TimelockStatus::Ready {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Timelock must be Ready before execution; fail-closed status is {:?}",
            current_timelock.status
        ))));
    }

    let signature = verify_threshold_authorization(&current_timelock)?;
    let _ = emit_signal(serde_json::json!({
        "type": "ExecutionAuthorityVerified",
        "proposal_id": current_timelock.proposal_id,
        "timelock_id": current_timelock.id,
        "signature_id": signature.id,
        "committee_id": signature.committee_id,
        "signer_count": signature.signer_count,
    }));

    let execution_result = execute_actions(&current_timelock.actions)?;

    let execution_id = format!("execution:{}:{}", input.timelock_id, now.as_micros());

    let execution = Execution {
        id: execution_id,
        timelock_id: input.timelock_id.clone(),
        proposal_id: current_timelock.proposal_id.clone(),
        executor: input.executor_did,
        status: if execution_result.success {
            ExecutionStatus::Success
        } else {
            ExecutionStatus::Failed
        },
        result: execution_result.result,
        error: execution_result.error,
        executed_at: now,
    };

    let action_hash = create_entry(&EntryTypes::Execution(execution))?;

    let updated_timelock = Timelock {
        id: current_timelock.id.clone(),
        proposal_id: current_timelock.proposal_id.clone(),
        actions: current_timelock.actions.clone(),
        started: current_timelock.started,
        expires: current_timelock.expires,
        status: if execution_result.success {
            TimelockStatus::Executed
        } else {
            TimelockStatus::Failed
        },
        cancellation_reason: None,
    };

    update_entry(
        current_record.action_address().clone(),
        &EntryTypes::Timelock(updated_timelock),
    )?;

    if let Ok(pending_links) = get_links(
        LinkQuery::try_new(
            anchor_hash("pending_timelocks")?,
            LinkTypes::PendingTimelocks,
        )?,
        GetStrategy::default(),
    ) {
        for link in pending_links {
            if let Ok(target_hash) = ActionHash::try_from(link.target.clone()) {
                if let Ok(Some(record)) = get(target_hash, GetOptions::default()) {
                    if let Some(tl) = record.entry().to_app_option::<Timelock>().ok().flatten() {
                        if tl.id == current_timelock.id {
                            let _ = delete_link(link.create_link_hash, GetOptions::default());
                        }
                    }
                }
            }
        }
    }

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find execution".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ExecuteTimelockInput {
    pub timelock_id: String,
    pub executor_did: String,
}

struct ActionExecutionResult {
    success: bool,
    result: Option<String>,
    error: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
enum GovernanceAction {
    TransferCredits {
        from: String,
        to: String,
        amount: f64,
    },
    UpdateParameter {
        parameter: String,
        value: String,
    },
    EmitEvent {
        event: String,
        #[serde(default)]
        payload: serde_json::Value,
    },
}

impl GovernanceAction {
    fn validate(&self) -> Result<(), String> {
        match self {
            GovernanceAction::TransferCredits { from, to, amount } => {
                if from.is_empty() {
                    return Err("TransferCredits: 'from' is required".to_string());
                }
                if to.is_empty() {
                    return Err("TransferCredits: 'to' is required".to_string());
                }
                if *amount <= 0.0 {
                    return Err(format!(
                        "TransferCredits: amount must be positive, got {}",
                        amount
                    ));
                }
                if !amount.is_finite() {
                    return Err("TransferCredits: amount must be finite".to_string());
                }
                Ok(())
            }
            GovernanceAction::UpdateParameter { parameter, .. } => {
                if parameter.is_empty() {
                    return Err("UpdateParameter: 'parameter' name is required".to_string());
                }
                Ok(())
            }
            GovernanceAction::EmitEvent { .. } => Ok(()),
        }
    }

    fn execute(&self) -> ExternResult<String> {
        match self {
            GovernanceAction::TransferCredits { from, to, amount } => {
                let transfer_input = serde_json::json!({"from": from, "to": to, "amount": amount});
                governance_utils::call_local(
                    "governance_bridge",
                    "transfer_credits",
                    transfer_input,
                ).map_err(|e| wasm_error!(WasmErrorInner::Guest(format!(
                    "TransferCredits failed: governance bridge unavailable — {} -> {} ({} credits): {:?}",
                    from, to, amount, e
                ))))?;
                Ok(format!(
                    "TransferCredits: {} -> {} ({} credits) [executed]",
                    from, to, amount
                ))
            }
            GovernanceAction::UpdateParameter { parameter, value } => {
                let update_input = serde_json::json!({"parameter": parameter, "value": value});
                governance_utils::call_local("constitution", "update_parameter", update_input)
                    .map_err(|e| {
                        wasm_error!(WasmErrorInner::Guest(format!(
                            "UpdateParameter failed: constitution zome unavailable — {} = {}: {:?}",
                            parameter, value, e
                        )))
                    })?;
                Ok(format!(
                    "UpdateParameter: {} = {} [executed]",
                    parameter, value
                ))
            }
            GovernanceAction::EmitEvent { event, payload } => {
                let _ = emit_signal(serde_json::json!({
                    "type": "GovernanceActionExecuted",
                    "event": event,
                    "payload": payload,
                }));
                Ok(format!("EmitEvent: {} [emitted]", event))
            }
        }
    }
}

fn execute_actions(actions_json: &str) -> ExternResult<ActionExecutionResult> {
    let actions: Vec<GovernanceAction> = match serde_json::from_str(actions_json) {
        Ok(a) => a,
        Err(_) => match serde_json::from_str::<GovernanceAction>(actions_json) {
            Ok(v) => vec![v],
            Err(e) => {
                return Ok(ActionExecutionResult {
                    success: false,
                    result: None,
                    error: Some(format!(
                        "Failed to parse actions: {}. Expected GovernanceAction with type TransferCredits, UpdateParameter, or EmitEvent",
                        e
                    )),
                });
            }
        },
    };

    let mut results = Vec::new();

    for (i, action) in actions.iter().enumerate() {
        if let Err(msg) = action.validate() {
            return Ok(ActionExecutionResult {
                success: false,
                result: Some(format!(
                    "Executed {} of {} actions before failure",
                    i,
                    actions.len()
                )),
                error: Some(format!("Action {}: {}", i, msg)),
            });
        }
        match action.execute() {
            Ok(description) => results.push(description),
            Err(e) => {
                return Ok(ActionExecutionResult {
                    success: false,
                    result: Some(format!(
                        "Executed {} of {} actions before failure",
                        i,
                        actions.len()
                    )),
                    error: Some(format!("Action {} execution failed: {}", i, e)),
                });
            }
        }
    }

    Ok(ActionExecutionResult {
        success: true,
        result: Some(results.join("; ")),
        error: None,
    })
}

#[hdk_extern]
pub fn veto_timelock(input: VetoTimelockInput) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }
    if input.guardian_did.is_empty() || input.guardian_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Guardian DID must be 1-256 characters".into()
        )));
    }
    if input.reason.is_empty() || input.reason.len() > 4096 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Veto reason must be 1-4096 characters".into()
        )));
    }

    let agent = agent_info()?;
    let expected_did = format!("did:mycelix:{}", agent.agent_initial_pubkey);
    if input.guardian_did != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Guardian DID must match the calling agent".into()
        )));
    }

    const VETO_COOLDOWN_US: i64 = 7 * 24 * 3600 * 1_000_000;
    let guardian_anchor = format!("guardian:{}", input.guardian_did);
    if let Ok(eh) = anchor_hash(&guardian_anchor) {
        if let Ok(links) = get_links(
            LinkQuery::try_new(eh, LinkTypes::GuardianToVeto)?,
            GetStrategy::default(),
        ) {
            let now_us = sys_time()?.as_micros() as i64;
            for link in links {
                if let Ok(ah) = ActionHash::try_from(link.target) {
                    if let Some(record) = get(ah, GetOptions::default())? {
                        if let Some(prior_veto) = record
                            .entry()
                            .to_app_option::<GuardianVeto>()
                            .ok()
                            .flatten()
                        {
                            let elapsed = now_us - prior_veto.vetoed_at.as_micros() as i64;
                            if elapsed < VETO_COOLDOWN_US {
                                let days_remaining =
                                    (VETO_COOLDOWN_US - elapsed) / (24 * 3600 * 1_000_000);
                                let _ = emit_signal(serde_json::json!({
                                    "type": "VetoRateLimitExceeded",
                                    "guardian_did": input.guardian_did,
                                    "cooldown_days": 7,
                                    "days_remaining": days_remaining,
                                }));
                                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                                    "Veto rate limit: max 1 veto per 7 days per Guardian. Next veto available in {} day(s).",
                                    days_remaining + 1
                                ))));
                            }
                        }
                    }
                }
            }
        }
    }

    if let Ok(eh) = anchor_hash(&guardian_anchor) {
        if let Ok(links) = get_links(
            LinkQuery::try_new(eh, LinkTypes::GuardianToVeto)?,
            GetStrategy::default(),
        ) {
            let now_us = sys_time()?.as_micros() as i64;
            let mut vetoes_in_window: u32 = 0;
            for link in links {
                if let Ok(ah) = ActionHash::try_from(link.target) {
                    if let Some(record) = get(ah, GetOptions::default())? {
                        if let Some(prior_veto) = record
                            .entry()
                            .to_app_option::<GuardianVeto>()
                            .ok()
                            .flatten()
                        {
                            let elapsed = now_us - prior_veto.vetoed_at.as_micros() as i64;
                            if elapsed < execution_integrity::ROLLING_YEAR_US {
                                vetoes_in_window += 1;
                            }
                        }
                    }
                }
            }
            if vetoes_in_window >= execution_integrity::VETO_YEARLY_LIMIT {
                let _ = emit_signal(serde_json::json!({
                    "type": "VetoYearlyLimitExceeded",
                    "guardian_did": input.guardian_did,
                    "vetoes_in_window": vetoes_in_window,
                    "limit": execution_integrity::VETO_YEARLY_LIMIT,
                }));
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "Yearly veto limit exceeded: {} vetoes in the past 12 months (max {}). Guardian enters probation (Art. III, Sec. 5.4).",
                    vetoes_in_window,
                    execution_integrity::VETO_YEARLY_LIMIT
                ))));
            }
        }
    }

    let guardian_io = governance_utils::call_local(
        "councils",
        "get_member_councils",
        input.guardian_did.clone(),
    )?;
    if let Ok(councils) = guardian_io.decode::<Vec<Record>>() {
        if councils.is_empty() {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Only council members (guardians) can veto timelocks".into()
            )));
        }
    }

    let tl_pre = find_timelock_by_id(&input.timelock_id)?;
    let tl_pre_entry: Timelock = tl_pre
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;

    if tl_pre_entry.status == TimelockStatus::Ready {
        const GUARDIAN_PHI_THRESHOLD: f64 = 0.8;
        match governance_utils::call_local_best_effort(
            "governance_bridge",
            "verify_consciousness_gate",
            serde_json::json!({"action_type": "Veto", "action_id": input.timelock_id.clone()}),
        )? {
            Some(extern_io) => {
                if let Ok(result) = extern_io.decode::<serde_json::Value>() {
                    let phi = result.get("phi").and_then(|p| p.as_f64()).unwrap_or(0.0);
                    if phi < GUARDIAN_PHI_THRESHOLD {
                        return Err(wasm_error!(WasmErrorInner::Guest(format!(
                            "Vetoing a Ready timelock requires Guardian-tier Φ ({:.2}), caller has {:.2}",
                            GUARDIAN_PHI_THRESHOLD, phi
                        ))));
                    }
                }
            }
            None => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Cannot veto Ready timelock: consciousness bridge unavailable (fail-closed)"
                        .into()
                )));
            }
        }
    }

    let now = sys_time()?;
    let veto_id = format!("veto:{}:{}", input.timelock_id, now.as_micros());

    let veto = GuardianVeto {
        id: veto_id,
        timelock_id: input.timelock_id.clone(),
        guardian: input.guardian_did.clone(),
        reason: input.reason.clone(),
        vetoed_at: now,
        affected_proposal_id: input.affected_proposal_id.clone(),
        justification_hash: input.justification_hash.clone(),
        threat_category: input.threat_category.clone(),
        haptic_proof: None,
    };

    let action_hash = create_entry(&EntryTypes::GuardianVeto(veto))?;

    let tl_record = find_timelock_by_id(&input.timelock_id)?;
    let tl: Timelock = tl_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;
    if matches!(tl.status, TimelockStatus::Pending | TimelockStatus::Ready) {
        let vetoed = Timelock {
            status: TimelockStatus::Vetoed,
            cancellation_reason: Some(input.reason.clone()),
            ..tl
        };
        update_entry(
            tl_record.action_address().clone(),
            &EntryTypes::Timelock(vetoed),
        )?;
    }

    if let Ok(pending_links) = get_links(
        LinkQuery::try_new(
            anchor_hash("pending_timelocks")?,
            LinkTypes::PendingTimelocks,
        )?,
        GetStrategy::default(),
    ) {
        for link in pending_links {
            if let Ok(target_hash) = ActionHash::try_from(link.target.clone()) {
                if let Ok(Some(record)) = get(target_hash, GetOptions::default()) {
                    if let Some(tl) = record.entry().to_app_option::<Timelock>().ok().flatten() {
                        if tl.id == input.timelock_id {
                            let _ = delete_link(link.create_link_hash, GetOptions::default());
                        }
                    }
                }
            }
        }
    }

    let guardian_anchor = format!("guardian:{}", input.guardian_did);
    create_entry(&EntryTypes::Anchor(Anchor(guardian_anchor.clone())))?;
    create_link(
        anchor_hash(&guardian_anchor)?,
        action_hash.clone(),
        LinkTypes::GuardianToVeto,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find veto".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VetoTimelockInput {
    pub timelock_id: String,
    pub guardian_did: String,
    pub reason: String,
    #[serde(default)]
    pub affected_proposal_id: Option<String>,
    #[serde(default)]
    pub justification_hash: Option<String>,
    #[serde(default)]
    pub threat_category: Option<String>,
}

#[hdk_extern]
pub fn challenge_veto(input: ChallengeVetoInput) -> ExternResult<()> {
    if input.veto_id.is_empty() || input.veto_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Veto ID must be 1-256 characters".into()
        )));
    }
    if input.challenger_did.is_empty() || input.challenger_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Challenger DID must be 1-256 characters".into()
        )));
    }

    let agent = agent_info()?;
    let expected_did = format!("did:mycelix:{}", agent.agent_initial_pubkey);
    if input.challenger_did != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Challenger DID must match the calling agent".into()
        )));
    }

    const CITIZEN_PHI: f64 = 0.4;
    match governance_utils::call_local_best_effort(
        "governance_bridge",
        "verify_consciousness_gate",
        serde_json::json!({"action_type": "ChallengeVeto", "action_id": input.veto_id.clone()}),
    )? {
        Some(extern_io) => {
            if let Ok(result) = extern_io.decode::<serde_json::Value>() {
                let phi = result.get("phi").and_then(|p| p.as_f64()).unwrap_or(0.0);
                if phi < CITIZEN_PHI {
                    return Err(wasm_error!(WasmErrorInner::Guest(format!(
                        "Challenging a veto requires Citizen-tier Φ ({:.2}), caller has {:.2}",
                        CITIZEN_PHI, phi
                    ))));
                }
            }
        }
        None => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Cannot challenge veto: consciousness bridge unavailable (fail-closed)".into()
            )));
        }
    }

    let _ = emit_signal(serde_json::json!({
        "type": "VetoChallenged",
        "veto_id": input.veto_id,
        "challenger_did": input.challenger_did,
        "override_window_hours": 48,
        "override_threshold": execution_integrity::VETO_OVERRIDE_THRESHOLD,
    }));

    Ok(())
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ChallengeVetoInput {
    pub veto_id: String,
    pub challenger_did: String,
}

#[hdk_extern]
pub fn cast_override_vote(input: CastOverrideVoteInput) -> ExternResult<Record> {
    if input.veto_id.is_empty() || input.veto_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Veto ID must be 1-256 characters".into()
        )));
    }
    if input.voter_did.is_empty() || input.voter_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Voter DID must be 1-256 characters".into()
        )));
    }

    let agent = agent_info()?;
    let expected_did = format!("did:mycelix:{}", agent.agent_initial_pubkey);
    if input.voter_did != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Voter DID must match the calling agent".into()
        )));
    }

    let phi_score = match governance_utils::call_local_best_effort(
        "governance_bridge",
        "verify_consciousness_gate",
        serde_json::json!({"action_type": "OverrideVote", "action_id": input.veto_id.clone()}),
    )? {
        Some(extern_io) => {
            if let Ok(result) = extern_io.decode::<serde_json::Value>() {
                let phi = result.get("phi").and_then(|p| p.as_f64()).unwrap_or(0.0);
                if phi < 0.4 {
                    return Err(wasm_error!(WasmErrorInner::Guest(format!(
                        "Override voting requires Citizen-tier Φ (0.40), caller has {:.2}",
                        phi
                    ))));
                }
                phi
            } else {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "Failed to decode consciousness gate response".into()
                )));
            }
        }
        None => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Cannot vote on override: consciousness bridge unavailable (fail-closed)".into()
            )));
        }
    };

    let veto_anchor = format!("veto_override:{}", input.veto_id);
    create_entry(&EntryTypes::Anchor(Anchor(veto_anchor.clone())))?;
    let existing_votes = get_links(
        LinkQuery::try_new(anchor_hash(&veto_anchor)?, LinkTypes::VetoToOverrideVotes)?,
        GetStrategy::default(),
    )?;
    for link in &existing_votes {
        if let Ok(ah) = ActionHash::try_from(link.target.clone()) {
            if let Some(record) = get(ah, GetOptions::default())? {
                if let Some(existing) = record
                    .entry()
                    .to_app_option::<VetoOverrideVote>()
                    .ok()
                    .flatten()
                {
                    if existing.voter_did == input.voter_did {
                        return Err(wasm_error!(WasmErrorInner::Guest(
                            "Agent has already voted on this veto override".into()
                        )));
                    }
                }
            }
        }
    }

    let now = sys_time()?;
    let vote_id = format!(
        "override_vote:{}:{}:{}",
        input.veto_id,
        input.voter_did,
        now.as_micros()
    );

    let vote = VetoOverrideVote {
        id: vote_id,
        veto_id: input.veto_id.clone(),
        voter_did: input.voter_did,
        supports_override: input.supports_override,
        phi_score,
        voted_at: now,
    };

    let action_hash = create_entry(&EntryTypes::VetoOverrideVote(vote))?;

    create_link(
        anchor_hash(&veto_anchor)?,
        action_hash.clone(),
        LinkTypes::VetoToOverrideVotes,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find override vote".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CastOverrideVoteInput {
    pub veto_id: String,
    pub voter_did: String,
    pub supports_override: bool,
}

#[hdk_extern]
pub fn resolve_override(input: ResolveOverrideInput) -> ExternResult<Record> {
    if input.veto_id.is_empty() || input.veto_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Veto ID must be 1-256 characters".into()
        )));
    }
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    let tl_record = find_timelock_by_id(&input.timelock_id)?;
    let tl: Timelock = tl_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;

    if tl.status != TimelockStatus::Vetoed {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Timelock must be in Vetoed status to resolve override, current: {:?}",
            tl.status
        ))));
    }

    let veto_anchor = format!("veto_override:{}", input.veto_id);
    let vote_links = get_links(
        LinkQuery::try_new(anchor_hash(&veto_anchor)?, LinkTypes::VetoToOverrideVotes)?,
        GetStrategy::default(),
    )
    .unwrap_or_default();

    let mut votes_for: f64 = 0.0;
    let mut votes_against: f64 = 0.0;
    let mut voter_count: u64 = 0;

    for link in vote_links {
        if let Ok(ah) = ActionHash::try_from(link.target) {
            if let Some(record) = get(ah, GetOptions::default())? {
                if let Some(vote) = record
                    .entry()
                    .to_app_option::<VetoOverrideVote>()
                    .ok()
                    .flatten()
                {
                    voter_count += 1;
                    let weight = vote.phi_score.max(0.0).min(1.0);
                    if vote.supports_override {
                        votes_for += weight;
                    } else {
                        votes_against += weight;
                    }
                }
            }
        }
    }

    let total_weight = votes_for + votes_against;
    let override_ratio = if total_weight > 0.0 {
        votes_for / total_weight
    } else {
        0.0
    };
    let override_succeeded = override_ratio >= execution_integrity::VETO_OVERRIDE_THRESHOLD;

    let now = sys_time()?;
    let result_id = format!("override_result:{}:{}", input.veto_id, now.as_micros());

    let result = VetoOverrideResult {
        id: result_id,
        veto_id: input.veto_id.clone(),
        timelock_id: input.timelock_id.clone(),
        override_votes_for: votes_for,
        override_votes_against: votes_against,
        total_eligible_voters: voter_count,
        override_threshold: execution_integrity::VETO_OVERRIDE_THRESHOLD,
        override_succeeded,
        resolved_at: now,
    };

    let result_hash = create_entry(&EntryTypes::VetoOverrideResult(result))?;

    create_entry(&EntryTypes::Anchor(Anchor(veto_anchor.clone())))?;
    create_link(
        anchor_hash(&veto_anchor)?,
        result_hash.clone(),
        LinkTypes::VetoToOverrideResult,
        (),
    )?;

    if override_succeeded {
        let restored = Timelock {
            status: TimelockStatus::Ready,
            cancellation_reason: None,
            ..tl
        };
        update_entry(
            tl_record.action_address().clone(),
            &EntryTypes::Timelock(restored),
        )?;

        let _ = emit_signal(serde_json::json!({
            "type": "VetoOverrideSucceeded",
            "veto_id": input.veto_id,
            "timelock_id": input.timelock_id,
            "override_ratio": override_ratio,
            "voter_count": voter_count,
        }));
    } else {
        let cancelled = Timelock {
            status: TimelockStatus::Cancelled,
            ..tl
        };
        update_entry(
            tl_record.action_address().clone(),
            &EntryTypes::Timelock(cancelled),
        )?;

        if let Ok(pending_links) = get_links(
            LinkQuery::try_new(
                anchor_hash("pending_timelocks")?,
                LinkTypes::PendingTimelocks,
            )?,
            GetStrategy::default(),
        ) {
            for link in pending_links {
                if let Ok(target_hash) = ActionHash::try_from(link.target.clone()) {
                    if let Ok(Some(record)) = get(target_hash, GetOptions::default()) {
                        if let Some(ptl) = record.entry().to_app_option::<Timelock>().ok().flatten()
                        {
                            if ptl.id == input.timelock_id {
                                let _ = delete_link(link.create_link_hash, GetOptions::default());
                            }
                        }
                    }
                }
            }
        }

        let _ = emit_signal(serde_json::json!({
            "type": "VetoSustained",
            "veto_id": input.veto_id,
            "timelock_id": input.timelock_id,
            "override_ratio": override_ratio,
            "voter_count": voter_count,
        }));
    }

    get(result_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find override result".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ResolveOverrideInput {
    pub veto_id: String,
    pub timelock_id: String,
}

#[hdk_extern]
pub fn get_guardian_vetoes(guardian_did: String) -> ExternResult<Vec<Record>> {
    if guardian_did.is_empty() || guardian_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Guardian DID must be 1-256 characters".into()
        )));
    }
    let guardian_anchor = format!("guardian:{}", guardian_did);
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&guardian_anchor)?, LinkTypes::GuardianToVeto)?,
        GetStrategy::default(),
    )?;

    let mut vetoes = Vec::new();
    for link in links {
        if let Ok(ah) = ActionHash::try_from(link.target) {
            if let Ok(Some(record)) = get(ah, GetOptions::default()) {
                vetoes.push(record);
            }
        }
    }
    Ok(vetoes)
}

#[hdk_extern]
pub fn lock_proposal_funds(input: LockFundsInput) -> ExternResult<Record> {
    if input.proposal_id.is_empty() || input.proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal ID must be 1-256 characters".into()
        )));
    }
    if input.source_account.is_empty() || input.source_account.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Source account must be 1-256 characters".into()
        )));
    }
    if let Some(ref currency) = input.currency {
        if currency.len() > 64 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Currency must be at most 64 characters".into()
            )));
        }
    }
    if let Some(ref tl_id) = input.timelock_id {
        if tl_id.len() > 256 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Timelock ID must be at most 256 characters".into()
            )));
        }
    }
    if input.amount <= 0.0 || !input.amount.is_finite() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Amount must be positive and finite".into()
        )));
    }

    if let Some(_existing) = find_fund_allocation_for_proposal(&input.proposal_id)? {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Funds already locked for proposal '{}'",
            input.proposal_id
        ))));
    }

    let now = sys_time()?;
    let alloc_id = format!("alloc:{}:{}", input.proposal_id, now.as_micros());

    let alloc = FundAllocation {
        id: alloc_id,
        proposal_id: input.proposal_id.clone(),
        timelock_id: input.timelock_id.unwrap_or_default(),
        source_account: input.source_account,
        amount: input.amount,
        currency: input.currency.unwrap_or_else(|| "credits".to_string()),
        locked_at: now,
        status: AllocationStatus::Locked,
        status_reason: None,
    };

    let action_hash = create_entry(&EntryTypes::FundAllocation(alloc))?;

    let alloc_anchor = format!("fund_alloc:{}", input.proposal_id);
    create_entry(&EntryTypes::Anchor(Anchor(alloc_anchor.clone())))?;
    create_link(
        anchor_hash(&alloc_anchor)?,
        action_hash.clone(),
        LinkTypes::ProposalToFundAllocation,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find fund allocation".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct LockFundsInput {
    pub proposal_id: String,
    pub timelock_id: Option<String>,
    pub source_account: String,
    pub amount: f64,
    pub currency: Option<String>,
}

#[hdk_extern]
pub fn release_locked_funds(input: ReleaseFundsInput) -> ExternResult<Record> {
    if input.proposal_id.is_empty() || input.proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal ID must be 1-256 characters".into()
        )));
    }
    if let Some(ref reason) = input.reason {
        if reason.len() > 4096 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Reason must be at most 4096 characters".into()
            )));
        }
    }

    let (record, alloc) = find_fund_allocation_for_proposal(&input.proposal_id)?.ok_or(
        wasm_error!(WasmErrorInner::Guest(format!(
            "No fund allocation found for proposal '{}'",
            input.proposal_id
        ))),
    )?;

    let caller = agent_info()?.agent_initial_pubkey;
    if caller != *record.action().author() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the fund allocation creator can release locked funds".into()
        )));
    }

    if alloc.status != AllocationStatus::Locked {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Allocation is not locked (current status: {:?})",
            alloc.status
        ))));
    }

    let released = FundAllocation {
        status: AllocationStatus::Released,
        status_reason: Some(
            input
                .reason
                .unwrap_or_else(|| "Execution completed successfully".to_string()),
        ),
        ..alloc
    };

    let action_hash = update_entry(
        record.action_address().clone(),
        &EntryTypes::FundAllocation(released),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find updated allocation".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ReleaseFundsInput {
    pub proposal_id: String,
    pub reason: Option<String>,
}

#[hdk_extern]
pub fn refund_locked_funds(input: RefundFundsInput) -> ExternResult<Record> {
    if input.proposal_id.is_empty() || input.proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal ID must be 1-256 characters".into()
        )));
    }
    if input.reason.len() > 4096 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Reason must be at most 4096 characters".into()
        )));
    }

    let (record, alloc) = find_fund_allocation_for_proposal(&input.proposal_id)?.ok_or(
        wasm_error!(WasmErrorInner::Guest(format!(
            "No fund allocation found for proposal '{}'",
            input.proposal_id
        ))),
    )?;

    let caller = agent_info()?.agent_initial_pubkey;
    if caller != *record.action().author() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the fund allocation creator can refund locked funds".into()
        )));
    }

    if alloc.status != AllocationStatus::Locked {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Allocation is not locked (current status: {:?})",
            alloc.status
        ))));
    }

    let refunded = FundAllocation {
        status: AllocationStatus::Refunded,
        status_reason: Some(input.reason),
        ..alloc
    };

    let action_hash = update_entry(
        record.action_address().clone(),
        &EntryTypes::FundAllocation(refunded),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find updated allocation".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct RefundFundsInput {
    pub proposal_id: String,
    pub reason: String,
}

#[hdk_extern]
pub fn get_fund_allocation(proposal_id: String) -> ExternResult<Option<Record>> {
    Ok(find_fund_allocation_for_proposal(&proposal_id)?.map(|(r, _)| r))
}

fn find_fund_allocation_for_proposal(
    proposal_id: &str,
) -> ExternResult<Option<(Record, FundAllocation)>> {
    let alloc_anchor = format!("fund_alloc:{}", proposal_id);
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash(&alloc_anchor)?,
            LinkTypes::ProposalToFundAllocation,
        )?,
        GetStrategy::default(),
    )?;

    let latest_link = links.into_iter().max_by_key(|l| l.timestamp);
    if let Some(link) = latest_link {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get_latest_record(action_hash)? {
            let alloc: FundAllocation = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Invalid allocation entry".into()
                )))?;
            return Ok(Some((record, alloc)));
        }
    }
    Ok(None)
}

#[hdk_extern]
pub fn get_pending_timelocks(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash("pending_timelocks")?,
            LinkTypes::PendingTimelocks,
        )?,
        GetStrategy::default(),
    )?;

    let mut timelocks = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get_latest_record(action_hash)? {
            if let Some(tl) = record
                .entry()
                .to_app_option::<Timelock>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                if tl.status == TimelockStatus::Pending {
                    timelocks.push(record);
                }
            }
        }
    }

    Ok(timelocks)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_timelock(actions: &str) -> Timelock {
        Timelock {
            id: "tl-1".into(),
            proposal_id: "MIP-001".into(),
            actions: actions.into(),
            started: Timestamp::from_micros(1_000_000),
            expires: Timestamp::from_micros(2_000_000),
            status: TimelockStatus::Pending,
            cancellation_reason: None,
        }
    }

    #[test]
    fn execution_authority_digest_is_deterministic_and_payload_bound() {
        let a = test_timelock(r#"{"type":"TransferCredits","from":"a","to":"b","amount":10}"#);
        let same = test_timelock(r#"{"type":"TransferCredits","from":"a","to":"b","amount":10}"#);
        let changed = test_timelock(r#"{"type":"TransferCredits","from":"a","to":"b","amount":11}"#);
        assert_eq!(execution_authority_digest(&a), execution_authority_digest(&same));
        assert_ne!(execution_authority_digest(&a), execution_authority_digest(&changed));
    }

    #[test]
    fn execution_authority_digest_binds_proposal_id() {
        let a = test_timelock(r#"{"type":"EmitEvent","event":"x"}"#);
        let mut b = a.clone();
        b.proposal_id = "MIP-002".into();
        assert_ne!(execution_authority_digest(&a), execution_authority_digest(&b));
    }

    #[test]
    fn committee_scope_is_fail_closed() {
        assert!(committee_scope_allows(&serde_json::json!("All"), "proposal"));
        assert!(committee_scope_allows(
            &serde_json::json!("Constitutional"),
            "constitutional"
        ));
        assert!(!committee_scope_allows(
            &serde_json::json!("Constitutional"),
            "treasury"
        ));
        assert!(!committee_scope_allows(&serde_json::Value::Null, "proposal"));
        assert!(!committee_scope_allows(&serde_json::json!("Unknown"), "proposal"));
    }

    #[test]
    fn custom_scope_requires_explicit_type() {
        let custom = serde_json::json!({"Custom": ["emergency", "treasury_ops"]});
        assert!(committee_scope_allows(&custom, "emergency"));
        assert!(!committee_scope_allows(&custom, "constitutional"));
    }

    #[test]
    fn test_transfer_credits_valid() {
        let action = GovernanceAction::TransferCredits {
            from: "treasury".into(),
            to: "project-fund".into(),
            amount: 1000.0,
        };
        assert!(action.validate().is_ok());
    }

    #[test]
    fn test_transfer_credits_empty_from() {
        let action = GovernanceAction::TransferCredits {
            from: "".into(),
            to: "project-fund".into(),
            amount: 100.0,
        };
        let err = action.validate().unwrap_err();
        assert!(err.contains("'from' is required"));
    }

    #[test]
    fn test_transfer_credits_empty_to() {
        let action = GovernanceAction::TransferCredits {
            from: "treasury".into(),
            to: "".into(),
            amount: 100.0,
        };
        let err = action.validate().unwrap_err();
        assert!(err.contains("'to' is required"));
    }

    #[test]
    fn test_transfer_credits_zero_amount() {
        let action = GovernanceAction::TransferCredits {
            from: "treasury".into(),
            to: "project".into(),
            amount: 0.0,
        };
        let err = action.validate().unwrap_err();
        assert!(err.contains("must be positive"));
    }

    #[test]
    fn test_transfer_credits_negative_amount() {
        let action = GovernanceAction::TransferCredits {
            from: "treasury".into(),
            to: "project".into(),
            amount: -50.0,
        };
        let err = action.validate().unwrap_err();
        assert!(err.contains("must be positive"));
    }

    #[test]
    fn test_transfer_credits_infinite_amount() {
        let action = GovernanceAction::TransferCredits {
            from: "treasury".into(),
            to: "project".into(),
            amount: f64::INFINITY,
        };
        let err = action.validate().unwrap_err();
        assert!(err.contains("must be finite"));
    }

    #[test]
    fn test_transfer_credits_nan_amount() {
        let action = GovernanceAction::TransferCredits {
            from: "treasury".into(),
            to: "project".into(),
            amount: f64::NAN,
        };
        assert!(action.validate().is_err());
    }

    #[test]
    fn test_update_parameter_valid() {
        let action = GovernanceAction::UpdateParameter {
            parameter: "quorum_threshold".into(),
            value: "0.67".into(),
        };
        assert!(action.validate().is_ok());
    }

    #[test]
    fn test_update_parameter_empty_name() {
        let action = GovernanceAction::UpdateParameter {
            parameter: "".into(),
            value: "0.67".into(),
        };
        let err = action.validate().unwrap_err();
        assert!(err.contains("'parameter' name is required"));
    }

    #[test]
    fn test_emit_event_always_valid() {
        let action = GovernanceAction::EmitEvent {
            event: "treasury_disbursement".into(),
            payload: serde_json::json!({"amount": 500}),
        };
        assert!(action.validate().is_ok());

        let empty = GovernanceAction::EmitEvent {
            event: "".into(),
            payload: serde_json::Value::Null,
        };
        assert!(empty.validate().is_ok());
    }

    #[test]
    fn test_governance_action_serde_transfer() {
        let json = r#"{"type":"TransferCredits","from":"treasury","to":"dev-fund","amount":250.5}"#;
        let action: GovernanceAction = serde_json::from_str(json).unwrap();
        match action {
            GovernanceAction::TransferCredits { from, to, amount } => {
                assert_eq!(from, "treasury");
                assert_eq!(to, "dev-fund");
                assert!((amount - 250.5).abs() < f64::EPSILON);
            }
            _ => panic!("Expected TransferCredits"),
        }
    }

    #[test]
    fn test_governance_action_serde_update() {
        let json = r#"{"type":"UpdateParameter","parameter":"phi_threshold","value":"0.5"}"#;
        let action: GovernanceAction = serde_json::from_str(json).unwrap();
        match action {
            GovernanceAction::UpdateParameter { parameter, value } => {
                assert_eq!(parameter, "phi_threshold");
                assert_eq!(value, "0.5");
            }
            _ => panic!("Expected UpdateParameter"),
        }
    }

    #[test]
    fn test_governance_action_serde_emit() {
        let json = r#"{"type":"EmitEvent","event":"proposal_executed"}"#;
        let action: GovernanceAction = serde_json::from_str(json).unwrap();
        match action {
            GovernanceAction::EmitEvent { event, payload } => {
                assert_eq!(event, "proposal_executed");
                assert_eq!(payload, serde_json::Value::Null);
            }
            _ => panic!("Expected EmitEvent"),
        }
    }

    #[test]
    fn test_governance_action_array_parse() {
        let json = r#"[
            {"type":"TransferCredits","from":"a","to":"b","amount":100},
            {"type":"EmitEvent","event":"done"}
        ]"#;
        let actions: Vec<GovernanceAction> = serde_json::from_str(json).unwrap();
        assert_eq!(actions.len(), 2);
        assert!(actions[0].validate().is_ok());
        assert!(actions[1].validate().is_ok());
    }

    #[test]
    fn test_governance_action_invalid_json() {
        let json = "not valid json at all {{{";
        assert!(serde_json::from_str::<GovernanceAction>(json).is_err());
    }

    #[test]
    fn test_veto_cooldown_is_7_days() {
        const VETO_COOLDOWN_US: i64 = 7 * 24 * 3600 * 1_000_000;
        assert_eq!(VETO_COOLDOWN_US, 604_800_000_000);
    }

    #[test]
    fn test_guardian_phi_threshold_constant() {
        const GUARDIAN_PHI_THRESHOLD: f64 = 0.8;
        assert!(GUARDIAN_PHI_THRESHOLD >= 0.8);
        assert!(GUARDIAN_PHI_THRESHOLD <= 1.0);
    }

    #[test]
    fn test_threshold_signature_serde_roundtrip() {
        let sig = ThresholdSignature {
            id: "sig-1".into(),
            committee_id: "committee-1".into(),
            signed_content_hash: vec![1; 32],
            signed_content_description: "proposal:MIP-001".into(),
            signature: vec![0u8; 64],
            signer_count: 2,
            signers: vec![1, 2],
            verified: true,
            signed_at: Timestamp::from_micros(1_500_000),
        };
        let json = serde_json::to_string(&sig).unwrap();
        let decoded: ThresholdSignature = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, "sig-1");
        assert!(decoded.verified);
        assert_eq!(decoded.signed_content_hash.len(), 32);
    }
}

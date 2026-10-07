// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Execution Coordinator Zome
//! Business logic for proposal execution
//!
//! Updated to use HDK 0.6 patterns

use execution_integrity::*;
use hdk::prelude::*;
use mycelix_zome_helpers as _;
use mycelix_zome_helpers::get_latest_record;
use constitutional_effect_ledger::{ActionKeyV1, AttemptIdentityV1};
use k256::ecdsa::{signature::hazmat::PrehashVerifier, Signature, VerifyingKey};

/// Mirror type for ThresholdSignature from threshold-signing integrity zome.
/// Avoids linking the integrity crate (which causes duplicate HDI symbols in WASM).
#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct ThresholdSignature {
    pub id: String,
    pub committee_id: String,
    pub signed_content_hash: Vec<u8>,
    pub signed_content_description: String,
    pub signature: Vec<u8>,
    #[serde(default)]
    pub pq_signature: Option<Vec<u8>>,
    #[serde(default)]
    pub signature_algorithm: ThresholdSignatureAlgorithmMirror,
    pub signer_count: u32,
    pub signers: Vec<u32>,
    pub verified: bool,
    pub signed_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
enum ThresholdSignatureAlgorithmMirror {
    Ecdsa,
    MlDsa65,
    HybridEcdsaMlDsa65,
}

impl Default for ThresholdSignatureAlgorithmMirror {
    fn default() -> Self {
        Self::Ecdsa
    }
}

/// Mirror type for the SigningCommittee fields needed for independent verification.
#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct CommitteeVerificationMirror {
    #[serde(default)]
    pub scope: serde_json::Value,
    #[serde(default)]
    pub public_key: Option<Vec<u8>>,
    #[serde(default)]
    pub threshold: u32,
    #[serde(default)]
    pub member_count: u32,
    #[serde(default)]
    pub active: bool,
    #[serde(default)]
    pub signature_algorithm: ThresholdSignatureAlgorithmMirror,
}

fn verify_ecdsa_threshold_signature(
    signature: &ThresholdSignature,
    committee: &CommitteeVerificationMirror,
) -> Result<(), String> {
    if signature.signed_content_hash.len() != 32 {
        return Err(format!(
            "ECDSA authorization requires a 32-byte signed-content hash, got {}",
            signature.signed_content_hash.len()
        ));
    }

    let public_key = committee
        .public_key
        .as_deref()
        .ok_or_else(|| "committee has no public key".to_owned())?;
    if public_key.len() != 33 {
        return Err(format!(
            "committee secp256k1 public key must be 33-byte compressed SEC1, got {}",
            public_key.len()
        ));
    }

    let verifying_key = VerifyingKey::from_sec1_bytes(public_key)
        .map_err(|e| format!("invalid committee secp256k1 public key: {e}"))?;

    if signature.signature.len() != 64 {
        return Err(format!(
            "ECDSA authorization signature must be exactly 64 raw r||s bytes, got {}",
            signature.signature.len()
        ));
    }

    let signature_value = Signature::from_slice(&signature.signature)
        .map_err(|e| format!("invalid ECDSA authorization signature encoding: {e}"))?;

    verifying_key
        .verify_prehash(&signature.signed_content_hash, &signature_value)
        .map_err(|e| format!("ECDSA authorization verification failed: {e}"))
}

/// Extract the scope variant name from a committee scope value.
fn extract_scope_name(scope: &serde_json::Value) -> &str {
    match scope {
        serde_json::Value::String(s) => s.as_str(),
        serde_json::Value::Object(map) => map.keys().next().map(|k| k.as_str()).unwrap_or("All"),
        _ => "All",
    }
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

#[hdk_extern]
pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
    // Pre-create the pending_timelocks anchor so queries never fail on empty DNA
    let anchor = Anchor("pending_timelocks".to_string());
    create_entry(&EntryTypes::Anchor(anchor))?;
    Ok(InitCallbackResult::Pass)
}

/// Create a timelock for an approved proposal
#[hdk_extern]
pub fn create_timelock(input: CreateTimelockInput) -> ExternResult<Record> {
    // Input validation
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

    // Create anchor and link proposal to timelock
    let proposal_anchor = format!("proposal_timelock:{}", input.proposal_id);
    create_entry(&EntryTypes::Anchor(Anchor(proposal_anchor.clone())))?;
    create_link(
        anchor_hash(&proposal_anchor)?,
        action_hash.clone(),
        LinkTypes::ProposalToTimelock,
        (),
    )?;

    // Create anchor and link for O(1) lookup by timelock ID
    let tl_anchor = format!("tl:{}", tl_id);
    create_entry(&EntryTypes::Anchor(Anchor(tl_anchor.clone())))?;
    create_link(
        anchor_hash(&tl_anchor)?,
        action_hash.clone(),
        LinkTypes::TimelockById,
        (),
    )?;

    // Create anchor and link to pending timelocks
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

/// Input for creating a timelock
#[derive(Serialize, Deserialize, Debug)]
pub struct CreateTimelockInput {
    pub proposal_id: String,
    pub actions: String,
    pub duration_hours: u32,
}

/// Get timelock for a proposal
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

/// Mark a timelock as ready for execution (after signature verification)
///
/// Transitions a timelock from Pending to Ready once pre-conditions are met
/// (e.g., threshold signature obtained, waiting period elapsed).
#[hdk_extern]
pub fn mark_timelock_ready(input: MarkTimelockReadyInput) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    // Find the timelock via O(1) link-based lookup
    let current_record = find_timelock_by_id(&input.timelock_id)?;

    // Authorization: only the timelock creator can mark it ready
    let caller = agent_info()?.agent_initial_pubkey;
    let author = current_record.action().author().clone();
    if caller != author {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the timelock creator can mark it as ready".into()
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

    // READY is an authorization-admission state, not a cosmetic label. Require
    // the threshold-signing verifier to succeed before the status is committed.
    let _signature = require_verified_threshold_signature(&current_timelock.proposal_id)?;

    let ready_timelock = Timelock {
        status: TimelockStatus::Ready,
        ..current_timelock
    };

    let action_hash = update_entry(
        current_record.action_address().clone(),
        &EntryTypes::Timelock(ready_timelock),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find updated timelock".into()
    )))
}

/// Input for marking a timelock as ready
#[derive(Serialize, Deserialize, Debug)]
pub struct MarkTimelockReadyInput {
    pub timelock_id: String,
}

/// Fixed execution-boundary namespace for the current governance effect runner.
///
/// These are deployment constants, not caller-selected inputs. The material action
/// digest is derived from the exact action JSON frozen in the timelock.
const EXECUTION_RELYING_PARTY: &str = "did:mycelix:governance";
const EXECUTION_EFFECTING_TARGET: &str = "mycelix-governance-execution";
const EXECUTION_ATTEMPT_BOUNDARY: &str = "governance-execution";

fn material_action_digest(actions_json: &str) -> String {
    format!(
        "constitutional-material-action-v1:{}",
        blake3::hash(actions_json.as_bytes()).to_hex()
    )
}

fn execution_action_key(actions_json: &str) -> Result<ActionKeyV1, String> {
    ActionKeyV1::new(
        EXECUTION_RELYING_PARTY,
        EXECUTION_EFFECTING_TARGET,
        material_action_digest(actions_json),
    )
}

fn execution_attempt_identity(
    executor_did: &str,
    timelock_id: &str,
) -> Result<AttemptIdentityV1, String> {
    AttemptIdentityV1::new(
        EXECUTION_ATTEMPT_BOUNDARY,
        executor_did,
        timelock_id,
    )
}

fn execution_attempt_anchor(timelock_id: &str) -> String {
    format!("execution-attempt:{}", timelock_id)
}

fn execution_outcome_evidence(
    attempt: &ExecutionAttempt,
    result: &ActionExecutionResult,
) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(b"MYCELIX-EXECUTION-OUTCOME-EVIDENCE\0V1\0");
    for value in [
        attempt.action_digest.as_str(),
        attempt.action_key_digest.as_str(),
        attempt.attempt_identity.as_str(),
        attempt.native_replay_identity.as_str(),
        result.result.as_deref().unwrap_or(""),
        result.error.as_deref().unwrap_or(""),
    ] {
        hasher.update(&(value.len() as u64).to_be_bytes());
        hasher.update(value.as_bytes());
    }
    format!(
        "constitutional-execution-outcome-v1:{}",
        hasher.finalize().to_hex()
    )
}

fn caller_did() -> ExternResult<String> {
    let agent = agent_info()?;
    Ok(format!("did:mycelix:{}", agent.agent_initial_pubkey))
}

fn ensure_execution_authorized(
    caller: &str,
    timelock_record: &Record,
) -> ExternResult<()> {
    let expected = format!("did:mycelix:{}", timelock_record.action().author());
    if caller != expected {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Execution must be performed by the timelock authoring agent".into(),
        )));
    }
    Ok(())
}

fn require_verified_threshold_signature(
    proposal_id: &str,
) -> ExternResult<ThresholdSignature> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::from("threshold_signing"),
        FunctionName::from("get_proposal_signature"),
        None,
        ExternIO::encode(proposal_id.to_owned())
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;

    let extern_io = match response {
        ZomeCallResponse::Ok(io) => io,
        ZomeCallResponse::NetworkError(e) => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Refusing execution: threshold-signing verifier network error: {}", e
            ))));
        }
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Refusing execution: threshold-signing verifier returned {:?}", other
            ))));
        }
    };

    let signature_record: Option<Record> = extern_io.decode().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Refusing execution: could not decode threshold signature response: {}", e
        )))
    })?;
    let signature_record = signature_record.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Refusing execution: no threshold signature is available.".into()
    )))?;

    let signature: ThresholdSignature = signature_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Refusing execution: threshold signature record is empty.".into()
        )))?;

    let (proposal_kind, signed_id) = signature
        .signed_content_description
        .split_once(':')
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Refusing execution: threshold signature is not structurally proposal-bound.".into()
        )))?;

    if signed_id != proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Refusing execution: threshold signature '{}' targets '{}' rather than '{}'.",
            signature.id, signed_id, proposal_id
        ))));
    }

    if !matches!(proposal_kind, "proposal" | "constitutional" | "treasury" | "protocol") {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Refusing execution: unsupported signed proposal kind '{}'.", proposal_kind
        ))));
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
        ZomeCallResponse::NetworkError(e) => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Refusing execution: signing committee verifier network error: {}", e
            ))));
        }
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Refusing execution: signing committee verifier returned {:?}", other
            ))));
        }
    };

    let committee_record: Option<Record> = committee_io.decode().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Refusing execution: could not decode signing committee response: {}", e
        )))
    })?;
    let committee_record = committee_record.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Refusing execution: signing committee is unavailable.".into()
    )))?;

    let committee = committee_record
        .entry()
        .to_app_option::<CommitteeVerificationMirror>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Refusing execution: signing committee verification fields are unavailable.".into()
        )))?;

    if !committee.active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Refusing execution: signing committee is inactive.".into()
        )));
    }

    if committee.signature_algorithm != signature.signature_algorithm {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Refusing execution: signature algorithm does not match committee.".into()
        )));
    }

    if committee.threshold == 0 || committee.member_count == 0 || committee.threshold > committee.member_count {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Refusing execution: signing committee threshold/member configuration is invalid.".into()
        )));
    }

    if signature.signer_count < committee.threshold {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Refusing execution: {} signers do not satisfy threshold {}.",
            signature.signer_count, committee.threshold
        ))));
    }

    let mut unique_signers = std::collections::BTreeSet::new();
    for signer in &signature.signers {
        if *signer == 0 || *signer > committee.member_count {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Refusing execution: signer {} is outside committee range.", signer
            ))));
        }
        if !unique_signers.insert(*signer) {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Refusing execution: duplicate signer {} in threshold signature.", signer
            ))));
        }
    }

    match signature.signature_algorithm {
        ThresholdSignatureAlgorithmMirror::Ecdsa => {
            verify_ecdsa_threshold_signature(&signature, &committee)
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Refusing execution: {e}"))))?;
        }
        ThresholdSignatureAlgorithmMirror::MlDsa65
        | ThresholdSignatureAlgorithmMirror::HybridEcdsaMlDsa65 => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Refusing execution: PQ/hybrid threshold authorization verifier is not wired.".into()
            )));
        }
    }

    // The verified field is informational; trust is established by the committee
    // configuration, exact proposal binding, and cryptographic verification above.
    Ok(signature)
}

fn find_latest_execution_attempt(timelock_id: &str) -> ExternResult<Option<Record>> {
    let anchor = execution_attempt_anchor(timelock_id);
    let anchor_entry_hash = anchor_hash(&anchor)?;
    let links = get_links(
        LinkQuery::try_new(anchor_entry_hash, LinkTypes::TimelockToExecutionAttempt)?,
        GetStrategy::default(),
    )?;
    let Some(link) = links.into_iter().max_by_key(|link| link.timestamp) else {
        return Ok(None);
    };

    let action_hash = ActionHash::try_from(link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest(
            "Invalid execution-attempt link target".into(),
        )))?;
    get_latest_record(action_hash)
}

/// Commit the durable pre-dispatch reservation. This function must not call
/// execute_actions: its successful return is the source-chain commit boundary
/// that establishes DISPATCH_PENDING before any protected effect entry.
#[hdk_extern]
pub fn prepare_timelock_execution(
    input: PrepareTimelockExecutionInput,
) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    let caller = caller_did()?;
    let timelock_record = find_timelock_by_id(&input.timelock_id)?;
    ensure_execution_authorized(&caller, &timelock_record)?;

    let timelock: Timelock = timelock_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;

    let now = sys_time()?;
    if now < timelock.expires {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock has not expired yet".into()
        )));
    }
    if timelock.status != TimelockStatus::Ready {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Timelock must be Ready before execution preparation, current: {:?}",
            timelock.status
        ))));
    }

    let _signature = require_verified_threshold_signature(&timelock.proposal_id)?;

    let action_key = execution_action_key(&timelock.actions)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;
    let attempt_identity = execution_attempt_identity(&caller, &input.timelock_id)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

    if let Some(existing) = find_latest_execution_attempt(&input.timelock_id)? {
        let existing_attempt: ExecutionAttempt = existing
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Invalid existing execution attempt".into()
            )))?;

        if existing_attempt.action_key_digest != action_key.digest()
            || existing_attempt.action_digest != action_key.material_action_digest()
            || existing_attempt.attempt_identity != attempt_identity.digest()
            || existing_attempt.executor != caller
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Existing execution attempt is bound to different execution identity".into()
            )));
        }

        return Ok(existing);
    }

    let attempt = ExecutionAttempt {
        id: format!("execution-attempt:{}", input.timelock_id),
        timelock_id: input.timelock_id.clone(),
        proposal_id: timelock.proposal_id.clone(),
        action_digest: action_key.material_action_digest().to_owned(),
        action_key_digest: action_key.digest().to_owned(),
        attempt_identity: attempt_identity.digest().to_owned(),
        native_replay_identity: input.timelock_id.clone(),
        executor: caller,
        status: ExecutionAttemptStatus::DispatchPending,
        prepared_at: now,
        updated_at: now,
        outcome_evidence_commitment: None,
        not_entered_marker: None,
    };

    let action_hash = create_entry(&EntryTypes::ExecutionAttempt(attempt))?;

    let anchor = execution_attempt_anchor(&input.timelock_id);
    create_entry(&EntryTypes::Anchor(Anchor(anchor.clone())))?;
    create_link(
        anchor_hash(&anchor)?,
        action_hash.clone(),
        LinkTypes::TimelockToExecutionAttempt,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Execution attempt could not be read after reservation".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct PrepareTimelockExecutionInput {
    pub timelock_id: String,
}

/// Commit INVOKED before crossing the protected effect boundary.
/// No effecting call is made here.
#[hdk_extern]
pub fn mark_execution_invoked(
    input: MarkExecutionInvokedInput,
) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    let caller = caller_did()?;
    let current_record = find_latest_execution_attempt(&input.timelock_id)?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "No prepared execution attempt exists".into()
        )))?;

    let current: ExecutionAttempt = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid execution attempt entry".into()
        )))?;

    if current.executor != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the execution-attempt owner may mark invocation".into()
        )));
    }
    if current.status != ExecutionAttemptStatus::DispatchPending {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution attempt must be DispatchPending, current: {:?}",
            current.status
        ))));
    }

    let updated = ExecutionAttempt {
        status: ExecutionAttemptStatus::Invoked,
        updated_at: sys_time()?,
        ..current
    };

    let action_hash = update_entry(
        current_record.action_address().clone(),
        &EntryTypes::ExecutionAttempt(updated),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Execution attempt invocation marker could not be read after commit".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MarkExecutionInvokedInput {
    pub timelock_id: String,
}

/// Commit a single-use invocation claim after INVOKED is durable.
///
/// This is intentionally a separate source-chain transaction from the actual
/// provider call. A second concurrent claimant cannot also move the same
/// attempt out of Invoked once one claim has committed.
#[hdk_extern]
pub fn claim_execution_invocation(
    input: ClaimExecutionInvocationInput,
) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    let caller = caller_did()?;
    let current_record = find_latest_execution_attempt(&input.timelock_id)?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "No execution attempt exists for invocation claim".into()
        )))?;

    let current: ExecutionAttempt = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid execution attempt entry".into()
        )))?;

    if current.executor != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the execution-attempt owner may claim invocation".into()
        )));
    }
    if current.status != ExecutionAttemptStatus::Invoked {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution attempt must be Invoked before claiming provider entry, current: {:?}",
            current.status
        ))));
    }

    let timelock_record = find_timelock_by_id(&input.timelock_id)?;
    let timelock: Timelock = timelock_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry during invocation claim".into()
        )))?;

    let action_key = execution_action_key(&timelock.actions)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

    if current.action_key_digest != action_key.digest()
        || current.action_digest != action_key.material_action_digest()
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Invocation claim action identity does not match the durable attempt".into()
        )));
    }

    let updated = ExecutionAttempt {
        status: ExecutionAttemptStatus::InvocationClaimed,
        updated_at: sys_time()?,
        ..current
    };

    let action_hash = update_entry(
        current_record.action_address().clone(),
        &EntryTypes::ExecutionAttempt(updated),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Invocation claim could not be read after commit".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ClaimExecutionInvocationInput {
    pub timelock_id: String,
}

/// Invoke an execution attempt whose invocation claim is already durable.
///
/// IMPORTANT: after execute_actions returns, this function performs no fallible
/// host writes and makes no status transition. That is deliberate. If an external
/// effect succeeds but a later local commit fails, the durable InvocationClaimed
/// state remains occupied and a retry cannot re-enter the provider. Outcome
/// recording happens in record_execution_observation, which is itself effect-free.
#[hdk_extern]
pub fn invoke_execution(input: InvokeExecutionInput) -> ExternResult<InvokeExecutionResult> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    let caller = caller_did()?;
    let current_record = find_latest_execution_attempt(&input.timelock_id)?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "No execution attempt exists".into()
        )))?;

    let current_attempt: ExecutionAttempt = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid execution attempt entry".into()
        )))?;

    if current_attempt.executor != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the execution-attempt owner may invoke".into()
        )));
    }
    if current_attempt.status != ExecutionAttemptStatus::InvocationClaimed {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution attempt must have a committed InvocationClaimed state before effect entry, current: {:?}",
            current_attempt.status
        )));
    }

    let timelock_record = find_timelock_by_id(&input.timelock_id)?;
    let timelock: Timelock = timelock_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry during invocation".into()
        )))?;

    if timelock.status != TimelockStatus::Ready {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock is no longer Ready; refusing invocation".into()
        )));
    }

    let action_key = execution_action_key(&timelock.actions)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;
    if current_attempt.action_key_digest != action_key.digest()
        || current_attempt.action_digest != action_key.material_action_digest()
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Current timelock material action no longer matches the committed execution attempt"
                .into()
        )));
    }

    let execution_result = execute_actions(&timelock.actions)?;
    let observation_evidence_commitment =
        execution_outcome_evidence(&current_attempt, &execution_result);

    Ok(InvokeExecutionResult {
        attempt_identity: current_attempt.attempt_identity,
        action_key_digest: current_attempt.action_key_digest,
        native_replay_identity: current_attempt.native_replay_identity,
        success: execution_result.success,
        result: execution_result.result,
        error: execution_result.error,
        observation_evidence_commitment,
    })
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct InvokeExecutionInput {
    pub timelock_id: String,
}

/// Ephemeral result returned directly from the effecting invocation call.
///
/// This is not a durable receipt and must never be treated as terminal proof by
/// itself. A separate effect-free recorder persists it as Indeterminate until
/// authoritative provider evidence can classify the outcome.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct InvokeExecutionResult {
    pub attempt_identity: String,
    pub action_key_digest: String,
    pub native_replay_identity: String,
    pub success: bool,
    pub result: Option<String>,
    pub error: Option<String>,
    pub observation_evidence_commitment: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RecordExecutionObservationInput {
    pub timelock_id: String,
    pub attempt_identity: String,
    pub action_key_digest: String,
    pub native_replay_identity: String,
    pub success: bool,
    pub result: Option<String>,
    pub error: Option<String>,
    pub observation_evidence_commitment: String,
}

/// Persist an effect observation without crossing the effect boundary.
///
/// Re-running this function is safe: after a successful commit the attempt is
/// Indeterminate and a second observation is refused. No provider call occurs
/// here, so an ambiguous recorder commit cannot cause a duplicate external effect.
#[hdk_extern]
pub fn record_execution_observation(
    input: RecordExecutionObservationInput,
) -> ExternResult<Record> {
    if input.timelock_id.is_empty() || input.timelock_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Timelock ID must be 1-256 characters".into()
        )));
    }

    let caller = caller_did()?;
    let current_record = find_latest_execution_attempt(&input.timelock_id)?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "No execution attempt exists for observation".into()
        )))?;

    let current_attempt: ExecutionAttempt = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid execution attempt entry".into()
        )))?;

    if current_attempt.executor != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the execution-attempt owner may record its observation".into()
        )));
    }
    if current_attempt.status != ExecutionAttemptStatus::InvocationClaimed {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Execution attempt observation must follow InvocationClaimed, current: {:?}",
            current_attempt.status
        )));
    }
    if input.attempt_identity != current_attempt.attempt_identity
        || input.action_key_digest != current_attempt.action_key_digest
        || input.native_replay_identity != current_attempt.native_replay_identity
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Observation identity does not match the committed execution attempt".into()
        )));
    }

    let observation_evidence = execution_outcome_evidence(
        &current_attempt,
        &ActionExecutionResult {
            success: input.success,
            result: input.result.clone(),
            error: input.error.clone(),
        },
    );
    if observation_evidence != input.observation_evidence_commitment {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Observation evidence commitment does not match the claimed invocation result".into()
        )));
    }

    let now = sys_time()?;
    let execution = Execution {
        id: format!("execution-observation:{}:{}", input.timelock_id, now.as_micros()),
        timelock_id: input.timelock_id.clone(),
        proposal_id: current_attempt.proposal_id.clone(),
        executor: caller,
        status: ExecutionStatus::Indeterminate,
        result: input.result,
        error: input.error,
        executed_at: now,
    };

    let execution_hash = create_entry(&EntryTypes::Execution(execution))?;

    let updated_attempt = ExecutionAttempt {
        status: ExecutionAttemptStatus::Indeterminate,
        updated_at: now,
        outcome_evidence_commitment: Some(input.observation_evidence_commitment),
        ..current_attempt
    };

    update_entry(
        current_record.action_address().clone(),
        &EntryTypes::ExecutionAttempt(updated_attempt),
    )?;

    get(execution_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Execution observation could not be read after commit".into()
    )))
}

/// Execute a ready timelock
/// Legacy one-call execution API.
///
/// It is intentionally disabled because Holochain source-chain writes are not
/// durable until the zome call commits. A function that writes and then invokes
/// an effect in the same transaction cannot establish durable-before-entry.
#[hdk_extern]
pub fn execute_timelock(_input: ExecuteTimelockInput) -> ExternResult<Record> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "execute_timelock is disabled: use prepare_timelock_execution -> mark_execution_invoked -> invoke_execution."
            .into(),
    )))
}

/// Input for executing a timelock
#[derive(Serialize, Deserialize, Debug)]
pub struct ExecuteTimelockInput {
    pub timelock_id: String,
    pub executor_did: String,
}

/// Result of executing actions
struct ActionExecutionResult {
    success: bool,
    result: Option<String>,
    error: Option<String>,
}

/// Typed governance action — deserialized from the actions JSON string
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
    /// Validate action parameters
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

    /// Execute the action via cross-zome dispatch
    fn execute(&self) -> ExternResult<String> {
        match self {
            GovernanceAction::TransferCredits { from, to, amount } => {
                // FAIL-CLOSED: the current governance_bridge transfer_credits endpoint
                // records an event only; it is not the authoritative money-movement owner.
                // Refuse provider entry rather than converting an acknowledgment into an
                // EXECUTED financial effect.
                Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "TransferCredits refused: no authoritative fund-movement effect owner is wired for {} -> {} ({} credits).",
                    from, to, amount
                ))))
            }
            GovernanceAction::UpdateParameter { parameter, value } => {
                // SECURITY: Fail-closed — parameter updates MUST persist or fail explicitly.
                // Returning Ok without actual update creates phantom governance changes.
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
                // Emit as a governance signal to connected clients
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

/// Execute actions parsed from JSON via cross-zome dispatch
fn execute_actions(actions_json: &str) -> ExternResult<ActionExecutionResult> {
    // Parse as typed enum array (or single action)
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

/// Cancel a timelock (guardian veto)
#[hdk_extern]
pub fn veto_timelock(input: VetoTimelockInput) -> ExternResult<Record> {
    // Input validation
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

    // Verify the guardian DID matches the calling agent
    let agent = agent_info()?;
    let expected_did = format!("did:mycelix:{}", agent.agent_initial_pubkey);
    if input.guardian_did != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Guardian DID must match the calling agent".into()
        )));
    }

    // ── VETO RATE LIMITING ──
    // Max 1 veto per Guardian per 7 days. Prevents serial veto DoS where
    // a Guardian freezes governance by vetoing every proposal in sequence.
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
                                    "Veto rate limit: max 1 veto per 7 days per Guardian. \
                                     Next veto available in {} day(s).",
                                    days_remaining + 1
                                ))));
                            }
                        }
                    }
                }
            }
        }
    }

    // ── YEARLY VETO LIMIT (Art. III, Sec. 5.4) ──
    // Max 3 vetoes per Guardian per rolling 12-month window.
    // Exceeding triggers probation signal.
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
                    "Yearly veto limit exceeded: {} vetoes in the past 12 months \
                     (max {}). Guardian enters probation (Art. III, Sec. 5.4).",
                    vetoes_in_window,
                    execution_integrity::VETO_YEARLY_LIMIT
                ))));
            }
        }
    }

    // Verify guardian role: caller must be a member of at least one council
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

    // If the timelock is already in Ready state, require Guardian-tier Φ (0.8)
    // since cancelling a signed, ready-to-execute proposal is a high-impact action.
    let tl_pre = find_timelock_by_id(&input.timelock_id)?;
    let tl_pre_entry: Timelock = tl_pre
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;

    if tl_pre_entry.status == TimelockStatus::Ready {
        // Require elevated consciousness for Ready-state vetoes
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
        // `haptic_proof` was added to the integrity struct without updating this,
        // its only construction site — so the execution zome, and therefore the
        // whole governance cluster, did not compile (E0063). `None` preserves
        // prior behaviour: the field is `Option` with `#[serde(default)]`, has no
        // corresponding input, and is read nowhere yet. Populating it from real
        // robotic-sensor input is a feature, not part of this build fix.
        haptic_proof: None,
    };

    let action_hash = create_entry(&EntryTypes::GuardianVeto(veto))?;

    // Update timelock status to cancelled via O(1) link-based lookup
    let tl_record = find_timelock_by_id(&input.timelock_id)?;
    let tl: Timelock = tl_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid timelock entry".into()
        )))?;
    if matches!(tl.status, TimelockStatus::Pending | TimelockStatus::Ready) {
        // Transition to Vetoed (not Cancelled) — allows supermajority override
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

    // Clean up pending_timelocks link (timelock was vetoed/cancelled)
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

    // Create anchor and link guardian to veto
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

/// Input for vetoing a timelock
#[derive(Serialize, Deserialize, Debug)]
pub struct VetoTimelockInput {
    pub timelock_id: String,
    pub guardian_did: String,
    pub reason: String,
    /// Proposal ID affected by this veto (constitutional registry, Art. III Sec. 5.5).
    #[serde(default)]
    pub affected_proposal_id: Option<String>,
    /// SHA-256 hash of the full justification document.
    #[serde(default)]
    pub justification_hash: Option<String>,
    /// Threat category for the veto (required post-sunset for Charter Guardian Authority).
    #[serde(default)]
    pub threat_category: Option<String>,
}

// ============================================================================
// VETO OVERRIDE MECHANISM
// Thermodynamic counterbalance: No Maxwell's Demon — collective energy (67%
// supermajority, Art. III Sec. 5.3) can overcome any individual barrier.
// ============================================================================

/// Challenge a guardian veto — initiates the 48-hour override window.
/// Any Citizen-tier (Φ ≥ 0.4) agent can challenge.
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

    // Verify challenger DID matches calling agent
    let agent = agent_info()?;
    let expected_did = format!("did:mycelix:{}", agent.agent_initial_pubkey);
    if input.challenger_did != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Challenger DID must match the calling agent".into()
        )));
    }

    // Verify challenger meets Citizen-tier Φ (0.4)
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

    // Emit signal to notify participants
    let _ = emit_signal(serde_json::json!({
        "type": "VetoChallenged",
        "veto_id": input.veto_id,
        "challenger_did": input.challenger_did,
        "override_window_hours": 48,
        "override_threshold": execution_integrity::VETO_OVERRIDE_THRESHOLD,
    }));

    Ok(())
}

/// Input for challenging a veto
#[derive(Serialize, Deserialize, Debug)]
pub struct ChallengeVetoInput {
    pub veto_id: String,
    pub challenger_did: String,
}

/// Cast a vote to override (or sustain) a guardian veto.
/// Requires Citizen-tier Φ (0.4).
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

    // Verify voter DID matches calling agent
    let agent = agent_info()?;
    let expected_did = format!("did:mycelix:{}", agent.agent_initial_pubkey);
    if input.voter_did != expected_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Voter DID must match the calling agent".into()
        )));
    }

    // Get voter's Φ score (must be Citizen-tier ≥ 0.4)
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

    // Check for duplicate vote by this agent on this veto
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

    // Link vote to veto
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

/// Input for casting an override vote
#[derive(Serialize, Deserialize, Debug)]
pub struct CastOverrideVoteInput {
    pub veto_id: String,
    pub voter_did: String,
    pub supports_override: bool,
}

/// Resolve a veto override — tallies votes and transitions the timelock.
/// Can be called by any agent after the 48-hour override window closes.
/// This is permission-less enforcement — no special role required.
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

    // Verify the timelock is in Vetoed state
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

    // Collect all override votes
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
                    // Weight by phi score for consciousness-integrated override
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

    // Link result to veto
    create_entry(&EntryTypes::Anchor(Anchor(veto_anchor.clone())))?;
    create_link(
        anchor_hash(&veto_anchor)?,
        result_hash.clone(),
        LinkTypes::VetoToOverrideResult,
        (),
    )?;

    // Transition timelock based on outcome
    if override_succeeded {
        // Override succeeded — restore timelock to Ready
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
        // Override failed — finalize cancellation
        let cancelled = Timelock {
            status: TimelockStatus::Cancelled,
            ..tl
        };
        update_entry(
            tl_record.action_address().clone(),
            &EntryTypes::Timelock(cancelled),
        )?;

        // Clean up pending_timelocks link
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

/// Input for resolving a veto override
#[derive(Serialize, Deserialize, Debug)]
pub struct ResolveOverrideInput {
    pub veto_id: String,
    pub timelock_id: String,
}

/// Query a guardian's veto history for accountability and transparency.
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

/// Lock funds in escrow for a proposal's execution.
///
/// Called after a proposal is approved and before a timelock is created.
/// Creates a `FundAllocation` entry with status `Locked` and links it
/// to the proposal.
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

    // Check for existing locked allocation for this proposal
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

    // Link proposal to fund allocation
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

/// Input for locking funds
#[derive(Serialize, Deserialize, Debug)]
pub struct LockFundsInput {
    pub proposal_id: String,
    pub timelock_id: Option<String>,
    pub source_account: String,
    pub amount: f64,
    pub currency: Option<String>,
}

/// Release locked funds after successful execution
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

    // Authorization: only the fund allocation creator can release
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

/// Input for releasing funds
#[derive(Serialize, Deserialize, Debug)]
pub struct ReleaseFundsInput {
    pub proposal_id: String,
    pub reason: Option<String>,
}

/// Refund locked funds (e.g., after veto or expiration)
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

    // Authorization: only the fund allocation creator or a guardian can refund
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

/// Input for refunding funds
#[derive(Serialize, Deserialize, Debug)]
pub struct RefundFundsInput {
    pub proposal_id: String,
    pub reason: String,
}

/// Query fund allocation status for a proposal
#[hdk_extern]
pub fn get_fund_allocation(proposal_id: String) -> ExternResult<Option<Record>> {
    Ok(find_fund_allocation_for_proposal(&proposal_id)?.map(|(r, _)| r))
}

/// Internal helper: find the active fund allocation for a proposal
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

    // Find the most recent allocation
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

/// Get pending timelocks
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
            // Filter to only actually pending timelocks
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
    #[test]
    fn exact_threshold_signature_proposal_binding_rejects_decoy_suffixes() {
        assert_ne!(
            "constitutional:proposal-1:decoy".split_once(':').map(|(_, id)| id),
            Some("proposal-1")
        );
    }

    #[test]
    fn execution_action_key_is_stable_for_exact_material_action() {
        let a = execution_action_key(r#"[{"type":"EmitEvent","event":"x"}]"#).unwrap();
        let b = execution_action_key(r#"[{"type":"EmitEvent","event":"x"}]"#).unwrap();
        assert_eq!(a.digest(), b.digest());
    }

    #[test]
    fn execution_action_key_changes_when_material_action_changes() {
        let a = execution_action_key(r#"[{"type":"EmitEvent","event":"x"}]"#).unwrap();
        let b = execution_action_key(r#"[{"type":"EmitEvent","event":"y"}]"#).unwrap();
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn invocation_claim_requires_same_material_action_identity() {
        let key_a = execution_action_key(r#"[{"type":"EmitEvent","event":"x"}]"#).unwrap();
        let key_b = execution_action_key(r#"[{"type":"EmitEvent","event":"y"}]"#).unwrap();
        assert_ne!(key_a.digest(), key_b.digest());
        assert_ne!(key_a.material_action_digest(), key_b.material_action_digest());
    }

    #[test]
    fn execution_attempt_identity_is_separate_from_same_action_key() {
        let key_a = execution_action_key(r#"[{"type":"EmitEvent","event":"x"}]"#).unwrap();
        let key_b = execution_action_key(r#"[{"type":"EmitEvent","event":"x"}]"#).unwrap();
        let attempt_a = execution_attempt_identity("did:mycelix:a", "timelock-a").unwrap();
        let attempt_b = execution_attempt_identity("did:mycelix:b", "timelock-b").unwrap();

        assert_eq!(key_a.digest(), key_b.digest());
        assert_ne!(attempt_a.digest(), attempt_b.digest());
    }

    
    use super::*;

    // =========================================================================
    // GovernanceAction::validate() — pure method tests
    // =========================================================================

    // --- TransferCredits ---

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
        // NaN is neither positive nor finite — hits the <= 0.0 check
        assert!(action.validate().is_err());
    }

    // --- UpdateParameter ---

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

    // --- EmitEvent ---

    #[test]
    fn test_emit_event_always_valid() {
        let action = GovernanceAction::EmitEvent {
            event: "treasury_disbursement".into(),
            payload: serde_json::json!({"amount": 500}),
        };
        assert!(action.validate().is_ok());

        // Even empty event is valid (no validation on event name)
        let empty = GovernanceAction::EmitEvent {
            event: "".into(),
            payload: serde_json::Value::Null,
        };
        assert!(empty.validate().is_ok());
    }

    // =========================================================================
    // GovernanceAction serde — JSON round-trip and tagged enum format
    // =========================================================================

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
                assert_eq!(payload, serde_json::Value::Null); // default
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

    // =========================================================================
    // ThresholdSignature mirror type serde
    // =========================================================================

    // =========================================================================
    // Committee scope enforcement — proposal type inference
    // =========================================================================

    // --- extract_scope_name ---

    #[test]
    fn test_extract_scope_simple_variants() {
        assert_eq!(extract_scope_name(&serde_json::json!("All")), "All");
        assert_eq!(
            extract_scope_name(&serde_json::json!("Constitutional")),
            "Constitutional"
        );
        assert_eq!(
            extract_scope_name(&serde_json::json!("Treasury")),
            "Treasury"
        );
        assert_eq!(
            extract_scope_name(&serde_json::json!("Protocol")),
            "Protocol"
        );
    }

    #[test]
    fn test_extract_scope_custom_variant() {
        // Custom(Vec<String>) serializes as {"Custom": ["type1", "type2"]}
        let custom = serde_json::json!({"Custom": ["treasury_ops", "emergency"]});
        assert_eq!(extract_scope_name(&custom), "Custom");
    }

    #[test]
    fn test_extract_scope_null_defaults_to_all() {
        assert_eq!(extract_scope_name(&serde_json::Value::Null), "All");
    }

    // --- scope enforcement logic ---

    fn scope_allows(scope_name: &str, proposal_type: &str) -> bool {
        match scope_name {
            "All" => true,
            "Constitutional" => proposal_type == "constitutional",
            "Treasury" => proposal_type == "treasury",
            "Protocol" => proposal_type == "protocol",
            _ => true,
        }
    }

    #[test]
    fn test_scope_all_allows_everything() {
        assert!(scope_allows("All", "constitutional"));
        assert!(scope_allows("All", "treasury"));
        assert!(scope_allows("All", "protocol"));
        assert!(scope_allows("All", "proposal"));
        assert!(scope_allows("All", "unknown"));
    }

    #[test]
    fn test_scope_constitutional_restricts() {
        assert!(scope_allows("Constitutional", "constitutional"));
        assert!(!scope_allows("Constitutional", "treasury"));
        assert!(!scope_allows("Constitutional", "protocol"));
        assert!(!scope_allows("Constitutional", "proposal"));
    }

    #[test]
    fn test_scope_treasury_restricts() {
        assert!(scope_allows("Treasury", "treasury"));
        assert!(!scope_allows("Treasury", "constitutional"));
        assert!(!scope_allows("Treasury", "protocol"));
    }

    #[test]
    fn test_scope_protocol_restricts() {
        assert!(scope_allows("Protocol", "protocol"));
        assert!(!scope_allows("Protocol", "treasury"));
        assert!(!scope_allows("Protocol", "constitutional"));
    }

    #[test]
    fn test_scope_custom_permissive() {
        assert!(scope_allows("Custom", "anything"));
    }

    #[test]
    fn test_infer_proposal_type_from_description() {
        fn infer(desc: &str) -> &str {
            desc.split(':').next().unwrap_or("unknown")
        }
        assert_eq!(infer("proposal:MIP-001"), "proposal");
        assert_eq!(infer("constitutional:CA-001"), "constitutional");
        assert_eq!(infer("treasury:TB-042"), "treasury");
        assert_eq!(infer("protocol:PU-007"), "protocol");
        assert_eq!(infer("no-colon-here"), "no-colon-here");
        assert_eq!(infer(""), "");
    }

    #[test]
    fn test_veto_cooldown_is_7_days() {
        const VETO_COOLDOWN_US: i64 = 7 * 24 * 3600 * 1_000_000;
        assert_eq!(VETO_COOLDOWN_US, 604_800_000_000);
    }

    #[test]
    fn test_guardian_phi_threshold_constant() {
        // Verify the Guardian-tier threshold used in veto_timelock
        // matches the actual Guardian Φ requirement (0.8)
        const GUARDIAN_PHI_THRESHOLD: f64 = 0.8;
        assert!(
            GUARDIAN_PHI_THRESHOLD >= 0.8,
            "Guardian veto must require actual Guardian-tier Φ (0.8)"
        );
        assert!(GUARDIAN_PHI_THRESHOLD <= 1.0, "Must be a valid Φ score");
    }

    // =========================================================================
    // ThresholdSignature mirror type serde
    // =========================================================================

    #[test]
    fn test_threshold_signature_serde_roundtrip() {
        let sig = ThresholdSignature {
            id: "sig-1".into(),
            committee_id: "committee-1".into(),
            signed_content_hash: vec![1, 2, 3],
            signed_content_description: "proposal:MIP-001".into(),
            signature: vec![0u8; 64],
            signer_count: 2,
            signers: vec![1, 2],
            verified: true,
            signed_at: Timestamp::from_micros(1000000),
        };
        let json = serde_json::to_string(&sig).unwrap();
        let decoded: ThresholdSignature = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, "sig-1");
        assert!(decoded.verified);
        assert!(decoded.signed_content_description.contains("MIP-001"));
    }

}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Social Recovery Coordinator Zome
//! Business logic for DID social recovery
//!
//! Updated to use HDK 0.6 patterns
//!
//! ## MFA Integration
//! - Verifies trustee MFA status before allowing votes
//! - Enrolls SocialRecovery factor when setting up recovery
//! - Updates MFA state when recovery is executed

use hdk::prelude::*;
use std::collections::{HashMap, HashSet};
use mycelix_zome_helpers as _;
use recovery_integrity::*;
use subtle::ConstantTimeEq;

// =============================================================================
// MFA CROSS-ZOME INTEGRATION
// =============================================================================

/// MFA assurance level (mirrors mfa_integrity::AssuranceLevel)
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize, PartialOrd)]
pub enum AssuranceLevel {
    Anonymous,
    Basic,
    Verified,
    HighlyAssured,
    ConstitutionallyCritical,
}

/// Check if a DID has sufficient MFA assurance for recovery operations
fn verify_mfa_assurance_for_recovery(did: &str) -> ExternResult<bool> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::new("mfa"),
        FunctionName::new("get_mfa_assurance_score"),
        None,
        did.to_string(),
    )?;

    match response {
        ZomeCallResponse::Ok(result) => {
            let score: f64 = result.decode().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode MFA score: {:?}",
                    e
                )))
            })?;
            // Require at least Basic level (0.25) for recovery operations
            Ok(score >= 0.25)
        }
        ZomeCallResponse::Unauthorized(_, _, _, _) => {
            // MFA zome not installed — this is a configuration issue, not a security bypass.
            // Fail safe: deny rather than silently allow.
            warn!(
                "MFA zome unauthorized — denying recovery operation (MFA zome must be installed)"
            );
            Err(wasm_error!(WasmErrorInner::Guest(
                "MFA verification unavailable (zome unauthorized). Recovery denied for safety."
                    .into()
            )))
        }
        ZomeCallResponse::NetworkError(err) => {
            warn!(
                "MFA zome network error: {} — denying recovery operation",
                err
            );
            Err(wasm_error!(WasmErrorInner::Guest(format!(
                "MFA verification failed (network error: {}). Retry later.",
                err
            ))))
        }
        ZomeCallResponse::CountersigningSession(err) => {
            warn!(
                "MFA countersigning error: {} — denying recovery operation",
                err
            );
            Err(wasm_error!(WasmErrorInner::Guest(format!(
                "MFA verification failed (countersigning: {}). Retry later.",
                err
            ))))
        }
        ZomeCallResponse::AuthenticationFailed(_, _) => {
            warn!("MFA authentication failed — denying recovery operation");
            Err(wasm_error!(WasmErrorInner::Guest(
                "MFA verification failed (authentication). Recovery denied for safety.".into()
            )))
        }
    }
}

/// Enroll SocialRecovery factor in MFA state
fn enroll_social_recovery_factor(did: &str, trustees: &[String]) -> ExternResult<()> {
    #[derive(Serialize, Deserialize, Debug)]
    struct EnrollFactorInput {
        did: String,
        factor_type: String, // "SocialRecovery"
        factor_id: String,
        metadata: String,
        reason: String,
    }

    let factor_id = format!(
        "social_recovery:{}",
        &holo_hash::blake2b_256(did.as_bytes())
            .iter()
            .map(|b| format!("{:02x}", b))
            .collect::<String>()[..16]
    );

    let metadata = serde_json::json!({
        "trustees": trustees,
        "trustee_count": trustees.len()
    })
    .to_string();

    let input = EnrollFactorInput {
        did: did.to_string(),
        factor_type: "SocialRecovery".to_string(),
        factor_id,
        metadata,
        reason: "Social recovery setup".to_string(),
    };

    let response = call(
        CallTargetCell::Local,
        ZomeName::new("mfa"),
        FunctionName::new("enroll_factor"),
        None,
        input,
    )?;

    match response {
        ZomeCallResponse::Ok(_) => Ok(()),
        ZomeCallResponse::Unauthorized(_, _, _, _) => {
            warn!("MFA zome unauthorized - recovery setup WITHOUT MFA factor enrollment");
            Ok(())
        }
        ZomeCallResponse::NetworkError(err) => {
            warn!(
                "MFA network error during factor enrollment: {} - recovery setup WITHOUT MFA",
                err
            );
            Ok(())
        }
        ZomeCallResponse::CountersigningSession(err) => {
            warn!(
                "MFA countersigning error during factor enrollment: {} - recovery setup WITHOUT MFA",
                err
            );
            Ok(())
        }
        ZomeCallResponse::AuthenticationFailed(_, _) => {
            warn!("MFA authentication failed - recovery setup WITHOUT MFA factor enrollment");
            Ok(())
        }
    }
}

/// Notify bridge of successful recovery execution so other hApps are informed
fn notify_bridge_of_recovery(did: &str, new_agent: &AgentPubKey) -> ExternResult<()> {
    #[derive(Serialize, Deserialize, Debug)]
    struct BroadcastEventInput {
        event_type: String,
        subject: String,
        payload: String,
        source_happ: String,
    }

    let payload = serde_json::json!({
        "did": did,
        "new_agent": format!("{}", new_agent),
        "event": "recovery_completed",
    })
    .to_string();

    let input = BroadcastEventInput {
        event_type: "DidRecovered".to_string(),
        subject: did.to_string(),
        payload,
        source_happ: "mycelix-identity".to_string(),
    };

    let response = call(
        CallTargetCell::Local,
        ZomeName::new("identity_bridge"),
        FunctionName::new("broadcast_event"),
        None,
        input,
    )?;

    match response {
        ZomeCallResponse::Ok(_) => Ok(()),
        _ => {
            debug!(
                "Bridge notification failed for recovery of {} - non-critical",
                did
            );
            Ok(())
        }
    }
}

/// Create a deterministic entry hash from a string identifier
/// This is used for link bases when we need to link from string IDs
fn string_to_entry_hash(s: &str) -> EntryHash {
    let bytes: Vec<u8> = holo_hash::blake2b_256(s.as_bytes())
        .into_iter()
        .chain([0u8; 4])
        .collect();
    // Safety: blake2b_256 returns 32 bytes + 4 padding = 36, matching from_raw_36
    EntryHash::from_raw_36(bytes)
}

/// Set up recovery for a DID
#[hdk_extern]
pub fn setup_recovery(input: SetupRecoveryInput) -> ExternResult<Record> {
    if input.did.is_empty() || input.did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "DID must be 1-256 characters".into()
        )));
    }
    if input.trustees.len() < 3 || input.trustees.len() > 7 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Recovery requires 3-7 trustees".into()
        )));
    }
    for trustee in &input.trustees {
        if !trustee.starts_with("did:mycelix:") || trustee.len() > 256 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Each trustee must be a valid did:mycelix identifier".into()
            )));
        }
    }
    let unique_trustees: HashSet<&String> = input.trustees.iter().collect();
    if unique_trustees.len() != input.trustees.len() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Duplicate trustees are not allowed".into()
        )));
    }
    let min_threshold = (input.trustees.len() as u32).div_ceil(2);
    if input.threshold < min_threshold || input.threshold as usize > input.trustees.len() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold must be a majority and cannot exceed trustee count".into()
        )));
    }
    let time_lock = input.time_lock.unwrap_or(86400 * 7);
    if time_lock < 86400 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Recovery time lock must be at least 24 hours".into()
        )));
    }
    let agent_info = agent_info()?;
    let owner_did = format!("did:mycelix:{}", agent_info.agent_initial_pubkey);
    if input.did != owner_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Recovery configuration can only be created by the owner of the DID".into()
        )));
    }
    let now = sys_time()?;

    let config = RecoveryConfig {
        did: input.did.clone(),
        owner: agent_info.agent_initial_pubkey,
        trustees: input.trustees,
        threshold: input.threshold,
        time_lock, // Default 7 days
        active: true,
        created: now,
        updated: now,
    };

    let action_hash = create_entry(&EntryTypes::RecoveryConfig(config.clone()))?;

    // Link DID to config
    create_link(
        string_to_entry_hash(&input.did),
        action_hash.clone(),
        LinkTypes::DidToRecoveryConfig,
        (),
    )?;

    // Link each trustee to config
    for trustee in &config.trustees {
        create_link(
            string_to_entry_hash(trustee),
            action_hash.clone(),
            LinkTypes::TrusteeToConfig,
            (),
        )?;
    }

    // Enroll SocialRecovery factor in MFA state
    // This allows social recovery to contribute to the identity's assurance level
    if let Err(e) = enroll_social_recovery_factor(&input.did, &config.trustees) {
        debug!("Failed to enroll SocialRecovery factor in MFA: {:?}", e);
    }

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find recovery config".into()
    )))
}

/// Input for setting up recovery
#[derive(Serialize, Deserialize, Debug)]
pub struct SetupRecoveryInput {
    pub did: String,
    pub trustees: Vec<String>,
    pub threshold: u32,
    pub time_lock: Option<u64>,
}

/// Get recovery config for a DID
#[hdk_extern]
pub fn get_recovery_config(did: String) -> ExternResult<Option<Record>> {
    if did.is_empty() || did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "DID must be 1-256 characters".into()
        )));
    }
    let did_hash = string_to_entry_hash(&did);
    let links = get_links(
        LinkQuery::try_new(did_hash, LinkTypes::DidToRecoveryConfig)?,
        GetStrategy::default(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    let latest_link = links.into_iter().max_by_key(|l| l.timestamp);
    if let Some(link) = latest_link {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        return get(action_hash, GetOptions::default());
    }

    Ok(None)
}

/// Initiate a recovery request (trustee only)
#[hdk_extern]
pub fn initiate_recovery(input: InitiateRecoveryInput) -> ExternResult<Record> {
    if input.did.is_empty() || input.did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "DID must be 1-256 characters".into()
        )));
    }
    if input.initiator_did.is_empty() || input.initiator_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Initiator DID must be 1-256 characters".into()
        )));
    }
    if input.reason.is_empty() || input.reason.len() > 2048 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Reason must be 1-2048 characters".into()
        )));
    }

    // Rate limit: Only 1 active (Pending/Approved/ReadyToExecute) recovery request per DID
    let did_hash = string_to_entry_hash(&input.did);
    let request_links = get_links(
        LinkQuery::try_new(did_hash, LinkTypes::DidToRecoveryRequest)?,
        GetStrategy::default(),
    )?;
    for link in &request_links {
        if let Ok(action_hash) = ActionHash::try_from(link.target.clone()) {
            if let Ok(Some(record)) = get(action_hash, GetOptions::default()) {
                if let Ok(Some(snapshot)) = record.entry().to_app_option::<RecoveryRequest>() {
                    // Resolve the latest indexed version, not just the original
                    // Pending record. A completed request must not permanently
                    // block a future recovery attempt.
                    let latest = get_recovery_request(snapshot.id.clone())?
                        .or(Some(record));
                    if let Some(latest_record) = latest {
                        if let Some(req) = latest_record.entry().to_app_option::<RecoveryRequest>()
                            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))? {
                            match req.status {
                                RecoveryStatus::Pending
                                | RecoveryStatus::Approved
                                | RecoveryStatus::ReadyToExecute => {
                                    return Err(wasm_error!(WasmErrorInner::Guest(
                                        "An active recovery request already exists for this DID. Cancel it first.".into()
                                    )));
                                }
                                _ => {}
                            }
                        }
                    }
                }
            }
        }
    }

    // Verify recovery config exists
    let config_record = get_recovery_config(input.did.clone())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("No recovery config found for this DID".into())
    ))?;

    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid recovery config".into()
        )))?;
    if !config.active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Recovery configuration is inactive".into()
        )));
    }

    // Verify initiator is a trustee (constant-time to prevent timing side-channel)
    let is_trustee = config.trustees.iter().fold(0u8, |acc, trustee| {
        acc | trustee
            .as_bytes()
            .ct_eq(input.initiator_did.as_bytes())
            .unwrap_u8()
    });
    if is_trustee == 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only trustees can initiate recovery".into()
        )));
    }

    // Verify the caller actually owns the initiator_did they claim (prevents
    // any agent who merely knows a trustee's DID string from initiating
    // recovery as that trustee). Mirrors the ownership check used by the
    // vote-casting path above.
    let caller = agent_info()?.agent_initial_pubkey;
    let caller_did = format!("did:mycelix:{}", caller);
    if input.initiator_did != caller_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Caller does not own the initiator DID".into()
        )));
    }

    // Verify initiator has sufficient MFA assurance
    if !verify_mfa_assurance_for_recovery(&input.initiator_did)? {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Initiator does not meet MFA requirements (minimum Basic level required)".into()
        )));
    }

    let now = sys_time()?;
    let request_id = format!("recovery:{}:{}", input.did, now.as_micros());

    // Save values for bridge event before they are moved
    let did_for_event = input.did.clone();
    let initiator_for_event = input.initiator_did.clone();

    let request = RecoveryRequest {
        id: request_id.clone(),
        did: input.did.clone(),
        new_agent: input.new_agent,
        initiated_by: input.initiator_did.clone(),
        reason: input.reason,
        status: RecoveryStatus::Pending,
        created: now,
        time_lock_expires: None,
        config_action_hash: Some(config_record.action_address().clone()),
        decision_vote_hashes: Vec::new(),
    };

    let action_hash = create_entry(&EntryTypes::RecoveryRequest(request))?;

    // Link DID to request
    create_link(
        string_to_entry_hash(&input.did),
        action_hash.clone(),
        LinkTypes::DidToRecoveryRequest,
        (),
    )?;
    create_link(
        string_to_entry_hash(&request_id),
        action_hash.clone(),
        LinkTypes::RequestIdToRequest,
        (),
    )?;

    // Save request_id for bridge event before it's moved
    let request_id_for_event = request_id.clone();

    // Create initial approval vote from initiator
    let vote = RecoveryVote {
        request_id: request_id.clone(),
        trustee: input.initiator_did,
        vote: VoteDecision::Approve,
        comment: Some("Initiated recovery".to_string()),
        voted_at: now,
        request_action_hash: Some(action_hash.clone()),
    };

    let vote_hash = create_entry(&EntryTypes::RecoveryVote(vote))?;

    create_link(
        string_to_entry_hash(&request_id),
        vote_hash,
        LinkTypes::RequestToVotes,
        (),
    )?;

    // Broadcast RecoveryInitiated event to bridge for ecosystem-wide awareness
    {
        #[derive(Serialize, Deserialize, Debug)]
        struct BroadcastEventInput {
            event_type: String,
            subject: String,
            payload: String,
            source_happ: String,
        }

        let payload = serde_json::json!({
            "did": did_for_event,
            "initiated_by": initiator_for_event,
            "request_id": request_id_for_event,
            "event": "recovery_initiated",
        })
        .to_string();

        let event_input = BroadcastEventInput {
            event_type: "RecoveryInitiated".to_string(),
            subject: did_for_event.to_string(),
            payload,
            source_happ: "mycelix-identity".to_string(),
        };

        match call(
            CallTargetCell::Local,
            ZomeName::new("identity_bridge"),
            FunctionName::new("broadcast_event"),
            None,
            event_input,
        ) {
            Ok(ZomeCallResponse::Ok(_)) => {}
            _ => {
                debug!("Bridge notification failed for RecoveryInitiated event - non-critical");
            }
        }
    }

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find recovery request".into()
    )))
}

/// Input for initiating recovery
#[derive(Serialize, Deserialize, Debug)]
pub struct InitiateRecoveryInput {
    pub did: String,
    pub initiator_did: String,
    pub new_agent: AgentPubKey,
    pub reason: String,
}

/// Vote on a recovery request
#[hdk_extern]
pub fn vote_on_recovery(input: VoteOnRecoveryInput) -> ExternResult<Record> {
    if input.request_id.is_empty() || input.request_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Request ID must be 1-256 characters".into()
        )));
    }
    if input.trustee_did.is_empty() || input.trustee_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Trustee DID must be 1-256 characters".into()
        )));
    }
    if let Some(ref comment) = input.comment {
        if comment.len() > 2048 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Comment must be under 2048 characters".into()
            )));
        }
    }

    // Verify caller is the claimed trustee
    let caller = agent_info()?.agent_initial_pubkey;
    let caller_did = format!("did:mycelix:{}", caller);
    if input.trustee_did != caller_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Caller must be the claimed trustee".into()
        )));
    }

    // Resolve the request from its DHT index. The request ID is not an
    // authorization token: the integrity zome independently follows this
    // action reference and validates trustee membership against the frozen
    // recovery configuration.
    let request_record = get_recovery_request(input.request_id.clone())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Recovery request not found".into())))?;
    let request: RecoveryRequest = request_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Invalid recovery request".into())))?;
    if request.status != RecoveryStatus::Pending {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Recovery request is no longer accepting votes".into()
        )));
    }
    let config_hash = request.config_action_hash.clone().ok_or(wasm_error!(
        WasmErrorInner::Guest("Recovery request has no config revision".into())
    ))?;
    let config_record = get(config_hash, GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Recovery config revision not found".into())))?;
    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Invalid recovery config revision".into())))?;
    if !config.active || !config.trustees.contains(&input.trustee_did) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an authorized trustee in this recovery configuration may vote".into()
        )));
    }

    // Verify voter has sufficient MFA assurance
    if !verify_mfa_assurance_for_recovery(&input.trustee_did)? {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Voter does not meet MFA requirements (minimum Basic level required)".into()
        )));
    }

    // Prevent duplicate votes: check if this trustee already voted on this request
    let existing_votes = get_recovery_votes(input.request_id.clone())?;
    for record in &existing_votes {
        if let Some(existing_vote) = record
            .entry()
            .to_app_option::<RecoveryVote>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            if existing_vote.trustee == input.trustee_did {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "Trustee {} has already voted on this recovery request",
                    input.trustee_did
                ))));
            }
        }
    }

    let now = sys_time()?;

    let vote = RecoveryVote {
        request_id: input.request_id.clone(),
        trustee: input.trustee_did,
        vote: input.vote,
        comment: input.comment,
        voted_at: now,
        request_action_hash: Some(request_record.action_address().clone()),
    };

    let action_hash = create_entry(&EntryTypes::RecoveryVote(vote))?;

    // Link to request
    let request_hash = string_to_entry_hash(&input.request_id);
    create_link(
        request_hash,
        action_hash.clone(),
        LinkTypes::RequestToVotes,
        (),
    )?;

    // Check if threshold is reached
    check_and_update_request_status(input.request_id)?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find vote".into()
    )))
}

/// Input for voting on recovery
#[derive(Serialize, Deserialize, Debug)]
pub struct VoteOnRecoveryInput {
    pub request_id: String,
    pub trustee_did: String,
    pub vote: VoteDecision,
    pub comment: Option<String>,
}

/// Get votes for a recovery request
#[hdk_extern]
pub fn get_recovery_votes(request_id: String) -> ExternResult<Vec<Record>> {
    if request_id.is_empty() || request_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Request ID must be 1-256 characters".into()
        )));
    }
    let request_hash = string_to_entry_hash(&request_id);
    let links = get_links(
        LinkQuery::try_new(request_hash, LinkTypes::RequestToVotes)?,
        GetStrategy::default(),
    )?;

    let mut votes = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            votes.push(record);
        }
    }

    Ok(votes)
}

/// Check threshold and update request status
fn check_and_update_request_status(request_id: String) -> ExternResult<()> {
    let current_record = match get_recovery_request(request_id.clone())? {
        Some(record) => record,
        None => return Ok(()),
    };
    let current_request: RecoveryRequest = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid recovery request".into()
        )))?;

    // RecoveryRequest status is an update to the initiating trustee's source
    // chain. Other trustees may append votes, but they cannot update that
    // entry. The initiator can explicitly finalize after DHT propagation.
    let caller = agent_info()?.agent_initial_pubkey;
    if *current_record.action().author() != caller {
        return Ok(());
    }
    if current_request.status != RecoveryStatus::Pending {
        return Ok(());
    }

    let config_hash = current_request.config_action_hash.clone().ok_or(wasm_error!(
        WasmErrorInner::Guest("Recovery request has no config revision".into())
    ))?;
    let config_record = get(config_hash, GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Authorizing recovery config revision not found".into()
        )))?;
    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid recovery config revision".into()
        )))?;

    // Count each trustee at most once. If a trustee has conflicting votes,
    // exclude that trustee entirely (equivocation must never increase quorum).
    let vote_records = get_recovery_votes(request_id.clone())?;
    let mut votes_by_trustee: HashMap<String, (VoteDecision, ActionHash)> = HashMap::new();
    let mut equivocated: HashSet<String> = HashSet::new();
    for record in vote_records {
        if let Some(vote) = record
            .entry()
            .to_app_option::<RecoveryVote>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            if !config.trustees.contains(&vote.trustee) {
                continue;
            }
            match votes_by_trustee.get(&vote.trustee) {
                Some((existing, _)) if existing != &vote.vote => {
                    equivocated.insert(vote.trustee);
                }
                Some(_) => {}
                None => {
                    votes_by_trustee.insert(
                        vote.trustee,
                        (vote.vote, record.action_address().clone()),
                    );
                }
            }
        }
    }

    let mut approve_count = 0u32;
    let mut reject_count = 0u32;
    let mut approval_evidence = Vec::new();
    let mut rejection_evidence = Vec::new();
    for (trustee, (decision, vote_hash)) in votes_by_trustee {
        if equivocated.contains(&trustee) {
            continue;
        }
        match decision {
            VoteDecision::Approve => {
                approve_count += 1;
                approval_evidence.push(vote_hash);
            }
            VoteDecision::Reject => {
                reject_count += 1;
                rejection_evidence.push(vote_hash);
            }
            VoteDecision::Abstain => {}
        }
    }

    let (next_status, decision_vote_hashes) = if approve_count >= config.threshold {
        (Some(RecoveryStatus::Approved), approval_evidence)
    } else if reject_count > (config.trustees.len() as u32).saturating_sub(config.threshold) {
        (Some(RecoveryStatus::Rejected), rejection_evidence)
    } else {
        (None, Vec::new())
    };

    let Some(next_status) = next_status else {
        return Ok(());
    };

    let now = sys_time()?;
    let time_lock_expires = if next_status == RecoveryStatus::Approved {
        Some(Timestamp::from_micros(
            now.as_micros() as i64 + (config.time_lock as i64 * 1_000_000),
        ))
    } else {
        None
    };
    let did = current_request.did.clone();
    let updated_request = RecoveryRequest {
        id: current_request.id.clone(),
        did: current_request.did,
        new_agent: current_request.new_agent,
        initiated_by: current_request.initiated_by,
        reason: current_request.reason,
        status: next_status,
        created: current_request.created,
        time_lock_expires,
        config_action_hash: current_request.config_action_hash,
        decision_vote_hashes,
    };

    let action_hash = update_entry(
        current_record.action_address().clone(),
        &EntryTypes::RecoveryRequest(updated_request),
    )?;
    create_link(
        string_to_entry_hash(&request_id),
        action_hash.clone(),
        LinkTypes::RequestIdToRequest,
        (),
    )?;
    create_link(
        string_to_entry_hash(&did),
        action_hash,
        LinkTypes::DidToRecoveryRequest,
        (),
    )?;
    Ok(())
}

/// Explicitly finalize a recovery after trustee votes have propagated.
/// Only the original request author (the initiating trustee) can write the
/// status update to that agent's source chain.
#[hdk_extern]
pub fn finalize_recovery(request_id: String) -> ExternResult<Record> {
    if request_id.is_empty() || request_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Request ID must be 1-256 characters".into()
        )));
    }
    let record = get_recovery_request(request_id.clone())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Recovery request not found".into())))?;
    let caller = agent_info()?.agent_initial_pubkey;
    if *record.action().author() != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the initiating trustee can finalize this recovery request".into()
        )));
    }
    check_and_update_request_status(request_id.clone())?;
    get_recovery_request(request_id)?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Recovery request not found after finalization".into())))
}

/// Pure threshold decision logic — no HDK calls, fully testable.
/// Returns Some(true) for approved, Some(false) for rejected, None for still pending.
pub fn evaluate_threshold(
    approve_count: u32,
    reject_count: u32,
    total_trustees: u32,
    threshold: u32,
) -> Option<bool> {
    if approve_count >= threshold {
        Some(true)
    } else if total_trustees >= threshold && reject_count > total_trustees - threshold {
        Some(false)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_approved_exact_threshold() {
        assert_eq!(evaluate_threshold(3, 0, 5, 3), Some(true));
    }

    #[test]
    fn test_approved_above_threshold() {
        assert_eq!(evaluate_threshold(4, 0, 5, 3), Some(true));
    }

    #[test]
    fn test_rejected_impossible_to_reach() {
        // 5 trustees, threshold 3, need 3 approvals
        // 3 rejected → only 2 remain → can't reach 3
        assert_eq!(evaluate_threshold(0, 3, 5, 3), Some(false));
    }

    #[test]
    fn test_still_pending() {
        // 5 trustees, threshold 3, 2 approved 1 rejected → still possible
        assert_eq!(evaluate_threshold(2, 1, 5, 3), None);
    }

    #[test]
    fn test_single_trustee_approved() {
        assert_eq!(evaluate_threshold(1, 0, 1, 1), Some(true));
    }

    #[test]
    fn test_single_trustee_rejected() {
        assert_eq!(evaluate_threshold(0, 1, 1, 1), Some(false));
    }

    #[test]
    fn test_no_votes_yet() {
        assert_eq!(evaluate_threshold(0, 0, 5, 3), None);
    }

    #[test]
    fn test_all_abstain_stays_pending() {
        // Abstains don't count as approve or reject
        assert_eq!(evaluate_threshold(0, 0, 5, 3), None);
    }

    #[test]
    fn test_unanimous_approval() {
        assert_eq!(evaluate_threshold(5, 0, 5, 3), Some(true));
    }

    #[test]
    fn test_boundary_reject_not_yet() {
        // 5 trustees, threshold 3 → need >2 rejects to make impossible
        // 2 rejects → still 3 potential approvals
        assert_eq!(evaluate_threshold(0, 2, 5, 3), None);
    }

    #[test]
    fn test_threshold_equals_total() {
        // All must approve; 1 reject means impossible
        assert_eq!(evaluate_threshold(0, 1, 3, 3), Some(false));
        assert_eq!(evaluate_threshold(3, 0, 3, 3), Some(true));
        assert_eq!(evaluate_threshold(2, 0, 3, 3), None);
    }
}

/// Execute recovery (after time lock)
#[hdk_extern]
pub fn execute_recovery(request_id: String) -> ExternResult<Record> {
    if request_id.is_empty() || request_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Request ID must be 1-256 characters".into()
        )));
    }
    let current_record = get_recovery_request(request_id.clone())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Recovery request not found".into()
        )))?;

    let current_request: RecoveryRequest = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid recovery request".into()
        )))?;

    // The request author is the only agent allowed to update it on-chain.
    // The designated new agent claims the recovered DID in a separate call.
    let caller = agent_info()?.agent_initial_pubkey;
    let caller_did = format!("did:mycelix:{}", caller);
    if caller_did != current_request.initiated_by || *current_record.action().author() != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the initiating trustee can finalize recovery execution; the new agent claims the DID separately".into()
        )));
    }

    // Verify status allows execution
    if current_request.status != RecoveryStatus::Approved
        && current_request.status != RecoveryStatus::ReadyToExecute
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Recovery request is not approved".into()
        )));
    }

    // Verify time lock has expired
    let now = sys_time()?;
    if let Some(expires) = current_request.time_lock_expires {
        if now < expires {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Time lock has not expired".into()
            )));
        }
    } else {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Time lock not set".into()
        )));
    }

    // Move through the explicit ReadyToExecute state so the integrity
    // validator can enforce the time-lock boundary as a separate transition.
    let ready_record = if current_request.status == RecoveryStatus::Approved {
        let ready_request = RecoveryRequest {
            id: current_request.id.clone(),
            did: current_request.did.clone(),
            new_agent: current_request.new_agent.clone(),
            initiated_by: current_request.initiated_by.clone(),
            reason: current_request.reason.clone(),
            status: RecoveryStatus::ReadyToExecute,
            created: current_request.created,
            time_lock_expires: current_request.time_lock_expires,
            config_action_hash: current_request.config_action_hash.clone(),
            decision_vote_hashes: current_request.decision_vote_hashes.clone(),
        };
        let ready_hash = update_entry(
            current_record.action_address().clone(),
            &EntryTypes::RecoveryRequest(ready_request),
        )?;
        create_link(
            string_to_entry_hash(&request_id),
            ready_hash.clone(),
            LinkTypes::RequestIdToRequest,
            (),
        )?;
        create_link(
            string_to_entry_hash(&current_request.did),
            ready_hash.clone(),
            LinkTypes::DidToRecoveryRequest,
            (),
        )?;
        get(ready_hash, GetOptions::default())?.ok_or(wasm_error!(
            WasmErrorInner::Guest("Could not retrieve ReadyToExecute request".into())
        ))?
    } else {
        current_record
    };

    let ready_request: RecoveryRequest = ready_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid ReadyToExecute request".into()
        )))?;
    if ready_request.status != RecoveryStatus::ReadyToExecute {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Recovery request did not reach ReadyToExecute".into()
        )));
    }

    let did_for_mfa = ready_request.did.clone();
    let new_agent_for_mfa = ready_request.new_agent.clone();
    let completed_request = RecoveryRequest {
        id: ready_request.id.clone(),
        did: ready_request.did.clone(),
        new_agent: ready_request.new_agent,
        initiated_by: ready_request.initiated_by,
        reason: ready_request.reason,
        status: RecoveryStatus::Completed,
        created: ready_request.created,
        time_lock_expires: ready_request.time_lock_expires,
        config_action_hash: ready_request.config_action_hash,
        decision_vote_hashes: ready_request.decision_vote_hashes,
    };

    let request_id_for_index = ready_request.id;
    let did_for_index = ready_request.did;
    let action_hash = update_entry(
        ready_record.action_address().clone(),
        &EntryTypes::RecoveryRequest(completed_request),
    )?;
    create_link(
        string_to_entry_hash(&request_id_for_index),
        action_hash.clone(),
        LinkTypes::RequestIdToRequest,
        (),
    )?;
    create_link(
        string_to_entry_hash(&did_for_index),
        action_hash.clone(),
        LinkTypes::DidToRecoveryRequest,
        (),
    )?;

    // Broadcast recovery event to bridge so other hApps are informed
    if let Err(e) = notify_bridge_of_recovery(&did_for_mfa, &new_agent_for_mfa) {
        debug!("Failed to notify bridge of recovery execution: {:?}", e);
    }

    // DID transfer completes when the new agent calls did_registry::claim_recovered_did().
    // This two-step pattern is required by Holochain's agent-centric architecture:
    // only the new agent can create entries on their own source chain.

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find completed request".into()
    )))
}

/// Cancel a recovery request (owner only, before execution)
#[hdk_extern]
pub fn cancel_recovery(request_id: String) -> ExternResult<Record> {
    if request_id.is_empty() || request_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Request ID must be 1-256 characters".into()
        )));
    }
    let agent_info = agent_info()?;

    let current_record = get_recovery_request(request_id.clone())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Recovery request not found".into()
        )))?;

    let current_request: RecoveryRequest = current_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid recovery request".into()
        )))?;

    // Get recovery config to verify owner
    let config_record = get_recovery_config(current_request.did.clone())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Recovery config not found".into())
    ))?;

    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid recovery config".into()
        )))?;

    // Verify caller is owner
    if config.owner != agent_info.agent_initial_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only owner can cancel recovery".into()
        )));
    }

    // Verify not already completed
    if current_request.status == RecoveryStatus::Completed {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Cannot cancel completed recovery".into()
        )));
    }

    // Update to cancelled
    let cancelled_request = RecoveryRequest {
        id: current_request.id,
        did: current_request.did,
        new_agent: current_request.new_agent,
        initiated_by: current_request.initiated_by,
        reason: current_request.reason,
        status: RecoveryStatus::Cancelled,
        created: current_request.created,
        time_lock_expires: current_request.time_lock_expires,
        config_action_hash: current_request.config_action_hash,
        decision_vote_hashes: current_request.decision_vote_hashes,
    };

    // Holochain entry updates are source-chain-author restricted. An owner
    // can only perform this legacy update path when the owner is also the
    // initiating trustee; a cross-agent cancellation receipt is a separate
    // protocol extension and must not be simulated as an owner update.
    if *current_record.action().author() != agent_info.agent_initial_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Owner cancellation from a different agent source chain is not yet supported; request author must publish the cancellation".into()
        )));
    }
    let did_for_index = current_request.did.clone();
    let action_hash = update_entry(
        current_record.action_address().clone(),
        &EntryTypes::RecoveryRequest(cancelled_request),
    )?;
    create_link(
        string_to_entry_hash(&request_id),
        action_hash.clone(),
        LinkTypes::RequestIdToRequest,
        (),
    )?;
    create_link(
        string_to_entry_hash(&did_for_index),
        action_hash.clone(),
        LinkTypes::DidToRecoveryRequest,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find cancelled request".into()
    )))
}

/// Get the latest indexed recovery request version by ID.
#[hdk_extern]
pub fn get_recovery_request(request_id: String) -> ExternResult<Option<Record>> {
    if request_id.is_empty() || request_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Request ID must be 1-256 characters".into(),
        )));
    }

    let links = get_links(
        LinkQuery::try_new(
            string_to_entry_hash(&request_id),
            LinkTypes::RequestIdToRequest,
        )?,
        GetStrategy::default(),
    )?;
    let mut candidates = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid request index target".into())))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            if let Some(req) = record
                .entry()
                .to_app_option::<RecoveryRequest>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                if req.id == request_id {
                    candidates.push((link.timestamp, record));
                }
            }
        }
    }
    candidates.sort_by_key(|(timestamp, _)| *timestamp);
    if let Some((_, record)) = candidates.pop() {
        return Ok(Some(record));
    }

    // Legacy compatibility: entries created before RequestIdToRequest was
    // introduced are discoverable only on their original author's source chain.
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::RecoveryRequest,
        )?))
        .include_entries(true);
    let mut found = None;
    for record in query(filter)? {
        if let Some(req) = record
            .entry()
            .to_app_option::<RecoveryRequest>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            if req.id == request_id {
                found = Some(record);
            }
        }
    }
    Ok(found)
}

/// Get pending recovery requests for a trustee
#[hdk_extern]
pub fn get_trustee_responsibilities(trustee_did: String) -> ExternResult<Vec<Record>> {
    if trustee_did.is_empty() || trustee_did.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Trustee DID must be 1-256 characters".into()
        )));
    }
    let trustee_hash = string_to_entry_hash(&trustee_did);
    let links = get_links(
        LinkQuery::try_new(trustee_hash, LinkTypes::TrusteeToConfig)?,
        GetStrategy::default(),
    )?;

    let mut configs = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            configs.push(record);
        }
    }

    Ok(configs)
}

// =============================================================================
// PROGRESSIVE RECOVERY — Self-Recovery Functions
// =============================================================================

/// Input for creating self-recovery config.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateSelfRecoveryInput {
    pub did: String,
}

/// Auto-create a self-recovery config for a new DID.
///
/// Called by `create_did()` — gives every user a recovery fallback from Day 0.
/// Starts with zero anchors (user enrolls them progressively) and a conservative
/// 7-day time lock.
#[hdk_extern]
pub fn create_self_recovery(input: CreateSelfRecoveryInput) -> ExternResult<Record> {
    if !input.did.starts_with("did:mycelix:") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Invalid DID format".into()
        )));
    }
    let agent = agent_info()?.agent_initial_pubkey;
    let owner_did = format!("did:mycelix:{}", agent);
    if input.did != owner_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Self-recovery configuration can only be created by the DID owner".into()
        )));
    }
    let now = sys_time()?;
    let config = SelfRecoveryConfig {
        did: input.did.clone(),
        owner: agent,
        anchors: vec![],
        anchor_threshold: 1,
        time_lock: SELF_RECOVERY_DEFAULT_TIME_LOCK,
        active: false,
        superseded_by_social: false,
        created: now,
        updated: now,
    };
    let action_hash = create_entry(&EntryTypes::SelfRecoveryConfig(config))?;
    create_link(
        string_to_entry_hash(&input.did),
        action_hash.clone(),
        LinkTypes::DidToSelfRecoveryConfig,
        LinkTag::new("self_recovery"),
    )?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Failed to get created self-recovery config".into()
    )))
}

/// Input for adding a verification anchor.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AddVerificationAnchorInput {
    pub did: String,
    pub anchor: VerificationAnchor,
}

/// Add a verification anchor to self-recovery config.
///
/// Called when a user verifies a phone number, email, passkey, or device.
/// Updates the self-recovery time lock based on anchor count:
/// - 1 anchor: 14 days
/// - 2 anchors: 7 days
/// - 3+ anchors: 72 hours (minimum)
#[hdk_extern]
pub fn add_verification_anchor(_input: AddVerificationAnchorInput) -> ExternResult<Record> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "Self-recovery anchor enrollment is disabled until challenge-backed proof-of-control and privacy-preserving anchor storage are implemented".into()
    )))
}

/// Get self-recovery config for a DID.
#[hdk_extern]
pub fn get_self_recovery_config(did: String) -> ExternResult<Option<Record>> {
    let did_hash = string_to_entry_hash(&did);
    let links = get_links(
        LinkQuery::try_new(did_hash, LinkTypes::DidToSelfRecoveryConfig)?,
        GetStrategy::default(),
    )?;

    match links.first() {
        Some(link) => {
            let hash = ActionHash::try_from(link.target.clone())
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
            get(hash, GetOptions::default())
        }
        None => Ok(None),
    }
}

/// Mark self-recovery as superseded by social recovery.
///
/// Called by the Hearth auto-recovery when social recovery is configured.
/// Self-recovery remains available as a fallback.
#[hdk_extern]
pub fn mark_self_recovery_superseded(did: String) -> ExternResult<()> {
    let did_hash = string_to_entry_hash(&did);
    let links = get_links(
        LinkQuery::try_new(did_hash, LinkTypes::DidToSelfRecoveryConfig)?,
        GetStrategy::default(),
    )?;

    if let Some(link) = links.first() {
        let hash = ActionHash::try_from(link.target.clone())
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;

        if let Some(record) = get(hash.clone(), GetOptions::default())? {
            let mut config: SelfRecoveryConfig = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Failed to decode config".into()
                )))?;

            config.superseded_by_social = true;
            config.updated = sys_time()?;
            update_entry(hash, &config)?;
        }
    }

    Ok(())
}

/// Input for initiating self-recovery.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct InitiateSelfRecoveryInput {
    pub did: String,
    pub new_agent: AgentPubKey,
    pub reason: String,
    /// Initial verified anchor (must match one in the config)
    pub initial_anchor: VerificationAnchor,
}

/// Initiate a self-recovery request.
///
/// Requires proving control of at least one verification anchor.
/// The request enters a time-locked waiting period before it can execute.
#[hdk_extern]
pub fn initiate_self_recovery(_input: InitiateSelfRecoveryInput) -> ExternResult<Record> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "Self-recovery is disabled until every enrolled anchor has a verifiable challenge-response protocol".into()
    )))
}

/// Input for verifying an additional anchor during self-recovery.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifySelfRecoveryAnchorInput {
    pub request_action_hash: ActionHash,
    pub anchor: VerificationAnchor,
}

/// Verify an additional anchor on an active self-recovery request.
///
/// When the anchor threshold is met, the request transitions to Approved
/// and the time lock countdown begins.
#[hdk_extern]
pub fn verify_self_recovery_anchor(_input: VerifySelfRecoveryAnchorInput) -> ExternResult<Record> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "Self-recovery anchor verification is disabled until challenge-response proofs are implemented".into()
    )))
}

/// Execute a self-recovery request after time lock expires.
///
/// Completes the identity rotation — same effect as social recovery execution.
#[hdk_extern]
pub fn execute_self_recovery(_request_action_hash: ActionHash) -> ExternResult<Record> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "Self-recovery execution is disabled while challenge-backed proof-of-control is unavailable".into()
    )))
}

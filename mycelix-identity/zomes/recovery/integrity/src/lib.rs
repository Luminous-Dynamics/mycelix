// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Social Recovery Integrity Zome
//! Defines entry types and validation for DID social recovery
//!
//! Updated to use HDI 0.7 patterns with FlatOp validation

use hdi::prelude::*;
use std::collections::HashSet;

/// Recovery configuration for a DID
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RecoveryConfig {
    /// The DID being protected
    pub did: String,
    /// Owner's agent pub key
    pub owner: AgentPubKey,
    /// List of trustee DIDs
    pub trustees: Vec<String>,
    /// Minimum trustees required (threshold)
    pub threshold: u32,
    /// Time lock in seconds before recovery executes
    pub time_lock: u64,
    /// Whether recovery is currently active
    pub active: bool,
    /// Creation timestamp
    pub created: Timestamp,
    /// Last update timestamp
    pub updated: Timestamp,
}

/// A recovery request initiated by trustees
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RecoveryRequest {
    /// Request identifier
    pub id: String,
    /// DID being recovered
    pub did: String,
    /// New agent pub key to recover to
    pub new_agent: AgentPubKey,
    /// Initiating trustee's DID
    pub initiated_by: String,
    /// Reason for recovery
    pub reason: String,
    /// Current status
    pub status: RecoveryStatus,
    /// When the request was created
    pub created: Timestamp,
    /// When time lock expires (if approved)
    pub time_lock_expires: Option<Timestamp>,
    /// Exact recovery configuration revision authorizing this request.
    /// Optional in the Rust type for decoding historical entries; required
    /// for every newly-created request by integrity validation.
    #[serde(default)]
    pub config_action_hash: Option<ActionHash>,
    /// Vote records that independently justify an Approved/Rejected transition.
    #[serde(default)]
    pub decision_vote_hashes: Vec<ActionHash>,
}

/// Owner-signed cancellation receipt. This is an independent DHT event,
/// not an update to another agent's source-chain entry.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RecoveryCancellation {
    pub request_id: String,
    pub did: String,
    pub request_action_hash: ActionHash,
    pub owner: AgentPubKey,
    pub reason: String,
    pub cancelled_at: Timestamp,
}

/// Trustee vote on a recovery request
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RecoveryVote {
    /// Recovery request ID
    pub request_id: String,
    /// Voting trustee's DID
    pub trustee: String,
    /// Vote decision
    pub vote: VoteDecision,
    /// Optional comment
    pub comment: Option<String>,
    /// Vote timestamp
    pub voted_at: Timestamp,
    /// Exact RecoveryRequest action being voted on.
    /// Optional only to keep historical records decodable; required for new votes.
    #[serde(default)]
    pub request_action_hash: Option<ActionHash>,
}

/// Status of a recovery request
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RecoveryStatus {
    /// Waiting for trustee votes
    Pending,
    /// Threshold reached, in time lock period
    Approved,
    /// Time lock expired, can execute
    ReadyToExecute,
    /// Recovery completed
    Completed,
    /// Recovery was rejected
    Rejected,
    /// Recovery was cancelled by owner
    Cancelled,
}

/// Trustee vote decision
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum VoteDecision {
    Approve,
    Reject,
    Abstain,
}

// =============================================================================
// Progressive Recovery — Self-Recovery Types
// =============================================================================

/// A verification anchor for self-recovery — a hashed external identifier
/// that the user proves control of to authorize recovery.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum VerificationAnchor {
    /// SHA-256 hash of normalized phone number (E.164 format)
    PhoneHash(String),
    /// SHA-256 hash of normalized email address (lowercase, trimmed)
    EmailHash(String),
    /// WebAuthn credential ID for passkey-based recovery
    PasskeyCredentialId(String),
    /// Device attestation hash (TPM/Secure Enclave binding)
    DeviceAttestation(String),
    /// Biometric template hash (privacy-preserving, never raw biometrics)
    BiometricHash(String),
}

/// Minimum self-recovery time lock: 72 hours (vs 24h for social recovery).
/// Longer because no humans are independently verifying the request.
pub const SELF_RECOVERY_MIN_TIME_LOCK: u64 = 72 * 3600;

/// Default time lock: 7 days (conservative for zero-anchor configs).
pub const SELF_RECOVERY_DEFAULT_TIME_LOCK: u64 = 7 * 24 * 3600;

/// Self-recovery configuration — created automatically at DID creation.
/// Recovery is time-locked and requires proving control of enrolled anchors.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SelfRecoveryConfig {
    /// The DID being protected
    pub did: String,
    /// Owner's agent pub key
    pub owner: AgentPubKey,
    /// Enrolled verification anchors (hashed — never stores raw identifiers)
    pub anchors: Vec<VerificationAnchor>,
    /// Minimum anchors required to initiate self-recovery
    pub anchor_threshold: u32,
    /// Time lock in seconds before self-recovery executes (minimum 72 hours)
    pub time_lock: u64,
    /// Whether self-recovery is active
    pub active: bool,
    /// True when social recovery has been configured (self-recovery becomes fallback)
    pub superseded_by_social: bool,
    /// Creation timestamp
    pub created: Timestamp,
    /// Last update timestamp
    pub updated: Timestamp,
}

/// A self-recovery request — initiated by proving control of verification anchors.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SelfRecoveryRequest {
    /// Request identifier
    pub id: String,
    /// DID being recovered
    pub did: String,
    /// New agent pub key to recover to
    pub new_agent: AgentPubKey,
    /// Anchors that have been verified so far
    pub verified_anchors: Vec<VerificationAnchor>,
    /// Current status (reuses existing state machine)
    pub status: RecoveryStatus,
    /// When the request was created
    pub created: Timestamp,
    /// When time lock expires (set when anchor threshold met)
    pub time_lock_expires: Option<Timestamp>,
    /// Reason for recovery
    pub reason: String,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    RecoveryConfig(RecoveryConfig),
    RecoveryRequest(RecoveryRequest),
    RecoveryVote(RecoveryVote),
    RecoveryCancellation(RecoveryCancellation),
    SelfRecoveryConfig(SelfRecoveryConfig),
    SelfRecoveryRequest(SelfRecoveryRequest),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// DID to social recovery config
    DidToRecoveryConfig,
    /// DID to social recovery requests
    DidToRecoveryRequest,
    /// Recovery request to votes
    RequestToVotes,
    /// Request identifier to all versions of its RecoveryRequest entry
    RequestIdToRequest,
    /// Request identifier to owner-signed cancellation receipts
    RequestToCancellations,
    /// Trustee to their responsibilities
    TrusteeToConfig,
    /// DID to self-recovery config (progressive recovery)
    DidToSelfRecoveryConfig,
    /// DID to self-recovery requests
    DidToSelfRecoveryRequest,
}

/// Genesis self-check - called when app is installed
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern matching
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::RecoveryConfig(config) => {
                    validate_create_recovery_config(EntryCreationAction::Create(action), config)
                }
                EntryTypes::RecoveryRequest(request) => {
                    validate_create_recovery_request(EntryCreationAction::Create(action), request)
                }
                EntryTypes::RecoveryVote(vote) => {
                    validate_create_recovery_vote(EntryCreationAction::Create(action), vote)
                }
                EntryTypes::RecoveryCancellation(cancellation) => {
                    validate_create_recovery_cancellation(
                        EntryCreationAction::Create(action),
                        cancellation,
                    )
                }
                EntryTypes::SelfRecoveryConfig(config) => validate_create_self_recovery_config(
                    EntryCreationAction::Create(action),
                    config,
                ),
                EntryTypes::SelfRecoveryRequest(request) => {
                    validate_create_self_recovery_request(request)
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::RecoveryConfig(config) => {
                    validate_update_recovery_config(action, config)
                }
                EntryTypes::RecoveryRequest(request) => {
                    validate_update_recovery_request(action, request)
                }
                EntryTypes::RecoveryVote(_) => Ok(ValidateCallbackResult::Invalid(
                    "Recovery votes cannot be updated".into(),
                )),
                EntryTypes::RecoveryCancellation(_) => Ok(ValidateCallbackResult::Invalid(
                    "Recovery cancellation receipts cannot be updated".into(),
                )),
                EntryTypes::SelfRecoveryConfig(config) => {
                    validate_update_self_recovery_config(action, config)
                }
                EntryTypes::SelfRecoveryRequest(request) => {
                    validate_update_self_recovery_request(action, request)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, tag, .. } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            match link_type {
                LinkTypes::DidToRecoveryConfig
                | LinkTypes::DidToRecoveryRequest
                | LinkTypes::RequestToVotes
                | LinkTypes::RequestIdToRequest
                | LinkTypes::RequestToCancellations
                | LinkTypes::TrusteeToConfig
                | LinkTypes::DidToSelfRecoveryConfig
                | LinkTypes::DidToSelfRecoveryRequest => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            ..
        } => {
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the link creator can delete their links".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can update their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Canonical DID binding for this deployment: the identifier suffix is
/// the Holochain agent public key that authored the identity record.
fn recovery_config_matches_owner_did(did: &str, owner: &AgentPubKey) -> bool {
    did == format!("did:mycelix:{}", owner)
}

/// Verify that a recovery request is authorized by the frozen policy revision.
fn recovery_request_matches_config(
    request: &RecoveryRequest,
    config: &RecoveryConfig,
    author: &AgentPubKey,
) -> bool {
    let author_did = format!("did:mycelix:{}", author);
    config.did == request.did
        && config.active
        && config.trustees.contains(&request.initiated_by)
        && request.initiated_by == author_did
}

/// Verify signer binding, request binding, and trustee membership independent
/// of coordinator behavior.
fn recovery_vote_matches_policy(
    vote: &RecoveryVote,
    request: &RecoveryRequest,
    config: &RecoveryConfig,
    author: &AgentPubKey,
) -> bool {
    let author_did = format!("did:mycelix:{}", author);
    vote.request_id == request.id
        && vote.trustee == author_did
        && config.did == request.did
        && config.active
        && config.trustees.contains(&vote.trustee)
}

/// Validate an owner cancellation receipt against the frozen recovery
/// configuration and current request state.
fn validate_create_recovery_cancellation(
    action: EntryCreationAction,
    cancellation: RecoveryCancellation,
) -> ExternResult<ValidateCallbackResult> {
    if cancellation.request_id.is_empty()
        || cancellation.request_id.len() > 256
        || cancellation.reason.is_empty()
        || cancellation.reason.len() > 2048
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Cancellation request ID and reason must be non-empty and bounded".into(),
        ));
    }
    if cancellation.owner != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Cancellation owner must be the committing agent".into(),
        ));
    }
    let expected_owner_did = format!("did:mycelix:{}", action.author());
    let request_record = must_get_valid_record(cancellation.request_action_hash.clone())?;
    let request: RecoveryRequest = request_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Cancellation references an invalid recovery request".into()
        )))?;
    if request.id != cancellation.request_id || request.did != cancellation.did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cancellation receipt does not match the referenced recovery request".into(),
        ));
    }
    if request.status != RecoveryStatus::Pending
        && request.status != RecoveryStatus::Approved
        && request.status != RecoveryStatus::ReadyToExecute
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only an active recovery request can be cancelled".into(),
        ));
    }
    let config_hash = request.config_action_hash.as_ref().ok_or(wasm_error!(
        WasmErrorInner::Guest("Recovery request has no policy revision".into())
    ))?;
    let config_record = must_get_valid_record(config_hash.clone())?;
    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Cancellation references an invalid recovery policy".into()
        )))?;
    if config.did != cancellation.did
        || config.owner != *action.author()
        || expected_owner_did != cancellation.did
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the owner of the protected DID may cancel recovery".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate recovery config creation
fn validate_create_recovery_config(
    action: EntryCreationAction,
    config: RecoveryConfig,
) -> ExternResult<ValidateCallbackResult> {
    // Validate DID format
    if !config.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:mycelix:'".into(),
        ));
    }

    // Validate owner is author
    if config.owner != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must be the author".into(),
        ));
    }

    // Bind the protected DID itself to the owner. Without this check an
    // attacker can create a config whose owner is their own key but whose
    // DID names a victim, then publish attacker-controlled trustees for it.
    if !recovery_config_matches_owner_did(&config.did, action.author()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery config DID must correspond to the committing owner".into(),
        ));
    }

    // Validate trustee count (3-7)
    if config.trustees.len() < 3 || config.trustees.len() > 7 {
        return Ok(ValidateCallbackResult::Invalid(
            "Must have 3-7 trustees".into(),
        ));
    }

    // Validate threshold
    let min_threshold = (config.trustees.len() as f64 * 0.5).ceil() as u32;
    if config.threshold < min_threshold {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Threshold must be at least {} (majority)",
            min_threshold
        )));
    }

    if config.threshold as usize > config.trustees.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Threshold cannot exceed trustee count".into(),
        ));
    }

    // Validate time lock (minimum 24 hours)
    if config.time_lock < 86400 {
        return Ok(ValidateCallbackResult::Invalid(
            "Time lock must be at least 24 hours (86400 seconds)".into(),
        ));
    }

    // Validate all trustees are unique (prevent Sybil attacks)
    let unique_trustees: HashSet<&String> = config.trustees.iter().collect();
    if unique_trustees.len() != config.trustees.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Duplicate trustees are not allowed".into(),
        ));
    }

    // Validate all trustees are valid DIDs
    for trustee in &config.trustees {
        if !trustee.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Invalid trustee DID: {}",
                trustee
            )));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate recovery config update
///
/// Author-binding for updates is already enforced universally by this
/// crate's `FlatOp::RegisterUpdate` arm in `validate()` (checks
/// `original.action().author() == action.author` for every entry type), so
/// this function only needs to check content invariants, not identity.
fn validate_update_recovery_config(
    action: Update,
    config: RecoveryConfig,
) -> ExternResult<ValidateCallbackResult> {
    // Validate trustee count (3-7)
    if config.trustees.len() < 3 || config.trustees.len() > 7 {
        return Ok(ValidateCallbackResult::Invalid(
            "Must have 3-7 trustees".into(),
        ));
    }

    // Validate all trustees are unique (prevent Sybil attacks)
    let unique_trustees: HashSet<&String> = config.trustees.iter().collect();
    if unique_trustees.len() != config.trustees.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Duplicate trustees are not allowed".into(),
        ));
    }

    // Validate threshold
    let min_threshold = (config.trustees.len() as f64 * 0.5).ceil() as u32;
    if config.threshold < min_threshold {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Threshold must be at least {} (majority)",
            min_threshold
        )));
    }

    // Fetch original to enforce invariants
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: RecoveryConfig = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original recovery config not found".into()
        )))?;

    // Immutable fields
    if config.did != original.did {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery config DID cannot be changed".into(),
        ));
    }
    if config.owner != original.owner {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery config owner cannot be changed".into(),
        ));
    }
    if config.created != original.created {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery config created timestamp cannot be changed".into(),
        ));
    }

    // Updated timestamp must advance
    if config.updated <= original.updated {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery config updated timestamp must advance".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate recovery request creation
fn validate_create_recovery_request(
    action: EntryCreationAction,
    request: RecoveryRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Validate DID format
    if !request.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:mycelix:'".into(),
        ));
    }

    // Validate initiator is a DID.
    if !request.initiated_by.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Initiator must be a valid Mycelix DID".into(),
        ));
    }

    // The request must reference the exact config revision that authorized it.
    // This makes trustee membership independently verifiable by every peer,
    // without trusting the coordinator zome's checks or current config state.
    let config_action_hash = match request.config_action_hash.as_ref() {
        Some(hash) => hash,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery request must reference its authorizing config revision".into(),
            ));
        }
    };
    let config_record = must_get_valid_record(config_action_hash.clone())?;
    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Referenced recovery config entry is invalid".into()
        )))?;
    if !recovery_request_matches_config(&request, &config, action.author()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request must target an active config and be initiated by one of its trustees".into(),
        ));
    }

    // Validate reason provided
    if request.reason.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery reason is required".into(),
        ));
    }

    // Validate initial status is Pending
    if request.status != RecoveryStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "Initial status must be Pending".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Count quorum only from distinct, authorized trustees. Repeated entries from
/// the same trustee are rejected instead of contributing additional weight.
fn recovery_decision_has_quorum(
    votes: &[(String, VoteDecision)],
    config: &RecoveryConfig,
    target: &RecoveryStatus,
) -> bool {
    let mut seen = HashSet::new();
    let mut approvals = 0usize;
    let mut rejections = 0usize;
    for (trustee, decision) in votes {
        if !config.trustees.contains(trustee) || !seen.insert(trustee) {
            return false;
        }
        match decision {
            VoteDecision::Approve => approvals += 1,
            VoteDecision::Reject => rejections += 1,
            VoteDecision::Abstain => {}
        }
    }
    match target {
        RecoveryStatus::Approved => approvals >= config.threshold as usize,
        RecoveryStatus::Rejected => {
            rejections > config.trustees.len().saturating_sub(config.threshold as usize)
        }
        _ => false,
    }
}

/// Validate recovery request update (status transitions)
///
/// Author-binding for updates is already enforced universally by this
/// crate's `FlatOp::RegisterUpdate` arm in `validate()`: only the SAME agent
/// who created the RecoveryRequest can commit an Update to it.
///
/// KNOWN GAP found while reviewing this path (2026-07-08, out of scope to
/// fix in this pass): the coordinator's `check_and_update_request_status`
/// (called after every `vote_on_recovery`) locates the RecoveryRequest via
/// `query()`, which only searches the CALLING agent's own local source
/// chain -- not the DHT. Since the request entry only lives on whichever
/// agent originally called `initiate_recovery`, this lookup returns `None`
/// (and silently no-ops) for every trustee except that original initiator.
/// In a real multi-agent deployment this likely means threshold-reached
/// auto-approval never actually fires except in the degenerate case where
/// the initiator is also the vote that crosses the threshold. This is a
/// functional/availability bug in vote tallying, not a security-binding
/// gap -- flagging for a dedicated follow-up (would need
/// `check_and_update_request_status` to look up the request via its DHT
/// link, the same way `get_recovery_votes` does, rather than local `query()`).
fn validate_update_recovery_request(
    action: Update,
    request: RecoveryRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Validate DID format
    if !request.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:mycelix:'".into(),
        ));
    }

    // Fetch original to enforce invariants
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: RecoveryRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original recovery request not found".into()
        )))?;

    // Immutable fields
    if request.id != original.id {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request ID cannot be changed".into(),
        ));
    }
    if request.did != original.did {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request DID cannot be changed".into(),
        ));
    }
    if request.new_agent != original.new_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request new_agent cannot be changed".into(),
        ));
    }
    if request.initiated_by != original.initiated_by {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request initiator cannot be changed".into(),
        ));
    }
    if request.created != original.created {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request created timestamp cannot be changed".into(),
        ));
    }

    if request.time_lock_expires != original.time_lock_expires
        && original.time_lock_expires.is_some()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery time-lock expiry cannot be shortened or changed".into(),
        ));
    }

    if request.config_action_hash != original.config_action_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request config revision cannot be changed".into(),
        ));
    }

    // Validate status transitions via state machine
    // Terminal states: Completed, Rejected, Cancelled cannot transition further
    let valid_transition = match (&original.status, &request.status) {
        // Pending → Approved (threshold reached), Rejected, or Cancelled
        (RecoveryStatus::Pending, RecoveryStatus::Approved)
        | (RecoveryStatus::Pending, RecoveryStatus::Rejected)
        | (RecoveryStatus::Pending, RecoveryStatus::Cancelled) => true,
        // Approved → ReadyToExecute (timelock expired) or Cancelled
        (RecoveryStatus::Approved, RecoveryStatus::ReadyToExecute)
        | (RecoveryStatus::Approved, RecoveryStatus::Cancelled) => true,
        // ReadyToExecute → Completed or Cancelled
        (RecoveryStatus::ReadyToExecute, RecoveryStatus::Completed)
        | (RecoveryStatus::ReadyToExecute, RecoveryStatus::Cancelled) => true,
        // No-op status updates are unnecessary and would create room for
        // mutating time-lock or other decision metadata.
        (a, b) if a == b => false,
        _ => false,
    };
    if !valid_transition {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid recovery status transition from {:?} to {:?}",
            original.status, request.status
        )));
    }

    // Decision evidence is fixed once the request leaves Pending.
    if original.status != RecoveryStatus::Pending
        && request.decision_vote_hashes != original.decision_vote_hashes
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery decision evidence cannot be changed after finalization".into(),
        ));
    }

    // An Approved/Rejected transition must carry verifiable, author-bound
    // vote records. This prevents a malicious coordinator from changing the
    // status to Approved without a real trustee quorum.
    if original.status == RecoveryStatus::Pending
        && (request.status == RecoveryStatus::Approved
            || request.status == RecoveryStatus::Rejected)
    {
        let config_hash = original.config_action_hash.as_ref().ok_or(wasm_error!(
            WasmErrorInner::Guest("Original recovery request has no config revision".into())
        ))?;
        let config_record = must_get_valid_record(config_hash.clone())?;
        let config: RecoveryConfig = config_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Original recovery config entry is invalid".into()
            )))?;

        let mut verified_votes = Vec::new();
        let mut unique_vote_hashes = HashSet::new();
        for vote_hash in &request.decision_vote_hashes {
            if !unique_vote_hashes.insert(vote_hash.clone()) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Recovery decision evidence contains duplicate vote references".into(),
                ));
            }
            let vote_record = must_get_valid_record(vote_hash.clone())?;
            let vote: RecoveryVote = vote_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Recovery decision evidence references a non-vote entry".into()
                )))?;
            let vote_request_hash = vote.request_action_hash.as_ref().ok_or(wasm_error!(
                WasmErrorInner::Guest("Evidence vote has no request action reference".into())
            ))?;
            let vote_request_record = must_get_valid_record(vote_request_hash.clone())?;
            let vote_request: RecoveryRequest = vote_request_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Evidence vote references an invalid recovery request".into()
                )))?;
            if vote_request.id != original.id
                || vote_request.config_action_hash != original.config_action_hash
                || !recovery_vote_matches_policy(
                    &vote,
                    &vote_request,
                    &config,
                    vote_record.action().author(),
                )
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Recovery decision evidence contains an unauthorized or unrelated vote".into(),
                ));
            }
            verified_votes.push((vote.trustee, vote.vote));
        }

        if !recovery_decision_has_quorum(&verified_votes, &config, &request.status) {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery status transition lacks a distinct-trustee quorum".into(),
            ));
        }
        if request.status == RecoveryStatus::Approved {
            let minimum_expiry = action.timestamp.as_micros()
                + (config.time_lock as i64 * 1_000_000);
            if request.time_lock_expires.map(|expiry| expiry.as_micros()).unwrap_or(0)
                < minimum_expiry
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Recovery time lock must run for the full configured duration after approval".into(),
                ));
            }
        }
    }

    // Approved status must have time_lock_expires set
    if request.status == RecoveryStatus::Approved && request.time_lock_expires.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "Approved recovery must have time_lock_expires set".into(),
        ));
    }

    // The timelock must have expired before entering ReadyToExecute or Completed.
    if original.status == RecoveryStatus::Approved
        && request.status == RecoveryStatus::ReadyToExecute
    {
        let expires = original.time_lock_expires.ok_or(wasm_error!(
            WasmErrorInner::Guest("Approved request has no time lock expiry".into())
        ))?;
        if action.timestamp < expires {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery cannot enter ReadyToExecute before the time lock expires".into(),
            ));
        }
    }
    if original.status == RecoveryStatus::ReadyToExecute
        && request.status == RecoveryStatus::Completed
    {
        let expires = original.time_lock_expires.ok_or(wasm_error!(
            WasmErrorInner::Guest("ReadyToExecute request has no time lock expiry".into())
        ))?;
        if action.timestamp < expires {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery cannot complete before the time lock expires".into(),
            ));
        }
    }

    // Completed status requires time_lock_expires to be set
    if request.status == RecoveryStatus::Completed && request.time_lock_expires.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot complete recovery without timelock (time_lock_expires must be set)".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

// =============================================================================
// Self-Recovery Validation
// =============================================================================

/// Validate self-recovery config creation
fn validate_create_self_recovery_config(
    action: EntryCreationAction,
    config: SelfRecoveryConfig,
) -> ExternResult<ValidateCallbackResult> {
    if !config.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:mycelix:'".into(),
        ));
    }
    if config.owner != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must be the author".into(),
        ));
    }
    if !recovery_config_matches_owner_did(&config.did, action.author()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery DID must correspond to the committing owner".into(),
        ));
    }
    // This implementation intentionally keeps self-recovery inactive until
    // enrolled anchors are backed by cryptographic challenge-response proofs.
    if config.active {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery cannot be activated without challenge-backed anchor verification".into(),
        ));
    }
    // Time lock minimum: 72 hours for self-recovery (stronger than social's 24h)
    if config.time_lock < SELF_RECOVERY_MIN_TIME_LOCK {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Self-recovery time lock must be at least {} seconds (72 hours)",
            SELF_RECOVERY_MIN_TIME_LOCK
        )));
    }
    // Anchors can be empty at creation (progressive enrollment)
    // Threshold must not exceed anchor count (when anchors exist)
    if !config.anchors.is_empty() && config.anchor_threshold as usize > config.anchors.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Anchor threshold cannot exceed anchor count".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Self-recovery request creation is disabled until the proof-of-control
/// protocol is implemented. A caller-supplied hash is not evidence that the
/// caller controls the phone, email, device, passkey, or biometric factor.
fn validate_create_self_recovery_request(
    _request: SelfRecoveryRequest,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(
        "Self-recovery requests are disabled until challenge-backed anchor proof is implemented".into(),
    ))
}

/// Only an inactive self-recovery configuration may be maintained. Anchor
/// enrollment and activation are deliberately blocked until every anchor type
/// has a real proof-of-control protocol.
fn validate_update_self_recovery_config(
    action: Update,
    config: SelfRecoveryConfig,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: SelfRecoveryConfig = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original self-recovery config not found".into()
        )))?;

    if config.did != original.did
        || config.owner != original.owner
        || config.created != original.created
        || config.anchors != original.anchors
        || config.anchor_threshold != original.anchor_threshold
        || config.time_lock != original.time_lock
        || config.active
        || original.active
        || (original.superseded_by_social && !config.superseded_by_social)
        || config.updated <= original.updated
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery configuration must remain inactive and cannot change anchors or ownership".into(),
        ));
    }
    if *original_record.action().author() != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the original self-recovery config author may update it".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Existing self-recovery requests cannot transition while the verification
/// mechanism is disabled. This prevents historical hash-only requests from
/// being completed through an old coordinator path.
fn validate_update_self_recovery_request(
    _action: Update,
    _request: SelfRecoveryRequest,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(
        "Self-recovery state transitions are disabled pending proof-of-control support".into(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn arb_recovery_status() -> impl Strategy<Value = RecoveryStatus> {
        prop_oneof![
            Just(RecoveryStatus::Pending),
            Just(RecoveryStatus::Approved),
            Just(RecoveryStatus::ReadyToExecute),
            Just(RecoveryStatus::Completed),
            Just(RecoveryStatus::Rejected),
            Just(RecoveryStatus::Cancelled),
        ]
    }

    fn arb_vote_decision() -> impl Strategy<Value = VoteDecision> {
        prop_oneof![
            Just(VoteDecision::Approve),
            Just(VoteDecision::Reject),
            Just(VoteDecision::Abstain),
        ]
    }

    /// Generate a valid trustee list (3-7 unique DID strings).
    fn arb_trustee_list() -> impl Strategy<Value = Vec<String>> {
        (3usize..=7).prop_flat_map(|count| {
            proptest::collection::vec(
                "[a-zA-Z0-9]{8,16}".prop_map(|s| format!("did:mycelix:{}", s)),
                count,
            )
        })
    }

    proptest! {
        /// RecoveryStatus round-trips through JSON.
        #[test]
        fn recovery_status_json_roundtrip(status in arb_recovery_status()) {
            let json = serde_json::to_string(&status).unwrap();
            let back: RecoveryStatus = serde_json::from_str(&json).unwrap();
            prop_assert_eq!(status, back);
        }

        /// VoteDecision round-trips through JSON.
        #[test]
        fn vote_decision_json_roundtrip(vote in arb_vote_decision()) {
            let json = serde_json::to_string(&vote).unwrap();
            let back: VoteDecision = serde_json::from_str(&json).unwrap();
            prop_assert_eq!(vote, back);
        }

        /// Majority threshold is always ceil(n/2) for any valid trustee count.
        #[test]
        fn majority_threshold_formula(count in 3usize..=7) {
            let min_threshold = (count as f64 * 0.5).ceil() as u32;
            // For 3 trustees: ceil(1.5) = 2
            // For 4 trustees: ceil(2.0) = 2
            // For 5 trustees: ceil(2.5) = 3
            // For 6 trustees: ceil(3.0) = 3
            // For 7 trustees: ceil(3.5) = 4
            prop_assert!(min_threshold >= 2, "Majority must be at least 2");
            prop_assert!(min_threshold as usize <= count, "Majority can't exceed count");
        }

        /// Valid thresholds are within [majority, trustees.len()].
        #[test]
        fn threshold_bounds(trustees in arb_trustee_list()) {
            let n = trustees.len();
            let min_threshold = (n as f64 * 0.5).ceil() as u32;
            for threshold in min_threshold..=(n as u32) {
                prop_assert!(threshold >= min_threshold);
                prop_assert!(threshold as usize <= n);
            }
        }

        /// Terminal recovery states cannot transition to non-self states.
        #[test]
        fn terminal_recovery_states(
            target in arb_recovery_status()
        ) {
            let terminals = [
                RecoveryStatus::Completed,
                RecoveryStatus::Rejected,
                RecoveryStatus::Cancelled,
            ];
            for terminal in &terminals {
                let valid = match (terminal, &target) {
                    (RecoveryStatus::Completed, RecoveryStatus::Completed)
                    | (RecoveryStatus::Rejected, RecoveryStatus::Rejected)
                    | (RecoveryStatus::Cancelled, RecoveryStatus::Cancelled) => true,
                    _ => false,
                };
                if terminal != &target {
                    prop_assert!(!valid, "Terminal {:?} should not transition to {:?}", terminal, target);
                }
            }
        }

        /// Pending can transition to Approved, Rejected, or Cancelled (not ReadyToExecute or Completed).
        #[test]
        fn pending_valid_transitions(target in arb_recovery_status()) {
            let valid = match target {
                RecoveryStatus::Approved
                | RecoveryStatus::Rejected
                | RecoveryStatus::Cancelled
                | RecoveryStatus::Pending => true,
                RecoveryStatus::ReadyToExecute | RecoveryStatus::Completed => false,
            };
            // This matches the validation logic in validate_update_recovery_request
            let matches_validation = match (&RecoveryStatus::Pending, &target) {
                (RecoveryStatus::Pending, RecoveryStatus::Approved)
                | (RecoveryStatus::Pending, RecoveryStatus::Rejected)
                | (RecoveryStatus::Pending, RecoveryStatus::Cancelled) => true,
                (a, b) if a == b => true,
                _ => false,
            };
            prop_assert_eq!(valid, matches_validation);
        }

        /// Generated trustee lists always have 3-7 entries.
        #[test]
        fn trustee_list_size(trustees in arb_trustee_list()) {
            prop_assert!(trustees.len() >= 3);
            prop_assert!(trustees.len() <= 7);
            for t in &trustees {
                prop_assert!(t.starts_with("did:mycelix:"));
            }
        }
    }

    // ── Self-Recovery Tests ──

    #[test]
    fn self_recovery_min_time_lock_is_72_hours() {
        assert_eq!(SELF_RECOVERY_MIN_TIME_LOCK, 72 * 3600);
    }

    #[test]
    fn self_recovery_default_time_lock_is_7_days() {
        assert_eq!(SELF_RECOVERY_DEFAULT_TIME_LOCK, 7 * 24 * 3600);
    }

    #[test]
    fn self_recovery_default_longer_than_social() {
        // Self-recovery (72h) must be longer than social recovery (24h)
        assert!(SELF_RECOVERY_MIN_TIME_LOCK > 86400);
    }

    #[test]
    fn verification_anchor_serde_roundtrip() {
        let anchors = vec![
            VerificationAnchor::PhoneHash("sha256:abc123".into()),
            VerificationAnchor::EmailHash("sha256:def456".into()),
            VerificationAnchor::PasskeyCredentialId("cred-001".into()),
            VerificationAnchor::DeviceAttestation("device-hash".into()),
            VerificationAnchor::BiometricHash("bio-hash".into()),
        ];
        for anchor in anchors {
            let json = serde_json::to_string(&anchor).unwrap();
            let back: VerificationAnchor = serde_json::from_str(&json).unwrap();
            assert_eq!(anchor, back);
        }
    }

    #[test]
    fn self_recovery_request_status_machine_matches_social() {
        // Self-recovery uses the same RecoveryStatus state machine
        // Verify key transitions work
        let valid = |from: &RecoveryStatus, to: &RecoveryStatus| -> bool {
            match (from, to) {
                (RecoveryStatus::Pending, RecoveryStatus::Approved)
                | (RecoveryStatus::Pending, RecoveryStatus::Cancelled) => true,
                (RecoveryStatus::Approved, RecoveryStatus::ReadyToExecute)
                | (RecoveryStatus::Approved, RecoveryStatus::Cancelled) => true,
                (RecoveryStatus::ReadyToExecute, RecoveryStatus::Completed)
                | (RecoveryStatus::ReadyToExecute, RecoveryStatus::Cancelled) => true,
                (a, b) if a == b => true,
                _ => false,
            }
        };

        assert!(valid(&RecoveryStatus::Pending, &RecoveryStatus::Approved));
        assert!(valid(
            &RecoveryStatus::Approved,
            &RecoveryStatus::ReadyToExecute
        ));
        assert!(valid(
            &RecoveryStatus::ReadyToExecute,
            &RecoveryStatus::Completed
        ));
        assert!(!valid(&RecoveryStatus::Pending, &RecoveryStatus::Completed)); // can't skip
        assert!(!valid(&RecoveryStatus::Completed, &RecoveryStatus::Pending)); // terminal
    }
}

/// Validate recovery vote creation
fn validate_create_recovery_vote(
    action: EntryCreationAction,
    vote: RecoveryVote,
) -> ExternResult<ValidateCallbackResult> {
    // Validate trustee is a DID
    if !vote.trustee.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Trustee must be a valid DID".into(),
        ));
    }

    // Validate request ID not empty
    if vote.request_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request ID is required".into(),
        ));
    }

    // Resolve the exact request revision. A caller must not be able to vote
    // by inventing a request id or by omitting the DHT action reference.
    let request_action_hash = match vote.request_action_hash.as_ref() {
        Some(hash) => hash,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery vote must reference its RecoveryRequest action".into(),
            ));
        }
    };
    let request_record = must_get_valid_record(request_action_hash.clone())?;
    let request: RecoveryRequest = request_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Referenced recovery request entry is invalid".into()
        )))?;
    if request.id != vote.request_id {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery vote request ID does not match the referenced request".into(),
        ));
    }

    // Resolve the frozen config revision and enforce trustee membership at
    // the integrity layer. Coordinator-only checks are not a security boundary.
    let config_action_hash = match request.config_action_hash.as_ref() {
        Some(hash) => hash,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Referenced recovery request has no config revision".into(),
            ));
        }
    };
    let config_record = must_get_valid_record(config_action_hash.clone())?;
    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
               .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Referenced recovery config entry is invalid".into()
        )))?;
    if !recovery_vote_matches_policy(&vote, &request, &config, action.author()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery vote author is not an authorized trustee for this request".into(),
        ));
    }

    // Author-binding: the coordinator's vote_on_recovery already checks
    // `input.trustee_did != caller_did` ("Caller must be the claimed
    // trustee") before creating the entry, but that check is bypassable by
    // a modified coordinator -- the integrity validator is the real
    // security boundary. Without this, any agent could commit a
    // RecoveryVote claiming to be an arbitrary trustee, forging approval
    // votes for unauthorized DID/identity takeover. Bind here as
    // belt-and-suspenders.
    let expected_trustee_did = format!("did:mycelix:{}", action.author());
    if vote.trustee != expected_trustee_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery vote trustee DID must correspond to the committing agent".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn config(did: String, trustees: Vec<String>, active: bool) -> RecoveryConfig {
        RecoveryConfig {
            did,
            owner: me(),
            trustees,
            threshold: 2,
            time_lock: 86_400,
            active,
            created: Timestamp::from_micros(0),
            updated: Timestamp::from_micros(0),
        }
    }

    fn request(initiated_by: String) -> RecoveryRequest {
        RecoveryRequest {
            id: "req-1".into(),
            did: format!("did:mycelix:{}", other_agent()),
            new_agent: other_agent(),
            initiated_by,
            reason: "Owner lost access".into(),
            status: RecoveryStatus::Pending,
            created: Timestamp::from_micros(0),
            time_lock_expires: None,
            config_action_hash: None,
            decision_vote_hashes: Vec::new(),
        }
    }

    fn vote(trustee: String) -> RecoveryVote {
        RecoveryVote {
            request_id: "req-1".into(),
            trustee,
            vote: VoteDecision::Approve,
            comment: None,
            voted_at: Timestamp::from_micros(0),
            request_action_hash: None,
        }
    }

    #[test]
    fn recovery_config_must_name_its_owner_did() {
        assert!(recovery_config_matches_owner_did(
            &format!("did:mycelix:{}", me()),
            &me()
        ));
        assert!(!recovery_config_matches_owner_did(
            &format!("did:mycelix:{}", other_agent()),
            &me()
        ));
    }

    #[test]
    fn recovery_request_requires_config_trustee_and_author_binding() {
        let author = me();
        let initiator = format!("did:mycelix:{}", author);
        let target_did = format!("did:mycelix:{}", other_agent());
        let req = request(initiator.clone());
        let policy = config(
            target_did,
            vec![initiator.clone(), "did:mycelix:another-trustee".into(), "did:mycelix:third".into()],
            true,
        );
        assert!(recovery_request_matches_config(&req, &policy, &author));

        let non_trustee = request("did:mycelix:outsider".into());
        assert!(!recovery_request_matches_config(&non_trustee, &policy, &author));

        let inactive = config(
            req.did.clone(),
            policy.trustees.clone(),
            false,
        );
        assert!(!recovery_request_matches_config(&req, &inactive, &author));

        let wrong_target = config(
            format!("did:mycelix:{}", me()),
            policy.trustees.clone(),
            true,
        );
        assert!(!recovery_request_matches_config(&req, &wrong_target, &author));

        assert!(!recovery_request_matches_config(&req, &policy, &other_agent()));
    }

    #[test]
    fn recovery_decision_requires_distinct_trustee_quorum() {
        let target_did = format!("did:mycelix:{}", other_agent());
        let trustees = vec![
            format!("did:mycelix:{}", me()),
            "did:mycelix:trustee-two".into(),
            "did:mycelix:trustee-three".into(),
        ];
        let policy = config(target_did, trustees, true);
        let approvals = vec![
            (format!("did:mycelix:{}", me()), VoteDecision::Approve),
            ("did:mycelix:trustee-two".into(), VoteDecision::Approve),
        ];
        assert!(recovery_decision_has_quorum(
            &approvals,
            &policy,
            &RecoveryStatus::Approved
        ));

        let one_approval = vec![(
            format!("did:mycelix:{}", me()),
            VoteDecision::Approve,
        )];
        assert!(!recovery_decision_has_quorum(
            &one_approval,
            &policy,
            &RecoveryStatus::Approved
        ));

        let duplicate_signer = vec![
            (format!("did:mycelix:{}", me()), VoteDecision::Approve),
            (format!("did:mycelix:{}", me()), VoteDecision::Approve),
        ];
        assert!(!recovery_decision_has_quorum(
            &duplicate_signer,
            &policy,
            &RecoveryStatus::Approved
        ));

        let outsider = vec![
            (format!("did:mycelix:{}", me()), VoteDecision::Approve),
            ("did:mycelix:outside-policy".into(), VoteDecision::Approve),
        ];
        assert!(!recovery_decision_has_quorum(
            &outsider,
            &policy,
            &RecoveryStatus::Approved
        ));

        let rejections = vec![
            (format!("did:mycelix:{}", me()), VoteDecision::Reject),
            ("did:mycelix:trustee-two".into(), VoteDecision::Reject),
        ];
        assert!(recovery_decision_has_quorum(
            &rejections,
            &policy,
            &RecoveryStatus::Rejected
        ));
    }

    #[test]
    fn recovery_vote_requires_trustee_membership_and_author_binding() {
        let author = me();
        let trustee_did = format!("did:mycelix:{}", author);
        let req = request("did:mycelix:initiator".into());
        let policy = config(
            req.did.clone(),
            vec![trustee_did.clone(), "did:mycelix:other".into(), "did:mycelix:third".into()],
            true,
        );

        assert!(recovery_vote_matches_policy(
            &vote(trustee_did.clone()),
            &req,
            &policy,
            &author
        ));
        assert!(!recovery_vote_matches_policy(
            &vote("did:mycelix:outsider".into()),
            &req,
            &policy,
            &other_agent()
        ));
        assert!(!recovery_vote_matches_policy(
            &vote("did:mycelix:outsider".into()),
            &req,
            &policy,
            &author
        ));
        assert!(!recovery_vote_matches_policy(
            &vote(trustee_did),
            &request("other-request".into()),
            &policy,
            &author
        ));
    }
}

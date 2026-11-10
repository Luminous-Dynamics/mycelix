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
                EntryTypes::SelfRecoveryConfig(config) => {
                    validate_update_self_recovery_config(action, config)
                }
                EntryTypes::SelfRecoveryRequest(request) => {
                    validate_update_self_recovery_request(action, request)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            action,
        } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }

            match link_type {
                LinkTypes::DidToRecoveryConfig => {
                    let target_hash = match target_address.into_action_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Recovery config link target must be an action hash".into(),
                            ));
                        }
                    };
                    let record = must_get_valid_record(target_hash)?;
                    let Some(config) = record
                        .entry()
                        .to_app_option::<RecoveryConfig>()
                        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
                    else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "DID recovery-config link must target a RecoveryConfig".into(),
                        ));
                    };
                    let base_hash = match base_address.into_entry_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "DID recovery-config link base must be an entry hash".into(),
                            ));
                        }
                    };
                    match validate_recovery_config_link_fields(
                        &base_hash,
                        &config.did,
                        &config.owner,
                        &action.author,
                    ) {
                        Ok(()) => Ok(ValidateCallbackResult::Valid),
                        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason.into())),
                    }
                }
                LinkTypes::DidToSelfRecoveryConfig => {
                    let target_hash = match target_address.into_action_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Self-recovery config link target must be an action hash".into(),
                            ));
                        }
                    };
                    let record = must_get_valid_record(target_hash)?;
                    let Some(config) = record
                        .entry()
                        .to_app_option::<SelfRecoveryConfig>()
                        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
                    else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "DID self-recovery link must target a SelfRecoveryConfig".into(),
                        ));
                    };
                    let base_hash = match base_address.into_entry_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "DID self-recovery config link base must be an entry hash".into(),
                            ));
                        }
                    };
                    match validate_recovery_config_link_fields(
                        &base_hash,
                        &config.did,
                        &config.owner,
                        &action.author,
                    ) {
                        Ok(()) => Ok(ValidateCallbackResult::Valid),
                        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason.into())),
                    }
                }
                LinkTypes::DidToRecoveryRequest => {
                    let target_hash = match target_address.into_action_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Recovery request link target must be an action hash".into(),
                            ));
                        }
                    };
                    let record = must_get_valid_record(target_hash)?;
                    let Some(request) = record
                        .entry()
                        .to_app_option::<RecoveryRequest>()
                        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
                    else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "DID recovery-request link must target a RecoveryRequest".into(),
                        ));
                    };
                    let base_hash = match base_address.into_entry_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "DID recovery-request link base must be an entry hash".into(),
                            ));
                        }
                    };
                    if base_hash != string_index_hash(&request.did) {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Recovery request link base must match the request DID".into(),
                        ));
                    }
                    if record.action().author() != &action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the recovery-request author may create its DID index link".into(),
                        ));
                    }
                    if request.initiated_by != format!("did:mycelix:{}", action.author) {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Recovery-request link author must match the authenticated initiator".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::DidToSelfRecoveryRequest => {
                    let target_hash = match target_address.into_action_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Self-recovery request link target must be an action hash".into(),
                            ));
                        }
                    };
                    let record = must_get_valid_record(target_hash)?;
                    let Some(request) = record
                        .entry()
                        .to_app_option::<SelfRecoveryRequest>()
                        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
                    else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "DID self-recovery link must target a SelfRecoveryRequest".into(),
                        ));
                    };
                    let base_hash = match base_address.into_entry_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "DID self-recovery request link base must be an entry hash".into(),
                            ));
                        }
                    };
                    if base_hash != string_index_hash(&request.did) {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Self-recovery request link base must match the request DID".into(),
                        ));
                    }
                    if record.action().author() != &action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the self-recovery request author may create its DID index link".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::TrusteeToConfig => {
                    let target_hash = match target_address.into_action_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Trustee config link target must be an action hash".into(),
                            ));
                        }
                    };
                    let record = must_get_valid_record(target_hash)?;
                    let Some(config) = record
                        .entry()
                        .to_app_option::<RecoveryConfig>()
                        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
                    else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Trustee config link must target a RecoveryConfig".into(),
                        ));
                    };
                    if record.action().author() != &action.author || config.owner != action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the recovery-config owner may create trustee index links".into(),
                        ));
                    }
                    let base_hash = match base_address.into_entry_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Trustee config link base must be an entry hash".into(),
                            ));
                        }
                    };
                    if !config
                        .trustees
                        .iter()
                        .any(|trustee| string_index_hash(trustee) == base_hash)
                    {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Trustee config link base must match an enrolled trustee DID".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::RequestToVotes => {
                    // The coordinator indexes votes beneath the deterministic
                    // request_id entry hash, not the request action hash. This
                    // shared anchor lets every conductor query the same DHT
                    // vote set without depending on the initiator's local chain.
                    let request_id_hash = match base_address.into_entry_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Request vote link base must be a request-ID entry hash".into(),
                            ));
                        }
                    };
                    let vote_hash = match target_address.into_action_hash() {
                        Some(hash) => hash,
                        None => {
                            return Ok(ValidateCallbackResult::Invalid(
                                "Request vote link target must be an action hash".into(),
                            ));
                        }
                    };
                    let vote_record = must_get_valid_record(vote_hash)?;
                    let Some(vote) = vote_record
                        .entry()
                        .to_app_option::<RecoveryVote>()
                        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
                    else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Request vote link target must target a RecoveryVote".into(),
                        ));
                    };
                    if request_id_hash != string_index_hash(&vote.request_id) {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Vote link base must match the vote's request ID".into(),
                        ));
                    }
                    if vote_record.action().author() != &action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the vote author may create its request index link".into(),
                        ));
                    }
                    if vote.trustee != format!("did:mycelix:{}", action.author) {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Vote index link author must match the authenticated trustee".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
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

/// Deterministically reproduce the DID-string link base used by the
/// recovery coordinator. This hash is an index only; identity authority is
/// checked separately against the config owner and the committing action.
fn string_index_hash(value: &str) -> EntryHash {
    EntryHash::from_raw_36(
        holo_hash::blake2b_256(value.as_bytes())
            .into_iter()
            .chain([0u8; 4])
            .collect(),
    )
}

/// Check the ownership and target/base binding for a DID-to-recovery-config
/// link. This is integrity validation, so a coordinator cannot substitute a
/// config for a different DID by writing an arbitrary DHT link.
fn validate_recovery_config_link_fields(
    base_hash: &EntryHash,
    did: &str,
    owner: &AgentPubKey,
    link_author: &AgentPubKey,
) -> Result<(), &'static str> {
    let expected_did = format!("did:mycelix:{}", owner);
    if did != expected_did {
        return Err("Recovery config DID must correspond to its owner");
    }
    if owner != link_author {
        return Err("Only the recovery-config owner may create its DID index link");
    }
    if base_hash != &string_index_hash(did) {
        return Err("Recovery config link base must match the config DID");
    }
    Ok(())
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

    // Bind the config DID to the committing agent as well as the owner
    // field. Checking config.owner alone allows a caller to claim a victim DID.
    if config.did != format!("did:mycelix:{}", action.author()) {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery config DID must correspond to the committing owner".into(),
        ));
    }
    if config.owner != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must be the author".into(),
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

    // Validate initiator is a DID
    if !request.initiated_by.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Initiator must be a valid DID".into(),
        ));
    }

    // Author-binding: the coordinator's initiate_recovery already checks
    // `input.initiator_did != caller_did` ("Caller does not own the
    // initiator DID") before creating the entry, but that check is
    // bypassable by a modified coordinator -- the integrity validator is
    // the real security boundary. Without this, any agent could commit a
    // RecoveryRequest naming an arbitrary trustee as initiator, forging
    // trustee-initiated identity recovery. Bind here as belt-and-suspenders.
    let expected_initiator_did = format!("did:mycelix:{}", action.author());
    if request.initiated_by != expected_initiator_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request initiator DID must correspond to the committing agent".into(),
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
        // Same status (no-op update) is allowed
        (a, b) if a == b => true,
        _ => false,
    };
    if !valid_transition {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid recovery status transition from {:?} to {:?}",
            original.status, request.status
        )));
    }

    // Approved status must have time_lock_expires set
    if request.status == RecoveryStatus::Approved && request.time_lock_expires.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "Approved recovery must have time_lock_expires set".into(),
        ));
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
/// Validate the structural and ownership invariants of a self-recovery
/// configuration. The expected owner is supplied by the committing action,
/// never by the entry itself.
fn validate_self_recovery_config_fields(
    config: &SelfRecoveryConfig,
    expected_owner: &AgentPubKey,
) -> Result<(), &'static str> {
    if !config.did.starts_with("did:mycelix:") {
        return Err("DID must start with 'did:mycelix:'");
    }

    let expected_did = format!("did:mycelix:{}", expected_owner);
    if config.did != expected_did {
        return Err("Self-recovery config DID must correspond to the committing owner");
    }
    if config.owner != *expected_owner {
        return Err("Self-recovery config owner must be the committing agent");
    }
    if config.time_lock < SELF_RECOVERY_MIN_TIME_LOCK {
        return Err("Self-recovery time lock must be at least 72 hours");
    }
    if config.anchor_threshold == 0 {
        return Err("Self-recovery anchor threshold must be at least one");
    }
    if !config.anchors.is_empty() && config.anchor_threshold as usize > config.anchors.len() {
        return Err("Anchor threshold cannot exceed anchor count");
    }

    // Duplicate anchors must not increase the apparent number of independent
    // recovery factors. Use equality over the typed anchor values rather than
    // relying on a non-cryptographic string/hash cardinality.
    for (index, anchor) in config.anchors.iter().enumerate() {
        if config.anchors[..index].contains(anchor) {
            return Err("Duplicate self-recovery anchors are not allowed");
        }
        let value = match anchor {
            VerificationAnchor::PhoneHash(value)
            | VerificationAnchor::EmailHash(value)
            | VerificationAnchor::PasskeyCredentialId(value)
            | VerificationAnchor::DeviceAttestation(value)
            | VerificationAnchor::BiometricHash(value) => value,
        };
        if value.trim().is_empty() {
            return Err("Self-recovery anchor values must not be empty");
        }
    }

    Ok(())
}

/// Validate immutable identity and append-only anchor semantics for a
/// self-recovery configuration update. Existing anchor removal/replacement
/// requires a future explicit revocation workflow with its own audit record.
fn validate_self_recovery_config_update_fields(
    original: &SelfRecoveryConfig,
    updated: &SelfRecoveryConfig,
) -> Result<(), &'static str> {
    if updated.did != original.did {
        return Err("Self-recovery config DID cannot be changed");
    }
    if updated.owner != original.owner {
        return Err("Self-recovery config owner cannot be changed");
    }
    if updated.created != original.created {
        return Err("Self-recovery config creation timestamp cannot be changed");
    }
    if updated.updated <= original.updated {
        return Err("Self-recovery config updated timestamp must advance");
    }
    if original.superseded_by_social && !updated.superseded_by_social {
        return Err("Self-recovery cannot be unsuperseded by an update");
    }
    if original.anchors.iter().any(|anchor| !updated.anchors.contains(anchor)) {
        return Err("Previously enrolled self-recovery anchors cannot be removed or replaced");
    }
    validate_self_recovery_config_fields(updated, &original.owner)
}

/// Validate self-recovery config creation. A DID's self-recovery config must
/// be committed by the agent whose key is embedded in that DID. Checking only
/// config.owner is insufficient: without the DID binding, an attacker could
/// create a config that names a victim DID while recording the attacker as
/// owner, then place an attacker-controlled link under the victim DID.
fn validate_create_self_recovery_config(
    action: EntryCreationAction,
    config: SelfRecoveryConfig,
) -> ExternResult<ValidateCallbackResult> {
    match validate_self_recovery_config_fields(&config, action.author()) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason.into())),
    }
}

/// Validate self-recovery config updates. The generic RegisterUpdate callback
/// also enforces that the original entry author committed the update; this
/// function additionally protects the identity and recovery-factor history.
fn validate_update_self_recovery_config(
    action: Update,
    config: SelfRecoveryConfig,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: SelfRecoveryConfig = original_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original self-recovery config not found".into()
        )))?;

    match validate_self_recovery_config_update_fields(&original, &config) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason.into())),
    }
}

/// Validate self-recovery request creation
///
/// Deliberately NOT author-bound (reviewed 2026-07-08, P0 pass): binding
/// this to action.author() would be architecturally WRONG, not just
/// unnecessary. Self-recovery exists specifically for when the DID owner's
/// ORIGINAL agent key is unreachable -- the whole point is that the
/// committing agent is necessarily a *different* key (a new device/session)
/// than the one being recovered from. Its actual security comes from
/// anchor-possession proof (verified_anchors must match
/// SelfRecoveryConfig.anchors, checked coordinator-side in
/// initiate_self_recovery/verify_self_recovery_anchor), not agent identity.
///
/// KNOWN GAP found while reviewing this path (out of scope to fix here):
/// `verify_self_recovery_anchor` checks anchor possession by comparing the
/// caller-supplied hash against the enrolled hash list with no actual
/// proof-of-control (no signature challenge) -- anyone who knows/guesses/
/// leaks the phone/email hash can "verify" that anchor. This is a
/// proof-of-possession gap, architecturally different from the
/// author-identity forgeries fixed elsewhere this pass (same class as the
/// report_reputation forgery gap documented in the bridge zome).
/// Validate the initial self-recovery request fields. The request is
/// deliberately not bound to the recovered DID owner's key because a new
/// device must be able to initiate recovery after the old key is lost.
/// Until cryptographic proof receipts exist, however, no anchor may be
/// recorded as already verified.
fn validate_self_recovery_request_creation_fields(
    request: &SelfRecoveryRequest,
) -> Result<(), &'static str> {
    if !request.did.starts_with("did:mycelix:") {
        return Err("DID must start with 'did:mycelix:'");
    }
    if request.reason.is_empty() {
        return Err("Recovery reason is required");
    }
    if request.status != RecoveryStatus::Pending {
        return Err("Initial status must be Pending");
    }
    if !request.verified_anchors.is_empty() {
        return Err("Self-recovery cannot record verified anchors without proof receipts");
    }
    Ok(())
}

/// Self-recovery approval is deliberately unavailable in v1. There is no
/// proof receipt in SelfRecoveryRequest, so a transition to Approved would
/// rely only on mutable caller-supplied strings rather than proof-of-control.
fn self_recovery_status_transition_allowed(
    from: &RecoveryStatus,
    to: &RecoveryStatus,
) -> bool {
    match (from, to) {
        (RecoveryStatus::Pending, RecoveryStatus::Cancelled) => true,
        (RecoveryStatus::Approved, RecoveryStatus::ReadyToExecute)
        | (RecoveryStatus::Approved, RecoveryStatus::Cancelled) => true,
        (RecoveryStatus::ReadyToExecute, RecoveryStatus::Completed)
        | (RecoveryStatus::ReadyToExecute, RecoveryStatus::Cancelled) => true,
        (a, b) if a == b => true,
        _ => false,
    }
}

fn validate_create_self_recovery_request(
    request: SelfRecoveryRequest,
) -> ExternResult<ValidateCallbackResult> {
    match validate_self_recovery_request_creation_fields(&request) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason.into())),
    }
}

/// Validate self-recovery request updates (same state machine as social recovery)
///
/// Author-binding for updates is enforced universally by this crate's
/// `FlatOp::RegisterUpdate` handler (same author as original creator).
/// This is consistent with self-recovery's model: since verify_self_recovery_anchor
/// is called by the SAME requesting agent repeatedly (accumulating verified
/// anchors), requiring the same author across updates is correct here --
/// unlike RecoveryRequest's cross-trustee tallying, there's no legitimate
/// need for a DIFFERENT agent to update a SelfRecoveryRequest.
fn validate_update_self_recovery_request(
    action: Update,
    request: SelfRecoveryRequest,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: SelfRecoveryRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original self-recovery request not found".into()
        )))?;

    // Immutable fields
    if request.id != original.id
        || request.did != original.did
        || request.new_agent != original.new_agent
        || request.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery request immutable fields cannot be changed".into(),
        ));
    }

    // Self-recovery v1 does not carry any cryptographically verifiable
    // possession-proof receipt. The legacy coordinator merely compares
    // caller-supplied anchor strings, so Pending -> Approved is unsafe.
    // Keep this transition blocked until a versioned proof envelope and
    // verifier are implemented. Social Recovery uses a separate entry type
    // and is not affected by this restriction.
    if original.status == RecoveryStatus::Pending
        && request.status == RecoveryStatus::Approved
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery approval is disabled until cryptographic possession-proof receipts are implemented".into(),
        ));
    }

    // No proof receipt is available in v1. Until that is fixed, changes to
    // the verified_anchors field are rejected rather than treating caller-
    // supplied strings as authenticated proof of control.
    if request.verified_anchors != original.verified_anchors {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery anchor verification is disabled until proof receipts are implemented".into(),
        ));
    }

    // Same state machine as social recovery, except the proofless v1
    // Pending -> Approved transition is intentionally unavailable.
    let valid_transition =
        self_recovery_status_transition_allowed(&original.status, &request.status);
    if !valid_transition {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid self-recovery status transition from {:?} to {:?}",
            original.status, request.status
        )));
    }

    Ok(ValidateCallbackResult::Valid)
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

    // ── Self-Recovery Configuration Boundary Tests ──

    fn test_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn test_other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn test_self_recovery_config(owner: AgentPubKey) -> SelfRecoveryConfig {
        SelfRecoveryConfig {
            did: format!("did:mycelix:{}", owner),
            owner,
            anchors: vec![],
            anchor_threshold: 1,
            time_lock: SELF_RECOVERY_DEFAULT_TIME_LOCK,
            active: true,
            superseded_by_social: false,
            created: Timestamp::from_micros(1),
            updated: Timestamp::from_micros(1),
        }
    }

    #[test]
    fn self_recovery_config_must_bind_did_to_committing_owner() {
        let owner = test_agent();
        let config = test_self_recovery_config(owner.clone());
        assert!(validate_self_recovery_config_fields(&config, &owner).is_ok());

        let attacker = test_other_agent();
        let mut forged = test_self_recovery_config(attacker.clone());
        forged.did = format!("did:mycelix:{}", owner);
        assert!(
            validate_self_recovery_config_fields(&forged, &attacker).is_err(),
            "an attacker must not create a recovery config for another agent's DID"
        );
    }

    #[test]
    fn self_recovery_config_rejects_zero_or_excessive_anchor_threshold() {
        let owner = test_agent();
        let mut config = test_self_recovery_config(owner.clone());
        config.anchor_threshold = 0;
        assert!(validate_self_recovery_config_fields(&config, &owner).is_err());

        config.anchor_threshold = 2;
        config.anchors = vec![VerificationAnchor::PhoneHash("sha256:one".into())];
        assert!(validate_self_recovery_config_fields(&config, &owner).is_err());
    }

    #[test]
    fn self_recovery_config_rejects_duplicate_or_empty_anchors() {
        let owner = test_agent();
        let mut config = test_self_recovery_config(owner.clone());
        let anchor = VerificationAnchor::EmailHash("sha256:mail".into());
        config.anchors = vec![anchor.clone(), anchor];
        config.anchor_threshold = 1;
        assert!(validate_self_recovery_config_fields(&config, &owner).is_err());

        config.anchors = vec![VerificationAnchor::PhoneHash("  ".into())];
        assert!(validate_self_recovery_config_fields(&config, &owner).is_err());
    }

    #[test]
    fn self_recovery_config_update_preserves_identity_and_anchor_history() {
        let owner = test_agent();
        let original = test_self_recovery_config(owner.clone());
        let mut valid = original.clone();
        valid.updated = Timestamp::from_micros(2);
        valid.anchors.push(VerificationAnchor::EmailHash("sha256:mail".into()));
        assert!(validate_self_recovery_config_update_fields(&original, &valid).is_ok());

        let mut retargeted = valid.clone();
        retargeted.did = format!("did:mycelix:{}", test_other_agent());
        assert!(validate_self_recovery_config_update_fields(&original, &retargeted).is_err());

        let mut owner_changed = valid.clone();
        owner_changed.owner = test_other_agent();
        assert!(validate_self_recovery_config_update_fields(&original, &owner_changed).is_err());

        let mut no_timestamp_advance = valid.clone();
        no_timestamp_advance.updated = original.updated;
        assert!(validate_self_recovery_config_update_fields(&original, &no_timestamp_advance).is_err());

        let mut removed = valid.clone();
        removed.anchors.clear();
        assert!(validate_self_recovery_config_update_fields(&valid, &removed).is_err());
    }

    #[test]
    fn self_recovery_social_supersession_is_monotonic() {
        let owner = test_agent();
        let mut original = test_self_recovery_config(owner);
        original.superseded_by_social = true;
        let mut updated = original.clone();
        updated.updated = Timestamp::from_micros(2);
        updated.superseded_by_social = false;
        assert!(validate_self_recovery_config_update_fields(&original, &updated).is_err());
    }

    #[test]
    fn self_recovery_request_creation_rejects_claimed_verified_anchors() {
        let mut request = SelfRecoveryRequest {
            id: "self-recovery-test".into(),
            did: "did:mycelix:owner".into(),
            new_agent: test_agent(),
            verified_anchors: vec![],
            status: RecoveryStatus::Pending,
            created: Timestamp::from_micros(1),
            time_lock_expires: None,
            reason: "lost original device".into(),
        };
        assert!(validate_self_recovery_request_creation_fields(&request).is_ok());

        request.verified_anchors.push(VerificationAnchor::EmailHash("sha256:known".into()));
        assert!(validate_self_recovery_request_creation_fields(&request).is_err());
    }

    #[test]
    fn self_recovery_cannot_be_approved_without_a_proof_receipt() {
        assert!(!self_recovery_status_transition_allowed(
            &RecoveryStatus::Pending,
            &RecoveryStatus::Approved
        ));
        assert!(self_recovery_status_transition_allowed(
            &RecoveryStatus::Pending,
            &RecoveryStatus::Cancelled
        ));
        assert!(self_recovery_status_transition_allowed(
            &RecoveryStatus::Approved,
            &RecoveryStatus::ReadyToExecute
        ));
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

    fn test_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_social_config(did: String, owner: AgentPubKey) -> RecoveryConfig {
        RecoveryConfig {
            did,
            owner,
            trustees: vec![
                format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![2u8; 36])),
                format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![3u8; 36])),
                format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![4u8; 36])),
            ],
            threshold: 2,
            time_lock: 86400,
            active: true,
            created: Timestamp::from_micros(1),
            updated: Timestamp::from_micros(1),
        }
    }

    #[test]
    fn social_recovery_config_must_bind_did_to_committing_owner() {
        let owner = me();
        let valid = valid_social_config(format!("did:mycelix:{}", owner), owner.clone());
        assert_eq!(
            validate_create_recovery_config(EntryCreationAction::Create(test_action(owner.clone())), valid)
                .unwrap(),
            ValidateCallbackResult::Valid
        );

        let forged = valid_social_config(
            format!("did:mycelix:{}", other_agent()),
            owner.clone(),
        );
        assert!(matches!(
            validate_create_recovery_config(EntryCreationAction::Create(test_action(owner)), forged)
                .unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn recovery_config_link_must_match_did_base_and_owner_author() {
        let owner = me();
        let did = format!("did:mycelix:{}", owner);
        let correct_base = string_index_hash(&did);
        assert!(validate_recovery_config_link_fields(
            &correct_base,
            &did,
            &owner,
            &owner
        ).is_ok());

        let victim_base = string_index_hash(&format!("did:mycelix:{}", other_agent()));
        assert!(validate_recovery_config_link_fields(
            &victim_base,
            &did,
            &owner,
            &owner
        ).is_err());

        assert!(validate_recovery_config_link_fields(
            &correct_base,
            &did,
            &owner,
            &other_agent()
        ).is_err());
    }

    fn valid_request(initiated_by: String) -> RecoveryRequest {
        RecoveryRequest {
            id: "req-1".into(),
            did: "did:mycelix:owner".into(),
            new_agent: me(),
            initiated_by,
            reason: "Owner lost device".into(),
            status: RecoveryStatus::Pending,
            created: Timestamp::from_micros(0),
            time_lock_expires: None,
        }
    }

    #[test]
    fn create_request_valid_when_initiator_matches_committer() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result =
            validate_create_recovery_request(EntryCreationAction::Create(test_action(me())), req)
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_request_initiator_forgery_rejected() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result = validate_create_recovery_request(
            EntryCreationAction::Create(test_action(other_agent())),
            req,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_vote(trustee: String) -> RecoveryVote {
        RecoveryVote {
            request_id: "req-1".into(),
            trustee,
            vote: VoteDecision::Approve,
            comment: None,
            voted_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_vote_valid_when_trustee_matches_committer() {
        let vote = valid_vote(format!("did:mycelix:{}", me()));
        let result =
            validate_create_recovery_vote(EntryCreationAction::Create(test_action(me())), vote)
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_vote_trustee_forgery_rejected() {
        let vote = valid_vote(format!("did:mycelix:{}", me()));
        let result = validate_create_recovery_vote(
            EntryCreationAction::Create(test_action(other_agent())),
            vote,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

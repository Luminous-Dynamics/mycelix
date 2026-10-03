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
    /// Exact recovery configuration snapshot governing this request.
    pub recovery_config_action_hash: ActionHash,
    /// Reason for recovery
    pub reason: String,
    /// Current status
    pub status: RecoveryStatus,
    /// When the request was created
    pub created: Timestamp,
    /// When time lock expires (if approved)
    pub time_lock_expires: Option<Timestamp>,
    /// Deterministic certificate proving which trustee votes authorized approval.
    pub approval_certificate: Option<ActionHash>,
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
    /// Informational application timestamp; never authoritative for quorum
    /// ordering or security decisions. Signed source-chain action metadata is
    /// authoritative for ordering and certificate timing.
    pub voted_at: Timestamp,
}

/// Deterministic quorum certificate for a social recovery approval.
///
/// Every referenced vote is an immutable DHT record. The certificate itself is
/// authored by the recovery-request author so the eventual RecoveryRequest
/// update can remain within Holochain's source-chain authorship model.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RecoveryApprovalCertificate {
    pub request_id: String,
    pub request_action_hash: ActionHash,
    pub recovery_config_action_hash: ActionHash,
    pub vote_action_hashes: Vec<ActionHash>,
    pub threshold: u32,
    /// Informational issue timestamp. Authorization timing uses the signed
    /// certificate action timestamp.
    pub issued_at: Timestamp,
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
    RecoveryApprovalCertificate(RecoveryApprovalCertificate),
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
    /// Deterministic request-id index for cross-agent lookup
    RecoveryRequestIdToRequest,
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
                EntryTypes::RecoveryApprovalCertificate(certificate) => {
                    validate_create_recovery_approval_certificate(
                        EntryCreationAction::Create(action),
                        certificate,
                    )
                }
                EntryTypes::SelfRecoveryConfig(config) => validate_create_self_recovery_config(
                    EntryCreationAction::Create(action),
                    config,
                ),
                EntryTypes::SelfRecoveryRequest(request) => {
                    validate_create_self_recovery_request(EntryCreationAction::Create(action), request)
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
            base_address,
            target_address,
            link_type,
            tag,
            action,
        } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            match link_type {
                LinkTypes::DidToRecoveryConfig
                | LinkTypes::DidToRecoveryRequest
                | LinkTypes::RequestToVotes
                | LinkTypes::TrusteeToConfig
                | LinkTypes::DidToSelfRecoveryConfig
                | LinkTypes::DidToSelfRecoveryRequest
                | LinkTypes::RecoveryRequestIdToRequest => {
                    validate_recovery_link(link_type, &base_address, &target_address, &action)
                },
            }
        }
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            link_type,
            ..
        } => {
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the link creator can delete their links".into(),
                ));
            }

            // Recovery links are part of the security state. In particular,
            // deleting RequestToVotes can make an immutable trustee vote
            // disappear from DHT-derived quorum, and deleting the DID/request
            // indexes can hide otherwise valid recovery state. New delete-link
            // operations for this integrity zome therefore fail closed.
            match link_type {
                LinkTypes::DidToRecoveryConfig
                | LinkTypes::DidToRecoveryRequest
                | LinkTypes::RequestToVotes
                | LinkTypes::TrusteeToConfig
                | LinkTypes::DidToSelfRecoveryConfig
                | LinkTypes::DidToSelfRecoveryRequest
                | LinkTypes::RecoveryRequestIdToRequest => Ok(ValidateCallbackResult::Invalid(
                    "Recovery security links cannot be deleted".into(),
                )),
            }
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        // RegisterAgentActivity is the chain-authority validation boundary.
        // A malicious coordinator can bypass coordinator-side duplicate-vote
        // checks, so enforce one RecoveryVote per request on the signer's
        // cryptographically ordered source chain rather than relying on DHT
        // link order or self-reported timestamps.
        FlatOp::RegisterAgentActivity(activity) => match activity {
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::RecoveryConfig),
                action,
            } => validate_recovery_config_chain_uniqueness(action),
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::RecoveryRequest),
                action,
            } => validate_recovery_request_chain_uniqueness(action),
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::RecoveryVote),
                action,
            } => validate_recovery_vote_chain_uniqueness(action),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update) => {
            match update {
                OpUpdate::Entry { app_entry, action, .. } => {
                    let original = must_get_action(action.original_action_address.clone())?;
                    if *original.action().author() != action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the original entry author can update their entries".into(),
                        ));
                    }

                    match app_entry {
                        EntryTypes::RecoveryConfig(config) => {
                            validate_update_recovery_config(action, config)
                        }
                        EntryTypes::RecoveryRequest(request) => {
                            validate_update_recovery_request(action, request)
                        }
                        EntryTypes::RecoveryVote(_) | EntryTypes::RecoveryApprovalCertificate(_) => {
                            Ok(ValidateCallbackResult::Invalid(
                                "Recovery votes and approval certificates are immutable".into(),
                            ))
                        }
                        EntryTypes::SelfRecoveryConfig(config) => {
                            validate_update_self_recovery_config(action, config)
                        }
                        EntryTypes::SelfRecoveryRequest(request) => {
                            validate_update_self_recovery_request(action, request)
                        }
                    }
                }
                OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => {
                    let original = must_get_action(action.original_action_address.clone())?;
                    if *original.action().author() != action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Only the original entry author can update their entries".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
            }
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }

            match original.action().entry_type() {
                Some(EntryType::App(entry_def))
                    if *entry_def == UnitEntryTypes::RecoveryConfig.into()
                        || *entry_def == UnitEntryTypes::RecoveryRequest.into()
                        || *entry_def == UnitEntryTypes::RecoveryVote.into()
                        || *entry_def == UnitEntryTypes::RecoveryApprovalCertificate.into()
                        || *entry_def == UnitEntryTypes::SelfRecoveryConfig.into()
                        || *entry_def == UnitEntryTypes::SelfRecoveryRequest.into() =>
                {
                    Ok(ValidateCallbackResult::Invalid(
                        "Recovery security entries cannot be deleted".into(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }
    }
}

fn string_to_entry_hash(value: &str) -> EntryHash {
    let bytes = holo_hash::blake2b_256(value.as_bytes())
        .into_iter()
        .chain([0u8; 4])
        .collect::<Vec<u8>>();
    EntryHash::from_raw_36(bytes)
}

fn did_to_agent(did: &str) -> Option<AgentPubKey> {
    did.strip_prefix("did:mycelix:")
        .and_then(|value| AgentPubKey::try_from(value).ok())
}

fn validate_recovery_link(
    link_type: LinkTypes,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let base = match base_address.clone().into_entry_hash() {
        Some(base) => base,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery link base must be an EntryHash".into(),
            ));
        }
    };

    let target_action = match target_address.clone().into_action_hash() {
        Some(target) => target,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery link target must be an ActionHash".into(),
            ));
        }
    };

    let record = must_get_valid_record(target_action)?;
    match link_type {
        LinkTypes::DidToRecoveryConfig => {
            let config: RecoveryConfig = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Recovery config link target must decode as RecoveryConfig".into()
                )))?;
            if string_to_entry_hash(&config.did) != base {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToRecoveryConfig base does not match target DID".into(),
                ));
            }
            if action.author != config.owner {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToRecoveryConfig link must be authored by the recovery owner".into(),
                ));
            }
        }
        LinkTypes::DidToRecoveryRequest => {
            let request: RecoveryRequest = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Recovery request link target must decode as RecoveryRequest".into()
                )))?;
            if string_to_entry_hash(&request.did) != base {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToRecoveryRequest base does not match target DID".into(),
                ));
            }
            let initiator = did_to_agent(&request.initiated_by).ok_or(wasm_error!(WasmErrorInner::Guest(
                "Recovery request initiator must be a valid did:mycelix identifier".into()
            )))?;
            if action.author != initiator {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToRecoveryRequest link must be authored by the request initiator".into(),
                ));
            }
        }
        LinkTypes::RecoveryRequestIdToRequest => {
            let request: RecoveryRequest = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Recovery request index target must decode as RecoveryRequest".into()
                )))?;
            if string_to_entry_hash(&request.id) != base {
                return Ok(ValidateCallbackResult::Invalid(
                    "Recovery request index base does not match target request ID".into(),
                ));
            }
            let initiator = did_to_agent(&request.initiated_by).ok_or(wasm_error!(WasmErrorInner::Guest(
                "Recovery request initiator must be a valid did:mycelix identifier".into()
            )))?;
            if action.author != initiator {
                return Ok(ValidateCallbackResult::Invalid(
                    "Recovery request index link must be authored by the request initiator".into(),
                ));
            }
        }
        LinkTypes::RequestToVotes => {
            let vote: RecoveryVote = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "RequestToVotes target must decode as RecoveryVote".into()
                )))?;
            if string_to_entry_hash(&vote.request_id) != base {
                return Ok(ValidateCallbackResult::Invalid(
                    "RequestToVotes base does not match target request ID".into(),
                ));
            }
            let trustee = did_to_agent(&vote.trustee).ok_or(wasm_error!(WasmErrorInner::Guest(
                "Recovery vote trustee must be a valid did:mycelix identifier".into()
            )))?;
            if action.author != trustee {
                return Ok(ValidateCallbackResult::Invalid(
                    "RequestToVotes link must be authored by the voting trustee".into(),
                ));
            }
        }
        LinkTypes::TrusteeToConfig => {
            let config: RecoveryConfig = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "TrusteeToConfig target must decode as RecoveryConfig".into()
                )))?;
            if !config
                .trustees
                .iter()
                .any(|trustee| string_to_entry_hash(trustee) == base)
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "TrusteeToConfig base does not match a configured trustee".into(),
                ));
            }
            if action.author != config.owner {
                return Ok(ValidateCallbackResult::Invalid(
                    "TrusteeToConfig link must be authored by the recovery config owner".into(),
                ));
            }
        }
        LinkTypes::DidToSelfRecoveryConfig => {
            let config: SelfRecoveryConfig = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Self-recovery config link target must decode as SelfRecoveryConfig".into()
                )))?;
            if string_to_entry_hash(&config.did) != base {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToSelfRecoveryConfig base does not match target DID".into(),
                ));
            }
            if action.author != config.owner {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToSelfRecoveryConfig link must be authored by the self-recovery owner".into(),
                ));
            }
        }
        LinkTypes::DidToSelfRecoveryRequest => {
            let request: SelfRecoveryRequest = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Self-recovery request link target must decode as SelfRecoveryRequest".into()
                )))?;
            if string_to_entry_hash(&request.did) != base {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToSelfRecoveryRequest base does not match target DID".into(),
                ));
            }
            if action.author != request.new_agent {
                return Ok(ValidateCallbackResult::Invalid(
                    "DidToSelfRecoveryRequest link must be authored by the replacement agent".into(),
                ));
            }
        }
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

    // Recovery configuration belongs to the DID's controller.
    if config.owner != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must be the author".into(),
        ));
    }
    let expected_did = format!("did:mycelix:{}", action.author());
    if config.did != expected_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery configuration DID must match the author's canonical DID".into(),
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

    // Recovery trustees participate in a Mycelix-specific authority protocol;
    // accepting arbitrary DID methods here would make the quorum semantics
    // impossible to bind to a concrete Holochain signer.
    for trustee in &config.trustees {
        if did_to_agent(trustee).is_none() {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Trustee must be a valid did:mycelix AgentPubKey DID: {}",
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

    for trustee in &config.trustees {
        if did_to_agent(trustee).is_none() {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Trustee must be a valid did:mycelix AgentPubKey DID: {}",
                trustee
            )));
        }
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

/// Validate a quorum certificate using only deterministic DHT dependencies.
fn validate_create_recovery_approval_certificate(
    action: EntryCreationAction,
    certificate: RecoveryApprovalCertificate,
) -> ExternResult<ValidateCallbackResult> {
    if certificate.vote_action_hashes.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate must cite at least one vote".into(),
        ));
    }
    if certificate.threshold == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate threshold must be greater than zero".into(),
        ));
    }
    if certificate.issued_at.as_micros() <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate timestamp must be positive".into(),
        ));
    }

    // The signed Create action is the authoritative certificate clock.
    // `issued_at` is retained as an informational compatibility field and
    // must not be allowed to move authorization time independently.
    let certificate_action_timestamp = *action.timestamp();
    if certificate.issued_at > certificate_action_timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate issued_at cannot be later than its signed action timestamp".into(),
        ));
    }

    if *action.author() != *must_get_action(certificate.request_action_hash.clone())?.action().author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate must be authored by the request author".into(),
        ));
    }

    let request_action = must_get_action(certificate.request_action_hash.clone())?;
    if !matches!(request_action.action().data, ActionData::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate must bind to the original RecoveryRequest creation action".into(),
        ));
    }

    let request_record = must_get_valid_record(certificate.request_action_hash.clone())?;
    let request: RecoveryRequest = request_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Certificate request action must reference a RecoveryRequest".into()
        )))?;

    if request.id != certificate.request_id {
        return Ok(ValidateCallbackResult::Invalid(
            "Certificate request_id does not match the referenced RecoveryRequest".into(),
        ));
    }

    let config_action = must_get_action(certificate.recovery_config_action_hash.clone())?;
    if !matches!(config_action.action().data, ActionData::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate must reference the original RecoveryConfig creation action".into(),
        ));
    }
    let config_record = must_get_valid_record(certificate.recovery_config_action_hash.clone())?;
    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Certificate config action must reference a RecoveryConfig".into()
        )))?;

    if config.did != request.did || config.owner != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Certificate recovery config does not match the request or author".into(),
        ));
    }
    if config.threshold != certificate.threshold {
        return Ok(ValidateCallbackResult::Invalid(
            "Certificate threshold does not match the recovery configuration".into(),
        ));
    }

    if let Some(expires) = request.time_lock_expires {
        let duration_micros = (config.time_lock as i64)
            .checked_mul(1_000_000)
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Recovery time-lock duration overflow".into()
            )))?;
        let expected_expiry = certificate_action_timestamp
            .as_micros()
            .checked_add(duration_micros)
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Recovery time-lock expiry overflow".into()
            )))?;
        if expires.as_micros() != expected_expiry {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery time-lock expiry must equal certificate issuance plus the pinned policy duration".into(),
            ));
        }
    }

    if certificate.issued_at < request.created {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate cannot predate the request".into(),
        ));
    }

    // Certificates are canonical: exactly threshold approvals, ordered by
    // trustee DID. This matches coordinator-side certificate construction and
    // prevents two different serializations from representing the same quorum.
    if certificate.vote_action_hashes.len() != certificate.threshold as usize {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate must cite exactly threshold approval votes".into(),
        ));
    }

    let mut seen_trustees = HashSet::new();
    let mut previous_trustee: Option<String> = None;
    for vote_hash in &certificate.vote_action_hashes {
        let vote_record = must_get_valid_record(vote_hash.clone())?;
        let vote: RecoveryVote = vote_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Certificate vote action must reference a RecoveryVote".into()
            )))?;

        if vote.request_id != certificate.request_id {
            return Ok(ValidateCallbackResult::Invalid(
                "Certificate contains a vote for a different request".into(),
            ));
        }
        if vote.vote != VoteDecision::Approve {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery approval certificates may only cite approval votes".into(),
            ));
        }
        if !config.trustees.contains(&vote.trustee) {
            return Ok(ValidateCallbackResult::Invalid(
                "Certificate contains a vote from a non-trustee".into(),
            ));
        }
        if !seen_trustees.insert(vote.trustee.clone()) {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery approval certificate contains duplicate trustees".into(),
            ));
        }
        if let Some(previous) = previous_trustee.as_ref() {
            if vote.trustee <= *previous {
                return Ok(ValidateCallbackResult::Invalid(
                    "Recovery approval certificate vote hashes must be ordered by trustee DID".into(),
                ));
            }
        }
        previous_trustee = Some(vote.trustee.clone());

        // Vote timing is derived from the signed action header, not the
        // compatibility `voted_at` field.
        if vote_record.action().timestamp() > certificate_action_timestamp {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery approval certificate cannot predate a cited approval vote".into(),
            ));
        }

        let trustee = did_to_agent(&vote.trustee).ok_or(wasm_error!(WasmErrorInner::Guest(
            "Certificate vote trustee must be a valid did:mycelix identifier".into()
        )))?;
        if *vote_record.action().author() != trustee {
            return Ok(ValidateCallbackResult::Invalid(
                "Certificate vote must be authored by its claimed trustee".into(),
            ));
        }

        match validate_recovery_vote_is_first_for_request(&vote_record, &vote)? {
            ValidateCallbackResult::Valid => {}
            invalid => return Ok(invalid),
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Reject certificates that cite a later duplicate vote while ignoring an
/// earlier request-specific vote from the same trustee.
///
/// This closes the legacy-data escape hatch left by coordinator-side
/// canonicalization: authorization cannot be recovered by simply selecting
/// a later approval if the trustee's source chain already contains a vote
/// for the same request.
fn validate_recovery_vote_is_first_for_request(
    vote_record: &Record,
    vote: &RecoveryVote,
) -> ExternResult<ValidateCallbackResult> {
    let Action::Create(vote_create) = vote_record.action().action() else {
        return Ok(ValidateCallbackResult::Invalid(
            "Certificate vote must reference a create action".into(),
        ));
    };

    let activity = must_get_agent_activity(
        vote_create.author.clone(),
        ChainFilter::new(vote_create.prev_action.clone()),
    )?;

    let recovery_vote_entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::RecoveryVote)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };

        if prior_create.entry_type != recovery_vote_entry_type {
            continue;
        }

        // The action itself is already cryptographically authenticated by
        // must_get_agent_activity. Read only the referenced entry here so this
        // check does not recursively revalidate the same chain rule.
        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior_vote: RecoveryVote = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "RecoveryVote history entry could not be decoded: {e}"
            )))
        })?;

        if prior_vote.request_id == vote.request_id {
            return Ok(ValidateCallbackResult::Invalid(
                "Recovery approval certificate must cite the first vote cast by each trustee for the request".into(),
            ));
        }
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

    let config_action = must_get_action(request.recovery_config_action_hash.clone())?;
    if !matches!(config_action.action().data, ActionData::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request must pin the original RecoveryConfig creation action".into(),
        ));
    }
    let config_record = must_get_valid_record(request.recovery_config_action_hash.clone())?;
    let config: RecoveryConfig = config_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Recovery request config snapshot must reference a RecoveryConfig".into()
        )))?;
    if config.did != request.did
        || config.owner != *action.author()
        || !config.trustees.contains(&request.initiated_by)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request config snapshot does not authorize its DID and initiator".into(),
        ));
    }

    // Bind request identity to immutable request fields. This prevents a
    // modified coordinator from choosing arbitrary identifiers that could make
    // two logically distinct requests share a quorum namespace.
    let expected_request_id = format!(
        "recovery:{}:{}",
        request.did,
        request.created.as_micros()
    );
    if request.id != expected_request_id {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request ID must be derived from DID and creation timestamp".into(),
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
/// Recovery-request quorum evaluation is intentionally DHT-derived in the
/// coordinator. The request lookup now resolves through the request-ID index
/// before falling back to the caller's legacy local chain, so cross-agent
/// threshold observation does not depend on the trustee who authored the vote.
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
    if request.recovery_config_action_hash != original.recovery_config_action_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request recovery-config snapshot cannot be changed".into(),
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
    if request.reason != original.reason {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery request reason cannot be changed".into(),
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

    // Once a request has a quorum certificate or time lock, those
    // authorization artifacts are immutable for the remainder of the request.
    if original.approval_certificate.is_some()
        && request.approval_certificate != original.approval_certificate
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate cannot be changed after approval".into(),
        ));
    }
    if original.time_lock_expires.is_some()
        && request.time_lock_expires != original.time_lock_expires
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery time lock expiry cannot be changed after it is armed".into(),
        ));
    }

    // Approval and execution states require an immutable quorum certificate.
    if matches!(
        request.status,
        RecoveryStatus::Approved | RecoveryStatus::ReadyToExecute | RecoveryStatus::Completed
    ) {
        let certificate_hash = request.approval_certificate.clone().ok_or(
            wasm_error!(WasmErrorInner::Guest(
                "Approved recovery requires an approval certificate".into()
            ))
        )?;
        let certificate_record = must_get_valid_record(certificate_hash)?;
        let certificate: RecoveryApprovalCertificate = certificate_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Approval certificate reference must decode as RecoveryApprovalCertificate".into()
            )))?;
        if certificate.request_id != request.id {
            return Ok(ValidateCallbackResult::Invalid(
                "Approval certificate request_id does not match RecoveryRequest".into(),
            ));
        }
        // The certificate binds to the immutable request's creation action.
        // Later state-machine transitions may validly update the request again
        // while retaining the same approval certificate.
    }

    if request.status == RecoveryStatus::Pending
        && (request.approval_certificate.is_some() || request.time_lock_expires.is_some())
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Pending recovery cannot carry approval or time-lock artifacts".into(),
        ));
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
    let expected_did = format!("did:mycelix:{}", action.author());
    if config.did != expected_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery configuration DID must match the author's canonical DID".into(),
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

/// Validate updates to self-recovery configuration at the integrity boundary.
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
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery config identity fields cannot be changed".into(),
        ));
    }

    if config.updated <= original.updated {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery config updated timestamp must advance".into(),
        ));
    }

    if config.time_lock < SELF_RECOVERY_MIN_TIME_LOCK {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Self-recovery time lock must be at least {} seconds",
            SELF_RECOVERY_MIN_TIME_LOCK
        )));
    }

    let unique_anchor_count = {
        let mut set = HashSet::new();
        for anchor in &config.anchors {
            let encoded = serde_json::to_string(anchor)
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            set.insert(encoded);
        }
        set.len()
    };
    if unique_anchor_count != config.anchors.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Duplicate self-recovery anchors are not allowed".into(),
        ));
    }

    if config.anchor_threshold == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery anchor threshold must be greater than zero".into(),
        ));
    }
    if !config.anchors.is_empty() && config.anchor_threshold as usize > config.anchors.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery anchor threshold cannot exceed anchor count".into(),
        ));
    }

    // Once disabled or superseded, configuration cannot silently become active
    // again through a generic update.
    if original.active == false && config.active {
        return Ok(ValidateCallbackResult::Invalid(
            "Inactive self-recovery configuration cannot be reactivated".into(),
        ));
    }
    if original.superseded_by_social && !config.superseded_by_social {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery supersession cannot be reversed".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate self-recovery request creation
///
/// Self-recovery deliberately allows the DID subject to differ from the
/// replacement agent because the original controller may be offline. The
/// replacement agent, however, is the author of the request and therefore is
/// bound to request.new_agent by the integrity rule below.
fn validate_create_self_recovery_request(
    action: EntryCreationAction,
    request: SelfRecoveryRequest,
) -> ExternResult<ValidateCallbackResult> {
    if request.new_agent != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery request new_agent must be the committing agent".into(),
        ));
    }
    if !request.did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:mycelix:'".into(),
        ));
    }
    if request.reason.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery reason is required".into(),
        ));
    }
    if request.status != RecoveryStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "Initial status must be Pending".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Whether a self-recovery status transition is permitted while proof-of-control is disabled.
pub fn self_recovery_transition_allowed(
    from: &RecoveryStatus,
    to: &RecoveryStatus,
) -> bool {
    matches!(
        (from, to),
        (RecoveryStatus::Pending, RecoveryStatus::Pending)
            | (RecoveryStatus::Pending, RecoveryStatus::Cancelled)
    )
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

    // Immutable request identity.
    if request.id != original.id
        || request.did != original.did
        || request.new_agent != original.new_agent
        || request.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery request immutable fields cannot be changed".into(),
        ));
    }

    // Until cryptographic proof-of-control exists, self-recovery is deliberately
    // prevented from entering any executable state, even if a modified
    // coordinator attempts to update the entry directly.
    let allowed_transition = self_recovery_transition_allowed(
        &original.status,
        &request.status,
    );
    if !allowed_transition {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Self-recovery transition {:?} -> {:?} is disabled until cryptographic proof-of-control exists",
            original.status, request.status
        )));
    }

    if request.time_lock_expires != original.time_lock_expires {
        return Ok(ValidateCallbackResult::Invalid(
            "Self-recovery time lock cannot be changed while proof-of-control is disabled".into(),
        ));
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

    // ── Self-Recovery Tests ──

    #[test]
    fn self_recovery_transition_predicate_is_fail_closed() {
        assert!(self_recovery_transition_allowed(
            &RecoveryStatus::Pending,
            &RecoveryStatus::Pending,
        ));
        assert!(self_recovery_transition_allowed(
            &RecoveryStatus::Pending,
            &RecoveryStatus::Cancelled,
        ));
        for target in [
            RecoveryStatus::Approved,
            RecoveryStatus::ReadyToExecute,
            RecoveryStatus::Completed,
            RecoveryStatus::Rejected,
        ] {
            assert!(!self_recovery_transition_allowed(
                &RecoveryStatus::Pending,
                &target,
            ));
        }
        assert!(!self_recovery_transition_allowed(
            &RecoveryStatus::Approved,
            &RecoveryStatus::ReadyToExecute,
        ));
    }

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
    fn self_recovery_request_status_machine_is_fail_closed() {
        assert!(self_recovery_transition_allowed(
            &RecoveryStatus::Pending,
            &RecoveryStatus::Pending,
        ));
        assert!(self_recovery_transition_allowed(
            &RecoveryStatus::Pending,
            &RecoveryStatus::Cancelled,
        ));
        assert!(!self_recovery_transition_allowed(
            &RecoveryStatus::Pending,
            &RecoveryStatus::Approved,
        ));
        assert!(!self_recovery_transition_allowed(
            &RecoveryStatus::Approved,
            &RecoveryStatus::ReadyToExecute,
        ));
        assert!(!self_recovery_transition_allowed(
            &RecoveryStatus::ReadyToExecute,
            &RecoveryStatus::Completed,
        ));
    }
}

/// Validate recovery vote creation
/// Enforce one RecoveryConfig creation per DID on the owner's source chain.
///
/// Coordinator-side existence checks are useful for availability, but a
/// modified coordinator can bypass them. The integrity rule makes duplicate
/// recovery configurations invalid immutable evidence.
fn validate_recovery_config_chain_uniqueness(
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    let current_entry = must_get_entry(action.entry_hash.clone())?;
    let current_config: RecoveryConfig = current_entry.try_into().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "RecoveryConfig entry could not be decoded: {e}"
        )))
    })?;

    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;

    let entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::RecoveryConfig)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };
        if prior_create.entry_type != entry_type {
            continue;
        }

        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior_config: RecoveryConfig = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "RecoveryConfig history entry could not be decoded: {e}"
            )))
        })?;

        if prior_config.did == current_config.did {
            return Ok(ValidateCallbackResult::Invalid(
                "A controller may create at most one recovery configuration for a DID".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Enforce one RecoveryRequest identity per request author.
///
/// The request ID is derived from DID + creation timestamp, so a timestamp
/// collision must not produce two independent quorum namespaces under the same
/// identifier.
fn validate_recovery_request_chain_uniqueness(
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    let current_entry = must_get_entry(action.entry_hash.clone())?;
    let current_request: RecoveryRequest = current_entry.try_into().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "RecoveryRequest entry could not be decoded: {e}"
        )))
    })?;

    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;

    let entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::RecoveryRequest)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };
        if prior_create.entry_type != entry_type {
            continue;
        }

        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior_request: RecoveryRequest = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "RecoveryRequest history entry could not be decoded: {e}"
            )))
        })?;

        if prior_request.id == current_request.id {
            return Ok(ValidateCallbackResult::Invalid(
                "A recovery request ID may only be created once by an initiator".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Enforce the one-vote-per-trustee-per-request invariant at the
/// chain-authority boundary.
///
/// Coordinator checks are availability/convenience protections only. A
/// modified coordinator can call `create_entry` directly, so the integrity
/// zome must inspect the signer's prior source-chain history and reject a
/// second immutable vote for the same request.
///
/// `ChainFilter::new(prev_action)` walks the signer chain backwards to
/// genesis, making the history dependency deterministic for validation.
fn validate_recovery_vote_chain_uniqueness(action: Create) -> ExternResult<ValidateCallbackResult> {
    let current_entry = must_get_entry(action.entry_hash.clone())?;
    let current_vote: RecoveryVote = current_entry.try_into()?;

    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;

    let recovery_vote_entry_type =
        EntryType::App(AppEntryDef::try_from(UnitEntryTypes::RecoveryVote)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(create) = prior_action else {
            continue;
        };

        if create.entry_type != recovery_vote_entry_type {
            continue;
        }

        // The action itself is already cryptographically authenticated by
        // must_get_agent_activity. Read only the referenced entry here so this
        // check does not recursively revalidate the same chain rule.
        let prior_entry = must_get_entry(create.entry_hash.clone())?;
        let prior_vote: RecoveryVote = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "RecoveryVote history entry could not be decoded: {e}"
            )))
        })?;

        if prior_vote.request_id == current_vote.request_id {
            return Ok(ValidateCallbackResult::Invalid(
                "A trustee may cast at most one recovery vote for a request".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

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

    // `voted_at` is retained for compatibility/display only. It is not
    // authoritative security state: quorum ordering is derived from the signed
    // source-chain action sequence, and Holochain action timestamps belong to
    // the action header rather than the application entry.
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
    fn recovery_config_rejects_foreign_did_method_trustee() {
        let author = me();
        let config = RecoveryConfig {
            did: format!("did:mycelix:{}", author),
            owner: author,
            trustees: vec![
                format!("did:mycelix:{}", me()),
                "did:key:z6Mkforeign".into(),
                format!("did:mycelix:{}", other_agent()),
            ],
            threshold: 2,
            time_lock: 7 * 24 * 3600,
            active: true,
            created: Timestamp::from_micros(0),
            updated: Timestamp::from_micros(1),
        };
        let result = validate_create_recovery_config(
            EntryCreationAction::Create(test_action(me())),
            config,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn recovery_config_must_name_author_did() {
        let author = me();
        let config = RecoveryConfig {
            did: "did:mycelix:other".into(),
            owner: author,
            trustees: vec![
                "did:mycelix:t1".into(),
                "did:mycelix:t2".into(),
                "did:mycelix:t3".into(),
            ],
            threshold: 2,
            time_lock: 7 * 24 * 3600,
            active: true,
            created: Timestamp::from_micros(0),
            updated: Timestamp::from_micros(1),
        };
        let result = validate_create_recovery_config(
            EntryCreationAction::Create(test_action(author)),
            config,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn self_recovery_request_must_target_committing_replacement_agent() {
        let mut request = SelfRecoveryRequest {
            id: "req-1".into(),
            did: "did:mycelix:owner".into(),
            new_agent: other_agent(),
            verified_anchors: vec![],
            status: RecoveryStatus::Pending,
            created: Timestamp::from_micros(0),
            time_lock_expires: None,
            reason: "lost device".into(),
        };
        let result = validate_create_self_recovery_request(
            EntryCreationAction::Create(test_action(me())),
            request.clone(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));

        request.new_agent = me();
        let result = validate_create_self_recovery_request(
            EntryCreationAction::Create(test_action(me())),
            request,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
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

    #[test]
    fn create_vote_treats_voted_at_as_non_authoritative_metadata() {
        let mut vote = valid_vote(format!("did:mycelix:{}", me()));
        vote.voted_at = Timestamp::from_micros(1_000_000_000);
        let action = test_action(me());

        let result =
            validate_create_recovery_vote(EntryCreationAction::Create(action), vote).unwrap();

        assert_eq!(result, ValidateCallbackResult::Valid);
    }
}

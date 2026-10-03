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
    /// Vote timestamp
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
                EntryTypes::SelfRecoveryConfig(_) => {
                    // Updates allowed (adding/removing anchors, marking superseded)
                    Ok(ValidateCallbackResult::Valid)
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

    if *action.author() != *must_get_action(certificate.request_action_hash.clone())?.action().author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate must be authored by the request author".into(),
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

    let mut seen_trustees = HashSet::new();
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

        let trustee = did_to_agent(&vote.trustee).ok_or(wasm_error!(WasmErrorInner::Guest(
            "Certificate vote trustee must be a valid did:mycelix identifier".into()
        )))?;
        if *vote_record.action().author() != trustee {
            return Ok(ValidateCallbackResult::Invalid(
                "Certificate vote must be authored by its claimed trustee".into(),
            ));
        }
    }

    if seen_trustees.len() < certificate.threshold as usize {
        return Ok(ValidateCallbackResult::Invalid(
            "Recovery approval certificate does not reach threshold".into(),
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
        if certificate.request_action_hash != action.original_action_address {
            return Ok(ValidateCallbackResult::Invalid(
                "Approval certificate must bind to the RecoveryRequest action being approved".into(),
            ));
        }
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

    // Same state machine as social recovery
    let valid_transition = match (&original.status, &request.status) {
        (RecoveryStatus::Pending, RecoveryStatus::Approved)
        | (RecoveryStatus::Pending, RecoveryStatus::Cancelled) => true,
        (RecoveryStatus::Approved, RecoveryStatus::ReadyToExecute)
        | (RecoveryStatus::Approved, RecoveryStatus::Cancelled) => true,
        (RecoveryStatus::ReadyToExecute, RecoveryStatus::Completed)
        | (RecoveryStatus::ReadyToExecute, RecoveryStatus::Cancelled) => true,
        (a, b) if a == b => true,
        _ => false,
    };
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
}

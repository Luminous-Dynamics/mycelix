// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Auditable emergency access to encrypted Hearth disclosures.
//!
//! Break-glass never publishes plaintext or unwrapped content keys. Guardians
//! approve an exact request scope, then publish a short-lived encrypted key
//! release addressed to one registered X25519 key. Consumption receipts are an
//! audit/nullifier mechanism; they cannot make already-released bytes
//! cryptographically disappear or prevent a malicious client from copying them.

use hdi::prelude::*;
use hearth_emergency_integrity::EmergencyAlert;
use hearth_kinship_integrity::HearthMembership;
use hearth_types::{MembershipStatus, validate_claimed_agent};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

use crate::{
    EncryptedContentKind, HearthEncryptedEnvelope, HearthEncryptionKey, HearthEnvelopeCipher,
    HearthKeyWrapAlgorithm, MAX_WRAPPED_KEY_BYTES,
};

pub const MAX_BREAK_GLASS_APPROVERS: usize = 16;
pub const MAX_BREAK_GLASS_SCOPE: usize = 16;
pub const MAX_BREAK_GLASS_POLICY_ID_BYTES: usize = 96;
pub const MAX_BREAK_GLASS_POLICY_TTL_SECS: i64 = 365 * 24 * 60 * 60;
pub const MAX_BREAK_GLASS_REQUEST_TTL_SECS: i64 = 30 * 60;
pub const MAX_BREAK_GLASS_ACCESS_TTL_SECS: i64 = 15 * 60;
pub const MIN_BREAK_GLASS_ACCESS_TTL_SECS: i64 = 60;
pub const MAX_BREAK_GLASS_CLOCK_SKEW_SECS: i64 = 300;
pub const MAX_BREAK_GLASS_RELEASE_CIPHERTEXT_BYTES: usize = 8_192;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EmergencyApprover {
    pub agent: AgentPubKey,
    /// Guardian membership configured when the policy is created.
    pub membership_hash: ActionHash,
}

/// Immutable guardian policy for one exact class of emergency access.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyAccessPolicy {
    pub hearth_hash: ActionHash,
    pub policy_id: String,
    pub version: u32,
    /// Only explicit built-in content classes are accepted in v1.
    pub allowed_content_kinds: Vec<EncryptedContentKind>,
    pub approvers: Vec<EmergencyApprover>,
    pub threshold: u8,
    pub max_access_ttl_secs: u32,
    pub authorization_membership_hash: ActionHash,
    pub created_by: AgentPubKey,
    pub created_at: Timestamp,
    pub expires_at: Timestamp,
}

/// One exact request bound to an alert, accessor, key, and encrypted records.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyAccessRequest {
    pub hearth_hash: ActionHash,
    pub request_id: [u8; 32],
    pub policy_hash: ActionHash,
    pub incident_alert_hash: ActionHash,
    pub requester: AgentPubKey,
    pub accessor: AgentPubKey,
    pub accessor_membership_hash: ActionHash,
    pub accessor_encryption_key_hash: ActionHash,
    pub requested_envelope_hashes: Vec<ActionHash>,
    /// Commitment to the human justification kept off-DHT or encrypted.
    pub reason_commitment: [u8; 32],
    /// Canonical AAD digest required on the encrypted key-release package.
    pub release_context_commitment: [u8; 32],
    /// Context-scoped nullifier used to audit one consumption attempt.
    pub consumption_nullifier: [u8; 32],
    pub requested_access_ttl_secs: u32,
    pub authorization_membership_hash: ActionHash,
    pub created_at: Timestamp,
    pub expires_at: Timestamp,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EmergencyApprovalDecision {
    Approve,
    Deny,
}

/// Immutable response by one policy approver.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyAccessApproval {
    pub hearth_hash: ActionHash,
    pub request_hash: ActionHash,
    pub policy_hash: ActionHash,
    pub approver: AgentPubKey,
    pub authorization_membership_hash: ActionHash,
    pub decision: EmergencyApprovalDecision,
    /// Commitment to optional private reasoning or an incident note.
    pub decision_commitment: [u8; 32],
    pub created_at: Timestamp,
}

/// Threshold-authorized, short-lived release of encrypted content keys.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyAccessGrant {
    pub hearth_hash: ActionHash,
    pub grant_id: [u8; 32],
    pub request_hash: ActionHash,
    pub policy_hash: ActionHash,
    pub incident_alert_hash: ActionHash,
    pub accessor: AgentPubKey,
    pub accessor_encryption_key_hash: ActionHash,
    pub requested_envelope_hashes: Vec<ActionHash>,
    /// Encrypted content-key package addressed only to the accessor key.
    pub release_cipher: HearthEnvelopeCipher,
    pub release_key_wrap: HearthKeyWrapAlgorithm,
    pub release_aad_digest: [u8; 32],
    pub release_plaintext_commitment: [u8; 32],
    pub release_nonce: [u8; 24],
    pub release_ephemeral_public_key: [u8; 32],
    pub release_ciphertext: Vec<u8>,
    pub release_wrapped_key: Vec<u8>,
    pub approval_hashes: Vec<ActionHash>,
    pub consumption_nullifier: [u8; 32],
    pub issuer_membership_hash: ActionHash,
    pub issued_by: AgentPubKey,
    pub issued_at: Timestamp,
    pub expires_at: Timestamp,
}

/// Accessor-authored audit record. Duplicate receipts are a conflict and make
/// the access state suspicious; DHT eventual consistency prevents a global
/// exactly-once guarantee.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyAccessReceipt {
    pub hearth_hash: ActionHash,
    pub receipt_id: [u8; 32],
    pub grant_hash: ActionHash,
    pub request_hash: ActionHash,
    pub accessor: AgentPubKey,
    pub consumption_nullifier: [u8; 32],
    pub local_audit_commitment: [u8; 32],
    pub consumed_at: Timestamp,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EmergencyReviewOutcome {
    Appropriate,
    Concern,
    PolicyChangeRequired,
    Unresolved,
}

/// Guardian-authored post-incident review without plaintext incident notes.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyAccessReview {
    pub hearth_hash: ActionHash,
    pub grant_hash: ActionHash,
    pub reviewer: AgentPubKey,
    pub reviewer_membership_hash: ActionHash,
    pub outcome: EmergencyReviewOutcome,
    pub notes_commitment: [u8; 32],
    pub reviewed_at: Timestamp,
}

pub fn validate_emergency_access_policy_create(
    policy: &EmergencyAccessPolicy,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let authorship = validate_claimed_agent(
        &action.author,
        &policy.created_by,
        "EmergencyAccessPolicy.created_by",
    );
    if authorship != ValidateCallbackResult::Valid {
        return Ok(authorship);
    }
    if !is_safe_identifier(&policy.policy_id, MAX_BREAK_GLASS_POLICY_ID_BYTES) {
        return invalid("Emergency access policy_id is malformed or too long");
    }
    if policy.version == 0 {
        return invalid("Emergency access policy version must be greater than zero");
    }
    if policy.allowed_content_kinds.is_empty() || policy.allowed_content_kinds.len() > 8 {
        return invalid("Emergency access policy must allow 1-8 content kinds");
    }
    for (index, kind) in policy.allowed_content_kinds.iter().enumerate() {
        if matches!(kind, EncryptedContentKind::Custom(_)) {
            return invalid("Break-glass v1 permits only built-in protected content kinds");
        }
        if policy.allowed_content_kinds[..index].contains(kind) {
            return invalid("Emergency access policy cannot duplicate content kinds");
        }
    }
    if policy.approvers.is_empty() || policy.approvers.len() > MAX_BREAK_GLASS_APPROVERS {
        return invalid("Emergency access policy must configure 1-16 approvers");
    }
    let unique_agents: HashSet<_> = policy.approvers.iter().map(|item| &item.agent).collect();
    let unique_memberships: HashSet<_> = policy
        .approvers
        .iter()
        .map(|item| &item.membership_hash)
        .collect();
    if unique_agents.len() != policy.approvers.len()
        || unique_memberships.len() != policy.approvers.len()
    {
        return invalid("Emergency access approvers and membership references must be unique");
    }
    if policy.threshold == 0 || policy.threshold as usize > policy.approvers.len() {
        return invalid("Emergency access threshold must be between 1 and approver count");
    }
    let max_access = policy.max_access_ttl_secs as i64;
    if !(MIN_BREAK_GLASS_ACCESS_TTL_SECS..=MAX_BREAK_GLASS_ACCESS_TTL_SECS).contains(&max_access) {
        return invalid("Emergency access max TTL must be between 60 and 900 seconds");
    }
    if let Err(message) = validate_time_window(
        policy.created_at,
        policy.expires_at,
        MAX_BREAK_GLASS_POLICY_TTL_SECS,
        "emergency access policy",
    ) {
        return invalid(&message);
    }
    if !within_clock_skew(policy.created_at, action.timestamp) {
        return invalid("Emergency access policy created_at is not bound to action time");
    }
    let creator_membership = validate_active_guardian_membership(
        &policy.authorization_membership_hash,
        &policy.hearth_hash,
        &action.author,
        "Emergency access policy creator",
    )?;
    if creator_membership != ValidateCallbackResult::Valid {
        return Ok(creator_membership);
    }
    for approver in &policy.approvers {
        let membership = validate_active_guardian_membership(
            &approver.membership_hash,
            &policy.hearth_hash,
            &approver.agent,
            "Emergency access approver",
        )?;
        if membership != ValidateCallbackResult::Valid {
            return Ok(membership);
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_emergency_access_request_create(
    request: &EmergencyAccessRequest,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let authorship = validate_claimed_agent(
        &action.author,
        &request.requester,
        "EmergencyAccessRequest.requester",
    );
    if authorship != ValidateCallbackResult::Valid {
        return Ok(authorship);
    }
    for (label, value) in [
        ("request_id", request.request_id),
        ("reason_commitment", request.reason_commitment),
        (
            "release_context_commitment",
            request.release_context_commitment,
        ),
        ("consumption_nullifier", request.consumption_nullifier),
    ] {
        if value == [0u8; 32] {
            return invalid(&format!("Emergency access request {label} cannot be zero"));
        }
    }
    if request.requested_envelope_hashes.is_empty()
        || request.requested_envelope_hashes.len() > MAX_BREAK_GLASS_SCOPE
    {
        return invalid("Emergency access request must contain 1-16 encrypted envelopes");
    }
    let unique: HashSet<_> = request.requested_envelope_hashes.iter().collect();
    if unique.len() != request.requested_envelope_hashes.len() {
        return invalid("Emergency access request cannot duplicate encrypted envelopes");
    }
    if !within_clock_skew(request.created_at, action.timestamp) {
        return invalid("Emergency access request created_at is not bound to action time");
    }
    if let Err(message) = validate_time_window(
        request.created_at,
        request.expires_at,
        MAX_BREAK_GLASS_REQUEST_TTL_SECS,
        "emergency access request",
    ) {
        return invalid(&message);
    }

    let requester_membership = validate_active_membership(
        &request.authorization_membership_hash,
        &request.hearth_hash,
        &action.author,
        "Emergency access requester",
    )?;
    if requester_membership != ValidateCallbackResult::Valid {
        return Ok(requester_membership);
    }
    let accessor_membership = validate_active_membership(
        &request.accessor_membership_hash,
        &request.hearth_hash,
        &request.accessor,
        "Emergency access accessor",
    )?;
    if accessor_membership != ValidateCallbackResult::Valid {
        return Ok(accessor_membership);
    }

    let policy: EmergencyAccessPolicy = load_entry(&request.policy_hash, "EmergencyAccessPolicy")?;
    if policy.hearth_hash != request.hearth_hash {
        return invalid("Emergency access request policy belongs to another hearth");
    }
    if request.created_at >= policy.expires_at || request.expires_at > policy.expires_at {
        return invalid("Emergency access request must remain within the policy lifetime");
    }
    let requested_ttl = request.requested_access_ttl_secs as i64;
    if requested_ttl < MIN_BREAK_GLASS_ACCESS_TTL_SECS
        || requested_ttl > policy.max_access_ttl_secs as i64
        || requested_ttl > MAX_BREAK_GLASS_ACCESS_TTL_SECS
    {
        return invalid("Requested emergency access TTL exceeds policy limits");
    }

    let incident: EmergencyAlert = load_entry(&request.incident_alert_hash, "EmergencyAlert")?;
    if incident.hearth_hash != request.hearth_hash {
        return invalid("Emergency access incident belongs to another hearth");
    }
    if incident.resolved_at.is_some() {
        return invalid("Emergency access cannot be requested for an already resolved alert");
    }

    let key: HearthEncryptionKey =
        load_entry(&request.accessor_encryption_key_hash, "HearthEncryptionKey")?;
    if key.hearth_hash != request.hearth_hash || key.owner != request.accessor {
        return invalid("Emergency accessor does not own the referenced key in this hearth");
    }
    if key.expires_at <= request.created_at {
        return invalid("Emergency accessor encryption key was expired at request time");
    }

    for envelope_hash in &request.requested_envelope_hashes {
        let envelope: HearthEncryptedEnvelope =
            load_entry(envelope_hash, "HearthEncryptedEnvelope")?;
        if envelope.hearth_hash != request.hearth_hash {
            return invalid("Emergency access scope contains a cross-hearth envelope");
        }
        if !policy
            .allowed_content_kinds
            .contains(&envelope.content_kind)
        {
            return invalid("Emergency access scope contains a content kind forbidden by policy");
        }
        if envelope
            .expires_at
            .is_some_and(|expiry| expiry <= request.created_at)
        {
            return invalid("Emergency access scope contains an expired envelope");
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_emergency_access_approval_create(
    approval: &EmergencyAccessApproval,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let authorship = validate_claimed_agent(
        &action.author,
        &approval.approver,
        "EmergencyAccessApproval.approver",
    );
    if authorship != ValidateCallbackResult::Valid {
        return Ok(authorship);
    }
    if approval.decision_commitment == [0u8; 32] {
        return invalid("Emergency access approval decision commitment cannot be zero");
    }
    if !within_clock_skew(approval.created_at, action.timestamp) {
        return invalid("Emergency access approval created_at is not bound to action time");
    }
    let request: EmergencyAccessRequest =
        load_entry(&approval.request_hash, "EmergencyAccessRequest")?;
    let policy: EmergencyAccessPolicy = load_entry(&approval.policy_hash, "EmergencyAccessPolicy")?;
    if approval.hearth_hash != request.hearth_hash
        || approval.hearth_hash != policy.hearth_hash
        || request.policy_hash != approval.policy_hash
    {
        return invalid("Emergency access approval request, policy, and hearth do not match");
    }
    if approval.created_at >= request.expires_at || approval.created_at >= policy.expires_at {
        return invalid("Emergency access approval was created after request or policy expiry");
    }
    let configured = policy
        .approvers
        .iter()
        .find(|item| item.agent == action.author)
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Emergency access approval author is not configured by policy".into()
        )))?;
    if configured.membership_hash != approval.authorization_membership_hash {
        return invalid("Emergency access approval uses a membership not pinned by policy");
    }
    validate_active_guardian_membership(
        &approval.authorization_membership_hash,
        &approval.hearth_hash,
        &action.author,
        "Emergency access approver",
    )
}

pub fn validate_emergency_access_grant_create(
    grant: &EmergencyAccessGrant,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let authorship = validate_claimed_agent(
        &action.author,
        &grant.issued_by,
        "EmergencyAccessGrant.issued_by",
    );
    if authorship != ValidateCallbackResult::Valid {
        return Ok(authorship);
    }
    if grant.grant_id == [0u8; 32] || grant.consumption_nullifier == [0u8; 32] {
        return invalid("Emergency access grant identifiers cannot be zero");
    }
    if !within_clock_skew(grant.issued_at, action.timestamp) {
        return invalid("Emergency access grant issued_at is not bound to action time");
    }
    let issuer = validate_active_guardian_membership(
        &grant.issuer_membership_hash,
        &grant.hearth_hash,
        &action.author,
        "Emergency access grant issuer",
    )?;
    if issuer != ValidateCallbackResult::Valid {
        return Ok(issuer);
    }

    let request: EmergencyAccessRequest =
        load_entry(&grant.request_hash, "EmergencyAccessRequest")?;
    let policy: EmergencyAccessPolicy = load_entry(&grant.policy_hash, "EmergencyAccessPolicy")?;
    if grant.hearth_hash != request.hearth_hash
        || grant.hearth_hash != policy.hearth_hash
        || request.policy_hash != grant.policy_hash
        || grant.incident_alert_hash != request.incident_alert_hash
        || grant.accessor != request.accessor
        || grant.accessor_encryption_key_hash != request.accessor_encryption_key_hash
        || grant.requested_envelope_hashes != request.requested_envelope_hashes
        || grant.consumption_nullifier != request.consumption_nullifier
    {
        return invalid("Emergency access grant does not exactly reproduce its request scope");
    }
    if grant.issued_at >= request.expires_at || grant.issued_at >= policy.expires_at {
        return invalid("Emergency access grant was issued after request or policy expiry");
    }
    let ttl_micros = grant.expires_at.as_micros() - grant.issued_at.as_micros();
    if ttl_micros <= 0
        || ttl_micros > request.requested_access_ttl_secs as i64 * 1_000_000
        || ttl_micros > policy.max_access_ttl_secs as i64 * 1_000_000
        || ttl_micros > MAX_BREAK_GLASS_ACCESS_TTL_SECS * 1_000_000
        || grant.expires_at > request.expires_at
        || grant.expires_at > policy.expires_at
    {
        return invalid("Emergency access grant lifetime exceeds request or policy bounds");
    }

    if grant.release_cipher != HearthEnvelopeCipher::XChaCha20Poly1305V1
        || grant.release_key_wrap != HearthKeyWrapAlgorithm::X25519HkdfSha256XChaCha20Poly1305V1
        || grant.release_aad_digest != request.release_context_commitment
    {
        return invalid(
            "Emergency key release algorithms or AAD are not exactly bound to the request",
        );
    }
    for (label, value) in [
        (
            "release_plaintext_commitment",
            grant.release_plaintext_commitment,
        ),
        (
            "release_ephemeral_public_key",
            grant.release_ephemeral_public_key,
        ),
    ] {
        if value == [0u8; 32] {
            return invalid(&format!("Emergency access grant {label} cannot be zero"));
        }
    }
    if grant.release_nonce == [0u8; 24] {
        return invalid("Emergency access grant release nonce cannot be zero");
    }
    if grant.release_ciphertext.len() < 16
        || grant.release_ciphertext.len() > MAX_BREAK_GLASS_RELEASE_CIPHERTEXT_BYTES
    {
        return invalid("Emergency key release ciphertext must be 16-8192 bytes");
    }
    if grant.release_wrapped_key.len() < 48
        || grant.release_wrapped_key.len() > MAX_WRAPPED_KEY_BYTES
    {
        return invalid("Emergency key release wrapped key must be 48-160 bytes");
    }
    let accessor_key: HearthEncryptionKey =
        load_entry(&grant.accessor_encryption_key_hash, "HearthEncryptionKey")?;
    if accessor_key.hearth_hash != grant.hearth_hash
        || accessor_key.owner != grant.accessor
        || accessor_key.expires_at <= grant.issued_at
    {
        return invalid("Emergency key release is not addressed to a valid exact accessor key");
    }

    if grant.approval_hashes.len() < policy.threshold as usize
        || grant.approval_hashes.len() > policy.approvers.len()
    {
        return invalid("Emergency access grant does not contain a valid approval threshold");
    }
    let unique_hashes: HashSet<_> = grant.approval_hashes.iter().collect();
    if unique_hashes.len() != grant.approval_hashes.len() {
        return invalid("Emergency access grant cannot duplicate approval hashes");
    }
    let mut approvers = HashSet::new();
    for approval_hash in &grant.approval_hashes {
        let approval: EmergencyAccessApproval =
            load_entry(approval_hash, "EmergencyAccessApproval")?;
        if approval.hearth_hash != grant.hearth_hash
            || approval.request_hash != grant.request_hash
            || approval.policy_hash != grant.policy_hash
            || approval.decision != EmergencyApprovalDecision::Approve
            || !approvers.insert(approval.approver.clone())
        {
            return invalid("Emergency access grant contains invalid or duplicate approvals");
        }
        if !policy
            .approvers
            .iter()
            .any(|item| item.agent == approval.approver)
        {
            return invalid("Emergency access grant contains an approver outside policy");
        }
    }
    if approvers.len() < policy.threshold as usize {
        return invalid("Emergency access grant approval threshold was not met");
    }
    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_emergency_access_receipt_create(
    receipt: &EmergencyAccessReceipt,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let authorship = validate_claimed_agent(
        &action.author,
        &receipt.accessor,
        "EmergencyAccessReceipt.accessor",
    );
    if authorship != ValidateCallbackResult::Valid {
        return Ok(authorship);
    }
    if receipt.receipt_id == [0u8; 32]
        || receipt.local_audit_commitment == [0u8; 32]
        || receipt.consumption_nullifier == [0u8; 32]
    {
        return invalid("Emergency access receipt commitments cannot be zero");
    }
    if !within_clock_skew(receipt.consumed_at, action.timestamp) {
        return invalid("Emergency access receipt consumed_at is not bound to action time");
    }
    let grant: EmergencyAccessGrant = load_entry(&receipt.grant_hash, "EmergencyAccessGrant")?;
    if receipt.hearth_hash != grant.hearth_hash
        || receipt.request_hash != grant.request_hash
        || receipt.accessor != grant.accessor
        || receipt.consumption_nullifier != grant.consumption_nullifier
    {
        return invalid("Emergency access receipt does not exactly match its grant");
    }
    if receipt.consumed_at < grant.issued_at || receipt.consumed_at >= grant.expires_at {
        return invalid("Emergency access receipt was created outside the grant lifetime");
    }
    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_emergency_access_review_create(
    review: &EmergencyAccessReview,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    let authorship = validate_claimed_agent(
        &action.author,
        &review.reviewer,
        "EmergencyAccessReview.reviewer",
    );
    if authorship != ValidateCallbackResult::Valid {
        return Ok(authorship);
    }
    if review.notes_commitment == [0u8; 32] {
        return invalid("Emergency access review notes commitment cannot be zero");
    }
    if !within_clock_skew(review.reviewed_at, action.timestamp) {
        return invalid("Emergency access review time is not bound to action time");
    }
    let grant: EmergencyAccessGrant = load_entry(&review.grant_hash, "EmergencyAccessGrant")?;
    if grant.hearth_hash != review.hearth_hash || review.reviewed_at < grant.issued_at {
        return invalid("Emergency access review does not match the referenced grant");
    }
    validate_active_guardian_membership(
        &review.reviewer_membership_hash,
        &review.hearth_hash,
        &action.author,
        "Emergency access reviewer",
    )
}

fn load_entry<T: serde::de::DeserializeOwned>(hash: &ActionHash, label: &str) -> ExternResult<T> {
    let record = must_get_valid_record(hash.clone())?;
    record
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode {label}: {error}"
            )))
        })?
        .ok_or(wasm_error!(WasmErrorInner::Guest(format!(
            "Referenced entry is not a {label}"
        ))))
}

fn validate_active_membership(
    membership_hash: &ActionHash,
    hearth_hash: &ActionHash,
    expected_agent: &AgentPubKey,
    label: &str,
) -> ExternResult<ValidateCallbackResult> {
    let membership: HearthMembership = load_entry(membership_hash, "HearthMembership")?;
    if &membership.hearth_hash != hearth_hash || &membership.agent != expected_agent {
        return invalid(&format!(
            "{label} membership belongs to another hearth or agent"
        ));
    }
    if membership.status != MembershipStatus::Active {
        return invalid(&format!("{label} requires an active hearth membership"));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_active_guardian_membership(
    membership_hash: &ActionHash,
    hearth_hash: &ActionHash,
    expected_agent: &AgentPubKey,
    label: &str,
) -> ExternResult<ValidateCallbackResult> {
    let membership: HearthMembership = load_entry(membership_hash, "HearthMembership")?;
    if &membership.hearth_hash != hearth_hash || &membership.agent != expected_agent {
        return invalid(&format!(
            "{label} membership belongs to another hearth or agent"
        ));
    }
    if membership.status != MembershipStatus::Active || !membership.role.is_guardian() {
        return invalid(&format!("{label} requires an active guardian membership"));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_time_window(
    start: Timestamp,
    end: Timestamp,
    max_ttl_secs: i64,
    label: &str,
) -> Result<(), String> {
    let ttl_micros = end.as_micros() - start.as_micros();
    if ttl_micros <= 0 {
        return Err(format!("{label} expires_at must be after its start time"));
    }
    if ttl_micros > max_ttl_secs * 1_000_000 {
        return Err(format!("{label} exceeds its maximum lifetime"));
    }
    Ok(())
}

fn within_clock_skew(entry_time: Timestamp, action_time: Timestamp) -> bool {
    (entry_time.as_micros() - action_time.as_micros()).abs()
        <= MAX_BREAK_GLASS_CLOCK_SKEW_SECS * 1_000_000
}

fn is_safe_identifier(value: &str, max_bytes: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_bytes
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b':'))
}

fn invalid(message: &str) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(message.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn access_window_is_strictly_bounded() {
        assert_eq!(MAX_BREAK_GLASS_ACCESS_TTL_SECS, 900);
        assert_eq!(MAX_BREAK_GLASS_REQUEST_TTL_SECS, 1800);
    }

    #[test]
    fn policy_identifiers_reject_path_like_values() {
        assert!(is_safe_identifier("medical-v1:primary", 96));
        assert!(!is_safe_identifier("../medical", 96));
        assert!(!is_safe_identifier("", 96));
    }
}

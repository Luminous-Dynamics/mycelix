// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Execution Integrity Zome
//! Defines entry types and validation for proposal execution
//!
//! Updated to use HDI 0.7 patterns

use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Timelock for approved proposals
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Timelock {
    /// Timelock identifier
    pub id: String,
    /// Proposal ID
    pub proposal_id: String,
    /// Actions to execute (JSON)
    pub actions: String,
    /// When timelock started
    pub started: Timestamp,
    /// When timelock expires
    pub expires: Timestamp,
    /// Timelock status
    pub status: TimelockStatus,
    /// Cancellation reason if cancelled
    pub cancellation_reason: Option<String>,
}

/// Status of a timelock
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TimelockStatus {
    /// Waiting for timelock to expire
    Pending,
    /// Ready to execute
    Ready,
    /// Successfully executed
    Executed,
    /// Cancelled before execution
    Cancelled,
    /// Execution failed
    Failed,
    /// Vetoed by a guardian — pending possible override (48-hour window)
    Vetoed,
}

/// Status of a guardian veto (can be overridden by supermajority)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum VetoStatus {
    /// Veto in effect, override window open
    Active,
    /// Override challenge initiated
    Challenged,
    /// Supermajority (80%) override succeeded — timelock restored to Ready
    Overridden,
    /// Override attempt failed or window expired — timelock cancelled
    Sustained,
}

/// Override window duration: 48 hours in microseconds.
/// Hardcoded — not configurable by governance to prevent the threshold
/// itself from being weakened by governance capture.
pub const VETO_OVERRIDE_WINDOW_US: i64 = 48 * 3600 * 1_000_000;

/// Supermajority threshold required to override a guardian veto.
/// Aligned with Constitution Art. III, Sec. 5.3: "two-thirds (2/3)
/// majority in both houses." Hardcoded — not configurable by governance.
pub const VETO_OVERRIDE_THRESHOLD: f64 = 0.67;

/// Participation insurance: if override quorum fails, the threshold
/// decreases by this amount per failed attempt.
/// 67% -> 62% -> 60% (floor). Prevents the 34% boycott attack
/// where a minority sustains a veto by suppressing participation.
pub const OVERRIDE_THRESHOLD_DECAY_PER_ATTEMPT: f64 = 0.05;

/// Absolute floor for the adaptive override threshold.
/// Never goes below 60% — preserving supermajority legitimacy.
pub const OVERRIDE_THRESHOLD_FLOOR: f64 = 0.60;

/// Maximum vetoes per Guardian per 12-month period before probation.
/// Aligned with Constitution Art. III, Sec. 5.4.
pub const VETO_YEARLY_LIMIT: u32 = 3;

/// Rolling year window for veto limit enforcement (microseconds).
/// 12 months ≈ 365.25 days.
pub const ROLLING_YEAR_US: i64 = 365 * 24 * 3600 * 1_000_000 + 6 * 3600 * 1_000_000;

/// Strategic Override sunset: 36 months from Genesis Epoch (microseconds).
/// After this period, veto authority transitions to Charter Guardian Authority,
/// which requires constitutional justification (Art. III, Sec. 3).
pub const STRATEGIC_OVERRIDE_SUNSET_US: i64 = 36 * 30 * 24 * 3600 * 1_000_000_i64;

/// Threat categories that constitute valid constitutional justification
/// for Charter Guardian Authority vetoes (post-sunset period).
/// Non-charter vetoes are rejected after the sunset.
pub const CHARTER_THREAT_CATEGORIES: &[&str] = &[
    "constitutional_violation",
    "core_principle_violation",
    "member_rights_violation",
];

/// Check whether the Strategic Override period has expired.
pub fn is_post_sunset(genesis_epoch_us: i64, now_us: i64) -> bool {
    (now_us - genesis_epoch_us) > STRATEGIC_OVERRIDE_SUNSET_US
}

/// Compute the adaptive override threshold based on failed attempts.
///
/// Thermodynamic metaphor: each failed attempt lowers the energy barrier,
/// making it progressively easier for the community to overcome the veto.
/// The floor at 67% ensures the barrier never disappears entirely.
pub fn adaptive_override_threshold(failed_attempts: u32) -> f64 {
    let decay = failed_attempts as f64 * OVERRIDE_THRESHOLD_DECAY_PER_ATTEMPT;
    (VETO_OVERRIDE_THRESHOLD - decay).max(OVERRIDE_THRESHOLD_FLOOR)
}

/// Durable execution attempt state.
///
/// This record is committed before the protected action is invoked. The state
/// machine deliberately separates durable admission from provider entry so a
/// restart can distinguish a committed pre-dispatch reservation from an
/// invocation whose outcome must be reconciled.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ExecutionAttemptStatus {
    DispatchPending,
    Invoked,
    InvocationClaimed,
    Succeeded,
    Failed,
    Indeterminate,
    NotEntered,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ExecutionAttempt {
    /// Stable attempt identifier.
    pub id: String,
    /// Operational identity of this execution operation.
    pub operation_id: String,
    /// Timelock that authorized the exact material action.
    pub timelock_id: String,
    /// Proposal for contextual provenance.
    pub proposal_id: String,
    /// Exact raw material action digest committed before invocation.
    pub action_digest: String,
    /// Same-action fence digest. This is the shared collision namespace.
    pub action_key_digest: String,
    /// Boundary-scoped attempt identity.
    pub attempt_identity: String,
    /// Native replay identity derived from the accepted native authorization namespace + ID.
    pub native_replay_identity: String,
    /// Historical provider execution environment.
    pub provider_environment: String,
    /// Historical provider audience.
    pub provider_audience: String,
    /// Exact adapter identity used for effect entry.
    pub adapter_identity: String,
    /// Execution authority DID.
    pub executor: String,
    /// Current durable lifecycle state.
    pub status: ExecutionAttemptStatus,
    /// Source-chain timestamp for reservation.
    pub prepared_at: Timestamp,
    /// Source-chain timestamp for the latest state transition.
    pub updated_at: Timestamp,
    /// Explicit evidence commitment for a terminal provider outcome.
    pub outcome_evidence_commitment: Option<String>,
    /// Explicit proof that the attempt never entered the provider.
    pub not_entered_marker: Option<String>,
}

/// Execution record for a proposal
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Execution {
    /// Execution identifier
    pub id: String,
    /// Timelock ID
    pub timelock_id: String,
    /// Proposal ID
    pub proposal_id: String,
    /// Executor's DID
    pub executor: String,
    /// Execution status
    pub status: ExecutionStatus,
    /// Result data (JSON)
    pub result: Option<String>,
    /// Error message if failed
    pub error: Option<String>,
    /// Execution timestamp
    pub executed_at: Timestamp,
}

/// Status of an execution
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ExecutionStatus {
    Success,
    PartialSuccess,
    Failed,
    /// Provider/effect entry occurred or may have occurred, but no authoritative
    /// no-effect proof exists yet. This is deliberately distinct from Failed.
    Indeterminate,
}

/// Guardian veto (for emergency cancellation)
///
/// Constitutional registry fields (Art. III, Sec. 5.5):
/// All veto exercises are recorded in an immutable public ledger with
/// justification hash, threat category, and affected proposal reference.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct GuardianVeto {
    /// Veto identifier
    pub id: String,
    /// Timelock ID being vetoed
    pub timelock_id: String,
    /// Guardian's DID
    pub guardian: String,
    /// Reason for veto
    pub reason: String,
    /// Veto timestamp
    pub vetoed_at: Timestamp,
    // ── Constitutional registry fields (Art. III, Sec. 5.5) ──────────
    /// Proposal ID affected by this veto (if known from timelock context).
    /// Option for backward compatibility with pre-existing DHT entries.
    #[serde(default)]
    pub affected_proposal_id: Option<String>,
    /// SHA-256 hash of the full justification document.
    /// Enables verifiable justification without storing full text on-chain.
    #[serde(default)]
    pub justification_hash: Option<String>,
    /// Threat category classifying the reason for veto.
    /// e.g., "safety", "constitutional", "fiscal", "governance", "security"
    /// Required for Charter Guardian Authority vetoes (post-sunset).
    #[serde(default)]
    pub threat_category: Option<String>,
    /// Haptic proof from robotic sensors justifying the veto (Moral Manifold).
    #[serde(default)]
    pub haptic_proof: Option<HapticVerification>,
}

/// Hardware-signed haptic proof for a constitutional veto.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct HapticVerification {
    pub joint_id: String,
    pub surprise_magnitude: f64,
    pub hardware_signature: Vec<u8>,
    pub enclave_pubkey: [u8; 32],
}

/// A vote to override a guardian veto
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct VetoOverrideVote {
    /// Vote identifier
    pub id: String,
    /// Veto being challenged
    pub veto_id: String,
    /// Voter's DID
    pub voter_did: String,
    /// Whether this vote supports overriding the veto
    pub supports_override: bool,
    /// Voter's phi score at time of vote
    pub phi_score: f64,
    /// Vote timestamp
    pub voted_at: Timestamp,
}

/// Result of a veto override attempt
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct VetoOverrideResult {
    /// Result identifier
    pub id: String,
    /// Veto that was challenged
    pub veto_id: String,
    /// Timelock that was vetoed
    pub timelock_id: String,
    /// Weighted votes supporting override
    pub override_votes_for: f64,
    /// Weighted votes sustaining the veto
    pub override_votes_against: f64,
    /// Total eligible voters at resolution time
    pub total_eligible_voters: u64,
    /// Override threshold (always 0.80)
    pub override_threshold: f64,
    /// Whether the override succeeded
    pub override_succeeded: bool,
    /// Resolution timestamp
    pub resolved_at: Timestamp,
}

/// Status of a fund allocation
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AllocationStatus {
    /// Funds locked in escrow awaiting execution
    Locked,
    /// Funds released after successful execution
    Released,
    /// Funds returned after veto or expiration
    Refunded,
}

/// Fund allocation — tracks locked funds for a proposal's execution
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FundAllocation {
    /// Allocation identifier
    pub id: String,
    /// Associated proposal ID
    pub proposal_id: String,
    /// Associated timelock ID
    pub timelock_id: String,
    /// Account the funds are locked from
    pub source_account: String,
    /// Amount locked
    pub amount: f64,
    /// Currency denomination (e.g., "credits")
    pub currency: String,
    /// When funds were locked
    pub locked_at: Timestamp,
    /// Current allocation status
    pub status: AllocationStatus,
    /// Reason for status change (refund reason, release confirmation, etc.)
    pub status_reason: Option<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Timelock(Timelock),
    Execution(Execution),
    ExecutionAttempt(ExecutionAttempt),
    GuardianVeto(GuardianVeto),
    FundAllocation(FundAllocation),
    VetoOverrideVote(VetoOverrideVote),
    VetoOverrideResult(VetoOverrideResult),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Proposal to timelock
    ProposalToTimelock,
    /// Timelock to immutable execution outcome
    TimelockToExecution,
    /// Timelock to its durable execution-attempt record
    TimelockToExecutionAttempt,
    /// Pending timelocks
    PendingTimelocks,
    /// Guardian to vetoes
    GuardianToVeto,
    /// Proposal to fund allocation
    ProposalToFundAllocation,
    /// O(1) lookup: timelock ID anchor → timelock record
    TimelockById,
    /// Veto to override votes
    VetoToOverrideVotes,
    /// Veto to override result
    VetoToOverrideResult,
}

// ---------------------------------------------------------------------------
// Pure check functions (no HDI host calls — unit-testable)
// ---------------------------------------------------------------------------

/// Check that a new timelock is valid: expires > started, valid JSON actions,
/// and initial status is Pending.
pub fn check_create_timelock(timelock: &Timelock) -> Result<(), String> {
    if timelock.expires <= timelock.started {
        return Err("Timelock expiry must be after start".into());
    }
    if serde_json::from_str::<serde_json::Value>(&timelock.actions).is_err() {
        return Err("Actions must be valid JSON".into());
    }
    if timelock.status != TimelockStatus::Pending {
        return Err("Initial timelock status must be Pending".into());
    }
    Ok(())
}

/// Check that a timelock update is valid: immutable proposal_id, and only
/// whitelisted status transitions are allowed.
pub fn check_update_timelock(original: &Timelock, updated: &Timelock) -> Result<(), String> {
    if updated.id != original.id {
        return Err("Cannot change timelock ID".into());
    }
    if updated.proposal_id != original.proposal_id {
        return Err("Cannot change timelock proposal ID".into());
    }
    if updated.actions != original.actions {
        return Err("Cannot change timelock material actions after creation".into());
    }
    if updated.started != original.started {
        return Err("Cannot change timelock start time after creation".into());
    }
    if updated.expires != original.expires {
        return Err("Cannot change timelock expiry after creation".into());
    }
    match (&original.status, &updated.status) {
        (TimelockStatus::Pending, TimelockStatus::Ready)
        | (TimelockStatus::Pending, TimelockStatus::Cancelled)
        | (TimelockStatus::Ready, TimelockStatus::Executed)
        | (TimelockStatus::Ready, TimelockStatus::Failed)
        | (TimelockStatus::Ready, TimelockStatus::Cancelled)
        // Veto override transitions:
        | (TimelockStatus::Ready, TimelockStatus::Vetoed)      // Guardian veto
        | (TimelockStatus::Pending, TimelockStatus::Vetoed)    // Guardian veto on pending
        | (TimelockStatus::Vetoed, TimelockStatus::Ready)      // Override succeeded
        | (TimelockStatus::Vetoed, TimelockStatus::Cancelled)  // Override failed/window expired
        => Ok(()),
        _ => Err("Invalid timelock status transition".into()),
    }
}

/// Validate creation of a durable pre-dispatch execution attempt.
pub fn check_create_execution_attempt(attempt: &ExecutionAttempt) -> Result<(), String> {
    if attempt.id.trim().is_empty() {
        return Err("Execution attempt ID is required".into());
    }
    if attempt.operation_id.trim().is_empty() {
        return Err("Execution attempt operation_id is required".into());
    }
    if attempt.timelock_id.trim().is_empty() {
        return Err("Execution attempt timelock_id is required".into());
    }
    if attempt.proposal_id.trim().is_empty() {
        return Err("Execution attempt proposal_id is required".into());
    }
    if attempt.action_digest.trim().is_empty() {
        return Err("Execution attempt action_digest is required".into());
    }
    if !attempt.action_key_digest.starts_with("constitutional-action-key-v1:") {
        return Err("Execution attempt action_key_digest must use the canonical ActionKey namespace".into());
    }
    if !attempt.attempt_identity.starts_with("constitutional-attempt-identity-v1:") {
        return Err("Execution attempt attempt_identity must use the canonical AttemptIdentity namespace".into());
    }
    if attempt.native_replay_identity.trim().is_empty() {
        return Err("Execution attempt native_replay_identity is required".into());
    }
    if attempt.provider_environment.trim().is_empty() {
        return Err("Execution attempt provider_environment is required".into());
    }
    if attempt.provider_audience.trim().is_empty() {
        return Err("Execution attempt provider_audience is required".into());
    }
    if attempt.adapter_identity.trim().is_empty() {
        return Err("Execution attempt adapter_identity is required".into());
    }
    if !attempt.executor.starts_with("did:") {
        return Err("Execution attempt executor must be a valid DID".into());
    }
    if attempt.status != ExecutionAttemptStatus::DispatchPending {
        return Err("Execution attempt must be created in DispatchPending state".into());
    }
    if attempt.outcome_evidence_commitment.is_some() || attempt.not_entered_marker.is_some() {
        return Err("Initial ExecutionAttempt cannot contain terminal evidence or a not-entered marker".into());
    }
    Ok(())
}

pub fn check_update_execution_attempt(
    original: &ExecutionAttempt,
    updated: &ExecutionAttempt,
) -> Result<(), String> {
    if updated.id != original.id
        || updated.operation_id != original.operation_id
        || updated.timelock_id != original.timelock_id
        || updated.proposal_id != original.proposal_id
        || updated.action_digest != original.action_digest
        || updated.action_key_digest != original.action_key_digest
        || updated.attempt_identity != original.attempt_identity
        || updated.native_replay_identity != original.native_replay_identity
        || updated.provider_environment != original.provider_environment
        || updated.provider_audience != original.provider_audience
        || updated.adapter_identity != original.adapter_identity
        || updated.executor != original.executor
        || updated.prepared_at != original.prepared_at
    {
        return Err("ExecutionAttempt identity/material fields are immutable".into());
    }
    if updated.updated_at < original.updated_at {
        return Err("ExecutionAttempt updated_at cannot move backwards".into());
    }

    match (&original.status, &updated.status) {
        (ExecutionAttemptStatus::DispatchPending, ExecutionAttemptStatus::Invoked) => {
            if updated.outcome_evidence_commitment.is_some()
                || updated.not_entered_marker.is_some()
            {
                return Err("Invoked execution attempt cannot carry terminal evidence".into());
            }
        }
        (ExecutionAttemptStatus::DispatchPending, ExecutionAttemptStatus::NotEntered) => {
            if updated.not_entered_marker.as_deref().map(str::trim).unwrap_or("").is_empty()
            {
                return Err("NotEntered requires an explicit marker".into());
            }
            if updated.outcome_evidence_commitment.is_some() {
                return Err("NotEntered cannot carry provider outcome evidence".into());
            }
        }
        (ExecutionAttemptStatus::InvocationClaimed, ExecutionAttemptStatus::Succeeded)
        | (ExecutionAttemptStatus::Indeterminate, ExecutionAttemptStatus::Succeeded)
        | (ExecutionAttemptStatus::InvocationClaimed, ExecutionAttemptStatus::Failed)
        | (ExecutionAttemptStatus::Indeterminate, ExecutionAttemptStatus::Failed) => {
            if updated
                .outcome_evidence_commitment
                .as_deref()
                .map(str::trim)
                .unwrap_or("")
                .is_empty()
            {
                return Err("terminal execution outcome requires evidence commitment".into());
            }
            if updated.not_entered_marker.is_some() {
                return Err("terminal provider outcome cannot carry a not-entered marker".into());
            }
        }
        (ExecutionAttemptStatus::Invoked, ExecutionAttemptStatus::InvocationClaimed) => {
            if updated.outcome_evidence_commitment.is_some()
                || updated.not_entered_marker.is_some()
            {
                return Err("InvocationClaimed attempt cannot carry terminal evidence".into());
            }
        }
        (ExecutionAttemptStatus::InvocationClaimed, ExecutionAttemptStatus::Indeterminate) => {
            if updated.outcome_evidence_commitment.is_some()
                || updated.not_entered_marker.is_some()
            {
                return Err("Indeterminate claim cannot carry terminal evidence".into());
            }
        }
        (ExecutionAttemptStatus::Indeterminate, ExecutionAttemptStatus::NotEntered) => {
            return Err("Indeterminate attempt cannot be downgraded to NotEntered".into());
        }
        _ if original.status == updated.status => {
            return Err("ExecutionAttempt state must advance through an explicit transition".into());
        }
        _ => return Err("Invalid ExecutionAttempt status transition".into()),
    }

    Ok(())
}

/// Check that a new execution record is valid: executor starts with "did:",
/// and result (if present) is valid JSON.
pub fn check_create_execution(execution: &Execution) -> Result<(), String> {
    if !execution.executor.starts_with("did:") {
        return Err("Executor must be a valid DID".into());
    }
    if let Some(ref result) = execution.result {
        if serde_json::from_str::<serde_json::Value>(result).is_err() {
            return Err("Result must be valid JSON".into());
        }
    }
    Ok(())
}

/// Check that a new guardian veto is valid: guardian starts with "did:",
/// reason is not empty, and constitutional registry fields are well-formed.
pub fn check_create_veto(veto: &GuardianVeto) -> Result<(), String> {
    if !veto.guardian.starts_with("did:") {
        return Err("Guardian must be a valid DID".into());
    }
    if veto.reason.is_empty() {
        return Err("Veto reason is required".into());
    }
    // Validate justification_hash format when present (SHA-256 = 64 hex chars)
    if let Some(ref hash) = veto.justification_hash {
        if hash.len() != 64 || !hash.chars().all(|c| c.is_ascii_hexdigit()) {
            return Err("justification_hash must be a 64-character hex string (SHA-256)".into());
        }
    }
    // Validate threat_category is non-empty when present
    if let Some(ref cat) = veto.threat_category {
        if cat.is_empty() {
            return Err("threat_category must not be empty when provided".into());
        }
    }
    Ok(())
}

/// Check that a veto override vote is valid.
pub fn check_create_override_vote(vote: &VetoOverrideVote) -> Result<(), String> {
    if !vote.voter_did.starts_with("did:") {
        return Err("Override voter must be a valid DID".into());
    }
    if vote.veto_id.is_empty() {
        return Err("Veto ID is required for override vote".into());
    }
    if vote.phi_score < 0.0 || vote.phi_score > 1.0 {
        return Err("Phi score must be between 0 and 1".into());
    }
    Ok(())
}

/// Check that a veto override result is valid.
pub fn check_create_override_result(result: &VetoOverrideResult) -> Result<(), String> {
    if result.veto_id.is_empty() {
        return Err("Veto ID is required for override result".into());
    }
    if result.timelock_id.is_empty() {
        return Err("Timelock ID is required for override result".into());
    }
    // Threshold must be exactly 0.80 — hardcoded, not configurable
    if (result.override_threshold - VETO_OVERRIDE_THRESHOLD).abs() > f64::EPSILON {
        return Err(format!(
            "Override threshold must be exactly {}, got {}",
            VETO_OVERRIDE_THRESHOLD, result.override_threshold
        ));
    }
    if result.override_votes_for < 0.0 || result.override_votes_against < 0.0 {
        return Err("Override vote counts must be non-negative".into());
    }
    Ok(())
}

/// Check that a new fund allocation is valid: amount > 0 and finite,
/// source_account not empty, currency not empty, initial status is Locked.
pub fn check_create_fund_allocation(alloc: &FundAllocation) -> Result<(), String> {
    if alloc.amount <= 0.0 || !alloc.amount.is_finite() {
        return Err("Fund allocation amount must be positive and finite".into());
    }
    if alloc.source_account.is_empty() {
        return Err("Source account is required".into());
    }
    if alloc.currency.is_empty() {
        return Err("Currency is required".into());
    }
    if alloc.status != AllocationStatus::Locked {
        return Err("Initial allocation status must be Locked".into());
    }
    Ok(())
}

/// Check that a fund allocation update is valid: immutable proposal_id,
/// immutable amount, and only Locked→Released or Locked→Refunded transitions.
pub fn check_update_fund_allocation(
    original: &FundAllocation,
    updated: &FundAllocation,
) -> Result<(), String> {
    if updated.proposal_id != original.proposal_id {
        return Err("Cannot change allocation proposal ID".into());
    }
    if (updated.amount - original.amount).abs() > f64::EPSILON {
        return Err("Cannot change allocation amount".into());
    }
    match (&original.status, &updated.status) {
        (AllocationStatus::Locked, AllocationStatus::Released)
        | (AllocationStatus::Locked, AllocationStatus::Refunded) => Ok(()),
        _ => Err(format!(
            "Invalid allocation status transition: {:?} → {:?}",
            original.status, updated.status
        )),
    }
}

/// HDI 0.7 single validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Timelock(timelock) => validate_create_timelock(action, timelock),
                EntryTypes::Execution(execution) => validate_create_execution(action, execution),
                EntryTypes::ExecutionAttempt(attempt) => {
                    validate_create_execution_attempt(action, attempt)
                },
                EntryTypes::GuardianVeto(veto) => validate_create_veto(action, veto),
                EntryTypes::FundAllocation(alloc) => validate_create_fund_allocation(action, alloc),
                EntryTypes::VetoOverrideVote(vote) => validate_create_override_vote(action, vote),
                EntryTypes::VetoOverrideResult(result) => {
                    validate_create_override_result(action, result)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Timelock(timelock) => {
                    validate_update_timelock(action, timelock, original_action_hash)
                }
                EntryTypes::Execution(_) => {
                    // Executions cannot be updated once created
                    Ok(ValidateCallbackResult::Invalid(
                        "Execution records cannot be modified".into(),
                    ))
                }
                EntryTypes::ExecutionAttempt(attempt) => {
                    validate_update_execution_attempt(action, attempt, original_action_hash)
                }
                EntryTypes::GuardianVeto(_) => {
                    // Vetoes cannot be updated
                    Ok(ValidateCallbackResult::Invalid(
                        "Vetoes cannot be modified".into(),
                    ))
                }
                EntryTypes::VetoOverrideVote(_) => Ok(ValidateCallbackResult::Invalid(
                    "Override votes cannot be modified".into(),
                )),
                EntryTypes::VetoOverrideResult(_) => Ok(ValidateCallbackResult::Invalid(
                    "Override results cannot be modified".into(),
                )),
                EntryTypes::FundAllocation(alloc) => {
                    validate_update_fund_allocation(action, alloc, original_action_hash)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address,
            tag: _,
            action,
        } => match link_type {
            LinkTypes::ProposalToTimelock => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TimelockToExecution => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TimelockToExecutionAttempt => {
                validate_execution_index_target(action, target_address, ExecutionIndexTargetKind::Attempt)
            }
            LinkTypes::PendingTimelocks => Ok(ValidateCallbackResult::Valid),
            LinkTypes::GuardianToVeto => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ProposalToFundAllocation => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TimelockById => {
                validate_execution_index_target(action, target_address, ExecutionIndexTargetKind::Timelock)
            }
            LinkTypes::VetoToOverrideVotes => Ok(ValidateCallbackResult::Valid),
            LinkTypes::VetoToOverrideResult => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            link_type,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            // Allow removing from pending list when executed/cancelled
            LinkTypes::PendingTimelocks => Ok(ValidateCallbackResult::Valid),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

#[derive(Clone, Copy)]
enum ExecutionIndexTargetKind {
    Timelock,
    Attempt,
}

fn validate_execution_index_target(
    action: CreateLink,
    target_address: AnyLinkableHash,
    expected: ExecutionIndexTargetKind,
) -> ExternResult<ValidateCallbackResult> {
    let target_action_hash = target_address.into_action_hash().ok_or(wasm_error!(
        WasmErrorInner::Guest("Execution index target must be an action hash".into())
    ))?;

    let target_record = must_get_valid_record(target_action_hash)?;

    if action.author() != target_record.action().author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Execution index target must be authored by the link author".into(),
        ));
    }

    match expected {
        ExecutionIndexTargetKind::Timelock => {
            let timelock = target_record
                .entry()
                .to_app_option::<Timelock>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "TimelockById target has no Timelock entry".into()
                )))?;
            if timelock.id.trim().is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "TimelockById target has an empty timelock ID".into(),
                ));
            }
        }
        ExecutionIndexTargetKind::Attempt => {
            let attempt = target_record
                .entry()
                .to_app_option::<ExecutionAttempt>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "TimelockToExecutionAttempt target has no ExecutionAttempt entry".into()
                )))?;
            if attempt.id.trim().is_empty() || attempt.timelock_id.trim().is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Execution attempt index target has incomplete identity fields".into(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate timelock creation
fn validate_create_timelock(
    _action: Create,
    timelock: Timelock,
) -> ExternResult<ValidateCallbackResult> {
    match check_create_timelock(&timelock) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate timelock update
fn validate_update_timelock(
    action: Update,
    timelock: Timelock,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    // Get original timelock for comparison
    let original_record = must_get_valid_record(original_action_hash)?;
    let original_timelock: Timelock = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original timelock not found".into()
        )))?;

    if action.author() != original_record.action().author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the original timelock author may update the timelock".into(),
        ));
    }

    match check_update_timelock(&original_timelock, &timelock) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate execution creation
fn validate_create_execution_attempt(
    action: Create,
    attempt: ExecutionAttempt,
) -> ExternResult<ValidateCallbackResult> {
    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("ExecutionAttempt", "executor", &attempt.executor, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    match check_create_execution_attempt(&attempt) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

fn validate_update_execution_attempt(
    action: Update,
    attempt: ExecutionAttempt,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original_attempt: ExecutionAttempt = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original ExecutionAttempt not found".into()
        )))?;

    if action.author() != original_record.action().author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the original ExecutionAttempt author may update the attempt".into(),
        ));
    }

    match check_update_execution_attempt(&original_attempt, &attempt) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

fn validate_create_execution(
    action: Create,
    execution: Execution,
) -> ExternResult<ValidateCallbackResult> {
    // Bind to the committer. `execute_timelock` (execution/coordinator:297-300)
    // already compares input.executor_did against an agent_info()-derived DID,
    // so this enforces that at the DHT level. (governance Class-A, `execution:581`.)
    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("Execution", "executor", &execution.executor, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    match check_create_execution(&execution) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate guardian veto creation
fn validate_create_veto(
    action: Create,
    veto: GuardianVeto,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the veto to its committer. A forged `guardian` lets any agent freeze
    // any timelock under a real guardian's name — the highest-severity item on
    // the governance Class-A list (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md,
    // `execution:592`).
    //
    // Safe to bind: `veto_timelock` (execution/coordinator:747-749) already
    // derives the expected DID from agent_info() and rejects a mismatch, so this
    // enforces at the DHT level what the coordinator already does — and closes
    // the path where a peer bypasses the coordinator entirely.
    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("GuardianVeto", "guardian", &veto.guardian, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    match check_create_veto(&veto) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate veto override vote creation
fn validate_create_override_vote(
    action: Create,
    vote: VetoOverrideVote,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the override vote to its committer — a forged `voter_did` swings the
    // 67% veto-override threshold (governance Class-A, `execution:603`).
    //
    // Safe to bind: `cast_override_vote` (execution/coordinator:1084-1086)
    // already compares input.voter_did against an agent_info()-derived DID.
    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(msg) = require_did_is_author(
        "VetoOverrideVote",
        "voter_did",
        &vote.voter_did,
        &author_did,
    ) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    match check_create_override_vote(&vote) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate veto override result creation
fn validate_create_override_result(
    _action: Create,
    result: VetoOverrideResult,
) -> ExternResult<ValidateCallbackResult> {
    match check_create_override_result(&result) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate fund allocation creation
fn validate_create_fund_allocation(
    _action: Create,
    alloc: FundAllocation,
) -> ExternResult<ValidateCallbackResult> {
    match check_create_fund_allocation(&alloc) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate fund allocation update (status transitions)
fn validate_update_fund_allocation(
    _action: Update,
    alloc: FundAllocation,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: FundAllocation = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original fund allocation not found".into()
        )))?;

    match check_update_fund_allocation(&original, &alloc) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn timelock_material_action_is_immutable() {
        let base = Timelock {
            id: "timelock-1".into(),
            proposal_id: "proposal-1".into(),
            actions: "{\"type\":\"EmitEvent\",\"event\":\"x\"}".into(),
            started: Timestamp::from_micros(1),
            expires: Timestamp::from_micros(2),
            status: TimelockStatus::Pending,
            cancellation_reason: None,
        };

        let mut changed_actions = base.clone();
        changed_actions.actions = "{\"type\":\"EmitEvent\",\"event\":\"y\"}".into();
        changed_actions.status = TimelockStatus::Ready;
        assert!(check_update_timelock(&base, &changed_actions).is_err());

        let mut changed_id = base.clone();
        changed_id.id = "timelock-2".into();
        changed_id.status = TimelockStatus::Ready;
        assert!(check_update_timelock(&base, &changed_id).is_err());

        let mut changed_expiry = base.clone();
        changed_expiry.expires = Timestamp::from_micros(3);
        changed_expiry.status = TimelockStatus::Ready;
        assert!(check_update_timelock(&base, &changed_expiry).is_err());
    }

    #[test]
    fn execution_indeterminate_is_not_a_failed_outcome() {
        assert_ne!(
            ExecutionStatus::Indeterminate,
            ExecutionStatus::Failed
        );
    }

    #[test]
    fn invoked_cannot_bypass_single_use_claim() {
        let now = Timestamp::from_micros(1);
        let base = ExecutionAttempt {
            id: "execution-attempt-bypass-1".into(),
            operation_id: "operation-bypass-1".into(),
            timelock_id: "timelock-bypass-1".into(),
            proposal_id: "proposal-bypass-1".into(),
            action_digest: "constitutional-material-action-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            action_key_digest: "constitutional-action-key-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            attempt_identity: "constitutional-attempt-identity-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            native_replay_identity: "native-replay-bypass-1".into(),
            provider_environment: "holochain-local-source-chain".into(),
            provider_audience: "mycelix-governance-execution".into(),
            adapter_identity: "governance-execution-coordinator-v1".into(),
            executor: "did:mycelix:executor".into(),
            status: ExecutionAttemptStatus::Invoked,
            prepared_at: now,
            updated_at: now,
            outcome_evidence_commitment: None,
            not_entered_marker: None,
        };

        let mut indeterminate = base.clone();
        indeterminate.status = ExecutionAttemptStatus::Indeterminate;
        assert!(check_update_execution_attempt(&base, &indeterminate).is_err());

        let mut claimed = base.clone();
        claimed.status = ExecutionAttemptStatus::InvocationClaimed;
        assert!(check_update_execution_attempt(&base, &claimed).is_ok());

        let mut succeeded = claimed.clone();
        succeeded.status = ExecutionAttemptStatus::Succeeded;
        succeeded.outcome_evidence_commitment = Some("provider-proof".into());
        assert!(check_update_execution_attempt(&claimed, &succeeded).is_ok());
    }

    #[test]
    fn execution_attempt_requires_single_use_invocation_claim() {
        let now = Timestamp::from_micros(1);
        let base = ExecutionAttempt {
            id: "execution-attempt-1".into(),
            operation_id: "operation-1".into(),
            timelock_id: "timelock-1".into(),
            proposal_id: "proposal-1".into(),
            action_digest: "constitutional-material-action-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            action_key_digest: "constitutional-action-key-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            attempt_identity: "constitutional-attempt-identity-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            native_replay_identity: "native-replay-1".into(),
            provider_environment: "holochain-local-source-chain".into(),
            provider_audience: "mycelix-governance-execution".into(),
            adapter_identity: "governance-execution-coordinator-v1".into(),
            executor: "did:mycelix:executor".into(),
            status: ExecutionAttemptStatus::Invoked,
            prepared_at: now,
            updated_at: now,
            outcome_evidence_commitment: None,
            not_entered_marker: None,
        };
        let mut claimed = base.clone();
        claimed.status = ExecutionAttemptStatus::InvocationClaimed;

        assert!(check_update_execution_attempt(&base, &claimed).is_ok());

        let mut second_claim = claimed.clone();
        second_claim.updated_at = Timestamp::from_micros(2);
        assert!(check_update_execution_attempt(&claimed, &second_claim).is_err());
    }

    #[test]
    fn execution_attempt_starts_only_at_dispatch_pending() {
        let now = Timestamp::from_micros(1);
        let attempt = ExecutionAttempt {
            id: "execution-attempt-1".into(),
            operation_id: "operation-1".into(),
            timelock_id: "timelock-1".into(),
            proposal_id: "proposal-1".into(),
            action_digest: "constitutional-material-action-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            action_key_digest: "constitutional-action-key-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            attempt_identity: "constitutional-attempt-identity-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            native_replay_identity: "native-replay-1".into(),
            provider_environment: "holochain-local-source-chain".into(),
            provider_audience: "mycelix-governance-execution".into(),
            adapter_identity: "governance-execution-coordinator-v1".into(),
            executor: "did:mycelix:executor".into(),
            status: ExecutionAttemptStatus::Invoked,
            prepared_at: now,
            updated_at: now,
            outcome_evidence_commitment: None,
            not_entered_marker: None,
        };
        assert!(check_create_execution_attempt(&attempt).is_err());
    }

    #[test]
    fn execution_attempt_rejects_terminal_state_without_evidence() {
        let now = Timestamp::from_micros(1);
        let base = ExecutionAttempt {
            id: "execution-attempt-1".into(),
            timelock_id: "timelock-1".into(),
            proposal_id: "proposal-1".into(),
            action_digest: "constitutional-material-action-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            action_key_digest: "constitutional-action-key-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            attempt_identity: "constitutional-attempt-identity-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            native_replay_identity: "native-replay-1".into(),
            executor: "did:mycelix:executor".into(),
            status: ExecutionAttemptStatus::DispatchPending,
            prepared_at: now,
            updated_at: now,
            outcome_evidence_commitment: None,
            not_entered_marker: None,
        };
        let mut succeeded = base.clone();
        succeeded.status = ExecutionAttemptStatus::Succeeded;
        assert!(check_update_execution_attempt(&base, &succeeded).is_err());
    }


    use super::*;

    fn ts(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    fn make_timelock() -> Timelock {
        Timelock {
            id: "tl-1".into(),
            proposal_id: "prop-1".into(),
            actions: r#"["transfer"]"#.into(),
            started: ts(1_000_000),
            expires: ts(2_000_000),
            status: TimelockStatus::Pending,
            cancellation_reason: None,
        }
    }

    fn make_create() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    /// DID of the agent `make_create()` attributes actions to. Fixtures meant to
    /// be VALID must use this — veto/override-vote bind their DID to the committer.
    fn test_author_did() -> String {
        format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![0; 36]))
    }

    fn make_execution() -> Execution {
        Execution {
            id: "ex-1".into(),
            timelock_id: "tl-1".into(),
            proposal_id: "prop-1".into(),
            executor: "did:key:z6Mk".into(),
            status: ExecutionStatus::Success,
            result: Some(r#"{"ok":true}"#.into()),
            error: None,
            executed_at: ts(3_000_000),
        }
    }

    fn make_veto() -> GuardianVeto {
        GuardianVeto {
            id: "v-1".into(),
            timelock_id: "tl-1".into(),
            guardian: test_author_did(),
            reason: "Emergency safety concern".into(),
            vetoed_at: ts(1_500_000),
            affected_proposal_id: Some("prop-1".into()),
            justification_hash: Some(
                "a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2".into(),
            ),
            threat_category: Some("safety".into()),
            haptic_proof: None,
        }
    }

    fn make_fund_allocation() -> FundAllocation {
        FundAllocation {
            id: "fa-1".into(),
            proposal_id: "prop-1".into(),
            timelock_id: "tl-1".into(),
            source_account: "treasury-main".into(),
            amount: 1000.0,
            currency: "credits".into(),
            locked_at: ts(1_000_000),
            status: AllocationStatus::Locked,
            status_reason: None,
        }
    }

    // ---- Timelock creation tests ----

    #[test]
    fn test_valid_timelock_accepted() {
        assert!(check_create_timelock(&make_timelock()).is_ok());
    }

    #[test]
    fn test_timelock_expiry_after_start() {
        let mut tl = make_timelock();
        // expires == started
        tl.expires = tl.started;
        assert_eq!(
            check_create_timelock(&tl).unwrap_err(),
            "Timelock expiry must be after start"
        );

        // expires < started
        tl.expires = ts(500_000);
        assert_eq!(
            check_create_timelock(&tl).unwrap_err(),
            "Timelock expiry must be after start"
        );
    }

    #[test]
    fn test_timelock_actions_must_be_json() {
        let mut tl = make_timelock();
        tl.actions = "not json {{{".into();
        assert_eq!(
            check_create_timelock(&tl).unwrap_err(),
            "Actions must be valid JSON"
        );
    }

    #[test]
    fn test_timelock_initial_status_pending() {
        let mut tl = make_timelock();
        tl.status = TimelockStatus::Ready;
        assert_eq!(
            check_create_timelock(&tl).unwrap_err(),
            "Initial timelock status must be Pending"
        );
    }

    // ---- Timelock update / status transition tests ----

    #[test]
    fn test_timelock_valid_status_transitions() {
        let original = make_timelock();

        // Pending → Ready
        let mut updated = original.clone();
        updated.status = TimelockStatus::Ready;
        assert!(check_update_timelock(&original, &updated).is_ok());

        // Pending → Cancelled
        let mut updated = original.clone();
        updated.status = TimelockStatus::Cancelled;
        assert!(check_update_timelock(&original, &updated).is_ok());

        // Ready → Executed
        let mut ready = original.clone();
        ready.status = TimelockStatus::Ready;
        let mut updated = ready.clone();
        updated.status = TimelockStatus::Executed;
        assert!(check_update_timelock(&ready, &updated).is_ok());

        // Ready → Failed
        let mut updated = ready.clone();
        updated.status = TimelockStatus::Failed;
        assert!(check_update_timelock(&ready, &updated).is_ok());

        // Ready → Cancelled
        let mut updated = ready.clone();
        updated.status = TimelockStatus::Cancelled;
        assert!(check_update_timelock(&ready, &updated).is_ok());
    }

    #[test]
    fn test_timelock_veto_override_transitions() {
        let original = make_timelock();

        // Ready → Vetoed (guardian veto)
        let mut ready = original.clone();
        ready.status = TimelockStatus::Ready;
        let mut vetoed = ready.clone();
        vetoed.status = TimelockStatus::Vetoed;
        assert!(check_update_timelock(&ready, &vetoed).is_ok());

        // Pending → Vetoed (guardian veto on pending)
        let mut vetoed = original.clone();
        vetoed.status = TimelockStatus::Vetoed;
        assert!(check_update_timelock(&original, &vetoed).is_ok());

        // Vetoed → Ready (override succeeded)
        let mut restored = vetoed.clone();
        restored.status = TimelockStatus::Ready;
        assert!(check_update_timelock(&vetoed, &restored).is_ok());

        // Vetoed → Cancelled (override failed or window expired)
        let mut cancelled = vetoed.clone();
        cancelled.status = TimelockStatus::Cancelled;
        assert!(check_update_timelock(&vetoed, &cancelled).is_ok());

        // Vetoed → Executed (INVALID — must go through Ready first)
        let mut bad = vetoed.clone();
        bad.status = TimelockStatus::Executed;
        assert!(check_update_timelock(&vetoed, &bad).is_err());
    }

    #[test]
    fn test_timelock_invalid_status_transition() {
        let original = make_timelock(); // Pending
        let mut updated = original.clone();
        updated.status = TimelockStatus::Executed; // Pending → Executed not allowed
        assert_eq!(
            check_update_timelock(&original, &updated).unwrap_err(),
            "Invalid timelock status transition"
        );
    }

    // ---- Veto override vote tests ----

    #[test]
    fn test_override_vote_valid() {
        let vote = VetoOverrideVote {
            id: "ov-1".into(),
            veto_id: "v-1".into(),
            voter_did: test_author_did(),
            supports_override: true,
            phi_score: 0.7,
            voted_at: ts(2_000_000),
        };
        assert!(check_create_override_vote(&vote).is_ok());
    }

    #[test]
    fn test_override_vote_requires_did() {
        let vote = VetoOverrideVote {
            id: "ov-1".into(),
            veto_id: "v-1".into(),
            voter_did: "not-a-did".into(),
            supports_override: true,
            phi_score: 0.5,
            voted_at: ts(2_000_000),
        };
        assert!(check_create_override_vote(&vote).is_err());
    }

    #[test]
    fn test_override_vote_requires_veto_id() {
        let vote = VetoOverrideVote {
            id: "ov-1".into(),
            veto_id: "".into(),
            voter_did: "did:key:z6Test".into(),
            supports_override: true,
            phi_score: 0.5,
            voted_at: ts(2_000_000),
        };
        assert!(check_create_override_vote(&vote).is_err());
    }

    // ---- Veto override result tests ----

    #[test]
    fn test_override_result_threshold_must_be_080() {
        let result = VetoOverrideResult {
            id: "or-1".into(),
            veto_id: "v-1".into(),
            timelock_id: "tl-1".into(),
            override_votes_for: 8.0,
            override_votes_against: 2.0,
            total_eligible_voters: 10,
            override_threshold: 0.67,
            override_succeeded: true,
            resolved_at: ts(3_000_000),
        };
        assert!(check_create_override_result(&result).is_ok());

        // Wrong threshold rejected
        let mut bad = result.clone();
        bad.override_threshold = 0.51;
        assert!(check_create_override_result(&bad).is_err());
    }

    #[test]
    fn test_override_result_requires_ids() {
        let result = VetoOverrideResult {
            id: "or-1".into(),
            veto_id: "".into(),
            timelock_id: "tl-1".into(),
            override_votes_for: 5.0,
            override_votes_against: 5.0,
            total_eligible_voters: 10,
            override_threshold: 0.67,
            override_succeeded: false,
            resolved_at: ts(3_000_000),
        };
        assert!(check_create_override_result(&result).is_err());
    }

    #[test]
    fn test_veto_override_threshold_constant() {
        // Constitutional threshold: 2/3 (Art. III, Sec. 5.3)
        assert!((VETO_OVERRIDE_THRESHOLD - 0.67).abs() < f64::EPSILON);
    }

    #[test]
    fn test_veto_override_window_is_48_hours() {
        assert_eq!(VETO_OVERRIDE_WINDOW_US, 48 * 3600 * 1_000_000);
    }

    #[test]
    fn test_veto_yearly_limit_constant() {
        // Constitutional: Art. III, Sec. 5.4 — 3 vetoes per 12 months
        assert_eq!(VETO_YEARLY_LIMIT, 3);
    }

    #[test]
    fn test_strategic_override_sunset_is_36_months() {
        // Constitutional: Art. III, Sec. 3 — 36 months from Genesis Epoch
        let thirty_six_months_us: i64 = 36 * 30 * 24 * 3600 * 1_000_000;
        assert_eq!(STRATEGIC_OVERRIDE_SUNSET_US, thirty_six_months_us);
    }

    #[test]
    fn test_is_post_sunset() {
        let genesis = 0_i64;
        // Before sunset: 35 months
        let before = 35 * 30 * 24 * 3600 * 1_000_000_i64;
        assert!(!is_post_sunset(genesis, before));

        // After sunset: 37 months
        let after = 37 * 30 * 24 * 3600 * 1_000_000_i64;
        assert!(is_post_sunset(genesis, after));

        // Exactly at sunset boundary
        assert!(!is_post_sunset(genesis, STRATEGIC_OVERRIDE_SUNSET_US));
    }

    #[test]
    fn test_charter_threat_categories() {
        assert!(CHARTER_THREAT_CATEGORIES.contains(&"constitutional_violation"));
        assert!(CHARTER_THREAT_CATEGORIES.contains(&"core_principle_violation"));
        assert!(CHARTER_THREAT_CATEGORIES.contains(&"member_rights_violation"));
        assert!(!CHARTER_THREAT_CATEGORIES.contains(&"fiscal"));
    }

    // ---- Participation insurance (adaptive threshold) tests ----

    #[test]
    fn test_adaptive_threshold_initial() {
        // Constitutional threshold: 2/3 (Art. III, Sec. 5.3)
        assert!((adaptive_override_threshold(0) - 0.67).abs() < f64::EPSILON);
    }

    #[test]
    fn test_adaptive_threshold_decays() {
        assert!((adaptive_override_threshold(1) - 0.62).abs() < f64::EPSILON);
        assert!((adaptive_override_threshold(2) - 0.60).abs() < 0.001);
    }

    #[test]
    fn test_adaptive_threshold_floor() {
        // Floor at 60%
        assert!((adaptive_override_threshold(3) - 0.60).abs() < 0.001);
        assert!((adaptive_override_threshold(10) - 0.60).abs() < 0.001);
        assert!((adaptive_override_threshold(100) - 0.60).abs() < 0.001);
    }

    #[test]
    fn test_adaptive_threshold_never_below_two_thirds() {
        for attempts in 0..1000 {
            assert!(
                adaptive_override_threshold(attempts) >= OVERRIDE_THRESHOLD_FLOOR,
                "Threshold must never go below 2/3 supermajority"
            );
        }
    }

    // ---- Execution tests ----

    #[test]
    fn test_execution_executor_must_be_did() {
        let mut ex = make_execution();
        ex.executor = "agent:abc".into();
        assert_eq!(
            check_create_execution(&ex).unwrap_err(),
            "Executor must be a valid DID"
        );

        // Valid DID passes
        ex.executor = "did:key:z6Mk".into();
        assert!(check_create_execution(&ex).is_ok());
    }

    // ---- Veto tests ----

    #[test]
    fn test_veto_guardian_must_be_did() {
        let mut v = make_veto();
        v.guardian = "not-a-did".into();
        assert_eq!(
            check_create_veto(&v).unwrap_err(),
            "Guardian must be a valid DID"
        );
    }

    #[test]
    fn test_veto_reason_required() {
        let mut v = make_veto();
        v.reason = String::new();
        assert_eq!(
            check_create_veto(&v).unwrap_err(),
            "Veto reason is required"
        );
    }

    #[test]
    fn test_veto_justification_hash_format() {
        let mut v = make_veto();
        // Valid 64-char hex passes
        assert!(check_create_veto(&v).is_ok());

        // Too short
        v.justification_hash = Some("abc123".into());
        assert!(
            check_create_veto(&v)
                .unwrap_err()
                .contains("64-character hex")
        );

        // Non-hex chars
        v.justification_hash =
            Some("g1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2".into());
        assert!(
            check_create_veto(&v)
                .unwrap_err()
                .contains("64-character hex")
        );

        // None is valid (backward compat)
        v.justification_hash = None;
        assert!(check_create_veto(&v).is_ok());
    }

    #[test]
    fn test_veto_threat_category_not_empty() {
        let mut v = make_veto();
        v.threat_category = Some(String::new());
        assert!(
            check_create_veto(&v)
                .unwrap_err()
                .contains("threat_category")
        );

        v.threat_category = Some("fiscal".into());
        assert!(check_create_veto(&v).is_ok());

        // None is valid (backward compat)
        v.threat_category = None;
        assert!(check_create_veto(&v).is_ok());
    }

    // ---- Fund allocation tests ----

    #[test]
    fn test_fund_allocation_amount_positive() {
        let mut fa = make_fund_allocation();
        fa.amount = 0.0;
        assert_eq!(
            check_create_fund_allocation(&fa).unwrap_err(),
            "Fund allocation amount must be positive and finite"
        );

        fa.amount = -50.0;
        assert_eq!(
            check_create_fund_allocation(&fa).unwrap_err(),
            "Fund allocation amount must be positive and finite"
        );

        fa.amount = f64::NAN;
        assert_eq!(
            check_create_fund_allocation(&fa).unwrap_err(),
            "Fund allocation amount must be positive and finite"
        );

        fa.amount = f64::INFINITY;
        assert_eq!(
            check_create_fund_allocation(&fa).unwrap_err(),
            "Fund allocation amount must be positive and finite"
        );
    }

    #[test]
    fn test_fund_allocation_initial_status_locked() {
        let mut fa = make_fund_allocation();
        fa.status = AllocationStatus::Released;
        assert_eq!(
            check_create_fund_allocation(&fa).unwrap_err(),
            "Initial allocation status must be Locked"
        );
    }

    #[test]
    fn test_fund_allocation_valid_transitions() {
        let original = make_fund_allocation(); // Locked

        // Locked → Released
        let mut updated = original.clone();
        updated.status = AllocationStatus::Released;
        assert!(check_update_fund_allocation(&original, &updated).is_ok());

        // Locked → Refunded
        let mut updated = original.clone();
        updated.status = AllocationStatus::Refunded;
        assert!(check_update_fund_allocation(&original, &updated).is_ok());

        // Released → Refunded should fail
        let mut released = original.clone();
        released.status = AllocationStatus::Released;
        let mut updated = released.clone();
        updated.status = AllocationStatus::Refunded;
        assert!(check_update_fund_allocation(&released, &updated).is_err());
    }

    #[test]
    fn test_veto_forged_guardian_is_rejected() {
        // A forged guardian freezes any timelock under a real guardian's name.
        let mut v = make_veto();
        v.guardian = "did:mycelix:uhCAkSomeoneElse".into();
        let result = validate_create_veto(make_create(), v).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("GuardianVeto") && msg.contains("forgery"),
                    "got: {msg}"
                )
            }
            other => panic!("forged guardian must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_veto_from_the_committing_guardian_is_accepted() {
        let result = validate_create_veto(make_create(), make_veto()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_override_vote_forged_voter_is_rejected() {
        // A forged voter_did swings the 67% veto-override threshold.
        let vote = VetoOverrideVote {
            id: "ov-1".into(),
            veto_id: "v-1".into(),
            voter_did: "did:mycelix:uhCAkSomeoneElse".into(),
            supports_override: true,
            phi_score: 0.7,
            voted_at: ts(2_000_000),
        };
        let result = validate_create_override_vote(make_create(), vote).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => assert!(
                msg.contains("VetoOverrideVote") && msg.contains("forgery"),
                "got: {msg}"
            ),
            other => panic!("forged voter_did must be rejected, got {other:?}"),
        }
    }

}

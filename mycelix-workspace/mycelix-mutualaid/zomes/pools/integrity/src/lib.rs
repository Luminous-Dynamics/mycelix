// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Pools Integrity Zome - Community mutual aid pools
//!
//! This zome defines the data structures and validation rules for mutual aid
//! pools, contributions, and disbursements within the Mycelix network.

use hdi::prelude::*;

/// Anchor entry type for string-based link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct Anchor(pub String);

impl Anchor {
    pub fn new(value: impl Into<String>) -> Self {
        Anchor(value.into())
    }
}

/// Rules for pool contributions
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContributionRule {
    /// Minimum monthly contribution required (in smallest currency unit)
    pub min_monthly: u64,
    /// Maximum amount that can be withdrawn per request
    pub max_withdrawal: u64,
    /// Cooldown period in days between withdrawals
    pub cooldown_days: u32,
}

impl Default for ContributionRule {
    fn default() -> Self {
        ContributionRule {
            min_monthly: 0,
            max_withdrawal: u64::MAX,
            cooldown_days: 0,
        }
    }
}

/// Rules for pool disbursements
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisbursementRule {
    /// Minimum number of approvals required
    pub min_approvals: u32,
    /// Percentage of members that must approve (0-100)
    pub approval_threshold_percent: u8,
    /// Maximum disbursement per request
    pub max_disbursement: u64,
    /// Whether emergency disbursements bypass approvals
    pub allow_emergency_bypass: bool,
}

impl Default for DisbursementRule {
    fn default() -> Self {
        DisbursementRule {
            min_approvals: 1,
            approval_threshold_percent: 50,
            max_disbursement: u64::MAX,
            allow_emergency_bypass: false,
        }
    }
}

/// Pool status
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PoolStatus {
    Active,
    Paused,
    Closed,
}

/// A mutual aid pool managed by a community
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MutualAidPool {
    /// Unique identifier for this pool
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// Description of the pool's purpose
    pub description: String,
    /// List of member DIDs
    pub members: Vec<String>,
    /// Rules for contributions
    pub contribution_rules: ContributionRule,
    /// Rules for disbursements
    pub disbursement_rules: DisbursementRule,
    /// Current balance (in smallest currency unit)
    pub balance: u64,
    /// Pool status
    pub status: PoolStatus,
    /// Timestamp when pool was created
    pub created_at: Timestamp,
    /// Timestamp when pool was last updated
    pub updated_at: Timestamp,
}

/// A contribution to a mutual aid pool
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Contribution {
    /// Unique identifier for this contribution
    pub id: String,
    /// Reference to the pool
    pub pool_id: String,
    /// DID of the contributing member
    pub member_did: String,
    /// Amount contributed (in smallest currency unit)
    pub amount: u64,
    /// Optional note from the contributor
    pub note: Option<String>,
    /// Timestamp of the contribution
    pub timestamp: Timestamp,
}

/// Status of a disbursement request
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisbursementStatus {
    Pending,
    Approved,
    Rejected,
    Completed,
    Cancelled,
}

/// A disbursement from a mutual aid pool
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Disbursement {
    /// Unique identifier for this disbursement
    pub id: String,
    /// Reference to the pool
    pub pool_id: String,
    /// DID of the recipient
    pub recipient_did: String,
    /// Amount requested/disbursed (in smallest currency unit)
    pub amount: u64,
    /// Reason for the disbursement
    pub reason: String,
    /// DIDs of members who approved
    pub approved_by: Vec<String>,
    /// DIDs of members who rejected
    pub rejected_by: Vec<String>,
    /// Current status
    pub status: DisbursementStatus,
    /// Whether this is an emergency request
    pub is_emergency: bool,
    /// Timestamp of the request
    pub requested_at: Timestamp,
    /// Timestamp when processed (if applicable)
    pub processed_at: Option<Timestamp>,
}

/// Pool membership record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PoolMembership {
    /// Pool ID
    pub pool_id: String,
    /// Member DID
    pub member_did: String,
    /// Role in the pool
    pub role: MemberRole,
    /// Timestamp of joining
    pub joined_at: Timestamp,
    /// Total contributed by this member
    pub total_contributed: u64,
    /// Total received by this member
    pub total_received: u64,
    /// Last contribution timestamp
    pub last_contribution: Option<Timestamp>,
    /// Last disbursement timestamp
    pub last_disbursement: Option<Timestamp>,
}

/// Member role within a pool
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MemberRole {
    Admin,
    Member,
    Observer,
}

/// All entry types for this zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    #[entry_type(visibility = "public")]
    MutualAidPool(MutualAidPool),
    #[entry_type(visibility = "public")]
    Contribution(Contribution),
    #[entry_type(visibility = "public")]
    Disbursement(Disbursement),
    #[entry_type(visibility = "public")]
    PoolMembership(PoolMembership),
}

/// Link types for connecting entries
#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to all pools
    AnchorToPool,
    /// Pool to its contributions
    PoolToContribution,
    /// Pool to its disbursements
    PoolToDisbursement,
    /// Pool to its memberships
    PoolToMembership,
    /// Member DID to their pool memberships
    MemberToMembership,
    /// Member DID to their contributions
    MemberToContribution,
    /// Member DID to their received disbursements
    MemberToDisbursement,
    /// Pool to pending disbursements
    PoolToPendingDisbursement,
}

/// Validation errors for pools zome
#[derive(Debug)]
pub enum PoolsError {
    InvalidDid(String),
    InvalidId(String),
    ZeroAmount,
    EmptyName,
    EmptyReason,
    InvalidApprovalThreshold(u8),
    InsufficientBalance,
    NotAMember,
}

impl std::fmt::Display for PoolsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDid(s) => write!(f, "Invalid DID format: {}", s),
            Self::InvalidId(s) => write!(f, "Invalid ID format: {}", s),
            Self::ZeroAmount => write!(f, "Amount cannot be zero"),
            Self::EmptyName => write!(f, "Pool name cannot be empty"),
            Self::EmptyReason => write!(f, "Reason cannot be empty"),
            Self::InvalidApprovalThreshold(t) => write!(f, "Invalid approval threshold: {}", t),
            Self::InsufficientBalance => write!(f, "Disbursement exceeds pool balance"),
            Self::NotAMember => write!(f, "Member not in pool"),
        }
    }
}

/// Validate that a DID has a valid format
fn validate_did(did: &str) -> ExternResult<()> {
    if did.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            PoolsError::InvalidDid("DID cannot be empty".to_string()).to_string()
        )));
    }
    // Basic DID format check: did:method:identifier
    if !did.starts_with("did:") || did.split(':').count() < 3 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            PoolsError::InvalidDid(format!("Invalid DID format: {}", did)).to_string()
        )));
    }
    Ok(())
}

/// Validate that an ID is non-empty
fn validate_id(id: &str, field_name: &str) -> ExternResult<()> {
    if id.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            PoolsError::InvalidId(format!("{} cannot be empty", field_name)).to_string()
        )));
    }
    Ok(())
}

/// Validate a MutualAidPool entry. `members` is NOT bound to the
/// committer -- see the zome-wide disclosed-gap note on
/// validate_update_entry_type above. Case (d).
fn validate_mutual_aid_pool(pool: &MutualAidPool) -> ExternResult<ValidateCallbackResult> {
    // Validate ID
    validate_id(&pool.id, "Pool ID")?;

    // Validate name is not empty
    if pool.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            PoolsError::EmptyName.to_string(),
        ));
    }

    // Validate all member DIDs
    for member in &pool.members {
        validate_did(member)?;
    }

    // Validate disbursement approval threshold
    if pool.disbursement_rules.approval_threshold_percent > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            PoolsError::InvalidApprovalThreshold(
                pool.disbursement_rules.approval_threshold_percent,
            )
            .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a Contribution entry. `member_did` is NOT bound to the
/// committer -- case (d), see above.
fn validate_contribution(contribution: &Contribution) -> ExternResult<ValidateCallbackResult> {
    // Validate IDs
    validate_id(&contribution.id, "Contribution ID")?;
    validate_id(&contribution.pool_id, "Pool ID")?;

    // Validate member DID
    validate_did(&contribution.member_did)?;

    // Validate amount is positive
    if contribution.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            PoolsError::ZeroAmount.to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a Disbursement entry. `recipient_did`/`approved_by`/
/// `rejected_by` are NOT bound to the committer -- case (d), see above
/// (vote_disbursement's vote-forgery vector is the live consequence).
fn validate_disbursement(disbursement: &Disbursement) -> ExternResult<ValidateCallbackResult> {
    // Validate IDs
    validate_id(&disbursement.id, "Disbursement ID")?;
    validate_id(&disbursement.pool_id, "Pool ID")?;

    // Validate recipient DID
    validate_did(&disbursement.recipient_did)?;

    // Validate amount is positive
    if disbursement.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            PoolsError::ZeroAmount.to_string(),
        ));
    }

    // Validate reason is not empty
    if disbursement.reason.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            PoolsError::EmptyReason.to_string(),
        ));
    }

    // Validate approver and rejector DIDs
    for approver in &disbursement.approved_by {
        validate_did(approver)?;
    }
    for rejector in &disbursement.rejected_by {
        validate_did(rejector)?;
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a PoolMembership entry. `member_did` is NOT bound to the
/// committer -- case (d), see above.
fn validate_pool_membership(membership: &PoolMembership) -> ExternResult<ValidateCallbackResult> {
    // Validate IDs
    validate_id(&membership.pool_id, "Pool ID")?;

    // Validate member DID
    validate_did(&membership.member_did)?;

    Ok(ValidateCallbackResult::Valid)
}

/// Genesis self-check callback
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::MutualAidPool(pool) => validate_mutual_aid_pool(&pool),
                EntryTypes::Contribution(contribution) => validate_contribution(&contribution),
                EntryTypes::Disbursement(disbursement) => validate_disbursement(&disbursement),
                EntryTypes::PoolMembership(membership) => validate_pool_membership(&membership),
            },
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                ..
            } => validate_update_entry_type(original_action_hash, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToPool
            | LinkTypes::PoolToContribution
            | LinkTypes::PoolToDisbursement
            | LinkTypes::PoolToMembership
            | LinkTypes::MemberToMembership
            | LinkTypes::MemberToContribution
            | LinkTypes::MemberToDisbursement
            | LinkTypes::PoolToPendingDisbursement => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap):
        // process_disbursement (via its caller) manages the
        // PoolToPendingDisbursement link independent of who originally
        // requested the disbursement, and there is no local DID-to-agent
        // verification convention (see the zome-wide disclosed-gap note
        // below) to restrict link deletion to "the real requester" even
        // if we wanted to.
        FlatOp::RegisterDeleteLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToPool
            | LinkTypes::PoolToContribution
            | LinkTypes::PoolToDisbursement
            | LinkTypes::PoolToMembership
            | LinkTypes::MemberToMembership
            | LinkTypes::MemberToContribution
            | LinkTypes::MemberToDisbursement
            | LinkTypes::PoolToPendingDisbursement => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 29th confirmed instance of this
            // exact bug pattern this pass. Found + fixed 2026-07-09
            // during the P0 author-binding pass. Route through the same
            // per-type validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => {
                validate_update_entry_type(action.original_action_address, app_entry)
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry here, so this is pure hardening, zero
            // functional impact.
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// **Zome-wide disclosed, NOT-fixed gap** (same class as
/// mycelix-mutualaid/bridge and mycelix-mutualaid/requests, fixed
/// earlier this pass): every identity-bearing field here
/// (MutualAidPool.members, Contribution.member_did,
/// Disbursement.recipient_did/approved_by/rejected_by,
/// PoolMembership.member_did) is a free-form String DID with NO local
/// convention for verifying it against the committing agent's real
/// action.author. Case (d).
///
/// **Two additional, more severe findings specific to this zome, both
/// disclosed and NOT fixed here (need real design work, not
/// author-binding)**:
/// 1. `vote_disbursement` takes `voter_did` directly from caller input
///    with ZERO check that the caller is even a pool member, let alone
///    the claimed voter -- unlike `request_disbursement`, which DOES
///    verify `pool.members.contains(&recipient_did)`. Any agent can
///    stuff approval/rejection votes onto any pending disbursement
///    under an arbitrary claimed identity -- a live vote-forgery vector
///    gating the disbursement of real pooled funds.
/// 2. `process_disbursement`'s OWN code comment admits it is
///    incomplete: `"Note: In a full implementation, we would: 1. Get
///    the pool and check approval threshold... For now, just mark as
///    completed"`. It marks ANY pending disbursement Completed with
///    ZERO verification that `disbursement_rules.min_approvals`/
///    `approval_threshold_percent` was ever met -- voting is
///    structurally decorative. This can't be fixed at the integrity
///    layer either: `Disbursement` stores `pool_id: String`, not a
///    `pool_hash: ActionHash`, so there's no hash-addressable reference
///    a `must_get_valid_record` could use to cross-check the pool's
///    rules against the vote tallies even if we wanted to -- a schema
///    gap, not just a missing check.
fn validate_update_entry_type(
    original_action_hash: ActionHash,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::MutualAidPool(pool) => validate_update_pool(original_action_hash, pool),
        // No live update_entry call for Contribution (confirmed via
        // grep) -- previously silently accepted any field change. Made
        // explicitly immutable.
        EntryTypes::Contribution(_) => Ok(ValidateCallbackResult::Invalid(
            "Contributions are immutable".into(),
        )),
        EntryTypes::Disbursement(disbursement) => {
            validate_update_disbursement(original_action_hash, disbursement)
        }
        // No live update_entry call for PoolMembership either.
        EntryTypes::PoolMembership(_) => Ok(ValidateCallbackResult::Invalid(
            "Pool memberships are immutable".into(),
        )),
    }
}

/// Content restricted to members/balance/status/updated_at -- the union
/// of fields changed across add_member/contribute/update_pool_status.
/// No author requirement: none of those three flows have any
/// caller-identity check (case c, compounded by the case-(d) DID gap
/// above for the `members` field specifically).
fn validate_update_pool(
    original_action_hash: ActionHash,
    pool: MutualAidPool,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: MutualAidPool = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original pool not found".into()
        )))?;

    if pool.id != original.id
        || pool.name != original.name
        || pool.description != original.description
        || pool.contribution_rules != original.contribution_rules
        || pool.disbursement_rules != original.disbursement_rules
        || pool.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only members/balance/status/updated_at can change on a pool update".into(),
        ));
    }

    validate_mutual_aid_pool(&pool)
}

/// Content restricted to approved_by/rejected_by/status/processed_at --
/// the union of fields changed across vote_disbursement/
/// process_disbursement. No author requirement possible (case d, see
/// above) -- this closes only the wide-open bug that previously let
/// pool_id/recipient_did/amount/reason change unconditionally on update
/// too; it does NOT and cannot fix the vote-forgery or
/// threshold-bypass findings disclosed above.
fn validate_update_disbursement(
    original_action_hash: ActionHash,
    disbursement: Disbursement,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Disbursement = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original disbursement not found".into()
        )))?;

    if disbursement.id != original.id
        || disbursement.pool_id != original.pool_id
        || disbursement.recipient_did != original.recipient_did
        || disbursement.amount != original.amount
        || disbursement.reason != original.reason
        || disbursement.is_emergency != original.is_emergency
        || disbursement.requested_at != original.requested_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only approved_by/rejected_by/status/processed_at can change on a disbursement update"
                .into(),
        ));
    }

    validate_disbursement(&disbursement)
}

#[cfg(test)]
mod content_integrity_tests {
    use super::*;

    fn update_action(author: AgentPubKey) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
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

    #[test]
    fn update_entry_type_rejects_contribution_update() {
        // No live update_entry call exists for Contribution -- dead-path
        // immutability, testable without must_get_valid_record.
        let contribution = Contribution {
            id: "c-1".into(),
            pool_id: "p-1".into(),
            member_did: "did:mycelix:member1".into(),
            amount: 5000,
            note: None,
            timestamp: Timestamp::from_micros(0),
        };
        let action = update_action(me());
        let result = validate_update_entry_type(
            action.original_action_address,
            EntryTypes::Contribution(contribution),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_pool_membership_update() {
        let membership = PoolMembership {
            pool_id: "p-1".into(),
            member_did: "did:mycelix:member1".into(),
            role: MemberRole::Member,
            joined_at: Timestamp::from_micros(0),
            total_contributed: 0,
            total_received: 0,
            last_contribution: None,
            last_disbursement: None,
        };
        let action = update_action(me());
        let result = validate_update_entry_type(
            action.original_action_address,
            EntryTypes::PoolMembership(membership),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Space Integrity Zome
//!
//! Entry types and validation for private spaces (families, squads, cooperatives)
//! within the public Commons DHT. Implements the "membrane factory" concept from
//! the Fractal CivOS architecture.
//!
//! Spaces use capability grants for access control: only members with valid
//! grants can read/write space-scoped entries.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_author_match, check_link_author_match};

/// Anchor for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// A private space within the Commons
///
/// Spaces provide sub-group privacy within the public DHT using
/// capability-grant-based access control.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Space {
    /// Unique space identifier
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// Space type (family, squad, cooperative, custom)
    pub space_type: SpaceType,
    /// Description of the space's purpose
    pub description: String,
    /// Agent who created the space
    pub creator: AgentPubKey,
    /// Maximum number of members (0 = unlimited)
    pub max_members: u32,
    /// Whether new members need approval from existing members
    pub requires_approval: bool,
    /// Minimum number of approvals needed for new members
    pub approval_threshold: u32,
    /// Whether the space is currently accepting new members
    pub open: bool,
    /// Creation timestamp
    pub created_at: Timestamp,
}

/// Types of private spaces
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SpaceType {
    /// Family unit — small, high-trust
    Family,
    /// Squad — project team, working group
    Squad,
    /// Cooperative — economic collaboration
    Cooperative,
    /// Custom space type
    Custom(String),
}

/// Membership in a space
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Membership {
    /// Reference to parent space
    pub space_id: String,
    /// Member's agent public key
    pub member: AgentPubKey,
    /// Member's role within the space
    pub role: MemberRole,
    /// Whether membership is active
    pub active: bool,
    /// Who invited/approved this member
    pub invited_by: AgentPubKey,
    /// Timestamp of joining
    pub joined_at: Timestamp,
}

/// Roles within a space
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MemberRole {
    /// Full admin rights (can invite, remove, modify space)
    Admin,
    /// Standard member (can read/write within space)
    Member,
    /// Read-only observer
    Observer,
}

/// Capability token for space access
///
/// Wraps a Holochain capability grant with space-specific metadata.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SpaceCapability {
    /// Reference to parent space
    pub space_id: String,
    /// Agent this capability is granted to
    pub grantee: AgentPubKey,
    /// What functions this capability allows
    pub allowed_functions: Vec<String>,
    /// Expiry timestamp (None = no expiry)
    pub expires_at: Option<Timestamp>,
    /// Whether this capability has been revoked
    pub revoked: bool,
    /// Timestamp of grant
    pub granted_at: Timestamp,
    /// Agent who granted this capability (must equal the committing agent).
    pub granted_by: AgentPubKey,
    /// ActionHash of `granted_by`'s own active Admin-role Membership entry
    /// for this space -- proof of authority to grant capabilities. Unlike
    /// plain Member/Observer memberships, capability grants have no
    /// "founding" bootstrap case: the coordinator's grant_access already
    /// requires verified admin status for every grant, no exceptions.
    pub authorized_by: ActionHash,
}

/// Invitation to join a space (pending approval)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SpaceInvitation {
    /// Reference to parent space
    pub space_id: String,
    /// Agent being invited
    pub invitee: AgentPubKey,
    /// Agent who created the invitation
    pub inviter: AgentPubKey,
    /// Optional message
    pub message: String,
    /// Current approval count
    pub approvals: u32,
    /// Agents who have approved
    pub approved_by: Vec<AgentPubKey>,
    /// Status: pending, approved, rejected, expired
    pub status: InvitationStatus,
    /// Timestamp
    pub created_at: Timestamp,
}

/// Invitation status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum InvitationStatus {
    Pending,
    Approved,
    Rejected,
    Expired,
}

/// A booking for a shared resource within a space
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ResourceBooking {
    /// Unique booking identifier
    pub id: String,
    /// Reference to parent space
    pub space_id: String,
    /// Name of the resource being booked (room, tool, equipment)
    pub resource_name: String,
    /// Agent who made the booking
    pub booked_by: AgentPubKey,
    /// Start time (microseconds since epoch)
    pub start_time: u64,
    /// End time (microseconds since epoch)
    pub end_time: u64,
    /// Current booking status
    pub status: BookingStatus,
    /// Optional notes about the booking
    pub notes: String,
    /// Creation timestamp
    pub created_at: Timestamp,
}

/// Booking status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum BookingStatus {
    Pending,
    Confirmed,
    Cancelled,
}

/// A recurring schedule/event within a space
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SpaceSchedule {
    /// Unique schedule identifier
    pub id: String,
    /// Reference to parent space
    pub space_id: String,
    /// Event/meeting title
    pub title: String,
    /// Description of the event
    pub description: String,
    /// How often this recurs
    pub recurrence: Recurrence,
    /// Next occurrence (microseconds since epoch)
    pub next_occurrence: u64,
    /// Duration in minutes
    pub duration_minutes: u32,
    /// Agent who created the schedule
    pub creator: AgentPubKey,
    /// Creation timestamp
    pub created_at: Timestamp,
}

/// Recurrence pattern for schedules
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum Recurrence {
    Once,
    Daily,
    Weekly,
    Monthly,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Space(Space),
    Membership(Membership),
    SpaceCapability(SpaceCapability),
    SpaceInvitation(SpaceInvitation),
    ResourceBooking(ResourceBooking),
    SpaceSchedule(SpaceSchedule),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All spaces anchor → space
    AllSpaces,
    /// Space → its members
    SpaceToMembers,
    /// Space → capability grants
    SpaceToCapabilities,
    /// Space → invitations
    SpaceToInvitations,
    /// Agent → spaces they belong to
    AgentToSpaces,
    /// Space type anchor → spaces of that type
    TypeToSpaces,
    /// Space → resource bookings
    SpaceToBookings,
    /// Space → scheduled events
    SpaceToSchedules,
    /// Agent → their bookings
    AgentToBookings,
}

/// Validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Space(space) => validate_create_space(action, space),
                EntryTypes::Membership(membership) => {
                    validate_create_membership(action, membership)
                }
                EntryTypes::SpaceCapability(cap) => validate_create_capability(action, cap),
                EntryTypes::SpaceInvitation(inv) => validate_create_invitation(action, inv),
                EntryTypes::ResourceBooking(booking) => validate_create_booking(action, booking),
                EntryTypes::SpaceSchedule(schedule) => validate_create_schedule(action, schedule),
            },
            // Every arm below except Space/Anchor previously returned an
            // unconditional Valid with no author check at all -- any agent
            // could rewrite anyone else's membership, capability grant,
            // invitation, booking, or schedule (P0 author-binding gap, the
            // wide-open-update class). Space itself had a dedicated
            // validate_update_space, but that function only checked field
            // bounds, never who was allowed to update -- same gap, just
            // hidden behind a real-looking function name.
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Space(space) => {
                    let original = must_get_action(original_action_hash)?;
                    let author_check =
                        check_author_match(original.action().author(), &action.author, "update");
                    if !matches!(author_check, ValidateCallbackResult::Valid) {
                        return Ok(author_check);
                    }
                    validate_update_space(action, space)
                }
                EntryTypes::Membership(new_membership) => {
                    let original_record = must_get_valid_record(original_action_hash)?;
                    let author_check = check_author_match(
                        original_record.action().author(),
                        &action.author,
                        "update",
                    );
                    if !matches!(author_check, ValidateCallbackResult::Valid) {
                        return Ok(author_check);
                    }
                    // P0 privilege-escalation fix, update-path half: without
                    // this, an agent could create an ordinary Member
                    // membership (allowed by the current model -- see
                    // validate_create_membership) and then simply update
                    // their own entry's role to Admin, bypassing the
                    // create-time founding check entirely.
                    let original_membership: Membership = original_record
                        .entry()
                        .to_app_option()
                        .map_err(|e| wasm_error!(e))?
                        .ok_or(wasm_error!(WasmErrorInner::Guest(
                            "Original membership entry not found".to_string()
                        )))?;
                    Ok(verify_no_admin_escalation(
                        &original_membership,
                        &new_membership,
                    ))
                }
                EntryTypes::SpaceCapability(_)
                | EntryTypes::SpaceInvitation(_)
                | EntryTypes::ResourceBooking(_)
                | EntryTypes::SpaceSchedule(_) => {
                    let original = must_get_action(original_action_hash)?;
                    Ok(check_author_match(
                        original.action().author(),
                        &action.author,
                        "update",
                    ))
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { action, .. } => {
            let original_action = must_get_action(action.link_add_address.clone())?;
            Ok(check_link_author_match(
                original_action.action().author(),
                &action.author,
            ))
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
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "update",
            ))
        }
        FlatOp::RegisterDelete(OpDelete { action, .. }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "delete",
            ))
        }
    }
}

fn validate_create_space(action: Create, space: Space) -> ExternResult<ValidateCallbackResult> {
    // Bind the space to its committer (P0 author-binding gap).
    if space.creator != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Space creator must be the committing agent (forgery)".to_string(),
        ));
    }
    if space.name.is_empty() || space.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space name must be 1-256 characters".into(),
        ));
    }

    if space.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Description must be at most 4096 characters".into(),
        ));
    }

    if space.id.is_empty() || space.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space ID must be 1-256 characters".into(),
        ));
    }

    if space.requires_approval && space.approval_threshold == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Approval threshold must be > 0 when approval is required".into(),
        ));
    }

    if let SpaceType::Custom(ref name) = space.space_type
        && (name.is_empty() || name.len() > 64)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Custom space type name must be 1-64 characters".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_space(_action: Update, space: Space) -> ExternResult<ValidateCallbackResult> {
    if space.name.is_empty() || space.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space name must be 1-256 characters".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_membership(
    action: Create,
    membership: Membership,
) -> ExternResult<ValidateCallbackResult> {
    if membership.space_id.is_empty() || membership.space_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space ID must be 1-256 characters".into(),
        ));
    }

    // P0 privilege-escalation fix: Admin is the space's highest privilege
    // tier. In the current coordinator, it is granted exactly once -- to
    // the space's own founder, atomically alongside space creation (see
    // create_space). No coordinator flow ever promotes an existing member
    // to Admin. Without this check, any agent could self-commit a
    // Membership entry with role: Admin naming any space_id, becoming a
    // full admin of a space they never created.
    //
    // Deliberately NOT checked here: plain Member/Observer memberships.
    // The real coordinator model lets any active member invite or approve
    // a new plain member (invite_member/approve_invitation) -- the
    // committing agent legitimately differs from both `member` and
    // `invited_by` in the approval-threshold flow (the last approver
    // commits the entry, not the original inviter). Verifying that the
    // committer is a genuine active member at all would need its own
    // authorization-proof mechanism; left as a disclosed, lower-severity
    // gap (unauthorized self-enrollment as a plain member, not privilege
    // escalation) rather than expanding this fix's scope.
    if membership.role == MemberRole::Admin {
        if membership.member != action.author || membership.invited_by != action.author {
            return Ok(ValidateCallbackResult::Invalid(
                "Admin membership must be self-granted by the committing agent".to_string(),
            ));
        }
        let prev = must_get_valid_record(action.prev_action.clone())?;
        let prev_space: Space = prev
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(e))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Admin membership must immediately follow that exact space's own creation"
                    .to_string()
            )))?;
        return Ok(verify_founding_admin(
            &prev_space,
            &membership,
            &action.author,
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Pure comparison logic for the Admin-founding check: given the Space
/// entry that immediately preceded this Membership on the committer's own
/// chain (already fetched via must_get_valid_record), does it actually
/// prove `author` founded exactly this space? Split out from
/// validate_create_membership so this decision logic is unit-testable
/// without a live conductor (must_get_valid_record itself is not
/// resolvable in a bare `#[test]`).
fn verify_founding_admin(
    prev_space: &Space,
    membership: &Membership,
    author: &AgentPubKey,
) -> ValidateCallbackResult {
    if prev_space.id != membership.space_id || prev_space.creator != *author {
        ValidateCallbackResult::Invalid(
            "Admin membership must immediately follow that exact space's own creation, by its creator"
                .to_string(),
        )
    } else {
        ValidateCallbackResult::Valid
    }
}

/// Pure comparison logic for the update-path half of the privilege-
/// escalation fix: a membership update may never change `role` to Admin
/// unless it already was Admin. Split out for the same unit-testability
/// reason as `verify_founding_admin`.
fn verify_no_admin_escalation(original: &Membership, new: &Membership) -> ValidateCallbackResult {
    if new.role == MemberRole::Admin && original.role != MemberRole::Admin {
        ValidateCallbackResult::Invalid(
            "Membership role cannot be escalated to Admin via update".to_string(),
        )
    } else {
        ValidateCallbackResult::Valid
    }
}

fn validate_capability_fields(cap: &SpaceCapability) -> ExternResult<ValidateCallbackResult> {
    if cap.space_id.is_empty() || cap.space_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space ID must be 1-256 characters".into(),
        ));
    }

    if cap.allowed_functions.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Capability must allow at least one function".into(),
        ));
    }

    if cap.allowed_functions.len() > 50 {
        return Ok(ValidateCallbackResult::Invalid(
            "Capability cannot allow more than 50 functions".into(),
        ));
    }

    for func in &cap.allowed_functions {
        if func.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Allowed function name too long (max 256 chars per item)".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_capability(
    action: Create,
    cap: SpaceCapability,
) -> ExternResult<ValidateCallbackResult> {
    // P0 privilege-escalation fix: SpaceCapability previously had no
    // granted_by field at all, and no authorization check whatsoever --
    // any agent could grant themselves arbitrary allowed_functions in any
    // space. Bind granted_by to the committer (prevents misattributing a
    // grant to a different agent), then verify authorized_by actually
    // proves the committer is a currently active Admin of this space.
    if cap.granted_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Capability granted_by must be the committing agent (forgery)".to_string(),
        ));
    }
    let proof = must_get_valid_record(cap.authorized_by.clone())?;
    let admin_membership: Membership = proof
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
        "authorized_by must reference a Membership entry".to_string()
    )))?;
    if admin_membership.space_id != cap.space_id
        || admin_membership.member != cap.granted_by
        || !admin_membership.active
        || admin_membership.role != MemberRole::Admin
    {
        return Ok(ValidateCallbackResult::Invalid(
            "authorized_by does not prove the committing agent is an active admin of this space"
                .to_string(),
        ));
    }

    validate_capability_fields(&cap)
}

fn validate_create_invitation(
    action: Create,
    inv: SpaceInvitation,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the invitation to its committer -- inviter, not invitee, since
    // the invitee is the one being named, not the one acting (P0
    // author-binding gap). Whether the inviter actually has authority to
    // invite (e.g. is a member of the space) is a separate, deferred
    // Class-D question -- see validate_create_membership.
    if inv.inviter != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Invitation inviter must be the committing agent (forgery)".to_string(),
        ));
    }
    if inv.space_id.is_empty() || inv.space_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space ID must be 1-256 characters".into(),
        ));
    }

    if inv.message.len() > 1024 {
        return Ok(ValidateCallbackResult::Invalid(
            "Invitation message must be at most 1024 characters".into(),
        ));
    }

    if inv.status != InvitationStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New invitations must start with Pending status".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_booking(
    action: Create,
    booking: ResourceBooking,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the booking to its committer (P0 author-binding gap).
    if booking.booked_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Booking booked_by must be the committing agent (forgery)".to_string(),
        ));
    }
    validate_booking(booking)
}

fn validate_booking(booking: ResourceBooking) -> ExternResult<ValidateCallbackResult> {
    if booking.id.is_empty() || booking.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Booking ID must be 1-256 characters".into(),
        ));
    }

    if booking.space_id.is_empty() || booking.space_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space ID must be 1-256 characters".into(),
        ));
    }

    if booking.resource_name.is_empty() || booking.resource_name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource name must be 1-256 characters".into(),
        ));
    }

    if booking.end_time <= booking.start_time {
        return Ok(ValidateCallbackResult::Invalid(
            "End time must be after start time".into(),
        ));
    }

    if booking.notes.len() > 1024 {
        return Ok(ValidateCallbackResult::Invalid(
            "Notes must be at most 1024 characters".into(),
        ));
    }

    if booking.status != BookingStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New bookings must start with Pending status".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_schedule(
    action: Create,
    schedule: SpaceSchedule,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the schedule to its committer (P0 author-binding gap).
    if schedule.creator != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule creator must be the committing agent (forgery)".to_string(),
        ));
    }
    validate_schedule(schedule)
}

fn validate_schedule(schedule: SpaceSchedule) -> ExternResult<ValidateCallbackResult> {
    if schedule.id.is_empty() || schedule.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule ID must be 1-256 characters".into(),
        ));
    }

    if schedule.space_id.is_empty() || schedule.space_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Space ID must be 1-256 characters".into(),
        ));
    }

    if schedule.title.is_empty() || schedule.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule title must be 1-256 characters".into(),
        ));
    }

    if schedule.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Description must be at most 4096 characters".into(),
        ));
    }

    if schedule.duration_minutes == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Duration must be greater than 0 minutes".into(),
        ));
    }

    if schedule.duration_minutes > 1440 {
        return Ok(ValidateCallbackResult::Invalid(
            "Duration cannot exceed 24 hours (1440 minutes)".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn space_type_serde_roundtrip() {
        let types = vec![
            SpaceType::Family,
            SpaceType::Squad,
            SpaceType::Cooperative,
            SpaceType::Custom("guild".into()),
        ];
        for t in types {
            let json = serde_json::to_string(&t).unwrap();
            let back: SpaceType = serde_json::from_str(&json).unwrap();
            assert_eq!(t, back);
        }
    }

    #[test]
    fn member_role_serde_roundtrip() {
        let roles = vec![MemberRole::Admin, MemberRole::Member, MemberRole::Observer];
        for r in roles {
            let json = serde_json::to_string(&r).unwrap();
            let back: MemberRole = serde_json::from_str(&json).unwrap();
            assert_eq!(r, back);
        }
    }

    #[test]
    fn invitation_status_serde_roundtrip() {
        let statuses = vec![
            InvitationStatus::Pending,
            InvitationStatus::Approved,
            InvitationStatus::Rejected,
            InvitationStatus::Expired,
        ];
        for s in statuses {
            let json = serde_json::to_string(&s).unwrap();
            let back: InvitationStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(s, back);
        }
    }

    #[test]
    fn booking_status_serde_roundtrip() {
        let statuses = vec![
            BookingStatus::Pending,
            BookingStatus::Confirmed,
            BookingStatus::Cancelled,
        ];
        for s in statuses {
            let json = serde_json::to_string(&s).unwrap();
            let back: BookingStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(s, back);
        }
    }

    #[test]
    fn recurrence_serde_roundtrip() {
        let variants = vec![
            Recurrence::Once,
            Recurrence::Daily,
            Recurrence::Weekly,
            Recurrence::Monthly,
        ];
        for v in variants {
            let json = serde_json::to_string(&v).unwrap();
            let back: Recurrence = serde_json::from_str(&json).unwrap();
            assert_eq!(v, back);
        }
    }

    fn agent_1() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0xaa; 36])
    }

    fn valid_booking() -> ResourceBooking {
        ResourceBooking {
            id: "booking_001".to_string(),
            space_id: "space:test:123".to_string(),
            resource_name: "Meeting Room A".to_string(),
            booked_by: agent_1(),
            start_time: 1000000,
            end_time: 2000000,
            status: BookingStatus::Pending,
            notes: "Team standup".to_string(),
            created_at: Timestamp::from_micros(1000000),
        }
    }

    fn valid_schedule() -> SpaceSchedule {
        SpaceSchedule {
            id: "sched_001".to_string(),
            space_id: "space:test:123".to_string(),
            title: "Weekly Sync".to_string(),
            description: "Weekly team synchronization".to_string(),
            recurrence: Recurrence::Weekly,
            next_occurrence: 1000000,
            duration_minutes: 60,
            creator: agent_1(),
            created_at: Timestamp::from_micros(1000000),
        }
    }

    #[test]
    fn booking_serde_roundtrip() {
        let b = valid_booking();
        let json = serde_json::to_string(&b).unwrap();
        let back: ResourceBooking = serde_json::from_str(&json).unwrap();
        assert_eq!(b.id, back.id);
        assert_eq!(b.resource_name, back.resource_name);
    }

    #[test]
    fn schedule_serde_roundtrip() {
        let s = valid_schedule();
        let json = serde_json::to_string(&s).unwrap();
        let back: SpaceSchedule = serde_json::from_str(&json).unwrap();
        assert_eq!(s.id, back.id);
        assert_eq!(s.recurrence, back.recurrence);
    }

    #[test]
    fn validate_booking_valid() {
        let result = validate_booking(valid_booking());
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn validate_booking_empty_id() {
        let mut b = valid_booking();
        b.id = "".to_string();
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_booking_empty_resource_name() {
        let mut b = valid_booking();
        b.resource_name = "".to_string();
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_booking_end_before_start() {
        let mut b = valid_booking();
        b.end_time = b.start_time;
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_booking_notes_too_long() {
        let mut b = valid_booking();
        b.notes = "x".repeat(1025);
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_booking_wrong_initial_status() {
        let mut b = valid_booking();
        b.status = BookingStatus::Confirmed;
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_schedule_valid() {
        let result = validate_schedule(valid_schedule());
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn validate_schedule_empty_title() {
        let mut s = valid_schedule();
        s.title = "".to_string();
        let result = validate_schedule(s);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_schedule_zero_duration() {
        let mut s = valid_schedule();
        s.duration_minutes = 0;
        let result = validate_schedule(s);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_schedule_excessive_duration() {
        let mut s = valid_schedule();
        s.duration_minutes = 1441;
        let result = validate_schedule(s);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_schedule_max_duration() {
        let mut s = valid_schedule();
        s.duration_minutes = 1440;
        let result = validate_schedule(s);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn validate_schedule_description_too_long() {
        let mut s = valid_schedule();
        s.description = "x".repeat(4097);
        let result = validate_schedule(s);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn validate_booking_notes_at_limit() {
        let mut b = valid_booking();
        b.notes = "x".repeat(1024);
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn validate_booking_id_at_limit() {
        let mut b = valid_booking();
        b.id = "x".repeat(256);
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn validate_booking_id_over_limit() {
        let mut b = valid_booking();
        b.id = "x".repeat(257);
        let result = validate_booking(b);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── String length boundary tests ──────────────────────────────────

    fn fake_create() -> Create {
        Create {
            author: agent_1(),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: EntryRateWeight::default(),
        }
    }

    fn valid_capability() -> SpaceCapability {
        SpaceCapability {
            space_id: "space:test:123".to_string(),
            grantee: agent_1(),
            allowed_functions: vec!["read_data".to_string(), "write_data".to_string()],
            expires_at: None,
            revoked: false,
            granted_at: Timestamp::from_micros(1000000),
            granted_by: agent_1(),
            authorized_by: ActionHash::from_raw_36(vec![9u8; 36]),
        }
    }

    // These test pure field validation via validate_capability_fields
    // directly, bypassing validate_create_capability's authorization proof
    // check (which calls must_get_valid_record -- not resolvable in a bare
    // unit test without a live conductor; see the analogous split in
    // support-diagnostics's integrity crate for the established pattern).

    #[test]
    fn capability_allowed_function_at_limit_accepted() {
        let mut cap = valid_capability();
        cap.allowed_functions = vec!["x".repeat(256)];
        let result = validate_capability_fields(&cap);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn capability_allowed_function_over_limit_rejected() {
        let mut cap = valid_capability();
        cap.allowed_functions = vec!["x".repeat(257)];
        let result = validate_capability_fields(&cap);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn capability_second_function_over_limit_rejected() {
        let mut cap = valid_capability();
        cap.allowed_functions = vec!["valid_func".to_string(), "x".repeat(257)];
        let result = validate_capability_fields(&cap);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── Author-binding tests (P0) ─────────────────────────────────────

    fn agent_2() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0xbb; 36])
    }

    fn valid_space() -> Space {
        Space {
            id: "space:test:123".to_string(),
            name: "Test Space".to_string(),
            space_type: SpaceType::Family,
            description: "A test space".to_string(),
            creator: agent_1(),
            max_members: 0,
            requires_approval: false,
            approval_threshold: 0,
            open: true,
            created_at: Timestamp::from_micros(1000000),
        }
    }

    fn valid_invitation() -> SpaceInvitation {
        SpaceInvitation {
            space_id: "space:test:123".to_string(),
            invitee: agent_2(),
            inviter: agent_1(),
            message: "join us".to_string(),
            approvals: 0,
            approved_by: vec![],
            status: InvitationStatus::Pending,
            created_at: Timestamp::from_micros(1000000),
        }
    }

    #[test]
    fn create_space_valid_creator_accepted() {
        let result = validate_create_space(fake_create(), valid_space());
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn create_space_creator_forgery_rejected() {
        let mut forged = fake_create();
        forged.author = agent_2();
        let result = validate_create_space(forged, valid_space());
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn create_invitation_valid_inviter_accepted() {
        let result = validate_create_invitation(fake_create(), valid_invitation());
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn create_invitation_inviter_forgery_rejected() {
        let mut forged = fake_create();
        forged.author = agent_2();
        let result = validate_create_invitation(forged, valid_invitation());
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn create_booking_valid_booked_by_accepted() {
        let result = validate_create_booking(fake_create(), valid_booking());
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn create_booking_booked_by_forgery_rejected() {
        let mut forged = fake_create();
        forged.author = agent_2();
        let result = validate_create_booking(forged, valid_booking());
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn create_schedule_valid_creator_accepted() {
        let result = validate_create_schedule(fake_create(), valid_schedule());
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn create_schedule_creator_forgery_rejected() {
        let mut forged = fake_create();
        forged.author = agent_2();
        let result = validate_create_schedule(forged, valid_schedule());
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── Privilege-escalation tests (P0) ────────────────────────────────
    // Admin-role membership creation is rejected before ever reaching
    // must_get_valid_record when the mechanical self-grant check fails, so
    // these ARE fully testable at the unit level. The founding-check
    // comparison itself is tested separately via the extracted pure
    // verify_founding_admin/verify_no_admin_escalation helpers, since
    // must_get_valid_record needs a live conductor.

    fn valid_membership(role: MemberRole) -> Membership {
        Membership {
            space_id: "space:test:123".to_string(),
            member: agent_1(),
            role,
            active: true,
            invited_by: agent_1(),
            joined_at: Timestamp::from_micros(1000000),
        }
    }

    #[test]
    fn create_membership_admin_member_mismatch_rejected() {
        let mut m = valid_membership(MemberRole::Admin);
        m.member = agent_2(); // claims a founder identity different from the committer
        let result = validate_create_membership(fake_create(), m);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn create_membership_admin_invited_by_mismatch_rejected() {
        let mut m = valid_membership(MemberRole::Admin);
        m.invited_by = agent_2();
        let result = validate_create_membership(fake_create(), m);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn create_membership_plain_member_unaffected_by_escalation_check() {
        // Matches the real coordinator model: any active member may invite
        // or approve a plain member, and the committer legitimately differs
        // from both `member` and `invited_by` (see the doc comment on
        // validate_create_membership). No must_get_valid_record is reached
        // for non-Admin roles at all.
        let mut m = valid_membership(MemberRole::Member);
        m.member = agent_2();
        m.invited_by = agent_2();
        let result = validate_create_membership(fake_create(), m);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn verify_founding_admin_matching_space_and_creator_accepted() {
        let space = valid_space(); // creator: agent_1(), id: "space:test:123"
        let membership = valid_membership(MemberRole::Admin);
        let result = verify_founding_admin(&space, &membership, &agent_1());
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn verify_founding_admin_wrong_space_id_rejected() {
        let mut space = valid_space();
        space.id = "space:other:456".to_string();
        let membership = valid_membership(MemberRole::Admin);
        let result = verify_founding_admin(&space, &membership, &agent_1());
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn verify_founding_admin_wrong_creator_rejected() {
        let space = valid_space(); // creator: agent_1()
        let membership = valid_membership(MemberRole::Admin);
        // Someone other than the space's real creator claims the founding grant.
        let result = verify_founding_admin(&space, &membership, &agent_2());
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn verify_no_admin_escalation_allows_non_escalating_update() {
        let original = valid_membership(MemberRole::Member);
        let mut new = original.clone();
        new.active = false; // e.g. a legitimate revoke_access deactivation
        assert_eq!(
            verify_no_admin_escalation(&original, &new),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn verify_no_admin_escalation_blocks_member_to_admin() {
        let original = valid_membership(MemberRole::Member);
        let mut new = original.clone();
        new.role = MemberRole::Admin;
        assert!(matches!(
            verify_no_admin_escalation(&original, &new),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn verify_no_admin_escalation_allows_admin_to_admin() {
        // An update that keeps an already-Admin membership Admin (e.g. a
        // field-only change) is not an escalation.
        let original = valid_membership(MemberRole::Admin);
        let new = original.clone();
        assert_eq!(
            verify_no_admin_escalation(&original, &new),
            ValidateCallbackResult::Valid
        );
    }

    // ── SpaceCapability authorization tests (P0) ───────────────────────

    #[test]
    fn create_capability_granted_by_forgery_rejected() {
        let mut forged = fake_create();
        forged.author = agent_2();
        let result = validate_create_capability(forged, valid_capability());
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }
}

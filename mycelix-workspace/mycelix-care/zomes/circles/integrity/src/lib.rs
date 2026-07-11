// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Circles Integrity Zome
//! Defines entry types and validation for care circles and membership.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Type of care circle
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum CircleType {
    Neighborhood,
    Workplace,
    Faith,
    Family,
    School,
    Custom(String),
}

impl CircleType {
    pub fn anchor_key(&self) -> String {
        match self {
            CircleType::Neighborhood => "neighborhood".to_string(),
            CircleType::Workplace => "workplace".to_string(),
            CircleType::Faith => "faith".to_string(),
            CircleType::Family => "family".to_string(),
            CircleType::School => "school".to_string(),
            CircleType::Custom(s) => format!("custom_{}", s.to_lowercase().replace(' ', "_")),
        }
    }
}

/// Role within a care circle
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MemberRole {
    Organizer,
    Member,
    Observer,
}

/// A care circle - a group of people who coordinate mutual aid
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareCircle {
    /// Circle name
    pub name: String,
    /// Description of the circle's purpose
    pub description: String,
    /// Location or area the circle serves
    pub location: String,
    /// Maximum number of members allowed
    pub max_members: u32,
    /// Agent who created the circle
    pub created_by: AgentPubKey,
    /// Type of circle
    pub circle_type: CircleType,
    /// Whether the circle is currently active
    pub active: bool,
    /// When the circle was created
    pub created_at: Timestamp,
}

/// Membership record linking an agent to a circle
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CircleMembership {
    /// Hash of the CareCircle entry
    pub circle_hash: ActionHash,
    /// The member agent
    pub member: AgentPubKey,
    /// Role in the circle
    pub role: MemberRole,
    /// When the member joined
    pub joined_at: Timestamp,
    /// Whether the membership is currently active
    pub active: bool,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    CareCircle(CareCircle),
    CircleMembership(CircleMembership),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All circles anchor
    AllCircles,
    /// Circle type anchor to circles of that type
    TypeToCircle,
    /// Circle to its memberships
    CircleToMembership,
    /// Agent to their memberships
    AgentToMembership,
    /// Agent to circles they created
    AgentToCreatedCircle,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::CareCircle(circle) => validate_create_circle(action, circle),
                EntryTypes::CircleMembership(membership) => {
                    validate_create_membership(action, membership)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action,
            base_address: _,
            target_address: _,
            tag: _,
            action,
        } => {
            // Previously accepted unconditionally regardless of author --
            // the coordinator never calls delete_link here, so this is
            // pure hardening (zero functional impact). Found + fixed
            // 2026-07-09 during the P0 author-binding pass.
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original link creator can delete a link".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 6th confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
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

/// Shared per-entry-type update validation, called from BOTH the
/// StoreEntry (OpEntry::UpdateEntry) and RegisterUpdate DHT-op
/// perspectives so they agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::CareCircle(_) => Ok(ValidateCallbackResult::Invalid(
            "Care circles are immutable".into(),
        )),
        EntryTypes::CircleMembership(membership) => validate_update_membership(action, membership),
    }
}

/// Validate a CircleMembership update (the coordinator's leave_circle
/// deactivates a membership; both creation paths -- create_circle's
/// auto-join and join_circle -- always set `member` to the committing
/// agent themselves, so the original author IS always the member).
/// Content is restricted to the `active` flag flipping true -> false.
fn validate_update_membership(
    action: Update,
    membership: CircleMembership,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: CircleMembership = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original membership not found".into()
        )))?;

    if membership.circle_hash != original.circle_hash
        || membership.member != original.member
        || membership.role != original.role
        || membership.joined_at != original.joined_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the active flag can change on a membership update".into(),
        ));
    }

    if !(original.active && !membership.active) {
        return Ok(ValidateCallbackResult::Invalid(
            "Membership updates may only deactivate an active membership".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_circle(
    action: Create,
    circle: CareCircle,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's create_circle now derives
    // created_by from agent_info() rather than trusting caller input, but
    // that's bypassable by a modified coordinator -- the integrity
    // validator is the real security boundary. Without this, any agent
    // could commit a CareCircle claiming an arbitrary victim as creator
    // (who would then be auto-enrolled as Organizer). Found + fixed
    // 2026-07-09 during the P0 author-binding pass.
    if circle.created_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "created_by must correspond to the committing agent".into(),
        ));
    }

    if circle.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle name cannot be empty".into(),
        ));
    }
    if circle.name.len() > 128 {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle name must be 128 characters or fewer".into(),
        ));
    }
    if circle.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle description cannot be empty".into(),
        ));
    }
    if circle.description.len() > 2048 {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle description must be 2048 characters or fewer".into(),
        ));
    }
    if circle.max_members < 2 {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle must allow at least 2 members".into(),
        ));
    }
    if circle.max_members > 500 {
        return Ok(ValidateCallbackResult::Invalid(
            "Circle cannot have more than 500 members".into(),
        ));
    }
    if circle.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Location cannot be empty".into(),
        ));
    }
    if circle.location.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Location must be 512 characters or fewer".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_membership(
    action: Create,
    membership: CircleMembership,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: both creation paths (create_circle's auto-join of
    // its own creator, and join_circle) always set `member` to the
    // committing agent themselves -- neither ever enrolls a third party.
    // Without this, any agent could commit a CircleMembership claiming an
    // arbitrary victim as member. Found + fixed 2026-07-09 during the P0
    // author-binding pass.
    if membership.member != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Membership member must correspond to the committing agent".into(),
        ));
    }

    // Membership content validation (circle exists, member count, etc.)
    // is otherwise handled at the coordinator level.
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn create_action(author: AgentPubKey) -> Create {
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

    fn valid_circle(created_by: AgentPubKey) -> CareCircle {
        CareCircle {
            name: "Neighbors".into(),
            description: "Mutual aid for the block".into(),
            location: "Block 5".into(),
            max_members: 20,
            created_by,
            circle_type: CircleType::Neighborhood,
            active: true,
            created_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_circle_valid_when_creator_matches_committer() {
        let c = valid_circle(me());
        let result = validate_create_circle(create_action(me()), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_circle_forgery_rejected() {
        let c = valid_circle(me());
        let result = validate_create_circle(create_action(other_agent()), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_membership(member: AgentPubKey) -> CircleMembership {
        CircleMembership {
            circle_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            member,
            role: MemberRole::Member,
            joined_at: Timestamp::from_micros(0),
            active: true,
        }
    }

    #[test]
    fn create_membership_valid_when_member_matches_committer() {
        let m = valid_membership(me());
        let result = validate_create_membership(create_action(me()), m).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_membership_forgery_rejected() {
        let m = valid_membership(me());
        let result = validate_create_membership(create_action(other_agent()), m).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_circle_update() {
        let c = valid_circle(me());
        let action = Update {
            author: me(),
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
        };
        let result = validate_update_entry_type(action, EntryTypes::CareCircle(c)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Communications Integrity Zome
//! Offline-first emergency messaging with store-and-forward

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// An emergency message (offline-first capable)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyMessage {
    pub sender: AgentPubKey,
    pub channel_hash: Option<ActionHash>,
    pub priority: MessagePriority,
    pub content: String,
    pub location: Option<(f64, f64)>,
    pub created_at: Timestamp,
    pub ttl_hours: u8,
    pub hop_count: u8,
    pub synced: bool,
}

/// Message priority levels (NATO-aligned)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MessagePriority {
    Flash,
    Immediate,
    Priority,
    Routine,
}

/// An emergency communication channel
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyChannel {
    pub name: String,
    pub disaster_hash: ActionHash,
    pub channel_type: ChannelType,
    pub participants: Vec<AgentPubKey>,
    pub created_by: AgentPubKey,
}

/// Types of communication channels
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ChannelType {
    Command,
    Operations,
    Logistics,
    Medical,
    Public,
    Volunteer,
}

/// A broadcast message to an area
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Broadcast {
    pub disaster_hash: ActionHash,
    pub broadcast_type: BroadcastType,
    pub content: String,
    pub target_area: (f64, f64, f32),
    pub issued_by: AgentPubKey,
    pub expires_at: Timestamp,
}

/// Types of emergency broadcasts
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum BroadcastType {
    Evacuation,
    ShelterInPlace,
    AllClear,
    ResourceDrop,
    MedicalAlert,
    WeatherWarning,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    EmergencyMessage(EmergencyMessage),
    EmergencyChannel(EmergencyChannel),
    Broadcast(Broadcast),
}

#[hdk_link_types]
pub enum LinkTypes {
    ChannelToMessage,
    DisasterToChannel,
    DisasterToBroadcast,
    AgentToMessage,
    UnsyncedMessages,
    ActiveBroadcasts,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::EmergencyMessage(msg) => validate_create_message(action, msg),
                EntryTypes::EmergencyChannel(channel) => validate_create_channel(action, channel),
                EntryTypes::Broadcast(broadcast) => validate_create_broadcast(action, broadcast),
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
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::ChannelToMessage => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DisasterToChannel => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DisasterToBroadcast => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToMessage => Ok(ValidateCallbackResult::Valid),
            LinkTypes::UnsyncedMessages => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ActiveBroadcasts => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): unlike
        // every other zome fixed this pass, this one has two GENUINE
        // cross-agent link-deletion flows by design --
        // `get_active_broadcasts` deletes an ActiveBroadcasts link on
        // read-triggered cleanup of an expired broadcast (any reader, not
        // the original creator), and `mark_synced` deletes an
        // UnsyncedMessages link when a relay agent (in this offline-first
        // store-and-forward network, not necessarily the sender) marks a
        // message synced. Adding the usual "only original link creator can
        // delete" check here would break both flows.
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 10th confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Unlike RegisterDeleteLink above, no coordinator function
            // calls delete_entry in this zome -- so entry deletion has no
            // known legitimate cross-agent need, and this hardening (only
            // the original entry author may delete it) is pure defense in
            // depth with zero functional impact.
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
        EntryTypes::EmergencyMessage(msg) => validate_update_message(action, msg),
        EntryTypes::EmergencyChannel(_) => Ok(ValidateCallbackResult::Invalid(
            "Emergency channels are immutable".into(),
        )),
        EntryTypes::Broadcast(_) => Ok(ValidateCallbackResult::Invalid(
            "Broadcasts are immutable".into(),
        )),
    }
}

/// Validate an EmergencyMessage update (the coordinator's mark_synced
/// flips `synced` to true).
///
/// No author requirement: this is an offline-first store-and-forward
/// network, and a message may legitimately be marked synced by a
/// relaying agent, not just the original sender. Content is restricted to
/// the `synced` flag flipping false -> true; everything else is
/// immutable.
fn validate_update_message(
    action: Update,
    msg: EmergencyMessage,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: EmergencyMessage = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original message not found".into()
        )))?;

    if msg.sender != original.sender
        || msg.channel_hash != original.channel_hash
        || msg.priority != original.priority
        || msg.content != original.content
        || msg.location != original.location
        || msg.created_at != original.created_at
        || msg.ttl_hours != original.ttl_hours
        || msg.hop_count != original.hop_count
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only synced can change on a message update".into(),
        ));
    }

    if !(!original.synced && msg.synced) {
        return Ok(ValidateCallbackResult::Invalid(
            "Message updates may only flip synced from false to true".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_message(
    action: Create,
    msg: EmergencyMessage,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's send_message already derives
    // sender from agent_info(), so this is belt-and-suspenders against a
    // modified coordinator forging a victim agent as sender. Found + fixed
    // 2026-07-09 during the P0 author-binding pass.
    if msg.sender != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Message sender must correspond to the committing agent".into(),
        ));
    }

    if msg.content.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Message content cannot be empty".into(),
        ));
    }
    if msg.content.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Message content cannot exceed 4096 bytes".into(),
        ));
    }
    if msg.ttl_hours == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "TTL must be at least 1 hour".into(),
        ));
    }
    if let Some((lat, lon)) = msg.location {
        if lat < -90.0 || lat > 90.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Latitude must be between -90 and 90".into(),
            ));
        }
        if lon < -180.0 || lon > 180.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Longitude must be between -180 and 180".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_channel(
    action: Create,
    channel: EmergencyChannel,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's create_channel already derives
    // created_by from agent_info(), so this is belt-and-suspenders against
    // a modified coordinator forging a victim agent as creator. Found +
    // fixed 2026-07-09 during the P0 author-binding pass.
    if channel.created_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Channel created_by must correspond to the committing agent".into(),
        ));
    }

    if channel.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Channel name cannot be empty".into(),
        ));
    }
    if channel.participants.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Channel must have at least one participant".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_broadcast(
    action: Create,
    broadcast: Broadcast,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's broadcast already derives
    // issued_by from agent_info(), so this is belt-and-suspenders against
    // a modified coordinator forging a victim agent as issuer. Found +
    // fixed 2026-07-09 during the P0 author-binding pass.
    if broadcast.issued_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Broadcast issued_by must correspond to the committing agent".into(),
        ));
    }

    if broadcast.content.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Broadcast content cannot be empty".into(),
        ));
    }
    let (lat, lon, radius) = broadcast.target_area;
    if lat < -90.0 || lat > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Target area latitude must be between -90 and 90".into(),
        ));
    }
    if lon < -180.0 || lon > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Target area longitude must be between -180 and 180".into(),
        ));
    }
    if radius <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Target area radius must be positive".into(),
        ));
    }
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

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_message(sender: AgentPubKey) -> EmergencyMessage {
        EmergencyMessage {
            sender,
            channel_hash: None,
            priority: MessagePriority::Routine,
            content: "status update".into(),
            location: None,
            created_at: Timestamp::from_micros(0),
            ttl_hours: 24,
            hop_count: 0,
            synced: false,
        }
    }

    #[test]
    fn create_message_valid_when_sender_matches_committer() {
        let m = valid_message(me());
        let result = validate_create_message(create_action(me()), m).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_message_forgery_rejected() {
        let m = valid_message(me());
        let result = validate_create_message(create_action(other_agent()), m).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_channel(created_by: AgentPubKey) -> EmergencyChannel {
        EmergencyChannel {
            name: "Command".into(),
            disaster_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            channel_type: ChannelType::Command,
            participants: vec![created_by.clone()],
            created_by,
        }
    }

    #[test]
    fn create_channel_valid_when_creator_matches_committer() {
        let c = valid_channel(me());
        let result = validate_create_channel(create_action(me()), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_channel_forgery_rejected() {
        let c = valid_channel(me());
        let result = validate_create_channel(create_action(other_agent()), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_broadcast(issued_by: AgentPubKey) -> Broadcast {
        Broadcast {
            disaster_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            broadcast_type: BroadcastType::AllClear,
            content: "All clear".into(),
            target_area: (0.0, 0.0, 5.0),
            issued_by,
            expires_at: Timestamp::from_micros(1_000_000),
        }
    }

    #[test]
    fn create_broadcast_valid_when_issuer_matches_committer() {
        let b = valid_broadcast(me());
        let result = validate_create_broadcast(create_action(me()), b).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_broadcast_forgery_rejected() {
        let b = valid_broadcast(me());
        let result = validate_create_broadcast(create_action(other_agent()), b).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_channel_update() {
        let c = valid_channel(me());
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::EmergencyChannel(c))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_broadcast_update() {
        let b = valid_broadcast(me());
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::Broadcast(b)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

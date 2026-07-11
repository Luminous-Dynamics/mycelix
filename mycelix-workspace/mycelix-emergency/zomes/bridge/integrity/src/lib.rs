// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Emergency Bridge Integrity Zome
//! Cross-hApp integration types for emergency coordination

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// A cross-hApp query for emergency data
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyQuery {
    pub query_id: String,
    pub source_happ: String,
    pub query_type: QueryType,
    pub parameters: String,
    pub requested_by: AgentPubKey,
    pub requested_at: Timestamp,
    pub response: Option<String>,
    pub responded_at: Option<Timestamp>,
}

/// Types of cross-hApp queries
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum QueryType {
    ActiveDisasters,
    ResourceAvailability,
    ShelterCapacity,
    PersonnelStatus,
    HealthRecords,
    IdentityVerification,
    SupplyChainStatus,
}

/// An event broadcast to other hApps
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyEvent {
    pub event_id: String,
    pub event_type: EventType,
    pub disaster_hash: Option<ActionHash>,
    pub payload: String,
    pub emitted_by: AgentPubKey,
    pub emitted_at: Timestamp,
    pub target_happs: Vec<String>,
}

/// Types of emergency events
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EventType {
    DisasterDeclared,
    DisasterClosed,
    EvacuationOrdered,
    ResourceRequested,
    MassCasualtyDeclared,
    ShelterOpened,
    ShelterFull,
    AllClear,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    EmergencyQuery(EmergencyQuery),
    EmergencyEvent(EmergencyEvent),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllQueries,
    AllEvents,
    DisasterToEvent,
    HappToQuery,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::EmergencyQuery(query) => validate_create_query(action, query),
                EntryTypes::EmergencyEvent(event) => validate_create_event(action, event),
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
            LinkTypes::AllQueries => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllEvents => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DisasterToEvent => Ok(ValidateCallbackResult::Valid),
            LinkTypes::HappToQuery => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            ..
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
            // unconditionally) -- the 9th confirmed instance of this exact
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
    _action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        // Both types are immutable: confirmed via grep that no coordinator
        // function ever calls update_entry for either. Found + fixed
        // 2026-07-09 during the P0 author-binding pass -- EmergencyQuery
        // previously accepted any field change unconditionally.
        EntryTypes::EmergencyQuery(_) => Ok(ValidateCallbackResult::Invalid(
            "Emergency queries are immutable".into(),
        )),
        EntryTypes::EmergencyEvent(_) => Ok(ValidateCallbackResult::Invalid(
            "Emergency events are immutable".into(),
        )),
    }
}

fn validate_create_query(
    action: Create,
    query: EmergencyQuery,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's query_emergency already derives
    // requested_by from agent_info(), so this is belt-and-suspenders
    // against a modified coordinator forging a victim agent as requester.
    // Found + fixed 2026-07-09 during the P0 author-binding pass.
    if query.requested_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Query requested_by must correspond to the committing agent".into(),
        ));
    }

    if query.query_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Query ID cannot be empty".into(),
        ));
    }
    if query.source_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Source hApp cannot be empty".into(),
        ));
    }
    if query.parameters.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Query parameters cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_event(
    action: Create,
    event: EmergencyEvent,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's broadcast_event already derives
    // emitted_by from agent_info(), so this is belt-and-suspenders against
    // a modified coordinator forging a victim agent as emitter. Found +
    // fixed 2026-07-09 during the P0 author-binding pass.
    if event.emitted_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Event emitted_by must correspond to the committing agent".into(),
        ));
    }

    if event.event_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Event ID cannot be empty".into(),
        ));
    }
    if event.payload.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Event payload cannot be empty".into(),
        ));
    }
    if event.target_happs.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Event must target at least one hApp".into(),
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

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_query(requested_by: AgentPubKey) -> EmergencyQuery {
        EmergencyQuery {
            query_id: "q-1".into(),
            source_happ: "mycelix-health".into(),
            query_type: QueryType::ResourceAvailability,
            parameters: "{}".into(),
            requested_by,
            requested_at: Timestamp::from_micros(0),
            response: None,
            responded_at: None,
        }
    }

    #[test]
    fn create_query_valid_when_requester_matches_committer() {
        let q = valid_query(me());
        let result = validate_create_query(create_action(me()), q).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_query_forgery_rejected() {
        let q = valid_query(me());
        let result = validate_create_query(create_action(other_agent()), q).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_event(emitted_by: AgentPubKey) -> EmergencyEvent {
        EmergencyEvent {
            event_id: "e-1".into(),
            event_type: EventType::AllClear,
            disaster_hash: None,
            payload: "{}".into(),
            emitted_by,
            emitted_at: Timestamp::from_micros(0),
            target_happs: vec!["mycelix-health".into()],
        }
    }

    #[test]
    fn create_event_valid_when_emitter_matches_committer() {
        let e = valid_event(me());
        let result = validate_create_event(create_action(me()), e).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_event_forgery_rejected() {
        let e = valid_event(me());
        let result = validate_create_event(create_action(other_agent()), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_query_update() {
        let q = valid_query(me());
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
        let result = validate_update_entry_type(action, EntryTypes::EmergencyQuery(q)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Housing Bridge Integrity Zome
//! Entry types for cross-hApp queries and events.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Type of housing query for cross-hApp integration
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum HousingQueryType {
    MemberVerification,
    UnitAvailability,
    EquityStatus,
    MaintenanceHistory,
    AffordabilityCheck,
    LeaseStatus,
    GovernanceStatus,
}

/// A cross-hApp query about housing
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct HousingQuery {
    pub query_type: HousingQueryType,
    pub requester: AgentPubKey,
    pub target_agent: Option<AgentPubKey>,
    pub target_hash: Option<ActionHash>,
    pub parameters: String,
    pub response: Option<String>,
    pub queried_at: Timestamp,
    pub responded_at: Option<Timestamp>,
}

/// Type of housing event for broadcasting
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum HousingEventType {
    MemberJoined,
    MemberLeft,
    UnitAvailable,
    UnitOccupied,
    MaintenanceEmergency,
    ResolutionPassed,
    ElectionCompleted,
    LeaseTransferred,
    EquityMilestone,
    AffordabilityReportPublished,
}

/// A housing event broadcast to other hApps
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct HousingEvent {
    pub event_type: HousingEventType,
    pub source_agent: AgentPubKey,
    pub data: String,
    pub occurred_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    HousingQuery(HousingQuery),
    HousingEvent(HousingEvent),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All queries anchor
    AllQueries,
    /// All events anchor
    AllEvents,
    /// Event type index
    EventTypeToEvent,
    /// Agent to their queries
    AgentToQuery,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::HousingQuery(query) => validate_create_query(action, query),
                EntryTypes::HousingEvent(event) => validate_create_event(action, event),
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
            LinkTypes::EventTypeToEvent => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToQuery => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): the
        // coordinator never calls delete_link here, so this is pure
        // hardening territory but with no live flow either way -- kept
        // as-is since there's nothing to harden against (no delete_link
        // call exists at all).
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
            // unconditionally) -- the 20th confirmed instance of this exact
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
        EntryTypes::HousingQuery(query) => validate_update_query(action, query),
        EntryTypes::HousingEvent(_) => Ok(ValidateCallbackResult::Invalid(
            "Events are immutable".into(),
        )),
    }
}

/// No author requirement: the coordinator's respond_to_query has zero
/// caller-identity check (cross-hApp responder may legitimately be a
/// different agent than the requester). Content is restricted to
/// response/responded_at -- this closes the wide-open bug that
/// previously accepted ANY field change unconditionally, including
/// requester/target_agent/parameters.
fn validate_update_query(
    action: Update,
    query: HousingQuery,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: HousingQuery = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original query not found".into()
        )))?;

    if query.query_type != original.query_type
        || query.requester != original.requester
        || query.target_agent != original.target_agent
        || query.target_hash != original.target_hash
        || query.parameters != original.parameters
        || query.queried_at != original.queried_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only response/responded_at can change on a query update".into(),
        ));
    }

    if query.response.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "Update must set a response".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_query(
    action: Create,
    query: HousingQuery,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's query_housing already derives
    // requester from agent_info(), so this is belt-and-suspenders
    // against a modified coordinator forging a victim agent as
    // requester. Found + fixed 2026-07-09 during the P0 author-binding
    // pass.
    if query.requester != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Query requester must correspond to the committing agent".into(),
        ));
    }

    if query.parameters.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Query parameters must be at most 4096 characters".into(),
        ));
    }
    if query.response.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New queries should not have a response".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_event(
    action: Create,
    event: HousingEvent,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's broadcast_event already derives
    // source_agent from agent_info(), so this is belt-and-suspenders
    // against a modified coordinator forging a victim agent as emitter.
    // Found + fixed 2026-07-09 during the P0 author-binding pass.
    if event.source_agent != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Event source_agent must correspond to the committing agent".into(),
        ));
    }

    if event.data.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Event data cannot be empty".into(),
        ));
    }
    if event.data.len() > 8192 {
        return Ok(ValidateCallbackResult::Invalid(
            "Event data must be at most 8192 characters".into(),
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

    fn valid_query(requester: AgentPubKey) -> HousingQuery {
        HousingQuery {
            query_type: HousingQueryType::UnitAvailability,
            requester,
            target_agent: None,
            target_hash: None,
            parameters: "{}".into(),
            response: None,
            queried_at: Timestamp::from_micros(0),
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

    fn valid_event(source_agent: AgentPubKey) -> HousingEvent {
        HousingEvent {
            event_type: HousingEventType::MemberJoined,
            source_agent,
            data: "{}".into(),
            occurred_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_event_valid_when_source_matches_committer() {
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
    fn update_entry_type_rejects_event_update() {
        let e = valid_event(me());
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
        let result = validate_update_entry_type(action, EntryTypes::HousingEvent(e)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Care Bridge Integrity Zome
//! Defines entry types for cross-hApp integration queries and events.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Type of cross-hApp care query
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum CareQueryType {
    /// Check if an agent has care credentials
    CredentialCheck,
    /// Query available care providers in an area
    ProviderSearch,
    /// Request care plan coordination across hApps
    PlanCoordination,
    /// Query health-related needs from the health hApp
    HealthNeeds,
    /// Verify identity through the identity hApp
    IdentityVerification,
    /// Check governance standing
    GovernanceStanding,
}

/// A cross-hApp query record for auditability
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareQuery {
    /// Type of query
    pub query_type: CareQueryType,
    /// The agent initiating the query
    pub requester: AgentPubKey,
    /// Target hApp or zome
    pub target_happ: String,
    /// Query parameters (JSON)
    pub parameters: String,
    /// Query result (JSON, filled after response)
    pub result: Option<String>,
    /// When the query was made
    pub created_at: Timestamp,
    /// When the result was received
    pub resolved_at: Option<Timestamp>,
    /// Whether the query succeeded
    pub success: Option<bool>,
}

/// Type of care event broadcast
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum CareEventType {
    /// A new care plan was created
    PlanCreated,
    /// A care plan was updated
    PlanUpdated,
    /// A care session was logged
    SessionLogged,
    /// A new provider joined the network
    ProviderJoined,
    /// A credential was verified
    CredentialVerified,
    /// An urgent care request was posted
    UrgentRequest,
    /// A care circle was formed
    CircleFormed,
    /// A time exchange was completed
    ExchangeCompleted,
}

/// A care event broadcast to other hApps
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareEvent {
    /// Type of event
    pub event_type: CareEventType,
    /// Agent that triggered the event
    pub source_agent: AgentPubKey,
    /// Event payload (JSON)
    pub payload: String,
    /// When the event occurred
    pub created_at: Timestamp,
    /// Related entry hashes for cross-referencing
    pub related_hashes: Vec<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    CareQuery(CareQuery),
    CareEvent(CareEvent),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All queries anchor
    AllQueries,
    /// Agent to their queries
    AgentToQuery,
    /// All events anchor
    AllEvents,
    /// Event type to events
    EventTypeToEvent,
    /// Agent to events they triggered
    AgentToEvent,
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
                EntryTypes::CareQuery(query) => validate_create_query(action, query),
                EntryTypes::CareEvent(event) => validate_create_event(action, event),
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
            // unconditionally) -- meaning any agent could rewrite the
            // content of ANY entry in this zome via a modified coordinator.
            // Found + fixed 2026-07-09 during the P0 author-binding pass
            // (same class of gap already found in mycelix-knowledge's
            // bridge/inference/invention zomes). Route through the same
            // per-type validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry in this zome, so this is pure hardening,
            // zero functional impact.
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
        EntryTypes::CareQuery(query) => validate_update_query(action, query),
        EntryTypes::CareEvent(_) => Ok(ValidateCallbackResult::Invalid(
            "Care events are immutable".into(),
        )),
    }
}

/// Validate a CareQuery update (the coordinator's resolve_query fills in
/// result/resolved_at/success).
///
/// No author requirement: this is a cross-hApp query record, and the
/// responding system (potentially a different agent/hApp than the
/// original requester) is expected to resolve it -- so this enforces
/// content integrity instead: query_type/requester/target_happ/
/// parameters/created_at are immutable, and result/resolved_at/success
/// must be set together, only once (Pending -> Resolved, no re-resolving).
fn validate_update_query(action: Update, query: CareQuery) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: CareQuery = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original care query not found".into()
        )))?;

    if query.query_type != original.query_type
        || query.requester != original.requester
        || query.target_happ != original.target_happ
        || query.parameters != original.parameters
        || query.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only result/resolved_at/success can change on a care query update".into(),
        ));
    }

    if original.resolved_at.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Care query has already been resolved".into(),
        ));
    }

    if query.result.is_none() || query.resolved_at.is_none() || query.success.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resolving a care query requires result, resolved_at, and success together".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_query(action: Create, query: CareQuery) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's query_care now derives `requester`
    // from agent_info() rather than trusting caller input, but that's
    // bypassable by a modified coordinator -- the integrity validator is
    // the real security boundary. Without this, any agent could commit a
    // CareQuery claiming an arbitrary victim agent as requester. Found +
    // fixed 2026-07-09 during the P0 author-binding pass.
    if query.requester != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Care query requester must correspond to the committing agent".into(),
        ));
    }

    // A freshly created query cannot already claim a resolution
    if query.result.is_some() || query.resolved_at.is_some() || query.success.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New care queries cannot have resolution fields set".into(),
        ));
    }

    if query.target_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Target hApp cannot be empty".into(),
        ));
    }
    if query.target_happ.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Target hApp must be 256 characters or fewer".into(),
        ));
    }
    if query.parameters.len() > 8192 {
        return Ok(ValidateCallbackResult::Invalid(
            "Parameters must be 8192 characters or fewer".into(),
        ));
    }
    if !query.parameters.is_empty() {
        if serde_json::from_str::<serde_json::Value>(&query.parameters).is_err() {
            return Ok(ValidateCallbackResult::Invalid(
                "Parameters must be valid JSON".into(),
            ));
        }
    }
    if let Some(ref result) = query.result {
        if result.len() > 8192 {
            return Ok(ValidateCallbackResult::Invalid(
                "Result must be 8192 characters or fewer".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_event(action: Create, event: CareEvent) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's broadcast_event now derives
    // source_agent from agent_info() rather than trusting caller input, so
    // this is belt-and-suspenders against a modified coordinator forging a
    // victim agent as the event source. Found + fixed 2026-07-09 during
    // the P0 author-binding pass.
    if event.source_agent != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Care event source_agent must correspond to the committing agent".into(),
        ));
    }

    if event.payload.len() > 8192 {
        return Ok(ValidateCallbackResult::Invalid(
            "Payload must be 8192 characters or fewer".into(),
        ));
    }
    if !event.payload.is_empty() {
        if serde_json::from_str::<serde_json::Value>(&event.payload).is_err() {
            return Ok(ValidateCallbackResult::Invalid(
                "Payload must be valid JSON".into(),
            ));
        }
    }
    if event.related_hashes.len() > 20 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 20 related hashes".into(),
        ));
    }
    for hash in &event.related_hashes {
        if hash.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Each related hash must be 256 characters or fewer".into(),
            ));
        }
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

    fn valid_query(requester: AgentPubKey) -> CareQuery {
        CareQuery {
            query_type: CareQueryType::CredentialCheck,
            requester,
            target_happ: "mycelix-identity".into(),
            parameters: "{}".into(),
            result: None,
            created_at: Timestamp::from_micros(0),
            resolved_at: None,
            success: None,
        }
    }

    #[test]
    fn create_query_valid_when_requester_matches_committer() {
        let q = valid_query(me());
        let result = validate_create_query(test_action(me()), q).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_query_requester_forgery_rejected() {
        let q = valid_query(me());
        let result = validate_create_query(test_action(other_agent()), q).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_query_rejects_preset_resolution_fields() {
        let mut q = valid_query(me());
        q.result = Some("{}".into());
        let result = validate_create_query(test_action(me()), q).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_event(source_agent: AgentPubKey) -> CareEvent {
        CareEvent {
            event_type: CareEventType::PlanCreated,
            source_agent,
            payload: "{}".into(),
            created_at: Timestamp::from_micros(0),
            related_hashes: vec![],
        }
    }

    #[test]
    fn create_event_valid_when_source_matches_committer() {
        let e = valid_event(me());
        let result = validate_create_event(test_action(me()), e).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_event_source_forgery_rejected() {
        let e = valid_event(me());
        let result = validate_create_event(test_action(other_agent()), e).unwrap();
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
        let result = validate_update_entry_type(action, EntryTypes::CareEvent(e)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

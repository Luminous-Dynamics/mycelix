// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Water Bridge Integrity Zome
//! Cross-hApp integration types for water queries and events

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

// ============================================================================
// CROSS-HAPP QUERY & EVENT TYPES
// ============================================================================

/// Query type for cross-hApp water data requests
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum WaterQueryType {
    /// Query source status and capacity
    SourceStatus,
    /// Query quality readings for a source
    QualityReadings,
    /// Query active alerts
    ActiveAlerts,
    /// Query water rights for an agent
    AgentRights,
    /// Query watershed governance info
    WatershedInfo,
    /// Query credit balance for an agent
    CreditBalance,
    /// Query harvest statistics
    HarvestStats,
}

/// A cross-hApp query for water data
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WaterQuery {
    /// Type of query
    pub query_type: WaterQueryType,
    /// Requesting agent
    pub requester: AgentPubKey,
    /// Serialized query parameters (JSON)
    pub params: String,
    /// When the query was made
    pub requested_at: Timestamp,
    /// Serialized response (JSON), populated when answered
    pub response: Option<String>,
    /// Whether the query has been answered
    pub answered: bool,
}

/// Type of water event for cross-hApp broadcasting
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum WaterEventType {
    /// New source registered
    SourceRegistered,
    /// Source status changed
    SourceStatusChanged,
    /// Contamination alert raised
    AlertRaised,
    /// Alert resolved
    AlertResolved,
    /// Water right registered
    RightRegistered,
    /// Water right transferred
    RightTransferred,
    /// Dispute filed
    DisputeFiled,
    /// Dispute resolved
    DisputeResolved,
    /// Quality reading submitted
    QualityReadingSubmitted,
    /// Harvest recorded
    HarvestRecorded,
    /// Credit transfer completed
    CreditTransferred,
}

/// A water event broadcast to other hApps
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WaterEvent {
    /// Type of event
    pub event_type: WaterEventType,
    /// Serialized event payload (JSON)
    pub payload: String,
    /// Agent that triggered the event
    pub triggered_by: AgentPubKey,
    /// When the event occurred
    pub occurred_at: Timestamp,
    /// Optional reference to the source action hash
    pub reference_hash: Option<ActionHash>,
}

// ============================================================================
// ENTRY & LINK TYPE REGISTRATION
// ============================================================================

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    WaterQuery(WaterQuery),
    WaterEvent(WaterEvent),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All water queries anchor
    AllQueries,
    /// Requester to their queries
    RequesterToQuery,
    /// All water events anchor
    AllEvents,
    /// Event type to events
    EventTypeToEvent,
    /// Recent events anchor (for polling)
    RecentEvents,
}

// ============================================================================
// VALIDATION
// ============================================================================

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::WaterQuery(query) => validate_create_query(action, query),
                EntryTypes::WaterEvent(event) => validate_create_event(action, event),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action: _,
                original_action_hash: _,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::WaterQuery(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::WaterEvent(_) => Ok(ValidateCallbackResult::Valid),
            },
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
            LinkTypes::RequesterToQuery => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllEvents => Ok(ValidateCallbackResult::Valid),
            LinkTypes::EventTypeToEvent => Ok(ValidateCallbackResult::Valid),
            LinkTypes::RecentEvents => Ok(ValidateCallbackResult::Valid),
        },
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
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_query(
    action: Create,
    query: WaterQuery,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the query to its committer -- query_water/check_property_rights/
    // verify_steward_identity already derive `requester` from agent_info()
    // coordinator-side with zero user input, so this never rejects a
    // legitimate query; it's the real DHT-level enforcement a modified
    // coordinator could otherwise bypass (P0 author-binding gap).
    if query.requester != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Query requester must be the committing agent (forgery)".to_string(),
        ));
    }

    if query.params.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Query params cannot be empty".into(),
        ));
    }
    // Validate params is valid JSON
    if serde_json::from_str::<serde_json::Value>(&query.params).is_err() {
        return Ok(ValidateCallbackResult::Invalid(
            "Query params must be valid JSON".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_event(
    action: Create,
    event: WaterEvent,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the event to its committer -- broadcast_event already derives
    // `triggered_by` from agent_info() coordinator-side with zero user
    // input, same rationale as validate_create_query above.
    if event.triggered_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Event triggered_by must be the committing agent (forgery)".to_string(),
        ));
    }

    if event.payload.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Event payload cannot be empty".into(),
        ));
    }
    // Validate payload is valid JSON
    if serde_json::from_str::<serde_json::Value>(&event.payload).is_err() {
        return Ok(ValidateCallbackResult::Invalid(
            "Event payload must be valid JSON".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
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

    #[test]
    fn test_create_query_valid() {
        let query = WaterQuery {
            query_type: WaterQueryType::SourceStatus,
            requester: test_action().author,
            params: "{}".to_string(),
            requested_at: Timestamp::from_micros(0),
            response: None,
            answered: false,
        };
        let result = validate_create_query(test_action(), query).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_query_requester_forgery_rejected() {
        // requester claims test_action()'s author, but a different agent
        // committed the action.
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let query = WaterQuery {
            query_type: WaterQueryType::SourceStatus,
            requester: test_action().author,
            params: "{}".to_string(),
            requested_at: Timestamp::from_micros(0),
            response: None,
            answered: false,
        };
        let result = validate_create_query(forged_action, query).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_query_invalid_json_rejected() {
        let query = WaterQuery {
            query_type: WaterQueryType::SourceStatus,
            requester: test_action().author,
            params: "not json".to_string(),
            requested_at: Timestamp::from_micros(0),
            response: None,
            answered: false,
        };
        let result = validate_create_query(test_action(), query).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_event_valid() {
        let event = WaterEvent {
            event_type: WaterEventType::SourceRegistered,
            payload: "{}".to_string(),
            triggered_by: test_action().author,
            occurred_at: Timestamp::from_micros(0),
            reference_hash: None,
        };
        let result = validate_create_event(test_action(), event).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_event_triggered_by_forgery_rejected() {
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let event = WaterEvent {
            event_type: WaterEventType::SourceRegistered,
            payload: "{}".to_string(),
            triggered_by: test_action().author,
            occurred_at: Timestamp::from_micros(0),
            reference_hash: None,
        };
        let result = validate_create_event(forged_action, event).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

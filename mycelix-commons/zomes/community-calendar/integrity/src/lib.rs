// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Community Calendar Integrity Zome
//! Defines entry types and validation for calendar events, RSVPs, and anchors.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_author_match, check_link_author_match};

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Recurrence pattern for calendar events
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum Recurrence {
    None,
    Daily,
    Weekly,
    Monthly,
}

/// A community calendar event
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CalendarEvent {
    /// Event title
    pub title: String,
    /// Detailed description
    pub description: String,
    /// Physical location (optional)
    pub location: Option<commons_types::geo::GeoLocation>,
    /// Start time (UTC)
    pub start_time: Timestamp,
    /// End time (UTC)
    pub end_time: Timestamp,
    /// Recurrence pattern
    pub recurrence: Recurrence,
    /// DID of the organizer
    pub organizer_did: String,
    /// Event category (e.g., "meeting", "workshop", "celebration")
    pub category: String,
    /// Maximum number of attendees (0 = unlimited)
    pub max_attendees: u32,
    /// Whether RSVP is required to attend
    pub rsvp_required: bool,
}

/// RSVP status for an event
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RsvpStatus {
    Going,
    Maybe,
    NotGoing,
}

/// An RSVP response to a calendar event
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Rsvp {
    /// ActionHash of the CalendarEvent
    pub event_id: ActionHash,
    /// DID of the attendee
    pub attendee_did: String,
    /// RSVP status
    pub status: RsvpStatus,
    /// When the RSVP was submitted
    pub responded_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    CalendarEvent(CalendarEvent),
    Rsvp(Rsvp),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor("all_events") → CalendarEvent
    AllEvents,
    /// Anchor("events_date:<YYYY-MM-DD>") → CalendarEvent
    EventsByDate,
    /// Anchor("events_cat:<category>") → CalendarEvent
    EventsByCategory,
    /// CalendarEvent → Rsvp
    EventToRsvp,
    /// Anchor("agent_rsvps:<agent>") → Rsvp
    AgentToRsvp,
    /// Anchor("agent_events:<agent>") → CalendarEvent (organized)
    AgentToEvent,
    /// Anchor("geo:<geohash>") → CalendarEvent
    GeoIndex,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

// ============================================================================
// Validation
// ============================================================================

fn validate_create_event(
    _action: Create,
    event: CalendarEvent,
) -> ExternResult<ValidateCallbackResult> {
    // Title must be non-empty and bounded
    if event.title.trim().is_empty() || event.title.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Event title must be 1-512 non-whitespace characters".into(),
        ));
    }
    // Description bounded
    if event.description.len() > 8192 {
        return Ok(ValidateCallbackResult::Invalid(
            "Event description exceeds 8192 bytes".into(),
        ));
    }
    // Category bounded
    if event.category.trim().is_empty() || event.category.len() > 128 {
        return Ok(ValidateCallbackResult::Invalid(
            "Event category must be 1-128 non-whitespace characters".into(),
        ));
    }
    // Organizer DID bounded
    if event.organizer_did.trim().is_empty() || event.organizer_did.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "organizer_did must be 1-256 non-whitespace characters".into(),
        ));
    }
    // End time must be after start time
    if event.end_time <= event.start_time {
        return Ok(ValidateCallbackResult::Invalid(
            "end_time must be after start_time".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_rsvp(_action: Create, rsvp: Rsvp) -> ExternResult<ValidateCallbackResult> {
    if rsvp.attendee_did.trim().is_empty() || rsvp.attendee_did.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "attendee_did must be 1-256 non-whitespace characters".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::CalendarEvent(event) => validate_create_event(action, event),
                EntryTypes::Rsvp(rsvp) => validate_create_rsvp(action, rsvp),
            },
            // Previously every arm here returned an unconditional Valid, and
            // there were no RegisterUpdate/RegisterDelete/RegisterDeleteLink
            // arms below either (everything fell to the trailing catch-all)
            // -- any agent could rewrite or delete anyone else's calendar
            // events and RSVPs with zero authorization check at all (P0
            // author-binding gap, the wide-open-update/delete class,
            // confirmed 48+ times project-wide per
            // memory/mycelix_attribution_author_binding_jul8.md).
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::CalendarEvent(_) | EntryTypes::Rsvp(_) => {
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
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag,
            action: _,
        } => {
            // Uniform tag length check for all link types
            let max_tag = match link_type {
                LinkTypes::EventsByCategory => 512,
                _ => 256,
            };
            if tag.0.len() > max_tag {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Link tag too long (max {} bytes)",
                    max_tag
                )));
            }
            Ok(ValidateCallbackResult::Valid)
        }
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
            let original_action = must_get_action(action.deletes_address.clone())?;
            Ok(check_author_match(
                original_action.action().author(),
                &action.author,
                "delete",
            ))
        }
    }
}

// ============================================================================
// UNIT TESTS
// ============================================================================

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

    fn make_valid_event() -> CalendarEvent {
        CalendarEvent {
            title: "Community Potluck".to_string(),
            description: "Bring a dish to share".to_string(),
            location: None,
            start_time: Timestamp::from_micros(1_711_100_000_000_000),
            end_time: Timestamp::from_micros(1_711_110_000_000_000),
            recurrence: Recurrence::None,
            organizer_did: "did:example:organizer".to_string(),
            category: "celebration".to_string(),
            max_attendees: 0,
            rsvp_required: false,
        }
    }

    fn make_valid_rsvp() -> Rsvp {
        Rsvp {
            event_id: ActionHash::from_raw_36(vec![1u8; 36]),
            attendee_did: "did:example:attendee".to_string(),
            status: RsvpStatus::Going,
            responded_at: Timestamp::from_micros(1_711_050_000_000_000),
        }
    }

    #[test]
    fn test_create_event_valid() {
        let event = make_valid_event();
        let result = validate_create_event(test_action(), event).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_event_end_before_start_rejected() {
        let mut event = make_valid_event();
        event.end_time = event.start_time;
        let result = validate_create_event(test_action(), event).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_rsvp_valid() {
        let rsvp = make_valid_rsvp();
        let result = validate_create_rsvp(test_action(), rsvp).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_rsvp_empty_attendee_did_rejected() {
        let mut rsvp = make_valid_rsvp();
        rsvp.attendee_did = String::new();
        let result = validate_create_rsvp(test_action(), rsvp).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

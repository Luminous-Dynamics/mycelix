// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Property Bridge Integrity Zome
//!
//! Entry types for cross-hApp property queries, ownership verification, and collateral pledging.

use hdi::prelude::*;

/// Property ownership query from another hApp
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct OwnershipQuery {
    pub id: String,
    pub property_id: String,
    pub source_happ: String,
    pub purpose: OwnershipQueryPurpose,
    pub queried_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum OwnershipQueryPurpose {
    CollateralVerification,
    TransferVerification,
    DisputeResolution,
    TaxAssessment,
}

/// Property ownership verification result
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct OwnershipResult {
    pub id: String,
    pub property_id: String,
    pub owner_did: String,
    pub ownership_type: OwnershipType,
    pub share_percentage: f64,
    pub encumbrances: Vec<Encumbrance>,
    pub verified_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum OwnershipType {
    Sole,
    Joint,
    Fractional,
    Corporate,
    Trust,
    Commons,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Encumbrance {
    pub encumbrance_type: EncumbranceType,
    pub holder_did: String,
    pub amount: Option<u64>,
    pub expires_at: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EncumbranceType {
    Mortgage,
    Lien,
    Easement,
    Lease,
    CollateralPledge,
}

/// Cross-hApp collateral pledge
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralPledge {
    pub id: String,
    pub property_id: String,
    pub owner_did: String,
    pub pledged_to_happ: String,
    pub pledged_for_loan: String,
    pub value: u64,
    pub currency: String,
    pub status: PledgeStatus,
    pub pledged_at: Timestamp,
    pub released_at: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PledgeStatus {
    Active,
    Released,
    Foreclosed,
}

/// Property event for broadcasting
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PropertyBridgeEvent {
    pub id: String,
    pub event_type: PropertyEventType,
    pub property_id: String,
    pub subject_did: String,
    pub payload: String,
    pub source_happ: String,
    pub timestamp: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PropertyEventType {
    OwnershipTransferred,
    PropertyRegistered,
    CollateralPledged,
    CollateralReleased,
    EncumbranceAdded,
    EncumbranceRemoved,
    DisputeFiled,
}

/// Anchor entry for deterministic link bases from strings
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    OwnershipQuery(OwnershipQuery),
    OwnershipResult(OwnershipResult),
    CollateralPledge(CollateralPledge),
    PropertyBridgeEvent(PropertyBridgeEvent),
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    PropertyToOwnership,
    DidToProperties,
    ActivePledges,
    RecentEvents,
    /// Anchor("ownership_changes:{property_id}") -> PropertyBridgeEvent.
    /// Added while fixing this zome's compile errors (see validate() note
    /// below) -- coordinator's get_ownership_changes/broadcast_ownership_change
    /// already referenced this variant, but it didn't exist yet.
    PropertyOwnershipChanges,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
///
/// NOTE: this zome previously had no `validate(op: Op)` dispatcher at all --
/// the 5 functions below were each individually marked `#[hdk_extern]` while
/// taking two parameters, which is invalid for a zome extern (`hdk_extern`
/// requires 0 or 1 parameter) and does not compile. That's why `bridge` was
/// never listed as a member of this workspace (see Cargo.toml) -- it was
/// dead, uncompilable code, silently excluded rather than fixed. Fixed here
/// by de-externing the per-entry-type functions and wiring a real
/// dispatcher, same shape as every sibling zome in this hApp.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::OwnershipQuery(query) => validate_create_entry_ownership_query(
                    EntryCreationAction::Create(action),
                    query,
                ),
                EntryTypes::OwnershipResult(result) => validate_create_entry_ownership_result(
                    EntryCreationAction::Create(action),
                    result,
                ),
                EntryTypes::CollateralPledge(pledge) => validate_create_entry_collateral_pledge(
                    EntryCreationAction::Create(action),
                    pledge,
                ),
                EntryTypes::PropertyBridgeEvent(event) => {
                    validate_create_entry_property_bridge_event(
                        EntryCreationAction::Create(action),
                        event,
                    )
                }
                EntryTypes::Anchor(anchor) => {
                    validate_create_entry_anchor(EntryCreationAction::Create(action), anchor)
                }
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Bridge entries are immutable and cannot be updated".to_string(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_entry_ownership_query(
    _action: EntryCreationAction,
    query: OwnershipQuery,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: OwnershipQuery is a system-generated
    // audit trail entry created by verify_ownership on every query, with no
    // self-declared identity field to forge.
    if query.property_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Property ID required".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_entry_ownership_result(
    _action: EntryCreationAction,
    result: OwnershipResult,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: owner_did here is the REGISTRY's answer
    // to an ownership query, looked up cross-zome by verify_ownership, not a
    // self-declaration by the committing agent -- binding it to the caller
    // would reject every legitimate query result (the caller is the
    // querier, not the owner).
    if result.share_percentage < 0.0 || result.share_percentage > 100.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Share must be 0-100%".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_entry_collateral_pledge(
    action: EntryCreationAction,
    pledge: CollateralPledge,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the pledge to its committer -- pledge_collateral already derives
    // `owner_did` from agent_info() coordinator-side with zero user input,
    // so this never rejects a legitimate pledge; it's the real DHT-level
    // enforcement a modified coordinator could otherwise bypass (P0
    // author-binding gap). NOTE: this only proves the pledger isn't
    // impersonating someone else -- it does NOT verify the pledger actually
    // owns the property in the registry zome. pledge_collateral has no
    // cross-zome call to registry's verify_ownership/get_property today;
    // adding one is new integration logic beyond this pass's identity-
    // binding scope, flagged here for a follow-up.
    let expected_owner = format!("did:mycelix:{}", action.author());
    if pledge.owner_did != expected_owner {
        return Ok(ValidateCallbackResult::Invalid(
            "Collateral pledge owner must be the committing agent (forgery)".to_string(),
        ));
    }

    if !pledge.owner_did.starts_with("did:mycelix:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must have valid DID".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_entry_property_bridge_event(
    _action: EntryCreationAction,
    event: PropertyBridgeEvent,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: subject_did names whoever the event is
    // ABOUT (e.g. the new owner in an ownership-change broadcast), not
    // necessarily the committing agent -- broadcast_property_event/
    // broadcast_ownership_change are called cross-zome by other coordinators
    // (transfer's complete_transfer) reporting on a third party's property.
    if event.source_happ.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Source hApp required".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_entry_anchor(
    _action: EntryCreationAction,
    _anchor: Anchor,
) -> ExternResult<ValidateCallbackResult> {
    // Anchors are always valid - they're just string wrappers for link bases
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

    fn test_author_did() -> String {
        format!("did:mycelix:{}", test_action().author)
    }

    #[test]
    fn test_ownership_query_valid() {
        let query = OwnershipQuery {
            id: "ownership:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            source_happ: "mycelix-finance".to_string(),
            purpose: OwnershipQueryPurpose::CollateralVerification,
            queried_at: Timestamp::from_micros(0),
        };
        let result = validate_create_entry_ownership_query(
            EntryCreationAction::Create(test_action()),
            query,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_ownership_query_empty_property_rejected() {
        let query = OwnershipQuery {
            id: "ownership:test:0".to_string(),
            property_id: "".to_string(),
            source_happ: "mycelix-finance".to_string(),
            purpose: OwnershipQueryPurpose::CollateralVerification,
            queried_at: Timestamp::from_micros(0),
        };
        let result = validate_create_entry_ownership_query(
            EntryCreationAction::Create(test_action()),
            query,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_ownership_result_valid() {
        let result_entry = OwnershipResult {
            id: "result:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            owner_did: "did:mycelix:someone-else".to_string(),
            ownership_type: OwnershipType::Sole,
            share_percentage: 100.0,
            encumbrances: vec![],
            verified_at: Timestamp::from_micros(0),
        };
        // The querier (test_action's author) is NOT the owner -- this must
        // still be Valid since OwnershipResult is a query answer, not a
        // self-declaration.
        let result = validate_create_entry_ownership_result(
            EntryCreationAction::Create(test_action()),
            result_entry,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_ownership_result_bad_share_rejected() {
        let result_entry = OwnershipResult {
            id: "result:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            owner_did: "did:mycelix:someone-else".to_string(),
            ownership_type: OwnershipType::Sole,
            share_percentage: 150.0,
            encumbrances: vec![],
            verified_at: Timestamp::from_micros(0),
        };
        let result = validate_create_entry_ownership_result(
            EntryCreationAction::Create(test_action()),
            result_entry,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_pledge(owner_did: String) -> CollateralPledge {
        CollateralPledge {
            id: "pledge:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            owner_did,
            pledged_to_happ: "mycelix-finance".to_string(),
            pledged_for_loan: "loan:test:0".to_string(),
            value: 1000,
            currency: "USD".to_string(),
            status: PledgeStatus::Active,
            pledged_at: Timestamp::from_micros(0),
            released_at: None,
        }
    }

    #[test]
    fn test_collateral_pledge_valid() {
        let pledge = valid_pledge(test_author_did());
        let result = validate_create_entry_collateral_pledge(
            EntryCreationAction::Create(test_action()),
            pledge,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_collateral_pledge_owner_forgery_rejected() {
        // owner_did claims test_author_did(), but the committing action's
        // author is a different agent.
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let pledge = valid_pledge(test_author_did());
        let result = validate_create_entry_collateral_pledge(
            EntryCreationAction::Create(forged_action),
            pledge,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_property_bridge_event_valid() {
        let event = PropertyBridgeEvent {
            id: "event:test:0".to_string(),
            event_type: PropertyEventType::OwnershipTransferred,
            property_id: "property:test:0".to_string(),
            subject_did: "did:mycelix:new-owner".to_string(),
            payload: "{}".to_string(),
            source_happ: "mycelix-property".to_string(),
            timestamp: Timestamp::from_micros(0),
        };
        let result = validate_create_entry_property_bridge_event(
            EntryCreationAction::Create(test_action()),
            event,
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_property_bridge_event_empty_source_rejected() {
        let event = PropertyBridgeEvent {
            id: "event:test:0".to_string(),
            event_type: PropertyEventType::OwnershipTransferred,
            property_id: "property:test:0".to_string(),
            subject_did: "did:mycelix:new-owner".to_string(),
            payload: "{}".to_string(),
            source_happ: "".to_string(),
            timestamp: Timestamp::from_micros(0),
        };
        let result = validate_create_entry_property_bridge_event(
            EntryCreationAction::Create(test_action()),
            event,
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

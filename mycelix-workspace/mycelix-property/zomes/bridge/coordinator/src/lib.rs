// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Property Bridge Coordinator Zome
//!
//! Cross-hApp communication for property ownership verification,
//! collateral pledging, and encumbrance management.

use hdk::prelude::*;
use property_bridge_integrity::*;

const PROPERTY_HAPP_ID: &str = "mycelix-property";

/// Get or create an anchor entry and return its EntryHash for use as link base
fn anchor_hash(anchor_string: &str) -> ExternResult<EntryHash> {
    let anchor = Anchor(anchor_string.to_string());
    let _ = create_entry(&EntryTypes::Anchor(anchor.clone()));
    hash_entry(&anchor)
}

/// Verify property ownership by querying the registry zome
#[hdk_extern]
pub fn verify_ownership(input: VerifyOwnershipInput) -> ExternResult<OwnershipResult> {
    let now = sys_time()?;

    // Create audit trail
    let query = OwnershipQuery {
        id: format!(
            "ownership:{}:{}:{}",
            input.source_happ,
            input.property_id,
            now.as_micros()
        ),
        property_id: input.property_id.clone(),
        source_happ: input.source_happ.clone(),
        purpose: input.purpose.clone(),
        queried_at: now,
    };
    create_entry(&EntryTypes::OwnershipQuery(query))?;

    // Query the registry zome for actual ownership data
    //
    // NOTE: registry's coordinator has no `get_property_ownership_data`
    // extern today -- this call will fail at runtime (falling through to
    // the `None` branch below) until that function is added. Pre-existing
    // gap, not introduced or fixed here; this pass only fixes the compile
    // errors that were blocking `bridge` from ever being a buildable
    // workspace member (see Cargo.toml).
    let registry_result: Option<RegistryPropertyData> = call(
        CallTargetCell::Local,
        "registry",
        "get_property_ownership_data".into(),
        None,
        input.property_id.clone(),
    )
    .ok()
    .and_then(|response| match response {
        ZomeCallResponse::Ok(bytes) => bytes.decode().ok(),
        _ => None,
    });

    let (owner_did, ownership_type, share_percentage, encumbrances) = match registry_result {
        Some(data) => (
            data.owner_did,
            if data.co_owners.is_empty() {
                OwnershipType::Sole
            } else {
                OwnershipType::Joint
            },
            if data.co_owners.is_empty() {
                100.0
            } else {
                100.0
                    - data
                        .co_owners
                        .iter()
                        .map(|c| c.share_percentage)
                        .sum::<f64>()
            },
            data.encumbrances
                .iter()
                .map(|e| Encumbrance {
                    encumbrance_type: parse_encumbrance_type(&e.encumbrance_type),
                    holder_did: e.holder_did.clone(),
                    amount: e.amount.map(|a| a as u64),
                    expires_at: None,
                })
                .collect(),
        ),
        None => {
            // Fallback if registry query fails - use expected owner
            (
                input.expected_owner.clone().unwrap_or_default(),
                OwnershipType::Sole,
                100.0,
                vec![],
            )
        }
    };

    let result = OwnershipResult {
        id: format!("result:{}:{}", input.property_id, now.as_micros()),
        property_id: input.property_id.clone(),
        owner_did: owner_did.clone(),
        ownership_type,
        share_percentage,
        encumbrances,
        verified_at: now,
    };

    let hash = create_entry(&EntryTypes::OwnershipResult(result.clone()))?;
    create_link(
        anchor_hash(&input.property_id)?,
        hash,
        LinkTypes::PropertyToOwnership,
        (),
    )?;

    // Verify against expected owner if provided
    if let Some(expected) = &input.expected_owner {
        if owner_did != *expected {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Owner mismatch: expected {} but found {}",
                expected, owner_did
            ))));
        }
    }

    Ok(result)
}

/// Data structure for registry query response
#[derive(Serialize, Deserialize, Debug)]
struct RegistryPropertyData {
    pub owner_did: String,
    pub co_owners: Vec<CoOwnerInfo>,
    pub encumbrances: Vec<EncumbranceData>,
}

#[derive(Serialize, Deserialize, Debug)]
struct CoOwnerInfo {
    pub did: String,
    pub share_percentage: f64,
}

#[derive(Serialize, Deserialize, Debug)]
struct EncumbranceData {
    pub encumbrance_type: String,
    pub holder_did: String,
    pub amount: Option<f64>,
}

/// Best-effort mapping from the registry's free-form encumbrance_type string
/// to this crate's EncumbranceType enum. Registry sends an arbitrary string
/// (see EncumbranceData above); anything unrecognized defaults to Lien
/// rather than failing the whole ownership verification.
fn parse_encumbrance_type(s: &str) -> EncumbranceType {
    match s {
        "Mortgage" => EncumbranceType::Mortgage,
        "Lien" => EncumbranceType::Lien,
        "Easement" => EncumbranceType::Easement,
        "Lease" => EncumbranceType::Lease,
        "CollateralPledge" => EncumbranceType::CollateralPledge,
        _ => EncumbranceType::Lien,
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VerifyOwnershipInput {
    pub property_id: String,
    pub source_happ: String,
    pub purpose: OwnershipQueryPurpose,
    pub expected_owner: Option<String>,
}

/// Pledge property as collateral
#[hdk_extern]
pub fn pledge_collateral(input: PledgeCollateralInput) -> ExternResult<Record> {
    let now = sys_time()?;
    // Always the committing agent, never caller-supplied -- otherwise any
    // agent could pledge property they don't own as collateral (P0
    // author-binding gap; integrity validation now enforces this too, see
    // bridge integrity's validate_create_entry_collateral_pledge). NOTE:
    // this proves the pledger isn't impersonating someone else, but does
    // NOT verify they actually own the property in the registry zome --
    // that would need a cross-zome call to registry's verify_ownership,
    // which this function doesn't make today. Flagged, not added, in this
    // pass (see integrity comment for the same caveat).
    let owner_did = format!("did:mycelix:{}", agent_info()?.agent_initial_pubkey);

    let pledge = CollateralPledge {
        id: format!(
            "pledge:{}:{}:{}",
            input.property_id,
            input.pledged_to_happ,
            now.as_micros()
        ),
        property_id: input.property_id.clone(),
        owner_did: owner_did.clone(),
        pledged_to_happ: input.pledged_to_happ.clone(),
        pledged_for_loan: input.loan_id,
        value: input.value,
        currency: input.currency,
        status: PledgeStatus::Active,
        pledged_at: now,
        released_at: None,
    };

    let hash = create_entry(&EntryTypes::CollateralPledge(pledge))?;

    create_link(
        anchor_hash("active_pledges")?,
        hash.clone(),
        LinkTypes::ActivePledges,
        (),
    )?;

    broadcast_property_event(BroadcastPropertyEventInput {
        event_type: PropertyEventType::CollateralPledged,
        property_id: input.property_id,
        subject_did: owner_did,
        payload: serde_json::json!({
            "pledged_to": input.pledged_to_happ,
            "value": input.value,
        })
        .to_string(),
    })?;

    get(hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Pledge not found".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct PledgeCollateralInput {
    pub property_id: String,
    /// Deliberately ignored -- owner identity is always derived from the
    /// committing agent (P0 author-binding fix). Kept for client compat.
    pub owner_did: String,
    pub pledged_to_happ: String,
    pub loan_id: String,
    pub value: u64,
    pub currency: String,
}

/// Release collateral pledge
#[hdk_extern]
pub fn release_collateral(input: ReleaseCollateralInput) -> ExternResult<bool> {
    // Always the committing agent, never caller-supplied -- otherwise the
    // broadcast event log could misattribute who released a pledge.
    let owner_did = format!("did:mycelix:{}", agent_info()?.agent_initial_pubkey);
    broadcast_property_event(BroadcastPropertyEventInput {
        event_type: PropertyEventType::CollateralReleased,
        property_id: input.property_id,
        subject_did: owner_did,
        payload: serde_json::json!({
            "pledge_id": input.pledge_id,
        })
        .to_string(),
    })?;
    Ok(true)
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ReleaseCollateralInput {
    pub pledge_id: String,
    pub property_id: String,
    /// Deliberately ignored -- owner identity is always derived from the
    /// committing agent (P0 author-binding fix). Kept for client compat.
    pub owner_did: String,
}

/// Get properties owned by a DID
#[hdk_extern]
pub fn get_properties_by_owner(owner_did: String) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&owner_did)?, LinkTypes::DidToProperties)?,
        GetStrategy::default(),
    )?;

    let mut properties = Vec::new();
    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link".into())))?;
        if let Some(record) = get(hash, GetOptions::default())? {
            properties.push(record);
        }
    }

    Ok(properties)
}

/// Broadcast property event
#[hdk_extern]
pub fn broadcast_property_event(input: BroadcastPropertyEventInput) -> ExternResult<Record> {
    let now = sys_time()?;

    let event = PropertyBridgeEvent {
        id: format!("event:{:?}:{}", input.event_type, now.as_micros()),
        event_type: input.event_type,
        property_id: input.property_id,
        subject_did: input.subject_did,
        payload: input.payload,
        source_happ: PROPERTY_HAPP_ID.to_string(),
        timestamp: now,
    };

    let hash = create_entry(&EntryTypes::PropertyBridgeEvent(event))?;

    create_link(
        anchor_hash("recent_events")?,
        hash.clone(),
        LinkTypes::RecentEvents,
        (),
    )?;

    get(hash, GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Event not found".into())))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BroadcastPropertyEventInput {
    pub event_type: PropertyEventType,
    pub property_id: String,
    pub subject_did: String,
    pub payload: String,
}

// =============================================================================
// Ownership Change Notifications
// =============================================================================

/// Broadcast ownership change event
///
/// Called by the transfer zome when a transfer completes to notify other hApps
/// (finance, justice, etc.) about the ownership change.
#[hdk_extern]
pub fn broadcast_ownership_change(input: BroadcastOwnershipChangeInput) -> ExternResult<Record> {
    let now = sys_time()?;

    // Create property bridge event
    let event = PropertyBridgeEvent {
        id: format!("ownership_change:{}:{}", input.property_id, now.as_micros()),
        event_type: PropertyEventType::OwnershipTransferred,
        property_id: input.property_id.clone(),
        subject_did: input.to_did.clone(),
        payload: serde_json::json!({
            "from_did": input.from_did,
            "to_did": input.to_did,
            "transfer_type": input.transfer_type,
            "new_deed_id": input.new_deed_id,
        })
        .to_string(),
        source_happ: PROPERTY_HAPP_ID.to_string(),
        timestamp: now,
    };

    let hash = create_entry(&EntryTypes::PropertyBridgeEvent(event))?;

    // Link to recent events
    create_link(
        anchor_hash("recent_events")?,
        hash.clone(),
        LinkTypes::RecentEvents,
        (),
    )?;

    // Link to ownership change events for the property
    create_link(
        anchor_hash(&format!("ownership_changes:{}", input.property_id))?,
        hash.clone(),
        LinkTypes::PropertyOwnershipChanges,
        (),
    )?;

    // Update owner links: remove old owner link, add new owner link
    create_link(
        anchor_hash(&input.to_did)?,
        hash.clone(),
        LinkTypes::DidToProperties,
        (),
    )?;

    get(hash, GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Event not found".into())))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BroadcastOwnershipChangeInput {
    pub property_id: String,
    pub from_did: String,
    pub to_did: String,
    pub transfer_type: String,
    pub new_deed_id: String,
}

/// Get ownership change history for a property
#[hdk_extern]
pub fn get_ownership_changes(property_id: String) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash(&format!("ownership_changes:{}", property_id))?,
            LinkTypes::PropertyOwnershipChanges,
        )?,
        GetStrategy::default(),
    )?;

    let mut changes = Vec::new();
    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link".into())))?;
        if let Some(record) = get(hash, GetOptions::default())? {
            changes.push(record);
        }
    }

    Ok(changes)
}

/// Get recent property events
#[hdk_extern]
pub fn get_recent_property_events(limit: u32) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("recent_events")?, LinkTypes::RecentEvents)?,
        GetStrategy::default(),
    )?;

    let mut events = Vec::new();
    for link in links.into_iter().rev().take(limit as usize) {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link".into())))?;
        if let Some(record) = get(hash, GetOptions::default())? {
            events.push(record);
        }
    }

    Ok(events)
}

/// Get all active collateral pledges
#[hdk_extern]
pub fn get_active_pledges(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("active_pledges")?, LinkTypes::ActivePledges)?,
        GetStrategy::default(),
    )?;

    let mut pledges = Vec::new();
    for link in links {
        let hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link".into())))?;
        if let Some(record) = get(hash, GetOptions::default())? {
            pledges.push(record);
        }
    }

    Ok(pledges)
}

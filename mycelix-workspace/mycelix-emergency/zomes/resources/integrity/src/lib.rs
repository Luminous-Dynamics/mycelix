// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Resources Integrity Zome
//! Emergency resource tracking and deployment

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// An emergency resource
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EmergencyResource {
    pub id: String,
    pub resource_type: ResourceType,
    pub name: String,
    pub quantity: u32,
    pub unit: String,
    pub location: String,
    pub owner: AgentPubKey,
    pub status: ResourceStatus,
    pub deployed_to: Option<ActionHash>,
}

/// Types of emergency resources
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ResourceType {
    Medical,
    Personnel,
    Equipment,
    Shelter,
    Transport,
    Communication,
    Food,
    Water,
    Power,
    Fuel,
}

/// Resource status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ResourceStatus {
    Available,
    Deployed,
    InTransit,
    Depleted,
    Damaged,
}

/// A request for resources
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ResourceRequest {
    pub disaster_hash: ActionHash,
    pub requesting_team: AgentPubKey,
    pub resource_type: ResourceType,
    pub quantity_needed: u32,
    pub urgency: UrgencyLevel,
    pub location: String,
    pub status: RequestStatus,
    pub fulfilled_by: Option<ActionHash>,
}

/// Urgency levels for resource requests
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum UrgencyLevel {
    Critical,
    High,
    Medium,
    Low,
}

/// Status of a resource request
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RequestStatus {
    Pending,
    Approved,
    Fulfilled,
    PartiallyFulfilled,
    Denied,
    Cancelled,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    EmergencyResource(EmergencyResource),
    ResourceRequest(ResourceRequest),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllResources,
    AvailableResources,
    ResourceByType,
    DisasterToRequest,
    AgentToResource,
    RequestToResource,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::EmergencyResource(resource) => {
                    validate_create_resource(action, resource)
                }
                EntryTypes::ResourceRequest(request) => validate_create_request(action, request),
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
            LinkTypes::AllResources => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AvailableResources => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ResourceByType => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DisasterToRequest => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToResource => Ok(ValidateCallbackResult::Valid),
            LinkTypes::RequestToResource => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): the
        // coordinator's deploy_resource deletes the AvailableResources
        // link when a resource is deployed, and deployment is typically
        // ordered by a dispatcher/incident commander who need not be the
        // resource's original owner (and thus not the link's original
        // creator). Same reasoning as mycelix-emergency/incidents'
        // ActiveDisasters link and mycelix-emergency/comms.
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
            // unconditionally) -- the 13th confirmed instance of this exact
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
        EntryTypes::EmergencyResource(resource) => validate_update_resource(action, resource),
        EntryTypes::ResourceRequest(request) => validate_update_request(action, request),
    }
}

fn validate_create_resource(
    action: Create,
    resource: EmergencyResource,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's register_resource already derives
    // owner from agent_info(), so this is belt-and-suspenders against a
    // modified coordinator forging a victim agent as the resource owner.
    // Found + fixed 2026-07-09 during the P0 author-binding pass.
    if resource.owner != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource owner must correspond to the committing agent".into(),
        ));
    }

    if resource.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource ID cannot be empty".into(),
        ));
    }
    if resource.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource name cannot be empty".into(),
        ));
    }
    if resource.unit.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource unit cannot be empty".into(),
        ));
    }
    if resource.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource location cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate an EmergencyResource update (the coordinator's deploy_resource
/// changes status/deployed_to/location together).
///
/// No author requirement: deploy_resource has no caller-identity check at
/// all in the coordinator (deployment is typically ordered by a
/// dispatcher, not necessarily the resource's registered owner) -- no
/// established authority model here to bind against, same reasoning as
/// mycelix-emergency/incidents' Disaster update. Reviewed 2026-07-09
/// during the P0 author-binding pass; case (c). Content is restricted to
/// status/deployed_to/location -- this closes the wide-open bug that
/// previously let any field (including owner, quantity, or resource_type)
/// change unconditionally on update.
fn validate_update_resource(
    action: Update,
    resource: EmergencyResource,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: EmergencyResource = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original resource not found".into()
        )))?;

    if resource.id != original.id
        || resource.resource_type != original.resource_type
        || resource.name != original.name
        || resource.quantity != original.quantity
        || resource.unit != original.unit
        || resource.owner != original.owner
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/deployed_to/location can change on a resource update".into(),
        ));
    }

    if resource.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource ID cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_request(
    action: Create,
    request: ResourceRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's request_resource already derives
    // requesting_team from agent_info(), so this is belt-and-suspenders
    // against a modified coordinator forging a victim agent as requester.
    // Found + fixed 2026-07-09 during the P0 author-binding pass.
    if request.requesting_team != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Request requesting_team must correspond to the committing agent".into(),
        ));
    }

    if request.quantity_needed == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity needed must be greater than 0".into(),
        ));
    }
    if request.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request location cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a ResourceRequest update (the coordinator's fulfill_request
/// changes status/fulfilled_by together).
///
/// No author requirement: fulfill_request has no caller-identity check at
/// all in the coordinator (any agent holding a matching resource may
/// fulfill a request) -- no established authority model here to bind
/// against, case (c). Content is restricted to status/fulfilled_by --
/// this closes the wide-open bug that previously let any field (including
/// requesting_team, resource_type, quantity_needed, or location) change
/// unconditionally on update.
fn validate_update_request(
    action: Update,
    request: ResourceRequest,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: ResourceRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original request not found".into()
        )))?;

    if request.disaster_hash != original.disaster_hash
        || request.requesting_team != original.requesting_team
        || request.resource_type != original.resource_type
        || request.quantity_needed != original.quantity_needed
        || request.urgency != original.urgency
        || request.location != original.location
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/fulfilled_by can change on a request update".into(),
        ));
    }

    if request.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request location cannot be empty".into(),
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

    fn valid_resource(owner: AgentPubKey) -> EmergencyResource {
        EmergencyResource {
            id: "r-1".into(),
            resource_type: ResourceType::Medical,
            name: "First aid kits".into(),
            quantity: 10,
            unit: "kits".into(),
            location: "warehouse-1".into(),
            owner,
            status: ResourceStatus::Available,
            deployed_to: None,
        }
    }

    #[test]
    fn create_resource_valid_when_owner_matches_committer() {
        let r = valid_resource(me());
        let result = validate_create_resource(create_action(me()), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_resource_forgery_rejected() {
        let r = valid_resource(me());
        let result = validate_create_resource(create_action(other_agent()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_request(requesting_team: AgentPubKey) -> ResourceRequest {
        ResourceRequest {
            disaster_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            requesting_team,
            resource_type: ResourceType::Water,
            quantity_needed: 5,
            urgency: UrgencyLevel::High,
            location: "zone-a".into(),
            status: RequestStatus::Pending,
            fulfilled_by: None,
        }
    }

    #[test]
    fn create_request_valid_when_team_matches_committer() {
        let r = valid_request(me());
        let result = validate_create_request(create_action(me()), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_request_forgery_rejected() {
        let r = valid_request(me());
        let result = validate_create_request(create_action(other_agent()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

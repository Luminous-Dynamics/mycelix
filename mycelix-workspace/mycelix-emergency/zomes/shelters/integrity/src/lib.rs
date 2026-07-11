// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Shelters Integrity Zome
//! Emergency shelter registration and occupancy tracking

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// An emergency shelter
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Shelter {
    pub id: String,
    pub name: String,
    pub location_lat: f64,
    pub location_lon: f64,
    pub address: String,
    pub capacity: u32,
    pub current_occupancy: u32,
    pub shelter_type: ShelterType,
    pub amenities: Vec<Amenity>,
    pub status: ShelterStatus,
    pub contact: String,
}

/// Types of shelters
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ShelterType {
    Emergency,
    Community,
    Medical,
    PetFriendly,
    Accessible,
}

/// Amenities available at a shelter
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum Amenity {
    Power,
    Water,
    Medical,
    Food,
    Showers,
    Wifi,
    Charging,
    Cots,
    Blankets,
    PetArea,
    ChildCare,
    MentalHealth,
}

/// Shelter operational status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ShelterStatus {
    Open,
    Full,
    Closed,
    Evacuating,
}

/// A person registered at a shelter
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ShelterRegistration {
    pub shelter_hash: ActionHash,
    pub person_name: String,
    pub person_id: Option<String>,
    pub party_size: u8,
    pub special_needs: Vec<String>,
    pub registered_at: Timestamp,
    pub checked_out_at: Option<Timestamp>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Shelter(Shelter),
    ShelterRegistration(ShelterRegistration),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllShelters,
    OpenShelters,
    ShelterToRegistration,
    ShelterByType,
    PersonToRegistration,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Shelter(shelter) => validate_create_shelter(action, shelter),
                EntryTypes::ShelterRegistration(reg) => validate_create_registration(action, reg),
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
            LinkTypes::AllShelters => Ok(ValidateCallbackResult::Valid),
            LinkTypes::OpenShelters => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ShelterToRegistration => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ShelterByType => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PersonToRegistration => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): the
        // coordinator deletes/recreates the OpenShelters link from THREE
        // different functions (update_shelter_status, check_in_person,
        // check_out_person), each potentially called by a different
        // shelter-staff agent, and neither Shelter nor ShelterRegistration
        // has any owner/staff-identity field to compare against in the
        // first place (see the case-(a) notes on validate_create_shelter/
        // validate_create_registration below). Same reasoning as the
        // other zomes in this cluster with genuine cross-agent
        // link-deletion flows.
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
            // unconditionally) -- the 14th confirmed instance of this exact
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
        EntryTypes::Shelter(shelter) => validate_update_shelter(action, shelter),
        EntryTypes::ShelterRegistration(reg) => validate_update_registration(action, reg),
    }
}

/// No author-binding possible: Shelter has no self-declared owner/staff
/// field at all (any agent may register a shelter or later update its
/// status/occupancy on behalf of the physical location). Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (a). Adding an
/// owner field would be a schema change, out of scope for this pass.
fn validate_create_shelter(
    _action: Create,
    shelter: Shelter,
) -> ExternResult<ValidateCallbackResult> {
    if shelter.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Shelter ID cannot be empty".into(),
        ));
    }
    if shelter.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Shelter name cannot be empty".into(),
        ));
    }
    if shelter.address.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Shelter address cannot be empty".into(),
        ));
    }
    if shelter.capacity == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Shelter capacity must be greater than 0".into(),
        ));
    }
    if shelter.current_occupancy > shelter.capacity {
        return Ok(ValidateCallbackResult::Invalid(
            "Occupancy cannot exceed capacity".into(),
        ));
    }
    if shelter.location_lat < -90.0 || shelter.location_lat > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be between -90 and 90".into(),
        ));
    }
    if shelter.location_lon < -180.0 || shelter.location_lon > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be between -180 and 180".into(),
        ));
    }
    if shelter.contact.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Contact information cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a Shelter update (the coordinator's update_shelter_status
/// changes only `status`; check_in_person/check_out_person change
/// `current_occupancy`/`status` together).
///
/// No author requirement: see the case-(a) note on validate_create_shelter
/// above -- there's no owner/staff field to bind against at all. Content
/// is restricted to status/current_occupancy -- this closes the wide-open
/// bug that previously let any field (including capacity, address, or
/// amenities) change unconditionally on update.
fn validate_update_shelter(
    action: Update,
    shelter: Shelter,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Shelter = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original shelter not found".into()
        )))?;

    if shelter.id != original.id
        || shelter.name != original.name
        || shelter.location_lat != original.location_lat
        || shelter.location_lon != original.location_lon
        || shelter.address != original.address
        || shelter.capacity != original.capacity
        || shelter.shelter_type != original.shelter_type
        || shelter.amenities != original.amenities
        || shelter.contact != original.contact
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/current_occupancy can change on a shelter update".into(),
        ));
    }

    if shelter.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Shelter ID cannot be empty".into(),
        ));
    }
    if shelter.current_occupancy > shelter.capacity {
        return Ok(ValidateCallbackResult::Invalid(
            "Occupancy cannot exceed capacity".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// No author-binding possible: ShelterRegistration has no self-declared
/// registrant/staff field at all (person_name/person_id name the guest,
/// not the committing agent -- shelter staff register guests on their
/// behalf). Reviewed 2026-07-09 during the P0 author-binding pass; case
/// (a).
fn validate_create_registration(
    _action: Create,
    reg: ShelterRegistration,
) -> ExternResult<ValidateCallbackResult> {
    if reg.person_name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Person name cannot be empty".into(),
        ));
    }
    if reg.party_size == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Party size must be at least 1".into(),
        ));
    }
    if reg.checked_out_at.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot create registration already checked out".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a ShelterRegistration update (the coordinator's
/// check_out_person changes only `checked_out_at`).
///
/// No author requirement: same case-(a) reasoning as
/// validate_create_registration above. Content is restricted to
/// checked_out_at -- this closes the wide-open bug that previously
/// accepted ANY change unconditionally (the dispatcher's OpEntry arm for
/// ShelterRegistration was just `Ok(Valid)` with no checks at all).
fn validate_update_registration(
    action: Update,
    reg: ShelterRegistration,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: ShelterRegistration = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original registration not found".into()
        )))?;

    if reg.shelter_hash != original.shelter_hash
        || reg.person_name != original.person_name
        || reg.person_id != original.person_id
        || reg.party_size != original.party_size
        || reg.special_needs != original.special_needs
        || reg.registered_at != original.registered_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only checked_out_at can change on a registration update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

// No author_binding_tests module here: unlike every other zome fixed
// this pass, neither Shelter nor ShelterRegistration has a self-declared
// identity field to forge in the first place (see the case-(a) notes on
// validate_create_shelter/validate_create_registration above), so there
// is no forgery scenario to test. Both update validators require
// must_get_valid_record, which needs a live HDI host this crate's tests
// don't mock -- consistent with how other must_get-based validators were
// left untested directly earlier in this pass.

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Units Integrity Zome
//! Defines entry types and validation for buildings and housing units.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Type of building in the cooperative
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum BuildingType {
    Apartment,
    Townhouse,
    SingleFamily,
    Duplex,
    CoHousing,
    MixedUse,
}

/// A building managed by the housing cooperative
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Building {
    pub id: String,
    pub name: String,
    pub address: String,
    pub location_lat: f64,
    pub location_lon: f64,
    pub total_units: u16,
    pub year_built: Option<u16>,
    pub building_type: BuildingType,
    pub cooperative_hash: Option<ActionHash>,
}

/// Type of housing unit
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum UnitType {
    Studio,
    OneBedroom,
    TwoBedroom,
    ThreeBedroom,
    FourPlus,
    Accessible,
    Family,
}

/// Accessibility features available in a unit
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AccessFeature {
    WheelchairAccessible,
    Elevator,
    GrabBars,
    WideDoorways,
    LowCounters,
    VisualAlerts,
    HearingLoop,
}

/// Current status of a housing unit
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum UnitStatus {
    Available,
    Occupied,
    UnderMaintenance,
    Reserved,
    Renovation,
}

/// A housing unit within a building
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Unit {
    pub building_hash: ActionHash,
    pub unit_number: String,
    pub unit_type: UnitType,
    pub square_meters: u32,
    pub floor: u8,
    pub bedrooms: u8,
    pub bathrooms: u8,
    pub accessibility_features: Vec<AccessFeature>,
    pub current_occupant: Option<AgentPubKey>,
    pub status: UnitStatus,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Building(Building),
    HousingUnit(Unit),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// All buildings anchor
    AllBuildings,
    /// Building to its units
    BuildingToUnit,
    /// Available units anchor
    AvailableUnits,
    /// Occupant to their unit
    OccupantToUnit,
    /// Building type index
    BuildingTypeToBuilding,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Building(building) => validate_create_building(action, building),
                EntryTypes::HousingUnit(unit) => validate_create_unit(action, unit),
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
            LinkTypes::AllBuildings => Ok(ValidateCallbackResult::Valid),
            LinkTypes::BuildingToUnit => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AvailableUnits => Ok(ValidateCallbackResult::Valid),
            LinkTypes::OccupantToUnit => Ok(ValidateCallbackResult::Valid),
            LinkTypes::BuildingTypeToBuilding => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap):
        // update_unit_status/assign_occupant/vacate_unit all
        // delete/recreate the AvailableUnits and OccupantToUnit links
        // from what may be a different admin agent than whoever
        // originally created the link.
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
            // unconditionally) -- the 26th confirmed instance of this exact
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

/// No entry type in this zome has a self-declared committer-identity
/// field to bind -- Unit.current_occupant is a third-party field by
/// design (an admin assigns an occupant, who need not be the
/// committer), and Building has no agent field at all. Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (a)/(b) across
/// the board. WHO may register/update buildings and units at all is
/// unchecked (no established authority model exists in this zome) --
/// case (c), a real but separate gap, not fixed here.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        // No live update_entry call for Building (confirmed via grep) --
        // previously silently accepted any field change. Made explicitly
        // immutable.
        EntryTypes::Building(_) => Ok(ValidateCallbackResult::Invalid(
            "Buildings are immutable".into(),
        )),
        EntryTypes::HousingUnit(unit) => validate_update_unit(action, unit),
    }
}

fn validate_create_building(
    _action: Create,
    building: Building,
) -> ExternResult<ValidateCallbackResult> {
    if building.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Building ID cannot be empty".into(),
        ));
    }
    if building.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Building name cannot be empty".into(),
        ));
    }
    if building.address.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Building address cannot be empty".into(),
        ));
    }
    if building.location_lat < -90.0 || building.location_lat > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be between -90 and 90".into(),
        ));
    }
    if building.location_lon < -180.0 || building.location_lon > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be between -180 and 180".into(),
        ));
    }
    if building.total_units == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Building must have at least one unit".into(),
        ));
    }
    if let Some(year) = building.year_built {
        if year < 1800 || year > 2100 {
            return Ok(ValidateCallbackResult::Invalid(
                "Year built must be between 1800 and 2100".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_unit(_action: Create, unit: Unit) -> ExternResult<ValidateCallbackResult> {
    if unit.unit_number.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Unit number cannot be empty".into(),
        ));
    }
    if unit.square_meters == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Square meters must be greater than 0".into(),
        ));
    }
    if unit.bathrooms == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Unit must have at least one bathroom".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Content restricted to status/current_occupant -- the exact fields
/// update_unit_status/assign_occupant/vacate_unit change (confirmed via
/// reading all three coordinator functions). This closes the wide-open
/// bug that previously let building_hash/unit_number/unit_type/
/// square_meters/floor/bedrooms/bathrooms/accessibility_features change
/// unconditionally on update too. No author requirement: none of the
/// three update flows have any caller-identity check in the coordinator
/// -- case (c).
fn validate_update_unit(action: Update, unit: Unit) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Unit = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original unit not found".into()
        )))?;

    if unit.building_hash != original.building_hash
        || unit.unit_number != original.unit_number
        || unit.unit_type != original.unit_type
        || unit.square_meters != original.square_meters
        || unit.floor != original.floor
        || unit.bedrooms != original.bedrooms
        || unit.bathrooms != original.bathrooms
        || unit.accessibility_features != original.accessibility_features
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/current_occupant can change on a unit update".into(),
        ));
    }

    if unit.unit_number.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Unit number cannot be empty".into(),
        ));
    }
    if unit.square_meters == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Square meters must be greater than 0".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod content_integrity_tests {
    use super::*;

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

    #[test]
    fn update_entry_type_rejects_building_update() {
        // No live update_entry call exists for Building -- dead-path
        // immutability, testable without must_get_valid_record.
        let building = Building {
            id: "b-1".into(),
            name: "Riverside Commons".into(),
            address: "123 Main St".into(),
            location_lat: 0.0,
            location_lon: 0.0,
            total_units: 12,
            year_built: Some(1998),
            building_type: BuildingType::Apartment,
            cooperative_hash: None,
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::Building(building))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

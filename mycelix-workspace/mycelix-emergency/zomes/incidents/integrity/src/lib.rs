// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Incidents Integrity Zome
//! Defines entry types and validation for disaster incidents

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// A declared disaster incident
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Disaster {
    pub id: String,
    pub disaster_type: DisasterType,
    pub title: String,
    pub description: String,
    pub severity: SeverityLevel,
    pub declared_by: AgentPubKey,
    pub declared_at: Timestamp,
    pub affected_area: AffectedArea,
    pub status: DisasterStatus,
    pub estimated_affected: u32,
    pub coordination_lead: Option<AgentPubKey>,
}

/// Types of disasters
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DisasterType {
    Hurricane,
    Earthquake,
    Wildfire,
    Flood,
    Tornado,
    Pandemic,
    Industrial,
    MassCasualty,
    CyberAttack,
    Infrastructure,
    Other(String),
}

/// Severity levels (FEMA-aligned)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SeverityLevel {
    Level1,
    Level2,
    Level3,
    Level4,
    Level5,
}

/// Status of a disaster
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DisasterStatus {
    Declared,
    Active,
    Recovery,
    Closed,
}

/// Geographic area affected by the disaster
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct AffectedArea {
    pub center_lat: f64,
    pub center_lon: f64,
    pub radius_km: f32,
    pub boundary: Option<Vec<(f64, f64)>>,
    pub zones: Vec<OperationalZone>,
}

/// An operational zone within the affected area
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct OperationalZone {
    pub id: String,
    pub name: String,
    pub boundary: Vec<(f64, f64)>,
    pub priority: ZonePriority,
    pub status: ZoneStatus,
}

/// Priority level for operational zones
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ZonePriority {
    Critical,
    High,
    Medium,
    Low,
}

/// Status of an operational zone
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ZoneStatus {
    Unassessed,
    Active,
    Cleared,
    Hazardous,
    Evacuated,
}

/// An update to an incident
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct IncidentUpdate {
    pub disaster_hash: ActionHash,
    pub author: AgentPubKey,
    pub timestamp: Timestamp,
    pub update_type: UpdateType,
    pub content: String,
}

/// Types of incident updates
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum UpdateType {
    StatusChange,
    SeverityChange,
    AreaExpansion,
    AreaContraction,
    CasualtyReport,
    ResourceUpdate,
    WeatherUpdate,
    InfrastructureUpdate,
    EvacuationOrder,
    AllClear,
    General,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Disaster(Disaster),
    IncidentUpdate(IncidentUpdate),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllDisasters,
    ActiveDisasters,
    DisasterByType,
    DisasterToUpdate,
    AgentToDisaster,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Disaster(disaster) => validate_create_disaster(action, disaster),
                EntryTypes::IncidentUpdate(update) => validate_create_update(action, update),
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
            LinkTypes::AllDisasters => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ActiveDisasters => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DisasterByType => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DisasterToUpdate => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToDisaster => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): the
        // coordinator's update_disaster_status deletes the ActiveDisasters
        // link when a disaster transitions to Closed/Recovery, and that
        // transition is typically performed by an incident commander who
        // is not necessarily the same agent who originally declared the
        // disaster (and thus not the link's original creator). Adding the
        // usual "only original link creator can delete" check here would
        // break that legitimate flow -- same reasoning as
        // mycelix-emergency/comms's get_active_broadcasts/mark_synced.
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
            // unconditionally) -- the 12th confirmed instance of this exact
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
        EntryTypes::Disaster(disaster) => validate_update_disaster(action, disaster),
        EntryTypes::IncidentUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Incident updates are immutable".into(),
        )),
    }
}

/// coordination_lead is deliberately NOT bound to the committer: it's a
/// third-party field by design (an incident commander declaring a disaster
/// may legitimately name a DIFFERENT agent as coordination lead). Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (b).
fn validate_create_disaster(
    action: Create,
    disaster: Disaster,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's declare_disaster already derives
    // declared_by from agent_info(), so this is belt-and-suspenders against
    // a modified coordinator forging a victim agent as declarer. Found +
    // fixed 2026-07-09 during the P0 author-binding pass.
    if disaster.declared_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Disaster declared_by must correspond to the committing agent".into(),
        ));
    }

    if disaster.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Disaster ID cannot be empty".into(),
        ));
    }
    if disaster.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Disaster title cannot be empty".into(),
        ));
    }
    if disaster.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Disaster description cannot be empty".into(),
        ));
    }
    if disaster.affected_area.radius_km <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Affected area radius must be positive".into(),
        ));
    }
    if disaster.affected_area.center_lat < -90.0 || disaster.affected_area.center_lat > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be between -90 and 90".into(),
        ));
    }
    if disaster.affected_area.center_lon < -180.0 || disaster.affected_area.center_lon > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be between -180 and 180".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a Disaster update (the coordinator's update_disaster_status
/// changes only `status`; end_disaster is a thin wrapper over the same).
///
/// No author requirement: update_disaster_status has no caller-identity
/// check at all in the coordinator (may legitimately be called by an
/// incident commander other than the original declarer) -- there's no
/// established authority model here to bind against, same reasoning as
/// mycelix-emergency/coordination's Team update. Reviewed 2026-07-09
/// during the P0 author-binding pass; case (c). Content is restricted to
/// `status` -- this closes the wide-open bug that previously let any
/// field (including severity, affected_area, declared_by, or
/// coordination_lead) change unconditionally on update.
fn validate_update_disaster(
    action: Update,
    disaster: Disaster,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Disaster = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original disaster not found".into()
        )))?;

    if disaster.id != original.id
        || disaster.disaster_type != original.disaster_type
        || disaster.title != original.title
        || disaster.description != original.description
        || disaster.severity != original.severity
        || disaster.declared_by != original.declared_by
        || disaster.declared_at != original.declared_at
        || disaster.affected_area != original.affected_area
        || disaster.estimated_affected != original.estimated_affected
        || disaster.coordination_lead != original.coordination_lead
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change on a disaster update".into(),
        ));
    }

    if disaster.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Disaster ID cannot be empty".into(),
        ));
    }
    if disaster.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Disaster title cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_update(
    action: Create,
    update: IncidentUpdate,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's add_incident_update already derives
    // author from agent_info(), so this is belt-and-suspenders against a
    // modified coordinator forging a victim agent as the update's author.
    // Found + fixed 2026-07-09 during the P0 author-binding pass.
    if update.author != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Incident update author must correspond to the committing agent".into(),
        ));
    }

    if update.content.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Update content cannot be empty".into(),
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

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_area() -> AffectedArea {
        AffectedArea {
            center_lat: 10.0,
            center_lon: 20.0,
            radius_km: 5.0,
            boundary: None,
            zones: vec![],
        }
    }

    fn valid_disaster(declared_by: AgentPubKey) -> Disaster {
        Disaster {
            id: "d-1".into(),
            disaster_type: DisasterType::Flood,
            title: "Flood".into(),
            description: "Rising water".into(),
            severity: SeverityLevel::Level2,
            declared_by,
            declared_at: Timestamp::from_micros(0),
            affected_area: valid_area(),
            status: DisasterStatus::Declared,
            estimated_affected: 100,
            coordination_lead: None,
        }
    }

    #[test]
    fn create_disaster_valid_when_declarer_matches_committer() {
        let d = valid_disaster(me());
        let result = validate_create_disaster(create_action(me()), d).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_disaster_forgery_rejected() {
        let d = valid_disaster(me());
        let result = validate_create_disaster(create_action(other_agent()), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_update(author: AgentPubKey) -> IncidentUpdate {
        IncidentUpdate {
            disaster_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            author,
            timestamp: Timestamp::from_micros(0),
            update_type: UpdateType::General,
            content: "status quo".into(),
        }
    }

    #[test]
    fn create_update_valid_when_author_matches_committer() {
        let u = valid_update(me());
        let result = validate_create_update(create_action(me()), u).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_update_forgery_rejected() {
        let u = valid_update(me());
        let result = validate_create_update(create_action(other_agent()), u).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_incident_update_update() {
        let u = valid_update(me());
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::IncidentUpdate(u)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

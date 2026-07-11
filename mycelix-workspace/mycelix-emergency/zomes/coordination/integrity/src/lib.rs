// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Coordination Integrity Zome
//! Teams, zones, SITREPs, and agent check-ins

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// A response team
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Team {
    pub id: String,
    pub name: String,
    pub team_type: TeamType,
    pub members: Vec<AgentPubKey>,
    pub lead: AgentPubKey,
    pub assigned_zone: Option<ActionHash>,
    pub status: TeamStatus,
}

/// Types of response teams
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TeamType {
    SearchAndRescue,
    Medical,
    Logistics,
    Communications,
    Shelter,
    Assessment,
    HazMat,
    Volunteer,
}

/// Team operational status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TeamStatus {
    Forming,
    Active,
    OnBreak,
    Disbanded,
}

/// A zone assignment for a team
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Assignment {
    pub team_hash: ActionHash,
    pub zone_hash: ActionHash,
    pub objective: String,
    pub assigned_at: Timestamp,
    pub assigned_by: AgentPubKey,
    pub status: AssignmentStatus,
}

/// Assignment status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AssignmentStatus {
    Active,
    Completed,
    Cancelled,
    Reassigned,
}

/// A situation report from the field
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SituationReport {
    pub team_hash: ActionHash,
    pub zone_hash: ActionHash,
    pub timestamp: Timestamp,
    pub conditions: String,
    pub casualties_found: u32,
    pub resources_needed: Vec<String>,
    pub hazards: Vec<String>,
    pub access_status: AccessStatus,
    pub synced: bool,
}

/// Access status to a zone
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AccessStatus {
    Open,
    Restricted,
    Blocked,
    Hazardous,
    Flooded,
    Collapsed,
}

/// An agent location check-in
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Checkpoint {
    pub agent: AgentPubKey,
    pub lat: f64,
    pub lon: f64,
    pub timestamp: Timestamp,
    pub status: AgentStatus,
    pub battery_level: Option<u8>,
    pub connectivity: ConnectivityStatus,
}

/// Connectivity status of a field agent
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ConnectivityStatus {
    Online,
    Intermittent,
    Offline,
}

/// Status of a field agent
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AgentStatus {
    Active,
    NeedsRelief,
    Injured,
    Evacuating,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Team(Team),
    Assignment(Assignment),
    SituationReport(SituationReport),
    Checkpoint(Checkpoint),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllTeams,
    ActiveTeams,
    TeamToAssignment,
    TeamToSitrep,
    ZoneToTeam,
    AgentToCheckpoint,
    AgentToTeam,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Team(team) => validate_create_team(action, team),
                EntryTypes::Assignment(assignment) => {
                    validate_create_assignment(action, assignment)
                }
                EntryTypes::SituationReport(sitrep) => validate_create_sitrep(action, sitrep),
                EntryTypes::Checkpoint(checkpoint) => {
                    validate_create_checkpoint(action, checkpoint)
                }
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
            LinkTypes::AllTeams => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ActiveTeams => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TeamToAssignment => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TeamToSitrep => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ZoneToTeam => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToCheckpoint => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToTeam => Ok(ValidateCallbackResult::Valid),
        },
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
            // unconditionally) -- the 11th confirmed instance of this exact
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
        EntryTypes::Team(team) => validate_update_team(action, team),
        EntryTypes::Assignment(_) => Ok(ValidateCallbackResult::Invalid(
            "Assignments are immutable".into(),
        )),
        EntryTypes::SituationReport(_) => Ok(ValidateCallbackResult::Invalid(
            "Situation reports are immutable".into(),
        )),
        EntryTypes::Checkpoint(_) => Ok(ValidateCallbackResult::Invalid(
            "Checkpoints are immutable; create a new one".into(),
        )),
    }
}

/// Validate a Team update (the coordinator's assign_to_zone sets
/// assigned_zone/status when assigning a team to a zone).
///
/// No author requirement: form_team has no author check at all (any agent
/// may form a team naming others as members/lead -- see the rationale on
/// validate_create_team below), and assign_to_zone is typically called by
/// an incident commander, not necessarily the team's original creator.
/// Content is restricted to assigned_zone/status; id/name/team_type/
/// members/lead are immutable.
fn validate_update_team(action: Update, team: Team) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Team = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original team not found".into()
        )))?;

    if team.id != original.id
        || team.name != original.name
        || team.team_type != original.team_type
        || team.members != original.members
        || team.lead != original.lead
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only assigned_zone/status can change on a team update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// No author-binding possible on `lead`/`members`: the coordinator's
/// form_team has no caller-identity check at all -- an incident commander
/// (or dispatch system) may legitimately form a team naming OTHER agents
/// as members/lead, none of whom need be the committer. Reviewed
/// 2026-07-09 during the P0 author-binding pass; case (b), same reasoning
/// as mycelix-care/matching's CareMatch.provider/requester. Unlike
/// care-plans' create_care_plan (which had a clear "recipient or
/// caregiver" invariant to mirror from update_plan_status), there's no
/// established authority model here to bind against -- not fixed.
fn validate_create_team(_action: Create, team: Team) -> ExternResult<ValidateCallbackResult> {
    if team.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Team ID cannot be empty".into(),
        ));
    }
    if team.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Team name cannot be empty".into(),
        ));
    }
    if team.members.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Team must have at least one member".into(),
        ));
    }
    if !team.members.contains(&team.lead) {
        return Ok(ValidateCallbackResult::Invalid(
            "Team lead must be a member".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_assignment(
    action: Create,
    assignment: Assignment,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's assign_to_zone already derives
    // assigned_by from agent_info(), so this is belt-and-suspenders
    // against a modified coordinator forging a victim agent as assigner.
    // Found + fixed 2026-07-09 during the P0 author-binding pass.
    if assignment.assigned_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Assignment assigned_by must correspond to the committing agent".into(),
        ));
    }

    if assignment.objective.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Assignment objective cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// No author-binding possible: SituationReport has no self-declared
/// reporter/agent field at all (team_hash/zone_hash reference other
/// entities, not the committer). Reviewed 2026-07-09 during the P0
/// author-binding pass; case (a). Adding a reporter field would be a
/// schema change, out of scope for this pass.
fn validate_create_sitrep(
    _action: Create,
    sitrep: SituationReport,
) -> ExternResult<ValidateCallbackResult> {
    if sitrep.conditions.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "SITREP conditions cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_checkpoint(
    action: Create,
    checkpoint: Checkpoint,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's checkin already derives `agent`
    // from agent_info(), so this is belt-and-suspenders against a
    // modified coordinator forging a victim agent's location check-in.
    // Found + fixed 2026-07-09 during the P0 author-binding pass.
    if checkpoint.agent != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Checkpoint agent must correspond to the committing agent".into(),
        ));
    }

    if checkpoint.lat < -90.0 || checkpoint.lat > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be between -90 and 90".into(),
        ));
    }
    if checkpoint.lon < -180.0 || checkpoint.lon > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be between -180 and 180".into(),
        ));
    }
    if let Some(battery) = checkpoint.battery_level {
        if battery > 100 {
            return Ok(ValidateCallbackResult::Invalid(
                "Battery level cannot exceed 100".into(),
            ));
        }
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

    fn valid_assignment(assigned_by: AgentPubKey) -> Assignment {
        Assignment {
            team_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            zone_hash: ActionHash::from_raw_36(vec![3u8; 36]),
            objective: "clear debris".into(),
            assigned_at: Timestamp::from_micros(0),
            assigned_by,
            status: AssignmentStatus::Active,
        }
    }

    #[test]
    fn create_assignment_valid_when_assigner_matches_committer() {
        let a = valid_assignment(me());
        let result = validate_create_assignment(create_action(me()), a).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_assignment_forgery_rejected() {
        let a = valid_assignment(me());
        let result = validate_create_assignment(create_action(other_agent()), a).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_checkpoint(agent: AgentPubKey) -> Checkpoint {
        Checkpoint {
            agent,
            lat: 10.0,
            lon: 20.0,
            timestamp: Timestamp::from_micros(0),
            status: AgentStatus::Active,
            battery_level: Some(80),
            connectivity: ConnectivityStatus::Online,
        }
    }

    #[test]
    fn create_checkpoint_valid_when_agent_matches_committer() {
        let c = valid_checkpoint(me());
        let result = validate_create_checkpoint(create_action(me()), c).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_checkpoint_forgery_rejected() {
        let c = valid_checkpoint(me());
        let result = validate_create_checkpoint(create_action(other_agent()), c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_assignment_update() {
        let a = valid_assignment(me());
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::Assignment(a)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_sitrep_update() {
        let s = SituationReport {
            team_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            zone_hash: ActionHash::from_raw_36(vec![3u8; 36]),
            timestamp: Timestamp::from_micros(0),
            conditions: "stable".into(),
            casualties_found: 0,
            resources_needed: vec![],
            hazards: vec![],
            access_status: AccessStatus::Open,
            synced: false,
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::SituationReport(s))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

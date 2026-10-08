// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Machines Integrity Zome
//!
//! Entry types and validation for the machine registry.

use hdi::prelude::*;
use manufacturing_common::{MachineStatus, MachineType};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineEntry {
    pub name: String,
    pub machine_type: MachineType,
    pub capabilities: Vec<String>,
    pub location: String,
    pub max_throughput_per_hour: u32,
    pub status: MachineStatus,
    pub current_work_order: Option<ActionHash>,
    #[serde(default)]
    pub last_status_authority_hash: Option<ActionHash>,
    pub registered_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineControllerAuthorityEntry {
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineStatusLog {
    pub machine_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub previous_status: MachineStatus,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    pub changed_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Machine(MachineEntry),
    StatusLog(MachineStatusLog),
    MachineControllerAuthority(MachineControllerAuthorityEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMachines,
    TypeToMachines,
    MachineToStatusLog,
    MachineToAuthorities,
    LocationToMachines,
}

/// **P0 author-binding pass, 2026-07-09**: no identity field exists on
/// either entry (case a -- shared machine registry, not per-agent-owned).
/// What IS fixed: updates were previously routed through the wide-open
/// catch-all `_ => Valid` -- `update_machine_status`'s `update_entry`
/// call was accepted with zero validation, so a modified coordinator
/// could silently rewrite ANY field (name, machine_type, capabilities,
/// location, max_throughput_per_hour), not just the intended
/// status/current_work_order, and could skip the state-machine
/// transition check entirely. Fixed via must_get content-restriction
/// plus re-running `can_transition_to`.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => {
            match app_entry {
                EntryTypes::MachineControllerAuthority(authority) => {
                    validate_create_authority(action, authority)
                }
                EntryTypes::StatusLog(log) => validate_create_status_log(action, log),
                other => validate_create_entry(other),
            }
        }
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            action,
            ..
        }) => validate_update_entry(original_action_hash, action, app_entry),
        FlatOp::RegisterUpdate(OpUpdate::Entry {
            app_entry, action, ..
        }) => validate_update_entry(action.original_action_address, action, app_entry),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine registry records are immutable".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_authority(
    action: TypedAction<CreateData>,
    authority: MachineControllerAuthorityEntry,
) -> ExternResult<ValidateCallbackResult> {
    if authority.valid_until < authority.valid_from {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority validity window is inverted".into(),
        ));
    }
    if action.timestamp() < authority.valid_from || action.timestamp() > authority.valid_until {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority must be valid at issuance time".into(),
        ));
    }

    let machine_record = must_get_valid_record(authority.machine_hash.clone())?;
    let machine: Option<MachineEntry> = machine_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if machine.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority references a non-machine record".into(),
        ));
    }
    if machine_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "only the machine registrant may issue controller authority".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_status_log(
    action: TypedAction<CreateData>,
    log: MachineStatusLog,
) -> ExternResult<ValidateCallbackResult> {
    if !log.previous_status.can_transition_to(&log.new_status) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid machine transition: {:?} -> {:?}",
            log.previous_status, log.new_status
        )));
    }

    let authority_record = must_get_valid_record(log.authority_hash.clone())?;
    let authority: Option<MachineControllerAuthorityEntry> = authority_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(authority) = authority else {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status authority reference is not an authority record".into(),
        ));
    };

    if authority.machine_hash != log.machine_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status authority is bound to a different machine".into(),
        ));
    }
    if action.author() != authority.controller_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status action author is not an authorized controller".into(),
        ));
    }
    if action.timestamp() < authority.valid_from || action.timestamp() > authority.valid_until {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status action falls outside controller authority validity".into(),
        ));
    }
    if log.changed_at != action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status changed_at must equal the action timestamp".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_entry(entry: EntryTypes) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Machine(m) => {
            if m.name.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine name is required".into(),
                ));
            }
            if m.location.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine location is required".into(),
                ));
            }
            if m.max_throughput_per_hour == 0 {
                return Ok(ValidateCallbackResult::Invalid(
                    "max_throughput_per_hour must be > 0".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusLog(log) => {
            if !log.previous_status.can_transition_to(&log.new_status) {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Invalid machine transition: {:?} -> {:?}",
                    log.previous_status, log.new_status
                )));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::MachineControllerAuthority(_) => unreachable!(),
    }
}

fn resolve_machine_root_action_hash(
    start: ActionHash,
) -> ExternResult<ActionHash> {
    let mut current = start;
    for _ in 0..4096 {
        let record = must_get_valid_record(current.clone())?;
        match record.action() {
            Action::Create(_) => return Ok(current),
            Action::Update(update) => {
                current = update.original_action_address.clone();
            }
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "machine update lineage terminated at a non-entry action".into(),
                )));
            }
        }
    }

    Err(wasm_error!(WasmErrorInner::Guest(
        "machine update lineage exceeds safety bound".into(),
    )))
}

fn validate_update_entry(
    original_action_hash: ActionHash,
    action: TypedAction<UpdateData>,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Machine(m) => {
            let original_record = must_get_valid_record(original_action_hash)?;
            let original: MachineEntry = original_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Original machine not found".into()
                )))?;

            if m.name != original.name
                || m.machine_type != original.machine_type
                || m.capabilities != original.capabilities
                || m.location != original.location
                || m.max_throughput_per_hour != original.max_throughput_per_hour
                || m.registered_at != original.registered_at
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only status/current_work_order/last_status_authority_hash can change on a machine update".into(),
                ));
            }

            if m.status != original.status {
                let Some(authority_hash) = m.last_status_authority_hash.clone() else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine status changes require controller authority".into(),
                    ));
                };
                let authority_record = must_get_valid_record(authority_hash)?;
                let authority: Option<MachineControllerAuthorityEntry> = authority_record
                    .entry()
                    .to_app_option()
                    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                let Some(authority) = authority else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine status authority reference is not an authority record".into(),
                    ));
                };
                let machine_root = resolve_machine_root_action_hash(
                    original_action_hash.clone(),
                )?;
                if authority.machine_hash != machine_root {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine status authority is bound to a different machine".into(),
                    ));
                }
                if action.author() != authority.controller_agent {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine update author is not an authorized controller".into(),
                    ));
                }
                if action.timestamp() < authority.valid_from
                    || action.timestamp() > authority.valid_until
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine update falls outside controller authority validity".into(),
                    ));
                }
                if !original.status.can_transition_to(&m.status) {
                    return Ok(ValidateCallbackResult::Invalid(format!(
                        "Invalid machine transition: {:?} -> {:?}",
                        original.status, m.status
                    )));
                }
            }

            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusLog(_) => Ok(ValidateCallbackResult::Invalid(
            "Status log records are immutable".into(),
        )),
        EntryTypes::MachineControllerAuthority(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine controller authorities are immutable".into(),
        )),
    }
}

#[cfg(test)]
mod content_restriction_tests {
    use super::*;

    fn valid_machine() -> MachineEntry {
        MachineEntry {
            name: "Mill-1".into(),
            machine_type: MachineType::CNC3Axis,
            capabilities: vec![],
            location: "Bay 1".into(),
            max_throughput_per_hour: 10,
            status: MachineStatus::Available,
            current_work_order: None,
            last_status_authority_hash: None,
            registered_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_machine_requires_name_and_location() {
        let mut m = valid_machine();
        m.name = "".into();
        let result = validate_create_entry(EntryTypes::Machine(m)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_machine_requires_positive_throughput() {
        let mut m = valid_machine();
        m.max_throughput_per_hour = 0;
        let result = validate_create_entry(EntryTypes::Machine(m)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_machine_valid() {
        let result = validate_create_entry(EntryTypes::Machine(valid_machine())).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn status_log_rejects_invalid_transition() {
        let log = MachineStatusLog {
            machine_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            authority_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            previous_status: MachineStatus::Offline,
            new_status: MachineStatus::Running,
            work_order_hash: None,
            changed_at: Timestamp::from_micros(0),
        };
        let result = validate_create_entry(EntryTypes::StatusLog(log)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_entry calls must_get_valid_record, which requires a
    // live HDI host and can't run in a plain unit test -- matching the
    // established pattern from every other zome's update validator this
    // pass.
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Machines Coordinator Zome
//!
//! Machine registry and status management for the manufacturing floor.

use hdk::prelude::*;
use machines_integrity::*;
use manufacturing_common::{MachineStatus, MachineType};

// ============================================================================
// Input types
// ============================================================================

#[derive(Serialize, Deserialize, Debug)]
pub struct RegisterMachineInput {
    pub name: String,
    pub machine_type: MachineType,
    pub capabilities: Vec<String>,
    pub location: String,
    pub max_throughput_per_hour: u32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateMachineStatusInput {
    pub machine_hash: ActionHash,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct GetMachinesByTypeInput {
    pub machine_type: MachineType,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineStateResolution {
    NotFound,
    Resolved {
        status: MachineStatus,
        head_action: ActionHash,
    },
    Ambiguous {
        statuses: Vec<MachineStatus>,
        head_actions: Vec<ActionHash>,
    },
    Deleted,
}

/// Resolve the machine's current state from the full update graph rooted at
/// the original machine action.
///
/// Holochain does not define a global "latest" record. An action-addressed
/// get_details call exposes direct updates, while entry-level metadata exposes
/// the relationships for the entry. We therefore walk update edges to terminal
/// live heads and refuse to select among multiple heads.
fn resolve_machine_state_from_action(
    root: ActionHash,
) -> ExternResult<MachineStateResolution> {
    let mut pending = vec![root];
    let mut visited = std::collections::HashSet::new();
    let mut heads: Vec<(ActionHash, MachineStatus)> = Vec::new();
    let mut saw_deleted = false;

    while let Some(action_hash) = pending.pop() {
        if !visited.insert(action_hash.clone()) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "machine update graph contains a cycle".into(),
            )));
        }
        if visited.len() > 4096 {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "machine update graph exceeds safety bound".into(),
            )));
        }

        let details = get_details(action_hash.clone(), GetOptions::default())?;
        let Some(Details::Record(record_details)) = details else {
            return Ok(MachineStateResolution::NotFound);
        };

        if !record_details.deletes.is_empty() {
            saw_deleted = true;
            continue;
        }

        if !record_details.updates.is_empty() {
            for update in record_details.updates {
                pending.push(update.as_hash().clone());
            }
            continue;
        }

        let machine: MachineEntry = record_details
            .record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "machine state head has no entry".into(),
            )))?;

        heads.push((action_hash, machine.status));
    }

    if heads.is_empty() {
        return Ok(if saw_deleted {
            MachineStateResolution::Deleted
        } else {
            MachineStateResolution::NotFound
        });
    }

    if heads.len() == 1 {
        let (head_action, status) = heads.remove(0);
        return Ok(MachineStateResolution::Resolved {
            status,
            head_action,
        });
    }

    heads.sort_by(|a, b| a.0.to_string().cmp(&b.0.to_string()));
    Ok(MachineStateResolution::Ambiguous {
        statuses: heads.iter().map(|(_, status)| status.clone()).collect(),
        head_actions: heads.into_iter().map(|(hash, _)| hash).collect(),
    })
}

#[hdk_extern]
pub fn get_current_machine_state(machine_hash: ActionHash) -> ExternResult<MachineStateResolution> {
    resolve_machine_state_from_action(machine_hash)
}

/// Update machine status (e.g., Available -> Running).
#[hdk_extern]
pub fn update_machine_status(input: UpdateMachineStatusInput) -> ExternResult<ActionHash> {
    let record = get(input.machine_hash.clone(), GetOptions::default())?.ok_or(
        wasm_error!(WasmErrorInner::Guest("Machine not found".to_string())),
    )?;

    let machine: MachineEntry = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Could not deserialize machine".to_string()
        )))?;

    if !machine.status.can_transition_to(&input.new_status) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid machine transition: {:?} -> {:?}",
            machine.status, input.new_status
        ))));
    }

    let now = sys_time()?;

    // Log the status change
    let log = MachineStatusLog {
        machine_hash: input.machine_hash.clone(),
        previous_status: machine.status.clone(),
        new_status: input.new_status.clone(),
        work_order_hash: input.work_order_hash.clone(),
        changed_at: now,
    };
    let log_hash = create_entry(EntryTypes::StatusLog(log))?;
    create_link(
        input.machine_hash.clone(),
        log_hash,
        LinkTypes::MachineToStatusLog,
        (),
    )?;

    // Update the machine entry
    let updated = MachineEntry {
        status: input.new_status,
        current_work_order: input.work_order_hash,
        ..machine
    };
    update_entry(input.machine_hash, EntryTypes::Machine(updated))
}

/// Get all machines that are currently Available.
#[hdk_extern]
pub fn get_available_machines(_: ()) -> ExternResult<Vec<Record>> {
    let all_path = Path::from("all_machines").typed(LinkTypes::AllMachines)?;
    let links = get_links(
        GetLinksInputBuilder::try_new(all_path.path_entry_hash()?, LinkTypes::AllMachines)?.build(),
    )?;

    let mut available = Vec::new();
    for link in links {
        if let Some(hash) = link.target.into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                if let Some(machine) = record
                    .entry()
                    .to_app_option::<MachineEntry>()
                    .ok()
                    .flatten()
                {
                    if machine.status == MachineStatus::Available {
                        available.push(record);
                    }
                }
            }
        }
    }
    Ok(available)
}

/// Get machines by type via type anchor links.
#[hdk_extern]
pub fn get_machines_by_type(input: GetMachinesByTypeInput) -> ExternResult<Vec<Link>> {
    let tag = machine_type_tag(&input.machine_type);
    let type_path = Path::from(format!("machine_type/{tag}")).typed(LinkTypes::TypeToMachines)?;
    get_links(
        GetLinksInputBuilder::try_new(type_path.path_entry_hash()?, LinkTypes::TypeToMachines)?
            .build(),
    )
}

/// Produce a stable string tag for each machine type variant.
fn machine_type_tag(mt: &MachineType) -> String {
    match mt {
        MachineType::FDM => "fdm".to_string(),
        MachineType::SLA => "sla".to_string(),
        MachineType::SLS => "sls".to_string(),
        MachineType::CNC3Axis => "cnc3".to_string(),
        MachineType::CNC5Axis => "cnc5".to_string(),
        MachineType::LaserCutter => "laser".to_string(),
        MachineType::Lathe => "lathe".to_string(),
        MachineType::Assembly => "assembly".to_string(),
        MachineType::Custom(s) => format!("custom_{s}"),
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_input_serde() {
        let input = RegisterMachineInput {
            name: "CNC Mill #3".to_string(),
            machine_type: MachineType::CNC3Axis,
            capabilities: vec!["milling".to_string(), "drilling".to_string()],
            location: "Bay 2".to_string(),
            max_throughput_per_hour: 12,
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: RegisterMachineInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.name, "CNC Mill #3");
        assert_eq!(back.machine_type, MachineType::CNC3Axis);
    }

    #[test]
    fn test_machine_type_tag_variants() {
        assert_eq!(machine_type_tag(&MachineType::FDM), "fdm");
        assert_eq!(machine_type_tag(&MachineType::CNC5Axis), "cnc5");
        assert_eq!(machine_type_tag(&MachineType::LaserCutter), "laser");
        assert_eq!(
            machine_type_tag(&MachineType::Custom("Waterjet".to_string())),
            "custom_Waterjet"
        );
    }

    #[test]
    fn test_machine_state_resolution_serde() {
        let resolved = MachineStateResolution::Resolved {
            status: MachineStatus::Available,
            head_action: ActionHash::from_raw_36(vec![3; 36]),
        };
        let json = serde_json::to_string(&resolved).unwrap();
        let back: MachineStateResolution = serde_json::from_str(&json).unwrap();
        assert_eq!(back, resolved);

        let ambiguous = MachineStateResolution::Ambiguous {
            statuses: vec![MachineStatus::Available, MachineStatus::Running],
            head_actions: vec![
                ActionHash::from_raw_36(vec![4; 36]),
                ActionHash::from_raw_36(vec![5; 36]),
            ],
        };
        let json = serde_json::to_string(&ambiguous).unwrap();
        let back: MachineStateResolution = serde_json::from_str(&json).unwrap();
        assert_eq!(back, ambiguous);
    }

    #[test]
    fn test_update_status_input_serde() {
        let input = UpdateMachineStatusInput {
            machine_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            new_status: MachineStatus::Running,
            work_order_hash: Some(ActionHash::from_raw_36(vec![1u8; 36])),
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: UpdateMachineStatusInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.new_status, MachineStatus::Running);
        assert!(back.work_order_hash.is_some());
    }

    #[test]
    fn test_machine_status_all_variants_serde() {
        for s in [
            MachineStatus::Available,
            MachineStatus::Running,
            MachineStatus::Maintenance,
            MachineStatus::Offline,
        ] {
            let json = serde_json::to_string(&s).unwrap();
            let back: MachineStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(back, s);
        }
    }

    #[test]
    fn test_machine_entry_serde() {
        let entry = MachineEntry {
            name: "Lathe #1".to_string(),
            machine_type: MachineType::Lathe,
            capabilities: vec!["turning".to_string()],
            location: "Bay 3".to_string(),
            max_throughput_per_hour: 8,
            status: MachineStatus::Available,
            current_work_order: None,
            registered_at: Timestamp::from_micros(0),
        };
        let json = serde_json::to_string(&entry).unwrap();
        assert!(json.contains("Lathe #1"));
        assert!(json.contains("\"status\":\"Available\""));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Operations Coordinator Zome
//!
//! CRUD for manufacturing operations and routing sequences.

use hdk::prelude::*;
use operations_integrity::*;

// ============================================================================
// Input types
// ============================================================================

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateOperationInput {
    pub name: String,
    pub description: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub tooling: Option<String>,
    #[serde(default)]
    pub process_binding: Option<ProcessBindingV1>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateRoutingInput {
    pub design_id: String,
    pub revision: String,
    pub steps: Vec<RoutingStepInput>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateProcessRegistryEntryInput {
    pub process_definition_ref: ProcessDefinitionRefV1,
    pub display_label: String,
    #[serde(default)]
    pub tags: Vec<String>,
    pub status: ProcessRegistryStatusV1,
    #[serde(default)]
    pub supersedes_process_ref: Option<ProcessDefinitionRefV1>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RoutingStepInput {
    pub sequence: u32,
    pub operation_name: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub description: String,
    #[serde(default)]
    pub process_binding: Option<ProcessBindingV1>,
}

// ============================================================================
// Extern functions
// ============================================================================

/// Create a standalone operation definition.
#[hdk_extern]
pub fn create_operation(input: CreateOperationInput) -> ExternResult<ActionHash> {
    if input.name.is_empty() || input.name.len() > 200 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "operation name must be 1-200 characters".to_string()
        )));
    }

    let entry = OperationEntry {
        name: input.name,
        description: input.description,
        machine_type: input.machine_type,
        setup_time_min: input.setup_time_min,
        cycle_time_min: input.cycle_time_min,
        tooling: input.tooling,
        process_binding: input.process_binding,
    };

    let action_hash = create_entry(EntryTypes::Operation(entry))?;

    let all_path = Path::from("all_operations").typed(LinkTypes::AllOperations)?;
    all_path.ensure()?;
    create_link(
        all_path.path_entry_hash()?,
        action_hash.clone(),
        LinkTypes::AllOperations,
        (),
    )?;

    Ok(action_hash)
}

/// Create a routing sequence (ordered list of operations for a design).
#[hdk_extern]
pub fn create_routing(input: CreateRoutingInput) -> ExternResult<ActionHash> {
    if input.design_id.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "design_id is required".to_string()
        )));
    }
    if input.steps.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "routing must have at least one step".to_string()
        )));
    }

    // Verify ascending sequence order
    for w in input.steps.windows(2) {
        if w[0].sequence >= w[1].sequence {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "steps must be in ascending sequence order".to_string()
            )));
        }
    }

    let steps: Vec<RoutingStepEntry> = input
        .steps
        .into_iter()
        .map(|s| RoutingStepEntry {
            sequence: s.sequence,
            operation_name: s.operation_name,
            machine_type: s.machine_type,
            setup_time_min: s.setup_time_min,
            cycle_time_min: s.cycle_time_min,
            description: s.description,
            process_binding: s.process_binding,
        })
        .collect();

    let now = sys_time()?;
    let entry = RoutingEntry {
        design_id: input.design_id.clone(),
        revision: input.revision,
        steps,
        created_at: now,
    };

    let action_hash = create_entry(EntryTypes::Routing(entry))?;

    let design_path =
        Path::from(format!("design/{}", input.design_id)).typed(LinkTypes::DesignToRouting)?;
    design_path.ensure()?;
    create_link(
        design_path.path_entry_hash()?,
        action_hash.clone(),
        LinkTypes::DesignToRouting,
        (),
    )?;

    Ok(action_hash)
}

/// Register an immutable Mycelix index to an exact Symthaea process definition.
/// This stores references/display metadata only; it does not verify the external
/// object or qualify the process.
#[hdk_extern]
pub fn register_process_definition(
    input: CreateProcessRegistryEntryInput,
) -> ExternResult<ActionHash> {
    let now = sys_time()?;
    let entry = ProcessRegistryEntry {
        process_definition_ref: input.process_definition_ref,
        display_label: input.display_label,
        tags: input.tags,
        status: input.status,
        supersedes_process_ref: input.supersedes_process_ref,
        registered_at: now,
    };
    let action_hash = create_entry(EntryTypes::ProcessRegistry(entry))?;
    let all_path = Path::from("all_process_registry_entries")
        .typed(LinkTypes::AllProcessRegistryEntries)?;
    all_path.ensure()?;
    create_link(
        all_path.path_entry_hash()?,
        action_hash.clone(),
        LinkTypes::AllProcessRegistryEntries,
        (),
    )?;
    Ok(action_hash)
}

/// Get a routing by action hash.
#[hdk_extern]
pub fn get_routing(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_operation_input_serde() {
        let input = CreateOperationInput {
            name: "Mill profile".to_string(),
            description: "Machine final profile on CNC".to_string(),
            machine_type: "CNC".to_string(),
            setup_time_min: 15,
            cycle_time_min: 8,
            tooling: Some("EM-10mm".to_string()),
            process_binding: None,
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateOperationInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.name, "Mill profile");
        assert_eq!(back.cycle_time_min, 8);
        assert!(back.process_binding.is_none());

        // Pre-001A clients can omit the newly additive field.
        let legacy = r#"{"name":"Legacy","description":"","machine_type":"CNC","setup_time_min":1,"cycle_time_min":2,"tooling":null}"#;
        let decoded: CreateOperationInput = serde_json::from_str(legacy).unwrap();
        assert!(decoded.process_binding.is_none());
    }

    #[test]
    fn test_routing_step_ordering() {
        let steps = vec![
            RoutingStepInput {
                sequence: 10,
                operation_name: "Cut".to_string(),
                machine_type: "Saw".to_string(),
                setup_time_min: 5,
                cycle_time_min: 2,
                description: "Cut blank".to_string(),
                process_binding: None,
            },
            RoutingStepInput {
                sequence: 20,
                operation_name: "Mill".to_string(),
                machine_type: "CNC".to_string(),
                setup_time_min: 15,
                cycle_time_min: 8,
                description: "Mill profile".to_string(),
                process_binding: None,
            },
        ];
        // Verify ascending order
        for w in steps.windows(2) {
            assert!(w[0].sequence < w[1].sequence);
        }
    }

    #[test]
    fn test_routing_input_serde() {
        let input = CreateRoutingInput {
            design_id: "BRACKET-v2".to_string(),
            revision: "A".to_string(),
            steps: vec![RoutingStepInput {
                sequence: 10,
                operation_name: "Cut".to_string(),
                machine_type: "Saw".to_string(),
                setup_time_min: 5,
                cycle_time_min: 2,
                description: "Cut blank".to_string(),
                process_binding: None,
            }],
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateRoutingInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.design_id, "BRACKET-v2");
        assert_eq!(back.steps.len(), 1);
        assert!(back.steps[0].process_binding.is_none());

        let legacy = r#"{"sequence":10,"operation_name":"Cut","machine_type":"Saw","setup_time_min":5,"cycle_time_min":2,"description":"Cut blank"}"#;
        let decoded: RoutingStepInput = serde_json::from_str(legacy).unwrap();
        assert!(decoded.process_binding.is_none());
    }
}

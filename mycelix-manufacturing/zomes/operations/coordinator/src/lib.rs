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
pub struct CreateProcessRecipeInput {
    pub recipe_id: String,
    pub revision: String,
    pub process_family: String,
    pub payload_hash: String,
    pub parameter_schema: String,
    pub external_reference: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateCapabilityRequirementInput {
    pub requirement_id: String,
    pub revision: String,
    pub requirement: manufacturing_common::CapabilityRequirement,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateOperationInput {
    pub name: String,
    pub description: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub tooling: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateRoutingInput {
    pub design_id: String,
    pub revision: String,
    pub steps: Vec<RoutingStepInput>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct InspectionCriterionProjection {
    requirement_id: String,
    revision: String,
    characteristic: String,
    unit: String,
    lower_bound: Option<f64>,
    upper_bound: Option<f64>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RoutingStepInput {
    pub sequence: u32,
    pub process_recipe_hash: Option<ActionHash>,
    pub capability_requirement_hash: Option<ActionHash>,
    pub required_inspection_criterion_hashes: Vec<ActionHash>,
    pub operation_name: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub description: String,
}

// ============================================================================
// Extern functions
// ============================================================================

/// Create an immutable process recipe for routing references.
#[hdk_extern]
pub fn create_process_recipe(input: CreateProcessRecipeInput) -> ExternResult<ActionHash> {
    if input.recipe_id.is_empty() || input.recipe_id.len() > 200 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "recipe_id must be 1-200 characters".into(),
        )));
    }
    if input.revision.is_empty() || input.revision.len() > 100 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "recipe revision must be 1-100 characters".into(),
        )));
    }

    let hash = create_entry(EntryTypes::ProcessRecipe(ProcessRecipeEntry {
        schema_id: "mycelix-manufacturing-process-recipe-v1".into(),
        recipe_id: input.recipe_id,
        revision: input.revision,
        process_family: input.process_family,
        payload_hash: input.payload_hash,
        parameter_schema: input.parameter_schema,
        external_reference: input.external_reference,
        created_at: sys_time()?,
    }))?;

    let path = Path::from("all_process_recipes")
        .typed(LinkTypes::AllProcessRecipes)?;
    path.ensure()?;
    create_link(
        path.path_entry_hash()?,
        hash.clone(),
        LinkTypes::AllProcessRecipes,
        (),
    )?;
    Ok(hash)
}

/// Create an immutable typed capability requirement for routing references.
#[hdk_extern]
pub fn create_capability_requirement(
    input: CreateCapabilityRequirementInput,
) -> ExternResult<ActionHash> {
    if input.requirement_id.is_empty() || input.requirement_id.len() > 200 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "requirement_id must be 1-200 characters".into(),
        )));
    }
    if input.revision.is_empty() || input.revision.len() > 100 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "revision must be 1-100 characters".into(),
        )));
    }

    let hash = create_entry(EntryTypes::CapabilityRequirement(
        CapabilityRequirementEntry {
            requirement_id: input.requirement_id,
            revision: input.revision,
            requirement: input.requirement,
            created_at: sys_time()?,
        },
    ))?;

    let path = Path::from("all_capability_requirements")
        .typed(LinkTypes::AllCapabilityRequirements)?;
    path.ensure()?;
    create_link(
        path.path_entry_hash()?,
        hash.clone(),
        LinkTypes::AllCapabilityRequirements,
        (),
    )?;
    Ok(hash)
}

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

    for step in &input.steps {
        if let Some(recipe_hash) = step.process_recipe_hash.clone() {
            let Some(record) = get(recipe_hash, GetOptions::default())? else {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "routing process recipe not found".to_string()
                )));
            };
            let recipe: Option<ProcessRecipeEntry> = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            if recipe.is_none() {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "routing process recipe reference is not a process recipe".to_string()
                )));
            }
        }

        let mut step_criteria = std::collections::HashSet::new();
        for criterion_hash in &step.required_inspection_criterion_hashes {
            if !step_criteria.insert(criterion_hash.clone()) {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "routing step cannot require the same inspection criterion more than once".to_string()
                )));
            }

            let response = call(
                CallTargetCell::Local,
                ZomeName::from("execution"),
                FunctionName::from("get_inspection_criterion"),
                None,
                ExternIO::encode(criterion_hash.clone())?,
            )?;
            match response {
                ZomeCallResponse::Ok(data) => {
                    let criterion: Option<InspectionCriterionProjection> =
                        data.decode().map_err(|e| {
                            wasm_error!(WasmErrorInner::Guest(format!(
                                "failed to decode inspection criterion: {e}"
                            )))
                        })?;
                    if criterion.is_none() {
                        return Err(wasm_error!(WasmErrorInner::Guest(
                            "routing inspection criterion not found or is not an inspection criterion".to_string()
                        )));
                    }
                }
                _ => {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "routing inspection criterion lookup failed".to_string()
                    )));
                }
            }
        }
    }

    let steps: Vec<RoutingStepEntry> = input
        .steps
        .into_iter()
        .map(|s| RoutingStepEntry {
            sequence: s.sequence,
            process_recipe_hash: s.process_recipe_hash,
            capability_requirement_hash: s.capability_requirement_hash,
            required_inspection_criterion_hashes: s.required_inspection_criterion_hashes,
            operation_name: s.operation_name,
            machine_type: s.machine_type,
            setup_time_min: s.setup_time_min,
            cycle_time_min: s.cycle_time_min,
            description: s.description,
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

    for step in &entry.steps {
        if let Some(recipe_hash) = step.process_recipe_hash.clone() {
            create_link(
                action_hash.clone(),
                recipe_hash,
                LinkTypes::RoutingToProcessRecipes,
                (),
            )?;
        }
        for criterion_hash in &step.required_inspection_criterion_hashes {
            create_link(
                action_hash.clone(),
                criterion_hash.clone(),
                LinkTypes::RoutingToInspectionCriteria,
                (),
            )?;
        }
    }

    Ok(action_hash)
}

/// Get a typed capability requirement by action hash.
#[hdk_extern]
pub fn get_capability_requirement(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// List all typed capability requirements.
#[hdk_extern]
pub fn list_capability_requirements(_: ()) -> ExternResult<Vec<Link>> {
    let path = Path::from("all_capability_requirements")
        .typed(LinkTypes::AllCapabilityRequirements)?;
    get_links(
        GetLinksInputBuilder::try_new(
            path.path_entry_hash()?,
            LinkTypes::AllCapabilityRequirements,
        )?
        .build(),
    )
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
    fn test_process_recipe_input_serde() {
        let input = CreateProcessRecipeInput {
            recipe_id: "RECIPE-001".into(),
            revision: "A".into(),
            process_family: "milling".into(),
            payload_hash: "sha256:abc".into(),
            parameter_schema: "schema-v1".into(),
            external_reference: Some("vendor:v1".into()),
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateProcessRecipeInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.recipe_id, "RECIPE-001");
        assert_eq!(back.revision, "A");
        assert_eq!(back.process_family, "milling");
    }

    #[test]
    fn test_capability_requirement_input_serde() {
        let input = CreateCapabilityRequirementInput {
            requirement_id: "CAPREQ-001".into(),
            revision: "A".into(),
            requirement: manufacturing_common::CapabilityRequirement {
                process_family: "milling".into(),
                material_class: "aluminum".into(),
                envelope_x_mm: Some(400),
                envelope_y_mm: Some(200),
                envelope_z_mm: Some(100),
                tolerance_um: Some(25),
                required_protocols: vec!["opcua".into()],
            },
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateCapabilityRequirementInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.requirement_id, "CAPREQ-001");
        assert_eq!(back.revision, "A");
        assert_eq!(back.requirement.process_family, "milling");
    }

    #[test]
    fn test_create_operation_input_serde() {
        let input = CreateOperationInput {
            name: "Mill profile".to_string(),
            description: "Machine final profile on CNC".to_string(),
            machine_type: "CNC".to_string(),
            setup_time_min: 15,
            cycle_time_min: 8,
            tooling: Some("EM-10mm".to_string()),
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateOperationInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.name, "Mill profile");
        assert_eq!(back.cycle_time_min, 8);
    }

    #[test]
    fn test_routing_step_supports_typed_capability_requirement() {
        let input = RoutingStepInput {
            sequence: 10,
            capability_requirement_hash: Some(ActionHash::from_raw_36(vec![7; 36])),
            operation_name: "Mill".to_string(),
            machine_type: "CNC".to_string(),
            setup_time_min: 15,
            cycle_time_min: 8,
            description: "Mill profile".to_string(),
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: RoutingStepInput = serde_json::from_str(&json).unwrap();
        assert!(back.capability_requirement_hash.is_some());
    }

    #[test]
    fn test_routing_step_ordering() {
        let steps = vec![
            RoutingStepInput {
                sequence: 10,
                process_recipe_hash: None,
                process_recipe_hash: None,
                capability_requirement_hash: None,
                required_inspection_criterion_hashes: vec![],
                operation_name: "Cut".to_string(),
                machine_type: "Saw".to_string(),
                setup_time_min: 5,
                cycle_time_min: 2,
                description: "Cut blank".to_string(),
            },
            RoutingStepInput {
                sequence: 20,
                process_recipe_hash: None,
                capability_requirement_hash: None,
                required_inspection_criterion_hashes: vec![],
                operation_name: "Mill".to_string(),
                machine_type: "CNC".to_string(),
                setup_time_min: 15,
                cycle_time_min: 8,
                description: "Mill profile".to_string(),
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
                process_recipe_hash: None,
                capability_requirement_hash: None,
                required_inspection_criterion_hashes: vec![],
                operation_name: "Cut".to_string(),
                machine_type: "Saw".to_string(),
                setup_time_min: 5,
                cycle_time_min: 2,
                description: "Cut blank".to_string(),
            }],
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateRoutingInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.design_id, "BRACKET-v2");
        assert_eq!(back.steps.len(), 1);
    }
}

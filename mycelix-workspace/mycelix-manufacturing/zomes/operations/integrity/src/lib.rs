// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Operations Integrity Zome
//!
//! Entry types and validation for manufacturing operations and routing sequences.

use hdi::prelude::*;
pub use manufacturing_common::{ProcessBindingV1, ProcessDefinitionRefV1};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct OperationEntry {
    pub name: String,
    pub description: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub tooling: Option<String>,
    /// Absent on legacy records. When present, this exact external binding
    /// supersedes string matching for process semantics; it is not execution authority.
    #[serde(default)]
    pub process_binding: Option<ProcessBindingV1>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RoutingEntry {
    pub design_id: String,
    pub revision: String,
    pub steps: Vec<RoutingStepEntry>,
    pub created_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, SerializedBytes)]
pub struct RoutingStepEntry {
    pub sequence: u32,
    pub operation_name: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub description: String,
    /// Absent on legacy routing steps; legacy operation_name/machine_type
    /// remain descriptive compatibility fields only.
    #[serde(default)]
    pub process_binding: Option<ProcessBindingV1>,
}

/// Mycelix's distributed index to a canonical Symthaea process definition.
/// This contains references and display metadata only; it does not copy
/// process physics, quantities, recipe parameters, or capability limits.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ProcessRegistryEntry {
    pub process_definition_ref: ProcessDefinitionRefV1,
    pub display_label: String,
    pub tags: Vec<String>,
    pub status: ProcessRegistryStatusV1,
    pub supersedes_process_ref: Option<ProcessDefinitionRefV1>,
    pub registered_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, SerializedBytes)]
pub enum ProcessRegistryStatusV1 {
    Active,
    Deprecated,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Operation(OperationEntry),
    Routing(RoutingEntry),
    ProcessRegistry(ProcessRegistryEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllOperations,
    DesignToRouting,
    RoutingToOperations,
    AllProcessRegistryEntries,
}

/// **P0 author-binding pass, 2026-07-09**: no identity field exists on
/// either entry (case a). No coordinator function calls `update_entry`
/// for either entry type (confirmed via grep -- this zome is create-only)
/// so updates are now rejected outright, closing the wide-open
/// RegisterUpdate/RegisterDelete bug that previously routed both through
/// the unconditional `_ => Valid` catch-all.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => {
            validate_create_entry(app_entry)
        }
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) => Ok(ValidateCallbackResult::Invalid(
            "Operations/routings are immutable".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Operations/routings are immutable".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn invalid(message: impl Into<String>) -> ValidateCallbackResult {
    ValidateCallbackResult::Invalid(message.into())
}

fn validate_binding(binding: &Option<ProcessBindingV1>) -> Result<(), String> {
    if let Some(binding) = binding {
        binding.validate()?;
    }
    Ok(())
}

fn validate_create_entry(entry: EntryTypes) -> ExternResult<ValidateCallbackResult> {
    let result = match entry {
        EntryTypes::Operation(operation) => {
            if operation.name.is_empty() {
                Err("operation name is required".to_owned())
            } else {
                validate_binding(&operation.process_binding)
            }
        }
        EntryTypes::Routing(routing) => {
            if routing.design_id.is_empty() {
                Err("design_id is required".to_owned())
            } else if routing.steps.is_empty() {
                Err("routing must have at least one step".to_owned())
            } else if routing
                .steps
                .windows(2)
                .any(|w| w[0].sequence >= w[1].sequence)
            {
                Err("routing steps must be in ascending sequence order".to_owned())
            } else {
                routing
                    .steps
                    .iter()
                    .try_for_each(|step| validate_binding(&step.process_binding))
            }
        }
        EntryTypes::ProcessRegistry(registry) => validate_process_registry_entry(&registry),
    };

    Ok(match result {
        Ok(()) => ValidateCallbackResult::Valid,
        Err(error) => invalid(error),
    })
}

fn validate_process_registry_entry(entry: &ProcessRegistryEntry) -> Result<(), String> {
    entry.process_definition_ref.validate()?;
    if entry.display_label.trim().is_empty() || entry.display_label.len() > 200 {
        return Err("process registry display_label must be 1-200 characters".into());
    }
    if entry.tags.len() > 32 {
        return Err("process registry tags exceed the 32-tag limit".into());
    }
    let mut unique_tags = std::collections::BTreeSet::new();
    for tag in &entry.tags {
        if tag.trim().is_empty()
            || tag.len() > 64
            || tag.chars().any(char::is_control)
            || !unique_tags.insert(tag)
        {
            return Err("process registry tags must be unique, bounded, and canonical".into());
        }
    }
    if let Some(previous) = &entry.supersedes_process_ref {
        previous.validate()?;
        if previous.0.object_id_blake3 == entry.process_definition_ref.0.object_id_blake3 {
            return Err("process registry entry cannot supersede itself".into());
        }
    }
    Ok(())
}


#[cfg(test)]
mod typed_process_reference_tests {
    use super::*;
    use manufacturing_common::{
        ProcessCapabilityRequirementRefV1, SYMTHAEA_CONTRACTS_CRATE,
        SYMTHAEA_MANUFACTURING_CONTRACT_COMMIT_V1, SYMTHAEA_MANUFACTURING_CONTRACT_REPOSITORY,
        SYMTHAEA_PROCESS_CRATE, SymthaeaManufacturingObjectRefV1,
    };

    fn source_ref(
        contract_crate: &str,
        contract_type: &str,
        object_id: char,
    ) -> SymthaeaManufacturingObjectRefV1 {
        SymthaeaManufacturingObjectRefV1 {
            owner_repository: SYMTHAEA_MANUFACTURING_CONTRACT_REPOSITORY.into(),
            owner_commit: SYMTHAEA_MANUFACTURING_CONTRACT_COMMIT_V1.into(),
            contract_crate: contract_crate.into(),
            contract_type: contract_type.into(),
            object_id_blake3: object_id.to_string().repeat(64),
            process_id_blake3: "a".repeat(64),
            semantic_version: "1.0.0".into(),
        }
    }

    fn valid_process_ref() -> ProcessDefinitionRefV1 {
        let mut source = source_ref(SYMTHAEA_PROCESS_CRATE, "ProcessDefinitionV1", 'a');
        source.object_id_blake3 = source.process_id_blake3.clone();
        ProcessDefinitionRefV1(source)
    }

    #[test]
    fn legacy_operation_entry_without_process_binding_remains_deserializable() {
        let legacy = r#"{"name":"Legacy mill","description":"","machine_type":"CNC","setup_time_min":1,"cycle_time_min":2,"tooling":null}"#;
        let decoded: OperationEntry = serde_json::from_str(legacy).unwrap();
        assert!(decoded.process_binding.is_none());
        assert_eq!(decoded.machine_type, "CNC");
    }

    #[test]
    fn legacy_operation_without_binding_remains_valid() {
        let operation = OperationEntry {
            name: "Legacy milling".into(),
            description: "Old record".into(),
            machine_type: "CNC".into(),
            setup_time_min: 3,
            cycle_time_min: 4,
            tooling: None,
            process_binding: None,
        };
        assert_eq!(
            validate_create_entry(EntryTypes::Operation(operation)).unwrap(),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn typed_operation_requires_matching_process_requirement() {
        let binding = ProcessBindingV1 {
            process_definition: valid_process_ref(),
            capability_requirement: ProcessCapabilityRequirementRefV1(source_ref(
                SYMTHAEA_CONTRACTS_CRATE,
                "ProcessCapabilityRequirementV1",
                'b',
            )),
            recipe: None,
        };
        let operation = OperationEntry {
            name: "FDM fixture".into(),
            description: "Typed process route".into(),
            machine_type: "FDM".into(),
            setup_time_min: 1,
            cycle_time_min: 1,
            tooling: None,
            process_binding: Some(binding),
        };
        assert_eq!(
            validate_create_entry(EntryTypes::Operation(operation)).unwrap(),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn partial_or_mismatched_typed_operation_binding_is_rejected() {
        let mut requirement = source_ref(
            SYMTHAEA_CONTRACTS_CRATE,
            "ProcessCapabilityRequirementV1",
            'b',
        );
        requirement.process_id_blake3 = "c".repeat(64);
        let operation = OperationEntry {
            name: "Invalid binding".into(),
            description: String::new(),
            machine_type: "CNC".into(),
            setup_time_min: 1,
            cycle_time_min: 1,
            tooling: None,
            process_binding: Some(ProcessBindingV1 {
                process_definition: valid_process_ref(),
                capability_requirement: ProcessCapabilityRequirementRefV1(requirement),
                recipe: None,
            }),
        };
        assert!(matches!(
            validate_create_entry(EntryTypes::Operation(operation)).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn legacy_routing_step_without_binding_remains_valid() {
        let routing = RoutingEntry {
            design_id: "D-001".into(),
            revision: "A".into(),
            steps: vec![RoutingStepEntry {
                sequence: 10,
                operation_name: "Mill".into(),
                machine_type: "CNC".into(),
                setup_time_min: 1,
                cycle_time_min: 1,
                description: "Legacy".into(),
                process_binding: None,
            }],
            created_at: Timestamp::from_micros(0),
        };
        assert_eq!(
            validate_create_entry(EntryTypes::Routing(routing)).unwrap(),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn registry_rejects_duplicate_tags_and_self_supersession() {
        let process_ref = valid_process_ref();
        let duplicate_tags = ProcessRegistryEntry {
            process_definition_ref: process_ref.clone(),
            display_label: "Material extrusion".into(),
            tags: vec!["additive".into(), "additive".into()],
            status: ProcessRegistryStatusV1::Active,
            supersedes_process_ref: None,
            registered_at: Timestamp::from_micros(0),
        };
        assert!(validate_process_registry_entry(&duplicate_tags).is_err());

        let self_superseding = ProcessRegistryEntry {
            process_definition_ref: process_ref.clone(),
            display_label: "Material extrusion".into(),
            tags: vec!["additive".into()],
            status: ProcessRegistryStatusV1::Deprecated,
            supersedes_process_ref: Some(process_ref),
            registered_at: Timestamp::from_micros(0),
        };
        assert!(validate_process_registry_entry(&self_superseding).is_err());
    }
}

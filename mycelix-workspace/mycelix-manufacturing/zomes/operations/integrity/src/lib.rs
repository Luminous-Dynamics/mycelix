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
        },
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
            } else if routing.steps.windows(2).any(|w| w[0].sequence >= w[1].sequence) {
                Err("routing steps must be in ascending sequence order".to_owned())
            } else {
                routing.steps.iter().try_for_each(|step| {
                    validate_binding(&step.process_binding)
                })
            }
        }
        EntryTypes::ProcessRegistry(registry) => {
            validate_process_registry_entry(&registry)
        }
    };

    Ok(match result {
        Ok(()) => ValidateCallbackResult::Valid,
        Err(error) => invalid(error),
    })
}

fn validate_process_registry_entry(
    entry: &ProcessRegistryEntry,
) -> Result<(), String> {
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

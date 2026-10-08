// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Operations Integrity Zome
//!
//! Entry types and validation for manufacturing operations and routing sequences.

use hdi::prelude::*;
use manufacturing_common::CapabilityRequirement;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CapabilityRequirementEntry {
    pub requirement_id: String,
    pub revision: String,
    pub requirement: CapabilityRequirement,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct OperationEntry {
    pub name: String,
    pub description: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub tooling: Option<String>,
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
    /// Optional migration-era typed capability requirement.
    #[serde(default)]
    pub capability_requirement_hash: Option<ActionHash>,
    /// Exact immutable inspection criteria required for this routing step.
    #[serde(default)]
    pub required_inspection_criterion_hashes: Vec<ActionHash>,
    pub operation_name: String,
    pub machine_type: String,
    pub setup_time_min: u32,
    pub cycle_time_min: u32,
    pub description: String,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CapabilityRequirement(CapabilityRequirementEntry),
    Operation(OperationEntry),
    Routing(RoutingEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllOperations,
    AllCapabilityRequirements,
    DesignToRouting,
    RoutingToInspectionCriteria,
    RoutingToOperations,
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
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::CapabilityRequirement(requirement) => {
                if requirement.requirement_id.is_empty() || requirement.revision.is_empty() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability requirement requires requirement_id and revision".into(),
                    ));
                }
                if requirement.requirement.process_family.is_empty()
                    || requirement.requirement.material_class.is_empty()
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability requirement requires process_family and material_class".into(),
                    ));
                }
                if requirement.requirement.envelope_x_mm == Some(0)
                    || requirement.requirement.envelope_y_mm == Some(0)
                    || requirement.requirement.envelope_z_mm == Some(0)
                    || requirement.requirement.tolerance_um == Some(0)
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability requirement dimensions and tolerance must be > 0 when provided".into(),
                    ));
                }
                if requirement
                    .requirement
                    .required_protocols
                    .iter()
                    .any(|protocol| protocol.is_empty())
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability requirement protocols must be non-empty".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            EntryTypes::Operation(op_entry) => {
                if op_entry.name.is_empty() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "operation name is required".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            EntryTypes::Routing(routing) => {
                if routing.design_id.is_empty() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "design_id is required".into(),
                    ));
                }
                if routing.revision.is_empty() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "routing revision is required".into(),
                    ));
                }
                if routing.steps.is_empty() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "routing must have at least one step".into(),
                    ));
                }
                // Verify steps are in ascending sequence order
                for w in routing.steps.windows(2) {
                    if w[0].sequence >= w[1].sequence {
                        return Ok(ValidateCallbackResult::Invalid(
                            "routing steps must be in ascending sequence order".into(),
                        ));
                    }
                }
                for step in &routing.steps {
                    if step
                        .required_inspection_criterion_hashes
                        .iter()
                        .any(|hash| step.required_inspection_criterion_hashes.iter().filter(|h| *h == hash).count() > 1)
                    {
                        return Ok(ValidateCallbackResult::Invalid(
                            "routing step inspection criterion hashes must be unique".into(),
                        ));
                    }
                    if let Some(requirement_hash) = step.capability_requirement_hash.clone() {
                        let record = must_get_valid_record(requirement_hash)?;
                        let requirement: Option<CapabilityRequirementEntry> = record
                            .entry()
                            .to_app_option()
                            .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                        if requirement.is_none() {
                            return Ok(ValidateCallbackResult::Invalid(
                                "routing capability requirement reference is not a capability requirement record".into(),
                            ));
                        }
                    }
                }
                Ok(ValidateCallbackResult::Valid)
            }
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

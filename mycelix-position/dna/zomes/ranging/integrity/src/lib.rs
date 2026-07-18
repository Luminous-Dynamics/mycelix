// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Ranging integrity zome: validates range measurements between nodes.

use hdi::prelude::*;
use mycelix_position_shared::{RangeMeasurement, validate_node_id, validate_range};

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    RangeMeasurement(RangeMeasurement),
}

#[hdk_link_types]
pub enum LinkTypes {
    RangesByNode,
    RangesByPair,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::RangeMeasurement(m) => {
                    validate_create_range_measurement(EntryCreationAction::Create(action), m)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::RangeMeasurement(_) => Ok(ValidateCallbackResult::Invalid(
                    "RangeMeasurement entries cannot be updated".to_string(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_range_measurement(
    action: EntryCreationAction,
    m: RangeMeasurement,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(e) = validate_node_id(&m.from_node) {
        return Ok(ValidateCallbackResult::Invalid(format!("from_node: {}", e)));
    }
    if let Err(e) = validate_node_id(&m.to_node) {
        return Ok(ValidateCallbackResult::Invalid(format!("to_node: {}", e)));
    }
    if m.from_node == m.to_node {
        return Ok(ValidateCallbackResult::Invalid(
            "from_node and to_node must be different".to_string(),
        ));
    }
    if let Err(e) = validate_range(m.range_m, m.sigma_m) {
        return Ok(ValidateCallbackResult::Invalid(e));
    }
    // Bind the measurement to its committer -- submit_range already derives
    // measured_by from agent_info() coordinator-side with zero user input,
    // so this never rejects a legitimate submission; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap).
    if m.measured_by != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "RangeMeasurement must be measured by the committing agent (measurer forgery)"
                .to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

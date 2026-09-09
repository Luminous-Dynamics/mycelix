// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Position estimates integrity zome: validates fused position entries.

use hdi::prelude::*;
use mycelix_position_shared::{
    PositionEstimateEntry, validate_covariance_3x3, validate_geodetic, validate_node_id,
};

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    PositionEstimate(PositionEstimateEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    EstimatesByNode,
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
                EntryTypes::PositionEstimate(est) => {
                    validate_create_position_estimate(EntryCreationAction::Create(action), est)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::PositionEstimate(_) => Ok(ValidateCallbackResult::Invalid(
                    "PositionEstimate entries cannot be updated".to_string(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_position_estimate(
    action: EntryCreationAction,
    est: PositionEstimateEntry,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(e) = validate_node_id(&est.node_id) {
        return Ok(ValidateCallbackResult::Invalid(e));
    }
    if let Err(e) = validate_geodetic(est.latitude_deg, est.longitude_deg, est.altitude_m) {
        return Ok(ValidateCallbackResult::Invalid(e));
    }
    if let Err(e) = validate_covariance_3x3(&est.covariance) {
        return Ok(ValidateCallbackResult::Invalid(e));
    }
    // Bind the estimate to its committer. Unlike anchor_registry/ranging,
    // store_position_estimate's coordinator does NOT derive computed_by from
    // agent_info() -- it took the whole entry as caller-supplied input, so
    // this check is not just a backstop here, it's the only enforcement
    // (paired with a coordinator-side fix that now overrides computed_by
    // before create_entry, so honest callers are unaffected either way).
    if est.computed_by != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "PositionEstimate must be computed by the committing agent (computed_by forgery)"
                .to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

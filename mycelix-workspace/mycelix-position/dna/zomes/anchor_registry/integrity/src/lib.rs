// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Anchor registry integrity zome: validates anchor nodes and certifications.

use hdi::prelude::*;
use mycelix_position_shared::{
    AnchorCertification, AnchorNode, validate_geodetic, validate_node_id,
};

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    AnchorNode(AnchorNode),
    AnchorCertification(AnchorCertification),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Index anchors by geohash prefix for spatial queries.
    AnchorsByRegion,
    /// Link anchor to its certifications.
    AnchorToCertifications,
    /// Global index of all anchors.
    AllAnchors,
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
                EntryTypes::AnchorNode(node) => {
                    validate_create_anchor_node(EntryCreationAction::Create(action), node)
                }
                EntryTypes::AnchorCertification(cert) => {
                    validate_create_anchor_certification(EntryCreationAction::Create(action), cert)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::AnchorNode(_) => Ok(ValidateCallbackResult::Invalid(
                    "AnchorNode entries cannot be updated".to_string(),
                )),
                EntryTypes::AnchorCertification(_) => Ok(ValidateCallbackResult::Invalid(
                    "AnchorCertification entries cannot be updated".to_string(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_anchor_node(
    action: EntryCreationAction,
    node: AnchorNode,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(e) = validate_node_id(&node.node_id) {
        return Ok(ValidateCallbackResult::Invalid(e));
    }
    if let Err(e) = validate_geodetic(node.latitude_deg, node.longitude_deg, node.altitude_m) {
        return Ok(ValidateCallbackResult::Invalid(e));
    }
    if node.accuracy_m <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Accuracy must be positive, got {}",
            node.accuracy_m
        )));
    }
    if node.accuracy_m > 100_000.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Accuracy > 100km is not useful for positioning".to_string(),
        ));
    }
    // Bind the anchor to its committer -- register_anchor already derives
    // registered_by from agent_info() coordinator-side with zero user input,
    // so this never rejects a legitimate registration; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap).
    if node.registered_by != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "AnchorNode must be registered by the committing agent (registration forgery)"
                .to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_anchor_certification(
    action: EntryCreationAction,
    cert: AnchorCertification,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(e) = validate_node_id(&cert.anchor_node_id) {
        return Ok(ValidateCallbackResult::Invalid(e));
    }
    if cert.verified_accuracy_m <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Verified accuracy must be positive".to_string(),
        ));
    }
    if cert.certification_method.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Certification method cannot be empty".to_string(),
        ));
    }
    // Bind the certification to its committer, same reasoning as
    // validate_create_anchor_node above (P0 author-binding gap).
    if cert.certifier != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "AnchorCertification must be certified by the committing agent (certifier forgery)"
                .to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

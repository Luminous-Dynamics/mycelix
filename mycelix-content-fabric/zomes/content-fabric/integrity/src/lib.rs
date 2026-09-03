// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Integrity definitions for the Mycelix Content Fabric coordination DNA.
//! Bulk content never enters this DHT. Entries are append-only claims/evidence.

mod canonical;
mod types;
mod validation;

pub use canonical::*;
pub use types::*;

use hdi::prelude::*;

fn validate_endpoint_binding_freshness(op: &Op) -> ExternResult<Option<ValidateCallbackResult>> {
    if let FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) =
        op.clone().flattened::<EntryTypes, LinkTypes>()?
    {
        if let EntryTypes::ProviderAdvertisementV1(ad) = app_entry {
            if ad.binding_prev_action != action.prev_action {
                return Ok(Some(ValidateCallbackResult::Invalid(
                    "Iroh endpoint proof must bind the provider advertisement's immediate previous action"
                        .to_string(),
                )));
            }
        }
    }
    Ok(None)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    if let Some(result) = validate_endpoint_binding_freshness(&op)? {
        return Ok(result);
    }
    validation::validate_content_fabric_op(op)
}

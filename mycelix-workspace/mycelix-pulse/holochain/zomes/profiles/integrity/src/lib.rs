// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Profiles Integrity Zome for Mycelix Mail (Stub)
use hdi::prelude::*;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Profile {
    pub name: String,
    pub email: Option<String>,
    pub avatar_url: Option<String>,
    pub bio: Option<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Profile(Profile),
}

#[hdk_link_types]
pub enum LinkTypes {
    AgentToProfile,
    PathComponent,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            action,
            ..
        } => match link_type {
            // AgentToProfile links must be created by the agent themselves --
            // set_profile already derives the base from agent_info() coordinator-side with
            // zero user input, but this was previously completely unvalidated at the DHT
            // level, letting a modified coordinator link an arbitrary agent hash to any
            // Profile entry (P0 author-binding gap). Mirrors the identical pattern already
            // established in this cluster's messages zome.
            LinkTypes::AgentToProfile => {
                let author_hash: AnyLinkableHash = action.author.into();
                if base_address != author_hash {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Agent link base must match action author".to_string(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::PathComponent => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

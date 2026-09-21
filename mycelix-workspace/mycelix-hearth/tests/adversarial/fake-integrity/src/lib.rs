// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Qualification-only fake integrity zome.
//!
//! These entries deliberately reuse canonical Hearth structs under a different
//! AppEntryDef so the real Care transition validator can be attacked for
//! cross-zome type confusion. This zome is never wired into production DNA.

use hdi::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_kinship_integrity::HearthMembership;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    FakeMembership(HearthMembership),
    FakeSchedule(CareSchedule),
}

#[hdk_link_types]
pub enum LinkTypes {
    FixtureOnly,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { .. }) => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. })
        | FlatOp::RegisterUpdate(_)
        | FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "adversarial fixture entries are immutable".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "adversarial fixture does not permit links".into(),
            ))
        }
        FlatOp::StoreEntry(_) | FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! QUALIFICATION-ONLY integrity fixture.
//!
//! Defines entries that are deliberately Serde-compatible with canonical
//! HearthMembership and CareSchedule while belonging to a different integrity
//! zome. Production Hearth DNA never includes this zome.

use hdi::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_kinship_integrity::HearthMembership;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    FakeMembership(HearthMembership),
    FakeSchedule(CareSchedule),
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, ()>()? {
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) | FlatOp::RegisterUpdate(_) => {
            Ok(ValidateCallbackResult::Invalid(
                "Adversarial fixture entries are immutable".into(),
            ))
        }
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Adversarial fixture entries cannot be deleted".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! QUALIFICATION-ONLY fake CareCompletion integrity zome.
//!
//! `FakeCompletion` deliberately occupies entry index 0, matching the real
//! `hearth_care_transitions_integrity::CareCompletion` historical entry index,
//! while belonging to a different integrity zome. This isolates zome/type
//! provenance from entry-index coincidence.

use hdi::prelude::*;
use hearth_care_transitions_integrity::CareCompletion;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    FakeCompletion(CareCompletion),
}

#[hdk_link_types]
pub enum LinkTypes {
    Unused,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) | FlatOp::RegisterUpdate(_) => {
            Ok(ValidateCallbackResult::Invalid(
                "FakeCompletion fixture entries are immutable".into(),
            ))
        }
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "FakeCompletion fixture entries cannot be deleted".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "FakeCompletion fixture defines no usable links".into(),
            ))
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

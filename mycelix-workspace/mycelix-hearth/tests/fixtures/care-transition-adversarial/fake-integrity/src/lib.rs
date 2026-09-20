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

/// Entry ordering is intentionally adversarial:
///
/// - canonical `hearth_care_integrity::CareSchedule` is entry index 0;
/// - canonical `hearth_kinship_integrity::HearthMembership` is entry index 1.
///
/// Matching those historical indexes ensures rejection proves zome/type
/// provenance rather than succeeding merely because the fake index differs.
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    FakeSchedule(CareSchedule),
    FakeMembership(HearthMembership),
}

/// The adversarial fixture does not use links. A concrete link enum keeps the
/// `Op::flattened` type contract explicit instead of relying on an implicit
/// `()` implementation that may vary across HDI versions.
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
                "Adversarial fixture entries are immutable".into(),
            ))
        }
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Adversarial fixture entries cannot be deleted".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "Adversarial fixture defines no usable links".into(),
            ))
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

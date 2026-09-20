// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! QUALIFICATION-ONLY fake-entry cloner.
//!
//! This coordinator depends only on the fake integrity zome. It is appended
//! only to a generated adversarial DNA and is absent from production Hearth.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_transition_fake_integrity::EntryTypes as FakeEntryTypes;
use hearth_kinship_integrity::HearthMembership;

#[hdk_extern]
pub fn create_fake_membership(membership: HearthMembership) -> ExternResult<ActionHash> {
    create_entry(&FakeEntryTypes::FakeMembership(membership))
}

#[hdk_extern]
pub fn create_fake_schedule(schedule: CareSchedule) -> ExternResult<ActionHash> {
    create_entry(&FakeEntryTypes::FakeSchedule(schedule))
}

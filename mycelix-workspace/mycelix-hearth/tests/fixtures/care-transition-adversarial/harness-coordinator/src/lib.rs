// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! QUALIFICATION-ONLY coordinator fixture.
//!
//! This zome is appended only to a generated adversarial DNA used by CI. It is
//! deliberately absent from production `dna/dna.yaml` and `happ.yaml`.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_transition_fake_integrity::EntryTypes as FakeEntryTypes;
use hearth_care_transitions_integrity::{CareCompletion, EntryTypes as TransitionEntryTypes};
use hearth_kinship_integrity::{EntryTypes as KinshipEntryTypes, HearthMembership};

#[hdk_extern]
pub fn create_fake_membership(membership: HearthMembership) -> ExternResult<ActionHash> {
    create_entry(&FakeEntryTypes::FakeMembership(membership))
}

#[hdk_extern]
pub fn create_fake_schedule(schedule: CareSchedule) -> ExternResult<ActionHash> {
    create_entry(&FakeEntryTypes::FakeSchedule(schedule))
}

/// Submit raw CareCompletion evidence directly to the production transition
/// integrity rules. This exists only in the generated adversarial test DNA.
#[hdk_extern]
pub fn publish_completion_unchecked(completion: CareCompletion) -> ExternResult<Record> {
    let hash = create_entry(&TransitionEntryTypes::CareCompletion(completion))?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve adversarial CareCompletion".into()
        ))
    })
}

/// Submit a canonical HearthMembership entry directly to Kinship integrity.
/// This is qualification-only and exists to prove Create-level membership
/// admission/replay rules independently of the production Kinship coordinator.
#[hdk_extern]
pub fn publish_membership_unchecked(membership: HearthMembership) -> ExternResult<Record> {
    let hash = create_entry(&KinshipEntryTypes::HearthMembership(membership))?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve adversarial HearthMembership".into()
        ))
    })
}

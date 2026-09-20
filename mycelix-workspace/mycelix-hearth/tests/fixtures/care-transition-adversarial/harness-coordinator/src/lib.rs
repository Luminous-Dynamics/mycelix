// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! QUALIFICATION-ONLY coordinator fixture.
//!
//! This zome is appended only to a generated adversarial DNA used by CI. It is
//! deliberately absent from production `dna/dna.yaml` and `happ.yaml`.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_transition_fake_integrity::EntryTypes as FakeEntryTypes;
use hearth_care_transitions_integrity::{
    CareCompletion, EntryTypes as TransitionEntryTypes, LinkTypes as TransitionLinkTypes,
};
use hearth_kinship_integrity::HearthMembership;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct UpdateCompletionInput {
    pub original_action_hash: ActionHash,
    pub completion: CareCompletion,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompletionLinkInput {
    pub schedule_hash: ActionHash,
    pub completion_hash: ActionHash,
}

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

/// Attempt to mutate immutable CareCompletion evidence.
#[hdk_extern]
pub fn update_completion_unchecked(input: UpdateCompletionInput) -> ExternResult<Record> {
    let hash = update_entry(
        input.original_action_hash,
        &TransitionEntryTypes::CareCompletion(input.completion),
    )?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve adversarial CareCompletion update".into()
        ))
    })
}

/// Attempt to delete immutable CareCompletion evidence.
#[hdk_extern]
pub fn delete_completion_unchecked(completion_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(completion_hash)
}

/// Attempt to create a ScheduleToCompletions index link with caller-selected
/// base and target. The real transition integrity zome validates both binding
/// and link authorship.
#[hdk_extern]
pub fn create_completion_link_unchecked(input: CompletionLinkInput) -> ExternResult<ActionHash> {
    create_link(
        input.schedule_hash,
        input.completion_hash,
        TransitionLinkTypes::ScheduleToCompletions,
        (),
    )
}

/// Attempt to delete an append-only ScheduleToCompletions index link.
#[hdk_extern]
pub fn delete_completion_link_unchecked(link_add_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_link(link_add_hash)
}

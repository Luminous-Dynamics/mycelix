// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! QUALIFICATION-ONLY raw Care transition harness.
//!
//! The generated DNA gives this coordinator exactly one integrity dependency:
//! `hearth_care_transitions_integrity`. It can proxy typed payloads to the
//! separate fake cloner zome, but cannot register or author fake entry types
//! itself. None of these raw operations are wired into production Hearth.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_transitions_integrity::{CareCompletion, EntryTypes, LinkTypes};
use hearth_kinship_integrity::HearthMembership;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RawCompletionUpdateInput {
    pub original_action_hash: ActionHash,
    pub completion: CareCompletion,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RawCompletionLinkInput {
    pub base: ActionHash,
    pub target: ActionHash,
    pub tag: Vec<u8>,
}

fn call_fake_cloner<I, O>(function: &str, input: I) -> ExternResult<O>
where
    I: serde::Serialize + std::fmt::Debug,
    O: serde::de::DeserializeOwned + std::fmt::Debug,
{
    let response = call(
        CallTargetCell::Local,
        ZomeName::new("hearth_care_transition_fake_cloner"),
        FunctionName::new(function),
        None,
        input,
    )?;

    match response {
        ZomeCallResponse::Ok(io) => io.decode().map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "failed to decode qualification cloner response for {function}: {error}"
            )))
        }),
        other => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "qualification cloner call {function} failed: {other:?}"
        )))),
    }
}

#[hdk_extern]
pub fn create_fake_membership(input: HearthMembership) -> ExternResult<ActionHash> {
    call_fake_cloner("create_fake_membership", input)
}

#[hdk_extern]
pub fn create_fake_schedule(input: CareSchedule) -> ExternResult<ActionHash> {
    call_fake_cloner("create_fake_schedule", input)
}

#[hdk_extern]
pub fn publish_completion_unchecked(completion: CareCompletion) -> ExternResult<Record> {
    let hash = create_entry(&EntryTypes::CareCompletion(completion))?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve adversarial CareCompletion".into()
        ))
    })
}

#[hdk_extern]
pub fn update_completion_unchecked(input: RawCompletionUpdateInput) -> ExternResult<Record> {
    let hash = update_entry(
        input.original_action_hash,
        &EntryTypes::CareCompletion(input.completion),
    )?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve adversarial CareCompletion update".into()
        ))
    })
}

#[hdk_extern]
pub fn delete_completion_unchecked(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

#[hdk_extern]
pub fn create_completion_link_unchecked(input: RawCompletionLinkInput) -> ExternResult<ActionHash> {
    create_link(
        input.base,
        input.target,
        LinkTypes::ScheduleToCompletions,
        LinkTag::new(input.tag),
    )
}

#[hdk_extern]
pub fn delete_completion_link_unchecked(link_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_link(link_hash)
}

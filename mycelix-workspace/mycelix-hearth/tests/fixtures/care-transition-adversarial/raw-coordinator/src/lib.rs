// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! QUALIFICATION-ONLY raw CareCompletion publisher.
//!
//! The generated DNA gives this coordinator exactly one integrity dependency:
//! `hearth_care_transitions_integrity`. It can proxy typed payloads to the
//! separate fake cloner zome, but cannot register or author fake entry types
//! itself.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_transitions_integrity::{CareCompletion, EntryTypes, LinkTypes};
use hearth_kinship_integrity::HearthMembership;

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
pub fn create_fake_completion(input: CareCompletion) -> ExternResult<ActionHash> {
    call_fake_cloner("create_fake_completion", input)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RawCompletionLinkInput {
    pub schedule_hash: ActionHash,
    pub completion_hash: ActionHash,
}

/// Attempt to index an arbitrary target through the real transition link type.
/// The real transition integrity validator decides whether the target is valid.
#[hdk_extern]
pub fn create_completion_link_unchecked(input: RawCompletionLinkInput) -> ExternResult<ActionHash> {
    create_link(
        input.schedule_hash,
        input.completion_hash,
        LinkTypes::ScheduleToCompletions,
        (),
    )
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

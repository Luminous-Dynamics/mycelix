// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Qualification-only coordinator for authoring fake AppEntryDefs.
//! Never wired into production DNA.

use hdk::prelude::*;
use hearth_care_adversarial_fake_integrity::EntryTypes;
use hearth_care_integrity::CareSchedule;
use hearth_kinship_integrity::HearthMembership;

fn source_record(hash: ActionHash, label: &str) -> ExternResult<Record> {
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "canonical {label} source record was not found"
        )))
    })
}

#[hdk_extern]
pub fn clone_membership_as_fake(source_hash: ActionHash) -> ExternResult<Record> {
    let source = source_record(source_hash, "HearthMembership")?;
    let membership: HearthMembership = source
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "failed to decode canonical HearthMembership fixture: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "canonical HearthMembership fixture is missing its entry".into()
            ))
        })?;

    let hash = create_entry(&EntryTypes::FakeMembership(membership))?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "could not retrieve fake membership fixture".into()
        ))
    })
}

#[hdk_extern]
pub fn clone_schedule_as_fake(source_hash: ActionHash) -> ExternResult<Record> {
    let source = source_record(source_hash, "CareSchedule")?;
    let schedule: CareSchedule = source
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "failed to decode canonical CareSchedule fixture: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "canonical CareSchedule fixture is missing its entry".into()
            ))
        })?;

    let hash = create_entry(&EntryTypes::FakeSchedule(schedule))?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "could not retrieve fake schedule fixture".into()
        ))
    })
}

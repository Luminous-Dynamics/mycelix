// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Qualification-only raw CareCompletion publisher.
//! Never wired into production DNA.

use hdk::prelude::*;
use hearth_care_transitions_integrity::{CareCompletion, EntryTypes};

#[hdk_extern]
pub fn publish_raw_completion(completion: CareCompletion) -> ExternResult<Record> {
    let hash = create_entry(&EntryTypes::CareCompletion(completion))?;
    get(hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "could not retrieve raw CareCompletion fixture".into()
        ))
    })
}

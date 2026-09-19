// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Transition-aware Care occurrence materialization.
//!
//! The concrete occurrence keeps its stable `(schedule, window)` identity. A
//! separate append-only link binds the occurrence EntryHash to the exact Care
//! assignment-state ActionHash that authorized its assignee.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_ledger::{
    canonicalize_occurrence_assignment_bindings, canonicalize_occurrences,
    CareOccurrence as LedgerOccurrence, LedgerError, MemberId,
    OccurrenceAssignmentBinding, OccurrenceAssignmentBindingRecord, OccurrenceRecord,
    OccurrenceWindow, RecordId, ScheduleId,
};
use hearth_care_ledger_integrity::{CareOccurrenceEntry, EntryTypes, LinkTypes};
use hearth_coordinator_common::{decode_zome_response, get_latest_record, require_membership};
use hearth_types::CareScheduleStatus;
use mycelix_bridge_common::civic_requirement_basic;
use std::collections::BTreeSet;

use crate::legacy::{
    MaterializeOccurrenceInput, MaterializedOccurrence, OccurrenceView,
};

#[derive(Serialize, Deserialize, Debug, Clone)]
struct EffectiveAssignmentViewWire {
    hearth_hash: ActionHash,
    schedule_root_hash: ActionHash,
    latest_template_state_hash: ActionHash,
    assignment_state_hash: ActionHash,
    assignee: AgentPubKey,
    transition_count: u32,
    duplicate_evidence_count: u32,
    orphan_transition_count: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MaterializedOccurrenceV2 {
    pub occurrence: MaterializedOccurrence,
    pub assignment_state_hash: ActionHash,
    pub reused_existing_binding: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct QualifiedOccurrenceView {
    pub occurrence: OccurrenceView,
    pub assignment_state_hash: ActionHash,
    pub duplicate_binding_count: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ScheduleOccurrencesV2 {
    pub qualified: Vec<QualifiedOccurrenceView>,
    /// Legacy occurrence evidence remains visible but is not assignment-qualified.
    pub unbound_legacy_occurrence_ids: Vec<String>,
}

fn entry_from_record<T: TryFrom<SerializedBytes, Error = SerializedBytesError>>(
    record: &Record,
    type_name: &str,
) -> ExternResult<T> {
    record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid {type_name} entry"
            )))
        })
}

fn ledger_error(context: &str, error: impl std::fmt::Display) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{context}: {error}")))
}

fn latest_schedule(schedule_hash: &ActionHash) -> ExternResult<CareSchedule> {
    let record = get_latest_record(schedule_hash.clone())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Care schedule not found".into()))
    })?;
    entry_from_record(&record, "CareSchedule")
}

fn effective_assignment(schedule_hash: &ActionHash) -> ExternResult<EffectiveAssignmentViewWire> {
    decode_zome_response(
        call(
            CallTargetCell::Local,
            ZomeName::new("hearth_automation"),
            FunctionName::new("get_effective_assignment"),
            None,
            schedule_hash.clone(),
        )?,
        "hearth_automation::get_effective_assignment",
    )
}

fn occurrence_to_ledger(entry: &CareOccurrenceEntry) -> ExternResult<LedgerOccurrence> {
    let occurrence = LedgerOccurrence {
        schema_version: hearth_care_ledger::CARE_LEDGER_SCHEMA_VERSION,
        id: hearth_care_ledger::OccurrenceId(entry.occurrence_id.clone()),
        schedule_id: ScheduleId(entry.schedule_hash.to_string()),
        assigned_to: MemberId(entry.assigned_to.to_string()),
        window: OccurrenceWindow {
            start_micros: entry.window_start.as_micros(),
            end_micros: entry.window_end.as_micros(),
        },
        estimated_minutes: entry.estimated_minutes,
    };
    occurrence
        .validate()
        .map_err(|error| ledger_error("Invalid Care occurrence", format!("{error:?}")))?;
    Ok(occurrence)
}

fn occurrence_views_for_schedule(schedule_hash: &ActionHash) -> ExternResult<Vec<OccurrenceView>> {
    let links = get_links(
        LinkQuery::try_new(schedule_hash.clone(), LinkTypes::ScheduleToOccurrences)?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut views = Vec::new();
    for link in links {
        let entry_hash = EntryHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid Care occurrence link target".into()
            ))
        })?;
        if !seen.insert(entry_hash.to_string()) {
            continue;
        }
        if let Some(record) = get(entry_hash.clone(), GetOptions::default())? {
            let occurrence: CareOccurrenceEntry =
                entry_from_record(&record, "CareOccurrenceEntry")?;
            views.push(OccurrenceView {
                action_hash: record.action_address().clone(),
                entry_hash,
                occurrence,
            });
        }
    }
    views.sort_by(|left, right| {
        left.occurrence
            .occurrence_id
            .cmp(&right.occurrence.occurrence_id)
            .then_with(|| left.entry_hash.to_string().cmp(&right.entry_hash.to_string()))
    });

    let pure_records = views
        .iter()
        .map(|view| {
            Ok(OccurrenceRecord {
                record_id: RecordId(view.entry_hash.to_string()),
                occurrence: occurrence_to_ledger(&view.occurrence)?,
            })
        })
        .collect::<ExternResult<Vec<_>>>()?;
    canonicalize_occurrences(&pure_records)
        .map_err(|error| ledger_error("Conflicting Care occurrence evidence", format!("{error:?}")))?;
    Ok(views)
}

fn ensure_occurrence_links(
    schedule_hash: &ActionHash,
    hearth_hash: &ActionHash,
    entry_hash: &EntryHash,
) -> ExternResult<()> {
    create_link(
        schedule_hash.clone(),
        entry_hash.clone(),
        LinkTypes::ScheduleToOccurrences,
        (),
    )?;
    create_link(
        hearth_hash.clone(),
        entry_hash.clone(),
        LinkTypes::HearthToOccurrences,
        (),
    )?;
    Ok(())
}

fn assignment_bindings(
    entry_hash: &EntryHash,
    occurrence: &LedgerOccurrence,
) -> ExternResult<Vec<OccurrenceAssignmentBindingRecord>> {
    let links = get_links(
        LinkQuery::try_new(entry_hash.clone(), LinkTypes::OccurrenceToAssignmentState)?,
        GetStrategy::default(),
    )?;
    let mut records = Vec::new();
    let mut seen = BTreeSet::new();
    for link in links {
        let state_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid occurrence assignment-state link target".into()
            ))
        })?;
        if !seen.insert(state_hash.to_string()) {
            continue;
        }
        let binding = OccurrenceAssignmentBinding::from_occurrence(
            occurrence,
            state_hash.to_string(),
        )
        .map_err(|error| ledger_error("Invalid occurrence assignment binding", error))?;
        records.push(OccurrenceAssignmentBindingRecord {
            record_id: RecordId(state_hash.to_string()),
            binding,
        });
    }
    records.sort_by(|left, right| left.record_id.cmp(&right.record_id));
    Ok(records)
}

fn ensure_assignment_binding(
    entry_hash: &EntryHash,
    occurrence: &LedgerOccurrence,
    assignment_state_hash: &ActionHash,
) -> ExternResult<bool> {
    let existing = assignment_bindings(entry_hash, occurrence)?;
    let canonical = canonicalize_occurrence_assignment_bindings(&existing)
        .map_err(|error| ledger_error("Conflicting occurrence assignment authority", error))?;
    if let Some(binding) = canonical.get(&occurrence.id) {
        if binding.canonical.binding.assignment_state_ref == assignment_state_hash.to_string() {
            return Ok(true);
        }
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Care occurrence is already bound to a different assignment state".into()
        )));
    }

    create_link(
        entry_hash.clone(),
        assignment_state_hash.clone(),
        LinkTypes::OccurrenceToAssignmentState,
        (),
    )?;

    // Re-read after write so concurrent competing bindings fail closed.
    let after = assignment_bindings(entry_hash, occurrence)?;
    canonicalize_occurrence_assignment_bindings(&after)
        .map_err(|error| ledger_error("Conflicting occurrence assignment authority", error))?;
    Ok(false)
}

#[hdk_extern]
pub fn materialize_occurrence_v2(
    input: MaterializeOccurrenceInput,
) -> ExternResult<MaterializedOccurrenceV2> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "materialize_care_occurrence_v2",
    )?;

    let schedule = latest_schedule(&input.schedule_hash)?;
    require_membership(&schedule.hearth_hash)?;
    if schedule.status != CareScheduleStatus::Active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an Active CareSchedule may materialize new occurrences".into()
        )));
    }

    let effective = effective_assignment(&input.schedule_hash)?;
    if effective.hearth_hash != schedule.hearth_hash
        || effective.schedule_root_hash != input.schedule_hash
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Effective assignment does not belong to this Care schedule".into()
        )));
    }

    let ledger = LedgerOccurrence::new(
        ScheduleId(input.schedule_hash.to_string()),
        MemberId(effective.assignee.to_string()),
        OccurrenceWindow {
            start_micros: input.window_start.as_micros(),
            end_micros: input.window_end.as_micros(),
        },
        input.estimated_minutes,
    )
    .map_err(|error| ledger_error("Cannot materialize Care occurrence", format!("{error:?}")))?;

    let occurrence = CareOccurrenceEntry {
        hearth_hash: schedule.hearth_hash.clone(),
        schedule_hash: input.schedule_hash.clone(),
        occurrence_id: ledger.id.0.clone(),
        assigned_to: effective.assignee.clone(),
        window_start: input.window_start,
        window_end: input.window_end,
        estimated_minutes: input.estimated_minutes,
    };
    let entry = EntryTypes::CareOccurrence(occurrence.clone());
    let entry_hash = hash_entry(&entry)?;

    let existing_views = occurrence_views_for_schedule(&input.schedule_hash)?;
    for view in &existing_views {
        if view.occurrence.occurrence_id == occurrence.occurrence_id
            && view.occurrence != occurrence
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Conflicting Care occurrence already exists for this schedule/window".into()
            )));
        }
    }

    // Re-check immediately before first durable write. A transition committed
    // after this point does not retroactively rewrite an already materialized
    // occurrence; its exact state snapshot remains part of the evidence.
    let final_effective = effective_assignment(&input.schedule_hash)?;
    if final_effective.assignment_state_hash != effective.assignment_state_hash
        || final_effective.assignee != effective.assignee
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Care assignment changed while occurrence was being materialized; retry required"
                .into()
        )));
    }

    let (action_hash, reused_existing_content) = if let Some(existing) =
        get(entry_hash.clone(), GetOptions::default())?
    {
        let existing_occurrence: CareOccurrenceEntry =
            entry_from_record(&existing, "CareOccurrenceEntry")?;
        if existing_occurrence != occurrence {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care occurrence content hash resolved to unexpected semantics".into()
            )));
        }
        (existing.action_address().clone(), true)
    } else {
        (create_entry(&entry)?, false)
    };

    ensure_occurrence_links(&input.schedule_hash, &schedule.hearth_hash, &entry_hash)?;
    let reused_existing_binding =
        ensure_assignment_binding(&entry_hash, &ledger, &effective.assignment_state_hash)?;

    Ok(MaterializedOccurrenceV2 {
        occurrence: MaterializedOccurrence {
            action_hash,
            entry_hash,
            occurrence,
            reused_existing_content,
        },
        assignment_state_hash: effective.assignment_state_hash,
        reused_existing_binding,
    })
}

#[hdk_extern]
pub fn get_schedule_occurrences_v2(
    schedule_hash: ActionHash,
) -> ExternResult<ScheduleOccurrencesV2> {
    let schedule = latest_schedule(&schedule_hash)?;
    require_membership(&schedule.hearth_hash)?;
    let views = occurrence_views_for_schedule(&schedule_hash)?;
    let mut qualified = Vec::new();
    let mut unbound = Vec::new();

    for view in views {
        let occurrence = occurrence_to_ledger(&view.occurrence)?;
        let bindings = assignment_bindings(&view.entry_hash, &occurrence)?;
        let canonical = canonicalize_occurrence_assignment_bindings(&bindings)
            .map_err(|error| ledger_error("Conflicting occurrence assignment authority", error))?;
        let Some(binding) = canonical.get(&occurrence.id) else {
            unbound.push(view.occurrence.occurrence_id.clone());
            continue;
        };
        let assignment_state_hash = ActionHash::try_from(
            binding.canonical.binding.assignment_state_ref.clone(),
        )
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid canonical assignment-state reference: {error}"
        ))))?;
        qualified.push(QualifiedOccurrenceView {
            occurrence: view,
            assignment_state_hash,
            duplicate_binding_count: binding
                .duplicate_record_ids
                .len()
                .min(u32::MAX as usize) as u32,
        });
    }

    qualified.sort_by(|left, right| {
        left.occurrence
            .occurrence
            .occurrence_id
            .cmp(&right.occurrence.occurrence.occurrence_id)
    });
    unbound.sort();
    unbound.dedup();
    Ok(ScheduleOccurrencesV2 {
        qualified,
        unbound_legacy_occurrence_ids: unbound,
    })
}

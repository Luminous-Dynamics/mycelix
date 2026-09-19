// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Assignment-qualified completion and workload history.
//!
//! Legacy completion evidence remains readable, but automation-facing writes and
//! digests require one canonical occurrence→assignment-state binding.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_ledger::{
    build_digest_v3, canonicalize_occurrence_assignment_bindings,
    CareCompletion as LedgerCompletion, CareDigestV3, CareOccurrence as LedgerOccurrence,
    CompletionRecord, MemberId, OccurrenceAssignmentBinding,
    OccurrenceAssignmentBindingRecord, OccurrenceId, OccurrenceRecord, OccurrenceWindow,
    RecordId, ScheduleId, CARE_LEDGER_SCHEMA_VERSION,
};
use hearth_care_ledger_integrity::{
    CareCompletionEntry, CareOccurrenceEntry, EntryTypes, LinkTypes,
};
use hearth_coordinator_common::{decode_zome_response, get_latest_record, require_membership};
use hearth_types::{DigestEpochInput, HearthSignal, MemberRole};
use mycelix_bridge_common::civic_requirement_basic;
use std::collections::BTreeSet;

use crate::legacy::{CompleteOccurrenceInput, CompletedOccurrence, OccurrenceView};

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

fn evidence_error(context: &str, error: impl std::fmt::Display) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{context}: {error}")))
}

fn latest_schedule(schedule_hash: &ActionHash) -> ExternResult<CareSchedule> {
    let record = get_latest_record(schedule_hash.clone())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Care schedule not found".into()))
    })?;
    entry_from_record(&record, "CareSchedule")
}

fn occurrence_from_action(
    action_hash: &ActionHash,
) -> ExternResult<(EntryHash, CareOccurrenceEntry)> {
    let record = get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Care occurrence action not found".into()))
    })?;
    let occurrence: CareOccurrenceEntry = entry_from_record(&record, "CareOccurrenceEntry")?;
    let entry_hash = hash_entry(&EntryTypes::CareOccurrence(occurrence.clone()))?;
    Ok((entry_hash, occurrence))
}

fn occurrence_to_ledger(entry: &CareOccurrenceEntry) -> ExternResult<LedgerOccurrence> {
    let occurrence = LedgerOccurrence {
        schema_version: CARE_LEDGER_SCHEMA_VERSION,
        id: OccurrenceId(entry.occurrence_id.clone()),
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
        .map_err(|error| evidence_error("Invalid Care occurrence", format!("{error:?}")))?;
    Ok(occurrence)
}

fn completion_to_ledger(entry: &CareCompletionEntry) -> ExternResult<LedgerCompletion> {
    let completion = LedgerCompletion {
        schema_version: CARE_LEDGER_SCHEMA_VERSION,
        occurrence_id: OccurrenceId(entry.occurrence_id.clone()),
        performed_by: MemberId(entry.performed_by.to_string()),
        recorded_by: MemberId(entry.recorded_by.to_string()),
        completed_at_micros: entry.completed_at.as_micros(),
        actual_minutes: entry.actual_minutes,
        evidence_refs: entry.evidence_refs.clone(),
    };
    completion
        .validate()
        .map_err(|error| evidence_error("Invalid Care completion", format!("{error:?}")))?;
    Ok(completion)
}

fn assignment_binding_records(
    occurrence_entry_hash: &EntryHash,
    occurrence: &LedgerOccurrence,
) -> ExternResult<Vec<OccurrenceAssignmentBindingRecord>> {
    let links = get_links(
        LinkQuery::try_new(
            occurrence_entry_hash.clone(),
            LinkTypes::OccurrenceToAssignmentState,
        )?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut records = Vec::new();
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
        .map_err(|error| evidence_error("Invalid occurrence assignment binding", error))?;
        records.push(OccurrenceAssignmentBindingRecord {
            record_id: RecordId(state_hash.to_string()),
            binding,
        });
    }
    records.sort_by(|left, right| left.record_id.cmp(&right.record_id));
    Ok(records)
}

fn require_canonical_assignment_binding(
    occurrence_entry_hash: &EntryHash,
    occurrence: &LedgerOccurrence,
) -> ExternResult<()> {
    let bindings = assignment_binding_records(occurrence_entry_hash, occurrence)?;
    let canonical = canonicalize_occurrence_assignment_bindings(&bindings)
        .map_err(|error| evidence_error("Conflicting occurrence assignment authority", error))?;
    if canonical.get(&occurrence.id).is_none() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Care occurrence has no canonical assignment-state binding; use transition-aware materialization first"
                .into()
        )));
    }
    Ok(())
}

fn require_guardian_role(hearth_hash: &ActionHash) -> ExternResult<MemberRole> {
    let role: Option<MemberRole> = decode_zome_response(
        call(
            CallTargetCell::Local,
            ZomeName::new("hearth_kinship"),
            FunctionName::new("get_caller_role"),
            None,
            hearth_hash.clone(),
        )?,
        "get_caller_role",
    )?;
    match role {
        Some(role) if role.is_guardian() => Ok(role),
        _ => Err(wasm_error!(WasmErrorInner::Guest(
            "Only the assigned member or a guardian may record this Care completion".into()
        ))),
    }
}

fn completion_views_for_hearth(
    hearth_hash: &ActionHash,
) -> ExternResult<Vec<(ActionHash, CareCompletionEntry)>> {
    let links = get_links(
        LinkQuery::try_new(hearth_hash.clone(), LinkTypes::HearthToCompletions)?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid Care completion link target".into()
            ))
        })?;
        if !seen.insert(action_hash.to_string()) {
            continue;
        }
        if let Some(record) = get(action_hash.clone(), GetOptions::default())? {
            let completion: CareCompletionEntry =
                entry_from_record(&record, "CareCompletionEntry")?;
            if completion.hearth_hash == *hearth_hash {
                out.push((action_hash, completion));
            }
        }
    }
    out.sort_by(|left, right| left.0.to_string().cmp(&right.0.to_string()));
    Ok(out)
}

#[hdk_extern]
pub fn complete_occurrence_v2(
    input: CompleteOccurrenceInput,
) -> ExternResult<CompletedOccurrence> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "complete_care_occurrence_v2",
    )?;

    let (occurrence_entry_hash, occurrence) =
        occurrence_from_action(&input.occurrence_action_hash)?;
    require_membership(&occurrence.hearth_hash)?;
    let ledger_occurrence = occurrence_to_ledger(&occurrence)?;
    require_canonical_assignment_binding(&occurrence_entry_hash, &ledger_occurrence)?;

    let caller = agent_info()?.agent_initial_pubkey;
    if caller != occurrence.assigned_to {
        require_guardian_role(&occurrence.hearth_hash)?;
    }

    // Narrow a concurrent-binding race immediately before the completion write.
    require_canonical_assignment_binding(&occurrence_entry_hash, &ledger_occurrence)?;

    let now = sys_time()?;
    let completion = CareCompletionEntry {
        hearth_hash: occurrence.hearth_hash.clone(),
        occurrence_action_hash: input.occurrence_action_hash,
        occurrence_entry_hash: occurrence_entry_hash.clone(),
        occurrence_id: occurrence.occurrence_id.clone(),
        performed_by: occurrence.assigned_to.clone(),
        recorded_by: caller,
        completed_at: now,
        actual_minutes: input.actual_minutes,
        evidence_refs: input.evidence_refs,
    };
    completion_to_ledger(&completion)?;

    let action_hash = create_entry(&EntryTypes::CareCompletion(completion.clone()))?;
    create_link(
        occurrence_entry_hash,
        action_hash.clone(),
        LinkTypes::OccurrenceToCompletions,
        (),
    )?;
    create_link(
        occurrence.hearth_hash.clone(),
        action_hash.clone(),
        LinkTypes::HearthToCompletions,
        (),
    )?;

    let schedule = latest_schedule(&occurrence.schedule_hash)?;
    emit_signal(&HearthSignal::CareTaskCompleted {
        assignee: occurrence.assigned_to,
        schedule_hash: occurrence.schedule_hash,
        care_type: schedule.care_type,
    })?;

    Ok(CompletedOccurrence {
        action_hash,
        completion,
    })
}

#[hdk_extern]
pub fn create_care_digest_v3(input: DigestEpochInput) -> ExternResult<CareDigestV3> {
    if input.epoch_start >= input.epoch_end {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "epoch_start must be before epoch_end".into()
        )));
    }
    require_membership(&input.hearth_hash)?;

    let occurrence_views: Vec<OccurrenceView> =
        crate::legacy::get_hearth_occurrences(input.hearth_hash.clone())?;
    let mut occurrence_records = Vec::with_capacity(occurrence_views.len());
    let mut binding_records = Vec::new();
    for view in &occurrence_views {
        let occurrence = occurrence_to_ledger(&view.occurrence)?;
        occurrence_records.push(OccurrenceRecord {
            record_id: RecordId(view.entry_hash.to_string()),
            occurrence: occurrence.clone(),
        });
        binding_records.extend(assignment_binding_records(&view.entry_hash, &occurrence)?);
    }

    let completion_records = completion_views_for_hearth(&input.hearth_hash)?
        .into_iter()
        .filter(|(_, completion)| {
            completion.completed_at >= input.epoch_start
                && completion.completed_at < input.epoch_end
        })
        .map(|(action_hash, completion)| {
            Ok(CompletionRecord {
                record_id: RecordId(action_hash.to_string()),
                completion: completion_to_ledger(&completion)?,
            })
        })
        .collect::<ExternResult<Vec<_>>>()?;

    build_digest_v3(&occurrence_records, &binding_records, &completion_records)
        .map_err(|error| evidence_error("Cannot build CareDigestV3", error))
}

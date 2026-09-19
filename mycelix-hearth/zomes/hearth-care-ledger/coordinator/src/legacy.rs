// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Coordinator for Hearth's append-only recurring Care ledger.
//!
//! Legacy `hearth_care::CareSchedule` remains the template source. This zome
//! materializes concrete occurrences and immutable completion attestations.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_ledger::{
    build_digest_v2, canonicalize_occurrences, CareCompletion as LedgerCompletion,
    CareDigestV2, CareOccurrence as LedgerOccurrence, CompletionRecord, LedgerError, MemberId,
    OccurrenceId, OccurrenceRecord, OccurrenceWindow, RecordId, ScheduleId,
    CARE_LEDGER_SCHEMA_VERSION,
};
use hearth_care_ledger_integrity::{
    CareCompletionEntry, CareOccurrenceEntry, EntryTypes, LinkTypes,
};
use hearth_coordinator_common::{decode_zome_response, get_latest_record, require_membership};
use hearth_types::{CareScheduleStatus, DigestEpochInput, HearthSignal, MemberRole};
use mycelix_bridge_common::civic_requirement_basic;
use std::collections::BTreeSet;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MaterializeOccurrenceInput {
    /// Stable/root ActionHash of the source CareSchedule.
    pub schedule_hash: ActionHash,
    pub window_start: Timestamp,
    pub window_end: Timestamp,
    pub estimated_minutes: Option<u32>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MaterializedOccurrence {
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub occurrence: CareOccurrenceEntry,
    pub reused_existing_content: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CompleteOccurrenceInput {
    /// Any valid action whose entry is the immutable occurrence being completed.
    pub occurrence_action_hash: ActionHash,
    pub actual_minutes: Option<u32>,
    pub evidence_refs: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CompletedOccurrence {
    pub action_hash: ActionHash,
    pub completion: CareCompletionEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OccurrenceView {
    /// One valid action carrying this content. Identical concurrent actions may exist.
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub occurrence: CareOccurrenceEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CompletionView {
    pub action_hash: ActionHash,
    pub completion: CareCompletionEntry,
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

fn ledger_error(context: &str, error: LedgerError) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{context}: {error:?}")))
}

fn latest_schedule(schedule_hash: &ActionHash) -> ExternResult<CareSchedule> {
    let record = get_latest_record(schedule_hash.clone())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Care schedule not found".into()))
    })?;
    entry_from_record(&record, "CareSchedule")
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

fn occurrence_to_ledger(entry: &CareOccurrenceEntry) -> ExternResult<LedgerOccurrence> {
    let value = LedgerOccurrence {
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
    value
        .validate()
        .map_err(|error| ledger_error("Invalid Care occurrence", error))?;
    Ok(value)
}

fn completion_to_ledger(entry: &CareCompletionEntry) -> ExternResult<LedgerCompletion> {
    let value = LedgerCompletion {
        schema_version: CARE_LEDGER_SCHEMA_VERSION,
        occurrence_id: OccurrenceId(entry.occurrence_id.clone()),
        performed_by: MemberId(entry.performed_by.to_string()),
        recorded_by: MemberId(entry.recorded_by.to_string()),
        completed_at_micros: entry.completed_at.as_micros(),
        actual_minutes: entry.actual_minutes,
        evidence_refs: entry.evidence_refs.clone(),
    };
    value
        .validate()
        .map_err(|error| ledger_error("Invalid Care completion", error))?;
    Ok(value)
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

fn get_occurrence(entry_hash: &EntryHash) -> ExternResult<(ActionHash, CareOccurrenceEntry)> {
    let record = get(entry_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Care occurrence not found".into()))
    })?;
    let action_hash = record.action_address().clone();
    let occurrence = entry_from_record(&record, "CareOccurrenceEntry")?;
    Ok((action_hash, occurrence))
}

fn occurrence_views_for_links(
    base: impl Into<AnyLinkableHash>,
    link_type: LinkTypes,
) -> ExternResult<Vec<OccurrenceView>> {
    let links = get_links(
        LinkQuery::try_new(base.into(), link_type)?,
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
    views.sort_by(|a, b| {
        a.occurrence
            .occurrence_id
            .cmp(&b.occurrence.occurrence_id)
            .then_with(|| a.entry_hash.to_string().cmp(&b.entry_hash.to_string()))
    });
    Ok(views)
}

fn completion_views_for_hearth(hearth_hash: &ActionHash) -> ExternResult<Vec<CompletionView>> {
    let links = get_links(
        LinkQuery::try_new(hearth_hash.clone(), LinkTypes::HearthToCompletions)?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut views = Vec::new();
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
                views.push(CompletionView {
                    action_hash,
                    completion,
                });
            }
        }
    }
    views.sort_by(|a, b| a.action_hash.to_string().cmp(&b.action_hash.to_string()));
    Ok(views)
}

fn validate_occurrence_set(views: &[OccurrenceView]) -> ExternResult<()> {
    let records = views
        .iter()
        .map(|view| {
            Ok(OccurrenceRecord {
                record_id: RecordId(view.entry_hash.to_string()),
                occurrence: occurrence_to_ledger(&view.occurrence)?,
            })
        })
        .collect::<ExternResult<Vec<_>>>()?;
    canonicalize_occurrences(&records)
        .map_err(|error| ledger_error("Conflicting Care occurrence evidence", error))?;
    Ok(())
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

#[hdk_extern]
pub fn materialize_occurrence(
    input: MaterializeOccurrenceInput,
) -> ExternResult<MaterializedOccurrence> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "materialize_care_occurrence",
    )?;

    let schedule = latest_schedule(&input.schedule_hash)?;
    require_membership(&schedule.hearth_hash)?;
    if schedule.status != CareScheduleStatus::Active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an Active CareSchedule may materialize new occurrences".into()
        )));
    }

    let ledger = LedgerOccurrence::new(
        ScheduleId(input.schedule_hash.to_string()),
        MemberId(schedule.assigned_to.to_string()),
        OccurrenceWindow {
            start_micros: input.window_start.as_micros(),
            end_micros: input.window_end.as_micros(),
        },
        input.estimated_minutes,
    )
    .map_err(|error| ledger_error("Cannot materialize Care occurrence", error))?;

    let occurrence = CareOccurrenceEntry {
        hearth_hash: schedule.hearth_hash.clone(),
        schedule_hash: input.schedule_hash.clone(),
        occurrence_id: ledger.id.0,
        assigned_to: schedule.assigned_to,
        window_start: input.window_start,
        window_end: input.window_end,
        estimated_minutes: input.estimated_minutes,
    };
    let entry = EntryTypes::CareOccurrence(occurrence.clone());
    let entry_hash = hash_entry(&entry)?;

    // Exact-content reuse is an optimization, not a global-uniqueness claim.
    // Concurrent identical creations have the same EntryHash. We still ensure
    // discovery links because another writer may have crashed after create_entry.
    if let Some(existing) = get(entry_hash.clone(), GetOptions::default())? {
        let existing_occurrence: CareOccurrenceEntry =
            entry_from_record(&existing, "CareOccurrenceEntry")?;
        if existing_occurrence == occurrence {
            let views = occurrence_views_for_links(
                input.schedule_hash.clone(),
                LinkTypes::ScheduleToOccurrences,
            )?;
            validate_occurrence_set(&views)?;
            ensure_occurrence_links(&input.schedule_hash, &schedule.hearth_hash, &entry_hash)?;
            return Ok(MaterializedOccurrence {
                action_hash: existing.action_address().clone(),
                entry_hash,
                occurrence,
                reused_existing_content: true,
            });
        }
    }

    // Visible pre-check is defense in depth only. Races remain safe because
    // readers canonicalize same-ID evidence and fail closed on semantic conflict.
    let existing_views = occurrence_views_for_links(
        input.schedule_hash.clone(),
        LinkTypes::ScheduleToOccurrences,
    )?;
    validate_occurrence_set(&existing_views)?;
    for view in &existing_views {
        if view.occurrence.occurrence_id == occurrence.occurrence_id
            && view.occurrence != occurrence
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Conflicting Care occurrence already exists for this schedule/window".into()
            )));
        }
    }

    let action_hash = create_entry(&entry)?;
    ensure_occurrence_links(&input.schedule_hash, &schedule.hearth_hash, &entry_hash)?;

    Ok(MaterializedOccurrence {
        action_hash,
        entry_hash,
        occurrence,
        reused_existing_content: false,
    })
}

#[hdk_extern]
pub fn complete_occurrence(input: CompleteOccurrenceInput) -> ExternResult<CompletedOccurrence> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "complete_care_occurrence",
    )?;

    let (occurrence_entry_hash, occurrence) =
        occurrence_from_action(&input.occurrence_action_hash)?;
    require_membership(&occurrence.hearth_hash)?;
    let caller = agent_info()?.agent_initial_pubkey;
    if caller != occurrence.assigned_to {
        require_guardian_role(&occurrence.hearth_hash)?;
    }

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

    // Compatibility signal: attribute the task to the performer/assignee, not
    // to a guardian who merely recorded the completion.
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
pub fn get_schedule_occurrences(schedule_hash: ActionHash) -> ExternResult<Vec<OccurrenceView>> {
    let schedule = latest_schedule(&schedule_hash)?;
    require_membership(&schedule.hearth_hash)?;
    let views = occurrence_views_for_links(schedule_hash, LinkTypes::ScheduleToOccurrences)?;
    validate_occurrence_set(&views)?;
    Ok(views)
}

#[hdk_extern]
pub fn get_hearth_occurrences(hearth_hash: ActionHash) -> ExternResult<Vec<OccurrenceView>> {
    require_membership(&hearth_hash)?;
    let views = occurrence_views_for_links(hearth_hash.clone(), LinkTypes::HearthToOccurrences)?;
    for view in &views {
        if view.occurrence.hearth_hash != hearth_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care occurrence/hearth link mismatch".into()
            )));
        }
    }
    validate_occurrence_set(&views)?;
    Ok(views)
}

#[hdk_extern]
pub fn get_occurrence_completions(
    occurrence_entry_hash: EntryHash,
) -> ExternResult<Vec<CompletionView>> {
    let (_, occurrence) = get_occurrence(&occurrence_entry_hash)?;
    require_membership(&occurrence.hearth_hash)?;
    let links = get_links(
        LinkQuery::try_new(
            occurrence_entry_hash.clone(),
            LinkTypes::OccurrenceToCompletions,
        )?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut views = Vec::new();
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
            if completion.occurrence_entry_hash == occurrence_entry_hash
                && completion.occurrence_id == occurrence.occurrence_id
                && completion.hearth_hash == occurrence.hearth_hash
                && completion.performed_by == occurrence.assigned_to
            {
                views.push(CompletionView {
                    action_hash,
                    completion,
                });
            }
        }
    }
    views.sort_by(|a, b| a.action_hash.to_string().cmp(&b.action_hash.to_string()));
    Ok(views)
}

#[hdk_extern]
pub fn create_care_digest_v2(input: DigestEpochInput) -> ExternResult<CareDigestV2> {
    if input.epoch_start >= input.epoch_end {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "epoch_start must be before epoch_end".into()
        )));
    }
    require_membership(&input.hearth_hash)?;

    let occurrence_views = get_hearth_occurrences(input.hearth_hash.clone())?;
    let occurrence_records = occurrence_views
        .iter()
        .map(|view| {
            Ok(OccurrenceRecord {
                record_id: RecordId(view.entry_hash.to_string()),
                occurrence: occurrence_to_ledger(&view.occurrence)?,
            })
        })
        .collect::<ExternResult<Vec<_>>>()?;

    let completion_views = completion_views_for_hearth(&input.hearth_hash)?;
    let completion_records = completion_views
        .into_iter()
        .filter(|view| {
            view.completion.completed_at >= input.epoch_start
                && view.completion.completed_at < input.epoch_end
        })
        .map(|view| {
            Ok(CompletionRecord {
                record_id: RecordId(view.action_hash.to_string()),
                completion: completion_to_ledger(&view.completion)?,
            })
        })
        .collect::<ExternResult<Vec<_>>>()?;

    build_digest_v2(&occurrence_records, &completion_records)
        .map_err(|error| ledger_error("Cannot build CareDigestV2", error))
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Append-only Care recurrence revision coordinator.

use hdk::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_care_ledger_integrity::{CareRecurrenceRevisionEntry, EntryTypes, LinkTypes};
use hearth_care_recurrence::CareRecurrenceSpec;
use hearth_care_recurrence_state::{
    derive_effective_recurrence_state, CareRecurrenceDefinition, RecurrenceRevision,
    RecurrenceRevisionRecord, RECURRENCE_REVISION_SCHEMA_VERSION,
};
use hearth_coordinator_common::{decode_zome_response, get_latest_record, require_membership};
use hearth_types::{CareScheduleStatus, MemberRole};
use mycelix_bridge_common::civic_requirement_basic;
use std::collections::BTreeSet;

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
pub struct CreateCareRecurrenceRevisionInput {
    pub schedule_hash: ActionHash,
    pub parent_state_hash: Option<EntryHash>,
    pub definition: CareRecurrenceDefinition,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CareRecurrenceRevisionView {
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub revision: CareRecurrenceRevisionEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EffectiveCareRecurrenceView {
    pub schedule_hash: ActionHash,
    pub root_state_hash: EntryHash,
    pub effective_state_hash: EntryHash,
    pub effective_spec: CareRecurrenceSpec,
    pub revision_count: u32,
    pub duplicate_record_count: u32,
    pub orphan_record_count: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreatedCareRecurrenceRevision {
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub reused_existing_content: bool,
    pub effective: EffectiveCareRecurrenceView,
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

fn require_recurrence_editor(
    schedule_hash: &ActionHash,
    hearth_hash: &ActionHash,
) -> ExternResult<()> {
    let effective = effective_assignment(schedule_hash)?;
    if effective.hearth_hash != *hearth_hash || effective.schedule_root_hash != *schedule_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Effective assignment does not belong to this Care schedule".into()
        )));
    }
    let caller = agent_info()?.agent_initial_pubkey;
    if caller == effective.assignee {
        return Ok(());
    }

    let caller_role: Option<MemberRole> = decode_zome_response(
        call(
            CallTargetCell::Local,
            ZomeName::new("hearth_kinship"),
            FunctionName::new("get_caller_role"),
            None,
            hearth_hash.clone(),
        )?,
        "hearth_kinship::get_caller_role",
    )?;
    if caller_role.is_some_and(|role| role.is_guardian()) {
        return Ok(());
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Only the current Care assignee or a guardian may revise recurrence".into()
    )))
}

fn recurrence_views(schedule_hash: &ActionHash) -> ExternResult<Vec<CareRecurrenceRevisionView>> {
    let links = get_links(
        LinkQuery::try_new(
            schedule_hash.clone(),
            LinkTypes::ScheduleToRecurrenceRevisions,
        )?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut views = Vec::new();
    for link in links {
        let entry_hash = EntryHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid Care recurrence revision link target".into()
            ))
        })?;
        if !seen.insert(entry_hash.to_string()) {
            continue;
        }
        let Some(record) = get(entry_hash.clone(), GetOptions::default())? else {
            continue;
        };
        let revision: CareRecurrenceRevisionEntry =
            entry_from_record(&record, "CareRecurrenceRevisionEntry")?;
        if revision.schedule_hash != *schedule_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care recurrence revision link crosses schedule boundaries".into()
            )));
        }
        views.push(CareRecurrenceRevisionView {
            action_hash: record.action_address().clone(),
            entry_hash,
            revision,
        });
    }
    views.sort_by(|left, right| left.entry_hash.to_string().cmp(&right.entry_hash.to_string()));
    Ok(views)
}

fn pure_record(view: &CareRecurrenceRevisionView) -> RecurrenceRevisionRecord {
    RecurrenceRevisionRecord {
        state_ref: view.entry_hash.to_string(),
        revision: RecurrenceRevision {
            schema_version: view.revision.schema_version,
            schedule_ref: view.revision.schedule_hash.to_string(),
            parent_state_ref: view
                .revision
                .parent_state_hash
                .as_ref()
                .map(ToString::to_string),
            definition: view.revision.definition.clone(),
        },
    }
}

fn derive_effective_view(
    schedule_hash: &ActionHash,
    views: &[CareRecurrenceRevisionView],
) -> ExternResult<EffectiveCareRecurrenceView> {
    let pure = views.iter().map(pure_record).collect::<Vec<_>>();
    let effective = derive_effective_recurrence_state(&pure).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot establish authoritative Care recurrence state: {error:?}"
        )))
    })?;
    let root_state_hash = EntryHash::try_from(effective.root_state_ref.clone()).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid canonical recurrence root reference: {error}"
        )))
    })?;
    let effective_state_hash = EntryHash::try_from(effective.effective_state_ref.clone())
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid canonical recurrence state reference: {error}"
            )))
        })?;
    Ok(EffectiveCareRecurrenceView {
        schedule_hash: schedule_hash.clone(),
        root_state_hash,
        effective_state_hash,
        effective_spec: effective.effective_spec,
        revision_count: effective.revision_count,
        duplicate_record_count: effective.duplicate_record_count,
        orphan_record_count: effective.orphan_record_count,
    })
}

#[hdk_extern]
pub fn get_care_recurrence_revisions(
    schedule_hash: ActionHash,
) -> ExternResult<Vec<CareRecurrenceRevisionView>> {
    let schedule = latest_schedule(&schedule_hash)?;
    require_membership(&schedule.hearth_hash)?;
    recurrence_views(&schedule_hash)
}

#[hdk_extern]
pub fn get_effective_care_recurrence(
    schedule_hash: ActionHash,
) -> ExternResult<EffectiveCareRecurrenceView> {
    let schedule = latest_schedule(&schedule_hash)?;
    require_membership(&schedule.hearth_hash)?;
    let views = recurrence_views(&schedule_hash)?;
    derive_effective_view(&schedule_hash, &views)
}

#[hdk_extern]
pub fn create_care_recurrence_revision(
    input: CreateCareRecurrenceRevisionInput,
) -> ExternResult<CreatedCareRecurrenceRevision> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "create_care_recurrence_revision",
    )?;

    let schedule = latest_schedule(&input.schedule_hash)?;
    require_membership(&schedule.hearth_hash)?;
    if schedule.status == CareScheduleStatus::Completed {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Completed CareSchedule templates cannot receive recurrence revisions".into()
        )));
    }
    require_recurrence_editor(&input.schedule_hash, &schedule.hearth_hash)?;

    input.definition.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid Care recurrence definition: {error:?}"
        )))
    })?;

    let before = recurrence_views(&input.schedule_hash)?;
    if before.is_empty() {
        if input.parent_state_hash.is_some() {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Initial Care recurrence revision must not name a parent".into()
            )));
        }
    } else {
        let current = derive_effective_view(&input.schedule_hash, &before)?;
        if input.parent_state_hash.as_ref() != Some(&current.effective_state_hash) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care recurrence parent is stale; refresh authoritative recurrence state"
                    .into()
            )));
        }
        let current_definition = CareRecurrenceDefinition::from_spec(&current.effective_spec);
        if current_definition == input.definition {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care recurrence revision is a semantic no-op".into()
            )));
        }
    }

    let revision = CareRecurrenceRevisionEntry {
        schema_version: RECURRENCE_REVISION_SCHEMA_VERSION,
        hearth_hash: schedule.hearth_hash.clone(),
        schedule_hash: input.schedule_hash.clone(),
        parent_state_hash: input.parent_state_hash.clone(),
        definition: input.definition,
    };
    let entry = EntryTypes::CareRecurrenceRevision(revision.clone());
    let entry_hash = hash_entry(&entry)?;

    let (action_hash, reused_existing_content) = if let Some(existing) =
        get(entry_hash.clone(), GetOptions::default())?
    {
        let existing_revision: CareRecurrenceRevisionEntry =
            entry_from_record(&existing, "CareRecurrenceRevisionEntry")?;
        if existing_revision != revision {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care recurrence content hash resolved to unexpected semantics".into()
            )));
        }
        (existing.action_address().clone(), true)
    } else {
        (create_entry(&entry)?, false)
    };

    // Always repair discovery after a retry/crash. Duplicate links are harmless
    // because readers deduplicate by EntryHash.
    create_link(
        input.schedule_hash.clone(),
        entry_hash.clone(),
        LinkTypes::ScheduleToRecurrenceRevisions,
        (),
    )?;

    // Re-read after the durable write. Concurrent identical revisions converge
    // on one EntryHash; competing children fail closed in the pure theorem.
    let after = recurrence_views(&input.schedule_hash)?;
    let effective = derive_effective_view(&input.schedule_hash, &after)?;
    if effective.effective_state_hash != entry_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "New Care recurrence revision is not the authoritative head after write"
                .into()
        )));
    }

    Ok(CreatedCareRecurrenceRevision {
        action_hash,
        entry_hash,
        reused_existing_content,
        effective,
    })
}

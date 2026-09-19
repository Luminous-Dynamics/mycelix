// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Append-only Care occurrence/completion evidence for Hearth.
//!
//! This zome intentionally does not redefine `CareSchedule`; the legacy Care
//! zome remains the durable template source. Concrete recurring work lives here.

use hdi::prelude::*;
use hearth_automation_integrity::AssignmentTransitionEntry;
use hearth_care_integrity::CareSchedule;
use hearth_care_ledger::{
    CareCompletion as LedgerCompletion, CareOccurrence as LedgerOccurrence, MemberId,
    OccurrenceAssignmentBinding, OccurrenceId, OccurrenceWindow, ScheduleId,
    CARE_LEDGER_SCHEMA_VERSION,
};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareOccurrenceEntry {
    pub hearth_hash: ActionHash,
    /// Stable root ActionHash of the source CareSchedule template.
    pub schedule_hash: ActionHash,
    pub occurrence_id: String,
    /// Snapshot of the accepted assignee for this concrete occurrence.
    pub assigned_to: AgentPubKey,
    pub window_start: Timestamp,
    pub window_end: Timestamp,
    pub estimated_minutes: Option<u32>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareCompletionEntry {
    pub hearth_hash: ActionHash,
    /// A valid action containing the referenced immutable occurrence.
    pub occurrence_action_hash: ActionHash,
    /// Content identity of that occurrence entry.
    pub occurrence_entry_hash: EntryHash,
    pub occurrence_id: String,
    /// Member who actually performed the work.
    pub performed_by: AgentPubKey,
    /// Agent who authored/attested this completion record.
    pub recorded_by: AgentPubKey,
    pub completed_at: Timestamp,
    pub actual_minutes: Option<u32>,
    pub evidence_refs: Vec<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CareOccurrence(CareOccurrenceEntry),
    CareCompletion(CareCompletionEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    ScheduleToOccurrences,
    HearthToOccurrences,
    OccurrenceToCompletions,
    HearthToCompletions,
    /// Immutable authority binding: CareOccurrence EntryHash -> exact assignment state ActionHash.
    OccurrenceToAssignmentState,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => match app_entry {
            EntryTypes::CareOccurrence(entry) => validate_occurrence(&entry),
            EntryTypes::CareCompletion(entry) => validate_completion(&entry, &action.author),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) => Ok(ValidateCallbackResult::Invalid(
            "Care ledger occurrence/completion evidence is immutable".into(),
        )),
        FlatOp::StoreEntry(_) | FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            ..
        } => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Care ledger link tag too long (max 256 bytes)".into(),
                ));
            }
            match link_type {
                LinkTypes::OccurrenceToAssignmentState => {
                    validate_occurrence_assignment_state_link(base_address, target_address)
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Care ledger evidence links are append-only".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Care ledger entries cannot be updated".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Care ledger entries cannot be deleted".into(),
        )),
    }
}

fn occurrence_to_ledger(entry: &CareOccurrenceEntry) -> LedgerOccurrence {
    LedgerOccurrence {
        schema_version: CARE_LEDGER_SCHEMA_VERSION,
        id: OccurrenceId(entry.occurrence_id.clone()),
        schedule_id: ScheduleId(entry.schedule_hash.to_string()),
        assigned_to: MemberId(entry.assigned_to.to_string()),
        window: OccurrenceWindow {
            start_micros: entry.window_start.as_micros(),
            end_micros: entry.window_end.as_micros(),
        },
        estimated_minutes: entry.estimated_minutes,
    }
}

fn validate_occurrence(entry: &CareOccurrenceEntry) -> ExternResult<ValidateCallbackResult> {
    match occurrence_to_ledger(entry).validate() {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid CareOccurrence: {error:?}"
        ))),
    }
}

fn validate_occurrence_assignment_state_link(
    base: AnyLinkableHash,
    target: AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let occurrence_entry_hash = EntryHash::try_from(base).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "OccurrenceToAssignmentState base must be a CareOccurrence EntryHash".into(),
        ))
    })?;
    let assignment_state_hash = ActionHash::try_from(target).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "OccurrenceToAssignmentState target must be an assignment-state ActionHash".into(),
        ))
    })?;

    let occurrence_hashed = must_get_entry(occurrence_entry_hash)?;
    let app_bytes = occurrence_hashed
        .content
        .as_app_entry()
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "OccurrenceToAssignmentState base is not an app entry".into()
        )))?;
    let occurrence = CareOccurrenceEntry::try_from(app_bytes.as_ref().clone())
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(format!(
            "OccurrenceToAssignmentState base is not a CareOccurrence: {error}"
        ))))?;
    let ledger_occurrence = occurrence_to_ledger(&occurrence);
    if let Err(error) = ledger_occurrence.validate() {
        return invalid(format!("Invalid bound CareOccurrence: {error:?}"));
    }
    if let Err(error) = OccurrenceAssignmentBinding::from_occurrence(
        &ledger_occurrence,
        assignment_state_hash.to_string(),
    ) {
        return invalid(format!("Invalid occurrence assignment binding: {error}"));
    }

    let state_record = must_get_valid_record(assignment_state_hash.clone())?;
    let schedule_decode: Result<Option<CareSchedule>, SerializedBytesError> =
        state_record.entry().to_app_option();
    if let Ok(Some(schedule)) = schedule_decode {
        if schedule.hearth_hash != occurrence.hearth_hash {
            return invalid("Bound CareSchedule state belongs to a different hearth");
        }
        if schedule.assigned_to != occurrence.assigned_to {
            return invalid("Bound CareSchedule state names a different assignee");
        }
        return validate_schedule_lineage(&assignment_state_hash, &occurrence.schedule_hash);
    }

    let transition_decode: Result<Option<AssignmentTransitionEntry>, SerializedBytesError> =
        state_record.entry().to_app_option();
    if let Ok(Some(transition)) = transition_decode {
        if transition.hearth_hash != occurrence.hearth_hash
            || transition.schedule_root_hash != occurrence.schedule_hash
        {
            return invalid("Bound AssignmentTransition belongs to another hearth/schedule");
        }
        if transition.to_assignee != occurrence.assigned_to
            || transition.transition.to_assignee != occurrence.assigned_to.to_string()
        {
            return invalid("Bound AssignmentTransition names a different assignee");
        }
        return Ok(ValidateCallbackResult::Valid);
    }

    invalid("Occurrence assignment state must reference CareSchedule or AssignmentTransition evidence")
}

fn validate_schedule_lineage(
    state_hash: &ActionHash,
    root_hash: &ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let mut current = state_hash.clone();
    for _ in 0..64 {
        if &current == root_hash {
            return Ok(ValidateCallbackResult::Valid);
        }
        let signed = must_get_action(current)?;
        match signed.action() {
            Action::Update(update) => current = update.original_action_address.clone(),
            _ => return invalid("Bound CareSchedule state is not descended from claimed schedule root"),
        }
    }
    invalid("CareSchedule assignment-state lineage exceeds safety bound")
}

fn validate_completion(
    entry: &CareCompletionEntry,
    author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    if &entry.recorded_by != author {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion.recorded_by must equal the action author".into(),
        ));
    }

    let occurrence_record = must_get_valid_record(entry.occurrence_action_hash.clone())?;
    let occurrence: CareOccurrenceEntry = occurrence_record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "CareCompletion must reference a CareOccurrence entry".into(),
            ))
        })?;
    let expected_entry_hash = hash_entry(&EntryTypes::CareOccurrence(occurrence.clone()))?;
    if expected_entry_hash != entry.occurrence_entry_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion occurrence ActionHash/EntryHash binding mismatch".into(),
        ));
    }
    if occurrence.hearth_hash != entry.hearth_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion hearth does not match referenced occurrence".into(),
        ));
    }
    if occurrence.occurrence_id != entry.occurrence_id {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion occurrence_id does not match referenced occurrence".into(),
        ));
    }
    if occurrence.assigned_to != entry.performed_by {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion performer must match the occurrence assignee snapshot".into(),
        ));
    }

    let ledger = LedgerCompletion {
        schema_version: CARE_LEDGER_SCHEMA_VERSION,
        occurrence_id: OccurrenceId(entry.occurrence_id.clone()),
        performed_by: MemberId(entry.performed_by.to_string()),
        recorded_by: MemberId(entry.recorded_by.to_string()),
        completed_at_micros: entry.completed_at.as_micros(),
        actual_minutes: entry.actual_minutes,
        evidence_refs: entry.evidence_refs.clone(),
    };
    match ledger.validate() {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid CareCompletion: {error:?}"
        ))),
    }
}

fn invalid(message: impl Into<String>) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(message.into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_care_ledger::{derive_occurrence_id, OccurrenceWindow, ScheduleId};

    fn agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn action(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn occurrence() -> CareOccurrenceEntry {
        let schedule_hash = action(2);
        let window = OccurrenceWindow {
            start_micros: 10,
            end_micros: 20,
        };
        let id = derive_occurrence_id(&ScheduleId(schedule_hash.to_string()), &window).unwrap();
        CareOccurrenceEntry {
            hearth_hash: action(1),
            schedule_hash,
            occurrence_id: id.0,
            assigned_to: agent(3),
            window_start: Timestamp::from_micros(10),
            window_end: Timestamp::from_micros(20),
            estimated_minutes: Some(15),
        }
    }

    #[test]
    fn deterministic_occurrence_passes() {
        assert_eq!(
            validate_occurrence(&occurrence()).unwrap(),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn forged_occurrence_id_fails() {
        let mut value = occurrence();
        value.occurrence_id = "wrong".into();
        assert!(matches!(
            validate_occurrence(&value).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
    }
}

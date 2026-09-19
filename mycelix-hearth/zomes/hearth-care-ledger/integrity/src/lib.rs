// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Append-only Care occurrence/completion evidence for Hearth.
//!
//! This zome intentionally does not redefine `CareSchedule`; the legacy Care
//! zome remains the durable template source. Concrete recurring work lives here.

use hdi::prelude::*;
use hearth_care_ledger::{
    CareCompletion as LedgerCompletion, CareOccurrence as LedgerOccurrence, MemberId,
    OccurrenceId, OccurrenceWindow, ScheduleId, CARE_LEDGER_SCHEMA_VERSION,
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
        FlatOp::RegisterCreateLink { tag, .. } => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Care ledger link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
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

fn validate_occurrence(entry: &CareOccurrenceEntry) -> ExternResult<ValidateCallbackResult> {
    let ledger = LedgerOccurrence {
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
    match ledger.validate() {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid CareOccurrence: {error:?}"
        ))),
    }
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
                "CareCompletion must reference a CareOccurrence entry".into()
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

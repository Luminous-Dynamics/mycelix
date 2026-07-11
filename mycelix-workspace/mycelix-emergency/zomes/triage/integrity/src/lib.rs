// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Triage Integrity Zome
//! Mass casualty triage records using START protocol

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// A triage assessment record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TriageRecord {
    pub disaster_hash: ActionHash,
    pub patient_id: String,
    pub patient_hash: Option<ActionHash>,
    pub category: TriageCategory,
    pub injuries: String,
    pub location: String,
    pub timestamp: Timestamp,
    pub triaged_by: AgentPubKey,
    pub transport_priority: TransportPriority,
    pub notes: String,
}

/// START triage categories
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TriageCategory {
    /// Immediate - life-threatening, salvageable
    Immediate,
    /// Delayed - serious but can wait
    Delayed,
    /// Minor - walking wounded
    Minor,
    /// Expectant - unlikely to survive
    Expectant,
    /// Deceased
    Dead,
}

/// Transport priority
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TransportPriority {
    Urgent,
    Priority,
    Routine,
    None,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    TriageRecord(TriageRecord),
}

#[hdk_link_types]
pub enum LinkTypes {
    DisasterToTriage,
    CategoryToTriage,
    PatientToTriage,
    AgentToTriage,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::TriageRecord(record) => validate_create_triage(action, record),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::DisasterToTriage => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CategoryToTriage => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PatientToTriage => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToTriage => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): the
        // coordinator's update_triage (re-triage) deletes the OLD
        // CategoryToTriage link, which was created by whoever FIRST
        // triaged the patient -- not necessarily the same agent
        // performing the re-triage. Adding the usual "only original link
        // creator can delete" check here would break that legitimate
        // flow. Same reasoning as the other zomes in this cluster with
        // genuine cross-agent link-deletion flows.
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 15th confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry here, so this is pure hardening, zero
            // functional impact.
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Shared per-entry-type update validation, called from BOTH the
/// StoreEntry (OpEntry::UpdateEntry) and RegisterUpdate DHT-op
/// perspectives so they agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::TriageRecord(record) => validate_update_triage(action, record),
    }
}

fn validate_create_triage(
    action: Create,
    record: TriageRecord,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's triage_patient already derives
    // triaged_by from agent_info(), so this is belt-and-suspenders against
    // a modified coordinator forging a victim agent as the triager. Found
    // + fixed 2026-07-09 during the P0 author-binding pass.
    if record.triaged_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Triage record triaged_by must correspond to the committing agent".into(),
        ));
    }

    if record.patient_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Patient ID cannot be empty".into(),
        ));
    }
    if record.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Location cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a TriageRecord update (re-triage). Unlike every other
/// update validator fixed in this cluster, an author requirement DOES
/// apply here: the coordinator's update_triage always re-derives
/// triaged_by from the CALLING agent's agent_info() (a re-triage is
/// attributed to whoever performs it, which may legitimately be a
/// different medic than the original triager -- that's why triaged_by
/// itself, not the original author, is what's checked). Content is
/// restricted to category/injuries/timestamp/triaged_by/
/// transport_priority/notes -- this closes the wide-open bug that
/// previously let disaster_hash/patient_id/patient_hash/location change
/// unconditionally on update too.
fn validate_update_triage(
    action: Update,
    record: TriageRecord,
) -> ExternResult<ValidateCallbackResult> {
    if record.triaged_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Triage record triaged_by must correspond to the committing agent".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: TriageRecord = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original triage record not found".into()
        )))?;

    if record.disaster_hash != original.disaster_hash
        || record.patient_id != original.patient_id
        || record.patient_hash != original.patient_hash
        || record.location != original.location
    {
        return Ok(ValidateCallbackResult::Invalid(
            "disaster_hash/patient_id/patient_hash/location cannot change on a triage update"
                .into(),
        ));
    }

    if record.patient_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Patient ID cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn create_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn update_action(author: AgentPubKey) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_record(triaged_by: AgentPubKey) -> TriageRecord {
        TriageRecord {
            disaster_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            patient_id: "p-1".into(),
            patient_hash: None,
            category: TriageCategory::Delayed,
            injuries: "laceration".into(),
            location: "triage-tent-1".into(),
            timestamp: Timestamp::from_micros(0),
            triaged_by,
            transport_priority: TransportPriority::Routine,
            notes: "".into(),
        }
    }

    #[test]
    fn create_triage_valid_when_triager_matches_committer() {
        let r = valid_record(me());
        let result = validate_create_triage(create_action(me()), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_triage_forgery_rejected() {
        let r = valid_record(me());
        let result = validate_create_triage(create_action(other_agent()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_triage_forgery_rejected_before_must_get() {
        // triaged_by must match the committing agent even on re-triage;
        // this is checked before must_get_valid_record, so it's testable
        // without a live HDI host.
        let r = valid_record(me());
        let result = validate_update_triage(update_action(other_agent()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}

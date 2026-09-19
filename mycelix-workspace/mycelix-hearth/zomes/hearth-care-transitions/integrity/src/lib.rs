// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Immutable Care lifecycle transition evidence.
//!
//! This zome is deliberately isolated from legacy `hearth_care_integrity` so
//! v2 evidence can remain append-only while legacy mutable records coexist
//! during migration.

use hdi::prelude::*;
use hearth_authority_hdi::require_fresh_active_membership_from_chain;
use hearth_care_integrity::CareSchedule;

const LEGACY_CARE_INTEGRITY_ZOME: &str = "hearth_care_integrity";
const LEGACY_CARE_SCHEDULE_ENTRY_INDEX: u8 = 0;

/// Immutable evidence that a CareSchedule was completed by `actor`.
///
/// Transition time is not duplicated in this entry. The signed Holochain
/// Create action timestamp is the canonical completion timestamp.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareCompletion {
    pub hearth_hash: ActionHash,
    pub schedule_hash: ActionHash,
    pub assignee: AgentPubKey,
    pub actor: AgentPubKey,
    pub actor_membership_hash: ActionHash,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CareCompletion(CareCompletion),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Legacy CareSchedule ActionHash -> immutable CareCompletion ActionHash.
    ScheduleToCompletions,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CareCompletion(completion) => {
                    validate_care_completion(&completion, &action)
                }
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Care transition evidence is immutable and cannot be updated".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            action,
        } => {
            if !tag.0.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Care transition index links must use an empty tag".into(),
                ));
            }
            match link_type {
                LinkTypes::ScheduleToCompletions => validate_schedule_completion_link(
                    base_address,
                    target_address,
                    &action.author,
                ),
            }
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Care transition evidence links are append-only and cannot be deleted".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Care transition evidence is immutable and cannot be updated".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Care transition evidence is immutable and cannot be deleted".into(),
        )),
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_care_completion(
    completion: &CareCompletion,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if action.author != completion.actor {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion.actor must equal the Create action author".into(),
        ));
    }

    let schedule_record = must_get_valid_record(completion.schedule_hash.clone())?;
    let schedule_entry_def = match schedule_record.action().app_entry_def() {
        Some(entry_def) => entry_def,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "CareCompletion.schedule_hash must reference an application entry".into(),
            ));
        }
    };

    let dna = dna_info()?;
    if !is_legacy_care_schedule_entry_def(schedule_entry_def, &dna.zome_names) {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion.schedule_hash must reference hearth_care_integrity::CareSchedule"
                .into(),
        ));
    }

    let schedule: CareSchedule = schedule_record
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode referenced CareSchedule: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Referenced CareSchedule record is missing its entry".into(),
            ))
        })?;

    if schedule.hearth_hash != completion.hearth_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion Hearth does not match referenced CareSchedule".into(),
        ));
    }
    if schedule.assigned_to != completion.assignee {
        return Ok(ValidateCallbackResult::Invalid(
            "CareCompletion assignee does not match referenced CareSchedule".into(),
        ));
    }

    // IMPORTANT: legacy CareSchedule.status is mutable. This validated schedule
    // revision binds identity only; it is not strengthened into proof of the
    // latest legacy lifecycle state. #2016 owns coexistence semantics.
    let role = match require_fresh_active_membership_from_chain(
        action,
        &completion.actor,
        &completion.hearth_hash,
        &completion.actor_membership_hash,
    )? {
        Ok(role) => role,
        Err(error) => {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "CareCompletion membership authority is invalid: {error}"
            )));
        }
    };

    if completion.actor != completion.assignee && !role.is_guardian() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the assignee or a current guardian may author CareCompletion evidence".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_schedule_completion_link(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    link_author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    let schedule_hash = match ActionHash::try_from(base_address) {
        Ok(hash) => hash,
        Err(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "ScheduleToCompletions base must be a CareSchedule ActionHash".into(),
            ));
        }
    };
    let completion_hash = match ActionHash::try_from(target_address) {
        Ok(hash) => hash,
        Err(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "ScheduleToCompletions target must be a CareCompletion ActionHash".into(),
            ));
        }
    };

    let completion_record = must_get_valid_record(completion_hash)?;
    if completion_record.action().author() != link_author {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleToCompletions link must be authored by the CareCompletion actor".into(),
        ));
    }

    let completion: CareCompletion = completion_record
        .entry()
        .to_app_option()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode CareCompletion link target: {error}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "CareCompletion link target is missing its entry".into(),
            ))
        })?;

    if completion.actor != *link_author {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleToCompletions link author does not match CareCompletion.actor".into(),
        ));
    }
    if completion.schedule_hash != schedule_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "ScheduleToCompletions target references a different CareSchedule".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Bind a referenced AppEntryDef to the historical legacy CareSchedule type.
///
/// The zome position is discovered from deterministic DNA metadata rather than
/// hard-coded. Entry index 0 is the published legacy CareSchedule index and is
/// intentionally treated as a protocol constant; changing legacy Care's enum
/// order would already be a wire-compatibility break.
fn is_legacy_care_schedule_entry_def(
    entry_def: &AppEntryDef,
    integrity_zome_names: &[ZomeName],
) -> bool {
    let zome_name = integrity_zome_names.get(entry_def.zome_index.0 as usize);
    zome_name == Some(&ZomeName::new(LEGACY_CARE_INTEGRITY_ZOME))
        && entry_def.entry_index.0 == LEGACY_CARE_SCHEDULE_ENTRY_INDEX
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_types::MemberRole;

    fn app_entry_def(zome_index: u8, entry_index: u8) -> AppEntryDef {
        AppEntryDef {
            zome_index: zome_index.into(),
            entry_index: entry_index.into(),
            visibility: EntryVisibility::Public,
        }
    }

    fn zomes() -> Vec<ZomeName> {
        vec![
            ZomeName::new("hearth_kinship_integrity"),
            ZomeName::new(LEGACY_CARE_INTEGRITY_ZOME),
            ZomeName::new("hearth_care_transitions_integrity"),
        ]
    }

    #[test]
    fn legacy_schedule_type_provenance_accepts_exact_zome_and_entry() {
        assert!(is_legacy_care_schedule_entry_def(
            &app_entry_def(1, LEGACY_CARE_SCHEDULE_ENTRY_INDEX),
            &zomes(),
        ));
    }

    #[test]
    fn legacy_schedule_type_provenance_rejects_same_entry_index_from_other_zome() {
        assert!(!is_legacy_care_schedule_entry_def(
            &app_entry_def(0, LEGACY_CARE_SCHEDULE_ENTRY_INDEX),
            &zomes(),
        ));
    }

    #[test]
    fn legacy_schedule_type_provenance_rejects_other_entry_in_care_zome() {
        assert!(!is_legacy_care_schedule_entry_def(
            &app_entry_def(1, 1),
            &zomes(),
        ));
    }

    #[test]
    fn legacy_schedule_type_provenance_rejects_out_of_range_zome_index() {
        assert!(!is_legacy_care_schedule_entry_def(
            &app_entry_def(9, LEGACY_CARE_SCHEDULE_ENTRY_INDEX),
            &zomes(),
        ));
    }

    #[test]
    fn guardian_policy_matches_current_hearth_roles() {
        assert!(MemberRole::Founder.is_guardian());
        assert!(MemberRole::Elder.is_guardian());
        assert!(MemberRole::Adult.is_guardian());
        assert!(!MemberRole::Youth.is_guardian());
        assert!(!MemberRole::Child.is_guardian());
        assert!(!MemberRole::Guest.is_guardian());
        assert!(!MemberRole::Ancestor.is_guardian());
    }
}

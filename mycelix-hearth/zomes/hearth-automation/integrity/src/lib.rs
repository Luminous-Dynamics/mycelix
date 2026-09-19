// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Durable household automation proposal/consent evidence.
//!
//! HTH-AUTO-005A2 records proposals and responses only. It deliberately has
//! no entry or extern that can mutate Care assignments.

use hdi::prelude::*;
use hearth_care_integrity::CareSchedule;
use hearth_work_proposal::{
    AssignmentProposalId, HouseholdWorkProposal, WorkProposalResponse,
};
use std::collections::{BTreeMap, BTreeSet};

const ACTION_TIME_TOLERANCE_MICROS: u64 = 60_000_000;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WorkProposalScheduleBinding {
    pub assignment_id: AssignmentProposalId,
    pub occurrence_id: String,
    /// Stable/root ActionHash of the source CareSchedule.
    pub schedule_root_hash: ActionHash,
    /// Exact schedule state/version on which consent was requested.
    pub assignment_state_hash: ActionHash,
    pub current_assignee: AgentPubKey,
    pub proposed_assignee: AgentPubKey,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WorkProposalEntry {
    pub hearth_hash: ActionHash,
    pub proposal: HouseholdWorkProposal,
    pub bindings: Vec<WorkProposalScheduleBinding>,
    pub created_by: AgentPubKey,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WorkProposalResponseEntry {
    pub hearth_hash: ActionHash,
    pub proposal_action_hash: ActionHash,
    pub response: WorkProposalResponse,
    pub responder: AgentPubKey,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    WorkProposal(WorkProposalEntry),
    WorkProposalResponse(WorkProposalResponseEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    HearthToWorkProposals,
    AgentToWorkProposals,
    ProposalToResponses,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => match app_entry {
            EntryTypes::WorkProposal(entry) => validate_work_proposal(&entry, &action),
            EntryTypes::WorkProposalResponse(entry) => validate_work_response(&entry, &action),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) => Ok(ValidateCallbackResult::Invalid(
            "Automation proposal/response evidence is immutable".into(),
        )),
        FlatOp::StoreEntry(_) | FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            action,
        } => validate_create_link(
            link_type,
            base_address,
            target_address,
            &tag,
            &action.author,
        ),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Automation evidence links are append-only".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Automation proposal/response entries cannot be updated".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Automation proposal/response entries cannot be deleted".into(),
        )),
    }
}

fn validate_work_proposal(
    entry: &WorkProposalEntry,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if entry.created_by != action.author {
        return invalid("WorkProposal.created_by must equal the action author");
    }
    if let Err(error) = entry.proposal.validate() {
        return invalid(format!("Invalid household work proposal: {error}"));
    }
    if entry.proposal.hearth_id != entry.hearth_hash.to_string() {
        return invalid("WorkProposal hearth_id does not match hearth_hash");
    }
    if !timestamps_close(entry.proposal.created_at_micros, action.timestamp.as_micros()) {
        return invalid("WorkProposal created_at is not bound closely enough to action timestamp");
    }
    if action.timestamp.as_micros() >= entry.proposal.expires_at_micros {
        return invalid("WorkProposal action timestamp must precede proposal expiry");
    }
    if entry.bindings.len() != entry.proposal.assignments.len() {
        return invalid("WorkProposal bindings must cover every assignment exactly once");
    }

    let mut bindings = BTreeMap::new();
    for binding in &entry.bindings {
        if bindings.insert(binding.assignment_id.clone(), binding).is_some() {
            return invalid("WorkProposal contains duplicate assignment bindings");
        }
    }

    for assignment in &entry.proposal.assignments {
        let Some(binding) = bindings.get(&assignment.id) else {
            return invalid("WorkProposal assignment is missing its typed schedule binding");
        };
        if assignment.occurrence_id != binding.occurrence_id {
            return invalid("WorkProposal occurrence binding mismatch");
        }
        if assignment.source_schedule_ref != binding.schedule_root_hash.to_string() {
            return invalid("WorkProposal source schedule binding mismatch");
        }
        if assignment.assignment_state_ref != binding.assignment_state_hash.to_string() {
            return invalid("WorkProposal assignment-state binding mismatch");
        }
        if assignment.current_assignee.as_deref()
            != Some(binding.current_assignee.to_string().as_str())
        {
            return invalid("WorkProposal current-assignee binding mismatch");
        }
        if assignment.proposed_assignee != binding.proposed_assignee.to_string() {
            return invalid("WorkProposal proposed-assignee binding mismatch");
        }

        let state_record = must_get_valid_record(binding.assignment_state_hash.clone())?;
        let schedule: CareSchedule = decode_entry(&state_record, "CareSchedule")?;
        if schedule.hearth_hash != entry.hearth_hash {
            return invalid("Bound CareSchedule belongs to a different hearth");
        }
        if schedule.assigned_to != binding.current_assignee {
            return invalid("Bound CareSchedule assignee does not match proposal snapshot");
        }
        let lineage = validate_schedule_lineage(
            &binding.assignment_state_hash,
            &binding.schedule_root_hash,
        )?;
        if lineage != ValidateCallbackResult::Valid {
            return Ok(lineage);
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_work_response(
    entry: &WorkProposalResponseEntry,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if entry.responder != action.author {
        return invalid("WorkProposalResponse.responder must equal the action author");
    }

    let proposal_record = must_get_valid_record(entry.proposal_action_hash.clone())?;
    let proposal_entry: WorkProposalEntry = decode_entry(&proposal_record, "WorkProposalEntry")?;
    if proposal_entry.hearth_hash != entry.hearth_hash {
        return invalid("WorkProposalResponse hearth does not match its proposal");
    }
    if entry.response.responder_id != entry.responder.to_string() {
        return invalid("WorkProposalResponse responder_id does not match responder key");
    }
    if let Err(error) = entry.response.validate_against(&proposal_entry.proposal) {
        return invalid(format!("Invalid work proposal response: {error}"));
    }

    let action_micros = action.timestamp.as_micros();
    if action_micros < proposal_entry.proposal.created_at_micros
        || action_micros >= proposal_entry.proposal.expires_at_micros
    {
        return invalid("WorkProposalResponse action timestamp is outside proposal validity");
    }
    if !timestamps_close(entry.response.responded_at_micros, action_micros) {
        return invalid("WorkProposalResponse responded_at is not bound closely enough to action timestamp");
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_link(
    link_type: LinkTypes,
    base: AnyLinkableHash,
    target: AnyLinkableHash,
    tag: &LinkTag,
    author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    if tag.0.len() > 256 {
        return invalid("Automation link tag too long (max 256 bytes)");
    }

    match link_type {
        LinkTypes::HearthToWorkProposals => {
            let hearth_hash = ActionHash::try_from(base)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid hearth link base".into())))?;
            let proposal_hash = ActionHash::try_from(target)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid proposal link target".into())))?;
            let record = must_get_valid_record(proposal_hash)?;
            let proposal: WorkProposalEntry = decode_entry(&record, "WorkProposalEntry")?;
            if proposal.hearth_hash != hearth_hash || &proposal.created_by != author {
                return invalid("HearthToWorkProposals link does not match proposal evidence");
            }
        }
        LinkTypes::AgentToWorkProposals => {
            let agent = AgentPubKey::try_from(base)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid agent link base".into())))?;
            let proposal_hash = ActionHash::try_from(target)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid proposal link target".into())))?;
            let record = must_get_valid_record(proposal_hash)?;
            let proposal: WorkProposalEntry = decode_entry(&record, "WorkProposalEntry")?;
            if &proposal.created_by != author || !proposal_requires(&proposal.proposal, &agent.to_string()) {
                return invalid("AgentToWorkProposals link does not identify a required responder");
            }
        }
        LinkTypes::ProposalToResponses => {
            let proposal_hash = ActionHash::try_from(base)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid proposal link base".into())))?;
            let response_hash = ActionHash::try_from(target)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid response link target".into())))?;
            let record = must_get_valid_record(response_hash)?;
            let response: WorkProposalResponseEntry = decode_entry(&record, "WorkProposalResponseEntry")?;
            if response.proposal_action_hash != proposal_hash || &response.responder != author {
                return invalid("ProposalToResponses link does not match response evidence");
            }
        }
    }
    Ok(ValidateCallbackResult::Valid)
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
            _ => return invalid("Assignment state is not descended from claimed schedule root"),
        }
    }
    invalid("CareSchedule update lineage exceeds safety bound")
}

fn proposal_requires(proposal: &HouseholdWorkProposal, member: &str) -> bool {
    proposal
        .assignments
        .iter()
        .any(|assignment| assignment.required_responders.contains(member))
}

fn timestamps_close(left: i64, right: i64) -> bool {
    left.abs_diff(right) <= ACTION_TIME_TOLERANCE_MICROS
}

fn decode_entry<T: TryFrom<SerializedBytes, Error = SerializedBytesError>>(
    record: &Record,
    type_name: &str,
) -> ExternResult<T> {
    record
        .entry()
        .to_app_option()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Referenced record is not a valid {type_name}"
            )))
        })
}

fn invalid(message: impl Into<String>) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(message.into()))
}

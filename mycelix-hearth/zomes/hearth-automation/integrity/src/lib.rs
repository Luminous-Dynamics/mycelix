// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Durable Hearth automation evidence.
//!
//! A2 records proposals/responses. A3 adds final transition endorsements and
//! append-only assignment transitions; CareSchedule itself remains the template.

use hdi::prelude::*;
use hearth_assignment_endorsement::{
    evaluate_endorsements, validate_transition_binding, AssignmentTransitionEndorsement,
    EndorsementEvaluation, EndorsementRecord, EndorsementRecordId,
    ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION,
};
use hearth_assignment_transition::{AssignmentStateRef, AssignmentTransition, TransitionId};
use hearth_care_integrity::CareSchedule;
use hearth_work_proposal::{
    AssignmentProposalId, HouseholdWorkProposal, ResponseDecision, WorkProposalResponse,
};
use std::collections::{BTreeMap, BTreeSet};

const ACTION_TIME_TOLERANCE_MICROS: u64 = 60_000_000;
const MAX_TRANSITION_ENDORSEMENTS: usize = 32;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WorkProposalScheduleBinding {
    pub assignment_id: AssignmentProposalId,
    pub occurrence_id: String,
    pub schedule_root_hash: ActionHash,
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

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AssignmentTransitionEndorsementEntry {
    pub hearth_hash: ActionHash,
    pub proposal_action_hash: ActionHash,
    pub assignment_id: AssignmentProposalId,
    pub transition_id: TransitionId,
    pub accept_response_action_hash: ActionHash,
    pub endorser: AgentPubKey,
    pub endorsed_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AssignmentTransitionEntry {
    pub hearth_hash: ActionHash,
    pub schedule_root_hash: ActionHash,
    pub previous_state_action_hash: ActionHash,
    pub proposal_action_hash: ActionHash,
    pub assignment_id: AssignmentProposalId,
    pub from_assignee: AgentPubKey,
    pub to_assignee: AgentPubKey,
    pub transition: AssignmentTransition,
    pub endorsement_action_hashes: Vec<ActionHash>,
    pub applied_by: AgentPubKey,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    WorkProposal(WorkProposalEntry),
    WorkProposalResponse(WorkProposalResponseEntry),
    AssignmentTransitionEndorsement(AssignmentTransitionEndorsementEntry),
    AssignmentTransition(AssignmentTransitionEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    HearthToWorkProposals,
    AgentToWorkProposals,
    ProposalToResponses,
    ProposalToTransitionEndorsements,
    ScheduleToAssignmentTransitions,
    ProposalToAssignmentTransitions,
    HearthToAssignmentTransitions,
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
            EntryTypes::AssignmentTransitionEndorsement(entry) => {
                validate_transition_endorsement(&entry, &action)
            }
            EntryTypes::AssignmentTransition(entry) => validate_assignment_transition(&entry, &action),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) => Ok(ValidateCallbackResult::Invalid(
            "Automation evidence is immutable".into(),
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
            "Automation evidence entries cannot be updated".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Automation evidence entries cannot be deleted".into(),
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
        let state_result = validate_assignment_state_binding(entry, binding)?;
        if state_result != ValidateCallbackResult::Valid {
            return Ok(state_result);
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_assignment_state_binding(
    proposal: &WorkProposalEntry,
    binding: &WorkProposalScheduleBinding,
) -> ExternResult<ValidateCallbackResult> {
    let state_record = must_get_valid_record(binding.assignment_state_hash.clone())?;
    let schedule_decode: Result<Option<CareSchedule>, SerializedBytesError> =
        state_record.entry().to_app_option();
    if let Ok(Some(schedule)) = schedule_decode {
        if schedule.hearth_hash != proposal.hearth_hash {
            return invalid("Bound CareSchedule belongs to a different hearth");
        }
        if schedule.assigned_to != binding.current_assignee {
            return invalid("Bound CareSchedule assignee does not match proposal snapshot");
        }
        return validate_schedule_lineage(
            &binding.assignment_state_hash,
            &binding.schedule_root_hash,
        );
    }

    let transition_decode: Result<Option<AssignmentTransitionEntry>, SerializedBytesError> =
        state_record.entry().to_app_option();
    if let Ok(Some(transition)) = transition_decode {
        if transition.hearth_hash != proposal.hearth_hash
            || transition.schedule_root_hash != binding.schedule_root_hash
        {
            return invalid("Bound AssignmentTransition belongs to another hearth/schedule");
        }
        if transition.to_assignee != binding.current_assignee
            || transition.transition.to_assignee != binding.current_assignee.to_string()
        {
            return invalid("Bound AssignmentTransition assignee does not match proposal snapshot");
        }
        return Ok(ValidateCallbackResult::Valid);
    }
    invalid("Assignment state must reference CareSchedule or AssignmentTransition evidence")
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

fn validate_transition_endorsement(
    entry: &AssignmentTransitionEndorsementEntry,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if entry.endorser != action.author {
        return invalid("AssignmentTransitionEndorsement.endorser must equal action author");
    }
    let proposal_record = must_get_valid_record(entry.proposal_action_hash.clone())?;
    let proposal_entry: WorkProposalEntry = decode_entry(&proposal_record, "WorkProposalEntry")?;
    if proposal_entry.hearth_hash != entry.hearth_hash {
        return invalid("Transition endorsement hearth does not match proposal");
    }
    let assignment = proposal_entry
        .proposal
        .assignment(&entry.assignment_id)
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Unknown endorsed assignment".into())))?;
    let binding = proposal_entry
        .bindings
        .iter()
        .find(|binding| binding.assignment_id == entry.assignment_id)
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Missing endorsed assignment binding".into())))?;
    let transition = transition_for_binding(&proposal_entry, binding, entry.endorsed_at.as_micros())?;
    if transition.id != entry.transition_id {
        return invalid("Transition endorsement transition_id mismatch");
    }
    if assignment.id.0 != transition.assignment_ref {
        return invalid("Transition endorsement assignment binding mismatch");
    }

    let accept_record = must_get_valid_record(entry.accept_response_action_hash.clone())?;
    let accept: WorkProposalResponseEntry = decode_entry(&accept_record, "WorkProposalResponseEntry")?;
    if accept.hearth_hash != entry.hearth_hash
        || accept.proposal_action_hash != entry.proposal_action_hash
        || accept.responder != entry.endorser
        || accept.response.assignment_id != entry.assignment_id
        || accept.response.decision != ResponseDecision::Accept
    {
        return invalid("Transition endorsement must reference the endorser's Accept response");
    }
    if accept.response.responded_at_micros > entry.endorsed_at.as_micros() {
        return invalid("Transition endorsement cannot precede its referenced Accept response");
    }

    let pure = AssignmentTransitionEndorsement {
        schema_version: ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION,
        transition_id: entry.transition_id.clone(),
        proposal_id: proposal_entry.proposal.id.clone(),
        assignment_id: entry.assignment_id.clone(),
        endorser_id: entry.endorser.to_string(),
        endorsed_at_micros: entry.endorsed_at.as_micros(),
    };
    if let Err(error) = pure.validate_against(&proposal_entry.proposal, &transition) {
        return invalid(format!("Invalid assignment transition endorsement: {error}"));
    }
    if !timestamps_close(entry.endorsed_at.as_micros(), action.timestamp.as_micros()) {
        return invalid("Transition endorsement time is not bound closely enough to action timestamp");
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_assignment_transition(
    entry: &AssignmentTransitionEntry,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if entry.applied_by != action.author {
        return invalid("AssignmentTransition.applied_by must equal action author");
    }
    if entry.endorsement_action_hashes.is_empty()
        || entry.endorsement_action_hashes.len() > MAX_TRANSITION_ENDORSEMENTS
    {
        return invalid("AssignmentTransition endorsement count is outside allowed bounds");
    }
    if let Err(error) = entry.transition.validate() {
        return invalid(format!("Invalid assignment transition: {error}"));
    }
    if entry.transition.hearth_id != entry.hearth_hash.to_string() {
        return invalid("AssignmentTransition hearth binding mismatch");
    }
    if entry.transition.schedule_ref != entry.schedule_root_hash.to_string() {
        return invalid("AssignmentTransition schedule binding mismatch");
    }
    if entry.transition.assignment_ref != entry.assignment_id.0 {
        return invalid("AssignmentTransition assignment binding mismatch");
    }
    if entry.transition.previous_state_ref.0 != entry.previous_state_action_hash.to_string() {
        return invalid("AssignmentTransition previous-state binding mismatch");
    }
    if entry.transition.from_assignee != entry.from_assignee.to_string()
        || entry.transition.to_assignee != entry.to_assignee.to_string()
    {
        return invalid("AssignmentTransition typed assignee binding mismatch");
    }
    if !timestamps_close(entry.transition.applied_at_micros, action.timestamp.as_micros()) {
        return invalid("AssignmentTransition applied_at is not bound closely enough to action timestamp");
    }

    let proposal_record = must_get_valid_record(entry.proposal_action_hash.clone())?;
    let proposal_entry: WorkProposalEntry = decode_entry(&proposal_record, "WorkProposalEntry")?;
    if proposal_entry.hearth_hash != entry.hearth_hash {
        return invalid("AssignmentTransition hearth does not match proposal");
    }
    if entry.transition.proposal_ref != proposal_entry.proposal.id.0 {
        return invalid("AssignmentTransition pure proposal id does not match proposal evidence");
    }
    let binding = proposal_entry
        .bindings
        .iter()
        .find(|binding| binding.assignment_id == entry.assignment_id)
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Transition assignment binding missing".into())))?;
    if binding.schedule_root_hash != entry.schedule_root_hash
        || binding.assignment_state_hash != entry.previous_state_action_hash
        || binding.current_assignee != entry.from_assignee
        || binding.proposed_assignee != entry.to_assignee
    {
        return invalid("AssignmentTransition does not match consented proposal binding");
    }
    if let Err(error) = validate_transition_binding(&proposal_entry.proposal, &entry.transition) {
        return invalid(format!("Transition/proposal binding invalid: {error}"));
    }

    let mut seen = BTreeSet::new();
    let mut endorsement_records = Vec::with_capacity(entry.endorsement_action_hashes.len());
    for endorsement_hash in &entry.endorsement_action_hashes {
        if !seen.insert(endorsement_hash.to_string()) {
            return invalid("AssignmentTransition contains duplicate endorsement references");
        }
        let record = must_get_valid_record(endorsement_hash.clone())?;
        let endorsement: AssignmentTransitionEndorsementEntry =
            decode_entry(&record, "AssignmentTransitionEndorsementEntry")?;
        if endorsement.hearth_hash != entry.hearth_hash
            || endorsement.proposal_action_hash != entry.proposal_action_hash
            || endorsement.assignment_id != entry.assignment_id
            || endorsement.transition_id != entry.transition.id
        {
            return invalid("AssignmentTransition endorsement reference mismatch");
        }
        endorsement_records.push(EndorsementRecord {
            record_id: EndorsementRecordId(endorsement_hash.to_string()),
            endorsement: AssignmentTransitionEndorsement {
                schema_version: ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION,
                transition_id: endorsement.transition_id,
                proposal_id: proposal_entry.proposal.id.clone(),
                assignment_id: endorsement.assignment_id,
                endorser_id: endorsement.endorser.to_string(),
                endorsed_at_micros: endorsement.endorsed_at.as_micros(),
            },
        });
    }

    match evaluate_endorsements(
        &proposal_entry.proposal,
        &entry.transition,
        &endorsement_records,
        entry.transition.applied_at_micros,
    ) {
        Ok(EndorsementEvaluation::Ready(_)) => Ok(ValidateCallbackResult::Valid),
        Ok(_) => invalid("AssignmentTransition does not have all required final endorsements"),
        Err(error) => invalid(format!("AssignmentTransition endorsement evaluation failed: {error}")),
    }
}

fn transition_for_binding(
    proposal: &WorkProposalEntry,
    binding: &WorkProposalScheduleBinding,
    applied_at_micros: i64,
) -> ExternResult<AssignmentTransition> {
    AssignmentTransition::new(
        proposal.hearth_hash.to_string(),
        binding.schedule_root_hash.to_string(),
        AssignmentStateRef(binding.assignment_state_hash.to_string()),
        proposal.proposal.id.0.clone(),
        binding.assignment_id.0.clone(),
        binding.current_assignee.to_string(),
        binding.proposed_assignee.to_string(),
        applied_at_micros,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot construct assignment transition: {error}"
        )))
    })
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
        LinkTypes::ProposalToTransitionEndorsements => {
            let proposal_hash = ActionHash::try_from(base)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid proposal link base".into())))?;
            let endorsement_hash = ActionHash::try_from(target)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid endorsement link target".into())))?;
            let record = must_get_valid_record(endorsement_hash)?;
            let endorsement: AssignmentTransitionEndorsementEntry =
                decode_entry(&record, "AssignmentTransitionEndorsementEntry")?;
            if endorsement.proposal_action_hash != proposal_hash || &endorsement.endorser != author {
                return invalid("ProposalToTransitionEndorsements link mismatch");
            }
        }
        LinkTypes::ScheduleToAssignmentTransitions => {
            let schedule_hash = ActionHash::try_from(base)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid schedule link base".into())))?;
            let transition_hash = ActionHash::try_from(target)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid transition link target".into())))?;
            let record = must_get_valid_record(transition_hash)?;
            let transition: AssignmentTransitionEntry =
                decode_entry(&record, "AssignmentTransitionEntry")?;
            if transition.schedule_root_hash != schedule_hash || &transition.applied_by != author {
                return invalid("ScheduleToAssignmentTransitions link mismatch");
            }
        }
        LinkTypes::ProposalToAssignmentTransitions => {
            let proposal_hash = ActionHash::try_from(base)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid proposal link base".into())))?;
            let transition_hash = ActionHash::try_from(target)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid transition link target".into())))?;
            let record = must_get_valid_record(transition_hash)?;
            let transition: AssignmentTransitionEntry =
                decode_entry(&record, "AssignmentTransitionEntry")?;
            if transition.proposal_action_hash != proposal_hash || &transition.applied_by != author {
                return invalid("ProposalToAssignmentTransitions link mismatch");
            }
        }
        LinkTypes::HearthToAssignmentTransitions => {
            let hearth_hash = ActionHash::try_from(base)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid hearth link base".into())))?;
            let transition_hash = ActionHash::try_from(target)
                .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid transition link target".into())))?;
            let record = must_get_valid_record(transition_hash)?;
            let transition: AssignmentTransitionEntry =
                decode_entry(&record, "AssignmentTransitionEntry")?;
            if transition.hearth_hash != hearth_hash || &transition.applied_by != author {
                return invalid("HearthToAssignmentTransitions link mismatch");
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

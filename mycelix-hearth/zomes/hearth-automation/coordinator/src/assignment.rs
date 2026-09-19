// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Transition-aware assignment finalization for Hearth automation.
//!
//! CareSchedule remains the durable template. Responsibility is derived from an
//! append-only transition chain and final changes require explicit endorsements
//! from every affected member.

use hdk::prelude::*;
use hearth_assignment_endorsement::{
    evaluate_endorsements, AssignmentTransitionEndorsement, EndorsementEvaluation,
    EndorsementRecord, EndorsementRecordId, ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION,
};
use hearth_assignment_transition::{
    derive_effective_assignment, AssignmentBase, AssignmentStateRef, AssignmentTransition,
    TransitionId, TransitionRecord, TransitionRecordId,
};
use hearth_automation_integrity::{
    AssignmentTransitionEndorsementEntry, AssignmentTransitionEntry, EntryTypes, LinkTypes,
    WorkProposalEntry, WorkProposalResponseEntry, WorkProposalScheduleBinding,
};
use hearth_care_integrity::CareSchedule;
use hearth_coordinator_common::{decode_zome_response, get_latest_record, require_membership};
use hearth_kinship_integrity::HearthMembership;
use hearth_types::{CareScheduleStatus, MembershipStatus};
use hearth_work_proposal::{
    build_work_proposal, evaluate_proposal, AssignmentProposalId, AssignmentStateSnapshot,
    ProposalEvaluation, ProposalId, ResponseDecision, ResponseRecord, ResponseRecordId,
};
use mycelix_bridge_common::civic_requirement_basic;
use std::collections::{BTreeMap, BTreeSet};

use crate::{CreateWorkProposalInput, WorkProposalEvaluationView, WorkProposalRecord};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EndorseAssignmentTransitionInput {
    pub proposal_action_hash: ActionHash,
    pub assignment_id: AssignmentProposalId,
    pub accept_response_action_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApplyAssignmentTransitionInput {
    pub proposal_action_hash: ActionHash,
    pub assignment_id: AssignmentProposalId,
    pub endorsement_action_hashes: Vec<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct GetTransitionEndorsementsInput {
    pub proposal_action_hash: ActionHash,
    pub assignment_id: AssignmentProposalId,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AssignmentTransitionEndorsementRecord {
    pub action_hash: ActionHash,
    pub entry: AssignmentTransitionEndorsementEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AssignmentTransitionRecordView {
    pub action_hash: ActionHash,
    pub entry: AssignmentTransitionEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EffectiveAssignmentView {
    pub hearth_hash: ActionHash,
    pub schedule_root_hash: ActionHash,
    pub latest_template_state_hash: ActionHash,
    pub assignment_state_hash: ActionHash,
    pub assignee: AgentPubKey,
    pub transition_count: u32,
    pub duplicate_evidence_count: u32,
    pub orphan_transition_count: u32,
}

pub(crate) struct EffectiveAssignmentContext {
    pub view: EffectiveAssignmentView,
    pub latest_schedule: CareSchedule,
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

fn get_proposal(action_hash: &ActionHash) -> ExternResult<WorkProposalEntry> {
    let record = get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Work proposal not found".into()))
    })?;
    entry_from_record(&record, "WorkProposalEntry")
}

fn transition_views_for_schedule(
    schedule_root_hash: &ActionHash,
) -> ExternResult<Vec<AssignmentTransitionRecordView>> {
    let links = get_links(
        LinkQuery::try_new(
            schedule_root_hash.clone(),
            LinkTypes::ScheduleToAssignmentTransitions,
        )?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid assignment transition link target".into()
            ))
        })?;
        if !seen.insert(action_hash.to_string()) {
            continue;
        }
        if let Some(record) = get(action_hash.clone(), GetOptions::default())? {
            let entry: AssignmentTransitionEntry =
                entry_from_record(&record, "AssignmentTransitionEntry")?;
            if entry.schedule_root_hash == *schedule_root_hash {
                out.push(AssignmentTransitionRecordView { action_hash, entry });
            }
        }
    }
    out.sort_by(|left, right| left.action_hash.to_string().cmp(&right.action_hash.to_string()));
    Ok(out)
}

fn action_descends_from_root(state_hash: &ActionHash, root_hash: &ActionHash) -> ExternResult<bool> {
    let mut current = state_hash.clone();
    for _ in 0..64 {
        if &current == root_hash {
            return Ok(true);
        }
        let record = get(current.clone(), GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Care assignment base state not found".into()
            ))
        })?;
        match record.action() {
            Action::Update(update) => current = update.original_action_address.clone(),
            _ => return Ok(false),
        }
    }
    Ok(false)
}

/// Resolve the current responsibility overlay and fail closed on forks,
/// conflicting legacy reassignment, broken chain continuity, or ambiguous roots.
pub(crate) fn resolve_effective_assignment(
    schedule_root_hash: &ActionHash,
) -> ExternResult<EffectiveAssignmentContext> {
    let latest_record = get_latest_record(schedule_root_hash.clone())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Care schedule not found".into()))
    })?;
    let latest_schedule: CareSchedule = entry_from_record(&latest_record, "CareSchedule")?;
    let latest_template_state_hash = latest_record.action_address().clone();
    let transitions = transition_views_for_schedule(schedule_root_hash)?;

    if transitions.is_empty() {
        return Ok(EffectiveAssignmentContext {
            view: EffectiveAssignmentView {
                hearth_hash: latest_schedule.hearth_hash.clone(),
                schedule_root_hash: schedule_root_hash.clone(),
                latest_template_state_hash: latest_template_state_hash.clone(),
                assignment_state_hash: latest_template_state_hash,
                assignee: latest_schedule.assigned_to.clone(),
                transition_count: 0,
                duplicate_evidence_count: 0,
                orphan_transition_count: 0,
            },
            latest_schedule,
        });
    }

    let produced: BTreeSet<String> = transitions
        .iter()
        .map(|item| item.action_hash.to_string())
        .collect();
    let mut candidate_strings = BTreeSet::new();
    let mut candidate_actions = Vec::new();
    for item in &transitions {
        let previous = item.entry.previous_state_action_hash.clone();
        let key = previous.to_string();
        if !produced.contains(&key) && candidate_strings.insert(key) {
            candidate_actions.push(previous);
        }
    }

    // Orphan evidence may point to an unavailable/non-Care state. Only a
    // candidate that resolves to a CareSchedule descended from the schedule
    // root can seed the authoritative transition chain.
    let mut valid_bases = Vec::new();
    for candidate in candidate_actions {
        let Some(record) = get(candidate.clone(), GetOptions::default())? else {
            continue;
        };
        let decoded: Result<Option<CareSchedule>, SerializedBytesError> =
            record.entry().to_app_option();
        let Ok(Some(schedule)) = decoded else {
            continue;
        };
        if schedule.hearth_hash == latest_schedule.hearth_hash
            && action_descends_from_root(&candidate, schedule_root_hash)?
        {
            valid_bases.push((candidate, schedule));
        }
    }
    if valid_bases.len() != 1 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Assignment transition evidence has {} valid Care chain roots; expected exactly one",
            valid_bases.len()
        ))));
    }
    let (base_state_hash, base_schedule) = valid_bases.remove(0);

    // Once an append-only overlay exists, a separate mutation of the legacy
    // assigned_to field is competing authority and must be resolved explicitly.
    if latest_schedule.assigned_to != base_schedule.assigned_to {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Legacy CareSchedule assignee changed after assignment-transition overlay began"
                .into()
        )));
    }

    let base = AssignmentBase {
        hearth_id: latest_schedule.hearth_hash.to_string(),
        schedule_ref: schedule_root_hash.to_string(),
        state_ref: AssignmentStateRef(base_state_hash.to_string()),
        assignee: base_schedule.assigned_to.to_string(),
    };
    let pure_records = transitions
        .iter()
        .map(|item| TransitionRecord {
            state_ref: AssignmentStateRef(item.action_hash.to_string()),
            record_id: TransitionRecordId(item.action_hash.to_string()),
            transition: item.entry.transition.clone(),
        })
        .collect::<Vec<_>>();
    let effective = derive_effective_assignment(&base, &pure_records).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot derive effective Care assignment: {error}"
        )))
    })?;

    let mut state_index: BTreeMap<String, (ActionHash, AgentPubKey)> = BTreeMap::new();
    state_index.insert(
        base_state_hash.to_string(),
        (base_state_hash.clone(), base_schedule.assigned_to.clone()),
    );
    for item in &transitions {
        state_index.insert(
            item.action_hash.to_string(),
            (item.action_hash.clone(), item.entry.to_assignee.clone()),
        );
    }
    let (assignment_state_hash, assignee) = state_index
        .get(&effective.state_ref.0)
        .cloned()
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Derived assignment state is not backed by a known action".into()
            ))
        })?;
    if assignee.to_string() != effective.assignee {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Derived assignment assignee does not match typed transition evidence".into()
        )));
    }

    Ok(EffectiveAssignmentContext {
        view: EffectiveAssignmentView {
            hearth_hash: latest_schedule.hearth_hash.clone(),
            schedule_root_hash: schedule_root_hash.clone(),
            latest_template_state_hash,
            assignment_state_hash,
            assignee,
            transition_count: effective.applied_transition_ids.len().min(u32::MAX as usize) as u32,
            duplicate_evidence_count: effective.duplicate_evidence_count,
            orphan_transition_count: effective.orphan_transition_count,
        },
        latest_schedule,
    })
}

fn response_records(proposal_action_hash: &ActionHash) -> ExternResult<Vec<(ActionHash, WorkProposalResponseEntry)>> {
    let links = get_links(
        LinkQuery::try_new(
            proposal_action_hash.clone(),
            LinkTypes::ProposalToResponses,
        )?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest("Invalid work response link target".into()))
        })?;
        if !seen.insert(action_hash.to_string()) {
            continue;
        }
        if let Some(record) = get(action_hash.clone(), GetOptions::default())? {
            let entry: WorkProposalResponseEntry =
                entry_from_record(&record, "WorkProposalResponseEntry")?;
            if entry.proposal_action_hash == *proposal_action_hash {
                out.push((action_hash, entry));
            }
        }
    }
    out.sort_by(|left, right| left.0.to_string().cmp(&right.0.to_string()));
    Ok(out)
}

fn transition_endorsement_records(
    proposal_action_hash: &ActionHash,
    assignment_id: &AssignmentProposalId,
    transition_id: &TransitionId,
) -> ExternResult<Vec<(ActionHash, AssignmentTransitionEndorsementEntry)>> {
    let links = get_links(
        LinkQuery::try_new(
            proposal_action_hash.clone(),
            LinkTypes::ProposalToTransitionEndorsements,
        )?,
        GetStrategy::default(),
    )?;
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid transition endorsement link target".into()
            ))
        })?;
        if !seen.insert(action_hash.to_string()) {
            continue;
        }
        if let Some(record) = get(action_hash.clone(), GetOptions::default())? {
            let entry: AssignmentTransitionEndorsementEntry =
                entry_from_record(&record, "AssignmentTransitionEndorsementEntry")?;
            if entry.proposal_action_hash == *proposal_action_hash
                && entry.assignment_id == *assignment_id
                && entry.transition_id == *transition_id
            {
                out.push((action_hash, entry));
            }
        }
    }
    out.sort_by(|left, right| left.0.to_string().cmp(&right.0.to_string()));
    Ok(out)
}

fn require_active_members(hearth_hash: &ActionHash, members: &[AgentPubKey]) -> ExternResult<()> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::new("hearth_kinship"),
        FunctionName::new("get_hearth_members"),
        None,
        hearth_hash.clone(),
    )?;
    let records: Vec<Record> = decode_zome_response(response, "get_hearth_members")?;
    let mut active = BTreeSet::new();
    for record in records {
        let membership: HearthMembership = entry_from_record(&record, "HearthMembership")?;
        if membership.hearth_hash == *hearth_hash && membership.status == MembershipStatus::Active {
            active.insert(membership.agent.to_string());
        }
    }
    for member in members {
        if !active.contains(&member.to_string()) {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Assignment transition member {} is not currently active in this hearth",
                member
            ))));
        }
    }
    Ok(())
}

fn binding_for<'a>(
    proposal: &'a WorkProposalEntry,
    assignment_id: &AssignmentProposalId,
) -> ExternResult<&'a WorkProposalScheduleBinding> {
    proposal
        .bindings
        .iter()
        .find(|binding| &binding.assignment_id == assignment_id)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Work proposal assignment binding not found".into()
            ))
        })
}

fn transition_for_binding(
    proposal_action_hash: &ActionHash,
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
            "Cannot construct assignment transition for proposal {proposal_action_hash}: {error}"
        )))
    })
}

fn require_binding_is_current(
    binding: &WorkProposalScheduleBinding,
) -> ExternResult<EffectiveAssignmentContext> {
    let effective = resolve_effective_assignment(&binding.schedule_root_hash)?;
    if effective.view.assignment_state_hash != binding.assignment_state_hash
        || effective.view.assignee != binding.current_assignee
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Work proposal assignment state is stale; re-plan and re-confirm".into()
        )));
    }
    if effective.latest_schedule.status != CareScheduleStatus::Active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an Active CareSchedule may change responsibility".into()
        )));
    }
    Ok(effective)
}

#[hdk_extern]
pub fn create_work_proposal_v2(input: CreateWorkProposalInput) -> ExternResult<WorkProposalRecord> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "create_work_proposal_v2",
    )?;
    require_membership(&input.hearth_hash)?;
    if input.work_plan.hearth_id != input.hearth_hash.to_string() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Work plan hearth_id does not match requested hearth".into()
        )));
    }
    let now = sys_time()?;
    if input.expires_at <= now {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Work proposal expiry must be in the future".into()
        )));
    }

    let mut requested = BTreeMap::new();
    for binding in input.schedule_bindings {
        if requested
            .insert(binding.occurrence_id.clone(), binding)
            .is_some()
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Duplicate schedule binding for planner occurrence".into()
            )));
        }
    }

    let mut snapshots = Vec::with_capacity(input.work_plan.assignments.len());
    let mut resolved: BTreeMap<String, (ActionHash, ActionHash, AgentPubKey, AgentPubKey)> =
        BTreeMap::new();
    for assignment in &input.work_plan.assignments {
        let binding = requested.remove(&assignment.occurrence_id).ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Missing schedule binding for occurrence {}",
                assignment.occurrence_id
            )))
        })?;
        if assignment.source_schedule_ref != binding.schedule_hash.to_string() {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Planner source schedule does not match typed binding for {}",
                assignment.occurrence_id
            ))));
        }
        if assignment.member_id != binding.proposed_assignee.to_string() {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Planner proposed member does not match typed binding for {}",
                assignment.occurrence_id
            ))));
        }
        let effective = resolve_effective_assignment(&binding.schedule_hash)?;
        if effective.latest_schedule.hearth_hash != input.hearth_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care schedule belongs to a different hearth".into()
            )));
        }
        if effective.latest_schedule.status != CareScheduleStatus::Active {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Only Active Care schedules may enter a work proposal".into()
            )));
        }
        snapshots.push(AssignmentStateSnapshot {
            occurrence_id: assignment.occurrence_id.clone(),
            source_schedule_ref: binding.schedule_hash.to_string(),
            assignment_state_ref: effective.view.assignment_state_hash.to_string(),
            current_assignee: Some(effective.view.assignee.to_string()),
        });
        resolved.insert(
            assignment.occurrence_id.clone(),
            (
                binding.schedule_hash,
                effective.view.assignment_state_hash,
                effective.view.assignee,
                binding.proposed_assignee,
            ),
        );
    }
    if !requested.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Schedule bindings contain occurrences not present in the work plan".into()
        )));
    }

    let proposal = build_work_proposal(
        &input.work_plan,
        &snapshots,
        ProposalId(input.proposal_id),
        input.source_plan_ref,
        now.as_micros(),
        input.expires_at.as_micros(),
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot build household work proposal: {error}"
        )))
    })?;

    let mut bindings = Vec::with_capacity(proposal.assignments.len());
    let mut affected_agents = BTreeMap::new();
    for assignment in &proposal.assignments {
        let (schedule_root_hash, assignment_state_hash, current_assignee, proposed_assignee) =
            resolved.get(&assignment.occurrence_id).cloned().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Resolved schedule binding disappeared during proposal construction".into()
                ))
            })?;
        bindings.push(WorkProposalScheduleBinding {
            assignment_id: assignment.id.clone(),
            occurrence_id: assignment.occurrence_id.clone(),
            schedule_root_hash,
            assignment_state_hash,
            current_assignee: current_assignee.clone(),
            proposed_assignee: proposed_assignee.clone(),
        });
        affected_agents.insert(current_assignee.to_string(), current_assignee);
        affected_agents.insert(proposed_assignee.to_string(), proposed_assignee);
    }
    bindings.sort_by(|left, right| left.assignment_id.cmp(&right.assignment_id));

    let creator = agent_info()?.agent_initial_pubkey;
    let entry = WorkProposalEntry {
        hearth_hash: input.hearth_hash.clone(),
        proposal,
        bindings,
        created_by: creator,
    };
    let action_hash = create_entry(&EntryTypes::WorkProposal(entry.clone()))?;
    create_link(
        input.hearth_hash,
        action_hash.clone(),
        LinkTypes::HearthToWorkProposals,
        (),
    )?;
    for agent in affected_agents.into_values() {
        create_link(
            agent,
            action_hash.clone(),
            LinkTypes::AgentToWorkProposals,
            (),
        )?;
    }
    Ok(WorkProposalRecord { action_hash, entry })
}

#[hdk_extern]
pub fn evaluate_work_proposal_v2(
    proposal_action_hash: ActionHash,
) -> ExternResult<WorkProposalEvaluationView> {
    let proposal = get_proposal(&proposal_action_hash)?;
    require_membership(&proposal.hearth_hash)?;
    let response_views = response_records(&proposal_action_hash)?;
    let pure_records = response_views
        .iter()
        .map(|(action_hash, entry)| ResponseRecord {
            record_id: ResponseRecordId(action_hash.to_string()),
            response: entry.response.clone(),
        })
        .collect::<Vec<_>>();
    let now = sys_time()?;
    let evaluation: ProposalEvaluation =
        evaluate_proposal(&proposal.proposal, &pure_records, now.as_micros()).map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot evaluate work proposal: {error}"
            )))
        })?;
    let accepted: BTreeSet<_> = evaluation.accepted_assignment_ids().into_iter().collect();
    let mut stale_assignment_ids = Vec::new();
    let mut accepted_current_assignment_ids = Vec::new();
    for binding in &proposal.bindings {
        match resolve_effective_assignment(&binding.schedule_root_hash) {
            Ok(effective)
                if effective.view.assignment_state_hash == binding.assignment_state_hash
                    && effective.view.assignee == binding.current_assignee =>
            {
                if accepted.contains(&binding.assignment_id) {
                    accepted_current_assignment_ids.push(binding.assignment_id.clone());
                }
            }
            _ => stale_assignment_ids.push(binding.assignment_id.clone()),
        }
    }
    stale_assignment_ids.sort();
    accepted_current_assignment_ids.sort();
    Ok(WorkProposalEvaluationView {
        evaluation,
        stale_assignment_ids,
        accepted_current_assignment_ids,
    })
}

#[hdk_extern]
pub fn get_effective_assignment(
    schedule_root_hash: ActionHash,
) -> ExternResult<EffectiveAssignmentView> {
    let effective = resolve_effective_assignment(&schedule_root_hash)?;
    require_membership(&effective.view.hearth_hash)?;
    Ok(effective.view)
}

#[hdk_extern]
pub fn get_assignment_transitions(
    schedule_root_hash: ActionHash,
) -> ExternResult<Vec<AssignmentTransitionRecordView>> {
    let effective = resolve_effective_assignment(&schedule_root_hash)?;
    require_membership(&effective.view.hearth_hash)?;
    transition_views_for_schedule(&schedule_root_hash)
}

#[hdk_extern]
pub fn endorse_assignment_transition(
    input: EndorseAssignmentTransitionInput,
) -> ExternResult<AssignmentTransitionEndorsementRecord> {
    let proposal = get_proposal(&input.proposal_action_hash)?;
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "endorse_assignment_transition",
    )?;
    require_membership(&proposal.hearth_hash)?;
    let binding = binding_for(&proposal, &input.assignment_id)?;
    let before = require_binding_is_current(binding)?;
    require_active_members(
        &proposal.hearth_hash,
        &[binding.current_assignee.clone(), binding.proposed_assignee.clone()],
    )?;

    let caller = agent_info()?.agent_initial_pubkey;
    if !proposal
        .proposal
        .assignment(&input.assignment_id)
        .is_some_and(|assignment| assignment.required_responders.contains(&caller.to_string()))
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only a required responder may endorse this assignment transition".into()
        )));
    }

    let accept_record = get(input.accept_response_action_hash.clone(), GetOptions::default())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Referenced Accept response not found".into()
            ))
        })?;
    let accept: WorkProposalResponseEntry =
        entry_from_record(&accept_record, "WorkProposalResponseEntry")?;
    if accept.proposal_action_hash != input.proposal_action_hash
        || accept.response.assignment_id != input.assignment_id
        || accept.responder != caller
        || accept.response.decision != ResponseDecision::Accept
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Final endorsement must reference the caller's Accept response".into()
        )));
    }

    let now = sys_time()?;
    let transition = transition_for_binding(
        &input.proposal_action_hash,
        &proposal,
        binding,
        now.as_micros(),
    )?;
    let entry = AssignmentTransitionEndorsementEntry {
        hearth_hash: proposal.hearth_hash,
        proposal_action_hash: input.proposal_action_hash.clone(),
        assignment_id: input.assignment_id,
        transition_id: transition.id,
        accept_response_action_hash: input.accept_response_action_hash,
        endorser: caller,
        endorsed_at: now,
    };
    let action_hash = create_entry(&EntryTypes::AssignmentTransitionEndorsement(entry.clone()))?;
    create_link(
        input.proposal_action_hash,
        action_hash.clone(),
        LinkTypes::ProposalToTransitionEndorsements,
        (),
    )?;

    let after = resolve_effective_assignment(&binding.schedule_root_hash)?;
    if after.view.assignment_state_hash != before.view.assignment_state_hash
        || after.view.assignee != before.view.assignee
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Assignment state changed while endorsement was being recorded; re-confirm required"
                .into()
        )));
    }

    Ok(AssignmentTransitionEndorsementRecord { action_hash, entry })
}

#[hdk_extern]
pub fn apply_assignment_transition(
    input: ApplyAssignmentTransitionInput,
) -> ExternResult<AssignmentTransitionRecordView> {
    let proposal = get_proposal(&input.proposal_action_hash)?;
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "apply_assignment_transition",
    )?;
    require_membership(&proposal.hearth_hash)?;
    let binding = binding_for(&proposal, &input.assignment_id)?;
    require_binding_is_current(binding)?;
    require_active_members(
        &proposal.hearth_hash,
        &[binding.current_assignee.clone(), binding.proposed_assignee.clone()],
    )?;

    let now = sys_time()?;
    let transition = transition_for_binding(
        &input.proposal_action_hash,
        &proposal,
        binding,
        now.as_micros(),
    )?;

    let mut endorsement_hashes = input.endorsement_action_hashes;
    endorsement_hashes.sort_by(|left, right| left.to_string().cmp(&right.to_string()));
    endorsement_hashes.dedup();
    if endorsement_hashes.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Assignment transition requires final endorsements".into()
        )));
    }

    let mut pure_endorsements = Vec::with_capacity(endorsement_hashes.len());
    for hash in &endorsement_hashes {
        let record = get(hash.clone(), GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Assignment transition endorsement not found".into()
            ))
        })?;
        let endorsement: AssignmentTransitionEndorsementEntry =
            entry_from_record(&record, "AssignmentTransitionEndorsementEntry")?;
        if endorsement.proposal_action_hash != input.proposal_action_hash
            || endorsement.assignment_id != input.assignment_id
            || endorsement.transition_id != transition.id
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Assignment transition endorsement does not match this transition".into()
            )));
        }
        pure_endorsements.push(EndorsementRecord {
            record_id: EndorsementRecordId(hash.to_string()),
            endorsement: AssignmentTransitionEndorsement {
                schema_version: ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION,
                transition_id: endorsement.transition_id,
                proposal_id: proposal.proposal.id.clone(),
                assignment_id: endorsement.assignment_id,
                endorser_id: endorsement.endorser.to_string(),
                endorsed_at_micros: endorsement.endorsed_at.as_micros(),
            },
        });
    }
    match evaluate_endorsements(
        &proposal.proposal,
        &transition,
        &pure_endorsements,
        now.as_micros(),
    ) {
        Ok(EndorsementEvaluation::Ready(_)) => {}
        Ok(_) => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Not all required members have finalized this assignment transition".into()
            )))
        }
        Err(error) => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Assignment transition endorsements are invalid: {error}"
            ))))
        }
    }

    // A last read immediately before create_entry narrows the coordinator race.
    // DHT-concurrent commits remain safe because readers reject transition forks.
    require_binding_is_current(binding)?;
    let caller = agent_info()?.agent_initial_pubkey;
    let entry = AssignmentTransitionEntry {
        hearth_hash: proposal.hearth_hash.clone(),
        schedule_root_hash: binding.schedule_root_hash.clone(),
        previous_state_action_hash: binding.assignment_state_hash.clone(),
        proposal_action_hash: input.proposal_action_hash.clone(),
        assignment_id: input.assignment_id,
        from_assignee: binding.current_assignee.clone(),
        to_assignee: binding.proposed_assignee.clone(),
        transition,
        endorsement_action_hashes: endorsement_hashes,
        applied_by: caller,
    };
    let action_hash = create_entry(&EntryTypes::AssignmentTransition(entry.clone()))?;
    create_link(
        binding.schedule_root_hash.clone(),
        action_hash.clone(),
        LinkTypes::ScheduleToAssignmentTransitions,
        (),
    )?;
    create_link(
        input.proposal_action_hash,
        action_hash.clone(),
        LinkTypes::ProposalToAssignmentTransitions,
        (),
    )?;
    create_link(
        proposal.hearth_hash,
        action_hash.clone(),
        LinkTypes::HearthToAssignmentTransitions,
        (),
    )?;
    Ok(AssignmentTransitionRecordView { action_hash, entry })
}

#[hdk_extern]
pub fn get_transition_endorsements(
    input: GetTransitionEndorsementsInput,
) -> ExternResult<Vec<AssignmentTransitionEndorsementRecord>> {
    let proposal = get_proposal(&input.proposal_action_hash)?;
    require_membership(&proposal.hearth_hash)?;
    let binding = binding_for(&proposal, &input.assignment_id)?;
    let transition = transition_for_binding(
        &input.proposal_action_hash,
        &proposal,
        binding,
        proposal.proposal.created_at_micros,
    )?;
    Ok(transition_endorsement_records(
        &input.proposal_action_hash,
        &input.assignment_id,
        &transition.id,
    )?
    .into_iter()
    .map(|(action_hash, entry)| AssignmentTransitionEndorsementRecord { action_hash, entry })
    .collect())
}

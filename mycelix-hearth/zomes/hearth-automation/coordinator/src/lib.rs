// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Coordinator for durable Hearth automation proposals and consent responses.
//!
//! This tranche records/evaluates consent only. It intentionally exposes no
//! function that mutates Care assignments.

use hdk::prelude::*;
use hearth_automation_integrity::{
    EntryTypes, LinkTypes, WorkProposalEntry, WorkProposalResponseEntry,
    WorkProposalScheduleBinding,
};
use hearth_care_integrity::CareSchedule;
use hearth_coordinator_common::{get_latest_record, require_membership};
use hearth_household_planner::HouseholdWorkPlan;
use hearth_types::CareScheduleStatus;
use hearth_work_proposal::{
    build_work_proposal, evaluate_proposal, AssignmentProposalId, AssignmentStateSnapshot,
    HouseholdWorkProposal, ProposalEvaluation, ProposalId, ResponseDecision, ResponseRecord,
    ResponseRecordId, WorkProposalResponse, WORK_PROPOSAL_SCHEMA_VERSION,
};
use mycelix_bridge_common::civic_requirement_basic;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct WorkPlanScheduleBindingInput {
    pub occurrence_id: String,
    pub schedule_hash: ActionHash,
    pub proposed_assignee: AgentPubKey,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateWorkProposalInput {
    pub hearth_hash: ActionHash,
    pub proposal_id: String,
    pub source_plan_ref: String,
    pub work_plan: HouseholdWorkPlan,
    pub schedule_bindings: Vec<WorkPlanScheduleBindingInput>,
    pub expires_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RespondToWorkProposalInput {
    pub proposal_action_hash: ActionHash,
    pub assignment_id: AssignmentProposalId,
    pub decision: ResponseDecision,
    pub note: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct WorkProposalRecord {
    pub action_hash: ActionHash,
    pub entry: WorkProposalEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct WorkProposalResponseRecord {
    pub action_hash: ActionHash,
    pub entry: WorkProposalResponseEntry,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct WorkProposalEvaluationView {
    pub evaluation: ProposalEvaluation,
    /// Consent may be historically Accepted while its bound schedule state has changed.
    pub stale_assignment_ids: Vec<AssignmentProposalId>,
    /// Accepted assignments whose exact assignment-state binding is still current.
    /// A future apply tranche MUST re-check again before mutation.
    pub accepted_current_assignment_ids: Vec<AssignmentProposalId>,
}

#[derive(Clone)]
struct ResolvedBinding {
    occurrence_id: String,
    schedule_root_hash: ActionHash,
    assignment_state_hash: ActionHash,
    current_assignee: AgentPubKey,
    proposed_assignee: AgentPubKey,
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

fn proposal_error(context: &str, error: impl std::fmt::Display) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{context}: {error}")))
}

fn get_work_proposal_entry(action_hash: &ActionHash) -> ExternResult<WorkProposalEntry> {
    let record = get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Work proposal not found".into()))
    })?;
    entry_from_record(&record, "WorkProposalEntry")
}

fn proposal_records_from_links(links: Vec<Link>) -> ExternResult<Vec<WorkProposalRecord>> {
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest("Invalid work proposal link target".into()))
        })?;
        if !seen.insert(action_hash.to_string()) {
            continue;
        }
        if let Some(record) = get(action_hash.clone(), GetOptions::default())? {
            let entry: WorkProposalEntry = entry_from_record(&record, "WorkProposalEntry")?;
            out.push(WorkProposalRecord { action_hash, entry });
        }
    }
    out.sort_by(|left, right| left.action_hash.to_string().cmp(&right.action_hash.to_string()));
    Ok(out)
}

fn response_records(
    proposal_action_hash: &ActionHash,
) -> ExternResult<Vec<WorkProposalResponseRecord>> {
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
                out.push(WorkProposalResponseRecord { action_hash, entry });
            }
        }
    }
    out.sort_by(|left, right| left.action_hash.to_string().cmp(&right.action_hash.to_string()));
    Ok(out)
}

#[hdk_extern]
pub fn create_work_proposal(input: CreateWorkProposalInput) -> ExternResult<WorkProposalRecord> {
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "create_work_proposal",
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
    let mut resolved = BTreeMap::new();
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

        let state_record = get_latest_record(binding.schedule_hash.clone())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest("Care schedule not found".into()))
        })?;
        let schedule: CareSchedule = entry_from_record(&state_record, "CareSchedule")?;
        if schedule.hearth_hash != input.hearth_hash {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Care schedule belongs to a different hearth".into()
            )));
        }
        if schedule.status != CareScheduleStatus::Active {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Only Active Care schedules may enter a work proposal".into()
            )));
        }
        let state_hash = state_record.action_address().clone();
        snapshots.push(AssignmentStateSnapshot {
            occurrence_id: assignment.occurrence_id.clone(),
            source_schedule_ref: binding.schedule_hash.to_string(),
            assignment_state_ref: state_hash.to_string(),
            current_assignee: Some(schedule.assigned_to.to_string()),
        });
        resolved.insert(
            assignment.occurrence_id.clone(),
            ResolvedBinding {
                occurrence_id: assignment.occurrence_id.clone(),
                schedule_root_hash: binding.schedule_hash,
                assignment_state_hash: state_hash,
                current_assignee: schedule.assigned_to,
                proposed_assignee: binding.proposed_assignee,
            },
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
    .map_err(|error| proposal_error("Cannot build household work proposal", error))?;

    let mut bindings = Vec::with_capacity(proposal.assignments.len());
    let mut affected_agents = BTreeMap::new();
    for assignment in &proposal.assignments {
        let state = resolved.get(&assignment.occurrence_id).ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Resolved schedule binding disappeared during proposal construction".into()
            ))
        })?;
        bindings.push(WorkProposalScheduleBinding {
            assignment_id: assignment.id.clone(),
            occurrence_id: state.occurrence_id.clone(),
            schedule_root_hash: state.schedule_root_hash.clone(),
            assignment_state_hash: state.assignment_state_hash.clone(),
            current_assignee: state.current_assignee.clone(),
            proposed_assignee: state.proposed_assignee.clone(),
        });
        affected_agents.insert(
            state.current_assignee.to_string(),
            state.current_assignee.clone(),
        );
        affected_agents.insert(
            state.proposed_assignee.to_string(),
            state.proposed_assignee.clone(),
        );
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
pub fn respond_to_work_proposal(
    input: RespondToWorkProposalInput,
) -> ExternResult<WorkProposalResponseRecord> {
    let proposal = get_work_proposal_entry(&input.proposal_action_hash)?;
    mycelix_zome_helpers::require_civic(
        "hearth_bridge",
        &civic_requirement_basic(),
        "respond_to_work_proposal",
    )?;
    require_membership(&proposal.hearth_hash)?;
    let caller = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;
    let response = WorkProposalResponse {
        schema_version: WORK_PROPOSAL_SCHEMA_VERSION,
        proposal_id: proposal.proposal.id.clone(),
        assignment_id: input.assignment_id,
        responder_id: caller.to_string(),
        decision: input.decision,
        responded_at_micros: now.as_micros(),
        note: input.note,
    };
    response
        .validate_against(&proposal.proposal)
        .map_err(|error| proposal_error("Cannot record work proposal response", error))?;

    let entry = WorkProposalResponseEntry {
        hearth_hash: proposal.hearth_hash,
        proposal_action_hash: input.proposal_action_hash.clone(),
        response,
        responder: caller,
    };
    let action_hash = create_entry(&EntryTypes::WorkProposalResponse(entry.clone()))?;
    create_link(
        input.proposal_action_hash,
        action_hash.clone(),
        LinkTypes::ProposalToResponses,
        (),
    )?;
    Ok(WorkProposalResponseRecord { action_hash, entry })
}

#[hdk_extern]
pub fn get_work_proposal(action_hash: ActionHash) -> ExternResult<WorkProposalRecord> {
    let entry = get_work_proposal_entry(&action_hash)?;
    require_membership(&entry.hearth_hash)?;
    Ok(WorkProposalRecord { action_hash, entry })
}

#[hdk_extern]
pub fn get_hearth_work_proposals(
    hearth_hash: ActionHash,
) -> ExternResult<Vec<WorkProposalRecord>> {
    require_membership(&hearth_hash)?;
    let links = get_links(
        LinkQuery::try_new(hearth_hash.clone(), LinkTypes::HearthToWorkProposals)?,
        GetStrategy::default(),
    )?;
    let mut records = proposal_records_from_links(links)?;
    records.retain(|record| record.entry.hearth_hash == hearth_hash);
    Ok(records)
}

#[hdk_extern]
pub fn get_my_work_proposals(_: ()) -> ExternResult<Vec<WorkProposalRecord>> {
    let caller = agent_info()?.agent_initial_pubkey;
    let caller_id = caller.to_string();
    let links = get_links(
        LinkQuery::try_new(caller, LinkTypes::AgentToWorkProposals)?,
        GetStrategy::default(),
    )?;
    let mut records = proposal_records_from_links(links)?;
    records.retain(|record| {
        record
            .entry
            .proposal
            .assignments
            .iter()
            .any(|assignment| assignment.required_responders.contains(&caller_id))
    });
    Ok(records)
}

#[hdk_extern]
pub fn get_work_proposal_responses(
    proposal_action_hash: ActionHash,
) -> ExternResult<Vec<WorkProposalResponseRecord>> {
    let proposal = get_work_proposal_entry(&proposal_action_hash)?;
    require_membership(&proposal.hearth_hash)?;
    response_records(&proposal_action_hash)
}

#[hdk_extern]
pub fn evaluate_work_proposal(
    proposal_action_hash: ActionHash,
) -> ExternResult<WorkProposalEvaluationView> {
    let proposal = get_work_proposal_entry(&proposal_action_hash)?;
    require_membership(&proposal.hearth_hash)?;
    let responses = response_records(&proposal_action_hash)?;
    let pure_records = responses
        .iter()
        .map(|record| ResponseRecord {
            record_id: ResponseRecordId(record.action_hash.to_string()),
            response: record.entry.response.clone(),
        })
        .collect::<Vec<_>>();
    let now = sys_time()?;
    let evaluation = evaluate_proposal(&proposal.proposal, &pure_records, now.as_micros())
        .map_err(|error| proposal_error("Cannot evaluate work proposal", error))?;

    let accepted: BTreeSet<_> = evaluation.accepted_assignment_ids().into_iter().collect();
    let mut stale_assignment_ids = Vec::new();
    let mut accepted_current_assignment_ids = Vec::new();
    for binding in &proposal.bindings {
        let is_current = get_latest_record(binding.schedule_root_hash.clone())?
            .is_some_and(|record| record.action_address() == &binding.assignment_state_hash);
        if !is_current {
            stale_assignment_ids.push(binding.assignment_id.clone());
        } else if accepted.contains(&binding.assignment_id) {
            accepted_current_assignment_ids.push(binding.assignment_id.clone());
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

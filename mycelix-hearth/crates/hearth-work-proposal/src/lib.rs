// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Consent-bearing household work proposals for Hearth.
//!
//! Planner output is advisory. This crate binds recommendations to an
//! authoritative assignment snapshot and requires explicit responses from the
//! affected members before an assignment can resolve as accepted.

use hearth_household_planner::{HouseholdWorkPlan, WorkAssignment};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const WORK_PROPOSAL_SCHEMA_VERSION: u16 = 1;
const MAX_ID_LEN: usize = 512;
const MAX_ASSIGNMENTS: usize = 256;
const MAX_NOTE_LEN: usize = 2048;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ProposalId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AssignmentProposalId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ResponseRecordId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssignmentStateSnapshot {
    pub occurrence_id: String,
    pub source_schedule_ref: String,
    /// Exact ActionHash/version/state reference used when consent was requested.
    pub assignment_state_ref: String,
    pub current_assignee: Option<String>,
}

impl AssignmentStateSnapshot {
    pub fn validate(&self) -> Result<(), ProposalError> {
        require_id("occurrence_id", &self.occurrence_id)?;
        require_id("source_schedule_ref", &self.source_schedule_ref)?;
        require_id("assignment_state_ref", &self.assignment_state_ref)?;
        if let Some(member) = &self.current_assignee {
            require_id("current_assignee", member)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkAssignmentProposal {
    pub id: AssignmentProposalId,
    pub occurrence_id: String,
    pub source_schedule_ref: String,
    pub assignment_state_ref: String,
    pub title: String,
    pub current_assignee: Option<String>,
    pub proposed_assignee: String,
    pub start_micros: i64,
    pub end_micros: i64,
    pub estimated_minutes: u32,
    pub kept_current_assignee: bool,
    pub load_before_bp: u32,
    pub load_after_bp: u32,
    /// V1 direct-consent set. It must equal the proposed assignee plus the
    /// current assignee when the proposal changes assignment.
    pub required_responders: BTreeSet<String>,
}

impl WorkAssignmentProposal {
    pub fn validate(&self, proposal_id: &ProposalId) -> Result<(), ProposalError> {
        require_id("assignment_proposal_id", &self.id.0)?;
        require_id("occurrence_id", &self.occurrence_id)?;
        require_id("source_schedule_ref", &self.source_schedule_ref)?;
        require_id("assignment_state_ref", &self.assignment_state_ref)?;
        require_id("title", &self.title)?;
        require_id("proposed_assignee", &self.proposed_assignee)?;
        if self.end_micros <= self.start_micros {
            return Err(ProposalError::InvalidWindow(self.occurrence_id.clone()));
        }
        if self.estimated_minutes == 0 {
            return Err(ProposalError::InvalidEstimate(self.occurrence_id.clone()));
        }
        if let Some(current) = &self.current_assignee {
            require_id("current_assignee", current)?;
        }

        let mut expected = BTreeSet::from([self.proposed_assignee.clone()]);
        if let Some(current) = &self.current_assignee {
            if current != &self.proposed_assignee {
                expected.insert(current.clone());
            }
        }
        if self.required_responders != expected {
            return Err(ProposalError::ResponderSetMismatch(self.occurrence_id.clone()));
        }

        let expected_kept = self.current_assignee.as_ref() == Some(&self.proposed_assignee);
        if self.kept_current_assignee != expected_kept {
            return Err(ProposalError::RationaleMismatch(self.occurrence_id.clone()));
        }

        let derived = derive_assignment_proposal_id(
            proposal_id,
            &self.occurrence_id,
            &self.assignment_state_ref,
            &self.proposed_assignee,
            self.start_micros,
            self.end_micros,
        )?;
        if derived != self.id {
            return Err(ProposalError::AssignmentIdentityMismatch(
                self.occurrence_id.clone(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HouseholdWorkProposal {
    pub schema_version: u16,
    pub id: ProposalId,
    pub hearth_id: String,
    pub source_plan_ref: String,
    pub created_at_micros: i64,
    pub expires_at_micros: i64,
    pub assignments: Vec<WorkAssignmentProposal>,
}

impl HouseholdWorkProposal {
    pub fn validate(&self) -> Result<(), ProposalError> {
        if self.schema_version != WORK_PROPOSAL_SCHEMA_VERSION {
            return Err(ProposalError::UnsupportedSchema(self.schema_version));
        }
        require_id("proposal_id", &self.id.0)?;
        require_id("hearth_id", &self.hearth_id)?;
        require_id("source_plan_ref", &self.source_plan_ref)?;
        if self.expires_at_micros <= self.created_at_micros {
            return Err(ProposalError::InvalidProposalWindow);
        }
        if self.assignments.is_empty() || self.assignments.len() > MAX_ASSIGNMENTS {
            return Err(ProposalError::InvalidAssignmentCount(self.assignments.len()));
        }

        let mut assignment_ids = BTreeSet::new();
        let mut occurrence_ids = BTreeSet::new();
        for assignment in &self.assignments {
            assignment.validate(&self.id)?;
            if !assignment_ids.insert(assignment.id.clone()) {
                return Err(ProposalError::DuplicateAssignmentId(assignment.id.0.clone()));
            }
            if !occurrence_ids.insert(assignment.occurrence_id.clone()) {
                return Err(ProposalError::DuplicateOccurrence(
                    assignment.occurrence_id.clone(),
                ));
            }
        }
        Ok(())
    }

    pub fn assignment(&self, id: &AssignmentProposalId) -> Option<&WorkAssignmentProposal> {
        self.assignments.iter().find(|assignment| &assignment.id == id)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResponseDecision {
    Accept,
    Decline,
    RequestChanges,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkProposalResponse {
    pub schema_version: u16,
    pub proposal_id: ProposalId,
    pub assignment_id: AssignmentProposalId,
    pub responder_id: String,
    pub decision: ResponseDecision,
    pub responded_at_micros: i64,
    pub note: Option<String>,
}

impl WorkProposalResponse {
    pub fn validate_against(&self, proposal: &HouseholdWorkProposal) -> Result<(), ProposalError> {
        if self.schema_version != WORK_PROPOSAL_SCHEMA_VERSION {
            return Err(ProposalError::UnsupportedSchema(self.schema_version));
        }
        if self.proposal_id != proposal.id {
            return Err(ProposalError::ForeignProposalResponse);
        }
        require_id("responder_id", &self.responder_id)?;
        let assignment = proposal
            .assignment(&self.assignment_id)
            .ok_or_else(|| ProposalError::UnknownAssignment(self.assignment_id.0.clone()))?;
        if !assignment.required_responders.contains(&self.responder_id) {
            return Err(ProposalError::UnauthorizedResponder {
                assignment_id: self.assignment_id.0.clone(),
                responder_id: self.responder_id.clone(),
            });
        }
        if self.responded_at_micros < proposal.created_at_micros
            || self.responded_at_micros >= proposal.expires_at_micros
        {
            return Err(ProposalError::ResponseOutsideValidityWindow {
                assignment_id: self.assignment_id.0.clone(),
                responder_id: self.responder_id.clone(),
            });
        }
        if let Some(note) = &self.note {
            if note.len() > MAX_NOTE_LEN {
                return Err(ProposalError::NoteTooLong(note.len()));
            }
        }
        if self.decision == ResponseDecision::RequestChanges
            && self.note.as_ref().is_none_or(|note| note.trim().is_empty())
        {
            return Err(ProposalError::ChangeRequestNeedsNote);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResponseRecord {
    pub record_id: ResponseRecordId,
    pub response: WorkProposalResponse,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanonicalResponse {
    pub canonical: ResponseRecord,
    pub duplicate_record_ids: Vec<ResponseRecordId>,
    /// Same responder produced contradictory decisions for one assignment.
    pub semantic_conflict: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AssignmentResolution {
    Pending,
    Accepted,
    Declined,
    NeedsChanges,
    Conflict,
    Expired,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssignmentEvaluation {
    pub assignment_id: AssignmentProposalId,
    pub resolution: AssignmentResolution,
    pub missing_responders: Vec<String>,
    pub duplicate_response_count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProposalEvaluation {
    pub proposal_id: ProposalId,
    pub assignments: Vec<AssignmentEvaluation>,
    pub accepted_count: u32,
    pub declined_count: u32,
    pub needs_changes_count: u32,
    pub conflict_count: u32,
    pub pending_count: u32,
    pub expired_count: u32,
}

impl ProposalEvaluation {
    pub fn fully_accepted(&self) -> bool {
        !self.assignments.is_empty()
            && self
                .assignments
                .iter()
                .all(|item| item.resolution == AssignmentResolution::Accepted)
    }

    pub fn accepted_assignment_ids(&self) -> Vec<AssignmentProposalId> {
        self.assignments
            .iter()
            .filter(|item| item.resolution == AssignmentResolution::Accepted)
            .map(|item| item.assignment_id.clone())
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProposalError {
    EmptyId(&'static str),
    IdTooLong { field: &'static str, len: usize },
    UnsupportedSchema(u16),
    InvalidProposalWindow,
    InvalidWindow(String),
    InvalidEstimate(String),
    InvalidAssignmentCount(usize),
    DuplicateSnapshot(String),
    MissingSnapshot(String),
    UnusedSnapshot(String),
    SnapshotScheduleMismatch(String),
    DuplicateAssignmentId(String),
    DuplicateOccurrence(String),
    ResponderSetMismatch(String),
    RationaleMismatch(String),
    AssignmentIdentityMismatch(String),
    ForeignProposalResponse,
    UnknownAssignment(String),
    UnauthorizedResponder { assignment_id: String, responder_id: String },
    ResponseOutsideValidityWindow { assignment_id: String, responder_id: String },
    ChangeRequestNeedsNote,
    NoteTooLong(usize),
    DuplicateRecordConflict(String),
    EvaluationBeforeCreation,
}

impl std::fmt::Display for ProposalError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for ProposalError {}

pub fn build_work_proposal(
    work_plan: &HouseholdWorkPlan,
    snapshots: &[AssignmentStateSnapshot],
    proposal_id: ProposalId,
    source_plan_ref: String,
    created_at_micros: i64,
    expires_at_micros: i64,
) -> Result<HouseholdWorkProposal, ProposalError> {
    require_id("proposal_id", &proposal_id.0)?;
    require_id("source_plan_ref", &source_plan_ref)?;
    require_id("hearth_id", &work_plan.hearth_id)?;
    if expires_at_micros <= created_at_micros {
        return Err(ProposalError::InvalidProposalWindow);
    }
    if work_plan.assignments.is_empty() {
        return Err(ProposalError::InvalidAssignmentCount(0));
    }

    let mut snapshot_map = BTreeMap::new();
    for snapshot in snapshots {
        snapshot.validate()?;
        if snapshot_map
            .insert(snapshot.occurrence_id.clone(), snapshot.clone())
            .is_some()
        {
            return Err(ProposalError::DuplicateSnapshot(
                snapshot.occurrence_id.clone(),
            ));
        }
    }

    let mut assignments = Vec::with_capacity(work_plan.assignments.len());
    for assignment in &work_plan.assignments {
        let snapshot = snapshot_map
            .remove(&assignment.occurrence_id)
            .ok_or_else(|| ProposalError::MissingSnapshot(assignment.occurrence_id.clone()))?;
        if snapshot.source_schedule_ref != assignment.source_schedule_ref {
            return Err(ProposalError::SnapshotScheduleMismatch(
                assignment.occurrence_id.clone(),
            ));
        }
        assignments.push(proposal_assignment(&proposal_id, assignment, snapshot)?);
    }
    if let Some((unused, _)) = snapshot_map.into_iter().next() {
        return Err(ProposalError::UnusedSnapshot(unused));
    }
    assignments.sort_by(|left, right| left.id.cmp(&right.id));

    let proposal = HouseholdWorkProposal {
        schema_version: WORK_PROPOSAL_SCHEMA_VERSION,
        id: proposal_id,
        hearth_id: work_plan.hearth_id.clone(),
        source_plan_ref,
        created_at_micros,
        expires_at_micros,
        assignments,
    };
    proposal.validate()?;
    Ok(proposal)
}

fn proposal_assignment(
    proposal_id: &ProposalId,
    assignment: &WorkAssignment,
    snapshot: AssignmentStateSnapshot,
) -> Result<WorkAssignmentProposal, ProposalError> {
    require_id("occurrence_id", &assignment.occurrence_id)?;
    require_id("source_schedule_ref", &assignment.source_schedule_ref)?;
    require_id("title", &assignment.title)?;
    require_id("proposed_assignee", &assignment.member_id)?;
    if assignment.end_micros <= assignment.start_micros {
        return Err(ProposalError::InvalidWindow(assignment.occurrence_id.clone()));
    }
    if assignment.rationale.explicit_estimated_minutes == 0 {
        return Err(ProposalError::InvalidEstimate(assignment.occurrence_id.clone()));
    }

    let mut required_responders = BTreeSet::from([assignment.member_id.clone()]);
    if let Some(current) = &snapshot.current_assignee {
        if current != &assignment.member_id {
            required_responders.insert(current.clone());
        }
    }

    let id = derive_assignment_proposal_id(
        proposal_id,
        &assignment.occurrence_id,
        &snapshot.assignment_state_ref,
        &assignment.member_id,
        assignment.start_micros,
        assignment.end_micros,
    )?;

    Ok(WorkAssignmentProposal {
        id,
        occurrence_id: assignment.occurrence_id.clone(),
        source_schedule_ref: assignment.source_schedule_ref.clone(),
        assignment_state_ref: snapshot.assignment_state_ref,
        title: assignment.title.clone(),
        current_assignee: snapshot.current_assignee.clone(),
        proposed_assignee: assignment.member_id.clone(),
        start_micros: assignment.start_micros,
        end_micros: assignment.end_micros,
        estimated_minutes: assignment.rationale.explicit_estimated_minutes,
        kept_current_assignee: snapshot.current_assignee.as_ref() == Some(&assignment.member_id),
        load_before_bp: assignment.rationale.load_before_bp,
        load_after_bp: assignment.rationale.load_after_bp,
        required_responders,
    })
}

pub fn canonicalize_responses(
    proposal: &HouseholdWorkProposal,
    records: &[ResponseRecord],
) -> Result<BTreeMap<(AssignmentProposalId, String), CanonicalResponse>, ProposalError> {
    proposal.validate()?;
    let mut seen_records = BTreeSet::new();
    let mut grouped: BTreeMap<(AssignmentProposalId, String), Vec<ResponseRecord>> = BTreeMap::new();

    for record in records {
        require_id("response_record_id", &record.record_id.0)?;
        record.response.validate_against(proposal)?;
        if !seen_records.insert(record.record_id.clone()) {
            continue;
        }
        grouped
            .entry((
                record.response.assignment_id.clone(),
                record.response.responder_id.clone(),
            ))
            .or_default()
            .push(record.clone());
    }

    let mut out = BTreeMap::new();
    for (key, mut group) in grouped {
        group.sort_by(|left, right| left.record_id.cmp(&right.record_id));
        let canonical = group[0].clone();
        let semantic_conflict = group
            .iter()
            .skip(1)
            .any(|other| other.response.decision != canonical.response.decision);
        let duplicate_record_ids = group
            .iter()
            .skip(1)
            .map(|record| record.record_id.clone())
            .collect();
        out.insert(
            key,
            CanonicalResponse {
                canonical,
                duplicate_record_ids,
                semantic_conflict,
            },
        );
    }
    Ok(out)
}

pub fn evaluate_proposal(
    proposal: &HouseholdWorkProposal,
    records: &[ResponseRecord],
    now_micros: i64,
) -> Result<ProposalEvaluation, ProposalError> {
    proposal.validate()?;
    if now_micros < proposal.created_at_micros {
        return Err(ProposalError::EvaluationBeforeCreation);
    }
    let canonical = canonicalize_responses(proposal, records)?;
    let mut evaluations = Vec::with_capacity(proposal.assignments.len());

    for assignment in &proposal.assignments {
        let mut missing = Vec::new();
        let mut has_conflict = false;
        let mut has_decline = false;
        let mut has_change_request = false;
        let mut duplicate_response_count = 0_u32;

        for responder in &assignment.required_responders {
            let key = (assignment.id.clone(), responder.clone());
            let Some(response) = canonical.get(&key) else {
                missing.push(responder.clone());
                continue;
            };
            duplicate_response_count = duplicate_response_count
                .saturating_add(response.duplicate_record_ids.len() as u32);
            if response.semantic_conflict {
                has_conflict = true;
                continue;
            }
            match response.canonical.response.decision {
                ResponseDecision::Accept => {}
                ResponseDecision::Decline => has_decline = true,
                ResponseDecision::RequestChanges => has_change_request = true,
            }
        }

        missing.sort();
        let resolution = if has_conflict {
            AssignmentResolution::Conflict
        } else if has_decline {
            AssignmentResolution::Declined
        } else if has_change_request {
            AssignmentResolution::NeedsChanges
        } else if missing.is_empty() {
            AssignmentResolution::Accepted
        } else if now_micros >= proposal.expires_at_micros {
            AssignmentResolution::Expired
        } else {
            AssignmentResolution::Pending
        };

        evaluations.push(AssignmentEvaluation {
            assignment_id: assignment.id.clone(),
            resolution,
            missing_responders: missing,
            duplicate_response_count,
        });
    }

    evaluations.sort_by(|left, right| left.assignment_id.cmp(&right.assignment_id));
    let mut result = ProposalEvaluation {
        proposal_id: proposal.id.clone(),
        assignments: evaluations,
        accepted_count: 0,
        declined_count: 0,
        needs_changes_count: 0,
        conflict_count: 0,
        pending_count: 0,
        expired_count: 0,
    };
    for item in &result.assignments {
        match item.resolution {
            AssignmentResolution::Accepted => {
                result.accepted_count = result.accepted_count.saturating_add(1)
            }
            AssignmentResolution::Declined => {
                result.declined_count = result.declined_count.saturating_add(1)
            }
            AssignmentResolution::NeedsChanges => {
                result.needs_changes_count = result.needs_changes_count.saturating_add(1)
            }
            AssignmentResolution::Conflict => {
                result.conflict_count = result.conflict_count.saturating_add(1)
            }
            AssignmentResolution::Pending => {
                result.pending_count = result.pending_count.saturating_add(1)
            }
            AssignmentResolution::Expired => {
                result.expired_count = result.expired_count.saturating_add(1)
            }
        }
    }
    Ok(result)
}

pub fn derive_assignment_proposal_id(
    proposal_id: &ProposalId,
    occurrence_id: &str,
    assignment_state_ref: &str,
    proposed_assignee: &str,
    start_micros: i64,
    end_micros: i64,
) -> Result<AssignmentProposalId, ProposalError> {
    require_id("proposal_id", &proposal_id.0)?;
    require_id("occurrence_id", occurrence_id)?;
    require_id("assignment_state_ref", assignment_state_ref)?;
    require_id("proposed_assignee", proposed_assignee)?;
    if end_micros <= start_micros {
        return Err(ProposalError::InvalidWindow(occurrence_id.into()));
    }
    Ok(AssignmentProposalId(format!(
        "work-item-v1|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}",
        proposal_id.0.len(),
        proposal_id.0,
        occurrence_id.len(),
        occurrence_id,
        assignment_state_ref.len(),
        assignment_state_ref,
        proposed_assignee.len(),
        proposed_assignee,
        start_micros,
        end_micros,
        WORK_PROPOSAL_SCHEMA_VERSION
    )))
}

fn require_id(field: &'static str, value: &str) -> Result<(), ProposalError> {
    if value.trim().is_empty() {
        return Err(ProposalError::EmptyId(field));
    }
    if value.len() > MAX_ID_LEN {
        return Err(ProposalError::IdTooLong {
            field,
            len: value.len(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_household_planner::{
        AssignmentRationale, FairnessSummary, TimeWindow, WorkAssignment,
    };

    fn assignment(occurrence: &str, member: &str) -> WorkAssignment {
        WorkAssignment {
            occurrence_id: occurrence.into(),
            source_schedule_ref: format!("schedule:{occurrence}"),
            title: format!("Task {occurrence}"),
            member_id: member.into(),
            start_micros: 100,
            end_micros: 200,
            rationale: AssignmentRationale {
                kept_current_assignee: false,
                load_before_bp: 1000,
                load_after_bp: 2000,
                explicit_estimated_minutes: 30,
                history_component_minutes: 0,
            },
        }
    }

    fn plan(assignments: Vec<WorkAssignment>) -> HouseholdWorkPlan {
        HouseholdWorkPlan {
            hearth_id: "hearth:1".into(),
            horizon: TimeWindow {
                start_micros: 0,
                end_micros: 1000,
            },
            assignments,
            issues: vec![],
            fairness: FairnessSummary {
                member_loads: vec![],
                new_assignment_spread_minutes: 0,
                unknown_effort_occurrences: 0,
            },
        }
    }

    fn snapshot(occurrence: &str, current: Option<&str>) -> AssignmentStateSnapshot {
        AssignmentStateSnapshot {
            occurrence_id: occurrence.into(),
            source_schedule_ref: format!("schedule:{occurrence}"),
            assignment_state_ref: format!("state:{occurrence}:v3"),
            current_assignee: current.map(str::to_string),
        }
    }

    fn proposal(current: Option<&str>, proposed: &str) -> HouseholdWorkProposal {
        build_work_proposal(
            &plan(vec![assignment("x", proposed)]),
            &[snapshot("x", current)],
            ProposalId("proposal:1".into()),
            "plan:1".into(),
            10,
            100,
        )
        .unwrap()
    }

    fn response(
        proposal: &HouseholdWorkProposal,
        responder: &str,
        decision: ResponseDecision,
        record: &str,
    ) -> ResponseRecord {
        let assignment = &proposal.assignments[0];
        ResponseRecord {
            record_id: ResponseRecordId(record.into()),
            response: WorkProposalResponse {
                schema_version: WORK_PROPOSAL_SCHEMA_VERSION,
                proposal_id: proposal.id.clone(),
                assignment_id: assignment.id.clone(),
                responder_id: responder.into(),
                decision,
                responded_at_micros: 20,
                note: (decision == ResponseDecision::RequestChanges)
                    .then(|| "Please move this later".into()),
            },
        }
    }

    #[test]
    fn unassigned_work_requires_proposed_assignee() {
        let proposal = proposal(None, "alice");
        assert_eq!(
            proposal.assignments[0].required_responders,
            BTreeSet::from(["alice".to_string()])
        );
    }

    #[test]
    fn reassignment_requires_current_and_proposed_members() {
        let proposal = proposal(Some("alice"), "bob");
        assert_eq!(
            proposal.assignments[0].required_responders,
            BTreeSet::from(["alice".to_string(), "bob".to_string()])
        );
    }

    #[test]
    fn kept_assignment_requires_one_response() {
        let proposal = proposal(Some("alice"), "alice");
        assert_eq!(proposal.assignments[0].required_responders.len(), 1);
        assert!(proposal.assignments[0].kept_current_assignee);
    }

    #[test]
    fn silence_is_pending_then_expired_never_accepted() {
        let proposal = proposal(None, "alice");
        let pending = evaluate_proposal(&proposal, &[], 50).unwrap();
        assert_eq!(pending.assignments[0].resolution, AssignmentResolution::Pending);
        let expired = evaluate_proposal(&proposal, &[], 100).unwrap();
        assert_eq!(expired.assignments[0].resolution, AssignmentResolution::Expired);
    }

    #[test]
    fn all_required_acceptance_resolves_assignment() {
        let proposal = proposal(Some("alice"), "bob");
        let records = vec![
            response(&proposal, "alice", ResponseDecision::Accept, "r1"),
            response(&proposal, "bob", ResponseDecision::Accept, "r2"),
        ];
        let result = evaluate_proposal(&proposal, &records, 50).unwrap();
        assert_eq!(result.assignments[0].resolution, AssignmentResolution::Accepted);
        assert!(result.fully_accepted());
    }

    #[test]
    fn one_decline_blocks_acceptance() {
        let proposal = proposal(Some("alice"), "bob");
        let records = vec![
            response(&proposal, "alice", ResponseDecision::Accept, "r1"),
            response(&proposal, "bob", ResponseDecision::Decline, "r2"),
        ];
        let result = evaluate_proposal(&proposal, &records, 50).unwrap();
        assert_eq!(result.assignments[0].resolution, AssignmentResolution::Declined);
    }

    #[test]
    fn change_request_is_distinct_from_decline() {
        let proposal = proposal(None, "alice");
        let records = vec![response(
            &proposal,
            "alice",
            ResponseDecision::RequestChanges,
            "r1",
        )];
        let result = evaluate_proposal(&proposal, &records, 50).unwrap();
        assert_eq!(
            result.assignments[0].resolution,
            AssignmentResolution::NeedsChanges
        );
    }

    #[test]
    fn agreeing_duplicate_responses_do_not_change_semantics() {
        let proposal = proposal(None, "alice");
        let records = vec![
            response(&proposal, "alice", ResponseDecision::Accept, "r:z"),
            response(&proposal, "alice", ResponseDecision::Accept, "r:a"),
        ];
        let result = evaluate_proposal(&proposal, &records, 50).unwrap();
        assert_eq!(result.assignments[0].resolution, AssignmentResolution::Accepted);
        assert_eq!(result.assignments[0].duplicate_response_count, 1);
    }

    #[test]
    fn contradictory_responses_fail_closed_as_conflict() {
        let proposal = proposal(None, "alice");
        let records = vec![
            response(&proposal, "alice", ResponseDecision::Accept, "r:a"),
            response(&proposal, "alice", ResponseDecision::Decline, "r:b"),
        ];
        let result = evaluate_proposal(&proposal, &records, 50).unwrap();
        assert_eq!(result.assignments[0].resolution, AssignmentResolution::Conflict);
    }

    #[test]
    fn unrelated_member_cannot_respond() {
        let proposal = proposal(None, "alice");
        let records = vec![response(&proposal, "mallory", ResponseDecision::Accept, "r1")];
        let err = evaluate_proposal(&proposal, &records, 50).unwrap_err();
        assert!(matches!(err, ProposalError::UnauthorizedResponder { .. }));
    }

    #[test]
    fn snapshot_must_match_planner_source_schedule() {
        let mut bad = snapshot("x", None);
        bad.source_schedule_ref = "schedule:other".into();
        let err = build_work_proposal(
            &plan(vec![assignment("x", "alice")]),
            &[bad],
            ProposalId("proposal:1".into()),
            "plan:1".into(),
            10,
            100,
        )
        .unwrap_err();
        assert!(matches!(err, ProposalError::SnapshotScheduleMismatch(_)));
    }

    #[test]
    fn assignment_identity_binds_state_reference() {
        let a = proposal(None, "alice");
        let changed = build_work_proposal(
            &plan(vec![assignment("x", "alice")]),
            &[AssignmentStateSnapshot {
                assignment_state_ref: "state:x:v4".into(),
                ..snapshot("x", None)
            }],
            ProposalId("proposal:1".into()),
            "plan:1".into(),
            10,
            100,
        )
        .unwrap();
        assert_ne!(a.assignments[0].id, changed.assignments[0].id);
    }

    #[test]
    fn response_at_expiry_is_not_valid_consent() {
        let proposal = proposal(None, "alice");
        let mut record = response(&proposal, "alice", ResponseDecision::Accept, "r1");
        record.response.responded_at_micros = proposal.expires_at_micros;
        let err = evaluate_proposal(&proposal, &[record], 100).unwrap_err();
        assert!(matches!(
            err,
            ProposalError::ResponseOutsideValidityWindow { .. }
        ));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Final transition endorsement semantics for Hearth.
//!
//! Proposal responses are negotiation evidence. Assignment transitions require
//! an explicit endorsement of the exact state-bound transition intent by every
//! responder required by the proposal assignment.

use hearth_assignment_transition::{AssignmentTransition, TransitionId};
use hearth_work_proposal::{
    AssignmentProposalId, HouseholdWorkProposal, ProposalId, WorkAssignmentProposal,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION: u16 = 1;
const MAX_ID_LEN: usize = 512;
const MAX_ENDORSEMENTS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct EndorsementRecordId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssignmentTransitionEndorsement {
    pub schema_version: u16,
    pub transition_id: TransitionId,
    pub proposal_id: ProposalId,
    pub assignment_id: AssignmentProposalId,
    pub endorser_id: String,
    pub endorsed_at_micros: i64,
}

impl AssignmentTransitionEndorsement {
    pub fn validate_against(
        &self,
        proposal: &HouseholdWorkProposal,
        transition: &AssignmentTransition,
    ) -> Result<(), EndorsementError> {
        if self.schema_version != ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION {
            return Err(EndorsementError::UnsupportedSchema(self.schema_version));
        }
        require_id("endorser_id", &self.endorser_id)?;
        validate_transition_binding(proposal, transition)?;
        if self.transition_id != transition.id {
            return Err(EndorsementError::ForeignTransition);
        }
        if self.proposal_id != proposal.id {
            return Err(EndorsementError::ForeignProposal);
        }
        let assignment = proposal
            .assignment(&self.assignment_id)
            .ok_or_else(|| EndorsementError::UnknownAssignment(self.assignment_id.0.clone()))?;
        if assignment.id.0 != transition.assignment_ref {
            return Err(EndorsementError::AssignmentBindingMismatch);
        }
        if !assignment.required_responders.contains(&self.endorser_id) {
            return Err(EndorsementError::UnauthorizedEndorser {
                assignment_id: assignment.id.0.clone(),
                endorser_id: self.endorser_id.clone(),
            });
        }
        if self.endorsed_at_micros < proposal.created_at_micros
            || self.endorsed_at_micros >= proposal.expires_at_micros
        {
            return Err(EndorsementError::EndorsementOutsideValidityWindow {
                endorser_id: self.endorser_id.clone(),
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EndorsementRecord {
    pub record_id: EndorsementRecordId,
    pub endorsement: AssignmentTransitionEndorsement,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransitionAuthorization {
    pub transition_id: TransitionId,
    pub proposal_id: ProposalId,
    pub assignment_id: AssignmentProposalId,
    pub endorsers: Vec<String>,
    pub canonical_record_ids: Vec<EndorsementRecordId>,
    pub agreeing_duplicate_count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum EndorsementEvaluation {
    Pending {
        transition_id: TransitionId,
        missing_endorsers: Vec<String>,
    },
    Ready(TransitionAuthorization),
    Expired {
        transition_id: TransitionId,
        missing_endorsers: Vec<String>,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EndorsementError {
    EmptyId(&'static str),
    IdTooLong(&'static str, usize),
    UnsupportedSchema(u16),
    ForeignTransition,
    ForeignProposal,
    UnknownAssignment(String),
    AssignmentBindingMismatch,
    TransitionBindingMismatch(&'static str),
    UnauthorizedEndorser {
        assignment_id: String,
        endorser_id: String,
    },
    EndorsementOutsideValidityWindow {
        endorser_id: String,
    },
    DuplicateRecordConflict(String),
    ContradictoryEndorsement(String),
    ApplyTimeBeforeEndorsement,
    ApplyTimeOutsideProposalWindow,
    TooManyEndorsements(usize),
}

impl std::fmt::Display for EndorsementError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyId(name) => write!(f, "{name} must not be empty"),
            Self::IdTooLong(name, len) => write!(f, "{name} is too long: {len}"),
            Self::UnsupportedSchema(v) => write!(f, "unsupported endorsement schema {v}"),
            Self::ForeignTransition => write!(f, "endorsement refers to another transition"),
            Self::ForeignProposal => write!(f, "endorsement refers to another proposal"),
            Self::UnknownAssignment(id) => write!(f, "unknown proposal assignment {id}"),
            Self::AssignmentBindingMismatch => {
                write!(f, "transition assignment reference does not match proposal assignment")
            }
            Self::TransitionBindingMismatch(field) => {
                write!(f, "transition does not match proposal binding: {field}")
            }
            Self::UnauthorizedEndorser {
                assignment_id,
                endorser_id,
            } => write!(
                f,
                "{endorser_id} is not a required responder for assignment {assignment_id}"
            ),
            Self::EndorsementOutsideValidityWindow { endorser_id } => write!(
                f,
                "endorsement by {endorser_id} is outside proposal validity"
            ),
            Self::DuplicateRecordConflict(id) => {
                write!(f, "record id {id} names conflicting endorsement content")
            }
            Self::ContradictoryEndorsement(member) => write!(
                f,
                "endorser {member} produced different endorsements for one transition"
            ),
            Self::ApplyTimeBeforeEndorsement => {
                write!(f, "transition apply time precedes a required endorsement")
            }
            Self::ApplyTimeOutsideProposalWindow => {
                write!(f, "transition apply time is outside proposal validity")
            }
            Self::TooManyEndorsements(n) => write!(f, "too many endorsement records: {n}"),
        }
    }
}

impl std::error::Error for EndorsementError {}

/// Verify that a proposed transition is exactly the assignment state the
/// household proposal asked affected members to consider.
pub fn validate_transition_binding<'a>(
    proposal: &'a HouseholdWorkProposal,
    transition: &AssignmentTransition,
) -> Result<&'a WorkAssignmentProposal, EndorsementError> {
    proposal
        .validate()
        .map_err(|_| EndorsementError::TransitionBindingMismatch("proposal invalid"))?;
    transition
        .validate()
        .map_err(|_| EndorsementError::TransitionBindingMismatch("transition invalid"))?;

    if transition.hearth_id != proposal.hearth_id {
        return Err(EndorsementError::TransitionBindingMismatch("hearth"));
    }
    if transition.proposal_ref != proposal.id.0 {
        return Err(EndorsementError::TransitionBindingMismatch("proposal id"));
    }
    let assignment_id = AssignmentProposalId(transition.assignment_ref.clone());
    let assignment = proposal
        .assignment(&assignment_id)
        .ok_or_else(|| EndorsementError::UnknownAssignment(transition.assignment_ref.clone()))?;
    if transition.schedule_ref != assignment.source_schedule_ref {
        return Err(EndorsementError::TransitionBindingMismatch("schedule"));
    }
    if transition.previous_state_ref.0 != assignment.assignment_state_ref {
        return Err(EndorsementError::TransitionBindingMismatch(
            "previous assignment state",
        ));
    }
    let current = assignment.current_assignee.as_deref().ok_or(
        EndorsementError::TransitionBindingMismatch("current assignee missing"),
    )?;
    if transition.from_assignee != current {
        return Err(EndorsementError::TransitionBindingMismatch("from assignee"));
    }
    if transition.to_assignee != assignment.proposed_assignee {
        return Err(EndorsementError::TransitionBindingMismatch("to assignee"));
    }
    Ok(assignment)
}

/// Evaluate final transition endorsements.
///
/// Endorsements are positive, transition-specific commitments. Ordinary
/// proposal responses are intentionally not accepted as substitutes here.
pub fn evaluate_endorsements(
    proposal: &HouseholdWorkProposal,
    transition: &AssignmentTransition,
    records: &[EndorsementRecord],
    apply_at_micros: i64,
) -> Result<EndorsementEvaluation, EndorsementError> {
    if records.len() > MAX_ENDORSEMENTS {
        return Err(EndorsementError::TooManyEndorsements(records.len()));
    }
    let assignment = validate_transition_binding(proposal, transition)?;
    if apply_at_micros < proposal.created_at_micros
        || apply_at_micros >= proposal.expires_at_micros
    {
        return Err(EndorsementError::ApplyTimeOutsideProposalWindow);
    }

    let mut seen_records: BTreeMap<EndorsementRecordId, AssignmentTransitionEndorsement> =
        BTreeMap::new();
    let mut by_endorser: BTreeMap<String, Vec<(EndorsementRecordId, AssignmentTransitionEndorsement)>> =
        BTreeMap::new();

    for record in records {
        require_id("endorsement_record_id", &record.record_id.0)?;
        record.endorsement.validate_against(proposal, transition)?;
        if record.endorsement.endorsed_at_micros > apply_at_micros {
            return Err(EndorsementError::ApplyTimeBeforeEndorsement);
        }
        match seen_records.get(&record.record_id) {
            Some(existing) if existing != &record.endorsement => {
                return Err(EndorsementError::DuplicateRecordConflict(
                    record.record_id.0.clone(),
                ));
            }
            Some(_) => continue,
            None => {
                seen_records.insert(record.record_id.clone(), record.endorsement.clone());
            }
        }
        by_endorser
            .entry(record.endorsement.endorser_id.clone())
            .or_default()
            .push((record.record_id.clone(), record.endorsement.clone()));
    }

    let mut canonical_ids = Vec::new();
    let mut agreeing_duplicate_count = 0_u32;
    let mut present = BTreeSet::new();
    for (endorser, mut group) in by_endorser {
        group.sort_by(|a, b| a.0.cmp(&b.0));
        let canonical = &group[0];
        if group.iter().skip(1).any(|(_, other)| {
            other.transition_id != canonical.1.transition_id
                || other.proposal_id != canonical.1.proposal_id
                || other.assignment_id != canonical.1.assignment_id
                || other.endorser_id != canonical.1.endorser_id
        }) {
            return Err(EndorsementError::ContradictoryEndorsement(endorser));
        }
        canonical_ids.push(canonical.0.clone());
        agreeing_duplicate_count = agreeing_duplicate_count
            .saturating_add(group.len().saturating_sub(1) as u32);
        present.insert(endorser);
    }
    canonical_ids.sort();

    let mut missing: Vec<String> = assignment
        .required_responders
        .difference(&present)
        .cloned()
        .collect();
    missing.sort();

    if missing.is_empty() {
        let mut endorsers: Vec<String> = present.into_iter().collect();
        endorsers.sort();
        return Ok(EndorsementEvaluation::Ready(TransitionAuthorization {
            transition_id: transition.id.clone(),
            proposal_id: proposal.id.clone(),
            assignment_id: assignment.id.clone(),
            endorsers,
            canonical_record_ids: canonical_ids,
            agreeing_duplicate_count,
        }));
    }

    Ok(EndorsementEvaluation::Pending {
        transition_id: transition.id.clone(),
        missing_endorsers: missing,
    })
}

fn require_id(name: &'static str, value: &str) -> Result<(), EndorsementError> {
    if value.trim().is_empty() {
        return Err(EndorsementError::EmptyId(name));
    }
    if value.len() > MAX_ID_LEN {
        return Err(EndorsementError::IdTooLong(name, value.len()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_assignment_transition::{AssignmentStateRef, AssignmentTransition};
    use hearth_work_proposal::{
        AssignmentProposalId, HouseholdWorkProposal, ProposalId, WorkAssignmentProposal,
        WORK_PROPOSAL_SCHEMA_VERSION,
    };
    use std::collections::BTreeSet;

    fn proposal() -> HouseholdWorkProposal {
        HouseholdWorkProposal {
            schema_version: WORK_PROPOSAL_SCHEMA_VERSION,
            id: ProposalId("proposal-1".into()),
            hearth_id: "hearth".into(),
            source_plan_ref: "plan".into(),
            created_at_micros: 10,
            expires_at_micros: 100,
            assignments: vec![WorkAssignmentProposal {
                id: AssignmentProposalId(
                    "work-item-v1|10|proposal-1|3|occ|5|state|3|bob|20|40|1".into(),
                ),
                occurrence_id: "occ".into(),
                source_schedule_ref: "schedule".into(),
                assignment_state_ref: "state".into(),
                title: "Task".into(),
                current_assignee: Some("alice".into()),
                proposed_assignee: "bob".into(),
                start_micros: 20,
                end_micros: 40,
                estimated_minutes: 20,
                kept_current_assignee: false,
                load_before_bp: 100,
                load_after_bp: 200,
                required_responders: BTreeSet::from(["alice".into(), "bob".into()]),
            }],
        }
    }

    fn transition(p: &HouseholdWorkProposal) -> AssignmentTransition {
        let assignment = &p.assignments[0];
        AssignmentTransition::new(
            p.hearth_id.clone(),
            assignment.source_schedule_ref.clone(),
            AssignmentStateRef(assignment.assignment_state_ref.clone()),
            p.id.0.clone(),
            assignment.id.0.clone(),
            assignment.current_assignee.clone().unwrap(),
            assignment.proposed_assignee.clone(),
            50,
        )
        .unwrap()
    }

    fn endorsement(
        p: &HouseholdWorkProposal,
        t: &AssignmentTransition,
        member: &str,
        at: i64,
        record: &str,
    ) -> EndorsementRecord {
        EndorsementRecord {
            record_id: EndorsementRecordId(record.into()),
            endorsement: AssignmentTransitionEndorsement {
                schema_version: ASSIGNMENT_ENDORSEMENT_SCHEMA_VERSION,
                transition_id: t.id.clone(),
                proposal_id: p.id.clone(),
                assignment_id: p.assignments[0].id.clone(),
                endorser_id: member.into(),
                endorsed_at_micros: at,
            },
        }
    }

    #[test]
    fn both_affected_members_are_required_for_reassignment() {
        let p = proposal();
        let t = transition(&p);
        let one = endorsement(&p, &t, "alice", 30, "e1");
        let result = evaluate_endorsements(&p, &t, &[one], 50).unwrap();
        assert!(matches!(
            result,
            EndorsementEvaluation::Pending { missing_endorsers, .. }
                if missing_endorsers == vec!["bob".to_string()]
        ));
    }

    #[test]
    fn exact_required_set_authorizes_transition() {
        let p = proposal();
        let t = transition(&p);
        let result = evaluate_endorsements(
            &p,
            &t,
            &[
                endorsement(&p, &t, "alice", 30, "e1"),
                endorsement(&p, &t, "bob", 31, "e2"),
            ],
            50,
        )
        .unwrap();
        assert!(matches!(result, EndorsementEvaluation::Ready(_)));
    }

    #[test]
    fn outsider_cannot_endorse_transition() {
        let p = proposal();
        let t = transition(&p);
        let err = endorsement(&p, &t, "mallory", 30, "e1")
            .endorsement
            .validate_against(&p, &t)
            .unwrap_err();
        assert!(matches!(err, EndorsementError::UnauthorizedEndorser { .. }));
    }

    #[test]
    fn stale_transition_state_does_not_match_proposal() {
        let p = proposal();
        let assignment = &p.assignments[0];
        let t = AssignmentTransition::new(
            p.hearth_id.clone(),
            assignment.source_schedule_ref.clone(),
            AssignmentStateRef("newer-state".into()),
            p.id.0.clone(),
            assignment.id.0.clone(),
            "alice".into(),
            "bob".into(),
            50,
        )
        .unwrap();
        assert!(matches!(
            validate_transition_binding(&p, &t),
            Err(EndorsementError::TransitionBindingMismatch(
                "previous assignment state"
            ))
        ));
    }

    #[test]
    fn endorsement_after_expiry_is_invalid() {
        let p = proposal();
        let t = transition(&p);
        let err = endorsement(&p, &t, "alice", 100, "e1")
            .endorsement
            .validate_against(&p, &t)
            .unwrap_err();
        assert!(matches!(
            err,
            EndorsementError::EndorsementOutsideValidityWindow { .. }
        ));
    }

    #[test]
    fn apply_cannot_precede_endorsement() {
        let p = proposal();
        let t = transition(&p);
        let err = evaluate_endorsements(
            &p,
            &t,
            &[
                endorsement(&p, &t, "alice", 60, "e1"),
                endorsement(&p, &t, "bob", 61, "e2"),
            ],
            50,
        )
        .unwrap_err();
        assert_eq!(err, EndorsementError::ApplyTimeBeforeEndorsement);
    }

    #[test]
    fn duplicate_endorsement_records_do_not_inflate_authority() {
        let p = proposal();
        let t = transition(&p);
        let alice = endorsement(&p, &t, "alice", 30, "e1");
        let result = evaluate_endorsements(
            &p,
            &t,
            &[
                alice.clone(),
                alice,
                endorsement(&p, &t, "bob", 31, "e2"),
            ],
            50,
        )
        .unwrap();
        match result {
            EndorsementEvaluation::Ready(auth) => {
                assert_eq!(auth.endorsers, vec!["alice", "bob"]);
                assert_eq!(auth.agreeing_duplicate_count, 0);
            }
            other => panic!("unexpected evaluation: {other:?}"),
        }
    }
}

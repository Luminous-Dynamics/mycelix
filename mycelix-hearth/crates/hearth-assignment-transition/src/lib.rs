// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Append-only household responsibility transitions.
//!
//! Care schedules remain durable templates. Responsibility changes form an
//! explicit transition chain whose effective state is derived deterministically.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const ASSIGNMENT_TRANSITION_SCHEMA_VERSION: u16 = 1;
const MAX_ID_LEN: usize = 512;
const MAX_TRANSITIONS: usize = 4096;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AssignmentStateRef(pub String);

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TransitionId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TransitionRecordId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssignmentBase {
    pub hearth_id: String,
    pub schedule_ref: String,
    pub state_ref: AssignmentStateRef,
    pub assignee: String,
}

impl AssignmentBase {
    pub fn validate(&self) -> Result<(), TransitionError> {
        require_id("hearth_id", &self.hearth_id)?;
        require_id("schedule_ref", &self.schedule_ref)?;
        require_id("state_ref", &self.state_ref.0)?;
        require_id("assignee", &self.assignee)?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssignmentTransition {
    pub schema_version: u16,
    pub id: TransitionId,
    pub hearth_id: String,
    pub schedule_ref: String,
    pub previous_state_ref: AssignmentStateRef,
    pub proposal_ref: String,
    pub assignment_ref: String,
    pub from_assignee: String,
    pub to_assignee: String,
    pub applied_at_micros: i64,
}

impl AssignmentTransition {
    pub fn new(
        hearth_id: String,
        schedule_ref: String,
        previous_state_ref: AssignmentStateRef,
        proposal_ref: String,
        assignment_ref: String,
        from_assignee: String,
        to_assignee: String,
        applied_at_micros: i64,
    ) -> Result<Self, TransitionError> {
        let id = derive_transition_id(
            &hearth_id,
            &schedule_ref,
            &previous_state_ref,
            &proposal_ref,
            &assignment_ref,
            &from_assignee,
            &to_assignee,
        )?;
        let value = Self {
            schema_version: ASSIGNMENT_TRANSITION_SCHEMA_VERSION,
            id,
            hearth_id,
            schedule_ref,
            previous_state_ref,
            proposal_ref,
            assignment_ref,
            from_assignee,
            to_assignee,
            applied_at_micros,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), TransitionError> {
        if self.schema_version != ASSIGNMENT_TRANSITION_SCHEMA_VERSION {
            return Err(TransitionError::UnsupportedSchema(self.schema_version));
        }
        require_id("transition_id", &self.id.0)?;
        require_id("hearth_id", &self.hearth_id)?;
        require_id("schedule_ref", &self.schedule_ref)?;
        require_id("previous_state_ref", &self.previous_state_ref.0)?;
        require_id("proposal_ref", &self.proposal_ref)?;
        require_id("assignment_ref", &self.assignment_ref)?;
        require_id("from_assignee", &self.from_assignee)?;
        require_id("to_assignee", &self.to_assignee)?;
        if self.from_assignee == self.to_assignee {
            return Err(TransitionError::NoOpTransition);
        }
        let expected = derive_transition_id(
            &self.hearth_id,
            &self.schedule_ref,
            &self.previous_state_ref,
            &self.proposal_ref,
            &self.assignment_ref,
            &self.from_assignee,
            &self.to_assignee,
        )?;
        if expected != self.id {
            return Err(TransitionError::TransitionIdentityMismatch);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransitionRecord {
    /// Stable identity of the resulting assignment state. A DHT adapter should
    /// prefer content identity so identical concurrent writes share this value.
    pub state_ref: AssignmentStateRef,
    /// Stable record identity used only for deterministic duplicate tie-breaks.
    pub record_id: TransitionRecordId,
    pub transition: AssignmentTransition,
}

impl TransitionRecord {
    pub fn validate(&self) -> Result<(), TransitionError> {
        require_id("transition_state_ref", &self.state_ref.0)?;
        require_id("transition_record_id", &self.record_id.0)?;
        self.transition.validate()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanonicalTransition {
    pub canonical: TransitionRecord,
    pub duplicate_record_ids: Vec<TransitionRecordId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectiveAssignment {
    pub hearth_id: String,
    pub schedule_ref: String,
    pub assignee: String,
    pub state_ref: AssignmentStateRef,
    pub applied_transition_ids: Vec<TransitionId>,
    pub duplicate_evidence_count: u32,
    pub orphan_transition_count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransitionError {
    EmptyId(&'static str),
    IdTooLong(&'static str, usize),
    UnsupportedSchema(u16),
    NoOpTransition,
    TransitionIdentityMismatch,
    DuplicateStateIdentityConflict(String),
    TransitionFork {
        previous_state_ref: String,
        transition_ids: Vec<String>,
    },
    ScopeMismatch(String),
    FromAssigneeMismatch {
        expected: String,
        actual: String,
    },
    NonMonotonicTime,
    Cycle(String),
    TooManyTransitions(usize),
}

impl std::fmt::Display for TransitionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyId(name) => write!(f, "{name} must not be empty"),
            Self::IdTooLong(name, len) => write!(f, "{name} is too long: {len}"),
            Self::UnsupportedSchema(v) => write!(f, "unsupported transition schema {v}"),
            Self::NoOpTransition => write!(f, "assignment transition must change assignee"),
            Self::TransitionIdentityMismatch => write!(f, "transition id does not match content"),
            Self::DuplicateStateIdentityConflict(state) => {
                write!(f, "different transition evidence claims state identity {state}")
            }
            Self::TransitionFork {
                previous_state_ref,
                transition_ids,
            } => write!(
                f,
                "assignment transition fork from {previous_state_ref}: {transition_ids:?}"
            ),
            Self::ScopeMismatch(id) => write!(f, "transition {id} belongs to another hearth/schedule"),
            Self::FromAssigneeMismatch { expected, actual } => write!(
                f,
                "transition from_assignee mismatch: expected {expected}, got {actual}"
            ),
            Self::NonMonotonicTime => write!(f, "transition chain timestamps are not monotonic"),
            Self::Cycle(state) => write!(f, "assignment transition cycle at state {state}"),
            Self::TooManyTransitions(n) => write!(f, "too many assignment transitions: {n}"),
        }
    }
}

impl std::error::Error for TransitionError {}

pub fn derive_transition_id(
    hearth_id: &str,
    schedule_ref: &str,
    previous_state_ref: &AssignmentStateRef,
    proposal_ref: &str,
    assignment_ref: &str,
    from_assignee: &str,
    to_assignee: &str,
) -> Result<TransitionId, TransitionError> {
    for (name, value) in [
        ("hearth_id", hearth_id),
        ("schedule_ref", schedule_ref),
        ("previous_state_ref", previous_state_ref.0.as_str()),
        ("proposal_ref", proposal_ref),
        ("assignment_ref", assignment_ref),
        ("from_assignee", from_assignee),
        ("to_assignee", to_assignee),
    ] {
        require_id(name, value)?;
    }
    Ok(TransitionId(format!(
        "assign-v1|{}|{}|{}|{}|{}|{}|{}",
        frame(hearth_id),
        frame(schedule_ref),
        frame(&previous_state_ref.0),
        frame(proposal_ref),
        frame(assignment_ref),
        frame(from_assignee),
        frame(to_assignee),
    )))
}

/// Canonicalize identical evidence and reject semantic ambiguity.
///
/// Two different semantic transitions from the same previous assignment state
/// are a fork. The caller must not pick one using DHT arrival order.
pub fn canonicalize_transitions(
    records: &[TransitionRecord],
) -> Result<BTreeMap<TransitionId, CanonicalTransition>, TransitionError> {
    if records.len() > MAX_TRANSITIONS {
        return Err(TransitionError::TooManyTransitions(records.len()));
    }

    let mut seen_record_ids = BTreeSet::new();
    let mut by_id: BTreeMap<TransitionId, Vec<TransitionRecord>> = BTreeMap::new();
    let mut state_semantics: BTreeMap<AssignmentStateRef, AssignmentTransition> = BTreeMap::new();

    for record in records {
        record.validate()?;
        if !seen_record_ids.insert(record.record_id.clone()) {
            continue;
        }
        if let Some(existing) = state_semantics.get(&record.state_ref) {
            if existing != &record.transition {
                return Err(TransitionError::DuplicateStateIdentityConflict(
                    record.state_ref.0.clone(),
                ));
            }
        } else {
            state_semantics.insert(record.state_ref.clone(), record.transition.clone());
        }
        by_id
            .entry(record.transition.id.clone())
            .or_default()
            .push(record.clone());
    }

    let mut canonical = BTreeMap::new();
    for (id, mut group) in by_id {
        group.sort_by(|a, b| a.record_id.cmp(&b.record_id));
        let first = group[0].clone();
        if group.iter().skip(1).any(|other| other.transition != first.transition) {
            return Err(TransitionError::TransitionIdentityMismatch);
        }
        canonical.insert(
            id,
            CanonicalTransition {
                canonical: first,
                duplicate_record_ids: group
                    .iter()
                    .skip(1)
                    .map(|record| record.record_id.clone())
                    .collect(),
            },
        );
    }

    let mut by_previous: BTreeMap<AssignmentStateRef, Vec<TransitionId>> = BTreeMap::new();
    for (id, item) in &canonical {
        by_previous
            .entry(item.canonical.transition.previous_state_ref.clone())
            .or_default()
            .push(id.clone());
    }
    for (previous, ids) in by_previous {
        if ids.len() > 1 {
            return Err(TransitionError::TransitionFork {
                previous_state_ref: previous.0,
                transition_ids: ids.into_iter().map(|id| id.0).collect(),
            });
        }
    }

    Ok(canonical)
}

/// Derive the current assignee from a trusted base plus append-only transitions.
/// Unreachable records are reported as orphan evidence rather than silently
/// influencing the effective assignment.
pub fn derive_effective_assignment(
    base: &AssignmentBase,
    records: &[TransitionRecord],
) -> Result<EffectiveAssignment, TransitionError> {
    base.validate()?;
    let canonical = canonicalize_transitions(records)?;

    let mut by_previous: BTreeMap<AssignmentStateRef, &CanonicalTransition> = BTreeMap::new();
    for item in canonical.values() {
        let transition = &item.canonical.transition;
        if transition.hearth_id != base.hearth_id || transition.schedule_ref != base.schedule_ref {
            return Err(TransitionError::ScopeMismatch(transition.id.0.clone()));
        }
        by_previous.insert(transition.previous_state_ref.clone(), item);
    }

    let mut current_state = base.state_ref.clone();
    let mut current_assignee = base.assignee.clone();
    let mut applied_transition_ids = Vec::new();
    let mut reached_ids = BTreeSet::new();
    let mut visited_states = BTreeSet::new();
    visited_states.insert(current_state.clone());
    let mut duplicate_evidence_count = 0_u32;
    let mut previous_time: Option<i64> = None;

    while let Some(item) = by_previous.get(&current_state) {
        let transition = &item.canonical.transition;
        if transition.from_assignee != current_assignee {
            return Err(TransitionError::FromAssigneeMismatch {
                expected: current_assignee,
                actual: transition.from_assignee.clone(),
            });
        }
        if let Some(previous) = previous_time {
            if transition.applied_at_micros <= previous {
                return Err(TransitionError::NonMonotonicTime);
            }
        }
        if !visited_states.insert(item.canonical.state_ref.clone()) {
            return Err(TransitionError::Cycle(item.canonical.state_ref.0.clone()));
        }

        current_assignee = transition.to_assignee.clone();
        current_state = item.canonical.state_ref.clone();
        previous_time = Some(transition.applied_at_micros);
        reached_ids.insert(transition.id.clone());
        applied_transition_ids.push(transition.id.clone());
        duplicate_evidence_count = duplicate_evidence_count
            .saturating_add(item.duplicate_record_ids.len() as u32);
    }

    let orphan_transition_count = canonical
        .keys()
        .filter(|id| !reached_ids.contains(*id))
        .count()
        .min(u32::MAX as usize) as u32;

    Ok(EffectiveAssignment {
        hearth_id: base.hearth_id.clone(),
        schedule_ref: base.schedule_ref.clone(),
        assignee: current_assignee,
        state_ref: current_state,
        applied_transition_ids,
        duplicate_evidence_count,
        orphan_transition_count,
    })
}

fn frame(value: &str) -> String {
    format!("{}:{}", value.len(), value)
}

fn require_id(name: &'static str, value: &str) -> Result<(), TransitionError> {
    if value.trim().is_empty() {
        return Err(TransitionError::EmptyId(name));
    }
    if value.len() > MAX_ID_LEN {
        return Err(TransitionError::IdTooLong(name, value.len()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> AssignmentBase {
        AssignmentBase {
            hearth_id: "hearth".into(),
            schedule_ref: "schedule".into(),
            state_ref: AssignmentStateRef("care-action-1".into()),
            assignee: "alice".into(),
        }
    }

    fn transition(
        previous: &str,
        from: &str,
        to: &str,
        proposal: &str,
        assignment: &str,
        at: i64,
    ) -> AssignmentTransition {
        AssignmentTransition::new(
            "hearth".into(),
            "schedule".into(),
            AssignmentStateRef(previous.into()),
            proposal.into(),
            assignment.into(),
            from.into(),
            to.into(),
            at,
        )
        .unwrap()
    }

    fn record(
        state: &str,
        record_id: &str,
        transition: AssignmentTransition,
    ) -> TransitionRecord {
        TransitionRecord {
            state_ref: AssignmentStateRef(state.into()),
            record_id: TransitionRecordId(record_id.into()),
            transition,
        }
    }

    #[test]
    fn transition_identity_is_deterministic_and_state_bound() {
        let a = transition("care-action-1", "alice", "bob", "p1", "a1", 10);
        let b = transition("care-action-1", "alice", "bob", "p1", "a1", 20);
        let c = transition("care-action-2", "alice", "bob", "p1", "a1", 10);
        assert_eq!(a.id, b.id);
        assert_ne!(a.id, c.id);
    }

    #[test]
    fn no_op_assignment_change_is_rejected() {
        assert!(matches!(
            AssignmentTransition::new(
                "hearth".into(),
                "schedule".into(),
                AssignmentStateRef("s".into()),
                "p".into(),
                "a".into(),
                "alice".into(),
                "alice".into(),
                10,
            ),
            Err(TransitionError::NoOpTransition)
        ));
    }

    #[test]
    fn identical_duplicate_evidence_does_not_advance_twice() {
        let t = transition("care-action-1", "alice", "bob", "p1", "a1", 10);
        let result = derive_effective_assignment(
            &base(),
            &[
                record("transition-state-1", "record:z", t.clone()),
                record("transition-state-1", "record:a", t),
            ],
        )
        .unwrap();
        assert_eq!(result.assignee, "bob");
        assert_eq!(result.applied_transition_ids.len(), 1);
        assert_eq!(result.duplicate_evidence_count, 1);
    }

    #[test]
    fn competing_transitions_from_one_state_fail_closed() {
        let a = transition("care-action-1", "alice", "bob", "p1", "a1", 10);
        let b = transition("care-action-1", "alice", "carol", "p2", "a2", 11);
        let err = derive_effective_assignment(
            &base(),
            &[
                record("state-bob", "r1", a),
                record("state-carol", "r2", b),
            ],
        )
        .unwrap_err();
        assert!(matches!(err, TransitionError::TransitionFork { .. }));
    }

    #[test]
    fn multi_step_chain_derives_effective_assignee() {
        let a = transition("care-action-1", "alice", "bob", "p1", "a1", 10);
        let b = transition("state-bob", "bob", "carol", "p2", "a2", 20);
        let result = derive_effective_assignment(
            &base(),
            &[
                record("state-bob", "r1", a),
                record("state-carol", "r2", b),
            ],
        )
        .unwrap();
        assert_eq!(result.assignee, "carol");
        assert_eq!(result.state_ref, AssignmentStateRef("state-carol".into()));
        assert_eq!(result.applied_transition_ids.len(), 2);
    }

    #[test]
    fn from_assignee_must_match_current_chain_state() {
        let bad = transition("care-action-1", "mallory", "bob", "p1", "a1", 10);
        let err = derive_effective_assignment(&base(), &[record("state-bob", "r1", bad)])
            .unwrap_err();
        assert!(matches!(err, TransitionError::FromAssigneeMismatch { .. }));
    }

    #[test]
    fn unreachable_transition_is_reported_as_orphan() {
        let reached = transition("care-action-1", "alice", "bob", "p1", "a1", 10);
        let orphan = transition("missing-state", "carol", "dave", "p2", "a2", 20);
        let result = derive_effective_assignment(
            &base(),
            &[
                record("state-bob", "r1", reached),
                record("state-dave", "r2", orphan),
            ],
        )
        .unwrap();
        assert_eq!(result.assignee, "bob");
        assert_eq!(result.orphan_transition_count, 1);
    }

    #[test]
    fn state_identity_cannot_name_different_semantics() {
        let a = transition("care-action-1", "alice", "bob", "p1", "a1", 10);
        let b = transition("other-parent", "carol", "dave", "p2", "a2", 20);
        let err = canonicalize_transitions(&[
            record("same-state", "r1", a),
            record("same-state", "r2", b),
        ])
        .unwrap_err();
        assert!(matches!(err, TransitionError::DuplicateStateIdentityConflict(_)));
    }

    #[test]
    fn chain_time_must_move_forward() {
        let a = transition("care-action-1", "alice", "bob", "p1", "a1", 20);
        let b = transition("state-bob", "bob", "carol", "p2", "a2", 10);
        let err = derive_effective_assignment(
            &base(),
            &[
                record("state-bob", "r1", a),
                record("state-carol", "r2", b),
            ],
        )
        .unwrap_err();
        assert_eq!(err, TransitionError::NonMonotonicTime);
    }
}

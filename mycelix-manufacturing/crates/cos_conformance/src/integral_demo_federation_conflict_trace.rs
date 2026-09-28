//! D6E graph-native conflict projection.
//!
//! Conflicting federation observations remain first-class evidence. The trace
//! receives the two observations, an explicit FRS assessment, and only the
//! relations warranted by the conflict itself: each observation disputes the
//! other, and the assessment responds to both. No winner or causal ordering is
//! inferred from delivery order.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_domain::{ProvenanceClass, SourceKind};
use crate::integral_demo_federation::{FederationObservation, ObservationConflict};
use crate::integral_demo_trace::{
    TraceActor, TraceEvent, TraceFixture, TraceKind, TraceRelation, TraceRelationRef,
    TraceStatus,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConflictTraceDecision { Projected, Replayed, RejectedIdentityMutation }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConflictTraceInput {
    pub conflict: ObservationConflict,
    pub left: FederationObservation,
    pub right: FederationObservation,
    pub assessment_id: &'static str,
    pub assessment_source_ref: &'static str,
    pub generation: u32,
    pub assessed_at: u64,
    pub uncertainty_present: bool,
}

/// Project an already-reconciled conflict into the D5 graph.
///
/// Observation ordering is serialization order only. The semantic relations
/// are explicit dispute edges and assessment-to-observation response edges.
pub fn project_conflict_trace(input: ConflictTraceInput) -> Option<TraceFixture> {
    if input.assessment_id.is_empty()
        || input.assessment_source_ref.is_empty()
        || input.left.observation_id.is_empty()
        || input.right.observation_id.is_empty()
        || input.left.observation_id == input.right.observation_id
        || input.left.work_id != input.right.work_id
        || input.left.quantity == input.right.quantity
        || input.left.observation_id != input.conflict.left_observation_id
        || input.right.observation_id != input.conflict.right_observation_id
        || input.left.work_id != input.conflict.work_id
        || input.generation == 0
        || input.assessed_at < input.left.observed_at
        || input.assessed_at < input.right.observed_at
    { return None; }

    let (left, right) = if input.left.observation_id <= input.right.observation_id {
        (input.left, input.right)
    } else { (input.right, input.left) };

    let left_event = observation_event(&left, 1, input.generation, input.uncertainty_present);
    let right_event = observation_event(&right, 2, input.generation, input.uncertainty_present);
    let assessment = TraceEvent {
        event_id: input.assessment_id,
        sequence: 3,
        kind: TraceKind::FrsAssessment,
        provenance: ProvenanceClass::Assessment,
        actor: TraceActor::System,
        source: SourceKind::Local,
        source_ref: input.assessment_source_ref,
        generation: input.generation,
        uncertainty_present: input.uncertainty_present,
        authority_ref: None,
        reversible: true,
        challengeable: true,
        recommendation_only: false,
        recovery_ref: None,
        appeal_ref: None,
        decision_accepted: None,
        status: TraceStatus::Disputed,
    };

    Some(TraceFixture {
        events: vec![left_event, right_event, assessment],
        relations: vec![
            TraceRelationRef { from_event: left.event_id, to_event: right.event_id, relation: TraceRelation::Disputes },
            TraceRelationRef { from_event: right.event_id, to_event: left.event_id, relation: TraceRelation::Disputes },
            TraceRelationRef { from_event: assessment.event_id, to_event: left.event_id, relation: TraceRelation::RespondsTo },
            TraceRelationRef { from_event: assessment.event_id, to_event: right.event_id, relation: TraceRelation::RespondsTo },
        ],
    })
}

fn observation_event(
    observation: &FederationObservation,
    sequence: u32,
    generation: u32,
    uncertainty_present: bool,
) -> TraceEvent {
    TraceEvent {
        event_id: observation.observation_id,
        sequence,
        kind: TraceKind::Observation,
        provenance: ProvenanceClass::Observation,
        actor: TraceActor::System,
        source: match observation.origin {
            crate::integral_demo_federation::FederationNode::Local => SourceKind::Local,
            crate::integral_demo_federation::FederationNode::Foreign => SourceKind::Foreign,
        },
        source_ref: observation.evidence_ref,
        generation,
        uncertainty_present,
        authority_ref: None,
        reversible: true,
        challengeable: true,
        recommendation_only: false,
        recovery_ref: None,
        appeal_ref: None,
        decision_accepted: None,
        status: TraceStatus::Accepted,
    }
}

pub fn conflict_trace_replay_decision(
    existing: &TraceFixture,
    replay: &TraceFixture,
) -> ConflictTraceDecision {
    if existing == replay { return ConflictTraceDecision::Replayed; }
    let existing_ids: Vec<_> = existing.events.iter().map(|e| e.event_id).collect();
    let replay_ids: Vec<_> = replay.events.iter().map(|e| e.event_id).collect();
    if existing_ids == replay_ids {
        ConflictTraceDecision::RejectedIdentityMutation
    } else { ConflictTraceDecision::Projected }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integral_demo_federation::FederationNode;

    fn input() -> ConflictTraceInput {
        let left = FederationObservation {
            observation_id: "obs-a", work_id: "work-1", origin: FederationNode::Local,
            quantity: 10, evidence_ref: "evidence://local-a", observed_at: 100,
        };
        let right = FederationObservation {
            observation_id: "obs-b", work_id: "work-1", origin: FederationNode::Foreign,
            quantity: 12, evidence_ref: "evidence://foreign-b", observed_at: 101,
        };
        ConflictTraceInput {
            conflict: ObservationConflict {
                work_id: "work-1",
                left_observation_id: "obs-a", left_origin: FederationNode::Local, left_quantity: 10,
                right_observation_id: "obs-b", right_origin: FederationNode::Foreign, right_quantity: 12,
            },
            left, right, assessment_id: "frs-conflict-1",
            assessment_source_ref: "assessment://conflict-1", generation: 7,
            assessed_at: 110, uncertainty_present: true,
        }
    }

    #[test]
    fn conflict_projects_as_explicit_dispute_without_winner() {
        let trace = project_conflict_trace(input()).expect("valid conflict");
        assert_eq!(trace.events.len(), 3);
        assert_eq!(trace.relations.len(), 4);
        assert_eq!(trace.events[0].source, SourceKind::Local);
        assert_eq!(trace.events[1].source, SourceKind::Foreign);
        assert_eq!(trace.events[2].kind, TraceKind::FrsAssessment);
        assert_eq!(trace.events[2].status, TraceStatus::Disputed);
        assert!(trace.events.iter().all(|event| event.authority_ref.is_none()));
        assert_eq!(trace.relations.iter().filter(|r| r.relation == TraceRelation::Disputes).count(), 2);
    }

    #[test]
    fn canonical_projection_is_independent_of_input_order() {
        let mut reversed = input();
        std::mem::swap(&mut reversed.left, &mut reversed.right);
        std::mem::swap(
            &mut reversed.conflict.left_observation_id,
            &mut reversed.conflict.right_observation_id,
        );
        std::mem::swap(&mut reversed.conflict.left_origin, &mut reversed.conflict.right_origin);
        std::mem::swap(&mut reversed.conflict.left_quantity, &mut reversed.conflict.right_quantity);
        let left = project_conflict_trace(input()).expect("left");
        let right = project_conflict_trace(reversed).expect("right");
        assert_eq!(left, right);
    }

    #[test]
    fn exact_replay_is_idempotent() {
        let trace = project_conflict_trace(input()).expect("trace");
        assert_eq!(conflict_trace_replay_decision(&trace, &trace), ConflictTraceDecision::Replayed);
    }

    #[test]
    fn source_mutation_is_not_replay() {
        let trace = project_conflict_trace(input()).expect("trace");
        let mut mutated = trace.clone();
        mutated.events[0].source_ref = "evidence://tampered";
        assert_eq!(conflict_trace_replay_decision(&trace, &mutated), ConflictTraceDecision::RejectedIdentityMutation);
    }

    #[test]
    fn malformed_conflict_fails_closed() {
        let mut malformed = input();
        malformed.right.quantity = malformed.left.quantity;
        assert!(project_conflict_trace(malformed).is_none());
    }

    #[test]
    fn assessment_does_not_gain_authority_from_conflict() {
        let trace = project_conflict_trace(input()).expect("trace");
        let assessment = &trace.events[2];
        assert_eq!(assessment.provenance, ProvenanceClass::Assessment);
        assert!(assessment.authority_ref.is_none());
        assert!(!assessment.recommendation_only);
    }
}

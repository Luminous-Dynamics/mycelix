//! D6E federation-to-D5 evidence trace projection.
//!
//! This adapter makes an accepted federation delivery observable in the D5
//! provenance graph without making transport delivery itself an Observation.
//! Replays are idempotent; rejected deliveries cannot manufacture evidence.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_domain::{ProvenanceClass, SourceKind};
use crate::integral_demo_federation::{
    EvidenceBindingDecision, FederationObservationBinding,
};
use crate::integral_demo_trace::{
    TraceActor, TraceEvent, TraceFixture, TraceKind, TraceRelation, TraceRelationRef,
    TraceStatus,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FederationTraceDecision {
    Projected,
    Replayed,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationObservationTrace {
    pub event: TraceEvent,
    pub binding: FederationObservationBinding,
}

/// Project an accepted/replayed evidence binding into D5 Observation lineage.
///
/// The caller supplies uncertainty explicitly because the federation binding
/// itself does not invent an uncertainty claim. Foreign origin is preserved
/// exactly; no local authority is attached to the observation.
pub fn observation_binding_trace(
    binding: FederationObservationBinding,
    decision: EvidenceBindingDecision,
    sequence: u32,
    uncertainty_present: bool,
) -> Option<FederationObservationTrace> {
    let status = match decision {
        EvidenceBindingDecision::Bound => TraceStatus::Accepted,
        EvidenceBindingDecision::Replayed => TraceStatus::Accepted,
        _ => return None,
    };

    let source = match binding.origin {
        crate::integral_demo_federation::FederationNode::Local => SourceKind::Local,
        crate::integral_demo_federation::FederationNode::Foreign => SourceKind::Foreign,
    };

    Some(FederationObservationTrace {
        event: TraceEvent {
            event_id: binding.observation_id,
            sequence,
            kind: TraceKind::Observation,
            provenance: ProvenanceClass::Observation,
            actor: TraceActor::System,
            source,
            source_ref: binding.source_ref,
            generation: binding.schema_generation,
            uncertainty_present,
            authority_ref: None,
            reversible: true,
            challengeable: true,
            recommendation_only: false,
            recovery_ref: None,
            appeal_ref: None,
            decision_accepted: None,
            status,
        },
        binding,
    })
}

/// Build a canonical observation-only trace from accepted federation bindings.
///
/// Ordering is by observation identity, making reconstruction independent of
/// delivery arrival order. Exact duplicate identities are replayed, while a
/// caller-supplied mutation is rejected before projection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FederationProjectionError {
    DuplicateIdentityMutation,
}

pub fn project_bindings_checked(
    bindings: &[(FederationObservationBinding, EvidenceBindingDecision, bool)],
) -> Result<TraceFixture, FederationProjectionError> {
    let mut projected: Vec<FederationObservationTrace> = bindings
        .iter()
        .filter_map(|(binding, decision, uncertainty)| {
            observation_binding_trace(*binding, *decision, 0, *uncertainty)
        })
        .collect();

    projected.sort_by_key(|item| item.event.event_id);

    let mut events = Vec::new();
    for item in projected {
        if let Some(existing) = events
            .iter()
            .find(|event: &&TraceEvent| event.event_id == item.event.event_id)
        {
            if *existing != item.event {
                return Err(FederationProjectionError::DuplicateIdentityMutation);
            }
            continue;
        }
        events.push(item.event);
    }

    for (sequence, event) in events.iter_mut().enumerate() {
        event.sequence = (sequence + 1) as u32;
    }

    let mut relations = Vec::new();
    for pair in events.windows(2) {
        relations.push(TraceRelationRef {
            from_event: pair[1].event_id,
            to_event: pair[0].event_id,
            relation: TraceRelation::Supports,
        });
    }

    Ok(TraceFixture { events, relations })
}

pub fn project_bindings(
    bindings: &[(FederationObservationBinding, EvidenceBindingDecision, bool)],
) -> TraceFixture {
    project_bindings_checked(bindings).expect("invalid federation evidence projection")
}

/// Re-project a binding against an existing event, distinguishing exact replay
/// from authoritative payload mutation.
pub fn replay_decision(
    existing: Option<&TraceEvent>,
    binding: FederationObservationBinding,
    decision: EvidenceBindingDecision,
    uncertainty_present: bool,
) -> FederationTraceDecision {
    let Some(projected) =
        observation_binding_trace(binding, decision, existing.map_or(1, |e| e.sequence), uncertainty_present)
    else {
        return FederationTraceDecision::Rejected;
    };

    match existing {
        None => FederationTraceDecision::Projected,
        Some(existing) if *existing == projected.event => FederationTraceDecision::Replayed,
        Some(_) => FederationTraceDecision::Rejected,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integral_demo_federation::{
        bind_delivery_to_observation, envelope, observation_binding_for, FederationNode,
        FederationObservation,
    };
    use crate::integral_demo_trace::validate_trace;

    #[test]
    fn foreign_bound_observation_projects_without_origin_laundering() {
        let delivery = envelope();
        let observation = FederationObservation {
            observation_id: "obs-foreign-trace",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            evidence_ref: delivery.source_ref,
            observed_at: delivery.observed_at,
        };
        let binding = observation_binding_for(delivery, observation, 7, 20).expect("binding");
        let decision = bind_delivery_to_observation(delivery, observation, 7, 20, None);
        let projected =
            observation_binding_trace(binding, decision, 1, true).expect("projection");

        assert_eq!(projected.event.source, SourceKind::Foreign);
        assert_eq!(projected.event.source_ref, delivery.source_ref);
        assert_eq!(projected.event.provenance, ProvenanceClass::Observation);
        assert_eq!(projected.event.authority_ref, None);
        assert!(projected.event.uncertainty_present);
        assert_eq!(validate_trace(&TraceFixture {
            events: vec![projected.event],
            relations: vec![],
        }), Ok(()));
    }

    #[test]
    fn rejected_delivery_cannot_materialize_an_observation_trace() {
        let mut delivery = envelope();
        delivery.state = crate::integral_demo_federation::DeliveryState::Partitioned;
        let observation = FederationObservation {
            observation_id: "obs-rejected",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            evidence_ref: delivery.source_ref,
            observed_at: delivery.observed_at,
        };
        let decision = bind_delivery_to_observation(delivery, observation, 7, 20, None);
        assert_eq!(decision, EvidenceBindingDecision::RejectedDelivery);
        assert!(observation_binding_trace(
            crate::integral_demo_federation::FederationObservationBinding {
                logical_delivery_id: delivery.logical_delivery_id,
                observation_id: observation.observation_id,
                source_ref: observation.evidence_ref,
                origin: observation.origin,
                schema_generation: delivery.schema_generation,
                payload_digest: delivery.payload_digest,
            },
            decision,
            1,
            true,
        ).is_none());
    }

    #[test]
    fn exact_replay_is_idempotent_but_binding_mutation_is_rejected() {
        let delivery = envelope();
        let observation = FederationObservation {
            observation_id: "obs-replay-trace",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            evidence_ref: delivery.source_ref,
            observed_at: delivery.observed_at,
        };
        let binding = observation_binding_for(delivery, observation, 7, 20).expect("binding");
        let projected = observation_binding_trace(binding, EvidenceBindingDecision::Bound, 1, true)
            .expect("projection");

        assert_eq!(
            replay_decision(
                Some(&projected.event),
                binding,
                EvidenceBindingDecision::Replayed,
                true
            ),
            FederationTraceDecision::Replayed
        );

        let mut mutated = binding;
        mutated.source_ref = "evidence://mutated";
        assert_eq!(
            replay_decision(
                Some(&projected.event),
                mutated,
                EvidenceBindingDecision::Replayed,
                true
            ),
            FederationTraceDecision::Rejected
        );
    }

    #[test]
    fn canonical_projection_is_order_invariant() {
        let delivery = envelope();
        let a = FederationObservationBinding {
            logical_delivery_id: delivery.logical_delivery_id,
            observation_id: "obs-a",
            source_ref: "evidence://a",
            origin: FederationNode::Local,
            schema_generation: 7,
            payload_digest: "digest-a",
        };
        let b = FederationObservationBinding {
            logical_delivery_id: "delivery-2",
            observation_id: "obs-b",
            source_ref: "evidence://b",
            origin: FederationNode::Foreign,
            schema_generation: 7,
            payload_digest: "digest-b",
        };

        let left = project_bindings_checked(&[(a, EvidenceBindingDecision::Bound, false), (b, EvidenceBindingDecision::Bound, true)]).expect("projection");
        let right = project_bindings_checked(&[(b, EvidenceBindingDecision::Bound, true), (a, EvidenceBindingDecision::Bound, false)]).expect("projection");

        assert_eq!(left, right);
        assert_eq!(left.events[0].event_id, "obs-a");
        assert_eq!(left.events[0].sequence, 1);
        assert_eq!(left.events[1].sequence, 2);
        assert_eq!(left.events[1].event_id, "obs-b");
        assert_eq!(left.events[1].source, SourceKind::Foreign);
    }
}

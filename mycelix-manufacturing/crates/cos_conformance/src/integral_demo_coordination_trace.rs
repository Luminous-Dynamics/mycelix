//! Cross-check A1 coordination artifacts against D5 evidence-bearing trace events.
//!
//! This is an alignment check, not an implicit conversion: D5 graph relations
//! and A1 parent links have different semantics and are never synthesized here.
//! Claim ceiling: ReferenceModelOnly.

use crate::integral_demo_coordination::{CoordinationArtifact, CoordinationKind, CoordinationOrigin, Disposition};
use crate::integral_demo_domain::{ProvenanceClass, SourceKind};
use crate::integral_demo_trace::{validate_trace, TraceError, TraceEvent, TraceFixture, TraceKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlignmentError {
    InvalidTrace(TraceError),
    MissingTraceEvent,
    DuplicateTraceIdentity,
    KindMismatch,
    OriginMismatch,
    SourceMismatch,
    GenerationMismatch,
    SourceReferenceMismatch,
    AuthorityMismatch,
    UncertaintyMismatch,
    DispositionMismatch,
    ProvenanceMismatch,
}

fn expected_trace_kind(kind: CoordinationKind) -> Option<TraceKind> {
    match kind {
        CoordinationKind::Intent => Some(TraceKind::Proposal),
        CoordinationKind::Decision => Some(TraceKind::Decision),
        CoordinationKind::Design => Some(TraceKind::Design),
        CoordinationKind::Authorization => Some(TraceKind::Authorization),
        CoordinationKind::ExecutionIntent => Some(TraceKind::ExecutionIntent),
        CoordinationKind::Observation => Some(TraceKind::Observation),
        CoordinationKind::Assessment => Some(TraceKind::FrsAssessment),
        CoordinationKind::Recommendation => Some(TraceKind::Recommendation),
        CoordinationKind::HumanDisposition => Some(TraceKind::HumanDecision),
        CoordinationKind::Revision => Some(TraceKind::Design),
        CoordinationKind::Appeal => Some(TraceKind::Appeal),
    }
}

fn expected_provenance(kind: CoordinationKind) -> Option<ProvenanceClass> {
    match kind {
        CoordinationKind::Intent => Some(ProvenanceClass::Proposal),
        CoordinationKind::Decision | CoordinationKind::HumanDisposition => Some(ProvenanceClass::Decision),
        CoordinationKind::Design | CoordinationKind::Revision => Some(ProvenanceClass::Design),
        CoordinationKind::Authorization => Some(ProvenanceClass::Authorization),
        CoordinationKind::ExecutionIntent => Some(ProvenanceClass::ExecutionIntent),
        CoordinationKind::Observation => Some(ProvenanceClass::Observation),
        CoordinationKind::Assessment => Some(ProvenanceClass::Assessment),
        CoordinationKind::Recommendation => Some(ProvenanceClass::Recommendation),
        CoordinationKind::Appeal => Some(ProvenanceClass::Appeal),
    }
}

fn align_one(artifact: &CoordinationArtifact, event: &TraceEvent) -> Result<(), AlignmentError> {
    if expected_trace_kind(artifact.kind) != Some(event.kind) { return Err(AlignmentError::KindMismatch); }
    let expected_origin = match artifact.origin {
        CoordinationOrigin::Local => SourceKind::Local,
        CoordinationOrigin::Foreign => SourceKind::Foreign,
    };
    if event.source != expected_origin { return Err(AlignmentError::OriginMismatch); }
    if event.generation != artifact.generation { return Err(AlignmentError::GenerationMismatch); }
    if event.source_ref != artifact.source_ref { return Err(AlignmentError::SourceReferenceMismatch); }
    if event.authority_ref != artifact.authority_ref { return Err(AlignmentError::AuthorityMismatch); }
    if event.uncertainty_present != artifact.uncertainty_present { return Err(AlignmentError::UncertaintyMismatch); }
    if expected_provenance(artifact.kind) != Some(event.provenance) { return Err(AlignmentError::ProvenanceMismatch); }
    if artifact.kind == CoordinationKind::HumanDisposition {
        let expected = match artifact.disposition {
            Some(Disposition::Accepted) => Some(true),
            Some(Disposition::Rejected) => Some(false),
            // D5 currently has a boolean disposition. Deferred is intentionally
            // not coerced into either value.
            Some(Disposition::Deferred) | None => None,
        };
        if event.decision_accepted != expected { return Err(AlignmentError::DispositionMismatch); }
    }
    Ok(())
}

/// Every A1 artifact must have exactly one D5 event with the same identity and
/// matching shared fields. Additional D5 events are allowed, but unpaired A1
/// artifacts fail closed. Parent lineage is not inferred from event sequence.
pub fn validate_coordination_trace_alignment(
    artifacts: &[CoordinationArtifact],
    trace: &TraceFixture,
) -> Result<(), AlignmentError> {
    validate_trace(trace).map_err(AlignmentError::InvalidTrace)?;
    for (index, artifact) in artifacts.iter().enumerate() {
        if artifacts[..index].iter().any(|prior| prior.id == artifact.id) {
            return Err(AlignmentError::DuplicateTraceIdentity);
        }
        let mut matches = trace.events.iter().filter(|event| event.event_id == artifact.id);
        let event = matches.next().ok_or(AlignmentError::MissingTraceEvent)?;
        if matches.next().is_some() { return Err(AlignmentError::DuplicateTraceIdentity); }
        align_one(artifact, event)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integral_demo_federation::{FederationNode, FederationObservation, ObservationConflict};
    use crate::integral_demo_federation_conflict_trace::{project_conflict_trace, ConflictTraceInput};

    fn conflict_trace() -> TraceFixture {
        let left = FederationObservation { observation_id: "obs-a", work_id: "work-1", origin: FederationNode::Local, quantity: 10, evidence_ref: "evidence://local-a", observed_at: 100 };
        let right = FederationObservation { observation_id: "obs-b", work_id: "work-1", origin: FederationNode::Foreign, quantity: 12, evidence_ref: "evidence://foreign-b", observed_at: 101 };
        let conflict = ObservationConflict { work_id: "work-1", left_observation_id: "obs-a", left_origin: FederationNode::Local, left_quantity: 10, right_observation_id: "obs-b", right_origin: FederationNode::Foreign, right_quantity: 12 };
        project_conflict_trace(ConflictTraceInput { conflict, left, right, assessment_id: "frs-conflict-1", assessment_source_ref: "assessment://conflict-1", generation: 7, assessed_at: 110, uncertainty_present: true }).expect("trace")
    }

    fn artifact(id: &'static str, kind: CoordinationKind, origin: CoordinationOrigin, source_ref: &'static str) -> CoordinationArtifact {
        CoordinationArtifact { id, kind, origin, generation: 7, source_ref, parent_ref: None, evidence_ref: Some(source_ref), authority_ref: None, disposition: None, uncertainty_present: true, challengeable: true, reversible: true, recovery_ref: None }
    }

    #[test]
    fn foreign_conflict_evidence_aligns_without_origin_laundering() {
        let trace = conflict_trace();
        assert_eq!(validate_trace(&trace), Ok(()));
        let artifacts = vec![
            artifact("obs-a", CoordinationKind::Observation, CoordinationOrigin::Local, "evidence://local-a"),
            artifact("obs-b", CoordinationKind::Observation, CoordinationOrigin::Foreign, "evidence://foreign-b"),
            artifact("frs-conflict-1", CoordinationKind::Assessment, CoordinationOrigin::Local, "assessment://conflict-1"),
        ];
        assert_eq!(validate_coordination_trace_alignment(&artifacts, &trace), Ok(()));
    }

    #[test]
    fn mismatched_origin_is_rejected() {
        let trace = conflict_trace();
        let artifacts = vec![artifact("obs-b", CoordinationKind::Observation, CoordinationOrigin::Local, "evidence://foreign-b")];
        assert_eq!(validate_coordination_trace_alignment(&artifacts, &trace), Err(AlignmentError::OriginMismatch));
    }

    #[test]
    fn mismatched_source_reference_is_rejected() {
        let trace = conflict_trace();
        let artifacts = vec![artifact("obs-a", CoordinationKind::Observation, CoordinationOrigin::Local, "evidence://changed")];
        assert_eq!(validate_coordination_trace_alignment(&artifacts, &trace), Err(AlignmentError::SourceReferenceMismatch));
    }

    #[test]
    fn missing_trace_counterpart_is_rejected() {
        let trace = conflict_trace();
        let artifacts = vec![artifact("not-in-trace", CoordinationKind::Observation, CoordinationOrigin::Local, "evidence://x")];
        assert_eq!(validate_coordination_trace_alignment(&artifacts, &trace), Err(AlignmentError::MissingTraceEvent));
    }

    #[test]
    fn event_sequence_does_not_create_parent_lineage() {
        // The cross-check deliberately verifies shared fields only. A1 parent
        // references must be validated by validate_loop, not guessed from D5 order.
        let trace = conflict_trace();
        let mut item = artifact("obs-a", CoordinationKind::Observation, CoordinationOrigin::Local, "evidence://local-a");
        item.parent_ref = Some("made-up-parent");
        assert_eq!(validate_coordination_trace_alignment(&[item], &trace), Ok(()));
    }
}

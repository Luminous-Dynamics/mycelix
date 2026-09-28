//! D5 machine-readable decision/explanation trace for the Integral reference node.
//!
//! The trace is the authoritative *reference-model lineage* consumed by later
//! UI/explanation layers. Generated explanations are views over this trace, not
//! new evidence or authority.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_domain::{ProvenanceClass, SourceKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceKind {
    Proposal,
    Design,
    Decision,
    Authorization,
    ExecutionIntent,
    Observation,
    ItcProjection,
    FrsAssessment,
    Recommendation,
    HumanDecision,
    Outcome,
    Appeal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceActor {
    Human,
    Symthaea,
    System,
}


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceRelation {
    Supports,
    Authorizes,
    RespondsTo,
    Disputes,
    Supersedes,
    AlternativeTo,
    Appeals,
    Reopens,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceStatus {
    Proposed,
    Accepted,
    Rejected,
    Superseded,
    Disputed,
    Appealed,
    Reopened,
    Reversed,
    Executed,
    Closed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceRelationRef {
    pub from_event: &'static str,
    pub to_event: &'static str,
    pub relation: TraceRelation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceEvent {
    pub event_id: &'static str,
    pub sequence: u32,
    pub kind: TraceKind,
    pub provenance: ProvenanceClass,
    pub actor: TraceActor,
    pub source: SourceKind,
    pub source_ref: &'static str,
    pub generation: u32,
    pub uncertainty_present: bool,
    pub authority_ref: Option<&'static str>,
    pub reversible: bool,
    pub challengeable: bool,
    pub recommendation_only: bool,
    pub recovery_ref: Option<&'static str>,
    pub appeal_ref: Option<&'static str>,
    /// Explicit human decision disposition; absence means no disposition is claimed.
    pub decision_accepted: Option<bool>,
    pub status: TraceStatus,
    pub relations: &'static [TraceRelationRef],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceError {
    EmptyIdentity,
    EmptySource,
    SequenceRegression,
    GenerationRegression,
    IllegalTransition,
    AuthorityOnRecommendation,
    MissingAuthorization,
    UnchallengeableConsequentialAction,
    UnreversibleWithoutRecovery,
    MissingAppealRoute,
    MissingDecisionDisposition,
    UncertaintyLoss,
    ProvenanceMutation,
    ForeignOriginLoss,
    DuplicateIdentityMutation,
    SupersededLineage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExplanationLevel {
    Summary,
    Rationale,
    Assurance,
    Technical,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExplanationRequest {
    pub level: ExplanationLevel,
    pub symthaea_available: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExplanationView {
    pub trace_ref: &'static str,
    pub level: ExplanationLevel,
    pub generated_by: TraceActor,
    pub authoritative: bool,
    pub includes_evidence: bool,
    pub includes_authority: bool,
    pub includes_uncertainty: bool,
    pub includes_recovery: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceDigest {
    pub event_count: u32,
    pub last_sequence: u32,
    pub generation: u32,
    pub uncertainty_present: bool,
    pub foreign_origin_present: bool,
}

fn expected_provenance(kind: TraceKind) -> ProvenanceClass {
    match kind {
        TraceKind::Proposal => ProvenanceClass::Proposal,
        TraceKind::Design => ProvenanceClass::Design,
        TraceKind::Decision | TraceKind::HumanDecision => ProvenanceClass::Decision,
        TraceKind::Authorization => ProvenanceClass::Authorization,
        TraceKind::ExecutionIntent => ProvenanceClass::ExecutionIntent,
        TraceKind::Observation => ProvenanceClass::Observation,
        TraceKind::ItcProjection | TraceKind::FrsAssessment => ProvenanceClass::Assessment,
        TraceKind::Recommendation => ProvenanceClass::Recommendation,
        TraceKind::Outcome => ProvenanceClass::Outcome,
        TraceKind::Appeal => ProvenanceClass::Appeal,
    }
}

fn transition_allowed(from: TraceKind, to: TraceKind) -> bool {
    matches!(
        (from, to),
        (TraceKind::Proposal, TraceKind::Design)
            | (TraceKind::Design, TraceKind::Decision)
            | (TraceKind::Decision, TraceKind::Authorization)
            | (TraceKind::Authorization, TraceKind::ExecutionIntent)
            | (TraceKind::ExecutionIntent, TraceKind::Observation)
            | (TraceKind::Observation, TraceKind::ItcProjection)
            | (TraceKind::ItcProjection, TraceKind::FrsAssessment)
            | (TraceKind::FrsAssessment, TraceKind::Recommendation)
            | (TraceKind::Recommendation, TraceKind::HumanDecision)
            | (TraceKind::HumanDecision, TraceKind::Outcome)
            | (TraceKind::Outcome, TraceKind::Appeal)
            | (TraceKind::Decision, TraceKind::Appeal)
            | (TraceKind::Authorization, TraceKind::Appeal)
            | (TraceKind::Recommendation, TraceKind::Appeal)
            | (TraceKind::FrsAssessment, TraceKind::Appeal)
            | (TraceKind::Observation, TraceKind::Appeal)
            | (TraceKind::Observation, TraceKind::Observation)
    )
}

/// Validate a complete trace without consulting any presentation or AI layer.
pub fn validate_trace(events: &[TraceEvent]) -> Result<(), TraceError> {
    if events.is_empty() {
        return Err(TraceError::EmptyIdentity);
    }

    for (index, event) in events.iter().enumerate() {
        if event.event_id.is_empty() || event.source_ref.is_empty() {
            return Err(TraceError::EmptyIdentity);
        }
        if event.provenance != expected_provenance(event.kind) {
            return Err(TraceError::ProvenanceMutation);
        }
        if events.iter().take(index).any(|prior| prior.event_id == event.event_id) {
            return Err(TraceError::DuplicateIdentityMutation);
        }
        if event.event_id.is_empty() || event.source_ref.is_empty() {
            return Err(TraceError::EmptyIdentity);
        }
        if event.kind != TraceKind::Proposal && event.source_ref.is_empty() {
            return Err(TraceError::EmptySource);
        }
        if event.kind == TraceKind::Recommendation {
            if !event.recommendation_only || event.authority_ref.is_some() {
                return Err(TraceError::AuthorityOnRecommendation);
            }
        }
        if matches!(event.kind, TraceKind::Authorization | TraceKind::HumanDecision)
            && event.authority_ref.is_none()
        {
            return Err(TraceError::MissingAuthorization);
        }
        if event.kind == TraceKind::Outcome && !event.challengeable {
            return Err(TraceError::UnchallengeableConsequentialAction);
        }
        if event.kind == TraceKind::HumanDecision && event.decision_accepted.is_none() {
            return Err(TraceError::MissingDecisionDisposition);
        }
        if event.kind == TraceKind::Outcome {
            if !event.reversible && event.recovery_ref.is_none() {
                return Err(TraceError::UnreversibleWithoutRecovery);
            }
            if event.challengeable && event.appeal_ref.is_none() {
                return Err(TraceError::MissingAppealRoute);
            }
        }
    }

    for relation in events.iter().flat_map(|e| e.relations.iter()) {
        let from = events.iter().find(|e| e.event_id == relation.from_event);
        let to = events.iter().find(|e| e.event_id == relation.to_event);
        if from.is_none() || to.is_none() || relation.from_event == relation.to_event {
            return Err(TraceError::SupersededLineage);
        }
        if relation.relation == TraceRelation::Authorizes && to.unwrap().kind == TraceKind::Recommendation {
            return Err(TraceError::AuthorityOnRecommendation);
        }
        if relation.relation == TraceRelation::Supersedes && to.unwrap().generation >= from.unwrap().generation {
            return Err(TraceError::SupersededLineage);
        }
    }

    // Relations are authoritative graph edges. They allow branches and
    // non-adjacent lineage without weakening the deterministic sequence check.
    for relation in events.iter().flat_map(|e| e.relations.iter()) {
        let from = events.iter().find(|e| e.event_id == relation.from_event).unwrap();
        let to = events.iter().find(|e| e.event_id == relation.to_event).unwrap();
        match relation.relation {
            TraceRelation::Disputes => {
                if from.kind != TraceKind::Observation || to.kind != TraceKind::Observation {
                    return Err(TraceError::IllegalTransition);
                }
                if from.source != to.source || from.generation != to.generation {
                    return Err(TraceError::ProvenanceMutation);
                }
            }
            TraceRelation::Reopens => {
                if from.kind != TraceKind::Appeal {
                    return Err(TraceError::IllegalTransition);
                }
                if !matches!(to.kind, TraceKind::Decision | TraceKind::Authorization | TraceKind::HumanDecision | TraceKind::Outcome) {
                    return Err(TraceError::IllegalTransition);
                }
            }
            TraceRelation::RespondsTo | TraceRelation::Appeals => {
                if from.sequence <= to.sequence {
                    return Err(TraceError::SequenceRegression);
                }
            }
            _ => {}
        }
    }

    for pair in events.windows(2) {
        let previous = pair[0];
        let current = pair[1];
        if current.sequence <= previous.sequence {
            return Err(TraceError::SequenceRegression);
        }
        if current.generation < previous.generation {
            return Err(TraceError::GenerationRegression);
        }
        if !transition_allowed(previous.kind, current.kind) {
            return Err(TraceError::IllegalTransition);
        }
        if previous.kind == TraceKind::Observation && current.kind == TraceKind::Observation
            && !events.iter().any(|e| e.relations.iter().any(|r| {
                r.relation == TraceRelation::Disputes
                    && ((r.from_event == previous.event_id && r.to_event == current.event_id)
                        || (r.from_event == current.event_id && r.to_event == previous.event_id))
            }))
        {
            return Err(TraceError::IllegalTransition);
        }
        if previous.uncertainty_present && !current.uncertainty_present {
            return Err(TraceError::UncertaintyLoss);
        }
        if previous.source != current.source
            && previous.source == SourceKind::Foreign
            && current.source == SourceKind::Local
        {
            return Err(TraceError::ForeignOriginLoss);
        }
    }

    Ok(())
}

/// The same logical event may be replayed, but its authoritative payload may not mutate.
pub fn replay_is_idempotent(existing: &TraceEvent, replay: &TraceEvent) -> bool {
    existing.event_id == replay.event_id && existing == replay
}

/// Explanations are deterministic views over trace identity; they cannot become authority.
pub fn explanation_view(
    request: ExplanationRequest,
    trace_ref: &'static str,
) -> Option<ExplanationView> {
    if trace_ref.is_empty() {
        return None;
    }

    Some(ExplanationView {
        trace_ref,
        level: request.level,
        generated_by: if request.symthaea_available {
            TraceActor::Symthaea
        } else {
            TraceActor::System
        },
        authoritative: false,
        includes_evidence: matches!(
            request.level,
            ExplanationLevel::Rationale | ExplanationLevel::Assurance | ExplanationLevel::Technical
        ),
        includes_authority: matches!(
            request.level,
            ExplanationLevel::Assurance | ExplanationLevel::Technical
        ),
        includes_uncertainty: true,
        includes_recovery: matches!(
            request.level,
            ExplanationLevel::Assurance | ExplanationLevel::Technical
        ),
    })
}

/// Compute only descriptive trace properties; this is not a legitimacy or human-outcome score.
pub fn digest(events: &[TraceEvent]) -> Option<TraceDigest> {
    let last = events.last()?;
    Some(TraceDigest {
        event_count: events.len() as u32,
        last_sequence: last.sequence,
        generation: last.generation,
        uncertainty_present: events.iter().any(|e| e.uncertainty_present),
        foreign_origin_present: events.iter().any(|e| e.source == SourceKind::Foreign),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(
        event_id: &'static str,
        sequence: u32,
        kind: TraceKind,
        actor: TraceActor,
        source: SourceKind,
        generation: u32,
        uncertainty_present: bool,
        authority_ref: Option<&'static str>,
        reversible: bool,
        challengeable: bool,
        recommendation_only: bool,
    ) -> TraceEvent {
        TraceEvent {
            event_id,
            sequence,
            kind,
            provenance: match kind {
                TraceKind::Proposal => ProvenanceClass::Proposal,
                TraceKind::Design => ProvenanceClass::Design,
                TraceKind::Decision | TraceKind::HumanDecision => ProvenanceClass::Decision,
                TraceKind::Authorization => ProvenanceClass::Authorization,
                TraceKind::ExecutionIntent => ProvenanceClass::ExecutionIntent,
                TraceKind::Observation => ProvenanceClass::Observation,
                TraceKind::ItcProjection => ProvenanceClass::Assessment,
                TraceKind::FrsAssessment => ProvenanceClass::Assessment,
                TraceKind::Recommendation => ProvenanceClass::Recommendation,
                TraceKind::Outcome => ProvenanceClass::Outcome,
                TraceKind::Appeal => ProvenanceClass::Appeal,
            },
            actor,
            source,
            source_ref: "evidence://demo",
            generation,
            uncertainty_present,
            authority_ref,
            reversible,
            challengeable,
            recommendation_only,
            recovery_ref: if kind == TraceKind::Outcome { Some("recovery-1") } else { None },
            appeal_ref: if kind == TraceKind::Outcome { Some("appeal-1") } else { None },
            decision_accepted: if kind == TraceKind::HumanDecision { Some(true) } else { None },
            status: match kind { TraceKind::Decision | TraceKind::Authorization | TraceKind::HumanDecision => TraceStatus::Accepted, TraceKind::Outcome => TraceStatus::Executed, _ => TraceStatus::Accepted },
            relations: &[],
        }
    }

    fn valid_trace() -> Vec<TraceEvent> {
        vec![
            event("e1", 1, TraceKind::Proposal, TraceActor::Human, SourceKind::Local, 7, true, None, true, true, false),
            event("e2", 2, TraceKind::Design, TraceActor::System, SourceKind::Local, 7, true, None, true, true, false),
            event("e3", 3, TraceKind::Decision, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-cds"), true, true, false),
            event("e4", 4, TraceKind::Authorization, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-prod"), true, true, false),
            event("e5", 5, TraceKind::ExecutionIntent, TraceActor::System, SourceKind::Local, 7, true, Some("auth-prod"), true, true, false),
            event("e6", 6, TraceKind::Observation, TraceActor::System, SourceKind::Local, 7, true, None, true, true, false),
            event("e7", 7, TraceKind::ItcProjection, TraceActor::System, SourceKind::Local, 7, true, None, true, true, false),
            event("e8", 8, TraceKind::FrsAssessment, TraceActor::System, SourceKind::Local, 7, true, None, true, true, false),
            event("e9", 9, TraceKind::Recommendation, TraceActor::Symthaea, SourceKind::Local, 7, true, None, true, true, true),
            event("e10", 10, TraceKind::HumanDecision, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-review"), true, true, false),
            event("e11", 11, TraceKind::Outcome, TraceActor::System, SourceKind::Local, 7, true, Some("auth-review"), true, true, false),
            event("e12", 12, TraceKind::Appeal, TraceActor::Human, SourceKind::Local, 7, true, None, true, true, false),
        ]
    }

    #[test]
    fn complete_trace_validates() {
        assert_eq!(validate_trace(&valid_trace()), Ok(()));
    }

    #[test]
    fn provenance_class_cannot_be_relabelled() {
        let mut t = valid_trace();
        t[5].provenance = ProvenanceClass::Assessment;
        assert_eq!(validate_trace(&t), Err(TraceError::ProvenanceMutation));

        let mut t = valid_trace();
        t[7].provenance = ProvenanceClass::Observation;
        assert_eq!(validate_trace(&t), Err(TraceError::ProvenanceMutation));
    }

    #[test]
    fn event_identity_cannot_be_reused_for_a_different_event() {
        let mut t = valid_trace();
        t[6].event_id = t[5].event_id;
        assert_eq!(validate_trace(&t), Err(TraceError::DuplicateIdentityMutation));
    }

    #[test]
    fn recommendation_cannot_carry_authority() {
        let mut t = valid_trace();
        t[8].authority_ref = Some("forbidden");
        assert_eq!(validate_trace(&t), Err(TraceError::AuthorityOnRecommendation));
    }

    #[test]
    fn recommendation_must_remain_recommendation_only() {
        let mut t = valid_trace();
        t[8].recommendation_only = false;
        assert_eq!(validate_trace(&t), Err(TraceError::AuthorityOnRecommendation));
    }

    #[test]
    fn explanation_is_never_authoritative() {
        let view = explanation_view(
            ExplanationRequest { level: ExplanationLevel::Technical, symthaea_available: true },
            "trace://demo-001",
        ).expect("view");
        assert!(!view.authoritative);
        assert_eq!(view.trace_ref, "trace://demo-001");
        assert_eq!(view.generated_by, TraceActor::Symthaea);
    }

    #[test]
    fn system_fallback_works_without_symthaea() {
        let view = explanation_view(
            ExplanationRequest { level: ExplanationLevel::Summary, symthaea_available: false },
            "trace://demo-001",
        ).expect("view");
        assert_eq!(view.generated_by, TraceActor::System);
        assert!(!view.authoritative);
    }

    #[test]
    fn uncertainty_cannot_disappear() {
        let mut t = valid_trace();
        t[7].uncertainty_present = false;
        assert_eq!(validate_trace(&t), Err(TraceError::UncertaintyLoss));
    }

    #[test]
    fn generation_cannot_regress() {
        let mut t = valid_trace();
        t[6].generation = 8;
        assert_eq!(validate_trace(&t), Err(TraceError::GenerationRegression));
    }

    #[test]
    fn foreign_origin_cannot_be_laundered_to_local() {
        let mut t = valid_trace();
        t[5].source = SourceKind::Foreign;
        t[6].source = SourceKind::Local;
        assert_eq!(validate_trace(&t), Err(TraceError::ForeignOriginLoss));
    }

    #[test]
    fn replay_cannot_mutate_authoritative_payload() {
        let t = valid_trace();
        assert!(replay_is_idempotent(&t[5], &t[5]));
        let mut replay = t[5];
        replay.source_ref = "evidence://mutated";
        assert!(!replay_is_idempotent(&t[5], &replay));
    }

    #[test]
    fn consequential_outcome_requires_recovery_and_contestability() {
        let mut t = valid_trace();
        t[10].reversible = false;
        t[10].recovery_ref = None;
        assert_eq!(validate_trace(&t), Err(TraceError::UnreversibleWithoutRecovery));

        let mut t = valid_trace();
        t[10].challengeable = false;
        assert_eq!(validate_trace(&t), Err(TraceError::UnchallengeableConsequentialAction));

        let mut t = valid_trace();
        t[10].appeal_ref = None;
        assert_eq!(validate_trace(&t), Err(TraceError::MissingAppealRoute));
    }

    #[test]
    fn human_decision_requires_explicit_authority_reference() {
        let mut t = valid_trace();
        t[9].authority_ref = None;
        assert_eq!(validate_trace(&t), Err(TraceError::MissingAuthorization));
    }

    #[test]
    fn digest_is_descriptive_not_legitimacy() {
        let d = digest(&valid_trace()).expect("digest");
        assert_eq!(d.event_count, 12);
        assert!(d.uncertainty_present);
        assert!(!d.foreign_origin_present);
    }

    #[test]
    fn all_trace_kinds_have_explicit_provenance_mapping() {
        let t = valid_trace();
        assert!(t.iter().all(|e| matches!(
            e.provenance,
            ProvenanceClass::Proposal
                | ProvenanceClass::Design
                | ProvenanceClass::Decision
                | ProvenanceClass::Authorization
                | ProvenanceClass::ExecutionIntent
                | ProvenanceClass::Observation
                | ProvenanceClass::Assessment
                | ProvenanceClass::Recommendation
                | ProvenanceClass::Outcome
                | ProvenanceClass::Appeal
        )));
    }
}

//! D6C executable adversarial scenario corpus for the Integral cockpit.
//!
//! A ScenarioFixture is the single source of truth for events, graph relations,
//! expected validation, and claim ceiling. Presentation code consumes this
//! owned artifact; it does not maintain scenario-specific truth.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_domain::{ProvenanceClass, SourceKind};
use crate::integral_demo_trace::{
    validate_trace, TraceActor, TraceError, TraceEvent, TraceFixture, TraceKind,
    TraceRelation, TraceRelationRef, TraceStatus,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioId {
    NormalFlow,
    RejectedCdsDecision,
    RejectedDecisionDescendant,
    StaleDesign,
    SupersededDesignUsed,
    UncertainObservation,
    ConflictingObservations,
    RecommendationAccepted,
    RecommendationRejected,
    ForeignEvidence,
    AppealedOutcome,
    AppealReopenWithoutNewPath,
    AppealReopenWithNewPath,
    NoSymthaea,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioExpected {
    Valid,
    Invalid(TraceError),
    PresentationOnly,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScenarioFixture {
    pub id: ScenarioId,
    pub name: &'static str,
    pub summary: &'static str,
    pub expected: ScenarioExpected,
    pub claim_ceiling: &'static str,
    pub symthaea_used: bool,
    pub trace_ref: &'static str,
    pub trace: TraceFixture,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioResult {
    pub id: ScenarioId,
    pub name: &'static str,
    pub summary: &'static str,
    pub expected: ScenarioExpected,
    pub actual_valid: bool,
    pub validation_error: Option<TraceError>,
    pub trace_len: usize,
    pub relation_count: usize,
    pub claim_ceiling: &'static str,
    pub symthaea_used: bool,
}

fn event(
    id: &'static str,
    sequence: u32,
    kind: TraceKind,
    actor: TraceActor,
    source: SourceKind,
    generation: u32,
    uncertainty: bool,
    authority: Option<&'static str>,
    recommendation_only: bool,
    decision_accepted: Option<bool>,
    status: TraceStatus,
) -> TraceEvent {
    TraceEvent {
        event_id: id,
        sequence,
        kind,
        provenance: match kind {
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
        },
        actor,
        source,
        source_ref: "evidence://integral-demo-d6c",
        generation,
        uncertainty_present: uncertainty,
        authority_ref: authority,
        reversible: true,
        challengeable: true,
        recommendation_only,
        recovery_ref: if kind == TraceKind::Outcome { Some("recovery://outcome") } else { None },
        appeal_ref: if kind == TraceKind::Outcome { Some("appeal://outcome") } else { None },
        decision_accepted,
        status,
    }
}

fn relation(from_event: &'static str, to_event: &'static str, relation: TraceRelation) -> TraceRelationRef {
    TraceRelationRef { from_event, to_event, relation }
}

fn base_events() -> Vec<TraceEvent> {
    vec![
        event("p1", 1, TraceKind::Proposal, TraceActor::Human, SourceKind::Local, 7, true, None, false, None, TraceStatus::Accepted),
        event("d1", 2, TraceKind::Design, TraceActor::System, SourceKind::Local, 7, true, None, false, None, TraceStatus::Accepted),
        event("c1", 3, TraceKind::Decision, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-cds"), false, Some(true), TraceStatus::Accepted),
        event("a1", 4, TraceKind::Authorization, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-production"), false, None, TraceStatus::Accepted),
        event("x1", 5, TraceKind::ExecutionIntent, TraceActor::System, SourceKind::Local, 7, true, Some("auth-production"), false, None, TraceStatus::Accepted),
        event("o1", 6, TraceKind::Observation, TraceActor::System, SourceKind::Local, 7, true, None, false, None, TraceStatus::Accepted),
        event("i1", 7, TraceKind::ItcProjection, TraceActor::System, SourceKind::Local, 7, true, None, false, None, TraceStatus::Accepted),
        event("f1", 8, TraceKind::FrsAssessment, TraceActor::System, SourceKind::Local, 7, true, None, false, None, TraceStatus::Accepted),
        event("r1", 9, TraceKind::Recommendation, TraceActor::Symthaea, SourceKind::Local, 7, true, None, true, None, TraceStatus::Proposed),
        event("h1", 10, TraceKind::HumanDecision, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-review"), false, Some(true), TraceStatus::Accepted),
        event("u1", 11, TraceKind::Outcome, TraceActor::System, SourceKind::Local, 7, true, Some("auth-review"), false, None, TraceStatus::Executed),
        event("ap1", 12, TraceKind::Appeal, TraceActor::Human, SourceKind::Local, 7, true, None, false, None, TraceStatus::Proposed),
    ]
}

fn base_relations() -> Vec<TraceRelationRef> {
    vec![
        relation("d1", "p1", TraceRelation::Supports),
        relation("c1", "d1", TraceRelation::Supports),
        relation("a1", "c1", TraceRelation::Authorizes),
        relation("x1", "a1", TraceRelation::Authorizes),
        relation("i1", "o1", TraceRelation::Supports),
        relation("f1", "i1", TraceRelation::Supports),
        relation("r1", "f1", TraceRelation::Supports),
        relation("h1", "r1", TraceRelation::RespondsTo),
        relation("u1", "h1", TraceRelation::RespondsTo),
        relation("ap1", "u1", TraceRelation::Appeals),
    ]
}

fn fixture(
    id: ScenarioId,
    expected: ScenarioExpected,
    symthaea_used: bool,
    events: Vec<TraceEvent>,
    relations: Vec<TraceRelationRef>,
) -> ScenarioFixture {
    let (name, summary) = metadata(id);
    ScenarioFixture {
        id,
        name,
        summary,
        expected,
        claim_ceiling: "ReferenceModelOnly",
        symthaea_used,
        trace_ref: "trace://integral-demo-d6c",
        trace: TraceFixture { events, relations },
    }
}

fn metadata(id: ScenarioId) -> (&'static str, &'static str) {
    match id {
        ScenarioId::NormalFlow => ("Normal flow", "Complete bounded lifecycle."),
        ScenarioId::RejectedCdsDecision => ("Rejected CDS decision", "Missing explicit decision authority must fail closed."),
        ScenarioId::RejectedDecisionDescendant => ("Rejected decision with descendant", "A rejected decision cannot authorize an executable descendant."),
        ScenarioId::StaleDesign => ("Stale design", "A superseded design generation cannot become current."),
        ScenarioId::SupersededDesignUsed => ("Superseded design used", "A later action cannot consume a superseded design."),
        ScenarioId::UncertainObservation => ("Uncertain observation", "Uncertainty loss is rejected."),
        ScenarioId::ConflictingObservations => ("Conflicting observations", "Two observations remain distinguishable through an explicit dispute edge."),
        ScenarioId::RecommendationAccepted => ("Recommendation accepted", "A recommendation can inform a later human decision."),
        ScenarioId::RecommendationRejected => ("Recommendation rejected", "A recommendation can be rejected without breaking authority boundaries."),
        ScenarioId::ForeignEvidence => ("Foreign evidence", "Foreign origin cannot be silently laundered into local origin."),
        ScenarioId::AppealedOutcome => ("Appealed outcome", "An appeal without a new decision path fails closed."),
        ScenarioId::AppealReopenWithoutNewPath => ("Appeal reopen without new path", "Reopening requires a new explicit decision path."),
        ScenarioId::AppealReopenWithNewPath => ("Appeal reopen with new path", "A reopened review can create a new explicit decision path."),
        ScenarioId::NoSymthaea => ("No-Symthaea fallback", "The trace remains usable without Symthaea."),
    }
}

pub fn fixture_for(id: ScenarioId) -> ScenarioFixture {
    let mut events = base_events();
    let mut relations = base_relations();

    match id {
        ScenarioId::NormalFlow => {}
        ScenarioId::RecommendationAccepted => {
            events[8].status = TraceStatus::Accepted;
            events[9].decision_accepted = Some(true);
            events[9].status = TraceStatus::Accepted;
        }
        ScenarioId::RecommendationRejected => {
            events[8].status = TraceStatus::Rejected;
            events[9].decision_accepted = Some(false);
            events[9].status = TraceStatus::Rejected;
        }
        ScenarioId::RejectedDecisionDescendant => {
            events[2].status = TraceStatus::Rejected;
            events[2].decision_accepted = Some(false);
            relations.push(relation("a1", "c1", TraceRelation::Supports));
        }
        ScenarioId::SupersededDesignUsed => {
            let mut prior = events[1];
            prior.event_id = "d0";
            prior.sequence = 2;
            prior.generation = 6;
            prior.status = TraceStatus::Superseded;
            events[1].event_id = "d1";
            events[1].sequence = 3;
            events[1].generation = 7;
            events.insert(1, prior);
            for (index, event) in events.iter_mut().enumerate().skip(2) {
                event.sequence = (index as u32) + 2;
            }
            relations.retain(|r| r.from_event != "d1" && r.to_event != "d1" && r.from_event != "d0" && r.to_event != "d0");
            relations.push(relation("d1", "d0", TraceRelation::Supersedes));
            relations.push(relation("c1", "d0", TraceRelation::Supports));
        }
        ScenarioId::RejectedCdsDecision => {
            events[2].status = TraceStatus::Rejected;
            events[2].decision_accepted = Some(false);
            events[2].authority_ref = None;
            events.truncate(3);
            relations.retain(|r| r.from_event != "a1" && r.from_event != "x1");
        }
        ScenarioId::StaleDesign => {
            let mut prior = events[1];
            prior.event_id = "d0";
            prior.sequence = 2;
            prior.generation = 6;
            prior.status = TraceStatus::Superseded;
            events[1].event_id = "d1";
            events[1].sequence = 3;
            events[1].generation = 7;
            events.insert(1, prior);
            for (index, event) in events.iter_mut().enumerate().skip(2) {
                event.sequence = (index as u32) + 2;
            }
            events[3].generation = 6;
            relations.retain(|r| r.from_event != "d1" && r.to_event != "d1" && r.from_event != "d0" && r.to_event != "d0");
            relations.push(relation("d1", "d0", TraceRelation::Supersedes));
        }
        ScenarioId::UncertainObservation => {
            events[5].uncertainty_present = false;
        }
        ScenarioId::ConflictingObservations => {
            let second = event("o2", 7, TraceKind::Observation, TraceActor::System, SourceKind::Local, 7, true, None, false, None, TraceStatus::Disputed);
            events.truncate(6);
            events.push(second);
            relations.truncate(4);
            relations.push(relation("o2", "o1", TraceRelation::Disputes));
        }
        ScenarioId::ForeignEvidence => {
            for index in 5..9 {
                events[index].source = SourceKind::Foreign;
            }
        }
        ScenarioId::AppealedOutcome => {
            events[10].status = TraceStatus::Reversed;
            events[11].status = TraceStatus::Reopened;
            relations.push(relation("ap1", "h1", TraceRelation::Reopens));
        }
        ScenarioId::AppealReopenWithoutNewPath => {
            events[10].status = TraceStatus::Reversed;
            events[11].status = TraceStatus::Reopened;
            relations.push(relation("ap1", "h1", TraceRelation::Reopens));
        }
        ScenarioId::AppealReopenWithNewPath => {
            events[10].status = TraceStatus::Reversed;
            events[11].status = TraceStatus::Reopened;
            relations.push(relation("ap1", "h1", TraceRelation::Reopens));
            events.push(event("c2", 13, TraceKind::Decision, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-reopen"), false, Some(true), TraceStatus::Accepted));
            events.push(event("a2", 14, TraceKind::Authorization, TraceActor::Human, SourceKind::Local, 7, true, Some("auth-reopen"), false, None, TraceStatus::Accepted));
            relations.push(relation("c2", "ap1", TraceRelation::RespondsTo));
            relations.push(relation("a2", "c2", TraceRelation::Authorizes));
        }
        ScenarioId::NoSymthaea => {
            events[8].actor = TraceActor::System;
        }
    }

    let expected = match id {
        ScenarioId::NormalFlow | ScenarioId::RecommendationAccepted | ScenarioId::RecommendationRejected
        | ScenarioId::AppealReopenWithNewPath | ScenarioId::NoSymthaea => ScenarioExpected::Valid,
        ScenarioId::RejectedCdsDecision => ScenarioExpected::Invalid(TraceError::MissingAuthorization),
        ScenarioId::RejectedDecisionDescendant => ScenarioExpected::Invalid(TraceError::RejectedDecisionHasDescendant),
        ScenarioId::StaleDesign => ScenarioExpected::Invalid(TraceError::GenerationRegression),
        ScenarioId::SupersededDesignUsed => ScenarioExpected::Invalid(TraceError::SupersededDesignUsed),
        ScenarioId::UncertainObservation => ScenarioExpected::Invalid(TraceError::UncertaintyLoss),
        ScenarioId::ConflictingObservations => ScenarioExpected::PresentationOnly,
        ScenarioId::ForeignEvidence => ScenarioExpected::Invalid(TraceError::ForeignOriginLoss),
        ScenarioId::AppealedOutcome | ScenarioId::AppealReopenWithoutNewPath => ScenarioExpected::Invalid(TraceError::ReopenRequiresNewPath),
    };

    fixture(
        id,
        expected,
        !matches!(id, ScenarioId::NoSymthaea),
        events,
        relations,
    )
}

pub fn evaluate(id: ScenarioId) -> ScenarioResult {
    let fixture = fixture_for(id);
    let validation = validate_trace(&fixture.trace);
    ScenarioResult {
        id,
        name: fixture.name,
        summary: fixture.summary,
        expected: fixture.expected,
        actual_valid: validation.is_ok(),
        validation_error: validation.err(),
        trace_len: fixture.trace.events.len(),
        relation_count: fixture.trace.relations.len(),
        claim_ceiling: fixture.claim_ceiling,
        symthaea_used: fixture.symthaea_used,
    }
}

pub const ALL_SCENARIOS: [ScenarioId; 10] = [
    ScenarioId::NormalFlow,
    ScenarioId::RejectedCdsDecision,
    ScenarioId::RejectedDecisionDescendant,
    ScenarioId::StaleDesign,
    ScenarioId::SupersededDesignUsed,
    ScenarioId::UncertainObservation,
    ScenarioId::ConflictingObservations,
    ScenarioId::RecommendationAccepted,
    ScenarioId::RecommendationRejected,
    ScenarioId::ForeignEvidence,
    ScenarioId::AppealedOutcome,
    ScenarioId::AppealReopenWithoutNewPath,
    ScenarioId::AppealReopenWithNewPath,
    ScenarioId::NoSymthaea,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normal_and_no_ai_paths_validate() {
        assert!(evaluate(ScenarioId::NormalFlow).actual_valid);
        assert!(evaluate(ScenarioId::NoSymthaea).actual_valid);
    }

    #[test]
    fn adversarial_scenarios_fail_at_the_declared_boundary() {
        for id in [
            ScenarioId::RejectedCdsDecision,
            ScenarioId::StaleDesign,
            ScenarioId::UncertainObservation,
            ScenarioId::ForeignEvidence,
        ] {
            let result = evaluate(id);
            assert!(!result.actual_valid, "{:?} unexpectedly validated", id);
            assert!(matches!(result.expected, ScenarioExpected::Invalid(_)));
        }
    }

    #[test]
    fn recommendation_acceptance_and_rejection_keep_the_same_trace_boundary() {
        assert!(evaluate(ScenarioId::RecommendationAccepted).actual_valid);
        assert!(evaluate(ScenarioId::RecommendationRejected).actual_valid);
    }

    #[test]
    fn conflict_is_explicitly_graph_native() {
        let fixture = fixture_for(ScenarioId::ConflictingObservations);
        assert!(fixture.trace.relations.iter().any(|r| r.relation == TraceRelation::Disputes));
        assert!(evaluate(ScenarioId::ConflictingObservations).actual_valid);
    }

    #[test]
    fn stale_design_has_owned_supersession_edge() {
        let fixture = fixture_for(ScenarioId::StaleDesign);
        assert!(fixture.trace.relations.iter().any(|r| {
            r.from_event == "d1" && r.to_event == "d0" && r.relation == TraceRelation::Supersedes
        }));
        assert_eq!(evaluate(ScenarioId::StaleDesign).validation_error, Some(TraceError::GenerationRegression));
    }

    #[test]
    fn accepted_and_rejected_recommendations_have_explicit_human_disposition() {
        for (id, accepted) in [
            (ScenarioId::RecommendationAccepted, true),
            (ScenarioId::RecommendationRejected, false),
        ] {
            let fixture = fixture_for(id);
            assert_eq!(fixture.trace.events[9].decision_accepted, Some(accepted));
        }
    }

    #[test]
    fn every_scenario_has_a_claim_ceiling() {
        for id in ALL_SCENARIOS {
            assert_eq!(evaluate(id).claim_ceiling, "ReferenceModelOnly");
        }
    }
}

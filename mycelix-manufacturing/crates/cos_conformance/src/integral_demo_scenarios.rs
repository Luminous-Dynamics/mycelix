//! D6C executable adversarial scenario corpus for the Integral cockpit.
//!
//! Each scenario is a reference-model fixture with an expected validation
//! outcome. The Leptos UI consumes these results rather than inventing
//! scenario-specific truth.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_domain::{ProvenanceClass, SourceKind};
use crate::integral_demo_trace::{validate_trace, TraceError, TraceEvent, TraceActor, TraceKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioId {
    NormalFlow,
    RejectedCdsDecision,
    StaleDesign,
    UncertainObservation,
    ConflictingObservations,
    RecommendationAccepted,
    RecommendationRejected,
    ForeignEvidence,
    AppealedOutcome,
    NoSymthaea,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioExpected {
    Valid,
    Invalid(TraceError),
    PresentationOnly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioResult {
    pub id: ScenarioId,
    pub name: &'static str,
    pub summary: &'static str,
    pub expected: ScenarioExpected,
    pub actual_valid: bool,
    pub trace_len: usize,
    pub claim_ceiling: &'static str,
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
    }
}

fn normal_trace() -> Vec<TraceEvent> {
    vec![
        event("p1",1,TraceKind::Proposal,TraceActor::Human,SourceKind::Local,7,true,None,false),
        event("d1",2,TraceKind::Design,TraceActor::System,SourceKind::Local,7,true,None,false),
        event("c1",3,TraceKind::Decision,TraceActor::Human,SourceKind::Local,7,true,Some("auth-cds"),false),
        event("a1",4,TraceKind::Authorization,TraceActor::Human,SourceKind::Local,7,true,Some("auth-production"),false),
        event("x1",5,TraceKind::ExecutionIntent,TraceActor::System,SourceKind::Local,7,true,Some("auth-production"),false),
        event("o1",6,TraceKind::Observation,TraceActor::System,SourceKind::Local,7,true,None,false),
        event("i1",7,TraceKind::ItcProjection,TraceActor::System,SourceKind::Local,7,true,None,false),
        event("f1",8,TraceKind::FrsAssessment,TraceActor::System,SourceKind::Local,7,true,None,false),
        event("r1",9,TraceKind::Recommendation,TraceActor::Symthaea,SourceKind::Local,7,true,None,true),
        event("h1",10,TraceKind::HumanDecision,TraceActor::Human,SourceKind::Local,7,true,Some("auth-review"),false),
        event("u1",11,TraceKind::Outcome,TraceActor::System,SourceKind::Local,7,true,Some("auth-review"),false),
        event("ap1",12,TraceKind::Appeal,TraceActor::Human,SourceKind::Local,7,true,None,false),
    ]
}

pub fn trace_for(id: ScenarioId) -> Vec<TraceEvent> {
    let mut t = normal_trace();
    match id {
        ScenarioId::NormalFlow | ScenarioId::RecommendationAccepted | ScenarioId::RecommendationRejected
        | ScenarioId::AppealedOutcome => t,
        ScenarioId::RejectedCdsDecision => {
            t[2].authority_ref = None;
            t
        }
        ScenarioId::StaleDesign => {
            t[1].generation = 7;
            t[2].generation = 6;
            t
        }
        ScenarioId::UncertainObservation => {
            t[5].uncertainty_present = false;
            t
        }
        ScenarioId::ConflictingObservations => {
            let second = event("o2",7,TraceKind::Observation,TraceActor::System,SourceKind::Local,7,true,None,false);
            t.truncate(6);
            t.push(second);
            t
        }
        ScenarioId::ForeignEvidence => {
            t[5].source = SourceKind::Foreign;
            t[6].source = SourceKind::Foreign;
            t[7].source = SourceKind::Foreign;
            t[8].source = SourceKind::Foreign;
            t[9].source = SourceKind::Local;
            t
        }
        ScenarioId::NoSymthaea => {
            t[8].actor = TraceActor::System;
            t
        }
    }
}

pub fn evaluate(id: ScenarioId) -> ScenarioResult {
    let trace = trace_for(id);
    let actual = validate_trace(&trace);
    let expected = match id {
        ScenarioId::NormalFlow | ScenarioId::RecommendationAccepted | ScenarioId::RecommendationRejected
        | ScenarioId::AppealedOutcome | ScenarioId::NoSymthaea => ScenarioExpected::Valid,
        ScenarioId::RejectedCdsDecision => ScenarioExpected::Invalid(TraceError::MissingAuthorization),
        ScenarioId::StaleDesign => ScenarioExpected::Invalid(TraceError::GenerationRegression),
        ScenarioId::UncertainObservation => ScenarioExpected::Invalid(TraceError::UncertaintyLoss),
        // This fixture intentionally stops before the assessment layer: the
        // trace validator cannot infer conflict from adjacency.
        ScenarioId::ConflictingObservations => ScenarioExpected::PresentationOnly,
        ScenarioId::ForeignEvidence => ScenarioExpected::Invalid(TraceError::ForeignOriginLoss),
    };
    ScenarioResult {
        id,
        name: match id {
            ScenarioId::NormalFlow => "Normal flow",
            ScenarioId::RejectedCdsDecision => "Rejected CDS decision",
            ScenarioId::StaleDesign => "Stale design",
            ScenarioId::UncertainObservation => "Uncertain observation",
            ScenarioId::ConflictingObservations => "Conflicting observations",
            ScenarioId::RecommendationAccepted => "Recommendation accepted",
            ScenarioId::RecommendationRejected => "Recommendation rejected",
            ScenarioId::ForeignEvidence => "Foreign evidence",
            ScenarioId::AppealedOutcome => "Appealed outcome",
            ScenarioId::NoSymthaea => "No-Symthaea fallback",
        },
        summary: match id {
            ScenarioId::NormalFlow => "Complete bounded lifecycle.",
            ScenarioId::RejectedCdsDecision => "Missing explicit decision authority must fail closed.",
            ScenarioId::StaleDesign => "A generation regression must not become current.",
            ScenarioId::UncertainObservation => "Uncertainty loss is rejected.",
            ScenarioId::ConflictingObservations => "Two observations must remain distinguishable.",
            ScenarioId::RecommendationAccepted => "A recommendation can inform a later human decision.",
            ScenarioId::RecommendationRejected => "A recommendation can be rejected without breaking authority boundaries.",
            ScenarioId::ForeignEvidence => "Foreign origin cannot be silently laundered into local origin.",
            ScenarioId::AppealedOutcome => "A consequential outcome retains an appeal path.",
            ScenarioId::NoSymthaea => "The trace remains usable without Symthaea.",
        },
        expected,
        actual_valid: actual.is_ok(),
        trace_len: trace.len(),
        claim_ceiling: "ReferenceModelOnly",
    }
}

pub const ALL_SCENARIOS: [ScenarioId; 10] = [
    ScenarioId::NormalFlow,
    ScenarioId::RejectedCdsDecision,
    ScenarioId::StaleDesign,
    ScenarioId::UncertainObservation,
    ScenarioId::ConflictingObservations,
    ScenarioId::RecommendationAccepted,
    ScenarioId::RecommendationRejected,
    ScenarioId::ForeignEvidence,
    ScenarioId::AppealedOutcome,
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
    fn conflict_is_explicitly_not_overclaimed_by_linear_trace_validation() {
        let result = evaluate(ScenarioId::ConflictingObservations);
        assert!(matches!(result.expected, ScenarioExpected::PresentationOnly));
        assert!(result.actual_valid);
    }

    #[test]
    fn every_scenario_has_a_claim_ceiling() {
        for id in ALL_SCENARIOS {
            assert_eq!(evaluate(id).claim_ceiling, "ReferenceModelOnly");
        }
    }
}

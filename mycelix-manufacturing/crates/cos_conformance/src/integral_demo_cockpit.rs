//! D6A deterministic human-facing cockpit projection.
//!
//! This module defines what a future Integral cockpit is allowed to say from
//! the authoritative reference trace. It contains no generated prose and no
//! legitimacy, satisfaction, or flourishing score.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_trace::{ExplanationLevel, TraceActor, TraceEvent, TraceKind};
use crate::integral_demo_domain::SourceKind;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CockpitField {
    WhatHappened,
    WhoProducedIt,
    Evidence,
    Authority,
    Uncertainty,
    RecommendationStatus,
    Recovery,
    Challenge,
    Generation,
    Origin,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CockpitFact {
    pub field: CockpitField,
    pub event_id: &'static str,
    pub source_ref: &'static str,
    pub actor: TraceActor,
    pub kind: TraceKind,
    pub generation: u32,
    pub uncertainty_present: bool,
    pub authoritative: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CockpitView {
    pub trace_ref: &'static str,
    pub level: ExplanationLevel,
    pub facts: Vec<CockpitFact>,
    pub has_recommendation: bool,
    pub has_human_decision: bool,
    pub has_uncertainty: bool,
    pub has_foreign_origin: bool,
    pub has_recovery: bool,
    pub has_challenge_path: bool,
}

fn fact_for(event: &TraceEvent, field: CockpitField) -> CockpitFact {
    CockpitFact {
        field,
        event_id: event.event_id,
        source_ref: event.source_ref,
        actor: event.actor,
        kind: event.kind,
        generation: event.generation,
        uncertainty_present: event.uncertainty_present,
        // A cockpit fact describes authoritative trace data; the fact itself
        // never becomes a new authority-bearing artifact.
        authoritative: true,
    }
}

/// Project a validated trace into a deterministic cockpit view.
///
/// The caller must validate the trace first. The projection is deliberately
/// boring: it copies facts from the trace instead of synthesizing conclusions.
pub fn project_cockpit(
    trace_ref: &'static str,
    level: ExplanationLevel,
    events: &'static [TraceEvent],
) -> Option<CockpitView> {
    if trace_ref.is_empty() || events.is_empty() {
        return None;
    }

    let has_recommendation = events.iter().any(|e| e.kind == TraceKind::Recommendation);
    let has_human_decision = events.iter().any(|e| e.kind == TraceKind::HumanDecision);
    let has_uncertainty = events.iter().any(|e| e.uncertainty_present);
    let has_foreign_origin = events.iter().any(|e| matches!(e.source, SourceKind::Foreign));
    let has_recovery = events.iter().any(|e| e.recovery_ref.is_some());
    let has_challenge_path = events.iter().any(|e| e.challengeable);

    let fields = fields_for(level);
    let facts = events.iter().flat_map(|event| fields.iter().map(move |field| fact_for(event, *field))).collect();
    Some(CockpitView {
        trace_ref,
        level,
        facts,
        has_recommendation,
        has_human_decision,
        has_uncertainty,
        has_foreign_origin,
        has_recovery,
        has_challenge_path,
    })
}

/// A small deterministic selector used by the eventual UI to answer
/// "what should be visible at this disclosure level?"
pub fn fields_for(level: ExplanationLevel) -> &'static [CockpitField] {
    match level {
        ExplanationLevel::Summary => &[CockpitField::WhatHappened, CockpitField::Uncertainty],
        ExplanationLevel::Rationale => &[
            CockpitField::WhatHappened,
            CockpitField::WhoProducedIt,
            CockpitField::Evidence,
            CockpitField::Uncertainty,
        ],
        ExplanationLevel::Assurance => &[
            CockpitField::WhatHappened,
            CockpitField::WhoProducedIt,
            CockpitField::Evidence,
            CockpitField::Authority,
            CockpitField::Uncertainty,
            CockpitField::Recovery,
            CockpitField::Challenge,
            CockpitField::Generation,
            CockpitField::Origin,
        ],
        ExplanationLevel::Technical => &[
            CockpitField::WhatHappened,
            CockpitField::WhoProducedIt,
            CockpitField::Evidence,
            CockpitField::Authority,
            CockpitField::Uncertainty,
            CockpitField::RecommendationStatus,
            CockpitField::Recovery,
            CockpitField::Challenge,
            CockpitField::Generation,
            CockpitField::Origin,
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integral_demo_trace::{TraceActor, TraceKind, TraceStatus};

    const TRACE: [TraceEvent; 1] = [TraceEvent {
        event_id: "e1",
        sequence: 1,
        kind: TraceKind::Recommendation,
        provenance: crate::integral_demo_domain::ProvenanceClass::Recommendation,
        actor: TraceActor::Symthaea,
        source: crate::integral_demo_domain::SourceKind::Local,
        source_ref: "evidence://recommendation",
        generation: 7,
        uncertainty_present: true,
        authority_ref: None,
        reversible: true,
        challengeable: true,
        recommendation_only: true,
        recovery_ref: None,
        appeal_ref: None,
        decision_accepted: None,
        status: TraceStatus::Proposed,
        relations: &[],
    }];

    #[test]
    fn projection_is_descriptive_not_authoritative() {
        let view = project_cockpit("trace://demo", ExplanationLevel::Summary, &TRACE).expect("view");
        assert!(view.has_recommendation);
        assert!(view.has_uncertainty);
        assert_eq!(view.trace_ref, "trace://demo");
        assert!(view.facts.iter().all(|f| f.authoritative));
        assert!(view.facts.iter().any(|f| f.field == CockpitField::WhatHappened));
        assert!(view.facts.iter().any(|f| f.field == CockpitField::Uncertainty));
    }

    #[test]
    fn disclosure_is_progressive() {
        assert!(fields_for(ExplanationLevel::Technical).len() > fields_for(ExplanationLevel::Summary).len());
        assert!(fields_for(ExplanationLevel::Assurance).contains(&CockpitField::Authority));
        assert!(!fields_for(ExplanationLevel::Summary).contains(&CockpitField::Authority));
    }

    #[test]
    fn empty_trace_is_not_presented_as_a_result() {
        assert!(project_cockpit("trace://demo", ExplanationLevel::Summary, &[]).is_none());
        assert!(project_cockpit("", ExplanationLevel::Summary, &TRACE).is_none());
    }
}

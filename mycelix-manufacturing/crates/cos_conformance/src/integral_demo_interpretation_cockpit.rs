//! D6E interpretation laboratory projection for the human-facing cockpit.
//!
//! This layer connects alternative semantic hypotheses to the existing D5 trace
//! without fabricating trace events for the hypotheses themselves. A hypothesis
//! is an analysis of a fixture; the underlying trace remains the evidence-bearing
//! reference artifact.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_cockpit::{fields_for, CockpitField};
use crate::integral_demo_interpretations::{
    comparison_key, evaluate_fixture, Fixture, FixtureResult, InterfaceSeam, Interpretation,
    InterpretationKind,
};
use crate::integral_demo_trace::{ExplanationLevel, TraceEvent, TraceFixture};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterpretationCockpitRow {
    pub interpretation: Interpretation,
    pub result: FixtureResult,
    pub trace_event_count: usize,
    pub trace_relation_count: usize,
    pub trace_ref: Option<&'static str>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InterpretationCockpitView {
    pub seam: InterfaceSeam,
    pub fixture: Fixture,
    pub level: ExplanationLevel,
    pub rows: Vec<InterpretationCockpitRow>,
    pub visible_fields: &'static [CockpitField],
    pub preserves_alternatives: bool,
    pub claim_ceiling: &'static str,
}

/// Build a comparison view over the real D5 trace plus the interpretation
/// hypotheses. No hypothesis is projected into the trace as if it were evidence.
pub fn project_interpretation_cockpit(
    seam: InterfaceSeam,
    fixture: Fixture,
    level: ExplanationLevel,
    trace: Option<(&'static str, &TraceFixture)>,
) -> InterpretationCockpitView {
    let mut interpretations = crate::integral_demo_interpretations::interpretations_for(seam).to_vec();
    interpretations.sort_by_key(|i| comparison_key(*i));

    let (trace_ref, trace) = trace
        .map(|(reference, fixture)| (Some(reference), Some(fixture)))
        .unwrap_or((None, None));

    let rows = interpretations
        .into_iter()
        .map(|interpretation| InterpretationCockpitRow {
            result: evaluate_fixture(interpretation, fixture),
            interpretation,
            trace_event_count: trace.map_or(0, |t| t.events.len()),
            trace_relation_count: trace.map_or(0, |t| t.relations.len()),
            trace_ref,
        })
        .collect();

    InterpretationCockpitView {
        seam,
        fixture,
        level,
        rows,
        visible_fields: fields_for(level),
        // Alternatives remain explicit; ordering is serialization stability only.
        preserves_alternatives: true,
        claim_ceiling: "ReferenceModelOnly",
    }
}

/// Return the underlying trace event IDs without treating the interpretation
/// result as a new event or causal edge.
pub fn trace_event_ids(trace: &TraceFixture) -> Vec<&'static str> {
    trace.events.iter().map(|event| event.event_id).collect()
}

/// Human-readable labels for UI selectors. These labels describe hypotheses,
/// not maturity levels or rankings.
pub fn interpretation_label(kind: InterpretationKind) -> &'static str {
    match kind {
        InterpretationKind::MinimalFaithful => "Minimal / Faithful hypothesis",
        InterpretationKind::StrongSafety => "Strong-Safety hypothesis",
        InterpretationKind::FederationAware => "Federation-Aware hypothesis",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integral_demo_interpretations::OAD_COS_INTERPRETATIONS;

    fn trace() -> TraceFixture {
        TraceFixture {
            events: vec![
                TraceEvent {
                    event_id: "d1",
                    sequence: 1,
                    kind: crate::integral_demo_trace::TraceKind::Design,
                    provenance: crate::integral_demo_domain::ProvenanceClass::Design,
                    actor: crate::integral_demo_trace::TraceActor::System,
                    source: crate::integral_demo_domain::SourceKind::Local,
                    source_ref: "evidence://design",
                    evidence_ref: None,
                    generation: 7,
                    uncertainty_present: true,
                    authority_ref: None,
                    reversible: true,
                    challengeable: true,
                    recommendation_only: false,
                    recovery_ref: None,
                    appeal_ref: None,
                    decision_accepted: None,
                    status: crate::integral_demo_trace::TraceStatus::Accepted,
                },
            ],
            relations: vec![],
        }
    }

    #[test]
    fn cockpit_exposes_three_hypotheses_without_turning_them_into_trace_events() {
        let trace = trace();
        let view = project_interpretation_cockpit(
            InterfaceSeam::OadToCos,
            Fixture::SupersededDesign,
            ExplanationLevel::Rationale,
            Some(("trace://d6e-fixture", &trace)),
        );

        assert_eq!(view.rows.len(), 3);
        assert!(view.preserves_alternatives);
        assert!(view.rows.iter().all(|row| row.trace_event_count == 1));
        assert!(view.rows.iter().all(|row| row.trace_relation_count == 0));
        assert!(view.rows.iter().all(|row| row.trace_ref == Some("trace://d6e-fixture")));
        assert_eq!(view.rows[0].result.outcome, crate::integral_demo_interpretations::InterpretationOutcome::Admitted);
        assert_eq!(view.rows[1].result.outcome, crate::integral_demo_interpretations::InterpretationOutcome::Rejected);
    }

    #[test]
    fn interpretation_results_are_not_added_to_trace() {
        let trace = trace();
        let ids_before = trace_event_ids(&trace);
        let view = project_interpretation_cockpit(
            InterfaceSeam::OadToCos,
            Fixture::ForeignAuthority,
            ExplanationLevel::Technical,
            Some(("trace://d6e-fixture", &trace)),
        );
        let ids_after = trace_event_ids(&trace);

        assert_eq!(ids_before, ids_after);
        assert_eq!(view.rows.len(), OAD_COS_INTERPRETATIONS.len());
    }

    #[test]
    fn no_trace_context_is_explicit() {
        let view = project_interpretation_cockpit(
            InterfaceSeam::FrsToCds,
            Fixture::FrsRecommendation,
            ExplanationLevel::Summary,
            None,
        );
        assert!(view.rows.iter().all(|row| row.trace_event_count == 0));
        assert!(view.rows.iter().all(|row| row.trace_ref.is_none()));
    }

    #[test]
    fn labels_do_not_encode_a_ranking() {
        assert!(interpretation_label(InterpretationKind::MinimalFaithful).contains("hypothesis"));
        assert!(interpretation_label(InterpretationKind::StrongSafety).contains("hypothesis"));
        assert!(interpretation_label(InterpretationKind::FederationAware).contains("hypothesis"));
    }
}

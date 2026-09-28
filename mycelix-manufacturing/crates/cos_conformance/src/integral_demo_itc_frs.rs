//! D4 ITC projection -> FRS feedback reference seam.
//!
//! FRS consumes source-bound accounting projections as feedback inputs. It
//! does not turn findings or recommendations into governance authority.
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_cos_itc::{ItcProjection, ObservationBinding};
use crate::integral_demo_domain::{DemoDecision, ProvenanceClass, SourceKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrsFinding {
    pub finding_id: &'static str,
    pub provenance: ProvenanceClass,
    pub source_projection_id: &'static str,
    pub source_observation_id: &'static str,
    pub source: SourceKind,
    pub uncertainty_present: bool,
    pub observed_quantity: u32,
    pub finding_basis: &'static str,
    pub observation_binding: ObservationBinding,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrsRecommendation {
    pub recommendation_id: &'static str,
    pub finding_id: &'static str,
    pub rationale: &'static str,
    pub source: SourceKind,
    pub uncertainty_present: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrsDecision {
    Finding(FrsFinding),
    RejectedWrongProvenance,
    RejectedEmptyBasis,
    RejectedSourceMutation,
}

pub fn project_itc_to_frs(
    projection: Option<&ItcProjection>,
    finding_id: &'static str,
    finding_basis: &'static str,
) -> FrsDecision {
    let Some(projection) = projection else {
        return FrsDecision::RejectedWrongProvenance;
    };

    if projection.source_observation_id.is_empty()
        || projection.projection_id.is_empty()
        || finding_id.is_empty()
        || finding_basis.is_empty()
    {
        return FrsDecision::RejectedEmptyBasis;
    }

    if projection.observation_binding.observation_id != projection.source_observation_id {
        return FrsDecision::RejectedSourceMutation;
    }

    FrsDecision::Finding(FrsFinding {
        finding_id,
        provenance: ProvenanceClass::Assessment,
        source_projection_id: projection.projection_id,
        source_observation_id: projection.source_observation_id,
        source: projection.source,
        uncertainty_present: projection.uncertainty_present,
        observed_quantity: projection.projected_contribution,
        finding_basis,
        observation_binding: projection.observation_binding,
    })
}

pub fn finding_to_recommendation(
    finding: &FrsFinding,
    recommendation_id: &'static str,
    rationale: &'static str,
) -> Option<FrsRecommendation> {
    if finding.provenance != ProvenanceClass::Assessment
        || recommendation_id.is_empty()
        || rationale.is_empty()
    {
        return None;
    }

    Some(FrsRecommendation {
        recommendation_id,
        finding_id: finding.finding_id,
        rationale,
        source: finding.source,
        uncertainty_present: finding.uncertainty_present,
    })
}

/// A finding or recommendation is never itself a CDS decision or authorization.
pub fn feedback_does_not_become_governance_authority() -> bool {
    ProvenanceClass::Assessment != ProvenanceClass::Decision
        && ProvenanceClass::Recommendation != ProvenanceClass::Decision
        && ProvenanceClass::Recommendation != ProvenanceClass::Authorization
}

/// The CDS decision remains an explicit human/community artifact; FRS feedback
/// may inform it but cannot manufacture the decision.
pub fn recommendation_requires_separate_decision(
    recommendation: &FrsRecommendation,
    decision: &DemoDecision,
) -> bool {
    !recommendation.recommendation_id.is_empty()
        && matches!(decision, DemoDecision::Accepted | DemoDecision::Rejected | DemoDecision::Draft)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn projection() -> ItcProjection {
        ItcProjection {
            projection_id: "projection-001",
            source_observation_id: "obs-001",
            projected_contribution: 12,
            accounting_basis: "demo-basis-v1",
            uncertainty_present: true,
            source: SourceKind::Local,
            observation_binding: ObservationBinding {
                observation_id: "obs-001",
                work_id: "work-001",
                actor_id: "actor-001",
                design_generation: 7,
                observed_quantity: 12,
                observed_at: 1_000,
                evidence_ref: "evidence-001",
                source: SourceKind::Local,
                uncertainty_present: true,
            },
        }
    }

    #[test]
    fn projection_becomes_assessment_not_decision() {
        let result = project_itc_to_frs(Some(&projection()), "finding-001", "quantity review");
        let FrsDecision::Finding(finding) = result else {
            panic!("expected finding");
        };
        assert_eq!(finding.provenance, ProvenanceClass::Assessment);
        assert_eq!(finding.source_observation_id, "obs-001");
        assert!(finding.uncertainty_present);
    }

    #[test]
    fn source_binding_cannot_be_rewritten() {
        let mut value = projection();
        value.observation_binding.observation_id = "obs-evil";
        assert_eq!(
            project_itc_to_frs(Some(&value), "finding-001", "quantity review"),
            FrsDecision::RejectedSourceMutation
        );
    }

    #[test]
    fn foreign_origin_survives_feedback() {
        let mut value = projection();
        value.source = SourceKind::Foreign;
        value.observation_binding.source = SourceKind::Foreign;
        let FrsDecision::Finding(finding) =
            project_itc_to_frs(Some(&value), "finding-001", "foreign observation review")
        else {
            panic!("expected finding");
        };
        assert_eq!(finding.source, SourceKind::Foreign);
        assert_eq!(finding.observation_binding.source, SourceKind::Foreign);
    }

    #[test]
    fn recommendation_preserves_uncertainty_and_finding_lineage() {
        let FrsDecision::Finding(finding) =
            project_itc_to_frs(Some(&projection()), "finding-001", "quantity review")
        else {
            panic!("expected finding");
        };
        let recommendation =
            finding_to_recommendation(&finding, "recommendation-001", "request human review")
                .expect("valid recommendation");
        assert_eq!(recommendation.finding_id, "finding-001");
        assert!(recommendation.uncertainty_present);
    }

    #[test]
    fn recommendation_cannot_become_governance_authority() {
        assert!(feedback_does_not_become_governance_authority());
    }

    #[test]
    fn recommendation_needs_separate_human_decision_artifact() {
        let recommendation = FrsRecommendation {
            recommendation_id: "recommendation-001",
            finding_id: "finding-001",
            rationale: "request review",
            source: SourceKind::Local,
            uncertainty_present: true,
        };
        assert!(recommendation_requires_separate_decision(
            &recommendation,
            &DemoDecision::Accepted
        ));
    }

    #[test]
    fn empty_finding_basis_is_rejected() {
        assert_eq!(
            project_itc_to_frs(Some(&projection()), "finding-001", ""),
            FrsDecision::RejectedEmptyBasis
        );
    }
}

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
    pub source_generation: u32,
    pub assessed_at: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrsRecommendation {
    pub recommendation_id: &'static str,
    pub finding_id: &'static str,
    pub rationale: &'static str,
    pub source: SourceKind,
    pub uncertainty_present: bool,
    pub finding_generation: u32,
    pub review_before: Option<u64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrsConflict {
    pub left_observation_id: &'static str,
    pub right_observation_id: &'static str,
    pub left_quantity: u32,
    pub right_quantity: u32,
    pub same_work_id: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrsDecision {
    Finding(FrsFinding),
    RejectedWrongProvenance,
    RejectedEmptyBasis,
    RejectedSourceMutation,
    RejectedStaleProjection,
    RejectedFutureAssessmentTime,
}

pub fn project_itc_to_frs(
    projection: Option<&ItcProjection>,
    finding_id: &'static str,
    finding_basis: &'static str,
    current_generation: u32,
    assessed_at: u64,
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

    if projection.observation_binding.observation_id != projection.source_observation_id
        || projection.observation_binding.work_id.is_empty()
        || projection.observation_binding.actor_id.is_empty()
        || projection.observation_binding.evidence_ref.is_empty()
        || projection.observation_binding.source != projection.source
        || projection.observation_binding.uncertainty_present != projection.uncertainty_present
        || projection.observation_binding.observed_quantity != projection.projected_contribution
    {
        return FrsDecision::RejectedSourceMutation;
    }

    if projection.observation_binding.design_generation != current_generation {
        return FrsDecision::RejectedStaleProjection;
    }

    if assessed_at < projection.observation_binding.observed_at {
        return FrsDecision::RejectedFutureAssessmentTime;
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
        source_generation: projection.observation_binding.design_generation,
        assessed_at,
    })
}

pub fn finding_to_recommendation(
    finding: &FrsFinding,
    recommendation_id: &'static str,
    rationale: &'static str,
    review_before: Option<u64>,
) -> Option<FrsRecommendation> {
    if finding.provenance != ProvenanceClass::Assessment
        || recommendation_id.is_empty()
        || rationale.is_empty()
        || review_before.is_some_and(|deadline| deadline < finding.assessed_at)
    {
        return None;
    }

    Some(FrsRecommendation {
        recommendation_id,
        finding_id: finding.finding_id,
        rationale,
        source: finding.source,
        uncertainty_present: finding.uncertainty_present,
        finding_generation: finding.source_generation,
        review_before,
    })
}

/// Two source observations can disagree without one being rewritten into the other.
pub fn preserve_conflicting_observations(
    left: &ObservationBinding,
    right: &ObservationBinding,
) -> Option<FrsConflict> {
    if left.work_id != right.work_id
        || left.observation_id == right.observation_id
        || left.observed_quantity == right.observed_quantity
    {
        return None;
    }

    Some(FrsConflict {
        left_observation_id: left.observation_id,
        right_observation_id: right.observation_id,
        left_quantity: left.observed_quantity,
        right_quantity: right.observed_quantity,
        same_work_id: left.work_id,
    })
}

/// A federation conflict can enter FRS as a dispute without becoming a decision.
/// No quantity is selected by this seam.
pub fn federation_conflict_requires_explicit_governance_resolution(
    conflict: &FrsConflict,
    decision: Option<&DemoDecision>,
) -> bool {
    if conflict.left_observation_id.is_empty()
        || conflict.right_observation_id.is_empty()
        || conflict.left_observation_id == conflict.right_observation_id
        || conflict.left_quantity == conflict.right_quantity
    {
        return false;
    }

    matches!(decision, Some(DemoDecision::Accepted) | Some(DemoDecision::Rejected))
}

/// A finding or recommendation is never itself a CDS decision or authorization.
pub fn feedback_does_not_become_governance_authority() -> bool {
    ProvenanceClass::Assessment != ProvenanceClass::Decision
        && ProvenanceClass::Recommendation != ProvenanceClass::Decision
        && ProvenanceClass::Recommendation != ProvenanceClass::Authorization
}

/// The CDS decision remains a separate human/community artifact; FRS feedback
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

    fn finding() -> FrsFinding {
        let FrsDecision::Finding(f) =
            project_itc_to_frs(Some(&projection()), "finding-001", "quantity review", 7, 1_100)
        else {
            panic!("expected finding");
        };
        f
    }

    #[test]
    fn projection_becomes_assessment_not_decision() {
        let f = finding();
        assert_eq!(f.provenance, ProvenanceClass::Assessment);
        assert_eq!(f.source_observation_id, "obs-001");
        assert!(f.uncertainty_present);
    }

    #[test]
    fn complete_source_binding_is_checked() {
        let mut value = projection();
        value.observation_binding.observed_quantity = 11;
        assert_eq!(
            project_itc_to_frs(Some(&value), "finding-001", "quantity review", 7, 1_100),
            FrsDecision::RejectedSourceMutation
        );
    }

    #[test]
    fn foreign_origin_survives_feedback() {
        let mut value = projection();
        value.source = SourceKind::Foreign;
        value.observation_binding.source = SourceKind::Foreign;
        let FrsDecision::Finding(f) =
            project_itc_to_frs(Some(&value), "finding-001", "foreign review", 7, 1_100)
        else {
            panic!("expected finding");
        };
        assert_eq!(f.source, SourceKind::Foreign);
        assert_eq!(f.observation_binding.source, SourceKind::Foreign);
    }

    #[test]
    fn stale_generation_cannot_become_current_feedback() {
        assert_eq!(
            project_itc_to_frs(Some(&projection()), "finding-001", "review", 8, 1_100),
            FrsDecision::RejectedStaleProjection
        );
    }

    #[test]
    fn assessment_cannot_predate_observation() {
        assert_eq!(
            project_itc_to_frs(Some(&projection()), "finding-001", "review", 7, 999),
            FrsDecision::RejectedFutureAssessmentTime
        );
    }

    #[test]
    fn recommendation_preserves_uncertainty_and_lineage() {
        let recommendation =
            finding_to_recommendation(&finding(), "recommendation-001", "request human review", Some(2_000))
                .expect("valid recommendation");
        assert_eq!(recommendation.finding_id, "finding-001");
        assert_eq!(recommendation.finding_generation, 7);
        assert!(recommendation.uncertainty_present);
        assert_eq!(recommendation.review_before, Some(2_000));
    }

    #[test]
    fn invalid_review_window_is_rejected() {
        assert!(finding_to_recommendation(&finding(), "recommendation-001", "review", Some(1_099)).is_none());
    }

    #[test]
    fn conflicting_observations_remain_distinguishable() {
        let left = projection().observation_binding;
        let mut right = left;
        right.observation_id = "obs-002";
        right.observed_quantity = 14;
        let conflict = preserve_conflicting_observations(&left, &right).expect("conflict preserved");
        assert_eq!(conflict.left_observation_id, "obs-001");
        assert_eq!(conflict.right_observation_id, "obs-002");
        assert_eq!(conflict.left_quantity, 12);
        assert_eq!(conflict.right_quantity, 14);
    }

    #[test]
    fn federation_conflict_stays_unresolved_without_explicit_decision() {
        let conflict = FrsConflict {
            left_observation_id: "obs-a",
            right_observation_id: "obs-b",
            left_quantity: 10,
            right_quantity: 12,
            same_work_id: "work-1",
        };
        assert!(!federation_conflict_requires_explicit_governance_resolution(
            &conflict,
            Some(&DemoDecision::Draft)
        ));
        assert!(federation_conflict_requires_explicit_governance_resolution(
            &conflict,
            Some(&DemoDecision::Accepted)
        ));
    }

    #[test]
    fn recommendation_cannot_become_governance_authority() {
        assert!(feedback_does_not_become_governance_authority());
    }

    #[test]
    fn recommendation_needs_separate_decision_artifact() {
        let recommendation = FrsRecommendation {
            recommendation_id: "recommendation-001",
            finding_id: "finding-001",
            rationale: "request review",
            source: SourceKind::Local,
            uncertainty_present: true,
            finding_generation: 7,
            review_before: Some(2_000),
        };
        assert!(recommendation_requires_separate_decision(&recommendation, &DemoDecision::Accepted));
    }

    #[test]
    fn empty_finding_basis_is_rejected() {
        assert_eq!(
            project_itc_to_frs(Some(&projection()), "finding-001", "", 7, 1_100),
            FrsDecision::RejectedEmptyBasis
        );
    }
}

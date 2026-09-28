//! D3 COS observation -> ITC contribution projection reference seam.
//!
//! The projection deliberately consumes an observed contribution event, not
//! COS admission, authorization, recommendation, or execution intent.
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_domain::{ProvenanceClass, SourceKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CosObservation {
    pub observation_id: &'static str,
    pub work_id: &'static str,
    pub actor_id: &'static str,
    pub design_generation: u32,
    pub observed_quantity: u32,
    pub observed_at: u64,
    pub evidence_ref: &'static str,
    pub source: SourceKind,
    pub uncertainty_present: bool,
    pub source_class: ProvenanceClass,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObservationBinding {
    pub observation_id: &'static str,
    pub work_id: &'static str,
    pub actor_id: &'static str,
    pub design_generation: u32,
    pub observed_quantity: u32,
    pub observed_at: u64,
    pub evidence_ref: &'static str,
    pub source: SourceKind,
    pub uncertainty_present: bool,
}

impl From<&CosObservation> for ObservationBinding {
    fn from(observation: &CosObservation) -> Self {
        Self {
            observation_id: observation.observation_id,
            work_id: observation.work_id,
            actor_id: observation.actor_id,
            design_generation: observation.design_generation,
            observed_quantity: observation.observed_quantity,
            observed_at: observation.observed_at,
            evidence_ref: observation.evidence_ref,
            source: observation.source,
            uncertainty_present: observation.uncertainty_present,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ItcProjection {
    pub projection_id: &'static str,
    pub source_observation_id: &'static str,
    pub projected_contribution: u32,
    pub accounting_basis: &'static str,
    pub uncertainty_present: bool,
    pub source: SourceKind,
    /// Immutable source binding: a repeated logical observation may only
    /// reproduce the same projection payload.
    pub observation_binding: ObservationBinding,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProjectionDecision {
    Projected(ItcProjection),
    Replayed(ItcProjection),
    RejectedMissingObservation,
    RejectedWrongProvenance,
    RejectedStaleObservation,
    RejectedEmptyEvidence,
    RejectedInvalidIdentity,
    RejectedDuplicatePayloadMutation,
}

pub fn project_observation_to_itc(
    observation: Option<&CosObservation>,
    expected_generation: u32,
    accounting_basis: &'static str,
) -> ProjectionDecision {
    let Some(observation) = observation else {
        return ProjectionDecision::RejectedMissingObservation;
    };

    if observation.source_class != ProvenanceClass::Observation {
        return ProjectionDecision::RejectedWrongProvenance;
    }
    if observation.design_generation != expected_generation {
        return ProjectionDecision::RejectedStaleObservation;
    }
    if observation.observation_id.is_empty()
        || observation.work_id.is_empty()
        || observation.actor_id.is_empty()
        || accounting_basis.is_empty()
    {
        return ProjectionDecision::RejectedInvalidIdentity;
    }
    if observation.evidence_ref.is_empty() {
        return ProjectionDecision::RejectedEmptyEvidence;
    }

    let binding = ObservationBinding::from(observation);

    ProjectionDecision::Projected(ItcProjection {
        projection_id: observation.observation_id,
        source_observation_id: observation.observation_id,
        projected_contribution: observation.observed_quantity,
        accounting_basis,
        uncertainty_present: observation.uncertainty_present,
        source: observation.source,
        observation_binding: binding,
    })
}

/// Replaying the same logical observation is idempotent. Reusing its logical
/// identity with a changed source payload is fail-closed rather than an
/// overwrite. This models the seam needed for retries/reconciliation.
pub fn reconcile_existing_projection(
    existing: Option<&ItcProjection>,
    observation: &CosObservation,
    expected_generation: u32,
    accounting_basis: &'static str,
) -> ProjectionDecision {
    let Some(existing) = existing else {
        return project_observation_to_itc(
            Some(observation),
            expected_generation,
            accounting_basis,
        );
    };

    let candidate = match project_observation_to_itc(
        Some(observation),
        expected_generation,
        accounting_basis,
    ) {
        ProjectionDecision::Projected(value) => value,
        other => return other,
    };

    if existing.projection_id != candidate.projection_id
        || existing.source_observation_id != candidate.source_observation_id
    {
        return ProjectionDecision::RejectedDuplicatePayloadMutation;
    }

    if existing == &candidate {
        ProjectionDecision::Replayed(candidate)
    } else {
        ProjectionDecision::RejectedDuplicatePayloadMutation
    }
}

/// A semantic admission or authorization is intentionally insufficient to
/// create an accounting projection. The ITC seam requires a source observation.
pub fn admission_event_is_not_observation() -> bool {
    ProvenanceClass::Observation != ProvenanceClass::Authorization
        && ProvenanceClass::Observation != ProvenanceClass::Decision
        && ProvenanceClass::Observation != ProvenanceClass::Recommendation
        && ProvenanceClass::Observation != ProvenanceClass::ExecutionIntent
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation() -> CosObservation {
        CosObservation {
            observation_id: "obs-001",
            work_id: "work-001",
            actor_id: "actor-001",
            design_generation: 7,
            observed_quantity: 12,
            observed_at: 1_000,
            evidence_ref: "evidence-001",
            source: SourceKind::Local,
            uncertainty_present: true,
            source_class: ProvenanceClass::Observation,
        }
    }

    fn projection() -> ItcProjection {
        let value = observation();
        let ProjectionDecision::Projected(p) =
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1")
        else {
            panic!("expected projection");
        };
        p
    }

    #[test]
    fn valid_observation_projects_without_minting_an_observation() {
        let result = project_observation_to_itc(Some(&observation()), 7, "demo-basis-v1");
        let ProjectionDecision::Projected(p) = result else {
            panic!("expected projection");
        };
        assert_eq!(p.source_observation_id, "obs-001");
        assert_eq!(p.projected_contribution, 12);
        assert!(p.uncertainty_present);
        assert_eq!(p.observation_binding.work_id, "work-001");
        assert_eq!(p.observation_binding.actor_id, "actor-001");
    }

    #[test]
    fn same_logical_observation_is_idempotent() {
        let value = observation();
        let existing = projection();
        assert_eq!(
            reconcile_existing_projection(
                Some(&existing),
                &value,
                7,
                "demo-basis-v1"
            ),
            ProjectionDecision::Replayed(existing)
        );
    }

    #[test]
    fn changed_quantity_cannot_mutate_existing_projection() {
        let existing = projection();
        let mut value = observation();
        value.observed_quantity = 13;
        assert_eq!(
            reconcile_existing_projection(
                Some(&existing),
                &value,
                7,
                "demo-basis-v1"
            ),
            ProjectionDecision::RejectedDuplicatePayloadMutation
        );
    }

    #[test]
    fn changed_evidence_cannot_mutate_existing_projection() {
        let existing = projection();
        let mut value = observation();
        value.evidence_ref = "evidence-002";
        assert_eq!(
            reconcile_existing_projection(
                Some(&existing),
                &value,
                7,
                "demo-basis-v1"
            ),
            ProjectionDecision::RejectedDuplicatePayloadMutation
        );
    }

    #[test]
    fn changed_actor_cannot_mutate_existing_projection() {
        let existing = projection();
        let mut value = observation();
        value.actor_id = "actor-002";
        assert_eq!(
            reconcile_existing_projection(
                Some(&existing),
                &value,
                7,
                "demo-basis-v1"
            ),
            ProjectionDecision::RejectedDuplicatePayloadMutation
        );
    }

    #[test]
    fn missing_observation_blocks_projection() {
        assert_eq!(
            project_observation_to_itc(None, 7, "demo-basis-v1"),
            ProjectionDecision::RejectedMissingObservation
        );
    }

    #[test]
    fn authorization_cannot_be_used_as_observation() {
        let mut value = observation();
        value.source_class = ProvenanceClass::Authorization;
        assert_eq!(
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1"),
            ProjectionDecision::RejectedWrongProvenance
        );
    }

    #[test]
    fn admission_cannot_be_used_as_observation() {
        let mut value = observation();
        value.source_class = ProvenanceClass::ExecutionIntent;
        assert_eq!(
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1"),
            ProjectionDecision::RejectedWrongProvenance
        );
    }

    #[test]
    fn stale_observation_is_rejected() {
        let mut value = observation();
        value.design_generation = 6;
        assert_eq!(
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1"),
            ProjectionDecision::RejectedStaleObservation
        );
    }

    #[test]
    fn evidence_reference_is_required() {
        let mut value = observation();
        value.evidence_ref = "";
        assert_eq!(
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1"),
            ProjectionDecision::RejectedEmptyEvidence
        );
    }

    #[test]
    fn invalid_identity_is_rejected() {
        let mut value = observation();
        value.work_id = "";
        assert_eq!(
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1"),
            ProjectionDecision::RejectedInvalidIdentity
        );
    }

    #[test]
    fn empty_accounting_basis_is_rejected() {
        assert_eq!(
            project_observation_to_itc(Some(&observation()), 7, ""),
            ProjectionDecision::RejectedInvalidIdentity
        );
    }

    #[test]
    fn foreign_origin_is_preserved() {
        let mut value = observation();
        value.source = SourceKind::Foreign;
        let ProjectionDecision::Projected(p) =
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1")
        else {
            panic!("expected projection");
        };
        assert_eq!(p.source, SourceKind::Foreign);
        assert_eq!(p.observation_binding.source, SourceKind::Foreign);
    }

    #[test]
    fn uncertainty_is_preserved() {
        let mut value = observation();
        value.uncertainty_present = true;
        let ProjectionDecision::Projected(p) =
            project_observation_to_itc(Some(&value), 7, "demo-basis-v1")
        else {
            panic!("expected projection");
        };
        assert!(p.uncertainty_present);
        assert!(p.observation_binding.uncertainty_present);
    }

    #[test]
    fn provenance_classes_keep_admission_and_observation_distinct() {
        assert!(admission_event_is_not_observation());
    }
}

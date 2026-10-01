use serde::{Deserialize, Serialize};

use crate::identity_lineage::{ApplicabilityInterval, IdentityKind, IdentityRef, LineageEdge, LineageRelation};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceEventInterval {
    pub start: u64,
    pub end: Option<u64>,
}

impl EvidenceEventInterval {
    pub fn validate(&self) -> Result<(), String> {
        ApplicabilityInterval { start: self.start, end: self.end }
            .validate()
            .map_err(|e| format!("invalid evidence event interval: {e}"))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalEvidenceApplicability {
    pub evidence: IdentityRef,
    pub configuration: IdentityRef,
    pub artifact: IdentityRef,
    pub evidence_target: LineageEdge,
    pub configuration_applicability: LineageEdge,
    pub event_interval: EvidenceEventInterval,
    pub effectivity_interval: ApplicabilityInterval,
}

impl TemporalEvidenceApplicability {
    pub fn validate(&self) -> Result<(), String> {
        self.evidence.validate()?;
        self.configuration.validate()?;
        self.artifact.validate()?;
        self.evidence_target.validate()?;
        self.configuration_applicability.validate()?;
        self.event_interval.validate()?;
        self.effectivity_interval.validate()?;

        if self.configuration.kind != IdentityKind::ConfigurationRevision {
            return Err("temporal evidence applicability requires a configuration revision".into());
        }
        if self.artifact.kind != IdentityKind::PhysicalArtifact {
            return Err("temporal evidence applicability requires a physical artifact".into());
        }

        let expected_relation = match self.evidence.kind {
            IdentityKind::InspectionRecord => LineageRelation::InspectedAs,
            IdentityKind::TestRecord => LineageRelation::TestedAs,
            IdentityKind::OperationalObservation => LineageRelation::ObservedAs,
            IdentityKind::MaintenanceEvent => LineageRelation::MaintainedAs,
            _ => return Err("temporal evidence applicability requires an inspection, test, observation, or maintenance event".into()),
        };

        if self.evidence_target.relation != expected_relation
            || self.evidence_target.source != self.evidence
            || self.evidence_target.target != self.artifact
        {
            return Err("evidence target must exactly bind the typed evidence event to the physical artifact".into());
        }

        if self.configuration_applicability.relation != LineageRelation::AppliesTo
            || self.configuration_applicability.source != self.configuration
            || self.configuration_applicability.target != self.artifact
        {
            return Err("configuration applicability must exactly bind the configuration revision to the physical artifact".into());
        }

        Ok(())
    }

    pub fn temporal_overlap(&self) -> bool {
        let event = ApplicabilityInterval {
            start: self.event_interval.start,
            end: self.event_interval.end,
        };
        event.overlaps(&self.effectivity_interval)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(kind: IdentityKind, value: &str) -> IdentityRef {
        IdentityRef { kind, namespace: "mobility".into(), id: value.into() }
    }

    fn applies_to(configuration: &IdentityRef, artifact: &IdentityRef) -> LineageEdge {
        LineageEdge {
            relation: LineageRelation::AppliesTo,
            source: configuration.clone(),
            target: artifact.clone(),
        }
    }

    fn evidence_case() -> TemporalEvidenceApplicability {
        let evidence = id(IdentityKind::InspectionRecord, "inspection-1");
        let configuration = id(IdentityKind::ConfigurationRevision, "cfg-1");
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-1");

        TemporalEvidenceApplicability {
            evidence: evidence.clone(),
            configuration: configuration.clone(),
            artifact: artifact.clone(),
            evidence_target: LineageEdge {
                relation: LineageRelation::InspectedAs,
                source: evidence,
                target: artifact.clone(),
            },
            configuration_applicability: applies_to(&configuration, &artifact),
            event_interval: EvidenceEventInterval { start: 100, end: Some(110) },
            effectivity_interval: ApplicabilityInterval { start: 120, end: Some(200) },
        }
    }

    #[test]
    fn event_time_and_effectivity_can_be_disjoint() {
        let value = evidence_case();
        assert!(value.validate().is_ok());
        assert!(!value.temporal_overlap());
    }

    #[test]
    fn effectivity_uses_half_open_bounds() {
        let value = evidence_case();
        assert!(value.effectivity_interval.contains(120));
        assert!(value.effectivity_interval.contains(199));
        assert!(!value.effectivity_interval.contains(200));
    }

    #[test]
    fn open_ended_effectivity_is_explicit() {
        let mut value = evidence_case();
        value.effectivity_interval.end = None;
        assert!(value.validate().is_ok());
    }

    #[test]
    fn zero_length_effectivity_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval.end = Some(value.effectivity_interval.start);
        assert!(value.validate().is_err());
    }

    #[test]
    fn wrong_artifact_binding_is_rejected() {
        let mut value = evidence_case();
        value.artifact = id(IdentityKind::PhysicalArtifact, "artifact-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn wrong_configuration_binding_is_rejected() {
        let mut value = evidence_case();
        value.configuration = id(IdentityKind::ConfigurationRevision, "cfg-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn evidence_does_not_inherit_to_a_replacement_artifact() {
        let mut value = evidence_case();
        value.evidence_target.target = id(IdentityKind::PhysicalArtifact, "artifact-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn evidence_does_not_inherit_to_a_successor_configuration() {
        let mut value = evidence_case();
        value.configuration_applicability.source = id(IdentityKind::ConfigurationRevision, "cfg-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn holochain_shaped_artifact_identity_is_rejected() {
        let mut value = evidence_case();
        value.artifact.namespace = "holochain".into();
        assert!(value.validate().is_err());
    }

    #[test]
    fn later_effectivity_does_not_erase_historical_event_time() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 1_000, end: Some(2_000) };
        assert!(value.validate().is_ok());
        assert_eq!(value.event_interval.start, 100);
    }

    #[test]
    fn wrong_evidence_kind_is_rejected() {
        let mut value = evidence_case();
        value.evidence = id(IdentityKind::DesignRevision, "design-1");
        assert!(value.validate().is_err());
    }
}

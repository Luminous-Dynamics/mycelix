use serde::{Deserialize, Serialize};

use crate::identity_lineage::{ApplicabilityInterval, IdentityKind, IdentityRef, LineageEdge, LineageRelation, TemporalConfigurationApplicability};

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

/// Current documented epistemic disposition of an evidence record.
///
/// This is deliberately separate from event/effectivity time. A disposition
/// never rewrites or erases the historical event or applicability interval.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceDisposition {
    Active,
    Superseded { by: IdentityRef },
    Disputed { by: IdentityRef },
    Retracted { by: IdentityRef },
    Unresolved,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDispositionTransition {
    /// Evidence record whose epistemic state is changing.
    pub evidence: IdentityRef,
    /// Explicit predecessor transition. None is permitted only for the initial
    /// disposition assertion; subsequent transitions must point to exactly one
    /// prior transition record.
    pub predecessor: Option<IdentityRef>,
    pub from: EvidenceDisposition,
    pub to: EvidenceDisposition,
    /// Addressable witness for why the transition is being asserted.
    pub basis: IdentityRef,
}

impl EvidenceDispositionTransition {
    pub fn validate(&self) -> Result<(), String> {
        self.evidence.validate()?;
        if !matches!(
            self.evidence.kind,
            IdentityKind::InspectionRecord
                | IdentityKind::TestRecord
                | IdentityKind::OperationalObservation
                | IdentityKind::MaintenanceEvent
        ) {
            return Err("disposition transition requires a typed evidence event".into());
        }

        if let Some(predecessor) = &self.predecessor {
            predecessor.validate()?;
            if predecessor == &self.evidence {
                return Err("disposition predecessor cannot be the evidence record itself".into());
            }
            if !matches!(
                predecessor.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
                return Err("disposition predecessor must be an EvidenceRecord or ReconciliationWitness".into());
            }
        }

        self.basis.validate()?;
        if self.basis == self.evidence {
            return Err("disposition basis cannot be the evidence record itself".into());
        }
        if !matches!(
            self.basis.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err("disposition basis must be an EvidenceRecord or ReconciliationWitness".into());
        }

        self.from.validate(&self.evidence)?;
        self.to.validate(&self.evidence)?;

        if self.from == self.to {
            return Err("disposition transition must change epistemic state".into());
        }

        if self.predecessor.is_none() && self.from != EvidenceDisposition::Active {
            return Err("genesis disposition must start from Active".into());
        }

        match (&self.from, &self.to) {
            (EvidenceDisposition::Active, EvidenceDisposition::Disputed { .. })
            | (EvidenceDisposition::Active, EvidenceDisposition::Superseded { .. })
            | (EvidenceDisposition::Active, EvidenceDisposition::Retracted { .. })
            | (EvidenceDisposition::Active, EvidenceDisposition::Unresolved)
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Active)
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Superseded { .. })
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Retracted { .. })
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Unresolved)
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Active)
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Disputed { .. })
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Superseded { .. })
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Retracted { .. }) => Ok(()),
            (EvidenceDisposition::Superseded { .. }, _)
            | (EvidenceDisposition::Retracted { .. }, _) => {
                Err("superseded and retracted dispositions are terminal".into())
            }
            _ => Err("unsupported evidence disposition transition".into()),
        }
    }
}

impl EvidenceDisposition {
    pub fn validate(&self, evidence: &IdentityRef) -> Result<(), String> {
        let witness = match self {
            Self::Active | Self::Unresolved => return Ok(()),
            Self::Superseded { by } | Self::Disputed { by } | Self::Retracted { by } => by,
        };
        witness.validate()?;
        if witness == evidence {
            return Err("evidence disposition witness cannot be the evidence record itself".into());
        }
        if !matches!(witness.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
            return Err("evidence disposition witness must be an EvidenceRecord or ReconciliationWitness".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalEvidenceApplicability {
    pub evidence: IdentityRef,
    pub configuration: IdentityRef,
    pub artifact: IdentityRef,
    pub evidence_target: LineageEdge,
    /// Explicit temporal witness for the exact configuration-to-artifact applicability.
    /// The evidence effectivity interval must be contained by this witness; the
    /// underlying lineage edge is never treated as timeless applicability.
    pub configuration_applicability: TemporalConfigurationApplicability,
    pub event_interval: EvidenceEventInterval,
    pub effectivity_interval: ApplicabilityInterval,
    /// Epistemic status is orthogonal to temporal scope and does not rewrite it.
    pub disposition: EvidenceDisposition,
}

impl TemporalEvidenceApplicability {
    pub fn validate(&self) -> Result<(), String> {
        self.evidence.validate()?;
        self.configuration.validate()?;
        self.artifact.validate()?;
        self.evidence_target.validate()?;
        self.configuration_applicability.validate()?;
        self.event_interval.validate()?
            .and_then(|_| self.validate_effectivity_containment())?;
        self.effectivity_interval.validate()?;
        self.disposition.validate(&self.evidence)?;

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

        if self.configuration_applicability.configuration != self.configuration
            || self.configuration_applicability.artifact != self.artifact
        {
            return Err("temporal configuration applicability must exactly bind the declared configuration and physical artifact".into());
        }

        Ok(())
    }

    /// Enforce scope containment without using event/effectivity overlap as a
    /// validity rule. Half-open intervals mean equal bounded ends are allowed;
    /// a finite configuration interval cannot contain an open-ended effectivity.
    fn validate_effectivity_containment(&self) -> Result<(), String> {
        let config = &self.configuration_applicability.interval;
        let effectivity = &self.effectivity_interval;

        if effectivity.start < config.start {
            return Err("evidence effectivity starts before configuration applicability".into());
        }

        match (config.end, effectivity.end) {
            (Some(config_end), Some(effectivity_end)) if effectivity_end > config_end => {
                Err("evidence effectivity extends beyond configuration applicability".into())
            }
            (Some(_), None) => {
                Err("open-ended evidence effectivity requires open-ended configuration applicability".into())
            }
            _ => Ok(()),
        }
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
            configuration_applicability: TemporalConfigurationApplicability {
                configuration: configuration.clone(),
                artifact: artifact.clone(),
                applicability: applies_to(&configuration, &artifact),
                interval: ApplicabilityInterval { start: 90, end: Some(300) },
            },
            event_interval: EvidenceEventInterval { start: 100, end: Some(110) },
            effectivity_interval: ApplicabilityInterval { start: 120, end: Some(200) },
            disposition: EvidenceDisposition::Active,
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
        value.configuration_applicability.applicability.source = id(IdentityKind::ConfigurationRevision, "cfg-2");
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
    fn effectivity_exactly_matches_configuration_interval() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 90, end: Some(300) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_strictly_inside_configuration_interval_is_accepted() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 100, end: Some(299) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_starting_at_configuration_end_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 300, end: Some(301) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn effectivity_starting_before_configuration_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 89, end: Some(200) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn effectivity_ending_at_configuration_end_is_accepted() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(300) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_extending_beyond_configuration_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(301) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn finite_configuration_cannot_contain_open_ended_effectivity() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: None };
        assert!(value.validate().is_err());
    }

    #[test]
    fn open_ended_configuration_can_contain_open_ended_effectivity() {
        let mut value = evidence_case();
        value.configuration_applicability.interval.end = None;
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: None };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn event_time_can_be_outside_effectivity_without_bypassing_containment() {
        let mut value = evidence_case();
        value.event_interval = EvidenceEventInterval { start: 10, end: Some(20) };
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(200) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn temporal_applicability_witness_mismatch_is_rejected() {
        let mut value = evidence_case();
        value.configuration_applicability.interval.start = 100;
        value.configuration_applicability.interval.end = Some(200);
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(250) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn disputed_disposition_does_not_change_temporal_scope() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "witness-1"),
        };
        assert!(value.validate().is_ok());
        assert_eq!(value.effectivity_interval, ApplicabilityInterval { start: 120, end: Some(200) });
        assert_eq!(value.event_interval.start, 100);
    }

    #[test]
    fn superseded_disposition_is_explicitly_witnessed() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Superseded {
            by: id(IdentityKind::EvidenceRecord, "evidence-2"),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn retracted_disposition_is_explicitly_witnessed() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Retracted {
            by: id(IdentityKind::ReconciliationWitness, "correction-1"),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn disposition_cannot_self_reference() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Disputed {
            by: value.evidence.clone(),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn disposition_witness_must_be_typed() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Superseded {
            by: id(IdentityKind::ConfigurationRevision, "cfg-2"),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn unresolved_disposition_preserves_temporal_evidence() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Unresolved;
        assert!(value.validate().is_ok());
        assert_eq!(value.effectivity_interval.start, 120);
    }

    #[test]
    fn wrong_evidence_kind_is_rejected() {
        let mut value = evidence_case();
        value.evidence = id(IdentityKind::DesignRevision, "design-1");
        assert!(value.validate().is_err());
    }
}

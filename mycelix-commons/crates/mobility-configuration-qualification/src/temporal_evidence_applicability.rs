use serde::{Deserialize, Serialize};

use crate::identity_lineage::{ApplicabilityInterval, IdentityKind, IdentityRef, LineageEdge, LineageRelation};

/// Temporal evidence applicability is constrained by the explicit temporal
/// applicability of the configuration-to-artifact binding it references.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceEventInterval { pub start: u64, pub end: Option<u64> }

impl EvidenceEventInterval {
    pub fn validate(&self) -> Result<(), String> { ApplicabilityInterval { start: self.start, end: self.end }.validate() }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalEvidenceApplicability {
    pub evidence: IdentityRef,
    pub configuration: IdentityRef,
    pub artifact: IdentityRef,
    pub evidence_target: LineageEdge,
    pub configuration_applicability: LineageEdge,
    pub configuration_applicability_interval: ApplicabilityInterval,
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
        self.configuration_applicability_interval.validate()?;
        self.event_interval.validate()?;
        self.effectivity_interval.validate()?;
        if self.configuration.kind != IdentityKind::ConfigurationRevision { return Err("temporal evidence applicability requires a configuration revision".into()); }
        if self.artifact.kind != IdentityKind::PhysicalArtifact { return Err("temporal evidence applicability requires a physical artifact".into()); }
        let expected_relation = match self.evidence.kind {
            IdentityKind::InspectionRecord => LineageRelation::InspectedAs,
            IdentityKind::TestRecord => LineageRelation::TestedAs,
            IdentityKind::OperationalObservation => LineageRelation::ObservedAs,
            IdentityKind::MaintenanceEvent => LineageRelation::MaintainedAs,
            _ => return Err("temporal evidence applicability requires a typed lifecycle evidence event".into()),
        };
        if self.evidence_target.relation != expected_relation || self.evidence_target.source != self.evidence || self.evidence_target.target != self.artifact { return Err("evidence target must exactly bind evidence to the physical artifact".into()); }
        if self.configuration_applicability.relation != LineageRelation::AppliesTo || self.configuration_applicability.source != self.configuration || self.configuration_applicability.target != self.artifact { return Err("configuration applicability must exactly bind configuration to the physical artifact".into()); }
        if self.effectivity_interval.start < self.configuration_applicability_interval.start { return Err("evidence effectivity begins before configuration applicability".into()); }
        match (self.effectivity_interval.end, self.configuration_applicability_interval.end) {
            (Some(e), Some(c)) if e > c => return Err("evidence effectivity extends beyond configuration applicability".into()),
            (None, Some(_)) => return Err("open-ended evidence effectivity cannot outlive finite configuration applicability".into()),
            _ => {}
        }
        Ok(())
    }
    pub fn temporal_overlap(&self) -> bool { ApplicabilityInterval { start: self.event_interval.start, end: self.event_interval.end }.overlaps(&self.effectivity_interval) }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn id(kind: IdentityKind, value: &str) -> IdentityRef { IdentityRef { kind, namespace: "mobility".into(), id: value.into() } }
    fn edge(relation: LineageRelation, source: IdentityRef, target: IdentityRef) -> LineageEdge { LineageEdge { relation, source, target } }
    fn evidence_case() -> TemporalEvidenceApplicability {
        let evidence=id(IdentityKind::InspectionRecord,"inspection-1"); let configuration=id(IdentityKind::ConfigurationRevision,"cfg-1"); let artifact=id(IdentityKind::PhysicalArtifact,"artifact-1");
        TemporalEvidenceApplicability { evidence:evidence.clone(), configuration:configuration.clone(), artifact:artifact.clone(), evidence_target:edge(LineageRelation::InspectedAs,evidence,artifact.clone()), configuration_applicability:edge(LineageRelation::AppliesTo,configuration.clone(),artifact), configuration_applicability_interval:ApplicabilityInterval{start:50,end:Some(300)}, event_interval:EvidenceEventInterval{start:100,end:Some(110)}, effectivity_interval:ApplicabilityInterval{start:120,end:Some(200)} }
    }
    #[test] fn event_and_effectivity_can_be_disjoint(){ let v=evidence_case(); assert!(v.validate().is_ok()); assert!(!v.temporal_overlap()); }
    #[test] fn exact_effectivity_equality_is_accepted(){ let mut v=evidence_case(); v.effectivity_interval=v.configuration_applicability_interval; assert!(v.validate().is_ok()); }
    #[test] fn effectivity_inside_configuration_is_accepted(){ assert!(evidence_case().validate().is_ok()); }
    #[test] fn effectivity_touching_configuration_end_is_rejected(){ let mut v=evidence_case(); v.effectivity_interval=ApplicabilityInterval{start:300,end:Some(301)}; assert!(v.validate().is_err()); }
    #[test] fn effectivity_past_configuration_end_is_rejected(){ let mut v=evidence_case(); v.effectivity_interval.end=Some(301); assert!(v.validate().is_err()); }
    #[test] fn effectivity_before_configuration_start_is_rejected(){ let mut v=evidence_case(); v.effectivity_interval.start=49; assert!(v.validate().is_err()); }
    #[test] fn open_configuration_can_contain_open_effectivity(){ let mut v=evidence_case(); v.configuration_applicability_interval.end=None; v.effectivity_interval.end=None; assert!(v.validate().is_ok()); }
    #[test] fn finite_configuration_rejects_open_effectivity(){ let mut v=evidence_case(); v.effectivity_interval.end=None; assert!(v.validate().is_err()); }
    #[test] fn zero_length_effectivity_is_rejected(){ let mut v=evidence_case(); v.effectivity_interval.end=Some(v.effectivity_interval.start); assert!(v.validate().is_err()); }
    #[test] fn wrong_artifact_binding_is_rejected(){ let mut v=evidence_case(); v.artifact=id(IdentityKind::PhysicalArtifact,"artifact-2"); assert!(v.validate().is_err()); }
    #[test] fn wrong_configuration_binding_is_rejected(){ let mut v=evidence_case(); v.configuration=id(IdentityKind::ConfigurationRevision,"cfg-2"); assert!(v.validate().is_err()); }
    #[test] fn replacement_artifact_cannot_inherit_evidence(){ let mut v=evidence_case(); v.evidence_target.target=id(IdentityKind::PhysicalArtifact,"artifact-2"); assert!(v.validate().is_err()); }
    #[test] fn successor_configuration_cannot_inherit_evidence(){ let mut v=evidence_case(); v.configuration_applicability.source=id(IdentityKind::ConfigurationRevision,"cfg-2"); assert!(v.validate().is_err()); }
    #[test] fn holochain_artifact_identity_is_rejected(){ let mut v=evidence_case(); v.artifact.namespace="holochain".into(); assert!(v.validate().is_err()); }
    #[test] fn later_effectivity_preserves_event_history(){ let mut v=evidence_case(); v.effectivity_interval=ApplicabilityInterval{start:200,end:Some(250)}; assert!(v.validate().is_ok()); assert_eq!(v.event_interval.start,100); }
    #[test] fn wrong_evidence_kind_is_rejected(){ let mut v=evidence_case(); v.evidence=id(IdentityKind::DesignRevision,"design-1"); assert!(v.validate().is_err()); }
}

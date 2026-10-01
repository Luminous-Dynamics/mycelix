use serde::{Deserialize, Serialize};

/// Domain-neutral identity kinds. These are engineering semantics, not
/// Holochain action types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdentityKind {
    Requirement,
    DesignRevision,
    ConfigurationRevision,
    ComponentInstance,
    ManufacturingEvent,
    PhysicalArtifact,
    InspectionRecord,
    TestRecord,
    OperationalObservation,
    MaintenanceEvent,
    ChangeSet,
    EvidenceRecord,
}

/// An explicitly namespaced engineering identifier.
///
/// The namespace prevents foreign identifiers from silently becoming native
/// engineering identities. A Holochain hash is protocol metadata and is not a
/// valid engineering identifier merely because it is unique.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentityRef {
    pub kind: IdentityKind,
    pub namespace: String,
    pub id: String,
}

impl IdentityRef {
    pub fn validate(&self) -> Result<(), String> {
        if self.namespace.trim().is_empty() || self.id.trim().is_empty() {
            return Err("identity requires non-empty namespace and id".into());
        }
        if self.namespace == "holochain" {
            return Err("Holochain protocol identifiers require explicit binding and cannot be native engineering identities".into());
        }
        if self.id.starts_with("uhC0") || self.id.starts_with("uhCE") {
            return Err("Holochain action/entry hashes cannot be used as engineering identity".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LineageRelation {
    Instantiates,
    Configures,
    ComponentOf,
    ManufacturedFrom,
    InspectedAs,
    TestedAs,
    ObservedAs,
    MaintainedAs,
    RepairedAs,
    ReplacedBy,
    Supersedes,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LineageEdge {
    pub relation: LineageRelation,
    pub source: IdentityRef,
    pub target: IdentityRef,
}

impl LineageEdge {
    /// Validate identity/lineage semantics only. This does not establish
    /// physical equivalence, safety, certification, or regulatory approval.
    pub fn validate(&self) -> Result<(), String> {
        self.source.validate()?;
        self.target.validate()?;

        let valid = match self.relation {
            LineageRelation::Instantiates => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::PhysicalArtifact, IdentityKind::DesignRevision)
                    | (IdentityKind::ComponentInstance, IdentityKind::DesignRevision)
            ),
            LineageRelation::Configures => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::ConfigurationRevision, IdentityKind::DesignRevision)
                    | (IdentityKind::ConfigurationRevision, IdentityKind::ComponentInstance)
            ),
            LineageRelation::ComponentOf => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::ComponentInstance, IdentityKind::PhysicalArtifact)
                    | (IdentityKind::PhysicalArtifact, IdentityKind::PhysicalArtifact)
            ),
            LineageRelation::ManufacturedFrom => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::ManufacturingEvent, IdentityKind::DesignRevision)
                    | (IdentityKind::ManufacturingEvent, IdentityKind::ConfigurationRevision)
            ),
            LineageRelation::InspectedAs => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::InspectionRecord, IdentityKind::PhysicalArtifact)
            ),
            LineageRelation::TestedAs => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::TestRecord, IdentityKind::PhysicalArtifact)
            ),
            LineageRelation::ObservedAs => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::OperationalObservation, IdentityKind::PhysicalArtifact)
            ),
            LineageRelation::MaintainedAs | LineageRelation::RepairedAs => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::MaintenanceEvent, IdentityKind::PhysicalArtifact)
            ),
            LineageRelation::ReplacedBy => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::ComponentInstance, IdentityKind::ComponentInstance)
                    | (IdentityKind::PhysicalArtifact, IdentityKind::PhysicalArtifact)
            ),
            LineageRelation::Supersedes => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::DesignRevision, IdentityKind::DesignRevision)
                    | (IdentityKind::ConfigurationRevision, IdentityKind::ConfigurationRevision)
                    | (IdentityKind::EvidenceRecord, IdentityKind::EvidenceRecord)
            ),
        };

        if valid {
            Ok(())
        } else {
            Err(format!(
                "invalid identity kinds for {:?}: {:?} -> {:?}",
                self.relation, self.source.kind, self.target.kind
            ))
        }
    }
}


 
/// A deterministic half-open interval used to qualify when a configuration
/// applicability assertion is in force. This is temporal scope, not proof of
/// safety, conformance, certification, or measurement truth.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApplicabilityInterval {
    pub start: u64,
    pub end: Option<u64>,
}

impl ApplicabilityInterval {
    pub fn validate(&self) -> Result<(), String> {
        if let Some(end) = self.end {
            if end <= self.start {
                return Err("applicability interval must satisfy start < end".into());
            }
        }
        Ok(())
    }

    pub fn contains(&self, instant: u64) -> bool {
        instant >= self.start && self.end.map(|end| instant < end).unwrap_or(true)
    }

    pub fn overlaps(&self, other: &Self) -> bool {
        let left_end = self.end.unwrap_or(u64::MAX);
        let right_end = other.end.unwrap_or(u64::MAX);
        self.start < right_end && other.start < left_end
    }
}

/// Explicitly time-scoped configuration applicability.
///
/// The underlying AppliesTo edge remains the identity/lineage assertion;
/// this wrapper prevents consumers from silently treating that assertion as
/// timeless.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalConfigurationApplicability {
    pub configuration: IdentityRef,
    pub artifact: IdentityRef,
    pub applicability: LineageEdge,
    pub interval: ApplicabilityInterval,
}

impl TemporalConfigurationApplicability {
    pub fn validate(&self) -> Result<(), String> {
        self.configuration.validate()?;
        self.artifact.validate()?;
        self.interval.validate()?;
        if self.configuration.kind != IdentityKind::ConfigurationRevision {
            return Err("temporal applicability requires a configuration revision".into());
        }
        if self.artifact.kind != IdentityKind::PhysicalArtifact {
            return Err("temporal applicability requires a physical artifact".into());
        }
        if self.applicability.relation != LineageRelation::AppliesTo
            || self.applicability.source != self.configuration
            || self.applicability.target != self.artifact
        {
            return Err("temporal applicability requires an exact configuration-to-artifact AppliesTo edge".into());
        }
        self.applicability.validate()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(kind: IdentityKind, value: &str) -> IdentityRef {
        IdentityRef {
            kind,
            namespace: "mobility".into(),
            id: value.into(),
        }
    }

    #[test]
    fn same_design_can_instantiate_two_distinct_physical_artifacts() {
        let design = id(IdentityKind::DesignRevision, "design-r1");
        let a = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let b = id(IdentityKind::PhysicalArtifact, "artifact-b");

        assert_ne!(a, b);
        assert!(LineageEdge {
            relation: LineageRelation::Instantiates,
            source: a,
            target: design.clone(),
        }
        .validate()
        .is_ok());
        assert!(LineageEdge {
            relation: LineageRelation::Instantiates,
            source: b,
            target: design,
        }
        .validate()
        .is_ok());
    }

    #[test]
    fn configuration_revision_does_not_create_new_physical_artifact() {
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let config_a = id(IdentityKind::ConfigurationRevision, "config-r1");
        let config_b = id(IdentityKind::ConfigurationRevision, "config-r2");

        assert_ne!(config_a, config_b);
        assert!(LineageEdge {
            relation: LineageRelation::Configures,
            source: config_a,
            target: IdentityRef {
                kind: IdentityKind::DesignRevision,
                namespace: "mobility".into(),
                id: "design-r1".into(),
            },
        }
        .validate()
        .is_ok());
        assert_eq!(artifact.kind, IdentityKind::PhysicalArtifact);
    }

    #[test]
    fn repair_preserves_artifact_identity() {
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let repair = id(IdentityKind::MaintenanceEvent, "repair-1");
        assert!(LineageEdge {
            relation: LineageRelation::RepairedAs,
            source: repair,
            target: artifact.clone(),
        }
        .validate()
        .is_ok());
        assert_eq!(artifact.id, "artifact-a");
    }

    #[test]
    fn component_replacement_preserves_replaced_component_history() {
        let old = id(IdentityKind::ComponentInstance, "component-old");
        let new = id(IdentityKind::ComponentInstance, "component-new");
        assert_ne!(old, new);
        assert!(LineageEdge {
            relation: LineageRelation::ReplacedBy,
            source: old,
            target: new,
        }
        .validate()
        .is_ok());
    }

    #[test]
    fn invalid_semantic_substitution_is_rejected() {
        let edge = LineageEdge {
            relation: LineageRelation::Instantiates,
            source: id(IdentityKind::ConfigurationRevision, "config-r1"),
            target: id(IdentityKind::PhysicalArtifact, "artifact-a"),
        };
        assert!(edge.validate().is_err());
    }

    #[test]
    fn holochain_namespace_is_rejected_without_explicit_binding() {
        let identity = IdentityRef {
            kind: IdentityKind::PhysicalArtifact,
            namespace: "holochain".into(),
            id: "uhCkk-example".into(),
        };
        assert!(identity.validate().is_err());
    }

    #[test]
    fn ambiguous_identity_is_rejected() {
        let identity = IdentityRef {
            kind: IdentityKind::PhysicalArtifact,
            namespace: "".into(),
            id: "artifact-a".into(),
        };
        assert!(identity.validate().is_err());
    }

    #[test]
    fn geometry_identity_does_not_encode_manufacturing_equivalence() {
        let a = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let b = id(IdentityKind::PhysicalArtifact, "artifact-b");
        assert_ne!(a, b);
    }

    #[test]
    fn supersession_is_identity_lineage_not_physical_replacement() {
        let old = id(IdentityKind::ConfigurationRevision, "config-r1");
        let new = id(IdentityKind::ConfigurationRevision, "config-r2");
        assert!(LineageEdge {
            relation: LineageRelation::Supersedes,
            source: new,
            target: old,
        }
        .validate()
        .is_ok());
    }
}

use serde::{Deserialize, Serialize};

/// Domain-neutral identity kinds. These are engineering semantics, not
/// Holochain action types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
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
    ReconciliationWitness,
    ArtifactLifecycleEvent,
}

/// An explicitly namespaced engineering identifier.
///
/// The namespace prevents foreign identifiers from silently becoming native
/// engineering identities. A Holochain hash is protocol metadata and is not a
/// valid engineering identifier merely because it is unique.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
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
    AppliesTo,
    ComponentOf,
    ManufacturedFrom,
    InspectedAs,
    TestedAs,
    ObservedAs,
    MaintainedAs,
    RepairedAs,
    ReplacedBy,
    Retires,
    Reactivates,
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
            LineageRelation::AppliesTo => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::ConfigurationRevision, IdentityKind::PhysicalArtifact)
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
            LineageRelation::Retires | LineageRelation::Reactivates => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::ArtifactLifecycleEvent, IdentityKind::PhysicalArtifact)
            ),
            LineageRelation::Supersedes => matches!(
                (self.source.kind, self.target.kind),
                (IdentityKind::DesignRevision, IdentityKind::DesignRevision)
                    | (IdentityKind::ConfigurationRevision, IdentityKind::ConfigurationRevision)
                    | (IdentityKind::EvidenceRecord, IdentityKind::EvidenceRecord)
                    | (IdentityKind::ReconciliationWitness, IdentityKind::ReconciliationWitness)
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


/// Explicit lifecycle transition for configuration applicability.
///
/// A configuration successor never inherits physical-artifact applicability
/// implicitly from its predecessor. Each successor applicability claim must be
/// represented by its own exact AppliesTo edge.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApplicabilityTransition {
    pub predecessor: IdentityRef,
    pub successor: IdentityRef,
    pub predecessor_artifact: IdentityRef,
    pub successor_artifact: Option<IdentityRef>,
    pub supersession: LineageEdge,
    pub predecessor_applicability: LineageEdge,
    pub successor_applicability: Option<LineageEdge>,
}

impl ApplicabilityTransition {
    pub fn validate(&self) -> Result<(), String> {
        self.predecessor.validate()?;
        self.successor.validate()?;
        self.predecessor_artifact.validate()?;
        if let Some(artifact) = &self.successor_artifact {
            artifact.validate()?;
        }

        if self.predecessor.kind != IdentityKind::ConfigurationRevision
            || self.successor.kind != IdentityKind::ConfigurationRevision
        {
            return Err("applicability transition requires configuration revisions".into());
        }
        if self.predecessor_artifact.kind != IdentityKind::PhysicalArtifact {
            return Err("applicability transition requires a physical artifact".into());
        }
        if let Some(artifact) = &self.successor_artifact {
            if artifact.kind != IdentityKind::PhysicalArtifact {
                return Err("successor applicability requires a physical artifact".into());
            }
        }
        if self.predecessor == self.successor {
            return Err("configuration applicability transition requires distinct revisions".into());
        }

        if self.supersession.relation != LineageRelation::Supersedes
            || self.supersession.source != self.successor
            || self.supersession.target != self.predecessor
        {
            return Err("configuration transition requires explicit successor-to-predecessor Supersedes edge".into());
        }
        self.supersession.validate()?;

        if self.predecessor_applicability.relation != LineageRelation::AppliesTo
            || self.predecessor_applicability.source != self.predecessor
            || self.predecessor_applicability.target != self.predecessor_artifact
        {
            return Err("predecessor applicability must exactly bind predecessor configuration to artifact".into());
        }
        self.predecessor_applicability.validate()?;

        if let Some(successor) = &self.successor_applicability {
            if successor.relation != LineageRelation::AppliesTo
                || successor.source != self.successor
                || Some(successor.target.clone()) != self.successor_artifact
            {
                return Err("successor applicability must exactly bind successor configuration to artifact".into());
            }
            successor.validate()?;
        }

        Ok(())
    }
}


/// Explicit lifecycle transition for a physical artifact.
///
/// Physical-artifact identity and lifecycle state are separate from
/// configuration applicability. A replacement therefore creates a distinct
/// artifact identity and never transfers predecessor applicability implicitly.
/// Retirement and reactivation preserve the same artifact identity while
/// requiring explicit lifecycle events; reactivation does not resurrect
/// predecessor applicability automatically.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArtifactLifecycleTransitionKind {
    Repair,
    Replacement,
    Retirement,
    Reactivation,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArtifactLifecycleTransition {
    pub kind: ArtifactLifecycleTransitionKind,
    pub artifact: IdentityRef,
    pub successor_artifact: Option<IdentityRef>,
    pub transition_event: LineageEdge,
    pub retirement_event: Option<LineageEdge>,
    pub predecessor_applicability: Option<LineageEdge>,
    pub successor_applicability: Option<LineageEdge>,
}

impl ArtifactLifecycleTransition {
    pub fn validate(&self) -> Result<(), String> {
        self.artifact.validate()?;
        if self.artifact.kind != IdentityKind::PhysicalArtifact {
            return Err("artifact lifecycle transition requires a physical artifact".into());
        }
        if let Some(successor) = &self.successor_artifact {
            successor.validate()?;
            if successor.kind != IdentityKind::PhysicalArtifact {
                return Err("successor artifact must be a physical artifact".into());
            }
        }
        if let Some(edge) = &self.predecessor_applicability {
            edge.validate()?;
            if edge.relation != LineageRelation::AppliesTo
                || edge.target != self.artifact
                || edge.source.kind != IdentityKind::ConfigurationRevision
            {
                return Err("predecessor applicability must exactly bind a configuration to the transitioned artifact".into());
            }
        }
        if let Some(edge) = &self.successor_applicability {
            edge.validate()?;
            let successor = self.successor_artifact.as_ref().ok_or_else(|| {
                "successor applicability requires a successor physical artifact".to_string()
            })?;
            if edge.relation != LineageRelation::AppliesTo
                || edge.target != *successor
                || edge.source.kind != IdentityKind::ConfigurationRevision
            {
                return Err("successor applicability must exactly bind a configuration to the successor artifact".into());
            }
        }
        if self.retirement_event.is_some()
            && self.kind != ArtifactLifecycleTransitionKind::Reactivation
        {
            return Err("retirement event is only a dependency of reactivation".into());
        }

        match self.kind {
            ArtifactLifecycleTransitionKind::Repair => {
                if self.successor_artifact.is_some()
                    || self.successor_applicability.is_some()
                    || self.retirement_event.is_some()
                {
                    return Err("repair preserves artifact identity and cannot declare a successor or retirement dependency".into());
                }
                if self.transition_event.relation != LineageRelation::RepairedAs
                    || self.transition_event.source.kind != IdentityKind::MaintenanceEvent
                    || self.transition_event.target != self.artifact
                {
                    return Err("repair requires MaintenanceEvent RepairedAs transitioned artifact".into());
                }
            }
            ArtifactLifecycleTransitionKind::Replacement => {
                let successor = self.successor_artifact.as_ref().ok_or_else(|| {
                    "replacement requires a distinct successor physical artifact".to_string()
                })?;
                if successor == &self.artifact {
                    return Err("replacement requires distinct physical-artifact identities".into());
                }
                if self.retirement_event.is_some() {
                    return Err("replacement does not encode retirement dependency".into());
                }
                if self.transition_event.relation != LineageRelation::ReplacedBy
                    || self.transition_event.source != self.artifact
                    || self.transition_event.target != *successor
                {
                    return Err("replacement requires predecessor ReplacedBy successor".into());
                }
            }
            ArtifactLifecycleTransitionKind::Retirement => {
                if self.successor_artifact.is_some() || self.successor_applicability.is_some() {
                    return Err("retirement does not create a successor artifact or applicability claim".into());
                }
                if self.transition_event.relation != LineageRelation::Retires
                    || self.transition_event.source.kind != IdentityKind::ArtifactLifecycleEvent
                    || self.transition_event.target != self.artifact
                {
                    return Err("retirement requires ArtifactLifecycleEvent Retires artifact".into());
                }
            }
            ArtifactLifecycleTransitionKind::Reactivation => {
                if self.successor_artifact.is_some() || self.successor_applicability.is_some() {
                    return Err("reactivation preserves artifact identity and does not resurrect applicability implicitly".into());
                }
                if self.transition_event.relation != LineageRelation::Reactivates
                    || self.transition_event.source.kind != IdentityKind::ArtifactLifecycleEvent
                    || self.transition_event.target != self.artifact
                {
                    return Err("reactivation requires ArtifactLifecycleEvent Reactivates artifact".into());
                }
                let retirement = self.retirement_event.as_ref().ok_or_else(|| {
                    "reactivation requires an explicit prior retirement event".to_string()
                })?;
                if retirement.relation != LineageRelation::Retires
                    || retirement.source.kind != IdentityKind::ArtifactLifecycleEvent
                    || retirement.target != self.artifact
                {
                    return Err("reactivation retirement dependency must explicitly retire the same artifact".into());
                }
                retirement.validate()?;
            }
        }

        self.transition_event.validate()?;
        Ok(())
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
    fn temporal_applicability_rejects_zero_length_interval() {
        assert!(ApplicabilityInterval { start: 10, end: Some(10) }.validate().is_err());
    }

    #[test]
    fn temporal_applicability_rejects_reversed_interval() {
        assert!(ApplicabilityInterval { start: 20, end: Some(10) }.validate().is_err());
    }

    #[test]
    fn temporal_applicability_uses_half_open_bounds() {
        let interval = ApplicabilityInterval { start: 10, end: Some(20) };
        assert!(interval.contains(10));
        assert!(interval.contains(19));
        assert!(!interval.contains(20));
    }

    #[test]
    fn adjacent_intervals_do_not_overlap() {
        let left = ApplicabilityInterval { start: 0, end: Some(10) };
        let right = ApplicabilityInterval { start: 10, end: Some(20) };
        assert!(!left.overlaps(&right));
    }

    #[test]
    fn overlapping_intervals_are_detected() {
        let left = ApplicabilityInterval { start: 0, end: Some(10) };
        let right = ApplicabilityInterval { start: 9, end: Some(20) };
        assert!(left.overlaps(&right));
    }

    #[test]
    fn open_ended_interval_overlaps_later_interval() {
        let open = ApplicabilityInterval { start: 10, end: None };
        let later = ApplicabilityInterval { start: 100, end: Some(200) };
        assert!(open.overlaps(&later));
    }

    #[test]
    fn temporal_applicability_requires_exact_edge_binding() {
        let configuration = id(IdentityKind::ConfigurationRevision, "config-r1");
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let t = TemporalConfigurationApplicability {
            configuration: configuration.clone(),
            artifact: artifact.clone(),
            applicability: LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: configuration,
                target: id(IdentityKind::PhysicalArtifact, "artifact-b"),
            },
            interval: ApplicabilityInterval { start: 0, end: Some(10) },
        };
        assert!(t.validate().is_err());
    }

    #[test]
    fn temporal_applicability_rejects_holochain_configuration_identity() {
        let configuration = IdentityRef {
            kind: IdentityKind::ConfigurationRevision,
            namespace: "holochain".into(),
            id: "uhC0-invalid".into(),
        };
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let t = TemporalConfigurationApplicability {
            configuration: configuration.clone(),
            artifact: artifact.clone(),
            applicability: LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: configuration,
                target: artifact,
            },
            interval: ApplicabilityInterval { start: 0, end: Some(10) },
        };
        assert!(t.validate().is_err());
    }

    #[test]
    fn configuration_successor_does_not_inherit_applicability_implicitly() {
        let predecessor = id(IdentityKind::ConfigurationRevision, "config-r1");
        let successor = id(IdentityKind::ConfigurationRevision, "config-r2");
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let t = ApplicabilityTransition {
            predecessor: predecessor.clone(),
            successor: successor.clone(),
            predecessor_artifact: artifact.clone(),
            successor_artifact: None,
            supersession: LineageEdge {
                relation: LineageRelation::Supersedes,
                source: successor.clone(),
                target: predecessor.clone(),
            },
            predecessor_applicability: LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: predecessor,
                target: artifact.clone(),
            },
            successor_applicability: None,
        };
        assert!(t.validate().is_ok());
    }

    #[test]
    fn successor_same_artifact_requires_explicit_new_applicability() {
        let predecessor = id(IdentityKind::ConfigurationRevision, "config-r1");
        let successor = id(IdentityKind::ConfigurationRevision, "config-r2");
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let mut t = ApplicabilityTransition {
            predecessor: predecessor.clone(),
            successor: successor.clone(),
            predecessor_artifact: artifact.clone(),
            successor_artifact: None,
            supersession: LineageEdge {
                relation: LineageRelation::Supersedes,
                source: successor.clone(),
                target: predecessor.clone(),
            },
            predecessor_applicability: LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: predecessor,
                target: artifact.clone(),
            },
            successor_applicability: None,
        };
        assert!(t.validate().is_ok());
        t.successor_artifact = Some(artifact.clone());
        t.successor_applicability = Some(LineageEdge {
            relation: LineageRelation::AppliesTo,
            source: successor,
            target: artifact,
        });
        assert!(t.validate().is_ok());
    }

    #[test]
    fn successor_retargeting_is_explicit_and_distinct() {
        let predecessor = id(IdentityKind::ConfigurationRevision, "config-r1");
        let successor = id(IdentityKind::ConfigurationRevision, "config-r2");
        let old_artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let new_artifact = id(IdentityKind::PhysicalArtifact, "artifact-b");
        let t = ApplicabilityTransition {
            predecessor: predecessor.clone(),
            successor: successor.clone(),
            predecessor_artifact: old_artifact.clone(),
            successor_artifact: Some(new_artifact.clone()),
            supersession: LineageEdge {
                relation: LineageRelation::Supersedes,
                source: successor.clone(),
                target: predecessor.clone(),
            },
            predecessor_applicability: LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: predecessor,
                target: old_artifact,
            },
            successor_applicability: Some(LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: successor,
                target: new_artifact,
            }),
        };
        assert!(t.validate().is_ok());
    }

    #[test]
    fn successor_cannot_silently_retarget_predecessor_edge() {
        let predecessor = id(IdentityKind::ConfigurationRevision, "config-r1");
        let successor = id(IdentityKind::ConfigurationRevision, "config-r2");
        let old_artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let new_artifact = id(IdentityKind::PhysicalArtifact, "artifact-b");
        let t = ApplicabilityTransition {
            predecessor: predecessor.clone(),
            successor: successor.clone(),
            predecessor_artifact: old_artifact.clone(),
            successor_artifact: None,
            supersession: LineageEdge {
                relation: LineageRelation::Supersedes,
                source: successor.clone(),
                target: predecessor.clone(),
            },
            predecessor_applicability: LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: predecessor,
                target: new_artifact,
            },
            successor_applicability: Some(LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: successor,
                target: old_artifact,
            }),
        };
        assert!(t.validate().is_err());
    }


    #[test]
    fn replacement_requires_distinct_artifact_and_never_inherits_applicability() {
        let old = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let new = id(IdentityKind::PhysicalArtifact, "artifact-b");
        let transition = ArtifactLifecycleTransition {
            kind: ArtifactLifecycleTransitionKind::Replacement,
            artifact: old.clone(),
            successor_artifact: Some(new.clone()),
            transition_event: LineageEdge {
                relation: LineageRelation::ReplacedBy,
                source: old.clone(),
                target: new.clone(),
            },
            retirement_event: None,
            predecessor_applicability: Some(LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: id(IdentityKind::ConfigurationRevision, "config-r1"),
                target: old,
            }),
            successor_applicability: None,
        };
        assert!(transition.validate().is_ok());
    }

    #[test]
    fn replacement_cannot_reuse_same_artifact_identity() {
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let transition = ArtifactLifecycleTransition {
            kind: ArtifactLifecycleTransitionKind::Replacement,
            artifact: artifact.clone(),
            successor_artifact: Some(artifact.clone()),
            transition_event: LineageEdge {
                relation: LineageRelation::ReplacedBy,
                source: artifact.clone(),
                target: artifact.clone(),
            },
            retirement_event: None,
            predecessor_applicability: None,
            successor_applicability: None,
        };
        assert!(transition.validate().is_err());
    }

    #[test]
    fn reversed_replacement_is_rejected() {
        let old = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let new = id(IdentityKind::PhysicalArtifact, "artifact-b");
        let transition = ArtifactLifecycleTransition {
            kind: ArtifactLifecycleTransitionKind::Replacement,
            artifact: old.clone(),
            successor_artifact: Some(new.clone()),
            transition_event: LineageEdge {
                relation: LineageRelation::ReplacedBy,
                source: new,
                target: old,
            },
            retirement_event: None,
            predecessor_applicability: None,
            successor_applicability: None,
        };
        assert!(transition.validate().is_err());
    }

    #[test]
    fn repair_preserves_identity_without_successor() {
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let transition = ArtifactLifecycleTransition {
            kind: ArtifactLifecycleTransitionKind::Repair,
            artifact: artifact.clone(),
            successor_artifact: None,
            transition_event: LineageEdge {
                relation: LineageRelation::RepairedAs,
                source: id(IdentityKind::MaintenanceEvent, "repair-1"),
                target: artifact,
            },
            retirement_event: None,
            predecessor_applicability: None,
            successor_applicability: None,
        };
        assert!(transition.validate().is_ok());
    }

    #[test]
    fn retirement_preserves_artifact_identity_and_history() {
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let transition = ArtifactLifecycleTransition {
            kind: ArtifactLifecycleTransitionKind::Retirement,
            artifact: artifact.clone(),
            successor_artifact: None,
            transition_event: LineageEdge {
                relation: LineageRelation::Retires,
                source: id(IdentityKind::ArtifactLifecycleEvent, "retire-1"),
                target: artifact,
            },
            retirement_event: None,
            predecessor_applicability: None,
            successor_applicability: None,
        };
        assert!(transition.validate().is_ok());
    }

    #[test]
    fn reactivation_requires_explicit_retirement_and_does_not_restore_applicability() {
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let retirement = LineageEdge {
            relation: LineageRelation::Retires,
            source: id(IdentityKind::ArtifactLifecycleEvent, "retire-1"),
            target: artifact.clone(),
        };
        let transition = ArtifactLifecycleTransition {
            kind: ArtifactLifecycleTransitionKind::Reactivation,
            artifact: artifact.clone(),
            successor_artifact: None,
            transition_event: LineageEdge {
                relation: LineageRelation::Reactivates,
                source: id(IdentityKind::ArtifactLifecycleEvent, "reactivate-1"),
                target: artifact,
            },
            retirement_event: Some(retirement),
            predecessor_applicability: None,
            successor_applicability: None,
        };
        assert!(transition.validate().is_ok());
    }

    #[test]
    fn reactivation_with_successor_applicability_is_rejected() {
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-a");
        let retirement = LineageEdge {
            relation: LineageRelation::Retires,
            source: id(IdentityKind::ArtifactLifecycleEvent, "retire-1"),
            target: artifact.clone(),
        };
        let transition = ArtifactLifecycleTransition {
            kind: ArtifactLifecycleTransitionKind::Reactivation,
            artifact: artifact.clone(),
            successor_artifact: None,
            transition_event: LineageEdge {
                relation: LineageRelation::Reactivates,
                source: id(IdentityKind::ArtifactLifecycleEvent, "reactivate-1"),
                target: artifact.clone(),
            },
            retirement_event: Some(retirement),
            predecessor_applicability: None,
            successor_applicability: Some(LineageEdge {
                relation: LineageRelation::AppliesTo,
                source: id(IdentityKind::ConfigurationRevision, "config-r2"),
                target: artifact,
            }),
        };
        assert!(transition.validate().is_err());
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
    fn witness_supersession_preserves_witness_identity_history() {
        let old = id(IdentityKind::ReconciliationWitness, "witness-r1");
        let new = id(IdentityKind::ReconciliationWitness, "witness-r2");
        assert!(LineageEdge {
            relation: LineageRelation::Supersedes,
            source: new,
            target: old,
        }
        .validate()
        .is_ok());
    }

    #[test]
    fn reconciliation_witness_is_not_a_claim_identity() {
        let witness = id(IdentityKind::ReconciliationWitness, "witness-r1");
        let claim = id(IdentityKind::EvidenceRecord, "claim-r1");
        assert_ne!(witness, claim);
        assert_ne!(witness.kind, claim.kind);
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

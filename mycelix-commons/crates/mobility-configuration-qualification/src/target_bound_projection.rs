use crate::{
    identity_lineage::{IdentityKind, IdentityRef},
    reconciliation_evidence_projection::ReconciliationEvidenceProjection,
    temporal_reconciliation_witness::TemporalReconciliationWitness,
    EvidenceState,
};
use serde::{Deserialize, Serialize};

/// A projection record whose semantic target is explicit and typed.
///
/// This wrapper prevents a valid reconciliation witness from being applied to
/// an unrelated EvidenceState merely because the caller supplied an arbitrary
/// base value. It is provenance binding only.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TargetBoundReconciliationProjection {
    pub target: IdentityRef,
    pub configuration_scope: IdentityRef,
    pub physical_artifact_scope: IdentityRef,
    pub configuration_artifact_binding: crate::identity_lineage::LineageEdge,
    pub projection: ReconciliationEvidenceProjection,
}

impl TargetBoundReconciliationProjection {
    pub fn validate(
        &self,
        target: &IdentityRef,
        configuration_scope: &IdentityRef,
        physical_artifact_scope: &IdentityRef,
        witness: &TemporalReconciliationWitness,
        base: &EvidenceState,
    ) -> Result<(), String> {
        self.target.validate()?;
        if self.target.kind != IdentityKind::EvidenceRecord {
            return Err("projection target must use EvidenceRecord identity kind".into());
        }
        if &self.target != target {
            return Err("projection target does not identify the supplied evidence target".into());
        }
        self.configuration_scope.validate()?;
        if self.configuration_scope.kind != IdentityKind::ConfigurationRevision {
            return Err("projection configuration scope must use ConfigurationRevision identity kind".into());
        }
        if &self.configuration_scope != configuration_scope {
            return Err("projection configuration scope does not identify the supplied configuration scope".into());
        }
        self.physical_artifact_scope.validate()?;
        if self.physical_artifact_scope.kind != IdentityKind::PhysicalArtifact {
            return Err("projection physical artifact scope must use PhysicalArtifact identity kind".into());
        }
        if &self.physical_artifact_scope != physical_artifact_scope {
            return Err("projection physical artifact scope does not identify the supplied physical artifact".into());
        }
        self.configuration_artifact_binding.validate()?;
        if self.configuration_artifact_binding.relation != crate::identity_lineage::LineageRelation::AppliesTo {
            return Err("configuration-artifact binding must use AppliesTo relation".into());
        }
        if self.configuration_artifact_binding.source != self.configuration_scope
            || self.configuration_artifact_binding.target != self.physical_artifact_scope
        {
            return Err("configuration-artifact binding must exactly match configuration and physical artifact scopes".into());
        }
        if self.configuration_scope == witness.witness_identity
            || self.configuration_scope == witness.left_claim
            || self.configuration_scope == witness.right_claim
        {
            return Err("projection configuration scope must be distinct from witness and claim identities".into());
        }
        if self.target == witness.witness_identity {
            return Err("projection target cannot be the reconciliation witness identity".into());
        }
        if self.target == witness.left_claim || self.target == witness.right_claim {
            return Err("projection target must be distinct from compared claim identities".into());
        }
        self.projection.validate(witness, base)
    }

    pub fn apply(
        &self,
        target: &IdentityRef,
        configuration_scope: &IdentityRef,
        physical_artifact_scope: &IdentityRef,
        witness: &TemporalReconciliationWitness,
        base: &EvidenceState,
    ) -> Result<EvidenceState, String> {
        self.validate(target, configuration_scope, physical_artifact_scope, witness, base)?;
        self.projection.apply(witness, base)
    }

    pub fn witness_identity(&self) -> &IdentityRef {
        self.projection.witness_identity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        identity_lineage::IdentityKind,
        temporal_applicability::{TemporalInterval, TemporalPoint},
        temporal_reconciliation::{classify, Comparability, Compatibility},
        ConflictDisposition, EpistemicDisposition, EvidenceModality, LifecycleDisposition,
        AuthorityProvenance,
    };

    fn id(kind: IdentityKind, value: &str) -> IdentityRef {
        IdentityRef { kind, namespace: "synthetic".into(), id: value.into() }
    }

    fn target(value: &str) -> IdentityRef { id(IdentityKind::EvidenceRecord, value) }

    fn witness() -> TemporalReconciliationWitness {
        let left_applicability = TemporalInterval {
            start: Some(TemporalPoint(0)),
            end: Some(TemporalPoint(10)),
        };
        let right_applicability = TemporalInterval {
            start: Some(TemporalPoint(5)),
            end: Some(TemporalPoint(15)),
        };
        let result = classify(
            Comparability::Comparable,
            Compatibility::Incompatible,
            &left_applicability,
            &right_applicability,
            false,
            true,
        ).unwrap();
        TemporalReconciliationWitness {
            witness_identity: id(IdentityKind::ReconciliationWitness, "w1"),
            left_claim: id(IdentityKind::EvidenceRecord, "claim-a"),
            right_claim: id(IdentityKind::EvidenceRecord, "claim-b"),
            left_applicability,
            right_applicability,
            comparability: Comparability::Comparable,
            compatibility: Compatibility::Incompatible,
            explicitly_superseded: false,
            disputed: true,
            result,
        }
    }

    fn base() -> EvidenceState {
        EvidenceState {
            epistemic_disposition: EpistemicDisposition::Supported,
            lifecycle_disposition: LifecycleDisposition::Current,
            conflict_disposition: ConflictDisposition::Uncontested,
            authority_provenance: AuthorityProvenance::Commons,
            evidence_modality: EvidenceModality::Interpretation,
            contradiction_reference: None,
            conflict_reference: None,
            unresolved_dependency_reference: None,
            external_authority_reference: None,
        }
    }

    fn projection(target: IdentityRef) -> TargetBoundReconciliationProjection {
        TargetBoundReconciliationProjection {
            target,
            configuration_scope: id(IdentityKind::ConfigurationRevision, "config-r1"),
            physical_artifact_scope: id(IdentityKind::PhysicalArtifact, "artifact-a"),
            configuration_artifact_binding: crate::identity_lineage::LineageEdge {
                relation: crate::identity_lineage::LineageRelation::AppliesTo,
                source: id(IdentityKind::ConfigurationRevision, "config-r1"),
                target: id(IdentityKind::PhysicalArtifact, "artifact-a"),
            },
            projection: ReconciliationEvidenceProjection::ConflictReferenceOnly {
                witness_ref: id(IdentityKind::ReconciliationWitness, "w1"),
            },
        }
    }

    #[test]
    fn exact_target_binding_is_accepted() {
        let t = target("target-a");
        assert!(projection(t.clone()).apply(&t, &id(IdentityKind::ConfigurationRevision, "config-r1"), &id(IdentityKind::PhysicalArtifact, "artifact-a"), &witness(), &base()).is_ok());
    }

    #[test]
    fn different_target_is_rejected_even_with_identical_state() {
        let declared = target("target-a");
        let supplied = target("target-b");
        assert!(projection(declared).apply(&supplied, &id(IdentityKind::ConfigurationRevision, "config-r1"), &witness(), &base()).is_err());
    }

    #[test]
    fn witness_identity_cannot_be_target() {
        let t = id(IdentityKind::ReconciliationWitness, "w1");
        let p = projection(t.clone());
        assert!(p.validate(&t, &id(IdentityKind::ConfigurationRevision, "config-r1"), &witness(), &base()).is_err());
    }

    #[test]
    fn compared_claim_identity_cannot_be_target() {
        let t = id(IdentityKind::EvidenceRecord, "claim-a");
        let p = projection(t.clone());
        assert!(p.validate(&t, &id(IdentityKind::ConfigurationRevision, "config-r1"), &witness(), &base()).is_err());
    }

    #[test]
    fn holochain_shaped_target_is_rejected() {
        let t = IdentityRef {
            kind: IdentityKind::EvidenceRecord,
            namespace: "holochain".into(),
            id: "uhC0target".into(),
        };
        let p = projection(t.clone());
        assert!(p.validate(&t, &witness(), &base()).is_err());
    }

    #[test]
    fn configuration_scope_must_be_explicitly_matching() {
        let t = target("target-a");
        let p = projection(t.clone());
        let wrong_scope = id(IdentityKind::ConfigurationRevision, "config-r2");
        assert!(p.validate(&t, &wrong_scope, &id(IdentityKind::PhysicalArtifact, "artifact-a"), &witness(), &base()).is_err());
    }

    #[test]
    fn witness_identity_cannot_be_configuration_scope() {
        let t = target("target-a");
        let p = projection(t.clone());
        let witness_scope = id(IdentityKind::ReconciliationWitness, "w1");
        assert!(p.validate(&t, &witness_scope, &id(IdentityKind::PhysicalArtifact, "artifact-a"), &witness(), &base()).is_err());
    }

    #[test]
    fn holochain_configuration_scope_is_rejected() {
        let t = target("target-a");
        let p = projection(t.clone());
        let scope = IdentityRef { kind: IdentityKind::ConfigurationRevision, namespace: "holochain".into(), id: "uhC0config".into() };
        assert!(p.validate(&t, &scope, &id(IdentityKind::PhysicalArtifact, "artifact-a"), &witness(), &base()).is_err());
    }

    #[test]
    fn historical_target_binding_is_independent_of_witness_revision() {
        let t = target("target-a");
        let p1 = projection(t.clone());
        assert!(p1.apply(&t, &id(IdentityKind::ConfigurationRevision, "config-r1"), &witness(), &base()).is_ok());
        assert_eq!(p1.witness_identity().id, "w1");
        assert_eq!(p1.target.id, "target-a");
    }
}

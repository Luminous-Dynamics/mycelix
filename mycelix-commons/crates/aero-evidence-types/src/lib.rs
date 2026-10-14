//! AeroCommons evidence primitives.
//!
//! This crate intentionally models evidence and provenance without making
//! certification or airworthiness claims. It is the typed boundary between
//! engineering records and the Mycelix provenance layer.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use thiserror::Error;

/// The epistemic origin of an evidence record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimKind {
    Measurement,
    Analysis,
    Simulation,
    Inspection,
    Test,
    Calculation,
    Qualification,
    Attestation,
    Observation,
    Derived,
}
/// Lifecycle state of an evidence record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceLifecycle {
    Active,
    Superseded,
    Disputed,
    Revoked,
    Withdrawn,
}

/// A reference to another artifact without embedding its payload.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArtifactRef {
    pub id: String,
    pub kind: String,
}

/// A bounded uncertainty statement.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Uncertainty {
    pub representation: String,
    pub value: Option<f64>,
    pub unit: Option<String>,
}

/// Conditions under which an evidence record is intended to apply.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ValidityDomain {
    #[serde(default)]
    pub conditions: BTreeMap<String, String>,
}

/// Explicit lifecycle transition metadata.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleEvent {
    pub state: EvidenceLifecycle,
    pub reason: Option<String>,
    pub predecessor: Option<String>,
}

/// Minimal typed AeroEvidence record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AeroEvidenceV1 {
    pub evidence_id: String,
    pub schema: String,
    pub claim: String,
    pub claim_kind: ClaimKind,
    pub configuration: ArtifactRef,
    pub subject: ArtifactRef,
    #[serde(default)]
    pub basis: Vec<ArtifactRef>,
    pub method: Option<ArtifactRef>,
    #[serde(default)]
    pub inputs: Vec<ArtifactRef>,
    #[serde(default)]
    pub toolchain: Vec<ArtifactRef>,
    pub result: serde_json::Value,
    pub uncertainty: Option<Uncertainty>,
    pub validity_domain: ValidityDomain,
    pub producer: String,
    #[serde(default)]
    pub review: Vec<ArtifactRef>,
    pub lifecycle: LifecycleEvent,
    #[serde(default)]
    pub provenance: Vec<ArtifactRef>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum EvidenceValidationError {
    #[error("schema must be AeroEvidenceV1")]
    WrongSchema,
    #[error("evidence_id must not be empty")]
    EmptyEvidenceId,
    #[error("claim must not be empty")]
    EmptyClaim,
    #[error("producer must not be empty")]
    EmptyProducer,
    #[error("configuration id must not be empty")]
    EmptyConfiguration,
    #[error("subject id must not be empty")]
    EmptySubject,
    #[error("derived evidence requires at least one basis record")]
    DerivedWithoutBasis,
    #[error("superseded evidence requires a predecessor")]
    SupersededWithoutPredecessor,
}

/// Lightweight structural validation.
///
/// This deliberately does not infer engineering validity. It only checks
/// invariants that are safe to enforce at the data-contract boundary.
pub fn validate_evidence(
    evidence: &AeroEvidenceV1,
) -> Result<(), EvidenceValidationError> {
    if evidence.schema != "AeroEvidenceV1" {
        return Err(EvidenceValidationError::WrongSchema);
    }
    if evidence.evidence_id.is_empty() {
        return Err(EvidenceValidationError::EmptyEvidenceId);
    }
    if evidence.claim.trim().is_empty() {
        return Err(EvidenceValidationError::EmptyClaim);
    }
    if evidence.producer.is_empty() {
        return Err(EvidenceValidationError::EmptyProducer);
    }
    if evidence.configuration.id.is_empty() {
        return Err(EvidenceValidationError::EmptyConfiguration);
    }
    if evidence.subject.id.is_empty() {
        return Err(EvidenceValidationError::EmptySubject);
    }
    if evidence.claim_kind == ClaimKind::Derived && evidence.basis.is_empty() {
        return Err(EvidenceValidationError::DerivedWithoutBasis);
    }
    if evidence.lifecycle.state == EvidenceLifecycle::Superseded
        && evidence.lifecycle.predecessor.is_none()
    {
        return Err(EvidenceValidationError::SupersededWithoutPredecessor);
    }
    Ok(())
}



/// Result of propagating a configuration change through one evidence record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImpactClass {
    Unaffected,
    ConditionallyValid,
    Invalidated,
    RequiresReview,
    Unknown,
}

/// A machine-readable configuration transition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChangeSetV1 {
    pub change_id: String,
    pub predecessor_configuration: ArtifactRef,
    pub proposed_configuration: ArtifactRef,
    #[serde(default)]
    pub changed_artifacts: Vec<ArtifactRef>,
    #[serde(default)]
    pub changed_inputs: Vec<ArtifactRef>,
    #[serde(default)]
    pub changed_requirements: Vec<ArtifactRef>,
    #[serde(default)]
    pub changed_methods: Vec<ArtifactRef>,
    #[serde(default)]
    pub changed_toolchains: Vec<ArtifactRef>,
}

/// The impact of a ChangeSet on one evidence record.
/// Identity class for a referenced engineering object.
///
/// These identities are deliberately distinct: an exact external payload,
/// an engineering artifact, a configuration, a source-evidence observation,
/// an execution instance, and an epistemic relation are not interchangeable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdentityKind {
    Artifact,
    Content,
    Configuration,
    SourceEvidence,
    Execution,
    EpistemicRelation,
}

/// A typed, opaque identity reference.
///
/// The identifier is intentionally not interpreted as a universal UUID or
/// digest. The identity profile belongs to the referenced system/profile.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityRef {
    pub kind: IdentityKind,
    pub id: String,
    pub profile: String,
}

/// A reference to evidence already governed by the broader Mycelix evidence
/// substrate. AeroCommons may specialize the surrounding engineering
/// relationship without creating a second evidence authority.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceRef {
    pub evidence: IdentityRef,
    pub source: Option<IdentityRef>,
}

/// Engineering assertion predicate.
///
/// These are assertions about relationships, not merely navigation edges.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssertionKind {
    DemonstratesRequirement,
    Supports,
    Contradicts,
    Reproduces,
    DependsOn,
    DerivedFrom,
    ObservedIn,
    Invalidates,
    RequiresRevalidation,
}

/// Provenance-bearing engineering relationship.
///
/// A relation is substantive evidence about the relationship between two
/// addressable objects. It must therefore carry its own identity and basis.
/// It is not equivalent to a Holochain link.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EpistemicRelationV1 {
    pub relation_id: IdentityRef,
    pub subject: IdentityRef,
    pub predicate: AssertionKind,
    pub object: IdentityRef,
    #[serde(default)]
    pub basis: Vec<EvidenceRef>,
    pub scope: Option<ValidityDomain>,
    pub effective_time: Option<String>,
    pub lifecycle: EvidenceLifecycle,
    pub predecessor: Option<IdentityRef>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum RelationValidationError {
    #[error("relation identity must have kind epistemic_relation")]
    WrongIdentityKind,
    #[error("relation identity id must not be empty")]
    EmptyRelationId,
    #[error("relation identity profile must not be empty")]
    EmptyRelationProfile,
    #[error("relation subject id must not be empty")]
    EmptySubjectId,
    #[error("relation object id must not be empty")]
    EmptyObjectId,
    #[error("reproduction or invalidation relations require basis evidence")]
    MissingBasis,
    #[error("superseded relation requires a predecessor")]
    SupersededWithoutPredecessor,
}

/// Structural validation only. This does not establish that the asserted
/// engineering relationship is physically true.
pub fn validate_epistemic_relation(
    relation: &EpistemicRelationV1,
) -> Result<(), RelationValidationError> {
    if relation.relation_id.kind != IdentityKind::EpistemicRelation {
        return Err(RelationValidationError::WrongIdentityKind);
    }
    if relation.relation_id.id.trim().is_empty() {
        return Err(RelationValidationError::EmptyRelationId);
    }
    if relation.relation_id.profile.trim().is_empty() {
        return Err(RelationValidationError::EmptyRelationProfile);
    }
    if relation.subject.id.trim().is_empty() {
        return Err(RelationValidationError::EmptySubjectId);
    }
    if relation.object.id.trim().is_empty() {
        return Err(RelationValidationError::EmptyObjectId);
    }
    if matches!(
        relation.predicate,
        AssertionKind::Reproduces | AssertionKind::Invalidates
    ) && relation.basis.is_empty()
    {
        return Err(RelationValidationError::MissingBasis);
    }
    if relation.lifecycle == EvidenceLifecycle::Superseded
        && relation.predecessor.is_none()
    {
        return Err(RelationValidationError::SupersededWithoutPredecessor);
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceImpact {
    pub evidence_id: String,
    pub class: ImpactClass,
    pub reasons: Vec<String>,
    pub obligations: Vec<String>,
}

/// Conservative impact classification.
///
/// This function intentionally prefers review/unknown over silent inheritance.
/// It is a structural propagation primitive, not an engineering or
/// certification decision.
pub fn classify_impact(
    evidence: &AeroEvidenceV1,
    change: &ChangeSetV1,
) -> EvidenceImpact {
    let mut reasons = Vec::new();
    let mut obligations = Vec::new();

    // This classifier only propagates evidence from the exact predecessor
    // configuration. Evidence from another configuration is not silently
    // imported into this transition.
    if evidence.configuration.id != change.predecessor_configuration.id {
        return EvidenceImpact {
            evidence_id: evidence.evidence_id.clone(),
            class: ImpactClass::Unknown,
            reasons: vec![
                "evidence belongs to a configuration other than the change predecessor".into(),
            ],
            obligations: vec!["configuration_alignment".into()],
        };
    }

    let subject_changed = change
        .changed_artifacts
        .iter()
        .any(|artifact| artifact.id == evidence.subject.id);
    if subject_changed {
        reasons.push("evidence subject changed".into());
        obligations.push("engineering_review".into());
        return EvidenceImpact {
            evidence_id: evidence.evidence_id.clone(),
            class: ImpactClass::RequiresReview,
            reasons,
            obligations,
        };
    }

    let input_changed = evidence.inputs.iter().any(|input| {
        change.changed_inputs.iter().any(|changed| changed.id == input.id)
    });
    if input_changed {
        reasons.push("declared evidence input changed".into());
        obligations.push("revalidation".into());
        return EvidenceImpact {
            evidence_id: evidence.evidence_id.clone(),
            class: ImpactClass::Invalidated,
            reasons,
            obligations,
        };
    }

    let method_changed = evidence.method.as_ref().is_some_and(|method| {
        change.changed_methods.iter().any(|changed| changed.id == method.id)
    });
    if method_changed {
        reasons.push("evidence method changed".into());
        obligations.push("method_review".into());
        return EvidenceImpact {
            evidence_id: evidence.evidence_id.clone(),
            class: ImpactClass::RequiresReview,
            reasons,
            obligations,
        };
    }

    let toolchain_changed = evidence.toolchain.iter().any(|tool| {
        change.changed_toolchains.iter().any(|changed| changed.id == tool.id)
    });
    if toolchain_changed {
        reasons.push("evidence toolchain changed".into());
        obligations.push("reproducibility_review".into());
        return EvidenceImpact {
            evidence_id: evidence.evidence_id.clone(),
            class: ImpactClass::RequiresReview,
            reasons,
            obligations,
        };
    }

    let requirement_changed = evidence.basis.iter().any(|basis| {
        change.changed_requirements.iter().any(|changed| changed.id == basis.id)
    });
    if requirement_changed {
        reasons.push("evidence basis requirement changed".into());
        obligations.push("requirement_review".into());
        return EvidenceImpact {
            evidence_id: evidence.evidence_id.clone(),
            class: ImpactClass::RequiresReview,
            reasons,
            obligations,
        };
    }

    if change.changed_artifacts.is_empty()
        && change.changed_inputs.is_empty()
        && change.changed_requirements.is_empty()
        && change.changed_methods.is_empty()
        && change.changed_toolchains.is_empty()
    {
        return EvidenceImpact {
            evidence_id: evidence.evidence_id.clone(),
            class: ImpactClass::Unaffected,
            reasons: vec!["no declared semantic dependencies changed".into()],
            obligations,
        };
    }

    EvidenceImpact {
        evidence_id: evidence.evidence_id.clone(),
        class: ImpactClass::Unknown,
        reasons: vec!["change dependency is not represented in evidence".into()],
        obligations: vec!["dependency_analysis".into()],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> AeroEvidenceV1 {
        AeroEvidenceV1 {
            evidence_id: "evidence-001".into(),
            schema: "AeroEvidenceV1".into(),
            claim: "measured diameter is 10.02 mm".into(),
            claim_kind: ClaimKind::Measurement,
            configuration: ArtifactRef {
                id: "fixture-v1".into(),
                kind: "configuration".into(),
            },
            subject: ArtifactRef {
                id: "fixture".into(),
                kind: "component".into(),
            },
            basis: vec![],
            method: Some(ArtifactRef {
                id: "caliper-procedure-v1".into(),
                kind: "method".into(),
            }),
            inputs: vec![],
            toolchain: vec![],
            result: serde_json::json!({"value": 10.02, "unit": "mm"}),
            uncertainty: Some(Uncertainty {
                representation: "absolute".into(),
                value: Some(0.02),
                unit: Some("mm".into()),
            }),
            validity_domain: ValidityDomain::default(),
            producer: "agent-a".into(),
            review: vec![],
            lifecycle: LifecycleEvent {
                state: EvidenceLifecycle::Active,
                reason: None,
                predecessor: None,
            },
            provenance: vec![],
        }
    }

    #[test]
    fn valid_measurement_passes_structural_validation() {
        assert_eq!(validate_evidence(&fixture()), Ok(()));
    }

    #[test]
    fn derived_evidence_requires_basis() {
        let mut value = fixture();
        value.claim_kind = ClaimKind::Derived;
        assert_eq!(
            validate_evidence(&value),
            Err(EvidenceValidationError::DerivedWithoutBasis)
        );
    }

    #[test]
    fn supersession_requires_predecessor() {
        let mut value = fixture();
        value.lifecycle.state = EvidenceLifecycle::Superseded;
        assert_eq!(
            validate_evidence(&value),
            Err(EvidenceValidationError::SupersededWithoutPredecessor)
        );
    }

    #[test]
    fn serde_round_trip_preserves_record() {
        let value = fixture();
        let encoded = serde_json::to_string(&value).expect("serialize");
        let decoded: AeroEvidenceV1 =
            serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(value, decoded);
    }

    #[test]
    fn changed_subject_requires_review() {
        let evidence = fixture();
        let change = ChangeSetV1 {
            change_id: "change-001".into(),
            predecessor_configuration: ArtifactRef {
                id: "fixture-v1".into(),
                kind: "configuration".into(),
            },
            proposed_configuration: ArtifactRef {
                id: "fixture-v2".into(),
                kind: "configuration".into(),
            },
            changed_artifacts: vec![ArtifactRef {
                id: "fixture".into(),
                kind: "component".into(),
            }],
            changed_inputs: vec![],
            changed_requirements: vec![],
            changed_methods: vec![],
            changed_toolchains: vec![],
        };
        let impact = classify_impact(&evidence, &change);
        assert_eq!(impact.class, ImpactClass::RequiresReview);
    }

    #[test]
    fn changed_declared_input_invalidates_evidence() {
        let mut evidence = fixture();
        evidence.inputs.push(ArtifactRef {
            id: "material-batch-1".into(),
            kind: "material_batch".into(),
        });
        let change = ChangeSetV1 {
            change_id: "change-002".into(),
            predecessor_configuration: ArtifactRef {
                id: "fixture-v1".into(),
                kind: "configuration".into(),
            },
            proposed_configuration: ArtifactRef {
                id: "fixture-v2".into(),
                kind: "configuration".into(),
            },
            changed_artifacts: vec![],
            changed_requirements: vec![],
            changed_methods: vec![],
            changed_toolchains: vec![],
            changed_inputs: vec![ArtifactRef {
                id: "material-batch-1".into(),
                kind: "material_batch".into(),
            }],
        };
        let impact = classify_impact(&evidence, &change);
        assert_eq!(impact.class, ImpactClass::Invalidated);
    }

    #[test]
    fn undeclared_dependency_is_unknown() {
        let evidence = fixture();
        let change = ChangeSetV1 {
            change_id: "change-003".into(),
            predecessor_configuration: ArtifactRef {
                id: "fixture-v1".into(),
                kind: "configuration".into(),
            },
            proposed_configuration: ArtifactRef {
                id: "fixture-v2".into(),
                kind: "configuration".into(),
            },
            changed_artifacts: vec![ArtifactRef {
                id: "hidden-dependency".into(),
                kind: "parameter".into(),
            }],
            changed_inputs: vec![],
            changed_requirements: vec![],
            changed_methods: vec![],
            changed_toolchains: vec![],
        };
        let impact = classify_impact(&evidence, &change);
        assert_eq!(impact.class, ImpactClass::Unknown);
    }


    #[test]
    fn epistemic_relation_requires_own_identity() {
        let relation = EpistemicRelationV1 {
            relation_id: IdentityRef { kind: IdentityKind::EpistemicRelation, id: "rel-1".into(), profile: "aero-relation-v1".into() },
            subject: IdentityRef { kind: IdentityKind::Artifact, id: "inspection-1".into(), profile: "artifact-v1".into() },
            predicate: AssertionKind::DemonstratesRequirement,
            object: IdentityRef { kind: IdentityKind::Artifact, id: "requirement-7".into(), profile: "requirement-v1".into() },
            basis: vec![EvidenceRef {
                evidence: IdentityRef { kind: IdentityKind::SourceEvidence, id: "evidence-1".into(), profile: "myc-evidence-v1".into() },
                source: None,
            }],
            scope: None,
            effective_time: None,
            lifecycle: EvidenceLifecycle::Active,
            predecessor: None,
        };
        assert_eq!(validate_epistemic_relation(&relation), Ok(()));
    }

    #[test]
    fn reproduction_relation_requires_basis() {
        let relation = EpistemicRelationV1 {
            relation_id: IdentityRef { kind: IdentityKind::EpistemicRelation, id: "rel-2".into(), profile: "aero-relation-v1".into() },
            subject: IdentityRef { kind: IdentityKind::Execution, id: "execution-2".into(), profile: "execution-v1".into() },
            predicate: AssertionKind::Reproduces,
            object: IdentityRef { kind: IdentityKind::SourceEvidence, id: "evidence-1".into(), profile: "myc-evidence-v1".into() },
            basis: vec![],
            scope: None,
            effective_time: None,
            lifecycle: EvidenceLifecycle::Active,
            predecessor: None,
        };
        assert_eq!(validate_epistemic_relation(&relation), Err(RelationValidationError::MissingBasis));
    }

    #[test]
    fn empty_semantic_change_is_unaffected() {
        let evidence = fixture();
        let change = ChangeSetV1 {
            change_id: "change-004".into(),
            predecessor_configuration: ArtifactRef {
                id: "fixture-v1".into(),
                kind: "configuration".into(),
            },
            proposed_configuration: ArtifactRef {
                id: "fixture-v1".into(),
                kind: "configuration".into(),
            },
            changed_artifacts: vec![],
            changed_inputs: vec![],
            changed_requirements: vec![],
            changed_methods: vec![],
            changed_toolchains: vec![],
        };
        let impact = classify_impact(&evidence, &change);
        assert_eq!(impact.class, ImpactClass::Unaffected);
    }

}

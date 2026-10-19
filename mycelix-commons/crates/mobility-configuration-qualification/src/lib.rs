use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fs;

/// Typed identity/lineage semantics for the mobility evidence substrate.
pub mod identity_lineage;
pub mod temporal_applicability;
pub mod temporal_reconciliation;

/// Typed identity/lineage semantics for the mobility evidence substrate.
///
/// Kept in a dedicated module so identity does not collapse into relationship
/// scope strings or Holochain protocol metadata.
pub mod identity_lineage;
pub mod temporal_applicability;

const EXPECTED_COUNT: usize = 20;
const EXPECTED_PREFIX: &str = "MC-CONFIG-";

const EXPECTED_OUTCOMES: &[&str] = &[
    "distinct-physical-artifacts",
    "manufacturing-lineage-preserved",
    "substitution-lineage-and-revalidation",
    "historical-evidence-preserved",
    "prediction-observation-remain-distinct",
    "negative-evidence-preserved",
    "unknown-or-review-required",
    "controlled-payload-with-public-provenance",
    "explicit-binding-or-rejection",
    "reject-semantic-substitution",
    "observation-separate-from-interpretation",
    "repair-history-and-resulting-state-preserved",
    "external-authority-remains-attributable",
    "metadata-remains-descriptive",
    "interface-dependencies-explicit",
    "historical-predecessor-preserved",
    "obligation-remains-open",
    "shared-core-profile-specific-divergence",
    "dependency-aware-change-lineage",
    "historical-artifact-not-active",
];

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Corpus {
    pub schema_version: String,
    pub status: String,
    pub vectors: Vec<Vector>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Vector {
    pub id: String,
    pub scenario: String,
    pub expected_outcome: String,
    pub forbidden_inference: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationResult {
    Valid,
    Invalid(String),
}

/// Compatibility vocabulary for the original seven-state outcome algebra.
///
/// New evidence records should use EvidenceState instead, because these
/// seven concepts belong to different semantic dimensions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvidenceOutcomeState {
    Supported,
    Contradicted,
    Unresolved,
    Indeterminate,
    Superseded,
    Disputed,
    ExternallyAuthoritative,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EpistemicDisposition {
    Supported,
    Contradicted,
    Unresolved,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LifecycleDisposition {
    Current,
    Superseded,
    Retired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConflictDisposition {
    Uncontested,
    Disputed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityProvenance {
    Commons,
    External,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceModality {
    Observation,
    Measurement,
    Prediction,
    Simulation,
    Interpretation,
    Attestation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceNodeKind {
    Requirement,
    DesignArtifact,
    PhysicalArtifact,
    ManufacturingEvent,
    InspectionRecord,
    TestRecord,
    OperationalObservation,
    MaintenanceEvent,
    EvidenceRecord,
    ChangeSet,
    ImpactAssessment,
    RevalidationObligation,
    ExternalAuthorityReference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceRelation {
    DerivedFrom,
    ManufacturedAs,
    InspectedAs,
    TestedAs,
    ObservedAs,
    Interprets,
    Supersedes,
    Changes,
    RequiresRevalidation,
    Disputes,
    Authorizes,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceRelationship {
    pub relation: EvidenceRelation,
    pub source_kind: EvidenceNodeKind,
    pub target_kind: EvidenceNodeKind,
    /// Stable engineering/configuration scope identifiers. These are semantic
    /// bindings, not Holochain action timestamps.
    pub source_scope_ref: String,
    pub target_scope_ref: String,
}

impl EvidenceRelationship {
    /// Validate relationship typing only. A valid edge never implies a
    /// different edge type, physical equivalence, safety, or certification.
    pub fn validate(&self) -> Result<(), String> {
        if self.source_scope_ref.trim().is_empty() || self.target_scope_ref.trim().is_empty() {
            return Err("relationship requires explicit source and target scope references".into());
        }

        let same_scope = self.source_scope_ref == self.target_scope_ref;
        let valid = match self.relation {
            EvidenceRelation::DerivedFrom => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::DesignArtifact, EvidenceNodeKind::Requirement)
                    | (EvidenceNodeKind::EvidenceRecord, EvidenceNodeKind::DesignArtifact)
                    | (EvidenceNodeKind::ChangeSet, EvidenceNodeKind::EvidenceRecord)
            ),
            EvidenceRelation::ManufacturedAs => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::ManufacturingEvent, EvidenceNodeKind::PhysicalArtifact)
            ),
            EvidenceRelation::InspectedAs => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::InspectionRecord, EvidenceNodeKind::PhysicalArtifact)
            ),
            EvidenceRelation::TestedAs => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::TestRecord, EvidenceNodeKind::PhysicalArtifact)
            ),
            EvidenceRelation::ObservedAs => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::OperationalObservation, EvidenceNodeKind::PhysicalArtifact)
            ),
            EvidenceRelation::Interprets => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::EvidenceRecord, EvidenceNodeKind::InspectionRecord)
                    | (EvidenceNodeKind::EvidenceRecord, EvidenceNodeKind::TestRecord)
                    | (EvidenceNodeKind::EvidenceRecord, EvidenceNodeKind::OperationalObservation)
            ),
            EvidenceRelation::Supersedes => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::DesignArtifact, EvidenceNodeKind::DesignArtifact)
                    | (EvidenceNodeKind::EvidenceRecord, EvidenceNodeKind::EvidenceRecord)
                    | (EvidenceNodeKind::ChangeSet, EvidenceNodeKind::ChangeSet)
            ),
            EvidenceRelation::Changes => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::ChangeSet, EvidenceNodeKind::DesignArtifact)
                    | (EvidenceNodeKind::ChangeSet, EvidenceNodeKind::PhysicalArtifact)
                    | (EvidenceNodeKind::ChangeSet, EvidenceNodeKind::EvidenceRecord)
            ),
            EvidenceRelation::RequiresRevalidation => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::ImpactAssessment, EvidenceNodeKind::RevalidationObligation)
                    | (EvidenceNodeKind::ChangeSet, EvidenceNodeKind::RevalidationObligation)
            ),
            EvidenceRelation::Disputes => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::EvidenceRecord, EvidenceNodeKind::EvidenceRecord)
            ),
            EvidenceRelation::Authorizes => matches!(
                (self.source_kind, self.target_kind),
                (EvidenceNodeKind::ExternalAuthorityReference, EvidenceNodeKind::EvidenceRecord)
                    | (EvidenceNodeKind::ExternalAuthorityReference, EvidenceNodeKind::PhysicalArtifact)
            ),
        };
        let scope_valid = match self.relation {
            EvidenceRelation::Supersedes | EvidenceRelation::Changes => true,
            _ => same_scope,
        };
        if valid && scope_valid { Ok(()) } else {
            Err(format!("invalid source/target kinds for {:?}", self.relation))
        }
    }
}

/// Orthogonal evidence state. The dimensions intentionally do not collapse
/// into one status enum, preventing semantic/product-enum explosion.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceState {
    pub epistemic_disposition: EpistemicDisposition,
    pub lifecycle_disposition: LifecycleDisposition,
    pub conflict_disposition: ConflictDisposition,
    pub authority_provenance: AuthorityProvenance,
    pub evidence_modality: EvidenceModality,
    pub contradiction_reference: Option<String>,
    pub conflict_reference: Option<String>,
    pub unresolved_dependency_reference: Option<String>,
    pub external_authority_reference: Option<String>,
}

impl EvidenceState {
    /// Validate only semantic/provenance consistency. This is not a safety,
    /// certification, regulatory, or physical-engineering assessment.
    pub fn validate(&self) -> Result<(), String> {
        if self.epistemic_disposition == EpistemicDisposition::Contradicted
            && self.contradiction_reference.is_none()
        {
            return Err("contradicted evidence requires a contradiction reference".into());
        }

        if self.epistemic_disposition == EpistemicDisposition::Unresolved
            && self.unresolved_dependency_reference.is_none()
        {
            return Err("unresolved evidence requires a dependency reference".into());
        }

        if self.conflict_disposition == ConflictDisposition::Disputed
            && self.conflict_reference.is_none()
        {
            return Err("disputed evidence requires a conflict reference".into());
        }

        match self.authority_provenance {
            AuthorityProvenance::External if self.external_authority_reference.is_none() => {
                Err("external authority requires an external-authority reference".into())
            }
            AuthorityProvenance::Commons if self.external_authority_reference.is_some() => {
                Err("commons authority cannot carry an external-authority reference".into())
            }
            _ => Ok(()),
        }
    }
}

pub fn parse_corpus(json: &str) -> Result<Corpus, String> {
    serde_json::from_str(json).map_err(|e| format!("invalid qualification corpus: {e}"))
}

pub fn qualify(corpus: &Corpus) -> QualificationResult {
    if corpus.schema_version != "mobility-configuration-contract-qualification-v1" {
        return QualificationResult::Invalid("unexpected schema version".into());
    }
    if corpus.status != "semantic-qualification-only" {
        return QualificationResult::Invalid("qualification corpus has unsafe status".into());
    }
    if corpus.vectors.len() != EXPECTED_COUNT {
        return QualificationResult::Invalid(format!(
            "expected {EXPECTED_COUNT} vectors, found {}",
            corpus.vectors.len()
        ));
    }

    let ids: BTreeSet<_> = corpus.vectors.iter().map(|v| v.id.as_str()).collect();
    let expected_ids: BTreeSet<_> = (1..=EXPECTED_COUNT)
        .map(|n| format!("{EXPECTED_PREFIX}{n:03}"))
        .collect();

    if ids.len() != EXPECTED_COUNT || ids != expected_ids {
        return QualificationResult::Invalid(
            "vector identifiers are incomplete or duplicated".into(),
        );
    }

    for vector in &corpus.vectors {
        if !vector.id.starts_with(EXPECTED_PREFIX)
            || vector.scenario.trim().is_empty()
            || vector.expected_outcome.trim().is_empty()
            || vector.forbidden_inference.trim().is_empty()
        {
            return QualificationResult::Invalid(format!(
                "{} is missing required semantic fields",
                vector.id
            ));
        }
        if !EXPECTED_OUTCOMES.contains(&vector.expected_outcome.as_str()) {
            return QualificationResult::Invalid(format!(
                "{} has an unrecognized expected outcome: {}",
                vector.id, vector.expected_outcome
            ));
        }
    }

    QualificationResult::Valid
}

pub fn load_and_qualify(path: &str) -> Result<(), String> {
    let json = fs::read_to_string(path).map_err(|e| format!("failed to read {path}: {e}"))?;
    let corpus = parse_corpus(&json)?;
    match qualify(&corpus) {
        QualificationResult::Valid => Ok(()),
        QualificationResult::Invalid(reason) => Err(reason),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corpus() -> Corpus {
        parse_corpus(include_str!("../../../../docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1.json"))
            .expect("bundled corpus must parse")
    }

    fn valid_state() -> EvidenceState {
        EvidenceState {
            epistemic_disposition: EpistemicDisposition::Supported,
            lifecycle_disposition: LifecycleDisposition::Current,
            conflict_disposition: ConflictDisposition::Uncontested,
            authority_provenance: AuthorityProvenance::Commons,
            evidence_modality: EvidenceModality::Measurement,
            contradiction_reference: None,
            conflict_reference: None,
            unresolved_dependency_reference: None,
            external_authority_reference: None,
        }
    }

    #[test]
    fn corpus_has_exactly_twenty_vectors() {
        assert_eq!(corpus().vectors.len(), 20);
    }

    #[test]
    fn corpus_has_exact_identifier_set() {
        assert_eq!(qualify(&corpus()), QualificationResult::Valid);
    }

    #[test]
    fn machine_corpus_ids_are_present_in_source_vector_document() {
        let prose = include_str!(
            "../../../../docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1_TEST_VECTORS.md"
        );
        for vector in corpus().vectors {
            assert!(
                prose.contains(&vector.id),
                "{} is missing from the source qualification document",
                vector.id
            );
        }
    }

    #[test]
    fn every_vector_preserves_a_forbidden_inference_boundary() {
        assert!(corpus()
            .vectors
            .iter()
            .all(|v| !v.forbidden_inference.trim().is_empty()));
    }

    #[test]
    fn every_vector_has_a_qualified_outcome_rule() {
        assert!(corpus()
            .vectors
            .iter()
            .all(|v| EXPECTED_OUTCOMES.contains(&v.expected_outcome.as_str())));
    }

    #[test]
    fn duplicate_vector_ids_are_rejected() {
        let mut c = corpus();
        c.vectors[1].id = c.vectors[0].id.clone();
        assert!(matches!(qualify(&c), QualificationResult::Invalid(_)));
    }

    #[test]
    fn missing_vector_is_rejected() {
        let mut c = corpus();
        c.vectors.pop();
        assert!(matches!(qualify(&c), QualificationResult::Invalid(_)));
    }

    #[test]
    fn unknown_outcome_is_rejected() {
        let mut c = corpus();
        c.vectors[0].expected_outcome = "unsafe-universal-safety-score".into();
        assert!(matches!(qualify(&c), QualificationResult::Invalid(_)));
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let mut value: serde_json::Value =
            serde_json::from_str(include_str!(
                "../../../../docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1.json"
            ))
            .expect("corpus must parse as JSON");
        value["unexpected"] = serde_json::json!("must-fail");
        assert!(parse_corpus(&value.to_string()).is_err());
    }

    #[test]
    fn wrong_schema_is_rejected() {
        let mut c = corpus();
        c.schema_version = "other".into();
        assert!(matches!(qualify(&c), QualificationResult::Invalid(_)));
    }

    #[test]
    fn outcome_algebra_has_explicit_unresolved_state() {
        assert_ne!(EvidenceOutcomeState::Unresolved, EvidenceOutcomeState::Contradicted);
        assert_eq!(EvidenceOutcomeState::Unresolved, EvidenceOutcomeState::Unresolved);
    }

    #[test]
    fn outcome_algebra_preserves_historical_and_disputed_states() {
        assert_ne!(EvidenceOutcomeState::Superseded, EvidenceOutcomeState::Supported);
        assert_ne!(EvidenceOutcomeState::Disputed, EvidenceOutcomeState::Supported);
    }

    #[test]
    fn authority_boundary_is_not_a_safety_claim() {
        let c = corpus();
        assert_eq!(c.status, "semantic-qualification-only");
        assert!(c.vectors.iter().any(|v| v.id == "MC-CONFIG-013"));
        assert!(c.vectors.iter().any(|v| v.id == "MC-CONFIG-017"));
    }

    #[test]
    fn supported_measurement_is_a_valid_current_commons_state() {
        assert!(valid_state().validate().is_ok());
    }

    #[test]
    fn unresolved_is_not_contradicted_and_requires_dependency() {
        let mut state = valid_state();
        state.epistemic_disposition = EpistemicDisposition::Unresolved;
        assert!(state.validate().is_err());

        state.unresolved_dependency_reference = Some("dep-1".into());
        assert!(state.validate().is_ok());
        assert_ne!(EpistemicDisposition::Unresolved, EpistemicDisposition::Contradicted);
    }

    #[test]
    fn contradicted_requires_explicit_conflict_reference() {
        let mut state = valid_state();
        state.epistemic_disposition = EpistemicDisposition::Contradicted;
        assert!(state.validate().is_err());

        state.contradiction_reference = Some("evidence-2".into());
        assert!(state.validate().is_ok());
    }

    #[test]
    fn supported_evidence_can_be_disputed_without_becoming_contradicted() {
        let mut state = valid_state();
        state.conflict_disposition = ConflictDisposition::Disputed;
        assert!(state.validate().is_err());

        state.conflict_reference = Some("claim-2".into());
        assert!(state.validate().is_ok());
        assert_eq!(state.epistemic_disposition, EpistemicDisposition::Supported);
        assert!(state.contradiction_reference.is_none());
        assert!(state.conflict_reference.is_some());
    }

    #[test]
    fn contradiction_and_dispute_use_distinct_references() {
        let mut contradiction = valid_state();
        contradiction.epistemic_disposition = EpistemicDisposition::Contradicted;
        contradiction.contradiction_reference = Some("evidence-2".into());
        assert!(contradiction.validate().is_ok());
        assert!(contradiction.conflict_reference.is_none());

        let mut dispute = valid_state();
        dispute.conflict_disposition = ConflictDisposition::Disputed;
        dispute.conflict_reference = Some("claim-2".into());
        assert!(dispute.validate().is_ok());
        assert!(dispute.contradiction_reference.is_none());
    }

    #[test]
    fn relationship_typing_accepts_declared_edges() {
        let relationship = EvidenceRelationship {
            relation: EvidenceRelation::ManufacturedAs,
            source_kind: EvidenceNodeKind::ManufacturingEvent,
            target_kind: EvidenceNodeKind::PhysicalArtifact,
            source_scope_ref: "cfg-a".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(relationship.validate().is_ok());
    }

    #[test]
    fn relationship_typing_rejects_semantic_substitution() {
        let relationship = EvidenceRelationship {
            relation: EvidenceRelation::DerivedFrom,
            source_kind: EvidenceNodeKind::ManufacturingEvent,
            target_kind: EvidenceNodeKind::PhysicalArtifact,
            source_scope_ref: "cfg-a".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(relationship.validate().is_err());
    }

    #[test]
    fn interpretation_does_not_become_observation() {
        let interpretation = EvidenceRelationship {
            relation: EvidenceRelation::Interprets,
            source_kind: EvidenceNodeKind::EvidenceRecord,
            target_kind: EvidenceNodeKind::OperationalObservation,
            source_scope_ref: "cfg-a".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(interpretation.validate().is_ok());
        let observation = EvidenceRelationship {
            relation: EvidenceRelation::ObservedAs,
            source_kind: EvidenceNodeKind::EvidenceRecord,
            target_kind: EvidenceNodeKind::PhysicalArtifact,
            source_scope_ref: "cfg-a".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(observation.validate().is_err());
    }

    #[test]
    fn authorization_is_external_authority_only() {
        let external = EvidenceRelationship {
            relation: EvidenceRelation::Authorizes,
            source_kind: EvidenceNodeKind::ExternalAuthorityReference,
            target_kind: EvidenceNodeKind::EvidenceRecord,
            source_scope_ref: "authority-scope".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(external.validate().is_err());
        let external_same_scope = EvidenceRelationship {
            relation: EvidenceRelation::Authorizes,
            source_kind: EvidenceNodeKind::ExternalAuthorityReference,
            target_kind: EvidenceNodeKind::EvidenceRecord,
            source_scope_ref: "cfg-a".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(external_same_scope.validate().is_ok());
        let commons = EvidenceRelationship {
            relation: EvidenceRelation::Authorizes,
            source_kind: EvidenceNodeKind::EvidenceRecord,
            target_kind: EvidenceNodeKind::EvidenceRecord,
            source_scope_ref: "cfg-a".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(commons.validate().is_err());
    }


    #[test]
    fn ordinary_relationships_cannot_cross_configuration_scope() {
        let relationship = EvidenceRelationship {
            relation: EvidenceRelation::InspectedAs,
            source_kind: EvidenceNodeKind::InspectionRecord,
            target_kind: EvidenceNodeKind::PhysicalArtifact,
            source_scope_ref: "cfg-a".into(),
            target_scope_ref: "cfg-b".into(),
        };
        assert!(relationship.validate().is_err());
    }

    #[test]
    fn supersession_can_explicitly_cross_configuration_scope() {
        let relationship = EvidenceRelationship {
            relation: EvidenceRelation::Supersedes,
            source_kind: EvidenceNodeKind::DesignArtifact,
            target_kind: EvidenceNodeKind::DesignArtifact,
            source_scope_ref: "cfg-b".into(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(relationship.validate().is_ok());
    }

    #[test]
    fn missing_scope_reference_is_fail_closed() {
        let relationship = EvidenceRelationship {
            relation: EvidenceRelation::TestedAs,
            source_kind: EvidenceNodeKind::TestRecord,
            target_kind: EvidenceNodeKind::PhysicalArtifact,
            source_scope_ref: String::new(),
            target_scope_ref: "cfg-a".into(),
        };
        assert!(relationship.validate().is_err());
    }
    #[test]
    fn external_authority_requires_explicit_reference() {
        let mut state = valid_state();
        state.authority_provenance = AuthorityProvenance::External;
        assert!(state.validate().is_err());

        state.external_authority_reference = Some("authority-1".into());
        assert!(state.validate().is_ok());
    }

    #[test]
    fn commons_cannot_silently_claim_external_authority() {
        let mut state = valid_state();
        state.external_authority_reference = Some("authority-1".into());
        assert!(state.validate().is_err());
    }

    #[test]
    fn modality_remains_independent_from_epistemic_disposition() {
        let mut state = valid_state();
        state.evidence_modality = EvidenceModality::Simulation;
        assert!(state.validate().is_ok());
        assert_eq!(state.epistemic_disposition, EpistemicDisposition::Supported);
    }

    #[test]
    fn superseded_and_retired_are_lifecycle_states_not_erasure() {
        let mut state = valid_state();
        state.lifecycle_disposition = LifecycleDisposition::Superseded;
        assert!(state.validate().is_ok());
        state.lifecycle_disposition = LifecycleDisposition::Retired;
        assert!(state.validate().is_ok());
    }
}

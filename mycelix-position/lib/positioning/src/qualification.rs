// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Machine-readable qualification theorem registry and requirement evaluation.
//!
//! This module deliberately carries **no positioning authority by itself**.
//! It models the meta-structure needed to keep independent qualification
//! theorems from collapsing into a scalar quality score or one `qualified`
//! boolean.
//!
//! The registry contains mathematical/semantic prerequisite edges only.
//! Consumer policy belongs in [`QualificationRequirementProfile`] and must not
//! be smuggled into theorem dependencies.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Stable, versioned identifier for one qualification theorem.
///
/// The registry validates identifiers as non-empty strings but intentionally
/// does not prescribe a naming scheme yet. Once a theorem identifier is used
/// in durable evidence, its semantics should be treated as immutable.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct TheoremId(pub String);

impl TheoremId {
    pub fn new(value: impl Into<String>) -> Self {
        Self(value.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for TheoremId {
    fn from(value: &str) -> Self {
        Self::new(value)
    }
}

impl From<String> for TheoremId {
    fn from(value: String) -> Self {
        Self::new(value)
    }
}

/// Definition of one theorem and the theorem prerequisites that must already
/// be established before this theorem may itself be recorded as established.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TheoremDefinition {
    pub id: TheoremId,
    #[serde(default)]
    pub prerequisites: Vec<TheoremId>,
}

impl TheoremDefinition {
    pub fn new(id: impl Into<TheoremId>, prerequisites: Vec<TheoremId>) -> Self {
        Self {
            id: id.into(),
            prerequisites,
        }
    }
}

/// Status of one theorem evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum FacetStatus {
    Established,
    NotEstablished,
    Failed,
    Indeterminate,
    NotApplicable,
    Expired,
    Superseded,
}

/// One evidence-bearing theorem facet for a specific subject commitment.
///
/// Evidence references and dependency commitments remain opaque strings here;
/// the surrounding evidence layer owns their concrete addressing scheme.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationFacet {
    pub theorem_id: TheoremId,
    pub subject_commitment: String,
    pub status: FacetStatus,
    #[serde(default)]
    pub verifier_or_profile: Option<String>,
    #[serde(default)]
    pub evidence_refs: Vec<String>,
    #[serde(default)]
    pub dependency_commitments: Vec<String>,
    #[serde(default)]
    pub diagnostics_commitment: Option<String>,
}

/// Qualification facets carried for one exact subject commitment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationManifest {
    pub subject_commitment: String,
    pub facets: Vec<QualificationFacet>,
}

impl QualificationManifest {
    /// Return theorem IDs in canonical lexical order without mutating the
    /// original evidence object.
    pub fn canonical_theorem_order(&self) -> Vec<TheoremId> {
        let mut ids: Vec<_> = self.facets.iter().map(|facet| facet.theorem_id.clone()).collect();
        ids.sort();
        ids
    }
}

/// Consumer-specific requirement profile.
///
/// This is intentionally separate from theorem prerequisites: a theorem DAG
/// states what a theorem *depends on*, while this profile states what a
/// consumer *requires* for one use.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationRequirementProfile {
    pub profile_id: String,
    pub required_theorems: Vec<TheoremId>,
    pub accepted_statuses: Vec<FacetStatus>,
}

impl QualificationRequirementProfile {
    /// Common strict profile: every required theorem must be `Established`.
    pub fn established_only(
        profile_id: impl Into<String>,
        required_theorems: Vec<TheoremId>,
    ) -> Self {
        Self {
            profile_id: profile_id.into(),
            required_theorems,
            accepted_statuses: vec![FacetStatus::Established],
        }
    }
}

/// One required theorem that was present but had an unacceptable status.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct UnacceptableFacet {
    pub theorem_id: TheoremId,
    pub status: FacetStatus,
}

/// Deterministic result of evaluating one manifest against one requirement
/// profile.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RequirementEvaluation {
    pub profile_id: String,
    pub subject_commitment: String,
    pub satisfied: bool,
    pub missing_theorems: Vec<TheoremId>,
    pub unacceptable_facets: Vec<UnacceptableFacet>,
}

/// Structural errors in the theorem registry, manifest, or requirement
/// profile. These are intentionally distinct from a legitimate
/// `RequirementEvaluation { satisfied: false, .. }` result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationError {
    EmptyTheoremId,
    EmptyProfileId,
    EmptySubjectCommitment,
    DuplicateTheorem { theorem_id: TheoremId },
    DuplicatePrerequisite {
        theorem_id: TheoremId,
        prerequisite: TheoremId,
    },
    UnknownPrerequisite {
        theorem_id: TheoremId,
        prerequisite: TheoremId,
    },
    DependencyCycle { theorem_id: TheoremId },
    DuplicateFacet { theorem_id: TheoremId },
    UnknownFacetTheorem { theorem_id: TheoremId },
    FacetSubjectMismatch { theorem_id: TheoremId },
    EstablishedFacetMissingPrerequisite {
        theorem_id: TheoremId,
        prerequisite: TheoremId,
    },
    DuplicateRequiredTheorem { theorem_id: TheoremId },
    UnknownRequiredTheorem { theorem_id: TheoremId },
    EmptyAcceptedStatuses,
}

impl std::fmt::Display for QualificationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyTheoremId => write!(f, "theorem identifier must not be empty"),
            Self::EmptyProfileId => write!(f, "requirement profile identifier must not be empty"),
            Self::EmptySubjectCommitment => write!(f, "subject commitment must not be empty"),
            Self::DuplicateTheorem { theorem_id } => {
                write!(f, "duplicate theorem definition: {}", theorem_id.as_str())
            }
            Self::DuplicatePrerequisite {
                theorem_id,
                prerequisite,
            } => write!(
                f,
                "theorem {} repeats prerequisite {}",
                theorem_id.as_str(),
                prerequisite.as_str()
            ),
            Self::UnknownPrerequisite {
                theorem_id,
                prerequisite,
            } => write!(
                f,
                "theorem {} references unknown prerequisite {}",
                theorem_id.as_str(),
                prerequisite.as_str()
            ),
            Self::DependencyCycle { theorem_id } => write!(
                f,
                "qualification theorem dependency cycle includes {}",
                theorem_id.as_str()
            ),
            Self::DuplicateFacet { theorem_id } => {
                write!(f, "duplicate qualification facet: {}", theorem_id.as_str())
            }
            Self::UnknownFacetTheorem { theorem_id } => write!(
                f,
                "manifest references unknown theorem {}",
                theorem_id.as_str()
            ),
            Self::FacetSubjectMismatch { theorem_id } => write!(
                f,
                "facet {} does not reference the manifest subject",
                theorem_id.as_str()
            ),
            Self::EstablishedFacetMissingPrerequisite {
                theorem_id,
                prerequisite,
            } => write!(
                f,
                "established facet {} lacks established prerequisite {}",
                theorem_id.as_str(),
                prerequisite.as_str()
            ),
            Self::DuplicateRequiredTheorem { theorem_id } => write!(
                f,
                "requirement profile repeats theorem {}",
                theorem_id.as_str()
            ),
            Self::UnknownRequiredTheorem { theorem_id } => write!(
                f,
                "requirement profile references unknown theorem {}",
                theorem_id.as_str()
            ),
            Self::EmptyAcceptedStatuses => {
                write!(f, "requirement profile must accept at least one status")
            }
        }
    }
}

impl std::error::Error for QualificationError {}

/// Validated acyclic theorem dependency registry.
#[derive(Debug, Clone)]
pub struct TheoremRegistry {
    definitions: BTreeMap<TheoremId, TheoremDefinition>,
}

impl TheoremRegistry {
    pub fn new(definitions: Vec<TheoremDefinition>) -> Result<Self, QualificationError> {
        let mut by_id = BTreeMap::new();

        for definition in definitions {
            if definition.id.as_str().trim().is_empty() {
                return Err(QualificationError::EmptyTheoremId);
            }
            if by_id.contains_key(&definition.id) {
                return Err(QualificationError::DuplicateTheorem {
                    theorem_id: definition.id,
                });
            }
            by_id.insert(definition.id.clone(), definition);
        }

        for definition in by_id.values() {
            let mut prerequisites = BTreeSet::new();
            for prerequisite in &definition.prerequisites {
                if prerequisite.as_str().trim().is_empty() {
                    return Err(QualificationError::EmptyTheoremId);
                }
                if !prerequisites.insert(prerequisite.clone()) {
                    return Err(QualificationError::DuplicatePrerequisite {
                        theorem_id: definition.id.clone(),
                        prerequisite: prerequisite.clone(),
                    });
                }
                if !by_id.contains_key(prerequisite) {
                    return Err(QualificationError::UnknownPrerequisite {
                        theorem_id: definition.id.clone(),
                        prerequisite: prerequisite.clone(),
                    });
                }
            }
        }

        let registry = Self { definitions: by_id };
        registry.validate_acyclic()?;
        Ok(registry)
    }

    pub fn contains(&self, theorem_id: &TheoremId) -> bool {
        self.definitions.contains_key(theorem_id)
    }

    pub fn definition(&self, theorem_id: &TheoremId) -> Option<&TheoremDefinition> {
        self.definitions.get(theorem_id)
    }

    /// Definitions are yielded in canonical theorem-ID order.
    pub fn definitions(&self) -> impl Iterator<Item = &TheoremDefinition> {
        self.definitions.values()
    }

    fn validate_acyclic(&self) -> Result<(), QualificationError> {
        // 0 = unseen, 1 = visiting, 2 = complete.
        let mut state: BTreeMap<TheoremId, u8> = self
            .definitions
            .keys()
            .cloned()
            .map(|id| (id, 0))
            .collect();

        for theorem_id in self.definitions.keys() {
            self.visit(theorem_id, &mut state)?;
        }
        Ok(())
    }

    fn visit(
        &self,
        theorem_id: &TheoremId,
        state: &mut BTreeMap<TheoremId, u8>,
    ) -> Result<(), QualificationError> {
        match state.get(theorem_id).copied().unwrap_or_default() {
            2 => return Ok(()),
            1 => {
                return Err(QualificationError::DependencyCycle {
                    theorem_id: theorem_id.clone(),
                })
            }
            _ => {}
        }

        state.insert(theorem_id.clone(), 1);
        if let Some(definition) = self.definitions.get(theorem_id) {
            for prerequisite in &definition.prerequisites {
                self.visit(prerequisite, state)?;
            }
        }
        state.insert(theorem_id.clone(), 2);
        Ok(())
    }

    /// Validate structural consistency of one manifest.
    ///
    /// In particular, an `Established` child theorem cannot launder a missing,
    /// failed, indeterminate, expired, or otherwise non-established
    /// prerequisite into stronger semantics.
    pub fn validate_manifest(
        &self,
        manifest: &QualificationManifest,
    ) -> Result<(), QualificationError> {
        if manifest.subject_commitment.trim().is_empty() {
            return Err(QualificationError::EmptySubjectCommitment);
        }

        let mut facets = BTreeMap::new();
        for facet in &manifest.facets {
            if !self.contains(&facet.theorem_id) {
                return Err(QualificationError::UnknownFacetTheorem {
                    theorem_id: facet.theorem_id.clone(),
                });
            }
            if facet.subject_commitment != manifest.subject_commitment {
                return Err(QualificationError::FacetSubjectMismatch {
                    theorem_id: facet.theorem_id.clone(),
                });
            }
            if facets.insert(facet.theorem_id.clone(), facet).is_some() {
                return Err(QualificationError::DuplicateFacet {
                    theorem_id: facet.theorem_id.clone(),
                });
            }
        }

        for facet in facets.values() {
            if facet.status != FacetStatus::Established {
                continue;
            }
            let definition = self
                .definitions
                .get(&facet.theorem_id)
                .expect("manifest theorem existence checked above");
            for prerequisite in &definition.prerequisites {
                let prerequisite_established = facets
                    .get(prerequisite)
                    .is_some_and(|candidate| candidate.status == FacetStatus::Established);
                if !prerequisite_established {
                    return Err(QualificationError::EstablishedFacetMissingPrerequisite {
                        theorem_id: facet.theorem_id.clone(),
                        prerequisite: prerequisite.clone(),
                    });
                }
            }
        }

        Ok(())
    }

    pub fn validate_requirement_profile(
        &self,
        profile: &QualificationRequirementProfile,
    ) -> Result<(), QualificationError> {
        if profile.profile_id.trim().is_empty() {
            return Err(QualificationError::EmptyProfileId);
        }
        if profile.accepted_statuses.is_empty() {
            return Err(QualificationError::EmptyAcceptedStatuses);
        }

        let mut seen = BTreeSet::new();
        for theorem_id in &profile.required_theorems {
            if !self.contains(theorem_id) {
                return Err(QualificationError::UnknownRequiredTheorem {
                    theorem_id: theorem_id.clone(),
                });
            }
            if !seen.insert(theorem_id.clone()) {
                return Err(QualificationError::DuplicateRequiredTheorem {
                    theorem_id: theorem_id.clone(),
                });
            }
        }
        Ok(())
    }

    /// Evaluate a structurally valid manifest against a consumer requirement
    /// profile. Missing/unacceptable requirements are ordinary negative
    /// results, not structural errors.
    pub fn evaluate(
        &self,
        manifest: &QualificationManifest,
        profile: &QualificationRequirementProfile,
    ) -> Result<RequirementEvaluation, QualificationError> {
        self.validate_manifest(manifest)?;
        self.validate_requirement_profile(profile)?;

        let facet_statuses: BTreeMap<_, _> = manifest
            .facets
            .iter()
            .map(|facet| (facet.theorem_id.clone(), facet.status))
            .collect();

        let accepted: BTreeSet<_> = profile.accepted_statuses.iter().copied().collect();
        let mut required = profile.required_theorems.clone();
        required.sort();

        let mut missing_theorems = Vec::new();
        let mut unacceptable_facets = Vec::new();

        for theorem_id in required {
            match facet_statuses.get(&theorem_id).copied() {
                None => missing_theorems.push(theorem_id),
                Some(status) if !accepted.contains(&status) => {
                    unacceptable_facets.push(UnacceptableFacet { theorem_id, status });
                }
                Some(_) => {}
            }
        }

        Ok(RequirementEvaluation {
            profile_id: profile.profile_id.clone(),
            subject_commitment: manifest.subject_commitment.clone(),
            satisfied: missing_theorems.is_empty() && unacceptable_facets.is_empty(),
            missing_theorems,
            unacceptable_facets,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> TheoremId {
        TheoremId::from(value)
    }

    fn definition(value: &str, prerequisites: &[&str]) -> TheoremDefinition {
        TheoremDefinition::new(
            value,
            prerequisites.iter().map(|value| id(value)).collect(),
        )
    }

    fn facet(theorem: &str, subject: &str, status: FacetStatus) -> QualificationFacet {
        QualificationFacet {
            theorem_id: id(theorem),
            subject_commitment: subject.to_string(),
            status,
            verifier_or_profile: None,
            evidence_refs: Vec::new(),
            dependency_commitments: Vec::new(),
            diagnostics_commitment: None,
        }
    }

    fn registry() -> TheoremRegistry {
        TheoremRegistry::new(vec![
            definition("Q0.SerializationSafety.v1", &[]),
            definition("Q2.FrameIdentity.v1", &["Q0.SerializationSafety.v1"]),
            definition(
                "Q4.GeometryObservability.v1",
                &["Q0.SerializationSafety.v1"],
            ),
            definition(
                "Q4.SolverConvergence.v1",
                &["Q0.SerializationSafety.v1"],
            ),
        ])
        .unwrap()
    }

    #[test]
    fn registry_rejects_unknown_prerequisite() {
        let result = TheoremRegistry::new(vec![definition("Q1", &["Q0"])]);
        assert!(matches!(
            result,
            Err(QualificationError::UnknownPrerequisite { .. })
        ));
    }

    #[test]
    fn registry_rejects_dependency_cycle() {
        let result = TheoremRegistry::new(vec![definition("Q0", &["Q1"]), definition("Q1", &["Q0"])]);
        assert!(matches!(
            result,
            Err(QualificationError::DependencyCycle { .. })
        ));
    }

    #[test]
    fn established_child_requires_established_prerequisite() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet(
                "Q2.FrameIdentity.v1",
                "state:abc",
                FacetStatus::Established,
            )],
        };

        assert!(matches!(
            registry.validate_manifest(&manifest),
            Err(QualificationError::EstablishedFacetMissingPrerequisite { .. })
        ));
    }

    #[test]
    fn indeterminate_prerequisite_cannot_launder_established_child() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet(
                    "Q0.SerializationSafety.v1",
                    "state:abc",
                    FacetStatus::Indeterminate,
                ),
                facet(
                    "Q2.FrameIdentity.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
            ],
        };

        assert!(matches!(
            registry.validate_manifest(&manifest),
            Err(QualificationError::EstablishedFacetMissingPrerequisite { .. })
        ));
    }

    #[test]
    fn indeterminate_does_not_satisfy_established_only_requirement() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet(
                "Q0.SerializationSafety.v1",
                "state:abc",
                FacetStatus::Indeterminate,
            )],
        };
        let profile = QualificationRequirementProfile::established_only(
            "StrictDisplay.v1",
            vec![id("Q0.SerializationSafety.v1")],
        );

        let evaluation = registry.evaluate(&manifest, &profile).unwrap();
        assert!(!evaluation.satisfied);
        assert!(evaluation.missing_theorems.is_empty());
        assert_eq!(evaluation.unacceptable_facets.len(), 1);
        assert_eq!(
            evaluation.unacceptable_facets[0].status,
            FacetStatus::Indeterminate
        );
    }

    #[test]
    fn different_consumers_can_require_different_facets() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet(
                    "Q0.SerializationSafety.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
                facet(
                    "Q2.FrameIdentity.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
            ],
        };

        let visualization = QualificationRequirementProfile::established_only(
            "Visualization.v1",
            vec![
                id("Q0.SerializationSafety.v1"),
                id("Q2.FrameIdentity.v1"),
            ],
        );
        let navigation = QualificationRequirementProfile::established_only(
            "Navigation.v1",
            vec![
                id("Q0.SerializationSafety.v1"),
                id("Q2.FrameIdentity.v1"),
                id("Q4.GeometryObservability.v1"),
                id("Q4.SolverConvergence.v1"),
            ],
        );

        assert!(registry.evaluate(&manifest, &visualization).unwrap().satisfied);
        let navigation_result = registry.evaluate(&manifest, &navigation).unwrap();
        assert!(!navigation_result.satisfied);
        assert_eq!(navigation_result.missing_theorems.len(), 2);
    }

    #[test]
    fn manifest_rejects_duplicate_facet() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet(
                    "Q0.SerializationSafety.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
                facet(
                    "Q0.SerializationSafety.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
            ],
        };
        assert!(matches!(
            registry.validate_manifest(&manifest),
            Err(QualificationError::DuplicateFacet { .. })
        ));
    }

    #[test]
    fn canonical_theorem_order_is_deterministic() {
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet(
                    "Q4.SolverConvergence.v1",
                    "state:abc",
                    FacetStatus::NotEstablished,
                ),
                facet(
                    "Q0.SerializationSafety.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
                facet(
                    "Q2.FrameIdentity.v1",
                    "state:abc",
                    FacetStatus::NotEstablished,
                ),
            ],
        };
        assert_eq!(
            manifest.canonical_theorem_order(),
            vec![
                id("Q0.SerializationSafety.v1"),
                id("Q2.FrameIdentity.v1"),
                id("Q4.SolverConvergence.v1"),
            ]
        );
    }

    #[test]
    fn structural_error_is_distinct_from_unsatisfied_requirement() {
        let registry = registry();
        let valid_manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet(
                "Q0.SerializationSafety.v1",
                "state:abc",
                FacetStatus::Established,
            )],
        };
        let missing_requirement = QualificationRequirementProfile::established_only(
            "NeedsFrame.v1",
            vec![id("Q2.FrameIdentity.v1")],
        );
        let evaluation = registry
            .evaluate(&valid_manifest, &missing_requirement)
            .unwrap();
        assert!(!evaluation.satisfied);
        assert_eq!(evaluation.missing_theorems, vec![id("Q2.FrameIdentity.v1")]);

        let malformed_manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet(
                "Q2.FrameIdentity.v1",
                "state:abc",
                FacetStatus::Established,
            )],
        };
        assert!(registry
            .evaluate(&malformed_manifest, &missing_requirement)
            .is_err());
    }
}

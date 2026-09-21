// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-bound learning analysis without authenticity, trust, or credential authority.
//!
//! This module provides the authority-safe successor contract for descriptive
//! learning analytics. It deliberately has no global "proof of learning" score and
//! no component named authenticity or cheating.

use crate::learning_evidence::{CapabilityId, EvidenceEventId, LearnerId};
use crate::proof_of_learning::ProofOfLearning;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Scope of an analysis. Domain scope is descriptive and must not silently become
/// a capability credential.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LearningAnalysisScope {
    Capability(CapabilityId),
    Domain(String),
}

/// Whether the analyzer consumed source observations directly or evidence admitted
/// by a named/versioned policy. The distinction remains visible in the receipt.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AnalysisEvidenceBasis {
    Observed,
    Admitted {
        profile_id: String,
        profile_version: String,
    },
}

/// Descriptive component vocabulary. These names state what was measured and do
/// not infer learner motive, honesty, authenticity, or cheating.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum LearningAnalysisComponentKind {
    TrajectoryTrend,
    ErrorPatternDistribution,
    RetentionPerformance,
    TransferPerformance,
    ContributionActivity,
    TemporalConsistency,
    Other(String),
}

/// Analyzer identity and exact parameter-set digest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearningAnalyzerProvenance {
    pub analyzer_id: String,
    pub analyzer_version: String,
    pub parameters_digest: String,
}

/// One descriptive analysis component, bound to the exact evidence events used to
/// compute it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearningAnalysisComponent {
    pub kind: LearningAnalysisComponentKind,
    /// Normalized analyzer output. Its interpretation belongs to the named analyzer;
    /// it is not intrinsically a probability of mastery or truth.
    pub estimate_permille: u16,
    /// Explicit support/confidence metadata from the named analyzer. This field is
    /// not assumed to be statistically calibrated merely because it is normalized.
    pub support_confidence_permille: u16,
    pub input_event_ids: Vec<EvidenceEventId>,
}

/// Evidence-bound descriptive analysis projection.
///
/// There is intentionally no global composite score. Consumers that want to combine
/// components must cross a separate named/versioned policy boundary and preserve
/// the original component evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LearningAnalysisProjection {
    pub learner_id: LearnerId,
    pub scope: LearningAnalysisScope,
    pub evidence_basis: AnalysisEvidenceBasis,
    pub analyzer: LearningAnalyzerProvenance,
    /// Exact unique set of source events consumed anywhere in the projection.
    pub input_event_ids: Vec<EvidenceEventId>,
    pub components: Vec<LearningAnalysisComponent>,
    pub generated_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LearningAnalysisContractError {
    EmptyLearnerId,
    EmptyCapabilityId,
    EmptyDomain,
    EmptyAdmissionProfile,
    EmptyAnalyzerId,
    EmptyAnalyzerVersion,
    EmptyParametersDigest,
    NoInputs,
    EmptyEventId,
    DuplicateProjectionInput(EvidenceEventId),
    NoComponents,
    EmptyOtherComponentName,
    DuplicateComponentKind(LearningAnalysisComponentKind),
    ComponentEstimateOutOfRange {
        kind: LearningAnalysisComponentKind,
        value_permille: u16,
    },
    ComponentConfidenceOutOfRange {
        kind: LearningAnalysisComponentKind,
        value_permille: u16,
    },
    ComponentHasNoInputs(LearningAnalysisComponentKind),
    DuplicateComponentInput {
        kind: LearningAnalysisComponentKind,
        event_id: EvidenceEventId,
    },
    ComponentInputNotDeclared {
        kind: LearningAnalysisComponentKind,
        event_id: EvidenceEventId,
    },
    ProjectionInputUnused(EvidenceEventId),
}

impl LearningAnalysisProjection {
    /// Validate provenance topology and normalized ranges only. Passing validation
    /// does not establish that the analyzer is scientifically validated or that its
    /// estimates are suitable for a consequential decision.
    pub fn validate(&self) -> Result<(), LearningAnalysisContractError> {
        if self.learner_id.0.trim().is_empty() {
            return Err(LearningAnalysisContractError::EmptyLearnerId);
        }
        match &self.scope {
            LearningAnalysisScope::Capability(capability) if capability.0.trim().is_empty() => {
                return Err(LearningAnalysisContractError::EmptyCapabilityId)
            }
            LearningAnalysisScope::Domain(domain) if domain.trim().is_empty() => {
                return Err(LearningAnalysisContractError::EmptyDomain)
            }
            _ => {}
        }
        if let AnalysisEvidenceBasis::Admitted {
            profile_id,
            profile_version,
        } = &self.evidence_basis
        {
            if profile_id.trim().is_empty() || profile_version.trim().is_empty() {
                return Err(LearningAnalysisContractError::EmptyAdmissionProfile);
            }
        }
        if self.analyzer.analyzer_id.trim().is_empty() {
            return Err(LearningAnalysisContractError::EmptyAnalyzerId);
        }
        if self.analyzer.analyzer_version.trim().is_empty() {
            return Err(LearningAnalysisContractError::EmptyAnalyzerVersion);
        }
        if self.analyzer.parameters_digest.trim().is_empty() {
            return Err(LearningAnalysisContractError::EmptyParametersDigest);
        }
        if self.input_event_ids.is_empty() {
            return Err(LearningAnalysisContractError::NoInputs);
        }

        let mut declared_inputs = BTreeSet::new();
        for event_id in &self.input_event_ids {
            if event_id.0.trim().is_empty() {
                return Err(LearningAnalysisContractError::EmptyEventId);
            }
            if !declared_inputs.insert(event_id.clone()) {
                return Err(LearningAnalysisContractError::DuplicateProjectionInput(
                    event_id.clone(),
                ));
            }
        }

        if self.components.is_empty() {
            return Err(LearningAnalysisContractError::NoComponents);
        }

        let mut component_kinds = BTreeSet::new();
        let mut referenced_inputs = BTreeSet::new();
        for component in &self.components {
            if let LearningAnalysisComponentKind::Other(name) = &component.kind {
                if name.trim().is_empty() {
                    return Err(LearningAnalysisContractError::EmptyOtherComponentName);
                }
            }
            if !component_kinds.insert(component.kind.clone()) {
                return Err(LearningAnalysisContractError::DuplicateComponentKind(
                    component.kind.clone(),
                ));
            }
            if component.estimate_permille > 1000 {
                return Err(LearningAnalysisContractError::ComponentEstimateOutOfRange {
                    kind: component.kind.clone(),
                    value_permille: component.estimate_permille,
                });
            }
            if component.support_confidence_permille > 1000 {
                return Err(
                    LearningAnalysisContractError::ComponentConfidenceOutOfRange {
                        kind: component.kind.clone(),
                        value_permille: component.support_confidence_permille,
                    },
                );
            }
            if component.input_event_ids.is_empty() {
                return Err(LearningAnalysisContractError::ComponentHasNoInputs(
                    component.kind.clone(),
                ));
            }

            let mut local_inputs = BTreeSet::new();
            for event_id in &component.input_event_ids {
                if event_id.0.trim().is_empty() {
                    return Err(LearningAnalysisContractError::EmptyEventId);
                }
                if !local_inputs.insert(event_id.clone()) {
                    return Err(LearningAnalysisContractError::DuplicateComponentInput {
                        kind: component.kind.clone(),
                        event_id: event_id.clone(),
                    });
                }
                if !declared_inputs.contains(event_id) {
                    return Err(LearningAnalysisContractError::ComponentInputNotDeclared {
                        kind: component.kind.clone(),
                        event_id: event_id.clone(),
                    });
                }
                referenced_inputs.insert(event_id.clone());
            }
        }

        for event_id in declared_inputs {
            if !referenced_inputs.contains(&event_id) {
                return Err(LearningAnalysisContractError::ProjectionInputUnused(event_id));
            }
        }

        Ok(())
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }

    pub const fn grants_trust_authority(&self) -> bool {
        false
    }
}

/// Explicit compatibility summary for the historical `ProofOfLearning` object.
///
/// It deliberately contains no `input_event_ids`: legacy PoL evidence does not carry
/// the stable provenance-complete event identity required by `LearningAnalysisProjection`.
/// There is intentionally no conversion from this type into a complete projection.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyPoLCompatibilitySummary {
    pub learner_id: String,
    pub domain: String,
    pub legacy_score_permille: u16,
    pub legacy_confidence_permille: u16,
    pub legacy_algorithm_version: String,
    pub generated_at: i64,
    pub evidence_lineage_complete: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LegacyPoLAdapterError {
    EmptyLearnerId,
    EmptyDomain,
    EmptyAlgorithmVersion,
    NonFiniteScore,
    ScoreOutOfRange,
    NonFiniteConfidence,
    ConfidenceOutOfRange,
}

impl LegacyPoLCompatibilitySummary {
    /// Preserve a historical PoL scalar for migration/visualization without
    /// promoting it into the provenance-complete analysis contract.
    pub fn from_legacy(pol: &ProofOfLearning) -> Result<Self, LegacyPoLAdapterError> {
        if pol.learner_id.trim().is_empty() {
            return Err(LegacyPoLAdapterError::EmptyLearnerId);
        }
        if pol.domain.trim().is_empty() {
            return Err(LegacyPoLAdapterError::EmptyDomain);
        }
        if pol.algorithm_version.trim().is_empty() {
            return Err(LegacyPoLAdapterError::EmptyAlgorithmVersion);
        }

        Ok(Self {
            learner_id: pol.learner_id.clone(),
            domain: pol.domain.clone(),
            legacy_score_permille: unit_interval_to_permille(
                pol.score,
                LegacyPoLAdapterError::NonFiniteScore,
                LegacyPoLAdapterError::ScoreOutOfRange,
            )?,
            legacy_confidence_permille: unit_interval_to_permille(
                pol.confidence,
                LegacyPoLAdapterError::NonFiniteConfidence,
                LegacyPoLAdapterError::ConfidenceOutOfRange,
            )?,
            legacy_algorithm_version: pol.algorithm_version.clone(),
            generated_at: pol.generated_at,
            evidence_lineage_complete: false,
        })
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }

    pub const fn grants_trust_authority(&self) -> bool {
        false
    }
}

fn unit_interval_to_permille(
    value: f64,
    non_finite_error: LegacyPoLAdapterError,
    range_error: LegacyPoLAdapterError,
) -> Result<u16, LegacyPoLAdapterError> {
    if !value.is_finite() {
        return Err(non_finite_error);
    }
    if !(0.0..=1.0).contains(&value) {
        return Err(range_error);
    }
    Ok((value * 1000.0).round() as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proof_of_learning::PoLComponents;

    fn valid_projection() -> LearningAnalysisProjection {
        let a = EvidenceEventId("event-a".into());
        let b = EvidenceEventId("event-b".into());
        LearningAnalysisProjection {
            learner_id: LearnerId("learner-1".into()),
            scope: LearningAnalysisScope::Capability(CapabilityId("rust:ownership".into())),
            evidence_basis: AnalysisEvidenceBasis::Admitted {
                profile_id: "analysis-evidence-v1".into(),
                profile_version: "1".into(),
            },
            analyzer: LearningAnalyzerProvenance {
                analyzer_id: "praxis:learning-analysis".into(),
                analyzer_version: "1".into(),
                parameters_digest: "blake3:abc".into(),
            },
            input_event_ids: vec![a.clone(), b.clone()],
            components: vec![
                LearningAnalysisComponent {
                    kind: LearningAnalysisComponentKind::TrajectoryTrend,
                    estimate_permille: 700,
                    support_confidence_permille: 500,
                    input_event_ids: vec![a.clone(), b.clone()],
                },
                LearningAnalysisComponent {
                    kind: LearningAnalysisComponentKind::TransferPerformance,
                    estimate_permille: 600,
                    support_confidence_permille: 400,
                    input_event_ids: vec![b],
                },
            ],
            generated_at: 1_700_000_000,
        }
    }

    #[test]
    fn valid_projection_is_descriptive_not_authoritative() {
        let projection = valid_projection();
        assert_eq!(projection.validate(), Ok(()));
        assert!(!projection.grants_credential_authority());
        assert!(!projection.grants_trust_authority());
    }

    #[test]
    fn duplicate_projection_inputs_are_rejected() {
        let mut projection = valid_projection();
        projection
            .input_event_ids
            .push(EvidenceEventId("event-a".into()));
        assert_eq!(
            projection.validate(),
            Err(LearningAnalysisContractError::DuplicateProjectionInput(
                EvidenceEventId("event-a".into())
            ))
        );
    }

    #[test]
    fn component_cannot_reference_undeclared_evidence() {
        let mut projection = valid_projection();
        projection.components[0]
            .input_event_ids
            .push(EvidenceEventId("event-c".into()));
        assert_eq!(
            projection.validate(),
            Err(LearningAnalysisContractError::ComponentInputNotDeclared {
                kind: LearningAnalysisComponentKind::TrajectoryTrend,
                event_id: EvidenceEventId("event-c".into()),
            })
        );
    }

    #[test]
    fn declared_projection_input_must_actually_support_a_component() {
        let mut projection = valid_projection();
        projection
            .input_event_ids
            .push(EvidenceEventId("event-unused".into()));
        assert_eq!(
            projection.validate(),
            Err(LearningAnalysisContractError::ProjectionInputUnused(
                EvidenceEventId("event-unused".into())
            ))
        );
    }

    #[test]
    fn duplicate_component_kind_is_rejected() {
        let mut projection = valid_projection();
        projection.components.push(LearningAnalysisComponent {
            kind: LearningAnalysisComponentKind::TrajectoryTrend,
            estimate_permille: 500,
            support_confidence_permille: 500,
            input_event_ids: vec![EvidenceEventId("event-a".into())],
        });
        assert_eq!(
            projection.validate(),
            Err(LearningAnalysisContractError::DuplicateComponentKind(
                LearningAnalysisComponentKind::TrajectoryTrend
            ))
        );
    }

    #[test]
    fn component_requires_evidence_references() {
        let mut projection = valid_projection();
        projection.components[0].input_event_ids.clear();
        assert_eq!(
            projection.validate(),
            Err(LearningAnalysisContractError::ComponentHasNoInputs(
                LearningAnalysisComponentKind::TrajectoryTrend
            ))
        );
    }

    #[test]
    fn component_values_must_be_normalized() {
        let mut projection = valid_projection();
        projection.components[0].estimate_permille = 1001;
        assert_eq!(
            projection.validate(),
            Err(LearningAnalysisContractError::ComponentEstimateOutOfRange {
                kind: LearningAnalysisComponentKind::TrajectoryTrend,
                value_permille: 1001,
            })
        );
    }

    #[test]
    fn legacy_pol_summary_never_claims_complete_event_lineage() {
        let pol = ProofOfLearning {
            learner_id: "learner-1".into(),
            domain: "programming".into(),
            score: 0.82,
            components: PoLComponents::default(),
            evidence: vec![],
            generated_at: 42,
            confidence: 0.73,
            algorithm_version: "pol-v1.0".into(),
        };
        let summary = LegacyPoLCompatibilitySummary::from_legacy(&pol).unwrap();
        assert_eq!(summary.legacy_score_permille, 820);
        assert_eq!(summary.legacy_confidence_permille, 730);
        assert!(!summary.evidence_lineage_complete);
        assert!(!summary.grants_credential_authority());
        assert!(!summary.grants_trust_authority());
    }

    #[test]
    fn legacy_pol_nan_cannot_cross_compatibility_boundary() {
        let pol = ProofOfLearning {
            learner_id: "learner-1".into(),
            domain: "programming".into(),
            score: f64::NAN,
            components: PoLComponents::default(),
            evidence: vec![],
            generated_at: 42,
            confidence: 0.5,
            algorithm_version: "pol-v1.0".into(),
        };
        assert_eq!(
            LegacyPoLCompatibilitySummary::from_legacy(&pol),
            Err(LegacyPoLAdapterError::NonFiniteScore)
        );
    }
}

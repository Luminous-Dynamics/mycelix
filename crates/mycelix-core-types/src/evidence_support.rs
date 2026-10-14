// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-support semantics for factual/systemic claims.
//!
//! This module is deliberately orthogonal to the existing E/N/M/H
//! `EpistemicClassification`. E/N/M/H remains useful for describing empirical
//! mode, normative scope, materiality, harmonics, routing, and scrutiny. It is
//! **not** a probability that a real-world assertion is true.
//!
//! Core invariants:
//!
//! ```text
//! high stakes != high evidentiary support
//! normative scope != factual confidence
//! material persistence != truth
//! cryptographic integrity != empirical validity
//! signed false statement != verified fact
//! proof of computation != truth/completeness of inputs
//! mirrored source != independent corroboration
//! ```

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Whether a particular assurance dimension has been established.
///
/// This is intentionally not numeric: authenticity and byte integrity are
/// different questions and MUST NOT be averaged into a universal confidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum AssuranceStatus {
    #[default]
    Unassessed,
    Established,
    Failed,
    Contested,
    NotApplicable,
}

/// Overall factual-support disposition after examining the available evidence.
///
/// The status is a typed assessment outcome, not a calibrated probability.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum EvidenceAssessmentStatus {
    #[default]
    Unassessed,
    Supported,
    Contested,
    Refuted,
    Indeterminate,
}

/// Concrete procedure used to assess a claim or artifact.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum VerificationMethod {
    DirectObservation,
    Measurement,
    IndependentReplication,
    RegistryCrossCheck,
    DocumentaryAudit,
    CryptographicSignature,
    ContentDigest,
    ZeroKnowledgeProof,
    SecureComputationProof,
    Other(String),
}

/// Coverage/representativeness state for the evidence underlying a claim.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum CoverageAssessment {
    Unknown,
    /// The source definition itself establishes full enumeration of the stated
    /// population/scope. This says nothing about truth outside that scope.
    CompleteByDefinition,
    /// Known observed units and, when available, the known target population.
    Partial {
        observed: u64,
        total: Option<u64>,
    },
    /// Sample-based evidence where representativeness must be assessed rather
    /// than inferred from sample size alone.
    Sampled {
        sample_size: u64,
        population_size: Option<u64>,
        representativeness: AssuranceStatus,
    },
}

impl Default for CoverageAssessment {
    fn default() -> Self {
        Self::Unknown
    }
}

impl CoverageAssessment {
    /// Validate count relationships without inferring evidentiary quality.
    ///
    /// Zero-sized samples or populations are structurally representable here;
    /// whether they are informative belongs to the assessment context. What is
    /// never structurally coherent is observing more units than a known total,
    /// or sampling more units than a known population.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::Partial {
                observed,
                total: Some(total),
            } if observed > total => Err("observed coverage cannot exceed known total"),
            Self::Sampled {
                sample_size,
                population_size: Some(population_size),
                ..
            } if sample_size > population_size => {
                Err("sample size cannot exceed known population size")
            }
            _ => Ok(()),
        }
    }
}

/// Quantitative uncertainty attached to an assessed quantity.
///
/// No interval is interpreted as a frequentist confidence interval or Bayesian
/// credible interval unless `method` says so explicitly.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum QuantifiedUncertainty {
    NotQuantified,
    Interval {
        lower: f64,
        upper: f64,
        method: String,
        level: Option<f64>,
    },
    DistributionRef {
        locator: String,
        method: String,
    },
}

impl Default for QuantifiedUncertainty {
    fn default() -> Self {
        Self::NotQuantified
    }
}

impl QuantifiedUncertainty {
    /// Validate basic structural invariants without inventing statistical
    /// semantics that belong to the named method.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::NotQuantified | Self::DistributionRef { .. } => Ok(()),
            Self::Interval {
                lower,
                upper,
                level,
                ..
            } => {
                if !lower.is_finite() || !upper.is_finite() || upper < lower {
                    return Err("uncertainty interval must be finite and ordered");
                }
                if let Some(level) = level {
                    if !level.is_finite() || *level <= 0.0 || *level >= 1.0 {
                        return Err("uncertainty level must be finite and in (0, 1)");
                    }
                }
                Ok(())
            }
        }
    }
}

/// Provenance-family accounting for corroboration and contradiction.
///
/// IDs identify independent upstream provenance families, not individual URLs
/// or mirrors. Ten downstream copies of one filing should normally contribute
/// one family here, not ten independent confirmations.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CorroborationProfile {
    pub supporting_families: Vec<String>,
    pub contradicting_families: Vec<String>,
}

impl CorroborationProfile {
    pub fn independent_support_count(&self) -> usize {
        unique_count(&self.supporting_families)
    }

    pub fn independent_contradiction_count(&self) -> usize {
        unique_count(&self.contradicting_families)
    }
}

fn unique_count(values: &[String]) -> usize {
    use std::collections::HashSet;
    values.iter().collect::<HashSet<_>>().len()
}

/// Multi-dimensional factual-support assessment for a systemic claim.
///
/// There is deliberately no `confidence: f64` and no `score()` method. A caller
/// that needs a decision rule must state how these dimensions are combined in
/// that decision context and preserve the inputs used to make that decision.
#[derive(Debug, Clone, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EvidenceSupportProfile {
    /// Can the issuer/source artifact be authenticated as claimed?
    pub source_authenticity: AssuranceStatus,
    /// Do bytes/digests/cryptographic commitments match the referenced artifact?
    pub artifact_integrity: AssuranceStatus,
    /// Does the observation/measurement method support the quantity/assertion?
    pub measurement_validity: AssuranceStatus,
    /// Concrete assessment procedures that were actually performed.
    pub verification_methods: Vec<VerificationMethod>,
    /// Independent upstream support/contradiction families.
    pub corroboration: CorroborationProfile,
    /// Coverage and representativeness of the underlying evidence.
    pub coverage: CoverageAssessment,
    /// Quantitative uncertainty, when meaningful and actually assessed.
    pub uncertainty: QuantifiedUncertainty,
    /// Human/machine-readable statement of what the source is competent or
    /// authorized to assert. Absence means unassessed, not unlimited scope.
    pub source_scope: Option<String>,
    /// Overall typed disposition. This remains distinct from every dimension
    /// above so a cryptographic success cannot silently promote factual truth.
    pub assessment_status: EvidenceAssessmentStatus,
}

impl EvidenceSupportProfile {
    /// Validate local structural invariants only.
    pub fn validate(&self) -> Result<(), &'static str> {
        self.coverage.validate()?;
        self.uncertainty.validate()
    }

    /// Whether cryptographic/artifact integrity has been established.
    ///
    /// This intentionally says nothing about empirical truth or completeness.
    pub fn has_established_integrity(&self) -> bool {
        self.artifact_integrity == AssuranceStatus::Established
    }

    /// Whether the profile contains materially contradictory provenance.
    pub fn has_independent_contradiction(&self) -> bool {
        self.corroboration.independent_contradiction_count() > 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cryptographic_integrity_does_not_promote_factual_status() {
        let profile = EvidenceSupportProfile {
            source_authenticity: AssuranceStatus::Established,
            artifact_integrity: AssuranceStatus::Established,
            verification_methods: vec![VerificationMethod::CryptographicSignature],
            assessment_status: EvidenceAssessmentStatus::Unassessed,
            ..Default::default()
        };

        assert!(profile.has_established_integrity());
        assert_eq!(
            profile.assessment_status,
            EvidenceAssessmentStatus::Unassessed
        );
    }

    #[test]
    fn mirrored_records_do_not_inflate_independent_support() {
        let profile = CorroborationProfile {
            supporting_families: vec![
                "filing:abc".into(),
                "filing:abc".into(),
                "filing:abc".into(),
                "audit:def".into(),
            ],
            contradicting_families: vec![],
        };

        assert_eq!(profile.independent_support_count(), 2);
    }

    #[test]
    fn contradiction_is_preserved_separately_from_support() {
        let profile = EvidenceSupportProfile {
            corroboration: CorroborationProfile {
                supporting_families: vec!["registry:a".into()],
                contradicting_families: vec!["audit:b".into()],
            },
            assessment_status: EvidenceAssessmentStatus::Contested,
            ..Default::default()
        };

        assert_eq!(profile.corroboration.independent_support_count(), 1);
        assert_eq!(profile.corroboration.independent_contradiction_count(), 1);
        assert!(profile.has_independent_contradiction());
    }

    #[test]
    fn uncertainty_interval_is_validated_without_relabeling_method() {
        let valid = QuantifiedUncertainty::Interval {
            lower: 1.0,
            upper: 2.0,
            method: "bootstrap percentile".into(),
            level: Some(0.95),
        };
        assert!(valid.validate().is_ok());

        let reversed = QuantifiedUncertainty::Interval {
            lower: 2.0,
            upper: 1.0,
            method: "unknown".into(),
            level: None,
        };
        assert!(reversed.validate().is_err());
    }

    #[test]
    fn coverage_does_not_imply_support_status() {
        let profile = EvidenceSupportProfile {
            coverage: CoverageAssessment::CompleteByDefinition,
            assessment_status: EvidenceAssessmentStatus::Indeterminate,
            ..Default::default()
        };

        assert_eq!(
            profile.assessment_status,
            EvidenceAssessmentStatus::Indeterminate
        );
    }

    #[test]
    fn coverage_structural_count_relationships_are_validated() {
        let impossible_partial = EvidenceSupportProfile {
            coverage: CoverageAssessment::Partial {
                observed: 12,
                total: Some(10),
            },
            ..Default::default()
        };
        assert!(impossible_partial.validate().is_err());

        let impossible_sample = EvidenceSupportProfile {
            coverage: CoverageAssessment::Sampled {
                sample_size: 500,
                population_size: Some(100),
                representativeness: AssuranceStatus::Unassessed,
            },
            ..Default::default()
        };
        assert!(impossible_sample.validate().is_err());

        let valid_zero = EvidenceSupportProfile {
            coverage: CoverageAssessment::Sampled {
                sample_size: 0,
                population_size: Some(0),
                representativeness: AssuranceStatus::Unassessed,
            },
            assessment_status: EvidenceAssessmentStatus::Indeterminate,
            ..Default::default()
        };
        assert!(valid_zero.validate().is_ok());
        assert_eq!(
            valid_zero.assessment_status,
            EvidenceAssessmentStatus::Indeterminate
        );
    }
}

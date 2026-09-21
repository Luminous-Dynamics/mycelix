// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Quarantine boundary for historical Proof-of-Learning outputs.
//!
//! The legacy PoL implementation is retained for reproducibility, but its scalar
//! score and historical PoL->MATL adjustment are not authoritative trust inputs.
//! New code should cross this boundary before displaying, migrating, or inspecting
//! a legacy PoL result.

use crate::learning_analysis::{LegacyPoLAdapterError, LegacyPoLCompatibilitySummary};
use crate::proof_of_learning::{PoLMATLScore, ProofOfLearning};
use serde::{Deserialize, Serialize};

/// Why a historical PoL result is quarantined from consequential authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LegacyPoLQuarantineReason {
    /// Legacy evidence lacks stable provenance-complete event identifiers.
    IncompleteEvidenceLineage,
    /// The historical composite mixes heterogeneous heuristic components.
    HeterogeneousCompositeScore,
    /// Historical error-pattern scoring is not a validated authenticity detector.
    UnvalidatedAuthenticityInterpretation,
    /// Direct PoL->MATL weighting has no evidence-bound trust-policy receipt.
    NoExplicitTrustPolicy,
}

/// Authority status for historical PoL material.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LegacyPoLAuthorityStatus {
    /// Historical heuristic output may be retained for reproducibility or display,
    /// but it must not independently authorize a consequential decision.
    HistoricalHeuristicOnly,
}

/// Non-authoritative receipt for a historical PoL object.
///
/// This receipt deliberately carries no stable evidence-event IDs and offers no
/// conversion into `LearningAnalysisProjection`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyPoLQuarantineReceipt {
    pub compatibility: LegacyPoLCompatibilitySummary,
    pub authority_status: LegacyPoLAuthorityStatus,
    pub quarantine_reasons: Vec<LegacyPoLQuarantineReason>,
}

impl LegacyPoLQuarantineReceipt {
    /// Wrap a historical PoL output without strengthening its authority.
    pub fn from_legacy(pol: &ProofOfLearning) -> Result<Self, LegacyPoLAdapterError> {
        Ok(Self {
            compatibility: LegacyPoLCompatibilitySummary::from_legacy(pol)?,
            authority_status: LegacyPoLAuthorityStatus::HistoricalHeuristicOnly,
            quarantine_reasons: vec![
                LegacyPoLQuarantineReason::IncompleteEvidenceLineage,
                LegacyPoLQuarantineReason::HeterogeneousCompositeScore,
                LegacyPoLQuarantineReason::UnvalidatedAuthenticityInterpretation,
                LegacyPoLQuarantineReason::NoExplicitTrustPolicy,
            ],
        })
    }

    pub const fn grants_trust_authority(&self) -> bool {
        false
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }

    pub const fn grants_authorization(&self) -> bool {
        false
    }

    pub const fn evidence_lineage_complete(&self) -> bool {
        false
    }
}

/// Historical PoL->MATL arithmetic preserved strictly for reproducibility.
///
/// This type makes the quarantine explicit around the old `PoLMATLScore::combine`
/// behavior. The contained legacy value may be compared with historical output but
/// cannot be treated as an authorized trust decision by this contract.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LegacyPoLMATLCompatibilityReceipt {
    pub legacy: PoLMATLScore,
    pub pol: LegacyPoLCompatibilitySummary,
    pub authority_status: LegacyPoLAuthorityStatus,
    pub trust_policy_present: bool,
    pub evidence_lineage_complete: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LegacyPoLMATLQuarantineError {
    InvalidLegacyPoL(LegacyPoLAdapterError),
    NonFiniteMatlBase,
    MatlBaseOutOfRange,
    NonFiniteWeight,
    WeightOutOfRange,
}

impl From<LegacyPoLAdapterError> for LegacyPoLMATLQuarantineError {
    fn from(value: LegacyPoLAdapterError) -> Self {
        Self::InvalidLegacyPoL(value)
    }
}

impl LegacyPoLMATLCompatibilityReceipt {
    /// Reproduce the historical arithmetic while explicitly denying authority.
    ///
    /// No trust-policy identifier is accepted here by design: this is not a new
    /// trust policy implementation. Future trust integration must use a separate
    /// evidence-bound, named/versioned policy contract.
    pub fn reproduce_historical(
        matl_base: f64,
        pol: &ProofOfLearning,
        pol_weight: f64,
    ) -> Result<Self, LegacyPoLMATLQuarantineError> {
        if !matl_base.is_finite() {
            return Err(LegacyPoLMATLQuarantineError::NonFiniteMatlBase);
        }
        if !(0.0..=1.0).contains(&matl_base) {
            return Err(LegacyPoLMATLQuarantineError::MatlBaseOutOfRange);
        }
        if !pol_weight.is_finite() {
            return Err(LegacyPoLMATLQuarantineError::NonFiniteWeight);
        }
        if !(0.0..=1.0).contains(&pol_weight) {
            return Err(LegacyPoLMATLQuarantineError::WeightOutOfRange);
        }

        let compatibility = LegacyPoLCompatibilitySummary::from_legacy(pol)?;
        Ok(Self {
            legacy: PoLMATLScore::combine(matl_base, pol, pol_weight),
            pol: compatibility,
            authority_status: LegacyPoLAuthorityStatus::HistoricalHeuristicOnly,
            trust_policy_present: false,
            evidence_lineage_complete: false,
        })
    }

    pub const fn grants_trust_authority(&self) -> bool {
        false
    }

    pub const fn grants_credential_authority(&self) -> bool {
        false
    }

    pub const fn grants_authorization(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proof_of_learning::PoLComponents;

    fn legacy_pol() -> ProofOfLearning {
        ProofOfLearning {
            learner_id: "learner-1".into(),
            domain: "programming".into(),
            score: 0.8,
            components: PoLComponents::default(),
            evidence: vec![],
            generated_at: 42,
            confidence: 0.7,
            algorithm_version: "pol-v1.0".into(),
        }
    }

    #[test]
    fn legacy_pol_is_always_quarantined_from_authority() {
        let receipt = LegacyPoLQuarantineReceipt::from_legacy(&legacy_pol()).unwrap();
        assert_eq!(
            receipt.authority_status,
            LegacyPoLAuthorityStatus::HistoricalHeuristicOnly
        );
        assert!(!receipt.evidence_lineage_complete());
        assert!(!receipt.grants_trust_authority());
        assert!(!receipt.grants_credential_authority());
        assert!(!receipt.grants_authorization());
        assert!(receipt
            .quarantine_reasons
            .contains(&LegacyPoLQuarantineReason::NoExplicitTrustPolicy));
    }

    #[test]
    fn historical_matl_arithmetic_can_be_reproduced_without_authority() {
        let pol = legacy_pol();
        let historical = PoLMATLScore::combine(0.7, &pol, 0.3);
        let receipt = LegacyPoLMATLCompatibilityReceipt::reproduce_historical(0.7, &pol, 0.3)
            .unwrap();

        assert_eq!(receipt.legacy.combined, historical.combined);
        assert_eq!(receipt.legacy.pol_adjustment, historical.pol_adjustment);
        assert!(!receipt.trust_policy_present);
        assert!(!receipt.evidence_lineage_complete);
        assert!(!receipt.grants_trust_authority());
        assert!(!receipt.grants_credential_authority());
        assert!(!receipt.grants_authorization());
    }

    #[test]
    fn invalid_weight_cannot_cross_quarantine_boundary() {
        assert!(matches!(
            LegacyPoLMATLCompatibilityReceipt::reproduce_historical(0.7, &legacy_pol(), 1.1),
            Err(LegacyPoLMATLQuarantineError::WeightOutOfRange)
        ));
    }

    #[test]
    fn invalid_matl_base_cannot_cross_quarantine_boundary() {
        assert!(matches!(
            LegacyPoLMATLCompatibilityReceipt::reproduce_historical(-0.1, &legacy_pol(), 0.3),
            Err(LegacyPoLMATLQuarantineError::MatlBaseOutOfRange)
        ));
    }
}

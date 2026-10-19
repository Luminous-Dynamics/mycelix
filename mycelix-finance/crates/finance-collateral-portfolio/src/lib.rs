#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Fail-closed multi-collateral portfolio health evaluation.
//!
//! This crate replaces caller-derived aggregate health with a checked evaluation
//! over exact, current FIN-SAFE-002 component evidence. It intentionally grants
//! zero risk-reducing diversification credit. A future diversification model must
//! be separately versioned and evidenced rather than inferred from caller labels.
//!
//! The aggregate obligation remains an explicit upstream trust boundary in this
//! tranche. Callers must bind it to the canonical obligation authority before
//! invoking this evaluator; this crate does not manufacture that authority.

use std::collections::BTreeMap;

use finance_collateral_evidence::{
    CollateralHealthEvidenceV2, CollateralHealthEvidenceV2ValidationError,
    CollateralHealthFreshnessPolicy,
};
use finance_collateral_safety::{assess_collateral_health, CollateralHealthAssessment};
use finance_collateral_valuation::CollateralValuationOutcome;
use serde::{Deserialize, Serialize};

/// Hard ceiling for a single checked portfolio evaluation.
pub const MAX_PORTFOLIO_COMPONENTS: usize = 256;

/// FIN-SAFE-005 conservative rule: unsupported diversification assumptions may
/// not reduce effective LTV or increase borrowing capacity.
pub const RISK_REDUCING_DIVERSIFICATION_CREDIT_BPS: u16 = 0;

/// Error returned before a portfolio health evaluation can be trusted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PortfolioHealthEvaluationError {
    EmptyPortfolio,
    TooManyComponents,
    ComponentEvidence {
        index: u16,
        source: CollateralHealthEvidenceV2ValidationError,
    },
    DuplicateCollateralId {
        first_index: u16,
        duplicate_index: u16,
    },
    DuplicateAssetIdentity {
        first_index: u16,
        duplicate_index: u16,
    },
    AggregateValueOverflow,
    UnavailableAssessmentMismatch,
}

/// A checked portfolio result valid at one explicit evaluation time.
///
/// This is a derived response, not a persistence format and not a liquidation
/// authorization. Reuse at a later time requires a new evaluation of the source
/// component evidence under the then-current expected policy.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CheckedPortfolioHealthEvaluation {
    pub component_count: u16,
    /// Absent whenever any required component lacks a numeric observed value.
    pub aggregate_value: Option<u64>,
    /// This amount is assumed to have been verified by an upstream obligation
    /// authority before entering this pure evaluator.
    pub aggregate_obligation: u64,
    pub ltv_ratio: Option<f64>,
    pub assessment: CollateralHealthAssessment,
    /// Always zero in this conservative tranche.
    pub risk_reducing_diversification_credit_bps: u16,
    /// IDs of components whose valuation outcome was unavailable. Empty for a
    /// Known aggregate result.
    pub unavailable_component_ids: Vec<String>,
    pub evaluated_at_micros: i64,
}

impl CheckedPortfolioHealthEvaluation {
    /// Evidence predicate only. This does not authorize default, seizure, or
    /// liquidation and becomes stale as soon as its source evidence ages out.
    pub fn is_liquidation_threshold_evidence_at_evaluation(&self) -> bool {
        self.unavailable_component_ids.is_empty() && self.assessment.is_liquidation_evidence()
    }
}

/// Evaluate portfolio health from exact current component evidence.
///
/// Security properties:
///
/// - every component must pass FIN-SAFE-002 currentness validation under the
///   consumer's exact expected policy;
/// - duplicate Finance collateral IDs are rejected;
/// - duplicate authoritative `(source_happ, asset_id)` identities are rejected;
/// - aggregate value uses checked addition, never wrap/saturation;
/// - any unavailable component makes aggregate value/LTV unavailable;
/// - observed zero remains observed zero and therefore produces an
///   Indeterminate zero-valuation result when the whole aggregate is zero;
/// - the canonical single-collateral classifier is reused for portfolio LTV;
/// - diversification credit is exactly zero.
///
/// `aggregate_obligation` is intentionally named as a trust boundary. This
/// function does not prove where that amount came from; callers must supply only
/// an amount already verified against the canonical obligation authority.
pub fn evaluate_current_portfolio_health(
    components: &[CollateralHealthEvidenceV2],
    expected_policy: &CollateralHealthFreshnessPolicy,
    aggregate_obligation: u64,
    evaluated_at_micros: i64,
) -> Result<CheckedPortfolioHealthEvaluation, PortfolioHealthEvaluationError> {
    if components.is_empty() {
        return Err(PortfolioHealthEvaluationError::EmptyPortfolio);
    }
    if components.len() > MAX_PORTFOLIO_COMPONENTS {
        return Err(PortfolioHealthEvaluationError::TooManyComponents);
    }

    let mut collateral_ids: BTreeMap<String, u16> = BTreeMap::new();
    let mut authoritative_assets: BTreeMap<(String, String), u16> = BTreeMap::new();
    let mut aggregate_value = 0u64;
    let mut unavailable_component_ids = Vec::new();
    let mut first_indeterminate_assessment: Option<CollateralHealthAssessment> = None;

    for (index, evidence) in components.iter().enumerate() {
        let index = u16::try_from(index)
            .map_err(|_| PortfolioHealthEvaluationError::TooManyComponents)?;

        evidence
            .validate_current_at(expected_policy, evaluated_at_micros)
            .map_err(|source| PortfolioHealthEvaluationError::ComponentEvidence {
                index,
                source,
            })?;

        let collateral_id = evidence.request.subject.collateral_id.clone();
        if let Some(first_index) = collateral_ids.insert(collateral_id.clone(), index) {
            return Err(PortfolioHealthEvaluationError::DuplicateCollateralId {
                first_index,
                duplicate_index: index,
            });
        }

        let authoritative_asset = (
            evidence.request.subject.source_happ.clone(),
            evidence.request.subject.asset_id.clone(),
        );
        if let Some(first_index) = authoritative_assets.insert(authoritative_asset, index) {
            return Err(PortfolioHealthEvaluationError::DuplicateAssetIdentity {
                first_index,
                duplicate_index: index,
            });
        }

        match evidence.envelope.outcome {
            CollateralValuationOutcome::Observed { value, .. } => {
                aggregate_value = aggregate_value
                    .checked_add(value)
                    .ok_or(PortfolioHealthEvaluationError::AggregateValueOverflow)?;
            }
            CollateralValuationOutcome::Unavailable { .. } => {
                unavailable_component_ids.push(collateral_id);
                if first_indeterminate_assessment.is_none() {
                    first_indeterminate_assessment = Some(evidence.snapshot.assessment.clone());
                }
            }
        }
    }

    let component_count = u16::try_from(components.len())
        .map_err(|_| PortfolioHealthEvaluationError::TooManyComponents)?;

    if !unavailable_component_ids.is_empty() {
        let assessment = first_indeterminate_assessment
            .ok_or(PortfolioHealthEvaluationError::UnavailableAssessmentMismatch)?;
        return Ok(CheckedPortfolioHealthEvaluation {
            component_count,
            aggregate_value: None,
            aggregate_obligation,
            ltv_ratio: None,
            assessment,
            risk_reducing_diversification_credit_bps:
                RISK_REDUCING_DIVERSIFICATION_CREDIT_BPS,
            unavailable_component_ids,
            evaluated_at_micros,
        });
    }

    let ltv_ratio = if aggregate_value > 0 {
        Some(aggregate_obligation as f64 / aggregate_value as f64)
    } else {
        None
    };
    let assessment = assess_collateral_health(aggregate_obligation, Some(aggregate_value));

    Ok(CheckedPortfolioHealthEvaluation {
        component_count,
        aggregate_value: Some(aggregate_value),
        aggregate_obligation,
        ltv_ratio,
        assessment,
        risk_reducing_diversification_credit_bps: RISK_REDUCING_DIVERSIFICATION_CREDIT_BPS,
        unavailable_component_ids,
        evaluated_at_micros,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_evidence::CollateralHealthFreshnessPolicy;
    use finance_collateral_safety::{
        CollateralHealthAssessment, CollateralHealthIndeterminateReason,
    };
    use finance_collateral_valuation::{
        CollateralValuationCapability, CollateralValuationEnvelope,
        CollateralValuationFailure, CollateralValuationOutcome,
        CollateralValuationRequest, CollateralValuationSubject,
        COLLATERAL_VALUATION_PROTOCOL_VERSION,
    };

    fn policy() -> CollateralHealthFreshnessPolicy {
        CollateralHealthFreshnessPolicy {
            policy_id: "portfolio-default".into(),
            policy_version: 1,
            max_evidence_age_micros: 100,
        }
    }

    fn evidence(
        collateral_id: &str,
        source_happ: &str,
        asset_id: &str,
        outcome: CollateralValuationOutcome,
    ) -> CollateralHealthEvidenceV2 {
        let capability = CollateralValuationCapability {
            provider_id: "portfolio-test-provider".into(),
            method: "value_asset".into(),
            protocol_version: COLLATERAL_VALUATION_PROTOCOL_VERSION,
        };
        let subject = CollateralValuationSubject {
            collateral_id: collateral_id.into(),
            source_happ: source_happ.into(),
            asset_id: asset_id.into(),
        };
        let request = CollateralValuationRequest {
            subject: subject.clone(),
            capability: capability.clone(),
        };
        let envelope = CollateralValuationEnvelope {
            subject,
            capability,
            outcome,
        };
        CollateralHealthEvidenceV2::new(request, envelope, policy(), 0, 20)
            .expect("test evidence must be valid at construction")
    }

    fn observed(
        collateral_id: &str,
        source_happ: &str,
        asset_id: &str,
        value: u64,
    ) -> CollateralHealthEvidenceV2 {
        evidence(
            collateral_id,
            source_happ,
            asset_id,
            CollateralValuationOutcome::Observed {
                value,
                observed_at_micros: 10,
            },
        )
    }

    fn unavailable(
        collateral_id: &str,
        source_happ: &str,
        asset_id: &str,
    ) -> CollateralHealthEvidenceV2 {
        evidence(
            collateral_id,
            source_happ,
            asset_id,
            CollateralValuationOutcome::Unavailable {
                reason: CollateralValuationFailure::ProviderUnavailable,
                attempted_at_micros: 10,
            },
        )
    }

    #[test]
    fn known_portfolio_uses_checked_sum_and_canonical_thresholds() {
        let components = vec![
            observed("c1", "property", "lot-1", 100),
            observed("c2", "energy", "solar-1", 100),
        ];
        let result = evaluate_current_portfolio_health(&components, &policy(), 160, 30)
            .expect("valid portfolio");

        assert_eq!(result.aggregate_value, Some(200));
        assert_eq!(result.ltv_ratio, Some(0.8));
        assert_eq!(result.assessment.status_label(), "Healthy");
        assert_eq!(result.risk_reducing_diversification_credit_bps, 0);
        assert!(result.unavailable_component_ids.is_empty());
    }

    #[test]
    fn distinct_source_domains_do_not_create_free_diversification_credit() {
        let components = vec![
            observed("c1", "property", "lot-1", 100),
            observed("c2", "crypto", "wallet-1", 100),
            observed("c3", "energy", "solar-1", 100),
            observed("c4", "agri", "grain-1", 100),
            observed("c5", "equipment", "machine-1", 100),
        ];
        let result = evaluate_current_portfolio_health(&components, &policy(), 384, 30)
            .expect("valid portfolio");

        assert_eq!(result.aggregate_value, Some(500));
        assert_eq!(result.ltv_ratio, Some(0.768));
        assert_eq!(result.risk_reducing_diversification_credit_bps, 0);
        assert_eq!(result.assessment.status_label(), "Healthy");
    }

    #[test]
    fn unavailable_component_makes_whole_portfolio_indeterminate() {
        let components = vec![
            observed("c1", "property", "lot-1", 100),
            unavailable("c2", "energy", "solar-1"),
        ];
        let result = evaluate_current_portfolio_health(&components, &policy(), 190, 30)
            .expect("unavailability is representable, not an evaluator failure");

        assert_eq!(result.aggregate_value, None);
        assert_eq!(result.ltv_ratio, None);
        assert_eq!(result.unavailable_component_ids, vec!["c2".to_string()]);
        assert_eq!(
            result.assessment,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::OracleUnavailable
            )
        );
        assert!(!result.is_liquidation_threshold_evidence_at_evaluation());
    }

    #[test]
    fn duplicate_finance_collateral_id_is_rejected() {
        let components = vec![
            observed("same", "property", "lot-1", 100),
            observed("same", "energy", "solar-1", 100),
        ];
        assert_eq!(
            evaluate_current_portfolio_health(&components, &policy(), 50, 30),
            Err(PortfolioHealthEvaluationError::DuplicateCollateralId {
                first_index: 0,
                duplicate_index: 1,
            })
        );
    }

    #[test]
    fn duplicate_authoritative_asset_is_rejected_even_under_different_local_ids() {
        let components = vec![
            observed("c1", "property", "lot-1", 100),
            observed("c2", "property", "lot-1", 100),
        ];
        assert_eq!(
            evaluate_current_portfolio_health(&components, &policy(), 50, 30),
            Err(PortfolioHealthEvaluationError::DuplicateAssetIdentity {
                first_index: 0,
                duplicate_index: 1,
            })
        );
    }

    #[test]
    fn aggregate_value_overflow_is_rejected_not_saturated() {
        let components = vec![
            observed("c1", "property", "lot-1", u64::MAX),
            observed("c2", "energy", "solar-1", 1),
        ];
        assert_eq!(
            evaluate_current_portfolio_health(&components, &policy(), 50, 30),
            Err(PortfolioHealthEvaluationError::AggregateValueOverflow)
        );
    }

    #[test]
    fn zero_total_observed_value_is_indeterminate_not_liquidation() {
        let components = vec![
            observed("c1", "property", "lot-1", 0),
            observed("c2", "energy", "solar-1", 0),
        ];
        let result = evaluate_current_portfolio_health(&components, &policy(), 100, 30)
            .expect("observed zero remains valid source evidence");

        assert_eq!(result.aggregate_value, Some(0));
        assert_eq!(result.ltv_ratio, None);
        assert_eq!(
            result.assessment,
            CollateralHealthAssessment::Indeterminate(
                CollateralHealthIndeterminateReason::ZeroValuation
            )
        );
        assert!(!result.is_liquidation_threshold_evidence_at_evaluation());
    }

    #[test]
    fn canonical_liquidation_boundary_is_preserved_without_bonus() {
        let components = vec![observed("c1", "property", "lot-1", 100)];
        let at_boundary = evaluate_current_portfolio_health(&components, &policy(), 95, 30)
            .expect("valid boundary portfolio");
        let above_boundary = evaluate_current_portfolio_health(&components, &policy(), 96, 30)
            .expect("valid above-boundary portfolio");

        assert_eq!(at_boundary.assessment.status_label(), "MarginCall");
        assert_eq!(above_boundary.assessment.status_label(), "Liquidation");
        assert!(!at_boundary.is_liquidation_threshold_evidence_at_evaluation());
        assert!(above_boundary.is_liquidation_threshold_evidence_at_evaluation());
    }

    #[test]
    fn stale_component_rejects_the_whole_evaluation() {
        let components = vec![observed("c1", "property", "lot-1", 100)];
        assert!(matches!(
            evaluate_current_portfolio_health(&components, &policy(), 50, 111),
            Err(PortfolioHealthEvaluationError::ComponentEvidence { index: 0, .. })
        ));
    }

    #[test]
    fn self_declared_component_policy_cannot_override_consumer_policy() {
        let mut permissive = policy();
        permissive.max_evidence_age_micros = 10_000;
        let capability = CollateralValuationCapability {
            provider_id: "portfolio-test-provider".into(),
            method: "value_asset".into(),
            protocol_version: COLLATERAL_VALUATION_PROTOCOL_VERSION,
        };
        let subject = CollateralValuationSubject {
            collateral_id: "c1".into(),
            source_happ: "property".into(),
            asset_id: "lot-1".into(),
        };
        let request = CollateralValuationRequest {
            subject: subject.clone(),
            capability: capability.clone(),
        };
        let envelope = CollateralValuationEnvelope {
            subject,
            capability,
            outcome: CollateralValuationOutcome::Observed {
                value: 100,
                observed_at_micros: 10,
            },
        };
        let component = CollateralHealthEvidenceV2::new(request, envelope, permissive, 0, 20)
            .expect("record is valid under its own persisted policy");

        assert!(matches!(
            evaluate_current_portfolio_health(&[component], &policy(), 50, 30),
            Err(PortfolioHealthEvaluationError::ComponentEvidence { index: 0, .. })
        ));
    }

    #[test]
    fn empty_portfolio_is_rejected() {
        assert_eq!(
            evaluate_current_portfolio_health(&[], &policy(), 0, 30),
            Err(PortfolioHealthEvaluationError::EmptyPortfolio)
        );
    }

    #[test]
    fn checked_evaluation_round_trip_preserves_zero_credit_and_time() {
        let components = vec![observed("c1", "property", "lot-1", 100)];
        let result = evaluate_current_portfolio_health(&components, &policy(), 50, 30)
            .expect("valid portfolio");
        let encoded = serde_json::to_string(&result).expect("serialize evaluation");
        let decoded: CheckedPortfolioHealthEvaluation =
            serde_json::from_str(&encoded).expect("deserialize evaluation");
        assert_eq!(decoded, result);
        assert_eq!(decoded.risk_reducing_diversification_credit_bps, 0);
        assert_eq!(decoded.evaluated_at_micros, 30);
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Procurement Substrate Guard
//!
//! Decision-layer reference model for applying substrate constraints to
//! procurement options.
//!
//! The guard never converts ecological condition into a monetary score and
//! never declares which option is morally superior. It only determines whether
//! each option is inside the declared operating envelope. Cost comparison may
//! then occur among options that remain eligible.

use super::substrate::{DistributionPurpose, GateDecision, SubstrateDimension, SubstrateLedger};
use serde::{Deserialize, Serialize};

/// A procurement option with a declared substrate policy scope.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProcurementOption {
    /// Stable option identifier.
    pub id: String,
    /// Monetary price in the supplied unit.
    pub price: u64,
    /// Price unit, such as SAP.
    pub price_unit: String,
    /// Policy evidence explaining the scope of review.
    pub policy_ref: String,
    /// Substrate dimensions required by this procurement policy.
    pub required_dimensions: Vec<SubstrateDimension>,
}

impl ProcurementOption {
    /// Structural validation.
    pub fn validate(&self) -> Result<(), String> {
        if self.id.trim().is_empty() {
            return Err("Procurement option ID cannot be empty".into());
        }
        if self.price_unit.trim().is_empty() {
            return Err("Procurement price unit cannot be empty".into());
        }
        if self.policy_ref.trim().is_empty() {
            return Err("Procurement policy reference cannot be empty".into());
        }
        if self.required_dimensions.is_empty() {
            return Err("Procurement option requires at least one substrate dimension".into());
        }
        Ok(())
    }
}

/// Result of evaluating one procurement option.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProcurementAssessment {
    /// Option identifier.
    pub option_id: String,
    /// Monetary price.
    pub price: u64,
    /// Gate decision.
    pub decision: GateDecision,
    /// Required dimensions considered.
    pub required_dimensions: Vec<SubstrateDimension>,
}

/// Assess one option against current substrate state.
pub fn assess_procurement_option(
    option: &ProcurementOption,
    ledger: &SubstrateLedger,
) -> Result<ProcurementAssessment, String> {
    option.validate()?;

    let mut required = option.required_dimensions.clone();
    required.sort();
    required.dedup();

    Ok(ProcurementAssessment {
        option_id: option.id.clone(),
        price: option.price,
        decision: ledger.gate(&required, DistributionPurpose::Discretionary),
        required_dimensions: required,
    })
}

/// Select the cheapest option among those allowed for discretionary action.
///
/// This is intentionally a constrained price comparison, not a universal
/// value function. An option blocked by substrate state is not made eligible
/// merely because it has a lower price.
pub fn cheapest_eligible_procurement(
    options: &[ProcurementOption],
    ledger: &SubstrateLedger,
) -> Result<ProcurementAssessment, String> {
    let mut eligible = Vec::new();

    for option in options {
        let assessment = assess_procurement_option(option, ledger)?;
        if matches!(
            assessment.decision,
            GateDecision::Allowed | GateDecision::AllowedWithWarning
        ) {
            eligible.push(assessment);
        }
    }

    eligible
        .into_iter()
        .min_by(|left, right| {
            left.price
                .cmp(&right.price)
                .then_with(|| left.option_id.cmp(&right.option_id))
        })
        .ok_or_else(|| "No procurement option is eligible under the declared substrate policy".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::substrate::{
        SubstrateAccount, SubstrateBoundary, SubstrateState,
    };

    fn ecological_account(current: i128) -> SubstrateAccount {
        SubstrateAccount::new(
            SubstrateDimension::Ecological,
            "condition-index",
            900,
            current,
            SubstrateBoundary::minimum(700, 50, true),
            2_000,
        )
    }

    fn option(id: &str, price: u64) -> ProcurementOption {
        ProcurementOption {
            id: id.into(),
            price,
            price_unit: "SAP".into(),
            policy_ref: "municipal:procurement:substrate-v1".into(),
            required_dimensions: vec![SubstrateDimension::Ecological],
        }
    }

    #[test]
    fn cheaper_noncompliant_option_does_not_win_on_price() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(ecological_account(650)).unwrap();

        let cheap = assess_procurement_option(&option("project-a", 800), &ledger).unwrap();
        let expensive = assess_procurement_option(&option("project-b", 1_000), &ledger).unwrap();

        assert_eq!(
            ledger
                .account(SubstrateDimension::Ecological)
                .unwrap()
                .state(),
            SubstrateState::Breached
        );
        assert_eq!(cheap.decision, GateDecision::Blocked);
        assert_eq!(expensive.decision, GateDecision::Blocked);
        assert!(expensive.price > cheap.price);
    }

    #[test]
    fn cheapest_eligible_option_is_selected_only_after_gating() {
        let mut ledger = SubstrateLedger::new();
        ledger.register_account(ecological_account(760)).unwrap();

        let chosen = cheapest_eligible_procurement(
            &[
                option("project-b", 1_000),
                option("project-a", 800),
            ],
            &ledger,
        )
        .unwrap();

        assert_eq!(chosen.option_id, "project-a");
        assert_eq!(chosen.decision, GateDecision::Allowed);
    }

    #[test]
    fn missing_substrate_evidence_blocks_price_selection() {
        let ledger = SubstrateLedger::new();

        let result = cheapest_eligible_procurement(
            &[option("project-a", 1)],
            &ledger,
        );

        assert!(result.is_err());
    }
}

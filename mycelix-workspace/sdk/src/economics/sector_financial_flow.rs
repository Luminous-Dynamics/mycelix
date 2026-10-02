// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Sector-level financial-claim flow projection.
//!
//! This matrix is intentionally distinct from the cash/liquidity transaction
//! matrix. It records changes in contractual financial claims and obligations
//! so deferred trade settlement is visible without being mislabeled as cash.

use serde::{Deserialize, Serialize};

use super::sector_balance::SectorAssignment;
use super::sector_flow::EconomicSector;
use super::stock_flow::ActorId;
use super::transition::EconomicTransition;

/// Category of a sector-to-sector financial-claim flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FinancialFlowCategory {
    LoanCreation,
    DebtRepayment,
    TradeCreditExtension,
    TradeCreditSettlement,
}

/// One directed financial-claim transition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SectorFinancialFlow {
    pub from: EconomicSector,
    pub to: EconomicSector,
    pub category: FinancialFlowCategory,
    pub amount: i128,
}

impl SectorFinancialFlow {
    pub fn new(
        from: EconomicSector,
        to: EconomicSector,
        category: FinancialFlowCategory,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("sector financial flow amount must be positive".into());
        }
        Ok(Self { from, to, category, amount })
    }
}

/// A period of sector-level financial-claim changes.
///
/// This is not a cash-flow statement. In particular, TradeCreditExtension
/// records the creation of a receivable/payable pair, not a cash receipt.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SectorFinancialFlowMatrix {
    pub flows: Vec<SectorFinancialFlow>,
}

impl SectorFinancialFlowMatrix {
    pub fn push(&mut self, flow: SectorFinancialFlow) {
        self.flows.push(flow);
    }

    /// Net signed financial-claim flow received by a sector.
    pub fn net_flow(&self, sector: EconomicSector) -> i128 {
        self.flows.iter().fold(0, |sum, flow| {
            if flow.to == sector && flow.from != sector {
                sum + flow.amount
            } else if flow.from == sector && flow.to != sector {
                sum - flow.amount
            } else {
                sum
            }
        })
    }

    /// Every inter-sector financial-claim transition has a counterpart.
    pub fn clears(&self) -> bool {
        [
            EconomicSector::Household,
            EconomicSector::Firm,
            EconomicSector::Bank,
            EconomicSector::Commons,
            EconomicSector::Public,
            EconomicSector::External,
        ]
        .iter()
        .map(|sector| self.net_flow(*sector))
        .sum::<i128>()
            == 0
    }

    pub fn gross_flow_volume(&self) -> i128 {
        self.flows.iter().map(|flow| flow.amount).sum()
    }

    /// Derive financial-claim flows from the authoritative transition log.
    ///
    /// Trade credit is oriented buyer -> seller because the buyer acquires a
    /// payable while the seller acquires the corresponding receivable.
    pub fn from_transitions(
        transitions: &[EconomicTransition],
        assignments: &[SectorAssignment],
    ) -> Result<Self, String> {
        let sector_for = |actor: &str| {
            let matching = assignments
                .iter()
                .filter(|assignment| assignment.actor == actor)
                .collect::<Vec<_>>();
            if matching.len() != 1 {
                return Err(format!(
                    "actor {actor} must have exactly one sector assignment"
                ));
            }
            Ok(matching[0].sector)
        };

        let mut matrix = Self::default();
        for transition in transitions {
            let (from, to, category, amount) = match transition {
                EconomicTransition::CreditCreation(credit) => (
                    sector_for(&credit.lender)?,
                    sector_for(&credit.borrower)?,
                    FinancialFlowCategory::LoanCreation,
                    credit.amount,
                ),
                EconomicTransition::DebtRepayment(repayment) => (
                    sector_for(&repayment.borrower)?,
                    sector_for(&repayment.lender)?,
                    FinancialFlowCategory::DebtRepayment,
                    repayment.amount,
                ),
                EconomicTransition::TradeCreditSale(sale) => (
                    sector_for(&sale.buyer)?,
                    sector_for(&sale.seller)?,
                    FinancialFlowCategory::TradeCreditExtension,
                    sale.consideration,
                ),
                EconomicTransition::TradeCreditSettlement(settlement) => (
                    sector_for(&settlement.buyer)?,
                    sector_for(&settlement.seller)?,
                    FinancialFlowCategory::TradeCreditSettlement,
                    settlement.amount,
                ),
                _ => continue,
            };
            matrix.push(SectorFinancialFlow::new(from, to, category, amount)?);
        }
        Ok(matrix)
    }

    /// Validate exact agreement with the authoritative transition log and
    /// exact-one sector assignment coverage.
    pub fn validate_against(
        &self,
        state: &super::stock_flow::EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<(), String> {
        if assignments.len() != state.actors.len()
            || state.actors.iter().any(|actor| {
                assignments
                    .iter()
                    .filter(|assignment| assignment.actor == actor.actor)
                    .count()
                    != 1
            })
            || assignments.iter().any(|assignment| {
                state
                    .actors
                    .iter()
                    .filter(|actor| actor.actor == assignment.actor)
                    .count()
                    != 1
            })
        {
            return Err("sector assignments must cover each economic actor exactly once".into());
        }

        if !self.clears() {
            return Err("sector financial-flow matrix does not clear".into());
        }

        let expected = Self::from_transitions(transitions, assignments)?;
        if self.flows != expected.flows {
            return Err("sector financial-flow matrix does not match transition projection".into());
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{
        ActorBalanceSheet, CreditCreation, EconomicState, TradeCreditSale,
        TradeCreditSettlement, DebtRepayment,
    };

    fn assignments() -> Vec<SectorAssignment> {
        vec![
            SectorAssignment { actor: "bank".into(), sector: EconomicSector::Bank },
            SectorAssignment { actor: "firm".into(), sector: EconomicSector::Firm },
            SectorAssignment { actor: "household".into(), sector: EconomicSector::Household },
        ]
    }

    #[test]
    fn financial_matrix_exposes_deferred_trade_claims_without_cash_misclassification() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("firm"),
            ActorBalanceSheet::new("household"),
        ]);
        let transitions = vec![
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "firm", 100).unwrap(),
            ),
            EconomicTransition::TradeCreditSale(
                TradeCreditSale::new("firm", "household", 2, 40).unwrap(),
            ),
            EconomicTransition::TradeCreditSettlement(
                TradeCreditSettlement::new("firm", "household", 15).unwrap(),
            ),
            EconomicTransition::DebtRepayment(
                DebtRepayment::new("bank", "firm", 10).unwrap(),
            ),
        ];

        let matrix =
            SectorFinancialFlowMatrix::from_transitions(&transitions, &assignments()).unwrap();

        assert_eq!(matrix.flows.len(), 4);
        assert_eq!(matrix.flows[1].from, EconomicSector::Household);
        assert_eq!(matrix.flows[1].to, EconomicSector::Firm);
        assert_eq!(
            matrix.flows[1].category,
            FinancialFlowCategory::TradeCreditExtension
        );
        assert!(matrix.clears());
        matrix
            .validate_against(&state, &assignments(), &transitions)
            .unwrap();
    }

    #[test]
    fn financial_matrix_rejects_duplicate_or_missing_assignments() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("firm"),
            ActorBalanceSheet::new("household"),
        ]);
        let transitions = vec![EconomicTransition::TradeCreditSale(
            TradeCreditSale::new("firm", "household", 1, 10).unwrap(),
        )];

        let duplicate = vec![
            SectorAssignment { actor: "firm".into(), sector: EconomicSector::Firm },
            SectorAssignment { actor: "firm".into(), sector: EconomicSector::Household },
        ];
        assert!(SectorFinancialFlowMatrix::from_transitions(
            &transitions,
            &duplicate
        )
        .is_err());

        assert!(SectorFinancialFlowMatrix::from_transitions(
            &transitions,
            &[SectorAssignment { actor: "firm".into(), sector: EconomicSector::Firm }]
        )
        .is_err());

        assert!(SectorFinancialFlowMatrix::default()
            .validate_against(&state, &assignments(), &transitions)
            .is_err());
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Sector-level stock-flow accounting.
//!
//! A Godley-style transaction matrix is the next layer above individual balance
//! sheets. Each transaction has an explicit source sector, destination sector,
//! and signed amount. This makes it possible to reconcile an economic scenario
//! before adding behavioral equations.

use serde::{Deserialize, Serialize};

use super::stock_flow::{ActorId, EconomicState};

/// Coarse sector classification. Implementations may map many actors into one sector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum EconomicSector {
    Household,
    Firm,
    Bank,
    Commons,
    Public,
    External,
}

/// Category of a sector-to-sector flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FlowCategory {
    Consumption,
    Wage,
    Investment,
    Tax,
    Transfer,
    Interest,
    LoanCreation,
    DebtRepayment,
    TradeCreditSettlement,
    Resource,
    Other,
}

/// A single entry in a transaction-flow matrix.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SectorFlow {
    pub from: EconomicSector,
    pub to: EconomicSector,
    pub category: FlowCategory,
    pub amount: i128,
}

impl SectorFlow {
    pub fn new(
        from: EconomicSector,
        to: EconomicSector,
        category: FlowCategory,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("sector flow amount must be positive".into());
        }
        Ok(Self { from, to, category, amount })
    }
}

/// A period of sectoral transactions.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SectorTransactionMatrix {
    pub flows: Vec<SectorFlow>,
}

impl SectorTransactionMatrix {
    pub fn push(&mut self, flow: SectorFlow) {
        self.flows.push(flow);
    }

    /// Net monetary flow received by a sector.
    pub fn net_flow(&self, sector: EconomicSector) -> i128 {
        self.try_net_flow(sector)
            .expect("sector net-flow overflow")
    }

    /// Checked sector net flow. Returns an error on arithmetic overflow.
    pub fn try_net_flow(&self, sector: EconomicSector) -> Result<i128, String> {
        self.flows.iter().try_fold(0i128, |sum, flow| {
            if flow.to == sector && flow.from != sector {
                sum.checked_add(flow.amount)
            } else if flow.from == sector && flow.to != sector {
                sum.checked_sub(flow.amount)
            } else {
                Some(sum)
            }
            .ok_or_else(|| "sector net-flow overflow".to_string())
        })
    }

    /// Every external sectoral transfer must sum to zero. Overflow fails closed.
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
        .try_fold(0i128, |sum, sector| {
            sum.checked_add(self.try_net_flow(*sector).ok()?)
        })
        == Some(0)
    }

    /// Checked sum of all positive flow volume.
    pub fn try_gross_flow_volume(&self) -> Result<i128, String> {
        self.flows.iter().try_fold(0i128, |sum, flow| {
            sum.checked_add(flow.amount)
                .ok_or_else(|| "sector gross-flow overflow".to_string())
        })
    }

    /// Sum of all positive flow volume.
    pub fn gross_flow_volume(&self) -> i128 {
        self.try_gross_flow_volume()
            .expect("sector gross-flow overflow")
    }

    /// Derive the sector transaction matrix directly from the authoritative
    /// ordered transition log.
    ///
    /// This keeps the transaction-flow matrix as a projection of the same
    /// transition evidence used by the stock-flow reconciliation layer rather
    /// than introducing a second, independently-authored flow ledger.
    pub fn from_transitions(
        transitions: &[super::transition::EconomicTransition],
        actors: &[(ActorId, EconomicSector)],
    ) -> Result<Self, String> {
        let sector_for = |actor: &str| {
            let matching = actors
                .iter()
                .filter(|(id, _)| id == actor)
                .collect::<Vec<_>>();
            if matching.len() != 1 {
                return Err(format!(
                    "actor {actor} must have exactly one sector assignment"
                ));
            }
            Ok(matching[0].1)
        };

        let mut matrix = Self::default();
        for transition in transitions {
            let (from, to, category, amount) = match transition {
                super::transition::EconomicTransition::Production(_) => continue,
                super::transition::EconomicTransition::InventoryTransfer(_) => continue,
                super::transition::EconomicTransition::InventoryConsumption(_) => continue,
                super::transition::EconomicTransition::InventoryCostAddition(_) => continue,
                super::transition::EconomicTransition::InventoryCostRelief(_) => continue,
                super::transition::EconomicTransition::Depreciation(_) => continue,
                // Trade credit is a non-cash balance-sheet transaction here;
                // the receivable/payable movement is reconciled in the stock
                // matrix rather than misrepresented as a deposit flow.
                super::transition::EconomicTransition::TradeCreditSale(_) => continue,
                super::transition::EconomicTransition::TradeCreditSettlement(settlement) => (
                    sector_for(&settlement.buyer)?,
                    sector_for(&settlement.seller)?,
                    FlowCategory::TradeCreditSettlement,
                    settlement.amount,
                ),
                super::transition::EconomicTransition::GoodsSale(sale) => (
                    sector_for(&sale.seller)?,
                    sector_for(&sale.buyer)?,
                    FlowCategory::Other,
                    sale.consideration,
                )
                super::transition::EconomicTransition::MonetaryTransfer(flow) => (
                    sector_for(&flow.from)?,
                    sector_for(&flow.to)?,
                    FlowCategory::Other,
                    flow.amount,
                ),
                super::transition::EconomicTransition::IncomeTransfer(flow) => (
                    sector_for(&flow.payer)?,
                    sector_for(&flow.recipient)?,
                    match flow.category {
                        super::stock_flow::EconomicFlowCategory::Wage => FlowCategory::Wage,
                        super::stock_flow::EconomicFlowCategory::Interest => FlowCategory::Interest,
                        super::stock_flow::EconomicFlowCategory::Tax => FlowCategory::Tax,
                        super::stock_flow::EconomicFlowCategory::Transfer => FlowCategory::Transfer,
                        super::stock_flow::EconomicFlowCategory::Consumption => FlowCategory::Consumption,
                        super::stock_flow::EconomicFlowCategory::Investment => FlowCategory::Investment,
                    },
                    flow.amount,
                ),
                super::transition::EconomicTransition::CapitalInvestment(investment) => (
                    sector_for(&investment.buyer)?,
                    sector_for(&investment.producer)?,
                    FlowCategory::Investment,
                    investment.amount,
                ),
                super::transition::EconomicTransition::CreditCreation(credit) => (
                    sector_for(&credit.lender)?,
                    sector_for(&credit.borrower)?,
                    FlowCategory::LoanCreation,
                    credit.amount,
                ),
                super::transition::EconomicTransition::DebtRepayment(repayment) => (
                    sector_for(&repayment.borrower)?,
                    sector_for(&repayment.lender)?,
                    FlowCategory::DebtRepayment,
                    repayment.amount,
                ),
            };
            matrix.push(SectorFlow::new(from, to, category, amount)?);
        }
        Ok(matrix)
    }

    /// Validate that this matrix is an exact projection of the authoritative
    /// transition log and that all sectoral external flows clear.
    pub fn validate_against(
        &self,
        state: &EconomicState,
        actors: &[(ActorId, EconomicSector)],
        transitions: &[super::transition::EconomicTransition],
    ) -> Result<(), String> {
        if state.actors.len() != actors.len()
            || state.actors.iter().any(|actor| {
                actors.iter().filter(|(id, _)| id == &actor.actor).count() != 1
            })
            || actors.iter().any(|(id, _)| {
                state.actors.iter().filter(|actor| &actor.actor == id).count() != 1
            })
        {
            return Err("sector assignments must cover each economic actor exactly once".into());
        }
        if !self.clears() {
            return Err("sector transaction matrix does not clear".into());
        }

        let expected = Self::from_transitions(transitions, actors)?;
        if self.flows != expected.flows {
            return Err("sector transaction matrix does not match transition-derived flows".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sector_flows_clear() {
        let mut m = SectorTransactionMatrix::default();
        m.push(SectorFlow::new(
            EconomicSector::Firm,
            EconomicSector::Household,
            FlowCategory::Wage,
            100,
        ).unwrap());
        m.push(SectorFlow::new(
            EconomicSector::Household,
            EconomicSector::Firm,
            FlowCategory::Consumption,
            100,
        ).unwrap());

        assert!(m.clears());
        assert_eq!(m.net_flow(EconomicSector::Household), 0);
        assert_eq!(m.gross_flow_volume(), 200);
    }

    #[test]
    fn transition_projection_is_deterministic_and_semantic() {
        use super::super::stock_flow::{
            CapitalInvestment, CreditCreation, DebtRepayment, EconomicFlowCategory,
            IncomeTransfer,
        };
        use super::super::transition::EconomicTransition;

        let actors = vec![
            ("bank".to_string(), EconomicSector::Bank),
            ("firm".to_string(), EconomicSector::Firm),
            ("household".to_string(), EconomicSector::Household),
        ];
        let transitions = vec![
            EconomicTransition::IncomeTransfer(
                IncomeTransfer::with_category(
                    "firm",
                    "household",
                    100,
                    EconomicFlowCategory::Wage,
                )
                .unwrap(),
            ),
            EconomicTransition::CapitalInvestment(
                CapitalInvestment::new("firm", "household", 25).unwrap(),
            ),
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "firm", 50).unwrap(),
            ),
            EconomicTransition::DebtRepayment(
                DebtRepayment::new("bank", "firm", 10).unwrap(),
            ),
        ];

        let matrix = SectorTransactionMatrix::from_transitions(&transitions, &actors).unwrap();
        assert_eq!(matrix.flows.len(), 4);
        assert_eq!(matrix.flows[0].category, FlowCategory::Wage);
        assert_eq!(matrix.flows[1].category, FlowCategory::Investment);
        assert_eq!(matrix.flows[2].category, FlowCategory::LoanCreation);
        assert_eq!(matrix.flows[3].category, FlowCategory::DebtRepayment);

        assert!(matrix.clears());

        let state = EconomicState::new(vec![
            super::super::stock_flow::ActorBalanceSheet::new("bank"),
            super::super::stock_flow::ActorBalanceSheet::new("firm"),
            super::super::stock_flow::ActorBalanceSheet::new("household"),
        ]);
        matrix.validate_against(&state, &actors, &transitions).unwrap();

        let mut tampered = matrix.clone();
        tampered.flows[0].amount = 99;
        assert!(tampered.validate_against(&state, &actors, &transitions).is_err());
    }

    #[test]
    fn goods_sale_projection_resolves_actor_ids_to_sectors() {
        use super::super::stock_flow::{ActorBalanceSheet, GoodsSale};
        use super::super::transition::EconomicTransition;

        let actors = vec![
            ("firm".to_string(), EconomicSector::Firm),
            ("household".to_string(), EconomicSector::Household),
        ];
        let transitions = vec![EconomicTransition::GoodsSale(
            GoodsSale::new("firm", "household", 2, 40).unwrap(),
        )];

        let matrix = SectorTransactionMatrix::from_transitions(&transitions, &actors).unwrap();
        assert_eq!(matrix.flows.len(), 1);
        assert_eq!(matrix.flows[0].from, EconomicSector::Firm);
        assert_eq!(matrix.flows[0].to, EconomicSector::Household);
        assert_eq!(matrix.flows[0].category, FlowCategory::Other);
        assert_eq!(matrix.flows[0].amount, 40);

        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("firm"),
            ActorBalanceSheet::new("household"),
        ]);
        matrix
            .validate_against(&state, &actors, &transitions)
            .unwrap();
    }

    #[test]
    fn clearing_fails_closed_on_overflow() {
        let mut m = SectorTransactionMatrix::default();
        m.push(SectorFlow {
            from: EconomicSector::Firm,
            to: EconomicSector::Household,
            category: FlowCategory::Other,
            amount: i128::MAX,
        });
        m.push(SectorFlow {
            from: EconomicSector::Firm,
            to: EconomicSector::Public,
            category: FlowCategory::Other,
            amount: 1,
        });
        assert!(!m.clears());
        assert!(m.try_gross_flow_volume().is_err());
    }

    #[test]
    fn trade_credit_settlement_is_projected_as_monetary_sector_flow() {
        use super::super::stock_flow::TradeCreditSettlement;
        use super::super::transition::EconomicTransition;

        let actors = vec![
            ("firm".to_string(), EconomicSector::Firm),
            ("household".to_string(), EconomicSector::Household),
        ];
        let transitions = vec![EconomicTransition::TradeCreditSettlement(
            TradeCreditSettlement::new("firm", "household", 15).unwrap(),
        )];

        let matrix = SectorTransactionMatrix::from_transitions(&transitions, &actors).unwrap();
        assert_eq!(matrix.flows.len(), 1);
        assert_eq!(matrix.flows[0].from, EconomicSector::Household);
        assert_eq!(matrix.flows[0].to, EconomicSector::Firm);
        assert_eq!(
            matrix.flows[0].category,
            FlowCategory::TradeCreditSettlement
        );
        assert!(matrix.clears());
    }

    #[test]
    fn unbalanced_matrix_is_rejected() {
        let mut m = SectorTransactionMatrix::default();
        m.push(SectorFlow::new(
            EconomicSector::Firm,
            EconomicSector::Household,
            FlowCategory::Wage,
            100,
        ).unwrap());

        assert!(!m.clears());
    }
}

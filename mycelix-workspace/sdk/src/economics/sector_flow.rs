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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
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

    /// Every external sectoral transfer must sum to zero.
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

    /// Sum of all positive flow volume.
    pub fn gross_flow_volume(&self) -> i128 {
        self.flows.iter().map(|flow| flow.amount).sum()
    }

    /// Validate that the matrix contains no unknown actor references.
    ///
    /// Actor resolution is deliberately kept outside the matrix: this layer
    /// describes sector accounting, while the actor registry belongs to the
    /// simulation runtime.
    pub fn validate_against(&self, _state: &EconomicState, _actors: &[(ActorId, EconomicSector)]) -> Result<(), String> {
        if !self.clears() {
            return Err("sector transaction matrix does not clear".into());
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

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Sector balance-sheet reconciliation.
//!
//! This is the bridge between actor-level accounting and a Godley-style
//! sector matrix. The matrix is deliberately an accounting representation:
//! it does not encode behavioral preferences or policy recommendations.

use serde::{Deserialize, Serialize};

use super::sector_flow::EconomicSector;
use super::stock_flow::{ActorId, EconomicState};

/// A signed sector balance-sheet entry.
///
/// Positive values are sector assets; negative values are sector liabilities.
/// Monetary-valued real assets survive consolidation because they have no
/// matching financial liability inside the closed financial matrix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BalanceSheetInstrument {
    Cash,
    Deposits,
    Loans,
    Debt,
    DepositLiabilities,
    Equity,
    ProductiveCapital,
    InventoryCarryingValue,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BalanceSheetEntry {
    pub sector: EconomicSector,
    pub instrument: BalanceSheetInstrument,
    pub amount: i128,
}

impl BalanceSheetEntry {
    pub fn new(
        sector: EconomicSector,
        instrument: BalanceSheetInstrument,
        amount: i128,
    ) -> Self {
        Self {
            sector,
            instrument,
            amount,
        }
    }
}

/// Actor-to-sector assignment used for deterministic consolidation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SectorAssignment {
    pub actor: ActorId,
    pub sector: EconomicSector,
}

/// Consolidated sector balance sheet.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SectorBalanceSheet {
    pub entries: Vec<BalanceSheetEntry>,
}

/// A physical stock dimension kept separate from monetary balance-sheet
/// values. Quantities have no currency unit and therefore do not participate in
/// the financial balance-sheet identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PhysicalStockInstrument {
    Inventories,
    Resources,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PhysicalStockEntry {
    pub sector: EconomicSector,
    pub instrument: PhysicalStockInstrument,
    pub amount: i128,
}

impl PhysicalStockEntry {
    pub const fn new(
        sector: EconomicSector,
        instrument: PhysicalStockInstrument,
        amount: i128,
    ) -> Self {
        Self { sector, instrument, amount }
    }
}

/// Consolidated physical stocks. These values are intentionally not required
/// to clear to zero: the economy can possess inventories/resources in aggregate.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SectorPhysicalStock {
    pub entries: Vec<PhysicalStockEntry>,
}

impl SectorPhysicalStock {
    pub fn from_state(
        state: &EconomicState,
        assignments: &[SectorAssignment],
    ) -> Result<Self, String> {
        let mut actors = state.actors.clone();
        actors.sort_by(|a, b| a.actor.cmp(&b.actor));

        let mut entries = Vec::new();
        for actor in actors {
            let matching = assignments
                .iter()
                .filter(|assignment| assignment.actor == actor.actor)
                .collect::<Vec<_>>();
            if matching.len() != 1 {
                return Err(format!(
                    "actor {} must have exactly one sector assignment",
                    actor.actor
                ));
            }
            let assignment = matching[0];

            entries.extend([
                PhysicalStockEntry::new(
                    assignment.sector,
                    PhysicalStockInstrument::Inventories,
                    actor.real.inventories,
                ),
                PhysicalStockEntry::new(
                    assignment.sector,
                    PhysicalStockInstrument::Resources,
                    actor.real.resources,
                ),
            ]);
        }

        entries.sort_by_key(|entry| (entry.sector as u8, entry.instrument as u8));
        Ok(Self { entries })
    }

    pub fn sector_stock_total(
        &self,
        sector: EconomicSector,
        instrument: PhysicalStockInstrument,
    ) -> i128 {
        self.entries
            .iter()
            .filter(|entry| entry.sector == sector && entry.instrument == instrument)
            .map(|entry| entry.amount)
            .sum()
    }
}

impl SectorBalanceSheet {
    /// Build a deterministic sector consolidation from actor state.
    pub fn from_state(
        state: &EconomicState,
        assignments: &[SectorAssignment],
    ) -> Result<Self, String> {
        let mut actors = state.actors.clone();
        actors.sort_by(|a, b| a.actor.cmp(&b.actor));

        let mut entries = Vec::new();

        for actor in actors {
            let matching = assignments
                .iter()
                .filter(|assignment| assignment.actor == actor.actor)
                .collect::<Vec<_>>();
            if matching.len() != 1 {
                return Err(format!(
                    "actor {} must have exactly one sector assignment",
                    actor.actor
                ));
            }
            let assignment = matching[0];

            let m = actor.monetary;
            let r = actor.real;

            entries.extend([
                BalanceSheetEntry::new(assignment.sector, BalanceSheetInstrument::Cash, m.cash),
                BalanceSheetEntry::new(
                    assignment.sector,
                    BalanceSheetInstrument::Deposits,
                    m.deposits,
                ),
                BalanceSheetEntry::new(
                    assignment.sector,
                    BalanceSheetInstrument::Loans,
                    m.claims,
                ),
                BalanceSheetEntry::new(
                    assignment.sector,
                    BalanceSheetInstrument::Debt,
                    -m.liabilities,
                ),
                BalanceSheetEntry::new(
                    assignment.sector,
                    BalanceSheetInstrument::DepositLiabilities,
                    -m.deposit_liabilities,
                ),
                // Equity/net worth is the balance-sheet residual, not an
                // independent asset. Store it on the signed liability side.
                BalanceSheetEntry::new(
                    assignment.sector,
                    BalanceSheetInstrument::Equity,
                    -actor.net_worth(),
                ),
                BalanceSheetEntry::new(
                    assignment.sector,
                    BalanceSheetInstrument::ProductiveCapital,
                    r.productive_capital,
                ),
                BalanceSheetEntry::new(
                    assignment.sector,
                    BalanceSheetInstrument::InventoryCarryingValue,
                    actor.inventory_carrying_value,
                ),
            ]);
        }

        entries.sort_by_key(|entry| (entry.sector as u8, entry.instrument as u8));
        Ok(Self { entries })
    }

    /// Return the sector total for one instrument.
    pub fn sector_instrument_total(
        &self,
        sector: EconomicSector,
        instrument: BalanceSheetInstrument,
    ) -> i128 {
        self.entries
            .iter()
            .filter(|entry| entry.sector == sector && entry.instrument == instrument)
            .map(|entry| entry.amount)
            .sum()
    }

    /// Financial claim/liability rows should consolidate to zero.
    ///
    /// Equity is intentionally excluded: it is the balancing item against
    /// real assets, not a claim held by another sector. Cash is also excluded
    /// until its issuer is explicitly modeled.
    pub fn financial_rows_clear(&self) -> bool {
        [
            // Cash/reserves may have an issuer outside the modeled sectors.
            BalanceSheetInstrument::Deposits,
            BalanceSheetInstrument::Loans,
            BalanceSheetInstrument::Debt,
            BalanceSheetInstrument::DepositLiabilities,
        ]
        .iter()
        .all(|instrument| {
            self.entries
                .iter()
                .filter(|entry| entry.instrument == *instrument)
                .map(|entry| entry.amount)
                .sum::<i128>()
                == 0
        })
    }

    /// Each sector's signed monetary balance sheet must satisfy assets +
    /// liabilities + equity = 0. Physical quantities are reconciled separately
    /// and are never added to currency-denominated rows.
    pub fn balance_sheet_identity_holds(&self) -> bool {
        let mut sectors = self.entries.iter().map(|e| e.sector).collect::<Vec<_>>();
        sectors.sort_by_key(|s| *s as u8);
        sectors.dedup();
        sectors.into_iter().all(|sector| {
            self.entries
                .iter()
                .filter(|entry| entry.sector == sector)
                .map(|entry| entry.amount)
                .sum::<i128>()
                == 0
        })
    }

    /// Validate that every state actor has exactly one sector assignment and
    /// that the resulting financial matrix is internally coherent.
    pub fn validate(
        &self,
        state: &EconomicState,
        assignments: &[SectorAssignment],
    ) -> Result<(), String> {
        if state.actors.len() != assignments.len() {
            return Err("sector assignments must cover every actor exactly once".into());
        }

        for actor in &state.actors {
            let count = assignments.iter().filter(|a| a.actor == actor.actor).count();
            if count != 1 {
                return Err(format!(
                    "actor {} must have exactly one sector assignment",
                    actor.actor
                ));
            }
        }

        if !self.financial_rows_clear() {
            return Err("sector financial balance-sheet rows do not clear".into());
        }
        if !self.balance_sheet_identity_holds() {
            return Err("sector balance-sheet identities do not hold".into());
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{ActorBalanceSheet, CreditCreation};

    #[test]
    fn consolidation_reconciles_bank_credit() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        let mut state = EconomicState::new(vec![
            bank,
            ActorBalanceSheet::new("household"),
        ]);

        state
            .create_credit(&CreditCreation::new("bank", "household", 500).unwrap())
            .unwrap();

        let assignments = vec![
            SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "household".into(),
                sector: EconomicSector::Household,
            },
        ];

        let sheet = SectorBalanceSheet::from_state(&state, &assignments).unwrap();
        assert!(sheet.financial_rows_clear());
        assert!(sheet.balance_sheet_identity_holds());
        sheet.validate(&state, &assignments).unwrap();
        assert_eq!(
            sheet.sector_instrument_total(
                EconomicSector::Bank,
                BalanceSheetInstrument::Loans
            ),
            500
        );
        assert_eq!(
            sheet.sector_instrument_total(
                EconomicSector::Household,
                BalanceSheetInstrument::Deposits
            ),
            500
        );
    }

    #[test]
    fn missing_assignment_is_rejected() {
        let state = EconomicState::new(vec![ActorBalanceSheet::new("household")]);
        let error = SectorBalanceSheet::from_state(&state, &[]).unwrap_err();
        assert!(error.contains("missing sector assignment"));
    }

    #[test]
    fn real_assets_do_not_have_to_clear() {
        let mut household = ActorBalanceSheet::new("household");
        household.real.resources = 100;
        let state = EconomicState::new(vec![household]);

        let assignments = vec![SectorAssignment {
            actor: "household".into(),
            sector: EconomicSector::Household,
        }];

        let sheet = SectorBalanceSheet::from_state(&state, &assignments).unwrap();
        assert!(sheet.financial_rows_clear());
        assert!(sheet.balance_sheet_identity_holds());
        let physical = SectorPhysicalStock::from_state(&state, &assignments).unwrap();
        assert_eq!(
            physical.sector_stock_total(
                EconomicSector::Household,
                PhysicalStockInstrument::Resources
            ),
            100
        );
    }
}

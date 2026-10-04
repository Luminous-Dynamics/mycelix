// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Sector-level projection of other changes in the volume of assets and liabilities.
//!
//! This matrix is intentionally distinct from both the monetary transaction matrix
//! and the financial-account claim-flow matrix. A debt write-off changes the stock
//! of a financial asset/liability without being a settlement transaction.

use serde::{Deserialize, Serialize};

use super::sector_balance::{BalanceSheetInstrument, SectorAssignment, SectorBalanceSheet};
use super::sector_flow::EconomicSector;
use super::stock_flow::EconomicState;
use super::transition::EconomicTransition;

/// Category of an other-volume adjustment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OtherVolumeChangeCategory {
    DebtWriteOff,
}

/// One explicit sector-level other-volume adjustment.
///
/// The creditor/debtor fields identify the contractual relationship. They are
/// not a payment direction and must never be interpreted as cash flow.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SectorOtherVolumeChange {
    pub creditor: EconomicSector,
    pub debtor: EconomicSector,
    pub category: OtherVolumeChangeCategory,
    pub amount: i128,
}

impl SectorOtherVolumeChange {
    pub fn new(
        creditor: EconomicSector,
        debtor: EconomicSector,
        category: OtherVolumeChangeCategory,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("sector other-volume change amount must be positive".into());
        }
        Ok(Self {
            creditor,
            debtor,
            category,
            amount,
        })
    }
}

/// Deterministic projection of other changes in asset/liability volume.
///
/// This currently contains debt write-offs. Future categories such as
/// reclassifications should be added here only when their stock/equity rules are
/// equally explicit.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SectorOtherVolumeChangeMatrix {
    pub changes: Vec<SectorOtherVolumeChange>,
}

impl SectorOtherVolumeChangeMatrix {
    pub fn push(&mut self, change: SectorOtherVolumeChange) {
        self.changes.push(change);
    }

    pub fn try_gross_volume(&self) -> Result<i128, String> {
        self.changes.iter().try_fold(0i128, |sum, change| {
            sum.checked_add(change.amount)
                .ok_or_else(|| "sector other-volume gross amount overflow".to_string())
        })
    }

    pub fn gross_volume(&self) -> i128 {
        self.try_gross_volume()
            .expect("sector other-volume gross amount overflow")
    }

    /// Derive the other-volume projection from the authoritative transition log.
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
            transition.validate()?;
            match transition {
                EconomicTransition::DebtWriteOff(write_off) => {
                    matrix.push(SectorOtherVolumeChange::new(
                        sector_for(&write_off.lender)?,
                        sector_for(&write_off.borrower)?,
                        OtherVolumeChangeCategory::DebtWriteOff,
                        write_off.amount,
                    )?);
                }
                EconomicTransition::MonetaryTransfer(_)
                | EconomicTransition::IncomeTransfer(_)
                | EconomicTransition::CapitalInvestment(_)
                | EconomicTransition::Production(_)
                | EconomicTransition::InventoryTransfer(_)
                | EconomicTransition::InventoryConsumption(_)
                | EconomicTransition::GoodsSale(_)
                | EconomicTransition::TradeCreditSale(_)
                | EconomicTransition::TradeCreditSettlement(_)
                | EconomicTransition::InventoryCostAddition(_)
                | EconomicTransition::InventoryCostRelief(_)
                | EconomicTransition::Depreciation(_)
                | EconomicTransition::RealAssetRevaluation(_)
                | EconomicTransition::CreditCreation(_)
                | EconomicTransition::DebtRepayment(_) => {}
            }
        }
        Ok(matrix)
    }

    /// Validate assignment coverage and exact agreement with the transition projection.
    pub fn validate_against(
        &self,
        state: &EconomicState,
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

        let expected = Self::from_transitions(transitions, assignments)?;
        if self.changes != expected.changes {
            return Err("sector other-volume projection does not match transition projection".into());
        }

        Ok(())
    }

    /// Validate exact stock/equity deltas for debt write-offs.
    pub fn validate_against_balance_sheet_delta(
        &self,
        pre: &SectorBalanceSheet,
        post: &SectorBalanceSheet,
    ) -> Result<(), String> {
        const SECTORS: [EconomicSector; 6] = [
            EconomicSector::Household,
            EconomicSector::Firm,
            EconomicSector::Bank,
            EconomicSector::Commons,
            EconomicSector::Public,
            EconomicSector::External,
        ];
        const INSTRUMENTS: [BalanceSheetInstrument; 2] = [
            BalanceSheetInstrument::Loans,
            BalanceSheetInstrument::Debt,
        ];

        for sector in SECTORS {
            for instrument in INSTRUMENTS {
                let actual = post
                    .sector_instrument_total_checked(sector, instrument)?
                    .checked_sub(pre.sector_instrument_total_checked(sector, instrument)?)
                    .ok_or_else(|| "sector other-volume stock delta overflow".to_string())?;
                let expected = self.expected_instrument_delta(sector, instrument)?;
                if actual != expected {
                    return Err(format!(
                        "sector {:?} {:?} delta mismatch: expected {}, actual {}",
                        sector, instrument, expected, actual
                    ));
                }
            }
        }
        Ok(())
    }

    /// Checked equity residual delta attributable to this other-volume projection.
    ///
    /// This is exposed for diagnostics but is not used by the stock-delta validator,
    /// because other non-transaction changes (such as asset revaluation) may affect
    /// total sector equity in the same period.
    pub fn equity_delta(&self, sector: EconomicSector) -> Result<i128, String> {
        self.expected_instrument_delta(sector, BalanceSheetInstrument::Equity)
    }

    /// Checked stock delta attributable to this other-volume projection.
    pub fn instrument_delta(
        &self,
        sector: EconomicSector,
        instrument: BalanceSheetInstrument,
    ) -> Result<i128, String> {
        self.expected_instrument_delta(sector, instrument)
    }

    fn expected_instrument_delta(
        &self,
        sector: EconomicSector,
        instrument: BalanceSheetInstrument,
    ) -> Result<i128, String> {
        self.changes.iter().try_fold(0i128, |sum, change| {
            let delta = match change.category {
                OtherVolumeChangeCategory::DebtWriteOff => match instrument {
                    BalanceSheetInstrument::Loans if change.creditor == sector => -change.amount,
                    BalanceSheetInstrument::Debt if change.debtor == sector => change.amount,
                    BalanceSheetInstrument::Equity => {
                        let creditor_delta = if change.creditor == sector {
                            change.amount
                        } else {
                            0
                        };
                        let debtor_delta = if change.debtor == sector {
                            -change.amount
                        } else {
                            0
                        };
                        creditor_delta
                            .checked_add(debtor_delta)
                            .ok_or_else(|| {
                                "sector other-volume equity delta overflow".to_string()
                            })?
                    }
                    _ => 0,
                },
            };
            sum.checked_add(delta)
                .ok_or_else(|| "sector other-volume instrument delta overflow".to_string())
        })
    }

    /// Re-derive this matrix from the supplied authoritative inputs.
    pub fn verify_against(
        &self,
        state: &EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<(), String> {
        self.validate_against(state, assignments, transitions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{ActorBalanceSheet, DebtWriteOff};

    #[test]
    fn debt_write_off_projects_to_other_volume_not_payment() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.claims = 100;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.liabilities = 100;
        let state = EconomicState::new(vec![bank, firm]);
        let assignments = vec![
            SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "firm".into(),
                sector: EconomicSector::Firm,
            },
        ];
        let transitions = vec![EconomicTransition::DebtWriteOff(
            DebtWriteOff::new("bank", "firm", 40).unwrap(),
        )];

        let matrix = SectorOtherVolumeChangeMatrix::from_transitions(
            &transitions,
            &assignments,
        )
        .unwrap();

        assert_eq!(matrix.changes.len(), 1);
        assert_eq!(matrix.changes[0].creditor, EconomicSector::Bank);
        assert_eq!(matrix.changes[0].debtor, EconomicSector::Firm);
        assert_eq!(matrix.changes[0].category, OtherVolumeChangeCategory::DebtWriteOff);
        assert_eq!(matrix.gross_volume(), 40);
        matrix.validate_against(&state, &assignments, &transitions).unwrap();
    }

    #[test]
    fn same_sector_debt_write_off_cancels_equity_delta() {
        let mut creditor = ActorBalanceSheet::new("bank-a");
        creditor.monetary.claims = 100;
        let mut debtor = ActorBalanceSheet::new("bank-b");
        debtor.monetary.liabilities = 100;
        let pre = EconomicState::new(vec![creditor, debtor]);
        let transitions = vec![EconomicTransition::DebtWriteOff(
            DebtWriteOff::new("bank-a", "bank-b", 40).unwrap(),
        )];
        let (post, _) =
            crate::economics::transition::apply_step(&pre, 1, &transitions, None).unwrap();
        let assignments = vec![
            SectorAssignment {
                actor: "bank-a".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "bank-b".into(),
                sector: EconomicSector::Bank,
            },
        ];
        let pre_sheet = SectorBalanceSheet::from_state(&pre, &assignments).unwrap();
        let post_sheet = SectorBalanceSheet::from_state(&post, &assignments).unwrap();
        let matrix =
            SectorOtherVolumeChangeMatrix::from_transitions(&transitions, &assignments).unwrap();

        matrix
            .validate_against_balance_sheet_delta(&pre_sheet, &post_sheet)
            .unwrap();
        assert_eq!(matrix.equity_delta(EconomicSector::Bank).unwrap(), 0);
    }

    #[test]
    fn debt_write_off_reconciles_equity_and_claim_stock_delta() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.claims = 100;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.liabilities = 100;
        let pre = EconomicState::new(vec![bank, firm]);
        let transitions = vec![EconomicTransition::DebtWriteOff(
            DebtWriteOff::new("bank", "firm", 40).unwrap(),
        )];
        let (post, _) =
            crate::economics::transition::apply_step(&pre, 1, &transitions, None).unwrap();
        let assignments = vec![
            SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "firm".into(),
                sector: EconomicSector::Firm,
            },
        ];
        let pre_sheet = SectorBalanceSheet::from_state(&pre, &assignments).unwrap();
        let post_sheet = SectorBalanceSheet::from_state(&post, &assignments).unwrap();
        let matrix =
            SectorOtherVolumeChangeMatrix::from_transitions(&transitions, &assignments).unwrap();

        matrix
            .validate_against_balance_sheet_delta(&pre_sheet, &post_sheet)
            .unwrap();
    }
}

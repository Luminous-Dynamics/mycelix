// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Sector-level financial-claim flow projection.
//!
//! This matrix is intentionally distinct from the cash/liquidity transaction
//! matrix. It records changes in contractual financial claims and obligations
//! so deferred trade settlement is visible without being mislabeled as cash.

use serde::{Deserialize, Serialize};

use super::sector_balance::{BalanceSheetInstrument, SectorAssignment, SectorBalanceSheet};
use super::sector_flow::EconomicSector;
use super::stock_flow::ActorId;
use super::transition::EconomicTransition;

/// Category of a sector-to-sector financial-claim flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FinancialFlowCategory {
    LoanCreation,
    DebtRepayment,
    DebtWriteOff,
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
        self.try_net_flow(sector)
            .expect("sector financial net-flow overflow")
    }

    /// Checked sector net claim-flow. Returns an error on arithmetic overflow.
    pub fn try_net_flow(&self, sector: EconomicSector) -> Result<i128, String> {
        self.flows.iter().try_fold(0i128, |sum, flow| {
            if flow.to == sector && flow.from != sector {
                sum.checked_add(flow.amount)
            } else if flow.from == sector && flow.to != sector {
                sum.checked_sub(flow.amount)
            } else {
                Some(sum)
            }
            .ok_or_else(|| "sector financial net-flow overflow".to_string())
        })
    }

    /// Every inter-sector financial-claim transition must clear. Overflow fails closed.
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

    pub fn try_gross_flow_volume(&self) -> Result<i128, String> {
        self.flows.iter().try_fold(0i128, |sum, flow| {
            sum.checked_add(flow.amount)
                .ok_or_else(|| "sector financial gross-flow overflow".to_string())
        })
    }

    pub fn gross_flow_volume(&self) -> i128 {
        self.try_gross_flow_volume()
            .expect("sector financial gross-flow overflow")
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
            transition.validate()?;
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
                EconomicTransition::DebtWriteOff(write_off) => (
                    sector_for(&write_off.lender)?,
                    sector_for(&write_off.borrower)?,
                    FinancialFlowCategory::DebtWriteOff,
                    write_off.amount,
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
                EconomicTransition::MonetaryTransfer(_)
                | EconomicTransition::IncomeTransfer(_)
                | EconomicTransition::CapitalInvestment(_)
                | EconomicTransition::Production(_)
                | EconomicTransition::InventoryTransfer(_)
                | EconomicTransition::InventoryConsumption(_)
                | EconomicTransition::GoodsSale(_)
                | EconomicTransition::InventoryCostAddition(_)
                | EconomicTransition::InventoryCostRelief(_)
                | EconomicTransition::Depreciation(_)
                | EconomicTransition::RealAssetRevaluation(_) => continue,
            };
            matrix.push(SectorFinancialFlow::new(from, to, category, amount)?);
        }
        Ok(matrix)
    }

    /// Non-financial transition variants are listed explicitly above so future enum additions force
    /// a compile-time review of whether they create contractual financial claims.
    /// Validate that financial-claim flows explain the relevant
    /// balance-sheet instrument deltas between two sector snapshots.
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
        const INSTRUMENTS: [BalanceSheetInstrument; 4] = [
            BalanceSheetInstrument::Loans,
            BalanceSheetInstrument::Debt,
            BalanceSheetInstrument::TradeReceivables,
            BalanceSheetInstrument::TradePayables,
        ];

        for sector in SECTORS {
            for instrument in INSTRUMENTS {
                let actual = post
                    .sector_instrument_total_checked(sector, instrument)?
                    .checked_sub(pre.sector_instrument_total_checked(sector, instrument)?)
                    .ok_or_else(|| "sector financial stock delta overflow".to_string())?;
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

    fn expected_instrument_delta(
        &self,
        sector: EconomicSector,
        instrument: BalanceSheetInstrument,
    ) -> Result<i128, String> {
        self.flows.iter().try_fold(0i128, |sum, flow| {
            let delta = match (flow.category, flow.from == sector, flow.to == sector) {
                (FinancialFlowCategory::LoanCreation, true, _)
                    if instrument == BalanceSheetInstrument::Loans =>
                    flow.amount,
                (FinancialFlowCategory::LoanCreation, _, true)
                    if instrument == BalanceSheetInstrument::Debt =>
                    -flow.amount,
                (FinancialFlowCategory::DebtRepayment, true, _)
                    if instrument == BalanceSheetInstrument::Debt =>
                    flow.amount,
                (FinancialFlowCategory::DebtRepayment, _, true)
                    if instrument == BalanceSheetInstrument::Loans =>
                    -flow.amount,
                (FinancialFlowCategory::DebtWriteOff, true, _)
                    if instrument == BalanceSheetInstrument::Loans =>
                    -flow.amount,
                (FinancialFlowCategory::DebtWriteOff, _, true)
                    if instrument == BalanceSheetInstrument::Debt =>
                    flow.amount,
                (FinancialFlowCategory::TradeCreditExtension, true, _)
                    if instrument == BalanceSheetInstrument::TradeReceivables =>
                    flow.amount,
                (FinancialFlowCategory::TradeCreditExtension, _, true)
                    if instrument == BalanceSheetInstrument::TradePayables =>
                    -flow.amount,
                (FinancialFlowCategory::TradeCreditSettlement, true, _)
                    if instrument == BalanceSheetInstrument::TradeReceivables =>
                    -flow.amount,
                (FinancialFlowCategory::TradeCreditSettlement, _, true)
                    if instrument == BalanceSheetInstrument::TradePayables =>
                    flow.amount,
                (FinancialFlowCategory::LoanCreation
                    | FinancialFlowCategory::DebtRepayment
                    | FinancialFlowCategory::DebtWriteOff
                    | FinancialFlowCategory::TradeCreditExtension
                    | FinancialFlowCategory::TradeCreditSettlement)
                    => 0,
            };
            sum.checked_add(delta)
                .ok_or_else(|| "sector financial claim delta overflow".to_string())
        })
    }

    /// Re-derive this matrix from the supplied transition sequence and require exact equality.
    ///
    /// This is stronger than checking only that the matrix clears: every serialized financial
    /// claim-flow entry must be reproducible from the authoritative transitions.
    pub fn verify_against(
        &self,
        state: &super::stock_flow::EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<(), String> {
        self.validate_against(state, assignments, transitions)
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
        TradeCreditSettlement, DebtRepayment, DebtWriteOff,
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
    fn financial_clearing_fails_closed_on_overflow() {
        let mut m = SectorFinancialFlowMatrix::default();
        m.push(SectorFinancialFlow {
            from: EconomicSector::Firm,
            to: EconomicSector::Household,
            category: FinancialFlowCategory::TradeCreditExtension,
            amount: i128::MAX,
        });
        m.push(SectorFinancialFlow {
            from: EconomicSector::Firm,
            to: EconomicSector::Public,
            category: FinancialFlowCategory::TradeCreditExtension,
            amount: 1,
        });
        assert!(!m.clears());
        assert!(m.try_gross_flow_volume().is_err());
    }

    #[test]
    fn financial_matrix_reconciles_balance_sheet_claim_deltas() {
        let bank = ActorBalanceSheet::new("bank");
        let mut firm = ActorBalanceSheet::new("firm");
        firm.real.inventories = 2;
        let mut household = ActorBalanceSheet::new("household");
        household.monetary.deposits = 20;
        let pre = EconomicState::new(vec![bank, firm, household]);
        let transitions = vec![
            EconomicTransition::TradeCreditSale(
                TradeCreditSale::new("firm", "household", 2, 40).unwrap(),
            ),
            EconomicTransition::TradeCreditSettlement(
                TradeCreditSettlement::new("firm", "household", 15).unwrap(),
            ),
        ];
        let (post, _) =
            crate::economics::transition::apply_step(&pre, 1, &transitions, None).unwrap();
        let assignments = assignments();
        let pre_sheet = SectorBalanceSheet::from_state(&pre, &assignments).unwrap();
        let post_sheet = SectorBalanceSheet::from_state(&post, &assignments).unwrap();
        let matrix = SectorFinancialFlowMatrix::from_transitions(&transitions, &assignments).unwrap();

        matrix
            .validate_against_balance_sheet_delta(&pre_sheet, &post_sheet)
            .unwrap();
        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Firm,
                BalanceSheetInstrument::TradeReceivables
            ),
            25
        );
        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Household,
                BalanceSheetInstrument::TradePayables
            ),
            -25
        );

        let mut tampered = post_sheet.clone();
        tampered.entries.iter_mut().find(|entry| {
            entry.sector == EconomicSector::Firm
                && entry.instrument == BalanceSheetInstrument::TradeReceivables
        }).unwrap().amount += 1;
        assert!(matrix
            .validate_against_balance_sheet_delta(&pre_sheet, &tampered)
            .is_err());
    }

    #[test]
    #[test]
    fn financial_matrix_reconciles_debt_write_off_claim_and_liability_deltas() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.claims = 100;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.liabilities = 100;
        let pre = EconomicState::new(vec![bank, firm]);

        let transitions = vec![
            EconomicTransition::DebtWriteOff(
                DebtWriteOff::new("bank", "firm", 40).unwrap(),
            ),
        ];
        let (post, _) =
            crate::economics::transition::apply_step(&pre, 1, &transitions, None).unwrap();
        let assignments = vec![
            SectorAssignment { actor: "bank".into(), sector: EconomicSector::Bank },
            SectorAssignment { actor: "firm".into(), sector: EconomicSector::Firm },
        ];
        let pre_sheet = SectorBalanceSheet::from_state(&pre, &assignments).unwrap();
        let post_sheet = SectorBalanceSheet::from_state(&post, &assignments).unwrap();
        let matrix =
            SectorFinancialFlowMatrix::from_transitions(&transitions, &assignments).unwrap();

        assert_eq!(matrix.flows.len(), 1);
        assert_eq!(matrix.flows[0].category, FinancialFlowCategory::DebtWriteOff);
        matrix
            .validate_against_balance_sheet_delta(&pre_sheet, &post_sheet)
            .unwrap();
        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Bank,
                BalanceSheetInstrument::Loans
            ),
            60
        );
        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Firm,
                BalanceSheetInstrument::Debt
            ),
            -60
        );
    }

    fn financial_matrix_reconciles_loan_claim_and_debt_liability_signs() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.deposits = 100;
        let pre = EconomicState::new(vec![bank, firm]);

        let transitions = vec![
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "firm", 60).unwrap(),
            ),
            EconomicTransition::DebtRepayment(
                DebtRepayment::new("bank", "firm", 20).unwrap(),
            ),
        ];

        let (post, _) =
            crate::economics::transition::apply_step(&pre, 1, &transitions, None).unwrap();
        let assignments = assignments();
        let pre_sheet = SectorBalanceSheet::from_state(&pre, &assignments).unwrap();
        let post_sheet = SectorBalanceSheet::from_state(&post, &assignments).unwrap();
        let matrix =
            SectorFinancialFlowMatrix::from_transitions(&transitions, &assignments).unwrap();

        matrix
            .validate_against_balance_sheet_delta(&pre_sheet, &post_sheet)
            .unwrap();

        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Bank,
                BalanceSheetInstrument::Loans
            ),
            40
        );
        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Firm,
                BalanceSheetInstrument::Debt
            ),
            -40
        );
    }

    #[test]
    fn verify_against_rejects_rehashed_but_wrong_projection() {
        let pre = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("firm"),
        ]);
        let assignments = vec![
            SectorAssignment { actor: "bank".into(), sector: EconomicSector::Bank },
            SectorAssignment { actor: "firm".into(), sector: EconomicSector::Firm },
        ];
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "firm", 50).unwrap(),
        )];
        let mut matrix = SectorFinancialFlowMatrix::from_transitions(&transitions, &assignments).unwrap();
        matrix.flows[0].amount = 51;
        assert!(matrix.verify_against(&pre, &assignments, &transitions).is_err());
    }

    #[test]
    fn financial_matrix_handles_intra_sector_claim_changes() {
        let pre = EconomicState::new(vec![
            ActorBalanceSheet::new("firm-a"),
            ActorBalanceSheet::new("firm-b"),
        ]);
        let transitions = vec![EconomicTransition::TradeCreditSale(
            TradeCreditSale::new("firm-a", "firm-b", 1, 25).unwrap(),
        )];
        let (post, _) =
            crate::economics::transition::apply_step(&pre, 1, &transitions, None).unwrap();

        let same_sector = vec![
            SectorAssignment {
                actor: "firm-a".into(),
                sector: EconomicSector::Firm,
            },
            SectorAssignment {
                actor: "firm-b".into(),
                sector: EconomicSector::Firm,
            },
        ];
        let pre_sheet = SectorBalanceSheet::from_state(&pre, &same_sector).unwrap();
        let post_sheet = SectorBalanceSheet::from_state(&post, &same_sector).unwrap();
        let matrix =
            SectorFinancialFlowMatrix::from_transitions(&transitions, &same_sector).unwrap();

        assert_eq!(matrix.net_flow(EconomicSector::Firm), 0);
        matrix
            .validate_against_balance_sheet_delta(&pre_sheet, &post_sheet)
            .unwrap();
        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Firm,
                BalanceSheetInstrument::TradeReceivables
            ),
            25
        );
        assert_eq!(
            post_sheet.sector_instrument_total(
                EconomicSector::Firm,
                BalanceSheetInstrument::TradePayables
            ),
            -25
        );
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

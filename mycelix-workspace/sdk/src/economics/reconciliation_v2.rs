// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic stock-flow reconciliation.
//!
//! Binds the sector balance-sheet layer to the ordered economic transition log.
//! For modeled financial transitions:
//!
//!     balance_sheet(t+1) - balance_sheet(t) == transition postings
//!
//! Behavioral flows remain intentionally outside this layer until they have
//! explicit double-entry posting rules.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use super::sector_balance::{BalanceSheetInstrument, SectorAssignment, SectorBalanceSheet};
use super::sector_flow::EconomicSector;
use super::stock_flow::{EconomicState, MonetaryInstrument};
use super::transition::EconomicTransition;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct StockPosting {
    pub sector: EconomicSector,
    pub instrument: BalanceSheetInstrument,
    pub delta: i128,
}

impl StockPosting {
    pub const fn new(sector: EconomicSector, instrument: BalanceSheetInstrument, delta: i128) -> Self {
        Self { sector, instrument, delta }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StockFlowReconciliation {
    pub pre_state_hash: String,
    pub post_state_hash: String,
    pub transition_hash: String,
    pub posting_hash: String,
    pub posting_count: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StockFlowMismatch {
    pub sector: EconomicSector,
    pub instrument: BalanceSheetInstrument,
    pub expected: i128,
    pub actual: i128,
}

pub fn reconcile_step(
    pre_state: &EconomicState,
    post_state: &EconomicState,
    assignments: &[SectorAssignment],
    transitions: &[EconomicTransition],
) -> Result<StockFlowReconciliation, String> {
    let pre = SectorBalanceSheet::from_state(pre_state, assignments)?;
    let post = SectorBalanceSheet::from_state(post_state, assignments)?;
    let expected = aggregate_postings(postings_for_step(pre_state, assignments, transitions)?);
    let actual = balance_sheet_delta(&pre, &post);

    let mut mismatches = Vec::new();
    for key in union_keys(&actual, &expected) {
        let actual_delta = actual.get(&key).copied().unwrap_or(0);
        let expected_delta = expected.get(&key).copied().unwrap_or(0);
        if actual_delta != expected_delta {
            mismatches.push(StockFlowMismatch {
                sector: key.0,
                instrument: key.1,
                expected: expected_delta,
                actual: actual_delta,
            });
        }
    }

    if !mismatches.is_empty() {
        return Err(serde_json::to_string(&mismatches)
            .map_err(|e| format!("failed to serialize reconciliation mismatches: {e}"))?);
    }

    Ok(StockFlowReconciliation {
        pre_state_hash: super::transition::state_hash(pre_state).map_err(|e| e.to_string())?,
        post_state_hash: super::transition::state_hash(post_state).map_err(|e| e.to_string())?,
        transition_hash: super::transition::transition_hash(transitions).map_err(|e| e.to_string())?,
        posting_hash: hash_postings(&expected)?,
        posting_count: expected.len() as u64,
    })
}

pub fn postings_for_step(
    pre_state: &EconomicState,
    assignments: &[SectorAssignment],
    transitions: &[EconomicTransition],
) -> Result<Vec<StockPosting>, String> {
    let mut working = pre_state.clone();
    let mut postings = Vec::new();

    for transition in transitions {
        let transition_postings = match transition {
            EconomicTransition::MonetaryTransfer(flow) => {
                let from = sector_for(assignments, &flow.from)?;
                let to = sector_for(assignments, &flow.to)?;
                let instrument = match flow.instrument {
                    MonetaryInstrument::Cash => BalanceSheetInstrument::Cash,
                    MonetaryInstrument::Deposit => BalanceSheetInstrument::Deposits,
                };
                vec![
                    StockPosting::new(from, instrument, -flow.amount),
                    StockPosting::new(to, instrument, flow.amount),
                ]
            }
            EconomicTransition::IncomeTransfer(transfer) => {
                let payer = sector_for(assignments, &transfer.payer)?;
                let recipient = sector_for(assignments, &transfer.recipient)?;
                vec![
                    StockPosting::new(payer, BalanceSheetInstrument::Deposits, -transfer.amount),
                    StockPosting::new(payer, BalanceSheetInstrument::Equity, transfer.amount),
                    StockPosting::new(recipient, BalanceSheetInstrument::Deposits, transfer.amount),
                    StockPosting::new(recipient, BalanceSheetInstrument::Equity, -transfer.amount),
                ]
            }
            EconomicTransition::CapitalInvestment(investment) => {
                let buyer = sector_for(assignments, &investment.buyer)?;
                let producer = sector_for(assignments, &investment.producer)?;
                vec![
                    StockPosting::new(
                        buyer,
                        BalanceSheetInstrument::Deposits,
                        -investment.amount,
                    ),
                    StockPosting::new(
                        buyer,
                        BalanceSheetInstrument::ProductiveCapital,
                        investment.amount,
                    ),
                    StockPosting::new(
                        producer,
                        BalanceSheetInstrument::Deposits,
                        investment.amount,
                    ),
                    StockPosting::new(
                        producer,
                        BalanceSheetInstrument::Equity,
                        -investment.amount,
                    ),
                ]
            }
            EconomicTransition::CreditCreation(credit) => {
                let lender = sector_for(assignments, &credit.lender)?;
                let borrower = sector_for(assignments, &credit.borrower)?;
                vec![
                    StockPosting::new(lender, BalanceSheetInstrument::Loans, credit.amount),
                    StockPosting::new(lender, BalanceSheetInstrument::DepositLiabilities, -credit.amount),
                    StockPosting::new(borrower, BalanceSheetInstrument::Deposits, credit.amount),
                    StockPosting::new(borrower, BalanceSheetInstrument::Debt, -credit.amount),
                ]
            }
            EconomicTransition::DebtRepayment(repayment) => {
                let borrower = working.actors.iter().find(|a| a.actor == repayment.borrower)
                    .ok_or_else(|| format!("unknown actor: {}", repayment.borrower))?;
                let lender = working.actors.iter().find(|a| a.actor == repayment.lender)
                    .ok_or_else(|| format!("unknown actor: {}", repayment.lender))?;
                let lender_sector = sector_for(assignments, &repayment.lender)?;
                let borrower_sector = sector_for(assignments, &repayment.borrower)?;

                if borrower.monetary.deposits >= repayment.amount {
                    if lender.monetary.deposit_liabilities < repayment.amount {
                        return Err("deposit repayment requires a matching lender deposit liability".into());
                    }
                    vec![
                        StockPosting::new(borrower_sector, BalanceSheetInstrument::Deposits, -repayment.amount),
                        StockPosting::new(lender_sector, BalanceSheetInstrument::DepositLiabilities, repayment.amount),
                        StockPosting::new(borrower_sector, BalanceSheetInstrument::Debt, repayment.amount),
                        StockPosting::new(lender_sector, BalanceSheetInstrument::Loans, -repayment.amount),
                    ]
                } else if borrower.monetary.cash >= repayment.amount {
                    vec![
                        StockPosting::new(borrower_sector, BalanceSheetInstrument::Cash, -repayment.amount),
                        StockPosting::new(lender_sector, BalanceSheetInstrument::Cash, repayment.amount),
                        StockPosting::new(borrower_sector, BalanceSheetInstrument::Debt, repayment.amount),
                        StockPosting::new(lender_sector, BalanceSheetInstrument::Loans, -repayment.amount),
                    ]
                } else {
                    return Err(format!("borrower {} cannot settle repayment", repayment.borrower));
                }
            }
        };

        apply_transition(&mut working, transition)?;
        postings.extend(transition_postings);
    }

    Ok(postings)
}

fn apply_transition(state: &mut EconomicState, transition: &EconomicTransition) -> Result<(), String> {
    match transition {
        EconomicTransition::MonetaryTransfer(flow) => state.apply_flow(flow),
        EconomicTransition::IncomeTransfer(transfer) => state.apply_income_transfer(transfer),
        EconomicTransition::CapitalInvestment(investment) => state.apply_capital_investment(investment),
        EconomicTransition::CreditCreation(credit) => state.create_credit(credit),
        EconomicTransition::DebtRepayment(repayment) => state.repay_debt(repayment),
    }
}

fn sector_for(assignments: &[SectorAssignment], actor: &str) -> Result<EconomicSector, String> {
    assignments.iter().find(|a| a.actor == actor).map(|a| a.sector)
        .ok_or_else(|| format!("missing sector assignment for {actor}"))
}

type StockKey = (EconomicSector, BalanceSheetInstrument);

fn balance_sheet_delta(pre: &SectorBalanceSheet, post: &SectorBalanceSheet) -> HashMap<StockKey, i128> {
    let mut result = HashMap::new();
    for entry in pre.entries.iter().chain(post.entries.iter()) {
        result.entry((entry.sector, entry.instrument)).or_insert(0);
    }
    for key in result.keys().copied().collect::<Vec<_>>() {
        let before = pre.entries.iter().filter(|e| (e.sector, e.instrument) == key).map(|e| e.amount).sum::<i128>();
        let after = post.entries.iter().filter(|e| (e.sector, e.instrument) == key).map(|e| e.amount).sum::<i128>();
        result.insert(key, after - before);
    }
    result
}

fn aggregate_postings(postings: Vec<StockPosting>) -> HashMap<StockKey, i128> {
    let mut result = HashMap::new();
    for posting in postings {
        *result.entry((posting.sector, posting.instrument)).or_insert(0) += posting.delta;
    }
    result
}

fn union_keys(left: &HashMap<StockKey, i128>, right: &HashMap<StockKey, i128>) -> Vec<StockKey> {
    let mut keys = left.keys().chain(right.keys()).copied().collect::<Vec<_>>();
    keys.sort_by_key(|(sector, instrument)| (*sector as u8, *instrument as u8));
    keys.dedup();
    keys
}

fn hash_postings(postings: &HashMap<StockKey, i128>) -> Result<String, String> {
    let mut ordered = postings.iter().map(|(key, value)| (key.0 as u8, key.1 as u8, *value)).collect::<Vec<_>>();
    ordered.sort_unstable();
    let bytes = serde_json::to_vec(&ordered).map_err(|e| format!("failed to serialize postings: {e}"))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{ActorBalanceSheet, CapitalInvestment, CreditCreation, DebtRepayment, IncomeTransfer, MonetaryFlow};
    use crate::economics::transition::apply_step;

    fn setup() -> (EconomicState, Vec<SectorAssignment>) {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        (
            EconomicState::new(vec![bank, ActorBalanceSheet::new("household"), ActorBalanceSheet::new("firm")]),
            vec![
                SectorAssignment { actor: "bank".into(), sector: EconomicSector::Bank },
                SectorAssignment { actor: "household".into(), sector: EconomicSector::Household },
                SectorAssignment { actor: "firm".into(), sector: EconomicSector::NonFinancialCorporation },
            ],
        )
    }

    #[test]
    fn credit_creation_reconciles_exact_stock_delta() {
        let (pre, assignments) = setup();
        let transitions = vec![EconomicTransition::CreditCreation(CreditCreation::new("bank", "household", 500).unwrap())];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        let receipt = reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
        assert_eq!(receipt.posting_count, 4);
    }

    #[test]
    fn deposit_transfer_reconciles_exact_stock_delta() {
        let (mut pre, assignments) = setup();
        pre.create_credit(&CreditCreation::new("bank", "household", 500).unwrap()).unwrap();
        let transitions = vec![EconomicTransition::MonetaryTransfer(MonetaryFlow::deposit_transfer("household", "bank", 100).unwrap())];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn income_transfer_reconciles_equity_and_deposits() {
        let (mut pre, assignments) = setup();
        pre.create_credit(&CreditCreation::new("bank", "household", 500).unwrap()).unwrap();
        let transitions = vec![EconomicTransition::IncomeTransfer(
            IncomeTransfer::new("household", "bank", 100).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn capital_investment_reconciles_real_and_financial_stocks() {
        let (mut pre, assignments) = setup();
        pre.actors.iter_mut()
            .find(|a| a.actor == "firm")
            .unwrap()
            .monetary.deposits = 500;
        let transitions = vec![EconomicTransition::CapitalInvestment(
            CapitalInvestment::new("firm", "household", 200).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn debt_repayment_reconciles_deposit_settlement() {
        let (mut pre, assignments) = setup();
        pre.create_credit(&CreditCreation::new("bank", "household", 500).unwrap()).unwrap();
        let transitions = vec![EconomicTransition::DebtRepayment(DebtRepayment::new("bank", "household", 200).unwrap())];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn tampered_post_state_is_rejected() {
        let (pre, assignments) = setup();
        let transitions = vec![EconomicTransition::CreditCreation(CreditCreation::new("bank", "household", 500).unwrap())];
        let (mut post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        post.actors[1].monetary.deposits += 1;
        let error = reconcile_step(&pre, &post, &assignments, &transitions).unwrap_err();
        assert!(error.contains("expected"));
    }
}

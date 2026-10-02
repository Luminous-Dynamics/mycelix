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

use super::sector_balance::{
    BalanceSheetEntry, BalanceSheetInstrument, PhysicalStockInstrument, SectorAssignment,
    SectorBalanceSheet, SectorPhysicalStock,
};
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PhysicalStockPosting {
    pub sector: EconomicSector,
    pub instrument: PhysicalStockInstrument,
    pub delta: i128,
}

impl PhysicalStockPosting {
    pub const fn new(
        sector: EconomicSector,
        instrument: PhysicalStockInstrument,
        delta: i128,
    ) -> Self {
        Self { sector, instrument, delta }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StockFlowReconciliation {
    pub pre_state_hash: String,
    pub post_state_hash: String,
    pub transition_hash: String,
    /// Hash of monetary balance-sheet postings.
    pub posting_hash: String,
    pub posting_count: u64,
    /// Hash of physical quantity postings.
    pub physical_posting_hash: String,
    pub physical_posting_count: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StockFlowMismatch {
    pub sector: EconomicSector,
    pub instrument: BalanceSheetInstrument,
    pub expected: i128,
    pub actual: i128,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PhysicalStockMismatch {
    pub sector: EconomicSector,
    pub instrument: PhysicalStockInstrument,
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
    let expected = aggregate_postings(postings_for_step(pre_state, assignments, transitions)?)?;
    let actual = balance_sheet_delta(&pre, &post)?;

    let mut financial_mismatches = Vec::new();
    for key in union_keys(&actual, &expected) {
        let actual_delta = actual.get(&key).copied().unwrap_or(0);
        let expected_delta = expected.get(&key).copied().unwrap_or(0);
        if actual_delta != expected_delta {
            financial_mismatches.push(StockFlowMismatch {
                sector: key.0,
                instrument: key.1,
                expected: expected_delta,
                actual: actual_delta,
            });
        }
    }
    if !financial_mismatches.is_empty() {
        return Err(serde_json::to_string(&financial_mismatches)
            .map_err(|e| format!("failed to serialize financial reconciliation mismatches: {e}"))?);
    }

    let pre_physical = SectorPhysicalStock::from_state(pre_state, assignments)?;
    let post_physical = SectorPhysicalStock::from_state(post_state, assignments)?;
    let expected_physical =
        aggregate_physical_postings(physical_postings_for_step(pre_state, assignments, transitions)?)?;
    let actual_physical = physical_stock_delta(&pre_physical, &post_physical)?;

    let mut physical_mismatches = Vec::new();
    for key in union_physical_keys(&actual_physical, &expected_physical) {
        let actual_delta = actual_physical.get(&key).copied().unwrap_or(0);
        let expected_delta = expected_physical.get(&key).copied().unwrap_or(0);
        if actual_delta != expected_delta {
            physical_mismatches.push(PhysicalStockMismatch {
                sector: key.0,
                instrument: key.1,
                expected: expected_delta,
                actual: actual_delta,
            });
        }
    }
    if !physical_mismatches.is_empty() {
        return Err(serde_json::to_string(&physical_mismatches)
            .map_err(|e| format!("failed to serialize physical reconciliation mismatches: {e}"))?);
    }

    Ok(StockFlowReconciliation {
        pre_state_hash: super::transition::state_hash(pre_state).map_err(|e| e.to_string())?,
        post_state_hash: super::transition::state_hash(post_state).map_err(|e| e.to_string())?,
        transition_hash: super::transition::transition_hash(transitions).map_err(|e| e.to_string())?,
        posting_hash: hash_postings(&expected)?,
        posting_count: expected.len() as u64,
        physical_posting_hash: hash_physical_postings(&expected_physical)?,
        physical_posting_count: expected_physical.len() as u64,
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
            EconomicTransition::Production(_)
            | EconomicTransition::InventoryTransfer(_)
            | EconomicTransition::InventoryConsumption(_) => Vec::new(),
            EconomicTransition::GoodsSale(sale) => {
                let seller = sector_for(assignments, &sale.seller)?;
                let buyer = sector_for(assignments, &sale.buyer)?;
                vec![
                    StockPosting::new(seller, BalanceSheetInstrument::Deposits, sale.consideration),
                    StockPosting::new(seller, BalanceSheetInstrument::Equity, -sale.consideration),
                    StockPosting::new(buyer, BalanceSheetInstrument::Deposits, -sale.consideration),
                    StockPosting::new(buyer, BalanceSheetInstrument::Equity, sale.consideration),
                ]
            }
            EconomicTransition::TradeCreditSale(sale) => {
                let seller = sector_for(assignments, &sale.seller)?;
                let buyer = sector_for(assignments, &sale.buyer)?;
                vec![
                    StockPosting::new(
                        seller,
                        BalanceSheetInstrument::TradeReceivables,
                        sale.consideration,
                    ),
                    StockPosting::new(
                        seller,
                        BalanceSheetInstrument::Equity,
                        -sale.consideration,
                    ),
                    StockPosting::new(
                        buyer,
                        BalanceSheetInstrument::TradePayables,
                        -sale.consideration,
                    ),
                    StockPosting::new(
                        buyer,
                        BalanceSheetInstrument::Equity,
                        sale.consideration,
                    ),
                ]
            }
            EconomicTransition::TradeCreditSettlement(settlement) => {
                let seller = sector_for(assignments, &settlement.seller)?;
                let buyer = sector_for(assignments, &settlement.buyer)?;
                vec![
                    StockPosting::new(seller, BalanceSheetInstrument::TradeReceivables, -settlement.amount),
                    StockPosting::new(buyer, BalanceSheetInstrument::TradePayables, settlement.amount),
                    StockPosting::new(seller, BalanceSheetInstrument::Deposits, settlement.amount),
                    StockPosting::new(buyer, BalanceSheetInstrument::Deposits, -settlement.amount),
                ]
            }
            EconomicTransition::InventoryCostAddition(addition) => {
                let actor = sector_for(assignments, &addition.actor)?;
                vec![
                    StockPosting::new(
                        actor,
                        BalanceSheetInstrument::InventoryCarryingValue,
                        addition.carrying_value,
                    ),
                    StockPosting::new(
                        actor,
                        BalanceSheetInstrument::Equity,
                        -addition.carrying_value,
                    ),
                ]
            }
            EconomicTransition::InventoryCostRelief(relief) => {
                let actor = sector_for(assignments, &relief.actor)?;
                vec![
                    StockPosting::new(
                        actor,
                        BalanceSheetInstrument::InventoryCarryingValue,
                        -relief.carrying_value,
                    ),
                    StockPosting::new(
                        actor,
                        BalanceSheetInstrument::Equity,
                        relief.carrying_value,
                    ),
                ]
            }
            EconomicTransition::Depreciation(depreciation) => {
                let actor = sector_for(assignments, &depreciation.actor)?;
                vec![
                    StockPosting::new(
                        actor,
                        BalanceSheetInstrument::ProductiveCapital,
                        -depreciation.amount,
                    ),
                    StockPosting::new(
                        actor,
                        BalanceSheetInstrument::Equity,
                        depreciation.amount,
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

/// Derive physical quantity postings from the authoritative transition log.
pub fn physical_postings_for_step(
    pre_state: &EconomicState,
    assignments: &[SectorAssignment],
    transitions: &[EconomicTransition],
) -> Result<Vec<PhysicalStockPosting>, String> {
    let mut working = pre_state.clone();
    let mut postings = Vec::new();

    for transition in transitions {
        match transition {
            EconomicTransition::Production(production) => {
                let sector = sector_for(assignments, &production.producer)?;
                postings.push(PhysicalStockPosting::new(
                    sector,
                    PhysicalStockInstrument::Resources,
                    -production.resource_input,
                ));
                postings.push(PhysicalStockPosting::new(
                    sector,
                    PhysicalStockInstrument::Inventories,
                    production.output,
                ));
            }
            EconomicTransition::InventoryTransfer(transfer) => {
                let from = sector_for(assignments, &transfer.from)?;
                let to = sector_for(assignments, &transfer.to)?;
                postings.push(PhysicalStockPosting::new(
                    from,
                    PhysicalStockInstrument::Inventories,
                    -transfer.quantity,
                ));
                postings.push(PhysicalStockPosting::new(
                    to,
                    PhysicalStockInstrument::Inventories,
                    transfer.quantity,
                ));
            }
            EconomicTransition::InventoryConsumption(consumption) => {
                let sector = sector_for(assignments, &consumption.consumer)?;
                postings.push(PhysicalStockPosting::new(
                    sector,
                    PhysicalStockInstrument::Inventories,
                    -consumption.quantity,
                ));
            }
            EconomicTransition::GoodsSale(sale) => {
                let seller = sector_for(assignments, &sale.seller)?;
                let buyer = sector_for(assignments, &sale.buyer)?;
                postings.push(PhysicalStockPosting::new(
                    seller,
                    PhysicalStockInstrument::Inventories,
                    -sale.quantity,
                ));
                postings.push(PhysicalStockPosting::new(
                    buyer,
                    PhysicalStockInstrument::Inventories,
                    sale.quantity,
                ));
            }
            EconomicTransition::TradeCreditSale(sale) => {
                let seller = sector_for(assignments, &sale.seller)?;
                let buyer = sector_for(assignments, &sale.buyer)?;
                postings.push(PhysicalStockPosting::new(
                    seller,
                    PhysicalStockInstrument::Inventories,
                    -sale.quantity,
                ));
                postings.push(PhysicalStockPosting::new(
                    buyer,
                    PhysicalStockInstrument::Inventories,
                    sale.quantity,
                ));
            }
            EconomicTransition::TradeCreditSettlement(_) => {}
            _ => {}
        }

        apply_transition(&mut working, transition)?;
    }

    Ok(postings)
}

fn apply_transition(state: &mut EconomicState, transition: &EconomicTransition) -> Result<(), String> {
    match transition {
        EconomicTransition::MonetaryTransfer(flow) => state.apply_flow(flow),
        EconomicTransition::IncomeTransfer(transfer) => state.apply_income_transfer(transfer),
        EconomicTransition::CapitalInvestment(investment) => state.apply_capital_investment(investment),
        EconomicTransition::Production(production) => state.apply_production(production),
        EconomicTransition::InventoryTransfer(transfer) => state.apply_inventory_transfer(transfer),
        EconomicTransition::InventoryConsumption(consumption) => state.apply_inventory_consumption(consumption),
        EconomicTransition::GoodsSale(sale) => state.apply_goods_sale(sale),
        EconomicTransition::TradeCreditSale(sale) => state.apply_trade_credit_sale(sale),
        EconomicTransition::TradeCreditSettlement(settlement) => {
            state.apply_trade_credit_settlement(settlement)
        }
        EconomicTransition::InventoryCostAddition(addition) => state.apply_inventory_cost_addition(addition),
        EconomicTransition::InventoryCostRelief(relief) => state.apply_inventory_cost_relief(relief),
        EconomicTransition::CreditCreation(credit) => state.create_credit(credit),
        EconomicTransition::DebtRepayment(repayment) => state.repay_debt(repayment),
    }
}

fn sector_for(assignments: &[SectorAssignment], actor: &str) -> Result<EconomicSector, String> {
    assignments.iter().find(|a| a.actor == actor).map(|a| a.sector)
        .ok_or_else(|| format!("missing sector assignment for {actor}"))
}

type StockKey = (EconomicSector, BalanceSheetInstrument);

fn balance_sheet_delta(
    pre: &SectorBalanceSheet,
    post: &SectorBalanceSheet,
) -> Result<HashMap<StockKey, i128>, String> {
    let mut keys = HashMap::new();
    for entry in pre.entries.iter().chain(post.entries.iter()) {
        keys.entry((entry.sector, entry.instrument)).or_insert(());
    }

    let mut result = HashMap::new();
    for key in keys.keys().copied() {
        let before = pre
            .entries
            .iter()
            .filter(|e| (e.sector, e.instrument) == key)
            .try_fold(0i128, |sum, entry| {
                sum.checked_add(entry.amount)
                    .ok_or_else(|| format!("pre-state balance-sheet delta overflow for {:?}", key))
            })?;
        let after = post
            .entries
            .iter()
            .filter(|e| (e.sector, e.instrument) == key)
            .try_fold(0i128, |sum, entry| {
                sum.checked_add(entry.amount)
                    .ok_or_else(|| format!("post-state balance-sheet delta overflow for {:?}", key))
            })?;
        let delta = after
            .checked_sub(before)
            .ok_or_else(|| format!("balance-sheet stock delta overflow for {:?}", key))?;
        result.insert(key, delta);
    }
    Ok(result)
}

fn aggregate_postings(postings: Vec<StockPosting>) -> Result<HashMap<StockKey, i128>, String> {
    let mut result = HashMap::new();
    for posting in postings {
        let slot = result
            .entry((posting.sector, posting.instrument))
            .or_insert(0);
        *slot = slot
            .checked_add(posting.delta)
            .ok_or_else(|| {
                format!(
                    "stock-posting aggregation overflow for {:?}",
                    (posting.sector, posting.instrument)
                )
            })?;
    }
    Ok(result)
}

fn physical_stock_delta(
    pre: &SectorPhysicalStock,
    post: &SectorPhysicalStock,
) -> Result<HashMap<PhysicalKey, i128>, String> {
    let mut keys = HashMap::new();
    for entry in pre.entries.iter().chain(post.entries.iter()) {
        keys.entry((entry.sector, entry.instrument)).or_insert(());
    }

    let mut result = HashMap::new();
    for key in keys.keys().copied() {
        let before = pre
            .entries
            .iter()
            .filter(|e| (e.sector, e.instrument) == key)
            .try_fold(0i128, |sum, entry| {
                sum.checked_add(entry.amount)
                    .ok_or_else(|| format!("pre-state physical-stock delta overflow for {:?}", key))
            })?;
        let after = post
            .entries
            .iter()
            .filter(|e| (e.sector, e.instrument) == key)
            .try_fold(0i128, |sum, entry| {
                sum.checked_add(entry.amount)
                    .ok_or_else(|| format!("post-state physical-stock delta overflow for {:?}", key))
            })?;
        let delta = after
            .checked_sub(before)
            .ok_or_else(|| format!("physical-stock delta overflow for {:?}", key))?;
        result.insert(key, delta);
    }
    Ok(result)
}

type PhysicalKey = (EconomicSector, PhysicalStockInstrument);

fn aggregate_physical_postings(
    postings: Vec<PhysicalStockPosting>,
) -> Result<HashMap<PhysicalKey, i128>, String> {
    let mut result = HashMap::new();
    for posting in postings {
        let slot = result
            .entry((posting.sector, posting.instrument))
            .or_insert(0);
        *slot = slot
            .checked_add(posting.delta)
            .ok_or_else(|| {
                format!(
                    "physical-posting aggregation overflow for {:?}",
                    (posting.sector, posting.instrument)
                )
            })?;
    }
    Ok(result)
}

fn union_physical_keys(
    left: &HashMap<PhysicalKey, i128>,
    right: &HashMap<PhysicalKey, i128>,
) -> Vec<PhysicalKey> {
    let mut keys = left.keys().chain(right.keys()).copied().collect::<Vec<_>>();
    keys.sort_by_key(|(sector, instrument)| (*sector as u8, *instrument as u8));
    keys.dedup();
    keys
}

fn union_keys(left: &HashMap<StockKey, i128>, right: &HashMap<StockKey, i128>) -> Vec<StockKey> {
    let mut keys = left.keys().chain(right.keys()).copied().collect::<Vec<_>>();
    keys.sort_by_key(|(sector, instrument)| (*sector as u8, *instrument as u8));
    keys.dedup();
    keys
}

fn hash_physical_postings(postings: &HashMap<PhysicalKey, i128>) -> Result<String, String> {
    let mut ordered = postings
        .iter()
        .map(|(key, value)| (key.0 as u8, key.1 as u8, *value))
        .collect::<Vec<_>>();
    ordered.sort_unstable();
    let bytes = serde_json::to_vec(&ordered)
        .map_err(|e| format!("failed to serialize physical postings: {e}"))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
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
    use crate::economics::stock_flow::{
        ActorBalanceSheet, CapitalInvestment, ProductionEvent, InventoryTransfer,
        InventoryConsumption, GoodsSale, InventoryCostAddition, InventoryCostRelief,
        Depreciation, CreditCreation, DebtRepayment, IncomeTransfer, MonetaryFlow,
    };
    use crate::economics::transition::apply_step;

    fn setup() -> (EconomicState, Vec<SectorAssignment>) {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        (
            EconomicState::new(vec![bank, ActorBalanceSheet::new("household"), ActorBalanceSheet::new("firm")]),
            vec![
                SectorAssignment { actor: "bank".into(), sector: EconomicSector::Bank },
                SectorAssignment { actor: "household".into(), sector: EconomicSector::Household },
                SectorAssignment { actor: "firm".into(), sector: EconomicSector::Firm },
            ],
        )
    }

    #[test]
    fn reconciliation_deltas_fail_closed_on_arithmetic_overflow() {
        let mut pre = SectorBalanceSheet::default();
        pre.entries.push(BalanceSheetEntry {
            sector: EconomicSector::Firm,
            instrument: BalanceSheetInstrument::Equity,
            amount: i128::MIN,
        });
        pre.entries.push(BalanceSheetEntry {
            sector: EconomicSector::Firm,
            instrument: BalanceSheetInstrument::Equity,
            amount: -1,
        });
        let post = SectorBalanceSheet::default();
        let err = balance_sheet_delta(&pre, &post).unwrap_err();
        assert!(err.contains("pre-state balance-sheet delta overflow"));
    }

    #[test]
    fn posting_aggregation_fails_closed_on_overflow() {
        let postings = vec![
            StockPosting::new(
                EconomicSector::Firm,
                BalanceSheetInstrument::Equity,
                i128::MAX,
            ),
            StockPosting::new(
                EconomicSector::Firm,
                BalanceSheetInstrument::Equity,
                1,
            ),
        ];
        assert!(aggregate_postings(postings).is_err());

        let physical = vec![
            PhysicalStockPosting::new(
                EconomicSector::Firm,
                PhysicalStockInstrument::Inventories,
                i128::MAX,
            ),
            PhysicalStockPosting::new(
                EconomicSector::Firm,
                PhysicalStockInstrument::Inventory,
                1,
            ),
        ];
        assert!(aggregate_physical_postings(physical).is_err());
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
        pre.actors.iter_mut()
            .find(|a| a.actor == "bank")
            .unwrap()
            .monetary.deposit_liabilities = 500;
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
    fn production_reconciles_inventory_and_resource_stocks() {
        let (mut pre, assignments) = setup();
        let firm = pre.actors.iter_mut().find(|a| a.actor == "firm").unwrap();
        firm.real.resources = 100;
        let transitions = vec![EconomicTransition::Production(
            ProductionEvent::new("firm", 30, 24).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        assert_eq!(post.actors.iter().find(|a| a.actor == "firm").unwrap().real.resources, 70);
        assert_eq!(post.actors.iter().find(|a| a.actor == "firm").unwrap().real.inventories, 24);
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn goods_sale_reconciles_physical_and_monetary_dimensions() {
        let (mut pre, assignments) = setup();
        pre.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 20;
        pre.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 100;
        pre.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.deposit_liabilities = 100;

        let transitions = vec![EconomicTransition::GoodsSale(
            GoodsSale::new("firm", "household", 5, 30).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        let receipt = reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
        assert_eq!(receipt.posting_count, 4);
        assert_eq!(receipt.physical_posting_count, 2);
    }

    #[test]
    fn inventory_cost_and_sale_reconcile_profit_carried_by_the_balance_sheet() {
        let (mut pre, assignments) = setup();
        {
            let firm = pre.actors.iter_mut().find(|a| a.actor == "firm").unwrap();
            firm.real.inventories = 20;
            firm.inventory_carrying_value = 80;
        }
        pre.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 100;
        pre.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.deposit_liabilities = 100;

        let transitions = vec![
            EconomicTransition::InventoryCostRelief(
                InventoryCostRelief::new("firm", 5, 20).unwrap(),
            ),
            EconomicTransition::GoodsSale(
                GoodsSale::new("firm", "household", 5, 30).unwrap(),
            ),
            EconomicTransition::InventoryCostAddition(
                InventoryCostAddition::new("household", 5, 30).unwrap(),
            ),
        ];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
        let firm = post.actors.iter().find(|a| a.actor == "firm").unwrap();
        let household = post.actors.iter().find(|a| a.actor == "household").unwrap();
        assert_eq!(firm.inventory_carrying_value, 60);
        assert_eq!(household.inventory_carrying_value, 30);
        assert_eq!(firm.monetary.deposits, 30);
        assert_eq!(household.monetary.deposits, 70);
    }

    #[test]
    fn goods_sale_reconciles_inventory_deposits_and_equity() {
        let (mut pre, assignments) = setup();
        pre.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 20;
        pre.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 100;
        pre.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.deposit_liabilities = 100;

        let transitions = vec![EconomicTransition::GoodsSale(
            GoodsSale::new("firm", "household", 5, 30).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn inventory_transfer_reconciles_physical_stock_and_equity_residuals() {
        let (mut pre, assignments) = setup();
        pre.actors.iter_mut()
            .find(|a| a.actor == "firm").unwrap()
            .real.inventories = 100;
        let transitions = vec![EconomicTransition::InventoryTransfer(
            InventoryTransfer::new("firm", "household", 40).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        assert_eq!(post.actors.iter().find(|a| a.actor == "firm").unwrap().real.inventories, 60);
        assert_eq!(post.actors.iter().find(|a| a.actor == "household").unwrap().real.inventories, 40);
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn inventory_consumption_reconciles_drawdown_and_equity_residual() {
        let (mut pre, assignments) = setup();
        pre.actors.iter_mut()
            .find(|a| a.actor == "household").unwrap()
            .real.inventories = 40;
        let transitions = vec![EconomicTransition::InventoryConsumption(
            InventoryConsumption::new("household", 15).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        assert_eq!(post.actors.iter().find(|a| a.actor == "household").unwrap().real.inventories, 25);
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn wage_payment_and_inventory_cost_capitalization_reconcile_separately() {
        let (mut pre, assignments) = setup();
        {
            let firm = pre.actors.iter_mut().find(|a| a.actor == "firm").unwrap();
            firm.monetary.deposits = 100;
            firm.real.inventories = 10;
            pre.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.deposit_liabilities = 100;
        }

        let transitions = vec![
            EconomicTransition::IncomeTransfer(
                IncomeTransfer::with_category(
                    "firm",
                    "household",
                    30,
                    crate::economics::stock_flow::EconomicFlowCategory::Wage,
                )
                .unwrap(),
            ),
            EconomicTransition::InventoryCostAddition(
                InventoryCostAddition::new("firm", 10, 30).unwrap(),
            ),
        ];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();

        let firm = post.actors.iter().find(|a| a.actor == "firm").unwrap();
        let household = post.actors.iter().find(|a| a.actor == "household").unwrap();
        assert_eq!(firm.monetary.deposits, 70);
        assert_eq!(firm.inventory_carrying_value, 30);
        assert_eq!(household.monetary.deposits, 30);
    }

    #[test]
    fn trade_credit_sale_and_settlement_reconcile_both_dimensions() {
        let mut pre = EconomicState::new(vec![
            super::super::stock_flow::ActorBalanceSheet::new("bank"),
            super::super::stock_flow::ActorBalanceSheet::new("firm"),
            super::super::stock_flow::ActorBalanceSheet::new("household"),
        ]);
        pre.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 10;
        pre.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 100;
        pre.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.deposit_liabilities = 100;

        let sale = super::super::stock_flow::TradeCreditSale::new("firm", "household", 4, 80).unwrap();
        let settlement = super::super::stock_flow::TradeCreditSettlement::new("firm", "household", 30).unwrap();
        let transitions = vec![
            EconomicTransition::TradeCreditSale(sale.clone()),
            EconomicTransition::TradeCreditSettlement(settlement.clone()),
        ];

        let mut post = pre.clone();
        post.apply_trade_credit_sale(&sale).unwrap();
        post.apply_trade_credit_settlement(&settlement).unwrap();

        let assignments = vec![
            SectorAssignment { actor: "bank".into(), sector: EconomicSector::Bank },
            SectorAssignment { actor: "firm".into(), sector: EconomicSector::Firm },
            SectorAssignment { actor: "household".into(), sector: EconomicSector::Household },
        ];
        reconcile_step(&pre, &post, &assignments, &transitions).unwrap();
    }

    #[test]
    fn trade_credit_sale_alone_reconciles_accrual_equity_and_physical_stock() {
        let (mut pre, assignments) = setup();
        pre.actors.iter_mut()
            .find(|a| a.actor == "firm")
            .unwrap()
            .real.inventories = 20;
        pre.actors.iter_mut()
            .find(|a| a.actor == "household")
            .unwrap()
            .monetary.deposits = 100;
        pre.actors.iter_mut()
            .find(|a| a.actor == "bank")
            .unwrap()
            .monetary.deposit_liabilities = 100;

        let transitions = vec![EconomicTransition::TradeCreditSale(
            super::super::stock_flow::TradeCreditSale::new(
                "firm", "household", 5, 30
            ).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        let receipt = reconcile_step(&pre, &post, &assignments, &transitions).unwrap();

        assert_eq!(receipt.posting_count, 4);
        assert_eq!(receipt.physical_posting_count, 2);
        assert_eq!(
            post.actors.iter().find(|a| a.actor == "firm").unwrap().monetary.trade_receivables,
            30
        );
        assert_eq!(
            post.actors.iter().find(|a| a.actor == "household").unwrap().monetary.trade_payables,
            30
        );
    }

    #[test]
    fn depreciation_reconciles_capital_and_equity() {
        let (mut pre, assignments) = setup();
        pre.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.productive_capital = 100;

        let transitions = vec![
            EconomicTransition::Depreciation(
                Depreciation::new("firm", 25).unwrap(),
            ),
        ];
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

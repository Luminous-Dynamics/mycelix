// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic period-level accounting derived from the transition ledger.
//!
//! This is intentionally a projection, not a second source of truth. All
//! totals are derived from the ordered transition list so semantic summaries
//! cannot drift away from the actual state-transition evidence.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::stock_flow::EconomicFlowCategory;
use super::transition::{transition_hash, EconomicStepError, EconomicTransition};

/// A deterministic summary of one ordered economic period.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct EconomicPeriodLedger {
    /// Semantic income/expenditure totals, keyed by economic category.
    pub category_totals: BTreeMap<EconomicFlowCategory, i128>,
    /// Gross value of ordinary monetary transfers.
    pub monetary_transfer_total: i128,
    /// Gross newly-created credit.
    pub credit_created: i128,
    /// Gross debt repayment.
    pub debt_repaid: i128,
    /// Gross capital formation settled during the period.
    pub investment: i128,
    /// Physical output added to inventories during the period.
    pub production_output: i128,
    /// Physical resources consumed by production during the period.
    pub resource_input: i128,
    /// Finished-goods units transferred between actors.
    pub inventory_transferred: i128,
    /// Finished-goods units explicitly drawn down by final use or loss.
    pub inventory_consumed: i128,
    /// Monetary consideration recognized from explicit goods sales.
    pub sales_consideration: i128,
    /// Physical units sold through explicit goods sales.
    pub sales_quantity: i128,
    /// Inventory carrying amount capitalized during the period.
    pub inventory_cost_added: i128,
    /// Inventory carrying amount relieved during the period (COGS at sale).
    pub cost_of_goods_sold: i128,
    /// Productive-capital carrying amount consumed through explicit depreciation.
    pub depreciation: i128,
    /// Number of transitions represented by the ledger.
    pub transition_count: u64,
    /// Hash of the exact ordered transition list from which this ledger came.
    pub transition_hash: String,
}

impl EconomicPeriodLedger {
    /// Derive a period ledger directly from the authoritative transition log.
    pub fn from_transitions(
        transitions: &[EconomicTransition],
    ) -> Result<Self, EconomicStepError> {
        let mut ledger = Self {
            transition_count: transitions.len() as u64,
            transition_hash: transition_hash(transitions)?,
            ..Self::default()
        };

        for transition in transitions {
            match transition {
                EconomicTransition::MonetaryTransfer(flow) => {
                    ledger.monetary_transfer_total = ledger
                        .monetary_transfer_total
                        .checked_add(flow.amount)
                        .ok_or_else(|| EconomicStepError::Serialization(
                            "period monetary transfer total overflow".into(),
                        ))?;
                }
                EconomicTransition::CapitalInvestment(investment) => {
                    ledger.investment = ledger
                        .investment
                        .checked_add(investment.amount)
                        .ok_or_else(|| EconomicStepError::Serialization(
                            "period investment total overflow".into(),
                        ))?;
                }
                EconomicTransition::IncomeTransfer(flow) => {
                    let entry = ledger.category_totals.entry(flow.category).or_insert(0);
                    *entry = entry.checked_add(flow.amount).ok_or_else(|| {
                        EconomicStepError::Serialization(
                            "period category total overflow".into(),
                        )
                    })?;
                }
                EconomicTransition::Production(production) => {
                    ledger.production_output = ledger
                        .production_output
                        .checked_add(production.output)
                        .ok_or_else(|| EconomicStepError::Serialization("period production output overflow".into()))?;
                    ledger.resource_input = ledger
                        .resource_input
                        .checked_add(production.resource_input)
                        .ok_or_else(|| EconomicStepError::Serialization("period resource input overflow".into()))?;
                }
                EconomicTransition::InventoryTransfer(transfer) => {
                    ledger.inventory_transferred = ledger
                        .inventory_transferred
                        .checked_add(transfer.quantity)
                        .ok_or_else(|| EconomicStepError::Serialization("period inventory transfer overflow".into()))?;
                }
                EconomicTransition::InventoryConsumption(consumption) => {
                    ledger.inventory_consumed = ledger
                        .inventory_consumed
                        .checked_add(consumption.quantity)
                        .ok_or_else(|| EconomicStepError::Serialization("period inventory consumption overflow".into()))?;
                }
                EconomicTransition::GoodsSale(sale) => {
                    ledger.sales_quantity = ledger
                        .sales_quantity
                        .checked_add(sale.quantity)
                        .ok_or_else(|| EconomicStepError::Serialization("period sales quantity overflow".into()))?;
                    ledger.sales_consideration = ledger
                        .sales_consideration
                        .checked_add(sale.consideration)
                        .ok_or_else(|| EconomicStepError::Serialization("period sales consideration overflow".into()))?;
                }
                EconomicTransition::InventoryCostAddition(addition) => {
                    ledger.inventory_cost_added = ledger
                        .inventory_cost_added
                        .checked_add(addition.carrying_value)
                        .ok_or_else(|| EconomicStepError::Serialization("period inventory cost addition overflow".into()))?;
                }
                EconomicTransition::InventoryCostRelief(relief) => {
                    ledger.cost_of_goods_sold = ledger
                        .cost_of_goods_sold
                        .checked_add(relief.carrying_value)
                        .ok_or_else(|| EconomicStepError::Serialization("period COGS overflow".into()))?;
                }
                EconomicTransition::Depreciation(depreciation) => {
                    ledger.depreciation = ledger
                        .depreciation
                        .checked_add(depreciation.amount)
                        .ok_or_else(|| EconomicStepError::Serialization("period depreciation overflow".into()))?;
                }
                EconomicTransition::CreditCreation(credit) => {
                    ledger.credit_created = ledger
                        .credit_created
                        .checked_add(credit.amount)
                        .ok_or_else(|| EconomicStepError::Serialization(
                            "period credit total overflow".into(),
                        ))?;
                }
                EconomicTransition::DebtRepayment(repayment) => {
                    ledger.debt_repaid = ledger
                        .debt_repaid
                        .checked_add(repayment.amount)
                        .ok_or_else(|| EconomicStepError::Serialization(
                            "period repayment total overflow".into(),
                        ))?;
                }
            }
        }

        Ok(ledger)
    }

    pub fn category_total(&self, category: EconomicFlowCategory) -> i128 {
        self.category_totals.get(&category).copied().unwrap_or(0)
    }

    pub fn net_credit(&self) -> i128 {
        self.credit_created - self.debt_repaid
    }

    /// Gross sales revenue less explicit COGS reliefs. This is a gross trading
    /// surplus measure; wages, intermediate costs, depreciation, interest and
    /// taxes remain separate until their own accounting boundaries are applied.
    pub fn gross_operating_surplus(&self) -> i128 {
        self.sales_consideration - self.cost_of_goods_sold
    }

    /// Gross operating surplus less separately recognized depreciation.
    ///
    /// This is intentionally not a complete profit measure. Interest, taxes,
    /// other operating expenses, and any depreciation already capitalized into
    /// inventory and later included in COGS must not be counted again.
    pub fn operating_surplus_after_depreciation(&self) -> i128 {
        self.gross_operating_surplus() - self.depreciation
    }

    /// Hash the complete derived ledger for evidence binding.
    pub fn hash(&self) -> Result<String, EconomicStepError> {
        let bytes = serde_json::to_vec(self)
            .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        Ok(blake3::hash(&bytes).to_hex().to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{
        CapitalInvestment, ProductionEvent, InventoryTransfer, InventoryConsumption, GoodsSale, CreditCreation, DebtRepayment, EconomicFlowCategory, IncomeTransfer, MonetaryFlow,
    };

    #[test]
    fn production_is_aggregated_without_becoming_a_monetary_flow() {
        let transitions = vec![EconomicTransition::Production(
            ProductionEvent::new("firm", 30, 24).unwrap(),
        )];
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        assert_eq!(ledger.production_output, 24);
        assert_eq!(ledger.resource_input, 30);
        assert_eq!(ledger.monetary_transfer_total, 0);
        assert_eq!(ledger.investment, 0);
    }

    #[test]
    fn inventory_flows_are_aggregated_without_monetary_side_effects() {
        let transitions = vec![
            EconomicTransition::InventoryTransfer(
                InventoryTransfer::new("firm", "household", 12).unwrap(),
            ),
            EconomicTransition::InventoryConsumption(
                InventoryConsumption::new("household", 5).unwrap(),
            ),
        ];
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        assert_eq!(ledger.inventory_transferred, 12);
        assert_eq!(ledger.inventory_consumed, 5);
        assert_eq!(ledger.monetary_transfer_total, 0);
        assert_eq!(ledger.credit_created, 0);
        assert_eq!(ledger.debt_repaid, 0);
    }

    #[test]
    fn depreciation_is_derived_without_becoming_sales_or_cogs() {
        let transitions = vec![
            EconomicTransition::Depreciation(
                Depreciation::new("firm", 15).unwrap(),
            ),
        ];
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        assert_eq!(ledger.depreciation, 15);
        assert_eq!(ledger.sales_consideration, 0);
        assert_eq!(ledger.cost_of_goods_sold, 0);
        assert_eq!(ledger.operating_surplus_after_depreciation(), -15);
    }

    #[test]
    fn sales_and_cogs_produce_an_explicit_gross_operating_surplus() {
        let transitions = vec![
            EconomicTransition::InventoryCostAddition(
                InventoryCostAddition::new("firm", 10, 80).unwrap(),
            ),
            EconomicTransition::InventoryCostRelief(
                InventoryCostRelief::new("firm", 5, 40).unwrap(),
            ),
            EconomicTransition::GoodsSale(
                GoodsSale::new("firm", "household", 5, 60).unwrap(),
            ),
            EconomicTransition::InventoryCostAddition(
                InventoryCostAddition::new("household", 5, 60).unwrap(),
            ),
        ];
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        assert_eq!(ledger.sales_consideration, 60);
        assert_eq!(ledger.sales_quantity, 5);
        assert_eq!(ledger.inventory_cost_added, 140);
        assert_eq!(ledger.cost_of_goods_sold, 40);
        assert_eq!(ledger.gross_operating_surplus(), 20);
    }

    #[test]
    fn categories_are_aggregated_deterministically() {
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
            EconomicTransition::IncomeTransfer(
                IncomeTransfer::with_category(
                    "bank",
                    "household",
                    20,
                    EconomicFlowCategory::Interest,
                )
                .unwrap(),
            ),
            EconomicTransition::IncomeTransfer(
                IncomeTransfer::with_category(
                    "household",
                    "government",
                    30,
                    EconomicFlowCategory::Tax,
                )
                .unwrap(),
            ),
            EconomicTransition::CapitalInvestment(
                CapitalInvestment::new("firm", "capital-producer", 200).unwrap(),
            ),
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "firm", 500).unwrap(),
            ),
            EconomicTransition::DebtRepayment(
                DebtRepayment::new("bank", "firm", 100).unwrap(),
            ),
            EconomicTransition::MonetaryTransfer(
                MonetaryFlow::deposit_transfer("household", "firm", 25).unwrap(),
            ),
        ];

        let a = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        let b = EconomicPeriodLedger::from_transitions(&transitions).unwrap();

        assert_eq!(a, b);
        assert_eq!(a.category_total(EconomicFlowCategory::Wage), 100);
        assert_eq!(a.category_total(EconomicFlowCategory::Interest), 20);
        assert_eq!(a.category_total(EconomicFlowCategory::Tax), 30);
        assert_eq!(a.investment, 200);
        assert_eq!(a.credit_created, 500);
        assert_eq!(a.debt_repaid, 100);
        assert_eq!(a.net_credit(), 400);
        assert_eq!(a.monetary_transfer_total, 25);
        assert_eq!(a.transition_count, 6);
        assert_eq!(a.hash().unwrap(), b.hash().unwrap());
    }

    #[test]
    fn category_is_part_of_transition_evidence() {
        let wage = vec![EconomicTransition::IncomeTransfer(
            IncomeTransfer::with_category(
                "firm",
                "household",
                100,
                EconomicFlowCategory::Wage,
            )
            .unwrap(),
        )];
        let interest = vec![EconomicTransition::IncomeTransfer(
            IncomeTransfer::with_category(
                "firm",
                "household",
                100,
                EconomicFlowCategory::Interest,
            )
            .unwrap(),
        )];

        let wage_ledger = EconomicPeriodLedger::from_transitions(&wage).unwrap();
        let interest_ledger = EconomicPeriodLedger::from_transitions(&interest).unwrap();

        assert_ne!(wage_ledger.transition_hash, interest_ledger.transition_hash);
        assert_ne!(wage_ledger.hash().unwrap(), interest_ledger.hash().unwrap());
    }
}

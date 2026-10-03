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
    /// Monetary consideration recognized from explicit goods sales,
    /// including deferred trade-credit sales.
    pub sales_consideration: i128,
    /// Gross consideration newly placed on trade credit.
    pub trade_credit_extended: i128,
    /// Gross trade-credit consideration settled through deposits.
    pub trade_credit_settled: i128,
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
            transition
                .validate()
                .map_err(EconomicStepError::TransitionRejected)?;

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
                EconomicTransition::TradeCreditSale(sale) => {
                    ledger.sales_quantity = ledger
                        .sales_quantity
                        .checked_add(sale.quantity)
                        .ok_or_else(|| EconomicStepError::Serialization("period trade-credit sales quantity overflow".into()))?;
                    ledger.sales_consideration = ledger
                        .sales_consideration
                        .checked_add(sale.consideration)
                        .ok_or_else(|| EconomicStepError::Serialization("period trade-credit sales consideration overflow".into()))?;
                    ledger.trade_credit_extended = ledger
                        .trade_credit_extended
                        .checked_add(sale.consideration)
                        .ok_or_else(|| EconomicStepError::Serialization("period trade credit extension overflow".into()))?;
                }
                EconomicTransition::TradeCreditSettlement(settlement) => {
                    ledger.trade_credit_settled = ledger
                        .trade_credit_settled
                        .checked_add(settlement.amount)
                        .ok_or_else(|| EconomicStepError::Serialization("period trade credit settlement overflow".into()))?;
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

    /// Verify that this serialized ledger exactly matches a supplied ordered
    /// transition list, including every derived total and the transition hash.
    ///
    /// The ledger stores only a digest of its source transitions, so the digest
    /// alone cannot prove that the derived fields were computed from that list.
    /// Re-derivation closes that provenance gap without mutating the ledger.
    pub fn verify_against_transitions(
        &self,
        transitions: &[EconomicTransition],
    ) -> Result<(), EconomicStepError> {
        let expected = Self::from_transitions(transitions)?;
        if self != &expected {
            return Err(EconomicStepError::Serialization(
                "period ledger does not match supplied transition list".into(),
            ));
        }
        Ok(())
    }

    pub fn category_total(&self, category: EconomicFlowCategory) -> i128 {
        self.category_totals.get(&category).copied().unwrap_or(0)
    }

    pub fn try_net_credit(&self) -> Result<i128, String> {
        self.credit_created
            .checked_sub(self.debt_repaid)
            .ok_or_else(|| "period net credit overflow".into())
    }

    pub fn net_credit(&self) -> i128 {
        self.try_net_credit()
            .expect("period net credit overflow")
    }

    /// Gross sales revenue less explicit COGS reliefs. This is a gross trading
    /// surplus measure; wages, intermediate costs, depreciation, interest and
    /// taxes remain separate until their own accounting boundaries are applied.
    pub fn try_gross_operating_surplus(&self) -> Result<i128, String> {
        self.sales_consideration
            .checked_sub(self.cost_of_goods_sold)
            .ok_or_else(|| "period gross operating surplus overflow".into())
    }

    pub fn gross_operating_surplus(&self) -> i128 {
        self.try_gross_operating_surplus()
            .expect("period gross operating surplus overflow")
    }

    /// Gross operating surplus less separately recognized depreciation.
    ///
    /// This is intentionally not a complete profit measure. Interest, taxes,
    /// other operating expenses, and any depreciation already capitalized into
    /// inventory and later included in COGS must not be counted again.
    pub fn try_operating_surplus_after_depreciation(&self) -> Result<i128, String> {
        self.try_gross_operating_surplus()?
            .checked_sub(self.depreciation)
            .ok_or_else(|| "period operating surplus after depreciation overflow".into())
    }

    pub fn operating_surplus_after_depreciation(&self) -> i128 {
        self.try_operating_surplus_after_depreciation()
            .expect("period operating surplus after depreciation overflow")
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
        CapitalInvestment, CreditCreation, DebtRepayment, Depreciation, EconomicFlowCategory,
        GoodsSale, IncomeTransfer, InventoryConsumption, InventoryCostAddition,
        InventoryCostRelief, InventoryTransfer, MonetaryFlow, ProductionEvent,
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
    fn trade_credit_is_separated_from_deposit_settled_sales() {
        let transitions = vec![
            EconomicTransition::TradeCreditSale(
                super::super::stock_flow::TradeCreditSale::new("firm", "household", 4, 80).unwrap(),
            ),
            EconomicTransition::TradeCreditSettlement(
                super::super::stock_flow::TradeCreditSettlement::new("firm", "household", 30).unwrap(),
            ),
        ];
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        assert_eq!(ledger.sales_quantity, 4);
        assert_eq!(ledger.sales_consideration, 80);
        assert_eq!(ledger.trade_credit_extended, 80);
        assert_eq!(ledger.trade_credit_settled, 30);
        assert_eq!(ledger.monetary_transfer_total, 0);
    }

    #[test]
    fn checked_derived_totals_fail_closed_on_subtraction_overflow() {
        let net_credit = EconomicPeriodLedger {
            credit_created: i128::MIN,
            debt_repaid: 1,
            ..EconomicPeriodLedger::default()
        };
        assert!(net_credit.try_net_credit().is_err());

        let surplus = EconomicPeriodLedger {
            sales_consideration: i128::MIN,
            cost_of_goods_sold: 1,
            ..EconomicPeriodLedger::default()
        };
        assert!(surplus.try_gross_operating_surplus().is_err());

        let after_depreciation = EconomicPeriodLedger {
            sales_consideration: i128::MIN,
            depreciation: 1,
            ..EconomicPeriodLedger::default()
        };
        assert!(after_depreciation
            .try_operating_surplus_after_depreciation()
            .is_err());
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
        assert_eq!(a.transition_count, 7);
        assert_eq!(a.hash().unwrap(), b.hash().unwrap());
    }

    #[test]
    fn verify_against_transitions_rejects_tampered_derived_fields() {
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "firm", 500).unwrap(),
        )];
        let mut ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        ledger.credit_created = 499;

        assert!(matches!(
            ledger.verify_against_transitions(&transitions),
            Err(EconomicStepError::Serialization(message))
                if message.contains("does not match")
        ));
    }

    #[test]
    fn verify_against_transitions_binds_order_and_hash() {
        let transitions = vec![
            EconomicTransition::MonetaryTransfer(
                MonetaryFlow::deposit_transfer("household", "firm", 25).unwrap(),
            ),
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "firm", 500).unwrap(),
            ),
        ];
        let reversed = vec![
            transitions[1].clone(),
            transitions[0].clone(),
        ];
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();

        ledger.verify_against_transitions(&transitions).unwrap();
        assert!(ledger.verify_against_transitions(&reversed).is_err());
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

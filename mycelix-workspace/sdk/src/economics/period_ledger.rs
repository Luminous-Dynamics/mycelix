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
        CapitalInvestment, ProductionEvent, CreditCreation, DebtRepayment, EconomicFlowCategory, IncomeTransfer, MonetaryFlow,
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

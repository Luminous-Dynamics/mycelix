// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Actor-level derived economic observations.
//!
//! These observations are projections of reconciled stocks and the ordered
//! transition log. They do not mutate state and do not encode policy judgments.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::observables::{classify_financing_regime, FinancingRegime, RatioObservation};
use super::stock_flow::{
    ActorId, EconomicFlowCategory, EconomicState,
};
use super::transition::EconomicTransition;

fn add_checked(slot: &mut i128, amount: i128, label: &str) -> Result<(), String> {
    *slot = slot
        .checked_add(amount)
        .ok_or_else(|| format!("{label} overflow"))?;
    Ok(())
}

fn liquidity(state: &EconomicState, actor: &str) -> Result<i128, String> {
    let balance_sheet = state
        .actors
        .iter()
        .find(|a| a.actor == actor)
        .ok_or_else(|| format!("unknown actor in economic transition: {actor}"))?;
    balance_sheet
        .monetary
        .cash
        .checked_add(balance_sheet.monetary.deposits)
        .ok_or_else(|| format!("liquidity overflow for {actor}"))
}

/// Derived stock/flow observations for one actor over one period.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ActorEconomicObservables {
    pub actor: ActorId,
    pub cash: i128,
    pub deposits: i128,
    pub liquidity: i128,
    pub net_liquidity_change: i128,
    pub loan_claims: i128,
    pub debt: i128,
    pub inventory_quantity: i128,
    pub inventory_carrying_value: i128,
    pub productive_capital: i128,
    pub net_financial_position: i128,
    pub net_worth: i128,

    pub credit_received: i128,
    pub credit_originated: i128,
    pub debt_repaid: i128,
    pub interest_paid: i128,
    pub interest_received: i128,
    pub wages_paid: i128,
    pub wages_received: i128,
    pub taxes_paid: i128,
    pub transfers_paid: i128,
    pub transfers_received: i128,
    pub consumption_paid: i128,
    pub investment_paid: i128,
    pub investment_received: i128,
    pub sales_revenue: i128,
    pub goods_purchases: i128,
    pub cost_of_goods_sold: i128,
    pub depreciation: i128,
}

impl ActorEconomicObservables {
    pub fn from_state_and_transitions(
        state: &EconomicState,
        transitions: &[EconomicTransition],
    ) -> Result<BTreeMap<ActorId, Self>, String> {
        let mut observations = BTreeMap::new();

        for actor in &state.actors {
            observations.insert(
                actor.actor.clone(),
                Self {
                    actor: actor.actor.clone(),
                    cash: actor.monetary.cash,
                    deposits: actor.monetary.deposits,
                    liquidity: actor.monetary.cash + actor.monetary.deposits,
                    loan_claims: actor.monetary.claims,
                    debt: actor.monetary.liabilities,
                    inventory_quantity: actor.real.inventories,
                    inventory_carrying_value: actor.inventory_carrying_value,
                    productive_capital: actor.real.productive_capital,
                    net_financial_position: actor.monetary.net_position(),
                    net_worth: actor.net_worth(),
                    ..Self::default()
                },
            );
        }

        let mut ensure_actor = |actor: &str| -> Result<(), String> {
            if !observations.contains_key(actor) {
                return Err(format!("unknown actor in economic transition: {actor}"));
            }
            Ok(())
        };

        let mut working = state.clone();

        for transition in transitions {
            let affected = affected_actors(transition);
            for actor in &affected {
                ensure_actor(actor)?;
            }
            let before_liquidity = affected
                .iter()
                .map(|actor| (actor.clone(), liquidity(&working, actor)))
                .collect::<Result<Vec<_>, _>>()?;


            match transition {
                EconomicTransition::CreditCreation(credit) => {
                    ensure_actor(&credit.lender)?;
                    ensure_actor(&credit.borrower)?;
                    add_checked(
                        &mut observations.get_mut(&credit.lender).unwrap().credit_originated,
                        credit.amount,
                        "actor credit originated",
                    )?;
                    add_checked(
                        &mut observations.get_mut(&credit.borrower).unwrap().credit_received,
                        credit.amount,
                        "actor credit received",
                    )?;
                }
                EconomicTransition::DebtRepayment(repayment) => {
                    ensure_actor(&repayment.lender)?;
                    ensure_actor(&repayment.borrower)?;
                    add_checked(
                        &mut observations.get_mut(&repayment.borrower).unwrap().debt_repaid,
                        repayment.amount,
                        "actor debt repaid",
                    )?;
                }
                EconomicTransition::IncomeTransfer(flow) => {
                    ensure_actor(&flow.payer)?;
                    ensure_actor(&flow.recipient)?;
                    match flow.category {
                        EconomicFlowCategory::Wage => {
                            add_checked(&mut observations.get_mut(&flow.payer).unwrap().wages_paid, flow.amount, "actor wages paid")?;
                            add_checked(&mut observations.get_mut(&flow.recipient).unwrap().wages_received, flow.amount, "actor wages received")?;
                        }
                        EconomicFlowCategory::Interest => {
                            add_checked(&mut observations.get_mut(&flow.payer).unwrap().interest_paid, flow.amount, "actor interest paid")?;
                            add_checked(&mut observations.get_mut(&flow.recipient).unwrap().interest_received, flow.amount, "actor interest received")?;
                        }
                        EconomicFlowCategory::Tax => {
                            add_checked(&mut observations.get_mut(&flow.payer).unwrap().taxes_paid, flow.amount, "actor taxes paid")?;
                        }
                        EconomicFlowCategory::Transfer => {
                            add_checked(&mut observations.get_mut(&flow.payer).unwrap().transfers_paid, flow.amount, "actor transfers paid")?;
                            add_checked(&mut observations.get_mut(&flow.recipient).unwrap().transfers_received, flow.amount, "actor transfers received")?;
                        }
                        EconomicFlowCategory::Consumption => {
                            add_checked(&mut observations.get_mut(&flow.payer).unwrap().consumption_paid, flow.amount, "actor consumption paid")?;
                        }
                        EconomicFlowCategory::Investment => {
                            add_checked(&mut observations.get_mut(&flow.payer).unwrap().investment_paid, flow.amount, "actor investment paid")?;
                            add_checked(&mut observations.get_mut(&flow.recipient).unwrap().investment_received, flow.amount, "actor investment received")?;
                        }
                    }
                }
                EconomicTransition::CapitalInvestment(investment) => {
                    ensure_actor(&investment.buyer)?;
                    ensure_actor(&investment.producer)?;
                    add_checked(&mut observations.get_mut(&investment.buyer).unwrap().investment_paid, investment.amount, "actor investment paid")?;
                    add_checked(&mut observations.get_mut(&investment.producer).unwrap().investment_received, investment.amount, "actor investment received")?;
                }
                EconomicTransition::GoodsSale(sale) => {
                    ensure_actor(&sale.seller)?;
                    ensure_actor(&sale.buyer)?;
                    add_checked(&mut observations.get_mut(&sale.seller).unwrap().sales_revenue, sale.consideration, "actor sales revenue")?;
                    add_checked(&mut observations.get_mut(&sale.buyer).unwrap().goods_purchases, sale.consideration, "actor goods purchases")?;
                }
                EconomicTransition::InventoryCostRelief(relief) => {
                    ensure_actor(&relief.actor)?;
                    add_checked(
                        &mut observations.get_mut(&relief.actor).unwrap().cost_of_goods_sold,
                        relief.carrying_value,
                        "actor COGS",
                    )?;
                }
                EconomicTransition::Depreciation(depreciation) => {
                    ensure_actor(&depreciation.actor)?;
                    add_checked(
                        &mut observations.get_mut(&depreciation.actor).unwrap().depreciation,
                        depreciation.amount,
                        "actor depreciation",
                    )?;
                }
                EconomicTransition::MonetaryTransfer(_)
                | EconomicTransition::Production(_)
                | EconomicTransition::InventoryTransfer(_)
                | EconomicTransition::InventoryConsumption(_)
                | EconomicTransition::InventoryCostAddition(_) => {}
            }

            apply_transition(&mut working, transition)?;
            for (actor, before) in before_liquidity {
                let after = liquidity(&working, &actor)?;
                let delta = after
                    .checked_sub(before)
                    .ok_or_else(|| format!("liquidity delta overflow for {actor}"))?;
                add_checked(
                    &mut observations.get_mut(&actor).unwrap().net_liquidity_change,
                    delta,
                    "actor net liquidity change",
                )?;
            }
        }

        Ok(observations)
    }

    pub fn gross_debt_service(&self) -> i128 {
        self.interest_paid + self.debt_repaid
    }

    pub fn gross_surplus(&self) -> i128 {
        self.sales_revenue - self.cost_of_goods_sold
    }

    pub fn operating_surplus_after_depreciation(&self) -> i128 {
        self.gross_surplus() - self.depreciation
    }

    pub fn financing_net_liquidity(&self) -> i128 {
        self.credit_received - self.debt_repaid
    }

    pub fn investing_net_liquidity(&self) -> i128 {
        self.investment_received - self.investment_paid
    }

    /// Liquidity change not explained by explicitly classified financing or
    /// investment transitions. This is a residual, not a claim that every
    /// remaining flow is operating cash flow.
    pub fn non_financing_liquidity_change(&self) -> i128 {
        self.net_liquidity_change
            - self.financing_net_liquidity()
            - self.investing_net_liquidity()
    }

    pub fn leverage(&self) -> Option<RatioObservation> {
        let assets = self.cash
            + self.deposits
            + self.loan_claims;
        (assets != 0).then_some(RatioObservation {
            numerator: self.debt,
            denominator: assets,
        })
    }

    /// Classify this actor's financing position from an externally supplied
    /// cash-flow measure and explicit contractual debt-service obligations.
    ///
    /// Actual repayment is deliberately not substituted for principal due:
    /// failing to repay principal must not make a position look more covered.
    pub fn financing_regime(
        &self,
        cash_flow_available: i128,
        interest_due: i128,
        principal_due: i128,
    ) -> Result<FinancingRegime, String> {
        classify_financing_regime(cash_flow_available, interest_due, principal_due)
    }
}

fn affected_actors(transition: &EconomicTransition) -> Vec<ActorId> {
    match transition {
        EconomicTransition::MonetaryTransfer(flow) => vec![flow.from.clone(), flow.to.clone()],
        EconomicTransition::IncomeTransfer(flow) => vec![flow.payer.clone(), flow.recipient.clone()],
        EconomicTransition::CapitalInvestment(investment) => {
            vec![investment.buyer.clone(), investment.producer.clone()]
        }
        EconomicTransition::Production(event) => vec![event.producer.clone()],
        EconomicTransition::InventoryTransfer(transfer) => {
            vec![transfer.from.clone(), transfer.to.clone()]
        }
        EconomicTransition::InventoryConsumption(consumption) => vec![consumption.consumer.clone()],
        EconomicTransition::GoodsSale(sale) => vec![sale.seller.clone(), sale.buyer.clone()],
        EconomicTransition::InventoryCostAddition(addition) => vec![addition.actor.clone()],
        EconomicTransition::InventoryCostRelief(relief) => vec![relief.actor.clone()],
        EconomicTransition::Depreciation(depreciation) => vec![depreciation.actor.clone()],
        EconomicTransition::CreditCreation(credit) => vec![credit.lender.clone(), credit.borrower.clone()],
        EconomicTransition::DebtRepayment(repayment) => {
            vec![repayment.lender.clone(), repayment.borrower.clone()]
        }
    }
}

fn apply_transition(
    state: &mut EconomicState,
    transition: &EconomicTransition,
) -> Result<(), String> {
    match transition {
        EconomicTransition::MonetaryTransfer(flow) => state.apply_flow(flow),
        EconomicTransition::IncomeTransfer(flow) => state.apply_income_transfer(flow),
        EconomicTransition::CapitalInvestment(investment) => {
            state.apply_capital_investment(investment)
        }
        EconomicTransition::Production(event) => state.apply_production(event),
        EconomicTransition::InventoryTransfer(transfer) => {
            state.apply_inventory_transfer(transfer)
        }
        EconomicTransition::InventoryConsumption(consumption) => {
            state.apply_inventory_consumption(consumption)
        }
        EconomicTransition::GoodsSale(sale) => state.apply_goods_sale(sale),
        EconomicTransition::InventoryCostAddition(addition) => {
            state.apply_inventory_cost_addition(addition)
        }
        EconomicTransition::InventoryCostRelief(relief) => {
            state.apply_inventory_cost_relief(relief)
        }
        EconomicTransition::Depreciation(depreciation) => state.apply_depreciation(depreciation),
        EconomicTransition::CreditCreation(credit) => state.create_credit(credit),
        EconomicTransition::DebtRepayment(repayment) => state.repay_debt(repayment),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{
        ActorBalanceSheet, CreditCreation, DebtRepayment, EconomicState, IncomeTransfer,
        GoodsSale, EconomicFlowCategory,
    };

    #[test]
    fn actor_observables_are_deterministic_and_semantic() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.deposits = 500;
        firm.real.inventories = 10;
        firm.inventory_carrying_value = 80;

        let mut household = ActorBalanceSheet::new("household");
        household.monetary.deposits = 100;

        let state = EconomicState::new(vec![bank, firm, household]);
        let transitions = vec![
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "firm", 200).unwrap(),
            ),
            EconomicTransition::IncomeTransfer(
                IncomeTransfer::with_category(
                    "firm",
                    "household",
                    30,
                    EconomicFlowCategory::Wage,
                )
                .unwrap(),
            ),
            EconomicTransition::GoodsSale(
                GoodsSale::new("firm", "household", 5, 60).unwrap(),
            ),
            EconomicTransition::InventoryCostRelief(
                super::super::stock_flow::InventoryCostRelief::new("firm", 5, 40).unwrap(),
            ),
            EconomicTransition::DebtRepayment(
                DebtRepayment::new("bank", "firm", 20).unwrap(),
            ),
        ];

        let a = ActorEconomicObservables::from_state_and_transitions(&state, &transitions).unwrap();
        let b = ActorEconomicObservables::from_state_and_transitions(&state, &transitions).unwrap();
        assert_eq!(a, b);

        let firm = &a["firm"];
        assert_eq!(firm.credit_received, 200);
        assert_eq!(firm.wages_paid, 30);
        assert_eq!(firm.sales_revenue, 60);
        assert_eq!(firm.cost_of_goods_sold, 40);
        assert_eq!(firm.debt_repaid, 20);
        assert_eq!(firm.gross_surplus(), 20);
        assert_eq!(firm.gross_debt_service(), 20);
        assert_eq!(
            firm.financing_regime(20, 20, 0).unwrap(),
            FinancingRegime::Hedge
        );
        assert_eq!(firm.net_liquidity_change, 210);
        assert_eq!(firm.financing_net_liquidity(), 180);
        assert_eq!(firm.investing_net_liquidity(), 0);
    }

    #[test]
    fn financing_regime_uses_principal_due_not_actual_repayment() {
        let observables = ActorEconomicObservables::default();
        assert_eq!(
            observables.financing_regime(100, 50, 100).unwrap(),
            FinancingRegime::Speculative
        );
    }

    #[test]
    fn unknown_transition_actor_is_rejected() {
        let state = EconomicState::new(vec![ActorBalanceSheet::new("firm")]);
        let transitions = vec![EconomicTransition::GoodsSale(
            GoodsSale::new("firm", "missing", 1, 2).unwrap(),
        )];
        assert!(ActorEconomicObservables::from_state_and_transitions(&state, &transitions).is_err());
    }
}

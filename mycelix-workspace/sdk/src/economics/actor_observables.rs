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
use super::stock_flow::{ActorId, EconomicFlowCategory, EconomicState};
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
    #[serde(default)]
    pub opening_liquidity: i128,
    pub liquidity: i128,
    pub net_liquidity_change: i128,
    /// Liquidity changes from generic monetary transfers whose economic
    /// purpose is not further classified by the transition type.
    pub other_liquidity_change: i128,
    #[serde(default)]
    pub opening_net_working_capital: i128,
    pub net_working_capital_change: i128,
    pub loan_claims: i128,
    pub debt: i128,
    pub trade_receivables: i128,
    pub trade_payables: i128,
    pub inventory_quantity: i128,
    pub inventory_carrying_value: i128,
    pub productive_capital: i128,
    pub net_financial_position: i128,
    pub net_worth: i128,

    pub credit_received: i128,
    pub credit_originated: i128,
    pub debt_repaid: i128,
    pub trade_credit_received: i128,
    pub trade_credit_extended: i128,
    pub trade_credit_settled: i128,
    pub trade_credit_collected: i128,
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
    /// Derive period-end actor stocks and period flows by replaying an ordered
    /// transition sequence from the supplied starting state.
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
                    opening_liquidity: actor
                        .monetary
                        .cash
                        .checked_add(actor.monetary.deposits)
                        .ok_or_else(|| format!("liquidity overflow for {}", actor.actor))?,
                    opening_net_working_capital: actor.try_net_working_capital()?,
                    ..Self::default()
                },
            );
        }

        if observations.len() != state.actors.len() {
            return Err("economic state contains duplicate actor ids".into());
        }

        let mut ensure_actor = |actor: &str| -> Result<(), String> {
            if !observations.contains_key(actor) {
                return Err(format!("unknown actor in economic transition: {actor}"));
            }
            Ok(())
        };

        let initial_working_capital = state
            .actors
            .iter()
            .map(|actor| -> Result<_, String> {
                Ok((actor.actor.clone(), actor.try_net_working_capital()?))
            })
            .collect::<Result<BTreeMap<_, _>, _>>()?;

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
                EconomicTransition::TradeCreditSale(sale) => {
                    ensure_actor(&sale.seller)?;
                    ensure_actor(&sale.buyer)?;
                    add_checked(
                        &mut observations.get_mut(&sale.seller).unwrap().trade_credit_extended,
                        sale.consideration,
                        "actor trade credit extended",
                    )?;
                    add_checked(
                        &mut observations.get_mut(&sale.buyer).unwrap().trade_credit_received,
                        sale.consideration,
                        "actor trade credit received",
                    )?;
                    add_checked(
                        &mut observations.get_mut(&sale.seller).unwrap().sales_revenue,
                        sale.consideration,
                        "actor trade-credit sales revenue",
                    )?;
                    add_checked(
                        &mut observations.get_mut(&sale.buyer).unwrap().goods_purchases,
                        sale.consideration,
                        "actor trade-credit goods purchases",
                    )?;
                }
                EconomicTransition::TradeCreditSettlement(settlement) => {
                    ensure_actor(&settlement.seller)?;
                    ensure_actor(&settlement.buyer)?;
                    add_checked(
                        &mut observations.get_mut(&settlement.buyer).unwrap().trade_credit_settled,
                        settlement.amount,
                        "actor trade credit settled",
                    )?;
                    add_checked(
                        &mut observations.get_mut(&settlement.seller).unwrap().trade_credit_collected,
                        settlement.amount,
                        "actor trade credit collected",
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
                EconomicTransition::MonetaryTransfer(flow) => {
                    ensure_actor(&flow.from)?;
                    ensure_actor(&flow.to)?;
                    let amount = flow.amount;
                    add_checked(
                        &mut observations.get_mut(&flow.from).unwrap().other_liquidity_change,
                        -amount,
                        "actor other liquidity change",
                    )?;
                    add_checked(
                        &mut observations.get_mut(&flow.to).unwrap().other_liquidity_change,
                        amount,
                        "actor other liquidity change",
                    )?;
                }
                EconomicTransition::Production(_)
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

        for (actor, observation) in observations.iter_mut() {
            let final_actor = working
                .actors
                .iter()
                .find(|candidate| &candidate.actor == actor)
                .ok_or_else(|| format!("unknown actor after transition replay: {actor}"))?;
            observation.cash = final_actor.monetary.cash;
            observation.deposits = final_actor.monetary.deposits;
            observation.liquidity = final_actor
                .monetary
                .cash
                .checked_add(final_actor.monetary.deposits)
                .ok_or_else(|| format!("liquidity overflow for {actor}"))?;
            observation.loan_claims = final_actor.monetary.claims;
            observation.debt = final_actor.monetary.liabilities;
            observation.trade_receivables = final_actor.monetary.trade_receivables;
            observation.trade_payables = final_actor.monetary.trade_payables;
            observation.inventory_quantity = final_actor.real.inventories;
            observation.inventory_carrying_value = final_actor.inventory_carrying_value;
            observation.productive_capital = final_actor.real.productive_capital;
            observation.net_financial_position = final_actor.monetary.try_net_position()?;
            observation.net_worth = final_actor.try_net_worth()?;

            let final_nwc = final_actor.try_net_working_capital()?;
            let initial_nwc = *initial_working_capital
                .get(actor)
                .ok_or_else(|| format!("unknown initial actor working capital: {actor}"))?;
            observation.net_working_capital_change = final_nwc
                .checked_sub(initial_nwc)
                .ok_or_else(|| format!("working-capital change overflow for {actor}"))?;
        }

        Ok(observations)
    }

    /// Liquidity change attributed to operating activity after removing
    /// explicitly classified investing, financing, and other transfers.
    ///
    /// This is an exact model reconciliation, not a claim of IFRS
    /// presentation compliance; classification policy remains model-defined.
    pub fn try_operating_liquidity_change(&self) -> Result<i128, String> {
        self.net_liquidity_change
            .checked_sub(self.investing_net_liquidity()?)
            .and_then(|value| value.checked_sub(self.financing_net_liquidity()?))
            .and_then(|value| value.checked_sub(self.other_liquidity_change))
            .ok_or_else(|| "actor operating liquidity overflow".into())
    }

    pub fn operating_liquidity_change(&self) -> i128 {
        self.try_operating_liquidity_change()
            .expect("actor operating liquidity overflow")
    }

    /// Verify that every observed liquidity change is classified exactly once.
    /// Verify the period opening/closing net working-capital identity.
    pub fn net_working_capital_stock_flow_reconciliation_holds(&self) -> bool {
        self.opening_net_working_capital
            .checked_add(self.net_working_capital_change)
            == Some(self.net_working_capital())
    }

    /// Verify the period opening/closing liquidity stock-flow identity.
    pub fn liquidity_stock_flow_reconciliation_holds(&self) -> bool {
        self.opening_liquidity
            .checked_add(self.net_liquidity_change)
            == Some(self.liquidity)
    }

    pub fn liquidity_flow_reconciliation_holds(&self) -> bool {
        self.try_operating_liquidity_change()
            .ok()
            .and_then(|operating| operating.checked_add(self.investing_net_liquidity().ok()?))
            .and_then(|value| value.checked_add(self.financing_net_liquidity().ok()?))
            .and_then(|value| value.checked_add(self.other_liquidity_change))
            == Some(self.net_liquidity_change)
    }

    pub fn try_gross_debt_service(&self) -> Result<i128, String> {
        self.interest_paid
            .checked_add(self.debt_repaid)
            .ok_or_else(|| "actor debt-service overflow".into())
    }

    pub fn gross_debt_service(&self) -> i128 {
        self.try_gross_debt_service()
            .expect("actor debt-service overflow")
    }

    pub fn try_gross_surplus(&self) -> Result<i128, String> {
        self.sales_revenue
            .checked_sub(self.cost_of_goods_sold)
            .ok_or_else(|| "actor surplus overflow".into())
    }

    pub fn gross_surplus(&self) -> i128 {
        self.try_gross_surplus()
            .expect("actor surplus overflow")
    }

    pub fn try_operating_surplus_after_depreciation(&self) -> Result<i128, String> {
        self.try_gross_surplus()?
            .checked_sub(self.depreciation)
            .ok_or_else(|| "actor operating-surplus overflow".into())
    }

    pub fn operating_surplus_after_depreciation(&self) -> i128 {
        self.try_operating_surplus_after_depreciation()
            .expect("actor operating-surplus overflow")
    }

    pub fn try_financing_net_liquidity(&self) -> Result<i128, String> {
        self.credit_received
            .checked_sub(self.debt_repaid)
            .ok_or_else(|| "actor financing liquidity overflow".into())
    }

    pub fn financing_net_liquidity(&self) -> i128 {
        self.try_financing_net_liquidity()
            .expect("actor financing liquidity overflow")
    }

    pub fn try_investing_net_liquidity(&self) -> Result<i128, String> {
        self.investment_received
            .checked_sub(self.investment_paid)
            .ok_or_else(|| "actor investing liquidity overflow".into())
    }

    pub fn investing_net_liquidity(&self) -> i128 {
        self.try_investing_net_liquidity()
            .expect("actor investing liquidity overflow")
    }

    /// Monetary operating working capital excludes cash and deposits.
    pub fn try_net_working_capital(&self) -> Result<i128, String> {
        self.inventory_carrying_value
            .checked_add(self.trade_receivables)
            .and_then(|value| value.checked_sub(self.trade_payables))
            .ok_or_else(|| "actor working-capital overflow".into())
    }

    pub fn net_working_capital(&self) -> i128 {
        self.try_net_working_capital()
            .expect("actor working-capital overflow")
    }

    /// Liquidity change not explained by explicitly classified financing or
    /// investment transitions. This is a residual, not a claim that every
    /// remaining flow is operating cash flow.
    pub fn try_non_financing_liquidity_change(&self) -> Result<i128, String> {
        self.net_liquidity_change
            .checked_sub(self.financing_net_liquidity()?)
            .and_then(|value| value.checked_sub(self.investing_net_liquidity()?))
            .ok_or_else(|| "actor non-financing liquidity overflow".into())
    }

    pub fn non_financing_liquidity_change(&self) -> i128 {
        self.try_non_financing_liquidity_change()
            .expect("actor non-financing liquidity overflow")
    }

    pub fn leverage(&self) -> Option<RatioObservation> {
        let assets = self
            .cash
            .checked_add(self.deposits)?
            .checked_add(self.loan_claims)?
            .checked_add(self.trade_receivables)?;
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
    let mut ids = match transition {
        EconomicTransition::MonetaryTransfer(flow) => vec![flow.from.clone(), flow.to.clone()],
        EconomicTransition::IncomeTransfer(flow) => vec![flow.payer.clone(), flow.recipient.clone()],
        EconomicTransition::CapitalInvestment(investment) => {
            vec![investment.buyer.clone(), investment.producer.clone()]
        }
        EconomicTransition::Production(event) => vec![event.producer.clone()],
        EconomicTransition::InventoryTransfer(transfer) => {
            vec![transfer.from.clone(), transfer.to.clone()]
        }
        EconomicTransition::InventoryConsumption(consumption) => {
            vec![consumption.consumer.clone()]
        }
        EconomicTransition::GoodsSale(sale) => vec![sale.seller.clone(), sale.buyer.clone()],
        EconomicTransition::TradeCreditSale(sale) => {
            vec![sale.seller.clone(), sale.buyer.clone()]
        }
        EconomicTransition::TradeCreditSettlement(settlement) => {
            vec![settlement.seller.clone(), settlement.buyer.clone()]
        }
        EconomicTransition::InventoryCostAddition(addition) => vec![addition.actor.clone()],
        EconomicTransition::InventoryCostRelief(relief) => vec![relief.actor.clone()],
        EconomicTransition::Depreciation(depreciation) => vec![depreciation.actor.clone()],
        EconomicTransition::CreditCreation(credit) => {
            vec![credit.lender.clone(), credit.borrower.clone()]
        }
        EconomicTransition::DebtRepayment(repayment) => {
            vec![repayment.lender.clone(), repayment.borrower.clone()]
        }
    };
    ids.dedup();
    ids
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
        EconomicTransition::TradeCreditSale(sale) => state.apply_trade_credit_sale(sale),
        EconomicTransition::TradeCreditSettlement(settlement) => {
            state.apply_trade_credit_settlement(settlement)
        }
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
        assert!(firm.liquidity_stock_flow_reconciliation_holds());
        assert!(firm.net_working_capital_stock_flow_reconciliation_holds());
        assert_eq!(firm.operating_liquidity_change(), 30);
        assert_eq!(firm.financing_net_liquidity(), 180);
        assert_eq!(firm.investing_net_liquidity(), 0);
        assert_eq!(firm.other_liquidity_change, 0);
        assert!(firm.liquidity_flow_reconciliation_holds());
        assert_eq!(firm.financing_net_liquidity(), 180);
        assert_eq!(firm.investing_net_liquidity(), 0);
    }

    #[test]
    fn actor_observation_construction_fails_closed_on_opening_working_capital_overflow() {
        let mut actor = ActorBalanceSheet::new("firm");
        actor.inventory_carrying_value = i128::MAX;
        actor.monetary.trade_receivables = 1;
        let state = EconomicState::new(vec![actor]);

        assert!(ActorEconomicObservables::from_state_and_transitions(&state, &[]).is_err());
    }

    #[test]
    fn checked_actor_derived_totals_fail_closed_on_overflow() {
        let debt_service = ActorEconomicObservables {
            interest_paid: i128::MAX,
            debt_repaid: 1,
            ..ActorEconomicObservables::default()
        };
        assert!(debt_service.try_gross_debt_service().is_err());

        let surplus = ActorEconomicObservables {
            sales_revenue: i128::MIN,
            cost_of_goods_sold: 1,
            ..ActorEconomicObservables::default()
        };
        assert!(surplus.try_gross_surplus().is_err());

        let after_depreciation = ActorEconomicObservables {
            sales_revenue: i128::MIN,
            depreciation: 1,
            ..ActorEconomicObservables::default()
        };
        assert!(after_depreciation
            .try_operating_surplus_after_depreciation()
            .is_err());

        let working_capital = ActorEconomicObservables {
            inventory_carrying_value: i128::MAX,
            trade_receivables: 1,
            ..ActorEconomicObservables::default()
        };
        assert!(working_capital.try_net_working_capital().is_err());

        let non_financing = ActorEconomicObservables {
            net_liquidity_change: i128::MIN,
            credit_received: 1,
            ..ActorEconomicObservables::default()
        };
        assert!(non_financing.try_non_financing_liquidity_change().is_err());
    }

    #[test]
    fn financing_regime_uses_principal_due_not_actual_repayment() {
        let observables = ActorEconomicObservables::default();
        assert_eq!(
            observables.financing_regime(100, 50, 100).unwrap(),
            FinancingRegime::Speculative
        );
    }

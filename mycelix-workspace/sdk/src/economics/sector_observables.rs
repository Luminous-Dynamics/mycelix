// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Sector-level derived liquidity observations.
//!
//! This module is a deterministic projection of actor observations into
//! Godley-style sector aggregates. It deliberately does not become a second
//! transaction ledger and does not introduce behavioral equations.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::actor_observables::ActorEconomicObservables;
use super::sector_balance::{BalanceSheetInstrument, SectorAssignment, SectorBalanceSheet};
use super::sector_flow::EconomicSector;
use super::stock_flow::{ActorId, EconomicState};

fn add_checked(slot: &mut i128, amount: i128, label: &str) -> Result<(), String> {
    *slot = slot
        .checked_add(amount)
        .ok_or_else(|| format!("{label} overflow"))?;
    Ok(())
}

/// Sector-level derived liquidity and cash-flow observations for one period.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SectorEconomicObservables {
    pub sector: EconomicSector,
    pub opening_liquidity: i128,
    pub closing_liquidity: i128,
    pub net_liquidity_change: i128,
    pub operating_liquidity_change: i128,
    pub investing_net_liquidity: i128,
    pub financing_net_liquidity: i128,
    pub other_liquidity_change: i128,

    pub opening_net_working_capital: i128,
    pub net_working_capital: i128,
    pub net_working_capital_change: i128,
    /// Closing-minus-opening inventory carrying-value change.
    #[serde(default)]
    pub inventory_carrying_value_change: i128,
    /// Closing-minus-opening trade-receivables change.
    #[serde(default)]
    pub trade_receivables_change: i128,
    /// Closing-minus-opening trade-payables change.
    #[serde(default)]
    pub trade_payables_change: i128,

    pub loan_claims: i128,
    pub debt: i128,
    pub trade_receivables: i128,
    pub trade_payables: i128,
    pub inventory_carrying_value: i128,
    pub productive_capital: i128,
    pub net_financial_position: i128,
    pub net_worth: i128,

    pub credit_received: i128,
    pub credit_originated: i128,
    pub debt_repaid: i128,
    /// Debt claims extinguished by write-off in this sector as lender.
    #[serde(default)]
    pub debt_written_off_as_lender: i128,
    /// Debt liabilities extinguished by write-off in this sector as borrower.
    #[serde(default)]
    pub debt_written_off_as_borrower: i128,

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
    /// Signed non-cash revaluation of monetary-valued real assets during the period.
    #[serde(default)]
    pub real_asset_revaluation: i128,
}

impl Default for SectorEconomicObservables {
    fn default() -> Self {
        Self {
            sector: EconomicSector::Household,
            opening_liquidity: 0,
            closing_liquidity: 0,
            net_liquidity_change: 0,
            operating_liquidity_change: 0,
            investing_net_liquidity: 0,
            financing_net_liquidity: 0,
            other_liquidity_change: 0,
            opening_net_working_capital: 0,
            net_working_capital: 0,
            net_working_capital_change: 0,
            inventory_carrying_value_change: 0,
            trade_receivables_change: 0,
            trade_payables_change: 0,
            loan_claims: 0,
            debt: 0,
            trade_receivables: 0,
            trade_payables: 0,
            inventory_carrying_value: 0,
            productive_capital: 0,
            net_financial_position: 0,
            net_worth: 0,
            credit_received: 0,
            credit_originated: 0,
            debt_repaid: 0,
            debt_written_off_as_lender: 0,
            debt_written_off_as_borrower: 0,
            trade_credit_received: 0,
            trade_credit_extended: 0,
            trade_credit_settled: 0,
            trade_credit_collected: 0,
            interest_paid: 0,
            interest_received: 0,
            wages_paid: 0,
            wages_received: 0,
            taxes_paid: 0,
            transfers_paid: 0,
            transfers_received: 0,
            consumption_paid: 0,
            investment_paid: 0,
            investment_received: 0,
            sales_revenue: 0,
            goods_purchases: 0,
            cost_of_goods_sold: 0,
            depreciation: 0,
            real_asset_revaluation: 0,
        }
    }
}

impl SectorEconomicObservables {
    /// Derive actor observations from a starting state and immediately
    /// consolidate them into sectors. This keeps the transition replay
    /// authoritative and avoids a second actor-level ledger.
    pub fn from_state_and_transitions(
        state: &EconomicState,
        transitions: &[super::transition::EconomicTransition],
        assignments: &[SectorAssignment],
    ) -> Result<BTreeMap<EconomicSector, Self>, String> {
        let observations =
            ActorEconomicObservables::from_state_and_transitions(state, transitions)?;
        let sectors = Self::from_actor_observations(&observations, assignments)?;
        let (post_state, _) = super::transition::apply_step(state, 0, transitions, None)
            .map_err(|error| format!("sector post-state replay failed: {error}"))?;
        let balance_sheet = SectorBalanceSheet::from_state(&post_state, assignments)?;
        for observation in sectors.values() {
            observation.validate_against_balance_sheet(&balance_sheet)?;
        }
        Ok(sectors)
    }

    /// Aggregate actor observations into sectors using exactly one assignment
    /// per actor. The result is deterministic because both inputs are keyed
    /// and iterated canonically.
    pub fn from_actor_observations(
        observations: &BTreeMap<ActorId, ActorEconomicObservables>,
        assignments: &[SectorAssignment],
    ) -> Result<BTreeMap<EconomicSector, Self>, String> {
        let mut assignment_map = BTreeMap::new();
        for assignment in assignments {
            if !observations.contains_key(&assignment.actor) {
                return Err(format!(
                    "sector assignment references unknown actor: {}",
                    assignment.actor
                ));
            }
            if assignment_map
                .insert(assignment.actor.clone(), assignment.sector)
                .is_some()
            {
                return Err(format!(
                    "actor {} must have exactly one sector assignment",
                    assignment.actor
                ));
            }
        }

        if assignment_map.len() != observations.len() {
            return Err("sector assignments must cover each actor exactly once".into());
        }

        let mut sectors = BTreeMap::new();
        for (actor_id, observation) in observations {
            if &observation.actor != actor_id {
                return Err(format!(
                    "actor observation key mismatch for {}",
                    actor_id
                ));
            }

            let sector = *assignment_map
                .get(actor_id)
                .ok_or_else(|| format!("missing sector assignment for actor: {actor_id}"))?;

            let expected_liquidity = observation
                .cash
                .checked_add(observation.deposits)
                .ok_or_else(|| format!("actor liquidity overflow for {actor_id}"))?;
            if expected_liquidity != observation.liquidity {
                return Err(format!(
                    "actor liquidity snapshot does not match cash + deposits for {actor_id}"
                ));
            }

            let sector_observation = sectors.entry(sector).or_insert_with(|| Self {
                sector,
                ..Self::default()
            });

            if !observation.liquidity_stock_flow_reconciliation_holds() {
                return Err(format!(
                    "actor opening/closing liquidity does not reconcile for {actor_id}"
                ));
            }
            if !observation.net_working_capital_stock_flow_reconciliation_holds() {
                return Err(format!(
                    "actor opening/closing working capital does not reconcile for {actor_id}"
                ));
            }

            add_checked(
                &mut sector_observation.opening_liquidity,
                observation.opening_liquidity,
                "sector opening liquidity",
            )?;
            add_checked(
                &mut sector_observation.closing_liquidity,
                observation.liquidity,
                "sector closing liquidity",
            )?;
            add_checked(
                &mut sector_observation.net_liquidity_change,
                observation.net_liquidity_change,
                "sector net liquidity change",
            )?;
            add_checked(
                &mut sector_observation.opening_net_working_capital,
                observation.opening_net_working_capital,
                "sector opening net working capital",
            )?;
            add_checked(
                &mut sector_observation.net_working_capital,
                observation.net_working_capital(),
                "sector net working capital",
            )?;
            add_checked(
                &mut sector_observation.inventory_carrying_value_change,
                observation.inventory_carrying_value_change,
                "sector inventory carrying-value change",
            )?;
            add_checked(
                &mut sector_observation.trade_receivables_change,
                observation.trade_receivables_change,
                "sector trade-receivables change",
            )?;
            add_checked(
                &mut sector_observation.trade_payables_change,
                observation.trade_payables_change,
                "sector trade-payables change",
            )?;

            for (slot, value, label) in [
                (
                    &mut sector_observation.loan_claims,
                    observation.loan_claims,
                    "sector loan claims",
                ),
                (
                    &mut sector_observation.debt,
                    observation.debt,
                    "sector debt",
                ),
                (
                    &mut sector_observation.trade_receivables,
                    observation.trade_receivables,
                    "sector trade receivables",
                ),
                (
                    &mut sector_observation.trade_payables,
                    observation.trade_payables,
                    "sector trade payables",
                ),
                (
                    &mut sector_observation.inventory_carrying_value,
                    observation.inventory_carrying_value,
                    "sector inventory carrying value",
                ),
                (
                    &mut sector_observation.productive_capital,
                    observation.productive_capital,
                    "sector productive capital",
                ),
                (
                    &mut sector_observation.net_financial_position,
                    observation.net_financial_position,
                    "sector net financial position",
                ),
                (
                    &mut sector_observation.net_worth,
                    observation.net_worth,
                    "sector net worth",
                ),
                (
                    &mut sector_observation.real_asset_revaluation,
                    observation.real_asset_revaluation,
                    "sector real-asset revaluation",
                ),
            ] {
                add_checked(slot, value, label)?;
            }

            add_checked(
                &mut sector_observation.operating_liquidity_change,
                observation.try_operating_liquidity_change()?,
                "sector operating liquidity change",
            )?;
            add_checked(
                &mut sector_observation.investing_net_liquidity,
                observation.try_investing_net_liquidity()?,
                "sector investing liquidity",
            )?;
            add_checked(
                &mut sector_observation.financing_net_liquidity,
                observation.try_financing_net_liquidity()?,
                "sector financing liquidity",
            )?;
            add_checked(
                &mut sector_observation.other_liquidity_change,
                observation.other_liquidity_change,
                "sector other liquidity change",
            )?;

            for (slot, value, label) in [
                (
                    &mut sector_observation.net_working_capital_change,
                    observation.net_working_capital_change,
                    "sector working-capital change",
                ),
                (
                    &mut sector_observation.credit_received,
                    observation.credit_received,
                    "sector credit received",
                ),
                (
                    &mut sector_observation.credit_originated,
                    observation.credit_originated,
                    "sector credit originated",
                ),
                (
                    &mut sector_observation.debt_repaid,
                    observation.debt_repaid,
                    "sector debt repaid",
                ),
                (
                    &mut sector_observation.debt_written_off_as_lender,
                    observation.debt_written_off_as_lender,
                    "sector debt written off as lender",
                ),
                (
                    &mut sector_observation.debt_written_off_as_borrower,
                    observation.debt_written_off_as_borrower,
                    "sector debt written off as borrower",
                ),
                (
                    &mut sector_observation.trade_credit_received,
                    observation.trade_credit_received,
                    "sector trade credit received",
                ),
                (
                    &mut sector_observation.trade_credit_extended,
                    observation.trade_credit_extended,
                    "sector trade credit extended",
                ),
                (
                    &mut sector_observation.trade_credit_settled,
                    observation.trade_credit_settled,
                    "sector trade credit settled",
                ),
                (
                    &mut sector_observation.trade_credit_collected,
                    observation.trade_credit_collected,
                    "sector trade credit collected",
                ),
                (
                    &mut sector_observation.interest_paid,
                    observation.interest_paid,
                    "sector interest paid",
                ),
                (
                    &mut sector_observation.interest_received,
                    observation.interest_received,
                    "sector interest received",
                ),
                (
                    &mut sector_observation.wages_paid,
                    observation.wages_paid,
                    "sector wages paid",
                ),
                (
                    &mut sector_observation.wages_received,
                    observation.wages_received,
                    "sector wages received",
                ),
                (
                    &mut sector_observation.taxes_paid,
                    observation.taxes_paid,
                    "sector taxes paid",
                ),
                (
                    &mut sector_observation.transfers_paid,
                    observation.transfers_paid,
                    "sector transfers paid",
                ),
                (
                    &mut sector_observation.transfers_received,
                    observation.transfers_received,
                    "sector transfers received",
                ),
                (
                    &mut sector_observation.consumption_paid,
                    observation.consumption_paid,
                    "sector consumption paid",
                ),
                (
                    &mut sector_observation.investment_paid,
                    observation.investment_paid,
                    "sector investment paid",
                ),
                (
                    &mut sector_observation.investment_received,
                    observation.investment_received,
                    "sector investment received",
                ),
                (
                    &mut sector_observation.sales_revenue,
                    observation.sales_revenue,
                    "sector sales revenue",
                ),
                (
                    &mut sector_observation.goods_purchases,
                    observation.goods_purchases,
                    "sector goods purchases",
                ),
                (
                    &mut sector_observation.cost_of_goods_sold,
                    observation.cost_of_goods_sold,
                    "sector COGS",
                ),
                (
                    &mut sector_observation.depreciation,
                    observation.depreciation,
                    "sector depreciation",
                ),
            ] {
                add_checked(slot, value, label)?;
            }
        }

        for observation in sectors.values() {
            if !observation.liquidity_flow_reconciliation_holds() {
                return Err(format!(
                    "sector liquidity decomposition does not reconcile for {:?}",
                    observation.sector
                ));
            }
            if !observation.working_capital_component_reconciliation_holds() {
                return Err(format!(
                    "sector working-capital components do not reconcile for {:?}",
                    observation.sector
                ));
            }
            let expected_closing = observation
                .opening_liquidity
                .checked_add(observation.net_liquidity_change)
                .ok_or_else(|| {
                    format!("sector closing liquidity overflow for {:?}", observation.sector)
                })?;
            let expected_nwc = observation
                .opening_net_working_capital
                .checked_add(observation.net_working_capital_change)
                .ok_or_else(|| {
                    format!("sector working-capital closure overflow for {:?}", observation.sector)
                })?;
            if expected_nwc != observation.net_working_capital {
                return Err(format!(
                    "sector opening/closing working capital does not reconcile for {:?}",
                    observation.sector
                ));
            }
            if expected_closing != observation.closing_liquidity {
                return Err(format!(
                    "sector opening/closing liquidity does not reconcile for {:?}",
                    observation.sector
                ));
            }
        }

        Ok(sectors)
    }

    /// Re-derive this sector observation from the supplied state, transitions, and assignments.
    ///
    /// The replayed sector projection must match this serialized observation exactly.
    pub fn verify_against(
        &self,
        state: &EconomicState,
        transitions: &[super::transition::EconomicTransition],
        assignments: &[SectorAssignment],
    ) -> Result<(), String> {
        let expected = Self::from_state_and_transitions(state, transitions, assignments)?;
        let expected_observation = expected
            .get(&self.sector)
            .ok_or_else(|| format!("sector observation {:?} is not present in replayed state", self.sector))?;
        if self != expected_observation {
            return Err(format!(
                "sector observation does not match transition replay for {:?}",
                self.sector
            ));
        }
        Ok(())
    }

    /// Re-derive the complete sector observation map and require exact sector coverage.
    ///
    /// This rejects missing or unexpected sectors as well as tampered observation fields.
    pub fn verify_map_against(
        observations: &BTreeMap<EconomicSector, SectorEconomicObservables>,
        state: &EconomicState,
        transitions: &[super::transition::EconomicTransition],
        assignments: &[SectorAssignment],
    ) -> Result<(), String> {
        let expected = Self::from_state_and_transitions(state, transitions, assignments)?;
        if observations != &expected {
            return Err("sector observation map does not match transition-derived projection".into());
        }
        Ok(())
    }

    /// Closing monetary net working capital.
    pub fn net_working_capital(&self) -> i128 {
        self.inventory_carrying_value
            .checked_add(self.trade_receivables)
            .and_then(|value| value.checked_sub(self.trade_payables))
            .expect("sector working-capital overflow")
    }

    /// Verify that net working-capital change decomposes exactly into its
    /// inventory carrying-value, receivable, and payable components.
    pub fn working_capital_component_reconciliation_holds(&self) -> bool {
        self.inventory_carrying_value_change
            .checked_add(self.trade_receivables_change)
            .and_then(|value| value.checked_sub(self.trade_payables_change))
            == Some(self.net_working_capital_change)
    }

    /// Validate the sector stock snapshot against the consolidated
    /// balance-sheet projection for the same sector.
    pub fn validate_against_balance_sheet(
        &self,
        balance_sheet: &SectorBalanceSheet,
    ) -> Result<(), String> {
        let expected = [
            (
                BalanceSheetInstrument::Loans,
                self.loan_claims,
                1,
                "loan claims",
            ),
            (
                BalanceSheetInstrument::Debt,
                self.debt,
                -1,
                "debt",
            ),
            (
                BalanceSheetInstrument::TradeReceivables,
                self.trade_receivables,
                1,
                "trade receivables",
            ),
            (
                BalanceSheetInstrument::TradePayables,
                self.trade_payables,
                -1,
                "trade payables",
            ),
            (
                BalanceSheetInstrument::InventoryCarryingValue,
                self.inventory_carrying_value,
                1,
                "inventory carrying value",
            ),
            (
                BalanceSheetInstrument::ProductiveCapital,
                self.productive_capital,
                1,
                "productive capital",
            ),
        ];

        for (instrument, observed, sign, label) in expected {
            let actual = balance_sheet.sector_instrument_total_checked(self.sector, instrument)?;
            let expected = if sign == 1 {
                observed
            } else {
                observed
                    .checked_neg()
                    .ok_or_else(|| format!("sector {label} sign overflow"))?
            };
            if actual != expected {
                return Err(format!(
                    "sector {label} does not match balance-sheet snapshot for {:?}",
                    self.sector
                ));
            }
        }

        let nfp = balance_sheet
            .sector_instrument_total_checked(self.sector, BalanceSheetInstrument::Cash)?
            .checked_add(balance_sheet.sector_instrument_total_checked(
                self.sector,
                BalanceSheetInstrument::Deposits,
            )?)
            .ok_or_else(|| format!("sector net-financial-position overflow for {:?}", self.sector))?
            .checked_add(balance_sheet.sector_instrument_total_checked(
                self.sector,
                BalanceSheetInstrument::Loans,
            )?)
            .ok_or_else(|| format!("sector net-financial-position overflow for {:?}", self.sector))?
            .checked_add(balance_sheet.sector_instrument_total_checked(
                self.sector,
                BalanceSheetInstrument::TradeReceivables,
            )?)
            .ok_or_else(|| format!("sector net-financial-position overflow for {:?}", self.sector))?
            .checked_add(balance_sheet.sector_instrument_total_checked(
                self.sector,
                BalanceSheetInstrument::Debt,
            )?)
            .ok_or_else(|| format!("sector net-financial-position overflow for {:?}", self.sector))?
            .checked_add(balance_sheet.sector_instrument_total_checked(
                self.sector,
                BalanceSheetInstrument::DepositLiabilities,
            )?)
            .ok_or_else(|| format!("sector net-financial-position overflow for {:?}", self.sector))?
            .checked_add(balance_sheet.sector_instrument_total_checked(
                self.sector,
                BalanceSheetInstrument::TradePayables,
            )?)
            .ok_or_else(|| format!("sector net-financial-position overflow for {:?}", self.sector))?;
        if nfp != self.net_financial_position {
            return Err(format!(
                "sector net financial position does not match balance sheet for {:?}",
                self.sector
            ));
        }

        let net_worth = balance_sheet
            .sector_instrument_total_checked(self.sector, BalanceSheetInstrument::Equity)?
            .checked_neg()
            .ok_or_else(|| format!("sector net-worth sign overflow for {:?}", self.sector))?;
        if net_worth != self.net_worth {
            return Err(format!(
                "sector net worth does not match balance sheet for {:?}",
                self.sector
            ));
        }

        Ok(())
    }

    /// Verify the exact operating/investing/financing/other decomposition.
    pub fn liquidity_flow_reconciliation_holds(&self) -> bool {
        self.operating_liquidity_change
            .checked_add(self.investing_net_liquidity)
            .and_then(|value| value.checked_add(self.financing_net_liquidity))
            .and_then(|value| value.checked_add(self.other_liquidity_change))
            == Some(self.net_liquidity_change)
    }

    /// Verify the opening-stock plus flow identity.
    pub fn liquidity_stock_flow_reconciliation_holds(&self) -> bool {
        self.opening_liquidity
            .checked_add(self.net_liquidity_change)
            == Some(self.closing_liquidity)
    }

    pub fn gross_surplus(&self) -> i128 {
        self.sales_revenue
            .checked_sub(self.cost_of_goods_sold)
            .expect("sector surplus overflow")
    }

    pub fn operating_surplus_after_depreciation(&self) -> i128 {
        self.gross_surplus()
            .checked_sub(self.depreciation)
            .expect("sector operating-surplus overflow")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(
        actor: &str,
        opening_liquidity: i128,
        net_liquidity_change: i128,
        _operating: i128,
        investing: i128,
        financing: i128,
        other: i128,
    ) -> ActorEconomicObservables {
        ActorEconomicObservables {
            actor: actor.into(),
            opening_liquidity,
            liquidity: opening_liquidity
                .checked_add(net_liquidity_change)
                .unwrap(),
            net_liquidity_change,
            other_liquidity_change: other,
            investment_paid: if investing < 0 { -investing } else { 0 },
            investment_received: if investing > 0 { investing } else { 0 },
            credit_received: if financing > 0 { financing } else { 0 },
            debt_repaid: if financing < 0 { -financing } else { 0 },
            sales_revenue: operating.max(0),
            ..ActorEconomicObservables::default()
        }
    }


    #[test]
    fn sector_observation_map_verifier_rejects_extra_sector() {
        let state = EconomicState::new(vec![crate::economics::stock_flow::ActorBalanceSheet::new("household")]);
        let assignments = vec![SectorAssignment {
            actor: "household".into(),
            sector: EconomicSector::Household,
        }];
        let transitions = vec![];
        let mut observations =
            SectorEconomicObservables::from_state_and_transitions(
                &state,
                &transitions,
                &assignments,
            )
            .unwrap();
        let mut unexpected = observations[&EconomicSector::Household].clone();
        unexpected.sector = EconomicSector::Bank;
        observations.insert(EconomicSector::Bank, unexpected);

        assert!(SectorEconomicObservables::verify_map_against(
            &observations,
            &state,
            &transitions,
            &assignments,
        )
        .is_err());
    }

    #[test]
    fn sector_projection_reconciles_and_is_deterministic() {
        let mut observations = BTreeMap::new();
        let mut firm = observation("firm-a", 100, 30, 20, -10, 20, 0);
        firm.net_working_capital_change = 15;
        firm.opening_net_working_capital = -15;
        firm.trade_credit_extended = 40;
        firm.sales_revenue = 80;
        firm.cost_of_goods_sold = 50;
        firm.wages_paid = 12;
        firm.interest_paid = 3;
        firm.taxes_paid = 4;
        observations.insert("firm-a".into(), firm);

        let mut household = observation("household-a", 50, -20, -20, 0, 0, 0);
        household.goods_purchases = 80;
        observations.insert("household-a".into(), household);

        let assignments = vec![
            SectorAssignment {
                actor: "firm-a".into(),
                sector: EconomicSector::Firm,
            },
            SectorAssignment {
                actor: "household-a".into(),
                sector: EconomicSector::Household,
            },
        ];

        let first =
            SectorEconomicObservables::from_actor_observations(&observations, &assignments).unwrap();
        let second =
            SectorEconomicObservables::from_actor_observations(&observations, &assignments).unwrap();

        assert_eq!(first, second);

        let firm = &first[&EconomicSector::Firm];
        assert_eq!(firm.opening_liquidity, 100);
        assert_eq!(firm.closing_liquidity, 130);
        assert_eq!(firm.net_liquidity_change, 30);
        assert_eq!(firm.operating_liquidity_change, 20);
        assert_eq!(firm.investing_net_liquidity, -10);
        assert_eq!(firm.financing_net_liquidity, 20);
        assert_eq!(firm.trade_credit_extended, 40);
        assert_eq!(firm.net_working_capital_change, 15);
        assert_eq!(firm.inventory_carrying_value_change, 0);
        assert_eq!(firm.trade_receivables_change, 15);
        assert_eq!(firm.trade_payables_change, 0);
        assert!(firm.working_capital_component_reconciliation_holds());
        assert_eq!(firm.net_working_capital(), 0);
        assert_eq!(firm.opening_net_working_capital, -15);
        assert!(firm.net_working_capital_stock_flow_reconciliation_holds());
        assert_eq!(firm.wages_paid, 12);
        assert_eq!(firm.interest_paid, 3);
        assert_eq!(firm.taxes_paid, 4);
        assert_eq!(firm.gross_surplus(), 30);
        assert_eq!(firm.operating_surplus_after_depreciation(), 30);
        assert!(firm.liquidity_flow_reconciliation_holds());
        assert!(firm.liquidity_stock_flow_reconciliation_holds());
    }

    #[test]
    fn sector_observation_verify_against_rejects_tampering() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("firm-a"),
            ActorBalanceSheet::new("household-a"),
        ]);
        let assignments = vec![
            SectorAssignment { actor: "firm-a".into(), sector: EconomicSector::Firm },
            SectorAssignment { actor: "household-a".into(), sector: EconomicSector::Household },
        ];
        let sectors = SectorEconomicObservables::from_state_and_transitions(&state, &[], &assignments).unwrap();
        let mut observation = sectors.get(&EconomicSector::Firm).unwrap().clone();
        observation.closing_liquidity = 1;
        assert!(observation.verify_against(&state, &[], &assignments).is_err());
    }

    #[test]
    fn state_to_sector_projection_validates_terminal_stocks() {
        let mut firm = ActorBalanceSheet::new("firm-a");
        firm.monetary.deposits = 100;
        firm.real.inventories = 2;
        let mut household = ActorBalanceSheet::new("household-a");
        household.monetary.deposits = 50;
        let state = EconomicState::new(vec![firm, household]);
        let assignments = vec![
            SectorAssignment {
                actor: "firm-a".into(),
                sector: EconomicSector::Firm,
            },
            SectorAssignment {
                actor: "household-a".into(),
                sector: EconomicSector::Household,
            },
        ];
        let transitions = vec![EconomicTransition::TradeCreditSale(
            crate::economics::stock_flow::TradeCreditSale::new(
                "firm-a",
                "household-a",
                1,
                25,
            )
            .unwrap(),
        )];

        let sectors =
            SectorEconomicObservables::from_state_and_transitions(
                &state,
                &transitions,
                &assignments,
            )
            .unwrap();
        let firm = &sectors[&EconomicSector::Firm];
        assert_eq!(firm.trade_receivables, 25);
        assert_eq!(firm.inventory_carrying_value_change, 0);
        assert_eq!(firm.trade_receivables_change, 25);
        assert_eq!(firm.trade_payables_change, 0);
        assert_eq!(firm.net_working_capital_change, 25);
        assert!(firm.working_capital_component_reconciliation_holds());
        assert_eq!(firm.net_working_capital(), 25);
    }

    #[test]
    fn state_to_sector_projection_matches_balance_sheet_stocks() {
        let mut firm = crate::economics::stock_flow::ActorBalanceSheet::new("firm-a");
        firm.monetary.cash = 10;
        firm.monetary.deposits = 20;
        firm.monetary.claims = 30;
        firm.monetary.liabilities = 5;
        firm.monetary.trade_receivables = 8;
        firm.monetary.trade_payables = 3;
        firm.inventory_carrying_value = 40;
        firm.real.productive_capital = 100;

        let state = EconomicState::new(vec![firm]);
        let assignments = vec![SectorAssignment {
            actor: "firm-a".into(),
            sector: EconomicSector::Firm,
        }];
        let actors =
            ActorEconomicObservables::from_state_and_transitions(&state, &[]).unwrap();
        let sectors = SectorEconomicObservables::from_actor_observations(&actors, &assignments)
            .unwrap();
        let sheet = SectorBalanceSheet::from_state(&state, &assignments).unwrap();
        sectors[&EconomicSector::Firm]
            .validate_against_balance_sheet(&sheet)
            .unwrap();
        assert_eq!(sectors[&EconomicSector::Firm].loan_claims, 30);
        assert_eq!(sectors[&EconomicSector::Firm].debt, 5);
        assert_eq!(sectors[&EconomicSector::Firm].trade_receivables, 8);
        assert_eq!(sectors[&EconomicSector::Firm].trade_payables, 3);
        assert_eq!(sectors[&EconomicSector::Firm].inventory_carrying_value, 40);
        assert_eq!(sectors[&EconomicSector::Firm].productive_capital, 100);
    }

    #[test]
    fn sector_projection_rejects_inconsistent_actor_liquidity() {
        let mut observations = BTreeMap::new();
        observations.insert(
            "firm-a".into(),
            ActorEconomicObservables {
                actor: "firm-a".into(),
                cash: 10,
                deposits: 5,
                liquidity: 99,
                ..ActorEconomicObservables::default()
            },
        );
        let assignments = vec![SectorAssignment {
            actor: "firm-a".into(),
            sector: EconomicSector::Firm,
        }];
        assert!(SectorEconomicObservables::from_actor_observations(
            &observations,
            &assignments
        )
        .is_err());
    }

    #[test]
    fn sector_projection_rejects_duplicate_or_missing_assignments() {
        let mut observations = BTreeMap::new();
        observations.insert(
            "firm-a".into(),
            ActorEconomicObservables {
                actor: "firm-a".into(),
                opening_liquidity: 10,
                liquidity: 10,
                ..ActorEconomicObservables::default()
            },
        );

        let duplicate = vec![
            SectorAssignment {
                actor: "firm-a".into(),
                sector: EconomicSector::Firm,
            },
            SectorAssignment {
                actor: "firm-a".into(),
                sector: EconomicSector::Household,
            },
        ];
        assert!(SectorEconomicObservables::from_actor_observations(
            &observations,
            &duplicate
        )
        .is_err());

        let missing = vec![SectorAssignment {
            actor: "other".into(),
            sector: EconomicSector::Firm,
        }];
        assert!(SectorEconomicObservables::from_actor_observations(
            &observations,
            &missing
        )
        .is_err());
    }
}

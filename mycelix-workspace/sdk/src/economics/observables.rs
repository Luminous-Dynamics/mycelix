// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Derived economic observations.
//!
//! These are measurements projected from reconciled state/period data. They do
//! not encode a preferred policy outcome and do not mutate the accounting
//! substrate.

use serde::{Deserialize, Serialize};

use super::period_ledger::EconomicPeriodLedger;
use super::stock_flow::{EconomicFlowCategory, EconomicState};
use super::transition::{EconomicStepError, EconomicTransition};

/// Exact non-floating-point ratio observation.
///
/// Keeping numerator and denominator explicit avoids introducing rounding into
/// evidence-bound accounting diagnostics. A zero denominator means the ratio
/// is not defined and should be represented as None by callers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RatioObservation {
    pub numerator: i128,
    pub denominator: i128,
}

/// Aggregate financial/operating observations for one reconciled period.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicObservables {
    pub aggregate_cash: i128,
    pub aggregate_deposits: i128,
    pub aggregate_loans: i128,
    pub aggregate_debt: i128,
    pub aggregate_deposit_liabilities: i128,
    pub aggregate_trade_receivables: i128,
    pub aggregate_trade_payables: i128,
    pub net_working_capital: i128,
    pub liquidity: i128,
    pub gross_leverage: Option<RatioObservation>,
    pub credit_created: i128,
    pub debt_repaid: i128,
    /// Gross debt extinguished through explicit non-cash write-off.
    #[serde(default)]
    pub debt_written_off: i128,
    pub trade_credit_extended: i128,
    pub trade_credit_settled: i128,
    pub net_credit_impulse: i128,
    pub interest_paid: i128,
    pub gross_debt_service: i128,
    pub sales_consideration: i128,
    pub cost_of_goods_sold: i128,
    pub gross_operating_surplus: i128,
    pub depreciation: i128,
    /// Signed non-cash revaluation of monetary-valued real assets during the period.
    #[serde(default)]
    pub real_asset_revaluation: i128,
    pub operating_surplus_after_depreciation: i128,
}

impl EconomicObservables {
    /// Derive aggregate observations directly from a validated opening
    /// state and the authoritative transition sequence.
    ///
    /// This is the preferred trust-boundary constructor because the period
    /// ledger is derived inside the method rather than supplied independently.
    pub fn try_from_state_and_transitions(
        state: &EconomicState,
        transitions: &[EconomicTransition],
    ) -> Result<Self, EconomicStepError> {
        state
            .validate()
            .map_err(EconomicStepError::InvalidState)?;
        let (post_state, _) = super::transition::apply_step(state, 0, transitions, None)?;
        let ledger = EconomicPeriodLedger::from_transitions(transitions)?;
        Self::try_from_state_and_ledger(&post_state, &ledger)
            .map_err(EconomicStepError::Serialization)
    }

    /// Re-derive these aggregate observations from the supplied state and transitions.
    ///
    /// This is stronger than checking only the serialized observation hash: all derived
    /// values must be reproduced by the authoritative transition projection.
    pub fn verify_against_transitions(
        &self,
        state: &EconomicState,
        transitions: &[EconomicTransition],
    ) -> Result<(), EconomicStepError> {
        let expected = Self::try_from_state_and_transitions(state, transitions)?;
        if self != &expected {
            return Err(EconomicStepError::Serialization(
                "aggregate observations do not match transition-derived projection".into(),
            ));
        }
        Ok(())
    }

    /// Checked derivation of aggregate measurements.
    pub fn try_from_state_and_ledger(
        state: &EconomicState,
        ledger: &EconomicPeriodLedger,
    ) -> Result<Self, String> {
        state.validate()?;

        let mut aggregate_cash = 0i128;
        let mut aggregate_deposits = 0i128;
        let mut aggregate_loans = 0i128;
        let mut aggregate_debt = 0i128;
        let mut aggregate_deposit_liabilities = 0i128;
        let mut aggregate_trade_receivables = 0i128;
        let mut aggregate_trade_payables = 0i128;
        let mut aggregate_inventory_value = 0i128;

        for actor in &state.actors {
            aggregate_cash = aggregate_cash.checked_add(actor.monetary.cash)
                .ok_or_else(|| "aggregate cash overflow".to_string())?;
            aggregate_deposits = aggregate_deposits.checked_add(actor.monetary.deposits)
                .ok_or_else(|| "aggregate deposits overflow".to_string())?;
            aggregate_loans = aggregate_loans.checked_add(actor.monetary.claims)
                .ok_or_else(|| "aggregate loans overflow".to_string())?;
            aggregate_debt = aggregate_debt.checked_add(actor.monetary.liabilities)
                .ok_or_else(|| "aggregate debt overflow".to_string())?;
            aggregate_deposit_liabilities = aggregate_deposit_liabilities
                .checked_add(actor.monetary.deposit_liabilities)
                .ok_or_else(|| "aggregate deposit liabilities overflow".to_string())?;
            aggregate_trade_receivables = aggregate_trade_receivables
                .checked_add(actor.monetary.trade_receivables)
                .ok_or_else(|| "aggregate trade receivables overflow".to_string())?;
            aggregate_trade_payables = aggregate_trade_payables
                .checked_add(actor.monetary.trade_payables)
                .ok_or_else(|| "aggregate trade payables overflow".to_string())?;
            aggregate_inventory_value = aggregate_inventory_value
                .checked_add(actor.inventory_carrying_value)
                .ok_or_else(|| "aggregate inventory value overflow".to_string())?;
        }

        let net_working_capital = aggregate_inventory_value
            .checked_add(aggregate_trade_receivables)
            .and_then(|value| value.checked_sub(aggregate_trade_payables))
            .ok_or_else(|| "aggregate working-capital overflow".to_string())?;
        let liquidity = aggregate_cash
            .checked_add(aggregate_deposits)
            .ok_or_else(|| "aggregate liquidity overflow".to_string())?;
        let assets = state.try_aggregate_assets()?;
        let liabilities = state.try_aggregate_liabilities()?;

        let interest_paid = ledger.category_total(EconomicFlowCategory::Interest);
        let gross_debt_service = interest_paid
            .checked_add(ledger.debt_repaid)
            .ok_or_else(|| "aggregate debt service overflow".to_string())?;
        let gross_operating_surplus = ledger
            .try_gross_operating_surplus()
            .map_err(|error| format!("aggregate {error}"))?;
        let operating_surplus_after_depreciation = ledger
            .try_operating_surplus_after_depreciation()
            .map_err(|error| format!("aggregate {error}"))?;
        let net_credit_impulse = ledger
            .try_net_credit()
            .map_err(|error| format!("aggregate {error}"))?;

        Ok(Self {
            aggregate_cash,
            aggregate_deposits,
            aggregate_loans,
            aggregate_debt,
            aggregate_deposit_liabilities,
            aggregate_trade_receivables,
            aggregate_trade_payables,
            net_working_capital,
            liquidity,
            gross_leverage: (assets != 0).then_some(RatioObservation {
                numerator: liabilities,
                denominator: assets,
            }),
            credit_created: ledger.credit_created,
            debt_repaid: ledger.debt_repaid,
            debt_written_off: ledger.debt_written_off,
            trade_credit_extended: ledger.trade_credit_extended,
            trade_credit_settled: ledger.trade_credit_settled,
            net_credit_impulse,
            interest_paid,
            gross_debt_service,
            sales_consideration: ledger.sales_consideration,
            cost_of_goods_sold: ledger.cost_of_goods_sold,
            gross_operating_surplus,
            depreciation: ledger.depreciation,
            real_asset_revaluation: ledger.real_asset_revaluation,
            operating_surplus_after_depreciation,
        })
    }

    /// Derive measurements from state stocks and the deterministic period ledger.
    pub fn from_state_and_ledger(
        state: &EconomicState,
        ledger: &EconomicPeriodLedger,
    ) -> Self {
        Self::try_from_state_and_ledger(state, ledger)
            .expect("aggregate economic observable overflow")
    }

    /// Ratio of available liquidity to the measured gross debt service.
    pub fn liquidity_to_debt_service(&self) -> Option<RatioObservation> {
        (self.gross_debt_service != 0).then_some(RatioObservation {
            numerator: self.liquidity,
            denominator: self.gross_debt_service,
        })
    }

    /// Ratio of gross operating surplus to measured gross debt service.
    ///
    /// This is explicitly a surplus/service ratio, not a cash-flow coverage
    /// ratio. Callers that have a cash-flow measure should use that directly.
    pub fn gross_surplus_to_debt_service(&self) -> Option<RatioObservation> {
        (self.gross_debt_service != 0).then_some(RatioObservation {
            numerator: self.gross_operating_surplus,
            denominator: self.gross_debt_service,
        })
    }
}

/// Minsky-style financing regime derived from an externally supplied cash-flow
/// amount and contractual debt service.
///
/// The classifier is descriptive only. It does not state that one regime is
/// preferable and does not infer cash flow from accounting profit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinancingRegime {
    Hedge,
    Speculative,
    Ponzi,
}

/// Classify a financing position from cash available for debt service.
///
/// Hedge: cash flow covers interest and principal due.
/// Speculative: cash flow covers interest but not principal.
/// Ponzi: cash flow does not cover interest.
///
/// The cash-flow input must be supplied by the caller from an explicit model;
/// this function never substitutes revenue, surplus, or liquidity for cash flow.
pub fn classify_financing_regime(
    cash_flow_available: i128,
    interest_due: i128,
    principal_due: i128,
) -> Result<FinancingRegime, String> {
    if interest_due < 0 || principal_due < 0 {
        return Err("debt service obligations must be non-negative".into());
    }

    let total_due = interest_due
        .checked_add(principal_due)
        .ok_or_else(|| "debt service obligation overflow".to_string())?;

    if cash_flow_available >= total_due {
        Ok(FinancingRegime::Hedge)
    } else if cash_flow_available >= interest_due {
        Ok(FinancingRegime::Speculative)
    } else {
        Ok(FinancingRegime::Ponzi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{ActorBalanceSheet, CreditCreation, Depreciation};
    use crate::economics::transition::EconomicTransition;

    #[test]
    fn revaluation_changes_wealth_without_operating_surplus_or_cash_flow() {
        let mut firm = crate::economics::stock_flow::ActorBalanceSheet::new("firm");
        firm.real.productive_capital = 100;
        let state = EconomicState::new(vec![firm]);
        let transitions = vec![EconomicTransition::RealAssetRevaluation(
            crate::economics::stock_flow::RealAssetRevaluation::new(
                "firm",
                crate::economics::stock_flow::RealAssetRevaluationTarget::ProductiveCapital,
                25,
            )
            .unwrap(),
        )];

        let observations =
            EconomicObservables::try_from_state_and_transitions(&state, &transitions).unwrap();

        assert_eq!(observations.real_asset_revaluation, 25);
        assert_eq!(observations.liquidity, 0);
        assert_eq!(observations.gross_operating_surplus, 0);
        assert_eq!(observations.operating_surplus_after_depreciation, 0);
        assert_eq!(observations.credit_created, 0);
        assert_eq!(observations.debt_repaid, 0);
    }

    #[test]
    fn aggregate_observations_reject_invalid_state() {
        let mut state = EconomicState::new(vec![
            ActorBalanceSheet::new("household"),
        ]);
        state.actors[0].monetary.deposits = -1;

        let result =
            EconomicObservables::try_from_state_and_transitions(&state, &[]);

        assert!(matches!(
            result,
            Err(EconomicStepError::InvalidState(_))
        ));
    }

    #[test]
    fn aggregate_observations_derive_ledger_from_transitions() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("household"),
        ]);
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 25).unwrap(),
        )];

        let observations =
            EconomicObservables::try_from_state_and_transitions(&state, &transitions)
                .unwrap();

        assert_eq!(observations.credit_created, 25);
        assert_eq!(observations.aggregate_loans, 25);
        assert_eq!(observations.aggregate_debt, 25);
    }

    #[test]
    fn aggregate_observations_verify_against_transitions_rejects_tampering() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("household"),
        ]);
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 25).unwrap(),
        )];
        let mut observations = EconomicObservables::try_from_state_and_transitions(&state, &transitions).unwrap();
        observations.credit_created = 24;
        assert!(observations.verify_against_transitions(&state, &transitions).is_err());
    }

    #[test]
    fn aggregate_observations_track_post_transition_stocks() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("household"),
        ]);
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 25).unwrap(),
        )];
        let observations = EconomicObservables::try_from_state_and_transitions(&state, &transitions).unwrap();
        assert_eq!(observations.aggregate_loans, 25);
        assert_eq!(observations.aggregate_debt, 25);
        assert_eq!(observations.aggregate_deposits, 25);
    }

    #[test]
    fn aggregate_observations_reject_deserialized_invalid_transition() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("household"),
        ]);
        let transition = EconomicTransition::CreditCreation(
            CreditCreation {
                lender: "bank".into(),
                borrower: "household".into(),
                amount: 0,
            },
        );

        assert!(EconomicObservables::try_from_state_and_transitions(
            &state,
            &[transition],
        )
        .is_err());
    }

    #[test]
    fn observables_project_reconciled_stocks_and_period_totals() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        bank.monetary.claims = 200;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.deposits = 500;
        firm.monetary.liabilities = 200;
        firm.real.productive_capital = 800;

        let mut state = EconomicState::new(vec![bank, firm]);
        state
            .create_credit(&CreditCreation::new("bank", "firm", 1).unwrap())
            .unwrap();

        let transitions = vec![EconomicTransition::Depreciation(
            Depreciation::new("firm", 25).unwrap(),
        )];
        let ledger = EconomicPeriodLedger::from_transitions(&transitions).unwrap();
        let observations = EconomicObservables::from_state_and_ledger(&state, &ledger);

        assert_eq!(observations.aggregate_cash, 1_000);
        assert_eq!(observations.aggregate_deposits, 501);
        assert_eq!(observations.aggregate_debt, 201);
        assert_eq!(observations.aggregate_trade_receivables, 0);
        assert_eq!(observations.aggregate_trade_payables, 0);
        assert_eq!(observations.net_working_capital, 0);
        assert_eq!(observations.depreciation, 25);
        assert_eq!(observations.net_credit_impulse, 0);
        assert_eq!(observations.trade_credit_extended, 0);
        assert_eq!(observations.trade_credit_settled, 0);
        assert_eq!(observations.liquidity_to_debt_service(), None);
    }

    #[test]
    fn financing_regimes_are_exact_and_descriptive() {
        assert_eq!(
            classify_financing_regime(150, 100, 50).unwrap(),
            FinancingRegime::Hedge
        );
        assert_eq!(
            classify_financing_regime(120, 100, 50).unwrap(),
            FinancingRegime::Speculative
        );
        assert_eq!(
            classify_financing_regime(80, 100, 50).unwrap(),
            FinancingRegime::Ponzi
        );
        assert!(classify_financing_regime(10, -1, 2).is_err());
    }
}

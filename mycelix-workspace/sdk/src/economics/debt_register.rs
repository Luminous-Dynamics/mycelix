// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Opt-in counterparty identity for modeled debt positions.
//!
//! EconomicState stores aggregate loan claims and debt liabilities on each
//! actor. That is sufficient for aggregate SFC closure but cannot prove which
//! creditor owns which borrower obligation. This module supplies an explicit,
//! read-only provenance overlay until the canonical state carries typed debt
//! instruments.
//!
//! The register is deliberately narrower than a full debt-contract model:
//! positions are identified by lender/borrower pair and outstanding principal.
//! Instrument IDs, maturity, interest rate, currency, collateral, and other
//! contract terms remain future typed fields required for rescheduling,
//! refinancing, or debt assumption.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use super::stock_flow::{ActorId, EconomicState};
use super::transition::EconomicTransition;

/// One modeled outstanding debt position identified by its counterparties.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CounterpartyDebtPosition {
    pub lender: ActorId,
    pub borrower: ActorId,
    pub outstanding_principal: i128,
}

impl CounterpartyDebtPosition {
    pub fn new(
        lender: impl Into<ActorId>,
        borrower: impl Into<ActorId>,
        outstanding_principal: i128,
    ) -> Result<Self, String> {
        let lender = lender.into();
        let borrower = borrower.into();
        if lender.is_empty() || borrower.is_empty() {
            return Err("debt position counterparties must be non-empty".into());
        }
        if lender == borrower {
            return Err("debt position counterparties must be distinct".into());
        }
        if outstanding_principal <= 0 {
            return Err("debt position principal must be positive".into());
        }
        Ok(Self {
            lender,
            borrower,
            outstanding_principal,
        })
    }
}

/// Canonical counterparty overlay for aggregate loan/debt stocks.
///
/// The register is not part of EconomicState yet. It is an explicit
/// provenance artifact that can be supplied alongside a state and transition
/// program to prove counterparty-consistent debt extinction.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct CounterpartyDebtRegister {
    pub positions: Vec<CounterpartyDebtPosition>,
}

impl CounterpartyDebtRegister {
    pub fn new(positions: Vec<CounterpartyDebtPosition>) -> Result<Self, String> {
        let register = Self { positions };
        register.validate()?;
        Ok(register)
    }

    pub fn empty() -> Self {
        Self::default()
    }

    /// Validate position identities, uniqueness, and arithmetic domain.
    pub fn validate(&self) -> Result<(), String> {
        let mut keys = BTreeSet::new();
        for position in &self.positions {
            if position.lender.is_empty() || position.borrower.is_empty() {
                return Err("debt position counterparties must be non-empty".into());
            }
            if position.lender == position.borrower {
                return Err("debt position counterparties must be distinct".into());
            }
            if position.outstanding_principal <= 0 {
                return Err("debt position principal must be positive".into());
            }
            if !keys.insert((&position.lender, &position.borrower)) {
                return Err(format!(
                    "duplicate counterparty debt position: {} -> {}",
                    position.lender, position.borrower
                ));
            }
        }
        Ok(())
    }

    /// Require exact agreement with the aggregate loan/debt rows in a state.
    ///
    /// This proves both directions: lender outgoing positions sum to that
    /// actor's loan claims, and borrower incoming positions sum to that actor's
    /// debt liabilities.
    pub fn validate_against_state(&self, state: &EconomicState) -> Result<(), String> {
        state.validate()?;
        self.validate()?;

        let known: BTreeSet<&str> =
            state.actors.iter().map(|actor| actor.actor.as_str()).collect();
        let mut totals: BTreeMap<&str, (i128, i128)> = BTreeMap::new();

        for position in &self.positions {
            if !known.contains(position.lender.as_str()) {
                return Err(format!(
                    "debt register references unknown lender: {}",
                    position.lender
                ));
            }
            if !known.contains(position.borrower.as_str()) {
                return Err(format!(
                    "debt register references unknown borrower: {}",
                    position.borrower
                ));
            }

            let lender = totals.entry(position.lender.as_str()).or_default();
            lender.0 = lender
                .0
                .checked_add(position.outstanding_principal)
                .ok_or_else(|| "debt register lender total overflow".to_string())?;

            let borrower = totals.entry(position.borrower.as_str()).or_default();
            borrower.1 = borrower
                .1
                .checked_add(position.outstanding_principal)
                .ok_or_else(|| "debt register borrower total overflow".to_string())?;
        }

        for actor in &state.actors {
            let (claims, liabilities) = totals
                .get(actor.actor.as_str())
                .copied()
                .unwrap_or((0, 0));
            if claims != actor.monetary.claims {
                return Err(format!(
                    "debt register loan-claim mismatch for {}: expected {}, actual {}",
                    actor.actor, actor.monetary.claims, claims
                ));
            }
            if liabilities != actor.monetary.liabilities {
                return Err(format!(
                    "debt register debt-liability mismatch for {}: expected {}, actual {}",
                    actor.actor, actor.monetary.liabilities, liabilities
                ));
            }
        }

        Ok(())
    }

    /// Replay debt-affecting transitions against this opening counterparty set.
    ///
    /// Non-debt transitions are ignored. Debt creation adds to the exact
    /// lender/borrower pair; repayment, write-off, and forgiveness can only
    /// extinguish principal from that same pair.
    pub fn replay(
        &self,
        transitions: &[EconomicTransition],
    ) -> Result<Self, String> {
        self.validate()?;

        let mut positions: BTreeMap<(ActorId, ActorId), i128> = self
            .positions
            .iter()
            .map(|position| {
                (
                    (position.lender.clone(), position.borrower.clone()),
                    position.outstanding_principal,
                )
            })
            .collect();

        for transition in transitions {
            transition.validate()?;

            match transition {
                EconomicTransition::CreditCreation(credit) => {
                    let key = (credit.lender.clone(), credit.borrower.clone());
                    let current = positions.get(&key).copied().unwrap_or(0);
                    let next = current
                        .checked_add(credit.amount)
                        .ok_or_else(|| "debt register principal overflow".to_string())?;
                    positions.insert(key, next);
                }
                EconomicTransition::DebtRepayment(repayment) => {
                    reduce_position(
                        &mut positions,
                        &repayment.lender,
                        &repayment.borrower,
                        repayment.amount,
                        "debt repayment",
                    )?;
                }
                EconomicTransition::DebtWriteOff(write_off) => {
                    reduce_position(
                        &mut positions,
                        &write_off.lender,
                        &write_off.borrower,
                        write_off.amount,
                        "debt write-off",
                    )?;
                }
                EconomicTransition::DebtForgiveness(forgiveness) => {
                    reduce_position(
                        &mut positions,
                        &forgiveness.lender,
                        &forgiveness.borrower,
                        forgiveness.amount,
                        "debt forgiveness",
                    )?;
                }
                _ => {}
            }
        }

        let positions = positions
            .into_iter()
            .filter(|(_, amount)| *amount > 0)
            .map(|((lender, borrower), outstanding_principal)| CounterpartyDebtPosition {
                lender,
                borrower,
                outstanding_principal,
            })
            .collect();

        let result = Self { positions };
        result.validate()?;
        Ok(result)
    }

    /// Validate the opening register against state stocks, then replay the
    /// ordered debt transitions using exact counterparty matching.
    pub fn validate_against_state_and_replay(
        &self,
        state: &EconomicState,
        transitions: &[EconomicTransition],
    ) -> Result<Self, String> {
        self.validate_against_state(state)?;
        self.replay(transitions)
    }
}

fn reduce_position(
    positions: &mut BTreeMap<(ActorId, ActorId), i128>,
    lender: &str,
    borrower: &str,
    amount: i128,
    label: &str,
) -> Result<(), String> {
    let key = (lender.to_owned(), borrower.to_owned());
    let current = positions.get(&key).copied().ok_or_else(|| {
        format!(
            "{label} has no outstanding counterparty position for {lender} -> {borrower}"
        )
    })?;

    if current < amount {
        return Err(format!(
            "{label} exceeds outstanding counterparty position for {lender} -> {borrower}: have {current}, need {amount}"
        ));
    }

    let remaining = current - amount;
    if remaining == 0 {
        positions.remove(&key);
    } else {
        positions.insert(key, remaining);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::{
        ActorBalanceSheet, CreditCreation, DebtForgiveness, DebtRepayment, DebtWriteOff,
    };

    fn closed_state() -> EconomicState {
        let mut bank = ActorBalanceSheet::new("bank");
        let mut firm_a = ActorBalanceSheet::new("firm-a");
        let mut firm_b = ActorBalanceSheet::new("firm-b");
        bank.monetary.claims = 100;
        firm_a.monetary.liabilities = 60;
        firm_b.monetary.liabilities = 40;
        EconomicState::new(vec![bank, firm_a, firm_b])
    }

    #[test]
    fn register_matches_actor_level_loan_and_debt_stocks() {
        let register = CounterpartyDebtRegister::new(vec![
            CounterpartyDebtPosition::new("bank", "firm-a", 60).unwrap(),
            CounterpartyDebtPosition::new("bank", "firm-b", 40).unwrap(),
        ])
        .unwrap();

        register.validate_against_state(&closed_state()).unwrap();
    }

    #[test]
    fn register_rejects_wrong_counterparty_even_when_aggregate_rows_balance() {
        let register = CounterpartyDebtRegister::new(vec![
            CounterpartyDebtPosition::new("bank", "firm-a", 60).unwrap(),
            CounterpartyDebtPosition::new("bank", "firm-b", 40).unwrap(),
        ])
        .unwrap();

        let transitions = [EconomicTransition::DebtRepayment(
            DebtRepayment::new("bank", "firm-b", 50).unwrap(),
        )];

        let error = register.replay(&transitions).unwrap_err();
        assert!(error.contains("counterparty position"));
    }

    #[test]
    fn register_replays_creation_then_extinction_on_exact_pair() {
        let register = CounterpartyDebtRegister::empty();
        let transitions = [
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "firm-a", 50).unwrap(),
            ),
            EconomicTransition::DebtRepayment(
                DebtRepayment::new("bank", "firm-a", 10).unwrap(),
            ),
            EconomicTransition::DebtForgiveness(
                DebtForgiveness::new("bank", "firm-a", 15).unwrap(),
            ),
            EconomicTransition::DebtWriteOff(
                DebtWriteOff::new("bank", "firm-a", 25).unwrap(),
            ),
        ];

        let terminal = register.replay(&transitions).unwrap();
        assert!(terminal.positions.is_empty());
    }

    #[test]
    fn register_rejects_duplicate_pairs() {
        assert!(CounterpartyDebtRegister::new(vec![
            CounterpartyDebtPosition::new("bank", "firm", 10).unwrap(),
            CounterpartyDebtPosition::new("bank", "firm", 5).unwrap(),
        ])
        .is_err());
    }

    #[test]
    fn register_rejects_extinction_above_exact_pair() {
        let register = CounterpartyDebtRegister::new(vec![
            CounterpartyDebtPosition::new("bank", "firm-a", 20).unwrap(),
        ])
        .unwrap();

        let transitions = [EconomicTransition::DebtForgiveness(
            DebtForgiveness::new("bank", "firm-b", 20).unwrap(),
        )];

        assert!(register.replay(&transitions).is_err());
    }
}

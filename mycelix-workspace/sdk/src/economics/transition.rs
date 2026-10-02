// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic, evidence-bound economic timestep execution.
//!
//! This layer separates accounting primitives, an ordered transition log,
//! and deterministic state-transition evidence.

use serde::{Deserialize, Serialize};

use super::stock_flow::{CapitalInvestment, ProductionEvent, InventoryTransfer, InventoryConsumption, GoodsSale, CreditCreation, DebtRepayment, EconomicState, IncomeTransfer, MonetaryFlow};

/// One explicit economic state transition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum EconomicTransition {
    MonetaryTransfer(MonetaryFlow),
    IncomeTransfer(IncomeTransfer),
    CapitalInvestment(CapitalInvestment),
    Production(ProductionEvent),
    InventoryTransfer(InventoryTransfer),
    InventoryConsumption(InventoryConsumption),
    GoodsSale(GoodsSale),
    CreditCreation(CreditCreation),
    DebtRepayment(DebtRepayment),
}

/// Stable receipt describing one successfully applied timestep.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicStepReceipt {
    pub period: u64,
    pub pre_state_hash: String,
    pub transition_hash: String,
    pub post_state_hash: String,
    pub transition_count: u64,
}

/// Error returned when a step cannot be applied without violating an invariant.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EconomicStepError {
    PreStateMismatch { expected: String, actual: String },
    TransitionRejected(String),
    AccountingInvariant { claims: i128, liabilities: i128 },
    Serialization(String),
}

impl std::fmt::Display for EconomicStepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PreStateMismatch { expected, actual } => {
                write!(f, "pre-state hash mismatch: expected {expected}, got {actual}")
            }
            Self::TransitionRejected(message) => write!(f, "transition rejected: {message}"),
            Self::AccountingInvariant { claims, liabilities } => write!(
                f,
                "financial claim/liability invariant failed: claims={claims}, liabilities={liabilities}"
            ),
            Self::Serialization(message) => write!(f, "serialization failed: {message}"),
        }
    }
}

impl std::error::Error for EconomicStepError {}

/// Execute an ordered, deterministic economic timestep.
///
/// The input state is cloned and only committed to by returning the resulting
/// state. A failed transition therefore cannot leave a partially-mutated state
/// behind.
pub fn apply_step(
    state: &EconomicState,
    period: u64,
    transitions: &[EconomicTransition],
    expected_pre_state_hash: Option<&str>,
) -> Result<(EconomicState, EconomicStepReceipt), EconomicStepError> {
    let actual_pre_state_hash = state_hash(state)?;
    if let Some(expected) = expected_pre_state_hash {
        if expected != actual_pre_state_hash {
            return Err(EconomicStepError::PreStateMismatch {
                expected: expected.to_owned(),
                actual: actual_pre_state_hash,
            });
        }
    }

    let transition_hash = transition_hash(transitions)?;
    let mut next = state.clone();

    for transition in transitions {
        let result = match transition {
            EconomicTransition::MonetaryTransfer(flow) => next.apply_flow(flow),
            EconomicTransition::IncomeTransfer(transfer) => next.apply_income_transfer(transfer),
            EconomicTransition::CapitalInvestment(investment) => next.apply_capital_investment(investment),
            EconomicTransition::Production(production) => next.apply_production(production),
            EconomicTransition::InventoryTransfer(transfer) => next.apply_inventory_transfer(transfer),
            EconomicTransition::InventoryConsumption(consumption) => next.apply_inventory_consumption(consumption),
            EconomicTransition::GoodsSale(sale) => next.apply_goods_sale(sale),
            EconomicTransition::CreditCreation(credit) => next.create_credit(credit),
            EconomicTransition::DebtRepayment(repayment) => next.repay_debt(repayment),
        };

        if let Err(error) = result {
            return Err(EconomicStepError::TransitionRejected(error));
        }
    }

    let claims = aggregate_claims(&next);
    let liabilities = next.aggregate_liabilities();
    if claims != liabilities {
        return Err(EconomicStepError::AccountingInvariant {
            claims,
            liabilities,
        });
    }

    let post_state_hash = state_hash(&next)?;
    let receipt = EconomicStepReceipt {
        period,
        pre_state_hash: actual_pre_state_hash,
        transition_hash,
        post_state_hash,
        transition_count: transitions.len() as u64,
    };

    Ok((next, receipt))
}

/// Hash a complete economic state using canonical JSON serialization.
pub fn state_hash(state: &EconomicState) -> Result<String, EconomicStepError> {
    let bytes =
        serde_json::to_vec(state).map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
}

/// Hash the exact ordered transition list.
pub fn transition_hash(
    transitions: &[EconomicTransition],
) -> Result<String, EconomicStepError> {
    let bytes = serde_json::to_vec(transitions)
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
}

fn aggregate_claims(state: &EconomicState) -> i128 {
    state.aggregate_claims()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::ActorBalanceSheet;

    fn initial_state() -> EconomicState {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        EconomicState::new(vec![
            bank,
            ActorBalanceSheet::new("household"),
        ])
    }

    #[test]
    fn step_is_deterministic() {
        let state = initial_state();
        let transitions = vec![
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "household", 500).unwrap(),
            ),
            EconomicTransition::MonetaryTransfer(
                MonetaryFlow::deposit_transfer("household", "bank", 100).unwrap(),
            ),
        ];

        let (first, receipt_a) = apply_step(&state, 1, &transitions, None).unwrap();
        let (second, receipt_b) = apply_step(&state, 1, &transitions, None).unwrap();

        assert_eq!(first, second);
        assert_eq!(receipt_a, receipt_b);
    }

    #[test]
    fn pre_state_binding_rejects_wrong_state() {
        let state = initial_state();
        let error = apply_step(&state, 1, &[], Some("wrong-hash")).unwrap_err();

        assert!(matches!(
            error,
            EconomicStepError::PreStateMismatch { .. }
        ));
    }

    #[test]
    fn failed_step_does_not_mutate_caller_state() {
        let state = initial_state();
        let before = state.clone();

        let transitions = vec![EconomicTransition::MonetaryTransfer(
            MonetaryFlow::new("household", "bank", 1).unwrap(),
        )];

        assert!(apply_step(&state, 1, &transitions, None).is_err());
        assert_eq!(state, before);
    }

    #[test]
    fn receipt_binds_exact_transition_sequence() {
        let state = initial_state();
        let one = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 500).unwrap(),
        )];
        let two = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 501).unwrap(),
        )];

        let (_, receipt_one) = apply_step(&state, 1, &one, None).unwrap();
        let (_, receipt_two) = apply_step(&state, 1, &two, None).unwrap();

        assert_ne!(receipt_one.transition_hash, receipt_two.transition_hash);
        assert_ne!(receipt_one.post_state_hash, receipt_two.post_state_hash);
    }

    #[test]
    fn physical_inventory_circuit_is_deterministic_and_atomic() {
        let mut state = initial_state();
        state.actors.iter_mut().find(|a| a.actor == "household").unwrap().real.inventories = 10;
        let transitions = vec![
            EconomicTransition::InventoryTransfer(
                InventoryTransfer::new("household", "bank", 4).unwrap(),
            ),
            EconomicTransition::InventoryConsumption(
                InventoryConsumption::new("bank", 2).unwrap(),
            ),
        ];

        let (first, receipt_a) = apply_step(&state, 1, &transitions, None).unwrap();
        let (second, receipt_b) = apply_step(&state, 1, &transitions, None).unwrap();

        assert_eq!(first, second);
        assert_eq!(receipt_a, receipt_b);
        assert_eq!(
            first.actors.iter().find(|a| a.actor == "household").unwrap().real.inventories,
            6
        );
        assert_eq!(
            first.actors.iter().find(|a| a.actor == "bank").unwrap().real.inventories,
            2
        );
    }

    #[test]
    fn credit_and_repayment_can_close_a_step() {
        let state = initial_state();
        let transitions = vec![
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "household", 500).unwrap(),
            ),
            EconomicTransition::DebtRepayment(
                DebtRepayment::new("bank", "household", 500).unwrap(),
            ),
        ];

        let (next, receipt) = apply_step(&state, 1, &transitions, None).unwrap();

        assert_eq!(next.aggregate_liabilities(), 0);
        assert_eq!(next.actors[0].monetary.claims, 0);
        assert_eq!(receipt.transition_count, 2);
    }
}

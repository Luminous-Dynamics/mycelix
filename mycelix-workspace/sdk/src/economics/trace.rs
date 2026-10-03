// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic multi-period economic simulation traces.

use serde::{Deserialize, Serialize};

use super::stock_flow::EconomicState;
use super::transition::{
    apply_step, state_hash, EconomicChainReceipt, EconomicStepError, EconomicTransition,
};

/// One period's ordered transition program.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicSimulationStep {
    pub period: u64,
    pub transitions: Vec<EconomicTransition>,
}

/// Complete deterministic trace of a multi-period simulation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicSimulationTrace {
    pub initial_state_hash: String,
    pub final_state_hash: String,
    pub receipts: Vec<EconomicChainReceipt>,
    pub trace_hash: String,
}

impl EconomicSimulationTrace {
    pub fn run(
        initial_state: &EconomicState,
        steps: &[EconomicSimulationStep],
    ) -> Result<(EconomicState, Self), EconomicStepError> {
        initial_state
            .validate()
            .map_err(EconomicStepError::InvalidState)?;

        let initial_state_hash = state_hash(initial_state)?;
        let mut state = initial_state.clone();
        let mut previous_receipt: Option<EconomicChainReceipt> = None;
        let mut previous_period: Option<u64> = None;
        let mut receipts = Vec::with_capacity(steps.len());

        for step in steps {
            if let Some(previous) = previous_period {
                if step.period <= previous {
                    return Err(EconomicStepError::Serialization(
                        "simulation trace periods must be strictly increasing".into(),
                    ));
                }
            }

            let (next_state, receipt) = apply_step(&state, step.period, &step.transitions, None)?;
            let chained = EconomicChainReceipt::link(previous_receipt.as_ref(), receipt)?;
            receipts.push(chained.clone());
            previous_receipt = Some(chained);
            previous_period = Some(step.period);
            state = next_state;
        }

        let final_state_hash = state_hash(&state)?;
        let bytes = serde_json::to_vec(&(
            &initial_state_hash,
            &final_state_hash,
            &receipts,
        ))
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let trace_hash = blake3::hash(&bytes).to_hex().to_string();

        Ok((
            state,
            Self {
                initial_state_hash,
                final_state_hash,
                receipts,
                trace_hash,
            },
        ))
    }

    pub fn final_receipt(&self) -> Option<&EconomicChainReceipt> {
        self.receipts.last()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::{
        ActorBalanceSheet, CreditCreation, DebtRepayment, MonetaryFlow,
    };

    fn initial_state() -> EconomicState {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        EconomicState::new(vec![
            bank,
            ActorBalanceSheet::new("household"),
        ])
    }

    #[test]
    fn multi_period_trace_is_deterministic_and_chain_bound() {
        let initial = initial_state();
        let steps = vec![
            EconomicSimulationStep {
                period: 1,
                transitions: vec![EconomicTransition::CreditCreation(
                    CreditCreation::new("bank", "household", 500).unwrap(),
                )],
            },
            EconomicSimulationStep {
                period: 2,
                transitions: vec![EconomicTransition::MonetaryTransfer(
                    MonetaryFlow::deposit_transfer("household", "bank", 100).unwrap(),
                )],
            },
        ];

        let (state_a, trace_a) = EconomicSimulationTrace::run(&initial, &steps).unwrap();
        let (state_b, trace_b) = EconomicSimulationTrace::run(&initial, &steps).unwrap();

        assert_eq!(state_a, state_b);
        assert_eq!(trace_a, trace_b);
        assert_eq!(trace_a.receipts.len(), 2);
        assert_eq!(
            trace_a.receipts[1].previous_receipt_hash,
            Some(trace_a.receipts[0].chain_hash.clone())
        );
        assert_eq!(
            trace_a.final_state_hash,
            state_hash(&state_a).unwrap()
        );
    }

    #[test]
    fn changing_one_period_changes_the_trace() {
        let initial = initial_state();
        let first = vec![EconomicSimulationStep {
            period: 1,
            transitions: vec![EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "household", 500).unwrap(),
            )],
        }];
        let second = vec![EconomicSimulationStep {
            period: 1,
            transitions: vec![EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "household", 501).unwrap(),
            )],
        }];

        let (_, trace_a) = EconomicSimulationTrace::run(&initial, &first).unwrap();
        let (_, trace_b) = EconomicSimulationTrace::run(&initial, &second).unwrap();

        assert_ne!(trace_a.trace_hash, trace_b.trace_hash);
        assert_ne!(
            trace_a.final_state_hash,
            trace_b.final_state_hash
        );
    }

    #[test]
    fn empty_trace_is_well_defined() {
        let initial = initial_state();
        let (state, trace) = EconomicSimulationTrace::run(&initial, &[]).unwrap();

        assert_eq!(state, initial);
        assert!(trace.receipts.is_empty());
        assert_eq!(trace.final_state_hash, trace.initial_state_hash);
        assert_eq!(trace.final_receipt(), None);
    }

    #[test]
    fn empty_trace_rejects_invalid_initial_state() {
        let mut initial = initial_state();
        initial.actors[1].monetary.deposits = -1;

        assert!(matches!(
            EconomicSimulationTrace::run(&initial, &[]),
            Err(EconomicStepError::InvalidState(_))
        ));
    }

    #[test]
    fn trace_rejects_non_monotonic_periods() {
        let initial = initial_state();
        let steps = vec![
            EconomicSimulationStep {
                period: 2,
                transitions: vec![],
            },
            EconomicSimulationStep {
                period: 1,
                transitions: vec![],
            },
        ];

        assert!(matches!(
            EconomicSimulationTrace::run(&initial, &steps),
            Err(EconomicStepError::Serialization(message))
                if message.contains("strictly increasing")
        ));
    }

    #[test]
    fn invalid_later_period_does_not_commit_partial_trace() {
        let initial = initial_state();
        let steps = vec![
            EconomicSimulationStep {
                period: 1,
                transitions: vec![EconomicTransition::CreditCreation(
                    CreditCreation::new("bank", "household", 500).unwrap(),
                )],
            },
            EconomicSimulationStep {
                period: 2,
                transitions: vec![EconomicTransition::DebtRepayment(
                    DebtRepayment::new("bank", "household", 600).unwrap(),
                )],
            },
        ];

        assert!(EconomicSimulationTrace::run(&initial, &steps).is_err());
    }
}

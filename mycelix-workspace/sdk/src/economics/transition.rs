// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic, evidence-bound economic timestep execution.
//!
//! This layer separates accounting primitives, an ordered transition log,
//! and deterministic state-transition evidence.

use serde::{Deserialize, Serialize};

use super::stock_flow::{CapitalInvestment, ProductionEvent, InventoryTransfer, InventoryConsumption, GoodsSale, InventoryCostAddition, InventoryCostRelief, Depreciation, CreditCreation, DebtRepayment, EconomicState, IncomeTransfer, MonetaryFlow};

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
    InventoryCostAddition(InventoryCostAddition),
    InventoryCostRelief(InventoryCostRelief),
    Depreciation(Depreciation),
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

/// Evidence-chain receipt linking one timestep receipt to its predecessor.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicChainReceipt {
    pub genesis_state_hash: String,
    pub previous_receipt_hash: Option<String>,
    pub step: EconomicStepReceipt,
    pub chain_hash: String,
}

impl EconomicChainReceipt {
    /// Link a successful step receipt to the previous chain node.
    ///
    /// The chain hash binds the genesis state, ordering, complete step receipt,
    /// and predecessor. A changed initial state, period, transition list,
    /// state hash, or predecessor therefore changes the downstream evidence.
    pub fn link(
        previous: Option<&EconomicChainReceipt>,
        step: EconomicStepReceipt,
    ) -> Result<Self, EconomicStepError> {
        let (genesis_state_hash, previous_receipt_hash) = match previous {
            Some(previous) => {
                if step.pre_state_hash != previous.step.post_state_hash {
                    return Err(EconomicStepError::PreStateMismatch {
                        expected: previous.step.post_state_hash.clone(),
                        actual: step.pre_state_hash.clone(),
                    });
                }
                (
                    previous.genesis_state_hash.clone(),
                    Some(previous.chain_hash.clone()),
                )
            }
            None => (step.pre_state_hash.clone(), None),
        };

        if genesis_state_hash.is_empty() {
            return Err(EconomicStepError::Serialization(
                "evidence chain requires a non-empty genesis state hash".into(),
            ));
        }

        let bytes = serde_json::to_vec(&(
            &genesis_state_hash,
            &previous_receipt_hash,
            &step,
        ))
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let chain_hash = blake3::hash(&bytes).to_hex().to_string();

        Ok(Self {
            genesis_state_hash,
            previous_receipt_hash,
            step,
            chain_hash,
        })
    }
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
            EconomicTransition::InventoryCostAddition(addition) => next.apply_inventory_cost_addition(addition),
            EconomicTransition::InventoryCostRelief(relief) => next.apply_inventory_cost_relief(relief),
            EconomicTransition::Depreciation(depreciation) => next.apply_depreciation(depreciation),
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
    let mut canonical = state.clone();
    canonical.actors.sort_by(|a, b| a.actor.cmp(&b.actor));

    for window in canonical.actors.windows(2) {
        if window[0].actor == window[1].actor {
            return Err(EconomicStepError::Serialization(
                format!("duplicate economic actor id: {}", window[0].actor),
            ));
        }
    }

    let bytes = serde_json::to_vec(&canonical)
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
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
    fn state_hash_is_canonical_over_actor_order() {
        let state_a = EconomicState::new(vec![
            ActorBalanceSheet::new("z"),
            ActorBalanceSheet::new("a"),
        ]);
        let state_b = EconomicState::new(vec![
            ActorBalanceSheet::new("a"),
            ActorBalanceSheet::new("z"),
        ]);
        assert_eq!(state_hash(&state_a).unwrap(), state_hash(&state_b).unwrap());
    }

    #[test]
    fn duplicate_actor_ids_are_rejected_by_state_hash() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("same"),
            ActorBalanceSheet::new("same"),
        ]);
        assert!(state_hash(&state).is_err());
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
    fn evidence_chain_is_deterministic_and_order_bound() {
        let state = initial_state();
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 500).unwrap(),
        )];
        let (_, step) = apply_step(&state, 1, &transitions, None).unwrap();

        let first = EconomicChainReceipt::link(None, step.clone()).unwrap();
        let second = EconomicChainReceipt::link(Some(&first), step.clone()).unwrap();
        let second_again = EconomicChainReceipt::link(Some(&first), step).unwrap();

        assert_eq!(second, second_again);
        assert_eq!(first.genesis_state_hash, first.step.pre_state_hash);
        assert_eq!(second.genesis_state_hash, first.genesis_state_hash);
        assert_ne!(first.chain_hash, second.chain_hash);

        let reordered = EconomicChainReceipt::link(None, second.step).unwrap();
        assert_ne!(second.chain_hash, reordered.chain_hash);
    }

    #[test]
    fn evidence_chain_rejects_disconnected_receipts() {
        let state = initial_state();
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 500).unwrap(),
        )];
        let (_, step) = apply_step(&state, 1, &transitions, None).unwrap();
        let first = EconomicChainReceipt::link(None, step.clone()).unwrap();

        let mut disconnected = step;
        disconnected.pre_state_hash = "disconnected".into();

        assert!(matches!(
            EconomicChainReceipt::link(Some(&first), disconnected),
            Err(EconomicStepError::PreStateMismatch { .. })
        ));
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
    fn goods_sale_bridges_inventory_and_deposits_without_hidden_valuation() {
        let mut state = initial_state();
        state.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 20;
        state.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 100;

        let transitions = vec![EconomicTransition::GoodsSale(
            GoodsSale::new("firm", "household", 5, 30).unwrap(),
        )];
        let (post, _) = apply_step(&state, 1, &transitions, None).unwrap();

        let firm = post.actors.iter().find(|a| a.actor == "firm").unwrap();
        let household = post.actors.iter().find(|a| a.actor == "household").unwrap();
        assert_eq!(firm.real.inventories, 15);
        assert_eq!(household.real.inventories, 5);
        assert_eq!(firm.monetary.deposits, 30);
        assert_eq!(household.monetary.deposits, 70);
        assert_eq!(post.monetary_flow_volume, 30);
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

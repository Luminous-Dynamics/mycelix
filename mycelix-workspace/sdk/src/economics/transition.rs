// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic, evidence-bound economic timestep execution.
//!
//! This layer separates accounting primitives, an ordered transition log,
//! and deterministic state-transition evidence.

use serde::{Deserialize, Serialize};

use super::stock_flow::{CapitalInvestment, ProductionEvent, InventoryTransfer, InventoryConsumption, GoodsSale, TradeCreditSale, TradeCreditSettlement, InventoryCostAddition, InventoryCostRelief, Depreciation, CreditCreation, DebtRepayment, EconomicState, IncomeTransfer, MonetaryFlow};

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
    TradeCreditSale(TradeCreditSale),
    TradeCreditSettlement(TradeCreditSettlement),
    InventoryCostAddition(InventoryCostAddition),
    InventoryCostRelief(InventoryCostRelief),
    Depreciation(Depreciation),
    CreditCreation(CreditCreation),
    DebtRepayment(DebtRepayment),
}

impl EconomicTransition {
    /// Apply this validated transition to a mutable economic state.
    ///
    /// This is the single canonical mutation dispatcher used by execution and
    /// observation replay so those paths cannot silently diverge in the future.
    pub(crate) fn apply_to_state(&self, state: &mut EconomicState) -> Result<(), String> {
        match self {
            Self::MonetaryTransfer(flow) => state.apply_flow(flow),
            Self::IncomeTransfer(flow) => state.apply_income_transfer(flow),
            Self::CapitalInvestment(investment) => state.apply_capital_investment(investment),
            Self::Production(production) => state.apply_production(production),
            Self::InventoryTransfer(transfer) => state.apply_inventory_transfer(transfer),
            Self::InventoryConsumption(consumption) => state.apply_inventory_consumption(consumption),
            Self::GoodsSale(sale) => state.apply_goods_sale(sale),
            Self::TradeCreditSale(sale) => state.apply_trade_credit_sale(sale),
            Self::TradeCreditSettlement(settlement) => state.apply_trade_credit_settlement(settlement),
            Self::InventoryCostAddition(addition) => state.apply_inventory_cost_addition(addition),
            Self::InventoryCostRelief(relief) => state.apply_inventory_cost_relief(relief),
            Self::Depreciation(depreciation) => state.apply_depreciation(depreciation),
            Self::CreditCreation(credit) => state.create_credit(credit),
            Self::DebtRepayment(repayment) => state.repay_debt(repayment),
        }
    }

    /// Validate transition-domain invariants independently of constructors.
    ///
    /// The transition enum is deserializable, so callers can construct values
    /// without invoking the individual constructor functions. This shared
    /// validator keeps derived projections and execution on the same domain.
    pub fn validate(&self) -> Result<(), String> {
        let require_positive = |amount: i128, label: &str| {
            if amount <= 0 {
                Err(format!("{label} amount must be positive"))
            } else {
                Ok(())
            }
        };

        match self {
            Self::MonetaryTransfer(flow) => require_positive(flow.amount, "monetary transfer"),
            Self::IncomeTransfer(flow) => require_positive(flow.amount, "income transfer"),
            Self::CapitalInvestment(investment) => {
                require_positive(investment.amount, "capital investment")
            }
            Self::Production(production) => {
                require_positive(production.resource_input, "production resource input")?;
                require_positive(production.output, "production output")
            }
            Self::InventoryTransfer(transfer) => {
                require_positive(transfer.quantity, "inventory transfer")
            }
            Self::InventoryConsumption(consumption) => {
                require_positive(consumption.quantity, "inventory consumption")
            }
            Self::GoodsSale(sale) => {
                require_positive(sale.quantity, "goods sale quantity")?;
                require_positive(sale.consideration, "goods sale consideration")
            }
            Self::TradeCreditSale(sale) => {
                require_positive(sale.quantity, "trade-credit sale quantity")?;
                require_positive(sale.consideration, "trade-credit sale consideration")
            }
            Self::TradeCreditSettlement(settlement) => {
                require_positive(settlement.amount, "trade-credit settlement")
            }
            Self::InventoryCostAddition(addition) => {
                require_positive(addition.quantity, "inventory cost addition quantity")?;
                require_positive(
                    addition.carrying_value,
                    "inventory cost addition carrying value",
                )
            }
            Self::InventoryCostRelief(relief) => {
                require_positive(relief.quantity, "inventory cost relief quantity")?;
                require_positive(
                    relief.carrying_value,
                    "inventory cost relief carrying value",
                )
            }
            Self::Depreciation(depreciation) => {
                require_positive(depreciation.amount, "depreciation")
            }
            Self::CreditCreation(credit) => {
                require_positive(credit.amount, "credit creation")
            }
            Self::DebtRepayment(repayment) => {
                require_positive(repayment.amount, "debt repayment")
            }
        }
    }
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
    /// Verify the receipt's internal hash and predecessor relationship.
    ///
    /// This does not prove that the step was actually executed; that proof
    /// comes from apply_step or a higher-level accounting closure. It does
    /// prove that the serialized receipt has not been altered without
    /// recomputing its chain hash.
    pub fn verify(&self) -> Result<(), EconomicStepError> {
        if self.genesis_state_hash.is_empty()
            || self.step.pre_state_hash.is_empty()
            || self.step.transition_hash.is_empty()
            || self.step.post_state_hash.is_empty()
            || self.chain_hash.is_empty()
        {
            return Err(EconomicStepError::Serialization(
                "evidence chain receipt requires non-empty state, transition, and chain hashes"
                    .into(),
            ));
        }

        if self.previous_receipt_hash.is_none()
            && self.genesis_state_hash != self.step.pre_state_hash
        {
            return Err(EconomicStepError::Serialization(
                "genesis receipt must bind genesis state to step pre-state".into(),
            ));
        }

        if self
            .previous_receipt_hash
            .as_ref()
            .is_some_and(String::is_empty)
        {
            return Err(EconomicStepError::Serialization(
                "previous receipt hash must be non-empty when present".into(),
            ));
        }

        let bytes = serde_json::to_vec(&(
            &self.genesis_state_hash,
            &self.previous_receipt_hash,
            &self.step,
        ))
        .map_err(|error| EconomicStepError::Serialization(error.to_string()))?;
        let expected = blake3::hash(&bytes).to_hex().to_string();
        if self.chain_hash != expected {
            return Err(EconomicStepError::Serialization(
                "evidence chain receipt hash mismatch".into(),
            ));
        }

        Ok(())
    }

    /// Link a successful step receipt to the previous chain node.
    ///
    /// The chain hash binds the genesis state, ordering, complete step receipt,
    /// and predecessor. A changed initial state, period, transition list,
    /// state hash, or predecessor therefore changes the downstream evidence.
    pub fn link(
        previous: Option<&EconomicChainReceipt>,
        step: EconomicStepReceipt,
    ) -> Result<Self, EconomicStepError> {
        if step.pre_state_hash.is_empty()
            || step.transition_hash.is_empty()
            || step.post_state_hash.is_empty()
        {
            return Err(EconomicStepError::Serialization(
                "evidence chain step requires non-empty state and transition hashes".into(),
            ));
        }

        let (genesis_state_hash, previous_receipt_hash) = match previous {
            Some(previous) => {
                previous.verify()?;

                if step.period <= previous.step.period {
                    return Err(EconomicStepError::Serialization(
                        "evidence chain periods must be strictly increasing".into(),
                    ));
                }

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
    InvalidState(String),
    AccountingInvariant { claims: i128, liabilities: i128 },
    AccountingArithmetic(String),
    Serialization(String),
}

impl std::fmt::Display for EconomicStepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PreStateMismatch { expected, actual } => {
                write!(f, "pre-state hash mismatch: expected {expected}, got {actual}")
            }
            Self::TransitionRejected(message) => write!(f, "transition rejected: {message}"),
            Self::InvalidState(message) => write!(f, "invalid economic state: {message}"),
            Self::AccountingInvariant { claims, liabilities } => write!(
                f,
                "financial claim/liability invariant failed: claims={claims}, liabilities={liabilities}"
            ),
            Self::AccountingArithmetic(message) => {
                write!(f, "accounting arithmetic failed: {message}")
            }
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
    state
        .validate()
        .map_err(EconomicStepError::InvalidState)?;

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
        transition
            .validate()
            .map_err(EconomicStepError::TransitionRejected)?;

        let result = transition.apply_to_state(&mut next);

        if let Err(error) = result {
            return Err(EconomicStepError::TransitionRejected(error));
        }
    }

    next
        .validate()
        .map_err(EconomicStepError::InvalidState)?;

    let claims = next
        .try_aggregate_claims()
        .map_err(EconomicStepError::AccountingArithmetic)?;
    let liabilities = next
        .try_aggregate_liabilities()
        .map_err(EconomicStepError::AccountingArithmetic)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::stock_flow::ActorBalanceSheet;

    fn initial_state() -> EconomicState {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;
        EconomicState::new(vec![
            bank,
            ActorBalanceSheet::new("firm"),
            ActorBalanceSheet::new("household"),
        ])
    }

    #[test]
    fn transition_validation_catches_deserialized_zero_amount() {
        let transition = EconomicTransition::CreditCreation(CreditCreation {
            lender: "bank".into(),
            borrower: "household".into(),
            amount: 0,
        });
        assert!(transition.validate().is_err());
    }

    #[test]
    fn step_rejects_invalid_pre_state_before_applying_transitions() {
        let mut state = initial_state();
        state.actors[1].monetary.deposits = -1;
        let before = state.clone();

        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 10).unwrap(),
        )];

        assert!(matches!(
            apply_step(&state, 1, &transitions, None),
            Err(EconomicStepError::InvalidState(_))
        ));
        assert_eq!(state, before);
    }

    #[test]
    fn step_rejects_aggregate_arithmetic_overflow_without_panicking() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet {
                actor: "a".into(),
                monetary: crate::economics::stock_flow::MonetaryStock {
                    deposits: i128::MAX,
                    ..Default::default()
                },
                real: Default::default(),
                inventory_carrying_value: 0,
            },
            ActorBalanceSheet {
                actor: "b".into(),
                monetary: crate::economics::stock_flow::MonetaryStock {
                    deposits: 1,
                    ..Default::default()
                },
                real: Default::default(),
                inventory_carrying_value: 0,
            },
        ]);

        assert!(matches!(
            apply_step(&state, 1, &[], None),
            Err(EconomicStepError::InvalidState(_))
        ));
    }

    #[test]
    fn step_rejects_unmatched_pre_state_financial_claims() {
        let mut state = initial_state();
        state.actors[0].monetary.deposit_liabilities = 10;
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 10).unwrap(),
        )];

        assert!(matches!(
            apply_step(&state, 1, &transitions, None),
            Err(EconomicStepError::InvalidState(_))
        ));
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
    fn chain_link_rejects_non_monotonic_period() {
        let state = initial_state();
        let (_, step) = apply_step(&state, 2, &[], None).unwrap();
        let predecessor = EconomicChainReceipt::link(None, step).unwrap();

        let (_, successor_step) = apply_step(&state, 1, &[], None).unwrap();
        assert!(matches!(
            EconomicChainReceipt::link(Some(&predecessor), successor_step),
            Err(EconomicStepError::Serialization(message))
                if message.contains("strictly increasing")
        ));
    }

    #[test]
    fn chain_link_rejects_same_period() {
        let state = initial_state();
        let (_, step) = apply_step(&state, 1, &[], None).unwrap();
        let predecessor = EconomicChainReceipt::link(None, step.clone()).unwrap();

        let mut successor_step = step;
        successor_step.pre_state_hash = predecessor.step.post_state_hash.clone();
        assert!(EconomicChainReceipt::link(Some(&predecessor), successor_step).is_err());
    }

    #[test]
    fn chain_link_rejects_tampered_predecessor() {
        let state = initial_state();
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 500).unwrap(),
        )];
        let (_, step) = apply_step(&state, 1, &transitions, None).unwrap();
        let mut predecessor = EconomicChainReceipt::link(None, step).unwrap();
        predecessor.chain_hash = "tampered".into();

        let (_, successor_step) = apply_step(
            &state,
            2,
            &[],
            None,
        )
        .unwrap();

        assert!(EconomicChainReceipt::link(
            Some(&predecessor),
            successor_step,
        )
        .is_err());
    }

    #[test]
    fn chain_link_rejects_empty_step_hashes() {
        let step = EconomicStepReceipt {
            period: 1,
            pre_state_hash: "pre".into(),
            transition_hash: String::new(),
            post_state_hash: "post".into(),
            transition_count: 0,
        };

        assert!(EconomicChainReceipt::link(None, step).is_err());
    }

    #[test]
    fn chain_receipt_verify_detects_tampering() {
        let state = initial_state();
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 500).unwrap(),
        )];
        let (_, step) = apply_step(&state, 1, &transitions, None).unwrap();
        let receipt = EconomicChainReceipt::link(None, step).unwrap();
        receipt.verify().unwrap();

        let mut tampered = receipt.clone();
        tampered.step.post_state_hash = "tampered".into();
        assert!(tampered.verify().is_err());

        let mut rehashed = receipt;
        rehashed.step.post_state_hash = "tampered".into();
        let bytes = serde_json::to_vec(&(
            &rehashed.genesis_state_hash,
            &rehashed.previous_receipt_hash,
            &rehashed.step,
        ))
        .unwrap();
        rehashed.chain_hash = blake3::hash(&bytes).to_hex().to_string();
        assert!(rehashed.verify().is_ok());
    }

    #[test]
    fn genesis_receipt_verify_rejects_inconsistent_pre_state() {
        let state = initial_state();
        let (_, step) = apply_step(&state, 1, &[], None).unwrap();
        let mut receipt = EconomicChainReceipt::link(None, step).unwrap();
        receipt.step.pre_state_hash = "wrong-pre-state".into();

        assert!(receipt.verify().is_err());
    }

    #[test]
    fn evidence_chain_is_deterministic_and_order_bound() {
        let state = initial_state();
        let transitions = vec![EconomicTransition::CreditCreation(
            CreditCreation::new("bank", "household", 500).unwrap(),
        )];
        let (_, step_one) = apply_step(&state, 1, &transitions, None).unwrap();
        let (_, step_two) = apply_step(&state, 2, &transitions, None).unwrap();

        let first = EconomicChainReceipt::link(None, step_one.clone()).unwrap();
        let second = EconomicChainReceipt::link(Some(&first), step_two.clone()).unwrap();
        let second_again = EconomicChainReceipt::link(Some(&first), step_two).unwrap();

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
    fn trade_credit_sale_and_settlement_are_evidence_bound() {
        let mut state = initial_state();
        state.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 10;
        state.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 100;

        let transitions = vec![
            EconomicTransition::TradeCreditSale(
                TradeCreditSale::new("firm", "household", 4, 80).unwrap(),
            ),
            EconomicTransition::TradeCreditSettlement(
                TradeCreditSettlement::new("firm", "household", 80).unwrap(),
            ),
        ];

        let (post, receipt) = apply_step(&state, 1, &transitions, None).unwrap();
        let firm = post.actors.iter().find(|a| a.actor == "firm").unwrap();
        let household = post.actors.iter().find(|a| a.actor == "household").unwrap();

        assert_eq!(firm.monetary.trade_receivables, 0);
        assert_eq!(household.monetary.trade_payables, 0);
        assert_eq!(firm.monetary.deposits, 80);
        assert_eq!(household.monetary.deposits, 20);
        assert_eq!(firm.real.inventories, 6);
        assert_eq!(household.real.inventories, 4);
        assert_eq!(receipt.transition_count, 2);
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

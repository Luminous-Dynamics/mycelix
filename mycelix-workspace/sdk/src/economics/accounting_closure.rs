// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! One-call closure proof for the SFC accounting projections.
//!
//! The authoritative economic state transition is projected independently into:
//! - sector transaction flows,
//! - sector financial-claim flows,
//! - sector terminal observables, and
//! - monetary/physical stock-flow postings.
//!
//! This receipt verifies that those projections agree with the same ordered
//! transition log and with the resulting post-state.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::actor_observables::ActorEconomicObservables;
use super::observables::EconomicObservables;
use super::period_ledger::EconomicPeriodLedger;
use super::reconciliation::{reconcile_step, StockFlowReconciliation};
use super::sector_balance::{SectorAssignment, SectorBalanceSheet};
use super::sector_financial_flow::SectorFinancialFlowMatrix;
use super::sector_flow::{EconomicSector, SectorTransactionMatrix};
use super::sector_observables::SectorEconomicObservables;
use super::stock_flow::{ActorId, EconomicState};
use super::transition::{state_hash, transition_hash, EconomicTransition};

/// Deterministic cross-layer accounting closure receipt for one economic step.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicAccountingClosure {
    pub pre_state_hash: String,
    pub post_state_hash: String,
    pub transition_hash: String,
    pub transition_count: u64,
    pub actor_observations_hash: String,
    pub stock_flow_posting_hash: String,
    pub stock_flow_posting_count: u64,
    pub physical_posting_hash: String,
    pub physical_posting_count: u64,
    pub sector_transaction_hash: String,
    pub sector_financial_flow_hash: String,
    pub sector_observations_hash: String,
    pub closure_hash: String,
}

impl EconomicAccountingClosure {
    /// Validate and bind all current accounting projections for one step.
    ///
    /// No behavioral equation is introduced here: this is strictly a
    /// cross-layer accounting and provenance check.
    pub fn validate_and_seal(
        pre_state: &EconomicState,
        post_state: &EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<Self, String> {
        let pre_state_hash = state_hash(pre_state).map_err(|error| error.to_string())?;
        let post_state_hash = state_hash(post_state).map_err(|error| error.to_string())?;
        let transition_hash = transition_hash(transitions).map_err(|error| error.to_string())?;

        let actor_assignments: Vec<(ActorId, EconomicSector)> = assignments
            .iter()
            .map(|assignment| (assignment.actor.clone(), assignment.sector))
            .collect();

        let sector_transaction = SectorTransactionMatrix::from_transitions(
            transitions,
            &actor_assignments,
        )?;
        sector_transaction.validate_against(
            pre_state,
            &actor_assignments,
            transitions,
        )?;

        let pre_balance = SectorBalanceSheet::from_state(pre_state, assignments)?;
        let post_balance = SectorBalanceSheet::from_state(post_state, assignments)?;

        let sector_financial_flow =
            SectorFinancialFlowMatrix::from_transitions(transitions, assignments)?;
        sector_financial_flow.validate_against(
            pre_state,
            assignments,
            transitions,
        )?;
        sector_financial_flow
            .validate_against_balance_sheet_delta(&pre_balance, &post_balance)?;

        let actor_observations =
            ActorEconomicObservables::from_state_and_transitions(pre_state, transitions)?;
        validate_actor_terminal_state(post_state, &actor_observations)?;

        let sector_observations = SectorEconomicObservables::from_state_and_transitions(
            pre_state,
            transitions,
            assignments,
        )?;

        let stock_flow: StockFlowReconciliation =
            reconcile_step(pre_state, post_state, assignments, transitions)?;

        let period_ledger = EconomicPeriodLedger::from_transitions(transitions)?;
        let aggregate_observations =
            EconomicObservables::try_from_state_and_ledger(post_state, &period_ledger)?;
        let aggregate_observations_hash = hash_json(&aggregate_observations)?;

        let actor_observations_hash = hash_json(&actor_observations)?;
        let sector_transaction_hash = hash_json(&sector_transaction)?;
        let sector_financial_flow_hash = hash_json(&sector_financial_flow)?;
        let sector_observations_hash = hash_json(&sector_observations)?;

        let binding = (
            &pre_state_hash,
            &post_state_hash,
            &transition_hash,
            transitions.len() as u64,
            &actor_observations_hash,
            &aggregate_observations_hash,
            &stock_flow.posting_hash,
            stock_flow.posting_count,
            &stock_flow.physical_posting_hash,
            stock_flow.physical_posting_count,
            &sector_transaction_hash,
            &sector_financial_flow_hash,
            &sector_observations_hash,
        );
        let closure_hash = hash_json(&binding)?;

        Ok(Self {
            pre_state_hash,
            post_state_hash,
            transition_hash,
            transition_count: transitions.len() as u64,
            actor_observations_hash,
            aggregate_observations_hash,
            stock_flow_posting_hash: stock_flow.posting_hash,
            stock_flow_posting_count: stock_flow.posting_count,
            physical_posting_hash: stock_flow.physical_posting_hash,
            physical_posting_count: stock_flow.physical_posting_count,
            sector_transaction_hash,
            sector_financial_flow_hash,
            sector_observations_hash,
            closure_hash,
        })
    }

    /// Verify that the receipt's closure hash matches all bound material.
    pub fn verify(&self) -> Result<(), String> {
        let binding = (
            &self.pre_state_hash,
            &self.post_state_hash,
            &self.transition_hash,
            self.transition_count,
            &self.actor_observations_hash,
            &self.aggregate_observations_hash,
            &self.stock_flow_posting_hash,
            self.stock_flow_posting_count,
            &self.physical_posting_hash,
            self.physical_posting_count,
            &self.sector_transaction_hash,
            &self.sector_financial_flow_hash,
            &self.sector_observations_hash,
        );
        let expected = hash_json(&binding)?;
        if self.closure_hash != expected {
            return Err("accounting closure hash mismatch".into());
        }
        Ok(())
    }
}

fn validate_actor_terminal_state(
    post_state: &EconomicState,
    observations: &BTreeMap<ActorId, ActorEconomicObservables>,
) -> Result<(), String> {
    if observations.len() != post_state.actors.len() {
        return Err("actor observations do not cover post-state actors exactly once".into());
    }

    for actor in &post_state.actors {
        let observation = observations
            .get(&actor.actor)
            .ok_or_else(|| format!("missing terminal actor observation for {}", actor.actor))?;

        if observation.cash != actor.monetary.cash
            || observation.deposits != actor.monetary.deposits
            || observation.liquidity
                != actor
                    .monetary
                    .cash
                    .checked_add(actor.monetary.deposits)
                    .ok_or_else(|| format!("terminal liquidity overflow for {}", actor.actor))?
            || observation.loan_claims != actor.monetary.claims
            || observation.debt != actor.monetary.liabilities
            || observation.trade_receivables != actor.monetary.trade_receivables
            || observation.trade_payables != actor.monetary.trade_payables
            || observation.inventory_quantity != actor.real.inventories
            || observation.inventory_carrying_value != actor.inventory_carrying_value
            || observation.productive_capital != actor.real.productive_capital
            || observation.net_financial_position != actor.monetary.try_net_position()?
            || observation.net_worth != actor.try_net_worth()?
        {
            return Err(format!(
                "actor terminal observation does not match post-state for {}",
                actor.actor
            ));
        }
    }

    Ok(())
}

fn hash_json<T: Serialize>(value: &T) -> Result<String, String> {
    let bytes = serde_json::to_vec(value)
        .map_err(|error| format!("failed to serialize accounting closure material: {error}"))?;
    Ok(blake3::hash(&bytes).to_hex().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::sector_balance::SectorAssignment;
    use crate::economics::stock_flow::{
        ActorBalanceSheet, CreditCreation, EconomicState, TradeCreditSale,
        TradeCreditSettlement,
    };
    use crate::economics::transition::{apply_step, EconomicTransition};

    fn assignments() -> Vec<SectorAssignment> {
        vec![
            SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "firm".into(),
                sector: EconomicSector::Firm,
            },
            SectorAssignment {
                actor: "household".into(),
                sector: EconomicSector::Household,
            },
        ]
    }

    fn fixture() -> (
        EconomicState,
        Vec<SectorAssignment>,
        Vec<EconomicTransition>,
        EconomicState,
    ) {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;

        let mut firm = ActorBalanceSheet::new("firm");
        firm.real.inventories = 10;
        firm.inventory_carrying_value = 30;

        let pre = EconomicState::new(vec![
            bank,
            firm,
            ActorBalanceSheet::new("household"),
        ]);

        let transitions = vec![
            EconomicTransition::CreditCreation(
                CreditCreation::new("bank", "household", 100).unwrap(),
            ),
            EconomicTransition::TradeCreditSale(
                TradeCreditSale::new("firm", "household", 1, 30).unwrap(),
            ),
            EconomicTransition::TradeCreditSettlement(
                TradeCreditSettlement::new("firm", "household", 30).unwrap(),
            ),
        ];

        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        (pre, assignments(), transitions, post)
    }

    #[test]
    fn closure_binds_all_accounting_projections_deterministically() {
        let (pre, assignments, transitions, post) = fixture();
        let a = EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .unwrap();
        let b = EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .unwrap();

        assert_eq!(a, b);
        assert!(!a.closure_hash.is_empty());
        assert!(!a.aggregate_observations_hash.is_empty());
        assert!(!a.sector_transaction_hash.is_empty());
        assert!(!a.sector_financial_flow_hash.is_empty());
        assert!(!a.sector_observations_hash.is_empty());
    }

    #[test]
    fn closure_rejects_same_sector_actor_tampering() {
        let (pre, assignments, transitions, mut post) = fixture();
        post.actors
            .iter_mut()
            .find(|actor| actor.actor == "household")
            .unwrap()
            .monetary.deposits = 99;
        post.actors
            .iter_mut()
            .find(|actor| actor.actor == "firm")
            .unwrap()
            .monetary.deposits = 1;

        assert!(EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .is_err());
    }

    #[test]
    fn closure_self_verification_rejects_tampering() {
        let (pre, assignments, transitions, post) = fixture();
        let mut closure = EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .unwrap();
        closure.sector_observations_hash = "tampered".into();
        assert!(closure.verify().is_err());
    }

    #[test]
    fn closure_rejects_tampered_post_state() {
        let (pre, assignments, transitions, mut post) = fixture();
        post.actors
            .iter_mut()
            .find(|actor| actor.actor == "firm")
            .unwrap()
            .monetary.trade_receivables += 1;

        assert!(EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .is_err());
    }

    #[test]
    fn closure_rejects_duplicate_assignments() {
        let (pre, mut assignments, transitions, post) = fixture();
        assignments.push(assignments[0].clone());

        assert!(EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .is_err());
    }
}

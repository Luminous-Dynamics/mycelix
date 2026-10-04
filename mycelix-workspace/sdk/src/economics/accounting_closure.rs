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
use super::sector_other_volume::SectorOtherVolumeChangeMatrix;
use super::sector_revaluation::SectorRevaluationChangeMatrix;
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
    #[serde(default)]
    pub other_volume_change_hash: String,
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

        let other_volume_change =
            SectorOtherVolumeChangeMatrix::from_transitions(transitions, assignments)?;
        let sector_revaluation_change =
            SectorRevaluationChangeMatrix::from_transitions(transitions, assignments)?;
        other_volume_change.validate_against(pre_state, assignments, transitions)?;
        sector_revaluation_change.validate_against(pre_state, assignments, transitions)?;
        other_volume_change
            .validate_against_balance_sheet_delta(&pre_balance, &post_balance)?;

        sector_financial_flow
            .validate_against_balance_sheet_delta_with_other_volume(
                &pre_balance,
                &post_balance,
                &other_volume_change,
            )?;

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
        validate_projection_closure(
            &aggregate_observations,
            &actor_observations,
            &sector_observations,
        )?;
        let aggregate_observations_hash = hash_json(&aggregate_observations)?;

        let actor_observations_hash = hash_json(&actor_observations)?;
        let sector_transaction_hash = hash_json(&sector_transaction)?;
        let sector_financial_flow_hash = hash_json(&sector_financial_flow)?;
        let other_volume_change_hash = hash_json(&other_volume_change)?;
        let sector_revaluation_change_hash = hash_json(&sector_revaluation_change)?;
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
            &other_volume_change_hash,
            &sector_revaluation_change_hash,
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
            other_volume_change_hash,
            sector_revaluation_change_hash,
            sector_observations_hash,
            closure_hash,
        })
    }

    /// Re-derive the closure from the supplied economic artifacts and require exact equality.
    ///
    /// This closes the distinction between a self-consistent serialized receipt and a closure
    /// actually produced by the supplied states, assignments, and ordered transition program.
    pub fn verify_against(
        &self,
        pre_state: &EconomicState,
        post_state: &EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<(), String> {
        self.verify()?;
        let expected = Self::validate_and_seal(
            pre_state,
            post_state,
            assignments,
            transitions,
        )?;
        if self != &expected {
            return Err("accounting closure does not match supplied economic artifacts".into());
        }
        Ok(())
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
            &self.other_volume_change_hash,
            &self.sector_revaluation_change_hash,
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

fn validate_projection_closure(
    aggregate: &EconomicObservables,
    actors: &BTreeMap<ActorId, ActorEconomicObservables>,
    sectors: &BTreeMap<EconomicSector, SectorEconomicObservables>,
) -> Result<(), String> {
    fn sum_actor<F>(
        actors: &BTreeMap<ActorId, ActorEconomicObservables>,
        field: &str,
        select: F,
    ) -> Result<i128, String>
    where
        F: Fn(&ActorEconomicObservables) -> i128,
    {
        actors.values().try_fold(0i128, |total, observation| {
            total
                .checked_add(select(observation))
                .ok_or_else(|| format!("aggregate actor {field} overflow"))
        })
    }

    fn sum_sector<F>(
        sectors: &BTreeMap<EconomicSector, SectorEconomicObservables>,
        field: &str,
        select: F,
    ) -> Result<i128, String>
    where
        F: Fn(&SectorEconomicObservables) -> i128,
    {
        sectors.values().try_fold(0i128, |total, observation| {
            total
                .checked_add(select(observation))
                .ok_or_else(|| format!("aggregate sector {field} overflow"))
        })
    }

    fn require_equal(label: &str, expected: i128, actual: i128) -> Result<(), String> {
        if expected != actual {
            return Err(format!("cross-layer {label} mismatch: expected {expected}, got {actual}"));
        }
        Ok(())
    }

    let actor_checks = [
        ("cash", sum_actor(actors, "cash", |o| o.cash)?, aggregate.aggregate_cash),
        (
            "deposits",
            sum_actor(actors, "deposits", |o| o.deposits)?,
            aggregate.aggregate_deposits,
        ),
        ("loans", sum_actor(actors, "loans", |o| o.loan_claims)?, aggregate.aggregate_loans),
        ("debt", sum_actor(actors, "debt", |o| o.debt)?, aggregate.aggregate_debt),
        (
            "trade receivables",
            sum_actor(actors, "trade receivables", |o| o.trade_receivables)?,
            aggregate.aggregate_trade_receivables,
        ),
        (
            "trade payables",
            sum_actor(actors, "trade payables", |o| o.trade_payables)?,
            aggregate.aggregate_trade_payables,
        ),
        ("liquidity", sum_actor(actors, "liquidity", |o| o.liquidity)?, aggregate.liquidity),
        (
            "working capital",
            sum_actor(actors, "working capital", |o| o.net_working_capital())?,
            aggregate.net_working_capital,
        ),
        (
            "credit created",
            sum_actor(actors, "credit originated", |o| o.credit_originated)?,
            aggregate.credit_created,
        ),
        (
            "debt repaid",
            sum_actor(actors, "debt repaid", |o| o.debt_repaid)?,
            aggregate.debt_repaid,
        ),
        (
            "debt written off as lender",
            sum_actor(actors, "debt written off as lender", |o| o.debt_written_off_as_lender)?,
            aggregate.debt_written_off,
        ),
        (
            "debt written off as borrower",
            sum_actor(actors, "debt written off as borrower", |o| o.debt_written_off_as_borrower)?,
            aggregate.debt_written_off,
        ),
        (
            "trade credit extended",
            sum_actor(actors, "trade credit extended", |o| o.trade_credit_extended)?,
            aggregate.trade_credit_extended,
        ),
        (
            "trade credit settled",
            sum_actor(actors, "trade credit settled", |o| o.trade_credit_settled)?,
            aggregate.trade_credit_settled,
        ),
        (
            "interest paid",
            sum_actor(actors, "interest paid", |o| o.interest_paid)?,
            aggregate.interest_paid,
        ),
        (
            "sales",
            sum_actor(actors, "sales", |o| o.sales_revenue)?,
            aggregate.sales_consideration,
        ),
        (
            "cost of goods sold",
            sum_actor(actors, "cost of goods sold", |o| o.cost_of_goods_sold)?,
            aggregate.cost_of_goods_sold,
        ),
        (
            "depreciation",
            sum_actor(actors, "depreciation", |o| o.depreciation)?,
            aggregate.depreciation,
        ),
        (
            "real-asset revaluation",
            sum_actor(actors, "real-asset revaluation", |o| o.real_asset_revaluation)?,
            aggregate.real_asset_revaluation,
        ),
    ];
    for (label, actor_value, aggregate_value) in actor_checks {
        require_equal(label, actor_value, aggregate_value)?;
    }

    let sector_checks = [
        (
            "sector liquidity",
            sum_sector(sectors, "liquidity", |o| o.closing_liquidity)?,
            aggregate.liquidity,
        ),
        (
            "sector loans",
            sum_sector(sectors, "loans", |o| o.loan_claims)?,
            aggregate.aggregate_loans,
        ),
        (
            "sector debt",
            sum_sector(sectors, "debt", |o| o.debt)?,
            aggregate.aggregate_debt,
        ),
        (
            "sector trade receivables",
            sum_sector(sectors, "trade receivables", |o| o.trade_receivables)?,
            aggregate.aggregate_trade_receivables,
        ),
        (
            "sector trade payables",
            sum_sector(sectors, "trade payables", |o| o.trade_payables)?,
            aggregate.aggregate_trade_payables,
        ),
        (
            "sector working capital",
            sum_sector(sectors, "working capital", |o| o.net_working_capital())?,
            aggregate.net_working_capital,
        ),
        (
            "sector credit originated",
            sum_sector(sectors, "credit originated", |o| o.credit_originated)?,
            aggregate.credit_created,
        ),
        (
            "sector debt repaid",
            sum_sector(sectors, "debt repaid", |o| o.debt_repaid)?,
            aggregate.debt_repaid,
        ),
        (
            "sector debt written off as lender",
            sum_sector(
                sectors,
                "debt written off as lender",
                |o| o.debt_written_off_as_lender,
            )?,
            aggregate.debt_written_off,
        ),
        (
            "sector debt written off as borrower",
            sum_sector(
                sectors,
                "debt written off as borrower",
                |o| o.debt_written_off_as_borrower,
            )?,
            aggregate.debt_written_off,
        ),
        (
            "sector trade credit extended",
            sum_sector(sectors, "trade credit extended", |o| o.trade_credit_extended)?,
            aggregate.trade_credit_extended,
        ),
        (
            "sector trade credit settled",
            sum_sector(sectors, "trade credit settled", |o| o.trade_credit_settled)?,
            aggregate.trade_credit_settled,
        ),
        (
            "sector interest paid",
            sum_sector(sectors, "interest paid", |o| o.interest_paid)?,
            aggregate.interest_paid,
        ),
        (
            "sector sales",
            sum_sector(sectors, "sales", |o| o.sales_revenue)?,
            aggregate.sales_consideration,
        ),
        (
            "sector cost of goods sold",
            sum_sector(sectors, "cost of goods sold", |o| o.cost_of_goods_sold)?,
            aggregate.cost_of_goods_sold,
        ),
        (
            "sector depreciation",
            sum_sector(sectors, "depreciation", |o| o.depreciation)?,
            aggregate.depreciation,
        ),
        (
            "sector real-asset revaluation",
            sum_sector(
                sectors,
                "real-asset revaluation",
                |o| o.real_asset_revaluation,
            )?,
            aggregate.real_asset_revaluation,
        ),
    ];
    for (label, sector_value, aggregate_value) in sector_checks {
        require_equal(label, sector_value, aggregate_value)?;
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
        ActorBalanceSheet, CreditCreation, DebtWriteOff, EconomicState, TradeCreditSale,
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
        assert!(!a.other_volume_change_hash.is_empty());
        assert!(!a.sector_revaluation_change_hash.is_empty());
        assert!(!a.sector_observations_hash.is_empty());
    }

    #[test]
    fn closure_verify_against_rejects_rehashed_tampering() {
        let (pre, assignments, transitions, post) = fixture();
        let mut closure = EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .unwrap();
        closure.post_state_hash = "rehashed-but-wrong".into();
        let binding = (
            &closure.pre_state_hash,
            &closure.post_state_hash,
            &closure.transition_hash,
            closure.transition_count,
            &closure.actor_observations_hash,
            &closure.aggregate_observations_hash,
            &closure.stock_flow_posting_hash,
            closure.stock_flow_posting_count,
            &closure.physical_posting_hash,
            closure.physical_posting_count,
            &closure.sector_transaction_hash,
            &closure.sector_financial_flow_hash,
            &closure.other_volume_change_hash,
            &closure.sector_revaluation_change_hash,
            &closure.sector_observations_hash,
        );
        closure.closure_hash = super::hash_json(&binding).unwrap();

        assert!(closure
            .verify_against(&pre, &post, &assignments, &transitions)
            .is_err());
 
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
    fn mixed_write_off_and_revaluation_share_one_closure_without_overlap() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.claims = 100;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.liabilities = 100;
        firm.real.productive_capital = 100;
        let pre = EconomicState::new(vec![bank, firm]);

        let transitions = vec![
            EconomicTransition::DebtWriteOff(
                DebtWriteOff::new("bank", "firm", 40).unwrap(),
            ),
            EconomicTransition::RealAssetRevaluation(
                crate::economics::stock_flow::RealAssetRevaluation::new(
                    "firm",
                    crate::economics::stock_flow::RealAssetRevaluationTarget::ProductiveCapital,
                    10,
                )
                .unwrap(),
            ),
        ];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        let assignments = vec![
            SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "firm".into(),
                sector: EconomicSector::Firm,
            },
        ];

        let closure = EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .unwrap();
        assert!(!closure.sector_revaluation_change_hash.is_empty());
    }

    #[test]
    fn closure_binds_debt_write_off_counterparts() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.claims = 100;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.liabilities = 100;
        let pre = EconomicState::new(vec![bank, firm]);

        let transitions = vec![EconomicTransition::DebtWriteOff(
            DebtWriteOff::new("bank", "firm", 40).unwrap(),
        )];
        let (post, _) = apply_step(&pre, 1, &transitions, None).unwrap();
        let assignments = vec![
            SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "firm".into(),
                sector: EconomicSector::Firm,
            },
        ];

        let closure = EconomicAccountingClosure::validate_and_seal(
            &pre,
            &post,
            &assignments,
            &transitions,
        )
        .unwrap();

        assert_eq!(post.debt_written_off, 40);
        assert_eq!(closure.transition_count, 1);
        closure
            .verify_against(&pre, &post, &assignments, &transitions)
            .unwrap();
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

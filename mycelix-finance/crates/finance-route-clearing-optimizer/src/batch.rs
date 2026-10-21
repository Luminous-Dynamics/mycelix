#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bounded exact atomic-batch selector for queued gross obligations.
//!
//! Unlike the sequential queue simulator, this model evaluates all selected transfers as
//! one atomic batch: each participant's post-batch balance must be non-negative, but an
//! incoming transfer in the same batch may fund an outgoing transfer. This can settle a
//! closed payment cycle without pre-funding every gross leg. The model requires a
//! caller-side authority qualification assertion and remains a proposal, not a legal
//! netting engine, real rail instruction, or settlement-finality receipt.
//!
//! Exact subset search is deliberately bounded to 16 eligible payment orders (65,536
//! subsets). Larger batches are rejected rather than silently switched to a heuristic.

use super::AuthorityQualification;
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

pub const MAX_EXACT_BATCH_PAYMENTS: usize = 16;
pub const MAX_BATCH_INPUTS: usize = 100_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchContext {
    pub rail_id: String,
    pub asset_id: String,
    pub policy_version: String,
    pub tick: u64,
    /// Caller-side assertion only; this module does not authenticate the evidence.
    pub authority: AuthorityQualification,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchAccount {
    pub participant_id: String,
    pub balance_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchRoute {
    pub route_id: String,
    pub rail_id: String,
    pub asset_id: String,
    pub policy_version: String,
    pub enabled: bool,
    /// Caller-supplied capacity remaining in this route's current settlement window.
    pub remaining_capacity_minor: u64,
    /// Caller-supplied instantaneous availability. This is not externally verified.
    pub available_now: bool,
    /// Modeled fee charged once for a selected payment, not a provider quote.
    pub fixed_fee_units: u64,
    /// Modeled fee per minor unit, not a provider quote.
    pub fee_units_per_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchPayment {
    pub payment_id: String,
    pub rail_id: String,
    pub asset_id: String,
    pub policy_version: String,
    pub route_id: String,
    pub from: String,
    pub to: String,
    pub amount_minor: u64,
    pub arrival_tick: u64,
    pub deadline_tick: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BatchExclusionReason {
    EmptyPaymentId,
    ZeroAmount,
    SelfPayment,
    UnknownParticipant,
    UnknownRoute,
    PaymentProfileMismatch,
    RouteProfileMismatch,
    DisabledRoute,
    RouteUnavailable,
    ZeroRemainingCapacity,
    AmountExceedsRemainingCapacity,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchExclusion {
    pub payment_id: String,
    pub reason: BatchExclusionReason,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchPlan {
    pub rail_id: String,
    pub asset_id: String,
    pub policy_version: String,
    pub tick: u64,
    pub authority_evidence_ref: String,
    /// Sorted IDs selected for atomic settlement at this tick.
    pub selected_payment_ids: Vec<String>,
    /// Eligible payments not selected by the chosen feasible subset.
    pub unselected_eligible_payment_ids: Vec<String>,
    /// Orders arriving after the selected tick; not treated as invalid.
    pub deferred_payment_ids: Vec<String>,
    /// Invalid or currently ineligible input observations retained explicitly.
    pub excluded_payments: Vec<BatchExclusion>,
    /// Post-batch balances, sorted by participant ID.
    pub final_balances: Vec<BatchAccount>,
    pub selected_payment_count: usize,
    /// Sum of full payment amounts in the selected batch, not the net cash movement.
    pub selected_gross_minor: u128,
    pub modeled_fee_units: u128,
    pub late_selected_count: usize,
    /// Eligible payments with deadlines before this tick that remain unselected.
    pub missed_deadline_count: usize,
    /// Number of candidate subsets examined, including the empty batch.
    pub evaluated_subsets: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BatchError {
    InvalidContext(&'static str),
    AuthorityNotQualified,
    InvalidAccount(String),
    DuplicateAccount(String),
    InvalidRoute(String),
    DuplicateRoute(String),
    DuplicatePayment(String),
    TooManyInputs { observed: usize, maximum: usize },
    TooManyEligiblePayments { observed: usize, maximum: usize },
    ArithmeticOverflow,
}

impl fmt::Display for BatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for BatchError {}

fn same_profile(rail: &str, asset: &str, policy: &str, context: &BatchContext) -> bool {
    rail == context.rail_id && asset == context.asset_id && policy == context.policy_version
}

#[derive(Clone, Debug)]
struct Candidate {
    payment: BatchPayment,
    route: BatchRoute,
}

#[derive(Clone, Debug)]
struct FeasibleBatch {
    mask: u64,
    gross_minor: u128,
    fee_units: u128,
    payment_count: usize,
    final_balances: BTreeMap<String, u64>,
    selected_ids: Vec<String>,
}

fn is_better(candidate: &FeasibleBatch, current: &FeasibleBatch) -> bool {
    if candidate.gross_minor != current.gross_minor {
        return candidate.gross_minor > current.gross_minor;
    }
    if candidate.fee_units != current.fee_units {
        return candidate.fee_units < current.fee_units;
    }
    if candidate.payment_count != current.payment_count {
        return candidate.payment_count > current.payment_count;
    }
    candidate.selected_ids < current.selected_ids
}

fn evaluate_subset(
    mask: u64,
    candidates: &[Candidate],
    accounts: &BTreeMap<String, u64>,
) -> Result<Option<FeasibleBatch>, BatchError> {
    let mut outgoing = BTreeMap::<String, u128>::new();
    let mut incoming = BTreeMap::<String, u128>::new();
    let mut route_used = BTreeMap::<String, u128>::new();
    let mut gross_minor = 0_u128;
    let mut fee_units = 0_u128;
    let mut payment_count = 0_usize;
    let mut selected_ids = Vec::new();

    for (index, candidate) in candidates.iter().enumerate() {
        if mask & (1_u64 << index) == 0 {
            continue;
        }
        let payment = &candidate.payment;
        let amount = payment.amount_minor as u128;
        let sent = outgoing.entry(payment.from.clone()).or_default();
        *sent = sent.checked_add(amount).ok_or(BatchError::ArithmeticOverflow)?;
        let received = incoming.entry(payment.to.clone()).or_default();
        *received = received
            .checked_add(amount)
            .ok_or(BatchError::ArithmeticOverflow)?;
        let capacity = route_used.entry(candidate.route.route_id.clone()).or_default();
        *capacity = capacity
            .checked_add(amount)
            .ok_or(BatchError::ArithmeticOverflow)?;

        let variable_fee = amount
            .checked_mul(candidate.route.fee_units_per_minor as u128)
            .ok_or(BatchError::ArithmeticOverflow)?;
        let payment_fee = variable_fee
            .checked_add(candidate.route.fixed_fee_units as u128)
            .ok_or(BatchError::ArithmeticOverflow)?;
        fee_units = fee_units
            .checked_add(payment_fee)
            .ok_or(BatchError::ArithmeticOverflow)?;
        gross_minor = gross_minor
            .checked_add(amount)
            .ok_or(BatchError::ArithmeticOverflow)?;
        payment_count += 1;
        selected_ids.push(payment.payment_id.clone());
    }

    for (route_id, used) in route_used {
        let route = candidates
            .iter()
            .find(|candidate| candidate.route.route_id == route_id)
            .ok_or(BatchError::InvalidRoute(route_id.clone()))?;
        if used > route.route.remaining_capacity_minor as u128 {
            return Ok(None);
        }
    }

    let mut final_balances = BTreeMap::new();
    for (participant, starting_balance) in accounts {
        let received = incoming.get(participant).copied().unwrap_or(0);
        let sent = outgoing.get(participant).copied().unwrap_or(0);
        let available = (*starting_balance as u128)
            .checked_add(received)
            .ok_or(BatchError::ArithmeticOverflow)?;
        if sent > available {
            return Ok(None);
        }
        let final_balance = available - sent;
        if final_balance > u64::MAX as u128 {
            return Ok(None);
        }
        final_balances.insert(participant.clone(), final_balance as u64);
    }

    selected_ids.sort();
    Ok(Some(FeasibleBatch {
        mask,
        gross_minor,
        fee_units,
        payment_count,
        final_balances,
        selected_ids,
    }))
}

/// Select a maximum-gross-value feasible atomic batch, then minimize modeled fees.
///
/// Feasibility is checked on *post-batch* participant balances and route remaining
/// capacities. Credits from selected payments may fund debits within the same batch.
/// Among solutions with equal gross value, the selector minimizes modeled fees, then
/// prefers more payments, then the lexicographically smallest sorted payment-ID set.
/// Orders that have not arrived are deferred; unusable input observations are preserved
/// as exclusions. Eligible instances above 16 payments fail closed.
pub fn select_atomic_batch(
    context: &BatchContext,
    accounts: &[BatchAccount],
    routes: &[BatchRoute],
    payments: &[BatchPayment],
) -> Result<BatchPlan, BatchError> {
    for (name, value) in [
        ("rail_id", context.rail_id.as_str()),
        ("asset_id", context.asset_id.as_str()),
        ("policy_version", context.policy_version.as_str()),
    ] {
        if value.trim().is_empty() {
            return Err(BatchError::InvalidContext(name));
        }
    }

    let authority_evidence_ref = match &context.authority {
        AuthorityQualification::NotQualified => return Err(BatchError::AuthorityNotQualified),
        AuthorityQualification::ExternallyQualified { evidence_ref }
            if evidence_ref.trim().is_empty() =>
        {
            return Err(BatchError::InvalidContext("authority evidence reference"));
        }
        AuthorityQualification::ExternallyQualified { evidence_ref } => evidence_ref.clone(),
    };

    if payments.len() > MAX_BATCH_INPUTS {
        return Err(BatchError::TooManyInputs {
            observed: payments.len(),
            maximum: MAX_BATCH_INPUTS,
        });
    }

    let mut account_map = BTreeMap::<String, u64>::new();
    for account in accounts {
        if account.participant_id.trim().is_empty() {
            return Err(BatchError::InvalidAccount("empty participant ID".into()));
        }
        if account_map
            .insert(account.participant_id.clone(), account.balance_minor)
            .is_some()
        {
            return Err(BatchError::DuplicateAccount(account.participant_id.clone()));
        }
    }

    let mut route_map = BTreeMap::<String, BatchRoute>::new();
    for route in routes {
        if route.route_id.trim().is_empty()
            || route.rail_id.trim().is_empty()
            || route.asset_id.trim().is_empty()
            || route.policy_version.trim().is_empty()
        {
            return Err(BatchError::InvalidRoute(route.route_id.clone()));
        }
        if route_map
            .insert(route.route_id.clone(), route.clone())
            .is_some()
        {
            return Err(BatchError::DuplicateRoute(route.route_id.clone()));
        }
    }

    let mut seen_ids = BTreeSet::new();
    let mut eligible = Vec::<Candidate>::new();
    let mut deferred_payment_ids = Vec::new();
    let mut excluded_payments = Vec::new();

    for payment in payments {
        if !seen_ids.insert(payment.payment_id.clone()) {
            return Err(BatchError::DuplicatePayment(payment.payment_id.clone()));
        }
        if payment.arrival_tick > context.tick {
            deferred_payment_ids.push(payment.payment_id.clone());
            continue;
        }

        let reason = if payment.payment_id.trim().is_empty() {
            Some(BatchExclusionReason::EmptyPaymentId)
        } else if payment.amount_minor == 0 {
            Some(BatchExclusionReason::ZeroAmount)
        } else if payment.from == payment.to {
            Some(BatchExclusionReason::SelfPayment)
        } else if !same_profile(
            &payment.rail_id,
            &payment.asset_id,
            &payment.policy_version,
            context,
        ) {
            Some(BatchExclusionReason::PaymentProfileMismatch)
        } else if !account_map.contains_key(&payment.from)
            || !account_map.contains_key(&payment.to)
        {
            Some(BatchExclusionReason::UnknownParticipant)
        } else if let Some(route) = route_map.get(&payment.route_id) {
            if !same_profile(&route.rail_id, &route.asset_id, &route.policy_version, context) {
                Some(BatchExclusionReason::RouteProfileMismatch)
            } else if !route.enabled {
                Some(BatchExclusionReason::DisabledRoute)
            } else if !route.available_now {
                Some(BatchExclusionReason::RouteUnavailable)
            } else if route.remaining_capacity_minor == 0 {
                Some(BatchExclusionReason::ZeroRemainingCapacity)
            } else if payment.amount_minor > route.remaining_capacity_minor {
                Some(BatchExclusionReason::AmountExceedsRemainingCapacity)
            } else {
                eligible.push(Candidate {
                    payment: payment.clone(),
                    route: route.clone(),
                });
                None
            }
        } else {
            Some(BatchExclusionReason::UnknownRoute)
        };

        if let Some(reason) = reason {
            excluded_payments.push(BatchExclusion {
                payment_id: payment.payment_id.clone(),
                reason,
            });
        }
    }

    if eligible.len() > MAX_EXACT_BATCH_PAYMENTS {
        return Err(BatchError::TooManyEligiblePayments {
            observed: eligible.len(),
            maximum: MAX_EXACT_BATCH_PAYMENTS,
        });
    }
    eligible.sort_by(|a, b| a.payment.payment_id.cmp(&b.payment.payment_id));
    deferred_payment_ids.sort();
    excluded_payments.sort_by(|a, b| a.payment_id.cmp(&b.payment_id));

    let subset_count = 1_u64
        .checked_shl(eligible.len() as u32)
        .ok_or(BatchError::ArithmeticOverflow)?;
    let mut best: Option<FeasibleBatch> = None;
    for mask in 0..subset_count {
        let Some(candidate) = evaluate_subset(mask, &eligible, &account_map)? else {
            continue;
        };
        if best.as_ref().is_none_or(|current| is_better(&candidate, current)) {
            best = Some(candidate);
        }
    }
    let best = best.ok_or(BatchError::ArithmeticOverflow)?;

    let selected_mask = best.mask;
    let mut unselected_eligible_payment_ids = eligible
        .iter()
        .enumerate()
        .filter(|(index, _)| selected_mask & (1_u64 << index) == 0)
        .map(|(_, candidate)| candidate.payment.payment_id.clone())
        .collect::<Vec<_>>();
    unselected_eligible_payment_ids.sort();

    let late_selected_count = eligible
        .iter()
        .enumerate()
        .filter(|(index, candidate)| {
            selected_mask & (1_u64 << index) != 0
                && candidate
                    .payment
                    .deadline_tick
                    .is_some_and(|deadline| context.tick > deadline)
        })
        .count();
    let missed_deadline_count = eligible
        .iter()
        .enumerate()
        .filter(|(index, candidate)| {
            selected_mask & (1_u64 << index) == 0
                && candidate
                    .payment
                    .deadline_tick
                    .is_some_and(|deadline| context.tick > deadline)
        })
        .count();

    Ok(BatchPlan {
        rail_id: context.rail_id.clone(),
        asset_id: context.asset_id.clone(),
        policy_version: context.policy_version.clone(),
        tick: context.tick,
        authority_evidence_ref,
        selected_payment_ids: best.selected_ids,
        unselected_eligible_payment_ids,
        deferred_payment_ids,
        excluded_payments,
        final_balances: best
            .final_balances
            .into_iter()
            .map(|(participant_id, balance_minor)| BatchAccount {
                participant_id,
                balance_minor,
            })
            .collect(),
        selected_payment_count: best.payment_count,
        selected_gross_minor: best.gross_minor,
        modeled_fee_units: best.fee_units,
        late_selected_count,
        missed_deadline_count,
        evaluated_subsets: subset_count,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context(tick: u64) -> BatchContext {
        BatchContext {
            rail_id: "synthetic-rtgs".into(),
            asset_id: "zar-minor".into(),
            policy_version: "batch-v1".into(),
            tick,
            authority: AuthorityQualification::ExternallyQualified {
                evidence_ref: "fixture-only://not-legal-evidence".into(),
            },
        }
    }

    fn account(id: &str, balance_minor: u64) -> BatchAccount {
        BatchAccount {
            participant_id: id.into(),
            balance_minor,
        }
    }

    fn route(id: &str, capacity: u64, available: bool, fixed_fee: u64, variable_fee: u64) -> BatchRoute {
        BatchRoute {
            route_id: id.into(),
            rail_id: "synthetic-rtgs".into(),
            asset_id: "zar-minor".into(),
            policy_version: "batch-v1".into(),
            enabled: true,
            remaining_capacity_minor: capacity,
            available_now: available,
            fixed_fee_units: fixed_fee,
            fee_units_per_minor: variable_fee,
        }
    }

    fn payment(id: &str, route_id: &str, from: &str, to: &str, amount: u64) -> BatchPayment {
        BatchPayment {
            payment_id: id.into(),
            rail_id: "synthetic-rtgs".into(),
            asset_id: "zar-minor".into(),
            policy_version: "batch-v1".into(),
            route_id: route_id.into(),
            from: from.into(),
            to: to.into(),
            amount_minor: amount,
            arrival_tick: 0,
            deadline_tick: None,
        }
    }

    #[test]
    fn atomic_cycle_clears_without_prefunding_each_gross_leg() {
        let accounts = [account("A", 0), account("B", 0), account("C", 0)];
        let routes = [
            route("ab", 5, true, 1, 1),
            route("bc", 5, true, 1, 1),
            route("ca", 5, true, 1, 1),
        ];
        let payments = [
            payment("p-ab", "ab", "A", "B", 5),
            payment("p-bc", "bc", "B", "C", 5),
            payment("p-ca", "ca", "C", "A", 5),
        ];
        let plan = select_atomic_batch(&context(0), &accounts, &routes, &payments).unwrap();

        assert_eq!(plan.selected_payment_count, 3);
        assert_eq!(plan.selected_gross_minor, 15);
        assert_eq!(plan.final_balances.iter().map(|a| a.balance_minor).sum::<u64>(), 0);
        assert_eq!(plan.selected_payment_ids, vec!["p-ab", "p-bc", "p-ca"]);
    }

    #[test]
    fn non_clearing_cycle_is_not_selected_when_it_would_overdraw_a_member() {
        let accounts = [account("A", 0), account("B", 0)];
        let routes = [route("ab", 5, true, 0, 0), route("ba", 5, true, 0, 0)];
        let payments = [
            payment("p-ab", "ab", "A", "B", 5),
            payment("p-ba", "ba", "B", "A", 4),
        ];
        let plan = select_atomic_batch(&context(0), &accounts, &routes, &payments).unwrap();
        assert_eq!(plan.selected_payment_count, 0);
        assert_eq!(plan.selected_gross_minor, 0);
        assert_eq!(plan.unselected_eligible_payment_ids.len(), 2);
    }

    #[test]
    fn maximizes_gross_value_then_minimizes_modeled_fees() {
        let accounts = [account("A", 5), account("B", 5), account("C", 0)];
        let routes = [route("a-c", 5, true, 0, 5), route("b-c", 5, true, 0, 1)];
        let payments = [
            payment("expensive", "a-c", "A", "C", 5),
            payment("cheap", "b-c", "B", "C", 5),
        ];
        let plan = select_atomic_batch(&context(0), &accounts, &routes, &payments).unwrap();
        assert_eq!(plan.selected_gross_minor, 5);
        assert_eq!(plan.modeled_fee_units, 5);
        assert_eq!(plan.selected_payment_ids, vec!["cheap"]);
    }

    #[test]
    fn remaining_route_capacity_is_enforced_across_the_whole_batch() {
        let accounts = [account("A", 10), account("B", 0), account("C", 0)];
        let routes = [route("shared", 6, true, 0, 1)];
        let payments = [
            payment("p1", "shared", "A", "B", 4),
            payment("p2", "shared", "A", "C", 4),
        ];
        let plan = select_atomic_batch(&context(0), &accounts, &routes, &payments).unwrap();
        assert_eq!(plan.selected_gross_minor, 4);
        assert_eq!(plan.selected_payment_count, 1);
        assert_eq!(plan.selected_payment_ids, vec!["p1"]);
    }

    #[test]
    fn unqualified_authority_fails_closed() {
        let mut ctx = context(0);
        ctx.authority = AuthorityQualification::NotQualified;
        let result = select_atomic_batch(
            &ctx,
            &[account("A", 1), account("B", 0)],
            &[route("ab", 1, true, 0, 1)],
            &[payment("p", "ab", "A", "B", 1)],
        );
        assert_eq!(result, Err(BatchError::AuthorityNotQualified));
    }

    #[test]
    fn future_payments_are_deferred_and_profile_mismatches_are_retained() {
        let mut future = payment("future", "ab", "A", "B", 1);
        future.arrival_tick = 2;
        let mut wrong = payment("wrong-profile", "ab", "A", "B", 1);
        wrong.asset_id = "other-asset".into();
        let plan = select_atomic_batch(
            &context(0),
            &[account("A", 3), account("B", 0)],
            &[route("ab", 3, true, 0, 0)],
            &[future, wrong],
        )
        .unwrap();

        assert_eq!(plan.deferred_payment_ids, vec!["future"]);
        assert_eq!(plan.excluded_payments.len(), 1);
        assert_eq!(
            plan.excluded_payments[0].reason,
            BatchExclusionReason::PaymentProfileMismatch
        );
    }

    #[test]
    fn exact_batch_limit_fails_closed() {
        let accounts = [account("A", 100), account("B", 0)];
        let routes = [route("ab", 100, true, 0, 0)];
        let payments = (0..=MAX_EXACT_BATCH_PAYMENTS)
            .map(|i| payment(&format!("p{i:02}"), "ab", "A", "B", 1))
            .collect::<Vec<_>>();
        let result = select_atomic_batch(&context(0), &accounts, &routes, &payments);
        assert_eq!(
            result,
            Err(BatchError::TooManyEligiblePayments {
                observed: MAX_EXACT_BATCH_PAYMENTS + 1,
                maximum: MAX_EXACT_BATCH_PAYMENTS,
            })
        );
    }
}

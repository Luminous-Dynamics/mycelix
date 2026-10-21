#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deterministic gross-payment queue simulation, kept separate from static clearing.
//! This models integer balances, arrivals, capacity windows, route outages and deadlines.
//! It is a research simulator: fees are modeled, not provider quotes; outcomes are not
//! rail instructions, legal netting, settlement receipts or evidence of finality.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

pub const MAX_QUEUE_EVENTS: usize = 100_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueueContext {
    pub rail_id: String,
    pub asset_id: String,
    pub policy_version: String,
    /// Inclusive final tick in this deterministic simulation.
    pub horizon_tick: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueueAccount {
    pub participant_id: String,
    pub balance_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct TickWindow {
    pub start_tick: u64,
    /// Half-open interval [start_tick, end_tick).
    pub end_tick: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueueRoute {
    pub route_id: String,
    pub rail_id: String,
    pub asset_id: String,
    pub policy_version: String,
    pub enabled: bool,
    pub capacity_minor_per_window: u64,
    /// Capacity resets at 0, N, 2N, ...; must be greater than zero.
    pub capacity_window_ticks: u64,
    pub fixed_fee_units: u64,
    pub fee_units_per_minor: u64,
    pub outages: Vec<TickWindow>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PaymentOrder {
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
    /// Larger values run first under PriorityScan.
    pub priority: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueuePolicy {
    /// A blocked head prevents later queue entries from being attempted at that tick.
    StrictFifo,
    /// FIFO order, but skip blocked payments and retry after any successful settlement.
    LiquidityAwareFifo,
    /// Higher priority first, then arrival time and ID; retries after successful settlement.
    PriorityScan,
    /// Priority increases by one level per configured interval, then arrival time and ID.
    /// The interval must be positive; different intervals expose a fairness/latency trade-off.
    AgingPriorityScan { interval_ticks: u64 },
    /// Earliest explicit deadline first (missing deadlines last), then arrival time and ID.
    /// This is a simple heuristic, not a globally optimal deadline schedule.
    EarliestDeadlineFirst,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExclusionReason {
    EmptyPaymentId,
    InvalidAmount,
    SelfPayment,
    UnknownParticipant,
    UnknownRoute,
    PaymentProfileMismatch,
    RouteProfileMismatch,
    DisabledRoute,
    ZeroCapacity,
    AmountExceedsWindowCapacity,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BlockReason {
    InsufficientLiquidity,
    RouteUnavailable,
    CapacityExhausted,
    HeadOfLineBlocked,
    NotArrivedByHorizon,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PaymentState {
    Excluded {
        reason: ExclusionReason,
    },
    Settled {
        tick: u64,
        fee_units: u128,
        late: bool,
    },
    Unsettled {
        reason: BlockReason,
        deadline_missed: bool,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PaymentResult {
    pub payment_id: String,
    pub amount_minor: u64,
    pub state: PaymentState,
    /// Settled wait or horizon-censored age for eligible unresolved payments.
    /// Exclusions and payments not yet arrived have no wait sample.
    pub queue_wait_ticks: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AccountMetric {
    pub participant_id: String,
    pub starting_balance_minor: u64,
    pub final_balance_minor: u64,
    pub minimum_observed_balance_minor: u64,
    pub maximum_observed_balance_minor: u64,
    /// Balance integrated over time, measured in minor-unit ticks.
    /// This is a state metric, not a standalone liquidity-savings measure.
    pub balance_time_minor_ticks: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueueResult {
    pub context: QueueContext,
    pub policy: QueuePolicy,
    pub final_balances: Vec<QueueAccount>,
    /// Per-participant balance and balance-time metrics, sorted by participant ID.
    pub account_metrics: Vec<AccountMetric>,
    /// Results are sorted by payment ID for deterministic reporting.
    pub payments: Vec<PaymentResult>,
    pub settled_count: usize,
    pub unsettled_count: usize,
    pub excluded_count: usize,
    pub late_settled_count: usize,
    pub missed_deadline_count: usize,
    pub settled_minor: u128,
    pub modeled_fee_units: u128,
    /// Sum of observed and horizon-censored eligible-payment wait ticks.
    pub total_queue_wait_ticks: u128,
    /// Nearest-rank p95 over settled wait and horizon-censored unresolved age samples.
    pub p95_queue_wait_ticks: Option<u64>,
    pub max_queue_wait_ticks: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QueueError {
    InvalidContext(&'static str),
    InvalidPolicy(&'static str),
    InvalidAccount(String),
    DuplicateAccount(String),
    InvalidRoute(String),
    DuplicateRoute(String),
    DuplicatePayment(String),
    InvalidOutage {
        route_id: String,
        start: u64,
        end: u64,
    },
    TooManyEvents {
        observed_at_least: usize,
        maximum: usize,
    },
    ArithmeticOverflow,
}

impl fmt::Display for QueueError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for QueueError {}

fn matches_profile(rail: &str, asset: &str, policy: &str, context: &QueueContext) -> bool {
    rail == context.rail_id && asset == context.asset_id && policy == context.policy_version
}

fn route_unavailable(route: &QueueRoute, tick: u64) -> bool {
    route
        .outages
        .iter()
        .any(|w| w.start_tick <= tick && tick < w.end_tick)
}

fn add_cost(amount: u64, route: &QueueRoute) -> Result<u128, QueueError> {
    (amount as u128)
        .checked_mul(route.fee_units_per_minor as u128)
        .and_then(|variable| variable.checked_add(route.fixed_fee_units as u128))
        .ok_or(QueueError::ArithmeticOverflow)
}

/// Replay the supplied gross payments using a pinned profile, policy and finite horizon.
///
/// Account transfers are gross and conserve value: sender debit equals receiver credit.
/// Fees are reported separately and are not debited from participant balances. Deadlines
/// are targets, not expiry rules: a late payment may settle and is labeled late. Eligible
/// payments not settled by the horizon remain visible as unsettled; invalid or unknown
/// inputs remain visible as exclusions. This function does not net, submit or settle real
/// payments, and a modeled fee is not a quoted rail fee.
pub fn simulate_queue(
    context: &QueueContext,
    accounts: &[QueueAccount],
    routes: &[QueueRoute],
    payments: &[PaymentOrder],
    policy: QueuePolicy,
) -> Result<QueueResult, QueueError> {
    if matches!(policy, QueuePolicy::AgingPriorityScan { interval_ticks: 0 }) {
        return Err(QueueError::InvalidPolicy(
            "priority aging interval must be greater than zero",
        ));
    }

    for (name, value) in [
        ("rail_id", context.rail_id.as_str()),
        ("asset_id", context.asset_id.as_str()),
        ("policy_version", context.policy_version.as_str()),
    ] {
        if value.trim().is_empty() {
            return Err(QueueError::InvalidContext(name));
        }
    }

    let mut balances = BTreeMap::<String, u64>::new();
    for account in accounts {
        if account.participant_id.trim().is_empty() {
            return Err(QueueError::InvalidAccount("empty participant ID".into()));
        }
        if balances
            .insert(account.participant_id.clone(), account.balance_minor)
            .is_some()
        {
            return Err(QueueError::DuplicateAccount(account.participant_id.clone()));
        }
    }

    let starting_balances = balances.clone();
    let mut minimum_balances = balances.clone();
    let mut maximum_balances = balances.clone();
    let mut balance_time = balances
        .keys()
        .map(|participant| (participant.clone(), 0_u128))
        .collect::<BTreeMap<_, _>>();

    let mut route_map = BTreeMap::<String, QueueRoute>::new();
    for route in routes {
        if route.route_id.trim().is_empty() || route.capacity_window_ticks == 0 {
            return Err(QueueError::InvalidRoute(route.route_id.clone()));
        }
        let mut normalized = route.clone();
        normalized.outages.sort();
        for window in &normalized.outages {
            if window.start_tick >= window.end_tick {
                return Err(QueueError::InvalidOutage {
                    route_id: route.route_id.clone(),
                    start: window.start_tick,
                    end: window.end_tick,
                });
            }
        }
        if route_map
            .insert(normalized.route_id.clone(), normalized)
            .is_some()
        {
            return Err(QueueError::DuplicateRoute(route.route_id.clone()));
        }
        if route_map.len() > MAX_QUEUE_EVENTS {
            return Err(QueueError::TooManyEvents {
                observed_at_least: route_map.len(),
                maximum: MAX_QUEUE_EVENTS,
            });
        }
    }
    let route_list = route_map.into_values().collect::<Vec<_>>();
    let route_indexes = route_list
        .iter()
        .enumerate()
        .map(|(i, r)| (r.route_id.clone(), i))
        .collect::<BTreeMap<_, _>>();

    let mut ids = BTreeSet::new();
    let mut states = vec![None::<PaymentState>; payments.len()];
    let mut last_block = vec![None::<BlockReason>; payments.len()];
    let mut eligible_route = vec![None::<usize>; payments.len()];
    let mut events = BTreeSet::from([0_u64, context.horizon_tick]);

    for (i, payment) in payments.iter().enumerate() {
        if !ids.insert(payment.payment_id.clone()) {
            return Err(QueueError::DuplicatePayment(payment.payment_id.clone()));
        }
        if payment.arrival_tick <= context.horizon_tick {
            events.insert(payment.arrival_tick);
        }
        let reason = if payment.payment_id.trim().is_empty() {
            Some(ExclusionReason::EmptyPaymentId)
        } else if payment.amount_minor == 0 {
            Some(ExclusionReason::InvalidAmount)
        } else if payment.from == payment.to {
            Some(ExclusionReason::SelfPayment)
        } else if !matches_profile(
            &payment.rail_id,
            &payment.asset_id,
            &payment.policy_version,
            context,
        ) {
            Some(ExclusionReason::PaymentProfileMismatch)
        } else if !balances.contains_key(&payment.from) || !balances.contains_key(&payment.to) {
            Some(ExclusionReason::UnknownParticipant)
        } else if let Some(route_index) = route_indexes.get(&payment.route_id).copied() {
            let route = &route_list[route_index];
            if !matches_profile(
                &route.rail_id,
                &route.asset_id,
                &route.policy_version,
                context,
            ) {
                Some(ExclusionReason::RouteProfileMismatch)
            } else if !route.enabled {
                Some(ExclusionReason::DisabledRoute)
            } else if route.capacity_minor_per_window == 0 {
                Some(ExclusionReason::ZeroCapacity)
            } else if payment.amount_minor > route.capacity_minor_per_window {
                Some(ExclusionReason::AmountExceedsWindowCapacity)
            } else {
                eligible_route[i] = Some(route_index);
                None
            }
        } else {
            Some(ExclusionReason::UnknownRoute)
        };
        if let Some(reason) = reason {
            states[i] = Some(PaymentState::Excluded { reason });
        }
        if events.len() > MAX_QUEUE_EVENTS {
            return Err(QueueError::TooManyEvents {
                observed_at_least: events.len(),
                maximum: MAX_QUEUE_EVENTS,
            });
        }
    }

    for route in &route_list {
        for window in &route.outages {
            if window.start_tick <= context.horizon_tick {
                events.insert(window.start_tick);
            }
            if window.end_tick <= context.horizon_tick {
                events.insert(window.end_tick);
            }
        }
        let last_bucket = context.horizon_tick / route.capacity_window_ticks;
        if last_bucket > MAX_QUEUE_EVENTS as u64 {
            return Err(QueueError::TooManyEvents {
                observed_at_least: MAX_QUEUE_EVENTS + 1,
                maximum: MAX_QUEUE_EVENTS,
            });
        }
        for bucket in 0..=last_bucket {
            events.insert(bucket * route.capacity_window_ticks);
            if events.len() > MAX_QUEUE_EVENTS {
                return Err(QueueError::TooManyEvents {
                    observed_at_least: events.len(),
                    maximum: MAX_QUEUE_EVENTS,
                });
            }
        }
    }

    let mut used = BTreeMap::<(String, u64), u64>::new();
    let mut settled_minor = 0_u128;
    let mut total_fee = 0_u128;
    let mut previous_tick = 0_u64;

    for tick in events.into_iter().filter(|t| *t <= context.horizon_tick) {
        let elapsed = (tick - previous_tick) as u128;
        if elapsed > 0 {
            for (participant, balance) in &balances {
                let contribution = (*balance as u128)
                    .checked_mul(elapsed)
                    .ok_or(QueueError::ArithmeticOverflow)?;
                let accumulated = balance_time.get(participant).copied().unwrap_or(0);
                balance_time.insert(
                    participant.clone(),
                    accumulated
                        .checked_add(contribution)
                        .ok_or(QueueError::ArithmeticOverflow)?,
                );
            }
        }
        previous_tick = tick;

        loop {
            let mut pending = (0..payments.len())
                .filter(|i| {
                    states[*i].is_none()
                        && eligible_route[*i].is_some()
                        && payments[*i].arrival_tick <= tick
                })
                .collect::<Vec<_>>();

            match policy {
                QueuePolicy::PriorityScan => pending.sort_by(|a, b| {
                    payments[*b]
                        .priority
                        .cmp(&payments[*a].priority)
                        .then_with(|| payments[*a].arrival_tick.cmp(&payments[*b].arrival_tick))
                        .then_with(|| payments[*a].payment_id.cmp(&payments[*b].payment_id))
                }),
                QueuePolicy::EarliestDeadlineFirst => pending.sort_by(|a, b| {
                    payments[*a]
                        .deadline_tick
                        .unwrap_or(u64::MAX)
                        .cmp(&payments[*b].deadline_tick.unwrap_or(u64::MAX))
                        .then_with(|| payments[*a].arrival_tick.cmp(&payments[*b].arrival_tick))
                        .then_with(|| payments[*a].payment_id.cmp(&payments[*b].payment_id))
                }),
                QueuePolicy::AgingPriorityScan { interval_ticks } => pending.sort_by(|a, b| {
                    let priority_a = (payments[*a].priority as u64).saturating_add(
                        tick.saturating_sub(payments[*a].arrival_tick) / interval_ticks,
                    );
                    let priority_b = (payments[*b].priority as u64).saturating_add(
                        tick.saturating_sub(payments[*b].arrival_tick) / interval_ticks,
                    );
                    priority_b
                        .cmp(&priority_a)
                        .then_with(|| payments[*a].arrival_tick.cmp(&payments[*b].arrival_tick))
                        .then_with(|| payments[*a].payment_id.cmp(&payments[*b].payment_id))
                }),
                QueuePolicy::StrictFifo | QueuePolicy::LiquidityAwareFifo => {
                    pending.sort_by(|a, b| {
                        payments[*a]
                            .arrival_tick
                            .cmp(&payments[*b].arrival_tick)
                            .then_with(|| payments[*a].payment_id.cmp(&payments[*b].payment_id))
                    })
                }
            }
            if pending.is_empty() {
                break;
            }

            let mut progress = false;
            let mut strict_block = false;
            for (slot, i) in pending.iter().copied().enumerate() {
                let payment = &payments[i];
                let route = &route_list[eligible_route[i].expect("eligible route")];
                let bucket = tick / route.capacity_window_ticks;
                let key = (route.route_id.clone(), bucket);
                let current_used = used.get(&key).copied().unwrap_or(0);

                let blocked = if route_unavailable(route, tick) {
                    Some(BlockReason::RouteUnavailable)
                } else if payment.amount_minor
                    > route.capacity_minor_per_window.saturating_sub(current_used)
                {
                    Some(BlockReason::CapacityExhausted)
                } else if balances[&payment.from] < payment.amount_minor {
                    Some(BlockReason::InsufficientLiquidity)
                } else {
                    None
                };

                if let Some(reason) = blocked {
                    last_block[i] = Some(reason);
                    if policy == QueuePolicy::StrictFifo {
                        strict_block = true;
                        for blocked_i in pending.iter().skip(slot + 1).copied() {
                            last_block[blocked_i] = Some(BlockReason::HeadOfLineBlocked);
                        }
                        break;
                    }
                    continue;
                }

                let sender_balance = balances[&payment.from];
                let receiver_balance = balances[&payment.to];
                let receiver_after = receiver_balance
                    .checked_add(payment.amount_minor)
                    .ok_or(QueueError::ArithmeticOverflow)?;
                let used_after = current_used
                    .checked_add(payment.amount_minor)
                    .ok_or(QueueError::ArithmeticOverflow)?;
                let fee = add_cost(payment.amount_minor, route)?;
                let settled_after = settled_minor
                    .checked_add(payment.amount_minor as u128)
                    .ok_or(QueueError::ArithmeticOverflow)?;
                let fee_after = total_fee
                    .checked_add(fee)
                    .ok_or(QueueError::ArithmeticOverflow)?;

                // Apply state only after all arithmetic checks succeed.
                let sender_after = sender_balance - payment.amount_minor;
                balances.insert(payment.from.clone(), sender_after);
                balances.insert(payment.to.clone(), receiver_after);
                minimum_balances
                    .entry(payment.from.clone())
                    .and_modify(|v| *v = (*v).min(sender_after));
                minimum_balances
                    .entry(payment.to.clone())
                    .and_modify(|v| *v = (*v).min(receiver_after));
                maximum_balances
                    .entry(payment.from.clone())
                    .and_modify(|v| *v = (*v).max(sender_after));
                maximum_balances
                    .entry(payment.to.clone())
                    .and_modify(|v| *v = (*v).max(receiver_after));
                used.insert(key, used_after);
                settled_minor = settled_after;
                total_fee = fee_after;
                states[i] = Some(PaymentState::Settled {
                    tick,
                    fee_units: fee,
                    late: payment.deadline_tick.is_some_and(|d| tick > d),
                });
                last_block[i] = None;
                progress = true;
            }

            if policy == QueuePolicy::StrictFifo || !progress || strict_block {
                break;
            }
        }
    }

    let mut result_payments = payments
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let state = states[i].clone().unwrap_or_else(|| {
                let reason = if p.arrival_tick > context.horizon_tick {
                    BlockReason::NotArrivedByHorizon
                } else {
                    last_block[i]
                        .clone()
                        .unwrap_or(BlockReason::InsufficientLiquidity)
                };
                PaymentState::Unsettled {
                    reason,
                    deadline_missed: p.deadline_tick.is_some_and(|d| d <= context.horizon_tick),
                }
            });
            let queue_wait_ticks = match &state {
                PaymentState::Settled { tick, .. } => Some((*tick).saturating_sub(p.arrival_tick)),
                PaymentState::Unsettled { .. }
                    if eligible_route[i].is_some() && p.arrival_tick <= context.horizon_tick =>
                {
                    Some(context.horizon_tick.saturating_sub(p.arrival_tick))
                }
                PaymentState::Excluded { .. } | PaymentState::Unsettled { .. } => None,
            };
            PaymentResult {
                payment_id: p.payment_id.clone(),
                amount_minor: p.amount_minor,
                state,
                queue_wait_ticks,
            }
        })
        .collect::<Vec<_>>();
    result_payments.sort_by(|a, b| a.payment_id.cmp(&b.payment_id));

    let account_metrics = starting_balances
        .iter()
        .map(|(participant_id, starting_balance_minor)| AccountMetric {
            participant_id: participant_id.clone(),
            starting_balance_minor: *starting_balance_minor,
            final_balance_minor: balances[participant_id],
            minimum_observed_balance_minor: minimum_balances[participant_id],
            maximum_observed_balance_minor: maximum_balances[participant_id],
            balance_time_minor_ticks: balance_time[participant_id],
        })
        .collect::<Vec<_>>();
    let final_balances = balances
        .into_iter()
        .map(|(participant_id, balance_minor)| QueueAccount {
            participant_id,
            balance_minor,
        })
        .collect::<Vec<_>>();
    let settled_count = result_payments
        .iter()
        .filter(|p| matches!(p.state, PaymentState::Settled { .. }))
        .count();
    let unsettled_count = result_payments
        .iter()
        .filter(|p| matches!(p.state, PaymentState::Unsettled { .. }))
        .count();
    let excluded_count = result_payments
        .iter()
        .filter(|p| matches!(p.state, PaymentState::Excluded { .. }))
        .count();
    let late_settled_count = result_payments
        .iter()
        .filter(|p| matches!(p.state, PaymentState::Settled { late: true, .. }))
        .count();
    let missed_deadline_count = result_payments
        .iter()
        .filter(|p| match p.state {
            PaymentState::Settled { late, .. } => late,
            PaymentState::Unsettled {
                deadline_missed, ..
            } => deadline_missed,
            PaymentState::Excluded { .. } => false,
        })
        .count();
    let mut wait_samples = result_payments
        .iter()
        .filter_map(|payment| payment.queue_wait_ticks)
        .collect::<Vec<_>>();
    let total_queue_wait_ticks = wait_samples
        .iter()
        .try_fold(0_u128, |total, wait| total.checked_add(*wait as u128))
        .ok_or(QueueError::ArithmeticOverflow)?;
    wait_samples.sort_unstable();
    let max_queue_wait_ticks = wait_samples.last().copied();
    let p95_queue_wait_ticks = if wait_samples.is_empty() {
        None
    } else {
        let rank = ((wait_samples.len() - 1) * 95).div_ceil(100);
        wait_samples.get(rank).copied()
    };

    Ok(QueueResult {
        context: context.clone(),
        policy,
        final_balances,
        account_metrics,
        payments: result_payments,
        settled_count,
        unsettled_count,
        excluded_count,
        late_settled_count,
        missed_deadline_count,
        settled_minor,
        modeled_fee_units: total_fee,
        total_queue_wait_ticks,
        p95_queue_wait_ticks,
        max_queue_wait_ticks,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context(horizon_tick: u64) -> QueueContext {
        QueueContext {
            rail_id: "rtgs".into(),
            asset_id: "ZAR-minor".into(),
            policy_version: "v1".into(),
            horizon_tick,
        }
    }
    fn account(id: &str, balance_minor: u64) -> QueueAccount {
        QueueAccount {
            participant_id: id.into(),
            balance_minor,
        }
    }
    fn route(id: &str, capacity: u64, window: u64) -> QueueRoute {
        QueueRoute {
            route_id: id.into(),
            rail_id: "rtgs".into(),
            asset_id: "ZAR-minor".into(),
            policy_version: "v1".into(),
            enabled: true,
            capacity_minor_per_window: capacity,
            capacity_window_ticks: window,
            fixed_fee_units: 2,
            fee_units_per_minor: 1,
            outages: Vec::new(),
        }
    }
    fn payment(
        id: &str,
        route_id: &str,
        from: &str,
        to: &str,
        amount: u64,
        arrival: u64,
        deadline: Option<u64>,
        priority: u8,
    ) -> PaymentOrder {
        PaymentOrder {
            payment_id: id.into(),
            rail_id: "rtgs".into(),
            asset_id: "ZAR-minor".into(),
            policy_version: "v1".into(),
            route_id: route_id.into(),
            from: from.into(),
            to: to.into(),
            amount_minor: amount,
            arrival_tick: arrival,
            deadline_tick: deadline,
            priority,
        }
    }

    #[test]
    fn queue_scan_can_use_liquidity_received_at_same_tick() {
        let accounts = [account("A", 0), account("B", 0), account("C", 5)];
        let routes = [route("ab", 5, 10), route("ca", 5, 10)];
        let orders = [
            payment("1-ab", "ab", "A", "B", 5, 0, None, 0),
            payment("2-ca", "ca", "C", "A", 5, 1, None, 0),
        ];
        let result = simulate_queue(
            &context(2),
            &accounts,
            &routes,
            &orders,
            QueuePolicy::LiquidityAwareFifo,
        )
        .unwrap();
        assert_eq!(result.settled_count, 2);
        assert_eq!(result.settled_minor, 10);
        assert_eq!(
            result
                .final_balances
                .iter()
                .map(|a| a.balance_minor as u128)
                .sum::<u128>(),
            5
        );
        assert_eq!(
            result
                .account_metrics
                .iter()
                .map(|account| account.balance_time_minor_ticks)
                .sum::<u128>(),
            10
        );
    }

    #[test]
    fn strict_fifo_exposes_head_of_line_blocking() {
        let accounts = [account("A", 0), account("B", 0), account("C", 5)];
        let routes = [route("ab", 5, 10), route("ca", 5, 10)];
        let orders = [
            payment("1-ab", "ab", "A", "B", 5, 0, None, 0),
            payment("2-ca", "ca", "C", "A", 5, 1, None, 0),
        ];
        let result = simulate_queue(
            &context(2),
            &accounts,
            &routes,
            &orders,
            QueuePolicy::StrictFifo,
        )
        .unwrap();
        assert_eq!(result.settled_count, 0);
        assert!(result.payments.iter().any(|p| matches!(
            p.state,
            PaymentState::Unsettled {
                reason: BlockReason::HeadOfLineBlocked,
                ..
            }
        )));
        assert_eq!(result.total_queue_wait_ticks, 3);
        assert_eq!(result.p95_queue_wait_ticks, Some(2));
        assert_eq!(result.max_queue_wait_ticks, Some(2));
    }

    #[test]
    fn priority_scan_chooses_higher_priority_under_scarce_liquidity() {
        let accounts = [account("A", 6), account("B", 0), account("C", 0)];
        let routes = [route("ab", 10, 10), route("ac", 10, 10)];
        let orders = [
            payment("low", "ab", "A", "B", 4, 0, None, 1),
            payment("high", "ac", "A", "C", 6, 0, None, 9),
        ];
        let result = simulate_queue(
            &context(0),
            &accounts,
            &routes,
            &orders,
            QueuePolicy::PriorityScan,
        )
        .unwrap();
        assert_eq!(result.settled_minor, 6);
        assert!(matches!(
            result
                .payments
                .iter()
                .find(|p| p.payment_id == "high")
                .unwrap()
                .state,
            PaymentState::Settled { .. }
        ));
        assert!(matches!(
            result
                .payments
                .iter()
                .find(|p| p.payment_id == "low")
                .unwrap()
                .state,
            PaymentState::Unsettled {
                reason: BlockReason::InsufficientLiquidity,
                ..
            }
        ));
    }

    #[test]
    fn zero_priority_aging_interval_fails_closed() {
        let result = simulate_queue(
            &context(2),
            &[account("A", 1), account("B", 0)],
            &[route("r", 1, 2)],
            &[payment("p", "r", "A", "B", 1, 0, None, 0)],
            QueuePolicy::AgingPriorityScan { interval_ticks: 0 },
        );
        assert_eq!(
            result,
            Err(QueueError::InvalidPolicy(
                "priority aging interval must be greater than zero"
            ))
        );
    }

    #[test]
    fn aging_priority_saturates_at_extreme_tick_without_wrapping() {
        let result = simulate_queue(
            &context(u64::MAX),
            &[account("A", 0), account("B", 0), account("C", 1)],
            &[route("ab", 1, u64::MAX), route("ca", 1, u64::MAX)],
            &[
                payment("old", "ab", "A", "B", 1, 0, None, u8::MAX),
                payment("fund", "ca", "C", "A", 1, u64::MAX, None, 0),
            ],
            QueuePolicy::AgingPriorityScan { interval_ticks: 1 },
        )
        .unwrap();

        assert!(matches!(
            result
                .payments
                .iter()
                .find(|p| p.payment_id == "old")
                .unwrap()
                .state,
            PaymentState::Settled { tick: u64::MAX, .. }
        ));
    }

    #[test]
    fn priority_aging_eventually_serves_an_old_low_priority_payment() {
        let ctx = context(50);
        let accounts = [account("A", 4), account("B", 0), account("C", 0)];
        let routes = [
            route("ab", 1_000, 100),
            route("ac", 1_000, 100),
            route("ca", 1_000, 100),
        ];
        let mut orders = vec![payment("old-low", "ab", "A", "B", 4, 0, None, 0)];
        for tick in 0..=50 {
            orders.push(payment(
                &format!("high-{tick:02}"),
                "ac",
                "A",
                "C",
                4,
                tick,
                None,
                9,
            ));
            if tick > 0 {
                orders.push(payment(
                    &format!("fund-{tick:02}"),
                    "ca",
                    "C",
                    "A",
                    4,
                    tick,
                    None,
                    0,
                ));
            }
        }

        let static_priority =
            simulate_queue(&ctx, &accounts, &routes, &orders, QueuePolicy::PriorityScan).unwrap();
        let aging_priority = simulate_queue(
            &ctx,
            &accounts,
            &routes,
            &orders,
            QueuePolicy::AgingPriorityScan { interval_ticks: 5 },
        )
        .unwrap();

        assert!(matches!(
            static_priority
                .payments
                .iter()
                .find(|p| p.payment_id == "old-low")
                .unwrap()
                .state,
            PaymentState::Unsettled { .. }
        ));
        assert!(matches!(
            aging_priority
                .payments
                .iter()
                .find(|p| p.payment_id == "old-low")
                .unwrap()
                .state,
            PaymentState::Settled { tick, .. } if tick <= 50
        ));
    }

    #[test]
    fn earliest_deadline_and_priority_policies_expose_distinct_tradeoffs() {
        let accounts = [account("A", 6), account("B", 0), account("C", 0)];
        let routes = [route("ab", 10, 10), route("ac", 10, 10)];
        let orders = [
            payment("urgent", "ab", "A", "B", 4, 0, Some(0), 1),
            payment("high-priority", "ac", "A", "C", 6, 0, Some(5), 9),
        ];

        let deadline_first = simulate_queue(
            &context(0),
            &accounts,
            &routes,
            &orders,
            QueuePolicy::EarliestDeadlineFirst,
        )
        .unwrap();
        let priority_first = simulate_queue(
            &context(0),
            &accounts,
            &routes,
            &orders,
            QueuePolicy::PriorityScan,
        )
        .unwrap();

        assert_eq!(deadline_first.settled_minor, 4);
        assert_eq!(priority_first.settled_minor, 6);
        assert_eq!(deadline_first.missed_deadline_count, 0);
        assert!(matches!(
            deadline_first
                .payments
                .iter()
                .find(|p| p.payment_id == "urgent")
                .unwrap()
                .state,
            PaymentState::Settled { .. }
        ));
    }

    #[test]
    fn route_recovers_at_half_open_outage_end() {
        let mut r = route("r", 10, 10);
        r.outages = vec![TickWindow {
            start_tick: 0,
            end_tick: 2,
        }];
        let result = simulate_queue(
            &context(3),
            &[account("A", 10), account("B", 0)],
            &[r],
            &[payment("p", "r", "A", "B", 4, 0, None, 0)],
            QueuePolicy::LiquidityAwareFifo,
        )
        .unwrap();
        assert!(matches!(
            result.payments[0].state,
            PaymentState::Settled { tick: 2, .. }
        ));
    }

    #[test]
    fn route_capacity_resets_at_window_boundary() {
        let result = simulate_queue(
            &context(3),
            &[account("A", 10), account("B", 0), account("C", 0)],
            &[route("r", 5, 3)],
            &[
                payment("a", "r", "A", "B", 4, 0, None, 0),
                payment("b", "r", "A", "C", 4, 0, None, 0),
            ],
            QueuePolicy::LiquidityAwareFifo,
        )
        .unwrap();
        assert_eq!(result.settled_minor, 8);
        assert!(matches!(
            result
                .payments
                .iter()
                .find(|p| p.payment_id == "b")
                .unwrap()
                .state,
            PaymentState::Settled { tick: 3, .. }
        ));
    }

    #[test]
    fn profile_mismatches_and_unknown_routes_stay_excluded() {
        let mut wrong = payment("wrong", "r", "A", "B", 1, 0, None, 0);
        wrong.asset_id = "other".into();
        let result = simulate_queue(
            &context(1),
            &[account("A", 3), account("B", 0)],
            &[route("r", 3, 2)],
            &[
                wrong,
                payment("unknown", "missing", "A", "B", 1, 0, None, 0),
            ],
            QueuePolicy::LiquidityAwareFifo,
        )
        .unwrap();
        assert_eq!(result.excluded_count, 2);
        assert_eq!(result.settled_count, 0);
    }

    #[test]
    fn input_reordering_does_not_change_priority_policy_result() {
        let ctx = context(4);
        let accounts = [account("A", 5), account("B", 1), account("C", 0)];
        let routes = [route("ab", 5, 2), route("bc", 5, 2)];
        let orders = [
            payment("p1", "ab", "A", "B", 4, 0, Some(1), 2),
            payment("p2", "bc", "B", "C", 4, 1, Some(2), 3),
            payment("p3", "ab", "A", "B", 2, 1, Some(3), 1),
        ];
        let first =
            simulate_queue(&ctx, &accounts, &routes, &orders, QueuePolicy::PriorityScan).unwrap();
        let mut a = accounts.to_vec();
        a.reverse();
        let mut r = routes.to_vec();
        r.reverse();
        let mut p = orders.to_vec();
        p.reverse();
        let second = simulate_queue(&ctx, &a, &r, &p, QueuePolicy::PriorityScan).unwrap();
        assert_eq!(first, second);
    }
}

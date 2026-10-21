#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Reference payment-queue simulation under fixed participant opening liquidity.
//!
//! This module models repeated queue scans for accepted gross payments. A payment settles
//! only when its payer has enough available balance; settlement transfers balance and does
//! not mint liquidity. The simulator is not an RTGS implementation, a payment executor, a
//! legally binding settlement record, or a reproduction of any central-bank dataset.
//!
//! It provides FIFO and unlock-potential ordering baselines, a seeded simulated-annealing
//! search, and an exhaustive permutation oracle for batches of at most eight payments.
//! Monetary amounts use integer minor units; the annealing temperature is only a search
//! heuristic and never participates in monetary arithmetic.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

/// Exhaustive permutation search is limited to small fixtures (8! = 40,320 orders).
pub const MAX_EXACT_ORDER_PAYMENTS: usize = 8;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueueContext {
    pub network_id: String,
    pub asset_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParticipantLiquidity {
    pub participant_id: String,
    pub opening_liquidity_minor: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PaymentState {
    Accepted,
    Proposed,
    Disputed,
    Cancelled,
    AlreadySettled,
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueuedPayment {
    pub payment_id: String,
    pub network_id: String,
    pub asset_id: String,
    pub payer: String,
    pub payee: String,
    pub amount_minor: u64,
    /// Lower values mean the payment entered the queue earlier.
    pub queue_sequence: u64,
    /// Larger values mean a greater priority-delay penalty.
    pub priority_weight: u32,
    /// Simulated settlement index by which the payment should settle.
    pub deadline_step: Option<u64>,
    pub state: PaymentState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExclusionReason {
    Proposed,
    Disputed,
    Cancelled,
    AlreadySettled,
    UnknownState,
    ProfileMismatch,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExcludedPayment {
    pub payment_id: String,
    pub reason: ExclusionReason,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BalanceSnapshot {
    pub participant_id: String,
    pub final_liquidity_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettlementEvent {
    pub payment_id: String,
    pub pass_index: u64,
    pub settlement_index: u64,
    pub amount_minor: u64,
    pub payer_balance_before_minor: u64,
    pub payer_balance_after_minor: u64,
    pub payee_balance_before_minor: u64,
    pub payee_balance_after_minor: u64,
    pub missed_deadline: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueueSimulation {
    pub network_id: String,
    pub asset_id: String,
    pub opening_liquidity_total_minor: u128,
    pub closing_liquidity_total_minor: u128,
    pub total_settled_minor: u128,
    pub settled_count: u64,
    pub settled_payment_ids: Vec<String>,
    pub pending_payment_ids: Vec<String>,
    pub final_balances: Vec<BalanceSnapshot>,
    pub events: Vec<SettlementEvent>,
    pub excluded_payments: Vec<ExcludedPayment>,
    pub deduplicated_payment_observations: usize,
    pub passes_with_progress: u64,
    /// Includes the final no-progress scan that proves the queue is blocked or empty.
    pub passes_inspected: u64,
    pub missed_deadline_count: u64,
    pub priority_weighted_delay_penalty: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParticipantMinimumLiquidity {
    pub participant_id: String,
    pub required_opening_liquidity_minor: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MinimumLiquidityStep {
    pub payment_id: String,
    pub settlement_index: u64,
    pub payer_cumulative_net_minor: i128,
    pub payee_cumulative_net_minor: i128,
    pub missed_deadline: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MinimumLiquidityReport {
    pub network_id: String,
    pub asset_id: String,
    pub order: Vec<String>,
    pub total_payment_value_minor: u128,
    pub total_required_opening_liquidity_minor: u128,
    pub required_by_participant: Vec<ParticipantMinimumLiquidity>,
    pub steps: Vec<MinimumLiquidityStep>,
    pub excluded_payments: Vec<ExcludedPayment>,
    pub deduplicated_payment_observations: usize,
    pub missed_deadline_count: u64,
    pub priority_weighted_delay_penalty: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MinimumLiquidityOracleResult {
    pub best_order: Vec<String>,
    pub best_report: MinimumLiquidityReport,
    pub candidate_orders_evaluated: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MinimumLiquidityAnnealingResult {
    pub best_order: Vec<String>,
    pub best_report: MinimumLiquidityReport,
    pub iterations_evaluated: usize,
    pub seed: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QueueError {
    InvalidContext(&'static str),
    InvalidParticipant(String),
    DuplicateParticipant(String),
    InvalidPayment(String),
    ConflictingDuplicatePayment(String),
    UnknownParticipant(String),
    InvalidQueueOrder,
    TooManyExactPayments { observed: usize, maximum: usize },
    InvalidAnnealingConfig,
    ArithmeticOverflow,
    ConservationViolation,
}

impl fmt::Display for QueueError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for QueueError {}

#[derive(Clone, Debug)]
struct PreparedPayments {
    eligible: Vec<QueuedPayment>,
    excluded: Vec<ExcludedPayment>,
    deduplicated_observations: usize,
}

#[derive(Clone, Debug)]
struct PreparedInput {
    context: QueueContext,
    balances: BTreeMap<String, u64>,
    opening_total: u128,
    payments: PreparedPayments,
}

fn checked_add_u128(left: u128, right: u128) -> Result<u128, QueueError> {
    left.checked_add(right)
        .ok_or(QueueError::ArithmeticOverflow)
}

fn nonempty(field: &'static str, value: &str) -> Result<(), QueueError> {
    if value.trim().is_empty() {
        return Err(QueueError::InvalidContext(field));
    }
    Ok(())
}

fn prepare_payment_observations(
    context: &QueueContext,
    observations: &[QueuedPayment],
) -> Result<PreparedPayments, QueueError> {
    nonempty("network_id", &context.network_id)?;
    nonempty("asset_id", &context.asset_id)?;

    let mut by_id = BTreeMap::<String, QueuedPayment>::new();
    let mut deduplicated_observations = 0_usize;

    for payment in observations {
        if payment.payment_id.trim().is_empty() {
            return Err(QueueError::InvalidPayment("empty payment_id".into()));
        }
        if payment.network_id.trim().is_empty() || payment.asset_id.trim().is_empty() {
            return Err(QueueError::InvalidPayment(format!(
                "payment {} has an empty network or asset",
                payment.payment_id
            )));
        }
        if payment.payer.trim().is_empty() || payment.payee.trim().is_empty() {
            return Err(QueueError::InvalidPayment(format!(
                "payment {} has an empty payer or payee",
                payment.payment_id
            )));
        }
        if payment.payer == payment.payee {
            return Err(QueueError::InvalidPayment(format!(
                "payment {} is a self-payment",
                payment.payment_id
            )));
        }
        if payment.amount_minor == 0 {
            return Err(QueueError::InvalidPayment(format!(
                "payment {} has a zero amount",
                payment.payment_id
            )));
        }

        if let Some(previous) = by_id.get(&payment.payment_id) {
            if previous == payment {
                deduplicated_observations = deduplicated_observations
                    .checked_add(1)
                    .ok_or(QueueError::ArithmeticOverflow)?;
                continue;
            }
            return Err(QueueError::ConflictingDuplicatePayment(
                payment.payment_id.clone(),
            ));
        }
        by_id.insert(payment.payment_id.clone(), payment.clone());
    }

    let mut eligible = Vec::new();
    let mut excluded = Vec::new();

    for payment in by_id.into_values() {
        let reason = match payment.state {
            PaymentState::Accepted => {
                if payment.network_id != context.network_id || payment.asset_id != context.asset_id
                {
                    Some(ExclusionReason::ProfileMismatch)
                } else {
                    None
                }
            }
            PaymentState::Proposed => Some(ExclusionReason::Proposed),
            PaymentState::Disputed => Some(ExclusionReason::Disputed),
            PaymentState::Cancelled => Some(ExclusionReason::Cancelled),
            PaymentState::AlreadySettled => Some(ExclusionReason::AlreadySettled),
            PaymentState::Unknown => Some(ExclusionReason::UnknownState),
        };

        if let Some(reason) = reason {
            excluded.push(ExcludedPayment {
                payment_id: payment.payment_id,
                reason,
            });
        } else {
            eligible.push(payment);
        }
    }

    eligible.sort_by(|left, right| {
        left.queue_sequence
            .cmp(&right.queue_sequence)
            .then_with(|| left.payment_id.cmp(&right.payment_id))
    });
    excluded.sort_by(|left, right| left.payment_id.cmp(&right.payment_id));

    Ok(PreparedPayments {
        eligible,
        excluded,
        deduplicated_observations,
    })
}

fn prepare_input(
    context: &QueueContext,
    balances: &[ParticipantLiquidity],
    payments: &[QueuedPayment],
) -> Result<PreparedInput, QueueError> {
    let mut balance_map = BTreeMap::<String, u64>::new();
    let mut opening_total = 0_u128;

    for balance in balances {
        if balance.participant_id.trim().is_empty() {
            return Err(QueueError::InvalidParticipant(
                "empty participant_id".into(),
            ));
        }
        if balance_map.contains_key(&balance.participant_id) {
            return Err(QueueError::DuplicateParticipant(
                balance.participant_id.clone(),
            ));
        }
        opening_total = checked_add_u128(opening_total, balance.opening_liquidity_minor as u128)?;
        balance_map.insert(
            balance.participant_id.clone(),
            balance.opening_liquidity_minor,
        );
    }

    let prepared_payments = prepare_payment_observations(context, payments)?;
    for payment in &prepared_payments.eligible {
        if !balance_map.contains_key(&payment.payer) {
            return Err(QueueError::UnknownParticipant(payment.payer.clone()));
        }
        if !balance_map.contains_key(&payment.payee) {
            return Err(QueueError::UnknownParticipant(payment.payee.clone()));
        }
    }

    Ok(PreparedInput {
        context: context.clone(),
        balances: balance_map,
        opening_total,
        payments: prepared_payments,
    })
}

/// Return eligible payment IDs in FIFO order. Ineligible observations remain excluded.
pub fn fifo_order(
    context: &QueueContext,
    payments: &[QueuedPayment],
) -> Result<Vec<String>, QueueError> {
    Ok(prepare_payment_observations(context, payments)?
        .eligible
        .into_iter()
        .map(|payment| payment.payment_id)
        .collect())
}

/// Return a deterministic order that prioritizes payments feeding participants with
/// larger accepted outgoing queues; priority weight breaks ties before FIFO sequence.
/// This is a heuristic baseline, not a claimed optimal schedule.
pub fn unlock_potential_order(
    context: &QueueContext,
    payments: &[QueuedPayment],
) -> Result<Vec<String>, QueueError> {
    let prepared = prepare_payment_observations(context, payments)?;
    let mut outgoing = BTreeMap::<String, u128>::new();
    for payment in &prepared.eligible {
        let current = outgoing.get(&payment.payer).copied().unwrap_or(0);
        outgoing.insert(
            payment.payer.clone(),
            checked_add_u128(current, payment.amount_minor as u128)?,
        );
    }

    let mut ordered = prepared.eligible;
    ordered.sort_by(|left, right| {
        right
            .priority_weight
            .cmp(&left.priority_weight)
            .then_with(|| {
                outgoing
                    .get(&right.payee)
                    .copied()
                    .unwrap_or(0)
                    .cmp(&outgoing.get(&left.payee).copied().unwrap_or(0))
            })
            .then_with(|| left.queue_sequence.cmp(&right.queue_sequence))
            .then_with(|| left.payment_id.cmp(&right.payment_id))
    });

    Ok(ordered
        .into_iter()
        .map(|payment| payment.payment_id)
        .collect())
}

/// Simulate a fixed queue order by repeatedly scanning it from the head and settling
/// every currently executable payment. A payment skipped for insufficient liquidity
/// remains pending and may execute during a later pass after an incoming payment funds
/// its payer. The scan terminates on the first pass with no settlement.
pub fn simulate_order(
    context: &QueueContext,
    balances: &[ParticipantLiquidity],
    payments: &[QueuedPayment],
    order: &[String],
) -> Result<QueueSimulation, QueueError> {
    let prepared = prepare_input(context, balances, payments)?;
    validate_order(&prepared.payments.eligible, order)?;

    let payments_by_id = prepared
        .payments
        .eligible
        .iter()
        .map(|payment| (payment.payment_id.as_str(), payment))
        .collect::<BTreeMap<_, _>>();
    let mut remaining = prepared
        .payments
        .eligible
        .iter()
        .map(|payment| payment.payment_id.clone())
        .collect::<BTreeSet<_>>();
    let mut mutable_balances = prepared.balances.clone();
    let mut events = Vec::new();
    let mut settled_payment_ids = Vec::new();
    let mut total_settled_minor = 0_u128;
    let mut missed_deadline_count = 0_u64;
    let mut priority_weighted_delay_penalty = 0_u128;
    let mut passes_with_progress = 0_u64;
    let mut passes_inspected = 0_u64;
    let mut settlement_index = 0_u64;

    loop {
        passes_inspected = passes_inspected
            .checked_add(1)
            .ok_or(QueueError::ArithmeticOverflow)?;
        let mut progress = false;

        for payment_id in order {
            if !remaining.contains(payment_id) {
                continue;
            }

            let payment = payments_by_id
                .get(payment_id.as_str())
                .copied()
                .ok_or(QueueError::InvalidQueueOrder)?;
            let payer_balance = mutable_balances
                .get(&payment.payer)
                .copied()
                .ok_or_else(|| QueueError::UnknownParticipant(payment.payer.clone()))?;
            if payer_balance < payment.amount_minor {
                continue;
            }
            let payee_balance = mutable_balances
                .get(&payment.payee)
                .copied()
                .ok_or_else(|| QueueError::UnknownParticipant(payment.payee.clone()))?;
            let payer_after = payer_balance - payment.amount_minor;
            let payee_after = payee_balance
                .checked_add(payment.amount_minor)
                .ok_or(QueueError::ArithmeticOverflow)?;

            mutable_balances.insert(payment.payer.clone(), payer_after);
            mutable_balances.insert(payment.payee.clone(), payee_after);
            total_settled_minor =
                checked_add_u128(total_settled_minor, payment.amount_minor as u128)?;
            settlement_index = settlement_index
                .checked_add(1)
                .ok_or(QueueError::ArithmeticOverflow)?;

            let missed_deadline = payment
                .deadline_step
                .is_some_and(|deadline| settlement_index > deadline);
            if missed_deadline {
                missed_deadline_count = missed_deadline_count
                    .checked_add(1)
                    .ok_or(QueueError::ArithmeticOverflow)?;
            }

            let delay_penalty = (payment.priority_weight as u128)
                .checked_mul(settlement_index as u128)
                .ok_or(QueueError::ArithmeticOverflow)?;
            priority_weighted_delay_penalty =
                checked_add_u128(priority_weighted_delay_penalty, delay_penalty)?;

            events.push(SettlementEvent {
                payment_id: payment.payment_id.clone(),
                pass_index: passes_inspected,
                settlement_index,
                amount_minor: payment.amount_minor,
                payer_balance_before_minor: payer_balance,
                payer_balance_after_minor: payer_after,
                payee_balance_before_minor: payee_balance,
                payee_balance_after_minor: payee_after,
                missed_deadline,
            });
            settled_payment_ids.push(payment.payment_id.clone());
            remaining.remove(payment_id);
            progress = true;
        }

        if !progress {
            break;
        }
        passes_with_progress = passes_with_progress
            .checked_add(1)
            .ok_or(QueueError::ArithmeticOverflow)?;
        if passes_with_progress as usize > prepared.payments.eligible.len() {
            return Err(QueueError::ConservationViolation);
        }
    }

    let mut closing_total = 0_u128;
    let final_balances = mutable_balances
        .iter()
        .map(|(participant_id, final_liquidity_minor)| {
            closing_total = checked_add_u128(closing_total, *final_liquidity_minor as u128)?;
            Ok(BalanceSnapshot {
                participant_id: participant_id.clone(),
                final_liquidity_minor: *final_liquidity_minor,
            })
        })
        .collect::<Result<Vec<_>, QueueError>>()?;

    if closing_total != prepared.opening_total {
        return Err(QueueError::ConservationViolation);
    }

    // A queued payment that remains blocked after its simulated deadline is
    // counted as missed; it is not silently treated as timely merely because it
    // never generated a settlement event.
    for payment_id in &remaining {
        let payment = payments_by_id
            .get(payment_id.as_str())
            .copied()
            .ok_or(QueueError::InvalidQueueOrder)?;
        if payment
            .deadline_step
            .is_some_and(|deadline| deadline <= settlement_index)
        {
            missed_deadline_count = missed_deadline_count
                .checked_add(1)
                .ok_or(QueueError::ArithmeticOverflow)?;
        }
    }

    let pending_payment_ids = order
        .iter()
        .filter(|payment_id| remaining.contains(*payment_id))
        .cloned()
        .collect::<Vec<_>>();

    Ok(QueueSimulation {
        network_id: prepared.context.network_id,
        asset_id: prepared.context.asset_id,
        opening_liquidity_total_minor: prepared.opening_total,
        closing_liquidity_total_minor: closing_total,
        total_settled_minor,
        settled_count: settled_payment_ids.len() as u64,
        settled_payment_ids,
        pending_payment_ids,
        final_balances,
        events,
        excluded_payments: prepared.payments.excluded,
        deduplicated_payment_observations: prepared.payments.deduplicated_observations,
        passes_with_progress,
        passes_inspected,
        missed_deadline_count,
        priority_weighted_delay_penalty,
    })
}

fn validate_order(eligible: &[QueuedPayment], order: &[String]) -> Result<(), QueueError> {
    if eligible.len() != order.len() {
        return Err(QueueError::InvalidQueueOrder);
    }
    let expected = eligible
        .iter()
        .map(|payment| payment.payment_id.as_str())
        .collect::<BTreeSet<_>>();
    let actual = order.iter().map(String::as_str).collect::<BTreeSet<_>>();
    if expected != actual || actual.len() != order.len() {
        return Err(QueueError::InvalidQueueOrder);
    }
    Ok(())
}

/// Compute the minimum opening liquidity vector needed to execute every eligible
/// payment in exactly the supplied sequence. For each participant this is the maximum
/// prefix deficit (cumulative outflows minus cumulative inflows); the sum is the aggregate
/// starting-liquidity requirement for this order. This evaluates a strict execution
/// sequence, unlike simulate_order, which treats its order as a queue priority list and
/// may execute a later item first when an earlier item is blocked.
pub fn minimum_opening_liquidity_for_sequence(
    context: &QueueContext,
    payments: &[QueuedPayment],
    order: &[String],
) -> Result<MinimumLiquidityReport, QueueError> {
    let prepared = prepare_payment_observations(context, payments)?;
    validate_order(&prepared.eligible, order)?;
    let payments_by_id = prepared
        .eligible
        .iter()
        .map(|payment| (payment.payment_id.as_str(), payment))
        .collect::<BTreeMap<_, _>>();

    let mut running_net = BTreeMap::<String, i128>::new();
    let mut required_by_id = BTreeMap::<String, u128>::new();
    for payment in &prepared.eligible {
        running_net.entry(payment.payer.clone()).or_insert(0);
        running_net.entry(payment.payee.clone()).or_insert(0);
        required_by_id.entry(payment.payer.clone()).or_insert(0);
        required_by_id.entry(payment.payee.clone()).or_insert(0);
    }

    let mut total_payment_value_minor = 0_u128;
    let mut missed_deadline_count = 0_u64;
    let mut priority_weighted_delay_penalty = 0_u128;
    let mut steps = Vec::with_capacity(order.len());

    for (index, payment_id) in order.iter().enumerate() {
        let payment = payments_by_id
            .get(payment_id.as_str())
            .copied()
            .ok_or(QueueError::InvalidQueueOrder)?;
        let settlement_index = (index as u64)
            .checked_add(1)
            .ok_or(QueueError::ArithmeticOverflow)?;
        let amount = payment.amount_minor as i128;
        let payer_net = running_net
            .get(&payment.payer)
            .copied()
            .unwrap_or(0)
            .checked_sub(amount)
            .ok_or(QueueError::ArithmeticOverflow)?;
        running_net.insert(payment.payer.clone(), payer_net);
        let payer_deficit = if payer_net < 0 {
            payer_net.unsigned_abs()
        } else {
            0
        };
        let payer_required = required_by_id
            .get(&payment.payer)
            .copied()
            .unwrap_or(0)
            .max(payer_deficit);
        required_by_id.insert(payment.payer.clone(), payer_required);

        let payee_net = running_net
            .get(&payment.payee)
            .copied()
            .unwrap_or(0)
            .checked_add(amount)
            .ok_or(QueueError::ArithmeticOverflow)?;
        running_net.insert(payment.payee.clone(), payee_net);

        total_payment_value_minor =
            checked_add_u128(total_payment_value_minor, payment.amount_minor as u128)?;
        let missed_deadline = payment
            .deadline_step
            .is_some_and(|deadline| settlement_index > deadline);
        if missed_deadline {
            missed_deadline_count = missed_deadline_count
                .checked_add(1)
                .ok_or(QueueError::ArithmeticOverflow)?;
        }
        let delay_penalty = (payment.priority_weight as u128)
            .checked_mul(settlement_index as u128)
            .ok_or(QueueError::ArithmeticOverflow)?;
        priority_weighted_delay_penalty =
            checked_add_u128(priority_weighted_delay_penalty, delay_penalty)?;

        steps.push(MinimumLiquidityStep {
            payment_id: payment.payment_id.clone(),
            settlement_index,
            payer_cumulative_net_minor: payer_net,
            payee_cumulative_net_minor: payee_net,
            missed_deadline,
        });
    }

    let mut total_required_opening_liquidity_minor = 0_u128;
    let required_by_participant = required_by_id
        .into_iter()
        .map(|(participant_id, required_opening_liquidity_minor)| {
            total_required_opening_liquidity_minor = checked_add_u128(
                total_required_opening_liquidity_minor,
                required_opening_liquidity_minor,
            )?;
            Ok(ParticipantMinimumLiquidity {
                participant_id,
                required_opening_liquidity_minor,
            })
        })
        .collect::<Result<Vec<_>, QueueError>>()?;

    Ok(MinimumLiquidityReport {
        network_id: context.network_id.clone(),
        asset_id: context.asset_id.clone(),
        order: order.to_vec(),
        total_payment_value_minor,
        total_required_opening_liquidity_minor,
        required_by_participant,
        steps,
        excluded_payments: prepared.excluded,
        deduplicated_payment_observations: prepared.deduplicated_observations,
        missed_deadline_count,
        priority_weighted_delay_penalty,
    })
}

/// Greater means better: less required opening liquidity, fewer missed deadlines,
/// then lower priority-weighted settlement-index penalty.
pub fn compare_minimum_liquidity_reports(
    left: &MinimumLiquidityReport,
    right: &MinimumLiquidityReport,
) -> Ordering {
    right
        .total_required_opening_liquidity_minor
        .cmp(&left.total_required_opening_liquidity_minor)
        .then_with(|| right.missed_deadline_count.cmp(&left.missed_deadline_count))
        .then_with(|| {
            right
                .priority_weighted_delay_penalty
                .cmp(&left.priority_weighted_delay_penalty)
        })
}

/// Exhaustively find the order requiring the least total opening liquidity. This oracle
/// is capped at eight eligible payments and is meant for objective-gap validation.
pub fn minimum_liquidity_order_oracle(
    context: &QueueContext,
    payments: &[QueuedPayment],
) -> Result<MinimumLiquidityOracleResult, QueueError> {
    let prepared = prepare_payment_observations(context, payments)?;
    let mut order = prepared
        .eligible
        .iter()
        .map(|payment| payment.payment_id.clone())
        .collect::<Vec<_>>();
    if order.len() > MAX_EXACT_ORDER_PAYMENTS {
        return Err(QueueError::TooManyExactPayments {
            observed: order.len(),
            maximum: MAX_EXACT_ORDER_PAYMENTS,
        });
    }

    struct Best {
        order: Vec<String>,
        report: MinimumLiquidityReport,
    }

    fn visit(
        index: usize,
        order: &mut [String],
        context: &QueueContext,
        payments: &[QueuedPayment],
        best: &mut Option<Best>,
        candidates: &mut u64,
    ) -> Result<(), QueueError> {
        if index == order.len() {
            let report = minimum_opening_liquidity_for_sequence(context, payments, order)?;
            *candidates = candidates
                .checked_add(1)
                .ok_or(QueueError::ArithmeticOverflow)?;
            let replace = best.as_ref().is_none_or(|current| {
                let comparison = compare_minimum_liquidity_reports(&report, &current.report);
                comparison == Ordering::Greater
                    || (comparison == Ordering::Equal
                        && order.iter().cmp(current.order.iter()) == Ordering::Less)
            });
            if replace {
                *best = Some(Best {
                    order: order.to_vec(),
                    report,
                });
            }
            return Ok(());
        }

        for next in index..order.len() {
            order.swap(index, next);
            visit(index + 1, order, context, payments, best, candidates)?;
            order.swap(index, next);
        }
        Ok(())
    }

    let mut best = None;
    let mut candidates = 0_u64;
    visit(0, &mut order, context, payments, &mut best, &mut candidates)?;
    let best = best.ok_or(QueueError::ConservationViolation)?;
    Ok(MinimumLiquidityOracleResult {
        best_order: best.order,
        best_report: best.report,
        candidate_orders_evaluated: candidates,
    })
}

fn normalized_minimum_liquidity_loss(
    candidate: &MinimumLiquidityReport,
    current: &MinimumLiquidityReport,
    maximum_payment_value: u128,
    payment_count: usize,
    maximum_delay_penalty: u128,
) -> f64 {
    if candidate.total_required_opening_liquidity_minor
        != current.total_required_opening_liquidity_minor
    {
        return candidate
            .total_required_opening_liquidity_minor
            .abs_diff(current.total_required_opening_liquidity_minor) as f64
            / maximum_payment_value.max(1) as f64;
    }
    if candidate.missed_deadline_count != current.missed_deadline_count {
        return candidate
            .missed_deadline_count
            .abs_diff(current.missed_deadline_count) as f64
            / payment_count.max(1) as f64;
    }
    candidate
        .priority_weighted_delay_penalty
        .abs_diff(current.priority_weighted_delay_penalty) as f64
        / maximum_delay_penalty.max(1) as f64
}

/// Seeded simulated annealing that minimizes the strict-sequence opening-liquidity
/// requirement. This is separate from simulated_annealing_order, which maximizes settled
/// value under an existing fixed-liquidity vector.
pub fn simulated_annealing_minimum_liquidity_order(
    context: &QueueContext,
    payments: &[QueuedPayment],
    initial_order: &[String],
    config: &AnnealingConfig,
) -> Result<MinimumLiquidityAnnealingResult, QueueError> {
    if !config.initial_temperature.is_finite()
        || config.initial_temperature <= 0.0
        || !config.cooling_factor.is_finite()
        || config.cooling_factor <= 0.0
        || config.cooling_factor > 1.0
    {
        return Err(QueueError::InvalidAnnealingConfig);
    }

    let prepared = prepare_payment_observations(context, payments)?;
    validate_order(&prepared.eligible, initial_order)?;
    let payment_count = prepared.eligible.len();
    let maximum_payment_value = prepared
        .eligible
        .iter()
        .try_fold(0_u128, |total, payment| {
            checked_add_u128(total, payment.amount_minor as u128)
        })?;
    let priority_sum = prepared
        .eligible
        .iter()
        .try_fold(0_u128, |total, payment| {
            checked_add_u128(total, payment.priority_weight as u128)
        })?;
    let maximum_delay_penalty = priority_sum
        .checked_mul(payment_count as u128)
        .ok_or(QueueError::ArithmeticOverflow)?;

    let mut current_order = initial_order.to_vec();
    let mut current_report =
        minimum_opening_liquidity_for_sequence(context, payments, &current_order)?;
    let mut best_order = current_order.clone();
    let mut best_report = current_report.clone();
    let mut rng = XorShift64::new(config.seed);
    let mut temperature = config.initial_temperature;
    let mut iterations_evaluated = 0_usize;

    if payment_count < 2 {
        return Ok(MinimumLiquidityAnnealingResult {
            best_order,
            best_report,
            iterations_evaluated,
            seed: config.seed,
        });
    }

    for _ in 0..config.iterations {
        iterations_evaluated = iterations_evaluated
            .checked_add(1)
            .ok_or(QueueError::ArithmeticOverflow)?;
        let first = (rng.next_u64() as usize) % payment_count;
        let mut second = (rng.next_u64() as usize) % payment_count;
        if second == first {
            second = (second + 1) % payment_count;
        }

        // Insertion moves explore re-orderings that require multiple pairwise
        // swaps to realize. For example, moving a blocked early payment to the
        // end can expose an incoming-liquidity cycle in a single neighborhood step.
        let mut candidate_order = current_order.clone();
        let moved = candidate_order.remove(first);
        candidate_order.insert(second, moved);
        let candidate_report =
            minimum_opening_liquidity_for_sequence(context, payments, &candidate_order)?;
        let comparison = compare_minimum_liquidity_reports(&candidate_report, &current_report);

        let accept = if comparison != Ordering::Less {
            comparison == Ordering::Greater || candidate_order < current_order
        } else {
            let loss = normalized_minimum_liquidity_loss(
                &candidate_report,
                &current_report,
                maximum_payment_value,
                payment_count,
                maximum_delay_penalty,
            );
            rng.unit_interval() < (-loss / temperature).exp()
        };

        if accept {
            current_order = candidate_order.clone();
            current_report = candidate_report.clone();
        }

        let best_comparison = compare_minimum_liquidity_reports(&candidate_report, &best_report);
        if best_comparison == Ordering::Greater
            || (best_comparison == Ordering::Equal && candidate_order < best_order)
        {
            best_order = candidate_order;
            best_report = candidate_report;
        }

        temperature *= config.cooling_factor;
    }

    Ok(MinimumLiquidityAnnealingResult {
        best_order,
        best_report,
        iterations_evaluated,
        seed: config.seed,
    })
}

/// Compare two simulations. Greater means better: more settled value, more settled
/// payments, fewer missed deadlines, then lower priority-weighted delay penalty.
pub fn compare_simulations(left: &QueueSimulation, right: &QueueSimulation) -> Ordering {
    left.total_settled_minor
        .cmp(&right.total_settled_minor)
        .then_with(|| left.settled_count.cmp(&right.settled_count))
        .then_with(|| right.missed_deadline_count.cmp(&left.missed_deadline_count))
        .then_with(|| {
            right
                .priority_weighted_delay_penalty
                .cmp(&left.priority_weighted_delay_penalty)
        })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactOrderResult {
    pub best_order: Vec<String>,
    pub best_simulation: QueueSimulation,
    pub candidate_orders_evaluated: u64,
}

/// Enumerate all payment permutations for an exact small-instance reference result.
/// More than eight eligible payments fail closed instead of silently using a heuristic.
pub fn exact_order_oracle(
    context: &QueueContext,
    balances: &[ParticipantLiquidity],
    payments: &[QueuedPayment],
) -> Result<ExactOrderResult, QueueError> {
    let prepared = prepare_input(context, balances, payments)?;
    let mut order = prepared
        .payments
        .eligible
        .iter()
        .map(|payment| payment.payment_id.clone())
        .collect::<Vec<_>>();
    if order.len() > MAX_EXACT_ORDER_PAYMENTS {
        return Err(QueueError::TooManyExactPayments {
            observed: order.len(),
            maximum: MAX_EXACT_ORDER_PAYMENTS,
        });
    }

    struct Best {
        order: Vec<String>,
        simulation: QueueSimulation,
    }

    fn visit(
        index: usize,
        order: &mut [String],
        context: &QueueContext,
        balances: &[ParticipantLiquidity],
        payments: &[QueuedPayment],
        best: &mut Option<Best>,
        candidates: &mut u64,
    ) -> Result<(), QueueError> {
        if index == order.len() {
            let simulation = simulate_order(context, balances, payments, order)?;
            *candidates = candidates
                .checked_add(1)
                .ok_or(QueueError::ArithmeticOverflow)?;
            let replace = best.as_ref().is_none_or(|current| {
                let comparison = compare_simulations(&simulation, &current.simulation);
                comparison == Ordering::Greater
                    || (comparison == Ordering::Equal
                        && order.iter().cmp(current.order.iter()) == Ordering::Less)
            });
            if replace {
                *best = Some(Best {
                    order: order.to_vec(),
                    simulation,
                });
            }
            return Ok(());
        }

        for next in index..order.len() {
            order.swap(index, next);
            visit(
                index + 1,
                order,
                context,
                balances,
                payments,
                best,
                candidates,
            )?;
            order.swap(index, next);
        }
        Ok(())
    }

    let mut best = None;
    let mut candidates = 0_u64;
    visit(
        0,
        &mut order,
        context,
        balances,
        payments,
        &mut best,
        &mut candidates,
    )?;
    let best = best.ok_or(QueueError::ConservationViolation)?;
    Ok(ExactOrderResult {
        best_order: best.order,
        best_simulation: best.simulation,
        candidate_orders_evaluated: candidates,
    })
}

#[derive(Clone, Debug, PartialEq)]
pub struct AnnealingConfig {
    pub iterations: usize,
    pub seed: u64,
    pub initial_temperature: f64,
    /// Must be greater than zero and at most one.
    pub cooling_factor: f64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AnnealingResult {
    pub best_order: Vec<String>,
    pub best_simulation: QueueSimulation,
    pub iterations_evaluated: usize,
    pub seed: u64,
}

#[derive(Clone, Debug)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0x9E3779B97F4A7C15 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut value = self.state;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.state = value;
        value
    }

    fn unit_interval(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64) * (1.0 / ((1_u64 << 53) as f64))
    }
}

fn normalized_loss(
    candidate: &QueueSimulation,
    current: &QueueSimulation,
    maximum_value: u128,
    payment_count: usize,
    maximum_delay_penalty: u128,
) -> f64 {
    if candidate.total_settled_minor != current.total_settled_minor {
        let gap = candidate
            .total_settled_minor
            .abs_diff(current.total_settled_minor);
        return gap as f64 / maximum_value.max(1) as f64;
    }
    if candidate.settled_count != current.settled_count {
        let gap = candidate.settled_count.abs_diff(current.settled_count);
        return gap as f64 / payment_count.max(1) as f64;
    }
    if candidate.missed_deadline_count != current.missed_deadline_count {
        let gap = candidate
            .missed_deadline_count
            .abs_diff(current.missed_deadline_count);
        return gap as f64 / payment_count.max(1) as f64;
    }
    let gap = candidate
        .priority_weighted_delay_penalty
        .abs_diff(current.priority_weighted_delay_penalty);
    gap as f64 / maximum_delay_penalty.max(1) as f64
}

/// Run seeded simulated annealing over payment-order swaps. The best result is retained
/// under the lexicographic comparison objective regardless of accepted uphill moves.
/// This is an experimental heuristic and must be compared against the exact oracle on
/// tiny cases before its solution gap is reported.
pub fn simulated_annealing_order(
    context: &QueueContext,
    balances: &[ParticipantLiquidity],
    payments: &[QueuedPayment],
    initial_order: &[String],
    config: &AnnealingConfig,
) -> Result<AnnealingResult, QueueError> {
    if !config.initial_temperature.is_finite()
        || config.initial_temperature <= 0.0
        || !config.cooling_factor.is_finite()
        || config.cooling_factor <= 0.0
        || config.cooling_factor > 1.0
    {
        return Err(QueueError::InvalidAnnealingConfig);
    }

    let prepared = prepare_input(context, balances, payments)?;
    validate_order(&prepared.payments.eligible, initial_order)?;
    let payment_count = prepared.payments.eligible.len();
    let maximum_value = prepared
        .payments
        .eligible
        .iter()
        .try_fold(0_u128, |total, payment| {
            checked_add_u128(total, payment.amount_minor as u128)
        })?;
    let priority_sum = prepared
        .payments
        .eligible
        .iter()
        .try_fold(0_u128, |total, payment| {
            checked_add_u128(total, payment.priority_weight as u128)
        })?;
    let maximum_delay_penalty = priority_sum
        .checked_mul(payment_count as u128)
        .ok_or(QueueError::ArithmeticOverflow)?;

    let mut current_order = initial_order.to_vec();
    let mut current_simulation = simulate_order(context, balances, payments, &current_order)?;
    let mut best_order = current_order.clone();
    let mut best_simulation = current_simulation.clone();
    let mut rng = XorShift64::new(config.seed);
    let mut temperature = config.initial_temperature;
    let mut iterations_evaluated = 0_usize;

    if payment_count < 2 {
        return Ok(AnnealingResult {
            best_order,
            best_simulation,
            iterations_evaluated,
            seed: config.seed,
        });
    }

    for _ in 0..config.iterations {
        iterations_evaluated = iterations_evaluated
            .checked_add(1)
            .ok_or(QueueError::ArithmeticOverflow)?;
        let first = (rng.next_u64() as usize) % payment_count;
        let mut second = (rng.next_u64() as usize) % payment_count;
        if second == first {
            second = (second + 1) % payment_count;
        }

        let mut candidate_order = current_order.clone();
        candidate_order.swap(first, second);
        let candidate_simulation = simulate_order(context, balances, payments, &candidate_order)?;
        let comparison = compare_simulations(&candidate_simulation, &current_simulation);

        let accept = if comparison != Ordering::Less {
            comparison == Ordering::Greater || candidate_order < current_order
        } else {
            let loss = normalized_loss(
                &candidate_simulation,
                &current_simulation,
                maximum_value,
                payment_count,
                maximum_delay_penalty,
            );
            rng.unit_interval() < (-loss / temperature).exp()
        };

        if accept {
            current_order = candidate_order.clone();
            current_simulation = candidate_simulation.clone();
        }

        let best_comparison = compare_simulations(&candidate_simulation, &best_simulation);
        if best_comparison == Ordering::Greater
            || (best_comparison == Ordering::Equal && candidate_order < best_order)
        {
            best_order = candidate_order;
            best_simulation = candidate_simulation;
        }

        temperature *= config.cooling_factor;
    }

    Ok(AnnealingResult {
        best_order,
        best_simulation,
        iterations_evaluated,
        seed: config.seed,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context() -> QueueContext {
        QueueContext {
            network_id: "network-test".into(),
            asset_id: "asset-zar-minor".into(),
        }
    }

    fn balance(id: &str, opening: u64) -> ParticipantLiquidity {
        ParticipantLiquidity {
            participant_id: id.into(),
            opening_liquidity_minor: opening,
        }
    }

    fn payment(id: &str, payer: &str, payee: &str, amount: u64, sequence: u64) -> QueuedPayment {
        QueuedPayment {
            payment_id: id.into(),
            network_id: "network-test".into(),
            asset_id: "asset-zar-minor".into(),
            payer: payer.into(),
            payee: payee.into(),
            amount_minor: amount,
            queue_sequence: sequence,
            priority_weight: 1,
            deadline_step: None,
            state: PaymentState::Accepted,
        }
    }

    fn recycling_case() -> (Vec<ParticipantLiquidity>, Vec<QueuedPayment>) {
        (
            vec![balance("A", 10), balance("B", 0), balance("C", 0)],
            vec![
                payment("feed-b", "A", "B", 6, 2),
                payment("drain-a", "A", "C", 10, 1),
                payment("return-a", "B", "A", 6, 3),
            ],
        )
    }

    #[test]
    fn unlock_order_can_settle_more_value_than_fifo() {
        let (balances, payments) = recycling_case();
        let fifo = fifo_order(&context(), &payments).unwrap();
        let unlock = unlock_potential_order(&context(), &payments).unwrap();
        let fifo_report = simulate_order(&context(), &balances, &payments, &fifo).unwrap();
        let unlock_report = simulate_order(&context(), &balances, &payments, &unlock).unwrap();

        assert_eq!(fifo_report.total_settled_minor, 10);
        assert_eq!(unlock_report.total_settled_minor, 22);
        assert_eq!(unlock_report.settled_count, 3);
        assert!(compare_simulations(&unlock_report, &fifo_report).is_gt());
    }

    #[test]
    fn settlement_trace_conserves_liquidity_and_explains_each_step() {
        let (balances, payments) = recycling_case();
        let order = unlock_potential_order(&context(), &payments).unwrap();
        let report = simulate_order(&context(), &balances, &payments, &order).unwrap();

        assert_eq!(report.opening_liquidity_total_minor, 10);
        assert_eq!(report.closing_liquidity_total_minor, 10);
        assert_eq!(report.total_settled_minor, 22);
        assert_eq!(report.events.len(), 3);
        assert_eq!(report.passes_inspected, report.passes_with_progress + 1);
        for event in &report.events {
            assert_eq!(
                event.payer_balance_before_minor,
                event.payer_balance_after_minor + event.amount_minor
            );
            assert_eq!(
                event.payee_balance_after_minor,
                event.payee_balance_before_minor + event.amount_minor
            );
        }
    }

    #[test]
    fn insufficient_liquidity_never_creates_a_settlement() {
        let balances = vec![balance("A", 0), balance("B", 0)];
        let payments = vec![payment("p", "A", "B", 1, 1)];
        let order = fifo_order(&context(), &payments).unwrap();
        let report = simulate_order(&context(), &balances, &payments, &order).unwrap();

        assert_eq!(report.total_settled_minor, 0);
        assert!(report.settled_payment_ids.is_empty());
        assert_eq!(report.pending_payment_ids, vec!["p"]);
        assert_eq!(
            report.opening_liquidity_total_minor,
            report.closing_liquidity_total_minor
        );
    }

    #[test]
    fn nonaccepted_and_cross_profile_payments_are_preserved_as_excluded() {
        let balances = vec![balance("A", 5), balance("B", 0)];
        let mut proposed = payment("proposed", "A", "B", 1, 1);
        proposed.state = PaymentState::Proposed;
        let mut foreign = payment("foreign", "A", "B", 1, 2);
        foreign.asset_id = "asset-usd-minor".into();
        let active = payment("active", "A", "B", 2, 3);
        let payments = vec![proposed, foreign, active];

        let order = fifo_order(&context(), &payments).unwrap();
        let report = simulate_order(&context(), &balances, &payments, &order).unwrap();

        assert_eq!(report.total_settled_minor, 2);
        assert_eq!(report.excluded_payments.len(), 2);
        assert_eq!(
            report.excluded_payments[0],
            ExcludedPayment {
                payment_id: "foreign".into(),
                reason: ExclusionReason::ProfileMismatch,
            }
        );
        assert_eq!(
            report.excluded_payments[1],
            ExcludedPayment {
                payment_id: "proposed".into(),
                reason: ExclusionReason::Proposed,
            }
        );
    }

    #[test]
    fn identical_observations_dedupe_and_conflicting_ids_fail_closed() {
        let balances = vec![balance("A", 5), balance("B", 0)];
        let p = payment("p", "A", "B", 2, 1);
        let order = fifo_order(&context(), &[p.clone(), p.clone()]).unwrap();
        let report = simulate_order(&context(), &balances, &[p.clone(), p], &order).unwrap();
        assert_eq!(report.deduplicated_payment_observations, 1);
        assert_eq!(report.total_settled_minor, 2);

        let conflicting = payment("p", "A", "B", 3, 1);
        assert_eq!(
            fifo_order(&context(), &[payment("p", "A", "B", 2, 1), conflicting,]),
            Err(QueueError::ConflictingDuplicatePayment("p".into()))
        );
    }

    #[test]
    fn queue_order_must_be_an_exact_permutation_of_eligible_ids() {
        let balances = vec![balance("A", 5), balance("B", 0)];
        let payments = vec![payment("p1", "A", "B", 1, 1), payment("p2", "A", "B", 1, 2)];
        assert_eq!(
            simulate_order(
                &context(),
                &balances,
                &payments,
                &["p1".into(), "p1".into()]
            ),
            Err(QueueError::InvalidQueueOrder)
        );
    }

    #[test]
    fn exact_oracle_evaluates_every_small_permutation_and_beats_fifo_or_ties() {
        let (balances, payments) = recycling_case();
        let oracle = exact_order_oracle(&context(), &balances, &payments).unwrap();
        let fifo = simulate_order(
            &context(),
            &balances,
            &payments,
            &fifo_order(&context(), &payments).unwrap(),
        )
        .unwrap();

        assert_eq!(oracle.candidate_orders_evaluated, 6);
        assert!(compare_simulations(&oracle.best_simulation, &fifo).is_ge());
        assert_eq!(oracle.best_simulation.total_settled_minor, 22);
    }

    #[test]
    fn exact_oracle_rejects_batches_above_bound() {
        let balances = vec![balance("A", 100), balance("B", 0)];
        let payments = (0..=MAX_EXACT_ORDER_PAYMENTS)
            .map(|i| payment(&format!("p-{i}"), "A", "B", 1, i as u64))
            .collect::<Vec<_>>();
        assert_eq!(
            exact_order_oracle(&context(), &balances, &payments),
            Err(QueueError::TooManyExactPayments {
                observed: MAX_EXACT_ORDER_PAYMENTS + 1,
                maximum: MAX_EXACT_ORDER_PAYMENTS,
            })
        );
    }

    #[test]
    fn annealing_is_reproducible_for_a_pinned_seed() {
        let (balances, payments) = recycling_case();
        let initial = fifo_order(&context(), &payments).unwrap();
        let config = AnnealingConfig {
            iterations: 150,
            seed: 0x5A17,
            initial_temperature: 0.35,
            cooling_factor: 0.985,
        };
        let first =
            simulated_annealing_order(&context(), &balances, &payments, &initial, &config).unwrap();
        let second =
            simulated_annealing_order(&context(), &balances, &payments, &initial, &config).unwrap();

        assert_eq!(first, second);
        assert_eq!(first.best_simulation.total_settled_minor, 22);
        assert_eq!(first.iterations_evaluated, 150);
    }

    #[test]
    fn invalid_annealing_temperature_fails_closed() {
        let (balances, payments) = recycling_case();
        let config = AnnealingConfig {
            iterations: 10,
            seed: 1,
            initial_temperature: f64::NAN,
            cooling_factor: 0.9,
        };
        assert_eq!(
            simulated_annealing_order(
                &context(),
                &balances,
                &payments,
                &fifo_order(&context(), &payments).unwrap(),
                &config,
            ),
            Err(QueueError::InvalidAnnealingConfig)
        );
    }

    #[test]
    fn blocked_payment_past_deadline_counts_as_missed() {
        let balances = vec![balance("A", 0), balance("B", 0)];
        let mut p = payment("late", "A", "B", 1, 1);
        p.deadline_step = Some(0);
        let order = fifo_order(&context(), &[p.clone()]).unwrap();
        let report = simulate_order(&context(), &balances, &[p], &order).unwrap();

        assert_eq!(report.settled_count, 0);
        assert_eq!(report.pending_payment_ids, vec!["late"]);
        assert_eq!(report.missed_deadline_count, 1);
    }

    #[test]
    fn strict_sequence_calculates_prefix_deficit_not_queue_retry_result() {
        let (_, payments) = recycling_case();
        let fifo = fifo_order(&context(), &payments).unwrap();
        let low_liquidity_order = vec![
            "feed-b".to_string(),
            "return-a".to_string(),
            "drain-a".to_string(),
        ];

        let fifo_report =
            minimum_opening_liquidity_for_sequence(&context(), &payments, &fifo).unwrap();
        let optimized_report =
            minimum_opening_liquidity_for_sequence(&context(), &payments, &low_liquidity_order)
                .unwrap();

        assert_eq!(fifo, vec!["drain-a", "feed-b", "return-a"]);
        assert_eq!(fifo_report.total_required_opening_liquidity_minor, 16);
        assert_eq!(optimized_report.total_required_opening_liquidity_minor, 10);
        assert_eq!(optimized_report.required_by_participant.len(), 3);
        assert_eq!(
            optimized_report
                .required_by_participant
                .iter()
                .find(|x| x.participant_id == "A")
                .unwrap()
                .required_opening_liquidity_minor,
            10
        );
        assert_eq!(
            optimized_report
                .required_by_participant
                .iter()
                .filter(|x| x.participant_id != "A")
                .map(|x| x.required_opening_liquidity_minor)
                .sum::<u128>(),
            0
        );
        assert!(compare_minimum_liquidity_reports(&optimized_report, &fifo_report).is_gt());
    }

    #[test]
    fn minimum_liquidity_oracle_checks_all_orders_and_finds_ten_unit_schedule() {
        let (_, payments) = recycling_case();
        let fifo = fifo_order(&context(), &payments).unwrap();
        let fifo_report =
            minimum_opening_liquidity_for_sequence(&context(), &payments, &fifo).unwrap();
        let oracle = minimum_liquidity_order_oracle(&context(), &payments).unwrap();

        assert_eq!(oracle.candidate_orders_evaluated, 6);
        assert_eq!(
            oracle.best_report.total_required_opening_liquidity_minor,
            10
        );
        assert_eq!(
            oracle.best_report.total_payment_value_minor,
            payments
                .iter()
                .map(|p| p.amount_minor as u128)
                .sum::<u128>()
        );
        assert!(compare_minimum_liquidity_reports(&oracle.best_report, &fifo_report).is_gt());
        let replay =
            minimum_opening_liquidity_for_sequence(&context(), &payments, &oracle.best_order)
                .unwrap();
        assert_eq!(replay, oracle.best_report);
    }

    #[test]
    fn minimum_liquidity_annealing_is_reproducible_and_no_worse_than_fifo() {
        let (_, payments) = recycling_case();
        let initial = fifo_order(&context(), &payments).unwrap();
        let fifo_report =
            minimum_opening_liquidity_for_sequence(&context(), &payments, &initial).unwrap();
        let config = AnnealingConfig {
            iterations: 150,
            seed: 0x5A17,
            initial_temperature: 0.35,
            cooling_factor: 0.985,
        };
        let first =
            simulated_annealing_minimum_liquidity_order(&context(), &payments, &initial, &config)
                .unwrap();
        let second =
            simulated_annealing_minimum_liquidity_order(&context(), &payments, &initial, &config)
                .unwrap();

        assert_eq!(first, second);
        assert_eq!(first.iterations_evaluated, 150);
        assert!(
            first.best_report.total_required_opening_liquidity_minor
                <= fifo_report.total_required_opening_liquidity_minor
        );
        assert_eq!(first.best_report.total_required_opening_liquidity_minor, 10);
    }

    #[test]
    fn minimum_liquidity_oracle_fails_closed_above_eight_payments() {
        let payments = (0..=MAX_EXACT_ORDER_PAYMENTS)
            .map(|i| payment(&format!("p-{i}"), "A", "B", 1, i as u64))
            .collect::<Vec<_>>();
        assert_eq!(
            minimum_liquidity_order_oracle(&context(), &payments),
            Err(QueueError::TooManyExactPayments {
                observed: MAX_EXACT_ORDER_PAYMENTS + 1,
                maximum: MAX_EXACT_ORDER_PAYMENTS,
            })
        );
    }

    #[test]
    fn minimum_liquidity_report_preserves_ineligible_observations() {
        let mut proposed = payment("proposed", "A", "B", 1, 1);
        proposed.state = PaymentState::Proposed;
        let active = payment("active", "A", "B", 2, 2);
        let order = vec!["active".to_string()];
        let report =
            minimum_opening_liquidity_for_sequence(&context(), &[proposed, active], &order)
                .unwrap();

        assert_eq!(report.total_payment_value_minor, 2);
        assert_eq!(
            report.excluded_payments,
            vec![ExcludedPayment {
                payment_id: "proposed".into(),
                reason: ExclusionReason::Proposed,
            }]
        );
    }

    #[test]
    fn unknown_participant_on_accepted_payment_fails_closed() {
        let balances = vec![balance("A", 10)];
        let payments = vec![payment("p", "A", "missing", 1, 1)];
        assert_eq!(
            simulate_order(&context(), &balances, &payments, &vec!["p".into()]),
            Err(QueueError::UnknownParticipant("missing".into()))
        );
    }

    #[test]
    fn recipient_balance_overflow_is_an_error_not_wrapping_arithmetic() {
        let balances = vec![balance("A", 1), balance("B", u64::MAX)];
        let payments = vec![payment("p", "A", "B", 1, 1)];
        let order = fifo_order(&context(), &payments).unwrap();
        assert_eq!(
            simulate_order(&context(), &balances, &payments, &order),
            Err(QueueError::ArithmeticOverflow)
        );
    }
}

//! Online arrival/deadline replay for bounded-window payment ordering experiments.
//!
//! This is a synthetic simulator, not rail I/O or a model of any particular RTGS rulebook.
//! Each committed payment occupies one settlement tick. A global outage set blocks settlement
//! on listed ticks. Participants' available cash is not directly modelled here; liquidity is
//! evaluated from the resulting full-day gross order after the replay has completed.

use crate::{
    evaluate_order, heuristic_portfolio, net_position_liquidity_lower_bound,
    receiver_ready_event_driven, Evaluation, Payment, Problem, ReorderError,
    MAX_SEARCH_WORK_UNITS,
};
use std::collections::BTreeSet;
use std::error::Error;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OnlineBatchStrategy {
    /// Preserve oldest-first order within each committed batch.
    Fifo,
    /// Order each committed batch by deadline, then release tick, then original index.
    EarliestDeadlineFirst,
    /// Reorder each batch with the event-driven liquidity-aware greedy heuristic.
    LiquidityGreedy,
    /// Use the bounded composite heuristic portfolio within each committed batch.
    LiquidityPortfolio,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StreamReplayConfig {
    /// Release/arrival tick for each payment, indexed like Problem::payments().
    pub release_ticks: Vec<u64>,
    /// Inclusive deadline tick for each payment, indexed like Problem::payments().
    pub deadline_ticks: Vec<u64>,
    /// Maximum payments committed into one batch; must be nonzero.
    pub batch_size: usize,
    /// Maximum time to wait from the oldest eligible payment before flushing a partial batch.
    pub max_batch_wait_ticks: u64,
    /// Settlement is unavailable at these global ticks; the set is deterministic.
    pub unavailable_ticks: BTreeSet<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StreamReplayResult {
    /// Global payment indices in actual replay settlement order.
    pub order: Vec<usize>,
    /// Settlement tick for each original payment index.
    pub settlement_ticks: Vec<u64>,
    pub deadline_misses: usize,
    pub total_queue_wait_ticks: u128,
    pub max_queue_wait_ticks: u64,
    pub p95_queue_wait_ticks: u64,
    pub batches_processed: usize,
    pub maximum_batch_size_observed: usize,
    /// Liquidity metrics calculated from the final gross order across all batches.
    pub evaluation: Evaluation,
    /// Certified net-position floor for the same gross obligations.
    pub lower_bound_aggregate_liquidity_minor: u128,
    pub lower_bound_max_participant_liquidity_minor: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StreamReplayError {
    TimingLengthMismatch {
        payments: usize,
        release_ticks: usize,
        deadline_ticks: usize,
    },
    ZeroBatchSize,
    Unschedulable,
    TimeOverflow,
    Reorder(ReorderError),
}

impl fmt::Display for StreamReplayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for StreamReplayError {}

impl From<ReorderError> for StreamReplayError {
    fn from(value: ReorderError) -> Self {
        Self::Reorder(value)
    }
}

fn can_settle_now(predecessors: &[Vec<usize>], payment_index: usize, settled: &[bool]) -> bool {
    predecessors[payment_index]
        .iter()
        .all(|&before| settled[before])
}

fn batch_order(
    problem: &Problem,
    batch: &[usize],
    config: &StreamReplayConfig,
    strategy: OnlineBatchStrategy,
) -> Result<Vec<usize>, StreamReplayError> {
    if batch.len() <= 1 || strategy == OnlineBatchStrategy::Fifo {
        return Ok(batch.to_vec());
    }

    if strategy == OnlineBatchStrategy::EarliestDeadlineFirst {
        let mut result = batch.to_vec();
        result.sort_by_key(|&index| {
            (config.deadline_ticks[index], config.release_ticks[index], index)
        });
        return Ok(result);
    }

    // Every member of this batch is already precedence-ready, so no unsatisfied hard edge
    // can connect two batch members. Preserve global indices through a local subproblem.
    let payments = batch
        .iter()
        .map(|&index| problem.payments()[index].clone())
        .collect::<Vec<Payment>>();
    let subproblem = Problem::new(problem.participant_count(), payments)?;
    let result = match strategy {
        OnlineBatchStrategy::Fifo | OnlineBatchStrategy::EarliestDeadlineFirst => {
            unreachable!("handled above")
        }
        OnlineBatchStrategy::LiquidityGreedy => receiver_ready_event_driven(&subproblem)?,
        OnlineBatchStrategy::LiquidityPortfolio => {
            let count = batch.len();
            let budget = count
                .saturating_mul(4)
                .clamp(40, 1_000)
                .min(MAX_SEARCH_WORK_UNITS / count.max(1));
            heuristic_portfolio(&subproblem, budget, 41, budget)?
        }
    };
    Ok(result.order.into_iter().map(|local_index| batch[local_index]).collect())
}

/// Replay a time-stamped payment stream using bounded look-ahead batches.
///
/// The next batch is formed from released and precedence-ready payments. The scheduler waits
/// for more arrivals only until max_batch_wait_ticks, unless a batch fills or an eligible
/// deadline falls inside the waiting window. Once a batch is committed, it settles in that
/// order; arrivals during its execution join the next batch. This exposes the liquidity-vs-delay
/// trade-off instead of assuming the whole day is known in advance.
pub fn replay_stream(
    problem: &Problem,
    config: &StreamReplayConfig,
    strategy: OnlineBatchStrategy,
) -> Result<StreamReplayResult, StreamReplayError> {
    let count = problem.payments().len();
    if config.release_ticks.len() != count || config.deadline_ticks.len() != count {
        return Err(StreamReplayError::TimingLengthMismatch {
            payments: count,
            release_ticks: config.release_ticks.len(),
            deadline_ticks: config.deadline_ticks.len(),
        });
    }
    if config.batch_size == 0 {
        return Err(StreamReplayError::ZeroBatchSize);
    }

    let mut predecessors = vec![Vec::<usize>::new(); count];
    for &(before, after) in problem.precedence_constraints() {
        predecessors[after].push(before);
    }
    let mut release_order = (0..count).collect::<Vec<_>>();
    release_order.sort_by_key(|&index| (config.release_ticks[index], index));
    let mut release_order_cursor = 0_usize;

    let mut settled = vec![false; count];
    let mut order = Vec::with_capacity(count);
    let mut settlement_ticks = vec![0_u64; count];
    let mut current_tick = 0_u64;
    let mut batches_processed = 0_usize;
    let mut maximum_batch_size_observed = 0_usize;

    while order.len() < count {
        while release_order_cursor < count
            && config.release_ticks[release_order[release_order_cursor]] <= current_tick
        {
            release_order_cursor += 1;
        }
        let next_release = release_order
            .get(release_order_cursor)
            .map(|&index| config.release_ticks[index]);

        let mut eligible = (0..count)
            .filter(|&index| {
                !settled[index]
                    && config.release_ticks[index] <= current_tick
                    && can_settle_now(&predecessors, index, &settled)
            })
            .collect::<Vec<_>>();

        if eligible.is_empty() {
            if let Some(next_tick) = next_release {
                current_tick = next_tick;
                continue;
            }
            // A valid Problem has an acyclic precedence graph. If all remaining payments
            // have arrived but none is eligible, fail closed rather than spin.
            return Err(StreamReplayError::Unschedulable);
        }

        eligible.sort_by_key(|&index| (config.release_ticks[index], index));
        let oldest_release = config.release_ticks[eligible[0]];
        let wait_deadline = oldest_release.saturating_add(config.max_batch_wait_ticks);
        let full = eligible.len() >= config.batch_size;
        let wait_expired = current_tick >= wait_deadline;
        let deadline_pressure = eligible
            .iter()
            .any(|&index| config.deadline_ticks[index] <= wait_deadline);
        let no_useful_future_release =
            next_release.is_none_or(|next_tick| next_tick > wait_deadline);

        if !full && !wait_expired && !deadline_pressure && !no_useful_future_release {
            current_tick = next_release.unwrap_or(wait_deadline);
            continue;
        }

        let batch = eligible
            .into_iter()
            .take(config.batch_size)
            .collect::<Vec<_>>();
        let ordered_batch = batch_order(problem, &batch, config, strategy)?;
        batches_processed = batches_processed
            .checked_add(1)
            .ok_or(StreamReplayError::TimeOverflow)?;
        maximum_batch_size_observed = maximum_batch_size_observed.max(ordered_batch.len());

        for index in ordered_batch {
            while config.unavailable_ticks.contains(&current_tick) {
                current_tick = current_tick
                    .checked_add(1)
                    .ok_or(StreamReplayError::TimeOverflow)?;
            }
            if config.release_ticks[index] > current_tick {
                // A committed batch must not contain a payment whose release is still future.
                return Err(StreamReplayError::Unschedulable);
            }
            settlement_ticks[index] = current_tick;
            settled[index] = true;
            order.push(index);
            current_tick = current_tick
                .checked_add(1)
                .ok_or(StreamReplayError::TimeOverflow)?;
        }
    }

    let evaluation = evaluate_order(problem, &order)?;
    let lower_bound = net_position_liquidity_lower_bound(problem)?;
    let mut waits = Vec::with_capacity(count);
    let mut total_queue_wait_ticks = 0_u128;
    let mut max_queue_wait_ticks = 0_u64;
    let mut deadline_misses = 0_usize;
    for index in 0..count {
        let wait = settlement_ticks[index].saturating_sub(config.release_ticks[index]);
        waits.push(wait);
        total_queue_wait_ticks = total_queue_wait_ticks
            .checked_add(wait as u128)
            .ok_or(StreamReplayError::TimeOverflow)?;
        max_queue_wait_ticks = max_queue_wait_ticks.max(wait);
        if settlement_ticks[index] > config.deadline_ticks[index] {
            deadline_misses = deadline_misses
                .checked_add(1)
                .ok_or(StreamReplayError::TimeOverflow)?;
        }
    }
    waits.sort_unstable();
    let p95_index = if waits.is_empty() {
        0
    } else {
        ((waits.len() * 95).div_ceil(100)).saturating_sub(1)
    };
    let p95_queue_wait_ticks = waits.get(p95_index).copied().unwrap_or(0);

    Ok(StreamReplayResult {
        order,
        settlement_ticks,
        deadline_misses,
        total_queue_wait_ticks,
        max_queue_wait_ticks,
        p95_queue_wait_ticks,
        batches_processed,
        maximum_batch_size_observed,
        evaluation,
        lower_bound_aggregate_liquidity_minor: lower_bound
            .aggregate_required_initial_liquidity_minor,
        lower_bound_max_participant_liquidity_minor: lower_bound
            .max_participant_required_initial_liquidity_minor,
    })
}

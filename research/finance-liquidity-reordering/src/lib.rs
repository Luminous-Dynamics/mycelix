#![forbid(unsafe_code)]

//! Research kernel for ordering a fixed set of gross transfers to reduce required
//! initial participant liquidity. This crate is deliberately independent of rail I/O,
//! legal-netting authorization, settlement finality, credit facilities, and real payment
//! data. Amounts are integer minor units; all reported results are model outputs.

use std::cmp::Ordering;
use std::collections::{BTreeSet, BinaryHeap};
use std::error::Error;
use std::fmt;

pub const MAX_REORDER_PAYMENTS: usize = 100_000;
pub const MAX_PARTICIPANTS: usize = 100_000;
pub const MAX_EXACT_ORACLE_PAYMENTS: usize = 8;
/// Receiver-ready greedy scans the remaining queue on each tick (quadratic work).
pub const MAX_GREEDY_PAYMENTS: usize = 5_000;
/// Per-search bound on candidate count multiplied by payment count.
pub const MAX_SEARCH_WORK_UNITS: usize = 20_000_000;
/// Additional cap on heap entries created by the event-driven queue heuristic.
pub const MAX_EVENT_QUEUE_REFRESHES: usize = 1_000_000;

mod stream_replay;
pub use stream_replay::{
    replay_stream, replay_stream_with_liquidity_budget, BudgetedStreamReplayResult,
    OnlineBatchStrategy, StreamReplayConfig, StreamReplayError, StreamReplayResult,
};


#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Payment {
    pub id: String,
    pub from: usize,
    pub to: usize,
    pub amount_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    participant_count: usize,
    payments: Vec<Payment>,
    /// Hard precedence edges (payment index that must occur first, payment index that follows).
    precedence_constraints: Vec<(usize, usize)>,
}

impl Problem {
    pub fn new(participant_count: usize, payments: Vec<Payment>) -> Result<Self, ReorderError> {
        if participant_count == 0 {
            return Err(ReorderError::EmptyParticipantSet);
        }
        if participant_count > MAX_PARTICIPANTS {
            return Err(ReorderError::TooManyParticipants {
                observed: participant_count,
                maximum: MAX_PARTICIPANTS,
            });
        }
        if payments.len() > MAX_REORDER_PAYMENTS {
            return Err(ReorderError::TooManyPayments {
                observed: payments.len(),
                maximum: MAX_REORDER_PAYMENTS,
            });
        }

        let mut ids = BTreeSet::new();
        for payment in &payments {
            if payment.id.trim().is_empty() {
                return Err(ReorderError::EmptyPaymentId);
            }
            if !ids.insert(payment.id.clone()) {
                return Err(ReorderError::DuplicatePaymentId(payment.id.clone()));
            }
            if payment.amount_minor == 0 {
                return Err(ReorderError::ZeroAmount(payment.id.clone()));
            }
            if payment.from >= participant_count {
                return Err(ReorderError::InvalidParticipantIndex {
                    payment_id: payment.id.clone(),
                    index: payment.from,
                });
            }
            if payment.to >= participant_count {
                return Err(ReorderError::InvalidParticipantIndex {
                    payment_id: payment.id.clone(),
                    index: payment.to,
                });
            }
            if payment.from == payment.to {
                return Err(ReorderError::SelfPayment(payment.id.clone()));
            }
        }

        Ok(Self {
            participant_count,
            payments,
            precedence_constraints: Vec::new(),
        })
    }

    pub fn participant_count(&self) -> usize {
        self.participant_count
    }

    pub fn payments(&self) -> &[Payment] {
        &self.payments
    }

    /// Attach hard precedence constraints, expressed as (before_index, after_index).
    /// Indices refer to payments() order. Cyclic or malformed constraints fail closed.
    pub fn with_precedence_constraints(
        mut self,
        constraints: Vec<(usize, usize)>,
    ) -> Result<Self, ReorderError> {
        let mut seen = BTreeSet::new();
        for &(before, after) in &constraints {
            if before >= self.payments.len()
                || after >= self.payments.len()
                || before == after
            {
                return Err(ReorderError::InvalidPrecedenceConstraint { before, after });
            }
            if !seen.insert((before, after)) {
                return Err(ReorderError::DuplicatePrecedenceConstraint { before, after });
            }
        }
        self.precedence_constraints = constraints;
        self.precedence_constraints.sort_unstable();
        if stable_topological_order(&self).is_none() {
            return Err(ReorderError::PrecedenceCycle);
        }
        Ok(self)
    }

    pub fn precedence_constraints(&self) -> &[(usize, usize)] {
        &self.precedence_constraints
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReorderError {
    EmptyParticipantSet,
    EmptyPaymentId,
    DuplicatePaymentId(String),
    ZeroAmount(String),
    SelfPayment(String),
    InvalidParticipantIndex { payment_id: String, index: usize },
    InvalidOrderLength { expected: usize, observed: usize },
    InvalidPaymentIndex(usize),
    DuplicatePaymentIndex(usize),
    InvalidPrecedenceConstraint { before: usize, after: usize },
    DuplicatePrecedenceConstraint { before: usize, after: usize },
    PrecedenceViolation { before: usize, after: usize },
    PrecedenceCycle,
    TooManyParticipants { observed: usize, maximum: usize },
    TooManyPayments { observed: usize, maximum: usize },
    ExactOracleLimit { observed: usize, maximum: usize },
    GreedyLimit { observed: usize, maximum: usize },
    SearchWorkLimit { observed: usize, maximum: usize },
    ArithmeticOverflow,
}

impl fmt::Display for ReorderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for ReorderError {}

/// Score for one complete gross-transfer order.
///
/// The minimum initial liquidity requirement for participant p is the largest deficit
/// observed in any prefix of the order: max(0, cumulative outflow - cumulative inflow).
/// It is not a credit limit, loss estimate, legal offset, or external liquidity quote.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Evaluation {
    pub required_initial_liquidity_by_participant_minor: Vec<u128>,
    pub aggregate_required_initial_liquidity_minor: u128,
    pub max_participant_required_initial_liquidity_minor: u128,
    pub final_net_balance_by_participant_minor: Vec<i128>,
}

/// Order-independent lower bound derived from participants' final net debit positions.
///
/// Every feasible order must prefund each participant by at least its total outgoing
/// amount minus its total incoming amount when that difference is positive. This is a
/// certified lower bound, not generally the exact minimum: circular flows may require
/// startup liquidity even when every final net position is zero.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LiquidityLowerBound {
    pub required_initial_liquidity_by_participant_minor: Vec<u128>,
    pub aggregate_required_initial_liquidity_minor: u128,
    pub max_participant_required_initial_liquidity_minor: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchResult {
    /// Indices into Problem::payments(), in execution order.
    pub order: Vec<usize>,
    pub evaluation: Evaluation,
    /// Number of candidates/scoring passes considered by this algorithm. Different
    /// algorithms count these differently; use runtime as a separate measured metric.
    pub candidate_evaluations: usize,
}

/// One nondominated ordering under (aggregate required liquidity, maximum participant need).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParetoPoint {
    /// Indices into Problem::payments(), in execution order.
    pub order: Vec<usize>,
    pub evaluation: Evaluation,
}

/// Exact Pareto set for a tiny payment batch. Points are sorted by aggregate liquidity,
/// then maximum participant requirement, then the payment-ID ordering.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParetoFrontier {
    pub points: Vec<ParetoPoint>,
    /// Number of valid precedence-respecting permutations scored exactly.
    pub evaluated_orders: usize,
}

fn deficit(balance: i128) -> u128 {
    if balance < 0 {
        balance.unsigned_abs()
    } else {
        0
    }
}

fn update_required(required: &mut [u128], participant: usize, balance: i128) {
    required[participant] = required[participant].max(deficit(balance));
}

/// Compute a certified, order-independent liquidity lower bound from final net positions.
pub fn net_position_liquidity_lower_bound(
    problem: &Problem,
) -> Result<LiquidityLowerBound, ReorderError> {
    let mut outgoing = vec![0_u128; problem.participant_count];
    let mut incoming = vec![0_u128; problem.participant_count];
    for payment in &problem.payments {
        outgoing[payment.from] = outgoing[payment.from]
            .checked_add(payment.amount_minor as u128)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        incoming[payment.to] = incoming[payment.to]
            .checked_add(payment.amount_minor as u128)
            .ok_or(ReorderError::ArithmeticOverflow)?;
    }

    let mut required = Vec::with_capacity(problem.participant_count);
    let mut aggregate = 0_u128;
    let mut maximum = 0_u128;
    for participant in 0..problem.participant_count {
        let bound = outgoing[participant].saturating_sub(incoming[participant]);
        aggregate = aggregate
            .checked_add(bound)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        maximum = maximum.max(bound);
        required.push(bound);
    }

    Ok(LiquidityLowerBound {
        required_initial_liquidity_by_participant_minor: required,
        aggregate_required_initial_liquidity_minor: aggregate,
        max_participant_required_initial_liquidity_minor: maximum,
    })
}

fn evaluate_unchecked(problem: &Problem, order: &[usize]) -> Result<Evaluation, ReorderError> {
    let mut balances = vec![0_i128; problem.participant_count];
    let mut required = vec![0_u128; problem.participant_count];

    for &index in order {
        let payment = &problem.payments[index];
        let amount = i128::from(payment.amount_minor);

        balances[payment.from] = balances[payment.from]
            .checked_sub(amount)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        update_required(&mut required, payment.from, balances[payment.from]);

        balances[payment.to] = balances[payment.to]
            .checked_add(amount)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        update_required(&mut required, payment.to, balances[payment.to]);
    }

    let mut aggregate = 0_u128;
    let mut maximum = 0_u128;
    for &amount in &required {
        aggregate = aggregate
            .checked_add(amount)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        maximum = maximum.max(amount);
    }

    Ok(Evaluation {
        required_initial_liquidity_by_participant_minor: required,
        aggregate_required_initial_liquidity_minor: aggregate,
        max_participant_required_initial_liquidity_minor: maximum,
        final_net_balance_by_participant_minor: balances,
    })
}

/// Evaluate an order, requiring every payment index to appear exactly once.
pub fn evaluate_order(problem: &Problem, order: &[usize]) -> Result<Evaluation, ReorderError> {
    validate_order(problem, order)?;
    evaluate_unchecked(problem, order)
}

fn validate_order(problem: &Problem, order: &[usize]) -> Result<(), ReorderError> {
    if order.len() != problem.payments.len() {
        return Err(ReorderError::InvalidOrderLength {
            expected: problem.payments.len(),
            observed: order.len(),
        });
    }
    let mut seen = vec![false; problem.payments.len()];
    for &index in order {
        if index >= seen.len() {
            return Err(ReorderError::InvalidPaymentIndex(index));
        }
        if seen[index] {
            return Err(ReorderError::DuplicatePaymentIndex(index));
        }
        seen[index] = true;
    }
    if let Some((before, after)) = violating_precedence(problem, order) {
        return Err(ReorderError::PrecedenceViolation { before, after });
    }
    Ok(())
}

fn violating_precedence(problem: &Problem, order: &[usize]) -> Option<(usize, usize)> {
    if problem.precedence_constraints.is_empty() {
        return None;
    }
    let mut position = vec![0_usize; order.len()];
    for (offset, &index) in order.iter().enumerate() {
        position[index] = offset;
    }
    problem
        .precedence_constraints
        .iter()
        .copied()
        .find(|(before, after)| position[*before] >= position[*after])
}

/// Stable topological order: preserve original payment-index order whenever constraints
/// permit. None means the precedence graph contains a cycle.
fn stable_topological_order(problem: &Problem) -> Option<Vec<usize>> {
    let count = problem.payments.len();
    let mut successors = vec![Vec::new(); count];
    let mut indegree = vec![0_usize; count];
    for &(before, after) in &problem.precedence_constraints {
        successors[before].push(after);
        indegree[after] += 1;
    }
    let mut ready = BTreeSet::new();
    for (index, &degree) in indegree.iter().enumerate() {
        if degree == 0 {
            ready.insert(index);
        }
    }
    let mut order = Vec::with_capacity(count);
    while let Some(index) = ready.pop_first() {
        order.push(index);
        for &after in &successors[index] {
            indegree[after] -= 1;
            if indegree[after] == 0 {
                ready.insert(after);
            }
        }
    }
    (order.len() == count).then_some(order)
}

fn objective_order(a: &Evaluation, b: &Evaluation) -> Ordering {
    a.aggregate_required_initial_liquidity_minor
        .cmp(&b.aggregate_required_initial_liquidity_minor)
        .then_with(|| {
            a.max_participant_required_initial_liquidity_minor
                .cmp(&b.max_participant_required_initial_liquidity_minor)
        })
}

fn order_ids_cmp(problem: &Problem, a: &[usize], b: &[usize]) -> Ordering {
    for (&left, &right) in a.iter().zip(b) {
        let ordering = problem.payments[left].id.cmp(&problem.payments[right].id);
        if ordering != Ordering::Equal {
            return ordering;
        }
    }
    a.len().cmp(&b.len())
}

fn plan_is_better(
    problem: &Problem,
    candidate_order: &[usize],
    candidate: &Evaluation,
    current_order: &[usize],
    current: &Evaluation,
) -> bool {
    match objective_order(candidate, current) {
        Ordering::Less => true,
        Ordering::Greater => false,
        Ordering::Equal => order_ids_cmp(problem, candidate_order, current_order) == Ordering::Less,
    }
}

/// Stable input-order baseline. With precedence constraints, it changes order only as
/// needed to satisfy hard constraints, preserving input order among currently ready items.
pub fn input_order(problem: &Problem) -> Result<SearchResult, ReorderError> {
    let order = stable_topological_order(problem).ok_or(ReorderError::PrecedenceCycle)?;
    let evaluation = evaluate_unchecked(problem, &order)?;
    Ok(SearchResult {
        order,
        evaluation,
        candidate_evaluations: 1,
    })
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct QueueCandidate {
    incremental_prefunding: u128,
    unlock_value: u128,
    payment_id: String,
    payment_index: usize,
    generation: u64,
}

impl Ord for QueueCandidate {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .incremental_prefunding
            .cmp(&self.incremental_prefunding)
            .then_with(|| self.unlock_value.cmp(&other.unlock_value))
            .then_with(|| other.payment_id.cmp(&self.payment_id))
            .then_with(|| self.payment_index.cmp(&other.payment_index))
            .then_with(|| self.generation.cmp(&other.generation))
    }
}

impl PartialOrd for QueueCandidate {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn push_queue_candidate(
    index: usize,
    problem: &Problem,
    balances: &[i128],
    required: &[u128],
    outstanding: &[u128],
    generations: &mut [u64],
    heap: &mut BinaryHeap<QueueCandidate>,
) -> Result<(), ReorderError> {
    let payment = &problem.payments[index];
    let amount = i128::from(payment.amount_minor);
    let post_debit = balances[payment.from]
        .checked_sub(amount)
        .ok_or(ReorderError::ArithmeticOverflow)?;
    let required_after = required[payment.from].max(deficit(post_debit));
    let incremental_prefunding = required_after.saturating_sub(required[payment.from]);
    let unlock_value = (payment.amount_minor as u128).min(outstanding[payment.to]);

    generations[index] = generations[index]
        .checked_add(1)
        .ok_or(ReorderError::ArithmeticOverflow)?;
    heap.push(QueueCandidate {
        incremental_prefunding,
        unlock_value,
        payment_id: payment.id.clone(),
        payment_index: index,
        generation: generations[index],
    });
    Ok(())
}

fn mark_affected_queue_candidate(
    index: usize,
    epoch: usize,
    remaining: &[bool],
    indegree: &[usize],
    seen_epoch: &mut [usize],
    affected: &mut Vec<usize>,
) {
    if remaining[index] && indegree[index] == 0 && seen_epoch[index] != epoch {
        seen_epoch[index] = epoch;
        affected.push(index);
    }
}

/// Event-driven variant of receiver-ready greedy. It keeps a versioned max-heap and
/// refreshes candidates whose sender state, receiver unlock value, or precedence readiness
/// changed after a settlement. Sparse graphs avoid rescanning the full pending set, while
/// high-degree graphs can still be quadratic; an explicit refresh-work cap fails closed.
pub fn receiver_ready_event_driven(problem: &Problem) -> Result<SearchResult, ReorderError> {
    let count = problem.payments.len();
    let mut remaining = vec![true; count];
    let mut outstanding = vec![0_u128; problem.participant_count];
    let mut outgoing_by_participant = vec![Vec::<usize>::new(); problem.participant_count];
    let mut incoming_by_participant = vec![Vec::<usize>::new(); problem.participant_count];
    let mut successors = vec![Vec::<usize>::new(); count];
    let mut indegree = vec![0_usize; count];

    for (index, payment) in problem.payments.iter().enumerate() {
        outstanding[payment.from] = outstanding[payment.from]
            .checked_add(payment.amount_minor as u128)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        outgoing_by_participant[payment.from].push(index);
        incoming_by_participant[payment.to].push(index);
    }
    for &(before, after) in &problem.precedence_constraints {
        successors[before].push(after);
        indegree[after] += 1;
    }

    let mut balances = vec![0_i128; problem.participant_count];
    let mut required = vec![0_u128; problem.participant_count];
    let mut generations = vec![0_u64; count];
    let mut heap = BinaryHeap::<QueueCandidate>::with_capacity(count);
    let mut candidate_evaluations = 0_usize;
    let mut work_units = count;
    if work_units > MAX_SEARCH_WORK_UNITS {
        return Err(ReorderError::SearchWorkLimit {
            observed: work_units,
            maximum: MAX_SEARCH_WORK_UNITS,
        });
    }
    for index in 0..count {
        if indegree[index] == 0 {
            push_queue_candidate(
                index,
                problem,
                &balances,
                &required,
                &outstanding,
                &mut generations,
                &mut heap,
            )?;
            candidate_evaluations = candidate_evaluations
                .checked_add(1)
                .ok_or(ReorderError::ArithmeticOverflow)?;
        }
    }

    let mut order = Vec::with_capacity(count);
    let mut seen_epoch = vec![0_usize; count];
    let mut affected = Vec::<usize>::new();
    let mut epoch = 0_usize;

    while order.len() < count {
        let selected = loop {
            match heap.pop() {
                Some(candidate)
                    if remaining[candidate.payment_index]
                        && indegree[candidate.payment_index] == 0
                        && generations[candidate.payment_index] == candidate.generation =>
                {
                    break candidate.payment_index;
                }
                Some(_) => continue,
                None => return Err(ReorderError::PrecedenceCycle),
            }
        };

        let payment = &problem.payments[selected];
        let from = payment.from;
        let to = payment.to;
        let amount = i128::from(payment.amount_minor);

        balances[from] = balances[from]
            .checked_sub(amount)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        update_required(&mut required, from, balances[from]);
        balances[to] = balances[to]
            .checked_add(amount)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        update_required(&mut required, to, balances[to]);
        outstanding[from] = outstanding[from]
            .checked_sub(payment.amount_minor as u128)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        remaining[selected] = false;
        order.push(selected);

        epoch = epoch.checked_add(1).ok_or(ReorderError::ArithmeticOverflow)?;
        affected.clear();
        for &after in &successors[selected] {
            indegree[after] = indegree[after]
                .checked_sub(1)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            if indegree[after] == 0 {
                mark_affected_queue_candidate(
                    after,
                    epoch,
                    &remaining,
                    &indegree,
                    &mut seen_epoch,
                    &mut affected,
                );
            }
        }

        let refreshed_edges = outgoing_by_participant[from]
            .len()
            .checked_add(outgoing_by_participant[to].len())
            .and_then(|n| n.checked_add(incoming_by_participant[from].len()))
            .and_then(|n| n.checked_add(successors[selected].len()))
            .ok_or(ReorderError::ArithmeticOverflow)?;
        work_units = work_units
            .checked_add(refreshed_edges)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        if work_units > MAX_SEARCH_WORK_UNITS {
            return Err(ReorderError::SearchWorkLimit {
                observed: work_units,
                maximum: MAX_SEARCH_WORK_UNITS,
            });
        }

        for &index in &outgoing_by_participant[from] {
            mark_affected_queue_candidate(
                index,
                epoch,
                &remaining,
                &indegree,
                &mut seen_epoch,
                &mut affected,
            );
        }
        for &index in &outgoing_by_participant[to] {
            mark_affected_queue_candidate(
                index,
                epoch,
                &remaining,
                &indegree,
                &mut seen_epoch,
                &mut affected,
            );
        }
        for &index in &incoming_by_participant[from] {
            mark_affected_queue_candidate(
                index,
                epoch,
                &remaining,
                &indegree,
                &mut seen_epoch,
                &mut affected,
            );
        }

        for &index in &affected {
            push_queue_candidate(
                index,
                problem,
                &balances,
                &required,
                &outstanding,
                &mut generations,
                &mut heap,
            )?;
            candidate_evaluations = candidate_evaluations
                .checked_add(1)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            if candidate_evaluations > MAX_EVENT_QUEUE_REFRESHES {
                return Err(ReorderError::SearchWorkLimit {
                    observed: candidate_evaluations,
                    maximum: MAX_EVENT_QUEUE_REFRESHES,
                });
            }
        }
    }

    let evaluation = evaluate_unchecked(problem, &order)?;
    Ok(SearchResult {
        order,
        evaluation,
        candidate_evaluations,
    })
}

/// Greedy baseline that prioritizes low incremental prefunding needs, then chooses the
/// transfer that can unlock the most still-pending outgoing value at its receiver.
pub fn receiver_ready_greedy(problem: &Problem) -> Result<SearchResult, ReorderError> {
    let count = problem.payments.len();
    if count > MAX_GREEDY_PAYMENTS {
        return Err(ReorderError::GreedyLimit {
            observed: count,
            maximum: MAX_GREEDY_PAYMENTS,
        });
    }
    let mut remaining = vec![true; count];
    let mut outstanding = vec![0_u128; problem.participant_count];
    for payment in &problem.payments {
        outstanding[payment.from] = outstanding[payment.from]
            .checked_add(payment.amount_minor as u128)
            .ok_or(ReorderError::ArithmeticOverflow)?;
    }

    let mut balances = vec![0_i128; problem.participant_count];
    let mut required = vec![0_u128; problem.participant_count];
    let mut order = Vec::with_capacity(count);
    let mut candidate_evaluations = 0_usize;

    while order.len() < count {
        let mut best: Option<(usize, u128, u128)> = None;
        for (index, payment) in problem.payments.iter().enumerate() {
            if !remaining[index] {
                continue;
            }
            if problem
                .precedence_constraints
                .iter()
                .any(|(before, after)| *after == index && remaining[*before])
            {
                continue;
            }
            candidate_evaluations = candidate_evaluations
                .checked_add(1)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            let amount = i128::from(payment.amount_minor);
            let post_debit = balances[payment.from]
                .checked_sub(amount)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            let required_after = required[payment.from].max(deficit(post_debit));
            let incremental_prefunding = required_after.saturating_sub(required[payment.from]);
            let unlock_value = (payment.amount_minor as u128).min(outstanding[payment.to]);

            let replace = match best {
                None => true,
                Some((best_index, best_increment, best_unlock)) => {
                    incremental_prefunding < best_increment
                        || (incremental_prefunding == best_increment && unlock_value > best_unlock)
                        || (incremental_prefunding == best_increment
                            && unlock_value == best_unlock
                            && payment.id < problem.payments[best_index].id)
                }
            };
            if replace {
                best = Some((index, incremental_prefunding, unlock_value));
            }
        }

        let index = best
            .map(|entry| entry.0)
            .ok_or(ReorderError::PrecedenceCycle)?;
        let payment = &problem.payments[index];
        let amount = i128::from(payment.amount_minor);
        balances[payment.from] = balances[payment.from]
            .checked_sub(amount)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        update_required(&mut required, payment.from, balances[payment.from]);
        balances[payment.to] = balances[payment.to]
            .checked_add(amount)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        update_required(&mut required, payment.to, balances[payment.to]);
        outstanding[payment.from] = outstanding[payment.from]
            .checked_sub(payment.amount_minor as u128)
            .ok_or(ReorderError::ArithmeticOverflow)?;

        remaining[index] = false;
        order.push(index);
    }

    let evaluation = evaluate_unchecked(problem, &order)?;
    Ok(SearchResult {
        order,
        evaluation,
        candidate_evaluations,
    })
}

fn move_payment(order: &[usize], from: usize, to: usize) -> Vec<usize> {
    let mut candidate = order.to_vec();
    let payment = candidate.remove(from);
    candidate.insert(to, payment);
    candidate
}

/// Deterministic, budget-bounded insertion local search. It accepts only strict
/// improvements under aggregate-liquidity, then maximum-participant-liquidity,
/// then lexicographic payment-ID order. The budget counts candidate insertions.
pub fn bounded_insertion_search(
    problem: &Problem,
    initial_order: &[usize],
    max_candidate_evaluations: usize,
) -> Result<SearchResult, ReorderError> {
    validate_order(problem, initial_order)?;
    let count = initial_order.len();
    let work_units = count
        .checked_mul(max_candidate_evaluations)
        .ok_or(ReorderError::ArithmeticOverflow)?;
    if work_units > MAX_SEARCH_WORK_UNITS {
        return Err(ReorderError::SearchWorkLimit {
            observed: work_units,
            maximum: MAX_SEARCH_WORK_UNITS,
        });
    }
    let mut order = initial_order.to_vec();
    let mut evaluation = evaluate_unchecked(problem, &order)?;
    if count < 2 || max_candidate_evaluations == 0 {
        return Ok(SearchResult {
            order,
            evaluation,
            candidate_evaluations: 0,
        });
    }

    let mut evaluations = 0_usize;
    let mut from = 0_usize;
    let mut to = 0_usize;
    while from < count && evaluations < max_candidate_evaluations {
        if to >= count {
            from += 1;
            to = 0;
            continue;
        }
        let current_from = from;
        let current_to = to;
        to += 1;
        if current_from == current_to {
            continue;
        }

        let candidate_order = move_payment(&order, current_from, current_to);
        evaluations += 1;
        if violating_precedence(problem, &candidate_order).is_some() {
            continue;
        }
        let candidate_evaluation = evaluate_unchecked(problem, &candidate_order)?;
        if plan_is_better(
            problem,
            &candidate_order,
            &candidate_evaluation,
            &order,
            &evaluation,
        ) {
            order = candidate_order;
            evaluation = candidate_evaluation;
            from = 0;
            to = 0;
        }
    }

    Ok(SearchResult {
        order,
        evaluation,
        candidate_evaluations: evaluations,
    })
}

#[derive(Clone, Debug)]
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0x9e37_79b9_7f4a_7c15
            } else {
                seed
            },
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        value ^ (value >> 31)
    }

    fn unit_f64(&mut self) -> f64 {
        let mantissa = self.next_u64() >> 11;
        (mantissa as f64) * (1.0 / ((1_u64 << 53) as f64))
    }
}

/// Seeded simulated annealing over pair swaps. The returned plan is the best feasible
/// order seen according to the stated lexicographic objective, not necessarily optimal.
/// Repeating a seed and iteration budget yields identical output.
pub fn simulated_annealing(
    problem: &Problem,
    initial_order: &[usize],
    seed: u64,
    iterations: usize,
) -> Result<SearchResult, ReorderError> {
    validate_order(problem, initial_order)?;
    let count = initial_order.len();
    let work_units = count
        .checked_mul(iterations)
        .ok_or(ReorderError::ArithmeticOverflow)?;
    if work_units > MAX_SEARCH_WORK_UNITS {
        return Err(ReorderError::SearchWorkLimit {
            observed: work_units,
            maximum: MAX_SEARCH_WORK_UNITS,
        });
    }
    let mut current_order = initial_order.to_vec();
    let mut current_evaluation = evaluate_unchecked(problem, &current_order)?;
    let mut best_order = current_order.clone();
    let mut best_evaluation = current_evaluation.clone();
    let mut rng = SplitMix64::new(seed);

    if count < 2 || iterations == 0 {
        return Ok(SearchResult {
            order: best_order,
            evaluation: best_evaluation,
            candidate_evaluations: 0,
        });
    }

    let mut temperature =
        ((current_evaluation.aggregate_required_initial_liquidity_minor as f64) * 0.05).max(1.0);
    let cooling = 0.01_f64.powf(1.0 / iterations as f64);

    for _ in 0..iterations {
        let from = (rng.next_u64() as usize) % count;
        let mut to = (rng.next_u64() as usize) % count;
        if to == from {
            to = (to + 1) % count;
        }

        let mut candidate_order = current_order.clone();
        candidate_order.swap(from, to);
        if violating_precedence(problem, &candidate_order).is_some() {
            temperature = (temperature * cooling).max(1e-9);
            continue;
        }
        let candidate_evaluation = evaluate_unchecked(problem, &candidate_order)?;
        let objective = objective_order(&candidate_evaluation, &current_evaluation);
        let accept = if plan_is_better(
            problem,
            &candidate_order,
            &candidate_evaluation,
            &current_order,
            &current_evaluation,
        ) {
            true
        } else {
            let delta = if objective == Ordering::Greater {
                if candidate_evaluation.aggregate_required_initial_liquidity_minor
                    != current_evaluation.aggregate_required_initial_liquidity_minor
                {
                    candidate_evaluation
                        .aggregate_required_initial_liquidity_minor
                        .abs_diff(current_evaluation.aggregate_required_initial_liquidity_minor)
                        as f64
                } else {
                    candidate_evaluation
                        .max_participant_required_initial_liquidity_minor
                        .abs_diff(
                            current_evaluation.max_participant_required_initial_liquidity_minor,
                        ) as f64
                }
            } else {
                0.0
            };
            rng.unit_f64() < (-delta / temperature).exp()
        };

        if accept {
            current_order = candidate_order;
            current_evaluation = candidate_evaluation;
            if plan_is_better(
                problem,
                &current_order,
                &current_evaluation,
                &best_order,
                &best_evaluation,
            ) {
                best_order.clone_from(&current_order);
                best_evaluation.clone_from(&current_evaluation);
            }
        }
        temperature = (temperature * cooling).max(1e-9);
    }

    Ok(SearchResult {
        order: best_order,
        evaluation: best_evaluation,
        candidate_evaluations: iterations,
    })
}

/// Evaluate a bounded portfolio of complementary heuristics and return the best plan.
///
/// The portfolio always considers input order, receiver-ready greedy, insertion search,
/// and seeded simulated annealing. Its wall-clock cost is necessarily higher than a
/// single baseline; compare solution quality and runtime separately. This is a candidate
/// for an algorithm portfolio, not a claim that feature-based algorithm selection is
/// already trained or validated.
pub fn heuristic_portfolio(
    problem: &Problem,
    insertion_candidate_budget: usize,
    annealing_seed: u64,
    annealing_iterations: usize,
) -> Result<SearchResult, ReorderError> {
    let count = problem.payments.len();
    let fifo = input_order(problem)?;
    let mut best = fifo;
    let mut total_evaluations = best.candidate_evaluations;

    // Preserve the original full-scan baseline at sizes where its O(n²) work is capped.
    // Above that limit, use the event-driven heap as the greedy candidate generator.
    if count <= MAX_GREEDY_PAYMENTS {
        let greedy = receiver_ready_greedy(problem)?;
        total_evaluations = total_evaluations
            .checked_add(greedy.candidate_evaluations)
            .ok_or(ReorderError::ArithmeticOverflow)?;
        if plan_is_better(
            problem,
            &greedy.order,
            &greedy.evaluation,
            &best.order,
            &best.evaluation,
        ) {
            best.order = greedy.order;
            best.evaluation = greedy.evaluation;
        }
    }

    match receiver_ready_event_driven(problem) {
        Ok(event_driven) => {
            total_evaluations = total_evaluations
                .checked_add(event_driven.candidate_evaluations)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            if plan_is_better(
                problem,
                &event_driven.order,
                &event_driven.evaluation,
                &best.order,
                &best.evaluation,
            ) {
                best.order = event_driven.order;
                best.evaluation = event_driven.evaluation;
            }
        }
        // On a small, high-degree graph, event refresh overhead can exceed its cap while
        // full-scan greedy is still within its declared limit. Continue with that baseline.
        Err(ReorderError::SearchWorkLimit { .. }) if count <= MAX_GREEDY_PAYMENTS => {}
        Err(error) => return Err(error),
    }

    let local = bounded_insertion_search(problem, &best.order, insertion_candidate_budget)?;
    total_evaluations = total_evaluations
        .checked_add(local.candidate_evaluations)
        .ok_or(ReorderError::ArithmeticOverflow)?;
    if plan_is_better(
        problem,
        &local.order,
        &local.evaluation,
        &best.order,
        &best.evaluation,
    ) {
        best.order = local.order;
        best.evaluation = local.evaluation;
    }

    let annealed = simulated_annealing(
        problem,
        &best.order,
        annealing_seed,
        annealing_iterations,
    )?;
    total_evaluations = total_evaluations
        .checked_add(annealed.candidate_evaluations)
        .ok_or(ReorderError::ArithmeticOverflow)?;
    if plan_is_better(
        problem,
        &annealed.order,
        &annealed.evaluation,
        &best.order,
        &best.evaluation,
    ) {
        best.order = annealed.order;
        best.evaluation = annealed.evaluation;
    }

    best.candidate_evaluations = total_evaluations;
    Ok(best)
}

/// Exhaustive permutation oracle for tiny batches. The input order is not assumed to be
/// good. This function fails closed above eight payments instead of implying optimality.
pub fn exact_permutation_oracle(problem: &Problem) -> Result<SearchResult, ReorderError> {
    let count = problem.payments.len();
    if count > MAX_EXACT_ORACLE_PAYMENTS {
        return Err(ReorderError::ExactOracleLimit {
            observed: count,
            maximum: MAX_EXACT_ORACLE_PAYMENTS,
        });
    }

    fn visit(
        problem: &Problem,
        prefix: &mut Vec<usize>,
        used: &mut [bool],
        best_order: &mut Option<Vec<usize>>,
        best_evaluation: &mut Option<Evaluation>,
        evaluations: &mut usize,
    ) -> Result<(), ReorderError> {
        if prefix.len() == problem.payments.len() {
            let candidate = evaluate_unchecked(problem, prefix)?;
            *evaluations = evaluations
                .checked_add(1)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            let better = match (best_order.as_ref(), best_evaluation.as_ref()) {
                (Some(order), Some(evaluation)) => {
                    plan_is_better(problem, prefix, &candidate, order, evaluation)
                }
                _ => true,
            };
            if better {
                *best_order = Some(prefix.clone());
                *best_evaluation = Some(candidate);
            }
            return Ok(());
        }

        for index in 0..problem.payments.len() {
            if used[index] {
                continue;
            }
            if problem
                .precedence_constraints
                .iter()
                .any(|(before, after)| *after == index && !used[*before])
            {
                continue;
            }
            used[index] = true;
            prefix.push(index);
            visit(
                problem,
                prefix,
                used,
                best_order,
                best_evaluation,
                evaluations,
            )?;
            prefix.pop();
            used[index] = false;
        }
        Ok(())
    }

    let mut best_order = None;
    let mut best_evaluation = None;
    let mut evaluations = 0_usize;
    visit(
        problem,
        &mut Vec::with_capacity(count),
        &mut vec![false; count],
        &mut best_order,
        &mut best_evaluation,
        &mut evaluations,
    )?;

    Ok(SearchResult {
        order: best_order.unwrap_or_default(),
        evaluation: best_evaluation.ok_or(ReorderError::ArithmeticOverflow)?,
        candidate_evaluations: evaluations,
    })
}


/// Compute the exact nondominated trade-off between aggregate initial liquidity and
/// maximum single-participant initial liquidity. Unlike the lexicographic oracle, this
/// preserves multiple nondominated options when one improves aggregate liquidity but
/// worsens the largest participant requirement. It fails closed above eight payments.
pub fn exact_pareto_frontier(problem: &Problem) -> Result<ParetoFrontier, ReorderError> {
    let count = problem.payments.len();
    if count > MAX_EXACT_ORACLE_PAYMENTS {
        return Err(ReorderError::ExactOracleLimit {
            observed: count,
            maximum: MAX_EXACT_ORACLE_PAYMENTS,
        });
    }

    fn visit(
        problem: &Problem,
        prefix: &mut Vec<usize>,
        used: &mut [bool],
        frontier: &mut Vec<ParetoPoint>,
        evaluated_orders: &mut usize,
    ) -> Result<(), ReorderError> {
        if prefix.len() == problem.payments.len() {
            let evaluation = evaluate_unchecked(problem, prefix)?;
            *evaluated_orders = evaluated_orders
                .checked_add(1)
                .ok_or(ReorderError::ArithmeticOverflow)?;

            let candidate_aggregate = evaluation.aggregate_required_initial_liquidity_minor;
            let candidate_max = evaluation.max_participant_required_initial_liquidity_minor;
            let candidate_is_redundant = frontier.iter().any(|point| {
                let aggregate = point
                    .evaluation
                    .aggregate_required_initial_liquidity_minor;
                let max_participant =
                    point.evaluation.max_participant_required_initial_liquidity_minor;
                let dominates = aggregate <= candidate_aggregate
                    && max_participant <= candidate_max
                    && (aggregate < candidate_aggregate || max_participant < candidate_max);
                let equal_but_lexically_better = aggregate == candidate_aggregate
                    && max_participant == candidate_max
                    && order_ids_cmp(problem, &point.order, prefix) != Ordering::Greater;
                dominates || equal_but_lexically_better
            });
            if candidate_is_redundant {
                return Ok(());
            }

            frontier.retain(|point| {
                let aggregate = point
                    .evaluation
                    .aggregate_required_initial_liquidity_minor;
                let max_participant =
                    point.evaluation.max_participant_required_initial_liquidity_minor;
                let candidate_dominates = candidate_aggregate <= aggregate
                    && candidate_max <= max_participant
                    && (candidate_aggregate < aggregate || candidate_max < max_participant);
                let equal_but_candidate_lexically_better = candidate_aggregate == aggregate
                    && candidate_max == max_participant
                    && order_ids_cmp(problem, prefix, &point.order) == Ordering::Less;
                !(candidate_dominates || equal_but_candidate_lexically_better)
            });
            frontier.push(ParetoPoint {
                order: prefix.clone(),
                evaluation,
            });
            return Ok(());
        }

        for index in 0..problem.payments.len() {
            if used[index] {
                continue;
            }
            if problem
                .precedence_constraints
                .iter()
                .any(|(before, after)| *after == index && !used[*before])
            {
                continue;
            }
            used[index] = true;
            prefix.push(index);
            visit(problem, prefix, used, frontier, evaluated_orders)?;
            prefix.pop();
            used[index] = false;
        }
        Ok(())
    }

    let mut points = Vec::new();
    let mut evaluated_orders = 0_usize;
    visit(
        problem,
        &mut Vec::with_capacity(count),
        &mut vec![false; count],
        &mut points,
        &mut evaluated_orders,
    )?;
    points.sort_by(|a, b| {
        objective_order(&a.evaluation, &b.evaluation)
            .then_with(|| order_ids_cmp(problem, &a.order, &b.order))
    });

    Ok(ParetoFrontier {
        points,
        evaluated_orders,
    })
}

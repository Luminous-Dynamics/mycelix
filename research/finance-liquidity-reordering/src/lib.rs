#![forbid(unsafe_code)]

//! Research kernel for ordering a fixed set of gross transfers to reduce required
//! initial participant liquidity. This crate is deliberately independent of rail I/O,
//! legal-netting authorization, settlement finality, credit facilities, and real payment
//! data. Amounts are integer minor units; all reported results are model outputs.

use std::cmp::Ordering;
use std::collections::BTreeSet;
use std::error::Error;
use std::fmt;

pub const MAX_REORDER_PAYMENTS: usize = 100_000;
pub const MAX_PARTICIPANTS: usize = 100_000;
pub const MAX_EXACT_ORACLE_PAYMENTS: usize = 8;

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
        })
    }

    pub fn participant_count(&self) -> usize {
        self.participant_count
    }

    pub fn payments(&self) -> &[Payment] {
        &self.payments
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReorderError {
    EmptyParticipantSet,
    EmptyPaymentId,
    DuplicatePaymentId(String),
    ZeroAmount(String),
    SelfPayment(String),
    InvalidParticipantIndex {
        payment_id: String,
        index: usize,
    },
    InvalidOrderLength {
        expected: usize,
        observed: usize,
    },
    InvalidPaymentIndex(usize),
    DuplicatePaymentIndex(usize),
    TooManyParticipants {
        observed: usize,
        maximum: usize,
    },
    TooManyPayments {
        observed: usize,
        maximum: usize,
    },
    ExactOracleLimit {
        observed: usize,
        maximum: usize,
    },
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchResult {
    /// Indices into Problem::payments(), in execution order.
    pub order: Vec<usize>,
    pub evaluation: Evaluation,
    /// Number of candidates/scoring passes considered by this algorithm. Different
    /// algorithms count these differently; use runtime as a separate measured metric.
    pub candidate_evaluations: usize,
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
    Ok(())
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

/// Baseline: preserve the caller's input order exactly.
pub fn input_order(problem: &Problem) -> Result<SearchResult, ReorderError> {
    let order = (0..problem.payments.len()).collect::<Vec<_>>();
    let evaluation = evaluate_unchecked(problem, &order)?;
    Ok(SearchResult {
        order,
        evaluation,
        candidate_evaluations: 1,
    })
}

/// Greedy baseline that prioritizes low incremental prefunding needs, then chooses the
/// transfer that can unlock the most still-pending outgoing value at its receiver.
pub fn receiver_ready_greedy(problem: &Problem) -> Result<SearchResult, ReorderError> {
    let count = problem.payments.len();
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
            candidate_evaluations = candidate_evaluations
                .checked_add(1)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            let amount = i128::from(payment.amount_minor);
            let post_debit = balances[payment.from]
                .checked_sub(amount)
                .ok_or(ReorderError::ArithmeticOverflow)?;
            let required_after = required[payment.from].max(deficit(post_debit));
            let incremental_prefunding =
                required_after.saturating_sub(required[payment.from]);
            let unlock_value = (payment.amount_minor as u128).min(outstanding[payment.to]);

            let replace = match best {
                None => true,
                Some((best_index, best_increment, best_unlock)) => {
                    incremental_prefunding < best_increment
                        || (incremental_prefunding == best_increment
                            && unlock_value > best_unlock)
                        || (incremental_prefunding == best_increment
                            && unlock_value == best_unlock
                            && payment.id < problem.payments[best_index].id)
                }
            };
            if replace {
                best = Some((index, incremental_prefunding, unlock_value));
            }
        }

        let index = best.map(|entry| entry.0).ok_or(ReorderError::ArithmeticOverflow)?;
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
    let mut order = initial_order.to_vec();
    let mut evaluation = evaluate_unchecked(problem, &order)?;
    let count = order.len();
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
        let candidate_evaluation = evaluate_unchecked(problem, &candidate_order)?;
        evaluations += 1;
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

    let mut temperature = ((current_evaluation.aggregate_required_initial_liquidity_minor as f64)
        * 0.05)
        .max(1.0);
    let cooling = 0.01_f64.powf(1.0 / iterations as f64);

    for _ in 0..iterations {
        let from = (rng.next_u64() as usize) % count;
        let mut to = (rng.next_u64() as usize) % count;
        if to == from {
            to = (to + 1) % count;
        }

        let mut candidate_order = current_order.clone();
        candidate_order.swap(from, to);
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

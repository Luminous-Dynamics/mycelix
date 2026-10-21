use finance_liquidity_reordering::{
    bounded_insertion_search, evaluate_order, exact_permutation_oracle, input_order,
    receiver_ready_greedy, simulated_annealing, Evaluation, Payment, Problem, ReorderError,
};
use std::cmp::Ordering;

fn problem(participant_count: usize, payments: Vec<Payment>) -> Problem {
    Problem::new(participant_count, payments).unwrap()
}

fn chain_problem(length: usize, reversed: bool) -> Problem {
    let ids: Vec<usize> = if reversed {
        (0..length).rev().collect()
    } else {
        (0..length).collect()
    };
    let payments = ids
        .into_iter()
        .map(|i| Payment {
            id: format!("p{i:02}"),
            from: i,
            to: i + 1,
            amount_minor: 10,
        })
        .collect();
    problem(length + 1, payments)
}

// This reference uses cumulative in/out totals at each prefix rather than the production
// kernel's signed running-balance representation.
fn independent_measure(problem: &Problem, order: &[usize]) -> Evaluation {
    let mut cumulative_out = vec![0_u128; problem.participant_count()];
    let mut cumulative_in = vec![0_u128; problem.participant_count()];
    let mut required = vec![0_u128; problem.participant_count()];

    for &index in order {
        let payment = &problem.payments()[index];
        let amount = payment.amount_minor as u128;
        cumulative_out[payment.from] += amount;
        cumulative_in[payment.to] += amount;

        for participant in [payment.from, payment.to] {
            let deficit = cumulative_out[participant].saturating_sub(cumulative_in[participant]);
            required[participant] = required[participant].max(deficit);
        }
    }

    let aggregate = required.iter().sum::<u128>();
    let maximum = required.iter().copied().max().unwrap_or(0);
    let final_balances = cumulative_in
        .iter()
        .zip(&cumulative_out)
        .map(|(incoming, outgoing)| *incoming as i128 - *outgoing as i128)
        .collect();

    Evaluation {
        required_initial_liquidity_by_participant_minor: required,
        aggregate_required_initial_liquidity_minor: aggregate,
        max_participant_required_initial_liquidity_minor: maximum,
        final_net_balance_by_participant_minor: final_balances,
    }
}

fn order_ids(problem: &Problem, order: &[usize]) -> Vec<String> {
    order
        .iter()
        .map(|&index| problem.payments()[index].id.clone())
        .collect()
}

fn compare_order_and_score(
    problem: &Problem,
    order_a: &[usize],
    score_a: &Evaluation,
    order_b: &[usize],
    score_b: &Evaluation,
) -> Ordering {
    score_a
        .aggregate_required_initial_liquidity_minor
        .cmp(&score_b.aggregate_required_initial_liquidity_minor)
        .then_with(|| {
            score_a
                .max_participant_required_initial_liquidity_minor
                .cmp(&score_b.max_participant_required_initial_liquidity_minor)
        })
        .then_with(|| order_ids(problem, order_a).cmp(&order_ids(problem, order_b)))
}

fn independent_oracle(problem: &Problem) -> (Vec<usize>, Evaluation) {
    fn visit(
        problem: &Problem,
        prefix: &mut Vec<usize>,
        used: &mut [bool],
        best: &mut Option<(Vec<usize>, Evaluation)>,
    ) {
        if prefix.len() == problem.payments().len() {
            let score = independent_measure(problem, prefix);
            let production = evaluate_order(problem, prefix).unwrap();
            assert_eq!(
                production,
                score,
                "prefix evaluator differs for order {:?}",
                order_ids(problem, prefix)
            );
            let replace = best.as_ref().is_none_or(|(best_order, best_score)| {
                compare_order_and_score(problem, prefix, &score, best_order, best_score)
                    == Ordering::Less
            });
            if replace {
                *best = Some((prefix.clone(), score));
            }
            return;
        }

        for index in 0..problem.payments().len() {
            if used[index] {
                continue;
            }
            used[index] = true;
            prefix.push(index);
            visit(problem, prefix, used, best);
            prefix.pop();
            used[index] = false;
        }
    }

    let mut best = None;
    visit(
        problem,
        &mut Vec::new(),
        &mut vec![false; problem.payments().len()],
        &mut best,
    );
    best.unwrap()
}

#[test]
fn reversed_chain_is_expensive_and_credit_unlock_order_is_cheaper() {
    let problem = chain_problem(6, true);
    let fifo = input_order(&problem).unwrap();
    let greedy = receiver_ready_greedy(&problem).unwrap();

    assert_eq!(
        fifo.evaluation.aggregate_required_initial_liquidity_minor,
        60
    );
    assert_eq!(
        greedy.evaluation.aggregate_required_initial_liquidity_minor,
        10
    );
    assert_eq!(
        greedy
            .evaluation
            .max_participant_required_initial_liquidity_minor,
        10
    );
    assert_eq!(
        order_ids(&problem, &greedy.order),
        vec!["p00", "p01", "p02", "p03", "p04", "p05"]
    );
}

#[test]
fn credit_unlock_greedy_reduces_hub_round_trip_prefunding() {
    let pairs = 3;
    let mut payments = Vec::new();
    for participant in (1..=pairs).rev() {
        payments.push(Payment {
            id: format!("r{participant:04}"),
            from: participant,
            to: 0,
            amount_minor: 10,
        });
    }
    for participant in (1..=pairs).rev() {
        payments.push(Payment {
            id: format!("o{participant:04}"),
            from: 0,
            to: participant,
            amount_minor: 10,
        });
    }
    let problem = problem(pairs + 1, payments);
    let fifo = input_order(&problem).unwrap();
    let greedy = receiver_ready_greedy(&problem).unwrap();
    let exact = exact_permutation_oracle(&problem).unwrap();

    assert_eq!(fifo.evaluation.aggregate_required_initial_liquidity_minor, 30);
    assert_eq!(greedy.evaluation.aggregate_required_initial_liquidity_minor, 10);
    assert_eq!(exact.evaluation.aggregate_required_initial_liquidity_minor, 10);
}

#[test]
fn credit_unlock_greedy_orders_a_closed_cycle_with_one_initial_buffer() {
    let count = 6;
    let payments = (0..count)
        .rev()
        .map(|i| Payment {
            id: format!("p{i:02}"),
            from: i,
            to: (i + 1) % count,
            amount_minor: 10,
        })
        .collect();
    let problem = problem(count, payments);
    let fifo = input_order(&problem).unwrap();
    let greedy = receiver_ready_greedy(&problem).unwrap();
    let exact = exact_permutation_oracle(&problem).unwrap();

    assert_eq!(fifo.evaluation.aggregate_required_initial_liquidity_minor, 50);
    assert_eq!(greedy.evaluation.aggregate_required_initial_liquidity_minor, 10);
    assert_eq!(exact.evaluation.aggregate_required_initial_liquidity_minor, 10);
}

#[test]
fn production_evaluator_matches_independent_prefix_oracle_for_every_small_order() {
    for seed in 0..24_usize {
        let payment_count = 4 + seed % 4;
        let payments = (0..payment_count)
            .map(|i| {
                let from = (seed * 3 + i * 2) % 5;
                let mut to = (seed + i * 3 + 1) % 5;
                if to == from {
                    to = (to + 1) % 5;
                }
                Payment {
                    id: format!("s{seed:02}-p{i:02}"),
                    from,
                    to,
                    amount_minor: ((seed * 5 + i * 7) % 13 + 1) as u64,
                }
            })
            .collect();
        let problem = problem(5, payments);
        let (expected_order, expected_score) = independent_oracle(&problem);
        let exact = exact_permutation_oracle(&problem).unwrap();

        assert_eq!(
            exact.order, expected_order,
            "exact order differs from independent oracle for seed={seed}"
        );
        assert_eq!(exact.evaluation, expected_score, "seed={seed}");
        assert_eq!(
            exact.candidate_evaluations,
            (1..=problem.payments().len()).product::<usize>(),
            "permutation count, seed={seed}"
        );
    }
}

#[test]
fn all_heuristics_are_feasible_and_never_beat_the_exact_oracle() {
    for seed in 0..20_usize {
        let payment_count = 4 + seed % 4;
        let payments = (0..payment_count)
            .map(|i| {
                let from = (seed + i * 3) % 6;
                let mut to = (seed * 2 + i * 5 + 1) % 6;
                if to == from {
                    to = (to + 1) % 6;
                }
                Payment {
                    id: format!("h{seed:02}-p{i:02}"),
                    from,
                    to,
                    amount_minor: ((seed * 11 + i * 9) % 17 + 1) as u64,
                }
            })
            .collect();
        let problem = problem(6, payments);
        let exact = exact_permutation_oracle(&problem).unwrap();
        let fifo = input_order(&problem).unwrap();
        let greedy = receiver_ready_greedy(&problem).unwrap();
        let local = bounded_insertion_search(&problem, &fifo.order, 300).unwrap();
        let annealed = simulated_annealing(&problem, &fifo.order, seed as u64 + 1, 500).unwrap();
        assert_eq!(
            simulated_annealing(
                &problem,
                &fifo.order,
                seed as u64 + 1,
                500
            )
            .unwrap(),
            annealed,
            "seeded annealing must be reproducible for seed={seed}"
        );
        for candidate in [fifo, greedy, local, annealed] {
            assert_eq!(candidate.order.len(), payment_count);
            let remeasured = evaluate_order(&problem, &candidate.order).unwrap();
            assert_eq!(remeasured, candidate.evaluation);
            assert!(
                compare_order_and_score(
                    &problem,
                    &exact.order,
                    &exact.evaluation,
                    &candidate.order,
                    &candidate.evaluation,
                ) != Ordering::Greater,
                "heuristic appears better than exhaustive oracle for seed={seed}"
            );
            assert_eq!(
                candidate
                    .evaluation
                    .final_net_balance_by_participant_minor
                    .iter()
                    .sum::<i128>(),
                0
            );
        }
    }
}

#[test]
fn invalid_orders_and_inputs_fail_closed() {
    let p = chain_problem(3, false);
    assert!(matches!(
        evaluate_order(&p, &[0, 0, 2]),
        Err(ReorderError::DuplicatePaymentIndex(0))
    ));
    assert!(matches!(
        evaluate_order(&p, &[0, 1]),
        Err(ReorderError::InvalidOrderLength { .. })
    ));
    assert!(matches!(
        evaluate_order(&p, &[0, 1, 9]),
        Err(ReorderError::InvalidPaymentIndex(9))
    ));
    assert!(matches!(
        Problem::new(
            2,
            vec![Payment {
                id: String::new(),
                from: 0,
                to: 1,
                amount_minor: 1
            }]
        ),
        Err(ReorderError::EmptyPaymentId)
    ));
}

#[test]
fn exact_oracle_refuses_batches_above_its_declared_limit() {
    let p = chain_problem(9, false);
    assert!(matches!(
        exact_permutation_oracle(&p),
        Err(ReorderError::ExactOracleLimit {
            observed: 9,
            maximum: 8
        })
    ));
}

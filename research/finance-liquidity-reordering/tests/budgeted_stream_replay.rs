use finance_liquidity_reordering::{
    replay_stream_with_liquidity_budget, OnlineBatchStrategy, Payment, Problem, StreamReplayConfig,
    StreamReplayError,
};
use std::collections::BTreeSet;

fn p(id: &str, from: usize, to: usize, amount_minor: u64) -> Payment {
    Payment {
        id: id.into(),
        from,
        to,
        amount_minor,
    }
}

fn cfg(releases: Vec<u64>, deadlines: Vec<u64>, batch_size: usize, wait: u64) -> StreamReplayConfig {
    StreamReplayConfig {
        release_ticks: releases,
        deadline_ticks: deadlines,
        batch_size,
        max_batch_wait_ticks: wait,
        unavailable_ticks: BTreeSet::from([3, 4]),
    }
}

#[test]
fn incoming_payments_recycle_fixed_liquidity_and_unlock_a_previously_unaffordable_payment() {
    let problem = Problem::new(
        3,
        vec![p("spend_first", 1, 2, 5), p("fund_sender", 0, 1, 5)],
    )
    .unwrap();
    let result = replay_stream_with_liquidity_budget(
        &problem,
        &cfg(vec![0, 1], vec![20, 20], 2, 1),
        OnlineBatchStrategy::LiquidityGreedy,
        &[5, 0, 0],
    )
    .unwrap();

    assert!(result.fully_settled);
    assert!(!result.liquidity_deadlock);
    assert_eq!(result.order, vec![1, 0]);
    assert_eq!(result.settlement_ticks, vec![Some(2), Some(1)]);
    assert_eq!(result.initial_liquidity_total_minor, 5);
    assert_eq!(result.total_value_settled_minor, 10);
    assert_eq!(result.total_value_unsettled_minor, 0);
    assert_eq!(result.final_balances_by_participant_minor, vec![0, 0, 5]);
    assert_eq!(
        result
            .evaluation
            .as_ref()
            .unwrap()
            .aggregate_required_initial_liquidity_minor,
        5
    );
}

#[test]
fn zero_funding_cycle_returns_explicit_deadlock_and_partial_result() {
    let problem = Problem::new(
        3,
        vec![p("ab", 0, 1, 10), p("bc", 1, 2, 10), p("ca", 2, 0, 10)],
    )
    .unwrap();
    let result = replay_stream_with_liquidity_budget(
        &problem,
        &cfg(vec![0, 0, 0], vec![10, 10, 10], 3, 0),
        OnlineBatchStrategy::LiquidityGreedy,
        &[0, 0, 0],
    )
    .unwrap();

    assert!(!result.fully_settled);
    assert!(result.liquidity_deadlock);
    assert!(result.order.is_empty());
    assert_eq!(result.unsettled_indices, vec![0, 1, 2]);
    assert_eq!(result.total_value_settled_minor, 0);
    assert_eq!(result.total_value_unsettled_minor, 30);
    assert!(result.evaluation.is_none());
}

#[test]
fn cash_constraints_and_precedence_are_both_hard() {
    let problem = Problem::new(
        3,
        vec![p("fund", 0, 1, 5), p("spend", 1, 2, 5)],
    )
    .unwrap()
    .with_precedence_constraints(vec![(0, 1)])
    .unwrap();
    let result = replay_stream_with_liquidity_budget(
        &problem,
        &cfg(vec![0, 0], vec![10, 10], 2, 0),
        OnlineBatchStrategy::Fifo,
        &[5, 0, 0],
    )
    .unwrap();

    assert!(result.fully_settled);
    assert_eq!(result.order, vec![0, 1]);
    assert_eq!(result.settlement_ticks, vec![Some(0), Some(1)]);
    assert_eq!(result.final_balances_by_participant_minor, vec![0, 0, 5]);
}

#[test]
fn payment_outage_blocks_execution_but_does_not_consume_cash_or_drop_queue_entries() {
    let problem = Problem::new(2, vec![p("one", 0, 1, 3)]).unwrap();
    let result = replay_stream_with_liquidity_budget(
        &problem,
        &cfg(vec![3], vec![10], 1, 0),
        OnlineBatchStrategy::Fifo,
        &[3, 0],
    )
    .unwrap();

    assert!(result.fully_settled);
    assert_eq!(result.settlement_ticks, vec![Some(5)]);
    assert_eq!(result.final_balances_by_participant_minor, vec![0, 3]);
}

#[test]
fn initial_balance_length_mismatch_fails_closed() {
    let problem = Problem::new(2, vec![p("one", 0, 1, 1)]).unwrap();
    let result = replay_stream_with_liquidity_budget(
        &problem,
        &cfg(vec![0], vec![10], 1, 0),
        OnlineBatchStrategy::Fifo,
        &[1],
    );
    assert_eq!(
        result,
        Err(StreamReplayError::InitialBalanceLengthMismatch {
            participants: 2,
            initial_balances: 1,
        })
    );
}

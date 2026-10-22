use finance_liquidity_reordering::{
    evaluate_order, replay_stream, OnlineBatchStrategy, Payment, Problem, StreamReplayConfig,
    StreamReplayError,
};
use std::collections::BTreeSet;

fn payment(id: &str, from: usize, to: usize, amount_minor: u64) -> Payment {
    Payment {
        id: id.into(),
        from,
        to,
        amount_minor,
    }
}

fn config(
    release_ticks: Vec<u64>,
    deadline_ticks: Vec<u64>,
    batch_size: usize,
    max_batch_wait_ticks: u64,
    unavailable_ticks: &[u64],
) -> StreamReplayConfig {
    StreamReplayConfig {
        release_ticks,
        deadline_ticks,
        batch_size,
        max_batch_wait_ticks,
        unavailable_ticks: unavailable_ticks.iter().copied().collect(),
    }
}

#[test]
fn each_strategy_preserves_unique_payments_release_times_precedence_and_outage_ticks() {
    let problem = Problem::new(
        5,
        vec![
            payment("a", 0, 1, 5),
            payment("b", 1, 2, 3),
            payment("c", 2, 0, 2),
            payment("d", 0, 3, 4),
            payment("e", 3, 1, 1),
            payment("f", 4, 0, 2),
        ],
    )
    .unwrap()
    .with_precedence_constraints(vec![(0, 1)])
    .unwrap();
    let config = config(
        vec![0, 0, 1, 2, 3, 5],
        vec![10, 9, 12, 8, 7, 15],
        3,
        2,
        &[2, 3, 7],
    );

    for strategy in [
        OnlineBatchStrategy::Fifo,
        OnlineBatchStrategy::EarliestDeadlineFirst,
        OnlineBatchStrategy::LiquidityGreedy,
        OnlineBatchStrategy::LiquidityPortfolio,
    ] {
        let result = replay_stream(&problem, &config, strategy).unwrap();
        assert_eq!(result.order.len(), problem.payments().len());
        let mut sorted = result.order.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, (0..problem.payments().len()).collect::<Vec<_>>());
        assert_eq!(
            evaluate_order(&problem, &result.order).unwrap(),
            result.evaluation
        );

        let mut positions = vec![0; result.order.len()];
        for (offset, &index) in result.order.iter().enumerate() {
            positions[index] = offset;
            assert!(result.settlement_ticks[index] >= config.release_ticks[index]);
            assert!(!config
                .unavailable_ticks
                .contains(&result.settlement_ticks[index]));
        }
        assert!(
            positions[0] < positions[1],
            "precedence violation in {strategy:?}"
        );
        assert_eq!(
            result.deadline_misses,
            result
                .settlement_ticks
                .iter()
                .enumerate()
                .filter(|(index, tick)| **tick > config.deadline_ticks[*index])
                .count()
        );
        assert!(result.max_queue_wait_ticks >= result.p95_queue_wait_ticks);
        assert!(result.batches_processed > 0);
        assert!(result.maximum_batch_size_observed <= config.batch_size);
        assert!(
            result.evaluation.aggregate_required_initial_liquidity_minor
                >= result.lower_bound_aggregate_liquidity_minor
        );
        assert!(
            result
                .evaluation
                .max_participant_required_initial_liquidity_minor
                >= result.lower_bound_max_participant_liquidity_minor
        );
    }
}

#[test]
fn bounded_batch_wait_trades_delay_for_lower_liquidity_need() {
    // The first payment spends liquidity at participant 1 before the replenishing transfer
    // from participant 0 arrives. A one-tick look-ahead lets the second payment fund the first.
    let problem = Problem::new(
        3,
        vec![
            payment("spend_first", 1, 2, 5),
            payment("fund_sender", 0, 1, 5),
        ],
    )
    .unwrap();
    let immediate = replay_stream(
        &problem,
        &config(vec![0, 1], vec![20, 20], 2, 0, &[]),
        OnlineBatchStrategy::LiquidityGreedy,
    )
    .unwrap();
    let bounded_wait = replay_stream(
        &problem,
        &config(vec![0, 1], vec![20, 20], 2, 1, &[]),
        OnlineBatchStrategy::LiquidityGreedy,
    )
    .unwrap();

    assert_eq!(immediate.order, vec![0, 1]);
    assert_eq!(
        immediate
            .evaluation
            .aggregate_required_initial_liquidity_minor,
        10
    );
    assert_eq!(bounded_wait.order, vec![1, 0]);
    assert_eq!(
        bounded_wait
            .evaluation
            .aggregate_required_initial_liquidity_minor,
        5
    );
    assert!(bounded_wait.total_queue_wait_ticks > immediate.total_queue_wait_ticks);
    assert_eq!(bounded_wait.deadline_misses, 0);
}

#[test]
fn a_settlement_outage_increases_delay_and_can_miss_deadlines() {
    let problem = Problem::new(2, vec![payment("p", 0, 1, 1)]).unwrap();
    let result = replay_stream(
        &problem,
        &config(vec![0], vec![0], 1, 0, &[0, 1, 2]),
        OnlineBatchStrategy::Fifo,
    )
    .unwrap();
    assert_eq!(result.settlement_ticks, vec![3]);
    assert_eq!(result.deadline_misses, 1);
    assert_eq!(result.max_queue_wait_ticks, 3);
}

#[test]
fn invalid_stream_configurations_fail_closed() {
    let problem = Problem::new(2, vec![payment("p", 0, 1, 1)]).unwrap();
    assert_eq!(
        replay_stream(
            &problem,
            &config(vec![], vec![1], 1, 0, &[]),
            OnlineBatchStrategy::Fifo,
        ),
        Err(StreamReplayError::TimingLengthMismatch {
            payments: 1,
            release_ticks: 0,
            deadline_ticks: 1,
        })
    );
    assert_eq!(
        replay_stream(
            &problem,
            &config(vec![0], vec![1], 0, 0, &[]),
            OnlineBatchStrategy::Fifo,
        ),
        Err(StreamReplayError::ZeroBatchSize)
    );
}

use finance_liquidity_reordering::{
    replay_stream, OnlineBatchStrategy, Payment, Problem, StreamReplayConfig, StreamReplayResult,
};
use std::collections::BTreeSet;
use std::error::Error;
use std::time::Instant;

const WARMUPS: usize = 2;
const REPETITIONS: usize = 11;
const PAYMENT_COUNT: usize = 240;
const PARTICIPANT_COUNT: usize = 32;

#[derive(Clone, Copy)]
enum Scenario {
    Staggered,
    Bursty,
    TightDeadlines,
    DependencyHeavy,
}

impl Scenario {
    const ALL: [Self; 4] = [
        Self::Staggered,
        Self::Bursty,
        Self::TightDeadlines,
        Self::DependencyHeavy,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::Staggered => "staggered_arrivals",
            Self::Bursty => "bursty_arrivals",
            Self::TightDeadlines => "tight_deadlines",
            Self::DependencyHeavy => "dependency_heavy",
        }
    }
}

fn make_case(
    scenario: Scenario,
) -> Result<(Problem, StreamReplayConfig), Box<dyn Error>> {
    let payments = (0..PAYMENT_COUNT)
        .map(|i| {
            let from = (i * 7 + 3) % PARTICIPANT_COUNT;
            let mut to = (i * 11 + 9) % PARTICIPANT_COUNT;
            if from == to {
                to = (to + 1) % PARTICIPANT_COUNT;
            }
            Payment {
                id: format!("stream-{i:04}"),
                from,
                to,
                amount_minor: ((i * 17 + i / 3) % 97 + 1) as u64,
            }
        })
        .collect::<Vec<_>>();
    let mut problem = Problem::new(PARTICIPANT_COUNT, payments)?;
    if matches!(scenario, Scenario::DependencyHeavy) {
        let constraints = (8..PAYMENT_COUNT)
            .step_by(8)
            .map(|after| (after - 1, after))
            .collect();
        problem = problem.with_precedence_constraints(constraints)?;
    }

    let mut release_ticks = Vec::with_capacity(PAYMENT_COUNT);
    let mut deadline_ticks = Vec::with_capacity(PAYMENT_COUNT);
    for i in 0..PAYMENT_COUNT {
        let release = match scenario {
            Scenario::Staggered => (i / 3) as u64,
            Scenario::Bursty => ((i / 24) * 5) as u64,
            Scenario::TightDeadlines => (i / 5) as u64,
            Scenario::DependencyHeavy => (i / 4) as u64,
        };
        let slack = match scenario {
            Scenario::Staggered => 75 + (i % 20) as u64,
            Scenario::Bursty => 18 + (i % 15) as u64,
            Scenario::TightDeadlines => 3 + (i % 4) as u64,
            Scenario::DependencyHeavy => 65 + (i % 18) as u64,
        };
        release_ticks.push(release);
        deadline_ticks.push(release.saturating_add(slack));
    }

    let unavailable_ticks = (35_u64..=39)
        .chain(105_u64..=110)
        .collect::<BTreeSet<_>>();
    Ok((
        problem,
        StreamReplayConfig {
            release_ticks,
            deadline_ticks,
            batch_size: 1,
            max_batch_wait_ticks: 0,
            unavailable_ticks,
        },
    ))
}

fn signed_delta(reference: u128, observed: u128) -> String {
    if reference >= observed {
        (reference - observed).to_string()
    } else {
        format!("-{}", observed - reference)
    }
}

fn percentile(samples: &[u128], percentile: usize) -> u128 {
    let rank = (samples.len() * percentile).div_ceil(100).max(1);
    samples[rank.min(samples.len()) - 1]
}

fn measure(
    problem: &Problem,
    config: &StreamReplayConfig,
    strategy: OnlineBatchStrategy,
) -> Result<(StreamReplayResult, Vec<u128>), Box<dyn Error>> {
    for _ in 0..WARMUPS {
        std::hint::black_box(replay_stream(problem, config, strategy)?);
    }
    let mut samples = Vec::with_capacity(REPETITIONS);
    let mut last = None;
    for _ in 0..REPETITIONS {
        let start = Instant::now();
        let result = replay_stream(problem, config, strategy)?;
        samples.push(start.elapsed().as_nanos());
        last = Some(result);
    }
    samples.sort_unstable();
    Ok((
        last.ok_or("no measured replay")?,
        samples,
    ))
}

fn emit(
    scenario: Scenario,
    problem: &Problem,
    config: &StreamReplayConfig,
    strategy: OnlineBatchStrategy,
    baseline: &StreamReplayResult,
) -> Result<(), Box<dyn Error>> {
    let (result, samples) = measure(problem, config, strategy)?;
    if result.order.len() != problem.payments().len() {
        return Err("replay did not settle every payment".into());
    }
    let mut sorted = result.order.clone();
    sorted.sort_unstable();
    if sorted != (0..problem.payments().len()).collect::<Vec<_>>() {
        return Err("replay order is not a permutation".into());
    }
    for (index, &tick) in result.settlement_ticks.iter().enumerate() {
        if tick < config.release_ticks[index] {
            return Err(format!("payment {index} settled before release").into());
        }
        if config.unavailable_ticks.contains(&tick) {
            return Err(format!("payment {index} settled during outage").into());
        }
    }

    println!(
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        scenario.label(),
        PAYMENT_COUNT,
        PARTICIPANT_COUNT,
        strategy_label(strategy),
        config.batch_size,
        config.max_batch_wait_ticks,
        config.unavailable_ticks.len(),
        result.batches_processed,
        result.maximum_batch_size_observed,
        result.deadline_misses,
        result.total_queue_wait_ticks,
        result.max_queue_wait_ticks,
        result.p95_queue_wait_ticks,
        result.evaluation.aggregate_required_initial_liquidity_minor,
        result.evaluation.max_participant_required_initial_liquidity_minor,
        signed_delta(
            baseline.evaluation.aggregate_required_initial_liquidity_minor,
            result.evaluation.aggregate_required_initial_liquidity_minor
        ),
        signed_delta(
            baseline.total_queue_wait_ticks,
            result.total_queue_wait_ticks
        ),
        percentile(&samples, 50),
        percentile(&samples, 95),
        percentile(&samples, 99)
    );
    Ok(())
}

fn strategy_label(strategy: OnlineBatchStrategy) -> &'static str {
    match strategy {
        OnlineBatchStrategy::Fifo => "fifo",
        OnlineBatchStrategy::EarliestDeadlineFirst => "earliest_deadline_first",
        OnlineBatchStrategy::LiquidityGreedy => "liquidity_greedy",
        OnlineBatchStrategy::LiquidityPortfolio => "liquidity_portfolio",
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    println!("scenario,payments,participants,strategy,batch_size,max_batch_wait_ticks,outage_tick_count,batches_processed,maximum_batch_size_observed,deadline_misses,total_queue_wait_ticks,max_queue_wait_ticks,p95_queue_wait_ticks,aggregate_required_initial_liquidity_minor,max_participant_required_initial_liquidity_minor,aggregate_saving_vs_fifo_minor,queue_wait_reduction_vs_fifo_ticks,p50_ns,p95_ns,p99_ns");

    for scenario in Scenario::ALL {
        let (problem, mut base_config) = make_case(scenario)?;
        for batch_size in [1_usize, 8, 32] {
            for max_wait in [0_u64, 2, 10] {
                for strategy in [
                    OnlineBatchStrategy::Fifo,
                    OnlineBatchStrategy::EarliestDeadlineFirst,
                    OnlineBatchStrategy::LiquidityGreedy,
                    OnlineBatchStrategy::LiquidityPortfolio,
                ] {
                    base_config.batch_size = batch_size;
                    base_config.max_batch_wait_ticks = max_wait;
                    let config_baseline = replay_stream(
                        &problem,
                        &base_config,
                        OnlineBatchStrategy::Fifo,
                    )?;
                    emit(
                        scenario,
                        &problem,
                        &base_config,
                        strategy,
                        &config_baseline,
                    )?;
                }
            }
        }
    }
    Ok(())
}

use finance_liquidity_reordering::{
    replay_stream_with_liquidity_budget, OnlineBatchStrategy, Payment, Problem,
    StreamReplayConfig, BudgetedStreamReplayResult,
};
use std::collections::BTreeSet;
use std::error::Error;
use std::time::Instant;

const WARMUPS: usize = 1;
const REPETITIONS: usize = 7;
const PAYMENT_COUNT: usize = 48;
const PARTICIPANT_COUNT: usize = 16;

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

#[derive(Clone, Copy)]
enum FundingProfile {
    Zero,
    DistributedQuarterOutgoing,
    DistributedHalfOutgoing,
    ConcentratedQuarterGross,
}

impl FundingProfile {
    const ALL: [Self; 4] = [
        Self::Zero,
        Self::DistributedQuarterOutgoing,
        Self::DistributedHalfOutgoing,
        Self::ConcentratedQuarterGross,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::Zero => "zero",
            Self::DistributedQuarterOutgoing => "distributed_quarter_outgoing",
            Self::DistributedHalfOutgoing => "distributed_half_outgoing",
            Self::ConcentratedQuarterGross => "concentrated_quarter_gross_at_p0",
        }
    }
}

fn make_case(
    scenario: Scenario,
) -> Result<(Problem, Vec<u64>, Vec<u64>), Box<dyn Error>> {
    let payments = (0..PAYMENT_COUNT)
        .map(|i| {
            let from = (i * 7 + 3) % PARTICIPANT_COUNT;
            let mut to = (i * 11 + 9) % PARTICIPANT_COUNT;
            if from == to {
                to = (to + 1) % PARTICIPANT_COUNT;
            }
            Payment {
                id: format!("budget-{i:03}"),
                from,
                to,
                amount_minor: ((i * 17 + i / 3) % 97 + 1) as u64,
            }
        })
        .collect::<Vec<_>>();
    let mut problem = Problem::new(PARTICIPANT_COUNT, payments)?;
    if matches!(scenario, Scenario::DependencyHeavy) {
        let constraints = (6..PAYMENT_COUNT)
            .step_by(6)
            .map(|after| (after - 1, after))
            .collect();
        problem = problem.with_precedence_constraints(constraints)?;
    }

    let mut releases = Vec::with_capacity(PAYMENT_COUNT);
    let mut deadlines = Vec::with_capacity(PAYMENT_COUNT);
    for i in 0..PAYMENT_COUNT {
        let release = match scenario {
            Scenario::Staggered => (i / 2) as u64,
            Scenario::Bursty => ((i / 8) * 3) as u64,
            Scenario::TightDeadlines => (i / 3) as u64,
            Scenario::DependencyHeavy => (i / 2) as u64,
        };
        let slack = match scenario {
            Scenario::Staggered => 45 + (i % 12) as u64,
            Scenario::Bursty => 12 + (i % 8) as u64,
            Scenario::TightDeadlines => 3 + (i % 3) as u64,
            Scenario::DependencyHeavy => 40 + (i % 10) as u64,
        };
        releases.push(release);
        deadlines.push(release.saturating_add(slack));
    }
    Ok((problem, releases, deadlines))
}

fn initial_balances(problem: &Problem, profile: FundingProfile) -> Vec<u128> {
    let mut outgoing = vec![0_u128; problem.participant_count()];
    let mut total_gross = 0_u128;
    for payment in problem.payments() {
        let amount = u128::from(payment.amount_minor);
        outgoing[payment.from] += amount;
        total_gross += amount;
    }
    match profile {
        FundingProfile::Zero => vec![0; problem.participant_count()],
        FundingProfile::DistributedQuarterOutgoing => {
            outgoing.into_iter().map(|amount| amount / 4).collect()
        }
        FundingProfile::DistributedHalfOutgoing => {
            outgoing.into_iter().map(|amount| amount / 2).collect()
        }
        FundingProfile::ConcentratedQuarterGross => {
            let mut balances = vec![0; problem.participant_count()];
            balances[0] = total_gross / 4;
            balances
        }
    }
}

fn make_config(
    release_ticks: &[u64],
    deadline_ticks: &[u64],
    batch_size: usize,
    max_wait: u64,
) -> StreamReplayConfig {
    StreamReplayConfig {
        release_ticks: release_ticks.to_vec(),
        deadline_ticks: deadline_ticks.to_vec(),
        batch_size,
        max_batch_wait_ticks: max_wait,
        unavailable_ticks: [20_u64, 21, 50].into_iter().collect::<BTreeSet<_>>(),
    }
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
    balances: &[u128],
) -> Result<(BudgetedStreamReplayResult, Vec<u128>), Box<dyn Error>> {
    for _ in 0..WARMUPS {
        std::hint::black_box(replay_stream_with_liquidity_budget(
            problem, config, strategy, balances
        )?);
    }
    let mut samples = Vec::with_capacity(REPETITIONS);
    let mut last = None;
    for _ in 0..REPETITIONS {
        let start = Instant::now();
        let result = replay_stream_with_liquidity_budget(problem, config, strategy, balances)?;
        samples.push(start.elapsed().as_nanos());
        last = Some(result);
    }
    samples.sort_unstable();
    Ok((last.ok_or("no measured replay")?, samples))
}

fn value_or_blank(value: Option<u128>) -> String {
    value.map(|value| value.to_string()).unwrap_or_default()
}

fn emit(
    scenario: Scenario,
    profile: FundingProfile,
    problem: &Problem,
    config: &StreamReplayConfig,
    strategy: OnlineBatchStrategy,
    balances: &[u128],
    baseline: &BudgetedStreamReplayResult,
) -> Result<(), Box<dyn Error>> {
    let (result, samples) = measure(problem, config, strategy, balances)?;
    if result.order.len() + result.unsettled_indices.len() != PAYMENT_COUNT {
        return Err("settled/unsettled partition lost payments".into());
    }
    if result.order.iter().any(|&index| result.unsettled_indices.contains(&index)) {
        return Err("payment appears as settled and unsettled".into());
    }
    let mut coverage = result.order.clone();
    coverage.extend(result.unsettled_indices.iter().copied());
    coverage.sort_unstable();
    if coverage != (0..PAYMENT_COUNT).collect::<Vec<_>>() {
        return Err("settled/unsettled partition is not a payment-index partition".into());
    }
    let gross_total = problem
        .payments()
        .iter()
        .map(|payment| u128::from(payment.amount_minor))
        .sum::<u128>();
    if result.total_value_settled_minor + result.total_value_unsettled_minor != gross_total {
        return Err("settled and unsettled values do not conserve gross obligations".into());
    }
    if result.initial_liquidity_total_minor
        != result.final_balances_by_participant_minor.iter().sum::<u128>()
    {
        return Err("gross-transfer cash conservation failed".into());
    }
    if result.fully_settled != result.unsettled_indices.is_empty() {
        return Err("fully_settled disagrees with unsettled list".into());
    }
    if result.fully_settled != result.evaluation.is_some() {
        return Err("full-order evaluation must exist only on full settlement".into());
    }

    let (aggregate_need, max_participant_need) = result
        .evaluation
        .as_ref()
        .map(|evaluation| (
            value_or_blank(Some(evaluation.aggregate_required_initial_liquidity_minor)),
            value_or_blank(Some(evaluation.max_participant_required_initial_liquidity_minor)),
        ))
        .unwrap_or_else(|| (String::new(), String::new()));

    println!(
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        scenario.label(),
        profile.label(),
        strategy_label(strategy),
        PAYMENT_COUNT,
        PARTICIPANT_COUNT,
        result.initial_liquidity_total_minor,
        config.batch_size,
        config.max_batch_wait_ticks,
        config.unavailable_ticks.len(),
        result.order.len(),
        result.unsettled_indices.len(),
        usize::from(result.liquidity_deadlock),
        result.deadline_misses,
        result.total_queue_wait_ticks,
        result.max_queue_wait_ticks,
        result.p95_queue_wait_ticks,
        result.total_value_settled_minor,
        result.total_value_unsettled_minor,
        result.final_balances_by_participant_minor.iter().sum::<u128>(),
        result.batches_processed,
        result.maximum_batch_size_observed,
        aggregate_need,
        max_participant_need,
        signed_delta(
            baseline.total_value_settled_minor,
            result.total_value_settled_minor,
        ),
        signed_delta(
            baseline.deadline_misses as u128,
            result.deadline_misses as u128,
        ),
        signed_delta(baseline.total_queue_wait_ticks, result.total_queue_wait_ticks),
        percentile(&samples, 50),
        percentile(&samples, 95),
        percentile(&samples, 99),
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
    println!("scenario,funding_profile,strategy,payments,participants,initial_liquidity_total_minor,batch_size,max_batch_wait_ticks,outage_tick_count,settled_count,unsettled_count,liquidity_deadlock,deadline_misses,total_queue_wait_ticks,max_queue_wait_ticks,p95_queue_wait_ticks,settled_value_minor,unsettled_value_minor,final_balance_total_minor,batches_processed,maximum_batch_size_observed,full_order_aggregate_liquidity_minor,full_order_max_participant_need_minor,settled_value_delta_vs_fifo_minor,deadline_misses_reduction_vs_fifo,queue_wait_reduction_vs_fifo_ticks,p50_ns,p95_ns,p99_ns");

    for scenario in Scenario::ALL {
        let (problem, releases, deadlines) = make_case(scenario)?;
        for profile in FundingProfile::ALL {
            let balances = initial_balances(&problem, profile);
            for batch_size in [1_usize, 8, 32] {
                for max_wait in [0_u64, 2, 10] {
                    let config = make_config(&releases, &deadlines, batch_size, max_wait);
                    let baseline = replay_stream_with_liquidity_budget(
                        &problem,
                        &config,
                        OnlineBatchStrategy::Fifo,
                        &balances,
                    )?;
                    for strategy in [
                        OnlineBatchStrategy::Fifo,
                        OnlineBatchStrategy::EarliestDeadlineFirst,
                        OnlineBatchStrategy::LiquidityGreedy,
                        OnlineBatchStrategy::LiquidityPortfolio,
                    ] {
                        emit(
                            scenario,
                            profile,
                            &problem,
                            &config,
                            strategy,
                            &balances,
                            &baseline,
                        )?;
                    }
                }
            }
        }
    }
    Ok(())
}

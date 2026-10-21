use finance_route_clearing_optimizer::queue::{
    simulate_queue, PaymentOrder, QueueAccount, QueueContext, QueuePolicy, QueueRoute, TickWindow,
};
use std::time::Instant;

const PARTICIPANTS: usize = 16;
const HORIZON: u64 = 99;
const WARMUP_REPETITIONS: usize = 5;
const REPETITIONS: usize = 101;
const SIZES: [usize; 3] = [70, 140, 700];

#[derive(Clone, Copy, Debug)]
enum Scenario {
    Baseline,
    ScarceLiquidity,
    OutageHeavy,
}

impl Scenario {
    fn name(self) -> &'static str {
        match self {
            Self::Baseline => "baseline",
            Self::ScarceLiquidity => "scarce_liquidity",
            Self::OutageHeavy => "outage_heavy",
        }
    }
}

fn fixture(
    payment_count: usize,
    scenario: Scenario,
) -> (
    QueueContext,
    Vec<QueueAccount>,
    Vec<QueueRoute>,
    Vec<PaymentOrder>,
) {
    let context = QueueContext {
        rail_id: "synthetic-zar-rtgs".into(),
        asset_id: "ZAR-minor-unit".into(),
        policy_version: "queue-bench-v1".into(),
        horizon_tick: HORIZON,
    };
    let liquidity_multiplier = match scenario {
        Scenario::ScarceLiquidity => 1,
        Scenario::Baseline | Scenario::OutageHeavy => 5,
    };
    let accounts = (0..PARTICIPANTS)
        .map(|i| QueueAccount {
            participant_id: format!("P{i:02}"),
            // Deterministic heterogeneous starting liquidity, including bottlenecks.
            balance_minor: ((i * 7) % 13) as u64 * liquidity_multiplier,
        })
        .collect::<Vec<_>>();

    let routes = (0..PARTICIPANTS)
        .map(|i| {
            let outage = match scenario {
                Scenario::Baseline => i % 5 == 0,
                Scenario::ScarceLiquidity => false,
                Scenario::OutageHeavy => i % 2 == 0,
            };
            let outages = if outage {
                let (start_tick, end_tick) = if matches!(scenario, Scenario::OutageHeavy) {
                    (10, 40)
                } else {
                    (20, 30)
                };
                vec![TickWindow {
                    start_tick,
                    end_tick,
                }]
            } else {
                Vec::new()
            };
            QueueRoute {
                route_id: format!("R{i:02}"),
                rail_id: context.rail_id.clone(),
                asset_id: context.asset_id.clone(),
                policy_version: context.policy_version.clone(),
                enabled: true,
                capacity_minor_per_window: if matches!(scenario, Scenario::OutageHeavy) {
                    45
                } else {
                    70
                },
                capacity_window_ticks: 10,
                fixed_fee_units: 3,
                fee_units_per_minor: 1,
                outages,
            }
        })
        .collect::<Vec<_>>();

    let payments = (0..payment_count)
        .map(|i| {
            let participant = i % PARTICIPANTS;
            let amount_minor = ((i * 17 + 3) % 5 + 1) as u64;
            let arrival_tick = ((i * 13) % 80) as u64;
            PaymentOrder {
                payment_id: format!("PAY{i:04}"),
                rail_id: context.rail_id.clone(),
                asset_id: context.asset_id.clone(),
                policy_version: context.policy_version.clone(),
                route_id: format!("R{participant:02}"),
                from: format!("P{participant:02}"),
                to: format!("P{:02}", (participant + 1) % PARTICIPANTS),
                amount_minor,
                arrival_tick,
                deadline_tick: Some(arrival_tick.saturating_add(15)),
                priority: ((i * 11) % 10) as u8,
            }
        })
        .collect::<Vec<_>>();

    (context, accounts, routes, payments)
}

fn percentile(samples: &mut [u128], percentile: usize) -> u128 {
    samples.sort_unstable();
    let index = ((samples.len() - 1) * percentile).div_ceil(100);
    samples[index.min(samples.len() - 1)]
}

fn main() {
    println!(
        "scenario,payments,policy,settled_count,unsettled_count,excluded_count,settled_minor,late_settled,missed_deadlines,modeled_fee_units,total_queue_wait_ticks,p95_queue_wait_ticks,max_queue_wait_ticks,p95_participant_balance_time_minor_ticks,max_participant_peak_balance_minor,p50_ns,p95_ns,p99_ns"
    );

    for scenario in [
        Scenario::Baseline,
        Scenario::ScarceLiquidity,
        Scenario::OutageHeavy,
    ] {
        for payment_count in SIZES {
            let (context, accounts, routes, payments) = fixture(payment_count, scenario);
            for policy in [
                QueuePolicy::StrictFifo,
                QueuePolicy::LiquidityAwareFifo,
                QueuePolicy::PriorityScan,
                QueuePolicy::AgingPriorityScan { interval_ticks: 1 },
                QueuePolicy::AgingPriorityScan { interval_ticks: 5 },
                QueuePolicy::AgingPriorityScan { interval_ticks: 20 },
                QueuePolicy::EarliestDeadlineFirst,
            ] {
                // Warm the code path before measuring. Keep warmup samples out of the
                // reported distributions.
                for _ in 0..WARMUP_REPETITIONS {
                    let _ = simulate_queue(&context, &accounts, &routes, &payments, policy)
                        .expect("synthetic warmup fixture must be valid");
                }

                let mut timings = Vec::with_capacity(REPETITIONS);
                let mut final_result = None;
                for _ in 0..REPETITIONS {
                    let start = Instant::now();
                    let result = simulate_queue(&context, &accounts, &routes, &payments, policy)
                        .expect("synthetic benchmark fixture must be valid");
                    timings.push(start.elapsed().as_nanos());
                    final_result = Some(result);
                }

                let result = final_result.expect("at least one repetition");
                let p50 = percentile(&mut timings.clone(), 50);
                let p95 = percentile(&mut timings.clone(), 95);
                let p99 = percentile(&mut timings, 99);
                let mut balance_time = result
                    .account_metrics
                    .iter()
                    .map(|a| a.balance_time_minor_ticks)
                    .collect::<Vec<_>>();
                let p95_balance_time = percentile(&mut balance_time, 95);
                let p95_queue_wait = result
                    .p95_queue_wait_ticks
                    .expect("synthetic trace must contain queue wait samples");
                let max_queue_wait = result
                    .max_queue_wait_ticks
                    .expect("synthetic trace must contain queue wait samples");
                let max_peak_balance = result
                    .account_metrics
                    .iter()
                    .map(|a| a.maximum_observed_balance_minor)
                    .max()
                    .unwrap_or(0);

                println!(
                    "{},{payment_count},{policy:?},{},{},{},{},{},{},{},{},{},{},{p95_balance_time},{max_peak_balance},{p50},{p95},{p99}",
                    scenario.name(),
                    result.settled_count,
                    result.unsettled_count,
                    result.excluded_count,
                    result.settled_minor,
                    result.late_settled_count,
                    result.missed_deadline_count,
                    result.modeled_fee_units,
                    result.total_queue_wait_ticks,
                    p95_queue_wait,
                    max_queue_wait,
                );
            }
        }
    }
}

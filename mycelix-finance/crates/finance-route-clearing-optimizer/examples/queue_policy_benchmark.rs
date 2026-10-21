use finance_route_clearing_optimizer::queue::{
    simulate_queue, PaymentOrder, QueueAccount, QueueContext, QueuePolicy, QueueRoute, TickWindow,
};
use std::time::Instant;

const PARTICIPANTS: usize = 16;
const PAYMENT_COUNT: usize = 700;
const HORIZON: u64 = 99;
const REPETITIONS: usize = 9;

fn fixture() -> (
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
    let accounts = (0..PARTICIPANTS)
        .map(|i| QueueAccount {
            participant_id: format!("P{i:02}"),
            // Deterministic heterogeneous starting liquidity, including bottlenecks.
            balance_minor: ((i * 7) % 13) as u64 * 5,
        })
        .collect::<Vec<_>>();
    let routes = (0..PARTICIPANTS)
        .map(|i| {
            let mut outages = Vec::new();
            if i % 5 == 0 {
                outages.push(TickWindow {
                    start_tick: 20,
                    end_tick: 30,
                });
            }
            QueueRoute {
                route_id: format!("R{i:02}"),
                rail_id: context.rail_id.clone(),
                asset_id: context.asset_id.clone(),
                policy_version: context.policy_version.clone(),
                enabled: true,
                capacity_minor_per_window: 70,
                capacity_window_ticks: 10,
                fixed_fee_units: 3,
                fee_units_per_minor: 1,
                outages,
            }
        })
        .collect::<Vec<_>>();
    let payments = (0..PAYMENT_COUNT)
        .map(|i| {
            let participant = i % PARTICIPANTS;
            let amount = ((i * 17 + 3) % 5 + 1) as u64;
            let arrival_tick = (i % 100) as u64;
            PaymentOrder {
                payment_id: format!("PAY{i:04}"),
                rail_id: context.rail_id.clone(),
                asset_id: context.asset_id.clone(),
                policy_version: context.policy_version.clone(),
                route_id: format!("R{participant:02}"),
                from: format!("P{participant:02}"),
                to: format!("P{:02}", (participant + 1) % PARTICIPANTS),
                amount_minor: amount,
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
    let (context, accounts, routes, payments) = fixture();
    println!(
        "policy,payments,settled_count,unsettled_count,excluded_count,settled_minor,late_settled,missed_deadlines,modeled_fee_units,p50_ns,p95_ns"
    );
    for policy in [
        QueuePolicy::StrictFifo,
        QueuePolicy::LiquidityAwareFifo,
        QueuePolicy::PriorityScan,
    ] {
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
        let p95 = percentile(&mut timings, 95);
        println!(
            "{policy:?},{},{},{},{},{},{},{},{},{p50},{p95}",
            payments.len(),
            result.settled_count,
            result.unsettled_count,
            result.excluded_count,
            result.settled_minor,
            result.late_settled_count,
            result.missed_deadline_count,
            result.modeled_fee_units
        );
    }
}

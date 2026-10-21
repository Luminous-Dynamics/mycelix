use finance_queue_lsm_sim::{
    fifo_order, simulate_order, simulated_annealing_order, unlock_potential_order,
    AnnealingConfig, ParticipantLiquidity, PaymentState, QueueContext, QueuedPayment,
};
use std::time::Instant;

fn make_case(count: usize) -> (QueueContext, Vec<ParticipantLiquidity>, Vec<QueuedPayment>) {
    let context = QueueContext {
        network_id: "synthetic-south-africa-profile".into(),
        asset_id: "zar-minor".into(),
    };
    let participant_count = (count / 4).max(8);
    let mut balances = Vec::with_capacity(participant_count);
    for i in 0..participant_count {
        balances.push(ParticipantLiquidity {
            participant_id: format!("P{i:04}"),
            opening_liquidity_minor: ((i * 13 + 7) % 29) as u64,
        });
    }

    let mut payments = Vec::with_capacity(count);
    for i in 0..count {
        let payer_index = (i * 17 + 3) % participant_count;
        let mut payee_index = (i * 31 + 11) % participant_count;
        if payee_index == payer_index {
            payee_index = (payee_index + 1) % participant_count;
        }
        payments.push(QueuedPayment {
            payment_id: format!("PAY-{i:05}"),
            network_id: context.network_id.clone(),
            asset_id: context.asset_id.clone(),
            payer: format!("P{payer_index:04}"),
            payee: format!("P{payee_index:04}"),
            amount_minor: ((i * 19 + 5) % 31 + 1) as u64,
            queue_sequence: i as u64,
            priority_weight: ((i * 7 + 1) % 5) as u32,
            deadline_step: Some(((i * 23) % (count / 2 + 1) + 1) as u64),
            state: PaymentState::Accepted,
        });
    }
    (context, balances, payments)
}

fn percentile(values: &[u128], p: usize) -> u128 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = (p * sorted.len() + 99) / 100;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn main() {
    println!("batch_size,algorithm,iterations,seed,settled_count,settled_value_minor,opening_liquidity_minor,pending_count,missed_deadlines,weighted_delay_penalty,runtime_p50_us,runtime_p95_us");

    for count in [70_usize, 140, 700] {
        let (context, balances, payments) = make_case(count);
        let fifo = fifo_order(&context, &payments).expect("FIFO order");
        let unlock = unlock_potential_order(&context, &payments).expect("unlock order");

        for (algorithm, order) in [("fifo", fifo.clone()), ("unlock-potential", unlock.clone())] {
            let mut timings = Vec::with_capacity(25);
            let mut final_report = None;
            for _ in 0..25 {
                let start = Instant::now();
                final_report = Some(
                    simulate_order(&context, &balances, &payments, &order)
                        .expect("queue simulation"),
                );
                timings.push(start.elapsed().as_micros());
            }
            let report = final_report.expect("non-empty repetitions");
            println!(
                "{},{},0,0,{},{},{},{},{},{},{},{}",
                count,
                algorithm,
                report.settled_count,
                report.total_settled_minor,
                report.opening_liquidity_total_minor,
                report.pending_payment_ids.len(),
                report.missed_deadline_count,
                report.priority_weighted_delay_penalty,
                percentile(&timings, 50),
                percentile(&timings, 95),
            );
        }

        // Fixed seeds and a public iteration budget make the heuristic reproducible.
        // Keep the budget lower for the larger batch; this is a compute-budget comparison,
        // not a claim of equivalence with any central-bank production algorithm.
        let iterations = match count {
            70 => 300,
            140 => 120,
            _ => 30,
        };
        for seed in [0x51A, 0x51B, 0x51C, 0x51D, 0x51E] {
            let config = AnnealingConfig {
                iterations,
                seed,
                initial_temperature: 0.35,
                cooling_factor: 0.985,
            };
            let mut timings = Vec::with_capacity(5);
            let mut final_result = None;
            for _ in 0..5 {
                let start = Instant::now();
                final_result = Some(
                    simulated_annealing_order(&context, &balances, &payments, &fifo, &config)
                        .expect("seeded annealing"),
                );
                timings.push(start.elapsed().as_micros());
            }
            let result = final_result.expect("non-empty repetitions");
            println!(
                "{},simulated-annealing,{},{},{},{},{},{},{},{},{},{}",
                count,
                result.iterations_evaluated,
                seed,
                result.best_simulation.settled_count,
                result.best_simulation.total_settled_minor,
                result.best_simulation.opening_liquidity_total_minor,
                result.best_simulation.pending_payment_ids.len(),
                result.best_simulation.missed_deadline_count,
                result.best_simulation.priority_weighted_delay_penalty,
                percentile(&timings, 50),
                percentile(&timings, 95),
            );
        }
    }
}

use finance_queue_lsm_sim::{
    fifo_order, minimum_opening_liquidity_for_sequence,
    simulated_annealing_minimum_liquidity_order, unlock_potential_order, AnnealingConfig,
    MinimumLiquidityReport, PaymentState, QueueContext, QueuedPayment,
};
use std::time::Instant;

fn make_case(count: usize) -> (QueueContext, Vec<QueuedPayment>) {
    let context = QueueContext {
        network_id: "synthetic-south-africa-profile".into(),
        asset_id: "zar-minor".into(),
    };
    let participant_count = (count / 4).max(8);
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

    (context, payments)
}

fn percentile(values: &[u128], p: usize) -> u128 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = (p * sorted.len() + 99) / 100;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn measure_order(
    context: &QueueContext,
    payments: &[QueuedPayment],
    order: &[String],
    repetitions: usize,
) -> (MinimumLiquidityReport, Vec<u128>) {
    let mut samples = Vec::with_capacity(repetitions);
    let mut final_report = None;
    for _ in 0..repetitions {
        let start = Instant::now();
        final_report = Some(
            minimum_opening_liquidity_for_sequence(context, payments, order)
                .expect("strict-sequence liquidity requirement"),
        );
        samples.push(start.elapsed().as_micros());
    }
    (final_report.expect("positive repetition count"), samples)
}

fn print_row(
    count: usize,
    algorithm: &str,
    iterations: usize,
    seed: u64,
    report: &MinimumLiquidityReport,
    timings: &[u128],
) {
    println!(
        "{},{},{},{},{},{},{},{},{}",
        count,
        algorithm,
        iterations,
        seed,
        report.total_required_opening_liquidity_minor,
        report.missed_deadline_count,
        report.priority_weighted_delay_penalty,
        percentile(timings, 50),
        percentile(timings, 95),
    );
}

fn main() {
    println!("batch_size,algorithm,iterations,seed,required_opening_liquidity_minor,missed_deadlines,priority_weighted_delay,runtime_p50_us,runtime_p95_us");

    for count in [70_usize, 140, 700] {
        let (context, payments) = make_case(count);
        let fifo = fifo_order(&context, &payments).expect("FIFO order");
        let unlock = unlock_potential_order(&context, &payments).expect("unlock order");

        for (algorithm, order) in [("fifo", fifo.clone()), ("unlock-potential", unlock)] {
            let (report, timings) = measure_order(&context, &payments, &order, 25);
            print_row(count, algorithm, 0, 0, &report, &timings);
        }

        // Insertion-neighbour simulated annealing searches strict execution order.
        // It optimizes a different objective from fixed-liquidity queue simulation.
        let iterations = match count {
            70 => 100,
            140 => 30,
            _ => 5,
        };
        for seed in [0x51A_u64, 0x51B, 0x51C, 0x51D, 0x51E] {
            let config = AnnealingConfig {
                iterations,
                seed,
                initial_temperature: 0.35,
                cooling_factor: 0.985,
            };
            let mut timings = Vec::with_capacity(3);
            let mut final_result = None;
            for _ in 0..3 {
                let start = Instant::now();
                final_result = Some(
                    simulated_annealing_minimum_liquidity_order(
                        &context, &payments, &fifo, &config,
                    )
                    .expect("minimum-liquidity annealing"),
                );
                timings.push(start.elapsed().as_micros());
            }
            let result = final_result.expect("positive repetition count");
            print_row(
                count,
                "minimum-liquidity-annealing",
                result.iterations_evaluated,
                seed,
                &result.best_report,
                &timings,
            );
        }
    }}

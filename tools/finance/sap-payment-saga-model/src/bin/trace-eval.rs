use std::time::Instant;

use sha2::{Digest, Sha256};
use sap_payment_saga_model::trace_eval::evaluate_generated_trace_v1;

fn percentile(sorted: &[u128], numerator: usize, denominator: usize) -> u128 {
    if sorted.is_empty() {
        return 0;
    }
    let index = ((sorted.len() - 1) * numerator).div_ceil(denominator);
    sorted[index]
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let seed = args.first().map(|value| value.parse::<u64>().expect("seed must be u64"))
        .unwrap_or(0x5eed_2026);
    let count = args.get(1).map(|value| value.parse::<u64>().expect("count must be u64"))
        .unwrap_or(10_000);
    let start_index = args.get(2).map(|value| value.parse::<u64>().expect("start index must be u64"))
        .unwrap_or(0);

    if count == 0 || start_index.checked_add(count).is_none() {
        eprintln!("count must be positive and start_index + count must not overflow");
        std::process::exit(2);
    }

    let wall_start = Instant::now();
    let mut latency_ns = Vec::with_capacity(count.min(2_000_000) as usize);
    let mut scenarios = [0_u64; 8];
    let mut commands = 0_u64;
    let mut digest_chain = Sha256::new();

    for index in start_index..start_index + count {
        let item_start = Instant::now();
        let evaluation = match evaluate_generated_trace_v1(seed, index) {
            Ok(value) => value,
            Err(error) => {
                eprintln!("TRACE_EVAL_ERROR seed={seed} index={index} error={error:?}");
                std::process::exit(1);
            }
        };
        latency_ns.push(item_start.elapsed().as_nanos());
        commands = commands.checked_add(evaluation.command_count() as u64)
            .expect("command count overflow");
        scenarios[usize::from(evaluation.scenario - 1)] += 1;
        digest_chain.update(evaluation.trace_digest.as_bytes());

        if !evaluation.passed() {
            eprintln!(
                "TRACE_EVAL_FAILURE seed={} index={} scenario={} digest={} failures={:?}",
                evaluation.seed, evaluation.trace_index, evaluation.scenario,
                evaluation.trace_digest, evaluation.failures
            );
            std::process::exit(1);
        }
    }

    latency_ns.sort_unstable();
    let elapsed = wall_start.elapsed();
    let ns = elapsed.as_nanos().max(1);
    let traces_per_sec = (count as u128 * 1_000_000_000) / ns;
    let commands_per_sec = (commands as u128 * 1_000_000_000) / ns;
    let corpus_digest = digest_chain.finalize();
    let corpus_sha256: String = corpus_digest.iter().map(|byte| format!("{byte:02x}")).collect();

    println!(
        "TRACE_EVAL_SUMMARY_V1 seed={seed} start_index={start_index} traces={count} commands={commands} scenarios={:?} elapsed_ms={} traces_per_sec={} commands_per_sec={} p50_us={} p95_us={} p99_us={} corpus_sha256={}",
        scenarios, elapsed.as_millis(), traces_per_sec, commands_per_sec,
        percentile(&latency_ns, 50, 100) / 1_000,
        percentile(&latency_ns, 95, 100) / 1_000,
        percentile(&latency_ns, 99, 100) / 1_000,
        corpus_sha256
    );
}

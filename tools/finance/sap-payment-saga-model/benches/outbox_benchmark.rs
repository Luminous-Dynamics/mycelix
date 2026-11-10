use sap_payment_saga_model::outbox::{
    BackoffPolicy, NewOutboxEffect, OutboxBook,
};
use std::hint::black_box;
use std::time::Instant;

const SAMPLE_COUNT: usize = 30;
const QUEUE_SIZES: [usize; 3] = [1_000, 10_000, 100_000];

fn policy() -> BackoffPolicy {
    BackoffPolicy {
        base_delay_ms: 1,
        max_delay_ms: 1_000,
        jitter_window_ms: 10,
        max_dispatch_attempts: 10,
        max_reconciliation_attempts: 10,
        max_reconciliation_age_ms: 60_000,
    }
}

fn build_book(size: usize) -> OutboxBook {
    let mut book = OutboxBook::default();
    let payload_digest = format!("sha256:{}", "a".repeat(64));
    for index in 0..size {
        let operation_id = format!("outbox-bench-operation-{index:08}");
        let effect_id = format!("outbox-bench-treasury-fee-{index:08}");
        book.enqueue(NewOutboxEffect {
            operation_id,
            effect_id,
            destination_id: "treasury:benchmark-destination".into(),
            payload_digest: payload_digest.clone(),
            created_at_ms: 1_000,
        })
        .expect("benchmark effect must enqueue");
    }
    book
}

fn percentile(sorted: &[u128], percent: usize) -> u128 {
    let rank = (percent * sorted.len() + 99) / 100;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn report(operation: &str, size: usize, samples: &[u128]) {
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    println!(
        "OUTBOX_RUST_SUMMARY operation={operation} items={size} samples={} p50_ns/op={} p95_ns/op={} p99_ns/op={} min_ns/op={} max_ns/op={}",
        sorted.len(),
        percentile(&sorted, 50),
        percentile(&sorted, 95),
        percentile(&sorted, 99),
        sorted[0],
        sorted[sorted.len() - 1],
    );
}

fn main() {
    for size in QUEUE_SIZES {
        let mut book = build_book(size);
        let mut claim_samples = Vec::with_capacity(SAMPLE_COUNT);
        for sample in 0..SAMPLE_COUNT {
            let now_ms = 1_000 + (sample as u64 * 1_001);
            let started = Instant::now();
            let leases = book
                .claim_due("outbox-benchmark-worker", now_ms, 1_000, 64, policy())
                .expect("bounded claim");
            let elapsed = started.elapsed().as_nanos();
            assert_eq!(leases.len(), 64);
            claim_samples.push(black_box(elapsed));
            println!(
                "OUTBOX_RUST_SAMPLE operation=claim_due items={size} sample={} ns_per_op={elapsed}",
                sample + 1
            );
        }
        report("claim_due", size, &claim_samples);

        let mut metric_samples = Vec::with_capacity(SAMPLE_COUNT);
        for sample in 0..SAMPLE_COUNT {
            let now_ms = 40_000 + sample as u64;
            let started = Instant::now();
            let metrics = book.metrics(now_ms);
            let elapsed = started.elapsed().as_nanos();
            assert_eq!(metrics.ready + metrics.leased, size);
            metric_samples.push(black_box(elapsed));
            println!(
                "OUTBOX_RUST_SAMPLE operation=metrics items={size} sample={} ns_per_op={elapsed}",
                sample + 1
            );
        }
        report("metrics", size, &metric_samples);
    }
}

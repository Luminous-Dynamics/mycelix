use std::hint::black_box;
use std::time::Instant;

use mycelix_region_policy::effect_journal::{
    EffectAttempt, EffectIdentity, SinkDisposition, SinkOutcomeEvidence, SinkOutcomeVerifier,
    SinkProofError, VerifiedSinkOutcome,
};

struct FixtureSinkVerifier;
impl SinkOutcomeVerifier for FixtureSinkVerifier {
    fn verify(&self, evidence: &SinkOutcomeEvidence) -> Result<(), SinkProofError> {
        // This intentionally measures state-machine overhead only, not cryptography.
        if evidence.proof == vec![0xA5; 64] {
            Ok(())
        } else {
            Err(SinkProofError::InvalidSignature)
        }
    }
}

fn percentile(sorted: &[f64], quantile: f64) -> f64 {
    let index = ((sorted.len() as f64 * quantile).ceil() as usize)
        .saturating_sub(1)
        .min(sorted.len() - 1);
    sorted[index]
}

fn timer_baseline(samples: usize) -> Vec<f64> {
    let mut durations = Vec::with_capacity(samples);
    for _ in 0..samples {
        let start = Instant::now();
        black_box(0usize);
        durations.push(start.elapsed().as_secs_f64() * 1_000_000.0);
    }
    durations.sort_by(f64::total_cmp);
    durations
}

fn main() {
    const OPERATIONS: usize = 100_000;
    const TIMER_BASELINE_SAMPLES: usize = 10_000;
    let verifier = FixtureSinkVerifier;
    let target_digest = "a".repeat(64);
    let policy_digest = "b".repeat(64);
    let outcome_digest = "c".repeat(64);
    let mut operation_latencies_us = Vec::with_capacity(OPERATIONS);
    let campaign_start = Instant::now();

    for operation in 0..OPERATIONS {
        let operation_start = Instant::now();
        let key = format!("idem-{operation}");
        let identity = EffectIdentity::new(
            "benchmark-community",
            &key,
            &target_digest,
            &policy_digest,
        )
        .expect("benchmark identity");
        let mut attempt = EffectAttempt::prepare(identity.clone());
        let sink_operation_id = format!("sink-{operation}");
        attempt.reserve(&sink_operation_id).expect("prepare -> reserve");
        attempt.mark_dispatch_started().expect("reserve -> dispatch");

        let evidence = SinkOutcomeEvidence {
            authority_domain: "benchmark-community".to_owned(),
            idempotency_key: key,
            target_digest: target_digest.clone(),
            policy_binding_digest: policy_digest.clone(),
            operation_id: sink_operation_id,
            disposition: SinkDisposition::Applied,
            outcome_digest: outcome_digest.clone(),
            proof: vec![0xA5; 64],
        };
        let verified = VerifiedSinkOutcome::verify(evidence, &identity, &verifier)
            .expect("benchmark fixture proof");
        attempt.resolve(verified).expect("dispatched -> committed");
        black_box(attempt);
        operation_latencies_us.push(operation_start.elapsed().as_secs_f64() * 1_000_000.0);
    }
    let campaign_seconds = campaign_start.elapsed().as_secs_f64();

    operation_latencies_us.sort_by(f64::total_cmp);
    let timer_baseline_us = timer_baseline(TIMER_BASELINE_SAMPLES);
    println!("benchmark=effect_journal_state_machine");
    println!("operations={OPERATIONS}");
    println!("sample_model=single-operation wall-clock; measurement overhead included");
    println!(
        "latency_us_per_op p50={:.3} p95={:.3} p99={:.3}",
        percentile(&operation_latencies_us, 0.50),
        percentile(&operation_latencies_us, 0.95),
        percentile(&operation_latencies_us, 0.99),
    );
    println!(
        "timer_baseline_us p50={:.3} p95={:.3} p99={:.3} samples={TIMER_BASELINE_SAMPLES}",
        percentile(&timer_baseline_us, 0.50),
        percentile(&timer_baseline_us, 0.95),
        percentile(&timer_baseline_us, 0.99),
    );
    println!("instrumented_ops_per_sec={:.0}", OPERATIONS as f64 / campaign_seconds);
    println!("notes=in-memory prepare/reserve/dispatch/fixture-verify/resolve only; no database, network, cryptography, or durable CAS; baseline is reported but not subtracted; instrumented throughput includes timer and sample-capture overhead");
}
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

fn main() {
    const BATCHES: usize = 1_000;
    const OPERATIONS_PER_BATCH: usize = 100;
    let verifier = FixtureSinkVerifier;
    let target_digest = "a".repeat(64);
    let policy_digest = "b".repeat(64);
    let outcome_digest = "c".repeat(64);
    let mut micros_per_operation = Vec::with_capacity(BATCHES);

    for batch in 0..BATCHES {
        let start = Instant::now();
        for operation in 0..OPERATIONS_PER_BATCH {
            let key = format!("idem-{batch}-{operation}");
            let identity = EffectIdentity::new("benchmark-community", &key, &target_digest, &policy_digest)
                .expect("benchmark identity");
            let mut attempt = EffectAttempt::prepare(identity.clone());
            let sink_operation_id = format!("sink-{batch}-{operation}");
            attempt.mark_submitted(&sink_operation_id).expect("prepare -> submit");

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
            attempt.resolve(verified).expect("submitted -> committed");
            black_box(attempt);
        }
        let elapsed_us = start.elapsed().as_secs_f64() * 1_000_000.0;
        micros_per_operation.push(elapsed_us / OPERATIONS_PER_BATCH as f64);
    }

    micros_per_operation.sort_by(f64::total_cmp);
    println!("benchmark=effect_journal_state_machine");
    println!("operations={}", BATCHES * OPERATIONS_PER_BATCH);
    println!(
        "latency_us_per_op p50={:.3} p95={:.3} p99={:.3}",
        percentile(&micros_per_operation, 0.50),
        percentile(&micros_per_operation, 0.95),
        percentile(&micros_per_operation, 0.99),
    );
    println!("notes=in-memory lifecycle transitions only; no database, network, cryptography, or durable CAS");
}

use sap_payment_saga_model::checkpoint::{
    canonical_intent_digest, CheckpointOwnerDomain, JournalCheckpointSubject,
};
use sap_payment_saga_model::PaymentIntent;
use std::hint::black_box;
use std::time::{Duration, Instant};

const SAMPLE_COUNT: usize = 30;
const SAMPLE_WINDOW: Duration = Duration::from_millis(100);

fn action_hash(index: u32) -> Vec<u8> {
    let mut hash = vec![0x84, 0x29, 0x24];
    hash.extend(std::iter::repeat((index & 0xff) as u8).take(32));
    hash.extend(index.to_be_bytes());
    hash
}

fn benchmark_intent() -> PaymentIntent {
    PaymentIntent::new(
        "did:sender",
        "did:recipient",
        "client-key-001",
        format!("sha256:{}", "a".repeat(64)),
        100,
        3,
        "SAP",
    )
}

fn build_checkpoint_chain(
    owner_domain: CheckpointOwnerDomain,
    checkpoint_count: usize,
) -> Vec<JournalCheckpointSubject> {
    let intent = benchmark_intent();
    let intent_digest = canonical_intent_digest(&intent).expect("valid benchmark intent");
    let operation_id = "sap-payment:10:did:sender:14:client-key-001".to_owned();
    let (owner_did, event_log_id) = match owner_domain {
        CheckpointOwnerDomain::Payments => (
            "did:mycelix:payments:alice",
            "sap-journal:payments:benchmark",
        ),
        CheckpointOwnerDomain::Treasury => (
            "did:mycelix:treasury:community",
            "sap-journal:treasury:benchmark",
        ),
    };

    let mut previous_checkpoint_digest = None;
    let mut previous_action_hash = action_hash(41);
    let mut subjects = Vec::with_capacity(checkpoint_count);
    for index in 0..checkpoint_count {
        let action_sequence = u32::try_from(index + 42).expect("benchmark sequence fits u32");
        let current_action_hash = action_hash(action_sequence);
        let subject = JournalCheckpointSubject {
            owner_domain,
            operation_id: operation_id.clone(),
            intent_digest: intent_digest.clone(),
            owner_did: owner_did.to_owned(),
            event_log_id: event_log_id.to_owned(),
            event_count: u64::try_from(index + 1).expect("benchmark event count fits u64"),
            tail_digest: format!("sha256:{:064x}", index + 1),
            checkpoint_sequence: u64::try_from(index + 1).expect("benchmark sequence fits u64"),
            previous_checkpoint_digest: previous_checkpoint_digest.clone(),
            source_action_hash: current_action_hash.clone(),
            source_action_sequence: action_sequence,
            source_previous_action_hash: Some(previous_action_hash),
        };
        previous_checkpoint_digest = Some(subject.canonical_digest().expect("valid checkpoint"));
        previous_action_hash = current_action_hash;
        subjects.push(subject);
    }
    subjects
}

fn main() {
    println!(
        "RUST_CHECKPOINT_METADATA sample_count={SAMPLE_COUNT} sample_window_ms={}",
        SAMPLE_WINDOW.as_millis()
    );
    for (owner_name, owner_domain) in [
        ("payments", CheckpointOwnerDomain::Payments),
        ("treasury", CheckpointOwnerDomain::Treasury),
    ] {
        for checkpoint_count in [100_usize, 1_000, 10_000] {
            // Build the synthetic owner-local chain outside the timed region.
            let subjects = build_checkpoint_chain(owner_domain, checkpoint_count);
            black_box(
                subjects
                    .last()
                    .expect("non-empty checkpoint set")
                    .canonical_digest()
                    .expect("warmup digest"),
            );
            for sample in 1..=SAMPLE_COUNT {
                let start = Instant::now();
                let mut iterations = 0_u64;
                while start.elapsed() < SAMPLE_WINDOW {
                    for subject in &subjects {
                        black_box(subject.canonical_digest().expect("valid checkpoint digest"));
                    }
                    iterations += 1;
                }
                let elapsed = start.elapsed();
                let denominator = iterations * u64::try_from(subjects.len()).expect("size fits u64");
                let ns_per_checkpoint = elapsed.as_nanos() as f64 / denominator as f64;
                println!(
                    "RUST_CHECKPOINT_SAMPLE owner={owner_name} checkpoints={checkpoint_count} sample={sample} iterations={iterations} elapsed_ns={} ns_per_checkpoint={ns_per_checkpoint:.3}",
                    elapsed.as_nanos()
                );
            }
        }
    }
}

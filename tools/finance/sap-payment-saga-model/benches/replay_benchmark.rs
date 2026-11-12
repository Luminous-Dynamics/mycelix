use sap_payment_saga_model::{
    EffectEvidence, EffectEvidenceKind, PaymentIntent, PaymentLeg, PaymentRecordStatus,
    replay::{JournalEvent, JournalEventFrame, JournalEventLog, ReplayAnchor, replay_journal},
};
use std::hint::black_box;
use std::time::{Duration, Instant};

const SAMPLE_COUNT: usize = 30;
const SAMPLE_WINDOW: Duration = Duration::from_millis(200);

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

fn local_evidence(operation_id: &str) -> EffectEvidence {
    let effect_id = format!("{operation_id}:local-payment-transaction");
    let mut evidence = EffectEvidence {
        operation_id: operation_id.to_owned(),
        effect_id,
        leg: PaymentLeg::LocalPaymentTransaction,
        sender_did: "did:sender".into(),
        recipient_did: "did:recipient".into(),
        principal_minor: 100,
        fee_minor: 3,
        currency: "SAP".into(),
        evidence_digest: String::new(),
        kind: EffectEvidenceKind::LocalPaymentTransaction {
            sender_account_id: "did:sender".into(),
            recipient_account_id: "did:recipient".into(),
            sender_debit_minor: 103,
            recipient_credit_minor: 100,
            sender_predecessor_action: "action:sender-before".into(),
            sender_debit_action: "action:sender-debit".into(),
            recipient_predecessor_action: "action:recipient-before".into(),
            recipient_credit_action: "action:recipient-credit".into(),
            payment_action_hash: "action:payment".into(),
            payment_status: PaymentRecordStatus::Completed,
            receipt_action_hash: "action:receipt".into(),
            receipt_signer_did: "did:sender".into(),
            signing_profile: "mycelix.finance.payment-receipt.v1".into(),
            signature_digest: format!("sha256:{}", "b".repeat(64)),
            treasury_fee_outbox_id: Some(format!("{operation_id}:treasury-fee")),
        },
    };
    evidence.evidence_digest = evidence.canonical_digest().expect("local evidence digest");
    evidence
}

fn treasury_evidence(operation_id: &str) -> EffectEvidence {
    let effect_id = format!("{operation_id}:treasury-fee");
    let mut evidence = EffectEvidence {
        operation_id: operation_id.to_owned(),
        effect_id: effect_id.clone(),
        leg: PaymentLeg::TreasuryFee,
        sender_did: "did:sender".into(),
        recipient_did: "did:recipient".into(),
        principal_minor: 100,
        fee_minor: 3,
        currency: "SAP".into(),
        evidence_digest: String::new(),
        kind: EffectEvidenceKind::TreasuryFee {
            pool_id: "treasury:global-fee-pool".into(),
            source_member_did: "did:sender".into(),
            amount_minor: 3,
            payment_action_hash: "action:payment".into(),
            idempotency_key: effect_id,
            predecessor_action: "action:treasury-before".into(),
            treasury_action: "action:treasury-credit".into(),
        },
    };
    evidence.evidence_digest = evidence
        .canonical_digest()
        .expect("Treasury evidence digest");
    evidence
}

fn build_history(event_count: usize) -> (Vec<JournalEventFrame>, ReplayAnchor) {
    assert!(
        event_count >= 6 && (event_count - 6).is_multiple_of(2),
        "event count must be 6 + 2*n"
    );

    let mut log = JournalEventLog::new(benchmark_intent()).expect("create event log");
    let operation_id = match &log.frames()[0].event {
        JournalEvent::Created { operation_id, .. } => operation_id.clone(),
        _ => panic!("first event must create the journal"),
    };
    let local_id = format!("{operation_id}:local-payment-transaction");
    let treasury_id = format!("{operation_id}:treasury-fee");

    log.record_authorized().expect("record authorization");
    log.record_effect_prepared(PaymentLeg::LocalPaymentTransaction, local_id)
        .expect("record local prepare");
    log.record_effect_committed(local_evidence(&operation_id))
        .expect("record local commit");
    log.record_effect_prepared(PaymentLeg::TreasuryFee, treasury_id.clone())
        .expect("record Treasury prepare");

    let retry_cycles = (event_count - 6) / 2;
    for _ in 0..retry_cycles {
        log.record_effect_indeterminate(PaymentLeg::TreasuryFee, treasury_id.clone())
            .expect("record indeterminate Treasury outcome");
        log.record_effect_prepared(PaymentLeg::TreasuryFee, treasury_id.clone())
            .expect("record same-effect Treasury retry");
    }

    log.record_effect_committed(treasury_evidence(&operation_id))
        .expect("record Treasury commit");

    let frames = log.frames().to_vec();
    assert_eq!(frames.len(), event_count, "fixture frame count");
    let anchor = log.anchor();
    let replayed = replay_journal(&frames, &anchor).expect("benchmark history must replay");
    assert!(
        replayed.is_completed(),
        "benchmark history must be completed"
    );
    (frames, anchor)
}

fn main() {
    println!(
        "RUST_REPLAY_METADATA sample_count={SAMPLE_COUNT} sample_window_ms={}",
        SAMPLE_WINDOW.as_millis()
    );
    for event_count in [100_usize, 1_000, 10_000] {
        // Fixture construction, digest generation and validation are outside the timed region.
        let (frames, anchor) = build_history(event_count);
        black_box(replay_journal(black_box(&frames), black_box(&anchor)).expect("warmup replay"));

        for sample in 1..=SAMPLE_COUNT {
            let start = Instant::now();
            let mut iterations = 0_u64;
            while start.elapsed() < SAMPLE_WINDOW {
                black_box(
                    replay_journal(black_box(&frames), black_box(&anchor))
                        .expect("replay must remain valid"),
                );
                iterations += 1;
            }
            let elapsed = start.elapsed();
            let ns_per_op = elapsed.as_nanos() as f64 / iterations as f64;
            println!(
                "RUST_REPLAY_SAMPLE events={event_count} sample={sample} iterations={iterations} elapsed_ns={} ns_per_op={ns_per_op:.3}",
                elapsed.as_nanos()
            );
        }
    }
}

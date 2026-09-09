use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, IdempotencyKey, IntegrationCommandId, OutboundStage,
    SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, RuntimeError, SqliteIntegrationStore,
};

const MAX_ATTEMPTS_V01: u32 = 1_024;
const REOPEN_AFTER_ATTEMPT: u32 = MAX_ATTEMPTS_V01 / 2;

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: IntegrationCommandId::new("attempt-budget-command")
            .expect("fixture command must be valid"),
        connector_instance: ConnectorInstanceId::new("attempt-budget-connector")
            .expect("fixture connector must be valid"),
        command_commitment: ContentCommitment::sha256(b"attempt-budget-command"),
        authority_commitment: ContentCommitment::sha256(b"attempt-budget-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("attempt-budget-idempotency")
                .expect("fixture key must be valid"),
        ),
        command_bytes: b"sealed-attempt-budget-command".to_vec(),
        created_at_ms: 100,
    }
}

fn consume_pre_dispatch_attempts(
    store: &mut SqliteIntegrationStore,
    range: std::ops::Range<u32>,
    previous_attempt_id: &mut Option<mycelix_integration_core::ExecutionAttemptId>,
) {
    for attempt_index in range {
        let now_ms = 1_000 + i64::from(attempt_index) * 2;
        let claim = store
            .claim_outbox("worker-budget", now_ms, 1, 1)
            .expect("attempts below the v0.1 bound must be admitted")
            .remove(0);
        assert_eq!(claim.attempt_count, attempt_index + 1);
        if let Some(previous) = previous_attempt_id.as_ref() {
            assert_ne!(previous, &claim.attempt_id, "attempt identities must never be reused");
        }
        *previous_attempt_id = Some(claim.attempt_id.clone());

        let recovered = store
            .recover_expired_claims(now_ms + 1)
            .expect("pre-dispatch expiry must safely return to the queue");
        assert_eq!(recovered.pre_dispatch_requeued, 1);
    }
}

#[test]
fn attempt_generation_is_explicitly_bounded_persistent_and_never_reused() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("attempt-budget.sqlite");
    let entry_id;
    let mut previous_attempt_id = None;

    {
        let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
        entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected enqueue disposition: {other:?}"),
        };
        consume_pre_dispatch_attempts(
            &mut store,
            0..REOPEN_AFTER_ATTEMPT,
            &mut previous_attempt_id,
        );
        let snapshot = store.outbox_snapshot(entry_id).expect("snapshot before reopen");
        assert_eq!(snapshot.stage, OutboundStage::OutboxCommitted);
        assert_eq!(snapshot.attempt_count, REOPEN_AFTER_ATTEMPT);
    }

    let mut store = SqliteIntegrationStore::open(&path).expect("store must reopen");
    let reopened = store.outbox_snapshot(entry_id).expect("snapshot after reopen");
    assert_eq!(reopened.stage, OutboundStage::OutboxCommitted);
    assert_eq!(reopened.attempt_count, REOPEN_AFTER_ATTEMPT);

    consume_pre_dispatch_attempts(
        &mut store,
        REOPEN_AFTER_ATTEMPT..MAX_ATTEMPTS_V01,
        &mut previous_attempt_id,
    );

    let before = store.outbox_snapshot(entry_id).expect("snapshot before exhaustion");
    assert_eq!(before.stage, OutboundStage::OutboxCommitted);
    assert_eq!(before.attempt_count, MAX_ATTEMPTS_V01);

    assert!(matches!(
        store.claim_outbox(
            "worker-budget",
            1_000 + i64::from(MAX_ATTEMPTS_V01) * 2,
            1,
            1,
        ),
        Err(RuntimeError::AttemptBudgetExceeded { entry_id: id, limit })
            if id == entry_id && limit == MAX_ATTEMPTS_V01
    ));

    let after = store.outbox_snapshot(entry_id).expect("snapshot after exhaustion");
    assert_eq!(after.stage, OutboundStage::OutboxCommitted);
    assert_eq!(after.attempt_count, MAX_ATTEMPTS_V01);
    assert_eq!(after.current_attempt_id, None);
}

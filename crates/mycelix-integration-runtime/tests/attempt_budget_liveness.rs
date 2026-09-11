use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, IdempotencyKey, IntegrationCommandId, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, SqliteIntegrationStore,
};

const MAX_ATTEMPTS_V01: u32 = 1_024;

fn intent(command: &str) -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: IntegrationCommandId::new(command).expect("fixture command must be valid"),
        connector_instance: ConnectorInstanceId::new("attempt-liveness-connector")
            .expect("fixture connector must be valid"),
        command_commitment: ContentCommitment::sha256(command.as_bytes()),
        authority_commitment: ContentCommitment::sha256(b"attempt-liveness-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new(format!("idem:{command}"))
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: format!("sealed:{command}").into_bytes(),
        created_at_ms: 100,
    }
}

#[test]
fn exhausted_oldest_entry_does_not_head_of_line_block_later_eligible_work() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let exhausted_entry = match store
        .enqueue_outbound(&intent("exhausted-command"))
        .expect("enqueue exhausted fixture")
    {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };

    for attempt_index in 0..MAX_ATTEMPTS_V01 {
        let now_ms = 1_000 + i64::from(attempt_index) * 2;
        let claim = store
            .claim_outbox("worker-exhaust", now_ms, 1, 1)
            .expect("attempts below the bound must be admitted")
            .remove(0);
        assert_eq!(claim.entry_id, exhausted_entry);
        store
            .recover_expired_claims(now_ms + 1)
            .expect("pre-dispatch attempt must requeue");
    }

    let eligible_entry = match store
        .enqueue_outbound(&intent("eligible-command"))
        .expect("enqueue eligible fixture")
    {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };

    let claimed = store
        .claim_outbox(
            "worker-eligible",
            1_000 + i64::from(MAX_ATTEMPTS_V01) * 2,
            10,
            1,
        )
        .expect("an exhausted older row must not block unrelated eligible work");

    assert_eq!(claimed.len(), 1);
    assert_eq!(claimed[0].entry_id, eligible_entry);
    assert_eq!(claimed[0].attempt_count, 1);
}

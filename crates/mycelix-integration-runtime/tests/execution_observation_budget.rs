use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalOpaqueId,
    ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId, OutboundStage,
    SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, ExecutionRecordDisposition, RuntimeError,
    SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("budget-test-connector").expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("budget-test-command").expect("fixture command must be valid")
}

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command_id(),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(b"budget-test-command"),
        authority_commitment: ContentCommitment::sha256(b"budget-test-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("budget-test-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-budget-test-command".to_vec(),
        created_at_ms: 100,
    }
}

fn confirmed() -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: ExternalOperationRef {
            command_id: command_id(),
            connector_instance: connector(),
            provider_operation: None,
        },
        provider_receipt: Some(
            ExternalOpaqueId::new("late-budget-test-receipt")
                .expect("fixture receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(b"late-budget-test-receipt"),
        confirmed_at_ms: 150,
    })
}

#[test]
fn crash_generated_ambiguity_consumes_the_same_observation_budget() {
    const LATE_OBSERVATIONS_TO_FILL_BOUND: i64 = 4_095;

    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };

    let claim = store
        .claim_outbox("worker-budget", 110, 10, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-budget", 111)
        .expect("dispatch boundary must succeed");
    store
        .recover_expired_claims(121)
        .expect("post-dispatch recovery must become ambiguous");

    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Ambiguous
    );

    for offset in 0..LATE_OBSERVATIONS_TO_FILL_BOUND {
        let disposition = store
            .record_execution(
                entry_id,
                &claim.attempt_id,
                "worker-budget",
                &confirmed(),
                122 + offset,
            )
            .expect("late observation below the bound must be preserved");
        assert_eq!(
            disposition,
            ExecutionRecordDisposition::RecordedForStaleAttempt
        );
    }

    assert!(matches!(
        store.record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-budget",
            &confirmed(),
            122 + LATE_OBSERVATIONS_TO_FILL_BOUND,
        ),
        Err(RuntimeError::ExecutionObservationHistoryLimit { entry_id: id }) if id == entry_id
    ));

    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Ambiguous
    );
}

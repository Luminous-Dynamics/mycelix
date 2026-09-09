use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalOpaqueId,
    ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId, OutboundStage,
    ReconciliationDisposition, ReconciliationHint, ReconciliationResult, ReconciliationStrategy,
    SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, RuntimeError, SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("provider-operation-binding")
        .expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("provider-operation-command")
        .expect("fixture command must be valid")
}

fn operation(provider_operation: Option<&str>) -> ExternalOperationRef {
    ExternalOperationRef {
        command_id: command_id(),
        connector_instance: connector(),
        provider_operation: provider_operation.map(|value| {
            ExternalOpaqueId::new(value).expect("fixture provider operation must be valid")
        }),
    }
}

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command_id(),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(b"provider-operation-command"),
        authority_commitment: ContentCommitment::sha256(b"provider-operation-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("provider-operation-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-provider-operation-command".to_vec(),
        created_at_ms: 100,
    }
}

fn confirmed(provider_operation: &str, receipt: &str, at_ms: i64) -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: operation(Some(provider_operation)),
        provider_receipt: Some(
            ExternalOpaqueId::new(receipt).expect("fixture provider receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(receipt.as_bytes()),
        confirmed_at_ms: at_ms,
    })
}

fn ambiguous(provider_operation: &str) -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Ambiguous {
        operation: operation(Some(provider_operation)),
        reconciliation_hint: ReconciliationHint {
            strategy: ReconciliationStrategy::ManualReview,
            object: None,
            idempotency_key: None,
            earliest_retry_at_ms: None,
        },
    }
}

fn enqueue_and_dispatch() -> (
    SqliteIntegrationStore,
    i64,
    mycelix_integration_core::ExecutionAttemptId,
) {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };
    let claim = store
        .claim_outbox("worker-operation", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-operation", 111)
        .expect("dispatch boundary must succeed");
    (store, entry_id, claim.attempt_id)
}

#[test]
fn reconciliation_cannot_substitute_a_different_established_provider_operation() {
    let (mut store, entry_id, attempt_id) = enqueue_and_dispatch();
    store
        .record_execution(
            entry_id,
            &attempt_id,
            "worker-operation",
            &confirmed("provider-op-a", "receipt-a", 120),
            120,
        )
        .expect("first exact provider operation should bind the outcome");
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Confirmed
    );

    let wrong_operation_reconciliation = ReconciliationResult {
        operation: operation(Some("provider-op-b")),
        disposition: ReconciliationDisposition::ConfirmsEffect,
        evidence: ContentCommitment::sha256(b"reconciliation-for-provider-op-b"),
        reconciled_at_ms: 130,
    };

    assert!(matches!(
        store.record_reconciliation(entry_id, &wrong_operation_reconciliation, 130),
        Err(RuntimeError::ReconciliationOperationMismatch { entry_id: id }) if id == entry_id
    ));
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Confirmed,
        "provider operation substitution must not advance reconciliation"
    );
}

#[test]
fn late_same_attempt_result_cannot_substitute_a_different_provider_operation() {
    let (mut store, entry_id, attempt_id) = enqueue_and_dispatch();
    store
        .record_execution(
            entry_id,
            &attempt_id,
            "worker-operation",
            &ambiguous("provider-op-a"),
            120,
        )
        .expect("first provider operation should bind the ambiguous outcome");
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Ambiguous
    );

    assert!(matches!(
        store.record_execution(
            entry_id,
            &attempt_id,
            "worker-operation",
            &confirmed("provider-op-b", "receipt-b", 130),
            130,
        ),
        Err(RuntimeError::OutcomeOperationMismatch { entry_id: id }) if id == entry_id
    ));
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Ambiguous,
        "provider operation substitution must remain rejected historical evidence"
    );
}

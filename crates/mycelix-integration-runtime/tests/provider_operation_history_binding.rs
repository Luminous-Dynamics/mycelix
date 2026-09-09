use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExecutionAttemptId, ExternalExecutionOutcome,
    ExternalOpaqueId, ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId,
    ReconciliationDisposition, ReconciliationResult, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, ExecutionRecordDisposition, RuntimeError,
    SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("provider-history-binding")
        .expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("provider-history-binding-command")
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
        command_commitment: ContentCommitment::sha256(b"provider-history-binding-command"),
        authority_commitment: ContentCommitment::sha256(b"provider-history-binding-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("provider-history-binding-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-provider-history-binding-command".to_vec(),
        created_at_ms: 100,
    }
}

fn confirmed(provider_operation: Option<&str>, receipt: &str, at_ms: i64) -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: operation(provider_operation),
        provider_receipt: Some(
            ExternalOpaqueId::new(receipt).expect("fixture receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(receipt.as_bytes()),
        confirmed_at_ms: at_ms,
    })
}

fn reconciliation(at_ms: i64) -> ReconciliationResult {
    ReconciliationResult {
        operation: operation(None),
        disposition: ReconciliationDisposition::ConfirmsEffect,
        evidence: ContentCommitment::sha256(b"provider-history-binding-reconciliation"),
        reconciled_at_ms: at_ms,
    }
}

fn create_finalized_without_provider_operation(
    store: &mut SqliteIntegrationStore,
) -> (i64, ExecutionAttemptId) {
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };
    let claim = store
        .claim_outbox("worker-history-binding", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(
            entry_id,
            &claim.attempt_id,
            "worker-history-binding",
            111,
        )
        .expect("dispatch must succeed");
    store
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-history-binding",
            &confirmed(None, "receipt-without-provider-op", 120),
            120,
        )
        .expect("initial result without provider operation must succeed");
    store
        .record_reconciliation(entry_id, &reconciliation(130), 130)
        .expect("reconciliation without provider operation must succeed");
    store
        .finalize_outbound(entry_id, 140)
        .expect("fixture must finalize");
    (entry_id, claim.attempt_id)
}

#[test]
fn late_historical_provider_operation_binding_survives_reopen_and_rejects_substitution() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("provider-history-binding.sqlite");
    let entry_id;
    let attempt_id;

    {
        let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
        (entry_id, attempt_id) = create_finalized_without_provider_operation(&mut store);

        let disposition = store
            .record_execution(
                entry_id,
                &attempt_id,
                "worker-history-binding",
                &confirmed(Some("provider-op-a"), "late-receipt-a", 150),
                150,
            )
            .expect("late historical evidence may refine provider-operation identity");
        assert_eq!(
            disposition,
            ExecutionRecordDisposition::RecordedForStaleAttempt
        );
    }

    let mut store = SqliteIntegrationStore::open(&path)
        .expect("semantic and provider-operation bindings must survive reopen");

    assert!(matches!(
        store.record_execution(
            entry_id,
            &attempt_id,
            "worker-history-binding",
            &confirmed(Some("provider-op-b"), "late-receipt-b", 160),
            160,
        ),
        Err(RuntimeError::OutcomeOperationMismatch { entry_id: id }) if id == entry_id
    ));

    let same_binding = store
        .record_execution(
            entry_id,
            &attempt_id,
            "worker-history-binding",
            &confirmed(Some("provider-op-a"), "late-receipt-a2", 160),
            160,
        )
        .expect("the exact durable provider-operation binding must remain admissible");
    assert_eq!(
        same_binding,
        ExecutionRecordDisposition::RecordedForStaleAttempt
    );
}

use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalOpaqueId,
    ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId,
    ReconciliationDisposition, ReconciliationResult, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("causal-time-connector").expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("causal-time-command").expect("fixture command must be valid")
}

fn operation() -> ExternalOperationRef {
    ExternalOperationRef {
        command_id: command_id(),
        connector_instance: connector(),
        provider_operation: Some(
            ExternalOpaqueId::new("causal-time-provider-operation")
                .expect("fixture provider operation must be valid"),
        ),
    }
}

fn intent(created_at_ms: i64) -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command_id(),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(b"causal-time-command"),
        authority_commitment: ContentCommitment::sha256(b"causal-time-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("causal-time-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-causal-time-command".to_vec(),
        created_at_ms,
    }
}

fn confirmed(at_ms: i64) -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: operation(),
        provider_receipt: Some(
            ExternalOpaqueId::new("causal-time-receipt").expect("fixture receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(b"causal-time-receipt"),
        confirmed_at_ms: at_ms,
    })
}

fn reconciliation(at_ms: i64) -> ReconciliationResult {
    ReconciliationResult {
        operation: operation(),
        disposition: ReconciliationDisposition::ConfirmsEffect,
        evidence: ContentCommitment::sha256(b"causal-time-reconciliation"),
        reconciled_at_ms: at_ms,
    }
}

fn enqueue(store: &mut SqliteIntegrationStore, created_at_ms: i64) -> i64 {
    match store
        .enqueue_outbound(&intent(created_at_ms))
        .expect("enqueue must succeed")
    {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    }
}

#[test]
fn claim_time_cannot_precede_durable_intent_creation() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = enqueue(&mut store, 100);

    assert!(
        store.claim_outbox("worker-time", 99, 10, 1).is_err(),
        "claim must fail closed when runtime time moves before the entry's durable creation time"
    );
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").attempt_count,
        0,
        "clock rollback must not mint an attempt"
    );
}

#[test]
fn dispatch_time_cannot_precede_claim_time() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = enqueue(&mut store, 100);
    let claim = store
        .claim_outbox("worker-time", 110, 20, 1)
        .expect("claim must succeed")
        .remove(0);

    assert!(
        store
            .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-time", 109)
            .is_err(),
        "dispatch timestamp must not move backward before the claim transition"
    );
}

#[test]
fn provider_result_time_cannot_precede_dispatch_time() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = enqueue(&mut store, 100);
    let claim = store
        .claim_outbox("worker-time", 110, 30, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-time", 111)
        .expect("dispatch must succeed");

    assert!(
        store
            .record_execution(
                entry_id,
                &claim.attempt_id,
                "worker-time",
                &confirmed(110),
                110,
            )
            .is_err(),
        "runtime observation time must not precede the durable dispatch boundary"
    );
}

#[test]
fn reconciliation_and_finalization_times_cannot_move_backward() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = enqueue(&mut store, 100);
    let claim = store
        .claim_outbox("worker-time", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-time", 111)
        .expect("dispatch must succeed");
    store
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-time",
            &confirmed(120),
            120,
        )
        .expect("provider result must succeed");

    assert!(
        store.record_reconciliation(entry_id, &reconciliation(119), 119).is_err(),
        "reconciliation must not predate the provider result it resolves"
    );

    store
        .record_reconciliation(entry_id, &reconciliation(130), 130)
        .expect("forward reconciliation must succeed");
    assert!(
        store.finalize_outbound(entry_id, 129).is_err(),
        "finalization must not predate the reconciliation transition"
    );
}

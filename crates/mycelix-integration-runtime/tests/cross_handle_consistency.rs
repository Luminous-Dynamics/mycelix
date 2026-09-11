use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalOpaqueId,
    ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId, OutboundStage,
    ReconciliationDisposition, ReconciliationResult, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, ExecutionRecordDisposition, RuntimeError,
    SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("cross-handle-connector").expect("fixture connector must be valid")
}

fn command_id(name: &str) -> IntegrationCommandId {
    IntegrationCommandId::new(name).expect("fixture command must be valid")
}

fn operation(name: &str, provider_operation: Option<&str>) -> ExternalOperationRef {
    ExternalOperationRef {
        command_id: command_id(name),
        connector_instance: connector(),
        provider_operation: provider_operation.map(|value| {
            ExternalOpaqueId::new(value).expect("fixture provider operation must be valid")
        }),
    }
}

fn intent(name: &str) -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command_id(name),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(name.as_bytes()),
        authority_commitment: ContentCommitment::sha256(format!("authority:{name}").as_bytes()),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new(format!("idem:{name}"))
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: format!("sealed:{name}").into_bytes(),
        created_at_ms: 100,
    }
}

fn confirmed(
    name: &str,
    provider_operation: Option<&str>,
    receipt: &str,
    at_ms: i64,
) -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: operation(name, provider_operation),
        provider_receipt: Some(
            ExternalOpaqueId::new(receipt).expect("fixture receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(receipt.as_bytes()),
        confirmed_at_ms: at_ms,
    })
}

fn reconciliation(name: &str, at_ms: i64) -> ReconciliationResult {
    ReconciliationResult {
        operation: operation(name, None),
        disposition: ReconciliationDisposition::ConfirmsEffect,
        evidence: ContentCommitment::sha256(format!("reconcile:{name}").as_bytes()),
        reconciled_at_ms: at_ms,
    }
}

fn enqueue(store: &mut SqliteIntegrationStore, name: &str) -> i64 {
    match store.enqueue_outbound(&intent(name)).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    }
}

#[test]
fn stale_handle_cannot_roll_runtime_causal_time_backward() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("cross-handle-time.sqlite");
    let name = "cross-handle-time-command";

    let mut writer = SqliteIntegrationStore::open(&path).expect("writer must open");
    let entry_id = enqueue(&mut writer, name);

    // Reader snapshots the v3.1 causal frontier before another handle advances it.
    let mut stale = SqliteIntegrationStore::open(&path).expect("second handle must open");

    let claim = writer
        .claim_outbox("worker-cross-time", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    writer
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-cross-time", 111)
        .expect("dispatch must succeed");

    assert!(
        stale
            .record_execution(
                entry_id,
                &claim.attempt_id,
                "worker-cross-time",
                &confirmed(name, None, "receipt-too-early", 105),
                105,
            )
            .is_err(),
        "a handle opened before dispatch must still observe the durable causal frontier and reject time rollback"
    );
    assert_eq!(
        stale.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::DispatchStarted,
        "stale process-local time state must not permit an impossible current transition"
    );
}

#[test]
fn stale_handle_observes_new_durable_provider_operation_binding() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("cross-handle-provider.sqlite");
    let name = "cross-handle-provider-command";

    let mut writer = SqliteIntegrationStore::open(&path).expect("writer must open");
    let entry_id = enqueue(&mut writer, name);
    let mut stale = SqliteIntegrationStore::open(&path).expect("second handle must open");

    let claim = writer
        .claim_outbox("worker-cross-provider", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    writer
        .mark_dispatch_started(
            entry_id,
            &claim.attempt_id,
            "worker-cross-provider",
            111,
        )
        .expect("dispatch must succeed");
    writer
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-cross-provider",
            &confirmed(name, None, "receipt-initial", 120),
            120,
        )
        .expect("initial result must succeed");
    writer
        .record_reconciliation(entry_id, &reconciliation(name, 130), 130)
        .expect("reconciliation must succeed");
    writer
        .finalize_outbound(entry_id, 140)
        .expect("fixture must finalize");

    let learned = writer
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-cross-provider",
            &confirmed(name, Some("provider-op-a"), "late-a", 150),
            150,
        )
        .expect("late history may establish provider operation A");
    assert_eq!(learned, ExecutionRecordDisposition::RecordedForStaleAttempt);

    assert!(matches!(
        stale.record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-cross-provider",
            &confirmed(name, Some("provider-op-b"), "late-b", 160),
            160,
        ),
        Err(RuntimeError::OutcomeOperationMismatch { entry_id: id }) if id == entry_id
    ));

    let same = stale
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-cross-provider",
            &confirmed(name, Some("provider-op-a"), "late-a-2", 160),
            160,
        )
        .expect("stale handle must admit the exact newly durable binding");
    assert_eq!(same, ExecutionRecordDisposition::RecordedForStaleAttempt);
}

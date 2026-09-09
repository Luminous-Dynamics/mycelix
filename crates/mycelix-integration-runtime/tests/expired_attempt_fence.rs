use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalObjectRef,
    ExternalOpaqueId, ExternalOperationRef, ExternalReceipt, ExternalSystemId, IdempotencyKey,
    IntegrationCommandId, OutboundStage, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, ExecutionRecordDisposition, SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("expired-fence-connector").expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("expired-fence-command").expect("fixture command must be valid")
}

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command_id(),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(b"expired-fence-command"),
        authority_commitment: ContentCommitment::sha256(b"expired-fence-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("expired-fence-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-expired-fence-command".to_vec(),
        created_at_ms: 100,
    }
}

fn confirmed() -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: ExternalOperationRef {
            command_id: command_id(),
            connector_instance: connector(),
            provider_operation: Some(
                ExternalOpaqueId::new("provider-op-expired")
                    .expect("fixture provider operation must be valid"),
            ),
        },
        provider_receipt: Some(
            ExternalOpaqueId::new("provider-receipt-expired")
                .expect("fixture provider receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(b"provider-receipt-expired"),
        confirmed_at_ms: 121,
    })
}

#[test]
fn expired_post_dispatch_attempt_cannot_complete_current_workflow_before_recovery() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };

    let claim = store
        .claim_outbox("worker-expired", 110, 10, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-expired", 111)
        .expect("dispatch boundary must succeed while lease is live");

    // The lease expired at 120, but recovery has deliberately not run. I-12
    // requires the stale attempt to stop being a current completion capability
    // anyway. The exact provider response remains evidence, not permission to
    // move the workflow directly to Confirmed.
    let disposition = store
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-expired",
            &confirmed(),
            121,
        )
        .expect("late exact result should remain preservable as historical evidence");

    assert_eq!(
        disposition,
        ExecutionRecordDisposition::RecordedForStaleAttempt,
        "expired lease must demote completion to historical evidence even before recovery runs"
    );
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Ambiguous,
        "post-dispatch stale attempt must preserve commit uncertainty rather than complete current state"
    );
}

use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalOpaqueId,
    ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId, OutboundStage,
    ReconciliationDisposition, ReconciliationResult, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, ExecutionRecordDisposition, SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("post-resolution-connector")
        .expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("post-resolution-command")
        .expect("fixture command must be valid")
}

fn operation() -> ExternalOperationRef {
    ExternalOperationRef {
        command_id: command_id(),
        connector_instance: connector(),
        provider_operation: None,
    }
}

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command_id(),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(b"post-resolution-command"),
        authority_commitment: ContentCommitment::sha256(b"post-resolution-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("post-resolution-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-post-resolution-command".to_vec(),
        created_at_ms: 100,
    }
}

fn confirmed(receipt: &str, at_ms: i64) -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: operation(),
        provider_receipt: Some(
            ExternalOpaqueId::new(receipt).expect("fixture receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(receipt.as_bytes()),
        confirmed_at_ms: at_ms,
    })
}

fn reconciliation(
    disposition: ReconciliationDisposition,
    evidence: &[u8],
    at_ms: i64,
) -> ReconciliationResult {
    ReconciliationResult {
        operation: operation(),
        disposition,
        evidence: ContentCommitment::sha256(evidence),
        reconciled_at_ms: at_ms,
    }
}

fn finalized_fixture() -> (
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
        .claim_outbox("worker-history", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-history", 111)
        .expect("dispatch boundary must succeed");
    store
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-history",
            &confirmed("receipt-initial", 120),
            120,
        )
        .expect("initial provider result must apply");
    store
        .record_reconciliation(
            entry_id,
            &reconciliation(
                ReconciliationDisposition::ConfirmsEffect,
                b"initial-reconciliation",
                130,
            ),
            130,
        )
        .expect("initial reconciliation must succeed");
    store
        .finalize_outbound(entry_id, 140)
        .expect("fixture must finalize");
    (store, entry_id, claim.attempt_id)
}

#[test]
fn contradictory_reconciliation_after_finalization_is_preserved_without_state_rewrite() {
    let (mut store, entry_id, _) = finalized_fixture();

    store
        .record_reconciliation(
            entry_id,
            &reconciliation(
                ReconciliationDisposition::ConfirmsNoEffect,
                b"later-contradictory-reconciliation",
                200,
            ),
            200,
        )
        .expect("late exact-operation reconciliation must remain historical evidence");

    let history = store
        .reconciliation_history(entry_id)
        .expect("history must remain readable");
    assert_eq!(history.len(), 2);
    assert_eq!(
        history[1].result.disposition,
        ReconciliationDisposition::ConfirmsNoEffect
    );
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Finalized
    );
}

#[test]
fn late_same_attempt_provider_result_after_finalization_is_preserved_as_non_applying_evidence() {
    let (mut store, entry_id, attempt_id) = finalized_fixture();

    let disposition = store
        .record_execution(
            entry_id,
            &attempt_id,
            "worker-history",
            &confirmed("receipt-late", 210),
            210,
        )
        .expect("late same-attempt provider evidence must be appendable");

    assert_eq!(
        disposition,
        ExecutionRecordDisposition::RecordedForStaleAttempt
    );
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Finalized
    );
}

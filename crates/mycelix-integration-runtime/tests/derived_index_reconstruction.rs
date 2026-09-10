use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExecutionAttemptId, ExternalExecutionOutcome,
    ExternalOpaqueId, ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId,
    ReconciliationDisposition, ReconciliationResult, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, ExecutionRecordDisposition, RuntimeError,
    SqliteIntegrationStore,
};
use rusqlite::{params, Connection};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("derived-index-connector").expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("derived-index-command").expect("fixture command must be valid")
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
        command_commitment: ContentCommitment::sha256(b"derived-index-command"),
        authority_commitment: ContentCommitment::sha256(b"derived-index-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("derived-index-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-derived-index-command".to_vec(),
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
        evidence: ContentCommitment::sha256(b"derived-index-reconciliation"),
        reconciled_at_ms: at_ms,
    }
}

fn finalized_without_provider_operation(
    store: &mut SqliteIntegrationStore,
) -> (i64, ExecutionAttemptId) {
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };
    let claim = store
        .claim_outbox("derived-index-worker", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "derived-index-worker", 111)
        .expect("dispatch must succeed");
    store
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "derived-index-worker",
            &confirmed(None, "receipt-initial", 120),
            120,
        )
        .expect("initial provider result must succeed");
    store
        .record_reconciliation(entry_id, &reconciliation(130), 130)
        .expect("reconciliation must succeed");
    store
        .finalize_outbound(entry_id, 140)
        .expect("fixture must finalize");
    (entry_id, claim.attempt_id)
}

#[test]
fn derived_binding_index_is_reconstructed_from_append_only_history() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("derived-index.sqlite");
    let entry_id;
    let attempt_id;

    {
        let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
        (entry_id, attempt_id) = finalized_without_provider_operation(&mut store);
        let disposition = store
            .record_execution(
                entry_id,
                &attempt_id,
                "derived-index-worker",
                &confirmed(Some("provider-op-a"), "receipt-late-a", 150),
                150,
            )
            .expect("late history may establish provider operation identity");
        assert_eq!(
            disposition,
            ExecutionRecordDisposition::RecordedForStaleAttempt
        );
        assert_eq!(
            store
                .provider_operation_binding(entry_id)
                .expect("binding must load")
                .as_deref(),
            Some("provider-op-a")
        );
    }

    {
        let conn = Connection::open(&path).expect("raw connection must open");
        conn.execute(
            "DELETE FROM integration_runtime_operation_binding WHERE entry_id = ?1",
            params![entry_id],
        )
        .expect("test must remove only the derived index row");
    }

    let mut reopened = SqliteIntegrationStore::open(&path)
        .expect("reopen must reconstruct a missing derived binding index");
    assert_eq!(
        reopened
            .provider_operation_binding(entry_id)
            .expect("reconstructed binding must load")
            .as_deref(),
        Some("provider-op-a")
    );
    assert!(matches!(
        reopened.record_execution(
            entry_id,
            &attempt_id,
            "derived-index-worker",
            &confirmed(Some("provider-op-b"), "receipt-late-b", 160),
            160,
        ),
        Err(RuntimeError::OutcomeOperationMismatch { entry_id: id }) if id == entry_id
    ));
    drop(reopened);

    // A wrong derived row is also repairable because history, not the index,
    // is the source material. The second v3.1 open must load the repaired value
    // rather than retaining the stale pre-reconstruction cache.
    {
        let conn = Connection::open(&path).expect("raw connection must open");
        conn.execute(
            "UPDATE integration_runtime_operation_binding\n\
             SET provider_operation = 'provider-op-stale' WHERE entry_id = ?1",
            params![entry_id],
        )
        .expect("test must corrupt only the derived index row");
    }

    let mut repaired = SqliteIntegrationStore::open(&path)
        .expect("reopen must repair a stale derived binding index");
    assert_eq!(
        repaired
            .provider_operation_binding(entry_id)
            .expect("repaired binding must load")
            .as_deref(),
        Some("provider-op-a")
    );
    let same_binding = repaired
        .record_execution(
            entry_id,
            &attempt_id,
            "derived-index-worker",
            &confirmed(Some("provider-op-a"), "receipt-late-a2", 170),
            170,
        )
        .expect("the reloaded process cache must agree with reconstructed history");
    assert_eq!(
        same_binding,
        ExecutionRecordDisposition::RecordedForStaleAttempt
    );
}

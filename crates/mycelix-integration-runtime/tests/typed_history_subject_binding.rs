use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExecutionAttemptId, ExternalExecutionOutcome,
    ExternalOpaqueId, ExternalOperationRef, ExternalReceipt, IdempotencyKey, IntegrationCommandId,
    ReconciliationDisposition, ReconciliationResult, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, RuntimeError, SqliteIntegrationStore,
};
use rusqlite::{params, Connection};

fn connector(value: &str) -> ConnectorInstanceId {
    ConnectorInstanceId::new(value).expect("fixture connector must be valid")
}

fn command(value: &str) -> IntegrationCommandId {
    IntegrationCommandId::new(value).expect("fixture command must be valid")
}

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command("typed-history-command"),
        connector_instance: connector("typed-history-connector"),
        command_commitment: ContentCommitment::sha256(b"typed-history-command"),
        authority_commitment: ContentCommitment::sha256(b"typed-history-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("typed-history-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-typed-history-command".to_vec(),
        created_at_ms: 100,
    }
}

fn foreign_operation() -> ExternalOperationRef {
    ExternalOperationRef {
        command_id: command("foreign-command"),
        connector_instance: connector("typed-history-connector"),
        provider_operation: Some(
            ExternalOpaqueId::new("provider-op-foreign")
                .expect("fixture provider operation must be valid"),
        ),
    }
}

fn foreign_outcome() -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: foreign_operation(),
        provider_receipt: Some(
            ExternalOpaqueId::new("foreign-receipt").expect("fixture receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(b"foreign-receipt"),
        confirmed_at_ms: 120,
    })
}

fn foreign_reconciliation() -> ReconciliationResult {
    ReconciliationResult {
        operation: ExternalOperationRef {
            command_id: command("typed-history-command"),
            connector_instance: connector("foreign-connector"),
            provider_operation: Some(
                ExternalOpaqueId::new("provider-op-foreign")
                    .expect("fixture provider operation must be valid"),
            ),
        },
        disposition: ReconciliationDisposition::StillAmbiguous,
        evidence: ContentCommitment::sha256(b"foreign-reconciliation"),
        reconciled_at_ms: 121,
    }
}

fn prepare_dispatched(
    store: &mut SqliteIntegrationStore,
) -> (i64, ExecutionAttemptId) {
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };
    let claim = store
        .claim_outbox("typed-history-worker", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(
            entry_id,
            &claim.attempt_id,
            "typed-history-worker",
            111,
        )
        .expect("dispatch must succeed");
    (entry_id, claim.attempt_id)
}

#[test]
fn sqlite_subject_fences_reject_typed_foreign_execution_and_reconciliation() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("typed-history-subject.sqlite");
    let (entry_id, attempt_id) = {
        let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
        prepare_dispatched(&mut store)
    };

    let conn = Connection::open(&path).expect("raw connection must open");
    let outcome_json = serde_json::to_vec(&foreign_outcome()).expect("outcome must serialize");
    assert!(
        conn.execute(
            "INSERT INTO integration_execution_observation (\n\
                 entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\n\
             ) VALUES (?1, ?2, ?3, ?4, 0)",
            params![entry_id, attempt_id.as_str(), outcome_json, 120_i64],
        )
        .is_err(),
        "database boundary must reject typed execution evidence for another command"
    );

    let reconciliation_json =
        serde_json::to_vec(&foreign_reconciliation()).expect("reconciliation must serialize");
    assert!(
        conn.execute(
            "INSERT INTO integration_reconciliation_history (entry_id, result_json, recorded_at_ms)\n\
             VALUES (?1, ?2, ?3)",
            params![entry_id, reconciliation_json, 121_i64],
        )
        .is_err(),
        "database boundary must reject typed reconciliation evidence for another connector"
    );
}

#[test]
fn reopen_rejects_typed_foreign_history_before_reconstruction() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("typed-history-reopen.sqlite");
    let (entry_id, attempt_id) = {
        let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
        prepare_dispatched(&mut store)
    };

    {
        let conn = Connection::open(&path).expect("raw connection must open");
        conn.execute_batch(
            "DROP TRIGGER integration_validate_execution_subject_before;",
        )
        .expect("test must remove only the execution subject fence");
        let outcome_json =
            serde_json::to_vec(&foreign_outcome()).expect("outcome must serialize");
        conn.execute(
            "INSERT INTO integration_execution_observation (\n\
                 entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\n\
             ) VALUES (?1, ?2, ?3, ?4, 0)",
            params![entry_id, attempt_id.as_str(), outcome_json, 120_i64],
        )
        .expect("fixture must persist foreign typed history after removing the subject fence");
    }

    assert!(matches!(
        SqliteIntegrationStore::open(&path),
        Err(RuntimeError::StoredIdentifier(detail)) if detail.contains("subject mismatch")
    ));
}

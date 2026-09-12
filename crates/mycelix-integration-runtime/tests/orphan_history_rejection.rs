use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalOpaqueId,
    ExternalOperationRef, ExternalReceipt, IntegrationCommandId, ReconciliationDisposition,
    ReconciliationResult,
};
use mycelix_integration_runtime::{RuntimeError, SqliteIntegrationStore};
use rusqlite::{params, Connection};

fn operation() -> ExternalOperationRef {
    ExternalOperationRef {
        command_id: IntegrationCommandId::new("orphan-command")
            .expect("fixture command must be valid"),
        connector_instance: ConnectorInstanceId::new("orphan-connector")
            .expect("fixture connector must be valid"),
        provider_operation: Some(
            ExternalOpaqueId::new("orphan-provider-operation")
                .expect("fixture provider operation must be valid"),
        ),
    }
}

fn confirmed() -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Confirmed(ExternalReceipt {
        operation: operation(),
        provider_receipt: Some(
            ExternalOpaqueId::new("orphan-receipt").expect("fixture receipt must be valid"),
        ),
        receipt_commitment: ContentCommitment::sha256(b"orphan-receipt"),
        confirmed_at_ms: 120,
    })
}

fn reconciliation() -> ReconciliationResult {
    ReconciliationResult {
        operation: operation(),
        disposition: ReconciliationDisposition::StillAmbiguous,
        evidence: ContentCommitment::sha256(b"orphan-reconciliation"),
        reconciled_at_ms: 121,
    }
}

fn disable_foreign_keys_for_corrupt_fixture(conn: &Connection) {
    conn.execute_batch("PRAGMA foreign_keys = OFF;")
        .expect("fixture must disable foreign-key enforcement");
    let enabled: i64 = conn
        .pragma_query_value(None, "foreign_keys", |row| row.get(0))
        .expect("fixture must read foreign-key mode");
    assert_eq!(enabled, 0, "fixture requires foreign keys to be disabled");
}

#[test]
fn reopen_rejects_orphan_execution_history() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("orphan-execution.sqlite");
    SqliteIntegrationStore::open(&path).expect("store bootstrap must succeed");

    {
        let conn = Connection::open(&path).expect("raw connection must open");
        disable_foreign_keys_for_corrupt_fixture(&conn);
        conn.execute_batch(
            "DROP TRIGGER integration_validate_execution_subject_before;",
        )
        .expect("test must remove only the execution subject fence");
        let bytes = serde_json::to_vec(&confirmed()).expect("outcome must serialize");
        conn.execute(
            "INSERT INTO integration_execution_observation (\n\
                 entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\n\
             ) VALUES (?1, ?2, ?3, ?4, 0)",
            params![999_i64, "999:1", bytes, 120_i64],
        )
        .expect("fixture must create orphan execution history with foreign keys disabled");
    }

    assert!(matches!(
        SqliteIntegrationStore::open(&path),
        Err(RuntimeError::StoredIdentifier(detail)) if detail.contains("orphan execution history")
    ));
}

#[test]
fn reopen_rejects_orphan_reconciliation_history() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("orphan-reconciliation.sqlite");
    SqliteIntegrationStore::open(&path).expect("store bootstrap must succeed");

    {
        let conn = Connection::open(&path).expect("raw connection must open");
        disable_foreign_keys_for_corrupt_fixture(&conn);
        conn.execute_batch(
            "DROP TRIGGER integration_validate_reconciliation_subject_before;",
        )
        .expect("test must remove only the reconciliation subject fence");
        let bytes = serde_json::to_vec(&reconciliation()).expect("result must serialize");
        conn.execute(
            "INSERT INTO integration_reconciliation_history (entry_id, result_json, recorded_at_ms)\n\
             VALUES (?1, ?2, ?3)",
            params![999_i64, bytes, 121_i64],
        )
        .expect("fixture must create orphan reconciliation history with foreign keys disabled");
    }

    assert!(matches!(
        SqliteIntegrationStore::open(&path),
        Err(RuntimeError::StoredIdentifier(detail)) if detail.contains("orphan reconciliation history")
    ));
}

use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, IdempotencyKey, IntegrationCommandId, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, SqliteIntegrationStore,
};
use rusqlite::{params, Connection};

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: IntegrationCommandId::new("atomic-causal-command")
            .expect("fixture command id must be valid"),
        connector_instance: ConnectorInstanceId::new("atomic-causal-connector")
            .expect("fixture connector id must be valid"),
        command_commitment: ContentCommitment::sha256(b"atomic-causal-command"),
        authority_commitment: ContentCommitment::sha256(b"atomic-causal-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("atomic-causal-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-atomic-causal-command".to_vec(),
        created_at_ms: 100,
    }
}

#[test]
fn sqlite_write_boundary_rejects_runtime_time_rollback() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("atomic-causal.sqlite");

    let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };
    let claim = store
        .claim_outbox("atomic-causal-worker", 110, 100, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "atomic-causal-worker", 111)
        .expect("dispatch boundary must succeed");

    let conn = Connection::open(&path).expect("raw verification connection must open");

    assert!(
        conn.execute(
            "UPDATE integration_outbox SET updated_at_ms = ?1 WHERE entry_id = ?2",
            params![105_i64, entry_id],
        )
        .is_err(),
        "the database must atomically reject an outbox timestamp below the durable causal frontier"
    );

    assert!(
        conn.execute(
            "INSERT INTO integration_execution_observation (\n\
                entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\n\
             ) VALUES (?1, ?2, ?3, ?4, 0)",
            params![
                entry_id,
                claim.attempt_id.as_str(),
                b"{}".as_slice(),
                110_i64
            ],
        )
        .is_err(),
        "the database must atomically reject a backdated execution observation"
    );

    assert!(
        conn.execute(
            "INSERT INTO integration_reconciliation_history (\n\
                entry_id, result_json, recorded_at_ms\n\
             ) VALUES (?1, ?2, ?3)",
            params![entry_id, b"{}".as_slice(), 110_i64],
        )
        .is_err(),
        "the database must atomically reject a backdated reconciliation observation"
    );
}

use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, IdempotencyKey, IntegrationCommandId, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, SqliteIntegrationStore,
    RUNTIME_ENFORCEMENT_PROFILE_V4,
};
use rusqlite::{params, Connection};

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: IntegrationCommandId::new("enforcement-tamper-command")
            .expect("fixture command must be valid"),
        connector_instance: ConnectorInstanceId::new("enforcement-tamper-connector")
            .expect("fixture connector must be valid"),
        command_commitment: ContentCommitment::sha256(b"enforcement-tamper-command"),
        authority_commitment: ContentCommitment::sha256(b"enforcement-tamper-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("enforcement-tamper-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-enforcement-tamper-command".to_vec(),
        created_at_ms: 100,
    }
}

#[test]
fn reopen_replaces_weakened_trigger_definition_before_use() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("enforcement-tamper.sqlite");
    let entry_id;

    {
        let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
        entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected enqueue disposition: {other:?}"),
        };
        let claim = store
            .claim_outbox("enforcement-worker", 110, 100, 1)
            .expect("claim must succeed")
            .remove(0);
        store
            .mark_dispatch_started(entry_id, &claim.attempt_id, "enforcement-worker", 111)
            .expect("dispatch must succeed");
    }

    {
        let conn = Connection::open(&path).expect("raw tamper connection must open");
        conn.execute_batch(
            r#"
            DROP TRIGGER integration_causal_outbox_update_before;
            CREATE TRIGGER integration_causal_outbox_update_before
            BEFORE UPDATE OF updated_at_ms ON integration_outbox
            BEGIN
                SELECT 1;
            END;
            "#,
        )
        .expect("test must install a weaker trigger under the expected name");
        conn.execute(
            "UPDATE integration_runtime_enforcement\n\
             SET enforcement_profile = 'tampered-profile' WHERE singleton = 1",
            [],
        )
        .expect("test must tamper the derived enforcement profile");
    }

    let _reopened = SqliteIntegrationStore::open(&path)
        .expect("reopen must atomically restore reconstructable enforcement machinery");

    let conn = Connection::open(&path).expect("verification connection must open");
    let trigger_sql: String = conn
        .query_row(
            "SELECT sql FROM sqlite_master\n\
             WHERE type = 'trigger' AND name = 'integration_causal_outbox_update_before'",
            [],
            |row| row.get(0),
        )
        .expect("restored trigger must exist");
    assert!(trigger_sql.contains("runtime causal time rollback"));

    let profile: String = conn
        .query_row(
            "SELECT enforcement_profile FROM integration_runtime_enforcement WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("enforcement profile must exist");
    assert_eq!(profile, RUNTIME_ENFORCEMENT_PROFILE_V4);

    assert!(
        conn.execute(
            "UPDATE integration_outbox SET updated_at_ms = ?1 WHERE entry_id = ?2",
            params![105_i64, entry_id],
        )
        .is_err(),
        "the restored trigger must atomically reject causal rollback"
    );
}

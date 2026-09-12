use mycelix_integration_runtime::{
    RUNTIME_STRUCTURAL_MANIFEST_V2, RuntimeError, SqliteIntegrationStore,
};
use rusqlite::Connection;

#[test]
fn unsupported_structural_schema_is_rejected_before_enforcement_repair() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("structural-schema-admission.sqlite");

    {
        let _store = SqliteIntegrationStore::open(&path).expect("fresh store must open");
    }

    {
        let conn = Connection::open(&path).expect("tamper connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 99;
            UPDATE integration_runtime_enforcement
            SET enforcement_profile = 'tampered-enforcement-profile'
            WHERE singleton = 1;

            DROP TRIGGER integration_causal_outbox_update_before;
            CREATE TRIGGER integration_causal_outbox_update_before
            BEFORE UPDATE OF updated_at_ms ON integration_outbox
            BEGIN
                SELECT 1;
            END;
            "#,
        )
        .expect("test must install unsupported structural version and weakened enforcement");
    }

    match SqliteIntegrationStore::open(&path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(
                message.starts_with(RUNTIME_STRUCTURAL_MANIFEST_V2),
                "structural rejection must be owned by the structural manifest: {message}"
            );
            assert!(
                message.contains("structural manifest expected schema v2, observed 99"),
                "structural rejection must bind expected and observed schema versions: {message}"
            );
        }
        Err(other) => panic!("unexpected reopen error: {other:?}"),
        Ok(_) => panic!("unsupported structural schema must fail closed"),
    }

    let conn = Connection::open(&path).expect("verification connection must open");
    let version: i64 = conn
        .query_row("PRAGMA user_version", [], |row| row.get(0))
        .expect("user_version must be readable");
    assert_eq!(version, 99);

    let profile: String = conn
        .query_row(
            "SELECT enforcement_profile FROM integration_runtime_enforcement WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("tampered enforcement profile must remain readable");
    assert_eq!(profile, "tampered-enforcement-profile");

    let trigger_sql: String = conn
        .query_row(
            "SELECT sql FROM sqlite_master\n\
             WHERE type = 'trigger' AND name = 'integration_causal_outbox_update_before'",
            [],
            |row| row.get(0),
        )
        .expect("weakened trigger must remain present");
    assert!(trigger_sql.contains("SELECT 1"));
    assert!(!trigger_sql.contains("runtime causal time rollback"));
}

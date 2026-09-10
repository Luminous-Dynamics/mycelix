use mycelix_integration_runtime::{
    RuntimeError, SqliteIntegrationStore, RUNTIME_STRUCTURAL_MANIFEST_V2,
};
use rusqlite::{params, Connection};

const PRE_HARDENING_PROFILE: &str = "pre-v5-rollback-marker";

#[test]
fn post_manifest_failure_rolls_back_all_hardening_mutations() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("atomic-hardening-rollback.sqlite");

    {
        let _store =
            SqliteIntegrationStore::open(&path).expect("baseline runtime must initialize");
    }

    {
        let conn = Connection::open(&path).expect("tamper connection must open");
        conn.execute_batch(
            r#"
            PRAGMA foreign_keys = OFF;

            CREATE TABLE IF NOT EXISTS integration_runtime_quarantine (
                entry_id INTEGER PRIMARY KEY,
                reason TEXT NOT NULL,
                quarantined_at_ms INTEGER NOT NULL,
                FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
            );

            INSERT INTO integration_runtime_quarantine (
                entry_id, reason, quarantined_at_ms
            ) VALUES (999999, 'orphan-post-manifest-proof', 777);

            DROP TRIGGER integration_causal_outbox_update_before;
            CREATE TRIGGER integration_causal_outbox_update_before
            BEFORE UPDATE OF updated_at_ms ON integration_outbox
            BEGIN
                SELECT 1;
            END;
            "#,
        )
        .expect("post-manifest failure fixture must be installed");
        conn.execute(
            "UPDATE integration_runtime_enforcement\n\
             SET enforcement_profile = ?1 WHERE singleton = 1",
            params![PRE_HARDENING_PROFILE],
        )
        .expect("pre-hardening profile marker must be installed");
    }

    match SqliteIntegrationStore::open(&path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(message.contains(RUNTIME_STRUCTURAL_MANIFEST_V2));
            assert!(message.contains("foreign-key integrity violation"));
        }
        Err(other) => panic!("unexpected post-manifest failure: {other:?}"),
        Ok(_) => panic!("orphan quarantine evidence must make post-manifest admission fail"),
    }

    let conn = Connection::open(&path).expect("verification connection must open");

    let profile: String = conn
        .query_row(
            "SELECT enforcement_profile FROM integration_runtime_enforcement WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("enforcement profile must remain readable");
    assert_eq!(
        profile, PRE_HARDENING_PROFILE,
        "failed post-manifest qualification must roll back the v5 profile update"
    );

    let trigger_sql: String = conn
        .query_row(
            "SELECT sql FROM sqlite_master\n\
             WHERE type = 'trigger' AND name = 'integration_causal_outbox_update_before'",
            [],
            |row| row.get(0),
        )
        .expect("weakened trigger must remain readable after rollback");
    assert!(trigger_sql.contains("SELECT 1"));
    assert!(
        !trigger_sql.contains("runtime causal time rollback"),
        "failed post-manifest qualification must roll back trigger replacement"
    );

    let orphan_count: i64 = conn
        .query_row(
            "SELECT COUNT(*) FROM integration_runtime_quarantine WHERE entry_id = 999999",
            [],
            |row| row.get(0),
        )
        .expect("orphan fixture must remain readable");
    assert_eq!(orphan_count, 1);
}

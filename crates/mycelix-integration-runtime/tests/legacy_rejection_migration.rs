use rusqlite::{params, Connection};
use mycelix_integration_runtime::SqliteIntegrationStore;

fn create_common_tables(conn: &Connection) {
    conn.execute_batch(
        r#"
        CREATE TABLE integration_inbound (
            connector_instance TEXT NOT NULL,
            event_id TEXT NOT NULL,
            commitment_algorithm INTEGER NOT NULL,
            commitment_digest BLOB NOT NULL,
            stage INTEGER NOT NULL,
            normalized_payload BLOB NOT NULL,
            received_at_ms INTEGER NOT NULL,
            PRIMARY KEY (connector_instance, event_id)
        );
        CREATE TABLE integration_reconcile_checkpoint (
            connector_instance TEXT PRIMARY KEY,
            cursor TEXT NOT NULL,
            commitment_algorithm INTEGER NOT NULL,
            commitment_digest BLOB NOT NULL,
            updated_at_ms INTEGER NOT NULL
        );
        "#,
    )
    .expect("common legacy tables must be created");
}

fn insert_legacy_row(conn: &Connection, stage: i64) {
    let digest = [0_u8; 32];
    conn.execute(
        "INSERT INTO integration_outbox (\n\
            command_id, connector_instance, command_commitment_algorithm, command_commitment_digest,\n\
            authority_commitment_algorithm, authority_commitment_digest, side_effect_class,\n\
            command_bytes, stage, worker_id, lease_until_ms, attempt_count, created_at_ms, updated_at_ms\n\
         ) VALUES (?1, ?2, 1, ?3, 1, ?3, 3, ?4, ?5, NULL, NULL, 1, 100, 100)",
        params![
            "legacy-rejected-command",
            "legacy-rejected-connector",
            digest.as_slice(),
            b"sealed".as_slice(),
            stage,
        ],
    )
    .expect("legacy row must be inserted");
}

#[test]
fn implicit_v1_generic_rejection_requires_manual_semantic_migration() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("legacy-v1-rejected.sqlite");
    {
        let conn = Connection::open(&path).expect("legacy database must open");
        conn.execute_batch(
            r#"
            CREATE TABLE integration_outbox (
                entry_id INTEGER PRIMARY KEY AUTOINCREMENT,
                command_id TEXT NOT NULL UNIQUE,
                connector_instance TEXT NOT NULL,
                command_commitment_algorithm INTEGER NOT NULL,
                command_commitment_digest BLOB NOT NULL,
                authority_commitment_algorithm INTEGER NOT NULL,
                authority_commitment_digest BLOB NOT NULL,
                side_effect_class INTEGER NOT NULL,
                idempotency_key TEXT,
                command_bytes BLOB NOT NULL,
                stage INTEGER NOT NULL,
                worker_id TEXT,
                lease_until_ms INTEGER,
                attempt_count INTEGER NOT NULL DEFAULT 0,
                outcome_json BLOB,
                reconciliation_json BLOB,
                created_at_ms INTEGER NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );
            "#,
        )
        .expect("legacy v1 outbox must be created");
        create_common_tables(&conn);

        // In the pre-split v1 state machine, numeric stage 6 was the generic
        // `Rejected` state reachable from both AuthorityChecked and
        // DispatchStarted. The stored row therefore proves no-effect/denial
        // under the old semantics, but not which cause occurred.
        insert_legacy_row(&conn, 6);
    }

    assert!(
        SqliteIntegrationStore::open(&path).is_err(),
        "automatic migration must fail closed rather than relabel generic legacy rejection as provider RejectedBeforeCommit"
    );
}

#[test]
fn v2_stage7_rejection_requires_manual_semantic_migration() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("legacy-v2-rejected.sqlite");
    {
        let conn = Connection::open(&path).expect("legacy database must open");
        conn.execute_batch(
            r#"
            CREATE TABLE integration_outbox (
                entry_id INTEGER PRIMARY KEY AUTOINCREMENT,
                command_id TEXT NOT NULL UNIQUE,
                connector_instance TEXT NOT NULL,
                command_commitment_algorithm INTEGER NOT NULL,
                command_commitment_digest BLOB NOT NULL,
                authority_commitment_algorithm INTEGER NOT NULL,
                authority_commitment_digest BLOB NOT NULL,
                side_effect_class INTEGER NOT NULL,
                idempotency_key TEXT,
                command_bytes BLOB NOT NULL,
                stage INTEGER NOT NULL,
                worker_id TEXT,
                lease_until_ms INTEGER,
                attempt_count INTEGER NOT NULL DEFAULT 0,
                current_attempt_id TEXT,
                dispatch_started_at_ms INTEGER,
                outcome_json BLOB,
                reconciliation_json BLOB,
                created_at_ms INTEGER NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );
            PRAGMA user_version = 2;
            "#,
        )
        .expect("legacy v2 outbox must be created");
        create_common_tables(&conn);

        // Schema v2 existed across the semantic split, so numeric stage 7 is
        // not self-describing enough to distinguish the old generic rejection
        // from the later provider-specific name. Version 2 alone cannot prove
        // which semantic producer wrote the row.
        insert_legacy_row(&conn, 7);
    }

    assert!(
        SqliteIntegrationStore::open(&path).is_err(),
        "v2 stage 7 is semantically underdetermined and must not be silently promoted to provider RejectedBeforeCommit"
    );
}

use mycelix_integration_runtime::{
    RuntimeError, SqliteIntegrationStore, RUNTIME_ENFORCEMENT_PROFILE_V4,
    RUNTIME_SEMANTIC_PROFILE_V31,
};
use rusqlite::Connection;
use std::sync::{Arc, Barrier};
use std::thread;

#[test]
fn concurrent_first_openers_converge_on_one_valid_runtime() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("concurrent-first-open.sqlite");
    let worker_count = 8;
    let barrier = Arc::new(Barrier::new(worker_count));

    let handles: Vec<_> = (0..worker_count)
        .map(|_| {
            let barrier = Arc::clone(&barrier);
            let path = path.clone();
            thread::spawn(move || {
                barrier.wait();
                SqliteIntegrationStore::open(&path)
                    .map(|_| ())
                    .map_err(|error| error.to_string())
            })
        })
        .collect();

    for handle in handles {
        let result = handle.join().expect("first-open worker must not panic");
        assert!(result.is_ok(), "concurrent first-open failed: {result:?}");
    }

    let _store = SqliteIntegrationStore::open(&path).expect("converged runtime must reopen");
    assert_runtime_identity(&path);
}

#[test]
fn empty_partial_v2_bootstrap_resumes_without_legacy_migration() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("partial-v2-bootstrap.sqlite");

    {
        let conn = Connection::open(&path).expect("partial bootstrap connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 2;

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

            CREATE TABLE integration_runtime_semantics (
                singleton INTEGER PRIMARY KEY CHECK (singleton = 1),
                semantic_profile TEXT NOT NULL
            );
            "#,
        )
        .expect("partial structural-v2 fixture must be created");
    }

    let _store = SqliteIntegrationStore::open(&path)
        .expect("empty structural-v2 bootstrap must be safely resumable");
    assert_runtime_identity(&path);

    let conn = Connection::open(&path).expect("verification connection must open");
    let execution_table: i64 = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master\n\
             WHERE type = 'table' AND name = 'integration_execution_observation'",
            [],
            |row| row.get(0),
        )
        .expect("schema catalog query must succeed");
    assert_eq!(execution_table, 1);
}

#[test]
fn nonempty_untagged_store_is_not_promoted_as_incomplete_bootstrap() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("nonempty-untagged.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 2;
            CREATE TABLE integration_outbox (entry_id INTEGER PRIMARY KEY);
            INSERT INTO integration_outbox (entry_id) VALUES (1);
            "#,
        )
        .expect("nonempty untagged fixture must be created");
    }

    match SqliteIntegrationStore::open(&path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(message.contains("no semantic producer identity"));
        }
        Err(other) => panic!("unexpected error for nonempty untagged store: {other:?}"),
        Ok(_) => panic!("nonempty untagged store must never be auto-promoted"),
    }

    let conn = Connection::open(&path).expect("verification connection must open");
    let row_count: i64 = conn
        .query_row("SELECT COUNT(*) FROM integration_outbox", [], |row| row.get(0))
        .expect("outbox count must remain readable");
    assert_eq!(row_count, 1);
    let semantic_table: i64 = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master\n\
             WHERE type = 'table' AND name = 'integration_runtime_semantics'",
            [],
            |row| row.get(0),
        )
        .expect("schema catalog query must succeed");
    assert_eq!(semantic_table, 0);
}

fn assert_runtime_identity(path: &std::path::Path) {
    let conn = Connection::open(path).expect("identity verification connection must open");
    let version: i64 = conn
        .query_row("PRAGMA user_version", [], |row| row.get(0))
        .expect("user_version must be readable");
    assert_eq!(version, 2);

    let semantic_profile: String = conn
        .query_row(
            "SELECT semantic_profile FROM integration_runtime_semantics WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("semantic producer identity must exist");
    assert_eq!(semantic_profile, RUNTIME_SEMANTIC_PROFILE_V31);

    let enforcement_profile: String = conn
        .query_row(
            "SELECT enforcement_profile FROM integration_runtime_enforcement WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("enforcement profile must exist");
    assert_eq!(enforcement_profile, RUNTIME_ENFORCEMENT_PROFILE_V4);
}

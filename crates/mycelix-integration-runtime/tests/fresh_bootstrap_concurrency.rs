use mycelix_integration_runtime::{
    RuntimeError, SqliteIntegrationStore, RUNTIME_BOOTSTRAP_PROFILE_V1,
    RUNTIME_ENFORCEMENT_PROFILE_V5, RUNTIME_SEMANTIC_PROFILE_V31,
};
use rusqlite::{params, Connection};
use std::sync::{Arc, Barrier};
use std::thread;

#[test]
fn concurrent_first_openers_converge_on_one_valid_runtime() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("concurrent-first-open.sqlite");
    let worker_count = 8;
    let barrier = Arc::new(Barrier::new(worker_count));

    assert_eq!(
        RUNTIME_BOOTSTRAP_PROFILE_V1,
        "mycelix-integration-runtime/bootstrap-profile-v1"
    );

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
fn unversioned_historical_mycelix_state_is_not_stamped_or_promoted() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("unversioned-historical-mycelix.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            CREATE TABLE integration_inbound (
                event_id TEXT PRIMARY KEY
            );
            INSERT INTO integration_inbound (event_id) VALUES ('historical-event');
            "#,
        )
        .expect("historical unversioned fixture must be created");
        assert_schema_version(&conn, 0);
    }

    expect_unversioned_foreign_store_rejection(&path);

    let conn = Connection::open(&path).expect("verification connection must open");
    assert_schema_version(&conn, 0);
    let row_count: i64 = conn
        .query_row("SELECT COUNT(*) FROM integration_inbound", [], |row| row.get(0))
        .expect("historical state must remain readable");
    assert_eq!(row_count, 1);
    assert_semantic_table_absent(&conn);
}

#[test]
fn unrelated_unversioned_schema_is_not_annexed_even_when_empty() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("unrelated-unversioned.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            CREATE TABLE app_settings (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );
            "#,
        )
        .expect("unrelated schema fixture must be created");
        assert_schema_version(&conn, 0);
    }

    expect_unversioned_foreign_store_rejection(&path);

    let conn = Connection::open(&path).expect("verification connection must open");
    assert_schema_version(&conn, 0);
    let table_count: i64 = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master WHERE type = 'table' AND name = 'app_settings'",
            [],
            |row| row.get(0),
        )
        .expect("unrelated schema must remain readable");
    assert_eq!(table_count, 1);
    assert_semantic_table_absent(&conn);
}

#[test]
fn unrelated_structural_v2_schema_is_not_annexed_or_semantically_tagged() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("unrelated-v2.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 2;
            CREATE TABLE app_settings (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );
            "#,
        )
        .expect("unrelated v2 schema fixture must be created");
        assert_schema_version(&conn, 2);
    }

    match SqliteIntegrationStore::open(&path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(message.contains("structural-manifest-v2"));
        }
        Err(other) => panic!("unexpected foreign-v2 error: {other:?}"),
        Ok(_) => panic!("foreign structural-v2 database must not be annexed as Mycelix"),
    }

    let conn = Connection::open(&path).expect("verification connection must open");
    assert_schema_version(&conn, 2);
    let table_count: i64 = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master WHERE type = 'table' AND name = 'app_settings'",
            [],
            |row| row.get(0),
        )
        .expect("unrelated v2 schema must remain readable");
    assert_eq!(table_count, 1);
    assert_semantic_table_absent(&conn);
}

#[test]
fn empty_partial_v2_bootstrap_with_empty_semantic_table_resumes() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("partial-v2-empty-semantic.sqlite");

    {
        let conn = Connection::open(&path).expect("partial bootstrap connection must open");
        conn.execute_batch("PRAGMA user_version = 2;")
            .expect("structural intent must be writable");
        create_partial_outbox_schema(&conn);
        conn.execute_batch(
            r#"
            CREATE TABLE integration_runtime_semantics (
                singleton INTEGER PRIMARY KEY CHECK (singleton = 1),
                semantic_profile TEXT NOT NULL
            );
            "#,
        )
        .expect("empty semantic table fixture must be created");
    }

    let _store = SqliteIntegrationStore::open(&path)
        .expect("zero-history structural-v2 bootstrap must be safely resumable");
    assert_runtime_identity(&path);
    assert_execution_table_exists(&path);
}

#[test]
fn empty_partial_v2_bootstrap_with_exact_semantic_profile_resumes() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("partial-v2-exact-semantic.sqlite");

    {
        let conn = Connection::open(&path).expect("partial bootstrap connection must open");
        conn.execute_batch("PRAGMA user_version = 2;")
            .expect("structural intent must be writable");
        create_partial_outbox_schema(&conn);
        conn.execute_batch(
            r#"
            CREATE TABLE integration_runtime_semantics (
                singleton INTEGER PRIMARY KEY CHECK (singleton = 1),
                semantic_profile TEXT NOT NULL
            );
            "#,
        )
        .expect("semantic table fixture must be created");
        conn.execute(
            "INSERT INTO integration_runtime_semantics (singleton, semantic_profile) VALUES (1, ?1)",
            params![RUNTIME_SEMANTIC_PROFILE_V31],
        )
        .expect("exact semantic bootstrap identity must be installed");
    }

    let _store = SqliteIntegrationStore::open(&path)
        .expect("exact-profile zero-history partial bootstrap must resume");
    assert_runtime_identity(&path);
    assert_execution_table_exists(&path);
}

#[test]
fn nonempty_untagged_outbox_is_not_promoted_as_incomplete_bootstrap() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("nonempty-untagged-outbox.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 2;
            CREATE TABLE integration_outbox (entry_id INTEGER PRIMARY KEY);
            INSERT INTO integration_outbox (entry_id) VALUES (1);
            "#,
        )
        .expect("nonempty untagged outbox fixture must be created");
    }

    expect_missing_semantic_rejection(&path);

    let conn = Connection::open(&path).expect("verification connection must open");
    let row_count: i64 = conn
        .query_row("SELECT COUNT(*) FROM integration_outbox", [], |row| row.get(0))
        .expect("outbox count must remain readable");
    assert_eq!(row_count, 1);
    assert_semantic_table_absent(&conn);
}

#[test]
fn inbound_only_untagged_state_is_not_promoted_as_empty_bootstrap() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("inbound-only-untagged.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 2;
            CREATE TABLE integration_inbound (event_id TEXT PRIMARY KEY);
            INSERT INTO integration_inbound (event_id) VALUES ('historical-event');
            "#,
        )
        .expect("inbound-only durable fixture must be created");
    }

    expect_missing_semantic_rejection(&path);

    let conn = Connection::open(&path).expect("verification connection must open");
    let row_count: i64 = conn
        .query_row("SELECT COUNT(*) FROM integration_inbound", [], |row| row.get(0))
        .expect("inbound history must remain readable");
    assert_eq!(row_count, 1);
    assert_semantic_table_absent(&conn);
}

#[test]
fn deleted_autoincrement_activity_blocks_automatic_semantic_adoption() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("deleted-autoincrement-history.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 2;
            CREATE TABLE integration_outbox (
                entry_id INTEGER PRIMARY KEY AUTOINCREMENT
            );
            INSERT INTO integration_outbox DEFAULT VALUES;
            DELETE FROM integration_outbox;
            "#,
        )
        .expect("deleted-history trace fixture must be created");
    }

    expect_missing_semantic_rejection(&path);

    let conn = Connection::open(&path).expect("verification connection must open");
    let live_rows: i64 = conn
        .query_row("SELECT COUNT(*) FROM integration_outbox", [], |row| row.get(0))
        .expect("outbox count must remain readable");
    assert_eq!(live_rows, 0);
    let sequence: i64 = conn
        .query_row(
            "SELECT seq FROM sqlite_sequence WHERE name = 'integration_outbox'",
            [],
            |row| row.get(0),
        )
        .expect("prior AUTOINCREMENT activity must remain visible");
    assert!(sequence > 0);
    assert_semantic_table_absent(&conn);
}

#[test]
fn wrong_semantic_profile_is_never_rewritten_by_bootstrap_recovery() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("wrong-semantic-empty.sqlite");

    {
        let conn = Connection::open(&path).expect("fixture connection must open");
        conn.execute_batch(
            r#"
            PRAGMA user_version = 2;
            CREATE TABLE integration_runtime_semantics (
                singleton INTEGER PRIMARY KEY CHECK (singleton = 1),
                semantic_profile TEXT NOT NULL
            );
            INSERT INTO integration_runtime_semantics (singleton, semantic_profile)
            VALUES (1, 'foreign-runtime/semantic-profile-v1');
            "#,
        )
        .expect("wrong semantic profile fixture must be created");
    }

    match SqliteIntegrationStore::open(&path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(message.contains("unsupported integration runtime semantic profile"));
        }
        Err(other) => panic!("unexpected wrong-profile error: {other:?}"),
        Ok(_) => panic!("wrong semantic producer must never be rewritten by bootstrap recovery"),
    }

    let conn = Connection::open(&path).expect("verification connection must open");
    let profile: String = conn
        .query_row(
            "SELECT semantic_profile FROM integration_runtime_semantics WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .expect("foreign semantic identity must remain readable");
    assert_eq!(profile, "foreign-runtime/semantic-profile-v1");
}

fn create_partial_outbox_schema(conn: &Connection) {
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
        "#,
    )
    .expect("partial outbox schema must be created");
}

fn expect_unversioned_foreign_store_rejection(path: &std::path::Path) {
    match SqliteIntegrationStore::open(path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(message.contains("structural-manifest-v2"));
            assert!(message.contains("observed 0"));
        }
        Err(other) => panic!("unexpected unversioned foreign-store error: {other:?}"),
        Ok(_) => panic!("unversioned database with existing user schema must not be adopted"),
    }
}

fn expect_missing_semantic_rejection(path: &std::path::Path) {
    match SqliteIntegrationStore::open(path) {
        Err(RuntimeError::StoredIdentifier(message)) => {
            assert!(message.contains("no semantic producer identity"));
        }
        Err(other) => panic!("unexpected error for historical untagged store: {other:?}"),
        Ok(_) => panic!("historical untagged store must never be auto-promoted"),
    }
}

fn assert_semantic_table_absent(conn: &Connection) {
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

fn assert_schema_version(conn: &Connection, expected: i64) {
    let version: i64 = conn
        .query_row("PRAGMA user_version", [], |row| row.get(0))
        .expect("user_version must be readable");
    assert_eq!(version, expected);
}

fn assert_execution_table_exists(path: &std::path::Path) {
    let conn = Connection::open(path).expect("verification connection must open");
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

fn assert_runtime_identity(path: &std::path::Path) {
    let conn = Connection::open(path).expect("identity verification connection must open");
    assert_schema_version(&conn, 2);

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
    assert_eq!(enforcement_profile, RUNTIME_ENFORCEMENT_PROFILE_V5);
}

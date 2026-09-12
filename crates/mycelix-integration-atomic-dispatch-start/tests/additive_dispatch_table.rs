use mycelix_integration_runtime::SqliteIntegrationStore;
use rusqlite::Connection;
use tempfile::tempdir;

#[test]
fn additive_dispatch_binding_table_survives_runtime_reopen_hardening() {
    let temp = tempdir().unwrap();
    let path = temp.path().join("runtime.sqlite");

    let store = SqliteIntegrationStore::open(&path).unwrap();
    drop(store);

    let conn = Connection::open(&path).unwrap();
    conn.execute_batch(
        r#"
        PRAGMA foreign_keys = ON;
        CREATE TABLE integration_dispatch_binding_v1 (
            entry_id INTEGER NOT NULL,
            attempt_id TEXT NOT NULL,
            command_id TEXT NOT NULL,
            connector_instance TEXT NOT NULL,
            command_commitment_algorithm INTEGER NOT NULL,
            command_commitment_digest BLOB NOT NULL,
            materialization_digest BLOB NOT NULL,
            output_commitment_algorithm INTEGER NOT NULL,
            output_commitment_digest BLOB NOT NULL,
            admission_digest BLOB NOT NULL,
            determinism_digest BLOB NOT NULL,
            provider_profile_algorithm INTEGER NOT NULL,
            provider_profile_digest BLOB NOT NULL,
            provider_root_algorithm INTEGER NOT NULL,
            provider_root_digest BLOB NOT NULL,
            materializer_release_algorithm INTEGER NOT NULL,
            materializer_release_digest BLOB NOT NULL,
            worker_id TEXT NOT NULL,
            lease_until_ms INTEGER NOT NULL,
            started_at_ms INTEGER NOT NULL,
            binding_digest BLOB NOT NULL,
            PRIMARY KEY(entry_id, attempt_id),
            FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
        );
        "#,
    )
    .unwrap();
    drop(conn);

    // INT-03's startup structural/semantic hardening must continue to admit its
    // own managed substrate when this additive, trigger-free audit table exists.
    let reopened = SqliteIntegrationStore::open(&path).unwrap();
    drop(reopened);
}

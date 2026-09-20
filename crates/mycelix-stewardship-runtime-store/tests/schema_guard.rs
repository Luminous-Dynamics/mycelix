// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_stewardship_runtime_store::{RuntimeStoreError, RuntimeStoreV1};
use rusqlite::Connection;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

fn temp_path(label: &str) -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock after unix epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "mycelix-stewardship-runtime-store-schema-{label}-{}-{nonce}.sqlite",
        std::process::id()
    ))
}

fn remove_sqlite_files(path: &Path) {
    let _ = fs::remove_file(path);
    let base = path.to_string_lossy();
    let _ = fs::remove_file(format!("{base}-wal"));
    let _ = fs::remove_file(format!("{base}-shm"));
}

fn create_then_mutate(label: &str, sql: &str) -> RuntimeStoreError {
    let path = temp_path(label);
    let store = RuntimeStoreV1::create(&path).expect("create exact store");
    drop(store);

    let connection = Connection::open(&path).expect("raw adversarial connection");
    connection.execute_batch(sql).expect("apply adversarial schema mutation");
    drop(connection);

    let error = RuntimeStoreV1::open(&path)
        .err()
        .expect("mutated schema must fail closed");
    remove_sqlite_files(&path);
    error
}

#[test]
fn metadata_schema_id_cannot_hide_live_table_ddl_change() {
    let error = create_then_mutate(
        "added-column",
        "ALTER TABLE state_cells ADD COLUMN surprise TEXT;",
    );
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch(_)));
}

#[test]
fn unexpected_application_index_is_rejected() {
    let error = create_then_mutate(
        "added-index",
        "CREATE INDEX surprise_idx ON state_cells(namespace);",
    );
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch(_)));
}

#[test]
fn unexpected_view_is_rejected() {
    let error = create_then_mutate(
        "added-view",
        "CREATE VIEW surprise_view AS SELECT namespace FROM state_cells;",
    );
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch(_)));
}

#[test]
fn unexpected_trigger_is_rejected() {
    let error = create_then_mutate(
        "added-trigger",
        "CREATE TRIGGER surprise_trigger AFTER UPDATE ON state_cells BEGIN SELECT 1; END;",
    );
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch(_)));
}

#[test]
fn schema_label_drift_is_rejected_even_when_live_ddl_is_unchanged() {
    let path = temp_path("metadata-label");
    let store = RuntimeStoreV1::create(&path).expect("create exact store");
    drop(store);

    let connection = Connection::open(&path).expect("raw adversarial connection");
    connection
        .execute(
            "UPDATE runtime_store_meta SET schema_id = 'sha256:not-the-frozen-schema' WHERE singleton = 1",
            [],
        )
        .expect("mutate metadata label");
    drop(connection);

    let error = RuntimeStoreV1::open(&path)
        .err()
        .expect("metadata drift must fail closed");
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch(_)));
    remove_sqlite_files(&path);
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_stewardship_runtime_store::{
    RuntimeStoreError, RuntimeStoreV1, AUTO_VACUUM_V1, ENCODING_V1, PAGE_SIZE_V1,
    PROFILE_ID_V1, SCHEMA_ID_V1, SCHEMA_VERSION_V1,
};
use rusqlite::{params, Connection};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

const SCHEMA_SQL_V1: &str = "CREATE TABLE runtime_store_meta (\n    singleton INTEGER PRIMARY KEY CHECK (singleton = 1),\n    schema_version INTEGER NOT NULL CHECK (schema_version = 1),\n    profile_id TEXT NOT NULL,\n    schema_id TEXT NOT NULL\n) STRICT;\n\nCREATE TABLE state_cells (\n    namespace TEXT NOT NULL,\n    state_key BLOB NOT NULL,\n    version INTEGER NOT NULL CHECK (version >= 0),\n    commitment BLOB NOT NULL CHECK (length(commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, state_key)\n) STRICT, WITHOUT ROWID;\n\nCREATE TABLE transition_receipts (\n    namespace TEXT NOT NULL,\n    receipt_id BLOB NOT NULL,\n    state_key BLOB NOT NULL,\n    from_version INTEGER NOT NULL CHECK (from_version >= 0),\n    from_commitment BLOB NOT NULL CHECK (length(from_commitment) = 32),\n    to_version INTEGER NOT NULL CHECK (to_version = from_version + 1),\n    to_commitment BLOB NOT NULL CHECK (length(to_commitment) = 32),\n    receipt_commitment BLOB NOT NULL CHECK (length(receipt_commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, receipt_id),\n    UNIQUE (namespace, state_key, to_version)\n) STRICT, WITHOUT ROWID;\n";

fn temp_path(label: &str) -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock after unix epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "mycelix-stewardship-runtime-store-format-{label}-{}-{nonce}.sqlite",
        std::process::id()
    ))
}

fn remove_sqlite_files(path: &Path) {
    let _ = fs::remove_file(path);
    let base = path.to_string_lossy();
    let _ = fs::remove_file(format!("{base}-wal"));
    let _ = fs::remove_file(format!("{base}-shm"));
}

fn set_journal_mode(connection: &Connection, mode: &str) {
    let sql = format!("PRAGMA journal_mode = {mode}");
    let observed: String = connection
        .query_row(&sql, [], |row| row.get(0))
        .expect("set journal mode");
    assert_eq!(observed.to_ascii_lowercase(), mode.to_ascii_lowercase());
}

#[test]
fn create_freezes_persistent_file_format_and_full_integrity() {
    let path = temp_path("exact");
    let store = RuntimeStoreV1::create(&path).expect("create exact store");
    let profile = store.profile_snapshot().expect("profile snapshot");

    assert_eq!(profile.page_size, PAGE_SIZE_V1);
    assert_eq!(profile.auto_vacuum, AUTO_VACUUM_V1);
    assert_eq!(profile.encoding.to_ascii_uppercase(), ENCODING_V1);
    store.integrity_check().expect("full integrity_check");

    drop(store);
    remove_sqlite_files(&path);
}

#[test]
fn changed_page_size_is_rejected_even_if_wal_is_restored() {
    let path = temp_path("page-size");
    let store = RuntimeStoreV1::create(&path).expect("create exact store");
    drop(store);

    let connection = Connection::open(&path).expect("raw adversarial connection");
    set_journal_mode(&connection, "delete");
    connection
        .execute_batch("PRAGMA page_size = 8192; VACUUM;")
        .expect("rewrite page size");
    set_journal_mode(&connection, "wal");
    drop(connection);

    let error = RuntimeStoreV1::open(&path)
        .err()
        .expect("changed page size must fail closed");
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch("page_size")));
    remove_sqlite_files(&path);
}

#[test]
fn changed_auto_vacuum_is_rejected_even_if_wal_is_restored() {
    let path = temp_path("auto-vacuum");
    let store = RuntimeStoreV1::create(&path).expect("create exact store");
    drop(store);

    let connection = Connection::open(&path).expect("raw adversarial connection");
    set_journal_mode(&connection, "delete");
    connection
        .execute_batch("PRAGMA auto_vacuum = FULL; VACUUM;")
        .expect("rewrite auto-vacuum mode");
    set_journal_mode(&connection, "wal");
    drop(connection);

    let error = RuntimeStoreV1::open(&path)
        .err()
        .expect("changed auto-vacuum must fail closed");
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch("auto_vacuum")));
    remove_sqlite_files(&path);
}

#[test]
fn exact_schema_and_labels_cannot_hide_wrong_text_encoding() {
    let path = temp_path("encoding");
    let connection = Connection::open(&path).expect("create adversarial database");
    connection
        .execute_batch(
            "PRAGMA page_size = 4096;\nPRAGMA auto_vacuum = NONE;\nPRAGMA encoding = 'UTF-16le';",
        )
        .expect("set adversarial persistent format");
    set_journal_mode(&connection, "wal");
    connection.execute_batch(SCHEMA_SQL_V1).expect("create exact schema");
    connection
        .execute(
            "INSERT INTO runtime_store_meta(singleton, schema_version, profile_id, schema_id)\
             VALUES (1, ?1, ?2, ?3)",
            params![SCHEMA_VERSION_V1, PROFILE_ID_V1, SCHEMA_ID_V1],
        )
        .expect("insert exact labels");
    connection
        .execute_batch("PRAGMA user_version = 1;")
        .expect("set exact user version");
    drop(connection);

    let error = RuntimeStoreV1::open(&path)
        .err()
        .expect("wrong encoding must fail closed");
    assert!(matches!(error, RuntimeStoreError::ProfileMismatch("encoding")));
    remove_sqlite_files(&path);
}

#[test]
fn online_backup_preserves_exact_persistent_format() {
    let source = temp_path("backup-source");
    let destination = temp_path("backup-destination");
    let store = RuntimeStoreV1::create(&source).expect("create source");
    store.backup_to(&destination).expect("create validated backup");
    drop(store);

    let backup = RuntimeStoreV1::open(&destination).expect("open backup");
    let profile = backup.profile_snapshot().expect("backup profile");
    assert_eq!(profile.page_size, PAGE_SIZE_V1);
    assert_eq!(profile.auto_vacuum, AUTO_VACUUM_V1);
    assert_eq!(profile.encoding.to_ascii_uppercase(), ENCODING_V1);
    backup.integrity_check().expect("backup full integrity_check");
    drop(backup);

    remove_sqlite_files(&source);
    remove_sqlite_files(&destination);
}

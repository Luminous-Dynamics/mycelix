// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Persistent-format and full-integrity admission wrapper for STEW-RUNTIME-STORE-001.
//!
//! The underlying transaction kernel remains in `lib.rs` as a private module so
//! its reviewed CAS/state semantics stay byte-identical. This crate root adds the
//! v1 persistent SQLite file-format, local-file, and full-integrity admission.

#![forbid(unsafe_code)]

#[allow(dead_code)]
#[path = "lib.rs"]
mod legacy;

pub use legacy::{
    CasOutcomeV1, CommittedTransitionV1, InsertOnceOutcomeV1, RuntimeStoreError,
    StateCellKeyV1, StateCellValueV1, TransitionReceiptV1, TransitionWriteV1,
    BUSY_TIMEOUT_MS_V1, MAX_NAMESPACE_UTF8_BYTES_V1, MAX_RECEIPT_ID_BYTES_V1,
    MAX_RECEIPT_PAYLOAD_BYTES_V1, MAX_STATE_KEY_BYTES_V1, MAX_STATE_PAYLOAD_BYTES_V1,
    PROFILE_ID_V1, SCHEMA_ID_V1, SCHEMA_VERSION_V1,
};

use rusqlite::{params, Connection, OpenFlags, TransactionBehavior};
use std::ffi::OsString;
use std::fs::{self, OpenOptions};
use std::io::ErrorKind;
use std::path::{Path, PathBuf};
use std::time::Duration;

pub const PAGE_SIZE_V1: i64 = 4_096;
pub const AUTO_VACUUM_V1: i64 = 0;
pub const ENCODING_V1: &str = "UTF-8";

const SCHEMA_SQL_V1: &str = "CREATE TABLE runtime_store_meta (\n    singleton INTEGER PRIMARY KEY CHECK (singleton = 1),\n    schema_version INTEGER NOT NULL CHECK (schema_version = 1),\n    profile_id TEXT NOT NULL,\n    schema_id TEXT NOT NULL\n) STRICT;\n\nCREATE TABLE state_cells (\n    namespace TEXT NOT NULL,\n    state_key BLOB NOT NULL,\n    version INTEGER NOT NULL CHECK (version >= 0),\n    commitment BLOB NOT NULL CHECK (length(commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, state_key)\n) STRICT, WITHOUT ROWID;\n\nCREATE TABLE transition_receipts (\n    namespace TEXT NOT NULL,\n    receipt_id BLOB NOT NULL,\n    state_key BLOB NOT NULL,\n    from_version INTEGER NOT NULL CHECK (from_version >= 0),\n    from_commitment BLOB NOT NULL CHECK (length(from_commitment) = 32),\n    to_version INTEGER NOT NULL CHECK (to_version = from_version + 1),\n    to_commitment BLOB NOT NULL CHECK (length(to_commitment) = 32),\n    receipt_commitment BLOB NOT NULL CHECK (length(receipt_commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, receipt_id),\n    UNIQUE (namespace, state_key, to_version)\n) STRICT, WITHOUT ROWID;\n";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RuntimeStoreProfileSnapshotV1 {
    pub profile_id: String,
    pub schema_id: String,
    pub schema_version: i64,
    pub page_size: i64,
    pub auto_vacuum: i64,
    pub encoding: String,
    pub journal_mode: String,
    pub synchronous: i64,
    pub foreign_keys: i64,
    pub trusted_schema: i64,
    pub busy_timeout_ms: i64,
    pub sqlite_version: String,
    pub sqlite_source_id: String,
}

/// Closed v1 runtime store with an independent private profile/integrity connection.
/// Neither SQLite connection is exposed publicly.
pub struct RuntimeStoreV1 {
    inner: legacy::RuntimeStoreV1,
    profile_connection: Connection,
    path: PathBuf,
}

impl RuntimeStoreV1 {
    /// Create a new exact-format database. Existing filesystem objects are never overwritten.
    pub fn create(path: impl AsRef<Path>) -> Result<Self, RuntimeStoreError> {
        let path = path.as_ref();
        validate_parent(path)?;
        create_new_database_file(path)?;

        if let Err(error) = initialize_exact_database(path) {
            cleanup_new_database_files(path);
            return Err(error);
        }

        Self::open(path)
    }

    /// Open an existing database without format conversion, initialization, or migration.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RuntimeStoreError> {
        let path = path.as_ref();
        ensure_existing_regular_file(path)?;
        verify_runtime_file_permissions(path)?;
        let inner = legacy::RuntimeStoreV1::open(path)?;
        let profile_connection = open_profile_connection(path)?;
        let store = Self {
            inner,
            profile_connection,
            path: path.to_path_buf(),
        };
        store.verify_profile()?;
        Ok(store)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn profile_snapshot(&self) -> Result<RuntimeStoreProfileSnapshotV1, RuntimeStoreError> {
        let legacy = self.inner.profile_snapshot()?;
        let format = persistent_format_snapshot(&self.profile_connection)?;
        Ok(RuntimeStoreProfileSnapshotV1 {
            profile_id: legacy.profile_id,
            schema_id: legacy.schema_id,
            schema_version: legacy.schema_version,
            page_size: format.page_size,
            auto_vacuum: format.auto_vacuum,
            encoding: format.encoding,
            journal_mode: legacy.journal_mode,
            synchronous: legacy.synchronous,
            foreign_keys: legacy.foreign_keys,
            trusted_schema: legacy.trusted_schema,
            busy_timeout_ms: legacy.busy_timeout_ms,
            sqlite_version: legacy.sqlite_version,
            sqlite_source_id: legacy.sqlite_source_id,
        })
    }

    pub fn verify_profile(&self) -> Result<(), RuntimeStoreError> {
        verify_runtime_file_permissions(&self.path)?;
        self.inner.verify_profile()?;
        let format = persistent_format_snapshot(&self.profile_connection)?;
        if format.page_size != PAGE_SIZE_V1 {
            return Err(RuntimeStoreError::ProfileMismatch("page_size"));
        }
        if format.auto_vacuum != AUTO_VACUUM_V1 {
            return Err(RuntimeStoreError::ProfileMismatch("auto_vacuum"));
        }
        if !format.encoding.eq_ignore_ascii_case(ENCODING_V1) {
            return Err(RuntimeStoreError::ProfileMismatch("encoding"));
        }
        Ok(())
    }

    /// Authoritative reads also require the exact storage/profile admission.
    pub fn load_state(
        &self,
        key: &StateCellKeyV1,
    ) -> Result<Option<StateCellValueV1>, RuntimeStoreError> {
        self.verify_profile()?;
        self.inner.load_state(key)
    }

    /// Authoritative receipt reads also require the exact storage/profile admission.
    pub fn load_receipt(
        &self,
        namespace: &str,
        receipt_id: &[u8],
    ) -> Result<Option<TransitionReceiptV1>, RuntimeStoreError> {
        self.verify_profile()?;
        self.inner.load_receipt(namespace, receipt_id)
    }

    pub fn insert_once(
        &mut self,
        key: &StateCellKeyV1,
        genesis: &StateCellValueV1,
    ) -> Result<InsertOnceOutcomeV1, RuntimeStoreError> {
        self.verify_profile()?;
        self.inner.insert_once(key, genesis)
    }

    pub fn compare_and_swap(
        &mut self,
        key: &StateCellKeyV1,
        transition: &TransitionWriteV1,
    ) -> Result<CasOutcomeV1, RuntimeStoreError> {
        self.verify_profile()?;
        self.inner.compare_and_swap(key, transition)
    }

    /// Cheap diagnostic only. This is not the positive integrity theorem.
    pub fn quick_check(&self) -> Result<(), RuntimeStoreError> {
        self.verify_profile()?;
        self.inner.quick_check()
    }

    /// Full SQLite integrity check used for positive storage/backup evidence.
    pub fn integrity_check(&self) -> Result<(), RuntimeStoreError> {
        self.verify_profile()?;
        run_integrity_check(&self.profile_connection)
    }

    /// Create a consistent online backup, requiring full integrity before and after copying.
    pub fn backup_to(&self, destination: impl AsRef<Path>) -> Result<(), RuntimeStoreError> {
        self.verify_profile()?;
        self.integrity_check()?;
        let destination = destination.as_ref();
        self.inner.backup_to(destination)?;
        let backup = Self::open(destination)?;
        backup.integrity_check()?;
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PersistentFormatSnapshotV1 {
    page_size: i64,
    auto_vacuum: i64,
    encoding: String,
}

fn initialize_exact_database(path: &Path) -> Result<(), RuntimeStoreError> {
    let mut connection = Connection::open_with_flags(path, OpenFlags::SQLITE_OPEN_READ_WRITE)?;
    connection.busy_timeout(Duration::from_millis(BUSY_TIMEOUT_MS_V1))?;

    connection.execute_batch(
        "PRAGMA page_size = 4096;\nPRAGMA auto_vacuum = NONE;\nPRAGMA encoding = 'UTF-8';",
    )?;
    let format = persistent_format_snapshot(&connection)?;
    if format.page_size != PAGE_SIZE_V1 {
        return Err(RuntimeStoreError::ProfileMismatch("page_size"));
    }
    if format.auto_vacuum != AUTO_VACUUM_V1 {
        return Err(RuntimeStoreError::ProfileMismatch("auto_vacuum"));
    }
    if !format.encoding.eq_ignore_ascii_case(ENCODING_V1) {
        return Err(RuntimeStoreError::ProfileMismatch("encoding"));
    }

    let journal_mode: String =
        connection.query_row("PRAGMA journal_mode = WAL", [], |row| row.get(0))?;
    if !journal_mode.eq_ignore_ascii_case("wal") {
        return Err(RuntimeStoreError::ProfileMismatch("journal_mode"));
    }
    connection.execute_batch(
        "PRAGMA synchronous = FULL;\nPRAGMA foreign_keys = ON;\nPRAGMA trusted_schema = OFF;",
    )?;

    let tx = connection.transaction_with_behavior(TransactionBehavior::Immediate)?;
    tx.execute_batch(SCHEMA_SQL_V1)?;
    tx.execute(
        "INSERT INTO runtime_store_meta(singleton, schema_version, profile_id, schema_id)\
         VALUES (1, ?1, ?2, ?3)",
        params![SCHEMA_VERSION_V1, PROFILE_ID_V1, SCHEMA_ID_V1],
    )?;
    tx.execute_batch("PRAGMA user_version = 1;")?;
    tx.commit()?;
    Ok(())
}

fn open_profile_connection(path: &Path) -> Result<Connection, RuntimeStoreError> {
    ensure_existing_regular_file(path)?;
    let connection = Connection::open_with_flags(path, OpenFlags::SQLITE_OPEN_READ_WRITE)?;
    connection.busy_timeout(Duration::from_millis(BUSY_TIMEOUT_MS_V1))?;
    Ok(connection)
}

fn persistent_format_snapshot(
    connection: &Connection,
) -> Result<PersistentFormatSnapshotV1, RuntimeStoreError> {
    Ok(PersistentFormatSnapshotV1 {
        page_size: connection.query_row("PRAGMA page_size", [], |row| row.get(0))?,
        auto_vacuum: connection.query_row("PRAGMA auto_vacuum", [], |row| row.get(0))?,
        encoding: connection.query_row("PRAGMA encoding", [], |row| row.get(0))?,
    })
}

fn run_integrity_check(connection: &Connection) -> Result<(), RuntimeStoreError> {
    let mut statement = connection.prepare("PRAGMA integrity_check")?;
    let rows = statement.query_map([], |row| row.get::<_, String>(0))?;
    let mut results = Vec::new();
    for row in rows {
        results.push(row?);
    }
    if results.len() == 1 && results[0] == "ok" {
        Ok(())
    } else {
        Err(RuntimeStoreError::IntegrityFailure(format!(
            "integrity_check: {}",
            results.join("; ")
        )))
    }
}

fn validate_parent(path: &Path) -> Result<(), RuntimeStoreError> {
    if path.as_os_str().is_empty() || path.file_name().is_none() {
        return Err(RuntimeStoreError::InvalidPath);
    }
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    let metadata = fs::symlink_metadata(parent)?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(RuntimeStoreError::InvalidPath);
    }
    Ok(())
}

fn create_new_database_file(path: &Path) -> Result<(), RuntimeStoreError> {
    match fs::symlink_metadata(path) {
        Ok(_) => return Err(RuntimeStoreError::PathAlreadyExists),
        Err(error) if error.kind() == ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    let mut options = OpenOptions::new();
    options.read(true).write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    options.open(path)?;
    Ok(())
}

fn cleanup_new_database_files(path: &Path) {
    for candidate in [sidecar_path(path, "-wal"), sidecar_path(path, "-shm"), path.to_path_buf()] {
        let _ = fs::remove_file(candidate);
    }
}

fn ensure_existing_regular_file(path: &Path) -> Result<(), RuntimeStoreError> {
    let metadata = match fs::symlink_metadata(path) {
        Ok(metadata) => metadata,
        Err(error) if error.kind() == ErrorKind::NotFound => {
            return Err(RuntimeStoreError::PathMissing)
        }
        Err(error) => return Err(error.into()),
    };
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(RuntimeStoreError::NonRegularFile);
    }
    Ok(())
}

fn sidecar_path(path: &Path, suffix: &str) -> PathBuf {
    let mut value: OsString = path.as_os_str().to_os_string();
    value.push(suffix);
    PathBuf::from(value)
}

fn verify_runtime_file_permissions(path: &Path) -> Result<(), RuntimeStoreError> {
    verify_one_runtime_file(path, false)?;
    verify_one_runtime_file(&sidecar_path(path, "-wal"), true)?;
    verify_one_runtime_file(&sidecar_path(path, "-shm"), true)?;
    Ok(())
}

fn verify_one_runtime_file(path: &Path, optional: bool) -> Result<(), RuntimeStoreError> {
    let metadata = match fs::symlink_metadata(path) {
        Ok(metadata) => metadata,
        Err(error) if optional && error.kind() == ErrorKind::NotFound => return Ok(()),
        Err(error) if error.kind() == ErrorKind::NotFound => return Err(RuntimeStoreError::PathMissing),
        Err(error) => return Err(error.into()),
    };
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(RuntimeStoreError::ProfileMismatch("runtime file type"));
    }

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if metadata.permissions().mode() & 0o077 != 0 {
            return Err(RuntimeStoreError::ProfileMismatch("runtime file permissions"));
        }
    }

    Ok(())
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Closed SQLite transaction substrate for local stewardship authority state.
//!
//! This crate proves no stewardship semantics on its own. It deliberately owns
//! only a small persistence surface: exact profile/schema verification,
//! single-assignment initialization, conditional compare-and-swap, append-only
//! transition receipts, integrity checking, and SQLite online backup.

#![forbid(unsafe_code)]

use rusqlite::backup::Backup;
use rusqlite::{params, Connection, OpenFlags, OptionalExtension, TransactionBehavior};
use std::fmt;
use std::fs::{self, OpenOptions};
use std::io::ErrorKind;
use std::path::{Path, PathBuf};
use std::time::Duration;

pub const PROFILE_ID_V1: &str = "mycelix-stewardship-runtime-store-sqlite-wal-full-v1";
pub const SCHEMA_VERSION_V1: i64 = 1;
pub const SCHEMA_ID_V1: &str =
    "sha256:1b6459705d0ee05990ae41075d87ac91afd36c567ab1328dd14c6e161bca5900";
pub const BUSY_TIMEOUT_MS_V1: u64 = 5_000;
pub const MAX_NAMESPACE_UTF8_BYTES_V1: usize = 128;
pub const MAX_STATE_KEY_BYTES_V1: usize = 256;
pub const MAX_RECEIPT_ID_BYTES_V1: usize = 256;
pub const MAX_STATE_PAYLOAD_BYTES_V1: usize = 1024 * 1024;
pub const MAX_RECEIPT_PAYLOAD_BYTES_V1: usize = 1024 * 1024;

const SCHEMA_SQL_V1: &str = "CREATE TABLE runtime_store_meta (\n    singleton INTEGER PRIMARY KEY CHECK (singleton = 1),\n    schema_version INTEGER NOT NULL CHECK (schema_version = 1),\n    profile_id TEXT NOT NULL,\n    schema_id TEXT NOT NULL\n) STRICT;\n\nCREATE TABLE state_cells (\n    namespace TEXT NOT NULL,\n    state_key BLOB NOT NULL,\n    version INTEGER NOT NULL CHECK (version >= 0),\n    commitment BLOB NOT NULL CHECK (length(commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, state_key)\n) STRICT, WITHOUT ROWID;\n\nCREATE TABLE transition_receipts (\n    namespace TEXT NOT NULL,\n    receipt_id BLOB NOT NULL,\n    state_key BLOB NOT NULL,\n    from_version INTEGER NOT NULL CHECK (from_version >= 0),\n    from_commitment BLOB NOT NULL CHECK (length(from_commitment) = 32),\n    to_version INTEGER NOT NULL CHECK (to_version = from_version + 1),\n    to_commitment BLOB NOT NULL CHECK (length(to_commitment) = 32),\n    receipt_commitment BLOB NOT NULL CHECK (length(receipt_commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, receipt_id),\n    UNIQUE (namespace, state_key, to_version)\n) STRICT, WITHOUT ROWID;\n";

const META_SCHEMA_SQL_V1: &str = "CREATE TABLE runtime_store_meta (\n    singleton INTEGER PRIMARY KEY CHECK (singleton = 1),\n    schema_version INTEGER NOT NULL CHECK (schema_version = 1),\n    profile_id TEXT NOT NULL,\n    schema_id TEXT NOT NULL\n) STRICT";
const STATE_SCHEMA_SQL_V1: &str = "CREATE TABLE state_cells (\n    namespace TEXT NOT NULL,\n    state_key BLOB NOT NULL,\n    version INTEGER NOT NULL CHECK (version >= 0),\n    commitment BLOB NOT NULL CHECK (length(commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, state_key)\n) STRICT, WITHOUT ROWID";
const RECEIPT_SCHEMA_SQL_V1: &str = "CREATE TABLE transition_receipts (\n    namespace TEXT NOT NULL,\n    receipt_id BLOB NOT NULL,\n    state_key BLOB NOT NULL,\n    from_version INTEGER NOT NULL CHECK (from_version >= 0),\n    from_commitment BLOB NOT NULL CHECK (length(from_commitment) = 32),\n    to_version INTEGER NOT NULL CHECK (to_version = from_version + 1),\n    to_commitment BLOB NOT NULL CHECK (length(to_commitment) = 32),\n    receipt_commitment BLOB NOT NULL CHECK (length(receipt_commitment) = 32),\n    payload BLOB NOT NULL,\n    PRIMARY KEY (namespace, receipt_id),\n    UNIQUE (namespace, state_key, to_version)\n) STRICT, WITHOUT ROWID";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StateCellKeyV1 {
    namespace: String,
    key: Vec<u8>,
}

impl StateCellKeyV1 {
    pub fn new(
        namespace: impl Into<String>,
        key: impl Into<Vec<u8>>,
    ) -> Result<Self, RuntimeStoreError> {
        let namespace = namespace.into();
        let key = key.into();
        validate_namespace(&namespace)?;
        validate_bounded_binary(&key, MAX_STATE_KEY_BYTES_V1)?;
        Ok(Self { namespace, key })
    }

    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    pub fn key(&self) -> &[u8] {
        &self.key
    }

    fn validate(&self) -> Result<(), RuntimeStoreError> {
        validate_namespace(&self.namespace)?;
        validate_bounded_binary(&self.key, MAX_STATE_KEY_BYTES_V1)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StateCellValueV1 {
    version: u64,
    commitment: [u8; 32],
    payload: Vec<u8>,
}

impl StateCellValueV1 {
    pub fn new(
        version: u64,
        commitment: [u8; 32],
        payload: impl Into<Vec<u8>>,
    ) -> Result<Self, RuntimeStoreError> {
        checked_sql_version(version)?;
        let payload = payload.into();
        if payload.len() > MAX_STATE_PAYLOAD_BYTES_V1 {
            return Err(RuntimeStoreError::InputTooLarge);
        }
        Ok(Self {
            version,
            commitment,
            payload,
        })
    }

    pub fn version(&self) -> u64 {
        self.version
    }

    pub fn commitment(&self) -> [u8; 32] {
        self.commitment
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    fn validate(&self) -> Result<(), RuntimeStoreError> {
        checked_sql_version(self.version)?;
        if self.payload.len() > MAX_STATE_PAYLOAD_BYTES_V1 {
            return Err(RuntimeStoreError::InputTooLarge);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransitionReceiptV1 {
    namespace: String,
    receipt_id: Vec<u8>,
    state_key: Vec<u8>,
    from_version: u64,
    from_commitment: [u8; 32],
    to_version: u64,
    to_commitment: [u8; 32],
    receipt_commitment: [u8; 32],
    payload: Vec<u8>,
}

impl TransitionReceiptV1 {
    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    pub fn receipt_id(&self) -> &[u8] {
        &self.receipt_id
    }

    pub fn state_key(&self) -> &[u8] {
        &self.state_key
    }

    pub fn from_version(&self) -> u64 {
        self.from_version
    }

    pub fn from_commitment(&self) -> [u8; 32] {
        self.from_commitment
    }

    pub fn to_version(&self) -> u64 {
        self.to_version
    }

    pub fn to_commitment(&self) -> [u8; 32] {
        self.to_commitment
    }

    pub fn receipt_commitment(&self) -> [u8; 32] {
        self.receipt_commitment
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransitionWriteV1 {
    expected_version: u64,
    expected_commitment: [u8; 32],
    next: StateCellValueV1,
    receipt_id: Vec<u8>,
    receipt_commitment: [u8; 32],
    receipt_payload: Vec<u8>,
}

impl TransitionWriteV1 {
    pub fn new(
        expected_version: u64,
        expected_commitment: [u8; 32],
        next: StateCellValueV1,
        receipt_id: impl Into<Vec<u8>>,
        receipt_commitment: [u8; 32],
        receipt_payload: impl Into<Vec<u8>>,
    ) -> Result<Self, RuntimeStoreError> {
        checked_sql_version(expected_version)?;
        let expected_next = expected_version
            .checked_add(1)
            .ok_or(RuntimeStoreError::VersionOutOfRange)?;
        if next.version != expected_next {
            return Err(RuntimeStoreError::VersionStepRequired);
        }
        let receipt_id = receipt_id.into();
        validate_bounded_binary(&receipt_id, MAX_RECEIPT_ID_BYTES_V1)?;
        let receipt_payload = receipt_payload.into();
        if receipt_payload.len() > MAX_RECEIPT_PAYLOAD_BYTES_V1 {
            return Err(RuntimeStoreError::InputTooLarge);
        }
        Ok(Self {
            expected_version,
            expected_commitment,
            next,
            receipt_id,
            receipt_commitment,
            receipt_payload,
        })
    }

    pub fn expected_version(&self) -> u64 {
        self.expected_version
    }

    pub fn expected_commitment(&self) -> [u8; 32] {
        self.expected_commitment
    }

    pub fn next(&self) -> &StateCellValueV1 {
        &self.next
    }

    pub fn receipt_id(&self) -> &[u8] {
        &self.receipt_id
    }

    fn validate(&self) -> Result<(), RuntimeStoreError> {
        checked_sql_version(self.expected_version)?;
        self.next.validate()?;
        if self.next.version
            != self
                .expected_version
                .checked_add(1)
                .ok_or(RuntimeStoreError::VersionOutOfRange)?
        {
            return Err(RuntimeStoreError::VersionStepRequired);
        }
        validate_bounded_binary(&self.receipt_id, MAX_RECEIPT_ID_BYTES_V1)?;
        if self.receipt_payload.len() > MAX_RECEIPT_PAYLOAD_BYTES_V1 {
            return Err(RuntimeStoreError::InputTooLarge);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InsertOnceOutcomeV1 {
    Inserted,
    ExistingExact,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommittedTransitionV1 {
    state: StateCellValueV1,
    receipt: TransitionReceiptV1,
}

impl CommittedTransitionV1 {
    pub fn state(&self) -> &StateCellValueV1 {
        &self.state
    }

    pub fn receipt(&self) -> &TransitionReceiptV1 {
        &self.receipt
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CasOutcomeV1 {
    Applied(Box<CommittedTransitionV1>),
    Stale(Box<StateCellValueV1>),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RuntimeStoreProfileSnapshotV1 {
    pub profile_id: String,
    pub schema_id: String,
    pub schema_version: i64,
    pub journal_mode: String,
    pub synchronous: i64,
    pub foreign_keys: i64,
    pub trusted_schema: i64,
    pub busy_timeout_ms: i64,
    pub sqlite_version: String,
    pub sqlite_source_id: String,
}

pub struct RuntimeStoreV1 {
    connection: Connection,
    path: PathBuf,
}

impl RuntimeStoreV1 {
    /// Create a brand-new authority database. Existing filesystem objects are never overwritten.
    pub fn create(path: impl AsRef<Path>) -> Result<Self, RuntimeStoreError> {
        let path = path.as_ref();
        validate_parent(path)?;
        create_new_database_file(path)?;
        let mut connection = open_existing_connection(path)?;
        configure_new_database_connection(&connection)?;
        initialize_schema(&mut connection)?;
        let store = Self {
            connection,
            path: path.to_path_buf(),
        };
        store.verify_profile()?;
        Ok(store)
    }

    /// Open an existing database without silently initializing or migrating it.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RuntimeStoreError> {
        let path = path.as_ref();
        ensure_existing_regular_file(path)?;
        let connection = open_existing_connection(path)?;
        configure_existing_database_connection(&connection)?;
        let store = Self {
            connection,
            path: path.to_path_buf(),
        };
        store.verify_profile()?;
        Ok(store)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn profile_snapshot(&self) -> Result<RuntimeStoreProfileSnapshotV1, RuntimeStoreError> {
        profile_snapshot(&self.connection)
    }

    pub fn verify_profile(&self) -> Result<(), RuntimeStoreError> {
        verify_live_schema(&self.connection)?;
        let snapshot = self.profile_snapshot()?;
        if snapshot.profile_id != PROFILE_ID_V1 {
            return Err(RuntimeStoreError::ProfileMismatch("profile_id"));
        }
        if snapshot.schema_id != SCHEMA_ID_V1 {
            return Err(RuntimeStoreError::ProfileMismatch("schema_id"));
        }
        if snapshot.schema_version != SCHEMA_VERSION_V1 {
            return Err(RuntimeStoreError::ProfileMismatch("schema_version"));
        }
        if !snapshot.journal_mode.eq_ignore_ascii_case("wal") {
            return Err(RuntimeStoreError::ProfileMismatch("journal_mode"));
        }
        if snapshot.synchronous != 2 {
            return Err(RuntimeStoreError::ProfileMismatch("synchronous"));
        }
        if snapshot.foreign_keys != 1 {
            return Err(RuntimeStoreError::ProfileMismatch("foreign_keys"));
        }
        if snapshot.trusted_schema != 0 {
            return Err(RuntimeStoreError::ProfileMismatch("trusted_schema"));
        }
        if snapshot.busy_timeout_ms != BUSY_TIMEOUT_MS_V1 as i64 {
            return Err(RuntimeStoreError::ProfileMismatch("busy_timeout"));
        }
        Ok(())
    }

    pub fn load_state(
        &self,
        key: &StateCellKeyV1,
    ) -> Result<Option<StateCellValueV1>, RuntimeStoreError> {
        key.validate()?;
        load_state_from_connection(&self.connection, key)
    }

    pub fn load_receipt(
        &self,
        namespace: &str,
        receipt_id: &[u8],
    ) -> Result<Option<TransitionReceiptV1>, RuntimeStoreError> {
        validate_namespace(namespace)?;
        validate_bounded_binary(receipt_id, MAX_RECEIPT_ID_BYTES_V1)?;
        load_receipt_from_connection(&self.connection, namespace, receipt_id)
    }

    /// Single-assignment initialization. Only version-zero state is accepted.
    pub fn insert_once(
        &mut self,
        key: &StateCellKeyV1,
        genesis: &StateCellValueV1,
    ) -> Result<InsertOnceOutcomeV1, RuntimeStoreError> {
        self.verify_profile()?;
        key.validate()?;
        genesis.validate()?;
        if genesis.version != 0 {
            return Err(RuntimeStoreError::GenesisVersionRequired);
        }

        let tx = self
            .connection
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        if let Some(existing) = load_state_from_connection(&tx, key)? {
            if existing == *genesis {
                return Ok(InsertOnceOutcomeV1::ExistingExact);
            }
            return Err(RuntimeStoreError::InitializationConflict);
        }

        tx.execute(
            "INSERT INTO state_cells(namespace, state_key, version, commitment, payload)\
             VALUES (?1, ?2, ?3, ?4, ?5)",
            params![
                key.namespace(),
                key.key(),
                sql_version(genesis.version)?,
                genesis.commitment.as_slice(),
                genesis.payload.as_slice(),
            ],
        )?;
        tx.commit()?;
        Ok(InsertOnceOutcomeV1::Inserted)
    }

    /// Atomically replace one exact state and append its transition receipt.
    pub fn compare_and_swap(
        &mut self,
        key: &StateCellKeyV1,
        transition: &TransitionWriteV1,
    ) -> Result<CasOutcomeV1, RuntimeStoreError> {
        self.verify_profile()?;
        key.validate()?;
        transition.validate()?;

        let tx = self
            .connection
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        let current = load_state_from_connection(&tx, key)?
            .ok_or(RuntimeStoreError::StateNotFound)?;
        if current.version != transition.expected_version
            || current.commitment != transition.expected_commitment
        {
            return Ok(CasOutcomeV1::Stale(Box::new(current)));
        }
        if load_receipt_from_connection(&tx, key.namespace(), transition.receipt_id())?.is_some() {
            return Err(RuntimeStoreError::ReceiptConflict);
        }

        let affected = tx.execute(
            "UPDATE state_cells\
             SET version = ?1, commitment = ?2, payload = ?3\
             WHERE namespace = ?4 AND state_key = ?5 AND version = ?6 AND commitment = ?7",
            params![
                sql_version(transition.next.version)?,
                transition.next.commitment.as_slice(),
                transition.next.payload.as_slice(),
                key.namespace(),
                key.key(),
                sql_version(transition.expected_version)?,
                transition.expected_commitment.as_slice(),
            ],
        )?;
        if affected != 1 {
            return Err(RuntimeStoreError::CorruptState);
        }

        tx.execute(
            "INSERT INTO transition_receipts(\
                 namespace, receipt_id, state_key, from_version, from_commitment,\
                 to_version, to_commitment, receipt_commitment, payload\
             ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)",
            params![
                key.namespace(),
                transition.receipt_id.as_slice(),
                key.key(),
                sql_version(transition.expected_version)?,
                transition.expected_commitment.as_slice(),
                sql_version(transition.next.version)?,
                transition.next.commitment.as_slice(),
                transition.receipt_commitment.as_slice(),
                transition.receipt_payload.as_slice(),
            ],
        )?;

        let receipt = TransitionReceiptV1 {
            namespace: key.namespace.clone(),
            receipt_id: transition.receipt_id.clone(),
            state_key: key.key.clone(),
            from_version: transition.expected_version,
            from_commitment: transition.expected_commitment,
            to_version: transition.next.version,
            to_commitment: transition.next.commitment,
            receipt_commitment: transition.receipt_commitment,
            payload: transition.receipt_payload.clone(),
        };
        let committed = CommittedTransitionV1 {
            state: transition.next.clone(),
            receipt,
        };
        tx.commit()?;
        Ok(CasOutcomeV1::Applied(Box::new(committed)))
    }

    pub fn quick_check(&self) -> Result<(), RuntimeStoreError> {
        let mut statement = self.connection.prepare("PRAGMA quick_check")?;
        let rows = statement.query_map([], |row| row.get::<_, String>(0))?;
        let mut results = Vec::new();
        for row in rows {
            results.push(row?);
        }
        if results.len() == 1 && results[0] == "ok" {
            Ok(())
        } else {
            Err(RuntimeStoreError::IntegrityFailure(results.join("; ")))
        }
    }

    /// Create a consistent SQLite online backup. Raw copying of the main DB file is not exposed.
    pub fn backup_to(&self, destination: impl AsRef<Path>) -> Result<(), RuntimeStoreError> {
        self.verify_profile()?;
        let destination = destination.as_ref();
        validate_parent(destination)?;
        create_new_database_file(destination)?;
        let mut destination_connection = open_existing_connection(destination)?;
        {
            let backup = Backup::new(&self.connection, &mut destination_connection)?;
            backup.run_to_completion(16, Duration::from_millis(10), None)?;
        }
        drop(destination_connection);
        let backup_store = Self::open(destination)?;
        backup_store.quick_check()?;
        Ok(())
    }
}

#[derive(Debug)]
pub enum RuntimeStoreError {
    InvalidPath,
    PathAlreadyExists,
    PathMissing,
    NonRegularFile,
    InvalidReference,
    InputTooLarge,
    VersionOutOfRange,
    GenesisVersionRequired,
    VersionStepRequired,
    ProfileMismatch(&'static str),
    InitializationConflict,
    ReceiptConflict,
    StateNotFound,
    CorruptState,
    IntegrityFailure(String),
    Sqlite(Box<rusqlite::Error>),
    Io(std::io::Error),
}

impl fmt::Display for RuntimeStoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPath => write!(f, "runtime-store path is invalid for the v1 profile"),
            Self::PathAlreadyExists => write!(f, "runtime-store create path already exists"),
            Self::PathMissing => write!(f, "runtime-store path does not exist"),
            Self::NonRegularFile => {
                write!(f, "runtime-store path must be a regular non-symlink file")
            }
            Self::InvalidReference => {
                write!(f, "runtime-store reference is empty, non-canonical, or oversized")
            }
            Self::InputTooLarge => write!(f, "runtime-store bounded input exceeds the v1 limit"),
            Self::VersionOutOfRange => {
                write!(f, "runtime-store version exceeds SQLite INTEGER range")
            }
            Self::GenesisVersionRequired => {
                write!(f, "single-assignment initialization requires version zero")
            }
            Self::VersionStepRequired => {
                write!(f, "compare-and-swap requires exactly one version increment")
            }
            Self::ProfileMismatch(field) => write!(f, "runtime-store profile mismatch: {field}"),
            Self::InitializationConflict => {
                write!(f, "state cell is already initialized to a different value")
            }
            Self::ReceiptConflict => write!(f, "transition receipt identity is already in use"),
            Self::StateNotFound => write!(f, "state cell does not exist"),
            Self::CorruptState => write!(f, "runtime-store state invariant failed during mutation"),
            Self::IntegrityFailure(detail) => write!(f, "SQLite quick_check failed: {detail}"),
            Self::Sqlite(source) => write!(f, "SQLite error: {source}"),
            Self::Io(source) => write!(f, "I/O error: {source}"),
        }
    }
}

impl std::error::Error for RuntimeStoreError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Sqlite(source) => Some(source.as_ref()),
            Self::Io(source) => Some(source),
            _ => None,
        }
    }
}

impl From<rusqlite::Error> for RuntimeStoreError {
    fn from(value: rusqlite::Error) -> Self {
        Self::Sqlite(Box::new(value))
    }
}

impl From<std::io::Error> for RuntimeStoreError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

fn validate_namespace(value: &str) -> Result<(), RuntimeStoreError> {
    if value.is_empty()
        || value.len() > MAX_NAMESPACE_UTF8_BYTES_V1
        || value.trim() != value
        || value.chars().any(char::is_control)
    {
        return Err(RuntimeStoreError::InvalidReference);
    }
    Ok(())
}

fn validate_bounded_binary(value: &[u8], max: usize) -> Result<(), RuntimeStoreError> {
    if value.is_empty() || value.len() > max {
        return Err(RuntimeStoreError::InvalidReference);
    }
    Ok(())
}

fn checked_sql_version(value: u64) -> Result<i64, RuntimeStoreError> {
    i64::try_from(value).map_err(|_| RuntimeStoreError::VersionOutOfRange)
}

fn sql_version(value: u64) -> Result<i64, RuntimeStoreError> {
    checked_sql_version(value)
}

fn rust_version(value: i64) -> Result<u64, RuntimeStoreError> {
    u64::try_from(value).map_err(|_| RuntimeStoreError::CorruptState)
}

fn blob32(value: Vec<u8>) -> Result<[u8; 32], RuntimeStoreError> {
    value.try_into().map_err(|_| RuntimeStoreError::CorruptState)
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

fn open_existing_connection(path: &Path) -> Result<Connection, RuntimeStoreError> {
    ensure_existing_regular_file(path)?;
    Ok(Connection::open_with_flags(
        path,
        OpenFlags::SQLITE_OPEN_READ_WRITE,
    )?)
}

fn configure_new_database_connection(connection: &Connection) -> Result<(), RuntimeStoreError> {
    let journal_mode: String =
        connection.query_row("PRAGMA journal_mode = WAL", [], |row| row.get(0))?;
    if !journal_mode.eq_ignore_ascii_case("wal") {
        return Err(RuntimeStoreError::ProfileMismatch("journal_mode"));
    }
    configure_session_pragmas(connection)
}

fn configure_existing_database_connection(
    connection: &Connection,
) -> Result<(), RuntimeStoreError> {
    let journal_mode: String =
        connection.query_row("PRAGMA journal_mode", [], |row| row.get(0))?;
    if !journal_mode.eq_ignore_ascii_case("wal") {
        return Err(RuntimeStoreError::ProfileMismatch("journal_mode"));
    }
    configure_session_pragmas(connection)
}

fn configure_session_pragmas(connection: &Connection) -> Result<(), RuntimeStoreError> {
    connection.busy_timeout(Duration::from_millis(BUSY_TIMEOUT_MS_V1))?;
    connection.execute_batch(
        "PRAGMA synchronous = FULL;\nPRAGMA foreign_keys = ON;\nPRAGMA trusted_schema = OFF;",
    )?;
    Ok(())
}

fn initialize_schema(connection: &mut Connection) -> Result<(), RuntimeStoreError> {
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

fn verify_live_schema(connection: &Connection) -> Result<(), RuntimeStoreError> {
    let expected = [
        ("runtime_store_meta", META_SCHEMA_SQL_V1),
        ("state_cells", STATE_SCHEMA_SQL_V1),
        ("transition_receipts", RECEIPT_SCHEMA_SQL_V1),
    ];
    let mut statement = connection.prepare(
        "SELECT name, sql FROM sqlite_schema\
         WHERE type = 'table' AND name NOT LIKE 'sqlite_%' ORDER BY name",
    )?;
    let rows = statement.query_map([], |row| {
        Ok((row.get::<_, String>(0)?, row.get::<_, String>(1)?))
    })?;
    let observed: Vec<(String, String)> = rows.collect::<Result<_, _>>()?;
    if observed.len() != expected.len() {
        return Err(RuntimeStoreError::ProfileMismatch("sqlite_schema table set"));
    }
    for ((observed_name, observed_sql), (expected_name, expected_sql)) in
        observed.iter().zip(expected)
    {
        if observed_name != expected_name || observed_sql != expected_sql {
            return Err(RuntimeStoreError::ProfileMismatch("sqlite_schema table DDL"));
        }
    }

    let unexpected_objects: i64 = connection.query_row(
        "SELECT COUNT(*) FROM sqlite_schema\
         WHERE type IN ('view', 'trigger')\
            OR (type = 'index' AND sql IS NOT NULL)",
        [],
        |row| row.get(0),
    )?;
    if unexpected_objects != 0 {
        return Err(RuntimeStoreError::ProfileMismatch(
            "unexpected sqlite_schema object",
        ));
    }
    Ok(())
}

fn profile_snapshot(
    connection: &Connection,
) -> Result<RuntimeStoreProfileSnapshotV1, RuntimeStoreError> {
    let journal_mode = connection.query_row("PRAGMA journal_mode", [], |row| row.get(0))?;
    let synchronous = connection.query_row("PRAGMA synchronous", [], |row| row.get(0))?;
    let foreign_keys = connection.query_row("PRAGMA foreign_keys", [], |row| row.get(0))?;
    let trusted_schema = connection.query_row("PRAGMA trusted_schema", [], |row| row.get(0))?;
    let busy_timeout_ms = connection.query_row("PRAGMA busy_timeout", [], |row| row.get(0))?;
    let schema_version = connection.query_row("PRAGMA user_version", [], |row| row.get(0))?;
    let sqlite_version =
        connection.query_row("SELECT sqlite_version()", [], |row| row.get(0))?;
    let sqlite_source_id =
        connection.query_row("SELECT sqlite_source_id()", [], |row| row.get(0))?;
    let (meta_version, profile_id, schema_id): (i64, String, String) = connection.query_row(
        "SELECT schema_version, profile_id, schema_id FROM runtime_store_meta WHERE singleton = 1",
        [],
        |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
    )?;
    let meta_count: i64 =
        connection.query_row("SELECT COUNT(*) FROM runtime_store_meta", [], |row| row.get(0))?;
    if meta_count != 1 || meta_version != schema_version {
        return Err(RuntimeStoreError::ProfileMismatch("runtime_store_meta"));
    }
    Ok(RuntimeStoreProfileSnapshotV1 {
        profile_id,
        schema_id,
        schema_version,
        journal_mode,
        synchronous,
        foreign_keys,
        trusted_schema,
        busy_timeout_ms,
        sqlite_version,
        sqlite_source_id,
    })
}

fn load_state_from_connection(
    connection: &Connection,
    key: &StateCellKeyV1,
) -> Result<Option<StateCellValueV1>, RuntimeStoreError> {
    connection
        .query_row(
            "SELECT version, commitment, payload FROM state_cells\
             WHERE namespace = ?1 AND state_key = ?2",
            params![key.namespace(), key.key()],
            |row| {
                Ok((
                    row.get::<_, i64>(0)?,
                    row.get::<_, Vec<u8>>(1)?,
                    row.get::<_, Vec<u8>>(2)?,
                ))
            },
        )
        .optional()?
        .map(|(version, commitment, payload)| {
            StateCellValueV1::new(rust_version(version)?, blob32(commitment)?, payload)
        })
        .transpose()
}

fn load_receipt_from_connection(
    connection: &Connection,
    namespace: &str,
    receipt_id: &[u8],
) -> Result<Option<TransitionReceiptV1>, RuntimeStoreError> {
    let raw = connection
        .query_row(
            "SELECT state_key, from_version, from_commitment, to_version, to_commitment,\
                    receipt_commitment, payload\
             FROM transition_receipts WHERE namespace = ?1 AND receipt_id = ?2",
            params![namespace, receipt_id],
            |row| {
                Ok((
                    row.get::<_, Vec<u8>>(0)?,
                    row.get::<_, i64>(1)?,
                    row.get::<_, Vec<u8>>(2)?,
                    row.get::<_, i64>(3)?,
                    row.get::<_, Vec<u8>>(4)?,
                    row.get::<_, Vec<u8>>(5)?,
                    row.get::<_, Vec<u8>>(6)?,
                ))
            },
        )
        .optional()?;
    raw.map(
        |(
            state_key,
            from_version,
            from_commitment,
            to_version,
            to_commitment,
            receipt_commitment,
            payload,
        )| {
            validate_bounded_binary(&state_key, MAX_STATE_KEY_BYTES_V1)?;
            if payload.len() > MAX_RECEIPT_PAYLOAD_BYTES_V1 {
                return Err(RuntimeStoreError::CorruptState);
            }
            Ok(TransitionReceiptV1 {
                namespace: namespace.to_owned(),
                receipt_id: receipt_id.to_vec(),
                state_key,
                from_version: rust_version(from_version)?,
                from_commitment: blob32(from_commitment)?,
                to_version: rust_version(to_version)?,
                to_commitment: blob32(to_commitment)?,
                receipt_commitment: blob32(receipt_commitment)?,
                payload,
            })
        },
    )
    .transpose()
}

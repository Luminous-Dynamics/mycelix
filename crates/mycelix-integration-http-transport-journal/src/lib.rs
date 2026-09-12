//! Crash-safe transport-attempt journal for HTTP issuance.
//!
//! This crate performs no network I/O. It makes the future native transport obey
//! a durable uncertainty ratchet:
//!
//! ```text
//! context-bound DispatchStarted
//!   -> Prepared (no application-write capability issued)
//!   -> WriteMayBegin (durable ambiguity floor)
//!   -> ObservationRecorded
//! ```
//!
//! A future application-write API must require `ArmedHttpApplicationWrite`. If a
//! process dies while only `Prepared`, recovery can prove that this journal never
//! issued a write capability. If it dies after `WriteMayBegin` without a durable
//! observation, recovery is `AmbiguousPossibleIssue` even if the underlying
//! syscall might never have emitted a byte.

#[cfg(not(unix))]
compile_error!("HTTP transport journal v0.1 requires Unix file identity semantics");

use mycelix_integration_atomic_dispatch_context::QualifiedContextBoundDispatchStart;
use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, DigestAlgorithm, ExecutionAttemptId,
    IntegrationCommandId,
};
use mycelix_integration_http_issuance_descriptor::QualifiedHttpIssuanceDescriptor;
use mycelix_integration_transport_observation::{
    QualifiedTransportObservation, TransportObservationDisposition,
};
use rusqlite::{params, Connection, OpenFlags, OptionalExtension, TransactionBehavior};
use std::fs;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};
use std::time::Duration;
use thiserror::Error;

const TABLE: &str = "integration_http_transport_attempt_v1";
const DISPATCH_STARTED_STAGE: i64 = 5;
const JOURNAL_PREPARED: i64 = 0;
const JOURNAL_WRITE_MAY_BEGIN: i64 = 1;
const JOURNAL_OBSERVATION_RECORDED: i64 = 2;

/// Linear-ish process-local capability proving an exact issuance descriptor is
/// durably attached to one exact context-bound DispatchStarted attempt, but no
/// application-write capability has yet been issued by this journal.
pub struct PreparedHttpTransportAttempt {
    entry_id: i64,
    attempt_id: ExecutionAttemptId,
    command_id: IntegrationCommandId,
    connector_instance: ConnectorInstanceId,
    dispatch_binding_digest: mycelix_institutional_core::Digest32,
    issuance_digest: mycelix_institutional_core::Digest32,
    observation_request_commitment: ContentCommitment,
    valid_until_ms: u64,
    prepared_at_ms: i64,
    store_device: u64,
    store_inode: u64,
}

impl PreparedHttpTransportAttempt {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &ExecutionAttemptId {
        &self.attempt_id
    }

    pub fn issuance_digest(&self) -> mycelix_institutional_core::Digest32 {
        self.issuance_digest
    }

    pub fn observation_request_commitment(&self) -> &ContentCommitment {
        &self.observation_request_commitment
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn application_write_capability_issued_here(&self) -> bool {
        false
    }

    pub const fn provider_io_performed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Process-local capability that may be consumed by the future native
/// application-write path. Its existence means the ambiguity floor was already
/// durably crossed before the first application-write call is permitted.
pub struct ArmedHttpApplicationWrite {
    entry_id: i64,
    attempt_id: ExecutionAttemptId,
    command_id: IntegrationCommandId,
    connector_instance: ConnectorInstanceId,
    dispatch_binding_digest: mycelix_institutional_core::Digest32,
    issuance_digest: mycelix_institutional_core::Digest32,
    observation_request_commitment: ContentCommitment,
    valid_until_ms: u64,
    armed_at_ms: i64,
    store_device: u64,
    store_inode: u64,
}

impl ArmedHttpApplicationWrite {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &ExecutionAttemptId {
        &self.attempt_id
    }

    pub fn issuance_digest(&self) -> mycelix_institutional_core::Digest32 {
        self.issuance_digest
    }

    pub fn observation_request_commitment(&self) -> &ContentCommitment {
        &self.observation_request_commitment
    }

    pub fn armed_at_ms(&self) -> i64 {
        self.armed_at_ms
    }

    pub const fn ambiguity_floor_crossed_here(&self) -> bool {
        true
    }

    pub const fn provider_io_performed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransportJournalRecovery {
    NotPrepared,
    /// The journal prepared an issuance descriptor but never minted the required
    /// application-write capability. This is a capability fact, not independent
    /// proof that arbitrary out-of-band code performed no I/O.
    NoWriteCapabilityIssued,
    /// `WriteMayBegin` was durable and no conclusive observation was recorded.
    AmbiguousPossibleIssue,
    ObservationRecorded(TransportObservationDisposition),
}

/// Attach one exact secret-free issuance descriptor to one exact durable v2
/// dispatch before any transport-write capability can exist.
pub fn prepare_http_transport_attempt(
    dispatch: &QualifiedContextBoundDispatchStart,
    descriptor: &QualifiedHttpIssuanceDescriptor,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> Result<PreparedHttpTransportAttempt, TransportJournalError> {
    let now_u64 = to_u64_time(now_ms)?;
    if now_ms < dispatch.dispatch().started_at_ms || !descriptor.is_live_at(now_u64) {
        return Err(TransportJournalError::InvalidTime);
    }

    let store_device = dispatch.store_device();
    let store_inode = dispatch.store_inode();
    let canonical = canonical_exact_store_path(store_path.as_ref(), store_device, store_inode)?;
    let mut conn = open_exact_store(&canonical, store_device, store_inode)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    ensure_table(&tx)?;
    validate_dispatch_row(&tx, dispatch, descriptor)?;

    let entry_id = dispatch.dispatch().entry_id;
    let attempt_id = dispatch.dispatch().attempt_id.clone();
    let existing: Option<i64> = tx
        .query_row(
            "SELECT stage FROM integration_http_transport_attempt_v1\n\
             WHERE entry_id = ?1 AND attempt_id = ?2",
            params![entry_id, attempt_id.as_str()],
            |row| row.get(0),
        )
        .optional()?;
    if existing.is_some() {
        return Err(TransportJournalError::JournalAlreadyExists);
    }

    let request_commitment = observation_request_commitment(descriptor);
    let operation = &dispatch.dispatch().operation;
    tx.execute(
        "INSERT INTO integration_http_transport_attempt_v1 (\n\
            entry_id, attempt_id, command_id, connector_instance,\n\
            dispatch_binding_digest, issuance_digest,\n\
            request_commitment_algorithm, request_commitment_digest,\n\
            descriptor_valid_until_ms, stage, prepared_at_ms, armed_at_ms,\n\
            observation_digest, observation_disposition, updated_at_ms\n\
         ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, NULL, NULL, NULL, ?11)",
        params![
            entry_id,
            attempt_id.as_str(),
            operation.command_id.as_str(),
            operation.connector_instance.as_str(),
            dispatch.binding_digest().0.as_slice(),
            descriptor.issuance_digest().0.as_slice(),
            digest_algorithm_code(request_commitment.algorithm),
            request_commitment.digest.as_slice(),
            i64::try_from(descriptor.valid_until_ms())
                .map_err(|_| TransportJournalError::TimeOverflow)?,
            JOURNAL_PREPARED,
            now_ms,
        ],
    )?;

    let prepared = PreparedHttpTransportAttempt {
        entry_id,
        attempt_id,
        command_id: operation.command_id.clone(),
        connector_instance: operation.connector_instance.clone(),
        dispatch_binding_digest: dispatch.binding_digest(),
        issuance_digest: descriptor.issuance_digest(),
        observation_request_commitment: request_commitment,
        valid_until_ms: descriptor.valid_until_ms(),
        prepared_at_ms: now_ms,
        store_device,
        store_inode,
    };

    // LAST FALLIBLE OPERATION. The returned capability reflects a durable row.
    tx.commit()?;
    Ok(prepared)
}

/// Durably cross the uncertainty floor before the future engine is allowed to
/// invoke its first application-request write.
pub fn arm_http_application_write(
    prepared: PreparedHttpTransportAttempt,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> Result<ArmedHttpApplicationWrite, TransportJournalError> {
    let now_u64 = to_u64_time(now_ms)?;
    if now_ms < prepared.prepared_at_ms || now_u64 >= prepared.valid_until_ms {
        return Err(TransportJournalError::InvalidTime);
    }
    let canonical = canonical_exact_store_path(
        store_path.as_ref(),
        prepared.store_device,
        prepared.store_inode,
    )?;
    let mut conn = open_exact_store(&canonical, prepared.store_device, prepared.store_inode)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    ensure_table(&tx)?;
    validate_journal_identity_prepared(&tx, &prepared)?;

    let changed = tx.execute(
        "UPDATE integration_http_transport_attempt_v1\n\
         SET stage = ?1, armed_at_ms = ?2, updated_at_ms = ?2\n\
         WHERE entry_id = ?3 AND attempt_id = ?4 AND stage = ?5\n\
           AND dispatch_binding_digest = ?6 AND issuance_digest = ?7",
        params![
            JOURNAL_WRITE_MAY_BEGIN,
            now_ms,
            prepared.entry_id,
            prepared.attempt_id.as_str(),
            JOURNAL_PREPARED,
            prepared.dispatch_binding_digest.0.as_slice(),
            prepared.issuance_digest.0.as_slice(),
        ],
    )?;
    if changed != 1 {
        return Err(TransportJournalError::ConcurrentJournalChange);
    }

    let armed = ArmedHttpApplicationWrite {
        entry_id: prepared.entry_id,
        attempt_id: prepared.attempt_id,
        command_id: prepared.command_id,
        connector_instance: prepared.connector_instance,
        dispatch_binding_digest: prepared.dispatch_binding_digest,
        issuance_digest: prepared.issuance_digest,
        observation_request_commitment: prepared.observation_request_commitment,
        valid_until_ms: prepared.valid_until_ms,
        armed_at_ms: now_ms,
        store_device: prepared.store_device,
        store_inode: prepared.store_inode,
    };

    // LAST FALLIBLE OPERATION. Once this commits, crash recovery is ambiguous
    // until a durable transport observation says otherwise.
    tx.commit()?;
    Ok(armed)
}

/// Record a conclusive zero-application-byte failure before the ambiguity floor
/// was crossed (for example connect/TLS failure in the eventual native engine).
pub fn record_prewrite_nonissuance(
    prepared: PreparedHttpTransportAttempt,
    observation: &QualifiedTransportObservation,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> Result<(), TransportJournalError> {
    if observation.disposition != TransportObservationDisposition::DefinitelyNotIssued
        || observation.evidence.application_bytes_written != 0
    {
        return Err(TransportJournalError::PrewriteObservationNotNonissuance);
    }
    validate_observation_identity_prepared(&prepared, observation, now_ms)?;
    record_observation_from_prepared(&prepared, observation, store_path, now_ms)
}

/// Record the transport observation after the durable `WriteMayBegin` fence.
pub fn record_armed_observation(
    armed: ArmedHttpApplicationWrite,
    observation: &QualifiedTransportObservation,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> Result<(), TransportJournalError> {
    validate_observation_identity_armed(&armed, observation, now_ms)?;
    let canonical = canonical_exact_store_path(
        store_path.as_ref(),
        armed.store_device,
        armed.store_inode,
    )?;
    let mut conn = open_exact_store(&canonical, armed.store_device, armed.store_inode)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    ensure_table(&tx)?;
    let changed = tx.execute(
        "UPDATE integration_http_transport_attempt_v1\n\
         SET stage = ?1, observation_digest = ?2, observation_disposition = ?3, updated_at_ms = ?4\n\
         WHERE entry_id = ?5 AND attempt_id = ?6 AND stage = ?7\n\
           AND dispatch_binding_digest = ?8 AND issuance_digest = ?9",
        params![
            JOURNAL_OBSERVATION_RECORDED,
            observation.observation_digest.0.as_slice(),
            disposition_code(observation.disposition),
            now_ms,
            armed.entry_id,
            armed.attempt_id.as_str(),
            JOURNAL_WRITE_MAY_BEGIN,
            armed.dispatch_binding_digest.0.as_slice(),
            armed.issuance_digest.0.as_slice(),
        ],
    )?;
    if changed != 1 {
        return Err(TransportJournalError::ConcurrentJournalChange);
    }
    tx.commit()?;
    Ok(())
}

pub fn recover_http_transport_attempt(
    dispatch: &QualifiedContextBoundDispatchStart,
    store_path: impl AsRef<Path>,
) -> Result<TransportJournalRecovery, TransportJournalError> {
    let canonical = canonical_exact_store_path(
        store_path.as_ref(),
        dispatch.store_device(),
        dispatch.store_inode(),
    )?;
    let conn = open_exact_store(&canonical, dispatch.store_device(), dispatch.store_inode())?;
    let table_exists: bool = conn.query_row(
        "SELECT EXISTS(SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1)",
        params![TABLE],
        |row| row.get(0),
    )?;
    if !table_exists {
        return Ok(TransportJournalRecovery::NotPrepared);
    }
    let row: Option<(i64, Option<i64>)> = conn
        .query_row(
            "SELECT stage, observation_disposition\n\
             FROM integration_http_transport_attempt_v1\n\
             WHERE entry_id = ?1 AND attempt_id = ?2 AND dispatch_binding_digest = ?3",
            params![
                dispatch.dispatch().entry_id,
                dispatch.dispatch().attempt_id.as_str(),
                dispatch.binding_digest().0.as_slice(),
            ],
            |row| Ok((row.get(0)?, row.get(1)?)),
        )
        .optional()?;
    match row {
        None => Ok(TransportJournalRecovery::NotPrepared),
        Some((JOURNAL_PREPARED, None)) => Ok(TransportJournalRecovery::NoWriteCapabilityIssued),
        Some((JOURNAL_WRITE_MAY_BEGIN, None)) => {
            Ok(TransportJournalRecovery::AmbiguousPossibleIssue)
        }
        Some((JOURNAL_OBSERVATION_RECORDED, Some(code))) => Ok(
            TransportJournalRecovery::ObservationRecorded(disposition_from_code(code)?),
        ),
        _ => Err(TransportJournalError::InvalidJournalState),
    }
}

fn record_observation_from_prepared(
    prepared: &PreparedHttpTransportAttempt,
    observation: &QualifiedTransportObservation,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> Result<(), TransportJournalError> {
    let canonical = canonical_exact_store_path(
        store_path.as_ref(),
        prepared.store_device,
        prepared.store_inode,
    )?;
    let mut conn = open_exact_store(&canonical, prepared.store_device, prepared.store_inode)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    ensure_table(&tx)?;
    validate_journal_identity_prepared(&tx, prepared)?;
    let changed = tx.execute(
        "UPDATE integration_http_transport_attempt_v1\n\
         SET stage = ?1, observation_digest = ?2, observation_disposition = ?3, updated_at_ms = ?4\n\
         WHERE entry_id = ?5 AND attempt_id = ?6 AND stage = ?7",
        params![
            JOURNAL_OBSERVATION_RECORDED,
            observation.observation_digest.0.as_slice(),
            disposition_code(observation.disposition),
            now_ms,
            prepared.entry_id,
            prepared.attempt_id.as_str(),
            JOURNAL_PREPARED,
        ],
    )?;
    if changed != 1 {
        return Err(TransportJournalError::ConcurrentJournalChange);
    }
    tx.commit()?;
    Ok(())
}

fn validate_observation_identity_prepared(
    prepared: &PreparedHttpTransportAttempt,
    observation: &QualifiedTransportObservation,
    now_ms: i64,
) -> Result<(), TransportJournalError> {
    validate_observation_identity(
        &prepared.attempt_id,
        &prepared.command_id,
        &prepared.connector_instance,
        &prepared.observation_request_commitment,
        prepared.prepared_at_ms,
        observation,
        now_ms,
    )
}

fn validate_observation_identity_armed(
    armed: &ArmedHttpApplicationWrite,
    observation: &QualifiedTransportObservation,
    now_ms: i64,
) -> Result<(), TransportJournalError> {
    validate_observation_identity(
        &armed.attempt_id,
        &armed.command_id,
        &armed.connector_instance,
        &armed.observation_request_commitment,
        armed.armed_at_ms,
        observation,
        now_ms,
    )
}

fn validate_observation_identity(
    attempt_id: &ExecutionAttemptId,
    command_id: &IntegrationCommandId,
    connector: &ConnectorInstanceId,
    request_commitment: &ContentCommitment,
    causal_floor_ms: i64,
    observation: &QualifiedTransportObservation,
    now_ms: i64,
) -> Result<(), TransportJournalError> {
    let evidence = &observation.evidence;
    if &evidence.attempt_id != attempt_id
        || &evidence.command_id != command_id
        || &evidence.connector_instance != connector
        || &evidence.request_commitment != request_commitment
    {
        return Err(TransportJournalError::ObservationIdentityMismatch);
    }
    if evidence.observed_at_ms < causal_floor_ms || evidence.observed_at_ms > now_ms {
        return Err(TransportJournalError::ObservationTimeMismatch);
    }
    Ok(())
}

fn observation_request_commitment(
    descriptor: &QualifiedHttpIssuanceDescriptor,
) -> ContentCommitment {
    // Secret-free convention for #676: identify the exact qualified issuance
    // descriptor, never hash final wire bytes containing Authorization/API keys.
    ContentCommitment::sha256(&descriptor.issuance_digest().0)
}

fn validate_dispatch_row(
    tx: &rusqlite::Transaction<'_>,
    dispatch: &QualifiedContextBoundDispatchStart,
    descriptor: &QualifiedHttpIssuanceDescriptor,
) -> Result<(), TransportJournalError> {
    let row: Option<(i64, String, Vec<u8>, Vec<u8>, i64, Vec<u8>)> = tx
        .query_row(
            "SELECT o.stage, d.context_profile, d.context_digest, d.binding_digest,\n\
                    d.command_commitment_algorithm, d.command_commitment_digest\n\
             FROM integration_outbox o\n\
             JOIN integration_dispatch_binding_v2 d ON d.entry_id = o.entry_id\n\
             WHERE d.entry_id = ?1 AND d.attempt_id = ?2",
            params![
                dispatch.dispatch().entry_id,
                dispatch.dispatch().attempt_id.as_str(),
            ],
            |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?, row.get(3)?, row.get(4)?, row.get(5)?)),
        )
        .optional()?;
    let Some((stage, context_profile, context_digest, binding_digest, algorithm, command_digest)) = row else {
        return Err(TransportJournalError::MissingContextDispatch);
    };
    if stage != DISPATCH_STARTED_STAGE
        || context_profile != dispatch.context().profile()
        || context_digest.as_slice() != dispatch.context().digest().0
        || binding_digest.as_slice() != dispatch.binding_digest().0
        || algorithm != digest_algorithm_code(descriptor.command_commitment().algorithm)
        || command_digest.as_slice() != descriptor.command_commitment().digest
    {
        return Err(TransportJournalError::ContextDispatchMismatch);
    }
    Ok(())
}

fn validate_journal_identity_prepared(
    tx: &rusqlite::Transaction<'_>,
    prepared: &PreparedHttpTransportAttempt,
) -> Result<(), TransportJournalError> {
    let row: Option<(i64, Vec<u8>, Vec<u8>, i64)> = tx
        .query_row(
            "SELECT stage, dispatch_binding_digest, issuance_digest, descriptor_valid_until_ms\n\
             FROM integration_http_transport_attempt_v1\n\
             WHERE entry_id = ?1 AND attempt_id = ?2",
            params![prepared.entry_id, prepared.attempt_id.as_str()],
            |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?, row.get(3)?)),
        )
        .optional()?;
    let Some((stage, dispatch_digest, issuance_digest, valid_until)) = row else {
        return Err(TransportJournalError::MissingJournal);
    };
    if stage != JOURNAL_PREPARED
        || dispatch_digest.as_slice() != prepared.dispatch_binding_digest.0
        || issuance_digest.as_slice() != prepared.issuance_digest.0
        || valid_until != i64::try_from(prepared.valid_until_ms).map_err(|_| TransportJournalError::TimeOverflow)?
    {
        return Err(TransportJournalError::JournalIdentityMismatch);
    }
    Ok(())
}

fn ensure_table(tx: &rusqlite::Transaction<'_>) -> Result<(), TransportJournalError> {
    tx.execute_batch(
        r#"
        CREATE TABLE IF NOT EXISTS integration_http_transport_attempt_v1 (
            entry_id INTEGER NOT NULL,
            attempt_id TEXT NOT NULL,
            command_id TEXT NOT NULL,
            connector_instance TEXT NOT NULL,
            dispatch_binding_digest BLOB NOT NULL,
            issuance_digest BLOB NOT NULL,
            request_commitment_algorithm INTEGER NOT NULL,
            request_commitment_digest BLOB NOT NULL,
            descriptor_valid_until_ms INTEGER NOT NULL,
            stage INTEGER NOT NULL,
            prepared_at_ms INTEGER NOT NULL,
            armed_at_ms INTEGER,
            observation_digest BLOB,
            observation_disposition INTEGER,
            updated_at_ms INTEGER NOT NULL,
            PRIMARY KEY(entry_id, attempt_id),
            FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
        );
        "#,
    )?;
    validate_table_schema(tx)
}

fn validate_table_schema(tx: &rusqlite::Transaction<'_>) -> Result<(), TransportJournalError> {
    const EXPECTED: &[(&str, &str, i64, i64)] = &[
        ("entry_id", "INTEGER", 1, 1),
        ("attempt_id", "TEXT", 1, 2),
        ("command_id", "TEXT", 1, 0),
        ("connector_instance", "TEXT", 1, 0),
        ("dispatch_binding_digest", "BLOB", 1, 0),
        ("issuance_digest", "BLOB", 1, 0),
        ("request_commitment_algorithm", "INTEGER", 1, 0),
        ("request_commitment_digest", "BLOB", 1, 0),
        ("descriptor_valid_until_ms", "INTEGER", 1, 0),
        ("stage", "INTEGER", 1, 0),
        ("prepared_at_ms", "INTEGER", 1, 0),
        ("armed_at_ms", "INTEGER", 0, 0),
        ("observation_digest", "BLOB", 0, 0),
        ("observation_disposition", "INTEGER", 0, 0),
        ("updated_at_ms", "INTEGER", 1, 0),
    ];
    let mut statement = tx.prepare("PRAGMA table_info(integration_http_transport_attempt_v1)")?;
    let actual = statement
        .query_map([], |row| {
            Ok((
                row.get::<_, String>(1)?,
                row.get::<_, String>(2)?,
                row.get::<_, i64>(3)?,
                row.get::<_, i64>(5)?,
            ))
        })?
        .collect::<Result<Vec<_>, _>>()?;
    if actual.len() != EXPECTED.len()
        || actual.iter().zip(EXPECTED).any(|(actual, expected)| {
            actual.0 != expected.0
                || actual.1.to_ascii_uppercase() != expected.1
                || actual.2 != expected.2
                || actual.3 != expected.3
        })
    {
        return Err(TransportJournalError::JournalSchemaMismatch);
    }
    let trigger_count: i64 = tx.query_row(
        "SELECT COUNT(*) FROM sqlite_master WHERE type = 'trigger' AND tbl_name = ?1",
        params![TABLE],
        |row| row.get(0),
    )?;
    if trigger_count != 0 {
        return Err(TransportJournalError::JournalSchemaMismatch);
    }
    Ok(())
}

fn open_exact_store(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<Connection, TransportJournalError> {
    require_exact_store_identity(path, expected_device, expected_inode)?;
    let conn = Connection::open_with_flags(
        path,
        OpenFlags::SQLITE_OPEN_READ_WRITE | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    require_exact_store_identity(path, expected_device, expected_inode)?;
    require_main_database_path(&conn, path)?;
    Ok(conn)
}

fn canonical_exact_store_path(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<PathBuf, TransportJournalError> {
    let link = fs::symlink_metadata(path).map_err(TransportJournalError::Io)?;
    if link.file_type().is_symlink() {
        return Err(TransportJournalError::SymlinkStorePath);
    }
    let canonical = fs::canonicalize(path).map_err(TransportJournalError::Io)?;
    require_exact_store_identity(&canonical, expected_device, expected_inode)?;
    Ok(canonical)
}

fn require_exact_store_identity(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<(), TransportJournalError> {
    let metadata = fs::metadata(path).map_err(TransportJournalError::Io)?;
    if !metadata.is_file()
        || metadata.dev() != expected_device
        || metadata.ino() != expected_inode
    {
        return Err(TransportJournalError::StoreIdentityChanged);
    }
    Ok(())
}

fn require_main_database_path(
    conn: &Connection,
    expected_path: &Path,
) -> Result<(), TransportJournalError> {
    let opened: String = conn.query_row(
        "SELECT file FROM pragma_database_list WHERE name = 'main'",
        [],
        |row| row.get(0),
    )?;
    if opened.is_empty() {
        return Err(TransportJournalError::StoreIdentityChanged);
    }
    let opened = fs::canonicalize(opened).map_err(TransportJournalError::Io)?;
    if opened != expected_path {
        return Err(TransportJournalError::StoreIdentityChanged);
    }
    Ok(())
}

fn disposition_code(disposition: TransportObservationDisposition) -> i64 {
    match disposition {
        TransportObservationDisposition::DefinitelyNotIssued => 1,
        TransportObservationDisposition::CompleteProviderResponse => 2,
        TransportObservationDisposition::AmbiguousPossibleIssue => 3,
    }
}

fn disposition_from_code(code: i64) -> Result<TransportObservationDisposition, TransportJournalError> {
    match code {
        1 => Ok(TransportObservationDisposition::DefinitelyNotIssued),
        2 => Ok(TransportObservationDisposition::CompleteProviderResponse),
        3 => Ok(TransportObservationDisposition::AmbiguousPossibleIssue),
        _ => Err(TransportJournalError::InvalidJournalState),
    }
}

fn digest_algorithm_code(value: DigestAlgorithm) -> i64 {
    match value {
        DigestAlgorithm::Sha256 => 1,
    }
}

fn to_u64_time(value: i64) -> Result<u64, TransportJournalError> {
    u64::try_from(value).map_err(|_| TransportJournalError::InvalidTime)
}

#[derive(Debug, Error)]
pub enum TransportJournalError {
    #[error("transport journal time is invalid")]
    InvalidTime,
    #[error("transport journal time cannot be represented")]
    TimeOverflow,
    #[error("runtime store path is a symlink")]
    SymlinkStorePath,
    #[error("runtime store identity changed")]
    StoreIdentityChanged,
    #[error("context-bound dispatch row is missing")]
    MissingContextDispatch,
    #[error("context-bound dispatch row differs from supplied proof/descriptor")]
    ContextDispatchMismatch,
    #[error("transport journal already exists for this attempt")]
    JournalAlreadyExists,
    #[error("transport journal row is missing")]
    MissingJournal,
    #[error("transport journal row differs from process-local capability")]
    JournalIdentityMismatch,
    #[error("transport journal changed concurrently")]
    ConcurrentJournalChange,
    #[error("transport journal schema differs from v1 structural contract")]
    JournalSchemaMismatch,
    #[error("transport journal is in an impossible/unknown state")]
    InvalidJournalState,
    #[error("pre-write observation is not conclusive zero-byte non-issuance")]
    PrewriteObservationNotNonissuance,
    #[error("transport observation does not match exact attempt/command/connector/request identity")]
    ObservationIdentityMismatch,
    #[error("transport observation time violates the journal causal frontier")]
    ObservationTimeMismatch,
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("runtime store filesystem error: {0}")]
    Io(std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disposition_storage_codes_are_stable() {
        assert_eq!(disposition_code(TransportObservationDisposition::DefinitelyNotIssued), 1);
        assert_eq!(
            disposition_code(TransportObservationDisposition::CompleteProviderResponse),
            2
        );
        assert_eq!(
            disposition_code(TransportObservationDisposition::AmbiguousPossibleIssue),
            3
        );
    }

    #[test]
    fn recovery_code_rejects_unknown_values() {
        assert!(matches!(
            disposition_from_code(9),
            Err(TransportJournalError::InvalidJournalState)
        ));
    }
}

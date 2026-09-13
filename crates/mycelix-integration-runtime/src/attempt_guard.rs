//! File-backed current-attempt qualification for INT-03.
//!
//! `ExecutionClaim` is transport/process data. This module re-reads the durable
//! outbox row owned by one `SqliteIntegrationStore` and returns a positive object
//! only when the complete prepared-attempt identity still matches at one exact
//! read instant.
//!
//! This is deliberately **not** atomic with `DispatchStarted`: another process
//! may change durable state after the read transaction closes. The eventual
//! effect-start theorem must therefore perform its own final store check/dispatch
//! transition inside the protected native critical section.

use mycelix_integration_core::{DigestAlgorithm, SideEffectClass};
use rusqlite::{Connection, OpenFlags, OptionalExtension, TransactionBehavior, params};
use std::fs;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};
use std::time::Duration;
use thiserror::Error;

use crate::ExecutionClaim;

pub const CURRENT_ATTEMPT_PROFILE: &str =
    "mycelix-integration-runtime-current-attempt-v1-blake3-framed";
const DOMAIN_CURRENT_ATTEMPT: &[u8] = b"mycelix/integration/runtime/current-attempt/v1";
const ATTEMPT_PREPARED_STAGE_V2: i64 = 4;
const MAX_WORKER_ID_BYTES: usize = 256;

/// Non-deserializable observation that one exact claim matched the exact durable
/// `AttemptPrepared` row in this file-backed runtime at `qualified_at_ms`.
#[derive(Clone, Debug)]
pub struct QualifiedCurrentExecutionAttempt {
    claim: ExecutionClaim,
    worker_id: String,
    store_device: u64,
    store_inode: u64,
    store_state_digest: [u8; 32],
    durable_updated_at_ms: i64,
    qualified_at_ms: i64,
}

impl QualifiedCurrentExecutionAttempt {
    pub fn claim(&self) -> &ExecutionClaim {
        &self.claim
    }

    pub fn worker_id(&self) -> &str {
        &self.worker_id
    }

    pub fn store_device(&self) -> u64 {
        self.store_device
    }

    pub fn store_inode(&self) -> u64 {
        self.store_inode
    }

    pub fn store_state_digest(&self) -> [u8; 32] {
        self.store_state_digest
    }

    pub fn durable_updated_at_ms(&self) -> i64 {
        self.durable_updated_at_ms
    }

    pub fn qualification_profile(&self) -> &'static str {
        CURRENT_ATTEMPT_PROFILE
    }

    pub fn qualified_at_ms(&self) -> i64 {
        self.qualified_at_ms
    }

    pub const fn file_backed_store_bound_here(&self) -> bool {
        true
    }

    pub const fn exact_prepared_attempt_observed_here(&self) -> bool {
        true
    }

    pub const fn lease_current_at_read_here(&self) -> bool {
        true
    }

    pub const fn quarantine_excluded_here(&self) -> bool {
        true
    }

    pub const fn causal_frontier_checked_here(&self) -> bool {
        true
    }

    /// This observation is intentionally not a mutex/transaction that survives
    /// the function return.
    pub const fn atomic_with_dispatch_started_here(&self) -> bool {
        false
    }

    pub const fn coordinator_update_excluded_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug)]
struct DurableAttemptRow {
    stage: i64,
    worker_id: Option<String>,
    lease_until_ms: Option<i64>,
    attempt_count: i64,
    current_attempt_id: Option<String>,
    command_id: String,
    connector_instance: String,
    command_commitment_algorithm: i64,
    command_commitment_digest: Vec<u8>,
    side_effect_class: i64,
    idempotency_key: Option<String>,
    updated_at_ms: i64,
}

pub(crate) fn qualify_file_current_attempt(
    path: &Path,
    claim: &ExecutionClaim,
    worker_id: &str,
    now_ms: i64,
) -> Result<QualifiedCurrentExecutionAttempt, CurrentAttemptQualificationError> {
    validate_inputs(claim, worker_id, now_ms)?;

    let canonical_before = canonical_store_path(path)?;
    let before = fs::metadata(&canonical_before).map_err(CurrentAttemptQualificationError::Io)?;
    if !before.is_file() {
        return Err(CurrentAttemptQualificationError::StoreFileIdentityChanged);
    }

    let mut conn =
        Connection::open_with_flags(&canonical_before, OpenFlags::SQLITE_OPEN_READ_ONLY)?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON; PRAGMA query_only = ON;")?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Deferred)?;

    if entry_is_quarantined(&tx, claim.entry_id)? {
        return Err(CurrentAttemptQualificationError::QuarantinedEntry {
            entry_id: claim.entry_id,
        });
    }

    let row = tx
        .query_row(
            "SELECT stage, worker_id, lease_until_ms, attempt_count, current_attempt_id,\n\
                    command_id, connector_instance,\n\
                    command_commitment_algorithm, command_commitment_digest,\n\
                    side_effect_class, idempotency_key, updated_at_ms\n\
             FROM integration_outbox WHERE entry_id = ?1",
            params![claim.entry_id],
            |row| {
                Ok(DurableAttemptRow {
                    stage: row.get(0)?,
                    worker_id: row.get(1)?,
                    lease_until_ms: row.get(2)?,
                    attempt_count: row.get(3)?,
                    current_attempt_id: row.get(4)?,
                    command_id: row.get(5)?,
                    connector_instance: row.get(6)?,
                    command_commitment_algorithm: row.get(7)?,
                    command_commitment_digest: row.get(8)?,
                    side_effect_class: row.get(9)?,
                    idempotency_key: row.get(10)?,
                    updated_at_ms: row.get(11)?,
                })
            },
        )
        .optional()?
        .ok_or(CurrentAttemptQualificationError::UnknownEntry {
            entry_id: claim.entry_id,
        })?;

    validate_row(&row, claim, worker_id, now_ms)?;
    let state_digest = durable_state_digest(claim, worker_id, row.updated_at_ms);
    tx.commit()?;

    let canonical_after = canonical_store_path(path)?;
    let after = fs::metadata(&canonical_after).map_err(CurrentAttemptQualificationError::Io)?;
    if canonical_before != canonical_after
        || before.dev() != after.dev()
        || before.ino() != after.ino()
    {
        return Err(CurrentAttemptQualificationError::StoreFileIdentityChanged);
    }

    Ok(QualifiedCurrentExecutionAttempt {
        claim: claim.clone(),
        worker_id: worker_id.to_owned(),
        store_device: after.dev(),
        store_inode: after.ino(),
        store_state_digest: state_digest,
        durable_updated_at_ms: row.updated_at_ms,
        qualified_at_ms: now_ms,
    })
}

fn canonical_store_path(path: &Path) -> Result<PathBuf, CurrentAttemptQualificationError> {
    fs::canonicalize(path).map_err(CurrentAttemptQualificationError::Io)
}

fn entry_is_quarantined(
    tx: &rusqlite::Transaction<'_>,
    entry_id: i64,
) -> Result<bool, CurrentAttemptQualificationError> {
    let table_exists: bool = tx.query_row(
        "SELECT EXISTS(\n\
             SELECT 1 FROM sqlite_master\n\
             WHERE type = 'table' AND name = 'integration_runtime_quarantine'\n\
         )",
        [],
        |row| row.get(0),
    )?;
    if !table_exists {
        return Ok(false);
    }
    let quarantined: bool = tx.query_row(
        "SELECT EXISTS(\n\
             SELECT 1 FROM integration_runtime_quarantine WHERE entry_id = ?1\n\
         )",
        params![entry_id],
        |row| row.get(0),
    )?;
    Ok(quarantined)
}

fn validate_inputs(
    claim: &ExecutionClaim,
    worker_id: &str,
    now_ms: i64,
) -> Result<(), CurrentAttemptQualificationError> {
    if claim.entry_id <= 0 || claim.attempt_count == 0 {
        return Err(CurrentAttemptQualificationError::InvalidClaim);
    }
    if worker_id.is_empty()
        || worker_id.len() > MAX_WORKER_ID_BYTES
        || worker_id.chars().any(char::is_control)
    {
        return Err(CurrentAttemptQualificationError::InvalidWorkerId);
    }
    if now_ms < 0 {
        return Err(CurrentAttemptQualificationError::InvalidTime);
    }
    Ok(())
}

fn validate_row(
    row: &DurableAttemptRow,
    claim: &ExecutionClaim,
    worker_id: &str,
    now_ms: i64,
) -> Result<(), CurrentAttemptQualificationError> {
    if row.stage != ATTEMPT_PREPARED_STAGE_V2 {
        return Err(CurrentAttemptQualificationError::AttemptNotPrepared);
    }
    if row.worker_id.as_deref() != Some(worker_id) {
        return Err(CurrentAttemptQualificationError::WorkerMismatch);
    }
    if row.current_attempt_id.as_deref() != Some(claim.attempt_id.as_str()) {
        return Err(CurrentAttemptQualificationError::AttemptIdentityMismatch);
    }
    if row.lease_until_ms != Some(claim.lease_until_ms) || claim.lease_until_ms <= now_ms {
        return Err(CurrentAttemptQualificationError::LeaseMismatchOrExpired);
    }
    if row.attempt_count != i64::from(claim.attempt_count) {
        return Err(CurrentAttemptQualificationError::AttemptCountMismatch);
    }
    if row.command_id != claim.command_id.as_str() {
        return Err(CurrentAttemptQualificationError::CommandIdMismatch);
    }
    if row.connector_instance != claim.connector_instance.as_str() {
        return Err(CurrentAttemptQualificationError::ConnectorMismatch);
    }
    if row.command_commitment_algorithm != digest_algorithm_code(claim.command_commitment.algorithm)
        || row.command_commitment_digest.as_slice() != claim.command_commitment.digest.as_slice()
    {
        return Err(CurrentAttemptQualificationError::CommandCommitmentMismatch);
    }
    if row.side_effect_class != side_effect_code(claim.side_effect_class) {
        return Err(CurrentAttemptQualificationError::SideEffectMismatch);
    }
    if row.idempotency_key.as_deref() != claim.idempotency_key.as_ref().map(|value| value.as_str())
    {
        return Err(CurrentAttemptQualificationError::IdempotencyKeyMismatch);
    }
    if row.updated_at_ms > now_ms {
        return Err(CurrentAttemptQualificationError::CausalTimeRegression {
            durable_updated_at_ms: row.updated_at_ms,
            observed_at_ms: now_ms,
        });
    }
    Ok(())
}

fn durable_state_digest(
    claim: &ExecutionClaim,
    worker_id: &str,
    durable_updated_at_ms: i64,
) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_CURRENT_ATTEMPT);
    frame(&mut h, CURRENT_ATTEMPT_PROFILE.as_bytes());
    frame(&mut h, &claim.entry_id.to_le_bytes());
    frame(&mut h, claim.attempt_id.as_str().as_bytes());
    frame(&mut h, claim.command_id.as_str().as_bytes());
    frame(&mut h, claim.connector_instance.as_str().as_bytes());
    frame(
        &mut h,
        &[digest_algorithm_code(claim.command_commitment.algorithm) as u8],
    );
    frame(&mut h, &claim.command_commitment.digest);
    frame(&mut h, &[side_effect_code(claim.side_effect_class) as u8]);
    match &claim.idempotency_key {
        Some(value) => {
            frame(&mut h, &[1]);
            frame(&mut h, value.as_str().as_bytes());
        }
        None => frame(&mut h, &[0]),
    }
    frame(&mut h, &claim.attempt_count.to_le_bytes());
    frame(&mut h, &claim.lease_until_ms.to_le_bytes());
    frame(&mut h, worker_id.as_bytes());
    frame(&mut h, &durable_updated_at_ms.to_le_bytes());
    *h.finalize().as_bytes()
}

fn digest_algorithm_code(value: DigestAlgorithm) -> i64 {
    match value {
        DigestAlgorithm::Sha256 => 1,
    }
}

fn side_effect_code(value: SideEffectClass) -> i64 {
    match value {
        SideEffectClass::ReadOnly => 0,
        SideEffectClass::Reversible => 1,
        SideEffectClass::Compensatable => 2,
        SideEffectClass::Irreversible => 3,
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum CurrentAttemptQualificationError {
    #[error("current-attempt qualification requires a file-backed integration runtime")]
    FileBackedStoreRequired,
    #[error("current-attempt qualification requires a valid durable claim")]
    InvalidClaim,
    #[error("current-attempt qualification requires a valid worker id")]
    InvalidWorkerId,
    #[error("current-attempt qualification time is invalid")]
    InvalidTime,
    #[error("outbox entry {entry_id} does not exist")]
    UnknownEntry { entry_id: i64 },
    #[error("outbox entry {entry_id} is quarantined")]
    QuarantinedEntry { entry_id: i64 },
    #[error("durable outbox row is not in AttemptPrepared")]
    AttemptNotPrepared,
    #[error("durable prepared attempt belongs to a different worker")]
    WorkerMismatch,
    #[error("durable current attempt id differs from the claim")]
    AttemptIdentityMismatch,
    #[error("durable lease differs from the claim or is expired")]
    LeaseMismatchOrExpired,
    #[error("durable attempt count differs from the claim")]
    AttemptCountMismatch,
    #[error("durable command id differs from the claim")]
    CommandIdMismatch,
    #[error("durable connector differs from the claim")]
    ConnectorMismatch,
    #[error("durable command commitment differs from the claim")]
    CommandCommitmentMismatch,
    #[error("durable side-effect class differs from the claim")]
    SideEffectMismatch,
    #[error("durable idempotency key differs from the claim")]
    IdempotencyKeyMismatch,
    #[error(
        "current-attempt observation time {observed_at_ms} predates durable frontier {durable_updated_at_ms}"
    )]
    CausalTimeRegression {
        durable_updated_at_ms: i64,
        observed_at_ms: i64,
    },
    #[error("runtime store file identity changed during qualification")]
    StoreFileIdentityChanged,
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("runtime store filesystem error: {0}")]
    Io(std::io::Error),
}

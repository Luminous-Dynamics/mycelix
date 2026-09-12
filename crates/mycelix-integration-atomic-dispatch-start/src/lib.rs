//! Atomic binding of one exact materialized request to one exact durable
//! `AttemptPrepared -> DispatchStarted` transition.
//!
//! This crate still does not perform a provider call. It closes the local crash
//! evidence gap by committing the exact materialization identity in the same
//! SQLite transaction that crosses `DispatchStarted`.

#[cfg(not(unix))]
compile_error!("atomic integration dispatch binding v0.1 requires Unix file identity semantics");

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{
    ContentCommitment, DigestAlgorithm, ExternalOperationRef, SideEffectClass,
};
use mycelix_integration_materializer_runtime::MaterializedProviderRequest;
use mycelix_integration_runtime::{
    DispatchStarted, QualifiedCurrentExecutionAttempt,
};
use rusqlite::{params, Connection, OpenFlags, OptionalExtension, TransactionBehavior};
use std::fs;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};
use std::time::Duration;
use thiserror::Error;

pub const DISPATCH_BINDING_PROFILE: &str =
    "mycelix-integration-atomic-dispatch-start-v1-blake3-framed";
const DOMAIN_BINDING: &[u8] = b"mycelix/integration/atomic-dispatch-start/v1";
const ATTEMPT_PREPARED_STAGE_V2: i64 = 4;
const DISPATCH_STARTED_STAGE_V2: i64 = 5;
const TABLE_NAME: &str = "integration_dispatch_binding_v1";

/// Non-deserializable proof that the exact materialization commitment and exact
/// current attempt were committed atomically with `DispatchStarted`.
#[derive(Debug)]
pub struct QualifiedAtomicDispatchStart {
    dispatch: DispatchStarted,
    materialization_digest: Digest32,
    output_commitment: ContentCommitment,
    binding_digest: Digest32,
    store_device: u64,
    store_inode: u64,
}

impl QualifiedAtomicDispatchStart {
    pub fn dispatch(&self) -> &DispatchStarted {
        &self.dispatch
    }

    pub fn materialization_digest(&self) -> Digest32 {
        self.materialization_digest
    }

    pub fn output_commitment(&self) -> &ContentCommitment {
        &self.output_commitment
    }

    pub fn binding_digest(&self) -> Digest32 {
        self.binding_digest
    }

    pub fn store_device(&self) -> u64 {
        self.store_device
    }

    pub fn store_inode(&self) -> u64 {
        self.store_inode
    }

    pub const fn exact_attempt_rechecked_in_transaction_here(&self) -> bool {
        true
    }

    pub const fn materialization_bound_in_same_transaction_here(&self) -> bool {
        true
    }

    pub const fn dispatch_started_durably_here(&self) -> bool {
        true
    }

    pub const fn provider_call_started_here(&self) -> bool {
        false
    }

    pub const fn provider_commit_confirmed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn bind_materialization_and_start_dispatch(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> Result<QualifiedAtomicDispatchStart, AtomicDispatchStartError> {
    if now_ms < 0 || now_ms < current_attempt.qualified_at_ms() {
        return Err(AtomicDispatchStartError::InvalidTime);
    }

    let claim = current_attempt.claim();
    if claim.lease_until_ms <= now_ms {
        return Err(AtomicDispatchStartError::LeaseExpired);
    }
    validate_materialization_identity(current_attempt, materialized)?;

    let canonical_path = canonical_exact_store_path(
        store_path.as_ref(),
        current_attempt.store_device(),
        current_attempt.store_inode(),
    )?;

    let mut conn = Connection::open_with_flags(
        &canonical_path,
        OpenFlags::SQLITE_OPEN_READ_WRITE | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;

    ensure_dispatch_binding_table(&tx)?;
    reject_quarantined_attempt(&tx, claim.entry_id)?;

    #[allow(clippy::type_complexity)]
    let row: (
        i64,
        Option<String>,
        Option<i64>,
        i64,
        Option<String>,
        String,
        String,
        i64,
        Vec<u8>,
        i64,
        Option<String>,
        i64,
    ) = tx
        .query_row(
            "SELECT stage, worker_id, lease_until_ms, attempt_count, current_attempt_id,\n\
                    command_id, connector_instance, command_commitment_algorithm,\n\
                    command_commitment_digest, side_effect_class, idempotency_key, updated_at_ms\n\
             FROM integration_outbox WHERE entry_id = ?1",
            params![claim.entry_id],
            |row| {
                Ok((
                    row.get(0)?,
                    row.get(1)?,
                    row.get(2)?,
                    row.get(3)?,
                    row.get(4)?,
                    row.get(5)?,
                    row.get(6)?,
                    row.get(7)?,
                    row.get(8)?,
                    row.get(9)?,
                    row.get(10)?,
                    row.get(11)?,
                ))
            },
        )
        .optional()?
        .ok_or(AtomicDispatchStartError::UnknownEntry)?;

    validate_durable_row(current_attempt, &row, now_ms)?;

    let existing: Option<Vec<u8>> = tx
        .query_row(
            "SELECT materialization_digest FROM integration_dispatch_binding_v1\n\
             WHERE entry_id = ?1 AND attempt_id = ?2",
            params![claim.entry_id, claim.attempt_id.as_str()],
            |row| row.get(0),
        )
        .optional()?;
    if existing.is_some() {
        return Err(AtomicDispatchStartError::DispatchBindingAlreadyExists);
    }

    tx.execute(
        "INSERT INTO integration_dispatch_binding_v1 (\n\
            entry_id, attempt_id, command_id, connector_instance,\n\
            command_commitment_algorithm, command_commitment_digest,\n\
            materialization_digest, output_commitment_algorithm, output_commitment_digest,\n\
            admission_digest, determinism_digest,\n\
            provider_profile_algorithm, provider_profile_digest,\n\
            provider_root_algorithm, provider_root_digest,\n\
            materializer_release_algorithm, materializer_release_digest,\n\
            worker_id, lease_until_ms, started_at_ms\n\
         ) VALUES (\n\
            ?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10,\n\
            ?11, ?12, ?13, ?14, ?15, ?16, ?17, ?18, ?19, ?20\n\
         )",
        params![
            claim.entry_id,
            claim.attempt_id.as_str(),
            claim.command_id.as_str(),
            claim.connector_instance.as_str(),
            digest_algorithm_code(claim.command_commitment.algorithm),
            claim.command_commitment.digest.as_slice(),
            materialized.materialization_digest().0.as_slice(),
            digest_algorithm_code(materialized.output_commitment().algorithm),
            materialized.output_commitment().digest.as_slice(),
            materialized.admission_digest().0.as_slice(),
            materialized.determinism_digest().0.as_slice(),
            digest_algorithm_code(materialized.provider_profile_commitment().algorithm),
            materialized.provider_profile_commitment().digest.as_slice(),
            digest_algorithm_code(materialized.provider_trust_root_commitment().algorithm),
            materialized.provider_trust_root_commitment().digest.as_slice(),
            digest_algorithm_code(materialized.materializer_release().algorithm),
            materialized.materializer_release().digest.as_slice(),
            current_attempt.worker_id(),
            claim.lease_until_ms,
            now_ms,
        ],
    )?;

    let changed = tx.execute(
        "UPDATE integration_outbox\n\
         SET stage = ?1, dispatch_started_at_ms = ?2, updated_at_ms = ?2\n\
         WHERE entry_id = ?3 AND stage = ?4 AND current_attempt_id = ?5",
        params![
            DISPATCH_STARTED_STAGE_V2,
            now_ms,
            claim.entry_id,
            ATTEMPT_PREPARED_STAGE_V2,
            claim.attempt_id.as_str(),
        ],
    )?;
    if changed != 1 {
        return Err(AtomicDispatchStartError::ConcurrentAttemptChange);
    }

    tx.commit()?;

    let after = fs::metadata(&canonical_path).map_err(AtomicDispatchStartError::Io)?;
    if after.dev() != current_attempt.store_device()
        || after.ino() != current_attempt.store_inode()
    {
        return Err(AtomicDispatchStartError::StoreIdentityChanged);
    }

    let dispatch = DispatchStarted {
        entry_id: claim.entry_id,
        attempt_id: claim.attempt_id.clone(),
        operation: ExternalOperationRef {
            command_id: claim.command_id.clone(),
            connector_instance: claim.connector_instance.clone(),
            provider_operation: None,
        },
        started_at_ms: now_ms,
    };
    let binding_digest = dispatch_binding_digest(
        current_attempt,
        materialized,
        now_ms,
        after.dev(),
        after.ino(),
    );

    Ok(QualifiedAtomicDispatchStart {
        dispatch,
        materialization_digest: materialized.materialization_digest(),
        output_commitment: materialized.output_commitment().clone(),
        binding_digest,
        store_device: after.dev(),
        store_inode: after.ino(),
    })
}

fn validate_materialization_identity(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
) -> Result<(), AtomicDispatchStartError> {
    let claim = current_attempt.claim();
    if materialized.entry_id() != claim.entry_id
        || materialized.attempt_id() != &claim.attempt_id
        || materialized.command_id() != &claim.command_id
        || materialized.connector_instance() != &claim.connector_instance
        || materialized.input_commitment() != &claim.command_commitment
    {
        return Err(AtomicDispatchStartError::MaterializationAttemptMismatch);
    }
    Ok(())
}

#[allow(clippy::type_complexity)]
fn validate_durable_row(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    row: &(
        i64,
        Option<String>,
        Option<i64>,
        i64,
        Option<String>,
        String,
        String,
        i64,
        Vec<u8>,
        i64,
        Option<String>,
        i64,
    ),
    now_ms: i64,
) -> Result<(), AtomicDispatchStartError> {
    let claim = current_attempt.claim();
    if row.0 != ATTEMPT_PREPARED_STAGE_V2 {
        return Err(AtomicDispatchStartError::AttemptNotPrepared);
    }
    if row.1.as_deref() != Some(current_attempt.worker_id()) {
        return Err(AtomicDispatchStartError::WorkerMismatch);
    }
    if row.2 != Some(claim.lease_until_ms) || claim.lease_until_ms <= now_ms {
        return Err(AtomicDispatchStartError::LeaseExpired);
    }
    if row.3 != i64::from(claim.attempt_count) {
        return Err(AtomicDispatchStartError::AttemptCountMismatch);
    }
    if row.4.as_deref() != Some(claim.attempt_id.as_str()) {
        return Err(AtomicDispatchStartError::AttemptIdentityMismatch);
    }
    if row.5 != claim.command_id.as_str() || row.6 != claim.connector_instance.as_str() {
        return Err(AtomicDispatchStartError::CommandSubjectMismatch);
    }
    if row.7 != digest_algorithm_code(claim.command_commitment.algorithm)
        || row.8.as_slice() != claim.command_commitment.digest
    {
        return Err(AtomicDispatchStartError::CommandCommitmentMismatch);
    }
    if row.9 != side_effect_code(claim.side_effect_class) {
        return Err(AtomicDispatchStartError::SideEffectMismatch);
    }
    if row.10.as_deref() != claim.idempotency_key.as_ref().map(|value| value.as_str()) {
        return Err(AtomicDispatchStartError::IdempotencyMismatch);
    }
    if now_ms < row.11 {
        return Err(AtomicDispatchStartError::CausalTimeRegression);
    }
    Ok(())
}

fn canonical_exact_store_path(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<PathBuf, AtomicDispatchStartError> {
    let link_meta = fs::symlink_metadata(path).map_err(AtomicDispatchStartError::Io)?;
    if link_meta.file_type().is_symlink() {
        return Err(AtomicDispatchStartError::SymlinkStorePath);
    }
    let canonical = fs::canonicalize(path).map_err(AtomicDispatchStartError::Io)?;
    let metadata = fs::metadata(&canonical).map_err(AtomicDispatchStartError::Io)?;
    if !metadata.is_file() || metadata.dev() != expected_device || metadata.ino() != expected_inode {
        return Err(AtomicDispatchStartError::StoreIdentityChanged);
    }
    Ok(canonical)
}

fn reject_quarantined_attempt(
    tx: &rusqlite::Transaction<'_>,
    entry_id: i64,
) -> Result<(), AtomicDispatchStartError> {
    let quarantine_exists: bool = tx.query_row(
        "SELECT EXISTS(SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = 'integration_runtime_quarantine')",
        [],
        |row| row.get(0),
    )?;
    if !quarantine_exists {
        return Ok(());
    }
    let quarantined: bool = tx.query_row(
        "SELECT EXISTS(SELECT 1 FROM integration_runtime_quarantine WHERE entry_id = ?1)",
        params![entry_id],
        |row| row.get(0),
    )?;
    if quarantined {
        Err(AtomicDispatchStartError::AttemptQuarantined)
    } else {
        Ok(())
    }
}

fn ensure_dispatch_binding_table(
    tx: &rusqlite::Transaction<'_>,
) -> Result<(), AtomicDispatchStartError> {
    tx.execute_batch(
        r#"
        CREATE TABLE IF NOT EXISTS integration_dispatch_binding_v1 (
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
            PRIMARY KEY(entry_id, attempt_id),
            FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
        );
        "#,
    )?;

    let expected = [
        "entry_id",
        "attempt_id",
        "command_id",
        "connector_instance",
        "command_commitment_algorithm",
        "command_commitment_digest",
        "materialization_digest",
        "output_commitment_algorithm",
        "output_commitment_digest",
        "admission_digest",
        "determinism_digest",
        "provider_profile_algorithm",
        "provider_profile_digest",
        "provider_root_algorithm",
        "provider_root_digest",
        "materializer_release_algorithm",
        "materializer_release_digest",
        "worker_id",
        "lease_until_ms",
        "started_at_ms",
    ];
    let mut statement = tx.prepare("PRAGMA table_info(integration_dispatch_binding_v1)")?;
    let actual = statement
        .query_map([], |row| row.get::<_, String>(1))?
        .collect::<Result<Vec<_>, _>>()?;
    if actual != expected {
        return Err(AtomicDispatchStartError::DispatchBindingSchemaMismatch);
    }
    Ok(())
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

fn dispatch_binding_digest(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
    started_at_ms: i64,
    store_device: u64,
    store_inode: u64,
) -> Digest32 {
    let claim = current_attempt.claim();
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_BINDING);
    frame(&mut h, DISPATCH_BINDING_PROFILE.as_bytes());
    frame(&mut h, &claim.entry_id.to_le_bytes());
    frame(&mut h, claim.attempt_id.as_str().as_bytes());
    frame(&mut h, &materialized.materialization_digest().0);
    frame(&mut h, &materialized.output_commitment().digest);
    frame(&mut h, &started_at_ms.to_le_bytes());
    frame(&mut h, &store_device.to_le_bytes());
    frame(&mut h, &store_inode.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum AtomicDispatchStartError {
    #[error("dispatch start time is invalid")]
    InvalidTime,
    #[error("durable attempt lease expired before dispatch start")]
    LeaseExpired,
    #[error("materialization does not belong to the exact current durable attempt")]
    MaterializationAttemptMismatch,
    #[error("runtime store path is a symlink")]
    SymlinkStorePath,
    #[error("runtime store identity changed")]
    StoreIdentityChanged,
    #[error("outbox entry does not exist")]
    UnknownEntry,
    #[error("durable attempt is not in AttemptPrepared")]
    AttemptNotPrepared,
    #[error("durable attempt belongs to a different worker")]
    WorkerMismatch,
    #[error("durable attempt count changed")]
    AttemptCountMismatch,
    #[error("durable attempt fence changed")]
    AttemptIdentityMismatch,
    #[error("durable command or connector changed")]
    CommandSubjectMismatch,
    #[error("durable command commitment changed")]
    CommandCommitmentMismatch,
    #[error("durable side-effect class changed")]
    SideEffectMismatch,
    #[error("durable idempotency identity changed")]
    IdempotencyMismatch,
    #[error("dispatch start time is behind the durable causal frontier")]
    CausalTimeRegression,
    #[error("durable attempt is quarantined")]
    AttemptQuarantined,
    #[error("dispatch binding for this attempt already exists")]
    DispatchBindingAlreadyExists,
    #[error("dispatch binding table differs from the v1 structural contract")]
    DispatchBindingSchemaMismatch,
    #[error("attempt changed while crossing DispatchStarted")]
    ConcurrentAttemptChange,
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("runtime store filesystem error: {0}")]
    Io(std::io::Error),
}

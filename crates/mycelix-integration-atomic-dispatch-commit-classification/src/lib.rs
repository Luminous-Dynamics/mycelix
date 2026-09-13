//! Fail-closed commit-outcome classification for context-bound dispatch start.
//!
//! The mature atomic dispatcher remains the only writer. This crate wraps that
//! operation and, after any returned error, independently reopens the exact
//! provisioned SQLite file and compares the complete durable predecessor and
//! successor identities. No error message, local autocommit bit, or source-order
//! assumption controls retry semantics.
//!
//! A successor observed after the original call returned an error is represented
//! as `RecoveredCommittedContextDispatch`. It intentionally does not recreate the
//! process-local `QualifiedContextBoundDispatchStart`; callers must enter recovery.

#[cfg(not(unix))]
compile_error!("classified atomic integration dispatch requires Unix file identity semantics");

use mycelix_institutional_core::Digest32;
use mycelix_integration_atomic_dispatch_context::{
    DISPATCH_CONTEXT_PROFILE, ContextDispatchError, DispatchContextCommitment,
    QualifiedContextBoundDispatchStart, bind_context_and_start_dispatch,
};
use mycelix_integration_core::{ContentCommitment, DigestAlgorithm, SideEffectClass};
use mycelix_integration_materializer_runtime::MaterializedProviderRequest;
use mycelix_integration_runtime::QualifiedCurrentExecutionAttempt;
use mycelix_integration_sqlite_commit_classification::{
    CommitClassification, classify_exact_durable_transition,
};
use rusqlite::{Connection, OpenFlags, OptionalExtension, params};
use std::fs;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};
use std::time::Duration;
use thiserror::Error;

pub const CLASSIFIED_CONTEXT_DISPATCH_PROFILE: &str =
    "mycelix-integration-atomic-dispatch-commit-classification-v1";

const DOMAIN_BINDING: &[u8] = b"mycelix/integration/atomic-dispatch-context/v1";
const ATTEMPT_PREPARED_STAGE_V2: i64 = 4;
const DISPATCH_STARTED_STAGE_V2: i64 = 5;
const DISPATCH_TABLE: &str = "integration_dispatch_binding_v2";

/// Result of attempting the exact atomic dispatch transition with fail-closed
/// post-error durable classification.
pub enum ClassifiedContextDispatchOutcome {
    /// The original atomic dispatcher returned success and retained its live
    /// process-local token.
    Committed(QualifiedContextBoundDispatchStart),
    /// The original call returned an error, but an independent exact reread
    /// proves the intended successor is durable. Recovery is required because
    /// the process-local live token did not escape the failed call.
    RecoveredCommitted(RecoveredCommittedContextDispatch),
    /// The original call returned an error and the complete exact predecessor is
    /// still durable. This does not itself authorize retry.
    DefinitelyNotCommitted(DefinitelyNotCommittedContextDispatch),
    /// Neither exact state could be positively proved, or the independent reread
    /// itself failed. No normal retry is permitted by this result.
    IndeterminateCommit(IndeterminateContextDispatch),
}

impl ClassifiedContextDispatchOutcome {
    pub const fn classification(&self) -> CommitClassification {
        match self {
            Self::Committed(_) | Self::RecoveredCommitted(_) => CommitClassification::Committed,
            Self::DefinitelyNotCommitted(_) => CommitClassification::DefinitelyNotCommitted,
            Self::IndeterminateCommit(_) => CommitClassification::IndeterminateCommit,
        }
    }

    pub const fn live_dispatch_token_available_here(&self) -> bool {
        matches!(self, Self::Committed(_))
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub struct RecoveredCommittedContextDispatch {
    entry_id: i64,
    attempt_id: String,
    binding_digest: Digest32,
    original_error: ContextDispatchError,
}

impl RecoveredCommittedContextDispatch {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn binding_digest(&self) -> Digest32 {
        self.binding_digest
    }

    pub fn original_error(&self) -> &ContextDispatchError {
        &self.original_error
    }

    pub const fn exact_successor_reobserved_here(&self) -> bool {
        true
    }

    pub const fn live_dispatch_token_reconstructed_here(&self) -> bool {
        false
    }

    pub const fn recovery_only_here(&self) -> bool {
        true
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub struct DefinitelyNotCommittedContextDispatch {
    entry_id: i64,
    attempt_id: String,
    original_error: ContextDispatchError,
}

impl DefinitelyNotCommittedContextDispatch {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn original_error(&self) -> &ContextDispatchError {
        &self.original_error
    }

    pub const fn exact_predecessor_reobserved_here(&self) -> bool {
        true
    }

    /// Retry remains an enclosing-policy decision; this object never authorizes it.
    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub struct IndeterminateContextDispatch {
    entry_id: i64,
    attempt_id: String,
    original_error: ContextDispatchError,
    reread_error: Option<DurableDispatchRereadError>,
}

impl IndeterminateContextDispatch {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn original_error(&self) -> &ContextDispatchError {
        &self.original_error
    }

    pub fn reread_error(&self) -> Option<&DurableDispatchRereadError> {
        self.reread_error.as_ref()
    }

    pub const fn recovery_only_here(&self) -> bool {
        true
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug)]
struct DurableObservation {
    successor_exact: bool,
    predecessor_exact: bool,
}

#[derive(Debug)]
struct DurableOutboxRow {
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
    dispatch_started_at_ms: Option<i64>,
}

#[derive(Debug)]
struct DurableBindingRow {
    entry_id: i64,
    attempt_id: String,
    command_id: String,
    connector_instance: String,
    command_commitment_algorithm: i64,
    command_commitment_digest: Vec<u8>,
    materialization_digest: Vec<u8>,
    output_commitment_algorithm: i64,
    output_commitment_digest: Vec<u8>,
    admission_digest: Vec<u8>,
    determinism_digest: Vec<u8>,
    provider_profile_algorithm: i64,
    provider_profile_digest: Vec<u8>,
    provider_root_algorithm: i64,
    provider_root_digest: Vec<u8>,
    materializer_release_algorithm: i64,
    materializer_release_digest: Vec<u8>,
    context_profile: String,
    context_digest: Vec<u8>,
    worker_id: String,
    lease_until_ms: i64,
    started_at_ms: i64,
    binding_digest: Vec<u8>,
}

/// Attempt one exact context-bound dispatch and classify any returned error by
/// independently re-reading the exact durable predecessor/successor state.
///
/// This function never interprets a commit error message and never turns a
/// persistence classification into retry or execution authority.
pub fn bind_context_and_start_dispatch_classified(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
    context: &DispatchContextCommitment,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> ClassifiedContextDispatchOutcome {
    let store_path = store_path.as_ref();
    let expected_binding_digest = dispatch_binding_digest(
        current_attempt,
        materialized,
        context,
        now_ms,
        current_attempt.store_device(),
        current_attempt.store_inode(),
    );

    match bind_context_and_start_dispatch(
        current_attempt,
        materialized,
        context,
        store_path,
        now_ms,
    ) {
        Ok(dispatch) => ClassifiedContextDispatchOutcome::Committed(dispatch),
        Err(original_error) => {
            let claim = current_attempt.claim();
            match observe_durable_dispatch_state(
                current_attempt,
                materialized,
                context,
                store_path,
                now_ms,
                expected_binding_digest,
            ) {
                Ok(observation) => match classify_exact_durable_transition(
                    observation.successor_exact,
                    observation.predecessor_exact,
                ) {
                    CommitClassification::Committed => {
                        ClassifiedContextDispatchOutcome::RecoveredCommitted(
                            RecoveredCommittedContextDispatch {
                                entry_id: claim.entry_id,
                                attempt_id: claim.attempt_id.as_str().to_owned(),
                                binding_digest: expected_binding_digest,
                                original_error,
                            },
                        )
                    }
                    CommitClassification::DefinitelyNotCommitted => {
                        ClassifiedContextDispatchOutcome::DefinitelyNotCommitted(
                            DefinitelyNotCommittedContextDispatch {
                                entry_id: claim.entry_id,
                                attempt_id: claim.attempt_id.as_str().to_owned(),
                                original_error,
                            },
                        )
                    }
                    CommitClassification::IndeterminateCommit => {
                        ClassifiedContextDispatchOutcome::IndeterminateCommit(
                            IndeterminateContextDispatch {
                                entry_id: claim.entry_id,
                                attempt_id: claim.attempt_id.as_str().to_owned(),
                                original_error,
                                reread_error: None,
                            },
                        )
                    }
                },
                Err(reread_error) => ClassifiedContextDispatchOutcome::IndeterminateCommit(
                    IndeterminateContextDispatch {
                        entry_id: claim.entry_id,
                        attempt_id: claim.attempt_id.as_str().to_owned(),
                        original_error,
                        reread_error: Some(reread_error),
                    },
                ),
            }
        }
    }
}

fn observe_durable_dispatch_state(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
    context: &DispatchContextCommitment,
    store_path: &Path,
    now_ms: i64,
    expected_binding_digest: Digest32,
) -> Result<DurableObservation, DurableDispatchRereadError> {
    let store_device = current_attempt.store_device();
    let store_inode = current_attempt.store_inode();
    let canonical = canonical_exact_store_path(store_path, store_device, store_inode)?;
    let conn = Connection::open_with_flags(
        &canonical,
        OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON; PRAGMA query_only = ON;")?;
    require_exact_store_identity(&canonical, store_device, store_inode)?;
    require_main_database_path(&conn, &canonical)?;

    if entry_is_quarantined(&conn, current_attempt.claim().entry_id)? {
        return Ok(DurableObservation {
            successor_exact: false,
            predecessor_exact: false,
        });
    }

    let Some(outbox) = load_outbox_row(&conn, current_attempt.claim().entry_id)? else {
        return Ok(DurableObservation {
            successor_exact: false,
            predecessor_exact: false,
        });
    };
    let binding = load_binding_row(&conn, current_attempt)?;

    let identity_exact = common_attempt_identity_exact(&outbox, current_attempt);
    let predecessor_exact = identity_exact
        && outbox.stage == ATTEMPT_PREPARED_STAGE_V2
        && outbox.dispatch_started_at_ms.is_none()
        && outbox.updated_at_ms == current_attempt.durable_updated_at_ms()
        && binding.is_none();

    let successor_exact = identity_exact
        && outbox.stage == DISPATCH_STARTED_STAGE_V2
        && outbox.dispatch_started_at_ms == Some(now_ms)
        && outbox.updated_at_ms == now_ms
        && binding.as_ref().is_some_and(|row| {
            binding_row_exact(
                row,
                current_attempt,
                materialized,
                context,
                now_ms,
                expected_binding_digest,
            )
        });

    require_exact_store_identity(&canonical, store_device, store_inode)?;
    Ok(DurableObservation {
        successor_exact,
        predecessor_exact,
    })
}

fn common_attempt_identity_exact(
    row: &DurableOutboxRow,
    current_attempt: &QualifiedCurrentExecutionAttempt,
) -> bool {
    let claim = current_attempt.claim();
    row.worker_id.as_deref() == Some(current_attempt.worker_id())
        && row.lease_until_ms == Some(claim.lease_until_ms)
        && row.attempt_count == i64::from(claim.attempt_count)
        && row.current_attempt_id.as_deref() == Some(claim.attempt_id.as_str())
        && row.command_id == claim.command_id.as_str()
        && row.connector_instance == claim.connector_instance.as_str()
        && row.command_commitment_algorithm == digest_algorithm_code(claim.command_commitment.algorithm)
        && row.command_commitment_digest.as_slice() == claim.command_commitment.digest
        && row.side_effect_class == side_effect_code(claim.side_effect_class)
        && row.idempotency_key.as_deref()
            == claim.idempotency_key.as_ref().map(|value| value.as_str())
}

fn binding_row_exact(
    row: &DurableBindingRow,
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
    context: &DispatchContextCommitment,
    started_at_ms: i64,
    expected_binding_digest: Digest32,
) -> bool {
    let claim = current_attempt.claim();
    row.entry_id == claim.entry_id
        && row.attempt_id == claim.attempt_id.as_str()
        && row.command_id == claim.command_id.as_str()
        && row.connector_instance == claim.connector_instance.as_str()
        && row.command_commitment_algorithm == digest_algorithm_code(claim.command_commitment.algorithm)
        && row.command_commitment_digest.as_slice() == claim.command_commitment.digest
        && row.materialization_digest.as_slice() == materialized.materialization_digest().0
        && row.output_commitment_algorithm
            == digest_algorithm_code(materialized.output_commitment().algorithm)
        && row.output_commitment_digest.as_slice() == materialized.output_commitment().digest
        && row.admission_digest.as_slice() == materialized.admission_digest().0
        && row.determinism_digest.as_slice() == materialized.determinism_digest().0
        && row.provider_profile_algorithm
            == digest_algorithm_code(materialized.provider_profile_commitment().algorithm)
        && row.provider_profile_digest.as_slice() == materialized.provider_profile_commitment().digest
        && row.provider_root_algorithm
            == digest_algorithm_code(materialized.provider_trust_root_commitment().algorithm)
        && row.provider_root_digest.as_slice()
            == materialized.provider_trust_root_commitment().digest
        && row.materializer_release_algorithm
            == digest_algorithm_code(materialized.materializer_release().algorithm)
        && row.materializer_release_digest.as_slice() == materialized.materializer_release().digest
        && row.context_profile == context.profile()
        && row.context_digest.as_slice() == context.digest().0
        && row.worker_id == current_attempt.worker_id()
        && row.lease_until_ms == claim.lease_until_ms
        && row.started_at_ms == started_at_ms
        && row.binding_digest.as_slice() == expected_binding_digest.0
}

fn load_outbox_row(
    conn: &Connection,
    entry_id: i64,
) -> Result<Option<DurableOutboxRow>, DurableDispatchRereadError> {
    conn.query_row(
        "SELECT stage, worker_id, lease_until_ms, attempt_count, current_attempt_id,\n\
                command_id, connector_instance, command_commitment_algorithm,\n\
                command_commitment_digest, side_effect_class, idempotency_key,\n\
                updated_at_ms, dispatch_started_at_ms\n\
         FROM integration_outbox WHERE entry_id = ?1",
        params![entry_id],
        |row| {
            Ok(DurableOutboxRow {
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
                dispatch_started_at_ms: row.get(12)?,
            })
        },
    )
    .optional()
    .map_err(DurableDispatchRereadError::Sqlite)
}

fn load_binding_row(
    conn: &Connection,
    current_attempt: &QualifiedCurrentExecutionAttempt,
) -> Result<Option<DurableBindingRow>, DurableDispatchRereadError> {
    if !table_exists(conn, DISPATCH_TABLE)? {
        return Ok(None);
    }
    let claim = current_attempt.claim();
    conn.query_row(
        "SELECT entry_id, attempt_id, command_id, connector_instance,\n\
                command_commitment_algorithm, command_commitment_digest,\n\
                materialization_digest, output_commitment_algorithm, output_commitment_digest,\n\
                admission_digest, determinism_digest,\n\
                provider_profile_algorithm, provider_profile_digest,\n\
                provider_root_algorithm, provider_root_digest,\n\
                materializer_release_algorithm, materializer_release_digest,\n\
                context_profile, context_digest, worker_id, lease_until_ms, started_at_ms,\n\
                binding_digest\n\
         FROM integration_dispatch_binding_v2\n\
         WHERE entry_id = ?1 AND attempt_id = ?2",
        params![claim.entry_id, claim.attempt_id.as_str()],
        |row| {
            Ok(DurableBindingRow {
                entry_id: row.get(0)?,
                attempt_id: row.get(1)?,
                command_id: row.get(2)?,
                connector_instance: row.get(3)?,
                command_commitment_algorithm: row.get(4)?,
                command_commitment_digest: row.get(5)?,
                materialization_digest: row.get(6)?,
                output_commitment_algorithm: row.get(7)?,
                output_commitment_digest: row.get(8)?,
                admission_digest: row.get(9)?,
                determinism_digest: row.get(10)?,
                provider_profile_algorithm: row.get(11)?,
                provider_profile_digest: row.get(12)?,
                provider_root_algorithm: row.get(13)?,
                provider_root_digest: row.get(14)?,
                materializer_release_algorithm: row.get(15)?,
                materializer_release_digest: row.get(16)?,
                context_profile: row.get(17)?,
                context_digest: row.get(18)?,
                worker_id: row.get(19)?,
                lease_until_ms: row.get(20)?,
                started_at_ms: row.get(21)?,
                binding_digest: row.get(22)?,
            })
        },
    )
    .optional()
    .map_err(DurableDispatchRereadError::Sqlite)
}

fn entry_is_quarantined(
    conn: &Connection,
    entry_id: i64,
) -> Result<bool, DurableDispatchRereadError> {
    if !table_exists(conn, "integration_runtime_quarantine")? {
        return Ok(false);
    }
    conn.query_row(
        "SELECT EXISTS(SELECT 1 FROM integration_runtime_quarantine WHERE entry_id = ?1)",
        params![entry_id],
        |row| row.get(0),
    )
    .map_err(DurableDispatchRereadError::Sqlite)
}

fn table_exists(conn: &Connection, name: &str) -> Result<bool, DurableDispatchRereadError> {
    conn.query_row(
        "SELECT EXISTS(SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1)",
        params![name],
        |row| row.get(0),
    )
    .map_err(DurableDispatchRereadError::Sqlite)
}

fn canonical_exact_store_path(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<PathBuf, DurableDispatchRereadError> {
    let link = fs::symlink_metadata(path).map_err(DurableDispatchRereadError::Io)?;
    if link.file_type().is_symlink() {
        return Err(DurableDispatchRereadError::SymlinkStorePath);
    }
    let canonical = fs::canonicalize(path).map_err(DurableDispatchRereadError::Io)?;
    require_exact_store_identity(&canonical, expected_device, expected_inode)?;
    Ok(canonical)
}

fn require_exact_store_identity(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<(), DurableDispatchRereadError> {
    let metadata = fs::metadata(path).map_err(DurableDispatchRereadError::Io)?;
    if !metadata.is_file()
        || metadata.dev() != expected_device
        || metadata.ino() != expected_inode
    {
        return Err(DurableDispatchRereadError::StoreIdentityChanged);
    }
    Ok(())
}

fn require_main_database_path(
    conn: &Connection,
    expected_path: &Path,
) -> Result<(), DurableDispatchRereadError> {
    let opened: String = conn.query_row(
        "SELECT file FROM pragma_database_list WHERE name = 'main'",
        [],
        |row| row.get(0),
    )?;
    if opened.is_empty() {
        return Err(DurableDispatchRereadError::StoreIdentityChanged);
    }
    let opened = fs::canonicalize(opened).map_err(DurableDispatchRereadError::Io)?;
    if opened != expected_path {
        return Err(DurableDispatchRereadError::StoreIdentityChanged);
    }
    Ok(())
}

fn dispatch_binding_digest(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
    context: &DispatchContextCommitment,
    started_at_ms: i64,
    store_device: u64,
    store_inode: u64,
) -> Digest32 {
    let claim = current_attempt.claim();
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_BINDING);
    frame(&mut h, DISPATCH_CONTEXT_PROFILE.as_bytes());
    frame(&mut h, &claim.entry_id.to_le_bytes());
    frame(&mut h, claim.attempt_id.as_str().as_bytes());
    frame(&mut h, claim.command_id.as_str().as_bytes());
    frame(&mut h, claim.connector_instance.as_str().as_bytes());
    frame_commitment(&mut h, &claim.command_commitment);
    frame(&mut h, &materialized.materialization_digest().0);
    frame_commitment(&mut h, materialized.output_commitment());
    frame(&mut h, &materialized.admission_digest().0);
    frame(&mut h, &materialized.determinism_digest().0);
    frame_commitment(&mut h, materialized.provider_profile_commitment());
    frame_commitment(&mut h, materialized.provider_trust_root_commitment());
    frame_commitment(&mut h, materialized.materializer_release());
    frame(&mut h, context.profile().as_bytes());
    frame(&mut h, &context.digest().0);
    frame(&mut h, current_attempt.worker_id().as_bytes());
    frame(&mut h, &claim.lease_until_ms.to_le_bytes());
    frame(&mut h, &started_at_ms.to_le_bytes());
    frame(&mut h, &store_device.to_le_bytes());
    frame(&mut h, &store_inode.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
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

fn frame_commitment(h: &mut blake3::Hasher, commitment: &ContentCommitment) {
    frame(h, &[digest_algorithm_code(commitment.algorithm) as u8]);
    frame(h, &commitment.digest);
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum DurableDispatchRereadError {
    #[error("runtime store path is a symlink")]
    SymlinkStorePath,
    #[error("runtime store identity changed during commit-outcome reread")]
    StoreIdentityChanged,
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("runtime store filesystem error during commit-outcome reread: {0}")]
    Io(std::io::Error),
}

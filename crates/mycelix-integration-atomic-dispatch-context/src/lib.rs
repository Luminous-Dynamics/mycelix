//! Context-bound atomic `AttemptPrepared -> DispatchStarted` transition.
//!
//! This is the v2 child of the generic materialization-binding theorem. It adds
//! one mandatory profiled dispatch-context commitment to the same durable
//! transaction. For HTTP, that context can commit the exact current transport
//! policy + canonical request-plan qualification without making this crate
//! HTTP-specific.
//!
//! No provider I/O occurs here. As with v1, `tx.commit()?` is the last fallible
//! operation: once `DispatchStarted` is durable, normal error return is no longer
//! permitted from this function.

#[cfg(not(unix))]
compile_error!("context-bound integration dispatch v0.1 requires Unix file identity semantics");

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{
    ContentCommitment, DigestAlgorithm, ExternalOperationRef, SideEffectClass,
};
use mycelix_integration_materializer_runtime::MaterializedProviderRequest;
use mycelix_integration_runtime::{DispatchStarted, QualifiedCurrentExecutionAttempt};
use rusqlite::{params, Connection, OpenFlags, OptionalExtension, TransactionBehavior};
use std::fs;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};
use std::time::Duration;
use thiserror::Error;

pub const DISPATCH_CONTEXT_PROFILE: &str =
    "mycelix-integration-atomic-dispatch-context-v1-blake3-framed";
const DOMAIN_BINDING: &[u8] = b"mycelix/integration/atomic-dispatch-context/v1";
const TABLE_NAME: &str = "integration_dispatch_binding_v2";
const ATTEMPT_PREPARED_STAGE_V2: i64 = 4;
const DISPATCH_STARTED_STAGE_V2: i64 = 5;
const MAX_CONTEXT_PROFILE_BYTES: usize = 256;

/// Provider-neutral, caller-visible description of the exact transport/request
/// semantics that must be committed with `DispatchStarted`.
///
/// This object is intentionally constructible and grants no authority. The
/// enclosing final composer is responsible for deriving it from non-forgeable
/// current policy/plan qualifications.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DispatchContextCommitment {
    profile: String,
    digest: Digest32,
}

impl DispatchContextCommitment {
    pub fn new(profile: impl Into<String>, digest: Digest32) -> Result<Self, ContextDispatchError> {
        let profile = profile.into();
        if profile.is_empty()
            || profile.len() > MAX_CONTEXT_PROFILE_BYTES
            || profile.chars().any(char::is_control)
        {
            return Err(ContextDispatchError::InvalidContextProfile);
        }
        if digest.is_zero() {
            return Err(ContextDispatchError::ZeroContextDigest);
        }
        Ok(Self { profile, digest })
    }

    pub fn profile(&self) -> &str {
        &self.profile
    }

    pub fn digest(&self) -> Digest32 {
        self.digest
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Non-deserializable proof that one exact profiled dispatch context and one
/// exact materialization were durably committed in the same transaction that
/// crossed `DispatchStarted`.
#[derive(Debug)]
pub struct QualifiedContextBoundDispatchStart {
    dispatch: DispatchStarted,
    context: DispatchContextCommitment,
    materialization_digest: Digest32,
    output_commitment: ContentCommitment,
    binding_digest: Digest32,
    store_device: u64,
    store_inode: u64,
}

impl QualifiedContextBoundDispatchStart {
    pub fn dispatch(&self) -> &DispatchStarted {
        &self.dispatch
    }

    pub fn context(&self) -> &DispatchContextCommitment {
        &self.context
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

    pub const fn dispatch_context_bound_in_same_transaction_here(&self) -> bool {
        true
    }

    pub const fn dispatch_started_durably_here(&self) -> bool {
        true
    }

    pub const fn raw_provider_payload_persisted_here(&self) -> bool {
        false
    }

    pub const fn provider_call_started_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug)]
struct DurableDispatchRow {
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

pub fn bind_context_and_start_dispatch(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
    context: &DispatchContextCommitment,
    store_path: impl AsRef<Path>,
    now_ms: i64,
) -> Result<QualifiedContextBoundDispatchStart, ContextDispatchError> {
    if now_ms < 0 || now_ms < current_attempt.qualified_at_ms() {
        return Err(ContextDispatchError::InvalidTime);
    }
    if context.digest.is_zero() {
        return Err(ContextDispatchError::ZeroContextDigest);
    }

    let claim = current_attempt.claim();
    if claim.lease_until_ms <= now_ms {
        return Err(ContextDispatchError::LeaseExpired);
    }
    validate_materialization_identity(current_attempt, materialized)?;

    let store_device = current_attempt.store_device();
    let store_inode = current_attempt.store_inode();
    let canonical_path = canonical_exact_store_path(store_path.as_ref(), store_device, store_inode)?;

    let mut conn = Connection::open_with_flags(
        &canonical_path,
        OpenFlags::SQLITE_OPEN_READ_WRITE | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    require_exact_store_identity(&canonical_path, store_device, store_inode)?;
    require_main_database_path(&conn, &canonical_path)?;

    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    ensure_dispatch_binding_table(&tx)?;
    reject_quarantined_attempt(&tx, claim.entry_id)?;
    let row = load_durable_dispatch_row(&tx, claim.entry_id)?;
    validate_durable_row(current_attempt, &row, now_ms)?;

    let existing: Option<Vec<u8>> = tx
        .query_row(
            "SELECT binding_digest FROM integration_dispatch_binding_v2\n\
             WHERE entry_id = ?1 AND attempt_id = ?2",
            params![claim.entry_id, claim.attempt_id.as_str()],
            |row| row.get(0),
        )
        .optional()?;
    if existing.is_some() {
        return Err(ContextDispatchError::DispatchBindingAlreadyExists);
    }

    let binding_digest = dispatch_binding_digest(
        current_attempt,
        materialized,
        context,
        now_ms,
        store_device,
        store_inode,
    );

    tx.execute(
        "INSERT INTO integration_dispatch_binding_v2 (\n\
            entry_id, attempt_id, command_id, connector_instance,\n\
            command_commitment_algorithm, command_commitment_digest,\n\
            materialization_digest, output_commitment_algorithm, output_commitment_digest,\n\
            admission_digest, determinism_digest,\n\
            provider_profile_algorithm, provider_profile_digest,\n\
            provider_root_algorithm, provider_root_digest,\n\
            materializer_release_algorithm, materializer_release_digest,\n\
            context_profile, context_digest, worker_id, lease_until_ms, started_at_ms,\n\
            binding_digest\n\
         ) VALUES (\n\
            ?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12,\n\
            ?13, ?14, ?15, ?16, ?17, ?18, ?19, ?20, ?21, ?22, ?23\n\
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
            context.profile(),
            context.digest().0.as_slice(),
            current_attempt.worker_id(),
            claim.lease_until_ms,
            now_ms,
            binding_digest.0.as_slice(),
        ],
    )?;

    let changed = tx.execute(
        "UPDATE integration_outbox\n\
         SET stage = ?1, dispatch_started_at_ms = ?2, updated_at_ms = ?2\n\
         WHERE entry_id = ?3\n\
           AND stage = ?4\n\
           AND current_attempt_id = ?5\n\
           AND worker_id = ?6\n\
           AND lease_until_ms = ?7\n\
           AND attempt_count = ?8",
        params![
            DISPATCH_STARTED_STAGE_V2,
            now_ms,
            claim.entry_id,
            ATTEMPT_PREPARED_STAGE_V2,
            claim.attempt_id.as_str(),
            current_attempt.worker_id(),
            claim.lease_until_ms,
            i64::from(claim.attempt_count),
        ],
    )?;
    if changed != 1 {
        return Err(ContextDispatchError::ConcurrentAttemptChange);
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
    let context = context.clone();
    let materialization_digest = materialized.materialization_digest();
    let output_commitment = materialized.output_commitment().clone();

    // LAST FALLIBLE OPERATION. No normal error return may occur after this.
    tx.commit()?;

    Ok(QualifiedContextBoundDispatchStart {
        dispatch,
        context,
        materialization_digest,
        output_commitment,
        binding_digest,
        store_device,
        store_inode,
    })
}

fn validate_materialization_identity(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    materialized: &MaterializedProviderRequest,
) -> Result<(), ContextDispatchError> {
    let claim = current_attempt.claim();
    if materialized.entry_id() != claim.entry_id
        || materialized.attempt_id() != &claim.attempt_id
        || materialized.command_id() != &claim.command_id
        || materialized.connector_instance() != &claim.connector_instance
        || materialized.input_commitment() != &claim.command_commitment
    {
        return Err(ContextDispatchError::MaterializationAttemptMismatch);
    }
    Ok(())
}

fn load_durable_dispatch_row(
    tx: &rusqlite::Transaction<'_>,
    entry_id: i64,
) -> Result<DurableDispatchRow, ContextDispatchError> {
    tx.query_row(
        "SELECT stage, worker_id, lease_until_ms, attempt_count, current_attempt_id,\n\
                command_id, connector_instance, command_commitment_algorithm,\n\
                command_commitment_digest, side_effect_class, idempotency_key, updated_at_ms\n\
         FROM integration_outbox WHERE entry_id = ?1",
        params![entry_id],
        |row| {
            Ok(DurableDispatchRow {
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
    .ok_or(ContextDispatchError::UnknownEntry)
}

fn validate_durable_row(
    current_attempt: &QualifiedCurrentExecutionAttempt,
    row: &DurableDispatchRow,
    now_ms: i64,
) -> Result<(), ContextDispatchError> {
    let claim = current_attempt.claim();
    if row.stage != ATTEMPT_PREPARED_STAGE_V2 {
        return Err(ContextDispatchError::AttemptNotPrepared);
    }
    if row.worker_id.as_deref() != Some(current_attempt.worker_id()) {
        return Err(ContextDispatchError::WorkerMismatch);
    }
    if row.lease_until_ms != Some(claim.lease_until_ms) || claim.lease_until_ms <= now_ms {
        return Err(ContextDispatchError::LeaseExpired);
    }
    if row.attempt_count != i64::from(claim.attempt_count) {
        return Err(ContextDispatchError::AttemptCountMismatch);
    }
    if row.current_attempt_id.as_deref() != Some(claim.attempt_id.as_str()) {
        return Err(ContextDispatchError::AttemptIdentityMismatch);
    }
    if row.command_id != claim.command_id.as_str()
        || row.connector_instance != claim.connector_instance.as_str()
    {
        return Err(ContextDispatchError::CommandSubjectMismatch);
    }
    if row.command_commitment_algorithm != digest_algorithm_code(claim.command_commitment.algorithm)
        || row.command_commitment_digest.as_slice() != claim.command_commitment.digest
    {
        return Err(ContextDispatchError::CommandCommitmentMismatch);
    }
    if row.side_effect_class != side_effect_code(claim.side_effect_class) {
        return Err(ContextDispatchError::SideEffectMismatch);
    }
    if row.idempotency_key.as_deref()
        != claim.idempotency_key.as_ref().map(|value| value.as_str())
    {
        return Err(ContextDispatchError::IdempotencyMismatch);
    }
    if now_ms < row.updated_at_ms {
        return Err(ContextDispatchError::CausalTimeRegression);
    }
    Ok(())
}

fn canonical_exact_store_path(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<PathBuf, ContextDispatchError> {
    let link_meta = fs::symlink_metadata(path).map_err(ContextDispatchError::Io)?;
    if link_meta.file_type().is_symlink() {
        return Err(ContextDispatchError::SymlinkStorePath);
    }
    let canonical = fs::canonicalize(path).map_err(ContextDispatchError::Io)?;
    require_exact_store_identity(&canonical, expected_device, expected_inode)?;
    Ok(canonical)
}

fn require_exact_store_identity(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<(), ContextDispatchError> {
    let metadata = fs::metadata(path).map_err(ContextDispatchError::Io)?;
    if !metadata.is_file()
        || metadata.dev() != expected_device
        || metadata.ino() != expected_inode
    {
        return Err(ContextDispatchError::StoreIdentityChanged);
    }
    Ok(())
}

fn require_main_database_path(
    conn: &Connection,
    expected_path: &Path,
) -> Result<(), ContextDispatchError> {
    let opened: String = conn.query_row(
        "SELECT file FROM pragma_database_list WHERE name = 'main'",
        [],
        |row| row.get(0),
    )?;
    if opened.is_empty() {
        return Err(ContextDispatchError::StoreIdentityChanged);
    }
    let opened = fs::canonicalize(opened).map_err(ContextDispatchError::Io)?;
    if opened != expected_path {
        return Err(ContextDispatchError::StoreIdentityChanged);
    }
    Ok(())
}

fn reject_quarantined_attempt(
    tx: &rusqlite::Transaction<'_>,
    entry_id: i64,
) -> Result<(), ContextDispatchError> {
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
        Err(ContextDispatchError::AttemptQuarantined)
    } else {
        Ok(())
    }
}

fn ensure_dispatch_binding_table(
    tx: &rusqlite::Transaction<'_>,
) -> Result<(), ContextDispatchError> {
    tx.execute_batch(
        r#"
        CREATE TABLE IF NOT EXISTS integration_dispatch_binding_v2 (
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
            context_profile TEXT NOT NULL,
            context_digest BLOB NOT NULL,
            worker_id TEXT NOT NULL,
            lease_until_ms INTEGER NOT NULL,
            started_at_ms INTEGER NOT NULL,
            binding_digest BLOB NOT NULL,
            PRIMARY KEY(entry_id, attempt_id),
            FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
        );
        "#,
    )?;
    validate_dispatch_binding_schema(tx)
}

fn validate_dispatch_binding_schema(
    tx: &rusqlite::Transaction<'_>,
) -> Result<(), ContextDispatchError> {
    const EXPECTED: &[(&str, &str, i64, i64)] = &[
        ("entry_id", "INTEGER", 1, 1),
        ("attempt_id", "TEXT", 1, 2),
        ("command_id", "TEXT", 1, 0),
        ("connector_instance", "TEXT", 1, 0),
        ("command_commitment_algorithm", "INTEGER", 1, 0),
        ("command_commitment_digest", "BLOB", 1, 0),
        ("materialization_digest", "BLOB", 1, 0),
        ("output_commitment_algorithm", "INTEGER", 1, 0),
        ("output_commitment_digest", "BLOB", 1, 0),
        ("admission_digest", "BLOB", 1, 0),
        ("determinism_digest", "BLOB", 1, 0),
        ("provider_profile_algorithm", "INTEGER", 1, 0),
        ("provider_profile_digest", "BLOB", 1, 0),
        ("provider_root_algorithm", "INTEGER", 1, 0),
        ("provider_root_digest", "BLOB", 1, 0),
        ("materializer_release_algorithm", "INTEGER", 1, 0),
        ("materializer_release_digest", "BLOB", 1, 0),
        ("context_profile", "TEXT", 1, 0),
        ("context_digest", "BLOB", 1, 0),
        ("worker_id", "TEXT", 1, 0),
        ("lease_until_ms", "INTEGER", 1, 0),
        ("started_at_ms", "INTEGER", 1, 0),
        ("binding_digest", "BLOB", 1, 0),
    ];

    let mut statement = tx.prepare("PRAGMA table_info(integration_dispatch_binding_v2)")?;
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
        return Err(ContextDispatchError::DispatchBindingSchemaMismatch);
    }

    let foreign_keys = tx
        .prepare("PRAGMA foreign_key_list(integration_dispatch_binding_v2)")?
        .query_map([], |row| {
            Ok((
                row.get::<_, String>(2)?,
                row.get::<_, String>(3)?,
                row.get::<_, String>(4)?,
                row.get::<_, String>(5)?,
                row.get::<_, String>(6)?,
            ))
        })?
        .collect::<Result<Vec<_>, _>>()?;
    if foreign_keys
        != vec![(
            "integration_outbox".to_owned(),
            "entry_id".to_owned(),
            "entry_id".to_owned(),
            "NO ACTION".to_owned(),
            "NO ACTION".to_owned(),
        )]
    {
        return Err(ContextDispatchError::DispatchBindingSchemaMismatch);
    }

    let trigger_count: i64 = tx.query_row(
        "SELECT COUNT(*) FROM sqlite_master WHERE type = 'trigger' AND tbl_name = ?1",
        params![TABLE_NAME],
        |row| row.get(0),
    )?;
    if trigger_count != 0 {
        return Err(ContextDispatchError::DispatchBindingSchemaMismatch);
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
pub enum ContextDispatchError {
    #[error("dispatch context profile is invalid")]
    InvalidContextProfile,
    #[error("dispatch context digest must be non-zero")]
    ZeroContextDigest,
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
    #[error("dispatch start time is behind durable causal frontier")]
    CausalTimeRegression,
    #[error("durable attempt is quarantined")]
    AttemptQuarantined,
    #[error("context-bound dispatch binding already exists for this attempt")]
    DispatchBindingAlreadyExists,
    #[error("context-bound dispatch table differs from v2 structural contract")]
    DispatchBindingSchemaMismatch,
    #[error("attempt changed while crossing DispatchStarted")]
    ConcurrentAttemptChange,
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("runtime store filesystem error: {0}")]
    Io(std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_outbox(conn: &Connection) {
        conn.execute_batch(
            "PRAGMA foreign_keys = ON;\n\
             CREATE TABLE integration_outbox (entry_id INTEGER PRIMARY KEY AUTOINCREMENT);",
        )
        .unwrap();
    }

    #[test]
    fn context_requires_profile_and_nonzero_digest() {
        assert!(matches!(
            DispatchContextCommitment::new("", Digest32([1; 32])),
            Err(ContextDispatchError::InvalidContextProfile)
        ));
        assert!(matches!(
            DispatchContextCommitment::new("http-v1", Digest32([0; 32])),
            Err(ContextDispatchError::ZeroContextDigest)
        ));
        assert!(DispatchContextCommitment::new("http-v1", Digest32([1; 32])).is_ok());
    }

    #[test]
    fn dispatch_binding_v2_schema_is_exact_and_trigger_free() {
        let mut conn = Connection::open_in_memory().unwrap();
        create_outbox(&conn);
        let tx = conn
            .transaction_with_behavior(TransactionBehavior::Immediate)
            .unwrap();
        ensure_dispatch_binding_table(&tx).unwrap();
        validate_dispatch_binding_schema(&tx).unwrap();
        tx.commit().unwrap();
    }

    #[test]
    fn context_changes_binding_digest_domain() {
        let a = DispatchContextCommitment::new("http-v1", Digest32([1; 32])).unwrap();
        let b = DispatchContextCommitment::new("http-v1", Digest32([2; 32])).unwrap();
        assert_ne!(a, b);
        assert!(!a.grants_execution_authority());
    }
}

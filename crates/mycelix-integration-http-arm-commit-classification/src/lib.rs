//! Fail-closed commit classification for HTTP `Prepared -> WriteMayBegin` arming.
//!
//! The existing HTTP transport journal remains the sole writer. This crate takes
//! an exact read-only snapshot of the durable Prepared row and its context-bound
//! dispatch identity, consumes the process-local Prepared capability through the
//! existing journal arming function exactly once, and classifies any returned
//! error only by independently re-reading the exact durable state.
//!
//! A recovered durable `WriteMayBegin` successor never reconstructs
//! `ArmedHttpApplicationWrite`; it is recovery-only. An exact unchanged
//! predecessor is persistence evidence only and never authorizes retry.

use mycelix_integration_core::DigestAlgorithm;
use mycelix_integration_http_transport_journal::{
    ArmedHttpApplicationWrite, PreparedHttpTransportAttempt, TransportJournalError,
    arm_http_application_write,
};
use mycelix_integration_runtime_store_binding::{
    QualifiedRuntimeStoreBinding, RuntimeStoreBindingError,
};
use mycelix_integration_sqlite_commit_classification::{
    CommitClassification, classify_exact_durable_transition,
};
use rusqlite::{Connection, OpenFlags, OptionalExtension, TransactionBehavior, params};
use std::fs;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;

pub const HTTP_ARM_COMMIT_CLASSIFICATION_PROFILE: &str =
    "mycelix-integration-http-arm-commit-classification-v1";

const JOURNAL_TABLE: &str = "integration_http_transport_attempt_v1";
const DISPATCH_TABLE: &str = "integration_dispatch_binding_v2";
const JOURNAL_PREPARED: i64 = 0;
const JOURNAL_WRITE_MAY_BEGIN: i64 = 1;

pub enum ClassifiedHttpArmOutcome {
    /// The journal writer returned success, so the live process-local write
    /// capability is retained.
    Committed(Box<ArmedHttpApplicationWrite>),
    /// The writer returned an error, but an independent exact reread proves the
    /// durable ambiguity floor was crossed. No live write capability is rebuilt.
    RecoveredCommitted(RecoveredCommittedHttpArm),
    /// The writer returned an error and the exact durable predecessor was
    /// independently re-observed. Retry is still not authorized here.
    DefinitelyNotCommitted(DefinitelyNotCommittedHttpArm),
    /// Neither exact durable state could be proved, or the reread itself failed.
    IndeterminateCommit(IndeterminateHttpArm),
}

impl ClassifiedHttpArmOutcome {
    pub const fn classification(&self) -> CommitClassification {
        match self {
            Self::Committed(_) | Self::RecoveredCommitted(_) => CommitClassification::Committed,
            Self::DefinitelyNotCommitted(_) => CommitClassification::DefinitelyNotCommitted,
            Self::IndeterminateCommit(_) => CommitClassification::IndeterminateCommit,
        }
    }

    pub const fn live_application_write_capability_available_here(&self) -> bool {
        matches!(self, Self::Committed(_))
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub struct RecoveredCommittedHttpArm {
    entry_id: i64,
    attempt_id: String,
    armed_at_ms: i64,
    original_error: TransportJournalError,
}

impl RecoveredCommittedHttpArm {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn armed_at_ms(&self) -> i64 {
        self.armed_at_ms
    }

    pub fn original_error(&self) -> &TransportJournalError {
        &self.original_error
    }

    pub const fn exact_write_may_begin_reobserved_here(&self) -> bool {
        true
    }

    pub const fn live_write_capability_reconstructed_here(&self) -> bool {
        false
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

pub struct DefinitelyNotCommittedHttpArm {
    entry_id: i64,
    attempt_id: String,
    original_error: TransportJournalError,
}

impl DefinitelyNotCommittedHttpArm {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn original_error(&self) -> &TransportJournalError {
        &self.original_error
    }

    pub const fn exact_prepared_predecessor_reobserved_here(&self) -> bool {
        true
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub struct IndeterminateHttpArm {
    entry_id: i64,
    attempt_id: String,
    original_error: TransportJournalError,
    reread_error: Option<DurableHttpArmRereadError>,
}

impl IndeterminateHttpArm {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn original_error(&self) -> &TransportJournalError {
        &self.original_error
    }

    pub fn reread_error(&self) -> Option<&DurableHttpArmRereadError> {
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

#[derive(Clone, Debug, PartialEq, Eq)]
struct ArmSubject {
    entry_id: i64,
    attempt_id: String,
    issuance_digest: [u8; 32],
    request_commitment_algorithm: i64,
    request_commitment_digest: [u8; 32],
    descriptor_valid_until_ms: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct DurableArmRow {
    stage: i64,
    command_id: String,
    connector_instance: String,
    dispatch_binding_digest: Vec<u8>,
    issuance_digest: Vec<u8>,
    request_commitment_algorithm: i64,
    request_commitment_digest: Vec<u8>,
    descriptor_valid_until_ms: i64,
    prepared_at_ms: i64,
    armed_at_ms: Option<i64>,
    observation_digest: Option<Vec<u8>>,
    observation_disposition: Option<i64>,
    updated_at_ms: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct DurableArmSnapshot {
    row: Option<DurableArmRow>,
    dispatch_binding_digest: Option<Vec<u8>>,
}

/// Consume one exact Prepared capability and classify persistence uncertainty
/// without deriving retry or reconstructing write authority from durable state.
pub fn arm_http_application_write_classified(
    prepared: PreparedHttpTransportAttempt,
    store_binding: &QualifiedRuntimeStoreBinding,
    now_ms: i64,
) -> Result<ClassifiedHttpArmOutcome, HttpArmClassificationError> {
    let subject = subject_from_prepared(&prepared)?;
    let predecessor = read_exact_arm_snapshot(store_binding, &subject)?;
    validate_prepared_snapshot(&predecessor, &subject)?;

    match arm_http_application_write(prepared, store_binding.path(), now_ms) {
        Ok(armed) => Ok(ClassifiedHttpArmOutcome::Committed(Box::new(armed))),
        Err(original_error) => Ok(classify_returned_arm_error(
            store_binding,
            subject,
            predecessor,
            now_ms,
            original_error,
        )),
    }
}

fn subject_from_prepared(
    prepared: &PreparedHttpTransportAttempt,
) -> Result<ArmSubject, HttpArmClassificationError> {
    let descriptor_valid_until_ms = i64::try_from(prepared.valid_until_ms())
        .map_err(|_| HttpArmClassificationError::TimeOverflow)?;
    Ok(ArmSubject {
        entry_id: prepared.entry_id(),
        attempt_id: prepared.attempt_id().as_str().to_owned(),
        issuance_digest: prepared.issuance_digest().0,
        request_commitment_algorithm: digest_algorithm_code(
            prepared.observation_request_commitment().algorithm,
        ),
        request_commitment_digest: prepared.observation_request_commitment().digest,
        descriptor_valid_until_ms,
    })
}

fn validate_prepared_snapshot(
    snapshot: &DurableArmSnapshot,
    subject: &ArmSubject,
) -> Result<(), HttpArmClassificationError> {
    let row = snapshot
        .row
        .as_ref()
        .ok_or(HttpArmClassificationError::MissingPreparedJournal)?;
    let dispatch = snapshot
        .dispatch_binding_digest
        .as_ref()
        .ok_or(HttpArmClassificationError::MissingContextDispatch)?;

    if row.stage != JOURNAL_PREPARED
        || row.issuance_digest.as_slice() != subject.issuance_digest.as_slice()
        || row.request_commitment_algorithm != subject.request_commitment_algorithm
        || row.request_commitment_digest.as_slice() != subject.request_commitment_digest.as_slice()
        || row.descriptor_valid_until_ms != subject.descriptor_valid_until_ms
        || row.armed_at_ms.is_some()
        || row.observation_digest.is_some()
        || row.observation_disposition.is_some()
        || row.dispatch_binding_digest.as_slice() != dispatch.as_slice()
    {
        return Err(HttpArmClassificationError::PreparedSubjectMismatch);
    }
    Ok(())
}

fn classify_returned_arm_error(
    store_binding: &QualifiedRuntimeStoreBinding,
    subject: ArmSubject,
    predecessor: DurableArmSnapshot,
    now_ms: i64,
    original_error: TransportJournalError,
) -> ClassifiedHttpArmOutcome {
    match read_exact_arm_snapshot(store_binding, &subject) {
        Ok(successor) => {
            let predecessor_exact = successor == predecessor;
            let successor_exact = arm_successor_exact(&predecessor, &successor, now_ms);
            match classify_exact_durable_transition(successor_exact, predecessor_exact) {
                CommitClassification::Committed => {
                    ClassifiedHttpArmOutcome::RecoveredCommitted(RecoveredCommittedHttpArm {
                        entry_id: subject.entry_id,
                        attempt_id: subject.attempt_id,
                        armed_at_ms: now_ms,
                        original_error,
                    })
                }
                CommitClassification::DefinitelyNotCommitted => {
                    ClassifiedHttpArmOutcome::DefinitelyNotCommitted(
                        DefinitelyNotCommittedHttpArm {
                            entry_id: subject.entry_id,
                            attempt_id: subject.attempt_id,
                            original_error,
                        },
                    )
                }
                CommitClassification::IndeterminateCommit => {
                    ClassifiedHttpArmOutcome::IndeterminateCommit(IndeterminateHttpArm {
                        entry_id: subject.entry_id,
                        attempt_id: subject.attempt_id,
                        original_error,
                        reread_error: None,
                    })
                }
            }
        }
        Err(reread_error) => ClassifiedHttpArmOutcome::IndeterminateCommit(IndeterminateHttpArm {
            entry_id: subject.entry_id,
            attempt_id: subject.attempt_id,
            original_error,
            reread_error: Some(reread_error),
        }),
    }
}

fn arm_successor_exact(
    predecessor: &DurableArmSnapshot,
    successor: &DurableArmSnapshot,
    now_ms: i64,
) -> bool {
    if predecessor.dispatch_binding_digest != successor.dispatch_binding_digest {
        return false;
    }
    let (Some(before), Some(after)) = (predecessor.row.as_ref(), successor.row.as_ref()) else {
        return false;
    };
    let mut expected = before.clone();
    expected.stage = JOURNAL_WRITE_MAY_BEGIN;
    expected.armed_at_ms = Some(now_ms);
    expected.updated_at_ms = now_ms;
    after == &expected
}

fn read_exact_arm_snapshot(
    store_binding: &QualifiedRuntimeStoreBinding,
    subject: &ArmSubject,
) -> Result<DurableArmSnapshot, DurableHttpArmRereadError> {
    store_binding.revalidate()?;
    let mut conn = Connection::open_with_flags(
        store_binding.path(),
        OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON; PRAGMA query_only = ON;")?;
    require_main_database_path(&conn, store_binding.path())?;

    // All facts used for one persistence verdict come from one coherent SQLite
    // snapshot. A later writer cannot splice predecessor and successor evidence.
    let tx = conn.transaction_with_behavior(TransactionBehavior::Deferred)?;
    require_exact_journal_schema(&tx)?;
    let row = load_arm_row(&tx, subject)?;
    let dispatch_binding_digest = load_dispatch_binding(&tx, subject)?;
    let snapshot = DurableArmSnapshot {
        row,
        dispatch_binding_digest,
    };
    tx.commit()?;
    store_binding.revalidate()?;
    Ok(snapshot)
}

fn load_arm_row(
    conn: &Connection,
    subject: &ArmSubject,
) -> Result<Option<DurableArmRow>, DurableHttpArmRereadError> {
    conn.query_row(
        "SELECT stage, command_id, connector_instance, dispatch_binding_digest, issuance_digest,\n\
                request_commitment_algorithm, request_commitment_digest,\n\
                descriptor_valid_until_ms, prepared_at_ms, armed_at_ms,\n\
                observation_digest, observation_disposition, updated_at_ms\n\
         FROM integration_http_transport_attempt_v1\n\
         WHERE entry_id = ?1 AND attempt_id = ?2",
        params![subject.entry_id, subject.attempt_id.as_str()],
        |row| {
            Ok(DurableArmRow {
                stage: row.get(0)?,
                command_id: row.get(1)?,
                connector_instance: row.get(2)?,
                dispatch_binding_digest: row.get(3)?,
                issuance_digest: row.get(4)?,
                request_commitment_algorithm: row.get(5)?,
                request_commitment_digest: row.get(6)?,
                descriptor_valid_until_ms: row.get(7)?,
                prepared_at_ms: row.get(8)?,
                armed_at_ms: row.get(9)?,
                observation_digest: row.get(10)?,
                observation_disposition: row.get(11)?,
                updated_at_ms: row.get(12)?,
            })
        },
    )
    .optional()
    .map_err(DurableHttpArmRereadError::Sqlite)
}

fn load_dispatch_binding(
    conn: &Connection,
    subject: &ArmSubject,
) -> Result<Option<Vec<u8>>, DurableHttpArmRereadError> {
    if !table_exists(conn, DISPATCH_TABLE)? {
        return Ok(None);
    }
    conn.query_row(
        "SELECT binding_digest FROM integration_dispatch_binding_v2\n\
         WHERE entry_id = ?1 AND attempt_id = ?2",
        params![subject.entry_id, subject.attempt_id.as_str()],
        |row| row.get(0),
    )
    .optional()
    .map_err(DurableHttpArmRereadError::Sqlite)
}

fn require_exact_journal_schema(conn: &Connection) -> Result<(), DurableHttpArmRereadError> {
    if !table_exists(conn, JOURNAL_TABLE)? {
        return Err(DurableHttpArmRereadError::MissingJournalTable);
    }
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
    let actual = conn
        .prepare("PRAGMA table_info(integration_http_transport_attempt_v1)")?
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
        return Err(DurableHttpArmRereadError::JournalSchemaMismatch);
    }

    let foreign_keys = conn
        .prepare("PRAGMA foreign_key_list(integration_http_transport_attempt_v1)")?
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
        return Err(DurableHttpArmRereadError::JournalSchemaMismatch);
    }

    let trigger_count: i64 = conn.query_row(
        "SELECT COUNT(*) FROM sqlite_master WHERE type = 'trigger' AND tbl_name = ?1",
        params![JOURNAL_TABLE],
        |row| row.get(0),
    )?;
    if trigger_count != 0 {
        return Err(DurableHttpArmRereadError::JournalSchemaMismatch);
    }
    Ok(())
}

fn table_exists(conn: &Connection, name: &str) -> Result<bool, DurableHttpArmRereadError> {
    conn.query_row(
        "SELECT EXISTS(SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1)",
        params![name],
        |row| row.get(0),
    )
    .map_err(DurableHttpArmRereadError::Sqlite)
}

fn require_main_database_path(
    conn: &Connection,
    expected_path: &Path,
) -> Result<(), DurableHttpArmRereadError> {
    let opened: String = conn.query_row(
        "SELECT file FROM pragma_database_list WHERE name = 'main'",
        [],
        |row| row.get(0),
    )?;
    if opened.is_empty() {
        return Err(DurableHttpArmRereadError::StoreIdentityChanged);
    }
    let opened = fs::canonicalize(opened).map_err(DurableHttpArmRereadError::Io)?;
    if opened != expected_path {
        return Err(DurableHttpArmRereadError::StoreIdentityChanged);
    }
    Ok(())
}

fn digest_algorithm_code(value: DigestAlgorithm) -> i64 {
    match value {
        DigestAlgorithm::Sha256 => 1,
    }
}

#[derive(Debug, Error)]
pub enum HttpArmClassificationError {
    #[error("prepared HTTP transport journal row is missing")]
    MissingPreparedJournal,
    #[error("context-bound dispatch evidence is missing for the prepared attempt")]
    MissingContextDispatch,
    #[error("prepared HTTP transport subject differs from exact durable state")]
    PreparedSubjectMismatch,
    #[error("prepared HTTP transport horizon cannot be represented")]
    TimeOverflow,
    #[error(transparent)]
    Reread(#[from] DurableHttpArmRereadError),
}

#[derive(Debug, Error)]
pub enum DurableHttpArmRereadError {
    #[error("HTTP transport journal table is missing")]
    MissingJournalTable,
    #[error("HTTP transport journal schema differs from the v1 structural contract")]
    JournalSchemaMismatch,
    #[error("runtime store identity changed during HTTP arm commit reread")]
    StoreIdentityChanged,
    #[error(transparent)]
    StoreBinding(#[from] RuntimeStoreBindingError),
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("runtime store filesystem error during HTTP arm commit reread: {0}")]
    Io(std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row() -> DurableArmRow {
        DurableArmRow {
            stage: JOURNAL_PREPARED,
            command_id: "cmd-1".to_owned(),
            connector_instance: "connector-1".to_owned(),
            dispatch_binding_digest: vec![1; 32],
            issuance_digest: vec![2; 32],
            request_commitment_algorithm: 1,
            request_commitment_digest: vec![3; 32],
            descriptor_valid_until_ms: 500,
            prepared_at_ms: 100,
            armed_at_ms: None,
            observation_digest: None,
            observation_disposition: None,
            updated_at_ms: 100,
        }
    }

    fn snapshot() -> DurableArmSnapshot {
        DurableArmSnapshot {
            row: Some(row()),
            dispatch_binding_digest: Some(vec![1; 32]),
        }
    }

    #[test]
    fn exact_successor_changes_only_monotonic_arm_fields() {
        let before = snapshot();
        let mut after = before.clone();
        let row = after.row.as_mut().unwrap();
        row.stage = JOURNAL_WRITE_MAY_BEGIN;
        row.armed_at_ms = Some(200);
        row.updated_at_ms = 200;
        assert!(arm_successor_exact(&before, &after, 200));

        after.row.as_mut().unwrap().issuance_digest[0] ^= 1;
        assert!(!arm_successor_exact(&before, &after, 200));
    }

    #[test]
    fn unchanged_snapshot_is_only_predecessor_shape() {
        let before = snapshot();
        let after = before.clone();
        assert_eq!(before, after);
        assert!(!arm_successor_exact(&before, &after, 200));
    }
}

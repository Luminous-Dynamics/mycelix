//! Fail-closed persistence classification for HTTP transport observations.
//!
//! The mature HTTP transport journal remains the only writer. This crate owns
//! the external observation-write boundary, first requalifies caller-supplied
//! observation evidence, snapshots the exact durable predecessor, invokes the
//! journal writer exactly once, and classifies any returned error only from a
//! fresh exact SQLite snapshot.
//!
//! Observation persistence consumes its Prepared/Armed capability. Therefore
//! even an exact `DefinitelyNotCommitted` verdict never recreates that capability
//! and never grants ordinary retry authority.

use mycelix_integration_http_transport_journal::{
    ArmedHttpApplicationWrite, PreparedHttpTransportAttempt, TransportJournalError,
    record_armed_observation, record_prewrite_nonissuance,
};
use mycelix_integration_runtime_store_binding::{
    QualifiedRuntimeStoreBinding, RuntimeStoreBindingError,
};
use mycelix_integration_sqlite_commit_classification::{
    CommitClassification, classify_exact_durable_transition,
};
use mycelix_integration_transport_observation::{
    QualifiedTransportObservation, TransportObservationDisposition, TransportObservationError,
    qualify_transport_observation,
};
use rusqlite::{Connection, OpenFlags, OptionalExtension, TransactionBehavior, params};
use std::fs;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;

pub const HTTP_OBSERVATION_COMMIT_CLASSIFICATION_PROFILE: &str =
    "mycelix-integration-http-observation-commit-classification-v1";

const JOURNAL_TABLE: &str = "integration_http_transport_attempt_v1";
const DISPATCH_TABLE: &str = "integration_dispatch_binding_v2";
const JOURNAL_PREPARED: i64 = 0;
const JOURNAL_WRITE_MAY_BEGIN: i64 = 1;
const JOURNAL_OBSERVATION_RECORDED: i64 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObservationWritePath {
    PreparedPrewriteNonissuance,
    Armed,
}

pub enum ClassifiedHttpObservationOutcome {
    Committed(CommittedHttpObservation),
    RecoveredCommitted(RecoveredCommittedHttpObservation),
    DefinitelyNotCommitted(DefinitelyNotCommittedHttpObservation),
    IndeterminateCommit(IndeterminateHttpObservation),
}

impl ClassifiedHttpObservationOutcome {
    pub const fn classification(&self) -> CommitClassification {
        match self {
            Self::Committed(_) | Self::RecoveredCommitted(_) => CommitClassification::Committed,
            Self::DefinitelyNotCommitted(_) => CommitClassification::DefinitelyNotCommitted,
            Self::IndeterminateCommit(_) => CommitClassification::IndeterminateCommit,
        }
    }

    pub const fn consumed_write_capability_reconstructed_here(&self) -> bool {
        false
    }

    pub const fn ordinary_retry_authorized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub struct CommittedHttpObservation {
    entry_id: i64,
    attempt_id: String,
    observation_digest: [u8; 32],
    disposition: TransportObservationDisposition,
    path: ObservationWritePath,
}

impl CommittedHttpObservation {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn observation_digest(&self) -> [u8; 32] {
        self.observation_digest
    }

    pub const fn disposition(&self) -> TransportObservationDisposition {
        self.disposition
    }

    pub const fn path(&self) -> ObservationWritePath {
        self.path
    }

    pub const fn exact_observation_committed_here(&self) -> bool {
        true
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub struct RecoveredCommittedHttpObservation {
    entry_id: i64,
    attempt_id: String,
    observation_digest: [u8; 32],
    disposition: TransportObservationDisposition,
    path: ObservationWritePath,
    original_error: TransportJournalError,
}

impl RecoveredCommittedHttpObservation {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn observation_digest(&self) -> [u8; 32] {
        self.observation_digest
    }

    pub const fn disposition(&self) -> TransportObservationDisposition {
        self.disposition
    }

    pub const fn path(&self) -> ObservationWritePath {
        self.path
    }

    pub fn original_error(&self) -> &TransportJournalError {
        &self.original_error
    }

    pub const fn exact_successor_reobserved_here(&self) -> bool {
        true
    }

    pub const fn consumed_write_capability_reconstructed_here(&self) -> bool {
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

pub struct DefinitelyNotCommittedHttpObservation {
    entry_id: i64,
    attempt_id: String,
    path: ObservationWritePath,
    original_error: TransportJournalError,
}

impl DefinitelyNotCommittedHttpObservation {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub const fn path(&self) -> ObservationWritePath {
        self.path
    }

    pub fn original_error(&self) -> &TransportJournalError {
        &self.original_error
    }

    pub const fn exact_prepared_predecessor_reobserved_here(&self) -> bool {
        true
    }

    pub const fn consumed_write_capability_reconstructed_here(&self) -> bool {
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

pub struct IndeterminateHttpObservation {
    entry_id: i64,
    attempt_id: String,
    path: ObservationWritePath,
    original_error: TransportJournalError,
    reread_error: Option<DurableHttpObservationRereadError>,
}

impl IndeterminateHttpObservation {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub const fn path(&self) -> ObservationWritePath {
        self.path
    }

    pub fn original_error(&self) -> &TransportJournalError {
        &self.original_error
    }

    pub fn reread_error(&self) -> Option<&DurableHttpObservationRereadError> {
        self.reread_error.as_ref()
    }

    pub const fn consumed_write_capability_reconstructed_here(&self) -> bool {
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

#[derive(Clone, Debug, PartialEq, Eq)]
struct ObservationSubject {
    entry_id: i64,
    attempt_id: String,
    issuance_digest: [u8; 32],
    request_commitment_algorithm: i64,
    request_commitment_digest: [u8; 32],
    descriptor_valid_until_ms: i64,
    predecessor_stage: i64,
    predecessor_armed_at_ms: Option<i64>,
    observation_digest: [u8; 32],
    observation_disposition: i64,
    path: ObservationWritePath,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct DurableObservationRow {
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
struct DurableObservationSnapshot {
    row: Option<DurableObservationRow>,
    dispatch_binding_digest: Option<Vec<u8>>,
}

pub fn record_prewrite_nonissuance_classified(
    prepared: PreparedHttpTransportAttempt,
    observation: &QualifiedTransportObservation,
    store_binding: &QualifiedRuntimeStoreBinding,
    now_ms: i64,
) -> Result<ClassifiedHttpObservationOutcome, HttpObservationClassificationError> {
    requalify_observation(observation)?;
    let subject = subject_from_prepared(&prepared, observation)?;
    let predecessor = read_exact_observation_snapshot(store_binding, &subject)?;
    validate_predecessor_snapshot(&predecessor, &subject)?;

    match record_prewrite_nonissuance(prepared, observation, store_binding.path(), now_ms) {
        Ok(()) => Ok(ClassifiedHttpObservationOutcome::Committed(committed(
            &subject,
        ))),
        Err(original_error) => Ok(classify_returned_observation_error(
            store_binding,
            subject,
            predecessor,
            now_ms,
            original_error,
        )),
    }
}

pub fn record_armed_observation_classified(
    armed: ArmedHttpApplicationWrite,
    observation: &QualifiedTransportObservation,
    store_binding: &QualifiedRuntimeStoreBinding,
    now_ms: i64,
) -> Result<ClassifiedHttpObservationOutcome, HttpObservationClassificationError> {
    requalify_observation(observation)?;
    let subject = subject_from_armed(&armed, observation)?;
    let predecessor = read_exact_observation_snapshot(store_binding, &subject)?;
    validate_predecessor_snapshot(&predecessor, &subject)?;

    match record_armed_observation(armed, observation, store_binding.path(), now_ms) {
        Ok(()) => Ok(ClassifiedHttpObservationOutcome::Committed(committed(
            &subject,
        ))),
        Err(original_error) => Ok(classify_returned_observation_error(
            store_binding,
            subject,
            predecessor,
            now_ms,
            original_error,
        )),
    }
}

fn requalify_observation(
    observation: &QualifiedTransportObservation,
) -> Result<(), HttpObservationClassificationError> {
    let recomputed = qualify_transport_observation(observation.evidence.clone())
        .map_err(HttpObservationClassificationError::ObservationEvidenceInvalid)?;
    if recomputed != *observation {
        return Err(HttpObservationClassificationError::ObservationIntegrityMismatch);
    }
    Ok(())
}

fn subject_from_prepared(
    prepared: &PreparedHttpTransportAttempt,
    observation: &QualifiedTransportObservation,
) -> Result<ObservationSubject, HttpObservationClassificationError> {
    Ok(ObservationSubject {
        entry_id: prepared.entry_id(),
        attempt_id: prepared.attempt_id().as_str().to_owned(),
        issuance_digest: prepared.issuance_digest().0,
        request_commitment_algorithm: digest_algorithm_code(
            prepared.observation_request_commitment().algorithm,
        ),
        request_commitment_digest: prepared.observation_request_commitment().digest,
        descriptor_valid_until_ms: i64::try_from(prepared.valid_until_ms())
            .map_err(|_| HttpObservationClassificationError::TimeOverflow)?,
        predecessor_stage: JOURNAL_PREPARED,
        predecessor_armed_at_ms: None,
        observation_digest: observation.observation_digest.0,
        observation_disposition: disposition_code(observation.disposition),
        path: ObservationWritePath::PreparedPrewriteNonissuance,
    })
}

fn subject_from_armed(
    armed: &ArmedHttpApplicationWrite,
    observation: &QualifiedTransportObservation,
) -> Result<ObservationSubject, HttpObservationClassificationError> {
    Ok(ObservationSubject {
        entry_id: armed.entry_id(),
        attempt_id: armed.attempt_id().as_str().to_owned(),
        issuance_digest: armed.issuance_digest().0,
        request_commitment_algorithm: digest_algorithm_code(
            armed.observation_request_commitment().algorithm,
        ),
        request_commitment_digest: armed.observation_request_commitment().digest,
        descriptor_valid_until_ms: i64::try_from(armed.valid_until_ms())
            .map_err(|_| HttpObservationClassificationError::TimeOverflow)?,
        predecessor_stage: JOURNAL_WRITE_MAY_BEGIN,
        predecessor_armed_at_ms: Some(armed.armed_at_ms()),
        observation_digest: observation.observation_digest.0,
        observation_disposition: disposition_code(observation.disposition),
        path: ObservationWritePath::Armed,
    })
}

fn committed(subject: &ObservationSubject) -> CommittedHttpObservation {
    CommittedHttpObservation {
        entry_id: subject.entry_id,
        attempt_id: subject.attempt_id.clone(),
        observation_digest: subject.observation_digest,
        disposition: disposition_from_code(subject.observation_disposition),
        path: subject.path,
    }
}

fn validate_predecessor_snapshot(
    snapshot: &DurableObservationSnapshot,
    subject: &ObservationSubject,
) -> Result<(), HttpObservationClassificationError> {
    let row = snapshot
        .row
        .as_ref()
        .ok_or(HttpObservationClassificationError::MissingJournal)?;
    let dispatch = snapshot
        .dispatch_binding_digest
        .as_ref()
        .ok_or(HttpObservationClassificationError::MissingContextDispatch)?;

    if row.stage != subject.predecessor_stage
        || row.issuance_digest.as_slice() != subject.issuance_digest.as_slice()
        || row.request_commitment_algorithm != subject.request_commitment_algorithm
        || row.request_commitment_digest.as_slice() != subject.request_commitment_digest.as_slice()
        || row.descriptor_valid_until_ms != subject.descriptor_valid_until_ms
        || row.armed_at_ms != subject.predecessor_armed_at_ms
        || row.observation_digest.is_some()
        || row.observation_disposition.is_some()
        || row.dispatch_binding_digest.as_slice() != dispatch.as_slice()
    {
        return Err(HttpObservationClassificationError::PredecessorSubjectMismatch);
    }
    Ok(())
}

fn classify_returned_observation_error(
    store_binding: &QualifiedRuntimeStoreBinding,
    subject: ObservationSubject,
    predecessor: DurableObservationSnapshot,
    now_ms: i64,
    original_error: TransportJournalError,
) -> ClassifiedHttpObservationOutcome {
    match read_exact_observation_snapshot(store_binding, &subject) {
        Ok(successor) => {
            let predecessor_exact = successor == predecessor;
            let successor_exact =
                observation_successor_exact(&predecessor, &successor, &subject, now_ms);
            match classify_exact_durable_transition(successor_exact, predecessor_exact) {
                CommitClassification::Committed => {
                    ClassifiedHttpObservationOutcome::RecoveredCommitted(
                        RecoveredCommittedHttpObservation {
                            entry_id: subject.entry_id,
                            attempt_id: subject.attempt_id,
                            observation_digest: subject.observation_digest,
                            disposition: disposition_from_code(subject.observation_disposition),
                            path: subject.path,
                            original_error,
                        },
                    )
                }
                CommitClassification::DefinitelyNotCommitted => {
                    ClassifiedHttpObservationOutcome::DefinitelyNotCommitted(
                        DefinitelyNotCommittedHttpObservation {
                            entry_id: subject.entry_id,
                            attempt_id: subject.attempt_id,
                            path: subject.path,
                            original_error,
                        },
                    )
                }
                CommitClassification::IndeterminateCommit => {
                    ClassifiedHttpObservationOutcome::IndeterminateCommit(
                        IndeterminateHttpObservation {
                            entry_id: subject.entry_id,
                            attempt_id: subject.attempt_id,
                            path: subject.path,
                            original_error,
                            reread_error: None,
                        },
                    )
                }
            }
        }
        Err(reread_error) => {
            ClassifiedHttpObservationOutcome::IndeterminateCommit(IndeterminateHttpObservation {
                entry_id: subject.entry_id,
                attempt_id: subject.attempt_id,
                path: subject.path,
                original_error,
                reread_error: Some(reread_error),
            })
        }
    }
}

fn observation_successor_exact(
    predecessor: &DurableObservationSnapshot,
    successor: &DurableObservationSnapshot,
    subject: &ObservationSubject,
    now_ms: i64,
) -> bool {
    if predecessor.dispatch_binding_digest != successor.dispatch_binding_digest {
        return false;
    }
    let (Some(before), Some(after)) = (predecessor.row.as_ref(), successor.row.as_ref()) else {
        return false;
    };
    let mut expected = before.clone();
    expected.stage = JOURNAL_OBSERVATION_RECORDED;
    expected.observation_digest = Some(subject.observation_digest.to_vec());
    expected.observation_disposition = Some(subject.observation_disposition);
    expected.updated_at_ms = now_ms;
    after == &expected
}

fn read_exact_observation_snapshot(
    store_binding: &QualifiedRuntimeStoreBinding,
    subject: &ObservationSubject,
) -> Result<DurableObservationSnapshot, DurableHttpObservationRereadError> {
    store_binding.revalidate()?;
    let mut conn = Connection::open_with_flags(
        store_binding.path(),
        OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON; PRAGMA query_only = ON;")?;
    require_main_database_path(&conn, store_binding.path())?;

    let tx = conn.transaction_with_behavior(TransactionBehavior::Deferred)?;
    require_exact_journal_schema(&tx)?;
    let row = load_observation_row(&tx, subject)?;
    let dispatch_binding_digest = load_dispatch_binding(&tx, subject)?;
    let snapshot = DurableObservationSnapshot {
        row,
        dispatch_binding_digest,
    };
    tx.commit()?;
    store_binding.revalidate()?;
    Ok(snapshot)
}

fn load_observation_row(
    conn: &Connection,
    subject: &ObservationSubject,
) -> Result<Option<DurableObservationRow>, DurableHttpObservationRereadError> {
    conn.query_row(
        "SELECT stage, command_id, connector_instance, dispatch_binding_digest, issuance_digest,\n\
                request_commitment_algorithm, request_commitment_digest,\n\
                descriptor_valid_until_ms, prepared_at_ms, armed_at_ms,\n\
                observation_digest, observation_disposition, updated_at_ms\n\
         FROM integration_http_transport_attempt_v1\n\
         WHERE entry_id = ?1 AND attempt_id = ?2",
        params![subject.entry_id, subject.attempt_id.as_str()],
        |row| {
            Ok(DurableObservationRow {
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
    .map_err(DurableHttpObservationRereadError::Sqlite)
}

fn load_dispatch_binding(
    conn: &Connection,
    subject: &ObservationSubject,
) -> Result<Option<Vec<u8>>, DurableHttpObservationRereadError> {
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
    .map_err(DurableHttpObservationRereadError::Sqlite)
}

fn require_exact_journal_schema(
    conn: &Connection,
) -> Result<(), DurableHttpObservationRereadError> {
    if !table_exists(conn, JOURNAL_TABLE)? {
        return Err(DurableHttpObservationRereadError::MissingJournalTable);
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
        return Err(DurableHttpObservationRereadError::JournalSchemaMismatch);
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
        return Err(DurableHttpObservationRereadError::JournalSchemaMismatch);
    }

    let trigger_count: i64 = conn.query_row(
        "SELECT COUNT(*) FROM sqlite_master WHERE type = 'trigger' AND tbl_name = ?1",
        params![JOURNAL_TABLE],
        |row| row.get(0),
    )?;
    if trigger_count != 0 {
        return Err(DurableHttpObservationRereadError::JournalSchemaMismatch);
    }
    Ok(())
}

fn table_exists(conn: &Connection, name: &str) -> Result<bool, DurableHttpObservationRereadError> {
    conn.query_row(
        "SELECT EXISTS(SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1)",
        params![name],
        |row| row.get(0),
    )
    .map_err(DurableHttpObservationRereadError::Sqlite)
}

fn require_main_database_path(
    conn: &Connection,
    expected_path: &Path,
) -> Result<(), DurableHttpObservationRereadError> {
    let opened: String = conn.query_row(
        "SELECT file FROM pragma_database_list WHERE name = 'main'",
        [],
        |row| row.get(0),
    )?;
    if opened.is_empty() {
        return Err(DurableHttpObservationRereadError::StoreIdentityChanged);
    }
    let opened = fs::canonicalize(opened).map_err(DurableHttpObservationRereadError::Io)?;
    if opened != expected_path {
        return Err(DurableHttpObservationRereadError::StoreIdentityChanged);
    }
    Ok(())
}

fn digest_algorithm_code(value: mycelix_integration_core::DigestAlgorithm) -> i64 {
    match value {
        mycelix_integration_core::DigestAlgorithm::Sha256 => 1,
    }
}

fn disposition_code(value: TransportObservationDisposition) -> i64 {
    match value {
        TransportObservationDisposition::DefinitelyNotIssued => 1,
        TransportObservationDisposition::CompleteProviderResponse => 2,
        TransportObservationDisposition::AmbiguousPossibleIssue => 3,
    }
}

fn disposition_from_code(value: i64) -> TransportObservationDisposition {
    match value {
        1 => TransportObservationDisposition::DefinitelyNotIssued,
        2 => TransportObservationDisposition::CompleteProviderResponse,
        3 => TransportObservationDisposition::AmbiguousPossibleIssue,
        _ => unreachable!("ObservationSubject stores only canonical disposition codes"),
    }
}

#[derive(Debug, Error)]
pub enum HttpObservationClassificationError {
    #[error("transport observation evidence failed deterministic requalification: {0}")]
    ObservationEvidenceInvalid(TransportObservationError),
    #[error(
        "transport observation disposition/digest/profile differs from deterministic requalification"
    )]
    ObservationIntegrityMismatch,
    #[error("HTTP transport journal row is missing")]
    MissingJournal,
    #[error("context-bound dispatch evidence is missing")]
    MissingContextDispatch,
    #[error("HTTP observation predecessor differs from the consumed capability")]
    PredecessorSubjectMismatch,
    #[error("HTTP observation horizon cannot be represented")]
    TimeOverflow,
    #[error(transparent)]
    Reread(#[from] DurableHttpObservationRereadError),
}

#[derive(Debug, Error)]
pub enum DurableHttpObservationRereadError {
    #[error("HTTP transport journal table is missing")]
    MissingJournalTable,
    #[error("HTTP transport journal schema differs from the v1 structural contract")]
    JournalSchemaMismatch,
    #[error("runtime store identity changed during HTTP observation commit reread")]
    StoreIdentityChanged,
    #[error(transparent)]
    StoreBinding(#[from] RuntimeStoreBindingError),
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("runtime store filesystem error during HTTP observation commit reread: {0}")]
    Io(std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_integration_core::{
        ConnectorInstanceId, ContentCommitment, ExecutionAttemptId, IntegrationCommandId,
    };
    use mycelix_integration_transport_observation::{
        TransportAttemptEvidence, TransportFailureClass, TransportTermination,
    };

    fn observation() -> QualifiedTransportObservation {
        qualify_transport_observation(TransportAttemptEvidence {
            attempt_id: ExecutionAttemptId::new("attempt-1").unwrap(),
            command_id: IntegrationCommandId::new("command-1").unwrap(),
            connector_instance: ConnectorInstanceId::new("connector-1").unwrap(),
            request_commitment: ContentCommitment::sha256(b"request"),
            planned_request_bytes: 7,
            application_bytes_written: 0,
            write_started_at_ms: None,
            write_completed_at_ms: None,
            response_started_at_ms: None,
            observed_at_ms: 20,
            termination: TransportTermination::Failure {
                class: TransportFailureClass::Connect,
                detail_commitment: None,
            },
        })
        .unwrap()
    }

    fn row(stage: i64, armed_at_ms: Option<i64>) -> DurableObservationRow {
        DurableObservationRow {
            stage,
            command_id: "command-1".to_owned(),
            connector_instance: "connector-1".to_owned(),
            dispatch_binding_digest: vec![1; 32],
            issuance_digest: vec![2; 32],
            request_commitment_algorithm: 1,
            request_commitment_digest: vec![3; 32],
            descriptor_valid_until_ms: 500,
            prepared_at_ms: 100,
            armed_at_ms,
            observation_digest: None,
            observation_disposition: None,
            updated_at_ms: armed_at_ms.unwrap_or(100),
        }
    }

    fn subject(path: ObservationWritePath) -> ObservationSubject {
        let (stage, armed) = match path {
            ObservationWritePath::PreparedPrewriteNonissuance => (JOURNAL_PREPARED, None),
            ObservationWritePath::Armed => (JOURNAL_WRITE_MAY_BEGIN, Some(150)),
        };
        ObservationSubject {
            entry_id: 1,
            attempt_id: "attempt-1".to_owned(),
            issuance_digest: [2; 32],
            request_commitment_algorithm: 1,
            request_commitment_digest: [3; 32],
            descriptor_valid_until_ms: 500,
            predecessor_stage: stage,
            predecessor_armed_at_ms: armed,
            observation_digest: [4; 32],
            observation_disposition: 1,
            path,
        }
    }

    fn snapshot(path: ObservationWritePath) -> DurableObservationSnapshot {
        let subject = subject(path);
        DurableObservationSnapshot {
            row: Some(row(
                subject.predecessor_stage,
                subject.predecessor_armed_at_ms,
            )),
            dispatch_binding_digest: Some(vec![1; 32]),
        }
    }

    #[test]
    fn observation_requalification_rejects_forged_disposition() {
        let mut forged = observation();
        forged.disposition = TransportObservationDisposition::AmbiguousPossibleIssue;
        assert!(matches!(
            requalify_observation(&forged),
            Err(HttpObservationClassificationError::ObservationIntegrityMismatch)
        ));
    }

    #[test]
    fn observation_requalification_rejects_forged_digest() {
        let mut forged = observation();
        forged.observation_digest.0[0] ^= 1;
        assert!(matches!(
            requalify_observation(&forged),
            Err(HttpObservationClassificationError::ObservationIntegrityMismatch)
        ));
    }

    #[test]
    fn observation_requalification_rejects_forged_profile() {
        let mut forged = observation();
        forged.observation_profile.push_str("-forged");
        assert!(matches!(
            requalify_observation(&forged),
            Err(HttpObservationClassificationError::ObservationIntegrityMismatch)
        ));
    }

    #[test]
    fn prepared_successor_changes_only_observation_fields() {
        let subject = subject(ObservationWritePath::PreparedPrewriteNonissuance);
        let before = snapshot(subject.path);
        let mut after = before.clone();
        let row = after.row.as_mut().unwrap();
        row.stage = JOURNAL_OBSERVATION_RECORDED;
        row.observation_digest = Some(subject.observation_digest.to_vec());
        row.observation_disposition = Some(subject.observation_disposition);
        row.updated_at_ms = 200;
        assert!(observation_successor_exact(&before, &after, &subject, 200));

        after.row.as_mut().unwrap().command_id.push_str("-tampered");
        assert!(!observation_successor_exact(&before, &after, &subject, 200));
    }

    #[test]
    fn armed_successor_preserves_arm_identity() {
        let subject = subject(ObservationWritePath::Armed);
        let before = snapshot(subject.path);
        let mut after = before.clone();
        let row = after.row.as_mut().unwrap();
        row.stage = JOURNAL_OBSERVATION_RECORDED;
        row.observation_digest = Some(subject.observation_digest.to_vec());
        row.observation_disposition = Some(subject.observation_disposition);
        row.updated_at_ms = 200;
        assert!(observation_successor_exact(&before, &after, &subject, 200));

        after.row.as_mut().unwrap().armed_at_ms = Some(151);
        assert!(!observation_successor_exact(&before, &after, &subject, 200));
    }
}

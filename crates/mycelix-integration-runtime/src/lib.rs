//! Durable local reference runtime for the Mycelix integration plane.
//!
//! The runtime owns causal durability. It does not decide institutional truth,
//! mint authority, or call provider APIs. In particular, inserting an outbound
//! intent or claiming work is not itself an execution capability; INT-04 remains
//! responsible for fresh execution qualification and payload materialization.

use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, DigestAlgorithm, ExecutionAttemptId,
    ExternalExecutionOutcome, ExternalOperationRef, IdempotencyKey, InboundStage,
    IntegrationCommandId, IntegrationEventId, OutboundStage, ReconcileCursor,
    ReconciliationHint, ReconciliationResult, ReconciliationStrategy, SideEffectClass,
};
use rusqlite::{params, Connection, OptionalExtension, Transaction, TransactionBehavior};
use std::{path::Path, time::Duration};
use thiserror::Error;

const RUNTIME_SCHEMA_VERSION: i64 = 2;
const MAX_WORKER_ID_BYTES: usize = 256;
const MAX_OUTBOX_CLAIM: usize = 1024;
const MAX_INBOUND_PAYLOAD_BYTES: usize = 1024 * 1024;
const MAX_COMMAND_BYTES: usize = 1024 * 1024;
const MAX_SERIALIZED_OUTCOME_BYTES: usize = 256 * 1024;
const MAX_SERIALIZED_RECONCILIATION_BYTES: usize = 256 * 1024;
const MAX_RECONCILIATION_HISTORY_PER_ENTRY: i64 = 4096;
const MAX_EXECUTION_OBSERVATIONS_PER_ENTRY: i64 = 4096;

#[derive(Debug, Error)]
pub enum RuntimeError {
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("worker id is empty")]
    EmptyWorkerId,
    #[error("worker id exceeds {MAX_WORKER_ID_BYTES} bytes")]
    WorkerIdTooLong,
    #[error("worker id contains a control character")]
    WorkerIdContainsControl,
    #[error("claim limit must be between 1 and {MAX_OUTBOX_CLAIM}")]
    InvalidClaimLimit,
    #[error("lease duration must be positive")]
    InvalidLeaseDuration,
    #[error("lease deadline overflows i64 milliseconds")]
    LeaseDeadlineOverflow,
    #[error("timestamp must be non-negative milliseconds")]
    InvalidTimestamp,
    #[error("inbound normalized payload exceeds {MAX_INBOUND_PAYLOAD_BYTES} bytes")]
    InboundPayloadTooLarge,
    #[error("outbound command bytes must not be empty")]
    EmptyCommandBytes,
    #[error("outbound command exceeds {MAX_COMMAND_BYTES} bytes")]
    CommandTooLarge,
    #[error("serialized execution outcome exceeds {MAX_SERIALIZED_OUTCOME_BYTES} bytes")]
    OutcomeTooLarge,
    #[error("serialized reconciliation result exceeds {MAX_SERIALIZED_RECONCILIATION_BYTES} bytes")]
    ReconciliationTooLarge,
    #[error("reconciliation history for outbox entry {entry_id} reached its v0.1 bound")]
    ReconciliationHistoryLimit { entry_id: i64 },
    #[error("execution observation history for outbox entry {entry_id} reached its v0.1 bound")]
    ExecutionObservationHistoryLimit { entry_id: i64 },
    #[error("inbound event identity collision for {connector_instance}:{event_id}")]
    InboundIdentityCollision {
        connector_instance: String,
        event_id: String,
    },
    #[error("same inbound event/content produced different normalized bytes for {connector_instance}:{event_id}")]
    InboundNormalizationCollision {
        connector_instance: String,
        event_id: String,
    },
    #[error("outbox command identity collision for {command_id}")]
    OutboxIdentityCollision { command_id: String },
    #[error("outbox entry {entry_id} was not found")]
    UnknownOutboxEntry { entry_id: i64 },
    #[error("outbox entry {entry_id} is in {actual:?}, expected {expected:?}")]
    UnexpectedOutboundStage {
        entry_id: i64,
        expected: OutboundStage,
        actual: OutboundStage,
    },
    #[error("outbox entry {entry_id} is leased by another worker")]
    LeaseOwnerMismatch { entry_id: i64 },
    #[error("outbox entry {entry_id} attempt does not match the supplied fence")]
    AttemptFenceMismatch { entry_id: i64 },
    #[error("outbox entry {entry_id} lease expired before dispatch")]
    LeaseExpired { entry_id: i64 },
    #[error("outcome operation does not match outbox entry {entry_id}")]
    OutcomeOperationMismatch { entry_id: i64 },
    #[error("reconciliation operation does not match outbox entry {entry_id}")]
    ReconciliationOperationMismatch { entry_id: i64 },
    #[error("invalid stored enum value for {field}: {value}")]
    InvalidStoredEnum { field: &'static str, value: i64 },
    #[error("stored commitment has invalid length {actual}, expected 32")]
    InvalidStoredDigestLength { actual: usize },
    #[error("stored integration identifier violates core invariants: {0}")]
    StoredIdentifier(String),
    #[error("illegal outbound transition {from:?} -> {to:?}")]
    IllegalOutboundTransition {
        from: OutboundStage,
        to: OutboundStage,
    },
    #[error("unsupported integration runtime schema version {actual}; expected {expected}")]
    UnsupportedRuntimeSchema { actual: i64, expected: i64 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PersistableInbound {
    pub event_id: IntegrationEventId,
    pub connector_instance: ConnectorInstanceId,
    pub event_commitment: ContentCommitment,
    pub normalized_payload: Vec<u8>,
    pub received_at_ms: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertDisposition {
    Inserted,
    Duplicate,
}

/// Durable outbound material produced after a higher layer has evaluated the
/// exact operation and bound its authority evidence.
///
/// `authority_commitment` is provenance, not a self-authenticating permission.
/// This crate deliberately exposes no provider execution API.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DurableOutboundIntent {
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub command_commitment: ContentCommitment,
    pub authority_commitment: ContentCommitment,
    pub side_effect_class: SideEffectClass,
    pub idempotency_key: Option<IdempotencyKey>,
    pub command_bytes: Vec<u8>,
    pub created_at_ms: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnqueueDisposition {
    Inserted(i64),
    AlreadyPresent(i64),
}

/// Durable claim metadata only. It intentionally does not expose command bytes
/// or turn the persisted authority commitment into a live capability.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecutionClaim {
    pub entry_id: i64,
    pub attempt_id: ExecutionAttemptId,
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub command_commitment: ContentCommitment,
    pub side_effect_class: SideEffectClass,
    pub idempotency_key: Option<IdempotencyKey>,
    pub attempt_count: u32,
    pub lease_until_ms: i64,
}

/// Proof that the durable runtime crossed the local pre-dispatch boundary for
/// one exact attempt. It still is not provider authority or proof of commit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DispatchStarted {
    pub entry_id: i64,
    pub attempt_id: ExecutionAttemptId,
    pub operation: ExternalOperationRef,
    pub started_at_ms: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionRecordDisposition {
    AppliedToCurrentAttempt,
    RecordedForStaleAttempt,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutboxSnapshot {
    pub entry_id: i64,
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub stage: OutboundStage,
    pub side_effect_class: SideEffectClass,
    pub attempt_count: u32,
    pub current_attempt_id: Option<ExecutionAttemptId>,
    pub worker_id: Option<String>,
    pub lease_until_ms: Option<i64>,
    pub dispatch_started_at_ms: Option<i64>,
    pub outcome: Option<ExternalExecutionOutcome>,
    pub reconciliation: Option<ReconciliationResult>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RecoverySummary {
    /// Prepared attempts that never crossed dispatch can safely return to queue.
    pub pre_dispatch_requeued: usize,
    /// Expired post-dispatch attempts become ambiguous until reconciled.
    pub post_dispatch_marked_ambiguous: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReconciliationHistoryItem {
    pub sequence: i64,
    pub result: ReconciliationResult,
}

pub struct SqliteIntegrationStore {
    conn: Connection,
}

impl SqliteIntegrationStore {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RuntimeError> {
        let conn = Connection::open(path)?;
        Self::from_connection(conn)
    }

    pub fn in_memory() -> Result<Self, RuntimeError> {
        let conn = Connection::open_in_memory()?;
        Self::from_connection(conn)
    }

    fn from_connection(conn: Connection) -> Result<Self, RuntimeError> {
        conn.busy_timeout(Duration::from_secs(5))?;
        conn.execute_batch(
            "PRAGMA foreign_keys = ON;\n\
             PRAGMA synchronous = FULL;",
        )?;
        conn.pragma_update(None, "journal_mode", "WAL")?;

        let mut store = Self { conn };
        store.initialize_or_migrate_schema()?;
        Ok(store)
    }

    fn initialize_or_migrate_schema(&mut self) -> Result<(), RuntimeError> {
        let version: i64 = self
            .conn
            .pragma_query_value(None, "user_version", |row| row.get(0))?;
        let has_outbox = table_exists(&self.conn, "integration_outbox")?;

        match (version, has_outbox) {
            (0, false) => self.create_schema_v2(),
            (0, true) => self.migrate_implicit_v1_to_v2(),
            (RUNTIME_SCHEMA_VERSION, _) => {
                self.create_schema_v2()?;
                Ok(())
            }
            (actual, _) => Err(RuntimeError::UnsupportedRuntimeSchema {
                actual,
                expected: RUNTIME_SCHEMA_VERSION,
            }),
        }
    }

    fn create_schema_v2(&self) -> Result<(), RuntimeError> {
        self.conn.execute_batch(
            r#"
            CREATE TABLE IF NOT EXISTS integration_inbound (
                connector_instance TEXT NOT NULL,
                event_id TEXT NOT NULL,
                commitment_algorithm INTEGER NOT NULL,
                commitment_digest BLOB NOT NULL,
                stage INTEGER NOT NULL,
                normalized_payload BLOB NOT NULL,
                received_at_ms INTEGER NOT NULL,
                PRIMARY KEY (connector_instance, event_id)
            );

            CREATE TABLE IF NOT EXISTS integration_outbox (
                entry_id INTEGER PRIMARY KEY AUTOINCREMENT,
                command_id TEXT NOT NULL UNIQUE,
                connector_instance TEXT NOT NULL,
                command_commitment_algorithm INTEGER NOT NULL,
                command_commitment_digest BLOB NOT NULL,
                authority_commitment_algorithm INTEGER NOT NULL,
                authority_commitment_digest BLOB NOT NULL,
                side_effect_class INTEGER NOT NULL,
                idempotency_key TEXT,
                command_bytes BLOB NOT NULL,
                stage INTEGER NOT NULL,
                worker_id TEXT,
                lease_until_ms INTEGER,
                attempt_count INTEGER NOT NULL DEFAULT 0,
                current_attempt_id TEXT,
                dispatch_started_at_ms INTEGER,
                outcome_json BLOB,
                reconciliation_json BLOB,
                created_at_ms INTEGER NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );

            CREATE INDEX IF NOT EXISTS integration_outbox_claim_idx
                ON integration_outbox(stage, entry_id);

            CREATE INDEX IF NOT EXISTS integration_outbox_lease_idx
                ON integration_outbox(stage, lease_until_ms);

            CREATE TABLE IF NOT EXISTS integration_execution_observation (
                observation_id INTEGER PRIMARY KEY AUTOINCREMENT,
                entry_id INTEGER NOT NULL,
                attempt_id TEXT NOT NULL,
                outcome_json BLOB NOT NULL,
                observed_at_ms INTEGER NOT NULL,
                applied_to_current INTEGER NOT NULL,
                FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
            );

            CREATE INDEX IF NOT EXISTS integration_execution_observation_idx
                ON integration_execution_observation(entry_id, observation_id);

            CREATE TABLE IF NOT EXISTS integration_reconciliation_history (
                sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                entry_id INTEGER NOT NULL,
                result_json BLOB NOT NULL,
                recorded_at_ms INTEGER NOT NULL,
                FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
            );

            CREATE INDEX IF NOT EXISTS integration_reconciliation_history_idx
                ON integration_reconciliation_history(entry_id, sequence);

            CREATE TABLE IF NOT EXISTS integration_reconcile_checkpoint (
                connector_instance TEXT PRIMARY KEY,
                cursor TEXT NOT NULL,
                commitment_algorithm INTEGER NOT NULL,
                commitment_digest BLOB NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );

            PRAGMA user_version = 2;
            "#,
        )?;
        Ok(())
    }

    fn migrate_implicit_v1_to_v2(&mut self) -> Result<(), RuntimeError> {
        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;

        tx.execute_batch(
            r#"
            ALTER TABLE integration_outbox ADD COLUMN current_attempt_id TEXT;
            ALTER TABLE integration_outbox ADD COLUMN dispatch_started_at_ms INTEGER;

            CREATE TABLE IF NOT EXISTS integration_execution_observation (
                observation_id INTEGER PRIMARY KEY AUTOINCREMENT,
                entry_id INTEGER NOT NULL,
                attempt_id TEXT NOT NULL,
                outcome_json BLOB NOT NULL,
                observed_at_ms INTEGER NOT NULL,
                applied_to_current INTEGER NOT NULL,
                FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
            );

            CREATE INDEX IF NOT EXISTS integration_execution_observation_idx
                ON integration_execution_observation(entry_id, observation_id);

            CREATE TABLE IF NOT EXISTS integration_reconciliation_history (
                sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                entry_id INTEGER NOT NULL,
                result_json BLOB NOT NULL,
                recorded_at_ms INTEGER NOT NULL,
                FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
            );

            CREATE INDEX IF NOT EXISTS integration_reconciliation_history_idx
                ON integration_reconciliation_history(entry_id, sequence);
            "#,
        )?;

        tx.execute(
            "UPDATE integration_outbox\n\
             SET worker_id = NULL, lease_until_ms = NULL\n\
             WHERE stage = 4",
            [],
        )?;
        tx.execute(
            "UPDATE integration_outbox\n\
             SET stage = CASE stage\n\
                 WHEN 0 THEN 0\n\
                 WHEN 1 THEN 1\n\
                 WHEN 2 THEN 2\n\
                 WHEN 3 THEN 3\n\
                 WHEN 4 THEN 8\n\
                 WHEN 5 THEN 6\n\
                 WHEN 6 THEN 7\n\
                 WHEN 7 THEN 8\n\
                 WHEN 8 THEN 9\n\
                 WHEN 9 THEN 10\n\
                 ELSE stage END",
            [],
        )?;
        tx.pragma_update(None, "user_version", RUNTIME_SCHEMA_VERSION)?;
        tx.commit()?;
        self.create_schema_v2()?;
        Ok(())
    }

    pub fn insert_inbound(
        &mut self,
        inbound: &PersistableInbound,
    ) -> Result<InsertDisposition, RuntimeError> {
        validate_timestamp(inbound.received_at_ms)?;
        if inbound.normalized_payload.len() > MAX_INBOUND_PAYLOAD_BYTES {
            return Err(RuntimeError::InboundPayloadTooLarge);
        }

        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        let key_connector = inbound.connector_instance.as_str();

        let existing: Option<(i64, Vec<u8>, Vec<u8>)> = tx
            .query_row(
                "SELECT commitment_algorithm, commitment_digest, normalized_payload\n\
                 FROM integration_inbound\n\
                 WHERE connector_instance = ?1 AND event_id = ?2",
                params![key_connector, inbound.event_id.as_str()],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
            )
            .optional()?;

        if let Some((algorithm, digest, normalized_payload)) = existing {
            let existing = commitment_from_parts(algorithm, digest)?;
            if existing != inbound.event_commitment {
                return Err(RuntimeError::InboundIdentityCollision {
                    connector_instance: key_connector.to_owned(),
                    event_id: inbound.event_id.to_string(),
                });
            }
            if normalized_payload != inbound.normalized_payload {
                return Err(RuntimeError::InboundNormalizationCollision {
                    connector_instance: key_connector.to_owned(),
                    event_id: inbound.event_id.to_string(),
                });
            }
            tx.commit()?;
            return Ok(InsertDisposition::Duplicate);
        }

        tx.execute(
            "INSERT INTO integration_inbound (\n\
                connector_instance, event_id, commitment_algorithm, commitment_digest,\n\
                stage, normalized_payload, received_at_ms\n\
             ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
            params![
                key_connector,
                inbound.event_id.as_str(),
                digest_algorithm_to_i64(inbound.event_commitment.algorithm),
                inbound.event_commitment.digest.as_slice(),
                inbound_stage_to_i64(InboundStage::Persisted),
                inbound.normalized_payload.as_slice(),
                inbound.received_at_ms,
            ],
        )?;
        tx.commit()?;
        Ok(InsertDisposition::Inserted)
    }

    pub fn enqueue_outbound(
        &mut self,
        intent: &DurableOutboundIntent,
    ) -> Result<EnqueueDisposition, RuntimeError> {
        validate_timestamp(intent.created_at_ms)?;
        if intent.command_bytes.is_empty() {
            return Err(RuntimeError::EmptyCommandBytes);
        }
        if intent.command_bytes.len() > MAX_COMMAND_BYTES {
            return Err(RuntimeError::CommandTooLarge);
        }

        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;

        #[allow(clippy::type_complexity)]
        let existing: Option<(
            i64,
            String,
            i64,
            Vec<u8>,
            i64,
            Vec<u8>,
            i64,
            Option<String>,
            Vec<u8>,
        )> = tx
            .query_row(
                "SELECT entry_id, connector_instance,\n\
                        command_commitment_algorithm, command_commitment_digest,\n\
                        authority_commitment_algorithm, authority_commitment_digest,\n\
                        side_effect_class, idempotency_key, command_bytes\n\
                 FROM integration_outbox WHERE command_id = ?1",
                params![intent.command_id.as_str()],
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
                    ))
                },
            )
            .optional()?;

        if let Some((
            entry_id,
            connector_instance,
            command_alg,
            command_digest,
            authority_alg,
            authority_digest,
            side_effect,
            idempotency_key,
            command_bytes,
        )) = existing
        {
            let same = connector_instance == intent.connector_instance.as_str()
                && commitment_from_parts(command_alg, command_digest)? == intent.command_commitment
                && commitment_from_parts(authority_alg, authority_digest)?
                    == intent.authority_commitment
                && side_effect_from_i64(side_effect)? == intent.side_effect_class
                && idempotency_key.as_deref()
                    == intent.idempotency_key.as_ref().map(IdempotencyKey::as_str)
                && command_bytes == intent.command_bytes;

            if same {
                tx.commit()?;
                return Ok(EnqueueDisposition::AlreadyPresent(entry_id));
            }
            return Err(RuntimeError::OutboxIdentityCollision {
                command_id: intent.command_id.to_string(),
            });
        }

        tx.execute(
            "INSERT INTO integration_outbox (\n\
                command_id, connector_instance,\n\
                command_commitment_algorithm, command_commitment_digest,\n\
                authority_commitment_algorithm, authority_commitment_digest,\n\
                side_effect_class, idempotency_key, command_bytes, stage,\n\
                created_at_ms, updated_at_ms\n\
             ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?11)",
            params![
                intent.command_id.as_str(),
                intent.connector_instance.as_str(),
                digest_algorithm_to_i64(intent.command_commitment.algorithm),
                intent.command_commitment.digest.as_slice(),
                digest_algorithm_to_i64(intent.authority_commitment.algorithm),
                intent.authority_commitment.digest.as_slice(),
                side_effect_to_i64(intent.side_effect_class),
                intent.idempotency_key.as_ref().map(IdempotencyKey::as_str),
                intent.command_bytes.as_slice(),
                outbound_stage_to_i64(OutboundStage::OutboxCommitted),
                intent.created_at_ms,
            ],
        )?;
        let entry_id = tx.last_insert_rowid();
        tx.commit()?;
        Ok(EnqueueDisposition::Inserted(entry_id))
    }

    pub fn claim_outbox(
        &mut self,
        worker_id: &str,
        now_ms: i64,
        lease_duration_ms: i64,
        limit: usize,
    ) -> Result<Vec<ExecutionClaim>, RuntimeError> {
        validate_worker_id(worker_id)?;
        validate_timestamp(now_ms)?;
        if lease_duration_ms <= 0 {
            return Err(RuntimeError::InvalidLeaseDuration);
        }
        if !(1..=MAX_OUTBOX_CLAIM).contains(&limit) {
            return Err(RuntimeError::InvalidClaimLimit);
        }
        let lease_until_ms = now_ms
            .checked_add(lease_duration_ms)
            .ok_or(RuntimeError::LeaseDeadlineOverflow)?;

        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        recover_expired_claims_in_tx(&tx, now_ms)?;

        let ids = {
            let mut statement = tx.prepare(
                "SELECT entry_id FROM integration_outbox\n\
                 WHERE stage = ?1\n\
                 ORDER BY entry_id ASC LIMIT ?2",
            )?;
            let rows = statement.query_map(
                params![
                    outbound_stage_to_i64(OutboundStage::OutboxCommitted),
                    limit as i64
                ],
                |row| row.get::<_, i64>(0),
            )?;
            rows.collect::<Result<Vec<_>, _>>()?
        };

        let mut claimed = Vec::with_capacity(ids.len());
        for entry_id in ids {
            let previous_attempt_count: i64 = tx.query_row(
                "SELECT attempt_count FROM integration_outbox WHERE entry_id = ?1",
                params![entry_id],
                |row| row.get(0),
            )?;
            let next_attempt_count = previous_attempt_count
                .checked_add(1)
                .ok_or(RuntimeError::InvalidStoredEnum {
                    field: "attempt_count",
                    value: previous_attempt_count,
                })?;
            let attempt_id = ExecutionAttemptId::new(format!("{entry_id}:{next_attempt_count}"))
                .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;

            let changed = tx.execute(
                "UPDATE integration_outbox\n\
                 SET stage = ?1, worker_id = ?2, lease_until_ms = ?3,\n\
                     attempt_count = ?4, current_attempt_id = ?5,\n\
                     dispatch_started_at_ms = NULL, updated_at_ms = ?6\n\
                 WHERE entry_id = ?7 AND stage = ?8",
                params![
                    outbound_stage_to_i64(OutboundStage::AttemptPrepared),
                    worker_id,
                    lease_until_ms,
                    next_attempt_count,
                    attempt_id.as_str(),
                    now_ms,
                    entry_id,
                    outbound_stage_to_i64(OutboundStage::OutboxCommitted),
                ],
            )?;
            if changed != 1 {
                return Err(RuntimeError::UnexpectedOutboundStage {
                    entry_id,
                    expected: OutboundStage::OutboxCommitted,
                    actual: load_outbound_stage(&tx, entry_id)?,
                });
            }
            claimed.push(load_execution_claim(&tx, entry_id, attempt_id, lease_until_ms)?);
        }

        tx.commit()?;
        Ok(claimed)
    }

    pub fn mark_dispatch_started(
        &mut self,
        entry_id: i64,
        attempt_id: &ExecutionAttemptId,
        worker_id: &str,
        now_ms: i64,
    ) -> Result<DispatchStarted, RuntimeError> {
        validate_worker_id(worker_id)?;
        validate_timestamp(now_ms)?;
        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;

        let (stage, owner, stored_attempt, lease_until, command_id, connector_instance): (
            i64,
            Option<String>,
            Option<String>,
            Option<i64>,
            String,
            String,
        ) = tx
            .query_row(
                "SELECT stage, worker_id, current_attempt_id, lease_until_ms, command_id, connector_instance\n\
                 FROM integration_outbox WHERE entry_id = ?1",
                params![entry_id],
                |row| {
                    Ok((
                        row.get(0)?,
                        row.get(1)?,
                        row.get(2)?,
                        row.get(3)?,
                        row.get(4)?,
                        row.get(5)?,
                    ))
                },
            )
            .optional()?
            .ok_or(RuntimeError::UnknownOutboxEntry { entry_id })?;

        let stage = outbound_stage_from_i64(stage)?;
        if stage != OutboundStage::AttemptPrepared {
            return Err(RuntimeError::UnexpectedOutboundStage {
                entry_id,
                expected: OutboundStage::AttemptPrepared,
                actual: stage,
            });
        }
        require_attempt_owner(entry_id, worker_id, attempt_id, owner, stored_attempt)?;
        if lease_until.is_none_or(|deadline| now_ms >= deadline) {
            return Err(RuntimeError::LeaseExpired { entry_id });
        }

        let command_id = IntegrationCommandId::new(command_id)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;
        let connector_instance = ConnectorInstanceId::new(connector_instance)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;

        require_outbound_transition(stage, OutboundStage::DispatchStarted)?;
        tx.execute(
            "UPDATE integration_outbox\n\
             SET stage = ?1, dispatch_started_at_ms = ?2, updated_at_ms = ?2\n\
             WHERE entry_id = ?3 AND stage = ?4 AND current_attempt_id = ?5",
            params![
                outbound_stage_to_i64(OutboundStage::DispatchStarted),
                now_ms,
                entry_id,
                outbound_stage_to_i64(OutboundStage::AttemptPrepared),
                attempt_id.as_str(),
            ],
        )?;
        tx.commit()?;

        Ok(DispatchStarted {
            entry_id,
            attempt_id: attempt_id.clone(),
            operation: ExternalOperationRef {
                command_id,
                connector_instance,
                provider_operation: None,
            },
            started_at_ms: now_ms,
        })
    }

    pub fn recover_expired_claims(
        &mut self,
        now_ms: i64,
    ) -> Result<RecoverySummary, RuntimeError> {
        validate_timestamp(now_ms)?;
        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        let summary = recover_expired_claims_in_tx(&tx, now_ms)?;
        tx.commit()?;
        Ok(summary)
    }

    pub fn record_execution(
        &mut self,
        entry_id: i64,
        attempt_id: &ExecutionAttemptId,
        worker_id: &str,
        outcome: &ExternalExecutionOutcome,
        now_ms: i64,
    ) -> Result<ExecutionRecordDisposition, RuntimeError> {
        validate_worker_id(worker_id)?;
        validate_timestamp(now_ms)?;
        let outcome_json = serde_json::to_vec(outcome)?;
        if outcome_json.len() > MAX_SERIALIZED_OUTCOME_BYTES {
            return Err(RuntimeError::OutcomeTooLarge);
        }

        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;

        let (stage, owner, stored_attempt, command_id, connector_instance): (
            i64,
            Option<String>,
            Option<String>,
            String,
            String,
        ) = tx
            .query_row(
                "SELECT stage, worker_id, current_attempt_id, command_id, connector_instance\n\
                 FROM integration_outbox WHERE entry_id = ?1",
                params![entry_id],
                |row| {
                    Ok((
                        row.get(0)?,
                        row.get(1)?,
                        row.get(2)?,
                        row.get(3)?,
                        row.get(4)?,
                    ))
                },
            )
            .optional()?
            .ok_or(RuntimeError::UnknownOutboxEntry { entry_id })?;
        let stage = outbound_stage_from_i64(stage)?;

        let operation = outcome.operation();
        if operation.command_id.as_str() != command_id
            || operation.connector_instance.as_str() != connector_instance
        {
            return Err(RuntimeError::OutcomeOperationMismatch { entry_id });
        }

        let is_current_dispatch = stage == OutboundStage::DispatchStarted
            && owner.as_deref() == Some(worker_id)
            && stored_attempt.as_deref() == Some(attempt_id.as_str());
        let is_late_same_attempt = stage == OutboundStage::Ambiguous
            && stored_attempt.as_deref() == Some(attempt_id.as_str());

        if !is_current_dispatch && !is_late_same_attempt {
            if stored_attempt.as_deref() != Some(attempt_id.as_str()) {
                return Err(RuntimeError::AttemptFenceMismatch { entry_id });
            }
            if owner.as_deref().is_some_and(|owner| owner != worker_id) {
                return Err(RuntimeError::LeaseOwnerMismatch { entry_id });
            }
            return Err(RuntimeError::UnexpectedOutboundStage {
                entry_id,
                expected: OutboundStage::DispatchStarted,
                actual: stage,
            });
        }

        let observation_count: i64 = tx.query_row(
            "SELECT COUNT(*) FROM integration_execution_observation WHERE entry_id = ?1",
            params![entry_id],
            |row| row.get(0),
        )?;
        if observation_count >= MAX_EXECUTION_OBSERVATIONS_PER_ENTRY {
            return Err(RuntimeError::ExecutionObservationHistoryLimit { entry_id });
        }

        tx.execute(
            "INSERT INTO integration_execution_observation (\n\
                entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\n\
             ) VALUES (?1, ?2, ?3, ?4, ?5)",
            params![
                entry_id,
                attempt_id.as_str(),
                outcome_json.as_slice(),
                now_ms,
                if is_current_dispatch { 1 } else { 0 },
            ],
        )?;

        if is_late_same_attempt {
            tx.commit()?;
            return Ok(ExecutionRecordDisposition::RecordedForStaleAttempt);
        }

        let next = match outcome {
            ExternalExecutionOutcome::Confirmed(_) => OutboundStage::Confirmed,
            ExternalExecutionOutcome::Rejected { .. } => OutboundStage::Rejected,
            ExternalExecutionOutcome::Ambiguous { .. } => OutboundStage::Ambiguous,
        };
        require_outbound_transition(stage, next)?;
        tx.execute(
            "UPDATE integration_outbox\n\
             SET stage = ?1, worker_id = NULL, lease_until_ms = NULL,\n\
                 outcome_json = ?2, updated_at_ms = ?3\n\
             WHERE entry_id = ?4 AND current_attempt_id = ?5",
            params![
                outbound_stage_to_i64(next),
                outcome_json.as_slice(),
                now_ms,
                entry_id,
                attempt_id.as_str(),
            ],
        )?;
        tx.commit()?;
        Ok(ExecutionRecordDisposition::AppliedToCurrentAttempt)
    }

    pub fn record_reconciliation(
        &mut self,
        entry_id: i64,
        reconciliation: &ReconciliationResult,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
        validate_timestamp(now_ms)?;
        let reconciliation_json = serde_json::to_vec(reconciliation)?;
        if reconciliation_json.len() > MAX_SERIALIZED_RECONCILIATION_BYTES {
            return Err(RuntimeError::ReconciliationTooLarge);
        }

        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        let (stage, command_id, connector_instance): (i64, String, String) = tx
            .query_row(
                "SELECT stage, command_id, connector_instance\n\
                 FROM integration_outbox WHERE entry_id = ?1",
                params![entry_id],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
            )
            .optional()?
            .ok_or(RuntimeError::UnknownOutboxEntry { entry_id })?;
        let stage = outbound_stage_from_i64(stage)?;

        if reconciliation.operation.command_id.as_str() != command_id
            || reconciliation.operation.connector_instance.as_str() != connector_instance
        {
            return Err(RuntimeError::ReconciliationOperationMismatch { entry_id });
        }

        let reconciliation_count: i64 = tx.query_row(
            "SELECT COUNT(*) FROM integration_reconciliation_history WHERE entry_id = ?1",
            params![entry_id],
            |row| row.get(0),
        )?;
        if reconciliation_count >= MAX_RECONCILIATION_HISTORY_PER_ENTRY {
            return Err(RuntimeError::ReconciliationHistoryLimit { entry_id });
        }

        tx.execute(
            "INSERT INTO integration_reconciliation_history (\n\
                entry_id, result_json, recorded_at_ms\n\
             ) VALUES (?1, ?2, ?3)",
            params![entry_id, reconciliation_json.as_slice(), now_ms],
        )?;

        if !reconciliation.is_conclusive() {
            if stage != OutboundStage::Ambiguous {
                return Err(RuntimeError::UnexpectedOutboundStage {
                    entry_id,
                    expected: OutboundStage::Ambiguous,
                    actual: stage,
                });
            }
            tx.execute(
                "UPDATE integration_outbox\n\
                 SET reconciliation_json = ?1, updated_at_ms = ?2\n\
                 WHERE entry_id = ?3",
                params![reconciliation_json.as_slice(), now_ms, entry_id],
            )?;
            tx.commit()?;
            return Ok(());
        }

        require_outbound_transition(stage, OutboundStage::Reconciled)?;
        tx.execute(
            "UPDATE integration_outbox\n\
             SET stage = ?1, reconciliation_json = ?2, updated_at_ms = ?3\n\
             WHERE entry_id = ?4",
            params![
                outbound_stage_to_i64(OutboundStage::Reconciled),
                reconciliation_json.as_slice(),
                now_ms,
                entry_id,
            ],
        )?;
        tx.commit()?;
        Ok(())
    }

    pub fn reconciliation_history(
        &self,
        entry_id: i64,
    ) -> Result<Vec<ReconciliationHistoryItem>, RuntimeError> {
        if !outbox_exists(&self.conn, entry_id)? {
            return Err(RuntimeError::UnknownOutboxEntry { entry_id });
        }
        let mut statement = self.conn.prepare(
            "SELECT sequence, result_json\n\
             FROM integration_reconciliation_history\n\
             WHERE entry_id = ?1 ORDER BY sequence ASC",
        )?;
        let rows = statement.query_map(params![entry_id], |row| {
            Ok((row.get::<_, i64>(0)?, row.get::<_, Vec<u8>>(1)?))
        })?;
        let mut history = Vec::new();
        for row in rows {
            let (sequence, bytes) = row?;
            history.push(ReconciliationHistoryItem {
                sequence,
                result: serde_json::from_slice(&bytes)?,
            });
        }
        Ok(history)
    }

    pub fn finalize_outbound(
        &mut self,
        entry_id: i64,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
        validate_timestamp(now_ms)?;
        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        let stage = load_outbound_stage(&tx, entry_id)?;
        require_outbound_transition(stage, OutboundStage::Finalized)?;
        tx.execute(
            "UPDATE integration_outbox SET stage = ?1, updated_at_ms = ?2 WHERE entry_id = ?3",
            params![
                outbound_stage_to_i64(OutboundStage::Finalized),
                now_ms,
                entry_id
            ],
        )?;
        tx.commit()?;
        Ok(())
    }

    pub fn checkpoint_reconciliation(
        &mut self,
        cursor: &ReconcileCursor,
        updated_at_ms: i64,
    ) -> Result<(), RuntimeError> {
        validate_timestamp(updated_at_ms)?;
        self.conn.execute(
            "INSERT INTO integration_reconcile_checkpoint (\n\
                connector_instance, cursor, commitment_algorithm, commitment_digest, updated_at_ms\n\
             ) VALUES (?1, ?2, ?3, ?4, ?5)\n\
             ON CONFLICT(connector_instance) DO UPDATE SET\n\
                cursor = excluded.cursor,\n\
                commitment_algorithm = excluded.commitment_algorithm,\n\
                commitment_digest = excluded.commitment_digest,\n\
                updated_at_ms = excluded.updated_at_ms",
            params![
                cursor.connector_instance.as_str(),
                cursor.cursor.as_str(),
                digest_algorithm_to_i64(cursor.checkpoint_commitment.algorithm),
                cursor.checkpoint_commitment.digest.as_slice(),
                updated_at_ms,
            ],
        )?;
        Ok(())
    }

    pub fn load_reconciliation_checkpoint(
        &self,
        connector_instance: &ConnectorInstanceId,
    ) -> Result<Option<ReconcileCursor>, RuntimeError> {
        let stored: Option<(String, i64, Vec<u8>)> = self
            .conn
            .query_row(
                "SELECT cursor, commitment_algorithm, commitment_digest\n\
                 FROM integration_reconcile_checkpoint WHERE connector_instance = ?1",
                params![connector_instance.as_str()],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
            )
            .optional()?;

        stored
            .map(|(cursor, algorithm, digest)| {
                Ok(ReconcileCursor {
                    connector_instance: connector_instance.clone(),
                    cursor: mycelix_integration_core::ExternalOpaqueId::new(cursor)
                        .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
                    checkpoint_commitment: commitment_from_parts(algorithm, digest)?,
                })
            })
            .transpose()
    }

    pub fn outbox_snapshot(&self, entry_id: i64) -> Result<OutboxSnapshot, RuntimeError> {
        self.conn
            .query_row(
                "SELECT command_id, connector_instance, stage, side_effect_class,\n\
                        attempt_count, current_attempt_id, worker_id, lease_until_ms,\n\
                        dispatch_started_at_ms, outcome_json, reconciliation_json\n\
                 FROM integration_outbox WHERE entry_id = ?1",
                params![entry_id],
                |row| {
                    Ok((
                        row.get::<_, String>(0)?,
                        row.get::<_, String>(1)?,
                        row.get::<_, i64>(2)?,
                        row.get::<_, i64>(3)?,
                        row.get::<_, i64>(4)?,
                        row.get::<_, Option<String>>(5)?,
                        row.get::<_, Option<String>>(6)?,
                        row.get::<_, Option<i64>>(7)?,
                        row.get::<_, Option<i64>>(8)?,
                        row.get::<_, Option<Vec<u8>>>(9)?,
                        row.get::<_, Option<Vec<u8>>>(10)?,
                    ))
                },
            )
            .optional()?
            .ok_or(RuntimeError::UnknownOutboxEntry { entry_id })
            .and_then(
                |(
                    command_id,
                    connector_instance,
                    stage,
                    side_effect,
                    attempt_count,
                    current_attempt_id,
                    worker_id,
                    lease_until_ms,
                    dispatch_started_at_ms,
                    outcome_json,
                    reconciliation_json,
                )| {
                    Ok(OutboxSnapshot {
                        entry_id,
                        command_id: IntegrationCommandId::new(command_id)
                            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
                        connector_instance: ConnectorInstanceId::new(connector_instance)
                            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
                        stage: outbound_stage_from_i64(stage)?,
                        side_effect_class: side_effect_from_i64(side_effect)?,
                        attempt_count: u32::try_from(attempt_count).map_err(|_| {
                            RuntimeError::InvalidStoredEnum {
                                field: "attempt_count",
                                value: attempt_count,
                            }
                        })?,
                        current_attempt_id: current_attempt_id
                            .map(ExecutionAttemptId::new)
                            .transpose()
                            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
                        worker_id,
                        lease_until_ms,
                        dispatch_started_at_ms,
                        outcome: outcome_json
                            .map(|bytes| serde_json::from_slice(&bytes))
                            .transpose()?,
                        reconciliation: reconciliation_json
                            .map(|bytes| serde_json::from_slice(&bytes))
                            .transpose()?,
                    })
                },
            )
    }
}

fn validate_timestamp(value: i64) -> Result<(), RuntimeError> {
    if value < 0 {
        return Err(RuntimeError::InvalidTimestamp);
    }
    Ok(())
}

fn validate_worker_id(worker_id: &str) -> Result<(), RuntimeError> {
    if worker_id.is_empty() {
        return Err(RuntimeError::EmptyWorkerId);
    }
    if worker_id.len() > MAX_WORKER_ID_BYTES {
        return Err(RuntimeError::WorkerIdTooLong);
    }
    if worker_id.chars().any(char::is_control) {
        return Err(RuntimeError::WorkerIdContainsControl);
    }
    Ok(())
}

fn require_outbound_transition(
    from: OutboundStage,
    to: OutboundStage,
) -> Result<(), RuntimeError> {
    if !from.allows_transition_to(to) {
        return Err(RuntimeError::IllegalOutboundTransition { from, to });
    }
    Ok(())
}

fn require_attempt_owner(
    entry_id: i64,
    worker_id: &str,
    attempt_id: &ExecutionAttemptId,
    owner: Option<String>,
    stored_attempt: Option<String>,
) -> Result<(), RuntimeError> {
    if owner.as_deref() != Some(worker_id) {
        return Err(RuntimeError::LeaseOwnerMismatch { entry_id });
    }
    if stored_attempt.as_deref() != Some(attempt_id.as_str()) {
        return Err(RuntimeError::AttemptFenceMismatch { entry_id });
    }
    Ok(())
}

fn recover_expired_claims_in_tx(
    tx: &Transaction<'_>,
    now_ms: i64,
) -> Result<RecoverySummary, RuntimeError> {
    let stale = {
        let mut statement = tx.prepare(
            "SELECT entry_id, stage, command_id, connector_instance, current_attempt_id\n\
             FROM integration_outbox\n\
             WHERE stage IN (?1, ?2) AND lease_until_ms IS NOT NULL AND lease_until_ms <= ?3\n\
             ORDER BY entry_id ASC",
        )?;
        let rows = statement.query_map(
            params![
                outbound_stage_to_i64(OutboundStage::AttemptPrepared),
                outbound_stage_to_i64(OutboundStage::DispatchStarted),
                now_ms,
            ],
            |row| {
                Ok((
                    row.get::<_, i64>(0)?,
                    row.get::<_, i64>(1)?,
                    row.get::<_, String>(2)?,
                    row.get::<_, String>(3)?,
                    row.get::<_, Option<String>>(4)?,
                ))
            },
        )?;
        rows.collect::<Result<Vec<_>, _>>()?
    };

    let mut summary = RecoverySummary::default();
    for (entry_id, stage, command_id, connector_instance, attempt_id) in stale {
        let stage = outbound_stage_from_i64(stage)?;
        match stage {
            OutboundStage::AttemptPrepared => {
                require_outbound_transition(stage, OutboundStage::OutboxCommitted)?;
                tx.execute(
                    "UPDATE integration_outbox\n\
                     SET stage = ?1, worker_id = NULL, lease_until_ms = NULL,\n\
                         current_attempt_id = NULL, dispatch_started_at_ms = NULL,\n\
                         updated_at_ms = ?2\n\
                     WHERE entry_id = ?3 AND stage = ?4",
                    params![
                        outbound_stage_to_i64(OutboundStage::OutboxCommitted),
                        now_ms,
                        entry_id,
                        outbound_stage_to_i64(OutboundStage::AttemptPrepared),
                    ],
                )?;
                summary.pre_dispatch_requeued += 1;
            }
            OutboundStage::DispatchStarted => {
                let command_id = IntegrationCommandId::new(command_id)
                    .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;
                let connector_instance = ConnectorInstanceId::new(connector_instance)
                    .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;
                let outcome = ExternalExecutionOutcome::Ambiguous {
                    operation: ExternalOperationRef {
                        command_id,
                        connector_instance,
                        provider_operation: None,
                    },
                    reconciliation_hint: ReconciliationHint {
                        strategy: ReconciliationStrategy::ManualReview,
                        object: None,
                        idempotency_key: None,
                        earliest_retry_at_ms: None,
                    },
                };
                let json = serde_json::to_vec(&outcome)?;
                if json.len() > MAX_SERIALIZED_OUTCOME_BYTES {
                    return Err(RuntimeError::OutcomeTooLarge);
                }
                require_outbound_transition(stage, OutboundStage::Ambiguous)?;
                tx.execute(
                    "UPDATE integration_outbox\n\
                     SET stage = ?1, worker_id = NULL, lease_until_ms = NULL,\n\
                         outcome_json = ?2, updated_at_ms = ?3\n\
                     WHERE entry_id = ?4 AND stage = ?5",
                    params![
                        outbound_stage_to_i64(OutboundStage::Ambiguous),
                        json.as_slice(),
                        now_ms,
                        entry_id,
                        outbound_stage_to_i64(OutboundStage::DispatchStarted),
                    ],
                )?;
                if let Some(attempt_id) = attempt_id {
                    tx.execute(
                        "INSERT INTO integration_execution_observation (\n\
                            entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\n\
                         ) VALUES (?1, ?2, ?3, ?4, 1)",
                        params![entry_id, attempt_id, json.as_slice(), now_ms],
                    )?;
                }
                summary.post_dispatch_marked_ambiguous += 1;
            }
            _ => unreachable!("stale query restricts stages"),
        }
    }
    Ok(summary)
}

fn load_execution_claim(
    tx: &Transaction<'_>,
    entry_id: i64,
    attempt_id: ExecutionAttemptId,
    lease_until_ms: i64,
) -> Result<ExecutionClaim, RuntimeError> {
    #[allow(clippy::type_complexity)]
    let row: (String, String, i64, Vec<u8>, i64, Option<String>, i64) = tx.query_row(
        "SELECT command_id, connector_instance,\n\
                command_commitment_algorithm, command_commitment_digest,\n\
                side_effect_class, idempotency_key, attempt_count\n\
         FROM integration_outbox WHERE entry_id = ?1",
        params![entry_id],
        |row| {
            Ok((
                row.get(0)?,
                row.get(1)?,
                row.get(2)?,
                row.get(3)?,
                row.get(4)?,
                row.get(5)?,
                row.get(6)?,
            ))
        },
    )?;

    Ok(ExecutionClaim {
        entry_id,
        attempt_id,
        command_id: IntegrationCommandId::new(row.0)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
        connector_instance: ConnectorInstanceId::new(row.1)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
        command_commitment: commitment_from_parts(row.2, row.3)?,
        side_effect_class: side_effect_from_i64(row.4)?,
        idempotency_key: row
            .5
            .map(IdempotencyKey::new)
            .transpose()
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
        attempt_count: u32::try_from(row.6).map_err(|_| RuntimeError::InvalidStoredEnum {
            field: "attempt_count",
            value: row.6,
        })?,
        lease_until_ms,
    })
}

fn load_outbound_stage(tx: &Transaction<'_>, entry_id: i64) -> Result<OutboundStage, RuntimeError> {
    let stage: i64 = tx
        .query_row(
            "SELECT stage FROM integration_outbox WHERE entry_id = ?1",
            params![entry_id],
            |row| row.get(0),
        )
        .optional()?
        .ok_or(RuntimeError::UnknownOutboxEntry { entry_id })?;
    outbound_stage_from_i64(stage)
}

fn table_exists(conn: &Connection, name: &str) -> Result<bool, RuntimeError> {
    let exists: Option<i64> = conn
        .query_row(
            "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1",
            params![name],
            |row| row.get(0),
        )
        .optional()?;
    Ok(exists.is_some())
}

fn outbox_exists(conn: &Connection, entry_id: i64) -> Result<bool, RuntimeError> {
    let exists: Option<i64> = conn
        .query_row(
            "SELECT 1 FROM integration_outbox WHERE entry_id = ?1",
            params![entry_id],
            |row| row.get(0),
        )
        .optional()?;
    Ok(exists.is_some())
}

fn digest_algorithm_to_i64(algorithm: DigestAlgorithm) -> i64 {
    match algorithm {
        DigestAlgorithm::Sha256 => 1,
    }
}

fn digest_algorithm_from_i64(value: i64) -> Result<DigestAlgorithm, RuntimeError> {
    match value {
        1 => Ok(DigestAlgorithm::Sha256),
        _ => Err(RuntimeError::InvalidStoredEnum {
            field: "digest_algorithm",
            value,
        }),
    }
}

fn commitment_from_parts(
    algorithm: i64,
    digest: Vec<u8>,
) -> Result<ContentCommitment, RuntimeError> {
    if digest.len() != 32 {
        return Err(RuntimeError::InvalidStoredDigestLength {
            actual: digest.len(),
        });
    }
    let mut fixed = [0_u8; 32];
    fixed.copy_from_slice(&digest);
    Ok(ContentCommitment {
        algorithm: digest_algorithm_from_i64(algorithm)?,
        digest: fixed,
    })
}

fn side_effect_to_i64(value: SideEffectClass) -> i64 {
    match value {
        SideEffectClass::ReadOnly => 0,
        SideEffectClass::Reversible => 1,
        SideEffectClass::Compensatable => 2,
        SideEffectClass::Irreversible => 3,
    }
}

fn side_effect_from_i64(value: i64) -> Result<SideEffectClass, RuntimeError> {
    match value {
        0 => Ok(SideEffectClass::ReadOnly),
        1 => Ok(SideEffectClass::Reversible),
        2 => Ok(SideEffectClass::Compensatable),
        3 => Ok(SideEffectClass::Irreversible),
        _ => Err(RuntimeError::InvalidStoredEnum {
            field: "side_effect_class",
            value,
        }),
    }
}

fn inbound_stage_to_i64(value: InboundStage) -> i64 {
    match value {
        InboundStage::Received => 0,
        InboundStage::Authenticated => 1,
        InboundStage::Decoded => 2,
        InboundStage::Normalized => 3,
        InboundStage::Persisted => 4,
        InboundStage::Projected => 5,
        InboundStage::DomainAccepted => 6,
        InboundStage::Reconciled => 7,
        InboundStage::Finalized => 8,
        InboundStage::Rejected => 9,
        InboundStage::Duplicate => 10,
        InboundStage::DomainRejected => 11,
    }
}

fn outbound_stage_to_i64(value: OutboundStage) -> i64 {
    match value {
        OutboundStage::Proposed => 0,
        OutboundStage::AuthorityChecked => 1,
        OutboundStage::Approved => 2,
        OutboundStage::OutboxCommitted => 3,
        OutboundStage::AttemptPrepared => 4,
        OutboundStage::DispatchStarted => 5,
        OutboundStage::Confirmed => 6,
        OutboundStage::Rejected => 7,
        OutboundStage::Ambiguous => 8,
        OutboundStage::Reconciled => 9,
        OutboundStage::Finalized => 10,
    }
}

fn outbound_stage_from_i64(value: i64) -> Result<OutboundStage, RuntimeError> {
    match value {
        0 => Ok(OutboundStage::Proposed),
        1 => Ok(OutboundStage::AuthorityChecked),
        2 => Ok(OutboundStage::Approved),
        3 => Ok(OutboundStage::OutboxCommitted),
        4 => Ok(OutboundStage::AttemptPrepared),
        5 => Ok(OutboundStage::DispatchStarted),
        6 => Ok(OutboundStage::Confirmed),
        7 => Ok(OutboundStage::Rejected),
        8 => Ok(OutboundStage::Ambiguous),
        9 => Ok(OutboundStage::Reconciled),
        10 => Ok(OutboundStage::Finalized),
        _ => Err(RuntimeError::InvalidStoredEnum {
            field: "outbound_stage",
            value,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_integration_core::{
        ExternalOpaqueId, ExternalReceipt, ExternalRejection, ExternalRejectionCode,
        ReconciliationDisposition,
    };

    fn connector() -> ConnectorInstanceId {
        ConnectorInstanceId::new("stripe-prod-1").unwrap()
    }

    fn command_id(value: &str) -> IntegrationCommandId {
        IntegrationCommandId::new(value).unwrap()
    }

    fn event_id(value: &str) -> IntegrationEventId {
        IntegrationEventId::new(value).unwrap()
    }

    fn intent(command: &str, side_effect_class: SideEffectClass) -> DurableOutboundIntent {
        DurableOutboundIntent {
            command_id: command_id(command),
            connector_instance: connector(),
            command_commitment: ContentCommitment::sha256(format!("command:{command}").as_bytes()),
            authority_commitment: ContentCommitment::sha256(b"authority:sealed"),
            side_effect_class,
            idempotency_key: Some(IdempotencyKey::new(format!("idem:{command}")).unwrap()),
            command_bytes: format!("sealed:{command}").into_bytes(),
            created_at_ms: 100,
        }
    }

    fn operation(command: &str) -> ExternalOperationRef {
        ExternalOperationRef {
            command_id: command_id(command),
            connector_instance: connector(),
            provider_operation: None,
        }
    }

    fn confirmed(command: &str) -> ExternalExecutionOutcome {
        ExternalExecutionOutcome::Confirmed(ExternalReceipt {
            operation: operation(command),
            provider_receipt: Some(ExternalOpaqueId::new("receipt-1").unwrap()),
            receipt_commitment: ContentCommitment::sha256(b"provider-receipt"),
            confirmed_at_ms: 150,
        })
    }

    fn rejected(command: &str) -> ExternalExecutionOutcome {
        ExternalExecutionOutcome::Rejected {
            operation: operation(command),
            reason: ExternalRejection {
                code: ExternalRejectionCode::new("declined").unwrap(),
                detail_commitment: None,
                rejected_at_ms: 150,
            },
        }
    }

    fn reconciliation(
        command: &str,
        disposition: ReconciliationDisposition,
    ) -> ReconciliationResult {
        ReconciliationResult {
            operation: operation(command),
            disposition,
            evidence: ContentCommitment::sha256(
                format!("reconciliation:{disposition:?}").as_bytes(),
            ),
            reconciled_at_ms: 175,
        }
    }

    fn enqueue(store: &mut SqliteIntegrationStore, command: &str, class: SideEffectClass) -> i64 {
        match store.enqueue_outbound(&intent(command, class)).unwrap() {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected disposition: {other:?}"),
        }
    }

    #[test]
    fn inbound_duplicate_is_idempotent_but_collisions_are_rejected() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let inbound = PersistableInbound {
            event_id: event_id("evt-1"),
            connector_instance: connector(),
            event_commitment: ContentCommitment::sha256(b"event-a"),
            normalized_payload: b"normalized".to_vec(),
            received_at_ms: 100,
        };

        assert_eq!(store.insert_inbound(&inbound).unwrap(), InsertDisposition::Inserted);
        assert_eq!(store.insert_inbound(&inbound).unwrap(), InsertDisposition::Duplicate);

        let mut content_conflict = inbound.clone();
        content_conflict.event_commitment = ContentCommitment::sha256(b"event-b");
        assert!(matches!(
            store.insert_inbound(&content_conflict),
            Err(RuntimeError::InboundIdentityCollision { .. })
        ));

        let mut normalization_conflict = inbound.clone();
        normalization_conflict.normalized_payload = b"different-normalization".to_vec();
        assert!(matches!(
            store.insert_inbound(&normalization_conflict),
            Err(RuntimeError::InboundNormalizationCollision { .. })
        ));
    }

    #[test]
    fn outbox_enqueue_is_idempotent_but_connector_or_command_mutation_is_rejected() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let intent = intent("cmd-1", SideEffectClass::Irreversible);
        let entry_id = match store.enqueue_outbound(&intent).unwrap() {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected disposition: {other:?}"),
        };
        assert_eq!(
            store.enqueue_outbound(&intent).unwrap(),
            EnqueueDisposition::AlreadyPresent(entry_id)
        );

        let mut bytes_conflict = intent.clone();
        bytes_conflict.command_bytes = b"different-command".to_vec();
        assert!(matches!(
            store.enqueue_outbound(&bytes_conflict),
            Err(RuntimeError::OutboxIdentityCollision { .. })
        ));

        let mut connector_conflict = intent.clone();
        connector_conflict.connector_instance = ConnectorInstanceId::new("other").unwrap();
        assert!(matches!(
            store.enqueue_outbound(&connector_conflict),
            Err(RuntimeError::OutboxIdentityCollision { .. })
        ));
    }

    #[test]
    fn claim_returns_metadata_not_executable_command_bytes() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-2", SideEffectClass::Irreversible);
        let claimed = store.claim_outbox("worker-a", 110, 50, 10).unwrap();
        assert_eq!(claimed.len(), 1);
        assert_eq!(claimed[0].entry_id, entry_id);
        assert_eq!(claimed[0].attempt_count, 1);
        assert_eq!(
            store.outbox_snapshot(entry_id).unwrap().stage,
            OutboundStage::AttemptPrepared
        );
    }

    #[test]
    fn pre_dispatch_crash_is_reclaimable_even_for_irreversible_work() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-pre", SideEffectClass::Irreversible);
        store.claim_outbox("worker-a", 110, 10, 10).unwrap();

        let recovered = store.recover_expired_claims(121).unwrap();
        assert_eq!(recovered.pre_dispatch_requeued, 1);
        assert_eq!(
            store.outbox_snapshot(entry_id).unwrap().stage,
            OutboundStage::OutboxCommitted
        );
    }

    #[test]
    fn post_dispatch_crash_becomes_ambiguous_without_inferring_idempotency_support() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-post", SideEffectClass::Reversible);
        let claim = store.claim_outbox("worker-a", 110, 10, 10).unwrap().remove(0);
        store
            .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-a", 111)
            .unwrap();

        let recovered = store.recover_expired_claims(121).unwrap();
        assert_eq!(recovered.post_dispatch_marked_ambiguous, 1);
        let snapshot = store.outbox_snapshot(entry_id).unwrap();
        assert_eq!(snapshot.stage, OutboundStage::Ambiguous);
        assert!(matches!(
            snapshot.outcome,
            Some(ExternalExecutionOutcome::Ambiguous {
                reconciliation_hint: ReconciliationHint {
                    strategy: ReconciliationStrategy::ManualReview,
                    ..
                },
                ..
            })
        ));
        assert!(store.claim_outbox("worker-b", 122, 10, 10).unwrap().is_empty());
    }

    #[test]
    fn stale_attempt_cannot_complete_a_new_attempt_even_with_same_worker_id() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-fence", SideEffectClass::Irreversible);
        let attempt_one = store.claim_outbox("worker-a", 110, 10, 10).unwrap().remove(0);
        store.recover_expired_claims(121).unwrap();
        let attempt_two = store.claim_outbox("worker-a", 122, 20, 10).unwrap().remove(0);
        store
            .mark_dispatch_started(entry_id, &attempt_two.attempt_id, "worker-a", 123)
            .unwrap();

        assert!(matches!(
            store.record_execution(
                entry_id,
                &attempt_one.attempt_id,
                "worker-a",
                &confirmed("cmd-fence"),
                124,
            ),
            Err(RuntimeError::AttemptFenceMismatch { .. })
        ));
    }

    #[test]
    fn late_response_after_post_dispatch_timeout_is_evidence_not_state_rewrite() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-late", SideEffectClass::Irreversible);
        let claim = store.claim_outbox("worker-a", 110, 10, 10).unwrap().remove(0);
        store
            .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-a", 111)
            .unwrap();
        store.recover_expired_claims(121).unwrap();

        let disposition = store
            .record_execution(
                entry_id,
                &claim.attempt_id,
                "worker-a",
                &confirmed("cmd-late"),
                122,
            )
            .unwrap();
        assert_eq!(
            disposition,
            ExecutionRecordDisposition::RecordedForStaleAttempt
        );
        assert_eq!(
            store.outbox_snapshot(entry_id).unwrap().stage,
            OutboundStage::Ambiguous
        );
    }

    #[test]
    fn rejected_outcome_for_another_operation_is_rejected() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-reject", SideEffectClass::Irreversible);
        let claim = store.claim_outbox("worker-a", 110, 50, 10).unwrap().remove(0);
        store
            .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-a", 111)
            .unwrap();

        assert!(matches!(
            store.record_execution(
                entry_id,
                &claim.attempt_id,
                "worker-a",
                &rejected("different"),
                150,
            ),
            Err(RuntimeError::OutcomeOperationMismatch { .. })
        ));
    }

    #[test]
    fn still_ambiguous_does_not_finalize_and_history_is_append_only() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-amb", SideEffectClass::Irreversible);
        let claim = store.claim_outbox("worker-a", 110, 10, 10).unwrap().remove(0);
        store
            .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-a", 111)
            .unwrap();
        store.recover_expired_claims(121).unwrap();

        store
            .record_reconciliation(
                entry_id,
                &reconciliation("cmd-amb", ReconciliationDisposition::StillAmbiguous),
                130,
            )
            .unwrap();
        store
            .record_reconciliation(
                entry_id,
                &reconciliation("cmd-amb", ReconciliationDisposition::StillAmbiguous),
                140,
            )
            .unwrap();

        assert_eq!(
            store.outbox_snapshot(entry_id).unwrap().stage,
            OutboundStage::Ambiguous
        );
        assert!(matches!(
            store.finalize_outbound(entry_id, 150),
            Err(RuntimeError::IllegalOutboundTransition { .. })
        ));
        assert_eq!(store.reconciliation_history(entry_id).unwrap().len(), 2);
    }

    #[test]
    fn confirmed_effect_requires_conclusive_reconciliation_before_finalization() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = enqueue(&mut store, "cmd-4", SideEffectClass::Irreversible);
        let claim = store.claim_outbox("worker-a", 110, 50, 10).unwrap().remove(0);
        store
            .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-a", 111)
            .unwrap();
        store
            .record_execution(
                entry_id,
                &claim.attempt_id,
                "worker-a",
                &confirmed("cmd-4"),
                150,
            )
            .unwrap();

        assert!(matches!(
            store.finalize_outbound(entry_id, 160),
            Err(RuntimeError::IllegalOutboundTransition { .. })
        ));

        store
            .record_reconciliation(
                entry_id,
                &reconciliation("cmd-4", ReconciliationDisposition::ConfirmsEffect),
                175,
            )
            .unwrap();
        store.finalize_outbound(entry_id, 180).unwrap();
        assert_eq!(
            store.outbox_snapshot(entry_id).unwrap().stage,
            OutboundStage::Finalized
        );
    }

    #[test]
    fn checkpoint_round_trips() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let cursor = ReconcileCursor {
            connector_instance: connector(),
            cursor: ExternalOpaqueId::new("cursor-42").unwrap(),
            checkpoint_commitment: ContentCommitment::sha256(b"checkpoint"),
        };
        store.checkpoint_reconciliation(&cursor, 200).unwrap();
        assert_eq!(
            store.load_reconciliation_checkpoint(&connector()).unwrap(),
            Some(cursor)
        );
    }

    #[test]
    fn outbox_survives_store_reopen() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("integration.sqlite");
        let entry_id;
        {
            let mut store = SqliteIntegrationStore::open(&path).unwrap();
            entry_id = enqueue(&mut store, "cmd-persist", SideEffectClass::Irreversible);
        }

        let store = SqliteIntegrationStore::open(&path).unwrap();
        let snapshot = store.outbox_snapshot(entry_id).unwrap();
        assert_eq!(snapshot.stage, OutboundStage::OutboxCommitted);
        assert_eq!(snapshot.attempt_count, 0);
    }

    #[test]
    fn legacy_executing_stage_migrates_to_ambiguous_not_attempt_prepared() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("legacy.sqlite");
        {
            let conn = Connection::open(&path).unwrap();
            conn.execute_batch(
                r#"
                CREATE TABLE integration_outbox (
                    entry_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    command_id TEXT NOT NULL UNIQUE,
                    connector_instance TEXT NOT NULL,
                    command_commitment_algorithm INTEGER NOT NULL,
                    command_commitment_digest BLOB NOT NULL,
                    authority_commitment_algorithm INTEGER NOT NULL,
                    authority_commitment_digest BLOB NOT NULL,
                    side_effect_class INTEGER NOT NULL,
                    idempotency_key TEXT,
                    command_bytes BLOB NOT NULL,
                    stage INTEGER NOT NULL,
                    worker_id TEXT,
                    lease_until_ms INTEGER,
                    attempt_count INTEGER NOT NULL DEFAULT 0,
                    outcome_json BLOB,
                    reconciliation_json BLOB,
                    created_at_ms INTEGER NOT NULL,
                    updated_at_ms INTEGER NOT NULL
                );
                CREATE TABLE integration_inbound (
                    connector_instance TEXT NOT NULL,
                    event_id TEXT NOT NULL,
                    commitment_algorithm INTEGER NOT NULL,
                    commitment_digest BLOB NOT NULL,
                    stage INTEGER NOT NULL,
                    normalized_payload BLOB NOT NULL,
                    received_at_ms INTEGER NOT NULL,
                    PRIMARY KEY (connector_instance, event_id)
                );
                CREATE TABLE integration_reconcile_checkpoint (
                    connector_instance TEXT PRIMARY KEY,
                    cursor TEXT NOT NULL,
                    commitment_algorithm INTEGER NOT NULL,
                    commitment_digest BLOB NOT NULL,
                    updated_at_ms INTEGER NOT NULL
                );
                "#,
            )
            .unwrap();
            let digest = [0_u8; 32];
            conn.execute(
                "INSERT INTO integration_outbox (\n\
                    command_id, connector_instance, command_commitment_algorithm, command_commitment_digest,\n\
                    authority_commitment_algorithm, authority_commitment_digest, side_effect_class,\n\
                    command_bytes, stage, worker_id, lease_until_ms, attempt_count, created_at_ms, updated_at_ms\n\
                 ) VALUES (?1, ?2, 1, ?3, 1, ?3, 3, ?4, 4, 'worker-old', 999, 1, 100, 100)",
                rusqlite::params![
                    "legacy-cmd",
                    "legacy-connector",
                    digest.as_slice(),
                    b"sealed".as_slice()
                ],
            )
            .unwrap();
        }

        let store = SqliteIntegrationStore::open(&path).unwrap();
        let snapshot = store.outbox_snapshot(1).unwrap();
        assert_eq!(snapshot.stage, OutboundStage::Ambiguous);
        assert!(snapshot.worker_id.is_none());
        assert!(snapshot.lease_until_ms.is_none());
    }
}

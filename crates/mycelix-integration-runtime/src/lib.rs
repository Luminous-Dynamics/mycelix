//! Durable local reference runtime for the Mycelix integration plane.
//!
//! The runtime owns causal durability. It does not decide institutional truth,
//! mint authority, or call provider APIs. In particular, inserting an outbound
//! intent into this store is not itself an execution capability; INT-04 remains
//! responsible for qualifying exact authority before a connector execution
//! boundary is permitted to consume it.

use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, DigestAlgorithm, ExternalExecutionOutcome,
    ExternalOperationRef, IdempotencyKey, InboundStage, IntegrationCommandId, OutboundStage,
    ReconcileCursor, ReconciliationHint, ReconciliationResult, ReconciliationStrategy,
    SideEffectClass,
};
use rusqlite::{params, Connection, OptionalExtension, Transaction, TransactionBehavior};
use serde::{Deserialize, Serialize};
use std::{path::Path, time::Duration};
use thiserror::Error;

const MAX_WORKER_ID_BYTES: usize = 256;
const MAX_OUTBOX_CLAIM: usize = 1024;

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
    #[error("inbound event identity collision for {connector_instance}:{event_id}")]
    InboundIdentityCollision {
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
    #[error("outbound command bytes must not be empty")]
    EmptyCommandBytes,
    #[error("illegal outbound transition {from:?} -> {to:?}")]
    IllegalOutboundTransition {
        from: OutboundStage,
        to: OutboundStage,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PersistableInbound {
    pub event_id: String,
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClaimedOutboxEntry {
    pub entry_id: i64,
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub command_commitment: ContentCommitment,
    pub authority_commitment: ContentCommitment,
    pub side_effect_class: SideEffectClass,
    pub idempotency_key: Option<IdempotencyKey>,
    pub command_bytes: Vec<u8>,
    pub attempt_count: u32,
    pub lease_until_ms: i64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutboxSnapshot {
    pub entry_id: i64,
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub stage: OutboundStage,
    pub side_effect_class: SideEffectClass,
    pub attempt_count: u32,
    pub worker_id: Option<String>,
    pub lease_until_ms: Option<i64>,
    pub outcome: Option<ExternalExecutionOutcome>,
    pub reconciliation: Option<ReconciliationResult>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RecoverySummary {
    /// Expired read-only claims can safely return to the committed queue.
    pub read_only_requeued: usize,
    /// Any expired side-effecting claim becomes ambiguous until reconciled.
    pub side_effecting_marked_ambiguous: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct StoredCrashAmbiguity {
    outcome: ExternalExecutionOutcome,
    reason: &'static str,
}

/// SQLite-backed durable reference implementation.
///
/// The connection is intentionally owned directly rather than shared across
/// threads. A future async orchestration layer can place this store behind a
/// dedicated blocking worker without making storage concurrency part of the
/// semantic contract.
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
        // WAL is a durability/performance preference for file-backed stores.
        // SQLite legitimately keeps `:memory:` databases in memory-journal mode.
        let _ = conn.pragma_update(None, "journal_mode", "WAL");

        let store = Self { conn };
        store.initialize_schema()?;
        Ok(store)
    }

    fn initialize_schema(&self) -> Result<(), RuntimeError> {
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
                outcome_json BLOB,
                reconciliation_json BLOB,
                created_at_ms INTEGER NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );

            CREATE INDEX IF NOT EXISTS integration_outbox_claim_idx
                ON integration_outbox(stage, entry_id);

            CREATE INDEX IF NOT EXISTS integration_outbox_lease_idx
                ON integration_outbox(stage, lease_until_ms);

            CREATE TABLE IF NOT EXISTS integration_reconcile_checkpoint (
                connector_instance TEXT PRIMARY KEY,
                cursor TEXT NOT NULL,
                commitment_algorithm INTEGER NOT NULL,
                commitment_digest BLOB NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );
            "#,
        )?;
        Ok(())
    }

    pub fn insert_inbound(
        &mut self,
        inbound: &PersistableInbound,
    ) -> Result<InsertDisposition, RuntimeError> {
        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        let key_connector = inbound.connector_instance.as_str();

        let existing: Option<(i64, Vec<u8>)> = tx
            .query_row(
                "SELECT commitment_algorithm, commitment_digest\n\
                 FROM integration_inbound\n\
                 WHERE connector_instance = ?1 AND event_id = ?2",
                params![key_connector, inbound.event_id],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()?;

        if let Some((algorithm, digest)) = existing {
            let existing = commitment_from_parts(algorithm, digest)?;
            if existing == inbound.event_commitment {
                tx.commit()?;
                return Ok(InsertDisposition::Duplicate);
            }
            return Err(RuntimeError::InboundIdentityCollision {
                connector_instance: key_connector.to_owned(),
                event_id: inbound.event_id.clone(),
            });
        }

        tx.execute(
            "INSERT INTO integration_inbound (\n\
                connector_instance, event_id, commitment_algorithm, commitment_digest,\n\
                stage, normalized_payload, received_at_ms\n\
             ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
            params![
                key_connector,
                inbound.event_id,
                digest_algorithm_to_i64(inbound.event_commitment.algorithm),
                inbound.event_commitment.digest.as_slice(),
                outbound_stage_or_inbound_to_i64(InboundStage::Persisted),
                inbound.normalized_payload,
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
        if intent.command_bytes.is_empty() {
            return Err(RuntimeError::EmptyCommandBytes);
        }

        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;

        #[allow(clippy::type_complexity)]
        let existing: Option<(i64, i64, Vec<u8>, i64, Vec<u8>, i64, Option<String>, Vec<u8>)> = tx
            .query_row(
                "SELECT entry_id, command_commitment_algorithm, command_commitment_digest,\n\
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
                    ))
                },
            )
            .optional()?;

        if let Some((
            entry_id,
            command_alg,
            command_digest,
            authority_alg,
            authority_digest,
            side_effect,
            idempotency_key,
            command_bytes,
        )) = existing
        {
            let same = commitment_from_parts(command_alg, command_digest)?
                == intent.command_commitment
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
                intent.command_bytes,
                outbound_stage_to_i64(OutboundStage::OutboxCommitted),
                intent.created_at_ms,
            ],
        )?;
        let entry_id = tx.last_insert_rowid();
        tx.commit()?;
        Ok(EnqueueDisposition::Inserted(entry_id))
    }

    /// Claim committed work atomically.
    ///
    /// Before selecting new work, expired claims are recovered conservatively:
    /// read-only operations may be retried, but every expired side-effecting
    /// operation becomes `Ambiguous` and leaves the executable queue.
    pub fn claim_outbox(
        &mut self,
        worker_id: &str,
        now_ms: i64,
        lease_duration_ms: i64,
        limit: usize,
    ) -> Result<Vec<ClaimedOutboxEntry>, RuntimeError> {
        validate_worker_id(worker_id)?;
        if lease_duration_ms <= 0 {
            return Err(RuntimeError::InvalidLeaseDuration);
        }
        if !(1..=MAX_OUTBOX_CLAIM).contains(&limit) {
            return Err(RuntimeError::InvalidClaimLimit);
        }

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
                params![outbound_stage_to_i64(OutboundStage::OutboxCommitted), limit as i64],
                |row| row.get::<_, i64>(0),
            )?;
            rows.collect::<Result<Vec<_>, _>>()?
        };

        let lease_until_ms = now_ms.saturating_add(lease_duration_ms);
        for entry_id in &ids {
            let changed = tx.execute(
                "UPDATE integration_outbox\n\
                 SET stage = ?1, worker_id = ?2, lease_until_ms = ?3,\n\
                     attempt_count = attempt_count + 1, updated_at_ms = ?4\n\
                 WHERE entry_id = ?5 AND stage = ?6",
                params![
                    outbound_stage_to_i64(OutboundStage::Executing),
                    worker_id,
                    lease_until_ms,
                    now_ms,
                    entry_id,
                    outbound_stage_to_i64(OutboundStage::OutboxCommitted),
                ],
            )?;
            if changed != 1 {
                return Err(RuntimeError::UnexpectedOutboundStage {
                    entry_id: *entry_id,
                    expected: OutboundStage::OutboxCommitted,
                    actual: load_outbound_stage(&tx, *entry_id)?,
                });
            }
        }

        let mut claimed = Vec::with_capacity(ids.len());
        for entry_id in ids {
            claimed.push(load_claimed_entry(&tx, entry_id, lease_until_ms)?);
        }
        tx.commit()?;
        Ok(claimed)
    }

    pub fn recover_expired_claims(
        &mut self,
        now_ms: i64,
    ) -> Result<RecoverySummary, RuntimeError> {
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
        worker_id: &str,
        outcome: &ExternalExecutionOutcome,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
        validate_worker_id(worker_id)?;
        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;

        let (stage, owner, command_id, connector_instance): (i64, Option<String>, String, String) =
            tx.query_row(
                "SELECT stage, worker_id, command_id, connector_instance\n\
                 FROM integration_outbox WHERE entry_id = ?1",
                params![entry_id],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?, row.get(3)?)),
            )
            .optional()?
            .ok_or(RuntimeError::UnknownOutboxEntry { entry_id })?;

        let stage = outbound_stage_from_i64(stage)?;
        if stage != OutboundStage::Executing {
            return Err(RuntimeError::UnexpectedOutboundStage {
                entry_id,
                expected: OutboundStage::Executing,
                actual: stage,
            });
        }
        if owner.as_deref() != Some(worker_id) {
            return Err(RuntimeError::LeaseOwnerMismatch { entry_id });
        }

        if let Some(operation) = outcome_operation(outcome)
            && (operation.command_id.as_str() != command_id
                || operation.connector_instance.as_str() != connector_instance)
        {
            return Err(RuntimeError::OutcomeOperationMismatch { entry_id });
        }

        let next = match outcome {
            ExternalExecutionOutcome::Confirmed(_) => OutboundStage::Confirmed,
            ExternalExecutionOutcome::Rejected { .. } => OutboundStage::Rejected,
            ExternalExecutionOutcome::Ambiguous { .. } => OutboundStage::Ambiguous,
        };
        require_outbound_transition(stage, next)?;

        let outcome_json = serde_json::to_vec(outcome)?;
        tx.execute(
            "UPDATE integration_outbox\n\
             SET stage = ?1, worker_id = NULL, lease_until_ms = NULL,\n\
                 outcome_json = ?2, updated_at_ms = ?3\n\
             WHERE entry_id = ?4",
            params![outbound_stage_to_i64(next), outcome_json, now_ms, entry_id],
        )?;
        tx.commit()?;
        Ok(())
    }

    pub fn record_reconciliation(
        &mut self,
        entry_id: i64,
        reconciliation: &ReconciliationResult,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
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

        require_outbound_transition(stage, OutboundStage::Reconciled)?;
        let reconciliation_json = serde_json::to_vec(reconciliation)?;
        tx.execute(
            "UPDATE integration_outbox\n\
             SET stage = ?1, reconciliation_json = ?2, updated_at_ms = ?3\n\
             WHERE entry_id = ?4",
            params![
                outbound_stage_to_i64(OutboundStage::Reconciled),
                reconciliation_json,
                now_ms,
                entry_id
            ],
        )?;
        tx.commit()?;
        Ok(())
    }

    pub fn finalize_outbound(
        &mut self,
        entry_id: i64,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
        let tx = self
            .conn
            .transaction_with_behavior(TransactionBehavior::Immediate)?;
        let stage = load_outbound_stage(&tx, entry_id)?;
        require_outbound_transition(stage, OutboundStage::Finalized)?;
        tx.execute(
            "UPDATE integration_outbox SET stage = ?1, updated_at_ms = ?2 WHERE entry_id = ?3",
            params![outbound_stage_to_i64(OutboundStage::Finalized), now_ms, entry_id],
        )?;
        tx.commit()?;
        Ok(())
    }

    pub fn checkpoint_reconciliation(
        &mut self,
        cursor: &ReconcileCursor,
        updated_at_ms: i64,
    ) -> Result<(), RuntimeError> {
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
                        attempt_count, worker_id, lease_until_ms, outcome_json, reconciliation_json\n\
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
                        row.get::<_, Option<i64>>(6)?,
                        row.get::<_, Option<Vec<u8>>>(7)?,
                        row.get::<_, Option<Vec<u8>>>(8)?,
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
                    worker_id,
                    lease_until_ms,
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
                        worker_id,
                        lease_until_ms,
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

fn outcome_operation(outcome: &ExternalExecutionOutcome) -> Option<&ExternalOperationRef> {
    match outcome {
        ExternalExecutionOutcome::Confirmed(receipt) => Some(&receipt.operation),
        ExternalExecutionOutcome::Ambiguous { operation, .. } => Some(operation),
        ExternalExecutionOutcome::Rejected { .. } => None,
    }
}

fn recover_expired_claims_in_tx(
    tx: &Transaction<'_>,
    now_ms: i64,
) -> Result<RecoverySummary, RuntimeError> {
    let stale = {
        let mut statement = tx.prepare(
            "SELECT entry_id, command_id, connector_instance, side_effect_class, idempotency_key\n\
             FROM integration_outbox\n\
             WHERE stage = ?1 AND lease_until_ms IS NOT NULL AND lease_until_ms <= ?2\n\
             ORDER BY entry_id ASC",
        )?;
        let rows = statement.query_map(
            params![outbound_stage_to_i64(OutboundStage::Executing), now_ms],
            |row| {
                Ok((
                    row.get::<_, i64>(0)?,
                    row.get::<_, String>(1)?,
                    row.get::<_, String>(2)?,
                    row.get::<_, i64>(3)?,
                    row.get::<_, Option<String>>(4)?,
                ))
            },
        )?;
        rows.collect::<Result<Vec<_>, _>>()?
    };

    let mut summary = RecoverySummary::default();
    for (entry_id, command_id, connector_instance, side_effect, idempotency_key) in stale {
        let side_effect = side_effect_from_i64(side_effect)?;
        if side_effect == SideEffectClass::ReadOnly {
            require_outbound_transition(OutboundStage::Executing, OutboundStage::Ambiguous)
                .or_else(|_| Ok::<(), RuntimeError>(()))?;
            tx.execute(
                "UPDATE integration_outbox\n\
                 SET stage = ?1, worker_id = NULL, lease_until_ms = NULL, updated_at_ms = ?2\n\
                 WHERE entry_id = ?3",
                params![
                    outbound_stage_to_i64(OutboundStage::OutboxCommitted),
                    now_ms,
                    entry_id
                ],
            )?;
            summary.read_only_requeued += 1;
            continue;
        }

        let command_id = IntegrationCommandId::new(command_id)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;
        let connector_instance = ConnectorInstanceId::new(connector_instance)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;
        let idempotency_key = idempotency_key
            .map(IdempotencyKey::new)
            .transpose()
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?;

        let outcome = ExternalExecutionOutcome::Ambiguous {
            operation: ExternalOperationRef {
                command_id,
                connector_instance,
                provider_operation: None,
            },
            reconciliation_hint: ReconciliationHint {
                strategy: if idempotency_key.is_some() {
                    ReconciliationStrategy::IdempotencyKey
                } else {
                    ReconciliationStrategy::ManualReview
                },
                object: None,
                idempotency_key,
                earliest_retry_at_ms: None,
            },
        };
        require_outbound_transition(OutboundStage::Executing, OutboundStage::Ambiguous)?;
        let stored = StoredCrashAmbiguity {
            outcome,
            reason: "worker lease expired while a side-effecting operation was executing",
        };
        let json = serde_json::to_vec(&stored.outcome)?;
        tx.execute(
            "UPDATE integration_outbox\n\
             SET stage = ?1, worker_id = NULL, lease_until_ms = NULL,\n\
                 outcome_json = ?2, updated_at_ms = ?3\n\
             WHERE entry_id = ?4",
            params![
                outbound_stage_to_i64(OutboundStage::Ambiguous),
                json,
                now_ms,
                entry_id
            ],
        )?;
        summary.side_effecting_marked_ambiguous += 1;
    }
    Ok(summary)
}

fn load_claimed_entry(
    tx: &Transaction<'_>,
    entry_id: i64,
    lease_until_ms: i64,
) -> Result<ClaimedOutboxEntry, RuntimeError> {
    #[allow(clippy::type_complexity)]
    let row: (String, String, i64, Vec<u8>, i64, Vec<u8>, i64, Option<String>, Vec<u8>, i64) =
        tx.query_row(
            "SELECT command_id, connector_instance,\n\
                    command_commitment_algorithm, command_commitment_digest,\n\
                    authority_commitment_algorithm, authority_commitment_digest,\n\
                    side_effect_class, idempotency_key, command_bytes, attempt_count\n\
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
                    row.get(7)?,
                    row.get(8)?,
                    row.get(9)?,
                ))
            },
        )?;

    Ok(ClaimedOutboxEntry {
        entry_id,
        command_id: IntegrationCommandId::new(row.0)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
        connector_instance: ConnectorInstanceId::new(row.1)
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
        command_commitment: commitment_from_parts(row.2, row.3)?,
        authority_commitment: commitment_from_parts(row.4, row.5)?,
        side_effect_class: side_effect_from_i64(row.6)?,
        idempotency_key: row
            .7
            .map(IdempotencyKey::new)
            .transpose()
            .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
        command_bytes: row.8,
        attempt_count: u32::try_from(row.9).map_err(|_| RuntimeError::InvalidStoredEnum {
            field: "attempt_count",
            value: row.9,
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

fn outbound_stage_or_inbound_to_i64(value: InboundStage) -> i64 {
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
        OutboundStage::Executing => 4,
        OutboundStage::Confirmed => 5,
        OutboundStage::Rejected => 6,
        OutboundStage::Ambiguous => 7,
        OutboundStage::Reconciled => 8,
        OutboundStage::Finalized => 9,
    }
}

fn outbound_stage_from_i64(value: i64) -> Result<OutboundStage, RuntimeError> {
    match value {
        0 => Ok(OutboundStage::Proposed),
        1 => Ok(OutboundStage::AuthorityChecked),
        2 => Ok(OutboundStage::Approved),
        3 => Ok(OutboundStage::OutboxCommitted),
        4 => Ok(OutboundStage::Executing),
        5 => Ok(OutboundStage::Confirmed),
        6 => Ok(OutboundStage::Rejected),
        7 => Ok(OutboundStage::Ambiguous),
        8 => Ok(OutboundStage::Reconciled),
        9 => Ok(OutboundStage::Finalized),
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
        ExternalOpaqueId, ExternalReceipt, ReconciliationDisposition,
    };

    fn connector() -> ConnectorInstanceId {
        ConnectorInstanceId::new("stripe-prod-1").unwrap()
    }

    fn command_id(value: &str) -> IntegrationCommandId {
        IntegrationCommandId::new(value).unwrap()
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

    fn reconciliation(command: &str) -> ReconciliationResult {
        ReconciliationResult {
            operation: operation(command),
            disposition: ReconciliationDisposition::ConfirmsEffect,
            evidence: ContentCommitment::sha256(b"reconciliation-evidence"),
            reconciled_at_ms: 175,
        }
    }

    #[test]
    fn inbound_duplicate_is_idempotent_but_collision_is_rejected() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let inbound = PersistableInbound {
            event_id: "evt-1".into(),
            connector_instance: connector(),
            event_commitment: ContentCommitment::sha256(b"event-a"),
            normalized_payload: b"normalized".to_vec(),
            received_at_ms: 100,
        };

        assert_eq!(store.insert_inbound(&inbound).unwrap(), InsertDisposition::Inserted);
        assert_eq!(store.insert_inbound(&inbound).unwrap(), InsertDisposition::Duplicate);

        let mut conflict = inbound.clone();
        conflict.event_commitment = ContentCommitment::sha256(b"event-b");
        assert!(matches!(
            store.insert_inbound(&conflict),
            Err(RuntimeError::InboundIdentityCollision { .. })
        ));
    }

    #[test]
    fn outbox_enqueue_is_idempotent_but_mutation_under_same_command_id_is_rejected() {
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

        let mut conflict = intent.clone();
        conflict.command_bytes = b"different-command".to_vec();
        assert!(matches!(
            store.enqueue_outbound(&conflict),
            Err(RuntimeError::OutboxIdentityCollision { .. })
        ));
    }

    #[test]
    fn claim_moves_only_durable_outbox_work_to_executing() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = match store
            .enqueue_outbound(&intent("cmd-2", SideEffectClass::Irreversible))
            .unwrap()
        {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected disposition: {other:?}"),
        };

        let claimed = store.claim_outbox("worker-a", 110, 50, 10).unwrap();
        assert_eq!(claimed.len(), 1);
        assert_eq!(claimed[0].entry_id, entry_id);
        assert_eq!(claimed[0].attempt_count, 1);

        let snapshot = store.outbox_snapshot(entry_id).unwrap();
        assert_eq!(snapshot.stage, OutboundStage::Executing);
        assert_eq!(snapshot.worker_id.as_deref(), Some("worker-a"));
    }

    #[test]
    fn expired_irreversible_claim_becomes_ambiguous_and_is_not_reexecuted() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = match store
            .enqueue_outbound(&intent("cmd-3", SideEffectClass::Irreversible))
            .unwrap()
        {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected disposition: {other:?}"),
        };
        store.claim_outbox("worker-a", 110, 10, 10).unwrap();

        let recovered = store.recover_expired_claims(121).unwrap();
        assert_eq!(recovered.side_effecting_marked_ambiguous, 1);
        let snapshot = store.outbox_snapshot(entry_id).unwrap();
        assert_eq!(snapshot.stage, OutboundStage::Ambiguous);
        assert!(matches!(snapshot.outcome, Some(ExternalExecutionOutcome::Ambiguous { .. })));

        assert!(store.claim_outbox("worker-b", 122, 10, 10).unwrap().is_empty());
    }

    #[test]
    fn expired_read_only_claim_can_be_requeued() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = match store
            .enqueue_outbound(&intent("cmd-read", SideEffectClass::ReadOnly))
            .unwrap()
        {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected disposition: {other:?}"),
        };
        store.claim_outbox("worker-a", 110, 10, 10).unwrap();

        let recovered = store.recover_expired_claims(121).unwrap();
        assert_eq!(recovered.read_only_requeued, 1);
        assert_eq!(
            store.outbox_snapshot(entry_id).unwrap().stage,
            OutboundStage::OutboxCommitted
        );
        assert_eq!(store.claim_outbox("worker-b", 122, 10, 10).unwrap().len(), 1);
    }

    #[test]
    fn confirmed_effect_must_reconcile_before_finalization() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = match store
            .enqueue_outbound(&intent("cmd-4", SideEffectClass::Irreversible))
            .unwrap()
        {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected disposition: {other:?}"),
        };
        store.claim_outbox("worker-a", 110, 50, 10).unwrap();
        store
            .record_execution(entry_id, "worker-a", &confirmed("cmd-4"), 150)
            .unwrap();

        assert!(matches!(
            store.finalize_outbound(entry_id, 160),
            Err(RuntimeError::IllegalOutboundTransition { .. })
        ));

        store
            .record_reconciliation(entry_id, &reconciliation("cmd-4"), 175)
            .unwrap();
        store.finalize_outbound(entry_id, 180).unwrap();
        assert_eq!(
            store.outbox_snapshot(entry_id).unwrap().stage,
            OutboundStage::Finalized
        );
    }

    #[test]
    fn outcome_for_another_operation_is_rejected() {
        let mut store = SqliteIntegrationStore::in_memory().unwrap();
        let entry_id = match store
            .enqueue_outbound(&intent("cmd-5", SideEffectClass::Irreversible))
            .unwrap()
        {
            EnqueueDisposition::Inserted(entry_id) => entry_id,
            other => panic!("unexpected disposition: {other:?}"),
        };
        store.claim_outbox("worker-a", 110, 50, 10).unwrap();
        assert!(matches!(
            store.record_execution(entry_id, "worker-a", &confirmed("different"), 150),
            Err(RuntimeError::OutcomeOperationMismatch { .. })
        ));
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
            entry_id = match store
                .enqueue_outbound(&intent("cmd-persist", SideEffectClass::Irreversible))
                .unwrap()
            {
                EnqueueDisposition::Inserted(entry_id) => entry_id,
                other => panic!("unexpected disposition: {other:?}"),
            };
        }

        let store = SqliteIntegrationStore::open(&path).unwrap();
        let snapshot = store.outbox_snapshot(entry_id).unwrap();
        assert_eq!(snapshot.stage, OutboundStage::OutboxCommitted);
        assert_eq!(snapshot.attempt_count, 0);
    }
}

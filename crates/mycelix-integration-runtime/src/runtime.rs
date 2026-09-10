//! Stable public runtime boundary for INT-03.
//!
//! `v31` owns the v3.1 durable semantic/storage contract. This thin shell adds
//! file-backed cross-handle freshness checks and SQLite write fences so a
//! process-local cache or check/write race cannot weaken causal-time or
//! exact-provider-operation guarantees when another process advances the same
//! database.

#[path = "v31.rs"]
mod v31;

pub use v31::{
    DispatchStarted, DurableOutboundIntent, EnqueueDisposition, ExecutionClaim,
    ExecutionRecordDisposition, InsertDisposition, OutboxSnapshot, PersistableInbound,
    ReconciliationHistoryItem, RecoverySummary, RuntimeError, RUNTIME_SEMANTIC_PROFILE_V31,
};

use mycelix_integration_core::{
    ConnectorInstanceId, ExecutionAttemptId, ExternalExecutionOutcome, ExternalOperationRef,
    ReconcileCursor, ReconciliationResult,
};
use rusqlite::{params, Connection, OptionalExtension};
use std::{
    path::{Path, PathBuf},
    time::Duration,
};

const OUTBOX_COMMITTED_STAGE_V2: i64 = 3;
const MAX_EXECUTION_ATTEMPTS_V01: i64 = 1_024;

pub struct SqliteIntegrationStore {
    inner: v31::SqliteIntegrationStore,
    path: Option<PathBuf>,
}

impl SqliteIntegrationStore {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RuntimeError> {
        let path = path.as_ref().to_path_buf();
        let inner = v31::SqliteIntegrationStore::open(&path)?;
        ensure_atomic_causal_fences(&path)?;
        Ok(Self {
            inner,
            path: Some(path),
        })
    }

    pub fn in_memory() -> Result<Self, RuntimeError> {
        Ok(Self {
            inner: v31::SqliteIntegrationStore::in_memory()?,
            path: None,
        })
    }

    pub fn insert_inbound(
        &mut self,
        inbound: &PersistableInbound,
    ) -> Result<InsertDisposition, RuntimeError> {
        self.inner.insert_inbound(inbound)
    }

    pub fn enqueue_outbound(
        &mut self,
        intent: &DurableOutboundIntent,
    ) -> Result<EnqueueDisposition, RuntimeError> {
        self.inner.enqueue_outbound(intent)
    }

    pub fn claim_outbox(
        &mut self,
        worker_id: &str,
        now_ms: i64,
        lease_duration_ms: i64,
        limit: usize,
    ) -> Result<Vec<ExecutionClaim>, RuntimeError> {
        self.require_fresh_claim_frontiers(now_ms, limit)?;
        self.inner
            .claim_outbox(worker_id, now_ms, lease_duration_ms, limit)
    }

    pub fn mark_dispatch_started(
        &mut self,
        entry_id: i64,
        attempt_id: &ExecutionAttemptId,
        worker_id: &str,
        now_ms: i64,
    ) -> Result<DispatchStarted, RuntimeError> {
        self.require_fresh_causal_time(entry_id, now_ms)?;
        self.inner
            .mark_dispatch_started(entry_id, attempt_id, worker_id, now_ms)
    }

    pub fn recover_expired_claims(
        &mut self,
        now_ms: i64,
    ) -> Result<RecoverySummary, RuntimeError> {
        self.inner.recover_expired_claims(now_ms)
    }

    pub fn record_execution(
        &mut self,
        entry_id: i64,
        attempt_id: &ExecutionAttemptId,
        worker_id: &str,
        outcome: &ExternalExecutionOutcome,
        now_ms: i64,
    ) -> Result<ExecutionRecordDisposition, RuntimeError> {
        self.require_fresh_causal_time(entry_id, now_ms)?;
        self.require_fresh_operation_binding(entry_id, outcome.operation(), false)?;
        self.inner
            .record_execution(entry_id, attempt_id, worker_id, outcome, now_ms)
    }

    pub fn record_reconciliation(
        &mut self,
        entry_id: i64,
        reconciliation: &ReconciliationResult,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
        self.require_fresh_causal_time(entry_id, now_ms)?;
        self.require_fresh_operation_binding(entry_id, &reconciliation.operation, true)?;
        self.inner
            .record_reconciliation(entry_id, reconciliation, now_ms)
    }

    pub fn reconciliation_history(
        &self,
        entry_id: i64,
    ) -> Result<Vec<ReconciliationHistoryItem>, RuntimeError> {
        self.inner.reconciliation_history(entry_id)
    }

    pub fn finalize_outbound(
        &mut self,
        entry_id: i64,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
        self.require_fresh_causal_time(entry_id, now_ms)?;
        self.inner.finalize_outbound(entry_id, now_ms)
    }

    pub fn checkpoint_reconciliation(
        &mut self,
        cursor: &ReconcileCursor,
        updated_at_ms: i64,
    ) -> Result<(), RuntimeError> {
        self.inner
            .checkpoint_reconciliation(cursor, updated_at_ms)
    }

    pub fn load_reconciliation_checkpoint(
        &self,
        connector_instance: &ConnectorInstanceId,
    ) -> Result<Option<ReconcileCursor>, RuntimeError> {
        self.inner
            .load_reconciliation_checkpoint(connector_instance)
    }

    pub fn outbox_snapshot(&self, entry_id: i64) -> Result<OutboxSnapshot, RuntimeError> {
        self.inner.outbox_snapshot(entry_id)
    }

    pub fn quarantine_reason(&self, entry_id: i64) -> Result<Option<String>, RuntimeError> {
        self.inner.quarantine_reason(entry_id)
    }

    pub fn provider_operation_binding(
        &self,
        entry_id: i64,
    ) -> Result<Option<String>, RuntimeError> {
        if let Some(path) = &self.path {
            return load_durable_provider_operation(path, entry_id);
        }
        self.inner.provider_operation_binding(entry_id)
    }

    fn require_fresh_causal_time(&self, entry_id: i64, now_ms: i64) -> Result<(), RuntimeError> {
        if now_ms < 0 {
            return Err(RuntimeError::InvalidTimestamp);
        }
        let Some(path) = &self.path else {
            return Ok(());
        };
        if load_durable_causal_frontier(path, entry_id)?
            .is_some_and(|frontier| now_ms < frontier)
        {
            return Err(RuntimeError::InvalidTimestamp);
        }
        Ok(())
    }

    fn require_fresh_claim_frontiers(
        &self,
        now_ms: i64,
        limit: usize,
    ) -> Result<(), RuntimeError> {
        if now_ms < 0 {
            return Err(RuntimeError::InvalidTimestamp);
        }
        let Some(path) = &self.path else {
            return Ok(());
        };
        let conn = open_aux(path)?;
        let mut statement = conn.prepare(
            "SELECT entry_id FROM integration_outbox\n\
             WHERE stage = ?1 AND attempt_count < ?2\n\
             ORDER BY entry_id ASC LIMIT ?3",
        )?;
        let rows = statement.query_map(
            params![
                OUTBOX_COMMITTED_STAGE_V2,
                MAX_EXECUTION_ATTEMPTS_V01,
                limit as i64,
            ],
            |row| row.get::<_, i64>(0),
        )?;
        for row in rows {
            self.require_fresh_causal_time(row?, now_ms)?;
        }
        Ok(())
    }

    fn require_fresh_operation_binding(
        &self,
        entry_id: i64,
        incoming: &ExternalOperationRef,
        reconciliation: bool,
    ) -> Result<(), RuntimeError> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        let Some(established) = load_durable_provider_operation(path, entry_id)? else {
            return Ok(());
        };
        let compatible = incoming
            .provider_operation
            .as_ref()
            .is_some_and(|candidate| candidate.as_str() == established);
        if compatible {
            return Ok(());
        }
        if reconciliation {
            Err(RuntimeError::ReconciliationOperationMismatch { entry_id })
        } else {
            Err(RuntimeError::OutcomeOperationMismatch { entry_id })
        }
    }
}

fn ensure_atomic_causal_fences(path: &Path) -> Result<(), RuntimeError> {
    let conn = open_aux(path)?;
    conn.execute_batch(
        r#"
        CREATE TRIGGER IF NOT EXISTS integration_causal_outbox_update_before
        BEFORE UPDATE OF updated_at_ms ON integration_outbox
        WHEN NEW.updated_at_ms < COALESCE(
            (
                SELECT MAX(ts) FROM (
                    SELECT OLD.updated_at_ms AS ts
                    UNION ALL
                    SELECT observed_at_ms FROM integration_execution_observation
                    WHERE entry_id = OLD.entry_id
                    UNION ALL
                    SELECT recorded_at_ms FROM integration_reconciliation_history
                    WHERE entry_id = OLD.entry_id
                )
            ),
            NEW.updated_at_ms
        )
        BEGIN
            SELECT RAISE(ABORT, 'runtime causal time rollback');
        END;

        CREATE TRIGGER IF NOT EXISTS integration_causal_execution_observation_before
        BEFORE INSERT ON integration_execution_observation
        WHEN NEW.observed_at_ms < COALESCE(
            (
                SELECT MAX(ts) FROM (
                    SELECT updated_at_ms AS ts FROM integration_outbox
                    WHERE entry_id = NEW.entry_id
                    UNION ALL
                    SELECT observed_at_ms FROM integration_execution_observation
                    WHERE entry_id = NEW.entry_id
                    UNION ALL
                    SELECT recorded_at_ms FROM integration_reconciliation_history
                    WHERE entry_id = NEW.entry_id
                )
            ),
            NEW.observed_at_ms
        )
        BEGIN
            SELECT RAISE(ABORT, 'runtime causal time rollback');
        END;

        CREATE TRIGGER IF NOT EXISTS integration_causal_reconciliation_before
        BEFORE INSERT ON integration_reconciliation_history
        WHEN NEW.recorded_at_ms < COALESCE(
            (
                SELECT MAX(ts) FROM (
                    SELECT updated_at_ms AS ts FROM integration_outbox
                    WHERE entry_id = NEW.entry_id
                    UNION ALL
                    SELECT observed_at_ms FROM integration_execution_observation
                    WHERE entry_id = NEW.entry_id
                    UNION ALL
                    SELECT recorded_at_ms FROM integration_reconciliation_history
                    WHERE entry_id = NEW.entry_id
                )
            ),
            NEW.recorded_at_ms
        )
        BEGIN
            SELECT RAISE(ABORT, 'runtime causal time rollback');
        END;
        "#,
    )?;
    Ok(())
}

fn load_durable_causal_frontier(
    path: &Path,
    entry_id: i64,
) -> Result<Option<i64>, RuntimeError> {
    let conn = open_aux(path)?;
    conn.query_row(
        "SELECT MAX(ts) FROM (\n\
             SELECT updated_at_ms AS ts FROM integration_outbox WHERE entry_id = ?1\n\
             UNION ALL\n\
             SELECT observed_at_ms AS ts FROM integration_execution_observation WHERE entry_id = ?1\n\
             UNION ALL\n\
             SELECT recorded_at_ms AS ts FROM integration_reconciliation_history WHERE entry_id = ?1\n\
         )",
        params![entry_id],
        |row| row.get(0),
    )
    .map_err(RuntimeError::from)
}

fn load_durable_provider_operation(
    path: &Path,
    entry_id: i64,
) -> Result<Option<String>, RuntimeError> {
    let conn = open_aux(path)?;
    conn.query_row(
        "SELECT provider_operation FROM integration_runtime_operation_binding WHERE entry_id = ?1",
        params![entry_id],
        |row| row.get(0),
    )
    .optional()
    .map_err(RuntimeError::from)
}

fn open_aux(path: &Path) -> Result<Connection, RuntimeError> {
    let conn = Connection::open(path)?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    Ok(conn)
}

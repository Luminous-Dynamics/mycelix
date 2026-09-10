//! Stable public runtime boundary for INT-03.
//!
//! `v31` owns the v3.1 durable semantic/storage contract. This shell adds
//! file-backed cross-handle freshness, lock-first SQLite enforcement repair,
//! and connector-checkpoint CAS semantics so process-local caches or
//! check/write races cannot weaken durable causal meaning.

#[path = "storage_guard.rs"]
mod storage_guard;
#[path = "v31.rs"]
mod v31;

pub use storage_guard::{
    ReconciliationCheckpointSnapshot, RUNTIME_ENFORCEMENT_PROFILE_V3,
};
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
    collections::BTreeMap,
    path::{Path, PathBuf},
    time::Duration,
};

const OUTBOX_COMMITTED_STAGE_V2: i64 = 3;
const MAX_EXECUTION_ATTEMPTS_V01: i64 = 1_024;

pub struct SqliteIntegrationStore {
    inner: v31::SqliteIntegrationStore,
    path: Option<PathBuf>,
    in_memory_checkpoint_times_ms: BTreeMap<String, i64>,
}

impl SqliteIntegrationStore {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RuntimeError> {
        let path = path.as_ref().to_path_buf();

        if storage_guard::is_initialized_file_store(&path)? {
            // Existing initialized stores are admitted lock-first: acquire the
            // SQLite write lock and validate semantic/history/enforcement state
            // before v3.1 opens and loads process-local security caches.
            storage_guard::harden_file_store(&path, RUNTIME_SEMANTIC_PROFILE_V31)?;
        } else {
            // Fresh/uninitialized files need one bootstrap open to create the
            // structural + semantic substrate. That handle is discarded before
            // enforcement repair, and no public runtime is exposed yet.
            let bootstrap = v31::SqliteIntegrationStore::open(&path)?;
            drop(bootstrap);
            storage_guard::harden_file_store(&path, RUNTIME_SEMANTIC_PROFILE_V31)?;
        }

        // Load process-local caches only after the durable store has been
        // validated/reconstructed under the storage guard transaction.
        let inner = v31::SqliteIntegrationStore::open(&path)?;

        Ok(Self {
            inner,
            path: Some(path),
            in_memory_checkpoint_times_ms: BTreeMap::new(),
        })
    }

    pub fn in_memory() -> Result<Self, RuntimeError> {
        Ok(Self {
            inner: v31::SqliteIntegrationStore::in_memory()?,
            path: None,
            in_memory_checkpoint_times_ms: BTreeMap::new(),
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
        if let Some(path) = &self.path {
            return storage_guard::checkpoint_file_cas(path, cursor, updated_at_ms);
        }

        let key = cursor.connector_instance.as_str().to_owned();
        let existing_cursor = self.inner.load_reconciliation_checkpoint(&cursor.connector_instance)?;
        let existing = match existing_cursor.as_ref() {
            Some(stored) => {
                let stored_at_ms = self
                    .in_memory_checkpoint_times_ms
                    .get(&key)
                    .copied()
                    .ok_or_else(|| {
                        RuntimeError::StoredIdentifier(
                            "in-memory reconciliation checkpoint is missing its causal timestamp"
                                .to_owned(),
                        )
                    })?;
                Some((stored, stored_at_ms))
            }
            None => None,
        };

        let should_write =
            storage_guard::validate_in_memory_checkpoint(existing, cursor, updated_at_ms)?;
        if should_write {
            self.inner
                .checkpoint_reconciliation(cursor, updated_at_ms)?;
            self.in_memory_checkpoint_times_ms.insert(key, updated_at_ms);
        }
        Ok(())
    }

    pub fn load_reconciliation_checkpoint(
        &self,
        connector_instance: &ConnectorInstanceId,
    ) -> Result<Option<ReconcileCursor>, RuntimeError> {
        Ok(self
            .load_reconciliation_checkpoint_snapshot(connector_instance)?
            .map(|snapshot| snapshot.cursor))
    }

    pub fn load_reconciliation_checkpoint_snapshot(
        &self,
        connector_instance: &ConnectorInstanceId,
    ) -> Result<Option<ReconciliationCheckpointSnapshot>, RuntimeError> {
        if let Some(path) = &self.path {
            return storage_guard::load_checkpoint_snapshot(path, connector_instance);
        }

        let Some(cursor) = self.inner.load_reconciliation_checkpoint(connector_instance)? else {
            return Ok(None);
        };
        let updated_at_ms = self
            .in_memory_checkpoint_times_ms
            .get(connector_instance.as_str())
            .copied()
            .ok_or_else(|| {
                RuntimeError::StoredIdentifier(
                    "in-memory reconciliation checkpoint is missing its causal timestamp".to_owned(),
                )
            })?;
        Ok(Some(ReconciliationCheckpointSnapshot {
            cursor,
            updated_at_ms,
        }))
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

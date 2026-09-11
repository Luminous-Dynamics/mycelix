//! INT-03 storage-semantics v3.1 safety facade.
//!
//! The original SQLite implementation remains intact in `lib.rs` and is loaded
//! as an internal engine. This public facade adds semantic-producer identity,
//! durable monotonic provider-operation binding, per-entry causal-time checks,
//! lease-expiry demotion, and entry-local recovery quarantine.

#[path = "lib.rs"]
mod legacy;

pub use legacy::{
    DispatchStarted, DurableOutboundIntent, EnqueueDisposition, ExecutionClaim,
    ExecutionRecordDisposition, InsertDisposition, OutboxSnapshot, PersistableInbound,
    ReconciliationHistoryItem, RecoverySummary, RuntimeError,
};

use mycelix_integration_core::{
    ConnectorInstanceId, ExecutionAttemptId, ExternalExecutionOutcome, ExternalOperationRef,
    OutboundStage, ReconcileCursor, ReconciliationResult,
};
use rusqlite::{params, Connection, OptionalExtension};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    time::Duration,
};

/// Durable semantic producer/profile identity for the v3.1 safety facade.
///
/// This is deliberately distinct from SQLite `PRAGMA user_version`. A change
/// that makes provider-operation identity durable is itself a semantic change
/// and therefore receives a new producer identity rather than silently reusing
/// the earlier v3 marker.
pub const RUNTIME_SEMANTIC_PROFILE_V31: &str =
    "mycelix-integration-runtime/semantic-profile-v3.1";

const STRUCTURAL_SCHEMA_V2: i64 = 2;
const LEGACY_V1_GENERIC_REJECTED_STAGE: i64 = 6;
const V2_REJECTION_SLOT: i64 = 7;
const OUTBOX_COMMITTED_STAGE_V2: i64 = 3;
const DISPATCH_STARTED_STAGE_V2: i64 = 5;
const MAX_EXECUTION_ATTEMPTS_V01: i64 = 1_024;
const MAX_EXECUTION_OBSERVATIONS_V01: i64 = 4_096;

pub struct SqliteIntegrationStore {
    inner: legacy::SqliteIntegrationStore,
    path: Option<PathBuf>,
    causal_frontier_ms: BTreeMap<i64, i64>,
    provider_operation_bindings: BTreeMap<i64, String>,
}

impl SqliteIntegrationStore {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RuntimeError> {
        let path = path.as_ref().to_path_buf();
        preflight_semantic_identity(&path)?;
        let inner = legacy::SqliteIntegrationStore::open(&path)?;
        ensure_semantic_identity(&path)?;
        ensure_operation_binding_infrastructure(&path)?;
        let causal_frontier_ms = load_causal_frontiers(&path)?;
        let provider_operation_bindings = load_provider_operation_bindings(&path)?;
        Ok(Self {
            inner,
            path: Some(path),
            causal_frontier_ms,
            provider_operation_bindings,
        })
    }

    pub fn in_memory() -> Result<Self, RuntimeError> {
        Ok(Self {
            inner: legacy::SqliteIntegrationStore::in_memory()?,
            path: None,
            causal_frontier_ms: BTreeMap::new(),
            provider_operation_bindings: BTreeMap::new(),
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
        let disposition = self.inner.enqueue_outbound(intent)?;
        if let EnqueueDisposition::Inserted(entry_id) = disposition {
            self.causal_frontier_ms.insert(entry_id, intent.created_at_ms);
        }
        Ok(disposition)
    }

    pub fn claim_outbox(
        &mut self,
        worker_id: &str,
        now_ms: i64,
        lease_duration_ms: i64,
        limit: usize,
    ) -> Result<Vec<ExecutionClaim>, RuntimeError> {
        self.isolate_poisoned_recovery_entries(now_ms)?;
        self.require_claim_candidates_not_in_future(now_ms, limit)?;
        let claims = self
            .inner
            .claim_outbox(worker_id, now_ms, lease_duration_ms, limit)?;
        for claim in &claims {
            self.advance_frontier(claim.entry_id, now_ms);
        }
        Ok(claims)
    }

    pub fn mark_dispatch_started(
        &mut self,
        entry_id: i64,
        attempt_id: &ExecutionAttemptId,
        worker_id: &str,
        now_ms: i64,
    ) -> Result<DispatchStarted, RuntimeError> {
        self.require_causal_time(entry_id, now_ms)?;
        let started = self
            .inner
            .mark_dispatch_started(entry_id, attempt_id, worker_id, now_ms)?;
        self.advance_frontier(entry_id, now_ms);
        Ok(started)
    }

    pub fn recover_expired_claims(
        &mut self,
        now_ms: i64,
    ) -> Result<RecoverySummary, RuntimeError> {
        self.isolate_poisoned_recovery_entries(now_ms)?;
        let summary = self.inner.recover_expired_claims(now_ms)?;
        self.refresh_frontiers_after_recovery(now_ms)?;
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
        self.require_causal_time(entry_id, now_ms)?;
        self.require_operation_compatible(entry_id, outcome.operation(), false)?;

        // Expiry independently removes current completion power. Recovery does
        // not need to have run first; the late exact result remains evidence.
        let snapshot = self.inner.outbox_snapshot(entry_id)?;
        if snapshot.stage == OutboundStage::DispatchStarted
            && snapshot
                .lease_until_ms
                .is_some_and(|deadline| now_ms >= deadline)
        {
            self.isolate_poisoned_recovery_entries(now_ms)?;
            self.inner.recover_expired_claims(now_ms)?;
            self.advance_frontier(entry_id, now_ms);
        }

        let disposition = self.inner.record_execution(
            entry_id,
            attempt_id,
            worker_id,
            outcome,
            now_ms,
        )?;
        self.remember_provider_operation(entry_id, outcome.operation(), now_ms)?;
        self.advance_frontier(entry_id, now_ms);
        Ok(disposition)
    }

    pub fn record_reconciliation(
        &mut self,
        entry_id: i64,
        reconciliation: &ReconciliationResult,
        now_ms: i64,
    ) -> Result<(), RuntimeError> {
        self.require_causal_time(entry_id, now_ms)?;
        self.require_operation_compatible(entry_id, &reconciliation.operation, true)?;
        self.inner
            .record_reconciliation(entry_id, reconciliation, now_ms)?;
        self.remember_provider_operation(entry_id, &reconciliation.operation, now_ms)?;
        self.advance_frontier(entry_id, now_ms);
        Ok(())
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
        self.require_causal_time(entry_id, now_ms)?;
        self.inner.finalize_outbound(entry_id, now_ms)?;
        self.advance_frontier(entry_id, now_ms);
        Ok(())
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

    /// Returns the durable v3.1 quarantine reason for an entry, if any.
    pub fn quarantine_reason(&self, entry_id: i64) -> Result<Option<String>, RuntimeError> {
        let Some(path) = &self.path else {
            return Ok(None);
        };
        let conn = open_aux(path)?;
        if !table_exists(&conn, "integration_runtime_quarantine")? {
            return Ok(None);
        }
        conn.query_row(
            "SELECT reason FROM integration_runtime_quarantine WHERE entry_id = ?1",
            params![entry_id],
            |row| row.get(0),
        )
        .optional()
        .map_err(RuntimeError::from)
    }

    /// Returns the exact provider-operation binding established for this entry.
    /// This is durable identity metadata, not provider authority or success.
    pub fn provider_operation_binding(
        &self,
        entry_id: i64,
    ) -> Result<Option<String>, RuntimeError> {
        let snapshot = self.inner.outbox_snapshot(entry_id)?;
        self.established_provider_operation(entry_id, &snapshot)
    }

    fn require_causal_time(&self, entry_id: i64, now_ms: i64) -> Result<(), RuntimeError> {
        if now_ms < 0 {
            return Err(RuntimeError::InvalidTimestamp);
        }
        if self
            .causal_frontier_ms
            .get(&entry_id)
            .is_some_and(|frontier| now_ms < *frontier)
        {
            return Err(RuntimeError::InvalidTimestamp);
        }
        Ok(())
    }

    fn advance_frontier(&mut self, entry_id: i64, now_ms: i64) {
        self.causal_frontier_ms
            .entry(entry_id)
            .and_modify(|frontier| *frontier = (*frontier).max(now_ms))
            .or_insert(now_ms);
    }

    fn require_claim_candidates_not_in_future(
        &self,
        now_ms: i64,
        limit: usize,
    ) -> Result<(), RuntimeError> {
        if now_ms < 0 {
            return Err(RuntimeError::InvalidTimestamp);
        }

        let candidate_ids = if let Some(path) = &self.path {
            claim_candidate_ids(path, limit)?
        } else {
            let mut ids = Vec::new();
            for entry_id in self.causal_frontier_ms.keys().copied() {
                let snapshot = self.inner.outbox_snapshot(entry_id)?;
                if snapshot.stage == OutboundStage::OutboxCommitted
                    && i64::from(snapshot.attempt_count) < MAX_EXECUTION_ATTEMPTS_V01
                {
                    ids.push(entry_id);
                    if ids.len() == limit {
                        break;
                    }
                }
            }
            ids
        };

        for entry_id in candidate_ids {
            self.require_causal_time(entry_id, now_ms)?;
        }
        Ok(())
    }

    fn require_operation_compatible(
        &self,
        entry_id: i64,
        incoming: &ExternalOperationRef,
        reconciliation: bool,
    ) -> Result<(), RuntimeError> {
        let snapshot = self.inner.outbox_snapshot(entry_id)?;

        if snapshot.command_id != incoming.command_id
            || snapshot.connector_instance != incoming.connector_instance
        {
            return operation_mismatch(entry_id, reconciliation);
        }

        if let Some(established) = self.established_provider_operation(entry_id, &snapshot)? {
            let compatible = incoming
                .provider_operation
                .as_ref()
                .is_some_and(|candidate| candidate.as_str() == established);
            if !compatible {
                return operation_mismatch(entry_id, reconciliation);
            }
        }
        Ok(())
    }

    fn established_provider_operation(
        &self,
        entry_id: i64,
        snapshot: &OutboxSnapshot,
    ) -> Result<Option<String>, RuntimeError> {
        let durable = self.provider_operation_bindings.get(&entry_id).cloned();
        let cached = cached_provider_operation(snapshot)?;
        match (durable, cached) {
            (Some(a), Some(b)) if a != b => Err(RuntimeError::StoredIdentifier(format!(
                "provider-operation binding conflicts with cached runtime state: {a} vs {b}"
            ))),
            (Some(a), _) => Ok(Some(a)),
            (None, Some(b)) => Ok(Some(b)),
            (None, None) => Ok(None),
        }
    }

    fn remember_provider_operation(
        &mut self,
        entry_id: i64,
        operation: &ExternalOperationRef,
        established_at_ms: i64,
    ) -> Result<(), RuntimeError> {
        let Some(provider_operation) = &operation.provider_operation else {
            return Ok(());
        };
        match self.provider_operation_bindings.get(&entry_id) {
            Some(existing) if existing != provider_operation.as_str() => {
                return Err(RuntimeError::StoredIdentifier(format!(
                    "provider-operation binding changed after accepted evidence: {existing} vs {}",
                    provider_operation.as_str()
                )));
            }
            Some(_) => return Ok(()),
            None => {}
        }

        // File-backed writes are already protected atomically by triggers on the
        // legacy append-only history tables. The explicit write is idempotent and
        // makes the invariant obvious; in-memory stores keep the same binding in
        // this map because there is no auxiliary connection to their database.
        if let Some(path) = &self.path {
            persist_provider_operation_binding(
                path,
                entry_id,
                provider_operation.as_str(),
                established_at_ms,
            )?;
        }
        self.provider_operation_bindings
            .insert(entry_id, provider_operation.as_str().to_owned());
        Ok(())
    }

    fn isolate_poisoned_recovery_entries(&self, now_ms: i64) -> Result<(), RuntimeError> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        if now_ms < 0 {
            return Err(RuntimeError::InvalidTimestamp);
        }

        let conn = open_aux(path)?;
        ensure_quarantine_table(&conn)?;
        let poisoned = {
            let mut statement = conn.prepare(
                "SELECT o.entry_id\n\
                 FROM integration_outbox o\n\
                 WHERE o.stage = ?1\n\
                   AND o.lease_until_ms IS NOT NULL\n\
                   AND o.lease_until_ms <= ?2\n\
                   AND (SELECT COUNT(*)\n\
                        FROM integration_execution_observation e\n\
                        WHERE e.entry_id = o.entry_id) >= ?3\n\
                 ORDER BY o.entry_id ASC",
            )?;
            let rows = statement.query_map(
                params![
                    DISPATCH_STARTED_STAGE_V2,
                    now_ms,
                    MAX_EXECUTION_OBSERVATIONS_V01,
                ],
                |row| row.get::<_, i64>(0),
            )?;
            rows.collect::<Result<Vec<_>, _>>()?
        };

        if poisoned.is_empty() {
            return Ok(());
        }

        let tx = conn.unchecked_transaction()?;
        for entry_id in poisoned {
            tx.execute(
                "INSERT INTO integration_runtime_quarantine (entry_id, reason, quarantined_at_ms)\n\
                 VALUES (?1, ?2, ?3)\n\
                 ON CONFLICT(entry_id) DO NOTHING",
                params![
                    entry_id,
                    "execution-observation-history-exhausted-before-stale-recovery",
                    now_ms,
                ],
            )?;
            tx.execute(
                "UPDATE integration_outbox\n\
                 SET worker_id = NULL, lease_until_ms = NULL,\n\
                     updated_at_ms = CASE WHEN updated_at_ms < ?1 THEN ?1 ELSE updated_at_ms END\n\
                 WHERE entry_id = ?2 AND stage = ?3",
                params![now_ms, entry_id, DISPATCH_STARTED_STAGE_V2],
            )?;
        }
        tx.commit()?;
        Ok(())
    }

    fn refresh_frontiers_after_recovery(&mut self, now_ms: i64) -> Result<(), RuntimeError> {
        if let Some(path) = &self.path {
            self.causal_frontier_ms = load_causal_frontiers(path)?;
            return Ok(());
        }

        let ids: Vec<i64> = self.causal_frontier_ms.keys().copied().collect();
        for entry_id in ids {
            let snapshot = self.inner.outbox_snapshot(entry_id)?;
            if matches!(
                snapshot.stage,
                OutboundStage::OutboxCommitted | OutboundStage::Ambiguous
            ) && snapshot.lease_until_ms.is_none()
            {
                self.advance_frontier(entry_id, now_ms);
            }
        }
        Ok(())
    }
}

fn operation_mismatch(
    entry_id: i64,
    reconciliation: bool,
) -> Result<(), RuntimeError> {
    if reconciliation {
        Err(RuntimeError::ReconciliationOperationMismatch { entry_id })
    } else {
        Err(RuntimeError::OutcomeOperationMismatch { entry_id })
    }
}

fn cached_provider_operation(snapshot: &OutboxSnapshot) -> Result<Option<String>, RuntimeError> {
    let mut established: Option<String> = None;
    let mut observe = |operation: &ExternalOperationRef| -> Result<(), RuntimeError> {
        let Some(provider_operation) = &operation.provider_operation else {
            return Ok(());
        };
        match &established {
            Some(existing) if existing != provider_operation.as_str() => Err(
                RuntimeError::StoredIdentifier(format!(
                    "cached runtime state contains conflicting provider-operation bindings: {existing} vs {}",
                    provider_operation.as_str()
                )),
            ),
            Some(_) => Ok(()),
            None => {
                established = Some(provider_operation.as_str().to_owned());
                Ok(())
            }
        }
    };

    if let Some(outcome) = &snapshot.outcome {
        observe(outcome.operation())?;
    }
    if let Some(reconciliation) = &snapshot.reconciliation {
        observe(&reconciliation.operation)?;
    }
    Ok(established)
}

fn preflight_semantic_identity(path: &Path) -> Result<(), RuntimeError> {
    if !path.exists() {
        return Ok(());
    }
    let Ok(metadata) = std::fs::metadata(path) else {
        return Ok(());
    };
    if metadata.len() == 0 {
        return Ok(());
    }

    let conn = open_aux(path)?;
    if !table_exists(&conn, "integration_outbox")? {
        return Ok(());
    }

    if table_exists(&conn, "integration_runtime_semantics")? {
        let stored: Option<String> = conn
            .query_row(
                "SELECT semantic_profile FROM integration_runtime_semantics WHERE singleton = 1",
                [],
                |row| row.get(0),
            )
            .optional()?;
        return match stored.as_deref() {
            Some(RUNTIME_SEMANTIC_PROFILE_V31) => Ok(()),
            Some(other) => Err(RuntimeError::StoredIdentifier(format!(
                "unsupported integration runtime semantic profile: {other}"
            ))),
            None => Err(RuntimeError::StoredIdentifier(
                "integration runtime semantic metadata exists without a producer identity".to_owned(),
            )),
        };
    }

    let row_count: i64 = conn.query_row(
        "SELECT COUNT(*) FROM integration_outbox",
        [],
        |row| row.get(0),
    )?;
    if row_count > 0 {
        let structural_version: i64 =
            conn.pragma_query_value(None, "user_version", |row| row.get(0))?;
        let detail = if structural_version == 0
            && stage_exists(&conn, LEGACY_V1_GENERIC_REJECTED_STAGE)?
        {
            "implicit-v1 generic Rejected cannot be relabeled as AuthorityDenied or RejectedBeforeCommit"
        } else if structural_version == STRUCTURAL_SCHEMA_V2
            && stage_exists(&conn, V2_REJECTION_SLOT)?
        {
            "schema-v2 rejection slot lacks semantic producer identity"
        } else {
            "non-empty runtime lacks a semantic producer identity"
        };
        return Err(RuntimeError::StoredIdentifier(format!(
            "semantic migration underdetermined: {detail}"
        )));
    }

    Ok(())
}

fn ensure_semantic_identity(path: &Path) -> Result<(), RuntimeError> {
    let conn = open_aux(path)?;
    conn.execute_batch(
        "CREATE TABLE IF NOT EXISTS integration_runtime_semantics (\n\
             singleton INTEGER PRIMARY KEY CHECK (singleton = 1),\n\
             semantic_profile TEXT NOT NULL\n\
         );",
    )?;
    conn.execute(
        "INSERT INTO integration_runtime_semantics (singleton, semantic_profile)\n\
         VALUES (1, ?1)\n\
         ON CONFLICT(singleton) DO NOTHING",
        params![RUNTIME_SEMANTIC_PROFILE_V31],
    )?;
    let stored: String = conn.query_row(
        "SELECT semantic_profile FROM integration_runtime_semantics WHERE singleton = 1",
        [],
        |row| row.get(0),
    )?;
    if stored != RUNTIME_SEMANTIC_PROFILE_V31 {
        return Err(RuntimeError::StoredIdentifier(format!(
            "unsupported integration runtime semantic profile: {stored}"
        )));
    }
    Ok(())
}

fn ensure_operation_binding_infrastructure(path: &Path) -> Result<(), RuntimeError> {
    let conn = open_aux(path)?;
    conn.execute_batch(
        r#"
        CREATE TABLE IF NOT EXISTS integration_runtime_operation_binding (
            entry_id INTEGER PRIMARY KEY,
            provider_operation TEXT NOT NULL,
            established_at_ms INTEGER NOT NULL,
            FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
        );

        CREATE TRIGGER IF NOT EXISTS integration_bind_execution_operation_before
        BEFORE INSERT ON integration_execution_observation
        WHEN json_valid(NEW.outcome_json)
        BEGIN
            SELECT RAISE(ABORT, 'provider operation binding conflict')
            WHERE json_extract(NEW.outcome_json, '$.data.operation.provider_operation') IS NOT NULL
              AND EXISTS (
                  SELECT 1 FROM integration_runtime_operation_binding b
                  WHERE b.entry_id = NEW.entry_id
                    AND b.provider_operation != json_extract(
                        NEW.outcome_json,
                        '$.data.operation.provider_operation'
                    )
              );
        END;

        CREATE TRIGGER IF NOT EXISTS integration_bind_execution_operation_after
        AFTER INSERT ON integration_execution_observation
        WHEN json_valid(NEW.outcome_json)
        BEGIN
            INSERT OR IGNORE INTO integration_runtime_operation_binding (
                entry_id, provider_operation, established_at_ms
            )
            SELECT
                NEW.entry_id,
                json_extract(NEW.outcome_json, '$.data.operation.provider_operation'),
                NEW.observed_at_ms
            WHERE json_extract(NEW.outcome_json, '$.data.operation.provider_operation') IS NOT NULL;
        END;

        CREATE TRIGGER IF NOT EXISTS integration_bind_reconciliation_operation_before
        BEFORE INSERT ON integration_reconciliation_history
        WHEN json_valid(NEW.result_json)
        BEGIN
            SELECT RAISE(ABORT, 'provider operation binding conflict')
            WHERE json_extract(NEW.result_json, '$.operation.provider_operation') IS NOT NULL
              AND EXISTS (
                  SELECT 1 FROM integration_runtime_operation_binding b
                  WHERE b.entry_id = NEW.entry_id
                    AND b.provider_operation != json_extract(
                        NEW.result_json,
                        '$.operation.provider_operation'
                    )
              );
        END;

        CREATE TRIGGER IF NOT EXISTS integration_bind_reconciliation_operation_after
        AFTER INSERT ON integration_reconciliation_history
        WHEN json_valid(NEW.result_json)
        BEGIN
            INSERT OR IGNORE INTO integration_runtime_operation_binding (
                entry_id, provider_operation, established_at_ms
            )
            SELECT
                NEW.entry_id,
                json_extract(NEW.result_json, '$.operation.provider_operation'),
                NEW.recorded_at_ms
            WHERE json_extract(NEW.result_json, '$.operation.provider_operation') IS NOT NULL;
        END;
        "#,
    )?;
    Ok(())
}

fn persist_provider_operation_binding(
    path: &Path,
    entry_id: i64,
    provider_operation: &str,
    established_at_ms: i64,
) -> Result<(), RuntimeError> {
    let conn = open_aux(path)?;
    let existing: Option<String> = conn
        .query_row(
            "SELECT provider_operation FROM integration_runtime_operation_binding WHERE entry_id = ?1",
            params![entry_id],
            |row| row.get(0),
        )
        .optional()?;
    match existing {
        Some(existing) if existing != provider_operation => {
            return Err(RuntimeError::StoredIdentifier(format!(
                "durable provider-operation binding conflict: {existing} vs {provider_operation}"
            )));
        }
        Some(_) => return Ok(()),
        None => {}
    }
    conn.execute(
        "INSERT INTO integration_runtime_operation_binding (\n\
             entry_id, provider_operation, established_at_ms\n\
         ) VALUES (?1, ?2, ?3)",
        params![entry_id, provider_operation, established_at_ms],
    )?;
    Ok(())
}

fn load_provider_operation_bindings(
    path: &Path,
) -> Result<BTreeMap<i64, String>, RuntimeError> {
    let conn = open_aux(path)?;
    let mut statement = conn.prepare(
        "SELECT entry_id, provider_operation\n\
         FROM integration_runtime_operation_binding\n\
         ORDER BY entry_id ASC",
    )?;
    let rows = statement.query_map([], |row| {
        Ok((row.get::<_, i64>(0)?, row.get::<_, String>(1)?))
    })?;
    Ok(rows.collect::<Result<BTreeMap<_, _>, _>>()?)
}

fn ensure_quarantine_table(conn: &Connection) -> Result<(), RuntimeError> {
    conn.execute_batch(
        "CREATE TABLE IF NOT EXISTS integration_runtime_quarantine (\n\
             entry_id INTEGER PRIMARY KEY,\n\
             reason TEXT NOT NULL,\n\
             quarantined_at_ms INTEGER NOT NULL,\n\
             FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)\n\
         );",
    )?;
    Ok(())
}

fn load_causal_frontiers(path: &Path) -> Result<BTreeMap<i64, i64>, RuntimeError> {
    let conn = open_aux(path)?;
    let mut statement = conn.prepare(
        "SELECT entry_id, MAX(ts)\n\
         FROM (\n\
             SELECT entry_id, updated_at_ms AS ts FROM integration_outbox\n\
             UNION ALL\n\
             SELECT entry_id, observed_at_ms AS ts FROM integration_execution_observation\n\
             UNION ALL\n\
             SELECT entry_id, recorded_at_ms AS ts FROM integration_reconciliation_history\n\
         )\n\
         GROUP BY entry_id\n\
         ORDER BY entry_id ASC",
    )?;
    let rows = statement.query_map([], |row| {
        Ok((row.get::<_, i64>(0)?, row.get::<_, i64>(1)?))
    })?;
    Ok(rows.collect::<Result<BTreeMap<_, _>, _>>()?)
}

fn claim_candidate_ids(path: &Path, limit: usize) -> Result<Vec<i64>, RuntimeError> {
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
    Ok(rows.collect::<Result<Vec<_>, _>>()?)
}

fn open_aux(path: &Path) -> Result<Connection, RuntimeError> {
    let conn = Connection::open(path)?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    Ok(conn)
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

fn stage_exists(conn: &Connection, stage: i64) -> Result<bool, RuntimeError> {
    let exists: Option<i64> = conn
        .query_row(
            "SELECT 1 FROM integration_outbox WHERE stage = ?1 LIMIT 1",
            params![stage],
            |row| row.get(0),
        )
        .optional()?;
    Ok(exists.is_some())
}

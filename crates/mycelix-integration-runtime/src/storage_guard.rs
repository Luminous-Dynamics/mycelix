use crate::RuntimeError;
use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, DigestAlgorithm, ExternalExecutionOutcome,
    ExternalOpaqueId, ExternalOperationRef, IntegrationCommandId, ReconcileCursor,
    ReconciliationResult,
};
use rusqlite::{params, Connection, OptionalExtension, Transaction, TransactionBehavior};
use std::{
    collections::{btree_map::Entry, BTreeMap},
    path::Path,
    time::Duration,
};

/// Identity of the reconstructable SQLite enforcement machinery used by INT-03.
///
/// This is deliberately separate from the durable semantic producer identity:
/// changing trigger/reconstruction enforcement does not reinterpret historical
/// records, but an opened store must know which enforcement contract is active.
pub const RUNTIME_ENFORCEMENT_PROFILE_V2: &str =
    "mycelix-integration-runtime/enforcement-profile-v2";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReconciliationCheckpointSnapshot {
    pub cursor: ReconcileCursor,
    pub updated_at_ms: i64,
}

/// Returns whether the file already contains the INT-03 runtime schema.
///
/// This is discovery only. No trust decision is made until `harden_file_store`
/// acquires its `IMMEDIATE` transaction and validates the semantic producer ID.
pub(crate) fn is_initialized_file_store(path: &Path) -> Result<bool, RuntimeError> {
    if !path.exists() {
        return Ok(false);
    }
    let metadata = std::fs::metadata(path).map_err(|error| {
        RuntimeError::StoredIdentifier(format!("failed to inspect runtime store: {error}"))
    })?;
    if metadata.len() == 0 {
        return Ok(false);
    }
    let conn = open_aux(path)?;
    table_exists(&conn, "integration_outbox")
}

/// Atomically admits and repairs an initialized file-backed runtime.
///
/// The write lock is acquired before semantic identity, history, derived-state,
/// or trigger enforcement is trusted. Append-only typed history is the source
/// material; indexes and triggers are reconstructable enforcement machinery.
pub(crate) fn harden_file_store(
    path: &Path,
    expected_semantic_profile: &str,
) -> Result<(), RuntimeError> {
    let mut conn = open_aux(path)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;

    require_semantic_profile(&tx, expected_semantic_profile)?;
    ensure_binding_table(&tx)?;
    let bindings = collect_typed_provider_operation_bindings(&tx)?;
    rebuild_provider_operation_bindings(&tx, &bindings)?;
    replace_operation_subject_and_binding_triggers(&tx)?;
    replace_causal_time_triggers(&tx)?;
    record_enforcement_profile(&tx)?;

    tx.commit()?;
    Ok(())
}

pub(crate) fn checkpoint_file_cas(
    path: &Path,
    cursor: &ReconcileCursor,
    updated_at_ms: i64,
) -> Result<(), RuntimeError> {
    validate_timestamp(updated_at_ms)?;
    let mut conn = open_aux(path)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;

    let existing: Option<(String, i64, Vec<u8>, i64)> = tx
        .query_row(
            "SELECT cursor, commitment_algorithm, commitment_digest, updated_at_ms\n\
             FROM integration_reconcile_checkpoint WHERE connector_instance = ?1",
            params![cursor.connector_instance.as_str()],
            |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?, row.get(3)?)),
        )
        .optional()?;

    match existing {
        None => {
            tx.execute(
                "INSERT INTO integration_reconcile_checkpoint (\n\
                    connector_instance, cursor, commitment_algorithm, commitment_digest, updated_at_ms\n\
                 ) VALUES (?1, ?2, ?3, ?4, ?5)",
                params![
                    cursor.connector_instance.as_str(),
                    cursor.cursor.as_str(),
                    digest_algorithm_to_i64(cursor.checkpoint_commitment.algorithm),
                    cursor.checkpoint_commitment.digest.as_slice(),
                    updated_at_ms,
                ],
            )?;
        }
        Some((stored_cursor, stored_algorithm, stored_digest, stored_at_ms)) => {
            if updated_at_ms < stored_at_ms {
                return Err(RuntimeError::StoredIdentifier(format!(
                    "reconciliation checkpoint rollback for {}: {updated_at_ms} < {stored_at_ms}",
                    cursor.connector_instance.as_str()
                )));
            }

            let exact_same = stored_cursor == cursor.cursor.as_str()
                && stored_algorithm
                    == digest_algorithm_to_i64(cursor.checkpoint_commitment.algorithm)
                && stored_digest.as_slice() == cursor.checkpoint_commitment.digest.as_slice();

            if updated_at_ms == stored_at_ms {
                if !exact_same {
                    return Err(RuntimeError::StoredIdentifier(format!(
                        "reconciliation checkpoint equal-time conflict for {} at {updated_at_ms}",
                        cursor.connector_instance.as_str()
                    )));
                }
                tx.commit()?;
                return Ok(());
            }

            tx.execute(
                "UPDATE integration_reconcile_checkpoint\n\
                 SET cursor = ?1, commitment_algorithm = ?2, commitment_digest = ?3, updated_at_ms = ?4\n\
                 WHERE connector_instance = ?5",
                params![
                    cursor.cursor.as_str(),
                    digest_algorithm_to_i64(cursor.checkpoint_commitment.algorithm),
                    cursor.checkpoint_commitment.digest.as_slice(),
                    updated_at_ms,
                    cursor.connector_instance.as_str(),
                ],
            )?;
        }
    }

    tx.commit()?;
    Ok(())
}

pub(crate) fn load_checkpoint_snapshot(
    path: &Path,
    connector: &ConnectorInstanceId,
) -> Result<Option<ReconciliationCheckpointSnapshot>, RuntimeError> {
    let conn = open_aux(path)?;
    let stored: Option<(String, i64, Vec<u8>, i64)> = conn
        .query_row(
            "SELECT cursor, commitment_algorithm, commitment_digest, updated_at_ms\n\
             FROM integration_reconcile_checkpoint WHERE connector_instance = ?1",
            params![connector.as_str()],
            |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?, row.get(3)?)),
        )
        .optional()?;

    stored
        .map(|(cursor, algorithm, digest, updated_at_ms)| {
            Ok(ReconciliationCheckpointSnapshot {
                cursor: ReconcileCursor {
                    connector_instance: connector.clone(),
                    cursor: ExternalOpaqueId::new(cursor)
                        .map_err(|error| RuntimeError::StoredIdentifier(error.to_string()))?,
                    checkpoint_commitment: commitment_from_parts(algorithm, digest)?,
                },
                updated_at_ms,
            })
        })
        .transpose()
}

pub(crate) fn validate_in_memory_checkpoint(
    existing: Option<(&ReconcileCursor, i64)>,
    incoming: &ReconcileCursor,
    updated_at_ms: i64,
) -> Result<bool, RuntimeError> {
    validate_timestamp(updated_at_ms)?;
    let Some((stored, stored_at_ms)) = existing else {
        return Ok(true);
    };
    if updated_at_ms < stored_at_ms {
        return Err(RuntimeError::StoredIdentifier(format!(
            "reconciliation checkpoint rollback for {}: {updated_at_ms} < {stored_at_ms}",
            incoming.connector_instance.as_str()
        )));
    }
    if updated_at_ms == stored_at_ms {
        if stored != incoming {
            return Err(RuntimeError::StoredIdentifier(format!(
                "reconciliation checkpoint equal-time conflict for {} at {updated_at_ms}",
                incoming.connector_instance.as_str()
            )));
        }
        return Ok(false);
    }
    Ok(true)
}

fn require_semantic_profile(
    tx: &Transaction<'_>,
    expected_semantic_profile: &str,
) -> Result<(), RuntimeError> {
    if !transaction_table_exists(tx, "integration_runtime_semantics")? {
        return Err(RuntimeError::StoredIdentifier(
            "initialized runtime store has no semantic producer identity".to_owned(),
        ));
    }

    let stored: Option<String> = tx
        .query_row(
            "SELECT semantic_profile FROM integration_runtime_semantics WHERE singleton = 1",
            [],
            |row| row.get(0),
        )
        .optional()?;
    match stored.as_deref() {
        Some(profile) if profile == expected_semantic_profile => Ok(()),
        Some(profile) => Err(RuntimeError::StoredIdentifier(format!(
            "unsupported integration runtime semantic profile: {profile}"
        ))),
        None => Err(RuntimeError::StoredIdentifier(
            "integration runtime semantic metadata exists without a producer identity".to_owned(),
        )),
    }
}

fn ensure_binding_table(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    tx.execute_batch(
        r#"
        CREATE TABLE IF NOT EXISTS integration_runtime_operation_binding (
            entry_id INTEGER PRIMARY KEY,
            provider_operation TEXT NOT NULL,
            established_at_ms INTEGER NOT NULL,
            FOREIGN KEY(entry_id) REFERENCES integration_outbox(entry_id)
        );
        "#,
    )?;
    Ok(())
}

fn collect_typed_provider_operation_bindings(
    tx: &Transaction<'_>,
) -> Result<BTreeMap<i64, (String, i64)>, RuntimeError> {
    let mut bindings = BTreeMap::new();

    {
        let mut statement = tx.prepare(
            "SELECT e.entry_id, e.outcome_json, e.observed_at_ms, o.command_id, o.connector_instance\n\
             FROM integration_execution_observation e\n\
             JOIN integration_outbox o ON o.entry_id = e.entry_id\n\
             ORDER BY e.observation_id ASC",
        )?;
        let rows = statement.query_map([], |row| {
            Ok((
                row.get::<_, i64>(0)?,
                row.get::<_, Vec<u8>>(1)?,
                row.get::<_, i64>(2)?,
                row.get::<_, String>(3)?,
                row.get::<_, String>(4)?,
            ))
        })?;
        for row in rows {
            let (entry_id, bytes, observed_at_ms, command_id, connector_instance) = row?;
            let outcome: ExternalExecutionOutcome = serde_json::from_slice(&bytes).map_err(|error| {
                RuntimeError::StoredIdentifier(format!(
                    "execution history for outbox entry {entry_id} is not a typed INT-02 outcome: {error}"
                ))
            })?;
            validate_operation_subject(
                entry_id,
                outcome.operation(),
                &command_id,
                &connector_instance,
                "execution",
            )?;
            observe_provider_binding(
                &mut bindings,
                entry_id,
                outcome.operation(),
                observed_at_ms,
            )?;
        }
    }

    {
        let mut statement = tx.prepare(
            "SELECT r.entry_id, r.result_json, r.recorded_at_ms, o.command_id, o.connector_instance\n\
             FROM integration_reconciliation_history r\n\
             JOIN integration_outbox o ON o.entry_id = r.entry_id\n\
             ORDER BY r.sequence ASC",
        )?;
        let rows = statement.query_map([], |row| {
            Ok((
                row.get::<_, i64>(0)?,
                row.get::<_, Vec<u8>>(1)?,
                row.get::<_, i64>(2)?,
                row.get::<_, String>(3)?,
                row.get::<_, String>(4)?,
            ))
        })?;
        for row in rows {
            let (entry_id, bytes, recorded_at_ms, command_id, connector_instance) = row?;
            let reconciliation: ReconciliationResult =
                serde_json::from_slice(&bytes).map_err(|error| {
                    RuntimeError::StoredIdentifier(format!(
                        "reconciliation history for outbox entry {entry_id} is not a typed INT-02 result: {error}"
                    ))
                })?;
            validate_operation_subject(
                entry_id,
                &reconciliation.operation,
                &command_id,
                &connector_instance,
                "reconciliation",
            )?;
            observe_provider_binding(
                &mut bindings,
                entry_id,
                &reconciliation.operation,
                recorded_at_ms,
            )?;
        }
    }

    Ok(bindings)
}

fn validate_operation_subject(
    entry_id: i64,
    operation: &ExternalOperationRef,
    stored_command_id: &str,
    stored_connector_instance: &str,
    history_kind: &str,
) -> Result<(), RuntimeError> {
    let owner_command = IntegrationCommandId::new(stored_command_id.to_owned()).map_err(|error| {
        RuntimeError::StoredIdentifier(format!(
            "outbox entry {entry_id} has invalid command identity: {error}"
        ))
    })?;
    let owner_connector =
        ConnectorInstanceId::new(stored_connector_instance.to_owned()).map_err(|error| {
            RuntimeError::StoredIdentifier(format!(
                "outbox entry {entry_id} has invalid connector identity: {error}"
            ))
        })?;

    if operation.command_id != owner_command || operation.connector_instance != owner_connector {
        return Err(RuntimeError::StoredIdentifier(format!(
            "{history_kind} history subject mismatch for outbox entry {entry_id}"
        )));
    }

    if let Some(provider_operation) = &operation.provider_operation {
        ExternalOpaqueId::new(provider_operation.as_str().to_owned()).map_err(|error| {
            RuntimeError::StoredIdentifier(format!(
                "{history_kind} history for outbox entry {entry_id} has invalid provider operation: {error}"
            ))
        })?;
    }
    Ok(())
}

fn observe_provider_binding(
    bindings: &mut BTreeMap<i64, (String, i64)>,
    entry_id: i64,
    operation: &ExternalOperationRef,
    established_at_ms: i64,
) -> Result<(), RuntimeError> {
    let Some(provider_operation) = &operation.provider_operation else {
        return Ok(());
    };
    validate_timestamp(established_at_ms)?;
    let candidate = provider_operation.as_str();
    match bindings.entry(entry_id) {
        Entry::Vacant(slot) => {
            slot.insert((candidate.to_owned(), established_at_ms));
        }
        Entry::Occupied(mut slot) if slot.get().0 == candidate => {
            slot.get_mut().1 = slot.get().1.min(established_at_ms);
        }
        Entry::Occupied(slot) => {
            return Err(RuntimeError::StoredIdentifier(format!(
                "conflicting provider-operation history for outbox entry {entry_id}: {} vs {candidate}",
                slot.get().0
            )));
        }
    }
    Ok(())
}

fn rebuild_provider_operation_bindings(
    tx: &Transaction<'_>,
    bindings: &BTreeMap<i64, (String, i64)>,
) -> Result<(), RuntimeError> {
    tx.execute("DELETE FROM integration_runtime_operation_binding", [])?;
    for (entry_id, (provider_operation, established_at_ms)) in bindings {
        tx.execute(
            "INSERT INTO integration_runtime_operation_binding (\n\
                 entry_id, provider_operation, established_at_ms\n\
             ) VALUES (?1, ?2, ?3)",
            params![entry_id, provider_operation, established_at_ms],
        )?;
    }
    Ok(())
}

fn replace_operation_subject_and_binding_triggers(
    tx: &Transaction<'_>,
) -> Result<(), RuntimeError> {
    tx.execute_batch(
        r#"
        DROP TRIGGER IF EXISTS integration_validate_execution_subject_before;
        DROP TRIGGER IF EXISTS integration_validate_reconciliation_subject_before;
        DROP TRIGGER IF EXISTS integration_bind_execution_operation_before;
        DROP TRIGGER IF EXISTS integration_bind_execution_operation_after;
        DROP TRIGGER IF EXISTS integration_bind_reconciliation_operation_before;
        DROP TRIGGER IF EXISTS integration_bind_reconciliation_operation_after;

        CREATE TRIGGER integration_validate_execution_subject_before
        BEFORE INSERT ON integration_execution_observation
        BEGIN
            SELECT RAISE(ABORT, 'execution operation subject mismatch')
            WHERE NOT json_valid(CAST(NEW.outcome_json AS TEXT))
               OR json_extract(CAST(NEW.outcome_json AS TEXT), '$.data.operation.command_id') IS NULL
               OR json_extract(CAST(NEW.outcome_json AS TEXT), '$.data.operation.connector_instance') IS NULL
               OR NOT EXISTS (
                   SELECT 1 FROM integration_outbox o
                   WHERE o.entry_id = NEW.entry_id
                     AND o.command_id = json_extract(
                         CAST(NEW.outcome_json AS TEXT),
                         '$.data.operation.command_id'
                     )
                     AND o.connector_instance = json_extract(
                         CAST(NEW.outcome_json AS TEXT),
                         '$.data.operation.connector_instance'
                     )
               );
        END;

        CREATE TRIGGER integration_validate_reconciliation_subject_before
        BEFORE INSERT ON integration_reconciliation_history
        BEGIN
            SELECT RAISE(ABORT, 'reconciliation operation subject mismatch')
            WHERE NOT json_valid(CAST(NEW.result_json AS TEXT))
               OR json_extract(CAST(NEW.result_json AS TEXT), '$.operation.command_id') IS NULL
               OR json_extract(CAST(NEW.result_json AS TEXT), '$.operation.connector_instance') IS NULL
               OR NOT EXISTS (
                   SELECT 1 FROM integration_outbox o
                   WHERE o.entry_id = NEW.entry_id
                     AND o.command_id = json_extract(
                         CAST(NEW.result_json AS TEXT),
                         '$.operation.command_id'
                     )
                     AND o.connector_instance = json_extract(
                         CAST(NEW.result_json AS TEXT),
                         '$.operation.connector_instance'
                     )
               );
        END;

        CREATE TRIGGER integration_bind_execution_operation_before
        BEFORE INSERT ON integration_execution_observation
        WHEN json_valid(CAST(NEW.outcome_json AS TEXT))
        BEGIN
            SELECT RAISE(ABORT, 'provider operation binding conflict')
            WHERE json_extract(
                    CAST(NEW.outcome_json AS TEXT),
                    '$.data.operation.provider_operation'
                  ) IS NOT NULL
              AND EXISTS (
                  SELECT 1 FROM integration_runtime_operation_binding b
                  WHERE b.entry_id = NEW.entry_id
                    AND b.provider_operation != json_extract(
                        CAST(NEW.outcome_json AS TEXT),
                        '$.data.operation.provider_operation'
                    )
              );
        END;

        CREATE TRIGGER integration_bind_execution_operation_after
        AFTER INSERT ON integration_execution_observation
        WHEN json_valid(CAST(NEW.outcome_json AS TEXT))
        BEGIN
            INSERT OR IGNORE INTO integration_runtime_operation_binding (
                entry_id, provider_operation, established_at_ms
            )
            SELECT
                NEW.entry_id,
                json_extract(
                    CAST(NEW.outcome_json AS TEXT),
                    '$.data.operation.provider_operation'
                ),
                NEW.observed_at_ms
            WHERE json_extract(
                    CAST(NEW.outcome_json AS TEXT),
                    '$.data.operation.provider_operation'
                  ) IS NOT NULL;
        END;

        CREATE TRIGGER integration_bind_reconciliation_operation_before
        BEFORE INSERT ON integration_reconciliation_history
        WHEN json_valid(CAST(NEW.result_json AS TEXT))
        BEGIN
            SELECT RAISE(ABORT, 'provider operation binding conflict')
            WHERE json_extract(
                    CAST(NEW.result_json AS TEXT),
                    '$.operation.provider_operation'
                  ) IS NOT NULL
              AND EXISTS (
                  SELECT 1 FROM integration_runtime_operation_binding b
                  WHERE b.entry_id = NEW.entry_id
                    AND b.provider_operation != json_extract(
                        CAST(NEW.result_json AS TEXT),
                        '$.operation.provider_operation'
                    )
              );
        END;

        CREATE TRIGGER integration_bind_reconciliation_operation_after
        AFTER INSERT ON integration_reconciliation_history
        WHEN json_valid(CAST(NEW.result_json AS TEXT))
        BEGIN
            INSERT OR IGNORE INTO integration_runtime_operation_binding (
                entry_id, provider_operation, established_at_ms
            )
            SELECT
                NEW.entry_id,
                json_extract(
                    CAST(NEW.result_json AS TEXT),
                    '$.operation.provider_operation'
                ),
                NEW.recorded_at_ms
            WHERE json_extract(
                    CAST(NEW.result_json AS TEXT),
                    '$.operation.provider_operation'
                  ) IS NOT NULL;
        END;
        "#,
    )?;
    Ok(())
}

fn replace_causal_time_triggers(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    tx.execute_batch(
        r#"
        DROP TRIGGER IF EXISTS integration_causal_outbox_update_before;
        DROP TRIGGER IF EXISTS integration_causal_execution_observation_before;
        DROP TRIGGER IF EXISTS integration_causal_reconciliation_before;

        CREATE TRIGGER integration_causal_outbox_update_before
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

        CREATE TRIGGER integration_causal_execution_observation_before
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

        CREATE TRIGGER integration_causal_reconciliation_before
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

fn record_enforcement_profile(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    tx.execute_batch(
        r#"
        CREATE TABLE IF NOT EXISTS integration_runtime_enforcement (
            singleton INTEGER PRIMARY KEY CHECK (singleton = 1),
            enforcement_profile TEXT NOT NULL
        );
        "#,
    )?;
    tx.execute(
        "INSERT INTO integration_runtime_enforcement (singleton, enforcement_profile)\n\
         VALUES (1, ?1)\n\
         ON CONFLICT(singleton) DO UPDATE SET enforcement_profile = excluded.enforcement_profile",
        params![RUNTIME_ENFORCEMENT_PROFILE_V2],
    )?;
    Ok(())
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
    let algorithm = match algorithm {
        1 => DigestAlgorithm::Sha256,
        value => {
            return Err(RuntimeError::InvalidStoredEnum {
                field: "digest_algorithm",
                value,
            })
        }
    };
    let mut fixed = [0_u8; 32];
    fixed.copy_from_slice(&digest);
    Ok(ContentCommitment {
        algorithm,
        digest: fixed,
    })
}

fn digest_algorithm_to_i64(algorithm: DigestAlgorithm) -> i64 {
    match algorithm {
        DigestAlgorithm::Sha256 => 1,
    }
}

fn validate_timestamp(value: i64) -> Result<(), RuntimeError> {
    if value < 0 {
        return Err(RuntimeError::InvalidTimestamp);
    }
    Ok(())
}

fn table_exists(conn: &Connection, name: &str) -> Result<bool, RuntimeError> {
    conn.query_row(
        "SELECT EXISTS(\n\
             SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1\n\
         )",
        params![name],
        |row| row.get(0),
    )
    .map_err(RuntimeError::from)
}

fn transaction_table_exists(tx: &Transaction<'_>, name: &str) -> Result<bool, RuntimeError> {
    tx.query_row(
        "SELECT EXISTS(\n\
             SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1\n\
         )",
        params![name],
        |row| row.get(0),
    )
    .map_err(RuntimeError::from)
}

fn open_aux(path: &Path) -> Result<Connection, RuntimeError> {
    let conn = Connection::open(path)?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    Ok(conn)
}

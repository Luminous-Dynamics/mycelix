use crate::RuntimeError;
use mycelix_integration_core::{
    ContentCommitment, DigestAlgorithm, ExternalOpaqueId, ReconcileCursor,
};
use rusqlite::{params, Connection, OptionalExtension, Transaction, TransactionBehavior};
use std::{path::Path, time::Duration};

/// Identity of the reconstructable SQLite enforcement machinery used by INT-03.
///
/// This is deliberately separate from the durable semantic producer identity:
/// changing a trigger implementation does not by itself reinterpret historical
/// records, but an opened store must know which enforcement contract it has
/// installed.
pub const RUNTIME_ENFORCEMENT_PROFILE_V1: &str =
    "mycelix-integration-runtime/enforcement-profile-v1";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReconciliationCheckpointSnapshot {
    pub cursor: ReconcileCursor,
    pub updated_at_ms: i64,
}

pub(crate) fn harden_file_store(path: &Path) -> Result<(), RuntimeError> {
    let mut conn = open_aux(path)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;

    ensure_binding_table(&tx)?;
    validate_provider_operation_history(&tx)?;
    rebuild_provider_operation_bindings(&tx)?;
    replace_operation_binding_triggers(&tx)?;
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
    connector: &mycelix_integration_core::ConnectorInstanceId,
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

fn validate_provider_operation_history(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    let malformed_execution: Option<i64> = tx
        .query_row(
            "SELECT entry_id FROM integration_execution_observation\n\
             WHERE NOT json_valid(outcome_json) LIMIT 1",
            [],
            |row| row.get(0),
        )
        .optional()?;
    if let Some(entry_id) = malformed_execution {
        return Err(RuntimeError::StoredIdentifier(format!(
            "execution history for outbox entry {entry_id} is not valid JSON"
        )));
    }

    let malformed_reconciliation: Option<i64> = tx
        .query_row(
            "SELECT entry_id FROM integration_reconciliation_history\n\
             WHERE NOT json_valid(result_json) LIMIT 1",
            [],
            |row| row.get(0),
        )
        .optional()?;
    if let Some(entry_id) = malformed_reconciliation {
        return Err(RuntimeError::StoredIdentifier(format!(
            "reconciliation history for outbox entry {entry_id} is not valid JSON"
        )));
    }

    let conflict: Option<i64> = tx
        .query_row(
            r#"
            WITH operation_evidence(entry_id, provider_operation) AS (
                SELECT
                    entry_id,
                    json_extract(outcome_json, '$.data.operation.provider_operation')
                FROM integration_execution_observation
                WHERE json_extract(outcome_json, '$.data.operation.provider_operation') IS NOT NULL
                UNION ALL
                SELECT
                    entry_id,
                    json_extract(result_json, '$.operation.provider_operation')
                FROM integration_reconciliation_history
                WHERE json_extract(result_json, '$.operation.provider_operation') IS NOT NULL
            )
            SELECT entry_id
            FROM operation_evidence
            GROUP BY entry_id
            HAVING COUNT(DISTINCT provider_operation) > 1
            LIMIT 1
            "#,
            [],
            |row| row.get(0),
        )
        .optional()?;
    if let Some(entry_id) = conflict {
        return Err(RuntimeError::StoredIdentifier(format!(
            "conflicting provider-operation history for outbox entry {entry_id}"
        )));
    }

    Ok(())
}

fn rebuild_provider_operation_bindings(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    tx.execute("DELETE FROM integration_runtime_operation_binding", [])?;
    tx.execute_batch(
        r#"
        INSERT INTO integration_runtime_operation_binding (
            entry_id, provider_operation, established_at_ms
        )
        WITH operation_evidence(entry_id, provider_operation, established_at_ms) AS (
            SELECT
                entry_id,
                json_extract(outcome_json, '$.data.operation.provider_operation'),
                observed_at_ms
            FROM integration_execution_observation
            WHERE json_extract(outcome_json, '$.data.operation.provider_operation') IS NOT NULL
            UNION ALL
            SELECT
                entry_id,
                json_extract(result_json, '$.operation.provider_operation'),
                recorded_at_ms
            FROM integration_reconciliation_history
            WHERE json_extract(result_json, '$.operation.provider_operation') IS NOT NULL
        )
        SELECT entry_id, provider_operation, MIN(established_at_ms)
        FROM operation_evidence
        GROUP BY entry_id, provider_operation;
        "#,
    )?;
    Ok(())
}

fn replace_operation_binding_triggers(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    tx.execute_batch(
        r#"
        DROP TRIGGER IF EXISTS integration_bind_execution_operation_before;
        DROP TRIGGER IF EXISTS integration_bind_execution_operation_after;
        DROP TRIGGER IF EXISTS integration_bind_reconciliation_operation_before;
        DROP TRIGGER IF EXISTS integration_bind_reconciliation_operation_after;

        CREATE TRIGGER integration_bind_execution_operation_before
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

        CREATE TRIGGER integration_bind_execution_operation_after
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

        CREATE TRIGGER integration_bind_reconciliation_operation_before
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

        CREATE TRIGGER integration_bind_reconciliation_operation_after
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
        params![RUNTIME_ENFORCEMENT_PROFILE_V1],
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

fn open_aux(path: &Path) -> Result<Connection, RuntimeError> {
    let conn = Connection::open(path)?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    Ok(conn)
}

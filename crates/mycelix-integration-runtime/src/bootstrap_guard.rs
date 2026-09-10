use crate::{storage_guard::FileStoreAdmission, RuntimeError};
use rusqlite::{params, Connection, OptionalExtension, Transaction, TransactionBehavior};
use std::{path::Path, time::Duration};

const EXPECTED_STRUCTURAL_SCHEMA_V2: i64 = 2;

const REQUIRED_STRUCTURAL_TABLES: &[&str] = &[
    "integration_inbound",
    "integration_outbox",
    "integration_execution_observation",
    "integration_reconciliation_history",
    "integration_reconcile_checkpoint",
];

const DURABLE_RECORD_TABLES: &[&str] = &[
    "integration_inbound",
    "integration_outbox",
    "integration_execution_observation",
    "integration_reconciliation_history",
    "integration_reconcile_checkpoint",
    "integration_runtime_operation_binding",
    "integration_runtime_quarantine",
    "integration_runtime_enforcement",
];

/// Refines the coarse structural bootstrap classification under a second
/// `IMMEDIATE` transaction before legacy v3.1 initialization is allowed to run.
///
/// Automatic semantic bootstrap is permitted only when the structural-v2 store
/// has no durable runtime records, no prior AUTOINCREMENT activity, and its
/// semantic producer metadata is absent/empty or already exactly the expected
/// profile. This lets a crash-interrupted *empty* bootstrap resume without
/// turning bootstrap recovery into a migration mechanism for historical data.
pub(crate) fn qualify_file_store_bootstrap(
    path: &Path,
    expected_semantic_profile: &str,
    _coarse: FileStoreAdmission,
) -> Result<FileStoreAdmission, RuntimeError> {
    let mut conn = open_aux(path)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;

    let structural_version: i64 = tx.query_row("PRAGMA user_version", [], |row| row.get(0))?;
    if structural_version != EXPECTED_STRUCTURAL_SCHEMA_V2 {
        tx.commit()?;
        return Ok(FileStoreAdmission::Initialized);
    }

    let semantic_profile = load_semantic_profile(&tx)?;
    if semantic_profile
        .as_deref()
        .is_some_and(|profile| profile != expected_semantic_profile)
    {
        tx.commit()?;
        return Ok(FileStoreAdmission::Initialized);
    }

    let required_schema_complete = required_structural_tables_exist(&tx)?;
    let durable_record_table = first_nonempty_durable_table(&tx)?;
    let prior_autoincrement_activity = has_prior_autoincrement_activity(&tx)?;

    if durable_record_table.is_some() || prior_autoincrement_activity {
        tx.commit()?;
        return Ok(FileStoreAdmission::Initialized);
    }

    // With zero durable records, establishing the expected producer identity
    // does not reinterpret historical state: there is no historical state to
    // promote. Keeping this write in the same transaction closes the race where
    // another opener observes an empty semantic table without its singleton row.
    ensure_expected_semantic_profile(&tx, expected_semantic_profile)?;

    let admission = if required_schema_complete {
        FileStoreAdmission::Initialized
    } else {
        FileStoreAdmission::NeedsBootstrap
    };
    tx.commit()?;
    Ok(admission)
}

fn load_semantic_profile(tx: &Transaction<'_>) -> Result<Option<String>, RuntimeError> {
    if !transaction_table_exists(tx, "integration_runtime_semantics")? {
        return Ok(None);
    }

    let row_count: i64 = tx.query_row(
        "SELECT COUNT(*) FROM integration_runtime_semantics",
        [],
        |row| row.get(0),
    )?;
    if row_count == 0 {
        return Ok(None);
    }
    if row_count != 1 {
        return Err(RuntimeError::StoredIdentifier(format!(
            "bootstrap semantic metadata has {row_count} rows; expected exactly one singleton"
        )));
    }

    let stored: Option<(i64, String)> = tx
        .query_row(
            "SELECT singleton, semantic_profile FROM integration_runtime_semantics LIMIT 1",
            [],
            |row| Ok((row.get(0)?, row.get(1)?)),
        )
        .optional()?;
    let Some((singleton, profile)) = stored else {
        return Ok(None);
    };
    if singleton != 1 {
        return Err(RuntimeError::StoredIdentifier(format!(
            "bootstrap semantic metadata uses invalid singleton {singleton}"
        )));
    }
    Ok(Some(profile))
}

fn ensure_expected_semantic_profile(
    tx: &Transaction<'_>,
    expected_semantic_profile: &str,
) -> Result<(), RuntimeError> {
    tx.execute_batch(
        "CREATE TABLE IF NOT EXISTS integration_runtime_semantics (\n\
             singleton INTEGER PRIMARY KEY CHECK (singleton = 1),\n\
             semantic_profile TEXT NOT NULL\n\
         );",
    )?;
    tx.execute(
        "INSERT INTO integration_runtime_semantics (singleton, semantic_profile)\n\
         VALUES (1, ?1)\n\
         ON CONFLICT(singleton) DO NOTHING",
        params![expected_semantic_profile],
    )?;

    let stored: String = tx.query_row(
        "SELECT semantic_profile FROM integration_runtime_semantics WHERE singleton = 1",
        [],
        |row| row.get(0),
    )?;
    if stored != expected_semantic_profile {
        return Err(RuntimeError::StoredIdentifier(format!(
            "unsupported integration runtime semantic profile: {stored}"
        )));
    }
    Ok(())
}

fn required_structural_tables_exist(tx: &Transaction<'_>) -> Result<bool, RuntimeError> {
    for &table in REQUIRED_STRUCTURAL_TABLES {
        if !transaction_table_exists(tx, table)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn first_nonempty_durable_table(
    tx: &Transaction<'_>,
) -> Result<Option<&'static str>, RuntimeError> {
    for &table in DURABLE_RECORD_TABLES {
        if !transaction_table_exists(tx, table)? {
            continue;
        }
        let sql = format!("SELECT EXISTS(SELECT 1 FROM {table} LIMIT 1)");
        let has_rows: bool = tx.query_row(&sql, [], |row| row.get(0))?;
        if has_rows {
            return Ok(Some(table));
        }
    }
    Ok(None)
}

fn has_prior_autoincrement_activity(tx: &Transaction<'_>) -> Result<bool, RuntimeError> {
    if !transaction_table_exists(tx, "sqlite_sequence")? {
        return Ok(false);
    }
    tx.query_row(
        "SELECT EXISTS(\n\
             SELECT 1 FROM sqlite_sequence\n\
             WHERE name IN (\n\
                 'integration_outbox',\n\
                 'integration_execution_observation',\n\
                 'integration_reconciliation_history'\n\
             ) AND seq > 0\n\
         )",
        [],
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

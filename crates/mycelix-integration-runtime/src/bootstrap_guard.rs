use crate::{RuntimeError, storage_guard::FileStoreAdmission};
use rusqlite::{Connection, OptionalExtension, Transaction, TransactionBehavior, params};
use std::{path::Path, time::Duration};

const EXPECTED_STRUCTURAL_SCHEMA_V2: i64 = 2;

/// Identity of the fresh-store/bootstrap-admission rules.
///
/// This is intentionally separate from semantic producer, structural manifest,
/// and initialized-store enforcement identities. v1 requires an unversioned
/// database to contain no user schema objects before Mycelix may stamp it as a
/// fresh structural-v2 bootstrap candidate, and partial-v2 recovery to contain
/// only recognized Mycelix bootstrap objects before semantic metadata is written.
pub const RUNTIME_BOOTSTRAP_PROFILE_V1: &str = "mycelix-integration-runtime/bootstrap-profile-v1";

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

const QUALIFIED_BOOTSTRAP_TABLES: &[&str] = &[
    "integration_inbound",
    "integration_outbox",
    "integration_execution_observation",
    "integration_reconciliation_history",
    "integration_reconcile_checkpoint",
    "integration_runtime_semantics",
    "integration_runtime_operation_binding",
    "integration_runtime_quarantine",
    "integration_runtime_enforcement",
];

const QUALIFIED_BOOTSTRAP_INDEXES: &[&str] = &[
    "integration_outbox_claim_idx",
    "integration_outbox_lease_idx",
    "integration_execution_observation_idx",
    "integration_reconciliation_history_idx",
];

const QUALIFIED_BOOTSTRAP_TRIGGERS: &[&str] = &[
    "integration_causal_outbox_update_before",
    "integration_validate_execution_subject_before",
    "integration_bind_execution_operation_before",
    "integration_bind_execution_operation_after",
    "integration_causal_execution_observation_before",
    "integration_validate_reconciliation_subject_before",
    "integration_bind_reconciliation_operation_before",
    "integration_bind_reconciliation_operation_after",
    "integration_causal_reconciliation_before",
];

/// Atomically handles the special `user_version = 0` bootstrap boundary before
/// the legacy coarse classifier is allowed to inspect the file.
///
/// A database is eligible for Mycelix structural-v2 adoption only when it is
/// genuinely schema-empty: no user table, index, trigger, or view exists. This
/// prevents an unrelated or historical unversioned SQLite database from being
/// mutated merely because it happens not to contain `integration_outbox`.
///
/// `Some(NeedsBootstrap)` means this transaction proved schema-emptiness and
/// stamped structural-v2 intent. `Some(Initialized)` means unversioned but not
/// pristine and therefore must fail strict admission without mutation. `None`
/// delegates nonzero structural versions to the existing classifier.
pub(crate) fn prepare_unversioned_file_store(
    path: &Path,
) -> Result<Option<FileStoreAdmission>, RuntimeError> {
    let mut conn = open_aux(path)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;

    let structural_version: i64 = tx.query_row("PRAGMA user_version", [], |row| row.get(0))?;
    if structural_version != 0 {
        tx.commit()?;
        return Ok(None);
    }

    let has_user_schema: bool = tx.query_row(
        "SELECT EXISTS(\n\
             SELECT 1 FROM sqlite_master\n\
             WHERE name NOT LIKE 'sqlite_%'\n\
               AND type IN ('table', 'index', 'trigger', 'view')\n\
         )",
        [],
        |row| row.get(0),
    )?;

    if has_user_schema {
        tx.commit()?;
        return Ok(Some(FileStoreAdmission::Initialized));
    }

    tx.pragma_update(None, "user_version", EXPECTED_STRUCTURAL_SCHEMA_V2)?;
    tx.commit()?;
    Ok(Some(FileStoreAdmission::NeedsBootstrap))
}

/// Refines the coarse structural bootstrap classification under a second
/// `IMMEDIATE` transaction before legacy v3.1 initialization is allowed to run.
///
/// Automatic semantic bootstrap is permitted only when the structural-v2 store
/// contains only recognized Mycelix bootstrap schema objects, has no durable
/// runtime records, no prior AUTOINCREMENT activity, and its semantic producer
/// metadata is absent/empty or already exactly the expected profile. This lets
/// a crash-interrupted *empty* bootstrap resume without turning bootstrap
/// recovery into a migration or foreign-database adoption mechanism.
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

    if first_unqualified_user_schema_object(&tx)?.is_some() {
        tx.commit()?;
        return Ok(FileStoreAdmission::Initialized);
    }

    let semantic_profile = load_semantic_profile(&tx)?;
    if let Some(profile) = semantic_profile.as_deref()
        && profile != expected_semantic_profile
    {
        return Err(RuntimeError::StoredIdentifier(format!(
            "unsupported integration runtime semantic profile: {profile}"
        )));
    }

    let required_schema_complete = required_structural_tables_exist(&tx)?;
    let durable_record_table = first_nonempty_durable_table(&tx)?;
    let prior_autoincrement_activity = has_prior_autoincrement_activity(&tx)?;

    if semantic_profile.is_none()
        && (durable_record_table.is_some() || prior_autoincrement_activity)
    {
        let observed_history = match durable_record_table {
            Some(table) => format!("durable history in {table}"),
            None => "prior AUTOINCREMENT activity".to_owned(),
        };
        return Err(RuntimeError::StoredIdentifier(format!(
            "integration runtime has {observed_history} but no semantic producer identity; automatic bootstrap is forbidden"
        )));
    }

    if durable_record_table.is_some() || prior_autoincrement_activity {
        tx.commit()?;
        return Ok(FileStoreAdmission::Initialized);
    }

    // With zero durable records and only recognized bootstrap schema objects,
    // establishing the expected producer identity does not reinterpret foreign
    // historical state. Keeping this write in the same transaction closes the
    // race where another opener observes an empty semantic table without its row.
    ensure_expected_semantic_profile(&tx, expected_semantic_profile)?;

    let admission = if required_schema_complete {
        FileStoreAdmission::Initialized
    } else {
        FileStoreAdmission::NeedsBootstrap
    };
    tx.commit()?;
    Ok(admission)
}

fn first_unqualified_user_schema_object(
    tx: &Transaction<'_>,
) -> Result<Option<(String, String)>, RuntimeError> {
    let mut statement = tx.prepare(
        "SELECT type, name FROM sqlite_master\n\
         WHERE name NOT LIKE 'sqlite_%'\n\
           AND type IN ('table', 'index', 'trigger', 'view')\n\
         ORDER BY type ASC, name ASC",
    )?;
    let rows = statement.query_map([], |row| {
        Ok((row.get::<_, String>(0)?, row.get::<_, String>(1)?))
    })?;

    for row in rows {
        let (object_type, name) = row?;
        let qualified = match object_type.as_str() {
            "table" => QUALIFIED_BOOTSTRAP_TABLES.contains(&name.as_str()),
            "index" => QUALIFIED_BOOTSTRAP_INDEXES.contains(&name.as_str()),
            "trigger" => QUALIFIED_BOOTSTRAP_TRIGGERS.contains(&name.as_str()),
            "view" => false,
            _ => false,
        };
        if !qualified {
            return Ok(Some((object_type, name)));
        }
    }
    Ok(None)
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

use crate::RuntimeError;
use rusqlite::{params, Connection, OptionalExtension, Transaction, TransactionBehavior};
use std::{collections::BTreeSet, path::Path, time::Duration};

/// Exact semantic manifest for the SQLite structures INT-03 relies on.
///
/// This is deliberately distinct from `PRAGMA user_version`: the pragma is a
/// declared schema generation, while this profile qualifies the concrete table,
/// key, foreign-key, index, and trigger structure observed at startup.
pub const RUNTIME_STRUCTURAL_MANIFEST_V1: &str =
    "mycelix-integration-runtime/structural-manifest-v1";

const EXPECTED_STRUCTURAL_SCHEMA_V2: i64 = 2;

#[derive(Debug, Clone, PartialEq, Eq)]
struct ColumnShape {
    cid: i64,
    name: String,
    declared_type: String,
    not_null: i64,
    default_value: Option<String>,
    pk_position: i64,
    hidden: i64,
}

#[derive(Debug, Clone, Copy)]
struct ColumnSpec {
    name: &'static str,
    declared_type: &'static str,
    not_null: i64,
    default_value: Option<&'static str>,
    pk_position: i64,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct ForeignKeyShape {
    referenced_table: String,
    from_column: String,
    to_column: String,
    on_update: String,
    on_delete: String,
    match_kind: String,
}

const INBOUND_COLUMNS: &[ColumnSpec] = &[
    col("connector_instance", "TEXT", 1, None, 1),
    col("event_id", "TEXT", 1, None, 2),
    col("commitment_algorithm", "INTEGER", 1, None, 0),
    col("commitment_digest", "BLOB", 1, None, 0),
    col("stage", "INTEGER", 1, None, 0),
    col("normalized_payload", "BLOB", 1, None, 0),
    col("received_at_ms", "INTEGER", 1, None, 0),
];

const OUTBOX_COLUMNS: &[ColumnSpec] = &[
    col("entry_id", "INTEGER", 0, None, 1),
    col("command_id", "TEXT", 1, None, 0),
    col("connector_instance", "TEXT", 1, None, 0),
    col("command_commitment_algorithm", "INTEGER", 1, None, 0),
    col("command_commitment_digest", "BLOB", 1, None, 0),
    col("authority_commitment_algorithm", "INTEGER", 1, None, 0),
    col("authority_commitment_digest", "BLOB", 1, None, 0),
    col("side_effect_class", "INTEGER", 1, None, 0),
    col("idempotency_key", "TEXT", 0, None, 0),
    col("command_bytes", "BLOB", 1, None, 0),
    col("stage", "INTEGER", 1, None, 0),
    col("worker_id", "TEXT", 0, None, 0),
    col("lease_until_ms", "INTEGER", 0, None, 0),
    col("attempt_count", "INTEGER", 1, Some("0"), 0),
    col("current_attempt_id", "TEXT", 0, None, 0),
    col("dispatch_started_at_ms", "INTEGER", 0, None, 0),
    col("outcome_json", "BLOB", 0, None, 0),
    col("reconciliation_json", "BLOB", 0, None, 0),
    col("created_at_ms", "INTEGER", 1, None, 0),
    col("updated_at_ms", "INTEGER", 1, None, 0),
];

const EXECUTION_COLUMNS: &[ColumnSpec] = &[
    col("observation_id", "INTEGER", 0, None, 1),
    col("entry_id", "INTEGER", 1, None, 0),
    col("attempt_id", "TEXT", 1, None, 0),
    col("outcome_json", "BLOB", 1, None, 0),
    col("observed_at_ms", "INTEGER", 1, None, 0),
    col("applied_to_current", "INTEGER", 1, None, 0),
];

const RECONCILIATION_COLUMNS: &[ColumnSpec] = &[
    col("sequence", "INTEGER", 0, None, 1),
    col("entry_id", "INTEGER", 1, None, 0),
    col("result_json", "BLOB", 1, None, 0),
    col("recorded_at_ms", "INTEGER", 1, None, 0),
];

const CHECKPOINT_COLUMNS: &[ColumnSpec] = &[
    col("connector_instance", "TEXT", 0, None, 1),
    col("cursor", "TEXT", 1, None, 0),
    col("commitment_algorithm", "INTEGER", 1, None, 0),
    col("commitment_digest", "BLOB", 1, None, 0),
    col("updated_at_ms", "INTEGER", 1, None, 0),
];

const SEMANTIC_COLUMNS: &[ColumnSpec] = &[
    col("singleton", "INTEGER", 0, None, 1),
    col("semantic_profile", "TEXT", 1, None, 0),
];

const BINDING_COLUMNS: &[ColumnSpec] = &[
    col("entry_id", "INTEGER", 0, None, 1),
    col("provider_operation", "TEXT", 1, None, 0),
    col("established_at_ms", "INTEGER", 1, None, 0),
];

const ENFORCEMENT_COLUMNS: &[ColumnSpec] = &[
    col("singleton", "INTEGER", 0, None, 1),
    col("enforcement_profile", "TEXT", 1, None, 0),
];

const QUARANTINE_COLUMNS: &[ColumnSpec] = &[
    col("entry_id", "INTEGER", 0, None, 1),
    col("reason", "TEXT", 1, None, 0),
    col("quarantined_at_ms", "INTEGER", 1, None, 0),
];

const MANAGED_TABLES: &[&str] = &[
    "integration_inbound",
    "integration_outbox",
    "integration_execution_observation",
    "integration_reconciliation_history",
    "integration_reconcile_checkpoint",
    "integration_runtime_semantics",
    "integration_runtime_operation_binding",
    "integration_runtime_enforcement",
    "integration_runtime_quarantine",
];

const EXPECTED_TRIGGERS: &[(&str, &str)] = &[
    (
        "integration_causal_outbox_update_before",
        "integration_outbox",
    ),
    (
        "integration_validate_execution_subject_before",
        "integration_execution_observation",
    ),
    (
        "integration_bind_execution_operation_before",
        "integration_execution_observation",
    ),
    (
        "integration_bind_execution_operation_after",
        "integration_execution_observation",
    ),
    (
        "integration_causal_execution_observation_before",
        "integration_execution_observation",
    ),
    (
        "integration_validate_reconciliation_subject_before",
        "integration_reconciliation_history",
    ),
    (
        "integration_bind_reconciliation_operation_before",
        "integration_reconciliation_history",
    ),
    (
        "integration_bind_reconciliation_operation_after",
        "integration_reconciliation_history",
    ),
    (
        "integration_causal_reconciliation_before",
        "integration_reconciliation_history",
    ),
];

const fn col(
    name: &'static str,
    declared_type: &'static str,
    not_null: i64,
    default_value: Option<&'static str>,
    pk_position: i64,
) -> ColumnSpec {
    ColumnSpec {
        name,
        declared_type,
        not_null,
        default_value,
        pk_position,
    }
}

/// Validates the immutable/non-reconstructable substrate before storage repair.
///
/// Missing derived binding/enforcement tables are allowed here because the
/// storage guard owns their reconstruction. If they already exist, their shape
/// must already be safe enough to mutate. Unknown triggers on managed tables are
/// rejected before repair so unqualified database code cannot fire during it.
pub(crate) fn validate_pre_repair_file_store(
    path: &Path,
    expected_semantic_profile: &str,
) -> Result<(), RuntimeError> {
    let mut conn = open_aux(path)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    require_schema_version(&tx)?;
    validate_required_base_tables(&tx)?;
    validate_table(&tx, "integration_runtime_semantics", SEMANTIC_COLUMNS, false, 1)?;
    require_semantic_singleton(&tx, expected_semantic_profile)?;

    validate_optional_table(
        &tx,
        "integration_runtime_operation_binding",
        BINDING_COLUMNS,
        false,
        0,
    )?;
    if transaction_table_exists(&tx, "integration_runtime_operation_binding")? {
        require_foreign_keys(
            &tx,
            "integration_runtime_operation_binding",
            &[fk("integration_outbox", "entry_id", "entry_id")],
        )?;
    }

    validate_optional_table(
        &tx,
        "integration_runtime_enforcement",
        ENFORCEMENT_COLUMNS,
        false,
        1,
    )?;
    require_enforcement_singleton_shape_if_present(&tx)?;

    validate_optional_table(
        &tx,
        "integration_runtime_quarantine",
        QUARANTINE_COLUMNS,
        false,
        0,
    )?;
    if transaction_table_exists(&tx, "integration_runtime_quarantine")? {
        require_foreign_keys(
            &tx,
            "integration_runtime_quarantine",
            &[fk("integration_outbox", "entry_id", "entry_id")],
        )?;
    }

    require_trigger_allowlist(&tx, false)?;
    validate_named_indexes(&tx, false)?;
    tx.commit()?;
    Ok(())
}

/// Validates the fully hardened schema immediately before the runtime is exposed.
///
/// This proves a point-in-time startup manifest, not lifetime tamper-proofness.
pub(crate) fn validate_hardened_file_store(
    path: &Path,
    expected_semantic_profile: &str,
    expected_enforcement_profile: &str,
) -> Result<(), RuntimeError> {
    let mut conn = open_aux(path)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    require_schema_version(&tx)?;
    validate_required_base_tables(&tx)?;
    validate_table(&tx, "integration_runtime_semantics", SEMANTIC_COLUMNS, false, 1)?;
    require_semantic_singleton(&tx, expected_semantic_profile)?;

    validate_table(
        &tx,
        "integration_runtime_operation_binding",
        BINDING_COLUMNS,
        false,
        0,
    )?;
    require_foreign_keys(
        &tx,
        "integration_runtime_operation_binding",
        &[fk("integration_outbox", "entry_id", "entry_id")],
    )?;

    validate_table(
        &tx,
        "integration_runtime_enforcement",
        ENFORCEMENT_COLUMNS,
        false,
        1,
    )?;
    require_exact_enforcement_singleton(&tx, expected_enforcement_profile)?;

    validate_optional_table(
        &tx,
        "integration_runtime_quarantine",
        QUARANTINE_COLUMNS,
        false,
        0,
    )?;
    if transaction_table_exists(&tx, "integration_runtime_quarantine")? {
        require_foreign_keys(
            &tx,
            "integration_runtime_quarantine",
            &[fk("integration_outbox", "entry_id", "entry_id")],
        )?;
    }

    require_trigger_allowlist(&tx, true)?;
    validate_named_indexes(&tx, true)?;
    require_no_foreign_key_violations(&tx)?;
    tx.commit()?;
    Ok(())
}

fn validate_required_base_tables(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    validate_table(tx, "integration_inbound", INBOUND_COLUMNS, false, 0)?;
    require_unique_sets(
        tx,
        "integration_inbound",
        &[&["connector_instance", "event_id"]],
    )?;
    require_foreign_keys(tx, "integration_inbound", &[])?;

    validate_table(tx, "integration_outbox", OUTBOX_COLUMNS, true, 0)?;
    require_unique_sets(tx, "integration_outbox", &[&["command_id"]])?;
    require_foreign_keys(tx, "integration_outbox", &[])?;

    validate_table(
        tx,
        "integration_execution_observation",
        EXECUTION_COLUMNS,
        true,
        0,
    )?;
    require_unique_sets(tx, "integration_execution_observation", &[])?;
    require_foreign_keys(
        tx,
        "integration_execution_observation",
        &[fk("integration_outbox", "entry_id", "entry_id")],
    )?;

    validate_table(
        tx,
        "integration_reconciliation_history",
        RECONCILIATION_COLUMNS,
        true,
        0,
    )?;
    require_unique_sets(tx, "integration_reconciliation_history", &[])?;
    require_foreign_keys(
        tx,
        "integration_reconciliation_history",
        &[fk("integration_outbox", "entry_id", "entry_id")],
    )?;

    validate_table(
        tx,
        "integration_reconcile_checkpoint",
        CHECKPOINT_COLUMNS,
        false,
        0,
    )?;
    require_unique_sets(
        tx,
        "integration_reconcile_checkpoint",
        &[&["connector_instance"]],
    )?;
    require_foreign_keys(tx, "integration_reconcile_checkpoint", &[])?;
    Ok(())
}

fn validate_table(
    tx: &Transaction<'_>,
    table: &str,
    expected: &[ColumnSpec],
    expect_autoincrement: bool,
    expected_check_count: usize,
) -> Result<(), RuntimeError> {
    if !transaction_table_exists(tx, table)? {
        return Err(schema_error(format!("required table {table} is missing")));
    }
    let actual = load_columns(tx, table)?;
    if actual.len() != expected.len() {
        return Err(schema_error(format!(
            "table {table} has {} columns; expected {}",
            actual.len(),
            expected.len()
        )));
    }

    for (position, (actual, expected)) in actual.iter().zip(expected).enumerate() {
        let default_matches = actual.default_value.as_deref().map(str::trim)
            == expected.default_value.map(str::trim);
        if actual.cid != position as i64
            || actual.name != expected.name
            || actual.declared_type.to_ascii_uppercase() != expected.declared_type
            || actual.not_null != expected.not_null
            || !default_matches
            || actual.pk_position != expected.pk_position
            || actual.hidden != 0
        {
            return Err(schema_error(format!(
                "table {table} column {position} does not match structural manifest"
            )));
        }
    }

    let sql = table_sql(tx, table)?;
    let upper = sql.to_ascii_uppercase();
    let has_autoincrement = upper.contains("AUTOINCREMENT");
    if has_autoincrement != expect_autoincrement {
        return Err(schema_error(format!(
            "table {table} AUTOINCREMENT contract differs from structural manifest"
        )));
    }
    if upper.contains("WITHOUT ROWID") || contains_strict_table_suffix(&upper) {
        return Err(schema_error(format!(
            "table {table} rowid/STRICT mode differs from structural manifest"
        )));
    }
    let check_count = upper.match_indices("CHECK").count();
    if check_count != expected_check_count {
        return Err(schema_error(format!(
            "table {table} has {check_count} CHECK constraints; expected {expected_check_count}"
        )));
    }
    Ok(())
}

fn validate_optional_table(
    tx: &Transaction<'_>,
    table: &str,
    expected: &[ColumnSpec],
    expect_autoincrement: bool,
    expected_check_count: usize,
) -> Result<(), RuntimeError> {
    if !transaction_table_exists(tx, table)? {
        return Ok(());
    }
    validate_table(
        tx,
        table,
        expected,
        expect_autoincrement,
        expected_check_count,
    )
}

fn load_columns(tx: &Transaction<'_>, table: &str) -> Result<Vec<ColumnShape>, RuntimeError> {
    let sql = format!("PRAGMA table_xinfo({})", quote_identifier(table));
    let mut statement = tx.prepare(&sql)?;
    let rows = statement.query_map([], |row| {
        Ok(ColumnShape {
            cid: row.get(0)?,
            name: row.get(1)?,
            declared_type: row.get(2)?,
            not_null: row.get(3)?,
            default_value: row.get(4)?,
            pk_position: row.get(5)?,
            hidden: row.get(6)?,
        })
    })?;
    rows.collect::<Result<Vec<_>, _>>().map_err(RuntimeError::from)
}

fn require_unique_sets(
    tx: &Transaction<'_>,
    table: &str,
    expected: &[&[&str]],
) -> Result<(), RuntimeError> {
    let sql = format!("PRAGMA index_list({})", quote_identifier(table));
    let mut statement = tx.prepare(&sql)?;
    let indexes = statement.query_map([], |row| {
        Ok((
            row.get::<_, String>(1)?,
            row.get::<_, i64>(2)?,
            row.get::<_, i64>(4)?,
        ))
    })?;

    let mut actual_sets = Vec::new();
    for index in indexes {
        let (name, unique, partial) = index?;
        if unique == 0 {
            continue;
        }
        if partial != 0 {
            return Err(schema_error(format!(
                "table {table} has unexpected partial UNIQUE index {name}"
            )));
        }
        actual_sets.push(index_columns(tx, &name)?);
    }
    actual_sets.sort();

    let mut expected_sets: Vec<Vec<String>> = expected
        .iter()
        .map(|columns| columns.iter().map(|column| (*column).to_owned()).collect())
        .collect();
    expected_sets.sort();

    if actual_sets != expected_sets {
        return Err(schema_error(format!(
            "table {table} UNIQUE/primary-key index set differs from structural manifest"
        )));
    }
    Ok(())
}

fn validate_named_indexes(tx: &Transaction<'_>, require_all: bool) -> Result<(), RuntimeError> {
    for (name, table, columns) in [
        (
            "integration_outbox_claim_idx",
            "integration_outbox",
            &["stage", "entry_id"][..],
        ),
        (
            "integration_outbox_lease_idx",
            "integration_outbox",
            &["stage", "lease_until_ms"][..],
        ),
        (
            "integration_execution_observation_idx",
            "integration_execution_observation",
            &["entry_id", "observation_id"][..],
        ),
        (
            "integration_reconciliation_history_idx",
            "integration_reconciliation_history",
            &["entry_id", "sequence"][..],
        ),
    ] {
        let stored: Option<String> = tx
            .query_row(
                "SELECT tbl_name FROM sqlite_master WHERE type = 'index' AND name = ?1",
                params![name],
                |row| row.get(0),
            )
            .optional()?;
        let Some(stored_table) = stored else {
            if require_all {
                return Err(schema_error(format!("required index {name} is missing")));
            }
            continue;
        };
        if stored_table != table {
            return Err(schema_error(format!(
                "index {name} is attached to {stored_table}; expected {table}"
            )));
        }

        let expected_columns: Vec<String> = columns.iter().map(|column| (*column).to_owned()).collect();
        if index_columns(tx, name)? != expected_columns {
            return Err(schema_error(format!(
                "index {name} column order differs from structural manifest"
            )));
        }

        let list_sql = format!("PRAGMA index_list({})", quote_identifier(table));
        let mut statement = tx.prepare(&list_sql)?;
        let rows = statement.query_map([], |row| {
            Ok((
                row.get::<_, String>(1)?,
                row.get::<_, i64>(2)?,
                row.get::<_, i64>(4)?,
            ))
        })?;
        let mut matched = false;
        for row in rows {
            let (candidate, unique, partial) = row?;
            if candidate == name {
                matched = true;
                if unique != 0 || partial != 0 {
                    return Err(schema_error(format!(
                        "index {name} uniqueness/partial contract differs from structural manifest"
                    )));
                }
            }
        }
        if !matched {
            return Err(schema_error(format!(
                "index {name} is absent from table {table} metadata"
            )));
        }
    }
    Ok(())
}

fn index_columns(tx: &Transaction<'_>, index: &str) -> Result<Vec<String>, RuntimeError> {
    let sql = format!("PRAGMA index_info({})", quote_identifier(index));
    let mut statement = tx.prepare(&sql)?;
    let rows = statement.query_map([], |row| row.get::<_, Option<String>>(2))?;
    let mut columns = Vec::new();
    for row in rows {
        let Some(column) = row? else {
            return Err(schema_error(format!(
                "index {index} contains an expression instead of an exact column"
            )));
        };
        columns.push(column);
    }
    Ok(columns)
}

const fn fk(
    referenced_table: &'static str,
    from_column: &'static str,
    to_column: &'static str,
) -> (&'static str, &'static str, &'static str) {
    (referenced_table, from_column, to_column)
}

fn require_foreign_keys(
    tx: &Transaction<'_>,
    table: &str,
    expected: &[(&str, &str, &str)],
) -> Result<(), RuntimeError> {
    let sql = format!("PRAGMA foreign_key_list({})", quote_identifier(table));
    let mut statement = tx.prepare(&sql)?;
    let rows = statement.query_map([], |row| {
        Ok(ForeignKeyShape {
            referenced_table: row.get(2)?,
            from_column: row.get(3)?,
            to_column: row.get(4)?,
            on_update: row.get(5)?,
            on_delete: row.get(6)?,
            match_kind: row.get(7)?,
        })
    })?;
    let mut actual = rows.collect::<Result<Vec<_>, _>>()?;
    actual.sort();

    let mut expected_shapes: Vec<ForeignKeyShape> = expected
        .iter()
        .map(|(referenced_table, from_column, to_column)| ForeignKeyShape {
            referenced_table: (*referenced_table).to_owned(),
            from_column: (*from_column).to_owned(),
            to_column: (*to_column).to_owned(),
            on_update: "NO ACTION".to_owned(),
            on_delete: "NO ACTION".to_owned(),
            match_kind: "NONE".to_owned(),
        })
        .collect();
    expected_shapes.sort();

    if actual != expected_shapes {
        return Err(schema_error(format!(
            "table {table} foreign-key set differs from structural manifest"
        )));
    }
    Ok(())
}

fn require_trigger_allowlist(tx: &Transaction<'_>, require_all: bool) -> Result<(), RuntimeError> {
    let managed: BTreeSet<String> = MANAGED_TABLES.iter().map(|table| (*table).to_owned()).collect();
    let expected: BTreeSet<(String, String)> = EXPECTED_TRIGGERS
        .iter()
        .map(|(name, table)| ((*name).to_owned(), (*table).to_owned()))
        .collect();

    let mut statement = tx.prepare(
        "SELECT name, tbl_name FROM sqlite_master WHERE type = 'trigger' ORDER BY name ASC",
    )?;
    let rows = statement.query_map([], |row| {
        Ok((row.get::<_, String>(0)?, row.get::<_, String>(1)?))
    })?;
    let mut actual = BTreeSet::new();
    for row in rows {
        let pair = row?;
        if !managed.contains(&pair.1) {
            continue;
        }
        if !expected.contains(&pair) {
            return Err(schema_error(format!(
                "unqualified trigger {} is attached to managed table {}",
                pair.0, pair.1
            )));
        }
        actual.insert(pair);
    }

    if require_all && actual != expected {
        return Err(schema_error(
            "required hardened trigger set is incomplete".to_owned(),
        ));
    }
    Ok(())
}

fn require_semantic_singleton(
    tx: &Transaction<'_>,
    expected_profile: &str,
) -> Result<(), RuntimeError> {
    let rows: i64 = tx.query_row(
        "SELECT COUNT(*) FROM integration_runtime_semantics",
        [],
        |row| row.get(0),
    )?;
    if rows != 1 {
        return Err(schema_error(format!(
            "semantic producer table has {rows} rows; expected one singleton"
        )));
    }
    let stored: Option<(i64, String)> = tx
        .query_row(
            "SELECT singleton, semantic_profile FROM integration_runtime_semantics LIMIT 1",
            [],
            |row| Ok((row.get(0)?, row.get(1)?)),
        )
        .optional()?;
    match stored {
        Some((1, profile)) if profile == expected_profile => Ok(()),
        Some((singleton, profile)) => Err(schema_error(format!(
            "semantic singleton/profile mismatch: singleton={singleton}, profile={profile}"
        ))),
        None => Err(schema_error("semantic producer singleton is missing".to_owned())),
    }
}

fn require_enforcement_singleton_shape_if_present(
    tx: &Transaction<'_>,
) -> Result<(), RuntimeError> {
    if !transaction_table_exists(tx, "integration_runtime_enforcement")? {
        return Ok(());
    }
    let bad: Option<i64> = tx
        .query_row(
            "SELECT singleton FROM integration_runtime_enforcement WHERE singleton != 1 LIMIT 1",
            [],
            |row| row.get(0),
        )
        .optional()?;
    if let Some(singleton) = bad {
        return Err(schema_error(format!(
            "enforcement metadata has unexpected singleton {singleton}"
        )));
    }
    let rows: i64 = tx.query_row(
        "SELECT COUNT(*) FROM integration_runtime_enforcement",
        [],
        |row| row.get(0),
    )?;
    if rows > 1 {
        return Err(schema_error(format!(
            "enforcement metadata has {rows} rows; expected at most one before repair"
        )));
    }
    Ok(())
}

fn require_exact_enforcement_singleton(
    tx: &Transaction<'_>,
    expected_profile: &str,
) -> Result<(), RuntimeError> {
    let rows: i64 = tx.query_row(
        "SELECT COUNT(*) FROM integration_runtime_enforcement",
        [],
        |row| row.get(0),
    )?;
    if rows != 1 {
        return Err(schema_error(format!(
            "enforcement metadata has {rows} rows; expected one singleton"
        )));
    }
    let stored: Option<(i64, String)> = tx
        .query_row(
            "SELECT singleton, enforcement_profile FROM integration_runtime_enforcement LIMIT 1",
            [],
            |row| Ok((row.get(0)?, row.get(1)?)),
        )
        .optional()?;
    match stored {
        Some((1, profile)) if profile == expected_profile => Ok(()),
        Some((singleton, profile)) => Err(schema_error(format!(
            "enforcement singleton/profile mismatch: singleton={singleton}, profile={profile}"
        ))),
        None => Err(schema_error("enforcement singleton is missing".to_owned())),
    }
}

fn require_no_foreign_key_violations(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    let violation: Option<String> = tx
        .query_row("PRAGMA foreign_key_check", [], |row| row.get(0))
        .optional()?;
    if let Some(table) = violation {
        return Err(schema_error(format!(
            "foreign-key integrity violation remains in table {table}"
        )));
    }
    Ok(())
}

fn require_schema_version(tx: &Transaction<'_>) -> Result<(), RuntimeError> {
    let stored: i64 = tx.query_row("PRAGMA user_version", [], |row| row.get(0))?;
    if stored != EXPECTED_STRUCTURAL_SCHEMA_V2 {
        return Err(schema_error(format!(
            "structural manifest expected schema v2, observed {stored}"
        )));
    }
    Ok(())
}

fn table_sql(tx: &Transaction<'_>, table: &str) -> Result<String, RuntimeError> {
    tx.query_row(
        "SELECT sql FROM sqlite_master WHERE type = 'table' AND name = ?1",
        params![table],
        |row| row.get(0),
    )
    .optional()?
    .ok_or_else(|| schema_error(format!("table {table} has no sqlite_master SQL")))
}

fn contains_strict_table_suffix(upper_sql: &str) -> bool {
    let compact = upper_sql.split_whitespace().collect::<Vec<_>>().join(" ");
    compact.ends_with(" STRICT") || compact.contains(") STRICT")
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

fn quote_identifier(identifier: &str) -> String {
    format!("\"{}\"", identifier.replace('"', "\"\""))
}

fn schema_error(detail: String) -> RuntimeError {
    RuntimeError::StoredIdentifier(format!(
        "{}: {detail}",
        RUNTIME_STRUCTURAL_MANIFEST_V1
    ))
}

fn open_aux(path: &Path) -> Result<Connection, RuntimeError> {
    let conn = Connection::open(path)?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    Ok(conn)
}

fn read_exact_witness(
    store_binding: &QualifiedRuntimeStoreBinding,
    entry_ids: &[i64],
) -> Result<BatchWitness, DurableClaimRereadError> {
    store_binding.revalidate()?;
    let mut conn = Connection::open_with_flags(
        store_binding.path(),
        OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON; PRAGMA query_only = ON;")?;
    require_main_database_path(&conn, store_binding.path())?;
    require_exact_runtime_pragmas(&conn)?;

    let tx = conn.transaction_with_behavior(TransactionBehavior::Deferred)?;
    require_exact_runtime_schema(&tx)?;
    let mut witness = BatchWitness::default();
    for entry_id in entry_ids.iter().copied() {
        witness
            .entries
            .insert(entry_id, capture_entry_snapshot(&tx, entry_id)?);
    }
    tx.commit()?;
    store_binding.revalidate()?;
    Ok(witness)
}

fn capture_entry_snapshot(
    conn: &Connection,
    entry_id: i64,
) -> Result<EntrySnapshot, ClaimClassificationError> {
    let outbox = load_outbox_row(conn, entry_id)?
        .ok_or(ClaimClassificationError::UnknownOutboxEntry { entry_id })?;
    let observations = load_observation_summary(conn, entry_id)?;
    Ok(EntrySnapshot {
        outbox,
        observations,
    })
}

fn load_outbox_row(
    conn: &Connection,
    entry_id: i64,
) -> Result<Option<OutboxRow>, ClaimClassificationError> {
    Ok(conn
        .query_row(
            "SELECT entry_id, command_id, connector_instance,\
                    command_commitment_algorithm, command_commitment_digest,\
                    authority_commitment_algorithm, authority_commitment_digest,\
                    side_effect_class, idempotency_key, command_bytes, stage,\
                    worker_id, lease_until_ms, attempt_count, current_attempt_id,\
                    dispatch_started_at_ms, outcome_json, reconciliation_json,\
                    created_at_ms, updated_at_ms\
             FROM integration_outbox WHERE entry_id = ?1",
            params![entry_id],
            |row| {
                Ok(OutboxRow {
                    entry_id: row.get(0)?,
                    command_id: row.get(1)?,
                    connector_instance: row.get(2)?,
                    command_commitment_algorithm: row.get(3)?,
                    command_commitment_digest: row.get(4)?,
                    authority_commitment_algorithm: row.get(5)?,
                    authority_commitment_digest: row.get(6)?,
                    side_effect_class: row.get(7)?,
                    idempotency_key: row.get(8)?,
                    command_bytes: row.get(9)?,
                    stage: row.get(10)?,
                    worker_id: row.get(11)?,
                    lease_until_ms: row.get(12)?,
                    attempt_count: row.get(13)?,
                    current_attempt_id: row.get(14)?,
                    dispatch_started_at_ms: row.get(15)?,
                    outcome_json: row.get(16)?,
                    reconciliation_json: row.get(17)?,
                    created_at_ms: row.get(18)?,
                    updated_at_ms: row.get(19)?,
                })
            },
        )
        .optional()?)
}

fn load_observation_summary(
    conn: &Connection,
    entry_id: i64,
) -> Result<ObservationSummary, ClaimClassificationError> {
    let (count, max_observation_id): (i64, Option<i64>) = conn.query_row(
        "SELECT COUNT(*), MAX(observation_id)\
         FROM integration_execution_observation WHERE entry_id = ?1",
        params![entry_id],
        |row| Ok((row.get(0)?, row.get(1)?)),
    )?;
    let max_row = match max_observation_id {
        Some(observation_id) => Some(conn.query_row(
            "SELECT observation_id, entry_id, attempt_id, outcome_json,\
                    observed_at_ms, applied_to_current\
             FROM integration_execution_observation\
             WHERE observation_id = ?1 AND entry_id = ?2",
            params![observation_id, entry_id],
            |row| {
                Ok(ObservationRow {
                    observation_id: row.get(0)?,
                    entry_id: row.get(1)?,
                    attempt_id: row.get(2)?,
                    outcome_json: row.get(3)?,
                    observed_at_ms: row.get(4)?,
                    applied_to_current: row.get(5)?,
                })
            },
        )?),
        None => None,
    };
    Ok(ObservationSummary {
        count,
        max_observation_id,
        max_row,
    })
}

fn open_exact_writer(
    store_binding: &QualifiedRuntimeStoreBinding,
) -> Result<Connection, ClaimClassificationError> {
    let conn = Connection::open_with_flags(
        store_binding.path(),
        OpenFlags::SQLITE_OPEN_READ_WRITE | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(Duration::from_secs(5))?;
    conn.execute_batch("PRAGMA foreign_keys = ON;")?;
    require_main_database_path(&conn, store_binding.path())?;
    require_exact_runtime_pragmas(&conn)?;
    store_binding.revalidate()?;
    Ok(conn)
}

fn require_exact_runtime_pragmas(
    conn: &Connection,
) -> Result<(), ClaimClassificationError> {
    let version: i64 = conn.pragma_query_value(None, "user_version", |row| row.get(0))?;
    if version != RUNTIME_SCHEMA_VERSION {
        return Err(ClaimClassificationError::UnsupportedRuntimeSchema {
            actual: version,
            expected: RUNTIME_SCHEMA_VERSION,
        });
    }
    let journal_mode: String =
        conn.pragma_query_value(None, "journal_mode", |row| row.get(0))?;
    if !journal_mode.eq_ignore_ascii_case("wal") {
        return Err(ClaimClassificationError::UnexpectedJournalMode(journal_mode));
    }
    let synchronous: i64 =
        conn.pragma_query_value(None, "synchronous", |row| row.get(0))?;
    if synchronous != 2 {
        return Err(ClaimClassificationError::UnexpectedSynchronousMode(
            synchronous,
        ));
    }
    Ok(())
}

fn require_exact_runtime_schema(
    conn: &Connection,
) -> Result<(), ClaimClassificationError> {
    if !table_exists(conn, OUTBOX_TABLE)? || !table_exists(conn, OBSERVATION_TABLE)? {
        return Err(ClaimClassificationError::RuntimeSchemaMismatch);
    }
    const OUTBOX_COLUMNS: &[(&str, &str, i64, i64)] = &[
        ("entry_id", "INTEGER", 0, 1),
        ("command_id", "TEXT", 1, 0),
        ("connector_instance", "TEXT", 1, 0),
        ("command_commitment_algorithm", "INTEGER", 1, 0),
        ("command_commitment_digest", "BLOB", 1, 0),
        ("authority_commitment_algorithm", "INTEGER", 1, 0),
        ("authority_commitment_digest", "BLOB", 1, 0),
        ("side_effect_class", "INTEGER", 1, 0),
        ("idempotency_key", "TEXT", 0, 0),
        ("command_bytes", "BLOB", 1, 0),
        ("stage", "INTEGER", 1, 0),
        ("worker_id", "TEXT", 0, 0),
        ("lease_until_ms", "INTEGER", 0, 0),
        ("attempt_count", "INTEGER", 1, 0),
        ("current_attempt_id", "TEXT", 0, 0),
        ("dispatch_started_at_ms", "INTEGER", 0, 0),
        ("outcome_json", "BLOB", 0, 0),
        ("reconciliation_json", "BLOB", 0, 0),
        ("created_at_ms", "INTEGER", 1, 0),
        ("updated_at_ms", "INTEGER", 1, 0),
    ];
    const OBSERVATION_COLUMNS: &[(&str, &str, i64, i64)] = &[
        ("observation_id", "INTEGER", 0, 1),
        ("entry_id", "INTEGER", 1, 0),
        ("attempt_id", "TEXT", 1, 0),
        ("outcome_json", "BLOB", 1, 0),
        ("observed_at_ms", "INTEGER", 1, 0),
        ("applied_to_current", "INTEGER", 1, 0),
    ];
    require_table_columns(conn, OUTBOX_TABLE, OUTBOX_COLUMNS)?;
    require_table_columns(conn, OBSERVATION_TABLE, OBSERVATION_COLUMNS)?;

    let foreign_keys = conn
        .prepare("PRAGMA foreign_key_list(integration_execution_observation)")?
        .query_map([], |row| {
            Ok((
                row.get::<_, String>(2)?,
                row.get::<_, String>(3)?,
                row.get::<_, String>(4)?,
                row.get::<_, String>(5)?,
                row.get::<_, String>(6)?,
            ))
        })?
        .collect::<Result<Vec<_>, _>>()?;
    if foreign_keys
        != vec![(
            "integration_outbox".to_owned(),
            "entry_id".to_owned(),
            "entry_id".to_owned(),
            "NO ACTION".to_owned(),
            "NO ACTION".to_owned(),
        )]
    {
        return Err(ClaimClassificationError::RuntimeSchemaMismatch);
    }

    for table in [OUTBOX_TABLE, OBSERVATION_TABLE] {
        let trigger_count: i64 = conn.query_row(
            "SELECT COUNT(*) FROM sqlite_master WHERE type = 'trigger' AND tbl_name = ?1",
            params![table],
            |row| row.get(0),
        )?;
        if trigger_count != 0 {
            return Err(ClaimClassificationError::RuntimeSchemaMismatch);
        }
    }
    Ok(())
}

fn require_table_columns(
    conn: &Connection,
    table: &str,
    expected: &[(&str, &str, i64, i64)],
) -> Result<(), ClaimClassificationError> {
    let sql = format!("PRAGMA table_info({table})");
    let actual = conn
        .prepare(&sql)?
        .query_map([], |row| {
            Ok((
                row.get::<_, String>(1)?,
                row.get::<_, String>(2)?,
                row.get::<_, i64>(3)?,
                row.get::<_, i64>(5)?,
            ))
        })?
        .collect::<Result<Vec<_>, _>>()?;
    if actual.len() != expected.len()
        || actual.iter().zip(expected).any(|(actual, expected)| {
            actual.0 != expected.0
                || actual.1.to_ascii_uppercase() != expected.1
                || actual.2 != expected.2
                || actual.3 != expected.3
        })
    {
        return Err(ClaimClassificationError::RuntimeSchemaMismatch);
    }
    Ok(())
}

fn table_exists(
    conn: &Connection,
    name: &str,
) -> Result<bool, ClaimClassificationError> {
    Ok(conn
        .query_row(
            "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?1",
            params![name],
            |row| row.get::<_, i64>(0),
        )
        .optional()?
        .is_some())
}

fn require_main_database_path(
    conn: &Connection,
    expected_path: &Path,
) -> Result<(), ClaimClassificationError> {
    let opened: String = conn.query_row(
        "SELECT file FROM pragma_database_list WHERE name = 'main'",
        [],
        |row| row.get(0),
    )?;
    if opened.is_empty() {
        return Err(ClaimClassificationError::StoreIdentityChanged);
    }
    let opened = fs::canonicalize(opened).map_err(ClaimClassificationError::Io)?;
    if opened != expected_path {
        return Err(ClaimClassificationError::StoreIdentityChanged);
    }
    Ok(())
}

fn validate_request(
    worker_id: &str,
    now_ms: i64,
    lease_duration_ms: i64,
    limit: usize,
) -> Result<(), ClaimClassificationError> {
    if worker_id.is_empty() {
        return Err(ClaimClassificationError::EmptyWorkerId);
    }
    if worker_id.len() > MAX_WORKER_ID_BYTES {
        return Err(ClaimClassificationError::WorkerIdTooLong);
    }
    if worker_id.chars().any(char::is_control) {
        return Err(ClaimClassificationError::WorkerIdContainsControl);
    }
    if now_ms < 0 {
        return Err(ClaimClassificationError::InvalidTimestamp);
    }
    if lease_duration_ms <= 0 {
        return Err(ClaimClassificationError::InvalidLeaseDuration);
    }
    if !(1..=MAX_OUTBOX_CLAIM).contains(&limit) {
        return Err(ClaimClassificationError::InvalidClaimLimit);
    }
    Ok(())
}

fn commitment_from_parts(
    algorithm: i64,
    digest: Vec<u8>,
) -> Result<ContentCommitment, ClaimClassificationError> {
    if digest.len() != 32 {
        return Err(ClaimClassificationError::InvalidStoredDigestLength {
            actual: digest.len(),
        });
    }
    let mut fixed = [0_u8; 32];
    fixed.copy_from_slice(&digest);
    let algorithm = match algorithm {
        1 => DigestAlgorithm::Sha256,
        value => {
            return Err(ClaimClassificationError::InvalidStoredValue {
                field: "digest_algorithm",
                value,
            });
        }
    };
    Ok(ContentCommitment {
        algorithm,
        digest: fixed,
    })
}

fn side_effect_from_i64(value: i64) -> Result<SideEffectClass, ClaimClassificationError> {
    match value {
        0 => Ok(SideEffectClass::ReadOnly),
        1 => Ok(SideEffectClass::Reversible),
        2 => Ok(SideEffectClass::Compensatable),
        3 => Ok(SideEffectClass::Irreversible),
        _ => Err(ClaimClassificationError::InvalidStoredValue {
            field: "side_effect_class",
            value,
        }),
    }
}

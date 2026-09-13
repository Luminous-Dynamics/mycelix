pub fn claim_outbox_classified(
    store_binding: &QualifiedRuntimeStoreBinding,
    worker_id: &str,
    now_ms: i64,
    lease_duration_ms: i64,
    limit: usize,
) -> Result<ClassifiedClaimOutcome, ClaimClassificationError> {
    claim_outbox_classified_with_mode(
        store_binding,
        worker_id,
        now_ms,
        lease_duration_ms,
        limit,
        CommitMode::Real,
    )
}

#[derive(Clone, Copy)]
enum CommitMode {
    Real,
    #[cfg(test)]
    RollbackThenError,
    #[cfg(test)]
    RollbackThenBusy,
    #[cfg(test)]
    CommitThenError,
}

fn claim_outbox_classified_with_mode(
    store_binding: &QualifiedRuntimeStoreBinding,
    worker_id: &str,
    now_ms: i64,
    lease_duration_ms: i64,
    limit: usize,
    commit_mode: CommitMode,
) -> Result<ClassifiedClaimOutcome, ClaimClassificationError> {
    validate_request(worker_id, now_ms, lease_duration_ms, limit)?;
    let lease_until_ms = now_ms
        .checked_add(lease_duration_ms)
        .ok_or(ClaimClassificationError::LeaseDeadlineOverflow)?;

    store_binding.revalidate()?;
    let mut conn = open_exact_writer(store_binding)?;
    let tx = conn.transaction_with_behavior(TransactionBehavior::Immediate)?;
    require_exact_runtime_schema(&tx)?;

    let product = execute_claim_transaction(
        &tx,
        worker_id,
        now_ms,
        lease_until_ms,
        limit,
    )?;

    let commit_result = finish_commit(tx, commit_mode);
    drop(conn);

    match commit_result {
        Ok(()) => match store_binding.revalidate() {
            Ok(()) => Ok(ClassifiedClaimOutcome::Committed(CommittedClaimBatch {
                claims: product.claims,
                recovery: product.recovery,
            })),
            Err(error) => Ok(ClassifiedClaimOutcome::RecoveredCommitted(
                RecoveredCommittedClaimBatch {
                    attempts: recovered_attempts(&product.candidate),
                    recovery: product.recovery,
                    commit_error: ClaimCommitFailure::PostCommitStoreBinding(error),
                },
            )),
        },
        Err(commit_error) => Ok(classify_returned_commit_error(
            store_binding,
            product,
            commit_error,
        )),
    }
}

fn execute_claim_transaction(
    tx: &Transaction<'_>,
    worker_id: &str,
    now_ms: i64,
    lease_until_ms: i64,
    limit: usize,
) -> Result<TransactionProduct, ClaimClassificationError> {
    let stale_ids = load_stale_entry_ids(tx, now_ms)?;
    let mut predecessor = BatchWitness::default();
    for entry_id in stale_ids.iter().copied() {
        predecessor
            .entries
            .insert(entry_id, capture_entry_snapshot(tx, entry_id)?);
    }

    let recovery = recover_stale_entries(tx, &stale_ids, now_ms)?;

    let claim_ids = load_claimable_ids(tx, limit)?;
    if claim_ids.is_empty() {
        if let Some(entry_id) = load_first_exhausted_entry(tx)? {
            return Err(ClaimClassificationError::AttemptBudgetExceeded {
                entry_id,
                limit: MAX_EXECUTION_ATTEMPTS_PER_ENTRY as u32,
            });
        }
    }

    for entry_id in claim_ids.iter().copied() {
        if !predecessor.entries.contains_key(&entry_id) {
            predecessor
                .entries
                .insert(entry_id, capture_entry_snapshot(tx, entry_id)?);
        }
    }

    let mut claims = Vec::with_capacity(claim_ids.len());
    for entry_id in claim_ids {
        claims.push(claim_entry(
            tx,
            entry_id,
            worker_id,
            now_ms,
            lease_until_ms,
        )?);
    }

    let mut candidate = BatchWitness::default();
    for entry_id in predecessor.entries.keys().copied() {
        candidate
            .entries
            .insert(entry_id, capture_entry_snapshot(tx, entry_id)?);
    }

    Ok(TransactionProduct {
        claims,
        recovery,
        predecessor,
        candidate,
    })
}

fn load_stale_entry_ids(
    conn: &Connection,
    now_ms: i64,
) -> Result<Vec<i64>, ClaimClassificationError> {
    let mut statement = conn.prepare(
        "SELECT entry_id FROM integration_outbox\
         WHERE stage IN (?1, ?2) AND lease_until_ms IS NOT NULL AND lease_until_ms <= ?3\
         ORDER BY entry_id ASC LIMIT ?4",
    )?;
    let rows = statement.query_map(
        params![
            ATTEMPT_PREPARED,
            DISPATCH_STARTED,
            now_ms,
            (MAX_RECOVERY_ROWS_PER_CLAIM + 1) as i64,
        ],
        |row| row.get::<_, i64>(0),
    )?;
    let ids = rows.collect::<Result<Vec<_>, _>>()?;
    if ids.len() > MAX_RECOVERY_ROWS_PER_CLAIM {
        return Err(ClaimClassificationError::RecoveryBatchTooLarge {
            limit: MAX_RECOVERY_ROWS_PER_CLAIM,
        });
    }
    Ok(ids)
}

fn recover_stale_entries(
    tx: &Transaction<'_>,
    stale_ids: &[i64],
    now_ms: i64,
) -> Result<RecoverySummary, ClaimClassificationError> {
    let mut summary = RecoverySummary::default();
    for entry_id in stale_ids.iter().copied() {
        let row = load_outbox_row(tx, entry_id)?
            .ok_or(ClaimClassificationError::UnknownOutboxEntry { entry_id })?;
        match row.stage {
            ATTEMPT_PREPARED => {
                let changed = tx.execute(
                    "UPDATE integration_outbox\
                     SET stage = ?1, worker_id = NULL, lease_until_ms = NULL,\
                         current_attempt_id = NULL, dispatch_started_at_ms = NULL,\
                         updated_at_ms = ?2\
                     WHERE entry_id = ?3 AND stage = ?4",
                    params![OUTBOX_COMMITTED, now_ms, entry_id, ATTEMPT_PREPARED],
                )?;
                if changed != 1 {
                    return Err(ClaimClassificationError::ConcurrentStateChange { entry_id });
                }
                summary.pre_dispatch_requeued += 1;
            }
            DISPATCH_STARTED => {
                let observations = load_observation_summary(tx, entry_id)?;
                if observations.count >= MAX_EXECUTION_OBSERVATIONS_PER_ENTRY {
                    return Err(ClaimClassificationError::ExecutionObservationHistoryLimit {
                        entry_id,
                    });
                }
                let command_id = IntegrationCommandId::new(row.command_id.clone())
                    .map_err(|error| ClaimClassificationError::StoredIdentifier(error.to_string()))?;
                let connector_instance = ConnectorInstanceId::new(row.connector_instance.clone())
                    .map_err(|error| ClaimClassificationError::StoredIdentifier(error.to_string()))?;
                let outcome = ExternalExecutionOutcome::Ambiguous {
                    operation: ExternalOperationRef {
                        command_id,
                        connector_instance,
                        provider_operation: None,
                    },
                    reconciliation_hint: ReconciliationHint {
                        strategy: ReconciliationStrategy::ManualReview,
                        object: None,
                        idempotency_key: None,
                        earliest_retry_at_ms: None,
                    },
                };
                let json = serde_json::to_vec(&outcome)?;
                if json.len() > MAX_SERIALIZED_OUTCOME_BYTES {
                    return Err(ClaimClassificationError::OutcomeTooLarge);
                }
                let changed = tx.execute(
                    "UPDATE integration_outbox\
                     SET stage = ?1, worker_id = NULL, lease_until_ms = NULL,\
                         outcome_json = ?2, updated_at_ms = ?3\
                     WHERE entry_id = ?4 AND stage = ?5",
                    params![AMBIGUOUS, json.as_slice(), now_ms, entry_id, DISPATCH_STARTED],
                )?;
                if changed != 1 {
                    return Err(ClaimClassificationError::ConcurrentStateChange { entry_id });
                }
                if let Some(attempt_id) = row.current_attempt_id {
                    tx.execute(
                        "INSERT INTO integration_execution_observation (\
                            entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\
                         ) VALUES (?1, ?2, ?3, ?4, 1)",
                        params![entry_id, attempt_id, json.as_slice(), now_ms],
                    )?;
                }
                summary.post_dispatch_marked_ambiguous += 1;
            }
            actual => {
                return Err(ClaimClassificationError::UnexpectedStage {
                    entry_id,
                    expected: DISPATCH_STARTED,
                    actual,
                });
            }
        }
    }
    Ok(summary)
}

fn load_claimable_ids(
    conn: &Connection,
    limit: usize,
) -> Result<Vec<i64>, ClaimClassificationError> {
    let mut statement = conn.prepare(
        "SELECT entry_id FROM integration_outbox\
         WHERE stage = ?1 AND attempt_count < ?2\
         ORDER BY entry_id ASC LIMIT ?3",
    )?;
    let rows = statement.query_map(
        params![OUTBOX_COMMITTED, MAX_EXECUTION_ATTEMPTS_PER_ENTRY, limit as i64],
        |row| row.get::<_, i64>(0),
    )?;
    Ok(rows.collect::<Result<Vec<_>, _>>()?)
}

fn load_first_exhausted_entry(
    conn: &Connection,
) -> Result<Option<i64>, ClaimClassificationError> {
    Ok(conn
        .query_row(
            "SELECT entry_id FROM integration_outbox\
             WHERE stage = ?1 AND attempt_count >= ?2\
             ORDER BY entry_id ASC LIMIT 1",
            params![OUTBOX_COMMITTED, MAX_EXECUTION_ATTEMPTS_PER_ENTRY],
            |row| row.get(0),
        )
        .optional()?)
}

fn claim_entry(
    tx: &Transaction<'_>,
    entry_id: i64,
    worker_id: &str,
    now_ms: i64,
    lease_until_ms: i64,
) -> Result<ExecutionClaim, ClaimClassificationError> {
    let before = load_outbox_row(tx, entry_id)?
        .ok_or(ClaimClassificationError::UnknownOutboxEntry { entry_id })?;
    if before.stage != OUTBOX_COMMITTED {
        return Err(ClaimClassificationError::UnexpectedStage {
            entry_id,
            expected: OUTBOX_COMMITTED,
            actual: before.stage,
        });
    }
    if before.attempt_count >= MAX_EXECUTION_ATTEMPTS_PER_ENTRY {
        return Err(ClaimClassificationError::AttemptBudgetExceeded {
            entry_id,
            limit: MAX_EXECUTION_ATTEMPTS_PER_ENTRY as u32,
        });
    }
    let next_attempt_count = before
        .attempt_count
        .checked_add(1)
        .ok_or(ClaimClassificationError::AttemptBudgetExceeded {
            entry_id,
            limit: MAX_EXECUTION_ATTEMPTS_PER_ENTRY as u32,
        })?;
    let attempt_id = ExecutionAttemptId::new(format!("{entry_id}:{next_attempt_count}"))
        .map_err(|error| ClaimClassificationError::StoredIdentifier(error.to_string()))?;

    let changed = tx.execute(
        "UPDATE integration_outbox\
         SET stage = ?1, worker_id = ?2, lease_until_ms = ?3,\
             attempt_count = ?4, current_attempt_id = ?5,\
             dispatch_started_at_ms = NULL, updated_at_ms = ?6\
         WHERE entry_id = ?7 AND stage = ?8",
        params![
            ATTEMPT_PREPARED,
            worker_id,
            lease_until_ms,
            next_attempt_count,
            attempt_id.as_str(),
            now_ms,
            entry_id,
            OUTBOX_COMMITTED,
        ],
    )?;
    if changed != 1 {
        return Err(ClaimClassificationError::ConcurrentStateChange { entry_id });
    }

    let after = load_outbox_row(tx, entry_id)?
        .ok_or(ClaimClassificationError::UnknownOutboxEntry { entry_id })?;
    Ok(execution_claim_from_row(after, attempt_id, lease_until_ms)?)
}

fn execution_claim_from_row(
    row: OutboxRow,
    attempt_id: ExecutionAttemptId,
    lease_until_ms: i64,
) -> Result<ExecutionClaim, ClaimClassificationError> {
    let command_id = IntegrationCommandId::new(row.command_id)
        .map_err(|error| ClaimClassificationError::StoredIdentifier(error.to_string()))?;
    let connector_instance = ConnectorInstanceId::new(row.connector_instance)
        .map_err(|error| ClaimClassificationError::StoredIdentifier(error.to_string()))?;
    let command_commitment = commitment_from_parts(
        row.command_commitment_algorithm,
        row.command_commitment_digest,
    )?;
    let side_effect_class = side_effect_from_i64(row.side_effect_class)?;
    let idempotency_key = row
        .idempotency_key
        .map(IdempotencyKey::new)
        .transpose()
        .map_err(|error| ClaimClassificationError::StoredIdentifier(error.to_string()))?;
    let attempt_count = u32::try_from(row.attempt_count).map_err(|_| {
        ClaimClassificationError::InvalidStoredValue {
            field: "attempt_count",
            value: row.attempt_count,
        }
    })?;

    Ok(ExecutionClaim {
        entry_id: row.entry_id,
        attempt_id,
        command_id,
        connector_instance,
        command_commitment,
        side_effect_class,
        idempotency_key,
        attempt_count,
        lease_until_ms,
    })
}

fn finish_commit(
    tx: Transaction<'_>,
    mode: CommitMode,
) -> Result<(), ClaimCommitFailure> {
    match mode {
        CommitMode::Real => tx.commit().map_err(ClaimCommitFailure::Sqlite),
        #[cfg(test)]
        CommitMode::RollbackThenError => {
            tx.rollback().map_err(ClaimCommitFailure::Sqlite)?;
            Err(ClaimCommitFailure::Injected("rollback-before-return"))
        }
        #[cfg(test)]
        CommitMode::RollbackThenBusy => {
            tx.rollback().map_err(ClaimCommitFailure::Sqlite)?;
            let busy = rusqlite::Error::SqliteFailure(
                rusqlite::ffi::Error::new(rusqlite::ffi::SQLITE_BUSY),
                None,
            );
            Err(ClaimCommitFailure::Sqlite(busy))
        }
        #[cfg(test)]
        CommitMode::CommitThenError => {
            tx.commit().map_err(ClaimCommitFailure::Sqlite)?;
            Err(ClaimCommitFailure::Injected("commit-before-return"))
        }
    }
}

fn classify_returned_commit_error(
    store_binding: &QualifiedRuntimeStoreBinding,
    product: TransactionProduct,
    commit_error: ClaimCommitFailure,
) -> ClassifiedClaimOutcome {
    let touched = product.predecessor.touched_entry_ids();
    match read_exact_witness(store_binding, &touched) {
        Ok(observed) => {
            let candidate_exact = observed == product.candidate;
            let predecessor_exact = observed == product.predecessor;
            match classify_exact_durable_transition(candidate_exact, predecessor_exact) {
                CommitClassification::Committed => {
                    ClassifiedClaimOutcome::RecoveredCommitted(RecoveredCommittedClaimBatch {
                        attempts: recovered_attempts(&product.candidate),
                        recovery: product.recovery,
                        commit_error,
                    })
                }
                CommitClassification::DefinitelyNotCommitted => {
                    ClassifiedClaimOutcome::DefinitelyNotCommitted(
                        DefinitelyNotCommittedClaimBatch {
                            touched_entry_ids: touched,
                            commit_error,
                        },
                    )
                }
                CommitClassification::IndeterminateCommit => {
                    ClassifiedClaimOutcome::IndeterminateCommit(IndeterminateClaimBatch {
                        touched_entry_ids: touched,
                        commit_error,
                        reread_error: None,
                    })
                }
            }
        }
        Err(reread_error) => ClassifiedClaimOutcome::IndeterminateCommit(IndeterminateClaimBatch {
            touched_entry_ids: touched,
            commit_error,
            reread_error: Some(reread_error),
        }),
    }
}

fn recovered_attempts(candidate: &BatchWitness) -> Vec<RecoveredAttemptIdentity> {
    candidate
        .entries
        .values()
        .filter(|snapshot| snapshot.outbox.stage == ATTEMPT_PREPARED)
        .filter_map(|snapshot| {
            Some(RecoveredAttemptIdentity {
                entry_id: snapshot.outbox.entry_id,
                attempt_id: snapshot.outbox.current_attempt_id.clone()?,
                lease_until_ms: snapshot.outbox.lease_until_ms?,
            })
        })
        .collect()
}

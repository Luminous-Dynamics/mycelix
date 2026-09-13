#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_integration_core::IntegrationCommandId;
    use mycelix_integration_runtime::{
        DurableOutboundIntent, EnqueueDisposition, SqliteIntegrationStore,
    };
    use mycelix_integration_runtime_store_binding::{
        RuntimeStoreProvisioningExpectation, qualify_runtime_store_binding,
    };
    use std::os::unix::fs::MetadataExt;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn unique_db_path(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock after epoch")
            .as_nanos();
        std::env::temp_dir().join(format!(
            "mycelix-runtime-claim-{label}-{}-{nonce}.sqlite",
            std::process::id()
        ))
    }

    fn cleanup(path: &Path) {
        let _ = fs::remove_file(path);
        for suffix in ["-wal", "-shm"] {
            let mut value = path.as_os_str().to_os_string();
            value.push(suffix);
            let _ = fs::remove_file(PathBuf::from(value));
        }
    }

    fn intent(command: &str) -> DurableOutboundIntent {
        DurableOutboundIntent {
            command_id: IntegrationCommandId::new(command).unwrap(),
            connector_instance: ConnectorInstanceId::new("connector-1").unwrap(),
            command_commitment: ContentCommitment::sha256(format!("command:{command}").as_bytes()),
            authority_commitment: ContentCommitment::sha256(b"authority"),
            side_effect_class: SideEffectClass::Compensatable,
            idempotency_key: Some(IdempotencyKey::new(format!("idem:{command}")).unwrap()),
            command_bytes: format!("sealed:{command}").into_bytes(),
            created_at_ms: 100,
        }
    }

    fn seeded_store(path: &Path, commands: &[&str]) {
        cleanup(path);
        let mut store = SqliteIntegrationStore::open(path).unwrap();
        for command in commands {
            assert!(matches!(
                store.enqueue_outbound(&intent(command)).unwrap(),
                EnqueueDisposition::Inserted(_)
            ));
        }
    }

    fn binding(path: &Path) -> QualifiedRuntimeStoreBinding {
        let metadata = fs::metadata(path).unwrap();
        qualify_runtime_store_binding(
            path,
            RuntimeStoreProvisioningExpectation {
                device: metadata.dev(),
                inode: metadata.ino(),
                uid: metadata.uid(),
                gid: metadata.gid(),
                mode: metadata.mode() & 0o7777,
            },
            "test/runtime-claim-store/v1",
        )
        .unwrap()
    }

    #[test]
    fn successful_commit_returns_live_claim_metadata() {
        let path = unique_db_path("success");
        seeded_store(&path, &["command-1", "command-2"]);
        let binding = binding(&path);
        let outcome = claim_outbox_classified(&binding, "worker-a", 200, 100, 2).unwrap();
        match outcome {
            ClassifiedClaimOutcome::Committed(batch) => {
                assert_eq!(batch.claims().len(), 2);
                assert!(batch.live_claim_metadata_returned_here());
                assert!(!batch.grants_execution_authority());
            }
            _ => panic!("expected committed batch"),
        }
        cleanup(&path);
    }

    #[test]
    fn injected_precommit_failure_reobserves_exact_predecessor() {
        let path = unique_db_path("predecessor");
        seeded_store(&path, &["command-1"]);
        let binding = binding(&path);
        let outcome = claim_outbox_classified_with_mode(
            &binding,
            "worker-a",
            200,
            100,
            1,
            CommitMode::RollbackThenError,
        )
        .unwrap();
        match outcome {
            ClassifiedClaimOutcome::DefinitelyNotCommitted(batch) => {
                assert!(batch.exact_predecessor_reobserved_here());
                assert!(batch.persistence_retry_eligible_here());
                assert!(!batch.ordinary_retry_authorized_here());
                assert!(!batch.live_claims_reconstructed_here());
            }
            _ => panic!("expected definitely-not-committed"),
        }
        cleanup(&path);
    }

    #[test]
    fn injected_sqlite_busy_is_classified_from_durable_state_not_error_code() {
        let path = unique_db_path("busy");
        seeded_store(&path, &["command-1"]);
        let binding = binding(&path);
        let outcome = claim_outbox_classified_with_mode(
            &binding,
            "worker-a",
            200,
            100,
            1,
            CommitMode::RollbackThenBusy,
        )
        .unwrap();
        match outcome {
            ClassifiedClaimOutcome::DefinitelyNotCommitted(batch) => {
                assert!(batch.exact_predecessor_reobserved_here());
                assert!(!batch.ordinary_retry_authorized_here());
                assert!(matches!(
                    batch.commit_error(),
                    ClaimCommitFailure::Sqlite(rusqlite::Error::SqliteFailure(error, _))
                        if error.code == rusqlite::ErrorCode::DatabaseBusy
                ));
            }
            _ => panic!("expected definitely-not-committed busy result"),
        }
        cleanup(&path);
    }

    #[test]
    fn injected_postcommit_failure_recovers_without_live_claims() {
        let path = unique_db_path("successor");
        seeded_store(&path, &["command-1"]);
        let binding = binding(&path);
        let outcome = claim_outbox_classified_with_mode(
            &binding,
            "worker-a",
            200,
            100,
            1,
            CommitMode::CommitThenError,
        )
        .unwrap();
        match outcome {
            ClassifiedClaimOutcome::RecoveredCommitted(batch) => {
                assert!(batch.exact_candidate_successor_reobserved_here());
                assert!(!batch.live_claims_reconstructed_here());
                assert!(!batch.ordinary_retry_authorized_here());
                assert_eq!(batch.attempts().len(), 1);
                assert_eq!(batch.attempts()[0].attempt_id(), "1:1");
            }
            _ => panic!("expected recovered committed"),
        }
        cleanup(&path);
    }

    #[test]
    fn stale_prepared_is_requeued_and_reclaimed_in_same_transaction() {
        let path = unique_db_path("reclaim");
        seeded_store(&path, &["command-1"]);
        {
            let mut store = SqliteIntegrationStore::open(&path).unwrap();
            let first = store.claim_outbox("worker-old", 200, 10, 1).unwrap();
            assert_eq!(first[0].attempt_id.as_str(), "1:1");
        }
        let binding = binding(&path);
        let outcome = claim_outbox_classified(&binding, "worker-new", 211, 100, 1).unwrap();
        match outcome {
            ClassifiedClaimOutcome::Committed(batch) => {
                assert_eq!(batch.recovery_summary().pre_dispatch_requeued, 1);
                assert_eq!(batch.claims()[0].attempt_id.as_str(), "1:2");
                assert_eq!(batch.claims()[0].attempt_count, 2);
            }
            _ => panic!("expected committed batch"),
        }
        cleanup(&path);
    }

    #[test]
    fn stale_dispatch_becomes_ambiguous_and_is_not_reclaimed() {
        let path = unique_db_path("ambiguous");
        seeded_store(&path, &["command-1"]);
        {
            let mut store = SqliteIntegrationStore::open(&path).unwrap();
            let first = store.claim_outbox("worker-old", 200, 10, 1).unwrap();
            store
                .mark_dispatch_started(
                    first[0].entry_id,
                    &first[0].attempt_id,
                    "worker-old",
                    205,
                )
                .unwrap();
        }
        let binding = binding(&path);
        let outcome = claim_outbox_classified(&binding, "worker-new", 211, 100, 1).unwrap();
        match outcome {
            ClassifiedClaimOutcome::Committed(batch) => {
                assert_eq!(batch.recovery_summary().post_dispatch_marked_ambiguous, 1);
                assert!(batch.claims().is_empty());
            }
            _ => panic!("expected committed recovery"),
        }

        let conn = Connection::open(&path).unwrap();
        let (stage, observation_count): (i64, i64) = (
            conn.query_row(
                "SELECT stage FROM integration_outbox WHERE entry_id = 1",
                [],
                |row| row.get(0),
            )
            .unwrap(),
            conn.query_row(
                "SELECT COUNT(*) FROM integration_execution_observation WHERE entry_id = 1",
                [],
                |row| row.get(0),
            )
            .unwrap(),
        );
        assert_eq!(stage, AMBIGUOUS);
        assert_eq!(observation_count, 1);
        drop(conn);
        cleanup(&path);
    }

    #[test]
    fn unexpected_durable_state_is_indeterminate() {
        let path = unique_db_path("indeterminate");
        seeded_store(&path, &["command-1"]);
        let binding = binding(&path);

        store_binding_round_trip(&binding);
        let mut conn = open_exact_writer(&binding).unwrap();
        let tx = conn
            .transaction_with_behavior(TransactionBehavior::Immediate)
            .unwrap();
        require_exact_runtime_schema(&tx).unwrap();
        let mut predecessor = BatchWitness::default();
        predecessor
            .entries
            .insert(1, capture_entry_snapshot(&tx, 1).unwrap());
        tx.execute(
            "UPDATE integration_outbox SET updated_at_ms = updated_at_ms + 1 WHERE entry_id = 1",
            [],
        )
        .unwrap();
        let mut candidate = BatchWitness::default();
        candidate
            .entries
            .insert(1, capture_entry_snapshot(&tx, 1).unwrap());
        tx.rollback().unwrap();
        drop(conn);

        let conn = Connection::open(&path).unwrap();
        conn.execute(
            "UPDATE integration_outbox SET updated_at_ms = updated_at_ms + 2 WHERE entry_id = 1",
            [],
        )
        .unwrap();
        drop(conn);

        let product = TransactionProduct {
            claims: Vec::new(),
            recovery: RecoverySummary::default(),
            predecessor,
            candidate,
        };
        let outcome = classify_returned_commit_error(
            &binding,
            product,
            ClaimCommitFailure::Injected("synthetic-indeterminate"),
        );
        assert!(matches!(
            outcome,
            ClassifiedClaimOutcome::IndeterminateCommit(_)
        ));
        cleanup(&path);
    }

    fn store_binding_round_trip(binding: &QualifiedRuntimeStoreBinding) {
        binding.revalidate().unwrap();
    }
}

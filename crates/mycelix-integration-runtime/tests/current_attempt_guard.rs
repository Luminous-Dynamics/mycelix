use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, IdempotencyKey, IntegrationCommandId, SideEffectClass,
};
use mycelix_integration_runtime::{
    CurrentAttemptQualificationError, DurableOutboundIntent, EnqueueDisposition,
    SqliteIntegrationStore,
};
use tempfile::tempdir;

fn intent(created_at_ms: i64) -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: IntegrationCommandId::new("cmd-current-attempt-1").unwrap(),
        connector_instance: ConnectorInstanceId::new("provider-prod-1").unwrap(),
        command_commitment: ContentCommitment::sha256(b"command-current-attempt-1"),
        authority_commitment: ContentCommitment::sha256(b"authority-current-attempt-1"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(IdempotencyKey::new("idem-current-attempt-1").unwrap()),
        command_bytes: b"opaque-command-bytes".to_vec(),
        created_at_ms,
    }
}

#[test]
fn exact_file_backed_prepared_attempt_qualifies() {
    let temp = tempdir().unwrap();
    let path = temp.path().join("runtime.sqlite");
    let mut store = SqliteIntegrationStore::open(&path).unwrap();
    let entry_id = match store.enqueue_outbound(&intent(900)).unwrap() {
        EnqueueDisposition::Inserted(value) => value,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };
    let claim = store
        .claim_outbox("worker-a", 1_000, 5_000, 1)
        .unwrap()
        .pop()
        .unwrap();
    assert_eq!(claim.entry_id, entry_id);

    let qualified = store
        .qualify_current_execution_attempt(&claim, "worker-a", 1_100)
        .unwrap();
    assert_eq!(qualified.claim(), &claim);
    assert_eq!(qualified.worker_id(), "worker-a");
    assert_eq!(qualified.durable_updated_at_ms(), 1_000);
    assert!(qualified.file_backed_store_bound_here());
    assert!(qualified.exact_prepared_attempt_observed_here());
    assert!(qualified.lease_current_at_read_here());
    assert!(qualified.quarantine_excluded_here());
    assert!(qualified.causal_frontier_checked_here());
    assert!(!qualified.atomic_with_dispatch_started_here());
    assert!(!qualified.coordinator_update_excluded_here());
    assert!(!qualified.grants_execution_authority());
}

#[test]
fn in_memory_runtime_cannot_mint_store_bound_attempt_proof() {
    let mut store = SqliteIntegrationStore::in_memory().unwrap();
    store.enqueue_outbound(&intent(900)).unwrap();
    let claim = store
        .claim_outbox("worker-a", 1_000, 5_000, 1)
        .unwrap()
        .pop()
        .unwrap();

    assert!(matches!(
        store.qualify_current_execution_attempt(&claim, "worker-a", 1_100),
        Err(CurrentAttemptQualificationError::FileBackedStoreRequired)
    ));
}

#[test]
fn forged_worker_and_lease_fail_closed() {
    let temp = tempdir().unwrap();
    let path = temp.path().join("runtime.sqlite");
    let mut store = SqliteIntegrationStore::open(&path).unwrap();
    store.enqueue_outbound(&intent(900)).unwrap();
    let claim = store
        .claim_outbox("worker-a", 1_000, 5_000, 1)
        .unwrap()
        .pop()
        .unwrap();

    assert!(matches!(
        store.qualify_current_execution_attempt(&claim, "worker-b", 1_100),
        Err(CurrentAttemptQualificationError::WorkerMismatch)
    ));

    let mut forged = claim.clone();
    forged.lease_until_ms += 1;
    assert!(matches!(
        store.qualify_current_execution_attempt(&forged, "worker-a", 1_100),
        Err(CurrentAttemptQualificationError::LeaseMismatchOrExpired)
    ));
}

#[test]
fn causal_time_cannot_move_behind_durable_attempt_frontier() {
    let temp = tempdir().unwrap();
    let path = temp.path().join("runtime.sqlite");
    let mut store = SqliteIntegrationStore::open(&path).unwrap();
    store.enqueue_outbound(&intent(900)).unwrap();
    let claim = store
        .claim_outbox("worker-a", 1_000, 5_000, 1)
        .unwrap()
        .pop()
        .unwrap();

    assert!(matches!(
        store.qualify_current_execution_attempt(&claim, "worker-a", 999),
        Err(CurrentAttemptQualificationError::CausalTimeRegression {
            durable_updated_at_ms: 1_000,
            observed_at_ms: 999,
        })
    ));
}

#[test]
fn prepared_attempt_proof_cannot_be_reused_after_dispatch_transition() {
    let temp = tempdir().unwrap();
    let path = temp.path().join("runtime.sqlite");
    let mut store = SqliteIntegrationStore::open(&path).unwrap();
    store.enqueue_outbound(&intent(900)).unwrap();
    let claim = store
        .claim_outbox("worker-a", 1_000, 5_000, 1)
        .unwrap()
        .pop()
        .unwrap();

    store
        .qualify_current_execution_attempt(&claim, "worker-a", 1_100)
        .unwrap();
    store
        .mark_dispatch_started(claim.entry_id, &claim.attempt_id, "worker-a", 1_200)
        .unwrap();

    assert!(matches!(
        store.qualify_current_execution_attempt(&claim, "worker-a", 1_300),
        Err(CurrentAttemptQualificationError::AttemptNotPrepared)
    ));
}

#[test]
fn expired_claim_cannot_requalify_even_before_recovery_runs() {
    let temp = tempdir().unwrap();
    let path = temp.path().join("runtime.sqlite");
    let mut store = SqliteIntegrationStore::open(&path).unwrap();
    store.enqueue_outbound(&intent(900)).unwrap();
    let claim = store
        .claim_outbox("worker-a", 1_000, 100, 1)
        .unwrap()
        .pop()
        .unwrap();

    assert!(matches!(
        store.qualify_current_execution_attempt(&claim, "worker-a", 1_100),
        Err(CurrentAttemptQualificationError::LeaseMismatchOrExpired)
    ));
}

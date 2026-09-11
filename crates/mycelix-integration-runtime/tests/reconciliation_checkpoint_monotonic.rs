use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalOpaqueId, ReconcileCursor,
};
use mycelix_integration_runtime::{RuntimeError, SqliteIntegrationStore};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("checkpoint-cas-connector")
        .expect("fixture connector must be valid")
}

fn checkpoint(cursor: &str, commitment: &[u8]) -> ReconcileCursor {
    ReconcileCursor {
        connector_instance: connector(),
        cursor: ExternalOpaqueId::new(cursor).expect("fixture cursor must be valid"),
        checkpoint_commitment: ContentCommitment::sha256(commitment),
    }
}

#[test]
fn checkpoint_currentness_is_monotonic_atomic_and_reopen_stable() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("checkpoint-cas.sqlite");
    let mut writer_a = SqliteIntegrationStore::open(&path).expect("first handle must open");
    let mut writer_b = SqliteIntegrationStore::open(&path).expect("second handle must open");

    let first = checkpoint("zzz-provider-cursor", b"checkpoint-a");
    writer_a
        .checkpoint_reconciliation(&first, 100)
        .expect("first checkpoint must insert");
    writer_a
        .checkpoint_reconciliation(&first, 100)
        .expect("exact same-time replay must be idempotent");

    let same_time_other_cursor = checkpoint("aaa-provider-cursor", b"checkpoint-a");
    assert!(matches!(
        writer_b.checkpoint_reconciliation(&same_time_other_cursor, 100),
        Err(RuntimeError::StoredIdentifier(message))
            if message.contains("equal-time conflict")
    ));

    let same_time_other_commitment = checkpoint("zzz-provider-cursor", b"checkpoint-b");
    assert!(matches!(
        writer_b.checkpoint_reconciliation(&same_time_other_commitment, 100),
        Err(RuntimeError::StoredIdentifier(message))
            if message.contains("equal-time conflict")
    ));

    assert!(matches!(
        writer_b.checkpoint_reconciliation(&first, 99),
        Err(RuntimeError::StoredIdentifier(message))
            if message.contains("checkpoint rollback")
    ));

    // Provider cursors are opaque: a lexically "smaller" cursor may become
    // current when the Mycelix checkpoint transition time advances.
    let later = checkpoint("aaa-provider-cursor", b"checkpoint-later");
    writer_b
        .checkpoint_reconciliation(&later, 110)
        .expect("newer Mycelix checkpoint time may advance an opaque cursor");

    // A stale concurrently open handle must not roll the durable checkpoint back.
    assert!(matches!(
        writer_a.checkpoint_reconciliation(&first, 105),
        Err(RuntimeError::StoredIdentifier(message))
            if message.contains("checkpoint rollback")
    ));

    let snapshot = writer_a
        .load_reconciliation_checkpoint_snapshot(&connector())
        .expect("checkpoint snapshot must load")
        .expect("checkpoint must exist");
    assert_eq!(snapshot.updated_at_ms, 110);
    assert_eq!(snapshot.cursor, later);

    drop(writer_a);
    drop(writer_b);

    let reopened = SqliteIntegrationStore::open(&path).expect("store must reopen");
    let reopened_snapshot = reopened
        .load_reconciliation_checkpoint_snapshot(&connector())
        .expect("checkpoint snapshot must load after reopen")
        .expect("checkpoint must survive reopen");
    assert_eq!(reopened_snapshot.updated_at_ms, 110);
    assert_eq!(reopened_snapshot.cursor, later);
}

#[test]
fn in_memory_checkpoint_obeys_the_same_cas_contract() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let first = checkpoint("cursor-a", b"checkpoint-a");
    store
        .checkpoint_reconciliation(&first, 10)
        .expect("first checkpoint must insert");
    store
        .checkpoint_reconciliation(&first, 10)
        .expect("same-time exact replay must be idempotent");

    let conflict = checkpoint("cursor-b", b"checkpoint-b");
    assert!(matches!(
        store.checkpoint_reconciliation(&conflict, 10),
        Err(RuntimeError::StoredIdentifier(message))
            if message.contains("equal-time conflict")
    ));
    assert!(matches!(
        store.checkpoint_reconciliation(&first, 9),
        Err(RuntimeError::StoredIdentifier(message))
            if message.contains("checkpoint rollback")
    ));
}

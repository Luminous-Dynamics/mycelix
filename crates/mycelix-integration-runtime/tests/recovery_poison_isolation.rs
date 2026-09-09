use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, IdempotencyKey, IntegrationCommandId, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, SqliteIntegrationStore,
};
use rusqlite::{params, Connection};

const EXECUTION_OBSERVATION_LIMIT: i64 = 4_096;

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("recovery-isolation-connector")
        .expect("fixture connector must be valid")
}

fn intent(name: &str) -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: IntegrationCommandId::new(name).expect("fixture command must be valid"),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(format!("command:{name}").as_bytes()),
        authority_commitment: ContentCommitment::sha256(format!("authority:{name}").as_bytes()),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new(format!("idem:{name}"))
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: format!("sealed:{name}").into_bytes(),
        created_at_ms: 100,
    }
}

fn enqueue(store: &mut SqliteIntegrationStore, name: &str) -> i64 {
    match store.enqueue_outbound(&intent(name)).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    }
}

#[test]
fn history_exhausted_stale_entry_does_not_block_unrelated_claims() {
    let temp = tempfile::tempdir().expect("tempdir must be created");
    let path = temp.path().join("recovery-isolation.sqlite");
    let poisoned_entry;
    let fresh_entry;

    {
        let mut store = SqliteIntegrationStore::open(&path).expect("store must open");
        poisoned_entry = enqueue(&mut store, "poisoned-command");
        fresh_entry = enqueue(&mut store, "fresh-command");

        let claim = store
            .claim_outbox("worker-poison", 110, 10, 1)
            .expect("first claim must succeed")
            .remove(0);
        assert_eq!(claim.entry_id, poisoned_entry);
        store
            .mark_dispatch_started(poisoned_entry, &claim.attempt_id, "worker-poison", 111)
            .expect("dispatch boundary must succeed");
    }

    // Build the hostile durable condition directly: the post-dispatch attempt is
    // now stale and its per-entry observation budget is already exhausted. A
    // boundedness failure for this one entry must not become global queue denial.
    {
        let conn = Connection::open(&path).expect("raw fixture connection must open");
        let tx = conn.unchecked_transaction().expect("fixture transaction must start");
        for sequence in 0..EXECUTION_OBSERVATION_LIMIT {
            tx.execute(
                "INSERT INTO integration_execution_observation (\n\
                    entry_id, attempt_id, outcome_json, observed_at_ms, applied_to_current\n\
                 ) VALUES (?1, ?2, ?3, ?4, 0)",
                params![
                    poisoned_entry,
                    "1:1",
                    br#"{"outcome":"fixture"}"#.as_slice(),
                    112 + sequence,
                ],
            )
            .expect("fixture history row must insert");
        }
        tx.commit().expect("fixture transaction must commit");
    }

    let mut store = SqliteIntegrationStore::open(&path).expect("store must reopen");
    let claimed = store
        .claim_outbox("worker-fresh", 121, 20, 1)
        .expect("one poisoned stale entry must not block unrelated eligible work");

    assert_eq!(claimed.len(), 1);
    assert_eq!(
        claimed[0].entry_id, fresh_entry,
        "recovery failure must remain isolated to the poisoned entry"
    );
}

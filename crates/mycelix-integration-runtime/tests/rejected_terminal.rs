use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExternalExecutionOutcome, ExternalOperationRef,
    ExternalRejection, ExternalRejectionCode, IdempotencyKey, IntegrationCommandId, OutboundStage,
    ReconciliationDisposition, ReconciliationResult, SideEffectClass,
};
use mycelix_integration_runtime::{
    DurableOutboundIntent, EnqueueDisposition, RuntimeError, SqliteIntegrationStore,
};

fn connector() -> ConnectorInstanceId {
    ConnectorInstanceId::new("rejection-test-connector").expect("fixture connector must be valid")
}

fn command_id() -> IntegrationCommandId {
    IntegrationCommandId::new("rejection-test-command").expect("fixture command must be valid")
}

fn operation() -> ExternalOperationRef {
    ExternalOperationRef {
        command_id: command_id(),
        connector_instance: connector(),
        provider_operation: None,
    }
}

fn intent() -> DurableOutboundIntent {
    DurableOutboundIntent {
        command_id: command_id(),
        connector_instance: connector(),
        command_commitment: ContentCommitment::sha256(b"rejection-test-command"),
        authority_commitment: ContentCommitment::sha256(b"rejection-test-authority"),
        side_effect_class: SideEffectClass::Irreversible,
        idempotency_key: Some(
            IdempotencyKey::new("rejection-test-idempotency")
                .expect("fixture idempotency key must be valid"),
        ),
        command_bytes: b"sealed-rejection-test-command".to_vec(),
        created_at_ms: 100,
    }
}

fn rejected() -> ExternalExecutionOutcome {
    ExternalExecutionOutcome::Rejected {
        operation: operation(),
        reason: ExternalRejection {
            code: ExternalRejectionCode::new("definite-pre-commit-denial")
                .expect("fixture rejection code must be valid"),
            detail_commitment: Some(ContentCommitment::sha256(b"provider-denial")),
            rejected_at_ms: 150,
        },
    }
}

fn confirms_no_effect() -> ReconciliationResult {
    ReconciliationResult {
        operation: operation(),
        disposition: ReconciliationDisposition::ConfirmsNoEffect,
        evidence: ContentCommitment::sha256(b"unnecessary-reconciliation"),
        reconciled_at_ms: 160,
    }
}

#[test]
fn definite_rejection_is_terminal_without_fabricated_reconciliation() {
    let mut store = SqliteIntegrationStore::in_memory().expect("store must open");
    let entry_id = match store.enqueue_outbound(&intent()).expect("enqueue must succeed") {
        EnqueueDisposition::Inserted(entry_id) => entry_id,
        other => panic!("unexpected enqueue disposition: {other:?}"),
    };

    let claim = store
        .claim_outbox("worker-reject", 110, 50, 1)
        .expect("claim must succeed")
        .remove(0);
    store
        .mark_dispatch_started(entry_id, &claim.attempt_id, "worker-reject", 111)
        .expect("dispatch boundary must succeed");
    store
        .record_execution(
            entry_id,
            &claim.attempt_id,
            "worker-reject",
            &rejected(),
            150,
        )
        .expect("definite rejection must be recorded");

    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Rejected
    );
    assert!(store
        .reconciliation_history(entry_id)
        .expect("history")
        .is_empty());

    assert!(matches!(
        store.record_reconciliation(entry_id, &confirms_no_effect(), 160),
        Err(RuntimeError::IllegalOutboundTransition {
            from: OutboundStage::Rejected,
            to: OutboundStage::Reconciled,
        })
    ));

    // The failed reconciliation attempt is rolled back; no fake evidence is kept.
    assert!(store
        .reconciliation_history(entry_id)
        .expect("history")
        .is_empty());
    assert!(matches!(
        store.finalize_outbound(entry_id, 170),
        Err(RuntimeError::IllegalOutboundTransition {
            from: OutboundStage::Rejected,
            to: OutboundStage::Finalized,
        })
    ));
    assert_eq!(
        store.outbox_snapshot(entry_id).expect("snapshot").stage,
        OutboundStage::Rejected
    );
}

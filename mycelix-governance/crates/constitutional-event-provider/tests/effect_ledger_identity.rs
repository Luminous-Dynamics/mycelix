use constitutional_effect_ledger::{
    ActionIntent, CapabilitySnapshot, CompensationCapability, ConstitutionalOperation,
    EFFECT_LEDGER_SCHEMA_VERSION, OutcomeObservability, ProviderBatchAtomicity, ReplaySafety,
};
use constitutional_event_provider::{
    DurableConstitutionalEvent, EventAuthorityBinding, PublishDecision, decide_publish,
};
use serde_json::json;

fn operation() -> ConstitutionalOperation {
    ConstitutionalOperation {
        schema_version: EFFECT_LEDGER_SCHEMA_VERSION,
        operation_id: "operation-001".into(),
        proposal_id: "proposal-003".into(),
        timelock_id: "timelock-004".into(),
        claim_binding: "claim-binding:abc123".into(),
        capability_profile_id: "provider-profile-unknown-v1".into(),
        capability_profile_revision: 1,
        ordered_action_commitments: vec!["action-commitment:def456".into()],
        committed_at_unix_ms: 1_000,
    }
}

fn intent() -> ActionIntent {
    ActionIntent {
        operation_id: "operation-001".into(),
        action_id: "action-002".into(),
        ordinal: 0,
        action_commitment: "action-commitment:def456".into(),
        capability: CapabilitySnapshot {
            action_type: "EmitEvent".into(),
            profile_id: "provider-profile-unknown-v1".into(),
            profile_revision: 1,
            replay_safety: ReplaySafety::Unknown,
            outcome_observability: OutcomeObservability::Unknown,
            compensation: CompensationCapability::Unknown,
            provider_batch_atomicity: ProviderBatchAtomicity::NoneObserved,
            safe_retry_without_reconciliation: false,
            qualification_evidence_id: None,
        },
        idempotency_identity: None,
    }
}

fn event_authority_from_d1c(
    operation: &ConstitutionalOperation,
    intent: &ActionIntent,
) -> EventAuthorityBinding {
    operation.validate().expect("valid D1C operation");
    intent
        .validate_against_operation(operation)
        .expect("valid D1C action intent");

    EventAuthorityBinding {
        operation_id: operation.operation_id.clone(),
        action_id: intent.action_id.clone(),
        proposal_id: operation.proposal_id.clone(),
        claim_binding_commitment: operation.claim_binding.clone(),
        action_commitment: intent.action_commitment.clone(),
        publisher_did: "did:mycelix:test-publisher".into(),
    }
}

#[test]
fn e0_reuses_d1c_operation_action_and_commitment_identity_without_regeneration() {
    let operation = operation();
    let intent = intent();
    let authority = event_authority_from_d1c(&operation, &intent);

    assert_eq!(authority.operation_id, operation.operation_id);
    assert_eq!(authority.action_id, intent.action_id);
    assert_eq!(authority.proposal_id, operation.proposal_id);
    assert_eq!(authority.claim_binding_commitment, operation.claim_binding);
    assert_eq!(authority.action_commitment, intent.action_commitment);
}

#[test]
fn d1c_retry_time_does_not_create_a_second_e0_event_identity() {
    let operation = operation();
    let intent = intent();
    let authority = event_authority_from_d1c(&operation, &intent);

    let first = DurableConstitutionalEvent::new(
        authority.clone(),
        "ProposalExecuted",
        &json!({"proposal_id": operation.proposal_id}),
        2_000,
    )
    .unwrap();
    let retry = DurableConstitutionalEvent::new(
        authority,
        "ProposalExecuted",
        &json!({"proposal_id": "proposal-003"}),
        9_000,
    )
    .unwrap();

    assert_eq!(first.provider_key(), intent.action_id);
    assert_eq!(first.event_commitment, retry.event_commitment);
    assert!(matches!(
        decide_publish(Some(&first), retry).unwrap(),
        PublishDecision::ExistingSame { .. }
    ));
}

#[test]
fn invalid_d1c_action_commitment_cannot_be_mapped_as_valid_event_authority() {
    let operation = operation();
    let mut bad_intent = intent();
    bad_intent.action_commitment = "different-action-commitment".into();

    assert!(bad_intent.validate_against_operation(&operation).is_err());
}

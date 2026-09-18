use constitutional_effect_ledger::{
    ActionIntent, CapabilitySnapshot, CompensationCapability, ConstitutionalOperation,
    OutcomeObservability, ProviderBatchAtomicity, ReplaySafety, EFFECT_LEDGER_SCHEMA_VERSION,
};
use constitutional_parameter_provider::{
    CanonicalParameterValue, ParameterAuthorityBinding, ParameterMutationRequest,
    PriorRevisionExpectation,
};

fn operation() -> ConstitutionalOperation {
    ConstitutionalOperation {
        schema_version: EFFECT_LEDGER_SCHEMA_VERSION,
        operation_id: "operation-parameter-001".into(),
        proposal_id: "proposal-parameter-003".into(),
        timelock_id: "timelock-parameter-004".into(),
        claim_binding: "claim-binding:opaque-qualified-later".into(),
        capability_profile_id: "parameter-provider-unknown-v1".into(),
        capability_profile_revision: 1,
        ordered_action_commitments: vec!["action-commitment:parameter-002".into()],
        committed_at_unix_ms: 1_000,
    }
}

fn intent() -> ActionIntent {
    ActionIntent {
        operation_id: "operation-parameter-001".into(),
        action_id: "action-parameter-002".into(),
        ordinal: 0,
        action_commitment: "action-commitment:parameter-002".into(),
        capability: CapabilitySnapshot {
            action_type: "UpdateParameter".into(),
            profile_id: "parameter-provider-unknown-v1".into(),
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

fn authority_from_d1c(
    operation: &ConstitutionalOperation,
    intent: &ActionIntent,
) -> ParameterAuthorityBinding {
    operation.validate().expect("valid D1C operation");
    intent
        .validate_against_operation(operation)
        .expect("valid D1C action intent");

    ParameterAuthorityBinding {
        operation_id: operation.operation_id.clone(),
        action_id: intent.action_id.clone(),
        proposal_id: operation.proposal_id.clone(),
        claim_binding_commitment: operation.claim_binding.clone(),
        action_commitment: intent.action_commitment.clone(),
        publisher_did: "did:mycelix:test-parameter-publisher".into(),
    }
}

#[test]
fn p0_reuses_d1c_operation_action_proposal_and_commitments_without_regeneration() {
    let operation = operation();
    let intent = intent();
    let authority = authority_from_d1c(&operation, &intent);

    assert_eq!(authority.operation_id, operation.operation_id);
    assert_eq!(authority.action_id, intent.action_id);
    assert_eq!(authority.proposal_id, operation.proposal_id);
    assert_eq!(authority.claim_binding_commitment, operation.claim_binding);
    assert_eq!(authority.action_commitment, intent.action_commitment);

    let request = ParameterMutationRequest::new(
        authority,
        "quorum_threshold",
        CanonicalParameterValue::Percentage("0.67".into()),
        PriorRevisionExpectation::CreateOnly,
    )
    .unwrap();
    assert_eq!(request.provider_key(), intent.action_id);
}

#[test]
fn d1c_action_commitment_mismatch_cannot_be_mapped_as_valid_parameter_authority() {
    let operation = operation();
    let mut bad_intent = intent();
    bad_intent.action_commitment = "different-action-commitment".into();

    assert!(bad_intent.validate_against_operation(&operation).is_err());
}

#[test]
fn d1c_claim_binding_remains_exact_opaque_material_in_p0() {
    let operation = operation();
    let intent = intent();
    let authority = authority_from_d1c(&operation, &intent);
    let request = ParameterMutationRequest::new(
        authority,
        "max_voting_weight",
        CanonicalParameterValue::Decimal("2.5".into()),
        PriorRevisionExpectation::CreateOnly,
    )
    .unwrap();

    assert_eq!(
        request.authority.claim_binding_commitment,
        operation.claim_binding
    );
    assert_eq!(request.authority.proposal_id, operation.proposal_id);
}

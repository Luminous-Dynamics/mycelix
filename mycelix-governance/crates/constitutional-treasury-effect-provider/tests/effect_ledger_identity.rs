use constitutional_effect_ledger::{
    ActionIntent, CapabilitySnapshot, CompensationCapability, ConstitutionalOperation,
    OutcomeObservability, ProviderBatchAtomicity, ReplaySafety, EFFECT_LEDGER_SCHEMA_VERSION,
};
use constitutional_treasury_effect_provider::{
    TreasuryEffectRequest, TreasuryEffectSubject,
};

fn operation_and_action() -> (ConstitutionalOperation, ActionIntent) {
    let capability = CapabilitySnapshot {
        action_type: "TransferCredits".into(),
        profile_id: "d1c-unproven-treasury-provider".into(),
        profile_revision: 1,
        replay_safety: ReplaySafety::Unknown,
        outcome_observability: OutcomeObservability::Unknown,
        compensation: CompensationCapability::Unknown,
        provider_batch_atomicity: ProviderBatchAtomicity::NoneObserved,
        safe_retry_without_reconciliation: false,
        qualification_evidence_id: None,
    };

    let operation = ConstitutionalOperation {
        schema_version: EFFECT_LEDGER_SCHEMA_VERSION,
        operation_id: "op-d1c-1".into(),
        proposal_id: "proposal-d1c-1".into(),
        timelock_id: "timelock-d1c-1".into(),
        claim_binding: "claim-binding-d1c-1".into(),
        capability_profile_id: capability.profile_id.clone(),
        capability_profile_revision: capability.profile_revision,
        ordered_action_commitments: vec!["action-commitment-d1c-1".into()],
        committed_at_unix_ms: 1,
    };

    let action = ActionIntent {
        operation_id: operation.operation_id.clone(),
        action_id: "action-d1c-1".into(),
        ordinal: 0,
        action_commitment: operation.ordered_action_commitments[0].clone(),
        capability,
        idempotency_identity: None,
    };

    operation.validate().unwrap();
    action.validate_against_operation(&operation).unwrap();
    (operation, action)
}

#[test]
fn f0_reuses_exact_d1c_operation_and_action_identity() {
    let (operation, action) = operation_and_action();

    let subject = TreasuryEffectSubject {
        operation_id: operation.operation_id.clone(),
        action_id: action.action_id.clone(),
        proposal_id: operation.proposal_id.clone(),
        claim_binding_commitment: operation.claim_binding.clone(),
        action_commitment: action.action_commitment.clone(),
        authorization_id: "authorization-1".into(),
        authorization_subject_commitment: "authorization-subject-1".into(),
        treasury_descriptor_commitment: "treasury-descriptor-1".into(),
        allocation_subject_commitment: "allocation-subject-1".into(),
        approval_projection_commitment: "approval-projection-1".into(),
        capacity_allocation_commitment: Some("capacity-allocation-1".into()),
        recipient_did: "did:mycelix:recipient".into(),
        recipient_commitment: "recipient-commitment-1".into(),
        value_profile_id: "pending-qualified-sap-amount-v1".into(),
        value_authority_commitment: "sap-authority-1".into(),
        policy_revision_commitment: "policy-revision-1".into(),
        adapter_profile_id: "treasury-adapter-v1".into(),
        effect_target_commitment: "provider-target-1".into(),
    };

    let request = TreasuryEffectRequest::new(subject).unwrap();

    assert_eq!(request.subject.operation_id, operation.operation_id);
    assert_eq!(request.subject.proposal_id, operation.proposal_id);
    assert_eq!(
        request.subject.claim_binding_commitment,
        operation.claim_binding
    );
    assert_eq!(request.subject.action_id, action.action_id);
    assert_eq!(request.subject.action_commitment, action.action_commitment);

    // F0 deliberately does not infer replay safety from D1C's structural identity.
    assert_eq!(action.capability.replay_safety, ReplaySafety::Unknown);
    assert_eq!(
        action.capability.outcome_observability,
        OutcomeObservability::Unknown
    );
}

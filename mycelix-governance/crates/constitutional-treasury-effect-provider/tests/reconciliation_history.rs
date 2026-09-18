use constitutional_treasury_effect_provider::{
    EffectIntent, EffectRecord, EffectState, ObservationDecision, ProviderObservation,
    ProviderObservationKind, RetryBasis, TreasuryEffectRequest, TreasuryEffectSubject,
};

fn request() -> TreasuryEffectRequest {
    TreasuryEffectRequest::new(TreasuryEffectSubject {
        operation_id: "op-1".into(),
        action_id: "action-1".into(),
        proposal_id: "proposal-1".into(),
        claim_binding_commitment: "claim-binding-1".into(),
        action_commitment: "action-commitment-1".into(),
        authorization_id: "authorization-1".into(),
        authorization_subject_commitment: "authorization-subject-1".into(),
        treasury_descriptor_commitment: "treasury-descriptor-1".into(),
        allocation_subject_commitment: "allocation-subject-1".into(),
        approval_projection_commitment: "approval-projection-1".into(),
        capacity_allocation_commitment: Some("capacity-allocation-1".into()),
        recipient_did: "did:mycelix:recipient".into(),
        recipient_commitment: "recipient-1".into(),
        value_profile_id: "pending-sap-v1".into(),
        value_authority_commitment: "value-1".into(),
        policy_revision_commitment: "policy-1".into(),
        adapter_profile_id: "adapter-1".into(),
        effect_target_commitment: "target-1".into(),
    })
    .unwrap()
}

fn no_effect(record: &EffectRecord) -> ProviderObservation {
    ProviderObservation {
        observation_id: "obs-no-effect-1".into(),
        execution_id: record.intent().execution_id.clone(),
        request_commitment: record.intent().request.request_commitment.clone(),
        covers_through_attempt_ordinal: 1,
        kind: ProviderObservationKind::KnownNoEffect {
            no_effect_evidence_id: "provider-no-effect-1".into(),
        },
        observed_at_unix_ms: 20,
    }
}

fn late_success_for_attempt_one(record: &EffectRecord) -> ProviderObservation {
    ProviderObservation {
        observation_id: "obs-late-success-1".into(),
        execution_id: record.intent().execution_id.clone(),
        request_commitment: record.intent().request.request_commitment.clone(),
        covers_through_attempt_ordinal: 1,
        kind: ProviderObservationKind::KnownSuccess {
            effect_commitment: "provider-effect-1".into(),
            external_receipt_id: "provider-receipt-1".into(),
            receipt_commitment: "provider-receipt-commitment-1".into(),
        },
        observed_at_unix_ms: 40,
    }
}

/// An authoritative no-effect observation that was used to justify a retry is
/// part of durable reconciliation history. A later receipt claiming that the
/// already-reconciled attempt actually succeeded contradicts that history even
/// while a newer retry is InFlight; it must not silently become KnownSuccess.
#[test]
fn late_success_cannot_overwrite_prior_no_effect_reconciliation() {
    let intent = EffectIntent::new(request(), 1).unwrap();
    let mut record = EffectRecord::new(intent).unwrap();

    record
        .begin_attempt("attempt-1", 10, RetryBasis::Initial)
        .unwrap();
    record
        .record_transport_failure("attempt-1", "transport-timeout")
        .unwrap();

    let no_effect_observation = no_effect(&record);
    let decision = record.apply_observation(no_effect_observation).unwrap();
    assert_eq!(decision, ObservationDecision::Accepted);

    record
        .begin_attempt(
            "attempt-2",
            30,
            RetryBasis::ReconciledNoEffect {
                observation_id: "obs-no-effect-1".into(),
            },
        )
        .unwrap();

    let late_success = late_success_for_attempt_one(&record);
    let decision = record.apply_observation(late_success).unwrap();

    assert!(matches!(decision, ObservationDecision::IntegrityConflict { .. }));
    assert!(matches!(record.state(), EffectState::IntegrityHalted { .. }));
}

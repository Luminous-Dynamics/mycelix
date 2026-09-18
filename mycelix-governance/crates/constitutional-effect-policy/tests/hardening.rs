use constitutional_effect_policy::*;

fn atomic_provider(reversibility: Reversibility) -> ProviderCapabilityProfile {
    ProviderCapabilityProfile {
        provider_id: "provider-a".into(),
        version: "v1".into(),
        effect_semantics: ProviderEffectSemantics::PersistsDurableStateMutation,
        delivery: DeliverySemantics::IdempotentByOperationKey,
        reconciliation: ReconciliationCapability::AuthoritativeQueryByOperationId,
        observability: OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        reversibility,
        batch_atomicity: BatchAtomicity::ProviderAtomicBatch,
        evidence_ref: Some("evidence:provider-a:v1".into()),
    }
}

fn action(id: &str, provider: ProviderCapabilityProfile) -> ActionEffectProfile {
    ActionEffectProfile {
        action_id: id.into(),
        effect_class: EffectClass::DurableStateMutation,
        provider,
        requires_durable_success: true,
    }
}

#[test]
fn empty_batch_is_not_a_valid_execution_plan() {
    assert_eq!(
        plan_batch(&[], "policy-v1", &BatchPlanningOptions::default()),
        Err(AdmissibilityError::EmptyBatch)
    );
}

#[test]
fn same_provider_and_version_are_not_enough_to_claim_atomic_batch() {
    let a = action("a", atomic_provider(Reversibility::ExactlyReversible));
    let b = action(
        "b",
        atomic_provider(Reversibility::CompensableWithNewAuthority),
    );
    let plan = plan_batch(&[a, b], "policy-v1", &BatchPlanningOptions::default()).unwrap();
    assert_eq!(plan.durable_strategy, BatchStrategy::SequentialCheckpointed);
    assert_eq!(plan.partial_completion, PartialCompletionSemantics::Required);
}

#[test]
fn notification_only_batch_never_claims_durable_execution() {
    let actions = vec![
        conservative_existing_action_profile(ExistingGovernanceActionKind::EmitEvent, "n1"),
        conservative_existing_action_profile(ExistingGovernanceActionKind::EmitEvent, "n2"),
    ];
    let plan = plan_batch(&actions, "policy-v1", &BatchPlanningOptions::default()).unwrap();
    assert_eq!(plan.durable_strategy, BatchStrategy::BestEffortNotificationFanout);
    assert!(plan.durable_actions.is_empty());
    assert_eq!(plan.post_commit_notifications.len(), 2);
    assert_eq!(plan.partial_completion, PartialCompletionSemantics::NotApplicable);
}

use constitutional_effect_policy::*;

fn provider(
    id: &str,
    delivery: DeliverySemantics,
    reconciliation: ReconciliationCapability,
    observability: OutcomeObservability,
    reversibility: Reversibility,
    batch_atomicity: BatchAtomicity,
) -> ProviderCapabilityProfile {
    ProviderCapabilityProfile {
        provider_id: id.into(),
        version: "v1".into(),
        effect_semantics: ProviderEffectSemantics::PersistsDurableStateMutation,
        delivery,
        reconciliation,
        observability,
        reversibility,
        batch_atomicity,
        evidence_ref: Some(format!("evidence:{id}:v1")),
    }
}

fn durable_action(id: &str, provider: ProviderCapabilityProfile) -> ActionEffectProfile {
    ActionEffectProfile {
        action_id: id.into(),
        effect_class: EffectClass::DurableStateMutation,
        provider,
        requires_durable_success: true,
    }
}

#[test]
fn best_effort_notification_cannot_masquerade_as_durable_success() {
    let mut action = conservative_existing_action_profile(
        ExistingGovernanceActionKind::EmitEvent,
        "notify",
    );
    action.requires_durable_success = true;
    assert_eq!(
        validate_action_profile(&action),
        Err(AdmissibilityError::NotificationCannotRequireDurableSuccess)
    );
}

#[test]
fn durable_effect_cannot_opt_out_of_durable_success_requirement() {
    let mut action = durable_action(
        "a",
        provider(
            "p",
            DeliverySemantics::IdempotentByOperationKey,
            ReconciliationCapability::AuthoritativeQueryByOperationId,
            OutcomeObservability::AuthoritativeSuccessAndNoEffect,
            Reversibility::ExactlyReversible,
            BatchAtomicity::PerOperationOnly,
        ),
    );
    action.requires_durable_success = false;
    assert_eq!(
        validate_action_profile(&action),
        Err(AdmissibilityError::DurableEffectMustRequireDurableSuccess)
    );
}

#[test]
fn capability_stronger_than_unproven_requires_evidence() {
    let mut p = provider(
        "p",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::ExactlyReversible,
        BatchAtomicity::PerOperationOnly,
    );
    p.evidence_ref = None;
    assert_eq!(
        validate_provider_profile(&p),
        Err(AdmissibilityError::MissingCapabilityEvidence)
    );
}

#[test]
fn transfer_bridge_intent_record_cannot_satisfy_value_transfer() {
    let action = conservative_existing_action_profile(
        ExistingGovernanceActionKind::TransferCredits,
        "transfer",
    );
    assert!(action.requires_durable_success);
    assert_eq!(
        action.provider.effect_semantics,
        ProviderEffectSemantics::RecordsIntentOrAuditOnly
    );
    assert_eq!(
        validate_action_profile(&action),
        Err(AdmissibilityError::ProviderEffectMismatch)
    );
    assert_eq!(
        plan_batch(&[action], "policy-v1", &BatchPlanningOptions::default()),
        Err(AdmissibilityError::ProviderEffectMismatch)
    );
}

#[test]
fn update_parameter_effect_matches_but_delivery_contract_is_unproven() {
    let action = conservative_existing_action_profile(
        ExistingGovernanceActionKind::UpdateParameter,
        "parameter",
    );
    assert_eq!(
        action.provider.effect_semantics,
        ProviderEffectSemantics::PersistsDurableStateMutation
    );
    assert_eq!(
        validate_action_profile(&action),
        Err(AdmissibilityError::UnprovenProviderForDurableAction)
    );
}

#[test]
fn non_idempotent_provider_with_authoritative_no_effect_query_requires_reconciliation() {
    let action = durable_action(
        "a",
        provider(
            "p",
            DeliverySemantics::NonIdempotent,
            ReconciliationCapability::AuthoritativeQueryByOperationId,
            OutcomeObservability::AuthoritativeSuccessAndNoEffect,
            Reversibility::ExactlyReversible,
            BatchAtomicity::PerOperationOnly,
        ),
    );
    assert_eq!(
        classify_retry_policy(&action, true).unwrap(),
        RetryPolicy::ReconcileBeforeRetry
    );
}

#[test]
fn non_idempotent_nonqueryable_provider_never_auto_retries() {
    let action = durable_action(
        "a",
        provider(
            "p",
            DeliverySemantics::NonIdempotent,
            ReconciliationCapability::None,
            OutcomeObservability::None,
            Reversibility::ExactlyReversible,
            BatchAtomicity::PerOperationOnly,
        ),
    );
    assert_eq!(
        classify_retry_policy(&action, true).unwrap(),
        RetryPolicy::NeverAutomatic
    );
}

#[test]
fn unstable_operation_identity_disables_automatic_retry() {
    let action = durable_action(
        "a",
        provider(
            "p",
            DeliverySemantics::IdempotentByOperationKey,
            ReconciliationCapability::AuthoritativeQueryByOperationId,
            OutcomeObservability::AuthoritativeSuccessAndNoEffect,
            Reversibility::ExactlyReversible,
            BatchAtomicity::PerOperationOnly,
        ),
    );
    assert_eq!(
        classify_retry_policy(&action, false).unwrap(),
        RetryPolicy::NeverAutomatic
    );
}

#[test]
fn irreversible_nonqueryable_multi_action_batch_is_rejected() {
    let p = provider(
        "p",
        DeliverySemantics::NonIdempotent,
        ReconciliationCapability::None,
        OutcomeObservability::None,
        Reversibility::Irreversible,
        BatchAtomicity::None,
    );
    let actions = vec![durable_action("a", p.clone()), durable_action("b", p)];
    assert_eq!(
        plan_batch(&actions, "policy-v1", &BatchPlanningOptions::default()),
        Err(AdmissibilityError::MultiActionRequiresSingleAction)
    );
}

#[test]
fn same_provider_atomic_batch_suppresses_partial_completion_claim() {
    let p = provider(
        "p",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::Irreversible,
        BatchAtomicity::ProviderAtomicBatch,
    );
    let actions = vec![durable_action("a", p.clone()), durable_action("b", p)];
    let plan = plan_batch(&actions, "policy-v1", &BatchPlanningOptions::default()).unwrap();
    assert_eq!(plan.durable_strategy, BatchStrategy::ProviderAtomicBatch);
    assert_eq!(
        plan.partial_completion,
        PartialCompletionSemantics::ImpossibleByProviderAtomicity
    );
}

#[test]
fn sequential_multi_action_plan_requires_partial_completion_semantics() {
    let p1 = provider(
        "p1",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::ExactlyReversible,
        BatchAtomicity::PerOperationOnly,
    );
    let p2 = provider(
        "p2",
        DeliverySemantics::NonIdempotent,
        ReconciliationCapability::ProviderLedger,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::CompensableWithNewAuthority,
        BatchAtomicity::PerOperationOnly,
    );
    let actions = vec![durable_action("a", p1), durable_action("b", p2)];
    let plan = plan_batch(&actions, "policy-v1", &BatchPlanningOptions::default()).unwrap();
    assert_eq!(plan.durable_strategy, BatchStrategy::SequentialCheckpointed);
    assert_eq!(plan.partial_completion, PartialCompletionSemantics::Required);
}

#[test]
fn saga_requires_explicitly_bound_compensation_authority() {
    let p = provider(
        "p",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::CompensableWithNewAuthority,
        BatchAtomicity::PerOperationOnly,
    );
    let actions = vec![durable_action("a", p.clone()), durable_action("b", p)];
    let options = BatchPlanningOptions {
        stable_operation_identity: true,
        allow_saga: true,
        compensation_authority_bound: false,
    };
    assert_eq!(
        plan_batch(&actions, "policy-v1", &options),
        Err(AdmissibilityError::CompensationAuthorityMissing)
    );
}

#[test]
fn saga_with_bound_compensation_authority_is_explicit() {
    let p = provider(
        "p",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::CompensableWithNewAuthority,
        BatchAtomicity::PerOperationOnly,
    );
    let actions = vec![durable_action("a", p.clone()), durable_action("b", p)];
    let options = BatchPlanningOptions {
        stable_operation_identity: true,
        allow_saga: true,
        compensation_authority_bound: true,
    };
    let plan = plan_batch(&actions, "policy-v1", &options).unwrap();
    assert_eq!(plan.durable_strategy, BatchStrategy::SagaWithExplicitCompensation);
    assert!(plan.compensation_authority_bound);
    assert_eq!(plan.partial_completion, PartialCompletionSemantics::Required);
}

#[test]
fn notifications_are_split_to_post_commit_fanout() {
    let p = provider(
        "p",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::ExactlyReversible,
        BatchAtomicity::PerOperationOnly,
    );
    let actions = vec![
        durable_action("durable", p),
        conservative_existing_action_profile(ExistingGovernanceActionKind::EmitEvent, "notify"),
    ];
    let plan = plan_batch(&actions, "policy-v1", &BatchPlanningOptions::default()).unwrap();
    assert_eq!(plan.durable_strategy, BatchStrategy::SingleActionOnly);
    assert_eq!(plan.durable_actions.len(), 1);
    assert_eq!(plan.post_commit_notifications.len(), 1);
    assert_eq!(plan.post_commit_notifications[0].action.action_id, "notify");
}

#[test]
fn plan_is_deterministic_for_identical_inputs() {
    let p = provider(
        "p",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::ExactlyReversible,
        BatchAtomicity::PerOperationOnly,
    );
    let actions = vec![durable_action("a", p.clone()), durable_action("b", p)];
    let options = BatchPlanningOptions::default();
    assert_eq!(
        plan_batch(&actions, "policy-v1", &options).unwrap(),
        plan_batch(&actions, "policy-v1", &options).unwrap()
    );
}

#[test]
fn provider_profile_drift_invalidates_committed_plan() {
    let p = provider(
        "p",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::ExactlyReversible,
        BatchAtomicity::PerOperationOnly,
    );
    let actions = vec![durable_action("a", p)];
    let options = BatchPlanningOptions::default();
    let plan = plan_batch(&actions, "policy-v1", &options).unwrap();

    let mut changed = actions.clone();
    changed[0].provider.version = "v2".into();
    changed[0].provider.evidence_ref = Some("evidence:p:v2".into());
    assert_eq!(
        validate_batch_plan(&plan, &changed, "policy-v1", &options),
        Err(AdmissibilityError::StaleOrDifferentPlan)
    );
}

#[test]
fn action_reordering_invalidates_committed_plan() {
    let p1 = provider(
        "p1",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::ExactlyReversible,
        BatchAtomicity::PerOperationOnly,
    );
    let p2 = provider(
        "p2",
        DeliverySemantics::IdempotentByOperationKey,
        ReconciliationCapability::AuthoritativeQueryByOperationId,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect,
        Reversibility::ExactlyReversible,
        BatchAtomicity::PerOperationOnly,
    );
    let actions = vec![durable_action("a", p1), durable_action("b", p2)];
    let options = BatchPlanningOptions::default();
    let plan = plan_batch(&actions, "policy-v1", &options).unwrap();
    let reordered = vec![actions[1].clone(), actions[0].clone()];
    assert_eq!(
        validate_batch_plan(&plan, &reordered, "policy-v1", &options),
        Err(AdmissibilityError::StaleOrDifferentPlan)
    );
}

#[test]
fn current_action_classification_does_not_invent_provider_guarantees() {
    let transfer = conservative_existing_action_profile(
        ExistingGovernanceActionKind::TransferCredits,
        "transfer",
    );
    assert!(transfer.requires_durable_success);
    assert_eq!(transfer.effect_class, EffectClass::ValueTransfer);
    assert_eq!(
        transfer.provider.effect_semantics,
        ProviderEffectSemantics::RecordsIntentOrAuditOnly
    );
    assert_eq!(
        validate_action_profile(&transfer),
        Err(AdmissibilityError::ProviderEffectMismatch)
    );

    let update = conservative_existing_action_profile(
        ExistingGovernanceActionKind::UpdateParameter,
        "parameter",
    );
    assert!(update.requires_durable_success);
    assert_eq!(
        update.provider.effect_semantics,
        ProviderEffectSemantics::PersistsDurableStateMutation
    );
    assert_eq!(update.provider.delivery, DeliverySemantics::Unproven);
    assert_eq!(
        update.provider.reconciliation,
        ReconciliationCapability::Unproven
    );
    assert_eq!(
        validate_action_profile(&update),
        Err(AdmissibilityError::UnprovenProviderForDurableAction)
    );

    let signal = conservative_existing_action_profile(ExistingGovernanceActionKind::EmitEvent, "s");
    assert_eq!(signal.effect_class, EffectClass::BestEffortNotification);
    assert_eq!(
        signal.provider.effect_semantics,
        ProviderEffectSemantics::EmitsBestEffortNotification
    );
    assert!(!signal.requires_durable_success);
    assert!(validate_action_profile(&signal).is_ok());
}

#[test]
fn durable_unknown_outcome_is_always_a_blocking_state() {
    let action = durable_action(
        "a",
        provider(
            "p",
            DeliverySemantics::IdempotentByOperationKey,
            ReconciliationCapability::AuthoritativeQueryByOperationId,
            OutcomeObservability::AuthoritativeSuccessAndNoEffect,
            Reversibility::ExactlyReversible,
            BatchAtomicity::PerOperationOnly,
        ),
    );
    assert!(requires_unknown_outcome_block(&action));

    let notification = conservative_existing_action_profile(
        ExistingGovernanceActionKind::EmitEvent,
        "n",
    );
    assert!(!requires_unknown_outcome_block(&notification));
}

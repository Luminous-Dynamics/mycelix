use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectClass {
    DurableStateMutation,
    ValueTransfer,
    BestEffortNotification,
    ExternalIrreversibleEffect,
}

/// What effect the provider endpoint itself can establish.
/// This is deliberately separate from delivery/idempotency: an endpoint may be
/// perfectly idempotent while still establishing the wrong semantic effect.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProviderEffectSemantics {
    PersistsDurableStateMutation,
    SettlesValueTransfer,
    EmitsBestEffortNotification,
    ExecutesExternalIrreversibleEffect,
    RecordsIntentOrAuditOnly,
    Unproven,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeliverySemantics {
    IdempotentByOperationKey,
    NaturallyIdempotent,
    AtMostOnceAttempt,
    NonIdempotent,
    Unproven,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReconciliationCapability {
    AuthoritativeQueryByOperationId,
    AuthoritativeReceiptLookup,
    ProviderLedger,
    None,
    Unproven,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutcomeObservability {
    AuthoritativeSuccessAndNoEffect,
    AuthoritativeSuccessOnly,
    None,
    Unproven,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reversibility {
    ExactlyReversible,
    CompensableWithNewAuthority,
    CompensableWithoutNewAuthority,
    Irreversible,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatchAtomicity {
    ProviderAtomicBatch,
    PerOperationOnly,
    None,
    Unproven,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProviderCapabilityProfile {
    pub provider_id: String,
    pub version: String,
    pub effect_semantics: ProviderEffectSemantics,
    pub delivery: DeliverySemantics,
    pub reconciliation: ReconciliationCapability,
    pub observability: OutcomeObservability,
    pub reversibility: Reversibility,
    pub batch_atomicity: BatchAtomicity,
    /// Reference to the evidence establishing any capability stronger than
    /// `Unproven`. The pure policy layer treats this as opaque provenance.
    pub evidence_ref: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActionEffectProfile {
    pub action_id: String,
    pub effect_class: EffectClass,
    pub provider: ProviderCapabilityProfile,
    /// Whether this action must establish durable constitutional success.
    /// All non-notification effects must set this true; best-effort
    /// notifications must set it false.
    pub requires_durable_success: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetryPolicy {
    /// Capability permits retry only with the exact same stable operation ID.
    /// A runtime may choose a stricter reconciliation-first policy.
    AutoRetrySameOperation,
    ReconcileBeforeRetry,
    NeverAutomatic,
    NotApplicable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatchStrategy {
    SingleActionOnly,
    ProviderAtomicBatch,
    SequentialCheckpointed,
    SagaWithExplicitCompensation,
    BestEffortNotificationFanout,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartialCompletionSemantics {
    NotApplicable,
    ImpossibleByProviderAtomicity,
    Required,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlannedAction {
    pub position: usize,
    pub action: ActionEffectProfile,
    pub retry_policy: RetryPolicy,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BatchPlan {
    pub policy_version: String,
    pub durable_strategy: BatchStrategy,
    pub durable_actions: Vec<PlannedAction>,
    pub post_commit_notifications: Vec<PlannedAction>,
    pub partial_completion: PartialCompletionSemantics,
    pub compensation_authority_bound: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BatchPlanningOptions {
    pub stable_operation_identity: bool,
    pub allow_saga: bool,
    pub compensation_authority_bound: bool,
}

impl Default for BatchPlanningOptions {
    fn default() -> Self {
        Self {
            stable_operation_identity: true,
            allow_saga: false,
            compensation_authority_bound: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissibilityError {
    EmptyProviderId,
    EmptyProviderVersion,
    EmptyActionId,
    EmptyPolicyVersion,
    EmptyBatch,
    DuplicateActionId,
    MissingCapabilityEvidence,
    InconsistentProviderProfile,
    NotificationCannotRequireDurableSuccess,
    DurableEffectMustRequireDurableSuccess,
    ProviderEffectMismatch,
    UnprovenProviderForDurableAction,
    MultiActionRequiresSingleAction,
    CompensationAuthorityMissing,
    StaleOrDifferentPlan,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn has_stronger_than_unproven(profile: &ProviderCapabilityProfile) -> bool {
    !matches!(profile.effect_semantics, ProviderEffectSemantics::Unproven)
        || !matches!(profile.delivery, DeliverySemantics::Unproven)
        || !matches!(profile.reconciliation, ReconciliationCapability::Unproven)
        || !matches!(profile.observability, OutcomeObservability::Unproven)
        || !matches!(profile.reversibility, Reversibility::Unknown)
        || !matches!(profile.batch_atomicity, BatchAtomicity::Unproven)
}

pub fn validate_provider_profile(
    profile: &ProviderCapabilityProfile,
) -> Result<(), AdmissibilityError> {
    if !nonempty(&profile.provider_id) {
        return Err(AdmissibilityError::EmptyProviderId);
    }
    if !nonempty(&profile.version) {
        return Err(AdmissibilityError::EmptyProviderVersion);
    }
    if has_stronger_than_unproven(profile)
        && !profile
            .evidence_ref
            .as_deref()
            .map(nonempty)
            .unwrap_or(false)
    {
        return Err(AdmissibilityError::MissingCapabilityEvidence);
    }

    let reconciliation_is_authoritative = matches!(
        profile.reconciliation,
        ReconciliationCapability::AuthoritativeQueryByOperationId
            | ReconciliationCapability::AuthoritativeReceiptLookup
            | ReconciliationCapability::ProviderLedger
    );
    let observability_is_authoritative = matches!(
        profile.observability,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect
            | OutcomeObservability::AuthoritativeSuccessOnly
    );
    if reconciliation_is_authoritative && !observability_is_authoritative {
        return Err(AdmissibilityError::InconsistentProviderProfile);
    }

    if matches!(profile.batch_atomicity, BatchAtomicity::ProviderAtomicBatch)
        && matches!(profile.delivery, DeliverySemantics::Unproven)
    {
        return Err(AdmissibilityError::InconsistentProviderProfile);
    }

    Ok(())
}

fn provider_effect_matches(action: &ActionEffectProfile) -> bool {
    matches!(
        (action.effect_class, action.provider.effect_semantics),
        (
            EffectClass::DurableStateMutation,
            ProviderEffectSemantics::PersistsDurableStateMutation
        ) | (
            EffectClass::ValueTransfer,
            ProviderEffectSemantics::SettlesValueTransfer
        ) | (
            EffectClass::BestEffortNotification,
            ProviderEffectSemantics::EmitsBestEffortNotification
        ) | (
            EffectClass::ExternalIrreversibleEffect,
            ProviderEffectSemantics::ExecutesExternalIrreversibleEffect
        )
    )
}

pub fn validate_action_profile(action: &ActionEffectProfile) -> Result<(), AdmissibilityError> {
    if !nonempty(&action.action_id) {
        return Err(AdmissibilityError::EmptyActionId);
    }
    validate_provider_profile(&action.provider)?;

    let is_notification = matches!(action.effect_class, EffectClass::BestEffortNotification);
    if is_notification && action.requires_durable_success {
        return Err(AdmissibilityError::NotificationCannotRequireDurableSuccess);
    }
    if !is_notification && !action.requires_durable_success {
        return Err(AdmissibilityError::DurableEffectMustRequireDurableSuccess);
    }
    if action.requires_durable_success
        && matches!(action.provider.effect_semantics, ProviderEffectSemantics::Unproven)
    {
        return Err(AdmissibilityError::UnprovenProviderForDurableAction);
    }
    if !provider_effect_matches(action) {
        return Err(AdmissibilityError::ProviderEffectMismatch);
    }
    if action.requires_durable_success
        && (matches!(action.provider.delivery, DeliverySemantics::Unproven)
            || matches!(
                action.provider.reconciliation,
                ReconciliationCapability::Unproven
            )
            || matches!(action.provider.observability, OutcomeObservability::Unproven))
    {
        return Err(AdmissibilityError::UnprovenProviderForDurableAction);
    }
    Ok(())
}

fn can_authoritatively_prove_no_effect(profile: &ProviderCapabilityProfile) -> bool {
    matches!(
        profile.observability,
        OutcomeObservability::AuthoritativeSuccessAndNoEffect
    ) && matches!(
        profile.reconciliation,
        ReconciliationCapability::AuthoritativeQueryByOperationId
            | ReconciliationCapability::AuthoritativeReceiptLookup
            | ReconciliationCapability::ProviderLedger
    )
}

pub fn classify_retry_policy(
    action: &ActionEffectProfile,
    stable_operation_identity: bool,
) -> Result<RetryPolicy, AdmissibilityError> {
    validate_action_profile(action)?;

    if matches!(action.effect_class, EffectClass::BestEffortNotification) {
        return Ok(RetryPolicy::NotApplicable);
    }
    if !stable_operation_identity {
        return Ok(RetryPolicy::NeverAutomatic);
    }

    let policy = match action.provider.delivery {
        DeliverySemantics::IdempotentByOperationKey | DeliverySemantics::NaturallyIdempotent => {
            RetryPolicy::AutoRetrySameOperation
        }
        DeliverySemantics::AtMostOnceAttempt => RetryPolicy::NeverAutomatic,
        DeliverySemantics::NonIdempotent => {
            if can_authoritatively_prove_no_effect(&action.provider) {
                RetryPolicy::ReconcileBeforeRetry
            } else {
                RetryPolicy::NeverAutomatic
            }
        }
        DeliverySemantics::Unproven => RetryPolicy::NeverAutomatic,
    };
    Ok(policy)
}

fn provider_atomic_batch(actions: &[PlannedAction]) -> bool {
    let Some(first) = actions.first() else {
        return false;
    };
    actions.iter().all(|planned| {
        planned.action.provider == first.action.provider
            && matches!(
                planned.action.provider.batch_atomicity,
                BatchAtomicity::ProviderAtomicBatch
            )
    })
}

fn all_explicitly_compensable(actions: &[PlannedAction]) -> bool {
    actions.iter().all(|planned| {
        matches!(
            planned.action.provider.reversibility,
            Reversibility::ExactlyReversible
                | Reversibility::CompensableWithNewAuthority
                | Reversibility::CompensableWithoutNewAuthority
        )
    })
}

fn unsafe_irreversible_or_unknown(action: &PlannedAction) -> bool {
    matches!(
        action.action.provider.reversibility,
        Reversibility::Irreversible | Reversibility::Unknown
    ) && !can_authoritatively_prove_no_effect(&action.action.provider)
}

pub fn plan_batch(
    actions: &[ActionEffectProfile],
    policy_version: &str,
    options: &BatchPlanningOptions,
) -> Result<BatchPlan, AdmissibilityError> {
    if !nonempty(policy_version) {
        return Err(AdmissibilityError::EmptyPolicyVersion);
    }
    if actions.is_empty() {
        return Err(AdmissibilityError::EmptyBatch);
    }

    let mut seen = BTreeSet::new();
    let mut durable = Vec::new();
    let mut notifications = Vec::new();

    for (position, action) in actions.iter().enumerate() {
        validate_action_profile(action)?;
        if !seen.insert(action.action_id.clone()) {
            return Err(AdmissibilityError::DuplicateActionId);
        }
        let planned = PlannedAction {
            position,
            action: action.clone(),
            retry_policy: classify_retry_policy(action, options.stable_operation_identity)?,
        };
        if matches!(action.effect_class, EffectClass::BestEffortNotification) {
            notifications.push(planned);
        } else {
            durable.push(planned);
        }
    }

    let (strategy, partial) = match durable.len() {
        0 => (
            BatchStrategy::BestEffortNotificationFanout,
            PartialCompletionSemantics::NotApplicable,
        ),
        1 => (
            BatchStrategy::SingleActionOnly,
            PartialCompletionSemantics::NotApplicable,
        ),
        _ if provider_atomic_batch(&durable) => (
            BatchStrategy::ProviderAtomicBatch,
            PartialCompletionSemantics::ImpossibleByProviderAtomicity,
        ),
        _ => {
            if durable.iter().any(unsafe_irreversible_or_unknown) {
                return Err(AdmissibilityError::MultiActionRequiresSingleAction);
            }
            if durable
                .iter()
                .any(|a| matches!(a.retry_policy, RetryPolicy::NeverAutomatic))
            {
                return Err(AdmissibilityError::MultiActionRequiresSingleAction);
            }

            if options.allow_saga && all_explicitly_compensable(&durable) {
                if !options.compensation_authority_bound {
                    return Err(AdmissibilityError::CompensationAuthorityMissing);
                }
                (
                    BatchStrategy::SagaWithExplicitCompensation,
                    PartialCompletionSemantics::Required,
                )
            } else {
                (
                    BatchStrategy::SequentialCheckpointed,
                    PartialCompletionSemantics::Required,
                )
            }
        }
    };

    Ok(BatchPlan {
        policy_version: policy_version.to_owned(),
        durable_strategy: strategy,
        durable_actions: durable,
        post_commit_notifications: notifications,
        partial_completion: partial,
        compensation_authority_bound: options.compensation_authority_bound,
    })
}

pub fn validate_batch_plan(
    plan: &BatchPlan,
    actions: &[ActionEffectProfile],
    policy_version: &str,
    options: &BatchPlanningOptions,
) -> Result<(), AdmissibilityError> {
    let expected = plan_batch(actions, policy_version, options)?;
    if &expected != plan {
        return Err(AdmissibilityError::StaleOrDifferentPlan);
    }
    Ok(())
}

pub fn requires_unknown_outcome_block(action: &ActionEffectProfile) -> bool {
    !matches!(action.effect_class, EffectClass::BestEffortNotification)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExistingGovernanceActionKind {
    TransferCredits,
    UpdateParameter,
    EmitEvent,
}

/// Conservative classifications for the current execution coordinator.
/// These deliberately encode only what the current code proves, not what the
/// called zomes/providers might happen to guarantee in practice.
pub fn conservative_existing_action_profile(
    kind: ExistingGovernanceActionKind,
    action_id: impl Into<String>,
) -> ActionEffectProfile {
    let action_id = action_id.into();
    match kind {
        ExistingGovernanceActionKind::TransferCredits => ActionEffectProfile {
            action_id,
            effect_class: EffectClass::ValueTransfer,
            provider: ProviderCapabilityProfile {
                provider_id: "governance_bridge.transfer_credits".into(),
                version: "current-intent-record-only".into(),
                effect_semantics: ProviderEffectSemantics::RecordsIntentOrAuditOnly,
                delivery: DeliverySemantics::Unproven,
                reconciliation: ReconciliationCapability::Unproven,
                observability: OutcomeObservability::Unproven,
                reversibility: Reversibility::Unknown,
                batch_atomicity: BatchAtomicity::Unproven,
                evidence_ref: Some(
                    "current-code:governance-bridge-transfer-credits-records-event-only".into(),
                ),
            },
            requires_durable_success: true,
        },
        ExistingGovernanceActionKind::UpdateParameter => ActionEffectProfile {
            action_id,
            effect_class: EffectClass::DurableStateMutation,
            provider: ProviderCapabilityProfile {
                provider_id: "constitution.update_parameter".into(),
                version: "current-durable-mutation-unqualified-delivery".into(),
                effect_semantics: ProviderEffectSemantics::PersistsDurableStateMutation,
                delivery: DeliverySemantics::Unproven,
                reconciliation: ReconciliationCapability::Unproven,
                observability: OutcomeObservability::Unproven,
                reversibility: Reversibility::Unknown,
                batch_atomicity: BatchAtomicity::Unproven,
                evidence_ref: Some(
                    "current-code:constitution-update-parameter-persists-governance-parameter".into(),
                ),
            },
            requires_durable_success: true,
        },
        ExistingGovernanceActionKind::EmitEvent => ActionEffectProfile {
            action_id,
            effect_class: EffectClass::BestEffortNotification,
            provider: ProviderCapabilityProfile {
                provider_id: "hdk.emit_signal".into(),
                version: "best-effort-current".into(),
                effect_semantics: ProviderEffectSemantics::EmitsBestEffortNotification,
                delivery: DeliverySemantics::Unproven,
                reconciliation: ReconciliationCapability::None,
                observability: OutcomeObservability::None,
                reversibility: Reversibility::Unknown,
                batch_atomicity: BatchAtomicity::None,
                evidence_ref: Some("current-code:emit_signal-result-ignored".into()),
            },
            requires_durable_success: false,
        },
    }
}

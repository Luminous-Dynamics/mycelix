#![forbid(unsafe_code)]

//! Synthetic reference semantics for IPB-FINALITY-001.
//!
//! This crate consumes already-resolved evidence references. It does not
//! authenticate them, establish legal authority, mint capabilities, hash
//! domain payloads, invoke providers, or cause external effects.
//!
//! Its narrow purpose is to check correspondence between an approved
//! operation and a pending operation, compare supplied current-state evidence,
//! and preserve unknown/partial effect outcomes. Production owners remain
//! authoritative for every input fact.

use std::collections::BTreeSet;

/// Domain-provided exact operation projection.
///
/// Equality compares every field. The adapter must supply a canonical digest
/// derived from the owner's exact operation semantics; this crate does not
/// canonicalize arbitrary application payloads or verify the digest itself.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OperationBinding {
    pub logical_operation_id: String,
    pub canonical_operation_digest: String,
    pub action_digest: String,
    pub target_ref: String,
    pub destination_or_audience: Option<String>,
    pub amount_minor: Option<u64>,
    pub currency: Option<String>,
    pub purpose: Option<String>,
    pub resource_revision: Option<String>,
    pub idempotency_ref: Option<String>,
}

/// Exact constitutional identity; a friendly name or epoch number alone
/// is not sufficient.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstitutionRef {
    pub institution_ref: String,
    pub epoch: u64,
    pub constitution_digest: String,
}

/// Currentness of the historical approval evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApprovalState {
    Authorized,
    Denied,
    Revoked,
    Historical,
    Pending,
    Unknown,
}

/// Current authority result supplied by the authority owner.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuthorityState {
    Current,
    Revoked,
    Expired,
    NotYetValid,
    Unknown,
}

/// Resource precondition result supplied by the resource/provider owner.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResourceState {
    Matches,
    Mismatch,
    Unknown,
    NotRequired,
}

/// Provider/effect state after any previous attempt for this logical operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PriorEffectState {
    NeverAttempted,
    OutcomeUnknownAfterDispatch,
    ConfirmedNotApplied,
    ConfirmedApplied,
    PartiallyApplied,
}

/// Evidence about whether a retry after a confirmed no-effect result is safe.
///
/// This is not a provider capability declaration invented by the evaluator.
/// The value must be backed by an exact owner/provider contract reference.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetrySafety {
    SameOperationSameIdempotencyKeyQualified,
    Unsupported,
    Unknown,
}

/// Historical approval binding.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ApprovalBinding {
    pub operation: OperationBinding,
    pub constitution: ConstitutionRef,
    pub policy_revision: String,
    pub state: ApprovalState,
}

/// Current protected state at the effectuation boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CurrentProtectedState {
    pub constitution: ConstitutionRef,
    pub policy_revision: String,
}

/// Inputs to the independent reference relation.
///
/// Every state is supplied by an owner-native adapter. This function does not
/// resolve identity, authority, policy, resource ownership or provider truth.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FinalityInput {
    pub approval: ApprovalBinding,
    pub actual_operation: OperationBinding,
    pub current: CurrentProtectedState,
    pub authority: AuthorityState,
    pub resource: ResourceState,
    pub prior_effect: PriorEffectState,
    pub retry_safety: RetrySafety,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DenialCode {
    ApprovalDenied,
    ApprovalRevoked,
    HistoricalApproval,
    ConstitutionMismatch,
    PolicyRevisionMismatch,
    OperationMismatch,
    AuthorityRevoked,
    AuthorityExpired,
    AuthorityNotYetValid,
    ResourcePreconditionMismatch,
    EffectAlreadyApplied,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HoldCode {
    MissingRequiredBinding,
    ApprovalPending,
    ApprovalUnknown,
    AuthorityUnknown,
    ResourceStateUnknown,
    PriorOutcomeUnknown,
    PartialOutcomeRequiresReconciliation,
    RetrySafetyUnproven,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FinalityDecision {
    Permit,
    Deny(DenialCode),
    Hold(HoldCode),
}

fn operation_binding_complete(operation: &OperationBinding) -> bool {
    !operation.logical_operation_id.trim().is_empty()
        && !operation.canonical_operation_digest.trim().is_empty()
        && !operation.action_digest.trim().is_empty()
        && !operation.target_ref.trim().is_empty()
        && operation.amount_minor.is_none() == operation.currency.is_none()
}

fn constitution_ref_complete(constitution: &ConstitutionRef) -> bool {
    !constitution.institution_ref.trim().is_empty()
        && !constitution.constitution_digest.trim().is_empty()
}

/// Evaluate the benchmark's narrow pre-effect correspondence relation.
///
/// Order is deliberate: a missing/invalid authorization or a changed
/// operation/constitution is denied before considering any provider retry.
/// Unknown evidence is held; it is never coerced into Permit.
pub fn evaluate_finality(input: &FinalityInput) -> FinalityDecision {
    if !operation_binding_complete(&input.approval.operation)
        || !operation_binding_complete(&input.actual_operation)
        || !constitution_ref_complete(&input.approval.constitution)
        || !constitution_ref_complete(&input.current.constitution)
        || input.approval.policy_revision.trim().is_empty()
        || input.current.policy_revision.trim().is_empty()
    {
        return FinalityDecision::Hold(HoldCode::MissingRequiredBinding);
    }

    match input.approval.state {
        ApprovalState::Authorized => {}
        ApprovalState::Denied => return FinalityDecision::Deny(DenialCode::ApprovalDenied),
        ApprovalState::Revoked => return FinalityDecision::Deny(DenialCode::ApprovalRevoked),
        ApprovalState::Historical => {
            return FinalityDecision::Deny(DenialCode::HistoricalApproval);
        }
        ApprovalState::Pending => return FinalityDecision::Hold(HoldCode::ApprovalPending),
        ApprovalState::Unknown => return FinalityDecision::Hold(HoldCode::ApprovalUnknown),
    }

    if input.approval.operation != input.actual_operation {
        return FinalityDecision::Deny(DenialCode::OperationMismatch);
    }

    if input.approval.constitution != input.current.constitution {
        return FinalityDecision::Deny(DenialCode::ConstitutionMismatch);
    }

    if input.approval.policy_revision != input.current.policy_revision {
        return FinalityDecision::Deny(DenialCode::PolicyRevisionMismatch);
    }

    match input.authority {
        AuthorityState::Current => {}
        AuthorityState::Revoked => {
            return FinalityDecision::Deny(DenialCode::AuthorityRevoked);
        }
        AuthorityState::Expired => {
            return FinalityDecision::Deny(DenialCode::AuthorityExpired);
        }
        AuthorityState::NotYetValid => {
            return FinalityDecision::Deny(DenialCode::AuthorityNotYetValid);
        }
        AuthorityState::Unknown => {
            return FinalityDecision::Hold(HoldCode::AuthorityUnknown);
        }
    }

    match input.resource {
        ResourceState::Matches | ResourceState::NotRequired => {}
        ResourceState::Mismatch => {
            return FinalityDecision::Deny(DenialCode::ResourcePreconditionMismatch);
        }
        ResourceState::Unknown => {
            return FinalityDecision::Hold(HoldCode::ResourceStateUnknown);
        }
    }

    match input.prior_effect {
        PriorEffectState::NeverAttempted => FinalityDecision::Permit,
        PriorEffectState::OutcomeUnknownAfterDispatch => {
            FinalityDecision::Hold(HoldCode::PriorOutcomeUnknown)
        }
        PriorEffectState::ConfirmedApplied => {
            FinalityDecision::Deny(DenialCode::EffectAlreadyApplied)
        }
        PriorEffectState::PartiallyApplied => {
            FinalityDecision::Hold(HoldCode::PartialOutcomeRequiresReconciliation)
        }
        PriorEffectState::ConfirmedNotApplied => match input.retry_safety {
            RetrySafety::SameOperationSameIdempotencyKeyQualified
                if input
                    .actual_operation
                    .idempotency_ref
                    .as_deref()
                    .is_some_and(|key| !key.trim().is_empty()) =>
            {
                FinalityDecision::Permit
            }
            RetrySafety::SameOperationSameIdempotencyKeyQualified
            | RetrySafety::Unsupported
            | RetrySafety::Unknown => FinalityDecision::Hold(HoldCode::RetrySafetyUnproven),
        },
    }
}

/// Per-action outcome evidence. NotAttempted is distinct from a failed
/// attempt: the latter needs owner evidence that no effect occurred.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionObservation {
    pub action_id: String,
    pub effect: ActionEffect,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionEffect {
    NotAttempted,
    ConfirmedApplied,
    ConfirmedNotApplied,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReconciledOutcome {
    NotAttempted,
    ConfirmedNoEffect,
    FullyApplied,
    PartiallyApplied,
    Unknown,
    PartialWithUnknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReconciliationError {
    DuplicateActionId,
}

/// Reconcile a multi-action result without inferring batch atomicity.
///
/// The caller supplies one observation per stable action identity. Duplicate
/// action identities are invalid; unknown suffixes and successful prefixes
/// remain visible rather than being collapsed into failure-no-effect.
pub fn reconcile_actions(
    actions: &[ActionObservation],
) -> Result<ReconciledOutcome, ReconciliationError> {
    let mut ids = BTreeSet::new();
    for action in actions {
        if !ids.insert(action.action_id.as_str()) {
            return Err(ReconciliationError::DuplicateActionId);
        }
    }

    if actions.is_empty()
        || actions
            .iter()
            .all(|action| action.effect == ActionEffect::NotAttempted)
    {
        return Ok(ReconciledOutcome::NotAttempted);
    }

    let has_applied = actions
        .iter()
        .any(|action| action.effect == ActionEffect::ConfirmedApplied);
    let has_unknown = actions
        .iter()
        .any(|action| action.effect == ActionEffect::Unknown);
    let has_no_effect = actions.iter().any(|action| {
        matches!(
            action.effect,
            ActionEffect::NotAttempted | ActionEffect::ConfirmedNotApplied
        )
    });

    if has_unknown && has_applied {
        return Ok(ReconciledOutcome::PartialWithUnknown);
    }
    if has_unknown {
        return Ok(ReconciledOutcome::Unknown);
    }
    if has_applied && has_no_effect {
        return Ok(ReconciledOutcome::PartiallyApplied);
    }
    if has_applied {
        return Ok(ReconciledOutcome::FullyApplied);
    }

    Ok(ReconciledOutcome::ConfirmedNoEffect)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn operation() -> OperationBinding {
        OperationBinding {
            logical_operation_id: "op-001".to_owned(),
            canonical_operation_digest: "fixture:operation-digest-001".to_owned(),
            action_digest: "fixture:action-digest-transfer".to_owned(),
            target_ref: "asset:treasury-a".to_owned(),
            destination_or_audience: Some("party-b".to_owned()),
            amount_minor: Some(10_000),
            currency: Some("USD".to_owned()),
            purpose: Some("approved-grant".to_owned()),
            resource_revision: Some("revision-7".to_owned()),
            idempotency_ref: Some("idem-001".to_owned()),
        }
    }

    fn constitution() -> ConstitutionRef {
        ConstitutionRef {
            institution_ref: "institution-a".to_owned(),
            epoch: 7,
            constitution_digest: "fixture:constitution-digest-7".to_owned(),
        }
    }

    fn input() -> FinalityInput {
        FinalityInput {
            approval: ApprovalBinding {
                operation: operation(),
                constitution: constitution(),
                policy_revision: "policy-r4".to_owned(),
                state: ApprovalState::Authorized,
            },
            actual_operation: operation(),
            current: CurrentProtectedState {
                constitution: constitution(),
                policy_revision: "policy-r4".to_owned(),
            },
            authority: AuthorityState::Current,
            resource: ResourceState::Matches,
            prior_effect: PriorEffectState::NeverAttempted,
            retry_safety: RetrySafety::Unknown,
        }
    }

    #[test]
    fn exact_current_operation_is_permitted() {
        assert_eq!(evaluate_finality(&input()), FinalityDecision::Permit);
    }

    #[test]
    fn one_field_operation_substitution_is_denied() {
        let mut candidate = input();
        candidate.actual_operation.destination_or_audience = Some("party-evil".to_owned());
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::OperationMismatch)
        );
    }

    #[test]
    fn changed_amount_is_denied_even_when_operation_label_is_reused() {
        let mut candidate = input();
        candidate.actual_operation.amount_minor = Some(100_000);
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::OperationMismatch)
        );
    }

    #[test]
    fn changed_target_is_denied() {
        let mut candidate = input();
        candidate.actual_operation.target_ref = "asset:other-treasury".to_owned();
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::OperationMismatch)
        );
    }

    #[test]
    fn same_institution_and_epoch_number_but_new_digest_is_denied() {
        let mut candidate = input();
        candidate.current.constitution.constitution_digest =
            "fixture:constitution-digest-substituted".to_owned();
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::ConstitutionMismatch)
        );
    }

    #[test]
    fn constitution_epoch_change_after_ready_is_denied() {
        let mut candidate = input();
        candidate.current.constitution.epoch = 8;
        candidate.current.constitution.constitution_digest =
            "fixture:constitution-digest-8".to_owned();
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::ConstitutionMismatch)
        );
    }

    #[test]
    fn policy_revision_change_is_denied() {
        let mut candidate = input();
        candidate.current.policy_revision = "policy-r5".to_owned();
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::PolicyRevisionMismatch)
        );
    }

    #[test]
    fn historical_approval_is_not_current_authority() {
        let mut candidate = input();
        candidate.approval.state = ApprovalState::Historical;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::HistoricalApproval)
        );
    }

    #[test]
    fn revoked_authority_is_denied() {
        let mut candidate = input();
        candidate.authority = AuthorityState::Revoked;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::AuthorityRevoked)
        );
    }

    #[test]
    fn unknown_authority_is_held_not_permitted() {
        let mut candidate = input();
        candidate.authority = AuthorityState::Unknown;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::AuthorityUnknown)
        );
    }

    #[test]
    fn changed_resource_precondition_is_denied() {
        let mut candidate = input();
        candidate.resource = ResourceState::Mismatch;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::ResourcePreconditionMismatch)
        );
    }

    #[test]
    fn unknown_resource_precondition_is_held() {
        let mut candidate = input();
        candidate.resource = ResourceState::Unknown;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::ResourceStateUnknown)
        );
    }

    #[test]
    fn unknown_provider_outcome_is_held_and_not_retried() {
        let mut candidate = input();
        candidate.prior_effect = PriorEffectState::OutcomeUnknownAfterDispatch;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::PriorOutcomeUnknown)
        );
    }

    #[test]
    fn confirmed_effect_cannot_be_replayed() {
        let mut candidate = input();
        candidate.prior_effect = PriorEffectState::ConfirmedApplied;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Deny(DenialCode::EffectAlreadyApplied)
        );
    }

    #[test]
    fn incomplete_operation_binding_is_held() {
        let mut candidate = input();
        candidate.actual_operation.canonical_operation_digest.clear();
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::MissingRequiredBinding)
        );
    }

    #[test]
    fn amount_without_currency_is_incomplete_binding() {
        let mut candidate = input();
        candidate.actual_operation.currency = None;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::MissingRequiredBinding)
        );
    }

    #[test]
    fn confirmed_no_effect_can_retry_only_with_qualified_same_key_contract() {
        let mut candidate = input();
        candidate.prior_effect = PriorEffectState::ConfirmedNotApplied;
        candidate.retry_safety = RetrySafety::SameOperationSameIdempotencyKeyQualified;
        assert_eq!(evaluate_finality(&candidate), FinalityDecision::Permit);

        candidate.retry_safety = RetrySafety::Unsupported;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::RetrySafetyUnproven)
        );

        candidate.retry_safety =
            RetrySafety::SameOperationSameIdempotencyKeyQualified;
        candidate.actual_operation.idempotency_ref = None;
        candidate.approval.operation.idempotency_ref = None;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::RetrySafetyUnproven)
        );
    }

    #[test]
    fn partial_prior_effect_is_held_for_reconciliation() {
        let mut candidate = input();
        candidate.prior_effect = PriorEffectState::PartiallyApplied;
        assert_eq!(
            evaluate_finality(&candidate),
            FinalityDecision::Hold(HoldCode::PartialOutcomeRequiresReconciliation)
        );
    }

    #[test]
    fn reconciliation_preserves_a_successful_prefix_and_unknown_suffix() {
        let actions = vec![
            ActionObservation {
                action_id: "a".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            },
            ActionObservation {
                action_id: "b".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            },
            ActionObservation {
                action_id: "c".to_owned(),
                effect: ActionEffect::Unknown,
            },
        ];
        assert_eq!(
            reconcile_actions(&actions),
            Ok(ReconciledOutcome::PartialWithUnknown)
        );
    }

    #[test]
    fn reconciliation_distinguishes_all_applied_from_partial() {
        let all_applied = vec![
            ActionObservation {
                action_id: "a".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            },
            ActionObservation {
                action_id: "b".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            },
        ];
        assert_eq!(
            reconcile_actions(&all_applied),
            Ok(ReconciledOutcome::FullyApplied)
        );

        let partial = vec![
            ActionObservation {
                action_id: "a".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            },
            ActionObservation {
                action_id: "b".to_owned(),
                effect: ActionEffect::ConfirmedNotApplied,
            },
        ];
        assert_eq!(
            reconcile_actions(&partial),
            Ok(ReconciledOutcome::PartiallyApplied)
        );
    }

    #[test]
    fn duplicate_action_identity_is_rejected() {
        let duplicate = vec![
            ActionObservation {
                action_id: "same".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            },
            ActionObservation {
                action_id: "same".to_owned(),
                effect: ActionEffect::Unknown,
            },
        ];
        assert_eq!(
            reconcile_actions(&duplicate),
            Err(ReconciliationError::DuplicateActionId)
        );
    }
}

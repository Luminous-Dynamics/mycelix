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
/// Source class for an outcome assertion. This is separate from the
/// asserted effect state: a provider acknowledgement can say "applied" without
/// being an independent observation of the external effect.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EffectEvidenceKind {
    ProviderAcknowledgement,
    ProviderStatusQuery,
    IndependentObservation,
    OwnerReconciliation,
}

/// Relationship to the candidate/effect under evaluation, supplied by the
/// independent dependency/evaluator owner. This crate does not infer it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceIndependence {
    QualifiedIndependent,
    KnownSharedDependency,
    Unknown,
    NotAssessed,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectEvidence {
    pub source_ref: String,
    pub kind: EffectEvidenceKind,
    pub independence: EvidenceIndependence,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObservationQualification {
    QualifiedIndependentObservation,
    ProviderReportedOnly,
    KnownSharedDependency,
    Indeterminate,
    NoAppliedEffects,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionObservation {
    pub action_id: String,
    pub effect: ActionEffect,
    pub evidence: Option<EffectEvidence>,
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
    DuplicateExpectedActionId,
    DuplicateObservedActionId,
    UnexpectedObservedActionId,
}

/// Reconcile a multi-action result without inferring batch atomicity.
///
/// The expected action sequence is supplied separately from observations.
/// Missing observations for an expected action become Unknown; they can never
/// make a successful observed prefix look fully applied. Duplicate expected
/// or observed identities and observations for unknown action IDs are invalid.
pub fn reconcile_actions(
    expected_action_ids: &[String],
    observations: &[ActionObservation],
) -> Result<ReconciledOutcome, ReconciliationError> {
    let mut expected = BTreeSet::new();
    for action_id in expected_action_ids {
        if !expected.insert(action_id.as_str()) {
            return Err(ReconciliationError::DuplicateExpectedActionId);
        }
    }

    let mut observed = std::collections::BTreeMap::new();
    for action in observations {
        if !expected.contains(action.action_id.as_str()) {
            return Err(ReconciliationError::UnexpectedObservedActionId);
        }
        if observed.insert(action.action_id.as_str(), action.effect).is_some() {
            return Err(ReconciliationError::DuplicateObservedActionId);
        }
    }

    if expected_action_ids.is_empty() {
        return Ok(ReconciledOutcome::NotAttempted);
    }

    let mut has_applied = false;
    let mut has_unknown = false;
    let mut has_no_effect = false;
    let mut all_not_attempted = true;

    for action_id in expected_action_ids {
        match observed.get(action_id.as_str()).copied() {
            Some(ActionEffect::NotAttempted) => {
                has_no_effect = true;
            }
            Some(ActionEffect::ConfirmedApplied) => {
                has_applied = true;
                all_not_attempted = false;
            }
            Some(ActionEffect::ConfirmedNotApplied) => {
                has_no_effect = true;
                all_not_attempted = false;
            }
            Some(ActionEffect::Unknown) | None => {
                has_unknown = true;
                all_not_attempted = false;
            }
        }
    }

    if all_not_attempted {
        return Ok(ReconciledOutcome::NotAttempted);
    }
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

/// Qualify whether every observed applied effect in the expected action
/// sequence has source evidence that the independent evaluator has already
/// classified as independent.
///
/// This function does not compute independence or trust. It consumes the
/// source/dependency classification supplied by the independent owner.
/// Provider acknowledgements and provider status queries remain provider
/// reports even when signed; they are not independent physical/external
/// observation. Missing expected observations remain indeterminate.
pub fn qualify_independent_effect_observation(
    expected_action_ids: &[String],
    observations: &[ActionObservation],
) -> Result<ObservationQualification, ReconciliationError> {
    let mut expected = BTreeSet::new();
    for action_id in expected_action_ids {
        if !expected.insert(action_id.as_str()) {
            return Err(ReconciliationError::DuplicateExpectedActionId);
        }
    }

    let mut observed = std::collections::BTreeMap::new();
    for action in observations {
        if !expected.contains(action.action_id.as_str()) {
            return Err(ReconciliationError::UnexpectedObservedActionId);
        }
        if observed.insert(action.action_id.as_str(), action).is_some() {
            return Err(ReconciliationError::DuplicateObservedActionId);
        }
    }

    let mut has_applied = false;
    for action_id in expected_action_ids {
        let Some(action) = observed.get(action_id.as_str()).copied() else {
            return Ok(ObservationQualification::Indeterminate);
        };

        if action.effect != ActionEffect::ConfirmedApplied {
            continue;
        }
        has_applied = true;

        let Some(evidence) = action.evidence.as_ref() else {
            return Ok(ObservationQualification::Indeterminate);
        };
        if evidence.source_ref.trim().is_empty() {
            return Ok(ObservationQualification::Indeterminate);
        }

        match evidence.kind {
            EffectEvidenceKind::ProviderAcknowledgement
            | EffectEvidenceKind::ProviderStatusQuery => {
                return Ok(ObservationQualification::ProviderReportedOnly);
            }
            EffectEvidenceKind::IndependentObservation | EffectEvidenceKind::OwnerReconciliation => {
                match evidence.independence {
                    EvidenceIndependence::QualifiedIndependent => {}
                    EvidenceIndependence::KnownSharedDependency => {
                        return Ok(ObservationQualification::KnownSharedDependency);
                    }
                    EvidenceIndependence::Unknown | EvidenceIndependence::NotAssessed => {
                        return Ok(ObservationQualification::Indeterminate);
                    }
                }
            }
        }
    }

    if has_applied {
        Ok(ObservationQualification::QualifiedIndependentObservation)
    } else {
        Ok(ObservationQualification::NoAppliedEffects)
    }
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
    fn provider_acknowledgement_is_not_independent_effect_observation() {
        let observations = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: Some(EffectEvidence {
                source_ref: "provider:ack-1".to_owned(),
                kind: EffectEvidenceKind::ProviderAcknowledgement,
                independence: EvidenceIndependence::Unknown,
            }),
        }];
        assert_eq!(
            qualify_independent_effect_observation(&["a".to_owned()], &observations),
            Ok(ObservationQualification::ProviderReportedOnly)
        );
    }

    #[test]
    fn provider_status_query_is_not_independent_observation() {
        let observations = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: Some(EffectEvidence {
                source_ref: "provider:status-1".to_owned(),
                kind: EffectEvidenceKind::ProviderStatusQuery,
                independence: EvidenceIndependence::QualifiedIndependent,
            }),
        }];
        assert_eq!(
            qualify_independent_effect_observation(&["a".to_owned()], &observations),
            Ok(ObservationQualification::ProviderReportedOnly)
        );
    }

    #[test]
    fn independent_observation_requires_owner_qualified_independence() {
        let observations = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: Some(EffectEvidence {
                source_ref: "observer:receipt-1".to_owned(),
                kind: EffectEvidenceKind::IndependentObservation,
                independence: EvidenceIndependence::QualifiedIndependent,
            }),
        }];
        assert_eq!(
            qualify_independent_effect_observation(&["a".to_owned()], &observations),
            Ok(ObservationQualification::QualifiedIndependentObservation)
        );
    }

    #[test]
    fn known_shared_dependency_prevents_independent_observation_claim() {
        let observations = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: Some(EffectEvidence {
                source_ref: "observer:receipt-2".to_owned(),
                kind: EffectEvidenceKind::IndependentObservation,
                independence: EvidenceIndependence::KnownSharedDependency,
            }),
        }];
        assert_eq!(
            qualify_independent_effect_observation(&["a".to_owned()], &observations),
            Ok(ObservationQualification::KnownSharedDependency)
        );
    }

    #[test]
    fn missing_or_unqualified_observation_remains_indeterminate() {
        let missing = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: None,
        }];
        assert_eq!(
            qualify_independent_effect_observation(&["a".to_owned()], &missing),
            Ok(ObservationQualification::Indeterminate)
        );

        let unqualified = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: Some(EffectEvidence {
                source_ref: "observer:receipt-3".to_owned(),
                kind: EffectEvidenceKind::IndependentObservation,
                independence: EvidenceIndependence::Unknown,
            }),
        }];
        assert_eq!(
            qualify_independent_effect_observation(&["a".to_owned()], &unqualified),
            Ok(ObservationQualification::Indeterminate)
        );
    }

    #[test]
    fn missing_expected_action_observation_is_not_complete() {
        let observations = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: Some(EffectEvidence {
                source_ref: "observer:receipt-a".to_owned(),
                kind: EffectEvidenceKind::IndependentObservation,
                independence: EvidenceIndependence::QualifiedIndependent,
            }),
        }];
        assert_eq!(
            qualify_independent_effect_observation(
                &["a".to_owned(), "b".to_owned()],
                &observations
            ),
            Ok(ObservationQualification::Indeterminate)
        );
    }

    #[test]
    fn no_applied_effects_does_not_claim_independent_observation() {
        let observations = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedNotApplied,
            evidence: None,
        }];
        assert_eq!(
            qualify_independent_effect_observation(&["a".to_owned()], &observations),
            Ok(ObservationQualification::NoAppliedEffects)
        );
    }

    #[test]
    fn reconciliation_preserves_a_successful_prefix_and_unknown_suffix() {
        let actions = vec![
            ActionObservation {
                action_id: "a".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            evidence: None,
            },
            ActionObservation {
                action_id: "b".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            evidence: None,
            },
            ActionObservation {
                action_id: "c".to_owned(),
                effect: ActionEffect::Unknown,
            evidence: None,
            },
        ];
        assert_eq!(
            reconcile_actions(
                &["a".to_owned(), "b".to_owned(), "c".to_owned()],
                &actions
            ),
            Ok(ReconciledOutcome::PartialWithUnknown)
        );
    }

    #[test]
    fn reconciliation_distinguishes_all_applied_from_partial() {
        let all_applied = vec![
            ActionObservation {
                action_id: "a".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            evidence: None,
            },
            ActionObservation {
                action_id: "b".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            evidence: None,
            },
        ];
        assert_eq!(
            reconcile_actions(&["a".to_owned(), "b".to_owned()], &all_applied),
            Ok(ReconciledOutcome::FullyApplied)
        );

        let partial = vec![
            ActionObservation {
                action_id: "a".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            evidence: None,
            },
            ActionObservation {
                action_id: "b".to_owned(),
                effect: ActionEffect::ConfirmedNotApplied,
            evidence: None,
            },
        ];
        assert_eq!(
            reconcile_actions(&["a".to_owned(), "b".to_owned()], &partial),
            Ok(ReconciledOutcome::PartiallyApplied)
        );
    }

    #[test]
    fn missing_action_observation_prevents_full_completion() {
        let observed_prefix = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
        evidence: None,
        }];
        assert_eq!(
            reconcile_actions(
                &["a".to_owned(), "b".to_owned()],
                &observed_prefix
            ),
            Ok(ReconciledOutcome::PartialWithUnknown)
        );
    }

    #[test]
    fn observation_for_unexpected_action_is_rejected() {
        let unexpected = vec![ActionObservation {
            action_id: "not-in-plan".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
        evidence: None,
        }];
        assert_eq!(
            reconcile_actions(&["expected".to_owned()], &unexpected),
            Err(ReconciliationError::UnexpectedObservedActionId)
        );
    }

    #[test]
    fn duplicate_expected_action_identity_is_rejected() {
        assert_eq!(
            reconcile_actions(&["same".to_owned(), "same".to_owned()], &[]),
            Err(ReconciliationError::DuplicateExpectedActionId)
        );
    }

    #[test]
    fn duplicate_action_identity_is_rejected() {
        let duplicate = vec![
            ActionObservation {
                action_id: "same".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
            evidence: None,
            },
            ActionObservation {
                action_id: "same".to_owned(),
                effect: ActionEffect::Unknown,
            evidence: None,
            },
        ];
        assert_eq!(
            reconcile_actions(&["same".to_owned()], &duplicate),
            Err(ReconciliationError::DuplicateObservedActionId)
        );
    }
}

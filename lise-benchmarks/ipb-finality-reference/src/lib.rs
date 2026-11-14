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
    let mut has_provider_only = false;
    let mut has_known_shared_dependency = false;
    let mut has_indeterminate = false;

    for action_id in expected_action_ids {
        let Some(action) = observed.get(action_id.as_str()).copied() else {
            has_indeterminate = true;
            continue;
        };

        match action.effect {
            ActionEffect::NotAttempted | ActionEffect::ConfirmedNotApplied => continue,
            ActionEffect::Unknown => {
                has_indeterminate = true;
                continue;
            }
            ActionEffect::ConfirmedApplied => has_applied = true,
        }

        let Some(evidence) = action.evidence.as_ref() else {
            has_indeterminate = true;
            continue;
        };
        if evidence.source_ref.trim().is_empty() {
            has_indeterminate = true;
            continue;
        }

        match evidence.kind {
            EffectEvidenceKind::ProviderAcknowledgement
            | EffectEvidenceKind::ProviderStatusQuery => {
                has_provider_only = true;
            }
            EffectEvidenceKind::IndependentObservation | EffectEvidenceKind::OwnerReconciliation => {
                match evidence.independence {
                    EvidenceIndependence::QualifiedIndependent => {}
                    EvidenceIndependence::KnownSharedDependency => {
                        has_known_shared_dependency = true;
                    }
                    EvidenceIndependence::Unknown | EvidenceIndependence::NotAssessed => {
                        has_indeterminate = true;
                    }
                }
            }
        }
    }

    // Aggregate before returning so reordering the expected action list cannot
    // change the qualification result.
    if has_known_shared_dependency {
        Ok(ObservationQualification::KnownSharedDependency)
    } else if has_indeterminate {
        Ok(ObservationQualification::Indeterminate)
    } else if has_provider_only {
        Ok(ObservationQualification::ProviderReportedOnly)
    } else if has_applied {
        Ok(ObservationQualification::QualifiedIndependentObservation)
    } else {
        Ok(ObservationQualification::NoAppliedEffects)
    }
}


/// A bounded delegation scope supplied by the authority owner.
///
/// Empty sets mean no authority for that dimension. `None` for an upper
/// bound means unbounded; a child may not turn a bounded parent into an
/// unbounded grant. This is a comparison model, not a credential parser.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityScope {
    pub actions: BTreeSet<String>,
    pub resources: BTreeSet<String>,
    pub purposes: BTreeSet<String>,
    pub fields: BTreeSet<String>,
    pub valid_from: u64,
    pub expires_at: Option<u64>,
    pub max_amount_minor: Option<u64>,
    pub max_operations: Option<u64>,
}

/// One owner-resolved grant that is available during the evaluated trajectory.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityGrant {
    pub grant_ref: String,
    pub active_during_trajectory: bool,
    pub scope: CapabilityScope,
}

/// Union of privileges available through any active grant/session in a
/// trajectory slice. This deliberately measures available privilege, not
/// only the permission used by the final successful tool call.
#[derive(Clone, Debug, Eq, PartialEq, Default)]
pub struct EffectivePrivilege {
    pub grant_refs: BTreeSet<String>,
    pub actions: BTreeSet<String>,
    pub resources: BTreeSet<String>,
    pub purposes: BTreeSet<String>,
    pub fields: BTreeSet<String>,
    /// Conservative sum of active grant amount ceilings; None means at least
    /// one active grant is unbounded.
    pub aggregate_amount_limit_minor: Option<u64>,
    /// Conservative sum of active grant operation-count ceilings; None means
    /// at least one active grant is unbounded.
    pub aggregate_operation_limit: Option<u64>,
}

fn bounded_child_attenuates(parent: Option<u64>, child: Option<u64>) -> bool {
    match (parent, child) {
        (None, _) => true,
        (Some(_), None) => false,
        (Some(parent_limit), Some(child_limit)) => child_limit <= parent_limit,
    }
}

/// Returns true only when every child dimension is a subset of its parent
/// and every child temporal/quantity limit is at least as restrictive.
pub fn delegation_is_attenuated(parent: &CapabilityScope, child: &CapabilityScope) -> bool {
    child.actions.is_subset(&parent.actions)
        && child.resources.is_subset(&parent.resources)
        && child.purposes.is_subset(&parent.purposes)
        && child.fields.is_subset(&parent.fields)
        && child.valid_from >= parent.valid_from
        && match (parent.expires_at, child.expires_at) {
            (None, _) => true,
            (Some(_), None) => false,
            (Some(parent_expiry), Some(child_expiry)) => child_expiry <= parent_expiry,
        }
        && bounded_child_attenuates(parent.max_amount_minor, child.max_amount_minor)
        && bounded_child_attenuates(parent.max_operations, child.max_operations)
}

/// Computes the union of all owner-resolved grants available to an agent
/// during the supplied trajectory. The caller must include tool/session
/// grants, forwarded credentials and ambient credentials in the input set.
pub fn union_effective_privilege(grants: &[CapabilityGrant]) -> EffectivePrivilege {
    let mut result = EffectivePrivilege {
        aggregate_amount_limit_minor: Some(0),
        aggregate_operation_limit: Some(0),
        ..EffectivePrivilege::default()
    };
    for grant in grants.iter().filter(|grant| grant.active_during_trajectory) {
        result.grant_refs.insert(grant.grant_ref.clone());
        result.actions.extend(grant.scope.actions.iter().cloned());
        result.resources.extend(grant.scope.resources.iter().cloned());
        result.purposes.extend(grant.scope.purposes.iter().cloned());
        result.fields.extend(grant.scope.fields.iter().cloned());
        result.aggregate_amount_limit_minor = match (result.aggregate_amount_limit_minor, grant.scope.max_amount_minor) {
            (Some(total), Some(limit)) => Some(total.saturating_add(limit)),
            _ => None,
        };
        result.aggregate_operation_limit = match (result.aggregate_operation_limit, grant.scope.max_operations) {
            (Some(total), Some(limit)) => Some(total.saturating_add(limit)),
            _ => None,
        };
    }
    result
}

/// Checks whether the trajectory-wide privilege union exceeds the approved
/// task scope. It is possible for a final call to be harmless while the
/// available credential envelope is materially broader than the task.
fn limit_within_task(effective: Option<u64>, task: Option<u64>) -> bool {
    match (effective, task) {
        (_, None) => true,
        (Some(actual), Some(approved)) => actual <= approved,
        (None, Some(_)) => false,
    }
}
pub fn effective_privilege_within_task(
    effective: &EffectivePrivilege,
    task: &CapabilityScope,
) -> bool {
    effective.actions.is_subset(&task.actions)
        && effective.resources.is_subset(&task.resources)
        && effective.purposes.is_subset(&task.purposes)
        && effective.fields.is_subset(&task.fields)
        && limit_within_task(effective.aggregate_amount_limit_minor, task.max_amount_minor)
        && limit_within_task(effective.aggregate_operation_limit, task.max_operations)
}

/// One executable route to a protected resource/effect.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProtectedRoute {
    pub route_ref: String,
    pub protected_effect_ref: String,
    pub reachable_from_agent: bool,
    pub enforcement_point_ref: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MediationCoverage {
    AllReachablePathsMediated,
    ReachableBypass,
    RouteInventoryIncomplete,
    NoReachablePath,
}

/// Evaluates complete-mediation evidence for one protected effect.
///
/// A single known reachable bypass is decisive even when the route inventory
/// is incomplete. A fully mediated known subset is not a complete-mediation
/// claim unless the owner says the route inventory is complete.
pub fn assess_mediation_coverage(
    protected_effect_ref: &str,
    expected_enforcement_point_ref: &str,
    route_inventory_complete: bool,
    routes: &[ProtectedRoute],
) -> MediationCoverage {
    if expected_enforcement_point_ref.trim().is_empty() {
        return MediationCoverage::ReachableBypass;
    }
    let reachable: Vec<&ProtectedRoute> = routes
        .iter()
        .filter(|route| {
            route.protected_effect_ref == protected_effect_ref && route.reachable_from_agent
        })
        .collect();
    if reachable
        .iter()
        .any(|route| route.enforcement_point_ref.as_deref() != Some(expected_enforcement_point_ref))
    {
        return MediationCoverage::ReachableBypass;
    }
    if !route_inventory_complete {
        return MediationCoverage::RouteInventoryIncomplete;
    }
    if reachable.is_empty() {
        return MediationCoverage::NoReachablePath;
    }
    MediationCoverage::AllReachablePathsMediated
}

/// Currentness of an active session against the authority owner's revocation
/// epoch and bitemporal revocation record.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SessionRevocationState {
    pub authority_state: AuthorityState,
    pub authoritative_epoch: u64,
    pub session_observed_epoch: u64,
    pub action_at: u64,
    pub known_at: u64,
    pub revocation_effective_at: Option<u64>,
    pub revocation_observed_at: Option<u64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SessionAuthorizationDecision {
    Permit,
    DenyRevoked,
    DenyInactive,
    HoldUnknown,
    HoldStaleSession,
}

/// Evaluates an already-issued session against current authority state.
///
/// A central revocation record alone does not prove all active sessions have
/// enforced it. A stale session epoch is held until refreshed or invalidated.
pub fn evaluate_session_revocation(
    state: SessionRevocationState,
) -> SessionAuthorizationDecision {
    if state.authority_state == AuthorityState::Unknown {
        return SessionAuthorizationDecision::HoldUnknown;
    }
    if state.authority_state == AuthorityState::Revoked {
        match (state.revocation_effective_at, state.revocation_observed_at) {
            (Some(effective), Some(observed)) if observed <= state.known_at => {
                if effective <= state.action_at {
                    return SessionAuthorizationDecision::DenyRevoked;
                }
            }
            _ => return SessionAuthorizationDecision::HoldUnknown,
        }
    }
    if state.session_observed_epoch < state.authoritative_epoch {
        return SessionAuthorizationDecision::HoldStaleSession;
    }
    match state.authority_state {
        AuthorityState::Current => SessionAuthorizationDecision::Permit,
        AuthorityState::Revoked => SessionAuthorizationDecision::Permit,
        AuthorityState::Expired | AuthorityState::NotYetValid => {
            SessionAuthorizationDecision::DenyInactive
        }
        AuthorityState::Unknown => SessionAuthorizationDecision::HoldUnknown,
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

    fn scope(
        actions: &[&str],
        resources: &[&str],
        purposes: &[&str],
        fields: &[&str],
        valid_from: u64,
        expires_at: Option<u64>,
        max_amount_minor: Option<u64>,
        max_operations: Option<u64>,
    ) -> CapabilityScope {
        CapabilityScope {
            actions: actions.iter().map(|x| (*x).to_owned()).collect(),
            resources: resources.iter().map(|x| (*x).to_owned()).collect(),
            purposes: purposes.iter().map(|x| (*x).to_owned()).collect(),
            fields: fields.iter().map(|x| (*x).to_owned()).collect(),
            valid_from,
            expires_at,
            max_amount_minor,
            max_operations,
        }
    }

    #[test]
    fn child_delegation_may_attenuate_every_scope_dimension() {
        let parent = scope(
            &["read", "update"], &["asset-a", "asset-b"], &["maintenance"],
            &["status", "condition"], 10, Some(100), Some(5000), Some(4)
        );
        let child = scope(
            &["read"], &["asset-a"], &["maintenance"], &["status"],
            12, Some(80), Some(1000), Some(1)
        );
        assert!(delegation_is_attenuated(&parent, &child));
    }

    #[test]
    fn child_delegation_cannot_amplify_action_resource_purpose_or_field() {
        let parent = scope(
            &["read"], &["asset-a"], &["inspection"], &["status"],
            10, Some(100), Some(5000), Some(2)
        );
        for child in [
            scope(&["read", "delete"], &["asset-a"], &["inspection"], &["status"], 10, Some(90), Some(1000), Some(1)),
            scope(&["read"], &["asset-a", "asset-b"], &["inspection"], &["status"], 10, Some(90), Some(1000), Some(1)),
            scope(&["read"], &["asset-a"], &["inspection", "marketing"], &["status"], 10, Some(90), Some(1000), Some(1)),
            scope(&["read"], &["asset-a"], &["inspection"], &["status", "bank-account"], 10, Some(90), Some(1000), Some(1)),
        ] {
            assert!(!delegation_is_attenuated(&parent, &child));
        }
    }

    #[test]
    fn child_delegation_cannot_extend_expiry_or_remove_parent_limits() {
        let parent = scope(&["read"], &["asset-a"], &["inspection"], &["status"], 10, Some(100), Some(500), Some(2));
        let expiry_amplification = scope(&["read"], &["asset-a"], &["inspection"], &["status"], 10, Some(101), Some(100), Some(1));
        let unbounded_expiry = scope(&["read"], &["asset-a"], &["inspection"], &["status"], 10, None, Some(100), Some(1));
        let unbounded_amount = scope(&["read"], &["asset-a"], &["inspection"], &["status"], 10, Some(90), None, Some(1));
        let unbounded_operations = scope(&["read"], &["asset-a"], &["inspection"], &["status"], 10, Some(90), Some(100), None);
        assert!(!delegation_is_attenuated(&parent, &expiry_amplification));
        assert!(!delegation_is_attenuated(&parent, &unbounded_expiry));
        assert!(!delegation_is_attenuated(&parent, &unbounded_amount));
        assert!(!delegation_is_attenuated(&parent, &unbounded_operations));
    }

    #[test]
    fn grandchild_must_attenuate_the_immediate_parent_not_only_the_root() {
        let root = scope(&["read", "update"], &["asset-a"], &["maintenance"], &["status", "condition"], 1, Some(100), None, None);
        let child = scope(&["read"], &["asset-a"], &["maintenance"], &["status"], 2, Some(80), None, None);
        let grandchild_amplifies = scope(&["read", "update"], &["asset-a"], &["maintenance"], &["status"], 3, Some(70), None, None);
        assert!(delegation_is_attenuated(&root, &child));
        assert!(!delegation_is_attenuated(&child, &grandchild_amplifies));
    }

    #[test]
    fn effective_privilege_unions_active_tool_and_ambient_grants() {
        let requested = scope(&["read"], &["record-1"], &["case-review"], &["summary"], 1, Some(50), Some(0), Some(1));
        let task_grant = CapabilityGrant {
            grant_ref: "task-token".to_owned(),
            active_during_trajectory: true,
            scope: requested.clone(),
        };
        let ambient_grant = CapabilityGrant {
            grant_ref: "shared-service-token".to_owned(),
            active_during_trajectory: true,
            scope: scope(&["read", "delete"], &["record-1", "record-2"], &["case-review", "admin"], &["summary", "private-notes"], 1, Some(50), None, None),
        };
        let expired_grant = CapabilityGrant {
            grant_ref: "inactive".to_owned(),
            active_during_trajectory: false,
            scope: scope(&["root"], &["*"], &["*"], &["*"], 1, None, None, None),
        };
        let effective = union_effective_privilege(&[task_grant, ambient_grant, expired_grant]);
        assert!(effective.actions.contains("delete"));
        assert!(effective.resources.contains("record-2"));
        assert!(effective.fields.contains("private-notes"));
        assert_eq!(effective.aggregate_amount_limit_minor, None);
        assert_eq!(effective.aggregate_operation_limit, None);
        assert!(effective.grant_refs.contains("shared-service-token"));
        assert!(!effective.grant_refs.contains("inactive"));
        assert!(!effective_privilege_within_task(&effective, &requested));
    }

    #[test]
    fn aggregate_bounded_grants_cannot_exceed_task_amount_or_operation_ceiling() {
        let task = scope(&["transfer"], &["account-a"], &["settlement"], &["amount"], 1, Some(100), Some(1000), Some(2));
        let first = CapabilityGrant {
            grant_ref: "token-a".to_owned(),
            active_during_trajectory: true,
            scope: scope(&["transfer"], &["account-a"], &["settlement"], &["amount"], 1, Some(90), Some(600), Some(1)),
        };
        let second = CapabilityGrant {
            grant_ref: "token-b".to_owned(),
            active_during_trajectory: true,
            scope: scope(&["transfer"], &["account-a"], &["settlement"], &["amount"], 1, Some(90), Some(600), Some(1)),
        };
        let effective = union_effective_privilege(&[first, second]);
        assert_eq!(effective.aggregate_amount_limit_minor, Some(1200));
        assert_eq!(effective.aggregate_operation_limit, Some(2));
        assert!(!effective_privilege_within_task(&effective, &task));
    }

    #[test]
    fn least_privilege_passes_when_every_active_grant_is_within_task_scope() {
        let task = scope(&["read", "inspect"], &["asset-a"], &["maintenance"], &["status", "condition"], 1, Some(50), Some(10), Some(2));
        let grant = CapabilityGrant {
            grant_ref: "bounded-session".to_owned(),
            active_during_trajectory: true,
            scope: scope(&["read"], &["asset-a"], &["maintenance"], &["status"], 2, Some(40), Some(5), Some(1)),
        };
        let effective = union_effective_privilege(&[grant]);
        assert!(effective_privilege_within_task(&effective, &task));
    }

    #[test]
    fn one_reachable_alternate_route_without_a_policy_enforcer_breaks_complete_mediation() {
        let routes = vec![
            ProtectedRoute { route_ref: "normal-api".to_owned(), protected_effect_ref: "effect:transfer".to_owned(), reachable_from_agent: true, enforcement_point_ref: Some("pep:gateway".to_owned()) },
            ProtectedRoute { route_ref: "bulk-import".to_owned(), protected_effect_ref: "effect:transfer".to_owned(), reachable_from_agent: true, enforcement_point_ref: None },
        ];
        assert_eq!(
            assess_mediation_coverage("effect:transfer", "pep:gateway", true, &routes),
            MediationCoverage::ReachableBypass
        );
    }

    #[test]
    fn alternate_policy_enforcement_point_does_not_satisfy_declared_mediation_boundary() {
        let routes = vec![ProtectedRoute {
            route_ref: "alternate-api".to_owned(),
            protected_effect_ref: "effect:transfer".to_owned(),
            reachable_from_agent: true,
            enforcement_point_ref: Some("pep:legacy".to_owned()),
        }];
        assert_eq!(
            assess_mediation_coverage("effect:transfer", "pep:current", true, &routes),
            MediationCoverage::ReachableBypass
        );
    }

    #[test]
    fn complete_mediation_requires_a_complete_route_inventory() {
        let routes = vec![ProtectedRoute {
            route_ref: "normal-api".to_owned(),
            protected_effect_ref: "effect:transfer".to_owned(),
            reachable_from_agent: true,
            enforcement_point_ref: Some("pep:gateway".to_owned()),
        }];
        assert_eq!(
            assess_mediation_coverage("effect:transfer", "pep:gateway", false, &routes),
            MediationCoverage::RouteInventoryIncomplete
        );
        assert_eq!(
            assess_mediation_coverage("effect:transfer", "pep:gateway", true, &routes),
            MediationCoverage::AllReachablePathsMediated
        );
    }

    #[test]
    fn unreachable_alternate_route_does_not_break_mediation_coverage() {
        let routes = vec![
            ProtectedRoute { route_ref: "normal-api".to_owned(), protected_effect_ref: "effect:transfer".to_owned(), reachable_from_agent: true, enforcement_point_ref: Some("pep:gateway".to_owned()) },
            ProtectedRoute { route_ref: "disabled-admin-api".to_owned(), protected_effect_ref: "effect:transfer".to_owned(), reachable_from_agent: false, enforcement_point_ref: None },
        ];
        assert_eq!(
            assess_mediation_coverage("effect:transfer", "pep:gateway", true, &routes),
            MediationCoverage::AllReachablePathsMediated
        );
    }

    #[test]
    fn stale_active_session_is_held_after_authority_epoch_advances() {
        let state = SessionRevocationState {
            authority_state: AuthorityState::Current,
            authoritative_epoch: 12,
            session_observed_epoch: 11,
            action_at: 100,
            known_at: 100,
            revocation_effective_at: None,
            revocation_observed_at: None,
        };
        assert_eq!(
            evaluate_session_revocation(state),
            SessionAuthorizationDecision::HoldStaleSession
        );
    }

    #[test]
    fn revoked_credential_is_denied_after_effective_and_observed_revocation() {
        let state = SessionRevocationState {
            authority_state: AuthorityState::Revoked,
            authoritative_epoch: 12,
            session_observed_epoch: 12,
            action_at: 100,
            known_at: 100,
            revocation_effective_at: Some(90),
            revocation_observed_at: Some(95),
        };
        assert_eq!(
            evaluate_session_revocation(state),
            SessionAuthorizationDecision::DenyRevoked
        );
    }

    #[test]
    fn unknown_revocation_observation_is_held_not_assumed_active() {
        let state = SessionRevocationState {
            authority_state: AuthorityState::Revoked,
            authoritative_epoch: 12,
            session_observed_epoch: 12,
            action_at: 100,
            known_at: 100,
            revocation_effective_at: Some(90),
            revocation_observed_at: None,
        };
        assert_eq!(
            evaluate_session_revocation(state),
            SessionAuthorizationDecision::HoldUnknown
        );
    }

    #[test]
    fn future_effective_revocation_does_not_rewrite_prior_action_authority() {
        let state = SessionRevocationState {
            authority_state: AuthorityState::Revoked,
            authoritative_epoch: 12,
            session_observed_epoch: 12,
            action_at: 80,
            known_at: 100,
            revocation_effective_at: Some(90),
            revocation_observed_at: Some(95),
        };
        assert_eq!(
            evaluate_session_revocation(state),
            SessionAuthorizationDecision::Permit
        );
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
    fn observation_qualification_is_invariant_to_action_order() {
        let observations = vec![
            ActionObservation {
                action_id: "a".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
                evidence: Some(EffectEvidence {
                    source_ref: "provider:ack-a".to_owned(),
                    kind: EffectEvidenceKind::ProviderAcknowledgement,
                    independence: EvidenceIndependence::Unknown,
                }),
            },
            ActionObservation {
                action_id: "b".to_owned(),
                effect: ActionEffect::ConfirmedApplied,
                evidence: Some(EffectEvidence {
                    source_ref: "observer:receipt-b".to_owned(),
                    kind: EffectEvidenceKind::IndependentObservation,
                    independence: EvidenceIndependence::QualifiedIndependent,
                }),
            },
        ];
        let forward = qualify_independent_effect_observation(
            &["a".to_owned(), "b".to_owned()],
            &observations,
        );
        let reverse = qualify_independent_effect_observation(
            &["b".to_owned(), "a".to_owned()],
            &observations,
        );
        assert_eq!(forward, reverse);
        assert_eq!(
            forward,
            Ok(ObservationQualification::ProviderReportedOnly)
        );
    }

    #[test]
    fn incomplete_plan_dominates_provider_only_evidence() {
        let observations = vec![ActionObservation {
            action_id: "a".to_owned(),
            effect: ActionEffect::ConfirmedApplied,
            evidence: Some(EffectEvidence {
                source_ref: "provider:ack-a".to_owned(),
                kind: EffectEvidenceKind::ProviderAcknowledgement,
                independence: EvidenceIndependence::Unknown,
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

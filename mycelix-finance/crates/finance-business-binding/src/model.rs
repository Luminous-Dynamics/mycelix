use std::collections::BTreeSet;

use mycelix_business_coordination::CoordinationError;
use mycelix_business_core::{
    ActionContractRef, AuthorityLeaseError, AuthorityLeaseRef, AuthorizedIntentRef, Digest32,
    ExecutionAttemptRef, PreparedActionError, ReferenceId, ReservationId, SubjectRef,
};
use mycelix_business_decision::{
    AuthorizationBinding, AuthorizationBindingError, DecisionCapsuleRef,
    ExecutionAttemptBindingError,
};
use mycelix_business_revalidation::RevalidationError;
use mycelix_finance_exact::AssetAmount;
use mycelix_finance_settlement::FinalityProfileRef;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinancialReservationRequest {
    pub request_id: ReferenceId,
    pub finance_domain: ReferenceId,
    pub action_contract: ActionContractRef,
    pub decision: DecisionCapsuleRef,
    pub authorized_intent: AuthorizedIntentRef,
    pub intent_digest: Digest32,
    pub authority_lease_id: ReferenceId,
    pub authority_epoch: u64,
    pub fencing_token: u64,
    pub subject: SubjectRef,
    pub amount: AssetAmount,
    pub effect_class: ReferenceId,
    pub aggregate_policy_keys: BTreeSet<ReferenceId>,
    pub idempotency_key: ReferenceId,
    pub required_finality_profile: FinalityProfileRef,
    pub requested_at_unix_ms: u64,
    pub expires_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationRequestError {
    ZeroAmount,
    InvalidWindow,
    DecisionMismatch,
    AuthorizedIntentMismatch,
    ActionContractMismatch,
    IntentDigestMismatch,
    AuthorityLeaseMismatch,
    AuthorityEpochMismatch,
    FencingTokenMismatch,
    RequestBeforeApproval,
    OutlivesLease,
    Lease(AuthorityLeaseError),
}

impl FinancialReservationRequest {
    pub fn validate_against_authorization(
        &self,
        authorization: &AuthorizationBinding,
        lease: &AuthorityLeaseRef,
        current_authority_epoch: u64,
    ) -> Result<(), ReservationRequestError> {
        if self.amount.atomic_units() == 0 {
            return Err(ReservationRequestError::ZeroAmount);
        }
        if self.requested_at_unix_ms >= self.expires_at_unix_ms {
            return Err(ReservationRequestError::InvalidWindow);
        }
        if self.decision != authorization.decision {
            return Err(ReservationRequestError::DecisionMismatch);
        }
        if self.authorized_intent != authorization.authorized_intent {
            return Err(ReservationRequestError::AuthorizedIntentMismatch);
        }
        if self.action_contract != authorization.action_contract {
            return Err(ReservationRequestError::ActionContractMismatch);
        }
        if self.intent_digest != authorization.intent_digest {
            return Err(ReservationRequestError::IntentDigestMismatch);
        }
        if self.authority_lease_id != authorization.authority_lease_id
            || self.authority_lease_id != lease.lease_id
        {
            return Err(ReservationRequestError::AuthorityLeaseMismatch);
        }
        if self.authority_epoch != authorization.authority_epoch
            || self.authority_epoch != lease.authority_epoch
        {
            return Err(ReservationRequestError::AuthorityEpochMismatch);
        }
        if self.fencing_token != authorization.fencing_token
            || self.fencing_token != lease.fencing_token
        {
            return Err(ReservationRequestError::FencingTokenMismatch);
        }
        if self.requested_at_unix_ms < authorization.approved_at_unix_ms {
            return Err(ReservationRequestError::RequestBeforeApproval);
        }
        lease
            .validate_at(self.requested_at_unix_ms, current_authority_epoch)
            .map_err(ReservationRequestError::Lease)?;
        if self.expires_at_unix_ms > lease.expires_at_unix_ms {
            return Err(ReservationRequestError::OutlivesLease);
        }
        Ok(())
    }
}

/// A Finance-domain reservation record supplied to this contract layer.
///
/// This value is intentionally not a proof of Finance issuance merely because it
/// is well-formed. The Business `PreparedAction` must carry the exact domain-owned
/// reservation reference/digest, and runtime integration must authenticate the
/// Finance source independently. This crate only proves cross-record consistency.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinancialReservation {
    pub reservation_id: ReservationId,
    pub request_id: ReferenceId,
    pub finance_domain: ReferenceId,
    pub subject: SubjectRef,
    pub amount: AssetAmount,
    pub effect_class: ReferenceId,
    pub action_contract: ActionContractRef,
    pub decision: DecisionCapsuleRef,
    pub authorized_intent: AuthorizedIntentRef,
    pub intent_digest: Digest32,
    pub authority_lease_id: ReferenceId,
    pub authority_epoch: u64,
    pub fencing_token: u64,
    pub aggregate_policy_keys: BTreeSet<ReferenceId>,
    pub idempotency_key: ReferenceId,
    pub required_finality_profile: FinalityProfileRef,
    pub reservation_digest: Digest32,
    pub finance_frontier: Digest32,
    pub issued_at_unix_ms: u64,
    pub expires_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FinancialReservationError {
    RequestIdMismatch,
    FinanceDomainMismatch,
    SubjectMismatch,
    AmountMismatch,
    EffectClassMismatch,
    ActionContractMismatch,
    DecisionMismatch,
    AuthorizedIntentMismatch,
    IntentDigestMismatch,
    AuthorityLeaseMismatch,
    AuthorityEpochMismatch,
    FencingTokenMismatch,
    AggregatePolicyMismatch,
    IdempotencyMismatch,
    FinalityProfileMismatch,
    IssuedBeforeRequest,
    InvalidWindow,
    OutlivesRequest,
    Expired,
}

impl FinancialReservation {
    pub fn validate_against_request(
        &self,
        request: &FinancialReservationRequest,
        now_unix_ms: u64,
    ) -> Result<(), FinancialReservationError> {
        if self.request_id != request.request_id {
            return Err(FinancialReservationError::RequestIdMismatch);
        }
        if self.finance_domain != request.finance_domain {
            return Err(FinancialReservationError::FinanceDomainMismatch);
        }
        if self.subject != request.subject {
            return Err(FinancialReservationError::SubjectMismatch);
        }
        if self.amount != request.amount {
            return Err(FinancialReservationError::AmountMismatch);
        }
        if self.effect_class != request.effect_class {
            return Err(FinancialReservationError::EffectClassMismatch);
        }
        if self.action_contract != request.action_contract {
            return Err(FinancialReservationError::ActionContractMismatch);
        }
        if self.decision != request.decision {
            return Err(FinancialReservationError::DecisionMismatch);
        }
        if self.authorized_intent != request.authorized_intent {
            return Err(FinancialReservationError::AuthorizedIntentMismatch);
        }
        if self.intent_digest != request.intent_digest {
            return Err(FinancialReservationError::IntentDigestMismatch);
        }
        if self.authority_lease_id != request.authority_lease_id {
            return Err(FinancialReservationError::AuthorityLeaseMismatch);
        }
        if self.authority_epoch != request.authority_epoch {
            return Err(FinancialReservationError::AuthorityEpochMismatch);
        }
        if self.fencing_token != request.fencing_token {
            return Err(FinancialReservationError::FencingTokenMismatch);
        }
        if self.aggregate_policy_keys != request.aggregate_policy_keys {
            return Err(FinancialReservationError::AggregatePolicyMismatch);
        }
        if self.idempotency_key != request.idempotency_key {
            return Err(FinancialReservationError::IdempotencyMismatch);
        }
        if self.required_finality_profile != request.required_finality_profile {
            return Err(FinancialReservationError::FinalityProfileMismatch);
        }
        if self.issued_at_unix_ms < request.requested_at_unix_ms {
            return Err(FinancialReservationError::IssuedBeforeRequest);
        }
        if self.issued_at_unix_ms >= self.expires_at_unix_ms {
            return Err(FinancialReservationError::InvalidWindow);
        }
        if self.expires_at_unix_ms > request.expires_at_unix_ms {
            return Err(FinancialReservationError::OutlivesRequest);
        }
        if now_unix_ms >= self.expires_at_unix_ms {
            return Err(FinancialReservationError::Expired);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FinanceBindingError {
    InvalidCoordination(CoordinationError),
    InvalidPreparedAction(PreparedActionError),
    InvalidAuthorization(AuthorizationBindingError),
    InvalidAttempt(ExecutionAttemptBindingError),
    Revalidation(RevalidationError),
    InvalidRequest(ReservationRequestError),
    InvalidReservation(FinancialReservationError),
    FinanceDomainNotRequired,
    ActionContractMismatch,
    DecisionMismatch,
    BindingTimeMismatch,
    ReservationIssuedAfterPreparation,
    AggregatePolicyMismatch,
    ReservationReferenceMissing,
    DuplicateReservationReference,
    ReservationReferenceMismatch,
    IdempotencyMismatch,
    RequestExpired,
}

/// Sealed result that the supplied records are mutually consistent for one exact
/// Business execution attempt.
///
/// This is not a proof that the Finance runtime genuinely issued the supplied
/// reservation; runtime source authentication remains a separate boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinanceBoundForAttempt {
    attempt: ExecutionAttemptRef,
    request_id: ReferenceId,
    reservation_id: ReservationId,
    amount: AssetAmount,
    effect_class: ReferenceId,
    required_finality_profile: FinalityProfileRef,
    finance_frontier: Digest32,
    bound_at_unix_ms: u64,
}

impl FinanceBoundForAttempt {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        attempt: ExecutionAttemptRef,
        request_id: ReferenceId,
        reservation_id: ReservationId,
        amount: AssetAmount,
        effect_class: ReferenceId,
        required_finality_profile: FinalityProfileRef,
        finance_frontier: Digest32,
        bound_at_unix_ms: u64,
    ) -> Self {
        Self {
            attempt,
            request_id,
            reservation_id,
            amount,
            effect_class,
            required_finality_profile,
            finance_frontier,
            bound_at_unix_ms,
        }
    }

    pub fn attempt(&self) -> &ExecutionAttemptRef {
        &self.attempt
    }

    pub fn request_id(&self) -> &ReferenceId {
        &self.request_id
    }

    pub fn reservation_id(&self) -> &ReservationId {
        &self.reservation_id
    }

    pub fn amount(&self) -> &AssetAmount {
        &self.amount
    }

    pub fn effect_class(&self) -> &ReferenceId {
        &self.effect_class
    }

    pub fn required_finality_profile(&self) -> &FinalityProfileRef {
        &self.required_finality_profile
    }

    pub fn finance_frontier(&self) -> Digest32 {
        self.finance_frontier
    }

    pub fn bound_at_unix_ms(&self) -> u64 {
        self.bound_at_unix_ms
    }
}

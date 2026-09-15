use std::collections::BTreeSet;

use mycelix_business_coordination::CoordinationEnvelope;
use mycelix_business_decision::{
    AuthorizationBinding, DecisionCapsule, ExecutionAttemptRecord,
};
use mycelix_business_revalidation::{
    ExecutionRevalidationContract, RevalidationEvidence,
};

use crate::{
    FinanceBindingError, FinanceBoundForAttempt, FinancialReservation,
    FinancialReservationRequest,
};

/// Bind one already-authorized Business execution attempt to one exact Finance
/// reservation.
///
/// Success is deliberately narrow: it proves only that the supplied Business
/// decision/authorization/attempt, Finance reservation, execution-time
/// revalidation, and policy identifiers are mutually consistent at the exact
/// attempt boundary. It does not execute a payment, mutate a balance, establish
/// settlement/business success, or authenticate Finance issuance by itself.
#[allow(clippy::too_many_arguments)]
pub fn bind_finance_for_attempt(
    decision: &DecisionCapsule,
    coordination: &CoordinationEnvelope,
    authorization: &AuthorizationBinding,
    attempt: &ExecutionAttemptRecord,
    request: &FinancialReservationRequest,
    reservation: &FinancialReservation,
    revalidation_contract: &ExecutionRevalidationContract,
    revalidation_evidence: &[RevalidationEvidence],
    current_authority_epoch: u64,
    bound_at_unix_ms: u64,
) -> Result<FinanceBoundForAttempt, FinanceBindingError> {
    coordination
        .validate()
        .map_err(FinanceBindingError::InvalidCoordination)?;

    // Binding is defined at the concrete Business attempt boundary. Allowing a
    // later timestamp would make post-hoc Finance binding indistinguishable from
    // a reservation that actually constrained execution.
    if bound_at_unix_ms != attempt.started_at_unix_ms {
        return Err(FinanceBindingError::BindingTimeMismatch);
    }

    coordination
        .prepared
        .validate_for_execution(bound_at_unix_ms, current_authority_epoch)
        .map_err(FinanceBindingError::InvalidPreparedAction)?;

    authorization
        .validate_prepared(&coordination.prepared)
        .map_err(FinanceBindingError::InvalidAuthorization)?;

    attempt
        .validate_binding(authorization, coordination)
        .map_err(FinanceBindingError::InvalidAttempt)?;

    // Do not trust a caller-supplied RevalidatedForAttempt marker. Execute the
    // Business Fabric's actual revalidation theorem against the exact attempt
    // timestamp and current authority epoch.
    revalidation_contract
        .evaluate(
            decision,
            coordination,
            current_authority_epoch,
            bound_at_unix_ms,
            revalidation_evidence,
        )
        .map_err(FinanceBindingError::Revalidation)?;

    request
        .validate_against_authorization(
            authorization,
            &coordination.prepared.authority_lease,
            current_authority_epoch,
        )
        .map_err(FinanceBindingError::InvalidRequest)?;

    reservation
        .validate_against_request(request, bound_at_unix_ms)
        .map_err(FinanceBindingError::InvalidReservation)?;

    if !coordination
        .contract
        .required_reservation_domains
        .contains(&request.finance_domain)
    {
        return Err(FinanceBindingError::FinanceDomainNotRequired);
    }

    if request.action_contract != coordination.contract.contract_ref {
        return Err(FinanceBindingError::ActionContractMismatch);
    }
    if request.decision != attempt.decision {
        return Err(FinanceBindingError::DecisionMismatch);
    }

    // PreparedAction carries the reservation reference, so the authoritative
    // Finance reservation must already have existed when preparation occurred.
    if reservation.issued_at_unix_ms > coordination.prepared.prepared_at_unix_ms {
        return Err(FinanceBindingError::ReservationIssuedAfterPreparation);
    }

    if bound_at_unix_ms >= request.expires_at_unix_ms {
        return Err(FinanceBindingError::RequestExpired);
    }

    if request.idempotency_key != coordination.prepared.idempotency_key {
        return Err(FinanceBindingError::IdempotencyMismatch);
    }

    let contract_aggregate_keys = coordination
        .contract
        .aggregate_policy_keys
        .iter()
        .map(|item| item.key.clone())
        .collect::<BTreeSet<_>>();
    if request.aggregate_policy_keys != contract_aggregate_keys {
        return Err(FinanceBindingError::AggregatePolicyMismatch);
    }

    // Find the exact domain-owned reservation reference embedded in the
    // PreparedAction. Duplicate exact identities are ambiguous and fail closed.
    let references = coordination
        .prepared
        .reservations
        .iter()
        .filter(|item| {
            item.domain == request.finance_domain
                && item.reservation_id == reservation.reservation_id
        })
        .collect::<Vec<_>>();

    let reservation_ref = match references.as_slice() {
        [] => return Err(FinanceBindingError::ReservationReferenceMissing),
        [single] => *single,
        _ => return Err(FinanceBindingError::DuplicateReservationReference),
    };

    if reservation_ref.subject != reservation.subject
        || reservation_ref.digest != reservation.reservation_digest
        || reservation_ref.expires_at_unix_ms != reservation.expires_at_unix_ms
    {
        return Err(FinanceBindingError::ReservationReferenceMismatch);
    }

    Ok(FinanceBoundForAttempt::new(
        attempt.attempt.clone(),
        request.request_id.clone(),
        reservation.reservation_id.clone(),
        reservation.amount.clone(),
        reservation.effect_class.clone(),
        reservation.required_finality_profile.clone(),
        reservation.finance_frontier,
        bound_at_unix_ms,
    ))
}

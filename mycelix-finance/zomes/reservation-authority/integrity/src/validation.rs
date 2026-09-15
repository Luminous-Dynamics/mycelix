use hdi::prelude::*;
use mycelix_finance_reservation_core::{
    apply_reservation_transition, ReservationLifecycle, ReservationTransition,
    ReservationTransitionOutcome,
};

use crate::wire::{ReservationAuthorityEntry, ReservationLifecycleWire};

pub fn validate_create_reservation(
    action: Create,
    entry: ReservationAuthorityEntry,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = entry.issuer_authority_matches(&action.author) {
        return invalid(reason);
    }

    let (descriptor, state) = match entry.verified_semantics() {
        Ok(value) => value,
        Err(reason) => return invalid(reason),
    };

    if !matches!(state.lifecycle(), ReservationLifecycle::Active) || state.sequence() != 1 {
        return invalid("reservation creation must commit the canonical Active sequence-1 state");
    }

    let action_ms = timestamp_millis(action.timestamp)?;
    if descriptor.issued_at_unix_ms > action_ms {
        return invalid("reservation issued_at cannot be after its Holochain Create action");
    }

    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_update_reservation(
    action: Update,
    entry: ReservationAuthorityEntry,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = entry.issuer_authority_matches(&action.author) {
        return invalid(reason);
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    if original_record.action().author() != &action.author {
        return invalid("only the original reservation issuer may author a lifecycle update");
    }

    let original_entry = original_record
        .entry()
        .to_app_option::<ReservationAuthorityEntry>()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "reservation update must reference a ReservationAuthorityEntry".into()
            ))
        })?;

    if original_entry.issuer_agent != entry.issuer_agent {
        return invalid("issuer_agent is immutable across reservation lifecycle updates");
    }
    if original_entry.descriptor != entry.descriptor
        || original_entry.descriptor_commitment != entry.descriptor_commitment
    {
        return invalid("reservation descriptor and descriptor commitment are immutable");
    }

    let (descriptor, prior_state) = match original_entry.verified_semantics() {
        Ok(value) => value,
        Err(reason) => return invalid(format!("invalid prior reservation state: {reason}")),
    };
    let (_, claimed_new_state) = match entry.verified_semantics() {
        Ok(value) => value,
        Err(reason) => return invalid(format!("invalid claimed reservation state: {reason}")),
    };

    let transition = match transition_from_wire(&entry.lifecycle) {
        Ok(value) => value,
        Err(reason) => return invalid(reason),
    };

    let transition_time = transition_time(&transition);
    let action_ms = timestamp_millis(action.timestamp)?;
    if transition_time > action_ms {
        return invalid("reservation transition time cannot be after its Holochain Update action");
    }

    let expected = prior_state.expected();
    let outcome = match apply_reservation_transition(
        &descriptor,
        &prior_state,
        expected,
        transition,
    ) {
        Ok(value) => value,
        Err(error) => {
            return invalid(format!(
                "reservation lifecycle update violates FIN-ECO-004A: {error:?}"
            ))
        }
    };

    let expected_new_state = match outcome {
        ReservationTransitionOutcome::Applied(state) => state,
        ReservationTransitionOutcome::IdempotentReplay(_) => {
            return invalid("idempotent replay must return the existing action, not create an update")
        }
    };

    if expected_new_state != claimed_new_state {
        return invalid("claimed lifecycle state is not the exact deterministic transition result");
    }

    Ok(ValidateCallbackResult::Valid)
}

fn transition_from_wire(value: &ReservationLifecycleWire) -> Result<ReservationTransition, String> {
    match value {
        ReservationLifecycleWire::Active => {
            Err("reservation updates cannot transition back to Active".into())
        }
        ReservationLifecycleWire::Consumed {
            attempt,
            idempotency_key,
            consumed_at_unix_ms,
        } => {
            let core = ReservationLifecycleWire::Consumed {
                attempt: attempt.clone(),
                idempotency_key: idempotency_key.clone(),
                consumed_at_unix_ms: *consumed_at_unix_ms,
            }
            .to_core()?;
            match core {
                ReservationLifecycle::Consumed {
                    attempt,
                    idempotency_key,
                    consumed_at_unix_ms,
                } => Ok(ReservationTransition::Consume {
                    attempt,
                    idempotency_key,
                    at_unix_ms: consumed_at_unix_ms,
                }),
                _ => unreachable!(),
            }
        }
        ReservationLifecycleWire::Released {
            evidence,
            released_at_unix_ms,
        } => {
            let core = ReservationLifecycleWire::Released {
                evidence: evidence.clone(),
                released_at_unix_ms: *released_at_unix_ms,
            }
            .to_core()?;
            match core {
                ReservationLifecycle::Released {
                    evidence,
                    released_at_unix_ms,
                } => Ok(ReservationTransition::Release {
                    evidence,
                    at_unix_ms: released_at_unix_ms,
                }),
                _ => unreachable!(),
            }
        }
        ReservationLifecycleWire::Revoked {
            evidence,
            revoked_at_unix_ms,
        } => {
            let core = ReservationLifecycleWire::Revoked {
                evidence: evidence.clone(),
                revoked_at_unix_ms: *revoked_at_unix_ms,
            }
            .to_core()?;
            match core {
                ReservationLifecycle::Revoked {
                    evidence,
                    revoked_at_unix_ms,
                } => Ok(ReservationTransition::Revoke {
                    evidence,
                    at_unix_ms: revoked_at_unix_ms,
                }),
                _ => unreachable!(),
            }
        }
        ReservationLifecycleWire::Expired { expired_at_unix_ms } => {
            Ok(ReservationTransition::Expire {
                at_unix_ms: *expired_at_unix_ms,
            })
        }
    }
}

fn transition_time(value: &ReservationTransition) -> u64 {
    match value {
        ReservationTransition::Consume { at_unix_ms, .. }
        | ReservationTransition::Release { at_unix_ms, .. }
        | ReservationTransition::Revoke { at_unix_ms, .. }
        | ReservationTransition::Expire { at_unix_ms } => *at_unix_ms,
    }
}

fn timestamp_millis(timestamp: Timestamp) -> ExternResult<u64> {
    let micros = timestamp.as_micros();
    if micros < 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "negative Holochain timestamps are not valid for Finance reservations".into()
        )));
    }
    u64::try_from(micros / 1_000)
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))
}

fn invalid(reason: impl Into<String>) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(reason.into()))
}

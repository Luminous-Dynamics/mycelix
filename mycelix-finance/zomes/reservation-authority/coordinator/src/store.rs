use hdk::prelude::*;
use mycelix_business_core::{ExecutionAttemptRef, ReferenceId};
use mycelix_finance_reservation_core::{
    apply_reservation_transition, reservation_commitment, ReservationLifecycle, ReservationState,
    ReservationTransition, ReservationTransitionOutcome,
};
use reservation_authority_integrity::{
    authority_did_for_agent, EntryTypes, ReservationAuthorityEntry, ReservationDescriptorWire,
    ReservationLifecycleWire, RESERVATION_AUTHORITY_SCHEMA_VERSION,
};

use crate::model::{ReservationStateView, ReservationTransitionReceipt};

struct ResolvedReservation {
    root_action: ActionHash,
    root_entry: ReservationAuthorityEntry,
    current_action: ActionHash,
    current_entry: ReservationAuthorityEntry,
}

pub fn issue_reservation(
    mut descriptor_wire: ReservationDescriptorWire,
) -> ExternResult<ReservationStateView> {
    let agent = agent_info()?.agent_initial_pubkey;
    let now = now_millis()?;

    // Issuer identity and issue time are runtime-owned, never trusted from caller input.
    descriptor_wire.issuer_authority = authority_did_for_agent(&agent);
    descriptor_wire.issued_at_unix_ms = now;
    if descriptor_wire.expires_at_unix_ms <= now {
        return guest_error("reservation expiry must be after runtime issuance time");
    }

    let descriptor = descriptor_wire
        .to_core()
        .map_err(|reason| guest(reason))?;
    ensure_unique_reservation_id(&descriptor_wire.reservation_id)?;

    let state = ReservationState::initial(&descriptor)
        .map_err(|error| guest(format!("cannot create initial reservation state: {error:?}")))?;
    let descriptor_commitment = reservation_commitment(&descriptor)
        .map_err(|error| guest(format!("cannot commit reservation descriptor: {error:?}")))?;

    let entry = ReservationAuthorityEntry {
        schema_version: RESERVATION_AUTHORITY_SCHEMA_VERSION,
        descriptor: descriptor_wire,
        descriptor_commitment: descriptor_commitment.into_bytes(),
        state_sequence: state.sequence(),
        lifecycle: ReservationLifecycleWire::from_core(state.lifecycle()),
        state_commitment: state.state_commitment().into_bytes(),
        issuer_agent: agent.clone(),
    };

    // create_entry uses strict source-chain ordering. Concurrent duplicate issuance
    // can both observe "not found", but only one source-chain commit may win; a
    // retry sees the existing reservation ID on the issuer's chain.
    let root_action = create_entry(&EntryTypes::ReservationAuthority(entry.clone()))?;
    let committed = get(root_action.clone(), GetOptions::default())?
        .ok_or_else(|| guest("reservation record missing immediately after creation"))?;
    let committed_entry = app_entry(&committed)?;
    committed_entry
        .issuer_authority_matches(committed.action().author())
        .map_err(guest)?;
    committed_entry.verified_semantics().map_err(guest)?;

    Ok(view(root_action.clone(), root_action, committed_entry))
}

pub fn get_reservation_state(root_action: ActionHash) -> ExternResult<ReservationStateView> {
    let resolved = resolve_unique(root_action)?;
    resolved.current_entry.verified_semantics().map_err(guest)?;
    Ok(view(
        resolved.root_action,
        resolved.current_action,
        resolved.current_entry,
    ))
}

pub fn consume_reservation(
    root_action: ActionHash,
    expected_descriptor_commitment: [u8; 32],
    expected_sequence: u64,
    expected_state_commitment: [u8; 32],
    attempt: String,
    idempotency_key: String,
) -> ExternResult<ReservationTransitionReceipt> {
    let resolved = resolve_unique(root_action)?;
    ensure_local_issuer(&resolved.current_entry)?;
    ensure_expected_root(
        &resolved,
        expected_descriptor_commitment,
        expected_sequence,
        expected_state_commitment,
    )?;

    let attempt_ref = ExecutionAttemptRef(reference(&attempt)?);
    let idempotency_ref = reference(&idempotency_key)?;
    let (_, current_state) = resolved.current_entry.verified_semantics().map_err(guest)?;

    if let ReservationLifecycle::Consumed {
        attempt: prior_attempt,
        idempotency_key: prior_idempotency,
        ..
    } = current_state.lifecycle()
    {
        if prior_attempt == &attempt_ref && prior_idempotency == &idempotency_ref {
            return receipt_for_existing_terminal(&resolved, true);
        }
        return guest_error("reservation is already consumed by another attempt/idempotency identity");
    }
    ensure_active(&current_state)?;

    let now = now_millis()?;
    let transition = ReservationTransition::Consume {
        attempt: attempt_ref,
        idempotency_key: idempotency_ref,
        at_unix_ms: now,
    };
    commit_terminal_transition(resolved, transition)
}

pub fn release_reservation(
    root_action: ActionHash,
    expected_descriptor_commitment: [u8; 32],
    expected_sequence: u64,
    expected_state_commitment: [u8; 32],
    evidence: String,
) -> ExternResult<ReservationTransitionReceipt> {
    let resolved = resolve_unique(root_action)?;
    ensure_local_issuer(&resolved.current_entry)?;
    ensure_expected_root(
        &resolved,
        expected_descriptor_commitment,
        expected_sequence,
        expected_state_commitment,
    )?;

    let evidence_ref = reference(&evidence)?;
    let (_, current_state) = resolved.current_entry.verified_semantics().map_err(guest)?;
    if let ReservationLifecycle::Released { evidence: prior, .. } = current_state.lifecycle() {
        if prior == &evidence_ref {
            return receipt_for_existing_terminal(&resolved, true);
        }
    }
    ensure_active(&current_state)?;

    commit_terminal_transition(
        resolved,
        ReservationTransition::Release {
            evidence: evidence_ref,
            at_unix_ms: now_millis()?,
        },
    )
}

pub fn revoke_reservation(
    root_action: ActionHash,
    expected_descriptor_commitment: [u8; 32],
    expected_sequence: u64,
    expected_state_commitment: [u8; 32],
    evidence: String,
) -> ExternResult<ReservationTransitionReceipt> {
    let resolved = resolve_unique(root_action)?;
    ensure_local_issuer(&resolved.current_entry)?;
    ensure_expected_root(
        &resolved,
        expected_descriptor_commitment,
        expected_sequence,
        expected_state_commitment,
    )?;

    let evidence_ref = reference(&evidence)?;
    let (_, current_state) = resolved.current_entry.verified_semantics().map_err(guest)?;
    if let ReservationLifecycle::Revoked { evidence: prior, .. } = current_state.lifecycle() {
        if prior == &evidence_ref {
            return receipt_for_existing_terminal(&resolved, true);
        }
    }
    ensure_active(&current_state)?;

    commit_terminal_transition(
        resolved,
        ReservationTransition::Revoke {
            evidence: evidence_ref,
            at_unix_ms: now_millis()?,
        },
    )
}

pub fn expire_reservation(
    root_action: ActionHash,
    expected_descriptor_commitment: [u8; 32],
    expected_sequence: u64,
    expected_state_commitment: [u8; 32],
) -> ExternResult<ReservationTransitionReceipt> {
    let resolved = resolve_unique(root_action)?;
    ensure_local_issuer(&resolved.current_entry)?;
    ensure_expected_root(
        &resolved,
        expected_descriptor_commitment,
        expected_sequence,
        expected_state_commitment,
    )?;

    let (_, current_state) = resolved.current_entry.verified_semantics().map_err(guest)?;
    if matches!(current_state.lifecycle(), ReservationLifecycle::Expired { .. }) {
        return receipt_for_existing_terminal(&resolved, true);
    }
    ensure_active(&current_state)?;

    commit_terminal_transition(
        resolved,
        ReservationTransition::Expire {
            at_unix_ms: now_millis()?,
        },
    )
}

fn commit_terminal_transition(
    resolved: ResolvedReservation,
    transition: ReservationTransition,
) -> ExternResult<ReservationTransitionReceipt> {
    let (descriptor, prior_state) = resolved.current_entry.verified_semantics().map_err(guest)?;
    let prior_sequence = prior_state.sequence();
    let prior_state_commitment = prior_state.state_commitment().into_bytes();

    let outcome = apply_reservation_transition(
        &descriptor,
        &prior_state,
        prior_state.expected(),
        transition,
    )
    .map_err(|error| guest(format!("reservation transition rejected: {error:?}")))?;

    let new_state = match outcome {
        ReservationTransitionOutcome::Applied(state) => state,
        ReservationTransitionOutcome::IdempotentReplay(_) => {
            return guest_error("internal error: idempotent replay reached write path")
        }
    };

    let mut new_entry = resolved.current_entry.clone();
    new_entry.state_sequence = new_state.sequence();
    new_entry.lifecycle = ReservationLifecycleWire::from_core(new_state.lifecycle());
    new_entry.state_commitment = new_state.state_commitment().into_bytes();

    // update_entry uses strict ChainTopOrdering in HDK 0.6.1. If another zome
    // call appended to this issuer chain after our call snapshot, this commit
    // fails rather than silently relaxing the chain top. Callers may retry from
    // the immutable root action, at which point we re-resolve authoritative state.
    let transition_action = update_entry(
        resolved.current_action.clone(),
        &EntryTypes::ReservationAuthority(new_entry.clone()),
    )
    .map_err(|error| {
        guest(format!(
            "strict issuer source-chain commit failed; re-resolve reservation before retry: {error}"
        ))
    })?;

    let committed = get(transition_action.clone(), GetOptions::default())?
        .ok_or_else(|| guest("reservation transition missing immediately after commit"))?;
    let committed_entry = app_entry(&committed)?;
    committed_entry
        .issuer_authority_matches(committed.action().author())
        .map_err(guest)?;
    let (_, committed_state) = committed_entry.verified_semantics().map_err(guest)?;
    if committed_state != new_state {
        return guest_error("committed reservation state differs from deterministic transition result");
    }

    let caller = call_info()?.provenance;
    Ok(ReservationTransitionReceipt {
        root_action: resolved.root_action,
        prior_action: resolved.current_action,
        transition_action,
        issuer_agent: committed_entry.issuer_agent,
        caller,
        descriptor_commitment: committed_entry.descriptor_commitment,
        prior_sequence,
        prior_state_commitment,
        resulting_sequence: committed_entry.state_sequence,
        resulting_state_commitment: committed_entry.state_commitment,
        resulting_lifecycle: committed_entry.lifecycle,
        idempotent_replay: false,
    })
}

fn receipt_for_existing_terminal(
    resolved: &ResolvedReservation,
    idempotent_replay: bool,
) -> ExternResult<ReservationTransitionReceipt> {
    let (_, root_state) = resolved.root_entry.verified_semantics().map_err(guest)?;
    let caller = call_info()?.provenance;
    Ok(ReservationTransitionReceipt {
        root_action: resolved.root_action.clone(),
        prior_action: resolved.root_action.clone(),
        transition_action: resolved.current_action.clone(),
        issuer_agent: resolved.current_entry.issuer_agent.clone(),
        caller,
        descriptor_commitment: resolved.current_entry.descriptor_commitment,
        prior_sequence: root_state.sequence(),
        prior_state_commitment: root_state.state_commitment().into_bytes(),
        resulting_sequence: resolved.current_entry.state_sequence,
        resulting_state_commitment: resolved.current_entry.state_commitment,
        resulting_lifecycle: resolved.current_entry.lifecycle.clone(),
        idempotent_replay,
    })
}

fn ensure_expected_root(
    resolved: &ResolvedReservation,
    expected_descriptor_commitment: [u8; 32],
    expected_sequence: u64,
    expected_state_commitment: [u8; 32],
) -> ExternResult<()> {
    if resolved.root_entry.descriptor_commitment != expected_descriptor_commitment {
        return guest_error("descriptor commitment does not match caller expectation");
    }
    if resolved.root_entry.state_sequence != expected_sequence {
        return guest_error("expected reservation sequence does not match issuance state");
    }
    if resolved.root_entry.state_commitment != expected_state_commitment {
        return guest_error("expected reservation state commitment does not match issuance state");
    }
    Ok(())
}

fn ensure_active(state: &ReservationState) -> ExternResult<()> {
    if matches!(state.lifecycle(), ReservationLifecycle::Active) {
        Ok(())
    } else {
        guest_error("reservation is terminal and cannot accept another lifecycle transition")
    }
}

fn ensure_local_issuer(entry: &ReservationAuthorityEntry) -> ExternResult<()> {
    let local_agent = agent_info()?.agent_initial_pubkey;
    if entry.issuer_agent != local_agent {
        return guest_error("reservation mutations must execute on the original issuer cell");
    }
    entry
        .issuer_authority_matches(&local_agent)
        .map_err(guest)
}

fn ensure_unique_reservation_id(reservation_id: &str) -> ExternResult<()> {
    let records = query(ChainQueryFilter::new().action_type(ActionType::Create).include_entries(true))?;
    for record in records {
        if let Ok(Some(entry)) = record.entry().to_app_option::<ReservationAuthorityEntry>() {
            if entry.descriptor.reservation_id == reservation_id {
                return guest_error("reservation ID already exists on this issuer source chain");
            }
        }
    }
    Ok(())
}

fn resolve_unique(root_action: ActionHash) -> ExternResult<ResolvedReservation> {
    let root_details = get_details(root_action.clone(), GetOptions::default())?
        .ok_or_else(|| guest("reservation root action not found"))?;
    let root_details = match root_details {
        Details::Record(details) => details,
        Details::Entry(_) => return guest_error("reservation root must resolve to record details"),
    };

    if !matches!(root_details.record.action(), Action::Create(_)) {
        return guest_error("reservation root action must be the immutable Create action");
    }
    if !root_details.deletes.is_empty() {
        return guest_error("reservation root carries delete metadata; fail closed");
    }
    if root_details.updates.len() > 1 {
        return guest_error("reservation has multiple sibling lifecycle updates; issuer equivocation/fork detected");
    }

    let root_entry = app_entry(&root_details.record)?;
    root_entry
        .issuer_authority_matches(root_details.record.action().author())
        .map_err(guest)?;
    let (_, root_state) = root_entry.verified_semantics().map_err(guest)?;
    if !matches!(root_state.lifecycle(), ReservationLifecycle::Active) || root_state.sequence() != 1 {
        return guest_error("reservation root is not canonical Active sequence 1");
    }

    let Some(update) = root_details.updates.first() else {
        return Ok(ResolvedReservation {
            root_action: root_action.clone(),
            root_entry: root_entry.clone(),
            current_action: root_action,
            current_entry: root_entry,
        });
    };

    let current_action = update.action_address().clone();
    let current_details = get_details(current_action.clone(), GetOptions::default())?
        .ok_or_else(|| guest("reservation terminal update record not found"))?;
    let current_details = match current_details {
        Details::Record(details) => details,
        Details::Entry(_) => return guest_error("reservation terminal action must resolve to record details"),
    };
    if !current_details.deletes.is_empty() {
        return guest_error("reservation terminal state carries delete metadata; fail closed");
    }
    if !current_details.updates.is_empty() {
        return guest_error("FIN-ECO-004B v1 permits exactly one terminal transition per reservation");
    }

    let current_entry = app_entry(&current_details.record)?;
    current_entry
        .issuer_authority_matches(current_details.record.action().author())
        .map_err(guest)?;
    current_entry.verified_semantics().map_err(guest)?;

    Ok(ResolvedReservation {
        root_action,
        root_entry,
        current_action,
        current_entry,
    })
}

fn app_entry(record: &Record) -> ExternResult<ReservationAuthorityEntry> {
    record
        .entry()
        .to_app_option::<ReservationAuthorityEntry>()
        .map_err(|error| guest(format!("reservation entry deserialization failed: {error:?}")))?
        .ok_or_else(|| guest("record does not contain a ReservationAuthorityEntry"))
}

fn view(
    root_action: ActionHash,
    current_action: ActionHash,
    entry: ReservationAuthorityEntry,
) -> ReservationStateView {
    ReservationStateView {
        root_action,
        current_action,
        issuer_agent: entry.issuer_agent,
        descriptor: entry.descriptor,
        descriptor_commitment: entry.descriptor_commitment,
        state_sequence: entry.state_sequence,
        state_commitment: entry.state_commitment,
        lifecycle: entry.lifecycle,
    }
}

fn now_millis() -> ExternResult<u64> {
    let micros = sys_time()?.as_micros();
    if micros < 0 {
        return guest_error("negative system timestamp is invalid for Finance reservations");
    }
    u64::try_from(micros / 1_000).map_err(|error| guest(error.to_string()))
}

fn reference(value: &str) -> ExternResult<ReferenceId> {
    ReferenceId::new(value.to_string())
        .map_err(|error| guest(format!("invalid canonical reference {value:?}: {error:?}")))
}

fn guest(reason: impl Into<String>) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(reason.into()))
}

fn guest_error<T>(reason: impl Into<String>) -> ExternResult<T> {
    Err(guest(reason))
}

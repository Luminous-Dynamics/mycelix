#![deny(unsafe_code)]

//! FIN-ECO-004B issuer-owned reservation authority coordinator.
//!
//! Mutation functions execute on the issuer cell's source chain. Holochain
//! capability grants remain the transport-level gate for remote callers; this
//! tranche records caller provenance but does not yet elevate a Business Fabric
//! authorization into a capability grant automatically.

use hdk::prelude::*;

mod model;
mod store;

pub use model::*;

#[hdk_extern]
pub fn issue_finance_reservation(
    input: IssueReservationInput,
) -> ExternResult<ReservationStateView> {
    store::issue_reservation(input.descriptor)
}

#[hdk_extern]
pub fn get_finance_reservation_state(
    root_action: ActionHash,
) -> ExternResult<ReservationStateView> {
    store::get_reservation_state(root_action)
}

#[hdk_extern]
pub fn consume_finance_reservation(
    input: ConsumeReservationInput,
) -> ExternResult<AuthenticatedReservationConsumption> {
    let transition = store::consume_reservation(
        input.root_action,
        input.expected_descriptor_commitment,
        input.expected_sequence,
        input.expected_state_commitment,
        input.attempt,
        input.idempotency_key,
    )?;

    // FIN-ECO-004B v1 permits exactly one terminal transition, so after a valid
    // consumption (or its exact idempotent replay) the root resolves to the same
    // immutable descriptor and consumed terminal state. Carry the descriptor in
    // the proof instead of forcing downstream consumers to perform an unbound
    // secondary lookup for amount/asset/effect/authority/finality semantics.
    let state = store::get_reservation_state(transition.root_action.clone())?;
    if state.descriptor_commitment != transition.descriptor_commitment {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "consumption proof descriptor commitment drifted after transition".into()
        )));
    }

    Ok(AuthenticatedReservationConsumption {
        descriptor: state.descriptor,
        transition,
    })
}

#[hdk_extern]
pub fn release_finance_reservation(
    input: EvidenceTransitionInput,
) -> ExternResult<ReservationTransitionReceipt> {
    store::release_reservation(
        input.root_action,
        input.expected_descriptor_commitment,
        input.expected_sequence,
        input.expected_state_commitment,
        input.evidence,
    )
}

#[hdk_extern]
pub fn revoke_finance_reservation(
    input: EvidenceTransitionInput,
) -> ExternResult<ReservationTransitionReceipt> {
    store::revoke_reservation(
        input.root_action,
        input.expected_descriptor_commitment,
        input.expected_sequence,
        input.expected_state_commitment,
        input.evidence,
    )
}

#[hdk_extern]
pub fn expire_finance_reservation(
    input: ExpireReservationInput,
) -> ExternResult<ReservationTransitionReceipt> {
    store::expire_reservation(
        input.root_action,
        input.expected_descriptor_commitment,
        input.expected_sequence,
        input.expected_state_commitment,
    )
}

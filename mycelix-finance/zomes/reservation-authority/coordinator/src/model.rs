use hdk::prelude::*;
use reservation_authority_integrity::{
    ReservationDescriptorWire, ReservationLifecycleWire,
};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IssueReservationInput {
    /// The coordinator overwrites `issuer_authority` and `issued_at_unix_ms`
    /// with values derived from the executing issuer cell before commitment.
    pub descriptor: ReservationDescriptorWire,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ConsumeReservationInput {
    pub root_action: ActionHash,
    pub expected_descriptor_commitment: [u8; 32],
    pub expected_sequence: u64,
    pub expected_state_commitment: [u8; 32],
    pub attempt: String,
    pub idempotency_key: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EvidenceTransitionInput {
    pub root_action: ActionHash,
    pub expected_descriptor_commitment: [u8; 32],
    pub expected_sequence: u64,
    pub expected_state_commitment: [u8; 32],
    pub evidence: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExpireReservationInput {
    pub root_action: ActionHash,
    pub expected_descriptor_commitment: [u8; 32],
    pub expected_sequence: u64,
    pub expected_state_commitment: [u8; 32],
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ReservationStateView {
    pub root_action: ActionHash,
    pub current_action: ActionHash,
    pub issuer_agent: AgentPubKey,
    pub descriptor: ReservationDescriptorWire,
    pub descriptor_commitment: [u8; 32],
    pub state_sequence: u64,
    pub state_commitment: [u8; 32],
    pub lifecycle: ReservationLifecycleWire,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ReservationTransitionReceipt {
    /// Hash of the immutable reservation issuance Create action.
    pub root_action: ActionHash,
    /// Action that represented the exact prior state.
    pub prior_action: ActionHash,
    /// Action that represents the terminal state. For an idempotent retry this
    /// is the already-existing terminal action rather than a new write.
    pub transition_action: ActionHash,
    pub issuer_agent: AgentPubKey,
    /// Provenance of the zome call as reported by Holochain. Remote callers must
    /// still satisfy Holochain capability grants before the call can execute.
    pub caller: AgentPubKey,
    pub descriptor_commitment: [u8; 32],
    pub prior_sequence: u64,
    pub prior_state_commitment: [u8; 32],
    pub resulting_sequence: u64,
    pub resulting_state_commitment: [u8; 32],
    pub resulting_lifecycle: ReservationLifecycleWire,
    pub idempotent_replay: bool,
}

/// Sealed runtime proof for the economically authoritative Consume transition.
///
/// The canonical descriptor is carried with the transition receipt so an auditor
/// or Business binding does not have to perform an unbound secondary lookup to
/// recover amount, asset, effect class, policy frontier, authority lease/fence,
/// idempotency identity, or required settlement-finality profile. The embedded
/// `descriptor_commitment` still binds the descriptor to FIN-ECO-004A canonical
/// bytes; consumers must verify that commitment rather than trusting wire fields.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AuthenticatedReservationConsumption {
    pub descriptor: ReservationDescriptorWire,
    pub transition: ReservationTransitionReceipt,
}

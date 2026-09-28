//! D6E deterministic heterogeneous federation reference model.
//!
//! Transport, storage, cryptography, and production networking are intentionally
//! outside this model. The oracle only answers whether a supplied federation
//! event preserves the declared identity, generation, origin, authorization,
//! expiry, and replay boundaries.
//!
//! Evidence ceiling: ReferenceModelOnly.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FederationNode {
    Local,
    Foreign,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeliveryState {
    Pending,
    Delivered,
    Partitioned,
    Reconciled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorizationState {
    Absent,
    Active,
    Expired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationEnvelope {
    pub logical_delivery_id: &'static str,
    pub attempt_id: &'static str,
    /// Transport attempt identity may change on retry; logical delivery identity may not.
    pub origin: FederationNode,
    /// The node whose authority is being exercised. Evidence origin and authority origin are distinct.
    pub authority_origin: FederationNode,
    pub source_ref: &'static str,
    pub schema_generation: u32,
    pub payload_digest: &'static str,
    pub state: DeliveryState,
    pub authorization: AuthorizationState,
    pub expires_at: u64,
    pub observed_at: u64,
    pub privacy_minimized: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FederationDecision {
    Accepted,
    Replayed,
    RejectedDuplicateMutation,
    RejectedStaleGeneration,
    RejectedExpiredAuthorization,
    RejectedForeignAuthority,
    RejectedPartitioned,
    RejectedPrivacyProjectionAsObservation,
    RejectedOriginMutation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationReceipt {
    pub logical_delivery_id: &'static str,
    pub attempt_id: &'static str,
    pub origin: FederationNode,
    pub payload_digest: &'static str,
    pub state: DeliveryState,
}

pub fn accept_delivery(
    envelope: FederationEnvelope,
    current_generation: u32,
    now: u64,
    existing: Option<FederationReceipt>,
) -> FederationDecision {
    if envelope.schema_generation != current_generation {
        return FederationDecision::RejectedStaleGeneration;
    }
    if envelope.authority_origin == FederationNode::Foreign {
        return FederationDecision::RejectedForeignAuthority;
    }
    if envelope.authorization != AuthorizationState::Active || now >= envelope.expires_at {
        return FederationDecision::RejectedExpiredAuthorization;
    }
    if envelope.state == DeliveryState::Partitioned {
        return FederationDecision::RejectedPartitioned;
    }
    if envelope.privacy_minimized {
        return FederationDecision::RejectedPrivacyProjectionAsObservation;
    }

    if let Some(receipt) = existing {
        if receipt.logical_delivery_id != envelope.logical_delivery_id {
            return FederationDecision::RejectedDuplicateMutation;
        }
        if receipt.origin != envelope.origin {
            return FederationDecision::RejectedOriginMutation;
        }
        if receipt.payload_digest != envelope.payload_digest {
            return FederationDecision::RejectedDuplicateMutation;
        }
        return FederationDecision::Replayed;
    }

    FederationDecision::Accepted
}

pub fn reconcile(
    envelope: FederationEnvelope,
    current_generation: u32,
    now: u64,
    existing: Option<FederationReceipt>,
) -> FederationDecision {
    let mut reconciled = envelope;
    reconciled.state = DeliveryState::Reconciled;
    accept_delivery(reconciled, current_generation, now, existing)
}

pub fn reconnect_cannot_revive_expired_authorization(
    envelope: FederationEnvelope,
    now: u64,
) -> bool {
    envelope.authorization == AuthorizationState::Expired || now >= envelope.expires_at
}

pub fn receipt_for(envelope: FederationEnvelope) -> FederationReceipt {
    FederationReceipt {
        logical_delivery_id: envelope.logical_delivery_id,
        attempt_id: envelope.attempt_id,
        origin: envelope.origin,
        payload_digest: envelope.payload_digest,
        state: envelope.state,
    }
}

pub fn foreign_origin_never_becomes_local(
    original: FederationEnvelope,
    projected: FederationEnvelope,
) -> bool {
    original.origin == FederationNode::Foreign
        && projected.origin == FederationNode::Foreign
        && projected.authority_origin == FederationNode::Local
}

pub fn attempt_may_change_without_mutating_logical_delivery(
    first: FederationEnvelope,
    retry: FederationEnvelope,
) -> bool {
    first.logical_delivery_id == retry.logical_delivery_id
        && first.payload_digest == retry.payload_digest
        && first.origin == retry.origin
        && first.attempt_id != retry.attempt_id
}

#[cfg(test)]
mod tests {
    use super::*;

    fn envelope() -> FederationEnvelope {
        FederationEnvelope {
            logical_delivery_id: "delivery-1",
            attempt_id: "attempt-1",
            origin: FederationNode::Foreign,
            authority_origin: FederationNode::Local,
            source_ref: "node://foreign/evidence/1",
            schema_generation: 7,
            payload_digest: "digest-1",
            state: DeliveryState::Delivered,
            authorization: AuthorizationState::Active,
            expires_at: 100,
            observed_at: 10,
            privacy_minimized: false,
        }
    }

    #[test]
    fn foreign_delivery_is_accepted_without_becoming_local() {
        assert_eq!(accept_delivery(envelope(), 7, 20, None), FederationDecision::Accepted);
        let mut projected = envelope();
        projected.source_ref = "node://foreign/evidence/1";
        assert!(foreign_origin_never_becomes_local(envelope(), projected));
    }

    #[test]
    fn foreign_authority_cannot_become_local_authority_by_projection() {
        let mut e = envelope();
        e.authority_origin = FederationNode::Foreign;
        assert_eq!(
            accept_delivery(e, 7, 20, None),
            FederationDecision::RejectedForeignAuthority
        );
    }

    #[test]
    fn retry_changes_attempt_but_not_logical_identity() {
        let mut retry = envelope();
        retry.attempt_id = "attempt-2";
        assert!(attempt_may_change_without_mutating_logical_delivery(envelope(), retry));
    }

    #[test]
    fn duplicate_identical_delivery_replays() {
        let e = envelope();
        let receipt = receipt_for(e);
        assert_eq!(accept_delivery(e, 7, 20, Some(receipt)), FederationDecision::Replayed);
    }

    #[test]
    fn reconnect_preserves_origin_and_logical_identity() {
        let e = envelope();
        let mut retry = e;
        retry.attempt_id = "attempt-2";
        assert!(attempt_may_change_without_mutating_logical_delivery(e, retry));
        assert_eq!(receipt_for(e).logical_delivery_id, receipt_for(retry).logical_delivery_id);
        assert_eq!(receipt_for(e).origin, FederationNode::Foreign);
    }

    #[test]
    fn duplicate_payload_mutation_fails_closed() {
        let e = envelope();
        let receipt = receipt_for(e);
        let mut replay = e;
        replay.payload_digest = "digest-mutated";
        assert_eq!(
            accept_delivery(replay, 7, 20, Some(receipt)),
            FederationDecision::RejectedDuplicateMutation
        );
    }

    #[test]
    fn stale_schema_fails_closed() {
        assert_eq!(accept_delivery(envelope(), 8, 20, None), FederationDecision::RejectedStaleGeneration);
    }

    #[test]
    fn expired_authorization_cannot_revive_on_reconnect() {
        let mut e = envelope();
        e.authorization = AuthorizationState::Expired;
        assert!(reconnect_cannot_revive_expired_authorization(e, 20));
        assert_eq!(reconcile(e, 7, 20, None), FederationDecision::RejectedExpiredAuthorization);
    }

    #[test]
    fn partition_does_not_fabricate_delivery() {
        let mut e = envelope();
        e.state = DeliveryState::Partitioned;
        assert_eq!(accept_delivery(e, 7, 20, None), FederationDecision::RejectedPartitioned);
    }

    #[test]
    fn privacy_projection_cannot_become_observation() {
        let mut e = envelope();
        e.privacy_minimized = true;
        assert_eq!(
            accept_delivery(e, 7, 20, None),
            FederationDecision::RejectedPrivacyProjectionAsObservation
        );
    }

    #[test]
    fn origin_mutation_fails_closed() {
        let e = envelope();
        let mut receipt = receipt_for(e);
        receipt.origin = FederationNode::Local;
        assert_eq!(
            accept_delivery(e, 7, 20, Some(receipt)),
            FederationDecision::RejectedOriginMutation
        );
    }
}

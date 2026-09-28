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


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationObservation {
    pub observation_id: &'static str,
    pub work_id: &'static str,
    pub origin: FederationNode,
    pub quantity: u32,
    pub evidence_ref: &'static str,
    pub observed_at: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObservationConflict {
    pub work_id: &'static str,
    pub left_observation_id: &'static str,
    pub left_origin: FederationNode,
    pub left_quantity: u32,
    pub right_observation_id: &'static str,
    pub right_origin: FederationNode,
    pub right_quantity: u32,
}

pub fn reconcile_observations(
    left: FederationObservation,
    right: FederationObservation,
) -> Option<ObservationConflict> {
    if left.work_id != right.work_id
        || left.observation_id == right.observation_id
        || left.quantity == right.quantity
    {
        return None;
    }

    Some(ObservationConflict {
        work_id: left.work_id,
        left_observation_id: left.observation_id,
        left_origin: left.origin,
        left_quantity: left.quantity,
        right_observation_id: right.observation_id,
        right_origin: right.origin,
        right_quantity: right.quantity,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReconciliationDecision {
    Agreement,
    ConflictPreserved(ObservationConflict),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResolutionDecision {
    AwaitingHumanDecision(ObservationConflict),
    ResolvedByExplicitDecision {
        conflict_work_id: &'static str,
        decision_ref: &'static str,
    },
}

pub fn resolve_conflict(
    reconciliation: ReconciliationDecision,
    decision_ref: Option<&'static str>,
) -> ResolutionDecision {
    let ReconciliationDecision::ConflictPreserved(conflict) = reconciliation else {
        return ResolutionDecision::AwaitingHumanDecision(ObservationConflict {
            work_id: "",
            left_observation_id: "",
            left_origin: FederationNode::Local,
            left_quantity: 0,
            right_observation_id: "",
            right_origin: FederationNode::Local,
            right_quantity: 0,
        });
    };

    match decision_ref {
        Some(reference) if !reference.is_empty() => ResolutionDecision::ResolvedByExplicitDecision {
            conflict_work_id: conflict.work_id,
            decision_ref: reference,
        },
        _ => ResolutionDecision::AwaitingHumanDecision(conflict),
    }
}

pub fn conflict_preserves_no_winner(decision: &ReconciliationDecision) -> bool {
    matches!(decision, ReconciliationDecision::ConflictPreserved(_))
}

pub fn reconcile_observation_set(
    observations: &[FederationObservation],
) -> ReconciliationDecision {
    for (index, left) in observations.iter().enumerate() {
        for right in observations.iter().skip(index + 1) {
            if let Some(conflict) = reconcile_observations(*left, *right) {
                return ReconciliationDecision::ConflictPreserved(conflict);
            }
        }
    }
    ReconciliationDecision::Agreement
}

pub fn reconciliation_is_order_invariant(
    observations: &[FederationObservation],
    reversed: &[FederationObservation],
) -> bool {
    reconcile_observation_set(observations) == reconcile_observation_set(reversed)
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
    if envelope.origin == FederationNode::Foreign && envelope.authority_origin == FederationNode::Local {
        // Explicit local recognition is allowed, but it never rewrites evidence origin.
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
    fn reconciliation_preserves_competing_observations() {
        let left = FederationObservation {
            observation_id: "obs-local",
            work_id: "work-1",
            origin: FederationNode::Local,
            quantity: 10,
            evidence_ref: "e-local",
            observed_at: 10,
        };
        let right = FederationObservation {
            observation_id: "obs-foreign",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            evidence_ref: "e-foreign",
            observed_at: 11,
        };
        let conflict = reconcile_observations(left, right).expect("conflict");
        assert_eq!(conflict.left_origin, FederationNode::Local);
        assert_eq!(conflict.right_origin, FederationNode::Foreign);
        assert_eq!(conflict.left_quantity, 10);
        assert_eq!(conflict.right_quantity, 12);
    }

    #[test]
    fn reconciliation_does_not_depend_on_delivery_order() {
        let left = FederationObservation {
            observation_id: "obs-a",
            work_id: "work-1",
            origin: FederationNode::Local,
            quantity: 10,
            evidence_ref: "e-a",
            observed_at: 10,
        };
        let right = FederationObservation {
            observation_id: "obs-b",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            evidence_ref: "e-b",
            observed_at: 11,
        };
        assert!(reconciliation_is_order_invariant(
            &[left, right],
            &[right, left]
        ));
        assert!(matches!(
            reconcile_observation_set(&[left, right]),
            ReconciliationDecision::ConflictPreserved(_)
        ));
    }

    #[test]
    fn conflict_reconciliation_has_no_implicit_winner() {
        let left = FederationObservation {
            observation_id: "obs-a",
            work_id: "work-1",
            origin: FederationNode::Local,
            quantity: 10,
            evidence_ref: "e-a",
            observed_at: 10,
        };
        let right = FederationObservation {
            observation_id: "obs-b",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            evidence_ref: "e-b",
            observed_at: 11,
        };
        assert!(conflict_preserves_no_winner(&reconcile_observation_set(&[left, right])));
    }

    #[test]
    fn agreement_does_not_create_a_resolution_artifact() {
        let observation = FederationObservation {
            observation_id: "obs-a",
            work_id: "work-1",
            origin: FederationNode::Local,
            quantity: 10,
            evidence_ref: "e-a",
            observed_at: 10,
        };
        assert!(matches!(
            resolve_conflict(
                reconcile_observation_set(&[observation]),
                Some("decision-should-not-resolve-agreement")
            ),
            ResolutionDecision::AwaitingHumanDecision(_)
        ));
    }

    #[test]
    fn conflict_requires_explicit_resolution() {
        let left = FederationObservation {
            observation_id: "obs-a",
            work_id: "work-1",
            origin: FederationNode::Local,
            quantity: 10,
            evidence_ref: "e-a",
            observed_at: 10,
        };
        let right = FederationObservation {
            observation_id: "obs-b",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            evidence_ref: "e-b",
            observed_at: 11,
        };
        let conflict = reconcile_observation_set(&[left, right]);
        assert!(matches!(
            resolve_conflict(conflict, None),
            ResolutionDecision::AwaitingHumanDecision(_)
        ));
        assert!(matches!(
            resolve_conflict(conflict, Some("decision-1")),
            ResolutionDecision::ResolvedByExplicitDecision { .. }
        ));
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

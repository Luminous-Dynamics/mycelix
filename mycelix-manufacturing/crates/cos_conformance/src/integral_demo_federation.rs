//! D6E deterministic heterogeneous federation reference model.
//!
//! Transport, storage, cryptography, and production networking are intentionally
//! outside this model. The oracle only answers whether a supplied federation
//! event preserves the declared identity, generation, origin, authorization,
//! expiry, replay, observation, and conflict boundaries.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_trace::{
    TraceActor, TraceEvent, TraceFixture, TraceKind, TraceRelation, TraceRelationRef,
    TraceStatus,
};

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
    /// Identity of the evidence artifact carried by the delivery; distinct from source_ref.
    pub evidence_ref: &'static str,
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
    RejectedMissingReference,
    RejectedOriginMutation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationReceipt {
    pub logical_delivery_id: &'static str,
    pub attempt_id: &'static str,
    pub origin: FederationNode,
    pub authority_origin: FederationNode,
    pub source_ref: &'static str,
    pub evidence_ref: &'static str,
    pub payload_digest: &'static str,
    pub state: DeliveryState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationObservation {
    pub observation_id: &'static str,
    pub work_id: &'static str,
    pub origin: FederationNode,
    pub quantity: u32,
    pub source_ref: &'static str,
    pub evidence_ref: &'static str,
    pub observed_at: u64,
}



#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvidenceBindingDecision {
    Bound,
    Replayed,
    RejectedDelivery,
    RejectedLogicalIdentity,
    RejectedOriginMutation,
    RejectedGenerationMismatch,
    RejectedSourceMutation,
    RejectedEvidenceMutation,
}

/// A source observation may be materialized from a federation delivery only when
/// the delivery itself has crossed the acceptance boundary. The binding keeps
/// logical delivery identity, source origin, schema generation, and source ref
/// visible rather than laundering transport provenance into local evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationObservationBinding {
    pub logical_delivery_id: &'static str,
    pub observation_id: &'static str,
    pub source_ref: &'static str,
    pub evidence_ref: &'static str,
    pub origin: FederationNode,
    pub schema_generation: u32,
    pub payload_digest: &'static str,
}

pub fn bind_delivery_to_observation(
    envelope: FederationEnvelope,
    observation: FederationObservation,
    current_generation: u32,
    now: u64,
    existing: Option<FederationReceipt>,
) -> EvidenceBindingDecision {
    let delivery_decision = accept_delivery(envelope, current_generation, now, existing);
    if !matches!(delivery_decision, FederationDecision::Accepted | FederationDecision::Replayed) {
        return EvidenceBindingDecision::RejectedDelivery;
    }
    if observation.observation_id.is_empty()
        || observation.work_id.is_empty()
        || observation.source_ref.is_empty()
        || observation.evidence_ref.is_empty()
    {
        return EvidenceBindingDecision::RejectedLogicalIdentity;
    }
    if observation.origin != envelope.origin {
        return EvidenceBindingDecision::RejectedOriginMutation;
    }
    if envelope.schema_generation != current_generation {
        return EvidenceBindingDecision::RejectedGenerationMismatch;
    }
    if observation.source_ref != envelope.source_ref {
        return EvidenceBindingDecision::RejectedSourceMutation;
    }
    if observation.evidence_ref != envelope.evidence_ref {
        return EvidenceBindingDecision::RejectedEvidenceMutation;
    }
    if matches!(delivery_decision, FederationDecision::Replayed) {
        EvidenceBindingDecision::Replayed
    } else {
        EvidenceBindingDecision::Bound
    }
}

pub fn observation_binding_for(
    envelope: FederationEnvelope,
    observation: FederationObservation,
    current_generation: u32,
    now: u64,
) -> Option<FederationObservationBinding> {
    if bind_delivery_to_observation(envelope, observation, current_generation, now, None)
        != EvidenceBindingDecision::Bound
    {
        return None;
    }
    Some(FederationObservationBinding {
        logical_delivery_id: envelope.logical_delivery_id,
        observation_id: observation.observation_id,
        source_ref: observation.source_ref,
        evidence_ref: observation.evidence_ref,
        origin: observation.origin,
        schema_generation: envelope.schema_generation,
        payload_digest: envelope.payload_digest,
    })
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

/// Stable identity for a conflict branch. Branch labels are navigational,
/// never a ranking or selection of either observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConflictBranch {
    pub branch_id: &'static str,
    pub work_id: &'static str,
    pub observation_id: &'static str,
    pub origin: FederationNode,
    pub quantity: u32,
}

/// Both conflicting observations remain represented as parallel branches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConflictBranchSet {
    pub conflict: ObservationConflict,
    pub left: ConflictBranch,
    pub right: ConflictBranch,
}

pub fn conflict_branches(conflict: ObservationConflict) -> Option<ConflictBranchSet> {
    if conflict.work_id.is_empty()
        || conflict.left_observation_id.is_empty()
        || conflict.right_observation_id.is_empty()
        || conflict.left_observation_id == conflict.right_observation_id
        || conflict.left_quantity == conflict.right_quantity
    {
        return None;
    }
    Some(ConflictBranchSet {
        conflict,
        left: ConflictBranch {
            branch_id: "branch:left",
            work_id: conflict.work_id,
            observation_id: conflict.left_observation_id,
            origin: conflict.left_origin,
            quantity: conflict.left_quantity,
        },
        right: ConflictBranch {
            branch_id: "branch:right",
            work_id: conflict.work_id,
            observation_id: conflict.right_observation_id,
            origin: conflict.right_origin,
            quantity: conflict.right_quantity,
        },
    })
}

/// Reconciliation records a human disposition and points to a newer revision.
/// It does not mutate or remove either source observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConflictReconciliation {
    pub reconciliation_id: &'static str,
    pub decision_id: &'static str,
    pub revision_id: &'static str,
    pub revision_generation: u32,
}

pub fn validate_conflict_reconciliation(
    branches: &ConflictBranchSet,
    decision: &FederationDecisionArtifact,
    reconciliation: &ConflictReconciliation,
    current_generation: u32,
) -> bool {
    !reconciliation.reconciliation_id.is_empty()
        && !reconciliation.revision_id.is_empty()
        && reconciliation.reconciliation_id != reconciliation.revision_id
        && reconciliation.reconciliation_id != decision.decision_id
        && reconciliation.revision_id != decision.decision_id
        && reconciliation.decision_id == decision.decision_id
        && validate_decision_artifact(decision, &branches.conflict, current_generation)
        && reconciliation.revision_generation > current_generation
        && branches.left.observation_id == branches.conflict.left_observation_id
        && branches.right.observation_id == branches.conflict.right_observation_id
        && branches.left.work_id == branches.right.work_id
        && branches.left.observation_id != branches.right.observation_id
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

    let (left, right) = if left.observation_id <= right.observation_id {
        (left, right)
    } else {
        (right, left)
    };

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
    NoConflict,
    AwaitingHumanDecision(ObservationConflict),
    ResolvedByExplicitDecision {
        conflict_work_id: &'static str,
        decision_ref: &'static str,
    },
}

/// A governance decision is an explicit human artifact. It is not an observation,
/// recommendation, or federation receipt, and it cannot rewrite either observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationDecisionArtifact {
    pub decision_id: &'static str,
    pub conflict_work_id: &'static str,
    pub left_observation_id: &'static str,
    pub right_observation_id: &'static str,
    pub actor: &'static str,
    pub authority_ref: &'static str,
    pub generation: u32,
    pub decided_at: u64,
    pub accepted: bool,
    pub source_ref: &'static str,
    pub recommendation_ref: Option<&'static str>,
}

/// Project the governance artifact into the D5 trace without changing its provenance.
pub fn decision_artifact_trace(
    artifact: &FederationDecisionArtifact,
    sequence: u32,
) -> (TraceEvent, Vec<TraceRelationRef>) {
    let event = TraceEvent {
        event_id: artifact.decision_id,
        sequence,
        kind: TraceKind::HumanDecision,
        provenance: crate::integral_demo_domain::ProvenanceClass::Decision,
        actor: TraceActor::Human,
        source: crate::integral_demo_domain::SourceKind::Local,
        source_ref: artifact.source_ref,
        evidence_ref: None,
        generation: artifact.generation,
        uncertainty_present: false,
        authority_ref: Some(artifact.authority_ref),
        reversible: true,
        challengeable: true,
        recommendation_only: false,
        recovery_ref: None,
        appeal_ref: None,
        decision_accepted: Some(artifact.accepted),
        status: if artifact.accepted {
            TraceStatus::Accepted
        } else {
            TraceStatus::Rejected
        },
    };
    let mut relations = vec![
        TraceRelationRef {
            from_event: artifact.decision_id,
            to_event: artifact.left_observation_id,
            relation: TraceRelation::RespondsTo,
        },
        TraceRelationRef {
            from_event: artifact.decision_id,
            to_event: artifact.right_observation_id,
            relation: TraceRelation::RespondsTo,
        },
    ];
    if let Some(recommendation_ref) = artifact.recommendation_ref {
        if !recommendation_ref.is_empty() {
            relations.push(TraceRelationRef {
                from_event: artifact.decision_id,
                to_event: recommendation_ref,
                relation: TraceRelation::RespondsTo,
            });
        }
    }
    (event, relations)
}

pub fn validate_decision_artifact(
    artifact: &FederationDecisionArtifact,
    conflict: &ObservationConflict,
    current_generation: u32,
) -> bool {
    !artifact.decision_id.is_empty()
        && !artifact.actor.is_empty()
        && !artifact.authority_ref.is_empty()
        && !artifact.source_ref.is_empty()
        && artifact.conflict_work_id == conflict.work_id
        && artifact.left_observation_id == conflict.left_observation_id
        && artifact.right_observation_id == conflict.right_observation_id
        && artifact.generation == current_generation
}

pub fn resolve_with_decision_artifact(
    reconciliation: ReconciliationDecision,
    artifact: Option<&FederationDecisionArtifact>,
    current_generation: u32,
) -> ResolutionDecision {
    let ReconciliationDecision::ConflictPreserved(conflict) = reconciliation else {
        return ResolutionDecision::NoConflict;
    };
    let Some(artifact) = artifact else {
        return ResolutionDecision::AwaitingHumanDecision(conflict);
    };
    if validate_decision_artifact(artifact, &conflict, current_generation) {
        ResolutionDecision::ResolvedByExplicitDecision {
            conflict_work_id: conflict.work_id,
            decision_ref: artifact.decision_id,
        }
    } else {
        ResolutionDecision::AwaitingHumanDecision(conflict)
    }
}

pub fn resolve_conflict(
    reconciliation: ReconciliationDecision,
    decision_ref: Option<&'static str>,
) -> ResolutionDecision {
    let ReconciliationDecision::ConflictPreserved(conflict) = reconciliation else {
        return ResolutionDecision::NoConflict;
    };

    match decision_ref {
        Some(reference) if !reference.is_empty() => {
            ResolutionDecision::ResolvedByExplicitDecision {
                conflict_work_id: conflict.work_id,
                decision_ref: reference,
            }
        }
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

/// A deterministic observation state reconstructed from an append-only event history.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationObservationState {
    pub observations: Vec<FederationObservation>,
    pub conflicts: Vec<ObservationConflict>,
}

impl FederationObservationState {
    pub fn empty() -> Self {
        Self {
            observations: Vec::new(),
            conflicts: Vec::new(),
        }
    }

    /// Rebuild the evidence state without choosing a winner or applying governance.
    pub fn replay(observations: &[FederationObservation]) -> Self {
        let mut ordered = observations.to_vec();
        ordered.sort_by_key(|observation| observation.observation_id);
        ordered.dedup_by_key(|observation| observation.observation_id);

        let mut conflicts = Vec::new();
        for (index, left) in ordered.iter().enumerate() {
            for right in ordered.iter().skip(index + 1) {
                if let Some(conflict) = reconcile_observations(*left, *right) {
                    conflicts.push(conflict);
                }
            }
        }
        conflicts.sort_by_key(|conflict| {
            (
                conflict.work_id,
                conflict.left_observation_id,
                conflict.right_observation_id,
            )
        });

        Self {
            observations: ordered,
            conflicts,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservationAppendDecision {
    Appended,
    Replayed,
    RejectedDuplicateMutation,
}

/// Append-only observation history. Exact duplicate events are replayable; the same
/// observation identity with changed evidence is a mutation and fails closed.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct FederationObservationLog {
    pub observations: Vec<FederationObservation>,
}

impl FederationObservationLog {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn append(&mut self, observation: FederationObservation) -> ObservationAppendDecision {
        if let Some(existing) = self
            .observations
            .iter()
            .find(|existing| existing.observation_id == observation.observation_id)
        {
            return if *existing == observation {
                ObservationAppendDecision::Replayed
            } else {
                ObservationAppendDecision::RejectedDuplicateMutation
            };
        }

        self.observations.push(observation);
        self.observations
            .sort_by_key(|item| item.observation_id);
        ObservationAppendDecision::Appended
    }

    pub fn replay(&self) -> FederationObservationState {
        FederationObservationState::replay(&self.observations)
    }
}

/// A deterministic, append-only reference log for federation deliveries.
/// Replay is pure: it never selects a winner or mutates an observation payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationLog {
    pub deliveries: Vec<FederationEnvelope>,
}

impl FederationLog {
    pub fn new() -> Self {
        Self {
            deliveries: Vec::new(),
        }
    }

    pub fn append(&mut self, envelope: FederationEnvelope) -> bool {
        if self.deliveries.iter().any(|existing| {
            existing.logical_delivery_id == envelope.logical_delivery_id
                && existing.attempt_id == envelope.attempt_id
        }) {
            return false;
        }
        self.deliveries.push(envelope);
        self.deliveries
            .sort_by_key(|event| (event.logical_delivery_id, event.attempt_id, event.observed_at));
        true
    }

    pub fn replay(&self, current_generation: u32, now: u64) -> Vec<FederationDecision> {
        let mut ordered = self.deliveries.clone();
        ordered.sort_by_key(|event| {
            (event.logical_delivery_id, event.attempt_id, event.observed_at)
        });

        let mut receipts: Vec<FederationReceipt> = Vec::new();
        ordered
            .iter()
            .map(|envelope| {
                let prior = ordered.iter().find(|candidate| {
                    candidate.logical_delivery_id == envelope.logical_delivery_id
                        && (candidate.attempt_id, candidate.observed_at)
                            < (envelope.attempt_id, envelope.observed_at)
                });

                // Logical delivery identity is canonical across attempts. A mutation must
                // fail regardless of which retry arrived first.
                if ordered.iter().any(|candidate| {
                    candidate.logical_delivery_id == envelope.logical_delivery_id
                        && (candidate.origin != envelope.origin
                            || candidate.authority_origin != envelope.authority_origin
                            || candidate.source_ref != envelope.source_ref
                            || candidate.evidence_ref != envelope.evidence_ref
                            || candidate.payload_digest != envelope.payload_digest)
                }) {
                    return if ordered.iter().any(|candidate| {
                        candidate.logical_delivery_id == envelope.logical_delivery_id
                            && candidate.origin != envelope.origin
                    }) {
                        FederationDecision::RejectedOriginMutation
                    } else {
                        FederationDecision::RejectedDuplicateMutation
                    };
                }

                let existing = receipts
                    .iter()
                    .find(|receipt| {
                        receipt.logical_delivery_id == envelope.logical_delivery_id
                    })
                    .copied();
                let decision = accept_delivery(*envelope, current_generation, now, existing);
                if matches!(decision, FederationDecision::Accepted) {
                    receipts.push(receipt_for(*envelope));
                }
                decision
            })
            .collect()
    }
}

impl Default for FederationLog {
    fn default() -> Self {
        Self::new()
    }
}

pub fn replay_is_deterministic(log: &FederationLog, generation: u32, now: u64) -> bool {
    let mut permuted = log.clone();
    permuted.deliveries.reverse();
    log.replay(generation, now) == permuted.replay(generation, now)
}

pub fn reconciliation_is_order_invariant(
    observations: &[FederationObservation],
    reversed: &[FederationObservation],
) -> bool {
    FederationObservationState::replay(observations)
        == FederationObservationState::replay(reversed)
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
    if envelope.source_ref.is_empty() || envelope.evidence_ref.is_empty() {
        return FederationDecision::RejectedMissingReference;
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
        if receipt.authority_origin != envelope.authority_origin {
            return FederationDecision::RejectedForeignAuthority;
        }
        if receipt.source_ref != envelope.source_ref {
            return FederationDecision::RejectedDuplicateMutation;
        }
        if receipt.evidence_ref != envelope.evidence_ref {
            return FederationDecision::RejectedDuplicateMutation;
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
        authority_origin: envelope.authority_origin,
        source_ref: envelope.source_ref,
        evidence_ref: envelope.evidence_ref,
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
        && first.source_ref == retry.source_ref
        && first.evidence_ref == retry.evidence_ref
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
            source_ref: "node://foreign/observation/1",
            evidence_ref: "evidence://foreign/1",
            schema_generation: 7,
            payload_digest: "digest-1",
            state: DeliveryState::Delivered,
            authorization: AuthorizationState::Active,
            expires_at: 100,
            observed_at: 10,
            privacy_minimized: false,
        }
    }

    fn observations() -> [FederationObservation; 3] {
        [
            FederationObservation {
                observation_id: "obs-a",
                work_id: "work-1",
                origin: FederationNode::Local,
                quantity: 10,
                source_ref: "e-a",
                evidence_ref: "e-a",
                observed_at: 10,
            },
            FederationObservation {
                observation_id: "obs-b",
                work_id: "work-1",
                origin: FederationNode::Foreign,
                quantity: 12,
                source_ref: "e-b",
                evidence_ref: "e-b",
                observed_at: 11,
            },
            FederationObservation {
                observation_id: "obs-c",
                work_id: "work-2",
                origin: FederationNode::Foreign,
                quantity: 4,
                source_ref: "e-c",
                evidence_ref: "e-c",
                observed_at: 12,
            },
        ]
    }

    #[test]
    fn federation_log_deduplicates_exact_attempts_and_replays_deterministically() {
        let mut log = FederationLog::new();
        let e = envelope();
        assert!(log.append(e));
        assert!(!log.append(e));
        let mut retry = e;
        retry.attempt_id = "attempt-2";
        assert!(log.append(retry));
        assert!(replay_is_deterministic(&log, 7, 20));
        assert_eq!(log.replay(7, 20)[0], FederationDecision::Accepted);
        assert_eq!(log.replay(7, 20)[1], FederationDecision::Replayed);
    }

    #[test]
    fn federation_replay_rejects_mutation_even_when_mutated_attempt_sorts_first() {
        let mut log = FederationLog::new();
        let mut valid = envelope();
        valid.attempt_id = "attempt-z";
        let mut mutated = envelope();
        mutated.attempt_id = "attempt-a";
        mutated.payload_digest = "digest-mutated";

        assert!(log.append(valid));
        assert!(log.append(mutated));

        let replay = log.replay(7, 20);
        assert_eq!(replay.len(), 2);
        assert_eq!(replay[0], FederationDecision::RejectedDuplicateMutation);
        assert_eq!(replay[1], FederationDecision::RejectedDuplicateMutation);
    }

    #[test]
    fn accepted_delivery_binds_observation_without_laundering_origin() {
        let e = envelope();
        let observation = FederationObservation {
            observation_id: "obs-foreign",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            source_ref: e.source_ref,
                evidence_ref: e.evidence_ref,
            observed_at: e.observed_at,
        };
        assert_eq!(
            bind_delivery_to_observation(e, observation, 7, 20, None),
            EvidenceBindingDecision::Bound
        );
        let binding = observation_binding_for(e, observation, 7, 20).expect("binding");
        assert_eq!(binding.logical_delivery_id, "delivery-1");
        assert_eq!(binding.origin, FederationNode::Foreign);
        assert_eq!(binding.source_ref, e.source_ref);
    }

    #[test]
    fn missing_source_or_evidence_reference_fails_closed() {
        let mut missing_source = envelope();
        missing_source.source_ref = "";
        assert_eq!(
            accept_delivery(missing_source, 7, 20, None),
            FederationDecision::RejectedMissingReference
        );

        let mut missing_evidence = envelope();
        missing_evidence.evidence_ref = "";
        assert_eq!(
            accept_delivery(missing_evidence, 7, 20, None),
            FederationDecision::RejectedMissingReference
        );
    }

    #[test]
    fn source_and_evidence_bindings_are_independently_conserved() {
        let e = envelope();
        let observation = FederationObservation {
            observation_id: "obs-distinct-binding",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            source_ref: e.source_ref,
            evidence_ref: e.evidence_ref,
            observed_at: e.observed_at,
        };
        assert_eq!(bind_delivery_to_observation(e, observation, 7, 20, None), EvidenceBindingDecision::Bound);

        let mut source_mutation = observation;
        source_mutation.source_ref = "source://tampered";
        assert_eq!(bind_delivery_to_observation(e, source_mutation, 7, 20, None), EvidenceBindingDecision::RejectedSourceMutation);

        let mut evidence_mutation = observation;
        evidence_mutation.evidence_ref = "evidence://tampered";
        assert_eq!(bind_delivery_to_observation(e, evidence_mutation, 7, 20, None), EvidenceBindingDecision::RejectedEvidenceMutation);
    }

    #[test]
    fn observation_cannot_mutate_delivery_origin_or_source() {
        let e = envelope();
        let mut observation = FederationObservation {
            observation_id: "obs-foreign",
            work_id: "work-1",
            origin: FederationNode::Local,
            quantity: 12,
            source_ref: e.source_ref,
                evidence_ref: e.evidence_ref,
            observed_at: e.observed_at,
        };
        assert_eq!(
            bind_delivery_to_observation(e, observation, 7, 20, None),
            EvidenceBindingDecision::RejectedOriginMutation
        );
        observation.origin = FederationNode::Foreign;
        observation.evidence_ref = "different-source";
        assert_eq!(
            bind_delivery_to_observation(e, observation, 7, 20, None),
            EvidenceBindingDecision::RejectedSourceMutation
        );
    }

    #[test]
    fn stale_or_partitioned_delivery_cannot_materialize_observation() {
        let mut stale = envelope();
        stale.schema_generation = 6;
        let observation = FederationObservation {
            observation_id: "obs-stale",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            source_ref: stale.source_ref,
                evidence_ref: stale.evidence_ref,
            observed_at: stale.observed_at,
        };
        assert_eq!(
            bind_delivery_to_observation(stale, observation, 7, 20, None),
            EvidenceBindingDecision::RejectedDelivery
        );

        let mut partitioned = envelope();
        partitioned.state = DeliveryState::Partitioned;
        let observation = FederationObservation {
            observation_id: "obs-partitioned",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            source_ref: partitioned.source_ref,
                evidence_ref: partitioned.evidence_ref,
            observed_at: partitioned.observed_at,
        };
        assert_eq!(
            bind_delivery_to_observation(partitioned, observation, 7, 20, None),
            EvidenceBindingDecision::RejectedDelivery
        );
    }

    #[test]
    fn replayed_delivery_replays_observation_binding() {
        let e = envelope();
        let receipt = receipt_for(e);
        let observation = FederationObservation {
            observation_id: "obs-replay",
            work_id: "work-1",
            origin: FederationNode::Foreign,
            quantity: 12,
            source_ref: e.source_ref,
                evidence_ref: e.evidence_ref,
            observed_at: e.observed_at,
        };
        assert_eq!(
            bind_delivery_to_observation(e, observation, 7, 20, Some(receipt)),
            EvidenceBindingDecision::Replayed
        );
    }

    #[test]
    fn foreign_delivery_is_accepted_without_becoming_local() {
        assert_eq!(
            accept_delivery(envelope(), 7, 20, None),
            FederationDecision::Accepted
        );
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
        assert!(attempt_may_change_without_mutating_logical_delivery(
            envelope(),
            retry
        ));
    }

    #[test]
    fn duplicate_identical_delivery_replays() {
        let e = envelope();
        let receipt = receipt_for(e);
        assert_eq!(
            accept_delivery(e, 7, 20, Some(receipt)),
            FederationDecision::Replayed
        );
    }

    #[test]
    fn reconnect_preserves_origin_and_logical_identity() {
        let e = envelope();
        let mut retry = e;
        retry.attempt_id = "attempt-2";
        assert!(attempt_may_change_without_mutating_logical_delivery(e, retry));
        assert_eq!(
            receipt_for(e).logical_delivery_id,
            receipt_for(retry).logical_delivery_id
        );
        assert_eq!(receipt_for(e).origin, FederationNode::Foreign);
    }

    #[test]
    fn conflict_branches_preserve_both_observations_without_preference() {
        let [left, right, _] = observations();
        let conflict = reconcile_observations(left, right).expect("conflict");
        let branches = conflict_branches(conflict).expect("branches");
        assert_eq!(branches.left.observation_id, conflict.left_observation_id);
        assert_eq!(branches.right.observation_id, conflict.right_observation_id);
        assert_ne!(branches.left.observation_id, branches.right.observation_id);
        assert_eq!(branches.left.work_id, branches.right.work_id);
        assert_eq!(branches.left.quantity, conflict.left_quantity);
        assert_eq!(branches.right.quantity, conflict.right_quantity);
    }

    #[test]
    fn reconciliation_requires_explicit_decision_and_new_revision() {
        let [left, right, _] = observations();
        let conflict = reconcile_observations(left, right).expect("conflict");
        let branches = conflict_branches(conflict).expect("branches");
        let decision = FederationDecisionArtifact {
            decision_id: "decision-conflict-1",
            conflict_work_id: conflict.work_id,
            left_observation_id: conflict.left_observation_id,
            right_observation_id: conflict.right_observation_id,
            actor: "human-1",
            authority_ref: "authority://review",
            generation: 7,
            decided_at: 120,
            accepted: true,
            source_ref: "decision://source",
            recommendation_ref: None,
        };
        let path = ConflictReconciliation {
            reconciliation_id: "reconciliation-1",
            decision_id: decision.decision_id,
            revision_id: "revision-8",
            revision_generation: 8,
        };
        assert!(validate_conflict_reconciliation(&branches, &decision, &path, 7));
        let stale = ConflictReconciliation { revision_generation: 7, ..path };
        assert!(!validate_conflict_reconciliation(&branches, &decision, &stale, 7));
        let wrong_branch = FederationDecisionArtifact {
            left_observation_id: "other-observation",
            ..decision
        };
        assert!(!validate_conflict_reconciliation(&branches, &wrong_branch, &path, 7));
    }

    #[test]
    fn reconciliation_preserves_competing_observations() {
        let [left, right, _] = observations();
        let conflict = reconcile_observations(left, right).expect("conflict");
        assert_eq!(conflict.left_origin, FederationNode::Local);
        assert_eq!(conflict.right_origin, FederationNode::Foreign);
        assert_eq!(conflict.left_quantity, 10);
        assert_eq!(conflict.right_quantity, 12);
    }

    #[test]
    fn observation_event_log_rebuilds_all_conflicts_without_winner_selection() {
        let observations = observations();
        let mut log = FederationObservationLog::new();
        assert_eq!(
            log.append(observations[2]),
            ObservationAppendDecision::Appended
        );
        assert_eq!(
            log.append(observations[1]),
            ObservationAppendDecision::Appended
        );
        assert_eq!(
            log.append(observations[0]),
            ObservationAppendDecision::Appended
        );

        let state = log.replay();
        assert_eq!(state.observations.len(), 3);
        assert_eq!(state.conflicts.len(), 1);
        assert_eq!(state.conflicts[0].left_observation_id, "obs-a");
        assert_eq!(state.conflicts[0].right_observation_id, "obs-b");
    }

    #[test]
    fn exact_observation_replay_is_idempotent_but_mutation_fails_closed() {
        let observation = observations()[0];
        let mut log = FederationObservationLog::new();
        assert_eq!(
            log.append(observation),
            ObservationAppendDecision::Appended
        );
        assert_eq!(
            log.append(observation),
            ObservationAppendDecision::Replayed
        );

        let mut mutated = observation;
        mutated.quantity = 99;
        assert_eq!(
            log.append(mutated),
            ObservationAppendDecision::RejectedDuplicateMutation
        );
        assert_eq!(log.replay().observations[0].quantity, 10);
    }

    #[test]
    fn observation_replay_is_canonical_across_delivery_order() {
        let source = observations();
        let order_a = [source[0], source[1], source[2]];
        let order_b = [source[2], source[0], source[1]];
        let order_c = [source[1], source[2], source[0]];

        assert_eq!(
            FederationObservationState::replay(&order_a),
            FederationObservationState::replay(&order_b)
        );
        assert_eq!(
            FederationObservationState::replay(&order_b),
            FederationObservationState::replay(&order_c)
        );
        assert!(reconciliation_is_order_invariant(&order_a, &order_b));
    }

    #[test]
    fn duplicated_observation_events_do_not_change_reconstructed_state() {
        let source = observations();
        let duplicated = [
            source[1],
            source[0],
            source[1],
            source[2],
            source[0],
        ];
        assert_eq!(
            FederationObservationState::replay(&source),
            FederationObservationState::replay(&duplicated)
        );
    }

    #[test]
    fn conflict_reconciliation_has_no_implicit_winner() {
        let [left, right, _] = observations();
        let decision = reconcile_observation_set(&[left, right]);
        assert!(conflict_preserves_no_winner(&decision));
    }

    #[test]
    fn agreement_is_distinct_from_unresolved_conflict() {
        let observation = observations()[0];
        assert_eq!(
            resolve_conflict(
                reconcile_observation_set(&[observation]),
                Some("decision")
            ),
            ResolutionDecision::NoConflict
        );
    }

    #[test]
    fn agreement_does_not_create_a_resolution_artifact() {
        let observation = observations()[0];
        assert_eq!(
            resolve_conflict(
                reconcile_observation_set(&[observation]),
                Some("decision-should-not-resolve-agreement")
            ),
            ResolutionDecision::NoConflict
        );
    }

    #[test]
    fn decision_artifact_must_target_exact_conflict() {
        let [left, right, _] = observations();
        let reconciliation = reconcile_observation_set(&[left, right]);
        let conflict = match reconciliation {
            ReconciliationDecision::ConflictPreserved(c) => c,
            _ => panic!("expected conflict"),
        };
        let artifact = FederationDecisionArtifact {
            decision_id: "decision-1",
            conflict_work_id: "work-1",
            left_observation_id: "obs-a",
            right_observation_id: "wrong",
            actor: "human-1",
            authority_ref: "auth-1",
            generation: 7,
            decided_at: 20,
            accepted: true,
            source_ref: "decision://1",
            recommendation_ref: None,
        };
        assert!(matches!(
            resolve_with_decision_artifact(reconciliation, Some(&artifact), 7),
            ResolutionDecision::AwaitingHumanDecision(_)
        ));
        assert!(!validate_decision_artifact(&artifact, &conflict, 7));
    }

    #[test]
    fn decision_artifact_can_resolve_only_matching_current_conflict() {
        let [left, right, _] = observations();
        let reconciliation = reconcile_observation_set(&[left, right]);
        let artifact = FederationDecisionArtifact {
            decision_id: "decision-1",
            conflict_work_id: "work-1",
            left_observation_id: "obs-a",
            right_observation_id: "obs-b",
            actor: "human-1",
            authority_ref: "auth-1",
            generation: 7,
            decided_at: 20,
            accepted: true,
            source_ref: "decision://1",
            recommendation_ref: None,
        };
        assert!(matches!(
            resolve_with_decision_artifact(reconciliation, Some(&artifact), 7),
            ResolutionDecision::ResolvedByExplicitDecision {
                decision_ref: "decision-1",
                ..
            }
        ));
    }

    #[test]
    fn stale_decision_artifact_cannot_resolve_current_conflict() {
        let [left, right, _] = observations();
        let reconciliation = reconcile_observation_set(&[left, right]);
        let artifact = FederationDecisionArtifact {
            decision_id: "decision-1",
            conflict_work_id: "work-1",
            left_observation_id: "obs-a",
            right_observation_id: "obs-b",
            actor: "human-1",
            authority_ref: "auth-1",
            generation: 6,
            decided_at: 20,
            accepted: true,
            source_ref: "decision://1",
            recommendation_ref: None,
        };
        assert!(matches!(
            resolve_with_decision_artifact(reconciliation, Some(&artifact), 7),
            ResolutionDecision::AwaitingHumanDecision(_)
        ));
    }

    #[test]
    fn conflict_requires_explicit_resolution() {
        let [left, right, _] = observations();
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
    fn governance_decision_projects_into_trace_without_laundering_foreign_observation() {
        let artifact = FederationDecisionArtifact {
            decision_id: "decision-2",
            conflict_work_id: "work-1",
            left_observation_id: "obs-a",
            right_observation_id: "obs-b",
            actor: "human-1",
            authority_ref: "auth-2",
            generation: 7,
            decided_at: 20,
            accepted: true,
            source_ref: "decision://2",
            recommendation_ref: Some("recommendation-1"),
        };
        let (decision, mut relations) = decision_artifact_trace(&artifact, 3);
        assert_eq!(decision.kind, TraceKind::HumanDecision);
        assert_eq!(decision.actor, TraceActor::Human);
        assert_eq!(decision.authority_ref, Some("auth-2"));
        assert_eq!(decision.decision_accepted, Some(true));
        assert_eq!(decision.status, TraceStatus::Accepted);

        relations.push(TraceRelationRef {
            from_event: "obs-b",
            to_event: "obs-a",
            relation: TraceRelation::Disputes,
        });
        let fixture = TraceFixture {
            events: vec![
                TraceEvent {
                    event_id: "obs-a",
                    sequence: 1,
                    kind: TraceKind::Observation,
                    provenance: crate::integral_demo_domain::ProvenanceClass::Observation,
                    actor: TraceActor::System,
                    source: crate::integral_demo_domain::SourceKind::Local,
                    source_ref: "evidence://local",
                    evidence_ref: Some("evidence://local"),
                    generation: 7,
                    uncertainty_present: true,
                    authority_ref: None,
                    reversible: true,
                    challengeable: true,
                    recommendation_only: false,
                    recovery_ref: None,
                    appeal_ref: None,
                    decision_accepted: None,
                    status: TraceStatus::Accepted,
                },
                TraceEvent {
                    event_id: "obs-b",
                    sequence: 2,
                    kind: TraceKind::Observation,
                    provenance: crate::integral_demo_domain::ProvenanceClass::Observation,
                    actor: TraceActor::System,
                    source: crate::integral_demo_domain::SourceKind::Foreign,
                    source_ref: "evidence://foreign",
                    evidence_ref: Some("evidence://foreign"),
                    generation: 7,
                    uncertainty_present: true,
                    authority_ref: None,
                    reversible: true,
                    challengeable: true,
                    recommendation_only: false,
                    recovery_ref: None,
                    appeal_ref: None,
                    decision_accepted: None,
                    status: TraceStatus::Disputed,
                },
                decision,
            ],
            relations,
        };
        assert_eq!(crate::integral_demo_trace::validate_trace(&fixture), Ok(()));
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
        assert_eq!(
            accept_delivery(envelope(), 8, 20, None),
            FederationDecision::RejectedStaleGeneration
        );
    }

    #[test]
    fn expired_authorization_cannot_revive_on_reconnect() {
        let mut e = envelope();
        e.authorization = AuthorizationState::Expired;
        assert!(reconnect_cannot_revive_expired_authorization(e, 20));
        assert_eq!(
            reconcile(e, 7, 20, None),
            FederationDecision::RejectedExpiredAuthorization
        );
    }

    #[test]
    fn partition_does_not_fabricate_delivery() {
        let mut e = envelope();
        e.state = DeliveryState::Partitioned;
        assert_eq!(
            accept_delivery(e, 7, 20, None),
            FederationDecision::RejectedPartitioned
        );
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

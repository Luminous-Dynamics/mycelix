//! Deterministic heterogeneous federation reference model for Integral/Mycelix.
//!
//! This module is intentionally transport-neutral. It models semantic delivery,
//! provenance, generation, authorization, reconciliation, and privacy projection.
//! A runtime adapter may realize the same events over Holochain, HTTP, a queue,
//! or another transport without changing the oracle's expected outcomes.
//
//! Claim ceiling: ReferenceModelOnly.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const FEDERATION_PROFILE_ID: &str = "INTEGRAL-FED-REF-001";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NodeProfile {
    pub node_id: String,
    pub schema_generation: u64,
    pub authorization_generation: u64,
    pub active: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorizationState {
    Active,
    Expired,
    Revoked,
    Absent,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum RecognitionMode {
    EvidenceOnly,
    DelegatedAuthority,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecognitionEdge {
    pub recognizing_node: String,
    pub origin_node: String,
    pub scope: String,
    pub mode: RecognitionMode,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FederationEnvelope {
    pub envelope_id: String,
    pub semantic_subject_id: String,
    pub payload_commitment: String,
    pub logical_delivery_id: String,
    pub attempt_id: String,
    pub origin_node: String,
    pub target_node: String,
    pub schema_generation: u64,
    pub authorization_generation: u64,
    pub authorization: AuthorizationState,
    pub expires_at: Option<u64>,
    pub predecessor_delivery_id: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationRecord {
    pub observation_id: String,
    pub semantic_subject_id: String,
    pub payload_commitment: String,
    pub origin_node: String,
    pub recognized_by: Option<String>,
    pub source_observation: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeliveryRecord {
    pub logical_delivery_id: String,
    pub semantic_subject_id: String,
    pub payload_commitment: String,
    pub origin_node: String,
    pub authority: AuthorityDisposition,
    pub attempts: BTreeSet<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FederationState {
    pub nodes: BTreeMap<String, NodeProfile>,
    pub recognition_edges: Vec<RecognitionEdge>,
    pub deliveries: BTreeMap<String, DeliveryRecord>,
    pub observations: BTreeMap<String, ObservationRecord>,
}

impl FederationState {
    pub fn new(nodes: impl IntoIterator<Item = NodeProfile>) -> Self {
        let nodes = nodes
            .into_iter()
            .map(|node| (node.node_id.clone(), node))
            .collect();
        Self {
            nodes,
            recognition_edges: Vec::new(),
            deliveries: BTreeMap::new(),
            observations: BTreeMap::new(),
        }
    }

    pub fn add_recognition(&mut self, edge: RecognitionEdge) {
        self.recognition_edges.push(edge);
        self.recognition_edges.sort_by(|a, b| {
            (
                &a.recognizing_node,
                &a.origin_node,
                &a.scope,
                a.mode,
            )
                .cmp(&(
                    &b.recognizing_node,
                    &b.origin_node,
                    &b.scope,
                    b.mode,
                ))
        });
    }

    fn recognition_mode(
        &self,
        recognizing_node: &str,
        origin_node: &str,
        scope: &str,
    ) -> Result<Option<RecognitionMode>, ()> {
        let mut modes = self
            .recognition_edges
            .iter()
            .filter(|edge| {
                edge.recognizing_node == recognizing_node
                    && edge.origin_node == origin_node
                    && edge.scope == scope
            })
            .map(|edge| edge.mode)
            .collect::<BTreeSet<_>>();
        if modes.len() > 1 {
            return Err(());
        }
        Ok(modes.iter().next().copied())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FederationDecision {
    AcceptedLocal,
    AcceptedForeign,
    AcceptedDelegatedForeign,
    Duplicate,
    PendingDependency,
    PartitionUnknown,
    StaleGeneration,
    ExpiredAuthorization,
    Unauthorized,
    PayloadConflict,
    OriginConflict,
    UnknownNode,
    RecognitionConflict,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorityDisposition {
    LocalAuthority,
    ForeignEvidence,
    RecognizedForeignEvidence,
    ExplicitDelegatedAuthority,
    NoAuthority,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FederationOutcome {
    pub decision: FederationDecision,
    pub authority: AuthorityDisposition,
    pub origin_node: Option<String>,
    pub logical_delivery_id: String,
    pub attempt_id: String,
    pub reason: &'static str,
}

impl FederationOutcome {
    fn new(
        decision: FederationDecision,
        authority: AuthorityDisposition,
        envelope: &FederationEnvelope,
        reason: &'static str,
    ) -> Self {
        Self {
            decision,
            authority,
            origin_node: Some(envelope.origin_node.clone()),
            logical_delivery_id: envelope.logical_delivery_id.clone(),
            attempt_id: envelope.attempt_id.clone(),
            reason,
        }
    }
}

pub fn deliver(
    state: &mut FederationState,
    envelope: &FederationEnvelope,
    now: u64,
    transport_available: bool,
) -> FederationOutcome {
    if !transport_available {
        return FederationOutcome::new(
            FederationDecision::PartitionUnknown,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Transport unavailability does not establish delivery success.",
        );
    }

    let Some(origin) = state.nodes.get(&envelope.origin_node) else {
        return FederationOutcome::new(
            FederationDecision::UnknownNode,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Origin node is not known to the reference federation.",
        );
    };
    let Some(target) = state.nodes.get(&envelope.target_node) else {
        return FederationOutcome::new(
            FederationDecision::UnknownNode,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Target node is not known to the reference federation.",
        );
    };

    if !origin.active || !target.active {
        return FederationOutcome::new(
            FederationDecision::Unauthorized,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Inactive federation nodes cannot establish an authoritative delivery.",
        );
    }

    if envelope.schema_generation != target.schema_generation {
        return FederationOutcome::new(
            FederationDecision::StaleGeneration,
            AuthorityDisposition::NoAuthority,
            envelope,
            "A stale schema generation cannot be admitted.",
        );
    }

    if envelope.authorization_generation != target.authorization_generation {
        return FederationOutcome::new(
            FederationDecision::StaleGeneration,
            AuthorityDisposition::NoAuthority,
            envelope,
            "A stale authorization generation cannot be revived by reconnect.",
        );
    }

    if envelope.authorization != AuthorizationState::Active {
        let decision = match envelope.authorization {
            AuthorizationState::Expired => FederationDecision::ExpiredAuthorization,
            AuthorizationState::Revoked | AuthorizationState::Absent => FederationDecision::Unauthorized,
            AuthorizationState::Active => unreachable!(),
        };
        return FederationOutcome::new(
            decision,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Authorization state is not active.",
        );
    }

    if envelope.expires_at.is_some_and(|expiry| now > expiry) {
        return FederationOutcome::new(
            FederationDecision::ExpiredAuthorization,
            AuthorityDisposition::NoAuthority,
            envelope,
            "An expired authorization cannot be revived by reconnect.",
        );
    }

    if let Some(predecessor) = &envelope.predecessor_delivery_id {
        if !state.deliveries.contains_key(predecessor) {
            return FederationOutcome::new(
                FederationDecision::PendingDependency,
                AuthorityDisposition::NoAuthority,
                envelope,
                "Causal predecessor has not yet been admitted.",
            );
        }
    }

    if let Some(existing) = state.deliveries.get_mut(&envelope.logical_delivery_id) {
        if existing.origin_node != envelope.origin_node {
            return FederationOutcome::new(
                FederationDecision::OriginConflict,
                AuthorityDisposition::NoAuthority,
                envelope,
                "A logical delivery identity cannot be reused by a different origin node.",
            );
        }
        if existing.payload_commitment != envelope.payload_commitment
            || existing.semantic_subject_id != envelope.semantic_subject_id
        {
            return FederationOutcome::new(
                FederationDecision::PayloadConflict,
                AuthorityDisposition::NoAuthority,
                envelope,
                "A logical delivery identity cannot be reused for a different semantic payload.",
            );
        }
        existing.attempts.insert(envelope.attempt_id.clone());
        return FederationOutcome::new(
            FederationDecision::Duplicate,
            existing.authority,
            envelope,
            "A replayed attempt is idempotent and preserves the original authority disposition.",
        );
    }

    let foreign = envelope.origin_node != envelope.target_node;
    let scope = envelope.semantic_subject_id.as_str();
    let recognition = match state.recognition_mode(
        &envelope.target_node,
        &envelope.origin_node,
        scope,
    ) {
        Ok(mode) => mode,
        Err(()) => {
            return FederationOutcome::new(
                FederationDecision::RecognitionConflict,
                AuthorityDisposition::NoAuthority,
                envelope,
                "Conflicting recognition edges cannot be resolved into authority.",
            );
        }
    };

    let (decision, authority) = if !foreign {
        (
            FederationDecision::AcceptedLocal,
            AuthorityDisposition::LocalAuthority,
        )
    } else {
        match recognition {
            Some(RecognitionMode::EvidenceOnly) => (
                FederationDecision::AcceptedForeign,
                AuthorityDisposition::RecognizedForeignEvidence,
            ),
            Some(RecognitionMode::DelegatedAuthority) => (
                FederationDecision::AcceptedDelegatedForeign,
                AuthorityDisposition::ExplicitDelegatedAuthority,
            ),
            None => (
                FederationDecision::AcceptedForeign,
                AuthorityDisposition::ForeignEvidence,
            ),
        }
    };

    state.deliveries.insert(
        envelope.logical_delivery_id.clone(),
        DeliveryRecord {
            logical_delivery_id: envelope.logical_delivery_id.clone(),
            semantic_subject_id: envelope.semantic_subject_id.clone(),
            payload_commitment: envelope.payload_commitment.clone(),
            origin_node: envelope.origin_node.clone(),
            authority,
            attempts: BTreeSet::from([envelope.attempt_id.clone()]),
        },
    );

    state.observations.insert(
        envelope.envelope_id.clone(),
        ObservationRecord {
            observation_id: envelope.envelope_id.clone(),
            semantic_subject_id: envelope.semantic_subject_id.clone(),
            payload_commitment: envelope.payload_commitment.clone(),
            origin_node: envelope.origin_node.clone(),
            recognized_by: foreign.then(|| envelope.target_node.clone()),
            source_observation: true,
        },
    );

    FederationOutcome::new(
        decision,
        authority,
        envelope,
        "Delivery admitted without rewriting origin or minting local authority.",
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservationConflict {
    DistinctRecords,
    DuplicateRecord,
}

pub fn record_observation(
    state: &mut FederationState,
    observation: ObservationRecord,
) -> ObservationConflict {
    if let Some(existing) = state.observations.values().find(|existing| {
        existing.semantic_subject_id == observation.semantic_subject_id
            && existing.payload_commitment == observation.payload_commitment
            && existing.origin_node == observation.origin_node
    }) {
        return if existing.observation_id == observation.observation_id {
            ObservationConflict::DuplicateRecord
        } else {
            ObservationConflict::DistinctRecords
        };
    }

    state.observations.insert(observation.observation_id.clone(), observation);
    ObservationConflict::DistinctRecords
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrivacyProjection {
    pub subject_id: String,
    pub origin_node: String,
    pub status: FederationDecision,
    pub source_observation: bool,
    pub may_authorize_local_action: bool,
}

pub fn privacy_minimized_projection(
    outcome: &FederationOutcome,
    subject_id: &str,
) -> PrivacyProjection {
    PrivacyProjection {
        subject_id: subject_id.to_owned(),
        origin_node: outcome.origin_node.clone().unwrap_or_default(),
        status: outcome.decision,
        source_observation: false,
        may_authorize_local_action: false,
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CockpitProjection {
    pub node_id: String,
    pub local_authority: bool,
    pub foreign_evidence: bool,
    pub delegated_foreign_authority: bool,
    pub decision: FederationDecision,
    pub origin_node: Option<String>,
    pub logical_delivery_id: String,
    pub attempt_id: String,
    pub reason: String,
}

pub fn cockpit_projection(node_id: &str, outcome: &FederationOutcome) -> CockpitProjection {
    CockpitProjection {
        node_id: node_id.to_owned(),
        local_authority: matches!(outcome.authority, AuthorityDisposition::LocalAuthority),
        foreign_evidence: matches!(
            outcome.authority,
            AuthorityDisposition::ForeignEvidence
                | AuthorityDisposition::RecognizedForeignEvidence
        ),
        delegated_foreign_authority: matches!(
            outcome.authority,
            AuthorityDisposition::ExplicitDelegatedAuthority
        ),
        decision: outcome.decision,
        origin_node: outcome.origin_node.clone(),
        logical_delivery_id: outcome.logical_delivery_id.clone(),
        attempt_id: outcome.attempt_id.clone(),
        reason: outcome.reason.to_owned(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FederationMutation {
    LocalEvidence,
    ForeignEvidence,
    DuplicateDelivery,
    DelayedDelivery,
    ReorderedDelivery,
    Partition,
    Reconnect,
    StaleSchema,
    ConflictingObservation,
    ExpiredAuthorization,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationScenarioStep {
    pub mutation: FederationMutation,
    pub expected: FederationDecision,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationScenarioResult {
    pub mutation: FederationMutation,
    pub actual: FederationDecision,
    pub passed: bool,
}

pub fn run_scenario(
    initial: FederationState,
    envelope: FederationEnvelope,
    steps: &[FederationScenarioStep],
    now: u64,
) -> Vec<FederationScenarioResult> {
    let mut state = initial;
    let mut results = Vec::with_capacity(steps.len());

    for step in steps {
        let mut candidate = envelope.clone();
        let transport_available = !matches!(step.mutation, FederationMutation::Partition);

        match step.mutation {
            FederationMutation::LocalEvidence
            | FederationMutation::ForeignEvidence
            | FederationMutation::DuplicateDelivery
            | FederationMutation::DelayedDelivery
            | FederationMutation::ReorderedDelivery
            | FederationMutation::Partition
            | FederationMutation::Reconnect
            | FederationMutation::StaleSchema
            | FederationMutation::ConflictingObservation
            | FederationMutation::ExpiredAuthorization => {}
        }

        match step.mutation {
            FederationMutation::ForeignEvidence => {
                candidate.origin_node = "node-b".to_owned();
            }
            FederationMutation::DuplicateDelivery => {
                candidate.attempt_id = format!("{}-retry", candidate.attempt_id);
            }
            FederationMutation::DelayedDelivery => {
                candidate.predecessor_delivery_id = Some("missing-predecessor".to_owned());
            }
            FederationMutation::ReorderedDelivery => {
                candidate.predecessor_delivery_id = Some("missing-predecessor".to_owned());
            }
            FederationMutation::StaleSchema => {
                candidate.schema_generation = candidate.schema_generation.saturating_add(1);
            }
            FederationMutation::ConflictingObservation => {
                candidate.payload_commitment.push_str("-conflict");
            }
            FederationMutation::ExpiredAuthorization => {
                candidate.expires_at = Some(now.saturating_sub(1));
            }
            FederationMutation::Reconnect => {}
            FederationMutation::LocalEvidence | FederationMutation::Partition => {}
        }

        let actual = deliver(&mut state, &candidate, now, transport_available).decision;
        results.push(FederationScenarioResult {
            mutation: step.mutation,
            actual,
            passed: actual == step.expected,
        });
    }

    results
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nodes() -> FederationState {
        FederationState::new([
            NodeProfile {
                node_id: "node-a".into(),
                schema_generation: 1,
                authorization_generation: 1,
                active: true,
            },
            NodeProfile {
                node_id: "node-b".into(),
                schema_generation: 1,
                authorization_generation: 1,
                active: true,
            },
        ])
    }

    fn envelope() -> FederationEnvelope {
        FederationEnvelope {
            envelope_id: "env-1".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:payload".into(),
            logical_delivery_id: "delivery-1".into(),
            attempt_id: "attempt-1".into(),
            origin_node: "node-a".into(),
            target_node: "node-a".into(),
            schema_generation: 1,
            authorization_generation: 1,
            authorization: AuthorizationState::Active,
            expires_at: Some(100),
            predecessor_delivery_id: None,
        }
    }

    #[test]
    fn local_authority_stays_local() {
        let mut state = nodes();
        let outcome = deliver(&mut state, &envelope(), 50, true);
        assert_eq!(outcome.decision, FederationDecision::AcceptedLocal);
        assert_eq!(outcome.authority, AuthorityDisposition::LocalAuthority);
    }

    #[test]
    fn foreign_evidence_keeps_foreign_origin() {
        let mut state = nodes();
        let mut foreign = envelope();
        foreign.envelope_id = "env-foreign".into();
        foreign.logical_delivery_id = "delivery-foreign".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();

        let outcome = deliver(&mut state, &foreign, 50, true);
        assert_eq!(outcome.decision, FederationDecision::AcceptedForeign);
        assert_eq!(outcome.authority, AuthorityDisposition::ForeignEvidence);
        assert_eq!(outcome.origin_node.as_deref(), Some("node-b"));

        let cockpit = cockpit_projection("node-a", &outcome);
        assert!(!cockpit.local_authority);
        assert!(cockpit.foreign_evidence);
        assert!(!cockpit.delegated_foreign_authority);
    }

    #[test]
    fn explicit_recognition_can_be_evidence_only_or_delegated() {
        let mut state = nodes();
        state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });

        let mut foreign = envelope();
        foreign.envelope_id = "env-recognized".into();
        foreign.logical_delivery_id = "delivery-recognized".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();

        let outcome = deliver(&mut state, &foreign, 50, true);
        assert_eq!(outcome.decision, FederationDecision::AcceptedForeign);
        assert_eq!(outcome.authority, AuthorityDisposition::RecognizedForeignEvidence);

        let mut delegated_state = nodes();
        delegated_state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::DelegatedAuthority,
        });
        let delegated = deliver(&mut delegated_state, &foreign, 50, true);
        assert_eq!(
            delegated.decision,
            FederationDecision::AcceptedDelegatedForeign
        );
        assert_eq!(
            delegated.authority,
            AuthorityDisposition::ExplicitDelegatedAuthority
        );

        let mut replay = foreign.clone();
        replay.attempt_id = "attempt-delegated-retry".into();
        let replayed = deliver(&mut delegated_state, &replay, 50, true);
        assert_eq!(replayed.decision, FederationDecision::Duplicate);
        assert_eq!(
            replayed.authority,
            AuthorityDisposition::ExplicitDelegatedAuthority
        );
    }

    #[test]
    fn conflicting_recognition_fails_closed_without_selecting_an_authority() {
        let mut state = nodes();
        state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });
        state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::DelegatedAuthority,
        });

        let mut foreign = envelope();
        foreign.envelope_id = "env-recognition-conflict".into();
        foreign.logical_delivery_id = "delivery-recognition-conflict".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();

        let outcome = deliver(&mut state, &foreign, 50, true);
        assert_eq!(outcome.decision, FederationDecision::RecognitionConflict);
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert!(state.deliveries.is_empty());
    }

    #[test]
    fn duplicate_delivery_is_idempotent_but_payload_mutation_conflicts() {
        let mut state = nodes();
        let first = deliver(&mut state, &envelope(), 50, true);
        assert_eq!(first.decision, FederationDecision::AcceptedLocal);

        let mut retry = envelope();
        retry.attempt_id = "attempt-2".into();
        let duplicate = deliver(&mut state, &retry, 50, true);
        assert_eq!(duplicate.decision, FederationDecision::Duplicate);

        let mut mutated = retry;
        mutated.attempt_id = "attempt-3".into();
        mutated.payload_commitment = "sha256:changed".into();
        let conflict = deliver(&mut state, &mutated, 50, true);
        assert_eq!(conflict.decision, FederationDecision::PayloadConflict);

        let mut foreign_origin = retry;
        foreign_origin.attempt_id = "attempt-4".into();
        foreign_origin.origin_node = "node-b".into();
        let origin_conflict = deliver(&mut state, &foreign_origin, 50, true);
        assert_eq!(origin_conflict.decision, FederationDecision::OriginConflict);
    }

    #[test]
    fn stale_generation_fails_closed() {
        let mut state = nodes();
        let mut stale = envelope();
        stale.schema_generation = 2;
        assert_eq!(
            deliver(&mut state, &stale, 50, true).decision,
            FederationDecision::StaleGeneration
        );
    }

    #[test]
    fn partition_never_fabricates_success_and_reconnect_replays_exact_delivery() {
        let mut state = nodes();
        let candidate = envelope();
        let partitioned = deliver(&mut state, &candidate, 50, false);
        assert_eq!(partitioned.decision, FederationDecision::PartitionUnknown);
        assert!(state.deliveries.is_empty());

        let recovered = deliver(&mut state, &candidate, 50, true);
        assert_eq!(recovered.decision, FederationDecision::AcceptedLocal);

        let retry = FederationEnvelope {
            attempt_id: "attempt-2".into(),
            ..candidate
        };
        assert_eq!(
            deliver(&mut state, &retry, 50, true).decision,
            FederationDecision::Duplicate
        );
    }

    #[test]
    fn delayed_and_reordered_events_remain_pending_until_dependency_exists() {
        let mut state = nodes();
        let mut child = envelope();
        child.envelope_id = "env-child".into();
        child.logical_delivery_id = "delivery-child".into();
        child.predecessor_delivery_id = Some("delivery-parent".into());

        assert_eq!(
            deliver(&mut state, &child, 50, true).decision,
            FederationDecision::PendingDependency
        );

        let mut parent = envelope();
        parent.envelope_id = "env-parent".into();
        parent.logical_delivery_id = "delivery-parent".into();
        assert_eq!(
            deliver(&mut state, &parent, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        assert_eq!(
            deliver(&mut state, &child, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
    }

    #[test]
    fn conflicting_observations_are_distinct_records() {
        let mut state = nodes();
        let a = ObservationRecord {
            observation_id: "obs-a".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:a".into(),
            origin_node: "node-a".into(),
            recognized_by: None,
            source_observation: true,
        };
        let b = ObservationRecord {
            observation_id: "obs-b".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:b".into(),
            origin_node: "node-b".into(),
            recognized_by: Some("node-a".into()),
            source_observation: true,
        };
        assert_eq!(
            record_observation(&mut state, a),
            ObservationConflict::DistinctRecords
        );
        assert_eq!(
            record_observation(&mut state, b),
            ObservationConflict::DistinctRecords
        );
        assert_eq!(state.observations.len(), 2);
    }

    #[test]
    fn expired_authorization_cannot_be_revived() {
        let mut state = nodes();
        let mut expired = envelope();
        expired.expires_at = Some(49);
        let first = deliver(&mut state, &expired, 50, true);
        assert_eq!(
            first.decision,
            FederationDecision::ExpiredAuthorization
        );
        assert!(state.deliveries.is_empty());

        assert_eq!(
            deliver(&mut state, &expired, 50, false).decision,
            FederationDecision::PartitionUnknown
        );
        assert!(state.deliveries.is_empty());
    }

    #[test]
    fn privacy_projection_is_never_a_source_observation_or_local_authority() {
        let mut state = nodes();
        let outcome = deliver(&mut state, &envelope(), 50, true);
        let projection = privacy_minimized_projection(&outcome, "subject-1");
        assert!(!projection.source_observation);
        assert!(!projection.may_authorize_local_action);
    }

    #[test]
    fn federation_summary_cannot_mint_local_authority() {
        let mut state = nodes();
        let mut foreign = envelope();
        foreign.envelope_id = "env-foreign-summary".into();
        foreign.logical_delivery_id = "delivery-foreign-summary".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();

        let outcome = deliver(&mut state, &foreign, 50, true);
        let projection = cockpit_projection("node-a", &outcome);
        assert!(!projection.local_authority);
        assert!(projection.foreign_evidence);
        assert!(!projection.delegated_foreign_authority);
    }

    #[test]
    fn scenario_oracle_is_deterministic_and_claim_bounded() {
        let initial = nodes();
        let candidate = envelope();
        let steps = [
            FederationScenarioStep {
                mutation: FederationMutation::LocalEvidence,
                expected: FederationDecision::AcceptedLocal,
            },
            FederationScenarioStep {
                mutation: FederationMutation::DuplicateDelivery,
                expected: FederationDecision::Duplicate,
            },
            FederationScenarioStep {
                mutation: FederationMutation::StaleSchema,
                expected: FederationDecision::StaleGeneration,
            },
            FederationScenarioStep {
                mutation: FederationMutation::Partition,
                expected: FederationDecision::PartitionUnknown,
            },
            FederationScenarioStep {
                mutation: FederationMutation::ExpiredAuthorization,
                expected: FederationDecision::ExpiredAuthorization,
            },
        ];

        let results = run_scenario(initial, candidate, &steps, 50);
        assert!(results.iter().all(|result| result.passed));
    }
}

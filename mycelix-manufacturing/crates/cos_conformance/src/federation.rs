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
    /// The origin claim carried by the observation.
    pub origin_node: String,
    /// Whether the reference model established that origin as a known node.
    pub origin_node_known: bool,
    pub recognized_by: Option<String>,
    pub source_observation: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ImmutableDeliveryContract {
    pub logical_delivery_id: String,
    pub origin_node: String,
    pub target_node: String,
    pub semantic_subject_id: String,
    pub payload_commitment: String,
    pub schema_generation: u64,
    pub authorization_generation: u64,
    pub predecessor_delivery_id: Option<String>,
    pub expires_at: Option<u64>,
}

impl From<&FederationEnvelope> for ImmutableDeliveryContract {
    fn from(envelope: &FederationEnvelope) -> Self {
        Self {
            logical_delivery_id: envelope.logical_delivery_id.clone(),
            origin_node: envelope.origin_node.clone(),
            target_node: envelope.target_node.clone(),
            semantic_subject_id: envelope.semantic_subject_id.clone(),
            payload_commitment: envelope.payload_commitment.clone(),
            schema_generation: envelope.schema_generation,
            authorization_generation: envelope.authorization_generation,
            predecessor_delivery_id: envelope.predecessor_delivery_id.clone(),
            expires_at: envelope.expires_at,
        }
    }
}

/// Admitted delivery state produced only by `deliver()`.
///
/// ```compile_fail
/// use serde_json::from_str;
/// # use cos_conformance::federation::DeliveryRecord;
/// let _: DeliveryRecord = from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DeliveryRecord {
    contract: ImmutableDeliveryContract,
    authority: AuthorityDisposition,
    attempts: BTreeSet<String>,
}

impl DeliveryRecord {
    pub fn contract(&self) -> &ImmutableDeliveryContract {
        &self.contract
    }

    pub fn authority(&self) -> AuthorityDisposition {
        self.authority
    }

    pub fn attempts(&self) -> &BTreeSet<String> {
        &self.attempts
    }
}

/// In-memory federation state owned by the reference-model transition API.
///
/// The authoritative maps are intentionally private and the state does not
/// implement `Deserialize`. Callers therefore cannot manufacture an admitted
/// delivery or observation by mutating a public map or deserializing a state
/// snapshot. State enters through the explicit transition methods below.
///
/// ```compile_fail
/// use serde_json::from_str;
/// # use cos_conformance::federation::FederationState;
/// let _: FederationState = from_str("{}").unwrap();
/// ```
///
/// The model remains `ReferenceModelOnly`; this is an API-boundary hardening,
/// not a claim about production persistence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationState {
    nodes: BTreeMap<String, NodeProfile>,
    recognition_edges: Vec<RecognitionEdge>,
    deliveries: BTreeMap<String, DeliveryRecord>,
    observations: BTreeMap<String, ObservationRecord>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FederationNodeError {
    EmptyNodeId,
    DuplicateNodeId,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FederationRecognitionError {
    EmptyRecognizingNode,
    EmptyOriginNode,
    EmptyScope,
    UnknownRecognizingNode,
    UnknownOriginNode,
}

impl FederationState {
    pub fn try_new(
        nodes: impl IntoIterator<Item = NodeProfile>,
    ) -> Result<Self, FederationNodeError> {
        let mut node_map = BTreeMap::new();
        for node in nodes {
            let node_id = node.node_id.clone();
            if node_id.is_empty() {
                return Err(FederationNodeError::EmptyNodeId);
            }
            if node_map.insert(node_id, node).is_some() {
                return Err(FederationNodeError::DuplicateNodeId);
            }
        }

        Ok(Self {
            nodes: node_map,
            recognition_edges: Vec::new(),
            deliveries: BTreeMap::new(),
            observations: BTreeMap::new(),
        })
    }

    pub fn new(nodes: impl IntoIterator<Item = NodeProfile>) -> Self {
        Self::try_new(nodes).expect("FederationState::new requires unique node IDs")
    }

    pub fn try_add_recognition(
        &mut self,
        edge: RecognitionEdge,
    ) -> Result<bool, FederationRecognitionError> {
        if edge.recognizing_node.is_empty() {
            return Err(FederationRecognitionError::EmptyRecognizingNode);
        }
        if edge.origin_node.is_empty() {
            return Err(FederationRecognitionError::EmptyOriginNode);
        }
        if edge.scope.is_empty() {
            return Err(FederationRecognitionError::EmptyScope);
        }
        if !self.nodes.contains_key(&edge.recognizing_node) {
            return Err(FederationRecognitionError::UnknownRecognizingNode);
        }
        if !self.nodes.contains_key(&edge.origin_node) {
            return Err(FederationRecognitionError::UnknownOriginNode);
        }
        if self.recognition_edges.iter().any(|existing| existing == &edge) {
            return Ok(false);
        }

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

        Ok(true)
    }

    pub fn add_recognition(&mut self, edge: RecognitionEdge) {
        self.try_add_recognition(edge)
            .expect("FederationState::add_recognition requires known, non-empty identities");
    }

    pub fn delivery(&self, logical_delivery_id: &str) -> Option<&DeliveryRecord> {
        self.deliveries.get(logical_delivery_id)
    }

    pub fn observation(&self, observation_id: &str) -> Option<&ObservationRecord> {
        self.observations.get(observation_id)
    }

    pub fn delivery_count(&self) -> usize {
        self.deliveries.len()
    }

    pub fn observation_count(&self) -> usize {
        self.observations.len()
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
        Ok(modes.pop())
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
    ContractConflict,
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

/// One-way authority/reporting output.
///
/// The authority-bearing fields are private, and the type intentionally
/// implements serialization without deserialization.
///
/// ```compile_fail
/// use serde_json::from_str;
/// # use cos_conformance::federation::FederationOutcome;
/// let _: FederationOutcome = from_str("{}").unwrap();
/// ```
///
/// ```compile_fail
/// # use cos_conformance::federation::{AuthorityDisposition, FederationDecision, FederationOutcome};
/// let _ = FederationOutcome {
///     decision: FederationDecision::AcceptedLocal,
///     authority: AuthorityDisposition::LocalAuthority,
///     origin_node: Some("node-a".into()),
///     origin_node_known: true,
///     logical_delivery_id: "delivery-1".into(),
///     attempt_id: "attempt-1".into(),
///     reason: "fabricated",
/// };
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct FederationOutcome {
    decision: FederationDecision,
    authority: AuthorityDisposition,
    /// The envelope's claimed origin. origin_node_known distinguishes this
    /// claim from a node identity established by the reference model.
    origin_node: Option<String>,
    origin_node_known: bool,
    logical_delivery_id: String,
    attempt_id: String,
    reason: &'static str,
}

impl FederationOutcome {
    pub fn decision(&self) -> FederationDecision {
        self.decision
    }

    pub fn authority(&self) -> AuthorityDisposition {
        self.authority
    }

    pub fn origin_node(&self) -> Option<&str> {
        self.origin_node.as_deref()
    }

    pub fn origin_node_known(&self) -> bool {
        self.origin_node_known
    }

    pub fn logical_delivery_id(&self) -> &str {
        &self.logical_delivery_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn reason(&self) -> &'static str {
        self.reason
    }
}

impl FederationOutcome {
    fn known_origin(
        decision: FederationDecision,
        authority: AuthorityDisposition,
        envelope: &FederationEnvelope,
        reason: &'static str,
    ) -> Self {
        Self::with_origin_knowledge(decision, authority, envelope, reason, true)
    }

    fn unknown_origin(
        decision: FederationDecision,
        authority: AuthorityDisposition,
        envelope: &FederationEnvelope,
        reason: &'static str,
    ) -> Self {
        Self::with_origin_knowledge(decision, authority, envelope, reason, false)
    }

    fn with_origin_knowledge(
        decision: FederationDecision,
        authority: AuthorityDisposition,
        envelope: &FederationEnvelope,
        reason: &'static str,
        origin_node_known: bool,
    ) -> Self {
        Self {
            decision,
            authority,
            origin_node: Some(envelope.origin_node.clone()),
            origin_node_known,
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
    if envelope.envelope_id.is_empty()
        || envelope.semantic_subject_id.is_empty()
        || envelope.payload_commitment.is_empty()
        || envelope.logical_delivery_id.is_empty()
        || envelope.attempt_id.is_empty()
        || envelope.origin_node.is_empty()
        || envelope.target_node.is_empty()
        || envelope
            .predecessor_delivery_id
            .as_ref()
            .is_some_and(String::is_empty)
    {
        return FederationOutcome::unknown_origin(
            FederationDecision::Rejected,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Required federation identity fields must be non-empty.",
        );
    }

    if !transport_available {
        return FederationOutcome::unknown_origin(
            FederationDecision::PartitionUnknown,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Transport unavailability does not establish delivery success.",
        );
    }

    if let Some(existing) = state.deliveries.get(&envelope.logical_delivery_id) {
        if existing.contract.origin_node != envelope.origin_node {
            let mut outcome = FederationOutcome::known_origin(
                FederationDecision::OriginConflict,
                AuthorityDisposition::NoAuthority,
                envelope,
                "A logical delivery identity cannot be rebound by a different origin node.",
            );
            outcome.origin_node_known = state.nodes.contains_key(&envelope.origin_node);
            return outcome;
        }
        if existing.contract.payload_commitment != envelope.payload_commitment
            || existing.contract.semantic_subject_id != envelope.semantic_subject_id
        {
            return FederationOutcome::known_origin(
                FederationDecision::PayloadConflict,
                AuthorityDisposition::NoAuthority,
                envelope,
                "A logical delivery identity cannot be rebound to a different semantic payload.",
            );
        }
        if existing.contract != ImmutableDeliveryContract::from(envelope) {
            return FederationOutcome::known_origin(
                FederationDecision::ContractConflict,
                AuthorityDisposition::NoAuthority,
                envelope,
                "A logical delivery identity cannot be rebound with a changed immutable contract.",
            );
        }
    }

    let Some(origin) = state.nodes.get(&envelope.origin_node) else {
        return FederationOutcome::unknown_origin(
            FederationDecision::UnknownNode,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Origin node is not known to the reference federation.",
        );
    };
    let Some(target) = state.nodes.get(&envelope.target_node) else {
        return FederationOutcome::known_origin(
            FederationDecision::UnknownNode,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Target node is not known to the reference federation.",
        );
    };

    if !origin.active || !target.active {
        return FederationOutcome::known_origin(
            FederationDecision::Unauthorized,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Inactive federation nodes cannot establish an authoritative delivery.",
        );
    }

    if envelope.schema_generation != target.schema_generation {
        return FederationOutcome::known_origin(
            FederationDecision::StaleGeneration,
            AuthorityDisposition::NoAuthority,
            envelope,
            "A stale schema generation cannot be admitted.",
        );
    }

    if envelope.authorization_generation != target.authorization_generation {
        return FederationOutcome::known_origin(
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
        return FederationOutcome::known_origin(
            decision,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Authorization state is not active.",
        );
    }

    if envelope.expires_at.is_some_and(|expiry| now >= expiry) {
        return FederationOutcome::known_origin(
            FederationDecision::ExpiredAuthorization,
            AuthorityDisposition::NoAuthority,
            envelope,
            "An expired authorization cannot be revived by reconnect.",
        );
    }

    if let Some(existing) = state.deliveries.get_mut(&envelope.logical_delivery_id) {
        existing.attempts.insert(envelope.attempt_id.clone());
        return FederationOutcome::known_origin(
            FederationDecision::Duplicate,
            existing.authority,
            envelope,
            "A replayed attempt is idempotent and preserves the original authority disposition.",
        );
    }

    // Admitted predecessors are historical state. Because a predecessor must
    // already exist and admitted contracts are immutable, each accepted edge
    // points backward in admission history; together with the self-edge guard
    // this makes the delivery graph acyclic by construction.
    if let Some(predecessor) = &envelope.predecessor_delivery_id {
        if predecessor == &envelope.logical_delivery_id {
            return FederationOutcome::known_origin(
                FederationDecision::Rejected,
                AuthorityDisposition::NoAuthority,
                envelope,
                "A delivery cannot causally depend on its own logical delivery identity.",
            );
        }
        if !state.deliveries.contains_key(predecessor) {
            return FederationOutcome::known_origin(
                FederationDecision::PendingDependency,
                AuthorityDisposition::NoAuthority,
                envelope,
                "Causal predecessor has not yet been admitted.",
            );
        }
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
            return FederationOutcome::known_origin(
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

    let observation = ObservationRecord {
        observation_id: envelope.envelope_id.clone(),
        semantic_subject_id: envelope.semantic_subject_id.clone(),
        payload_commitment: envelope.payload_commitment.clone(),
        origin_node: envelope.origin_node.clone(),
        origin_node_known: true,
        recognized_by: if foreign && recognition.is_some() {
            Some(envelope.target_node.clone())
        } else {
            None
        },
        source_observation: true,
    };

    if !matches!(
        record_source_observation(state, observation),
        ObservationWriteResult::Inserted
    ) {
        return FederationOutcome::known_origin(
            FederationDecision::Rejected,
            AuthorityDisposition::NoAuthority,
            envelope,
            "An existing observation identity cannot be rebound or reused while admitting a new delivery.",
        );
    }

    state.deliveries.insert(
        envelope.logical_delivery_id.clone(),
        DeliveryRecord {
            contract: ImmutableDeliveryContract::from(envelope),
            authority,
            attempts: BTreeSet::from([envelope.attempt_id.clone()]),
        },
    );

    FederationOutcome::known_origin(
        decision,
        authority,
        envelope,
        "Delivery admitted without rewriting origin or minting local authority.",
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservationWriteResult {
    Inserted,
    Duplicate,
    Conflict,
    RejectedSourceClaim,
    RejectedKnownOriginClaim,
    RejectedRecognitionClaim,
    RejectedMalformedObservation,
}

fn record_observation_identity(
    state: &mut FederationState,
    observation: ObservationRecord,
) -> ObservationWriteResult {
    if observation.observation_id.is_empty()
        || observation.semantic_subject_id.is_empty()
        || observation.payload_commitment.is_empty()
        || observation.origin_node.is_empty()
        || observation
            .recognized_by
            .as_ref()
            .is_some_and(String::is_empty)
    {
        return ObservationWriteResult::RejectedMalformedObservation;
    }

    if let Some(existing) = state.observations.get(&observation.observation_id) {
        return if existing == &observation {
            ObservationWriteResult::Duplicate
        } else {
            ObservationWriteResult::Conflict
        };
    }

    state.observations.insert(observation.observation_id.clone(), observation);
    ObservationWriteResult::Inserted
}

fn record_source_observation(
    state: &mut FederationState,
    observation: ObservationRecord,
) -> ObservationWriteResult {
    debug_assert!(observation.source_observation);
    record_observation_identity(state, observation)
}

pub fn record_observation(
    state: &mut FederationState,
    observation: ObservationRecord,
) -> ObservationWriteResult {
    if observation.source_observation {
        return ObservationWriteResult::RejectedSourceClaim;
    }
    if observation.origin_node_known {
        return ObservationWriteResult::RejectedKnownOriginClaim;
    }
    if observation.recognized_by.is_some() {
        return ObservationWriteResult::RejectedRecognitionClaim;
    }

    record_observation_identity(state, observation)
}

/// One-way privacy/reporting projection.
///
/// ```compile_fail
/// use serde_json::from_str;
/// # use cos_conformance::federation::PrivacyProjection;
/// let _: PrivacyProjection = from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct PrivacyProjection {
    pub subject_id: String,
    /// The envelope's claimed origin; consult origin_node_known before treating
    /// it as a model-established node identity.
    pub origin_node: String,
    pub origin_node_known: bool,
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
        origin_node_known: outcome.origin_node_known,
        status: outcome.decision,
        source_observation: false,
        may_authorize_local_action: false,
    }
}

/// One-way cockpit/reporting projection.
///
/// ```compile_fail
/// use serde_json::from_str;
/// # use cos_conformance::federation::CockpitProjection;
/// let _: CockpitProjection = from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct CockpitProjection {
    pub node_id: String,
    pub local_authority: bool,
    pub foreign_evidence: bool,
    pub delegated_foreign_authority: bool,
    pub decision: FederationDecision,
    pub origin_node: Option<String>,
    pub origin_node_known: bool,
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
        origin_node_known: outcome.origin_node_known,
        logical_delivery_id: outcome.logical_delivery_id.clone(),
        attempt_id: outcome.attempt_id.clone(),
        reason: outcome.reason.to_owned(),
    }
}

macro_rules! define_federation_mutations {
    ($( $variant:ident ),+ $(,)?) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
        pub enum FederationMutation {
            $( $variant ),+
        }

        const ALL_FEDERATION_MUTATIONS: &[FederationMutation] = &[
            $( FederationMutation::$variant ),+
        ];
    };
}

define_federation_mutations!(
    LocalEvidence,
    ForeignEvidence,
    DuplicateDelivery,
    NewLogicalDelivery,
    DelayedDelivery,
    ReorderedDelivery,
    Partition,
    Reconnect,
    StaleSchema,
    ConflictingObservation,
    ExpiredAuthorization,
    RevokedAuthorization,
    AbsentAuthorization,
    ContractOriginMutation,
    ContractTargetMutation,
    ContractSubjectMutation,
    ContractPayloadMutation,
    ContractSchemaGenerationMutation,
    ContractAuthorizationGenerationMutation,
    ContractPredecessorMutation,
    ContractExpiryMutation,
);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationScenarioStep {
    pub mutation: FederationMutation,
    pub expected: FederationDecision,
    pub expected_authority: AuthorityDisposition,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationScenarioResult {
    pub mutation: FederationMutation,
    pub actual: FederationDecision,
    pub actual_authority: AuthorityDisposition,
    pub passed: bool,
    pub delivery_identity_unchanged: bool,
    pub delivery_attempts_unchanged: bool,
    pub observation_ledger_unchanged: bool,
}

fn expected_scenario_decision(mutation: FederationMutation) -> FederationDecision {
    match mutation {
        FederationMutation::LocalEvidence => FederationDecision::AcceptedLocal,
        FederationMutation::ForeignEvidence => FederationDecision::AcceptedForeign,
        FederationMutation::DuplicateDelivery => FederationDecision::Duplicate,
        FederationMutation::NewLogicalDelivery => FederationDecision::AcceptedLocal,
        FederationMutation::DelayedDelivery | FederationMutation::ReorderedDelivery => {
            FederationDecision::PendingDependency
        }
        FederationMutation::Partition => FederationDecision::PartitionUnknown,
        FederationMutation::Reconnect => FederationDecision::Duplicate,
        FederationMutation::StaleSchema => FederationDecision::StaleGeneration,
        FederationMutation::ConflictingObservation => FederationDecision::PayloadConflict,
        FederationMutation::ExpiredAuthorization => FederationDecision::ExpiredAuthorization,
        FederationMutation::RevokedAuthorization | FederationMutation::AbsentAuthorization => {
            FederationDecision::Unauthorized
        }
        FederationMutation::ContractOriginMutation => FederationDecision::OriginConflict,
        FederationMutation::ContractTargetMutation => FederationDecision::ContractConflict,
        FederationMutation::ContractSubjectMutation | FederationMutation::ContractPayloadMutation => {
            FederationDecision::PayloadConflict
        }
        FederationMutation::ContractSchemaGenerationMutation
        | FederationMutation::ContractAuthorizationGenerationMutation => {
            FederationDecision::StaleGeneration
        }
        FederationMutation::ContractPredecessorMutation
        | FederationMutation::ContractExpiryMutation => FederationDecision::ContractConflict,
    }
}

fn expected_scenario_authority(mutation: FederationMutation) -> AuthorityDisposition {
    match mutation {
        FederationMutation::LocalEvidence | FederationMutation::NewLogicalDelivery => {
            AuthorityDisposition::LocalAuthority
        }
        FederationMutation::ForeignEvidence => AuthorityDisposition::ForeignEvidence,
        FederationMutation::DuplicateDelivery | FederationMutation::Reconnect => {
            AuthorityDisposition::LocalAuthority
        }
        FederationMutation::DelayedDelivery
        | FederationMutation::ReorderedDelivery
        | FederationMutation::Partition
        | FederationMutation::StaleSchema
        | FederationMutation::ConflictingObservation
        | FederationMutation::ExpiredAuthorization
        | FederationMutation::RevokedAuthorization
        | FederationMutation::AbsentAuthorization
        | FederationMutation::ContractOriginMutation
        | FederationMutation::ContractTargetMutation
        | FederationMutation::ContractSubjectMutation
        | FederationMutation::ContractPayloadMutation
        | FederationMutation::ContractSchemaGenerationMutation
        | FederationMutation::ContractAuthorizationGenerationMutation
        | FederationMutation::ContractPredecessorMutation
        | FederationMutation::ContractExpiryMutation => AuthorityDisposition::NoAuthority,
    }
}

fn scenario_steps() -> Vec<FederationScenarioStep> {
    ALL_FEDERATION_MUTATIONS
        .iter()
        .copied()
        .map(|mutation| FederationScenarioStep {
            mutation,
            expected: expected_scenario_decision(mutation),
            expected_authority: expected_scenario_authority(mutation),
        })
        .collect()
}

fn delivery_attempts_snapshot(
    state: &FederationState,
) -> BTreeMap<String, BTreeSet<String>> {
    state
        .deliveries
        .iter()
        .map(|(id, record)| (id.clone(), record.attempts.clone()))
        .collect()
}

fn delivery_identity_snapshot(
    state: &FederationState,
) -> BTreeMap<String, (ImmutableDeliveryContract, AuthorityDisposition)> {
    state
        .deliveries
        .iter()
        .map(|(id, record)| {
            (
                id.clone(),
                (record.contract.clone(), record.authority),
            )
        })
        .collect()
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
        let before_delivery_identity = delivery_identity_snapshot(&state);
        let before_delivery_attempts = delivery_attempts_snapshot(&state);
        let before_observations = state.observations.clone();

        let mut candidate = envelope.clone();
        let transport_available = !matches!(step.mutation, FederationMutation::Partition);

        match step.mutation {
            FederationMutation::ForeignEvidence => {
                candidate.envelope_id = format!("{}-foreign", candidate.envelope_id);
                candidate.logical_delivery_id =
                    format!("{}-foreign", candidate.logical_delivery_id);
                candidate.attempt_id = format!("{}-foreign", candidate.attempt_id);
                candidate.origin_node = "node-b".to_owned();
            }
            FederationMutation::DuplicateDelivery => {
                candidate.attempt_id = format!("{}-retry", candidate.attempt_id);
            }
            FederationMutation::NewLogicalDelivery => {
                candidate.envelope_id = format!("{}-new", candidate.envelope_id);
                candidate.logical_delivery_id = format!("{}-new", candidate.logical_delivery_id);
                candidate.attempt_id = format!("{}-new", candidate.attempt_id);
            }
            FederationMutation::DelayedDelivery => {
                candidate.envelope_id = format!("{}-delayed", candidate.envelope_id);
                candidate.logical_delivery_id =
                    format!("{}-delayed", candidate.logical_delivery_id);
                candidate.attempt_id = format!("{}-delayed", candidate.attempt_id);
                candidate.predecessor_delivery_id = Some("missing-predecessor".to_owned());
            }
            FederationMutation::ReorderedDelivery => {
                candidate.envelope_id = format!("{}-reordered", candidate.envelope_id);
                candidate.logical_delivery_id =
                    format!("{}-reordered", candidate.logical_delivery_id);
                candidate.attempt_id = format!("{}-reordered", candidate.attempt_id);
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
            FederationMutation::RevokedAuthorization => {
                candidate.authorization = AuthorizationState::Revoked;
            }
            FederationMutation::AbsentAuthorization => {
                candidate.authorization = AuthorizationState::Absent;
            }
            FederationMutation::Reconnect => {}
            FederationMutation::LocalEvidence | FederationMutation::Partition => {}
            FederationMutation::ContractOriginMutation => {
                candidate.origin_node = "node-b".to_owned();
            }
            FederationMutation::ContractTargetMutation => {
                candidate.target_node = "node-b".to_owned();
            }
            FederationMutation::ContractSubjectMutation => {
                candidate.semantic_subject_id = "subject-mutated".to_owned();
            }
            FederationMutation::ContractPayloadMutation => {
                candidate.payload_commitment.push_str("-mutated");
            }
            FederationMutation::ContractSchemaGenerationMutation => {
                candidate.schema_generation = candidate.schema_generation.saturating_add(1);
            }
            FederationMutation::ContractAuthorizationGenerationMutation => {
                candidate.authorization_generation =
                    candidate.authorization_generation.saturating_add(1);
            }
            FederationMutation::ContractPredecessorMutation => {
                candidate.predecessor_delivery_id = Some(candidate.logical_delivery_id.clone());
            }
            FederationMutation::ContractExpiryMutation => {
                candidate.expires_at = candidate.expires_at.map(|expiry| expiry.saturating_add(1));
            }
        }

        let outcome = deliver(&mut state, &candidate, now, transport_available);
        let actual = outcome.decision;
        let actual_authority = outcome.authority;
        let delivery_identity_unchanged =
            delivery_identity_snapshot(&state) == before_delivery_identity;
        let delivery_attempts_unchanged =
            delivery_attempts_snapshot(&state) == before_delivery_attempts;
        let observation_ledger_unchanged =
            state.observations == before_observations;

        results.push(FederationScenarioResult {
            mutation: step.mutation,
            actual,
            actual_authority,
            passed: actual == step.expected && actual_authority == step.expected_authority,
            delivery_identity_unchanged,
            delivery_attempts_unchanged,
            observation_ledger_unchanged,
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
            NodeProfile {
                node_id: "node-c".into(),
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
    fn empty_node_ids_are_rejected_by_the_safe_constructor() {
        let result = FederationState::try_new([NodeProfile {
            node_id: String::new(),
            schema_generation: 1,
            authorization_generation: 1,
            active: true,
        }]);

        assert_eq!(result, Err(FederationNodeError::EmptyNodeId));
    }

    #[test]
    fn duplicate_node_ids_are_rejected_by_the_safe_constructor() {
        let result = FederationState::try_new([
            NodeProfile {
                node_id: "node-a".into(),
                schema_generation: 1,
                authorization_generation: 1,
                active: true,
            },
            NodeProfile {
                node_id: "node-a".into(),
                schema_generation: 2,
                authorization_generation: 2,
                active: false,
            },
        ]);

        assert_eq!(result, Err(FederationNodeError::DuplicateNodeId));
    }

    #[test]
    fn local_authority_stays_local() {
        let mut state = nodes();
        let outcome = deliver(&mut state, &envelope(), 50, true);
        assert_eq!(outcome.decision, FederationDecision::AcceptedLocal);
        assert_eq!(outcome.authority, AuthorityDisposition::LocalAuthority);
    }

    #[test]
    fn unrecognized_foreign_evidence_does_not_claim_recognition() {
        let mut state = nodes();
        let mut foreign = envelope();
        foreign.envelope_id = "env-foreign-unrecognized".into();
        foreign.logical_delivery_id = "delivery-foreign-unrecognized".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();

        let outcome = deliver(&mut state, &foreign, 50, true);
        assert_eq!(outcome.decision, FederationDecision::AcceptedForeign);
        assert_eq!(outcome.authority, AuthorityDisposition::ForeignEvidence);
        assert_eq!(
            state
                .observation("env-foreign-unrecognized")
                .unwrap()
                .recognized_by,
            None
        );
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
        assert_eq!(
            state.observation("env-recognized").unwrap().recognized_by.as_deref(),
            Some("node-a")
        );

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
    fn recognition_edges_require_non_empty_known_identities() {
        let mut state = nodes();

        let empty_scope = RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: String::new(),
            mode: RecognitionMode::EvidenceOnly,
        };
        assert_eq!(
            state.try_add_recognition(empty_scope),
            Err(FederationRecognitionError::EmptyScope)
        );

        let unknown_origin = RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-unknown".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        };
        assert_eq!(
            state.try_add_recognition(unknown_origin),
            Err(FederationRecognitionError::UnknownOriginNode)
        );

        let unknown_recognizer = RecognitionEdge {
            recognizing_node: "node-unknown".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        };
        assert_eq!(
            state.try_add_recognition(unknown_recognizer),
            Err(FederationRecognitionError::UnknownRecognizingNode)
        );

        let valid = RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        };
        assert_eq!(state.try_add_recognition(valid), Ok(true));
    }

    #[test]
    fn exact_duplicate_recognition_edges_are_canonicalized_without_masking_conflicts() {
        let mut state = nodes();
        let edge = RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        };

        state.add_recognition(edge.clone());
        state.add_recognition(edge.clone());
        assert_eq!(state.recognition_edges.len(), 1);

        let mut foreign = envelope();
        foreign.envelope_id = "env-recognition-canonical".into();
        foreign.logical_delivery_id = "delivery-recognition-canonical".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();

        let outcome = deliver(&mut state, &foreign, 50, true);
        assert_eq!(outcome.decision, FederationDecision::AcceptedForeign);
        assert_eq!(
            outcome.authority,
            AuthorityDisposition::RecognizedForeignEvidence
        );

        state.add_recognition(RecognitionEdge {
            mode: RecognitionMode::DelegatedAuthority,
            ..edge
        });

        let second = FederationEnvelope {
            envelope_id: "env-recognition-conflicting-canonical".into(),
            logical_delivery_id: "delivery-recognition-conflicting-canonical".into(),
            ..foreign
        };
        let conflict = deliver(&mut state, &second, 50, true);
        assert_eq!(conflict.decision, FederationDecision::RecognitionConflict);
        assert_eq!(conflict.authority, AuthorityDisposition::NoAuthority);
    }

    #[test]
    fn recognition_changes_do_not_retroactively_rewrite_admitted_authority() {
        let mut state = nodes();
        let mut foreign = envelope();
        foreign.envelope_id = "env-recognition-drift".into();
        foreign.logical_delivery_id = "delivery-recognition-drift".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();

        let admitted = deliver(&mut state, &foreign, 50, true);
        assert_eq!(admitted.decision, FederationDecision::AcceptedForeign);
        assert_eq!(admitted.authority, AuthorityDisposition::ForeignEvidence);

        state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::DelegatedAuthority,
        });

        let mut retry = foreign.clone();
        retry.attempt_id = "attempt-recognition-drift".into();
        let replayed = deliver(&mut state, &retry, 50, true);
        assert_eq!(replayed.decision, FederationDecision::Duplicate);
        assert_eq!(replayed.authority, AuthorityDisposition::ForeignEvidence);
        assert_eq!(
            state.delivery("delivery-recognition-drift").unwrap().authority(),
            AuthorityDisposition::ForeignEvidence
        );

        state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });

        let mut second_retry = retry;
        second_retry.attempt_id = "attempt-recognition-conflict".into();
        let replayed_again = deliver(&mut state, &second_retry, 50, true);
        assert_eq!(replayed_again.decision, FederationDecision::Duplicate);
        assert_eq!(replayed_again.authority, AuthorityDisposition::ForeignEvidence);
        assert_eq!(state.observation_count(), 1);
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
        assert!(state.delivery_count() == 0);
    }

    #[test]
    fn exact_attempt_replay_is_side_effect_free() {
        let mut state = nodes();
        let candidate = envelope();

        let first = deliver(&mut state, &candidate, 50, true);
        assert_eq!(first.decision, FederationDecision::AcceptedLocal);
        let before_identity = delivery_identity_snapshot(&state);
        let before_attempts = delivery_attempts_snapshot(&state);
        let before_observations = state.observations.clone();

        let replay = deliver(&mut state, &candidate, 50, true);
        assert_eq!(replay.decision, FederationDecision::Duplicate);
        assert_eq!(replay.authority, AuthorityDisposition::LocalAuthority);
        assert_eq!(delivery_identity_snapshot(&state), before_identity);
        assert_eq!(delivery_attempts_snapshot(&state), before_attempts);
        assert_eq!(state.observations, before_observations);
        assert_eq!(
            state.delivery("delivery-1").unwrap().attempts().len(),
            1
        );
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

        let before_contract_conflicts = (
            delivery_identity_snapshot(&state),
            delivery_attempts_snapshot(&state),
            state.observations.clone(),
        );

        let mut mutated = retry;
        mutated.attempt_id = "attempt-3".into();
        mutated.payload_commitment = "sha256:changed".into();
        let conflict = deliver(&mut state, &mutated, 50, true);
        assert_eq!(conflict.decision, FederationDecision::PayloadConflict);
        assert_eq!(delivery_identity_snapshot(&state), before_contract_conflicts.0);
        assert_eq!(delivery_attempts_snapshot(&state), before_contract_conflicts.1);
        assert_eq!(state.observations, before_contract_conflicts.2);

        let mut foreign_origin = envelope();
        foreign_origin.attempt_id = "attempt-4".into();
        foreign_origin.origin_node = "node-b".into();
        let origin_conflict = deliver(&mut state, &foreign_origin, 50, true);
        assert_eq!(origin_conflict.decision, FederationDecision::OriginConflict);
        assert_eq!(delivery_identity_snapshot(&state), before_contract_conflicts.0);
        assert_eq!(delivery_attempts_snapshot(&state), before_contract_conflicts.1);
        assert_eq!(state.observations, before_contract_conflicts.2);
    }

    #[test]
    fn immutable_contract_conflict_precedes_unknown_target_or_origin() {
        let mut state = nodes();
        let original = envelope();

        assert_eq!(
            deliver(&mut state, &original, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
        let before = state.delivery("delivery-1").unwrap().clone();

        let mut unknown_target = original.clone();
        unknown_target.target_node = "node-unknown".into();
        unknown_target.attempt_id = "attempt-unknown-target".into();
        let target_outcome = deliver(&mut state, &unknown_target, 50, true);
        assert_eq!(target_outcome.decision, FederationDecision::ContractConflict);
        assert_eq!(target_outcome.authority, AuthorityDisposition::NoAuthority);

        let mut unknown_origin = original;
        unknown_origin.origin_node = "node-unknown".into();
        unknown_origin.attempt_id = "attempt-unknown-origin".into();
        let origin_outcome = deliver(&mut state, &unknown_origin, 50, true);
        assert_eq!(origin_outcome.decision, FederationDecision::OriginConflict);
        assert_eq!(origin_outcome.authority, AuthorityDisposition::NoAuthority);
        assert!(!origin_outcome.origin_node_known());

        assert_eq!(state.delivery("delivery-1"), Some(&before));
        assert_eq!(state.delivery_count(), 1);
        assert_eq!(state.observation_count(), 1);
    }

    #[test]
    fn logical_delivery_identity_is_bound_to_target_and_all_immutable_fields() {
        let mut state = nodes();
        let original = envelope();
        let admitted = deliver(&mut state, &original, 50, true);
        assert_eq!(admitted.authority, AuthorityDisposition::LocalAuthority);
        let original_record = state.delivery("delivery-1").unwrap().clone();

        let mut redirected = original.clone();
        redirected.target_node = "node-b".into();
        redirected.attempt_id = "attempt-redirected".into();
        let outcome = deliver(&mut state, &redirected, 50, true);
        assert_eq!(outcome.decision, FederationDecision::ContractConflict);
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert!(!cockpit_projection("node-b", &outcome).local_authority);
        assert_eq!(state.delivery("delivery-1"), Some(&original_record));
        assert_eq!(state.delivery_count(), 1);
    }

    #[test]
    fn generic_observation_api_cannot_mint_known_origin_claim() {
        let mut state = nodes();
        let known_claim = ObservationRecord {
            observation_id: "obs-forged-known-origin".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:forged".into(),
            origin_node: "node-a".into(),
            origin_node_known: true,
            recognized_by: None,
            source_observation: false,
        };

        assert_eq!(
            record_observation(&mut state, known_claim),
            ObservationWriteResult::RejectedKnownOriginClaim
        );
        assert_eq!(state.observation_count(), 0);
    }

    #[test]
    fn generic_observation_api_cannot_mint_recognition_claim() {
        let mut state = nodes();
        let recognition_claim = ObservationRecord {
            observation_id: "obs-forged-recognition".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:forged".into(),
            origin_node: "node-b".into(),
            origin_node_known: false,
            recognized_by: Some("node-a".into()),
            source_observation: false,
        };

        assert_eq!(
            record_observation(&mut state, recognition_claim),
            ObservationWriteResult::RejectedRecognitionClaim
        );
        assert_eq!(state.observation_count(), 0);
    }

    #[test]
    fn generic_observation_api_rejects_empty_identity_fields() {
        let mut state = nodes();
        let cases: [fn(&mut ObservationRecord); 5] = [
            |o| o.observation_id.clear(),
            |o| o.semantic_subject_id.clear(),
            |o| o.payload_commitment.clear(),
            |o| o.origin_node.clear(),
            |o| o.recognized_by = Some(String::new()),
        ];

        for mutate in cases {
            let mut observation = ObservationRecord {
                observation_id: "obs-valid".into(),
                semantic_subject_id: "subject-1".into(),
                payload_commitment: "sha256:valid".into(),
                origin_node: "node-a".into(),
                origin_node_known: false,
                recognized_by: None,
                source_observation: false,
            };
            mutate(&mut observation);

            assert_eq!(
                record_observation(&mut state, observation),
                ObservationWriteResult::RejectedMalformedObservation
            );
            assert_eq!(state.observation_count(), 0);
        }
    }

    #[test]
    fn generic_observation_api_cannot_mint_source_observation() {
        let mut state = nodes();
        let source_claim = ObservationRecord {
            observation_id: "obs-forged-source".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:forged".into(),
            origin_node: "node-a".into(),
            origin_node_known: false,
            recognized_by: None,
            source_observation: true,
        };

        assert_eq!(
            record_observation(&mut state, source_claim),
            ObservationWriteResult::RejectedSourceClaim
        );
        assert_eq!(state.observation_count(), 0);
    }

    #[test]
    fn observation_identity_is_insert_only_and_conflicts_never_overwrite() {
        let mut state = nodes();
        let first = ObservationRecord {
            observation_id: "obs-fixed".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:a".into(),
            origin_node: "node-a".into(),
            origin_node_known: false,
            recognized_by: None,
            source_observation: false,
        };
        assert_eq!(
            record_observation(&mut state, first.clone()),
            ObservationWriteResult::Inserted
        );
        assert_eq!(
            record_observation(&mut state, first.clone()),
            ObservationWriteResult::Duplicate
        );
        let mut changed = first.clone();
        changed.origin_node = "node-b".into();
        assert_eq!(
            record_observation(&mut state, changed),
            ObservationWriteResult::Conflict
        );
        assert_eq!(state.observation("obs-fixed"), Some(&first));
        assert_eq!(state.observation_count(), 1);

        let mut independent = first;
        independent.observation_id = "obs-independent".into();
        assert_eq!(
            record_observation(&mut state, independent),
            ObservationWriteResult::Inserted
        );
        assert_eq!(state.observations.len(), 2);
    }

    #[test]
    fn immutable_delivery_contract_rejects_each_mutable_component_change() {
        let cases: [(&str, fn(&mut FederationEnvelope), FederationDecision); 8] = [
            (
                "origin",
                |candidate| candidate.origin_node = "node-b".into(),
                FederationDecision::OriginConflict,
            ),
            (
                "target",
                |candidate| candidate.target_node = "node-b".into(),
                FederationDecision::ContractConflict,
            ),
            (
                "semantic_subject",
                |candidate| candidate.semantic_subject_id = "subject-2".into(),
                FederationDecision::PayloadConflict,
            ),
            (
                "payload_commitment",
                |candidate| candidate.payload_commitment = "sha256:changed".into(),
                FederationDecision::PayloadConflict,
            ),
            (
                "schema_generation",
                |candidate| candidate.schema_generation = 2,
                FederationDecision::StaleGeneration,
            ),
            (
                "authorization_generation",
                |candidate| candidate.authorization_generation = 2,
                FederationDecision::StaleGeneration,
            ),
            (
                "predecessor",
                |candidate| candidate.predecessor_delivery_id = Some("delivery-parent-2".into()),
                FederationDecision::ContractConflict,
            ),
            (
                "expiry",
                |candidate| candidate.expires_at = Some(101),
                FederationDecision::ContractConflict,
            ),
        ];

        for (name, mutate, expected) in cases {
            let mut state = nodes();

            let mut parent = envelope();
            parent.envelope_id = "env-parent-1".into();
            parent.logical_delivery_id = "delivery-parent-1".into();
            assert_eq!(
                deliver(&mut state, &parent, 50, true).decision,
                FederationDecision::AcceptedLocal
            );

            let mut second_parent = parent.clone();
            second_parent.envelope_id = "env-parent-2".into();
            second_parent.logical_delivery_id = "delivery-parent-2".into();
            second_parent.attempt_id = "attempt-parent-2".into();
            assert_eq!(
                deliver(&mut state, &second_parent, 50, true).decision,
                FederationDecision::AcceptedLocal
            );

            let mut original = envelope();
            original.envelope_id = format!("env-child-{name}");
            original.logical_delivery_id = format!("delivery-child-{name}");
            original.predecessor_delivery_id = Some("delivery-parent-1".into());

            assert_eq!(
                deliver(&mut state, &original, 50, true).decision,
                FederationDecision::AcceptedLocal,
                "baseline failed for {name}"
            );
            let before = state.delivery(original.logical_delivery_id.as_str()).cloned();
            let before_identity = delivery_identity_snapshot(&state);
            let before_attempts = delivery_attempts_snapshot(&state);
            let before_observations = state.observations.clone();

            let mut mutated = original.clone();
            mutated.attempt_id = format!("attempt-mutated-{name}");
            mutate(&mut mutated);

            let outcome = deliver(&mut state, &mutated, 50, true);
            assert_eq!(outcome.decision, expected, "mutation: {name}");
            assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
            assert_eq!(
                state.delivery(original.logical_delivery_id.as_str()),
                before.as_ref(),
                "state changed after {name} rejection"
            );
            assert_eq!(
                delivery_identity_snapshot(&state),
                before_identity,
                "delivery identity ledger changed after {name} rejection"
            );
            assert_eq!(
                delivery_attempts_snapshot(&state),
                before_attempts,
                "delivery attempt ledger changed after {name} rejection"
            );
            assert_eq!(
                state.observations,
                before_observations,
                "observation ledger changed after {name} rejection"
            );
            assert_eq!(state.delivery_count(), 3, "unexpected delivery count for {name}");
        }
    }

    #[test]
    fn self_predecessor_is_rejected_without_state_mutation() {
        let mut state = nodes();
        let mut candidate = envelope();
        candidate.predecessor_delivery_id = Some(candidate.logical_delivery_id.clone());

        let outcome = deliver(&mut state, &candidate, 50, true);
        assert_eq!(outcome.decision, FederationDecision::Rejected);
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert_eq!(state.delivery_count(), 0);
        assert_eq!(state.observation_count(), 0);
    }

    #[test]
    fn existing_delivery_contract_conflict_precedes_missing_dependency() {
        let mut state = nodes();
        let original = envelope();

        assert_eq!(
            deliver(&mut state, &original, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
        let before = state.delivery("delivery-1").unwrap().clone();

        let mut mutated = original;
        mutated.attempt_id = "attempt-missing-predecessor".into();
        mutated.predecessor_delivery_id = Some("missing-predecessor".into());

        let outcome = deliver(&mut state, &mutated, 50, true);
        assert_eq!(outcome.decision, FederationDecision::ContractConflict);
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert_eq!(state.delivery("delivery-1"), Some(&before));
        assert_eq!(state.delivery_count(), 1);
        assert_eq!(state.observation_count(), 1);
    }

    #[test]
    fn observation_identity_cannot_be_reused_to_mint_a_new_delivery() {
        let mut state = nodes();
        let original = envelope();

        assert_eq!(
            deliver(&mut state, &original, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
        let before = state.delivery("delivery-1").unwrap().clone();

        let mut aliased = original.clone();
        aliased.logical_delivery_id = "delivery-alias".into();
        aliased.attempt_id = "attempt-alias".into();

        let outcome = deliver(&mut state, &aliased, 50, true);
        assert_eq!(outcome.decision, FederationDecision::Rejected);
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert_eq!(state.delivery("delivery-1"), Some(&before));
        assert!(state.delivery("delivery-alias").is_none());
        assert_eq!(state.delivery_count(), 1);
        assert_eq!(state.observation_count(), 1);
    }

    #[test]
    fn new_logical_delivery_is_a_distinct_identity() {
        let mut state = nodes();
        let first = envelope();
        assert_eq!(
            deliver(&mut state, &first, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        let mut second = first.clone();
        second.envelope_id = "env-2".into();
        second.logical_delivery_id = "delivery-2".into();
        second.attempt_id = "attempt-2".into();

        assert_eq!(
            deliver(&mut state, &second, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
        assert_eq!(state.delivery_count(), 2);
        assert_eq!(state.observation_count(), 2);
        assert!(state.delivery("delivery-2").is_some());
        assert!(state.observation("env-2").is_some());
    }

    #[test]
    fn retry_identity_changes_do_not_create_alias_observations() {
        let mut state = nodes();
        let original = envelope();

        let first = deliver(&mut state, &original, 50, true);
        assert_eq!(first.decision, FederationDecision::AcceptedLocal);
        assert_eq!(state.observation_count(), 1);
        assert!(state.observation("env-1").is_some());

        let mut retry = original.clone();
        retry.attempt_id = "attempt-alias-check".into();
        retry.envelope_id = "env-retry-alias".into();

        let duplicate = deliver(&mut state, &retry, 50, true);
        assert_eq!(duplicate.decision, FederationDecision::Duplicate);
        assert_eq!(state.observation_count(), 1);
        assert!(state.observation("env-1").is_some());
        assert!(state.observation("env-retry-alias").is_none());
        assert_eq!(
            state.delivery("delivery-1").unwrap().attempts().len(),
            2
        );
        assert!(state.delivery("delivery-1").unwrap().attempts().contains("attempt-alias-check"));
        assert_eq!(
            state.delivery("delivery-1").unwrap().contract(),
            &ImmutableDeliveryContract::from(&original)
        );
    }

    #[test]
    fn delivery_admission_uses_insert_only_observation_identity() {
        let mut state = nodes();
        let existing = ObservationRecord {
            observation_id: "env-1".into(),
            semantic_subject_id: "subject-existing".into(),
            payload_commitment: "sha256:existing".into(),
            origin_node: "node-b".into(),
            origin_node_known: false,
            recognized_by: None,
            source_observation: false,
        };
        assert_eq!(
            record_observation(&mut state, existing.clone()),
            ObservationWriteResult::Inserted
        );

        let outcome = deliver(&mut state, &envelope(), 50, true);
        assert_eq!(outcome.decision, FederationDecision::Rejected);
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert!(state.deliveries.is_empty());
        assert_eq!(state.observations.get("env-1"), Some(&existing));
    }

    #[test]
    fn federation_outcome_exposes_only_read_only_accessors() {
        let mut state = nodes();
        let outcome = deliver(&mut state, &envelope(), 50, true);

        assert_eq!(outcome.decision(), FederationDecision::AcceptedLocal);
        assert_eq!(outcome.authority(), AuthorityDisposition::LocalAuthority);
        assert_eq!(outcome.origin_node(), Some("node-a"));
        assert!(outcome.origin_node_known());
        assert_eq!(outcome.logical_delivery_id(), "delivery-1");
        assert_eq!(outcome.attempt_id(), "attempt-1");
        assert_eq!(
            outcome.reason(),
            "Delivery admitted without rewriting origin or minting local authority."
        );
    }

    #[test]
    fn origin_projection_distinguishes_claimed_from_model_known_identity() {
        let mut state = nodes();

        let accepted = deliver(&mut state, &envelope(), 50, true);
        assert!(accepted.origin_node_known);
        assert_eq!(accepted.origin_node.as_deref(), Some("node-a"));
        assert!(cockpit_projection("node-a", &accepted).origin_node_known);

        let partitioned = deliver(&mut state, &envelope(), 50, false);
        assert!(!partitioned.origin_node_known);
        assert_eq!(partitioned.origin_node.as_deref(), Some("node-a"));
        assert!(!cockpit_projection("node-a", &partitioned).origin_node_known);

        let mut unknown_origin = envelope();
        unknown_origin.origin_node = "node-unknown".into();
        let unknown = deliver(&mut state, &unknown_origin, 50, true);
        assert_eq!(unknown.decision, FederationDecision::UnknownNode);
        assert!(!unknown.origin_node_known);
        assert_eq!(unknown.origin_node.as_deref(), Some("node-unknown"));

        let mut unknown_target = envelope();
        unknown_target.target_node = "node-unknown".into();
        let target_unknown = deliver(&mut state, &unknown_target, 50, true);
        assert_eq!(target_unknown.decision, FederationDecision::UnknownNode);
        assert!(target_unknown.origin_node_known);
        assert_eq!(target_unknown.origin_node.as_deref(), Some("node-a"));

        let mut malformed = envelope();
        malformed.origin_node.clear();
        let malformed_outcome = deliver(&mut state, &malformed, 50, true);
        assert_eq!(malformed_outcome.decision, FederationDecision::Rejected);
        assert!(!malformed_outcome.origin_node_known);
    }

    #[test]
    fn empty_envelope_identity_fields_fail_closed_without_state_mutation() {
        let cases: [fn(&mut FederationEnvelope); 8] = [
            |e| e.envelope_id.clear(),
            |e| e.semantic_subject_id.clear(),
            |e| e.payload_commitment.clear(),
            |e| e.logical_delivery_id.clear(),
            |e| e.attempt_id.clear(),
            |e| e.origin_node.clear(),
            |e| e.target_node.clear(),
            |e| e.predecessor_delivery_id = Some(String::new()),
        ];

        for mutate in cases {
            let mut state = nodes();
            let mut malformed = envelope();
            mutate(&mut malformed);

            let outcome = deliver(&mut state, &malformed, 50, true);
            assert_eq!(outcome.decision, FederationDecision::Rejected);
            assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
            assert_eq!(state.delivery_count(), 0);
            assert_eq!(state.observation_count(), 0);
        }
    }

    #[test]
    fn explicit_non_active_authorization_states_fail_closed_without_mutation() {
        for authorization in [AuthorizationState::Expired, AuthorizationState::Revoked, AuthorizationState::Absent] {
            let mut state = nodes();
            let mut candidate = envelope();
            candidate.authorization = authorization;

            let outcome = deliver(&mut state, &candidate, 50, true);
            let expected = match authorization {
                AuthorizationState::Expired => FederationDecision::ExpiredAuthorization,
                AuthorizationState::Revoked | AuthorizationState::Absent => FederationDecision::Unauthorized,
                AuthorizationState::Active => unreachable!(),
            };
            assert_eq!(outcome.decision, expected, "unexpected decision for {authorization:?}");
            assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
            assert_eq!(state.delivery_count(), 0);
            assert_eq!(state.observation_count(), 0);
        }
    }

    #[test]
    fn replay_cannot_upgrade_explicitly_inactive_authorization_state() {
        for authorization in [
            AuthorizationState::Expired,
            AuthorizationState::Revoked,
            AuthorizationState::Absent,
        ] {
            let mut state = nodes();
            let candidate = envelope();

            assert_eq!(
                deliver(&mut state, &candidate, 50, true).decision,
                FederationDecision::AcceptedLocal
            );
            let before_identity = delivery_identity_snapshot(&state);
            let before_attempts = delivery_attempts_snapshot(&state);
            let before_observations = state.observations.clone();

            let mut replay = candidate.clone();
            replay.authorization = authorization;
            replay.attempt_id = format!("attempt-{authorization:?}-replay");

            let outcome = deliver(&mut state, &replay, 50, true);
            let expected = match authorization {
                AuthorizationState::Expired => FederationDecision::ExpiredAuthorization,
                AuthorizationState::Revoked | AuthorizationState::Absent => {
                    FederationDecision::Unauthorized
                }
                AuthorizationState::Active => unreachable!(),
            };
            assert_eq!(outcome.decision, expected);
            assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
            assert_eq!(delivery_identity_snapshot(&state), before_identity);
            assert_eq!(delivery_attempts_snapshot(&state), before_attempts);
            assert_eq!(state.observations, before_observations);
        }
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
    fn admitted_delivery_history_is_acyclic_by_construction() {
        let mut state = nodes();

        let parent = envelope();
        assert_eq!(
            deliver(&mut state, &parent, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        let mut child = envelope();
        child.envelope_id = "env-child".into();
        child.logical_delivery_id = "delivery-child".into();
        child.attempt_id = "attempt-child".into();
        child.predecessor_delivery_id = Some("delivery-1".into());
        assert_eq!(
            deliver(&mut state, &child, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        let mut grandchild = envelope();
        grandchild.envelope_id = "env-grandchild".into();
        grandchild.logical_delivery_id = "delivery-grandchild".into();
        grandchild.attempt_id = "attempt-grandchild".into();
        grandchild.predecessor_delivery_id = Some("delivery-child".into());
        assert_eq!(
            deliver(&mut state, &grandchild, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        assert_eq!(
            state
                .delivery("delivery-child")
                .unwrap()
                .contract()
                .predecessor_delivery_id
                .as_deref(),
            Some("delivery-1")
        );
        assert_eq!(
            state
                .delivery("delivery-grandchild")
                .unwrap()
                .contract()
                .predecessor_delivery_id
                .as_deref(),
            Some("delivery-child")
        );

        let mut cycle_attempt = grandchild.clone();
        cycle_attempt.attempt_id = "attempt-cycle".into();
        cycle_attempt.logical_delivery_id = "delivery-cycle".into();
        cycle_attempt.envelope_id = "env-cycle".into();
        cycle_attempt.predecessor_delivery_id = Some("delivery-grandchild".into());

        assert_eq!(
            deliver(&mut state, &cycle_attempt, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        let mut rebinding = grandchild;
        rebinding.attempt_id = "attempt-rebinding".into();
        rebinding.predecessor_delivery_id = Some("delivery-cycle".into());
        assert_eq!(
            deliver(&mut state, &rebinding, 50, true).decision,
            FederationDecision::ContractConflict
        );
        assert_eq!(
            state
                .delivery("delivery-grandchild")
                .unwrap()
                .contract()
                .predecessor_delivery_id
                .as_deref(),
            Some("delivery-child")
        );
        assert_eq!(state.delivery_count(), 4);
    }

    #[test]
    fn admitted_predecessor_is_historical_dependency_not_fresh_authority() {
        let mut state = nodes();

        let mut predecessor = envelope();
        predecessor.envelope_id = "env-foreign-parent".into();
        predecessor.logical_delivery_id = "delivery-foreign-parent".into();
        predecessor.attempt_id = "attempt-foreign-parent".into();
        predecessor.origin_node = "node-b".into();
        predecessor.target_node = "node-a".into();

        assert_eq!(
            deliver(&mut state, &predecessor, 50, true).decision,
            FederationDecision::AcceptedForeign
        );
        assert_eq!(
            state.delivery("delivery-foreign-parent").unwrap().authority(),
            AuthorityDisposition::ForeignEvidence
        );

        state.nodes.get_mut("node-b").unwrap().active = false;

        let mut child = envelope();
        child.envelope_id = "env-local-child".into();
        child.logical_delivery_id = "delivery-local-child".into();
        child.attempt_id = "attempt-local-child".into();
        child.predecessor_delivery_id = Some("delivery-foreign-parent".into());

        let outcome = deliver(&mut state, &child, 50, true);
        assert_eq!(outcome.decision, FederationDecision::AcceptedLocal);
        assert_eq!(outcome.authority, AuthorityDisposition::LocalAuthority);
        assert_eq!(state.delivery_count(), 2);
        assert_eq!(state.observation_count(), 2);
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
            origin_node_known: false,
            recognized_by: None,
            source_observation: false,
        };
        let b = ObservationRecord {
            observation_id: "obs-b".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:b".into(),
            origin_node: "node-b".into(),
            origin_node_known: false,
            recognized_by: None,
            source_observation: false,
        };
        assert_eq!(
            record_observation(&mut state, a),
            ObservationWriteResult::Inserted
        );
        assert_eq!(
            record_observation(&mut state, b),
            ObservationWriteResult::Inserted
        );
        assert_eq!(state.observations.len(), 2);
    }

    #[test]
    fn admitted_replay_cannot_bypass_generation_or_expiry_gates() {
        let mut schema_state = nodes();
        let candidate = envelope();
        assert_eq!(
            deliver(&mut schema_state, &candidate, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
        schema_state
            .nodes
            .get_mut("node-a")
            .unwrap()
            .schema_generation = 2;

        let mut schema_retry = candidate.clone();
        schema_retry.attempt_id = "attempt-schema-stale-retry".into();
        let schema_outcome = deliver(&mut schema_state, &schema_retry, 50, true);
        assert_eq!(
            schema_outcome.decision,
            FederationDecision::StaleGeneration
        );
        assert_eq!(schema_outcome.authority, AuthorityDisposition::NoAuthority);
        assert_eq!(
            schema_state.delivery("delivery-1").unwrap().attempts().len(),
            1
        );
        assert_eq!(schema_state.observation_count(), 1);

        let mut authorization_state = nodes();
        assert_eq!(
            deliver(&mut authorization_state, &candidate, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
        authorization_state
            .nodes
            .get_mut("node-a")
            .unwrap()
            .authorization_generation = 2;

        let mut authorization_retry = candidate.clone();
        authorization_retry.attempt_id = "attempt-auth-stale-retry".into();
        let authorization_outcome =
            deliver(&mut authorization_state, &authorization_retry, 50, true);
        assert_eq!(
            authorization_outcome.decision,
            FederationDecision::StaleGeneration
        );
        assert_eq!(
            authorization_outcome.authority,
            AuthorityDisposition::NoAuthority
        );
        assert_eq!(
            authorization_state
                .delivery("delivery-1")
                .unwrap()
                .attempts()
                .len(),
            1
        );
        assert_eq!(authorization_state.observation_count(), 1);

        let mut expiry_state = nodes();
        assert_eq!(
            deliver(&mut expiry_state, &candidate, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        let mut expired_retry = candidate;
        expired_retry.attempt_id = "attempt-expired-retry".into();
        let expired_outcome = deliver(&mut expiry_state, &expired_retry, 101, true);
        assert_eq!(
            expired_outcome.decision,
            FederationDecision::ExpiredAuthorization
        );
        assert_eq!(expired_outcome.authority, AuthorityDisposition::NoAuthority);
        assert_eq!(
            expiry_state.delivery("delivery-1").unwrap().attempts().len(),
            1
        );
        assert_eq!(expiry_state.observation_count(), 1);
    }

    #[test]
    fn admitted_replay_cannot_bypass_node_deactivation() {
        let mut state = nodes();
        let candidate = envelope();

        assert_eq!(
            deliver(&mut state, &candidate, 50, true).decision,
            FederationDecision::AcceptedLocal
        );

        state.nodes.get_mut("node-a").unwrap().active = false;

        let mut retry = candidate;
        retry.attempt_id = "attempt-inactive-retry".into();
        let outcome = deliver(&mut state, &retry, 50, true);
        assert_eq!(outcome.decision, FederationDecision::Unauthorized);
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert_eq!(state.delivery("delivery-1").unwrap().attempts().len(), 1);
        assert_eq!(state.observation_count(), 1);
    }

    #[test]
    fn expiry_is_exclusive_at_the_declared_expiration_instant() {
        let mut state = nodes();
        let mut expiring = envelope();
        expiring.expires_at = Some(100);

        assert_eq!(
            deliver(&mut state, &expiring, 99, true).decision,
            FederationDecision::AcceptedLocal
        );

        let mut retry = expiring;
        retry.attempt_id = "attempt-at-expiry".into();
        let outcome = deliver(&mut state, &retry, 100, true);
        assert_eq!(
            outcome.decision,
            FederationDecision::ExpiredAuthorization
        );
        assert_eq!(outcome.authority, AuthorityDisposition::NoAuthority);
        assert_eq!(
            state.delivery("delivery-1").unwrap().attempts().len(),
            1
        );
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
        assert!(projection.origin_node_known);
        assert!(!projection.source_observation);
        assert!(!projection.may_authorize_local_action);

        let partitioned = deliver(&mut state, &envelope(), 50, false);
        let partition_projection = privacy_minimized_projection(&partitioned, "subject-1");
        assert_eq!(partition_projection.origin_node, "node-a");
        assert!(!partition_projection.origin_node_known);
        assert!(!partition_projection.source_observation);
        assert!(!partition_projection.may_authorize_local_action);
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
    fn scenario_oracle_is_invariant_to_recognition_input_order() {
        let mut forward = nodes();
        forward.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });
        forward.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-c".into(),
            scope: "subject-2".into(),
            mode: RecognitionMode::DelegatedAuthority,
        });

        let mut reverse = nodes();
        reverse.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-c".into(),
            scope: "subject-2".into(),
            mode: RecognitionMode::DelegatedAuthority,
        });
        reverse.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });

        let mut candidate = envelope();
        candidate.envelope_id = "env-recognition-order".into();
        candidate.logical_delivery_id = "delivery-recognition-order".into();
        candidate.origin_node = "node-b".into();
        candidate.target_node = "node-a".into();

        let steps = [
            FederationScenarioStep {
                mutation: FederationMutation::NewLogicalDelivery,
                expected: FederationDecision::AcceptedForeign,
                expected_authority: AuthorityDisposition::RecognizedForeignEvidence,
            },
            FederationScenarioStep {
                mutation: FederationMutation::DuplicateDelivery,
                expected: FederationDecision::Duplicate,
                expected_authority: AuthorityDisposition::RecognizedForeignEvidence,
            },
        ];

        assert_eq!(
            run_scenario(forward, candidate.clone(), &steps, 50),
            run_scenario(reverse, candidate, &steps, 50)
        );
    }

    #[test]
    fn scenario_corpus_matches_declared_mutation_set() {
        let steps = scenario_steps();
        let declared = ALL_FEDERATION_MUTATIONS.iter().copied().collect::<BTreeSet<_>>();
        let exercised = steps
            .iter()
            .map(|step| step.mutation)
            .collect::<BTreeSet<_>>();

        assert_eq!(
            steps.len(),
            declared.len(),
            "scenario corpus contains duplicate or missing mutation declarations"
        );
        assert_eq!(exercised, declared);
    }

    #[test]
    fn scenario_oracle_is_deterministic_and_claim_bounded() {
        let initial = nodes();
        let candidate = envelope();
        let steps = scenario_steps();

        let results = run_scenario(initial.clone(), candidate.clone(), &steps, 50);
        let repeated = run_scenario(initial, candidate, &steps, 50);

        assert_eq!(results, repeated);
        assert!(results.iter().all(|result| result.passed));

        for result in &results {
            if matches!(
                result.mutation,
                FederationMutation::ForeignEvidence
                    | FederationMutation::DuplicateDelivery
                    | FederationMutation::NewLogicalDelivery
                    | FederationMutation::LocalEvidence
                    | FederationMutation::Reconnect
            ) {
                // Acceptance/replay variants are the only scenario cases that may carry authority.
                assert_ne!(
                    result.actual_authority,
                    AuthorityDisposition::NoAuthority,
                    "accepted/replay mutation unexpectedly lost authority: {:?}",
                    result.mutation
                );
            } else {
                assert_eq!(
                    result.actual_authority,
                    AuthorityDisposition::NoAuthority,
                    "non-authorizing mutation carried authority: {:?}",
                    result.mutation
                );
            }

            match result.mutation {
                FederationMutation::LocalEvidence
                | FederationMutation::ForeignEvidence
                | FederationMutation::NewLogicalDelivery => {
                    assert!(!result.delivery_identity_unchanged);
                    assert!(!result.delivery_attempts_unchanged);
                    assert!(!result.observation_ledger_unchanged);
                }
                FederationMutation::DuplicateDelivery | FederationMutation::Reconnect => {
                    assert!(result.delivery_identity_unchanged);
                    assert!(!result.delivery_attempts_unchanged);
                    assert!(result.observation_ledger_unchanged);
                }
                _ => {
                    assert!(
                        result.delivery_identity_unchanged,
                        "rejected/non-new mutation changed delivery identity state: {:?}",
                        result.mutation
                    );
                    assert!(
                        result.delivery_attempts_unchanged,
                        "rejected/non-replay mutation changed delivery attempt state: {:?}",
                        result.mutation
                    );
                    assert!(
                        result.observation_ledger_unchanged,
                        "rejected/non-new mutation changed observation state: {:?}",
                        result.mutation
                    );
                }
            }
        }
    }
}

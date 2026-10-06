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
#[serde(deny_unknown_fields)]
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
#[serde(deny_unknown_fields)]
pub struct RecognitionEdge {
    pub recognizing_node: String,
    pub origin_node: String,
    pub scope: String,
    pub mode: RecognitionMode,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
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
#[serde(deny_unknown_fields)]
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
/// ```compile_fail
/// # use cos_conformance::federation::{AuthorityDisposition, DeliveryRecord, ImmutableDeliveryContract};
/// # use std::collections::{BTreeMap, BTreeSet};
/// let _ = DeliveryRecord {
///     contract: ImmutableDeliveryContract {
///         logical_delivery_id: "delivery-1".into(),
///         origin_node: "node-a".into(),
///         target_node: "node-a".into(),
///         semantic_subject_id: "subject-1".into(),
///         payload_commitment: "sha256:test".into(),
///         schema_generation: 1,
///         authorization_generation: 1,
///         predecessor_delivery_id: None,
///         expires_at: None,
///     },
///     authority: AuthorityDisposition::LocalAuthority,
///     attempts: BTreeSet::from(["attempt-1".into()]),
///     attempt_envelope_ids: BTreeMap::from([("attempt-1".into(), "env-1".into())]),
///     source_observation_id: "env-1".into(),
///     source_observation_recognized_by: None,
/// };
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DeliveryRecord {
    contract: ImmutableDeliveryContract,
    authority: AuthorityDisposition,
    /// Monotone admission ordinal assigned by the reference transition path.
    admission_index: u64,
    attempts: BTreeSet<String>,
    /// Binds each transport attempt to the envelope identity that carried it.
    /// Attempt identity remains separate from immutable logical-delivery identity.
    attempt_envelope_ids: BTreeMap<String, String>,
    /// Identifier of the source observation minted by the admitting transition.
    /// This provenance link is not part of the immutable logical delivery contract.
    source_observation_id: String,
    /// Recognition provenance captured at admission; later recognition changes
    /// do not rewrite the admitted delivery's provenance.
    source_observation_recognized_by: Option<String>,
}

impl DeliveryRecord {
    pub fn contract(&self) -> &ImmutableDeliveryContract {
        &self.contract
    }

    pub fn authority(&self) -> AuthorityDisposition {
        self.authority
    }

    pub fn admission_index(&self) -> u64 {
        self.admission_index
    }

    pub fn attempts(&self) -> &BTreeSet<String> {
        &self.attempts
    }

    pub fn source_observation_id(&self) -> &str {
        &self.source_observation_id
    }

    pub fn source_observation_recognized_by(&self) -> Option<&str> {
        self.source_observation_recognized_by.as_deref()
    }

    pub fn attempt_envelope_id(&self, attempt_id: &str) -> Option<&str> {
        self.attempt_envelope_ids.get(attempt_id).map(String::as_str)
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
    /// Next admission ordinal consumed only by successful admissions.
    next_admission_index: u64,
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
            next_admission_index: 0,
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
    AttemptConflict,
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
        self.origin_node.as_deref()    }

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
        return FederationOutcome::with_origin_knowledge(
            FederationDecision::PartitionUnknown,
            AuthorityDisposition::NoAuthority,
            envelope,
            "Transport unavailability does not establish delivery success.",
            state.nodes.contains_key(&envelope.origin_node),
        );
    }

    if let Some(existing) = state.deliveries.get(&envelope.logical_delivery_id) {
        if let Some(bound_envelope_id) = existing.attempt_envelope_ids.get(&envelope.attempt_id) {
            if bound_envelope_id != &envelope.envelope_id {
                return FederationOutcome::known_origin(
                    FederationDecision::AttemptConflict,
                    AuthorityDisposition::NoAuthority,
                    envelope,
                    "A transport attempt identity cannot be rebound to a different envelope identity.",
                );
            }
        }
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
        existing.attempt_envelope_ids
            .insert(envelope.attempt_id.clone(), envelope.envelope_id.clone());
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

    let next_admission_index = match state.next_admission_index.checked_add(1) {
        Some(next) => next,
        None => {
            return FederationOutcome::known_origin(
                FederationDecision::Rejected,
                AuthorityDisposition::NoAuthority,
                envelope,
                "The reference federation has exhausted its admission ordinal space.",
            );
        }
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

    let admission_index = state.next_admission_index;
    let prior = state.deliveries.insert(
        envelope.logical_delivery_id.clone(),
        DeliveryRecord {
            contract: ImmutableDeliveryContract::from(envelope),
            authority,
            admission_index,
            attempts: BTreeSet::from([envelope.attempt_id.clone()]),
            attempt_envelope_ids: BTreeMap::from([(
                envelope.attempt_id.clone(),
                envelope.envelope_id.clone(),
            )]),
            source_observation_id: envelope.envelope_id.clone(),
            source_observation_recognized_by: if foreign && recognition.is_some() {
                Some(envelope.target_node.clone())
            } else {
                None
            },
        },
    );
    debug_assert!(prior.is_none());
    state.next_admission_index = next_admission_index;

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
/// ```#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
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
    AttemptIdentityMutation,
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
        FederationMutation::AttemptIdentityMutation => FederationDecision::AttemptConflict,
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
        | FederationMutation::ContractExpiryMutation
        | FederationMutation::AttemptIdentityMutation => AuthorityDisposition::NoAuthority,
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

fn source_observation_count(state: &FederationState) -> usize {
    state
        .observations
        .values()
        .filter(|observation| observation.source_observation)
        .count()
}

fn delivery_attempts_snapshot(
    state: &FederationState,
) -> BTreeMap<String, (BTreeSet<String>, BTreeMap<String, String>)> {
    state
        .deliveries
        .iter()
        .map(|(id, record)| {
            (
                id.clone(),
                (record.attempts.clone(), record.attempt_envelope_ids.clone()),
            )
        })
        .collect()
}

fn attempt_history_matches_bindings(record: &DeliveryRecord) -> bool {
    record.attempts.len() == record.attempt_envelope_ids.len()
        && record
            .attempts
            .iter()
            .all(|attempt_id| record.attempt_envelope_ids.contains_key(attempt_id))
}

fn source_observation_matches_delivery(state: &FederationState, record: &DeliveryRecord) -> bool {
    let Some(observation) = state.observations.get(record.source_observation_id()) else {
        return false;
    };

    let contract = record.contract();

    observation.source_observation
        && observation.observation_id == record.source_observation_id()
        && observation.semantic_subject_id == contract.semantic_subject_id
        && observation.payload_commitment == contract.payload_commitment
        && observation.origin_node == contract.origin_node
        && observation.origin_node_known
        && observation.recognized_by.as_deref()
            == record.source_observation_recognized_by.as_deref()
}

fn delivery_map_keys_match_contracts(state: &FederationState) -> bool {
    state.deliveries.iter().all(|(map_id, record)| {
        map_id == &record.contract.logical_delivery_id
    })
}

fn observation_map_keys_match_records(state: &FederationState) -> bool {
    state.observations.iter().all(|(map_id, observation)| {
        map_id == &observation.observation_id
    })
}

/// Typed diagnostics for corruption or invariant drift at the federation state boundary.
///
/// This validator is observational only: it never repairs, normalizes, or mutates
/// state. Callers can therefore use it as a qualification gate without granting
/// the validator any authority to rewrite evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FederationInvariantViolation {
    NodeMapKeyMismatch, RecognitionEdgeInvalid, RecognitionEdgeOrderMismatch,
    DeliveryMapKeyMismatch, DeliveryNodeReferenceMismatch, DeliveryPredecessorMismatch,
    DeliveryAdmissionOrderMismatch, ObservationMapKeyMismatch, SourceObservationSetMismatch,
    DeliveryMissingAttemptHistory, AttemptBindingMismatch, SourceObservationMismatch,
}

/// Stable identifiers for the authoritative federation invariant registry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FederationInvariantId {
    NodeMapIdentity, RecognitionEdgeValidity, RecognitionEdgeCanonicalOrder,
    DeliveryMapIdentity, DeliveryNodeReferences, DeliveryPredecessorReferences,
    DeliveryAdmissionOrder, ObservationMapIdentity, SourceObservationBijection,
    DeliveryAttemptHistory, AttemptEnvelopeBindings, DeliverySourceObservationProvenance,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FederationInvariantSpec {
    pub id: FederationInvariantId,
    pub name: &'static str,
    pub description: &'static str,
    /// Invariants that semantically precede this check.
    pub dependencies: &'static [FederationInvariantId],
    check: fn(&FederationState) -> bool,
    violation: FederationInvariantViolation,
}

impl FederationInvariantSpec {
    pub const fn new(
        id: FederationInvariantId,
        name: &'static str,
        description: &'static str,
        dependencies: &'static [FederationInvariantId],
        check: fn(&FederationState) -> bool,
        violation: FederationInvariantViolation,
    ) -> Self {
        Self { id, name, description, dependencies, check, violation }
    }
}

/// Canonical executable inventory of the federation state-boundary invariants.
pub const FEDERATION_INVARIANT_REGISTRY: &[FederationInvariantSpec] = &[
    FederationInvariantSpec::new(FederationInvariantId::NodeMapIdentity, "node-map-identity",
        "Every node map key equals its NodeProfile node_id and node IDs are non-empty.",
        &[], node_map_keys_match_profiles, FederationInvariantViolation::NodeMapKeyMismatch),
    FederationInvariantSpec::new(FederationInvariantId::RecognitionEdgeValidity, "recognition-edge-validity",
        "Recognition edges have non-empty identities and reference known nodes.",
        &[], recognition_edges_are_valid, FederationInvariantViolation::RecognitionEdgeInvalid),
    FederationInvariantSpec::new(FederationInvariantId::RecognitionEdgeCanonicalOrder, "recognition-edge-canonical-order",
        "Recognition edges are strictly ordered by the canonical identity tuple.",
        &[FederationInvariantId::RecognitionEdgeValidity], recognition_edges_are_canonically_ordered, FederationInvariantViolation::RecognitionEdgeOrderMismatch),
    FederationInvariantSpec::new(FederationInvariantId::DeliveryMapIdentity, "delivery-map-identity",
        "Every delivery map key equals its immutable logical delivery identity.",
        &[], delivery_map_keys_match_contracts, FederationInvariantViolation::DeliveryMapKeyMismatch),
    FederationInvariantSpec::new(FederationInvariantId::DeliveryNodeReferences, "delivery-node-references",
        "Admitted deliveries reference known origin and target nodes and required identities are non-empty.",
        &[], delivery_node_references_match, FederationInvariantViolation::DeliveryNodeReferenceMismatch),
    FederationInvariantSpec::new(FederationInvariantId::DeliveryPredecessorReferences, "delivery-predecessor-references",
        "Every predecessor reference exists and no delivery points to itself.",
        &[FederationInvariantId::DeliveryMapIdentity], delivery_predecessors_match, FederationInvariantViolation::DeliveryPredecessorMismatch),
    FederationInvariantSpec::new(FederationInvariantId::DeliveryAdmissionOrder, "delivery-admission-order",
        "Admission ordinals are unique and contiguous, and every predecessor strictly precedes its child.",
        &[FederationInvariantId::DeliveryMapIdentity, FederationInvariantId::DeliveryPredecessorReferences],
        delivery_admission_order_matches, FederationInvariantViolation::DeliveryAdmissionOrderMismatch),
    FederationInvariantSpec::new(FederationInvariantId::ObservationMapIdentity, "observation-map-identity",
        "Every observation map key equals its observation identity.",
        &[], observation_map_keys_match_records, FederationInvariantViolation::ObservationMapKeyMismatch),
    FederationInvariantSpec::new(FederationInvariantId::SourceObservationBijection, "source-observation-bijection",
        "Admitted deliveries and source observations form a one-to-one identity set.",
        &[FederationInvariantId::DeliveryMapIdentity, FederationInvariantId::ObservationMapIdentity], source_observation_ids_match_delivery_links, FederationInvariantViolation::SourceObservationSetMismatch),
    FederationInvariantSpec::new(FederationInvariantId::DeliveryAttemptHistory, "delivery-attempt-history",
        "Every admitted delivery retains at least one transport-attempt identity.",
        &[], deliveries_have_attempt_history, FederationInvariantViolation::DeliveryMissingAttemptHistory),
    FederationInvariantSpec::new(FederationInvariantId::AttemptEnvelopeBindings, "attempt-envelope-bindings",
        "Each attempt identity has exactly one bound envelope identity and vice versa.",
        &[], delivery_attempt_bindings_match, FederationInvariantViolation::AttemptBindingMismatch),
    FederationInvariantSpec::new(FederationInvariantId::DeliverySourceObservationProvenance, "delivery-source-observation-provenance",
        "Each admitted delivery matches its immutable source observation and captured recognition provenance.",
        &[FederationInvariantId::DeliveryMapIdentity, FederationInvariantId::ObservationMapIdentity, FederationInvariantId::SourceObservationBijection], delivery_source_observation_provenance_matches, FederationInvariantViolation::SourceObservationMismatch),
];

fn node_map_keys_match_profiles(state: &FederationState) -> bool {
    state.nodes.iter().all(|(map_id, node)| map_id == &node.node_id && !node.node_id.is_empty())
}

fn recognition_edges_are_valid(state: &FederationState) -> bool {
    state.recognition_edges.iter().all(|edge| {
        !edge.recognizing_node.is_empty() && !edge.origin_node.is_empty() && !edge.scope.is_empty()
            && state.nodes.contains_key(&edge.recognizing_node) && state.nodes.contains_key(&edge.origin_node)
    })
}

fn recognition_edges_are_canonically_ordered(state: &FederationState) -> bool {
    state.recognition_edges.windows(2).all(|pair| {
        (&pair[0].recognizing_node, &pair[0].origin_node, &pair[0].scope, pair[0].mode)
            < (&pair[1].recognizing_node, &pair[1].origin_node, &pair[1].scope, pair[1].mode)
    })
}

fn delivery_node_references_match(state: &FederationState) -> bool {
    state.deliveries.values().all(|record| {
        state.nodes.contains_key(&record.contract.origin_node)
            && state.nodes.contains_key(&record.contract.target_node)
            && !record.contract.logical_delivery_id.is_empty()
            && !record.contract.semantic_subject_id.is_empty()
            && !record.contract.payload_commitment.is_empty()
            && !record.source_observation_id.is_empty()
    })
}

fn delivery_predecessors_match(state: &FederationState) -> bool {
    state.deliveries.values().all(|record| match record.contract.predecessor_delivery_id.as_deref() {
        None => true,
        Some(predecessor) => predecessor != record.contract.logical_delivery_id && state.deliveries.contains_key(predecessor),
    })
}

fn delivery_admission_order_matches(state: &FederationState) -> bool {
    let indexes = state
        .deliveries
        .values()
        .map(DeliveryRecord::admission_index)
        .collect::<BTreeSet<_>>();

    if indexes.len() != state.deliveries.len() {
        return false;
    }

    let expected = (0..state.next_admission_index).collect::<BTreeSet<_>>();
    if indexes != expected {
        return false;
    }

    state.deliveries.values().all(|record| {
        match record.contract.predecessor_delivery_id.as_deref() {
            None => true,
            Some(predecessor) => state
                .deliveries
                .get(predecessor)
                .is_some_and(|parent| parent.admission_index() < record.admission_index()),
        }
    })
}

fn deliveries_have_attempt_history(state: &FederationState) -> bool {
    state.deliveries.values().all(|record| !record.attempts.is_empty())
}

fn delivery_attempt_bindings_match(state: &FederationState) -> bool {
    state.deliveries.values().all(attempt_history_matches_bindings)
}

fn delivery_source_observation_provenance_matches(state: &FederationState) -> bool {
    state.deliveries.values().all(|record| source_observation_matches_delivery(state, record))
}

/// Returns every invariant violated by the supplied authoritative state, in
/// deterministic registry order.
///
/// This is the audit form of qualification: unlike `validate_state`, it does
/// not stop at the first failure. It is still observational and never mutates
/// or repairs the supplied state.
pub fn validate_state_all(
    state: &FederationState,
) -> Vec<(FederationInvariantId, FederationInvariantViolation)> {
    FEDERATION_INVARIANT_REGISTRY
        .iter()
        .filter(|spec| !(spec.check)(state))
        .map(|spec| (spec.id, spec.violation))
        .collect()
}

/// Classification emitted by the structured audit path.
///
/// Violated means the predicate itself failed. The current audit evaluates
/// every predicate so derived corruption remains observable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FederationInvariantAuditStatus {
    Passed,
    Violated(FederationInvariantViolation),
}

/// One deterministic audit result for an invariant registry entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FederationInvariantAuditEntry {
    pub id: FederationInvariantId,
    pub status: FederationInvariantAuditStatus,
}

/// Evaluates every registered invariant and preserves registry order in a
/// structured audit result without mutating authoritative state.
pub fn audit_state(state: &FederationState) -> Vec<FederationInvariantAuditEntry> {
    FEDERATION_INVARIANT_REGISTRY
        .iter()
        .map(|spec| FederationInvariantAuditEntry {
            id: spec.id,
            status: if (spec.check)(state) {
                FederationInvariantAuditStatus::Passed
            } else {
                FederationInvariantAuditStatus::Violated(spec.violation)
            },
        })
        .collect()
}

/// Returns the first violated invariant in canonical registry order.
pub fn validate_state(state: &FederationState) -> Result<(), FederationInvariantViolation> {
    validate_state_all(state)
        .into_iter()
        .next()
        .map_or(Ok(()), |(_, violation)| Err(violation))
}

/// Checks that every declared invariant dependency appears earlier in the
/// canonical registry. This makes registry order a semantic topological-order
/// contract rather than an incidental presentation choice.
pub fn invariant_registry_dependencies_are_ordered() -> bool {
    FEDERATION_INVARIANT_REGISTRY.iter().enumerate().all(|(index, spec)| {
        spec.dependencies.iter().all(|dependency| {
            FEDERATION_INVARIANT_REGISTRY[..index]
                .iter()
                .any(|prior| prior.id == *dependency)
        })
    })
}

/// Boolean compatibility wrapper for existing transition assertions.
fn federation_state_invariants_hold(state: &FederationState) -> bool { validate_state(state).is_ok() }

/// Canonical, order-independent representation of authoritative federation state.
///
/// The representation intentionally includes semantic state, provenance ledgers,
/// and the admission-order witness, while relying on BTreeMap ordering and the
/// already-normalized recognition edge ordering to eliminate incidental container
/// ordering from qualification evidence.
/// It is a representation for deterministic comparison, not a cryptographic hash.
pub fn canonical_state_fingerprint(state: &FederationState) -> Vec<u8> {
    serde_json::to_vec(&(
        &state.nodes,
        &state.recognition_edges,
        &state.deliveries,
        &state.observations,
        &state.next_admission_index,
    ))
    .expect("authoritative federation state is serializable")
}

fn source_observation_ids_match_delivery_links(state: &FederationState) -> bool {
    let linked_ids = state
        .deliveries
        .values()
        .map(DeliveryRecord::source_observation_id)
        .collect::<BTreeSet<_>>();

    let source_ids = state
        .observations
        .values()
        .filter(|observation| observation.source_observation)
        .map(|observation| observation.observation_id.as_str())
        .collect::<BTreeSet<_>>();

    linked_ids.len() == state.delivery_count()
        && linked_ids == source_ids
        && source_ids.len() == state.delivery_count()
}

fn delivery_identity_snapshot(
    state: &FederationState,
) -> BTreeMap<String, (
        ImmutableDeliveryContract,
        AuthorityDisposition,
        String,
        Option<String>,
    )> {
    state
        .deliveries
        .iter()
        .map(|(id, record)| {
            (
                id.clone(),
                (
                    record.contract.clone(),
                    record.authority,
                    record.source_observation_id.clone(),
                    record.source_observation_recognized_by.clone(),
                ),
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
            FederationMutation::AttemptIdentityMutation => {
                candidate.envelope_id = format!("{}-attempt-rebound", candidate.envelope_id);
            }
        }

        let outcome = deliver(&mut state, &candidate, now, transport_available);
        debug_assert!(
            federation_state_invariants_hold(&state),
            "federation transition violated an internal state invariant"
        );
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
            delivery_identity_unchanged,            delivery_attempts_unchanged,
            observation_ledger_unchanged,
        });
    }

    results
    #[test]
    fn external_evidence_anchor_reference_binds_exact_subject_and_witness_artifact() {
        let left =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(71, 8),
            )
            .expect("left capsule must deserialize");
        let right =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(71, 12),
            )
            .expect("right capsule must deserialize");
        let left_publication = state_machine_trace_checkpoint_publication(
            &left,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let right_publication = state_machine_trace_checkpoint_publication(
            &right,
            12,
            &left_publication.publication_sha256,
        );
        let receipt =
            state_machine_trace_publication_collection_reconciliation(
                &[(&left, &left_publication)],
                &[(&left, &left_publication), (&right, &right_publication)],
            )
            .expect("reconciliation receipt must build");
        let witness_artifact =
            serde_json::to_vec(&receipt).expect("reconciliation receipt must serialize");
        let reference = state_machine_trace_external_evidence_anchor_reference(
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE,
            &receipt.reconciliation_sha256,
            FederationStateMachineTraceExternalWitnessKind::ArchiveEvidenceRecord,
            1,
            "external-archive-evidence-record-v1",
            &witness_artifact,
            1_791_000_000,
        )
        .expect("external anchor reference must build");

        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE,
                &receipt.reconciliation_sha256,
                &witness_artifact,
                &reference,
            ),
            Ok(())
        );

        let round_trip = serde_json::to_string(&reference)
            .expect("external anchor reference must serialize");
        assert_eq!(
            serde_json::from_str::<
                FederationStateMachineTraceExternalEvidenceAnchorReference,
            >(&round_trip)
            .expect("external anchor reference must deserialize"),
            reference
        );
    }

    #[test]
    fn external_evidence_anchor_reference_detects_subject_witness_and_self_digest_tampering() {
        let subject_profile =
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE;
        let subject_digest = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let witness_artifact = b"external-witness-artifact-v1";
        let mut reference =
            state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                FederationStateMachineTraceExternalWitnessKind::TransparencyLogHead,
                2,
                "ct-log-sth-v2",
                witness_artifact,
                1_791_000_123,
            )
            .expect("external anchor reference must build");

        reference.subject_hash_algorithm = "sha-512".into();
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                witness_artifact,
                &reference,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::SubjectHashAlgorithmMismatch
            )
        );

        reference.subject_hash_algorithm =
            FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SUBJECT_HASH_ALGORITHM.into();
        reference.witness_schema_version = 0;
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                witness_artifact,
                &reference,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessSchemaVersionMismatch
            )
        );

        reference.witness_schema_version = 2;
        reference.witness_hash_encoding = "raw-bytes-v1".into();
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                witness_artifact,
                &reference,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessHashEncodingMismatch
            )
        );

        reference.witness_hash_encoding =
            FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_WITNESS_HASH_ENCODING.into();
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);

        reference.subject_sha256 =
            "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into();
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                witness_artifact,
                &reference,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::SubjectDigestMismatch
            )
        );

        reference.subject_sha256 = subject_digest.into();
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        let changed_witness = b"external-witness-artifact-v2";
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                changed_witness,
                &reference,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessDigestMismatch
            )
        );

        reference.witness_sha256 = "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into();
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                witness_artifact,
                &reference,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessDigestMismatch
            )
        );

        reference.witness_sha256 =
            state_machine_trace_external_witness_artifact_sha256(witness_artifact);
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        reference.claimed_observed_at_unix_seconds += 1;
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                witness_artifact,
                &reference,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::AnchorReferenceDigestMismatch
            )
        );

        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                subject_profile,
                subject_digest,
                witness_artifact,
                &reference,
            ),
            Ok(())
        );
    }

    #[test]
    fn external_evidence_anchor_reference_rejects_empty_inputs_and_unknown_fields() {
        assert_eq!(
            state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                "",
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                FederationStateMachineTraceExternalWitnessKind::Other,
                1,
                "external-v1",
                b"witness",
                0,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::EmptySubjectProfile
            )
        );
        assert_eq!(
            state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE,
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                FederationStateMachineTraceExternalWitnessKind::Other,
                0,
                "external-v1",
                b"witness",
                0,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::InvalidWitnessSchemaVersion
            )
        );

        assert_eq!(
            state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE,
                "sha256:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
                FederationStateMachineTraceExternalWitnessKind::Other,
                1,
                "external-v1",
                b"witness",
                0,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::InvalidSubjectDigest
            )
        );

        assert_eq!(
            state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE,
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                FederationStateMachineTraceExternalWitnessKind::Other,
                1,
                "external-v1",
                b"witness",
                0,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::InvalidSubjectDigest
            )
        );
        assert_eq!(
            state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE,
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                FederationStateMachineTraceExternalWitnessKind::Other,
                "",
                b"witness",
                0,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::EmptyWitnessProfile
            )
        );
        assert_eq!(
            state_machine_trace_external_evidence_anchor_reference(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE,
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                FederationStateMachineTraceExternalWitnessKind::Other,
                1,
                "external-v1",
                b"",
                0,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::EmptyWitnessArtifact
            )
        );

        let reference =
            state_machine_trace_external_evidence_anchor_reference(
                1,
                "subject-profile-v1",
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                FederationStateMachineTraceExternalWitnessKind::TimestampToken,
                1,
                "tsa-token-v1",
                b"token",
                1_791_000_001,
            )
            .expect("reference must build");
        let mut unknown = serde_json::to_value(&reference)
            .expect("external anchor reference must serialize");
        unknown
            .as_object_mut()
            .expect("external anchor reference must be an object")
            .insert("unexpected_anchor_field".into(), serde_json::Value::Bool(true));
        assert!(
            serde_json::from_value::<
                FederationStateMachineTraceExternalEvidenceAnchorReference,
            >(unknown)
            .is_err()
        );
    }

    #[test]
    fn external_evidence_anchor_reference_is_not_an_authority_or_timestamp_proof() {
        let witness_artifact = b"opaque-external-proof";
        let reference =
            state_machine_trace_external_evidence_anchor_reference(
                1,
                "subject-profile-v1",
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                FederationStateMachineTraceExternalWitnessKind::TimestampToken,
                1,
                "tsa-token-v1",
                witness_artifact,
                u64::MAX,
            )
            .expect("reference must build");

        assert_eq!(
            validate_state_machine_trace_external_evidence_anchor_reference(
                1,
                "subject-profile-v1",
                "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                witness_artifact,
                &reference,
            ),
            Ok(())
        );
    }


    #[test]
    fn external_evidence_verification_statement_chain_validates_end_to_end_binding() {
        let subject_schema_version =
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION;
        let subject_profile =
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE;
        let subject_sha256 =
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let anchor_witness = b"externally-retained-anchor-record";
        let anchor_reference =
            state_machine_trace_external_evidence_anchor_reference(
                subject_schema_version,
                subject_profile,
                subject_sha256,
                FederationStateMachineTraceExternalWitnessKind::ArchiveEvidenceRecord,
                1,
                "external-archive-evidence-record-v1",
                anchor_witness,
                1_791_000_600,
            )
            .expect("anchor reference must build");
        let verifier_report = b"external-verifier-report-v1";
        let statement =
            state_machine_trace_external_evidence_verification_statement(
                &anchor_reference.anchor_reference_sha256,
                1,
                "archive-verifier-v1",
                FederationStateMachineTraceExternalVerificationClaim::ArchiveEvidenceVerified,
                verifier_report,
                1_791_000_601,
            )
            .expect("verification statement must build");

        let result =
            validate_state_machine_trace_external_evidence_verification_statement_chain(
                subject_schema_version,
                subject_profile,
                subject_sha256,
                anchor_witness,
                &anchor_reference,
                verifier_report,
                &statement,
            )
            .expect("validated external evidence should yield a typed result");

        assert_eq!(
            result.anchor_reference_sha256(),
            anchor_reference.anchor_reference_sha256
        );
        assert_eq!(
            result.anchor_reference_schema_version(),
            anchor_reference.schema_version
        );
        assert_eq!(
            result.anchor_reference_profile(),
            anchor_reference.reference_profile
        );
        assert_eq!(result.verifier_schema_version(), 1);
        assert_eq!(result.verifier_profile(), "archive-verifier-v1");
        assert_eq!(
            result.claim(),
            FederationStateMachineTraceExternalVerificationClaim::ArchiveEvidenceVerified
        );
        assert_eq!(
            result.verifier_report_sha256(),
            state_machine_trace_external_witness_artifact_sha256(verifier_report)
        );
        assert_eq!(result.claimed_verified_at_unix_seconds(), 1_791_000_601);
        assert_eq!(result.statement_sha256(), statement.statement_sha256);

        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_chain(
                subject_schema_version,
                subject_profile,
                subject_sha256,
                b"changed-anchor-record",
                &anchor_reference,
                verifier_report,
                &statement,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceInvalid(
                    FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessDigestMismatch
                )
            )
        );

        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_chain(
                subject_schema_version,
                subject_profile,
                subject_sha256,
                anchor_witness,
                &anchor_reference,
                b"changed-verifier-report",
                &statement,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::VerifierReportDigestMismatch
            )
        );
    }

    #[test]
    fn external_verification_result_tracks_resealed_external_claim_without_self_authenticating() {
        let subject_schema_version =
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION;
        let subject_profile =
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE;
        let subject_sha256 =
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let witness_artifact = b"external-anchor";
        let anchor_reference = state_machine_trace_external_evidence_anchor_reference(
            subject_schema_version,
            subject_profile,
            subject_sha256,
            FederationStateMachineTraceExternalWitnessKind::TimestampToken,
            1,
            "tsa-token-v1",
            witness_artifact,
            1_791_001_000,
        )
        .expect("anchor reference must build");

        let verifier_report = b"verifier-report";
        let mut statement = state_machine_trace_external_evidence_verification_statement(
            &anchor_reference.anchor_reference_sha256,
            1,
            "rfc3161-verifier-v1",
            FederationStateMachineTraceExternalVerificationClaim::TimestampTokenVerified,
            verifier_report,
            1_791_001_001,
        )
        .expect("verification statement must build");

        let first =
            validate_state_machine_trace_external_evidence_verification_statement_chain(
                subject_schema_version,
                subject_profile,
                subject_sha256,
                witness_artifact,
                &anchor_reference,
                verifier_report,
                &statement,
            )
            .expect("initial statement should bind");

        assert_eq!(
            first.claim(),
            FederationStateMachineTraceExternalVerificationClaim::TimestampTokenVerified
        );

        statement.verification_claim =
            FederationStateMachineTraceExternalVerificationClaim::ArchiveEvidenceVerified;
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);

        let resealed =
            validate_state_machine_trace_external_evidence_verification_statement_chain(
                subject_schema_version,
                subject_profile,
                subject_sha256,
                witness_artifact,
                &anchor_reference,
                verifier_report,
                &statement,
            )
            .expect("re-sealed external claim remains a structurally valid assertion");

        assert_eq!(
            resealed.claim(),
            FederationStateMachineTraceExternalVerificationClaim::ArchiveEvidenceVerified
        );
        assert_ne!(first.claim(), resealed.claim());
        assert_eq!(
            resealed.anchor_reference_sha256(),
            first.anchor_reference_sha256()
        );
    }

    #[test]
    fn external_evidence_verification_statement_binds_anchor_and_verifier_report() {
        let anchor_reference_sha256 =
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let verifier_report = b"opaque-external-verifier-report-v1";
        let statement =
            state_machine_trace_external_evidence_verification_statement(
                anchor_reference_sha256,
                1,
                "rfc3161-verifier-v1",
                FederationStateMachineTraceExternalVerificationClaim::TimestampTokenVerified,
                verifier_report,
                1_791_000_500,
            )
            .expect("verification statement must build");

        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_binding(
                anchor_reference_sha256,
                verifier_report,
                &statement,
            ),
            Ok(())
        );

        let round_trip =
            serde_json::to_string(&statement).expect("verification statement must serialize");
        assert_eq!(
            serde_json::from_str::<
                FederationStateMachineTraceExternalEvidenceVerificationStatement,
            >(&round_trip)
            .expect("verification statement must deserialize"),
            statement
        );
    }

    #[test]
    fn external_evidence_verification_statement_rejects_binding_tampering() {
        let anchor_reference_sha256 =
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let verifier_report = b"opaque-external-verifier-report-v1";
        let mut statement =
            state_machine_trace_external_evidence_verification_statement(
                anchor_reference_sha256,
                1,
                "ct-verifier-v2",
                FederationStateMachineTraceExternalVerificationClaim::TransparencyConsistencyVerified,
                verifier_report,
                1_791_000_501,
            )
            .expect("verification statement must build");

        statement.anchor_reference_schema_version = 99;
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);
        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_binding(
                anchor_reference_sha256,
                verifier_report,
                &statement,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceSchemaVersionMismatch
            )
        );

        statement.anchor_reference_schema_version =
            FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SCHEMA_VERSION;
        statement.anchor_reference_profile = "wrong-anchor-profile-v1".into();
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);
        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_binding(
                anchor_reference_sha256,
                verifier_report,
                &statement,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceProfileMismatch
            )
        );

        statement.anchor_reference_profile =
            FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_PROFILE.into();
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);

        statement.anchor_reference_sha256 =
            "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into();
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);
        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_binding(
                anchor_reference_sha256,
                verifier_report,
                &statement,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceDigestMismatch
            )
        );

        statement.anchor_reference_sha256 = anchor_reference_sha256.into();
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);
        let changed_report = b"opaque-external-verifier-report-v2";
        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_binding(
                anchor_reference_sha256,
                changed_report,
                &statement,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::VerifierReportDigestMismatch
            )
        );

        statement.verifier_report_hash_encoding = "raw-bytes-v1".into();
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);
        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_binding(
                anchor_reference_sha256,
                verifier_report,
                &statement,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::VerifierReportHashEncodingMismatch
            )
        );
    }

    #[test]
    fn external_evidence_verification_statement_does_not_authenticate_external_claims() {
        let anchor_reference_sha256 =
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let verifier_report = b"this-report-may-claim-anything";
        let mut statement =
            state_machine_trace_external_evidence_verification_statement(
                anchor_reference_sha256,
                1,
                "untrusted-verifier-profile-v1",
                FederationStateMachineTraceExternalVerificationClaim::CryptographicSignatureVerified,
                verifier_report,
                1_791_000_502,
            )
            .expect("verification statement must build");

        statement.verification_claim =
            FederationStateMachineTraceExternalVerificationClaim::Rejected;
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);

        assert_eq!(
            validate_state_machine_trace_external_evidence_verification_statement_binding(
                anchor_reference_sha256,
                verifier_report,
                &statement,
            ),
            Ok(())
        );
    }

    #[test]
    fn external_evidence_verification_statement_rejects_invalid_metadata_and_unknown_fields() {
        let anchor_reference_sha256 =
            "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let verifier_report = b"opaque-report";
        assert_eq!(
            state_machine_trace_external_evidence_verification_statement(
                "not-a-digest",
                1,
                "verifier-v1",
                FederationStateMachineTraceExternalVerificationClaim::ArchiveEvidenceVerified,
                verifier_report,
                1,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation::InvalidAnchorReferenceDigest
            )
        );
        assert_eq!(
            state_machine_trace_external_evidence_verification_statement(
                anchor_reference_sha256,
                0,
                "verifier-v1",
                FederationStateMachineTraceExternalVerificationClaim::ArchiveEvidenceVerified,
                verifier_report,
                1,
            ),
            Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation::InvalidVerifierReportSchemaVersion
            )
        );

        let statement =
            state_machine_trace_external_evidence_verification_statement(
                anchor_reference_sha256,
                1,
                "verifier-v1",
                FederationStateMachineTraceExternalVerificationClaim::ArchiveEvidenceVerified,
                verifier_report,
                1,
            )
            .expect("verification statement must build");
        let mut unknown =
            serde_json::to_value(&statement).expect("verification statement must serialize");
        unknown
            .as_object_mut()
            .expect("verification statement must be an object")
            .insert("unexpected_verification_field".into(), serde_json::Value::Bool(true));
        assert!(
            serde_json::from_value::<
                FederationStateMachineTraceExternalEvidenceVerificationStatement,
            >(unknown)
            .is_err()
        );
    }

}

#[cfg(test)]
mod tests {
    use super::*;
    use sha2::{Digest, Sha256};

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
    fn validate_state_covers_node_recognition_and_dependency_identity() {
        let mut state = nodes();
        assert_eq!(validate_state(&state), Ok(()));

        let node = state.nodes.remove("node-a").unwrap();
        state.nodes.insert("wrong-node-key".into(), node);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::NodeMapKeyMismatch)
        );

        let mut state = nodes();
        state.recognition_edges.push(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "unknown-node".into(),
            scope: "scope-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::RecognitionEdgeInvalid)
        );

        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        let mut broken = state.delivery("delivery-1").unwrap().clone();
        broken.contract.predecessor_delivery_id = Some("missing-parent".into());
        state.deliveries.insert("delivery-1".into(), broken);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::DeliveryPredecessorMismatch)
        );

        let mut temporal = nodes();
        assert_eq!(
            deliver(&mut temporal, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        let mut child = envelope();
        child.envelope_id = "env-order-validation".into();
        child.logical_delivery_id = "delivery-order-validation".into();
        child.attempt_id = "attempt-order-validation".into();
        child.predecessor_delivery_id = Some("delivery-1".into());
        assert_eq!(
            deliver(&mut temporal, &child, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        let parent_index = temporal.deliveries.get("delivery-1").unwrap().admission_index();
        temporal
            .deliveries
            .get_mut("delivery-order-validation")
            .unwrap()
            .admission_index = parent_index;
        assert_eq!(
            validate_state(&temporal),
            Err(FederationInvariantViolation::DeliveryAdmissionOrderMismatch)
        );

        let mut cyclic = temporal.clone();
        cyclic
            .deliveries
            .get_mut("delivery-1")
            .unwrap()
            .contract
            .predecessor_delivery_id = Some("delivery-order-validation".into());
        assert_eq!(
            validate_state(&cyclic),
            Err(FederationInvariantViolation::DeliveryAdmissionOrderMismatch)
        );
    }

    #[test]
    fn admission_ordinal_advances_only_for_successful_new_deliveries() {
        let mut state = nodes();
        assert_eq!(state.next_admission_index, 0);

        let first = envelope();
        assert_eq!(
            deliver(&mut state, &first, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert_eq!(state.next_admission_index, 1);
        assert_eq!(
            state.delivery("delivery-1").unwrap().admission_index(),
            0
        );

        let mut retry = first.clone();
        retry.attempt_id = "attempt-admission-retry".into();
        assert_eq!(
            deliver(&mut state, &retry, 50, true).decision(),
            FederationDecision::Duplicate
        );
        assert_eq!(state.next_admission_index, 1);

        let mut expired = envelope();
        expired.envelope_id = "env-admission-expired".into();
        expired.logical_delivery_id = "delivery-admission-expired".into();
        expired.attempt_id = "attempt-admission-expired".into();
        expired.expires_at = Some(50);
        assert_eq!(
            deliver(&mut state, &expired, 50, true).decision(),
            FederationDecision::ExpiredAuthorization
        );
        assert_eq!(state.next_admission_index, 1);

        let mut second = envelope();
        second.envelope_id = "env-admission-second".into();
        second.logical_delivery_id = "delivery-admission-second".into();
        second.attempt_id = "attempt-admission-second".into();
        assert_eq!(
            deliver(&mut state, &second, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert_eq!(state.next_admission_index, 2);
        assert_eq!(
            state.delivery("delivery-admission-second").unwrap().admission_index(),
            1
        );
        assert_eq!(validate_state(&state), Ok(()));
    }

    #[test]
    fn admission_ordinal_exhaustion_is_fail_closed() {
        let mut state = nodes();
        state.next_admission_index = u64::MAX;

        let before = canonical_state_fingerprint(&state);
        let before_audit = audit_state(&state);
        let before_delivery_count = state.delivery_count();
        let before_observation_count = state.observation_count();

        let outcome = deliver(&mut state, &envelope(), 50, true);

        assert_eq!(outcome.decision(), FederationDecision::Rejected);
        assert_eq!(state.next_admission_index, u64::MAX);
        assert_eq!(state.delivery_count(), before_delivery_count);
        assert_eq!(state.observation_count(), before_observation_count);
        assert_eq!(canonical_state_fingerprint(&state), before);
        assert_eq!(audit_state(&state), before_audit);
    }

    #[test]
    fn validate_state_accepts_admitted_state_and_canonical_fingerprint_is_stable() {
        let mut state = nodes();
        assert_eq!(validate_state(&state), Ok(()));
        let before = canonical_state_fingerprint(&state);

        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert_eq!(validate_state(&state), Ok(()));

        let after = canonical_state_fingerprint(&state);
        assert_ne!(before, after);

        let cloned = state.clone();
        assert_eq!(after, canonical_state_fingerprint(&cloned));
    }

    #[test]
    fn validate_state_reports_typed_cross_ledger_corruption() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert_eq!(validate_state(&state), Ok(()));

        let original_delivery = state.deliveries.remove("delivery-1").unwrap();
        state
            .deliveries
            .insert("wrong-delivery-key".into(), original_delivery);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::DeliveryMapKeyMismatch)
        );

        let delivery = state.deliveries.remove("wrong-delivery-key").unwrap();
        state.deliveries.insert("delivery-1".into(), delivery);
        assert_eq!(validate_state(&state), Ok(()));

        let original_observation = state.observations.remove("env-1").unwrap();
        state
            .observations
            .insert("wrong-observation-key".into(), original_observation);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::ObservationMapKeyMismatch)
        );
    }

    #[test]
    fn validate_state_reports_attempt_and_provenance_corruption() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let mut missing_attempt = state.delivery("delivery-1").unwrap().clone();
        missing_attempt.attempts.clear();
        state
            .deliveries
            .insert("delivery-1".into(), missing_attempt);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::DeliveryMissingAttemptHistory)
        );

        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        let mut orphan_binding = state.delivery("delivery-1").unwrap().clone();
        orphan_binding
            .attempt_envelope_ids
            .insert("orphan-attempt".into(), "orphan-envelope".into());
        state
            .deliveries
            .insert("delivery-1".into(), orphan_binding);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::AttemptBindingMismatch)
        );

        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        let mut broken_source = state.delivery("delivery-1").unwrap().clone();
        broken_source.source_observation_id = "missing-source".into();
        state
            .deliveries
            .insert("delivery-1".into(), broken_source);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::SourceObservationMismatch)
        );
    }

    #[test]
    fn audit_state_reports_complete_deterministic_status_vector() {
        let mut state = nodes();

        let node = state.nodes.remove("node-a").unwrap();
        state.nodes.insert("wrong-node-key".into(), node);

        state.recognition_edges.push(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "unknown-node".into(),
            scope: "scope-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });

        let audit = audit_state(&state);
        assert_eq!(audit.len(), FEDERATION_INVARIANT_REGISTRY.len());
        assert_eq!(audit[0].id, FederationInvariantId::NodeMapIdentity);
        assert_eq!(
            audit[0].status,
            FederationInvariantAuditStatus::Violated(
                FederationInvariantViolation::NodeMapKeyMismatch
            )
        );
        assert_eq!(
            audit[1].status,
            FederationInvariantAuditStatus::Violated(
                FederationInvariantViolation::RecognitionEdgeInvalid
            )
        );
        assert_eq!(
            audit[2].status,
            FederationInvariantAuditStatus::Violated(
                FederationInvariantViolation::RecognitionEdgeOrderMismatch
            )
        );
        assert!(audit[3..]
            .iter()
            .all(|entry| entry.status == FederationInvariantAuditStatus::Passed));

        assert_eq!(audit, audit_state(&state));
    }

    #[test]
    fn validate_state_all_reports_multiple_corruptions_in_registry_order() {
        let mut state = nodes();

        let node = state.nodes.remove("node-a").unwrap();
        state.nodes.insert("wrong-node-key".into(), node);

        state.recognition_edges.push(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "unknown-node".into(),
            scope: "scope-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });

        let violations = validate_state_all(&state);
        assert_eq!(
            violations,
            vec![
                (
                    FederationInvariantId::NodeMapIdentity,
                    FederationInvariantViolation::NodeMapKeyMismatch
                ),
                (
                    FederationInvariantId::RecognitionEdgeValidity,
                    FederationInvariantViolation::RecognitionEdgeInvalid
                ),
                (
                    FederationInvariantId::RecognitionEdgeCanonicalOrder,
                    FederationInvariantViolation::RecognitionEdgeOrderMismatch
                ),
            ]
        );
    }

    #[test]
    fn invariant_registry_is_unique_documented_and_covers_valid_state() {
        let mut ids = BTreeSet::new();
        assert_eq!(FEDERATION_INVARIANT_REGISTRY.len(), 12);

        for spec in FEDERATION_INVARIANT_REGISTRY {
            assert!(ids.insert(spec.id), "duplicate invariant id: {:?}", spec.id);
            assert!(!spec.name.is_empty());
            assert!(!spec.description.is_empty());
            assert!(
                spec.dependencies.iter().all(|dependency| *dependency != spec.id),
                "invariant cannot depend on itself: {:?}",
                spec.id
            );
        }

        assert!(invariant_registry_dependencies_are_ordered());

        let state = nodes();
        assert_eq!(validate_state(&state), Ok(()));
        assert!(FEDERATION_INVARIANT_REGISTRY
            .iter()
            .all(|spec| (spec.check)(&state)));
    }

    #[test]
    fn invariant_registry_maps_each_corruption_to_a_single_typed_diagnostic() {
        let mut state = nodes();
        let node = state.nodes.remove("node-a").unwrap();
        state.nodes.insert("wrong-node-key".into(), node);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::NodeMapKeyMismatch)
        );

        let mut state = nodes();
        state.recognition_edges.push(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "scope-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });
        state.recognition_edges.push(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "scope-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::RecognitionEdgeOrderMismatch)
        );

        let mut state = nodes();
        assert_eq!(deliver(&mut state, &envelope(), 50, true).decision(), FederationDecision::AcceptedLocal);
        let mut broken = state.delivery("delivery-1").unwrap().clone();
        broken.attempts.clear();
        state.deliveries.insert("delivery-1".into(), broken);
        assert_eq!(
            validate_state(&state),
            Err(FederationInvariantViolation::DeliveryMissingAttemptHistory)
        );
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum FederationInvariantMutation {
        NodeMapIdentity,
        RecognitionEdgeValidity,
        RecognitionEdgeCanonicalOrder,
        DeliveryMapIdentity,
        DeliveryNodeReferences,
        DeliveryPredecessorReferences,
        DeliveryAdmissionOrder,
        ObservationMapIdentity,
        SourceObservationBijection,
        DeliveryAttemptHistory,
        AttemptEnvelopeBindings,
        DeliverySourceObservationProvenance,
    }

    impl FederationInvariantMutation {
        const ALL: &[Self] = &[
            Self::NodeMapIdentity,
            Self::RecognitionEdgeValidity,
            Self::RecognitionEdgeCanonicalOrder,
            Self::DeliveryMapIdentity,
            Self::DeliveryNodeReferences,
            Self::DeliveryPredecessorReferences,
            Self::DeliveryAdmissionOrder,
            Self::ObservationMapIdentity,
            Self::SourceObservationBijection,
            Self::DeliveryAttemptHistory,
            Self::AttemptEnvelopeBindings,
            Self::DeliverySourceObservationProvenance,
        ];

        fn expected_violations(
            self,
        ) -> &'static [(FederationInvariantId, FederationInvariantViolation)] {
            match self {
                Self::NodeMapIdentity => &[(
                    FederationInvariantId::NodeMapIdentity,
                    FederationInvariantViolation::NodeMapKeyMismatch,
                )],
                Self::RecognitionEdgeValidity => &[(
                    FederationInvariantId::RecognitionEdgeValidity,
                    FederationInvariantViolation::RecognitionEdgeInvalid,
                )],
                Self::RecognitionEdgeCanonicalOrder => &[(
                    FederationInvariantId::RecognitionEdgeCanonicalOrder,
                    FederationInvariantViolation::RecognitionEdgeOrderMismatch,
                )],
                Self::DeliveryMapIdentity => &[(
                    FederationInvariantId::DeliveryMapIdentity,
                    FederationInvariantViolation::DeliveryMapKeyMismatch,
                )],
                Self::DeliveryNodeReferences => &[(
                    FederationInvariantId::DeliveryNodeReferences,
                    FederationInvariantViolation::DeliveryNodeReferenceMismatch,
                )],
                Self::DeliveryPredecessorReferences => &[(
                    FederationInvariantId::DeliveryPredecessorReferences,
                    FederationInvariantViolation::DeliveryPredecessorMismatch,
                )],
                Self::DeliveryAdmissionOrder => &[(
                    FederationInvariantId::DeliveryAdmissionOrder,
                    FederationInvariantViolation::DeliveryAdmissionOrderMismatch,
                )],
                Self::ObservationMapIdentity => &[(
                    FederationInvariantId::ObservationMapIdentity,
                    FederationInvariantViolation::ObservationMapKeyMismatch,
                )],
                Self::SourceObservationBijection => &[(
                    FederationInvariantId::SourceObservationBijection,
                    FederationInvariantViolation::SourceObservationSetMismatch,
                )],
                Self::DeliveryAttemptHistory => &[(
                    FederationInvariantId::DeliveryAttemptHistory,
                    FederationInvariantViolation::DeliveryMissingAttemptHistory,
                )],
                Self::AttemptEnvelopeBindings => &[(
                    FederationInvariantId::AttemptEnvelopeBindings,
                    FederationInvariantViolation::AttemptBindingMismatch,
                )],
                Self::DeliverySourceObservationProvenance => &[(
                    FederationInvariantId::DeliverySourceObservationProvenance,
                    FederationInvariantViolation::SourceObservationMismatch,
                )],
            }
        }

        fn direct_id(self) -> FederationInvariantId {
            match self {
                Self::NodeMapIdentity => FederationInvariantId::NodeMapIdentity,
                Self::RecognitionEdgeValidity => FederationInvariantId::RecognitionEdgeValidity,
                Self::RecognitionEdgeCanonicalOrder => {
                    FederationInvariantId::RecognitionEdgeCanonicalOrder
                }
                Self::DeliveryMapIdentity => FederationInvariantId::DeliveryMapIdentity,
                Self::DeliveryNodeReferences => FederationInvariantId::DeliveryNodeReferences,
                Self::DeliveryPredecessorReferences => {
                    FederationInvariantId::DeliveryPredecessorReferences
                }
                Self::DeliveryAdmissionOrder => FederationInvariantId::DeliveryAdmissionOrder,
                Self::ObservationMapIdentity => FederationInvariantId::ObservationMapIdentity,
                Self::SourceObservationBijection => FederationInvariantId::SourceObservationBijection,
                Self::DeliveryAttemptHistory => FederationInvariantId::DeliveryAttemptHistory,
                Self::AttemptEnvelopeBindings => FederationInvariantId::AttemptEnvelopeBindings,
                Self::DeliverySourceObservationProvenance => {
                    FederationInvariantId::DeliverySourceObservationProvenance
                }
            }
        }

        fn mutate(self, state: &mut FederationState) {
            match self {
                Self::NodeMapIdentity => {
                    let node = state.nodes.remove("node-a").unwrap();
                    state.nodes.insert("wrong-node-key".into(), node);
                }
                Self::RecognitionEdgeValidity => {
                    state.recognition_edges.push(RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "node-b".into(),
                        scope: String::new(),
                        mode: RecognitionMode::EvidenceOnly,
                    });
                }
                Self::RecognitionEdgeCanonicalOrder => {
                    state.recognition_edges.push(RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "node-b".into(),
                        scope: "scope-z".into(),
                        mode: RecognitionMode::EvidenceOnly,
                    });
                    state.recognition_edges.push(RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "node-b".into(),
                        scope: "scope-a".into(),
                        mode: RecognitionMode::EvidenceOnly,
                    });
                }
                Self::DeliveryMapIdentity => {
                    let record = state.deliveries.remove("delivery-1").unwrap();
                    state.deliveries.insert("wrong-delivery-key".into(), record);
                }
                Self::DeliveryNodeReferences => {
                    state
                        .deliveries
                        .values_mut()
                        .next()
                        .unwrap()
                        .contract
                        .target_node = "unknown-target".into();
                }
                Self::DeliveryPredecessorReferences => {
                    state
                        .deliveries
                        .values_mut()
                        .next()
                        .unwrap()
                        .contract
                        .predecessor_delivery_id = Some("missing-predecessor".into());
                }
                Self::DeliveryAdmissionOrder => {
                    let mut delivery_ids = state.deliveries.keys().cloned().collect::<Vec<_>>();
                    delivery_ids.sort();
                    assert!(delivery_ids.len() >= 2);
                    let first = delivery_ids[0].clone();
                    let second = delivery_ids[1].clone();
                    let first_index = state.deliveries.get(&first).unwrap().admission_index();
                    state.deliveries.get_mut(&second).unwrap().admission_index = first_index;
                }
                Self::ObservationMapIdentity => {
                    state.observations.insert(
                        "wrong-observation-key".into(),
                        ObservationRecord {
                            observation_id: "ordinary-observation".into(),
                            semantic_subject_id: "subject-1".into(),
                            payload_commitment: "sha256:ordinary".into(),
                            origin_node: "node-a".into(),
                            origin_node_known: false,
                            recognized_by: None,
                            source_observation: false,
                        },
                    );
                }
                Self::SourceObservationBijection => {
                    state.observations.insert(
                        "orphan-source".into(),
                        ObservationRecord {
                            observation_id: "orphan-source".into(),
                            semantic_subject_id: "subject-1".into(),
                            payload_commitment: "sha256:orphan".into(),
                            origin_node: "node-a".into(),
                            origin_node_known: true,
                            recognized_by: None,
                            source_observation: true,
                        },
                    );
                }
                Self::DeliveryAttemptHistory => {
                    let record = state.deliveries.values_mut().next().unwrap();
                    record.attempts.clear();
                    record.attempt_envelope_ids.clear();
                }
                Self::AttemptEnvelopeBindings => {
                    state
                        .deliveries
                        .values_mut()
                        .next()
                        .unwrap()
                        .attempt_envelope_ids
                        .insert("orphan-attempt".into(), "orphan-envelope".into());
                }
                Self::DeliverySourceObservationProvenance => {
                    let source_id = state
                        .deliveries
                        .values()
                        .next()
                        .unwrap()
                        .source_observation_id()
                        .to_owned();
                    state.observations.get_mut(&source_id).unwrap().payload_commitment =
                        "sha256:tampered".into();
                }
            }
        }

        fn requires_delivery(self) -> bool {
            matches!(
                self,
                Self::DeliveryMapIdentity
                    | Self::DeliveryNodeReferences
                    | Self::DeliveryPredecessorReferences
                    | Self::DeliveryAdmissionOrder
                    | Self::DeliveryAttemptHistory
                    | Self::AttemptEnvelopeBindings
                    | Self::DeliverySourceObservationProvenance
                    | Self::SourceObservationBijection
            )
        }

        fn seed_state(self) -> FederationState {
            let mut state = nodes();
            if self == Self::DeliveryAdmissionOrder {
                assert_eq!(
                    deliver(&mut state, &envelope(), 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
                let mut child = envelope();
                child.envelope_id = "env-order-seed".into();
                child.logical_delivery_id = "delivery-order-seed".into();
                child.attempt_id = "attempt-order-seed".into();
                child.predecessor_delivery_id = Some("delivery-1".into());
                assert_eq!(
                    deliver(&mut state, &child, 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
            } else if self.requires_delivery() {
                assert_eq!(
                    deliver(&mut state, &envelope(), 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
            }
            state
        }

        fn pair_seed(
            first: FederationInvariantMutation,
            second: FederationInvariantMutation,
        ) -> FederationState {
            let mut state = nodes();
            if first == FederationInvariantMutation::DeliveryAdmissionOrder
                || second == FederationInvariantMutation::DeliveryAdmissionOrder
            {
                assert_eq!(
                    deliver(&mut state, &envelope(), 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
                let mut child = envelope();
                child.envelope_id = "env-cycle-seed".into();
                child.logical_delivery_id = "delivery-cycle-seed".into();
                child.attempt_id = "attempt-cycle-seed".into();
                child.predecessor_delivery_id = Some("delivery-1".into());
                assert_eq!(
                    deliver(&mut state, &child, 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
            } else if first.requires_delivery() || second.requires_delivery() {
                assert_eq!(
                    deliver(&mut state, &envelope(), 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
            }
            state
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    enum FederationAdmissionOrderMutation {
        DuplicateOrdinal,
        CounterGap,
        FuturePredecessor,
    }

    impl FederationAdmissionOrderMutation {
        const ALL: &[Self] = &[
            Self::DuplicateOrdinal,
            Self::CounterGap,
            Self::FuturePredecessor,
        ];

        fn seed_state(self) -> FederationState {
            let mut state = nodes();

            for (logical_delivery_id, envelope_id, attempt_id, predecessor) in [
                ("delivery-1", "env-1", "attempt-1", None),
                ("delivery-order-two", "env-order-two", "attempt-order-two", None),
                ("delivery-order-three", "env-order-three", "attempt-order-three", None),
            ] {
                let mut candidate = envelope();
                candidate.logical_delivery_id = logical_delivery_id.into();
                candidate.envelope_id = envelope_id.into();
                candidate.attempt_id = attempt_id.into();
                candidate.predecessor_delivery_id = predecessor.map(str::to_owned);
                assert_eq!(
                    deliver(&mut state, &candidate, 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
            }

            assert_eq!(state.next_admission_index, 3);
            assert_eq!(
                validate_state(&state),
                Ok(()),
                "admission-order mutation seed must satisfy every invariant"
            );
            state
        }

        fn mutate(self, state: &mut FederationState) {
            let mut ids = state.deliveries.keys().cloned().collect::<Vec<_>>();
            ids.sort();
            let first = ids[0].clone();
            let second = ids[1].clone();
            let third = ids[2].clone();

            match self {
                Self::DuplicateOrdinal => {
                    let second_index = state.deliveries.get(&second).unwrap().admission_index();
                    state.deliveries.get_mut(&third).unwrap().admission_index = second_index;
                }
                Self::CounterGap => {
                    state.next_admission_index += 1;
                }
                Self::FuturePredecessor => {
                    state
                        .deliveries
                        .get_mut(&first)
                        .unwrap()
                        .contract
                        .predecessor_delivery_id = Some(third);
                }
            }
        }
    }

    #[test]
    fn admission_order_mutation_corpus_separates_temporal_corruption_modes() {
        let mut fingerprints = BTreeSet::new();

        for mutation in FederationAdmissionOrderMutation::ALL {
            let mut state = mutation.seed_state();
            mutation.mutate(&mut state);

            let expected = vec![(
                FederationInvariantId::DeliveryAdmissionOrder,
                FederationInvariantViolation::DeliveryAdmissionOrderMismatch,
            )];
            assert_eq!(
                validate_state_all(&state),
                expected,
                "temporal mutation {:?} must isolate the admission-order invariant",
                mutation
            );
            assert_eq!(
                audit_state(&state)
                    .into_iter()
                    .filter_map(|entry| match entry.status {
                        FederationInvariantAuditStatus::Passed => None,
                        FederationInvariantAuditStatus::Violated(violation) => Some((entry.id, violation)),
                    })
                    .collect::<Vec<_>>(),
                expected
            );
            assert!(
                fingerprints.insert(canonical_state_fingerprint(&state)),
                "temporal mutation {:?} must produce a distinct canonical fingerprint",
                mutation
            );
        }

        assert_eq!(fingerprints.len(), FederationAdmissionOrderMutation::ALL.len());
    }

    #[test]
    fn admission_order_mutation_pair_matrix_is_deterministic_and_order_aware() {
        let mutations = FederationAdmissionOrderMutation::ALL;
        let mut pair_count = 0;

        for (first_index, first) in mutations.iter().enumerate() {
            for second in mutations.iter().skip(first_index + 1) {
                pair_count += 1;

                let mut forward = first.seed_state();
                first.mutate(&mut forward);
                second.mutate(&mut forward);

                let mut reverse = second.seed_state();
                second.mutate(&mut reverse);
                first.mutate(&mut reverse);

                let expected = vec![(
                    FederationInvariantId::DeliveryAdmissionOrder,
                    FederationInvariantViolation::DeliveryAdmissionOrderMismatch,
                )];
                assert_eq!(validate_state_all(&forward), expected);
                assert_eq!(
                    validate_state_all(&reverse),
                    expected,
                    "temporal mutation pair ({:?}, {:?}) must not depend on mutation order",
                    first,
                    second
                );
                let mut repeated_forward = first.seed_state();
                first.mutate(&mut repeated_forward);
                second.mutate(&mut repeated_forward);
                let mut repeated_reverse = second.seed_state();
                second.mutate(&mut repeated_reverse);
                first.mutate(&mut repeated_reverse);

                assert_eq!(
                    canonical_state_fingerprint(&forward),
                    canonical_state_fingerprint(&repeated_forward),
                    "forward temporal mutation order must be deterministic"
                );
                assert_eq!(
                    canonical_state_fingerprint(&reverse),
                    canonical_state_fingerprint(&repeated_reverse),
                    "reverse temporal mutation order must be deterministic"
                );
            }
        }

        assert_eq!(
            pair_count,
            mutations.len() * (mutations.len() - 1) / 2,
            "temporal mutation corpus must cover every unordered pair"
        );
    }

    #[test]
    fn invariant_mutation_matrix_isolated_failure_surface_is_complete() {
        assert_eq!(
            FederationInvariantMutation::ALL.len(),
            FEDERATION_INVARIANT_REGISTRY.len()
        );

        let registry_ids = FEDERATION_INVARIANT_REGISTRY
            .iter()
            .map(|spec| spec.id)
            .collect::<Vec<_>>();
        let mut fingerprints = BTreeSet::new();

        for mutation in FederationInvariantMutation::ALL {
            let mut state = mutation.seed_state();
            assert_eq!(
                validate_state(&state),
                Ok(()),
                "mutation seed must satisfy every invariant: {:?}",
                mutation
            );

            mutation.mutate(&mut state);

            let expected = mutation.expected_violations();
            let actual = validate_state_all(&state);
            assert_eq!(
                actual, expected,
                "mutation {:?} must have a distinguishable failure surface",
                mutation
            );

            let first_failure = expected.first().map(|(_, violation)| *violation);
            assert_eq!(
                validate_state(&state).err(),
                first_failure,
                "legacy first-failure gate must agree with the complete audit for {:?}",
                mutation
            );

            assert!(
                fingerprints.insert(canonical_state_fingerprint(&state)),
                "single-fault mutation {:?} must produce a unique canonical state fingerprint",
                mutation
            );

            let audit = audit_state(&state);
            assert_eq!(audit.len(), registry_ids.len());
            let audited_violations = audit
                .iter()
                .filter_map(|entry| match entry.status {
                    FederationInvariantAuditStatus::Passed => None,
                    FederationInvariantAuditStatus::Violated(violation) => {
                        Some((entry.id, violation))
                    }
                })
                .collect::<Vec<_>>();
            assert_eq!(audited_violations, expected);

            let expected_ids = expected.iter().map(|(id, _)| *id).collect::<BTreeSet<_>>();
            let actual_ids = actual.iter().map(|(id, _)| *id).collect::<BTreeSet<_>>();
            assert_eq!(actual_ids, expected_ids);
        }

        assert_eq!(
            fingerprints.len(),
            FederationInvariantMutation::ALL.len(),
            "single-fault fingerprint corpus must be unique and complete"
        );
    }

    #[test]
    fn invariant_mutation_matrix_covers_each_registered_invariant_exactly_once() {
        let exercised = FederationInvariantMutation::ALL
            .iter()
            .flat_map(|mutation| mutation.expected_violations().iter().map(|(id, _)| *id))
            .collect::<Vec<_>>();
        let registry_ids = FEDERATION_INVARIANT_REGISTRY
            .iter()
            .map(|spec| spec.id)
            .collect::<Vec<_>>();

        assert_eq!(exercised.len(), registry_ids.len());
        assert_eq!(exercised, registry_ids);
    }

    #[test]
    fn invariant_mutation_pair_matrix_is_deterministic_and_order_aware() {
        let mutations = FederationInvariantMutation::ALL;
        let mut pair_count = 0;
        let mut order_sensitive_pairs = 0;

        for (first_index, first) in mutations.iter().enumerate() {
            for second in mutations.iter().skip(first_index + 1) {
                pair_count += 1;
                // Build both semantic compositions from the same clean seed.
                let mut forward = FederationInvariantMutation::pair_seed(*first, *second);
                first.mutate(&mut forward);
                second.mutate(&mut forward);
                let forward_audit = audit_state(&forward);

                let mut reverse = FederationInvariantMutation::pair_seed(*first, *second);
                second.mutate(&mut reverse);
                first.mutate(&mut reverse);
                let reverse_audit = audit_state(&reverse);

                // Re-running the same composition must produce byte-for-byte
                // equivalent structured diagnostics.
                let mut repeated = FederationInvariantMutation::pair_seed(*first, *second);
                first.mutate(&mut repeated);
                second.mutate(&mut repeated);
                assert_eq!(
                    forward_audit,
                    audit_state(&repeated),
                    "pair ({:?}, {:?}) is not deterministic",
                    first,
                    second
                );

                let forward_violations = forward_audit
                    .iter()
                    .filter_map(|entry| match entry.status {
                        FederationInvariantAuditStatus::Passed => None,
                        FederationInvariantAuditStatus::Violated(violation) => {
                            Some((entry.id, violation))
                        }
                    })
                    .collect::<Vec<_>>();
                let reverse_violations = reverse_audit
                    .iter()
                    .filter_map(|entry| match entry.status {
                        FederationInvariantAuditStatus::Passed => None,
                        FederationInvariantAuditStatus::Violated(violation) => {
                            Some((entry.id, violation))
                        }
                    })
                    .collect::<Vec<_>>();

                let forward_ids = forward_violations
                    .iter()
                    .map(|(id, _)| *id)
                    .collect::<BTreeSet<_>>();
                let reverse_ids = reverse_violations
                    .iter()
                    .map(|(id, _)| *id)
                    .collect::<BTreeSet<_>>();

                assert!(
                    forward_ids.contains(&first.direct_id())
                        || reverse_ids.contains(&first.direct_id()),
                    "pair ({:?}, {:?}) masked first mutation in both orders",
                    first,
                    second
                );
                assert!(
                    forward_ids.contains(&second.direct_id())
                        || reverse_ids.contains(&second.direct_id()),
                    "pair ({:?}, {:?}) masked second mutation in both orders",
                    first,
                    second
                );

                if *first == FederationInvariantMutation::DeliveryAttemptHistory
                    && *second == FederationInvariantMutation::AttemptEnvelopeBindings
                {
                    order_sensitive_pairs += 1;
                    assert_eq!(
                        forward_violations,
                        vec![
                            (
                                FederationInvariantId::DeliveryAttemptHistory,
                                FederationInvariantViolation::DeliveryMissingAttemptHistory,
                            ),
                            (
                                FederationInvariantId::AttemptEnvelopeBindings,
                                FederationInvariantViolation::AttemptBindingMismatch,
                            ),
                        ]
                    );
                    assert_eq!(
                        reverse_violations,
                        vec![(
                            FederationInvariantId::DeliveryAttemptHistory,
                            FederationInvariantViolation::DeliveryMissingAttemptHistory,
                        )]
                    );
                } else {
                    assert_eq!(
                        forward_violations,
                        reverse_violations,
                        "pair ({:?}, {:?}) unexpectedly depends on mutation order",
                        first,
                        second
                    );
                }
            }
        }

        assert_eq!(
            pair_count,
            mutations.len() * (mutations.len() - 1) / 2,
            "pairwise corpus must cover every unordered mutation pair"
        );
        assert_eq!(
            order_sensitive_pairs, 1,
            "only attempt-history deletion plus binding injection is order-sensitive"
        );
    }

    fn assert_public_transition_preserves_invariants<F>(
        state: &mut FederationState,
        transition: &str,
        expect_state_change: bool,
        transition_fn: F,
    ) where
        F: FnOnce(&mut FederationState),
    {
        assert_eq!(
            state.next_admission_index,
            state.delivery_count() as u64,
            "transition fixture must begin from a valid admission-ordinal state: {transition}"
        );
        assert!(validate_state(state).is_ok(), "transition precondition invalid: {transition}");

        let before = canonical_state_fingerprint(state);
        let before_delivery_ids = state.deliveries.keys().cloned().collect::<BTreeSet<_>>();
        let before_delivery_count = state.delivery_count();
        let before_admission_index = state.next_admission_index;

        transition_fn(state);

        assert!(
            validate_state(state).is_ok(),
            "public transition produced invalid state: {transition}"
        );
        assert_eq!(
            state.next_admission_index,
            state.delivery_count() as u64,
            "valid transition state must keep admission counter equal to admitted delivery count: {transition}"
        );
        assert!(
            state.delivery_count() >= before_delivery_count,
            "public transition unexpectedly deleted an admitted delivery: {transition}"
        );

        if state.delivery_count() == before_delivery_count {
            assert_eq!(
                state.next_admission_index,
                before_admission_index,
                "a transition without a new delivery must not consume an admission ordinal: {transition}"
            );
        } else {
            assert_eq!(
                state.delivery_count(),
                before_delivery_count + 1,
                "each transition can admit at most one new logical delivery: {transition}"
            );
            assert_eq!(
                state.next_admission_index,
                before_admission_index + 1,
                "a new logical delivery must consume exactly one admission ordinal: {transition}"
            );

            let new_ids = state
                .deliveries
                .keys()
                .filter(|id| !before_delivery_ids.contains(*id))
                .collect::<Vec<_>>();
            assert_eq!(
                new_ids.len(),
                1,
                "exactly one new delivery identity must appear: {transition}"
            );
            assert_eq!(
                state.delivery(new_ids[0]).unwrap().admission_index(),
                before_admission_index,
                "the new delivery must receive the next available admission ordinal: {transition}"
            );
        }

        let after = canonical_state_fingerprint(state);
        assert_eq!(
            after != before,
            expect_state_change,
            "unexpected state-change classification for {transition}"
        );
    }

    #[test]
    fn public_state_transitions_preserve_all_registered_invariants() {
        let mut wrapper_state = nodes();
        assert_public_transition_preserves_invariants(
            &mut wrapper_state,
            "add_recognition/insert",
            true,
            |state| {
                state.add_recognition(RecognitionEdge {
                    recognizing_node: "node-a".into(),
                    origin_node: "node-b".into(),
                    scope: "subject-wrapper".into(),
                    mode: RecognitionMode::EvidenceOnly,
                });
            },
        );

        let mut state = nodes();
        assert_eq!(validate_state(&state), Ok(()));

        assert_public_transition_preserves_invariants(
            &mut state,
            "try_add_recognition/insert",
            true,
            |state| {
                assert_eq!(
                    state.try_add_recognition(RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "node-b".into(),
                        scope: "subject-1".into(),
                        mode: RecognitionMode::EvidenceOnly,
                    }),
                    Ok(true)
                );
            },
        );

        assert_public_transition_preserves_invariants(
            &mut state,
            "try_add_recognition/duplicate",
            false,
            |state| {
                assert_eq!(
                    state.try_add_recognition(RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "node-b".into(),
                        scope: "subject-1".into(),
                        mode: RecognitionMode::EvidenceOnly,
                    }),
                    Ok(false)
                );
            },
        );

        assert_public_transition_preserves_invariants(
            &mut state,
            "try_add_recognition/unknown-node",
            false,
            |state| {
                assert_eq!(
                    state.try_add_recognition(RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "unknown-node".into(),
                        scope: "subject-1".into(),
                        mode: RecognitionMode::EvidenceOnly,
                    }),
                    Err(FederationRecognitionError::UnknownOriginNode)
                );
            },
        );

        let independent = ObservationRecord {
            observation_id: "obs-transition".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:transition".into(),
            origin_node: "node-a".into(),
            origin_node_known: false,
            recognized_by: None,
            source_observation: false,
        };

        assert_public_transition_preserves_invariants(
            &mut state,
            "record_observation/insert",
            true,
            |state| {
                assert_eq!(
                    record_observation(state, independent.clone()),
                    ObservationWriteResult::Inserted
                );
            },
        );

        assert_public_transition_preserves_invariants(
            &mut state,
            "record_observation/duplicate",
            false,
            |state| {
                assert_eq!(
                    record_observation(state, independent.clone()),
                    ObservationWriteResult::Duplicate
                );
            },
        );

        let mut conflicting = independent.clone();
        conflicting.payload_commitment = "sha256:transition-conflict".into();
        assert_public_transition_preserves_invariants(
            &mut state,
            "record_observation/conflict",
            false,
            |state| {
                assert_eq!(
                    record_observation(state, conflicting),
                    ObservationWriteResult::Conflict
                );
            },
        );

        let candidate = envelope();
        assert_public_transition_preserves_invariants(
            &mut state,
            "deliver/accepted-local",
            true,
            |state| {
                assert_eq!(
                    deliver(state, &candidate, 50, true).decision(),
                    FederationDecision::AcceptedLocal
                );
            },
        );

        let mut retry = candidate.clone();
        retry.attempt_id = "attempt-2".into();
        assert_public_transition_preserves_invariants(
            &mut state,
            "deliver/duplicate-new-attempt",
            true,
            |state| {
                assert_eq!(
                    deliver(state, &retry, 50, true).decision(),
                    FederationDecision::Duplicate
                );
            },
        );

        let before_rejected = canonical_state_fingerprint(&state);
        let mut attempt_rebind = retry.clone();
        attempt_rebind.envelope_id = "env-rebound".into();
        let outcome = deliver(&mut state, &attempt_rebind, 50, true);
        assert_eq!(outcome.decision(), FederationDecision::AttemptConflict);
        assert_eq!(canonical_state_fingerprint(&state), before_rejected);
        assert_eq!(validate_state(&state), Ok(()));

        let before_partition = canonical_state_fingerprint(&state);
        let mut foreign = envelope();
        foreign.envelope_id = "env-transition-foreign".into();
        foreign.logical_delivery_id = "delivery-transition-foreign".into();
        foreign.attempt_id = "attempt-transition-foreign".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();
        let partitioned = deliver(&mut state, &foreign, 50, false);
        assert_eq!(partitioned.decision(), FederationDecision::PartitionUnknown);
        assert_eq!(canonical_state_fingerprint(&state), before_partition);
        assert_eq!(validate_state(&state), Ok(()));

        let before_stale = canonical_state_fingerprint(&state);
        let mut stale = envelope();
        stale.envelope_id = "env-transition-stale".into();
        stale.logical_delivery_id = "delivery-transition-stale".into();
        stale.attempt_id = "attempt-transition-stale".into();
        stale.schema_generation = 2;
        assert_eq!(
            deliver(&mut state, &stale, 50, true).decision(),
            FederationDecision::StaleGeneration
        );
        assert_eq!(canonical_state_fingerprint(&state), before_stale);
        assert_eq!(validate_state(&state), Ok(()));

        let before_expiry = canonical_state_fingerprint(&state);
        let mut expired = envelope();
        expired.envelope_id = "env-transition-expired".into();
        expired.logical_delivery_id = "delivery-transition-expired".into();
        expired.attempt_id = "attempt-transition-expired".into();
        expired.expires_at = Some(50);
        assert_eq!(
            deliver(&mut state, &expired, 50, true).decision(),
            FederationDecision::ExpiredAuthorization
        );
        assert_eq!(canonical_state_fingerprint(&state), before_expiry);
        assert_eq!(validate_state(&state), Ok(()));

        let before_revoked = canonical_state_fingerprint(&state);
        let mut revoked = envelope();
        revoked.envelope_id = "env-transition-revoked".into();
        revoked.logical_delivery_id = "delivery-transition-revoked".into();
        revoked.attempt_id = "attempt-transition-revoked".into();
        revoked.authorization = AuthorizationState::Revoked;
        assert_eq!(
            deliver(&mut state, &revoked, 50, true).decision(),
            FederationDecision::Unauthorized
        );
        assert_eq!(canonical_state_fingerprint(&state), before_revoked);
        assert_eq!(validate_state(&state), Ok(()));

        let before_absent = canonical_state_fingerprint(&state);
        let mut absent = envelope();
        absent.envelope_id = "env-transition-absent".into();
        absent.logical_delivery_id = "delivery-transition-absent".into();
        absent.attempt_id = "attempt-transition-absent".into();
        absent.authorization = AuthorizationState::Absent;
        assert_eq!(
            deliver(&mut state, &absent, 50, true).decision(),
            FederationDecision::Unauthorized
        );
        assert_eq!(canonical_state_fingerprint(&state), before_absent);
        assert_eq!(validate_state(&state), Ok(()));

        let before_rejected_observation = canonical_state_fingerprint(&state);
        let mut forged_source = independent;
        forged_source.observation_id = "obs-forged-source".into();
        forged_source.source_observation = true;
        assert_eq!(
            record_observation(&mut state, forged_source),
            ObservationWriteResult::RejectedSourceClaim
        );
        assert_eq!(
            canonical_state_fingerprint(&state),
            before_rejected_observation
        );
        assert_eq!(validate_state(&state), Ok(()));
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
    enum FederationStateMachineOperation {
        AddRecognition,
        DuplicateRecognition,
        RecordObservation,
        DuplicateObservation,
        ConflictObservation,
        AdmitLocal,
        ReplayExact,
        RetryExisting,
        RebindAttempt,
        AdmitForeign,
        AdmitRecognizedForeign,
        PendingDependency,
        AdmitWithPredecessor,
        Partition,
        StaleSchema,
        Expired,
        Revoked,
        Absent,
        ForgedSourceObservation,
        ConflictExistingDelivery,
        InjectDeliveryMapCorruption,
    }

    impl FederationStateMachineOperation {
        const ALL: &[Self] = &[
            Self::AddRecognition,
            Self::DuplicateRecognition,
            Self::RecordObservation,
            Self::DuplicateObservation,
            Self::ConflictObservation,
            Self::AdmitLocal,
            Self::ReplayExact,
            Self::RetryExisting,
            Self::RebindAttempt,
            Self::AdmitForeign,
            Self::AdmitRecognizedForeign,
            Self::PendingDependency,
            Self::AdmitWithPredecessor,
            Self::Partition,
            Self::StaleSchema,
            Self::Expired,
            Self::Revoked,
            Self::Absent,
            Self::ForgedSourceObservation,
            Self::ConflictExistingDelivery,
        ];
    }

    const FEDERATION_STATE_MACHINE_FAILURE_CAPSULE_SCHEMA_VERSION: u16 = 2;
    const FEDERATION_STATE_MACHINE_FAILURE_CAPSULE_KIND: &str = "invariant-violation";

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum FederationStateMachineFailureCapsuleViolation {
        UnsupportedSchemaVersion,
        UnsupportedFailureKind,
        FailedStepIndexMismatch,
        FailedStepIdentityMismatch,
        AuditRegistryMismatch,
        ObservedViolationAuditMismatch,
        InvalidStateValidityFlags,
        TemporalEvidenceMismatch,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineFailureCapsule {
        schema_version: u16,
        failure_kind: String,
        trace_index: usize,
        trace_prefix: Vec<(FederationStateMachineOperation, u64)>,
        failed_step_index: usize,
        operation: FederationStateMachineOperation,
        token: u64,
        expected_state_valid: bool,
        observed_state_valid: bool,
        observed_violations:
            Vec<(FederationInvariantId, FederationInvariantViolation)>,
        audit: Vec<FederationInvariantAuditEntry>,
        expected_decision: Option<FederationDecision>,
        observed_decision: Option<FederationDecision>,
        expected_authority: Option<AuthorityDisposition>,
        observed_authority: Option<AuthorityDisposition>,
        pre_state_fingerprint: Vec<u8>,
        post_state_fingerprint: Vec<u8>,
        pre_admission_index: u64,
        post_admission_index: u64,
        pre_delivery_count: usize,
        post_delivery_count: usize,
        newly_admitted_deliveries: Vec<(String, u64)>,
    }

    impl FederationStateMachineFailureCapsule {
        fn for_invariant_failure(
            trace_index: usize,
            plan: &[(FederationStateMachineOperation, u64)],
            failed_step_index: usize,
            operation: FederationStateMachineOperation,
            token: u64,
            audit: Vec<FederationInvariantAuditEntry>,
            pre_state_fingerprint: Vec<u8>,
            post_state_fingerprint: Vec<u8>,
            pre_admission_index: u64,
            post_admission_index: u64,
            pre_delivery_count: usize,
            post_delivery_count: usize,
            newly_admitted_deliveries: Vec<(String, u64)>,
        ) -> Self {
            let trace_prefix = plan[..=failed_step_index].to_vec();
            let observed_violations = audit
                .iter()
                .filter_map(|entry| match entry.status {
                    FederationInvariantAuditStatus::Passed => None,
                    FederationInvariantAuditStatus::Violated(violation) => {
                        Some((entry.id, violation))
                    }
                })
                .collect();

            Self {
                schema_version: FEDERATION_STATE_MACHINE_FAILURE_CAPSULE_SCHEMA_VERSION,
                failure_kind: FEDERATION_STATE_MACHINE_FAILURE_CAPSULE_KIND.into(),
                trace_index,
                trace_prefix,
                failed_step_index,
                operation,
                token,
                expected_state_valid: true,
                observed_state_valid: false,
                observed_violations,
                audit,
                expected_decision: None,
                observed_decision: None,
                expected_authority: None,
                observed_authority: None,
                pre_state_fingerprint,
                post_state_fingerprint,
                pre_admission_index,
                post_admission_index,
                pre_delivery_count,
                post_delivery_count,
                newly_admitted_deliveries,
            }
        }

        fn validate(&self) -> Result<(), FederationStateMachineFailureCapsuleViolation> {
            if self.schema_version != FEDERATION_STATE_MACHINE_FAILURE_CAPSULE_SCHEMA_VERSION {
                return Err(
                    FederationStateMachineFailureCapsuleViolation::UnsupportedSchemaVersion
                );
            }
            if self.failure_kind != FEDERATION_STATE_MACHINE_FAILURE_CAPSULE_KIND {
                return Err(FederationStateMachineFailureCapsuleViolation::UnsupportedFailureKind);
            }
            if !self.expected_state_valid || self.observed_state_valid {
                return Err(FederationStateMachineFailureCapsuleViolation::StateValidityFlagsMismatch);
            }
            if self.trace_prefix.len() != self.failed_step_index.saturating_add(1) {
                return Err(
                    FederationStateMachineFailureCapsuleViolation::FailedStepIndexMismatch
                );
            }
            if self.trace_prefix.get(self.failed_step_index)
                != Some(&(self.operation, self.token))
            {
                return Err(
                    FederationStateMachineFailureCapsuleViolation::FailedStepIdentityMismatch
                );
            }
            if !self.expected_state_valid || self.observed_state_valid {
                return Err(
                    FederationStateMachineFailureCapsuleViolation::InvalidStateValidityFlags
                );
            }
            let expected_audit_ids = FEDERATION_INVARIANT_REGISTRY
                .iter()
                .map(|spec| spec.id)
                .collect::<Vec<_>>();
            let observed_audit_ids = self
                .audit
                .iter()
                .map(|entry| entry.id)
                .collect::<Vec<_>>();
            if observed_audit_ids != expected_audit_ids {
                return Err(FederationStateMachineFailureCapsuleViolation::AuditRegistryMismatch);
            }
            let delivery_delta = self.post_delivery_count.checked_sub(self.pre_delivery_count);
            let admission_delta = self.post_admission_index.checked_sub(self.pre_admission_index);
            let admitted_count = self.newly_admitted_deliveries.len() as u64;
            if delivery_delta != Some(self.newly_admitted_deliveries.len())
                || admission_delta != Some(admitted_count)
            {
                return Err(FederationStateMachineFailureCapsuleViolation::TemporalEvidenceMismatch);
            }
            for (offset, (delivery_id, ordinal)) in
                self.newly_admitted_deliveries.iter().enumerate()
            {
                if delivery_id.is_empty()
                    || *ordinal
                        != self
                            .pre_admission_index
                            .checked_add(offset as u64)
                            .ok_or(FederationStateMachineFailureCapsuleViolation::TemporalEvidenceMismatch)?
                {
                    return Err(FederationStateMachineFailureCapsuleViolation::TemporalEvidenceMismatch);
                }
            }
            if self.audit.len() != FEDERATION_INVARIANT_REGISTRY.len()
                || self
                    .audit
                    .iter()
                    .zip(FEDERATION_INVARIANT_REGISTRY.iter())
                    .any(|(entry, spec)| entry.id != spec.id)
            {
                return Err(FederationStateMachineFailureCapsuleViolation::AuditRegistryShapeMismatch);
            }
            let derived_violations = self
                .audit
                .iter()
                .filter_map(|entry| match entry.status {
                    FederationInvariantAuditStatus::Passed => None,
                    FederationInvariantAuditStatus::Violated(violation) => {
                        Some((entry.id, violation))
                    }
                })
                .collect::<Vec<_>>();
            if derived_violations != self.observed_violations {
                return Err(
                    FederationStateMachineFailureCapsuleViolation::ObservedViolationAuditMismatch
                );
            }
            Ok(())
        }

        fn to_json(&self) -> String {
            self.validate().expect("failure capsule must self-validate");
            serde_json::to_string_pretty(self)
                .expect("failure capsule is serializable")
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceBoundary {
        admission_index: u64,
        delivery_count: usize,
        state_fingerprint: Vec<u8>,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineEvidence {
        step_index: usize,
        operation: FederationStateMachineOperation,
        token: u64,
        decision: Option<FederationDecision>,
        authority: Option<AuthorityDisposition>,
        pre_state_fingerprint: Vec<u8>,
        state_fingerprint: Vec<u8>,
        chain_prev_sha256: String,
        chain_sha256: String,
        pre_admission_index: u64,
        post_admission_index: u64,
        pre_delivery_count: usize,
        post_delivery_count: usize,
        newly_admitted_deliveries: Vec<(String, u64)>,
    }

    const FEDERATION_STATE_MACHINE_TRACE_CAPSULE_SCHEMA_VERSION: u16 = 5;
    const FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN: &str =
        "integral-federation-trace-body-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN: &str =
        "integral-federation-trace-evidence-chain-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_VERIFICATION_PROFILE: &str =
        "integral-federation-state-machine-trace-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ALGORITHM: &str = "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS: &str =
        "sha256:0000000000000000000000000000000000000000000000000000000000000000";

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceIntegrity {
        algorithm: String,
        encoding: String,
        body_sha256: String,
        chain_head_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineEvidenceHashView {
        hash_domain: String,
        step_index: usize,
        operation: FederationStateMachineOperation,
        token: u64,
        decision: Option<FederationDecision>,
        authority: Option<AuthorityDisposition>,
        pre_state_fingerprint: Vec<u8>,
        state_fingerprint: Vec<u8>,
        chain_prev_sha256: String,
        pre_admission_index: u64,
        post_admission_index: u64,
        pre_delivery_count: usize,
        post_delivery_count: usize,
        newly_admitted_deliveries: Vec<(String, u64)>,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTraceHashView {
        hash_domain: String,
        schema_version: u16,
        verification_profile: String,
        trace_index: usize,
        initial_seed: u64,
        operations: Vec<FederationStateMachineOperation>,
        tokens: Vec<u64>,
        initial_state: FederationStateMachineTraceBoundary,
        evidence: Vec<FederationStateMachineEvidence>,
        final_state: FederationStateMachineTraceBoundary,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceCapsule {
        schema_version: u16,
        verification_profile: String,
        trace_index: usize,
        initial_seed: u64,
        operations: Vec<FederationStateMachineOperation>,
        tokens: Vec<u64>,
        initial_state: FederationStateMachineTraceBoundary,
        evidence: Vec<FederationStateMachineEvidence>,
        final_state: FederationStateMachineTraceBoundary,
        integrity: FederationStateMachineTraceIntegrity,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceCheckpoint {
        schema_version: u16,
        checkpoint_profile: String,
        verification_profile: String,
        trace_index: usize,
        step_start: usize,
        step_end: usize,
        body_sha256: String,
        chain_start_prev_sha256: String,
        chain_head_sha256: String,
        previous_checkpoint_sha256: String,
        checkpoint_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTraceCheckpointHashView {
        hash_domain: String,
        schema_version: u16,
        checkpoint_profile: String,
        verification_profile: String,
        trace_index: usize,
        step_start: usize,
        step_end: usize,
        body_sha256: String,
        chain_start_prev_sha256: String,
        chain_head_sha256: String,
        previous_checkpoint_sha256: String,
    }

    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PROFILE: &str =
        "integral-federation-trace-checkpoint-receipt-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_HASH_DOMAIN: &str =
        "integral-federation-trace-checkpoint-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS: &str =
        "sha256:checkpoint-genesis-v1:0000000000000000000000000000000000000000000000000000000000000000";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_PROFILE: &str =
        "integral-federation-trace-checkpoint-publication-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_DOMAIN: &str =
        "integral-federation-trace-checkpoint-publication-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_ALGORITHM: &str = "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS: &str =
        "sha256:checkpoint-publication-genesis-v1:0000000000000000000000000000000000000000000000000000000000000000";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_PROFILE: &str =
        "integral-federation-trace-checkpoint-consistency-receipt-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_PROOF_TYPE: &str =
        "append-only-prefix-consistency-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_DOMAIN: &str =
        "integral-federation-trace-checkpoint-consistency-receipt-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";

    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_PROFILE: &str =
        "integral-federation-trace-publication-equivocation-witness-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_DOMAIN: &str =
        "integral-federation-trace-publication-equivocation-witness-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";

    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_PROFILE: &str =
        "integral-federation-trace-publication-equivocation-witness-set-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_DOMAIN: &str =
        "integral-federation-trace-publication-equivocation-witness-set-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";

    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE: &str =
        "integral-federation-trace-publication-collection-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_DOMAIN: &str =
        "integral-federation-trace-publication-collection-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";

    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE: &str =
        "integral-federation-trace-publication-collection-reconciliation-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_DOMAIN: &str =
        "integral-federation-trace-publication-collection-reconciliation-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";

    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_PROFILE: &str =
        "integral-federation-trace-external-evidence-anchor-reference-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_DOMAIN: &str =
        "integral-federation-trace-external-evidence-anchor-reference-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SUBJECT_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SUBJECT_HASH_ENCODING: &str =
        "sha256-lowercase-hex-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_WITNESS_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_WITNESS_HASH_ENCODING: &str =
        "sha256-lowercase-hex-v1";

    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_SCHEMA_VERSION: u16 = 1;
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_PROFILE: &str =
        "integral-federation-trace-external-evidence-verification-statement-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_DOMAIN: &str =
        "integral-federation-trace-external-evidence-verification-statement-sha256-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_ENCODING: &str =
        "serde-json-struct-order-v1";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_REPORT_HASH_ALGORITHM: &str =
        "sha-256";
    const FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_REPORT_HASH_ENCODING: &str =
        "sha256-lowercase-hex-v1";

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTracePublicationCollectionRelationship {
        ExactMatch,
        LeftStrictSubset,
        RightStrictSubset,
        OverlappingDivergence,
        Disjoint,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTracePublicationCollectionHistoryRelationship {
        ExactMatch,
        LeftStrictPrefix,
        RightStrictPrefix,
        DivergentAfterCommonPrefix,
        Disjoint,
        NotComparable,
    }

    /// Descriptive kind of an externally supplied witness artifact.
    ///
    /// The reference model does not interpret or authenticate these artifact
    /// formats; it only binds the supplied bytes and metadata into an explicit
    /// evidence-of-evidence reference.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceExternalWitnessKind {
        TimestampToken,
        TransparencyLogHead,
        ArchiveEvidenceRecord,
        Other,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceExternalEvidenceAnchorReference {
        schema_version: u16,
        reference_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        subject_hash_algorithm: String,
        subject_hash_encoding: String,
        subject_schema_version: u16,
        subject_profile: String,
        subject_sha256: String,
        witness_hash_algorithm: String,
        witness_hash_encoding: String,
        witness_schema_version: u16,
        witness_kind: FederationStateMachineTraceExternalWitnessKind,
        witness_profile: String,
        witness_sha256: String,
        claimed_observed_at_unix_seconds: u64,
        anchor_reference_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTraceExternalEvidenceAnchorReferenceHashView {
        hash_domain: String,
        schema_version: u16,
        reference_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        subject_hash_algorithm: String,
        subject_hash_encoding: String,
        subject_schema_version: u16,
        subject_profile: String,
        subject_sha256: String,
        witness_hash_algorithm: String,
        witness_hash_encoding: String,
        witness_schema_version: u16,
        witness_kind: FederationStateMachineTraceExternalWitnessKind,
        witness_profile: String,
        witness_sha256: String,
        claimed_observed_at_unix_seconds: u64,
    }

    /// An externally supplied verifier's claim about an anchor reference.
    ///
    /// This is a statement record, not a proof result. The reference model can
    /// verify the exact binding of the statement to the anchor reference and
    /// verifier-report bytes, but it does not verify the external claim itself.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceExternalVerificationClaim {
        CryptographicSignatureVerified,
        TimestampTokenVerified,
        TransparencyConsistencyVerified,
        ArchiveEvidenceVerified,
        Rejected,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceExternalEvidenceVerificationStatement {
        schema_version: u16,
        statement_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        anchor_reference_schema_version: u16,
        anchor_reference_profile: String,
        anchor_reference_sha256: String,
        verifier_schema_version: u16,
        verifier_profile: String,
        verification_claim: FederationStateMachineTraceExternalVerificationClaim,
        verifier_report_hash_algorithm: String,
        verifier_report_hash_encoding: String,
        verifier_report_sha256: String,
        claimed_verified_at_unix_seconds: u64,
        statement_sha256: String,
    }

    /// Typed read-only projection produced only after the external-evidence binding chain validates.
    ///
    /// This is an externally asserted verification result, not a locally verified proof or authority.
    /// It intentionally omits `Deserialize`, private-key/trust-root fields, and authority fields.
    ///
    /// It is constructed only by the validated statement/anchor chain.
    ///
    /// ```compile_fail
    /// use serde_json::from_str;
    /// # use cos_conformance::federation::FederationStateMachineTraceExternalEvidenceVerificationResult;
    /// let _: FederationStateMachineTraceExternalEvidenceVerificationResult = from_str("{}").unwrap();
    /// ```
    ///
    /// ```compile_fail
    /// # use cos_conformance::federation::{
    /// #     FederationStateMachineTraceExternalEvidenceVerificationResult,
    /// #     FederationStateMachineTraceExternalVerificationClaim,
    /// # };
    /// let _ = FederationStateMachineTraceExternalEvidenceVerificationResult {
    ///     anchor_reference_sha256: "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
    ///     anchor_reference_schema_version: 1,
    ///     anchor_reference_profile: "anchor-v1".into(),
    ///     verifier_schema_version: 1,
    ///     verifier_profile: "verifier-v1".into(),
    ///     claim: FederationStateMachineTraceExternalVerificationClaim::TimestampTokenVerified,
    ///     verifier_report_sha256: "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
    ///     claimed_verified_at_unix_seconds: 1,
    ///     statement_sha256: "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
    /// };
    /// ```
    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    pub struct FederationStateMachineTraceExternalEvidenceVerificationResult {
        anchor_reference_sha256: String,
        anchor_reference_schema_version: u16,
        anchor_reference_profile: String,
        verifier_schema_version: u16,
        verifier_profile: String,
        claim: FederationStateMachineTraceExternalVerificationClaim,
        verifier_report_sha256: String,
        claimed_verified_at_unix_seconds: u64,
        statement_sha256: String,
    }

    impl FederationStateMachineTraceExternalEvidenceVerificationResult {
        pub fn anchor_reference_sha256(&self) -> &str {
            &self.anchor_reference_sha256
        }

        pub fn anchor_reference_schema_version(&self) -> u16 {
            self.anchor_reference_schema_version
        }

        pub fn anchor_reference_profile(&self) -> &str {
            &self.anchor_reference_profile
        }

        pub fn verifier_schema_version(&self) -> u16 {
            self.verifier_schema_version
        }

        pub fn verifier_profile(&self) -> &str {
            &self.verifier_profile
        }

        pub fn claim(&self) -> FederationStateMachineTraceExternalVerificationClaim {
            self.claim
        }

        pub fn verifier_report_sha256(&self) -> &str {
            &self.verifier_report_sha256
        }

        pub fn claimed_verified_at_unix_seconds(&self) -> u64 {
            self.claimed_verified_at_unix_seconds
        }

        pub fn statement_sha256(&self) -> &str {
            &self.statement_sha256
        }
    }
    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTraceExternalEvidenceVerificationStatementHashView {
        hash_domain: String,
        schema_version: u16,
        statement_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        anchor_reference_schema_version: u16,
        anchor_reference_profile: String,
        anchor_reference_sha256: String,
        verifier_schema_version: u16,
        verifier_profile: String,
        verification_claim: FederationStateMachineTraceExternalVerificationClaim,
        verifier_report_hash_algorithm: String,
        verifier_report_hash_encoding: String,
        verifier_report_sha256: String,
        claimed_verified_at_unix_seconds: u64,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTracePublicationCollectionReconciliationReceipt {
        schema_version: u16,
        reconciliation_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        left_collection_schema_version: u16,
        left_collection_profile: String,
        left_collection_hash_algorithm: String,
        left_collection_hash_encoding: String,
        left_collection_size: usize,
        left_collection_sha256: String,
        right_collection_schema_version: u16,
        right_collection_profile: String,
        right_collection_hash_algorithm: String,
        right_collection_hash_encoding: String,
        right_collection_size: usize,
        right_collection_sha256: String,
        relationship: FederationStateMachineTracePublicationCollectionRelationship,
        history_relationship:
            FederationStateMachineTracePublicationCollectionHistoryRelationship,
        shared_publication_sha256s: Vec<String>,
        left_only_publication_sha256s: Vec<String>,
        right_only_publication_sha256s: Vec<String>,
        equivocation_witness_sha256s: Vec<String>,
        reconciliation_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTracePublicationCollectionReconciliationHashView {
        hash_domain: String,
        schema_version: u16,
        reconciliation_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        left_collection_schema_version: u16,
        left_collection_profile: String,
        left_collection_hash_algorithm: String,
        left_collection_hash_encoding: String,
        left_collection_size: usize,
        left_collection_sha256: String,
        right_collection_schema_version: u16,
        right_collection_profile: String,
        right_collection_hash_algorithm: String,
        right_collection_hash_encoding: String,
        right_collection_size: usize,
        right_collection_sha256: String,
        relationship: FederationStateMachineTracePublicationCollectionRelationship,
        history_relationship:
            FederationStateMachineTracePublicationCollectionHistoryRelationship,
        shared_publication_sha256s: Vec<String>,
        left_only_publication_sha256s: Vec<String>,
        right_only_publication_sha256s: Vec<String>,
        equivocation_witness_sha256s: Vec<String>,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceCheckpointConsistencyReceipt {
        schema_version: u16,
        receipt_profile: String,
        proof_type: String,
        hash_algorithm: String,
        hash_encoding: String,
        trace_schema_version: u16,
        trace_verification_profile: String,
        trace_index: usize,
        earlier_publication_sha256: String,
        later_publication_sha256: String,
        earlier_evidence_end: usize,
        later_evidence_end: usize,
        receipt_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTraceCheckpointConsistencyReceiptHashView {
        hash_domain: String,
        schema_version: u16,
        receipt_profile: String,
        proof_type: String,
        hash_algorithm: String,
        hash_encoding: String,
        trace_schema_version: u16,
        trace_verification_profile: String,
        trace_index: usize,
        earlier_publication_sha256: String,
        later_publication_sha256: String,
        earlier_evidence_end: usize,
        later_evidence_end: usize,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceCheckpointPublication {
        schema_version: u16,
        publication_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        trace_schema_version: u16,
        trace_verification_profile: String,
        trace_index: usize,
        evidence_end: usize,
        body_sha256: String,
        chain_head_sha256: String,
        previous_publication_sha256: String,
        publication_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTracePublicationEquivocationWitness {
        schema_version: u16,
        witness_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        trace_schema_version: u16,
        trace_verification_profile: String,
        trace_index: usize,
        predecessor_publication_sha256: String,
        first_publication_sha256: String,
        second_publication_sha256: String,
        witness_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTracePublicationEquivocationWitnessSet {
        schema_version: u16,
        set_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        collection_schema_version: u16,
        collection_profile: String,
        collection_hash_algorithm: String,
        collection_hash_encoding: String,
        collection_size: usize,
        collection_sha256: String,
        witnesses: Vec<FederationStateMachineTracePublicationEquivocationWitness>,
        set_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTracePublicationEquivocationWitnessSetHashView {
        hash_domain: String,
        schema_version: u16,
        set_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        collection_schema_version: u16,
        collection_profile: String,
        collection_hash_algorithm: String,
        collection_hash_encoding: String,
        collection_size: usize,
        collection_sha256: String,
        witnesses: Vec<String>,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTracePublicationCollectionHashView {
        hash_domain: String,
        schema_version: u16,
        collection_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        publication_sha256s: Vec<String>,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTracePublicationEquivocationWitnessHashView {
        hash_domain: String,
        schema_version: u16,
        witness_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        trace_schema_version: u16,
        trace_verification_profile: String,
        trace_index: usize,
        predecessor_publication_sha256: String,
        first_publication_sha256: String,
        second_publication_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTraceCheckpointPublicationHashView {
        hash_domain: String,
        schema_version: u16,
        publication_profile: String,
        hash_algorithm: String,
        hash_encoding: String,
        trace_schema_version: u16,
        trace_verification_profile: String,
        trace_index: usize,
        evidence_end: usize,
        body_sha256: String,
        chain_head_sha256: String,
        previous_publication_sha256: String,
    }

    fn state_machine_trace_hash_view(
        capsule: &FederationStateMachineTraceCapsule,
    ) -> FederationStateMachineTraceHashView {
        FederationStateMachineTraceHashView {
            hash_domain: FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN.into(),
            schema_version: capsule.schema_version,
            verification_profile: capsule.verification_profile.clone(),
            trace_index: capsule.trace_index,
            initial_seed: capsule.initial_seed,
            operations: capsule.operations.clone(),
            tokens: capsule.tokens.clone(),
            initial_state: capsule.initial_state.clone(),
            evidence: capsule.evidence.clone(),
            final_state: capsule.final_state.clone(),
        }
    }

    fn state_machine_domain_separated_sha256(domain: &str, bytes: &[u8]) -> String {
        assert!(
            domain.as_bytes().iter().all(|byte| *byte != 0),
            "hash domains must not contain the delimiter byte"
        );
        let mut input = Vec::with_capacity(domain.len() + 1 + bytes.len());
        input.extend_from_slice(domain.as_bytes());
        input.push(0);
        input.extend_from_slice(bytes);
        let digest = Sha256::digest(input);
        format!("sha256:{digest:x}")
    }

    fn state_machine_evidence_chain_sha256(
        evidence: &FederationStateMachineEvidence,
    ) -> String {
        let view = FederationStateMachineEvidenceHashView {
            hash_domain: FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN.into(),
            step_index: evidence.step_index,
            operation: evidence.operation,
            token: evidence.token,
            decision: evidence.decision,
            authority: evidence.authority,
            pre_state_fingerprint: evidence.pre_state_fingerprint.clone(),
            state_fingerprint: evidence.state_fingerprint.clone(),
            chain_prev_sha256: evidence.chain_prev_sha256.clone(),
            pre_admission_index: evidence.pre_admission_index,
            post_admission_index: evidence.post_admission_index,
            pre_delivery_count: evidence.pre_delivery_count,
            post_delivery_count: evidence.post_delivery_count,
            newly_admitted_deliveries: evidence.newly_admitted_deliveries.clone(),
        };
        let bytes = serde_json::to_vec(&view)
            .expect("trace evidence hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_body_sha256(capsule: &FederationStateMachineTraceCapsule) -> String {
        let bytes = serde_json::to_vec(&state_machine_trace_hash_view(capsule))
            .expect("trace hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_integrity(capsule: &FederationStateMachineTraceCapsule) -> FederationStateMachineTraceIntegrity {
        FederationStateMachineTraceIntegrity {
            algorithm: FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ALGORITHM.into(),
            encoding: FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ENCODING.into(),
            body_sha256: state_machine_trace_body_sha256(capsule),
            chain_head_sha256: capsule
                .evidence
                .last()
                .map(|evidence| evidence.chain_sha256.clone())
                .unwrap_or_else(|| FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.into()),
        }
    }

    fn state_machine_trace_checkpoint_publication_sha256(
        publication: &FederationStateMachineTraceCheckpointPublication,
    ) -> String {
        let view = FederationStateMachineTraceCheckpointPublicationHashView {
            hash_domain: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_DOMAIN.into(),
            schema_version: publication.schema_version,
            publication_profile: publication.publication_profile.clone(),
            hash_algorithm: publication.hash_algorithm.clone(),
            hash_encoding: publication.hash_encoding.clone(),
            trace_schema_version: publication.trace_schema_version,
            trace_verification_profile: publication.trace_verification_profile.clone(),
            trace_index: publication.trace_index,
            evidence_end: publication.evidence_end,
            body_sha256: publication.body_sha256.clone(),
            chain_head_sha256: publication.chain_head_sha256.clone(),
            previous_publication_sha256: publication.previous_publication_sha256.clone(),
        };
        let bytes = serde_json::to_vec(&view)
            .expect("trace checkpoint publication hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_checkpoint_publication(
        capsule: &FederationStateMachineTraceCapsule,
        evidence_end: usize,
        previous_publication_sha256: &str,
    ) -> FederationStateMachineTraceCheckpointPublication {
        assert!(
            evidence_end <= capsule.evidence.len(),
            "trace checkpoint publication endpoint must be within the trace evidence"
        );
        let chain_head_sha256 = if evidence_end == 0 {
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.to_owned()
        } else {
            capsule.evidence[evidence_end - 1].chain_sha256.clone()
        };
        let mut publication = FederationStateMachineTraceCheckpointPublication {
            schema_version: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_SCHEMA_VERSION,
            publication_profile: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_PROFILE.into(),
            hash_algorithm: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_ALGORITHM.into(),
            hash_encoding: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_ENCODING.into(),
            trace_schema_version: capsule.schema_version,
            trace_verification_profile: capsule.verification_profile.clone(),
            trace_index: capsule.trace_index,
            evidence_end,
            body_sha256: capsule.integrity.body_sha256.clone(),
            chain_head_sha256,
            previous_publication_sha256: previous_publication_sha256.to_owned(),
            publication_sha256: String::new(),
        };
        publication.publication_sha256 =
            state_machine_trace_checkpoint_publication_sha256(&publication);
        publication
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceCheckpointPublicationViolation {
        UnsupportedSchemaVersion,
        UnsupportedPublicationProfile,
        UnsupportedPublicationHashAlgorithm,
        UnsupportedPublicationHashEncoding,
        TraceVerificationProfileMismatch,
        TraceSchemaVersionMismatch,
        TraceIndexMismatch,
        InvalidEvidenceEnd,
        BodyDigestMismatch,
        ChainHeadMismatch,
        PublicationDigestMismatch,
        UnsupportedTraceSchemaVersion,
        TraceEvidenceInvalid(FederationStateMachineTraceEvidenceViolation),
        FirstPublicationPredecessorMismatch,
        SnapshotProfileMismatch,
        SnapshotTraceIndexMismatch,
        SnapshotPrefixMismatch,
        SnapshotRollback,
        SameEndpointSnapshotMismatch,
        PreviousPublicationMismatch,
        PublicationChainEmpty,
        PublicationCollectionEmpty,
        PublicationForkDetected,
        PublicationLineageNoRoot,
        PublicationLineageIdentityMismatch,
        PublicationLineageDisconnected,
        PublicationLineageCycle,
        ConsistencyReceiptSchemaMismatch,
        ConsistencyReceiptProfileMismatch,
        ConsistencyReceiptProofTypeMismatch,
        ConsistencyReceiptHashAlgorithmMismatch,
        ConsistencyReceiptHashEncodingMismatch,
        ConsistencyReceiptTraceBindingMismatch,
        ConsistencyReceiptPublicationBindingMismatch,
        ConsistencyReceiptEndpointMismatch,
        ConsistencyReceiptDigestMismatch,
    }


    fn state_machine_trace_publication_equivocation_witness_sha256(
        witness: &FederationStateMachineTracePublicationEquivocationWitness,
    ) -> String {
        let view = FederationStateMachineTracePublicationEquivocationWitnessHashView {
            hash_domain:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_DOMAIN.into(),
            schema_version: witness.schema_version,
            witness_profile: witness.witness_profile.clone(),
            hash_algorithm: witness.hash_algorithm.clone(),
            hash_encoding: witness.hash_encoding.clone(),
            trace_schema_version: witness.trace_schema_version,
            trace_verification_profile: witness.trace_verification_profile.clone(),
            trace_index: witness.trace_index,
            predecessor_publication_sha256: witness.predecessor_publication_sha256.clone(),
            first_publication_sha256: witness.first_publication_sha256.clone(),
            second_publication_sha256: witness.second_publication_sha256.clone(),
        };
        let bytes = serde_json::to_vec(&view)
            .expect("publication equivocation witness hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_publication_equivocation_witness(
        first: &FederationStateMachineTraceCheckpointPublication,
        second: &FederationStateMachineTraceCheckpointPublication,
    ) -> FederationStateMachineTracePublicationEquivocationWitness {
        assert_eq!(
            first.publication_profile, second.publication_profile,
            "equivocation witness requires a shared publication profile"
        );
        assert_eq!(
            first.hash_algorithm, second.hash_algorithm,
            "equivocation witness requires a shared hash algorithm"
        );
        assert_eq!(
            first.hash_encoding, second.hash_encoding,
            "equivocation witness requires a shared hash encoding"
        );
        assert_eq!(
            first.schema_version, second.schema_version,
            "equivocation witness requires a shared publication schema"
        );
        assert_eq!(
            first.trace_verification_profile, second.trace_verification_profile,
            "equivocation witness requires a shared trace verification profile"
        );
        assert_eq!(
            first.trace_schema_version, second.trace_schema_version,
            "equivocation witness requires a shared trace schema"
        );
        assert_eq!(
            first.trace_index, second.trace_index,
            "equivocation witness requires a shared trace identity"
        );
        assert_eq!(
            first.previous_publication_sha256, second.previous_publication_sha256,
            "equivocation witness requires a shared predecessor"
        );
        assert_ne!(
            first.publication_sha256, second.publication_sha256,
            "equivocation witness requires distinct successor publications"
        );

        let (first_publication_sha256, second_publication_sha256) =
            if first.publication_sha256 <= second.publication_sha256 {
                (first.publication_sha256.clone(), second.publication_sha256.clone())
            } else {
                (second.publication_sha256.clone(), first.publication_sha256.clone())
            };

        let mut witness = FederationStateMachineTracePublicationEquivocationWitness {
            schema_version:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SCHEMA_VERSION,
            witness_profile:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_PROFILE.into(),
            hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_ALGORITHM.into(),
            hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_ENCODING.into(),
            trace_schema_version: first.trace_schema_version,
            trace_verification_profile: first.trace_verification_profile.clone(),
            trace_index: first.trace_index,
            predecessor_publication_sha256: first.previous_publication_sha256.clone(),
            first_publication_sha256,
            second_publication_sha256,
            witness_sha256: String::new(),
        };
        witness.witness_sha256 =
            state_machine_trace_publication_equivocation_witness_sha256(&witness);
        witness
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTracePublicationEquivocationWitnessViolation {
        UnsupportedSchemaVersion,
        UnsupportedWitnessProfile,
        UnsupportedHashAlgorithm,
        UnsupportedHashEncoding,
        TraceSchemaVersionMismatch,
        TraceVerificationProfileMismatch,
        TraceIndexMismatch,
        PredecessorMismatch,
        PublicationsNotDistinct,
        FirstPublicationInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        SecondPublicationInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        WitnessPublicationBindingMismatch,
        WitnessDigestMismatch,
    }

    fn validate_state_machine_trace_publication_equivocation_witness(
        first_capsule: &FederationStateMachineTraceCapsule,
        first_publication: &FederationStateMachineTraceCheckpointPublication,
        second_capsule: &FederationStateMachineTraceCapsule,
        second_publication: &FederationStateMachineTraceCheckpointPublication,
        witness: &FederationStateMachineTracePublicationEquivocationWitness,
    ) -> Result<(), FederationStateMachineTracePublicationEquivocationWitnessViolation> {
        validate_state_machine_trace_checkpoint_publication(first_capsule, first_publication)
            .map_err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::FirstPublicationInvalid,
            )?;
        validate_state_machine_trace_checkpoint_publication(second_capsule, second_publication)
            .map_err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::SecondPublicationInvalid,
            )?;

        if witness.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::UnsupportedSchemaVersion,
            );
        }
        if witness.witness_profile
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_PROFILE
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::UnsupportedWitnessProfile,
            );
        }
        if witness.hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::UnsupportedHashAlgorithm,
            );
        }
        if witness.hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::UnsupportedHashEncoding,
            );
        }
        if first_publication.trace_schema_version != second_publication.trace_schema_version
            || witness.trace_schema_version != first_publication.trace_schema_version
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::TraceSchemaVersionMismatch,
            );
        }
        if first_publication.trace_verification_profile
            != second_publication.trace_verification_profile
            || witness.trace_verification_profile
                != first_publication.trace_verification_profile
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::TraceVerificationProfileMismatch,
            );
        }
        if first_publication.trace_index != second_publication.trace_index
            || witness.trace_index != first_publication.trace_index
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::TraceIndexMismatch,
            );
        }
        if first_publication.previous_publication_sha256
            != second_publication.previous_publication_sha256
            || witness.predecessor_publication_sha256
                != first_publication.previous_publication_sha256
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::PredecessorMismatch,
            );
        }
        if first_publication.publication_sha256 == second_publication.publication_sha256 {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::PublicationsNotDistinct,
            );
        }

        let (expected_first, expected_second) =
            if first_publication.publication_sha256
                <= second_publication.publication_sha256
            {
                (
                    first_publication.publication_sha256.as_str(),
                    second_publication.publication_sha256.as_str(),
                )
            } else {
                (
                    second_publication.publication_sha256.as_str(),
                    first_publication.publication_sha256.as_str(),
                )
            };
        if witness.first_publication_sha256 != expected_first
            || witness.second_publication_sha256 != expected_second
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::WitnessPublicationBindingMismatch,
            );
        }
        if witness.witness_sha256
            != state_machine_trace_publication_equivocation_witness_sha256(witness)
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::WitnessDigestMismatch,
            );
        }

        Ok(())
    }

    fn state_machine_trace_publication_collection_digest_list(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<Vec<String>, FederationStateMachineTraceCheckpointPublicationViolation> {
        if snapshots.is_empty() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationCollectionEmpty
            );
        }

        let mut publication_sha256s = Vec::with_capacity(snapshots.len());
        for (capsule, publication) in snapshots {
            validate_state_machine_trace_checkpoint_publication(capsule, publication)?;
            publication_sha256s.push(publication.publication_sha256.clone());
        }
        publication_sha256s.sort();
        publication_sha256s.dedup();
        Ok(publication_sha256s)
    }

    fn state_machine_trace_publication_collection_commitment(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<(usize, String), FederationStateMachineTraceCheckpointPublicationViolation> {
        let publication_sha256s =
            state_machine_trace_publication_collection_digest_list(snapshots)?;

        let view = FederationStateMachineTracePublicationCollectionHashView {
            hash_domain:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_DOMAIN.into(),
            schema_version: FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION,
            collection_profile: FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE.into(),
            hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM.into(),
            hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING.into(),
            publication_sha256s: publication_sha256s.clone(),
        };
        let bytes = serde_json::to_vec(&view)
            .expect("publication collection hash view must be serializable");
        Ok((
            publication_sha256s.len(),
            state_machine_domain_separated_sha256(
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_DOMAIN,
                &bytes,
            ),
        ))
    }

    fn state_machine_trace_publication_collection_history_chain(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<Option<Vec<String>>, FederationStateMachineTraceCheckpointPublicationViolation> {
        if snapshots.is_empty() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationCollectionEmpty
            );
        }
        if validate_state_machine_trace_checkpoint_publication_lineage(snapshots).is_err() {
            return Ok(None);
        }
        let mut publications = BTreeMap::<
            &str,
            &FederationStateMachineTraceCheckpointPublication,
        >::new();
        for (_, publication) in snapshots {
            publications.entry(publication.publication_sha256.as_str()).or_insert(publication);
        }
        let root = publications
            .values()
            .find(|publication| {
                publication.previous_publication_sha256
                    == FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS
            })
            .expect("validated lineage must have a root");
        let mut successor_by_predecessor = BTreeMap::<&str, &str>::new();
        for publication in publications.values() {
            if publication.previous_publication_sha256
                != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS
            {
                successor_by_predecessor.insert(
                    publication.previous_publication_sha256.as_str(),
                    publication.publication_sha256.as_str(),
                );
            }
        }
        let mut chain = Vec::with_capacity(publications.len());
        let mut current = root.publication_sha256.as_str();
        loop {
            chain.push(current.to_owned());
            let Some(successor) = successor_by_predecessor.get(current).copied() else {
                break;
            };
            current = successor;
        }
        Ok(Some(chain))
    }

    fn state_machine_trace_publication_collection_history_relationship<'a>(
        left: &[(
            &'a FederationStateMachineTraceCapsule,
            &'a FederationStateMachineTraceCheckpointPublication,
        )],
        right: &[(
            &'a FederationStateMachineTraceCapsule,
            &'a FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<
        FederationStateMachineTracePublicationCollectionHistoryRelationship,
        FederationStateMachineTraceCheckpointPublicationViolation,
    > {
        let left_digests = state_machine_trace_publication_collection_digest_list(left)?;
        let right_digests = state_machine_trace_publication_collection_digest_list(right)?;
        let left_set = left_digests.iter().cloned().collect::<BTreeSet<_>>();
        let right_set = right_digests.iter().cloned().collect::<BTreeSet<_>>();

        if left_set == right_set {
            return Ok(
                FederationStateMachineTracePublicationCollectionHistoryRelationship::ExactMatch
            );
        }
        if left_set.is_disjoint(&right_set) {
            return Ok(
                FederationStateMachineTracePublicationCollectionHistoryRelationship::Disjoint
            );
        }

        let left_chain = state_machine_trace_publication_collection_history_chain(left)?;
        let right_chain = state_machine_trace_publication_collection_history_chain(right)?;

        match (left_chain, right_chain) {
            (Some(left_chain), Some(right_chain))
                if left_chain.len() < right_chain.len()
                    && right_chain.starts_with(&left_chain) =>
            {
                Ok(
                    FederationStateMachineTracePublicationCollectionHistoryRelationship::LeftStrictPrefix
                )
            }
            (Some(left_chain), Some(right_chain))
                if right_chain.len() < left_chain.len()
                    && left_chain.starts_with(&right_chain) =>
            {
                Ok(
                    FederationStateMachineTracePublicationCollectionHistoryRelationship::RightStrictPrefix
                )
            }
            (Some(left_chain), Some(right_chain))
                if left_chain.first() == right_chain.first() =>
            {
                Ok(
                    FederationStateMachineTracePublicationCollectionHistoryRelationship::DivergentAfterCommonPrefix
                )
            }
            _ => Ok(
                FederationStateMachineTracePublicationCollectionHistoryRelationship::NotComparable
            ),
        }
    }

    fn state_machine_trace_publication_collection_relationship(
        left: &[String],
        right: &[String],
    ) -> (
        FederationStateMachineTracePublicationCollectionRelationship,
        Vec<String>,
        Vec<String>,
        Vec<String>,
    ) {
        let left_set = left.iter().cloned().collect::<BTreeSet<_>>();
        let right_set = right.iter().cloned().collect::<BTreeSet<_>>();
        let shared = left_set
            .intersection(&right_set)
            .cloned()
            .collect::<Vec<_>>();
        let left_only = left_set
            .difference(&right_set)
            .cloned()
            .collect::<Vec<_>>();
        let right_only = right_set
            .difference(&left_set)
            .cloned()
            .collect::<Vec<_>>();

        let relationship = if left_set == right_set {
            FederationStateMachineTracePublicationCollectionRelationship::ExactMatch
        } else if left_set.is_subset(&right_set) {
            FederationStateMachineTracePublicationCollectionRelationship::LeftStrictSubset
        } else if right_set.is_subset(&left_set) {
            FederationStateMachineTracePublicationCollectionRelationship::RightStrictSubset
        } else if shared.is_empty() {
            FederationStateMachineTracePublicationCollectionRelationship::Disjoint
        } else {
            FederationStateMachineTracePublicationCollectionRelationship::OverlappingDivergence
        };

        (relationship, shared, left_only, right_only)
    }

    fn state_machine_trace_publication_collection_reconciliation_sha256(
        receipt: &FederationStateMachineTracePublicationCollectionReconciliationReceipt,
    ) -> String {
        let view = FederationStateMachineTracePublicationCollectionReconciliationHashView {
            hash_domain:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_DOMAIN.into(),
            schema_version: receipt.schema_version,
            reconciliation_profile: receipt.reconciliation_profile.clone(),
            hash_algorithm: receipt.hash_algorithm.clone(),
            hash_encoding: receipt.hash_encoding.clone(),
            left_collection_schema_version: receipt.left_collection_schema_version,
            left_collection_profile: receipt.left_collection_profile.clone(),
            left_collection_hash_algorithm: receipt.left_collection_hash_algorithm.clone(),
            left_collection_hash_encoding: receipt.left_collection_hash_encoding.clone(),
            left_collection_size: receipt.left_collection_size,
            left_collection_sha256: receipt.left_collection_sha256.clone(),
            right_collection_schema_version: receipt.right_collection_schema_version,
            right_collection_profile: receipt.right_collection_profile.clone(),
            right_collection_hash_algorithm: receipt.right_collection_hash_algorithm.clone(),
            right_collection_hash_encoding: receipt.right_collection_hash_encoding.clone(),
            right_collection_size: receipt.right_collection_size,
            right_collection_sha256: receipt.right_collection_sha256.clone(),
            relationship: receipt.relationship,
            history_relationship: receipt.history_relationship,
            shared_publication_sha256s: receipt.shared_publication_sha256s.clone(),
            left_only_publication_sha256s: receipt.left_only_publication_sha256s.clone(),
            right_only_publication_sha256s: receipt.right_only_publication_sha256s.clone(),
            equivocation_witness_sha256s: receipt.equivocation_witness_sha256s.clone(),
        };
        let bytes = serde_json::to_vec(&view)
            .expect("publication collection reconciliation hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_external_witness_artifact_sha256(bytes: &[u8]) -> String {
        let digest = Sha256::digest(bytes);
        format!("sha256:{digest:x}")
    }

    fn state_machine_trace_is_sha256_digest(value: &str) -> bool {
        let Some(hex) = value.strip_prefix("sha256:") else {
            return false;
        };
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || matches!(byte, b'a'..=b'f'))
    }

    fn state_machine_trace_external_evidence_verification_statement_sha256(
        statement: &FederationStateMachineTraceExternalEvidenceVerificationStatement,
    ) -> String {
        let view = FederationStateMachineTraceExternalEvidenceVerificationStatementHashView {
            hash_domain:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_DOMAIN.into(),
            schema_version: statement.schema_version,
            statement_profile: statement.statement_profile.clone(),
            hash_algorithm: statement.hash_algorithm.clone(),
            hash_encoding: statement.hash_encoding.clone(),
            anchor_reference_schema_version: statement.anchor_reference_schema_version,
            anchor_reference_profile: statement.anchor_reference_profile.clone(),
            anchor_reference_sha256: statement.anchor_reference_sha256.clone(),
            verifier_schema_version: statement.verifier_schema_version,
            verifier_profile: statement.verifier_profile.clone(),
            verification_claim: statement.verification_claim,
            verifier_report_hash_algorithm: statement.verifier_report_hash_algorithm.clone(),
            verifier_report_hash_encoding: statement.verifier_report_hash_encoding.clone(),
            verifier_report_sha256: statement.verifier_report_sha256.clone(),
            claimed_verified_at_unix_seconds: statement.claimed_verified_at_unix_seconds,
        };
        let bytes = serde_json::to_vec(&view)
            .expect("external evidence verification statement hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_external_evidence_anchor_reference_sha256(
        reference: &FederationStateMachineTraceExternalEvidenceAnchorReference,
    ) -> String {
        let view = FederationStateMachineTraceExternalEvidenceAnchorReferenceHashView {
            hash_domain:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_DOMAIN.into(),
            schema_version: reference.schema_version,
            reference_profile: reference.reference_profile.clone(),
            hash_algorithm: reference.hash_algorithm.clone(),
            hash_encoding: reference.hash_encoding.clone(),
            subject_hash_algorithm: reference.subject_hash_algorithm.clone(),
            subject_hash_encoding: reference.subject_hash_encoding.clone(),
            subject_schema_version: reference.subject_schema_version,
            subject_profile: reference.subject_profile.clone(),
            subject_sha256: reference.subject_sha256.clone(),
            witness_hash_algorithm: reference.witness_hash_algorithm.clone(),
            witness_hash_encoding: reference.witness_hash_encoding.clone(),
            witness_schema_version: reference.witness_schema_version,
            witness_kind: reference.witness_kind,
            witness_profile: reference.witness_profile.clone(),
            witness_sha256: reference.witness_sha256.clone(),
            claimed_observed_at_unix_seconds: reference.claimed_observed_at_unix_seconds,
        };
        let bytes = serde_json::to_vec(&view)
            .expect("external evidence anchor reference hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_publication_equivocation_witness_set_sha256(
        witness_set: &FederationStateMachineTracePublicationEquivocationWitnessSet,
    ) -> String {
        let view = FederationStateMachineTracePublicationEquivocationWitnessSetHashView {
            hash_domain:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_DOMAIN.into(),
            schema_version: witness_set.schema_version,
            set_profile: witness_set.set_profile.clone(),
            hash_algorithm: witness_set.hash_algorithm.clone(),
            hash_encoding: witness_set.hash_encoding.clone(),
            collection_schema_version: witness_set.collection_schema_version,
            collection_profile: witness_set.collection_profile.clone(),
            collection_hash_algorithm: witness_set.collection_hash_algorithm.clone(),
            collection_hash_encoding: witness_set.collection_hash_encoding.clone(),
            collection_size: witness_set.collection_size,
            collection_sha256: witness_set.collection_sha256.clone(),
            witnesses: witness_set
                .witnesses
                .iter()
                .map(|witness| witness.witness_sha256.clone())
                .collect(),
        };
        let bytes = serde_json::to_vec(&view)
            .expect("publication equivocation witness set hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_checkpoint_consistency_receipt_sha256(
        receipt: &FederationStateMachineTraceCheckpointConsistencyReceipt,
    ) -> String {
        let view = FederationStateMachineTraceCheckpointConsistencyReceiptHashView {
            hash_domain:
                FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_DOMAIN.into(),
            schema_version: receipt.schema_version,
            receipt_profile: receipt.receipt_profile.clone(),
            proof_type: receipt.proof_type.clone(),
            hash_algorithm: receipt.hash_algorithm.clone(),
            hash_encoding: receipt.hash_encoding.clone(),
            trace_schema_version: receipt.trace_schema_version,
            trace_verification_profile: receipt.trace_verification_profile.clone(),
            trace_index: receipt.trace_index,
            earlier_publication_sha256: receipt.earlier_publication_sha256.clone(),
            later_publication_sha256: receipt.later_publication_sha256.clone(),
            earlier_evidence_end: receipt.earlier_evidence_end,
            later_evidence_end: receipt.later_evidence_end,
        };
        let bytes = serde_json::to_vec(&view)
            .expect("trace checkpoint consistency receipt hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_checkpoint_consistency_receipt(
        earlier_publication: &FederationStateMachineTraceCheckpointPublication,
        later_publication: &FederationStateMachineTraceCheckpointPublication,
    ) -> FederationStateMachineTraceCheckpointConsistencyReceipt {
        assert_eq!(
            earlier_publication.trace_schema_version,
            later_publication.trace_schema_version,
            "consistency receipt requires matching trace schema versions"
        );
        assert_eq!(
            earlier_publication.trace_verification_profile,
            later_publication.trace_verification_profile,
            "consistency receipt requires matching verification profiles"
        );
        assert_eq!(
            earlier_publication.trace_index,
            later_publication.trace_index,
            "consistency receipt requires matching trace identities"
        );
        assert!(
            earlier_publication.evidence_end <= later_publication.evidence_end,
            "consistency receipt requires a non-rollback evidence endpoint"
        );

        let mut receipt = FederationStateMachineTraceCheckpointConsistencyReceipt {
            schema_version:
                FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_SCHEMA_VERSION,
            receipt_profile:
                FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_PROFILE.into(),
            proof_type:
                FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_PROOF_TYPE.into(),
            hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_ALGORITHM.into(),
            hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_ENCODING.into(),
            trace_schema_version: earlier_publication.trace_schema_version,
            trace_verification_profile: earlier_publication.trace_verification_profile.clone(),
            trace_index: earlier_publication.trace_index,
            earlier_publication_sha256: earlier_publication.publication_sha256.clone(),
            later_publication_sha256: later_publication.publication_sha256.clone(),
            earlier_evidence_end: earlier_publication.evidence_end,
            later_evidence_end: later_publication.evidence_end,
            receipt_sha256: String::new(),
        };
        receipt.receipt_sha256 =
            state_machine_trace_checkpoint_consistency_receipt_sha256(&receipt);
        receipt
    }

    fn validate_state_machine_trace_checkpoint_consistency_receipt(
        earlier_capsule: &FederationStateMachineTraceCapsule,
        earlier_publication: &FederationStateMachineTraceCheckpointPublication,
        later_capsule: &FederationStateMachineTraceCapsule,
        later_publication: &FederationStateMachineTraceCheckpointPublication,
        receipt: &FederationStateMachineTraceCheckpointConsistencyReceipt,
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        validate_state_machine_trace_checkpoint_publication_consistency(
            earlier_capsule,
            earlier_publication,
            later_capsule,
            later_publication,
        )?;

        if receipt.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptSchemaMismatch
            );
        }
        if receipt.receipt_profile
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_PROFILE
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptProfileMismatch
            );
        }
        if receipt.proof_type
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_PROOF_TYPE
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptProofTypeMismatch
            );
        }
        if receipt.hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptHashAlgorithmMismatch
            );
        }
        if receipt.hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptHashEncodingMismatch
            );
        }
        if receipt.trace_schema_version != earlier_publication.trace_schema_version
            || receipt.trace_schema_version != later_publication.trace_schema_version
            || receipt.trace_verification_profile != earlier_publication.trace_verification_profile
            || receipt.trace_verification_profile != later_publication.trace_verification_profile
            || receipt.trace_index != earlier_publication.trace_index
            || receipt.trace_index != later_publication.trace_index
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptTraceBindingMismatch
            );
        }
        if receipt.earlier_publication_sha256 != earlier_publication.publication_sha256
            || receipt.later_publication_sha256 != later_publication.publication_sha256
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptPublicationBindingMismatch
            );
        }
        if receipt.earlier_evidence_end != earlier_publication.evidence_end
            || receipt.later_evidence_end != later_publication.evidence_end
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptEndpointMismatch
            );
        }
        if receipt.receipt_sha256
            != state_machine_trace_checkpoint_consistency_receipt_sha256(receipt)
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptDigestMismatch
            );
        }

        Ok(())
    }

    /// Performs the qualified collection audit before invoking the lower-level
    /// publication-only fork detector. The lower-level detector intentionally
    /// assumes that its records have already been validated against snapshots;
    /// this wrapper makes that precondition executable for callers that possess
    /// the concrete snapshot/publication pairs.
    fn validate_state_machine_trace_checkpoint_publication_set_against_snapshots(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        if snapshots.is_empty() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationCollectionEmpty
            );
        }

        let mut canonical_snapshots = snapshots.iter().copied().collect::<Vec<_>>();
        canonical_snapshots.sort_by_key(|(capsule, publication)| {
            serde_json::to_string((capsule, publication))
                .expect("publication collection audit entry must be serializable")
        });

        for (capsule, publication) in &canonical_snapshots {
            validate_state_machine_trace_checkpoint_publication(capsule, publication)?;
        }
        let publications = canonical_snapshots
            .iter()
            .map(|(_, publication)| (*publication).clone())
            .collect::<Vec<_>>();
        validate_state_machine_trace_checkpoint_publication_set(&publications)
    }

    fn validate_state_machine_trace_checkpoint_publication_set(
        publications: &[FederationStateMachineTraceCheckpointPublication],
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        let mut successors =
            BTreeMap::<(&str, &str, u16, &str, u16, &str, usize, &str), &str>::new();

        for publication in publications {
            let key = (
                publication.publication_profile.as_str(),
                publication.hash_algorithm.as_str(),
                publication.schema_version,
                publication.trace_verification_profile.as_str(),
                publication.trace_schema_version,
                publication.hash_encoding.as_str(),
                publication.trace_index,
                publication.previous_publication_sha256.as_str(),
            );
            if let Some(existing) = successors.insert(
                key,
                publication.publication_sha256.as_str(),
            ) {
                if existing != publication.publication_sha256.as_str() {
                    return Err(
                        FederationStateMachineTraceCheckpointPublicationViolation::PublicationForkDetected
                    );
                }
            }
        }

        Ok(())
    }


    /// Collects every distinct successor pair that constitutes equivocation in a
    /// qualified concrete publication/snapshot collection. Exact publication replays
    /// are deduplicated by publication digest; every remaining conflicting pair is
    /// preserved as its own canonical witness.
    fn collect_state_machine_trace_publication_equivocation_witnesses(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<
        Vec<FederationStateMachineTracePublicationEquivocationWitness>,
        FederationStateMachineTraceCheckpointPublicationViolation,
    > {
        if snapshots.is_empty() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationCollectionEmpty
            );
        }

        let mut canonical_snapshots = snapshots.iter().copied().collect::<Vec<_>>();
        canonical_snapshots.sort_by_key(|(capsule, publication)| {
            serde_json::to_string((capsule, publication))
                .expect("publication witness candidate must be serializable")
        });

        for (capsule, publication) in &canonical_snapshots {
            validate_state_machine_trace_checkpoint_publication(capsule, publication)?;
        }

        let mut successors = BTreeMap::new();
        for (capsule, publication) in canonical_snapshots {
            let key = (
                publication.publication_profile.clone(),
                publication.hash_algorithm.clone(),
                publication.schema_version,
                publication.trace_verification_profile.clone(),
                publication.trace_schema_version,
                publication.hash_encoding.clone(),
                publication.trace_index,
                publication.previous_publication_sha256.clone(),
            );
            successors
                .entry(key)
                .or_insert_with(Vec::new)
                .push((
                    publication.publication_sha256.clone(),
                    capsule,
                    publication,
                ));
        }

        let mut witnesses = Vec::new();
        for successor_group in successors.values_mut() {
            successor_group.sort_by(|left, right| left.0.cmp(&right.0));
            successor_group.dedup_by(|left, right| left.0 == right.0);

            for left in 0..successor_group.len() {
                for right in (left + 1)..successor_group.len() {
                    let (_, _, first) = successor_group[left];
                    let (_, _, second) = successor_group[right];
                    witnesses.push(
                        state_machine_trace_publication_equivocation_witness(
                            first,
                            second,
                        ),
                    );
                }
            }
        }

        witnesses.sort_by(|left, right| left.witness_sha256.cmp(&right.witness_sha256));
        Ok(witnesses)
    }

    /// Finds the first canonical equivocation witness. Callers that need complete
    /// fork coverage should use the collection form above.
    fn find_state_machine_trace_publication_equivocation_witness(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<
        Option<FederationStateMachineTracePublicationEquivocationWitness>,
        FederationStateMachineTraceCheckpointPublicationViolation,
    > {
        Ok(
            collect_state_machine_trace_publication_equivocation_witnesses(snapshots)?
                .into_iter()
                .next(),
        )
    }

    /// Builds a self-validating complete equivocation-witness set. A valid set is
    /// exhaustive for the supplied qualified publication collection: omission,
    /// duplication, or reordering is rejected.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTracePublicationEquivocationWitnessSetBuildViolation {
        PublicationInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        NoEquivocationDetected,
    }

    fn state_machine_trace_publication_equivocation_witness_set(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<
        FederationStateMachineTracePublicationEquivocationWitnessSet,
        FederationStateMachineTracePublicationEquivocationWitnessSetBuildViolation,
    > {
        let witnesses =
            collect_state_machine_trace_publication_equivocation_witnesses(snapshots)
                .map_err(
                    FederationStateMachineTracePublicationEquivocationWitnessSetBuildViolation::PublicationInvalid,
                )?;
        if witnesses.is_empty() {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetBuildViolation::NoEquivocationDetected
            );
        }
        let (collection_size, collection_sha256) =
            state_machine_trace_publication_collection_commitment(snapshots)
                .map_err(
                    FederationStateMachineTracePublicationEquivocationWitnessSetBuildViolation::PublicationInvalid,
                )?;

        let mut set = FederationStateMachineTracePublicationEquivocationWitnessSet {
            schema_version:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_SCHEMA_VERSION,
            set_profile:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_PROFILE.into(),
            hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_ALGORITHM.into(),
            hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_ENCODING.into(),
            collection_schema_version:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION,
            collection_profile:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE.into(),
            collection_hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM.into(),
            collection_hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING.into(),
            collection_size,
            collection_sha256,
            witnesses,
            set_sha256: String::new(),
        };
        set.set_sha256 = state_machine_trace_publication_equivocation_witness_set_sha256(&set);
        Ok(set)
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTracePublicationCollectionReconciliationBuildViolation {
        LeftCollectionInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        RightCollectionInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        UnionCollectionInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
    }

    fn state_machine_trace_publication_collection_reconciliation<'a>(
        left: &[(
            &'a FederationStateMachineTraceCapsule,
            &'a FederationStateMachineTraceCheckpointPublication,
        )],
        right: &[(
            &'a FederationStateMachineTraceCapsule,
            &'a FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<
        FederationStateMachineTracePublicationCollectionReconciliationReceipt,
        FederationStateMachineTracePublicationCollectionReconciliationBuildViolation,
    > {
        let left_digests =
            state_machine_trace_publication_collection_digest_list(left).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationBuildViolation::LeftCollectionInvalid,
            )?;
        let right_digests =
            state_machine_trace_publication_collection_digest_list(right).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationBuildViolation::RightCollectionInvalid,
            )?;

        let (left_size, left_collection_sha256) =
            state_machine_trace_publication_collection_commitment(left).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationBuildViolation::LeftCollectionInvalid,
            )?;
        let (right_size, right_collection_sha256) =
            state_machine_trace_publication_collection_commitment(right).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationBuildViolation::RightCollectionInvalid,
            )?;

        let (relationship, shared, left_only, right_only) =
            state_machine_trace_publication_collection_relationship(&left_digests, &right_digests);

        let mut union = Vec::with_capacity(left.len() + right.len());
        union.extend(left.iter().copied());
        union.extend(right.iter().copied());
        let equivocation_witness_sha256s =
            collect_state_machine_trace_publication_equivocation_witnesses(&union)
                .map_err(
                    FederationStateMachineTracePublicationCollectionReconciliationBuildViolation::UnionCollectionInvalid,
                )?
                .into_iter()
                .map(|witness| witness.witness_sha256)
                .collect::<Vec<_>>();

        let mut receipt = FederationStateMachineTracePublicationCollectionReconciliationReceipt {
            schema_version:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION,
            reconciliation_profile:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE.into(),
            hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_ALGORITHM.into(),
            hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_ENCODING.into(),
            left_collection_schema_version:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION,
            left_collection_profile:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE.into(),
            left_collection_hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM.into(),
            left_collection_hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING.into(),
            left_collection_size: left_size,
            left_collection_sha256: left_collection_sha256,
            right_collection_schema_version:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION,
            right_collection_profile:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE.into(),
            right_collection_hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM.into(),
            right_collection_hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING.into(),
            right_collection_size: right_size,
            right_collection_sha256: right_collection_sha256,
            relationship,
            history_relationship:
                state_machine_trace_publication_collection_history_relationship(left, right)
                    .map_err(
                        FederationStateMachineTracePublicationCollectionReconciliationBuildViolation::UnionCollectionInvalid,
                    )?,
            shared_publication_sha256s: shared,
            left_only_publication_sha256s: left_only,
            right_only_publication_sha256s: right_only,
            equivocation_witness_sha256s,
            reconciliation_sha256: String::new(),
        };
        receipt.reconciliation_sha256 =
            state_machine_trace_publication_collection_reconciliation_sha256(&receipt);
        Ok(receipt)
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation {
        EmptySubjectProfile,
        EmptySubjectDigest,
        InvalidSubjectDigest,
        InvalidWitnessSchemaVersion,
        EmptyWitnessProfile,
        EmptyWitnessArtifact,
    }

    fn state_machine_trace_external_evidence_anchor_reference(
        subject_schema_version: u16,
        subject_profile: &str,
        subject_sha256: &str,
        witness_kind: FederationStateMachineTraceExternalWitnessKind,
        witness_schema_version: u16,
        witness_profile: &str,
        witness_artifact: &[u8],
        claimed_observed_at_unix_seconds: u64,
    ) -> Result<
        FederationStateMachineTraceExternalEvidenceAnchorReference,
        FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation,
    > {
        if subject_profile.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::EmptySubjectProfile
            );
        }
        if subject_sha256.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::EmptySubjectDigest
            );
        }
        if !state_machine_trace_is_sha256_digest(subject_sha256) {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::InvalidSubjectDigest
            );
        }
        if witness_schema_version == 0 {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::InvalidWitnessSchemaVersion
            );
        }
        if witness_profile.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::EmptyWitnessProfile
            );
        }
        if witness_artifact.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceBuildViolation::EmptyWitnessArtifact
            );
        }

        let mut reference = FederationStateMachineTraceExternalEvidenceAnchorReference {
            schema_version:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SCHEMA_VERSION,
            reference_profile:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_PROFILE.into(),
            hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_ALGORITHM.into(),
            hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_ENCODING.into(),
            subject_hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SUBJECT_HASH_ALGORITHM.into(),
            subject_hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SUBJECT_HASH_ENCODING.into(),
            subject_schema_version,
            subject_profile: subject_profile.into(),
            subject_sha256: subject_sha256.into(),
            witness_hash_algorithm:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_WITNESS_HASH_ALGORITHM.into(),
            witness_hash_encoding:
                FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_WITNESS_HASH_ENCODING.into(),
            witness_schema_version,
            witness_kind,
            witness_profile: witness_profile.into(),
            witness_sha256: state_machine_trace_external_witness_artifact_sha256(witness_artifact),
            claimed_observed_at_unix_seconds,
            anchor_reference_sha256: String::new(),
        };
        reference.anchor_reference_sha256 =
            state_machine_trace_external_evidence_anchor_reference_sha256(&reference);
        Ok(reference)
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation {
        UnsupportedSchemaVersion,
        UnsupportedReferenceProfile,
        UnsupportedHashAlgorithm,
        UnsupportedHashEncoding,
        EmptySubjectProfile,
        EmptySubjectDigest,
        InvalidSubjectDigest,
        SubjectHashAlgorithmMismatch,
        SubjectHashEncodingMismatch,
        EmptyWitnessProfile,
        EmptyWitnessDigest,
        WitnessHashAlgorithmMismatch,
        WitnessHashEncodingMismatch,
        WitnessSchemaVersionMismatch,
        EmptyWitnessArtifact,
        SubjectSchemaVersionMismatch,
        SubjectProfileMismatch,
        SubjectDigestMismatch,
        WitnessDigestMismatch,
        AnchorReferenceDigestMismatch,
    }

    /// Verifies that a reference binds one exact model artifact to one exact
    /// externally supplied witness byte sequence.
    ///
    /// This verifier intentionally stops at the binding boundary. It does not
    /// verify a TSA signature, a transparency-log signature, an archive policy,
    /// observer independence, or the truth of the claimed observation time.
    fn validate_state_machine_trace_external_evidence_anchor_reference(
        subject_schema_version: u16,
        subject_profile: &str,
        subject_sha256: &str,
        witness_artifact: &[u8],
        reference: &FederationStateMachineTraceExternalEvidenceAnchorReference,
    ) -> Result<
        (),
        FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation,
    > {
        if reference.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::UnsupportedSchemaVersion
            );
        }
        if reference.reference_profile
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_PROFILE
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::UnsupportedReferenceProfile
            );
        }
        if reference.hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::UnsupportedHashAlgorithm
            );
        }
        if reference.hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::UnsupportedHashEncoding
            );
        }
        if reference.subject_profile.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::EmptySubjectProfile
            );
        }
        if reference.subject_sha256.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::EmptySubjectDigest
            );
        }
        if !state_machine_trace_is_sha256_digest(&reference.subject_sha256) {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::InvalidSubjectDigest
            );
        }
        if reference.witness_schema_version == 0 {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessSchemaVersionMismatch
            );
        }
        if reference.witness_profile.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::EmptyWitnessProfile
            );
        }
        if reference.witness_sha256.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::EmptyWitnessDigest
            );
        }
        if witness_artifact.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::EmptyWitnessArtifact
            );
        }
        if reference.subject_schema_version != subject_schema_version {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::SubjectSchemaVersionMismatch
            );
        }
        if reference.subject_hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SUBJECT_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::SubjectHashAlgorithmMismatch
            );
        }
        if reference.subject_hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SUBJECT_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::SubjectHashEncodingMismatch
            );
        }
        if reference.subject_profile != subject_profile {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::SubjectProfileMismatch
            );
        }
        if reference.subject_sha256 != subject_sha256 {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::SubjectDigestMismatch
            );
        }
        if reference.witness_hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_WITNESS_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessHashAlgorithmMismatch
            );
        }
        if reference.witness_hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_WITNESS_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessHashEncodingMismatch
            );
        }
        let expected_witness_sha256 =
            state_machine_trace_external_witness_artifact_sha256(witness_artifact);
        if reference.witness_sha256 != expected_witness_sha256 {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::WitnessDigestMismatch
            );
        }
        if reference.anchor_reference_sha256
            != state_machine_trace_external_evidence_anchor_reference_sha256(reference)
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation::AnchorReferenceDigestMismatch
            );
        }
        Ok(())
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation {
        EmptyVerifierProfile,
        EmptyVerifierReport,
    }

    fn state_machine_trace_external_evidence_verification_statement(
        anchor_reference_sha256: &str,
        verifier_schema_version: u16,
        verifier_profile: &str,
        verification_claim: FederationStateMachineTraceExternalVerificationClaim,
        verifier_report: &[u8],
        claimed_verified_at_unix_seconds: u64,
    ) -> Result<
        FederationStateMachineTraceExternalEvidenceVerificationStatement,
        FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation,
    > {
        if !state_machine_trace_is_sha256_digest(anchor_reference_sha256) {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation::InvalidAnchorReferenceDigest
            );
        }
        if verifier_schema_version == 0 {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation::InvalidVerifierReportSchemaVersion
            );
        }
        if verifier_profile.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation::EmptyVerifierProfile
            );
        }
        if verifier_report.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementBuildViolation::EmptyVerifierReport
            );
        }

        let mut statement =
            FederationStateMachineTraceExternalEvidenceVerificationStatement {
                schema_version:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_SCHEMA_VERSION,
                statement_profile:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_PROFILE.into(),
                hash_algorithm:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_ALGORITHM.into(),
                hash_encoding:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_ENCODING.into(),
                anchor_reference_schema_version:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SCHEMA_VERSION,
                anchor_reference_profile:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_PROFILE.into(),
                anchor_reference_sha256: anchor_reference_sha256.into(),
                verifier_schema_version,
                verifier_profile: verifier_profile.into(),
                verification_claim,
                verifier_report_hash_algorithm:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_REPORT_HASH_ALGORITHM.into(),
                verifier_report_hash_encoding:
                    FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_REPORT_HASH_ENCODING.into(),
                verifier_report_sha256:
                    state_machine_trace_external_witness_artifact_sha256(verifier_report),
                claimed_verified_at_unix_seconds,
                statement_sha256: String::new(),
            };
        statement.statement_sha256 =
            state_machine_trace_external_evidence_verification_statement_sha256(&statement);
        Ok(statement)
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceExternalEvidenceVerificationStatementViolation {
        UnsupportedSchemaVersion,
        UnsupportedStatementProfile,
        UnsupportedHashAlgorithm,
        UnsupportedHashEncoding,
        InvalidAnchorReferenceDigest,
        AnchorReferenceInvalid(FederationStateMachineTraceExternalEvidenceAnchorReferenceViolation),
        AnchorReferenceSchemaVersionMismatch,
        AnchorReferenceProfileMismatch,
        AnchorReferenceDigestMismatch,
        InvalidVerifierReportSchemaVersion,
        EmptyVerifierProfile,
        VerifierReportHashAlgorithmMismatch,
        VerifierReportHashEncodingMismatch,
        EmptyVerifierReportDigest,
        InvalidVerifierReportDigest,
        VerifierReportDigestMismatch,
        StatementDigestMismatch,
    }

    /// Verifies only the binding of an external verifier statement to an exact
    /// anchor reference and exact verifier-report byte sequence.
    fn validate_state_machine_trace_external_evidence_verification_statement_binding(
        anchor_reference_sha256: &str,
        verifier_report: &[u8],
        statement: &FederationStateMachineTraceExternalEvidenceVerificationStatement,
    ) -> Result<
        (),
        FederationStateMachineTraceExternalEvidenceVerificationStatementViolation,
    > {
        if statement.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::UnsupportedSchemaVersion
            );
        }
        if statement.statement_profile
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_PROFILE
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::UnsupportedStatementProfile
            );
        }
        if statement.hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::UnsupportedHashAlgorithm
            );
        }
        if statement.hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::UnsupportedHashEncoding
            );
        }
        if statement.anchor_reference_schema_version
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceSchemaVersionMismatch
            );
        }
        if statement.anchor_reference_profile
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_ANCHOR_REFERENCE_PROFILE
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceProfileMismatch
            );
        }
        if !state_machine_trace_is_sha256_digest(anchor_reference_sha256) {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::InvalidAnchorReferenceDigest
            );
        }
        if statement.anchor_reference_sha256 != anchor_reference_sha256 {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceDigestMismatch
            );
        }
        if statement.verifier_schema_version == 0 {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::InvalidVerifierReportSchemaVersion
            );
        }
        if statement.verifier_profile.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::EmptyVerifierProfile
            );
        }
        if statement.verifier_report_hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_REPORT_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::VerifierReportHashAlgorithmMismatch
            );
        }
        if statement.verifier_report_hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_EXTERNAL_EVIDENCE_VERIFICATION_STATEMENT_REPORT_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::VerifierReportHashEncodingMismatch
            );
        }
        if statement.verifier_report_sha256.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::EmptyVerifierReportDigest
            );
        }
        if !state_machine_trace_is_sha256_digest(&statement.verifier_report_sha256) {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::InvalidVerifierReportDigest
            );
        }
        if verifier_report.is_empty() {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::VerifierReportDigestMismatch
            );
        }
        let expected_report_sha256 =
            state_machine_trace_external_witness_artifact_sha256(verifier_report);
        if statement.verifier_report_sha256 != expected_report_sha256 {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::VerifierReportDigestMismatch
            );
        }
        if statement.statement_sha256
            != state_machine_trace_external_evidence_verification_statement_sha256(statement)
        {
            return Err(
                FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::StatementDigestMismatch
            );
        }
        Ok(())
    }

    fn validate_state_machine_trace_external_evidence_verification_statement_chain(
        subject_schema_version: u16,
        subject_profile: &str,
        subject_sha256: &str,
        anchor_witness_artifact: &[u8],
        anchor_reference: &FederationStateMachineTraceExternalEvidenceAnchorReference,
        verifier_report: &[u8],
        statement: &FederationStateMachineTraceExternalEvidenceVerificationStatement,
    ) -> Result<
        FederationStateMachineTraceExternalEvidenceVerificationResult,
        FederationStateMachineTraceExternalEvidenceVerificationStatementViolation,
    > {
        validate_state_machine_trace_external_evidence_anchor_reference(
            subject_schema_version,
            subject_profile,
            subject_sha256,
            anchor_witness_artifact,
            anchor_reference,
        )
        .map_err(
            FederationStateMachineTraceExternalEvidenceVerificationStatementViolation::AnchorReferenceInvalid,
        )?;
        validate_state_machine_trace_external_evidence_verification_statement_binding(
            &anchor_reference.anchor_reference_sha256,
            verifier_report,
            statement,
        )?;

        Ok(FederationStateMachineTraceExternalEvidenceVerificationResult {
            anchor_reference_sha256: statement.anchor_reference_sha256.clone(),
            anchor_reference_schema_version: statement.anchor_reference_schema_version,
            anchor_reference_profile: statement.anchor_reference_profile.clone(),
            verifier_schema_version: statement.verifier_schema_version,
            verifier_profile: statement.verifier_profile.clone(),
            claim: statement.verification_claim,
            verifier_report_sha256: statement.verifier_report_sha256.clone(),
            claimed_verified_at_unix_seconds: statement.claimed_verified_at_unix_seconds,
            statement_sha256: statement.statement_sha256.clone(),
        })
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTracePublicationEquivocationWitnessSetViolation {
        UnsupportedSchemaVersion,
        UnsupportedSetProfile,
        UnsupportedHashAlgorithm,
        UnsupportedHashEncoding,
        UnsupportedCollectionSchemaVersion,
        UnsupportedCollectionProfile,
        UnsupportedCollectionHashAlgorithm,
        UnsupportedCollectionHashEncoding,
        CollectionSizeMismatch,
        CollectionDigestMismatch,
        EmptyWitnessSet,
        WitnessNotCanonical,
        DuplicateWitness,
        WitnessInvalid(FederationStateMachineTracePublicationEquivocationWitnessViolation),
        PublicationInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        WitnessCoverageMismatch,
        WitnessSetCoverageMismatch,
        SetDigestMismatch,
        WitnessSetCollectionEmpty,
    }

    fn validate_state_machine_trace_publication_equivocation_witness_set(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
        witness_set: &FederationStateMachineTracePublicationEquivocationWitnessSet,
    ) -> Result<(), FederationStateMachineTracePublicationEquivocationWitnessSetViolation> {
        if snapshots.is_empty() {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessSetCollectionEmpty
            );
        }
        if witness_set.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedSchemaVersion
            );
        }
        if witness_set.set_profile
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_PROFILE
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedSetProfile
            );
        }
        if witness_set.hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedHashAlgorithm
            );
        }
        if witness_set.hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_EQUIVOCATION_WITNESS_SET_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedHashEncoding
            );
        }
        if witness_set.collection_schema_version
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedCollectionSchemaVersion
            );
        }
        if witness_set.collection_profile
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedCollectionProfile
            );
        }
        if witness_set.collection_hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedCollectionHashAlgorithm
            );
        }
        if witness_set.collection_hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::UnsupportedCollectionHashEncoding
            );
        }
        for (capsule, publication) in snapshots {
            validate_state_machine_trace_checkpoint_publication(capsule, publication)
                .map_err(
                    FederationStateMachineTracePublicationEquivocationWitnessSetViolation::PublicationInvalid,
                )?;
        }

        let (expected_collection_size, expected_collection_sha256) =
            state_machine_trace_publication_collection_commitment(snapshots)
                .map_err(
                    FederationStateMachineTracePublicationEquivocationWitnessSetViolation::PublicationInvalid,
                )?;
        if witness_set.collection_size != expected_collection_size {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::CollectionSizeMismatch
            );
        }
        if witness_set.collection_sha256 != expected_collection_sha256 {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::CollectionDigestMismatch
            );
        }

        if witness_set.witnesses.is_empty() {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::EmptyWitnessSet
            );
        }
        if witness_set
            .witnesses
            .windows(2)
            .any(|pair| pair[0].witness_sha256 == pair[1].witness_sha256)
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::DuplicateWitness
            );
        }
        if witness_set
            .witnesses
            .windows(2)
            .any(|pair| pair[0].witness_sha256 > pair[1].witness_sha256)
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessNotCanonical
            );
        }

        let mut publication_index = BTreeMap::new();
        for (capsule, publication) in snapshots {
            publication_index
                .entry(publication.publication_sha256.as_str())
                .or_insert((*capsule, *publication));
        }

        for witness in &witness_set.witnesses {
            let Some((first_capsule, first_publication)) =
                publication_index.get(witness.first_publication_sha256.as_str()).copied()
            else {
                return Err(
                    FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessCoverageMismatch
                );
            };
            let Some((second_capsule, second_publication)) =
                publication_index.get(witness.second_publication_sha256.as_str()).copied()
            else {
                return Err(
                    FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessCoverageMismatch
                );
            };
            validate_state_machine_trace_publication_equivocation_witness(
                first_capsule,
                first_publication,
                second_capsule,
                second_publication,
                witness,
            )
            .map_err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessInvalid,
            )?;
        }

        let expected =
            collect_state_machine_trace_publication_equivocation_witnesses(snapshots)
                .map_err(|_| {
                    FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessSetCoverageMismatch
                })?;
        if witness_set.witnesses != expected {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessSetCoverageMismatch
            );
        }
        if witness_set.set_sha256
            != state_machine_trace_publication_equivocation_witness_set_sha256(witness_set)
        {
            return Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::SetDigestMismatch
            );
        }

        Ok(())
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTracePublicationCollectionReconciliationViolation {
        UnsupportedSchemaVersion,
        UnsupportedReconciliationProfile,
        UnsupportedHashAlgorithm,
        UnsupportedHashEncoding,
        LeftCollectionEmpty,
        RightCollectionEmpty,
        LeftCollectionInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        RightCollectionInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        UnsupportedLeftCollectionSchemaVersion,
        UnsupportedLeftCollectionProfile,
        UnsupportedLeftCollectionHashAlgorithm,
        UnsupportedLeftCollectionHashEncoding,
        UnsupportedRightCollectionSchemaVersion,
        UnsupportedRightCollectionProfile,
        UnsupportedRightCollectionHashAlgorithm,
        UnsupportedRightCollectionHashEncoding,
        LeftCollectionSizeMismatch,
        LeftCollectionDigestMismatch,
        RightCollectionSizeMismatch,
        RightCollectionDigestMismatch,
        RelationshipMismatch,
        HistoryRelationshipMismatch,
        SharedPublicationSetMismatch,
        LeftOnlyPublicationSetMismatch,
        RightOnlyPublicationSetMismatch,
        EquivocationWitnessCoverageMismatch,
        UnionCollectionInvalid(FederationStateMachineTraceCheckpointPublicationViolation),
        ReconciliationDigestMismatch,
    }

    fn validate_state_machine_trace_publication_collection_reconciliation<'a>(
        left: &[(
            &'a FederationStateMachineTraceCapsule,
            &'a FederationStateMachineTraceCheckpointPublication,
        )],
        right: &[(
            &'a FederationStateMachineTraceCapsule,
            &'a FederationStateMachineTraceCheckpointPublication,
        )],
        receipt: &FederationStateMachineTracePublicationCollectionReconciliationReceipt,
    ) -> Result<(), FederationStateMachineTracePublicationCollectionReconciliationViolation> {
        if left.is_empty() {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::LeftCollectionEmpty
            );
        }
        if right.is_empty() {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RightCollectionEmpty
            );
        }
        if receipt.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedSchemaVersion
            );
        }
        if receipt.reconciliation_profile
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_PROFILE
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedReconciliationProfile
            );
        }
        if receipt.hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedHashAlgorithm
            );
        }
        if receipt.hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_RECONCILIATION_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedHashEncoding
            );
        }

        if receipt.left_collection_schema_version
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedLeftCollectionSchemaVersion
            );
        }
        if receipt.left_collection_profile
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedLeftCollectionProfile
            );
        }
        if receipt.left_collection_hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedLeftCollectionHashAlgorithm
            );
        }
        if receipt.left_collection_hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedLeftCollectionHashEncoding
            );
        }
        if receipt.right_collection_schema_version
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedRightCollectionSchemaVersion
            );
        }
        if receipt.right_collection_profile
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_PROFILE
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedRightCollectionProfile
            );
        }
        if receipt.right_collection_hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedRightCollectionHashAlgorithm
            );
        }
        if receipt.right_collection_hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_PUBLICATION_COLLECTION_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::UnsupportedRightCollectionHashEncoding
            );
        }

        let left_digests =
            state_machine_trace_publication_collection_digest_list(left).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::LeftCollectionInvalid,
            )?;
        let right_digests =
            state_machine_trace_publication_collection_digest_list(right).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RightCollectionInvalid,
            )?;

        let (left_size, left_sha256) =
            state_machine_trace_publication_collection_commitment(left).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::LeftCollectionInvalid,
            )?;
        let (right_size, right_sha256) =
            state_machine_trace_publication_collection_commitment(right).map_err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RightCollectionInvalid,
            )?;

        if receipt.left_collection_size != left_size {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::LeftCollectionSizeMismatch
            );
        }
        if receipt.left_collection_sha256 != left_sha256 {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::LeftCollectionDigestMismatch
            );
        }
        if receipt.right_collection_size != right_size {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RightCollectionSizeMismatch
            );
        }
        if receipt.right_collection_sha256 != right_sha256 {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RightCollectionDigestMismatch
            );
        }

        let (relationship, shared, left_only, right_only) =
            state_machine_trace_publication_collection_relationship(&left_digests, &right_digests);
        if receipt.relationship != relationship {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RelationshipMismatch
            );
        }
        let expected_history_relationship =
            state_machine_trace_publication_collection_history_relationship(left, right)
                .map_err(
                    FederationStateMachineTracePublicationCollectionReconciliationViolation::UnionCollectionInvalid,
                )?;
        if receipt.history_relationship != expected_history_relationship {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::HistoryRelationshipMismatch
            );
        }
        if receipt.shared_publication_sha256s != shared {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::SharedPublicationSetMismatch
            );
        }
        if receipt.left_only_publication_sha256s != left_only {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::LeftOnlyPublicationSetMismatch
            );
        }
        if receipt.right_only_publication_sha256s != right_only {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RightOnlyPublicationSetMismatch
            );
        }

        let mut union = Vec::with_capacity(left.len() + right.len());
        union.extend(left.iter().copied());
        union.extend(right.iter().copied());
        let expected_witnesses =
            collect_state_machine_trace_publication_equivocation_witnesses(&union)
                .map_err(
                    FederationStateMachineTracePublicationCollectionReconciliationViolation::UnionCollectionInvalid,
                )?
                .into_iter()
                .map(|witness| witness.witness_sha256)
                .collect::<Vec<_>>();
        if receipt.equivocation_witness_sha256s != expected_witnesses {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::EquivocationWitnessCoverageMismatch
            );
        }

        if receipt.reconciliation_sha256
            != state_machine_trace_publication_collection_reconciliation_sha256(receipt)
        {
            return Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::ReconciliationDigestMismatch
            );
        }

        Ok(())
    }

    /// Validates a complete, anchored publication lineage. Unlike the
    /// order-independent fork detector, this requires exactly one publication
    /// rooted at the dedicated genesis and requires every unique publication
    /// to be reachable from that root. This closes the disconnected-suffix case:
    /// individually valid publications are not sufficient to establish a complete
    /// lineage if some records are not connected to the declared root.
    fn validate_state_machine_trace_checkpoint_publication_lineage(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        validate_state_machine_trace_checkpoint_publication_set_against_snapshots(snapshots)?;

        let mut publications = BTreeMap::<
            &str,
            &FederationStateMachineTraceCheckpointPublication,
        >::new();
        for (_, publication) in snapshots {
            publications
                .entry(publication.publication_sha256.as_str())
                .or_insert(publication);
        }

        let first = publications
            .values()
            .next()
            .expect("non-empty qualified collection must have a publication");
        let identity = (
            first.publication_profile.as_str(),
            first.hash_algorithm.as_str(),
            first.schema_version,
            first.hash_encoding.as_str(),
            first.trace_verification_profile.as_str(),
            first.trace_schema_version,
            first.trace_index,
        );
        if publications.values().any(|publication| {
            (
                publication.publication_profile.as_str(),
                publication.hash_algorithm.as_str(),
                publication.schema_version,
                publication.hash_encoding.as_str(),
                publication.trace_verification_profile.as_str(),
                publication.trace_schema_version,
                publication.trace_index,
            ) != identity
        }) {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationLineageIdentityMismatch
            );
        }

        let roots = publications
            .values()
            .filter(|publication| {
                publication.previous_publication_sha256
                    == FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS
            })
            .collect::<Vec<_>>();
        if roots.is_empty() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationLineageNoRoot
            );
        }
        let mut successor_by_predecessor = BTreeMap::<
            &str,
            (
                &FederationStateMachineTraceCapsule,
                &FederationStateMachineTraceCheckpointPublication,
            ),
        >::new();
        for (publication_sha256, publication) in &publications {
            let snapshot = snapshots
                .iter()
                .find(|(_, candidate)| candidate.publication_sha256 == *publication_sha256)
                .expect("qualified publication must have a concrete snapshot");
            successor_by_predecessor.insert(
                publication.previous_publication_sha256.as_str(),
                (snapshot.0, *publication),
            );
        }

        let mut reachable = BTreeSet::new();
        let root_publication = roots[0];
        let mut current = root_publication.publication_sha256.as_str();
        let mut current_snapshot = snapshots
            .iter()
            .find(|(_, publication)| publication.publication_sha256 == current)
            .map(|(capsule, _)| *capsule)
            .expect("root publication must have a concrete snapshot");

        loop {
            if !reachable.insert(current) {
                return Err(
                    FederationStateMachineTraceCheckpointPublicationViolation::PublicationLineageCycle
                );
            }
            let Some((successor_snapshot, successor)) =
                successor_by_predecessor.get(current).copied()
            else {
                break;
            };
            validate_state_machine_trace_checkpoint_publication_consistency(
                current_snapshot,
                publications
                    .get(current)
                    .copied()
                    .expect("reachable publication must exist"),
                successor_snapshot,
                successor,
            )?;
            current = successor.publication_sha256.as_str();
            current_snapshot = successor_snapshot;
        }

        if reachable.len() != publications.len() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationLineageDisconnected
            );
        }

        Ok(())
    }

    fn validate_state_machine_trace_checkpoint_publication(
        capsule: &FederationStateMachineTraceCapsule,
        publication: &FederationStateMachineTraceCheckpointPublication,
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        if publication.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedSchemaVersion
            );
        }
        if publication.publication_profile
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_PROFILE
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedPublicationProfile
            );
        }
        if publication.hash_algorithm
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_ALGORITHM
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedPublicationHashAlgorithm
            );
        }
        if publication.hash_encoding
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_ENCODING
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedPublicationHashEncoding
            );
        }
        if publication.trace_schema_version != capsule.schema_version {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::TraceSchemaVersionMismatch
            );
        }
        if publication.trace_verification_profile != capsule.verification_profile {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::TraceVerificationProfileMismatch
            );
        }
        if capsule.schema_version != FEDERATION_STATE_MACHINE_TRACE_CAPSULE_SCHEMA_VERSION {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedTraceSchemaVersion
            );
        }
        if publication.trace_index != capsule.trace_index {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::TraceIndexMismatch
            );
        }
        if publication.evidence_end > capsule.evidence.len() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::InvalidEvidenceEnd
            );
        }
        if publication.body_sha256 != capsule.integrity.body_sha256 {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::BodyDigestMismatch
            );
        }
        let expected_chain_head = if publication.evidence_end == 0 {
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS
        } else {
            capsule.evidence[publication.evidence_end - 1]
                .chain_sha256
                .as_str()
        };
        if publication.chain_head_sha256 != expected_chain_head {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ChainHeadMismatch
            );
        }
        if publication.publication_sha256
            != state_machine_trace_checkpoint_publication_sha256(publication)
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationDigestMismatch
            );
        }
        validate_state_machine_trace_evidence(capsule)
            .map_err(FederationStateMachineTraceCheckpointPublicationViolation::TraceEvidenceInvalid)?;
        Ok(())
    }

    fn validate_state_machine_trace_checkpoint_publication_consistency(
        earlier_capsule: &FederationStateMachineTraceCapsule,
        earlier_publication: &FederationStateMachineTraceCheckpointPublication,
        later_capsule: &FederationStateMachineTraceCapsule,
        later_publication: &FederationStateMachineTraceCheckpointPublication,
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        validate_state_machine_trace_checkpoint_publication(
            earlier_capsule,
            earlier_publication,
        )?;
        validate_state_machine_trace_checkpoint_publication(
            later_capsule,
            later_publication,
        )?;

        if later_publication.publication_profile != earlier_publication.publication_profile {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotProfileMismatch
            );
        }
        if later_publication.trace_index != earlier_publication.trace_index {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotTraceIndexMismatch
            );
        }
        if earlier_publication.evidence_end > later_publication.evidence_end {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotRollback
            );
        }
        if earlier_publication.evidence_end == later_publication.evidence_end
            && (earlier_publication.body_sha256 != later_publication.body_sha256
                || earlier_publication.chain_head_sha256 != later_publication.chain_head_sha256)
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SameEndpointSnapshotMismatch
            );
        }
        if later_publication.previous_publication_sha256
            != earlier_publication.publication_sha256
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PreviousPublicationMismatch
            );
        }

        if earlier_capsule.initial_seed != later_capsule.initial_seed
            || earlier_capsule.initial_state != later_capsule.initial_state
            || earlier_capsule.verification_profile != later_capsule.verification_profile
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotPrefixMismatch
            );
        }

        let end = earlier_publication.evidence_end;
        if later_capsule.operations.get(..end)
            != earlier_capsule.operations.get(..end)
            || later_capsule.tokens.get(..end) != earlier_capsule.tokens.get(..end)
            || later_capsule.evidence.get(..end) != earlier_capsule.evidence.get(..end)
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotPrefixMismatch
            );
        }

        Ok(())
    }

    /// Validates an ordered publication sequence, including the otherwise
    /// unobservable root of the publication predecessor chain.
    ///
    /// Adjacent pairs prove snapshot prefix consistency; this wrapper additionally
    /// requires the first publication to start from the publication genesis marker.
    fn validate_state_machine_trace_checkpoint_publication_chain(
        snapshots: &[(
            &FederationStateMachineTraceCapsule,
            &FederationStateMachineTraceCheckpointPublication,
        )],
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        if snapshots.is_empty() {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationChainEmpty
            );
        }

        for (capsule, publication) in snapshots {
            validate_state_machine_trace_checkpoint_publication(capsule, publication)?;
        }

        if snapshots[0].1.previous_publication_sha256
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS
        {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::FirstPublicationPredecessorMismatch
            );
        }

        for pair in snapshots.windows(2) {
            validate_state_machine_trace_checkpoint_publication_consistency(
                pair[0].0,
                pair[0].1,
                pair[1].0,
                pair[1].1,
            )?;
        }

        Ok(())
    }

    fn state_machine_trace_checkpoint_sha256(
        checkpoint: &FederationStateMachineTraceCheckpoint,
    ) -> String {
        let view = FederationStateMachineTraceCheckpointHashView {
            hash_domain: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_HASH_DOMAIN.into(),
            schema_version: checkpoint.schema_version,
            checkpoint_profile: checkpoint.checkpoint_profile.clone(),
            verification_profile: checkpoint.verification_profile.clone(),
            trace_index: checkpoint.trace_index,
            step_start: checkpoint.step_start,
            step_end: checkpoint.step_end,
            body_sha256: checkpoint.body_sha256.clone(),
            chain_start_prev_sha256: checkpoint.chain_start_prev_sha256.clone(),
            chain_head_sha256: checkpoint.chain_head_sha256.clone(),
            previous_checkpoint_sha256: checkpoint.previous_checkpoint_sha256.clone(),
        };
        let bytes = serde_json::to_vec(&view)
            .expect("trace checkpoint hash view must be serializable");
        state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_HASH_DOMAIN,
            &bytes,
        )
    }

    fn state_machine_trace_checkpoint(
        capsule: &FederationStateMachineTraceCapsule,
        step_start: usize,
        step_end: usize,
        previous_checkpoint_sha256: &str,
    ) -> FederationStateMachineTraceCheckpoint {
        assert!(
            step_start <= step_end && step_end <= capsule.evidence.len(),
            "trace checkpoint range must be within the trace evidence"
        );
        let chain_start_prev_sha256 = if step_start == 0 {
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.to_owned()
        } else {
            capsule.evidence[step_start - 1].chain_sha256.clone()
        };
        let chain_head_sha256 = if step_start == step_end {
            chain_start_prev_sha256.clone()
        } else {
            capsule.evidence[step_end - 1].chain_sha256.clone()
        };
        let mut checkpoint = FederationStateMachineTraceCheckpoint {
            schema_version: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_SCHEMA_VERSION,
            checkpoint_profile: FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PROFILE.into(),
            verification_profile: capsule.verification_profile.clone(),
            trace_index: capsule.trace_index,
            step_start,
            step_end,
            body_sha256: capsule.integrity.body_sha256.clone(),
            chain_start_prev_sha256,
            chain_head_sha256,
            previous_checkpoint_sha256: previous_checkpoint_sha256.to_owned(),
            checkpoint_sha256: String::new(),
        };
        checkpoint.checkpoint_sha256 =
            state_machine_trace_checkpoint_sha256(&checkpoint);
        checkpoint
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceCheckpointViolation {
        UnsupportedSchemaVersion,
        UnsupportedCheckpointProfile,
        UnsupportedVerificationProfile,
        TraceIndexMismatch,
        InvalidStepRange,
        EmptyNonTerminalSegment,
        BodyDigestMismatch,
        ChainStartMismatch,
        ChainHeadMismatch,
        CheckpointDigestMismatch,
        FirstCheckpointPredecessorMismatch,
        CheckpointRangeGap,
        CheckpointChainStartMismatch,
        PreviousCheckpointMismatch,
        CoverageIncomplete,
    }

    fn validate_state_machine_trace_checkpoint(
        capsule: &FederationStateMachineTraceCapsule,
        checkpoint: &FederationStateMachineTraceCheckpoint,
    ) -> Result<(), FederationStateMachineTraceCheckpointViolation> {
        if checkpoint.schema_version
            != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_SCHEMA_VERSION
        {
            return Err(
                FederationStateMachineTraceCheckpointViolation::UnsupportedSchemaVersion
            );
        }
        if checkpoint.checkpoint_profile != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PROFILE {
            return Err(
                FederationStateMachineTraceCheckpointViolation::UnsupportedCheckpointProfile
            );
        }
        if checkpoint.verification_profile
            != capsule.verification_profile
        {
            return Err(
                FederationStateMachineTraceCheckpointViolation::UnsupportedVerificationProfile
            );
        }
        if checkpoint.trace_index != capsule.trace_index {
            return Err(
                FederationStateMachineTraceCheckpointViolation::TraceIndexMismatch
            );
        }
        if checkpoint.step_start > checkpoint.step_end
            || checkpoint.step_end > capsule.evidence.len()
        {
            return Err(
                FederationStateMachineTraceCheckpointViolation::InvalidStepRange
            );
        }
        if checkpoint.step_start == checkpoint.step_end && !capsule.evidence.is_empty() {
            return Err(
                FederationStateMachineTraceCheckpointViolation::EmptyNonTerminalSegment
            );
        }
        if checkpoint.body_sha256 != capsule.integrity.body_sha256 {
            return Err(
                FederationStateMachineTraceCheckpointViolation::BodyDigestMismatch
            );
        }
        let expected_chain_start = if checkpoint.step_start == 0 {
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS
        } else {
            capsule.evidence[checkpoint.step_start - 1]
                .chain_sha256
                .as_str()
        };
        if checkpoint.chain_start_prev_sha256 != expected_chain_start {
            return Err(
                FederationStateMachineTraceCheckpointViolation::ChainStartMismatch
            );
        }
        let expected_chain_head = if checkpoint.step_start == checkpoint.step_end {
            expected_chain_start
        } else {
            capsule.evidence[checkpoint.step_end - 1]
                .chain_sha256
                .as_str()
        };
        if checkpoint.chain_head_sha256 != expected_chain_head {
            return Err(
                FederationStateMachineTraceCheckpointViolation::ChainHeadMismatch
            );
        }
        if checkpoint.checkpoint_sha256 != state_machine_trace_checkpoint_sha256(checkpoint) {
            return Err(
                FederationStateMachineTraceCheckpointViolation::CheckpointDigestMismatch
            );
        }
        Ok(())
    }

    fn validate_state_machine_trace_checkpoint_chain(
        capsule: &FederationStateMachineTraceCapsule,
        checkpoints: &[FederationStateMachineTraceCheckpoint],
    ) -> Result<(), FederationStateMachineTraceCheckpointViolation> {
        if checkpoints.is_empty() {
            return Err(FederationStateMachineTraceCheckpointViolation::CoverageIncomplete);
        }

        for checkpoint in checkpoints {
            validate_state_machine_trace_checkpoint(capsule, checkpoint)?;
        }

        if checkpoints[0].step_start != 0
            || checkpoints[0].previous_checkpoint_sha256
                != FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS
        {
            return Err(
                FederationStateMachineTraceCheckpointViolation::FirstCheckpointPredecessorMismatch
            );
        }

        for pair in checkpoints.windows(2) {
            if pair[1].step_start != pair[0].step_end {
                return Err(FederationStateMachineTraceCheckpointViolation::CheckpointRangeGap);
            }
            if pair[1].chain_start_prev_sha256 != pair[0].chain_head_sha256 {
                return Err(
                    FederationStateMachineTraceCheckpointViolation::CheckpointChainStartMismatch
                );
            }
            if pair[1].previous_checkpoint_sha256 != pair[0].checkpoint_sha256 {
                return Err(
                    FederationStateMachineTraceCheckpointViolation::PreviousCheckpointMismatch
                );
            }
        }

        if checkpoints.last().unwrap().step_end != capsule.evidence.len() {
            return Err(FederationStateMachineTraceCheckpointViolation::CoverageIncomplete);
        }

        Ok(())
    }

    fn reseal_state_machine_trace_for_test(capsule: &mut FederationStateMachineTraceCapsule) {
        let mut previous_chain_sha256 =
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.to_string();

        for evidence in &mut capsule.evidence {
            evidence.chain_prev_sha256 = previous_chain_sha256.clone();
            evidence.chain_sha256 = String::new();
            evidence.chain_sha256 = state_machine_evidence_chain_sha256(evidence);
            previous_chain_sha256 = evidence.chain_sha256.clone();
        }

        capsule.integrity = state_machine_trace_integrity(capsule);
    }


    fn state_machine_next_seed(seed: &mut u64) -> u64 {
        *seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        *seed
    }

    fn state_machine_trace_plan(
        trace_index: usize,
        steps: usize,
    ) -> (u64, Vec<(FederationStateMachineOperation, u64)>) {
        let initial_seed = 0xD6E5_5EED_u64 ^ trace_index as u64;
        let mut seed = initial_seed;
        let mut plan = Vec::with_capacity(steps);

        for step_index in 0..steps {
            let next_seed = state_machine_next_seed(&mut seed);
            let selector = (next_seed >> 60) as usize;
            let index = (trace_index
                .wrapping_mul(FederationStateMachineOperation::ALL.len())
                .wrapping_add(step_index)
                .wrapping_add(selector))
                % FederationStateMachineOperation::ALL.len();
            let operation = FederationStateMachineOperation::ALL[index];
            let token = next_seed ^ ((trace_index as u64) << 32) ^ step_index as u64;
            plan.push((operation, token));
        }

        (initial_seed, plan)
    }

    fn state_machine_trace_capsule(trace_index: usize, steps: usize) -> String {
        let (initial_seed, plan) = state_machine_trace_plan(trace_index, steps);
        let evidence = run_state_machine_trace_plan(&plan);
        let initial_state = FederationStateMachineTraceBoundary {
            admission_index: 0,
            delivery_count: 0,
            state_fingerprint: canonical_state_fingerprint(&nodes()),
        };
        let final_state = evidence
            .last()
            .map(|step| FederationStateMachineTraceBoundary {
                admission_index: step.post_admission_index,
                delivery_count: step.post_delivery_count,
                state_fingerprint: step.state_fingerprint.clone(),
            })
            .unwrap_or_else(|| initial_state.clone());

        let mut capsule = FederationStateMachineTraceCapsule {
            schema_version: FEDERATION_STATE_MACHINE_TRACE_CAPSULE_SCHEMA_VERSION,
            verification_profile: FEDERATION_STATE_MACHINE_TRACE_VERIFICATION_PROFILE.into(),
            trace_index,
            initial_seed,
            operations: plan.iter().map(|(operation, _)| *operation).collect(),
            tokens: plan.iter().map(|(_, token)| *token).collect(),
            initial_state,
            evidence,
            final_state,
            integrity: FederationStateMachineTraceIntegrity {
                algorithm: FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ALGORITHM.into(),
                encoding: FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ENCODING.into(),
                body_sha256: String::new(),
                chain_head_sha256: FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.into(),
            },
        };
        capsule.integrity = state_machine_trace_integrity(&capsule);
        serde_json::to_string_pretty(&capsule).expect("trace capsule is serializable")
    }

    /// Stable diagnostic classifications for replay-independent trace-evidence verification.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    enum FederationStateMachineTraceEvidenceViolation {
        UnsupportedSchemaVersion,
        UnsupportedVerificationProfile,
        UnsupportedIntegrityAlgorithm,
        UnsupportedIntegrityEncoding,
        BodyDigestMismatch,
        ChainHeadMismatch,
        PlanLengthMismatch,
        InitialStateMismatch,
        InitialSeedMismatch,
        CanonicalPlanMismatch,
        EvidenceLengthMismatch,
        StepIndexMismatch,
        OperationMismatch,
        TokenMismatch,
        PreStateContinuityMismatch,
        PreAdmissionContinuityMismatch,
        PreDeliveryContinuityMismatch,
        ChainContinuityMismatch,
        StepChainDigestMismatch,
        AdmissionRegression,
        DeliveryRegression,
        AdmissionDeliveryDeltaMismatch,
        AdmissionEnumerationLengthMismatch,
        AdmissionOrdinalRangeMismatch,
        DuplicateAdmittedDeliveryIdentity,
        EmptyAdmittedDeliveryIdentity,
        FinalStateMismatch,
    }

    /// Validates the persisted trace capsule without executing any state-machine
    /// transition. This verifies the artifact's schema, canonical plan descriptor,
    /// state/temporal boundary continuity, and hash-chain/body integrity.
    ///
    /// This is intentionally weaker than replay verification: a consumer can establish
    /// that the artifact is internally self-consistent without establishing that the
    /// recorded decisions were produced by this transition implementation. Replay remains
    /// the model-coupled semantic oracle.
    fn validate_state_machine_trace_evidence(
        capsule: &FederationStateMachineTraceCapsule,
    ) -> Result<(), FederationStateMachineTraceEvidenceViolation> {
        macro_rules! require {
            ($condition:expr, $violation:expr) => {
                if !$condition {
                    return Err($violation);
                }
            };
        }

        require!(
            capsule.schema_version == FEDERATION_STATE_MACHINE_TRACE_CAPSULE_SCHEMA_VERSION,
                FederationStateMachineTraceEvidenceViolation::UnsupportedSchemaVersion
        );
        require!(
            capsule.verification_profile
                == FEDERATION_STATE_MACHINE_TRACE_VERIFICATION_PROFILE,
                FederationStateMachineTraceEvidenceViolation::UnsupportedVerificationProfile
        );
        require!(
            capsule.integrity.algorithm
                == FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ALGORITHM,
                FederationStateMachineTraceEvidenceViolation::UnsupportedIntegrityAlgorithm
        );
        require!(
            capsule.integrity.encoding
                == FEDERATION_STATE_MACHINE_TRACE_CAPSULE_HASH_ENCODING,
                FederationStateMachineTraceEvidenceViolation::UnsupportedIntegrityEncoding
        );
        require!(
            capsule.integrity.body_sha256
                == state_machine_trace_body_sha256(capsule),
                FederationStateMachineTraceEvidenceViolation::BodyDigestMismatch
        );
        let expected_chain_head = capsule
            .evidence
            .last()
            .map(|evidence| evidence.chain_sha256.as_str())
            .unwrap_or(FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS);
        require!(
            capsule.integrity.chain_head_sha256 == expected_chain_head,
                FederationStateMachineTraceEvidenceViolation::ChainHeadMismatch
        );
        require!(
            capsule.operations.len() == capsule.tokens.len(),
                FederationStateMachineTraceEvidenceViolation::PlanLengthMismatch
        );
        require!(
            capsule.initial_state
                == FederationStateMachineTraceBoundary {
                    admission_index: 0,
                    delivery_count: 0,
                    state_fingerprint: canonical_state_fingerprint(&nodes()),
                },
                FederationStateMachineTraceEvidenceViolation::InitialStateMismatch
        );

        let expected_seed = 0xD6E5_5EED_u64 ^ capsule.trace_index as u64;
        require!(
            capsule.initial_seed == expected_seed,
                FederationStateMachineTraceEvidenceViolation::InitialSeedMismatch
        );

        let (_, canonical_plan) =
            state_machine_trace_plan(capsule.trace_index, capsule.operations.len());
        let recorded_plan = capsule
            .operations
            .iter()
            .copied()
            .zip(capsule.tokens.iter().copied())
            .collect::<Vec<_>>();
        require!(
            recorded_plan == canonical_plan,
                FederationStateMachineTraceEvidenceViolation::CanonicalPlanMismatch
        );
        require!(
            capsule.evidence.len() == recorded_plan.len(),
                FederationStateMachineTraceEvidenceViolation::EvidenceLengthMismatch
        );

        let mut expected_pre_state_fingerprint =
            capsule.initial_state.state_fingerprint.clone();
        let mut expected_post_boundary = capsule.initial_state.clone();
        let mut expected_chain_prev_sha256 =
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.to_string();
        let mut admitted_delivery_identities = BTreeSet::new();

        for (step_index, ((operation, token), evidence)) in
            recorded_plan.iter().zip(&capsule.evidence).enumerate()
        {
            require!(
                evidence.step_index == step_index,
                FederationStateMachineTraceEvidenceViolation::StepIndexMismatch
            );
            require!(
                evidence.operation == *operation,
                FederationStateMachineTraceEvidenceViolation::OperationMismatch
            );
            require!(
                evidence.token == *token,
                FederationStateMachineTraceEvidenceViolation::TokenMismatch
            );
            require!(
                evidence.pre_state_fingerprint == expected_pre_state_fingerprint,
                FederationStateMachineTraceEvidenceViolation::PreStateContinuityMismatch
            );
            require!(
                evidence.pre_admission_index == expected_post_boundary.admission_index,
                FederationStateMachineTraceEvidenceViolation::PreAdmissionContinuityMismatch
            );
            require!(
                evidence.pre_delivery_count == expected_post_boundary.delivery_count,
                FederationStateMachineTraceEvidenceViolation::PreDeliveryContinuityMismatch
            );
            require!(
                evidence.chain_prev_sha256 == expected_chain_prev_sha256,
                FederationStateMachineTraceEvidenceViolation::ChainContinuityMismatch
            );
            require!(
                evidence.chain_sha256 == state_machine_evidence_chain_sha256(evidence),
                FederationStateMachineTraceEvidenceViolation::StepChainDigestMismatch
            );
            require!(
                evidence.post_admission_index >= evidence.pre_admission_index,
                FederationStateMachineTraceEvidenceViolation::AdmissionRegression
            );
            require!(
                evidence.post_delivery_count >= evidence.pre_delivery_count,
                FederationStateMachineTraceEvidenceViolation::DeliveryRegression
            );
            let admission_delta =
                evidence.post_admission_index - evidence.pre_admission_index;
            let delivery_delta =
                (evidence.post_delivery_count - evidence.pre_delivery_count) as u64;
            require!(
                admission_delta == delivery_delta,
                FederationStateMachineTraceEvidenceViolation::AdmissionDeliveryDeltaMismatch
            );
            require!(
                evidence.newly_admitted_deliveries.len() as u64 == admission_delta,
                FederationStateMachineTraceEvidenceViolation::AdmissionEnumerationLengthMismatch
            );
            require!(
                evidence
                    .newly_admitted_deliveries
                    .iter()
                    .map(|(_, index)| *index)
                    .collect::<BTreeSet<_>>()
                    == (evidence.pre_admission_index..evidence.post_admission_index)
                        .collect::<BTreeSet<_>>(),
                FederationStateMachineTraceEvidenceViolation::AdmissionOrdinalRangeMismatch
            );

            for (delivery_id, _) in &evidence.newly_admitted_deliveries {
                require!(
                    !delivery_id.is_empty(),
                    FederationStateMachineTraceEvidenceViolation::EmptyAdmittedDeliveryIdentity
                );
                require!(
                    admitted_delivery_identities.insert(delivery_id.clone()),
                    FederationStateMachineTraceEvidenceViolation::DuplicateAdmittedDeliveryIdentity
                );
            }

            expected_post_boundary = FederationStateMachineTraceBoundary {
                admission_index: evidence.post_admission_index,
                delivery_count: evidence.post_delivery_count,
                state_fingerprint: evidence.state_fingerprint.clone(),
            };
            expected_pre_state_fingerprint = evidence.state_fingerprint.clone();
            expected_chain_prev_sha256 = evidence.chain_sha256.clone();
        }

        require!(
            capsule.final_state == expected_post_boundary,
            FederationStateMachineTraceEvidenceViolation::FinalStateMismatch
        );

        Ok(())
    }

    fn state_machine_plan_from_capsule(
        capsule: &FederationStateMachineTraceCapsule,
    ) -> Vec<(FederationStateMachineOperation, u64)> {
        validate_state_machine_trace_evidence(capsule)
            .unwrap_or_else(|violation| panic!("trace evidence verification failed: {violation:?}"));

        let recorded_plan = capsule
            .operations
            .iter()
            .copied()
            .zip(capsule.tokens.iter().copied())
            .collect::<Vec<_>>();

        let replayed_evidence = run_state_machine_trace_plan(&recorded_plan);
        assert_eq!(
            capsule.evidence,
            replayed_evidence,
            "successful trace capsule evidence must exactly match canonical replay"
        );

        recorded_plan
    }

    fn shrink_failing_state_machine_plan<F>(
        plan: &[(FederationStateMachineOperation, u64)],
        mut fails: F,
    ) -> Vec<(FederationStateMachineOperation, u64)>
    where
        F: FnMut(&[(FederationStateMachineOperation, u64)]) -> bool,
    {
        assert!(
            fails(plan),
            "sequence shrinker requires an initially failing plan"
        );

        let mut current = plan.to_vec();
        let mut granularity = 2usize;

        while current.len() >= 2 {
            let chunk_size = current.len().div_ceil(granularity);
            let mut reduced = false;
            let mut start = 0usize;

            while start < current.len() {
                let end = (start + chunk_size).min(current.len());
                let mut candidate = Vec::with_capacity(current.len() - (end - start));
                candidate.extend_from_slice(&current[..start]);
                candidate.extend_from_slice(&current[end..]);

                if !candidate.is_empty() && fails(&candidate) {
                    current = candidate;
                    granularity = granularity.saturating_sub(1).max(2);
                    reduced = true;
                    break;
                }

                start = end;
            }

            if !reduced {
                if granularity >= current.len() {
                    break;
                }
                granularity = (granularity * 2).min(current.len());
            }
        }

        current
    }

    fn state_machine_token_shrink_candidates(token: u64) -> BTreeSet<u64> {
        let mut candidates = BTreeSet::from([
            0,
            1,
            2,
            3,
            4,
            8,
            16,
            32,
            64,
            128,
            token.saturating_sub(1),
            token / 2,
            token / 4,
            token & 0xff,
            token % 1024,
        ]);
        candidates.remove(&token);
        candidates
    }

    fn shrink_failing_state_machine_tokens<F>(
        plan: &[(FederationStateMachineOperation, u64)],
        mut fails: F,
    ) -> Vec<(FederationStateMachineOperation, u64)>
    where
        F: FnMut(&[(FederationStateMachineOperation, u64)]) -> bool,
    {
        assert!(
            fails(plan),
            "parameter shrinker requires an initially failing plan"
        );

        let mut current = plan.to_vec();

        loop {
            let mut changed = false;

            for index in 0..current.len() {
                let current_token = current[index].1;

                for candidate_token in state_machine_token_shrink_candidates(current_token) {
                    if candidate_token >= current_token {
                        continue;
                    }

                    let mut candidate = current.clone();
                    candidate[index].1 = candidate_token;
                    if fails(&candidate) {
                        current = candidate;
                        changed = true;
                        break;
                    }
                }
            }

            if changed {
                continue;
            }

            let distinct_tokens = current
                .iter()
                .map(|(_, token)| *token)
                .collect::<BTreeSet<_>>();

            'linked: for current_token in distinct_tokens {
                for candidate_token in state_machine_token_shrink_candidates(current_token) {
                    if candidate_token >= current_token {
                        continue;
                    }

                    let candidate = current
                        .iter()
                        .map(|(operation, token)| {
                            (
                                *operation,
                                if *token == current_token {
                                    candidate_token
                                } else {
                                    *token
                                },
                            )
                        })
                        .collect::<Vec<_>>();

                    if fails(&candidate) {
                        current = candidate;
                        changed = true;
                        break 'linked;
                    }
                }
            }

            if !changed {
                break;
            }
        }

        assert!(
            fails(&current),
            "parameter shrinker must preserve the failure predicate"
        );
        current
    }

    fn state_machine_envelope(
        operation: FederationStateMachineOperation,
        token: u64,
    ) -> FederationEnvelope {
        FederationEnvelope {
            envelope_id: format!("sm-env-{operation:?}-{token}"),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: format!("sha256:sm-{token}"),
            logical_delivery_id: format!("sm-delivery-{operation:?}-{token}"),
            attempt_id: format!("sm-attempt-{operation:?}-{token}"),
            origin_node: "node-a".into(),
            target_node: "node-a".into(),
            schema_generation: 1,
            authorization_generation: 1,
            authorization: AuthorizationState::Active,
            expires_at: Some(1_000_000),
            predecessor_delivery_id: None,
        }
    }

    #[derive(Debug)]
    struct FederationStateMachineInvariantFailure(
        FederationStateMachineFailureCapsule,
    );

    fn panic_invariant_failure(
        capsule: FederationStateMachineFailureCapsule,
    ) -> ! {
        std::panic::panic_any(FederationStateMachineInvariantFailure(capsule))
    }

    fn invariant_failure_from_plan(
        plan: &[(FederationStateMachineOperation, u64)],
        trace_index: usize,
    ) -> Option<FederationStateMachineFailureCapsule> {
        let payload = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_state_machine_trace_plan_core(plan, trace_index, false)
        }))
        .err()?;

        payload
            .downcast::<FederationStateMachineInvariantFailure>()
            .ok()
            .map(|failure| failure.0)
    }

    fn run_state_machine_trace(
        trace_index: usize,
        steps: usize,
    ) -> Vec<FederationStateMachineEvidence> {
        let (_, plan) = state_machine_trace_plan(trace_index, steps);
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_state_machine_trace_plan_core(&plan, trace_index, true)
        })) {
            Ok(evidence) => evidence,
            Err(payload) => match payload.downcast::<FederationStateMachineInvariantFailure>() {
                Ok(failure) => panic!("{}", failure.0.to_json()),
                Err(payload) => std::panic::resume_unwind(payload),
            },
        }
    }

    fn run_state_machine_trace_plan(
        plan: &[(FederationStateMachineOperation, u64)],
    ) -> Vec<FederationStateMachineEvidence> {
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_state_machine_trace_plan_core(plan, 0, true)
        })) {
            Ok(evidence) => evidence,
            Err(payload) => match payload.downcast::<FederationStateMachineInvariantFailure>() {
                Ok(failure) => panic!("{}", failure.0.to_json()),
                Err(payload) => std::panic::resume_unwind(payload),
            },
        }
    }

    fn run_state_machine_trace_plan_core(
        plan: &[(FederationStateMachineOperation, u64)],
        trace_index: usize,
        shrink_on_failure: bool,
    ) -> Vec<FederationStateMachineEvidence> {
        let mut state = nodes();
        let mut admitted = Vec::<FederationEnvelope>::new();
        let mut evidence = Vec::with_capacity(plan.len());
        let mut previous_chain_sha256 =
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.to_string();

        for (step_index, (operation, token)) in plan.iter().copied().enumerate() {
            let before = canonical_state_fingerprint(&state);
            let before_admission_index = state.next_admission_index;
            let before_delivery_count = state.delivery_count();
            let before_delivery_ids = state.deliveries.keys().cloned().collect::<BTreeSet<_>>();
            let mut decision = None;
            let mut authority = None;

            match operation {
                FederationStateMachineOperation::AddRecognition => {
                    let inserted = state
                        .try_add_recognition(RecognitionEdge {
                            recognizing_node: "node-a".into(),
                            origin_node: "node-b".into(),
                            scope: format!("sm-recognition-{token}"),
                            mode: RecognitionMode::EvidenceOnly,
                        })
                        .unwrap();
                    assert!(inserted);
                }
                FederationStateMachineOperation::DuplicateRecognition => {
                    let edge = RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "node-b".into(),
                        scope: "sm-recognition-duplicate".into(),
                        mode: RecognitionMode::EvidenceOnly,
                    };
                    let _ = state.try_add_recognition(edge.clone()).unwrap();
                    let before_duplicate = canonical_state_fingerprint(&state);
                    assert_eq!(state.try_add_recognition(edge), Ok(false));
                    assert_eq!(canonical_state_fingerprint(&state), before_duplicate);
                }
                FederationStateMachineOperation::RecordObservation => {
                    let observation = ObservationRecord {
                        observation_id: format!("sm-observation-{token}"),
                        semantic_subject_id: "subject-1".into(),
                        payload_commitment: format!("sha256:sm-observation-{token}"),
                        origin_node: "node-a".into(),
                        origin_node_known: false,
                        recognized_by: None,
                        source_observation: false,
                    };
                    assert_eq!(
                        record_observation(&mut state, observation),
                        ObservationWriteResult::Inserted
                    );
                }
                FederationStateMachineOperation::DuplicateObservation => {
                    let observation = ObservationRecord {
                        observation_id: "sm-observation-duplicate".into(),
                        semantic_subject_id: "subject-1".into(),
                        payload_commitment: "sha256:sm-observation-duplicate".into(),
                        origin_node: "node-a".into(),
                        origin_node_known: false,
                        recognized_by: None,
                        source_observation: false,
                    };
                    let _ = record_observation(&mut state, observation.clone());
                    let before_duplicate = canonical_state_fingerprint(&state);
                    assert_eq!(
                        record_observation(&mut state, observation),
                        ObservationWriteResult::Duplicate
                    );
                    assert_eq!(canonical_state_fingerprint(&state), before_duplicate);
                }
                FederationStateMachineOperation::ConflictObservation => {
                    let observation = ObservationRecord {
                        observation_id: "sm-observation-conflict".into(),
                        semantic_subject_id: "subject-1".into(),
                        payload_commitment: "sha256:sm-observation".into(),
                        origin_node: "node-a".into(),
                        origin_node_known: false,
                        recognized_by: None,
                        source_observation: false,
                    };
                    let _ = record_observation(&mut state, observation.clone());
                    let before_conflict = canonical_state_fingerprint(&state);
                    let mut conflict = observation;
                    conflict.payload_commitment = "sha256:sm-conflict".into();
                    assert_eq!(
                        record_observation(&mut state, conflict),
                        ObservationWriteResult::Conflict
                    );
                    assert_eq!(canonical_state_fingerprint(&state), before_conflict);
                }
                FederationStateMachineOperation::AdmitLocal => {
                    let candidate = state_machine_envelope(operation, token);
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::AcceptedLocal);
                    admitted.push(candidate);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                }
                FederationStateMachineOperation::ReplayExact => {
                    if admitted.is_empty() {
                        let candidate = state_machine_envelope(
                            FederationStateMachineOperation::AdmitLocal,
                            token.wrapping_add(1),
                        );
                        assert_eq!(
                            deliver(&mut state, &candidate, 100, true).decision(),
                            FederationDecision::AcceptedLocal
                        );
                        admitted.push(candidate);
                    }
                    let candidate = admitted.first().cloned().unwrap();
                    let before_replay = canonical_state_fingerprint(&state);
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::Duplicate);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before_replay);
                }
                FederationStateMachineOperation::RetryExisting => {
                    if admitted.is_empty() {
                        let candidate = state_machine_envelope(
                            FederationStateMachineOperation::AdmitLocal,
                            token.wrapping_add(1),
                        );
                        assert_eq!(
                            deliver(&mut state, &candidate, 100, true).decision(),
                            FederationDecision::AcceptedLocal
                        );
                        admitted.push(candidate);
                    }
                    let mut candidate = admitted.first().cloned().unwrap();
                    candidate.attempt_id = format!("sm-retry-{token}");
                    candidate.envelope_id = format!("sm-retry-envelope-{token}");
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::Duplicate);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                }
                FederationStateMachineOperation::RebindAttempt => {
                    if admitted.is_empty() {
                        let candidate = state_machine_envelope(
                            FederationStateMachineOperation::AdmitLocal,
                            token.wrapping_add(1),
                        );
                        assert_eq!(
                            deliver(&mut state, &candidate, 100, true).decision(),
                            FederationDecision::AcceptedLocal
                        );
                        admitted.push(candidate);
                    }
                    let candidate = admitted.first().cloned().unwrap();
                    let mut rebound = candidate.clone();
                    rebound.envelope_id = format!("sm-rebound-envelope-{token}");
                    let before_rebind = canonical_state_fingerprint(&state);
                    let outcome = deliver(&mut state, &rebound, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::AttemptConflict);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before_rebind);
                }
                FederationStateMachineOperation::AdmitForeign => {
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.origin_node = "node-b".into();
                    candidate.target_node = "node-a".into();
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::AcceptedForeign);
                    admitted.push(candidate);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                }
                FederationStateMachineOperation::AdmitRecognizedForeign => {
                    let _ = state.try_add_recognition(RecognitionEdge {
                        recognizing_node: "node-a".into(),
                        origin_node: "node-b".into(),
                        scope: "subject-1".into(),
                        mode: RecognitionMode::EvidenceOnly,
                    });
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.origin_node = "node-b".into();
                    candidate.target_node = "node-a".into();
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(
                        outcome.decision(),
                        FederationDecision::AcceptedForeign
                    );
                    admitted.push(candidate);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                }
                FederationStateMachineOperation::PendingDependency => {
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.predecessor_delivery_id =
                        Some(format!("sm-missing-{token}"));
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::PendingDependency);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before);
                }
                FederationStateMachineOperation::AdmitWithPredecessor => {
                    if admitted.is_empty() {
                        let predecessor = state_machine_envelope(
                            FederationStateMachineOperation::AdmitLocal,
                            token.wrapping_add(1),
                        );
                        assert_eq!(
                            deliver(&mut state, &predecessor, 100, true).decision(),
                            FederationDecision::AcceptedLocal
                        );
                        admitted.push(predecessor);
                    }
                    let predecessor = admitted.first().cloned().unwrap();
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.predecessor_delivery_id =
                        Some(predecessor.logical_delivery_id.clone());
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(
                        outcome.decision(),
                        FederationDecision::AcceptedLocal
                    );
                    admitted.push(candidate);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                }
                FederationStateMachineOperation::Partition => {
                    let candidate = state_machine_envelope(operation, token);
                    let outcome = deliver(&mut state, &candidate, 100, false);
                    assert_eq!(outcome.decision(), FederationDecision::PartitionUnknown);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before);
                }
                FederationStateMachineOperation::StaleSchema => {
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.schema_generation = 2;
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::StaleGeneration);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before);
                }
                FederationStateMachineOperation::Expired => {
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.expires_at = Some(100);
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(
                        outcome.decision(),
                        FederationDecision::ExpiredAuthorization
                    );
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before);
                }
                FederationStateMachineOperation::Revoked => {
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.authorization = AuthorizationState::Revoked;
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::Unauthorized);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before);
                }
                FederationStateMachineOperation::Absent => {
                    let mut candidate = state_machine_envelope(operation, token);
                    candidate.authorization = AuthorizationState::Absent;
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(outcome.decision(), FederationDecision::Unauthorized);
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before);
                }
                FederationStateMachineOperation::ForgedSourceObservation => {
                    let forged = ObservationRecord {
                        observation_id: format!("sm-forged-{token}"),
                        semantic_subject_id: "subject-1".into(),
                        payload_commitment: format!("sha256:forged-{token}"),
                        origin_node: "node-a".into(),
                        origin_node_known: false,
                        recognized_by: None,
                        source_observation: true,
                    };
                    assert_eq!(
                        record_observation(&mut state, forged),
                        ObservationWriteResult::RejectedSourceClaim
                    );
                    assert_eq!(canonical_state_fingerprint(&state), before);
                }
                FederationStateMachineOperation::ConflictExistingDelivery => {
                    if admitted.is_empty() {
                        let candidate = state_machine_envelope(
                            FederationStateMachineOperation::AdmitLocal,
                            token.wrapping_add(1),
                        );
                        assert_eq!(
                            deliver(&mut state, &candidate, 100, true).decision(),
                            FederationDecision::AcceptedLocal
                        );
                        admitted.push(candidate);
                    }
                    let mut candidate = admitted.first().cloned().unwrap();
                    candidate.payload_commitment = format!("sha256:conflict-{token}");
                    let before_conflict = canonical_state_fingerprint(&state);
                    let outcome = deliver(&mut state, &candidate, 100, true);
                    assert_eq!(
                        outcome.decision(),
                        FederationDecision::PayloadConflict
                    );
                    decision = Some(outcome.decision());
                    authority = Some(outcome.authority());
                    assert_eq!(canonical_state_fingerprint(&state), before_conflict);
                }
                FederationStateMachineOperation::InjectDeliveryMapCorruption => {
                    let logical_delivery_id = format!("sm-delivery-AdmitLocal-{token}");
                    let record = state
                        .deliveries
                        .remove(&logical_delivery_id)
                        .unwrap_or_else(|| {
                            panic!(
                                "test-only delivery-map corruption requires AdmitLocal token {token}"
                            )
                        });
                    state
                        .deliveries
                        .insert(format!("corrupt-delivery-key-{token}"), record);
                }
            }

            let post_admission_index = state.next_admission_index;
            let post_delivery_count = state.delivery_count();
            let newly_admitted_deliveries = state
                .deliveries
                .iter()
                .filter(|(id, _)| !before_delivery_ids.contains(*id))
                .map(|(id, record)| (id.clone(), record.admission_index()))
                .collect::<Vec<_>>();

            if validate_state(&state).is_err() {
                let audit = audit_state(&state);
                let mut capsule = FederationStateMachineFailureCapsule::for_invariant_failure(
                    trace_index,
                    plan,
                    step_index,
                    operation,
                    token,
                    audit,
                    before.clone(),
                    canonical_state_fingerprint(&state),
                    before_admission_index,
                    post_admission_index,
                    before_delivery_count,
                    post_delivery_count,
                    newly_admitted_deliveries.clone(),
                );
                capsule.expected_decision = match operation {
                    FederationStateMachineOperation::AdmitLocal
                    | FederationStateMachineOperation::AdmitWithPredecessor => {
                        Some(FederationDecision::AcceptedLocal)
                    }
                    FederationStateMachineOperation::ReplayExact
                    | FederationStateMachineOperation::RetryExisting => {
                        Some(FederationDecision::Duplicate)
                    }
                    FederationStateMachineOperation::RebindAttempt => {
                        Some(FederationDecision::AttemptConflict)
                    }
                    FederationStateMachineOperation::AdmitForeign
                    | FederationStateMachineOperation::AdmitRecognizedForeign => {
                        Some(FederationDecision::AcceptedForeign)
                    }
                    FederationStateMachineOperation::PendingDependency => {
                        Some(FederationDecision::PendingDependency)
                    }
                    FederationStateMachineOperation::Partition => {
                        Some(FederationDecision::PartitionUnknown)
                    }
                    FederationStateMachineOperation::StaleSchema => {
                        Some(FederationDecision::StaleGeneration)
                    }
                    FederationStateMachineOperation::Expired => {
                        Some(FederationDecision::ExpiredAuthorization)
                    }
                    FederationStateMachineOperation::Revoked
                    | FederationStateMachineOperation::Absent => {
                        Some(FederationDecision::Unauthorized)
                    }
                    FederationStateMachineOperation::ConflictExistingDelivery => {
                        Some(FederationDecision::PayloadConflict)
                    }
                    FederationStateMachineOperation::AddRecognition
                    | FederationStateMachineOperation::DuplicateRecognition
                    | FederationStateMachineOperation::RecordObservation
                    | FederationStateMachineOperation::DuplicateObservation
                    | FederationStateMachineOperation::ConflictObservation
                    | FederationStateMachineOperation::ForgedSourceObservation => None,
                    _ => None,
                };
                capsule.observed_decision = decision;
                capsule.observed_authority = authority;

                if shrink_on_failure {
                    let failing_prefix = &plan[..=step_index];
                    let target_violations = capsule.observed_violations.clone();
                    let target_operation = capsule.operation;
                    let target_token = capsule.token;
                    let minimized = shrink_failing_state_machine_plan(
                        failing_prefix,
                        |candidate| {
                            invariant_failure_from_plan(candidate, trace_index).is_some_and(|candidate_failure| {
                                candidate_failure.observed_violations == target_violations
                                    && candidate_failure.operation == target_operation
                                    && candidate_failure.token == target_token
                            })
                        },
                    );

                    let target_failed_step_index = capsule.failed_step_index;
                    let target_expected_decision = capsule.expected_decision;
                    let target_observed_decision = capsule.observed_decision;
                    let target_expected_authority = capsule.expected_authority;
                    let target_observed_authority = capsule.observed_authority;
                    let minimized = shrink_failing_state_machine_tokens(
                        &minimized,
                        |candidate| {
                            invariant_failure_from_plan(candidate, trace_index).is_some_and(|candidate_failure| {
                                candidate_failure.failed_step_index == target_failed_step_index
                                    && candidate_failure.observed_violations == target_violations
                                    && candidate_failure.operation == target_operation
                                    && candidate_failure.expected_decision == target_expected_decision
                                    && candidate_failure.observed_decision == target_observed_decision
                                    && candidate_failure.expected_authority == target_expected_authority
                                    && candidate_failure.observed_authority == target_observed_authority
                            })
                        },
                    );

                    if let Some(minimized_failure) = invariant_failure_from_plan(&minimized, trace_index) {
                        panic_invariant_failure(minimized_failure);
                    }
                }

                panic_invariant_failure(capsule);
            }

            assert!(
                post_admission_index >= before_admission_index,
                "valid state-machine transition cannot regress the admission counter"
            );
            assert!(
                post_delivery_count >= before_delivery_count,
                "valid state-machine transition cannot delete admitted deliveries"
            );

            let admission_delta = post_admission_index - before_admission_index;
            let delivery_delta = (post_delivery_count - before_delivery_count) as u64;
            assert_eq!(
                admission_delta,
                delivery_delta,
                "admission counter delta must equal admitted-delivery-count delta"
            );
            assert_eq!(
                newly_admitted_deliveries.len() as u64,
                admission_delta,
                "every consumed admission ordinal must correspond to one new delivery"
            );
            let expected_new_ordinals = (before_admission_index..post_admission_index)
                .collect::<BTreeSet<_>>();
            let observed_new_ordinals = newly_admitted_deliveries
                .iter()
                .map(|(_, admission_index)| *admission_index)
                .collect::<BTreeSet<_>>();
            assert_eq!(
                observed_new_ordinals,
                expected_new_ordinals,
                "new deliveries must receive exactly the consumed admission ordinal range"
            );

            let mut step_evidence = FederationStateMachineEvidence {
                step_index,
                operation,
                token,
                decision,
                authority,
                pre_state_fingerprint: before,
                state_fingerprint: canonical_state_fingerprint(&state),
                chain_prev_sha256: previous_chain_sha256.clone(),
                chain_sha256: String::new(),
                pre_admission_index: before_admission_index,
                post_admission_index,
                pre_delivery_count: before_delivery_count,
                post_delivery_count,
                newly_admitted_deliveries,
            };
            step_evidence.chain_sha256 =
                state_machine_evidence_chain_sha256(&step_evidence);
            previous_chain_sha256 = step_evidence.chain_sha256.clone();
            evidence.push(step_evidence);
        }

        evidence
    }

    #[test]
    fn state_machine_invariant_failure_path_emits_a_minimal_capsule() {
        let plan = vec![
            (FederationStateMachineOperation::AdmitLocal, 4096),
            (FederationStateMachineOperation::RecordObservation, 2),
            (FederationStateMachineOperation::InjectDeliveryMapCorruption, 4096),
            (FederationStateMachineOperation::Partition, 4),
        ];

        let payload = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_state_machine_trace_plan_core(&plan, 0, true);
        }))
        .expect_err("intentional invariant corruption must produce a typed failure");

        let failure = payload
            .downcast::<FederationStateMachineInvariantFailure>()
            .expect("failure path must preserve the typed capsule");

        assert_eq!(
            failure.0.observed_violations,
            vec![(
                FederationInvariantId::DeliveryMapIdentity,
                FederationInvariantViolation::DeliveryMapKeyMismatch,
            )]
        );
        assert_eq!(
            failure.0.trace_prefix,
            vec![
                (FederationStateMachineOperation::AdmitLocal, 0),
                (FederationStateMachineOperation::InjectDeliveryMapCorruption, 0),
            ]
        );
        assert_eq!(failure.0.failed_step_index, 1);
        assert_eq!(failure.0.operation, FederationStateMachineOperation::InjectDeliveryMapCorruption);
        assert_eq!(failure.0.token, 0);
        assert!(failure.0.expected_state_valid);
        assert!(!failure.0.observed_state_valid);
        assert!(failure.0.pre_state_fingerprint != failure.0.post_state_fingerprint);
        assert_eq!(failure.0.pre_admission_index, 1);
        assert_eq!(failure.0.post_admission_index, 1);
        assert_eq!(failure.0.pre_delivery_count, 1);
        assert_eq!(failure.0.post_delivery_count, 0);
        assert!(failure.0.newly_admitted_deliveries.is_empty());


        for index in 0..failure.0.trace_prefix.len() {
            let mut reduced = failure.0.trace_prefix.clone();
            reduced.remove(index);
            assert!(
                invariant_failure_from_plan(&reduced, 0).is_none(),
                "capsule is not minimal after removing step {index}"
            );
        }
        let replayed = invariant_failure_from_plan(&failure.0.trace_prefix, failure.0.trace_index)
            .expect("minimized failure capsule must replay");
        assert_eq!(
            replayed.observed_violations,
            failure.0.observed_violations,
            "replay must preserve the exact invariant-violation surface"
        );
        assert_eq!(replayed.operation, failure.0.operation);
        assert_eq!(replayed.token, failure.0.token);
        assert_eq!(
            replayed.pre_state_fingerprint,
            failure.0.pre_state_fingerprint,
            "replay must reproduce the pre-failure state fingerprint"
        );
        assert_eq!(
            replayed.post_state_fingerprint,
            failure.0.post_state_fingerprint,
            "replay must reproduce the post-failure state fingerprint"
        );

        let json = failure.0.to_json();
        let round_trip =
            serde_json::from_str::<FederationStateMachineFailureCapsule>(&json)
                .expect("failure capsule must deserialize");
        assert_eq!(round_trip, failure.0);
    }

    #[test]
    fn state_machine_parameter_shrinker_reaches_a_fixed_point() {
        let failing = vec![
            (FederationStateMachineOperation::AdmitLocal, 4096),
            (FederationStateMachineOperation::InjectDeliveryMapCorruption, 4096),
        ];
        let fails = |candidate: &[(FederationStateMachineOperation, u64)]| {
            candidate.len() == 2
                && candidate[0].0 == FederationStateMachineOperation::AdmitLocal
                && candidate[1].0 == FederationStateMachineOperation::InjectDeliveryMapCorruption
                && candidate[0].1 == candidate[1].1
                && candidate[0].1 > 0
        };

        let shrunk = shrink_failing_state_machine_tokens(&failing, fails);
        assert_eq!(
            shrunk,
            vec![
                (FederationStateMachineOperation::AdmitLocal, 1),
                (FederationStateMachineOperation::InjectDeliveryMapCorruption, 1),
            ]
        );
        assert!(fails(&shrunk));

        for index in 0..shrunk.len() {
            for candidate_token in state_machine_token_shrink_candidates(shrunk[index].1) {
                if candidate_token >= shrunk[index].1 {
                    continue;
                }
                let mut candidate = shrunk.clone();
                candidate[index].1 = candidate_token;
                assert!(
                    !fails(&candidate),
                    "parameter-shrunk sequence is not at a fixed point for token index {index}"
                );
            }
        }

        for candidate_token in state_machine_token_shrink_candidates(shrunk[0].1) {
            if candidate_token >= shrunk[0].1 {
                continue;
            }
            let candidate = shrunk
                .iter()
                .map(|(operation, token)| {
                    (
                        *operation,
                        if *token == shrunk[0].1 {
                            candidate_token
                        } else {
                            *token
                        },
                    )
                })
                .collect::<Vec<_>>();
            assert!(
                !fails(&candidate),
                "parameter-shrunk sequence is not at a fixed point for linked token replacement"
            );
        }
    }

    #[test]
    fn state_machine_failure_sequence_shrinker_finds_minimal_reproduction() {
        let (_, plan) = state_machine_trace_plan(23, 16);
        let mut failing = vec![
            (FederationStateMachineOperation::AddRecognition, 1),
            (FederationStateMachineOperation::RecordObservation, 2),
            (FederationStateMachineOperation::AdmitLocal, 3),
            (FederationStateMachineOperation::RetryExisting, 4),
            (FederationStateMachineOperation::Partition, 5),
            (FederationStateMachineOperation::Expired, 6),
            (FederationStateMachineOperation::Revoked, 7),
            (FederationStateMachineOperation::Absent, 8),
            (FederationStateMachineOperation::ConflictObservation, 9),
            (FederationStateMachineOperation::AddRecognition, 10),
            (FederationStateMachineOperation::RecordObservation, 11),
            (FederationStateMachineOperation::Partition, 12),
            (FederationStateMachineOperation::StaleSchema, 13),
            (FederationStateMachineOperation::AddRecognition, 14),
            (FederationStateMachineOperation::PendingDependency, 15),
            (FederationStateMachineOperation::AdmitForeign, 16),
        ];
        failing[2].1 = plan[2].1;
        failing[3].1 = plan[3].1;

        let fails = |candidate: &[(FederationStateMachineOperation, u64)]| {
            let has_admission = candidate
                .iter()
                .any(|(operation, _)| *operation == FederationStateMachineOperation::AdmitLocal);
            let has_retry = candidate
                .iter()
                .any(|(operation, _)| *operation == FederationStateMachineOperation::RetryExisting);
            has_admission && has_retry
        };

        assert!(fails(&failing));
        let shrunk = shrink_failing_state_machine_plan(&failing, fails);
        assert_eq!(
            shrunk,
            vec![
                (FederationStateMachineOperation::AdmitLocal, plan[2].1),
                (FederationStateMachineOperation::RetryExisting, plan[3].1),
            ]
        );
        assert!(fails(&shrunk));
        assert!(
            shrunk.len() < failing.len(),
            "shrinker must reduce a non-minimal failing sequence"
        );
        for index in 0..shrunk.len() {
            let mut reduced = shrunk.clone();
            reduced.remove(index);
            assert!(
                !fail    #[test]
    fn state_machine_trace_capsule_round_trips_and_replays_exactly() {
        let capsule_text = state_machine_trace_capsule(17, 32);
        let capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("deterministic trace capsule must deserialize");

        assert_eq!(
            capsule.schema_version,
            FEDERATION_STATE_MACHINE_TRACE_CAPSULE_SCHEMA_VERSION
        );

        let (_, generated_plan) = state_machine_trace_plan(capsule.trace_index, 32);
        let capsule_plan = state_machine_plan_from_capsule(&capsule);
        assert_eq!(generated_plan, capsule_plan);

        assert_eq!(
            state_machine_trace_capsule(capsule.trace_index, 32),
            serde_json::to_string_pretty(&capsule)
                .expect("trace capsule serialization must be deterministic")
        );

        assert_eq!(
            run_state_machine_trace_plan(&generated_plan),
            capsule.evidence
        );

        let last = capsule
            .evidence
            .last()
            .expect("32-step trace must have terminal evidence");
        assert_eq!(capsule.final_state.admission_index, last.post_admission_index);
        assert_eq!(capsule.final_state.delivery_count, last.post_delivery_count);
        assert_eq!(capsule.final_state.state_fingerprint, last.state_fingerprint);
    }

    #[test]
    fn state_machine_empty_trace_capsule_has_matching_boundaries() {
        let capsule_text = state_machine_trace_capsule(0, 0);
        let capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("empty trace capsule must deserialize");

        assert!(capsule.evidence.is_empty());
        assert_eq!(capsule.initial_state, capsule.final_state);
        assert_eq!(
            capsule.integrity.chain_head_sha256,
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS
        );
        assert_eq!(state_machine_plan_from_capsule(&capsule), Vec::new());
    }

    #[test]
    fn state_machine_trace_capsule_rejects_seed_operation_and_token_tampering() {
        let capsule_text = state_machine_trace_capsule(5, 12);
        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");

        capsule.initial_seed ^= 1;
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.verification_profile = "future-profile".into();
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        let original_operation = capsule.operations[3];
        capsule.operations[3] = match original_operation {
            FederationStateMachineOperation::AddRecognition => {
                FederationStateMachineOperation::DuplicateRecognition
            }
            _ => FederationStateMachineOperation::AddRecognition,
        };
        assert_ne!(capsule.operations[3], original_operation);
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.tokens[7] ^= 1;
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());
    }

    #[test]
    fn state_machine_trace_capsule_rejects_unknown_fields_at_every_struct_boundary() {
        let capsule_text = state_machine_trace_capsule(2, 6);

        let mut value =
            serde_json::from_str::<serde_json::Value>(&capsule_text)
                .expect("generated trace capsule JSON must parse");
        value
            .as_object_mut()
            .expect("trace capsule must serialize as an object")
            .insert("unexpected_field".into(), serde_json::Value::Bool(true));
        let tampered = serde_json::to_string_pretty(&value)
            .expect("tampered capsule JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&tampered).is_err(),
            "successful trace capsule must reject unknown top-level fields"
        );

        let mut value =
            serde_json::from_str::<serde_json::Value>(&capsule_text)
                .expect("generated trace capsule JSON must parse");
        value["initial_state"]
            .as_object_mut()
            .expect("initial state must serialize as an object")
            .insert("unexpected_boundary_field".into(), serde_json::Value::Bool(true));
        let tampered = serde_json::to_string_pretty(&value)
            .expect("tampered boundary JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&tampered).is_err(),
            "successful trace capsule must reject unknown boundary fields"
        );

        let mut value =
            serde_json::from_str::<serde_json::Value>(&capsule_text)
                .expect("generated trace capsule JSON must parse");
        value["evidence"][0]
            .as_object_mut()
            .expect("step evidence must serialize as an object")
            .insert("unexpected_evidence_field".into(), serde_json::Value::Bool(true));
        let tampered = serde_json::to_string_pretty(&value)
            .expect("tampered evidence JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&tampered).is_err(),
            "successful trace capsule must reject unknown evidence fields"
        );

        let mut value =
            serde_json::from_str::<serde_json::Value>(&capsule_text)
                .expect("generated trace capsule JSON must parse");
        value["integrity"]
            .as_object_mut()
            .expect("integrity must serialize as an object")
            .insert("unexpected_integrity_field".into(), serde_json::Value::Bool(true));
        let tampered = serde_json::to_string_pretty(&value)
            .expect("tampered integrity JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&tampered).is_err(),
            "successful trace capsule must reject unknown integrity fields"
        );

        let capsule = serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
            .expect("generated trace capsule must deserialize");
        let checkpoint = state_machine_trace_checkpoint(
            &capsule,
            0,
            capsule.evidence.len(),
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS,
        );
        let mut checkpoint_value =
            serde_json::to_value(&checkpoint).expect("checkpoint must serialize");
        checkpoint_value
            .as_object_mut()
            .expect("checkpoint must serialize as an object")
            .insert("unexpected_checkpoint_field".into(), serde_json::Value::Bool(true));
        let tampered_checkpoint = serde_json::to_string_pretty(&checkpoint_value)
            .expect("tampered checkpoint JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTraceCheckpoint>(&tampered_checkpoint)
                .is_err(),
            "trace checkpoint must reject unknown fields"
        );

        let publication = state_machine_trace_checkpoint_publication(
            &capsule,
            capsule.evidence.len(),
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let mut publication_value =
            serde_json::to_value(&publication).expect("publication must serialize");
        publication_value
            .as_object_mut()
            .expect("publication must serialize as an object")
            .insert("unexpected_publication_field".into(), serde_json::Value::Bool(true));
        let tampered_publication = serde_json::to_string_pretty(&publication_value)
            .expect("tampered publication JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTraceCheckpointPublication>(
                &tampered_publication
            )
            .is_err(),
            "trace checkpoint publication must reject unknown fields"
        );
    }

    #[test]
    fn state_machine_trace_capsule_rejects_integrity_tampering() {
        let capsule_text = state_machine_trace_capsule(6, 9);
        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");

        capsule.integrity.body_sha256 =
            "sha256:0000000000000000000000000000000000000000000000000000000000000000"
                .into();
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.integrity.algorithm = "sha-512".into();
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.integrity.encoding = "rfc8785-jcs".into();
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());
    }

    #[test]
    fn state_machine_trace_capsule_hash_changes_with_authenticated_body_mutation() {
        let capsule_text = state_machine_trace_capsule(12, 9);
        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        let original_digest = capsule.integrity.body_sha256.clone();

        capsule.final_state.delivery_count ^= 1;

        assert_ne!(
            original_digest,
            state_machine_trace_body_sha256(&capsule),
            "authenticated body mutations must change the body digest"
        );
        assert!(
            std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err(),
            "authenticated body mutations must invalidate the capsule"
        );
    }

    fn state_machine_trace_capsule_hash_is_stable_across_json_reformatting() {
        let capsule_text = state_machine_trace_capsule(4, 7);
        let capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        let compact = serde_json::to_string(&capsule).expect("compact JSON must serialize");
        let reformatted =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&compact)
                .expect("reformatted capsule must deserialize");
        assert_eq!(
            capsule.integrity.body_sha256,
            reformatted.integrity.body_sha256
        );
        assert_eq!(
            state_machine_trace_body_sha256(&capsule),
            state_machine_trace_body_sha256(&reformatted)
        );
    }

    fn reseal_trace_checkpoint_for_test(
        checkpoint: &mut FederationStateMachineTraceCheckpoint,
    ) {
        checkpoint.checkpoint_sha256 =
            state_machine_trace_checkpoint_sha256(checkpoint);
    }

    #[test]
    fn state_machine_trace_checkpoints_form_an_independently_verifiable_contiguous_receipt_chain() {
        let capsule_text = state_machine_trace_capsule(25, 12);
        let capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");

        let first = state_machine_trace_checkpoint(
            &capsule,
            0,
            4,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS,
        );
        let second = state_machine_trace_checkpoint(
            &capsule,
            4,
            8,
            &first.checkpoint_sha256,
        );
        let third = state_machine_trace_checkpoint(
            &capsule,
            8,
            12,
            &second.checkpoint_sha256,
        );
        let chain = vec![first.clone(), second.clone(), third.clone()];

        assert!(
            validate_state_machine_trace_checkpoint_chain(&capsule, &chain).is_ok(),
            "generated checkpoint receipt chain must validate independently of replay"
        );
        assert_eq!(
            first.chain_start_prev_sha256,
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS
        );
        assert_eq!(third.step_end, capsule.evidence.len());

        for checkpoint in &chain {
            let json = serde_json::to_string_pretty(checkpoint)
                .expect("checkpoint must serialize");
            let round_trip =
                serde_json::from_str::<FederationStateMachineTraceCheckpoint>(&json)
                    .expect("checkpoint must deserialize");
            assert_eq!(&round_trip, checkpoint);
            assert_eq!(
                json,
                serde_json::to_string_pretty(&round_trip)
                    .expect("checkpoint serialization must be deterministic")
            );
        }
    }

    #[test]
    fn state_machine_trace_checkpoints_reject_resealed_chain_reordering_and_range_forgery() {
        let capsule_text = state_machine_trace_capsule(26, 12);
        let capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");

        let first = state_machine_trace_checkpoint(
            &capsule,
            0,
            4,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS,
        );
        let second = state_machine_trace_checkpoint(
            &capsule,
            4,
            8,
            &first.checkpoint_sha256,
        );
        let third = state_machine_trace_checkpoint(
            &capsule,
            8,
            12,
            &second.checkpoint_sha256,
        );

        let mut reordered = vec![first.clone(), third.clone(), second.clone()];
        assert_eq!(
            validate_state_machine_trace_checkpoint_chain(&capsule, &reordered),
            Err(FederationStateMachineTraceCheckpointViolation::CheckpointRangeGap)
        );

        let mut forged = second.clone();
        forged.step_start = 5;
        reseal_trace_checkpoint_for_test(&mut forged);
        let forged_chain = vec![first.clone(), forged.clone(), third.clone()];
        assert_eq!(
            validate_state_machine_trace_checkpoint_chain(&capsule, &forged_chain),
            Err(FederationStateMachineTraceCheckpointViolation::CheckpointRangeGap)
        );

        reordered.swap(1, 2);
        let mut forged_profile = second.clone();
        forged_profile.checkpoint_profile =
            "integral-federation-trace-checkpoint-receipt-v2".into();
        reseal_trace_checkpoint_for_test(&mut forged_profile);
        assert_eq!(
            validate_state_machine_trace_checkpoint(&capsule, &forged_profile),
            Err(
                FederationStateMachineTraceCheckpointViolation::UnsupportedCheckpointProfile
            )
        );

        let mut resealed_prev = second.clone();
        resealed_prev.previous_checkpoint_sha256 =
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS.into();
        reseal_trace_checkpoint_for_test(&mut resealed_prev);
        let resealed_chain = vec![first, resealed_prev, third];
        assert_eq!(
            validate_state_machine_trace_checkpoint_chain(&capsule, &resealed_chain),
            Err(FederationStateMachineTraceCheckpointViolation::PreviousCheckpointMismatch)
        );
    }

    #[test]
    fn state_machine_trace_checkpoint_publication_hash_domain_is_distinct_from_other_receipts() {
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_HASH_DOMAIN
        );
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN
        );
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS
        );
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_PROFILE,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PROFILE
        );
    }

    #[test]
    fn state_machine_trace_checkpoint_hash_domain_is_distinct_from_trace_hash_domains() {
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN
        );
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN
        );
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_GENESIS,
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS
        );
        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PROFILE,
            FEDERATION_STATE_MACHINE_TRACE_VERIFICATION_PROFILE
        );
    }

    fn reseal_trace_checkpoint_publication_for_test(
        publication: &mut FederationStateMachineTraceCheckpointPublication,
    ) {
        publication.publication_sha256 =
            state_machine_trace_checkpoint_publication_sha256(publication);
    }

    #[test]
    fn publication_equivocation_witness_is_canonical_and_self_validating() {
        let base_text = state_machine_trace_capsule(41, 8);
        let fork_a_text = state_machine_trace_capsule(41, 12);
        let mut fork_b_text = state_machine_trace_capsule(41, 12);

        let base =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&base_text)
                .expect("base capsule must deserialize");
        let fork_a =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&fork_a_text)
                .expect("fork-a capsule must deserialize");
        let mut fork_b =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&fork_b_text)
                .expect("fork-b capsule must deserialize");

        fork_b.evidence[10].token ^= 1;
        reseal_state_machine_trace_for_test(&mut fork_b);

        let base_publication = state_machine_trace_checkpoint_publication(
            &base,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let fork_a_publication = state_machine_trace_checkpoint_publication(
            &fork_a,
            12,
            &base_publication.publication_sha256,
        );
        let fork_b_publication = state_machine_trace_checkpoint_publication(
            &fork_b,
            12,
            &base_publication.publication_sha256,
        );

        let witness =
            state_machine_trace_publication_equivocation_witness(&fork_a_publication, &fork_b_publication);

        assert!(
            validate_state_machine_trace_publication_equivocation_witness(
                &fork_a,
                &fork_a_publication,
                &fork_b,
                &fork_b_publication,
                &witness,
            )
            .is_ok()
        );

        let reverse_witness =
            state_machine_trace_publication_equivocation_witness(&fork_b_publication, &fork_a_publication);
        assert_eq!(witness, reverse_witness);

        let json = serde_json::to_string_pretty(&witness)
            .expect("equivocation witness must serialize");
        let round_trip =
            serde_json::from_str::<FederationStateMachineTracePublicationEquivocationWitness>(
                &json,
            )
            .expect("equivocation witness must deserialize");
        assert_eq!(round_trip, witness);
        assert_eq!(
            json,
            serde_json::to_string_pretty(&round_trip)
                .expect("equivocation witness serialization must be deterministic")
        );

        let mut bad_digest = witness.clone();
        bad_digest.witness_sha256 = "sha256:invalid-equivo-witness".into();
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness(
                &fork_a,
                &fork_a_publication,
                &fork_b,
                &fork_b_publication,
                &bad_digest,
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::WitnessDigestMismatch
            )
        );

        let mut bad_binding = witness.clone();
        bad_binding.first_publication_sha256 = bad_binding.second_publication_sha256.clone();
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness(
                &fork_a,
                &fork_a_publication,
                &fork_b,
                &fork_b_publication,
                &bad_binding,
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::WitnessPublicationBindingMismatch
            )
        );

        let mut bad_predecessor = witness.clone();
        bad_predecessor.predecessor_publication_sha256 =
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS.into();
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness(
                &fork_a,
                &fork_a_publication,
                &fork_b,
                &fork_b_publication,
                &bad_predecessor,
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::PredecessorMismatch
            )
        );

        let mut not_distinct_witness = witness.clone();
        not_distinct_witness.second_publication_sha256 =
            not_distinct_witness.first_publication_sha256.clone();
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness(
                &fork_a,
                &fork_a_publication,
                &fork_a,
                &fork_a_publication,
                &not_distinct_witness,
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::PublicationsNotDistinct
            )
        );

        let mut same_publication = fork_b_publication.clone();
        same_publication.publication_sha256 = fork_a_publication.publication_sha256.clone();
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness(
                &fork_a,
                &fork_a_publication,
                &fork_b,
                &same_publication,
                &witness,
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessViolation::SecondPublicationInvalid(
                    FederationStateMachineTraceCheckpointPublicationViolation::PublicationDigestMismatch
                )
            )
        );

        let mut unknown = serde_json::to_value(&witness)
            .expect("equivocation witness must serialize as a JSON value");
        unknown
            .as_object_mut()
            .expect("equivocation witness must serialize as an object")
            .insert("unexpected_equivocation_field".into(), serde_json::Value::Bool(true));
        let unknown_json =
            serde_json::to_string_pretty(&unknown).expect("tampered witness JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTracePublicationEquivocationWitness>(
                &unknown_json
            )
            .is_err()
        );
    }

    #[test]
    fn publication_equivocation_witness_set_is_complete_for_multiway_forks() {
        let base =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(43, 8),
            )
            .expect("base capsule must deserialize");

        let fork_a =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(43, 12),
            )
            .expect("fork-a capsule must deserialize");

        let mut fork_b =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(43, 12),
            )
            .expect("fork-b capsule must deserialize");
        fork_b.evidence[10].token ^= 1;
        reseal_state_machine_trace_for_test(&mut fork_b);

        let mut fork_c =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(43, 12),
            )
            .expect("fork-c capsule must deserialize");
        fork_c.evidence[9].token ^= 1;
        reseal_state_machine_trace_for_test(&mut fork_c);

        let base_publication = state_machine_trace_checkpoint_publication(
            &base,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let fork_a_publication = state_machine_trace_checkpoint_publication(
            &fork_a,
            12,
            &base_publication.publication_sha256,
        );
        let fork_b_publication = state_machine_trace_checkpoint_publication(
            &fork_b,
            12,
            &base_publication.publication_sha256,
        );
        let fork_c_publication = state_machine_trace_checkpoint_publication(
            &fork_c,
            12,
            &base_publication.publication_sha256,
        );

        let collection = vec![
            (&base, &base_publication),
            (&fork_a, &fork_a_publication),
            (&fork_b, &fork_b_publication),
            (&fork_c, &fork_c_publication),
        ];

        let witnesses =
            collect_state_machine_trace_publication_equivocation_witnesses(&collection)
                .expect("three-way fork collection must audit");
        assert_eq!(witnesses.len(), 3);
        assert!(
            witnesses
                .windows(2)
                .all(|pair| pair[0].witness_sha256 < pair[1].witness_sha256)
        );

        let set = state_machine_trace_publication_equivocation_witness_set(&collection)
            .expect("three-way fork must produce a complete witness set");
        assert_eq!(set.witnesses, witnesses);
        assert_eq!(set.collection_size, 4);
        assert_eq!(
            set.collection_sha256,
            state_machine_trace_publication_collection_commitment(&collection)
                .expect("collection commitment must compute")
                .1
        );
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &collection, &set
            ),
            Ok(())
        );

        let mut exact_replay_collection = collection.clone();
        exact_replay_collection.push((&fork_a, &fork_a_publication));
        let replay_set =
            state_machine_trace_publication_equivocation_witness_set(&exact_replay_collection)
                .expect("exact publication replay must remain idempotent");
        assert_eq!(replay_set, set);

        let unrelated_capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(49, 8),
            )
            .expect("unrelated capsule must deserialize");
        let unrelated_publication = state_machine_trace_checkpoint_publication(
            &unrelated_capsule,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let mut extended_collection = collection.clone();
        extended_collection.push((&unrelated_capsule, &unrelated_publication));
        let extended_set =
            state_machine_trace_publication_equivocation_witness_set(&extended_collection)
                .expect("extended qualified collection must produce a witness set");
        assert_eq!(extended_set.witnesses, set.witnesses);
        assert_eq!(extended_set.collection_size, 5);
        assert_ne!(extended_set.collection_sha256, set.collection_sha256);
        assert_ne!(extended_set.set_sha256, set.set_sha256);
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &extended_collection, &set
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::CollectionSizeMismatch
            )
        );

        let mut reversed = collection.clone();
        reversed.reverse();
        let reversed_set =
            state_machine_trace_publication_equivocation_witness_set(&reversed)
                .expect("reordered three-way fork must produce a complete witness set");
        assert_eq!(reversed_set, set);

        let json =
            serde_json::to_string_pretty(&set).expect("witness set must serialize");
        let round_trip =
            serde_json::from_str::<FederationStateMachineTracePublicationEquivocationWitnessSet>(
                &json,
            )
            .expect("witness set must deserialize");
        assert_eq!(round_trip, set);
        assert_eq!(json, serde_json::to_string_pretty(&round_trip).unwrap());

        let mut omitted = set.clone();
        omitted.witnesses.pop();
        omitted.set_sha256 =
            state_machine_trace_publication_equivocation_witness_set_sha256(&omitted);
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &collection, &omitted
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessSetCoverageMismatch
            )
        );

        assert_eq!(set.collection_size, 4);
        assert_eq!(
            set.collection_sha256,
            state_machine_trace_publication_collection_commitment(&collection)
                .expect("collection commitment must compute")
                .1
        );

        let mut bad_collection_size = set.clone();
        bad_collection_size.collection_size += 1;
        bad_collection_size.set_sha256 =
            state_machine_trace_publication_equivocation_witness_set_sha256(&bad_collection_size);
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &collection, &bad_collection_size
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::CollectionSizeMismatch
            )
        );

        let mut bad_collection_digest = set.clone();
        bad_collection_digest.collection_sha256 = "sha256:invalid-publication-collection".into();
        bad_collection_digest.set_sha256 =
            state_machine_trace_publication_equivocation_witness_set_sha256(&bad_collection_digest);
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &collection, &bad_collection_digest
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::CollectionDigestMismatch
            )
        );

        let mut duplicate = set.clone();
        duplicate.witnesses.push(duplicate.witnesses[0].clone());
        duplicate.witnesses.sort_by(|a, b| a.witness_sha256.cmp(&b.witness_sha256));
        duplicate.set_sha256 =
            state_machine_trace_publication_equivocation_witness_set_sha256(&duplicate);
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &collection, &duplicate
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::DuplicateWitness
            )
        );

        let mut reordered_set = set.clone();
        reordered_set.witnesses.reverse();
        reordered_set.set_sha256 =
            state_machine_trace_publication_equivocation_witness_set_sha256(&reordered_set);
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &collection, &reordered_set
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::WitnessNotCanonical
            )
        );

        let mut invalid_collection_publication = fork_a_publication.clone();
        invalid_collection_publication.body_sha256 = "sha256:invalid-set-publication".into();
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &[
                    (&base, &base_publication),
                    (&fork_a, &invalid_collection_publication),
                    (&fork_b, &fork_b_publication),
                    (&fork_c, &fork_c_publication),
                ],
                &set,
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::PublicationInvalid(
                    FederationStateMachineTraceCheckpointPublicationViolation::BodyDigestMismatch
                )
            )
        );

        let mut bad_digest = set.clone();
        bad_digest.set_sha256 = "sha256:invalid-witness-set".into();
        assert_eq!(
            validate_state_machine_trace_publication_equivocation_witness_set(
                &collection, &bad_digest
            ),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetViolation::SetDigestMismatch
            )
        );

        let mut unknown = serde_json::to_value(&set)
            .expect("witness set must serialize as a JSON value");
        unknown
            .as_object_mut()
            .expect("witness set must serialize as an object")
            .insert("unexpected_witness_set_field".into(), serde_json::Value::Bool(true));
        assert!(
            serde_json::from_value::<FederationStateMachineTracePublicationEquivocationWitnessSet>(
                unknown
            )
            .is_err()
        );
    }

    #[test]
    fn publication_equivocation_witness_set_requires_an_actual_fork() {
        let capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(47, 8),
            )
            .expect("capsule must deserialize");
        let publication = state_machine_trace_checkpoint_publication(
            &capsule,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );

        assert_eq!(
            state_machine_trace_publication_equivocation_witness_set(&[(&capsule, &publication)]),
            Err(
                FederationStateMachineTracePublicationEquivocationWitnessSetBuildViolation::NoEquivocationDetected
            )
        );
    }

    #[test]
    fn publication_collection_reconciliation_is_permutation_invariant_and_detects_cross_view_equivocation() {
        let base =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(53, 8),
            )
            .expect("base capsule must deserialize");
        let fork_a =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(53, 12),
            )
            .expect("fork-a capsule must deserialize");
        let mut fork_b =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(53, 12),
            )
            .expect("fork-b capsule must deserialize");
        fork_b.evidence[10].token ^= 1;
        reseal_state_machine_trace_for_test(&mut fork_b);

        let base_publication = state_machine_trace_checkpoint_publication(
            &base,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let fork_a_publication = state_machine_trace_checkpoint_publication(
            &fork_a,
            12,
            &base_publication.publication_sha256,
        );
        let fork_b_publication = state_machine_trace_checkpoint_publication(
            &fork_b,
            12,
            &base_publication.publication_sha256,
        );

        let left = vec![(&base, &base_publication), (&fork_a, &fork_a_publication)];
        let right = vec![(&base, &base_publication), (&fork_b, &fork_b_publication)];

        let receipt =
            state_machine_trace_publication_collection_reconciliation(&left, &right)
                .expect("cross-view reconciliation must build");

        assert_eq!(
            receipt.relationship,
            FederationStateMachineTracePublicationCollectionRelationship::OverlappingDivergence
        );
        assert_eq!(
            receipt.history_relationship,
            FederationStateMachineTracePublicationCollectionHistoryRelationship::DivergentAfterCommonPrefix
        );
        assert_eq!(
            receipt.shared_publication_sha256s,
            vec![base_publication.publication_sha256.clone()]
        );
        assert_eq!(
            receipt.left_only_publication_sha256s,
            vec![fork_a_publication.publication_sha256.clone()]
        );
        assert_eq!(
            receipt.right_only_publication_sha256s,
            vec![fork_b_publication.publication_sha256.clone()]
        );
        assert_eq!(receipt.equivocation_witness_sha256s.len(), 1);
        assert_eq!(
            validate_state_machine_trace_publication_collection_reconciliation(
                &left, &right, &receipt
            ),
            Ok(())
        );

        let mut reversed_left = left.clone();
        reversed_left.reverse();
        let mut reversed_right = right.clone();
        reversed_right.reverse();
        let reordered =
            state_machine_trace_publication_collection_reconciliation(
                &reversed_left,
                &reversed_right,
            )
            .expect("reordered views must reconcile identically");
        assert_eq!(reordered, receipt);

        let same =
            state_machine_trace_publication_collection_reconciliation(&left, &left)
                .expect("identical views must reconcile");
        assert_eq!(
            same.relationship,
            FederationStateMachineTracePublicationCollectionRelationship::ExactMatch
        );
        assert!(same.equivocation_witness_sha256s.is_empty());

        let subset =
            state_machine_trace_publication_collection_reconciliation(
                &[(&base, &base_publication)],
                &left,
            )
            .expect("subset views must reconcile");
        assert_eq!(
            subset.relationship,
            FederationStateMachineTracePublicationCollectionRelationship::LeftStrictSubset
        );
        assert_eq!(
            subset.history_relationship,
            FederationStateMachineTracePublicationCollectionHistoryRelationship::LeftStrictPrefix
        );

        let reverse_subset =
            state_machine_trace_publication_collection_reconciliation(
                &left,
                &[(&base, &base_publication)],
            )
            .expect("reverse subset views must reconcile");
        assert_eq!(
            reverse_subset.relationship,
            FederationStateMachineTracePublicationCollectionRelationship::RightStrictSubset
        );
        assert_eq!(
            reverse_subset.history_relationship,
            FederationStateMachineTracePublicationCollectionHistoryRelationship::RightStrictPrefix
        );

        let sparse =
            state_machine_trace_publication_collection_reconciliation(
                &[(&fork_a, &fork_a_publication)],
                &right,
            )
            .expect("sparse view must reconcile without being treated as a prefix");
        assert_eq!(
            sparse.relationship,
            FederationStateMachineTracePublicationCollectionRelationship::LeftStrictSubset
        );
        assert_eq!(
            sparse.history_relationship,
            FederationStateMachineTracePublicationCollectionHistoryRelationship::NotComparable
        );
        assert!(sparse.equivocation_witness_sha256s.is_empty());

        let unrelated =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(
                &state_machine_trace_capsule(59, 8),
            )
            .expect("unrelated capsule must deserialize");
        let unrelated_publication = state_machine_trace_checkpoint_publication(
            &unrelated,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let disjoint =
            state_machine_trace_publication_collection_reconciliation(
                &[(&unrelated, &unrelated_publication)],
                &right,
            )
            .expect("disjoint views must reconcile");
        assert_eq!(
            disjoint.relationship,
            FederationStateMachineTracePublicationCollectionRelationship::Disjoint
        );
        assert_eq!(
            disjoint.history_relationship,
            FederationStateMachineTracePublicationCollectionHistoryRelationship::Disjoint
        );
        assert!(disjoint.equivocation_witness_sha256s.is_empty());

        let mut bad = receipt.clone();
        bad.relationship =
            FederationStateMachineTracePublicationCollectionRelationship::ExactMatch;
        bad.reconciliation_sha256 =
            state_machine_trace_publication_collection_reconciliation_sha256(&bad);
        assert_eq!(
            validate_state_machine_trace_publication_collection_reconciliation(
                &left, &right, &bad
            ),
            Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::RelationshipMismatch
            )
        );

        let mut bad_left_digest = receipt.clone();
        bad_left_digest.left_collection_sha256 = "sha256:invalid-left-view".into();
        bad_left_digest.reconciliation_sha256 =
            state_machine_trace_publication_collection_reconciliation_sha256(&bad_left_digest);
        assert_eq!(
            validate_state_machine_trace_publication_collection_reconciliation(
                &left, &right, &bad_left_digest
            ),
            Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::LeftCollectionDigestMismatch
            )
        );

        let mut stale_history_digest = receipt.clone();
        stale_history_digest.history_relationship =
            FederationStateMachineTracePublicationCollectionHistoryRelationship::ExactMatch;
        assert_eq!(
            validate_state_machine_trace_publication_collection_reconciliation(
                &left, &right, &stale_history_digest
            ),
            Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::ReconciliationDigestMismatch
            )
        );

        let mut bad_history = receipt.clone();
        bad_history.history_relationship =
            FederationStateMachineTracePublicationCollectionHistoryRelationship::ExactMatch;
        bad_history.reconciliation_sha256 =
            state_machine_trace_publication_collection_reconciliation_sha256(&bad_history);
        assert_eq!(
            validate_state_machine_trace_publication_collection_reconciliation(
                &left, &right, &bad_history
            ),
            Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::HistoryRelationshipMismatch
            )
        );

        let mut bad_witnesses = receipt.clone();
        bad_witnesses.equivocation_witness_sha256s.clear();
        bad_witnesses.reconciliation_sha256 =
            state_machine_trace_publication_collection_reconciliation_sha256(&bad_witnesses);
        assert_eq!(
            validate_state_machine_trace_publication_collection_reconciliation(
                &left, &right, &bad_witnesses
            ),
            Err(
                FederationStateMachineTracePublicationCollectionReconciliationViolation::EquivocationWitnessCoverageMismatch
            )
        );

        let mut unknown = serde_json::to_value(&receipt)
            .expect("reconciliation receipt must serialize");
        unknown
            .as_object_mut()
            .expect("reconciliation receipt must be an object")
            .insert("unexpected_reconciliation_field".into(), serde_json::Value::Bool(true));
        assert!(
            serde_json::from_value::<
                FederationStateMachineTracePublicationCollectionReconciliationReceipt,
            >(unknown)
            .is_err()
        );
    }

    #[test]
    fn state_machine_trace_checkpoint_publications_detect_equivocation() {
        let base_text = state_machine_trace_capsule(32, 8);
        let fork_a_text = state_machine_trace_capsule(32, 12);
        let fork_b_text = state_machine_trace_capsule(32, 12);

        let base =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&base_text)
                .expect("base capsule must deserialize");
        let fork_a =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&fork_a_text)
                .expect("fork-a capsule must deserialize");
        let mut fork_b =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&fork_b_text)
                .expect("fork-b capsule must deserialize");

        fork_b.evidence[10].token ^= 1;
        reseal_state_machine_trace_for_test(&mut fork_b);

        let base_publication = state_machine_trace_checkpoint_publication(
            &base,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let fork_a_publication = state_machine_trace_checkpoint_publication(
            &fork_a,
            12,
            &base_publication.publication_sha256,
        );
        let fork_b_publication = state_machine_trace_checkpoint_publication(
            &fork_b,
            12,
            &base_publication.publication_sha256,
        );

        assert_ne!(
            fork_a_publication.publication_sha256,
            fork_b_publication.publication_sha256,
            "the divergent successor publications must have distinct digests"
        );
        let fork_set = vec![
            base_publication.clone(),
            fork_a_publication.clone(),
            fork_b_publication.clone(),
        ];
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_set(&fork_set),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationForkDetected
            )
        );
        let mut reversed_fork_set = fork_set.clone();
        reversed_fork_set.reverse();
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_set(&reversed_fork_set),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationForkDetected
            ),
            "fork classification must not depend on publication collection order"
        );

        let same_endpoint_a = state_machine_trace_checkpoint_publication(
            &fork_a,
            8,
            &base_publication.publication_sha256,
        );
        let mut same_endpoint_b_snapshot = fork_a.clone();
        same_endpoint_b_snapshot.evidence[0].token ^= 1;
        reseal_state_machine_trace_for_test(&mut same_endpoint_b_snapshot);
        let same_endpoint_b = state_machine_trace_checkpoint_publication(
            &same_endpoint_b_snapshot,
            8,
            &base_publication.publication_sha256,
        );
        assert_ne!(
            same_endpoint_a.publication_sha256,
            same_endpoint_b.publication_sha256,
            "divergent snapshots at one checkpoint endpoint must have distinct publication digests"
        );
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_set(
                &[base_publication.clone(), same_endpoint_a, same_endpoint_b]
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationForkDetected
            )
        );

        let mut profile_forged = fork_a_publication.clone();
        profile_forged.trace_verification_profile =
            "integral-federation-state-machine-trace-v2".into();
        reseal_trace_checkpoint_publication_for_test(&mut profile_forged);
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication(
                &fork_a,
                &profile_forged
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::TraceVerificationProfileMismatch
            )
        );

        let duplicate_set = vec![base_publication.clone(), base_publication];
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(&duplicate_set).is_ok(),
            "exact publication replay is not a fork"
        );

        let mut future_publication_schema = fork_a_publication.clone();
        future_publication_schema.schema_version += 1;
        reseal_trace_checkpoint_publication_for_test(&mut future_publication_schema);
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(&[
                fork_a_publication.clone(),
                future_publication_schema,
            ])
            .is_ok(),
            "publication records from distinct schema namespaces must not be conflated as a fork"
        );

        let mut future_trace_schema = fork_a_publication.clone();
        future_trace_schema.trace_schema_version += 1;
        reseal_trace_checkpoint_publication_for_test(&mut future_trace_schema);
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(&[
                fork_a_publication.clone(),
                future_trace_schema,
            ])
            .is_ok(),
            "trace records from distinct schema namespaces must not be conflated as a fork"
        );

        let mut alternate_hash_algorithm = fork_a_publication.clone();
        alternate_hash_algorithm.hash_algorithm = "sha-512".into();
        reseal_trace_checkpoint_publication_for_test(&mut alternate_hash_algorithm);
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(&[
                fork_a_publication.clone(),
                alternate_hash_algorithm,
            ])
            .is_ok(),
            "publication records from distinct hash-algorithm namespaces must not be conflated as a fork"
        );

        let mut alternate_hash_encoding = fork_a_publication.clone();
        alternate_hash_encoding.hash_encoding = "rfc-8785-jcs".into();
        reseal_trace_checkpoint_publication_for_test(&mut alternate_hash_encoding);
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(&[
                fork_a_publication.clone(),
                alternate_hash_encoding,
            ])
            .is_ok(),
            "publication records from distinct hash-encoding namespaces must not be conflated as a fork"
        );

        let other_trace = state_machine_trace_capsule(33, 12);
        let other_trace =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&other_trace)
                .expect("independent trace capsule must deserialize");
        let other_publication = state_machine_trace_checkpoint_publication(
            &other_trace,
            12,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let independent_set = vec![fork_a_publication, other_publication];
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(&independent_set)
                .is_ok(),
            "independent trace identities must not be classified as a fork"
        );
        let mut reversed_independent_set = independent_set.clone();
        reversed_independent_set.reverse();
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(&reversed_independent_set)
                .is_ok(),
            "non-fork classification must not depend on publication collection order"
        );

        let witness_from_collection =
            find_state_machine_trace_publication_equivocation_witness(&[
                (&base, &base_publication),
                (&fork_a, &fork_a_publication),
                (&fork_b, &fork_b_publication),
            ])
            .expect("qualified fork collection must audit")
            .expect("qualified fork collection must emit a witness");
        assert_eq!(witness_from_collection, witness);

        let witness_from_reversed_collection =
            find_state_machine_trace_publication_equivocation_witness(&[
                (&fork_b, &fork_b_publication),
                (&base, &base_publication),
                (&fork_a, &fork_a_publication),
            ])
            .expect("reordered qualified fork collection must audit")
            .expect("reordered qualified fork collection must emit a witness");
        assert_eq!(
            witness_from_reversed_collection, witness,
            "witness extraction must be permutation-invariant"
        );

        let no_fork = find_state_machine_trace_publication_equivocation_witness(&[
            (&base, &base_publication),
            (&fork_a, &fork_a_publication),
        ])
        .expect("non-fork collection must audit");
        assert_eq!(no_fork, None);

        let mut invalid_publication = fork_a_publication.clone();
        invalid_publication.body_sha256 = "sha256:invalid-witness-source".into();
        assert_eq!(
            find_state_machine_trace_publication_equivocation_witness(&[
                (&base, &base_publication),
                (&fork_a, &invalid_publication),
                (&fork_b, &fork_b_publication),
            ]),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::BodyDigestMismatch
            )
        );

        let qualified_collection = vec![
            (&base, &base_publication),
            (&fork_a, &fork_a_publication),
        ];
        assert!(
            validate_state_machine_trace_checkpoint_publication_set_against_snapshots(
                &qualified_collection
            )
            .is_ok(),
            "qualified collection audit must validate concrete publications before fork detection"
        );
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_set_against_snapshots(&[]),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationCollectionEmpty
            )
        );

        let mut forged_collection_publication = fork_a_publication.clone();
        forged_collection_publication.body_sha256 = "sha256:forged-body".into();
        let forged_collection = vec![(&base, &base_publication), (&fork_a, &forged_collection_publication)];
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_set_against_snapshots(
                &forged_collection
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::BodyDigestMismatch
            )
        );

        let mut malformed_a = base_publication.clone();
        malformed_a.body_sha256 = "sha256:malformed-a".into();
        let mut malformed_b = base_publication.clone();
        malformed_b.body_sha256 = "sha256:malformed-b".into();
        let malformed_a_first = vec![(&base, &malformed_a), (&base, &malformed_b)];
        let malformed_b_first = vec![(&base, &malformed_b), (&base, &malformed_a)];
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_set_against_snapshots(
                &malformed_a_first
            ),
            validate_state_machine_trace_checkpoint_publication_set_against_snapshots(
                &malformed_b_first
            ),
            "qualified collection diagnostics must be permutation-invariant"
        );

        let complete_lineage = vec![(&base, &base_publication), (&fork_a, &fork_a_publication)];
        assert!(
            validate_state_machine_trace_checkpoint_publication_lineage(&complete_lineage)
                .is_ok(),
            "complete publication lineage must be rooted and fully reachable"
        );
        let mut reversed_complete_lineage = complete_lineage.clone();
        reversed_complete_lineage.reverse();
        assert!(
            validate_state_machine_trace_checkpoint_publication_lineage(
                &reversed_complete_lineage
            )
            .is_ok(),
            "complete publication lineage validation must be permutation-invariant"
        );

        let replayed_complete_lineage = vec![
            (&base, &base_publication),
            (&base, &base_publication),
            (&fork_a, &fork_a_publication),
            (&fork_a, &fork_a_publication),
        ];
        assert!(
            validate_state_machine_trace_checkpoint_publication_lineage(
                &replayed_complete_lineage
            )
            .is_ok(),
            "exact publication replays must not create duplicate lineage nodes or roots"
        );
        let mut reversed_replayed_complete_lineage = replayed_complete_lineage.clone();
        reversed_replayed_complete_lineage.reverse();
        assert!(
            validate_state_machine_trace_checkpoint_publication_lineage(
                &reversed_replayed_complete_lineage
            )
            .is_ok(),
            "exact publication replay handling must remain permutation-invariant"
        );

        let mut disconnected_suffix = fork_a_publication.clone();
        disconnected_suffix.previous_publication_sha256 =
            "sha256:uncollected-predecessor".into();
        reseal_trace_checkpoint_publication_for_test(&mut disconnected_suffix);
        let disconnected_collection =
            vec![(&base, &base_publication), (&fork_a, &disconnected_suffix)];
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_lineage(
                &disconnected_collection
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationLineageDisconnected
            )
        );

        let suffix_only = vec![(&fork_a, &fork_a_publication)];
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_lineage(&suffix_only),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationLineageNoRoot
            )
        );

        let mut semantically_divergent_later = state_machine_trace_capsule(31, 12);
        let mut divergent_publication = state_machine_trace_checkpoint_publication(
            &semantically_divergent_later,
            12,
            &base_publication.publication_sha256,
        );
        semantically_divergent_later
            .evidence[2]
            .token ^= 1;
        reseal_state_machine_trace_for_test(&mut semantically_divergent_later);
        divergent_publication.body_sha256 = semantically_divergent_later.integrity.body_sha256.clone();
        divergent_publication.chain_head_sha256 =
            semantically_divergent_later.evidence[11].chain_sha256.clone();
        reseal_trace_checkpoint_publication_for_test(&mut divergent_publication);
        let semantically_divergent_collection =
            vec![(&base, &base_publication), (&semantically_divergent_later, &divergent_publication)];
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_lineage(
                &semantically_divergent_collection
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotPrefixMismatch
            )
        );
        let mut reversed_semantically_divergent_collection =
            semantically_divergent_collection.clone();
        reversed_semantically_divergent_collection.reverse();
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_lineage(
                &reversed_semantically_divergent_collection
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotPrefixMismatch
            ),
            "invalid lineage diagnostics must be permutation-invariant"
        );

        let other_trace = state_machine_trace_capsule(99, 12);
        let other_trace =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&other_trace)
                .expect("independent trace capsule must deserialize");
        let cross_lineage_publication = state_machine_trace_checkpoint_publication(
            &other_trace,
            12,
            &base_publication.publication_sha256,
        );
        let cross_lineage_collection =
            vec![(&base, &base_publication), (&other_trace, &cross_lineage_publication)];
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_lineage(
                &cross_lineage_collection
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationLineageIdentityMismatch
            )
        );
    }

    #[test]
    fn state_machine_trace_checkpoint_publications_prove_append_only_prefix_consistency() {
        let earlier_text = state_machine_trace_capsule(31, 8);
        let later_text = state_machine_trace_capsule(31, 12);
        let earlier =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&earlier_text)
                .expect("earlier capsule must deserialize");
        let later =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&later_text)
                .expect("later capsule must deserialize");

        let earlier_publication = state_machine_trace_checkpoint_publication(
            &earlier,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let later_publication = state_machine_trace_checkpoint_publication(
            &later,
            12,
            &earlier_publication.publication_sha256,
        );

        assert!(
            validate_state_machine_trace_checkpoint_publication_chain(&[
                (&earlier, &earlier_publication),
                (&later, &later_publication),
            ])
            .is_ok()
        );

        let mut bad_publication_algorithm = later_publication.clone();
        bad_publication_algorithm.hash_algorithm = "sha-512".into();
        reseal_trace_checkpoint_publication_for_test(&mut bad_publication_algorithm);
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication(
                &later,
                &bad_publication_algorithm,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedPublicationHashAlgorithm
            )
        );

        let mut bad_publication_encoding = later_publication.clone();
        bad_publication_encoding.hash_encoding = "rfc-8785-jcs".into();
        reseal_trace_checkpoint_publication_for_test(&mut bad_publication_encoding);
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication(
                &later,
                &bad_publication_encoding,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedPublicationHashEncoding
            )
        );

        let mut forged_publication_schema = later_publication.clone();
        forged_publication_schema.trace_schema_version = 4;
        reseal_trace_checkpoint_publication_for_test(&mut forged_publication_schema);
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication(
                &later,
                &forged_publication_schema,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::TraceSchemaVersionMismatch
            )
        );

        let mut unsupported_later_schema = later.clone();
        unsupported_later_schema.schema_version += 1;
        reseal_state_machine_trace_for_test(&mut unsupported_later_schema);
        let unsupported_schema_publication = state_machine_trace_checkpoint_publication(
            &unsupported_later_schema,
            12,
            &earlier_publication.publication_sha256,
        );
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication(
                &unsupported_later_schema,
                &unsupported_schema_publication,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::UnsupportedTraceSchemaVersion
            )
        );

        let mut forged_evidence_later = later.clone();
        let previous_post_delivery_count = forged_evidence_later.evidence[2].post_delivery_count;
        forged_evidence_later.evidence[3].pre_delivery_count = previous_post_delivery_count + 1;
        reseal_state_machine_trace_for_test(&mut forged_evidence_later);
        let forged_evidence_publication = state_machine_trace_checkpoint_publication(
            &forged_evidence_later,
            12,
            &earlier_publication.publication_sha256,
        );
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication(
                &forged_evidence_later,
                &forged_evidence_publication,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::TraceEvidenceInvalid(
                    FederationStateMachineTraceEvidenceViolation::PreDeliveryContinuityMismatch
                )
            )
        );

        let mut orphaned_earlier_publication = earlier_publication.clone();
        orphaned_earlier_publication.previous_publication_sha256 =
            "sha256:unexpected-publication-root".into();
        orphaned_earlier_publication.publication_sha256 =
            state_machine_trace_checkpoint_publication_sha256(
                &orphaned_earlier_publication,
            );
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_chain(&[(
                &earlier,
                &orphaned_earlier_publication,
            )]),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::FirstPublicationPredecessorMismatch
            )
        );

        let mut forged_later = later.clone();
        forged_later.evidence[2].token ^= 1;
        reseal_state_machine_trace_for_test(&mut forged_later);
        let forged_publication = state_machine_trace_checkpoint_publication(
            &forged_later,
            12,
            &earlier_publication.publication_sha256,
        );
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_consistency(
                &earlier,
                &earlier_publication,
                &forged_later,
                &forged_publication,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotPrefixMismatch
            )
        );

        let later_same_endpoint_publication = state_machine_trace_checkpoint_publication(
            &later,
            8,
            &earlier_publication.publication_sha256,
        );
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_consistency(
                &earlier,
                &earlier_publication,
                &later,
                &later_same_endpoint_publication,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SameEndpointSnapshotMismatch
            )
        );

        let mut rollback_publication = state_machine_trace_checkpoint_publication(
            &later,
            6,
            &earlier_publication.publication_sha256,
        );
        rollback_publication.previous_publication_sha256 =
            earlier_publication.publication_sha256.clone();
        rollback_publication.publication_sha256 =
            state_machine_trace_checkpoint_publication_sha256(&rollback_publication);
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_consistency(
                &earlier,
                &earlier_publication,
                &later,
                &rollback_publication,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::SnapshotRollback
            )
        );
    }

    #[test]
    fn checkpoint_consistency_receipt_binds_both_publications_and_rejects_unknown_fields() {
        let earlier_text = state_machine_trace_capsule(34, 8);
        let later_text = state_machine_trace_capsule(34, 12);
        let earlier =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&earlier_text)
                .expect("earlier capsule must deserialize");
        let later =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&later_text)
                .expect("later capsule must deserialize");

        let earlier_publication = state_machine_trace_checkpoint_publication(
            &earlier,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let later_publication = state_machine_trace_checkpoint_publication(
            &later,
            12,
            &earlier_publication.publication_sha256,
        );
        let receipt = state_machine_trace_checkpoint_consistency_receipt(
            &earlier_publication,
            &later_publication,
        );

        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &receipt,
            ),
            Ok(())
        );

        let mut forged_digest = receipt.clone();
        forged_digest.later_publication_sha256 = earlier_publication.publication_sha256.clone();
        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &forged_digest,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptPublicationBindingMismatch
            )
        );

        let mut unknown = serde_json::to_value(&receipt)
            .expect("consistency receipt must serialize");
        unknown
            .as_object_mut()
            .expect("consistency receipt must serialize as an object")
            .insert("unexpected_consistency_field".into(), serde_json::Value::Bool(true));
        let unknown_json =
            serde_json::to_string_pretty(&unknown).expect("unknown-field JSON must serialize");
        assert!(
            serde_json::from_str::<FederationStateMachineTraceCheckpointConsistencyReceipt>(
                &unknown_json
            )
            .is_err()
        );
    }

    #[test]
    fn checkpoint_consistency_receipt_is_deterministic_and_typed() {
        let earlier_text = state_machine_trace_capsule(35, 8);
        let later_text = state_machine_trace_capsule(35, 12);
        let earlier =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&earlier_text)
                .expect("earlier capsule must deserialize");
        let later =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&later_text)
                .expect("later capsule must deserialize");
        let earlier_publication = state_machine_trace_checkpoint_publication(
            &earlier,
            8,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        let later_publication = state_machine_trace_checkpoint_publication(
            &later,
            12,
            &earlier_publication.publication_sha256,
        );
        let receipt = state_machine_trace_checkpoint_consistency_receipt(
            &earlier_publication,
            &later_publication,
        );

        let json = serde_json::to_string_pretty(&receipt)
            .expect("consistency receipt must serialize");
        let round_trip =
            serde_json::from_str::<FederationStateMachineTraceCheckpointConsistencyReceipt>(
                &json,
            )
            .expect("consistency receipt must deserialize");
        assert_eq!(round_trip, receipt);
        assert_eq!(
            json,
            serde_json::to_string_pretty(&round_trip)
                .expect("consistency receipt serialization must be deterministic")
        );

        let mut bad_schema = receipt.clone();
        bad_schema.schema_version += 1;
        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &bad_schema,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptSchemaMismatch
            )
        );

        let mut bad_profile = receipt.clone();
        bad_profile.receipt_profile = "future-consistency-receipt-profile".into();
        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &bad_profile,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptProfileMismatch
            )
        );

        let mut bad_proof_type = receipt.clone();
        bad_proof_type.proof_type = "inclusion-v1".into();
        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &bad_proof_type,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptProofTypeMismatch
            )
        );

        let mut bad_algorithm = receipt.clone();
        bad_algorithm.hash_algorithm = "sha-512".into();
        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &bad_algorithm,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptHashAlgorithmMismatch
            )
        );

        let mut bad_encoding = receipt.clone();
        bad_encoding.hash_encoding = "rfc-8785-jcs".into();
        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &bad_encoding,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptHashEncodingMismatch
            )
        );

        let mut bad_digest = receipt;
        bad_digest.receipt_sha256 = "sha256:invalid-consistency-receipt-digest".into();
        assert_eq!(
            validate_state_machine_trace_checkpoint_consistency_receipt(
                &earlier,
                &earlier_publication,
                &later,
                &later_publication,
                &bad_digest,
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::ConsistencyReceiptDigestMismatch
            )
        );
    }

    #[test]
    fn empty_publication_chain_is_rejected() {
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_chain(&[]),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationChainEmpty
            )
        );
    }

    #[test]
    fn state_machine_trace_hash_purposes_are_domain_separated() {
        let capsule_text = state_machine_trace_capsule(21, 6);
        let capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        let evidence = capsule.evidence.first().expect("trace must contain evidence");

        let hash_domains = [
            FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_CONSISTENCY_RECEIPT_HASH_DOMAIN,
        ];
        let unique_hash_domains = hash_domains.iter().copied().collect::<BTreeSet<_>>();
        assert_eq!(
            unique_hash_domains.len(),
            hash_domains.len(),
            "every trace/checkpoint artifact must use a distinct hash domain"
        );
        assert!(
            hash_domains
                .iter()
                .all(|domain| domain.bytes().all(|byte| byte != 0)),
            "hash domains must not contain the delimiter byte"
        );

        let sample = serde_json::to_vec(evidence)
            .expect("evidence sample must serialize");
        let body_domain_digest = state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN,
            &sample,
        );
        let chain_domain_digest = state_machine_domain_separated_sha256(
            FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN,
            &sample,
        );
        assert_ne!(
            body_domain_digest, chain_domain_digest,
            "distinct hash purposes must produce distinct domain-separated invocations"
        );

        let chain_digest = state_machine_evidence_chain_sha256(evidence);
        assert_ne!(
            capsule.integrity.body_sha256, chain_digest,
            "distinct hash purposes must not share an undifferentiated digest domain"
        );
    }

    #[test]
    fn state_machine_trace_evidence_verifier_is_replay_independent_and_rejects_resealed_boundary_forgery() {
        let capsule_text = state_machine_trace_capsule(18, 10);
        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");

        assert!(
            validate_state_machine_trace_evidence(&capsule).is_ok(),
            "generated capsule must pass the replay-independent evidence verifier"
        );

        let mut unsupported_profile =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        unsupported_profile.verification_profile = "future-profile".into();
        reseal_state_machine_trace_for_test(&mut unsupported_profile);
        let error = validate_state_machine_trace_evidence(&unsupported_profile)
            .expect_err("re-sealed unsupported verification profile must be rejected");
        assert_eq!(
            error,
            FederationStateMachineTraceEvidenceViolation::UnsupportedVerificationProfile
        );

        let previous_post_delivery_count = capsule.evidence[2].post_delivery_count;
        capsule.evidence[3].pre_delivery_count = previous_post_delivery_count + 1;
        reseal_state_machine_trace_for_test(&mut capsule);

        let error = validate_state_machine_trace_evidence(&capsule)
            .expect_err("re-sealed delivery-count forgery must be rejected");
        assert_eq!(
            error,
            FederationStateMachineTraceEvidenceViolation::PreDeliveryContinuityMismatch
        );

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        let admitted_steps = capsule
            .evidence
            .iter()
            .enumerate()
            .filter(|(_, step)| !step.newly_admitted_deliveries.is_empty())
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        assert!(
            admitted_steps.len() >= 2,
            "test trace must contain two admitted-delivery steps"
        );
        let source_index = admitted_steps[0];
        let duplicate_index = admitted_steps[1];
        let reused_identity = capsule.evidence[source_index]
            .newly_admitted_deliveries[0]
            .0
            .clone();
        capsule.evidence[duplicate_index].newly_admitted_deliveries[0].0 = reused_identity;
        reseal_state_machine_trace_for_test(&mut capsule);
        let error = validate_state_machine_trace_evidence(&capsule)
            .expect_err("re-sealed duplicate delivery identity must be rejected");
        assert_eq!(
            error,
            FederationStateMachineTraceEvidenceViolation::DuplicateAdmittedDeliveryIdentity
        );

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        let empty_index = capsule.evidence.iter().position(|step| !step.newly_admitted_deliveries.is_empty())
            .expect("test trace must contain an admitted delivery");
        capsule.evidence[empty_index].newly_admitted_deliveries[0].0.clear();
        reseal_state_machine_trace_for_test(&mut capsule);
        let error = validate_state_machine_trace_evidence(&capsule)
            .expect_err("re-sealed empty delivery identity must be rejected");
        assert_eq!(
            error,
            FederationStateMachineTraceEvidenceViolation::EmptyAdmittedDeliveryIdentity
        );

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        let previous_post_admission_index = capsule.evidence[2].post_admission_index;
        capsule.evidence[3].pre_admission_index = previous_post_admission_index + 1;
        capsule.evidence[3].post_admission_index = previous_post_admission_index + 1;
        capsule.evidence[3].newly_admitted_deliveries.clear();
        reseal_state_machine_trace_for_test(&mut capsule);

        let error = validate_state_machine_trace_evidence(&capsule)
            .expect_err("re-sealed admission-index forgery must be rejected");
        assert_eq!(
            error,
            FederationStateMachineTraceEvidenceViolation::PreAdmissionContinuityMismatch
        );
    }

    #[test]
    fn state_machine_trace_capsule_rejects_schema_boundary_and_temporal_tampering() {
        let capsule_text = state_machine_trace_capsule(9, 10);

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.schema_version += 1;
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.evidence[2].step_index ^= 1;
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.evidence[2].pre_state_fingerprint[0] ^= 1;
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.evidence[2].pre_admission_index ^= 1;
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.evidence[4].newly_admitted_deliveries =
            vec![("tampered-delivery".into(), 0)];
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.final_state.admission_index ^= 1;
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());

        let mut capsule =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
                .expect("capsule must deserialize");
        capsule.integrity.chain_head_sha256 =
            FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS.into();
        assert!(std::panic::catch_unwind(|| state_machine_plan_from_capsule(&capsule)).is_err());
    }

    #[test]
    fn state_machine_trace_capsule_is_a_self_validating_evidence_artifact() {
        let capsule_text = state_machine_trace_capsule(3, 8);
        assert!(capsule_text.contains("\"schema_version\": 5"));
        assert!(capsule_text.contains(
            "\"verification_profile\": \"integral-federation-state-machine-trace-v1\""
        ));
        assert!(capsule_text.contains("\"trace_index\": 3"));
        assert!(capsule_text.contains("\"initial_seed\":"));
        assert!(capsule_text.contains("\"operations\": ["));
        assert!(capsule_text.contains("\"tokens\": ["));
        assert!(capsule_text.contains("\"initial_state\": {"));
        assert!(capsule_text.contains("\"evidence\": ["));
        assert!(capsule_text.contains("\"final_state\": {"));
        assert!(capsule_text.contains("\"integrity\": {"));
        assert!(capsule_text.contains("\"body_sha256\": \"sha256:"));
        assert!(capsule_text.contains("\"chain_head_sha256\": \"sha256:"));

        let capsule = serde_json::from_str::<FederationStateMachineTraceCapsule>(&capsule_text)
            .expect("capsule must remain self-describing");
        let capsule_plan = state_machine_plan_from_capsule(&capsule);
        let replay = run_state_machine_trace_plan(&capsule_plan);

        assert_eq!(replay.len(), 8);
        assert_eq!(replay, capsule.evidence);
        assert_eq!(
            capsule.initial_state,
            FederationStateMachineTraceBoundary {
                admission_index: 0,
                delivery_count: 0,
                state_fingerprint: canonical_state_fingerprint(&nodes()),
            }
        );
        assert_eq!(
            capsule.integrity.chain_head_sha256,
            capsule
                .evidence
                .last()
                .map(|evidence| evidence.chain_sha256.as_str())
                .unwrap_or(FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS),
            "successful trace capsule chain head must match its final evidence"
        );
        assert_eq!(
            capsule.final_state,
            FederationStateMachineTraceBoundary {
                admission_index: replay.last().unwrap().post_admission_index,
                delivery_count: replay.last().unwrap().post_delivery_count,
                state_fingerprint: replay.last().unwrap().state_fingerprint.clone(),
            }
        );
        assert!(
            replay.iter().enumerate().all(|(index, step)| {
                step.step_index == index
                    && !step.pre_state_fingerprint.is_empty()
                    && !step.state_fingerprint.is_empty()
                    && !step.chain_sha256.is_empty()
            }),
            "successful trace evidence must carry explicit step, state-boundary, and chain identities"
        );
        assert_eq!(
            replay.first().map(|step| step.chain_prev_sha256.as_str()),
            Some(FEDERATION_STATE_MACHINE_TRACE_CHAIN_GENESIS),
            "successful trace evidence must begin at the declared chain genesis"
        );
    }

    #[test]
    fn state_machine_trace_capsule_is_deterministic_across_repeated_generation() {
        assert_eq!(
            state_machine_trace_capsule(31, 24),
            state_machine_trace_capsule(31, 24)
        );
    }

    #[test]
    fn state_machine_failure_capsule_round_trips_and_is_deterministic() {
        let (_, plan) = state_machine_trace_plan(11, 6);
        let audit = FEDERATION_INVARIANT_REGISTRY
            .iter()
            .map(|spec| FederationInvariantAuditEntry {
                id: spec.id,
                status: if spec.id == FederationInvariantId::SourceObservationBijection {
                    FederationInvariantAuditStatus::Violated(
                        FederationInvariantViolation::SourceObservationSetMismatch,
                    )
                } else {
                    FederationInvariantAuditStatus::Passed
                },
            })
            .collect::<Vec<_>>();
        let capsule = FederationStateMachineFailureCapsule {
            schema_version: 2,
            failure_kind: "invariant-violation".into(),
            trace_prefix: plan.clone(),
            failed_step_index: 5,
            operation: plan[5].0,
            token: plan[5].1,
            expected_state_valid: true,
            observed_state_valid: false,
            observed_violations: vec![(
                FederationInvariantId::SourceObservationBijection,
                FederationInvariantViolation::SourceObservationSetMismatch,
            )],
            audit,
            expected_decision: Some(FederationDecision::AcceptedLocal),
            observed_decision: Some(FederationDecision::AcceptedLocal),
            expected_authority: Some(AuthorityDisposition::LocalAuthority),
            observed_authority: Some(AuthorityDisposition::LocalAuthority),
            pre_state_fingerprint: b"pre-state".to_vec(),
            post_state_fingerprint: b"post-state".to_vec(),
            pre_admission_index: 5,
            post_admission_index: 6,
            pre_delivery_count: 5,
            post_delivery_count: 6,
            newly_admitted_deliveries: vec![("delivery-evidence".into(), 5)],
        };

        let json = capsule.to_json();
        let round_trip =
            serde_json::from_str::<FederationStateMachineFailureCapsule>(&json)
                .expect("failure capsule must deserialize");
        assert_eq!(round_trip, capsule);
        assert_eq!(round_trip.validate(), Ok(()));

        let mut invalid_state_flags = capsule.clone();
        invalid_state_flags.expected_state_valid = false;
        assert_eq!(
            invalid_state_flags.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::StateValidityFlagsMismatch)
        );

        let mut truncated_audit = capsule.clone();
        truncated_audit.audit.pop();
        assert_eq!(
            truncated_audit.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::AuditRegistryShapeMismatch)
        );
        let mut reordered_audit = capsule.clone();
        reordered_audit.audit.swap(0, 1);
        assert_eq!(
            reordered_audit.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::AuditRegistryShapeMismatch)
        );

        let mut unsupported_schema = capsule.clone();
        unsupported_schema.schema_version += 1;
        assert_eq!(
            unsupported_schema.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::UnsupportedSchemaVersion)
        );

        let mut unsupported_kind = capsule.clone();
        unsupported_kind.failure_kind = "future-kind".into();
        assert_eq!(
            unsupported_kind.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::UnsupportedFailureKind)
        );

        let mut mismatched_step = capsule.clone();
        mismatched_step.failed_step_index -= 1;
        assert_eq!(
            mismatched_step.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::FailedStepIndexMismatch)
        );

        let mut mismatched_identity = capsule.clone();
        mismatched_identity.token ^= 1;
        assert_eq!(
            mismatched_identity.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::FailedStepIdentityMismatch)
        );

        let mut mismatched_audit = capsule.clone();
        mismatched_audit.observed_violations.clear();
        assert_eq!(
            mismatched_audit.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::ObservedViolationAuditMismatch)
        );

        let mut truncated_audit = capsule.clone();
        truncated_audit.audit.pop();
        assert_eq!(
            truncated_audit.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::AuditRegistryMismatch)
        );

        let mut mismatched_flags = capsule.clone();
        mismatched_flags.observed_state_valid = true;
        assert_eq!(
            mismatched_flags.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::InvalidStateValidityFlags)
        );

        let mut mismatched_temporal = capsule.clone();
        mismatched_temporal.post_admission_index += 1;
        assert_eq!(
            mismatched_temporal.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::TemporalEvidenceMismatch)
        );

        let mut mismatched_ordinal = capsule.clone();
        mismatched_ordinal.newly_admitted_deliveries[0].1 += 1;
        assert_eq!(
            mismatched_ordinal.validate(),
            Err(FederationStateMachineFailureCapsuleViolation::TemporalEvidenceMismatch)
        );

        let mut unknown = serde_json::to_value(&capsule)
            .expect("failure capsule must serialize as a JSON value");
        unknown
            .as_object_mut()
            .expect("failure capsule must serialize as an object")
            .insert("unexpected_failure_field".into(), serde_json::Value::Bool(true));
        assert!(
            serde_json::from_value::<FederationStateMachineFailureCapsule>(unknown).is_err(),
            "failure capsule must reject unknown top-level fields"
        );

        assert_eq!(round_trip.failure_kind, "invariant-violation");
        assert_eq!(json, round_trip.to_json());
        assert_eq!(
            round_trip.trace_prefix.len(),
            round_trip.failed_step_index + 1
        );
        assert_eq!(
            round_trip.trace_prefix[round_trip.failed_step_index],
            (round_trip.operation, round_trip.token)
        );
    }

    #[test]
    fn bounded_state_machine_corpus_preserves_invariants_and_replays_exactly() {
        assert!(FederationStateMachineOperation::ALL.len() >= 16);

        let trace_count = 64;
        let steps_per_trace = 64;
        let mut exercised = BTreeSet::new();

        for trace_index in 0..trace_count {
            let first = run_state_machine_trace(trace_index, steps_per_trace);
            let second = run_state_machine_trace(trace_index, steps_per_trace);

            assert_eq!(
                first, second,
                "state-machine trace {trace_index} was not deterministic"
            );

            for step in &first {
                exercised.insert(step.operation);
                assert!(
                    !step.state_fingerprint.is_empty(),
                    "state-machine trace emitted an empty fingerprint"
                );
                assert!(
                    step.post_admission_index >= step.pre_admission_index,
                    "state-machine evidence regressed the admission counter"
                );
                assert!(
                    step.post_delivery_count >= step.pre_delivery_count,
                    "state-machine evidence regressed admitted-delivery count"
                );
                let admission_delta = step.post_admission_index - step.pre_admission_index;
                let delivery_delta = (step.post_delivery_count - step.pre_delivery_count) as u64;
                assert_eq!(admission_delta, delivery_delta);
                assert_eq!(step.newly_admitted_deliveries.len() as u64, admission_delta);
                assert_eq!(
                    step.newly_admitted_deliveries
                        .iter()
                        .map(|(_, index)| *index)
                        .collect::<BTreeSet<_>>(),
                    (step.pre_admission_index..step.post_admission_index)
                        .collect::<BTreeSet<_>>()
                );
                if let Some(authority) = step.authority {
                    if matches!(
                        step.decision,
                        Some(
                            FederationDecision::AcceptedLocal
                                | FederationDecision::AcceptedForeign
                                | FederationDecision::AcceptedDelegatedForeign
                                | FederationDecision::Duplicate
                        )
                    ) {
                        assert_ne!(
                            authority,
                            AuthorityDisposition::NoAuthority,
                            "accepted/replay step lost authority: {:?}",
                            step.operation
                        );
                    } else {
                        assert_eq!(
                            authority,
                            AuthorityDisposition::NoAuthority,
                            "rejected step carried authority: {:?}",
                            step.operation
                        );
                    }
                }
            }
        }

        assert_eq!(
            exercised.len(),
            FederationStateMachineOperation::ALL.len(),
            "bounded corpus did not exercise every state-machine operation"
        );
        for operation in FederationStateMachineOperation::ALL {
            assert!(
                exercised.contains(operation),
                "state-machine operation was never exercised: {:?}",
                operation
            );
        }
    }

    #[test]
    fn source_observations_track_admitted_deliveries() {
        let mut state = nodes();
        assert_eq!(source_observation_count(&state), 0);
        assert_eq!(state.delivery_count(), 0);

        let local = envelope();
        assert_eq!(
            deliver(&mut state, &local, 50, true).decision,
            FederationDecision::AcceptedLocal
        );
        assert_eq!(source_observation_count(&state), 1);
        assert_eq!(state.delivery_count(), 1);

        let mut foreign = envelope();
        foreign.envelope_id = "env-source-foreign".into();
        foreign.logical_delivery_id = "delivery-source-foreign".into();
        foreign.attempt_id = "attempt-source-foreign".into();
        foreign.origin_node = "node-b".into();
        foreign.target_node = "node-a".into();
        assert_eq!(
            deliver(&mut state, &foreign, 50, true).decision,
            FederationDecision::AcceptedForeign
        );
        assert_eq!(source_observation_count(&state), 2);
        assert_eq!(state.delivery_count(), 2);

        let independent = ObservationRecord {
            observation_id: "obs-non-source".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:independent".into(),
            origin_node: "node-a".into(),
            origin_node_known: false,
            recognized_by: None,
            source_observation: false,
        };
        assert_eq!(
            record_observation(&mut state, independent),
            ObservationWriteResult::Inserted
        );
        assert_eq!(source_observation_count(&state), 2);
        assert_eq!(state.observation_count(), 3);
    }

    #[test]
    fn every_admitted_delivery_binds_exactly_one_matching_source_observation() {
        let mut state = nodes();

        let local_outcome = deliver(&mut state, &envelope(), 50, true);
        assert_eq!(local_outcome.decision(), FederationDecision::AcceptedLocal);

        let mut recognized = envelope();
        recognized.envelope_id = "env-recognized-source".into();
        recognized.logical_delivery_id = "delivery-recognized-source".into();
        recognized.attempt_id = "attempt-recognized-source".into();
        recognized.origin_node = "node-b".into();
        recognized.target_node = "node-a".into();
        state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        });
        let recognized_outcome = deliver(&mut state, &recognized, 50, true);
        assert_eq!(recognized_outcome.decision(), FederationDecision::AcceptedForeign);

        let mut delegated = envelope();
        delegated.envelope_id = "env-delegated-source".into();
        delegated.logical_delivery_id = "delivery-delegated-source".into();
        delegated.attempt_id = "attempt-delegated-source".into();
        delegated.origin_node = "node-c".into();
        delegated.target_node = "node-a".into();
        state.add_recognition(RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-c".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::DelegatedAuthority,
        });
        let delegated_outcome = deliver(&mut state, &delegated, 50, true);
        assert_eq!(
            delegated_outcome.decision(),
            FederationDecision::AcceptedDelegatedForeign
        );

        assert_eq!(source_observation_count(&state), state.delivery_count());
        assert!(source_observation_ids_match_delivery_links(&state));
        for record in state.deliveries.values() {
            assert!(source_observation_matches_delivery(&state, record));
            assert!(attempt_history_matches_bindings(record));
        }
    }

    #[test]
    fn admitted_state_satisfies_all_cross_ledger_invariants() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert!(federation_state_invariants_hold(&state));

        let mut second = envelope();
        second.envelope_id = "env-second".into();
        second.logical_delivery_id = "delivery-second".into();
        second.attempt_id = "attempt-second".into();
        assert_eq!(
            deliver(&mut state, &second, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert!(federation_state_invariants_hold(&state));
    }

    #[test]
    fn cross_ledger_invariant_detects_map_key_identity_corruption() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let record = state.deliveries.remove("delivery-1").unwrap();
        state
            .deliveries
            .insert("wrong-delivery-key".into(), record);
        assert!(!federation_state_invariants_hold(&state));

        let mut observation = state.observations.remove("env-1").unwrap();
        observation.observation_id = "env-1".into();
        state
            .observations
            .insert("wrong-observation-key".into(), observation);
        assert!(!federation_state_invariants_hold(&state));
    }

    #[test]
    fn source_observation_bijection_detects_orphaned_source_records() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert!(source_observation_ids_match_delivery_links(&state));

        state.observations.insert(
            "orphan-source".into(),
            ObservationRecord {
                observation_id: "orphan-source".into(),
                semantic_subject_id: "subject-1".into(),
                payload_commitment: "sha256:orphan".into(),
                origin_node: "node-a".into(),
                origin_node_known: true,
                recognized_by: None,
                source_observation: true,
            },
        );

        assert!(!source_observation_ids_match_delivery_links(&state));
    }

    #[test]
    fn source_observation_bijection_rejects_duplicate_delivery_links() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let source_id = state
            .delivery("delivery-1")
            .unwrap()
            .source_observation_id()
            .to_owned();
        assert_eq!(
            state
                .delivery("delivery-1")
                .unwrap()
                .source_observation_recognized_by(),
            None
        );
        assert_eq!(
            state.delivery("delivery-1").unwrap().attempt_envelope_id("attempt-1"),
            Some("env-1")
        );

        let duplicate_record = state.delivery("delivery-1").unwrap().clone();
        state
            .deliveries
            .insert("delivery-duplicate-link".into(), duplicate_record);

        assert_eq!(
            state
                .delivery("delivery-duplicate-link")
                .unwrap()
                .source_observation_id(),
            source_id
        );
        assert!(!source_observation_ids_match_delivery_links(&state));
    }

    #[test]
    fn cross_ledger_provenance_invariant_detects_observation_rebinding() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let source_id = state
            .delivery("delivery-1")
            .unwrap()
            .source_observation_id()
            .to_owned();
        let observation = state.observations.get_mut(&source_id).unwrap();
        observation.payload_commitment = "sha256:tampered".into();

        let record = state.delivery("delivery-1").unwrap();
        assert!(!source_observation_matches_delivery(&state, record));
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
    fn semantic_input_types_reject_unknown_fields() {
        let node = NodeProfile {
            node_id: "node-a".into(),
            schema_generation: 1,
            authorization_generation: 1,
            active: true,
        };
        let mut node_json = serde_json::to_value(&node).expect("node profile must serialize");
        node_json
            .as_object_mut()
            .expect("node profile must serialize as object")
            .insert("unexpected".into(), serde_json::Value::Bool(true));
        assert!(serde_json::from_value::<NodeProfile>(node_json).is_err());

        let edge = RecognitionEdge {
            recognizing_node: "node-a".into(),
            origin_node: "node-b".into(),
            scope: "subject-1".into(),
            mode: RecognitionMode::EvidenceOnly,
        };
        let mut edge_json = serde_json::to_value(&edge).expect("recognition edge must serialize");
        edge_json
            .as_object_mut()
            .expect("recognition edge must serialize as object")
            .insert("unexpected".into(), serde_json::Value::Bool(true));
        assert!(serde_json::from_value::<RecognitionEdge>(edge_json).is_err());

        let envelope = envelope();
        let mut envelope_json =
            serde_json::to_value(&envelope).expect("federation envelope must serialize");
        envelope_json
            .as_object_mut()
            .expect("federation envelope must serialize as object")
            .insert("unexpected".into(), serde_json::Value::Bool(true));
        assert!(serde_json::from_value::<FederationEnvelope>(envelope_json).is_err());

        let observation = ObservationRecord {
            observation_id: "observation-1".into(),
            semantic_subject_id: "subject-1".into(),
            payload_commitment: "sha256:payload".into(),
            origin_node: "node-a".into(),
            origin_node_known: true,
            recognized_by: None,
            source_observation: false,
        };
        let mut observation_json =
            serde_json::to_value(&observation).expect("observation must serialize");
        observation_json
            .as_object_mut()
            .expect("observation must serialize as object")
            .insert("unexpected".into(), serde_json::Value::Bool(true));
        assert!(serde_json::from_value::<ObservationRecord>(observation_json).is_err());

        let audit_entry = FederationInvariantAuditEntry {
            id: FederationInvariantId::NodeMapIdentity,
            status: FederationInvariantAuditStatus::Passed,
        };
        let mut audit_json =
            serde_json::to_value(&audit_entry).expect("audit entry must serialize");
        audit_json
            .as_object_mut()
            .expect("audit entry must serialize as object")
            .insert("unexpected".into(), serde_json::Value::Bool(true));
        assert!(serde_json::from_value::<FederationInvariantAuditEntry>(audit_json).is_err());
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
            recognizing_node: "node-a".into(),            origin_node: "node-b".into(),
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
        assert_eq!(
            source_observation_matches_delivery(
                &state,
                state.delivery("delivery-recognition-drift").unwrap()
            ),
            true
        );
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
        assert_eq!(
            source_observation_matches_delivery(
                &state,
                state.delivery("delivery-recognition-drift").unwrap()
            ),
            true
        );
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
        assert!(attempt_history_matches_bindings(
            state.delivery("delivery-1").unwrap()
        ));
    }

    #[test]
    fn attempt_history_bijection_detects_orphan_bindings() {
        let mut state = nodes();
        assert_eq!(
            deliver(&mut state, &envelope(), 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let mut corrupted = state.delivery("delivery-1").unwrap().clone();
        corrupted
            .attempt_envelope_ids
            .insert("orphan-attempt".into(), "orphan-envelope".into());

        assert!(!attempt_history_matches_bindings(&corrupted));
    }

    #[test]
    fn new_attempt_can_change_envelope_without_minting_a_second_source_observation() {
        let mut state = nodes();
        let original = envelope();

        assert_eq!(
            deliver(&mut state, &original, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        let source_id = state
            .delivery("delivery-1")
            .unwrap()
            .source_observation_id()
            .to_owned();

        let mut retry = original.clone();
        retry.envelope_id = "env-retry".into();
        retry.attempt_id = "attempt-retry".into();

        assert_eq!(
            deliver(&mut state, &retry, 50, true).decision(),
            FederationDecision::Duplicate
        );
        assert_eq!(
            state.delivery("delivery-1").unwrap().source_observation_id(),
            source_id
        );
        assert_eq!(state.observation_count(), 1);
        assert_eq!(source_observation_count(&state), 1);
        assert!(attempt_history_matches_bindings(
            state.delivery("delivery-1").unwrap()
        ));
        assert_eq!(
            state.delivery("delivery-1").unwrap().attempts(),
            &BTreeSet::from([
                "attempt-1".to_owned(),
                "attempt-retry".to_owned()
            ])
        );
        assert_eq!(
            state.delivery("delivery-1").unwrap().attempt_envelope_id("attempt-retry"),
            Some("env-retry")
        );
    }

    #[test]
    fn attempt_identity_is_scoped_to_its_logical_delivery() {
        let mut state = nodes();

        let first = envelope();
        assert_eq!(
            deliver(&mut state, &first, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let mut second = envelope();
        second.envelope_id = "env-second-attempt-scope".into();
        second.logical_delivery_id = "delivery-second-attempt-scope".into();
        // Deliberately reuse the attempt ID across distinct logical deliveries.
        second.attempt_id = first.attempt_id.clone();

        assert_eq!(
            deliver(&mut state, &second, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );
        assert_eq!(state.delivery_count(), 2);
        assert_eq!(state.observation_count(), 2);
        assert!(attempt_history_matches_bindings(
            state.delivery("delivery-1").unwrap()
        ));
        assert!(attempt_history_matches_bindings(
            state.delivery("delivery-second-attempt-scope").unwrap()
        ));
        assert_eq!(
            state
                .delivery("delivery-1")
                .unwrap()
                .attempt_envelope_ids
                .get(&first.attempt_id)
                .map(String::as_str),
            Some(first.envelope_id.as_str())
        );
        assert_eq!(
            state
                .delivery("delivery-second-attempt-scope")
                .unwrap()
                .attempt_envelope_ids
                .get(&second.attempt_id)
                .map(String::as_str),
            Some(second.envelope_id.as_str())
        );
    }

    #[test]
    fn attempt_identity_cannot_rebind_to_a_different_envelope() {
        let mut state = nodes();
        let original = envelope();
        assert_eq!(
            deliver(&mut state, &original, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let before_identity = delivery_identity_snapshot(&state);
        let before_attempts = delivery_attempts_snapshot(&state);
        let before_observations = state.observations.clone();

        let mut rebound = original.clone();
        rebound.envelope_id = "env-rebound".into();

        let outcome = deliver(&mut state, &rebound, 50, true);
        assert_eq!(outcome.decision(), FederationDecision::AttemptConflict);
        assert_eq!(outcome.authority(), AuthorityDisposition::NoAuthority);
        assert_eq!(delivery_identity_snapshot(&state), before_identity);
        assert_eq!(delivery_attempts_snapshot(&state), before_attempts);
        assert_eq!(state.observations, before_observations);
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
    fn envelope_identity_cannot_be_reused_for_a_different_logical_delivery() {
        let mut state = nodes();
        let first = envelope();

        assert_eq!(
            deliver(&mut state, &first, 50, true).decision(),
            FederationDecision::AcceptedLocal
        );

        let mut aliased = first.clone();
        aliased.logical_delivery_id = "delivery-aliased".into();
        aliased.attempt_id = "attempt-aliased".into();

        let before_identity = delivery_identity_snapshot(&state);
        let before_attempts = delivery_attempts_snapshot(&state);
        let before_observations = state.observations.clone();

        let outcome = deliver(&mut state, &aliased, 50, true);
        assert_eq!(outcome.decision(), FederationDecision::Rejected);
        assert_eq!(outcome.authority(), AuthorityDisposition::NoAuthority);
        assert_eq!(delivery_identity_snapshot(&state), before_identity);
        assert_eq!(delivery_attempts_snapshot(&state), before_attempts);
        assert_eq!(state.observations, before_observations);
        assert_eq!(state.delivery_count(), 1);
        assert_eq!(state.observation_count(), 1);
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
            outcome.reason(),            "Delivery admitted without rewriting origin or minting local authority."
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
        assert!(partitioned.origin_node_known);
        assert_eq!(partitioned.origin_node.as_deref(), Some("node-a"));
        assert!(cockpit_projection("node-a", &partitioned).origin_node_known);

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
    fn partition_preserves_independent_origin_node_knowledge() {
        let mut state = nodes();

        let known = envelope();
        let known_outcome = deliver(&mut state, &known, 50, false);
        assert_eq!(known_outcome.decision, FederationDecision::PartitionUnknown);
        assert!(!known_outcome.authority().eq(&AuthorityDisposition::LocalAuthority));
        assert_eq!(known_outcome.origin_node(), Some("node-a"));
        assert!(known_outcome.origin_node_known());

        let mut unknown = envelope();
        unknown.origin_node = "node-unknown".into();
        let unknown_outcome = deliver(&mut state, &unknown, 50, false);
        assert_eq!(unknown_outcome.decision, FederationDecision::PartitionUnknown);
        assert!(!unknown_outcome.origin_node_known());
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
                .delivery("delivery-1")                .unwrap()
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
        assert!(partition_projection.origin_node_known);
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
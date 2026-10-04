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

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
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
                schema_version: 2,
                failure_kind: "invariant-violation".into(),
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

        fn to_json(&self) -> String {
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
    const FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS: &str =
        "sha256:checkpoint-publication-genesis-v1:0000000000000000000000000000000000000000000000000000000000000000";

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FederationStateMachineTraceCheckpointPublication {
        schema_version: u16,
        publication_profile: String,
        trace_verification_profile: String,
        trace_index: usize,
        evidence_end: usize,
        body_sha256: String,
        chain_head_sha256: String,
        previous_publication_sha256: String,
        publication_sha256: String,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Serialize)]
    struct FederationStateMachineTraceCheckpointPublicationHashView {
        hash_domain: String,
        schema_version: u16,
        publication_profile: String,
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
        TraceVerificationProfileMismatch,
        TraceIndexMismatch,
        InvalidEvidenceEnd,
        BodyDigestMismatch,
        ChainHeadMismatch,
        PublicationDigestMismatch,
        FirstPublicationPredecessorMismatch,
        SnapshotProfileMismatch,
        SnapshotTraceIndexMismatch,
        SnapshotPrefixMismatch,
        SnapshotRollback,
        PreviousPublicationMismatch,
        PublicationChainEmpty,
        PublicationForkDetected,
    }

    fn validate_state_machine_trace_checkpoint_publication_set(
        publications: &[FederationStateMachineTraceCheckpointPublication],
    ) -> Result<(), FederationStateMachineTraceCheckpointPublicationViolation> {
        let mut successors =
            BTreeMap::<(&str, &str, usize, &str), &str>::new();

        for publication in publications {
            let key = (
                publication.publication_profile.as_str(),
                publication.trace_verification_profile.as_str(),
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
        if publication.trace_verification_profile != capsule.verification_profile {
            return Err(
                FederationStateMachineTraceCheckpointPublicationViolation::TraceVerificationProfileMismatch
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
        assert_eq!(
            validate_state_machine_trace_checkpoint_publication_set(
                &[
                    base_publication.clone(),
                    fork_a_publication.clone(),
                    fork_b_publication.clone(),
                ]
            ),
            Err(
                FederationStateMachineTraceCheckpointPublicationViolation::PublicationForkDetected
            )
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

        let other_trace = state_machine_trace_capsule(33, 12);
        let other_trace =
            serde_json::from_str::<FederationStateMachineTraceCapsule>(&other_trace)
                .expect("independent trace capsule must deserialize");
        let other_publication = state_machine_trace_checkpoint_publication(
            &other_trace,
            12,
            FEDERATION_STATE_MACHINE_TRACE_CHECKPOINT_PUBLICATION_GENESIS,
        );
        assert!(
            validate_state_machine_trace_checkpoint_publication_set(
                &[fork_a_publication, other_publication]
            )
            .is_ok(),
            "independent trace identities must not be classified as a fork"
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

        assert_ne!(
            FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN,
            FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN
        );
        assert!(FEDERATION_STATE_MACHINE_TRACE_BODY_HASH_DOMAIN
            .bytes()
            .all(|byte| byte != 0));
        assert!(FEDERATION_STATE_MACHINE_TRACE_EVIDENCE_CHAIN_HASH_DOMAIN
            .bytes()
            .all(|byte| byte != 0));

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
        let audit = vec![
            FederationInvariantAuditEntry {
                id: FederationInvariantId::SourceObservationBijection,
                status: FederationInvariantAuditStatus::Violated(
                    FederationInvariantViolation::SourceObservationSetMismatch,
                ),
            },
        ];
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
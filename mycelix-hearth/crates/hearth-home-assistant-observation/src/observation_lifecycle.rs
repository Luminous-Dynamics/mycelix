// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Explicit lifecycle semantics for read-only Home Assistant observations.
//!
//! The entity projector owns provider-cache atomicity; this ledger owns whether
//! any previously emitted positive observation is still current. A removed,
//! unknown, unavailable, malformed, expired, disconnected, or reconnect-omitted
//! entity must never leave an old positive observation current.
//!
//! This is an in-memory state machine. A checkpoint preserves revision/epoch
//! high-water marks but never restores a positive observation as current. The
//! caller must persist checkpoints atomically if it wants crash/replay
//! protection across process restarts.

use crate::entity_stream::EntityStreamProjector;
use crate::websocket_protocol::EntityStreamKind;
use crate::{
    map_state, HomeAssistantObservation, HomeAssistantObservationProfile,
    HomeAssistantSourceStatus, MappingOutcome,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const OBSERVATION_LIFECYCLE_SCHEMA_VERSION: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecyclePhase {
    Dormant,
    AwaitingSubscription,
    AwaitingSubscriptionAck,
    AwaitingSnapshot,
    SnapshotPendingApply,
    Live,
    NeedsNewSession,
    Closed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleFailure {
    InvalidProfile,
    InvalidCheckpoint,
    CheckpointProfileMismatch,
    EpochNotIncreasing,
    StaleSessionEpoch,
    StaleSubscriptionId,
    InvalidTransition,
    ProjectionRejected,
    MappingRejected,
    TimeRegression,
    RevisionExhausted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InvalidationReason {
    SourceUnknown,
    SourceUnavailable,
    EntityRemoved,
    FreshnessExpired,
    TransportDisconnected,
    SnapshotOmitted,
    SessionReplaced,
    ProjectionRejected,
    MappingRejected,
    ProtocolBoundaryRejected,
    ClockRegression,
}

/// Opaque reference to one positive lifecycle revision. It deliberately does
/// not hash or contain the observation value: low-entropy household state
/// hashes can leak the state through dictionary attacks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PositiveRevisionRef {
    pub session_epoch: u64,
    pub revision: u64,
}

/// Outcome of an authoritative lifecycle currentness check. The lifecycle
/// update (including any expiry/clock-fault invalidations) must be durably
/// committed before a consumer acts on `revision`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CurrentnessCheck {
    pub revision: Option<PositiveRevisionRef>,
    pub update: LifecycleUpdate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleRecordKind {
    PositiveCurrent {
        valid_until_micros: i64,
        supersedes: Option<PositiveRevisionRef>,
    },
    Invalidated {
        reason: InvalidationReason,
        previous_positive: PositiveRevisionRef,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationLifecycleRecord {
    /// Exact allowlisted entity identity; no friendly name or arbitrary fields.
    pub entity_id: String,
    /// Session in which this lifecycle transition was committed.
    pub session_epoch: u64,
    /// Monotonically increasing per-entity lifecycle revision.
    pub revision: u64,
    /// Local monotonic time supplied by the caller.
    pub at_micros: i64,
    pub kind: LifecycleRecordKind,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LifecycleUpdate {
    pub phase: LifecyclePhase,
    /// Safe, deterministic lifecycle metadata. Never includes raw provider data.
    pub records: Vec<ObservationLifecycleRecord>,
    /// Typed semantic observations are returned separately from lifecycle records.
    pub observations: Vec<HomeAssistantObservation>,
    /// Failure category only; no raw payload, credentials, or provider error text.
    pub failure: Option<LifecycleFailure>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationLifecycleCheckpointV1 {
    pub schema_version: u16,
    pub profile_ref: String,
    pub highest_session_epoch: Option<u64>,
    /// Includes active revisions and tombstones after invalidation.
    pub entity_revisions: BTreeMap<String, u64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ActivePositive {
    session_epoch: u64,
    revision: u64,
    valid_until_micros: i64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum StagedEntityState {
    Positive(HomeAssistantObservation),
    Unknown,
    Unavailable,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum StagedDeltaAction {
    Projected(StagedEntityState),
    Invalidate(InvalidationReason),
}

pub struct ObservationLifecycle {
    profile: HomeAssistantObservationProfile,
    allowlisted_entity_ids: BTreeSet<String>,
    projector: Option<EntityStreamProjector>,
    phase: LifecyclePhase,
    current_epoch: Option<u64>,
    highest_session_epoch: Option<u64>,
    subscription_id: Option<u64>,
    pending_snapshot: Option<BTreeMap<String, StagedEntityState>>,
    revisions: BTreeMap<String, u64>,
    active: BTreeMap<String, ActivePositive>,
    last_at_micros: Option<i64>,
}

impl ObservationLifecycle {
    pub fn new(profile: &HomeAssistantObservationProfile) -> Result<Self, LifecycleFailure> {
        profile.validate().map_err(|_| LifecycleFailure::InvalidProfile)?;
        let allowlisted_entity_ids = profile
            .rules
            .iter()
            .map(|rule| rule.entity_id.clone())
            .collect::<BTreeSet<_>>();
        if allowlisted_entity_ids.is_empty() || allowlisted_entity_ids.len() != profile.rules.len() {
            return Err(LifecycleFailure::InvalidProfile);
        }
        Ok(Self {
            profile: profile.clone(),
            allowlisted_entity_ids,
            projector: None,
            phase: LifecyclePhase::Dormant,
            current_epoch: None,
            highest_session_epoch: None,
            subscription_id: None,
            pending_snapshot: None,
            revisions: BTreeMap::new(),
            active: BTreeMap::new(),
            last_at_micros: None,
        })
    }

    /// Restore only the replay/epoch high-water marks. No restored observation
    /// becomes current; a strictly newer session and a fresh snapshot are required.
    pub fn restore(
        profile: &HomeAssistantObservationProfile,
        checkpoint: ObservationLifecycleCheckpointV1,
    ) -> Result<Self, LifecycleFailure> {
        if checkpoint.schema_version != OBSERVATION_LIFECYCLE_SCHEMA_VERSION {
            return Err(LifecycleFailure::InvalidCheckpoint);
        }
        profile.validate().map_err(|_| LifecycleFailure::InvalidProfile)?;
        if profile.profile_ref != checkpoint.profile_ref {
            return Err(LifecycleFailure::CheckpointProfileMismatch);
        }
        let allowlist = profile
            .rules
            .iter()
            .map(|rule| rule.entity_id.clone())
            .collect::<BTreeSet<_>>();
        if checkpoint.highest_session_epoch == Some(0)
            || checkpoint.highest_session_epoch == Some(u64::MAX)
            || allowlist.is_empty()
            || allowlist.len() != profile.rules.len()
            || checkpoint.entity_revisions.iter().any(|(entity_id, revision)| {
                !allowlist.contains(entity_id) || *revision == 0 || *revision > u64::MAX - 2
            })
        {
            return Err(LifecycleFailure::InvalidCheckpoint);
        }

        let mut lifecycle = Self::new(profile)?;
        lifecycle.highest_session_epoch = checkpoint.highest_session_epoch;
        lifecycle.revisions = checkpoint.entity_revisions;
        // Intentionally no current_epoch, projector, or active positive state.
        Ok(lifecycle)
    }

    pub fn phase(&self) -> LifecyclePhase {
        self.phase
    }

    pub fn current_session_epoch(&self) -> Option<u64> {
        self.current_epoch
    }

    /// A positive revision is current only during a live, snapshot-applied
    /// session, before its local freshness deadline. A cache hit alone is not
    /// enough to establish currentness.
    pub fn current_revision(&self, entity_id: &str, now_micros: i64) -> Option<PositiveRevisionRef> {
        if self.phase != LifecyclePhase::Live {
            return None;
        }
        let epoch = self.current_epoch?;
        let active = self.active.get(entity_id)?;
        if active.session_epoch != epoch
            || now_micros < self.last_at_micros.unwrap_or(i64::MIN)
            || now_micros >= active.valid_until_micros
        {
            return None;
        }
        Some(PositiveRevisionRef {
            session_epoch: active.session_epoch,
            revision: active.revision,
        })
    }

    /// Time-check and currentness check for callers that need a fail-closed
    /// lifecycle transition rather than a read-only lookup. This method runs
    /// expiry processing first; a regressed clock tears down the session and
    /// returns invalidation records. The caller must durably persist every
    /// returned record before using any returned positive revision.
    pub fn check_currentness(
        &mut self,
        entity_id: &str,
        now_micros: i64,
    ) -> CurrentnessCheck {
        let update = self.expire(now_micros);
        let revision = if update.failure.is_none() && update.phase == LifecyclePhase::Live {
            self.current_revision(entity_id, now_micros)
        } else {
            None
        };
        CurrentnessCheck { revision, update }
    }

    /// Start a strictly newer local session. Old positives become non-current
    /// immediately because the lifecycle phase leaves Live. Their per-entity
    /// invalidation reason is emitted when the new snapshot is reconciled, so
    /// the ledger can distinguish a missing entity from unknown/unavailable.
    pub fn start_session(&mut self, session_epoch: u64, now_micros: i64) -> LifecycleUpdate {
        // Reserve u64::MAX as exhausted: checkpoint restore rejects it
        // because no strictly newer session epoch could ever be admitted.
        if session_epoch == 0
            || session_epoch == u64::MAX
            || self
                .highest_session_epoch
                .is_some_and(|previous| session_epoch <= previous)
        {
            return self.update(Vec::new(), Vec::new(), Some(LifecycleFailure::EpochNotIncreasing));
        }
        if self.time_regressed(now_micros) {
            return self.fail_session(LifecycleFailure::TimeRegression, InvalidationReason::ClockRegression);
        }

        let Some(projector) = EntityStreamProjector::new(&self.profile).ok() else {
            return self.fail_session(LifecycleFailure::InvalidProfile, InvalidationReason::ProjectionRejected);
        };
        // Revoke old positive evidence at the moment the old session stops
        // being authoritative, not after authentication or the next snapshot.
        // Preflight all tombstone increments so exhaustion cannot partially
        // invalidate a multi-entity old session.
        let previous_ids = self.active.keys().cloned().collect::<Vec<_>>();
        if previous_ids.iter().any(|entity_id| self.revision_would_overflow(entity_id, 1)) {
            self.last_at_micros = Some(now_micros);
            return self.fail_session(LifecycleFailure::RevisionExhausted, InvalidationReason::ProjectionRejected);
        }
        self.last_at_micros = Some(now_micros);
        let mut records = Vec::new();
        for entity_id in previous_ids {
            if let Some(record) = self.invalidate(
                &entity_id,
                InvalidationReason::SessionReplaced,
                session_epoch,
                now_micros,
            ) {
                records.push(record);
            }
        }
        records.sort_by(|a, b| a.entity_id.cmp(&b.entity_id));
        self.current_epoch = Some(session_epoch);
        self.highest_session_epoch = Some(session_epoch);
        self.subscription_id = None;
        self.pending_snapshot = None;
        self.projector = Some(projector);
        self.phase = LifecyclePhase::AwaitingSubscription;
        self.update(records, Vec::new(), None)
    }

    /// Bind the exact subscription command ID emitted by the protocol gate.
    /// A subscription acknowledgement is still required before snapshot input.
    pub fn bind_subscription(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        now_micros: i64,
    ) -> LifecycleUpdate {
        if !self.matches_epoch(session_epoch) {
            return self.update(Vec::new(), Vec::new(), Some(LifecycleFailure::StaleSessionEpoch));
        }
        if self.time_regressed(now_micros) {
            return self.fail_session(LifecycleFailure::TimeRegression, InvalidationReason::ClockRegression);
        }
        self.last_at_micros = Some(now_micros);
        if self.phase != LifecyclePhase::AwaitingSubscription || subscription_id == 0 {
            return self.fail_session(LifecycleFailure::InvalidTransition, InvalidationReason::ProtocolBoundaryRejected);
        }
        self.subscription_id = Some(subscription_id);
        self.phase = LifecyclePhase::AwaitingSubscriptionAck;
        self.last_at_micros = Some(now_micros);
        self.update(Vec::new(), Vec::new(), None)
    }

    pub fn subscription_acknowledged(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        now_micros: i64,
    ) -> LifecycleUpdate {
        if !self.matches_epoch(session_epoch) {
            return self.update(Vec::new(), Vec::new(), Some(LifecycleFailure::StaleSessionEpoch));
        }
        if self.time_regressed(now_micros) {
            return self.fail_session(LifecycleFailure::TimeRegression, InvalidationReason::ClockRegression);
        }
        self.last_at_micros = Some(now_micros);
        if self.phase != LifecyclePhase::AwaitingSubscriptionAck
            || self.subscription_id != Some(subscription_id)
        {
            return self.fail_session(LifecycleFailure::StaleSubscriptionId, InvalidationReason::ProtocolBoundaryRejected);
        }
        self.phase = LifecyclePhase::AwaitingSnapshot;
        self.last_at_micros = Some(now_micros);
        self.update(Vec::new(), Vec::new(), None)
    }

    /// Strictly parse, project, and apply one event. Projection failures fail the
    /// lifecycle session closed and invalidate every active positive observation;
    /// projector-level transactional rollback alone is not enough to revoke a
    /// positive observation already consumed downstream.
    pub fn apply_json(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        kind: EntityStreamKind,
        json_bytes: &[u8],
        now_micros: i64,
    ) -> LifecycleUpdate {
        if !self.matches_epoch(session_epoch) {
            return self.update(Vec::new(), Vec::new(), Some(LifecycleFailure::StaleSessionEpoch));
        }
        if self.time_regressed(now_micros) {
            return self.fail_session(LifecycleFailure::TimeRegression, InvalidationReason::ClockRegression);
        }
        self.last_at_micros = Some(now_micros);
        if self.subscription_id != Some(subscription_id) {
            return self.fail_session(LifecycleFailure::StaleSubscriptionId, InvalidationReason::ProtocolBoundaryRejected);
        }

        let allowed_kind = match self.phase {
            LifecyclePhase::AwaitingSnapshot => kind == EntityStreamKind::InitialSnapshot,
            LifecyclePhase::Live => kind == EntityStreamKind::Changes,
            _ => false,
        };
        if !allowed_kind {
            return self.fail_session(LifecycleFailure::InvalidTransition, InvalidationReason::ProtocolBoundaryRejected);
        }

        let Some(projector) = self.projector.as_mut() else {
            return self.fail_session(LifecycleFailure::InvalidTransition, InvalidationReason::ProjectionRejected);
        };
        let projection = match projector.apply_json(kind, json_bytes, now_micros) {
            Ok(projection) => projection,
            Err(_) => return self.fail_session(LifecycleFailure::ProjectionRejected, InvalidationReason::ProjectionRejected),
        };
        let staged = match self.map_frames(&projection.frames) {
            Ok(staged) => staged,
            Err(_) => return self.fail_session(LifecycleFailure::MappingRejected, InvalidationReason::MappingRejected),
        };
        self.last_at_micros = Some(now_micros);

        if kind == EntityStreamKind::InitialSnapshot {
            if !projection.initial_snapshot {
                return self.fail_session(LifecycleFailure::InvalidTransition, InvalidationReason::ProtocolBoundaryRejected);
            }
            self.pending_snapshot = Some(staged);
            self.phase = LifecyclePhase::SnapshotPendingApply;
            return self.update(Vec::new(), Vec::new(), None);
        }

        if projection.initial_snapshot {
            return self.fail_session(LifecycleFailure::InvalidTransition, InvalidationReason::ProtocolBoundaryRejected);
        }
        self.commit_delta(staged, projection.removed_entity_ids, now_micros)
    }

    /// Acknowledge the exact snapshot only after the protocol machine has
    /// accepted it and the transport has applied its normalized snapshot.
    /// No observation becomes current before this handshake.
    pub fn snapshot_applied(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        now_micros: i64,
    ) -> LifecycleUpdate {
        if !self.matches_epoch(session_epoch) {
            return self.update(Vec::new(), Vec::new(), Some(LifecycleFailure::StaleSessionEpoch));
        }
        if self.time_regressed(now_micros) {
            return self.fail_session(LifecycleFailure::TimeRegression, InvalidationReason::ClockRegression);
        }
        self.last_at_micros = Some(now_micros);
        if self.phase != LifecyclePhase::SnapshotPendingApply
            || self.subscription_id != Some(subscription_id)
        {
            return self.fail_session(LifecycleFailure::InvalidTransition, InvalidationReason::ProtocolBoundaryRejected);
        }
        let Some(staged) = self.pending_snapshot.take() else {
            return self.fail_session(LifecycleFailure::InvalidTransition, InvalidationReason::ProjectionRejected);
        };

        // Reconcile every old positive before promoting anything from the new
        // snapshot. This makes omission, source-unavailable and ordinary session
        // replacement explicit; no previous session's positive remains current.
        // A reconnect snapshot can require one invalidation plus one new
        // positive revision for an entity that is still present. Preflight the
        // entire write set before changing any entity revision.
        let mut required_increments = BTreeMap::<String, u64>::new();
        for entity_id in self.active.keys() {
            *required_increments.entry(entity_id.clone()).or_default() += 1;
        }
        for (entity_id, state) in &staged {
            if matches!(state, StagedEntityState::Positive(_)) {
                *required_increments.entry(entity_id.clone()).or_default() += 1;
            }
        }
        if required_increments.iter().any(|(entity_id, count)| {
            self.revision_would_overflow(entity_id, *count)
        }) {
            return self.fail_session(LifecycleFailure::RevisionExhausted, InvalidationReason::ProjectionRejected);
        }

        let prior_ids = self.active.keys().cloned().collect::<Vec<_>>();
        let mut records = Vec::new();
        let mut observations = Vec::new();
        for entity_id in prior_ids {
            let reason = match staged.get(&entity_id) {
                Some(StagedEntityState::Unknown) => InvalidationReason::SourceUnknown,
                Some(StagedEntityState::Unavailable) => InvalidationReason::SourceUnavailable,
                Some(StagedEntityState::Positive(_)) => InvalidationReason::SessionReplaced,
                None => InvalidationReason::SnapshotOmitted,
            };
            if let Some(record) = self.invalidate(&entity_id, reason, session_epoch, now_micros) {
                records.push(record);
            }
        }

        self.phase = LifecyclePhase::Live;
        for (entity_id, staged_state) in staged {
            match staged_state {
                StagedEntityState::Positive(observation) => {
                    if let Some((record, observation)) = self.activate(entity_id, observation, session_epoch, now_micros) {
                        records.push(record);
                        observations.push(observation);
                    } else {
                        return self.fail_session(LifecycleFailure::RevisionExhausted, InvalidationReason::ProjectionRejected);
                    }
                }
                StagedEntityState::Unknown => {
                    if let Some(record) = self.invalidate(&entity_id, InvalidationReason::SourceUnknown, session_epoch, now_micros) {
                        records.push(record);
                    }
                }
                StagedEntityState::Unavailable => {
                    if let Some(record) = self.invalidate(&entity_id, InvalidationReason::SourceUnavailable, session_epoch, now_micros) {
                        records.push(record);
                    }
                }
            }
        }
        self.last_at_micros = Some(now_micros);
        records.sort_by(|a, b| a.entity_id.cmp(&b.entity_id).then(a.revision.cmp(&b.revision)));
        self.update(records, observations, None)
    }

    pub fn expire(&mut self, now_micros: i64) -> LifecycleUpdate {
        if self.time_regressed(now_micros) {
            return self.fail_session(LifecycleFailure::TimeRegression, InvalidationReason::ClockRegression);
        }
        self.last_at_micros = Some(now_micros);
        let epoch = self.current_epoch.unwrap_or(self.highest_session_epoch.unwrap_or(0));
        let expired = self
            .active
            .iter()
            .filter(|(_, active)| now_micros >= active.valid_until_micros)
            .map(|(entity_id, _)| entity_id.clone())
            .collect::<Vec<_>>();
        let mut records = Vec::new();
        for entity_id in expired {
            if let Some(record) = self.invalidate(&entity_id, InvalidationReason::FreshnessExpired, epoch, now_micros) {
                records.push(record);
            }
        }
        records.sort_by(|a, b| a.entity_id.cmp(&b.entity_id));
        self.update(records, Vec::new(), None)
    }

    pub fn disconnect(&mut self, session_epoch: u64, now_micros: i64) -> LifecycleUpdate {
        if !self.matches_epoch(session_epoch) {
            return self.update(Vec::new(), Vec::new(), Some(LifecycleFailure::StaleSessionEpoch));
        }
        if self.time_regressed(now_micros) {
            return self.fail_session(LifecycleFailure::TimeRegression, InvalidationReason::ClockRegression);
        }
        self.last_at_micros = Some(now_micros);
        let ids = self.active.keys().cloned().collect::<Vec<_>>();
        let mut records = Vec::new();
        for entity_id in ids {
            if let Some(record) = self.invalidate(&entity_id, InvalidationReason::TransportDisconnected, session_epoch, now_micros) {
                records.push(record);
            }
        }
        self.phase = LifecyclePhase::Closed;
        self.projector = None;
        self.pending_snapshot = None;
        self.subscription_id = None;
        self.last_at_micros = Some(now_micros);
        records.sort_by(|a, b| a.entity_id.cmp(&b.entity_id));
        self.update(records, Vec::new(), None)
    }

    /// Export deterministic replay metadata only. Positive state values,
    /// attributes, timestamps from the provider, credentials and URLs are absent.
    pub fn checkpoint(&self) -> ObservationLifecycleCheckpointV1 {
        ObservationLifecycleCheckpointV1 {
            schema_version: OBSERVATION_LIFECYCLE_SCHEMA_VERSION,
            profile_ref: self.profile.profile_ref.clone(),
            highest_session_epoch: self.highest_session_epoch,
            entity_revisions: self.revisions.clone(),
        }
    }

    fn matches_epoch(&self, session_epoch: u64) -> bool {
        self.current_epoch == Some(session_epoch)
    }

    fn time_regressed(&self, now_micros: i64) -> bool {
        self.last_at_micros.is_some_and(|previous| now_micros < previous)
    }

    fn map_frames(
        &self,
        frames: &[crate::HomeAssistantStateFrame],
    ) -> Result<BTreeMap<String, StagedEntityState>, LifecycleFailure> {
        let mut staged = BTreeMap::new();
        for frame in frames {
            if !self.allowlisted_entity_ids.contains(&frame.entity_id) {
                continue;
            }
            let outcome = map_state(&self.profile, frame).map_err(|_| LifecycleFailure::MappingRejected)?;
            let state = match outcome {
                MappingOutcome::Observation(observation) => StagedEntityState::Positive(observation),
                MappingOutcome::SourceState { status: HomeAssistantSourceStatus::Unknown, .. } => StagedEntityState::Unknown,
                MappingOutcome::SourceState { status: HomeAssistantSourceStatus::Unavailable, .. } => StagedEntityState::Unavailable,
                MappingOutcome::NotAllowlisted { .. } => continue,
            };
            if staged.insert(frame.entity_id.clone(), state).is_some() {
                return Err(LifecycleFailure::MappingRejected);
            }
        }
        Ok(staged)
    }

    fn commit_delta(
        &mut self,
        staged: BTreeMap<String, StagedEntityState>,
        removed_entity_ids: Vec<String>,
        now_micros: i64,
    ) -> LifecycleUpdate {
        let epoch = self.current_epoch.unwrap_or(0);
        let mut actions = BTreeMap::<String, StagedDeltaAction>::new();
        for (entity_id, state) in staged {
            actions.insert(entity_id, StagedDeltaAction::Projected(state));
        }
        for entity_id in removed_entity_ids {
            if !self.allowlisted_entity_ids.contains(&entity_id) {
                continue;
            }
            if actions.insert(entity_id, StagedDeltaAction::Invalidate(InvalidationReason::EntityRemoved)).is_some() {
                return self.fail_session(LifecycleFailure::MappingRejected, InvalidationReason::MappingRejected);
            }
        }

        // Preflight the revision writes so a pathological restored high-water
        // mark cannot partially commit a multi-entity lifecycle update.
        for (entity_id, action) in &actions {
            let increments = match action {
                StagedDeltaAction::Projected(StagedEntityState::Positive(_)) => 1,
                StagedDeltaAction::Projected(StagedEntityState::Unknown | StagedEntityState::Unavailable)
                | StagedDeltaAction::Invalidate(_) => if self.active.contains_key(entity_id) { 1 } else { 0 },
            };
            if self.revision_would_overflow(entity_id, increments) {
                return self.fail_session(LifecycleFailure::RevisionExhausted, InvalidationReason::ProjectionRejected);
            }
        }

        let mut records = Vec::new();
        let mut observations = Vec::new();
        for (entity_id, action) in actions {
            match action {
                StagedDeltaAction::Projected(StagedEntityState::Positive(observation)) => {
                    let Some((record, observation)) = self.activate(entity_id, observation, epoch, now_micros) else {
                        return self.fail_session(LifecycleFailure::RevisionExhausted, InvalidationReason::ProjectionRejected);
                    };
                    records.push(record);
                    observations.push(observation);
                }
                StagedDeltaAction::Projected(StagedEntityState::Unknown) => {
                    if let Some(record) = self.invalidate(&entity_id, InvalidationReason::SourceUnknown, epoch, now_micros) {
                        records.push(record);
                    }
                }
                StagedDeltaAction::Projected(StagedEntityState::Unavailable) => {
                    if let Some(record) = self.invalidate(&entity_id, InvalidationReason::SourceUnavailable, epoch, now_micros) {
                        records.push(record);
                    }
                }
                StagedDeltaAction::Invalidate(reason) => {
                    if let Some(record) = self.invalidate(&entity_id, reason, epoch, now_micros) {
                        records.push(record);
                    }
                }
            }
        }
        records.sort_by(|a, b| a.entity_id.cmp(&b.entity_id).then(a.revision.cmp(&b.revision)));
        self.last_at_micros = Some(now_micros);
        self.update(records, observations, None)
    }

    fn activate(
        &mut self,
        entity_id: String,
        observation: HomeAssistantObservation,
        transition_epoch: u64,
        now_micros: i64,
    ) -> Option<(ObservationLifecycleRecord, HomeAssistantObservation)> {
        let valid_until_micros = observation.observation.valid_until_micros?;
        let supersedes = self.active.get(&entity_id).copied().map(|active| PositiveRevisionRef {
            session_epoch: active.session_epoch,
            revision: active.revision,
        });
        let revision = self.bump_revision(&entity_id)?;
        self.active.insert(entity_id.clone(), ActivePositive {
            session_epoch: transition_epoch,
            revision,
            valid_until_micros,
        });
        Some((
            ObservationLifecycleRecord {
                entity_id,
                session_epoch: transition_epoch,
                revision,
                at_micros: now_micros,
                kind: LifecycleRecordKind::PositiveCurrent {
                    valid_until_micros,
                    supersedes,
                },
            },
            observation,
        ))
    }

    fn invalidate(
        &mut self,
        entity_id: &str,
        reason: InvalidationReason,
        transition_epoch: u64,
        now_micros: i64,
    ) -> Option<ObservationLifecycleRecord> {
        let previous = self.active.get(entity_id).copied()?;
        let revision = self.bump_revision(entity_id)?;
        self.active.remove(entity_id);
        Some(ObservationLifecycleRecord {
            entity_id: entity_id.to_owned(),
            session_epoch: transition_epoch,
            revision,
            at_micros: now_micros,
            kind: LifecycleRecordKind::Invalidated {
                reason,
                previous_positive: PositiveRevisionRef {
                    session_epoch: previous.session_epoch,
                    revision: previous.revision,
                },
            },
        })
    }

    fn bump_revision(&mut self, entity_id: &str) -> Option<u64> {
        let current = self.revisions.get(entity_id).copied().unwrap_or(0);
        let next = current.checked_add(1)?;
        self.revisions.insert(entity_id.to_owned(), next);
        Some(next)
    }

    fn revision_would_overflow(&self, entity_id: &str, increments: u64) -> bool {
        self.revisions
            .get(entity_id)
            .copied()
            .unwrap_or(0)
            .checked_add(increments)
            .is_none()
    }

    fn fail_session(
        &mut self,
        failure: LifecycleFailure,
        reason: InvalidationReason,
    ) -> LifecycleUpdate {
        let event_time = self.last_at_micros.unwrap_or(0);
        let epoch = self.current_epoch.unwrap_or(self.highest_session_epoch.unwrap_or(0));
        let ids = self.active.keys().cloned().collect::<Vec<_>>();
        let mut records = Vec::new();
        for entity_id in ids {
            if let Some(record) = self.invalidate(&entity_id, reason, epoch, event_time) {
                records.push(record);
            }
        }
        self.phase = LifecyclePhase::NeedsNewSession;
        self.projector = None;
        self.pending_snapshot = None;
        self.subscription_id = None;
        records.sort_by(|a, b| a.entity_id.cmp(&b.entity_id));
        self.update(records, Vec::new(), Some(failure))
    }

    fn update(
        &self,
        records: Vec<ObservationLifecycleRecord>,
        observations: Vec<HomeAssistantObservation>,
        failure: Option<LifecycleFailure>,
    ) -> LifecycleUpdate {
        LifecycleUpdate { phase: self.phase, records, observations, failure }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EntityObservationRule, EntityRef, StateCodec};
    use serde_json::{json, Value};
    use std::collections::BTreeSet;

    fn profile() -> HomeAssistantObservationProfile {
        HomeAssistantObservationProfile {
            schema_version: 1,
            instance_id: "home-main".into(),
            profile_ref: "profile:ha-v1".into(),
            rules: vec![EntityObservationRule {
                entity_id: "sensor.living_room_temperature".into(),
                subject: EntityRef { kind: "room".into(), id: "living-room".into() },
                attribute: "temperature_celsius".into(),
                codec: StateCodec::FixedRange { scale: 2, min_mantissa: -5_000, max_mantissa: 8_000 },
                expected_unit: Some("°C".into()),
                expected_device_class: Some("temperature".into()),
                max_age_ms: 30_000,
                confidence_bp: 9_500,
            }],
        }
    }

    fn bytes(value: Value) -> Vec<u8> {
        serde_json::to_vec(&value).expect("test JSON serializes")
    }

    fn snapshot(state: &str) -> Value {
        json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": state,
                    "a": {
                        "unit_of_measurement": "°C",
                        "device_class": "temperature",
                        "friendly_name": "private fixture marker"
                    }
                }
            }
        })
    }

    fn two_sensor_profile() -> HomeAssistantObservationProfile {
        let mut p = profile();
        let mut second = p.rules[0].clone();
        second.entity_id = "sensor.z_temperature".into();
        second.subject.id = "kitchen".into();
        p.rules.push(second);
        p
    }

    fn snapshot_two() -> Value {
        json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": "21.50",
                    "a": { "unit_of_measurement": "°C", "device_class": "temperature" }
                },
                "sensor.z_temperature": {
                    "s": "20.00",
                    "a": { "unit_of_measurement": "°C", "device_class": "temperature" }
                }
            }
        })
    }

    fn change(state: &str) -> Value {
        json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": { "s": state }
                }
            }
        })
    }

    fn start_live() -> ObservationLifecycle {
        let p = profile();
        let mut lifecycle = ObservationLifecycle::new(&p).expect("profile validates");
        assert_eq!(lifecycle.start_session(1, 10).phase, LifecyclePhase::AwaitingSubscription);
        assert_eq!(lifecycle.bind_subscription(1, 1, 11).phase, LifecyclePhase::AwaitingSubscriptionAck);
        assert_eq!(lifecycle.subscription_acknowledged(1, 1, 12).phase, LifecyclePhase::AwaitingSnapshot);
        let pending = lifecycle.apply_json(1, 1, EntityStreamKind::InitialSnapshot, &bytes(snapshot("21.50")), 20);
        assert_eq!(pending.phase, LifecyclePhase::SnapshotPendingApply);
        assert!(pending.observations.is_empty());
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 20).is_none());
        let live = lifecycle.snapshot_applied(1, 1, 21);
        assert_eq!(live.phase, LifecyclePhase::Live);
        assert_eq!(live.observations.len(), 1);
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 21).is_some());
        lifecycle
    }

    #[test]
    fn snapshot_is_not_current_until_explicit_apply_ack() {
        let p = profile();
        let mut lifecycle = ObservationLifecycle::new(&p).unwrap();
        lifecycle.start_session(1, 10);
        lifecycle.bind_subscription(1, 1, 11);
        lifecycle.subscription_acknowledged(1, 1, 12);
        let pending = lifecycle.apply_json(1, 1, EntityStreamKind::InitialSnapshot, &bytes(snapshot("21.50")), 20);
        assert_eq!(pending.phase, LifecyclePhase::SnapshotPendingApply);
        assert!(pending.observations.is_empty());
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 20).is_none());
        let applied = lifecycle.snapshot_applied(1, 1, 21);
        assert_eq!(applied.phase, LifecyclePhase::Live);
        assert_eq!(applied.observations.len(), 1);
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 21).is_some());
    }

    #[test]
    fn unknown_invalidates_positive_exactly_once() {
        let mut lifecycle = start_live();
        let id = "sensor.living_room_temperature";
        let out = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(change("unknown")), 30);
        assert!(out.observations.is_empty());
        assert_eq!(out.records.len(), 1);
        assert!(matches!(out.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::SourceUnknown, ..
        }));
        assert!(lifecycle.current_revision(id, 30).is_none());

        let duplicate = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(change("unknown")), 31);
        assert!(duplicate.records.is_empty(), "duplicate unknown state must not emit duplicate invalidation");
    }

    #[test]
    fn unavailable_invalidates_positive() {
        let mut lifecycle = start_live();
        let out = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(change("unavailable")), 30);
        assert_eq!(out.records.len(), 1);
        assert!(matches!(out.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::SourceUnavailable, ..
        }));
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 30).is_none());
    }

    #[test]
    fn explicit_removal_invalidates_once_and_never_means_physical_absence() {
        let mut lifecycle = start_live();
        let removed = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(json!({
            "r": ["sensor.living_room_temperature"]
        })), 30);
        assert_eq!(removed.observations.len(), 0);
        assert_eq!(removed.records.len(), 1);
        assert!(matches!(removed.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::EntityRemoved, ..
        }));
        let repeated = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(json!({
            "r": ["sensor.living_room_temperature"]
        })), 31);
        assert!(repeated.records.is_empty());
    }

    #[test]
    fn expiry_invalidates_at_the_deadline_and_does_not_repeat() {
        let mut lifecycle = start_live();
        let expired = lifecycle.expire(30_000_020);
        assert_eq!(expired.records.len(), 1);
        assert!(matches!(expired.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::FreshnessExpired, ..
        }));
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 30_000_020).is_none());
        assert!(lifecycle.expire(30_000_021).records.is_empty());
    }

    #[test]
    fn disconnect_revokes_every_positive_revision() {
        let mut lifecycle = start_live();
        let out = lifecycle.disconnect(1, 30);
        assert_eq!(out.phase, LifecyclePhase::Closed);
        assert_eq!(out.records.len(), 1);
        assert!(matches!(out.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::TransportDisconnected, ..
        }));
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 31).is_none());
    }

    #[test]
    fn reconnect_omission_cannot_keep_old_positive_current() {
        let mut lifecycle = start_live();
        let old = lifecycle.current_revision("sensor.living_room_temperature", 22).unwrap();
        let replaced = lifecycle.start_session(2, 23);
        assert_eq!(replaced.phase, LifecyclePhase::AwaitingSubscription);
        assert_eq!(replaced.records.len(), 1);
        assert!(matches!(replaced.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::SessionReplaced,
            previous_positive: PositiveRevisionRef { session_epoch: 1, revision },
        } if revision == old.revision));
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 23).is_none());
        lifecycle.bind_subscription(2, 9, 24);
        lifecycle.subscription_acknowledged(2, 9, 25);
        let pending = lifecycle.apply_json(2, 9, EntityStreamKind::InitialSnapshot, &bytes(json!({"a": {}})), 26);
        assert_eq!(pending.phase, LifecyclePhase::SnapshotPendingApply);
        assert!(pending.observations.is_empty());
        let applied = lifecycle.snapshot_applied(2, 9, 27);
        assert!(applied.records.is_empty(), "already-invalidated old state must not emit another tombstone");
        assert!(applied.observations.is_empty());
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 28).is_none());
    }

    #[test]
    fn reconnect_snapshot_refreshes_only_after_early_invalidation() {
        let mut lifecycle = start_live();
        let old = lifecycle.current_revision("sensor.living_room_temperature", 22).unwrap();
        let replaced = lifecycle.start_session(2, 23);
        assert_eq!(replaced.records.len(), 1);
        assert!(matches!(replaced.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::SessionReplaced,
            previous_positive: PositiveRevisionRef { session_epoch: 1, revision },
        } if revision == old.revision));
        lifecycle.bind_subscription(2, 9, 24);
        lifecycle.subscription_acknowledged(2, 9, 25);
        lifecycle.apply_json(2, 9, EntityStreamKind::InitialSnapshot, &bytes(snapshot("21.50")), 26);
        let out = lifecycle.snapshot_applied(2, 9, 27);
        assert_eq!(out.records.len(), 1);
        assert!(matches!(out.records[0].kind, LifecycleRecordKind::PositiveCurrent {
            supersedes: None,
            ..
        }));
        assert_eq!(out.observations.len(), 1);
        let fresh = lifecycle.current_revision("sensor.living_room_temperature", 27).unwrap();
        assert_eq!(fresh.session_epoch, 2);
        assert!(fresh.revision > old.revision);
    }

    #[test]
    fn malformed_delta_invalidates_session_instead_of_preserving_stale_positive() {
        let mut lifecycle = start_live();
        let out = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(json!({
            "c": { "sensor.living_room_temperature": { "+": { "s": "999999999999" } } }
        })), 30);
        assert_eq!(out.phase, LifecyclePhase::NeedsNewSession);
        assert_eq!(out.failure, Some(LifecycleFailure::ProjectionRejected));
        assert_eq!(out.records.len(), 1);
        assert!(matches!(out.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::ProjectionRejected, ..
        }));
        assert!(out.observations.is_empty());
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 31).is_none());
    }

    #[test]
    fn non_allowlisted_delta_is_ignored_while_valid_allowlisted_state_projects() {
        let mut lifecycle = start_live();
        let out = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(json!({
            "c": {
                "sensor.living_room_temperature": { "+": { "s": "22.0" } },
                "sensor.not_allowlisted": { "+": { "s": "99.0" } }
            }
        })), 30);
        assert_eq!(out.phase, LifecyclePhase::Live);
        assert_eq!(out.observations.len(), 1);
        assert_eq!(out.observations[0].observation.value, hearth_automation_types::AutomationValue::Fixed { mantissa: 2200, scale: 2 });
    }

    #[test]
    fn identical_semantic_replay_does_not_refresh_lifecycle_revision() {
        let mut lifecycle = start_live();
        let id = "sensor.living_room_temperature";
        let prior = lifecycle.current_revision(id, 22).unwrap();
        let duplicate = lifecycle.apply_json(
            1,
            1,
            EntityStreamKind::Changes,
            &bytes(change("21.5")),
            30,
        );
        assert!(duplicate.observations.is_empty());
        assert!(duplicate.records.is_empty());
        assert_eq!(lifecycle.current_revision(id, 30), Some(prior));
    }

    #[test]
    fn valid_observation_after_unknown_gets_a_new_revision() {
        let mut lifecycle = start_live();
        let id = "sensor.living_room_temperature";
        let old = lifecycle.current_revision(id, 22).unwrap();
        lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(change("unknown")), 30);
        assert!(lifecycle.current_revision(id, 30).is_none());
        let next = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(change("21.5")), 31);
        assert_eq!(next.observations.len(), 1);
        let fresh = lifecycle.current_revision(id, 31).unwrap();
        assert!(fresh.revision > old.revision);
    }

    #[test]
    fn malformed_late_entity_invalidates_entire_staged_delta_atomically() {
        let p = two_sensor_profile();
        let mut lifecycle = ObservationLifecycle::new(&p).unwrap();
        lifecycle.start_session(1, 10);
        lifecycle.bind_subscription(1, 1, 11);
        lifecycle.subscription_acknowledged(1, 1, 12);
        let pending = lifecycle.apply_json(
            1,
            1,
            EntityStreamKind::InitialSnapshot,
            &bytes(snapshot_two()),
            20,
        );
        assert_eq!(pending.phase, LifecyclePhase::SnapshotPendingApply);
        let initial = lifecycle.snapshot_applied(1, 1, 21);
        assert_eq!(initial.observations.len(), 2);
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 21).is_some());
        assert!(lifecycle.current_revision("sensor.z_temperature", 21).is_some());

        // The first entity is a valid staged change; the lexicographically later
        // entity is out of range. No staged positive may escape, and both prior
        // positives are explicitly invalidated because the stream is restarted.
        let invalid = json!({
            "c": {
                "sensor.living_room_temperature": { "+": { "s": "22.00" } },
                "sensor.z_temperature": { "+": { "s": "999999999999" } }
            }
        });
        let out = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(invalid), 30);
        assert_eq!(out.phase, LifecyclePhase::NeedsNewSession);
        assert_eq!(out.failure, Some(LifecycleFailure::ProjectionRejected));
        assert!(out.observations.is_empty());
        assert_eq!(out.records.len(), 2);
        assert!(out.records.iter().all(|record| matches!(
            record.kind,
            LifecycleRecordKind::Invalidated {
                reason: InvalidationReason::ProjectionRejected,
                ..
            }
        )));
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 31).is_none());
        assert!(lifecycle.current_revision("sensor.z_temperature", 31).is_none());
    }

    #[test]
    fn wrong_subscription_id_revokes_current_session() {
        let mut lifecycle = start_live();
        let out = lifecycle.apply_json(
            1,
            99,
            EntityStreamKind::Changes,
            &bytes(change("22.0")),
            30,
        );
        assert_eq!(out.failure, Some(LifecycleFailure::StaleSubscriptionId));
        assert_eq!(out.phase, LifecyclePhase::NeedsNewSession);
        assert_eq!(out.records.len(), 1);
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 31).is_none());
    }

    #[test]
    fn clock_regression_fails_closed_and_invalidates_positive() {
        let mut lifecycle = start_live();
        let out = lifecycle.expire(5);
        assert_eq!(out.failure, Some(LifecycleFailure::TimeRegression));
        assert_eq!(out.phase, LifecyclePhase::NeedsNewSession);
        assert_eq!(out.records.len(), 1);
        assert!(matches!(out.records[0].kind, LifecycleRecordKind::Invalidated {
            reason: InvalidationReason::ClockRegression, ..
        }));
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 30).is_none());
    }

    #[test]
    fn checked_currentness_latches_clock_regression_and_invalidates_positive() {
        let mut lifecycle = start_live();
        let entity_id = "sensor.living_room_temperature";
        let valid = lifecycle.check_currentness(entity_id, 22);
        assert!(valid.revision.is_some());
        assert!(valid.update.records.is_empty());
        assert_eq!(valid.update.phase, LifecyclePhase::Live);

        let regressed = lifecycle.check_currentness(entity_id, 5);
        assert_eq!(regressed.revision, None);
        assert_eq!(regressed.update.failure, Some(LifecycleFailure::TimeRegression));
        assert_eq!(regressed.update.phase, LifecyclePhase::NeedsNewSession);
        assert_eq!(regressed.update.records.len(), 1);
        assert!(matches!(
            regressed.update.records[0].kind,
            LifecycleRecordKind::Invalidated {
                reason: InvalidationReason::ClockRegression,
                ..
            }
        ));

        // A later clock reading must not resurrect the old positive. A newer
        // session/bootstrap is required after a clock regression.
        let recovered = lifecycle.check_currentness(entity_id, 30);
        assert_eq!(recovered.revision, None);
        assert_eq!(recovered.update.phase, LifecyclePhase::NeedsNewSession);
        assert!(recovered.update.records.is_empty());
    }

    #[test]
    fn rejects_epoch_max_before_checkpoint_can_become_unrestorable() {
        let mut lifecycle = ObservationLifecycle::new(&profile()).unwrap();
        let update = lifecycle.start_session(u64::MAX, 10);
        assert_eq!(update.failure, Some(LifecycleFailure::EpochNotIncreasing));
        assert_eq!(lifecycle.phase(), LifecyclePhase::Dormant);
        assert_eq!(lifecycle.current_session_epoch(), None);
        let checkpoint = lifecycle.checkpoint();
        assert_eq!(checkpoint.highest_session_epoch, None);
        assert!(ObservationLifecycle::restore(&profile(), checkpoint).is_ok());
    }

    #[test]
    fn stale_epoch_cannot_mutate_current_session() {
        let mut lifecycle = start_live();
        let before = lifecycle.current_revision("sensor.living_room_temperature", 25);
        let out = lifecycle.apply_json(0, 1, EntityStreamKind::Changes, &bytes(change("22.0")), 26);
        assert_eq!(out.failure, Some(LifecycleFailure::StaleSessionEpoch));
        assert_eq!(lifecycle.current_revision("sensor.living_room_temperature", 26), before);
    }

    #[test]
    fn stale_session_epoch_cannot_replace_a_live_session() {
        let mut lifecycle = start_live();
        let out = lifecycle.start_session(1, 30);
        assert_eq!(out.failure, Some(LifecycleFailure::EpochNotIncreasing));
        assert_eq!(lifecycle.phase(), LifecyclePhase::Live);
        assert!(lifecycle.current_revision("sensor.living_room_temperature", 30).is_some());
    }

    #[test]
    fn checkpoint_preserves_tombstone_revision_without_restoring_positive_state() {
        let mut lifecycle = start_live();
        lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(json!({
            "r": ["sensor.living_room_temperature"]
        })), 30);
        let checkpoint = lifecycle.checkpoint();
        let encoded = serde_json::to_vec(&checkpoint).unwrap();
        let decoded: ObservationLifecycleCheckpointV1 = serde_json::from_slice(&encoded).unwrap();
        let restored = ObservationLifecycle::restore(&profile(), decoded).unwrap();
        assert_eq!(restored.phase(), LifecyclePhase::Dormant);
        assert!(restored.current_revision("sensor.living_room_temperature", 31).is_none());
        assert_eq!(restored.start_session(1, 32).failure, Some(LifecycleFailure::EpochNotIncreasing));
        assert_eq!(restored.start_session(2, 33).phase, LifecyclePhase::AwaitingSubscription);
    }

    #[test]
    fn checkpoint_is_bound_to_the_same_profile_reference() {
        let lifecycle = start_live();
        let checkpoint = lifecycle.checkpoint();
        let mut other_profile = profile();
        other_profile.profile_ref = "profile:other-v1".into();
        assert_eq!(
            ObservationLifecycle::restore(&other_profile, checkpoint).err(),
            Some(LifecycleFailure::CheckpointProfileMismatch)
        );
    }

    #[test]
    fn lifecycle_records_do_not_contain_raw_state_or_arbitrary_attributes() {
        let mut lifecycle = start_live();
        let out = lifecycle.apply_json(1, 1, EntityStreamKind::Changes, &bytes(change("unknown")), 30);
        let serialized = serde_json::to_string(&out.records).unwrap();
        assert!(!serialized.contains("21.50"));
        assert!(!serialized.contains("private fixture marker"));
        assert!(!serialized.contains("token"));
    }
}

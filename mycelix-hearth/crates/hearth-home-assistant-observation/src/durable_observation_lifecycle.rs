// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Append-and-sync boundary and cold-start replay validation for HA observation
//! lifecycle transitions.
//!
//! The inner lifecycle is transactional in memory. This wrapper runs each
//! transition against a private clone, persists one value-minimized journal
//! envelope, and only then swaps the candidate into the visible state. If
//! persistence fails, the previous state is retained internally, the wrapper
//! latches unhealthy, and no positive currentness can be returned.
//!
//! Recovery verifies the entire append-only sequence and per-entity revision
//! chain, then deliberately returns to Dormant. Positive observations are never
//! restored as current from a journal. A new higher session epoch and fresh
//! snapshot are required.
//!
//! The journal deliberately does not store state values or unsalted hashes of
//! low-entropy values (for example, binary on/off states). A positive version is
//! identified by its entity/profile/session/revision and creating ledger
//! sequence. This is the privacy-preserving alternative to a value digest.

use crate::entity_stream::EntityStreamKind;
use crate::observation_lifecycle::{
    InvalidationReason, LifecycleFailure, LifecyclePhase, LifecycleRecordKind,
    LifecycleUpdate, ObservationLifecycle, ObservationLifecycleCheckpointV1,
    ObservationLifecycleRecord, PositiveRevisionRef,
};
use crate::HomeAssistantObservationProfile;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const DURABLE_LIFECYCLE_JOURNAL_SCHEMA_VERSION: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DurableLifecycleTransition {
    SessionStart,
    SubscriptionBind,
    SubscriptionAcknowledged,
    InitialSnapshotReceived,
    DeltaReceived,
    SnapshotApplied,
    CurrentnessCheck,
    ExpirySweep,
    Disconnect,
    RecoveryBarrier,
}

/// Value-minimized envelope for one atomically appended lifecycle transition.
///
/// `records` contains only entity IDs, local times/revisions, validity bounds,
/// and bounded reason/reference enums. It never stores provider state strings,
/// arbitrary attributes, credentials, URLs, or state-value hashes.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DurableLifecycleRecord {
    pub schema_version: u16,
    /// Strictly increasing global append-only ledger sequence, starting at 1.
    pub ledger_sequence: u64,
    /// Immutable profile-version reference used by this journal.
    pub profile_ref: String,
    pub transition: DurableLifecycleTransition,
    pub phase_before: LifecyclePhase,
    pub phase_after: LifecyclePhase,
    /// Caller-supplied epoch for this request, which may be stale on rejected input.
    pub requested_session_epoch: Option<u64>,
    pub active_session_epoch_before: Option<u64>,
    pub active_session_epoch_after: Option<u64>,
    /// Exact protocol subscription binding before/after this transition.
    pub active_subscription_id_before: Option<u64>,
    pub active_subscription_id_after: Option<u64>,
    /// Subscription ID supplied for this transition, if applicable.
    pub subscription_id: Option<u64>,
    pub at_micros: i64,
    pub failure: Option<LifecycleFailure>,
    pub records: Vec<ObservationLifecycleRecord>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleStoreError {
    ReadFailed,
    AppendAndSyncFailed,
}

/// A successful append means the *whole* record is appended and synced before
/// this method returns. An error is treated as outcome-unknown: the wrapper
/// latches unhealthy and requires a fresh recovery from the store.
pub trait ObservationLifecycleStore {
    fn load_all(&mut self) -> Result<Vec<DurableLifecycleRecord>, LifecycleStoreError>;
    fn append_and_sync(
        &mut self,
        record: &DurableLifecycleRecord,
    ) -> Result<(), LifecycleStoreError>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DurableLifecycleError {
    StorageReadFailed,
    StorageAppendAndSyncFailed,
    StorageFaulted,
    LedgerSequenceExhausted,
    InvalidJournal,
    InvalidCandidateTransition,
    InvalidCandidateRevision,
    CheckpointRejected,
}

/// Durable identity of a positive observation version. The ledger sequence
/// identifies the committed record that created this revision; the state value
/// and any hash of that value are intentionally absent.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DurablePositiveRevision {
    pub entity_id: String,
    pub profile_ref: String,
    pub session_epoch: u64,
    pub revision: u64,
    pub observed_at_micros: i64,
    pub valid_until_micros: i64,
    pub ledger_sequence: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DurableLifecycleUpdate {
    pub ledger_sequence: u64,
    pub update: LifecycleUpdate,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DurableCurrentnessCheck {
    pub ledger_sequence: u64,
    pub revision: Option<DurablePositiveRevision>,
    pub update: LifecycleUpdate,
}

pub struct DurableObservationLifecycle<S: ObservationLifecycleStore> {
    profile: HomeAssistantObservationProfile,
    inner: ObservationLifecycle,
    store: S,
    last_sequence: u64,
    current: BTreeMap<String, DurablePositiveRevision>,
    faulted: bool,
}

impl<S: ObservationLifecycleStore> DurableObservationLifecycle<S> {
    pub fn new(
        profile: &HomeAssistantObservationProfile,
        store: S,
    ) -> Result<Self, DurableLifecycleError> {
        let inner = ObservationLifecycle::new(profile)
            .map_err(|_| DurableLifecycleError::CheckpointRejected)?;
        Ok(Self {
            profile: profile.clone(),
            inner,
            store,
            last_sequence: 0,
            current: BTreeMap::new(),
            faulted: false,
        })
    }

    /// Validate all records, recover only revision/epoch high-water marks, and
    /// append a recovery barrier if the prior process had an open lifecycle.
    /// No positive observation is restored as current.
    pub fn recover(
        profile: &HomeAssistantObservationProfile,
        mut store: S,
        now_micros: i64,
    ) -> Result<Self, DurableLifecycleError> {
        let records = store
            .load_all()
            .map_err(|_| DurableLifecycleError::StorageReadFailed)?;
        if records.is_empty() {
            return Self::new(profile, store);
        }

        let replay = replay_records(profile, &records)?;
        let checkpoint = ObservationLifecycleCheckpointV1 {
            schema_version: crate::observation_lifecycle::OBSERVATION_LIFECYCLE_SCHEMA_VERSION,
            profile_ref: profile.profile_ref.clone(),
            highest_session_epoch: replay.highest_session_epoch,
            entity_revisions: replay.entity_revisions,
        };
        let inner = ObservationLifecycle::restore(profile, checkpoint)
            .map_err(|_| DurableLifecycleError::CheckpointRejected)?;
        let mut ledger = Self {
            profile: profile.clone(),
            inner,
            store,
            last_sequence: replay.last_sequence,
            current: BTreeMap::new(),
            faulted: false,
        };

        // Even a history ending in Live or Closed must be converted to a cold
        // start before this process can accept further input. If append-and-sync
        // fails, no usable ledger is returned.
        if replay.last_phase != LifecyclePhase::Dormant
            || replay.active_session_epoch.is_some()
        {
            let next = ledger
                .last_sequence
                .checked_add(1)
                .ok_or(DurableLifecycleError::LedgerSequenceExhausted)?;
            let barrier = DurableLifecycleRecord {
                schema_version: DURABLE_LIFECYCLE_JOURNAL_SCHEMA_VERSION,
                ledger_sequence: next,
                profile_ref: profile.profile_ref.clone(),
                transition: DurableLifecycleTransition::RecoveryBarrier,
                phase_before: replay.last_phase,
                phase_after: LifecyclePhase::Dormant,
                requested_session_epoch: None,
                active_session_epoch_before: replay.active_session_epoch,
                active_session_epoch_after: None,
                active_subscription_id_before: replay.active_subscription_id,
                active_subscription_id_after: None,
                subscription_id: None,
                at_micros: now_micros,
                failure: None,
                records: Vec::new(),
            };
            validate_transition(&barrier).map_err(|_| DurableLifecycleError::InvalidJournal)?;
            ledger
                .store
                .append_and_sync(&barrier)
                .map_err(|_| DurableLifecycleError::StorageAppendAndSyncFailed)?;
            ledger.last_sequence = next;
        }
        Ok(ledger)
    }

    pub fn phase(&self) -> LifecyclePhase {
        if self.faulted {
            // Do not expose a misleading Live phase after uncertain persistence.
            LifecyclePhase::NeedsNewSession
        } else {
            self.inner.phase()
        }
    }

    pub fn last_sequence(&self) -> u64 {
        self.last_sequence
    }

    pub fn is_healthy(&self) -> bool {
        !self.faulted
    }

    /// Read-only currentness view. It is suppressed after any ambiguous storage
    /// failure. Call `check_currentness` when a durable expiry/clock-fault
    /// transition is required before a consumer acts.
    pub fn current_revision(
        &self,
        entity_id: &str,
        now_micros: i64,
    ) -> Option<DurablePositiveRevision> {
        if self.faulted {
            return None;
        }
        let inner = self.inner.current_revision(entity_id, now_micros)?;
        let current = self.current.get(entity_id)?;
        if current.session_epoch != inner.session_epoch
            || current.revision != inner.revision
            || now_micros < current.observed_at_micros
            || now_micros >= current.valid_until_micros
        {
            return None;
        }
        Some(current.clone())
    }

    pub fn store(&self) -> &S {
        &self.store
    }

    /// Extract the store for repair/reopen after a storage failure. This does
    /// not make this ledger healthy; recover a new instance from the store.
    pub fn into_store(self) -> S {
        self.store
    }

    pub fn start_session(
        &mut self,
        session_epoch: u64,
        now_micros: i64,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError> {
        self.transact(
            DurableLifecycleTransition::SessionStart,
            Some(session_epoch),
            None,
            now_micros,
            |inner| inner.start_session(session_epoch, now_micros),
        )
    }

    pub fn bind_subscription(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        now_micros: i64,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError> {
        self.transact(
            DurableLifecycleTransition::SubscriptionBind,
            Some(session_epoch),
            Some(subscription_id),
            now_micros,
            |inner| inner.bind_subscription(session_epoch, subscription_id, now_micros),
        )
    }

    pub fn subscription_acknowledged(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        now_micros: i64,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError> {
        self.transact(
            DurableLifecycleTransition::SubscriptionAcknowledged,
            Some(session_epoch),
            Some(subscription_id),
            now_micros,
            |inner| {
                inner.subscription_acknowledged(session_epoch, subscription_id, now_micros)
            },
        )
    }

    pub fn apply_json(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        kind: EntityStreamKind,
        json_bytes: &[u8],
        now_micros: i64,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError> {
        let transition = match kind {
            EntityStreamKind::InitialSnapshot => DurableLifecycleTransition::InitialSnapshotReceived,
            EntityStreamKind::Changes => DurableLifecycleTransition::DeltaReceived,
        };
        self.transact(
            transition,
            Some(session_epoch),
            Some(subscription_id),
            now_micros,
            |inner| inner.apply_json(session_epoch, subscription_id, kind, json_bytes, now_micros),
        )
    }

    pub fn snapshot_applied(
        &mut self,
        session_epoch: u64,
        subscription_id: u64,
        now_micros: i64,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError> {
        self.transact(
            DurableLifecycleTransition::SnapshotApplied,
            Some(session_epoch),
            Some(subscription_id),
            now_micros,
            |inner| inner.snapshot_applied(session_epoch, subscription_id, now_micros),
        )
    }

    pub fn check_currentness(
        &mut self,
        entity_id: &str,
        now_micros: i64,
    ) -> Result<DurableCurrentnessCheck, DurableLifecycleError> {
        let committed = self.transact(
            DurableLifecycleTransition::CurrentnessCheck,
            self.inner.current_session_epoch(),
            None,
            now_micros,
            |inner| inner.check_currentness(entity_id, now_micros).update,
        )?;
        let revision = self.current_revision(entity_id, now_micros);
        Ok(DurableCurrentnessCheck {
            ledger_sequence: committed.ledger_sequence,
            revision,
            update: committed.update,
        })
    }

    pub fn expire(
        &mut self,
        now_micros: i64,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError> {
        self.transact(
            DurableLifecycleTransition::ExpirySweep,
            self.inner.current_session_epoch(),
            None,
            now_micros,
            |inner| inner.expire(now_micros),
        )
    }

    pub fn disconnect(
        &mut self,
        session_epoch: u64,
        now_micros: i64,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError> {
        self.transact(
            DurableLifecycleTransition::Disconnect,
            Some(session_epoch),
            None,
            now_micros,
            |inner| inner.disconnect(session_epoch, now_micros),
        )
    }

    fn transact<F>(
        &mut self,
        transition: DurableLifecycleTransition,
        requested_session_epoch: Option<u64>,
        subscription_id: Option<u64>,
        at_micros: i64,
        apply: F,
    ) -> Result<DurableLifecycleUpdate, DurableLifecycleError>
    where
        F: FnOnce(&mut ObservationLifecycle) -> LifecycleUpdate,
    {
        if self.faulted {
            return Err(DurableLifecycleError::StorageFaulted);
        }
        let sequence = match self.last_sequence.checked_add(1) {
            Some(sequence) => sequence,
            None => {
                self.faulted = true;
                return Err(DurableLifecycleError::LedgerSequenceExhausted);
            }
        };
        let phase_before = self.inner.phase();
        let active_epoch_before = self.inner.current_session_epoch();
        let active_subscription_before = self.inner.current_subscription_id();
        let mut candidate = self.inner.clone();
        let update = apply(&mut candidate);
        let record = DurableLifecycleRecord {
            schema_version: DURABLE_LIFECYCLE_JOURNAL_SCHEMA_VERSION,
            ledger_sequence: sequence,
            profile_ref: self.profile.profile_ref.clone(),
            transition,
            phase_before,
            phase_after: candidate.phase(),
            requested_session_epoch,
            active_session_epoch_before: active_epoch_before,
            active_session_epoch_after: candidate.current_session_epoch(),
            active_subscription_id_before: active_subscription_before,
            active_subscription_id_after: candidate.current_subscription_id(),
            subscription_id,
            at_micros,
            failure: update.failure,
            records: update.records.clone(),
        };
        if validate_transition(&record).is_err() {
            self.faulted = true;
            return Err(DurableLifecycleError::InvalidCandidateTransition);
        }
        let mut candidate_current = self.current.clone();
        if apply_entity_records(
            &mut candidate_current,
            &self.profile.profile_ref,
            sequence,
            &record.records,
        )
        .is_err()
        {
            self.faulted = true;
            return Err(DurableLifecycleError::InvalidCandidateRevision);
        }

        // No candidate lifecycle mutation or observation output becomes visible
        // unless the complete envelope and entity-record batch are durable.
        if self.store.append_and_sync(&record).is_err() {
            self.faulted = true;
            return Err(DurableLifecycleError::StorageAppendAndSyncFailed);
        }
        self.inner = candidate;
        self.current = candidate_current;
        self.last_sequence = sequence;
        Ok(DurableLifecycleUpdate {
            ledger_sequence: sequence,
            update,
        })
    }
}

struct ReplaySummary {
    last_sequence: u64,
    last_phase: LifecyclePhase,
    active_session_epoch: Option<u64>,
    active_subscription_id: Option<u64>,
    highest_session_epoch: Option<u64>,
    entity_revisions: BTreeMap<String, u64>,
}

fn replay_records(
    profile: &HomeAssistantObservationProfile,
    records: &[DurableLifecycleRecord],
) -> Result<ReplaySummary, DurableLifecycleError> {
    profile
        .validate()
        .map_err(|_| DurableLifecycleError::CheckpointRejected)?;
    let allowlist = profile
        .rules
        .iter()
        .map(|rule| rule.entity_id.clone())
        .collect::<BTreeSet<_>>();
    let mut expected_sequence = 1u64;
    let mut expected_phase = LifecyclePhase::Dormant;
    let mut active_session_epoch = None;
    let mut active_subscription_id = None;
    let mut highest_session_epoch = None;
    let mut entity_revisions = BTreeMap::<String, u64>::new();
    let mut active_positives = BTreeMap::<String, PositiveRevisionRef>::new();

    for record in records {
        if record.schema_version != DURABLE_LIFECYCLE_JOURNAL_SCHEMA_VERSION
            || record.ledger_sequence != expected_sequence
            || record.profile_ref != profile.profile_ref
            || record.phase_before != expected_phase
            || record.active_session_epoch_before != active_session_epoch
            || record.active_subscription_id_before != active_subscription_id
            || validate_transition(record).is_err()
        {
            return Err(DurableLifecycleError::InvalidJournal);
        }

        match record.transition {
            DurableLifecycleTransition::SessionStart if record.failure.is_none() => {
                let epoch = record
                    .requested_session_epoch
                    .ok_or(DurableLifecycleError::InvalidJournal)?;
                if epoch == 0
                    || epoch == u64::MAX
                    || highest_session_epoch.is_some_and(|old| epoch <= old)
                    || record.active_session_epoch_after != Some(epoch)
                {
                    return Err(DurableLifecycleError::InvalidJournal);
                }
                highest_session_epoch = Some(epoch);
            }
            DurableLifecycleTransition::SessionStart => {
                if record.active_session_epoch_after != active_session_epoch {
                    return Err(DurableLifecycleError::InvalidJournal);
                }
            }
            DurableLifecycleTransition::RecoveryBarrier => {
                if record.active_session_epoch_after.is_some()
                    || record.active_subscription_id_after.is_some()
                    || record.phase_after != LifecyclePhase::Dormant
                    || !record.records.is_empty()
                {
                    return Err(DurableLifecycleError::InvalidJournal);
                }
                // The durable audit history may end with positive records, but
                // recovery never imports that positive state as current.
                active_positives.clear();
                active_subscription_id = None;
            }
            _ => {
                if record.active_session_epoch_after != active_session_epoch {
                    return Err(DurableLifecycleError::InvalidJournal);
                }
            }
        }

        for lifecycle_record in &record.records {
            if !allowlist.contains(&lifecycle_record.entity_id)
                || lifecycle_record.revision == 0
                || lifecycle_record.revision > u64::MAX - 2
                || lifecycle_record.session_epoch != record.active_session_epoch_after.unwrap_or(0)
            {
                return Err(DurableLifecycleError::InvalidJournal);
            }
            let previous_revision = entity_revisions
                .get(&lifecycle_record.entity_id)
                .copied()
                .unwrap_or(0);
            if previous_revision.checked_add(1) != Some(lifecycle_record.revision) {
                return Err(DurableLifecycleError::InvalidJournal);
            }
            match lifecycle_record.kind {
                LifecycleRecordKind::PositiveCurrent { supersedes, .. } => {
                    let active = active_positives.get(&lifecycle_record.entity_id).copied();
                    match (active, supersedes) {
                        (Some(expected), Some(actual)) if expected == actual => {}
                        (None, None) => {}
                        _ => return Err(DurableLifecycleError::InvalidJournal),
                    }
                    active_positives.insert(
                        lifecycle_record.entity_id.clone(),
                        PositiveRevisionRef {
                            session_epoch: lifecycle_record.session_epoch,
                            revision: lifecycle_record.revision,
                        },
                    );
                }
                LifecycleRecordKind::Invalidated { previous_positive, .. } => {
                    if active_positives.get(&lifecycle_record.entity_id).copied()
                        != Some(previous_positive)
                    {
                        return Err(DurableLifecycleError::InvalidJournal);
                    }
                    active_positives.remove(&lifecycle_record.entity_id);
                }
            }
            entity_revisions.insert(lifecycle_record.entity_id.clone(), lifecycle_record.revision);
        }

        expected_sequence = expected_sequence
            .checked_add(1)
            .ok_or(DurableLifecycleError::InvalidJournal)?;
        expected_phase = record.phase_after;
        active_session_epoch = record.active_session_epoch_after;
        active_subscription_id = record.active_subscription_id_after;
    }

    Ok(ReplaySummary {
        last_sequence: expected_sequence - 1,
        last_phase: expected_phase,
        active_session_epoch,
        active_subscription_id,
        highest_session_epoch,
        entity_revisions,
    })
}

fn validate_transition(record: &DurableLifecycleRecord) -> Result<(), ()> {
    if record.schema_version != DURABLE_LIFECYCLE_JOURNAL_SCHEMA_VERSION
        || record.ledger_sequence == 0
        || record.profile_ref.trim().is_empty()
    {
        return Err(());
    }
    if record.transition == DurableLifecycleTransition::RecoveryBarrier {
        return if record.failure.is_none()
            && record.requested_session_epoch.is_none()
            && record.subscription_id.is_none()
            && record.phase_after == LifecyclePhase::Dormant
            && record.active_session_epoch_after.is_none()
            && record.records.is_empty()
        {
            Ok(())
        } else {
            Err(())
        };
    }

    if record.failure.is_some() {
        return if record.phase_after == record.phase_before
            || record.phase_after == LifecyclePhase::NeedsNewSession
        {
            Ok(())
        } else {
            Err(())
        };
    }

    let valid = match record.transition {
        DurableLifecycleTransition::SessionStart => {
            record.phase_after == LifecyclePhase::AwaitingSubscription
                && record.requested_session_epoch.is_some()
                && record.active_session_epoch_after == record.requested_session_epoch
        }
        DurableLifecycleTransition::SubscriptionBind => {
            record.phase_before == LifecyclePhase::AwaitingSubscription
                && record.phase_after == LifecyclePhase::AwaitingSubscriptionAck
                && record.subscription_id.is_some_and(|id| id > 0)
                && record.active_session_epoch_after == record.active_session_epoch_before
        }
        DurableLifecycleTransition::SubscriptionAcknowledged => {
            record.phase_before == LifecyclePhase::AwaitingSubscriptionAck
                && record.phase_after == LifecyclePhase::AwaitingSnapshot
                && record.subscription_id.is_some_and(|id| id > 0)
                && record.active_session_epoch_after == record.active_session_epoch_before
        }
        DurableLifecycleTransition::InitialSnapshotReceived => {
            record.phase_before == LifecyclePhase::AwaitingSnapshot
                && record.phase_after == LifecyclePhase::SnapshotPendingApply
                && record.subscription_id.is_some_and(|id| id > 0)
                && record.active_session_epoch_after == record.active_session_epoch_before
        }
        DurableLifecycleTransition::DeltaReceived => {
            record.phase_before == LifecyclePhase::Live
                && record.phase_after == LifecyclePhase::Live
                && record.subscription_id.is_some_and(|id| id > 0)
                && record.active_session_epoch_after == record.active_session_epoch_before
        }
        DurableLifecycleTransition::SnapshotApplied => {
            record.phase_before == LifecyclePhase::SnapshotPendingApply
                && record.phase_after == LifecyclePhase::Live
                && record.subscription_id.is_some_and(|id| id > 0)
                && record.active_session_epoch_after == record.active_session_epoch_before
        }
        DurableLifecycleTransition::CurrentnessCheck | DurableLifecycleTransition::ExpirySweep => {
            record.phase_before == record.phase_after
                && record.active_session_epoch_after == record.active_session_epoch_before
        }
        DurableLifecycleTransition::Disconnect => {
            record.phase_after == LifecyclePhase::Closed
                && record.active_session_epoch_after == record.active_session_epoch_before
        }
        DurableLifecycleTransition::RecoveryBarrier => false,
    };
    if valid { Ok(()) } else { Err(()) }
}

fn apply_entity_records(
    current: &mut BTreeMap<String, DurablePositiveRevision>,
    profile_ref: &str,
    ledger_sequence: u64,
    records: &[ObservationLifecycleRecord],
) -> Result<(), ()> {
    for record in records {
        match record.kind {
            LifecycleRecordKind::PositiveCurrent {
                valid_until_micros,
                supersedes,
            } => {
                if valid_until_micros < record.at_micros {
                    return Err(());
                }
                match (current.get(&record.entity_id), supersedes) {
                    (Some(old), Some(reference))
                        if old.session_epoch == reference.session_epoch
                            && old.revision == reference.revision => {}
                    (None, None) => {}
                    _ => return Err(()),
                }
                current.insert(
                    record.entity_id.clone(),
                    DurablePositiveRevision {
                        entity_id: record.entity_id.clone(),
                        profile_ref: profile_ref.to_owned(),
                        session_epoch: record.session_epoch,
                        revision: record.revision,
                        observed_at_micros: record.at_micros,
                        valid_until_micros,
                        ledger_sequence,
                    },
                );
            }
            LifecycleRecordKind::Invalidated {
                previous_positive, ..
            } => {
                let Some(old) = current.get(&record.entity_id) else {
                    return Err(());
                };
                if old.session_epoch != previous_positive.session_epoch
                    || old.revision != previous_positive.revision
                {
                    return Err(());
                }
                current.remove(&record.entity_id);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EntityObservationRule, EntityRef, StateCodec};
    use serde_json::{json, Value};
    use std::collections::BTreeSet;

    #[derive(Debug, Clone, Default)]
    struct MemoryStore {
        records: Vec<DurableLifecycleRecord>,
        fail_next_append: bool,
        fail_reads: bool,
    }

    impl ObservationLifecycleStore for MemoryStore {
        fn load_all(&mut self) -> Result<Vec<DurableLifecycleRecord>, LifecycleStoreError> {
            if self.fail_reads {
                return Err(LifecycleStoreError::ReadFailed);
            }
            Ok(self.records.clone())
        }

        fn append_and_sync(
            &mut self,
            record: &DurableLifecycleRecord,
        ) -> Result<(), LifecycleStoreError> {
            if self.fail_next_append {
                self.fail_next_append = false;
                return Err(LifecycleStoreError::AppendAndSyncFailed);
            }
            self.records.push(record.clone());
            Ok(())
        }
    }

    fn profile() -> HomeAssistantObservationProfile {
        HomeAssistantObservationProfile {
            schema_version: 1,
            instance_id: "home-main".into(),
            profile_ref: "profile:ha-v1".into(),
            rules: vec![EntityObservationRule {
                entity_id: "sensor.living_room_temperature".into(),
                subject: crate::EntityRef {
                    kind: "room".into(),
                    id: "living-room".into(),
                },
                attribute: "temperature_celsius".into(),
                codec: StateCodec::FixedRange {
                    scale: 2,
                    min_mantissa: -5_000,
                    max_mantissa: 8_000,
                },
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

    fn make_live() -> DurableObservationLifecycle<MemoryStore> {
        let mut ledger = DurableObservationLifecycle::new(&profile(), MemoryStore::default()).unwrap();
        assert_eq!(
            ledger.start_session(1, 10).unwrap().update.phase,
            LifecyclePhase::AwaitingSubscription
        );
        ledger.bind_subscription(1, 7, 11).unwrap();
        ledger.subscription_acknowledged(1, 7, 12).unwrap();
        let snapshot = ledger.apply_json(
            1,
            7,
            EntityStreamKind::InitialSnapshot,
            &bytes(snapshot("21.50")),
            20,
        ).unwrap();
        assert_eq!(snapshot.update.phase, LifecyclePhase::SnapshotPendingApply);
        assert!(ledger.current_revision("sensor.living_room_temperature", 20).is_none());
        let applied = ledger.snapshot_applied(1, 7, 21).unwrap();
        assert_eq!(applied.update.phase, LifecyclePhase::Live);
        assert_eq!(applied.update.observations.len(), 1);
        assert!(ledger.current_revision("sensor.living_room_temperature", 21).is_some());
        ledger
    }

    #[test]
    fn currentness_is_published_only_after_append_and_sync() {
        let ledger = make_live();
        let current = ledger.current_revision("sensor.living_room_temperature", 21).unwrap();
        assert_eq!(current.profile_ref, "profile:ha-v1");
        assert_eq!(current.session_epoch, 1);
        assert_eq!(current.revision, 1);
        assert_eq!(current.ledger_sequence, 5);
        assert_eq!(ledger.last_sequence(), 5);
        assert!(ledger.is_healthy());
    }

    #[test]
    fn semantic_replay_does_not_extend_freshness_or_replace_revision() {
        let mut ledger = make_live();
        let before = ledger.current_revision("sensor.living_room_temperature", 21).unwrap();
        let replay = ledger.apply_json(
            1,
            7,
            EntityStreamKind::Changes,
            &bytes(change("21.50")),
            25,
        ).unwrap();
        assert!(replay.update.observations.is_empty());
        assert!(replay.update.records.is_empty());
        let after = ledger.current_revision("sensor.living_room_temperature", 25).unwrap();
        assert_eq!(after, before);
        assert_eq!(after.valid_until_micros, before.valid_until_micros);
        // The lifecycle transition is still sequenced, but it creates no new
        // positive revision and does not change the revision's creating record.
        assert_eq!(after.ledger_sequence, 5);
        assert_eq!(ledger.last_sequence(), 6);
    }

    #[test]
    fn disconnect_is_durable_before_positive_currentness_is_revoked() {
        let mut ledger = make_live();
        let before = ledger.current_revision("sensor.living_room_temperature", 21).unwrap();
        let old_records = ledger.store().records.len();
        let update = ledger.disconnect(1, 30).unwrap();
        assert_eq!(update.update.phase, LifecyclePhase::Closed);
        assert!(update.update.observations.is_empty());
        assert_eq!(ledger.store().records.len(), old_records + 1);
        assert_eq!(ledger.store().records.last().unwrap().records.len(), 1);
        assert!(ledger.current_revision("sensor.living_room_temperature", 30).is_none());
        assert_eq!(before.revision, 1);
    }

    #[test]
    fn append_and_sync_failure_keeps_candidate_unpublished_and_latches_closed() {
        let mut ledger = make_live();
        let current_before = ledger.current.get("sensor.living_room_temperature").unwrap().clone();
        let sequence_before = ledger.last_sequence();
        let phase_before = ledger.phase();
        ledger.store.fail_next_append = true;

        assert_eq!(
            ledger.disconnect(1, 30),
            Err(DurableLifecycleError::StorageAppendAndSyncFailed)
        );
        assert_eq!(ledger.last_sequence(), sequence_before);
        assert_eq!(ledger.inner.phase(), phase_before);
        assert_eq!(ledger.phase(), LifecyclePhase::NeedsNewSession);
        assert_eq!(
            ledger.current.get("sensor.living_room_temperature"),
            Some(&current_before)
        );
        assert!(ledger.current_revision("sensor.living_room_temperature", 30).is_none());
        assert!(!ledger.is_healthy());
        assert_eq!(
            ledger.start_session(2, 40),
            Err(DurableLifecycleError::StorageFaulted)
        );
    }

    #[test]
    fn recovery_verifies_records_then_requires_new_epoch_and_snapshot() {
        let mut ledger = make_live();
        let old = ledger.current_revision("sensor.living_room_temperature", 21).unwrap();
        let mut store = ledger.into_store();
        let recovered = DurableObservationLifecycle::recover(&profile(), store.clone(), 50).unwrap();
        assert_eq!(recovered.phase(), LifecyclePhase::Dormant);
        assert!(recovered.current_revision("sensor.living_room_temperature", 51).is_none());
        assert_eq!(recovered.last_sequence(), 6, "recovery barrier follows five lifecycle transactions");
        assert_eq!(
            recovered.store().records.last().unwrap().transition,
            DurableLifecycleTransition::RecoveryBarrier
        );
        let mut recovered = recovered;
        assert_eq!(
            recovered.start_session(1, 60).unwrap().update.failure,
            Some(LifecycleFailure::EpochNotIncreasing)
        );
        assert!(recovered.start_session(2, 61).is_ok());
        assert!(recovered.current_revision("sensor.living_room_temperature", 61).is_none());
        assert_eq!(old.revision, 1);
        store = recovered.into_store();
        assert!(DurableObservationLifecycle::recover(&profile(), store, 70).is_ok());
    }

    #[test]
    fn recovery_rejects_sequence_gaps_duplicates_unknown_schema_and_bad_transitions() {
        let ledger = make_live();
        let base_store = ledger.store().clone();

        let mut gap = base_store.clone();
        gap.records[1].ledger_sequence = 3;
        assert_eq!(
            DurableObservationLifecycle::recover(&profile(), gap, 50).err(),
            Some(DurableLifecycleError::InvalidJournal)
        );

        let mut duplicate = base_store.clone();
        duplicate.records[1].ledger_sequence = duplicate.records[0].ledger_sequence;
        assert_eq!(
            DurableObservationLifecycle::recover(&profile(), duplicate, 50).err(),
            Some(DurableLifecycleError::InvalidJournal)
        );

        let mut unknown_schema = base_store.clone();
        unknown_schema.records[0].schema_version = 999;
        assert_eq!(
            DurableObservationLifecycle::recover(&profile(), unknown_schema, 50).err(),
            Some(DurableLifecycleError::InvalidJournal)
        );

        let mut invalid_transition = base_store;
        invalid_transition.records[1].phase_after = LifecyclePhase::Live;
        assert_eq!(
            DurableObservationLifecycle::recover(&profile(), invalid_transition, 50).err(),
            Some(DurableLifecycleError::InvalidJournal)
        );
    }

    #[test]
    fn replay_rejects_invalidation_that_does_not_name_the_exact_active_positive() {
        let ledger = make_live();
        let mut store = ledger.into_store();
        store.records.push(DurableLifecycleRecord {
            schema_version: DURABLE_LIFECYCLE_JOURNAL_SCHEMA_VERSION,
            ledger_sequence: 6,
            profile_ref: profile().profile_ref,
            transition: DurableLifecycleTransition::Disconnect,
            phase_before: LifecyclePhase::Live,
            phase_after: LifecyclePhase::Closed,
            requested_session_epoch: Some(1),
            active_session_epoch_before: Some(1),
            active_session_epoch_after: Some(1),
            active_subscription_id_before: Some(1),
            active_subscription_id_after: None,
            subscription_id: None,
            at_micros: 30,
            failure: None,
            records: vec![ObservationLifecycleRecord {
                entity_id: "sensor.living_room_temperature".into(),
                session_epoch: 1,
                revision: 2,
                at_micros: 30,
                kind: LifecycleRecordKind::Invalidated {
                    reason: InvalidationReason::TransportDisconnected,
                    previous_positive: PositiveRevisionRef { session_epoch: 1, revision: 999 },
                },
            }],
        });
        assert_eq!(
            DurableObservationLifecycle::recover(&profile(), store, 40).err(),
            Some(DurableLifecycleError::InvalidJournal)
        );
    }

    #[test]
    fn journal_records_do_not_store_state_values_or_value_hashes() {
        let ledger = make_live();
        let serialized = serde_json::to_string(ledger.store().records.as_slice()).unwrap();
        assert!(!serialized.contains("21.50"));
        assert!(!serialized.contains("unit_of_measurement"));
        assert!(!serialized.contains("device_class"));
        assert!(!serialized.contains("access_token"));
    }

    #[test]
    fn currentness_query_after_storage_failure_is_always_withheld() {
        let mut ledger = make_live();
        ledger.store.fail_next_append = true;
        assert_eq!(
            ledger.expire(40_000_000),
            Err(DurableLifecycleError::StorageAppendAndSyncFailed)
        );
        assert!(!ledger.is_healthy());
        assert!(ledger.current_revision("sensor.living_room_temperature", 40_000_001).is_none());
    }
}

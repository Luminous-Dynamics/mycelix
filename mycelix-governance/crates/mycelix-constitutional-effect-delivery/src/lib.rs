// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Sink-neutral durable constitutional effect delivery core.
//!
//! The semantic binding is supplied by an already-qualified identity layer.
//! This crate never invokes a real external sink.

#![forbid(unsafe_code)]

use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::fmt;

pub const DELIVERY_SCHEMA_VERSION: u16 = 2;
pub const MAX_ATTEMPTS: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReplayProfile {
    NoAutomaticRetry,
    IdempotentByEffectInstance,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeliveryState {
    Pending,
    AttemptStarted,
    OutcomeUnknown,
    KnownSuccess,
    KnownNoEffect,
    IntegrityHalted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HaltReason {
    BindingDrift,
    AttemptProvenance,
    ContradictorySemanticEvidence,
    InvalidReconciliationProvenance,
    InvariantViolation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct HaltProvenance {
    reason: HaltReason,
    observation_sequence: Option<u64>,
}

impl HaltProvenance {
    const fn reason(reason: HaltReason) -> Self {
        Self {
            reason,
            observation_sequence: None,
        }
    }

    const fn contradictory(sequence: u64) -> Self {
        Self {
            reason: HaltReason::ContradictorySemanticEvidence,
            observation_sequence: Some(sequence),
        }
    }

    pub const fn reason_code(self) -> HaltReason {
        self.reason
    }

    pub const fn observation_sequence(self) -> Option<u64> {
        self.observation_sequence
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservationKind {
    TransportAccepted,
    ProviderAcknowledged,
    SemanticSuccess,
    OutcomeUnknown,
    ReconciliationSuccess,
    ReconciliationNoEffect,
}

impl ObservationKind {
    pub const fn is_semantic_success(self) -> bool {
        matches!(self, Self::SemanticSuccess | Self::ReconciliationSuccess)
    }

    pub const fn is_no_effect(self) -> bool {
        matches!(self, Self::ReconciliationNoEffect)
    }

    pub const fn is_unknown(self) -> bool {
        matches!(self, Self::OutcomeUnknown)
    }
}

/// The upstream semantic identity. The delivery layer neither derives nor
/// replaces any of these components.
pub trait EffectBinding: Clone + Eq + Serialize + DeserializeOwned {
    type InstanceRoot: Clone + Eq;
    type ContractRoot: Clone + Eq;
    type PayloadCommitment: Clone + Eq;
    type SinkRoot: Clone + Eq;
    type SinkEpoch: Clone + Eq;

    fn effect_instance_root(&self) -> &Self::InstanceRoot;
    fn effect_contract_root(&self) -> &Self::ContractRoot;
    fn payload_commitment(&self) -> &Self::PayloadCommitment;
    fn sink_root(&self) -> &Self::SinkRoot;
    fn sink_epoch(&self) -> &Self::SinkEpoch;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct AttemptId(u64);

impl AttemptId {
    pub const fn raw(self) -> u64 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IntentCommitOutcome {
    Committed,
    DefinitelyNotCommitted,
    CommitOutcomeUnknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreparedDeliveryIntent<B: EffectBinding> {
    binding: B,
    replay_profile: ReplayProfile,
}

impl<B: EffectBinding> PreparedDeliveryIntent<B> {
    pub fn new(binding: B, replay_profile: ReplayProfile) -> Self {
        Self { binding, replay_profile }
    }

    /// Call only after the owning durable store reports a definite commit.
    fn into_committed(self) -> CommittedDeliveryIntent<B> {
        CommittedDeliveryIntent {
            binding: self.binding,
            replay_profile: self.replay_profile,
        }
    }

    /// Unknown persistence truth grants no delivery authority.
    pub fn apply_commit_outcome(
        self,
        outcome: IntentCommitOutcome,
    ) -> Option<CommittedDeliveryIntent<B>> {
        match outcome {
            IntentCommitOutcome::Committed => Some(self.into_committed()),
            IntentCommitOutcome::DefinitelyNotCommitted
            | IntentCommitOutcome::CommitOutcomeUnknown => None,
        }
    }

    pub fn binding(&self) -> &B {
        &self.binding
    }

    pub const fn replay_profile(&self) -> ReplayProfile {
        self.replay_profile
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommittedDeliveryIntent<B: EffectBinding> {
    binding: B,
    replay_profile: ReplayProfile,
}

impl<B: EffectBinding> CommittedDeliveryIntent<B> {
    pub fn binding(&self) -> &B {
        &self.binding
    }

    pub const fn replay_profile(&self) -> ReplayProfile {
        self.replay_profile
    }

    pub fn start_record(self) -> DeliveryRecord<B> {
        DeliveryRecord {
            binding: self.binding,
            replay_profile: self.replay_profile,
            state: DeliveryState::Pending,
            attempts: Vec::new(),
            observations: Vec::new(),
            completion_committed: false,
            caller_acknowledged: false,
            next_observation_sequence: 1,
            halt_provenance: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Attempt<B: EffectBinding> {
    id: AttemptId,
    binding: B,
}

impl<B: EffectBinding> Attempt<B> {
    pub const fn id(&self) -> AttemptId {
        self.id
    }

    pub fn binding(&self) -> &B {
        &self.binding
    }
}

/// A dispatch permit contains the exact attempt identity but has no external
/// I/O capability. The adapter must revalidate it immediately before dispatch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DispatchPermit<B: EffectBinding> {
    attempt: Attempt<B>,
}

impl<B: EffectBinding> DispatchPermit<B> {
    pub const fn attempt_id(&self) -> AttemptId {
        self.attempt.id
    }

    pub fn binding(&self) -> &B {
        &self.attempt.binding
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Observation<B: EffectBinding> {
    binding: B,
    attempt_id: AttemptId,
    sequence: u64,
    kind: ObservationKind,
    evidence_id: u64,
    reconciles_sequence: Option<u64>,
}

impl<B: EffectBinding> Observation<B> {
    pub fn transport_accepted(binding: B, attempt_id: AttemptId, evidence_id: u64) -> Self {
        Self::new(binding, attempt_id, ObservationKind::TransportAccepted, evidence_id, None)
    }

    pub fn provider_acknowledged(
        binding: B,
        attempt_id: AttemptId,
        evidence_id: u64,
    ) -> Self {
        Self::new(binding, attempt_id, ObservationKind::ProviderAcknowledged, evidence_id, None)
    }

    pub fn semantic_success(binding: B, attempt_id: AttemptId, evidence_id: u64) -> Self {
        Self::new(binding, attempt_id, ObservationKind::SemanticSuccess, evidence_id, None)
    }

    pub fn outcome_unknown(binding: B, attempt_id: AttemptId, evidence_id: u64) -> Self {
        Self::new(binding, attempt_id, ObservationKind::OutcomeUnknown, evidence_id, None)
    }

    pub fn reconciliation_success(
        binding: B,
        attempt_id: AttemptId,
        evidence_id: u64,
        reconciles_sequence: u64,
    ) -> Self {
        Self::new(
            binding,
            attempt_id,
            ObservationKind::ReconciliationSuccess,
            evidence_id,
            Some(reconciles_sequence),
        )
    }

    pub fn reconciliation_no_effect(
        binding: B,
        attempt_id: AttemptId,
        evidence_id: u64,
        reconciles_sequence: u64,
    ) -> Self {
        Self::new(
            binding,
            attempt_id,
            ObservationKind::ReconciliationNoEffect,
            evidence_id,
            Some(reconciles_sequence),
        )
    }

    fn new(
        binding: B,
        attempt_id: AttemptId,
        kind: ObservationKind,
        evidence_id: u64,
        reconciles_sequence: Option<u64>,
    ) -> Self {
        Self {
            binding,
            attempt_id,
            sequence: 0,
            kind,
            evidence_id,
            reconciles_sequence,
        }
    }

    fn with_sequence(mut self, sequence: u64) -> Self {
        self.sequence = sequence;
        self
    }

    pub fn binding(&self) -> &B {
        &self.binding
    }

    pub const fn attempt_id(&self) -> AttemptId {
        self.attempt_id
    }

    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    pub const fn kind(&self) -> ObservationKind {
        self.kind
    }

    pub const fn evidence_id(&self) -> u64 {
        self.evidence_id
    }

    pub const fn reconciles_sequence(&self) -> Option<u64> {
        self.reconciles_sequence
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeliverySnapshot<B: EffectBinding> {
    schema_version: u16,
    binding: B,
    replay_profile: ReplayProfile,
    state: DeliveryState,
    attempts: Vec<Attempt<B>>,
    observations: Vec<Observation<B>>,
    completion_committed: bool,
    halt_provenance: Option<HaltProvenance>,
}

impl<B: EffectBinding> DeliverySnapshot<B> {
    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub fn binding(&self) -> &B {
        &self.binding
    }

    pub const fn state(&self) -> DeliveryState {
        self.state
    }

    pub const fn replay_profile(&self) -> ReplayProfile {
        self.replay_profile
    }

    pub fn attempts(&self) -> &[Attempt<B>] {
        &self.attempts
    }

    pub fn observations(&self) -> &[Observation<B>] {
        &self.observations
    }

    pub const fn completion_committed(&self) -> bool {
        self.completion_committed
    }

    pub const fn halt_provenance(&self) -> Option<HaltProvenance> {
        self.halt_provenance
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservationResult {
    Applied { state: DeliveryState, sequence: u64 },
    Duplicate { sequence: u64 },
    IntegrityHalted { sequence: u64 },
}

impl ObservationResult {
    pub const fn sequence(self) -> u64 {
        match self {
            Self::Applied { sequence, .. }
            | Self::Duplicate { sequence }
            | Self::IntegrityHalted { sequence } => sequence,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DeliveryError {
    InvalidSchemaVersion(u16),
    BindingMismatch,
    WrongAttempt,
    InitialAttemptAlreadyStarted,
    RetryBlocked,
    RetryNotPermitted(DeliveryState),
    RetryAfterSuccess,
    ReconciliationRequiresUnknown,
    ReconciliationTargetMissing(u64),
    ReconciliationTargetNotUnknown(u64),
    ObservationSequenceOverflow,
    CompletionRequiresKnownSuccess,
    AlreadyCompleted,
    SnapshotInvariantViolation(&'static str),
    AttemptLimitExceeded,
    HaltProvenanceMissing,
    HaltProvenanceUnexpected,
    HaltProvenanceMismatch,
}

impl fmt::Display for DeliveryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSchemaVersion(v) => write!(f, "unsupported delivery schema version {v}"),
            Self::BindingMismatch => write!(f, "semantic effect binding mismatch"),
            Self::WrongAttempt => write!(f, "observation references an unknown attempt"),
            Self::InitialAttemptAlreadyStarted => write!(f, "initial attempt already exists"),
            Self::RetryBlocked => write!(f, "automatic retry is blocked for an unknown outcome"),
            Self::RetryNotPermitted(state) => write!(f, "retry is not permitted from {state:?}"),
            Self::RetryAfterSuccess => write!(f, "known-successful effect cannot be retried"),
            Self::ReconciliationRequiresUnknown => write!(f, "reconciliation is not valid from this state"),
            Self::ReconciliationTargetMissing(seq) => write!(f, "reconciliation target {seq} is missing"),
            Self::ReconciliationTargetNotUnknown(seq) => {
                write!(f, "reconciliation target {seq} has the wrong evidence kind")
            }
            Self::ObservationSequenceOverflow => write!(f, "observation sequence overflow"),
            Self::CompletionRequiresKnownSuccess => {
                write!(f, "durable completion requires KnownSuccess")
            }
            Self::AlreadyCompleted => write!(f, "durable completion is already committed"),
            Self::SnapshotInvariantViolation(msg) => write!(f, "snapshot invariant violation: {msg}"),
            Self::AttemptLimitExceeded => write!(f, "maximum attempt count exceeded"),
            Self::HaltProvenanceMissing => {
                write!(f, "integrity halt is missing durable halt provenance")
            }
            Self::HaltProvenanceUnexpected => {
                write!(f, "non-halted delivery state contains halt provenance")
            }
            Self::HaltProvenanceMismatch => {
                write!(f, "durable halt provenance does not match the halt history")
            }
        }
    }
}

impl std::error::Error for DeliveryError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeliveryRecord<B: EffectBinding> {
    binding: B,
    replay_profile: ReplayProfile,
    state: DeliveryState,
    attempts: Vec<Attempt<B>>,
    observations: Vec<Observation<B>>,
    completion_committed: bool,
    caller_acknowledged: bool,
    next_observation_sequence: u64,
    halt_provenance: Option<HaltProvenance>,
}

impl<B: EffectBinding> DeliveryRecord<B> {
    pub fn binding(&self) -> &B {
        &self.binding
    }

    pub const fn state(&self) -> DeliveryState {
        self.state
    }

    pub const fn replay_profile(&self) -> ReplayProfile {
        self.replay_profile
    }

    pub fn attempts(&self) -> &[Attempt<B>] {
        &self.attempts
    }

    pub fn observations(&self) -> &[Observation<B>] {
        &self.observations
    }

    pub const fn completion_committed(&self) -> bool {
        self.completion_committed
    }

    pub const fn halt_provenance(&self) -> Option<HaltProvenance> {
        self.halt_provenance
    }

    pub const fn caller_acknowledged(&self) -> bool {
        self.caller_acknowledged
    }

    pub fn start_initial_attempt(
        &mut self,
        binding: &B,
    ) -> Result<DispatchPermit<B>, DeliveryError> {
        self.require_binding(binding)?;
        if self.state == DeliveryState::IntegrityHalted {
            return Err(DeliveryError::RetryNotPermitted(self.state));
        }
        if self.state != DeliveryState::Pending || !self.attempts.is_empty() {
            return Err(DeliveryError::InitialAttemptAlreadyStarted);
        }
        self.create_attempt()
    }

    /// Revalidate the permit immediately before external dispatch.
    pub fn authorize_dispatch(
        &mut self,
        permit: &DispatchPermit<B>,
    ) -> Result<(), DeliveryError> {
        if self.state != DeliveryState::AttemptStarted {
            return Err(DeliveryError::RetryNotPermitted(self.state));
        }
        if permit.attempt.binding != self.binding {
            self.enter_halt(HaltProvenance::reason(HaltReason::BindingDrift));
            return Err(DeliveryError::BindingMismatch);
        }
        if self.attempts.last().map(|attempt| attempt.id) != Some(permit.attempt.id) {
            self.enter_halt(HaltProvenance::reason(HaltReason::AttemptProvenance));
            return Err(DeliveryError::WrongAttempt);
        }
        Ok(())
    }

    pub fn retry_same_effect(
        &mut self,
        binding: &B,
    ) -> Result<DispatchPermit<B>, DeliveryError> {
        self.require_binding(binding)?;
        match self.state {
            DeliveryState::OutcomeUnknown => {
                if self.replay_profile != ReplayProfile::IdempotentByEffectInstance {
                    return Err(DeliveryError::RetryBlocked);
                }
                self.create_attempt()
            }
            DeliveryState::KnownSuccess => Err(DeliveryError::RetryAfterSuccess),
            state => Err(DeliveryError::RetryNotPermitted(state)),
        }
    }

    pub fn retry_after_no_effect(
        &mut self,
        binding: &B,
    ) -> Result<DispatchPermit<B>, DeliveryError> {
        self.require_binding(binding)?;
        if self.state == DeliveryState::KnownSuccess {
            return Err(DeliveryError::RetryAfterSuccess);
        }
        if self.state != DeliveryState::KnownNoEffect {
            return Err(DeliveryError::RetryNotPermitted(self.state));
        }
        if self.replay_profile != ReplayProfile::IdempotentByEffectInstance {
            return Err(DeliveryError::RetryNotPermitted(self.state));
        }
        self.create_attempt()
    }

    fn create_attempt(&mut self) -> Result<DispatchPermit<B>, DeliveryError> {
        if self.attempts.len() >= MAX_ATTEMPTS {
            return Err(DeliveryError::AttemptLimitExceeded);
        }
        let id = AttemptId((self.attempts.len() + 1) as u64);
        let attempt = Attempt {
            id,
            binding: self.binding.clone(),
        };
        self.attempts.push(attempt.clone());
        self.state = DeliveryState::AttemptStarted;
        Ok(DispatchPermit { attempt })
    }

    pub fn observe(
        &mut self,
        observation: Observation<B>,
    ) -> Result<ObservationResult, DeliveryError> {
        if self.binding != observation.binding {
            self.enter_halt(HaltProvenance::reason(HaltReason::BindingDrift));
            return Err(DeliveryError::BindingMismatch);
        }
        if !self
            .attempts
            .iter()
            .any(|attempt| attempt.id == observation.attempt_id)
        {
            self.enter_halt(HaltProvenance::reason(HaltReason::AttemptProvenance));
            return Err(DeliveryError::WrongAttempt);
        }

        if let Some(existing) = self.observations.iter().find(|item| {
            item.binding == observation.binding
                && item.attempt_id == observation.attempt_id
                && item.kind == observation.kind
                && item.evidence_id == observation.evidence_id
                && item.reconciles_sequence == observation.reconciles_sequence
        }) {
            return Ok(ObservationResult::Duplicate {
                sequence: existing.sequence,
            });
        }

        if matches!(
            observation.kind,
            ObservationKind::ReconciliationSuccess | ObservationKind::ReconciliationNoEffect
        ) {
            if let Err(error) = self.validate_reconciliation(&observation) {
                match error {
                    DeliveryError::WrongAttempt => {
                        self.enter_halt(HaltProvenance::reason(HaltReason::AttemptProvenance));
                    }
                    DeliveryError::ReconciliationTargetMissing(_)
                    | DeliveryError::ReconciliationTargetNotUnknown(_) => {
                        self.enter_halt(HaltProvenance::reason(
                            HaltReason::InvalidReconciliationProvenance,
                        ));
                    }
                    _ => return Err(error),
                }
                return Err(error);
            }
        }

        let sequence = self.next_observation_sequence;
        self.next_observation_sequence = match sequence.checked_add(1) {
            Some(next) => next,
            None => {
                self.enter_halt(HaltProvenance::reason(HaltReason::InvariantViolation));
                return Err(DeliveryError::ObservationSequenceOverflow);
            }
        };

        let committed = observation.with_sequence(sequence);
        let previous_state = self.state;
        let next_state = self.transition_for_observation(&committed);
        if next_state == DeliveryState::IntegrityHalted
            && previous_state != DeliveryState::IntegrityHalted
        {
            self.halt_provenance = Some(HaltProvenance::contradictory(sequence));
        }
        self.state = next_state;
        self.observations.push(committed);

        if self.state == DeliveryState::IntegrityHalted {
            Ok(ObservationResult::IntegrityHalted { sequence })
        } else {
            Ok(ObservationResult::Applied {
                state: self.state,
                sequence,
            })
        }
    }

    fn transition_for_observation(&self, observation: &Observation<B>) -> DeliveryState {
        match observation.kind {
            ObservationKind::TransportAccepted | ObservationKind::ProviderAcknowledged => self.state,
            ObservationKind::OutcomeUnknown => match self.state {
                DeliveryState::AttemptStarted | DeliveryState::OutcomeUnknown => {
                    DeliveryState::OutcomeUnknown
                }
                _ => self.state,
            },
            ObservationKind::SemanticSuccess | ObservationKind::ReconciliationSuccess => {
                match self.state {
                    DeliveryState::KnownNoEffect | DeliveryState::IntegrityHalted => {
                        DeliveryState::IntegrityHalted
                    }
                    _ => DeliveryState::KnownSuccess,
                }
            }
            ObservationKind::ReconciliationNoEffect => match self.state {
                DeliveryState::KnownSuccess | DeliveryState::IntegrityHalted => {
                    DeliveryState::IntegrityHalted
                }
                _ => DeliveryState::KnownNoEffect,
            },
        }
    }

    fn validate_reconciliation(
        &self,
        observation: &Observation<B>,
    ) -> Result<(), DeliveryError> {
        if self.state == DeliveryState::IntegrityHalted {
            return Ok(());
        }

        let target = observation
            .reconciles_sequence
            .ok_or(DeliveryError::ReconciliationRequiresUnknown)?;
        let target_observation = self
            .observations
            .iter()
            .find(|item| item.sequence == target)
            .ok_or(DeliveryError::ReconciliationTargetMissing(target))?;

        if target_observation.attempt_id != observation.attempt_id {
            return Err(DeliveryError::WrongAttempt);
        }

        match self.state {
            DeliveryState::OutcomeUnknown => {
                if !target_observation.kind.is_unknown() {
                    return Err(DeliveryError::ReconciliationTargetNotUnknown(target));
                }
            }
            DeliveryState::KnownSuccess => {
                if !observation.kind.is_no_effect()
                    || !target_observation.kind.is_semantic_success()
                {
                    return Err(DeliveryError::ReconciliationTargetNotUnknown(target));
                }
            }
            DeliveryState::KnownNoEffect => {
                if !observation.kind.is_semantic_success()
                    || !target_observation.kind.is_no_effect()
                {
                    return Err(DeliveryError::ReconciliationTargetNotUnknown(target));
                }
            }
            DeliveryState::Pending | DeliveryState::AttemptStarted => {
                return Err(DeliveryError::ReconciliationRequiresUnknown);
            }
            DeliveryState::IntegrityHalted => {}
        }
        Ok(())
    }

    fn require_binding(&mut self, binding: &B) -> Result<(), DeliveryError> {
        if &self.binding == binding {
            Ok(())
        } else {
            self.enter_halt(HaltProvenance::reason(HaltReason::BindingDrift));
            Err(DeliveryError::BindingMismatch)
        }
    }

    fn enter_halt(&mut self, provenance: HaltProvenance) {
        self.state = DeliveryState::IntegrityHalted;
        if self.halt_provenance.is_none() {
            self.halt_provenance = Some(provenance);
        }
    }

    pub fn commit_completion(&mut self) -> Result<(), DeliveryError> {
        if self.completion_committed {
            return Err(DeliveryError::AlreadyCompleted);
        }
        if self.state != DeliveryState::KnownSuccess {
            return Err(DeliveryError::CompletionRequiresKnownSuccess);
        }
        self.completion_committed = true;
        Ok(())
    }

    /// Caller acknowledgement is volatile and is excluded from snapshots.
    pub fn acknowledge_caller(&mut self) {
        self.caller_acknowledged = true;
    }

    pub fn snapshot(&self) -> DeliverySnapshot<B> {
        DeliverySnapshot {
            schema_version: DELIVERY_SCHEMA_VERSION,
            binding: self.binding.clone(),
            replay_profile: self.replay_profile,
            state: self.state,
            attempts: self.attempts.clone(),
            observations: self.observations.clone(),
            completion_committed: self.completion_committed,
            halt_provenance: self.halt_provenance,
        }
    }

    pub fn recover(snapshot: DeliverySnapshot<B>) -> Result<Self, DeliveryError> {
        if snapshot.schema_version != DELIVERY_SCHEMA_VERSION {
            return Err(DeliveryError::InvalidSchemaVersion(snapshot.schema_version));
        }
        if snapshot.attempts.iter().enumerate().any(|(index, attempt)| {
            attempt.id.0 != (index + 1) as u64 || attempt.binding != snapshot.binding
        }) {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "attempts are not contiguous and identity-stable",
            ));
        }
        if snapshot
            .observations
            .windows(2)
            .any(|pair| pair[0].sequence >= pair[1].sequence)
        {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "observation sequence is not strictly increasing",
            ));
        }
        if snapshot
            .observations
            .iter()
            .any(|observation| observation.binding != snapshot.binding)
        {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "observation binding drift",
            ));
        }
        if snapshot.observations.iter().any(|observation| {
            !snapshot
                .attempts
                .iter()
                .any(|attempt| attempt.id == observation.attempt_id)
        }) {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "observation references missing attempt",
            ));
        }
        let halt_provenance = match snapshot.state {
            DeliveryState::IntegrityHalted => {
                let provenance = snapshot
                    .halt_provenance
                    .ok_or(DeliveryError::HaltProvenanceMissing)?;
                match provenance.reason {
                    HaltReason::ContradictorySemanticEvidence => {
                        let sequence = provenance
                            .observation_sequence
                            .ok_or(DeliveryError::HaltProvenanceMismatch)?;
                        let trigger = snapshot
                            .observations
                            .iter()
                            .find(|observation| observation.sequence == sequence)
                            .ok_or(DeliveryError::HaltProvenanceMismatch)?;
                        if !(trigger.kind.is_semantic_success() || trigger.kind.is_no_effect())
                            || !(has_success && has_no_effect)
                        {
                            return Err(DeliveryError::HaltProvenanceMismatch);
                        }
                    }
                    _ if provenance.observation_sequence.is_some() => {
                        return Err(DeliveryError::HaltProvenanceMismatch);
                    }
                    _ => {}
                }
                Some(provenance)
            }
            _ => {
                if snapshot.halt_provenance.is_some() {
                    return Err(DeliveryError::HaltProvenanceUnexpected);
                }
                None
            }
        };

        if snapshot.state == DeliveryState::Pending && !snapshot.attempts.is_empty() {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "pending state cannot have an attempt",
            ));
        }
        if snapshot.state != DeliveryState::Pending
            && snapshot.state != DeliveryState::IntegrityHalted
            && snapshot.attempts.is_empty()
        {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "non-pending state requires an attempt",
            ));
        }
        if snapshot.state == DeliveryState::IntegrityHalted
            && snapshot.attempts.is_empty()
            && !matches!(
                halt_provenance.map(|provenance| provenance.reason),
                Some(HaltReason::BindingDrift)
            )
        {
            return Err(DeliveryError::HaltProvenanceMismatch);
        }

        let latest_attempt = snapshot.attempts.last().map(|attempt| attempt.id);
        let latest_has = |kind: ObservationKind| {
            latest_attempt.is_some_and(|attempt_id| {
                snapshot
                    .observations
                    .iter()
                    .any(|observation| observation.attempt_id == attempt_id && observation.kind == kind)
            })
        };
        let has_success = snapshot
            .observations
            .iter()
            .any(|observation| observation.kind.is_semantic_success());
        let has_no_effect = snapshot
            .observations
            .iter()
            .any(|observation| observation.kind.is_no_effect());

        if has_success && has_no_effect && snapshot.state != DeliveryState::IntegrityHalted {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "conflicting semantic evidence must halt",
            ));
        }

        match snapshot.state {
            DeliveryState::Pending => {}
            DeliveryState::AttemptStarted => {
                if latest_has(ObservationKind::SemanticSuccess)
                    || latest_has(ObservationKind::ReconciliationSuccess)
                    || latest_has(ObservationKind::ReconciliationNoEffect)
                {
                    return Err(DeliveryError::SnapshotInvariantViolation(
                        "attempt-started state has terminal evidence",
                    ));
                }
            }
            DeliveryState::OutcomeUnknown => {
                if !latest_has(ObservationKind::OutcomeUnknown) {
                    return Err(DeliveryError::SnapshotInvariantViolation(
                        "unknown state lacks current-attempt unknown evidence",
                    ));
                }
            }
            DeliveryState::KnownSuccess => {
                if !latest_has(ObservationKind::SemanticSuccess)
                    && !latest_has(ObservationKind::ReconciliationSuccess)
                {
                    return Err(DeliveryError::SnapshotInvariantViolation(
                        "known-success state lacks current-attempt success evidence",
                    ));
                }
            }
            DeliveryState::KnownNoEffect => {
                if !latest_has(ObservationKind::ReconciliationNoEffect) {
                    return Err(DeliveryError::SnapshotInvariantViolation(
                        "known-no-effect state lacks current-attempt reconciliation evidence",
                    ));
                }
            }
            DeliveryState::IntegrityHalted => {}
        }

        if snapshot.completion_committed && !has_success {
            return Err(DeliveryError::SnapshotInvariantViolation(
                "completion lacks success evidence",
            ));
        }

        let next_observation_sequence = snapshot
            .observations
            .last()
            .map_or(1, |observation| observation.sequence.saturating_add(1));

        Ok(Self {
            binding: snapshot.binding,
            replay_profile: snapshot.replay_profile,
            state: snapshot.state,
            attempts: snapshot.attempts,
            observations: snapshot.observations,
            completion_committed: snapshot.completion_committed,
            caller_acknowledged: false,
            next_observation_sequence,
            halt_provenance,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    struct TestBinding {
        instance: u64,
        contract: u64,
        payload: u64,
        sink: u64,
        epoch: u32,
    }

    impl EffectBinding for TestBinding {
        type InstanceRoot = u64;
        type ContractRoot = u64;
        type PayloadCommitment = u64;
        type SinkRoot = u64;
        type SinkEpoch = u32;

        fn effect_instance_root(&self) -> &Self::InstanceRoot { &self.instance }
        fn effect_contract_root(&self) -> &Self::ContractRoot { &self.contract }
        fn payload_commitment(&self) -> &Self::PayloadCommitment { &self.payload }
        fn sink_root(&self) -> &Self::SinkRoot { &self.sink }
        fn sink_epoch(&self) -> &Self::SinkEpoch { &self.epoch }
    }

    fn binding() -> TestBinding {
        TestBinding { instance: 10, contract: 20, payload: 30, sink: 40, epoch: 1 }
    }

    fn drifted() -> TestBinding {
        TestBinding { instance: 10, contract: 20, payload: 31, sink: 40, epoch: 1 }
    }

    fn record(profile: ReplayProfile) -> DeliveryRecord<TestBinding> {
        PreparedDeliveryIntent::new(binding(), profile)
            .into_committed()
            .start_record()
    }

    fn attempt(record: &mut DeliveryRecord<TestBinding>) -> AttemptId {
        record.start_initial_attempt(&binding()).unwrap().attempt_id()
    }

    #[test]
    fn dispatch_permit_binding_drift_halts_and_survives_recovery() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let mut other = PreparedDeliveryIntent::new(drifted(), ReplayProfile::NoAutomaticRetry)
            .into_committed()
            .start_record();
        let forged_permit = other.start_initial_attempt(&drifted()).unwrap();
        record.start_initial_attempt(&binding()).unwrap();

        assert_eq!(
            record.authorize_dispatch(&forged_permit),
            Err(DeliveryError::BindingMismatch)
        );
        assert_eq!(
            record.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::BindingDrift)
        );
        let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
        assert_eq!(recovered.state(), DeliveryState::IntegrityHalted);
    }

    #[test]
    fn dispatch_permit_wrong_attempt_halts_and_survives_recovery() {
        let mut record = record(ReplayProfile::IdempotentByEffectInstance);
        let mut other = record(ReplayProfile::IdempotentByEffectInstance);
        let _first = other.start_initial_attempt(&binding()).unwrap();
        other
            .observe(Observation::outcome_unknown(binding(), AttemptId(1), 8))
            .unwrap();
        let forged_permit = other.retry_same_effect(&binding()).unwrap();

        record.start_initial_attempt(&binding()).unwrap();
        assert_eq!(
            record.authorize_dispatch(&forged_permit),
            Err(DeliveryError::WrongAttempt)
        );
        assert_eq!(
            record.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::AttemptProvenance)
        );
        let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
        assert_eq!(recovered.state(), DeliveryState::IntegrityHalted);
    }

    #[test]
    fn snapshot_schema_change_rejects_previous_version() {
        let record = record(ReplayProfile::NoAutomaticRetry);
        let mut snapshot = record.snapshot();
        snapshot.schema_version = 1;

        assert_eq!(
            DeliveryRecord::recover(snapshot),
            Err(DeliveryError::InvalidSchemaVersion(1))
        );
    }

    #[test]
    fn binding_drift_halt_survives_recovery() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        assert_eq!(
            record.start_initial_attempt(&drifted()),
            Err(DeliveryError::BindingMismatch)
        );
        assert_eq!(
            record.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::BindingDrift)
        );

        let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
        assert_eq!(recovered.state(), DeliveryState::IntegrityHalted);
        assert_eq!(
            recovered.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::BindingDrift)
        );
    }

    #[test]
    fn wrong_attempt_halt_survives_recovery() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        assert_eq!(
            record.observe(Observation::outcome_unknown(binding(), AttemptId(id.raw() + 1), 8)),
            Err(DeliveryError::WrongAttempt)
        );
        assert_eq!(
            record.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::AttemptProvenance)
        );

        let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
        assert_eq!(recovered.state(), DeliveryState::IntegrityHalted);
        assert_eq!(
            recovered.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::AttemptProvenance)
        );
    }

    #[test]
    fn invalid_reconciliation_halt_survives_recovery() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        record.observe(Observation::outcome_unknown(binding(), id, 8)).unwrap();

        assert_eq!(
            record.observe(Observation::reconciliation_no_effect(binding(), id, 9, 999)),
            Err(DeliveryError::ReconciliationTargetMissing(999))
        );
        assert_eq!(
            record.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::InvalidReconciliationProvenance)
        );

        let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
        assert_eq!(recovered.state(), DeliveryState::IntegrityHalted);
        assert_eq!(
            recovered.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::InvalidReconciliationProvenance)
        );
    }

    #[test]
    fn recovered_halt_rejects_dispatch_retry_and_completion() {
        let mut record = record(ReplayProfile::IdempotentByEffectInstance);
        let permit = record.start_initial_attempt(&binding()).unwrap();
        record.start_initial_attempt(&binding()).err();

        assert_eq!(
            record.observe(Observation::outcome_unknown(drifted(), permit.attempt_id(), 8)),
            Err(DeliveryError::BindingMismatch)
        );
        let mut recovered = DeliveryRecord::recover(record.snapshot()).unwrap();

        assert_eq!(
            recovered.authorize_dispatch(&permit),
            Err(DeliveryError::RetryNotPermitted(DeliveryState::IntegrityHalted))
        );
        assert_eq!(
            recovered.retry_same_effect(&binding()),
            Err(DeliveryError::RetryNotPermitted(DeliveryState::IntegrityHalted))
        );
        assert_eq!(
            recovered.commit_completion(),
            Err(DeliveryError::CompletionRequiresKnownSuccess)
        );
    }

    #[test]
    fn missing_halt_provenance_is_rejected() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        record.observe(Observation::outcome_unknown(binding(), id, 8)).unwrap();
        let mut snapshot = record.snapshot();
        snapshot.state = DeliveryState::IntegrityHalted;
        snapshot.halt_provenance = None;

        assert_eq!(
            DeliveryRecord::recover(snapshot),
            Err(DeliveryError::HaltProvenanceMissing)
        );
    }

    #[test]
    fn forged_halt_provenance_is_rejected() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        record.observe(Observation::outcome_unknown(binding(), id, 8)).unwrap();
        let mut snapshot = record.snapshot();
        snapshot.state = DeliveryState::IntegrityHalted;
        snapshot.halt_provenance = Some(HaltProvenance::contradictory(1));

        assert_eq!(
            DeliveryRecord::recover(snapshot),
            Err(DeliveryError::HaltProvenanceMismatch)
        );
    }

    #[test]
    fn contradictory_halt_provenance_survives_recovery() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        let success = record
            .observe(Observation::semantic_success(binding(), id, 10))
            .unwrap()
            .sequence();
        record.commit_completion().unwrap();
        assert_eq!(
            record.observe(Observation::reconciliation_no_effect(binding(), id, 11, success)),
            Ok(ObservationResult::IntegrityHalted { sequence: 2 })
        );

        let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
        assert_eq!(recovered.state(), DeliveryState::IntegrityHalted);
        assert_eq!(
            recovered.halt_provenance().map(|p| p.reason_code()),
            Some(HaltReason::ContradictorySemanticEvidence)
        );
        assert_eq!(
            recovered.halt_provenance().and_then(|p| p.observation_sequence()),
            Some(2)
        );
        assert!(recovered.completion_committed());
    }

    #[test]
    fn unknown_commit_grants_no_delivery_authority() {
        let intent = PreparedDeliveryIntent::new(binding(), ReplayProfile::NoAutomaticRetry);
        assert!(intent.clone().apply_commit_outcome(IntentCommitOutcome::CommitOutcomeUnknown).is_none());
        assert!(intent.apply_commit_outcome(IntentCommitOutcome::DefinitelyNotCommitted).is_none());
    }

    #[test]
    fn only_committed_intent_can_start_attempt() {
        let intent = PreparedDeliveryIntent::new(binding(), ReplayProfile::NoAutomaticRetry);
        assert!(intent.apply_commit_outcome(IntentCommitOutcome::Committed).is_some());
    }

    #[test]
    fn initial_attempt_requires_exact_binding() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        assert_eq!(
            record.start_initial_attempt(&drifted()),
            Err(DeliveryError::BindingMismatch)
        );
        assert_eq!(record.state(), DeliveryState::IntegrityHalted);
    }

    #[test]
    fn dispatch_permit_is_revalidated_after_late_success() {
        let mut record = record(ReplayProfile::IdempotentByEffectInstance);
        let permit = record.start_initial_attempt(&binding()).unwrap();
        assert_eq!(record.authorize_dispatch(&permit), Ok(()));
        record
            .observe(Observation::semantic_success(binding(), permit.attempt_id(), 10))
            .unwrap();
        assert_eq!(
            record.authorize_dispatch(&permit),
            Err(DeliveryError::RetryNotPermitted(DeliveryState::KnownSuccess))
        );
    }

    #[test]
    fn provider_ack_does_not_prove_effect() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        assert_eq!(
            record.observe(Observation::provider_acknowledged(binding(), id, 7)),
            Ok(ObservationResult::Applied {
                state: DeliveryState::AttemptStarted,
                sequence: 1
            })
        );
    }

    #[test]
    fn timeout_becomes_unknown_not_no_effect() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        assert_eq!(
            record.observe(Observation::outcome_unknown(binding(), id, 8)),
            Ok(ObservationResult::Applied {
                state: DeliveryState::OutcomeUnknown,
                sequence: 1
            })
        );
    }

    #[test]
    fn no_automatic_retry_blocks_unknown() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        record.observe(Observation::outcome_unknown(binding(), id, 8)).unwrap();
        assert_eq!(record.retry_same_effect(&binding()), Err(DeliveryError::RetryBlocked));
    }

    #[test]
    fn idempotent_retry_preserves_full_binding() {
        let mut record = record(ReplayProfile::IdempotentByEffectInstance);
        let id = attempt(&mut record);
        record.observe(Observation::outcome_unknown(binding(), id, 8)).unwrap();
        let retry = record.retry_same_effect(&binding()).unwrap();
        assert_eq!(retry.attempt_id().raw(), 2);
        assert_eq!(record.attempts()[0].binding(), record.attempts()[1].binding());
        assert_eq!(record.attempts()[1].binding().effect_instance_root(), &10);
        assert_eq!(record.attempts()[1].binding().effect_contract_root(), &20);
        assert_eq!(record.attempts()[1].binding().payload_commitment(), &30);
        assert_eq!(record.attempts()[1].binding().sink_root(), &40);
        assert_eq!(record.attempts()[1].binding().sink_epoch(), &1);
    }

    #[test]
    fn retry_binding_drift_halts() {
        let mut record = record(ReplayProfile::IdempotentByEffectInstance);
        let id = attempt(&mut record);
        record.observe(Observation::outcome_unknown(binding(), id, 8)).unwrap();
        assert_eq!(record.retry_same_effect(&drifted()), Err(DeliveryError::BindingMismatch));
        assert_eq!(record.state(), DeliveryState::IntegrityHalted);
    }

    #[test]
    fn reconciliation_targets_same_unknown() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        let unknown = record
            .observe(Observation::outcome_unknown(binding(), id, 8))
            .unwrap()
            .sequence();
        assert_eq!(
            record.observe(Observation::reconciliation_no_effect(binding(), id, 9, unknown)),
            Ok(ObservationResult::Applied {
                state: DeliveryState::KnownNoEffect,
                sequence: 2
            })
        );
    }

    #[test]
    fn reconciliation_cannot_change_identity() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        let unknown = record
            .observe(Observation::outcome_unknown(binding(), id, 8))
            .unwrap()
            .sequence();
        assert_eq!(
            record.observe(Observation::reconciliation_success(drifted(), id, 9, unknown)),
            Err(DeliveryError::BindingMismatch)
        );
        assert_eq!(record.state(), DeliveryState::IntegrityHalted);
    }

    #[test]
    fn no_effect_retry_requires_idempotent_profile() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        let unknown = record
            .observe(Observation::outcome_unknown(binding(), id, 8))
            .unwrap()
            .sequence();
        record
            .observe(Observation::reconciliation_no_effect(binding(), id, 9, unknown))
            .unwrap();
        assert_eq!(
            record.retry_after_no_effect(&binding()),
            Err(DeliveryError::RetryNotPermitted(DeliveryState::KnownNoEffect))
        );
    }

    #[test]
    fn contradiction_halts_without_erasing_completion() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        let success = record
            .observe(Observation::semantic_success(binding(), id, 10))
            .unwrap()
            .sequence();
        record.commit_completion().unwrap();
        assert_eq!(
            record.observe(Observation::reconciliation_no_effect(binding(), id, 11, success)),
            Ok(ObservationResult::IntegrityHalted { sequence: 2 })
        );
        assert_eq!(record.state(), DeliveryState::IntegrityHalted);
        assert!(record.completion_committed());
        assert_eq!(record.observations().len(), 2);
    }

    #[test]
    fn success_cannot_be_retried() {
        let mut record = record(ReplayProfile::IdempotentByEffectInstance);
        let id = attempt(&mut record);
        record.observe(Observation::semantic_success(binding(), id, 10)).unwrap();
        assert_eq!(record.retry_same_effect(&binding()), Err(DeliveryError::RetryAfterSuccess));
    }

    #[test]
    fn caller_ack_is_not_persisted_authority() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        attempt(&mut record);
        let before = record.snapshot();
        record.acknowledge_caller();
        assert_eq!(record.snapshot(), before);
    }

    #[test]
    fn duplicate_observation_is_idempotent() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        let observation = Observation::outcome_unknown(binding(), id, 8);
        assert_eq!(
            record.observe(observation.clone()),
            Ok(ObservationResult::Applied {
                state: DeliveryState::OutcomeUnknown,
                sequence: 1
            })
        );
        assert_eq!(
            record.observe(observation),
            Ok(ObservationResult::Duplicate { sequence: 1 })
        );
        assert_eq!(record.observations().len(), 1);
    }

    #[test]
    fn snapshot_recovery_preserves_unknown_and_retry_state() {
        let mut record = record(ReplayProfile::IdempotentByEffectInstance);
        let id = attempt(&mut record);
        let unknown = record
            .observe(Observation::outcome_unknown(binding(), id, 8))
            .unwrap()
            .sequence();
        record
            .observe(Observation::reconciliation_no_effect(binding(), id, 9, unknown))
            .unwrap();
        record.retry_after_no_effect(&binding()).unwrap();
        let recovered = DeliveryRecord::recover(record.snapshot()).unwrap();
        assert_eq!(recovered.state(), DeliveryState::AttemptStarted);
        assert_eq!(recovered.attempts().len(), 2);
    }

    #[test]
    fn snapshot_rejects_conflict_without_halt() {
        let mut record = record(ReplayProfile::NoAutomaticRetry);
        let id = attempt(&mut record);
        let success = record
            .observe(Observation::semantic_success(binding(), id, 10))
            .unwrap()
            .sequence();
        record
            .observe(Observation::reconciliation_no_effect(binding(), id, 11, success))
            .unwrap();
        let mut snapshot = record.snapshot();
        snapshot.state = DeliveryState::KnownSuccess;
        assert_eq!(
            DeliveryRecord::recover(snapshot),
            Err(DeliveryError::SnapshotInvariantViolation(
                "conflicting semantic evidence must halt"
            ))
        );
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Append-only event models for workflows involving independent actors.
//!
//! Core invariants:
//! - an actor never mutates another actor's record to express a decision;
//! - event arrival order is not workflow truth: projections derive a deterministic
//!   causal order from event IDs, timestamps, and causal-parent references;
//! - recovery policy identity is digest-bound and recovery execution is explicitly
//!   authorized;
//! - workflow transitions bind the exact request / recovery target they authorize.
//!
//! This crate is intentionally Holochain-neutral. A persistence adapter should
//! bind `EventEnvelope.author` to the committing `AgentPubKey`, reject entry
//! update/delete, verify `proof_ref`, and only expose events that passed integrity
//! validation.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-multiparty-workflows-v0.1";
const MAX_ID_BYTES: usize = 512;

macro_rules! id_type {
    ($name:ident, $field:literal) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, WorkflowError> {
                let value = value.into();
                validate_text(&value, $field, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(ActorId, "actor_id");
id_type!(ProcessId, "process_id");
id_type!(EventId, "event_id");
id_type!(EvidenceRef, "evidence_ref");
id_type!(CredentialRef, "credential_ref");
id_type!(PolicyRef, "policy_ref");
id_type!(KeyRef, "key_ref");

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Digest32(pub [u8; 32]);

impl Digest32 {
    pub fn is_zero(&self) -> bool {
        self.0.iter().all(|b| *b == 0)
    }
}

/// Immutable event wrapper. Hosts should cryptographically bind `author` to the
/// committer and verify the proof reference before this event enters a projection.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EventEnvelope<E> {
    pub protocol_version: String,
    pub id: EventId,
    pub process_id: ProcessId,
    pub author: ActorId,
    pub occurred_at_ms: u64,
    /// Causal dependencies. Every non-root event must name at least one parent.
    /// Parents may be concurrent branches; projection canonicalizes the DAG.
    pub causal_parents: Vec<EventId>,
    pub payload: E,
    /// Signature / attestation reference verified by the integrating host.
    pub proof_ref: String,
}

impl<E> EventEnvelope<E> {
    pub fn validate_envelope(&self) -> Result<(), WorkflowError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(WorkflowError::WrongProtocolVersion);
        }
        validate_text(self.id.as_str(), "event.id", MAX_ID_BYTES)?;
        validate_text(self.process_id.as_str(), "event.process_id", MAX_ID_BYTES)?;
        validate_text(self.author.as_str(), "event.author", MAX_ID_BYTES)?;
        validate_text(&self.proof_ref, "event.proof_ref", MAX_ID_BYTES)?;
        if self.occurred_at_ms == 0 {
            return Err(WorkflowError::ZeroTimestamp);
        }
        if self.causal_parents.iter().any(|parent| parent == &self.id) {
            return Err(WorkflowError::SelfCausalReference);
        }
        let unique: BTreeSet<&EventId> = self.causal_parents.iter().collect();
        if unique.len() != self.causal_parents.len() {
            return Err(WorkflowError::DuplicateCausalParent);
        }
        Ok(())
    }
}

/// Deterministically orders one workflow event set from its causal graph.
///
/// The root is always first. Each non-root event must reference at least one
/// existing parent in the same process. Among concurrently-ready events, the
/// stable tie-break is `(occurred_at_ms, event_id)`, making projection independent
/// of DHT/network arrival order.
fn canonicalize_event_set<E: Clone>(
    events: &[EventEnvelope<E>],
    root_id: &EventId,
    process_id: &ProcessId,
) -> Result<Vec<EventEnvelope<E>>, WorkflowError> {
    let mut by_id: BTreeMap<EventId, EventEnvelope<E>> = BTreeMap::new();

    for event in events {
        event.validate_envelope()?;
        if &event.process_id != process_id {
            return Err(WorkflowError::ProcessMismatch);
        }
        if by_id.insert(event.id.clone(), event.clone()).is_some() {
            return Err(WorkflowError::DuplicateEvent);
        }
    }

    let root = by_id.get(root_id).ok_or(WorkflowError::MissingInitialEvent)?;
    if !root.causal_parents.is_empty() {
        return Err(WorkflowError::InitialEventHasParents);
    }

    for event in by_id.values() {
        if &event.id == root_id {
            continue;
        }
        if event.causal_parents.is_empty() {
            return Err(WorkflowError::MissingCausalParent);
        }
        for parent in &event.causal_parents {
            let parent_event = by_id
                .get(parent)
                .ok_or(WorkflowError::UnknownCausalParent)?;
            if parent_event.process_id != event.process_id {
                return Err(WorkflowError::ProcessMismatch);
            }
            if event.occurred_at_ms < parent_event.occurred_at_ms {
                return Err(WorkflowError::CausalTimeRegression);
            }
        }
    }

    let mut applied = BTreeSet::new();
    let mut ordered = Vec::with_capacity(events.len());

    ordered.push(root.clone());
    applied.insert(root_id.clone());

    while ordered.len() < events.len() {
        let next = by_id
            .values()
            .filter(|event| !applied.contains(&event.id))
            .filter(|event| event.causal_parents.iter().all(|p| applied.contains(p)))
            .min_by(|a, b| {
                (a.occurred_at_ms, a.id.as_str()).cmp(&(b.occurred_at_ms, b.id.as_str()))
            })
            .cloned();

        let Some(next) = next else {
            return Err(WorkflowError::CausalCycleOrDisconnectedGraph);
        };
        applied.insert(next.id.clone());
        ordered.push(next);
    }

    Ok(ordered)
}

// =============================================================================
// Credential request workflow
// =============================================================================

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialRequestSpec {
    pub requester: ActorId,
    pub subject: ActorId,
    pub issuer: ActorId,
    pub schema_id: String,
    pub policy_ref: PolicyRef,
    /// Digest of the exact claims / request payload being evaluated.
    pub request_digest: Digest32,
    /// Hard lifetime for this request. Issuer decisions and issuance after this
    /// instant are rejected by the pure projection.
    pub expires_at_ms: u64,
}

impl CredentialRequestSpec {
    pub fn validate(&self, requested_at_ms: u64) -> Result<(), WorkflowError> {
        validate_text(self.requester.as_str(), "credential.requester", MAX_ID_BYTES)?;
        validate_text(self.subject.as_str(), "credential.subject", MAX_ID_BYTES)?;
        validate_text(self.issuer.as_str(), "credential.issuer", MAX_ID_BYTES)?;
        validate_text(&self.schema_id, "credential.schema_id", MAX_ID_BYTES)?;
        validate_text(
            self.policy_ref.as_str(),
            "credential.policy_ref",
            MAX_ID_BYTES,
        )?;
        require_digest(self.request_digest, "credential.request_digest")?;
        if self.expires_at_ms <= requested_at_ms {
            return Err(WorkflowError::InvalidCredentialLifetime);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CredentialEvent {
    Requested {
        spec: CredentialRequestSpec,
    },
    ReviewStarted {
        issuer: ActorId,
    },
    EvidenceSubmitted {
        submitter: ActorId,
        evidence_ref: EvidenceRef,
    },
    Approved {
        issuer: ActorId,
        /// Must equal the original request digest.
        request_digest: Digest32,
        decision_ref: String,
    },
    Rejected {
        issuer: ActorId,
        reason_code: String,
        decision_ref: String,
    },
    CredentialIssued {
        issuer: ActorId,
        credential_ref: CredentialRef,
        credential_digest: Digest32,
    },
    Withdrawn {
        requester: ActorId,
        reason_code: String,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CredentialStatus {
    Pending,
    UnderReview,
    Approved,
    Rejected,
    Issued,
    Withdrawn,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialProjection {
    pub process_id: ProcessId,
    pub request_event_id: EventId,
    pub spec: CredentialRequestSpec,
    pub status: CredentialStatus,
    pub evidence: Vec<EvidenceRef>,
    pub decision_ref: Option<String>,
    pub approval_event_id: Option<EventId>,
    pub credential_ref: Option<CredentialRef>,
    pub credential_digest: Option<Digest32>,
    pub applied_events: Vec<EventId>,
}

pub fn project_credential_events(
    events: &[EventEnvelope<CredentialEvent>],
) -> Result<CredentialProjection, WorkflowError> {
    let request_events: Vec<&EventEnvelope<CredentialEvent>> = events
        .iter()
        .filter(|event| matches!(&event.payload, CredentialEvent::Requested { .. }))
        .collect();

    let first = match request_events.as_slice() {
        [] => return Err(WorkflowError::MissingInitialEvent),
        [only] => *only,
        _ => return Err(WorkflowError::DuplicateInitialEvent),
    };
    first.validate_envelope()?;

    let spec = match &first.payload {
        CredentialEvent::Requested { spec } => {
            spec.validate(first.occurred_at_ms)?;
            if first.author != spec.requester {
                return Err(WorkflowError::AuthorMismatch("credential.requester"));
            }
            spec.clone()
        }
        _ => unreachable!(),
    };

    let ordered = canonicalize_event_set(events, &first.id, &first.process_id)?;
    let mut projection = CredentialProjection {
        process_id: first.process_id.clone(),
        request_event_id: first.id.clone(),
        spec,
        status: CredentialStatus::Pending,
        evidence: Vec::new(),
        decision_ref: None,
        approval_event_id: None,
        credential_ref: None,
        credential_digest: None,
        applied_events: vec![first.id.clone()],
    };

    for event in &ordered[1..] {
        if event.occurred_at_ms > projection.spec.expires_at_ms {
            return Err(WorkflowError::CredentialRequestExpired);
        }
        apply_credential_event(&mut projection, event)?;
        projection.applied_events.push(event.id.clone());
    }
    Ok(projection)
}

fn apply_credential_event(
    state: &mut CredentialProjection,
    event: &EventEnvelope<CredentialEvent>,
) -> Result<(), WorkflowError> {
    match &event.payload {
        CredentialEvent::Requested { .. } => Err(WorkflowError::DuplicateInitialEvent),
        CredentialEvent::ReviewStarted { issuer } => {
            require_actor(&event.author, issuer, "credential.review.issuer")?;
            require_actor(issuer, &state.spec.issuer, "credential.expected_issuer")?;
            require_status(
                state.status,
                &[CredentialStatus::Pending],
                "review_started",
            )?;
            state.status = CredentialStatus::UnderReview;
            Ok(())
        }
        CredentialEvent::EvidenceSubmitted {
            submitter,
            evidence_ref,
        } => {
            require_actor(&event.author, submitter, "credential.evidence.submitter")?;
            if submitter != &state.spec.requester
                && submitter != &state.spec.subject
                && submitter != &state.spec.issuer
            {
                return Err(WorkflowError::UnauthorizedActor(
                    "credential evidence submitter",
                ));
            }
            if matches!(
                state.status,
                CredentialStatus::Rejected
                    | CredentialStatus::Issued
                    | CredentialStatus::Withdrawn
            ) {
                return Err(WorkflowError::TerminalState("credential"));
            }
            if !state.evidence.contains(evidence_ref) {
                state.evidence.push(evidence_ref.clone());
            }
            Ok(())
        }
        CredentialEvent::Approved {
            issuer,
            request_digest,
            decision_ref,
        } => {
            require_actor(&event.author, issuer, "credential.approval.issuer")?;
            require_actor(issuer, &state.spec.issuer, "credential.expected_issuer")?;
            require_status(
                state.status,
                &[CredentialStatus::Pending, CredentialStatus::UnderReview],
                "approved",
            )?;
            if request_digest != &state.spec.request_digest {
                return Err(WorkflowError::CredentialRequestDigestMismatch);
            }
            validate_text(decision_ref, "credential.decision_ref", MAX_ID_BYTES)?;
            state.status = CredentialStatus::Approved;
            state.decision_ref = Some(decision_ref.clone());
            state.approval_event_id = Some(event.id.clone());
            Ok(())
        }
        CredentialEvent::Rejected {
            issuer,
            reason_code,
            decision_ref,
        } => {
            require_actor(&event.author, issuer, "credential.rejection.issuer")?;
            require_actor(issuer, &state.spec.issuer, "credential.expected_issuer")?;
            require_status(
                state.status,
                &[CredentialStatus::Pending, CredentialStatus::UnderReview],
                "rejected",
            )?;
            validate_text(reason_code, "credential.rejection.reason_code", 256)?;
            validate_text(decision_ref, "credential.decision_ref", MAX_ID_BYTES)?;
            state.status = CredentialStatus::Rejected;
            state.decision_ref = Some(decision_ref.clone());
            Ok(())
        }
        CredentialEvent::CredentialIssued {
            issuer,
            credential_ref,
            credential_digest,
        } => {
            require_actor(&event.author, issuer, "credential.issuance.issuer")?;
            require_actor(issuer, &state.spec.issuer, "credential.expected_issuer")?;
            require_status(
                state.status,
                &[CredentialStatus::Approved],
                "credential_issued",
            )?;
            require_digest(*credential_digest, "credential.credential_digest")?;
            let approval_event = state
                .approval_event_id
                .as_ref()
                .ok_or(WorkflowError::CredentialNotApproved)?;
            if !event.causal_parents.contains(approval_event) {
                return Err(WorkflowError::IssuanceMissingApprovalCause);
            }
            state.status = CredentialStatus::Issued;
            state.credential_ref = Some(credential_ref.clone());
            state.credential_digest = Some(*credential_digest);
            Ok(())
        }
        CredentialEvent::Withdrawn {
            requester,
            reason_code,
        } => {
            require_actor(&event.author, requester, "credential.withdrawal.requester")?;
            require_actor(
                requester,
                &state.spec.requester,
                "credential.expected_requester",
            )?;
            if matches!(state.status, CredentialStatus::Issued | CredentialStatus::Rejected) {
                return Err(WorkflowError::TerminalState("credential"));
            }
            validate_text(reason_code, "credential.withdrawal.reason_code", 256)?;
            state.status = CredentialStatus::Withdrawn;
            Ok(())
        }
    }
}

// =============================================================================
// Social recovery workflow
// =============================================================================

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecoveryPolicy {
    pub subject: ActorId,
    pub trustees: Vec<ActorId>,
    pub threshold: u32,
    pub min_timelock_ms: u64,
    /// Explicit actors permitted to author the final execution event. In a
    /// Holochain adapter this would normally be a narrowly-scoped recovery
    /// service / identity authority, not an arbitrary client.
    pub executors: Vec<ActorId>,
    pub policy_ref: PolicyRef,
    /// Digest of the exact canonical recovery policy document.
    pub policy_digest: Digest32,
}

impl RecoveryPolicy {
    pub fn validate(&self) -> Result<(), WorkflowError> {
        validate_text(self.subject.as_str(), "recovery.subject", MAX_ID_BYTES)?;
        validate_text(self.policy_ref.as_str(), "recovery.policy_ref", MAX_ID_BYTES)?;
        require_digest(self.policy_digest, "recovery.policy_digest")?;

        if self.trustees.len() < 3 {
            return Err(WorkflowError::TooFewTrustees);
        }
        let unique_trustees: BTreeSet<&ActorId> = self.trustees.iter().collect();
        if unique_trustees.len() != self.trustees.len() {
            return Err(WorkflowError::DuplicateTrustee);
        }
        if self.trustees.iter().any(|trustee| trustee == &self.subject) {
            return Err(WorkflowError::SubjectCannotBeTrustee);
        }
        if self.threshold == 0 || self.threshold as usize > self.trustees.len() {
            return Err(WorkflowError::InvalidThreshold);
        }
        let majority = (self.trustees.len() as u32 / 2) + 1;
        if self.threshold < majority {
            return Err(WorkflowError::ThresholdBelowMajority);
        }
        if self.min_timelock_ms == 0 {
            return Err(WorkflowError::ZeroRecoveryTimelock);
        }

        if self.executors.is_empty() {
            return Err(WorkflowError::NoRecoveryExecutor);
        }
        let unique_executors: BTreeSet<&ActorId> = self.executors.iter().collect();
        if unique_executors.len() != self.executors.len() {
            return Err(WorkflowError::DuplicateRecoveryExecutor);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecoveryEvent {
    Initiated {
        subject: ActorId,
        initiator: ActorId,
        new_key_ref: KeyRef,
        policy_ref: PolicyRef,
        policy_digest: Digest32,
    },
    TrusteeApproved {
        trustee: ActorId,
        evidence_ref: Option<EvidenceRef>,
    },
    TrusteeRejected {
        trustee: ActorId,
        reason_code: String,
    },
    SubjectCancelled {
        subject: ActorId,
        reason_code: String,
    },
    Executed {
        executor: ActorId,
        new_key_ref: KeyRef,
        execution_proof_ref: String,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecoveryStatus {
    Pending,
    Timelocked,
    Ready,
    Rejected,
    Cancelled,
    Completed,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecoveryProjection {
    pub process_id: ProcessId,
    pub initiation_event_id: EventId,
    pub policy: RecoveryPolicy,
    pub initiator: ActorId,
    pub new_key_ref: KeyRef,
    pub status: RecoveryStatus,
    pub approvals: BTreeMap<ActorId, u64>,
    pub approval_event_ids: BTreeMap<ActorId, EventId>,
    pub rejections: BTreeMap<ActorId, u64>,
    pub threshold_satisfied_at_ms: Option<u64>,
    pub executable_at_ms: Option<u64>,
    pub applied_events: Vec<EventId>,
}

impl RecoveryProjection {
    pub fn effective_status_at(&self, now_ms: u64) -> RecoveryStatus {
        if self.status == RecoveryStatus::Timelocked
            && self.executable_at_ms.is_some_and(|at| now_ms >= at)
        {
            RecoveryStatus::Ready
        } else {
            self.status
        }
    }

    pub fn approvals_count(&self) -> usize {
        self.approvals.len()
    }
}

pub fn project_recovery_events(
    policy: &RecoveryPolicy,
    events: &[EventEnvelope<RecoveryEvent>],
) -> Result<RecoveryProjection, WorkflowError> {
    policy.validate()?;

    let initiation_events: Vec<&EventEnvelope<RecoveryEvent>> = events
        .iter()
        .filter(|event| matches!(&event.payload, RecoveryEvent::Initiated { .. }))
        .collect();

    let first = match initiation_events.as_slice() {
        [] => return Err(WorkflowError::MissingInitialEvent),
        [only] => *only,
        _ => return Err(WorkflowError::DuplicateInitialEvent),
    };
    first.validate_envelope()?;

    let (subject, initiator, new_key_ref, policy_ref, policy_digest) = match &first.payload {
        RecoveryEvent::Initiated {
            subject,
            initiator,
            new_key_ref,
            policy_ref,
            policy_digest,
        } => (
            subject,
            initiator,
            new_key_ref,
            policy_ref,
            policy_digest,
        ),
        _ => unreachable!(),
    };

    require_actor(&first.author, initiator, "recovery.initiator")?;
    require_actor(subject, &policy.subject, "recovery.subject")?;
    if policy_ref != &policy.policy_ref || policy_digest != &policy.policy_digest {
        return Err(WorkflowError::PolicyMismatch);
    }
    if initiator != subject && !policy.trustees.contains(initiator) {
        return Err(WorkflowError::UnauthorizedActor("recovery initiator"));
    }

    let ordered = canonicalize_event_set(events, &first.id, &first.process_id)?;
    let mut projection = RecoveryProjection {
        process_id: first.process_id.clone(),
        initiation_event_id: first.id.clone(),
        policy: policy.clone(),
        initiator: initiator.clone(),
        new_key_ref: new_key_ref.clone(),
        status: RecoveryStatus::Pending,
        approvals: BTreeMap::new(),
        approval_event_ids: BTreeMap::new(),
        rejections: BTreeMap::new(),
        threshold_satisfied_at_ms: None,
        executable_at_ms: None,
        applied_events: vec![first.id.clone()],
    };

    for event in &ordered[1..] {
        apply_recovery_event(&mut projection, event)?;
        projection.applied_events.push(event.id.clone());
    }
    Ok(projection)
}

fn apply_recovery_event(
    state: &mut RecoveryProjection,
    event: &EventEnvelope<RecoveryEvent>,
) -> Result<(), WorkflowError> {
    if matches!(
        state.status,
        RecoveryStatus::Rejected | RecoveryStatus::Cancelled | RecoveryStatus::Completed
    ) {
        return Err(WorkflowError::TerminalState("recovery"));
    }

    match &event.payload {
        RecoveryEvent::Initiated { .. } => Err(WorkflowError::DuplicateInitialEvent),
        RecoveryEvent::TrusteeApproved {
            trustee,
            evidence_ref: _,
        } => {
            require_actor(&event.author, trustee, "recovery.approval.trustee")?;
            if !state.policy.trustees.contains(trustee) {
                return Err(WorkflowError::UnauthorizedActor("recovery trustee"));
            }
            if state.approvals.contains_key(trustee) || state.rejections.contains_key(trustee) {
                return Err(WorkflowError::TrusteeAlreadyVoted);
            }
            state
                .approvals
                .insert(trustee.clone(), event.occurred_at_ms);
            state
                .approval_event_ids
                .insert(trustee.clone(), event.id.clone());
            recompute_recovery_threshold(state)?;
            Ok(())
        }
        RecoveryEvent::TrusteeRejected {
            trustee,
            reason_code,
        } => {
            require_actor(&event.author, trustee, "recovery.rejection.trustee")?;
            if !state.policy.trustees.contains(trustee) {
                return Err(WorkflowError::UnauthorizedActor("recovery trustee"));
            }
            if state.approvals.contains_key(trustee) || state.rejections.contains_key(trustee) {
                return Err(WorkflowError::TrusteeAlreadyVoted);
            }
            validate_text(reason_code, "recovery.rejection.reason_code", 256)?;
            state
                .rejections
                .insert(trustee.clone(), event.occurred_at_ms);

            let remaining =
                state.policy.trustees.len() - state.approvals.len() - state.rejections.len();
            if state.approvals.len() + remaining < state.policy.threshold as usize {
                state.status = RecoveryStatus::Rejected;
            }
            Ok(())
        }
        RecoveryEvent::SubjectCancelled {
            subject,
            reason_code,
        } => {
            require_actor(&event.author, subject, "recovery.cancel.subject")?;
            require_actor(subject, &state.policy.subject, "recovery.expected_subject")?;
            validate_text(reason_code, "recovery.cancel.reason_code", 256)?;
            state.status = RecoveryStatus::Cancelled;
            Ok(())
        }
        RecoveryEvent::Executed {
            executor,
            new_key_ref,
            execution_proof_ref,
        } => {
            require_actor(&event.author, executor, "recovery.executor")?;
            if !state.policy.executors.contains(executor) {
                return Err(WorkflowError::UnauthorizedActor("recovery executor"));
            }
            validate_text(
                execution_proof_ref,
                "recovery.execution_proof_ref",
                MAX_ID_BYTES,
            )?;
            if new_key_ref != &state.new_key_ref {
                return Err(WorkflowError::RecoveryTargetChanged);
            }
            let executable_at = state
                .executable_at_ms
                .ok_or(WorkflowError::RecoveryThresholdNotSatisfied)?;
            if event.occurred_at_ms < executable_at {
                return Err(WorkflowError::RecoveryTimelockActive);
            }

            let causal_approval_count = state
                .approval_event_ids
                .values()
                .filter(|approval_id| event.causal_parents.contains(approval_id))
                .count();
            if causal_approval_count < state.policy.threshold as usize {
                return Err(WorkflowError::ExecutionMissingThresholdCauses);
            }

            state.status = RecoveryStatus::Completed;
            Ok(())
        }
    }
}

fn recompute_recovery_threshold(state: &mut RecoveryProjection) -> Result<(), WorkflowError> {
    if state.approvals.len() < state.policy.threshold as usize
        || state.threshold_satisfied_at_ms.is_some()
    {
        return Ok(());
    }

    let satisfied_at = state
        .approvals
        .values()
        .copied()
        .max()
        .ok_or(WorkflowError::RecoveryThresholdNotSatisfied)?;
    let executable_at = satisfied_at
        .checked_add(state.policy.min_timelock_ms)
        .ok_or(WorkflowError::TimeOverflow)?;

    state.threshold_satisfied_at_ms = Some(satisfied_at);
    state.executable_at_ms = Some(executable_at);
    state.status = RecoveryStatus::Timelocked;
    Ok(())
}

// =============================================================================
// Shared validation
// =============================================================================

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum WorkflowError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    ZeroDigest(&'static str),
    ZeroTimestamp,
    MissingInitialEvent,
    DuplicateInitialEvent,
    InitialEventHasParents,
    DuplicateEvent,
    ProcessMismatch,
    AuthorMismatch(&'static str),
    UnauthorizedActor(&'static str),
    InvalidTransition {
        workflow: &'static str,
        transition: &'static str,
    },
    TerminalState(&'static str),
    SelfCausalReference,
    DuplicateCausalParent,
    MissingCausalParent,
    UnknownCausalParent,
    CausalTimeRegression,
    CausalCycleOrDisconnectedGraph,
    InvalidCredentialLifetime,
    CredentialRequestExpired,
    CredentialRequestDigestMismatch,
    CredentialNotApproved,
    IssuanceMissingApprovalCause,
    TooFewTrustees,
    DuplicateTrustee,
    SubjectCannotBeTrustee,
    InvalidThreshold,
    ThresholdBelowMajority,
    ZeroRecoveryTimelock,
    NoRecoveryExecutor,
    DuplicateRecoveryExecutor,
    TrusteeAlreadyVoted,
    PolicyMismatch,
    RecoveryTargetChanged,
    RecoveryThresholdNotSatisfied,
    RecoveryTimelockActive,
    ExecutionMissingThresholdCauses,
    TimeOverflow,
}

impl fmt::Display for WorkflowError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::ZeroTimestamp => write!(f, "event timestamp must be non-zero"),
            Self::MissingInitialEvent => write!(f, "workflow has no initial event"),
            Self::DuplicateInitialEvent => write!(f, "workflow has multiple initial events"),
            Self::InitialEventHasParents => write!(f, "initial event must not have causal parents"),
            Self::DuplicateEvent => write!(f, "duplicate event id"),
            Self::ProcessMismatch => write!(f, "event belongs to another process"),
            Self::AuthorMismatch(field) => write!(f, "event author does not match {field}"),
            Self::UnauthorizedActor(role) => write!(f, "actor is not authorized as {role}"),
            Self::InvalidTransition {
                workflow,
                transition,
            } => write!(f, "invalid {workflow} transition: {transition}"),
            Self::TerminalState(workflow) => write!(f, "{workflow} workflow is terminal"),
            Self::SelfCausalReference => write!(f, "event cannot name itself as a causal parent"),
            Self::DuplicateCausalParent => write!(f, "duplicate causal parent"),
            Self::MissingCausalParent => write!(f, "non-root event must name a causal parent"),
            Self::UnknownCausalParent => write!(f, "event references an unknown causal parent"),
            Self::CausalTimeRegression => write!(f, "event timestamp precedes one of its parents"),
            Self::CausalCycleOrDisconnectedGraph => {
                write!(f, "event graph contains a cycle or disconnected branch")
            }
            Self::InvalidCredentialLifetime => write!(f, "credential request lifetime is invalid"),
            Self::CredentialRequestExpired => write!(f, "credential request expired"),
            Self::CredentialRequestDigestMismatch => {
                write!(f, "credential decision does not bind the original request digest")
            }
            Self::CredentialNotApproved => write!(f, "credential was not approved"),
            Self::IssuanceMissingApprovalCause => {
                write!(f, "credential issuance does not causally depend on approval")
            }
            Self::TooFewTrustees => write!(f, "recovery requires at least three trustees"),
            Self::DuplicateTrustee => write!(f, "recovery trustees must be unique"),
            Self::SubjectCannotBeTrustee => write!(f, "recovery subject cannot also be a trustee"),
            Self::InvalidThreshold => write!(f, "invalid recovery threshold"),
            Self::ThresholdBelowMajority => {
                write!(f, "recovery threshold must be at least a majority")
            }
            Self::ZeroRecoveryTimelock => write!(f, "recovery timelock must be positive"),
            Self::NoRecoveryExecutor => write!(f, "recovery policy must name an executor"),
            Self::DuplicateRecoveryExecutor => write!(f, "recovery executors must be unique"),
            Self::TrusteeAlreadyVoted => write!(f, "trustee has already approved or rejected"),
            Self::PolicyMismatch => write!(f, "recovery event references a different policy"),
            Self::RecoveryTargetChanged => {
                write!(f, "recovery target key changed after initiation")
            }
            Self::RecoveryThresholdNotSatisfied => {
                write!(f, "recovery approval threshold not satisfied")
            }
            Self::RecoveryTimelockActive => write!(f, "recovery timelock is still active"),
            Self::ExecutionMissingThresholdCauses => {
                write!(f, "recovery execution is not causally bound to threshold approvals")
            }
            Self::TimeOverflow => write!(f, "workflow timestamp overflow"),
        }
    }
}

impl std::error::Error for WorkflowError {}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), WorkflowError> {
    if value.trim().is_empty() {
        return Err(WorkflowError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(WorkflowError::TooLong(field));
    }
    Ok(())
}

fn require_digest(digest: Digest32, field: &'static str) -> Result<(), WorkflowError> {
    if digest.is_zero() {
        Err(WorkflowError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

fn require_actor(
    actual: &ActorId,
    expected: &ActorId,
    field: &'static str,
) -> Result<(), WorkflowError> {
    if actual == expected {
        Ok(())
    } else {
        Err(WorkflowError::AuthorMismatch(field))
    }
}

fn require_status<T: PartialEq>(
    current: T,
    allowed: &[T],
    transition: &'static str,
) -> Result<(), WorkflowError> {
    if allowed.contains(&current) {
        Ok(())
    } else {
        Err(WorkflowError::InvalidTransition {
            workflow: "credential",
            transition,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn actor(name: &str) -> ActorId {
        ActorId::new(format!("did:example:{name}")).unwrap()
    }

    fn event_id(name: &str) -> EventId {
        EventId::new(name).unwrap()
    }

    fn process(name: &str) -> ProcessId {
        ProcessId::new(name).unwrap()
    }

    fn env<E>(
        id: &str,
        process_id: &ProcessId,
        author: ActorId,
        at: u64,
        parents: &[&str],
        payload: E,
    ) -> EventEnvelope<E> {
        EventEnvelope {
            protocol_version: PROTOCOL_VERSION.into(),
            id: event_id(id),
            process_id: process_id.clone(),
            author,
            occurred_at_ms: at,
            causal_parents: parents.iter().map(|id| event_id(id)).collect(),
            payload,
            proof_ref: format!("proof:{id}"),
        }
    }

    fn credential_spec() -> CredentialRequestSpec {
        CredentialRequestSpec {
            requester: actor("alice"),
            subject: actor("alice"),
            issuer: actor("university"),
            schema_id: "schema:degree:v1".into(),
            policy_ref: PolicyRef::new("policy:degree:v1").unwrap(),
            request_digest: Digest32([7; 32]),
            expires_at_ms: 10_000,
        }
    }

    fn credential_events() -> Vec<EventEnvelope<CredentialEvent>> {
        let pid = process("credential-1");
        let spec = credential_spec();
        vec![
            env(
                "e1",
                &pid,
                spec.requester.clone(),
                100,
                &[],
                CredentialEvent::Requested { spec: spec.clone() },
            ),
            env(
                "e2",
                &pid,
                spec.issuer.clone(),
                200,
                &["e1"],
                CredentialEvent::ReviewStarted {
                    issuer: spec.issuer.clone(),
                },
            ),
            env(
                "e3",
                &pid,
                spec.issuer.clone(),
                300,
                &["e2"],
                CredentialEvent::Approved {
                    issuer: spec.issuer.clone(),
                    request_digest: spec.request_digest,
                    decision_ref: "decision:degree-approved".into(),
                },
            ),
            env(
                "e4",
                &pid,
                spec.issuer.clone(),
                400,
                &["e3"],
                CredentialEvent::CredentialIssued {
                    issuer: spec.issuer.clone(),
                    credential_ref: CredentialRef::new("vc:degree:alice").unwrap(),
                    credential_digest: Digest32([9; 32]),
                },
            ),
        ]
    }

    #[test]
    fn credential_workflow_accepts_independently_authored_events() {
        let state = project_credential_events(&credential_events()).unwrap();
        assert_eq!(state.status, CredentialStatus::Issued);
        assert_eq!(state.applied_events.len(), 4);
    }

    #[test]
    fn credential_projection_is_independent_of_arrival_order() {
        let events = credential_events();
        let shuffled = vec![
            events[3].clone(),
            events[1].clone(),
            events[0].clone(),
            events[2].clone(),
        ];
        let a = project_credential_events(&events).unwrap();
        let b = project_credential_events(&shuffled).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn requester_cannot_author_issuer_decision() {
        let mut events = credential_events();
        events[2].author = actor("alice");
        assert!(matches!(
            project_credential_events(&events),
            Err(WorkflowError::AuthorMismatch("credential.approval.issuer"))
        ));
    }

    #[test]
    fn approval_must_bind_exact_request_digest() {
        let mut events = credential_events();
        if let CredentialEvent::Approved { request_digest, .. } = &mut events[2].payload {
            *request_digest = Digest32([8; 32]);
        }
        assert_eq!(
            project_credential_events(&events).unwrap_err(),
            WorkflowError::CredentialRequestDigestMismatch
        );
    }

    #[test]
    fn issuance_must_causally_depend_on_approval() {
        let mut events = credential_events();
        events[3].causal_parents = vec![event_id("e2")];
        assert_eq!(
            project_credential_events(&events).unwrap_err(),
            WorkflowError::IssuanceMissingApprovalCause
        );
    }

    #[test]
    fn unknown_parent_is_rejected() {
        let mut events = credential_events();
        events[1].causal_parents = vec![event_id("missing")];
        assert_eq!(
            project_credential_events(&events).unwrap_err(),
            WorkflowError::UnknownCausalParent
        );
    }

    #[test]
    fn timestamp_cannot_precede_parent() {
        let mut events = credential_events();
        events[1].occurred_at_ms = 50;
        assert_eq!(
            project_credential_events(&events).unwrap_err(),
            WorkflowError::CausalTimeRegression
        );
    }

    fn recovery_policy() -> RecoveryPolicy {
        RecoveryPolicy {
            subject: actor("alice"),
            trustees: vec![actor("t1"), actor("t2"), actor("t3")],
            threshold: 2,
            min_timelock_ms: 72 * 60 * 60 * 1000,
            executors: vec![actor("recovery-service")],
            policy_ref: PolicyRef::new("recovery-policy:alice:v1").unwrap(),
            policy_digest: Digest32([4; 32]),
        }
    }

    fn recovery_init(pid: &ProcessId, policy: &RecoveryPolicy) -> EventEnvelope<RecoveryEvent> {
        env(
            "r1",
            pid,
            actor("t1"),
            1_000,
            &[],
            RecoveryEvent::Initiated {
                subject: policy.subject.clone(),
                initiator: actor("t1"),
                new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                policy_ref: policy.policy_ref.clone(),
                policy_digest: policy.policy_digest,
            },
        )
    }

    fn recovery_ready_events() -> (RecoveryPolicy, Vec<EventEnvelope<RecoveryEvent>>, u64) {
        let pid = process("recovery-1");
        let policy = recovery_policy();
        let ready_at = 3_000 + policy.min_timelock_ms;
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("t1"),
                2_000,
                &["r1"],
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("t1"),
                    evidence_ref: None,
                },
            ),
            env(
                "r3",
                &pid,
                actor("t2"),
                3_000,
                &["r1"],
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("t2"),
                    evidence_ref: None,
                },
            ),
        ];
        (policy, events, ready_at)
    }

    #[test]
    fn recovery_requires_independently_authored_trustee_votes() {
        let (policy, events, _) = recovery_ready_events();
        let state = project_recovery_events(&policy, &events).unwrap();
        assert_eq!(state.status, RecoveryStatus::Timelocked);
        assert_eq!(state.approvals_count(), 2);
        assert_eq!(state.threshold_satisfied_at_ms, Some(3_000));
    }

    #[test]
    fn recovery_projection_is_independent_of_arrival_order() {
        let (policy, events, _) = recovery_ready_events();
        let shuffled = vec![events[2].clone(), events[0].clone(), events[1].clone()];
        assert_eq!(
            project_recovery_events(&policy, &events).unwrap(),
            project_recovery_events(&policy, &shuffled).unwrap()
        );
    }

    #[test]
    fn forged_trustee_vote_is_rejected() {
        let (policy, mut events, _) = recovery_ready_events();
        events[1].author = actor("mallory");
        assert!(matches!(
            project_recovery_events(&policy, &events),
            Err(WorkflowError::AuthorMismatch("recovery.approval.trustee"))
        ));
    }

    #[test]
    fn non_trustee_cannot_vote_even_as_self() {
        let pid = process("recovery-2");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("mallory"),
                2_000,
                &["r1"],
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("mallory"),
                    evidence_ref: None,
                },
            ),
        ];
        assert!(matches!(
            project_recovery_events(&policy, &events),
            Err(WorkflowError::UnauthorizedActor("recovery trustee"))
        ));
    }

    #[test]
    fn policy_digest_is_bound_at_initiation() {
        let pid = process("recovery-3");
        let policy = recovery_policy();
        let mut init = recovery_init(&pid, &policy);
        if let RecoveryEvent::Initiated { policy_digest, .. } = &mut init.payload {
            *policy_digest = Digest32([5; 32]);
        }
        assert_eq!(
            project_recovery_events(&policy, &[init]).unwrap_err(),
            WorkflowError::PolicyMismatch
        );
    }

    #[test]
    fn trustee_cannot_vote_twice() {
        let pid = process("recovery-4");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("t1"),
                2_000,
                &["r1"],
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("t1"),
                    evidence_ref: None,
                },
            ),
            env(
                "r3",
                &pid,
                actor("t1"),
                3_000,
                &["r2"],
                RecoveryEvent::TrusteeRejected {
                    trustee: actor("t1"),
                    reason_code: "changed-mind".into(),
                },
            ),
        ];
        assert_eq!(
            project_recovery_events(&policy, &events).unwrap_err(),
            WorkflowError::TrusteeAlreadyVoted
        );
    }

    #[test]
    fn unauthorized_executor_cannot_complete_recovery() {
        let (policy, mut events, ready_at) = recovery_ready_events();
        let pid = events[0].process_id.clone();
        events.push(env(
            "r4",
            &pid,
            actor("mallory"),
            ready_at,
            &["r2", "r3"],
            RecoveryEvent::Executed {
                executor: actor("mallory"),
                new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                execution_proof_ref: "proof:recovery-execution".into(),
            },
        ));
        assert!(matches!(
            project_recovery_events(&policy, &events),
            Err(WorkflowError::UnauthorizedActor("recovery executor"))
        ));
    }

    #[test]
    fn recovery_execution_fails_during_timelock() {
        let (policy, mut events, _) = recovery_ready_events();
        let pid = events[0].process_id.clone();
        events.push(env(
            "r4",
            &pid,
            actor("recovery-service"),
            4_000,
            &["r2", "r3"],
            RecoveryEvent::Executed {
                executor: actor("recovery-service"),
                new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                execution_proof_ref: "proof:recovery-execution".into(),
            },
        ));
        assert_eq!(
            project_recovery_events(&policy, &events).unwrap_err(),
            WorkflowError::RecoveryTimelockActive
        );
    }

    #[test]
    fn recovery_execution_must_causally_bind_threshold_votes() {
        let (policy, mut events, ready_at) = recovery_ready_events();
        let pid = events[0].process_id.clone();
        events.push(env(
            "r4",
            &pid,
            actor("recovery-service"),
            ready_at,
            &["r2"],
            RecoveryEvent::Executed {
                executor: actor("recovery-service"),
                new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                execution_proof_ref: "proof:recovery-execution".into(),
            },
        ));
        assert_eq!(
            project_recovery_events(&policy, &events).unwrap_err(),
            WorkflowError::ExecutionMissingThresholdCauses
        );
    }

    #[test]
    fn recovery_executes_after_threshold_timelock_and_causal_binding() {
        let (policy, mut events, ready_at) = recovery_ready_events();
        let pid = events[0].process_id.clone();
        events.push(env(
            "r4",
            &pid,
            actor("recovery-service"),
            ready_at,
            &["r2", "r3"],
            RecoveryEvent::Executed {
                executor: actor("recovery-service"),
                new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                execution_proof_ref: "proof:recovery-execution".into(),
            },
        ));
        let state = project_recovery_events(&policy, &events).unwrap();
        assert_eq!(state.status, RecoveryStatus::Completed);
    }

    #[test]
    fn subject_can_cancel_before_execution() {
        let pid = process("recovery-5");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                policy.subject.clone(),
                2_000,
                &["r1"],
                RecoveryEvent::SubjectCancelled {
                    subject: policy.subject.clone(),
                    reason_code: "account-recovered-locally".into(),
                },
            ),
        ];
        let state = project_recovery_events(&policy, &events).unwrap();
        assert_eq!(state.status, RecoveryStatus::Cancelled);
    }

    #[test]
    fn event_cannot_reference_itself_as_parent() {
        let pid = process("causal-1");
        let spec = credential_spec();
        let mut event = env(
            "self-parent",
            &pid,
            spec.requester.clone(),
            1,
            &[],
            CredentialEvent::Requested { spec },
        );
        event.causal_parents.push(event.id.clone());
        assert_eq!(
            event.validate_envelope().unwrap_err(),
            WorkflowError::SelfCausalReference
        );
    }

    #[test]
    fn wire_round_trip_preserves_event_author_and_causality() {
        let event = credential_events().remove(1);
        let json = serde_json::to_string(&event).unwrap();
        let decoded: EventEnvelope<CredentialEvent> = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, event);
    }
}

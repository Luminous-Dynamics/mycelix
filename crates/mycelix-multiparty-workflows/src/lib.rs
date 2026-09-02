// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Append-only event models for workflows involving independent actors.
//!
//! The central rule is simple:
//!
//! > An actor never mutates another actor's record to express their decision.
//!
//! Requesters author requests and withdrawals. Issuers author review/decision/
//! issuance events. Trustees author their own recovery approvals/rejections.
//! Current workflow state is a deterministic projection over valid append-only
//! events supplied by the persistence layer.
//!
//! This crate is intentionally Holochain-neutral. A Holochain adapter should
//! bind `EventEnvelope.author` to the committing `AgentPubKey` in integrity
//! validation and store each envelope as an immutable entry.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-multiparty-workflows-v0.1";
const MAX_ID_BYTES: usize = 512;
const MAX_TEXT_BYTES: usize = 4096;

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

/// Immutable event wrapper. Persistence adapters should reject update/delete
/// operations and cryptographically/structurally bind `author` to the committer.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EventEnvelope<E> {
    pub protocol_version: String,
    pub id: EventId,
    pub process_id: ProcessId,
    pub author: ActorId,
    pub occurred_at_ms: u64,
    /// Causal dependencies. This permits independent actors to append without a
    /// shared mutable sequence counter. Hosts must reject self-parenting and
    /// verify referenced parents belong to the same process.
    pub causal_parents: Vec<EventId>,
    pub payload: E,
    /// Signature/attestation reference verified by the host.
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

// =============================================================================
// Credential request workflow
// =============================================================================

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialRequestSpec {
    pub requester: ActorId,
    pub subject: ActorId,
    pub issuer: ActorId,
    pub schema_id: String,
    /// Digest of the exact claims/evidence package requested/initially supplied.
    pub request_digest: Digest32,
}

impl CredentialRequestSpec {
    pub fn validate(&self) -> Result<(), WorkflowError> {
        validate_text(self.requester.as_str(), "credential.requester", MAX_ID_BYTES)?;
        validate_text(self.subject.as_str(), "credential.subject", MAX_ID_BYTES)?;
        validate_text(self.issuer.as_str(), "credential.issuer", MAX_ID_BYTES)?;
        validate_text(&self.schema_id, "credential.schema_id", MAX_ID_BYTES)?;
        require_digest(self.request_digest, "credential.request_digest")
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
    pub spec: CredentialRequestSpec,
    pub status: CredentialStatus,
    pub evidence: Vec<EvidenceRef>,
    pub decision_ref: Option<String>,
    pub credential_ref: Option<CredentialRef>,
    pub applied_events: Vec<EventId>,
}

pub fn project_credential_events(
    events: &[EventEnvelope<CredentialEvent>],
) -> Result<CredentialProjection, WorkflowError> {
    let first = events.first().ok_or(WorkflowError::MissingInitialEvent)?;
    first.validate_envelope()?;
    let spec = match &first.payload {
        CredentialEvent::Requested { spec } => {
            spec.validate()?;
            if first.author != spec.requester {
                return Err(WorkflowError::AuthorMismatch("credential.requester"));
            }
            spec.clone()
        }
        _ => return Err(WorkflowError::InitialEventMustBeRequest),
    };

    let process_id = first.process_id.clone();
    let mut projection = CredentialProjection {
        process_id: process_id.clone(),
        spec,
        status: CredentialStatus::Pending,
        evidence: Vec::new(),
        decision_ref: None,
        credential_ref: None,
        applied_events: vec![first.id.clone()],
    };
    let mut seen = BTreeSet::new();
    seen.insert(first.id.clone());

    for event in &events[1..] {
        event.validate_envelope()?;
        if event.process_id != process_id {
            return Err(WorkflowError::ProcessMismatch);
        }
        if !seen.insert(event.id.clone()) {
            return Err(WorkflowError::DuplicateEvent);
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
            decision_ref,
        } => {
            require_actor(&event.author, issuer, "credential.approval.issuer")?;
            require_actor(issuer, &state.spec.issuer, "credential.expected_issuer")?;
            require_status(
                state.status,
                &[CredentialStatus::Pending, CredentialStatus::UnderReview],
                "approved",
            )?;
            validate_text(decision_ref, "credential.decision_ref", MAX_ID_BYTES)?;
            state.status = CredentialStatus::Approved;
            state.decision_ref = Some(decision_ref.clone());
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
        } => {
            require_actor(&event.author, issuer, "credential.issuance.issuer")?;
            require_actor(issuer, &state.spec.issuer, "credential.expected_issuer")?;
            require_status(
                state.status,
                &[CredentialStatus::Approved],
                "credential_issued",
            )?;
            state.status = CredentialStatus::Issued;
            state.credential_ref = Some(credential_ref.clone());
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
    /// Minimum delay after the threshold becomes satisfied.
    pub min_timelock_ms: u64,
    pub policy_ref: PolicyRef,
}

impl RecoveryPolicy {
    pub fn validate(&self) -> Result<(), WorkflowError> {
        validate_text(self.subject.as_str(), "recovery.subject", MAX_ID_BYTES)?;
        validate_text(self.policy_ref.as_str(), "recovery.policy_ref", MAX_ID_BYTES)?;
        if self.trustees.len() < 3 {
            return Err(WorkflowError::TooFewTrustees);
        }
        let unique: BTreeSet<&ActorId> = self.trustees.iter().collect();
        if unique.len() != self.trustees.len() {
            return Err(WorkflowError::DuplicateTrustee);
        }
        if self.trustees.iter().any(|trustee| trustee == &self.subject) {
            return Err(WorkflowError::SubjectCannotBeTrustee);
        }
        if self.threshold == 0 || self.threshold as usize > self.trustees.len() {
            return Err(WorkflowError::InvalidThreshold);
        }
        // At least a majority; deployments may choose stronger thresholds.
        let majority = (self.trustees.len() as u32 / 2) + 1;
        if self.threshold < majority {
            return Err(WorkflowError::ThresholdBelowMajority);
        }
        if self.min_timelock_ms == 0 {
            return Err(WorkflowError::ZeroRecoveryTimelock);
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
    pub policy: RecoveryPolicy,
    pub initiator: ActorId,
    pub new_key_ref: KeyRef,
    pub status: RecoveryStatus,
    pub approvals: BTreeMap<ActorId, u64>,
    pub rejections: BTreeMap<ActorId, u64>,
    pub threshold_satisfied_at_ms: Option<u64>,
    pub executable_at_ms: Option<u64>,
    pub applied_events: Vec<EventId>,
}

impl RecoveryProjection {
    pub fn effective_status_at(&self, now_ms: u64) -> RecoveryStatus {
        if self.status == RecoveryStatus::Timelocked {
            if self.executable_at_ms.is_some_and(|at| now_ms >= at) {
                return RecoveryStatus::Ready;
            }
        }
        self.status
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
    let first = events.first().ok_or(WorkflowError::MissingInitialEvent)?;
    first.validate_envelope()?;

    let (subject, initiator, new_key_ref, policy_ref) = match &first.payload {
        RecoveryEvent::Initiated {
            subject,
            initiator,
            new_key_ref,
            policy_ref,
        } => (subject, initiator, new_key_ref, policy_ref),
        _ => return Err(WorkflowError::InitialEventMustBeRecoveryInitiation),
    };
    require_actor(&first.author, initiator, "recovery.initiator")?;
    require_actor(subject, &policy.subject, "recovery.subject")?;
    if policy_ref != &policy.policy_ref {
        return Err(WorkflowError::PolicyMismatch);
    }
    if initiator != subject && !policy.trustees.contains(initiator) {
        return Err(WorkflowError::UnauthorizedActor("recovery initiator"));
    }

    let process_id = first.process_id.clone();
    let mut projection = RecoveryProjection {
        process_id: process_id.clone(),
        policy: policy.clone(),
        initiator: initiator.clone(),
        new_key_ref: new_key_ref.clone(),
        status: RecoveryStatus::Pending,
        approvals: BTreeMap::new(),
        rejections: BTreeMap::new(),
        threshold_satisfied_at_ms: None,
        executable_at_ms: None,
        applied_events: vec![first.id.clone()],
    };
    let mut seen = BTreeSet::new();
    seen.insert(first.id.clone());

    for event in &events[1..] {
        event.validate_envelope()?;
        if event.process_id != process_id {
            return Err(WorkflowError::ProcessMismatch);
        }
        if !seen.insert(event.id.clone()) {
            return Err(WorkflowError::DuplicateEvent);
        }
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
            state.approvals.insert(trustee.clone(), event.occurred_at_ms);
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
            state.rejections.insert(trustee.clone(), event.occurred_at_ms);

            let remaining = state.policy.trustees.len()
                - state.approvals.len()
                - state.rejections.len();
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
            state.status = RecoveryStatus::Completed;
            Ok(())
        }
    }
}

fn recompute_recovery_threshold(state: &mut RecoveryProjection) -> Result<(), WorkflowError> {
    if state.approvals.len() < state.policy.threshold as usize {
        return Ok(());
    }

    // The threshold is satisfied when the Nth approval needed by policy occurs.
    // Since a projection is deterministic over its supplied event order and each
    // trustee can approve only once, the maximum timestamp among the approvals
    // present at first satisfaction is the conservative start of the timelock.
    let satisfied_at = state
        .approvals
        .values()
        .copied()
        .max()
        .ok_or(WorkflowError::RecoveryThresholdNotSatisfied)?;
    let executable_at = satisfied_at
        .checked_add(state.policy.min_timelock_ms)
        .ok_or(WorkflowError::TimeOverflow)?;

    if state.threshold_satisfied_at_ms.is_none() {
        state.threshold_satisfied_at_ms = Some(satisfied_at);
        state.executable_at_ms = Some(executable_at);
        state.status = RecoveryStatus::Timelocked;
    }
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
    MissingInitialEvent,
    InitialEventMustBeRequest,
    InitialEventMustBeRecoveryInitiation,
    DuplicateInitialEvent,
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
    TooFewTrustees,
    DuplicateTrustee,
    SubjectCannotBeTrustee,
    InvalidThreshold,
    ThresholdBelowMajority,
    ZeroRecoveryTimelock,
    TrusteeAlreadyVoted,
    PolicyMismatch,
    RecoveryTargetChanged,
    RecoveryThresholdNotSatisfied,
    RecoveryTimelockActive,
    TimeOverflow,
}

impl fmt::Display for WorkflowError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::MissingInitialEvent => write!(f, "workflow has no initial event"),
            Self::InitialEventMustBeRequest => write!(f, "first credential event must be Requested"),
            Self::InitialEventMustBeRecoveryInitiation => {
                write!(f, "first recovery event must be Initiated")
            }
            Self::DuplicateInitialEvent => write!(f, "workflow contains a second initial event"),
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
            Self::TooFewTrustees => write!(f, "recovery requires at least three trustees"),
            Self::DuplicateTrustee => write!(f, "recovery trustees must be unique"),
            Self::SubjectCannotBeTrustee => write!(f, "recovery subject cannot also be a trustee"),
            Self::InvalidThreshold => write!(f, "invalid recovery threshold"),
            Self::ThresholdBelowMajority => write!(f, "recovery threshold must be at least a majority"),
            Self::ZeroRecoveryTimelock => write!(f, "recovery timelock must be positive"),
            Self::TrusteeAlreadyVoted => write!(f, "trustee has already approved or rejected"),
            Self::PolicyMismatch => write!(f, "recovery event references a different policy"),
            Self::RecoveryTargetChanged => write!(f, "recovery target key changed after initiation"),
            Self::RecoveryThresholdNotSatisfied => write!(f, "recovery approval threshold not satisfied"),
            Self::RecoveryTimelockActive => write!(f, "recovery timelock is still active"),
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
        payload: E,
    ) -> EventEnvelope<E> {
        EventEnvelope {
            protocol_version: PROTOCOL_VERSION.into(),
            id: event_id(id),
            process_id: process_id.clone(),
            author,
            occurred_at_ms: at,
            causal_parents: vec![],
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
            request_digest: Digest32([7; 32]),
        }
    }

    #[test]
    fn credential_workflow_accepts_independently_authored_events() {
        let pid = process("credential-1");
        let spec = credential_spec();
        let events = vec![
            env(
                "e1",
                &pid,
                spec.requester.clone(),
                100,
                CredentialEvent::Requested { spec: spec.clone() },
            ),
            env(
                "e2",
                &pid,
                spec.issuer.clone(),
                200,
                CredentialEvent::ReviewStarted {
                    issuer: spec.issuer.clone(),
                },
            ),
            env(
                "e3",
                &pid,
                spec.issuer.clone(),
                300,
                CredentialEvent::Approved {
                    issuer: spec.issuer.clone(),
                    decision_ref: "decision:degree-approved".into(),
                },
            ),
            env(
                "e4",
                &pid,
                spec.issuer.clone(),
                400,
                CredentialEvent::CredentialIssued {
                    issuer: spec.issuer.clone(),
                    credential_ref: CredentialRef::new("vc:degree:alice").unwrap(),
                },
            ),
        ];

        let state = project_credential_events(&events).unwrap();
        assert_eq!(state.status, CredentialStatus::Issued);
        assert_eq!(state.applied_events.len(), 4);
    }

    #[test]
    fn requester_cannot_author_issuer_decision() {
        let pid = process("credential-2");
        let spec = credential_spec();
        let events = vec![
            env(
                "e1",
                &pid,
                spec.requester.clone(),
                100,
                CredentialEvent::Requested { spec: spec.clone() },
            ),
            env(
                "e2",
                &pid,
                spec.requester.clone(),
                200,
                CredentialEvent::Approved {
                    issuer: spec.issuer.clone(),
                    decision_ref: "decision:forged".into(),
                },
            ),
        ];
        assert!(matches!(
            project_credential_events(&events),
            Err(WorkflowError::AuthorMismatch("credential.approval.issuer"))
        ));
    }

    #[test]
    fn issuer_does_not_mutate_requester_record_to_review() {
        let pid = process("credential-3");
        let spec = credential_spec();
        let requested = env(
            "request-by-alice",
            &pid,
            spec.requester.clone(),
            100,
            CredentialEvent::Requested { spec: spec.clone() },
        );
        let reviewed = env(
            "review-by-university",
            &pid,
            spec.issuer.clone(),
            200,
            CredentialEvent::ReviewStarted {
                issuer: spec.issuer.clone(),
            },
        );
        assert_ne!(requested.author, reviewed.author);
        assert_eq!(
            project_credential_events(&[requested, reviewed])
                .unwrap()
                .status,
            CredentialStatus::UnderReview
        );
    }

    fn recovery_policy() -> RecoveryPolicy {
        RecoveryPolicy {
            subject: actor("alice"),
            trustees: vec![actor("t1"), actor("t2"), actor("t3")],
            threshold: 2,
            min_timelock_ms: 72 * 60 * 60 * 1000,
            policy_ref: PolicyRef::new("recovery-policy:alice:v1").unwrap(),
        }
    }

    fn recovery_init(pid: &ProcessId, policy: &RecoveryPolicy) -> EventEnvelope<RecoveryEvent> {
        env(
            "r1",
            pid,
            actor("t1"),
            1_000,
            RecoveryEvent::Initiated {
                subject: policy.subject.clone(),
                initiator: actor("t1"),
                new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                policy_ref: policy.policy_ref.clone(),
            },
        )
    }

    #[test]
    fn recovery_requires_independently_authored_trustee_votes() {
        let pid = process("recovery-1");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("t1"),
                2_000,
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
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("t2"),
                    evidence_ref: None,
                },
            ),
        ];
        let state = project_recovery_events(&policy, &events).unwrap();
        assert_eq!(state.status, RecoveryStatus::Timelocked);
        assert_eq!(state.approvals_count(), 2);
        assert_eq!(state.threshold_satisfied_at_ms, Some(3_000));
    }

    #[test]
    fn forged_trustee_vote_is_rejected() {
        let pid = process("recovery-2");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("mallory"),
                2_000,
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("t2"),
                    evidence_ref: None,
                },
            ),
        ];
        assert!(matches!(
            project_recovery_events(&policy, &events),
            Err(WorkflowError::AuthorMismatch("recovery.approval.trustee"))
        ));
    }

    #[test]
    fn non_trustee_cannot_vote_even_as_self() {
        let pid = process("recovery-3");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("mallory"),
                2_000,
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
    fn recovery_execution_fails_during_timelock() {
        let pid = process("recovery-5");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("t1"),
                2_000,
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
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("t2"),
                    evidence_ref: None,
                },
            ),
            env(
                "r4",
                &pid,
                actor("executor"),
                4_000,
                RecoveryEvent::Executed {
                    executor: actor("executor"),
                    new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                    execution_proof_ref: "proof:recovery-execution".into(),
                },
            ),
        ];
        assert_eq!(
            project_recovery_events(&policy, &events).unwrap_err(),
            WorkflowError::RecoveryTimelockActive
        );
    }

    #[test]
    fn recovery_executes_after_threshold_and_timelock() {
        let pid = process("recovery-6");
        let policy = recovery_policy();
        let ready_at = 3_000 + policy.min_timelock_ms;
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                actor("t1"),
                2_000,
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
                RecoveryEvent::TrusteeApproved {
                    trustee: actor("t2"),
                    evidence_ref: None,
                },
            ),
            env(
                "r4",
                &pid,
                actor("executor"),
                ready_at,
                RecoveryEvent::Executed {
                    executor: actor("executor"),
                    new_key_ref: KeyRef::new("key:alice:new").unwrap(),
                    execution_proof_ref: "proof:recovery-execution".into(),
                },
            ),
        ];
        let state = project_recovery_events(&policy, &events).unwrap();
        assert_eq!(state.status, RecoveryStatus::Completed);
    }

    #[test]
    fn subject_can_cancel_before_execution() {
        let pid = process("recovery-7");
        let policy = recovery_policy();
        let events = vec![
            recovery_init(&pid, &policy),
            env(
                "r2",
                &pid,
                policy.subject.clone(),
                2_000,
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
        let mut event = env(
            "self-parent",
            &pid,
            actor("alice"),
            1,
            CredentialEvent::Requested {
                spec: credential_spec(),
            },
        );
        event.causal_parents.push(event.id.clone());
        assert_eq!(
            event.validate_envelope().unwrap_err(),
            WorkflowError::SelfCausalReference
        );
    }

    #[test]
    fn wire_round_trip_preserves_event_author() {
        let pid = process("wire-1");
        let spec = credential_spec();
        let event = env(
            "e1",
            &pid,
            spec.requester.clone(),
            100,
            CredentialEvent::Requested { spec },
        );
        let json = serde_json::to_string(&event).unwrap();
        let decoded: EventEnvelope<CredentialEvent> = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, event);
    }
}
